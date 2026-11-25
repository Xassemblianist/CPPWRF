// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/backend.hpp"
#include "core/column_work.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#if defined(CPPWRF_USE_GPU)
#define CPPWRF_GPU_RUNTIME_HOST 1
#include "core/gpu_runtime.hpp"
#include <cxxabi.h>
#endif

namespace cppwrf {

#if defined(CPPWRF_USE_GPU)
static void ck(gpuError_t e, const char* what) {
  if (e != gpuSuccess)
    throw std::runtime_error(std::string(CPPWRF_GPU_API_NAME " error in ") + what + ": " + gpuGetErrorString(e));
}
namespace ktime {
namespace {
struct Table {
  std::map<std::string, std::pair<double, long>> t;  // kernel type -> (seconds, launches)
  ~Table() {
    if (t.empty()) return;
    std::vector<std::pair<std::string, std::pair<double, long>>> v(t.begin(), t.end());
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second.first > b.second.first; });
    double tot = 0;
    for (auto& e : v) tot += e.second.first;
    std::fprintf(stderr, "cppwrf ktime: %.3f s in kernels\n", tot);
    for (auto& e : v) {
      int st = 0;
      char* dm = abi::__cxa_demangle(e.first.c_str(), nullptr, nullptr, &st);
      std::string name = (st == 0 && dm) ? dm : e.first;
      std::free(dm);
      if (name.size() > 110) name = name.substr(0, 110);
      std::fprintf(stderr, "%6.2f%% %9.2f ms %7ld  %s\n", 100 * e.second.first / tot, 1e3 * e.second.first,
                   e.second.second, name.c_str());
    }
  }
};
Table& table() { static Table tb; return tb; }
std::mutex table_m;  // the tile threads of run.tiles share the table
thread_local std::chrono::steady_clock::time_point t0;  // each its own launch
}  // namespace
bool on() {
  static const bool enabled = [] { const char* e = std::getenv("CPPWRF_KTIME"); return e && *e && *e != '0'; }();
  return enabled;
}
void begin() {
  ck(gpuDeviceSynchronize(), "ktime");
  t0 = std::chrono::steady_clock::now();
}
void end(const char* kernel_type) {
  ck(gpuDeviceSynchronize(), "ktime");
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::lock_guard<std::mutex> lk(table_m);
  auto& e = table().t[kernel_type];
  e.first += dt;
  e.second += 1;
}
}  // namespace ktime

void gpu_check_launch(const char* where) {
  ck(gpuGetLastError(), where);
#if defined(CPPWRF_CUDA_SYNC_CHECK) || defined(CPPWRF_GPU_SYNC_CHECK)
  ck(gpuDeviceSynchronize(), where);
#endif
}
// one per host thread: the tiles of a tiled run (src/core/tiles.cpp) each launch on their own stream; an ensemble
// member's thread destroys its own at its end (release_thread_device)
namespace {
thread_local gpuStream_t t_stream = nullptr;
}
void* gpu_stream_raw() {
  if (!t_stream) ck(gpuStreamCreate(&t_stream), "gpuStreamCreate");  // blocking: orders with any work on the legacy stream 0
  return t_stream;
}
static gpuStream_t stream() { return static_cast<gpuStream_t>(gpu_stream_raw()); }

// ---- time step graphs (StepScope) and the fatal flags of the step ----
namespace {
int graphs_mode() {  // 0 off, 1 on, 2 on with the counts at exit
#if !defined(CPPWRF_USE_CUDA)
  // HIP graph capture/update (4-arg hipGraphExecUpdate, 5-arg hipGraphInstantiate, and stream capture of
  // hipMemcpyFromSymbolAsync) is deferred until verified on AMD hardware.
  return 0;
#else
  static const int m = [] {
#if defined(CPPWRF_CUDA_SYNC_CHECK) || defined(CPPWRF_GPU_SYNC_CHECK)
    return 0;  // a device synchronisation after every launch cannot be captured
#endif
    if (ktime::on()) return 0;
    // tiles (several workers in one process): off. An MPI rank (one worker per process) captures the segments
    // between its halo exchanges and fatal checks, which wait for its stream outside the capture
    // (host_section_begin / host_section_end)
    if (tile::hooks.multi) return 0;
    const char* e = std::getenv("CPPWRF_GRAPHS");
    if (!e || !*e) return 1;
    if (std::strcmp(e, "stats") == 0) return 2;
    return *e == '0' ? 0 : 1;
  }();
  return m;
#endif
}
struct Poll {
  int* slot;
  void (*reset)();
};
constexpr int kSlots = 256;  // fatal checks per segment
struct Graphs {
  bool in_step = false, capturing = false;
  const void* key = nullptr;
  int seg = 0;
#if defined(CPPWRF_USE_CUDA)
  std::map<std::pair<const void*, int>, std::vector<cudaGraphExec_t>> cache;  // most recently used first
#endif
  int* slots = nullptr;  // pinned, 2 banks of kSlots
  int bank = 0;
  // the end marker of a segment with checks: 1 copied from the device behind its work (a pinned word per bank).
  // A device error raised after the checked work ended (the next segment runs before the flags are read) leaves
  // the marker set: the checked segment's flags are then read and reported first, as when they were read before
  // the next launch.
  int* done = nullptr;           // pinned, 2 (after the slots)
  const int* done_one = nullptr;  // device, 1
  int pend_done = -1;             // the marker of the checks in pend (-1: none)
  std::vector<Poll> cur, pend;  // checks of the open segment; of the launched, unchecked ones
  gpuEvent_t ev = nullptr;
  bool ev_pending = false;
  gpuEvent_t mark = nullptr;  // host_mark's marker
  long n_launch = 0, n_update = 0, n_inst = 0, n_seg_nodes = 0;
  void init() {
    if (slots) return;
    ck(gpuHostAlloc(reinterpret_cast<void**>(&slots), (2 * kSlots + 2) * sizeof(int), gpuHostAllocDefault),
       "gpuHostAlloc");
    done = slots + 2 * kSlots;
    ck(gpuEventCreateWithFlags(&ev, gpuEventDisableTiming), "gpuEventCreate");
    ck(gpuEventCreateWithFlags(&mark, gpuEventDisableTiming), "gpuEventCreate");
  }
  void begin_capture() {
#if defined(CPPWRF_USE_CUDA)
    ck(cudaStreamBeginCapture(stream(), cudaStreamCaptureModeThreadLocal), "cudaStreamBeginCapture");
    capturing = true;
#endif
  }
#if defined(CPPWRF_USE_CUDA)
  // the instantiated graph of this domain and segment, updated to g's launch parameters (a step of another
  // kind has another topology: up to 4 per segment are kept)
  cudaGraphExec_t exec_for(cudaGraph_t g) {
    auto& v = cache[{key, seg}];
    for (std::size_t i = 0; i < v.size(); ++i) {
      cudaGraphExecUpdateResultInfo info;
      if (cudaGraphExecUpdate(v[i], g, &info) == cudaSuccess) {
        ++n_update;
        std::rotate(v.begin(), v.begin() + long(i), v.begin() + long(i) + 1);
        return v[0];
      }
      (void)cudaGetLastError();  // a failed update is no launch error
    }
    cudaGraphExec_t e = nullptr;
    ck(cudaGraphInstantiate(&e, g, 0), "cudaGraphInstantiate");
    ++n_inst;
    if (v.size() >= 4) {
      cudaGraphExecDestroy(v.back());
      v.pop_back();
    }
    v.insert(v.begin(), e);
    return e;
  }
#endif
  // ends the open segment: launches its graph, then reads the previous segment's flags (waiting for it: the device
  // has this one queued behind it and runs on without a pause), and records the event that this segment's fatal
  // checks wait for. The launches run in the same order on the one stream as before. A raised flag stops the run
  // with this segment queued: its own flags are never read (as when it was not launched), nothing waits for it.
  void segment_point() {
    bool launched = false, ended = false;
#if defined(CPPWRF_USE_CUDA)
    if (capturing) {
      cudaGraph_t g = nullptr;
      capturing = false;
      ended = true;
      ck(cudaStreamEndCapture(stream(), &g), "cudaStreamEndCapture");
      std::size_t nn = 0;
      ck(cudaGraphGetNodes(g, nullptr, &nn), "cudaGraphGetNodes");
      cudaGraphExec_t e = nn ? exec_for(g) : nullptr;
      cudaGraphDestroy(g);
      if (e) {
        if (const cudaError_t le = cudaGraphLaunch(e, stream()); le != cudaSuccess) {
          (void)cudaGetLastError();
          flush();  // the previous segment's raised flag is the stop reported, as when it was read before
          ck(le, "cudaGraphLaunch");
        }
        launched = true;
        ++n_launch;
        n_seg_nodes += long(nn);
      }
      ++seg;
    }
#endif
    if (launched || !cur.empty()) {  // at most one step or segment in flight unchecked
      std::vector<Poll> next;  // this segment's checks (slots of this bank)
      next.swap(cur);
      bank ^= 1;
      flush();  // the previous step or segment: done, its flags read (the other bank)
      pend.swap(next);
      cur.swap(next);  // the read checks' vector (empty) keeps its storage
      pend_done = -1;
      if (!pend.empty()) {  // the bank of these checks (its previous marker was read: that segment was waited for)
        if (!done_one) {  // a segment with checks: not in a decomposed run (its checks are read at once)
          int* d = nullptr;
          const int one = 1;
          ck(gpuMalloc(reinterpret_cast<void**>(&d), sizeof(int)), "gpuMalloc");
          // on the thread's stream, not the legacy stream 0: another host thread (an ensemble member) may be
          // capturing its blocking stream, which a legacy-stream copy would join
          ck(gpuMemcpyAsync(d, &one, sizeof(int), gpuMemcpyHostToDevice, stream()), "gpuMemcpyAsync");
          ck(gpuStreamSynchronize(stream()), "gpuStreamSynchronize");
          done_one = d;
        }
        const int b = bank ^ 1;
        done[b] = 0;
        ck(gpuMemcpyAsync(done + b, done_one, sizeof(int), gpuMemcpyDeviceToHost, stream()), "gpuMemcpyAsync");
        pend_done = b;
      }
      ck(gpuEventRecord(ev, stream()), "gpuEventRecord");
      ev_pending = true;
    } else if (ended) {
      flush();
    }
  }
  // reads the flags of the launched checks in the order of the calls; the first raised code stops the run
  void flush() {
    if (!ev_pending) return;
    const gpuError_t se = gpuEventSynchronize(ev);
    ev_pending = false;
    if (se != gpuSuccess && !(pend_done >= 0 && done[pend_done] == 1)) {  // the checked work did not end
      pend.clear();
      ck(se, "gpuEventSynchronize");
    }
    int code = 0;
    for (const Poll& p : pend)
      if (*p.slot != 0) {
        if (code == 0) code = *p.slot;
        p.reset();
      }
    pend.clear();
    if (code != 0) fatal_stop(code);
    ck(se, "gpuEventSynchronize");  // an error of later work: after the checked segment's own stop
  }
  void abort() {  // an exception inside the step
#if defined(CPPWRF_USE_CUDA)
    if (capturing) {
      cudaGraph_t g = nullptr;
      cudaStreamEndCapture(stream(), &g);
      if (g) cudaGraphDestroy(g);
      (void)cudaGetLastError();
      capturing = false;
    }
#endif
    cur.clear();
    in_step = false;
  }
};
// one per host thread, never destroyed at exit (device memory may be freed by static objects then); an ensemble
// member's thread releases its own (release_thread_device)
thread_local Graphs* t_graphs = nullptr;
Graphs& graphs() {
  if (!t_graphs) t_graphs = new Graphs;
  return *t_graphs;
}
struct GraphStats {
  ~GraphStats() {
    const Graphs& g = graphs();
    if (graphs_mode() == 2 && g.n_launch > 0)
      std::fprintf(stderr, "cppwrf graphs: %ld launches, %ld updates, %ld instantiations, %.1f nodes per launch\n",
                   g.n_launch, g.n_update, g.n_inst, double(g.n_seg_nodes) / double(g.n_launch));
  }
} graph_stats;
// an operation that waits for the device: the open segment is launched first, the capture resumes after it
void sync_begin() { graphs().segment_point(); }
void sync_end() {
  Graphs& gs = graphs();
  gs.flush();
  if (gs.in_step && graphs_mode() != 0 && !gs.capturing) gs.begin_capture();
}
}  // namespace

bool step_graphs_on() { return graphs_mode() != 0; }

StepScope::StepScope(const void* key) {
  Graphs& gs = graphs();
  if (gs.in_step) throw std::logic_error("StepScope: a step inside a step");
  gs.init();
  gs.in_step = true;
  gs.key = key;
  gs.seg = 0;
  open_ = true;
  if (graphs_mode() != 0) gs.begin_capture();
}
StepScope::~StepScope() {
  if (open_) graphs().abort();
}
void StepScope::end() {
  Graphs& gs = graphs();
  open_ = false;
  gs.in_step = false;
  gs.segment_point();
}

namespace detail {
int* fatal_poll_slot(void (*reset)()) {
  Graphs& gs = graphs();
  // a decomposed run reads its flags at once: the smallest code over the workers (tile::hooks.first_code) is a
  // collective of every check
  if (!gs.in_step || tile::hooks.first_code) return nullptr;
  // a stop raised by host code (a kernel run on the host): the checks before this one come first, as before
  if (int hc = __atomic_exchange_n(&fatal_code_host[fatal_slot], 0, __ATOMIC_RELAXED)) {
    sync_begin();
    gs.flush();
    fatal_stop(hc);
  }
  if (gs.cur.size() >= std::size_t(kSlots)) throw std::runtime_error("fatal_check: too many checks in one step");
  int* slot = gs.slots + gs.bank * kSlots + int(gs.cur.size());
  *slot = 0;  // a copy that fails (fatal_check throws) leaves no earlier value to be read
  gs.cur.push_back({slot, reset});
  return slot;
}
void fatal_flush() { graphs().flush(); }
void fatal_transfer_failed(int err, const char* what) { ck(gpuError_t(err), what); }
}  // namespace detail

void discard_pending_checks() noexcept {
  if (!t_graphs) return;
  Graphs& gs = *t_graphs;
#if defined(CPPWRF_USE_CUDA)
  if (gs.capturing) {
    cudaGraph_t g = nullptr;
    cudaStreamEndCapture(stream(), &g);
    if (g) cudaGraphDestroy(g);
    gs.capturing = false;
  }
#endif
  if (gs.ev_pending) (void)gpuEventSynchronize(gs.ev);
  (void)gpuGetLastError();
  gs.ev_pending = false;
  gs.pend.clear();
  gs.cur.clear();
  gs.pend_done = -1;
  gs.in_step = false;
}

void release_thread_device() noexcept {
  discard_pending_checks();
  if (t_stream) (void)gpuStreamSynchronize(t_stream);
  column_work_release();  // (dev_free: before the graphs go, which it would make anew)
  if (Graphs* gs = t_graphs) {
#if defined(CPPWRF_USE_CUDA)
    for (auto& [k, v] : gs->cache)
      for (cudaGraphExec_t e : v) cudaGraphExecDestroy(e);
#endif
    if (gs->ev) (void)gpuEventDestroy(gs->ev);
    if (gs->mark) (void)gpuEventDestroy(gs->mark);
    if (gs->slots) (void)gpuFreeHost(gs->slots);
    if (gs->done_one) (void)gpuFree(const_cast<int*>(gs->done_one));
    delete gs;
    t_graphs = nullptr;
  }
  if (t_stream) {
    (void)gpuStreamDestroy(t_stream);
    t_stream = nullptr;
  }
  (void)gpuGetLastError();
}
namespace detail {

void reserve_local_memory(const void* kernel, const char* kernel_type) {
#if defined(CPPWRF_USE_CUDA)
  gpuFuncAttributes a{};
  ck(gpuFuncGetAttributes(&a, kernel), "cudaFuncGetAttributes");
  static std::mutex m;
  static std::map<int, std::size_t> reserved_on;  // per device: the largest frame of the process's launches
  int cur = 0;
  ck(gpuGetDevice(&cur), "cudaGetDevice");
  std::lock_guard<std::mutex> lk(m);
  std::size_t& reserved = reserved_on[cur];
  if (a.localSizeBytes <= reserved) return;
  std::size_t limit = 0;
  ck(gpuDeviceGetLimit(&limit, gpuLimitStackSize), "cudaDeviceGetLimit");
  if (a.localSizeBytes <= limit) {
    reserved = a.localSizeBytes;
    return;
  }
  int dev = 0, sms = 0, tpsm = 0;
  ck(gpuGetDevice(&dev), "cudaGetDevice");
  ck(gpuDeviceGetAttribute(&sms, gpuDevAttrMultiProcessorCount, dev), "cudaDeviceGetAttribute");
  ck(gpuDeviceGetAttribute(&tpsm, gpuDevAttrMaxThreadsPerMultiProcessor, dev), "cudaDeviceGetAttribute");
  sync_begin();  // inside a step: the captured part runs first (as for an allocation)
  std::size_t free_b = 0, total_b = 0;
  const gpuError_t mi = gpuMemGetInfo(&free_b, &total_b);
  const gpuError_t e = gpuDeviceSetLimit(gpuLimitStackSize, a.localSizeBytes);
  if (e != gpuSuccess) (void)gpuGetLastError();
  sync_end();
  if (e != gpuSuccess) {
    int st = 0;
    char* dm = abi::__cxa_demangle(kernel_type, nullptr, nullptr, &st);
    const std::string name = (st == 0 && dm) ? dm : kernel_type;
    std::free(dm);
    const double mib = 1024. * 1024.;
    char msg[640];
    std::snprintf(msg, sizeof msg,
                  "device memory too small: the kernel %.200s keeps %zu bytes per thread in local memory, which the GPU "
                  "reserves for each of its %d x %d resident threads (%.0f MiB in all, %.0f MiB of it reserved so far); %.0f MiB of "
                  "%.0f MiB are free (%s). Use a smaller domain, fewer levels or a GPU with more memory.",
                  name.c_str(), std::size_t(a.localSizeBytes), sms, tpsm,
                  double(a.localSizeBytes) * sms * tpsm / mib, double(limit) * sms * tpsm / mib,
                  mi == gpuSuccess ? double(free_b) / mib : 0., mi == gpuSuccess ? double(total_b) / mib : 0.,
                  gpuGetErrorString(e));
    throw std::runtime_error(msg);
  }
  reserved = a.localSizeBytes;
#else
  // On ROCm/AMDGPU scratch memory is managed per queue by HSA/KFD rather than permanently reserved per resident
  // thread via cudaLimitStackSize; leave as a no-op until validated on AMD hardware.
  (void)kernel;
  (void)kernel_type;
#endif
}
}  // namespace detail

void* dev_alloc(std::size_t bytes) {
  sync_begin();
  void* p = nullptr;
  if (const gpuError_t e = gpuMalloc(&p, bytes); e != gpuSuccess) {
    (void)gpuGetLastError();
    std::size_t free_b = 0, total_b = 0;
    gpuMemGetInfo(&free_b, &total_b);
    const double mib = 1024. * 1024.;
    char msg[320];
    std::snprintf(msg, sizeof msg,
                  "device memory too small: an allocation of %.0f MiB failed (%s); %.0f MiB of %.0f MiB are free. Use "
                  "a smaller domain, fewer levels or a GPU with more memory.",
                  double(bytes) / mib, gpuGetErrorString(e), double(free_b) / mib, double(total_b) / mib);
    throw std::runtime_error(msg);
  }
  ck(gpuMemsetAsync(p, 0, bytes, stream()), "gpuMemset");
  ck(gpuStreamSynchronize(stream()), "gpuMemset");
  sync_end();
  return p;
}
void dev_free(void* p) {
  if (!p) return;
  sync_begin();
  gpuFree(p);
  sync_end();
}
void copy_to_host(void* d, const void* s, std::size_t n) {
  sync_begin();
  ck(gpuMemcpyAsync(d, s, n, gpuMemcpyDeviceToHost, stream()), "D2H");
  ck(gpuStreamSynchronize(stream()), "D2H");
  sync_end();
}
void copy_to_dev(void* d, const void* s, std::size_t n) {
  sync_begin();
  ck(gpuMemcpyAsync(d, s, n, gpuMemcpyHostToDevice, stream()), "H2D");
  ck(gpuStreamSynchronize(stream()), "H2D");
  sync_end();
}
void copy_dev(void* d, const void* s, std::size_t n) { ck(gpuMemcpyAsync(d, s, n, gpuMemcpyDeviceToDevice, stream()), "D2D"); }
void copy_to_host_async(void* d, const void* s, std::size_t n) {
  ck(gpuMemcpyAsync(d, s, n, gpuMemcpyDeviceToHost, stream()), "D2H");
}
void copy_to_dev_async(void* d, const void* s, std::size_t n) {
  ck(gpuMemcpyAsync(d, s, n, gpuMemcpyHostToDevice, stream()), "H2D");
}
void host_section_begin() {
  sync_begin();
  ck(gpuStreamSynchronize(stream()), "sync");
}
void host_section_end() { sync_end(); }
void host_mark() {
  Graphs& gs = graphs();
  gs.init();
  sync_begin();  // the open segment launched (it waits for the segment before it, as every segment point)
  ck(gpuEventRecord(gs.mark, stream()), "gpuEventRecord");
  // the capture resumes at once: the launches queued next run while the host waits for the marker
  if (gs.in_step && graphs_mode() != 0 && !gs.capturing) {
    if (!gs.pend.empty()) gs.flush();  // a segment's fatal checks are never left unread behind a new capture
    gs.begin_capture();
  }
}
void host_section_begin_at_mark() {
  Graphs& gs = graphs();
  sync_begin();  // the launches since host_mark: launched, not waited for
  ck(gpuEventSynchronize(gs.mark), "sync");
}
void host_section_end_running() {
  Graphs& gs = graphs();
  if (!gs.pend.empty()) {  // fatal checks of a launched segment: read as host_section_end reads them
    sync_end();
    return;
  }
  if (gs.in_step && graphs_mode() != 0 && !gs.capturing) gs.begin_capture();
}
namespace {
thread_local bool stream_sync_only = false;  // set_thread_stream_sync
}  // namespace
void set_thread_stream_sync(bool on) { stream_sync_only = on; }
void dev_sync() {
  sync_begin();
  if (stream_sync_only) ck(gpuStreamSynchronize(stream()), "sync");
  else ck(gpuDeviceSynchronize(), "sync");
  sync_end();
}
int current_device() {
  int d = 0;
  ck(gpuGetDevice(&d), "gpuGetDevice");
  return d;
}
const char* backend_name() { return CPPWRF_GPU_BACKEND_NAME; }
void device_memory_report(const char* when) {
  static const bool on = [] { const char* e = std::getenv("CPPWRF_MEMINFO"); return e && *e && *e != '0'; }();
  if (!on) return;
  dev_sync();
  std::size_t free_b = 0, total_b = 0, limit = 0;
  int dev = 0, sms = 0, tpsm = 0;
  ck(gpuMemGetInfo(&free_b, &total_b), "gpuMemGetInfo");
#if defined(CPPWRF_USE_CUDA)
  ck(gpuDeviceGetLimit(&limit, gpuLimitStackSize), "gpuDeviceGetLimit");
#else
  if (gpuDeviceGetLimit(&limit, gpuLimitStackSize) != gpuSuccess) {
    (void)gpuGetLastError();
    limit = 0;
  }
#endif
  ck(gpuGetDevice(&dev), "gpuGetDevice");
  ck(gpuDeviceGetAttribute(&sms, gpuDevAttrMultiProcessorCount, dev), "gpuDeviceGetAttribute");
  ck(gpuDeviceGetAttribute(&tpsm, gpuDevAttrMaxThreadsPerMultiProcessor, dev), "gpuDeviceGetAttribute");
  const double mib = 1024. * 1024.;
  std::printf("cppwrf: device memory %s: %.0f MiB used of %.0f MiB; stack limit %zu bytes x %d resident threads "
              "(%.0f MiB)\n",
              when, double(total_b - free_b) / mib, double(total_b) / mib, limit, sms * tpsm,
              double(limit) * sms * tpsm / mib);
}
namespace {
int work_share = 1;  // set_device_work_share
}  // namespace
void set_device_work_share(int processes) { work_share = processes > 1 ? processes : 1; }
int device_work_share() { return work_share; }
std::size_t device_work_budget() {
  static const std::size_t cap = [] {
    const char* e = std::getenv("CPPWRF_DEVICE_MEMORY_MIB");
    const long long v = e && *e ? std::atoll(e) : 0;
    return v > 0 ? std::size_t(v) << 20 : std::size_t(0);
  }();
  std::size_t free_b = 0, total_b = 0;
  if (gpuMemGetInfo(&free_b, &total_b) != gpuSuccess) return 0;
  if (cap && cap < total_b) total_b = cap;
  return std::min(total_b / 8, free_b / 4) / std::size_t(work_share);
}
Reducer::Reducer(int max_k) : max_k_(max_k), hpart_(max_k) { dpart_ = static_cast<double*>(dev_alloc(sizeof(double) * max_k)); }
Reducer::~Reducer() { dev_free(dpart_); }
#else
namespace {
// Threaded memset/memcpy in contiguous static chunks: the same thread-to-memory mapping as the
// static parallel_for over (k, j), so first-touch places pages on the NUMA node that uses them
// and the per-step field copies of RK3 use all cores.
constexpr std::size_t kChunk = std::size_t(1) << 16;
void par_set0(void* p, std::size_t n) {
  const long nc = long((n + kChunk - 1) / kChunk);
#pragma omp parallel for schedule(static)
  for (long c = 0; c < nc; ++c) {
    const std::size_t o = std::size_t(c) * kChunk;
    std::memset(static_cast<char*>(p) + o, 0, std::min(kChunk, n - o));
  }
}
void par_copy(void* d, const void* s, std::size_t n) {
  const long nc = long((n + kChunk - 1) / kChunk);
#pragma omp parallel for schedule(static)
  for (long c = 0; c < nc; ++c) {
    const std::size_t o = std::size_t(c) * kChunk;
    std::memcpy(static_cast<char*>(d) + o, static_cast<const char*>(s) + o, std::min(kChunk, n - o));
  }
}
}  // namespace
void* dev_alloc(std::size_t bytes) {
  std::size_t n = (bytes + 63) / 64 * 64;
  void* p = std::aligned_alloc(64, n ? n : 64);
  if (!p) throw std::runtime_error("allocation failed");
  par_set0(p, n);
  return p;
}
void dev_free(void* p) { std::free(p); }
void copy_to_host(void* d, const void* s, std::size_t n) { par_copy(d, s, n); }
void copy_to_dev(void* d, const void* s, std::size_t n) { par_copy(d, s, n); }
void copy_dev(void* d, const void* s, std::size_t n) { par_copy(d, s, n); }
void copy_to_host_async(void* d, const void* s, std::size_t n) { std::memcpy(d, s, n); }
void copy_to_dev_async(void* d, const void* s, std::size_t n) { std::memcpy(d, s, n); }
void host_section_begin() {}
void host_section_end() {}
void host_mark() {}
void host_section_begin_at_mark() {}
void host_section_end_running() {}
void dev_sync() {}
void set_thread_stream_sync(bool) {}
void discard_pending_checks() noexcept {}
void release_thread_device() noexcept {}
namespace detail {
void fatal_transfer_failed(int err, const char* what) {
  throw std::runtime_error(std::string(what) + " failed (" + std::to_string(err) + ")");
}
}  // namespace detail
int current_device() { return 0; }
const char* backend_name() { return "cpu"; }
void device_memory_report(const char*) {}
std::size_t device_work_budget() { return 0; }
void set_device_work_share(int) {}
int device_work_share() { return 1; }
StepScope::StepScope(const void*) {}
StepScope::~StepScope() {}
void StepScope::end() {}
bool step_graphs_on() { return false; }
Reducer::Reducer(int max_k) : max_k_(max_k), hpart_(max_k) {}
Reducer::~Reducer() {}
#endif

// One warp per block: a domain of ~10^4 columns gives ~330 blocks, every SM of a large GPU gets some (128-thread
// blocks left more than half of the RTX PRO 6000's 188 SMs idle); one thread per column either way
ColumnBlock column_block() {
  static const ColumnBlock b = [] {
    ColumnBlock r{32, 1};
    if (const char* e = std::getenv("CPPWRF_COLUMN_BLOCK")) {
      int x = 0, y = 0;
      if (std::sscanf(e, "%dx%d", &x, &y) == 2 && x > 0 && x % 32 == 0 && y > 0 && x * y <= 1024) r = {x, y};
      else throw std::runtime_error(std::string("CPPWRF_COLUMN_BLOCK=") + e + ": XxY with X a multiple of 32");
    }
    return r;
  }();
  return b;
}

namespace {
struct ThreadStaging {
  void* p = nullptr;
  std::size_t bytes = 0;
  bool held = false;
};
ThreadStaging& thread_staging() {  // kept for the run; a worker thread frees it at its exit (release_host_staging)
  thread_local ThreadStaging t;  // trivially destroyed: never freed at process exit (the device may be gone)
  return t;
}
}  // namespace
HostStaging::HostStaging(std::size_t bytes) {
  ThreadStaging& t = thread_staging();
  if (!t.held && t.bytes < bytes) {
    void* p = nullptr;
#if defined(CPPWRF_USE_GPU)
    if (gpuHostAlloc(&p, bytes, gpuHostAllocDefault) != gpuSuccess) {
      (void)gpuGetLastError();
      p = nullptr;
    } else if (t.p) {
      gpuFreeHost(t.p);
    }
#else
    p = std::malloc(bytes);
    if (p && t.p) std::free(t.p);
#endif
    if (p) {
      t.p = p;
      t.bytes = bytes;
    }
  }
  if (!t.held && t.bytes >= bytes) {
    t.held = held_ = true;
    p_ = t.p;
    return;
  }
  own_.resize(bytes);  // nested, or the allocation failed
  p_ = own_.data();
}
HostStaging::~HostStaging() {
  if (held_) thread_staging().held = false;
}
void release_host_staging() {
  ThreadStaging& t = thread_staging();
  if (t.held || !t.p) return;
#if defined(CPPWRF_USE_GPU)
  if (gpuFreeHost(t.p) != gpuSuccess) (void)gpuGetLastError();
#else
  std::free(t.p);
#endif
  t.p = nullptr;
  t.bytes = 0;
}

int fatal_code_host[kFatalSlots] = {};

void set_fatal_slot(int slot) {
  if (slot < 0 || slot >= kFatalSlots) throw std::logic_error("set_fatal_slot: slot " + std::to_string(slot));
  detail::fatal_slot = slot;
}

const char* fatal_message(int code) {
  switch (code) {
    case kFatalSnowDensity: return "Physical snow depth is less than snow water equiv.";
    case kFatalKfQgSurface: return "QG, QG(NK).LT.0";
    case kFatalKfIstop: return "KAIN-FRITSCH, istop=1, diags";
    case kFatalNoahSoilTypes: return "Warning: too many input soil types";
    case kFatalNoahLanduseTypes: return "Warning: too many input landuse types";
    case kFatalNoahSlopeTypes: return "Warning: too many input slope types";
    case kFatalNoahRootLayers: return "Error: too many root layers";
    case kFatalNoahmpWater: return "Error: Water budget problem in NoahMP LSM";
    case kFatalNoahmpSolar: return "Error: Solar radiation budget problem in NoahMP LSM";
    case kFatalNoahmpEnergy: return "Error: Energy budget problem in NoahMP LSM";
    case kFatalNoahmpLongwave: return "Error: Longwave radiation budget problem in NoahMP LSM";
    case kFatalNoahmpZeroPlane: return "Error: ZeroPlaneDisp problem in NoahMP LSM";
    case kFatalNoahmpRefHeightCanopy: return "Error in ResistanceAboveCanopyMostMod.F90";
    case kFatalNoahmpRefHeightBare: return "Error in ResistanceBareGroundMostMod.F90";
    case kFatalNoahmpGlacierEnergy: return "Error: Surface energy budget problem in NoahMP LSM";
    case kFatalNoahUrbanLai:
      return "rdlai2d: an urban point with VEGFRA = 0 and LAI <= 0 is not supported (WRF's SFLX sets SHDFAC = 0.05, "
             "skips CANRES for XLAI = 0 and uses the plant coefficient PC of the previous land point, an undefined value)";
    case kFatalAcmRibx: return " RIBX never exceeds RIC";
    case kFatalUrbanZa:
      return "ZDC + Z0C + 2m is larger than the 1st WRF level - Stop in subroutine urban - change ZDC and Z0C";
    case kFatalUrbanSurface:
      return "sf_urban_physics = 1: an urban land-use point that is water or sea ice is not supported (WRF's surface "
             "driver gives it the undefined U10_URB2D, PSIM_URB2D, ... of a point lsm skips)";
    case kFatalUrbanInput:
      return "sf_urban_physics = 1: urban morphology input (HGT_URB2D > 0, NUDAPT: the UCM reads an undefined U10 and "
             "SW then) is not supported";
    case kFatalRucVilka: return " crash in surface energy budget ";
    case kFatalObsRegime: return "fddaobs_driver: compute_VIH STOP (Unknown regime type)";
    case kFatalObsKpblObs:
      return "obs nudging: a single-level upper-air observation whose influence layer holds the PBL top level is not "
             "supported (WRF's nudob reads kpbl_obs there, which its serial build never sets)";
    case kFatalObsSoundingLevels: return "wrf_fddaobs_in: in4dob";
    default: return "unknown fatal condition in a physics kernel";
  }
}

void fatal_stop(int code) {
  // wrf_error_fatal: the block on stdout (wrf_message) and on stderr (wrf_message2)
  const char* msg = fatal_message(code);
  for (std::FILE* f : {stdout, stderr}) {
    std::fprintf(f, "-------------- FATAL CALLED ---------------\n%s\n-------------------------------------------\n", msg);
    std::fflush(f);
  }
  throw FatalError(code, msg);
}

}  // namespace cppwrf
