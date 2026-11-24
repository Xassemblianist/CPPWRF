// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/tiles.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/tiles_internal.hpp"

#if defined(CPPWRF_USE_GPU)
#define CPPWRF_GPU_RUNTIME_HOST 1
#include "core/gpu_runtime.hpp"
#endif
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace cppwrf::tile {

using detail::Aborted;
using detail::Comm;
using detail::Post;

namespace {

// A barrier that another tile's error releases (Aborted); the check's meetings of the tiles and the reference run
class Barrier {
 public:
  void init(int n) { n_ = n; }
  void wait() {
    std::unique_lock<std::mutex> lk(m_);
    if (cancel_) throw Aborted();
    const long g = gen_;
    if (++count_ == n_) {
      count_ = 0;
      ++gen_;
      cv_.notify_all();
      return;
    }
    cv_.wait(lk, [&] { return gen_ != g || cancel_; });
    if (gen_ == g) throw Aborted();
  }
  void cancel() {
    std::lock_guard<std::mutex> lk(m_);
    cancel_ = true;
    cv_.notify_all();
  }

 private:
  std::mutex m_;
  std::condition_variable cv_;
  int n_ = 1, count_ = 0;
  long gen_ = 0;
  bool cancel_ = false;
};

struct Entry {
  char* base = nullptr;
  Layout L;
  int esz = 0;
  std::string name;
  bool live = false;
};
struct Registry {
  std::vector<Entry> e;
  std::unordered_map<const void*, int> idx;  // live allocations by base
  std::map<const char*, int> by_addr;        // live allocations by the address of element (0, 0, 0) (a view's data)
};

struct Iv {
  int lo, hi;
  bool empty() const { return hi <= lo; }
  int n() const { return hi - lo; }
};
Iv meet(Iv a, Iv b) { return Iv{std::max(a.lo, b.lo), std::min(a.hi, b.hi)}; }

struct Decomp {
  int nx = 0, ny = 0;
  bool per_x = false, per_y = false;
  std::vector<PatchBox> patch;
};

// A worker: the thread's (process's) patch number, its fields and its domains' decompositions (every worker creates
// the same domains and fields in the same order)
struct Worker {
  int id = 0, n = 1, px = 1, py = 1;
  Comm* comm = nullptr;
  Registry reg;
  std::vector<Decomp> doms;
  std::uint32_t seq = 0;  // collectives so far (checked in every message)
  bool service = false;   // the writer inside a tile::Service scope
  int dev = 0;
#if defined(CPPWRF_USE_GPU)
  char* dbuf = nullptr;  // device staging buffer of the messages (gathers, downloads, uploads)
  std::size_t dbuf_bytes = 0;
#endif
  struct Halo* halo = nullptr;  // the halo exchanges' plans and buffers (created at the first exchange)
};
thread_local Worker* W = nullptr;

// tiles of one process: what they share
struct Set {
  Options o;
  int n = 1;
  Registry* ref = nullptr;  // the check's reference run
  Barrier all;
  std::mutex serial;
  std::mutex err_m;
  std::exception_ptr err;
  bool err_root = false;
  std::string chk_msg;  // the check's first difference
  std::mutex chk_m;
  std::function<void()> cancel_net;
};
Set* S = nullptr;
thread_local long n_checks = 0;
// the check: fields whose values around the patch differed at the last check
thread_local std::vector<char> stale_before;
thread_local Registry* ref_reg = nullptr;  // the reference run's thread: its registry
thread_local std::unique_lock<std::mutex>* held_serial = nullptr;  // the Serial lock this thread holds

bool worker() { return W != nullptr; }
Registry* my_reg() { return W ? &W->reg : ref_reg; }

void reg_add(const void* base, const Layout& L, int esz, const std::string& name) {
  Registry* reg = my_reg();
  Entry e;
  e.base = static_cast<char*>(const_cast<void*>(base));
  e.L = L;
  e.esz = esz;
  e.name = name;
  e.live = true;
  reg->idx[base] = int(reg->e.size());
  reg->by_addr[e.base + L.offset * std::ptrdiff_t(esz)] = int(reg->e.size());
  reg->e.push_back(std::move(e));
}
void reg_remove(const void* base) {
  Registry* reg = my_reg();
  auto it = reg->idx.find(base);
  if (it == reg->idx.end()) return;
  Entry& e = reg->e[std::size_t(it->second)];
  e.live = false;
  reg->by_addr.erase(e.base + e.L.offset * std::ptrdiff_t(e.esz));
  reg->idx.erase(it);
}
const Track kTrack{&reg_add, &reg_remove};

// a field of a decomposed domain
bool domain_field(const Entry& e) { return e.live && e.L.dom >= 0 && e.L.si == 1; }
bool two_d(const Layout& L) { return L.sj == 0; }
int levels(const Layout& L) { return L.sk ? std::max(L.nz, 1) : 1; }

// ---- geometry: all from the patch table, the same on every worker ----
// the indices worker t's allocation of a field with layout L (any worker's) holds
Iv win_x(const Decomp& D, int t, const Layout& L) {
  const PatchBox& p = D.patch[std::size_t(t)];
  return Iv{p.i0 - Grid::halo, (p.i1 == D.nx ? L.nx : p.i1) + Grid::halo};
}
Iv win_y(const Decomp& D, int t, const Layout& L) {
  if (two_d(L)) return Iv{0, 1};
  const PatchBox& p = D.patch[std::size_t(t)];
  return Iv{p.j0 - L.hy, (p.j1 == D.ny ? L.ny : p.j1) + L.hy};
}
// the points worker s owns of the field: its patch; at the domain's edges up to the end of the halo
Iv own_x(const Decomp& D, int s, const Layout& L) {
  const PatchBox& p = D.patch[std::size_t(s)];
  return Iv{p.i0 == 0 ? -Grid::halo : p.i0, p.i1 == D.nx ? L.nx + Grid::halo : p.i1};
}
Iv own_y(const Decomp& D, int s, const Layout& L) {
  if (two_d(L)) return Iv{0, 1};
  const PatchBox& p = D.patch[std::size_t(s)];
  return Iv{p.j0 == 0 ? -L.hy : p.j0, p.j1 == D.ny ? L.ny + L.hy : p.j1};
}

const Decomp& decomp(const Layout& L) {
  if (L.dom < 0 || L.dom >= int(W->doms.size())) throw std::logic_error("decomposition: a field of no decomposed domain");
  return W->doms[std::size_t(L.dom)];
}

// ---- boxes of a field <-> message bytes (k slowest, i fastest) ----
struct Piece {
  int entry;
  Iv x, y;
  std::size_t off = 0;  // in the message
};
std::size_t piece_bytes(const Entry& e, const Piece& p) {
  return std::size_t(p.x.n()) * std::size_t(p.y.n()) * std::size_t(levels(e.L)) * std::size_t(e.esz);
}
// the messages' offsets (8-byte aligned) and the total
std::size_t layout_pieces(std::vector<Piece>& ps) {
  std::size_t o = 0;
  for (Piece& p : ps) {
    p.off = o;
    o += (piece_bytes(W->reg.e[std::size_t(p.entry)], p) + 7) / 8 * 8;
  }
  return o;
}

#if defined(CPPWRF_USE_GPU)
template <class U>
struct KBox {
  static constexpr bool kNoMulAdd = true;
  U* f;  // the field's element (0, 0, 0)
  U* b;  // the box in the staging buffer
  int sj, sk, x0, y0, nx, ny;
  bool put;
  CPPWRF_HD void operator()(int i, int j, int k) const {
    U& a = f[(x0 + i) + (y0 + j) * sj + k * sk];
    U& c = b[(std::size_t(k) * std::size_t(ny) + std::size_t(j)) * std::size_t(nx) + std::size_t(i)];
    if (put) a = c;
    else c = a;
  }
};
char* staging(std::size_t bytes) {
  if (bytes > W->dbuf_bytes) {
    dev_free(W->dbuf);
    W->dbuf = nullptr;
    W->dbuf_bytes = 0;
    W->dbuf = static_cast<char*>(dev_alloc(bytes));
    W->dbuf_bytes = bytes;
  }
  return W->dbuf;
}
template <class U>
void launch_box(const Entry& e, const Piece& p, char* dbuf, bool put) {
  const Layout& L = e.L;
  const Whole whole;
  parallel_for(Range3D{0, p.x.n(), 0, p.y.n(), 0, levels(L)},
               KBox<U>{reinterpret_cast<U*>(e.base) + L.offset, reinterpret_cast<U*>(dbuf + p.off), int(L.sj), int(L.sk),
                       p.x.lo, two_d(L) ? 0 : p.y.lo, p.x.n(), p.y.n(), put});
}
void launch_box(const Entry& e, const Piece& p, char* dbuf, bool put) {
  if (e.esz == 8) launch_box<std::uint64_t>(e, p, dbuf, put);
  else launch_box<std::uint32_t>(e, p, dbuf, put);
}
#endif

// the pieces ps of this worker's fields into msg (get) or from msg (put); msg has layout_pieces(ps) bytes
void move_pieces(const std::vector<Piece>& ps, char* msg, std::size_t bytes, bool put) {
  if (ps.empty()) return;
#if defined(CPPWRF_USE_GPU)
  char* d = staging(bytes);
  if (put) copy_to_dev(d, msg, bytes);
  for (const Piece& p : ps) launch_box(W->reg.e[std::size_t(p.entry)], p, d, put);
  if (!put) copy_to_host(msg, d, bytes);
#else
  (void)bytes;
  for (const Piece& p : ps) {
    const Entry& e = W->reg.e[std::size_t(p.entry)];
    const Layout& L = e.L;
    const std::size_t esz = std::size_t(e.esz), w = std::size_t(p.x.n()) * esz;
    char* m = msg + p.off;
    const int nk = levels(L);
    for (int k = 0; k < nk; ++k)
      for (int j = p.y.lo; j < p.y.hi; ++j) {
        char* f = e.base + (L.offset + p.x.lo + (two_d(L) ? 0 : j) * L.sj + k * L.sk) * std::ptrdiff_t(esz);
        if (put) std::memcpy(f, m, w);
        else std::memcpy(m, f, w);
        m += w;
      }
    if (!put) {  // the box's alignment padding is sent as zeros, not as stale bytes (layout_pieces)
      const std::size_t pb = piece_bytes(e, p);
      std::memset(m, 0, (pb + 7) / 8 * 8 - pb);
    }
  }
#endif
}

// ---- messages: a header that names the collective, then the payload ----
struct Hdr {
  std::uint32_t seq, kind;
  std::uint64_t bytes;
  std::uint64_t fp = 0;  // a halo message: the fingerprint of its boxes (Seg::fp); else 0
};
enum Kind : std::uint32_t {
  kExchange = 1, kWrap, kMax, kSum, kCode, kOwner, kDownload, kRaw, kService, kUpload, kGather, kLine, kMaxEach
};
enum ServiceOp : std::int32_t { kOpDone = 0, kOpDownload, kOpUpload };

// CPPWRF_TILE_TRACE=1: every message on stderr (worker, collective, kind, peer, bytes). It only prints: every worker
// moves the same messages with it or without it (a rank's own setting cannot make its calls differ from the others')
bool trace() {
  static const bool on = [] {
    const char* e = std::getenv("CPPWRF_TILE_TRACE");
    return e && *e && *e != '0';
  }();
  return on;
}

void send_msg(int to, Kind kind, const char* p, std::size_t n) {
  if (trace()) std::fprintf(stderr, "tile %d: collective %u kind %u send to %d, %zu bytes\n", W->id, W->seq, unsigned(kind), to, n);
  std::vector<char> m(sizeof(Hdr) + n);
  const Hdr h{W->seq, kind, n};
  std::memcpy(m.data(), &h, sizeof h);
  if (n) std::memcpy(m.data() + sizeof h, p, n);
  W->comm->send(to, std::move(m));
}
// the next message from `from`: it must be of this collective (every worker makes the same collective calls in the
// same order: a different one is an error of the program, named here)
std::vector<char> recv_msg(int from, Kind kind, std::size_t n) {
  if (trace()) std::fprintf(stderr, "tile %d: collective %u kind %u recv from %d, %zu bytes\n", W->id, W->seq, unsigned(kind), from, n);
  std::vector<char> m = W->comm->recv(from);
  Hdr h{};
  if (m.size() >= sizeof h) std::memcpy(&h, m.data(), sizeof h);
  if (m.size() < sizeof h || h.seq != W->seq || h.kind != kind || h.bytes != n || m.size() != sizeof h + n) {
    char b[240];
    std::snprintf(b, sizeof b,
                  "decomposition: worker %d expected collective %u of kind %u with %zu bytes from worker %d and got "
                  "collective %u of kind %u with %llu bytes (the workers' calls differ)",
                  W->id, W->seq, unsigned(kind), n, from, h.seq, h.kind, static_cast<unsigned long long>(h.bytes));
    throw std::logic_error(b);
  }
  m.erase(m.begin(), m.begin() + std::ptrdiff_t(sizeof h));
  return m;
}
void end_collective() {
  W->comm->flush();
  ++W->seq;
}

// ---- halo exchanges: persistent plans, one pack and one unpack launch, one transfer per direction ----
// An exchange (halo, wrap, line) of a set of fields is planned once (Worker::halo, by the fields and the kind): the
// boxes each worker sends and receives (the same geometry on every worker), their places in one send and one receive
// buffer, and on the GPU the piece table of the pack and unpack kernels. A message per peer is [Hdr][its boxes]; the
// boxes are 8-byte aligned. Every exchange of a plan then: packs every box it sends with one launch on the worker's
// stream; moves the messages (a GPU-aware MPI: from and into the device buffers; else one copy of the whole send
// buffer into pinned host memory behind the pack launch, the wait for the stream, the transport over the pinned
// buffers, one copy of the receive buffer to the device); unpacks every box received with one launch. No device-wide
// wait: the worker's later launches are behind the unpack on the same stream. The values are bytes copied from the
// points their owner holds to the same points of the receiver: what each point holds after the exchange does not
// depend on the transport, the order of the messages or the launches' scheduling (the boxes received are disjoint).
constexpr std::size_t kHdr = sizeof(Hdr);
static_assert(kHdr % 8 == 0, "the boxes of a message are 8-byte aligned");

struct Seg {
  int peer;
  std::size_t off, bytes;  // the header at off, the boxes (bytes) after it
  // the fingerprint of the message's boxes, the same for its sender and its receiver: the exchange's kind and for
  // each box its field's name, element size, levels and extents, in their order. A receiver that expects other
  // boxes than its sender packed (the workers' calls differ in their kind, their fields or their order) stops
  // before it unpacks a byte.
  std::uint64_t fp = 0;
};

#if defined(CPPWRF_USE_GPU)
// one box of the pack or unpack launch
struct DevPiece {
  char* f;             // the field's element (0, 0, 0)
  long long sj, sk;    // its strides (elements)
  int x0, y0, nx, ny, nk, esz;
  unsigned long long off;  // the box in the buffer (k slowest, i fastest)
  long long blk0;          // its first block of the launch
};
constexpr int kPieceThreads = 256;
// every box of a list: block b belongs to the last piece with blk0 <= b; one element per thread
__global__ void kpieces(const DevPiece* ps, int n, char* buf, int put) {
  const long long b = blockIdx.x;
  int lo = 0, hi = n - 1;
  while (lo < hi) {
    const int mid = (lo + hi + 1) / 2;
    if (ps[mid].blk0 <= b) lo = mid;
    else hi = mid - 1;
  }
  const DevPiece p = ps[lo];
  const long long q = (b - p.blk0) * kPieceThreads + threadIdx.x;
  if (q >= (long long)p.nx * p.ny * p.nk) return;
  const int i = int(q % p.nx);
  const long long r = q / p.nx;
  const int j = int(r % p.ny), k = int(r / p.ny);
  const long long e = (p.x0 + i) + (long long)(p.y0 + j) * p.sj + (long long)k * p.sk;
  if (p.esz == 8) {
    std::uint64_t* a = reinterpret_cast<std::uint64_t*>(p.f) + e;
    std::uint64_t* c = reinterpret_cast<std::uint64_t*>(buf + p.off) + q;
    if (put) *a = *c;
    else *c = *a;
  } else {
    std::uint32_t* a = reinterpret_cast<std::uint32_t*>(p.f) + e;
    std::uint32_t* c = reinterpret_cast<std::uint32_t*>(buf + p.off) + q;
    if (put) {
      *a = *c;
    } else {
      *c = *a;
      // an odd count of 4-byte elements: the box's 4 bytes of alignment padding are sent as zeros, not stale bytes
      if (q == (long long)p.nx * p.ny * p.nk - 1 && (q & 1) == 0) c[1] = 0;
    }
  }
}
#endif

// CPPWRF_HALO_STATS=1: the wall time of every exchange's parts, per plan (printed at the worker's end)
struct PlanTimes {
  long n = 0, overlapped = 0;  // exchanges; those begun before the launches that overlap them (FillAsync, core/halo.hpp)
  double between = 0;  // the host's time between begin and finish of the overlapped ones
  double pack = 0, wait = 0, xfer = 0, unpack = 0;  // seconds: pack launch (+ copy queued), the wait for the stream,
                                                    // the transport, the unpack (+ copy) queued
};
double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Plan {
  Kind kind = kExchange;
  std::string label;  // the fields (CPPWRF_HALO_STATS)
  mutable PlanTimes times;
  std::vector<Piece> snd, rcv;            // offsets in the send / receive buffer
  std::vector<Piece> self_src, self_dst;  // a wrap's points the worker sends itself: in the send buffer after smsg
  std::vector<Seg> out, in;
  std::size_t smsg = 0, sbytes = 0, rbytes = 0;  // the send buffer's messages, all of it; the receive buffer
  std::size_t pieces = 0;
#if defined(CPPWRF_USE_GPU)
  // the device piece table: [snd, self_src] (pack into the send buffer), [rcv] (unpack from the receive buffer),
  // [self_dst] (unpack from the send buffer)
  DevPiece* dev = nullptr;
  int first[3] = {0, 0, 0}, count[3] = {0, 0, 0};
  long long blocks[3] = {0, 0, 0};
  ~Plan() { dev_free(dev); }
#endif
};

struct HostBuf {
  char* p = nullptr;
  std::size_t cap = 0;
  bool pinned = false;
};

struct Halo {
  std::map<std::vector<long long>, std::unique_ptr<Plan>> plans;
  HostBuf hs, hr;  // host send / receive buffers (pinned on the GPU)
  std::vector<Hdr> ctl_out, ctl_in;  // the headers sent and received beside device-buffer messages
#if defined(CPPWRF_USE_GPU)
  char* ds = nullptr;  // device send / receive buffers
  char* dr = nullptr;
  std::size_t dscap = 0, drcap = 0;
#endif
  long exchanges = 0, messages = 0, plans_made = 0;
  double sent = 0;
  // CPPWRF_HALO_STATS: the plans' times (kept when a plan is dropped) and the other collectives' (by kind)
  std::map<std::string, PlanTimes> dropped;
  std::map<unsigned, PlanTimes> others;
  bool timed = false;
  double tm[5] = {0, 0, 0, 0, 0};  // the open exchange's times
  const Plan* pending = nullptr;     // an exchange begun (exchange_begin) and not finished
  std::unique_ptr<Plan> pending_own;  // its plan when it is not kept
  static void release(HostBuf& b) {
    if (!b.p) return;
#if defined(CPPWRF_USE_GPU)
    if (b.pinned) {
      if (gpuFreeHost(b.p) != gpuSuccess) (void)gpuGetLastError();
    } else {
      std::free(b.p);
    }
#else
    std::free(b.p);
#endif
    b = HostBuf{};
  }
  ~Halo() {
    plans.clear();
    release(hs);
    release(hr);
#if defined(CPPWRF_USE_GPU)
    dev_free(ds);
    dev_free(dr);
#endif
  }
};

// CPPWRF_HALO_STATS=1: each worker's exchanges, plans, messages and bytes sent at its end (stderr)
bool halo_stats() {
  static const bool on = [] {
    const char* e = std::getenv("CPPWRF_HALO_STATS");
    return e && *e && *e != '0';
  }();
  return on;
}

Halo& halo() {
  if (!W->halo) W->halo = new Halo;
  return *W->halo;
}

// the host buffer b with at least n bytes (its contents are not kept)
void grow(HostBuf& b, std::size_t n) {
  if (n <= b.cap) return;
  const std::size_t cap = std::max(n, b.cap + b.cap / 2);
#if defined(CPPWRF_USE_GPU)
  host_section_begin();  // a copy from or into the old buffer may still be queued
  Halo::release(b);
  void* p = nullptr;
  if (gpuHostAlloc(&p, cap, gpuHostAllocDefault) == gpuSuccess) {
    b.pinned = true;
  } else {
    (void)gpuGetLastError();
    p = std::malloc(cap);
    b.pinned = false;
  }
  host_section_end();
#else
  Halo::release(b);
  void* p = std::malloc(cap);
#endif
  if (!p) throw std::runtime_error("decomposition: no memory for the halo buffers");
  b.p = static_cast<char*>(p);
  b.cap = cap;
}
#if defined(CPPWRF_USE_GPU)
void grow_dev(char*& d, std::size_t& c, std::size_t n) {
  if (n <= c) return;
  const std::size_t cap = std::max(n, c + c / 2);
  dev_free(d);  // waits for the device (as every free)
  d = nullptr;
  c = 0;
  d = static_cast<char*>(dev_alloc(cap));
  c = cap;
}
void launch_pieces(const Plan& P, int list, char* buf, bool put) {
  if (P.count[list] == 0) return;
  if (P.blocks[list] > 0x7fffffffLL) throw std::runtime_error("decomposition: a halo exchange too large for one launch");
  kpieces<<<unsigned(P.blocks[list]), kPieceThreads, 0, gpu_launch_stream()>>>(P.dev + P.first[list], P.count[list],
                                                                                 buf, put ? 1 : 0);
  gpu_check_launch("halo pieces");
}
#endif

std::uint64_t fnv(std::uint64_t h, const void* p, std::size_t n) {
  const auto* c = static_cast<const unsigned char*>(p);
  for (std::size_t k = 0; k < n; ++k) h = (h ^ c[k]) * 0x100000001b3ULL;
  return h;
}
std::uint64_t fingerprint(Kind kind, const std::vector<Piece>& ps) {
  std::uint64_t h = fnv(0xcbf29ce484222325ULL, &kind, sizeof kind);
  for (const Piece& p : ps) {
    const Entry& e = W->reg.e[std::size_t(p.entry)];
    h = fnv(h, e.name.data(), e.name.size());
    const std::int64_t v[4] = {e.esz, levels(e.L), p.x.n(), p.y.n()};
    h = fnv(h, v, sizeof v);
  }
  return h;
}

// The plan of the boxes snd[r] to every worker r, rcv[s] from every worker s (empty lists: no message) and the
// pieces self_src the worker copies into its own self_dst (the same sizes, in the same order)
std::unique_ptr<Plan> make_plan(Kind kind, std::vector<std::vector<Piece>>& snd, std::vector<std::vector<Piece>>& rcv,
                                std::vector<Piece>& self_src, std::vector<Piece>& self_dst) {
  auto P = std::make_unique<Plan>();
  P->kind = kind;
  auto place = [kind](std::vector<std::vector<Piece>>& lists, std::vector<Piece>& all, std::vector<Seg>& segs) {
    std::size_t o = 0;
    for (int r = 0; r < int(lists.size()); ++r) {
      std::vector<Piece>& ps = lists[std::size_t(r)];
      if (ps.empty()) continue;
      const std::size_t nb = layout_pieces(ps);
      for (Piece& p : ps) {
        p.off += o + kHdr;
        all.push_back(p);
      }
      segs.push_back(Seg{r, o, nb, fingerprint(kind, ps)});
      o += kHdr + nb;
    }
    return o;
  };
  P->smsg = place(snd, P->snd, P->out);
  std::size_t o = P->smsg;
  if (!self_src.empty()) {
    const std::size_t nb = layout_pieces(self_src);
    layout_pieces(self_dst);
    for (Piece& p : self_src) p.off += o;
    for (Piece& p : self_dst) p.off += o;
    o += nb;
  }
  P->sbytes = o;
  P->self_src = self_src;
  P->self_dst = self_dst;
  P->rbytes = place(rcv, P->rcv, P->in);
  P->pieces = P->snd.size() + P->rcv.size() + P->self_src.size() + P->self_dst.size();
#if defined(CPPWRF_USE_GPU)
  std::vector<DevPiece> t;
  t.reserve(P->pieces);
  int list = 0;
  long long blk = 0;
  auto add = [&](const std::vector<Piece>& ps) {
    for (const Piece& p : ps) {
      const Entry& e = W->reg.e[std::size_t(p.entry)];
      const Layout& L = e.L;
      DevPiece d{};
      d.f = e.base + L.offset * std::ptrdiff_t(e.esz);
      d.sj = L.sj;
      d.sk = L.sk;
      d.x0 = p.x.lo;
      d.y0 = two_d(L) ? 0 : p.y.lo;
      d.nx = p.x.n();
      d.ny = p.y.n();
      d.nk = levels(L);
      d.esz = e.esz;
      d.off = p.off;
      d.blk0 = blk;
      const long long n = (long long)d.nx * d.ny * d.nk;
      blk += (n + kPieceThreads - 1) / kPieceThreads;
      t.push_back(d);
      ++P->count[list];
    }
  };
  auto next = [&](int l) {
    P->blocks[list] = blk;
    list = l;
    blk = 0;
    P->first[l] = int(t.size());
  };
  add(P->snd);
  add(P->self_src);
  next(1);
  add(P->rcv);
  next(2);
  add(P->self_dst);
  P->blocks[2] = blk;
  if (!t.empty()) {
    P->dev = static_cast<DevPiece*>(dev_alloc(t.size() * sizeof(DevPiece)));
    copy_to_dev(P->dev, t.data(), t.size() * sizeof(DevPiece));
  }
#endif
  return P;
}

// One exchange of the plan P (a collective) in two calls. begin_plan packs every box sent (and, staged, queues the
// copy of the send buffer to the host); with `mark` it then marks the stream (host_mark) and returns: the worker may
// launch kernels that neither write the points sent nor read the points received before it calls finish_plan, which
// waits for the marker only (those kernels run on during the transfer), moves the messages and queues the unpack.
// Without `mark` finish_plan waits for the stream, as one call run_plan does.
bool plan_empty(const Plan& P) { return P.out.empty() && P.in.empty() && P.self_src.empty(); }
void begin_plan(const Plan& P, bool mark) {
  Halo& H = halo();
  ++H.exchanges;
  if (plan_empty(P)) return;  // nothing of this worker's
  H.timed = halo_stats();
  if (H.timed) H.tm[0] = now_s();
#if defined(CPPWRF_USE_GPU)
  const bool dev = W->comm->device_buffers();
  grow_dev(H.ds, H.dscap, P.sbytes);
  grow_dev(H.dr, H.drcap, P.rbytes);
  if (!dev) {
    grow(H.hs, P.smsg);
    grow(H.hr, P.rbytes);
  }
  launch_pieces(P, 0, H.ds, false);
  if (!dev && P.smsg) {
    if (H.hs.pinned) copy_to_host_async(H.hs.p, H.ds, P.smsg);
    else copy_to_host(H.hs.p, H.ds, P.smsg);
  }
  if (mark) host_mark();
#else
  (void)mark;
  grow(H.hs, P.sbytes);
  grow(H.hr, P.rbytes);
  move_pieces(P.snd, H.hs.p, P.smsg, false);
  move_pieces(P.self_src, H.hs.p, P.sbytes, false);
#endif
  if (H.timed) H.tm[1] = now_s();
}
void finish_plan(const Plan& P, bool marked) {
  Halo& H = halo();
  if (plan_empty(P)) {
    end_collective();
    return;
  }
  const bool timed = H.timed;
  double* tm = H.tm;
  if (timed) tm[4] = now_s();  // the end of the launches between begin_plan and finish_plan
#if defined(CPPWRF_USE_GPU)
  const bool dev = W->comm->device_buffers();
  // the boxes are in the send buffer; the previous exchange's unpack has ended
  if (marked) host_section_begin_at_mark();
  else host_section_begin();
  char* sb = dev ? H.ds : H.hs.p;
  char* rb = dev ? H.dr : H.hr.p;
#else
  (void)marked;
  const bool dev = false;
  char* sb = H.hs.p;
  char* rb = H.hr.p;
#endif
  if (timed) tm[2] = now_s();
  // each message's header (the collective, its kind, its bytes, its boxes' fingerprint) in front of its boxes:
  // staged, in the host buffer; with device buffers, a host header of its own sent before the message (the same
  // pair of workers and tag: MPI keeps their order)
  std::vector<Post> out, in;
  out.reserve(2 * P.out.size());
  in.reserve(2 * P.in.size());
  std::vector<Hdr>& hs = H.ctl_out;
  std::vector<Hdr>& hr = H.ctl_in;
  hs.assign(P.out.size(), Hdr{});
  hr.assign(P.in.size(), Hdr{});
  for (std::size_t q = 0; q < P.out.size(); ++q) {
    const Seg& m = P.out[q];
    if (trace())
      std::fprintf(stderr, "tile %d: collective %u kind %u send to %d, %zu bytes\n", W->id, W->seq, unsigned(P.kind), m.peer, m.bytes);
    const Hdr h{W->seq, P.kind, m.bytes, m.fp};
    if (dev) {
      hs[q] = h;
      out.push_back(Post{m.peer, reinterpret_cast<char*>(&hs[q]), kHdr});
      out.push_back(Post{m.peer, sb + m.off + kHdr, m.bytes});
    } else {
      std::memcpy(sb + m.off, &h, sizeof h);
      out.push_back(Post{m.peer, sb + m.off, kHdr + m.bytes});
    }
  }
  for (std::size_t q = 0; q < P.in.size(); ++q) {
    const Seg& m = P.in[q];
    if (trace())
      std::fprintf(stderr, "tile %d: collective %u kind %u recv from %d, %zu bytes\n", W->id, W->seq, unsigned(P.kind), m.peer, m.bytes);
    if (dev) {
      in.push_back(Post{m.peer, reinterpret_cast<char*>(&hr[q]), kHdr});
      in.push_back(Post{m.peer, rb + m.off + kHdr, m.bytes});
    } else {
      in.push_back(Post{m.peer, rb + m.off, kHdr + m.bytes});
    }
  }
  W->comm->exchange(out, in, dev);
  if (timed) tm[3] = now_s();
  H.messages += long(P.out.size());
  for (const Seg& m : P.out) H.sent += double(m.bytes);
  for (std::size_t q = 0; q < P.in.size(); ++q) {  // before a byte of the messages is unpacked
    const Seg& m = P.in[q];
    Hdr h{};
    if (dev) h = hr[q];
    else std::memcpy(&h, rb + m.off, sizeof h);
    if (h.seq != W->seq || h.kind != P.kind || h.bytes != m.bytes || h.fp != m.fp) {
      char b[300];
      std::snprintf(b, sizeof b,
                    "decomposition: worker %d expected collective %u of kind %u with %zu bytes from worker %d and got "
                    "collective %u of kind %u with %llu bytes%s (the workers' calls differ)",
                    W->id, W->seq, unsigned(P.kind), m.bytes, m.peer, h.seq, h.kind,
                    static_cast<unsigned long long>(h.bytes),
                    h.fp != m.fp ? " of other fields or in another order" : "");
      throw std::logic_error(b);
    }
  }
#if defined(CPPWRF_USE_GPU)
  // marked: the kernels launched since the marker may still run; the unpack is queued behind them
  if (marked) host_section_end_running();
  else host_section_end();
  if (!dev && P.rbytes) {
    if (H.hr.pinned) copy_to_dev_async(H.dr, H.hr.p, P.rbytes);
    else copy_to_dev(H.dr, H.hr.p, P.rbytes);
  }
  launch_pieces(P, 1, H.dr, true);
  launch_pieces(P, 2, H.ds, true);
#else
  move_pieces(P.rcv, H.hr.p, P.rbytes, true);
  move_pieces(P.self_dst, H.hs.p, P.sbytes, true);
#endif
  end_collective();
  if (timed) {
    const double t5 = now_s();
    PlanTimes& T = P.times;
    ++T.n;
    T.pack += tm[1] - tm[0];
    T.wait += tm[2] - tm[4];
    T.xfer += tm[3] - tm[2];
    T.unpack += t5 - tm[3];
    if (marked) {
      ++T.overlapped;
      T.between += tm[4] - tm[1];
    }
  }
}
void run_plan(const Plan& P) {
  begin_plan(P, false);
  finish_plan(P, false);
}

// CPPWRF_HALO_STATS: the plan's kind and fields
void label_plan(Plan& P) {
  if (!halo_stats()) return;
  P.label = P.kind == kExchange ? "halo" : P.kind == kWrap ? "wrap" : P.kind == kLine ? "line" : "other";
  std::vector<int> seen;
  for (const std::vector<Piece>* v : {&P.snd, &P.rcv, &P.self_src})
    for (const Piece& p : *v)
      if (std::find(seen.begin(), seen.end(), p.entry) == seen.end()) {
        seen.push_back(p.entry);
        P.label += (seen.size() == 1 ? " " : ",") + W->reg.e[std::size_t(p.entry)].name;
      }
}
void keep_times(Halo& H, const Plan& P) {
  if (!halo_stats() || P.times.n == 0) return;
  PlanTimes& T = H.dropped[P.label];
  T.n += P.times.n;
  T.pack += P.times.pack;
  T.wait += P.times.wait;
  T.xfer += P.times.xfer;
  T.unpack += P.times.unpack;
  T.overlapped += P.times.overlapped;
  T.between += P.times.between;
}
// the plan of `key` (made by make() at its first use and kept; keep false: made, run and dropped), run
std::unique_ptr<Plan> labelled(const std::function<std::unique_ptr<Plan>()>& make) {
  std::unique_ptr<Plan> P = make();
  label_plan(*P);
  return P;
}
// the plan of `key` (made at its first use and kept), or with keep false a plan made for this exchange only (`own`)
const Plan& plan_of(std::vector<long long>&& key, const std::function<std::unique_ptr<Plan>()>& make, bool keep,
                    std::unique_ptr<Plan>& own) {
  Halo& H = halo();
  if (!keep) {
    own = labelled(make);
    ++H.plans_made;
    return *own;
  }
  auto it = H.plans.find(key);
  if (it == H.plans.end()) {
    if (H.plans.size() >= 4096) {  // fields allocated and freed again and again: no unbounded growth
      for (const auto& kv : H.plans) keep_times(H, *kv.second);
      H.plans.clear();
    }
    it = H.plans.emplace(std::move(key), labelled(make)).first;
    ++H.plans_made;
  }
  return *it->second;
}
void exchange_planned(std::vector<long long>&& key, const std::function<std::unique_ptr<Plan>()>& make, bool keep = true) {
  Halo& H = halo();
  if (H.pending) throw std::logic_error("decomposition: a collective while a halo exchange is open (FillAsync, core/halo.hpp)");
  std::unique_ptr<Plan> own;
  const Plan& P = plan_of(std::move(key), make, keep, own);
  run_plan(P);
  if (own) keep_times(H, *own);
}

// CPPWRF_HALO_STATS: the exchanges' times per plan (the largest first) and the other collectives' (stderr)
void print_times(int id, Halo& H) {
  for (const auto& kv : H.plans) keep_times(H, *kv.second);
  std::vector<std::pair<std::string, PlanTimes>> v(H.dropped.begin(), H.dropped.end());
  std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) {
    return a.second.pack + a.second.wait + a.second.xfer + a.second.unpack >
           b.second.pack + b.second.wait + b.second.xfer + b.second.unpack;
  });
  PlanTimes all;
  for (const auto& [l, T] : v) {
    all.n += T.n; all.pack += T.pack; all.wait += T.wait; all.xfer += T.xfer; all.unpack += T.unpack;
    all.overlapped += T.overlapped; all.between += T.between;
  }
  std::fprintf(stderr,
               "cppwrf halo: worker %d: exchanges %ld: %.3f s in all: pack %.3f s, wait for the stream %.3f s, transfer "
               "%.3f s, unpack %.3f s\n",
               id, all.n, all.pack + all.wait + all.xfer + all.unpack, all.pack, all.wait, all.xfer, all.unpack);
  if (all.overlapped)
    std::fprintf(stderr, "cppwrf halo: worker %d: %ld exchanges overlapped by launches (%.3f s of launching between)\n",
                 id, all.overlapped, all.between);
  std::fprintf(stderr, "cppwrf halo: worker %d:   count   total ms  pack us  wait us  xfer us unpack us  plan\n", id);
  for (const auto& [l, T] : v) {
    const double us = 1e6 / double(T.n);
    std::fprintf(stderr, "cppwrf halo: worker %d: %7ld %10.2f %8.1f %8.1f %8.1f %9.1f  %s%s\n", id, T.n,
                 1e3 * (T.pack + T.wait + T.xfer + T.unpack), T.pack * us, T.wait * us, T.xfer * us, T.unpack * us,
                 l.c_str(), T.overlapped ? " (overlapped)" : "");
  }
  for (const auto& [k, T] : H.others)
    std::fprintf(stderr, "cppwrf halo: worker %d: collective kind %u: %ld calls, %.2f ms (%.1f us each)\n", id, k, T.n,
                 1e3 * T.xfer, 1e6 * T.xfer / double(T.n));
}

// the field with the allocation `p` (data false) or with the element (0, 0, 0) at `p` (a view's data pointer; with
// a patch it may lie outside the allocation, inside another field's)
int index_of(const void* p, bool data = false) {
  Registry* reg = &W->reg;
  if (!data) {
    auto it = reg->idx.find(p);
    if (it != reg->idx.end()) return it->second;
  } else {
    auto it = reg->by_addr.find(static_cast<const char*>(p));
    if (it != reg->by_addr.end()) return it->second;
  }
  throw std::logic_error("decomposition: exchange of a field the worker did not allocate");
}

// (a field given twice is exchanged once, at its first place: two boxes of one field would write the same points from
// two blocks of the unpack launch)
std::vector<int> entries_of(const void* const* bases, int n, bool data = false) {
  std::vector<int> idx;
  idx.reserve(std::size_t(n));
  for (int k = 0; k < n; ++k) {
    const int i = index_of(bases[k], data);
    if (std::find(idx.begin(), idx.end(), i) != idx.end()) continue;
    if (!domain_field(W->reg.e[std::size_t(i)]))
      throw std::logic_error("decomposition: exchange of " + W->reg.e[std::size_t(i)].name + ", not a field of a decomposed domain");
    idx.push_back(i);
  }
  return idx;
}

// the points the other workers own inside each worker's allocation
std::unique_ptr<Plan> exchange_plan(const std::vector<int>& idx) {
  const int t = W->id;
  std::vector<std::vector<Piece>> snd(std::size_t(W->n)), rcv(std::size_t(W->n));
  for (int i : idx) {
    const Entry& e = W->reg.e[std::size_t(i)];
    const Layout& L = e.L;
    const Decomp& D = decomp(L);
    for (int o = 0; o < W->n; ++o) {
      if (o == t) continue;
      const Iv sx = meet(win_x(D, o, L), own_x(D, t, L)), sy = meet(win_y(D, o, L), own_y(D, t, L));
      if (!sx.empty() && !sy.empty()) snd[std::size_t(o)].push_back(Piece{i, sx, sy});
      const Iv rx = meet(win_x(D, t, L), own_x(D, o, L)), ry = meet(win_y(D, t, L), own_y(D, o, L));
      if (!rx.empty() && !ry.empty()) rcv[std::size_t(o)].push_back(Piece{i, rx, ry});
    }
  }
  std::vector<Piece> none, none2;
  return make_plan(kExchange, snd, rcv, none, none2);
}
void exchange_entries(const std::vector<int>& idx, bool keep = true) {
  std::vector<long long> key{0};
  key.insert(key.end(), idx.begin(), idx.end());
  exchange_planned(std::move(key), [&] { return exchange_plan(idx); }, keep);
}

void exchange_hook(const void* const* bases, int n, bool data) {
  if (!W) return;
  exchange_entries(entries_of(bases, n, data));
}
// the exchange of these fields begun (hooks.exchange_begin): packed, the stream marked; finished by exchange_finish
void exchange_begin_hook(const void* const* bases, int n) {
  if (!W) return;
  Halo& H = halo();
  if (H.pending) throw std::logic_error("decomposition: a halo exchange begun while another is open");
  const std::vector<int> idx = entries_of(bases, n);
  std::vector<long long> key{0};
  key.insert(key.end(), idx.begin(), idx.end());
  const Plan& P = plan_of(std::move(key), [&] { return exchange_plan(idx); }, true, H.pending_own);
  begin_plan(P, true);
  H.pending = &P;
}
void exchange_finish_hook() {
  if (!W) return;
  Halo& H = halo();
  if (!H.pending) throw std::logic_error("decomposition: no halo exchange to finish");
  const Plan* P = H.pending;
  H.pending = nullptr;
  finish_plan(*P, true);
  if (H.pending_own) {
    keep_times(H, *H.pending_own);
    H.pending_own.reset();
  }
}

bool wraps_hook(int dom, int axis) {
  if (!W || dom < 0) return false;
  const Decomp& D = W->doms[std::size_t(dom)];
  return axis == 0 ? (D.per_x && W->px > 1) : (D.per_y && W->py > 1);
}

// The periodic halo of direction `axis` (several patches along it): the halo point i takes the value of the point
// i mod n (core/halo.cpp HaloLine: -3 .. -1 <- n - 3 .. n - 1; n .. <- 0 ..; faces also n + 3 <- 3) from the worker
// that owns it, over the receiver's tangential range: its rows (x) or its whole x range with the halo (y).
std::unique_ptr<Plan> wrap_plan(const std::vector<int>& idx, int axis) {
  const int t = W->id;
  std::vector<std::vector<Piece>> snd(std::size_t(W->n)), rcv(std::size_t(W->n));
  std::vector<Piece> self_src, self_dst;  // the wrapped points the receiver owns itself
  for (int i : idx) {
    const Entry& e = W->reg.e[std::size_t(i)];
    const Layout& L = e.L;
    const Decomp& D = decomp(L);
    const int nn = axis == 0 ? D.nx : D.ny, ln = axis == 0 ? L.nx : L.ny;
    auto lo = [&](int w) { return axis == 0 ? D.patch[std::size_t(w)].i0 : D.patch[std::size_t(w)].j0; };
    auto hi = [&](int w) { return axis == 0 ? D.patch[std::size_t(w)].i1 : D.patch[std::size_t(w)].j1; };
    auto tang = [&](int w) { return axis == 0 ? w / W->px : w % W->px; };  // the row (x) or column (y) of patches
    // the tangential range of receiver w
    auto trange = [&](int w) { return axis == 0 ? meet(win_y(D, w, L), Iv{0, L.ny}) : win_x(D, w, L); };
    // from sender s to receiver r: the receiver's halo boxes (dst) and the sender's source boxes (dst shifted)
    auto pieces = [&](int s, int r, bool dst, std::vector<Piece>& out) {
      if (tang(s) != tang(r)) return;
      const Iv own{lo(s), hi(s)};
      const Iv tr = trange(r);
      for (int side = 0; side < 2; ++side) {
        if (side == 0 ? lo(r) != 0 : hi(r) != nn) continue;
        const Iv halo = side == 0 ? Iv{-Grid::halo, 0} : Iv{nn, ln + Grid::halo};
        const int shift = side == 0 ? nn : -nn;
        const Iv src = meet(Iv{halo.lo + shift, halo.hi + shift}, own);
        if (src.empty() || tr.empty()) continue;
        const Iv a = dst ? Iv{src.lo - shift, src.hi - shift} : src;
        out.push_back(axis == 0 ? Piece{i, a, tr} : Piece{i, tr, a});
      }
    };
    for (int o = 0; o < W->n; ++o) {
      if (o == t) continue;
      pieces(t, o, false, snd[std::size_t(o)]);
      pieces(o, t, true, rcv[std::size_t(o)]);
    }
    // two patches along a direction of 6 or 7 points: the face halo point n + 3 of the east (north) patch is its
    // own face 3
    pieces(t, t, false, self_src);
    pieces(t, t, true, self_dst);
  }
  // the points the worker sends itself are copied after the received ones are in place (they are points it owns:
  // no received box writes them)
  return make_plan(kWrap, snd, rcv, self_src, self_dst);
}
void wrap_hook(const void* const* bases, int n, int axis) {
  if (!W) return;
  const std::vector<int> idx = entries_of(bases, n);
  std::vector<long long> key{1, axis};
  key.insert(key.end(), idx.begin(), idx.end());
  exchange_planned(std::move(key), [&] { return wrap_plan(idx, axis); });
}

// One line of a field from the other side of a periodic direction with several patches along it (hooks.line): the
// worker that owns the line `dst` receives, for the tangential positions it owns in [t0, t1), the line `src` from
// the worker that owns it (the same row or column of patches: it owns the same tangential positions).
bool line_hook(const void* data, int axis, int dst, int src, int t0, int t1) {
  if (!W) return false;
  const int i = index_of(data, true);
  const Entry& e = W->reg.e[std::size_t(i)];
  if (!domain_field(e)) throw std::logic_error("decomposition: a line of " + e.name + ", not a field of a decomposed domain");
  const Layout& L = e.L;
  const Decomp& D = decomp(L);
  if (!(axis == 0 ? D.per_x && W->px > 1 : D.per_y && W->py > 1)) return false;
  const int t = W->id;
  auto along = [&](int w) { return axis == 0 ? own_x(D, w, L) : own_y(D, w, L); };
  auto across = [&](int w) { return meet(axis == 0 ? own_y(D, w, L) : own_x(D, w, L), Iv{t0, t1}); };
  auto tang = [&](int w) { return axis == 0 ? w / W->px : w % W->px; };
  auto has = [](Iv v, int p) { return p >= v.lo && p < v.hi; };
  std::vector<std::vector<Piece>> snd(std::size_t(W->n)), rcv(std::size_t(W->n));
  auto piece = [&](int line, Iv tr) { return axis == 0 ? Piece{i, Iv{line, line + 1}, tr} : Piece{i, tr, Iv{line, line + 1}}; };
  for (int o = 0; o < W->n; ++o) {
    if (o == t || tang(o) != tang(t)) continue;
    // this worker sends the line src to o, which owns dst
    if (has(along(t), src) && has(along(o), dst) && !across(o).empty()) snd[std::size_t(o)].push_back(piece(src, across(o)));
    if (has(along(o), src) && has(along(t), dst) && !across(t).empty()) rcv[std::size_t(o)].push_back(piece(dst, across(t)));
  }
  std::vector<Piece> none, none2;
  exchange_planned(std::vector<long long>{2, axis, dst, src, t0, t1, i},
                   [&] { return make_plan(kLine, snd, rcv, none, none2); });
  return true;
}

// every worker's n bytes at v, in worker order (a collective): one allgather of the transport (MPI_Allgather), each
// worker's value behind a header that names the collective; else (tiles) a message to every worker
std::vector<char> all_values_impl(Kind kind, const void* v, std::size_t n);
void no_open_exchange() {
  if (W && W->halo && W->halo->pending)
    throw std::logic_error("decomposition: a collective while a halo exchange is open (FillAsync, core/halo.hpp)");
}
std::vector<char> all_values(Kind kind, const void* v, std::size_t n) {
  no_open_exchange();
  if (!halo_stats()) return all_values_impl(kind, v, n);
  const double t0 = now_s();
  std::vector<char> r = all_values_impl(kind, v, n);
  PlanTimes& T = halo().others[unsigned(kind)];
  ++T.n;
  T.xfer += now_s() - t0;
  return r;
}
std::vector<char> all_values_impl(Kind kind, const void* v, std::size_t n) {
  std::vector<char> all(std::size_t(W->n) * n);
  if (n) std::memcpy(all.data() + std::size_t(W->id) * n, v, n);
  {
    if (trace())
      std::fprintf(stderr, "tile %d: collective %u kind %u allgather, %zu bytes\n", W->id, W->seq, unsigned(kind), n);
    std::vector<char> mine(kHdr + n), got(std::size_t(W->n) * (kHdr + n));
    const Hdr h{W->seq, kind, n};
    std::memcpy(mine.data(), &h, sizeof h);
    if (n) std::memcpy(mine.data() + kHdr, v, n);
    if (W->comm->allgather(mine.data(), mine.size(), got.data())) {
      for (int o = 0; o < W->n; ++o) {
        if (o == W->id) continue;
        const char* m = got.data() + std::size_t(o) * (kHdr + n);
        Hdr r{};
        std::memcpy(&r, m, sizeof r);
        if (r.seq != W->seq || r.kind != kind || r.bytes != n) {
          char b[240];
          std::snprintf(b, sizeof b,
                        "decomposition: worker %d expected collective %u of kind %u with %zu bytes from worker %d and "
                        "got collective %u of kind %u with %llu bytes (the workers' calls differ)",
                        W->id, W->seq, unsigned(kind), n, o, r.seq, r.kind, static_cast<unsigned long long>(r.bytes));
          throw std::logic_error(b);
        }
        if (n) std::memcpy(all.data() + std::size_t(o) * n, m + kHdr, n);
      }
      end_collective();
      return all;
    }
  }
  for (int o = 0; o < W->n; ++o)
    if (o != W->id) send_msg(o, kind, static_cast<const char*>(v), n);
  for (int o = 0; o < W->n; ++o)
    if (o != W->id) {
      const std::vector<char> m = recv_msg(o, kind, n);
      if (n) std::memcpy(all.data() + std::size_t(o) * n, m.data(), n);
    }
  end_collective();
  return all;
}

// every worker's value on every worker
template <class T>
std::vector<T> share(T v, Kind kind) {
  const std::vector<char> b = all_values(kind, &v, sizeof v);
  std::vector<T> all(std::size_t(W->n), v);
  for (int o = 0; o < W->n; ++o) std::memcpy(&all[std::size_t(o)], b.data() + std::size_t(o) * sizeof v, sizeof v);
  return all;
}

double max_hook(double v) {
  if (!W) return v;
  const std::vector<double> all = share(v, kMax);
  double m = all[0];
  for (int s = 1; s < W->n; ++s) m = all[std::size_t(s)] > m ? all[std::size_t(s)] : m;
  return m;
}

// Reducer::sum of a run with one patch over the range r of the values v (k slowest, i fastest): the CPU build adds
// each level's values in (j, i) order and then the levels; the fast CUDA FP32 build each level's values in 256 strided
// partial sums combined by a tree (core/backend.hpp kred), then the levels; the CUDA FP64 and FP32 identity builds as
// the CPU
double ordered_sum(const Range3D& r, const std::vector<double>& v) {
  const std::size_t ni = std::size_t(r.i1 - r.i0), nj = std::size_t(r.j1 - r.j0), n = ni * nj;
  double acc = 0.0;
  for (int k = r.k0; k < r.k1; ++k) {
    const double* p = v.data() + std::size_t(k - r.k0) * n;
#if defined(CPPWRF_USE_GPU) && !defined(CPPWRF_FP32_IDENTITY) && !defined(CPPWRF_DOUBLE)
    double sh[256];
    for (std::size_t t = 0; t < 256; ++t) {
      double a = 0.0;
      for (std::size_t q = t; q < n; q += 256) a = a + p[q];
      sh[t] = a;
    }
    for (int s = 128; s > 0; s >>= 1)
      for (int t = 0; t < s; ++t) sh[t] = sh[t] + sh[t + s];
    acc = acc + sh[0];
#else
    double a = 0.0;
    for (std::size_t q = 0; q < n; ++q) a = a + p[q];
    acc = acc + a;
#endif
  }
  return acc;
}

double sum_hook(const Range3D& r, const Range3D& own, const double* v) {
  const std::size_t nown = std::size_t(std::max(own.i1 - own.i0, 0)) * std::size_t(std::max(own.j1 - own.j0, 0)) *
                           std::size_t(std::max(own.k1 - own.k0, 0));
  if (!W) return 0.0;
  double result = 0.0;
  if (W->id != 0) {
    std::vector<char> m(sizeof(Range3D) + nown * sizeof(double));
    std::memcpy(m.data(), &own, sizeof own);
    if (nown) std::memcpy(m.data() + sizeof own, v, nown * sizeof(double));
    W->comm->send(0, [&] {
      std::vector<char> f(sizeof(Hdr) + m.size());
      const Hdr h{W->seq, kSum, m.size()};
      std::memcpy(f.data(), &h, sizeof h);
      std::memcpy(f.data() + sizeof h, m.data(), m.size());
      return f;
    }());
    const std::vector<char> back = recv_msg(0, kSum, sizeof(double));
    std::memcpy(&result, back.data(), sizeof result);
  } else {
    const std::size_t ni = std::size_t(std::max(r.i1 - r.i0, 0)), nj = std::size_t(std::max(r.j1 - r.j0, 0));
    std::vector<double> all(ni * nj * std::size_t(std::max(r.k1 - r.k0, 0)), 0.0);
    auto place = [&](const Range3D& o, const double* p) {
      const std::size_t oi = std::size_t(o.i1 - o.i0);
      for (int k = o.k0; k < o.k1; ++k)
        for (int j = o.j0; j < o.j1; ++j)
          std::memcpy(&all[(std::size_t(k - r.k0) * nj + std::size_t(j - r.j0)) * ni + std::size_t(o.i0 - r.i0)],
                      p + (std::size_t(k - o.k0) * std::size_t(o.j1 - o.j0) + std::size_t(j - o.j0)) * oi, oi * sizeof(double));
    };
    if (nown) place(own, v);
    for (int s = 1; s < W->n; ++s) {
      std::vector<char> m = W->comm->recv(s);
      Hdr h{};
      if (m.size() >= sizeof h) std::memcpy(&h, m.data(), sizeof h);
      if (m.size() < sizeof h + sizeof(Range3D) || h.seq != W->seq || h.kind != kSum)
        throw std::logic_error("decomposition: the workers' calls differ (a sum)");
      Range3D o;
      std::memcpy(&o, m.data() + sizeof h, sizeof o);
      if (o.i1 > o.i0 && o.j1 > o.j0 && o.k1 > o.k0)
        place(o, reinterpret_cast<const double*>(m.data() + sizeof h + sizeof o));
    }
    result = ordered_sum(r, all);
    for (int s = 1; s < W->n; ++s) send_msg(s, kSum, reinterpret_cast<const char*>(&result), sizeof result);
  }
  end_collective();
  return result;
}

std::string value_str(const char* p, int esz) {
  char b[64];
  if (esz == 8) {
    double v;
    std::memcpy(&v, p, 8);
    std::snprintf(b, sizeof b, "%.17g", v);
  } else {
    std::int32_t iv;
    float fv;
    std::memcpy(&iv, p, 4);
    std::memcpy(&fv, p, 4);
    std::snprintf(b, sizeof b, "%.9g (int %d)", double(fv), int(iv));
  }
  return b;
}

// the first point of the box x * y of the worker's field `mine` that differs from the reference run's field
// (same field, the whole domain); empty: none
std::string first_difference(const Entry& mine, const Entry& ref, Iv x, Iv y) {
  const Layout& L = mine.L;
  const Layout& R = ref.L;
  const int nk = levels(L);
  const std::size_t esz = std::size_t(mine.esz), bytes = std::size_t(x.n()) * esz;
  for (int k = 0; k < nk; ++k)
    for (int j = y.lo; j < y.hi; ++j) {
      const int jj = two_d(L) ? 0 : j;
      const char* a = mine.base + (L.offset + x.lo + jj * L.sj + k * L.sk) * std::ptrdiff_t(esz);
      const char* b = ref.base + (R.offset + x.lo + jj * R.sj + k * R.sk) * std::ptrdiff_t(esz);
      if (std::memcmp(a, b, bytes) == 0) continue;
      for (int ii = 0; ii < x.n(); ++ii) {
        const char* pa = a + std::size_t(ii) * esz;
        const char* pb = b + std::size_t(ii) * esz;
        if (std::memcmp(pa, pb, esz) == 0) continue;
        char h[200];
        std::snprintf(h, sizeof h, "%s (i, j, k) = (%d, %d, %d) from 0: ", mine.name.c_str(), x.lo + ii, j, k);
        return h + value_str(pa, mine.esz) + " (tile " + std::to_string(W->id) + "), " + value_str(pb, mine.esz) +
               " (reference)";
      }
    }
  return "";
}

// CPPWRF_TILE_CHECK: after every launch of a step, each tile's points against the reference run
void check_hook(const char* what) {
  if (!S || !S->o.check) return;
  ++n_checks;
  S->all.wait();
  if (W) {
    const Registry& r = *S->ref;
    const Registry* reg = &W->reg;
    std::string msg;
    if (r.e.size() != reg->e.size()) {
      msg = "the fields allocated differ from the reference run's";
    } else {
      for (std::size_t i = 0; i < reg->e.size() && msg.empty(); ++i) {
        const Entry& mine = reg->e[i];
        const Entry& ref = r.e[i];
        if (mine.live != ref.live || (mine.live && (mine.L.nx != ref.L.nx || mine.L.ny != ref.L.ny || mine.L.nz != ref.L.nz))) {
          msg = "the fields allocated differ from the reference run's (" + mine.name + ")";
          break;
        }
        if (!domain_field(mine)) continue;
        const Decomp& D = decomp(mine.L);
        msg = first_difference(mine, ref, meet(own_x(D, W->id, mine.L), win_x(D, W->id, mine.L)),
                               meet(own_y(D, W->id, mine.L), win_y(D, W->id, mine.L)));
      }
    }
    // the fields whose values around the patch are stale now (the inputs of the next launch)
    std::vector<char> stale(reg->e.size(), 0);
    for (std::size_t i = 0; i < reg->e.size() && i < r.e.size(); ++i) {
      const Entry& mine = reg->e[i];
      const Entry& ref = r.e[i];
      if (!domain_field(mine) || !ref.live) continue;
      const Decomp& D = decomp(mine.L);
      if (!first_difference(mine, ref, win_x(D, W->id, mine.L), win_y(D, W->id, mine.L)).empty()) stale[i] = 1;
    }
    if (!msg.empty()) {
      std::string names;
      int nn = 0;
      for (std::size_t i = 0; i < stale_before.size() && i < reg->e.size(); ++i)
        if (stale_before[i] && nn++ < 80) names += (names.empty() ? "" : ", ") + reg->e[i].name;
      msg += "; stale around tile " + std::to_string(W->id) + " before the launch: " + (names.empty() ? "none" : names);
    }
    stale_before.swap(stale);
    if (!msg.empty()) {
      std::lock_guard<std::mutex> lk(S->chk_m);
      if (S->chk_msg.empty())
        S->chk_msg = "tile check: " + msg + " after launch " + std::to_string(n_checks) + " of the run (" + what + ")";
    }
  }
  S->all.wait();
  if (!S->chk_msg.empty()) {
    if (W && W->id == 0) {
      std::fprintf(stderr, "cppwrf: %s\n", S->chk_msg.c_str());
      if (const char* f = std::getenv("CPPWRF_TILE_CHECK_LOG"); f && *f)
        if (std::FILE* fp = std::fopen(f, "a")) {
          std::fprintf(fp, "%s\n", S->chk_msg.c_str());
          std::fclose(fp);
        }
    }
    throw std::runtime_error(S->chk_msg);
  }
}

void fail(std::exception_ptr e, bool root) {
  {
    std::lock_guard<std::mutex> lk(S->err_m);
    if (!S->err || (root && !S->err_root)) {
      S->err = e;
      S->err_root = root;
    }
  }
  S->all.cancel();
  if (S->cancel_net) S->cancel_net();
}

std::vector<int> parse_ints(const std::string& s) {
  std::vector<int> v;
  std::size_t p = 0;
  while (p < s.size()) {
    std::size_t q = s.find(',', p);
    if (q == std::string::npos) q = s.size();
    v.push_back(std::stoi(s.substr(p, q - p)));
    p = q + 1;
  }
  return v;
}

}  // namespace

int rank_device(const char* spec, int local, int count) {
  if (!spec || !*spec) return count > 1 ? local % count : -1;
  const std::string s(spec);
  std::vector<int> d;
  for (std::size_t p = 0;;) {
    std::size_t q = s.find(',', p);
    if (q == std::string::npos) q = s.size();
    const std::string e = s.substr(p, q - p);
    if (e.empty() || e.size() > 9 || e.find_first_not_of("0123456789") != std::string::npos)
      throw std::runtime_error("CPPWRF_RANK_DEVICES=" + s + ": the entries are CUDA device numbers (non-negative "
                               "integers separated by commas), not '" + e + "'");
    d.push_back(std::stoi(e));
    if (q == s.size()) break;
    p = q + 1;
  }
  // only the entry this rank takes must be one of its devices (the ranks of other nodes may see more)
  const int v = d[std::size_t(local) % d.size()];
  if (count > 0 && v >= count)
    throw std::runtime_error("CPPWRF_RANK_DEVICES=" + s + ": device " + std::to_string(v) + " for node-local rank " +
                             std::to_string(local) + ", but the rank sees " + std::to_string(count) + " CUDA device" +
                             (count == 1 ? "" : "s"));
  return v;
}

namespace {

// ---- tiles' transport: a mailbox per (receiver, sender) ----
class ThreadNet {
 public:
  explicit ThreadNet(int n) : n_(n), box_(std::size_t(n) * std::size_t(n)) {}
  void send(int from, int to, std::vector<char>&& m) {
    Mail& b = box_[std::size_t(to) * std::size_t(n_) + std::size_t(from)];
    {
      std::lock_guard<std::mutex> lk(b.m);
      b.q.push_back(std::move(m));
    }
    b.cv.notify_one();
  }
  std::vector<char> recv(int me, int from) {
    Mail& b = box_[std::size_t(me) * std::size_t(n_) + std::size_t(from)];
    std::vector<char> m;
    bool got = false;
    {
      std::unique_lock<std::mutex> lk(b.m);
      if (!b.q.empty()) {
        m = std::move(b.q.front());
        b.q.pop_front();
        got = true;
      }
    }
    if (got) return m;
    // wait for the sender without the tiles' Serial lock (the sender may need it to get here)
    std::unique_lock<std::mutex>* ser = held_serial && held_serial->owns_lock() ? held_serial : nullptr;
    if (ser) ser->unlock();
    {
      std::unique_lock<std::mutex> lk(b.m);
      b.cv.wait(lk, [&] { return !b.q.empty() || cancel_.load(); });
      if (!b.q.empty()) {
        m = std::move(b.q.front());
        b.q.pop_front();
        got = true;
      }
    }
    if (ser) ser->lock();
    if (!got) throw Aborted();
    return m;
  }
  void cancel() {
    cancel_.store(true);
    for (Mail& b : box_) {
      std::lock_guard<std::mutex> lk(b.m);
      b.cv.notify_all();
    }
  }

 private:
  struct Mail {
    std::mutex m;
    std::condition_variable cv;
    std::deque<std::vector<char>> q;
  };
  int n_;
  std::vector<Mail> box_;
  std::atomic<bool> cancel_{false};
};
class ThreadComm : public Comm {
 public:
  ThreadComm(ThreadNet& net, int me) : net_(net), me_(me) {}
  void send(int to, std::vector<char>&& m) override { net_.send(me_, to, std::move(m)); }
  std::vector<char> recv(int from) override { return net_.recv(me_, from); }
#if defined(CPPWRF_USE_GPU)
  // CPPWRF_TILE_DEVICE_BUFFERS=1 (a check of the halo exchanges' device-buffer path, which a GPU-aware MPI takes, on
  // tiles): the messages leave from and arrive in the device buffers, copied through the mailboxes
  bool device_buffers() const override {
    static const bool on = [] {
      const char* e = std::getenv("CPPWRF_TILE_DEVICE_BUFFERS");
      return e && *e && *e != '0';
    }();
    return on;
  }
  void exchange(const std::vector<Post>& out, const std::vector<Post>& in, bool device) override {
    if (!device) {
      Comm::exchange(out, in, false);
      return;
    }
    for (const Post& m : out) {
      std::vector<char> b(m.bytes);
      if (m.bytes && gpuMemcpy(b.data(), m.p, m.bytes, gpuMemcpyDefault) != gpuSuccess)  // (a header: host memory)
        throw std::runtime_error("tiles: a device-buffer message (D2H)");
      send(m.peer, std::move(b));
    }
    for (const Post& m : in) {
      const std::vector<char> r = recv(m.peer);
      if (r.size() != m.bytes) throw std::logic_error("decomposition: a halo message of another size (the workers' calls differ)");
      if (m.bytes && gpuMemcpy(m.p, r.data(), m.bytes, gpuMemcpyDefault) != gpuSuccess)
        throw std::runtime_error("tiles: a device-buffer message (H2D)");
    }
  }
#endif

 private:
  ThreadNet& net_;
  int me_;
};

void set_ctx(const Grid& g) {
  ctx.on = ctx.patch = g.patched();
  if (!ctx.on) return;
  ctx.i0 = g.pi0 == 0 ? INT_MIN : g.pi0;
  ctx.i1 = g.pi1 == g.nx ? INT_MAX : g.pi1;
  ctx.j0 = (g.ny == 1 || g.pj0 == 0) ? INT_MIN : g.pj0;
  ctx.j1 = (g.ny == 1 || g.pj1 == g.ny) ? INT_MAX : g.pj1;
}

void set_hooks(bool multi) {
  hooks.multi = multi;
  hooks.max = &max_hook;
  hooks.exchange = &exchange_hook;
  hooks.exchange_begin = &exchange_begin_hook;
  hooks.exchange_finish = &exchange_finish_hook;
  hooks.wraps = &wraps_hook;
  hooks.wrap = &wrap_hook;
  hooks.sum = &sum_hook;
  hooks.line = &line_hook;
  hooks.first_code = &first_code;
}

}  // namespace

namespace detail {
int run_worker(Comm& c, int id, const Options& o, const std::function<int(int)>& body) {
  Worker w;
  w.id = id;
  w.n = o.count();
  w.px = o.px;
  w.py = o.py;
  w.comm = &c;
  w.dev = current_device();
  W = &w;
  track = &kTrack;
  struct Reset {
    ~Reset() {
      if (W && W->halo) {
        if (halo_stats())
        {
          std::fprintf(stderr, "cppwrf halo: worker %d: %ld exchanges, %ld plans, %ld messages sent, %.1f MiB sent\n",
                       W->id, W->halo->exchanges, W->halo->plans_made, W->halo->messages, W->halo->sent / 1048576.0);
          print_times(W->id, *W->halo);
        }
        delete W->halo;
        W->halo = nullptr;
      }
#if defined(CPPWRF_USE_GPU)
      if (W && W->dbuf) dev_free(W->dbuf);
#endif
      track = nullptr;
      W = nullptr;
      ctx = Ctx{};
    }
  } reset;
  return body(id);
}
}  // namespace detail

Options options(const std::string& run_tiles) {
  Options o;
  std::string t = run_tiles;
  if (const char* e = std::getenv("CPPWRF_TILES"); e && *e) t = e;
  int px = 1, py = 1;
  char c = 0;
  if (t.empty() || std::sscanf(t.c_str(), "%d%c%d", &px, &c, &py) < 1) px = 1;
  if (c == 0) py = 1;
  else if (c != 'x' && c != 'X') throw std::runtime_error("run.tiles: P or PxQ (tiles along x and y), not '" + t + "'");
  if (px < 1 || py < 1) throw std::runtime_error("run.tiles: P or PxQ with P, Q >= 1, not '" + t + "'");
  o.px = px;
  o.py = py;
  if (const char* e = std::getenv("CPPWRF_TILE_DEVICES"); e && *e) o.devices = parse_ints(e);
  if (const char* e = std::getenv("CPPWRF_TILE_CHECK"); e && *e && *e != '0') o.check = true;
#if defined(CPPWRF_USE_GPU)
  if (o.check) throw std::runtime_error("CPPWRF_TILE_CHECK: the CPU build only");
#endif
  return o;
}

int id() { return W ? W->id : (ref_reg ? -1 : 0); }
int count() { return W ? W->n : 1; }
bool tiled() { return W != nullptr; }
bool checking() { return S && S->o.check; }
bool writer() { return !W || W->id == 0; }
int patches_x() { return W ? W->px : 1; }
int patches_y() { return W ? W->py : 1; }

void set_domain(Grid& g) {
  if (!W) return;
  const int px = W->px, py = W->py;
  const int nx = g.nx, ny = g.ny;
  if (ny == 1 && py > 1) throw std::runtime_error("run.tiles: a 2-D (x-z) domain has one patch along y");
  if ((px > 1 && nx / px < Grid::halo) || (py > 1 && ny / py < Grid::halo))
    throw std::runtime_error("run.tiles: the patches must be at least " + std::to_string(Grid::halo) +
                             " points wide (" + std::to_string(nx) + " x " + std::to_string(ny) + " points in " +
                             std::to_string(px) + " x " + std::to_string(py) + " patches)");
  Decomp D;
  D.nx = nx;
  D.ny = ny;
  D.per_x = g.bc_x == BC::Periodic;
  D.per_y = g.bc_y == BC::Periodic;
  D.patch.resize(std::size_t(W->n));
  for (int t = 0; t < W->n; ++t) {
    const int tx = t % px, ty = t / px;
    D.patch[std::size_t(t)] = PatchBox{int(long(tx) * nx / px), int(long(tx + 1) * nx / px), int(long(ty) * ny / py),
                                       int(long(ty + 1) * ny / py)};
  }
  g.dom = int(W->doms.size());
  const PatchBox& p = D.patch[std::size_t(W->id)];
  g.pi0 = p.i0;
  g.pi1 = p.i1;
  g.pj0 = p.j0;
  g.pj1 = p.j1;
  W->doms.push_back(std::move(D));
  set_ctx(g);
}

PatchBox patch(int dom, int t) {
  if (!W || dom < 0 || dom >= int(W->doms.size())) throw std::logic_error("decomposition: no such domain");
  return W->doms[std::size_t(dom)].patch[std::size_t(t)];
}

Use::Use(const Grid& g) : saved_(ctx) {
  if (W) set_ctx(g);
}
Use::~Use() { ctx = saved_; }

Step::Step() { in_step = true; }
Step::~Step() { in_step = false; }

void exchange_all() {
  if (!W) return;
  std::vector<int> idx;
  for (std::size_t i = 0; i < W->reg.e.size(); ++i)
    if (domain_field(W->reg.e[i])) idx.push_back(int(i));
  exchange_entries(idx, false);  // once, at the start: the plan is not kept
}

Layout whole_layout(const Layout& L) {
  Layout w = L;
  if (L.si == 0) return w;
  w.dom = -1;
  w.wi0 = -Grid::halo;
  w.wi1 = L.nx + Grid::halo;
  w.wj0 = -L.hy;
  w.wj1 = L.ny + L.hy;
  const std::ptrdiff_t pitch = (Layout::lpad + L.nx + Grid::halo + 31) / 32 * 32;
  const std::ptrdiff_t plane = pitch * (L.ny + 2 * L.hy);
  w.sj = L.sj ? pitch : 0;
  w.sk = L.sk ? plane : 0;
  w.offset = Layout::lpad + L.hy * w.sj;
  w.total = std::size_t(plane) * std::size_t(L.sk ? L.nz : 1);
  return w;
}

namespace {
Piece own_piece(const Entry& e, int i, int s) {
  const Decomp& D = decomp(e.L);
  return Piece{i, meet(own_x(D, s, e.L), win_x(D, s, e.L)), meet(own_y(D, s, e.L), win_y(D, s, e.L))};
}
// the points of the domain (no lateral halo) worker s holds: its own and its halo's
Piece inner_piece(const Entry& e, int i, int s) {
  const Decomp& D = decomp(e.L);
  return Piece{i, meet(win_x(D, s, e.L), Iv{0, e.L.nx}), meet(win_y(D, s, e.L), Iv{0, two_d(e.L) ? 1 : e.L.ny})};
}
void request(ServiceOp op, int entry) {
  const std::int32_t r[2] = {op, entry};
  for (int o = 1; o < W->n; ++o) send_msg(o, kService, reinterpret_cast<const char*>(r), sizeof r);
}
// a worker's part of a download: its points of field i to the writer
void download_send(int i) {
  const Entry& e = W->reg.e[std::size_t(i)];
  dev_sync();
  std::vector<Piece> mine{own_piece(e, i, W->id)};
  const std::size_t nb = layout_pieces(mine);
  std::vector<char> buf(nb);
  move_pieces(mine, buf.data(), nb, false);
  send_msg(0, kDownload, buf.data(), nb);
}
void upload_recv(int i) {
  const Entry& e = W->reg.e[std::size_t(i)];
  std::vector<Piece> mine{inner_piece(e, i, W->id)};
  const std::size_t nb = layout_pieces(mine);
  std::vector<char> m = recv_msg(0, kUpload, nb);
  move_pieces(mine, m.data(), nb, true);
  dev_sync();
}
void check_domain_field(const Entry& e, const char* what) {
  if (!domain_field(e)) throw std::logic_error(std::string("decomposition: ") + what + " of " + e.name + ", not a field of a decomposed domain");
}
}  // namespace

namespace {
// a worker's piece p of a field (message bytes m) into the whole field h (layout whole)
void place_piece(const Layout& L, const Layout& whole, std::size_t esz, const Piece& p, const char* m, std::vector<char>& h) {
  const std::size_t w = std::size_t(p.x.n()) * esz;
  for (int k = 0; k < levels(L); ++k)
    for (int j = p.y.lo; j < p.y.hi; ++j) {
      std::memcpy(h.data() + (whole.offset + p.x.lo + (two_d(L) ? 0 : j) * whole.sj + k * whole.sk) * std::ptrdiff_t(esz), m, w);
      m += w;
    }
}
}  // namespace

void download(const void* base, std::vector<char>& h, Layout& whole) {
  if (!W) throw std::logic_error("decomposition: tile::download without a decomposition");
  const int i = index_of(base);
  const Entry& e = W->reg.e[std::size_t(i)];
  check_domain_field(e, "download");
  const Layout& L = e.L;
  whole = whole_layout(L);
  h.clear();
  if (W->service) request(kOpDownload, i);
  if (W->id != 0) {
    download_send(i);
  } else {
    dev_sync();
    std::vector<Piece> mine{own_piece(e, i, 0)};
    const std::size_t nb = layout_pieces(mine);
    std::vector<char> buf(nb);
    move_pieces(mine, buf.data(), nb, false);
    const std::size_t esz = std::size_t(e.esz);
    h.assign(whole.total * esz, 0);
    place_piece(L, whole, esz, mine[0], buf.data(), h);
    for (int s = 1; s < W->n; ++s) {
      std::vector<Piece> ps{own_piece(e, i, s)};
      const std::size_t n = layout_pieces(ps);
      const std::vector<char> m = recv_msg(s, kDownload, n);
      place_piece(L, whole, esz, ps[0], m.data(), h);
    }
  }
  end_collective();
}

void gather(const void* base, std::vector<char>& h, Layout& whole) {
  if (!W) throw std::logic_error("decomposition: tile::gather without a decomposition");
  if (W->service) throw std::logic_error("decomposition: tile::gather inside a tile::Service scope");
  const int i = index_of(base);
  const Entry& e = W->reg.e[std::size_t(i)];
  check_domain_field(e, "gather");
  const Layout& L = e.L;
  whole = whole_layout(L);
  dev_sync();
  std::vector<Piece> mine{own_piece(e, i, W->id)};
  const std::size_t nb = layout_pieces(mine);
  std::vector<char> buf(nb);
  move_pieces(mine, buf.data(), nb, false);
  for (int o = 0; o < W->n; ++o)
    if (o != W->id) send_msg(o, kGather, buf.data(), nb);
  const std::size_t esz = std::size_t(e.esz);
  h.assign(whole.total * esz, 0);
  place_piece(L, whole, esz, mine[0], buf.data(), h);
  for (int s = 0; s < W->n; ++s) {
    if (s == W->id) continue;
    std::vector<Piece> ps{own_piece(e, i, s)};
    const std::size_t n = layout_pieces(ps);
    const std::vector<char> m = recv_msg(s, kGather, n);
    place_piece(L, whole, esz, ps[0], m.data(), h);
  }
  end_collective();
}

namespace {
// the layout of the box b of a field with layout L (inside the whole field's allocation), under the field's indices
Layout box_layout(const Layout& L, const PatchBox& b) {
  Layout w = L;
  w.dom = -1;
  w.wi0 = std::max(b.i0, -Grid::halo);
  w.wi1 = std::max(w.wi0, std::min(b.i1, L.nx + Grid::halo));
  if (two_d(L)) {
    w.wj0 = 0;
    w.wj1 = 1;
  } else {
    w.wj0 = std::max(b.j0, -L.hy);
    w.wj1 = std::max(w.wj0, std::min(b.j1, L.ny + L.hy));
  }
  const std::ptrdiff_t pitch = (Layout::lpad + (w.wi1 - w.wi0) + 31) / 32 * 32;
  const std::ptrdiff_t plane = pitch * (w.wj1 - w.wj0);
  w.sj = L.sj ? pitch : 0;
  w.sk = L.sk ? plane : 0;
  w.offset = Layout::lpad - w.wi0 - (two_d(L) ? 0 : w.wj0) * w.sj;
  w.total = std::size_t(plane) * std::size_t(L.sk ? L.nz : 1);
  return w;
}
// the part of worker s's own piece of entry e inside the box layout B
Piece box_piece(const Entry& e, int i, int s, const Layout& B) {
  Piece p = own_piece(e, i, s);
  p.x = meet(p.x, Iv{B.wi0, B.wi1});
  if (!two_d(e.L)) p.y = meet(p.y, Iv{B.wj0, B.wj1});
  if (p.x.hi < p.x.lo) p.x.hi = p.x.lo;
  if (p.y.hi < p.y.lo) p.y.hi = p.y.lo;
  return p;
}
}  // namespace

void gather_box(const void* base, const std::function<PatchBox(int)>& box, std::vector<char>& h, Layout& mine_l) {
  if (!W) throw std::logic_error("decomposition: tile::gather_box without a decomposition");
  if (W->service) throw std::logic_error("decomposition: tile::gather_box inside a tile::Service scope");
  const int i = index_of(base);
  const Entry& e = W->reg.e[std::size_t(i)];
  check_domain_field(e, "gather_box");
  const Layout& L = e.L;
  dev_sync();
  for (int o = 0; o < W->n; ++o) {  // this worker's points in every other worker's box
    if (o == W->id) continue;
    std::vector<Piece> ps{box_piece(e, i, W->id, box_layout(L, box(o)))};
    const std::size_t nb = layout_pieces(ps);
    if (ps[0].x.n() == 0 || ps[0].y.n() == 0) continue;
    std::vector<char> buf(nb);
    move_pieces(ps, buf.data(), nb, false);
    send_msg(o, kGather, buf.data(), nb);
  }
  mine_l = box_layout(L, box(W->id));
  const std::size_t esz = std::size_t(e.esz);
  h.assign(mine_l.total * esz, 0);
  for (int s = 0; s < W->n; ++s) {
    std::vector<Piece> ps{box_piece(e, i, s, mine_l)};
    const std::size_t nb = layout_pieces(ps);
    if (ps[0].x.n() == 0 || ps[0].y.n() == 0) continue;
    if (s == W->id) {
      std::vector<char> buf(nb);
      move_pieces(ps, buf.data(), nb, false);
      place_piece(L, mine_l, esz, ps[0], buf.data(), h);
    } else {
      const std::vector<char> m = recv_msg(s, kGather, nb);
      place_piece(L, mine_l, esz, ps[0], m.data(), h);
    }
  }
  end_collective();
}

WholeCopy::~WholeCopy() { dev_free(buf_); }
void WholeCopy::get(const void* base, const Layout& L, const std::function<PatchBox(int)>& box) {
  if (!W || L.si == 0) return;
  std::vector<char> h;
  gather_box(base, box, h, L_);
  if (h.size() != bytes_) {
    dev_free(buf_);
    buf_ = static_cast<char*>(dev_alloc(std::max<std::size_t>(h.size(), 8)));
    bytes_ = h.size();
  }
  if (!h.empty()) copy_to_dev(buf_, h.data(), h.size());
  esz_ = L_.total ? h.size() / L_.total : 0;
  on_ = true;
}
void WholeCopy::get(const void* base, const Layout& L) {
  if (!W || L.si == 0) return;
  std::vector<char> h;
  gather(base, h, L_);
  if (h.size() != bytes_) {
    dev_free(buf_);
    buf_ = static_cast<char*>(dev_alloc(h.size()));
    bytes_ = h.size();
  }
  copy_to_dev(buf_, h.data(), h.size());
  esz_ = L_.total ? h.size() / L_.total : 0;
  on_ = true;
}

int field_index(const void* base) {
  if (!W) throw std::logic_error("decomposition: tile::field_index without a decomposition");
  return index_of(base);
}
FieldRef field_at(int index) {
  if (!W || index < 0 || index >= int(W->reg.e.size()) || !W->reg.e[std::size_t(index)].live)
    throw std::logic_error("decomposition: tile::field_at of no live field");
  const Entry& e = W->reg.e[std::size_t(index)];
  return FieldRef{e.base, e.L, e.esz};
}
void exchange_base(const void* base) {
  if (!W) return;
  exchange_entries({index_of(base)});
}

void upload(const void* base, const char* h, const Layout& whole) {
  if (!W) throw std::logic_error("decomposition: tile::upload without a decomposition");
  if (!W->service) throw std::logic_error("decomposition: tile::upload outside a tile::Service scope");
  const int i = index_of(base);
  const Entry& e = W->reg.e[std::size_t(i)];
  check_domain_field(e, "upload");
  const Layout& L = e.L;
  const std::size_t esz = std::size_t(e.esz);
  request(kOpUpload, i);
  for (int s = 0; s < W->n; ++s) {
    std::vector<Piece> ps{inner_piece(e, i, s)};
    const std::size_t n = layout_pieces(ps);
    std::vector<char> m(n);
    char* q = m.data();
    const Piece& p = ps[0];
    const std::size_t w = std::size_t(p.x.n()) * esz;
    for (int k = 0; k < levels(L); ++k)
      for (int j = p.y.lo; j < p.y.hi; ++j) {
        std::memcpy(q, h + (whole.offset + p.x.lo + (two_d(L) ? 0 : j) * whole.sj + k * whole.sk) * std::ptrdiff_t(esz), w);
        q += w;
      }
    if (s == 0) {
      move_pieces(ps, m.data(), n, true);
      dev_sync();
    } else {
      send_msg(s, kUpload, m.data(), n);
    }
  }
  end_collective();
}

Service::Service() {
  if (!W) return;
  if (W->id == 0) {
    if (W->service) return;  // a nested scope
    W->service = true;
    opened_ = true;
    return;
  }
  served_ = true;
  for (;;) {
    const std::vector<char> m = recv_msg(0, kService, 2 * sizeof(std::int32_t));
    std::int32_t r[2];
    std::memcpy(r, m.data(), sizeof r);
    if (r[0] == kOpDownload) download_send(r[1]);
    else if (r[0] == kOpUpload) upload_recv(r[1]);
    end_collective();
    if (r[0] == kOpDone) break;
  }
}
Service::~Service() {
  if (!opened_ || !W) return;
  W->service = false;
  try {
    request(kOpDone, 0);
    end_collective();
  } catch (...) {  // the run is stopping
  }
}

double max(double v) { return max_hook(v); }

int first_code(int code) {
  if (!W) return code;
  const std::vector<int> all = share(code, kCode);
  int c = 0;
  for (int v : all)
    if (v != 0 && (c == 0 || v < c)) c = v;
  return c;
}

void max_each(double* v, std::size_t n) {
  if (!W) return;
  const std::vector<char> b = all_values(kMaxEach, v, n * sizeof(double));
  // the other workers' values in worker order, as they were received one by one
  for (int o = 0; o < W->n; ++o)
    if (o != W->id) {
      const char* m = b.data() + std::size_t(o) * n * sizeof(double);
      for (std::size_t q = 0; q < n; ++q) {
        double p;
        std::memcpy(&p, m + q * sizeof(double), sizeof p);
        v[q] = p > v[q] ? p : v[q];
      }
    }
}

PatchBox input_window(int nx, int ny) {
  if (!W) return PatchBox{0, nx, 0, ny};
  const int px = W->px, py = W->py, tx = W->id % px, ty = W->id / px;  // set_domain's patches
  const int i0 = int(long(tx) * nx / px), i1 = int(long(tx + 1) * nx / px);
  const int j0 = int(long(ty) * ny / py), j1 = int(long(ty + 1) * ny / py);
  return PatchBox{std::max(0, i0 - Grid::halo), std::min(nx, i1 + Grid::halo), std::max(0, j0 - Grid::halo),
                  std::min(ny, j1 + Grid::halo)};
}

PatchBox own_box(const Grid& g) {
  if (!g.patched()) return PatchBox{0, g.nx, 0, g.ny};
  return PatchBox{g.pi0, g.pi1, g.pj0, g.pj1};
}

void from_owner(bool have, void* v, std::size_t n) {
  if (!W) return;
  std::vector<char> m(1 + n);
  m[0] = have ? 1 : 0;
  std::memcpy(m.data() + 1, v, n);
  const std::vector<char> b = all_values(kOwner, m.data(), m.size());
  int owners = have ? 1 : 0;
  for (int o = 0; o < W->n; ++o)
    if (o != W->id) {
      const char* r = b.data() + std::size_t(o) * m.size();
      if (r[0]) {
        std::memcpy(v, r + 1, n);
        ++owners;
      }
    }
  if (owners != 1) throw std::logic_error("decomposition: a value with " + std::to_string(owners) + " owners");
}

void send(int to, const void* p, std::size_t n) {
  if (!W) throw std::logic_error("decomposition: tile::send without a decomposition");
  send_msg(to, kRaw, static_cast<const char*>(p), n);
}
void recv(int from, void* p, std::size_t n) {
  if (!W) throw std::logic_error("decomposition: tile::recv without a decomposition");
  const std::vector<char> m = recv_msg(from, kRaw, n);
  if (n) std::memcpy(p, m.data(), n);
}

Serial::Serial() {
  if (S && !held_serial) {  // a nested scope: the outer one holds the lock
    lk_ = std::unique_lock<std::mutex>(S->serial);
    held_serial = &lk_;
  }
}
Serial::~Serial() {
  if (held_serial == &lk_) held_serial = nullptr;
}
void Serial::unlock() {
  if (held_serial == &lk_) held_serial = nullptr;
  if (lk_.owns_lock()) lk_.unlock();
}
void Serial::lock() {
  if (!S || held_serial || !lk_.mutex()) return;
  lk_.lock();
  held_serial = &lk_;
}

int run(const Options& o, const std::function<int(int)>& body) {
  if (S || W) throw std::logic_error("tiles: a tiled run inside a decomposed run");
  Set set;
  set.o = o;
  set.n = o.count();
  Registry ref;
  set.ref = &ref;
  ThreadNet net(set.n);
  set.cancel_net = [&net] { net.cancel(); };
  std::vector<int> dev(std::size_t(set.n), current_device());
  if (!o.devices.empty())
    for (int t = 0; t < set.n; ++t) dev[std::size_t(t)] = o.devices[std::size_t(t) % o.devices.size()];
  set.all.init(set.n + (o.check ? 1 : 0));
  {  // the tiles on one device share its work budget as ranks do (device_work_budget, device_work_share); the
    // check's reference run works on this thread's device as well
    std::vector<int> on = dev;
    if (o.check) on.push_back(current_device());
    int most = 1;
    for (int d : on) most = std::max(most, int(std::count(on.begin(), on.end(), d)));
    set_device_work_share(most);
  }
  S = &set;
  set_hooks(true);
  if (o.check) hooks.check = &check_hook;
  int per = 1;
#if defined(_OPENMP)
  per = std::max(1, omp_get_max_threads() / (set.n + (o.check ? 1 : 0)));
#endif
  std::printf("cppwrf: %d x %d tiles%s, %d OpenMP threads each\n", o.px, o.py,
              o.check ? " and the check's reference run" : "", per);
  std::vector<int> rc(std::size_t(set.n), 0);
  auto thread_main = [&](int t) {
    try {
#if defined(CPPWRF_USE_GPU)
      if (t >= 0) {
        const gpuError_t e = gpuSetDevice(dev[std::size_t(t)]);
        if (e != gpuSuccess) throw std::runtime_error(std::string("tiles: gpuSetDevice: ") + gpuGetErrorString(e));
      }
#endif
#if defined(_OPENMP)
      omp_set_num_threads(per);
#endif
      int r = 0;
      if (t >= 0) {
        ThreadComm comm(net, t);
        r = detail::run_worker(comm, t, o, body);
        rc[std::size_t(t)] = r;
      } else {  // the check's reference run: one patch, its own fields
        ref_reg = &ref;
        track = &kTrack;
        r = body(t);
        track = nullptr;
      }
      if (r != 0) fail(std::make_exception_ptr(std::runtime_error("a tile stopped")), false);
    } catch (const Aborted&) {
      fail(std::current_exception(), false);
    } catch (...) {
      fail(std::current_exception(), true);
    }
    track = nullptr;
    ref_reg = nullptr;
    release_host_staging();  // the thread ends: its upload buffer is not kept for the process
  };
  std::vector<std::thread> th;
  for (int t = 0; t < set.n; ++t) th.emplace_back(thread_main, t);
  if (o.check) th.emplace_back(thread_main, -1);
  for (std::thread& x : th) x.join();
  hooks = Hooks{};
  S = nullptr;
  set_device_work_share(1);
  if (set.err && set.err_root) std::rethrow_exception(set.err);
  for (int r : rc)
    if (r != 0) return r;
  if (set.err) std::rethrow_exception(set.err);
  return rc[0];
}

#if !defined(CPPWRF_MPI)
bool mpi_start(int*, char***) { return false; }
int mpi_run(const std::string&, const std::string&, const std::function<int(int)>&) {
  throw std::logic_error("this build has no MPI (CPPWRF_MPI)");
}
void mpi_finish() {}
void mpi_abort(int) {}
#else
namespace detail {
// an MPI rank is the only worker of its process (CUDA graphs stay on: core/backend.cpp graphs_mode)
void install_hooks() { set_hooks(false); }
}  // namespace detail
#endif

}  // namespace cppwrf::tile
