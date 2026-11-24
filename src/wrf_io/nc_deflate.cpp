// SPDX-License-Identifier: GPL-3.0-or-later
#include "wrf_io/nc_deflate.hpp"

#include <zlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef CPPWRF_HAVE_HDF5
#include <dlfcn.h>
#include <hdf5.h>
#include <link.h>
#endif

namespace cppwrf::wrf_io::deflate {

struct Group {
  std::mutex m;
  std::condition_variable done;
  long pending = 0;
  std::atomic<long> precomputed{0};
};

std::shared_ptr<Group> new_group() { return std::make_shared<Group>(); }

namespace {

using clk = std::chrono::steady_clock;
double since(clk::time_point t0) { return std::chrono::duration<double>(clk::now() - t0).count(); }

// a precompressed chunk: the filter's input (shuffled chunk) and zlib's output
struct Entry {
  std::uint64_t key = 0;
  int level = 0;
  const Group* group = nullptr;  // clear(group) drops it
  std::vector<unsigned char> in, out;
};

// key of a filter input: its length and 64 samples of 32 bytes (the filter compares the whole buffer)
std::uint64_t sample_key(const unsigned char* p, std::size_t n) {
  std::uint64_t h = 1469598103934665603ULL ^ n;
  auto mix = [&](unsigned char c) { h = (h ^ c) * 1099511628211ULL; };
  const std::size_t span = 32, ns = 64;
  if (n <= span * ns) {
    for (std::size_t i = 0; i < n; ++i) mix(p[i]);
    return h;
  }
  const std::size_t stride = (n - span) / (ns - 1);
  for (std::size_t s = 0; s < ns; ++s)
    for (std::size_t i = 0; i < span; ++i) mix(p[s * stride + i]);
  return h;
}

class Pool {
 public:
  explicit Pool(int n) {
    for (int t = 0; t < n; ++t) std::thread([this] { run(); }).detach();  // live until the process ends
  }
  void submit(std::function<void()> f) {
    {
      std::lock_guard<std::mutex> lk(m_);
      q_.push_back(std::move(f));
    }
    cv_.notify_one();
  }

 private:
  void run() {
    for (;;) {
      std::function<void()> f;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [this] { return !q_.empty(); });
        f = std::move(q_.front());
        q_.pop_front();
      }
      f();
    }
  }
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::function<void()>> q_;
};

int pool_threads() {
  static const int n = [] {
    if (const char* e = std::getenv("CPPWRF_NC_THREADS")) return std::max(0, std::atoi(e));
    return int(std::clamp(std::thread::hardware_concurrency(), 1u, 64u));
  }();
  return n;
}

Pool& pool() {
  static Pool* p = new Pool(pool_threads());  // never destroyed: its threads may still wait at exit
  return *p;
}

std::mutex g_mutex;  // the results
std::unordered_multimap<std::uint64_t, std::unique_ptr<Entry>> g_ready;  // by sample_key
std::atomic<bool> g_installed{false};
// the counters: pool threads, the writing threads and the timing report use them
std::atomic<long> g_precomputed{0}, g_hits{0}, g_misses{0};
std::mutex g_time_mutex;
thread_local const Group* t_group = nullptr;  // UseGroup
double g_miss_s = 0, g_wait_s = 0, g_download_s = 0;
void add_time(double& t, double s) {
  std::lock_guard<std::mutex> lk(g_time_mutex);
  t += s;
}

// cut chunk c (index over the chunk grid, row-major) of r, pad with the fill value, shuffle as HDF5's shuffle filter
// (H5Zshuffle.c: byte j of element i goes to j * n + i, for elements of more than 1 byte and more than 1 element)
std::vector<unsigned char> cut_chunk(const Record& r, std::size_t c) {
  const std::size_t nd = r.len.size(), es = r.elsize;
  std::vector<std::size_t> nch(nd), org(nd);
  for (std::size_t d = 0; d < nd; ++d) nch[d] = (r.len[d] + r.chunk[d] - 1) / r.chunk[d];
  for (std::size_t d = nd; d-- > 0;) {
    org[d] = (c % nch[d]) * r.chunk[d];
    c /= nch[d];
  }
  std::size_t n = 1;
  for (std::size_t d = 0; d < nd; ++d) n *= r.chunk[d];
  std::vector<unsigned char> buf(n * es);
  const std::size_t row = r.chunk[nd - 1];
  const std::size_t valid = std::min(row, r.len[nd - 1] - org[nd - 1]);
  auto fill = [&](unsigned char* p, std::size_t cnt) {
    for (std::size_t e = 0; e < cnt; ++e) std::memcpy(p + e * es, r.fill.data(), es);
  };
  std::vector<std::size_t> idx(nd, 0);  // chunk-local index of the row (last entry unused)
  for (std::size_t rw = 0, nrows = n / row; rw < nrows; ++rw) {
    unsigned char* dst = buf.data() + rw * row * es;
    bool in = true;
    std::size_t src = 0;
    for (std::size_t d = 0; d + 1 < nd; ++d) {
      const std::size_t g = org[d] + idx[d];
      in = in && g < r.len[d];
      src = src * r.len[d] + g;
    }
    if (in) {
      src = src * r.len[nd - 1] + org[nd - 1];
      std::memcpy(dst, r.data + src * es, valid * es);
      fill(dst + valid * es, row - valid);
    } else {
      fill(dst, row);
    }
    for (std::size_t d = nd - 1; d-- > 0;) {  // next row
      if (++idx[d] < r.chunk[d]) break;
      idx[d] = 0;
    }
  }
  if (!r.shuffle || es <= 1 || n <= 1) return buf;
  std::vector<unsigned char> sh(buf.size());
  for (std::size_t j = 0; j < es; ++j) {
    unsigned char* o = sh.data() + j * n;
    const unsigned char* s = buf.data() + j;
    for (std::size_t i = 0; i < n; ++i) o[i] = s[i * es];
  }
  return sh;
}

void compress_chunk(const Record& r, std::size_t c, Group& g) {
  auto e = std::make_unique<Entry>();
  e->level = r.level;
  e->group = &g;
  e->in = cut_chunk(r, c);
  e->key = sample_key(e->in.data(), e->in.size());
  uLongf n = compressBound(uLong(e->in.size()));
  e->out.resize(n);
  if (compress2(e->out.data(), &n, e->in.data(), uLong(e->in.size()), r.level) != Z_OK) return;  // the filter does it
  e->out.resize(n);
  std::lock_guard<std::mutex> lk(g_mutex);
  const std::uint64_t key = e->key;
  g_ready.emplace(key, std::move(e));
  ++g_precomputed;
  ++g.precomputed;
}

#ifdef CPPWRF_HAVE_HDF5
// every shared library of the process that has an H5Z_register (its own or of a library it depends on) has this one:
// one HDF5 library in the process (two would each have their own filter table)
bool one_h5z_register(void* reg) {
  std::vector<std::string> names;  // (dlopen after the walk: not under the loader's lock of dl_iterate_phdr)
  dl_iterate_phdr(
      [](dl_phdr_info* info, std::size_t, void* data) {
        // (the program itself, without a name: RTLD_DEFAULT found reg there)
        if (info->dlpi_name && *info->dlpi_name) static_cast<std::vector<std::string>*>(data)->push_back(info->dlpi_name);
        return 0;
      },
      &names);
  for (const std::string& n : names)
    if (void* h = dlopen(n.c_str(), RTLD_LAZY | RTLD_NOLOAD)) {
      void* f = dlsym(h, "H5Z_register");
      dlclose(h);
      if (f && f != reg) return false;
    }
  return true;
}

// the precomputed output of a filter input, if there is one (removed from the results)
bool take(int level, const unsigned char* p, std::size_t n, std::vector<unsigned char>& out) {
  const std::uint64_t key = sample_key(p, n);
  std::lock_guard<std::mutex> lk(g_mutex);
  // the entries of this key only (equal inputs give equal outputs: any entry of the same bytes and level will do)
  // the writing thread's own group only (UseGroup): another file's record with the same values (wrfrst and the
  // history frame of the same time, another ensemble member's static fields) keeps its own entries for its own puts;
  // a thread without a group (a direct NcOut caller) takes any entry
  const auto [b, end] = g_ready.equal_range(key);
  for (int pass = 0; pass < 2; ++pass)
    for (auto it = b; it != end; ++it) {
      Entry& e = *it->second;
      if ((pass == 1 ? t_group == nullptr : e.group == t_group) && e.level == level && e.in.size() == n &&
          std::memcmp(e.in.data(), p, n) == 0) {
        out = std::move(e.out);
        g_ready.erase(it);
        return true;
      }
    }
  return false;
}

// HDF5's deflate filter (H5Zdeflate.c H5Z__filter_deflate, the same in HDF5 1.10.7 and 1.14.5) with the precomputed
// compression; the library's allocator through its public wrappers of H5MM_malloc / H5MM_realloc / H5MM_xfree
size_t filter(unsigned flags, size_t cd_nelmts, const unsigned cd_values[], size_t nbytes, size_t* buf_size, void** buf) {
  if (cd_nelmts != 1 || cd_values[0] > 9) return 0;  // invalid deflate aggression level
  if (flags & H5Z_FLAG_REVERSE) {  // uncompress
    size_t nalloc = *buf_size;
    void* outbuf = H5allocate_memory(nalloc, false);
    if (!outbuf) return 0;
    z_stream z;
    std::memset(&z, 0, sizeof z);
    z.next_in = static_cast<Bytef*>(*buf);
    z.avail_in = unsigned(nbytes);
    z.next_out = static_cast<Bytef*>(outbuf);
    z.avail_out = unsigned(nalloc);
    if (inflateInit(&z) != Z_OK) {
      H5free_memory(outbuf);
      return 0;
    }
    int status;
    do {
      status = inflate(&z, Z_SYNC_FLUSH);
      if (status == Z_STREAM_END) break;
      if (status != Z_OK) {
        inflateEnd(&z);
        H5free_memory(outbuf);
        return 0;
      }
      if (z.avail_out == 0) {
        nalloc *= 2;
        void* nb = H5resize_memory(outbuf, nalloc);
        if (!nb) {
          inflateEnd(&z);
          H5free_memory(outbuf);
          return 0;
        }
        outbuf = nb;
        z.next_out = static_cast<unsigned char*>(outbuf) + z.total_out;
        z.avail_out = uInt(nalloc - z.total_out);
      }
    } while (status == Z_OK);
    H5free_memory(*buf);
    *buf = outbuf;
    *buf_size = nalloc;
    const size_t ret = z.total_out;
    inflateEnd(&z);
    return ret;
  }
  // compress
  const int level = int(cd_values[0]);
  uLongf dst_n = compressBound(uLong(nbytes));
  std::vector<unsigned char> pre;
  const bool hit = g_installed && take(level, static_cast<const unsigned char*>(*buf), nbytes, pre);
  if (hit && pre.size() > dst_n) return 0;  // cannot happen (compressBound); as HDF5's "overflow"
  void* outbuf = H5allocate_memory(dst_n, false);
  if (!outbuf) return 0;
  if (hit) {
    std::memcpy(outbuf, pre.data(), pre.size());
    dst_n = uLongf(pre.size());
    ++g_hits;
  } else {
    const auto t0 = clk::now();
    const int status = compress2(static_cast<Bytef*>(outbuf), &dst_n, static_cast<const Bytef*>(*buf), uLong(nbytes), level);
    add_time(g_miss_s, since(t0));
    ++g_misses;
    if (status != Z_OK) {  // overflow, memory or other deflate error
      H5free_memory(outbuf);
      return 0;
    }
  }
  H5free_memory(*buf);
  *buf = outbuf;
  *buf_size = nbytes;
  return dst_n;
}
#endif

}  // namespace

bool active() { return g_installed && pool_threads() > 1; }

void install() {
#ifdef CPPWRF_HAVE_HDF5
  static std::once_flag once;
  std::call_once(once, [] {
    if (pool_threads() == 0) return;  // CPPWRF_NC_THREADS=0: HDF5's own filter
    // HDF5's public H5Zregister refuses the predefined filter ids (H5Z.c: "unable to modify predefined filters" for
    // an id below H5Z_FILTER_RESERVED), and a filter of another id changes the filter pipeline message of every
    // variable (docs/PERF.md, IO2). The library's own H5Z_register (H5Z.c, "allows filters to be set for predefined
    // method numbers") replaces the table entry of H5Z_FILTER_DEFLATE. It is used only with the HDF5 versions whose
    // H5Z_register and H5Z__filter_deflate this file follows (1.10 to 1.14: the same signature and the same deflate
    // filter); another version, a library without the exported symbol (a static build) or a failed registration
    // keeps HDF5's own filter: the same bytes, compressed in the writing thread. The library and its filter table are
    // initialised first (H5open, H5Zfilter_avail), so that the built-in registration cannot come after this one.
    // Also kept: a library of another minor version than the headers of the build (the class's layout is theirs), an
    // odd (development) minor version, and an H5Z_register of another shared library than the H5open in use.
    static_assert(H5Z_CLASS_T_VERS == 1, "H5Z_class2_t of HDF5 1.10 to 1.14");
    unsigned maj = 0, min = 0, rel = 0;
    if (H5open() < 0 || H5get_libversion(&maj, &min, &rel) < 0 || maj != 1 || min < 10 || min > 14 || min % 2 != 0 ||
        maj != H5_VERS_MAJOR || min != H5_VERS_MINOR)
      return;
    if (H5Zfilter_avail(H5Z_FILTER_DEFLATE) <= 0) return;
    using RegisterFn = herr_t (*)(const H5Z_class2_t*);
    auto reg = reinterpret_cast<RegisterFn>(dlsym(RTLD_DEFAULT, "H5Z_register"));
    if (!reg) return;
    if (!one_h5z_register(reinterpret_cast<void*>(reg))) return;
    // same id and name as HDF5's own class (H5Zdeflate.c H5Z_DEFLATE): the name goes into the filter pipeline messages
    static const H5Z_class2_t cls = {H5Z_CLASS_T_VERS, H5Z_FILTER_DEFLATE, 1, 1, "deflate", nullptr, nullptr, filter};
    g_installed = reg(&cls) >= 0;
  });
#endif
}

void submit(const Record& r, Group& g) {
  if (!active() || r.len.empty() || r.len.size() != r.chunk.size() || r.fill.size() != r.elsize) return;
  std::size_t nc = 1;
  for (std::size_t d = 0; d < r.len.size(); ++d) {
    if (r.chunk[d] == 0 || r.len[d] == 0) return;
    nc *= (r.len[d] + r.chunk[d] - 1) / r.chunk[d];
  }
  auto shared = std::make_shared<Record>(r);
  for (std::size_t c = 0; c < nc; ++c) {
    {  // counted before it is queued, uncounted if it cannot be (wait(g) then waits for the chunks queued only)
      std::lock_guard<std::mutex> lk(g.m);
      ++g.pending;
    }
    try {
      pool().submit([shared, c, &g] {
        try {
          compress_chunk(*shared, c, g);
        } catch (...) {  // (memory): no precomputed chunk, HDF5's filter compresses it itself
        }
        std::lock_guard<std::mutex> lk(g.m);
        if (--g.pending == 0) g.done.notify_all();
      });
    } catch (...) {
      std::lock_guard<std::mutex> lk(g.m);
      if (--g.pending == 0) g.done.notify_all();
      throw;
    }
  }
}

void wait(Group& g) {
  const auto t0 = clk::now();
  std::unique_lock<std::mutex> lk(g.m);
  if (g.pending == 0) return;
  g.done.wait(lk, [&g] { return g.pending == 0; });
  lk.unlock();
  add_wait(since(t0));
}

void clear(Group& g) {
  std::lock_guard<std::mutex> lk(g_mutex);
  for (auto it = g_ready.begin(); it != g_ready.end();)
    it = it->second->group == &g ? g_ready.erase(it) : std::next(it);
}

long precomputed(const Group& g) { return g.precomputed; }

UseGroup::UseGroup(const Group* g) : saved_(t_group) { t_group = g; }
UseGroup::~UseGroup() { t_group = saved_; }

Stats stats() {
  Stats s;
  s.precomputed = g_precomputed;
  s.hits = g_hits;
  s.misses = g_misses;
  std::lock_guard<std::mutex> lk(g_time_mutex);
  s.miss_s = g_miss_s;
  s.wait_s = g_wait_s;
  s.download_s = g_download_s;
  return s;
}
void add_download(double s) { add_time(g_download_s, s); }
void add_wait(double s) { add_time(g_wait_s, s); }

bool timing() {
  static const bool t = [] {
    const char* e = std::getenv("CPPWRF_IO_TIMING");
    return e && *e && *e != '0';
  }();
  return t;
}

}  // namespace cppwrf::wrf_io::deflate
