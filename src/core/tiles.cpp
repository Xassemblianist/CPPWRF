// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/tiles.hpp"

#include <algorithm>
#include <atomic>
#include <climits>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#if defined(CPPWRF_USE_CUDA)
#include <cuda_runtime.h>
#endif
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace cppwrf::tile {

namespace {

struct Aborted : std::runtime_error {
  Aborted() : std::runtime_error("another tile stopped") {}
};

// A barrier that another tile's error releases (Aborted)
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
  std::map<const char*, int> by_addr;        // live allocations by address (a view's data pointer)
};

struct Iv {
  int lo, hi;
  bool empty() const { return hi <= lo; }
};
Iv meet(Iv a, Iv b) { return Iv{std::max(a.lo, b.lo), std::min(a.hi, b.hi)}; }

struct Patch {
  int i0, i1, j0, j1;
};

struct Set {
  Options o;
  int n = 1;
  std::vector<Registry> regs;  // per tile
  Registry ref;                // the check's reference run
  std::vector<int> dev;        // per tile
  std::mutex dom_m;
  bool dom_set = false;
  int nx = 0, ny = 0;
  bool per_x = false, per_y = false;
  std::vector<Patch> patch;
  Barrier tiles, all;
  std::vector<double> red;
  std::mutex serial;
  std::mutex err_m;
  std::exception_ptr err;
  bool err_root = false;
  std::string chk_msg;  // the check's first difference
  std::mutex chk_m;
};
Set* S = nullptr;
thread_local int tid = 0;
thread_local Registry* reg = nullptr;
thread_local long n_checks = 0;
// the check: fields whose copy around the patch differed at the last check
thread_local std::vector<char> stale_before;

void reg_add(const void* base, const Layout& L, int esz, const std::string& name) {
  Entry e;
  e.base = static_cast<char*>(const_cast<void*>(base));
  e.L = L;
  e.esz = esz;
  e.name = name;
  e.live = true;
  reg->idx[base] = int(reg->e.size());
  reg->by_addr[e.base] = int(reg->e.size());
  reg->e.push_back(std::move(e));
}
void reg_remove(const void* base) {
  auto it = reg->idx.find(base);
  if (it == reg->idx.end()) return;
  reg->e[std::size_t(it->second)].live = false;
  reg->by_addr.erase(static_cast<const char*>(base));
  reg->idx.erase(it);
}
const Track kTrack{&reg_add, &reg_remove};

// a field of the domain: its extents are those of a mass, U or V field of the tiled domain
bool domain_field(const Entry& e) {
  const bool x = e.L.nx == S->nx || e.L.nx == S->nx + 1;
  const bool y = e.L.ny == S->ny || e.L.ny == S->ny + 1;
  return e.live && x && y && e.L.si == 1;
}

// the points tile s owns of a field with layout L (its patch; at the domain's edges up to the end of the halo)
Iv own_x(int s, const Layout& L) {
  const Patch& p = S->patch[std::size_t(s)];
  return Iv{p.i0 == 0 ? -Grid::halo : p.i0, p.i1 == S->nx ? L.nx + Grid::halo : p.i1};
}
Iv own_y(int s, const Layout& L) {
  const Patch& p = S->patch[std::size_t(s)];
  return Iv{p.j0 == 0 ? -L.hy : p.j0, p.j1 == S->ny ? L.ny + L.hy : p.j1};
}
// the points tile t reads around its patch: Grid::halo wide, and with periodic boundaries the sources of the wrap
// (core/halo.cpp: halo point -a <- n - a, n + a <- a; the face n + 3 <- 3)
std::vector<Iv> need(int lo, int hi, int n, bool periodic, int ntiles, Iv array) {
  const int w = Grid::halo;
  std::vector<Iv> v{{lo - w, hi + w}};
  if (periodic && ntiles > 1) {
    if (lo == 0) v.push_back({n - w, n});
    if (hi == n) v.push_back({0, w + 1});
  }
  for (Iv& x : v) x = meet(x, array);
  return v;
}

// element offset of (i, j, k)
std::ptrdiff_t off(const Layout& L, int i, int j, int k) { return L.offset + i * L.si + j * L.sj + k * L.sk; }

// copies the box x * y * (all levels) of a field from src to dst (same layout); on dst's stream (CUDA)
void copy_box(const Entry& src, const Entry& dst, Iv x, Iv y, int sdev, int ddev) {
  const Layout& L = dst.L;
  const std::size_t esz = std::size_t(dst.esz);
  const int nk = L.sk ? std::max(L.nz, 1) : 1;
  const std::size_t w = std::size_t(x.hi - x.lo) * esz;
#if defined(CPPWRF_USE_CUDA)
  cudaStream_t st = static_cast<cudaStream_t>(cuda_stream_raw());
  cudaError_t e = cudaSuccess;
  if (L.sj > 0) {
    const std::size_t pitch = std::size_t(L.sj) * esz;
    const std::size_t rows = L.sk ? std::size_t(L.sk / L.sj) : std::size_t(L.ny + 2 * L.hy);
    const std::size_t x0 = std::size_t(L.offset - std::ptrdiff_t(L.hy) * L.sj + x.lo) * esz;
    const cudaPos pos = make_cudaPos(x0, std::size_t(L.hy + y.lo), 0);
    const cudaExtent ext = make_cudaExtent(w, std::size_t(y.hi - y.lo), std::size_t(nk));
    if (sdev == ddev) {
      cudaMemcpy3DParms p{};
      p.srcPtr = make_cudaPitchedPtr(src.base, pitch, pitch / esz, rows);
      p.dstPtr = make_cudaPitchedPtr(dst.base, pitch, pitch / esz, rows);
      p.srcPos = pos;
      p.dstPos = pos;
      p.extent = ext;
      p.kind = cudaMemcpyDeviceToDevice;
      e = cudaMemcpy3DAsync(&p, st);
    } else {
      cudaMemcpy3DPeerParms p{};
      p.srcPtr = make_cudaPitchedPtr(src.base, pitch, pitch / esz, rows);
      p.dstPtr = make_cudaPitchedPtr(dst.base, pitch, pitch / esz, rows);
      p.srcPos = pos;
      p.dstPos = pos;
      p.srcDevice = sdev;
      p.dstDevice = ddev;
      p.extent = ext;
      e = cudaMemcpy3DPeerAsync(&p, st);
    }
  } else {  // one row per level (ny == 1)
    const std::ptrdiff_t o = off(L, x.lo, 0, 0) * std::ptrdiff_t(esz);
    const std::size_t pitch = L.sk ? std::size_t(L.sk) * esz : w;  // one level: any pitch >= the width
    if (sdev == ddev)
      e = cudaMemcpy2DAsync(dst.base + o, pitch, src.base + o, pitch, w, std::size_t(nk), cudaMemcpyDeviceToDevice, st);
    else
      for (int k = 0; k < nk && e == cudaSuccess; ++k)
        e = cudaMemcpyPeerAsync(dst.base + o + std::ptrdiff_t(k) * std::ptrdiff_t(pitch), ddev,
                                src.base + o + std::ptrdiff_t(k) * std::ptrdiff_t(pitch), sdev, w, st);
  }
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("tiles: halo copy of ") + dst.name + ": " + cudaGetErrorString(e));
#else
  (void)sdev;
  (void)ddev;
  const int ny = L.sj ? y.hi - y.lo : 1;
  for (int k = 0; k < nk; ++k)
    for (int jj = 0; jj < ny; ++jj) {
      const std::ptrdiff_t o = off(L, x.lo, L.sj ? y.lo + jj : 0, k) * std::ptrdiff_t(esz);
      std::memcpy(dst.base + o, src.base + o, w);
    }
#endif
}

void stream_sync() {
#if defined(CPPWRF_USE_CUDA)
  const cudaError_t e = cudaStreamSynchronize(static_cast<cudaStream_t>(cuda_stream_raw()));
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("tiles: stream synchronisation: ") + cudaGetErrorString(e));
#endif
}

const Entry& peer_entry(int s, int idx, const Entry& mine) {
  const Registry& r = S->regs[std::size_t(s)];
  if (idx >= int(r.e.size())) throw std::logic_error("tiles: the tiles' fields differ (" + mine.name + ")");
  const Entry& e = r.e[std::size_t(idx)];
  if (!e.live || e.L.total != mine.L.total || e.esz != mine.esz)
    throw std::logic_error("tiles: the tiles' fields differ (" + mine.name + " / " + e.name + ")");
  return e;
}

// the field whose allocation holds p (its base, or a view's data pointer)
int index_of(const void* p) {
  auto it = reg->idx.find(p);
  if (it != reg->idx.end()) return it->second;
  const char* c = static_cast<const char*>(p);
  auto a = reg->by_addr.upper_bound(c);
  if (a != reg->by_addr.begin()) {
    --a;
    const Entry& e = reg->e[std::size_t(a->second)];
    if (c < e.base + e.L.total * std::size_t(e.esz)) return a->second;
  }
  throw std::logic_error("tiles: exchange of a field the tile did not allocate");
}

// pulls into this tile's copies of the fields idx the points the other tiles own: around the patch (halo) or all
void pull(const std::vector<int>& idx, bool halo) {
  const int t = tid;
  const Patch& p = S->patch[std::size_t(t)];
  stream_sync();  // this tile's launches are done
  S->tiles.wait();  // and every other tile's
  for (int i : idx) {
    const Entry& mine = reg->e[std::size_t(i)];
    const Layout& L = mine.L;
    const Iv ax{-Grid::halo, L.nx + Grid::halo}, ay{-L.hy, L.ny + L.hy};
    const std::vector<Iv> nxs = halo ? need(p.i0, p.i1, S->nx, S->per_x, S->o.px, ax) : std::vector<Iv>{ax};
    const std::vector<Iv> nys = halo ? need(p.j0, p.j1, S->ny, S->per_y, S->o.py, ay) : std::vector<Iv>{ay};
    for (int s = 0; s < S->n; ++s) {
      if (s == t) continue;
      const Entry& src = peer_entry(s, i, mine);
      const Iv ox = own_x(s, L), oy = own_y(s, L);
      for (const Iv& a : nxs)
        for (const Iv& b : nys) {
          const Iv x = meet(a, ox), y = meet(b, oy);
          if (!x.empty() && !y.empty()) copy_box(src, mine, x, y, S->dev[std::size_t(s)], S->dev[std::size_t(t)]);
        }
    }
  }
  stream_sync();
  S->tiles.wait();  // no tile writes before every tile has its copies
}

void exchange_hook(const void* const* bases, int n) {
  if (!S || tid < 0) return;
  std::vector<int> idx;
  idx.reserve(std::size_t(n));
  for (int k = 0; k < n; ++k) {
    const int i = index_of(bases[k]);
    if (!domain_field(reg->e[std::size_t(i)]))
      throw std::logic_error("tiles: exchange of " + reg->e[std::size_t(i)].name + ", not a field of the domain");
    idx.push_back(i);
  }
  pull(idx, true);
}

double max_hook(double v) {
  if (!S || tid < 0) return v;
  S->red[std::size_t(tid)] = v;
  S->tiles.wait();
  double m = S->red[0];
  for (int s = 1; s < S->n; ++s) m = S->red[std::size_t(s)] > m ? S->red[std::size_t(s)] : m;
  S->tiles.wait();
  return m;
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

// CPPWRF_TILE_CHECK: after every launch of a step, each tile's points against the reference run
void check_hook(const char* what) {
  if (!S) return;
  ++n_checks;
  S->all.wait();
  if (tid >= 0) {
    const Registry& r = S->ref;
    std::string msg;
    if (r.e.size() != reg->e.size()) {
      msg = "the fields allocated differ from the reference run's";
    } else {
      for (std::size_t i = 0; i < reg->e.size() && msg.empty(); ++i) {
        const Entry& mine = reg->e[i];
        const Entry& ref = r.e[i];
        if (mine.live != ref.live || (mine.live && mine.L.total != ref.L.total)) {
          msg = "the fields allocated differ from the reference run's (" + mine.name + ")";
          break;
        }
        if (!domain_field(mine)) continue;
        const Layout& L = mine.L;
        const Iv x = meet(own_x(tid, L), Iv{-Grid::halo, L.nx + Grid::halo});
        const Iv y = meet(own_y(tid, L), Iv{-L.hy, L.ny + L.hy});
        const int nk = L.sk ? std::max(L.nz, 1) : 1, ny = L.sj ? y.hi - y.lo : 1;
        const std::size_t bytes = std::size_t(x.hi - x.lo) * std::size_t(mine.esz);
        for (int k = 0; k < nk && msg.empty(); ++k)
          for (int jj = 0; jj < ny && msg.empty(); ++jj) {
            const int j = L.sj ? y.lo + jj : 0;
            const std::ptrdiff_t o = off(L, x.lo, j, k) * mine.esz;
            const char* a = mine.base + o;  // the CPU build: host memory
            const char* b = ref.base + o;
            if (std::memcmp(a, b, bytes) == 0) continue;
            for (int ii = 0; ii < x.hi - x.lo; ++ii) {
              const char* pa = a + std::size_t(ii) * std::size_t(mine.esz);
              const char* pb = b + std::size_t(ii) * std::size_t(mine.esz);
              if (std::memcmp(pa, pb, std::size_t(mine.esz)) == 0) continue;
              char h[200];
              std::snprintf(h, sizeof h, "%s (i, j, k) = (%d, %d, %d) from 0: ", mine.name.c_str(), x.lo + ii, j, k);
              msg = h + value_str(pa, mine.esz) + " (tile " + std::to_string(tid) + "), " + value_str(pb, mine.esz) +
                    " (reference)";
              break;
            }
          }
      }
    }
    // the fields this tile's copy has stale around its patch now (the inputs of the next launch)
    std::vector<char> stale(reg->e.size(), 0);
    {
      const Patch& p = S->patch[std::size_t(tid)];
      for (std::size_t i = 0; i < reg->e.size() && i < r.e.size(); ++i) {
        const Entry& mine = reg->e[i];
        const Entry& ref = r.e[i];
        if (!domain_field(mine) || !ref.live || ref.L.total != mine.L.total) continue;
        const Layout& L = mine.L;
        const Iv x = meet(Iv{p.i0 - Grid::halo, p.i1 + Grid::halo}, Iv{-Grid::halo, L.nx + Grid::halo});
        const Iv y = meet(Iv{p.j0 - Grid::halo, p.j1 + Grid::halo}, Iv{-L.hy, L.ny + L.hy});
        const int nk = L.sk ? std::max(L.nz, 1) : 1, ny = L.sj ? y.hi - y.lo : 1;
        const std::size_t bytes = std::size_t(x.hi - x.lo) * std::size_t(mine.esz);
        for (int k = 0; k < nk && !stale[i]; ++k)
          for (int jj = 0; jj < ny && !stale[i]; ++jj) {
            const std::ptrdiff_t o = off(L, x.lo, L.sj ? y.lo + jj : 0, k) * mine.esz;
            if (std::memcmp(mine.base + o, ref.base + o, bytes) != 0) stale[i] = 1;
          }
      }
    }
    if (!msg.empty()) {
      std::string names;
      int nn = 0;
      for (std::size_t i = 0; i < stale_before.size() && i < reg->e.size(); ++i)
        if (stale_before[i] && nn++ < 80) names += (names.empty() ? "" : ", ") + reg->e[i].name;
      msg += "; stale around tile " + std::to_string(tid) + " before the launch: " + (names.empty() ? "none" : names);
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
    if (tid == 0) {
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
  S->tiles.cancel();
  S->all.cancel();
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
#if defined(CPPWRF_USE_CUDA)
  if (o.check) throw std::runtime_error("CPPWRF_TILE_CHECK: the CPU build only");
#endif
  return o;
}

int id() { return S ? tid : 0; }
bool tiled() { return S != nullptr && tid >= 0; }

void set_domain(int nx, int ny, bool periodic_x, bool periodic_y) {
  if (!S) return;
  std::lock_guard<std::mutex> lk(S->dom_m);
  if (S->dom_set) {
    if (S->nx != nx || S->ny != ny || S->per_x != periodic_x || S->per_y != periodic_y)
      throw std::logic_error("tiles: the tiles' domains differ");
    return;
  }
  const int px = S->o.px, py = S->o.py;
  if (ny == 1 && py > 1) throw std::runtime_error("run.tiles: a 2-D (x-z) domain has one tile along y");
  if ((px > 1 && nx / px < Grid::halo) || (py > 1 && ny / py < Grid::halo))
    throw std::runtime_error("run.tiles: the tiles must be at least " + std::to_string(Grid::halo) +
                             " points wide (" + std::to_string(nx) + " x " + std::to_string(ny) + " points in " +
                             std::to_string(px) + " x " + std::to_string(py) + " tiles)");
  S->nx = nx;
  S->ny = ny;
  S->per_x = periodic_x;
  S->per_y = periodic_y;
  S->patch.resize(std::size_t(S->n));
  for (int t = 0; t < S->n; ++t) {
    const int tx = t % px, ty = t / px;
    S->patch[std::size_t(t)] = Patch{int(long(tx) * nx / px), int(long(tx + 1) * nx / px), int(long(ty) * ny / py),
                                     int(long(ty + 1) * ny / py)};
  }
  S->dom_set = true;
}

Step::Step() {
  in_step = true;
  if (!S || tid < 0) return;
  if (!S->dom_set) throw std::logic_error("tiles: a step before set_domain");
  const Patch& p = S->patch[std::size_t(tid)];
  ctx.i0 = p.i0 == 0 ? INT_MIN : p.i0;
  ctx.i1 = p.i1 == S->nx ? INT_MAX : p.i1;
  ctx.j0 = (S->ny == 1 || p.j0 == 0) ? INT_MIN : p.j0;
  ctx.j1 = (S->ny == 1 || p.j1 == S->ny) ? INT_MAX : p.j1;
  ctx.on = true;
}
Step::~Step() {
  ctx.on = false;
  in_step = false;
}

void allgather() {
  if (!S || tid < 0) return;
  std::vector<int> idx;
  for (std::size_t i = 0; i < reg->e.size(); ++i)
    if (domain_field(reg->e[i])) idx.push_back(int(i));
  pull(idx, false);
}

void gather(const void* const* bases, int n) {
  if (!S || tid < 0) return;
  std::vector<int> idx;
  for (int k = 0; k < n; ++k) idx.push_back(index_of(bases[k]));
  pull(idx, false);
}

Serial::Serial() {
  if (S) lk_ = std::unique_lock<std::mutex>(S->serial);
}

int run(const Options& o, const std::function<int(int)>& body) {
  if (S) throw std::logic_error("tiles: a tiled run inside a tiled run");
  Set set;
  set.o = o;
  set.n = o.count();
  set.regs.resize(std::size_t(set.n));
  set.red.resize(std::size_t(set.n));
  set.dev.resize(std::size_t(set.n), current_device());
  if (!o.devices.empty())
    for (int t = 0; t < set.n; ++t) set.dev[std::size_t(t)] = o.devices[std::size_t(t) % o.devices.size()];
  set.tiles.init(set.n);
  set.all.init(set.n + (o.check ? 1 : 0));
  S = &set;
  hooks.multi = true;
  hooks.max = &max_hook;
  hooks.exchange = &exchange_hook;
  if (o.check) hooks.check = &check_hook;
  int per = 1;
#if defined(_OPENMP)
  per = std::max(1, omp_get_max_threads() / (set.n + (o.check ? 1 : 0)));
#endif
  std::printf("cppwrf: %d x %d tiles%s, %d OpenMP threads each\n", o.px, o.py,
              o.check ? " and the check's reference run" : "", per);
  std::vector<int> rc(std::size_t(set.n), 0);
  auto thread_main = [&](int t) {
    tid = t;
    reg = t >= 0 ? &set.regs[std::size_t(t)] : &set.ref;
    track = &kTrack;
    try {
#if defined(CPPWRF_USE_CUDA)
      if (t >= 0) {
        const cudaError_t e = cudaSetDevice(set.dev[std::size_t(t)]);
        if (e != cudaSuccess) throw std::runtime_error(std::string("tiles: cudaSetDevice: ") + cudaGetErrorString(e));
      }
#endif
#if defined(_OPENMP)
      omp_set_num_threads(per);
#endif
      const int r = body(t);
      if (t >= 0) rc[std::size_t(t)] = r;
      if (r != 0) fail(std::make_exception_ptr(std::runtime_error("a tile stopped")), false);
    } catch (const Aborted&) {
      fail(std::current_exception(), false);
    } catch (...) {
      fail(std::current_exception(), true);
    }
    track = nullptr;
    reg = nullptr;
  };
  std::vector<std::thread> th;
  for (int t = 0; t < set.n; ++t) th.emplace_back(thread_main, t);
  if (o.check) th.emplace_back(thread_main, -1);
  for (std::thread& x : th) x.join();
  hooks = Hooks{};
  S = nullptr;
  if (set.err && set.err_root) std::rethrow_exception(set.err);
  for (int r : rc)
    if (r != 0) return r;
  if (set.err) std::rethrow_exception(set.err);
  return rc[0];
}

}  // namespace cppwrf::tile
