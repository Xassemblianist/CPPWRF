// SPDX-License-Identifier: GPL-3.0-or-later
#if defined(CPPWRF_USE_CUDA) || defined(CPPWRF_USE_HIP)
#define CPPWRF_GPU_RUNTIME_HOST 1
#include "core/gpu_runtime.hpp"
#endif
#include "cases/ensemble.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "cases/domain_runner.hpp"
#include "cases/model.hpp"
#include "cases/nest_driver.hpp"
#include "core/backend.hpp"
#include "core/tiles.hpp"
#include "io/option_tiers.hpp"
#ifdef CPPWRF_HAVE_NETCDF
#include "wrf_io/namelist.hpp"
#include "wrf_io/output_queue.hpp"
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace cppwrf::cases {

namespace {

std::string trim(const std::string& s) {
  std::size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

int member_count(const io::Config& cfg) {
  const long long n = cfg.get_int("ensemble.members");
  if (n < 1 || n > 1024) throw io::ConfigError("ensemble.members: 1 to 1024 members, not " + std::to_string(n));
  return int(n);
}

// the memberI key's overrides ('|'-separated section.key=value)
std::vector<std::string> member_sets(const io::Config& cfg, int i) {
  std::vector<std::string> out;
  const std::string key = "ensemble.member" + std::to_string(i);
  const std::string v = cfg.get_string(key, "");
  std::size_t a = 0;
  while (a <= v.size()) {
    std::size_t b = v.find('|', a);
    if (b == std::string::npos) b = v.size();
    const std::string s = trim(v.substr(a, b - a));
    if (!s.empty()) {
      const std::size_t eq = s.find('=');
      if (eq == std::string::npos) throw io::ConfigError(key + ": '" + s + "' is not section.key=value");
      if (trim(s.substr(0, eq)).rfind("ensemble.", 0) == 0)
        throw io::ConfigError(key + ": a member cannot set the ensemble's own keys ('" + s + "')");
      out.push_back(s);
    }
    a = b + 1;
  }
  return out;
}

// the ensemble.* keys of cfg: members, mode, member (one member alone), member1 .. memberN; others are errors
void check_ensemble_keys(const io::Config& cfg, int n) {
  for (const auto& [k, v] : cfg.entries()) {
    if (k.rfind("ensemble.", 0) != 0) continue;
    const std::string s = k.substr(9);
    if (s == "members" || s == "mode" || s == "member") continue;
    bool ok = s.size() > 6 && s.rfind("member", 0) == 0 && s[6] != '0';
    for (std::size_t c = 6; ok && c < s.size(); ++c) ok = std::isdigit(static_cast<unsigned char>(s[c])) != 0;
    if (!ok) throw io::ConfigError("unknown configuration key '" + k + "' (ensemble: members, mode, member, memberI)");
    if (std::stoll(s.substr(6)) > n)
      throw io::ConfigError(k + ": the ensemble has " + std::to_string(n) + " members (ensemble.members)");
  }
}

std::string member_dir(const io::Config& base, int i) {
  const std::string d = base.get_string("output.dir", "out/" + base.get_string("run.case"));
  return d + "/member" + std::to_string(i);
}

// a path as the file system resolves it (symbolic links of its existing part followed, . and .. removed)
std::string resolved(const std::string& p) {
  std::error_code ec;
  const std::filesystem::path a = std::filesystem::absolute(p, ec);
  if (ec) return std::filesystem::path(p).lexically_normal().string();
  const std::filesystem::path c = std::filesystem::weakly_canonical(a, ec);
  return (ec ? a.lexically_normal() : c).string();
}

// The directories a member writes files into, as the file system resolves them: its output.dir in every
// output.format (run.json, diag.csv, native snapshots and restart_*.nc: DomainRunner writes them whatever the
// format), and with output.format = wrfout / both the directory of its WRF files (output.wrf_dir, default
// output.dir) and the directories that the namelist's stream names (history_outname, rst_outname,
// auxhist3_outname) put below it. Two members never share one of these directories (also across kinds: one
// member's output.dir is not another's WRF directory), so no file of one member has the path of another's.
struct Dest {
  std::string dir;   // resolved
  std::string what;  // for the message
};
std::vector<Dest> member_dests(const io::Config& m, int i) {
  const std::string fmt = m.get_string("output.format", "native");
  const std::string dir = m.get_string("output.dir");
  std::vector<Dest> d;
  d.push_back({resolved(dir), "output.dir '" + dir + "'"});
  if (fmt != "wrfout" && fmt != "both") return d;
  const std::string wrf = m.get_string("output.wrf_dir", dir);
  d.push_back({resolved(wrf), "WRF files (output.wrf_dir, default output.dir) '" + wrf + "'"});
#ifdef CPPWRF_HAVE_NETCDF
  // the namelist WrfHistory reads (output.wrf_namelist, default real.dir/namelist.input; none: WRF's default names,
  // which have no directory part)
  const std::string real_dir = m.get_string("real.dir", "");
  const std::string nl = m.get_string("output.wrf_namelist", real_dir.empty() ? "" : real_dir + "/namelist.input");
  if (nl.empty() || !std::filesystem::exists(nl)) return d;
  const wrf_io::Namelist n = wrf_io::Namelist::read(nl);
  const std::string who = "ensemble member " + std::to_string(i) + ": " + nl + ": ";
  for (const char* key : {"history_outname", "rst_outname", "auxhist3_outname"})
    for (std::string v : n.values("time_control", key)) {
      if (v.size() >= 2 && (v.front() == '\'' || v.front() == '"') && v.back() == v.front()) v = v.substr(1, v.size() - 2);
      const std::filesystem::path p(v);
      for (const auto& c : p)
        if (c == "..")
          throw io::ConfigError(who + key + " = '" + v +
                                "' leaves the member's WRF directory (a '..' component): members would share its files");
      // (WrfHistory: <WRF directory>/<name>) a member is domain 1 (no nests); a date tag left in the directory part: a directory per date, which the
      // check cannot list
      std::string sub = p.parent_path().string();
      for (std::size_t k = sub.find("<domain>"); k != std::string::npos; k = sub.find("<domain>")) sub.replace(k, 8, "01");
      if (sub.empty()) continue;
      if (sub.find('<') != std::string::npos)
        throw io::ConfigError(who + key + " = '" + v + "': a date tag in its directory part is not supported in an ensemble");
      d.push_back({resolved(wrf + "/" + sub), std::string(key) + " '" + v + "' below '" + wrf + "'"});
    }
#else
  (void)i;
#endif
  return d;
}

// a member is one domain in one patch: no nests, no tiles (MPI ranks are refused by main)
void check_member(const io::Config& m, int i) {
  const std::string who = "ensemble member " + std::to_string(i) + ": ";
  if (has_nests(m)) throw io::ConfigError(who + "nested domains are not supported in an ensemble (yet)");
  if (tile::options(m.get_string("run.tiles", "1")).on())
    throw io::ConfigError(who + "run.tiles / CPPWRF_TILES are not supported in an ensemble");
  m.get_string("run.ranks", "");
}

struct Shared {
#ifndef CPPWRF_HAVE_NETCDF
  std::mutex host;  // the host work of the members: one member at a time
#endif
  std::atomic<bool> stop{false};  // an error in a member: the others stop at their next step
  std::atomic<int> first{-1};     // the member whose error came first (the one rethrown)
  void fail(int i) {
    int none = -1;
    first.compare_exchange_strong(none, i);
    stop.store(true);
  }
};

// The lock of a member's host work (input, output, boundaries, the start and the end: one member at a time). With
// NetCDF it is the process's NetCDF lock (wrf_io/output_queue.hpp): one lock for the members' host work and the
// output writer thread's NetCDF calls (output.write_queue_mb > 0), which therefore fall in between the members' host
// work, while the members step; a member that waits for the writer's queue lets it go there (output_queue::submit,
// drain) as a single run does. One lock, so no order between two locks to keep. Else a mutex.
class HostLock {
 public:
  explicit HostLock(Shared& sh) : sh_(sh) {}
  ~HostLock() {
    if (owns()) unlock();
  }
  HostLock(const HostLock&) = delete;
  HostLock& operator=(const HostLock&) = delete;
#ifdef CPPWRF_HAVE_NETCDF
  void lock() { wrf_io::nc_hold(); }
  void unlock() { wrf_io::nc_unhold(); }
  bool owns() const { return wrf_io::nc_held(); }  // (output_queue::drain lets it go: DomainRunner::finish)
#else
  void lock() {
    sh_.host.lock();
    owns_ = true;
  }
  void unlock() {
    owns_ = false;
    sh_.host.unlock();
  }
  bool owns() const { return owns_; }
#endif

 private:
  Shared& sh_;
#ifndef CPPWRF_HAVE_NETCDF
  bool owns_ = false;
#endif
};

// one member's run: main's time loop of a single domain, the host work under the members' lock
int run_member(const io::Config& cfg, const std::string& dir, Shared& sh, int i) {
#ifdef CPPWRF_HAVE_NETCDF
  // the member's WRF records: its own writer errors (named with the file and the member) and its own drain at the end
  const std::string label = "ensemble member " + std::to_string(i + 1);
  const wrf_io::output_queue::OwnerScope owner(i + 1, label.c_str());
#else
  (void)i;
#endif
  HostLock lk(sh);  // declared first: the runner and the model are destroyed under it
  lk.lock();
  Model m = Model::build(cfg);
  DomainRunner r(cfg, m, dir, "d01");
  try {
    int rc = 0;
    if (!r.start()) {
      rc = 3;
    } else {
      const bool overlap = step_graphs_on();
      lk.unlock();
      for (long n = r.start_step + 1; n <= m.nsteps && rc == 0; ++n) {
        lk.lock();
        if (sh.stop.load()) {
          rc = 4;
          break;
        }
        r.before_step(n);
        lk.unlock();
        m.rk->step(*m.st, *m.tend, *m.base);
        // the step's own wait before the lock: a member that reads its fields does not hold the others' host work
        if (!overlap || n == m.nsteps || r.reads_state(n)) dev_sync();
        lk.lock();
        if (!r.after_step(n)) rc = 3;
        else if (n < m.nsteps) lk.unlock();
      }
      if (!lk.owns()) lk.lock();
      if (rc == 0) r.finish();
      if (!lk.owns()) lk.lock();  // (finish lets the NetCDF lock go when it drains the output queue)
    }
    // the fatal checks of the launched steps are read here, where a raised code is this member's error (not in the
    // destructors below, which free device memory)
    dev_sync();
    return rc;
  } catch (...) {
    if (!lk.owns()) lk.lock();
    discard_pending_checks();
    throw;
  }
}

// debugging dumps write fixed file names: one member's run only
void check_no_dumps(int n) {
  if (n < 2) return;
  for (const char* e : {"CPPWRF_VM_DUMP", "CPPWRF_NOAH_DUMP", "CPPWRF_GF_DUMP", "CPPWRF_GFL_DUMP", "CPPWRF_NTIEDTKE_DUMP",
                        "CPPWRF_G3_DUMP"})
    if (const char* v = std::getenv(e); v && *v)
      throw io::ConfigError(std::string(e) + ": its files have one name for every member (run the member alone: "
                            "--set ensemble.member=I)");
}

// the threads of the members: joined on every way out (one that could not be started stops the others)
struct Workers {
  std::vector<std::thread> th;
  ~Workers() {
    for (std::thread& t : th)
      if (t.joinable()) t.join();
  }
};
// the device's work budget shared while the members run; back to one share on every way out
struct WorkShare {
  explicit WorkShare(int n) { set_device_work_share(n); }
  ~WorkShare() { set_device_work_share(1); }
  WorkShare(const WorkShare&) = delete;
  WorkShare& operator=(const WorkShare&) = delete;
};

}  // namespace

bool is_ensemble(const io::Config& cfg) { return cfg.has("ensemble.members"); }

io::Config ensemble_member_config(const std::string& path, const std::vector<std::string>& ov, const io::Config& cfg,
                                  int i) {
  const int n = member_count(cfg);
  check_ensemble_keys(cfg, n);
  if (i < 1 || i > n)
    throw io::ConfigError("ensemble.member: 1 to " + std::to_string(n) + " (ensemble.members), not " +
                          std::to_string(i));
  std::vector<std::string> all = ov;
  all.push_back("output.dir=" + member_dir(cfg, i));  // before the member's own: an output.dir there wins
  // a WRF directory of the configuration (output.wrf_dir) is the members' root as output.dir is: member I writes its
  // WRF files to <output.wrf_dir>/memberI (an output.wrf_dir in memberI wins)
  if (cfg.has("output.wrf_dir")) all.push_back("output.wrf_dir=" + cfg.get_string("output.wrf_dir") + "/member" + std::to_string(i));
  for (const std::string& s : member_sets(cfg, i)) all.push_back(s);
  io::Config m = io::Config::load(path, all);
  m.erase_section("ensemble");  // the member's configuration is that of its single run (restart files record it)
  return m;
}

int run_ensemble(const std::string& path, const std::vector<std::string>& ov, const io::Config& cfg) {
  const int n = member_count(cfg);
  check_ensemble_keys(cfg, n);
  const std::string mode = cfg.get_string("ensemble.mode", "threads");
  if (mode != "threads" && mode != "sequential")
    throw io::ConfigError("ensemble.mode: threads or sequential, not '" + mode + "'");
  const bool threads = mode == "threads" && n > 1;

  std::vector<io::Config> cfgs;
  std::vector<std::string> dirs;
  std::vector<std::vector<Dest>> dests;
  for (int i = 1; i <= n; ++i) {
    cfgs.push_back(ensemble_member_config(path, ov, cfg, i));
    check_member(cfgs.back(), i);
    io::check_option_tiers(cfgs.back(), "ensemble member " + std::to_string(i) + ": ");
    dirs.push_back(cfgs.back().get_string("output.dir"));
    dests.push_back(member_dests(cfgs.back(), i));
    // the directories as the file system resolves them (a/../a, symbolic links): no file of two members in one
    for (int j = 0; j + 1 < i; ++j)
      for (const Dest& a : dests[std::size_t(j)])
        for (const Dest& b : dests.back())
          if (a.dir == b.dir) {
            const std::string who = "ensemble members " + std::to_string(j + 1) + " and " + std::to_string(i);
            if (a.what.rfind("output.dir", 0) == 0 && b.what.rfind("output.dir", 0) == 0)
              throw io::ConfigError(who + " write to the same output.dir '" + dirs.back() + "' (" + b.dir + ")");
            if (a.what.rfind("WRF files", 0) == 0 && b.what.rfind("WRF files", 0) == 0)
              throw io::ConfigError(who + " write their WRF files to the same directory " + b.dir +
                                    " (output.wrf_dir, default output.dir)");
            throw io::ConfigError(who + " write files to the same directory " + b.dir + ": member " +
                                  std::to_string(j + 1) + "'s " + a.what + ", member " + std::to_string(i) + "'s " +
                                  b.what);
          }
  }
  check_no_dumps(n);
  int per = 1;
#if defined(_OPENMP)
  per = std::max(1, omp_get_max_threads() / (threads ? n : 1));
#endif
  std::printf("cppwrf: ensemble of %d members, %s%s\n", n, threads ? "one host thread each" : "one after the other",
              threads ? (", " + std::to_string(per) + " OpenMP threads each").c_str() : "");
  for (int i = 1; i <= n; ++i) std::printf("cppwrf: ensemble member %d: %s\n", i, dirs[std::size_t(i - 1)].c_str());
  std::fflush(stdout);

  Shared sh;
  std::vector<int> rc(std::size_t(n), 0);
  std::vector<char> started(std::size_t(n), 0);
  std::vector<double> wall(std::size_t(n), 0.0);
  std::vector<std::exception_ptr> err(static_cast<std::size_t>(n));
  const auto t0 = std::chrono::steady_clock::now();
  const int dev = current_device();
  auto body = [&](int i) {
    const auto t = std::chrono::steady_clock::now();
    started[std::size_t(i)] = 1;
    try {
      rc[std::size_t(i)] = run_member(cfgs[std::size_t(i)], dirs[std::size_t(i)], sh, i);
    } catch (...) {
      err[std::size_t(i)] = std::current_exception();
      sh.fail(i);
    }
    wall[std::size_t(i)] = std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
  };
  if (!threads) {
    for (int i = 0; i < n && !sh.stop.load(); ++i) body(i);
  } else {
    const WorkShare share(n);  // the members share the device's work budget, as ranks on one device do
    Workers w;
    for (int i = 0; i < n && !sh.stop.load(); ++i) {
      try {
        w.th.emplace_back([&, i] {
#if defined(CPPWRF_USE_GPU)
          if (gpuSetDevice(dev) != gpuSuccess) {
            started[std::size_t(i)] = 1;
            err[std::size_t(i)] = std::make_exception_ptr(std::runtime_error("ensemble: gpuSetDevice failed"));
            sh.fail(i);
            return;
          }
#else
          (void)dev;
#endif
#if defined(_OPENMP)
          omp_set_num_threads(per);
#endif
          set_thread_stream_sync(true);
          // the member's own fatal flags (core/backend.hpp): a code one member raises stops that member, never
          // another one that runs the same scheme at the same time (CUDA and HIP: the launches carry the slot)
          set_fatal_slot(i + 1);
          body(i);
          // the thread ends: its stream, graphs, flag slots, work and upload buffers are not kept for the process
          HostLock lk(sh);
          lk.lock();
          release_thread_device();
          release_host_staging();
        });
      } catch (const std::exception& e) {  // (a thread limit of the system): the members started stop
        started[std::size_t(i)] = 1;
        err[std::size_t(i)] = std::make_exception_ptr(
            std::runtime_error(std::string("ensemble: the thread of the member cannot be started (") + e.what() + ")"));
        sh.fail(i);
      }
    }
  }  // (the threads joined, the work share back to 1)
  const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  int worst = 0;
  for (int i = 0; i < n; ++i) {
    const char* what = err[std::size_t(i)]        ? "error"
                       : !started[std::size_t(i)] ? "not started (another member failed)"
                       : rc[std::size_t(i)] == 4  ? "stopped (another member failed)"
                       : rc[std::size_t(i)] == 3  ? "stopped (non-finite values)"
                                                  : "done";
    std::printf("cppwrf: ensemble member %d: %s, %.2f s\n", i + 1, what, wall[std::size_t(i)]);
    worst = std::max(worst, rc[std::size_t(i)]);
  }
  std::printf("cppwrf: ensemble wall time %.2f s\n", total);
  std::fflush(stdout);
  if (const int f = sh.first.load(); f >= 0) {
    std::fprintf(stderr, "cppwrf: ensemble member %d failed first\n", f + 1);
    std::rethrow_exception(err[std::size_t(f)]);
  }
  return worst;
}

}  // namespace cppwrf::cases
