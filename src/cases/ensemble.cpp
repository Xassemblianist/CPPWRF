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
#include <cstdio>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "cases/domain_runner.hpp"
#include "cases/model.hpp"
#include "cases/nest_driver.hpp"
#include "core/backend.hpp"
#include "core/tiles.hpp"
#include "io/option_tiers.hpp"

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

// a member is one domain in one patch: no nests, no tiles (MPI ranks are refused by main)
void check_member(const io::Config& m, int i) {
  const std::string who = "ensemble member " + std::to_string(i) + ": ";
  if (has_nests(m)) throw io::ConfigError(who + "nested domains are not supported in an ensemble (yet)");
  if (tile::options(m.get_string("run.tiles", "1")).on())
    throw io::ConfigError(who + "run.tiles / CPPWRF_TILES are not supported in an ensemble");
  m.get_string("run.ranks", "");
}

struct Shared {
  std::mutex host;  // the host work of the members (NetCDF is not thread-safe): one member at a time
  std::atomic<bool> stop{false};  // an error in a member: the others stop at their next step
  std::atomic<int> first{-1};     // the member whose error came first (the one rethrown)
  void fail(int i) {
    int none = -1;
    first.compare_exchange_strong(none, i);
    stop.store(true);
  }
};

// one member's run: main's time loop of a single domain, the host work under the shared lock
int run_member(const io::Config& cfg, const std::string& dir, Shared& sh) {
  std::unique_lock<std::mutex> lk(sh.host);  // declared first: the runner and the model are destroyed under it
  Model m = Model::build(cfg);
  DomainRunner r(cfg, m, dir, "d01");
  try {
    if (!r.start()) return 3;
    const bool overlap = step_graphs_on();
    lk.unlock();
    for (long n = r.start_step + 1; n <= m.nsteps; ++n) {
      lk.lock();
      if (sh.stop.load()) return 4;
      r.before_step(n);
      lk.unlock();
      m.rk->step(*m.st, *m.tend, *m.base);
      // the step's own wait before the lock: a member that reads its fields does not hold the others' host work
      if (!overlap || n == m.nsteps || r.reads_state(n)) dev_sync();
      lk.lock();
      if (!r.after_step(n)) return 3;
      lk.unlock();
    }
    lk.lock();
    r.finish();
    return 0;
  } catch (...) {
    if (!lk.owns_lock()) lk.lock();
    throw;
  }
}

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
  for (int i = 1; i <= n; ++i) {
    cfgs.push_back(ensemble_member_config(path, ov, cfg, i));
    check_member(cfgs.back(), i);
    io::check_option_tiers(cfgs.back(), "ensemble member " + std::to_string(i) + ": ");
    dirs.push_back(cfgs.back().get_string("output.dir"));
    for (int j = 0; j + 1 < i; ++j)
      if (dirs[std::size_t(j)] == dirs.back())
        throw io::ConfigError("ensemble members " + std::to_string(j + 1) + " and " + std::to_string(i) +
                              " write to the same output.dir '" + dirs.back() + "'");
  }
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
  std::vector<double> wall(std::size_t(n), 0.0);
  std::vector<std::exception_ptr> err(static_cast<std::size_t>(n));
  const auto t0 = std::chrono::steady_clock::now();
  const int dev = current_device();
  auto body = [&](int i) {
    const auto t = std::chrono::steady_clock::now();
    try {
      rc[std::size_t(i)] = run_member(cfgs[std::size_t(i)], dirs[std::size_t(i)], sh);
    } catch (...) {
      err[std::size_t(i)] = std::current_exception();
      sh.fail(i);
    }
    wall[std::size_t(i)] = std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
  };
  if (!threads) {
    for (int i = 0; i < n && !sh.stop.load(); ++i) body(i);
  } else {
    set_device_work_share(n);  // the members share the device's work budget, as ranks on one device do
    std::vector<std::thread> th;
    for (int i = 0; i < n; ++i)
      th.emplace_back([&, i] {
#if defined(CPPWRF_USE_GPU)
        if (gpuSetDevice(dev) != gpuSuccess) {
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
        body(i);
        {
          std::lock_guard<std::mutex> g(sh.host);
          release_host_staging();  // the thread ends: its upload buffer is not kept for the process
        }
      });
    for (std::thread& x : th) x.join();
    set_device_work_share(1);
  }
  const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  int worst = 0;
  for (int i = 0; i < n; ++i) {
    const char* what = err[std::size_t(i)] ? "error" : rc[std::size_t(i)] == 4 ? "stopped (another member failed)"
                       : rc[std::size_t(i)] == 3                              ? "stopped (non-finite values)"
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
