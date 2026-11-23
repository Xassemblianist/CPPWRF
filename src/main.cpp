// SPDX-License-Identifier: GPL-3.0-or-later
// cppwrf: ./cppwrf <config.ini> [--set section.key=value ...] | --version
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "cases/domain_runner.hpp"
#include "cases/ensemble.hpp"
#include "cases/model.hpp"
#include "cases/nest_driver.hpp"
#include "core/backend.hpp"
#include "core/precision.hpp"
#include "core/tiles.hpp"
#include "io/config.hpp"
#include "io/option_tiers.hpp"
#ifdef CPPWRF_HAVE_NETCDF
#include "wrf_io/output_queue.hpp"
#endif

using namespace cppwrf;

// One worker of a decomposed run (core/tiles.hpp): the run of a single domain on the worker's patch. The writer
// (patch 0) writes the output in out_dir, the check's reference run in out_dir.ref.
static int run_tile(const io::Config& cfg, const std::string& out_dir, int t) {
  // tiles: the start, the input and the output one tile at a time (tile::Serial); the time steps together
  tile::Serial serial;
  const std::string dir = t >= 0 ? out_dir : out_dir + ".ref";
  cases::Model m = cases::Model::build(cfg);
  cases::DomainRunner r(cfg, m, dir, "d01");
  if (!r.start()) return 3;
  tile::exchange_all();
  serial.unlock();
  for (long n = r.start_step + 1; n <= m.nsteps; ++n) {
    {
      tile::Serial s;
      r.before_step(n);
    }
    {
      tile::Step step;
      m.rk->step(*m.st, *m.tend, *m.base);
    }
    // CUDA graphs (an MPI rank: core/backend.cpp graphs_mode): the host prepares the next step while the device runs
    // this one, as one process does; else (tiles) the step is waited for
    if (!step_graphs_on() || n == m.nsteps) dev_sync();
    tile::Serial s;
    if (!r.after_step(n)) return 3;
  }
  tile::Serial s;
  r.finish();
  return 0;
}

static int run(int argc, char** argv, bool mpi) {
  if (argc == 2 && std::string(argv[1]) == "--version") {  // the drop-in tools choose the binary of a mode by it
    std::printf("cppwrf %s commit %s backend %s precision %s\n", CPPWRF_VERSION, CPPWRF_COMMIT, backend_name(),
                precision_name);
    return 0;
  }
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <config.ini> [--set section.key=value ...]\n", argv[0]);
    return 2;
  }
  std::vector<std::string> ov;
  for (int a = 2; a < argc; ++a) {
    std::string s = argv[a];
    if (s == "--set" && a + 1 < argc) ov.push_back(argv[++a]);
    else if (s.rfind("--set=", 0) == 0) ov.push_back(s.substr(6));
    else throw std::runtime_error("unknown argument: " + s);
  }
  io::Config cfg = io::Config::load(argv[1], ov);
  // an [ensemble] section (cases/ensemble.hpp): every member in this process, or member ensemble.member alone
  if (cases::is_ensemble(cfg)) {
    if (mpi) throw std::runtime_error("ensemble: not in an MPI run (run the members in one process)");
    if (!cfg.has("ensemble.member")) return cases::run_ensemble(argv[1], ov, cfg);
    cfg = cases::ensemble_member_config(argv[1], ov, cfg, int(cfg.get_int("ensemble.member")));
  }
  const std::string cname = cfg.get_string("run.case");
  const std::string out_dir = cfg.get_string("output.dir", "out/" + cname);
  // options below the verified tier stop here without cppwrf.allow_unverified (a nested run: per domain, run_nested)
  if (!cases::has_nests(cfg)) io::check_option_tiers(cfg);

  const tile::Options tiles = tile::options(cfg.get_string("run.tiles", "1"));
  const std::string ranks = cfg.get_string("run.ranks", "");  // the layout of an MPI run (a known key in every run)
  // an MPI run's halo transport: device buffers to a GPU-aware MPI, or pinned host buffers (docs/MPI.md)
  const std::string gpu_aware = cfg.get_string("run.gpu_aware_mpi", "auto");
#ifdef CPPWRF_HAVE_NETCDF
  // a decomposed run writes its files in the model's thread (output.write_queue_mb has no effect)
  if (mpi || tiles.on()) wrf_io::output_queue::forbid();
#endif
  if (mpi) {  // several MPI ranks: each is the worker of one patch (docs/MPI.md)
    if (tiles.on()) throw std::runtime_error("run.tiles: an MPI run has one patch per rank (run.ranks)");
    const bool nests = cases::has_nests(cfg);  // every domain in the same P x Q patches
    return tile::mpi_run(ranks, gpu_aware,
                         [&](int t) { return nests ? cases::run_nested(cfg, out_dir) : run_tile(cfg, out_dir, t); });
  }
  if (tiles.on()) {
    const bool nests = cases::has_nests(cfg);
    if (nests && tiles.check) throw std::runtime_error("CPPWRF_TILE_CHECK: not with nests");
    return tile::run(tiles, [&](int t) { return nests ? cases::run_nested(cfg, out_dir) : run_tile(cfg, out_dir, t); });
  }

  if (cases::has_nests(cfg)) {
    return cases::run_nested(cfg, out_dir);
  }

  cases::Model m = cases::Model::build(cfg);
  cases::DomainRunner r(cfg, m, out_dir, "d01");
  if (!r.start()) return 3;

  // CUDA graphs: the host prepares step n + 1 while the device runs step n (every read of device data waits for
  // it; the step's fatal checks are read once the next step is launched); the last step is waited for
  const bool overlap = step_graphs_on();
  for (long n = r.start_step + 1; n <= m.nsteps; ++n) {
    r.before_step(n);
    {
#ifdef CPPWRF_HAVE_NETCDF
      const wrf_io::NcReleased io;  // the time step makes no NetCDF call: the output writer thread may (output_queue)
#endif
      m.rk->step(*m.st, *m.tend, *m.base);
      if (!overlap || n == m.nsteps) dev_sync();
    }
    if (!r.after_step(n)) return 3;
  }
  r.finish();
  return 0;
}

int main(int argc, char** argv) {
  int rc = 2;
  try {
    const bool mpi = tile::mpi_start(&argc, &argv);
    rc = run(argc, argv, mpi);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cppwrf: error: %s\n", e.what());
#ifdef CPPWRF_HAVE_NETCDF
    wrf_io::output_queue::finish();  // the records queued before the error are written, as the synchronous writer had
#endif
    tile::mpi_abort(2);
    rc = 2;
  }
#ifdef CPPWRF_HAVE_NETCDF
  wrf_io::output_queue::finish();
#endif
  tile::mpi_finish();
  return rc;
}
