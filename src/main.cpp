// SPDX-License-Identifier: GPL-3.0-or-later
// cppwrf: ./cppwrf <config.ini> [--set section.key=value ...] | --version
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "cases/domain_runner.hpp"
#include "cases/model.hpp"
#include "cases/nest_driver.hpp"
#include "core/backend.hpp"
#include "core/precision.hpp"
#include "core/tiles.hpp"
#include "io/config.hpp"

using namespace cppwrf;

// One tile of a tiled run (core/tiles.hpp): the run of a single domain, its output in out_dir (tile 0),
// out_dir.tileN (tile N) or out_dir.ref (the check's reference run). The start, the input and the output one tile at a
// time; the time steps together (the halo exchanges inside them), with the fields gathered before the output.
static int run_tile(const io::Config& cfg0, const std::string& out_dir, int t) {
  tile::Serial serial;
  io::Config cfg = cfg0;
  const std::string dir = t == 0 ? out_dir : out_dir + (t < 0 ? ".ref" : ".tile" + std::to_string(t));
  cases::Model m = cases::Model::build(cfg);
  tile::set_domain(m.g.nx, m.g.ny, m.g.bc_x == BC::Periodic, m.g.bc_y == BC::Periodic);
  cases::DomainRunner r(cfg, m, dir, "d01");
  if (!r.start()) return 3;
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
    dev_sync();
    if (r.reads_state(n)) tile::allgather();
    tile::Serial s;
    if (!r.after_step(n)) return 3;
  }
  tile::Serial s;
  r.finish();
  return 0;
}

static int run(int argc, char** argv) {
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
  const std::string cname = cfg.get_string("run.case");
  const std::string out_dir = cfg.get_string("output.dir", "out/" + cname);

  const tile::Options tiles = tile::options(cfg.get_string("run.tiles", "1"));
  if (tiles.on()) {
    if (cases::has_nests(cfg)) throw std::runtime_error("run.tiles: nested runs have one tile per domain");
    return tile::run(tiles, [&](int t) { return run_tile(cfg, out_dir, t); });
  }

  if (cases::has_nests(cfg)) {
    return cases::run_nested(cfg, out_dir);
  }

  cases::Model m = cases::Model::build(cfg);
  cases::DomainRunner r(cfg, m, out_dir, "d01");
  if (!r.start()) return 3;

  // CUDA graphs: the host prepares step n + 1 while the device runs step n (every read of device data waits for
  // it; the step's fatal checks are read before the next step is launched); the last step is waited for
  const bool overlap = step_graphs_on();
  for (long n = r.start_step + 1; n <= m.nsteps; ++n) {
    r.before_step(n);
    m.rk->step(*m.st, *m.tend, *m.base);
    if (!overlap || n == m.nsteps) dev_sync();
    if (!r.after_step(n)) return 3;
  }
  r.finish();
  return 0;
}

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cppwrf: error: %s\n", e.what());
    return 2;
  }
}
