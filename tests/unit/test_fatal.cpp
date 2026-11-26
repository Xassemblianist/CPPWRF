// SPDX-License-Identifier: GPL-3.0-or-later
// WRF's fatal stops inside physics kernels (raise_fatal in the kernel, fatal_check on the host):
//  (a) Noah land SFLX, glacial ice and sea ice with SNEQV / SNOWH > 1 stop with
//      "Physical snow depth is less than snow water equiv.";
//      Noah land with a category beyond the tables or NROOT > NSOIL stops as WRF's REDPRM; a land-use category beyond
//      the VEGPARM.TBL dataset of MMINLU (a tiny synthetic table read by read_noah_tables) stops with REDPRM's
//      'Warning: too many input landuse types' (module_sf_noahlsm.F:2436-2438), as wrf.exe does on MODIS input
//      whose lake category 21 stays land without ISLAKE (VEGPARM.TBL lists 20 MODIS categories);
//  (b) the Kain-Fritsch codes (QG(1) < 0, ISTOP = 1) raised in a kernel stop with WRF's messages;
//  (c) a physical snowpack raises nothing, and fatal_check clears the flag after a stop;
//  (d) of several codes raised in one call the smallest stops the run, whatever the threads' order (an atomic
//      minimum): a large domain whose last column raises the smallest code; a decomposed run (tiles in this process)
//      stops every worker with the smallest code any worker raised (tile::hooks.first_code), as one patch does;
//  (e) the fatal slots of ensemble members (set_fatal_slot): a code raised by a kernel of a thread with slot 3
//      (the launch carries the slot) stops that thread only; a thread with slot 0 and one with slot 5 that
//      raise nothing see no code, also while the other thread raises.
// The kernels run on the backend of the build (CPU or CUDA device flag).
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "core/backend.hpp"
#include "core/field.hpp"
#include "core/precision.hpp"
#include "core/tiles.hpp"
#include "phys/driver/noah_ice.hpp"
#include "phys/lsm_noah/noah.hpp"
#include "phys/lsm_noah/noah_tables.hpp"

using namespace cppwrf;
static int fails = 0;
#define CHECK(c, ...)                                                                                       \
  do {                                                                                                      \
    if (!(c)) { std::printf("FAIL %s:%d %s ", __FILE__, __LINE__, #c); std::printf(__VA_ARGS__); std::printf("\n"); ++fails; } \
  } while (0)

// Runs f, returns the FatalError code (0: no stop).
template <class F> static int stop_code(F&& f) {
  try {
    f();
    fatal_check();
  } catch (const FatalError& e) {
    return e.code;
  }
  return 0;
}

static const Range2D one{0, 1, 0, 1};

struct KGlacial {
  real sneqv, snowh;
  CPPWRF_HD void operator()(int, int) const {
    real sldpth[4] = {real(0.1), real(0.3), real(0.6), real(1.0)};
    real stc[4] = {real(260.), real(261.), real(262.), real(263.)};
    real emissi = real(0.95), t1 = real(260.), sh = snowh, sw = sneqv, sncovr = real(1.), snotime1 = real(0.),
         ribb = real(0.);
    phys::noah_ice::glacial::Out o{};
    phys::noah_ice::glacial::sflx_glacial(real(0.), real(60.), 4, sldpth, real(250.), real(100.), real(1.e5),
                                          real(0.), real(260.), real(1.e-3), real(260.), real(1.2e-3),
                                          real(1.e-4), real(0.8), real(0.8), real(263.), real(0.001), emissi,
                                          real(0.95), t1, stc, sh, sw, real(0.01), sncovr, snotime1, ribb,
                                          real(2.834e6), o);
  }
};

struct KSeaice {
  real sneqv, snowh;
  CPPWRF_HD void operator()(int, int) const {
    real stc[4] = {real(265.), real(266.), real(268.), real(270.)};
    real z0brd = real(0.001), emissi = real(0.98), t1 = real(265.), sh = snowh, sw = sneqv, sncovr = real(1.),
         ribb = real(0.);
    phys::noah_ice::seaice::Opts op{};
    phys::noah_ice::seaice::Out o{};
    phys::noah_ice::seaice::sflx_seaice(op, real(0.), real(60.), 4, real(3.), real(250.), real(100.), real(1.e5),
                                        real(0.), real(265.), real(1.e-3), real(265.), real(1.2e-3), real(1.e-4),
                                        real(0.8), real(271.4), z0brd, emissi, t1, stc, sh, sw, real(0.01),
                                        sncovr, ribb, o);
  }
};

struct KRaise {
  int code;
  CPPWRF_HD void operator()(int, int) const { raise_fatal(code); }
};

// every column raises 9 + (i + j) % 5, the last one (nx - 1, ny - 1) 3
struct KRaiseMany {
  int nx, ny;
  CPPWRF_HD void operator()(int i, int j) const { raise_fatal(i == nx - 1 && j == ny - 1 ? 3 : 9 + (i + j) % 5); }
};

// Noah land SFLX on one column: every real view points into a zeroed scratch block except SNEQV and
// SNOWH; VEGTYP = SOILTYP = SLOPETYP = ACTIVE = 1; one category in each table (cats: SLCATS, LUCATS,
// SLPCATS; nroot: NROTBL(1)). With tb: those tables and VEGTYP = vegtyp.
static int noah_land(real sneqv, real snowh, int cats = 1, int nroot = 1, const phys::noah::NoahTables* tb = nullptr,
                     int vegtyp = 1) {
  const int nscr = 64;
  real* scr = static_cast<real*>(dev_alloc(sizeof(real) * nscr));
  real* snow = static_cast<real*>(dev_alloc(sizeof(real) * 2));
  int* ones = static_cast<int*>(dev_alloc(sizeof(int)));
  int* veg = static_cast<int*>(dev_alloc(sizeof(int)));
  const real hs[2] = {sneqv, snowh};
  const int one_i = 1;
  copy_to_dev(snow, hs, sizeof hs);
  copy_to_dev(ones, &one_i, sizeof one_i);
  copy_to_dev(veg, &vegtyp, sizeof vegtyp);
  phys::noah::NoahKernel k{};
  View2<real> s{scr, 1};
  View2<const real> c{scr, 1};
  View3<real> s3{scr, 1, 1};
  View2<const int> n{ones, 1};
  k.lwdn = k.soldn = k.solnet = k.sfcprs = k.prcp = k.sfctmp = k.th2 = k.q2 = k.sfcspd = k.q2sat = k.dqsdt2 =
      k.ffrozp = k.shdfac = k.shdmin = k.shdmax = k.alb = k.snoalb = k.tbot = c;
  k.vegtyp = k.soiltyp = k.slopetyp = k.active = n;
  k.vegtyp = View2<const int>{veg, 1};
  k.z0brd = k.embrd = k.cmc = k.t1 = k.albedo = k.ch = k.cm = k.emissi = k.z0 = k.xlai = k.snotime1 = k.ribb = s;
  k.stc = k.smc = k.sh2o = k.et = k.smav = s3;
  k.eta = k.sheat = k.eta_kinematic = k.fdown = k.ec = k.edir = k.ett = k.esnow = k.drip = k.dew = k.beta = k.etp =
      k.ssoil = k.flx1 = k.flx2 = k.flx3 = k.snomlt = k.sncovr = k.runoff1 = k.runoff2 = k.runoff3 = k.rc = k.pc =
          k.rsmin = k.soilw = k.soilm = k.q1 = s;
  k.sneqv = View2<real>{snow, 1};
  k.snowh = View2<real>{snow + 1, 1};
  k.dt = real(60.);
  k.zlvl = real(10.);
  k.nsoil = 4;
  const real sl[4] = {real(0.1), real(0.3), real(0.6), real(1.0)};
  for (int z = 0; z < 4; ++z) k.sldpth[z] = sl[z];
  k.isurban = 1;
  phys::noah::NoahTables t{};
  if (tb) {
    t = *tb;
  } else {
    t.slcats = t.lucats = t.slpcats = cats;
    t.nrotbl[0] = nroot;
  }
  auto* d_tb = static_cast<phys::noah::NoahTables*>(dev_alloc(sizeof t));
  copy_to_dev(d_tb, &t, sizeof t);
  k.tables = d_tb;
  const int code = stop_code([&] { phys::noah::run_noah(k, one); });
  dev_free(d_tb);
  dev_free(scr);
  dev_free(snow);
  dev_free(ones);
  dev_free(veg);
  return code;
}

// GENPARM.TBL, SOILPARM.TBL and VEGPARM.TBL with one slope, one soil and two land-use categories (dataset SYNTH2)
static bool synthetic_tables(const std::filesystem::path& dir, phys::noah::NoahTables& tb) {
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "GENPARM.TBL") << "General Parameters\nSLOPE_DATA\n1\n0.1\nZBOT_DATA\n-8.0\n";
  std::ofstream(dir / "SOILPARM.TBL") << "Soil Parameters\nSTAS\n1,1 'BB DRYSMC'\n"
                                         "1, 2.79, 0.010, -0.472, 0.339, 0.236, 0.069, 1.07E-6, 0.608E-6, 0.010, "
                                         "0.92, 'SAND'\n";
  std::ofstream v(dir / "VEGPARM.TBL");
  v << "Vegetation Parameters\nSYNTH2\n2,1 'SHDFAC NROOT'\n";
  for (int lc = 1; lc <= 2; ++lc)
    v << lc << ", .10, 1, 300., 100., 42.00, 0.04, 70., 0.50, 3.29, .920, .920, 0.15, 0.15, .03, .05, 0.0, 0.0, "
             "'CAT'\n";
  v << "TOPT_DATA\n298.0\nCMCMAX_DATA\n0.5E-3\nCFACTR_DATA\n0.5\nRSMAX_DATA\n5000.0\nBARE\n2\nNATURAL\n1\n";
  for (int n = 1; n <= 11; ++n) v << "LCZ_" << n << "\n1\n";
  v.close();
  return phys::noah::read_noah_tables(dir.string(), "SYNTH2", "STAS", tb);
}

int main() {
  // (a) snow density > 1: 1 m of water in 0.5 m of snow
  CHECK(noah_land(real(1.0), real(0.5)) == kFatalSnowDensity, "noah land");
  CHECK(stop_code([] { column_for(one, KGlacial{real(1.0), real(0.5)}); }) == kFatalSnowDensity, "glacial");
  CHECK(stop_code([] { column_for(one, KSeaice{real(1.0), real(0.5)}); }) == kFatalSnowDensity, "sea ice");
  // Noah REDPRM: a category beyond the tables, more root layers than soil layers
  CHECK(noah_land(real(0.2), real(0.8), 0) == kFatalNoahSoilTypes, "noah soil types");
  CHECK(noah_land(real(0.2), real(0.8), 1, 5) == kFatalNoahRootLayers, "noah root layers");
  // a land-use category beyond VEGPARM.TBL's LUCATS (2 in the synthetic SYNTH2 dataset): REDPRM's stop
  {
    const auto dir = std::filesystem::temp_directory_path() / ("cppwrf_test_fatal_" + std::to_string(::getpid()));
    phys::noah::NoahTables tb{};
    const bool read = synthetic_tables(dir, tb);
    CHECK(read && tb.lucats == 2 && tb.slcats == 1 && tb.slpcats == 1, "synthetic tables");
    if (read) {
      CHECK(noah_land(real(0.2), real(0.8), 1, 1, &tb, 3) == kFatalNoahLanduseTypes, "noah landuse types");
      CHECK(noah_land(real(0.2), real(0.8), 1, 1, &tb, 2) != kFatalNoahLanduseTypes, "noah landuse in table");
    }
    std::filesystem::remove_all(dir);
  }
  CHECK(std::strcmp(fatal_message(kFatalNoahLanduseTypes), "Warning: too many input landuse types") == 0, "msg");
  // (b) Kain-Fritsch codes and the messages
  CHECK(stop_code([] { column_for(one, KRaise{kFatalKfQgSurface}); }) == kFatalKfQgSurface, "kf qg");
  CHECK(stop_code([] { column_for(one, KRaise{kFatalKfIstop}); }) == kFatalKfIstop, "kf istop");
  CHECK(std::strcmp(fatal_message(kFatalSnowDensity), "Physical snow depth is less than snow water equiv.") == 0, "msg");
  CHECK(std::strcmp(fatal_message(kFatalKfQgSurface), "QG, QG(NK).LT.0") == 0, "msg");
  CHECK(std::strcmp(fatal_message(kFatalKfIstop), "KAIN-FRITSCH, istop=1, diags") == 0, "msg");
  // (c) physical snow (density 0.25) raises nothing; the flag is clear after each stop above
  CHECK(stop_code([] { column_for(one, KGlacial{real(0.2), real(0.8)}); }) == 0, "glacial ok");
  CHECK(stop_code([] { column_for(one, KSeaice{real(0.2), real(0.8)}); }) == 0, "sea ice ok");
  CHECK(stop_code([] {}) == 0, "flag cleared");
  // (d) the smallest of the codes raised in one call, on every run of it
  for (int rep = 0; rep < 20; ++rep)
    CHECK(stop_code([] { column_for(Range2D{0, 97, 0, 89}, KRaiseMany{97, 89}); }) == 3, "smallest code (rep %d)", rep);
  CHECK(stop_code([] {}) == 0, "flag cleared after the smallest code");
  {  // 2 x 2 tiles: worker t raises codes[t]; every worker stops with the smallest, 2 (worker 2's)
    tile::Options o;
    o.px = 2;
    o.py = 2;
    for (int rep = 0; rep < 10; ++rep) {
      const int codes[4] = {7, 5, 2, 9};
      std::vector<int> got(4, -1);
      tile::run(o, [&](int t) {
        got[std::size_t(t)] = stop_code([&] { raise_fatal(codes[t]); });
        return 0;
      });
      for (int t = 0; t < 4; ++t) CHECK(got[std::size_t(t)] == 2, "tile %d stopped with %d (rep %d)", t, got[std::size_t(t)], rep);
      std::vector<int> none(4, -1);  // no code raised: no worker stops, the flags are clear
      tile::run(o, [&](int t) {
        none[std::size_t(t)] = stop_code([] {});
        return 0;
      });
      for (int t = 0; t < 4; ++t) CHECK(none[std::size_t(t)] == 0, "tile %d without a code (rep %d)", t, rep);
    }
  }
  CHECK(stop_code([] {}) == 0, "flag cleared after the tiles");
  // (e) per-thread slots
  for (int rep = 0; rep < 5; ++rep) {
    int a = -1, b = -1, c = -1;
    std::thread ta([&] {
      set_fatal_slot(3);
      a = stop_code([] { column_for(Range2D{0, 40, 0, 30}, KRaise{kFatalKfIstop}); });
      dev_sync();
    });
    std::thread tb([&] {
      set_fatal_slot(5);
      b = stop_code([] { column_for(Range2D{0, 40, 0, 30}, KRaise{0x7fffffff}); column_for(one, KGlacial{real(0.2), real(0.8)}); });
    });
    ta.join();
    tb.join();
    // (KRaise{INT_MAX} raises the largest code: a slot shared with ta would read 3)
    CHECK(a == kFatalKfIstop, "slot 3 stopped with %d (rep %d)", a, rep);
    CHECK(b == 0x7fffffff, "slot 5 stopped with %d, its own code expected (rep %d)", b, rep);
    c = stop_code([] {});
    CHECK(c == 0, "slot 0 saw %d (rep %d)", c, rep);
    std::thread td([&] {
      set_fatal_slot(3);
      c = stop_code([] {});
    });
    td.join();
    CHECK(c == 0, "slot 3 cleared after its stop: %d (rep %d)", c, rep);
  }
  std::printf("%s (%d failures)\n", fails ? "FAILED" : "OK", fails);
  return fails ? 1 : 0;
}
