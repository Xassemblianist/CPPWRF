// SPDX-License-Identifier: GPL-3.0-or-later
// C++ RUC LSM (src/phys/lsm_ruc) on the points of generate_inputs.py, the counterpart of driver_ruc.f90.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "phys/lsm_ruc/ruc_tables.hpp"

using namespace cppwrf::phys::ruc;
using V = std::vector<double>;
using cppwrf::real;
static std::FILE* in;
static std::FILE* out;
static void rd(V& a) {
  if (std::fread(a.data(), 8, a.size(), in) != a.size()) { std::fprintf(stderr, "short input\n"); std::exit(1); }
}
static void wr(const V& a) { std::fwrite(a.data(), 8, a.size(), out); }

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  in = std::fopen(argv[1], "rb");
  out = std::fopen(argv[2], "wb");
  V hdr(11);
  rd(hdr);
  const int n = int(hdr[0] + 0.5), nsl = int(hdr[1] + 0.5), nsteps = int(hdr[2] + 0.5);
  LsmConfig cf{};
  cf.myj = hdr[3] > 0.5; cf.frpcpn = hdr[4] > 0.5; cf.rdlai2d = hdr[5] > 0.5;
  const std::string mminlu = int(hdr[6] + 0.5) == 2 ? "MODIFIED_IGBP_MODIS_NOAH" : "USGS";
  cf.dt = hdr[7]; cf.xice_threshold = hdr[8]; cf.iswater = int(hdr[9] + 0.5); cf.isice = int(hdr[10] + 0.5);
  cf.nzs = nsl;
  // gfortran keeps SFCTMP's uninitialised ILNB in a stack slot nothing else touches in this driver: carry it
  // serially (see ruc.hpp); the model passes no carry.
  static int ilnb_gfortran = 0;
  cf.ilnb_carry = &ilnb_gfortran;
  const double cp = 7. * 287. / 2., r_d = 287., rcp = r_d / cp;
  cf.c.cp = cp; cf.c.rovcp = rcp; cf.c.g0 = 9.81; cf.c.lv = 2.5e6; cf.c.stbolt = 5.67051e-8;
  static RucTables tb;
  const std::string err = read_ruc_tables(".", mminlu, tb);
  if (!err.empty()) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
  V zs(nsl);
  rd(zs);
  for (int k = 0; k < nsl; ++k) cf.zs[k + 1] = zs[k];
  auto mk = [&] { return V(n); };
  V ivg = mk(), isl = mk(), xland = mk(), xice = mk(), tbot = mk(), shdmin = mk(), shdmax = mk(), albbck = mk(),
    vegfra = mk();
  for (V* a : {&ivg, &isl, &xland, &xice, &tbot, &shdmin, &shdmax, &albbck, &vegfra}) rd(*a);
  std::vector<LsmState> st(n);
  V tmp = mk();
  real LsmState::*init[] = {&LsmState::snow, &LsmState::snowh, &LsmState::snowc, &LsmState::canwat, &LsmState::snoalb,
                            &LsmState::alb, &LsmState::emiss, &LsmState::lai, &LsmState::mavail, &LsmState::soilt,
                            &LsmState::soilt1, &LsmState::tsnav, &LsmState::qvg, &LsmState::qcg, &LsmState::qsg,
                            &LsmState::hfx, &LsmState::qfx, &LsmState::lh, &LsmState::znt};
  for (auto m : init) {
    rd(tmp);
    for (int i = 0; i < n; ++i) st[i].*m = tmp[i];
  }
  V smois(n * nsl), tslb(n * nsl);
  rd(smois);
  rd(tslb);
  for (int i = 0; i < n; ++i) {
    LsmState& s = st[i];
    s.vegfra = vegfra[i];
    s.z0 = s.znt;
    for (int k = 1; k <= nsl; ++k) {
      s.soilmois[k] = smois[i + n * (k - 1)];
      s.tso[k] = tslb[i + n * (k - 1)];
    }
    ruclsminit_point(tb, nsl, int(ivg[i]), int(isl[i]), xice[i], s.tso, s.soilmois, s.sh2o, s.smfr3d, s.mavail, s.znt);
  }
  auto put = [&](auto get) { for (int i = 0; i < n; ++i) tmp[i] = get(st[i]); wr(tmp); };
  auto put3 = [&](auto get) {
    V a(n * nsl);
    for (int k = 1; k <= nsl; ++k) for (int i = 0; i < n; ++i) a[i + n * (k - 1)] = get(st[i], k);
    wr(a);
  };
  put([](LsmState& s) { return s.znt; });
  put([](LsmState& s) { return s.mavail; });
  put3([](LsmState& s, int k) { return s.sh2o[k]; });
  put3([](LsmState& s, int k) { return s.smfr3d[k]; });
  V f[20];
  for (int step = 1; step <= nsteps; ++step) {
    for (V& a : f) { a = mk(); rd(a); }
    cf.ktau = step;
    V cqs = mk(), chs = mk(), t2 = mk(), th2 = mk(), q2 = mk();
    for (int i = 0; i < n; ++i) {
      LsmForcing fo{};
      fo.rainbl = f[0][i]; fo.rainncv = f[1][i]; fo.snowncv = f[2][i]; fo.graupelncv = f[3][i]; fo.frzfrac = f[4][i];
      fo.glw = f[5][i]; fo.gsw = f[6][i]; fo.flhc = f[7][i]; fo.flqc = f[8][i]; fo.chs = f[9][i];
      fo.t3d = f[10][i]; fo.qv3d = f[11][i]; fo.qc3d = f[12][i]; fo.p8w = f[13][i]; fo.rho3d = f[14][i];
      fo.z3d = f[15][i];
      fo.albbck = albbck[i]; fo.xice = xice[i]; fo.xland = xland[i]; fo.tbot = tbot[i]; fo.shdmin = shdmin[i];
      fo.shdmax = shdmax[i]; fo.ivgtyp = int(ivg[i]); fo.isltyp = int(isl[i]);
      LsmState& s = st[i];
      lsmruc_point(cf, tb, fo, s);
      cqs[i] = fo.flqc / (s.mavail * fo.rho3d);
      chs[i] = fo.flhc / (f[16][i] * fo.rho3d);
      sfcdiags_ruclsm(s.hfx, s.qfx, s.soilt, s.qsfc, cqs[i], f[17][i], f[18][i], fo.t3d, fo.qv3d, fo.rho3d, fo.p8w,
                      f[19][i], cp, rcp, t2[i], th2[i], q2[i]);
    }
    for (auto m : init) put([m](LsmState& s) { return s.*m; });
    for (auto m : {&LsmState::vegfra, &LsmState::z0, &LsmState::sfcexc, &LsmState::sfcevp, &LsmState::sfcrunoff,
                   &LsmState::udrunoff, &LsmState::acrunoff, &LsmState::grdflx, &LsmState::snom, &LsmState::dew,
                   &LsmState::qsfc, &LsmState::chklowq, &LsmState::smavail, &LsmState::smmax, &LsmState::rhosnf,
                   &LsmState::precipfr, &LsmState::snowfallac})
      put([m](LsmState& s) { return s.*m; });
    wr(cqs); wr(chs); wr(t2); wr(th2); wr(q2);
    put3([](LsmState& s, int k) { return s.soilmois[k]; });
    put3([](LsmState& s, int k) { return s.sh2o[k]; });
    put3([](LsmState& s, int k) { return s.tso[k]; });
    put3([](LsmState& s, int k) { return s.smfr3d[k]; });
    put3([](LsmState& s, int k) { return s.keepfr3dflag[k]; });
  }
  std::fclose(out);
  return 0;
}
