// SPDX-License-Identifier: GPL-3.0-or-later
// The C++ QNSE surface layer and PBL (phys/sf_qnsesfc, phys/bl_qnse; CPU FP64, or FP32 as a smoke test) for the
// columns of generate_inputs.py, in driver_qnse.f90's order: the tables, the surface layer, then the PBL.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "phys/bl_qnse/qnsepbl.hpp"
#include "phys/sf_qnsesfc/qnsesfc.hpp"

using namespace cppwrf;
using V = std::vector<real>;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: driver_qnse_cpu IN OUT\n");
    return 2;
  }
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  int32_t hd[4];
  double dt;
  if (std::fread(hd, 4, 4, f) != 4 || std::fread(&dt, 8, 1, f) != 1) return 1;
  const int n = hd[0], nz = hd[1], itimestep = hd[2], stepbl = hd[3];
  auto rd = [&](std::size_t m) {
    std::vector<double> d(m);
    if (std::fread(d.data(), 8, m, f) != m) std::fprintf(stderr, "short read\n");
    return V(d.begin(), d.end());
  };
  const std::size_t n3 = std::size_t(n) * nz;
  V dz = rd(n3), pmid = rd(n3), th = rd(n3), t = rd(n3), exner = rd(n3), qv = rd(n3), qc = rd(n3), u = rd(n3),
    v = rd(n3), tke = rd(n3), pint = rd(n3 + n);
  V ht = rd(n), tsk = rd(n), xland = rd(n), z0base = rd(n), mavail = rd(n), znt = rd(n), ust = rd(n), qsfc = rd(n),
    thz0 = rd(n), qz0 = rd(n), uz0 = rd(n), vz0 = rd(n), akhs = rd(n), akms = rd(n), hfx = rd(n), qfx = rd(n),
    corf = rd(n), chklowq = rd(n), sice = rd(n), snow = rd(n), ct = rd(n);
  std::fclose(f);
  std::vector<int> lowlyr(n, 1), kpbl(n, 0);  // qnsesfcinit: LOWLYR = 1

  auto c3 = [&](const V& x) { return View3<const real>{x.data(), n, n}; };
  auto v3 = [&](V& x) { return View3<real>{x.data(), n, n}; };
  auto c2 = [&](const V& x) { return View2<const real>{x.data(), n}; };
  auto v2 = [&](V& x) { return View2<real>{x.data(), n}; };
  const Range2D cols{0, n, 0, 1};

  // tables
  V tb(4 * phys::sf_qnsesfc::KZTM);
  phys::sf_qnsesfc::Tables bounds{};
  phys::sf_qnsesfc::QNSESFCTableManager::compute(tb.data(), tb.data() + phys::sf_qnsesfc::KZTM,
                                                 tb.data() + 2 * phys::sf_qnsesfc::KZTM,
                                                 tb.data() + 3 * phys::sf_qnsesfc::KZTM, bounds);
  phys::sf_qnsesfc::QNSESFCTableManager mgr;

  // surface layer
  std::vector<V> s(22, V(n, real(0)));  // rib .. v10e
  V pblh(n, real(0)), rmol(n, real(0));
  phys::sf_qnsesfc::QNSESFCKernel ks{};
  ks.ht = c2(ht); ks.dz = c3(dz); ks.pmid = c3(pmid); ks.pint = c3(pint); ks.th = c3(th); ks.t = c3(t);
  ks.qv = c3(qv); ks.qc = c3(qc); ks.u = c3(u); ks.v = c3(v); ks.q2 = c3(tke);
  ks.tsk = c2(tsk); ks.xland = c2(xland); ks.z0base = c2(z0base); ks.mavail = c2(mavail);
  ks.lowlyr = View2<const int>{lowlyr.data(), n};
  ks.qsfc = v2(qsfc); ks.thz0 = v2(thz0); ks.qz0 = v2(qz0); ks.uz0 = v2(uz0); ks.vz0 = v2(vz0); ks.ustar = v2(ust);
  ks.znt = v2(znt); ks.pblh = v2(pblh); ks.rmol = v2(rmol); ks.akhs = v2(akhs); ks.akms = v2(akms);
  ks.hfx = v2(hfx); ks.qfx = v2(qfx);
  ks.rib = v2(s[0]); ks.chs = v2(s[1]); ks.chs2 = v2(s[2]); ks.cqs2 = v2(s[3]); ks.flx_lh = v2(s[4]);
  ks.flhc = v2(s[5]); ks.flqc = v2(s[6]); ks.qgh = v2(s[7]); ks.cpm = v2(s[8]); ks.ct = v2(ct); ks.u10 = v2(s[9]);
  ks.v10 = v2(s[10]); ks.t02 = v2(s[11]); ks.th02 = v2(s[12]); ks.tshltr = v2(s[13]); ks.th10 = v2(s[14]);
  ks.q02 = v2(s[15]); ks.qshltr = v2(s[16]); ks.q10 = v2(s[17]); ks.pshltr = v2(s[18]); ks.u10e = v2(s[19]);
  ks.v10e = v2(s[20]);
  ks.tables = mgr.get_tables();
  ks.itimestep = itimestep;
  ks.nz = nz;
  phys::sf_qnsesfc::run_qnsesfc(ks, cols);

  std::vector<double> out;
  auto put = [&](const V& x) { out.insert(out.end(), x.begin(), x.end()); };
  put(tb);
  for (real b : {bounds.ztmin1, bounds.ztmax1, bounds.dzeta1, bounds.ztmin2, bounds.ztmax2, bounds.dzeta2})
    out.push_back(b);
  for (const V* x : {&qsfc, &thz0, &qz0, &uz0, &vz0, &ust, &znt, &pblh, &rmol, &akhs, &akms, &s[0], &s[1], &s[2],
                     &s[3], &hfx, &qfx, &s[4], &s[5], &s[6], &s[7], &s[8], &ct, &s[9], &s[10], &s[11], &s[12],
                     &s[13], &s[14], &s[15], &s[16], &s[17], &s[18], &s[19], &s[20]})
    put(*x);

  // PBL (ELFLX = the surface layer's FLX_LH)
  std::vector<V> o(9, V(n3, real(0)));  // tke exch_h exch_m el rublten rvblten rthblten rqvblten rqcblten
  o[0] = tke;
  phys::qnsepbl::QNSEPBLKernel kp{};
  kp.ht = c2(ht); kp.dz = c3(dz); kp.pmid = c3(pmid); kp.pint = c3(pint); kp.th = c3(th); kp.t = c3(t);
  kp.exner = c3(exner); kp.qv = c3(qv); kp.cwm = c3(qc); kp.u = c3(u); kp.v = c3(v);
  kp.tsk = c2(tsk); kp.chklowq = c2(chklowq); kp.corf = BView2<const real>{corf.data(), 1, n};
  kp.xland = c2(xland); kp.sice = c2(sice);
  kp.snow = c2(snow); kp.elflx = c2(s[4]); kp.hfx = c2(hfx);
  kp.lowlyr = View2<const int>{lowlyr.data(), n};
  kp.qsfc = v2(qsfc); kp.qz0 = v2(qz0); kp.thz0 = v2(thz0); kp.uz0 = v2(uz0); kp.vz0 = v2(vz0); kp.ustar = v2(ust);
  kp.znt = v2(znt); kp.ct = v2(ct); kp.akhs = v2(akhs); kp.akms = v2(akms); kp.pblh = v2(pblh);
  kp.kpbl = View2<int>{kpbl.data(), n};
  kp.tke = v3(o[0]); kp.exch_h = v3(o[1]); kp.exch_m = v3(o[2]); kp.el_myj = v3(o[3]); kp.rublten = v3(o[4]);
  kp.rvblten = v3(o[5]); kp.rthblten = v3(o[6]); kp.rqvblten = v3(o[7]); kp.rqcblten = v3(o[8]);
  kp.dt = real(dt); kp.stepbl = stepbl; kp.nz = nz;
  phys::qnsepbl::qnsepbl_reserve(cols);
  phys::qnsepbl::run_qnsepbl(kp, cols);
  for (const V& x : o) put(x);
  put(pblh);
  for (int i = 0; i < n; ++i) out.push_back(double(kpbl[i]));
  put(ct); put(qsfc); put(thz0); put(qz0);

  FILE* w = std::fopen(argv[2], "wb");
  if (!w) return 1;
  std::fwrite(out.data(), 8, out.size(), w);
  std::fclose(w);
  return 0;
}
