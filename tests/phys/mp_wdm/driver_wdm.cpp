// SPDX-License-Identifier: GPL-3.0-or-later
// The C++ side of the WDM column test: wdm6_params (wdm6init) and Wdm6Kernel + WdmEffRadKernel on the slab of
// generate_inputs.py with the lanes i0 = 0, ni as src/dyn/physics_coupling.cpp passes them (itimestep 1: NN = ccn0
// first, as PhysicsCoupling::step); the output layout of driver_wdm.f90.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "core/backend.hpp"
#include "core/field.hpp"
#include "phys/mp_wdm6/wdm6.hpp"

using namespace cppwrf;
using namespace cppwrf::phys;

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: driver_wdm_cpu IN OUT\n"); return 2; }
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  int32_t hdr[7];
  double dt, ccn;
  if (std::fread(hdr, 4, 7, f) != 7 || std::fread(&dt, 8, 1, f) != 1 || std::fread(&ccn, 8, 1, f) != 1) return 1;
  const int ni = hdr[0], nj = hdr[1], nz = hdr[2], nsteps = hdr[3], scheme = hdr[4], hail = hdr[5], it0 = hdr[6];
  const std::size_t m = std::size_t(ni) * nz * nj, m2 = std::size_t(ni) * nj;
  std::vector<double> th(m), q(m), qc(m), qr(m), qi(m), qs(m), qg(m), qh(m), nn(m), nc(m), nr(m), den(m), pii(m),
      p(m), delz(m), rec(m, 0.), rei(m, 0.), res(m, 0.);
  std::vector<double> xland(m2), rain(m2), rainncv(m2, 0.), snow(m2), snowncv(m2, 0.), graupel(m2),
      graupelncv(m2, 0.), hail2(m2), hailncv(m2, 0.), sr(m2, 0.);
  for (auto* v : {&th, &q, &qc, &qr, &qi, &qs, &qg, &qh, &nn, &nc, &nr, &den, &pii, &p, &delz})
    if (std::fread(v->data(), 8, m, f) != m) return 1;
  for (auto* v : {&xland, &rain, &snow, &graupel, &hail2})
    if (std::fread(v->data(), 8, m2, f) != m2) return 1;
  std::fclose(f);
  // Fortran (i, k, j) order: index i + k * ni + j * ni * nz
  const int sj = ni * nz, sk = ni;
  auto V = [&](std::vector<double>& a) { return View3<double>{a.data(), sj, sk}; };
  auto CV = [&](std::vector<double>& a) { return View3<const double>{a.data(), sj, sk}; };
  auto V2 = [&](std::vector<double>& a) { return View2<double>{a.data(), ni}; };
  auto CV2 = [&](std::vector<double>& a) { return View2<const double>{a.data(), ni}; };
  FILE* o = std::fopen(argv[2], "wb");
  double cv[63] = {};
  const wdm6::Wdm6Params<double> c6 = wdm6::wdm6_params<double>(hail, ccn);
  if (scheme == 6) {
    const auto& c = c6;
    const double v[63] = {c.qc0, c.qc1, c.qck1, c.pidnc, c.g1pbr, c.g2pbr, c.g3pbr, c.g4pbr, c.g5pbr, c.g6pbr,
                          c.g7pbr, c.g5pbro2, c.g7pbro2, c.pi, c.pvtr, c.pvtrn, c.pacrr, c.pidn0r, c.pidnr,
                          c.precr1, c.precr2, c.xmmax, c.roqimax, c.g1pbs, c.g3pbs, c.g4pbs, c.g5pbso2, c.pvts,
                          c.pacrs, c.precs1, c.precs2, c.pidn0s, c.xlv1, c.pacrc, c.g1pbg, c.g3pbg, c.g4pbg,
                          c.g5pbgo2, c.pvtg, c.pacrg, c.precg1, c.precg2, c.pidn0g, c.n0g, c.avtg, c.bvtg, c.deng,
                          c.lamdagmax, c.rslopecmax, c.rslopec2max, c.rslopec3max, c.rslopermax, c.rslopesmax,
                          c.rslopegmax, c.rsloperbmax, c.rslopesbmax, c.rslopegbmax, c.rsloper2max, c.rslopes2max,
                          c.rslopeg2max, c.rsloper3max, c.rslopes3max, c.rslopeg3max};
    for (int n = 0; n < 63; ++n) cv[n] = v[n];
  }
  std::fwrite(cv, 8, 63, o);
  const Range2D r{0, ni, 0, nj};
  const double cdm2 = wsm6::rgmma_calc(5. / 3.);
  for (int s = 0; s < nsteps; ++s) {
    if (it0 + s == 1)  // wdm6: itimestep == 1 sets NN = ccn0 on the whole memory range first
      for (auto& x : nn) x = ccn;
    if (scheme == 6) {
      column_for(r, wdm6::Wdm6Kernel<double, 128>{c6, dt, nz, V(th), V(q), V(qc), V(qr), V(qi), V(qs), V(qg), V(nn),
                                                  V(nc), V(nr), CV(den), CV(pii), CV(p), CV(delz), CV2(xland),
                                                  V2(rain), V2(rainncv), V2(snow), V2(snowncv), V2(graupel),
                                                  V2(graupelncv), V2(sr), 0, ni});
      parallel_for(Range3D{0, ni, 0, nj, 0, nz},
                   wdm6::WdmEffRadKernel<double>{c6.pidnc, c6.pidn0s, c6.alpha, c6.n0s, c6.n0smax, c6.t0c, c6.qmin,
                                                 cdm2, 2.49E-6, 4.99E-6, 9.99E-6, CV(th), CV(pii), CV(qc), CV(nc),
                                                 CV(qi), CV(qs), CV(den), V(rec), V(rei), V(res)});
    }
    for (auto* v : {&th, &q, &qc, &qr, &qi, &qs, &qg, &qh, &nn, &nc, &nr, &rec, &rei, &res}) std::fwrite(v->data(), 8, m, o);
    for (auto* v : {&rain, &rainncv, &snow, &snowncv, &graupel, &graupelncv, &hail2, &hailncv, &sr})
      std::fwrite(v->data(), 8, m2, o);
  }
  std::fclose(o);
  return 0;
}
