// SPDX-License-Identifier: GPL-3.0-or-later
// The C++ EDKF mass flux (phys/bl_qnse/mfshconv; CPU FP64, or FP32 as a smoke test) on the tile of
// generate_inputs.py, with the outputs of driver_mf.f90 in its order.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "phys/bl_qnse/mfshconv.hpp"

using namespace cppwrf;
using V = std::vector<real>;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: driver_mf_cpu IN OUT\n");
    return 2;
  }
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  int32_t hd[4];
  double dt;
  if (std::fread(hd, 4, 4, f) != 4 || std::fread(&dt, 8, 1, f) != 1) return 1;
  const int nx = hd[0], ny = hd[1], nz = hd[2], stepbl = hd[3], n = nx * ny;
  auto rd = [&](std::size_t m) {
    std::vector<double> d(m);
    if (std::fread(d.data(), 8, m, f) != m) std::fprintf(stderr, "short read\n");
    return V(d.begin(), d.end());
  };
  const std::size_t n3 = std::size_t(n) * nz;
  V dz = rd(n3), rho = rd(n3), pmid = rd(n3), th = rd(n3), exner = rd(n3), qv = rd(n3), qc = rd(n3), u = rd(n3),
    v = rd(n3), tke = rd(n3), rc_mf = rd(n3);
  V hfx = rd(n), qfx = rd(n);
  std::fclose(f);

  auto c3 = [&](const V& x) { return View3<const real>{x.data(), nx, n}; };
  auto v3 = [&](V& x) { return View3<real>{x.data(), nx, n}; };
  std::vector<V> o(11, V(n3, real(0)));  // rthblten rqvblten rqcblten rublten rvblten rc_mf emf entr thl_up rt_up frac
  o[5] = rc_mf;
  phys::mfshconv::MFSHCONVKernel k{};
  k.dz = c3(dz); k.rho = c3(rho); k.pmid = c3(pmid); k.th = c3(th); k.exner = c3(exner); k.qv = c3(qv);
  k.tke = c3(tke); k.hfx = View2<const real>{hfx.data(), nx}; k.qfx = View2<const real>{qfx.data(), nx};
  k.rthblten = v3(o[0]); k.rqvblten = v3(o[1]); k.rc_mf = v3(o[5]);
  k.emf_d = v3(o[6]); k.entr_d = v3(o[7]); k.thl_up_d = v3(o[8]); k.rt_up_d = v3(o[9]); k.frac_up_d = v3(o[10]);
  k.dt = real(dt); k.stepbl = stepbl; k.nz = nz;
  k.tile_i0 = 0; k.tile_j0 = 0; k.tile_nx = nx; k.tile_n = n;
  const Range2D cols{0, nx, 0, ny};
  phys::mfshconv::mfshconv_reserve(cols);
  phys::mfshconv::run_mfshconv(k, cols);
  (void)qc; (void)u; (void)v;

  std::vector<double> out;
  for (const V& x : o) out.insert(out.end(), x.begin(), x.end());
  FILE* w = std::fopen(argv[2], "wb");
  if (!w) return 1;
  std::fwrite(out.data(), 8, out.size(), w);
  std::fclose(w);
  return 0;
}
