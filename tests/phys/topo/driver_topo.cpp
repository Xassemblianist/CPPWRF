// SPDX-License-Identifier: GPL-3.0-or-later
// Terrain-option column test, CPPWRF side: radconst, topo::KTopoShad (pre_radiation_driver's toposhad_init and
// toposhad of one patch) and topo::KTopoRadAdj (TOPO_RAD_ADJ_DRVR) on the inputs of generate_inputs.py, in the layout
// of driver_topo.f90's output.
//   driver_topo_cpu INPUT OUTPUT
#include <cmath>
#include <cstdio>
#include <vector>

#include "phys/driver/radiation.hpp"
#include "phys/driver/topo.hpp"

using namespace cppwrf;

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  double h[10];
  auto rd = [&](double* p, std::size_t n) { return std::fread(p, sizeof(double), n, f) == n; };
  if (!rd(h, 10)) return 1;
  const int nx = int(std::lround(h[0])), ny = int(std::lround(h[1])), nt = int(std::lround(h[8])),
            n = int(std::lround(h[9]));
  const double dx = h[2], dy = h[3], shadlen = h[4], gmt = h[5], radfrq = h[6], julian = h[7];
  std::vector<double> times(nt);
  rd(times.data(), nt);
  const int H = 3, sx = nx + 2 * H, sy = ny + 2 * H;  // halo 3, as Field2
  auto field = [&](std::vector<double>& v) {
    v.assign(std::size_t(sx) * sy, 0.0);
    for (int j = 0; j < ny; ++j) rd(&v[std::size_t(j + H) * sx + H], nx);
  };
  std::vector<double> ht, xlat, xlong, sina, cosa, shad(std::size_t(sx) * sy, 0.0);
  field(ht); field(xlat); field(xlong); field(sina); field(cosa);
  {  // the model's ht_loc: the domain's heights with the halo of topo::ht_bc (PhysicsSuite::init_topo)
    std::vector<double> d(std::size_t(nx) * ny);
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) d[std::size_t(j) * nx + i] = ht[std::size_t(j + H) * sx + i + H];
    for (int j = -H; j < ny + H; ++j)
      for (int i = -H; i < nx + H; ++i) ht[std::size_t(j + H) * sx + i + H] = phys::driver::topo::ht_bc(d, nx, ny, i, j);
  }
  std::vector<int> mask(std::size_t(sx) * sy, 0);
  auto v2 = [&](std::vector<double>& v) { return View2<real>{&v[std::size_t(H) * sx + H], sx}; };
  auto c2 = [&](std::vector<double>& v) { return View2<const real>{&v[std::size_t(H) * sx + H], sx}; };
  double rdecl;
  rd(&rdecl, 1);
  std::vector<double> in[9];
  for (auto& a : in) { a.resize(n); rd(a.data(), n); }
  std::fclose(f);
  std::FILE* o = std::fopen(argv[2], "wb");
  for (int it = 0; it < nt; ++it) {
    double declin = 0, solcon = 0;
    phys::driver::radconst(julian, declin, solcon);
    const real xt24 = std::fmod(real(times[it]) + real(radfrq) * real(0.5), real(1440.));
    const int gpshad = int(real(shadlen) / real(dx) + real(1.));
    phys::driver::topo::KTopoShad k{c2(xlat), c2(xlong), c2(sina), c2(cosa), c2(ht), v2(shad),
                                    View2<int>{&mask[std::size_t(H) * sx + H], sx}, xt24, real(gmt), real(declin),
                                    real(dx), real(dy), gpshad, nx, ny};
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) k(i, j);
    std::fwrite(&declin, sizeof(double), 1, o);
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        const double m = mask[std::size_t(j + H) * sx + i + H];
        std::fwrite(&m, sizeof(double), 1, o);
      }
    for (int j = 0; j < ny; ++j) std::fwrite(&shad[std::size_t(j + H) * sx + H], sizeof(double), nx, o);
  }
  // xlat, coszen, diffuse_frac, swdown, gsw, hrang, slope, slp_azi, shadow
  std::vector<int> sh(n);
  for (int p = 0; p < n; ++p) sh[p] = int(std::lround(in[8][p]));
  std::vector<double> swn(n, 0.0), gss(n, 0.0);
  phys::driver::topo::KTopoRadAdj a{{in[0].data(), n}, {in[1].data(), n}, {in[2].data(), n}, {in[5].data(), n},
                                    {in[6].data(), n}, {in[7].data(), n}, {sh.data(), n}, {in[3].data(), n},
                                    {in[4].data(), n}, {swn.data(), n}, {gss.data(), n}, real(rdecl)};
  for (int p = 0; p < n; ++p) a(p, 0);
  for (auto* v : {&in[3], &in[4], &swn, &gss}) std::fwrite(v->data(), sizeof(double), n, o);
  std::fclose(o);
  return 0;
}
