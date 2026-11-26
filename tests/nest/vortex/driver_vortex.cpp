// SPDX-License-Identifier: GPL-3.0-or-later
// Vortex-following nest test, CPPWRF side: src/nest/vortex_center.hpp (height_column, search, decide) in the
// sequence of driver_vortex.f90 on the same input; the output has the same layout.
//   driver_vortex_cpu INPUT OUTPUT
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "nest/vortex_center.hpp"

using cppwrf::real;
namespace vx = cppwrf::nest::vortex;

namespace {
std::FILE* out = nullptr;
void put(const std::vector<double>& v) { std::fwrite(v.data(), sizeof(double), v.size(), out); }
template <class T>
void get(std::FILE* f, std::vector<T>& v) {
  std::vector<double> d(v.size());
  if (std::fread(d.data(), sizeof(double), d.size(), f) != d.size()) throw std::runtime_error("short input");
  for (std::size_t n = 0; n < v.size(); ++n) v[n] = T(d[n]);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: driver_vortex_cpu INPUT OUTPUT\n");
    return 2;
  }
  std::FILE* in = std::fopen(argv[1], "rb");
  out = std::fopen(argv[2], "wb");
  if (!in || !out) return 1;
  std::vector<double> h(8);
  get(in, h);
  const int nx = int(h[0]), ny = int(h[1]), nz = int(h[2]), track_level = int(h[3]), vi = int(h[4]), mvs = int(h[5]),
            pgr = int(h[6]);
  const real dx = real(h[7]);
  vx::Centre c;
  const std::size_t n3w = std::size_t(nx) * (nz + 1) * ny, n3 = std::size_t(nx) * nz * ny, n2 = std::size_t(nx) * ny;
  for (int set = 1; set <= 2; ++set) {
    // Fortran order (i, k, j)
    std::vector<real> ph(n3w), phb(n3w), p(n3), pb(n3), psfc(n2), ht(n2);
    get(in, ph);
    get(in, phb);
    get(in, p);
    get(in, pb);
    get(in, psfc);
    get(in, ht);
    std::vector<real> height(n2), cph(nz + 1), cphb(nz + 1), cp(nz), cpb(nz), pres(nz), geo(nz);
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        for (int k = 0; k <= nz; ++k) {
          cph[k] = ph[(std::size_t(j) * (nz + 1) + k) * nx + i];
          cphb[k] = phb[(std::size_t(j) * (nz + 1) + k) * nx + i];
        }
        for (int k = 0; k < nz; ++k) {
          cp[k] = p[(std::size_t(j) * nz + k) * nx + i];
          cpb[k] = pb[(std::size_t(j) * nz + k) * nx + i];
        }
        if (!vx::height_column(cph.data(), cphb.data(), cp.data(), cpb.data(), nz, track_level, pres.data(), geo.data(),
                               height[std::size_t(j) * nx + i]))
          throw std::runtime_error("interp too high");
      }
    put(std::vector<double>(height.begin(), height.end()));
    const vx::Found f = vx::search(height, psfc, ht, nx, ny, dx, vi, mvs, c);
    put({double(c.xi), double(c.xj), double(c.vc_i), double(c.vc_j), double(f.pmin), double(f.imploc),
         double(f.jmploc), double(f.imloc), double(f.jmloc), double(f.is), double(f.ie), double(f.js), double(f.je)});
    for (int r = 0; r < (set == 1 ? 2 : 1); ++r) {
      const vx::Decision d = vx::decide(nx + 1, ny + 1, pgr, c);
      put({double(d.dc_i), double(d.dc_j), double(d.disp_x), double(d.disp_y), double(d.move_cd_x),
           double(d.move_cd_y), double(c.vc_i), double(c.vc_j)});
    }
  }
  std::fclose(in);
  std::fclose(out);
  return 0;
}
