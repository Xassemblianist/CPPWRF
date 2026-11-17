// SPDX-License-Identifier: GPL-3.0-or-later
// The last RK3 stage of the moist species through CPPWRF's ScalarTransport::run_stage (src/dyn/scalars.cpp) on the
// inputs of generate_inputs.py, the counterpart of driver_advection.f90 (WRF's rk_update_scalar_pd,
// advect_scalar_pd / advect_scalar_mono, set_tend, rk_update_scalar). Output as the Fortran driver's.
//   driver_advection IN OUT
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/backend.hpp"
#include "core/field.hpp"
#include "core/grid.hpp"
#include "core/halo.hpp"
#include "dyn/scalars.hpp"
#include "dyn/state.hpp"

using namespace cppwrf;
using namespace cppwrf::dyn;

namespace {
constexpr int H = 5;  // the inputs' padding

struct Reader {
  std::ifstream in;
  template <class T> T get() {
    T v;
    in.read(reinterpret_cast<char*>(&v), sizeof(T));
    if (!in) throw std::runtime_error("short input");
    return v;
  }
  std::vector<double> arr(std::size_t n) {
    std::vector<double> v(n);
    in.read(reinterpret_cast<char*>(v.data()), std::streamsize(n * sizeof(double)));
    if (!in) throw std::runtime_error("short input");
    return v;
  }
};

// a padded input array (x fastest, then y, then k) into a field's allocation, halo included where it has one
template <class F> void load(F& f, const std::vector<double>& a, int nxs, int nys, int nzs) {
  const Layout& L = f.layout();
  std::vector<real> h(L.total, real(0));
  const int px = nxs + 2 * H, py = nys + 2 * H;
  for (int k = 0; k < nzs; ++k)
    for (int j = L.wj0; j < L.wj1; ++j)
      for (int i = L.wi0; i < L.wi1; ++i)
        h[L.index(i, j, k)] = real(a[(std::size_t(k) * py + (j + H)) * px + (i + H)]);
  copy_to_dev(f.data() - L.offset, h.data(), h.size() * sizeof(real));
}

void dump(std::ofstream& out, const Field3& f, int nx, int ny, int nz) {
  std::vector<real> h;
  f.download(h);
  const Layout& L = f.layout();
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        const double v = double(h[L.index(i, j, k)]);
        out.write(reinterpret_cast<const char*>(&v), sizeof v);
      }
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) { std::fprintf(stderr, "usage: driver_advection IN OUT\n"); return 2; }
  Reader r{std::ifstream(argv[1], std::ios::binary)};
  const int nx = r.get<int32_t>(), ny = r.get<int32_t>(), nz = r.get<int32_t>(), bc = r.get<int32_t>(),
            nspec = r.get<int32_t>(), p_qv = r.get<int32_t>(), adv_opt = r.get<int32_t>();
  const double dx = r.get<double>(), dy = r.get<double>(), dt = r.get<double>();
  Grid g;
  g.nx = nx; g.ny = ny; g.nz = nz; g.dx = dx; g.dy = dy; g.dt = dt;
  g.bc_x = g.bc_y = bc == 0 ? BC::Specified : bc == 1 ? BC::Periodic : BC::Open;
  State s(g);
  Tendencies t(g);
  const char* names[] = {"qv", "qc", "qr", "qi", "qs", "qg"};
  if (p_qv != 0 || nspec > 6) throw std::runtime_error("qv first, at most 6 species");
  for (int n = 0; n < nspec; ++n) {
    s.add_scalar(g, names[n], true);
    t.add_scalar(g, names[n]);
  }
  BaseState b(g, false, true);
  const auto c1h = r.arr(nz), c2h = r.arr(nz), rdnw = r.arr(nz), fnm = r.arr(nz + 1), fnp = r.arr(nz + 1);
  for (int k = 0; k < nz; ++k) { b.c1h[k] = c1h[k]; b.c2h[k] = c2h[k]; b.rdnw[k] = rdnw[k]; }
  for (int k = 0; k <= nz; ++k) { b.fnm[k] = fnm[k]; b.fnp[k] = fnp[k]; }
  b.upload_levels();
  b.rdx = 1.0 / dx;
  b.rdy = 1.0 / dy;
  const std::size_t n2 = std::size_t(nx + 2 * H) * (ny + 2 * H);
  load(b.msftx, r.arr(n2), nx, ny, 1);
  load(b.msfty, r.arr(n2), nx, ny, 1);
  load(b.mub, r.arr(n2), nx, ny, 1);
  load(s.mu_1, r.arr(n2), nx, ny, 1);
  load(s.mu_2, r.arr(n2), nx, ny, 1);
  load(s.muts, r.arr(n2), nx, ny, 1);
  load(s.ru_m, r.arr(std::size_t(nx + 1 + 2 * H) * (ny + 2 * H) * nz), nx + 1, ny, nz);
  load(s.rv_m, r.arr(std::size_t(nx + 2 * H) * (ny + 1 + 2 * H) * nz), nx, ny + 1, nz);
  load(s.ww_m, r.arr(n2 * (nz + 1)), nx, ny, nz + 1);
  for (int n = 0; n < nspec; ++n) {
    load(s.q_1[n], r.arr(n2 * nz), nx, ny, nz);
    load(s.q_2[n], r.arr(n2 * nz), nx, ny, nz);
    load(t.q_tendf[n], r.arr(n2 * nz), nx, ny, nz);
  }
  ScalarParams sp;
  sp.moist_adv_opt = adv_opt;
  sp.h_adv_order = 5;
  sp.v_adv_order = 3;
  ScalarTransport st(g, sp);
  Field3 rqvften(g, Stagger::M, "rqvften");
  st.set_qv_forcing(&rqvften);
  st.reserve(nspec);
  st.run_stage(s, t, b, 3, real(dt));
  dev_sync();
  std::ofstream out(argv[2], std::ios::binary);
  for (int n = 0; n < nspec; ++n) {
    dump(out, s.q_2[n], nx, ny, nz);
    dump(out, s.q_1[n], nx, ny, nz);
  }
  dump(out, rqvften, nx, ny, nz);
  return 0;
}
