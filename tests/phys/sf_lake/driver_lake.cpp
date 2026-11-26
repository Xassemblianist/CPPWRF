// SPDX-License-Identifier: GPL-3.0-or-later
// Lake model column test, C++ side: lakeini_points and lake_point (src/phys/sf_lake) on the points and forcing of
// lake_input.bin, written in the layout of driver_lake.f90 (Fortran order: point fastest, then level).
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "core/backend.hpp"
#include "phys/sf_lake/lake_init.hpp"

using namespace cppwrf;
using namespace cppwrf::phys::lake;

static std::FILE* fi;
static std::FILE* fo;
static std::vector<double> rd(std::size_t n) {
  std::vector<double> v(n);
  if (std::fread(v.data(), sizeof(double), n, fi) != n) throw std::runtime_error("short input");
  return v;
}
static void wr(const std::vector<double>& v) { std::fwrite(v.data(), sizeof(double), v.size(), fo); }

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  fi = std::fopen(argv[1], "rb");
  fo = std::fopen(argv[2], "wb");
  const std::vector<double> h = rd(10);
  const int n = int(h[0]), nsteps = int(h[1]);
  const real dt = real(h[2]);
  LakeIniOptions o;
  o.lakedepth_default = real(h[3]);
  o.use_lakedepth = int(h[4]);
  o.lake_depth_flag = int(h[5]);
  o.lakeflag = int(h[6]);
  o.lake_min_elev = real(h[7]);
  o.xice_threshold = real(h[8]);
  o.iswater = int(h[9]);
  const auto ht = rd(n), snow = rd(n), tsk0 = rd(n), xice0 = rd(n), xland0 = rd(n), lm0 = rd(n), ldep = rd(n),
             xlat = rd(n), isl = rd(n), ivg = rd(n);
  std::vector<LakeIniIn> pts(n);
  std::vector<int> ivgtyp(n);
  std::vector<real> xland(n), xice(n), lakemask(n), tsk(n);
  for (int p = 0; p < n; ++p) {
    pts[p] = {real(ht[p]), real(snow[p]), real(tsk0[p]), real(ldep[p]), int(std::lround(isl[p]))};
    ivgtyp[p] = int(std::lround(ivg[p]));
    xland[p] = real(xland0[p]); xice[p] = real(xice0[p]); lakemask[p] = real(lm0[p]); tsk[p] = real(tsk0[p]);
  }
  std::vector<LakeInitPoint> st;
  try {
    lakeini_points(o, pts, ivgtyp, xland, xice, lakemask, st);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  auto col2 = [&](auto get) { std::vector<double> v(n); for (int p = 0; p < n; ++p) v[p] = double(get(st[p])); return v; };
  auto col3 = [&](int k0, int k1, auto get) {
    std::vector<double> v;
    for (int k = k0; k <= k1; ++k) for (int p = 0; p < n; ++p) v.push_back(double(get(st[p], k)));
    return v;
  };
  {
    std::vector<double> a(n);
    for (int p = 0; p < n; ++p) a[p] = ivgtyp[p];
    wr(a);
    for (int p = 0; p < n; ++p) a[p] = xland[p];
    wr(a);
    for (int p = 0; p < n; ++p) a[p] = xice[p];
    wr(a);
    for (int p = 0; p < n; ++p) a[p] = lakemask[p];
    wr(a);
  }
  wr(col3(1, 10, [](auto& q, int k) { return q.watsat[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.csol[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.tkmg[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.tkdry[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.tksatu[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.z_lake[k]; }));
  wr(col3(1, 10, [](auto& q, int k) { return q.dz_lake[k]; }));
  wr(col2([](auto& q) { return q.lakedepth2d; }));
  auto put_state = [&]() {
    wr(col2([](auto& q) { return q.savedtke12d; }));
    wr(col2([](auto& q) { return q.snowdp2d; }));
    wr(col2([](auto& q) { return q.h2osno2d; }));
    wr(col2([](auto& q) { return q.snl2d; }));
    wr(col2([](auto& q) { return q.t_grnd2d; }));
    wr(col3(1, 10, [](auto& q, int k) { return q.t_lake[k]; }));
    wr(col3(1, 10, [](auto& q, int k) { return q.lake_icefrac[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.t_soisno[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.h2osoi_ice[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.h2osoi_liq[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.h2osoi_vol[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.z[k]; }));
    wr(col3(0, 14, [](auto& q, int k) { return q.dz[k]; }));
    wr(col3(0, 15, [](auto& q, int k) { return q.zi[k]; }));
  };
  put_state();
  // Lake's outputs keep their values at non-lake points (Fortran: -1 before the first step)
  std::vector<double> hfx(n, -1), lh(n, -1), grdflx(n, -1), qfx(n, -1), t2(n, -1), th2(n, -1), q2(n, -1);
  const real rcp = real(287.) / (real(7.) * real(287.) / real(2.));  // module_model_constants: r_d / cp
  int n_imelt = 0;
  for (int s = 0; s < nsteps; ++s) {
    const auto t = rd(n), p1 = rd(n), p2 = rd(n), dz = rd(n), qv = rd(n), u = rd(n), v = rd(n), glw = rd(n),
               em = rd(n), rb = rd(n), sw = rd(n);
    std::vector<double> alb = rd(n);
    std::vector<double> tskd(n);
    for (int p = 0; p < n; ++p) {
      if (lakemask[p] == real(1)) {
        LakeInitPoint& q = st[p];
        LakeSoil so{};
        LakeColumn c{};
        c.savedtke1 = q.savedtke12d; c.snowdp = q.snowdp2d; c.h2osno = q.h2osno2d; c.snl = int(q.snl2d);
        c.t_grnd = q.t_grnd2d; c.lakedepth = q.lakedepth2d;
        for (int k = 1; k <= 10; ++k) {
          c.t_lake[k] = q.t_lake[k]; c.lake_icefrac[k] = q.lake_icefrac[k]; c.z_lake[k] = q.z_lake[k];
          c.dz_lake[k] = q.dz_lake[k]; so.watsat[k] = q.watsat[k]; so.csol[k] = q.csol[k]; so.tkmg[k] = q.tkmg[k];
          so.tkdry[k] = q.tkdry[k]; so.tksatu[k] = q.tksatu[k];
        }
        for (int k = 0; k < 15; ++k) {
          c.t_soisno[k] = q.t_soisno[k]; c.h2osoi_ice[k] = q.h2osoi_ice[k]; c.h2osoi_liq[k] = q.h2osoi_liq[k];
          c.h2osoi_vol[k] = q.h2osoi_vol[k]; c.z[k] = q.z[k]; c.dz[k] = q.dz[k];
        }
        for (int k = 0; k < 16; ++k) c.zi[k] = q.zi[k];
        LakeForcing f{real(t[p]), real(p1[p]), real(p2[p]), real(dz[p]), real(qv[p]), real(u[p]), real(v[p]),
                      real(glw[p]), real(em[p]), real(rb[p]), real(sw[p]), real(alb[p]), real(xlat[p])};
        LakeOut out{};
        lake_point(f, dt, rcp, so, c, out);
        if (int code = fatal_code_host[0]) {
          // a new snow layer with liquid water: WRF reads its undefined imelt(0) there (the model stops); in this
          // driver WRF's stack holds no 1 there (imelt(0) = 1 makes the comparison fail), the port takes 0
          if (code != kFatalLakeImelt) { std::fprintf(stderr, "fatal %d at point %d step %d\n", code, p, s + 1); return 1; }
          fatal_code_host[0] = 0;
          ++n_imelt;
        }
        hfx[p] = out.hfx; lh[p] = out.lh; grdflx[p] = out.grdflx; tsk[p] = out.tsk; qfx[p] = out.qfx; t2[p] = out.t2;
        th2[p] = out.th2; q2[p] = out.q2; alb[p] = out.albedo;
        q.savedtke12d = real(c.savedtke1); q.snowdp2d = real(c.snowdp); q.h2osno2d = real(c.h2osno);
        q.snl2d = real(c.snl); q.t_grnd2d = real(c.t_grnd);
        for (int k = 1; k <= 10; ++k) { q.t_lake[k] = real(c.t_lake[k]); q.lake_icefrac[k] = real(c.lake_icefrac[k]); }
        for (int k = 0; k < 15; ++k) {
          q.t_soisno[k] = real(c.t_soisno[k]); q.h2osoi_ice[k] = real(c.h2osoi_ice[k]);
          q.h2osoi_liq[k] = real(c.h2osoi_liq[k]); q.h2osoi_vol[k] = real(c.h2osoi_vol[k]); q.z[k] = real(c.z[k]);
          q.dz[k] = real(c.dz[k]);
        }
        for (int k = 0; k < 16; ++k) q.zi[k] = real(c.zi[k]);
      }
      tskd[p] = tsk[p];
    }
    wr(hfx); wr(lh); wr(grdflx); wr(tskd); wr(qfx); wr(t2); wr(th2); wr(q2); wr(alb);
    put_state();
  }
  std::fclose(fo);
  std::printf("new snow layers with liquid water (undefined imelt in WRF): %d\n", n_imelt);
  return 0;
}
