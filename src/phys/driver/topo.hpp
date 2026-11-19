// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// WRF v4.8.0's terrain options of the physics:
//  - start_em (dyn_em/start_em.F:1574-1666, not a restart): the terrain slopes TOPOSLPX / TOPOSLPY, the laplacian
//    LAP_HGT, SLOPE and SLP_AZI, and the topographic wind correction factors CTOPO / CTOPO2 of topo_wind = 1
//    (Jimenez and Dudhia: VAR_SSO and LAP_HGT) or 2 (Mass and Ovens: VAR);
//  - pre_radiation_driver (phys/module_radiation_driver.F:3285-3463) with toposhad_init / toposhad: the shadow mask
//    SHADOWMASK and the shadow height HT_SHAD of topo_shading = 1 (one patch: niter = 1, nested = .false.);
//  - surface_driver (phys/module_surface_driver.F:1981-2000, 4577-4600) with TOPO_RAD_ADJ_DRVR / TOPO_RAD_ADJ:
//    slope_rad = 1 changes SWDOWN and GSW for the surface schemes and puts them back at the end (SWNORM keeps the
//    slope-affected SWDOWN).
// Statement-by-statement ports; the kernels run on the CPU and the GPU (CPPWRF_HD).
#include <algorithm>
#include <cmath>
#include <vector>

#include "core/backend.hpp"
#include "core/field.hpp"

namespace cppwrf::phys::driver::topo {

// start_em's terrain arrays (domain points, i + nx*j): toposlpx, toposlpy, lap_hgt, slope, slp_azi
struct Slopes {
  std::vector<double> toposlpx, toposlpy, lap_hgt, slope, slp_azi;
};

// start_em (dyn_em/start_em.F:1578-1615) for the domain's mass points (ht, msftx, msfty, sina, cosa: i + nx*j; a run
// without map factors: msftx = msfty = 1, sina = 0, cosa = 1); periodic_x / periodic_y wrap the neighbours
inline Slopes start_em_slopes(int nx, int ny, double dx, double dy, bool periodic_x, bool periodic_y,
                              const std::vector<double>& ht, const std::vector<double>* msftx,
                              const std::vector<double>* msfty, const std::vector<double>* sina,
                              const std::vector<double>* cosa) {
  const std::size_t n2 = std::size_t(nx) * ny;
  Slopes s;
  s.toposlpx.assign(n2, 0.0); s.toposlpy.assign(n2, 0.0); s.lap_hgt.assign(n2, 0.0);
  s.slope.assign(n2, 0.0); s.slp_azi.assign(n2, 0.0);
  const real rdx = real(1.) / real(dx), rdy = real(1.) / real(dy);  // grid%rdx, grid%rdy
  auto H = [&](int i, int j) { return real(ht[std::size_t(j) * nx + i]); };
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      const int im1 = periodic_x ? (i - 1 + nx) % nx : std::max(i - 1, 0);
      const int ip1 = periodic_x ? (i + 1) % nx : std::min(i + 1, nx - 1);
      const int jm1 = periodic_y ? (j - 1 + ny) % ny : std::max(j - 1, 0);
      const int jp1 = periodic_y ? (j + 1) % ny : std::min(j + 1, ny - 1);
      const int di = periodic_x ? 2 : ip1 - im1, dj = periodic_y ? 2 : jp1 - jm1;  // WRF's (ip1-im1), (jp1-jm1)
      const std::size_t q = std::size_t(j) * nx + i;
      const real msx = msftx ? real((*msftx)[q]) : real(1), msy = msfty ? real((*msfty)[q]) : real(1);
      const real hx = (H(ip1, j) - H(im1, j)) * msx * rdx / real(di);
      const real hy = (H(i, jp1) - H(i, jm1)) * msy * rdy / real(dj);
      s.toposlpx[q] = double(hx);
      s.toposlpy[q] = double(hy);
      s.lap_hgt[q] = double((H(ip1, j) + H(im1, j) + H(i, jp1) + H(i, jm1) - H(i, j) * real(4.)) / real(4.));
      const real pi = real(4.) * libm::atan(real(1.));
      real sl = libm::atan(libm::pow(hx * hx + hy * hy, real(.5)));  // (hx**2+hy**2)**.5: gfortran calls pow
      real az = 0;
      if (sl < real(1.e-4)) {
        sl = 0;
      } else {
        az = libm::atan2(hx, hy) + pi;
        // rotate the slope azimuth to the lat-lon grid
        const real sa = sina ? real((*sina)[q]) : real(0), ca = cosa ? real((*cosa)[q]) : real(1);
        if (ca >= 0) az = az - libm::asin(sa);
        else az = az - (pi - libm::asin(sa));
      }
      s.slope[q] = double(sl);
      s.slp_azi[q] = double(az);
    }
  return s;
}

// start_em (dyn_em/start_em.F:1618-1666): CTOPO = CTOPO2 = 1, then for topo_wind = 1 the subgrid-orography factor
// from VAR_SSO over land and its reduction in valleys (LAP_HGT < -10 m), for topo_wind = 2 the factor
// min(1.575, VAR*0.4/200 + 1.175)**2 over land (CTOPO2 stays 1)
inline void start_em_ctopo(int topo_wind, const std::vector<double>& xland, const std::vector<double>& var_sso,
                           const std::vector<double>& var2d, const std::vector<double>& lap_hgt,
                           std::vector<double>& ctopo, std::vector<double>& ctopo2) {
  const std::size_t n2 = xland.size();
  ctopo.assign(n2, double(real(1.)));
  ctopo2.assign(n2, double(real(1.)));
  for (std::size_t q = 0; q < n2; ++q) {
    real ct = real(1.), ct2 = real(1.);
    if (topo_wind == 1) {
      if (real(xland[q]) < real(1.5)) ct = std::sqrt(real(var_sso[q]));
      if (ct <= real(2.718)) ct = real(1.);
      else ct = libm::log(ct);  // alog
      const real lap = real(lap_hgt[q]);
      if (lap > real(-10.)) {
        // ctopo unchanged
      } else if (lap >= real(-20)) {
        const real alpha = (lap + real(20.)) / real(10.);
        ct = alpha * ct + (real(1) - alpha);
      } else if (lap >= real(-30.)) {
        ct = (lap + real(30.)) / real(10.);
        ct2 = (lap + real(30.)) / real(10.);
      } else {
        ct = real(0.);
        ct2 = real(0.);
      }
    } else if (topo_wind == 2) {
      real vfac;
      if (real(xland[q]) < real(1.5)) {
        const real v = real(var2d[q]) * real(0.4) / real(200.) + real(1.175);
        vfac = real(1.575) < v ? real(1.575) : v;  // amin1(1.575, ...)
        vfac = vfac * vfac;
      } else {  // over water, leave it alone
        vfac = real(1.);
      }
      ct = ct * vfac;
    }
    ctopo[q] = double(ct);
    ctopo2[q] = double(ct2);
  }
}

// pre_radiation_driver's toposhad_init (iter = 1, not nested) and toposhad (iter = 1) for one point of a run with
// one patch (ips = jps = 1, ipe = nx, jpe = ny in WRF's indices: min(ipe, ide-1)): SHADOWMASK = 0 and
// start_em's set_physical_bc2d(grid%ht, 'r') on a specified-boundary domain (module_bc.F open_xs / open_xe /
// open_ys / open_ye copies): the height beyond the domain is the nearest edge point's (x first, then y over the
// whole memory row, so a corner takes the corner point's). (i, j) 0-based, any point of the halo.
inline double ht_bc(const std::vector<double>& ht, int nx, int ny, int i, int j) {
  const int ic = i < 0 ? 0 : (i > nx - 1 ? nx - 1 : i), jc = j < 0 ? 0 : (j > ny - 1 ? ny - 1 : j);
  return ht[std::size_t(jc) * nx + ic];
}

// HT_SHAD = HT_LOC - 0.001, then the search along the sun's direction for terrain above the sun's elevation within
// gpshad points. ht_loc: pre_radiation_driver's copy of HT over the whole memory, its halo as ht_bc (the search reads
// up to 2 points beyond the domain).
struct KTopoShad {
  View2<const real> xlat, xlong, sina, cosa, ht_loc;
  View2<real> ht_shad;
  View2<int> shadowmask;
  real xt24, gmt, declin, dx, dy;
  int gpshad, nx, ny;
  CPPWRF_HD real H(int I, int J) const { return ht_loc(I - 1, J - 1); }  // WRF's 1-based indices
  CPPWRF_HD void operator()(int i0, int j0) const {
    constexpr real piconst = real(3.1415926535897932384626433);
    constexpr real DEGRAD = piconst / real(180.);  // module_model_constants
    const int i = i0 + 1, j = j0 + 1;
    const int ips = 1, jps = 1, ipe = nx, jpe = ny;
    // toposhad_init (iter = 1, nested = .false.)
    shadowmask(i0, j0) = 0;
    ht_shad(i0, j0) = H(i, j) - real(0.001);
    // toposhad (iter = 1)
    const real pi = real(4.) * libm::atan(real(1.));
    const real TLOCTM = gmt + xt24 / real(60.) + xlong(i0, j0) / real(15.);
    const real HRANG = real(15.) * (TLOCTM - real(12.)) * DEGRAD;
    const real XXLAT = xlat(i0, j0) * DEGRAD;
    const real CSZA = libm::sin(XXLAT) * libm::sin(declin) + libm::cos(XXLAT) * libm::cos(declin) * libm::cos(HRANG);
    if (CSZA < real(1.e-2)) {  // shadow mask does not need to be computed
      shadowmask(i0, j0) = 0;
      ht_shad(i0, j0) = H(i, j) - real(0.001);
      return;
    }
    // solar azimuth angle
    real argu = (CSZA * libm::sin(XXLAT) - libm::sin(declin)) / (libm::sin(libm::acos(CSZA)) * libm::cos(XXLAT));
    if (argu > real(1)) argu = real(1);
    if (argu < real(-1)) argu = real(-1);
    real sol_azi = std::copysign(libm::acos(argu), libm::sin(HRANG)) + pi;  // sign(acos(argu), sin(HRANG)) + pi
    if (cosa(i0, j0) >= real(0)) sol_azi = sol_azi + libm::asin(sina(i0, j0));  // rotation towards the WRF grid
    else sol_azi = sol_azi + pi - libm::asin(sina(i0, j0));
    // scan for higher surrounding topography
    auto hit = [&](real topoelev, real dxabs) {
      if (libm::sin(topoelev) >= CSZA) {
        shadowmask(i0, j0) = 1;
        const real h = H(i, j) + dxabs * (libm::tan(topoelev) - libm::tan(libm::asin(CSZA)));
        ht_shad(i0, j0) = ht_shad(i0, j0) > h ? ht_shad(i0, j0) : h;  // max(ht_shad, ...)
      }
    };
    if ((sol_azi > real(1.75) * pi) || (sol_azi < real(0.25) * pi)) {  // sun in the northern quarter
      for (int jj = j + 1; jj <= j + gpshad; ++jj) {
        const real ri = real(i) + real(jj - j) * libm::tan(sol_azi);
        const int i1 = int(ri), i2 = i1 + 1;
        const real wgt = ri - real(i1);
        const real a = dy * real(jj - j), b = dx * (ri - real(i));
        const real dxabs = std::sqrt(a * a + b * b);
        if ((jj >= jpe + 3) || (i1 <= ips - 3) || (i2 >= ipe + 3)) return;
        hit(libm::atan((wgt * H(i2, jj) + (real(1.) - wgt) * H(i1, jj) - H(i, j)) / dxabs), dxabs);
      }
    } else if (sol_azi < real(0.75) * pi) {  // sun in the eastern quarter
      for (int ii = i + 1; ii <= i + gpshad; ++ii) {
        const real rj = real(j) - real(ii - i) * libm::tan(pi / real(2.) + sol_azi);
        const int j1 = int(rj), j2 = j1 + 1;
        const real wgt = rj - real(j1);
        const real a = dx * real(ii - i), b = dy * (rj - real(j));
        const real dxabs = std::sqrt(a * a + b * b);
        if ((ii >= ipe + 3) || (j1 <= jps - 3) || (j2 >= jpe + 3)) return;
        hit(libm::atan((wgt * H(ii, j2) + (real(1.) - wgt) * H(ii, j1) - H(i, j)) / dxabs), dxabs);
      }
    } else if (sol_azi < real(1.25) * pi) {  // sun in the southern quarter
      for (int jj = j - 1; jj >= j - gpshad; --jj) {
        const real ri = real(i) + real(jj - j) * libm::tan(sol_azi);
        const int i1 = int(ri), i2 = i1 + 1;
        const real wgt = ri - real(i1);
        const real a = dy * real(jj - j), b = dx * (ri - real(i));
        const real dxabs = std::sqrt(a * a + b * b);
        if ((jj <= jps - 3) || (i1 <= ips - 3) || (i2 >= ipe + 3)) return;
        hit(libm::atan((wgt * H(i2, jj) + (real(1.) - wgt) * H(i1, jj) - H(i, j)) / dxabs), dxabs);
      }
    } else {  // sun in the western quarter
      for (int ii = i - 1; ii >= i - gpshad; --ii) {
        const real rj = real(j) - real(ii - i) * libm::tan(pi / real(2.) + sol_azi);
        const int j1 = int(rj), j2 = j1 + 1;
        const real wgt = rj - real(j1);
        const real a = dx * real(ii - i), b = dy * (rj - real(j));
        const real dxabs = std::sqrt(a * a + b * b);
        if ((ii <= ips - 3) || (j1 <= jps - 3) || (j2 >= jpe + 3)) return;
        hit(libm::atan((wgt * H(ii, j2) + (real(1.) - wgt) * H(ii, j1) - H(i, j)) / dxabs), dxabs);
      }
    }
  }
};

// TOPO_RAD_ADJ (module_surface_driver.F:7187-7254) for one point: SWDOWN_IN with the slope and the shading
CPPWRF_HD inline real topo_rad_adj(real xlat1, real coszen, real diffuse_frac_in, real declin, real degrad,
                                   real swdown_in, real hrang, real slope, real slp_azi, int shadow) {
  real swdown_teradj = swdown_in;
  const real CSZA = coszen;
  const real XXLAT = xlat1 * degrad;
  if (CSZA <= real(1.E-4)) return swdown_teradj;  // night
  const real diffuse_frac = diffuse_frac_in;
  real corr_fac;
  if ((slope == real(0)) || (diffuse_frac == real(1)) || (CSZA <= real(1.e-4))) {
    // no topographic effects when all radiation is diffuse or the sun is too close to the horizon
    corr_fac = real(1);
    if (shadow == 1) corr_fac = diffuse_frac;
  } else {
    // cosine of the zenith angle over sloping topography
    const real sx = libm::sin(XXLAT), cx = libm::cos(XXLAT), sh = libm::sin(hrang), ch = libm::cos(hrang);
    const real sa = libm::sin(slp_azi), ca = libm::cos(slp_azi), ss = libm::sin(slope), cs = libm::cos(slope);
    real csza_slp = ((sx * ch) * (-(ca * ss)) - sh * (sa * ss) + (cx * ch) * cs) * libm::cos(declin) +
                    (cx * (ca * ss) + sx * cs) * libm::sin(declin);
    if (csza_slp <= real(1.E-4)) csza_slp = real(0);
    if (shadow == 1) csza_slp = real(0);  // topographic shading
    // the diffuse fraction of the solar radiation is assumed to be unaffected by the slope
    corr_fac = diffuse_frac + (real(1) - diffuse_frac) * csza_slp / CSZA;
  }
  swdown_teradj = (real(1.)) * swdown_in * corr_fac;
  return swdown_teradj;
}

// TOPO_RAD_ADJ_DRVR (module_surface_driver.F:7117-7184): SWNORM = SWDOWN; in daytime GSWSAVE = GSW, then SWDOWN and
// GSW scaled by the slope / shading factor
struct KTopoRadAdj {
  View2<const real> xlat, coszen, diffuse_frac, hrang, slope, slp_azi;
  View2<const int> shadowmask;
  View2<real> swdown, gsw, swnorm, gswsave;
  real declin;
  CPPWRF_HD void operator()(int i, int j) const {
    const real pi = real(4.) * libm::atan(real(1.));
    const real degrad = pi / real(180.);
    swnorm(i, j) = swdown(i, j);  // save
    if (swdown(i, j) > real(1.E-3)) {  // daytime
      const int shadow = shadowmask(i, j);
      const real swdown_teradj = topo_rad_adj(xlat(i, j), coszen(i, j), diffuse_frac(i, j), declin, degrad,
                                              swdown(i, j), hrang(i, j), slope(i, j), slp_azi(i, j), shadow);
      gswsave(i, j) = gsw(i, j);  // save
      gsw(i, j) = gsw(i, j) * swdown_teradj / swdown(i, j);
      swdown(i, j) = swdown_teradj;
    }
  }
};

// the end of surface_driver (module_surface_driver.F:4577-4600): SWDOWN back to the unaffected value for the output,
// SWNORM the slope-affected one, GSW back
struct KTopoRadRestore {
  View2<real> swdown, gsw, swnorm;
  View2<const real> gswsave;
  CPPWRF_HD void operator()(int i, int j) const {
    if (swnorm(i, j) > real(1.E-3)) {  // daytime
      const real swsave = swdown(i, j);
      swdown(i, j) = swnorm(i, j);  // SWDOWN contains the unaffected SWDOWN in the output
      swnorm(i, j) = swsave;        // SWNORM contains the slope-affected SWDOWN in the output
      gsw(i, j) = gswsave(i, j);
    }
  }
};

}  // namespace cppwrf::phys::driver::topo
