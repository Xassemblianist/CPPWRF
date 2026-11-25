// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The lake model of WRF v4.8.0, sf_lake_physics = 1 (phys/module_sf_lake.F, the CLM 4.5 shallow lake model of
// Subin et al. as WRF compiles it: EM_CORE = 1; PERGRO, LAKEDEBUG, SHLAKETEST, ETALAKE, OFFLINE, EXTRALAKELAYERS, DGVM,
// CN, DUST undefined). One lake column per call: SUBROUTINE Lake's point body (lake_point) with LakeMain,
// ShalLakeFluxes, ShalLakeTemperature, SoilThermProp_Lake, PhaseChange_Lake, ShalLakeHydrology, SnowWater,
// SnowCompaction, CombineSnowLayers, DivideSnowLayers, Combo, QSat, Tridiagonal, FrictionVelocity, StabilityFunc1/2
// and MoninObukIni, and lakeini's point body (lakeini_point). Statement by statement in WRF's order and
// parenthesisation; comments give the lines of module_sf_lake.F.
//
// Precision: the column works in REAL(r8) (double) in every build, as WRF's real*4 build does; the grid arrays and
// Lake's forcing locals are REAL (the build's `real`). A Fortran literal without the _r8 kind is a default REAL
// (rl(): rounded to float in FP32 builds, as gfortran's REAL literal of WRF real*4). WRF's module variables (dtime,
// watsat, csol, tkmg, tkdry, tksatu, zsoi, ...) are per-column values here (LakeSoil, dtime arguments): no state is
// shared between columns.
//
// Math: gfortran vectorises (2-lane libmvec, every iteration in a lane: 10 trips) the two density loops of
// ShalLakeTemperature (rhow, |t - 277|**1.68: vm::pow) and lakeini's zsoi (exp) and soil-property (tkmg, tksatu: pow)
// loops; every other exp / log / pow / sin / atan is the scalar libm (module_sf_lake.o's call sites). Integer powers
// are products (x**2 = x*x, x**4 = (x*x)*(x*x)); the powers of constants (zetam**0.333, 17**1.5) are folded.
#include <algorithm>
#include <cmath>

#include "core/backend.hpp"
#include "core/precision.hpp"

namespace cppwrf::phys::lake {

using r8 = double;
// a default REAL literal of module_sf_lake.F (no _r8 kind) in a REAL(r8) expression
CPPWRF_HD constexpr r8 rl(double x) { return r8(real(x)); }

inline constexpr int nlevsoil = 10, nlevlake = 10, nlevsnow = 5;
// module constants (module_sf_lake.F:58-86)
inline constexpr r8 vkc = 0.4, pie = 3.141592653589793, grav = 9.80616, sb = 5.67e-8, tfrz = 273.16,
                    denh2o = 1.000e3, denice = 0.917e3, cpice = 2.11727e3, cpliq = 4.188e3, hfus = 3.337e5,
                    hvap = 2.501e6, hsub = 2.501e6 + 3.337e5, rair = 287.0423, cpair = 1.00464e3;
inline constexpr r8 tcrit = rl(2.5), tkwat = rl(0.6), tkice = rl(2.290), tkairc = rl(0.023), bdsno = rl(250.),
                    spval = rl(1.e36), wimp = rl(0.05), ssi = rl(0.033), cnfac = rl(0.5);
inline constexpr real depth_c = real(50.);  // REAL parameter
// percent sand / clay of the 19 soil categories (module_sf_lake.F:104-108, REAL data into REAL(r8))
inline constexpr double kSand[19] = {92., 80., 66., 20., 5., 43., 60., 10., 32., 51., 6., 22., 39.7, 0., 100.,
                                     54., 17., 100., 92.};
inline constexpr double kClay[19] = {3., 5., 10., 15., 5., 18., 27., 33., 33., 41., 47., 58., 14.7, 0., 0., 8.5,
                                     54., 0., 3.};

// WRF's module arrays watsat, csol, tkmg, tkdry, tksatu: the column's (Lake:350-356), index 1..10
struct LakeSoil {
  r8 watsat[11], csol[11], tkmg[11], tkdry[11], tksatu[11];
};

// The column state of Lake (the 14 INOUT arrays and the static z_lake / dz_lake / lakedepth). Index ranges as
// WRF's: lake 1..10 (element 0 unused), snow + soil -4..10 (element j + 4), interfaces -5..10 (element j + 5).
struct LakeColumn {
  r8 savedtke1, snowdp, h2osno, t_grnd, lakedepth;
  int snl;
  r8 t_lake[11], lake_icefrac[11], z_lake[11], dz_lake[11];
  r8 t_soisno[15], h2osoi_ice[15], h2osoi_liq[15], h2osoi_vol[15], z[15], dz[15];
  r8 zi[16];
};

// the fatal conditions of the column (raise_fatal codes in core/backend.hpp)
//   snl out of -5..0 (ShalLakeFluxes:795-799), |t_soisno - 288| > 150 (ShalLakeTemperature:1622-1625), and a new
//   snow layer that SnowCompaction would compact by WRF's undefined imelt(0) (not a WRF stop: see snow_compaction)

// ---------------------------------------------------------------- QSat (3309-3421)
CPPWRF_HD inline void qsat(r8 T, r8 p, r8& es, r8& esdT, r8& qs, r8& qsdT) {
  const r8 a0 = rl(6.11213476), a1 = rl(0.444007856), a2 = rl(0.143064234e-01), a3 = rl(0.264461437e-03),
           a4 = rl(0.305903558e-05), a5 = rl(0.196237241e-07), a6 = rl(0.892344772e-10),
           a7 = -rl(0.373208410e-12), a8 = rl(0.209339997e-15);
  const r8 b0 = rl(0.444017302), b1 = rl(0.286064092e-01), b2 = rl(0.794683137e-03), b3 = rl(0.121211669e-04),
           b4 = rl(0.103354611e-06), b5 = rl(0.404125005e-09), b6 = -rl(0.788037859e-12),
           b7 = -rl(0.114596802e-13), b8 = rl(0.381294516e-16);
  const r8 c0 = rl(6.11123516), c1 = rl(0.503109514), c2 = rl(0.188369801e-01), c3 = rl(0.420547422e-03),
           c4 = rl(0.614396778e-05), c5 = rl(0.602780717e-07), c6 = rl(0.387940929e-09),
           c7 = rl(0.149436277e-11), c8 = rl(0.262655803e-14);
  const r8 d0 = rl(0.503277922), d1 = rl(0.377289173e-01), d2 = rl(0.126801703e-02), d3 = rl(0.249468427e-04),
           d4 = rl(0.313703411e-06), d5 = rl(0.257180651e-08), d6 = rl(0.133268878e-10),
           d7 = rl(0.394116744e-13), d8 = rl(0.498070196e-16);
  r8 T_limit = T - tfrz;
  if (T_limit > rl(100.0)) T_limit = rl(100.0);
  if (T_limit < -rl(75.0)) T_limit = -rl(75.0);
  const r8 td = T_limit;
  if (td >= rl(0.0)) {
    es = a0 + td * (a1 + td * (a2 + td * (a3 + td * (a4 + td * (a5 + td * (a6 + td * (a7 + td * a8)))))));
    esdT = b0 + td * (b1 + td * (b2 + td * (b3 + td * (b4 + td * (b5 + td * (b6 + td * (b7 + td * b8)))))));
  } else {
    es = c0 + td * (c1 + td * (c2 + td * (c3 + td * (c4 + td * (c5 + td * (c6 + td * (c7 + td * c8)))))));
    esdT = d0 + td * (d1 + td * (d2 + td * (d3 + td * (d4 + td * (d5 + td * (d6 + td * (d7 + td * d8)))))));
  }
  es = es * rl(100.);
  esdT = esdT * rl(100.);
  const r8 vp = rl(1.0) / (p - rl(0.378) * es);
  const r8 vp1 = rl(0.622) * vp;
  const r8 vp2 = vp1 * vp;
  qs = es * vp1;
  qsdT = esdT * vp2 * p;
}

// ---------------------------------------------------------------- stability functions (4733-4802)
CPPWRF_HD inline r8 stability_func1(r8 zeta) {
  const r8 chik2 = std::sqrt(1. - 16. * zeta);
  const r8 chik = std::sqrt(chik2);
  return 2. * libm::log((1. + chik) * 0.5) + libm::log((1. + chik2) * 0.5) - 2. * libm::atan(chik) + pie * 0.5;
}
CPPWRF_HD inline r8 stability_func2(r8 zeta) {
  const r8 chik2 = std::sqrt(1. - 16. * zeta);
  return 2. * libm::log((1. + chik2) * 0.5);
}

// MoninObukIni (4810-4875)
CPPWRF_HD inline void monin_obuk_ini(r8 ur, r8 thv, r8 dthv, r8 zldis, r8 z0m, r8& um, r8& obu) {
  const r8 wc = 0.5;
  if (dthv >= 0.) {
    um = c_max(ur, 0.1);
  } else {
    um = std::sqrt(ur * ur + wc * wc);
  }
  const r8 rib = grav * zldis * dthv / (thv * um * um);
  r8 zeta;
  if (rib >= 0.) {
    zeta = rib * libm::log(zldis / z0m) / (1. - 5. * c_min(rib, 0.19));
    zeta = c_min(2., c_max(zeta, 0.01));
  } else {
    zeta = rib * libm::log(zldis / z0m);
    zeta = c_max(-100., c_min(zeta, -0.01));
  }
  obu = zldis / zeta;
}

// FrictionVelocity's temperature / humidity profile (4492-4506; the same form for temp2, temp12m, temp22m)
CPPWRF_HD inline r8 fv_profile(r8 zldis, r8 obu, r8 z0) {
  const r8 zetat = 0.465;
  const r8 zeta = zldis / obu;
  if (zeta < -zetat) {
    return vkc / (libm::log(-zetat * obu / z0) - stability_func2(-zetat) + stability_func2(z0 / obu) +
                  0.8 * (libm::pow(zetat, -0.333) - libm::pow(-zeta, -0.333)));
  } else if (zeta < 0.) {
    return vkc / (libm::log(zldis / z0) - stability_func2(zeta) + stability_func2(z0 / obu));
  } else if (zeta <= 1.) {
    return vkc / (libm::log(zldis / z0) + 5. * zeta - 5. * z0 / obu);
  } else {
    return vkc / (libm::log(obu / z0) + 5. - 5. * z0 / obu + (5. * libm::log(zeta) + zeta - 1.));
  }
}

// FrictionVelocity (4376-4728) for the lake's one pft: forc_hgt_u = forc_hgt_t = forc_hgt_q = hgt, displa = 0
CPPWRF_HD inline void friction_velocity(r8 hgt_u, r8 hgt_t, r8 hgt_q, r8 z0m, r8 z0h, r8 z0q, r8 obu, r8 um,
                                        r8& ustar, r8& temp1, r8& temp2, r8& temp12m, r8& temp22m) {
  const r8 zetam = 1.574;
  const r8 displa = 0.;
  r8 zldis = hgt_u - displa;
  const r8 zeta = zldis / obu;
  if (zeta < -zetam) {
    ustar = vkc * um / (libm::log(-zetam * obu / z0m) - stability_func1(-zetam) + stability_func1(z0m / obu) +
                        1.14 * (libm::pow(-zeta, 0.333) - libm::pow(zetam, 0.333)));
  } else if (zeta < 0.) {
    ustar = vkc * um / (libm::log(zldis / z0m) - stability_func1(zeta) + stability_func1(z0m / obu));
  } else if (zeta <= 1.) {
    ustar = vkc * um / (libm::log(zldis / z0m) + 5. * zeta - 5. * z0m / obu);
  } else {
    ustar = vkc * um / (libm::log(obu / z0m) + 5. - 5. * z0m / obu + (5. * libm::log(zeta) + zeta - 1.));
  }
  temp1 = fv_profile(hgt_t - displa, obu, z0h);
  if (hgt_q == hgt_t && z0q == z0h) {
    temp2 = temp1;
  } else {
    temp2 = fv_profile(hgt_q - displa, obu, z0q);
  }
  temp12m = fv_profile(2.0 + z0h, obu, z0h);
  if (z0q == z0h) {
    temp22m = temp12m;
  } else {
    temp22m = fv_profile(2.0 + z0q, obu, z0q);
  }
}

// ---------------------------------------------------------------- Tridiagonal (3424-3506), levels -4..20
CPPWRF_HD inline void tridiagonal(int jtop, const r8* a, const r8* b, const r8* c, const r8* r, r8* u) {
  // a, b, c, r, u: element j + 4 for j = -4..20
  r8 gam[25];
  r8 bet = b[jtop + 4];
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    if (j >= jtop) {
      if (j == jtop) {
        u[j + 4] = r[j + 4] / bet;
      } else {
        gam[j + 4] = c[j - 1 + 4] / bet;
        bet = b[j + 4] - a[j + 4] * gam[j + 4];
        u[j + 4] = (r[j + 4] - a[j + 4] * u[j - 1 + 4]) / bet;
      }
    }
  }
  for (int j = nlevlake + nlevsoil - 1; j >= -nlevsnow + 1; --j)
    if (j >= jtop) u[j + 4] = u[j + 4] - gam[j + 1 + 4] * u[j + 1 + 4];
}

// ---------------------------------------------------------------- SoilThermProp_Lake (2126-2314)
// tk, cv: element j + 4 (j = -4..10); returns tktopsoillay
CPPWRF_HD inline r8 soil_therm_prop(const LakeColumn& col, const LakeSoil& so, r8* tk, r8* cv) {
  const int snl = col.snl;
  const r8* dz = col.dz + 4;
  const r8* zi = col.zi + 5;
  const r8* z = col.z + 4;
  const r8* t_soisno = col.t_soisno + 4;
  const r8* liq = col.h2osoi_liq + 4;
  const r8* ice = col.h2osoi_ice + 4;
  r8 thk[15];
  r8* th = thk + 4;
  r8 tktopsoillay = 0.;
  for (int j = -nlevsnow + 1; j <= nlevsoil; ++j) {
    if (j >= 1) {
      const r8 satw = 1.;
      const r8 fl = liq[j] / (ice[j] + liq[j]);
      r8 dke, dksat;
      if (t_soisno[j] >= tfrz) {
        dke = c_max(0., 0. + 1.0);  // log10(satw) + 1.0, log10(1) = 0 (2231)
        dksat = so.tksatu[j];
      } else {
        dke = satw;
        dksat = so.tkmg[j] * libm::pow(0.249, fl * so.watsat[j]) * libm::pow(2.29, so.watsat[j]);
      }
      th[j] = dke * dksat + (1. - dke) * so.tkdry[j];
    }
    if (snl + 1 < 1 && (j >= snl + 1) && (j <= 0)) {
      const r8 bw = (ice[j] + liq[j]) / dz[j];
      th[j] = tkairc + (7.75e-5 * bw + 1.105e-6 * bw * bw) * (tkice - tkairc);
    }
  }
  for (int j = -nlevsnow + 1; j <= nlevsoil; ++j) {
    if (j >= snl + 1 && j <= nlevsoil - 1 && j != 0) {
      tk[j + 4] = th[j] * th[j + 1] * (z[j + 1] - z[j]) / (th[j] * (z[j + 1] - zi[j]) + th[j + 1] * (zi[j] - z[j]));
    } else if (j == 0) {
      tk[j + 4] = th[j];
    } else if (j == nlevsoil) {
      tk[j + 4] = 0.;
    }
    if (j == 1) tktopsoillay = th[j];
  }
  for (int j = 1; j <= nlevsoil; ++j)
    cv[j + 4] = so.csol[j] * (1 - so.watsat[j]) * dz[j] + (ice[j] * cpice + liq[j] * cpliq);
  for (int j = -nlevsnow + 1; j <= 0; ++j)
    if (snl + 1 < 1 && j >= snl + 1) cv[j + 4] = cpliq * liq[j] + cpice * ice[j];
  return tktopsoillay;
}

// ---------------------------------------------------------------- PhaseChange_Lake (2323-2541)
CPPWRF_HD inline void phase_change(LakeColumn& col, r8* cv, r8* cv_lake, r8& qflx_snomelt, int* imelt) {
  // cv: element j + 4; cv_lake: 1..10; imelt: element j + 4
  const r8 smallnumber = 1.e-7;
  const int snl = col.snl;
  r8* t_soisno = col.t_soisno + 4;
  r8* liq = col.h2osoi_liq + 4;
  r8* ice = col.h2osoi_ice + 4;
  r8* t_lake = col.t_lake;
  r8* icefrac = col.lake_icefrac;
  qflx_snomelt = 0.;
  r8 lhabs = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j)
    if (j >= snl + 1) imelt[j + 4] = 0;
  if (snl == 0 && col.h2osno > 0. && t_lake[1] > tfrz) {
    const r8 heatavail = (t_lake[1] - tfrz) * cv_lake[1];
    const r8 melt = c_min(col.h2osno, heatavail / hfus);
    const r8 heatrem = c_max(heatavail - melt * hfus, 0.);
    t_lake[1] = tfrz + heatrem / (cv_lake[1]);
    col.snowdp = col.snowdp * (1. - melt / col.h2osno);
    col.h2osno = col.h2osno - melt;
    lhabs = lhabs + melt * hfus;
    qflx_snomelt = qflx_snomelt + melt;
    if (col.h2osno < smallnumber) col.h2osno = 0.;
    if (col.snowdp < smallnumber) col.snowdp = 0.;
  }
  for (int j = 1; j <= nlevlake; ++j) {
    bool dophasechangeflag = false;
    r8 heatavail = 0., melt = 0., heatrem = 0.;
    if (t_lake[j] > tfrz && icefrac[j] > 0.) {
      dophasechangeflag = true;
      heatavail = (t_lake[j] - tfrz) * cv_lake[j];
      melt = c_min(icefrac[j] * denh2o * col.dz_lake[j], heatavail / hfus);
      heatrem = c_max(heatavail - melt * hfus, 0.);
    } else if (t_lake[j] < tfrz && icefrac[j] < 1.) {
      dophasechangeflag = true;
      heatavail = (t_lake[j] - tfrz) * cv_lake[j];
      melt = c_max(-(1. - icefrac[j]) * denh2o * col.dz_lake[j], heatavail / hfus);
      heatrem = c_min(heatavail - melt * hfus, 0.);
    }
    if (dophasechangeflag) {
      icefrac[j] = icefrac[j] - melt / (denh2o * col.dz_lake[j]);
      lhabs = lhabs + melt * hfus;
      cv_lake[j] = cv_lake[j] + melt * (cpliq - cpice);
      t_lake[j] = tfrz + heatrem / cv_lake[j];
      if (icefrac[j] > 1. - smallnumber) icefrac[j] = 1.;
      if (icefrac[j] < smallnumber) icefrac[j] = 0.;
    }
  }
  for (int j = -nlevsnow + 1; j <= nlevsoil; ++j) {
    bool dophasechangeflag = false;
    if (j >= snl + 1) {
      r8 heatavail = 0., melt = 0., heatrem = 0.;
      if (t_soisno[j] > tfrz && ice[j] > 0.) {
        dophasechangeflag = true;
        heatavail = (t_soisno[j] - tfrz) * cv[j + 4];
        melt = c_min(ice[j], heatavail / hfus);
        heatrem = c_max(heatavail - melt * hfus, 0.);
        if (j <= 0) {
          imelt[j + 4] = 1;
          qflx_snomelt = qflx_snomelt + melt;
        }
      } else if (t_soisno[j] < tfrz && liq[j] > 0.) {
        dophasechangeflag = true;
        heatavail = (t_soisno[j] - tfrz) * cv[j + 4];
        melt = c_max(-liq[j], heatavail / hfus);
        heatrem = c_min(heatavail - melt * hfus, 0.);
        if (j <= 0) {
          imelt[j + 4] = 2;
          qflx_snomelt = qflx_snomelt + melt;
        }
      }
      if (dophasechangeflag) {
        ice[j] = ice[j] - melt;
        liq[j] = liq[j] + melt;
        lhabs = lhabs + melt * hfus;
        cv[j + 4] = cv[j + 4] + melt * (cpliq - cpice);
        t_soisno[j] = tfrz + heatrem / cv[j + 4];
        if (ice[j] < smallnumber) ice[j] = 0.;
        if (liq[j] < smallnumber) liq[j] = 0.;
      }
    }
  }
  (void)lhabs;  // eflx_snomelt, lhabs: not used further (dead in WRF's LakeMain)
}

// density of the lake layers (1445-1458, 1924-1933): the vectorised loop, libmvec pow for all 10 iterations
CPPWRF_HD inline void lake_density(const LakeColumn& col, r8* rhow) {
  for (int j = 1; j <= nlevlake; ++j)
    rhow[j] = (1. - col.lake_icefrac[j]) * 1000. *
                  (1.0 - 1.9549e-05 * vm::pow(std::fabs(col.t_lake[j] - 277.), 1.68, j - 1, nlevlake)) +
              col.lake_icefrac[j] * denice;
}

// ---------------------------------------------------------------- ShalLakeFluxes (612-1151)
struct FluxOut {
  r8 qflx_evap_soi, qflx_evap_tot, eflx_sh_grnd, eflx_soil_grnd, eflx_sh_tot, eflx_lh_tot, eflx_gnet, t_ref2m,
      q_ref2m, ws, ks, z0mg;
};

CPPWRF_HD inline void shal_lake_fluxes(r8 forc_t, r8 forc_pbot, r8 forc_psrf, r8 forc_hgt, r8 forc_q, r8 forc_u,
                                       r8 forc_v, r8 forc_lwrad, r8 sabg, r8 lat, LakeColumn& col, FluxOut& o) {
  const r8 beta1 = 1., emg = 0.97, zii = 1000., tdmax = 277.;
  const r8 beta_islak = 0.4;  // beta(islak = 2)
  const r8 forc_hgt_u = forc_hgt, forc_hgt_t = forc_hgt, forc_hgt_q = forc_hgt;
  const r8* t_lake = col.t_lake;
  const r8 forc_th = forc_t * libm::pow(forc_psrf / forc_pbot, rair / cpair);
  const r8 forc_vp = forc_q * forc_pbot / (rl(0.622) + rl(0.378) * forc_q);
  const r8 forc_rho = (forc_pbot - rl(0.378) * forc_vp) / (rair * forc_t);
  const int jtop = col.snl + 1;
  r8 betaprime, dzsur;
  if (col.snl < 0) {
    betaprime = 1.;
    dzsur = col.dz[jtop + 4] / 2.;
  } else {
    betaprime = beta_islak;
    dzsur = col.dz_lake[1] / 2.;
  }
  r8 eg, degdT, qsatg, qsatgdT;
  qsat(col.t_grnd, forc_pbot, eg, degdT, qsatg, qsatgdT);
  const r8 thm = forc_t + 0.0098 * forc_hgt_t;
  const r8 thv = forc_th * (1. + 0.61 * forc_q);
  int nmozsgn = 0;
  r8 obuold = 0.;
  r8 z0mg;
  if (col.t_grnd >= tfrz) {
    z0mg = 0.001;
  } else if (col.snl == 0) {
    z0mg = 0.005;
  } else {
    z0mg = 0.0024;
  }
  const r8 z0hg = z0mg, z0qg = z0mg;
  r8 htvp = col.t_grnd > tfrz ? hvap : hsub;
  const r8 ur = c_max(1.0, std::sqrt(forc_u * forc_u + forc_v * forc_v));
  r8 dth = thm - col.t_grnd;
  r8 dqh = forc_q - qsatg;
  const r8 dthv = dth * (1. + 0.61 * forc_q) + 0.61 * forc_th * dqh;
  const r8 zldis = forc_hgt_u - 0.;
  r8 um, obu;
  monin_obuk_ini(ur, thv, dthv, zldis, z0mg, um, obu);
  r8 ustar = 0, temp1 = 0, temp2 = 0, temp12m = 0, temp22m = 0, ram = 0, rah = 0, raw = 0;
  r8 eflx_sh_grnd = 0, qflx_evap_soi = 0;
  int iter = 1;
  bool active = true;  // fncopy > 0
  while (iter <= 3 && active) {  // ITERATION (901-1022), niters = 3
    friction_velocity(forc_hgt_u, forc_hgt_t, forc_hgt_q, z0mg, z0hg, z0qg, obu, um, ustar, temp1, temp2, temp12m,
                      temp22m);
    const r8 tgbef = col.t_grnd;
    r8 tksur, tsur;
    if (col.t_grnd > tfrz && t_lake[1] > tfrz && col.snl == 0) {
      tksur = col.savedtke1;
      tsur = t_lake[1];
    } else if (col.snl == 0) {
      tksur = tkice;
      tsur = t_lake[1];
    } else {
      const r8 bw = (col.h2osoi_ice[jtop + 4] + col.h2osoi_liq[jtop + 4]) / col.dz[jtop + 4];
      tksur = tkairc + (7.75e-5 * bw + 1.105e-6 * bw * bw) * (tkice - tkairc);
      tsur = col.t_soisno[jtop + 4];
    }
    ram = 1. / (ustar * ustar / um);
    rah = 1. / (temp1 * ustar);
    raw = 1. / (temp2 * ustar);
    const r8 stftg3 = emg * sb * tgbef * tgbef * tgbef;
    const r8 ax = betaprime * sabg + emg * forc_lwrad + 3. * stftg3 * tgbef + forc_rho * cpair / rah * thm -
                  htvp * forc_rho / raw * (qsatg - qsatgdT * tgbef - forc_q) + tksur * tsur / dzsur;
    const r8 bx = 4. * stftg3 + forc_rho * cpair / rah + htvp * forc_rho / raw * qsatgdT + tksur / dzsur;
    col.t_grnd = ax / bx;
    htvp = col.t_grnd > tfrz ? hvap : hsub;
    eflx_sh_grnd = forc_rho * cpair * (col.t_grnd - thm) / rah;
    qflx_evap_soi = forc_rho * (qsatg + qsatgdT * (col.t_grnd - tgbef) - forc_q) / raw;
    qsat(col.t_grnd, forc_pbot, eg, degdT, qsatg, qsatgdT);
    dth = thm - col.t_grnd;
    dqh = forc_q - qsatg;
    const r8 tstar = temp1 * dth;
    const r8 qstar = temp2 * dqh;
    const r8 thvstar = tstar * (1. + 0.61 * forc_q) + 0.61 * forc_th * qstar;
    r8 zeta = zldis * vkc * grav * thvstar / (ustar * ustar * thv);
    if (zeta >= 0.) {
      zeta = c_min(2., c_max(zeta, 0.01));
      um = c_max(ur, 0.1);
    } else {
      zeta = c_max(-100., c_min(zeta, -0.01));
      const r8 wc = beta1 * libm::pow(-grav * ustar * thvstar * zii / thv, 0.333);
      um = std::sqrt(ur * ur + wc * wc);
    }
    obu = zldis / zeta;
    if (obuold * obu < 0.) nmozsgn = nmozsgn + 1;
    obuold = obu;
    iter = iter + 1;
    if (iter <= 3) active = nmozsgn < 3;
  }
  // 1026-1130
  if ((col.h2osno > 0.5 || t_lake[1] <= tfrz) && col.t_grnd > tfrz) {
    const r8 t_grnd_temp = col.t_grnd;
    col.t_grnd = tfrz;
    eflx_sh_grnd = forc_rho * cpair * (col.t_grnd - thm) / rah;
    qflx_evap_soi = forc_rho * (qsatg + qsatgdT * (col.t_grnd - t_grnd_temp) - forc_q) / raw;
  } else if ((t_lake[1] > col.t_grnd && col.t_grnd > tdmax) ||
             (t_lake[1] < col.t_grnd && t_lake[1] > tfrz && col.t_grnd < tdmax)) {
    const r8 t_grnd_temp = col.t_grnd;
    col.t_grnd = t_lake[1];
    eflx_sh_grnd = forc_rho * cpair * (col.t_grnd - thm) / rah;
    qflx_evap_soi = forc_rho * (qsatg + qsatgdT * (col.t_grnd - t_grnd_temp) - forc_q) / raw;
  }
  htvp = col.t_grnd > tfrz ? hvap : hsub;
  const r8 tg2 = col.t_grnd * col.t_grnd;
  const r8 eflx_lwrad_out = (1. - emg) * forc_lwrad + emg * sb * (tg2 * tg2);
  o.eflx_soil_grnd = sabg + forc_lwrad - eflx_lwrad_out - eflx_sh_grnd - htvp * qflx_evap_soi;
  o.eflx_sh_tot = eflx_sh_grnd;
  o.qflx_evap_tot = qflx_evap_soi;
  o.eflx_lh_tot = htvp * qflx_evap_soi;
  o.t_ref2m = thm + temp1 * dth * (1. / temp12m - 1. / temp1);
  o.q_ref2m = forc_q + temp2 * dqh * (1. / temp22m - 1. / temp2);
  o.eflx_gnet = betaprime * sabg + forc_lwrad - (eflx_lwrad_out + o.eflx_sh_tot + o.eflx_lh_tot);
  const r8 u2m = c_max(0.1, ustar / vkc * libm::log(2. / z0mg));
  o.ws = 1.2e-03 * u2m;
  o.ks = 6.6 * std::sqrt(std::fabs(libm::sin(lat))) * (libm::pow(u2m, -1.84));
  o.eflx_sh_grnd = eflx_sh_grnd;
  o.qflx_evap_soi = qflx_evap_soi;
  o.z0mg = z0mg;
  (void)ram;  // taux, tauy, ram1: not used by Lake
}

// ---------------------------------------------------------------- ShalLakeTemperature (1153-2117)
CPPWRF_HD inline void shal_lake_temperature(LakeColumn& col, const LakeSoil& so, r8 dtime, r8 sabg, r8 ws, r8 ks,
                                            FluxOut& o, r8* frac_iceold, r8& qflx_snomelt, int* imelt) {
  // frac_iceold, imelt: element j + 4
  const r8 p0 = 1.;
  const r8 beta_islak = 0.4, za_islak = 0.6;
  const int snl = col.snl;
  r8* t_lake = col.t_lake;
  r8* icefrac = col.lake_icefrac;
  const r8* z_lake = col.z_lake;
  const r8* dz_lake = col.dz_lake;
  r8* t_soisno = col.t_soisno + 4;
  const r8* liq = col.h2osoi_liq + 4;
  const r8* ice = col.h2osoi_ice + 4;
  const r8* z = col.z + 4;
  const r8 cwat = cpliq * denh2o;
  const r8 cice_eff = cpice * denh2o;
  const r8 cfus = hfus * denh2o;
  const r8 tkice_eff = tkice * denice / denh2o;
  const r8 km = tkwat / cwat;
  r8 ocvts = 0., ncvts = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j)
    if (j >= snl + 1) frac_iceold[j + 4] = ice[j] / (liq[j] + ice[j]);
  // wsum (1417-1425): not used further
  r8 fin = o.eflx_gnet;
  r8 rhow[11], phi[11], kme[11], tk_lake[11], cv_lake[11];
  lake_density(col, rhow);
  const bool unfrozen = col.t_grnd > tfrz && t_lake[1] > tfrz && snl == 0;
  for (int j = 1; j <= nlevlake - 1; ++j) {
    const r8 drhodz = (rhow[j + 1] - rhow[j]) / (z_lake[j + 1] - z_lake[j]);
    const r8 n2 = grav / rhow[j] * drhodz;
    const r8 vz = vkc * z_lake[j];
    const r8 num = 40. * n2 * (vz * vz);
    const r8 den = c_max((ws * ws) * libm::exp(-2. * ks * z_lake[j]), 1.e-10);
    const r8 ri = (-1. + std::sqrt(c_max(1. + num / den, 0.))) / 20.;
    if (unfrozen) {
      r8 ke;
      if (t_lake[1] > 277.15) {
        if (col.lakedepth > rl(15.0)) {
          ke = 1.e+2 * vkc * ws * z_lake[j] / p0 * libm::exp(-ks * z_lake[j]) / (1. + 37. * ri * ri);
        } else {
          ke = vkc * ws * z_lake[j] / p0 * libm::exp(-ks * z_lake[j]) / (1. + 37. * ri * ri);
        }
      } else {
        if (col.lakedepth > rl(15.0)) {
          if (col.lakedepth > rl(150.0)) {
            ke = 1.e+5 * vkc * ws * z_lake[j] / p0 * libm::exp(-ks * z_lake[j]) / (1. + 37. * ri * ri);
          } else {
            ke = 1.e+4 * vkc * ws * z_lake[j] / p0 * libm::exp(-ks * z_lake[j]) / (1. + 37. * ri * ri);
          }
        } else {
          ke = vkc * ws * z_lake[j] / p0 * libm::exp(-ks * z_lake[j]) / (1. + 37. * ri * ri);
        }
      }
      kme[j] = km + ke;
      tk_lake[j] = kme[j] * cwat;
    } else {
      kme[j] = km;
      tk_lake[j] = tkwat * tkice_eff / ((1. - icefrac[j]) * tkice_eff + tkwat * icefrac[j]);
    }
  }
  {
    const int j = nlevlake;
    kme[nlevlake] = kme[nlevlake - 1];
    if (unfrozen) {
      tk_lake[j] = tk_lake[j - 1];
    } else {
      tk_lake[j] = tkwat * tkice_eff / ((1. - icefrac[j]) * tkice_eff + tkwat * icefrac[j]);
    }
  }
  col.savedtke1 = kme[1] * cwat;
  const int jtop = snl + 1;
  // solar absorption (1531-1569); eta = 1.1925*lakedepth**(-0.424) (REAL exponent), the same in every iteration
  const r8 eta = 1.1925 * libm::pow(col.lakedepth, -rl(0.424));
  r8 phi_soil = 0.;
  for (int j = 1; j <= nlevlake; ++j) {
    const r8 zin = z_lake[j] - 0.5 * dz_lake[j];
    const r8 zout = z_lake[j] + 0.5 * dz_lake[j];
    const r8 rsfin = libm::exp(-eta * c_max(zin - za_islak, 0.));
    const r8 rsfout = libm::exp(-eta * c_max(zout - za_islak, 0.));
    r8 phidum;
    if (unfrozen) {
      phidum = (rsfin - rsfout) * sabg * (1. - beta_islak);
      if (j == nlevlake) phi_soil = rsfout * sabg * (1. - beta_islak);
    } else if (j == 1 && snl == 0) {
      phidum = sabg * (1. - beta_islak);
    } else {
      phidum = 0.;
      if (j == nlevlake) phi_soil = 0.;
    }
    phi[j] = phidum;
  }
  for (int j = 1; j <= nlevlake; ++j)
    cv_lake[j] = dz_lake[j] * (cwat * (1. - icefrac[j]) + cice_eff * icefrac[j]);
  r8 tk[15], cv[15];
  r8 tktopsoillay = soil_therm_prop(col, so, tk, cv);
  for (int j = 1; j <= nlevlake; ++j)
    ocvts = ocvts + cv_lake[j] * (t_lake[j] - tfrz) + cfus * dz_lake[j] * (1. - icefrac[j]);
  for (int j = -nlevsnow + 1; j <= nlevsoil; ++j) {
    if (j >= jtop) {
      ocvts = ocvts + cv[j + 4] * (t_soisno[j] - tfrz) + hfus * liq[j];
      if (j == 1 && col.h2osno > 0. && j == jtop) ocvts = ocvts - col.h2osno * hfus;
      if (std::fabs(t_soisno[j] - 288) > 150) raise_fatal(kFatalLakeTsoisno);
    }
  }
  // the whole column -4..20 (element j + 4)
  r8 zx[25], cvx[25], phix[25], tx[25], tkix[25], factx[25], fnx[25], a[25], b[25], c1[25], r[25];
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    const int jp = j - nlevlake;
    const int x = j + 4;
    if (j >= jtop) {
      if (j < 1) {
        zx[x] = z[j]; cvx[x] = cv[j + 4]; phix[x] = 0.; tx[x] = t_soisno[j];
      } else if (j <= nlevlake) {
        zx[x] = z_lake[j]; cvx[x] = cv_lake[j]; phix[x] = phi[j]; tx[x] = t_lake[j];
      } else {
        zx[x] = zx[nlevlake + 4] + dz_lake[nlevlake] / 2. + z[jp];
        cvx[x] = cv[jp + 4];
        phix[x] = j == nlevlake + 1 ? phi_soil : 0.;
        tx[x] = t_soisno[jp];
      }
    }
  }
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    const int jp = j - nlevlake;
    const int x = j + 4;
    if (j >= jtop) {
      if (j < 0) {
        tkix[x] = tk[j + 4];
      } else if (j == 0) {
        const r8 dzp = zx[x + 1] - zx[x];
        tkix[x] = tk_lake[1] * tk[j + 4] * dzp / (tk[j + 4] * z_lake[1] + tk_lake[1] * (-z[j]));
      } else if (j < nlevlake) {
        tkix[x] = (tk_lake[j] * tk_lake[j + 1] * (dz_lake[j + 1] + dz_lake[j])) /
                  (tk_lake[j] * dz_lake[j + 1] + tk_lake[j + 1] * dz_lake[j]);
      } else if (j == nlevlake) {
        const r8 dzp = zx[x + 1] - zx[x];
        tkix[x] = (tktopsoillay * tk_lake[j] * dzp / (tktopsoillay * dz_lake[j] / 2. + tk_lake[j] * z[1]));
      } else {
        tkix[x] = tk[jp + 4];
      }
    }
  }
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    const int x = j + 4;
    if (j >= jtop) {
      if (j < nlevlake + nlevsoil) {
        factx[x] = dtime / cvx[x];
        fnx[x] = tkix[x] * (tx[x + 1] - tx[x]) / (zx[x + 1] - zx[x]);
      } else {
        factx[x] = dtime / cvx[x];
        fnx[x] = 0.;
      }
    }
  }
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    const int x = j + 4;
    if (j >= jtop) {
      if (j == jtop) {
        const r8 dzp = zx[x + 1] - zx[x];
        a[x] = 0.;
        b[x] = 1 + (1. - cnfac) * factx[x] * tkix[x] / dzp;
        c1[x] = -(1. - cnfac) * factx[x] * tkix[x] / dzp;
        r[x] = tx[x] + factx[x] * (fin + phix[x] + cnfac * fnx[x]);
      } else if (j < nlevlake + nlevsoil) {
        const r8 dzm = (zx[x] - zx[x - 1]);
        const r8 dzp = (zx[x + 1] - zx[x]);
        a[x] = -(1. - cnfac) * factx[x] * tkix[x - 1] / dzm;
        b[x] = 1. + (1. - cnfac) * factx[x] * (tkix[x] / dzp + tkix[x - 1] / dzm);
        c1[x] = -(1. - cnfac) * factx[x] * tkix[x] / dzp;
        r[x] = tx[x] + cnfac * factx[x] * (fnx[x] - fnx[x - 1]) + factx[x] * phix[x];
      } else {
        const r8 dzm = (zx[x] - zx[x - 1]);
        a[x] = -(1. - cnfac) * factx[x] * tkix[x - 1] / dzm;
        b[x] = 1. + (1. - cnfac) * factx[x] * tkix[x - 1] / dzm;
        c1[x] = 0.;
        r[x] = tx[x] - cnfac * factx[x] * fnx[x - 1];
      }
    }
  }
  tridiagonal(jtop, a, b, c1, r, tx);
  for (int j = -nlevsnow + 1; j <= nlevlake + nlevsoil; ++j) {
    const int jp = j - nlevlake;
    if (j >= jtop) {
      if (j < 1) {
        t_soisno[j] = tx[j + 4];
      } else if (j <= nlevlake) {
        t_lake[j] = tx[j + 4];
      } else {
        t_soisno[jp] = tx[j + 4];
      }
    }
  }
  phase_change(col, cv, cv_lake, qflx_snomelt, imelt);
  // convective mixing (1924-2026)
  lake_density(col, rhow);
  for (int j = 1; j <= nlevlake - 1; ++j) {
    r8 qav = 0., nav = 0., iceav = 0.;
    const bool mix = rhow[j] > rhow[j + 1] || (icefrac[j] < 1. && icefrac[j + 1] > 0.);
    for (int i = 1; i <= j + 1; ++i) {
      if (mix) {
        qav = qav + dz_lake[i] * (t_lake[i] - tfrz) * ((1. - icefrac[i]) * cwat + icefrac[i] * cice_eff);
        iceav = iceav + icefrac[i] * dz_lake[i];
        nav = nav + dz_lake[i];
      }
    }
    r8 tav_froz = 0., tav_unfr = 0.;
    if (mix) {
      qav = qav / nav;
      iceav = iceav / nav;
      if (qav > 0.) {
        tav_froz = 0.;
        tav_unfr = qav / ((1. - iceav) * cwat);
      } else if (qav < 0.) {
        tav_froz = qav / (iceav * cice_eff);
        tav_unfr = 0.;
      } else {
        tav_froz = 0.;
        tav_unfr = 0.;
      }
    }
    r8 zsum = 0.;
    for (int i = 1; i <= j + 1; ++i) {
      if (nav > 0.) {
        if (i == 1) zsum = 0.;
        if ((zsum + dz_lake[i]) / nav <= iceav) {
          icefrac[i] = 1.;
          t_lake[i] = tav_froz + tfrz;
        } else if (zsum / nav < iceav) {
          icefrac[i] = (iceav * nav - zsum) / dz_lake[i];
          t_lake[i] = (icefrac[i] * tav_froz * cice_eff + (1. - icefrac[i]) * tav_unfr * cwat) /
                          (icefrac[i] * cice_eff + (1 - icefrac[i]) * cwat) +
                      tfrz;
        } else {
          icefrac[i] = 0.;
          t_lake[i] = tav_unfr + tfrz;
        }
        zsum = zsum + dz_lake[i];
        rhow[i] = (1. - icefrac[i]) * 1000. * (1.0 - 1.9549e-05 * libm::pow(std::fabs(t_lake[i] - 277.), 1.68)) +
                  icefrac[i] * denice;
      }
    }
  }
  for (int j = 1; j <= nlevlake; ++j)
    cv_lake[j] = dz_lake[j] * (cwat * (1. - icefrac[j]) + cice_eff * icefrac[j]);
  tktopsoillay = soil_therm_prop(col, so, tk, cv);
  (void)tktopsoillay;
  for (int j = 1; j <= nlevlake; ++j) {
    ncvts = ncvts + cv_lake[j] * (t_lake[j] - tfrz) + cfus * dz_lake[j] * (1. - icefrac[j]);
    fin = fin + phi[j];
  }
  for (int j = -nlevsnow + 1; j <= nlevsoil; ++j) {
    if (j >= jtop) {
      ncvts = ncvts + cv[j + 4] * (t_soisno[j] - tfrz) + hfus * liq[j];
      if (j == 1 && col.h2osno > 0. && j == jtop) ncvts = ncvts - col.h2osno * hfus;
    }
    if (j == 1) fin = fin + phi_soil;
  }
  const r8 errsoi = (ncvts - ocvts) / dtime - fin;
  if (std::fabs(errsoi) < 10.) {
    o.eflx_sh_tot = o.eflx_sh_tot - errsoi;
    o.eflx_sh_grnd = o.eflx_sh_grnd - errsoi;
    o.eflx_soil_grnd = o.eflx_soil_grnd + errsoi;
    o.eflx_gnet = o.eflx_gnet + errsoi;
  }
}

// ---------------------------------------------------------------- Combo (4254-4317)
CPPWRF_HD inline void combo(r8& dz, r8& wliq, r8& wice, r8& t, r8 dz2, r8 wliq2, r8 wice2, r8 t2) {
  const r8 dzc = dz + dz2;
  const r8 wicec = (wice + wice2);
  const r8 wliqc = (wliq + wliq2);
  const r8 h = (cpice * wice + cpliq * wliq) * (t - tfrz) + hfus * wliq;
  const r8 h2 = (cpice * wice2 + cpliq * wliq2) * (t2 - tfrz) + hfus * wliq2;
  const r8 hc = h + h2;
  r8 tc;
  if (hc < rl(0.)) {
    tc = tfrz + hc / (cpice * wicec + cpliq * wliqc);
  } else if (hc <= hfus * wliqc) {
    tc = tfrz;
  } else {
    tc = tfrz + (hc - hfus * wliqc) / (cpice * wicec + cpliq * wliqc);
  }
  dz = dzc;
  wice = wicec;
  wliq = wliqc;
  t = tc;
}

// ---------------------------------------------------------------- SnowWater (3509-3671), snow columns only
CPPWRF_HD inline void snow_water(LakeColumn& col, r8 dtime, r8 qflx_rain_grnd, r8 qflx_sub_snow, r8 qflx_evap_grnd,
                                 r8 qflx_dew_snow, r8 qflx_dew_grnd) {
  const int snl = col.snl;
  r8* liq = col.h2osoi_liq + 4;
  r8* ice = col.h2osoi_ice + 4;
  const r8* dz = col.dz + 4;
  // do_capsnow = .false.
  const r8 wgdif = ice[snl + 1] + (qflx_dew_snow - qflx_sub_snow) * dtime;
  ice[snl + 1] = wgdif;
  if (wgdif < rl(0.)) {
    ice[snl + 1] = rl(0.);
    liq[snl + 1] = liq[snl + 1] + wgdif;
  }
  liq[snl + 1] = liq[snl + 1] + (qflx_rain_grnd + qflx_dew_grnd - qflx_evap_grnd) * dtime;
  liq[snl + 1] = c_max(0., liq[snl + 1]);
  r8 vol_liq[6], vol_ice[6], eff_porosity[6];  // element j + 5, j = -4..0
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (j >= snl + 1) {
      vol_ice[j + 5] = c_min(1., ice[j] / (dz[j] * denice));
      eff_porosity[j + 5] = rl(1.) - vol_ice[j + 5];
      vol_liq[j + 5] = c_min(eff_porosity[j + 5], liq[j] / (dz[j] * denh2o));
    }
  }
  r8 qin = 0., qout = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (j >= snl + 1) {
      liq[j] = liq[j] + qin;
      if (j <= -1) {
        if (eff_porosity[j + 5] < wimp || eff_porosity[j + 6] < wimp) {
          qout = 0.;
        } else {
          qout = c_max(0., (vol_liq[j + 5] - ssi * eff_porosity[j + 5]) * dz[j]);
          qout = c_min(qout, (rl(1.) - vol_ice[j + 6] - vol_liq[j + 6]) * dz[j + 1]);
        }
      } else {
        qout = c_max(0., (vol_liq[j + 5] - ssi * eff_porosity[j + 5]) * dz[j]);
      }
      qout = qout * rl(1000.);
      liq[j] = liq[j] - qout;
      qin = qout;
    }
  }
  // qflx_top_soil: not used further
}

// ---------------------------------------------------------------- SnowCompaction (3673-3801)
// new_layer_undef: the top layer was made this step from no snow (ShalLakeHydrology 2822-2833), where WRF reads its
// undefined imelt(0) (PhaseChange_Lake sets imelt only for the layers that existed)
CPPWRF_HD inline void snow_compaction(LakeColumn& col, r8 dtime, const int* imelt, const r8* frac_iceold,
                                      bool new_layer_undef) {
  const r8 c2 = rl(23.e-3), c3 = rl(2.777e-6), c4 = rl(0.04), c5 = rl(2.0), dm = rl(100.0), eta0 = rl(9.e+5);
  const int snl = col.snl;
  const r8* liq = col.h2osoi_liq + 4;
  const r8* ice = col.h2osoi_ice + 4;
  const r8* t_soisno = col.t_soisno + 4;
  r8* dz = col.dz + 4;
  r8 burden = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (j >= snl + 1) {
      const r8 wx = ice[j] + liq[j];
      const r8 void_ = rl(1.) - (ice[j] / denice + liq[j] / denh2o) / dz[j];
      if (void_ > rl(0.001) && ice[j] > rl(.1)) {
        const r8 bi = ice[j] / dz[j];
        const r8 fi = ice[j] / wx;
        const r8 td = tfrz - t_soisno[j];
        const r8 dexpf = libm::exp(-c4 * td);
        r8 ddz1 = -c3 * dexpf;
        if (bi > dm) ddz1 = ddz1 * libm::exp(-rl(46.0e-3) * (bi - dm));
        if (liq[j] > rl(0.01) * dz[j]) ddz1 = ddz1 * c5;
        const r8 ddz2 = -burden * libm::exp(-rl(0.08) * td - c2 * bi) / eta0;
        r8 ddz3;
        if (new_layer_undef && j == 0 && (frac_iceold[j + 4] - fi) / frac_iceold[j + 4] > 0.)
          raise_fatal(kFatalLakeImelt);
        if (imelt[j + 4] == 1) {
          ddz3 = -rl(1.) / dtime * c_max(0., (frac_iceold[j + 4] - fi) / frac_iceold[j + 4]);
        } else {
          ddz3 = 0.;
        }
        const r8 pdzdtc = ddz1 + ddz2 + ddz3;
        dz[j] = dz[j] * (rl(1.) + pdzdtc * dtime);
      }
      burden = burden + wx;
    }
  }
}

// ---------------------------------------------------------------- CombineSnowLayers (3803-4024)
CPPWRF_HD inline void combine_snow_layers(LakeColumn& col) {
  const r8 dzmin[5] = {rl(0.010), rl(0.015), rl(0.025), rl(0.055), rl(0.115)};
  r8* liq = col.h2osoi_liq + 4;
  r8* ice = col.h2osoi_ice + 4;
  r8* t = col.t_soisno + 4;
  r8* dz = col.dz + 4;
  r8* z = col.z + 4;
  r8* zi = col.zi + 5;
  int& snl = col.snl;
  int msn_old = snl;
  for (int j = msn_old + 1; j <= 0; ++j) {
    if (ice[j] <= rl(.1)) {
      liq[j + 1] = liq[j + 1] + liq[j];
      ice[j + 1] = ice[j + 1] + ice[j];
      if (j > snl + 1 && snl < -1) {
        for (int i = j; i >= snl + 2; --i) {
          t[i] = t[i - 1];
          liq[i] = liq[i - 1];
          ice[i] = ice[i - 1];
          dz[i] = dz[i - 1];
        }
      }
      snl = snl + 1;
    }
  }
  col.h2osno = 0.;
  col.snowdp = 0.;
  r8 zwice = 0., zwliq = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (j >= snl + 1) {
      col.h2osno = col.h2osno + ice[j] + liq[j];
      col.snowdp = col.snowdp + dz[j];
      zwice = zwice + ice[j];
      zwliq = zwliq + liq[j];
    }
  }
  if (col.snowdp < rl(0.01) && col.snowdp > rl(0.)) {
    snl = 0;
    col.h2osno = zwice;
    if (col.h2osno <= rl(0.)) col.snowdp = 0.;
  }
  if (snl < -1) {
    msn_old = snl;
    int mssi = 1;
    for (int i = msn_old + 1; i <= 0; ++i) {
      if (dz[i] < dzmin[mssi - 1]) {
        int neibor;
        if (i == snl + 1) {
          neibor = i + 1;
        } else if (i == 0) {
          neibor = i - 1;
        } else {
          neibor = i + 1;
          if ((dz[i - 1] + dz[i]) < (dz[i + 1] + dz[i])) neibor = i - 1;
        }
        int j, l;
        if (neibor > i) {
          j = neibor;
          l = i;
        } else {
          j = i;
          l = neibor;
        }
        combo(dz[j], liq[j], ice[j], t[j], dz[l], liq[l], ice[l], t[l]);
        if (j - 1 > snl + 1) {
          for (int k = j - 1; k >= snl + 2; --k) {
            t[k] = t[k - 1];
            ice[k] = ice[k - 1];
            liq[k] = liq[k - 1];
            dz[k] = dz[k - 1];
          }
        }
        snl = snl + 1;
        if (snl >= -1) break;
      } else {
        mssi = mssi + 1;
      }
    }
  }
  for (int j = 0; j >= -nlevsnow + 1; --j) {
    if (j >= snl + 1) {
      z[j] = zi[j] - rl(0.5) * dz[j];
      zi[j - 1] = zi[j] - dz[j];
    }
  }
}

// ---------------------------------------------------------------- DivideSnowLayers (4026-4252)
CPPWRF_HD inline void divide_snow_layers(LakeColumn& col) {
  r8* liq = col.h2osoi_liq + 4;
  r8* ice = col.h2osoi_ice + 4;
  r8* t = col.t_soisno + 4;
  r8* dz = col.dz + 4;
  r8* z = col.z + 4;
  r8* zi = col.zi + 5;
  int& snl = col.snl;
  r8 dzsno[6], swice[6], swliq[6], tsno[6];  // 1..5
  for (int j = 1; j <= nlevsnow; ++j) {
    if (j <= (snl < 0 ? -snl : snl)) {
      dzsno[j] = dz[j + snl];
      swice[j] = ice[j + snl];
      swliq[j] = liq[j + snl];
      tsno[j] = t[j + snl];
    }
  }
  int msno = snl < 0 ? -snl : snl;
  // one subdivision step: layer n above its threshold keeps dzk, the rest goes to layer n + 1
  auto split = [&](int n, r8 dzk) {
    const r8 drr = dzsno[n] - dzk;
    r8 propor = drr / dzsno[n];
    const r8 zwice = propor * swice[n];
    const r8 zwliq = propor * swliq[n];
    propor = dzk / dzsno[n];
    swice[n] = propor * swice[n];
    swliq[n] = propor * swliq[n];
    dzsno[n] = dzk;
    combo(dzsno[n + 1], swliq[n + 1], swice[n + 1], tsno[n + 1], drr, zwliq, zwice, tsno[n]);
  };
  auto halve = [&](int n) {
    dzsno[n] = dzsno[n] / rl(2.);
    swice[n] = swice[n] / rl(2.);
    swliq[n] = swliq[n] / rl(2.);
    dzsno[n + 1] = dzsno[n];
    swice[n + 1] = swice[n];
    swliq[n + 1] = swliq[n];
    tsno[n + 1] = tsno[n];
  };
  if (msno == 1) {
    if (dzsno[1] > rl(0.03)) {
      msno = 2;
      halve(1);
    }
  }
  if (msno > 1) {
    if (dzsno[1] > rl(0.02)) {
      split(1, rl(0.02));
      if (msno <= 2 && dzsno[2] > rl(0.07)) {
        msno = 3;
        halve(2);
      }
    }
  }
  if (msno > 2) {
    if (dzsno[2] > rl(0.05)) {
      split(2, rl(0.05));
      if (msno <= 3 && dzsno[3] > rl(0.18)) {
        msno = 4;
        halve(3);
      }
    }
  }
  if (msno > 3) {
    if (dzsno[3] > rl(0.11)) {
      split(3, rl(0.11));
      if (msno <= 4 && dzsno[4] > rl(0.41)) {
        msno = 5;
        halve(4);
      }
    }
  }
  if (msno > 4) {
    if (dzsno[4] > rl(0.23)) split(4, rl(0.23));
  }
  snl = -msno;
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (j >= snl + 1) {
      dz[j] = dzsno[j - snl];
      ice[j] = swice[j - snl];
      liq[j] = swliq[j - snl];
      t[j] = tsno[j - snl];
    }
  }
  for (int j = 0; j >= -nlevsnow + 1; --j) {
    if (j >= snl + 1) {
      z[j] = zi[j] - rl(0.5) * dz[j];
      zi[j - 1] = zi[j] - dz[j];
    }
  }
}

// ---------------------------------------------------------------- ShalLakeHydrology (2544-3307)
CPPWRF_HD inline void shal_lake_hydrology(LakeColumn& col, const LakeSoil& so, r8 dtime, r8 forc_rain, r8 forc_snow,
                                          r8 forc_t, r8 qflx_evap_soi, r8* frac_iceold, const int* imelt) {
  const r8 snow_bd = 250.;
  r8* liq = col.h2osoi_liq + 4;
  r8* ice = col.h2osoi_ice + 4;
  r8* t_soisno = col.t_soisno + 4;
  r8* dz = col.dz + 4;
  r8* z = col.z + 4;
  r8* zi = col.zi + 5;
  r8* vol = col.h2osoi_vol + 4;
  // do_capsnow = .false.: qflx_snowcap = 0, the snow and rain reach the ground
  const r8 qflx_snow_grnd_col = forc_snow;
  const r8 qflx_rain_grnd = forc_rain;
  r8 dz_snowf;
  {
    r8 bifall;
    if (forc_t > tfrz + 2.) {
      bifall = 50. + 1.7 * libm::pow(17.0, 1.5);
    } else if (forc_t > tfrz - 15.) {
      bifall = 50. + 1.7 * libm::pow(forc_t - tfrz + 15., 1.5);
    } else {
      bifall = 50.;
    }
    dz_snowf = qflx_snow_grnd_col / bifall;
    col.snowdp = col.snowdp + dz_snowf * dtime;
    col.h2osno = col.h2osno + qflx_snow_grnd_col * dtime;
  }
  bool newnode = false;
  if (col.snl == 0 && qflx_snow_grnd_col > 0.0 && col.snowdp >= 0.01) {
    newnode = true;
    col.snl = -1;
    dz[0] = col.snowdp;
    z[0] = -0.5 * dz[0];
    zi[-1] = -dz[0];
    t_soisno[0] = c_min(tfrz, forc_t);
    ice[0] = col.h2osno;
    liq[0] = 0.;
    frac_iceold[0 + 4] = 1.;
  }
  if (col.snl < 0 && !newnode) {
    ice[col.snl + 1] = ice[col.snl + 1] + dtime * qflx_snow_grnd_col;
    dz[col.snl + 1] = dz[col.snl + 1] + dz_snowf * dtime;
  }
  const int jtop = col.snl + 1;
  r8 qflx_evap_grnd = 0., qflx_sub_snow = 0., qflx_dew_snow = 0., qflx_dew_grnd = 0.;
  if (jtop <= 0) {
    const int j = jtop;
    if (qflx_evap_soi >= 0.) {
      const r8 qflx_evap_soi_lim = c_min(qflx_evap_soi, (liq[j] + ice[j]) / dtime);
      if ((liq[j] + ice[j]) > 0.) {
        qflx_evap_grnd = c_max(qflx_evap_soi_lim * (liq[j] / (liq[j] + ice[j])), 0.);
      } else {
        qflx_evap_grnd = 0.;
      }
      qflx_sub_snow = qflx_evap_soi_lim - qflx_evap_grnd;
    } else {
      if (col.t_grnd < tfrz) {
        qflx_dew_snow = std::fabs(qflx_evap_soi);
      } else {
        qflx_dew_grnd = std::fabs(qflx_evap_soi);
      }
    }
  } else {
    if (qflx_evap_soi >= 0.) {
      qflx_sub_snow = c_min(qflx_evap_soi, col.h2osno / dtime);
      qflx_evap_grnd = qflx_evap_soi - qflx_sub_snow;
    } else {
      if (col.t_grnd < tfrz - 0.1) {
        qflx_dew_snow = std::fabs(qflx_evap_soi);
      } else {
        qflx_dew_grnd = std::fabs(qflx_evap_soi);
      }
    }
    const r8 h2osno_temp = col.h2osno;
    col.h2osno = col.h2osno + (-qflx_sub_snow + qflx_dew_snow) * dtime;
    if (h2osno_temp > 0.) {
      col.snowdp = col.snowdp * col.h2osno / h2osno_temp;
    } else {
      col.snowdp = col.h2osno / snow_bd;
    }
    col.h2osno = c_max(col.h2osno, 0.);
  }
  // BuildSnowFilter: snow column iff snl < 0
  const bool snowc = col.snl < 0;
  if (snowc) snow_water(col, dtime, qflx_rain_grnd, qflx_sub_snow, qflx_evap_grnd, qflx_dew_snow, qflx_dew_grnd);
  for (int j = 1; j <= nlevsoil; ++j) {
    if (vol[j] < so.watsat[j]) {
      liq[j] = (so.watsat[j] * dz[j] - ice[j] / denice) * denh2o;
    } else if (liq[j] > so.watsat[j] * denh2o * dz[j]) {
      liq[j] = so.watsat[j] * denh2o * dz[j];
    }
  }
  if (snowc) {
    snow_compaction(col, dtime, imelt, frac_iceold, newnode);
    combine_snow_layers(col);
    divide_snow_layers(col);
  }
  // snow on an unfrozen lake (3023-3080)
  const bool unfrozen = col.t_lake[1] > tfrz && col.lake_icefrac[1] == 0. && col.snl < 0;
  r8 sumsnowice = 0., heatsum = 0.;
  for (int j = -nlevsnow + 1; j <= 0; ++j) {
    if (unfrozen) {
      if (j == -nlevsnow + 1) {
        sumsnowice = 0.;
        heatsum = 0.;
      }
      if (j >= col.snl + 1) {
        sumsnowice = sumsnowice + ice[j];
        heatsum = heatsum + ice[j] * cpice * (tfrz - t_soisno[j]) + liq[j] * cpliq * (tfrz - t_soisno[j]);
      }
    }
  }
  if (unfrozen) {
    heatsum = heatsum + sumsnowice * hfus;
    const r8 heatrem = (col.t_lake[1] - tfrz) * cpliq * denh2o * col.dz_lake[1] - heatsum;
    if (heatrem + denh2o * col.dz_lake[1] * hfus > 0.) {
      col.h2osno = 0.;
      col.snl = 0;
      if (heatrem > 0.) {
        col.t_lake[1] = col.t_lake[1] - heatrem / (cpliq * denh2o * col.dz_lake[1]);
      } else {
        col.t_lake[1] = tfrz;
        col.lake_icefrac[1] = -heatrem / (denh2o * col.dz_lake[1] * hfus);
      }
    }
  }
  // zero the empty snow layers of the snow columns (the filter of 2935, before the layer changes)
  if (snowc) {
    for (int j = -nlevsnow + 1; j <= 0; ++j) {
      if (j <= col.snl && col.snl > -nlevsnow) {
        ice[j] = 0.;
        liq[j] = 0.;
        t_soisno[j] = 0.;
        dz[j] = 0.;
        z[j] = 0.;
        zi[j - 1] = 0.;
      }
    }
  }
  // t_snow, snowice, snowliq, endwb, qflx_qrgwl, ...: not used by Lake
  for (int j = 1; j <= nlevsoil; ++j) vol[j] = liq[j] / (dz[j] * denh2o) + ice[j] / (dz[j] * denice);
}

// ---------------------------------------------------------------- Lake point (120-421) with LakeMain (424-609)
struct LakeForcing {
  real t_phy, p8w1, p8w2, dz8w, qv, u_phy, v_phy, glw, emiss, rainbl, swdown, albedo, xlat;
};
struct LakeOut {
  real hfx, lh, grdflx, tsk, qfx, t2, th2, q2, albedo;
};

CPPWRF_HD inline void lake_point(const LakeForcing& f, real dtbl, real rcp, const LakeSoil& so, LakeColumn& col,
                                 LakeOut& out) {
  const r8 dtime = dtbl;  // 278
  // Lake's REAL forcing locals (283-292)
  const real SFCTMP = f.t_phy;
  const real PBOT = f.p8w2;
  const real PSFC = f.p8w1;
  const real ZLVL = real(0.5) * f.dz8w;
  const real Q2K = f.qv / (real(1.0) + f.qv);
  const real EMISSI = f.emiss;
  const real LWDN = f.glw * EMISSI;
  const real PRCP = f.rainbl / dtbl;
  const real SOLDN = f.swdown;
  const real SOLNET = SOLDN * (real(1.) - f.albedo);
  const r8 forc_t = SFCTMP, forc_pbot = PBOT, forc_psrf = PSFC, forc_hgt = ZLVL, forc_q = Q2K, forc_u = f.u_phy,
           forc_v = f.v_phy, forc_lwrad = LWDN, prec = PRCP, sabg = SOLNET;
  const r8 lat = r8(f.xlat) * pie / 180;
  if (col.snl > 0 || col.snl < -5) {  // ShalLakeFluxes 795-799
    raise_fatal(kFatalLakeSnl);
    return;
  }
  // LakeMain (544-564)
  r8 forc_rain, forc_snow;
  if (prec > rl(0.)) {
    if (forc_t > (tfrz + tcrit)) {
      forc_rain = prec;
      forc_snow = rl(0.);
    } else {
      forc_rain = rl(0.);
      forc_snow = prec;
    }
  } else {
    forc_rain = rl(0.);
    forc_snow = rl(0.);
  }
  FluxOut o{};
  shal_lake_fluxes(forc_t, forc_pbot, forc_psrf, forc_hgt, forc_q, forc_u, forc_v, forc_lwrad, sabg, lat, col, o);
  r8 frac_iceold[15] = {};
  int imelt[15] = {};
  r8 qflx_snomelt = 0.;
  shal_lake_temperature(col, so, dtime, sabg, o.ws, o.ks, o, frac_iceold, qflx_snomelt, imelt);
  shal_lake_hydrology(col, so, dtime, forc_rain, forc_snow, forc_t, o.qflx_evap_soi, frac_iceold, imelt);
  // outputs (374-388)
  out.hfx = real(o.eflx_sh_tot);
  out.lh = real(o.eflx_lh_tot);
  out.grdflx = real(o.eflx_gnet);
  out.tsk = real(col.t_grnd);
  out.t2 = real(o.t_ref2m);
  out.th2 = out.t2 * libm::pow(real(1.E5) / PSFC, rcp);
  out.q2 = real(o.q_ref2m);
  out.albedo = real((rl(0.6) * col.lake_icefrac[1]) + ((rl(1.0) - col.lake_icefrac[1]) * rl(0.08)));
  if (out.tsk >= tfrz) {
    out.qfx = real(o.eflx_lh_tot / hvap);
  } else {
    out.qfx = real(o.eflx_lh_tot / hsub);
  }
}

// ---------------------------------------------------------------- lakeini (4886-5397), one lake point
// The soil grid (5146-5216: zlak, dzlak, zsoi with the vectorised exp, dzsoi, zisoi), computed once.
struct LakeGrid {
  r8 zlak[11], dzlak[11], zsoi[11], dzsoi[11], zisoi[11];  // zisoi: 0..10
};
inline LakeGrid lake_grid() {
  LakeGrid g{};
  for (int j = 1; j <= nlevlake; ++j) g.dzlak[j] = 0.1;
  const r8 zl[11] = {0., 0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95};  // the literals of 5157-5166
  for (int j = 1; j <= nlevlake; ++j) g.zlak[j] = zl[j];
  const r8 scalez = 0.025;
  // gfortran folds the first and the last iteration (exp of a constant, correctly rounded) and runs j = 2..9 in
  // libmvec's 2-lane exp
  for (int j = 1; j <= nlevsoil; ++j) {
    const r8 e = (j == 1 || j == nlevsoil) ? libm::exp(0.5 * (j - 0.5)) : vm::exp(0.5 * (j - 0.5), j - 2, 8);
    g.zsoi[j] = scalez * (e - 1.);
  }
  g.dzsoi[1] = 0.5 * (g.zsoi[1] + g.zsoi[2]);
  for (int j = 2; j <= nlevsoil - 1; ++j) g.dzsoi[j] = 0.5 * (g.zsoi[j + 1] - g.zsoi[j - 1]);
  g.dzsoi[nlevsoil] = g.zsoi[nlevsoil] - g.zsoi[nlevsoil - 1];
  g.zisoi[0] = 0.;
  for (int j = 1; j <= nlevsoil - 1; ++j) g.zisoi[j] = 0.5 * (g.zsoi[j] + g.zsoi[j + 1]);
  g.zisoi[nlevsoil] = g.zsoi[nlevsoil] + 0.5 * g.dzsoi[nlevsoil];
  return g;
}

// The arrays of one lake point after lakeini (REAL values, the grid's); index as LakeColumn, plus the soil arrays
struct LakeInitPoint {
  real lakedepth2d, savedtke12d, snowdp2d, h2osno2d, snl2d, t_grnd2d;
  real t_lake[11], lake_icefrac[11], z_lake[11], dz_lake[11];
  real t_soisno[15], h2osoi_ice[15], h2osoi_liq[15], h2osoi_vol[15], z[15], dz[15];
  real zi[16];
  real watsat[11], csol[11], tkmg[11], tkdry[11], tksatu[11];
};

// a lake point (lake(i,j) = .true.) after 5076-5118: lakedepth2d is the depth chosen there (or spval); tsk, snow,
// isltyp the point's; p.lake_icefrac / snowdp2d / h2osno2d as set before (5085-5090: zero; 4987-4988)
// (isltyp in 1..19: the caller checks; WRF indexes sand / clay out of bounds otherwise)
inline void lakeini_point(const LakeGrid& g, real tsk, int isltyp, LakeInitPoint& p) {
  using std::fill;
  for (auto* a : {p.z, p.dz, p.h2osoi_liq, p.h2osoi_ice, p.h2osoi_vol}) fill(a, a + 15, real(0.0));
  fill(p.zi, p.zi + 16, real(0.0));
  fill(p.lake_icefrac, p.lake_icefrac + 11, real(0.0));
  p.snl2d = real(0.0);
  // soil properties (5227-5255): sand / clay of the category (14 -> 15), the k loop vectorised (pow lanes)
  int isl = isltyp;
  if (isl == 14) isl = isl + 1;
  for (int k = 1; k <= nlevsoil; ++k) {
    const r8 clay2d = rl(kClay[isl - 1]), sand2d = rl(kSand[isl - 1]);  // sand3d / clay3d: REAL
    const real watsat = real(0.489 - 0.00126 * sand2d);
    p.watsat[k] = watsat;
    const r8 bd2d = (1. - r8(watsat)) * 2.7e3;
    const r8 tkm2d = (8.80 * sand2d + 2.92 * clay2d) / (sand2d + clay2d);
    p.tkmg[k] = real(vm::pow(tkm2d, 1. - r8(watsat), k - 1, nlevsoil));
    p.tksatu[k] = real(r8(p.tkmg[k]) * vm::pow(0.57, r8(watsat), k - 1, nlevsoil));
    p.tkdry[k] = real((0.135 * bd2d + 64.7) / (2.7e3 - 0.947 * bd2d));
    p.csol[k] = real((2.128 * sand2d + 2.385 * clay2d) / (sand2d + clay2d) * 1.e6);
  }
  if (r8(p.lakedepth2d) == spval) {
    p.lakedepth2d = real(g.zlak[nlevlake] + 0.5 * g.dzlak[nlevlake]);
    for (int k = 1; k <= nlevlake; ++k) {
      p.z_lake[k] = real(g.zlak[k]);
      p.dz_lake[k] = real(g.dzlak[k]);
    }
  } else {
    const r8 depthratio2d = r8(p.lakedepth2d) / (g.zlak[nlevlake] + 0.5 * g.dzlak[nlevlake]);
    p.z_lake[1] = real(g.zlak[1]);
    p.dz_lake[1] = real(g.dzlak[1]);
    for (int k = 2; k <= nlevlake; ++k) p.dz_lake[k] = real(g.dzlak[k] * depthratio2d);
    for (int k = 2; k <= nlevlake; ++k)
      p.z_lake[k] = real(g.zlak[k] * depthratio2d + r8(p.dz_lake[1]) * (1. - depthratio2d));
  }
  real* ts = p.t_soisno + 4;  // element j + 4
  ts[1] = tsk;
  p.t_lake[1] = tsk;
  p.t_grnd2d = real(277.0);
  for (int k = 2; k <= nlevlake; ++k) {
    if (p.z_lake[k] <= depth_c) {
      ts[k] = tsk + (real(277.0) - tsk) / depth_c * p.z_lake[k];
      p.t_lake[k] = tsk + (real(277.0) - tsk) / depth_c * p.z_lake[k];
    } else {
      ts[k] = real(277.0);
      p.t_lake[k] = real(277.0);
    }
  }
  real* z = p.z + 4;
  real* dz = p.dz + 4;
  real* zi = p.zi + 5;
  for (int k = 1; k <= nlevsoil; ++k) {
    z[k] = real(g.zsoi[k]);
    dz[k] = real(g.dzsoi[k]);
  }
  for (int k = 0; k <= nlevsoil; ++k) zi[k] = real(g.zisoi[k]);
  p.savedtke12d = real(tkwat);
  const r8 sd = p.snowdp2d;
  if (sd < 0.01) {
    p.snl2d = 0;
    for (int k = -nlevsnow + 1; k <= 0; ++k) {
      dz[k] = 0.;
      z[k] = 0.;
    }
    for (int k = -nlevsnow; k <= 0; ++k) zi[k] = 0.;
  } else {
    // the layer thicknesses (5293-5340): REAL arrays, each statement rounded to REAL
    const real s = p.snowdp2d;
    if ((sd >= 0.01) && (sd <= 0.03)) {
      p.snl2d = -1;
      dz[0] = s;
    } else if ((sd > 0.03) && (sd <= 0.04)) {
      p.snl2d = -2;
      dz[-1] = real(sd / 2.);
      dz[0] = dz[-1];
    } else if ((sd > 0.04) && (sd <= 0.07)) {
      p.snl2d = -2;
      dz[-1] = real(0.02);
      dz[0] = s - dz[-1];
    } else if ((sd > 0.07) && (sd <= 0.12)) {
      p.snl2d = -3;
      dz[-2] = real(0.02);
      dz[-1] = real((sd - 0.02) / 2.);
      dz[0] = dz[-1];
    } else if ((sd > 0.12) && (sd <= 0.18)) {
      p.snl2d = -3;
      dz[-2] = real(0.02);
      dz[-1] = real(0.05);
      dz[0] = s - dz[-2] - dz[-1];
    } else if ((sd > 0.18) && (sd <= 0.29)) {
      p.snl2d = -4;
      dz[-3] = real(0.02);
      dz[-2] = real(0.05);
      dz[-1] = (s - dz[-3] - dz[-2]) / real(2.);
      dz[0] = dz[-1];
    } else if ((sd > 0.29) && (sd <= 0.41)) {
      p.snl2d = -4;
      dz[-3] = real(0.02);
      dz[-2] = real(0.05);
      dz[-1] = real(0.11);
      dz[0] = s - dz[-3] - dz[-2] - dz[-1];
    } else if ((sd > 0.41) && (sd <= 0.64)) {
      p.snl2d = -5;
      dz[-4] = real(0.02);
      dz[-3] = real(0.05);
      dz[-2] = real(0.11);
      dz[-1] = (s - dz[-4] - dz[-3] - dz[-2]) / real(2.);
      dz[0] = dz[-1];
    } else if (sd > 0.64) {
      p.snl2d = -5;
      dz[-4] = real(0.02);
      dz[-3] = real(0.05);
      dz[-2] = real(0.11);
      dz[-1] = real(0.23);
      dz[0] = s - dz[-4] - dz[-3] - dz[-2] - dz[-1];
    }
  }
  const int snl = int(p.snl2d);
  for (int k = 0; k >= snl + 1; --k) {
    z[k] = real(r8(zi[k]) - 0.5 * r8(dz[k]));
    zi[k - 1] = zi[k] - dz[k];
  }
  if (snl < 0)
    for (int k = snl + 1; k <= 0; ++k) ts[k] = real(250.);  // arbinit
  for (int k = 1; k <= nlevsoil; ++k) ts[k] = p.t_lake[nlevlake];  // arbinit
  for (int k = 1; k <= nlevlake; ++k) p.lake_icefrac[k] = r8(p.t_lake[k]) >= tfrz ? real(0.) : real(1.);
  real* vol = p.h2osoi_vol + 4;
  real* ice = p.h2osoi_ice + 4;
  real* liq = p.h2osoi_liq + 4;
  for (int k = 1; k <= nlevsoil; ++k) {
    vol[k] = real(1.0);  // arbinit
    vol[k] = real(c_min(r8(vol[k]), r8(p.watsat[k])));
    if (r8(ts[k]) <= tfrz) {
      ice[k] = real(r8(dz[k]) * denice * r8(vol[k]));
      liq[k] = real(0.);
    } else {
      ice[k] = real(0.);
      liq[k] = real(r8(dz[k]) * denh2o * r8(vol[k]));
    }
  }
  for (int k = -nlevsnow + 1; k <= 0; ++k) {
    if (k > snl) {
      ice[k] = real(r8(dz[k]) * bdsno);
      liq[k] = real(0.);
    }
  }
}

}  // namespace cppwrf::phys::lake
