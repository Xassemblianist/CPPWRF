// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// WDM6 (WRF mp_physics = 16): phys/module_mp_wdm6.F of WRF v4.8.0 (wdm6, wdm62D, slope_wdm6, slope_rain, slope_snow,
// slope_graup, nislfv_rain_plmr, nislfv_rain_plm6, effectRad_wdm6, wdm6init). Double-moment warm rain (cloud and rain
// number NC, NR and the CCN number NN, kg-1) with WSM6's single-moment ice, snow and graupel (hail with hail_opt = 1).
// WRF's wdm62D works on an i-k slab (i fastest); every column is independent, so a thread runs one column with the
// statements in WRF's order. WRF's k loops between two column-wide steps (sedimentation, slope_wdm6 calls) only read
// and write level k, so they run here as one k loop per stage (the same operations on each level in the same order).
// gfortran (-O2 -ftree-vectorize) vectorises one loop with libmvec: the diffac / venfac loop before the warm-rain
// processes (2-lane log(t), exp(1.81 log t), log, exp: vm:: lanes over its..ite = i0..i0+ni-1); every other exp / log
// / pow is the scalar libm (checked against the -r8 build's object: the lanes of module_mp_wdm6.o are that loop's).
// refl10cm_wdm6 (DOUBLE PRECISION, REAL*16 in the -r8 build) is not ported: do_radar_ref = 1 with WDM6 is an error.
// effectRad_wdm6's DOUBLE PRECISION lamc (REAL*16) holds a REAL*8 value; sngl(1.0d0/lamc) is the quad quotient
// rounded to REAL*8, which equals the REAL*8 quotient (113 >= 2*53 + 2 bits: the double rounding is exact).

#include <cmath>

#include "core/backend.hpp"
#include "core/field.hpp"
#include "core/libm.hpp"
#include "phys/mp_wsm6/wsm6.hpp"
#include "phys/phys_constants.hpp"

namespace cppwrf::phys::wdm6 {

constexpr int kMaxLevels = 128;

template <typename Real>
struct Wdm6Params {
  // the microphysics_driver arguments (module_model_constants) and the namelist's ccn_conc
  Real cpd{}, cpv{}, rv{}, t0c{}, ep2{}, qmin{}, xls{}, xlv0{}, xlf0{}, den0{}, denr{}, cliq{}, cice{}, psat{}, ccn0{};
  // module_mp_wdm6.F parameters
  static constexpr double dtcldcr = 120., n0r = 8.e6, n0s = 2.e6, n0smax = 1.e11, dens = 100.0, alpha = .12,
                          avtr = 841.9, bvtr = 0.8, avts = 11.72, bvts = .41, lamdacmax = 5.e5, lamdacmin = 2.e4,
                          lamdarmax = 5.e4, lamdarmin = 2.e3, lamdasmax = 1.e5, r0 = .8e-5, peaut = .55, xncr = 3.e8,
                          xncr0 = 5.e7, xncr1 = 5.e8, xmyu = 1.718e-5, dicon = 11.9, dimax = 500.e-6, pfrz1 = 100.,
                          pfrz2 = 0.66, qcrmin = 1.e-9, ncmin = 1.e1, nrmin = 1.e-2, eacrc = 1.0, qs0 = 6.e-4,
                          satmax = 1.0048, actk = 0.6, actr = 1.5, ncrk1 = 3.03e3, ncrk2 = 2.59e15, di100 = 1.e-4,
                          di600 = 6.e-4, di2000 = 2000.e-6, di82 = 82.e-6, di15 = 15.e-6;
  // wdm6init (module SAVE variables)
  Real qc0{}, qc1{}, qck1{}, pidnc{}, bvtr1{}, bvtr2{}, bvtr3{}, bvtr4{}, bvtr5{}, bvtr6{}, bvtr7{}, bvtr2o5{},
      bvtr3o5{}, g1pbr{}, g2pbr{}, g3pbr{}, g4pbr{}, g5pbr{}, g6pbr{}, g7pbr{}, g5pbro2{}, g7pbro2{}, pi{}, pvtr{},
      pvtrn{}, eacrr{}, pacrr{}, pidn0r{}, pidnr{}, precr1{}, precr2{}, xmmax{}, roqimax{}, bvts1{}, bvts2{}, bvts3{},
      bvts4{}, g1pbs{}, g3pbs{}, g4pbs{}, g5pbso2{}, pvts{}, pacrs{}, precs1{}, precs2{}, pidn0s{}, xlv1{}, pacrc{},
      bvtg1{}, bvtg2{}, bvtg3{}, bvtg4{}, g1pbg{}, g3pbg{}, g4pbg{}, g5pbgo2{}, pvtg{}, pacrg{}, precg1{}, precg2{},
      pidn0g{}, n0g{}, avtg{}, bvtg{}, deng{}, lamdagmax{}, rslopecmax{}, rslopec2max{}, rslopec3max{}, rslopermax{},
      rslopesmax{}, rslopegmax{}, rsloperbmax{}, rslopesbmax{}, rslopegbmax{}, rsloper2max{}, rslopes2max{},
      rslopeg2max{}, rsloper3max{}, rslopes3max{}, rslopeg3max{};

  // wdm6init(den0, denr, dens, cl, cpv, ccn0, hail_opt) on the host. gfortran folds the powers of parameters (r0**3.,
  // dimax**8, (dimax/dicon)**2, avtr**.5, avts**.5: correctly rounded, = the libm values here); den0 / denr are
  // arguments, so denr**(1./3.), den0**(4./3.) and the max slopes' powers are libm calls (checked against WRF's
  // module variables by tests/phys/mp_wdm)
  void init(Real p_den0, Real p_denr, Real p_dens, Real cl, Real p_cpv, int hail_opt) {
    den0 = p_den0;
    denr = p_denr;
    if (hail_opt == 1) {  // hail
      n0g = Real(4.e4);
      deng = Real(700.);
      avtg = Real(285.0);
      bvtg = Real(0.8);
      lamdagmax = Real(2.e4);
    } else {  // graupel
      n0g = Real(4.e6);
      deng = Real(500);
      avtg = Real(330.0);
      bvtg = Real(0.8);
      lamdagmax = Real(6.e4);
    }
    pi = Real(4. * std::atan(1.));
    xlv1 = cl - p_cpv;
    constexpr double r03 = r0 * r0 * r0;  // r0**3., folded: the correctly rounded cube (= r0*r0*r0 here)
    qc0 = Real(4. / 3.) * pi * denr * Real(r03) * Real(xncr0) / den0;
    qc1 = Real(4. / 3.) * pi * denr * Real(r03) * Real(xncr1) / den0;
    qck1 = Real(.104 * 9.8 * peaut) / libm::pow(denr, Real(1. / 3.)) / Real(xmyu) * libm::pow(den0, Real(4. / 3.));
    pidnc = pi * denr / Real(6.);
    bvtr1 = Real(1. + bvtr);
    bvtr2 = Real(2. + bvtr);
    bvtr3 = Real(3. + bvtr);
    bvtr4 = Real(4. + bvtr);
    bvtr5 = Real(5. + bvtr);
    bvtr6 = Real(6. + bvtr);
    bvtr7 = Real(7. + bvtr);
    bvtr2o5 = Real(2.5 + .5 * bvtr);
    bvtr3o5 = Real(3.5 + .5 * bvtr);
    g1pbr = wsm6::rgmma_calc(bvtr1);
    g2pbr = wsm6::rgmma_calc(bvtr2);
    g3pbr = wsm6::rgmma_calc(bvtr3);
    g4pbr = wsm6::rgmma_calc(bvtr4);
    g5pbr = wsm6::rgmma_calc(bvtr5);
    g6pbr = wsm6::rgmma_calc(bvtr6);
    g7pbr = wsm6::rgmma_calc(bvtr7);
    g5pbro2 = wsm6::rgmma_calc(bvtr2o5);
    g7pbro2 = wsm6::rgmma_calc(bvtr3o5);
    pvtr = Real(avtr) * g5pbr / Real(24.);
    pvtrn = Real(avtr) * g2pbr;
    eacrr = Real(1.0);
    pacrr = pi * Real(n0r) * Real(avtr) * g3pbr * Real(.25) * eacrr;
    precr1 = Real(2.) * pi * Real(1.56);
    precr2 = Real(2.) * pi * Real(.31) * Real(std::sqrt(avtr)) * g7pbro2;
    pidn0r = pi * denr * Real(n0r);
    pidnr = Real(4.) * pi * denr;
    xmmax = Real((dimax / dicon) * (dimax / dicon));
    roqimax = Real(2.08e22 * std::pow(dimax, 8.));
    bvts1 = Real(1. + bvts);
    bvts2 = Real(2.5 + .5 * bvts);
    bvts3 = Real(3. + bvts);
    bvts4 = Real(4. + bvts);
    g1pbs = wsm6::rgmma_calc(bvts1);
    g3pbs = wsm6::rgmma_calc(bvts3);
    g4pbs = wsm6::rgmma_calc(bvts4);
    g5pbso2 = wsm6::rgmma_calc(bvts2);
    pvts = Real(avts) * g4pbs / Real(6.);
    pacrs = pi * Real(n0s) * Real(avts) * g3pbs * Real(.25);
    precs1 = Real(4. * n0s * .65);
    precs2 = Real(4. * n0s * .44) * Real(std::sqrt(avts)) * g5pbso2;
    pidn0s = pi * p_dens * Real(n0s);
    pacrc = pi * Real(n0s) * Real(avts) * g3pbs * Real(.25) * Real(eacrc);
    bvtg1 = Real(1.) + bvtg;
    bvtg2 = Real(2.5) + Real(.5) * bvtg;
    bvtg3 = Real(3.) + bvtg;
    bvtg4 = Real(4.) + bvtg;
    g1pbg = wsm6::rgmma_calc(bvtg1);
    g3pbg = wsm6::rgmma_calc(bvtg3);
    g4pbg = wsm6::rgmma_calc(bvtg4);
    g5pbgo2 = wsm6::rgmma_calc(bvtg2);
    pacrg = pi * n0g * avtg * g3pbg * Real(.25);
    pvtg = avtg * g4pbg / Real(6.);
    precg1 = Real(2.) * pi * n0g * Real(.78);
    precg2 = Real(2.) * pi * n0g * Real(.31) * libm::pow(avtg, Real(.5)) * g5pbgo2;
    pidn0g = pi * deng * n0g;
    rslopecmax = Real(1.) / Real(lamdacmax);
    rslopermax = Real(1.) / Real(lamdarmax);
    rslopesmax = Real(1.) / Real(lamdasmax);
    rslopegmax = Real(1.) / lamdagmax;
    rsloperbmax = libm::pow(rslopermax, Real(bvtr));
    rslopesbmax = libm::pow(rslopesmax, Real(bvts));
    rslopegbmax = libm::pow(rslopegmax, bvtg);
    rslopec2max = rslopecmax * rslopecmax;
    rsloper2max = rslopermax * rslopermax;
    rslopes2max = rslopesmax * rslopesmax;
    rslopeg2max = rslopegmax * rslopegmax;
    rslopec3max = rslopec2max * rslopecmax;
    rsloper3max = rsloper2max * rslopermax;
    rslopes3max = rslopes2max * rslopesmax;
    rslopeg3max = rslopeg2max * rslopegmax;
  }
};

// the microphysics_driver arguments (module_model_constants: CPV = 4 R_V, QMIN = epsilon = 1e-15, PSAT = 610.78),
// ccn_conc, and wdm6init(rhoair0, rhowater, rhosnow, cliq, cpv, ccn_conc, hail_opt) of phy_init
template <typename Real>
inline Wdm6Params<Real> wdm6_params(int hail_opt, double ccn_conc) {
  namespace pc = phys::cst;
  Wdm6Params<Real> c;
  c.cpd = Real(pc::cp);
  c.cpv = Real(4.0 * pc::r_v);
  c.rv = Real(pc::r_v);
  c.t0c = Real(pc::svpt0);
  c.ep2 = Real(pc::ep_2);
  c.qmin = Real(1.e-15);
  c.xls = Real(pc::xls);
  c.xlv0 = Real(pc::xlv);
  c.xlf0 = Real(pc::xlf);
  c.cliq = Real(pc::cliq);
  c.cice = Real(pc::cice);
  c.psat = Real(610.78);
  c.ccn0 = Real(ccn_conc);
  c.init(Real(pc::rhoair0), Real(pc::rhowater), Real(pc::rhosnow), Real(pc::cliq), Real(4.0 * pc::r_v), hail_opt);
  return c;
}

// slope_wdm6 / slope_rain / slope_snow / slope_graup at one level: the slope parameters rs, rb = rs**b, r2 = rs**2,
// r3 = rs**3 and the fall speeds vt of rain (1), snow (2) and graupel (3), vtn of the rain number
template <typename Real>
struct WdmSlope {
  Real rs[3], rb[3], r2[3], r3[3], vt[3], vtn;
};

// slope_rain (the rain part of slope_wdm6): qr, nr, den, denfac at one level
template <typename Real, class P>
CPPWRF_HD inline void wdm_slope_rain(const P& c, Real qr, Real nr, Real den, Real denfac, Real& rs, Real& rb, Real& r2,
                                     Real& r3, Real& vt, Real& vtn) {
  if (qr <= Real(c.qcrmin) || nr <= Real(c.nrmin)) {
    rs = c.rslopermax;
    rb = c.rsloperbmax;
    r2 = c.rsloper2max;
    r3 = c.rsloper3max;
  } else {  // lamdar = exp(log((pidnr*z)/(x*y))*.33333333)
    rs = c_min(Real(1.) / libm::exp(libm::log(((c.pidnr * nr) / (qr * den))) * (Real(.33333333))), Real(1.e-3));
    rb = libm::pow(rs, Real(c.bvtr));
    r2 = rs * rs;
    r3 = r2 * rs;
  }
  vt = c.pvtr * rb * denfac;
  vtn = c.pvtrn * rb * denfac;
  if (qr <= Real(0.0)) vt = Real(0.0);
  if (nr <= Real(0.0)) vtn = Real(0.0);
}

// slope_snow: n0sfac from t (its own t0c = 273.15)
template <typename Real, class P>
CPPWRF_HD inline void wdm_slope_snow(const P& c, Real qs, Real den, Real denfac, Real t, Real& rs, Real& rb, Real& r2,
                                     Real& r3, Real& vt) {
  const Real supcol = Real(273.15) - t;
  const Real n0sfac = c_max(c_min(libm::exp(Real(c.alpha) * supcol), Real(c.n0smax / c.n0s)), Real(1.));
  if (qs <= Real(c.qcrmin)) {
    rs = c.rslopesmax;
    rb = c.rslopesbmax;
    r2 = c.rslopes2max;
    r3 = c.rslopes3max;
  } else {
    rs = Real(1.) / std::sqrt(std::sqrt(c.pidn0s * n0sfac / (qs * den)));
    rb = libm::pow(rs, Real(c.bvts));
    r2 = rs * rs;
    r3 = r2 * rs;
  }
  vt = c.pvts * rb * denfac;
  if (qs <= Real(0.0)) vt = Real(0.0);
}

// slope_graup (graupel or hail: the init's n0g, deng, avtg, bvtg)
template <typename Real, class P>
CPPWRF_HD inline void wdm_slope_graup(const P& c, Real qg, Real den, Real denfac, Real& rs, Real& rb, Real& r2,
                                      Real& r3, Real& vt) {
  if (qg <= Real(c.qcrmin)) {
    rs = c.rslopegmax;
    rb = c.rslopegbmax;
    r2 = c.rslopeg2max;
    r3 = c.rslopeg3max;
  } else {
    rs = Real(1.) / std::sqrt(std::sqrt(c.pidn0g / (qg * den)));
    rb = libm::pow(rs, c.bvtg);
    r2 = rs * rs;
    r3 = r2 * rs;
  }
  vt = c.pvtg * rb * denfac;
  if (qg <= Real(0.0)) vt = Real(0.0);
}

// slope_wdm6 at one level (rain, snow, graupel; vtn of the rain number)
template <typename Real, class P>
CPPWRF_HD inline WdmSlope<Real> wdm6_slope(const P& c, Real qr, Real qs, Real qg, Real nr, Real den, Real denfac,
                                           Real t) {
  WdmSlope<Real> s;
  wdm_slope_rain(c, qr, nr, den, denfac, s.rs[0], s.rb[0], s.r2[0], s.r3[0], s.vt[0], s.vtn);
  wdm_slope_snow(c, qs, den, denfac, t, s.rs[1], s.rb[1], s.r2[1], s.r3[1], s.vt[1]);
  wdm_slope_graup(c, qg, den, denfac, s.rs[2], s.rb[2], s.r2[2], s.r3[2], s.vt[2]);
  return s;
}

// nislfv_rain_plmr (iter = 0: the departure fall speed, no slope iteration) and nislfv_rain_plm6 (iter = 1: snow and
// graupel share the mean fall speed of slope_snow / slope_graup at the arrival points, two = true) for one column (WRF's
// i_loop body; km levels): semi-Lagrangian sedimentation of rql = den*q (and rql2) with the fall speed wwl;
// precip / precip2: the amounts that leave the column bottom
template <typename Real, int NL, class P>
CPPWRF_HD inline void wdm_nislfv(const P& c, int km, const Real* den, const Real* denfac, const Real* tk,
                                 const Real* dz, const Real* wwl, Real* rql, Real* rql2, Real& precip1, Real& precip2,
                                 Real dt, int iter, bool two) {
  precip1 = Real(0.0);
  precip2 = Real(0.0);
  Real allold = Real(0.0);
  if (two) {
    for (int k = 0; k < km; ++k) allold = allold + rql[k] + rql2[k];
  } else {
    for (int k = 0; k < km; ++k) allold = allold + rql[k];
  }
  if (allold <= Real(0.0)) return;
  Real ww[NL], wd[NL], wa[NL], was[NL], qn[NL];
  Real wi[NL + 1], zi[NL + 1], za[NL + 1], dza[NL + 1], qa[NL + 1], qa2[NL + 1], qmi[NL + 1], qpi[NL + 1];
  for (int k = 0; k < km; ++k) ww[k] = wwl[k];
  zi[0] = Real(0.0);
  for (int k = 0; k < km; ++k) zi[k + 1] = zi[k] + dz[k];
  for (int k = 0; k < km; ++k) wd[k] = ww[k];
  int n = 1;
  for (;;) {  // label 100
    // (WRF's 2nd-order wi loop is overwritten by the 3rd-order statements)
    const Real fa1 = Real(9.) / Real(16.), fa2 = Real(1.) / Real(16.);
    wi[0] = ww[0];
    wi[1] = Real(0.5) * (ww[1] + ww[0]);
    for (int k = 2; k < km - 1; ++k) wi[k] = fa1 * (ww[k] + ww[k - 1]) - fa2 * (ww[k + 1] + ww[k - 2]);
    wi[km - 1] = Real(0.5) * (ww[km - 1] + ww[km - 2]);
    wi[km] = ww[km - 1];
    for (int k = 1; k < km; ++k)
      if (ww[k] == Real(0.0)) wi[k] = ww[k - 1];
    const Real con1 = Real(0.05);
    for (int k = km - 1; k >= 0; --k) {
      const Real decfl = (wi[k + 1] - wi[k]) * dt / dz[k];
      if (decfl > con1) wi[k] = wi[k + 1] - con1 * dz[k] / dt;
    }
    for (int k = 0; k <= km; ++k) za[k] = zi[k] - wi[k] * dt;
    for (int k = 0; k < km; ++k) dza[k] = za[k + 1] - za[k];
    dza[km] = zi[km] - za[km];
    for (int k = 0; k < km; ++k) {
      qa[k] = rql[k] * dz[k] / dza[k];
      if (two) qa2[k] = rql2[k] * dz[k] / dza[k];
    }
    qa[km] = Real(0.0);
    qa2[km] = Real(0.0);
    if (n <= iter) {  // nislfv_rain_plm6: slope_snow, slope_graup at the arrival points, the mass-weighted mean
      for (int k = 0; k < km; ++k) {
        const Real qr = qa[k] / den[k], qr2 = qa2[k] / den[k];
        Real rs, rb, r2, r3, w1, w2;
        wdm_slope_snow(c, qr, den[k], denfac[k], tk[k], rs, rb, r2, r3, w1);
        wdm_slope_graup(c, qr2, den[k], denfac[k], rs, rb, r2, r3, w2);
        const Real tmp = c_max((qr + qr2), Real(1.E-15));
        if (tmp > Real(1.e-15)) {
          wa[k] = (w1 * qr + w2 * qr2) / tmp;
        } else {
          wa[k] = Real(0.);
        }
      }
      if (n >= 2)
        for (int k = 0; k < km; ++k) wa[k] = Real(0.5) * (wa[k] + was[k]);
      for (int k = 0; k < km; ++k) ww[k] = Real(0.5) * (wd[k] + wa[k]);
      for (int k = 0; k < km; ++k) was[k] = wa[k];
      n = n + 1;
      continue;
    }
    break;
  }
  for (int ist = 1; ist <= (two ? 2 : 1); ++ist) {
    if (ist == 2)
      for (int k = 0; k <= km; ++k) qa[k] = qa2[k];
    Real precip = Real(0.);
    // piecewise linear profile in each layer, monotone
    for (int k = 1; k < km; ++k) {
      const Real dip = (qa[k + 1] - qa[k]) / (dza[k + 1] + dza[k]);
      const Real dim = (qa[k] - qa[k - 1]) / (dza[k - 1] + dza[k]);
      if (dip * dim <= Real(0.0)) {
        qmi[k] = qa[k];
        qpi[k] = qa[k];
      } else {
        qpi[k] = qa[k] + Real(0.5) * (dip + dim) * dza[k];
        qmi[k] = Real(2.0) * qa[k] - qpi[k];
        if (qpi[k] < Real(0.0) || qmi[k] < Real(0.0)) {
          qpi[k] = qa[k];
          qmi[k] = qa[k];
        }
      }
    }
    qpi[0] = qa[0];
    qmi[0] = qa[0];
    qmi[km] = qa[km];
    qpi[km] = qa[km];
    // interpolation to the regular grid (Fortran indices kb, kt, kk: 1-based)
    for (int k = 0; k < km; ++k) qn[k] = Real(0.0);
    int kb = 1, kt = 1;
    for (int k = 1; k <= km; ++k) {
      kb = kb - 1 > 1 ? kb - 1 : 1;
      kt = kt - 1 > 1 ? kt - 1 : 1;
      if (zi[k - 1] >= za[km]) break;
      for (int kk = kb; kk <= km; ++kk)
        if (zi[k - 1] <= za[kk]) {
          kb = kk;
          break;
        }
      for (int kk = kt; kk <= km; ++kk)
        if (zi[k] <= za[kk - 1]) {
          kt = kk;
          break;
        }
      kt = kt - 1;
      if (kt == kb) {
        const Real tl = (zi[k - 1] - za[kb - 1]) / dza[kb - 1];
        const Real th = (zi[k] - za[kb - 1]) / dza[kb - 1];
        const Real tl2 = tl * tl;
        const Real th2 = th * th;
        const Real qqd = Real(0.5) * (qpi[kb - 1] - qmi[kb - 1]);
        const Real qqh = qqd * th2 + qmi[kb - 1] * th;
        const Real qql = qqd * tl2 + qmi[kb - 1] * tl;
        qn[k - 1] = (qqh - qql) / (th - tl);
      } else if (kt > kb) {
        const Real tl = (zi[k - 1] - za[kb - 1]) / dza[kb - 1];
        const Real tl2 = tl * tl;
        Real qqd = Real(0.5) * (qpi[kb - 1] - qmi[kb - 1]);
        const Real qql = qqd * tl2 + qmi[kb - 1] * tl;
        const Real dql = qa[kb - 1] - qql;
        Real zsum = (Real(1.) - tl) * dza[kb - 1];
        Real qsum = dql * dza[kb - 1];
        if (kt - kb > 1)
          for (int m = kb + 1; m <= kt - 1; ++m) {
            zsum = zsum + dza[m - 1];
            qsum = qsum + qa[m - 1] * dza[m - 1];
          }
        const Real th = (zi[k] - za[kt - 1]) / dza[kt - 1];
        const Real th2 = th * th;
        qqd = Real(0.5) * (qpi[kt - 1] - qmi[kt - 1]);
        const Real dqh = qqd * th2 + qmi[kt - 1] * th;
        zsum = zsum + th * dza[kt - 1];
        qsum = qsum + dqh * dza[kt - 1];
        qn[k - 1] = qsum / zsum;
      }
    }
    // the precipitation that leaves the bottom
    for (int k = 0; k < km; ++k) {
      if (za[k] < Real(0.0) && za[k + 1] < Real(0.0)) {
        precip = precip + qa[k] * dza[k];
        continue;
      } else if (za[k] < Real(0.0) && za[k + 1] >= Real(0.0)) {
        precip = precip + qa[k] * (Real(0.0) - za[k]);
        break;
      }
      break;
    }
    if (ist == 1) {
      for (int k = 0; k < km; ++k) rql[k] = qn[k];
      precip1 = precip;
    } else {
      for (int k = 0; k < km; ++k) rql2[k] = qn[k];
      precip2 = precip;
    }
  }
}

// diffus, viscos, xka, diffac, venfac: wdm62D's statement functions (gfortran inlines them as written)
template <typename Real>
CPPWRF_HD inline Real wdm_viscos(Real x, Real y) {
  return Real(1.496e-6) * (x * std::sqrt(x)) / (x + Real(120.)) / y;
}

// wdm6 + wdm62D for the column (i, j); level k = Fortran k - 1. WRF's qci(:,:,1/2) = qc/qi, qrs(:,:,1/2/3) = qr/qs/qg,
// ncr(:,:,1/2/3) = nn/nc/nr. first: WRF's itimestep == 1 (nn = ccn0 everywhere first: wdm6's own loop, run by the
// caller over the whole field)
template <typename Real, int NL = kMaxLevels>
struct Wdm6Kernel {
  Wdm6Params<Real> c;
  Real delt;
  int nz;
  View3<Real> th, q, qc, qr, qi, qs, qg, nn, nc, nr;
  View3<const Real> den, pii, p, delz;
  View2<const Real> xland;
  View2<Real> rain, rainncv, snow, snowncv, graupel, graupelncv, sr;
  int i0 = 0, ni = 1;  // WRF's its..ite (the lanes of the vectorised i loop)

  CPPWRF_HD void operator()(int i, int j) const {
    using std::abs;
    using std::sqrt;
    const int li = i - i0;
    const int kte = nz;
    Real t[NL], qv[NL], qci1[NL], qci2[NL], qrs1[NL], qrs2[NL], qrs3[NL], ncr1[NL], ncr2[NL], ncr3[NL], dn[NL],
        pr[NL], dz[NL];
    Real denfac[NL], xl[NL], cpm[NL], qs1[NL], qs2[NL], rh1[NL], xni[NL], rslopec3[NL], rsl31[NL];
    Real work1r[NL], workn[NL], worka[NL], denqrs2[NL], denqrs3[NL], work1c[NL], denqci[NL];
    for (int k = 0; k < kte; ++k) {
      t[k] = th(i, j, k) * pii(i, j, k);
      qv[k] = q(i, j, k);
      qci1[k] = qc(i, j, k);
      qci2[k] = qi(i, j, k);
      qrs1[k] = qr(i, j, k);
      qrs2[k] = qs(i, j, k);
      qrs3[k] = qg(i, j, k);
      ncr1[k] = nn(i, j, k);
      ncr2[k] = nc(i, j, k);
      ncr3[k] = nr(i, j, k);
      dn[k] = den(i, j, k);
      pr[k] = p(i, j, k);
      dz[k] = delz(i, j, k);
    }
    // padding 0 for negative values generated by dynamics
    for (int k = 0; k < kte; ++k) {
      qci1[k] = c_max(qci1[k], Real(0.0));
      qrs1[k] = c_max(qrs1[k], Real(0.0));
      qci2[k] = c_max(qci2[k], Real(0.0));
      qrs2[k] = c_max(qrs2[k], Real(0.0));
      qrs3[k] = c_max(qrs3[k], Real(0.0));
      ncr1[k] = c_min(c_max(ncr1[k], Real(1.e8)), Real(2.e10));
      ncr2[k] = c_max(ncr2[k], Real(0.0));
      ncr3[k] = c_max(ncr3[k], Real(0.0));
    }
    for (int k = 0; k < kte; ++k) {  // cpmcal, xlcal
      cpm[k] = c.cpd * (Real(1.) - c_max(qv[k], c.qmin)) + c_max(qv[k], c.qmin) * c.cpv;
      xl[k] = c.xlv0 - c.xlv1 * (t[k] - c.t0c);
    }
    const Real qcr = xland(i, j) == Real(2) ? c.qc0 : c.qc1;  // slmsk = xland: 2 water
    Real rainncv_v = Real(0.), snowncv_v = Real(0.), graupelncv_v = Real(0.), sr_v = Real(0.);
    Real rain_v = rain(i, j), snow_v = snow(i, j), graupel_v = graupel(i, j);
    Real tstepsnow = Real(0.), tstepgraup = Real(0.);
    const int loops = c_max(int(std::round(delt / Real(c.dtcldcr))), 1);
    Real dtcld = delt / Real(loops);
    if (delt <= Real(c.dtcldcr)) dtcld = delt;
    const Real pi = c.pi;
    const Real hsub = c.xls, hvap = c.xlv0, cvap = c.cpv;
    const Real ttp = c.t0c + Real(0.01);
    const Real dldt = cvap - c.cliq;
    const Real xa = -dldt / c.rv;
    const Real xb = xa + hvap / (c.rv * ttp);
    const Real dldti = cvap - c.cice;
    const Real xai = -dldti / c.rv;
    const Real xbi = xai + hsub / (c.rv * ttp);

    for (int loop = 1; loop <= loops; ++loop) {
      for (int k = 0; k < kte; ++k) denfac[k] = sqrt(Real(1.0) / dn[k] * c.den0);  // vrec, vsqrt
      Real rh2k[NL];
      for (int k = 0; k < kte; ++k) {  // fpvs inlined
        Real tr = ttp / t[k];
        qs1[k] = c.psat * libm::exp(libm::log(tr) * (xa)) * libm::exp(xb * (Real(1.) - tr));
        qs1[k] = c_min(qs1[k], Real(0.99) * pr[k]);
        qs1[k] = c.ep2 * qs1[k] / (pr[k] - qs1[k]);
        qs1[k] = c_max(qs1[k], c.qmin);
        rh1[k] = c_max(qv[k] / qs1[k], c.qmin);
        tr = ttp / t[k];
        if (t[k] < ttp) {
          qs2[k] = c.psat * libm::exp(libm::log(tr) * (xai)) * libm::exp(xbi * (Real(1.) - tr));
        } else {
          qs2[k] = c.psat * libm::exp(libm::log(tr) * (xa)) * libm::exp(xb * (Real(1.) - tr));
        }
        qs2[k] = c_min(qs2[k], Real(0.99) * pr[k]);
        qs2[k] = c.ep2 * qs2[k] / (pr[k] - qs2[k]);
        qs2[k] = c_max(qs2[k], c.qmin);
        rh2k[k] = c_max(qv[k] / qs2[k], c.qmin);
      }
      for (int k = 0; k < kte; ++k) {
        if (qci1[k] <= c.qmin || ncr2[k] <= Real(c.ncmin)) {
          rslopec3[k] = c.rslopec3max;
        } else {  // lamdac = exp(log((pidnc*z)/(x*y))*.33333333)
          const Real rslopec =
              Real(1.) / libm::exp(libm::log(((c.pidnc * ncr2[k]) / (qci1[k] * dn[k]))) * (Real(.33333333)));
          const Real rslopec2 = rslopec * rslopec;
          rslopec3[k] = rslopec2 * rslopec;
        }
        // Ni: ice crystal number concentration [HDC 5c]
        Real temp = (dn[k] * c_max(qci2[k], c.qmin));
        temp = sqrt(sqrt(temp * temp * temp));
        xni[k] = c_min(c_max(Real(5.38e7) * temp, Real(1.e3)), Real(1.e6));
      }
      // the fallout terms: rain and its number by time splitting (mstep sub-steps), vt / delz
      int mstep = 1;
      for (int k = kte - 1; k >= 0; --k) {
        Real rs, rb, r2, r3;
        wdm_slope_rain(c, qrs1[k], ncr3[k], dn[k], denfac[k], rs, rb, r2, r3, work1r[k], workn[k]);
        work1r[k] = work1r[k] / dz[k];
        workn[k] = workn[k] / dz[k];
        const int numdt = c_max(int(std::round(c_max(work1r[k], workn[k]) * dtcld + Real(.5))), 1);
        if (numdt >= mstep) mstep = numdt;
      }
      Real fall1 = Real(0.);  // fall(i,kts,1)
      for (int n = 1; n <= mstep; ++n) {
        int k = kte - 1;
        Real falk_up = dn[k] * qrs1[k] * work1r[k] / Real(mstep);
        Real falkn_up = ncr3[k] * workn[k] / Real(mstep);
        if (k == 0) fall1 = fall1 + falk_up;
        qrs1[k] = c_max(qrs1[k] - falk_up * dtcld / dn[k], Real(0.));
        ncr3[k] = c_max(ncr3[k] - falkn_up * dtcld, Real(0.));
        for (k = kte - 2; k >= 0; --k) {
          const Real falk = dn[k] * qrs1[k] * work1r[k] / Real(mstep);
          const Real falkn = ncr3[k] * workn[k] / Real(mstep);
          if (k == 0) fall1 = fall1 + falk;
          const Real dqr = c_min(falk * dtcld / dn[k], qrs1[k]);
          const Real dqr1 = c_min(falk_up * dz[k + 1] / dz[k] * dtcld / dn[k], qrs1[k + 1]);
          const Real dnr = c_min(falkn * dtcld, ncr3[k]);
          const Real dnr1 = c_min(falkn_up * dz[k + 1] / dz[k] * dtcld, ncr3[k + 1]);
          qrs1[k] = c_max(qrs1[k] - dqr + dqr1, Real(0.));
          ncr3[k] = c_max(ncr3[k] - dnr + dnr1, Real(0.));
          falk_up = falk;
          falkn_up = falkn;
        }
        if (n < mstep)  // slope_rain of the new profile (after the last sub-step slope_wdm6 replaces it)
          for (k = kte - 1; k >= 0; --k) {
            Real rs, rb, r2, r3;
            wdm_slope_rain(c, qrs1[k], ncr3[k], dn[k], denfac[k], rs, rb, r2, r3, work1r[k], workn[k]);
            work1r[k] = work1r[k] / dz[k];
            workn[k] = workn[k] / dz[k];
          }
      }
      // snow and graupel: semi-Lagrangian with their mass-weighted mean fall speed (slope_wdm6 of the step start: the
      // snow and graupel slopes do not depend on the rain sedimented since)
      for (int k = kte - 1; k >= 0; --k) {
        Real rs, rb, r2, r3, vt2, vt3;
        wdm_slope_snow(c, qrs2[k], dn[k], denfac[k], t[k], rs, rb, r2, r3, vt2);
        wdm_slope_graup(c, qrs3[k], dn[k], denfac[k], rs, rb, r2, r3, vt3);
        const Real qsum = c_max((qrs2[k] + qrs3[k]), Real(1.E-15));
        if (qsum > Real(1.e-15)) {
          worka[k] = (vt2 * qrs2[k] + vt3 * qrs3[k]) / qsum;
        } else {
          worka[k] = Real(0.);
        }
        denqrs2[k] = dn[k] * qrs2[k];
        denqrs3[k] = dn[k] * qrs3[k];
      }
      Real delqrs2, delqrs3;
      wdm_nislfv<Real, NL>(c, kte, dn, denfac, t, dz, worka, denqrs2, denqrs3, delqrs2, delqrs3, dtcld, 1, true);
      for (int k = 0; k < kte; ++k) {
        qrs2[k] = c_max(denqrs2[k] / dn[k], Real(0.));
        qrs3[k] = c_max(denqrs3[k] / dn[k], Real(0.));
      }
      const Real fall2 = delqrs2 / dz[0] / dtcld;
      const Real fall3 = delqrs3 / dz[0] / dtcld;
      // melting of snow and graupel (slope_wdm6 of the sedimented profile)
      for (int k = kte - 1; k >= 0; --k) {
        const WdmSlope<Real> s = wdm6_slope(c, qrs1[k], qrs2[k], qrs3[k], ncr3[k], dn[k], denfac[k], t[k]);
        rsl31[k] = s.r3[0];
        const Real supcol = c.t0c - t[k];
        const Real n0sfac = c_max(c_min(libm::exp(Real(c.alpha) * supcol), Real(c.n0smax / c.n0s)), Real(1.));
        if (t[k] > c.t0c) {
          // psmlt: melting of snow [HL A33] [RH83 A25]
          const Real xlf = c.xlf0;
          const Real visc = wdm_viscos(t[k], dn[k]);
          const Real work2 = libm::exp(libm::log((visc / (Real(8.794e-5) * libm::exp(libm::log(t[k]) * (Real(1.81))) /
                                                          pr[k]))) *
                                       ((Real(.3333333)))) /
                             sqrt(visc) * sqrt(sqrt(c.den0 / dn[k]));
          if (qrs2[k] > Real(0.)) {
            const Real coeres = s.r2[1] * sqrt(s.rs[1] * s.rb[1]);
            Real psmlt = Real(1.414e3) * wdm_viscos(t[k], dn[k]) * dn[k] / xlf * (c.t0c - t[k]) * pi / Real(2.) *
                         n0sfac * (c.precs1 * s.r2[1] + c.precs2 * work2 * coeres) / dn[k];
            psmlt = c_min(c_max(psmlt * dtcld, -qrs2[k]), Real(0.));
            // nsmlt: melting of snow [LH A27]
            if (qrs2[k] > Real(c.qcrmin)) {
              const Real sfac = s.rs[1] * Real(c.n0s) * n0sfac / qrs2[k];
              ncr3[k] = ncr3[k] - sfac * psmlt;
            }
            qrs2[k] = qrs2[k] + psmlt;
            qrs1[k] = qrs1[k] - psmlt;
            t[k] = t[k] + xlf / cpm[k] * psmlt;
          }
          // pgmlt: melting of graupel [HL A23] [LFO 47]
          if (qrs3[k] > Real(0.)) {
            const Real coeres = s.r2[2] * sqrt(s.rs[2] * s.rb[2]);
            Real pgmlt = Real(1.414e3) * wdm_viscos(t[k], dn[k]) * dn[k] / xlf * (c.t0c - t[k]) *
                         (c.precg1 * s.r2[2] + c.precg2 * work2 * coeres) / dn[k];
            pgmlt = c_min(c_max(pgmlt * dtcld, -qrs3[k]), Real(0.));
            // ngmlt: melting of graupel [LH A28]
            if (qrs3[k] > Real(c.qcrmin)) {
              const Real gfac = s.rs[2] * c.n0g / qrs3[k];
              ncr3[k] = ncr3[k] - gfac * pgmlt;
            }
            qrs3[k] = qrs3[k] + pgmlt;
            qrs1[k] = qrs1[k] - pgmlt;
            t[k] = t[k] + xlf / cpm[k] * pgmlt;
          }
        }
      }
      // Vice [m/s]: fallout of ice crystal [HDC 5a]
      for (int k = kte - 1; k >= 0; --k) {
        if (qci2[k] <= Real(0.)) {
          work1c[k] = Real(0.);
        } else {
          const Real xmi = dn[k] * qci2[k] / xni[k];
          const Real diameter = c_max(c_min(Real(c.dicon) * sqrt(xmi), Real(c.dimax)), Real(1.e-25));
          work1c[k] = Real(1.49e4) * libm::exp(libm::log(diameter) * (Real(1.31)));
        }
      }
      for (int k = kte - 1; k >= 0; --k) denqci[k] = dn[k] * qci2[k];
      Real delqi, dummy;
      wdm_nislfv<Real, NL>(c, kte, dn, denfac, t, dz, work1c, denqci, denqci, delqi, dummy, dtcld, 0, false);
      for (int k = 0; k < kte; ++k) qci2[k] = c_max(denqci[k] / dn[k], Real(0.));
      const Real fallc1 = delqi / dz[0] / dtcld;
      // rain (unit is mm/sec; kgm-2s-1: /1000*delt ===> m) ==> mm for wrf
      {
        const Real fallsum = fall1 + fall2 + fall3 + fallc1;
        const Real fallsum_qsi = fall2 + fallc1;
        const Real fallsum_qg = fall3;
        if (fallsum > Real(0.)) {
          rainncv_v = fallsum * dz[0] / c.denr * dtcld * Real(1000.) + rainncv_v;
          rain_v = fallsum * dz[0] / c.denr * dtcld * Real(1000.) + rain_v;
        }
        if (fallsum_qsi > Real(0.)) {
          tstepsnow = fallsum_qsi * dz[0] / c.denr * dtcld * Real(1000.) + tstepsnow;
          snowncv_v = fallsum_qsi * dz[0] / c.denr * dtcld * Real(1000.) + snowncv_v;
          snow_v = fallsum_qsi * dz[0] / c.denr * dtcld * Real(1000.) + snow_v;
        }
        if (fallsum_qg > Real(0.)) {
          tstepgraup = fallsum_qg * dz[0] / c.denr * dtcld * Real(1000.) + tstepgraup;
          graupelncv_v = fallsum_qg * dz[0] / c.denr * dtcld * Real(1000.) + graupelncv_v;
          graupel_v = fallsum_qg * dz[0] / c.denr * dtcld * Real(1000.) + graupel_v;
        }
        if (fallsum > Real(0.)) sr_v = (snowncv_v + graupelncv_v) / (rainncv_v + Real(1.e-12));
      }
      // pimlt / nimlt, pihmf / nihmf, pihtf / nihtf, pgfrz / ngfrz (rslopec3 of the loop start, rslope3 of rain of
      // the slope_wdm6 before the melting)
      for (int k = 0; k < kte; ++k) {
        const Real supcol = c.t0c - t[k];
        Real xlf = c.xls - xl[k];
        if (supcol < Real(0.)) xlf = c.xlf0;
        if (supcol < Real(0) && qci2[k] > Real(0.)) {
          qci1[k] = qci1[k] + qci2[k];
          ncr2[k] = ncr2[k] + xni[k];
          t[k] = t[k] - xlf / cpm[k] * qci2[k];
          qci2[k] = Real(0.);
        }
        if (supcol > Real(40.) && qci1[k] > Real(0.)) {
          qci2[k] = qci2[k] + qci1[k];
          if (ncr2[k] > Real(0.)) ncr2[k] = Real(0.);
          t[k] = t[k] + xlf / cpm[k] * qci1[k];
          qci1[k] = Real(0.);
        }
        if (supcol > Real(0.) && qci1[k] > c.qmin) {
          const Real supcolt = c_min(supcol, Real(70.));
          const Real pfrzdtc = c_min(pi * pi * Real(c.pfrz1) * (libm::exp(Real(c.pfrz2) * supcolt) - Real(1.)) *
                                         c.denr / dn[k] * ncr2[k] * rslopec3[k] * rslopec3[k] / Real(18.) * dtcld,
                                     qci1[k]);
          if (ncr2[k] > Real(c.ncmin)) {
            const Real nfrzdtc = c_min(pi * Real(c.pfrz1) * (libm::exp(Real(c.pfrz2) * supcolt) - Real(1.)) *
                                           ncr2[k] * rslopec3[k] / Real(6.) * dtcld,
                                       ncr2[k]);
            ncr2[k] = ncr2[k] - nfrzdtc;
          }
          qci2[k] = qci2[k] + pfrzdtc;
          t[k] = t[k] + xlf / cpm[k] * pfrzdtc;
          qci1[k] = qci1[k] - pfrzdtc;
        }
        if (supcol > Real(0.) && qrs1[k] > Real(0.)) {
          const Real supcolt = c_min(supcol, Real(70.));
          const Real pfrzdtr = c_min(Real(140.) * (pi * pi) * Real(c.pfrz1) * ncr3[k] * c.denr / dn[k] *
                                         (libm::exp(Real(c.pfrz2) * supcolt) - Real(1.)) * rsl31[k] * rsl31[k] * dtcld,
                                     qrs1[k]);
          if (ncr3[k] > Real(c.nrmin)) {
            const Real nfrzdtr = c_min(Real(4.) * pi * Real(c.pfrz1) * ncr3[k] *
                                           (libm::exp(Real(c.pfrz2) * supcolt) - Real(1.)) * rsl31[k] * dtcld,
                                       ncr3[k]);
            ncr3[k] = ncr3[k] - nfrzdtr;
          }
          qrs3[k] = qrs3[k] + pfrzdtr;
          t[k] = t[k] + xlf / cpm[k] * pfrzdtr;
          qrs1[k] = qrs1[k] - pfrzdtr;
        }
        ncr2[k] = c_max(ncr2[k], Real(0.0));
        ncr3[k] = c_max(ncr3[k], Real(0.0));
      }
      // the processes, level by level: slope_wdm6, mean-volume diameters, diffac / venfac (vectorised in WRF: libmvec
      // lanes), warm rain, cold rain, mass conservation and update
      for (int k = 0; k < kte; ++k) {
        const WdmSlope<Real> s = wdm6_slope(c, qrs1[k], qrs2[k], qrs3[k], ncr3[k], dn[k], denfac[k], t[k]);
        const Real *rslope = s.rs, *rslopeb = s.rb, *rslope2 = s.r2, *rslope3 = s.r3;
        // mean-volume drop diameter of rain [LH A10] ((24.)**(.3333333), folded by gfortran) and of cloud [LH A7]
        const Real avedia2 = rslope[0] * Real(2.884498835045048);
        Real rslopec, rslopec2, rslopec3k;
        if (qci1[k] <= c.qmin || ncr2[k] <= Real(c.ncmin)) {
          rslopec = c.rslopecmax;
          rslopec2 = c.rslopec2max;
          rslopec3k = c.rslopec3max;
        } else {
          rslopec = Real(1.) / libm::exp(libm::log(((c.pidnc * ncr2[k]) / (qci1[k] * dn[k]))) * (Real(.33333333)));
          rslopec2 = rslopec * rslopec;
          rslopec3k = rslopec2 * rslopec;
        }
        const Real avedia1 = rslopec;
        // diffac, venfac (the vectorised loop: diffus once for the three uses)
        Real work11, work12, work2;
        {
          const Real tk = t[k];
          const Real dfs = Real(8.794e-5) * vm::exp(vm::log(tk, li, ni) * (Real(1.81)), li, ni) / pr[k];
          const Real visc = wdm_viscos(tk, dn[k]);
          const Real xka = Real(1.414e3) * visc * dn[k];
          work11 = dn[k] * xl[k] * xl[k] / (xka * c.rv * tk * tk) + Real(1.) / (qs1[k] * dfs);
          work12 = dn[k] * c.xls * c.xls / (xka * c.rv * tk * tk) + Real(1.) / (qs2[k] * dfs);
          work2 = vm::exp(vm::log((visc / dfs), li, ni) * ((Real(.3333333))), li, ni) / sqrt(visc) *
                  sqrt(sqrt(c.den0 / dn[k]));
        }
        Real pcact = Real(0.), prevp = Real(0.), psdep = Real(0.), pgdep = Real(0.), praut = Real(0.),
             psaut = Real(0.), pgaut = Real(0.), pracw = Real(0.), psacw = Real(0.), pgacw = Real(0.),
             pgacr = Real(0.), pgacs = Real(0.), psaci = Real(0.), praci = Real(0.), piacr = Real(0.),
             pracs = Real(0.), psacr = Real(0.), pgaci = Real(0.), pseml = Real(0.), pgeml = Real(0.),
             paacw = Real(0.), pigen = Real(0.), pidep = Real(0.), psevp = Real(0.), pgevp = Real(0.);
        Real nraut = Real(0.), nracw = Real(0.), nccol = Real(0.), nrcol = Real(0.), nsacw = Real(0.),
             ngacw = Real(0.), niacr = Real(0.), nsacr = Real(0.), ngacr = Real(0.), naacw = Real(0.),
             nseml = Real(0.), ngeml = Real(0.);
        (void)pcact;
        // ---- warm rain processes (the double-moment processes of Lim and Hong) ----
        {
          const Real supsat = c_max(qv[k], c.qmin) - qs1[k];
          const Real satdt = supsat / dtcld;
          // praut: auto conversion rate from cloud to rain [LH 9] [CP 17]
          const Real lencon =
              Real(2.7e-2) * dn[k] * qci1[k] * (Real(1.e20 / 16.) * rslopec2 * rslopec2 - Real(0.4));
          const Real lenconcr = c_max(Real(1.2) * lencon, Real(c.qcrmin));
          if (qci1[k] > qcr && ncr2[k] > Real(c.ncmin)) {
            praut = c.qck1 * libm::pow(qci1[k], Real(7. / 3.)) * libm::pow(ncr2[k], Real(-1. / 3.));
            praut = c_min(praut, qci1[k] / dtcld);
            // nraut: auto conversion rate from cloud to rain [LH A6] [CP 18 & 19]
            nraut = Real(3.5e9) * dn[k] * praut;
            if (qrs1[k] > lenconcr) nraut = ncr3[k] / qrs1[k] * praut;
            nraut = c_min(nraut, ncr2[k] / dtcld);
          }
          // pracw, nracw: accretion of cloud water by rain [LH 10] [CP 22 & 23] [LH A9]
          if (qrs1[k] >= lenconcr) {
            if (avedia2 >= Real(c.di100)) {
              nracw = c_min(Real(c.ncrk1) * ncr2[k] * ncr3[k] * (rslopec3k + Real(24.) * rslope3[0]),
                            ncr2[k] / dtcld);
              pracw = c_min(pi / Real(6.) * (c.denr / dn[k]) * Real(c.ncrk1) * ncr2[k] * ncr3[k] * rslopec3k *
                                (Real(2.) * rslopec3k + Real(24.) * rslope3[0]),
                            qci1[k] / dtcld);
            } else {
              nracw = c_min(Real(c.ncrk2) * ncr2[k] * ncr3[k] *
                                (Real(2.) * rslopec3k * rslopec3k + Real(5040.) * rslope3[0] * rslope3[0]),
                            ncr2[k] / dtcld);
              pracw = c_min(pi / Real(6.) * (c.denr / dn[k]) * Real(c.ncrk2) * ncr2[k] * ncr3[k] * rslopec3k *
                                (Real(6.) * rslopec3k * rslopec3k + Real(5040.) * rslope3[0] * rslope3[0]),
                            qci1[k] / dtcld);
            }
          }
          // nccol: self collection of cloud water [LH A8] [CP 24 & 25]
          if (avedia1 >= Real(c.di100)) {
            nccol = Real(c.ncrk1) * ncr2[k] * ncr2[k] * rslopec3k;
          } else {
            nccol = Real(2. * c.ncrk2) * ncr2[k] * ncr2[k] * rslopec3k * rslopec3k;
          }
          // nrcol: self collection of rain-drops and break-up [LH A21] [CP 24 & 25]
          if (qrs1[k] >= lenconcr) {
            if (avedia2 < Real(c.di100)) {
              nrcol = Real(5040. * c.ncrk2) * ncr3[k] * ncr3[k] * rslope3[0] * rslope3[0];
            } else if (avedia2 >= Real(c.di100) && avedia2 < Real(c.di600)) {
              nrcol = Real(24. * c.ncrk1) * ncr3[k] * ncr3[k] * rslope3[0];
            } else if (avedia2 >= Real(c.di600) && avedia2 < Real(c.di2000)) {
              const Real coecol = Real(-2.5e3) * (avedia2 - Real(c.di600));
              nrcol = Real(24.) * libm::exp(coecol) * Real(c.ncrk1) * ncr3[k] * ncr3[k] * rslope3[0];
            } else {
              nrcol = Real(0.);
            }
          }
          // prevp: evaporation / condensation rate of rain [HL A41]
          if (qrs1[k] > Real(0.)) {
            const Real coeres = rslope[0] * sqrt(rslope[0] * rslopeb[0]);
            prevp = (rh1[k] - Real(1.)) * ncr3[k] * (c.precr1 * rslope[0] + c.precr2 * work2 * coeres) / work11;
            if (prevp < Real(0.)) {
              prevp = c_max(prevp, -qrs1[k] / dtcld);
              prevp = c_max(prevp, satdt / Real(2));
              // Nrevp: evaporation / condensation rate of rain [LH A14] (NR -> NCCN)
              if (prevp == -qrs1[k] / dtcld) {
                ncr1[k] = ncr1[k] + ncr3[k];
                ncr3[k] = Real(0.);
              }
            } else {
              prevp = c_min(prevp, satdt / Real(2));
            }
          }
        }
        // ---- cold rain processes (the revised ice microphysics of HDC) ----
        {
          const Real supcol = c.t0c - t[k];
          const Real n0sfac = c_max(c_min(libm::exp(Real(c.alpha) * supcol), Real(c.n0smax / c.n0s)), Real(1.));
          const Real supsat = c_max(qv[k], c.qmin) - qs2[k];
          const Real satdt = supsat / dtcld;
          int ifsat = 0;
          Real temp = (dn[k] * c_max(qci2[k], c.qmin));
          temp = sqrt(sqrt(temp * temp * temp));
          xni[k] = c_min(c_max(Real(5.38e7) * temp, Real(1.e3)), Real(1.e6));
          const Real eacrs = libm::exp(Real(0.07) * (-supcol));
          const Real xmi = dn[k] * qci2[k] / xni[k];
          const Real diameter = c_min(Real(c.dicon) * sqrt(xmi), Real(c.dimax));
          const Real vt2i = Real(1.49e4) * libm::pow(diameter, Real(1.31));
          const Real vt2r = c.pvtr * rslopeb[0] * denfac[k];
          const Real vt2s = c.pvts * rslopeb[1] * denfac[k];
          const Real vt2g = c.pvtg * rslopeb[2] * denfac[k];
          const Real qsum = c_max((qrs2[k] + qrs3[k]), Real(1.e-15));
          Real vt2ave;
          if (qsum > Real(1.e-15)) {
            vt2ave = (vt2s * qrs2[k] + vt2g * qrs3[k]) / (qsum);
          } else {
            vt2ave = Real(0.);
          }
          auto sq = [](Real x) { return x * x; };
          if (supcol > Real(0.) && qci2[k] > c.qmin) {
            if (qrs1[k] > Real(c.qcrmin)) {
              // praci: accretion of cloud ice by rain [HL A15] [LFO 25]
              const Real acrfac =
                  Real(6.) * rslope2[0] + Real(4.) * diameter * rslope[0] + diameter * diameter;
              praci = pi * qci2[k] * ncr3[k] * abs(vt2r - vt2i) * acrfac / Real(4.);
              praci = praci * sq(c_min(c_max(Real(0.0), qrs1[k] / qci2[k]), Real(1.)));
              praci = c_min(praci, qci2[k] / dtcld);
              // piacr: accretion of rain by cloud ice [HL A19] [LFO 26]
              piacr = pi * pi * Real(c.avtr) * ncr3[k] * c.denr * xni[k] * denfac[k] * c.g7pbr * rslope3[0] *
                      rslope2[0] * rslopeb[0] / Real(24.) / dn[k];
              piacr = piacr * sq(c_min(c_max(Real(0.0), qci2[k] / qrs1[k]), Real(1.)));
              piacr = c_min(piacr, qrs1[k] / dtcld);
            }
            // niacr: accretion of rain by cloud ice [LH A25]
            if (ncr3[k] > Real(c.nrmin)) {
              niacr = pi * Real(c.avtr) * ncr3[k] * xni[k] * denfac[k] * c.g4pbr * rslope2[0] * rslopeb[0] /
                      Real(4.);
              niacr = niacr * sq(c_min(c_max(Real(0.0), qci2[k] / qrs1[k]), Real(1.)));
              niacr = c_min(niacr, ncr3[k] / dtcld);
            }
            // psaci: accretion of cloud ice by snow [HDC 10]
            if (qrs2[k] > Real(c.qcrmin)) {
              const Real acrfac = Real(2.) * rslope3[1] + Real(2.) * diameter * rslope2[1] +
                                  diameter * diameter * rslope[1];
              psaci = pi * qci2[k] * eacrs * Real(c.n0s) * n0sfac * abs(vt2ave - vt2i) * acrfac / Real(4.);
              psaci = c_min(psaci, qci2[k] / dtcld);
            }
            // pgaci: accretion of cloud ice by graupel [HL A17] [LFO 41]
            if (qrs3[k] > Real(c.qcrmin)) {
              const Real egi = libm::exp(Real(0.07) * (-supcol));
              const Real acrfac = Real(2.) * rslope3[2] + Real(2.) * diameter * rslope2[2] +
                                  diameter * diameter * rslope[2];
              pgaci = pi * egi * qci2[k] * c.n0g * abs(vt2ave - vt2i) * acrfac / Real(4.);
              pgaci = c_min(pgaci, qci2[k] / dtcld);
            }
          }
          // psacw: accretion of cloud water by snow [HL A7] [LFO 24]
          if (qrs2[k] > Real(c.qcrmin) && qci1[k] > c.qmin)
            psacw = c_min(c.pacrc * n0sfac * rslope3[1] * rslopeb[1] *
                              sq(c_min(c_max(Real(0.0), qrs2[k] / qci1[k]), Real(1.))) * qci1[k] * denfac[k],
                          qci1[k] / dtcld);
          // nsacw: accretion of cloud water by snow [LH A12]
          if (qrs2[k] > Real(c.qcrmin) && ncr2[k] > Real(c.ncmin))
            nsacw = c_min(c.pacrc * n0sfac * rslope3[1] * rslopeb[1] *
                              sq(c_min(c_max(Real(0.0), qrs2[k] / qci1[k]), Real(1.))) * ncr2[k] * denfac[k],
                          ncr2[k] / dtcld);
          // pgacw: accretion of cloud water by graupel [HL A6] [LFO 40]
          if (qrs3[k] > Real(c.qcrmin) && qci1[k] > c.qmin)
            pgacw = c_min(c.pacrg * rslope3[2] * rslopeb[2] * qci1[k] *
                              sq(c_min(c_max(Real(0.0), qrs3[k] / qci1[k]), Real(1.))) * denfac[k],
                          qci1[k] / dtcld);
          // ngacw: accretion of cloud water by graupel [LH A13]
          if (qrs3[k] > Real(c.qcrmin) && ncr2[k] > Real(c.ncmin))
            ngacw = c_min(c.pacrg * rslope3[2] * rslopeb[2] * ncr2[k] *
                              sq(c_min(c_max(Real(0.0), qrs3[k] / qci1[k]), Real(1.))) * denfac[k],
                          ncr2[k] / dtcld);
          // paacw, naacw: accretion of cloud water by averaged snow / graupel
          if (qsum > Real(1.e-15)) {
            paacw = (qrs2[k] * psacw + qrs3[k] * pgacw) / (qsum);
            naacw = (qrs2[k] * nsacw + qrs3[k] * ngacw) / (qsum);
          }
          if (qrs2[k] > Real(c.qcrmin) && qrs1[k] > Real(c.qcrmin)) {
            // pracs: accretion of snow by rain [HL A11] [LFO 27]
            if (supcol > Real(0)) {
              const Real acrfac = Real(5.) * rslope3[1] * rslope3[1] + Real(4.) * rslope3[1] * rslope2[1] * rslope[0] +
                                  Real(1.5) * rslope2[1] * rslope2[1] * rslope2[0];
              pracs = pi * pi * ncr3[k] * Real(c.n0s) * n0sfac * abs(vt2r - vt2ave) * (Real(c.dens) / dn[k]) *
                      acrfac;
              pracs = pracs * sq(c_min(c_max(Real(0.0), qrs1[k] / qrs2[k]), Real(1.)));
              pracs = c_min(pracs, qrs2[k] / dtcld);
            }
            // psacr: accretion of rain by snow [HL A10] [LFO 28]
            const Real acrfac = Real(30.) * rslope3[0] * rslope2[0] * rslope[1] +
                                Real(10.) * rslope2[0] * rslope2[0] * rslope2[1] +
                                Real(2.) * rslope3[0] * rslope3[1];
            psacr = pi * pi * ncr3[k] * Real(c.n0s) * n0sfac * abs(vt2ave - vt2r) * (c.denr / dn[k]) * acrfac;
            psacr = psacr * sq(c_min(c_max(Real(0.0), qrs2[k] / qrs1[k]), Real(1.)));
            psacr = c_min(psacr, qrs1[k] / dtcld);
          }
          // nsacr: accretion of rain by snow [LH A23]
          if (qrs2[k] > Real(c.qcrmin) && ncr3[k] > Real(c.nrmin)) {
            const Real acrfac = Real(1.5) * rslope2[0] * rslope[1] + Real(1.0) * rslope[0] * rslope2[1] +
                                Real(.5) * rslope3[1];
            nsacr = pi * ncr3[k] * Real(c.n0s) * n0sfac * abs(vt2ave - vt2r) * acrfac;
            nsacr = nsacr * sq(c_min(c_max(Real(0.0), qrs2[k] / qrs1[k]), Real(1.)));
            nsacr = c_min(nsacr, ncr3[k] / dtcld);
          }
          // pgacr: accretion of rain by graupel [HL A12] [LFO 42]
          if (qrs3[k] > Real(c.qcrmin) && qrs1[k] > Real(c.qcrmin)) {
            const Real acrfac = Real(30.) * rslope3[0] * rslope2[0] * rslope[2] +
                                Real(10.) * rslope2[0] * rslope2[0] * rslope2[2] +
                                Real(2.) * rslope3[0] * rslope3[2];
            pgacr = pi * pi * ncr3[k] * c.n0g * abs(vt2ave - vt2r) * (c.denr / dn[k]) * acrfac;
            pgacr = pgacr * sq(c_min(c_max(Real(0.0), qrs3[k] / qrs1[k]), Real(1.)));
            pgacr = c_min(pgacr, qrs1[k] / dtcld);
          }
          // ngacr: accretion of rain by graupel [LH A24]
          if (qrs3[k] > Real(c.qcrmin) && ncr3[k] > Real(c.nrmin)) {
            const Real acrfac = Real(1.5) * rslope2[0] * rslope[2] + Real(1.0) * rslope[0] * rslope2[2] +
                                Real(.5) * rslope3[2];
            ngacr = pi * ncr3[k] * c.n0g * abs(vt2ave - vt2r) * acrfac;
            ngacr = ngacr * sq(c_min(c_max(Real(0.0), qrs3[k] / qrs1[k]), Real(1.)));
            ngacr = c_min(ngacr, ncr3[k] / dtcld);
          }
          // pgacs: accretion of snow by graupel, eliminated (0) since V3.0
          if (supcol <= Real(0)) {
            const Real xlf = c.xlf0;
            // pseml / nseml: enhanced melting of snow by accretion of water [HL A34] [LH A29]
            if (qrs2[k] > Real(0.))
              pseml = c_min(c_max(c.cliq * supcol * (paacw + psacr) / xlf, -qrs2[k] / dtcld), Real(0.));
            if (qrs2[k] > Real(c.qcrmin)) {
              const Real sfac = rslope[1] * Real(c.n0s) * n0sfac / qrs2[k];
              nseml = -sfac * pseml;
            }
            // pgeml / ngeml: enhanced melting of graupel by accretion of water [HL A24] [RH84 A21-A22] [LH A30]
            if (qrs3[k] > Real(0.))
              pgeml = c_min(c_max(c.cliq * supcol * (paacw + pgacr) / xlf, -qrs3[k] / dtcld), Real(0.));
            if (qrs3[k] > Real(c.qcrmin)) {
              const Real gfac = rslope[2] * c.n0g / qrs3[k];
              ngeml = -gfac * pgeml;
            }
          }
          if (supcol > Real(0)) {
            // pidep: deposition / sublimation rate of ice [HDC 9]
            if (qci2[k] > Real(0.) && ifsat != 1) {
              pidep = Real(4.) * diameter * xni[k] * (rh2k[k] - Real(1.)) / work12;
              const Real supice = satdt - prevp;
              if (pidep < Real(0.)) {
                pidep = c_max(c_max(pidep, satdt / Real(2)), supice);
                pidep = c_max(pidep, -qci2[k] / dtcld);
              } else {
                pidep = c_min(c_min(pidep, satdt / Real(2)), supice);
              }
              if (abs(prevp + pidep) >= abs(satdt)) ifsat = 1;
            }
            // psdep: deposition / sublimation rate of snow [HDC 14]
            if (qrs2[k] > Real(0.) && ifsat != 1) {
              const Real coeres = rslope2[1] * sqrt(rslope[1] * rslopeb[1]);
              psdep = (rh2k[k] - Real(1.)) * n0sfac * (c.precs1 * rslope2[1] + c.precs2 * work2 * coeres) / work12;
              const Real supice = satdt - prevp - pidep;
              if (psdep < Real(0.)) {
                psdep = c_max(psdep, -qrs2[k] / dtcld);
                psdep = c_max(c_max(psdep, satdt / Real(2)), supice);
              } else {
                psdep = c_min(c_min(psdep, satdt / Real(2)), supice);
              }
              if (abs(prevp + pidep + psdep) >= abs(satdt)) ifsat = 1;
            }
            // pgdep: deposition / sublimation rate of graupel [HL A21] [LFO 46]
            if (qrs3[k] > Real(0.) && ifsat != 1) {
              const Real coeres = rslope2[2] * sqrt(rslope[2] * rslopeb[2]);
              pgdep = (rh2k[k] - Real(1.)) * (c.precg1 * rslope2[2] + c.precg2 * work2 * coeres) / work12;
              const Real supice = satdt - prevp - pidep - psdep;
              if (pgdep < Real(0.)) {
                pgdep = c_max(pgdep, -qrs3[k] / dtcld);
                pgdep = c_max(c_max(pgdep, satdt / Real(2)), supice);
              } else {
                pgdep = c_min(c_min(pgdep, satdt / Real(2)), supice);
              }
              if (abs(prevp + pidep + psdep + pgdep) >= abs(satdt)) ifsat = 1;
            }
            // pigen: generation (nucleation) of ice from vapor [HL 50] [HDC 7-8]
            if (supsat > Real(0.) && ifsat != 1) {
              const Real supice = satdt - prevp - pidep - psdep - pgdep;
              const Real xni0 = Real(1.e3) * libm::exp(Real(0.1) * supcol);
              const Real roqi0 = Real(4.92e-11) * libm::pow(xni0, Real(1.33));
              pigen = c_max(Real(0.), (roqi0 / dn[k] - c_max(qci2[k], Real(0.))) / dtcld);
              pigen = c_min(c_min(pigen, satdt), supice);
            }
            // psaut: conversion (aggregation) of ice to snow [HDC 12]
            if (qci2[k] > Real(0.)) {
              const Real qimax = c.roqimax / dn[k];
              psaut = c_max(Real(0.), (qci2[k] - qimax) / dtcld);
            }
            // pgaut: conversion (aggregation) of snow to graupel [HL A4] [LFO 37]
            if (qrs2[k] > Real(0.)) {
              const Real alpha2 = Real(1.e-3) * libm::exp(Real(0.09) * (-supcol));
              pgaut = c_min(c_max(Real(0.), alpha2 * (qrs2[k] - Real(c.qs0))), qrs2[k] / dtcld);
            }
          }
          if (supcol < Real(0.)) {
            // psevp: evaporation of melting snow [HL A35] [RH83 A27]
            if (qrs2[k] > Real(0.) && rh1[k] < Real(1.)) {
              const Real coeres = rslope2[1] * sqrt(rslope[1] * rslopeb[1]);
              psevp = (rh1[k] - Real(1.)) * n0sfac * (c.precs1 * rslope2[1] + c.precs2 * work2 * coeres) / work11;
              psevp = c_min(c_max(psevp, -qrs2[k] / dtcld), Real(0.));
            }
            // pgevp: evaporation of melting graupel [HL A25] [RH84 A19]
            if (qrs3[k] > Real(0.) && rh1[k] < Real(1.)) {
              const Real coeres = rslope2[2] * sqrt(rslope[2] * rslopeb[2]);
              pgevp = (rh1[k] - Real(1.)) * (c.precg1 * rslope2[2] + c.precg2 * work2 * coeres) / work11;
              pgevp = c_min(c_max(pgevp, -qrs3[k] / dtcld), Real(0.));
            }
          }
        }
        // ---- check mass conservation of the generation terms and feedback to the large scale ----
        {
          Real delta2 = Real(0.), delta3 = Real(0.);
          if (qrs1[k] < Real(1.e-4) && qrs2[k] < Real(1.e-4)) delta2 = Real(1.);
          if (qrs1[k] < Real(1.e-4)) delta3 = Real(1.);
          if (t[k] <= c.t0c) {
            // cloud water
            Real value = c_max(c.qmin, qci1[k]);
            Real source = (praut + pracw + paacw + paacw) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              praut = praut * factor;
              pracw = pracw * factor;
              paacw = paacw * factor;
            }
            // cloud ice
            value = c_max(c.qmin, qci2[k]);
            source = (psaut - pigen - pidep + praci + psaci + pgaci) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              psaut = psaut * factor;
              pigen = pigen * factor;
              pidep = pidep * factor;
              praci = praci * factor;
              psaci = psaci * factor;
              pgaci = pgaci * factor;
            }
            // rain
            value = c_max(c.qmin, qrs1[k]);
            source = (-praut - prevp - pracw + piacr + psacr + pgacr) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              praut = praut * factor;
              prevp = prevp * factor;
              pracw = pracw * factor;
              piacr = piacr * factor;
              psacr = psacr * factor;
              pgacr = pgacr * factor;
            }
            // snow
            value = c_max(c.qmin, qrs2[k]);
            source = -(psdep + psaut - pgaut + paacw + piacr * delta3 + praci * delta3 - pracs * (Real(1.) - delta2) +
                       psacr * delta2 + psaci - pgacs) *
                     dtcld;
            if (source > value) {
              const Real factor = value / source;
              psdep = psdep * factor;
              psaut = psaut * factor;
              pgaut = pgaut * factor;
              paacw = paacw * factor;
              piacr = piacr * factor;
              praci = praci * factor;
              psaci = psaci * factor;
              pracs = pracs * factor;
              psacr = psacr * factor;
              pgacs = pgacs * factor;
            }
            // graupel
            value = c_max(c.qmin, qrs3[k]);
            source = -(pgdep + pgaut + piacr * (Real(1.) - delta3) + praci * (Real(1.) - delta3) +
                       psacr * (Real(1.) - delta2) + pracs * (Real(1.) - delta2) + pgaci + paacw + pgacr + pgacs) *
                     dtcld;
            if (source > value) {
              const Real factor = value / source;
              pgdep = pgdep * factor;
              pgaut = pgaut * factor;
              piacr = piacr * factor;
              praci = praci * factor;
              psacr = psacr * factor;
              pracs = pracs * factor;
              paacw = paacw * factor;
              pgaci = pgaci * factor;
              pgacr = pgacr * factor;
              pgacs = pgacs * factor;
            }
            // cloud number
            value = c_max(Real(c.ncmin), ncr2[k]);
            source = (nraut + nccol + nracw + naacw + naacw) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              nraut = nraut * factor;
              nccol = nccol * factor;
              nracw = nracw * factor;
              naacw = naacw * factor;
            }
            // rain number
            value = c_max(Real(c.nrmin), ncr3[k]);
            source = (-nraut + nrcol + niacr + nsacr + ngacr) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              nraut = nraut * factor;
              nrcol = nrcol * factor;
              niacr = niacr * factor;
              nsacr = nsacr * factor;
              ngacr = ngacr * factor;
            }
            const Real work2u = -(prevp + psdep + pgdep + pigen + pidep);
            // update
            qv[k] = qv[k] + work2u * dtcld;
            qci1[k] = c_max(qci1[k] - (praut + pracw + paacw + paacw) * dtcld, Real(0.));
            qrs1[k] = c_max(qrs1[k] + (praut + pracw + prevp - piacr - pgacr - psacr) * dtcld, Real(0.));
            qci2[k] = c_max(qci2[k] - (psaut + praci + psaci + pgaci - pigen - pidep) * dtcld, Real(0.));
            qrs2[k] = c_max(qrs2[k] + (psdep + psaut + paacw - pgaut + piacr * delta3 + praci * delta3 + psaci -
                                       pgacs - pracs * (Real(1.) - delta2) + psacr * delta2) *
                                          dtcld,
                            Real(0.));
            qrs3[k] = c_max(qrs3[k] + (pgdep + pgaut + piacr * (Real(1.) - delta3) + praci * (Real(1.) - delta3) +
                                       psacr * (Real(1.) - delta2) + pracs * (Real(1.) - delta2) + pgaci + paacw +
                                       pgacr + pgacs) *
                                          dtcld,
                            Real(0.));
            ncr2[k] = c_max(ncr2[k] + (-nraut - nccol - nracw - naacw - naacw) * dtcld, Real(0.));
            ncr3[k] = c_max(ncr3[k] + (nraut - nrcol - niacr - nsacr - ngacr) * dtcld, Real(0.));
            const Real xlf = c.xls - xl[k];
            const Real xlwork2 = -c.xls * (psdep + pgdep + pidep + pigen) - xl[k] * prevp -
                                 xlf * (piacr + paacw + paacw + pgacr + psacr);
            t[k] = t[k] - xlwork2 / cpm[k] * dtcld;
          } else {
            // cloud water
            Real value = c_max(c.qmin, qci1[k]);
            Real source = (praut + pracw + paacw + paacw) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              praut = praut * factor;
              pracw = pracw * factor;
              paacw = paacw * factor;
            }
            // rain
            value = c_max(c.qmin, qrs1[k]);
            source = (-paacw - praut + pseml + pgeml - pracw - paacw - prevp) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              praut = praut * factor;
              prevp = prevp * factor;
              pracw = pracw * factor;
              paacw = paacw * factor;
              pseml = pseml * factor;
              pgeml = pgeml * factor;
            }
            // snow
            value = c_max(Real(c.qcrmin), qrs2[k]);
            source = (pgacs - pseml - psevp) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              pgacs = pgacs * factor;
              psevp = psevp * factor;
              pseml = pseml * factor;
            }
            // graupel
            value = c_max(Real(c.qcrmin), qrs3[k]);
            source = -(pgacs + pgevp + pgeml) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              pgacs = pgacs * factor;
              pgevp = pgevp * factor;
              pgeml = pgeml * factor;
            }
            // cloud number
            value = c_max(Real(c.ncmin), ncr2[k]);
            source = (+nraut + nccol + nracw + naacw + naacw) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              nraut = nraut * factor;
              nccol = nccol * factor;
              nracw = nracw * factor;
              naacw = naacw * factor;
            }
            // rain number
            value = c_max(Real(c.nrmin), ncr3[k]);
            source = (-nraut + nrcol - nseml - ngeml) * dtcld;
            if (source > value) {
              const Real factor = value / source;
              nraut = nraut * factor;
              nrcol = nrcol * factor;
              nseml = nseml * factor;
              ngeml = ngeml * factor;
            }
            const Real work2u = -(prevp + psevp + pgevp);
            // update
            qv[k] = qv[k] + work2u * dtcld;
            qci1[k] = c_max(qci1[k] - (praut + pracw + paacw + paacw) * dtcld, Real(0.));
            qrs1[k] = c_max(qrs1[k] + (praut + pracw + prevp + paacw + paacw - pseml - pgeml) * dtcld, Real(0.));
            qrs2[k] = c_max(qrs2[k] + (psevp - pgacs + pseml) * dtcld, Real(0.));
            qrs3[k] = c_max(qrs3[k] + (pgacs + pgevp + pgeml) * dtcld, Real(0.));
            ncr2[k] = c_max(ncr2[k] + (-nraut - nccol - nracw - naacw - naacw) * dtcld, Real(0.));
            ncr3[k] = c_max(ncr3[k] + (nraut - nrcol + nseml + ngeml) * dtcld, Real(0.));
            const Real xlf = c.xls - xl[k];
            const Real xlwork2 = -xl[k] * (prevp + psevp + pgevp) - xlf * (pseml + pgeml);
            t[k] = t[k] - xlwork2 / cpm[k] * dtcld;
          }
        }
      }
      // the new saturation, Nrevp_s / Prevp_s, CCN activation and condensation, small values, level by level
      for (int k = 0; k < kte; ++k) {
        Real tr = ttp / t[k];
        qs1[k] = c.psat * libm::exp(libm::log(tr) * (xa)) * libm::exp(xb * (Real(1.) - tr));
        qs1[k] = c_min(qs1[k], Real(0.99) * pr[k]);
        qs1[k] = c.ep2 * qs1[k] / (pr[k] - qs1[k]);
        qs1[k] = c_max(qs1[k], c.qmin);
        // (qs(i,k,2) is recomputed here too; nothing reads it before the next loop's saturation)
        rh1[k] = c_max(qv[k] / qs1[k], c.qmin);
        // re-compute the mean-volume drop diameter [LH A10] (slope_wdm6 of the updated profile)
        Real rs, rb, r2, r3, vt, vtn;
        wdm_slope_rain(c, qrs1[k], ncr3[k], dn[k], denfac[k], rs, rb, r2, r3, vt, vtn);
        const Real avedia2 = rs * Real(2.884498835045048);
        // Nrevp_s / Prevp_s: evaporation / condensation of rain [LH A14] [LH A15] [KK 23] (NR -> NC, QR -> QC)
        if (avedia2 <= Real(c.di82)) {
          ncr2[k] = ncr2[k] + ncr3[k];
          ncr3[k] = Real(0.);
          qci1[k] = qci1[k] + qrs1[k];
          qrs1[k] = Real(0.);
        }
        // pcact, ncact: CCN activation [LH 8] [KK 14] [LH A2] [KK 12]
        if (rh1[k] > Real(1.)) {
          Real ncact = c_max(Real(0.), ((ncr1[k] + ncr2[k]) *
                                            c_min(Real(1.), libm::pow((rh1[k] / Real(c.satmax)), Real(c.actk))) -
                                        ncr2[k])) /
                       dtcld;
          ncact = c_min(ncact, c_max(ncr1[k], Real(0.)) / dtcld);
          // (actr*1.E-6)**3, folded by gfortran: the correctly rounded cube 3.375e-18
          const Real pcact = c_min(Real(4.) * pi * c.denr * Real(3.375e-18) * ncact / (Real(3.) * dn[k]),
                                   c_max(qv[k], Real(0.)) / dtcld);
          qv[k] = c_max(qv[k] - pcact * dtcld, Real(0.));
          qci1[k] = c_max(qci1[k] + pcact * dtcld, Real(0.));
          ncr1[k] = c_max(ncr1[k] - ncact * dtcld, Real(0.));
          ncr2[k] = c_max(ncr2[k] + ncact * dtcld, Real(0.));
          t[k] = t[k] + pcact * xl[k] / cpm[k] * dtcld;
        }
        // pcond: condensational / evaporational rate of cloud water [HL A46] [RH83 A6]
        tr = ttp / t[k];
        qs1[k] = c.psat * libm::exp(libm::log(tr) * (xa)) * libm::exp(xb * (Real(1.) - tr));
        qs1[k] = c_min(qs1[k], Real(0.99) * pr[k]);
        qs1[k] = c.ep2 * qs1[k] / (pr[k] - qs1[k]);
        qs1[k] = c_max(qs1[k], c.qmin);
        const Real work11 =
            (c_max(qv[k], c.qmin) - qs1[k]) / (Real(1.) + xl[k] * xl[k] / (c.rv * cpm[k]) * qs1[k] / (t[k] * t[k]));
        Real pcond = c_min(c_max(work11 / dtcld, Real(0.)), c_max(qv[k], Real(0.)) / dtcld);
        if (qci1[k] > Real(0.) && work11 < Real(0.)) pcond = c_max(work11, -qci1[k]) / dtcld;
        // ncevp: evaporation of cloud number [LH A3] (NC -> NCCN)
        if (pcond == -qci1[k] / dtcld) {
          ncr1[k] = ncr1[k] + ncr2[k];
          ncr2[k] = Real(0.);
        }
        qv[k] = qv[k] - pcond * dtcld;
        qci1[k] = c_max(qci1[k] + pcond * dtcld, Real(0.));
        t[k] = t[k] + pcond * xl[k] / cpm[k] * dtcld;
        // padding for small values
        if (qci1[k] <= c.qmin) qci1[k] = Real(0.0);
        if (qci2[k] <= c.qmin) qci2[k] = Real(0.0);
        if (qrs1[k] >= Real(c.qcrmin) && ncr3[k] >= Real(c.nrmin)) {
          Real lamdr = libm::exp(libm::log(((c.pidnr * ncr3[k]) / (dn[k] * qrs1[k]))) * ((Real(.33333333))));
          if (lamdr <= Real(c.lamdarmin)) {
            lamdr = Real(c.lamdarmin);
            ncr3[k] = dn[k] * qrs1[k] * (lamdr * lamdr * lamdr) / c.pidnr;
          } else if (lamdr >= Real(c.lamdarmax)) {
            lamdr = Real(c.lamdarmax);
            ncr3[k] = dn[k] * qrs1[k] * (lamdr * lamdr * lamdr) / c.pidnr;
          }
        }
        if (qci1[k] >= c.qmin && ncr2[k] >= Real(c.ncmin)) {
          Real lamdc = libm::exp(libm::log(((c.pidnc * ncr2[k]) / (dn[k] * qci1[k]))) * ((Real(.33333333))));
          if (lamdc <= Real(c.lamdacmin)) {
            lamdc = Real(c.lamdacmin);
            ncr2[k] = dn[k] * qci1[k] * (lamdc * lamdc * lamdc) / c.pidnc;
          } else if (lamdc >= Real(c.lamdacmax)) {
            lamdc = Real(c.lamdacmax);
            ncr2[k] = dn[k] * qci1[k] * (lamdc * lamdc * lamdc) / c.pidnc;
          }
        }
      }
    }
    for (int k = 0; k < kte; ++k) {
      th(i, j, k) = t[k] / pii(i, j, k);
      q(i, j, k) = qv[k];
      qc(i, j, k) = qci1[k];
      qi(i, j, k) = qci2[k];
      qr(i, j, k) = qrs1[k];
      qs(i, j, k) = qrs2[k];
      qg(i, j, k) = qrs3[k];
      nn(i, j, k) = ncr1[k];
      nc(i, j, k) = ncr2[k];
      nr(i, j, k) = ncr3[k];
    }
    rain(i, j) = rain_v;
    rainncv(i, j) = rainncv_v;
    snow(i, j) = snow_v;
    snowncv(i, j) = snowncv_v;
    graupel(i, j) = graupel_v;
    graupelncv(i, j) = graupelncv_v;
    sr(i, j) = sr_v;
  }
};

// effectRad_wdm6 (also effectRad_wdm5 / _wdm7: the same statements) at (i, j, k) after the scheme, with wdm6's caps:
// re = max(RE_*_BG, min(re, 50e-6 / 125e-6 / 999e-6)); t = th*pii of the updated column, nc the cloud number
template <typename Real>
struct WdmEffRadKernel {
  Real pidnc, pidn0s, alpha, n0s, n0smax, t0c, qmin, cdm2;  // cdm2 = rgmma(5./3.)
  Real re_qc_bg, re_qi_bg, re_qs_bg;
  View3<const Real> th, pii, qc, nc, qi, qs, den;
  View3<Real> re_qc, re_qi, re_qs;
  CPPWRF_HD void operator()(int i, int j, int k) const {
    const Real R1 = Real(1.E-12), R2 = Real(1.E-6), obmr = Real(1.0 / 3.0);
    const Real rho = den(i, j, k), t = th(i, j, k) * pii(i, j, k);
    Real rc = re_qc_bg, ri = re_qi_bg, rs = re_qs_bg;
    const Real rqc = c_max(R1, qc(i, j, k) * rho);
    const Real rnc = c_max(R2, nc(i, j, k) * rho);
    const Real rqi = c_max(R1, qi(i, j, k) * rho);
    Real temp = (rho * c_max(qi(i, j, k), qmin));
    temp = std::sqrt(std::sqrt(temp * temp * temp));
    const Real ni = c_min(c_max(Real(5.38e7) * temp, Real(1.e3)), Real(1.e6));
    const Real rni = c_max(R2, ni * rho);
    const Real rqs = c_max(R1, qs(i, j, k) * rho);
    if (!(rqc <= R1 || rnc <= R2)) {
      const Real lamc = Real(2.) * cdm2 * libm::pow((pidnc * nc(i, j, k) / rqc), obmr);
      rc = c_max(Real(2.51e-6), c_min(Real(1.0) / lamc, Real(50.e-6)));
    }
    if (!(rqi <= R1 || rni <= R2)) {
      const Real diai = Real(11.9) * std::sqrt(rqi / ni);
      ri = c_max(Real(10.01e-6), c_min(Real(0.75) * Real(0.163) * diai, Real(125.e-6)));
    }
    if (!(rqs <= R1)) {
      const Real supcol = t0c - t;
      const Real n0sfac = c_max(c_min(libm::exp(alpha * supcol), n0smax / n0s), Real(1.));
      const Real lamdas = std::sqrt(std::sqrt(pidn0s * n0sfac / rqs));
      rs = c_max(Real(25.e-6), c_min(Real(0.5) * (Real(1.) / lamdas), Real(999.e-6)));
    }
    re_qc(i, j, k) = c_max(re_qc_bg, c_min(rc, Real(50.e-6)));
    re_qi(i, j, k) = c_max(re_qi_bg, c_min(ri, Real(125.e-6)));
    re_qs(i, j, k) = c_max(re_qs_bg, c_min(rs, Real(999.e-6)));
  }
};

}  // namespace cppwrf::phys::wdm6
