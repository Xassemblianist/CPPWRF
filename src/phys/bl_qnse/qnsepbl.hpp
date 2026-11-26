// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Port of WRF 4.8.0 phys/module_bl_qnsepbl.F (qnsepbl, mixlen, prodq2, difcof, vdifh; vdifq and vdifv are MYJ's,
// statement for statement, and come from bl_myj/myjpbl.hpp): the QNSE PBL scheme (bl_pbl_physics = 4) without the
// EDKF mass flux (mfshconv = 1 runs bl_qnse/mfshconv.hpp before; WTHV_MF and LM_BL89 are, as in WRF, not read).

#include "phys/bl_myj/myjpbl.hpp"

namespace cppwrf::phys::qnsepbl {

using myjpbl::MyjCPtr;
using myjpbl::MyjPtr;
static constexpr int kMaxLevels = myjpbl::kMaxLevels;

// The constants module_bl_qnsepbl.F shares with module_bl_myjpbl.F (lines 25-135 are the same) and its own (32-40)
using myjpbl::A2;
using myjpbl::A3;
using myjpbl::A4;
using myjpbl::ADNH;
using myjpbl::ADNM;
using myjpbl::ALPH;
using myjpbl::AUBH;
using myjpbl::AUBM;
using myjpbl::BDNH;
using myjpbl::BDNM;
using myjpbl::BUBH;
using myjpbl::BUBM;
using myjpbl::CAPA;
using myjpbl::CP;
using myjpbl::CUBR;
using myjpbl::EL0MAX;
using myjpbl::EL0MIN;
using myjpbl::ELFC;
using myjpbl::ELOCP;
using myjpbl::EPS1;
using myjpbl::EPSGH;
using myjpbl::EPSGM;
using myjpbl::EPSL;
using myjpbl::EPSRU;
using myjpbl::FH;
using myjpbl::G;
using myjpbl::P608;
using myjpbl::PQ0SEA;
using myjpbl::R_D;
using myjpbl::RCUBR;
using myjpbl::REQU;
using myjpbl::RLIVWV;
using myjpbl::VKARMAN;
using myjpbl::XLV;

static constexpr real EPSQ2L = real(0.01);
static constexpr real C0 = real(0.55);
static constexpr real CEPS = C0 * C0 * C0;  // C0**3, folded by gfortran to the same bits
static constexpr real BLCKDR = real(0.0063);
static constexpr real CN = real(0.75);
static constexpr real AM1 = real(8.0), AM2 = real(2.3), AM3 = real(35.0);
static constexpr real AH1 = real(1.4), AH2 = real(-0.01), AH3 = real(1.29), AH4 = real(2.44), AH5 = real(19.8);
static constexpr real ARIMIN = real(0.127);
static constexpr real BM1 = real(2.88), BM2 = real(16.0);
static constexpr real BH1 = real(3.6), BH2 = real(16.0), BH3 = real(720.0);
static constexpr real EPSKM = real(1.e-3);

// The kernel's column arrays (1-based as in the Fortran; zhk 1 .. nz + 1): in the column work buffer in CUDA FP64
// and FP32 identity builds, local arrays otherwise (as MYJ's, myjpbl.hpp)
#define CPPWRF_QNSE_COLUMN(X) \
  X(real, tk, kMaxLevels + 1) X(real, thek, kMaxLevels + 1) X(real, qk, kMaxLevels + 1) \
  X(real, cwmk, kMaxLevels + 1) X(real, uk, kMaxLevels + 1) X(real, vk, kMaxLevels + 1) \
  X(real, q2k, kMaxLevels + 1) X(real, zhk, kMaxLevels + 2) X(real, s2, kMaxLevels + 1) \
  X(real, gh, kMaxLevels + 1) X(real, ri, kMaxLevels + 1) X(real, el, kMaxLevels + 1) \
  X(real, akhk, kMaxLevels + 1) X(real, akmk, kMaxLevels + 1) X(real, rhok, kMaxLevels + 1) \
  X(real, w1, kMaxLevels + 1) X(real, w2, kMaxLevels + 1) X(real, w3, kMaxLevels + 1) \
  X(real, w4, kMaxLevels + 1) X(real, cm, kMaxLevels + 1) X(real, cr, kMaxLevels + 1) \
  X(real, dtoz, kMaxLevels + 1)

// ----------------------------------------------------------------------
// MIXLEN (module_bl_qnsepbl.F:665-882); en2, dth, elm, rel and q1 are the caller's scratch arrays
// ----------------------------------------------------------------------
CPPWRF_HD inline void mixlen(int lmh, MyjCPtr u, MyjCPtr v, MyjCPtr t, MyjCPtr the, MyjCPtr q, MyjCPtr cwm,
                             MyjCPtr q2, MyjCPtr z, real ustar, real corf, MyjPtr s2, MyjPtr gh, MyjPtr ri,
                             MyjPtr el, real& pblh, int& lpbl, int& lmxl, real& ct, MyjPtr en2, MyjPtr dth,
                             MyjPtr elm, MyjPtr rel, MyjPtr q1) {
  lpbl = 1;
  for (int k = lmh - 1; k >= 1; --k) {
    if (q2[k] <= EPSQ2L * FH) {
      lpbl = k;
      break;
    }
  }
  pblh = z[lpbl] - z[lmh + 1];

  for (int k = 1; k <= lmh; ++k) q1[k] = real(0.);
  for (int k = 1; k <= lmh - 1; ++k) dth[k] = the[k] - the[k + 1];
  for (int k = lmh - 2; k >= 1; --k) {
    if (dth[k] > real(0.) && dth[k + 1] <= real(0.)) {
      dth[k] = dth[k] + ct;
      break;
    }
  }
  ct = real(0.);

  // local gradient Richardson number
  for (int k = 1; k <= lmh - 1; ++k) {
    const real rdz = real(2.) / (z[k] - z[k + 2]);
    const real du = u[k] - u[k + 1];
    const real dv = v[k] - v[k + 1];
    real s2l = (du * du + dv * dv) * rdz * rdz;
    s2l = c_max(s2l, EPSGM);
    s2[k] = s2l;

    const real tem = (t[k] + t[k + 1]) * real(0.5);
    const real thm = (the[k] + the[k + 1]) * real(0.5);
    const real a = thm * P608;
    const real b = (ELOCP / tem - real(1.) - P608) * thm;
    real ghl = (dth[k] * ((q[k] + q[k + 1] + cwm[k] + cwm[k + 1]) * (real(0.5) * P608) + real(1.)) +
                (q[k] - q[k + 1] + cwm[k] - cwm[k + 1]) * a + (cwm[k] - cwm[k + 1]) * b) * rdz;
    if (myjpbl::c_abs(ghl) <= EPSGH) ghl = EPSGH;
    en2[k] = ghl * G / thm;
    gh[k] = ghl;
    ri[k] = en2[k] / s2l;
  }

  // maximum mixing lengths and the level of the PBL top
  lmxl = lmh;
  for (int k = 1; k <= lmh - 1; ++k) {
    const real s2l = s2[k];
    const real ghl = gh[k];
    if (ghl >= EPSGH) {
      if (s2l / ghl <= REQU) {
        elm[k] = EPSL;
        lmxl = k;
      } else {
        const real aubr = (AUBM * s2l + AUBH * ghl) * ghl;
        const real bubr = BUBM * s2l + BUBH * ghl;
        const real qol2st = (-real(0.5) * bubr + std::sqrt(bubr * bubr * real(0.25) - aubr * CUBR)) * RCUBR;
        const real eloq2x = real(1.) / qol2st;
        elm[k] = c_max(std::sqrt(eloq2x * q2[k]), EPSL);
      }
    } else {
      const real aden = (ADNM * s2l + ADNH * ghl) * ghl;
      const real bden = BDNM * s2l + BDNH * ghl;
      const real qol2un = -real(0.5) * bden + std::sqrt(bden * bden * real(0.25) - aden);
      const real eloq2x = real(1.) / (qol2un + EPSRU);
      elm[k] = c_max(std::sqrt(eloq2x * q2[k]), EPSL);
    }
  }
  if (elm[lmh - 1] == EPSL) lmxl = lmh;

  for (int k = lpbl; k <= lmh; ++k) q1[k] = std::sqrt(q2[k]);
  real szq = real(0.);
  real sq = real(0.);
  for (int k = 1; k <= lmh - 1; ++k) {
    const real qdzl = (q1[k] + q1[k + 1]) * (z[k + 1] - z[k + 2]);
    szq = (z[k + 1] + z[k + 2] - z[lmh + 1] - z[lmh + 1]) * qdzl + szq;
    sq = qdzl + sq;
  }
  // asymptotic L in Blackadar's formula
  real el0 = c_min(ALPH * szq * real(0.5) / sq, EL0MAX);
  el0 = c_max(el0, EL0MIN);

  // above the PBL top
  const int lpblm = c_max(lpbl - 1, 1);
  for (int k = 1; k <= lpblm; ++k) {
    el[k] = c_min((z[k] - z[k + 2]) * ELFC, elm[k]);
    rel[k] = el[k] / elm[k];
  }
  // inside the PBL
  if (lpbl < lmh) {
    for (int k = lpbl; k <= lmh - 1; ++k) {
      const real vkrmz = (z[k + 1] - z[lmh + 1]) * VKARMAN;
      el[k] = c_min(vkrmz / (vkrmz / el0 + real(1.)), elm[k]);
      rel[k] = el[k] / elm[k];
    }
  }
  for (int k = lpbl + 1; k <= lmh - 2; ++k) {
    const real srel = c_min(((rel[k - 1] + rel[k + 1]) * real(0.5) + rel[k]) * real(0.5), rel[k]);
    el[k] = c_max(srel * elm[k], EPSL);
  }

  // the QNSE mixing length in the stable case
  const real f = c_max(corf, EPS1);
  const real rlambda = f / (BLCKDR * ustar);
  for (int k = 1; k <= lmh - 1; ++k) {
    if (en2[k] >= real(0.0)) {
      const real vkrmz = (z[k + 1] - z[lmh + 1]) * VKARMAN;
      const real rlb = rlambda + real(1.) / vkrmz;
      const real rln = std::sqrt(real(2.) * en2[k] / q2[k]) / CN;
      el[k] = real(1.) / (rlb + rln);
    }
  }
}

// ----------------------------------------------------------------------
// PRODQ2 (module_bl_qnsepbl.F:886-968). gfortran vectorises the loop (two lanes): (0.5*Q2L)**1.5 is libmvec's
// _ZGVbN2vv_pow for the pairs of k and the scalar pow for an odd last k (vm::pow).
// ----------------------------------------------------------------------
CPPWRF_HD inline void prodq2(int lmh, real dtturbl, real ustar, MyjCPtr s2, MyjCPtr ri, MyjPtr q2, MyjCPtr el,
                             MyjCPtr z, MyjCPtr akm, MyjCPtr akh) {
  const real rc02 = real(2.0) / (C0 * C0);
  for (int k = 1; k <= lmh - 1; ++k) {
    const real s2l = s2[k];
    real q2l = q2[k];
    const real deltaz = real(0.5) * (z[k] - z[k + 2]);
    const real akml = akm[k] * deltaz;
    const real akhl = akh[k] * deltaz;
    const real en2 = ri[k] * s2l;
    const real pr = akml * s2l;          // turbulence production
    const real bpr = akhl * en2;         // buoyancy production (WRF: the WTHV_MF term commented out)
    const real dis = CEPS * vm::pow(real(0.5) * q2l, real(1.5), k - 1, lmh - 1) / el[k];  // dissipation
    q2l = q2l + real(2.0) * (pr - bpr - dis) * dtturbl;
    q2[k] = c_max(q2l, EPSQ2L);
  }
  q2[lmh] = c_max(rc02 * ustar * ustar, EPSQ2L);  // lower boundary condition
}

// ----------------------------------------------------------------------
// DIFCOF (module_bl_qnsepbl.F:973-1061): KM, KH from the QNSE stability functions
// ----------------------------------------------------------------------
CPPWRF_HD inline void difcof(int lmh, MyjCPtr el, MyjCPtr ri, MyjCPtr q2, MyjCPtr z, MyjPtr akm, MyjPtr akh) {
  for (int k = 1; k <= lmh - 1; ++k) {
    const real ell = el[k];
    const real q1l = std::sqrt(real(0.5) * q2[k]);  // SQRT(TKE)
    const real ril = ri[k];
    const real ak0 = C0 * ell * q1l;                // KM in the neutral case
    real alpham, alphah;
    if (ril <= real(0)) {  // unstable
      const real aril = c_min(myjpbl::c_abs(ril), real(2.) * ARIMIN);
      const real aril2 = aril * aril;
      const real aril4 = aril2 * aril2;
      alpham = real(1.0) + BM1 * aril + BM2 * aril2;
      alphah = AH1 + BH1 * aril + BH2 * aril2 + BH3 * aril4;
    } else {  // stable
      const real ril2 = ril * ril;
      alpham = (real(1.0) + AM1 * ril2) / (real(1.0) + AM2 * ril + AM3 * ril2);
      alphah = (AH1 + AH2 * ril + AH3 * ril2) / (real(1.0) + AH4 * ril + AH5 * ril2);
    }
    const real rdz = real(2.) / (z[k] - z[k + 2]);
    const real ak0dz = ak0 * rdz;
    const real akmin = EPSKM * rdz;
    akm[k] = c_max(ak0dz * alpham, akmin);
    akh[k] = c_max(ak0dz * alphah, akmin);
  }
}

// ----------------------------------------------------------------------
// VDIFH (module_bl_qnsepbl.F:1147-1240): THE, Q and CWM together, the countergradient term on THE only
// ----------------------------------------------------------------------
CPPWRF_HD inline void vdifh(real dtdif, int lmh, real thz0, real qz0, real rkhs, real chklowq, real ct, MyjPtr the,
                            MyjPtr q, MyjPtr cwm, MyjCPtr rkh, MyjCPtr z, MyjCPtr rho, MyjPtr cm, MyjPtr cr,
                            MyjPtr dtoz, MyjPtr rkct, MyjPtr rst, MyjPtr rsq, MyjPtr rsc) {
  const real cthf = real(0.5) * ct;
  for (int k = 1; k <= lmh - 1; ++k) {
    dtoz[k] = dtdif / (z[k] - z[k + 1]);
    cr[k] = -dtoz[k] * rkh[k];
    rkct[k] = rkh[k] * (z[k] - z[k + 2]) * cthf;
  }
  cm[1] = dtoz[1] * rkh[1] + rho[1];
  rst[1] = -rkct[1] * dtoz[1] + the[1] * rho[1];
  rsq[1] = q[1] * rho[1];
  rsc[1] = cwm[1] * rho[1];
  for (int k = 2; k <= lmh - 1; ++k) {
    const real dtozl = dtoz[k];
    const real cf = -dtozl * rkh[k - 1] / cm[k - 1];
    cm[k] = -cr[k - 1] * cf + (rkh[k - 1] + rkh[k]) * dtozl + rho[k];
    rst[k] = -rst[k - 1] * cf + (rkct[k - 1] - rkct[k]) * dtozl + the[k] * rho[k];
    rsq[k] = -rsq[k - 1] * cf + q[k] * rho[k];
    rsc[k] = -rsc[k - 1] * cf + cwm[k] * rho[k];
  }
  const real dtozs = dtdif / (z[lmh] - z[lmh + 1]);
  const real rkhh = rkh[lmh - 1];
  const real cf = -dtozs * rkhh / cm[lmh - 1];
  const real rkqs = rkhs * chklowq;
  const real cmb = cr[lmh - 1] * cf;
  const real cmtb = -cmb + (rkhh + rkhs) * dtozs + rho[lmh];
  const real cmqb = -cmb + (rkhh + rkqs) * dtozs + rho[lmh];
  const real cmcb = -cmb + (rkhh)*dtozs + rho[lmh];
  const real rstb = -rst[lmh - 1] * cf + rkct[lmh - 1] * dtozs + the[lmh] * rho[lmh];
  const real rsqb = -rsq[lmh - 1] * cf + q[lmh] * rho[lmh];
  const real rscb = -rsc[lmh - 1] * cf + cwm[lmh] * rho[lmh];
  the[lmh] = (dtozs * rkhs * thz0 + rstb) / cmtb;
  q[lmh] = (dtozs * rkqs * qz0 + rsqb) / cmqb;
  cwm[lmh] = (rscb) / cmcb;
  for (int k = lmh - 1; k >= 1; --k) {
    const real rcml = real(1.) / cm[k];
    the[k] = (-cr[k] * the[k + 1] + rst[k]) * rcml;
    q[k] = (-cr[k] * q[k + 1] + rsq[k]) * rcml;
    cwm[k] = (-cr[k] * cwm[k + 1] + rsc[k]) * rcml;
  }
}

// ----------------------------------------------------------------------
// QNSEPBL (module_bl_qnsepbl.F:142-660), one column. WRF's three passes over a row (TKE and the exchange
// coefficients; heat and moisture; momentum) do not mix columns (LOWLYR = 1 everywhere, so the momentum pass's LMH,
// left over from the previous pass, is the column's), and run here one after the other.
// ----------------------------------------------------------------------
struct QNSEPBLKernel {
  View2<const real> ht;
  View3<const real> dz, pmid, pint, th, t, exner, qv, cwm, u, v;
  View2<const real> tsk, chklowq, xland, sice, snow, elflx, hfx;
  BView2<const real> corf;  // the Coriolis parameter f (base state)
  View2<const int> lowlyr;
  View2<real> qsfc, qz0, thz0, uz0, vz0, ustar, znt, ct, akhs, akms, pblh;  // inout (WRF intent)
  View2<int> kpbl;
  View3<real> tke, exch_h, exch_m, el_myj;
  View3<real> rublten, rvblten, rthblten, rqvblten, rqcblten;  // incremented, as WRF's (the driver zeroes them)
  real dt;
  int stepbl;
  int nz;
#if defined(CPPWRF_SPLIT_PATHS)
  ColumnWork work{};  // the GPU's column arrays (column_for_work)
  static constexpr std::size_t kWorkBytes = 0 CPPWRF_QNSE_COLUMN(CPPWRF_COLUMN_BYTES);
#endif

  CPPWRF_HD void operator()(int i, int j) const {
    const real dtturbl = dt * real(stepbl);
    const real rdtturbl = real(1.) / dtturbl;
    const real dtdif = dtturbl;
    const int llow = lowlyr(i, j);
    const int lmh = nz - llow + 1;

    CPPWRF_MYJ_SLOTS(work, i, j)
    CPPWRF_QNSE_COLUMN(CPPWRF_MYJ_ARRAY)

    // interface heights ZINT (lines 271-295), the flipped column (320-350); THE uses T and TH of the level
    zhk[nz + 1] = ht(i, j);
    for (int kf = nz; kf >= 1; --kf) zhk[kf] = zhk[kf + 1] + dz(i, j, nz - kf);
    for (int kf = 1; kf <= nz; ++kf) {
      const int kw = nz - kf;
      tk[kf] = t(i, j, kw);
      cwmk[kf] = cwm(i, j, kw);
      thek[kf] = (cwmk[kf] * (-ELOCP / tk[kf]) + real(1.)) * th(i, j, kw);
      const real ratiomx = qv(i, j, kw);
      qk[kf] = ratiomx / (real(1.) + ratiomx);
      uk[kf] = u(i, j, kw);
      vk[kf] = v(i, j, kw);
      q2k[kf] = real(2.) * tke(i, j, kw);
    }
    for (int kw = 0; kw < nz; ++kw) el_myj(i, j, kw) = real(0.);  // EL_MYJ(its:ite,:,jts:jte) = 0.

    // mixing length (w1..w4, cm: EN2, DTH, ELM, REL, Q1)
    int lpbl = lmh, lmxl = lmh;
    real pblh_v = real(0.);
    real ct_v = ct(i, j);
    mixlen(lmh, uk, vk, tk, thek, qk, cwmk, q2k, zhk, ustar(i, j), corf(i, j), s2, gh, ri, el, pblh_v, lpbl, lmxl,
           ct_v, w1, w2, w3, w4, cm);
    ct(i, j) = ct_v;
    pblh(i, j) = pblh_v;
    kpbl(i, j) = nz - lpbl + 1;

    // exchange coefficients (AKH / AKM: akhk / akmk, kept for the diffusion passes)
    difcof(lmh, el, ri, q2k, zhk, akmk, akhk);
    for (int k = 1; k <= nz - 1; ++k) {
      const int kflip = nz - k;
      const real deltaz = real(0.5) * (zhk[kflip] - zhk[kflip + 2]);
      exch_h(i, j, k) = akhk[kflip] * deltaz;
      exch_m(i, j, k) = akmk[kflip] * deltaz;
    }

    // production / dissipation and vertical diffusion of TKE (w1, w2: VDIFQ's AKQ, RSQ2)
    prodq2(lmh, dtturbl, ustar(i, j), s2, ri, q2k, el, zhk, akmk, akhk);
    myjpbl::vdifq(lmh, dtdif, q2k, el, zhk, w1, w2, cm, cr, dtoz);
    for (int kw = 0; kw < nz; ++kw) {
      const int kflip = nz - kw;
      q2k[kflip] = c_max(q2k[kflip], EPSQ2L);
      tke(i, j, kw) = real(0.5) * q2k[kflip];
      if (kflip != nz) el_myj(i, j, kw) = el[kflip];  // EL is not defined at KTE
    }

    // surface potential temperature from the sensible heat flux (lines 442-451)
    const real psfc = pint(i, j, llow - 1);
    const real rlow = psfc / (R_D * t(i, j, 0));
    const real thsk = hfx(i, j) / (akhs(i, j) * rlow * CP) + th(i, j, 0);

    // heat and moisture (lines 467-546): QK, CWMK and THEK of the start of the step
    for (int kf = 1; kf <= nz; ++kf) {
      const int kw = nz - kf;
      thek[kf] = (cwm(i, j, kw) * (-ELOCP / t(i, j, kw)) + real(1.)) * th(i, j, kw);
      const real ratiomx = qv(i, j, kw);
      qk[kf] = ratiomx / (real(1.) + ratiomx);
      cwmk[kf] = cwm(i, j, kw);
      rhok[kf] = pmid(i, j, kw) / (R_D * t(i, j, kw) * (real(1.) + P608 * qk[kf] - cwmk[kf]));
    }
    for (int k = 1; k <= nz - 1; ++k) akhk[k] = akhk[k] * real(0.5) * (rhok[k] + rhok[k + 1]);

    const real seamask = xland(i, j) - real(1.);
    real thz0_v = (real(1.) - seamask) * thsk + seamask * thz0(i, j);
    thz0(i, j) = thz0_v;
    const real akhs_dens = akhs(i, j) * rhok[nz + 1 - llow];
    real qsfc_v = qsfc(i, j);
    if (seamask < real(0.5)) {
      real qfc1 = XLV * chklowq(i, j) * akhs_dens;
      if (snow(i, j) > real(0.) || sice(i, j) > real(0.5)) qfc1 = qfc1 * RLIVWV;
      if (qfc1 > real(0.)) {
        const real qlow = qk[nz + 1 - llow];
        qsfc_v = qlow + elflx(i, j) / qfc1;
      }
    } else {
      const real exnsfc = rpow(real(1.e5) / psfc, CAPA);
      qsfc_v = PQ0SEA / psfc * libm::exp(A2 * (thsk - A3 * exnsfc) / (thsk - A4 * exnsfc));
    }
    qsfc(i, j) = qsfc_v;
    const real qz0_v = (real(1.) - seamask) * qsfc_v + seamask * qz0(i, j);
    qz0(i, j) = qz0_v;

    vdifh(dtdif, lmh, thz0_v, qz0_v, akhs_dens, chklowq(i, j), ct_v, thek, qk, cwmk, akhk, zhk, rhok, cm, cr, dtoz,
          w1, w2, w3, w4);
    for (int kw = 0; kw < nz; ++kw) {
      const int kflip = nz - kw;
      const real thold = th(i, j, kw);
      const real thnew = thek[kflip] + cwmk[kflip] * ELOCP * (real(1.) / exner(i, j, kw));
      const real dtdt = (thnew - thold) * rdtturbl;
      const real qold = qv(i, j, kw) / (real(1.) + qv(i, j, kw));
      const real dqdt = (qk[kflip] - qold) * rdtturbl;
      const real dcdt = (cwmk[kflip] - cwm(i, j, kw)) * rdtturbl;
      rthblten(i, j, kw) = dtdt + rthblten(i, j, kw);
      rqvblten(i, j, kw) = dqdt / ((real(1.) - qk[kflip]) * (real(1.) - qk[kflip])) + rqvblten(i, j, kw);
      rqcblten(i, j, kw) = dcdt + rqcblten(i, j, kw);
    }

    // momentum (lines 608-651; w1, w2: VDIFV's RSU, RSV)
    for (int k = 1; k <= nz - 1; ++k) akmk[k] = akmk[k] * (rhok[k] + rhok[k + 1]) * real(0.5);
    const real akms_dens = akms(i, j) * rhok[nz + 1 - llow];
    for (int kf = 1; kf <= nz; ++kf) {
      uk[kf] = u(i, j, nz - kf);
      vk[kf] = v(i, j, nz - kf);
    }
    myjpbl::vdifv(lmh, dtdif, uz0(i, j), vz0(i, j), akms_dens, uk, vk, akmk, zhk, rhok, w1, w2, cm, cr, dtoz);
    for (int kw = 0; kw < nz; ++kw) {
      const int kflip = nz - kw;
      rublten(i, j, kw) = (uk[kflip] - u(i, j, kw)) * rdtturbl + rublten(i, j, kw);
      rvblten(i, j, kw) = (vk[kflip] - v(i, j, kw)) * rdtturbl + rvblten(i, j, kw);
    }
  }
};

// The GPU work buffer of run_qnsepbl over range (outside the time loop; CUDA FP64 and FP32 identity, else a no-op)
void qnsepbl_reserve(const Range2D& range);
void run_qnsepbl(const QNSEPBLKernel& kernel, const Range2D& range);

}  // namespace cppwrf::phys::qnsepbl
