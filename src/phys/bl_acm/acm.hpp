// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ACM2 PBL scheme (bl_pbl_physics = 7): WRF v4.8.0 phys/module_bl_acm.F (ACMPBL -> ACM2D, EDDYX, ACM, ACMM, MATRIX,
// TRI) for one column (WRF loops over the rows j and the points its..ite of a row; every point is independent).
// Every statement follows the Fortran in order and parenthesisation (real*8: the literal constants are doubles).
// libm: the micromet loop of ACM2D is vectorised by gfortran (WST = UST*(...)**0.333333 with libmvec's 2-lane pow
// over its..ite: vm::pow with the point's i of the nx-point row); the other powers (WSS**0.33333, PHIM**(-0.25),
// (KARMAN/HOVL)**0.3333) are scalar libm calls; x**2 = x*x, UST**3 = (UST*UST)*UST, RI**4 = (RI*RI)*(RI*RI),
// (1 - ZF/PBL)**P with the constant P = 2.0 is folded to a product as gfortran does.
// MATRIX keeps WRF's operations and order but holds only the row of L it is working on (its subdiagonal L(J,J-1) for
// the later rows), instead of the KL x KL matrix: L(I,J) depends on L(I,J-1) and L(J,J-1) only and Ly = d uses row I
// after it is complete, so every value is the Fortran one.
// The model's levels k = 0 .. nz-1 are WRF's k = 1 .. kte; the arrays below use WRF's indices (ZF from 0).
#include <cmath>

#include "core/backend.hpp"
#include "core/field.hpp"

namespace cppwrf::phys::acm {

inline constexpr int kMaxLevels = 128;

// module_bl_acm.F parameters
inline constexpr real ric = real(0.25);     // critical Richardson number
inline constexpr real crankp = real(0.5);   // Crank-Nicolson parameter
inline constexpr real karman = real(0.4);   // ACMPBL's local KARMAN (passed to ACM2D), EDDYX's and ACM's

// MATRIX (bordered band diagonal solver of ACM2): A, B, C, E of length kl, D and X of nsp x kl (WRF indices 1 .. kl)
template <int K, int NSP>
CPPWRF_HD inline void matrix(const real* a, const real* b, const real* c, real (*d)[K + 1], const real* e,
                             real (*x)[K + 1], int kl) {
  real lsub[K + 1];   // L(J,J-1)
  real lrow[K + 1];   // L(I,1 .. I-1) of the current row I
  real uii[K + 1], uiip1[K + 1], ruii[K + 1];
  real y[NSP][K + 1];
  // Define Upper and Lower matrices (the first loop over I of WRF; UII(I) = B(I)-L(I,I-1)*E(I-1) of the second
  // loop needs L(I,I-1) only, and the forward substitution row I of L)
  uii[1] = b[1];
  ruii[1] = real(1.) / uii[1];
  for (int v = 0; v < NSP; ++v) y[v][1] = d[v][1];
  for (int i = 2; i <= kl; ++i) {
    lrow[1] = a[i] / b[1];
    uiip1[i - 1] = e[i - 1];
    if (i >= 3) {
      for (int jj = 2; jj <= i - 1; ++jj) {
        const real aij = (i == jj + 1) ? c[i] : real(0.);
        // L(I,J) = (AIJ-L(I,J-1)*E(J-1))/(B(J)-L(J,J-1)*E(J-1))
        lrow[jj] = (aij - lrow[jj - 1] * e[jj - 1]) / (b[jj] - lsub[jj] * e[jj - 1]);
      }
    }
    lsub[i] = lrow[i - 1];
    // Forward sub for Ly=d (row I)
    for (int v = 0; v < NSP; ++v) {
      real sum = d[v][i];
      for (int jj = 1; jj <= i - 1; ++jj) sum = sum - lrow[jj] * y[v][jj];
      y[v][i] = sum;
    }
  }
  for (int i = 2; i <= kl; ++i) {
    uii[i] = b[i] - lsub[i] * e[i - 1];
    ruii[i] = real(1.) / uii[i];
  }
  // Back sub for Ux=y
  for (int v = 0; v < NSP; ++v) x[v][kl] = y[v][kl] * ruii[kl];
  for (int i = kl - 1; i >= 1; --i)
    for (int v = 0; v < NSP; ++v) x[v][i] = (y[v][i] - uiip1[i] * x[v][i + 1]) * ruii[i];
}

// TRI (Thomas algorithm): L sub-, D main, U super-diagonal, B right-hand sides, X solutions
template <int K, int NSP>
CPPWRF_HD inline void tri(const real* l, const real* d, const real* u, real (*b)[K + 1], real (*x)[K + 1],
                          int kl) {
  real gam[K + 1];
  real bet = real(1.0) / d[1];
  for (int v = 0; v < NSP; ++v) x[v][1] = bet * b[v][1];
  for (int k = 2; k <= kl; ++k) {
    gam[k] = bet * u[k - 1];
    bet = real(1.0) / (d[k] - l[k] * gam[k]);
    for (int v = 0; v < NSP; ++v) x[v][k] = bet * (b[v][k] - l[k] * x[v][k - 1]);
  }
  for (int k = kl - 1; k >= 1; --k)
    for (int v = 0; v < NSP; ++v) x[v][k] = x[v][k] - gam[k + 1] * x[v][k + 1];
}

// the column geometry and scaling values ACM and ACMM share (ACM2D's locals)
template <int K>
struct Column {
  // K + 2: WRF reads ZA / THETAV at KMIX+1 and DZH at KCBL+1 (beyond kte only in columns WRF itself gets wrong)
  real zf[K + 2], za[K + 2], dzh[K + 2], dzhi[K + 2], dzfi[K + 2];
  real eddyz[K + 2], eddyzm[K + 2];
  real pbl, mol, ust, tst, qst, ustm;
  int klpbl, noconv;
};

// ACM / ACMM for one column: NSP species with the values vci(l, 1 .. kl) (updated in place), surface fluxes fs(l),
// eddy diffusivity eddy (EDDYZ or EDDYZM, reduced in the convective part as WRF does); acmm selects ACMM's order of
// the factors in BI(1)
template <int K, int NSP>
CPPWRF_HD inline void acm_mix(const Column<K>& c, real* eddy, real (*vci)[K + 1], const real* fs, real dtpbl, int kl,
                              bool acmm) {
  constexpr real xx = real(0.5);  // factor applied to the convective mixing time step
  const int klm = kl - 1;
  real dtlim = dtpbl;
  int kcbl = 1;
  real fsacm = real(0.0);
  real mbarks[K + 2], mdwn[K + 2];
  for (int k = 0; k <= K + 1; ++k) mbarks[k] = mdwn[k] = real(0.);
  if (c.noconv == 1) {
    kcbl = c.klpbl;
    // MBARKS is the upward mixing rate, MDWN the downward mixing rate (new couple ACM & EDDY)
    const real hovl = -c.pbl / c.mol;
    fsacm = real(1.) / (real(1.) + (libm::pow(karman / (hovl), real(0.3333))) / (real(0.72) * karman));
    const real meddy = eddy[1] * c.dzfi[1] / (c.pbl - c.zf[1]);
    const real mbar = meddy * fsacm;
    for (int k = 1; k <= kcbl - 1; ++k) eddy[k] = eddy[k] * (real(1.0) - fsacm);
    for (int k = 2; k <= kcbl; ++k) {
      mbarks[k] = mbar;
      mdwn[k] = mbar * (c.pbl - c.zf[k - 1]) * c.dzhi[k];
    }
    mbarks[1] = mbar;
    mbarks[kcbl] = mdwn[kcbl];
    mdwn[kcbl + 1] = real(0.0);
  }
  for (int k = 1; k <= klm; ++k) {
    const real ekz = eddy[k] * c.dzfi[k] * c.dzhi[k];
    dtlim = c_min(real(0.75) / ekz, dtlim);
  }
  if (c.noconv == 1) {
    const real rz = (c.zf[kcbl] - c.zf[1]) * c.dzhi[1];
    dtlim = c_min(xx / (mbarks[1] * rz), dtlim);
  }
  const int nlp = int(dtpbl / dtlim + real(1.0));
  const real dts = (dtpbl / real(nlp));
  real ai[K + 1], bi[K + 1], ci[K + 1], ei[K + 1], xplus[K + 1], xminus[K + 1];
  real di[NSP][K + 1], ui[NSP][K + 1];
  for (int nl = 1; nl <= nlp; ++nl) {
    // array elements that are independent of the species
    for (int k = 1; k <= kl; ++k) ai[k] = bi[k] = ci[k] = ei[k] = real(0.0);
    for (int k = 2; k <= kcbl; ++k) {
      ei[k - 1] = -crankp * mdwn[k] * dts * c.dzh[k] * c.dzhi[k - 1];
      bi[k] = real(1.0) + crankp * mdwn[k] * dts;
      ai[k] = -crankp * mbarks[k] * dts;
    }
    ei[1] = ei[1] - eddy[1] * crankp * c.dzhi[1] * c.dzfi[1] * dts;
    ai[2] = ai[2] - eddy[1] * crankp * c.dzhi[2] * c.dzfi[1] * dts;
    for (int k = kcbl + 1; k <= kl; ++k) bi[k] = real(1.0);
    for (int k = 2; k <= kl; ++k) {
      xplus[k] = eddy[k] * c.dzhi[k] * c.dzfi[k] * dts;
      xminus[k] = eddy[k - 1] * c.dzhi[k] * c.dzfi[k - 1] * dts;
      ci[k] = -xminus[k] * crankp;
      ei[k] = ei[k] - xplus[k] * crankp;
      bi[k] = bi[k] + xplus[k] * crankp + xminus[k] * crankp;
    }
    if (c.noconv == 1) {
      bi[1] = acmm ? real(1.0) + crankp * mbarks[1] * (c.pbl - c.zf[1]) * dts * c.dzhi[1] +
                         eddy[1] * c.dzhi[1] * c.dzfi[1] * crankp * dts
                   : real(1.0) + crankp * mbarks[1] * (c.pbl - c.zf[1]) * dts * c.dzhi[1] +
                         eddy[1] * crankp * c.dzhi[1] * c.dzfi[1] * dts;
    } else {
      bi[1] = acmm ? real(1.0) + eddy[1] * c.dzhi[1] * c.dzfi[1] * crankp * dts
                   : real(1.0) + eddy[1] * crankp * c.dzhi[1] * c.dzfi[1] * dts;
    }
    for (int k = 1; k <= kl; ++k)
      for (int l = 0; l < NSP; ++l) di[l][k] = real(0.0);
    // tendency of the CBL concentrations: semi-implicit solution
    for (int k = 2; k <= kcbl; ++k)
      for (int l = 0; l < NSP; ++l) {
        const real delc = dts * (mbarks[k] * vci[l][1] - mdwn[k] * vci[l][k] +
                                 c.dzh[k + 1] * c.dzhi[k] * mdwn[k + 1] * vci[l][k + 1]);
        di[l][k] = vci[l][k] + (real(1.0) - crankp) * delc;
      }
    for (int k = kcbl + 1; k <= kl; ++k)
      for (int l = 0; l < NSP; ++l) di[l][k] = vci[l][k];
    for (int k = 2; k <= kl; ++k) {
      if (k == kl) {
        for (int l = 0; l < NSP; ++l)
          di[l][k] = di[l][k] - (real(1.0) - crankp) * xminus[k] * (vci[l][k] - vci[l][k - 1]);
      } else {
        for (int l = 0; l < NSP; ++l)
          di[l][k] = di[l][k] + (real(1.0) - crankp) * xplus[k] * (vci[l][k + 1] - vci[l][k]) -
                     (real(1.0) - crankp) * xminus[k] * (vci[l][k] - vci[l][k - 1]);
      }
    }
    if (c.noconv == 1) {
      for (int l = 0; l < NSP; ++l)
        di[l][1] = vci[l][1] + (fs[l] - (real(1.0) - crankp) * (mbarks[1] * (c.pbl - c.zf[1]) * vci[l][1] -
                                                                 mdwn[2] * vci[l][2] * c.dzh[2])) *
                                   c.dzhi[1] * dts;
    } else {
      for (int l = 0; l < NSP; ++l) di[l][1] = vci[l][1] + fs[l] * c.dzhi[1] * dts;
    }
    for (int l = 0; l < NSP; ++l)
      di[l][1] = di[l][1] + (real(1.0) - crankp) * eddy[1] * c.dzhi[1] * c.dzfi[1] * dts * (vci[l][2] - vci[l][1]);
    if (c.noconv == 1)
      matrix<K, NSP>(ai, bi, ci, di, ei, ui, kl);
    else
      tri<K, NSP>(ci, bi, ei, di, ui, kl);
    // new values
    for (int k = 1; k <= kl; ++k)
      for (int l = 0; l < NSP; ++l) vci[l][k] = ui[l][k];
  }
}

struct AcmKernel {
  View3<const real> us, vs, theta, tt, qvs, qcs, qis, dzf, densx;  // u_phytmp, v_phytmp, th_phy, t_phy, qv, qc, qi,
                                                                   // dz8w, rho
  View2<const real> ust, hfx, qfx, wspd;
  View2<real> pbl, regime, rmol;
  View2<int> klpbl;
  View3<real> exch_h, exch_m, utnp, vtnp, ttnp, qvtnp, qctnp, qitnp;
  real dtpbl, ep1, g, rd, cpd;
  int xtime;  // itimestep
  int nz;
  int nx;     // the row's points its..ite (the vectorised micromet loop's lanes)

  template <int K>
  CPPWRF_HD void column(int i, int j) const {
    const int kte = nz;
    const real rdt = real(1.0) / dtpbl;
    using std::abs;
    using std::copysign;
    using std::sqrt;
    Column<K> c;
    real thetav[K + 2], rib[K + 2];
    for (int k = 0; k <= K + 1; ++k) c.zf[k] = c.za[k] = c.dzh[k] = c.dzhi[k] = c.dzfi[k] = thetav[k] = real(0.);
    const auto US = [&](int k) { return us(i, j, k - 1); };
    const auto VS = [&](int k) { return vs(i, j, k - 1); };

    // micromet scaling variables (the loop gfortran vectorises)
    const real qv1 = qvs(i, j, 0), dens1 = densx(i, j, 0);
    const real cpair = cpd * (real(1.0) + real(0.84) * qv1);  // J/(K KG)
    const real tmpfx = hfx(i, j) / (cpair * dens1);
    const real tmpvtcon = real(1.0) + ep1 * qv1;  // conversion factor for virtual temperature
    const real ws1 = sqrt(US(1) * US(1) + VS(1) * VS(1));  // level 1 wind speed
    c.ust = ust(i, j);
    c.tst = -tmpfx / c.ust;
    c.qst = -qfx(i, j) / (c.ust * dens1);
    c.ustm = c.ust * ws1 / wspd(i, j);
    const real thv1 = tmpvtcon * theta(i, j, 0);
    real tstv = c.tst * tmpvtcon + thv1 * ep1 * c.qst;
    if (abs(tstv) < real(1.0E-6)) tstv = copysign(real(1.0E-6), tstv);
    c.mol = thv1 * (c.ust * c.ust) / (karman * g * tstv);
    rmol(i, j) = real(1.0) / c.mol;
    real pblv = pbl(i, j);
    const real wst = c.ust * vm::pow(pblv / (karman * abs(c.mol)), real(0.333333), i, nx);

    // heights of the full and half levels above ground
    c.zf[0] = real(0.0);
    c.klpbl = 1;
    for (int k = 1; k <= kte; ++k) {
      c.zf[k] = dzf(i, j, k - 1) + c.zf[k - 1];
      c.za[k] = real(0.5) * (c.zf[k] + c.zf[k - 1]);
      c.dzh[k] = c.zf[k] - c.zf[k - 1];
      c.dzhi[k] = real(1.) / c.dzh[k];
    }
    for (int k = 1; k <= kte - 1; ++k) c.dzfi[k] = real(1.) / (c.za[k + 1] - c.za[k]);
    c.dzfi[kte] = c.dzfi[kte - 1];
    for (int k = 1; k <= kte; ++k) {
      const real tvcon = real(1.0) + ep1 * qvs(i, j, k - 1);
      thetav[k] = theta(i, j, k - 1) * tvcon;
    }

    // PBL height where the Richardson number = RIC (Holtslag et al. 1990)
    for (int k = 0; k <= K + 1; ++k) rib[k] = real(0.0);
    int ksrc = 1;
    for (int k = 1; k <= kte; ++k) {
      rib[k] = real(0.0);
      ksrc = k;
      if (c.zf[k] > real(30.0)) break;
    }
    real th1 = real(0.0), zh1 = real(0.0), uh1 = real(0.0), vh1 = real(0.0);
    for (int k = 1; k <= ksrc; ++k) {
      th1 = th1 + thetav[k];
      zh1 = zh1 + c.za[k];
      uh1 = uh1 + US(k);
      vh1 = vh1 + VS(k);
    }
    th1 = th1 / real(ksrc);
    zh1 = zh1 / real(ksrc);
    uh1 = uh1 / real(ksrc);
    vh1 = vh1 / real(ksrc);
    if (c.mol < real(0.0) && xtime > 1) {
      const real u3 = c.ust * c.ust * c.ust, w3 = wst * wst * wst;
      const real wss = libm::pow(u3 + real(0.6) * w3, real(0.33333));
      const real tconv = real(-8.5) * c.ust * tstv / wss;
      th1 = th1 + tconv;
    }
    int kmix = ksrc;
    for (int k = ksrc; k <= kte; ++k) {
      const real dtmp = thetav[k] - th1;
      if (dtmp < real(0.0)) kmix = k;
    }
    real zmix, umix, vmix;
    if (kmix > ksrc) {
      const real fintt = (th1 - thetav[kmix]) / (thetav[kmix + 1] - thetav[kmix]);
      zmix = fintt * (c.za[kmix + 1] - c.za[kmix]) + c.za[kmix];
      umix = fintt * (US(kmix + 1) - US(kmix)) + US(kmix);
      vmix = fintt * (VS(kmix + 1) - VS(kmix)) + VS(kmix);
    } else {
      zmix = zh1;
      umix = uh1;
      vmix = vh1;
    }
    int kpblh = 0;
    for (int k = kmix; k <= kte; ++k) {
      const real dtmp = thetav[k] - th1;
      const real tog = real(0.5) * (thetav[k] + th1) / g;
      const real du = US(k) - umix, dv = VS(k) - vmix;
      real wssq = du * du + dv * dv;
      if (kmix == ksrc) {
        wssq = US(k) * US(k) + VS(k) * VS(k);
        wssq = wssq + real(100.) * c.ust * c.ust;
      }
      wssq = c_max(wssq, real(0.1));
      rib[k] = abs(c.za[k] - zmix) * dtmp / (tog * wssq);
      if (rib[k] >= ric) {
        kpblh = k;
        break;
      }
    }
    if (kpblh == 0) {  // ' RIBX never exceeds RIC' (wrf_error_fatal)
      raise_fatal(kFatalAcmRibx);
      return;
    }
    if (kpblh > ksrc) {
      // interpolate between levels
      real fint = (ric - rib[kpblh - 1]) / (rib[kpblh] - rib[kpblh - 1]);
      int kpblht;
      if (fint > real(0.5)) {
        kpblht = kpblh;
        fint = fint - real(0.5);
      } else {
        kpblht = kpblh - 1;
        fint = fint + real(0.5);
      }
      pblv = fint * (c.zf[kpblht] - c.zf[kpblht - 1]) + c.zf[kpblht - 1];
      c.klpbl = kpblht;
    } else {
      c.klpbl = ksrc;
      pblv = c.za[ksrc];
    }
    c.pbl = pblv;
    pbl(i, j) = pblv;
    klpbl(i, j) = c.klpbl;

    // CBL check: convective vs non-convective cells
    c.noconv = 0;
    if (c.pbl / c.mol < real(-0.02) && c.klpbl > 3 && thetav[1] > thetav[2] && xtime > 1) {
      c.noconv = 1;
      regime(i, j) = real(4.0);  // free convective - ACM
    }

    eddyx<K>(c, thetav, cpair, i, j);

    // ACM: theta, qv, qc, qi (NSP = 4); ACMM: u, v (NSP = 2)
    {
      real vci[4][K + 1];
      for (int k = 1; k <= kte; ++k) {
        vci[0][k] = theta(i, j, k - 1);
        vci[1][k] = qvs(i, j, k - 1);
        vci[2][k] = qcs(i, j, k - 1);
        vci[3][k] = qis(i, j, k - 1);
      }
      const real fs[4] = {-c.ust * c.tst, -c.ust * c.qst, real(0.0), real(0.0)};
      acm_mix<K, 4>(c, c.eddyz, vci, fs, dtpbl, kte, false);
      for (int k = 1; k <= kte; ++k) {
        const int kk = k - 1;
        ttnp(i, j, kk) = real(0.) + (vci[0][k] - theta(i, j, kk)) * rdt;
        qvtnp(i, j, kk) = real(0.) + (vci[1][k] - qvs(i, j, kk)) * rdt;
        qctnp(i, j, kk) = real(0.) + (vci[2][k] - qcs(i, j, kk)) * rdt;
        qitnp(i, j, kk) = real(0.) + (vci[3][k] - qis(i, j, kk)) * rdt;
      }
    }
    {
      real vci[2][K + 1];
      for (int k = 1; k <= kte; ++k) {
        vci[0][k] = US(k);
        vci[1][k] = VS(k);
      }
      const real fm = -c.ustm * c.ustm;
      const real wspd1 = sqrt(US(1) * US(1) + VS(1) * VS(1)) + real(1.E-9);
      const real fs[2] = {fm * US(1) / wspd1, fm * VS(1) / wspd1};
      acm_mix<K, 2>(c, c.eddyzm, vci, fs, dtpbl, kte, true);
      for (int k = 1; k <= kte; ++k) {
        const int kk = k - 1;
        utnp(i, j, kk) = real(0.) + (vci[0][k] - US(k)) * rdt;
        vtnp(i, j, kk) = real(0.) + (vci[1][k] - VS(k)) * rdt;
      }
    }
    // exch_h / exch_m (for CCN activation): levels kts .. kte-1 (ACM's reduction of EDDYZ / EDDYZM included)
    for (int k = 1; k <= kte - 1; ++k) {
      exch_h(i, j, k - 1) = c.eddyz[k];
      exch_m(i, j, k - 1) = c.eddyzm[k];
    }
  }

  // EDDYX: Kz by boundary-layer scaling (Holtslag and Boville 1993) and the local Kz of the Richardson number and the
  // wind shear (Liu and Carroll 1996), the larger of the two
  template <int K>
  CPPWRF_HD void eddyx(Column<K>& c, const real* thetav, real cpair, int i, int j) const {
    using std::sqrt;
    constexpr real rv = real(461.5);
    constexpr real rlam = real(80.0);
    constexpr real gamh = real(16.0);
    constexpr real gamm = real(16.0);
    constexpr real betah = real(5.0);
    constexpr real edyz0 = real(0.01);  // new min Kz
    constexpr real pr = real(0.8);      // Prandtl number
    const int kl = nz, klm = nz - 1;
    for (int k = 1; k <= klm; ++k) {
      real edyz = real(0.0), edyzm = real(0.0);
      real zovl = real(0.0);
      const real dzf = c.za[k + 1] - c.za[k];
      const real kzo = edyz0;
      if (c.zf[k] < c.pbl) {
        zovl = c.zf[k] / c.mol;
        real wt, wm;
        if (zovl < real(0.0)) {
          if (c.zf[k] < real(0.1) * c.pbl) {
            const real phih = real(1.0) / sqrt(real(1.0) - gamh * zovl);
            const real phim = libm::pow(real(1.0) - gamm * zovl, real(-0.25));
            wt = c.ust / phih;
            wm = c.ust / phim;
          } else {
            const real zsol = real(0.1) * c.pbl / c.mol;
            const real phih = real(1.0) / sqrt(real(1.0) - gamh * zsol);
            const real phim = libm::pow(real(1.0) - gamm * zsol, real(-0.25));
            wt = c.ust / phih;
            wm = c.ust / phim;
          }
        } else if (zovl < real(1.0)) {
          const real phih = real(1.0) + betah * zovl;
          wt = c.ust / phih;
          wm = wt;
        } else {
          const real phih = betah + zovl;
          wt = c.ust / phih;
          wm = wt;
        }
        const real f = real(1.0) - c.zf[k] / c.pbl;
        const real zfunc = c.zf[k] * (f * f);
        edyz = karman * wt * zfunc;
        edyzm = karman * wm * zfunc;
      }
      const real du = us(i, j, k) - us(i, j, k - 1), dv = vs(i, j, k) - vs(i, j, k - 1);
      const real ss = (du * du + dv * dv) / (dzf * dzf) + real(1.0E-9);
      const real goth = real(2.0) * g / (thetav[k + 1] + thetav[k]);
      real ri = goth * (thetav[k + 1] - thetav[k]) / (dzf * ss);
      // adjustment to the vertical diffusion in moist air (imvdif = 1)
      if ((qcs(i, j, k - 1) + qis(i, j, k - 1)) > real(0.01E-3) || (qcs(i, j, k) + qis(i, j, k)) > real(0.01E-3)) {
        const real qmean = real(0.5) * (qvs(i, j, k - 1) + qvs(i, j, k));
        const real tmean = real(0.5) * (tt(i, j, k - 1) + tt(i, j, k));
        const real xlv = (real(2.501) - real(0.00237) * (tmean - real(273.15))) * real(1.E6);
        const real alph = xlv * qmean / rd / tmean;
        const real chi = xlv * xlv * qmean / cpair / rv / tmean / tmean;
        ri = (real(1.0) + alph) * (ri - g * g / ss / tmean / cpair * ((chi - alph) / (real(1.0) + chi)));
      }
      const real zk = real(0.4) * c.zf[k];
      const real sq = zk * rlam / (rlam + zk);
      const real sql = sq * sq;
      real ez, ezm;
      if (ri >= real(0.0)) {
        const real ri2 = ri * ri;
        const real fh = real(1.) / (real(1.) + real(10.) * ri + real(50.) * ri2 + real(5000.) * (ri2 * ri2)) +
                        real(0.0012);  // pleim5
        const real fm = pr * fh + real(0.00104);
        ez = kzo + sqrt(ss) * fh * sql;
        ezm = kzo + sqrt(ss) * fm * sql;
      } else {
        ez = kzo + sqrt(ss * (real(1.0) - real(25.0) * ri)) * sql;
        ezm = ez * pr;
      }
      if (edyz > ez) {
        ez = edyz;
        ezm = c_min(edyzm, edyz * real(0.8));
      }
      ez = c_min(real(1000.0), ez);
      ez = c_max(kzo, ez);
      ezm = c_min(real(1000.0), ezm);
      ezm = c_max(kzo, ezm);
      c.eddyz[k] = ez;
      c.eddyzm[k] = ezm;
    }
    c.eddyz[kl] = real(0.0);
    c.eddyzm[kl] = real(0.0);
  }
};

template <int K>
struct AcmRunner {
  AcmKernel k;
  CPPWRF_HD void operator()(int i, int j) const { k.template column<K>(i, j); }
};

}  // namespace cppwrf::phys::acm
