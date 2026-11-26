// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// ============================================================================
// CPPWRF Nesting Interpolation Kernels (SINT / SINTB / interp_fcn_sint)
// Ports WRF 4.8.0 share/sint.F and share/interp_fcn.F (interp_fcn_sint)
// Single-source C++20 / CUDA FP64 bit-exact numerics.
// ============================================================================

#include <cmath>
#include <algorithm>
#include <vector>
#include <cstdint>
#include "core/backend.hpp"
#include "core/field.hpp"

namespace cppwrf::nest {

// The arithmetic type of the SINT interpolation (the View kernels below and nest_bdy.hpp's slab kernels): the
// model's real, as WRF's REAL in share/sint.F (float in the FP32 builds, WRF real*4, CPU and CUDA alike; double in
// the FP64 builds).
using nreal = ::cppwrf::real;

#if defined(CPPWRF_USE_HIP) || defined(__HIP__)
// HIP: x86's maxsd / minsd as glibc's fmax / fmin use them (core/backend.hpp, fmax_x86)
CPPWRF_HD inline double sint_max(double a, double b) { return fmax_x86(a, b); }
CPPWRF_HD inline double sint_min(double a, double b) { return fmin_x86(a, b); }
CPPWRF_HD inline float sint_max(float a, float b) { return fmax_x86(a, b); }
CPPWRF_HD inline float sint_min(float a, float b) { return fmin_x86(a, b); }
#else
CPPWRF_HD inline double sint_max(double a, double b) { return fmax(a, b); }
CPPWRF_HD inline double sint_min(double a, double b) { return fmin(a, b); }
CPPWRF_HD inline float sint_max(float a, float b) { return fmaxf(a, b); }
CPPWRF_HD inline float sint_min(float a, float b) { return fminf(a, b); }
#endif

// 1D Donor-cell advection flux (the SINTB kernels: see smolarkiewicz_1d)
template <class T>
CPPWRF_HD inline T donor(T y1, T y2, T a) {
  return (a >= T(0.0)) ? (y1 * a) : (y2 * a);
}

// share/sint.F:37 DONOR(Y1,Y2,A)=(Y1*AMAX1(0.,SIGN(1.,A))-Y2*AMIN1(0.,SIGN(1.,A)))*A as written: the products by 1
// and 0 keep the signs of zero (Y1 = -0, Y2 < 0, A > 0: +0, where Y1*A is -0)
template <class T>
CPPWRF_HD inline T sint_donor(T y1, T y2, T a) {
  const T s = std::copysign(T(1.0), a);
  const T s1 = s > T(0.0) ? s : T(0.0);  // AMAX1(0., SIGN(1., A))
  const T s2 = s < T(0.0) ? s : T(0.0);  // AMIN1(0., SIGN(1., A))
  return (y1 * s1 - y2 * s2) * a;
}

// gfortran's MAX / MIN of share/sint.F (MAX_EXPR <a, b> / MIN_EXPR <a, b>, maxsd / minsd): b when a and b compare
// equal (+0 and -0)
template <class T>
CPPWRF_HD inline T sint_gmax(T a, T b) { return a > b ? a : b; }
template <class T>
CPPWRF_HD inline T sint_gmin(T a, T b) { return a < b ? a : b; }

// 4th-order Crowley / Smolarkiewicz polynomial advective flux
template <class T>
CPPWRF_HD inline T tr4(T ym1, T y0, T yp1, T yp2, T a) {
  const T one12 = T(1.0) / T(12.0);
  const T one24 = T(1.0) / T(24.0);
  const T a2 = a * a;
  const T a3 = a2 * a;
  const T a4 = a3 * a;
  return a * one12 * (T(7.0) * (yp1 + y0) - (yp2 + ym1))
       - a2 * one24 * (T(15.0) * (yp1 - y0) - (yp2 - ym1))
       - a3 * one12 * ((yp1 + y0) - (yp2 + ym1))
       + a4 * one24 * (T(3.0) * (yp1 - y0) - (yp2 - ym1));
}

template <class T>
CPPWRF_HD inline T pp(T x) { return x > T(0.0) ? x : T(0.0); }
template <class T>
CPPWRF_HD inline T pn(T x) { return x < T(0.0) ? x : T(0.0); }

// 1D non-oscillatory MPDATA/FCT limiting step, one sweep of share/sint.F's SINT (sint.F:81-130 in x, 144-191 in y).
// The signs of zero are SINT's as WRF's gfortran -O2 build computes them: DONOR as written (sint_donor); MXM =
// AMAX1(Y(-1),Y(0),Y(1),W) evaluated as MAX(MAX(Y(-1),Y(0)), MAX(Y(1),W)) and MN likewise with MIN (the reassociated
// MAX_EXPR / MIN_EXPR tree of its optimized code, both sweeps, REAL*8 and REAL*4); PP(X) = MAX_EXPR <X, 0.>,
// PN(X) = MIN_EXPR <X, 0.>, AMIN1(1.,OV) = MIN_EXPR <OV, 1.>. Checked bit for bit, signed zeros included, against
// WRF's SINT on fields of +0, -0 and small values (tests/nest/test_nest_differential.cpp).
// kSintb: the kernel of the SINTB paths (share/sint.F SINTB: lateral boundaries, nest_bdy.hpp), unchanged: WRF's
// SINTB is vectorized and its MAX / MIN trees differ between the vector body and the scalar remainder.
template <class T, bool kSintb = false>
CPPWRF_HD inline T smolarkiewicz_1d(T ym2, T ym1, T y0, T yp1, T yp2, T a) {
  const T ep = T(1.0e-10);
  if constexpr (kSintb) {
    const T fl0 = donor(ym1, y0, a);
    const T fl1 = donor(y0, yp1, a);
    const T w = y0 - (fl1 - fl0);
    const T mxm = sint_max(sint_max(ym1, y0), sint_max(yp1, w));
    const T mn  = sint_min(sint_min(ym1, y0), sint_min(yp1, w));

    T f0 = tr4(ym2, ym1, y0, yp1, a) - fl0;
    T f1 = tr4(ym1, y0, yp1, yp2, a) - fl1;

    const T pp_f0 = pp(f0);
    const T pn_f0 = pn(f0);
    const T pp_f1 = pp(f1);
    const T pn_f1 = pn(f1);

    const T ov = (mxm - w) / (-pn_f1 + pp_f0 + ep);
    const T un = (w - mn) / (pp_f1 - pn_f0 + ep);

    f0 = pp_f0 * sint_min(T(1.0), ov) + pn_f0 * sint_min(T(1.0), un);
    f1 = pp_f1 * sint_min(T(1.0), un) + pn_f1 * sint_min(T(1.0), ov);

    return w - (f1 - f0);
  } else {
    const T fl0 = sint_donor(ym1, y0, a);                                // sint.F:81-82 (y sweep: 144-145)
    const T fl1 = sint_donor(y0, yp1, a);
    const T w = y0 - (fl1 - fl0);                                        // sint.F:86
    const T mxm = sint_gmax(sint_gmax(ym1, y0), sint_gmax(yp1, w));      // sint.F:90-92 (153)
    const T mn = sint_gmin(sint_gmin(ym1, y0), sint_gmin(yp1, w));       // sint.F:93 (154)
    T f0 = tr4(ym2, ym1, y0, yp1, a) - fl0;                              // sint.F:98-109
    T f1 = tr4(ym1, y0, yp1, yp2, a) - fl1;
    const T pp_f0 = pp(f0), pn_f0 = pn(f0), pp_f1 = pp(f1), pn_f1 = pn(f1);
    const T ov = (mxm - w) / (-pn_f1 + pp_f0 + ep);                      // sint.F:114-117
    const T un = (w - mn) / (pp_f1 - pn_f0 + ep);
    const T mo = sint_gmin(ov, T(1.0)), mu = sint_gmin(un, T(1.0));      // AMIN1(1.,OV), AMIN1(1.,UN)
    f0 = pp_f0 * mo + pn_f0 * mu;                                        // sint.F:122-125
    f1 = pp_f1 * mu + pn_f1 * mo;
    return w - (f1 - f0);                                                // sint.F:130
  }
}

// 2D SINT point evaluation given a 5x5 stencil of coarse values around (ci, cj):
// c_patch[j_idx][i_idx] where j_idx in 0..4 (j-2..j+2) and i_idx in 0..4 (i-2..i+2)
template <class T, bool kSintb = false>
CPPWRF_HD inline T sint_point_2d(const T c_patch[5][5], T ax, T ay) {
  T z[5];
  for (int j = 0; j < 5; ++j) {
    z[j] = smolarkiewicz_1d<T, kSintb>(c_patch[j][0], c_patch[j][1], c_patch[j][2], c_patch[j][3], c_patch[j][4], ax);
  }
  return smolarkiewicz_1d<T, kSintb>(z[0], z[1], z[2], z[3], z[4], ay);
}

// Compute subgrid offset coordinates XIG and XJG for a given ratio and subgrid index
// rr: refinement ratio (e.g. 3 or 5)
// ip, jp: 0-based subgrid position within the coarse cell (0 <= ip < rr, 0 <= jp < rr)
template <class T>
CPPWRF_HD inline void compute_sint_subgrid_offsets(
    int rr, bool xstag, bool ystag, int ip, int jp, T& ax, T& ay) {
  T rioff = T(0.0);
  T rjoff = T(0.0);
  if (xstag && (rr % 2 == 0)) rioff = T(1.0);
  if (ystag && (rr % 2 == 0)) rjoff = T(1.0);

  // In WRF sint.F:
  // J is x-coordinate subgrid index (1-based: 1..rr), I is y-coordinate subgrid index (1-based: 1..rr)
  // XIG = (rr - 1 - rioff)/(2*rr) - (J - 1)/rr
  // XJG = (rr - 1 - rjoff)/(2*rr) - (I - 1)/rr
  // Here 0-based: J - 1 = ip, I - 1 = jp
  ax = (static_cast<T>(rr) - T(1.0) - rioff) / (T(2.0) * rr) - static_cast<T>(ip) / rr;
  ay = (static_cast<T>(rr) - T(1.0) - rjoff) / (T(2.0) * rr) - static_cast<T>(jp) / rr;
}

// ============================================================================
// Direct 3D array SINT implementation matching WRF share/sint.F SINT subroutine
// XF layout: (i, j, nf) with i fastest, then j, then nf (Fortran column-major: XF(ims:ime, jms:jme, nf))
// ims, ime, jms, jme: 1-based Fortran memory limits
// its, ite, jts, jte: 1-based tile computation range
// icmask: bool mask (ims:ime, jms:jme)
// ============================================================================
inline void sint_fortran_layout(
    double* xf,
    int ims, int ime, int jms, int jme,
    const int* icmask,
    int its, int ite, int jts, int jte,
    int nf, bool xstag, bool ystag) {

  const int ni_tot = ime - ims + 1;
  const int nj_tot = jme - jms + 1;
  auto idx = [&](int i, int j, int iim) -> std::size_t {
    return static_cast<std::size_t>((i - ims) + (j - jms) * ni_tot + (iim - 1) * ni_tot * nj_tot);
  };
  auto mask_idx = [&](int i, int j) -> std::size_t {
    return static_cast<std::size_t>((i - ims) + (j - jms) * ni_tot);
  };

  const int rr = static_cast<int>(std::round(std::sqrt(static_cast<double>(nf))));
  double rioff = 0.0, rjoff = 0.0;
  if (xstag && (rr % 2 == 0)) rioff = 1.0;
  if (ystag && (rr % 2 == 0)) rjoff = 1.0;

  std::vector<double> xig(nf + 1), xjg(nf + 1);
  for (int i = 1; i <= rr; ++i) {
    for (int j = 1; j <= rr; ++j) {
      int m = j + (i - 1) * rr;
      xig[m] = (static_cast<double>(rr) - 1.0 - rioff) / (2.0 * rr) - static_cast<double>(j - 1) / rr;
      xjg[m] = (static_cast<double>(rr) - 1.0 - rjoff) / (2.0 * rr) - static_cast<double>(i - 1) / rr;
    }
  }

  const int ni_sub = ite - its + 1;
  const int nj_sub = jte - jts + 1;
  std::vector<double> z(static_cast<std::size_t>(ni_sub * 5));
  std::vector<double> out_sub(static_cast<std::size_t>(ni_sub * nj_sub));
  auto z_idx = [&](int ii, int j_rel) -> std::size_t {
    return static_cast<std::size_t>((ii - its) + (j_rel + 2) * ni_sub);
  };
  auto out_idx = [&](int ii, int jj) -> std::size_t {
    return static_cast<std::size_t>((ii - its) + (jj - jts) * ni_sub);
  };

  for (int iim = 1; iim <= nf; ++iim) {
    const double ax = xig[iim];
    const double ay = xjg[iim];

    for (int jj = jts; jj <= jte; ++jj) {
      // Step 1: 1D sweep in x for each j in -2..2
      for (int j_rel = -2; j_rel <= 2; ++j_rel) {
        for (int ii = its; ii <= ite; ++ii) {
          if (icmask[mask_idx(ii, jj)]) {
            double ym2 = xf[idx(ii - 2, jj + j_rel, iim)];
            double ym1 = xf[idx(ii - 1, jj + j_rel, iim)];
            double y0  = xf[idx(ii,     jj + j_rel, iim)];
            double yp1 = xf[idx(ii + 1, jj + j_rel, iim)];
            double yp2 = xf[idx(ii + 2, jj + j_rel, iim)];
            z[z_idx(ii, j_rel)] = smolarkiewicz_1d(ym2, ym1, y0, yp1, yp2, ax);
          }
        }
      }

      // Step 2: 1D sweep in y along Z
      for (int ii = its; ii <= ite; ++ii) {
        if (icmask[mask_idx(ii, jj)]) {
          double zm2 = z[z_idx(ii, -2)];
          double zm1 = z[z_idx(ii, -1)];
          double z0  = z[z_idx(ii,  0)];
          double zp1 = z[z_idx(ii,  1)];
          double zp2 = z[z_idx(ii,  2)];
          out_sub[out_idx(ii, jj)] = smolarkiewicz_1d(zm2, zm1, z0, zp1, zp2, ay);
        }
      }
    }

    // Step 3: Write back to xf only after all (jj, ii) sweeps are complete!
    for (int jj = jts; jj <= jte; ++jj) {
      for (int ii = its; ii <= ite; ++ii) {
        if (icmask[mask_idx(ii, jj)]) {
          xf[idx(ii, jj, iim)] = out_sub[out_idx(ii, jj)];
        }
      }
    }
  }
}

// ============================================================================
// SINTB implementation matching WRF share/sint.F SINTB subroutine
// Out-of-place: reads from xf1, writes to xf. Ignores mask.
// ============================================================================
inline void sintb_fortran_layout(
    const double* xf1, double* xf,
    int ims, int ime, int jms, int jme,
    const int* /*icmask*/,
    int its, int ite, int jts, int jte,
    int nf, bool xstag, bool ystag) {

  const int ni_tot = ime - ims + 1;
  const int nj_tot = jme - jms + 1;
  auto idx = [&](int i, int j, int iim) -> std::size_t {
    return static_cast<std::size_t>((i - ims) + (j - jms) * ni_tot + (iim - 1) * ni_tot * nj_tot);
  };

  const int rr = static_cast<int>(std::round(std::sqrt(static_cast<double>(nf))));
  double rioff = 0.0, rjoff = 0.0;
  if (xstag && (rr % 2 == 0)) rioff = 1.0;
  if (ystag && (rr % 2 == 0)) rjoff = 1.0;

  std::vector<double> xig(nf + 1), xjg(nf + 1);
  for (int i = 1; i <= rr; ++i) {
    for (int j = 1; j <= rr; ++j) {
      int m = j + (i - 1) * rr;
      xig[m] = (static_cast<double>(rr) - 1.0 - rioff) / (2.0 * rr) - static_cast<double>(j - 1) / rr;
      xjg[m] = (static_cast<double>(rr) - 1.0 - rjoff) / (2.0 * rr) - static_cast<double>(i - 1) / rr;
    }
  }

  const int ni_sub = ite - its + 1;
  std::vector<double> z(static_cast<std::size_t>(ni_sub * 5));
  auto z_idx = [&](int ii, int j_rel) -> std::size_t {
    return static_cast<std::size_t>((ii - its) + (j_rel + 2) * ni_sub);
  };

  for (int iim = 1; iim <= nf; ++iim) {
    const double ax = xig[iim];
    const double ay = xjg[iim];

    for (int jj = jts; jj <= jte; ++jj) {
      for (int j_rel = -2; j_rel <= 2; ++j_rel) {
        for (int ii = its; ii <= ite; ++ii) {
          double ym2 = xf1[idx(ii - 2, jj + j_rel, iim)];
          double ym1 = xf1[idx(ii - 1, jj + j_rel, iim)];
          double y0  = xf1[idx(ii,     jj + j_rel, iim)];
          double yp1 = xf1[idx(ii + 1, jj + j_rel, iim)];
          double yp2 = xf1[idx(ii + 2, jj + j_rel, iim)];
          z[z_idx(ii, j_rel)] = smolarkiewicz_1d<double, true>(ym2, ym1, y0, yp1, yp2, ax);
        }
      }

      for (int ii = its; ii <= ite; ++ii) {
        double zm2 = z[z_idx(ii, -2)];
        double zm1 = z[z_idx(ii, -1)];
        double z0  = z[z_idx(ii,  0)];
        double zp1 = z[z_idx(ii,  1)];
        double zp2 = z[z_idx(ii,  2)];
        xf[idx(ii, jj, iim)] = smolarkiewicz_1d<double, true>(zm2, zm1, z0, zp1, zp2, ay);
      }
    }
  }
}

// ============================================================================
// Full 3D interp_fcn_sint implementation
// Interpolates coarse field cfld into fine field nfld using SINT method
// cfld dimensions: (cims:cime, ckms:ckme, cjms:cjme) with i fastest, k middle, j slowest
// nfld dimensions: (nims:nime, nkms:nkme, njms:njme)
// imask: fine grid mask (nims:nime, njms:njme) (1 = active point to interpolate)
// ipos, jpos: 1-based start coordinates of child domain inside coarse domain
// nri, nrj: refinement ratios in x, y
// ============================================================================
inline void interp_fcn_sint(
    const double* cfld,
    int cids, int cide, int ckds, int ckde, int cjds, int cjde,
    int cims, int cime, int ckms, int ckme, int cjms, int cjme,
    int cits, int cite, int ckts, int ckte, int cjts, int cjte,
    double* nfld,
    int nids, int nide, int nkds, int nkde, int njds, int njde,
    int nims, int nime, int nkms, int nkme, int njms, int njme,
    int nits, int nite, int nkts, int nkte, int njts, int njte,
    int /*shw*/,
    const int* imask,
    bool xstag, bool ystag,
    int ipos, int jpos,
    int nri, int nrj) {

  const int c_ni = cime - cims + 1;
  const int c_nk = ckme - ckms + 1;
  auto c_idx = [&](int i, int k, int j) -> std::size_t {
    return static_cast<std::size_t>((i - cims) + (k - ckms) * c_ni + (j - cjms) * c_ni * c_nk);
  };

  const int n_ni = nime - nims + 1;
  const int n_nk = nkme - nkms + 1;
  auto n_idx = [&](int i, int k, int j) -> std::size_t {
    return static_cast<std::size_t>((i - nims) + (k - nkms) * n_ni + (j - njms) * n_ni * n_nk);
  };
  auto imask_idx = [&](int i, int j) -> std::size_t {
    return static_cast<std::size_t>((i - nims) + (j - njms) * n_ni);
  };

  int ioff = 0, joff = 0;
  int nioff = 0, njoff = 0;
  if (xstag) {
    ioff = (nri - 1) / 2;
    nioff = nri;
  }
  if (ystag) {
    joff = (nrj - 1) / 2;
    njoff = nrj;
  }

  const int nrio2 = nri / 2;
  const int nrjo2 = nrj / 2;
  const int nfx = nri * nrj;

  const int c_nj = cjme - cjms + 1;
  std::vector<double> psca(static_cast<std::size_t>(c_ni * c_nj * nfx));
  std::vector<int> icmask(static_cast<std::size_t>(c_ni * c_nj), 0);
  auto psca_idx = [&](int i, int j, int nf_idx) -> std::size_t {
    return static_cast<std::size_t>((i - cims) + (j - cjms) * c_ni + (nf_idx - 1) * c_ni * c_nj);
  };
  auto icmask_idx = [&](int i, int j) -> std::size_t {
    return static_cast<std::size_t>((i - cims) + (j - cjms) * c_ni);
  };

  for (int k = ckts; k <= ckte; ++k) {
    std::fill(icmask.begin(), icmask.end(), 0);

    for (int nf_idx = 1; nf_idx <= nfx; ++nf_idx) {
      for (int j = cjms; j <= cjme; ++j) {
        int nj = (j - jpos) * nrj + (nrjo2 + 1);
        for (int i = cims; i <= cime; ++i) {
          int ni = (i - ipos) * nri + (nrio2 + 1);
          if (ni >= nits - nioff - nrio2 && ni <= nite + nioff + nrio2 &&
              nj >= njts - njoff - nrjo2 && nj <= njte + njoff + nrjo2) {
            if (ni >= nims && ni <= nime && nj >= njms && nj <= njme) {
              if (imask[imask_idx(ni, nj)] == 1) {
                icmask[icmask_idx(i, j)] = 1;
              }
            }
            if (ni - nioff >= nims && ni <= nime && nj - njoff >= njms && nj <= njme) {
              if (ni >= nits - nioff && nj >= njts - njoff) {
                if (imask[imask_idx(ni - nioff, nj - njoff)] == 1) {
                  icmask[icmask_idx(i, j)] = 1;
                }
              }
            }
          }
          psca[psca_idx(i, j, nf_idx)] = cfld[c_idx(i, k, j)];
        }
      }
    }

    sint_fortran_layout(
        psca.data(),
        cims, cime, cjms, cjme,
        icmask.data(),
        cits - 1, cite + 1, cjts - 1, cjte + 1,
        nfx, xstag, ystag);

    for (int nj = njts; nj <= njte + joff; ++nj) {
      int cj = jpos + (nj - 1) / nrj;
      int jp = (nj - 1) % nrj;
      for (int ni = nits; ni <= nite + ioff; ++ni) {
        int ci = ipos + (ni - 1) / nri;
        int ip = (ni - 1) % nri;
        if ((ni - ioff >= nits) && (nj - joff >= njts)) {
          if (imask[imask_idx(ni, nj)] == 1 || imask[imask_idx(ni - ioff, nj - joff)] == 1) {
            nfld[n_idx(ni - ioff, k, nj - joff)] = psca[psca_idx(ci, cj, ip + 1 + jp * nri)];
          }
        }
      }
    }
  }
}

// ============================================================================
// Direct Single-Source Device Kernel for 3D SINT Interpolation (raw arrays)
// Computes fine grid points (ni, nj, k) independently directly in registers!
// ============================================================================
struct KSint3D {
  const double* cfld;
  double* nfld;
  const int* imask;
  int c_ni, c_nk, c_nj;
  int cims, ckms, cjms;
  int n_ni, n_nk, n_nj;
  int nims, nkms, njms;
  int nits, nite, njts, njte;
  int ipos, jpos, nri, nrj;
  int ioff, joff;
  bool xstag, ystag;

  CPPWRF_HD void operator()(int ni_raw, int nj_raw, int k) const {
    const int ni = ni_raw - ioff;
    const int nj = nj_raw - joff;
    if (ni < nits || nj < njts) return;

    const std::size_t m_idx1 = static_cast<std::size_t>((ni_raw - nims) + (nj_raw - njms) * n_ni);
    const std::size_t m_idx2 = static_cast<std::size_t>((ni - nims) + (nj - njms) * n_ni);
    if ((imask[m_idx1] != 1) && (imask[m_idx2] != 1)) return;

    const int ci = ipos + (ni_raw - 1) / nri;
    const int cj = jpos + (nj_raw - 1) / nrj;
    const int ip = (ni_raw - 1) % nri;
    const int jp = (nj_raw - 1) % nrj;

    double ax, ay;
    compute_sint_subgrid_offsets(nri, xstag, ystag, ip, jp, ax, ay);

    const int nrio2 = nri / 2;
    const int nrjo2 = nrj / 2;
    const int nioff = xstag ? nri : 0;
    const int njoff = ystag ? nrj : 0;
    const int nime = nims + n_ni - 1;
    const int njme = njms + n_nj - 1;

    const int ni_c = (ci - ipos) * nri + (nrio2 + 1);
    const int nj_c = (cj - jpos) * nrj + (nrjo2 + 1);

    bool icmask_active = false;
    if (ni_c >= nits - nioff - nrio2 && ni_c <= nite + nioff + nrio2 &&
        nj_c >= njts - njoff - nrjo2 && nj_c <= njte + njoff + nrjo2) {
      if (ni_c >= nims && ni_c <= nime && nj_c >= njms && nj_c <= njme) {
        if (imask[static_cast<std::size_t>((ni_c - nims) + (nj_c - njms) * n_ni)] == 1) {
          icmask_active = true;
        }
      }
      if (ni_c - nioff >= nims && ni_c <= nime && nj_c - njoff >= njms && nj_c <= njme) {
        if (ni_c >= nits - nioff && nj_c >= njts - njoff) {
          if (imask[static_cast<std::size_t>((ni_c - nioff - nims) + (nj_c - njoff - njms) * n_ni)] == 1) {
            icmask_active = true;
          }
        }
      }
    }

    double val;
    if (!icmask_active) {
      std::size_t c_idx = static_cast<std::size_t>(
          (ci - cims) + (k - ckms) * c_ni + (cj - cjms) * c_ni * c_nk);
      val = cfld[c_idx];
    } else {
      double c_patch[5][5];
      for (int dj = -2; dj <= 2; ++dj) {
        for (int di = -2; di <= 2; ++di) {
          std::size_t c_idx = static_cast<std::size_t>(
              (ci + di - cims) + (k - ckms) * c_ni + (cj + dj - cjms) * c_ni * c_nk);
          c_patch[dj + 2][di + 2] = cfld[c_idx];
        }
      }
      val = sint_point_2d(c_patch, ax, ay);
    }

    std::size_t out_idx = static_cast<std::size_t>(
        (ni - nims) + (k - nkms) * n_ni + (nj - njms) * n_ni * n_nk);
    nfld[out_idx] = val;
  }
};

// ============================================================================
// Single-Source View-based SINT Interpolation for CPPWRF Field3 / BaseField3
// and Field2 / BaseField2 (0-based indices i in 0..n_nx-1, j in 0..n_ny-1)
// ============================================================================
template <class CView, class NView>
struct KSintView3D {
  CView cfld;
  NView nfld;
  int ipos, jpos;
  int nri, nrj;
  bool xstag, ystag;

  CPPWRF_HD void operator()(int ni0, int nj0, int k) const {
    const int ioff = xstag ? (nri - 1) / 2 : 0;
    const int joff = ystag ? (nrj - 1) / 2 : 0;
    const int ni_raw = (ni0 + 1) + ioff;
    const int nj_raw = (nj0 + 1) + joff;
    const int ci0 = (ipos - 1) + (ni_raw - 1) / nri;
    const int cj0 = (jpos - 1) + (nj_raw - 1) / nrj;
    const int ip = (ni_raw - 1) % nri;
    const int jp = (nj_raw - 1) % nrj;

    nreal ax, ay;
    compute_sint_subgrid_offsets(nri, xstag, ystag, ip, jp, ax, ay);

    nreal c_patch[5][5];
    for (int dj = -2; dj <= 2; ++dj) {
      for (int di = -2; di <= 2; ++di) {
        c_patch[dj + 2][di + 2] = static_cast<nreal>(cfld(ci0 + di, cj0 + dj, k));
      }
    }
    nfld(ni0, nj0, k) = static_cast<real>(sint_point_2d(c_patch, ax, ay));
  }
};

template <class CView, class NView>
struct KSintView2D {
  CView cfld;
  NView nfld;
  int ipos, jpos;
  int nri, nrj;
  bool xstag, ystag;

  CPPWRF_HD void operator()(int ni0, int nj0, int /*k*/) const {
    const int ioff = xstag ? (nri - 1) / 2 : 0;
    const int joff = ystag ? (nrj - 1) / 2 : 0;
    const int ni_raw = (ni0 + 1) + ioff;
    const int nj_raw = (nj0 + 1) + joff;
    const int ci0 = (ipos - 1) + (ni_raw - 1) / nri;
    const int cj0 = (jpos - 1) + (nj_raw - 1) / nrj;
    const int ip = (ni_raw - 1) % nri;
    const int jp = (nj_raw - 1) % nrj;

    nreal ax, ay;
    compute_sint_subgrid_offsets(nri, xstag, ystag, ip, jp, ax, ay);

    nreal c_patch[5][5];
    for (int dj = -2; dj <= 2; ++dj) {
      for (int di = -2; di <= 2; ++di) {
        c_patch[dj + 2][di + 2] = static_cast<nreal>(cfld(ci0 + di, cj0 + dj));
      }
    }
    nfld(ni0, nj0) = static_cast<real>(sint_point_2d(c_patch, ax, ay));
  }
};

}  // namespace cppwrf::nest
