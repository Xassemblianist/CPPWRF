// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Grell-3D ensemble cumulus scheme (cu_physics = 5, G3SCHEME): WRF 4.8.0 phys/module_cu_g3.F. G3DRV's work for one
// column (g3_column: its inputs gathered by g3_gather, which reads the 3 x 3 neighbours as G3DRV does), CUP_enss_3d
// and the cup_* routines it calls, and CONV_GRELL_SPREAD3D for one column (g3_spread). Every statement follows the
// Fortran in order and parenthesisation (x**2 = x*x, x**3 = x*x*x and x**4 = (x*x)*(x*x) as gfortran's powi); the
// module has no vectorised libm call (nm module_cu_g3.o: exp, log, atan2 scalar). The arrays keep Fortran's 1-based
// level index (1 .. nz, sized KP); the routines' (its:ite) vectors are the column's scalars. A: the array type
// (R* on the host, ColArray<R> in the GPU's column work buffer).
// Constants of G3DRV / CUP_enss_3d for WRF's defaults (rejected otherwise): maxiens = 1, maxens = 3, maxens2 = 3,
// maxens3 = 16, ensdim = 144, ens4 = 9 (ens4_spread = 3), iens = 1; the cap-suppress option (WRF_DFI_RADAR) is not
// compiled in WRF's standard build. irandom is 0 in cup_forcing_ens_3d, so no random number is drawn.
#include <cmath>
#include <cstddef>

#include "core/backend.hpp"
#include "core/column_work.hpp"

namespace cppwrf::phys::cu_g3 {

inline constexpr int kMaxLevels = 100;
inline constexpr int KP = kMaxLevels + 3;  // 1-based levels (index 0 unused, the top + 2 for kbcon up to kbmax + 2)
inline constexpr int MAXENS = 3, MAXENS2 = 3, MAXENS3 = 16, ENSDIM = 144, ENS4 = 9;

// the column's arrays (CUP_enss_3d's (i,k) locals, G3DRV's per-column arrays and cup_axx's locals)
#define CPPWRF_G3_ARRAYS(X)                                                                                        \
  X(R, t2d, ::cppwrf::phys::cu_g3::KP) X(R, q2d, ::cppwrf::phys::cu_g3::KP) X(R, po, ::cppwrf::phys::cu_g3::KP) X(R, us, ::cppwrf::phys::cu_g3::KP) X(R, vs, ::cppwrf::phys::cu_g3::KP) X(R, tn, ::cppwrf::phys::cu_g3::KP) X(R, qo, ::cppwrf::phys::cu_g3::KP) X(R, tshall, ::cppwrf::phys::cu_g3::KP)   \
  X(R, qshall, ::cppwrf::phys::cu_g3::KP) X(R, dhdt, ::cppwrf::phys::cu_g3::KP) X(R, outt, ::cppwrf::phys::cu_g3::KP) X(R, outq, ::cppwrf::phys::cu_g3::KP) X(R, outqc, ::cppwrf::phys::cu_g3::KP) X(R, outts, ::cppwrf::phys::cu_g3::KP) X(R, outqs, ::cppwrf::phys::cu_g3::KP)   \
  X(R, subt, ::cppwrf::phys::cu_g3::KP) X(R, subq, ::cppwrf::phys::cu_g3::KP) X(R, subm, ::cppwrf::phys::cu_g3::KP) X(R, cupclw, ::cppwrf::phys::cu_g3::KP)                                                    \
  X(R, omeg, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::ENS4) X(R, tx, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::ENS4) X(R, qx, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::ENS4)                                                    \
  X(R, he3, ::cppwrf::phys::cu_g3::KP) X(R, hes3, ::cppwrf::phys::cu_g3::KP) X(R, qes3, ::cppwrf::phys::cu_g3::KP) X(R, z3, ::cppwrf::phys::cu_g3::KP) X(R, zdo3, ::cppwrf::phys::cu_g3::KP) X(R, zu3_0, ::cppwrf::phys::cu_g3::KP) X(R, hc3_0, ::cppwrf::phys::cu_g3::KP)          \
  X(R, dby3_0, ::cppwrf::phys::cu_g3::KP) X(R, qes3_cup, ::cppwrf::phys::cu_g3::KP) X(R, q3_cup, ::cppwrf::phys::cu_g3::KP) X(R, he3_cup, ::cppwrf::phys::cu_g3::KP) X(R, hes3_cup, ::cppwrf::phys::cu_g3::KP) X(R, z3_cup, ::cppwrf::phys::cu_g3::KP)       \
  X(R, gamma3_cup, ::cppwrf::phys::cu_g3::KP) X(R, t3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xhe3, ::cppwrf::phys::cu_g3::KP) X(R, xhes3, ::cppwrf::phys::cu_g3::KP) X(R, xqes3, ::cppwrf::phys::cu_g3::KP) X(R, xz3, ::cppwrf::phys::cu_g3::KP) X(R, xt3, ::cppwrf::phys::cu_g3::KP) \
  X(R, xq3, ::cppwrf::phys::cu_g3::KP) X(R, xqes3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xq3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xhe3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xhes3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xz3_cup, ::cppwrf::phys::cu_g3::KP)     \
  X(R, xt3_cup, ::cppwrf::phys::cu_g3::KP) X(R, xdby3, ::cppwrf::phys::cu_g3::KP) X(R, xhc3, ::cppwrf::phys::cu_g3::KP) X(R, xzu3, ::cppwrf::phys::cu_g3::KP) X(R, dby3, ::cppwrf::phys::cu_g3::KP) X(R, qc3, ::cppwrf::phys::cu_g3::KP) X(R, pw3, ::cppwrf::phys::cu_g3::KP)       \
  X(R, hc3, ::cppwrf::phys::cu_g3::KP) X(R, qrc3, ::cppwrf::phys::cu_g3::KP) X(R, zu3, ::cppwrf::phys::cu_g3::KP) X(R, cd3, ::cppwrf::phys::cu_g3::KP) X(R, dellah3, ::cppwrf::phys::cu_g3::KP) X(R, dellaq3, ::cppwrf::phys::cu_g3::KP) X(R, dsubt3, ::cppwrf::phys::cu_g3::KP)    \
  X(R, dsubq3, ::cppwrf::phys::cu_g3::KP) X(R, dellat3, ::cppwrf::phys::cu_g3::KP)                                                                               \
  X(R, he, ::cppwrf::phys::cu_g3::KP) X(R, hes, ::cppwrf::phys::cu_g3::KP) X(R, qes, ::cppwrf::phys::cu_g3::KP) X(R, z, ::cppwrf::phys::cu_g3::KP) X(R, heo, ::cppwrf::phys::cu_g3::KP) X(R, heso, ::cppwrf::phys::cu_g3::KP) X(R, qeso, ::cppwrf::phys::cu_g3::KP) X(R, zo, ::cppwrf::phys::cu_g3::KP)    \
  X(R, xhe, ::cppwrf::phys::cu_g3::KP) X(R, xhes, ::cppwrf::phys::cu_g3::KP) X(R, xqes, ::cppwrf::phys::cu_g3::KP) X(R, xz, ::cppwrf::phys::cu_g3::KP) X(R, xt, ::cppwrf::phys::cu_g3::KP) X(R, xq, ::cppwrf::phys::cu_g3::KP) X(R, qes_cup, ::cppwrf::phys::cu_g3::KP)             \
  X(R, q_cup, ::cppwrf::phys::cu_g3::KP) X(R, he_cup, ::cppwrf::phys::cu_g3::KP) X(R, hes_cup, ::cppwrf::phys::cu_g3::KP) X(R, z_cup, ::cppwrf::phys::cu_g3::KP) X(R, p_cup, ::cppwrf::phys::cu_g3::KP) X(R, gamma_cup, ::cppwrf::phys::cu_g3::KP)           \
  X(R, t_cup, ::cppwrf::phys::cu_g3::KP) X(R, qeso_cup, ::cppwrf::phys::cu_g3::KP) X(R, qo_cup, ::cppwrf::phys::cu_g3::KP) X(R, heo_cup, ::cppwrf::phys::cu_g3::KP) X(R, heso_cup, ::cppwrf::phys::cu_g3::KP) X(R, zo_cup, ::cppwrf::phys::cu_g3::KP)        \
  X(R, po_cup, ::cppwrf::phys::cu_g3::KP) X(R, gammao_cup, ::cppwrf::phys::cu_g3::KP) X(R, tn_cup, ::cppwrf::phys::cu_g3::KP) X(R, xqes_cup, ::cppwrf::phys::cu_g3::KP) X(R, xq_cup, ::cppwrf::phys::cu_g3::KP) X(R, xhe_cup, ::cppwrf::phys::cu_g3::KP)     \
  X(R, xhes_cup, ::cppwrf::phys::cu_g3::KP) X(R, xz_cup, ::cppwrf::phys::cu_g3::KP) X(R, xt_cup, ::cppwrf::phys::cu_g3::KP) X(R, dby, ::cppwrf::phys::cu_g3::KP) X(R, qc, ::cppwrf::phys::cu_g3::KP) X(R, qrcd, ::cppwrf::phys::cu_g3::KP) X(R, pwd, ::cppwrf::phys::cu_g3::KP)     \
  X(R, pw, ::cppwrf::phys::cu_g3::KP) X(R, hcd, ::cppwrf::phys::cu_g3::KP) X(R, qcd, ::cppwrf::phys::cu_g3::KP) X(R, dbyd, ::cppwrf::phys::cu_g3::KP) X(R, hc, ::cppwrf::phys::cu_g3::KP) X(R, qrc, ::cppwrf::phys::cu_g3::KP) X(R, zu, ::cppwrf::phys::cu_g3::KP) X(R, zd, ::cppwrf::phys::cu_g3::KP)     \
  X(R, clw_all, ::cppwrf::phys::cu_g3::KP) X(R, dbyo, ::cppwrf::phys::cu_g3::KP) X(R, qco, ::cppwrf::phys::cu_g3::KP) X(R, qrcdo, ::cppwrf::phys::cu_g3::KP) X(R, pwdo, ::cppwrf::phys::cu_g3::KP) X(R, pwo, ::cppwrf::phys::cu_g3::KP) X(R, hcdo, ::cppwrf::phys::cu_g3::KP)       \
  X(R, qcdo, ::cppwrf::phys::cu_g3::KP) X(R, hco, ::cppwrf::phys::cu_g3::KP) X(R, qrco, ::cppwrf::phys::cu_g3::KP) X(R, zuo, ::cppwrf::phys::cu_g3::KP) X(R, zdo, ::cppwrf::phys::cu_g3::KP) X(R, xdby, ::cppwrf::phys::cu_g3::KP) X(R, xqc, ::cppwrf::phys::cu_g3::KP)             \
  X(R, xqrcd, ::cppwrf::phys::cu_g3::KP) X(R, xpwd, ::cppwrf::phys::cu_g3::KP) X(R, xpw, ::cppwrf::phys::cu_g3::KP) X(R, xhcd, ::cppwrf::phys::cu_g3::KP) X(R, xqcd, ::cppwrf::phys::cu_g3::KP) X(R, xhc, ::cppwrf::phys::cu_g3::KP) X(R, xqrc, ::cppwrf::phys::cu_g3::KP)          \
  X(R, xzu, ::cppwrf::phys::cu_g3::KP) X(R, xzd, ::cppwrf::phys::cu_g3::KP) X(R, cd, ::cppwrf::phys::cu_g3::KP) X(R, cdd, ::cppwrf::phys::cu_g3::KP) X(R, scr1, ::cppwrf::phys::cu_g3::KP) X(R, dellah, ::cppwrf::phys::cu_g3::KP) X(R, dellaq, ::cppwrf::phys::cu_g3::KP)          \
  X(R, dellat, ::cppwrf::phys::cu_g3::KP) X(R, dellaqc, ::cppwrf::phys::cu_g3::KP) X(R, dsubt, ::cppwrf::phys::cu_g3::KP) X(R, dsubq, ::cppwrf::phys::cu_g3::KP) X(R, tv, ::cppwrf::phys::cu_g3::KP)                                  \
  X(R, dellat_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2) X(R, dellaqc_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2) X(R, dellaq_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2)                    \
  X(R, pwo_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2) X(R, subt_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2) X(R, subq_ens, ::cppwrf::phys::cu_g3::KP * ::cppwrf::phys::cu_g3::MAXENS2)                            \
  X(R, xf_ens, ::cppwrf::phys::cu_g3::ENSDIM + 1) X(R, pr_ens, ::cppwrf::phys::cu_g3::ENSDIM + 1) X(R, massfln, ::cppwrf::phys::cu_g3::ENSDIM + 1)                                      \
  X(R, a_he, ::cppwrf::phys::cu_g3::KP) X(R, a_hes, ::cppwrf::phys::cu_g3::KP) X(R, a_qes, ::cppwrf::phys::cu_g3::KP) X(R, a_z, ::cppwrf::phys::cu_g3::KP) X(R, a_qes_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_q_cup, ::cppwrf::phys::cu_g3::KP)               \
  X(R, a_he_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_hes_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_z_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_p_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_gamma_cup, ::cppwrf::phys::cu_g3::KP)                 \
  X(R, a_t_cup, ::cppwrf::phys::cu_g3::KP) X(R, a_dby, ::cppwrf::phys::cu_g3::KP) X(R, a_hc, ::cppwrf::phys::cu_g3::KP) X(R, a_zu, ::cppwrf::phys::cu_g3::KP) X(R, a_cd, ::cppwrf::phys::cu_g3::KP) X(R, a_tv, ::cppwrf::phys::cu_g3::KP)

template <typename R>
struct G3Work {
  CPPWRF_G3_ARRAYS(CPPWRF_COLUMN_MEMBER)
};
template <typename R>
inline constexpr std::size_t kWorkBytes = 0 CPPWRF_G3_ARRAYS(CPPWRF_COLUMN_BYTES);

// element (k, n) of an (its:ite, kts:kte, 1:n) array of the column
CPPWRF_HD inline int kn(int k, int n) { return (n - 1) * KP + k; }

// ------------------------------------------------------------------------------------------------ cup_env
template <typename R, class A>
CPPWRF_HD inline void cup_env(A z, A qes, A he, A hes, A t, A q, A p, R z1, R psur, int ierr, R tcrit, int itest, R xl,
                              R cp, A tv, int ktf) {
  const R ht1 = xl / cp, ht2 = R(2.834E6) / cp;
  const R be1 = R(.622) * ht1 / R(.286), ae1 = be1 / R(273.) + rlog(R(610.71));
  const R be2 = R(.622) * ht2 / R(.286), ae2 = be2 / R(273.) + rlog(R(610.71));
  if (ierr != 0) return;
  for (int k = 1; k <= ktf; ++k) {
    const bool ice = t[k] <= tcrit;
    const R e = rexp((ice ? ae2 : ae1) - (ice ? be2 : be1) / t[k]);
    qes[k] = R(.622) * e / (R(100.) * p[k] - e);
    if (qes[k] <= R(1.E-08)) qes[k] = R(1.E-08);
    if (qes[k] < q[k]) qes[k] = q[k];
    tv[k] = t[k] + R(.608) * q[k] * t[k];
  }
  if (itest != 2) {
    z[1] = c_max(R(0.), z1) - (rlog(p[1]) - rlog(psur)) * R(287.) * tv[1] / R(9.81);
    for (int k = 2; k <= ktf; ++k) {
      const R tvbar = R(.5) * tv[k] + R(.5) * tv[k - 1];
      z[k] = z[k - 1] - (rlog(p[k]) - rlog(p[k - 1])) * R(287.) * tvbar / R(9.81);
    }
  } else {
    for (int k = 1; k <= ktf; ++k) {
      z[k] = (he[k] - R(1004.) * t[k] - R(2.5e6) * q[k]) / R(9.81);
      z[k] = c_max(R(1.e-3), z[k]);
    }
  }
  for (int k = 1; k <= ktf; ++k) {
    if (itest == 0) he[k] = R(9.81) * z[k] + R(1004.) * t[k] + R(2.5E06) * q[k];
    hes[k] = R(9.81) * z[k] + R(1004.) * t[k] + R(2.5E06) * qes[k];
    if (he[k] >= hes[k]) he[k] = hes[k];
  }
}

// ------------------------------------------------------------------------------------------------ cup_env_clev
template <typename R, class A>
CPPWRF_HD inline void cup_env_clev(A t, A qes, A q, A he, A hes, A z, A p, A qes_cup, A q_cup, A he_cup, A hes_cup,
                                   A z_cup, A p_cup, A gamma_cup, A t_cup, R psur, int ierr, R z1, R xl, R rv, R cp,
                                   int ktf) {
  if (ierr != 0) return;
  for (int k = 2; k <= ktf; ++k) {
    qes_cup[k] = R(.5) * (qes[k - 1] + qes[k]);
    q_cup[k] = R(.5) * (q[k - 1] + q[k]);
    hes_cup[k] = R(.5) * (hes[k - 1] + hes[k]);
    he_cup[k] = R(.5) * (he[k - 1] + he[k]);
    if (he_cup[k] > hes_cup[k]) he_cup[k] = hes_cup[k];
    z_cup[k] = R(.5) * (z[k - 1] + z[k]);
    p_cup[k] = R(.5) * (p[k - 1] + p[k]);
    t_cup[k] = R(.5) * (t[k - 1] + t[k]);
    gamma_cup[k] = (xl / cp) * (xl / (rv * t_cup[k] * t_cup[k])) * qes_cup[k];
  }
  qes_cup[1] = qes[1];
  q_cup[1] = q[1];
  hes_cup[1] = hes[1];
  he_cup[1] = he[1];
  z_cup[1] = R(.5) * (z[1] + z1);
  p_cup[1] = R(.5) * (p[1] + psur);
  t_cup[1] = t[1];
  gamma_cup[1] = xl / cp * (xl / (rv * t_cup[1] * t_cup[1])) * qes_cup[1];
}

// ------------------------------------------------------------------------------------------------ cup_MAXIMI / MINIMI
template <typename R, class A>
CPPWRF_HD inline int cup_maximi(A array, int ks, int ke, int ierr) {
  int maxx = ks;
  if (ierr == 0) {
    R x = array[ks];
    for (int k = ks; k <= ke; ++k) {
      const R xar = array[k];
      if (xar >= x) {
        x = xar;
        maxx = k;
      }
    }
  }
  return maxx;
}
template <typename R, class A>
CPPWRF_HD inline int cup_minimi(A array, int ks, int kend, int ierr) {
  int kt = ks;
  if (ierr == 0) {
    R x = array[ks];
    const int kstop = ks + 1 > kend ? ks + 1 : kend;
    for (int k = ks + 1; k <= kstop; ++k) {
      if (array[k] < x) {
        x = array[k];
        kt = k;
      }
    }
  }
  return kt;
}

// ------------------------------------------------------------------------------------------------ cup_kbcon
template <typename R, class A>
CPPWRF_HD inline void cup_kbcon(R cap_inc, int iloop, int& k22, int& kbcon, A he_cup, A hes_cup, int& ierr, int kbmax,
                                A p_cup, R cap_max) {
  kbcon = 1;
  if (ierr != 0) return;
  kbcon = k22;
  for (;;) {  // label 32
    R hetest = he_cup[k22];
    if (iloop == 5)
      for (int k = 1; k <= k22; ++k) hetest = c_max(hetest, he_cup[k]);
    if (hetest < hes_cup[kbcon]) {  // GO TO 31
      kbcon = kbcon + 1;
      if (kbcon > kbmax + 2) {
        if (iloop != 4) ierr = 3;
        return;
      }
      continue;
    }
    if (kbcon - k22 == 1) return;
    if (iloop == 5 && (kbcon - k22) == 0) return;
    R pbcdif = -p_cup[kbcon] + p_cup[k22];
    R plus = c_max(R(25.), cap_max - R(iloop - 1) * cap_inc);
    if (iloop == 4) plus = cap_max;
    if (iloop == 5) plus = R(25.);
    if (iloop == 5 && cap_max > R(25)) pbcdif = -p_cup[kbcon] + cap_max;
    if (pbcdif > plus) {
      k22 = k22 + 1;
      kbcon = k22;
      continue;
    }
    return;
  }
}

// ------------------------------------------------------------------------------------------------ cup_ktop
template <typename R, class A>
CPPWRF_HD inline void cup_ktop(int ilo, A dby, int kbcon, int& ktop, int& ierr, int ktf) {
  ktop = 1;
  if (ierr != 0) return;
  for (int k = kbcon + 1; k <= ktf - 1; ++k) {
    if (dby[k] <= R(0.)) {
      ktop = k - 1;
      for (int kk = ktop + 1; kk <= ktf; ++kk) dby[kk] = R(0.);
      if (kbcon == ktop) ierr = 55;
      return;
    }
  }
  if (ilo == 1) ierr = 5;
}

// ------------------------------------------------------------------------------------------------ cup_up_he
template <typename R, class A>
CPPWRF_HD inline void cup_up_he(int k22, R& hkb, A z_cup, A cd, R entr, A he_cup, A hc, int kbcon, int ierr, A dby,
                                A he, A hes_cup, bool shallow, int ktf) {
  for (int k = 1; k <= ktf; ++k) {
    hc[k] = R(0.);
    dby[k] = R(0.);
  }
  hkb = R(0.);
  if (ierr != 0) return;
  hkb = he_cup[k22];
  if (shallow)
    for (int k = 1; k <= k22; ++k) hkb = c_max(hkb, he_cup[k]);
  for (int k = 1; k <= k22; ++k) hc[k] = he_cup[k];
  for (int k = k22; k <= kbcon - 1; ++k) hc[k] = hkb;
  hc[kbcon] = hkb;
  dby[kbcon] = hkb - hes_cup[kbcon];
  for (int k = 2; k <= ktf; ++k) {
    if (k > kbcon) {
      const R dz = z_cup[k] - z_cup[k - 1];
      hc[k] = (hc[k - 1] * (R(1.) - R(.5) * cd[k] * dz) + entr * dz * he[k - 1]) /
              (R(1.) + entr * dz - R(.5) * cd[k] * dz);
      dby[k] = hc[k] - hes_cup[k];
    }
  }
}

// ------------------------------------------------------------------------------------------------ cup_up_nms
template <typename R, class A>
CPPWRF_HD inline void cup_up_nms(A zu, A z_cup, R entr, A cd, int kbcon, int ktop, int ierr, int k22, int ktf) {
  for (int k = 1; k <= ktf; ++k) zu[k] = R(0.);
  if (ierr != 0) return;
  for (int k = k22; k <= kbcon; ++k) zu[k] = R(1.);
  for (int k = kbcon + 1; k <= ktop; ++k) {
    const R dz = z_cup[k] - z_cup[k - 1];
    zu[k] = zu[k - 1] * (R(1.) + (entr - cd[k]) * dz);
  }
}

// ------------------------------------------------------------------------------------------------ cup_dd_nms
template <typename R, class A>
CPPWRF_HD inline void cup_dd_nms(A zd, A z_cup, A cdd, R entr, int jmin, int ierr, int itest, int kdet, R z1,
                                 int ktf) {
  const R perc = R(.03);
  for (int k = 1; k <= ktf; ++k) {
    zd[k] = R(0.);
    if (itest == 0) cdd[k] = R(0.);
  }
  const R a = R(1.) - perc;
  if (ierr != 0) return;
  zd[jmin] = R(1.);
  for (int ki = jmin - 1; ki >= 1; --ki) {
    const R dz = z_cup[ki + 1] - z_cup[ki];
    if (ki <= kdet && itest == 0) {
      cdd[ki] = entr + (R(1.) - (a * (z_cup[ki] - z1) + perc * (z_cup[kdet] - z1)) /
                                    (a * (z_cup[ki + 1] - z1) + perc * (z_cup[kdet] - z1))) /
                           dz;
    }
    zd[ki] = zd[ki + 1] * (R(1.) + (entr - cdd[ki]) * dz);
  }
}

// ------------------------------------------------------------------------------------------------ cup_dd_he
template <typename R, class A>
CPPWRF_HD inline void cup_dd_he(A hes_cup, A /*zd*/, A hcd, A z_cup, A cdd, R entr, int jmin, int ierr, A he, A dby,
                                A /*he_cup*/, int ktf) {
  for (int k = 2; k <= ktf; ++k) {
    dby[k] = R(0.);
    if (ierr == 0) hcd[k] = hes_cup[k];
  }
  if (ierr != 0) return;
  const int k = jmin;
  hcd[k] = hes_cup[k];
  dby[k] = hcd[jmin] - hes_cup[k];
  for (int ki = jmin - 1; ki >= 1; --ki) {
    const R dz = z_cup[ki + 1] - z_cup[ki];
    hcd[ki] = (hcd[ki + 1] * (R(1.) - R(.5) * cdd[ki] * dz) + entr * dz * he[ki]) /
              (R(1.) + entr * dz - R(.5) * cdd[ki] * dz);
    dby[ki] = hcd[ki] - hes_cup[ki];
  }
}

// ------------------------------------------------------------------------------------------------ cup_dd_moisture_3d
template <typename R, class A>
CPPWRF_HD inline void cup_dd_moisture_3d(A zd, A hcd, A hes_cup, A qcd, A qes_cup, A pwd, A q_cup, A z_cup, A cdd,
                                         R entr, int jmin, int& ierr, A gamma_cup, R& pwev, R& bu, A qrcd, A q,
                                         A /*he*/, A /*t_cup*/, int iloop, R xl, int high_resolution, int ktf) {
  bu = R(0.);
  pwev = R(0.);
  for (int k = 1; k <= ktf; ++k) {
    qcd[k] = R(0.);
    qrcd[k] = R(0.);
    pwd[k] = R(0.);
  }
  if (ierr != 0) return;
  const int k = jmin;
  R dz = z_cup[k + 1] - z_cup[k];
  qcd[k] = q_cup[k];
  if (high_resolution == 1) qcd[k] = R(.5) * (qes_cup[k] + q_cup[k]);
  qrcd[k] = qes_cup[k];
  pwd[jmin] = c_min(R(0.), qcd[k] - qrcd[k]);
  pwev = pwev + pwd[jmin];
  qcd[k] = qes_cup[k];
  R dh = hcd[k] - hes_cup[k];
  bu = dz * dh;
  for (int ki = jmin - 1; ki >= 1; --ki) {
    dz = z_cup[ki + 1] - z_cup[ki];
    qcd[ki] = (qcd[ki + 1] * (R(1.) - R(.5) * cdd[ki] * dz) + entr * dz * q[ki]) /
              (R(1.) + entr * dz - R(.5) * cdd[ki] * dz);
    dh = hcd[ki] - hes_cup[ki];
    bu = bu + dz * dh;
    qrcd[ki] = qes_cup[ki] + (R(1.) / xl) * (gamma_cup[ki] / (R(1.) + gamma_cup[ki])) * dh;
    R dqeva = qcd[ki] - qrcd[ki];
    if (dqeva > R(0.)) dqeva = R(0.);
    pwd[ki] = zd[ki] * dqeva;
    qcd[ki] = qrcd[ki];
    pwev = pwev + pwd[ki];
  }
  if (pwev == R(0.) && iloop == 1) ierr = 7;
  if (bu >= R(0.) && iloop == 1) ierr = 7;
}

// ------------------------------------------------------------------------------------------------ cup_up_moisture
template <typename R, class A>
CPPWRF_HD inline void cup_up_moisture(bool shallow, int ierr, A z_cup, A qc, A qrc, A pw, R& pwav, int kbcon, int ktop,
                                      A cd, A dby, R mentr_rate, A clw_all, A q, A gamma_cup, A zu, A qes_cup, int k22,
                                      A qe_cup, R xl, int ktf) {
  const R c0 = shallow ? R(0.) : R(.002);
  pwav = R(0.);
  for (int k = 1; k <= ktf; ++k) {
    pw[k] = R(0.);
    qc[k] = R(0.);
    if (ierr == 0) qc[k] = qes_cup[k];
    clw_all[k] = R(0.);
    qrc[k] = R(0.);
  }
  if (ierr != 0) return;
  for (int k = k22; k <= kbcon - 1; ++k) qc[k] = qe_cup[k22];
  for (int k = 2; k <= ktf; ++k) {
    if (k < kbcon || k > ktop) continue;
    const R dz = z_cup[k] - z_cup[k - 1];
    qc[k] = (qc[k - 1] * (R(1.) - R(.5) * cd[k] * dz) + mentr_rate * dz * q[k - 1]) /
            (R(1.) + mentr_rate * dz - R(.5) * cd[k] * dz);
    const R qrch = qes_cup[k] + (R(1.) / xl) * (gamma_cup[k] / (R(1.) + gamma_cup[k])) * dby[k];
    clw_all[k] = qc[k] - qrch;
    qrc[k] = (qc[k] - qrch) / (R(1.) + c0 * dz * zu[k]);
    if (qrc[k] < R(0.)) qrc[k] = R(0.);
    pw[k] = c0 * dz * qrc[k] * zu[k];
    qc[k] = qrc[k] + qrch;
    pwav = pwav + pw[k];
  }
}

// ------------------------------------------------------------------------------------------------ cup_up_aa0
template <typename R, class A>
CPPWRF_HD inline void cup_up_aa0(R& aa0, A z, A zu, A dby, A gamma_cup, A t_cup, int kbcon, int ktop, int ierr,
                                 int ktf) {
  aa0 = R(0.);
  if (ierr != 0) return;
  for (int k = 2; k <= ktf; ++k) {
    if (k <= kbcon || k > ktop) continue;
    const R dz = z[k] - z[k - 1];
    const R da = zu[k] * dz * (R(9.81) / (R(1004.) * (t_cup[k]))) * dby[k - 1] / (R(1.) + gamma_cup[k]);
    if (k == ktop && da <= R(0.)) continue;
    aa0 = aa0 + da;
    if (aa0 < R(0.)) aa0 = R(0.);
  }
}

// ------------------------------------------------------------------------------------------------ cup_dd_edt
template <typename R, class A>
CPPWRF_HD inline void cup_dd_edt(int ierr, A us, A vs, A z, int ktop, int kbcon, R& edt, A p, R pwav, R pwev,
                                 R edtmax, R edtmin, R* edtc, int ktf) {
  edt = R(0.);
  R vws = R(0.), sdp = R(0.), vshear = R(0.);
  for (int k = 1; k <= MAXENS2; ++k) edtc[k] = R(0.);
  if (ierr != 0) return;
  for (int kk = 1; kk <= ktf - 1; ++kk) {
    if (kk <= (ktop < ktf ? ktop : ktf) && kk >= kbcon) {
      vws = vws + (std::fabs((us[kk + 1] - us[kk]) / (z[kk + 1] - z[kk])) +
                   std::fabs((vs[kk + 1] - vs[kk]) / (z[kk + 1] - z[kk]))) *
                      (p[kk] - p[kk + 1]);
      sdp = sdp + p[kk] - p[kk + 1];
    }
    if (kk == ktf - 1) vshear = R(1.e3) * vws / sdp;
  }
  R pef = (R(1.591) - R(.639) * vshear + R(.0953) * (vshear * vshear) - R(.00496) * (vshear * vshear * vshear));
  if (pef > R(1.)) pef = R(1.);
  if (pef < R(0.)) pef = R(0.);
  const R zkbc = z[kbcon] * R(3.281e-3);
  R prezk = R(.02);
  if (zkbc > R(3.)) {
    prezk = R(.96729352) +
            zkbc * (R(-.70034167) +
                    zkbc * (R(.162179896) + zkbc * (R(-1.2569798E-2) + zkbc * (R(4.2772E-4) - zkbc * R(5.44E-6)))));
  }
  if (zkbc > R(25)) prezk = R(2.4);
  R pefb = R(1.) / (R(1.) + prezk);
  if (pefb > R(1.)) pefb = R(1.);
  if (pefb < R(0.)) pefb = R(0.);
  edt = R(1.) - R(.5) * (pefb + pef);
  const R einc = R(.2) * edt;
  for (int k = 1; k <= MAXENS2; ++k) edtc[k] = edt + R(k - 2) * einc;
  for (int k = 1; k <= MAXENS2; ++k) {
    edtc[k] = -edtc[k] * pwav / pwev;
    if (edtc[k] > edtmax) edtc[k] = edtmax;
    if (edtc[k] < edtmin) edtc[k] = edtmin;
  }
}

// ------------------------------------------------------------------------------------------------ cup_dellabot
template <typename R, class A>
CPPWRF_HD inline void cup_dellabot(A he_cup, int ierr, A z_cup, A p_cup, A hcd, R edt, A zd, A cdd, A he, A della,
                                   A subs, R mentrd_rate, R g) {
  della[1] = R(0.);
  subs[1] = R(0.);
  if (ierr != 0) return;
  const R dz = z_cup[2] - z_cup[1];
  const R dp = R(100.) * (p_cup[1] - p_cup[2]);
  const R detdo1 = edt * zd[2] * cdd[1] * dz;
  const R detdo2 = edt * zd[1];
  const R entdo = edt * zd[2] * mentrd_rate * dz;
  const R subin = -(edt * zd[2]);
  della[1] = (detdo1 * R(.5) * (hcd[1] + hcd[2]) + detdo2 * hcd[1] + subin * he_cup[2] - entdo * he[1]) * g / dp;
  subs[1] = R(0.);
}

// ------------------------------------------------------------------------------------------------ cup_dellas_3d
template <typename R, class A>
CPPWRF_HD inline void cup_dellas_3d(int ierr, A z_cup, A p_cup, A hcd, R edt, A zd, A cdd, A he, A della, A subs,
                                    R mentrd_rate, A zu, R g, A cd, A hc, int ktop, int k22, int kbcon, R mentr_rate,
                                    int jmin, A he_cup, int kdet, int kpbl, bool shallow, int high_res, int ktf) {
  const int kstart = shallow ? 1 : 2;
  for (int k = kstart; k <= ktf; ++k) {
    della[k] = R(0.);
    subs[k] = R(0.);
  }
  if (ierr != 0) return;
  for (int k = 2; k <= ktf - 1; ++k) {
    if (k > ktop) continue;
    if (k < k22 - 1 && shallow) continue;
    const R dz = z_cup[k + 1] - z_cup[k];
    R detdo = edt * cdd[k] * dz * zd[k + 1];
    const R entdo = edt * mentrd_rate * dz * zd[k + 1];
    R subin = -(zd[k + 1] * edt);
    R entup = R(0.), detup = R(0.);
    if (k >= kbcon && k < ktop) {
      entup = mentr_rate * dz * zu[k];
      detup = cd[k + 1] * dz * zu[k];
    }
    R subdown = -(zd[k] * edt);
    R entdoj = R(0.), entupk = R(0.), detupk = R(0.);
    if (k == jmin) entdoj = edt * zd[k];
    if (k == k22 - 1) {
      entupk = zu[kpbl];
      subin = zu[k + 1] - zd[k + 1] * edt;
      if (high_res == 1) subin = -(zd[k + 1] * edt);
    }
    if (k > kdet) detdo = R(0.);
    if (k == ktop - 0) {
      detupk = zu[ktop];
      subin = R(0.);
      subdown = R(0.);
    }
    if (k < kbcon) detup = R(0.);
    const R dp = R(100.) * (p_cup[k - 1] - p_cup[k]);
    della[k] = (detup * R(.5) * (hc[k + 1] + hc[k]) + detdo * R(.5) * (hcd[k + 1] + hcd[k]) - entup * he[k] -
                entdo * he[k] + subin * he_cup[k + 1] - subdown * he_cup[k] + detupk * (hc[ktop] - he_cup[ktop]) -
                entupk * he_cup[k22] - entdoj * he_cup[jmin]) *
               g / dp;
    if (high_res == 1) {
      della[k] = (detup * R(.5) * (hc[k + 1] + hc[k]) - entup * he[k] + (entup - detup) * he[k] +
                  detdo * R(.5) * (hcd[k + 1] + hcd[k]) - entdo * he[k] + subin * he_cup[k + 1] -
                  subdown * he_cup[k] + detupk * (hc[ktop] - he[ktop]) - entdoj * he_cup[jmin] -
                  entupk * he_cup[k22] + entupk * he[k]) *
                 g / dp;
    }
    if (k >= k22 && k < ktop) subs[k] = (zu[k + 1] * he_cup[k + 1] - zu[k] * he_cup[k]) * g / dp;
    if (high_res == 1) {
      if (k >= k22 && k < ktop)
        subs[k] = (zu[k + 1] * he_cup[k + 1] - zu[k] * he_cup[k] - (entup - detup) * he[k]) * g / dp;
      else if (k == ktop)
        subs[k] = detupk * (he[ktop] - he_cup[ktop]) * g / dp;
      else if (k == k22 - 1)
        subs[k] = (entupk * he[k] - entupk * he_cup[k]) * g / dp;
    }
  }
}

// ------------------------------------------------------------------------------------------------ cup_axx
template <typename R, class W>
CPPWRF_HD inline void cup_axx(W& w, R tcrit, int kbmax, R z1, R psur, R xl, R rv, R cp, R* axx, int ierr, R cap_max,
                              R cap_max_increment, R entr_rate, R mentr_rate, int ktf) {
  auto tx = w.tx, qx = w.qx, p = w.po;
  auto he = w.a_he, hes = w.a_hes, qes = w.a_qes, z = w.a_z, qes_cup = w.a_qes_cup, q_cup = w.a_q_cup;
  auto he_cup = w.a_he_cup, hes_cup = w.a_hes_cup, z_cup = w.a_z_cup, p_cup = w.a_p_cup, gamma_cup = w.a_gamma_cup;
  auto t_cup = w.a_t_cup, dby = w.a_dby, hc = w.a_hc, zu = w.a_zu, cd = w.a_cd, tv = w.a_tv;
  for (int n = 1; n <= ENS4; ++n) axx[n] = R(0.);
  const R ht1 = xl / cp, ht2 = R(2.834E6) / cp;
  const R be1 = R(.622) * ht1 / R(.286), ae1 = be1 / R(273.) + rlog(R(610.71));
  const R be2 = R(.622) * ht2 / R(.286), ae2 = be2 / R(273.) + rlog(R(610.71));
  for (int n = 1; n <= ENS4; ++n) {
    for (int k = 1; k <= ktf; ++k) cd[k] = R(0.1) * entr_rate;
    int ierrxx = ierr, k22xx = 1, kbconxx = 1, ktopxx = 1;
    const int kstabm = ktf - 1;
    if (ierrxx == 0) {
      for (int k = 1; k <= ktf; ++k) {
        const bool ice = tx[kn(k, n)] <= tcrit;
        const R e = rexp((ice ? ae2 : ae1) - (ice ? be2 : be1) / tx[kn(k, n)]);
        qes[k] = R(.622) * e / (R(100.) * p[k] - e);
        if (qes[k] <= R(1.E-08)) qes[k] = R(1.E-08);
        if (qx[kn(k, n)] > qes[k]) qx[kn(k, n)] = qes[k];
        tv[k] = tx[kn(k, n)] + R(.608) * qx[kn(k, n)] * tx[kn(k, n)];
      }
      z[1] = c_max(R(0.), z1) - (rlog(p[1]) - rlog(psur)) * R(287.) * tv[1] / R(9.81);
      for (int k = 2; k <= ktf; ++k) {
        const R tvbar = R(.5) * tv[k] + R(.5) * tv[k - 1];
        z[k] = z[k - 1] - (rlog(p[k]) - rlog(p[k - 1])) * R(287.) * tvbar / R(9.81);
      }
      for (int k = 1; k <= ktf; ++k) {
        he[k] = R(9.81) * z[k] + R(1004.) * tx[kn(k, n)] + R(2.5E06) * qx[kn(k, n)];
        hes[k] = R(9.81) * z[k] + R(1004.) * tx[kn(k, n)] + R(2.5E06) * qes[k];
        if (he[k] >= hes[k]) he[k] = hes[k];
      }
      for (int k = 2; k <= ktf; ++k) {
        qes_cup[k] = R(.5) * (qes[k - 1] + qes[k]);
        q_cup[k] = R(.5) * (qx[kn(k - 1, n)] + qx[kn(k, n)]);
        hes_cup[k] = R(.5) * (hes[k - 1] + hes[k]);
        he_cup[k] = R(.5) * (he[k - 1] + he[k]);
        if (he_cup[k] > hes_cup[k]) he_cup[k] = hes_cup[k];
        z_cup[k] = R(.5) * (z[k - 1] + z[k]);
        p_cup[k] = R(.5) * (p[k - 1] + p[k]);
        t_cup[k] = R(.5) * (tx[kn(k - 1, n)] + tx[kn(k, n)]);
        gamma_cup[k] = (xl / cp) * (xl / (rv * t_cup[k] * t_cup[k])) * qes_cup[k];
      }
      qes_cup[1] = qes[1];
      q_cup[1] = qx[kn(1, n)];
      hes_cup[1] = hes[1];
      he_cup[1] = he[1];
      z_cup[1] = R(.5) * (z[1] + z1);
      p_cup[1] = R(.5) * (p[1] + psur);
      t_cup[1] = tx[kn(1, n)];
      gamma_cup[1] = xl / cp * (xl / (rv * t_cup[1] * t_cup[1])) * qes_cup[1];
    }
    k22xx = cup_maximi<R>(he_cup, 3, kbmax, ierrxx);
    if (ierrxx == 0 && k22xx >= kbmax) ierrxx = 2;
    cup_kbcon<R>(cap_max_increment, 1, k22xx, kbconxx, he_cup, hes_cup, ierrxx, kbmax, p_cup, cap_max);
    const int kstabi = cup_minimi<R>(hes_cup, kbconxx, kstabm, ierrxx);
    if (ierrxx == 0 && kstabm - 1 > kstabi) {
      for (int k = kstabi; k <= kstabm - 1; ++k) {
        cd[k] = cd[k - 1] + R(1.5) * entr_rate;
        if (cd[k] > R(10.0) * entr_rate) cd[k] = R(10.0) * entr_rate;
      }
    }
    R hkb;
    cup_up_he<R>(k22xx, hkb, z_cup, cd, mentr_rate, he_cup, hc, kbconxx, ierrxx, dby, he, hes_cup, false, ktf);
    cup_ktop<R>(1, dby, kbconxx, ktopxx, ierrxx, ktf);
    cup_up_nms<R>(zu, z_cup, mentr_rate, cd, kbconxx, ktopxx, ierrxx, k22xx, ktf);
    R aa0;
    cup_up_aa0<R>(aa0, z, zu, dby, gamma_cup, t_cup, kbconxx, ktopxx, ierrxx, ktf);
    if (ierrxx == 0) axx[n] = aa0;
  }
}

// ------------------------------------------------------------------------------------------------ cup_forcing_ens_3d
// name = 'deeps', iens = 1; xland: CUP_enss_3d's xland1 (1 land, 0 water)
template <typename R, class A>
CPPWRF_HD inline void cup_forcing_ens_3d(R& closure_n, R xland, R& aa0, R aa1, const R* xaa0, const R* mbdt, R dtime,
                                         int& ierr, int ierr2, int ierr3, A xf_ens, const R* axx, int iedt,
                                         const R* mconv, int ktop, A omeg, int k22, A pr_ens, R edt, int kbcon,
                                         A massfln, int icoic, int high_resolution) {
  (void)ktop;
  const R fens4 = R(ENS4);
  if (ierr > 995) {
    aa0 = R(0.);
    ierr = 0;
  }
  if (ierr == 0) {
    R a_ave = R(0.);
    for (int ne = 1; ne <= ENS4; ++ne) a_ave = a_ave + axx[ne];
    a_ave = c_max(R(0.), a_ave / fens4);
    a_ave = c_min(a_ave, aa1);
    a_ave = c_max(R(0.), a_ave);
    R xff_ens3[17];
    for (int ne = 1; ne <= 16; ++ne) xff_ens3[ne] = R(0.);
    R xff0 = (aa1 - aa0) / dtime;
    if (high_resolution == 1) xff0 = (a_ave - aa0) / dtime;
    xff_ens3[1] = (aa1 - aa0) / dtime;
    xff_ens3[2] = (a_ave - aa0) / dtime;
    xff_ens3[3] = (aa1 - aa0) / dtime;  // irandom = 0
    xff_ens3[13] = (aa1 - aa0) / dtime;
    if (high_resolution == 1) {
      xff_ens3[1] = (a_ave - aa0) / dtime;
      xff_ens3[2] = (a_ave - aa0) / dtime;
      xff_ens3[3] = (a_ave - aa0) / dtime;
      xff_ens3[13] = (a_ave - aa0) / dtime;
    }
    xff_ens3[14] = R(0.);
    for (int ne = 1; ne <= ENS4; ++ne) xff_ens3[14] = xff_ens3[14] - omeg[kn(k22, ne)] / (fens4 * R(9.81));
    if (xff_ens3[14] < R(0.)) xff_ens3[14] = R(0.);
    xff_ens3[5] = R(0.);
    for (int ne = 1; ne <= ENS4; ++ne) xff_ens3[5] = xff_ens3[5] - omeg[kn(kbcon, ne)] / (fens4 * R(9.81));
    if (xff_ens3[5] < R(0.)) xff_ens3[5] = R(0.);
    if (high_resolution == 0) {
      xff_ens3[4] = -omeg[kn(2, 1)] / R(9.81);
      for (int k = 2; k <= kbcon - 1; ++k)
        for (int ne = 1; ne <= ENS4; ++ne) {
          const R xomg = -omeg[kn(k, ne)] / R(9.81);
          if (xomg < xff_ens3[4]) xff_ens3[4] = xomg;
        }
      if (xff_ens3[4] < R(0.)) xff_ens3[4] = R(0.);
      xff_ens3[6] = -omeg[kn(2, 1)] / R(9.81);
      for (int k = 2; k <= kbcon - 1; ++k)
        for (int ne = 1; ne <= ENS4; ++ne) {
          const R xomg = -omeg[kn(k, ne)] / R(9.81);
          if (xomg > xff_ens3[6]) xff_ens3[6] = xomg;
        }
      if (xff_ens3[6] < R(0.)) xff_ens3[6] = R(0.);
    }
    if (high_resolution == 1) {
      xff_ens3[5] = c_min(xff_ens3[5], xff_ens3[14]);
      xff_ens3[4] = xff_ens3[5];
      xff_ens3[6] = xff_ens3[5];
    }
    xff_ens3[7] = mconv[1];
    xff_ens3[8] = mconv[1];
    xff_ens3[9] = mconv[1];
    for (int ne = 2; ne <= ENS4; ++ne)
      if (mconv[ne] > xff_ens3[7]) xff_ens3[7] = mconv[ne];
    for (int ne = 2; ne <= ENS4; ++ne)
      if (mconv[ne] < xff_ens3[8]) xff_ens3[8] = mconv[ne];
    for (int ne = 2; ne <= ENS4; ++ne) xff_ens3[9] = xff_ens3[9] + mconv[ne];
    xff_ens3[9] = xff_ens3[9] / fens4;
    if (high_resolution == 1) {
      xff_ens3[7] = xff_ens3[9];
      xff_ens3[8] = xff_ens3[9];
      xff_ens3[15] = xff_ens3[9];
    }
    if (high_resolution == 0) xff_ens3[15] = mconv[1];
    xff_ens3[10] = a_ave / (R(60.) * R(40.));
    xff_ens3[11] = aa1 / (R(60.) * R(40.));
    xff_ens3[12] = aa1 / (R(60.) * R(40.));
    if (high_resolution == 1) {
      xff_ens3[11] = xff_ens3[10];
      xff_ens3[12] = xff_ens3[10];
    }
    if (icoic == 0) {
      if (xff0 < R(0.)) {
        xff_ens3[1] = R(0.);
        xff_ens3[2] = R(0.);
        xff_ens3[3] = R(0.);
        xff_ens3[13] = R(0.);
        xff_ens3[10] = R(0.);
        xff_ens3[11] = R(0.);
        xff_ens3[12] = R(0.);
      }
    }
    R xk[MAXENS + 1];
    for (int nens = 1; nens <= MAXENS; ++nens) {
      xk[nens] = (xaa0[nens] - aa1) / mbdt[2];
      if (xk[nens] <= R(0.) && xk[nens] > R(-1.e-6)) xk[nens] = R(-1.e-6);
      if (xk[nens] > R(0.) && xk[nens] < R(1.e-6)) xk[nens] = R(1.e-6);
    }
    for (int ne = 1; ne <= MAXENS; ++ne) {
      const int nall = (iedt - 1) * MAXENS * MAXENS3 + (ne - 1) * MAXENS3;
      if (xland < R(0.1)) {
        if (ierr2 > 0 || ierr3 > 0) {
          const int cs[11] = {1, 2, 3, 10, 11, 12, 7, 8, 9, 13, 15};
          for (int c : cs) {
            xff_ens3[c] = R(0.);
            massfln[nall + c] = R(0.);
          }
        }
      }
      const R massfld = R(0.);
      // iresulte = 1
      if (xff0 >= R(0.)) {
        xf_ens[nall + 1] = massfld;
        xf_ens[nall + 2] = massfld;
        xf_ens[nall + 3] = massfld;
        xf_ens[nall + 13] = massfld;
        if (xff_ens3[1] > R(0)) xf_ens[nall + 1] = c_max(R(0.), -xff_ens3[1] / xk[ne]) + massfld;
        if (xff_ens3[2] > R(0)) xf_ens[nall + 2] = c_max(R(0.), -xff_ens3[2] / xk[ne]) + massfld;
        if (xff_ens3[3] > R(0)) xf_ens[nall + 3] = c_max(R(0.), -xff_ens3[3] / xk[ne]) + massfld;
        if (xff_ens3[13] > R(0)) xf_ens[nall + 13] = c_max(R(0.), -xff_ens3[13] / xk[ne]) + massfld;
      } else {
        xf_ens[nall + 1] = massfld;
        xf_ens[nall + 2] = massfld;
        xf_ens[nall + 3] = massfld;
        xf_ens[nall + 13] = massfld;
      }
      xf_ens[nall + 4] = c_max(R(0.), xff_ens3[4] + massfld);
      xf_ens[nall + 5] = c_max(R(0.), xff_ens3[5] + massfld);
      xf_ens[nall + 6] = c_max(R(0.), xff_ens3[6] + massfld);
      xf_ens[nall + 14] = c_max(R(0.), xff_ens3[14] + massfld);
      R a1 = c_max(R(1.e-3), pr_ens[nall + 7]);
      xf_ens[nall + 7] = c_max(R(0.), xff_ens3[7] / a1);
      a1 = c_max(R(1.e-3), pr_ens[nall + 8]);
      xf_ens[nall + 8] = c_max(R(0.), xff_ens3[8] / a1);
      a1 = c_max(R(1.e-3), pr_ens[nall + 9]);
      xf_ens[nall + 9] = c_max(R(0.), xff_ens3[9] / a1);
      a1 = c_max(R(1.e-3), pr_ens[nall + 15]);
      xf_ens[nall + 15] = c_max(R(0.), xff_ens3[15] / a1);
      if (xk[ne] < R(0.)) {
        xf_ens[nall + 10] = c_max(R(0.), -xff_ens3[10] / xk[ne]) + massfld;
        xf_ens[nall + 11] = c_max(R(0.), -xff_ens3[11] / xk[ne]) + massfld;
        xf_ens[nall + 12] = c_max(R(0.), -xff_ens3[12] / xk[ne]) + massfld;
      } else {
        xf_ens[nall + 10] = massfld;
        xf_ens[nall + 11] = massfld;
        xf_ens[nall + 12] = massfld;
      }
      if (icoic >= 1) {
        closure_n = R(0.);
        for (int c = 1; c <= 16; ++c) xf_ens[nall + c] = xf_ens[nall + icoic];
      }
      xf_ens[nall + 16] = xf_ens[nall + 1];  // irandom = 0
      for (int nens3 = 1; nens3 <= MAXENS3; ++nens3) {
        massfln[nall + nens3] = edt * xf_ens[nall + nens3];
        massfln[nall + nens3] = c_max(R(0.), massfln[nall + nens3]);
      }
      if ((ne == 2 && ierr2 > 0) || (ne == 3 && ierr3 > 0)) {
        for (int c = 1; c <= 16; ++c) xf_ens[nall + c] = R(0.);
        for (int c = 1; c <= 16; ++c) massfln[nall + c] = R(0.);
      }
    }
    const int nall = (iedt - 1) * MAXENS * MAXENS3;
    const int nall2 = (iedt - 1) * MAXENS * MAXENS3 + (2 - 1) * MAXENS3;
    const int cs[11] = {4, 5, 6, 14, 7, 8, 9, 15, 10, 11, 12};
    for (int c : cs) xf_ens[nall + c] = xf_ens[nall2 + c];
  } else if (ierr != 20 && ierr != 0) {
    for (int n = 1; n <= ENSDIM; ++n) {
      xf_ens[n] = R(0.);
      massfln[n] = R(0.);
    }
  }
}

// ------------------------------------------------------------------------------------------------ massflx_stats
// pr[0..7] = pr_gr, pr_w, pr_mc, pr_st, pr_as, pr_capma, pr_capme, pr_capmi; apr[0..7] likewise
template <typename R, class A>
CPPWRF_HD inline void massflx_stats(A xf_ens, R& xt_ave, R& xt_std, int ierr, int itest, R* apr, R* pr) {
  constexpr int num = ENSDIM / MAXENS3, num2 = ENSDIM / MAXENS;
  if (itest == 1)
    for (int c = 0; c < 8; ++c) pr[c] = R(0.);
  R x_ave_cap[MAXENS + 1], x_ave[MAXENS3 + 1], x_std[MAXENS3 + 1], x_ske[MAXENS3 + 1], x_cur[MAXENS3 + 1];
  for (int k = 1; k <= MAXENS; ++k) x_ave_cap[k] = R(0.);
  for (int k = 1; k <= MAXENS3; ++k) {
    x_ave[k] = R(0.);
    x_std[k] = R(0.);
    x_ske[k] = R(0.);
    x_cur[k] = R(0.);
  }
  xt_ave = R(0.);
  xt_std = R(0.);
  R xt_ske = R(0.), xt_cur = R(0.);
  if (ierr != 0) return;
  for (int kk = 1; kk <= num; ++kk)
    for (int k = 1; k <= MAXENS3; ++k) x_ave[k] = x_ave[k] + xf_ens[MAXENS3 * (kk - 1) + k];
  for (int iedt = 1; iedt <= MAXENS2; ++iedt)
    for (int k = 1; k <= MAXENS; ++k)
      for (int kk = 1; kk <= MAXENS3; ++kk)
        x_ave_cap[k] = x_ave_cap[k] + xf_ens[MAXENS3 * (k - 1) + (iedt - 1) * MAXENS * MAXENS3 + kk];
  for (int k = 1; k <= MAXENS; ++k) x_ave_cap[k] = x_ave_cap[k] / R(num2);
  for (int k = 1; k <= MAXENS3; ++k) x_ave[k] = x_ave[k] / R(num);
  for (int k = 1; k <= MAXENS3; ++k) xt_ave = xt_ave + x_ave[k];
  xt_ave = xt_ave / R(MAXENS3);
  for (int kk = 1; kk <= num; ++kk)
    for (int k = 1; k <= MAXENS3; ++k) {
      if (x_ave[k] > R(0.)) {
        const R d = xf_ens[MAXENS3 * (kk - 1) + k] - x_ave[k];
        x_std[k] = x_std[k] + d * d;
        x_ske[k] = x_ske[k] + d * d * d;
        x_cur[k] = x_cur[k] + (d * d) * (d * d);
      }
    }
  for (int k = 1; k <= MAXENS3; ++k) {
    if (xt_ave > R(0.)) {
      const R d = x_ave[k] - xt_ave;
      xt_std = xt_std + d * d;
      xt_ske = xt_ske + d * d * d;
      xt_cur = xt_cur + (d * d) * (d * d);
    }
  }
  for (int k = 1; k <= MAXENS3; ++k) {
    if (x_std[k] > R(0.)) {
      x_std[k] = x_std[k] / R(num);
      R a3 = c_max(R(1.e-6), x_std[k]);
      x_std[k] = std::sqrt(a3);
      a3 = c_max(R(1.e-6), x_std[k] * x_std[k] * x_std[k]);
      const R a4 = c_max(R(1.e-6), (x_std[k] * x_std[k]) * (x_std[k] * x_std[k]));
      x_ske[k] = x_ske[k] / R(num) / a3;
      x_cur[k] = x_cur[k] / R(num) / a4;
    }
  }
  if (xt_std > R(0.)) {
    xt_std = xt_std / R(MAXENS3);
    R a3 = c_max(R(1.e-6), xt_std);
    xt_std = std::sqrt(a3);
    a3 = c_max(R(1.e-6), xt_std * xt_std * xt_std);
    const R a4 = c_max(R(1.e-6), (xt_std * xt_std) * (xt_std * xt_std));
    xt_ske = xt_ske / R(MAXENS3) / a3;
    xt_cur = xt_cur / R(MAXENS3) / a4;
    if (itest == 1) {
      pr[0] = R(.25) * (x_ave[1] + x_ave[2] + x_ave[3] + x_ave[13]);
      pr[1] = R(.25) * (x_ave[4] + x_ave[5] + x_ave[6] + x_ave[14]);
      pr[2] = R(.25) * (x_ave[7] + x_ave[8] + x_ave[9] + x_ave[15]);
      pr[3] = R(.333) * (x_ave[10] + x_ave[11] + x_ave[12]);
      pr[4] = x_ave[16];
      pr[5] = x_ave_cap[1];
      pr[6] = x_ave_cap[2];
      pr[7] = x_ave_cap[3];
    } else if (itest == 2) {
      apr[0] = R(.25) * (x_ave[1] + x_ave[2] + x_ave[3] + x_ave[13]) * R(3600.) * pr[0] + apr[0];
      apr[1] = R(.25) * (x_ave[4] + x_ave[5] + x_ave[6] + x_ave[14]) * R(3600.) * pr[1] + apr[1];
      apr[2] = R(.25) * (x_ave[7] + x_ave[8] + x_ave[9] + x_ave[15]) * R(3600.) * pr[2] + apr[2];
      apr[3] = R(.333) * (x_ave[10] + x_ave[11] + x_ave[12]) * R(3600.) * pr[3] + apr[3];
      apr[4] = x_ave[16] * R(3600.) * pr[4] + apr[4];
      apr[5] = x_ave_cap[1] * R(3600.) * pr[5] + apr[5];
      apr[6] = x_ave_cap[2] * R(3600.) * pr[6] + apr[6];
      apr[7] = x_ave_cap[3] * R(3600.) * pr[7] + apr[7];
    }
  }
  (void)x_ske;
  (void)x_cur;
  (void)xt_ske;
  (void)xt_cur;
}

// ------------------------------------------------------------------------------------------------ cup_output_ens_3d
// name = 'deep', nx = maxens2, nx2 = maxens, iens = 1
template <typename R, class W>
CPPWRF_HD inline void cup_output_ens_3d(W& w, int& ierr, R& pre, R& xmb, int ktop, R* apr, R closure_n, R xland1,
                                        int ktf) {
  auto xf_ens = w.xf_ens, pr_ens = w.pr_ens, massfln = w.massfln;
  const R tuning = R(0.);
  for (int k = 1; k <= ktf; ++k) {
    w.outt[k] = R(0.);
    w.outq[k] = R(0.);
    w.outqc[k] = R(0.);
    w.subt[k] = R(0.);
    w.subq[k] = R(0.);
    w.subm[k] = R(0.);
  }
  pre = R(0.);
  xmb = R(0.);
  R xfac1 = R(0.);
  if (ierr == 0)
    for (int n = 1; n <= MAXENS2 * MAXENS * MAXENS3; ++n)
      if (pr_ens[n] <= R(0.)) xf_ens[n] = R(0.);
  R pr[8];
  R xmb_ave, xmb_std, pr_ave, pr_std;
  massflx_stats<R>(xf_ens, xmb_ave, xmb_std, ierr, 1, apr, pr);
  massflx_stats<R>(pr_ens, pr_ave, pr_std, ierr, 2, apr, pr);
  if (ierr == 0) {
    if (xmb_ave <= R(0.)) {
      ierr = 13;
      xmb_ave = R(0.);
    }
    xmb = c_max(R(.1) * xmb_ave, xmb_ave - tuning * xmb_std);
    const R clos_wei = R(16.) / c_max(R(1.), closure_n);
    if (xland1 < R(0.5)) xmb = xmb * clos_wei;
    if (xmb == R(0.)) ierr = 19;
    if (xmb > R(100.)) ierr = 19;
    xfac1 = xmb;
  }
  for (int k = 1; k <= ktf; ++k) {
    R dtt = R(0.), dtts = R(0.), dtq = R(0.), dtqs = R(0.), dtqc = R(0.), dtpw = R(0.);
    if (ierr == 0 && k <= ktop) {
      for (int n = 1; n <= MAXENS2; ++n) {
        dtt = dtt + w.dellat_ens[kn(k, n)];
        dtts = dtts + w.subt_ens[kn(k, n)];
        dtq = dtq + w.dellaq_ens[kn(k, n)];
        dtqs = dtqs + w.subq_ens[kn(k, n)];
        dtqc = dtqc + w.dellaqc_ens[kn(k, n)];
        dtpw = dtpw + w.pwo_ens[kn(k, n)];
      }
      w.outt[k] = xmb * dtt / R(MAXENS2);
      w.subt[k] = xmb * dtts / R(MAXENS2);
      w.outq[k] = xmb * dtq / R(MAXENS2);
      w.subq[k] = xmb * dtqs / R(MAXENS2);
      w.outqc[k] = xmb * dtqc / R(MAXENS2);
      pre = pre + xmb * dtpw / R(MAXENS2);
      w.subm[k] = w.zuo[k] * xmb;
    }
  }
  if (ierr == 0)
    for (int k = 1; k <= MAXENS2 * MAXENS * MAXENS3; ++k) {
      massfln[k] = massfln[k] * xfac1;
      xf_ens[k] = xf_ens[k] * xfac1;
    }
  (void)pr_ave;
  (void)pr_std;
}

// ------------------------------------------------------------------------------------------------ CUP_enss_3d
struct G3Opts {
  int ichoice, ishallow, high_resolution;
};
// the column's scalars of CUP_enss_3d (in: aaeq = 0 as G3DRV, z1 = ter11, psur, xland, gsw, kpbl, apr; out: the rest)
template <typename R>
struct G3Scalars {
  R z1, psur, xland, gsw;
  int kpbl;
  R mconv[ENS4 + 1];
  R apr[8];  // APR_GR, APR_W, APR_MC, APR_ST, APR_AS, APR_CAPMA, APR_CAPME, APR_CAPMI (in and out)
  R pre, xmb3, edt_out;
  int kbcon, ktop, k23, kbcon3, ktop3;
};

template <typename R, class W>
CPPWRF_HD inline void cup_enss_3d(W& w, G3Scalars<R>& s, const G3Opts& o, R dtime, R tscl_kf, R xl, R rv, R cp, R g,
                                  int ktf) {
  const int high_resolution = o.high_resolution;
  const R tcrit = R(258.);
  const R z1 = s.z1, psur = s.psur;
  auto t = w.t2d, q = w.q2d, p = w.po, po = w.po, tn = w.tn, qo = w.qo;
  R xmb3 = R(0.);
  R closure_n = R(16.);
  R xland1 = R(1.);
  if (s.xland > R(1.5)) xland1 = R(0.);
  const R cap_max_increment = R(25.);
  const int iens = 1;
  const R radius = iens <= 4 ? R(14000.) - R(iens) * R(2000.) : R(12000.);
  const R entr_rate = R(.2) / radius;
  const R entr_rate3 = R(.2) / R(200.);
  const R mentrd_rate = R(0.);
  const R mentr_rate = entr_rate;
  const R mentr_rate3 = entr_rate3;
  for (int k = 1; k <= ktf; ++k) {
    w.cupclw[k] = R(0.);
    w.cd[k] = R(0.01) * entr_rate;
    w.cd3[k] = entr_rate3;
    w.cdd[k] = R(0.);
    w.zdo3[k] = R(0.);
    w.hcdo[k] = R(0.);
    w.qrcdo[k] = R(0.);
    w.dellaqc[k] = R(0.);
  }
  const R edtmax = R(1.), edtmin = R(.2), depth_min = R(500.), cap_maxs = R(75.);
  int kbmax = 1;
  R aa0 = R(0.), aa3_0 = R(0.), aa1 = R(0.), aa3 = R(0.);
  R edt = R(0.), edt3 = R(0.);
  int kstabm = ktf - 1;
  int ierr = 0, ierr2 = 0, ierr3 = 0, ierr5 = 0;
  R cap_max = cap_maxs;
  R cap_max3 = R(25.);
  if (s.gsw < R(1) || high_resolution == 1) cap_max = R(25.);
  const R zkbmax = R(4000.), zcutdown = R(3000.), z_detr = R(1250.);
  R mbdt_ens[MAXENS + 1], edt_ens[MAXENS2 + 1];
  for (int nens = 1; nens <= MAXENS; ++nens)
    mbdt_ens[nens] = (R(nens) - R(3.)) * dtime * R(1.e-3) + dtime * R(5.E-03);
  for (int nens = 1; nens <= MAXENS2; ++nens) edt_ens[nens] = R(.95) - R(nens) * R(.01);
  (void)edt_ens;
  for (int k = 1; k <= MAXENS * MAXENS2 * MAXENS3; ++k) {  // ierr /= 20
    w.xf_ens[k] = R(0.);
    w.pr_ens[k] = R(0.);
  }
  cup_env<R>(w.z, w.qes, w.he, w.hes, t, q, p, z1, psur, ierr, tcrit, 0, xl, cp, w.tv, ktf);
  cup_env<R>(w.zo, w.qeso, w.heo, w.heso, tn, qo, po, z1, psur, ierr, tcrit, 0, xl, cp, w.tv, ktf);
  cup_env_clev<R>(t, w.qes, q, w.he, w.hes, w.z, p, w.qes_cup, w.q_cup, w.he_cup, w.hes_cup, w.z_cup, w.p_cup,
                  w.gamma_cup, w.t_cup, psur, ierr, z1, xl, rv, cp, ktf);
  cup_env_clev<R>(tn, w.qeso, qo, w.heo, w.heso, w.zo, po, w.qeso_cup, w.qo_cup, w.heo_cup, w.heso_cup, w.zo_cup,
                  w.po_cup, w.gammao_cup, w.tn_cup, psur, ierr, z1, xl, rv, cp, ktf);
  // aaeq = 0 (G3DRV): ierr is not set to 20
  int kdet = 0;  // WRF: undefined if no level is above z_detr (never in a real column)
  for (int k = 1; k <= ktf; ++k)
    if (w.zo_cup[k] > zkbmax + z1) {
      kbmax = k;
      break;
    }
  for (int k = 1; k <= ktf; ++k)
    if (w.zo_cup[k] > z_detr + z1) {
      kdet = k;
      break;
    }
  int k22 = cup_maximi<R>(w.heo_cup, 3, kbmax, ierr);
  if (ierr == 0 && k22 >= kbmax) ierr = 2;
  int kbcon = 1, ktop = 1;
  cup_kbcon<R>(cap_max_increment, 1, k22, kbcon, w.heo_cup, w.heso_cup, ierr, kbmax, w.po_cup, cap_max);
  const int kstabi = cup_minimi<R>(w.heso_cup, kbcon, kstabm, ierr);
  if (ierr == 0 && kstabm - 1 > kstabi) {
    for (int k = kstabi; k <= kstabm - 1; ++k) {
      w.cd[k] = w.cd[k - 1] + R(.15) * entr_rate;
      if (w.cd[k] > R(1.0) * entr_rate) w.cd[k] = R(1.0) * entr_rate;
    }
  }
  R hkb, hkbo;
  cup_up_he<R>(k22, hkb, w.z_cup, w.cd, mentr_rate, w.he_cup, w.hc, kbcon, ierr, w.dby, w.he, w.hes_cup, false, ktf);
  cup_up_he<R>(k22, hkbo, w.zo_cup, w.cd, mentr_rate, w.heo_cup, w.hco, kbcon, ierr, w.dbyo, w.heo, w.heso_cup, false,
               ktf);
  cup_ktop<R>(1, w.dbyo, kbcon, ktop, ierr, ktf);
  int kzdown = 0;
  if (ierr == 0) {
    R zktop = (w.zo_cup[ktop] - z1) * R(.6);
    zktop = c_min(zktop + z1, zcutdown + z1);
    for (int k = 1; k <= ktf; ++k)
      if (w.zo_cup[k] > zktop) {
        kzdown = k;
        break;
      }
  }
  int jmin = cup_minimi<R>(w.heso_cup, k22, kzdown, ierr);
  if (ierr == 0) {
    int jmini = jmin;
    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      if (jmini - 1 < kdet) kdet = jmini - 1;
      if (jmini >= ktop - 1) jmini = ktop - 2;
      const int ki = jmini;
      w.hcdo[ki] = w.heso_cup[ki];
      R dh = R(0.);
      for (int k = ki - 1; k >= 1; --k) {
        w.hcdo[k] = w.heso_cup[jmini];
        const R dz = w.zo_cup[k + 1] - w.zo_cup[k];
        dh = dh + dz * (w.hcdo[k] - w.heso_cup[k]);
        if (dh > R(0.)) {
          jmini = jmini - 1;
          if (jmini > 3) {
            keep_going = true;
          } else {
            ierr = 9;
            break;
          }
        }
      }
    }
    jmin = jmini;
    if (jmini <= 3) ierr = 4;
  }
  if (ierr == 0 && -w.zo_cup[kbcon] + w.zo_cup[ktop] < depth_min) ierr = 6;
  cup_up_nms<R>(w.zu, w.z_cup, mentr_rate, w.cd, kbcon, ktop, ierr, k22, ktf);
  cup_up_nms<R>(w.zuo, w.zo_cup, mentr_rate, w.cd, kbcon, ktop, ierr, k22, ktf);
  cup_dd_nms<R>(w.zd, w.z_cup, w.cdd, mentrd_rate, jmin, ierr, 0, kdet, z1, ktf);
  cup_dd_nms<R>(w.zdo, w.zo_cup, w.cdd, mentrd_rate, jmin, ierr, 1, kdet, z1, ktf);
  cup_dd_he<R>(w.hes_cup, w.zd, w.hcd, w.z_cup, w.cdd, mentrd_rate, jmin, ierr, w.he, w.dbyd, w.he_cup, ktf);
  cup_dd_he<R>(w.heso_cup, w.zdo, w.hcdo, w.zo_cup, w.cdd, mentrd_rate, jmin, ierr, w.heo, w.dbyd, w.he_cup, ktf);
  R pwev, bu, pwevo, pwav, pwavo;
  cup_dd_moisture_3d<R>(w.zd, w.hcd, w.hes_cup, w.qcd, w.qes_cup, w.pwd, w.q_cup, w.z_cup, w.cdd, mentrd_rate, jmin,
                        ierr, w.gamma_cup, pwev, bu, w.qrcd, q, w.he, w.t_cup, 2, xl, high_resolution, ktf);
  cup_dd_moisture_3d<R>(w.zdo, w.hcdo, w.heso_cup, w.qcdo, w.qeso_cup, w.pwdo, w.qo_cup, w.zo_cup, w.cdd, mentrd_rate,
                        jmin, ierr, w.gammao_cup, pwevo, bu, w.qrcdo, qo, w.heo, w.tn_cup, 1, xl, high_resolution, ktf);
  cup_up_moisture<R>(false, ierr, w.z_cup, w.qc, w.qrc, w.pw, pwav, kbcon, ktop, w.cd, w.dby, mentr_rate, w.clw_all, q,
                     w.gamma_cup, w.zu, w.qes_cup, k22, w.q_cup, xl, ktf);
  for (int k = 1; k <= ktf; ++k) w.cupclw[k] = w.qrc[k];
  cup_up_moisture<R>(false, ierr, w.zo_cup, w.qco, w.qrco, w.pwo, pwavo, kbcon, ktop, w.cd, w.dbyo, mentr_rate,
                     w.clw_all, qo, w.gammao_cup, w.zuo, w.qeso_cup, k22, w.qo_cup, xl, ktf);
  cup_up_aa0<R>(aa0, w.z, w.zu, w.dby, w.gamma_cup, w.t_cup, kbcon, ktop, ierr, ktf);
  cup_up_aa0<R>(aa1, w.zo, w.zuo, w.dbyo, w.gammao_cup, w.tn_cup, kbcon, ktop, ierr, ktf);
  if (ierr == 0 && aa1 == R(0.)) ierr = 17;
  int k23 = 0, kbcon3 = 0, ktop3 = 0;  // WRF: undefined without the shallow convection (only output)
  if (o.ishallow == 1) {
    auto tshall = w.tshall, qshall = w.qshall;
    cup_env<R>(w.z3, w.qes3, w.he3, w.hes3, tshall, qshall, po, z1, psur, ierr5, tcrit, 0, xl, cp, w.tv, ktf);
    cup_env_clev<R>(tshall, w.qes3, qshall, w.he3, w.hes3, w.z3, po, w.qes3_cup, w.q3_cup, w.he3_cup, w.hes3_cup,
                    w.z3_cup, w.po_cup, w.gamma3_cup, w.t3_cup, psur, ierr5, z1, xl, rv, cp, ktf);
    k23 = cup_maximi<R>(w.he3_cup, 1, kbmax, ierr5);
    if (s.kpbl > 5) cap_max3 = w.po_cup[s.kpbl];
    if (ierr5 == 0) {
      if (k23 > kbmax) ierr5 = 2;
      if (s.kpbl > 5) k23 = s.kpbl;
    }
    cup_kbcon<R>(cap_max_increment, 5, k23, kbcon3, w.he3_cup, w.hes3_cup, ierr5, kbmax, w.po_cup, cap_max3);
    R hkb3, hkb3_0;
    cup_up_he<R>(k23, hkb3, w.z3_cup, w.cd3, mentr_rate3, w.he3_cup, w.hc3, kbcon3, ierr5, w.dby3, w.he3, w.hes3_cup,
                 true, ktf);
    cup_up_he<R>(k23, hkb3_0, w.z_cup, w.cd3, mentr_rate3, w.he_cup, w.hc3_0, kbcon3, ierr5, w.dby3_0, w.he,
                 w.hes_cup, true, ktf);
    cup_ktop<R>(1, w.dby3, kbcon3, ktop3, ierr5, ktf);
    cup_up_nms<R>(w.zu3, w.z3_cup, mentr_rate3, w.cd3, kbcon3, ktop3, ierr5, k23, ktf);
    cup_up_nms<R>(w.zu3_0, w.z_cup, mentr_rate3, w.cd3, kbcon3, ktop3, ierr5, k23, ktf);
    cup_up_aa0<R>(aa3_0, w.z, w.zu3_0, w.dby3_0, w.gamma3_cup, w.t_cup, kbcon3, ktop3, ierr5, ktf);
    R pwav3;
    cup_up_moisture<R>(true, ierr5, w.z3_cup, w.qc3, w.qrc3, w.pw3, pwav3, kbcon3, ktop3, w.cd3, w.dby3, mentr_rate3,
                       w.clw_all, qshall, w.gamma3_cup, w.zu3, w.qes3_cup, k23, w.q3_cup, xl, ktf);
    cup_up_aa0<R>(aa3, w.z3, w.zu3, w.dby3, w.gamma3_cup, w.t3_cup, kbcon3, ktop3, ierr5, ktf);
    cup_dellas_3d<R>(ierr5, w.z3_cup, w.po_cup, w.hcdo, edt3, w.zdo3, w.cdd, w.he3, w.dellah3, w.dsubt3, mentrd_rate,
                     w.zu3, g, w.cd3, w.hc3, ktop3, k23, kbcon3, mentr_rate3, jmin, w.he3_cup, kdet, k23, true, 0, ktf);
    cup_dellas_3d<R>(ierr5, w.z3_cup, w.po_cup, w.qrcdo, edt3, w.zdo3, w.cdd, qshall, w.dellaq3, w.dsubq3,
                     mentrd_rate, w.zu3, g, w.cd3, w.qc3, ktop3, k23, kbcon3, mentr_rate3, jmin, w.q3_cup, kdet, k23,
                     true, 0, ktf);
    const R mbdt_s = R(1.e-1) * mbdt_ens[1];
    for (int k = 1; k <= ktf; ++k) {
      w.dellat3[k] = R(0.);
      if (ierr5 == 0) {
        w.xhe3[k] = (w.dsubt3[k] + w.dellah3[k]) * mbdt_s + w.he3[k];
        w.xq3[k] = (w.dsubq3[k] + w.dellaq3[k]) * mbdt_s + qshall[k];
        w.dellat3[k] = (R(1.) / cp) * (w.dellah3[k] - xl * w.dellaq3[k]);
        w.dsubt3[k] = (R(1.) / cp) * (w.dsubt3[k] - xl * w.dsubq3[k]);
        w.xt3[k] = (w.dellat3[k] + w.dsubt3[k]) * mbdt_s + tshall[k];
        if (w.xq3[k] <= R(0.)) w.xq3[k] = R(1.E-08);
      }
    }
    if (ierr5 == 0) {
      w.xhe3[ktf] = w.he3[ktf];
      w.xq3[ktf] = qshall[ktf];
      w.xt3[ktf] = tshall[ktf];
      if (w.xq3[ktf] <= R(0.)) w.xq3[ktf] = R(1.E-08);
    }
    cup_env<R>(w.xz3, w.xqes3, w.xhe3, w.xhes3, w.xt3, w.xq3, po, z1, psur, ierr5, tcrit, 2, xl, cp, w.tv, ktf);
    cup_env_clev<R>(w.xt3, w.xqes3, w.xq3, w.xhe3, w.xhes3, w.xz3, po, w.xqes3_cup, w.xq3_cup, w.xhe3_cup,
                    w.xhes3_cup, w.xz3_cup, w.po_cup, w.gamma3_cup, w.xt3_cup, psur, ierr5, z1, xl, rv, cp, ktf);
    R xhkb3 = R(0.);
    if (ierr5 == 0) xhkb3 = w.xhe3[k23];
    cup_up_he<R>(k23, xhkb3, w.xz3_cup, w.cd3, mentr_rate3, w.xhe3_cup, w.xhc3, kbcon3, ierr5, w.xdby3, w.xhe3,
                 w.xhes3_cup, true, ktf);
    cup_up_nms<R>(w.xzu3, w.xz3_cup, mentr_rate3, w.cd3, kbcon3, ktop3, ierr5, k23, ktf);
    R xaa3;
    cup_up_aa0<R>(xaa3, w.xz3, w.xzu3, w.xdby3, w.gamma3_cup, w.xt3_cup, kbcon3, ktop3, ierr5, ktf);
    xmb3 = R(0.);
    R xff_shal[10];
    for (int k = 1; k <= 9; ++k) xff_shal[k] = R(0.);
    if (ierr5 == 0) {
      R xkshal = (xaa3 - aa3) / mbdt_s;
      if (xkshal >= R(0.)) xkshal = R(+1.e6);
      if (xkshal > R(-1.e-4) && xkshal < R(0.)) xkshal = R(-1.e-4);
      xff_shal[1] = c_max(R(0.), -(aa3 - aa3_0) / (xkshal * dtime));
      xff_shal[2] = c_max(R(0.), -(aa3 - aa3_0) / (xkshal * dtime));
      xff_shal[3] = c_max(R(0.), -(aa3 - aa3_0) / (xkshal * dtime));
      if (aa3_0 <= R(0)) {
        xff_shal[1] = R(0.);
        xff_shal[2] = R(0.);
        xff_shal[3] = R(0.);
      }
      if (aa3 - aa3_0 <= R(0.)) {
        xff_shal[1] = R(0.);
        xff_shal[2] = R(0.);
        xff_shal[3] = R(0.);
      }
      R blqe = R(0.);
      R trash = R(0.);
      if (k23 < s.kpbl + 1) {
        for (int k = 1; k <= kbcon3 - 1; ++k)
          blqe = blqe + R(100.) * w.dhdt[k] * (w.p_cup[k] - w.p_cup[k + 1]) / g;
        trash = c_max((w.hc3[kbcon3] - w.he_cup[kbcon3]), R(1.e1));
        xff_shal[7] = c_max(R(0.), blqe / trash);
        xff_shal[7] = c_min(R(0.1), xff_shal[7]);
      } else {
        xff_shal[7] = R(0.);
      }
      if ((xkshal < R(-1.1e-04)) && ((aa3 - aa3_0 > R(0.)) || (xff_shal[7] > R(0)))) {
        xff_shal[4] = c_max(R(0.), -aa3 / (xkshal * tscl_kf));
        xff_shal[4] = c_min(R(0.1), xff_shal[4]);
        xff_shal[5] = xff_shal[4];
        xff_shal[6] = xff_shal[4];
      } else {
        xff_shal[4] = R(0.);
        xff_shal[5] = R(0.);
        xff_shal[6] = R(0.);
      }
      xff_shal[8] = xff_shal[7];
      xff_shal[9] = xff_shal[7];
      for (int k = 1; k <= 9; ++k) xmb3 = xmb3 + xff_shal[k];
      xmb3 = c_min(R(.1), xmb3 / R(9.));
      if (xmb3 == R(0.)) ierr5 = 22;
      if (xmb3 < R(0.)) ierr5 = 21;
    }
    if (ierr5 != 0) {
      k23 = 0;
      kbcon3 = 0;
      ktop3 = 0;
      xmb3 = R(0);
      for (int k = 1; k <= ktf; ++k) {
        w.outts[k] = R(0.);
        w.outqs[k] = R(0.);
      }
    } else {
      R trash = R(0.);
      for (int k = 2; k <= ktop3; ++k)
        trash = c_max(trash, R(86400.) * (w.dsubt3[k] + w.dellat3[k]) * xmb3);
      if (trash > R(150.)) xmb3 = xmb3 * R(150.) / trash;
      for (int k = 2; k <= ktop3; ++k) {
        trash = q[k] + (w.dsubq3[k] + w.dellaq3[k]) * xmb3 * dtime;
        if (trash < R(1.e-12)) {
          trash = ((R(1.e-12) - q[k]) / dtime) / ((w.dsubq3[k] + w.dellaq3[k]) * xmb3);
          trash = c_max(R(0.), trash);
          trash = c_min(R(1.), trash);
          xmb3 = trash * xmb3;
        }
      }
      for (int k = 2; k <= ktop3; ++k) {
        w.outts[k] = (w.dsubt3[k] + w.dellat3[k]) * xmb3;
        w.outqs[k] = (w.dsubq3[k] + w.dellaq3[k]) * xmb3;
      }
    }
  }
  R axx[ENS4 + 1];
  cup_axx<R>(w, tcrit, kbmax, z1, psur, xl, rv, cp, axx, ierr, cap_max, cap_max_increment, entr_rate, mentr_rate, ktf);
  R edtc[MAXENS2 + 1];
  cup_dd_edt<R>(ierr, w.us, w.vs, w.zo, ktop, kbcon, edt, po, pwavo, pwevo, edtmax, edtmin, edtc, ktf);
  R edto = R(0.), edtx = R(0.);
  R xaa0_ens[MAXENS + 1];
  for (int iedt = 1; iedt <= MAXENS2; ++iedt) {
    if (ierr == 0) {
      edt = edtc[iedt];
      edto = edtc[iedt];
      edtx = edtc[iedt];
      s.edt_out = edtc[2];
      if (high_resolution == 1) {
        edt = edtc[3];
        edto = edtc[3];
        edtx = edtc[3];
        s.edt_out = edtc[3];
      }
    }
    for (int k = 1; k <= ktf; ++k) {
      w.subt_ens[kn(k, iedt)] = R(0.);
      w.subq_ens[kn(k, iedt)] = R(0.);
      w.dellat_ens[kn(k, iedt)] = R(0.);
      w.dellaq_ens[kn(k, iedt)] = R(0.);
      w.dellaqc_ens[kn(k, iedt)] = R(0.);
      w.pwo_ens[kn(k, iedt)] = R(0.);
    }
    cup_dellabot<R>(w.heo_cup, ierr, w.zo_cup, po, w.hcdo, edto, w.zdo, w.cdd, w.heo, w.dellah, w.dsubt, mentrd_rate,
                    g);
    cup_dellabot<R>(w.qo_cup, ierr, w.zo_cup, po, w.qrcdo, edto, w.zdo, w.cdd, qo, w.dellaq, w.dsubq, mentrd_rate, g);
    cup_dellas_3d<R>(ierr, w.zo_cup, w.po_cup, w.hcdo, edto, w.zdo, w.cdd, w.heo, w.dellah, w.dsubt, mentrd_rate,
                     w.zuo, g, w.cd, w.hco, ktop, k22, kbcon, mentr_rate, jmin, w.heo_cup, kdet, k22, false,
                     high_resolution, ktf);
    for (int k = 1; k <= ktf - 1; ++k) {
      w.scr1[k] = R(0.);
      w.dellaqc[k] = R(0.);
      if (ierr == 0) {
        w.scr1[k] = w.qco[k] - w.qrco[k];
        if (k == ktop - 0)
          w.dellaqc[k] = R(.01) * w.zuo[ktop] * w.qrco[ktop] * R(9.81) / (w.po_cup[k] - w.po_cup[k + 1]);
        if (k < ktop && k > kbcon) {
          const R dz = w.zo_cup[k + 1] - w.zo_cup[k];
          w.dellaqc[k] = R(.01) * R(9.81) * w.cd[k] * dz * w.zuo[k] * R(.5) * (w.qrco[k] + w.qrco[k + 1]) /
                         (w.po_cup[k] - w.po_cup[k + 1]);
        }
      }
    }
    cup_dellas_3d<R>(ierr, w.zo_cup, w.po_cup, w.qrcdo, edto, w.zdo, w.cdd, qo, w.dellaq, w.dsubq, mentrd_rate, w.zuo,
                     g, w.cd, w.qco, ktop, k22, kbcon, mentr_rate, jmin, w.qo_cup, kdet, k22, false, high_resolution,
                     ktf);
    const R mbdt = mbdt_ens[2];
    xaa0_ens[1] = R(0.);
    xaa0_ens[2] = R(0.);
    xaa0_ens[3] = R(0.);
    for (int k = 1; k <= ktf; ++k) {
      w.dellat[k] = R(0.);
      if (ierr == 0) {
        w.xhe[k] = (w.dsubt[k] + w.dellah[k]) * mbdt + w.heo[k];
        w.xq[k] = (w.dsubq[k] + w.dellaq[k]) * mbdt + qo[k];
        w.dellat[k] = (R(1.) / cp) * (w.dellah[k] - xl * w.dellaq[k]);
        w.dsubt[k] = (R(1.) / cp) * (w.dsubt[k] - xl * w.dsubq[k]);
        w.xt[k] = (w.dellat[k] + w.dsubt[k]) * mbdt + tn[k];
        if (w.xq[k] <= R(0.)) w.xq[k] = R(1.E-08);
      }
    }
    if (ierr == 0) {
      w.xhe[ktf] = w.heo[ktf];
      w.xq[ktf] = qo[ktf];
      w.xt[ktf] = tn[ktf];
      if (w.xq[ktf] <= R(0.)) w.xq[ktf] = R(1.E-08);
    }
    cup_env<R>(w.xz, w.xqes, w.xhe, w.xhes, w.xt, w.xq, po, z1, psur, ierr, tcrit, 2, xl, cp, w.tv, ktf);
    cup_env_clev<R>(w.xt, w.xqes, w.xq, w.xhe, w.xhes, w.xz, po, w.xqes_cup, w.xq_cup, w.xhe_cup, w.xhes_cup, w.xz_cup,
                    w.po_cup, w.gamma_cup, w.xt_cup, psur, ierr, z1, xl, rv, cp, ktf);
    R xhkb = R(0.);
    if (ierr == 0) xhkb = w.xhe[k22];
    cup_up_he<R>(k22, xhkb, w.xz_cup, w.cd, mentr_rate, w.xhe_cup, w.xhc, kbcon, ierr, w.xdby, w.xhe, w.xhes_cup,
                 false, ktf);
    cup_up_nms<R>(w.xzu, w.xz_cup, mentr_rate, w.cd, kbcon, ktop, ierr, k22, ktf);
    cup_dd_nms<R>(w.xzd, w.xz_cup, w.cdd, mentrd_rate, jmin, ierr, 1, kdet, z1, ktf);
    cup_dd_he<R>(w.xhes_cup, w.xzd, w.xhcd, w.xz_cup, w.cdd, mentrd_rate, jmin, ierr, w.xhe, w.dbyd, w.xhe_cup, ktf);
    R xpwev, xbu, xpwav;
    cup_dd_moisture_3d<R>(w.xzd, w.xhcd, w.xhes_cup, w.xqcd, w.xqes_cup, w.xpwd, w.xq_cup, w.xz_cup, w.cdd,
                          mentrd_rate, jmin, ierr, w.gamma_cup, xpwev, xbu, w.xqrcd, w.xq, w.xhe, w.xt_cup, 3, xl,
                          high_resolution, ktf);
    cup_up_moisture<R>(false, ierr, w.xz_cup, w.xqc, w.xqrc, w.xpw, xpwav, kbcon, ktop, w.cd, w.xdby, mentr_rate,
                       w.clw_all, w.xq, w.gamma_cup, w.xzu, w.xqes_cup, k22, w.xq_cup, xl, ktf);
    R xaa0;
    cup_up_aa0<R>(xaa0, w.xz, w.xzu, w.xdby, w.gamma_cup, w.xt_cup, kbcon, ktop, ierr, ktf);
    for (int nens = 1; nens <= MAXENS; ++nens) {
      if (ierr == 0) {
        xaa0_ens[nens] = xaa0;
        const int nall = (iens - 1) * MAXENS3 * MAXENS * MAXENS2 + (iedt - 1) * MAXENS * MAXENS3 + (nens - 1) * MAXENS3;
        for (int k = 1; k <= ktf; ++k) {
          if (k <= ktop) {
            for (int nens3 = 1; nens3 <= MAXENS3; ++nens3) {
              if (nens3 == 7)
                w.pr_ens[nall + nens3] = w.pr_ens[nall + nens3] + edto * w.pwdo[k] + w.pwo[k];
              else if (nens3 == 8)
                w.pr_ens[nall + nens3] = w.pr_ens[nall + nens3] + w.pwo[k];
              else if (nens3 == 9)
                w.pr_ens[nall + nens3] = w.pr_ens[nall + nens3] + R(.5) * edto * w.pwdo[k] + w.pwo[k];
              else
                w.pr_ens[nall + nens3] = w.pr_ens[nall + nens3] + w.pwo[k] + edto * w.pwdo[k];
            }
          }
        }
        if (w.pr_ens[nall + 7] < R(1.e-6)) {
          ierr = 18;
          for (int nens3 = 1; nens3 <= MAXENS3; ++nens3) w.pr_ens[nall + nens3] = R(0.);
        }
        for (int nens3 = 1; nens3 <= MAXENS3; ++nens3)
          if (w.pr_ens[nall + nens3] < R(1.e-4)) w.pr_ens[nall + nens3] = R(0.);
      }
    }
    const int k22x = cup_maximi<R>(w.heo_cup, 3, kbmax, ierr);
    ierr2 = ierr;
    ierr3 = ierr;
    int k22xa = k22x, k22xb = k22x, kbconx;
    cup_kbcon<R>(cap_max_increment, 2, k22xa, kbconx, w.heo_cup, w.heso_cup, ierr2, kbmax, w.po_cup, cap_max);
    k22xb = k22xa;  // cup_kbcon's k22 is INTENT(INOUT): the second call starts from the first's K22X
    cup_kbcon<R>(cap_max_increment, 3, k22xb, kbconx, w.heo_cup, w.heso_cup, ierr3, kbmax, w.po_cup, cap_max);
    R mconv1[ENS4 + 1];
    for (int n = 1; n <= ENS4; ++n) mconv1[n] = s.mconv[n];
    cup_forcing_ens_3d<R>(closure_n, xland1, aa0, aa1, xaa0_ens, mbdt_ens, dtime, ierr, ierr2, ierr3, w.xf_ens, axx,
                          iedt, mconv1, ktop, w.omeg, k22, w.pr_ens, edto, kbcon, w.massfln, o.ichoice,
                          high_resolution);
    for (int k = 1; k <= ktf; ++k) {
      if (ierr == 0) {
        w.subt_ens[kn(k, iedt)] = w.dsubt[k];
        w.subq_ens[kn(k, iedt)] = w.dsubq[k];
        w.dellat_ens[kn(k, iedt)] = w.dellat[k];
        w.dellaq_ens[kn(k, iedt)] = w.dellaq[k];
        w.dellaqc_ens[kn(k, iedt)] = w.dellaqc[k];
        w.pwo_ens[kn(k, iedt)] = w.pwo[k] + edt * w.pwdo[k];
      } else {
        w.subt_ens[kn(k, iedt)] = R(0.);
        w.subq_ens[kn(k, iedt)] = R(0.);
        w.dellat_ens[kn(k, iedt)] = R(0.);
        w.dellaq_ens[kn(k, iedt)] = R(0.);
        w.dellaqc_ens[kn(k, iedt)] = R(0.);
        w.pwo_ens[kn(k, iedt)] = R(0.);
      }
    }
  }
  (void)edtx;
  R xmb;
  cup_output_ens_3d<R>(w, ierr, s.pre, xmb, ktop, s.apr, closure_n, xland1, ktf);
  if (ierr == 0 && ierr5 == 0 && kbcon < ktop3 + 1) {
    if (high_resolution == 1)
      for (int k = 1; k <= ktf; ++k) {
        w.outts[k] = R(0.);
        w.outqs[k] = R(0.);
      }
  }
  s.pre = c_max(s.pre, R(0.));
  s.xmb3 = xmb3;
  s.kbcon = kbcon;
  s.ktop = ktop;
  s.k23 = k23;
  s.kbcon3 = kbcon3;
  s.ktop3 = ktop3;
}

// ------------------------------------------------------------------------------------------------ G3DRV
// Fields of one call, read at (i, j) and the neighbours (domain indices 0 .. nx-1, 0 .. ny-1; level k = 0 .. nz-1,
// w at the faces): F provides them (CPPWRF_HD members) as
//   t(i,j,k) q(i,j,k) p(i,j,k) u v rho pi w(i,j,k) p8w(i,j,k) rthften rqvften rthblten rqvblten (3D, rthften already
//   the cumulus_driver's (RTHFTEN + RTHRATEN + RTHBLTEN) * pi), ht(i,j) xland(i,j) gsw(i,j) kpbl(i,j)
template <typename R>
struct G3Call {
  int nx, ny, nz;
  R dt, dx, xlv, cp, g, r_v;
  int cugd_avedx, ishallow, ichoice;
  bool periodic_x, periodic_y, f_qc, f_qi;
};

// RTHFTEN's 3 x 3 mean of G3DRV's high-resolution branch (ave_f_t / ave_f_q at (i, j); no clamping, as WRF)
template <typename R, class F>
CPPWRF_HD inline R g3_ave9(const F& f, bool th, int i, int j, int k) {
  auto v = [&](int a, int b) { return th ? f.rthften(a, b, k) : f.rqvften(a, b, k); };
  return (v(i - 1, j - 1) + v(i - 1, j) + v(i - 1, j + 1) + v(i, j - 1) + v(i, j) + v(i, j + 1) + v(i + 1, j - 1) +
          v(i + 1, j) + v(i + 1, j + 1)) /
         R(9.);
}

// G3DRV's column work before CUP_enss_3d for column (i, j)
template <typename R, class F, class W>
CPPWRF_HD inline void g3_gather(const G3Call<R>& c, const F& f, int i, int j, W& w, G3Scalars<R>& s) {
  const int ktf = c.nz;
  const R dt = c.dt, g = c.g;
  const int high_resolution = c.cugd_avedx > 1 ? 1 : 0;
  s.psur = f.p8w(i, j, 0) * R(.01);
  s.z1 = f.ht(i, j);
  s.xland = f.xland(i, j);
  s.gsw = f.gsw(i, j);
  s.kpbl = f.kpbl(i, j);
  for (int k = 1; k <= ktf; ++k) {
    const int kk = k - 1;
    w.po[k] = f.p(i, j, kk) * R(.01);
    w.us[k] = f.u(i, j, kk);
    w.vs[k] = f.v(i, j, kk);
    w.t2d[k] = f.t(i, j, kk);
    w.q2d[k] = f.q(i, j, kk);
    if (w.q2d[k] < R(1.E-08)) w.q2d[k] = R(1.E-08);
    w.subt[k] = R(0.);
    w.subq[k] = R(0.);
    w.outt[k] = R(0.);
    w.outq[k] = R(0.);
    w.outqc[k] = R(0.);
    w.outts[k] = R(0.);
    w.outqs[k] = R(0.);
    w.tn[k] = w.t2d[k] + f.rthften(i, j, kk) * dt;
    w.qo[k] = w.q2d[k] + f.rqvften(i, j, kk) * dt;
    w.tshall[k] = w.t2d[k] + f.rthblten(i, j, kk) * f.pi(i, j, kk) * dt;
    w.dhdt[k] = c.cp * f.rthblten(i, j, kk) * f.pi(i, j, kk) + c.xlv * f.rqvblten(i, j, kk);
    w.qshall[k] = w.q2d[k] + f.rqvblten(i, j, kk) * dt;
    if (high_resolution == 1) {
      w.tn[k] = w.t2d[k] + g3_ave9<R>(f, true, i, j, kk) * dt;
      w.qo[k] = w.q2d[k] + g3_ave9<R>(f, false, i, j, kk) * dt;
    }
    if (w.tn[k] < R(200.)) w.tn[k] = w.t2d[k];
    if (w.qo[k] < R(1.E-08)) w.qo[k] = R(1.E-08);
  }
  for (int n = 1; n <= ENSDIM; ++n) {  // xfi_ens, pri_ens, massflni
    w.xf_ens[n] = R(0.);
    w.pr_ens[n] = R(0.);
    w.massfln[n] = R(0.);
  }
  int ens4n = 0;
  for (int nn = -1; nn <= 1; ++nn) {
    int jss = j + nn > 0 ? j + nn : 0;
    jss = jss < c.ny - 1 ? jss : c.ny - 1;
    for (int n = -1; n <= 1; ++n) {
      ens4n = ens4n + 1;
      int iss = i + n > 0 ? i + n : 0;
      iss = iss < c.nx - 1 ? iss : c.nx - 1;
      for (int k = 1; k <= ktf; ++k) {
        const int kk = k - 1;
        const int e = kn(k, ens4n);
        w.omeg[e] = -g * f.rho(i, j, kk) * f.w(iss, jss, kk);
        w.tx[e] = w.t2d[k] + f.rthften(iss, jss, kk) * dt;
        if (high_resolution == 1) w.tx[e] = w.t2d[k] + g3_ave9<R>(f, true, iss, jss, kk) * dt;
        if (w.tx[e] < R(200.)) w.tx[e] = w.t2d[k];
        w.qx[e] = w.q2d[k] + f.rqvften(i, j, kk) * dt;
        if (high_resolution == 1) w.qx[e] = w.q2d[k] + g3_ave9<R>(f, false, iss, jss, kk) * dt;
        if (w.qx[e] < R(1.E-08)) w.qx[e] = R(1.E-08);
      }
    }
  }
  for (int n = 1; n <= ENS4; ++n) {
    s.mconv[n] = R(0.);
    for (int k = 1; k <= ktf - 1; ++k) {
      const R dq = (w.q2d[k + 1] - w.q2d[k]);
      s.mconv[n] = s.mconv[n] + w.omeg[kn(k, n)] * dq / g;
    }
    if (s.mconv[n] < R(0.)) s.mconv[n] = R(0.);
  }
}

// G3DRV's results for an interior column (ibegc .. iendc, jbegc .. jendc)
template <typename R>
struct G3Out {
  R raincv, pratec, htop, hbot, xmb_shallow, edt_out;
  int k22_shallow, kbcon_shallow, ktop_shallow, ktop_deep;
};

// G3DRV for one interior column: the gather, CUP_enss_3d and the copies into the column's outputs; the 3D results
// go to the arrays of out3 (O: CPPWRF_HD set(name, k, value) members, k = 0 .. nz-1)
template <typename R, class F, class W>
CPPWRF_HD inline void g3_column(const G3Call<R>& c, const F& f, int i, int j, W& w, G3Scalars<R>& s, G3Out<R>& out) {
  const int ktf = c.nz;
  g3_gather<R>(c, f, i, j, w, s);
  s.edt_out = R(0.);
  G3Opts o{c.ichoice, c.ishallow, c.cugd_avedx > 1 ? 1 : 0};
  cup_enss_3d<R>(w, s, o, c.dt, c.dx / R(25.), c.xlv, c.r_v, c.cp, c.g, ktf);
  out.xmb_shallow = s.xmb3;
  out.k22_shallow = s.k23;
  out.kbcon_shallow = s.kbcon3;
  out.ktop_shallow = s.ktop3;
  out.ktop_deep = s.ktop;
  out.raincv = R(0.);
  out.pratec = R(0.);
  out.htop = R(1);  // REAL(KTS)
  out.hbot = R(ktf);
  out.edt_out = s.edt_out;
  if (s.pre > R(0.)) {
    out.raincv = s.pre * c.dt;
    out.pratec = s.pre;
    if (R(s.ktop) > out.htop) out.htop = R(s.ktop) + R(.001);
    if (R(s.kbcon) < out.hbot) out.hbot = R(s.kbcon) + R(.001);
  }
}

// ------------------------------------------------------------------------------------------------ CONV_GRELL_SPREAD3D
// for an output column (i, j) inside ids+4 .. ide-5, jds+4 .. jde-5 (the other columns get RTHCUTEN = RQVCUTEN = 0
// and keep RAINCV / PRATEC). S provides tten / qvten / ttens / qvtens (i,j,k), qv(i,j,k), pi(i,j,k); the column's
// rthcuten / rqvcuten / rqccuten / rqicuten (1-based A arrays) are written, raincv / pratec scaled.
template <typename R, class S, class A>
CPPWRF_HD inline void g3_spread(const S& f, int i, int j, int nx, int ny, int nz, int cugd_avedx, R dt, A rth, A rqv,
                                A rqc, A rqi, bool f_qc, bool f_qi, R& raincv, R& pratec, A trasht, A trashq) {
  const int cugd_spread = cugd_avedx / 2;
  const int kts = 1, kte = nz;
  // RTHcutent / RQVcutent at (a, b), level k (0-based kk)
  auto cut = [&](bool th, int a, int b, int kk) {
    R x = th ? f.tten(a, b, kk) : f.qvten(a, b, kk);
    if (cugd_spread > 0) {
      for (int nn = -1; nn <= 1; ++nn) {
        int jdo = b + nn > 0 ? b + nn : 0;
        jdo = jdo < ny - 1 ? jdo : ny - 1;
        for (int kq = -1; kq <= 1; ++kq) {
          int ido = a + kq > 0 ? a + kq : 0;
          ido = ido < nx - 1 ? ido : nx - 1;
          x = x + R(1.) * (th ? f.ttens(ido, jdo, kk) : f.qvtens(ido, jdo, kk));
        }
      }
    } else {
      x = x + (th ? f.ttens(a, b, kk) : f.qvtens(a, b, kk));
    }
    return x;
  };
  // smoothh = 1
  for (int k = kts; k <= kte; ++k) {
    const int kk = k - 1;
    R smt[3], smq[3];
    for (int d = -1; d <= 1; ++d) {
      smt[d + 1] = R(.25) * (cut(true, i - 1, j + d, kk) + R(2.) * cut(true, i, j + d, kk) + cut(true, i + 1, j + d, kk));
      smq[d + 1] =
          R(.25) * (cut(false, i - 1, j + d, kk) + R(2.) * cut(false, i, j + d, kk) + cut(false, i + 1, j + d, kk));
    }
    rth[k] = R(.25) * (smt[0] + R(2.) * smt[1] + smt[2]);
    rqv[k] = R(.25) * (smq[0] + R(2.) * smq[1] + smq[2]);
  }
  R qmemf = R(1.);
  R thresh = R(1.e-20);
  for (int k = kts; k <= kte; ++k) {
    if (rqv[k] < R(0.)) {
      R qmem1 = f.qv(i, j, k - 1) + rqv[k] * dt;
      if (qmem1 < thresh) {
        qmem1 = rqv[k];
        const R qmem2 = (thresh - f.qv(i, j, k - 1)) / dt;
        qmemf = c_min(qmemf, qmem2 / qmem1);
        qmemf = c_max(R(0.), qmemf);
        qmemf = c_min(R(1.), qmemf);
      }
    }
  }
  for (int k = kts; k <= kte; ++k) {
    rqv[k] = rqv[k] * qmemf;
    rth[k] = rth[k] * qmemf;
  }
  if (f_qc)
    for (int k = kts; k <= kte; ++k) rqc[k] = rqc[k] * qmemf;
  if (f_qi)
    for (int k = kts; k <= kte; ++k) rqi[k] = rqi[k] * qmemf;
  raincv = raincv * qmemf;
  pratec = pratec * qmemf;
  thresh = R(200.);
  qmemf = R(1.);
  for (int k = kts; k <= kte; ++k) {
    const R qmem1 = std::fabs(rth[k]) * R(86400.);
    if (qmem1 > thresh) {
      const R qmem2 = thresh / qmem1;
      qmemf = c_min(qmemf, qmem2);
      qmemf = c_max(R(0.), qmemf);
    }
  }
  raincv = raincv * qmemf;
  pratec = pratec * qmemf;
  for (int k = kts; k <= kte; ++k) {
    rqv[k] = rqv[k] * qmemf;
    rth[k] = rth[k] * qmemf;
  }
  if (f_qc)
    for (int k = kts; k <= kte; ++k) rqc[k] = rqc[k] * qmemf;
  if (f_qi)
    for (int k = kts; k <= kte; ++k) rqi[k] = rqi[k] * qmemf;
  // smoothv = 1
  for (int k = kts + 2; k <= kte - 2; ++k) {
    trasht[k] = R(.25) * (rth[k - 1] + R(2.) * rth[k] + rth[k + 1]);
    trashq[k] = R(.25) * (rqv[k - 1] + R(2.) * rqv[k] + rqv[k + 1]);
  }
  for (int k = kts + 2; k <= kte - 2; ++k) {
    rth[k] = trasht[k];
    rqv[k] = trashq[k];
  }
  for (int k = kts; k <= kte; ++k) rth[k] = rth[k] / f.pi(i, j, k - 1);
}

// ------------------------------------------------------------------------------------------------ G3DRV per column
// G3DRV's writes for column (i, j). O provides the 3D outputs cugd_tten, cugd_qvten, cugd_ttens, cugd_qvtens,
// cugd_qcten, rqccuten, rqicuten, gdc, gdc2 ((i,j,k), k = 0 .. nz-1), the 2D raincv, pratec, htop, hbot and
// extra(i, j, scalars, out, work) for the results the model does not keep (APR_*, XF_ENS, PR_ENS, EDT_OUT, the
// shallow levels; WRF writes them for the restart file only). Columns outside ibegc .. iendc, jbegc .. jendc (4 rows
// from a non-periodic boundary) keep G3DRV's initial values.
template <typename R, class F, class O, class W>
CPPWRF_HD inline void g3drv(const G3Call<R>& c, const F& f, const O& o, int i, int j, W& w) {
  const int ktf = c.nz;
  const bool inside = (c.periodic_x || (i >= 4 && i <= c.nx - 5)) && (c.periodic_y || (j >= 4 && j <= c.ny - 5));
  if (!inside) {
    for (int k = 0; k < c.nz; ++k) {
      o.cugd_tten(i, j, k) = R(0.);
      o.cugd_ttens(i, j, k) = R(0.);
      o.cugd_qvten(i, j, k) = R(0.);
      o.cugd_qvtens(i, j, k) = R(0.);
      o.cugd_qcten(i, j, k) = R(0.);
    }
    o.raincv(i, j) = R(0.);
    o.pratec(i, j) = R(0.);
    o.htop(i, j) = R(1);
    o.hbot(i, j) = R(c.nz);
    return;
  }
  G3Scalars<R> s;
  for (int n = 0; n < 8; ++n) s.apr[n] = o.apr_in(i, j, n);
  G3Out<R> out;
  g3_column<R>(c, f, i, j, w, s, out);
  const R subcenter = R(0.);
  R sub_spread = c_max(R(1.), R(c.cugd_avedx * c.cugd_avedx - 1));
  sub_spread = (R(1.) - subcenter) / sub_spread;
  const R cuten = s.pre > R(0.) ? R(1.) : R(0.);
  for (int k = 1; k <= ktf; ++k) {
    o.cugd_ttens(i, j, k - 1) = w.subt[k] * cuten * sub_spread;
    o.cugd_qvtens(i, j, k - 1) = w.subq[k] * cuten * sub_spread;
    o.cugd_tten(i, j, k - 1) = w.outts[k] + w.outt[k] * cuten;
    o.cugd_qvten(i, j, k - 1) = w.outqs[k] + w.outq[k] * cuten;
    o.cugd_qcten(i, j, k - 1) = w.outqc[k] * cuten;
  }
  o.raincv(i, j) = out.raincv;
  o.pratec(i, j) = out.pratec;
  o.htop(i, j) = out.htop;
  o.hbot(i, j) = out.hbot;
  if (c.f_qc)
    for (int k = 1; k <= ktf; ++k) {
      o.rqccuten(i, j, k - 1) = w.outqc[k] * cuten;
      o.gdc(i, j, k - 1) = w.cupclw[k] * cuten;
      o.gdc2(i, j, k - 1) = R(0.);
    }
  if (c.f_qi)
    for (int k = 1; k <= ktf; ++k) {
      if (w.t2d[k] < R(258.)) {
        o.rqicuten(i, j, k - 1) = w.outqc[k] * cuten;
        o.cugd_qcten(i, j, k - 1) = R(0.);
        o.rqccuten(i, j, k - 1) = R(0.);
        o.gdc2(i, j, k - 1) = w.cupclw[k] * cuten;
      } else {
        o.rqicuten(i, j, k - 1) = R(0.);
        o.rqccuten(i, j, k - 1) = w.outqc[k] * cuten;
        o.gdc(i, j, k - 1) = w.cupclw[k] * cuten;
      }
    }
  o.extra(i, j, s, out, w);
}

// CONV_GRELL_SPREAD3D's column arrays
#define CPPWRF_G3_SPREAD_ARRAYS(X) \
  X(R, rth, ::cppwrf::phys::cu_g3::KP) X(R, rqv, ::cppwrf::phys::cu_g3::KP) X(R, rqc, ::cppwrf::phys::cu_g3::KP) X(R, rqi, ::cppwrf::phys::cu_g3::KP) X(R, trasht, ::cppwrf::phys::cu_g3::KP) X(R, trashq, ::cppwrf::phys::cu_g3::KP)
template <typename R>
struct G3SpreadWork {
  CPPWRF_G3_SPREAD_ARRAYS(CPPWRF_COLUMN_MEMBER)
};
template <typename R>
inline constexpr std::size_t kSpreadWorkBytes = 0 CPPWRF_G3_SPREAD_ARRAYS(CPPWRF_COLUMN_BYTES);

// CONV_GRELL_SPREAD3D for column (i, j): S as g3_spread's, O provides rthcuten, rqvcuten, rqccuten, rqicuten
// ((i,j,k)) and raincv, pratec ((i,j)); the columns outside ids+4 .. ide-5, jds+4 .. jde-5 (also when periodic)
// get RTHCUTEN = RQVCUTEN = 0
template <typename R, class S, class O, class W>
CPPWRF_HD inline void g3_spread_column(const G3Call<R>& c, const S& f, const O& o, int i, int j, W& w) {
  const int nz = c.nz;
  if (!(i >= 4 && i <= c.nx - 5 && j >= 4 && j <= c.ny - 5)) {
    for (int k = 0; k < nz; ++k) {
      o.rthcuten(i, j, k) = R(0.);
      o.rqvcuten(i, j, k) = R(0.);
    }
    return;
  }
  for (int k = 1; k <= nz; ++k) {
    w.rqc[k] = c.f_qc ? o.rqccuten(i, j, k - 1) : R(0.);
    w.rqi[k] = c.f_qi ? o.rqicuten(i, j, k - 1) : R(0.);
  }
  R raincv = o.raincv(i, j), pratec = o.pratec(i, j);
  g3_spread<R>(f, i, j, c.nx, c.ny, nz, c.cugd_avedx, c.dt, w.rth, w.rqv, w.rqc, w.rqi, c.f_qc, c.f_qi, raincv,
               pratec, w.trasht, w.trashq);
  for (int k = 1; k <= nz; ++k) {
    o.rthcuten(i, j, k - 1) = w.rth[k];
    o.rqvcuten(i, j, k - 1) = w.rqv[k];
    if (c.f_qc) o.rqccuten(i, j, k - 1) = w.rqc[k];
    if (c.f_qi) o.rqicuten(i, j, k - 1) = w.rqi[k];
  }
  o.raincv(i, j) = raincv;
  o.pratec(i, j) = pratec;
}

}  // namespace cppwrf::phys::cu_g3
