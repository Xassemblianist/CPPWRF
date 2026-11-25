// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Port of WRF 4.8.0 phys/module_bl_mfshconvpbl.F: the EDKF mass-flux shallow convection of the QNSE PBL
// (mfshconv = 1 with bl_pbl_physics = 4), called by module_pbl_driver.F before QNSEPBL. MFSHCONVPBL ->
// MFSHCONVPBL_CORE: COMPUTE_FUNCTION_THERMO_MF / THL_RT_FROM_TH_R_MF (theta_l, r_t), UPDRAFT_SOPE -> COMPUTE_UPDRAFT
// (the dry / cloudy updraft from level 1 with COMPUTE_BL89_ML's upward length, COMPUTE_ENTR_DETR, TH_R_FROM_THL_RT),
// MF_TURB (the explicit TRIDIAG_MASSFLUX of theta_l and r_t) and COMPUTE_MF_CLOUD (RC_MF). WRF's MFSHCONVPBL assigns
// RTHBLTEN = the theta_l tendency, RQVBLTEN = the r_t tendency and RUBLTEN = RVBLTEN = RQCBLTEN = 0 (OMIXUV false,
// PRRS(:,:,:,2) = 0); QNSEPBL then adds its own. RC_MF (Registry `r`) is the only state.
//
// What the port leaves out, because no output reads it: BL89 (PLM -> v_phytmp, not read by QNSEPBL), the theta_v
// flux (WTHV -> u_phytmp, not read), the dead PTH / PRRS lines of MFSHCONVPBL_CORE, COMPUTE_FUNCTION_THERMO_MF's
// AMOIST / ATHETA and the EDKF diagnostics without history (MASSFLUX_EDKF ... FRAC_UP are Registry `-`; the column
// test reads EMF, ENTR, THL_UP, RT_UP and FRAC_UP through the optional views).
//
// WRF works on the 2-D arrays (I*J, K) of the whole tile. Two things of the tile reach a column:
//  * gfortran vectorises TH_R_FROM_THL_RT_1D / _2D's exner (PP/XP00)**(XRD/XCPD) over the tile (libmvec's
//    _ZGVbN2vv_pow for the pairs, the scalar pow for an odd last point: vm::pow with the point's index i + j * nx
//    in the tile and the tile's point count). Every other exp / log / pow of the module that an output reads is the
//    scalar libm (objdump of the -r8 build's module_bl_mfshconvpbl.o: the vector calls are in BL89,
//    COMPUTE_FUNCTION_THERMO_MF's ZE and MFSHCONVPBL_CORE's ZEXN, all dead).
//  * The level loops stop for the whole tile when no column is left (COUNT(GTEST) == 0 in COMPUTE_UPDRAFT, ZTESTM
//    in COMPUTE_BL89_ML). For a column that has stopped, the remaining iterations change only DETR, RC_UP and RV_UP
//    at the levels above its top (no history; RC_UP is read by COMPUTE_MF_CLOUD there with FRAC_UP = 0), so a column
//    here stops at its own top.
// COMPUTE_ENTR_DETR's GTEST_LOCAL_LCL2 is not initialised in WRF; where the WHERE of its line 935 does not set it,
// the only use (line 997) adds (1 - ZPART_DRY) * ... with ZPART_DRY = 1, i.e. +0: false here.

#include "phys/bl_myj/myjpbl.hpp"

namespace cppwrf::phys::mfshconv {

using myjpbl::MyjCPtr;
using myjpbl::MyjPtr;
using myjpbl::c_abs;
static constexpr int kMaxLevels = myjpbl::kMaxLevels;

// module_bl_mfshconvpbl.F:8-62 (REAL PARAMETERs: real*8 with -r8, folded by gfortran as here)
static constexpr real XG = real(9.80665);
static constexpr real XP00 = real(1.E5);
static constexpr real XRD = wrfc::r_d;
static constexpr real XRV = wrfc::r_v;
static constexpr real XCPD = real(7.) * XRD / real(2.);
static constexpr real XCPV = real(4.) * XRV;
static constexpr real XCL = real(4.218E+3);
static constexpr real XCI = real(2.106E+3);
static constexpr real XTT = real(273.16);
static constexpr real XLVTT = real(2.5008E+6);
static constexpr real XLSTT = real(2.8345E+6);
static constexpr real XGAMW = (XCL - XCPV) / XRV;
static constexpr real XBETAW = (XLVTT / XRV) + (XGAMW * XTT);
static constexpr real LOG_611_14 = real(6.415326);
static constexpr real LOG_XTT = real(5.610058);
static constexpr real XALPW = LOG_611_14 + (XBETAW / XTT) + (XGAMW * LOG_XTT);
static constexpr real XALP_PERT = real(0.3);
static constexpr real XABUO = real(1.);
static constexpr real XBENTR = real(1.);
static constexpr real XBDETR = real(0.);
static constexpr real XCMF = real(0.065);
static constexpr real XENTR_DRY = real(0.55);
static constexpr real XDETR_DRY = real(10.);
static constexpr real XDETR_LUP = real(1.0);
static constexpr real XENTR_MF = real(0.035);
static constexpr real XCRAD_MF = real(50.);
static constexpr real XKCF_MF = real(2.75);
static constexpr real XFRAC_UP_MAX = real(0.33);
static constexpr real ZRVORD = XRV / XRD;
static constexpr real XRDCP = XRD / XCPD;

// The kernel's column arrays (1-based as in the Fortran, levels 1 .. nz + 1): in the column work buffer in CUDA FP64
// and FP32 identity builds, local arrays otherwise (as MYJ's, myjpbl.hpp)
#define CPPWRF_MF_COLUMN(X) \
  X(real, zz, kMaxLevels + 2) X(real, dzz, kMaxLevels + 2) X(real, rhodj, kMaxLevels + 2) \
  X(real, rhodref, kMaxLevels + 2) X(real, pabs, kMaxLevels + 2) X(real, thm, kMaxLevels + 2) \
  X(real, rvm, kMaxLevels + 2) X(real, rtm, kMaxLevels + 2) X(real, thlm, kMaxLevels + 2) \
  X(real, thvm, kMaxLevels + 2) X(real, thm_f, kMaxLevels + 2) X(real, thlm_f, kMaxLevels + 2) \
  X(real, rtm_f, kMaxLevels + 2) X(real, tkem_f, kMaxLevels + 2) X(real, pres_f, kMaxLevels + 2) \
  X(real, rho_f, kMaxLevels + 2) X(real, rvm_f, kMaxLevels + 2) X(real, thvm_f, kMaxLevels + 2) \
  X(real, thl_up, kMaxLevels + 2) X(real, rt_up, kMaxLevels + 2) X(real, rc_up, kMaxLevels + 2) \
  X(real, w_up2, kMaxLevels + 2) X(real, emf, kMaxLevels + 2) X(real, frac, kMaxLevels + 2) \
  X(real, entr, kMaxLevels + 2) X(real, detr, kMaxLevels + 2) X(real, mzmr, kMaxLevels + 2) \
  X(real, rdf, kMaxLevels + 2) X(real, flx, kMaxLevels + 2) X(real, varp, kMaxLevels + 2)

// Fortran SIGN(0.5, x) (gfortran: copysign, -0 negative)
CPPWRF_HD inline real sign_half(real x) { return std::signbit(x) ? real(-0.5) : real(0.5); }

// ----------------------------------------------------------------------
// TH_R_FROM_THL_RT_1D / _2D (module_bl_mfshconvpbl.F:1847-2044, the same statements) for one point: theta, r_v and
// r_c of theta_l and r_t at pressure pp (20 iterations of the all-or-nothing adjustment; the ice fraction 0, warm
// clouds only). The exner is libmvec's lane for point lane_i of lane_n (vectorised loop), the rest scalar.
// ----------------------------------------------------------------------
struct ThR {
  real th, rv, rl;
};
CPPWRF_HD inline ThR th_r_from_thl_rt(real pp, real pthl, real prt, int lane_i, int lane_n) {
  const real zcoef = real(0.8);
  real prl = real(0.), pri = real(0.), prv = prt, pth = pthl;
  const real zexn = vm::pow(pp / XP00, XRDCP, lane_i, lane_n);
  for (int ii = 1; ii <= 20; ++ii) {
    const real zt = pth * zexn;
    if (zt > real(273.15)) {
      const real zfoes = rexp(XALPW - XBETAW / zt - XGAMW * rlog(zt));
      const real zqvsat = XRD / XRV * zfoes / pp / (real(1.) + (XRD / XRV - real(1.)) * zfoes / pp);
      const real zrvsat = (real(1) - zcoef) * zqvsat * (real(1) + prt) + (zcoef)*prv;
      const real pfrac_ice = real(0.);
      const real zrltemp = c_max(real(0.), prv - zrvsat);
      prv = prv - zrltemp;
      prl = prl + pri + zrltemp;
      pri = pfrac_ice * (prl);
      prl = (real(1) - pfrac_ice) * (prt - prv);
      const real zcph = XCPD + XCPV * prv + XCL * prl + XCI * pri;
      const real zlovcpexn = (XLVTT + (XCPV - XCL) * (zt - XTT)) / (zcph * zexn);
      const real zloscpexn = (XLSTT + (XCPV - XCI) * (zt - XTT)) / (zcph * zexn);
      pth = pthl + zlovcpexn * prl + zloscpexn * pri;
    } else {  // cold shallow cloud not coded
      prl = real(0.);
      pri = real(0.);
      prv = prt;
      pth = pthl;
    }
  }
  return {pth, prv, prl};
}

// ----------------------------------------------------------------------
// COMPUTE_BL89_ML (module_bl_mfshconvpbl.F:476-619), the upward length from level kk = 1 as COMPUTE_UPDRAFT calls it
// (PTKEM2D(:,KK) = ZTKEM_F(:,KK) = 0): the particle of theta_v vpt[1] rises while its energy ZINTE is >= 0. WRF goes
// on while any column of the tile does; the iterations of a column with ZINTE < 0 add ZTEST0 (= 0) * finite = +-0
// and subtract +-0 from ZINTE (never -0), so the column stops here.
// ----------------------------------------------------------------------
CPPWRF_HD inline real bl89_up_from_ground(int iku, MyjCPtr dzz, real g_o_thvref, MyjCPtr vpt, real zinte0) {
  const int ike = iku - 1;
  real zinte = zinte0, plwork = real(0.);
  const real zth = vpt[1];
  for (int jkk = 2; jkk <= ike; ++jkk) {
    const real ztest0 = real(0.5) + sign_half(zinte);
    if (ztest0 == real(0.)) break;
    real zdeltvpt = vpt[jkk] - vpt[jkk - 1];
    if (c_abs(zdeltvpt) < real(1.E-10)) zdeltvpt = real(1.E-10);
    const real zhlvpt = real(0.5) * (vpt[jkk] + vpt[jkk - 1]);
    const real zpote = ztest0 * (g_o_thvref * (zhlvpt - zth)) * dzz[jkk];  // particle keeps its temperature
    const real ztest = real(0.5) + sign_half(zinte - zpote);
    const real zlwork1 = dzz[jkk];
    const real a = g_o_thvref * (vpt[jkk - 1] - zth);
    const real zlwork2 = (-(g_o_thvref * (vpt[jkk - 1] - zth)) +
                          std::sqrt(c_abs(a * a + real(2.) * zinte * g_o_thvref * zdeltvpt / dzz[jkk]))) /
                         (g_o_thvref * zdeltvpt / dzz[jkk]);
    plwork = plwork + ztest0 * (ztest * zlwork1 + (real(1) - ztest) * zlwork2);
    zinte = zinte - zpote;
  }
  return plwork;
}

// ----------------------------------------------------------------------
// TRIDIAG_MASSFLUX (module_bl_mfshconvpbl.F:2116-2272) with PIMPL = 0 (PIMPL_MF: the explicit branch), PDFDT = -emf:
// varp[1 .. iku] (the PIMPL terms are kept: they add +-0)
// ----------------------------------------------------------------------
CPPWRF_HD inline void tridiag_massflux(int iku, MyjCPtr varm, MyjCPtr f, MyjCPtr emf, real tstep, real pimpl,
                                       MyjCPtr dzz, MyjCPtr rhodj, MyjCPtr mzmr, MyjPtr rdf, MyjPtr varp) {
  const int ikb = 1, ike = iku - 1;
  for (int k = 1; k <= iku; ++k) rdf[k] = mzmr[k] * (-emf[k]) / dzz[k];  // ZRHODJ_DFDT_O_DZ
  varp[ikb] = (rhodj[ikb] * varm[ikb] / tstep - mzmr[ikb + 1] * f[ikb + 1] / dzz[ikb + 1] +
               mzmr[ikb] * f[ikb] / dzz[ikb] + rdf[ikb + 1] * real(0.5) * pimpl * varm[ikb + 1] +
               rdf[ikb + 1] * real(0.5) * pimpl * varm[ikb]) *
              tstep / rhodj[ikb];
  for (int jk = ikb + 1; jk <= ike - 1; ++jk)
    varp[jk] = (rhodj[jk] * varm[jk] / tstep - mzmr[jk + 1] * f[jk + 1] / dzz[jk + 1] + mzmr[jk] * f[jk] / dzz[jk] +
                rdf[jk + 1] * real(0.5) * pimpl * varm[jk + 1] + rdf[jk + 1] * real(0.5) * pimpl * varm[jk] -
                rdf[jk] * real(0.5) * pimpl * varm[jk] - rdf[jk] * real(0.5) * pimpl * varm[jk - 1]) *
               tstep / rhodj[jk];
  varp[ike] = (rhodj[ike] * varm[ike] / tstep - mzmr[ike + 1] * f[ike + 1] / dzz[ike + 1] +
               mzmr[ike] * f[ike] / dzz[ike] - rdf[ike] * real(0.5) * pimpl * varm[ike] -
               rdf[ike] * real(0.5) * pimpl * varm[ike - 1]) *
              tstep / rhodj[ike];
  varp[ike + 1] = varp[ike];
}

struct MFSHCONVKernel {
  View3<const real> dz, rho, pmid, th, exner, qv, tke;  // DZ = dz8w, RHO, PMID = p_phy, TH = th_phy, EXNER = pi_phy
  View2<const real> hfx, qfx;
  View3<real> rc_mf;               // inout (state)
  View3<real> rthblten, rqvblten;  // assigned (RUBLTEN, RVBLTEN, RQCBLTEN = 0: the driver's zeros)
  View3<real> emf_d, entr_d, thl_up_d, rt_up_d, frac_up_d;  // optional (p null: absent), the column test's
  real dt;
  int stepbl;
  int nz;
  int tile_i0 = 0, tile_j0 = 0, tile_nx = 1, tile_n = 1;  // WRF's tile (its..ite x jts..jte): the libmvec lanes
#if defined(CPPWRF_LOCAL_ROUNDING)
  ColumnWork work{};  // the GPU's column arrays (column_for_work)
  static constexpr std::size_t kWorkBytes = 0 CPPWRF_MF_COLUMN(CPPWRF_COLUMN_BYTES);
#endif

  CPPWRF_HD void operator()(int i, int j) const {
    const int iku = nz, ike = nz - 1;
    const int li = (i - tile_i0) + (j - tile_j0) * tile_nx, ln = tile_n;
    const real ptstep = dt * real(stepbl);  // PTSTEP = PTSTEP_MET = DT * STEPBL

    CPPWRF_MYJ_SLOTS(work, i, j)
    CPPWRF_MF_COLUMN(CPPWRF_MYJ_ARRAY)

    // ---- MFSHCONVPBL (lines 170-227): heights, the conservative variables of MFSHCONVPBL_CORE (396-413) ----
    zz[1] = real(0.);
    real zzm_prev = real(0.);
    for (int k = 1; k <= iku; ++k) {
      const int kw = k - 1;
      real zzm;
      if (k != iku) {
        zz[k + 1] = zz[k] + dz(i, j, kw);
        zzm = real(0.5) * (zz[k + 1] + zz[k]);  // z at mass point
      } else {
        zzm = zz[k] + real(0.5) * dz(i, j, kw - 1);
      }
      dzz[k] = k == 1 ? real(2) * (zzm) : zzm - zzm_prev;
      zzm_prev = zzm;
    }
    for (int k = 1; k <= iku; ++k) {
      const int kw = k == iku ? iku - 2 : k - 1;  // fill the kte+1 level: PxxM(:,:,KTE) = PxxM(:,:,KTE-1)
      const real qvk = qv(i, j, kw), rck = rc_mf(i, j, kw);
      const real rv = qvk - rck;  // PRM(:,:,:,1)
      thm[k] = th(i, j, kw);
      rvm[k] = rv;
      rhodref[k] = rho(i, j, kw) / (real(1.) + qvk);
      pabs[k] = pmid(i, j, kw);
      rhodj[k] = rhodref[k] * dz(i, j, kw);
      tkem_f[k] = tke(i, j, kw);  // PTKEM (averaged to the flux points below)
      const real pexn = exner(i, j, kw);
      // COMPUTE_FUNCTION_THERMO_MF (KRRL = 1, KRRI = 0) and THL_RT_FROM_TH_R_MF
      real zcp = XCPD;
      zcp = zcp + XCPV * rv;
      zcp = zcp + XCL * rck;
      const real zt = thm[k] * pexn;
      real lv = (XLVTT + (XCPV - XCL) * (zt - XTT)) / zcp;
      lv = lv / pexn;
      rtm[k] = rv + rck;
      thlm[k] = thm[k] - lv * rck;
    }
    const real psfth = hfx(i, j) / (rhodref[1] * XCPD);
    const real psfrv = qfx(i, j) / (rhodref[1]);

    // ---- COMPUTE_UPDRAFT (lines 1306-1619), KK = IKB = 1 ----
    int kklcl = ike, kkctl = ike;
    for (int k = 1; k <= iku; ++k) {
      rc_up[k] = real(0.); w_up2[k] = real(0.); emf[k] = real(0.); detr[k] = real(0.); entr[k] = real(0.);
      frac[k] = real(0.);
    }
    // environment at the flux points (the TKE, read from tkem_f, is averaged from the top down)
    for (int k = iku; k >= 2; --k) {
      thm_f[k] = real(0.5) * (thm[k - 1] + thm[k]);
      thlm_f[k] = real(0.5) * (thlm[k - 1] + thlm[k]);
      rtm_f[k] = real(0.5) * (rtm[k - 1] + rtm[k]);
      tkem_f[k] = real(0.5) * (tkem_f[k - 1] + tkem_f[k]);
      pres_f[k] = real(0.5) * (pabs[k - 1] + pabs[k]);
      rho_f[k] = real(0.5) * (rhodref[k - 1] + rhodref[k]);
      rvm_f[k] = real(0.5) * (rvm[k - 1] + rvm[k]);
    }
    thm_f[1] = thm[1]; thlm_f[1] = thlm[1]; rtm_f[1] = rtm[1]; pres_f[1] = pabs[1]; rho_f[1] = rhodref[1];
    rvm_f[1] = rvm[1];  // tkem_f[1] = PTKEM(:,1) already
    for (int k = 1; k <= iku; ++k) {
      thvm_f[k] = thm_f[k] * ((real(1.) + ZRVORD * rvm_f[k]) / (real(1.) + rtm_f[k]));
      thvm[k] = thm[k] * ((real(1.) + ZRVORD * rvm[k]) / (real(1.) + rtm[k]));
      thl_up[k] = thlm_f[k];
      rt_up[k] = rtm_f[k];
    }
    const real ztmax = real(2.0), zrmax = real(1.E-3);
    thl_up[1] = thlm_f[1] + c_max(real(0.), c_min(ztmax, (psfth / std::sqrt(tkem_f[1])) * XALP_PERT));
    rt_up[1] = rtm_f[1] + c_max(real(0.), c_min(zrmax, (psfrv / std::sqrt(tkem_f[1])) * XALP_PERT));
    w_up2[1] = c_max(real(0.0001), (real(2.) / real(3.)) * tkem_f[1]);
    rc_up[1] = th_r_from_thl_rt(pres_f[1], thl_up[1], rt_up[1], li, ln).rl;
    // closure: L_up from the ground (ZTKEM_F(:,KK) = 0), the surface buoyancy flux, the mass flux at level 1
    const real g1 = XG / thvm_f[1];
    const real zlup = c_max(bl89_up_from_ground(iku, dzz, g1, thvm_f, real(0.)), real(1.E-10));
    const real zwthvsurf = (thvm_f[1] / thm_f[1]) * psfth + (real(0.61) * thm_f[1]) * psfrv;
    bool gtest = false, gtestlcl = false;
    if (zwthvsurf > real(0.)) {
      emf[1] = XCMF * rho_f[1] * rpow((g1)*zwthvsurf * zlup, real(1.) / real(3.));
      frac[1] = c_min(emf[1] / (std::sqrt(w_up2[1]) * rho_f[1]), XFRAC_UP_MAX);
      const real w = emf[1] / (frac[1] * rho_f[1]);
      w_up2[1] = w * w;
      gtest = true;
    }
    for (int jk = 1; jk <= ike - 1 && gtest; ++jk) {
      if (rc_up[jk] > real(0.) && !gtestlcl) {  // the LCL
        kklcl = jk;
        gtestlcl = true;
      }
      real zbuo_integ;
      entr_detr(gtest, gtestlcl, jk, li, ln, zz, dzz, pabs, thvm, thlm[jk], rtm[jk], w_up2, thl_up[jk], rt_up[jk],
                zlup, entr[jk], detr[jk], zbuo_integ);
      if (jk == 1) detr[jk] = real(0.);
      // the updraft at level jk + 1
      if (gtest) {
        const real zmix1 = real(0.5) * (zz[jk + 1] - zz[jk]) * (entr[jk] - detr[jk]);
        emf[jk + 1] = emf[jk] * rexp(real(2) * zmix1);
      }
      if (gtest && emf[jk + 1] <= real(0.)) {  // stop the updraft if MF becomes negative
        emf[jk + 1] = real(0.);
        gtest = false;
        kkctl = jk + 1;
      }
      real zmix2 = real(0.), zmix3 = real(0.);
      if (gtest) {
        zmix2 = (zz[jk + 1] - zz[jk]) * entr[jk];
        zmix3 = (zz[jk + 1] - zz[jk]) * detr[jk];
        thl_up[jk + 1] = (thl_up[jk] * (real(1.) - real(0.5) * zmix2) + thlm[jk] * zmix2) / (real(1.) + real(0.5) * zmix2);
        rt_up[jk + 1] = (rt_up[jk] * (real(1.) - real(0.5) * zmix2) + rtm[jk] * zmix2) / (real(1.) + real(0.5) * zmix2);
      }
      rc_up[jk + 1] = th_r_from_thl_rt(pres_f[jk + 1], thl_up[jk + 1], rt_up[jk + 1], li, ln).rl;
      if (gtest) {
        if (!gtestlcl) {
          if (zbuo_integ > real(0.)) w_up2[jk + 1] = w_up2[jk] + real(2.) * (XABUO - XBENTR * XENTR_DRY) * zbuo_integ;
          if (zbuo_integ <= real(0.)) w_up2[jk + 1] = w_up2[jk] + real(2.) * XABUO * zbuo_integ;
        } else {
          const real s = XBDETR * zmix3 + XBENTR * zmix2;
          w_up2[jk + 1] = w_up2[jk] * (real(1.) - (s)) / (real(1.) + (s)) +
                          real(2.) * (XABUO)*zbuo_integ / (real(1.) + (s));
        }
      }
      const bool gtestetl = gtest && zbuo_integ <= real(0.);  // the ETL
      if (gtest && (w_up2[jk + 1] <= real(0.) || emf[jk + 1] <= real(0.))) {  // the top of the updraft
        w_up2[jk + 1] = real(0.);
        emf[jk + 1] = real(0.);
        gtest = false;
        thl_up[jk + 1] = thlm_f[jk + 1];
        rt_up[jk + 1] = rtm_f[jk + 1];
        rc_up[jk + 1] = real(0.);
        frac[jk + 1] = real(0.);
        kkctl = jk + 1;
      }
      if (gtest) {
        frac[jk + 1] = emf[jk + 1] / (std::sqrt(w_up2[jk + 1]) * rho_f[jk + 1]);
        frac[jk + 1] = c_min(XFRAC_UP_MAX, frac[jk + 1]);
      }
      if (gtest && gtestetl && gtestlcl) frac[jk + 1] = c_min(frac[jk + 1], frac[jk]);  // cloudy, non-buoyant
      emf[jk + 1] = frac[jk + 1] * std::sqrt(w_up2[jk + 1]) * rho_f[jk + 1];
    }
    emf[1] = real(0.);
    emf[iku] = real(0.);
    frac[iku] = real(0.);
    const real zdepth = zz[kkctl] - zz[kklcl];
    if (gtestlcl && zdepth > real(3000.)) {  // too deep clouds
      const real zcoef = c_min(c_max(real(1.) - (zdepth - real(3000.)) / real(1000.), real(0.)), real(1.));
      for (int k = 1; k <= iku; ++k) {
        emf[k] = emf[k] * zcoef;
        frac[k] = frac[k] * zcoef;
      }
    }
    if (emf_d.p) {
      for (int k = 1; k <= iku; ++k) {
        emf_d(i, j, k - 1) = emf[k];
        entr_d(i, j, k - 1) = entr[k];
        thl_up_d(i, j, k - 1) = thl_up[k];
        rt_up_d(i, j, k - 1) = rt_up[k];
        frac_up_d(i, j, k - 1) = frac[k];
      }
    }

    // ---- MF_TURB (lines 1701-1765): theta_l and r_t, flux = (EMF / RHODREF) (X_up - MZM(X)) ----
    for (int k = 1; k <= iku; ++k) emf[k] = emf[k] / rhodref[k];  // ZEMF_O_RHODREF
    for (int k = 2; k <= iku; ++k) mzmr[k] = real(0.5) * (rhodj[k] + rhodj[k - 1]);
    mzmr[1] = rhodj[2];
    const real pimpl = real(0.);
    for (int pass = 0; pass < 2; ++pass) {
      MyjCPtr xm = pass == 0 ? MyjCPtr(thlm) : MyjCPtr(rtm);
      MyjCPtr xu = pass == 0 ? MyjCPtr(thl_up) : MyjCPtr(rt_up);
      for (int k = 2; k <= iku; ++k) flx[k] = emf[k] * (xu[k] - real(0.5) * (xm[k] + xm[k - 1]));
      flx[1] = emf[1] * (xu[1] - xm[2]);
      tridiag_massflux(iku, xm, flx, emf, ptstep, pimpl, dzz, rhodj, mzmr, rdf, varp);
      const View3<real>& ten = pass == 0 ? rthblten : rqvblten;
      for (int k = 1; k <= iku; ++k) ten(i, j, k - 1) = (varp[k] - xm[k]) / ptstep;
    }

    // ---- COMPUTE_MF_CLOUD (lines 2514-2539): RC_MF ----
    for (int k = 1; k <= iku; ++k) {
      real r = real(0.);
      if (k >= kklcl && k <= ike) r = real(0.5) * XKCF_MF * (frac[k] * rc_up[k] + frac[k + 1] * rc_up[k + 1]);
      rc_mf(i, j, k - 1) = r;
    }
  }

  // ----------------------------------------------------------------------
  // COMPUTE_ENTR_DETR (module_bl_mfshconvpbl.F:624-1011) for one column with OTEST true at level kk
  // ----------------------------------------------------------------------
  CPPWRF_HD static void entr_detr(bool& otest, bool otestlcl, int kk, int li, int ln, MyjCPtr zz, MyjCPtr dzz,
                                  MyjCPtr pabs, MyjCPtr thvm, real pthlm, real prtm, MyjPtr w_up2, real pthl_up,
                                  real prt_up, real plup, real& pentr, real& pdetr, real& pbuo_integ) {
    const int ikb = 1;
    const real zg = XG / thvm[kk];
    real zcoeff_qsat = real(0.), zrc_ord = real(0.), zpart_dry = real(1.);
    bool lcl = false, lcl2 = false;
    const real zdz_half = (zz[kk + 1] - zz[kk]) / real(2.);
    real zdz_stop = zdz_half;
    real zkic = real(0.1);
    const real zmixthl = zkic * pthlm + (real(1.) - zkic) * pthl_up;
    const real zmixrt = zkic * prtm + (real(1.) - zkic) * prt_up;
    const real zcopre_minus = kk == ikb ? real(0.) : ((pabs[kk] - pabs[kk - 1]) / dzz[kk]);
    const real zcopre_plus = ((pabs[kk + 1] - pabs[kk]) / dzz[kk + 1]);
    const real zpre_minus =
        kk == ikb ? pabs[kk] : zcopre_minus * real(0.5) * (zz[kk] - zz[kk - 1]) + pabs[kk - 1];
    const real zpre_plus = zcopre_plus * real(0.5) * (zz[kk + 1] - zz[kk]) + pabs[kk];
    ThR m = th_r_from_thl_rt(pabs[kk], zmixthl, zmixrt, li, ln);  // the mixture at mass level kk
    const real zthvmix_1 = m.th * (real(1.) + ZRVORD * m.rv) / (real(1.) + zmixrt);
    m = th_r_from_thl_rt(zpre_plus, zmixthl, zmixrt, li, ln);  // at flux level kk + 1
    const real zthvmix_2 = m.th * (real(1.) + ZRVORD * m.rv) / (real(1.) + zmixrt);
    m = th_r_from_thl_rt(zpre_minus, pthl_up, prt_up, li, ln);  // the updraft at flux level kk
    const real zthv_up_f1 = m.th * (real(1.) + ZRVORD * m.rv) / (real(1.) + prt_up);
    const ThR u = th_r_from_thl_rt(pabs[kk], pthl_up, prt_up, li, ln);  // at mass level kk
    const real zthv_up = u.th * (real(1.) + ZRVORD * u.rv) / (real(1.) + prt_up);
    const ThR f2 = th_r_from_thl_rt(zpre_plus, pthl_up, prt_up, li, ln);  // at flux level kk + 1
    const real zthv_up_f2 = f2.th * (real(1.) + ZRVORD * f2.rv) / (real(1.) + prt_up);
    const real zrcmix = u.rl, zrcmix_f2 = f2.rl;

    // dry part: the LCL between flux level kk and mass level kk
    if (zrcmix > real(0.) && !otestlcl) {
      zcoeff_qsat = (zrcmix_f2 - zrcmix) / zdz_half;
      zrc_ord = zrcmix - zcoeff_qsat * zdz_half;
      zdz_stop = (-zrc_ord / zcoeff_qsat);
      zpart_dry = zdz_stop / (zz[kk + 1] - zz[kk]);
      lcl = true;
    }
    const real zcoeff_minus = kk == ikb ? real(0.) : ((thvm[kk] - thvm[kk - 1]) / dzz[kk]);
    const real zcoeff_plus = ((thvm[kk + 1] - thvm[kk]) / dzz[kk + 1]);
    const real zcothvu_minus = (zthv_up - zthv_up_f1) / zdz_half;
    const real zcothvu_plus = (zthv_up_f2 - zthv_up) / zdz_half;
    const real zthv_minus = kk == ikb ? thvm[kk] : zcoeff_minus * real(0.5) * (zz[kk] - zz[kk - 1]) + thvm[kk - 1];
    const real zthv_plus = zcoeff_plus * real(0.5) * (zz[kk + 1] - zz[kk]) + zthv_minus;  // BUG JP 16082010
    // integral buoyancy between flux level kk and mass level kk
    pbuo_integ = zg * zdz_half * (real(0.5) * (zcothvu_minus - zcoeff_minus) * zdz_half - zthv_minus + zthv_up_f1);
    real zw2_half = real(0.);
    constexpr real AE = XABUO - XBENTR * XENTR_DRY;
    if (otest && !otestlcl) {
      pentr = real(0.);
      pdetr = real(0.);
      const real zbuo = zg * zdz_stop * (real(0.5) * (-zcoeff_minus) * zdz_stop - zthv_minus + zthv_up_f1);
      if (zbuo >= real(0.)) {
        pentr = real(0.5) / (AE)*rlog(real(1.) + (real(2.) * (AE) / w_up2[kk]) * zbuo);
        pdetr = real(0.);
        zw2_half = w_up2[kk] + real(2) * (AE) * (zbuo);
      } else {
        pentr = real(0.);
        pdetr = real(0.5) / (XABUO)*rlog(real(1.) + (real(2.) * (XABUO) / w_up2[kk]) * c_max(real(0.), -zbuo));
        zw2_half = w_up2[kk] + real(2) * (XABUO) * (zbuo);
      }
    }
    zdz_stop = zdz_half;
    // total integral buoyancy between flux levels kk and kk + 1
    pbuo_integ = pbuo_integ + zg * zdz_half * (real(0.5) * (zcothvu_plus - zcoeff_plus) * zdz_half - thvm[kk] + zthv_up);
    if (((zrcmix_f2 > real(0.)) && (zrcmix <= real(0.))) && !otestlcl && !lcl) {
      // the LCL between mass level kk and flux level kk + 1 (scalar pow, exp, log)
      const real pt = f2.th * (rpow(pabs[kk + 1] / XP00, XRDCP));
      real zqvsat = rexp(XALPW - XBETAW / pt - XGAMW * rlog(pt));
      zqvsat = XRD / XRV * zqvsat / pabs[kk + 1] / (real(1.) + (XRD / XRV - real(1.)) * zqvsat / pabs[kk + 1]);
      zcoeff_qsat = (prt_up - zqvsat - zrcmix) / (real(0.5) * (zz[kk + 2] - zz[kk + 1]));
      zrc_ord = zrcmix_f2 - zcoeff_qsat * zdz_half;
      zdz_stop = (-zrc_ord / zcoeff_qsat);
      zpart_dry = real(0.5) + zdz_stop / (zz[kk + 1] - zz[kk]);
      lcl2 = true;
    }
    if ((otest && !otestlcl) && !lcl) {
      const real zbuo = zg * zdz_stop * (real(0.5) * (-zcoeff_plus) * zdz_stop - thvm[kk] + zthv_up);
      if (zw2_half > real(0.)) {
        if (zbuo >= real(0.))
          pentr = pentr + real(0.5) / (AE)*rlog(real(1.) + (real(2.) * (AE) / zw2_half) * zbuo);
        else
          pdetr = pdetr + real(0.5) / (XABUO)*rlog(real(1.) + (real(2.) * (XABUO) / zw2_half) * c_max(-zbuo, real(0.)));
      } else {
        otest = false;  // w**2 < 0: the updraft is stopped
      }
    }
    pentr = XENTR_DRY * pentr / (zz[kk + 1] - zz[kk]);
    pdetr = XDETR_DRY * pdetr / (zz[kk + 1] - zz[kk]);
    pdetr = c_max(zpart_dry * XDETR_LUP / (plup - real(0.5) * (zz[kk] + zz[kk + 1])), pdetr);
    constexpr real COEFFMF = XENTR_MF * XG / XCRAD_MF;
    if (otest && otestlcl) {  // the critical mixed fraction in the cloud
      zkic = c_max(real(0.), zthv_up - thvm[kk]) * zkic / (zthv_up - zthvmix_1 + real(1.E-10));
      zkic = c_max(real(0.), c_min(real(1.), zkic));
      const real zepsi = zkic * zkic, zdelta = (real(1.) - zkic) * (real(1.) - zkic);
      const real zepsi_cloud = c_min(zdelta, zepsi);
      pentr = COEFFMF * zepsi_cloud;
      pdetr = COEFFMF * zdelta;
    }
    if ((otest && !otestlcl) && (lcl || lcl2)) {
      zkic = c_max(real(0.), zthv_up_f2 - zthv_plus) * zkic / (zthv_up_f2 - zthvmix_2 + real(1.E-10));
      zkic = c_max(real(0.), c_min(real(1.), zkic));
      const real zepsi = zkic * zkic, zdelta = (real(1.) - zkic) * (real(1.) - zkic);
      const real zepsi_cloud = c_min(zdelta, zepsi);
      pentr = pentr + (real(1.) - zpart_dry) * COEFFMF * zepsi_cloud;
      pdetr = pdetr + (real(1.) - zpart_dry) * COEFFMF * zdelta;
    }
  }
};

void run_mfshconv(const MFSHCONVKernel& kernel, const Range2D& range);
void mfshconv_reserve(const Range2D& range);

}  // namespace cppwrf::phys::mfshconv
