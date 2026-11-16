// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Port of WRF 4.8.0 phys/module_sf_qnsesfc.F (qnsesfc, sfcdif, qnsesfcinit): the QNSE surface layer
// (sf_sfclay_physics = 4), MYJSFC's scheme with Sukoriansky's QNSE stability functions, PRT0 = 0.72, ZTFC = 0.1, the
// topographic Zilitinkevich factor and THZ0 from the surface heat flux over land. SCM_FORCE_FLUX is 0 (WRF passes
// it; the scm forcing is rejected), so HFX and QFX are always computed.

#include <cmath>
#include "core/backend.hpp"
#include "core/field.hpp"
#include "core/precision.hpp"
#include "phys/driver/wrf_constants.hpp"
#include "phys/sf_myjsfc/myjsfc.hpp"

namespace cppwrf::phys::sf_qnsesfc {

static constexpr int KZTM = sf_myjsfc::KZTM;
static constexpr int KZTM2 = sf_myjsfc::KZTM2;
static constexpr int kMaxLevels = sf_myjsfc::kMaxLevels;
using Tables = sf_myjsfc::MYJSFCTables;  // PSIM1/PSIH1/PSIM2/PSIH2 and their bounds (QNSESFCINIT's values)

// QNSESFCINIT's tables (module_sf_qnsesfc.F:1123-1277) on the device / host
class QNSESFCTableManager {
 public:
  QNSESFCTableManager();
  ~QNSESFCTableManager();
  QNSESFCTableManager(const QNSESFCTableManager&) = delete;
  QNSESFCTableManager& operator=(const QNSESFCTableManager&) = delete;
  Tables get_tables() const { return tables_; }
  // the host copies (column test)
  static void compute(real* psim1, real* psih1, real* psim2, real* psih2, Tables& bounds);

 private:
  void* dev_[4] = {nullptr, nullptr, nullptr, nullptr};
  Tables tables_{};
};

// QNSESFC + SFCDIF (module_sf_qnsesfc.F:68-1006), one column
struct QNSESFCKernel {
  View2<const real> ht;
  View3<const real> dz, pmid, pint, th, t, qv, qc, u, v;
  View3<const real> q2;  // TKE_PBL (Q2 = 2 TKE)
  View2<const real> tsk, xland, z0base, mavail;
  View2<const int> lowlyr;
  View2<real> qsfc, thz0, qz0, uz0, vz0, ustar, znt, pblh, rmol, akhs, akms, hfx, qfx;  // inout
  View2<real> rib, chs, chs2, cqs2, flx_lh, flhc, flqc, qgh, cpm, ct, u10, v10, t02, th02, tshltr, th10, q02,
      qshltr, q10, pshltr, u10e, v10e;
  Tables tables;
  int itimestep;
  int nz;

  CPPWRF_HD real lookup(View1<const real> tb, real zeta, real ztmin, real dzeta) const {
    const real rz = (zeta - ztmin) / dzeta;
    int k = static_cast<int>(rz);
    const real rdzt = rz - real(k);
    k = c_min(k, KZTM2);
    k = c_max(k, 0);
    return (tb(k + 1) - tb(k)) * rdzt + tb(k);
  }

  CPPWRF_HD void operator()(int i, int j) const {
    using sf_myjsfc::c_abs;
    constexpr real G = wrfc::g, R_D = wrfc::r_d, CP = wrfc::cp, R_V = wrfc::r_v, XLV = wrfc::xlv;
    constexpr real EPSQ2L = real(0.01);
    constexpr real VKARMAN = real(0.4);
    constexpr real CAPA = R_D / CP, ELOCP = real(2.72e6) / CP, RCAP = real(1.) / CAPA;
    constexpr real GOCP02 = G / CP * real(2.), GOCP10 = G / CP * real(10.);
    constexpr real EPSU2 = real(1.e-6), EPSUST = real(1.e-9), EPSZT = real(1.e-28);
    constexpr real A2S = real(17.2693882), A3S = real(273.16), A4S = real(35.86);
    constexpr real PQ0 = real(379.90516), SEAFC = real(0.98), PQ0SEA = PQ0 * SEAFC;
    constexpr real CZIL = real(0.1), EXCML = real(0.0001), EXCMS = real(0.0001);
    constexpr real GLKBR = real(10.), GLKBS = real(30.);
    constexpr real QVISC = real(2.1e-5), SMALL = real(0.35), SQPR = real(0.84), SQSC = real(0.84);
    constexpr real SQVISC = real(258.2), TVISC = real(2.1e-5), USTC = real(0.7), USTR = real(0.225);
    constexpr real VISC = real(1.5e-5), FH = real(1.01), WWST = real(1.2), ZTFC = real(0.1);
    constexpr real TOPOFAC = real(9.0e-6);
    constexpr real P608 = R_V / R_D - real(1.);
    constexpr real CZIV = SMALL * GLKBS, GRRS = GLKBR / GLKBS;
    constexpr real RTVISC = real(1.) / TVISC, RVISC = real(1.) / VISC, ZQRZT = SQSC / SQPR;
    constexpr real FZQ1 = RTVISC * QVISC * ZQRZT, FZQ2 = RTVISC * QVISC * ZQRZT;
    constexpr real FZT1 = RVISC * TVISC * SQPR, FZT2 = CZIV * GRRS * TVISC * SQPR, FZU1 = CZIV * VISC;
    constexpr real PRT0 = real(0.72);
    constexpr real RQVISC = real(1.) / QVISC, USTFC = real(0.018) / G, WWST2 = WWST * WWST;
    constexpr real ZILFC = -CZIL * VKARMAN * SQVISC;
    constexpr real A2 = real(17.2693882), A3 = real(273.16), A4 = real(35.86);
    constexpr int ITRMX = 5;

    const real ZTMIN1 = tables.ztmin1, ZTMAX1 = tables.ztmax1, DZETA1 = tables.dzeta1;
    const real ZTMIN2 = tables.ztmin2, ZTMAX2 = tables.ztmax2, DZETA2 = tables.dzeta2;
    const int ntsd = itimestep;

    // QNSESFC (lines 153-292)
    if (ntsd == 1) ustar(i, j) = real(0.1);
    ct(i, j) = real(0.);
    const int llow = lowlyr(i, j);
    const int lmh = nz - llow + 1;
    const int ks = llow - 1;  // the lowest layer (KTE + 1 - LMH flipped back)
    const real psfc = pint(i, j, ks);
    const real apesfc = rpow(psfc * real(1.e-5), CAPA);
    const real thsk = tsk(i, j) / apesfc;  // THSK = TSK/(PSFC*1.E-5)**CAPA (the same pow as APESFC's)
    const real seamask = xland(i, j) - real(1.);

    real zhk[kMaxLevels + 2];
    zhk[nz + 1] = ht(i, j);
    for (int kf = nz; kf >= 1; --kf) zhk[kf] = zhk[kf + 1] + dz(i, j, nz - kf);

    int lpbl = 1;
    for (int kf = lmh - 1; kf >= 1; --kf) {
      if (real(2.) * q2(i, j, nz - kf) <= EPSQ2L * FH) {
        lpbl = kf;
        break;
      }
    }
    const real pblh_v = zhk[lpbl] - zhk[lmh + 1];
    pblh(i, j) = pblh_v;

    const real plow = pmid(i, j, ks), tlow = t(i, j, ks), thlow = th(i, j, ks);
    const real qlow = qv(i, j, ks) / (real(1.) + qv(i, j, ks));
    const real cwmlow = qc(i, j, ks);
    const real thelow = (cwmlow * (-ELOCP / tlow) + real(1.)) * thlow;
    const real ulow = u(i, j, ks), vlow = v(i, j, ks);
    const real zsl = (zhk[lmh] - zhk[lmh + 1]) * real(0.5);
    const real tz0 = thz0(i, j) * apesfc;
    const real zsfc = zhk[lmh + 1];

    // SFCDIF (lines 347-1006)
    const real rdz = real(1.) / zsl;
    const real cxchl = EXCML * rdz, cxchs = EXCMS * rdz;
    const real btgx = G / thlow;
    const real elfc = VKARMAN * btgx;
    const real btgh = pblh_v > real(1000.) ? btgx * pblh_v : btgx * real(1000.);
    const real rlow = plow / (R_D * tlow);

    real ustar_v = ustar(i, j), z0 = znt(i, j), uz0_v = uz0(i, j), vz0_v = vz0(i, j);
    real thz0_v = thz0(i, j), qz0_v = qz0(i, j), qs = qsfc(i, j);
    real akms_v = akms(i, j), akhs_v = akhs(i, j);
    real hfx_v = hfx(i, j), qfx_v = qfx(i, j);
    const real z0base_v = z0base(i, j), wetm = mavail(i, j);
    real zu = real(0.), zt = real(0.), zq = real(0.), rib_v = real(0.), rlmo = real(0.);
    real ustark = real(0.), wstar2 = real(0.), psmz = real(0.), pshz = real(0.);

    if (seamask > real(0.5)) {  // sea points
      for (int itr = 1; itr <= ITRMX; ++itr) {
        z0 = c_max(USTFC * ustar_v * ustar_v, real(1.59e-5));
        if (ustar_v < USTC) {  // viscous sublayer, Janjic MWR 1994
          if (ustar_v < USTR) {
            if (ntsd == 1) {
              akms_v = cxchs;
              akhs_v = cxchs;
              qs = qlow;
            }
            zu = FZU1 * std::sqrt(std::sqrt(z0 * ustar_v * RVISC)) / ustar_v;
            const real wght = akms_v * zu * RVISC;
            const real rwgh = wght / (wght + real(1.));
            uz0_v = (ulow * rwgh + uz0_v) * real(0.5);
            vz0_v = (vlow * rwgh + vz0_v) * real(0.5);
            zt = FZT1 * zu;
            zq = FZQ1 * zt;
            const real wghtt = akhs_v * zt * RTVISC;
            const real wghtq = akhs_v * zq * RQVISC;
            if (ntsd > 1) {
              thz0_v = ((wghtt * thlow + thsk) / (wghtt + real(1.)) + thz0_v) * real(0.5);
              qz0_v = ((wghtq * qlow + qs) / (wghtq + real(1.)) + qz0_v) * real(0.5);
            } else {
              thz0_v = (wghtt * thlow + thsk) / (wghtt + real(1.));
              qz0_v = (wghtq * qlow + qs) / (wghtq + real(1.));
            }
          }
          if (ustar_v >= USTR && ustar_v < USTC) {
            zu = z0;
            uz0_v = real(0.);
            vz0_v = real(0.);
            zt = FZT2 * std::sqrt(std::sqrt(z0 * ustar_v * RVISC)) / ustar_v;
            zq = FZQ2 * zt;
            const real wghtt = akhs_v * zt * RTVISC;
            const real wghtq = akhs_v * zq * RQVISC;
            if (ntsd > 1) {
              thz0_v = ((wghtt * thlow + thsk) / (wghtt + real(1.)) + thz0_v) * real(0.5);
              qz0_v = ((wghtq * qlow + qs) / (wghtq + real(1.)) + qz0_v) * real(0.5);
            } else {
              thz0_v = (wghtt * thlow + thsk) / (wghtt + real(1.));
              qz0_v = (wghtq * qlow + qs) / (wghtq + real(1.));
            }
          }
        } else {
          zu = z0;
          uz0_v = real(0.);
          vz0_v = real(0.);
          zt = z0;
          thz0_v = thsk;
          zq = z0;
          qz0_v = qs;
        }
        const real tem = (tlow + tz0) * real(0.5);
        const real thm = (thelow + thz0_v) * real(0.5);
        const real a = thm * P608;
        const real b = (ELOCP / tem - real(1.) - P608) * thm;
        const real dthv = ((thelow - thz0_v) * ((qlow + qz0_v + cwmlow) * (real(0.5) * P608) + real(1.)) +
                           (qlow - qz0_v + cwmlow) * a + cwmlow * b);
        const real du2 = c_max((ulow - uz0_v) * (ulow - uz0_v) + (vlow - vz0_v) * (vlow - vz0_v), EPSU2);
        rib_v = btgx * dthv * zsl / du2;
        const real zslu = zsl + zu, zslt = zsl + zt;
        const real rzsu = zslu / zu, rzst = zslt / zt;
        const real rlogu = rlog(rzsu), rlogt = rlog(rzst);
        rlmo = elfc * akhs_v * dthv / (ustar_v * ustar_v * ustar_v);
        real zetalu = zslu * rlmo, zetalt = zslt * rlmo, zetau = zu * rlmo, zetat = zt * rlmo;
        zetalu = c_min(c_max(zetalu, ZTMIN1), ZTMAX1);
        zetalt = c_min(c_max(zetalt, ZTMIN1), ZTMAX1);
        zetau = c_min(c_max(zetau, ZTMIN1 / rzsu), ZTMAX1 / rzsu);
        zetat = c_min(c_max(zetat, ZTMIN1 / rzst), ZTMAX1 / rzst);
        psmz = lookup(tables.psim1, zetau, ZTMIN1, DZETA1);
        const real psmzl = lookup(tables.psim1, zetalu, ZTMIN1, DZETA1);
        const real simm = psmzl - psmz + rlogu;
        pshz = lookup(tables.psih1, zetat, ZTMIN1, DZETA1);
        const real pshzl = lookup(tables.psih1, zetalt, ZTMIN1, DZETA1);
        const real simh = (pshzl - pshz + rlogt) * PRT0;
        ustark = ustar_v * VKARMAN;
        akms_v = c_max(ustark / simm, cxchs);
        akhs_v = c_max(ustark / simh, cxchs);
        // Beljaars correction for USTAR (convection)
        wstar2 = dthv <= real(0.) ? WWST2 * rpow(c_abs(btgh * akhs_v * dthv), real(2.) / real(3.)) : real(0.);
        ustar_v = c_max(std::sqrt(akms_v * std::sqrt(du2 + wstar2)), EPSUST);
      }
    } else {  // land points
      if (ntsd == 1) qs = qlow;
      zu = z0;
      uz0_v = real(0.);
      vz0_v = real(0.);
      zt = zu * ZTFC;
      thz0_v = thsk;
      zq = zt;
      qz0_v = qs;
      const real tem = (tlow + tz0) * real(0.5);
      const real thm = (thelow + thz0_v) * real(0.5);
      const real a = thm * P608;
      const real b = (ELOCP / tem - real(1.) - P608) * thm;
      const real dthv = ((thelow - thz0_v) * ((qlow + qz0_v + cwmlow) * (real(0.5) * P608) + real(1.)) +
                         (qlow - qz0_v + cwmlow) * a + cwmlow * b);
      const real du2 = c_max((ulow - uz0_v) * (ulow - uz0_v) + (vlow - vz0_v) * (vlow - vz0_v), EPSU2);
      rib_v = btgx * dthv * zsl / du2;
      const real zslu = zsl + zu;
      const real rzsu = zslu / zu;
      const real rlogu = rlog(rzsu);
      const real zslt = zsl + zu;  // u, v and t are at the same level
      // topographic modification of the ZILFC term
      real topoterm = TOPOFAC * (zsfc * zsfc);
      topoterm = c_max(topoterm, real(3.0));
      const real zzil = dthv > real(0.) ? ZILFC * topoterm : ZILFC;
      for (int itr = 1; itr <= ITRMX; ++itr) {
        // Zilitinkevich fix for ZT
        zt = c_max(libm::exp(zzil * std::sqrt(ustar_v * z0base_v)) * z0base_v, EPSZT);
        zt = c_max(zt, zu * ZTFC);  // quick fix by S. Sukoriansky
        const real rzst = zslt / zt;
        const real rlogt = rlog(rzst);
        rlmo = elfc * akhs_v * dthv / (ustar_v * ustar_v * ustar_v);
        real zetalu = zslu * rlmo, zetalt = zslt * rlmo, zetau = zu * rlmo, zetat = zt * rlmo;
        zetalu = c_min(c_max(zetalu, ZTMIN2), ZTMAX2);
        zetalt = c_min(c_max(zetalt, ZTMIN2), ZTMAX2);
        zetau = c_min(c_max(zetau, ZTMIN2 / rzsu), ZTMAX2 / rzsu);
        zetat = c_min(c_max(zetat, ZTMIN2 / rzst), ZTMAX2 / rzst);
        psmz = lookup(tables.psim2, zetau, ZTMIN2, DZETA2);
        const real psmzl = lookup(tables.psim2, zetalu, ZTMIN2, DZETA2);
        const real simm = psmzl - psmz + rlogu;
        pshz = lookup(tables.psih2, zetat, ZTMIN2, DZETA2);
        const real pshzl = lookup(tables.psih2, zetalt, ZTMIN2, DZETA2);
        const real simh = (pshzl - pshz + rlogt) * PRT0;
        ustark = ustar_v * VKARMAN;
        akms_v = c_max(ustark / simm, cxchl);
        akhs_v = c_max(ustark / simh, cxchl);
        thz0_v = hfx_v / (akhs_v * rlow * CP) + thlow;  // TZ0 is updated too, and not read again
        wstar2 = dthv <= real(0.) ? WWST2 * rpow(c_abs(btgh * akhs_v * dthv), real(2.) / real(3.)) : real(0.);
        ustar_v = c_max(std::sqrt(akms_v * std::sqrt(du2 + wstar2)), EPSUST);
      }
    }

    // 2-m and 10-m diagnostics (lines 793-920)
    const real umflx = akms_v * (ulow - uz0_v);
    const real vmflx = akms_v * (vlow - vz0_v);
    const real hsflx = akhs_v * (thlow - thz0_v);
    const real hlflx = akhs_v * (qlow - qz0_v);
    real zu10 = zu + real(10.);
    const real zt02 = zt + real(2.), zt10 = zt + real(10.);
    real rlnu10 = rlog(zu10 / zu);
    const real rlnt02 = rlog(zt02 / zt), rlnt10 = rlog(zt10 / zt);
    real ztau10 = zu10 * rlmo;
    real ztat02 = zt02 * rlmo, ztat10 = zt10 * rlmo;
    real akms10, akhs02, akhs10;
    if (seamask > real(0.5)) {
      ztau10 = c_min(c_max(ztau10, ZTMIN1), ZTMAX1);
      ztat02 = c_min(c_max(ztat02, ZTMIN1), ZTMAX1);
      ztat10 = c_min(c_max(ztat10, ZTMIN1), ZTMAX1);
      const real simm10 = lookup(tables.psim1, ztau10, ZTMIN1, DZETA1) - psmz + rlnu10;
      const real simh02 = (lookup(tables.psih1, ztat02, ZTMIN1, DZETA1) - pshz + rlnt02) * PRT0;
      const real simh10 = (lookup(tables.psih1, ztat10, ZTMIN1, DZETA1) - pshz + rlnt10) * PRT0;
      akms10 = c_max(ustark / simm10, cxchs);
      akhs02 = c_max(ustark / simh02, cxchs);
      akhs10 = c_max(ustark / simh10, cxchs);
    } else {
      ztau10 = c_min(c_max(ztau10, ZTMIN2), ZTMAX2);
      ztat02 = c_min(c_max(ztat02, ZTMIN2), ZTMAX2);
      ztat10 = c_min(c_max(ztat10, ZTMIN2), ZTMAX2);
      const real simm10 = lookup(tables.psim2, ztau10, ZTMIN2, DZETA2) - psmz + rlnu10;
      const real simh02 = (lookup(tables.psih2, ztat02, ZTMIN2, DZETA2) - pshz + rlnt02) * PRT0;
      const real simh10 = (lookup(tables.psih2, ztat10, ZTMIN2, DZETA2) - pshz + rlnt10) * PRT0;
      akms10 = c_max(ustark / simm10, cxchl);
      akhs02 = c_max(ustark / simh02, cxchl);
      akhs10 = c_max(ustark / simh10, cxchl);
    }
    const real u10_v = umflx / akms10 + uz0_v;
    const real v10_v = vmflx / akms10 + vz0_v;
    const real th02_s = hsflx / akhs02 + thz0_v;  // SFCDIF's TH02 (QNSESFC's TSHLTR)
    const real th10_v = hsflx / akhs10 + thz0_v;
    real q02_s = hlflx / akhs02 + qz0_v;          // SFCDIF's Q02 (QSHLTR)
    real q10_v = hlflx / akhs10 + qz0_v;
    const real term1 = real(-0.068283) / tlow;
    const real pshltr_v = psfc * libm::exp(term1);

    // "equivalent" Z0 for the shelter winds over land
    real u10e_v = u10_v, v10e_v = v10_v;
    if (seamask < real(0.5)) {
      const real zuuz = c_min(zu * real(0.50), real(0.18));
      zu = c_max(zu * real(0.35), zuuz);
      zu10 = zu + real(10.);
      const real rzsu = zu10 / zu;
      rlnu10 = rlog(rzsu);
      ztau10 = zu10 * rlmo;
      ztau10 = c_min(c_max(ztau10, ZTMIN2), ZTMAX2);
      const real simm10 = lookup(tables.psim2, ztau10, ZTMIN2, DZETA2) - psmz + rlnu10;
      const real ekms10 = c_max(ustark / simm10, cxchl);
      u10e_v = umflx / ekms10 + uz0_v;
      v10e_v = vmflx / ekms10 + vz0_v;
    }

    // the other WRF driver arrays (SCM_FORCE_FLUX = 0)
    hfx_v = -rlow * CP * hsflx;
    qfx_v = -rlow * hlflx * wetm;
    const real flx_lh_v = XLV * qfx_v;
    const real flhc_v = rlow * CP * akhs_v;
    const real flqc_v = rlow * akhs_v * wetm;
    real qgh_v = ((real(1.) - seamask) * PQ0 + seamask * PQ0SEA) / plow * libm::exp(A2S * (tlow - A3S) / (tlow - A4S));
    qgh_v = qgh_v / (real(1.) - qgh_v);
    const real cpm_v = CP * (real(1.) + real(0.8) * qlow);
    if (seamask > real(0.5)) {
      qs = qlow + qfx_v / (rlow * akhs_v);
      qs = qs / (real(1.) - qs);
    }

    // remove supersaturation at 2 m and 10 m (lines 314-336)
    const real rapa = apesfc;
    const real th02p = th02_s, th10p = th10_v;
    const real rapa02 = rapa - GOCP02 / th02p;
    const real rapa10 = rapa - GOCP10 / th10p;
    const real t02p = th02p * rapa02, t10p = th10p * rapa10;
    const real t02_v = th02_s * apesfc;
    const real p02p = rpow(rapa02, RCAP) * real(1.e5);
    const real p10p = rpow(rapa10, RCAP) * real(1.e5);
    const real qs02 = PQ0 / p02p * libm::exp(A2 * (t02p - A3) / (t02p - A4));
    const real qs10 = PQ0 / p10p * libm::exp(A2 * (t10p - A3) / (t10p - A4));
    if (q02_s > qs02) q02_s = qs02;
    if (q10_v > qs10) q10_v = qs10;

    znt(i, j) = z0;
    ustar(i, j) = ustar_v;
    uz0(i, j) = uz0_v;
    vz0(i, j) = vz0_v;
    thz0(i, j) = thz0_v;
    qz0(i, j) = qz0_v;
    qsfc(i, j) = qs;
    akms(i, j) = akms_v;
    akhs(i, j) = akhs_v;
    rmol(i, j) = rlmo;
    rib(i, j) = rib_v;
    ct(i, j) = real(0.);
    chs(i, j) = akhs_v;
    chs2(i, j) = akhs02;
    cqs2(i, j) = akhs02;
    hfx(i, j) = hfx_v;
    qfx(i, j) = qfx_v;
    flx_lh(i, j) = flx_lh_v;
    flhc(i, j) = flhc_v;
    flqc(i, j) = flqc_v;
    qgh(i, j) = qgh_v;
    cpm(i, j) = cpm_v;
    u10(i, j) = u10_v;
    v10(i, j) = v10_v;
    tshltr(i, j) = th02_s;
    th02(i, j) = th02_s;
    th10(i, j) = th10_v;
    t02(i, j) = t02_v;
    qshltr(i, j) = q02_s;
    q10(i, j) = q10_v;
    q02(i, j) = q02_s / (real(1.) - q02_s);
    pshltr(i, j) = pshltr_v;
    u10e(i, j) = u10e_v;
    v10e(i, j) = v10e_v;
  }
};

void run_qnsesfc(const QNSESFCKernel& kernel, const Range2D& range);

}  // namespace cppwrf::phys::sf_qnsesfc
