// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// WRF phys/module_ra_sw.F (Dudhia shortwave, ra_sw_physics = 1): SWRAD's column set-up and SWPARA for one
// column, with WRF's module constants (r_d = 287, cp = 7*r_d/2) and without the WRF-Chem aerosols (pm2_5 = 0)
// and the eclipse (obscur = 0). SWRAD passes no slope_rad arguments, so SWPARA's slope code never runs.
// WRF's 1-D arrays run top-down (K = 1 is the model top, NK = kme-1-K+kms); every one of them but SDOWN
// depends on its own level only, so they are formed in the layer loop.
#include <algorithm>
#include <cmath>

#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"

namespace cppwrf::phys {

// swinit: CSSCA = swrad_scat * 1.e-5
CPPWRF_HD inline rb dudhia_cssca(rb swrad_scat) { return swrad_scat * rb(1.e-5); }

// Model levels k = 0 (lowest) .. nz-1: t (K), p (Pa), dz (dz8w, m); moist species qv..qg, a null pointer for
// a species the scheme lacks (WRF F_Q*). warm_rain (Kessler): without qi the cloud and rain below 273.15 K are
// not turned into ice and snow. tten: the heating TTEN1D(NK) at level k (K/s, 0 at night); returns GSW.
CPPWRF_HD inline rb dudhia_swpara(int nz, CSD t, CSD p, CSD dz, CSD qv, CSD qc, CSD qr, CSD qi, CSD qs, CSD qg,
                                      rb coszen, rb albedo, rb solcon, int icloud, rb cssca,
                                      bool warm_rain, SD tten) {
  constexpr rb R = rb(287.), CP = rb(7.) * rb(287.) / rb(2.);
  constexpr rb ALBTAB[5][4] = {{rb(0.), rb(0.), rb(0.), rb(0.)}, {rb(69.), rb(58.), rb(40.), rb(15.)}, {rb(90.), rb(80.), rb(70.), rb(60.)},
                                   {rb(94.), rb(90.), rb(82.), rb(78.)}, {rb(96.), rb(92.), rb(85.), rb(80.)}};  // ALBTAB(i,j) = ALBTAB[j-1][i-1]
  constexpr rb ABSTAB[5][4] = {{rb(0.), rb(0.), rb(0.), rb(0.)}, {rb(0.), rb(2.5), rb(4.), rb(5.)}, {rb(0.), rb(2.6), rb(7.), rb(10.)},
                                   {rb(0.), rb(3.3), rb(10.), rb(14.)}, {rb(0.), rb(3.7), rb(10.), rb(15.)}};
  constexpr rb XMUVAL[4] = {rb(0.), rb(0.2), rb(0.5), rb(1.0)};
  for (int k = 0; k < nz; ++k) tten[k] = rb(0.);
  rb GSW = rb(0.0);
  const rb bext340 = rb(5.E-6), bexth2o = rb(5.E-6);
  const rb SOLTOP = solcon * (1 - rb(0.));
  const rb CSZA = coszen;
  if (CSZA <= rb(1.E-9)) return GSW;  // night
  if (SOLTOP <= rb(1.E-9)) return GSW;
  const rb XMU = CSZA;
  const rb SDOWN1 = SOLTOP * XMU;
  rb SDOWN = SDOWN1;  // SDOWN(K)
  rb WW = rb(0.), UV = rb(0.), OLDALB = rb(0.), OLDABC = rb(0.), TOTABS = rb(0.), DSCA = rb(0.), DABS = rb(0.), DSCLD = rb(0.), DABSA = rb(0.);
  for (int K = 1; K <= nz; ++K) {
    const int m = nz - K;  // NK
    const rb T = t[m], P = p[m], DZ = dz[m];
    // SWRAD: max(0, q) of the species present; ice and snow from cloud and rain without F_QI
    rb QV = qv.p ? std::max(rb(0.), qv[m]) : rb(0.), QC = qc.p ? std::max(rb(0.), qc[m]) : rb(0.);
    rb QR = qr.p ? std::max(rb(0.), qr[m]) : rb(0.), QI = rb(0.), QS = rb(0.), QG = rb(0.);
    if (qi.p) {
      QI = std::max(rb(0.), qi[m]);
    } else if (!warm_rain && T < rb(273.15)) {
      QI = QC;
      QC = rb(0.);
      QS = QR;
      QR = rb(0.);
    }
    if (qs.p) QS = std::max(rb(0.), qs[m]);
    if (qg.p) QG = std::max(rb(0.), qg[m]);
    const rb RO = P / (R * T);
    const rb XWVP = RO * QV * DZ * rb(1000.);
    const rb XATP = RO * DZ;
    const rb XLWP = icloud == 0 ? rb(0.) : RO * rb(1000.) * DZ * (QC + rb(0.1) * QI + rb(0.05) * QR + rb(0.02) * QS + rb(0.05) * QG);
    WW = WW + XLWP;
    UV = UV + XWVP;
    const rb WGM = WW / XMU;
    const rb UGCM = UV * rb(0.0001) / XMU;
    const rb OLDABS = TOTABS;
    TOTABS = rb(2.9) * UGCM / (libm::pow(rb(1.) + rb(141.5) * UGCM, rb(0.635)) + rb(5.925) * UGCM);
    const rb beta = rb(0.4) * (rb(1.0) - XMU) + rb(0.1);
    const rb aer_dry1 = rb(0.), aer_water1 = rb(0.);
    rb XSCA = (cssca * XATP + beta * aer_dry1 * bext340 * DZ + beta * aer_water1 * bexth2o * DZ) / XMU;
    rb XABS = (TOTABS - OLDABS) * (SDOWN1 - DSCLD - DSCA - DABSA) / SDOWN;
    rb XABSA = rb(0.);
    if (XABS < rb(0.)) XABS = rb(0.);
    rb ALW = libm::log10(WGM + rb(1.));
    if (ALW > rb(3.999)) ALW = rb(3.999);
    int IIL = 0, IU = 0;
    rb XI = rb(0.);
    for (int II = 1; II <= 3; ++II) {
      if (XMU > XMUVAL[II - 1]) {
        IIL = II;
        IU = II + 1;
        XI = (XMU - XMUVAL[II - 1]) / (XMUVAL[II] - XMUVAL[II - 1]) + rb(IIL);
      }
    }
    const int JJL = int(ALW) + 1;
    const int JU = JJL + 1;
    const rb YJ = ALW + rb(1.);
    const rb dI1 = XI - rb(IIL), dI2 = rb(IU) - XI, dJ1 = YJ - rb(JJL), dJ2 = rb(JU) - YJ;
    const rb den = rb((IU - IIL) * (JU - JJL));
    const rb ALBA = (ALBTAB[JU - 1][IU - 1] * dI1 * dJ1 + ALBTAB[JU - 1][IIL - 1] * dI2 * dJ1 +
                         ALBTAB[JJL - 1][IU - 1] * dI1 * dJ2 + ALBTAB[JJL - 1][IIL - 1] * dI2 * dJ2) / den;
    const rb ABSC = (ABSTAB[JU - 1][IU - 1] * dI1 * dJ1 + ABSTAB[JU - 1][IIL - 1] * dI2 * dJ1 +
                         ABSTAB[JJL - 1][IU - 1] * dI1 * dJ2 + ABSTAB[JJL - 1][IIL - 1] * dI2 * dJ2) / den;
    rb XALB = (ALBA - OLDALB) * (SDOWN1 - DSCA - DABS) / SDOWN;
    rb XABSC = (ABSC - OLDABC) * (SDOWN1 - DSCA - DABS) / SDOWN;
    if (XALB < rb(0.)) XALB = rb(0.);
    if (XABSC < rb(0.)) XABSC = rb(0.);
    DSCLD = DSCLD + (XALB + XABSC) * SDOWN * rb(0.01);
    DSCA = DSCA + XSCA * SDOWN;
    DABS = DABS + XABS * SDOWN;
    DABSA = DABSA + XABSA * SDOWN;
    OLDALB = ALBA;
    OLDABC = ABSC;
    rb TRANS0 = rb(100.) - XALB - XABSC - XABS * rb(100.) - XSCA * rb(100.);
    if (TRANS0 < rb(1.)) {
      const rb FF = rb(99.) / (XALB + XABSC + XABS * rb(100.) + XSCA * rb(100.));
      XALB = XALB * FF;
      XABSC = XABSC * FF;
      XABS = XABS * FF;
      XSCA = XSCA * FF;
      TRANS0 = rb(1.);
    }
    const rb SDOWN_NEXT = std::max(rb(1.E-9), SDOWN * TRANS0 * rb(0.01));
    tten[m] = SDOWN * (XABSC + XABS * rb(100.) + XABSA * rb(100.)) * rb(0.01) / (RO * CP * DZ);
    SDOWN = SDOWN_NEXT;
  }
  GSW = (rb(1.) - albedo) * SDOWN;
  return GSW;
}

// radiation_driver's surface direct / diffuse split for SW schemes without their own (Ruiz-Arias et al. 2010),
// where coszen > 1e-3: swddif and swddir from SWDOWN, the solar constant and the terrain height ht (m); the
// caller forms swddni = swddir / max(coszen, 1e-4) from the stored swddir. Returns false where coszen <= 1e-3
CPPWRF_HD inline bool sw_diffuse_backup(rb coszen, rb swdown, rb solcon, rb ht, rb& swddif,
                                        rb& swddir) {
  if (!(coszen > rb(1e-3))) return false;
  const rb ioh = solcon * coszen;
  rb kt = swdown / std::max(ioh, rb(1e-3));
  const rb airmass =
      libm::exp(-ht / rb(8434.5)) / (coszen + rb(0.50572) * libm::pow(libm::asin(coszen) * rb(57.295779513082323) + rb(6.07995), -rb(1.6364)));
  kt = kt / (rb(0.1) + rb(1.031) * libm::exp(-rb(1.4) / (rb(0.9) + (rb(9.4) / std::max(airmass, rb(1e-3))))));
  const rb kd = rb(0.952) - rb(1.041) * libm::exp(-libm::exp(rb(2.300) - rb(4.702) * kt));
  swddif = kd * swdown;
  swddir = (rb(1.) - kd) * swdown;
  return true;
}

}  // namespace cppwrf::phys
