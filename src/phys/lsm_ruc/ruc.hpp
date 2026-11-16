// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// RUC land surface model, sf_surface_physics = 3: WRF v4.8.0 phys/module_sf_ruclsm.F (LSMRUC for one point, SFCTMP,
// SOIL, SICE, SNOWSOIL, SNOWSEAICE, SOILTEMP, SNOWTEMP, SOILMOIST, SOILPROP, TRANSF, VILKA, QSN, SOILVEGIN,
// RUCLSMINIT) and phys/module_sf_sfcdiags_ruclsm.F (SFCDIAGS_RUCLSM, RSLF, RSIF). Every statement follows the Fortran
// in order and parenthesisation; arrays are 1-based as in WRF (element 0 unused). Ported: mosaic_lu = mosaic_soil = 0,
// spp_lsm = 0, lakemodel = 0, the module's isncond_opt = 2 and isncovr_opt = 2. Statements whose results are never
// used (the debug prints, the water budget, ALFA / PSIT of SOIL, HK of SOILPROP, GX / PART of TRANSF, T3 / UPFLUX /
// XINET after the energy balance, the dead first COSMC / RHSMC of SOILMOIST) are left out. WRF's object calls the
// scalar libm only (no _ZGV symbols); X**2 / X**2. are products, TN**4 is (TN*TN)*(TN*TN), X**3. is pow.
// ILNB of SFCTMP is an uninitialised local in WRF that SNOWTEMP / SNOWSEAICE read when the snow is thinner than
// SNTH from the start (then TSNAV takes the 2-layer formula if the stale ILNB > 1). With gfortran the stack slot keeps
// the ILNB of the previous SFCTMP call, so WRF's value depends on the point order inside a tile and on what ran
// before in the step: undefined. Here ILNB starts at 0 (TSNAV = 0.5*(SOILT+TSO(1))-273.15) unless the caller passes
// LsmConfig::ilnb_carry, which the column test uses to reproduce gfortran's serial behaviour. TSNAV feeds the next
// step's snow compaction when 0.0081e3/RHOSN < SNHEI, so thin snow (8.1/RHOSN < SNHEI < 10/RHOSN) may differ from WRF.
#include "core/backend.hpp"
#include "core/precision.hpp"

namespace cppwrf::phys::ruc {

inline constexpr int kMaxSoil = 9;  // check_a_mundo: num_soil_layers 6 or 9
inline constexpr int kNlus = 50;
inline constexpr int kNsltype = 30;
inline constexpr int kTbq = 5001;
inline constexpr int kA = kMaxSoil + 2;  // 1-based arrays (zshalf(nroot+1) may reach nzs)

// VEGPARM.TBL (USGS-RUC / MODI-RUC), SOILPARM.TBL (STAS-RUC) and LSMRUC's saturation table TBQ
struct RucTables {
  int lucats = 0, bare = 0, natural = 0, crop = 0, urban = 0, slcats = 0;
  int ifortbl[kNlus] = {};
  real albtbl[kNlus] = {}, z0tbl[kNlus] = {}, lemitbl[kNlus] = {}, pctbl[kNlus] = {}, shdtbl[kNlus] = {},
       rstbl[kNlus] = {}, rgltbl[kNlus] = {}, hstbl[kNlus] = {}, snuptbl[kNlus] = {}, laitbl[kNlus] = {},
       maxalb[kNlus] = {};
  real topt_data = 0, cmcmax_data = 0, cfactr_data = 0, rsmax_data = 0;
  real bb[kNsltype] = {}, drysmc[kNsltype] = {}, hc[kNsltype] = {}, maxsmc[kNsltype] = {}, refsmc[kNsltype] = {},
       satpsi[kNsltype] = {}, satdk[kNsltype] = {}, satdw[kNsltype] = {}, wltsmc[kNsltype] = {}, qtz[kNsltype] = {};
  real tbq[kTbq] = {};
};

// LSMRUC's TBQ loop (computed at every call in WRF, the same values)
inline void make_tbq(real* tbq) {
  real cq = real(173.15) - real(.05);
  const real r61 = real(6.1153) * real(0.62198);
  for (int k = 1; k <= kTbq; ++k) {
    cq = cq + real(.05);
    const real evs = libm::exp(real(17.67) * (cq - real(273.15)) / (cq - real(29.65)));
    const real eis = libm::exp(real(22.514) - real(6.15e3) / cq);
    tbq[k - 1] = cq >= real(273.15) ? r61 * evs : r61 * eis;
  }
}

// module_model_constants
inline constexpr real kRhowater = real(1000.);
inline constexpr real kPiconst = real(3.1415926535897932384626433);
inline constexpr real kP1000mb = real(100000.);
inline constexpr real kXls = real(2.85E6);

// QSN: saturation mixing ratio * pressure from the table (t(1:5001))
CPPWRF_HD inline real qsn(real tn, const real* t) {
  real r = (tn - real(173.15)) / real(.05) + real(1.);
  int i = int(r);
  if (i < 1) {
    i = 1;
    r = real(1.);
  }
  if (i > 5000) {
    i = 5000;
    r = real(5001.);
  }
  const real r1 = t[i - 1];
  const real r2 = r - real(i);
  return (t[i] - r1) * r2 + r1;
}

// VILKA: the surface temperature TS and saturation QS of the energy balance by iteration on the table
CPPWRF_HD inline void vilka(real tn, real d1, real d2, real pp, real& qs, real& ts, const real* tt) {
  int i = int((tn - real(1.7315e2)) / real(.05) + real(1));
  if (i > 5000 || i < 1) {  // WRF reads TT(I) out of its bounds before the check
    raise_fatal(kFatalRucVilka);
    qs = ts = tn;
    return;
  }
  real t1 = real(173.1) + real(i) * real(.05);
  real f1 = t1 + d1 * tt[i - 1] - d2;
  int i1 = int(real(i) - f1 / (real(.05) + d1 * (tt[i] - tt[i - 1])));
  i = i1;
  real rn = 0;
  for (int it = 0;; ++it) {
    if (i > 5000 || i < 1 || it > 100000) {
      raise_fatal(kFatalRucVilka);
      qs = ts = tn;
      return;
    }
    i1 = i;
    t1 = real(173.1) + real(i) * real(.05);
    f1 = t1 + d1 * tt[i - 1] - d2;
    rn = f1 / (real(.05) + d1 * (tt[i] - tt[i - 1]));
    i = i - int(rn);
    if (i > 5000 || i < 1) continue;  // the check at the loop top stops
    if (i1 == i) break;
  }
  ts = t1 - real(.05) * rn;
  qs = (tt[i - 1] + (tt[i - 1] - tt[i]) * rn) / pp;
}

// Soil parameters of SOILVEGIN (and their sea-ice overrides in LSMRUC)
struct SoilParm {
  real qwrtz = 0, rhocs = 0, bclh = 0, dqm = 0, ksat = 0, psis = 0, qmin = 0, ref = 0, wilt = 0;
};

// Column geometry and the constants LSMRUC passes down
struct Geo {
  int nzs = 0, nroot = 4;
  real delt = 0, conflx = 0;
  real zsmain[kA] = {}, zshalf[kA] = {}, dtdzs[2 * kMaxSoil + 1] = {}, dtdzs2[kA] = {};
};
struct Consts {
  real cp, rovcp, g0, lv, stbolt;
  real cw = real(4.183e6), kqwrtz = real(7.7), kice = real(2.2), kwt = real(0.57), c1sn = real(0.026),
       c2sn = real(21.);
};
// Atmospheric forcing of the point (constant through SFCTMP's call tree)
struct Atm {
  real patm, tabs, qvatm, qcatm, rho, glw, qkms, tkms;
  bool myj;
};

// TRANSF: transpiration function of the root layers
CPPWRF_HD inline void transf(int nroot, const real* soiliqw, real tabs, real lai, real gswin, real qmin, real ref,
                             real wilt, const real* zshalf, real pc, int iland, const RucTables& tb, real* tranf,
                             real& transum, int nzs) {
  for (int k = 1; k <= nzs; ++k) tranf[k] = 0;
  transum = 0;
  real totliq = soiliqw[1] + qmin;
  real did = zshalf[2];
  if (totliq > ref) tranf[1] = did;
  else if (totliq <= wilt) tranf[1] = 0;
  else tranf[1] = (totliq - wilt) / (ref - wilt) * did;
  for (int k = 2; k <= nroot; ++k) {
    totliq = soiliqw[k] + qmin;
    did = zshalf[k + 1] - zshalf[k];
    if (totliq >= ref) tranf[k] = did;
    else if (totliq <= wilt) tranf[k] = 0;
    else tranf[k] = (totliq - wilt) / (ref - wilt) * did;
  }
  const real pctot = lai > real(4.) ? real(0.8) : pc;
  real ftem;
  if (tabs <= real(302.15)) ftem = real(1.0) / (real(1.0) + libm::exp(real(-0.41) * (tabs - real(282.05))));
  else ftem = real(1.0) / (real(1.0) + libm::exp(real(0.5) * (tabs - real(314.0))));
  const real cmin = real(1.) / tb.rsmax_data;
  real cmax = real(1.) / tb.rstbl[iland - 1];
  if (lai > real(1.)) cmax = lai / tb.rstbl[iland - 1];
  real fsol;
  if (gswin < tb.rgltbl[iland - 1]) fsol = real(1.) / (real(1.) + libm::exp(real(-0.034) * (gswin - real(3.5))));
  else fsol = real(1.);
  const real totcnd = (cmin + (cmax - cmin) * pctot * ftem * fsol) / cmax;
  transum = 0;
  for (int k = 1; k <= nroot; ++k) {
    tranf[k] = c_max(cmin, tranf[k] * totcnd);
    transum = transum + tranf[k];
  }
}

// SOILPROP: thermal and hydraulic properties
CPPWRF_HD inline void soilprop(int nzs, const real* fwsat, const real* lwsat, const real* tav, const real* keepfr,
                               const real* soilmois, const real* soiliqw, const real* soilice, const real* soilmoism,
                               const real* soiliqwm, const real* soilicem, const SoilParm& sp, real riw, real xlmelt,
                               real cp, real g0_p, real cvw, real ci, const Consts& c, real* thdif, real* diffu,
                               real* hydro, real* cap) {
  const int nzs1 = nzs - 1;
  const real kzero = real(2.);
  const real ws = sp.dqm + sp.qmin;
  const real x1 = xlmelt / (g0_p * sp.psis);
  const real x2 = x1 / sp.bclh * ws;
  const real x4 = (sp.bclh + real(1.)) / sp.bclh;
  const real gamd = (real(1.) - ws) * real(2700.);
  const real kdry = (real(0.135) * gamd + real(64.7)) / (real(2700.) - real(0.947) * gamd);
  real kas;
  if (sp.qwrtz > real(0.2)) kas = libm::pow(c.kqwrtz, sp.qwrtz) * libm::pow(kzero, real(1.) - sp.qwrtz);
  else kas = libm::pow(c.kqwrtz, sp.qwrtz) * libm::pow(real(3.), real(1.) - sp.qwrtz);
  for (int k = 1; k <= nzs1; ++k) {
    const real tn = tav[k] - real(273.15);
    real detal = 0;
    if (soilicem[k] != real(0) && tn < real(0.)) {
      detal = real(273.15) * x2 / (tav[k] * tav[k]) * libm::pow(tav[k] / (x1 * tn), x4);
      if (keepfr[k] == real(1.)) detal = 0;
    }
    const real kasat = libm::pow(kas, real(1.) - ws) * libm::pow(c.kice, fwsat[k]) * libm::pow(c.kwt, lwsat[k]);
    const real x5 = (soilmoism[k] + sp.qmin) / ws;
    real ke;
    if (soilicem[k] == real(0.)) {
      const real sr = c_max(real(0.101), x5);
      ke = libm::log10(sr) + real(1.);
    } else {
      ke = x5;
    }
    const real kjpl = ke * (kasat - kdry) + kdry;
    cap[k] = (real(1.) - ws) * sp.rhocs + (soiliqwm[k] + sp.qmin) * cvw + soilicem[k] * ci +
             (sp.dqm - soilmoism[k]) * cp * real(1.2) - detal * real(1.e3) * xlmelt;
    const real a = riw * soilicem[k];
    if ((ws - a) < real(0.12)) {
      diffu[k] = 0;
    } else {
      const real h = c_max(real(0.), (soilmoism[k] + sp.qmin - a) / (c_max(real(1.e-8), (ws - a))));
      real facd = real(1.);
      if (a != real(0.)) facd = real(1.) - a / c_max(real(1.e-8), soilmoism[k]);
      const real ame = c_max(real(1.e-8), ws - riw * soilicem[k]);
      diffu[k] = -sp.bclh * sp.ksat * sp.psis / ame * libm::pow(ws / ame, real(3.)) *
                 libm::pow(h, sp.bclh + real(2.)) * facd;
    }
    thdif[k] = kjpl / cap[k];
  }
  for (int k = 1; k <= nzs; ++k) {
    if ((ws - riw * soilice[k]) < real(0.12)) {
      hydro[k] = 0;
    } else {
      real fach = real(1.);
      if (soilice[k] != real(0.)) fach = real(1.) - riw * soilice[k] / c_max(real(1.e-8), soilmois[k]);
      const real am = c_max(real(1.e-8), ws - riw * soilice[k]);
      hydro[k] = c_min(sp.ksat, sp.ksat / am * libm::pow(soiliqw[k] / am, real(2.) * sp.bclh + real(2.)) * fach);
      if (hydro[k] < real(1.e-10)) hydro[k] = 0;
    }
  }
}

// SOILMOIST: soil moisture, runoff and infiltration
CPPWRF_HD inline void soilmoist(const Geo& g, real riw, const real* diffu, const real* hydro, real qsg, real qvg,
                                real qcg, real qcatm, real qvatm, real prcp, real qkms, const real* transp, real drip,
                                real dew, real smelt, const real* soilice, real vegfrac, real snowfrac, real soilres,
                                const SoilParm& sp, real ras, real& infmax, real* soilmois, real& mavail,
                                real& runoff, real& runoff2, real& infiltrp) {
  const int nzs = g.nzs, nzs1 = nzs - 1, nzs2 = nzs - 2;
  const real delt = g.delt;
  const real* zsmain = g.zsmain;
  const real* zshalf = g.zshalf;
  const real dqm = sp.dqm, qmin = sp.qmin, ref = sp.ref;
  real cosmc[kA] = {}, rhsmc[kA] = {};
  cosmc[1] = 0;
  rhsmc[1] = soilmois[nzs];
  for (int k = 1; k <= nzs2; ++k) {
    const int kn = nzs - k;
    const int k1 = 2 * kn - 3;
    const real x4 = real(2.) * g.dtdzs[k1] * diffu[kn - 1];
    const real x2 = real(2.) * g.dtdzs[k1 + 1] * diffu[kn];
    const real q4 = x4 + hydro[kn - 1] * g.dtdzs2[kn - 1];
    const real q2 = x2 - hydro[kn + 1] * g.dtdzs2[kn - 1];
    const real denom = real(1.) + x2 + x4 - q2 * cosmc[k];
    cosmc[k + 1] = q4 / denom;
    rhsmc[k + 1] = (soilmois[kn] + q2 * rhsmc[k] + transp[kn] / (zshalf[kn + 1] - zshalf[kn]) * delt) / denom;
  }
  const real trans = transp[1];
  const real umveg = (real(1.) - vegfrac) * soilres;
  runoff = 0;
  runoff2 = 0;
  const real dzs = zsmain[2];
  const real r1 = cosmc[nzs1];
  const real r2 = rhsmc[nzs1];
  const real r3 = diffu[1] / dzs;
  real r4 = r3 + hydro[1] * real(.5);
  const real r5 = r3 - hydro[2] * real(.5);
  const real r6 = qkms * ras;
  const real totliq = prcp - drip / delt - umveg * dew * ras - smelt;
  real flx = totliq;
  infiltrp = totliq;
  const real cvfrz = real(3.);
  const real refkdt = real(3.);
  const real refdk = real(3.4341e-6);
  const real delt1 = delt / real(86400.);
  const real f1max = dqm * zshalf[2];
  const real f1 = f1max * (real(1.) - soilmois[1] / dqm);
  real dice = soilice[1] * zshalf[2];
  real fd = f1;
  for (int k = 2; k <= nzs1; ++k) {
    dice = dice + (zshalf[k + 1] - zshalf[k]) * soilice[k];
    const real fkmax = dqm * (zshalf[k + 1] - zshalf[k]);
    const real fk = fkmax * (real(1.) - soilmois[k] / dqm);
    fd = fd + fk;
  }
  const real kdt = refkdt * sp.ksat / refdk;
  const real val = (real(1.) - libm::exp(-kdt * delt1));
  const real ddt = fd * val;
  real px = -totliq * delt;
  if (px < real(0.0)) px = real(0.0);
  real infmax1;
  if (px > real(0.0)) infmax1 = (px * (ddt / (px + ddt))) / delt;
  else infmax1 = 0;
  const real frzx = real(0.15) * ((dqm + qmin) / ref) * (real(0.412) / real(0.468));
  real fcr = real(1.);
  if (dice > real(1.e-2)) {
    const real acrt = cvfrz * frzx / dice;
    real sum = real(1.);
    const int ialp1 = int(cvfrz - real(1));
    for (int jk = 1; jk <= ialp1; ++jk) {
      int k = 1;
      for (int jj = jk + 1; jj <= ialp1; ++jj) k = k * jj;
      sum = sum + libm::pow(acrt, cvfrz - real(jk)) / real(k);
    }
    fcr = real(1.) - libm::exp(-acrt) * sum;
  }
  infmax1 = infmax1 * fcr;
  infmax = c_max(infmax1, hydro[1] * soilmois[1]);
  infmax = c_min(infmax, -totliq);
  if (-totliq > infmax) {
    runoff = -totliq - infmax;
    flx = -infmax;
  }
  infiltrp = flx;
  const real r7 = real(.5) * dzs / delt;
  r4 = r4 + r7;
  flx = flx - soilmois[1] * r7;
  const real r8 = umveg * r6 * (real(1.) - snowfrac);
  const real qtot = qvatm + qcatm;
  const real r9 = trans;
  const real r10 = qtot - qsg;
  real qq, flxsat;
  if (r10 <= real(0.)) {
    qq = (r5 * r2 - flx + r9) / (r4 - r5 * r1 - r10 * r8 / (ref - qmin));
    flxsat = -dqm * (r4 - r5 * r1 - r10 * r8 / (ref - qmin)) + r5 * r2 + r9;
  } else {
    qq = (r2 * r5 - flx + r8 * (qtot - qcg - qvg) + r9) / (r4 - r1 * r5);
    flxsat = -dqm * (r4 - r1 * r5) + r2 * r5 + r8 * (qtot - qvg - qcg) + r9;
  }
  if (qq < real(0.)) {
    soilmois[1] = real(1.e-8);
  } else if (qq > dqm) {
    soilmois[1] = dqm;
    runoff = runoff + (flxsat - flx);
  } else {
    soilmois[1] = c_min(dqm, c_max(real(1.e-8), qq));
  }
  for (int k = 2; k <= nzs; ++k) {
    const int kk = nzs - k + 1;
    qq = cosmc[kk] * soilmois[k - 1] + rhsmc[kk];
    if (qq < real(0.)) {
      soilmois[k] = real(1.e-8);
    } else if (qq > dqm) {
      soilmois[k] = dqm;
      if (k == nzs) runoff2 = runoff2 + ((qq - dqm) * (zsmain[k] - zshalf[k])) / delt;
      else runoff2 = runoff2 + ((qq - dqm) * (zshalf[k + 1] - zshalf[k])) / delt;
    } else {
      soilmois[k] = c_min(dqm, c_max(real(1.e-8), qq));
    }
  }
  mavail = c_max(real(.00001),
                 c_min(real(1.), (soilmois[1] / (ref - qmin) * (real(1.) - snowfrac) + real(1.) * snowfrac)));
}

// The soil temperature sweep shared by SOILTEMP, SICE, SNOWSEAICE and SNOWTEMP (loop 33)
CPPWRF_HD inline void tso_sweep(const Geo& g, const real* thd, const real* tso, real* cotso, real* rhtso) {
  const int nzs = g.nzs, nzs2 = nzs - 2;
  cotso[1] = 0;
  rhtso[1] = tso[nzs];
  for (int k = 1; k <= nzs2; ++k) {
    const int kn = nzs - k;
    const int k1 = 2 * kn - 3;
    const real x1 = g.dtdzs[k1] * thd[kn - 1];
    const real x2 = g.dtdzs[k1 + 1] * thd[kn];
    const real ft = tso[kn] + x1 * (tso[kn - 1] - tso[kn]) - x2 * (tso[kn] - tso[kn + 1]);
    const real denom = real(1.) + x1 + x2 - x2 * cotso[k];
    cotso[k + 1] = x1 / denom;
    rhtso[k + 1] = (ft + x2 * rhtso[k]) / denom;
  }
}

// SOILTEMP: the energy balance of the snow-free soil and the soil temperature
CPPWRF_HD inline void soiltemp(const Geo& g, const Atm& a, real prcpms, real rainf, real emiss, real rnet, real vegfrac,
                               const real* thdif, const real* cap, real drycan, real wetcan, real transum, real mavail,
                               real soilres, real xlv, real cp, real cvw, real stbolt, const real* tbq, real* tso,
                               real& soilt, real& qvg, real& qsg, real& qcg, real& x) {
  const int nzs = g.nzs, nzs1 = nzs - 1;
  const real delt = g.delt;
  const real dzstop = real(1.) / (g.zsmain[2] - g.zsmain[1]);
  const real qgold = qvg;
  real cotso[kA] = {}, rhtso[kA] = {};
  tso_sweep(g, thdif, tso, cotso, rhtso);
  const real rhcs = cap[1];
  const real h = mavail;
  const real trans = transum * drycan / g.zshalf[g.nroot + 1];
  const real can = wetcan + trans;
  const real umveg = (real(1.) - vegfrac) * soilres;
  const real d1 = cotso[nzs1];
  const real d2 = rhtso[nzs1];
  const real tn = soilt;
  const real d9 = thdif[1] * rhcs * dzstop;
  const real d10 = a.tkms * cp * a.rho;
  const real r211 = real(.5) * g.conflx / delt;
  const real r21 = r211 * cp * a.rho;
  const real r22 = real(.5) / (thdif[1] * delt * (dzstop * dzstop));
  const real r6 = emiss * stbolt * real(.5) * powi(tn, 4);
  const real r7 = r6 / tn;
  const real d11 = rnet + r6;
  const real tdenom = d9 * (real(1.) - d1 + r22) + d10 + r21 + r7 + rainf * cvw * prcpms;
  const real fkq = a.qkms * a.rho;
  const real r210 = r211 * a.rho;
  const real c = vegfrac * fkq * can;
  const real cc = c * xlv / tdenom;
  real aa = xlv * (fkq * umveg + r210) / tdenom;
  real bb = (d10 * a.tabs + r21 * tn + xlv * (a.qvatm * (fkq * umveg + c) + r210 * qvg) + d11 + d9 * (d2 + r22 * tn) +
             rainf * cvw * prcpms * c_max(real(273.15), a.tabs)) /
            tdenom;
  real aa1 = aa + cc;
  const real pp = a.patm * real(1.e3);
  aa1 = aa1 / pp;
  real qs1, ts1;
  vilka(tn, aa1, bb, pp, qs1, ts1, tbq);
  const real tq2 = a.qvatm;
  const real tx2 = tq2 * (real(1.) - h);
  real q1 = tx2 + h * qs1;
  bool sat = !(q1 < qs1);
  if (!sat) {  // label 100
    bb = bb - aa * tx2;
    aa = (aa * h + cc) / pp;
    vilka(tn, aa, bb, pp, qs1, ts1, tbq);
    q1 = tx2 + h * qs1;
    sat = q1 >= qs1;
    if (!sat) {
      qsg = qs1;
      qvg = q1;
      tso[1] = ts1;
      qcg = 0;
    }
  }
  if (sat) {  // label 90
    qvg = qs1;
    qsg = qs1;
    tso[1] = ts1;
    qcg = c_max(real(0.), q1 - qs1);
  }
  soilt = ts1;
  for (int k = 2; k <= nzs; ++k) {
    const int kk = nzs - k + 1;
    tso[k] = rhtso[kk] + cotso[kk] * tso[k - 1];
  }
  x = (cp * a.rho * r211 + rhcs * g.zsmain[2] * real(0.5) / delt) * (soilt - tn) +
      xlv * a.rho * r211 * (qvg - qgold);
  x = x - rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soilt);
}

// The liquid / frozen split of the soil water (repeated in SOIL and SNOWSOIL); ice_rho: the keepfr branch's factor
CPPWRF_HD inline void frozen_split(int nzs, const real* tso, const real* soilmois, const real* smfrkeep,
                                   const real* keepfr, const SoilParm& sp, real xlmelt, real riw, bool snowsoil_first,
                                   real* soiliqw, real* soilice) {
  for (int k = 1; k <= nzs; ++k) {
    const real tln = libm::log(tso[k] / real(273.15));
    if (tln < real(0.)) {
      soiliqw[k] = (sp.dqm + sp.qmin) *
                       libm::pow(xlmelt * (tso[k] - real(273.15)) / tso[k] / real(9.81) / sp.psis,
                                 real(-1.) / sp.bclh) -
                   sp.qmin;
      soiliqw[k] = c_max(real(0.), soiliqw[k]);
      soiliqw[k] = c_min(soiliqw[k], soilmois[k]);
      soilice[k] = (soilmois[k] - soiliqw[k]) / riw;
      if (keepfr[k] == real(1.)) {
        soilice[k] = c_min(soilice[k], smfrkeep[k]);
        if (snowsoil_first) soiliqw[k] = c_max(real(0.), soilmois[k] - soilice[k] * real(900.) * real(1.e-3));
        else soiliqw[k] = c_max(real(0.), soilmois[k] - soilice[k] * riw);
      }
    } else {
      soilice[k] = 0;
      soiliqw[k] = soilmois[k];
    }
  }
}

// The layer-mean liquid / frozen split and SMFRKEEP (SOIL and SNOWSOIL)
CPPWRF_HD inline void mid_split(int nzs, const real* tso, const real* soilmois, const real* soilice, real* smfrkeep,
                                const real* keepfr, const SoilParm& sp, real xlmelt, real riw, real* tav,
                                real* soilmoism, real* soiliqwm, real* soilicem, real* fwsat, real* lwsat) {
  const real dqm = sp.dqm, qmin = sp.qmin;
  for (int k = 1; k <= nzs - 1; ++k) {
    tav[k] = real(0.5) * (tso[k] + tso[k + 1]);
    soilmoism[k] = real(0.5) * (soilmois[k] + soilmois[k + 1]);
    const real tavln = libm::log(tav[k] / real(273.15));
    if (tavln < real(0.)) {
      soiliqwm[k] = (dqm + qmin) *
                        libm::pow(xlmelt * (tav[k] - real(273.15)) / tav[k] / real(9.81) / sp.psis,
                                  real(-1.) / sp.bclh) -
                    qmin;
      fwsat[k] = dqm - soiliqwm[k];
      lwsat[k] = soiliqwm[k] + qmin;
      soiliqwm[k] = c_max(real(0.), soiliqwm[k]);
      soiliqwm[k] = c_min(soiliqwm[k], soilmoism[k]);
      soilicem[k] = (soilmoism[k] - soiliqwm[k]) / riw;
      if (keepfr[k] == real(1.)) {
        soilicem[k] = c_min(soilicem[k], real(0.5) * (smfrkeep[k] + smfrkeep[k + 1]));
        soiliqwm[k] = c_max(real(0.), soilmoism[k] - soilicem[k] * riw);
        fwsat[k] = dqm - soiliqwm[k];
        lwsat[k] = soiliqwm[k] + qmin;
      }
    } else {
      soilicem[k] = 0;
      soiliqwm[k] = soilmoism[k];
      lwsat[k] = dqm + qmin;
      fwsat[k] = 0;
    }
  }
  for (int k = 1; k <= nzs; ++k) {
    if (soilice[k] > real(0.)) smfrkeep[k] = soilice[k];
    else smfrkeep[k] = soilmois[k] / riw;
  }
}

// Outputs of SOIL / SNOWSOIL / SICE / SNOWSEAICE shared with SFCTMP
struct Flux {
  real dew, edir1, ec1, ett1, eeta, qfx, hfx, s, evapl, prcpl, fltot, runoff1, runoff2, infiltr, smf, sublim;
};

// SOIL: snow-free land
CPPWRF_HD inline void soil(const Geo& g, const Atm& a, const Consts& c, const RucTables& tb, const SoilParm& sp, int iland,
                           real prcpms, real rainf, real gswin, real emiss, real rnet, real pc, real& cst, real drip,
                           real infwater, real vegfrac, real lai, real sat, real cn, real* soilmois, real* tso,
                           real* smfrkeep, real* keepfr, real& soilt, real& qvg, real& qsg, real& qcg, Flux& f,
                           real& mavail, real* soilice, real* soiliqw) {
  const int nzs = g.nzs;
  const real delt = g.delt;
  const real rhoice = real(900.);
  const real ci = rhoice * real(2100.);
  const real xlmelt = real(3.35e+5);
  const real cvw = c.cw;
  f.prcpl = prcpms;
  f.smf = 0;
  const real soiltold = soilt;
  (void)soiltold;
  real transp[kA] = {}, soilmoism[kA] = {}, soilicem[kA] = {}, soiliqwm[kA] = {}, lwsat[kA] = {}, fwsat[kA] = {},
       tav[kA] = {}, cap[kA] = {}, thdif[kA] = {}, diffu[kA] = {}, hydro[kA] = {}, tranf[kA] = {}, told[kA] = {},
       smold[kA] = {};
  for (int k = 1; k <= nzs; ++k) soilice[k] = soiliqw[k] = 0;
  const real dzstop = real(1.) / (g.zsmain[2] - g.zsmain[1]);
  const real ras = a.rho * real(1.e-3);
  const real riw = rhoice * real(1.e-3);
  frozen_split(nzs, tso, soilmois, smfrkeep, keepfr, sp, xlmelt, riw, false, soiliqw, soilice);
  mid_split(nzs, tso, soilmois, soilice, smfrkeep, keepfr, sp, xlmelt, riw, tav, soilmoism, soiliqwm, soilicem, fwsat,
            lwsat);
  soilprop(nzs, fwsat, lwsat, tav, keepfr, soilmois, soiliqw, soilice, soilmoism, soiliqwm, soilicem, sp, riw, xlmelt,
           c.cp, c.g0, cvw, ci, c, thdif, diffu, hydro, cap);
  f.dew = 0;
  if (a.qvatm >= qsg) f.dew = a.qkms * (a.qvatm - qsg);
  const real wetcan = c_min(real(0.25), libm::pow(c_max(real(0.), (cst / sat)), cn));
  const real drycan = real(1.) - wetcan;
  real transum;
  transf(g.nroot, soiliqw, a.tabs, lai, gswin, sp.qmin, sp.ref, sp.wilt, g.zshalf, pc, iland, tb, tranf, transum, nzs);
  for (int k = 1; k <= nzs; ++k) {
    told[k] = tso[k];
    smold[k] = soilmois[k];
  }
  const real fc = c_max(sp.qmin, sp.ref * real(0.5));
  real soilres;
  if ((soilmois[1] + sp.qmin) > fc || (a.qvatm - qvg) > real(0.)) {
    soilres = real(1.);
  } else {
    real fex_fc = c_min(real(1.), (soilmois[1] + sp.qmin) / fc);
    fex_fc = c_max(fex_fc, real(0.01));
    const real y = real(1.) - libm::cos(kPiconst * fex_fc);
    soilres = real(0.25) * (y * y);
  }
  real x;
  soiltemp(g, a, prcpms, rainf, emiss, rnet, vegfrac, thdif, cap, drycan, wetcan, transum, mavail, soilres, c.lv, c.cp,
           cvw, c.stbolt, tb.tbq, tso, soilt, qvg, qsg, qcg, x);
  f.ett1 = 0;
  f.dew = 0;
  if (a.qvatm >= qsg) {
    f.dew = a.qkms * (a.qvatm - qsg);
    f.ett1 = 0;
    for (int k = 1; k <= nzs; ++k) transp[k] = 0;
  } else {
    for (int k = 1; k <= g.nroot; ++k) {
      transp[k] = vegfrac * ras * a.qkms * (a.qvatm - qsg) * tranf[k] * drycan / g.zshalf[g.nroot + 1];
      if (transp[k] > real(0.)) transp[k] = 0;
      f.ett1 = f.ett1 - transp[k];
    }
    for (int k = g.nroot + 1; k <= nzs; ++k) transp[k] = 0;
  }
  frozen_split(nzs, tso, soilmois, smfrkeep, keepfr, sp, xlmelt, riw, false, soiliqw, soilice);
  real infmax;
  soilmoist(g, riw, diffu, hydro, qsg, qvg, qcg, a.qcatm, a.qvatm, -infwater, a.qkms, transp, drip, f.dew, real(0.),
            soilice, vegfrac, real(0.), soilres, sp, ras, infmax, soilmois, mavail, f.runoff1, f.runoff2, f.infiltr);
  for (int k = 1; k <= nzs; ++k) {
    if (soilice[k] > real(0.)) {
      if (tso[k] > told[k] && soilmois[k] > smold[k]) keepfr[k] = real(1.);
      else keepfr[k] = 0;
    }
  }
  const real hft = -a.tkms * c.cp * a.rho * (a.tabs - soilt);
  f.hfx = -a.tkms * c.cp * a.rho * (a.tabs - soilt) * libm::pow(kP1000mb * real(0.00001) / a.patm, c.rovcp);
  const real q1 = -a.qkms * ras * (a.qvatm - qsg);
  if (q1 <= real(0.)) {
    f.ec1 = 0;
    f.edir1 = 0;
    f.ett1 = 0;
    if (a.myj) {
      f.eeta = -a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qsg / (real(1.) + qsg)) * real(1.e3);
      cst = cst - f.eeta * delt * vegfrac;
    } else {
      f.eeta = -a.rho * f.dew;
      cst = cst + delt * f.dew * ras * vegfrac;
    }
    f.qfx = c.lv * f.eeta;
    f.eeta = -a.rho * f.dew;
  } else {
    f.edir1 = -soilres * (real(1.) - vegfrac) * a.qkms * ras * (a.qvatm - qvg);
    f.ec1 = q1 * wetcan * vegfrac;
    cst = c_max(real(0.), cst - f.ec1 * delt);
    if (a.myj) f.eeta = -soilres * a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qvg / (real(1.) + qvg)) * real(1.e3);
    else f.eeta = (f.edir1 + f.ec1 + f.ett1) * real(1.e3);
    f.qfx = c.lv * f.eeta;
    f.eeta = (f.edir1 + f.ec1 + f.ett1) * real(1.e3);
  }
  f.evapl = f.eeta;
  f.s = thdif[1] * cap[1] * dzstop * (tso[1] - tso[2]);
  f.fltot = rnet - hft - c.lv * f.eeta - f.s - x;
}

// SICE: snow-free sea ice (tice, rhosice unused there)
CPPWRF_HD inline void sice_balance(const Geo& g, const Atm& a, const Consts& c, const RucTables& tb, real prcpms,
                                   real rainf, real emiss, real rnet, const real* capice, const real* thdifice,
                                   real* tso, real& dew, real& soilt, real& qvg, real& qsg, real& qcg, real& eeta,
                                   real& qfx, real& hfx, real& s, real& evapl, real& prcpl, real& fltot) {
  const int nzs = g.nzs, nzs1 = nzs - 1;
  const real delt = g.delt, cp = c.cp;
  const real cvw = c.cw;
  prcpl = prcpms;
  const real dzstop = real(1.) / (g.zsmain[2] - g.zsmain[1]);
  const real ras = a.rho * real(1.e-3);
  real cotso[kA] = {}, rhtso[kA] = {};
  tso_sweep(g, thdifice, tso, cotso, rhtso);
  const real rhcs = capice[1];
  const real d1 = cotso[nzs1];
  const real d2 = rhtso[nzs1];
  const real tn = soilt;
  const real d9 = thdifice[1] * rhcs * dzstop;
  const real d10 = a.tkms * cp * a.rho;
  const real r211 = real(.5) * g.conflx / delt;
  const real r21 = r211 * cp * a.rho;
  const real r22 = real(.5) / (thdifice[1] * delt * (dzstop * dzstop));
  const real r6 = emiss * c.stbolt * real(.5) * powi(tn, 4);
  const real r7 = r6 / tn;
  const real d11 = rnet + r6;
  const real tdenom = d9 * (real(1.) - d1 + r22) + d10 + r21 + r7 + rainf * cvw * prcpms;
  const real fkq = a.qkms * a.rho;
  const real r210 = r211 * a.rho;
  const real aa = kXls * (fkq + r210) / tdenom;
  const real bb = (d10 * a.tabs + r21 * tn + kXls * (a.qvatm * fkq + r210 * qvg) + d11 + d9 * (d2 + r22 * tn) +
                   rainf * cvw * prcpms * c_max(real(273.15), a.tabs)) /
                  tdenom;
  real aa1 = aa;
  const real pp = a.patm * real(1.e3);
  aa1 = aa1 / pp;
  const real qgold = qsg;
  real qs1, ts1;
  vilka(tn, aa1, bb, pp, qs1, ts1, tb.tbq);
  qvg = qs1;
  qsg = qs1;
  tso[1] = c_min(real(271.4), ts1);
  qcg = 0;
  soilt = tso[1];
  for (int k = 2; k <= nzs; ++k) {
    const int kk = nzs - k + 1;
    tso[k] = c_min(real(271.4), rhtso[kk] + cotso[kk] * tso[k - 1]);
  }
  dew = 0;
  const real hft = -a.tkms * cp * a.rho * (a.tabs - soilt);
  hfx = -a.tkms * cp * a.rho * (a.tabs - soilt) * libm::pow(kP1000mb * real(0.00001) / a.patm, c.rovcp);
  const real q1 = -a.qkms * ras * (a.qvatm - qsg);
  if (q1 <= real(0.)) {
    if (a.myj) {
      eeta = -a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qsg / (real(1.) + qsg)) * real(1.e3);
    } else {
      dew = a.qkms * (a.qvatm - qsg);
      eeta = -a.rho * dew;
    }
    qfx = kXls * eeta;
    eeta = -a.rho * dew;
  } else {
    if (a.myj) eeta = -a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qvg / (real(1.) + qvg)) * real(1.e3);
    else eeta = q1 * real(1.e3);
    qfx = kXls * eeta;
    eeta = q1 * real(1.e3);
  }
  evapl = eeta;
  s = thdifice[1] * capice[1] * dzstop * (tso[1] - tso[2]);
  real x = (cp * a.rho * r211 + rhcs * g.zsmain[2] * real(0.5) / delt) * (soilt - tn) +
           kXls * a.rho * r211 * (qsg - qgold);
  x = x - rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soilt);
  const real icemelt = rnet - kXls * eeta - hft - s - x;
  fltot = rnet - kXls * eeta - hft - s - x - icemelt;
}

// The snow-layer coefficients shared by SNOWTEMP and SNOWSEAICE (thd: thdif or thdifice); thick_snhei: the snow
// depth of the two-layer case's upper half-layer distance (SNOWTEMP: deltsn, SNOWSEAICE: snhei)
struct SnowLayers {
  real snprim = 0, tsob = 0, cotsn = 0, rhtsn = 0, fsn = 1, fso = 0;
};
CPPWRF_HD inline void snow_layers(const Geo& g, real snhei, real snth, real deltsn, real thdifsn, const real* thd,
                                  const real* tso, real soilt, real& soilt1, real& tsnav, int& ilnb, real* cotso,
                                  real* rhtso, SnowLayers& L, bool seaice) {
  const int nzs = g.nzs, nzs1 = nzs - 1;
  const real delt = g.delt;
  if (snhei >= snth) {
    if (snhei <= deltsn + snth) {
      ilnb = 1;
      L.snprim = c_max(snth, snhei);
      if (seaice) {
        soilt1 = tso[1];
        L.tsob = tso[1];
      } else {
        L.tsob = tso[1];
        soilt1 = tso[1];
      }
      const real xsn = delt / real(2.) / (g.zshalf[2] + real(0.5) * L.snprim);
      const real ddzsn = xsn / L.snprim;
      const real x1sn = ddzsn * thdifsn;
      const real x2 = g.dtdzs[1] * thd[1];
      const real ft = tso[1] + x1sn * (soilt - tso[1]) - x2 * (tso[1] - tso[2]);
      const real denom = real(1.) + x1sn + x2 - x2 * cotso[nzs1];
      cotso[nzs] = x1sn / denom;
      rhtso[nzs] = (ft + x2 * rhtso[nzs1]) / denom;
      L.cotsn = cotso[nzs];
      L.rhtsn = rhtso[nzs];
      tsnav = real(0.5) * (soilt + tso[1]) - real(273.15);
    } else {
      ilnb = 2;
      L.snprim = deltsn;
      L.tsob = soilt1;
      const real xsn = seaice ? delt / real(2.) / (real(0.5) * snhei) : delt / real(2.) / (real(0.5) * deltsn);
      const real xsn1 = delt / real(2.) / (g.zshalf[2] + real(0.5) * (snhei - deltsn));
      const real ddzsn = xsn / deltsn;
      const real ddzsn1 = xsn1 / (snhei - deltsn);
      const real x1sn = ddzsn * thdifsn;
      const real x1sn1 = ddzsn1 * thdifsn;
      const real x2 = g.dtdzs[1] * thd[1];
      const real ft = tso[1] + x1sn1 * (soilt1 - tso[1]) - x2 * (tso[1] - tso[2]);
      const real denom = real(1.) + x1sn1 + x2 - x2 * cotso[nzs1];
      cotso[nzs] = x1sn1 / denom;
      rhtso[nzs] = (ft + x2 * rhtso[nzs1]) / denom;
      const real ftsnow = soilt1 + x1sn * (soilt - soilt1) - x1sn1 * (soilt1 - tso[1]);
      const real denomsn = real(1.) + x1sn + x1sn1 - x1sn1 * cotso[nzs];
      L.cotsn = x1sn / denomsn;
      L.rhtsn = (ftsnow + x1sn1 * rhtso[nzs]) / denomsn;
      tsnav = real(0.5) / snhei * ((soilt + soilt1) * deltsn + (soilt1 + tso[1]) * (snhei - deltsn)) - real(273.15);
    }
  }
  if (snhei < snth && snhei > real(0.)) {
    L.snprim = snhei + g.zsmain[2];
    L.fsn = snhei / L.snprim;
    L.fso = real(1.) - L.fsn;
    soilt1 = tso[1];
    L.tsob = tso[2];
    const real xsn = delt / real(2.) / ((g.zshalf[3] - g.zsmain[2]) + real(0.5) * L.snprim);
    const real ddzsn = xsn / L.snprim;
    const real x1sn = ddzsn * (L.fsn * thdifsn + L.fso * thd[1]);
    const real x2 = g.dtdzs[2] * thd[2];
    const real ft = tso[2] + x1sn * (soilt - tso[2]) - x2 * (tso[2] - tso[3]);
    const real denom = real(1.) + x1sn + x2 - x2 * cotso[nzs - 2];
    cotso[nzs1] = x1sn / denom;
    rhtso[nzs1] = (ft + x2 * rhtso[nzs - 2]) / denom;
    tsnav = real(0.5) * (soilt + tso[1]) - real(273.15);
    cotso[nzs] = cotso[nzs1];
    rhtso[nzs] = rhtso[nzs1];
    L.cotsn = cotso[nzs];
    L.rhtsn = rhtso[nzs];
  }
}

// SNOWTEMP's snow thermal diffusivity (isncond_opt = 2)
CPPWRF_HD inline real snow_thdif(real rhosn, real rhocsn, real newsnow, real rhonewsn, real snhei) {
  const real fact = real(1.);
  real keff;
  if (rhosn < real(156.) || (newsnow > real(0.) && rhonewsn < real(156.))) {
    keff = real(0.023) + real(0.234) * rhosn * real(1.e-3);
  } else {
    keff = real(0.138) - real(1.01) * rhosn * real(1.e-3) + real(3.233) * (rhosn * rhosn) * real(1.e-6);
  }
  if (newsnow <= real(0.) && snhei > real(1.) && rhosn > real(250.)) return real(4.431718e-7);
  return keff / rhocsn * fact;
}

// SNOWTEMP: the energy balance of the snow on land
CPPWRF_HD inline void snowtemp(const Geo& g, const Atm& a, const RucTables& tb, real& snwe, real snwepr, real& snhei,
                               real newsnow, real snowfrac, real& beta, real deltsn, real snth, real& rhosn,
                               real rhonewsn, real meltfactor, real prcpms, real rainf, real emiss, real rnet,
                               real vegfrac, const real* thdif, const real* cap, real drycan, real wetcan, real cst,
                               const real* tranf, real transum, real& dew, real xlvm, real cp, real cvw, real stbolt,
                               real& snweprint, real& snheiprint, real& rsm, real* tso, real& soilt, real& soilt1,
                               real& tsnav, real& qvg, real& qsg, real& qcg, real& smelt, real& snoh, real& snflx,
                               real& s, int& ilnb, real& x) {
  (void)cst;
  const int nzs = g.nzs, nzs1 = nzs - 1;
  const real delt = g.delt;
  const real* zshalf = g.zshalf;
  const int iter = 0;
  real transp[kA] = {}, cotso[kA] = {}, rhtso[kA] = {};
  const real xlmelt = real(3.35e+5);
  real rhocsn = real(2090.) * rhosn;
  const real rhonewcsn = real(2090.) * rhonewsn;
  real thdifsn = snow_thdif(rhosn, rhocsn, newsnow, rhonewsn, snhei);
  const real ras = a.rho * real(1.e-3);
  real soiltfrac = soilt;
  smelt = 0;
  real soh = 0;
  real smeltg = 0, snohg = 0;
  rsm = 0;
  real rsmfrac = 0;
  const real qgold = qvg;
  const real dzstop = real(1.) / (g.zsmain[2] - g.zsmain[1]);
  tso_sweep(g, thdif, tso, cotso, rhtso);
  SnowLayers L;
  snow_layers(g, snhei, snth, deltsn, thdifsn, thdif, tso, soilt, soilt1, tsnav, ilnb, cotso, rhtso, L, false);
  int nmelt = 0;
  snoh = 0;
  real ett1 = 0;
  real epot = -a.qkms * (a.qvatm - qgold);
  const real rhcs = cap[1];
  const real h = real(1.);
  const real trans = transum * drycan / zshalf[g.nroot + 1];
  const real can = wetcan + trans;
  const real umveg = real(1.) - vegfrac;
  const real d1 = cotso[nzs1];
  const real d2 = rhtso[nzs1];
  const real tn = soilt;
  const real d9 = thdif[1] * rhcs * dzstop;
  const real d10 = a.tkms * cp * a.rho;
  const real r211 = real(.5) * g.conflx / delt;
  const real r21 = r211 * cp * a.rho;
  const real r22 = real(.5) / (thdif[1] * delt * (dzstop * dzstop));
  const real r6 = emiss * stbolt * real(.5) * powi(tn, 4);
  const real r7 = r6 / tn;
  const real d11 = rnet + r6;
  real d1sn = 0, d2sn = 0, d9sn = 0, r22sn = 0;
  if (snhei >= snth) {
    if (snhei <= deltsn + snth) {
      d1sn = cotso[nzs];
      d2sn = rhtso[nzs];
    } else {
      d1sn = L.cotsn;
      d2sn = L.rhtsn;
    }
    d9sn = thdifsn * rhocsn / L.snprim;
    r22sn = L.snprim * L.snprim * real(0.5) / (thdifsn * delt);
  }
  if (snhei < snth && snhei > real(0.)) {
    d1sn = d1;
    d2sn = d2;
    d9sn = (L.fsn * thdifsn * rhocsn + L.fso * thdif[1] * rhcs) / L.snprim;
    r22sn = L.snprim * L.snprim * real(0.5) / ((L.fsn * thdifsn + L.fso * thdif[1]) * delt);
  }
  if (snhei == real(0.)) {
    d9sn = d9;
    r22sn = r22;
    d1sn = d1;
    d2sn = d2;
  }
  const real pp = a.patm * real(1.e3);
  real r210 = 0;
  for (;;) {  // label 212 (a second pass after the melt)
    const real tdenom = d9sn * (real(1.) - d1sn + r22sn) + d10 + r21 + r7 + rainf * cvw * prcpms +
                        rhonewcsn * newsnow / delt;
    const real fkq = a.qkms * a.rho;
    r210 = r211 * a.rho;
    const real c = vegfrac * fkq * can;
    const real cc = c * xlvm / tdenom;
    real aa = xlvm * (beta * fkq * umveg + r210) / tdenom;
    real bb = (d10 * a.tabs + r21 * tn + xlvm * (a.qvatm * (beta * fkq * umveg + c) + r210 * qgold) + d11 +
               d9sn * (d2sn + r22sn * tn) + rainf * cvw * prcpms * c_max(real(273.15), a.tabs) +
               rhonewcsn * newsnow / delt * c_min(real(273.15), a.tabs)) /
              tdenom;
    real aa1 = aa + cc;
    aa1 = aa1 / pp;
    bb = bb - snoh / tdenom;
    real qs1, ts1;
    vilka(tn, aa1, bb, pp, qs1, ts1, tb.tbq);
    const real tq2 = a.qvatm;
    const real tx2 = tq2 * (real(1.) - h);
    real q1 = tx2 + h * qs1;
    bool sat = !(q1 < qs1);
    if (!sat) {  // label 100
      bb = bb - aa * tx2;
      aa = (aa * h + cc) / pp;
      vilka(tn, aa, bb, pp, qs1, ts1, tb.tbq);
      q1 = tx2 + h * qs1;
      sat = q1 > qs1;
      if (!sat) {
        qsg = qs1;
        qvg = q1;
        qcg = 0;
      }
    }
    if (sat) {  // label 90
      qvg = qs1;
      qsg = qs1;
      qcg = c_max(real(0.), q1 - qs1);
    }
    soilt = ts1;
    if (nmelt == 1 && snowfrac == real(1.) && snwe > real(0.) && soilt > real(273.15))
      soilt = c_min(real(273.15), soilt);
    if (snhei >= snth) {
      if (snhei > deltsn + snth) {
        soilt1 = c_min(real(273.15), L.rhtsn + L.cotsn * soilt);
        tso[1] = rhtso[nzs] + cotso[nzs] * soilt1;
        L.tsob = soilt1;
      } else {
        tso[1] = rhtso[nzs] + cotso[nzs] * soilt;
        soilt1 = tso[1];
        L.tsob = tso[1];
      }
    } else if (snhei > real(0.) && snhei < snth) {
      tso[2] = rhtso[nzs1] + cotso[nzs1] * soilt;
      tso[1] = (tso[2] + (soilt - tso[2]) * L.fso);
      soilt1 = tso[1];
      L.tsob = tso[2];
    } else {
      tso[1] = soilt;
      soilt1 = soilt;
      L.tsob = tso[1];
    }
    if (nmelt == 1 && snowfrac == real(1.)) {
      soilt1 = c_min(real(273.15), soilt1);
      tso[1] = c_min(real(273.15), tso[1]);
      L.tsob = c_min(real(273.15), L.tsob);
    }
    if (snhei > real(0.) && snhei < snth) {
      for (int k = 3; k <= nzs; ++k) {
        const int kk = nzs - k + 1;
        tso[k] = rhtso[kk] + cotso[kk] * tso[k - 1];
      }
    } else {
      for (int k = 2; k <= nzs; ++k) {
        const int kk = nzs - k + 1;
        tso[k] = rhtso[kk] + cotso[kk] * tso[k - 1];
      }
    }
    if (nmelt == 1) break;  // go to 220
    if (soilt > real(273.15) && beta == real(1.) && snhei > real(0.)) {
      nmelt = 1;
      soiltfrac = snowfrac * real(273.15) + (real(1.) - snowfrac) * soilt;
      qsg = c_min(qsg, qsn(soiltfrac, tb.tbq) / pp);
      qvg = snowfrac * qsg + (real(1.) - snowfrac) * qvg;
      epot = -a.qkms * (a.qvatm - qsg);
      q1 = epot * ras;
      real qfx, eeta;
      if (q1 <= real(0.) || iter == 1) {
        dew = -epot;
        for (int k = 1; k <= nzs; ++k) transp[k] = 0;
        qfx = -xlvm * a.rho * dew;
        eeta = qfx / xlvm;
      } else {
        for (int k = 1; k <= g.nroot; ++k) {
          transp[k] = -vegfrac * q1 * tranf[k] * drycan / zshalf[g.nroot + 1];
          ett1 = ett1 - transp[k];
        }
        for (int k = g.nroot + 1; k <= nzs; ++k) transp[k] = 0;
        const real edir1 = q1 * umveg * beta;
        const real ec1 = q1 * wetcan * vegfrac;
        eeta = (edir1 + ec1 + ett1) * real(1.e3);
        qfx = xlvm * eeta;
      }
      const real hfx = -d10 * (a.tabs - soiltfrac);
      if (snhei >= snth) soh = thdifsn * rhocsn * (soiltfrac - L.tsob) / L.snprim;
      else soh = (L.fsn * thdifsn * rhocsn + L.fso * thdif[1] * rhcs) * (soiltfrac - L.tsob) / L.snprim;
      snflx = soh;
      x = (r21 + d9sn * r22sn) * (soiltfrac - tn) + xlvm * r210 * (qvg - qgold);
      snoh = rnet - qfx - hfx - soh - x + rhonewcsn * newsnow / delt * (c_min(real(273.15), a.tabs) - soiltfrac) +
             rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soiltfrac);
      snoh = c_max(real(0.), snoh);
      smelt = snoh / xlmelt * real(1.e-3);
      bool skip88 = false;
      if (epot > real(0.) && snwepr <= epot * ras * delt) {
        beta = snwepr / (epot * ras * delt);
        smelt = c_min(smelt, snwepr / delt - beta * epot * ras);
        snwe = 0;
        skip88 = true;
      }
      if (!skip88) {
        smelt = c_max(real(0.), smelt);
        if ((rhosn < real(350.) || (newsnow > real(0.) && rhonewsn < real(450.))) && soilt < real(283.))
          smelt = c_min(smelt, delt / real(60.) * real(5.6e-8) * meltfactor * c_max(real(1.), (soilt - real(273.15))));
        const real rr = c_max(real(0.), snwepr / delt - beta * epot * ras);
        if (smelt > rr) {
          smelt = c_min(smelt, rr);
          snwe = 0;
        }
      }
      // label 88
      const real snohgnew = smelt * xlmelt * real(1.e3);
      snoh = snohgnew;
      if (smelt > real(0.)) {
        rsmfrac = c_min(real(0.18), (c_max(real(0.08), snwepr / real(0.10) * real(0.13))));
        if (snhei > real(0.01) && rhosn < real(350.)) rsm = rsmfrac * smelt * delt;
        else rsm = 0;
        if (rsm > real(0.)) smelt = c_max(real(0.), smelt - rsm / delt);
      }
      if (snwe > real(0.)) snwe = c_max(real(0.), (snwepr - (smelt + beta * epot * ras) * delt));
    } else {
      if (snhei != real(0.) && beta == real(1.)) {
        epot = -a.qkms * (a.qvatm - qsg);
        snwe = c_max(real(0.), (snwepr - beta * epot * ras * delt));
      } else {
        snwe = 0;
      }
    }
    if (nmelt != 1) break;  // nmelt = 1: the second interation (goto 212)
  }
  // label 220
  if (smelt > real(0.) && rsm > real(0.)) {
    if (!(snwe <= rsm)) {
      const real xsn = (rhosn * (snwe - rsm) + real(1.e3) * rsm) / snwe;
      rhosn = c_min(c_max(real(58.8), xsn), real(500.));
      rhocsn = real(2090.) * rhosn;
      thdifsn = snow_thdif(rhosn, rhocsn, newsnow, rhonewsn, snhei);
    }
  }
  if (snhei >= snth) {
    s = thdifsn * rhocsn * (soilt - L.tsob) / L.snprim;
    snflx = s;
    s = d9 * (tso[1] - tso[2]);
  } else if (snhei < snth && snhei > real(0.)) {
    s = (L.fsn * thdifsn * rhocsn + L.fso * thdif[1] * rhcs) * (soilt - L.tsob) / L.snprim;
    snflx = s;
    s = d9 * (tso[1] - tso[2]);
  } else {
    s = d9sn * (soilt - L.tsob);
    snflx = s;
    s = d9 * (tso[1] - tso[2]);
  }
  snhei = snwe * real(1.e3) / rhosn;
  if (tso[1] > real(273.15) && snhei > real(0.)) {
    const real hsn = snhei > deltsn + snth ? snhei - deltsn : snhei;
    soiltfrac = snowfrac * real(273.15) + (real(1.) - snowfrac) * tso[1];
    snohg = (tso[1] - soiltfrac) * (cap[1] * zshalf[2] + rhocsn * real(0.5) * hsn) / delt;
    snohg = c_max(real(0.), snohg);
    smeltg = snohg / xlmelt * real(1.e-3);
    if ((rhosn < real(350.) || (newsnow > real(0.) && rhonewsn < real(450.))) && soilt < real(283.))
      smeltg = c_min(smeltg, real(5.8e-9));
    const real rr = snwe / delt;
    smeltg = c_min(smeltg, rr);
    snwe = c_max(real(0.), snwe - smeltg * delt);
    snhei = snwe * real(1.e3) / rhosn;
    smelt = smelt + smeltg;
    if (snhei > real(0.)) tso[1] = soiltfrac;
  }
  snweprint = snwe;
  snheiprint = snweprint * real(1.e3) / rhosn;
  x = (r21 + d9sn * r22sn) * (soilt - tn) + xlvm * r210 * (qsg - qgold);
  x = x - rhonewcsn * newsnow / delt * (c_min(real(273.15), a.tabs) - soilt) -
      rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soilt);
  if (snhei > real(0.)) {
    if (ilnb > 1) {
      tsnav = real(0.5) / snhei * ((soilt + soilt1) * deltsn + (soilt1 + tso[1]) * (snhei - deltsn)) - real(273.15);
    } else {
      tsnav = real(0.5) * (soilt + tso[1]) - real(273.15);
    }
  } else {
    tsnav = soilt - real(273.15);
  }
}

// SNOWSOIL: snow on land
CPPWRF_HD inline void snowsoil(const Geo& g, const Atm& a, const Consts& c, const RucTables& tb, const SoilParm& sp,
                               real meltfactor, real rhonewsn, int iland, real prcpms, real rainf, real newsnow,
                               real& snhei, real& snwe, real snowfrac, real& rhosn, real gsw, real gswin, real emiss,
                               real rnet, real pc, real& cst, real infwater, real vegfrac, real lai, real sat, real cn,
                               int& ilnb, real& snweprint, real& snheiprint, real& rsm, real* soilmois, real* tso,
                               real* smfrkeep, real* keepfr, real& soilt, real& soilt1, real& tsnav, real& qvg,
                               real& qsg, real& qcg, real& smelt, real& snoh, real& snflx, real& snom, Flux& f,
                               real& mavail, real* soilice, real* soiliqw) {
  (void)gsw;
  const int nzs = g.nzs;
  const real delt = g.delt;
  const real cvw = c.cw;
  const real xlmelt = real(3.35e+5);
  const real xlvm = c.lv + xlmelt;
  real deltsn = real(0.05) * real(1.e3) / rhosn;
  const real snth = real(0.01) * real(1.e3) / rhosn;
  if (snhei >= deltsn + snth) {
    if (snhei - deltsn - snth < snth) deltsn = real(0.5) * (snhei - snth);
  }
  const real rhoice = real(900.);
  const real ci = rhoice * real(2100.);
  const real ras = a.rho * real(1.e-3);
  const real riw = rhoice * real(1.e-3);
  rsm = 0;
  real transp[kA] = {}, soilmoism[kA] = {}, soiliqwm[kA] = {}, soilicem[kA] = {}, lwsat[kA] = {}, fwsat[kA] = {},
       tav[kA] = {}, cap[kA] = {}, diffu[kA] = {}, hydro[kA] = {}, thdif[kA] = {}, tranf[kA] = {}, told[kA] = {},
       smold[kA] = {};
  for (int k = 1; k <= nzs; ++k) soilice[k] = 0;
  snweprint = 0;
  snheiprint = 0;
  f.prcpl = prcpms;
  frozen_split(nzs, tso, soilmois, smfrkeep, keepfr, sp, xlmelt, riw, true, soiliqw, soilice);
  mid_split(nzs, tso, soilmois, soilice, smfrkeep, keepfr, sp, xlmelt, riw, tav, soilmoism, soiliqwm, soilicem, fwsat,
            lwsat);
  soilprop(nzs, fwsat, lwsat, tav, keepfr, soilmois, soiliqw, soilice, soilmoism, soiliqwm, soilicem, sp, riw, xlmelt,
           c.cp, c.g0, cvw, ci, c, thdif, diffu, hydro, cap);
  smelt = 0;
  const real fq = a.qkms;
  f.dew = 0;
  const real umveg = real(1.) - vegfrac;
  real epot = -fq * (a.qvatm - qsg);
  const real snwepr = snwe;
  real beta = real(1.);
  const real epdt = epot * ras * delt * umveg;
  if (epdt > real(0.) && snwepr <= epdt) {
    beta = snwepr / c_max(real(1.e-8), epdt);
    snwe = 0;
  }
  const real wetcan = c_min(real(0.25), libm::pow(c_max(real(0.), (cst / sat)), cn));
  const real drycan = real(1.) - wetcan;
  real transum;
  transf(g.nroot, soiliqw, a.tabs, lai, gswin, sp.qmin, sp.ref, sp.wilt, g.zshalf, pc, iland, tb, tranf, transum, nzs);
  for (int k = 1; k <= nzs; ++k) {
    told[k] = tso[k];
    smold[k] = soilmois[k];
  }
  real x = 0;
  snowtemp(g, a, tb, snwe, snwepr, snhei, newsnow, snowfrac, beta, deltsn, snth, rhosn, rhonewsn, meltfactor, prcpms,
           rainf, emiss, rnet, vegfrac, thdif, cap, drycan, wetcan, cst, tranf, transum, f.dew, xlvm, c.cp, cvw,
           c.stbolt, snweprint, snheiprint, rsm, tso, soilt, soilt1, tsnav, qvg, qsg, qcg, smelt, snoh, snflx, f.s, ilnb,
           x);
  f.dew = 0;
  f.ett1 = 0;
  epot = -fq * (a.qvatm - qsg);
  if (epot > real(0.)) {
    for (int k = 1; k <= g.nroot; ++k) {
      transp[k] = vegfrac * ras * fq * (a.qvatm - qsg) * tranf[k] * drycan / g.zshalf[g.nroot + 1];
      f.ett1 = f.ett1 - transp[k];
    }
    for (int k = g.nroot + 1; k <= nzs; ++k) transp[k] = 0;
  } else {
    f.dew = -epot;
    for (int k = 1; k <= nzs; ++k) transp[k] = 0;
    f.ett1 = 0;
  }
  frozen_split(nzs, tso, soilmois, smfrkeep, keepfr, sp, xlmelt, riw, false, soiliqw, soilice);
  real infmax;
  soilmoist(g, riw, diffu, hydro, qsg, qvg, qcg, a.qcatm, a.qvatm, -infwater, a.qkms, transp, real(0.), real(0.), smelt,
            soilice, vegfrac, snowfrac, real(1.), sp, ras, infmax, soilmois, mavail, f.runoff1, f.runoff2, f.infiltr);
  if (snhei == real(0.)) tsnav = soilt - real(273.15);
  snom = snom + smelt * delt * real(1.e3);
  for (int k = 1; k <= nzs; ++k) {
    if (soilice[k] > real(0.)) {
      if (tso[k] > told[k] && soilmois[k] > smold[k]) keepfr[k] = real(1.);
      else keepfr[k] = 0;
    }
  }
  f.hfx = -a.tkms * c.cp * a.rho * (a.tabs - soilt) * libm::pow(kP1000mb * real(0.00001) / a.patm, c.rovcp);
  const real hft = -a.tkms * c.cp * a.rho * (a.tabs - soilt);
  const real q1 = -fq * ras * (a.qvatm - qsg);
  if (q1 < real(0.)) {
    f.edir1 = 0;
    f.ec1 = 0;
    f.ett1 = 0;
    if (a.myj) {
      f.eeta = -a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qsg / (real(1.) + qsg)) * real(1.e3);
      cst = cst - f.eeta * delt * vegfrac;
    } else {
      f.dew = a.qkms * (a.qvatm - qsg);
      f.eeta = -a.rho * f.dew;
      cst = cst + delt * f.dew * ras * vegfrac;
    }
    f.qfx = xlvm * f.eeta;
    f.eeta = -a.rho * f.dew;
  } else {
    f.edir1 = q1 * umveg * beta;
    f.ec1 = q1 * wetcan * vegfrac;
    cst = c_max(real(0.), cst - f.ec1 * delt);
    if (a.myj) f.eeta = -(a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qsg / (real(1.) + qsg)) * real(1.e3)) * beta;
    else f.eeta = (f.edir1 + f.ec1 + f.ett1) * real(1.e3);
    f.qfx = xlvm * f.eeta;
    f.eeta = (f.edir1 + f.ec1 + f.ett1) * real(1.e3);
  }
  f.s = snflx;
  f.sublim = f.edir1 * real(1.e3);
  f.fltot = rnet - hft - xlvm * f.eeta - f.s - snoh - x;
}

// SNOWSEAICE: snow on sea ice
CPPWRF_HD inline void snowseaice(const Geo& g, const Atm& a, const Consts& c, const RucTables& tb, real meltfactor,
                                 real rhonewsn, real prcpms, real rainf, real newsnow, real& snhei, real& snwe,
                                 real snowfrac, real& rhosn, real& emiss, real rnet, real& alb, real& znt,
                                 const real* capice, const real* thdifice, int& ilnb, real& snweprint,
                                 real& snheiprint, real& rsm, real* tso, real& dew, real& soilt, real& soilt1,
                                 real& tsnav, real& qvg, real& qsg, real& qcg, real& smelt, real& snoh, real& snflx,
                                 real& snom, real& eeta, real& qfx, real& hfx, real& s, real& sublim, real& prcpl,
                                 real& fltot) {
  (void)meltfactor;
  const int nzs = g.nzs, nzs1 = nzs - 1;
  const real delt = g.delt, cp = c.cp;
  const real xlmelt = real(3.35e+5);
  const real xlvm = c.lv + xlmelt;
  real deltsn = real(0.05) * real(1.e3) / rhosn;
  const real snth = real(0.01) * real(1.e3) / rhosn;
  if (snhei >= deltsn + snth) {
    if (snhei - deltsn - snth < snth) deltsn = real(0.5) * (snhei - snth);
  }
  const real ras = a.rho * real(1.e-3);
  rsm = 0;
  real rhocsn = real(2090.) * rhosn;
  const real rhonewcsn = real(2090.) * rhonewsn;
  real thdifsn = real(0.265) / rhocsn;
  real soiltfrac = soilt;
  smelt = 0;
  real soh = 0;
  snoh = 0;
  real rsmfrac = 0;
  const real cvw = c.cw;
  const real qgold = qsg;
  const real tnold = soilt;
  const real dzstop = real(1.) / (g.zsmain[2] - g.zsmain[1]);
  snweprint = 0;
  snheiprint = 0;
  prcpl = prcpms;
  const real fq = a.qkms;
  snhei = snwe * real(1.e3) / rhosn;
  const real snwepr = snwe;
  real beta = real(1.);
  real epot = -fq * (a.qvatm - qsg);
  const real epdt = epot * ras * delt;
  if (epdt > real(0.) && snwepr <= epdt) {
    beta = snwepr / c_max(real(1.e-8), epdt);
    snwe = 0;
  }
  real cotso[kA] = {}, rhtso[kA] = {};
  tso_sweep(g, thdifice, tso, cotso, rhtso);
  SnowLayers L;
  snow_layers(g, snhei, snth, deltsn, thdifsn, thdifice, tso, soilt, soilt1, tsnav, ilnb, cotso, rhtso, L, true);
  int nmelt = 0;
  snoh = 0;
  epot = -a.qkms * (a.qvatm - qsg);
  const real rhcs = capice[1];
  const real d1 = cotso[nzs1];
  const real d2 = rhtso[nzs1];
  const real tn = soilt;
  const real d9 = thdifice[1] * rhcs * dzstop;
  const real d10 = a.tkms * cp * a.rho;
  const real r211 = real(.5) * g.conflx / delt;
  const real r21 = r211 * cp * a.rho;
  const real r22 = real(.5) / (thdifice[1] * delt * (dzstop * dzstop));
  const real r6 = emiss * c.stbolt * real(.5) * powi(tn, 4);
  const real r7 = r6 / tn;
  const real d11 = rnet + r6;
  real d1sn = 0, d2sn = 0, d9sn = 0, r22sn = 0;
  if (snhei >= snth) {
    if (snhei <= deltsn + snth) {
      d1sn = cotso[nzs];
      d2sn = rhtso[nzs];
    } else {
      d1sn = L.cotsn;
      d2sn = L.rhtsn;
    }
    d9sn = thdifsn * rhocsn / L.snprim;
    r22sn = L.snprim * L.snprim * real(0.5) / (thdifsn * delt);
  }
  if (snhei < snth && snhei > real(0.)) {
    d1sn = d1;
    d2sn = d2;
    d9sn = (L.fsn * thdifsn * rhocsn + L.fso * thdifice[1] * rhcs) / L.snprim;
    r22sn = L.snprim * L.snprim * real(0.5) / ((L.fsn * thdifsn + L.fso * thdifice[1]) * delt);
  }
  if (snhei == real(0.)) {
    d9sn = d9;
    r22sn = r22;
    d1sn = d1;
    d2sn = d2;
  }
  const real tdenom = d9sn * (real(1.) - d1sn + r22sn) + d10 + r21 + r7 + rainf * cvw * prcpms +
                      rhonewcsn * newsnow / delt;
  const real fkq = a.qkms * a.rho;
  const real r210 = r211 * a.rho;
  const real aa = xlvm * (beta * fkq + r210) / tdenom;
  real bb = (d10 * a.tabs + r21 * tn + xlvm * (a.qvatm * (beta * fkq) + r210 * qvg) + d11 + d9sn * (d2sn + r22sn * tn) +
             rainf * cvw * prcpms * c_max(real(273.15), a.tabs) +
             rhonewcsn * newsnow / delt * c_min(real(273.15), a.tabs)) /
            tdenom;
  real aa1 = aa;
  const real pp = a.patm * real(1.e3);
  aa1 = aa1 / pp;
  bb = bb - snoh / tdenom;
  real qs1, ts1;
  vilka(tn, aa1, bb, pp, qs1, ts1, tb.tbq);
  qvg = qs1;
  qsg = qs1;
  qcg = 0;
  soilt = ts1;
  if (snhei >= snth) {
    if (snhei > deltsn + snth) {
      soilt1 = c_min(real(273.15), L.rhtsn + L.cotsn * soilt);
      tso[1] = c_min(real(271.4), (rhtso[nzs] + cotso[nzs] * soilt1));
      L.tsob = soilt1;
    } else {
      tso[1] = c_min(real(271.4), (rhtso[nzs] + cotso[nzs] * soilt));
      soilt1 = tso[1];
      L.tsob = tso[1];
    }
  } else if (snhei > real(0.) && snhei < snth) {
    tso[2] = c_min(real(271.4), (rhtso[nzs1] + cotso[nzs1] * soilt));
    tso[1] = c_min(real(271.4), (tso[2] + (soilt - tso[2]) * L.fso));
    soilt1 = tso[1];
    L.tsob = tso[2];
  } else {
    tso[1] = c_min(real(271.4), soilt);
    soilt1 = c_min(real(271.4), soilt);
    L.tsob = tso[1];
  }
  if (snhei > real(0.) && snhei < snth) {
    for (int k = 3; k <= nzs; ++k) {
      const int kk = nzs - k + 1;
      tso[k] = c_min(real(271.4), rhtso[kk] + cotso[kk] * tso[k - 1]);
    }
  } else {
    for (int k = 2; k <= nzs; ++k) {
      const int kk = nzs - k + 1;
      tso[k] = c_min(real(271.4), rhtso[kk] + cotso[kk] * tso[k - 1]);
    }
  }
  real x = 0;
  if (nmelt != 1) {
    if (soilt > real(273.15) && snwepr - beta * epot * ras * delt > real(0.) && snhei > real(0.)) {
      nmelt = 1;
      soiltfrac = snowfrac * real(273.15) + (real(1.) - snowfrac) * c_min(real(271.4), soilt);
      qsg = qsn(soiltfrac, tb.tbq) / pp;
      epot = -a.qkms * (a.qvatm - qsg);
      const real q1 = epot * ras;
      if (q1 <= real(0.)) {
        dew = -epot;
        qfx = xlvm * a.rho * dew;
        eeta = qfx / xlvm;
      } else {
        eeta = q1 * beta * real(1.e3);
        qfx = -xlvm * eeta;
      }
      hfx = d10 * (a.tabs - soiltfrac);
      if (snhei >= snth) soh = thdifsn * rhocsn * (soiltfrac - L.tsob) / L.snprim;
      else soh = (L.fsn * thdifsn * rhocsn + L.fso * thdifice[1] * rhcs) * (soiltfrac - L.tsob) / L.snprim;
      snflx = soh;
      x = (r21 + d9sn * r22sn) * (soiltfrac - tnold) + xlvm * r210 * (qsg - qgold);
      snoh = rnet + qfx + hfx + rhonewcsn * newsnow / delt * (c_min(real(273.15), a.tabs) - soiltfrac) - soh - x +
             rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soiltfrac);
      snoh = c_max(real(0.), snoh);
      smelt = snoh / xlmelt * real(1.e-3);
      smelt = c_min(smelt, snwepr / delt - beta * epot * ras);
      smelt = c_max(real(0.), smelt);
      smelt = c_min(smelt, real(5.6e-8) * meltfactor * c_max(real(1.), (soilt - real(273.15))));
      const real rr = snwepr / delt - beta * epot * ras;
      smelt = c_min(smelt, rr);
      const real snohgnew = smelt * xlmelt * real(1.e3);
      snoh = snohgnew;
      rsmfrac = c_min(real(0.18), (c_max(real(0.08), snwepr / real(0.10) * real(0.13))));
      if (snhei > real(0.01)) rsm = rsmfrac * smelt * delt;
      else rsm = 0;
      smelt = c_max(real(0.), smelt - rsm / delt);
      snwe = c_max(real(0.), (snwepr - (smelt + beta * epot * ras) * delt));
      soilt = soiltfrac;
    } else {
      if (snhei != real(0.)) {
        epot = -a.qkms * (a.qvatm - qsg);
        snwe = c_max(real(0.), (snwepr - beta * epot * ras * delt));
      }
    }
  }
  // label 220
  if (smelt > real(0.) && rsm > real(0.)) {
    if (!(snwe <= rsm)) {
      const real xsn = (rhosn * (snwe - rsm) + real(1.e3) * rsm) / snwe;
      rhosn = c_min(c_max(real(58.8), xsn), real(500.));
      rhocsn = real(2090.) * rhosn;
      thdifsn = real(0.265) / rhocsn;
    }
  }
  snweprint = snwe;
  snheiprint = snweprint * real(1.e3) / rhosn;
  if (snhei > real(0.)) {
    if (ilnb > 1) {
      tsnav = real(0.5) / snhei * ((soilt + soilt1) * deltsn + (soilt1 + tso[1]) * (snhei - deltsn)) - real(273.15);
    } else {
      tsnav = real(0.5) * (soilt + tso[1]) - real(273.15);
    }
  }
  dew = 0;
  qsg = qsn(soilt, tb.tbq) / pp;
  epot = -fq * (a.qvatm - qsg);
  if (epot < real(0.)) dew = -epot;
  snom = snom + smelt * delt * real(1.e3);
  const real hft = -a.tkms * cp * a.rho * (a.tabs - soilt);
  hfx = -a.tkms * cp * a.rho * (a.tabs - soilt) * libm::pow(kP1000mb * real(0.00001) / a.patm, c.rovcp);
  const real q1 = -fq * ras * (a.qvatm - qsg);
  if (q1 < real(0.)) {
    if (a.myj) {
      eeta = -a.qkms * ras * (a.qvatm / (real(1.) + a.qvatm) - qsg / (real(1.) + qsg)) * real(1.e3);
    } else {
      dew = a.qkms * (a.qvatm - qsg);
      eeta = -a.rho * dew;
    }
    qfx = xlvm * eeta;
    eeta = -a.rho * dew;
    sublim = eeta;
  } else {
    if (a.myj) eeta = -a.qkms * ras * beta * (a.qvatm / (real(1.) + a.qvatm) - qvg / (real(1.) + qvg)) * real(1.e3);
    else eeta = q1 * beta * real(1.e3);
    qfx = xlvm * eeta;
    eeta = q1 * beta * real(1.e3);
    sublim = eeta;
  }
  if (snhei >= snth) {
    s = thdifsn * rhocsn * (soilt - L.tsob) / L.snprim;
    snflx = s;
  } else if (snhei < snth && snhei > real(0.)) {
    s = (L.fsn * thdifsn * rhocsn + L.fso * thdifice[1] * rhcs) * (soilt - L.tsob) / L.snprim;
    snflx = s;
  } else {
    snflx = d9sn * (soilt - L.tsob);
  }
  snhei = snwe * real(1.e3) / rhosn;
  x = (r21 + d9sn * r22sn) * (soilt - tnold) + xlvm * r210 * (qsg - qgold);
  x = x - rhonewcsn * newsnow / delt * (c_min(real(273.15), a.tabs) - soilt) -
      rainf * cvw * prcpms * (c_max(real(273.15), a.tabs) - soilt);
  const real icemelt = rnet - hft - xlvm * eeta - s - snoh - x;
  fltot = rnet - hft - xlvm * eeta - s - snoh - x - icemelt;
  if (snhei == real(0.)) {
    tsnav = soilt - real(273.15);
    emiss = real(0.98);
    znt = real(0.011);
    alb = real(0.55);
  }
}

// SOILVEGIN (mosaic_lu = mosaic_soil = 0)
CPPWRF_HD inline void soilvegin(const RucTables& tb, real shdmin, real shdmax, int iswater, int ivgtyp, int isltyp,
                                real vegfrac, bool rdlai2d, int& iforest, real& emiss, real& pc, real& znt, real& lai,
                                SoilParm& sp) {
  iforest = tb.ifortbl[ivgtyp - 1];
  real factor;
  if ((shdmax - shdmin) < real(1)) factor = real(1.);
  else factor = real(1.) - c_max(real(0.), c_min(real(1.), (vegfrac - shdmin) / c_max(real(1.), (shdmax - shdmin))));
  const int k = ivgtyp;
  real deltalai = 0;
  const int ifk = tb.ifortbl[k - 1];
  const real laik = tb.laitbl[k - 1];
  if (ifk == 1) deltalai = c_min(real(0.2), real(0.8) * laik);
  if (ifk == 2 || ifk == 7) deltalai = c_min(real(0.5), real(0.8) * laik);
  if (ifk == 3) deltalai = c_min(real(0.45), real(0.8) * laik);
  if (ifk == 4) deltalai = c_min(real(0.75), real(0.8) * laik);
  if (ifk == 5) deltalai = c_min(real(0.86), real(0.8) * laik);
  real laitoday, znttoday;
  if (k != iswater) {
    laitoday = laik - deltalai * factor;
    if (ifk == 7) znttoday = tb.z0tbl[k - 1] - real(0.125) * factor;
    else znttoday = tb.z0tbl[k - 1];
  } else {
    laitoday = laik;
    znttoday = znt;
  }
  emiss = tb.lemitbl[ivgtyp - 1];
  znt = znttoday;
  pc = tb.pctbl[ivgtyp - 1];
  if (!rdlai2d) lai = laitoday;
  sp = SoilParm{};
  if (isltyp != 14) {
    const int s = isltyp - 1;
    sp.rhocs = tb.hc[s] * real(1.e6);
    sp.bclh = tb.bb[s];
    sp.dqm = tb.maxsmc[s] - tb.drysmc[s];
    sp.ksat = tb.satdk[s];
    sp.psis = -tb.satpsi[s];
    sp.qmin = tb.drysmc[s];
    sp.ref = tb.refsmc[s];
    sp.wilt = tb.wltsmc[s];
    sp.qwrtz = tb.qtz[s];
  }
}

// SFCTMP: the point's snow, vegetation, energy and water budget
struct SfctmpIO {
  // in
  real prcpms, newsnms, snowrat, grauprat, icerat, curat, gsw, vegfra, alb_snow_free, lai, seaice, pc, sat, cn,
      meltfactor;
  int ivgtyp, isice;
  // inout
  int iland;
  real snwe, snhei, snowfrac, rhosn, rhonewsn, rhosnfall, emiss, mavail, cst, alb, znt, alb_snow, soilt, soilt1,
      tsnav, dew, qvg, qsg, qcg, smelt, snoh, snflx, snom, snowfallac, edir1, ec1, ett1, eeta, qfx, hfx, s, sublim,
      evapl, prcpl, fltot, runoff1, runoff2, infiltr, smf, rsm, snweprint, snheiprint;
  real soilm1d[kA], ts1d[kA], smfrkeep[kA], keepfr[kA], soilice[kA], soiliqw[kA];
  int* ilnb_carry = nullptr;  // see the header (ILNB)
};

CPPWRF_HD inline void sfctmp(const Geo& g, const Atm& a, const Consts& c, const RucTables& tb, const SoilParm& sp,
                             SfctmpIO& p) {
  const int nzs = g.nzs;
  const real delt = g.delt;
  const real snhei_crit = real(0.01601) * kRhowater / p.rhosn;
  const real snhei_crit_newsn = real(0.0005) * kRhowater / p.rhosn;
  real snow_mosaic = 0;
  real snfr = real(1.);
  real newsn = 0;
  real snowfracnewsn = 0;
  p.rhonewsn = real(100.);
  if (p.snhei == real(0.)) p.snowfrac = 0;
  p.smelt = 0;
  real rainf = 0;
  p.rsm = 0;
  p.infiltr = 0;
  const real vegfrac = real(0.01) * p.vegfra;
  real drip = 0, dripsn = 0, dripliq = 0;
  p.smf = 0;
  real interw = 0, intersn = 0, infwater = 0;
  real tice[kA] = {}, rhosice[kA] = {}, capice[kA] = {}, thdifice[kA] = {};
  real gswnew = p.gsw;
  const real gswin = p.gsw / (real(1.) - p.alb);
  real albice = p.alb_snow_free;
  real albsn = p.alb_snow;
  const real emissn = real(0.98);
  const real emiss_snowfree = tb.lemitbl[p.ivgtyp - 1];
  if (p.seaice >= real(0.5)) {
    for (int k = 1; k <= nzs; ++k) {
      tice[k] = p.ts1d[k] - real(273.15);
      rhosice[k] = real(917.6) / (real(1) - real(0.000165) * tice[k]);
      const real cice = real(2115.85) + real(7.7948) * tice[k];
      capice[k] = cice * rhosice[k];
      thdifice[k] = real(2.260872) / capice[k];
    }
    albice = c_min(p.alb_snow_free,
                   c_max(p.alb_snow_free - real(0.05), p.alb_snow_free - real(0.1) * (tice[1] + real(10.)) / real(10.)));
  }
  if (p.snhei > real(0.0081) * real(1.e3) / p.rhosn) {
    const real bsn = delt / real(3600.) * c.c1sn *
                     libm::exp(real(0.08) * c_min(real(0.), p.tsnav) - c.c2sn * p.rhosn * real(1.e-3));
    if (!(bsn * p.snwe * real(100.) < real(1.e-4))) {
      const real xsn = p.rhosn * (libm::exp(bsn * p.snwe * real(100.)) - real(1.)) / (bsn * p.snwe * real(100.));
      p.rhosn = c_min(c_max(real(58.8), xsn), real(500.));
    }
  }
  if (p.snowfrac < real(0.75)) snow_mosaic = real(1.);
  newsn = p.newsnms * delt;
  if (newsn > real(0.)) {
    p.rhonewsn = c_min(real(125.), real(1000.0) / c_max(real(8.), (real(17.) * libm::tanh((real(276.65) - a.tabs) * real(0.15)))));
    const real rhonewgr =
        c_min(real(500.), kRhowater / c_max(real(2.), (real(3.5) * libm::tanh((real(274.15) - a.tabs) * real(0.3333)))));
    const real rhonewice = p.rhonewsn;
    p.rhosnfall = c_min(real(500.), c_max(real(58.8), (p.rhonewsn * p.snowrat + rhonewgr * p.grauprat +
                                                        rhonewice * p.icerat + rhonewgr * p.curat)));
    p.rhonewsn = p.rhosnfall;
    const real xsn = (p.rhosn * p.snwe + p.rhonewsn * newsn) / (p.snwe + newsn);
    p.rhosn = c_min(c_max(real(58.8), xsn), real(500.));
  }
  if (p.prcpms != real(0.)) rainf = real(1.);
  drip = 0;
  real intwratio = 0;
  if (vegfrac > real(0.01)) {
    interw = real(0.25) * delt * p.prcpms * (real(1.) - libm::exp(real(-0.5) * p.lai)) * vegfrac;
    intersn = real(0.25) * newsn * (real(1.) - libm::exp(real(-0.5) * p.lai)) * vegfrac;
    infwater = p.prcpms - interw / delt;
    if ((interw + intersn) > real(0.)) intwratio = interw / (interw + intersn);
    const real dd1 = p.cst + interw + intersn;
    p.cst = dd1;
    if (p.cst > p.sat) {
      p.cst = p.sat;
      drip = dd1 - p.sat;
    }
  } else {
    p.cst = 0;
    drip = 0;
    interw = 0;
    intersn = 0;
    infwater = p.prcpms;
  }
  if (newsn > real(0.)) {
    p.snwe = c_max(real(0.), p.snwe + newsn - intersn);
    if (drip > real(0.)) {
      if (snow_mosaic == real(1.)) {
        dripliq = drip * intwratio;
        dripsn = drip - dripliq;
        p.snwe = p.snwe + dripsn;
        infwater = infwater + dripliq;
        dripliq = 0;
        dripsn = 0;
      } else {
        p.snwe = p.snwe + drip;
      }
    }
    p.snhei = p.snwe * kRhowater / p.rhosn;
    newsn = newsn * kRhowater / p.rhonewsn;
  }
  const Atm& at = a;
  if (p.snhei > real(0.0)) {
    p.iland = p.isice;
    p.snowfrac = c_min(real(1.), p.snhei / (real(2.) * snhei_crit));
    const real snowfrac2 = libm::tanh(p.snhei / (real(2.5) * c_min(real(0.2), p.znt) * (p.rhosn / p.rhonewsn)));
    p.snowfrac = real(0.5) * (p.snowfrac + snowfrac2);
    if (newsn > real(0.)) snowfracnewsn = c_min(real(1.), p.snowfallac * real(1.e-3) / snhei_crit_newsn);
    if (p.ivgtyp == tb.urban) p.snowfrac = c_min(real(0.75), p.snowfrac);
    if (p.snowfrac < real(0.75)) snow_mosaic = real(1.);
    real keep_snow_albedo = 0;
    if (snowfracnewsn > real(0.99) && p.rhosnfall < real(450.)) {
      keep_snow_albedo = real(1.);
      snow_mosaic = 0;
    }
    if (newsn == real(0.) && p.znt <= real(0.2) && p.ivgtyp != p.isice) {
      const real z0i = tb.z0tbl[p.iland - 1];
      if (p.snhei <= real(2.) * p.znt) {
        p.znt = real(0.55) * p.znt + real(0.45) * z0i;
      } else if (p.snhei > real(2.) * p.znt && p.snhei <= real(4.) * p.znt) {
        p.znt = real(0.2) * p.znt + real(0.8) * z0i;
      } else if (p.snhei > real(4.) * p.znt) {
        p.znt = z0i;
      }
    }
    if (p.seaice < real(0.5)) {
      if (snow_mosaic == real(1.)) {
        albsn = p.alb_snow;
        if (keep_snow_albedo > real(0.9) && albsn < real(0.4)) albsn = real(0.7);
        p.emiss = emissn;
      } else {
        albsn = c_max(keep_snow_albedo * p.alb_snow,
                      c_min((p.alb_snow_free + (p.alb_snow - p.alb_snow_free) * p.snowfrac), p.alb_snow));
        if (newsn > real(0.) && keep_snow_albedo > real(0.9) && albsn < real(0.4)) albsn = real(0.7);
        p.emiss = c_max(keep_snow_albedo * emissn,
                        c_min((emiss_snowfree + (emissn - emiss_snowfree) * p.snowfrac), emissn));
      }
      if (albsn < real(0.4) || keep_snow_albedo == real(1)) {
        p.alb = albsn;
      } else {
        p.alb = c_min(albsn, c_max(albsn - real(0.1) * (p.soilt - real(263.15)) / (real(273.15) - real(263.15)) * albsn,
                                   albsn - real(0.05)));
      }
    } else {
      if (snow_mosaic == real(1.)) {
        albsn = p.alb_snow;
        p.emiss = emissn;
      } else {
        albsn = c_max(keep_snow_albedo * p.alb_snow, c_min((albice + (p.alb_snow - albice) * p.snowfrac), p.alb_snow));
        p.emiss = c_max(keep_snow_albedo * emissn,
                        c_min((emiss_snowfree + (emissn - emiss_snowfree) * p.snowfrac), emissn));
      }
      if (albsn < p.alb_snow || keep_snow_albedo == real(1.)) {
        p.alb = albsn;
      } else {
        p.alb = c_min(albsn, c_max(albsn - real(0.15) * albsn * (p.soilt - real(263.15)) / (real(273.15) - real(263.15)),
                                   albsn - real(0.1)));
      }
    }
    // the snow-free part of a partly covered point
    real ts1ds[kA] = {}, soilm1ds[kA] = {}, smfrkeeps[kA] = {}, soiliqws[kA] = {}, soilices[kA] = {}, keepfrs[kA] = {};
    real soilts = 0, qvgs = 0, qsgs = 0, qcgs = 0, csts = 0, mavails = 0;
    Flux fs{};
    if (snow_mosaic == real(1.)) {
      if (p.seaice < real(0.5)) {
        gswnew = gswin * (real(1.) - p.alb_snow_free);
        const real t3 = c.stbolt * p.soilt * p.soilt * p.soilt;
        const real upflux = t3 * p.soilt;
        const real xinet = emiss_snowfree * (at.glw - upflux);
        const real rnet = gswnew + xinet;
        for (int k = 1; k <= nzs; ++k) {
          soilm1ds[k] = p.soilm1d[k];
          ts1ds[k] = p.ts1d[k];
          smfrkeeps[k] = p.smfrkeep[k];
          keepfrs[k] = p.keepfr[k];
          soilices[k] = p.soilice[k];
          soiliqws[k] = p.soiliqw[k];
        }
        soilts = p.soilt;
        qvgs = p.qvg;
        qsgs = p.qsg;
        qcgs = p.qcg;
        csts = p.cst;
        mavails = p.mavail;
        p.smelt = 0;
        fs.runoff1 = 0;
        fs.runoff2 = 0;
        const int ilands = p.ivgtyp;
        fs.smf = p.smf;
        soil(g, at, c, tb, sp, ilands, p.prcpms, rainf, gswin, emiss_snowfree, rnet, p.pc, csts, dripliq, infwater,
             vegfrac, p.lai, p.sat, p.cn, soilm1ds, ts1ds, smfrkeeps, keepfrs, soilts, qvgs, qsgs, qcgs, fs, mavails,
             soilices, soiliqws);
        p.smf = fs.smf;
      } else {
        gswnew = gswin * (real(1.) - albice);
        const real t3 = c.stbolt * p.soilt * p.soilt * p.soilt;
        const real upflux = t3 * p.soilt;
        const real xinet = emiss_snowfree * (at.glw - upflux);
        const real rnet = gswnew + xinet;
        for (int k = 1; k <= nzs; ++k) ts1ds[k] = p.ts1d[k];
        soilts = p.soilt;
        qvgs = p.qvg;
        qsgs = p.qsg;
        qcgs = p.qcg;
        p.smelt = 0;
        fs.runoff1 = 0;
        fs.runoff2 = 0;
        sice_balance(g, at, c, tb, p.prcpms, rainf, real(0.98), rnet, capice, thdifice, ts1ds, fs.dew, soilts, qvgs,
                     qsgs, qcgs, fs.eeta, fs.qfx, fs.hfx, fs.s, fs.evapl, fs.prcpl, fs.fltot);
        p.edir1 = p.eeta * real(1.e-3);
        p.ec1 = 0;
        p.ett1 = 0;
        p.runoff1 = p.prcpms;
        p.runoff2 = 0;
        p.mavail = real(1.);
        p.infiltr = 0;
        p.cst = 0;
        for (int k = 1; k <= nzs; ++k) {
          p.soilm1d[k] = real(1.);
          p.soiliqw[k] = 0;
          p.soilice[k] = real(1.);
          p.smfrkeep[k] = real(1.);
          p.keepfr[k] = 0;
        }
      }
    }
    gswnew = gswin * (real(1.) - p.alb);
    {
      const real t3 = c.stbolt * p.soilt * p.soilt * p.soilt;
      const real upflux = t3 * p.soilt;
      const real xinet = p.emiss * (at.glw - upflux);
      const real rnet = gswnew + xinet;
      int ilnb_local = 0;  // uninitialised in WRF (see the header)
      int& ilnb = p.ilnb_carry ? *p.ilnb_carry : ilnb_local;
      if (p.seaice < real(0.5)) {
        snfr = snow_mosaic == real(1.) ? real(1.) : p.snowfrac;
        Flux f{};
        f.dew = p.dew; f.edir1 = p.edir1; f.ec1 = p.ec1; f.ett1 = p.ett1; f.eeta = p.eeta; f.qfx = p.qfx;
        f.hfx = p.hfx; f.s = p.s; f.evapl = p.evapl; f.prcpl = p.prcpl; f.fltot = p.fltot; f.runoff1 = p.runoff1;
        f.runoff2 = p.runoff2; f.infiltr = p.infiltr; f.smf = p.smf; f.sublim = p.sublim;
        snowsoil(g, at, c, tb, sp, p.meltfactor, p.rhonewsn, p.iland, p.prcpms, rainf, newsn, p.snhei, p.snwe, snfr,
                 p.rhosn, gswnew, gswin, p.emiss, rnet, p.pc, p.cst, infwater, vegfrac, p.lai, p.sat, p.cn, ilnb,
                 p.snweprint, p.snheiprint, p.rsm, p.soilm1d, p.ts1d, p.smfrkeep, p.keepfr, p.soilt, p.soilt1, p.tsnav,
                 p.qvg, p.qsg, p.qcg, p.smelt, p.snoh, p.snflx, p.snom, f, p.mavail, p.soilice, p.soiliqw);
        p.dew = f.dew; p.edir1 = f.edir1; p.ec1 = f.ec1; p.ett1 = f.ett1; p.eeta = f.eeta; p.qfx = f.qfx;
        p.hfx = f.hfx; p.s = f.s; p.sublim = f.sublim; p.prcpl = f.prcpl; p.fltot = f.fltot; p.runoff1 = f.runoff1;
        p.runoff2 = f.runoff2; p.infiltr = f.infiltr;
      } else {
        snfr = snow_mosaic == real(1.) ? real(1.) : p.snowfrac;
        snowseaice(g, at, c, tb, p.meltfactor, p.rhonewsn, p.prcpms, rainf, newsn, p.snhei, p.snwe, snfr, p.rhosn,
                   p.emiss, rnet, p.alb, p.znt, capice, thdifice, ilnb, p.snweprint, p.snheiprint, p.rsm, p.ts1d, p.dew,
                   p.soilt, p.soilt1, p.tsnav, p.qvg, p.qsg, p.qcg, p.smelt, p.snoh, p.snflx, p.snom, p.eeta, p.qfx,
                   p.hfx, p.s, p.sublim, p.prcpl, p.fltot);
        p.edir1 = p.eeta * real(1.e-3);
        p.ec1 = 0;
        p.ett1 = 0;
        p.runoff1 = p.smelt;
        p.runoff2 = 0;
        p.mavail = real(1.);
        p.infiltr = 0;
        p.cst = 0;
        for (int k = 1; k <= nzs; ++k) {
          p.soilm1d[k] = real(1.);
          p.soiliqw[k] = 0;
          p.soilice[k] = real(1.);
          p.smfrkeep[k] = real(1.);
          p.keepfr[k] = 0;
        }
      }
    }
    if (snow_mosaic == real(1.)) {
      const real sf = p.snowfrac, sf1 = real(1.) - p.snowfrac;
      if (p.seaice < real(0.5)) {
        for (int k = 1; k <= nzs; ++k) {
          p.soilm1d[k] = soilm1ds[k] * sf1 + p.soilm1d[k] * sf;
          p.ts1d[k] = ts1ds[k] * sf1 + p.ts1d[k] * sf;
          p.smfrkeep[k] = smfrkeeps[k] * sf1 + p.smfrkeep[k] * sf;
          if (sf > real(0.5)) p.keepfr[k] = p.keepfr[k];
          else p.keepfr[k] = keepfrs[k];
          p.soilice[k] = soilices[k] * sf1 + p.soilice[k] * sf;
          p.soiliqw[k] = soiliqws[k] * sf1 + p.soiliqw[k] * sf;
        }
        p.dew = fs.dew * sf1 + p.dew * sf;
        p.soilt = soilts * sf1 + p.soilt * sf;
        p.qvg = qvgs * sf1 + p.qvg * sf;
        p.qsg = qsgs * sf1 + p.qsg * sf;
        p.qcg = qcgs * sf1 + p.qcg * sf;
        p.edir1 = fs.edir1 * sf1 + p.edir1 * sf;
        p.ec1 = fs.ec1 * sf1 + p.ec1 * sf;
        p.cst = csts * sf1 + p.cst * sf;
        p.ett1 = fs.ett1 * sf1 + p.ett1 * sf;
        p.eeta = fs.eeta * sf1 + p.eeta * sf;
        p.qfx = fs.qfx * sf1 + p.qfx * sf;
        p.hfx = fs.hfx * sf1 + p.hfx * sf;
        p.s = fs.s * sf1 + p.s * sf;
        p.evapl = fs.evapl * sf1;
        p.sublim = p.sublim * sf;
        p.prcpl = fs.prcpl * sf1 + p.prcpl * sf;
        p.fltot = fs.fltot * sf1 + p.fltot * sf;
        p.alb = c_max(keep_snow_albedo * p.alb, c_min((p.alb_snow_free + (p.alb - p.alb_snow_free) * sf), p.alb));
        p.emiss = c_max(keep_snow_albedo * emissn, c_min((emiss_snowfree + (emissn - emiss_snowfree) * sf), emissn));
        p.runoff1 = fs.runoff1 * sf1 + p.runoff1 * sf;
        p.runoff2 = fs.runoff2 * sf1 + p.runoff2 * sf;
        p.mavail = mavails * sf1 + real(1.) * sf;
        p.infiltr = fs.infiltr * sf1 + p.infiltr * sf;
      } else {
        for (int k = 1; k <= nzs; ++k) p.ts1d[k] = ts1ds[k] * sf1 + p.ts1d[k] * sf;
        p.dew = fs.dew * sf1 + p.dew * sf;
        p.soilt = soilts * sf1 + p.soilt * sf;
        p.qvg = qvgs * sf1 + p.qvg * sf;
        p.qsg = qsgs * sf1 + p.qsg * sf;
        p.qcg = qcgs * sf1 + p.qcg * sf;
        p.eeta = fs.eeta * sf1 + p.eeta * sf;
        p.qfx = fs.qfx * sf1 + p.qfx * sf;
        p.hfx = fs.hfx * sf1 + p.hfx * sf;
        p.s = fs.s * sf1 + p.s * sf;
        p.sublim = p.eeta;
        p.prcpl = fs.prcpl * sf1 + p.prcpl * sf;
        p.fltot = fs.fltot * sf1 + p.fltot * sf;
        p.alb = c_max(keep_snow_albedo * p.alb, c_min((albice + (p.alb - p.alb_snow_free) * sf), p.alb));
        p.emiss = c_max(keep_snow_albedo * emissn, c_min((emiss_snowfree + (emissn - emiss_snowfree) * sf), emissn));
        p.runoff1 = fs.runoff1 * sf1 + p.runoff1 * sf;
        p.runoff2 = fs.runoff2 * sf1 + p.runoff2 * sf;
      }
    }
    if (p.snhei == real(0.)) {
      p.alb = p.alb_snow_free;
      p.iland = p.ivgtyp;
    } else {
      p.snowfrac = c_min(real(1.), p.snhei / (real(2.) * snhei_crit));
      const real sf2 = libm::tanh(p.snhei / (real(2.5) * c_min(real(0.2), p.znt) * (p.rhosn / p.rhonewsn)));
      p.snowfrac = real(0.5) * (p.snowfrac + sf2);
    }
    if (p.ivgtyp == tb.urban) p.snowfrac = c_min(real(0.75), p.snowfrac);
    p.snowfallac = p.snowfallac + newsn * real(1.e3);
  } else {
    p.snheiprint = 0;
    p.snweprint = 0;
    p.smelt = 0;
    const real t3 = c.stbolt * p.soilt * p.soilt * p.soilt;
    const real upflux = t3 * p.soilt;
    const real xinet = p.emiss * (at.glw - upflux);
    real rnet = gswnew + xinet;
    if (p.seaice < real(0.5)) {
      Flux f{};
      f.dew = p.dew; f.edir1 = p.edir1; f.ec1 = p.ec1; f.ett1 = p.ett1; f.eeta = p.eeta; f.qfx = p.qfx;
      f.hfx = p.hfx; f.s = p.s; f.evapl = p.evapl; f.prcpl = p.prcpl; f.fltot = p.fltot; f.runoff1 = p.runoff1;
      f.runoff2 = p.runoff2; f.infiltr = p.infiltr; f.smf = p.smf; f.sublim = p.sublim;
      soil(g, at, c, tb, sp, p.iland, p.prcpms, rainf, gswin, p.emiss, rnet, p.pc, p.cst, drip, infwater, vegfrac, p.lai,
           p.sat, p.cn, p.soilm1d, p.ts1d, p.smfrkeep, p.keepfr, p.soilt, p.qvg, p.qsg, p.qcg, f, p.mavail, p.soilice,
           p.soiliqw);
      p.dew = f.dew; p.edir1 = f.edir1; p.ec1 = f.ec1; p.ett1 = f.ett1; p.eeta = f.eeta; p.qfx = f.qfx;
      p.hfx = f.hfx; p.s = f.s; p.evapl = f.evapl; p.prcpl = f.prcpl; p.fltot = f.fltot; p.runoff1 = f.runoff1;
      p.runoff2 = f.runoff2; p.infiltr = f.infiltr; p.smf = f.smf;
    } else {
      if (p.alb != albice) gswnew = p.gsw / (real(1.) - p.alb) * (real(1.) - albice);
      p.alb = albice;
      rnet = gswnew + xinet;
      sice_balance(g, at, c, tb, p.prcpms, rainf, p.emiss, rnet, capice, thdifice, p.ts1d, p.dew, p.soilt, p.qvg,
                   p.qsg, p.qcg, p.eeta, p.qfx, p.hfx, p.s, p.evapl, p.prcpl, p.fltot);
      p.edir1 = p.eeta * real(1.e-3);
      p.ec1 = 0;
      p.ett1 = 0;
      p.runoff1 = p.prcpms;
      p.runoff2 = 0;
      p.mavail = real(1.);
      p.infiltr = 0;
      p.cst = 0;
      for (int k = 1; k <= nzs; ++k) {
        p.soilm1d[k] = real(1.);
        p.soiliqw[k] = 0;
        p.soilice[k] = real(1.);
        p.smfrkeep[k] = real(1.);
        p.keepfr[k] = 0;
      }
    }
  }
}

// LSMRUC for one point (i, j): configuration, forcing and state
struct LsmConfig {
  real dt;
  int ktau, nzs, iswater, isice;
  real xice_threshold;
  bool myj, frpcpn, rdlai2d;
  real zs[kA];  // zs(1:nzs)
  Consts c;
  int* ilnb_carry = nullptr;  // column test only: SFCTMP's ILNB kept across calls as gfortran's stack does (header)
};
struct LsmForcing {
  real rainbl, rainncv, snowncv, graupelncv, frzfrac, glw, gsw, albbck, flhc, flqc, chs, xice, xland, tbot, shdmin,
      shdmax;
  real t3d, qv3d, qc3d, p8w, rho3d, z3d;  // level kms: t_phy, qv, qc, p_phy (Pa), rho, dz8w
  int ivgtyp, isltyp;
};
struct LsmState {
  real vegfra, snow, snowh, snowc, canwat, snoalb, alb, emiss, lai, mavail, sfcexc, z0, znt, soilt, hfx, qfx, lh,
      sfcevp, sfcrunoff, udrunoff, acrunoff, grdflx, acsnow, snom, qvg, qcg, dew, qsfc, qsg, chklowq, soilt1, tsnav,
      smavail, smmax, rhosnf, precipfr, snowfallac;
  real soilmois[kA], sh2o[kA], tso[kA], smfr3d[kA], keepfr3dflag[kA];
};

CPPWRF_HD inline void lsmruc_point(const LsmConfig& cf, const RucTables& tb, const LsmForcing& in, LsmState& st) {
  const int nzs = cf.nzs;
  const real dt = cf.dt;
  const real* tbq = tb.tbq;
  real snom_l = st.snom;
  if (cf.ktau == 1) {  // the first step's initialisation of the point
    for (int k = 1; k <= nzs; ++k) st.keepfr3dflag[k] = 0;
    if ((st.soilt1 < real(170.)) || (st.soilt1 > real(400.))) {
      if (st.snowc > real(0.)) st.soilt1 = real(0.5) * (st.soilt + st.tso[1]);
      else st.soilt1 = st.tso[1];
    }
    st.tsnav = real(0.5) * (st.soilt + st.tso[1]) - real(273.15);
    const real patmb = in.p8w * real(1.e-2);
    st.qsg = qsn(st.soilt, tbq) / patmb;
    if ((st.qcg < real(0.)) || (st.qcg > real(0.1))) st.qcg = in.qc3d;
    if ((st.qvg <= real(0.)) || (st.qvg > real(0.1))) st.qvg = st.qsg * st.mavail;
    st.qsfc = st.qvg / (real(1.) + st.qvg);
    snom_l = 0;
    st.snowfallac = 0;
    st.precipfr = 0;
    st.rhosnf = real(-1.e3);
    st.dew = 0;
    st.sfcrunoff = 0;
    st.udrunoff = 0;
    st.acrunoff = 0;
    st.chklowq = real(1.);
  }
  st.snom = snom_l;
  const int ivgtyp = in.ivgtyp;
  int iland = ivgtyp;
  int isoil = in.isltyp;
  const real tabs = in.t3d;
  const real qvatm = in.qv3d;
  const real qcatm = in.qc3d;
  const real patm = in.p8w * real(1.e-5);
  const real conflx = in.z3d * real(0.5);
  const real rho = in.rho3d;
  real snowrat = 0, grauprat = 0, icerat = 0, curat = 0;
  real prcpms, newsnms;
  if (cf.frpcpn) {
    const real prcpncliq = in.rainncv * (real(1.) - in.frzfrac);
    const real prcpncfr = in.rainncv * in.frzfrac;
    real prcpculiq, prcpcufr;
    if (in.frzfrac > real(0.) && tabs < real(273.)) {
      prcpculiq = c_max(real(0.), (in.rainbl - in.rainncv) * (real(1.) - in.frzfrac));
      prcpcufr = c_max(real(0.), (in.rainbl - in.rainncv) * in.frzfrac);
    } else {
      if (tabs < real(273.)) {
        prcpcufr = c_max(real(0.), (in.rainbl - in.rainncv));
        prcpculiq = 0;
      } else {
        prcpcufr = 0;
        prcpculiq = c_max(real(0.), (in.rainbl - in.rainncv));
      }
    }
    prcpms = (prcpncliq + prcpculiq) / dt * real(1.e-3);
    newsnms = (prcpncfr + prcpcufr) / dt * real(1.e-3);
    const real graupamt = in.graupelncv;
    if ((prcpncfr + prcpcufr) > real(0.)) {
      snowrat = c_min(real(1.), c_max(real(0.), in.snowncv / (prcpncfr + prcpcufr)));
      grauprat = c_min(real(1.), c_max(real(0.), graupamt / (prcpncfr + prcpcufr)));
      icerat = c_min(real(1.), c_max(real(0.), (prcpncfr - in.snowncv - graupamt) / (prcpncfr + prcpcufr)));
      curat = c_min(real(1.), c_max(real(0.), (prcpcufr / (prcpncfr + prcpcufr))));
    }
  } else {
    if (tabs <= real(273.15)) {
      prcpms = 0;
      newsnms = in.rainbl / dt * real(1.e-3);
      snowrat = real(1.);
    } else {
      prcpms = in.rainbl / dt * real(1.e-3);
      newsnms = 0;
    }
  }
  st.precipfr = newsnms * dt * real(1.e3);
  real qkms, tkms;
  if (cf.myj) {
    qkms = in.chs;
    tkms = in.chs;
  } else {
    qkms = in.flqc / rho / st.mavail;
    tkms = in.flhc / rho / (cf.c.cp * (real(1.) + real(0.84) * qvatm));
  }
  SfctmpIO p;
  p.snwe = st.snow * real(1.e-3);
  p.snhei = st.snowh;
  real canwatr = st.canwat * real(1.e-3);
  p.snowfrac = st.snowc;
  p.rhosnfall = st.rhosnf;
  Geo g;
  g.nzs = nzs;
  g.delt = dt;
  g.conflx = conflx;
  g.zsmain[1] = 0;
  g.zshalf[1] = 0;
  for (int k = 2; k <= nzs; ++k) {
    g.zsmain[k] = cf.zs[k];
    g.zshalf[k] = real(0.5) * (g.zsmain[k - 1] + g.zsmain[k]);
  }
  const int nzs1 = nzs - 1;
  for (int k = 2; k <= nzs1; ++k) {
    const int k1 = 2 * k - 3;
    const int k2 = k1 + 1;
    const real x = dt / real(2.) / (g.zshalf[k + 1] - g.zshalf[k]);
    g.dtdzs[k1] = x / (g.zsmain[k] - g.zsmain[k - 1]);
    g.dtdzs2[k - 1] = x;
    g.dtdzs[k2] = x / (g.zsmain[k + 1] - g.zsmain[k]);
  }
  int nroot = 4;
  if (st.snow > real(0.) && st.snowh > real(0.)) p.rhosn = st.snow / st.snowh;
  else p.rhosn = real(300.);
  int iforest;
  real emissl, pc;
  SoilParm sp;
  soilvegin(tb, in.shdmin, in.shdmax, cf.iswater, iland, isoil, st.vegfra, cf.rdlai2d, iforest, emissl, pc, st.znt,
            st.lai, sp);
  const real cn = tb.cfactr_data;
  const real sat = real(5.e-4);
  real meltfactor;
  if (iforest > 2) {
    meltfactor = real(2.0);
    for (int k = 2; k <= nzs; ++k)
      if (g.zsmain[k] >= real(0.4)) {
        nroot = k;
        break;
      }
  } else {
    meltfactor = real(0.85);
    for (int k = 2; k <= nzs; ++k)
      if (g.zsmain[k] >= real(1.1)) {
        nroot = k;
        break;
      }
  }
  g.nroot = nroot;
  if ((in.xland - real(1.5)) >= real(0.)) {  // water
    st.smavail = real(1.0);
    st.smmax = real(1.0);
    st.snow = real(0.0);
    st.snowh = real(0.0);
    st.snowc = real(0.0);
    const real patmb = in.p8w * real(1.e-2);
    st.qvg = qsn(st.soilt, tbq) / patmb;
    st.qsfc = st.qvg / (real(1.) + st.qvg);
    st.chklowq = real(1.);
    for (int k = 1; k <= nzs; ++k) {
      st.soilmois[k] = real(1.0);
      st.sh2o[k] = real(1.0);
      st.tso[k] = st.soilt;
    }
    return;
  }
  const real seaice = in.xice >= cf.xice_threshold ? real(1.) : real(0.);
  if (seaice > real(0.5)) {
    iland = cf.isice;
    isoil = 16;
    st.znt = real(0.011);
    st.snoalb = real(0.75);
    sp.dqm = real(1.);
    sp.ref = real(1.);
    sp.qmin = 0;
    sp.wilt = 0;
    emissl = real(0.98);
    const real patmb = in.p8w * real(1.e-2);
    st.qvg = qsn(st.soilt, tbq) / patmb;
    st.qsg = st.qvg;
    st.qsfc = st.qvg / (real(1.) + st.qvg);
    for (int k = 1; k <= nzs; ++k) {
      st.soilmois[k] = real(1.);
      st.smfr3d[k] = real(1.);
      st.sh2o[k] = 0;
      st.keepfr3dflag[k] = 0;
      st.tso[k] = c_min(real(271.4), st.tso[k]);
    }
  }
  (void)isoil;
  for (int k = 1; k <= nzs; ++k) {
    p.soilm1d[k] = c_min(c_max(real(0.), st.soilmois[k] - sp.qmin), sp.dqm);
    p.ts1d[k] = st.tso[k];
    p.soiliqw[k] = c_min(c_max(real(0.), st.sh2o[k] - sp.qmin), p.soilm1d[k]);
    p.soilice[k] = (p.soilm1d[k] - p.soiliqw[k]) / real(0.9);
  }
  for (int k = 1; k <= nzs; ++k) {
    p.smfrkeep[k] = st.smfr3d[k];
    p.keepfr[k] = st.keepfr3dflag[k];
  }
  p.mavail = c_max(real(0.00001), c_min(real(1.), p.soilm1d[1] / (sp.ref - sp.qmin)));
  Atm a{patm, tabs, qvatm, qcatm, rho, in.glw, qkms, tkms, cf.myj};
  p.prcpms = prcpms;
  p.newsnms = newsnms;
  p.snowrat = snowrat;
  p.grauprat = grauprat;
  p.icerat = icerat;
  p.curat = curat;
  p.gsw = in.gsw;
  p.vegfra = st.vegfra;
  p.alb_snow_free = in.albbck;
  p.lai = st.lai;
  p.seaice = seaice;
  p.pc = pc;
  p.sat = sat;
  p.cn = cn;
  p.meltfactor = meltfactor;
  p.ivgtyp = ivgtyp;
  p.isice = cf.isice;
  p.iland = iland;
  p.rhonewsn = real(200.);
  p.emiss = emissl;
  p.cst = canwatr;
  p.alb = st.alb;
  p.znt = st.znt;
  p.alb_snow = st.snoalb;
  p.soilt = st.soilt;
  p.soilt1 = st.soilt1;
  p.tsnav = st.tsnav;
  p.dew = st.dew;
  p.qvg = st.qvg;
  p.qsg = st.qsg;
  p.qcg = st.qcg;
  p.smelt = 0;
  p.snoh = 0;
  p.snflx = 0;
  p.snom = st.snom;
  p.snowfallac = st.snowfallac;
  p.edir1 = p.ec1 = p.ett1 = 0;
  p.eeta = st.qfx;  // SFCTMP's EETA is LSMRUC's QFX, its QFX is LH
  p.qfx = st.lh;
  p.hfx = st.hfx;
  p.s = 0;
  p.sublim = p.evapl = p.prcpl = p.fltot = p.runoff1 = p.runoff2 = 0;
  p.infiltr = 0;
  p.smf = 0;
  p.rsm = p.snweprint = p.snheiprint = 0;
  p.ilnb_carry = cf.ilnb_carry;
  sfctmp(g, a, cf.c, tb, sp, p);
  st.smavail = 0;
  st.smmax = 0;
  for (int k = 1; k <= nzs - 1; ++k) {
    st.smavail = st.smavail + (sp.qmin + p.soilm1d[k]) * (g.zshalf[k + 1] - g.zshalf[k]);
    st.smmax = st.smmax + (sp.qmin + sp.dqm) * (g.zshalf[k + 1] - g.zshalf[k]);
  }
  st.smavail = st.smavail + (sp.qmin + p.soilm1d[nzs]) * (g.zsmain[nzs] - g.zshalf[nzs]);
  st.smmax = st.smmax + (sp.qmin + sp.dqm) * (g.zsmain[nzs] - g.zshalf[nzs]);
  st.sfcrunoff = st.sfcrunoff + p.runoff1 * dt * real(1000.0);
  st.udrunoff = st.udrunoff + p.runoff2 * dt * real(1000.0);
  st.acrunoff = st.acrunoff + p.runoff1 * dt * real(1000.0);
  st.smavail = st.smavail * real(1000.);
  st.smmax = st.smmax * real(1000.);
  for (int k = 1; k <= nzs; ++k) {
    st.soilmois[k] = p.soilm1d[k] + sp.qmin;
    st.sh2o[k] = c_min(p.soiliqw[k] + sp.qmin, st.soilmois[k]);
    st.tso[k] = p.ts1d[k];
  }
  st.tso[nzs] = in.tbot;
  for (int k = 1; k <= nzs; ++k) {
    st.smfr3d[k] = p.smfrkeep[k];
    st.keepfr3dflag[k] = p.keepfr[k];
  }
  st.znt = p.znt;
  st.z0 = st.znt;
  st.sfcexc = tkms;
  st.alb = p.alb;
  st.snoalb = p.alb_snow;
  st.soilt = p.soilt;
  st.soilt1 = p.soilt1;
  st.tsnav = p.tsnav;
  st.dew = p.dew;
  st.qvg = p.qvg;
  st.qsg = p.qsg;
  st.qcg = p.qcg;
  st.snom = p.snom;
  st.snowfallac = p.snowfallac;
  st.qfx = p.eeta;
  st.lh = p.qfx;
  st.hfx = p.hfx;
  const real patmb = in.p8w * real(1.e-2);
  const real q2sat = qsn(tabs, tbq) / patmb;
  st.qsfc = st.qvg / (real(1.) + st.qvg);
  if ((qvatm >= q2sat * real(0.95)) && qvatm < st.qvg) st.chklowq = 0;
  else st.chklowq = real(1.);
  if (st.snow == real(0.)) p.emiss = tb.lemitbl[ivgtyp - 1];
  st.emiss = p.emiss;
  st.snow = p.snwe * real(1000.);
  st.snowh = p.snhei;
  st.canwat = p.cst * real(1000.);
  st.mavail = p.mavail;
  st.sfcevp = st.sfcevp + st.qfx * dt;
  st.grdflx = real(-1.) * p.s;
  real snowfrac = p.snowfrac;
  if (snowfrac > real(0.) && in.xice >= cf.xice_threshold) snowfrac = snowfrac * in.xice;
  st.snowc = snowfrac;
  st.rhosnf = p.rhosnfall;
  st.sfcevp = st.sfcevp + st.qfx * dt;
}

// RUCLSMINIT for one point (not on restarts): ZNT, MAVAIL, SH2O, SMFR3D
CPPWRF_HD inline void ruclsminit_point(const RucTables& tb, int nzs, int ivgtyp, int isltyp, real xice,
                                       const real* tslb, const real* smois, real* sh2o, real* smfr3d, real& mavail,
                                       real& znt) {
  const real riw = real(900.) * real(1.e-3);
  const real xlmelt = real(3.35e+5);
  znt = tb.z0tbl[ivgtyp - 1];
  const int s = isltyp - 1;
  const real dqm = tb.maxsmc[s] - tb.drysmc[s];
  const real ref = tb.refsmc[s];
  const real psis = -tb.satpsi[s];
  const real qmin = tb.drysmc[s];
  const real bclh = tb.bb[s];
  if (xice > real(0.)) {
    for (int l = 1; l <= nzs; ++l) {
      smfr3d[l] = real(1.);
      sh2o[l] = 0;
      mavail = real(1.);
    }
  } else if (isltyp != 14) {
    mavail = c_max(real(0.00001), c_min(real(1.), (smois[1] - qmin) / (ref - qmin)));
    for (int l = 1; l <= nzs; ++l) {
      const real tln = libm::log(tslb[l] / real(273.15));
      if (tln < real(0.)) {
        real soiliqw = (dqm + qmin) *
                       libm::pow(xlmelt * (tslb[l] - real(273.15)) / tslb[l] / real(9.81) / psis, real(-1.) / bclh);
        soiliqw = c_max(real(0.), soiliqw);
        soiliqw = c_min(soiliqw, smois[l]);
        sh2o[l] = soiliqw;
        smfr3d[l] = (smois[l] - soiliqw) / riw;
      } else {
        smfr3d[l] = 0;
        sh2o[l] = smois[l];
      }
    }
  } else {
    for (int l = 1; l <= nzs; ++l) {
      smfr3d[l] = 0;
      sh2o[l] = real(1.);
      mavail = real(1.);
    }
  }
}

// SFCDIAGS_RUCLSM (flux method) for one point
CPPWRF_HD inline real rslf(real p, real t) {
  constexpr real c0 = real(.611583699E03), c1 = real(.444606896E02), c2 = real(.143177157E01),
                 c3 = real(.264224321E-1), c4 = real(.299291081E-3), c5 = real(.203154182E-5),
                 c6 = real(.702620698E-8), c7 = real(.379534310E-11), c8 = real(-.321582393E-13);
  const real x = c_max(real(-80.), t - real(273.16));
  const real esl = c0 + x * (c1 + x * (c2 + x * (c3 + x * (c4 + x * (c5 + x * (c6 + x * (c7 + x * c8)))))));
  return real(.622) * esl / (p - esl);
}
CPPWRF_HD inline real rsif(real p, real t) {
  constexpr real c0 = real(.609868993E03), c1 = real(.499320233E02), c2 = real(.184672631E01),
                 c3 = real(.402737184E-1), c4 = real(.565392987E-3), c5 = real(.521693933E-5),
                 c6 = real(.307839583E-7), c7 = real(.105785160E-9), c8 = real(.161444444E-12);
  const real x = c_max(real(-80.), t - real(273.16));
  const real esi = c0 + x * (c1 + x * (c2 + x * (c3 + x * (c4 + x * (c5 + x * (c6 + x * (c7 + x * c8)))))));
  return real(.622) * esi / (p - esi);
}
CPPWRF_HD inline void sfcdiags_ruclsm(real hfx, real qfx, real tsk, real qsfc, real cqs, real cqs2, real chs2,
                                      real t3d, real qv3d, real rho3d, real p3d, real psfc, real cp, real rovcp,
                                      real& t2, real& th2, real& q2) {
  const real rho = rho3d;
  if (chs2 < real(1.E-5)) th2 = t3d * libm::pow(real(1.E5) / psfc, rovcp);
  else th2 = tsk * libm::pow(real(1.E5) / psfc, rovcp) - hfx / (rho * cp * chs2);
  t2 = th2 * libm::pow(real(1.E-5) * psfc, rovcp);
  real x2m = c_max(c_min(tsk, t3d), t2);
  t2 = c_min(c_max(tsk, t3d), x2m);
  th2 = t2 * libm::pow(real(1.E5) / psfc, rovcp);
  real qlev1 = qv3d;
  real tempc = t3d - real(273.15);
  real qsat = tempc <= real(0.0) ? rsif(p3d, t3d) : rslf(p3d, t3d);
  qlev1 = c_min(qsat, qlev1);
  const real qsfcprox = qlev1 + qfx / (rho * cqs);
  const real qsfcmr = qsfc / (real(1.) - qsfc);
  if (cqs2 < real(1.E-5)) q2 = qlev1;
  else q2 = qsfcprox - qfx / (rho * cqs2);
  x2m = c_max(c_min(qsfcmr, qlev1), q2);
  q2 = c_min(c_max(qsfcmr, qlev1), x2m);
  tempc = t2 - real(273.15);
  qsat = tempc <= real(0.0) ? rsif(psfc, t2) : rslf(psfc, t2);
  q2 = c_min(qsat, q2);
}

}  // namespace cppwrf::phys::ruc
