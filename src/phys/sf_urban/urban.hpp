// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Single-layer urban canopy model (SLUCM, sf_urban_physics = 1) of WRF v4.8.0 phys/module_sf_urban.F:
// SUBROUTINE urban with SFCDIF_URB and multi_layer, for the URBPARM.TBL options this port supports
// (urban_tables.hpp: TS_SCHEME 1, CH_SCHEME 2, IMP_SCHEME 1, IRI_SCHEME 0, AHOPTION 0, ALHOPTION 0, GROPTION 0,
// TREEOPTION 0, BOUNDR/B/G 1, OASIS 1, no NUDAPT morphology, slucm_distributed_drag = .false.). The other table
// values and the NUDAPT branch (mh_urb > 0, which reads WRF's undefined U10 and SW) are refused at the start.
// Statement order, parenthesisation and constants as WRF's; a REAL ** REAL exponent is pow (gfortran calls pow for
// x**3., x**4., x**0.25, ...; x**2. is x*x), an INTEGER exponent a product (powi).
#include "core/backend.hpp"
#include "core/precision.hpp"

namespace cppwrf::phys::urban {

static constexpr int kMaxUrbCat = 11;   // URBPARM_LCZ.TBL: 11 categories (URBPARM.TBL: 3)
static constexpr int kMaxUrbLayers = 8;  // num_roof_layers = num_wall_layers = num_road_layers = num_soil_layers

// urban_param_init's tables (module variables of module_sf_urban.F), per urban category UTYPE (index UTYPE - 1)
struct UrbanTables {
  int icate = 0;
  int nlayers = 0;  // num_soil_layers
  real zr[kMaxUrbCat], sigma_zed[kMaxUrbCat], z0c[kMaxUrbCat], z0hc[kMaxUrbCat], zdc[kMaxUrbCat], svf[kMaxUrbCat],
      r[kMaxUrbCat], rw[kMaxUrbCat], hgt[kMaxUrbCat], ah[kMaxUrbCat], alh[kMaxUrbCat], betr[kMaxUrbCat],
      betb[kMaxUrbCat], betg[kMaxUrbCat], frc_urb[kMaxUrbCat], capr[kMaxUrbCat], capb[kMaxUrbCat], capg[kMaxUrbCat],
      aksr[kMaxUrbCat], aksb[kMaxUrbCat], aksg[kMaxUrbCat], albr[kMaxUrbCat], albb[kMaxUrbCat], albg[kMaxUrbCat],
      epsr[kMaxUrbCat], epsb[kMaxUrbCat], epsg[kMaxUrbCat], z0r[kMaxUrbCat], z0b[kMaxUrbCat], z0g[kMaxUrbCat],
      z0hb[kMaxUrbCat], z0hg[kMaxUrbCat], trlend[kMaxUrbCat], tblend[kMaxUrbCat], tglend[kMaxUrbCat],
      akanda_urban[kMaxUrbCat];
  real dzr[kMaxUrbLayers], dzb[kMaxUrbLayers], dzg[kMaxUrbLayers];  // [cm] (DDZR * 100: grid%dzr ...)
};

// The state and outputs of one urban point (urban's INOUT / OUT arguments)
struct UrbanState {
  real tr, tb, tg, tc, qc, uc;
  real xxxr, xxxb, xxxg, xxxc;
  real trl[kMaxUrbLayers], tbl[kMaxUrbLayers], tgl[kMaxUrbLayers];
  real flxhumr, flxhumb, flxhumg;
  real cmr, chr, cmc, chc;  // CMR_URB, CHR_URB, CMC_URB, CHC_URB (grid%cmr_sfcdif ...)
  real chs, chs2;           // CH*U at za and 2 m (in)
};
struct UrbanOut {
  real ts, qs, sh, lh, lh_kinematic, sw, alb, lw, g, rn, psim, psih, gz1oz0, u10, v10, th2, q2, ust;
};

// SFCDIF_URB (module_sf_urban.F:4108-4312) without VEGFRAC / ZT_OUT (slucm_distributed_drag = .false.)
CPPWRF_HD inline void sfcdif_urb(real zlm, real z0, real thz0, real thlm, real sfcspd, real akanda, real& akms,
                                 real& akhs, real& rlmo, real& cd) {
  constexpr real wwst = real(1.2), wwst2 = wwst * wwst, g = real(9.8), vkrm = real(0.40), excm = real(0.001);
  constexpr real beta = real(1.) / real(270.), btg = beta * g, elfc = vkrm * btg, wold = real(.15),
                 wnew = real(1.) - wold;
  constexpr int itrmx = 5;
  constexpr real pihf = real(3.14159265) / real(2.);
  constexpr real epsu2 = real(1.E-4), epsust = real(0.07), ztmin = real(-5.), ztmax = real(1.), hpbl = real(1000.0),
                 sqvisc = real(258.2);
  constexpr real sqvisc2 = sqvisc * sqvisc;  // SQVISC**2 (folded)
  auto pspmu = [&](real xx) {
    return -real(2.) * libm::log((xx + real(1.)) * real(0.5)) - libm::log((xx * xx + real(1.)) * real(0.5)) +
           real(2.) * libm::atan(xx) - pihf;
  };
  auto pspms = [](real yy) { return real(5.) * yy; };
  auto psphu = [&](real xx) { return -real(2.) * libm::log((xx * xx + real(1.)) * real(0.5)); };
  auto psphs = [](real yy) { return real(5.) * yy; };

  const real zu = z0;
  const real rdz = real(1.) / zlm;
  const real cxch = excm * rdz;
  const real dthv = thlm - thz0;
  const real du2 = c_max(sfcspd * sfcspd, epsu2);
  const real btgh = btg * hpbl;
  real wstar2;
  if (btgh * akhs * dthv != real(0.0)) {
    wstar2 = wwst2 * rpow(std::abs(btgh * akhs * dthv), real(2.) / real(3.));
  } else {
    wstar2 = real(0.0);
  }
  real ustar = c_max(std::sqrt(akms * std::sqrt(du2 + wstar2)), epsust);
  real zt = libm::exp(real(2.0) - akanda * rpow(sqvisc2 * ustar * z0, real(0.25))) * z0;
  const real zslu = zlm + zu;
  real zslt = zlm + zt;
  const real rlogu = libm::log(zslu / zu);
  real rlogt = libm::log(zslt / zt);
  rlmo = elfc * akhs * dthv / powi(ustar, 3);
  for (int itr = 1; itr <= itrmx; ++itr) {
    real zetalt = c_max(zslt * rlmo, ztmin);
    rlmo = zetalt / zslt;
    real zetalu = zslu * rlmo;
    const real zetau = zu * rlmo;
    const real zetat = zt * rlmo;
    real simm, simh;
    if (rlmo < real(0.0)) {
      const real xlu4 = real(1.) - real(16.) * zetalu;
      const real xlt4 = real(1.) - real(16.) * zetalt;
      const real xu4 = real(1.) - real(16.) * zetau;
      const real xt4 = real(1.) - real(16.) * zetat;
      const real xlu = std::sqrt(std::sqrt(xlu4));
      const real xlt = std::sqrt(std::sqrt(xlt4));
      const real xu = std::sqrt(std::sqrt(xu4));
      const real xt = std::sqrt(std::sqrt(xt4));
      const real psmz = pspmu(xu);
      simm = pspmu(xlu) - psmz + rlogu;
      const real pshz = psphu(xt);
      simh = psphu(xlt) - pshz + rlogt;
    } else {
      zetalu = c_min(zetalu, ztmax);
      zetalt = c_min(zetalt, ztmax);
      const real psmz = pspms(zetau);
      simm = pspms(zetalu) - psmz + rlogu;
      const real pshz = psphs(zetat);
      simh = psphs(zetalt) - pshz + rlogt;
    }
    ustar = c_max(std::sqrt(akms * std::sqrt(du2 + wstar2)), epsust);
    zt = libm::exp(real(2.0) - akanda * rpow(sqvisc2 * ustar * z0, real(0.25))) * z0;
    zslt = zlm + zt;
    rlogt = libm::log(zslt / zt);
    const real ustark = ustar * vkrm;
    akms = c_max(ustark / simm, cxch);
    akhs = c_max(ustark / simh, cxch);
    if (btgh * akhs * dthv != real(0.0)) {
      wstar2 = wwst2 * rpow(std::abs(btgh * akhs * dthv), real(2.) / real(3.));
    } else {
      wstar2 = real(0.0);
    }
    const real rlmn = elfc * akhs * dthv / powi(ustar, 3);
    const real rlma = rlmo * wold + rlmn * wnew;
    rlmo = rlma;
  }
  cd = ustar * ustar / (sfcspd * sfcspd);
}

// multi_layer (module_sf_urban.F:2741-2816): implicit heat conduction in the roof / wall / road layers
CPPWRF_HD inline void multi_layer(int km, int bound, real g0, real cap, real aks, real* tsl, const real* dz, real delt,
                                  real tslend) {
  real a[kMaxUrbLayers], b[kMaxUrbLayers], c[kMaxUrbLayers], d[kMaxUrbLayers], x[kMaxUrbLayers], p[kMaxUrbLayers],
      q[kMaxUrbLayers];
  const real dzend = dz[km - 1];
  a[0] = real(0.0);
  b[0] = cap * dz[0] / delt + real(2.) * aks / (dz[0] + dz[1]);
  c[0] = -real(2.) * aks / (dz[0] + dz[1]);
  d[0] = cap * dz[0] / delt * tsl[0] + g0;
  for (int k = 1; k < km - 1; ++k) {
    a[k] = -real(2.) * aks / (dz[k - 1] + dz[k]);
    b[k] = cap * dz[k] / delt + real(2.) * aks / (dz[k - 1] + dz[k]) + real(2.) * aks / (dz[k] + dz[k + 1]);
    c[k] = -real(2.) * aks / (dz[k] + dz[k + 1]);
    d[k] = cap * dz[k] / delt * tsl[k];
  }
  const int m = km - 1;
  if (bound == 1) {  // Flux = 0
    a[m] = -real(2.) * aks / (dz[m - 1] + dz[m]);
    b[m] = cap * dz[m] / delt + real(2.) * aks / (dz[m - 1] + dz[m]);
    c[m] = real(0.0);
    d[m] = cap * dz[m] / delt * tsl[m];
  } else {  // T = constant
    a[m] = -real(2.) * aks / (dz[m - 1] + dz[m]);
    b[m] = cap * dz[m] / delt + real(2.) * aks / (dz[m - 1] + dz[m]) + real(2.) * aks / (dz[m] + dzend);
    c[m] = real(0.0);
    d[m] = cap * dz[m] / delt * tsl[m] + real(2.) * aks * tslend / (dz[m] + dzend);
  }
  p[0] = -c[0] / b[0];
  q[0] = d[0] / b[0];
  for (int k = 1; k < km; ++k) {
    p[k] = -c[k] / (a[k] * p[k - 1] + b[k]);
    q[k] = (-a[k] * q[k - 1] + d[k]) / (a[k] * p[k - 1] + b[k]);
  }
  x[m] = q[m];
  for (int k = m - 1; k >= 0; --k) x[k] = p[k] * x[k + 1] + q[k];
  for (int k = 0; k < km; ++k) tsl[k] = x[k];
}

// SUBROUTINE urban (module_sf_urban.F:323-2539) for one point. Returns false when WRF stops
// ("ZDC + Z0C + 2m is larger than the 1st WRF level").
// nl: num_roof_layers = num_wall_layers = num_road_layers; utype 1..icate; the inputs as lsm passes them.
CPPWRF_HD inline bool urban_point(const UrbanTables& tb, int nl, int utype, real ta, real qa, real ua, real u1, real v1,
                                  real ssg, real llg, real rain, real rhoo, real za, real declin, real cosz, real omg,
                                  real xlat, real delt, UrbanState& s, UrbanOut& o) {
  constexpr real cp = real(0.24), el = real(583.), sig = real(8.17E-11), pi = real(3.14159), sratio = real(0.75),
                 ell = real(2.442E+06), cpp = real(1004.5);
  constexpr real clc = real(2.5) * real(1.e6) / real(461.51);  // (2.5*10.**6./461.51), folded
  const int u = utype - 1;
  // read_param
  const real zr = tb.zr[u], z0c = tb.z0c[u], z0hc = tb.z0hc[u], zdc = tb.zdc[u], svf = tb.svf[u], r = tb.r[u],
             rw = tb.rw[u], hgt = tb.hgt[u];
  real betr = tb.betr[u], betb = tb.betb[u], betg = tb.betg[u];
  const real capr = tb.capr[u], capb = tb.capb[u], capg = tb.capg[u], aksr = tb.aksr[u], aksb = tb.aksb[u],
             aksg = tb.aksg[u], albr = tb.albr[u], albb = tb.albb[u], albg = tb.albg[u], epsr = tb.epsr[u],
             epsb = tb.epsb[u], epsg = tb.epsg[u], z0r = tb.z0r[u], trlend = tb.trlend[u], tblend = tb.tblend[u],
             tglend = tb.tglend[u], akanda_urban = tb.akanda_urban[u];
  const int boundr = 1, boundb = 1, boundg = 1;

  if (zdc + z0c + real(2.) >= za) return false;

  const real ssgd = sratio * ssg;  // No radiation scheme has SSGD and SSGQ.
  const real ssgq = ssg - ssgd;

  const real w = real(2.) * real(1.) * hgt;
  const real vfgs = svf;
  const real vfgw = real(1.) - svf;
  const real vfwg = (real(1.) - svf) * (real(1.) - r) / w;
  const real vfws = vfwg;
  const real vfww = real(1.) - real(2.) * vfwg;

  const real sx = (ssgd + ssgq) / real(697.7) / real(60.);
  const real sd = ssgd / real(697.7) / real(60.);
  const real sq = ssgq / real(697.7) / real(60.);
  const real rx = llg / real(697.7) / real(60.);
  const real rho = rhoo * real(0.001);

  real trp = s.tr, tbp = s.tb, tgp = s.tg, tcp = s.tc, qcp = s.qc;

  const real tav = ta * (real(1.) + real(0.61) * qa);
  const real ps = rhoo * real(287.) * tav / real(100.);

  // Canopy wind
  real uc;
  if (zr + real(2.) < za) {
    const real ur = ua * libm::log((zr - zdc) / z0c) / libm::log((za - zdc) / z0c);
    const real zc = real(0.7) * zr;
    const real xlb = real(0.4) * (zr - zdc);
    const real bb = real(0.4) * zr / (xlb * libm::log((zr - zdc) / z0c));
    uc = ur * libm::exp(-bb * (real(1.) - zc / zr));
  } else {
    uc = ua / real(2.);
  }
  s.uc = uc;

  // Net short wave radiation at roof, wall and road (SHADOW = .true., TREEOPTION /= 1)
  real sr, sg, sb, snet;
  if (ssg > real(0.0)) {
    const real fai = xlat * pi / real(180.);
    const real theatas = std::abs(libm::asin(cosz));
    const real theataz = std::abs(libm::acos(cosz));
    const real snt = libm::cos(declin) * libm::sin(omg) / libm::cos(theatas);
    const real cnt = (cosz * libm::sin(fai) - libm::sin(declin)) / libm::cos(theatas) / libm::cos(fai);
    const real houi1 = (snt * libm::cos(pi / real(8.)) - cnt * libm::sin(pi / real(8.)));
    const real houi2 = (snt * libm::cos(real(2.) * pi / real(8.)) - cnt * libm::sin(real(2.) * pi / real(8.)));
    const real houi3 = (snt * libm::cos(real(3.) * pi / real(8.)) - cnt * libm::sin(real(3.) * pi / real(8.)));
    const real houi4 = (snt * libm::cos(real(4.) * pi / real(8.)) - cnt * libm::sin(real(4.) * pi / real(8.)));
    const real houi5 = (snt * libm::cos(real(5.) * pi / real(8.)) - cnt * libm::sin(real(5.) * pi / real(8.)));
    const real houi6 = (snt * libm::cos(real(6.) * pi / real(8.)) - cnt * libm::sin(real(6.) * pi / real(8.)));
    const real houi7 = (snt * libm::cos(real(7.) * pi / real(8.)) - cnt * libm::sin(real(7.) * pi / real(8.)));
    const real houi8 = (snt * libm::cos(real(8.) * pi / real(8.)) - cnt * libm::sin(real(8.) * pi / real(8.)));
    const real tz = std::abs(libm::tan(theataz));
    real slx1 = hgt * tz * std::abs(houi1), slx2 = hgt * tz * std::abs(houi2), slx3 = hgt * tz * std::abs(houi3),
         slx4 = hgt * tz * std::abs(houi4), slx5 = hgt * tz * std::abs(houi5), slx6 = hgt * tz * std::abs(houi6),
         slx7 = hgt * tz * std::abs(houi7), slx8 = hgt * tz * std::abs(houi8);
    if (slx1 > rw) slx1 = rw;
    if (slx2 > rw) slx2 = rw;
    if (slx3 > rw) slx3 = rw;
    if (slx4 > rw) slx4 = rw;
    if (slx5 > rw) slx5 = rw;
    if (slx6 > rw) slx6 = rw;
    if (slx7 > rw) slx7 = rw;
    if (slx8 > rw) slx8 = rw;
    const real slx = (slx1 + slx2 + slx3 + slx4 + slx5 + slx6 + slx7 + slx8) / real(8.);

    const real sr1 = sd * (real(1.) - albr) + sq * (real(1.) - albr);
    const real sg1 = sd * (rw - slx) / rw * (real(1.) - albg) + sq * vfgs * (real(1.) - albg);
    const real sb1 = sd * slx / w * (real(1.) - albb) + sq * vfws * (real(1.) - albb);
    const real sg2 = sb1 * albb / (real(1.) - albb) * vfgw * (real(1.) - albg);
    const real sb2 = sg1 * albg / (real(1.) - albg) * vfwg * (real(1.) - albb) + sb1 * albb * vfww;
    sr = sr1;
    sg = sg1 + sg2;
    sb = sb1 + sb2;
    const real st = real(0.);
    snet = r * sr + w * sb + rw * sg + st;
  } else {
    sr = real(0.);
    sg = real(0.);
    sb = real(0.);
    snet = real(0.);
  }
  (void)sx;

  // Roof: CHR, CDR, BETR (SFCDIF_URB)
  const real t1vr = trp * (real(1.0) + real(0.61) * qa);
  const real th2v = (ta + (real(0.0098) * za)) * (real(1.0) + real(0.61) * qa);
  real rlmo_urb = real(0.0), cdr, cdc;
  sfcdif_urb(za, z0r, t1vr, th2v, ua, akanda_urban, s.cmr, s.chr, rlmo_urb, cdr);
  const real alphar = rho * cp * s.chr;
  const real chr = alphar / rho / cp / ua;

  if (rain > real(1.)) betr = real(0.7);  // IMP_SCHEME 1

  // TS_SCHEME 1: TR by Newton-Raphson, TRL by the tridiagonal heat equation
  real rr = 0, hr = 0, eler = 0, g0r = 0;
  for (int iteration = 1; iteration <= 20; ++iteration) {
    const real es = real(6.11) * libm::exp(clc * (trp - real(273.15)) / (real(273.15) * trp));
    const real desdt = clc * es / (trp * trp);
    const real qs0r = real(0.622) * es / (ps - real(0.378) * es);
    const real dqs0rdtr = desdt * real(0.622) * ps / ((ps - real(0.378) * es) * (ps - real(0.378) * es));

    rr = epsr * (rx - sig * rpow(trp, real(4.)) / real(60.));
    hr = rho * cp * chr * ua * (trp - ta) * real(100.);
    eler = rho * el * chr * ua * betr * (qs0r - qa) * real(100.);
    g0r = aksr * (trp - s.trl[0]) / (tb.dzr[0] / real(2.));

    const real f = sr + rr - hr - eler - g0r;

    const real drrdtr = (-(real(4.) * epsr * sig * rpow(trp, real(3.)))) / real(60.);
    const real dhrdtr = rho * cp * chr * ua * real(100.);
    const real delerdtr = rho * el * chr * ua * betr * dqs0rdtr * real(100.);
    const real dg0rdtr = real(2.) * aksr / tb.dzr[0];

    const real dfdt = drrdtr - dhrdtr - delerdtr - dg0rdtr;
    const real dtr = f / dfdt;

    s.tr = trp - dtr;
    trp = s.tr;

    if (std::abs(f) < real(0.000001) && std::abs(dtr) < real(0.000001)) break;
  }
  multi_layer(nl, boundr, g0r, capr, aksr, s.trl, tb.dzr, delt, trlend);

  const real flxthr = hr / rho / cp / real(100.);
  const real flxhumr = eler / rho / el / real(100.);
  s.flxhumr = flxhumr;

  // Wall and road: CHC (SFCDIF_URB), CH_SCHEME 2 (empirical) for the wall and the road
  const real t1vc = tcp * (real(1.0) + real(0.61) * qa);
  rlmo_urb = real(0.0);
  sfcdif_urb(za, z0c, t1vc, th2v, ua, akanda_urban, s.cmc, s.chc, rlmo_urb, cdc);
  const real alphac = rho * cp * s.chc;
  real alphab = rho * cp * (real(6.15) + real(4.18) * uc) / real(1200.);
  if (uc > real(5.)) alphab = rho * cp * (real(7.51) * rpow(uc, real(0.78))) / real(1200.);
  real alphag = rho * cp * (real(6.15) + real(4.18) * uc) / real(1200.);
  if (uc > real(5.)) alphag = rho * cp * (real(7.51) * rpow(uc, real(0.78))) / real(1200.);
  const real chb = alphab / rho / cp / uc;
  const real chg = alphag / rho / cp / uc;

  betb = real(0.0);  // IMP_SCHEME 1
  if (rain > real(1.)) betg = real(0.7);

  // Energy budget without urban trees, TS_SCHEME 1: TB, TG by Newton-Raphson, TBL, TGL by the heat equation
  real rb = 0, rg = 0, hb = 0, hg = 0, eleb = 0, eleg = 0, g0b = 0, g0g = 0;
  for (int iteration = 1; iteration <= 20; ++iteration) {
    real es = real(6.11) * libm::exp(clc * (tbp - real(273.15)) / (real(273.15) * tbp));
    real desdt = clc * es / (tbp * tbp);
    const real qs0b = real(0.622) * es / (ps - real(0.378) * es);
    const real dqs0bdtb = desdt * real(0.622) * ps / ((ps - real(0.378) * es) * (ps - real(0.378) * es));

    es = real(6.11) * libm::exp(clc * (tgp - real(273.15)) / (real(273.15) * tgp));
    desdt = clc * es / (tgp * tgp);
    const real qs0g = real(0.622) * es / (ps - real(0.378) * es);
    const real dqs0gdtg = desdt * real(0.622) * ps / ((ps - real(0.378) * es) * (ps - real(0.378) * es));

    const real tbp4 = rpow(tbp, real(4.)), tgp4 = rpow(tgp, real(4.));
    const real rg1 = epsg * (rx * vfgs + epsb * vfgw * sig * tbp4 / real(60.) - sig * tgp4 / real(60.));
    const real rb1 =
        epsb * (rx * vfws + epsg * vfwg * sig * tgp4 / real(60.) + epsb * vfww * sig * tbp4 / real(60.) -
                sig * tbp4 / real(60.));
    const real rg2 = epsg * ((real(1.) - epsb) * (real(1.) - svf) * vfws * rx +
                             (real(1.) - epsb) * (real(1.) - svf) * vfwg * epsg * sig * tgp4 / real(60.) +
                             epsb * (real(1.) - epsb) * (real(1.) - svf) * (real(1.) - real(2.) * vfws) * sig * tbp4 /
                                 real(60.));
    const real rb2 = epsb * ((real(1.) - epsg) * vfwg * vfgs * rx +
                             (real(1.) - epsg) * epsb * vfgw * vfwg * sig * (tbp4) / real(60.) +
                             (real(1.) - epsb) * vfws * (real(1.) - real(2.) * vfws) * rx +
                             (real(1.) - epsb) * vfwg * (real(1.) - real(2.) * vfws) * sig * epsg * tgp4 / real(60.) +
                             epsb * (real(1.) - epsb) * (real(1.) - real(2.) * vfws) * (real(1.) - real(2.) * vfws) *
                                 sig * tbp4 / real(60.));
    rg = rg1 + rg2;
    rb = rb1 + rb2;

    const real tb3 = rpow(s.tb, real(3.)), tg3 = rpow(s.tg, real(3.));
    const real drbdtb1 = epsb * (real(4.) * epsb * sig * tb3 * vfww - real(4.) * sig * tb3) / real(60.);
    const real drbdtg1 = epsb * (real(4.) * epsg * sig * tg3 * vfwg) / real(60.);
    const real drbdtb2 = epsb *
                         (real(4.) * (real(1.) - epsg) * epsb * sig * tb3 * vfgw * vfwg +
                          real(4.) * epsb * (real(1.) - epsb) * sig * tb3 * vfww * vfww) /
                         real(60.);
    const real drbdtg2 = epsb * (real(4.) * (real(1.) - epsb) * epsg * sig * tg3 * vfwg * vfww) / real(60.);
    const real drgdtb1 = epsg * (real(4.) * epsb * sig * tb3 * vfgw) / real(60.);
    const real drgdtg1 = epsg * (-(real(4.) * sig * tg3)) / real(60.);
    const real drgdtb2 = epsg * (real(4.) * epsb * (real(1.) - epsb) * sig * tb3 * vfww * vfgw) / real(60.);
    const real drgdtg2 = epsg * (real(4.) * (real(1.) - epsb) * epsg * sig * tg3 * vfwg * vfgw) / real(60.);

    const real drbdtb = drbdtb1 + drbdtb2;
    const real drbdtg = drbdtg1 + drbdtg2;
    const real drgdtb = drgdtb1 + drgdtb2;
    const real drgdtg = drgdtg1 + drgdtg2;

    hb = rho * cp * chb * uc * (tbp - tcp) * real(100.);
    hg = rho * cp * chg * uc * (tgp - tcp) * real(100.);

    const real dtcdtb = w * alphab / (rw * alphac + rw * alphag + w * alphab);
    const real dtcdtg = rw * alphag / (rw * alphac + rw * alphag + w * alphab);

    const real dhbdtb = rho * cp * chb * uc * (real(1.) - dtcdtb) * real(100.);
    const real dhbdtg = rho * cp * chb * uc * (real(0.) - dtcdtg) * real(100.);
    const real dhgdtg = rho * cp * chg * uc * (real(1.) - dtcdtg) * real(100.);
    const real dhgdtb = rho * cp * chg * uc * (real(0.) - dtcdtb) * real(100.);

    eleb = rho * el * chb * uc * betb * (qs0b - qcp) * real(100.);
    eleg = rho * el * chg * uc * betg * (qs0g - qcp) * real(100.);

    const real dqcdtb =
        w * alphab * betb * dqs0bdtb / (rw * alphac + rw * alphag * betg + w * alphab * betb);
    const real dqcdtg =
        rw * alphag * betg * dqs0gdtg / (rw * alphac + rw * alphag * betg + w * alphab * betb);

    const real delebdtb = rho * el * chb * uc * betb * (dqs0bdtb - dqcdtb) * real(100.);
    const real delebdtg = rho * el * chb * uc * betb * (real(0.) - dqcdtg) * real(100.);
    const real delegdtg = rho * el * chg * uc * betg * (dqs0gdtg - dqcdtg) * real(100.);
    const real delegdtb = rho * el * chg * uc * betg * (real(0.) - dqcdtb) * real(100.);

    g0b = aksb * (tbp - s.tbl[0]) / (tb.dzb[0] / real(2.));
    g0g = aksg * (tgp - s.tgl[0]) / (tb.dzg[0] / real(2.));

    const real dg0bdtb = real(2.) * aksb / tb.dzb[0];
    const real dg0bdtg = real(0.);
    const real dg0gdtg = real(2.) * aksg / tb.dzg[0];
    const real dg0gdtb = real(0.);

    const real f = sb + rb - hb - eleb - g0b;
    const real fx = drbdtb - dhbdtb - delebdtb - dg0bdtb;
    const real fy = drbdtg - dhbdtg - delebdtg - dg0bdtg;

    const real gf = sg + rg - hg - eleg - g0g;
    const real gx = drgdtb - dhgdtb - delegdtb - dg0gdtb;
    const real gy = drgdtg - dhgdtg - delegdtg - dg0gdtg;

    const real dtb = (gf * fy - f * gy) / (fx * gy - gx * fy);
    const real dtg = -(gf + gx * dtb) / gy;

    s.tb = tbp + dtb;
    s.tg = tgp + dtg;

    tbp = s.tb;
    tgp = s.tg;

    const real tc1 = rw * alphac + rw * alphag + w * alphab;
    const real tc2 = rw * alphac * ta + rw * alphag * tgp + w * alphab * tbp;
    s.tc = tc2 / tc1;

    const real qc1 = rw * alphac + rw * alphag * betg + w * alphab * betb;
    const real qc2 = rw * alphac * qa + rw * alphag * betg * qs0g + w * alphab * betb * qs0b;
    s.qc = qc2 / qc1;

    const real dtc = tcp - s.tc;
    tcp = s.tc;
    qcp = s.qc;

    if (std::abs(f) < real(0.000001) && std::abs(dtb) < real(0.000001) && std::abs(gf) < real(0.000001) &&
        std::abs(dtg) < real(0.000001) && std::abs(dtc) < real(0.000001))
      break;
  }
  multi_layer(nl, boundb, g0b, capb, aksb, s.tbl, tb.dzb, delt, tblend);
  multi_layer(nl, boundg, g0g, capg, aksg, s.tgl, tb.dzg, delt, tglend);

  const real flxthb = hb / rho / cp / real(100.);
  const real flxhumb = eleb / rho / el / real(100.);
  const real flxthg = hg / rho / cp / real(100.);
  const real flxhumg = eleg / rho / el / real(100.);
  s.flxhumb = flxhumb;
  s.flxhumg = flxhumg;

  // Total fluxes from the urban canopy (groption /= 1, TREEOPTION /= 1, ahoption /= 1)
  const real flxth = (r * flxthr + w * flxthb + rw * flxthg);
  const real flxhum = (r * flxhumr + w * flxhumb + rw * flxhumg);
  const real flxuv = (r * cdr + rw * cdc) * ua * ua;
  const real flxg = (r * g0r + w * g0b + rw * g0g);
  const real lnet = r * rr + w * rb + rw * rg;

  // Convert unit: fluxes and u* T* q* --> WRF
  o.sh = flxth * rhoo * cpp;
  o.lh = flxhum * rhoo * ell;
  o.lh_kinematic = flxhum * rhoo;
  o.lw = llg - (lnet * real(697.7) * real(60.));
  o.sw = ssg - (snet * real(697.7) * real(60.));
  o.alb = real(0.);
  if (std::abs(ssg) > real(0.0001)) o.alb = o.sw / ssg;
  o.g = -(flxg * real(697.7) * real(60.));
  o.rn = (snet + lnet) * real(697.7) * real(60.);

  o.ust = std::sqrt(flxuv);
  const real tst = -flxth / o.ust;
  (void)qa;

  // Diagnostic grid-averaged PSIM PSIH TS QS
  const real z0 = z0c;
  const real z0h = z0hc;
  const real z = za - zdc;

  real xxx = real(0.4) * real(9.81) * z * tst / ta / o.ust / o.ust;
  if (xxx >= real(1.)) xxx = real(1.);
  if (xxx <= real(-5.)) xxx = real(-5.);
  if (xxx > real(0)) {
    o.psim = real(-5.) * xxx;
    o.psih = real(-5.) * xxx;
  } else {
    const real x = rpow(real(1.) - real(16.) * xxx, real(0.25));
    o.psim = real(2.) * libm::log((real(1.) + x) / real(2.)) + libm::log((real(1.) + x * x) / real(2.)) -
             real(2.) * libm::atan(x) + pi / real(2.);
    o.psih = real(2.) * libm::log((real(1.) + x * x) / real(2.));
  }
  o.gz1oz0 = libm::log(z / z0);

  o.ts = ta + flxth / s.chs;
  o.qs = qa + flxhum / s.chs;

  // Diagnostic grid-averaged U10 V10 TH2 Q2
  real xxx2 = (real(2.) / z) * xxx;
  if (xxx2 >= real(1.)) xxx2 = real(1.);
  if (xxx2 <= real(-5.)) xxx2 = real(-5.);
  real psih2;
  if (xxx2 > real(0)) {
    psih2 = real(-5.) * xxx2;
  } else {
    const real x = rpow(real(1.) - real(16.) * xxx2, real(0.25));
    psih2 = real(2.) * libm::log((real(1.) + x * x) / real(2.));
  }
  real xxx10 = (real(10.) / z) * xxx;
  if (xxx10 >= real(1.)) xxx10 = real(1.);
  if (xxx10 <= real(-5.)) xxx10 = real(-5.);
  real psim10;
  if (xxx10 > real(0)) {
    psim10 = real(-5.) * xxx10;
  } else {
    const real x = rpow(real(1.) - real(16.) * xxx10, real(0.25));
    psim10 = real(2.) * libm::log((real(1.) + x) / real(2.)) + libm::log((real(1.) + x * x) / real(2.)) -
             real(2.) * libm::atan(x) + real(2.) * libm::atan(real(1.));
  }
  const real psix = libm::log(z / z0) - o.psim;
  const real psit = libm::log(z / z0h) - o.psih;
  const real psit2 = libm::log(real(2.) / z0h) - psih2;
  const real psix10 = libm::log(real(10.) / z0) - psim10;

  o.u10 = u1 * (psix10 / psix);
  o.v10 = v1 * (psix10 / psix);
  o.th2 = o.ts + (ta - o.ts) * (s.chs / s.chs2);
  o.q2 = o.qs + (qa - o.qs) * (psit2 / psit);
  return true;
}

}  // namespace cppwrf::phys::urban
