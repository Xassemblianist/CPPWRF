// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// NSSL 2-moment microphysics: WRF 4.8.0 nssl_2mom_init as phys/module_physics_init.F (mp_init) calls it for
// mp_physics = 18 with nssl_2moment_on = 1, nssl_3moment = 0, nssl_hail_on = 1, nssl_ccn_on = 1, nssl_density_on > 0
// (mp_physics = 17, and 18 with its defaults): the module variables and the lookup tables, statement by statement in
// WRF's order. The &nssl_mp_params namelist group is not read (the model rejects it).
#include <vector>
#include "phys/mp_nssl/nssl_args.hpp"
#include "phys/mp_nssl/nssl_kernel.hpp"

namespace cppwrf::phys::mp_nssl {


// the module's allocated tables (host copies; a GPU build copies them to the device and points NsslParams there)
template <class Real>
struct NsslTables {
  std::vector<Real> tabqvs, tabqis, dtabqvs, dtabqis;     // (nqsat)
  std::vector<double> gamxinflu;                          // (0:nqiacrratio, ialpstart:nqiacralpha, 12, 2)
  std::vector<Real> ciacrratio, qiacrratio, ziacrratio;   // (0:nqiacrratio, ialpstart:nqiacralpha)
};

// SUBROUTINE GAMMADP (the beta function's gamma)
inline double nssl_gammadp(double x) {
  static const double g[26] = {1.0e0, 0.5772156649015329e0, -0.6558780715202538e0, -0.420026350340952e-1,
                               0.1665386113822915e0, -.421977345555443e-1, -.96219715278770e-2, .72189432466630e-2,
                               -.11651675918591e-2, -.2152416741149e-3, .1280502823882e-3, -.201348547807e-4,
                               -.12504934821e-5, .11330272320e-5, -.2056338417e-6, .61160950e-8, .50020075e-8,
                               -.11812746e-8, .1043427e-9, .77823e-11, -.36968e-11, .51e-12, -.206e-13, -.54e-14,
                               .14e-14, .1e-15};
  const double pi = 3.141592653589793e0;
  double ga;
  if (x == double(int(x))) {
    if (x > 0.0) {
      ga = 1.0;
      const int m1 = int(x - 1);
      for (int k = 2; k <= m1; ++k) ga = ga * k;
    } else {
      ga = 1.0e+300;
    }
  } else {
    double z, r = 0;
    if (fabs(x) > 1.0) {
      z = fabs(x);
      const int m = int(z);
      r = 1.0;
      for (int k = 1; k <= m; ++k) r = r * (z - k);
      z = z - m;
    } else {
      z = x;
    }
    double gr = g[25];
    for (int k = 25; k >= 1; --k) gr = gr * z + g[k - 1];
    ga = 1.0 / (gr * z);
    if (fabs(x) > 1.0) {
      ga = ga * r;
      if (x < 0.0) ga = -pi / (x * ga * libm::sin(pi * x));
    }
  }
  return ga;
}
inline double nssl_beta(double p, double q) {
  const double gp = nssl_gammadp(p), gq = nssl_gammadp(q);
  const double ppq = p + q;
  return gp * gq / nssl_gammadp(ppq);
}

// (snu + 1.0)**(-1./3.) as gfortran folds it
inline constexpr std::uint64_t nssl_c1sw_pow_bits = 0x3FFB5C0FBCFEC4D4ull;

// nssl_2mom_init; returns ccn_is_ccna as the call leaves it (config_flags%nssl_ccn_is_ccna)
template <class Real>
inline int nssl_init(NsslParams<Real>& P, NsslTables<Real>& T, const NsslConfig& C) {
  using R = Real;
  // nssl_hail_on, nssl_density_on present (true): hail_on = 1, density_on = 1; icecrystals_on stays 1
  bool turn_on_ccna = false;
  const bool turn_on_cina = false;
  P.ccn = fabs(C.cccn);
  P.alphah = C.alphah;
  P.alphahl = C.alphahl;
  P.cnoh = C.cnoh;
  P.cnohl = C.cnohl;
  P.cnor = C.cnor;
  P.cnos = C.cnos;
  P.rho_qh = C.rho_qh;
  P.rho_qhl = C.rho_qhl;
  P.rho_qs = C.rho_qs;
  P.ccnuf = 0.0;  // abs(nssl_params(14)), reserved = 0
  P.icdx = C.icdx;
  P.icdxhl = C.icdxhl;
  P.ipconc = 5;
  if (P.ihlcnh <= 0) P.ihlcnh = 3;            // ipconc == 5
  if (P.irainbreak == -1) P.irainbreak = 0;   // ipconc < 6
  // nssl_ccn_on: ccn_on = 1 (nssl_ccn_opt absent); irenuc = 5 (>= 5): turn_on_ccna
  turn_on_ccna = true;
  int ccn_is_ccna = 1;  // present with value 0 and ccn_on = 1: set to 1
  P.cwccn = P.ccn;
  P.lhab = 8;
  P.lhl = 8;
  // Bolton saturation tables (iqvsopt = 1)
  T.tabqvs.assign(P.nqsat, 0.0);
  T.tabqis.assign(P.nqsat, 0.0);
  T.dtabqvs.assign(P.nqsat, 0.0);
  T.dtabqis.assign(P.nqsat, 0.0);
  for (int l = 1; l <= P.nqsat; ++l) {
    const R temq = R(163.15) + (l - 1) * P.fqsat;
    T.tabqvs[l - 1] = exp(P.cawbolton * (temq - R(273.15)) / (temq - P.cbwbolton));
    const R d1 = temq - P.cbwbolton;
    T.dtabqvs[l - 1] = ((-P.cawbolton * (R(-273.15) + temq)) / (d1 * d1) + P.cawbolton / (temq - P.cbwbolton)) *
                       T.tabqvs[l - 1];
    T.tabqis[l - 1] = exp(P.cai * (temq - R(273.15)) / (temq - P.cbi));
    const R d2 = temq - P.cbi;
    T.dtabqis[l - 1] = ((-P.cai * (R(-273.15) + temq)) / (d2 * d2) + P.cai / (temq - P.cbi)) * T.tabqis[l - 1];
  }
  P.bx(P.lr) = 0.85;
  P.ax(P.lr) = 1647.81;
  P.fx(P.lr) = 135.477;
  P.bx(P.lh) = 0.6;
  P.ax(P.lh) = 157.71;
  P.bx(P.lhl) = 0.593;
  P.ax(P.lhl) = 179.36;
  {  // DATA mmgraupvt (Milbrandt and Morrison 2013 graupel / hail fall speed coefficients)
    const double c1[9] = {50., 150., 250., 350., 450., 550., 650., 750., 850.};
    const double c2[9] = {62.923, 94.122, 114.74, 131.21, 145.26, 157.71, 168.98, 179.36, 189.02};
    const double c3[9] = {0.67819, 0.63789, 0.62197, 0.61240, 0.60572, 0.60066, 0.59663, 0.59330, 0.59048};
    for (int i = 1; i <= 9; ++i) {
      P.mmgraupvt(i, 1) = c1[i - 1];
      P.mmgraupvt(i, 2) = c2[i - 1];
      P.mmgraupvt(i, 3) = c3[i - 1];
    }
  }
  P.gmoi(0) = 1.e32;
  for (int igam = 1; igam <= P.ngm0; ++igam) {
    const double arg = P.dgam * igam;
    P.gmoi(igam) = gamma_dp(arg);
  }
  const R bxh1 = P.bx(P.lh);
  const R bxhl1 = P.bx(c_max(P.lh, P.lhl));
  const int ni = P.nqiacrratio, ja = P.ialpstart, nj = P.nqiacralpha;
  const std::size_t ntab = std::size_t(ni + 1) * (nj - ja + 1);
  T.gamxinflu.assign(ntab * 24, 0.0);
  T.ciacrratio.assign(ntab, 0.0);
  T.qiacrratio.assign(ntab, 0.0);
  T.ziacrratio.assign(ntab, 0.0);
  auto ix2 = [&](int i, int j) { return std::size_t(i) + std::size_t(ni + 1) * std::size_t(j - ja); };
  auto gx = [&](int i, int j, int k, int l) -> double& {
    return T.gamxinflu[ix2(i, j) + ntab * std::size_t((k - 1) + 12 * (l - 1))];
  };
  for (int j = ja; j <= nj; ++j) {
    const R alp = R(j) * P.dqiacralpha;
    const double y = gamma_dpr(R(1.) + alp);
    const double y2 = gamma_dpr(R(2.) + alp);
    for (int i = 0; i <= ni; ++i) {
      const R ratio = R(i) * P.dqiacrratio;
      const double x = gamxinfdp(R(1.) + alp, ratio);
      T.ciacrratio[ix2(i, j)] = x / y;
      gx(i, j, 1, 1) = x / y;
      gx(i, j, 2, 1) = gamxinfdp(R(2.0) + alp, ratio) / y;
      gx(i, j, 3, 1) = gamxinfdp(R(2.5) + alp + R(0.5) * bxh1, ratio) / y;
      gx(i, j, 5, 1) = (gamma_dpr(R(5.0) + alp) - gamxinfdp(R(5.0) + alp, ratio)) / y;
      gx(i, j, 6, 1) = (gamma_dpr(R(5.5) + alp + R(0.5) * bxh1) - gamxinfdp(R(5.5) + alp + R(0.5) * bxh1, ratio)) / y;
      gx(i, j, 9, 1) = gamxinfdp(R(1.0) + alp, ratio) / y;
      gx(i, j, 10, 1) = gamxinfdp(R(4.0) + alp, ratio) / y;
      gx(i, j, 12, 1) = gamxinfdp(R(2.0) + alp, ratio) / y2;
      gx(i, j, 1, 2) = gx(i, j, 1, 1);
      gx(i, j, 2, 2) = gx(i, j, 2, 1);
      gx(i, j, 3, 2) = gamxinfdp(R(2.5) + alp + R(0.5) * bxhl1, ratio) / y;
      gx(i, j, 5, 2) = gx(i, j, 5, 1);
      gx(i, j, 6, 2) = (gamma_dpr(R(5.5) + alp + R(0.5) * bxhl1) - gamxinfdp(R(5.5) + alp + R(0.5) * bxhl1, ratio)) / y;
      gx(i, j, 9, 2) = gx(i, j, 9, 1);
      gx(i, j, 10, 2) = gx(i, j, 10, 1);
      if (alp > R(1.1)) {
        gx(i, j, 7, 1) = (gamma_dpr(alp - R(1.)) - gamxinfdp(alp - R(1.), ratio)) / y;
        gx(i, j, 8, 1) =
            (gamma_dpr(alp - R(0.5) + R(0.5) * bxh1) - gamxinfdp(alp - R(0.5) + R(0.5) * bxh1, ratio)) / y;
        gx(i, j, 8, 2) =
            (gamma_dpr(alp - R(0.5) + R(0.5) * bxhl1) - gamxinfdp(alp - R(0.5) + R(0.5) * bxhl1, ratio)) / y;
      } else {
        gx(i, j, 7, 1) = (gamma_dpr(R(0.1)) - gamxinfdp(R(0.1), ratio)) / y;
        gx(i, j, 8, 1) =
            (gamma_dpr(R(1.1) - R(0.5) + R(0.5) * bxh1) - gamxinfdp(R(1.1) - R(0.5) + R(0.5) * bxh1, ratio)) / y;
        gx(i, j, 8, 2) =
            (gamma_dpr(R(1.1) - R(0.5) + R(0.5) * bxhl1) - gamxinfdp(R(1.1) - R(0.5) + R(0.5) * bxhl1, ratio)) / y;
      }
      gx(i, j, 7, 2) = gx(i, j, 7, 1);
    }
  }
  for (int j = ja; j <= nj; ++j) T.ciacrratio[ix2(0, j)] = 1.0;
  for (int j = ja; j <= nj; ++j) {
    const R alp = R(j) * P.dqiacralpha;
    const R y = gamma_sp(R(4.) + alp);
    const double y7 = gamma_sp(R(7.) + alp);
    for (int i = 0; i <= ni; ++i) {
      const R ratio = R(i) * P.dqiacrratio;
      double x = gamxinfdp(R(4.) + alp, ratio);
      T.qiacrratio[ix2(i, j)] = x / y;
      gx(i, j, 4, 1) = x / y;
      gx(i, j, 4, 2) = x / y;
      x = gamxinfdp(R(7.) + alp, ratio);
      T.ziacrratio[ix2(i, j)] = x / y7;
      gx(i, j, 11, 1) = x / y7;
      gx(i, j, 11, 2) = x / y7;
    }
  }
  for (int j = ja; j <= nj; ++j) T.qiacrratio[ix2(0, j)] = 1.0;
  P.lccn = 0; P.lccnuf = 0; P.lccna = 0; P.lccnaco = 0; P.lccnanu = 0;
  P.lnc = 0; P.lnr = 0; P.lni = 0; P.lnis = 0; P.lns = 0; P.lnh = 0; P.lnhl = 0;
  P.lvh = 0; P.lvhl = 0; P.lzr = 0; P.lzh = 0; P.lzhl = 0; P.lsw = 0; P.lhw = 0; P.lhlw = 0;
  for (int l = 3; l <= 30; ++l) P.denscale(l) = 0;
  int ltmp = P.lhab;  // iufccn = 0
  P.lccn = ltmp + 1;
  P.lnc = ltmp + 2;
  P.lnr = ltmp + 3;
  P.lni = ltmp + 4;
  P.lns = ltmp + 5;
  P.lnh = ltmp + 6;
  ltmp = P.lnh;
  ltmp = ltmp + 1;  // hail_on == 1
  P.lnhl = ltmp;
  ltmp = ltmp + 1;  // density_on >= 1
  P.lvh = ltmp;
  for (int l = P.lccn; l <= ltmp; ++l) P.denscale(l) = 1;
  ltmp = ltmp + 1;  // density_on == 1 and hail_on == 1
  P.lvhl = ltmp;
  P.denscale(P.lvhl) = 1;
  if (turn_on_ccna) {
    ltmp = ltmp + 1;
    P.lccna = ltmp;
    P.denscale(ltmp) = 1;
  }
  (void)turn_on_cina;
  P.na = ltmp;
  for (int l = 3; l <= 30; ++l) P.ln(l) = 0;
  P.ln(P.lc) = P.lnc; P.ln(P.lr) = P.lnr; P.ln(P.li) = P.lni; P.ln(P.ls) = P.lns; P.ln(P.lh) = P.lnh;
  P.ln(P.lhl) = P.lnhl;
  for (int l = 3; l <= 30; ++l) P.ipc(l) = 0;
  P.ipc(P.lc) = 2; P.ipc(P.lr) = 3; P.ipc(P.li) = 1; P.ipc(P.ls) = 4; P.ipc(P.lh) = 5; P.ipc(P.lhl) = 5;
  P.ldovol = false;
  for (int l = 3; l <= 30; ++l) P.lvol(l) = 0;
  P.lvol(P.li) = P.lvi; P.lvol(P.ls) = P.lvs; P.lvol(P.lh) = P.lvh; P.lvol(P.lhl) = P.lvhl;
  P.lne = c_max(P.lnh, P.lnhl);
  P.lne = c_max(P.lne, P.lvh);
  P.lne = c_max(P.lne, P.lvhl);
  P.lne = c_max(P.lne, P.na);
  for (int l = 3; l <= 30; ++l) P.lsc(l) = 0;
  P.lsc(P.lc) = P.lscw; P.lsc(P.lr) = P.lscr; P.lsc(P.li) = P.lsci; P.lsc(P.ls) = P.lscs; P.lsc(P.lh) = P.lsch;
  P.lsc(P.lhl) = P.lschl;
  for (int il = P.lc; il <= P.lhab; ++il) P.ldovol = P.ldovol || (P.lvol(il) > 1);
  for (int l = 3; l <= 30; ++l) P.lz(l) = 0;
  P.lz(P.lr) = P.lzr; P.lz(P.li) = P.lzi; P.lz(P.ls) = P.lzs; P.lz(P.lh) = P.lzh;
  for (int l = 5; l <= 30; ++l) P.lliq(l) = 0;
  P.lliq(P.ls) = P.lsw; P.lliq(P.lh) = P.lhw; P.lliq(P.lhl) = P.lhlw;
  P.xnu(P.lc) = P.cnu;
  P.xmu(P.lc) = 1.;
  P.xnu(P.lr) = (P.alphar - R(2.0)) / R(3.0);
  P.xmu(P.lr) = R(1.) / R(3.);
  P.xnu(P.li) = P.cinu;
  P.xmu(P.li) = 1.;
  P.dnu(P.lc) = R(3.) * P.xnu(P.lc) + R(2.);
  P.dmu(P.lc) = R(3.) * P.xmu(P.lc);
  P.dnu(P.lr) = R(3.) * P.xnu(P.lr) + R(2.);
  P.dmu(P.lr) = R(3.) * P.xmu(P.lr);
  P.xnu(P.ls) = P.snu;
  P.xmu(P.ls) = 1.;
  P.dnu(P.ls) = R(3.) * P.xnu(P.ls) + R(2.);
  P.dmu(P.ls) = R(3.) * P.xmu(P.ls);
  P.dnu(P.lh) = P.alphah;
  P.dmu(P.lh) = P.dmuh;
  P.xnu(P.lh) = (P.dnu(P.lh) - R(2.)) / R(3.);
  P.xmu(P.lh) = P.dmuh / R(3.);
  {
    const R ah = P.alphah, ar = P.alphar, ahl = P.alphahl;
    P.rz = ((R(4.) + ah) * (R(5.) + ah) * (R(6.) + ah) * (R(1.) + ar) * (R(2.) + ar) * (R(3.) + ar)) /
           ((1 + ah) * (2 + ah) * (3 + ah) * (R(4.) + ar) * (R(5.) + ar) * (R(6.) + ar));
    P.rzhl = ((R(4.) + ahl) * (R(5.) + ahl) * (R(6.) + ahl) * (R(1.) + ar) * (R(2.) + ar) * (R(3.) + ar)) /
             ((1 + ahl) * (2 + ahl) * (3 + ahl) * (R(4.) + ar) * (R(5.) + ar) * (R(6.) + ar));
    P.rzs = ((R(1.) + ar) * (R(2.) + ar) * (R(3.) + ar) * (R(2.) + P.xnu(P.ls))) /
            ((R(4.) + ar) * (R(5.) + ar) * (R(6.) + ar) * (R(1.) + P.xnu(P.ls)));
  }
  P.imltshddmr = c_min(1, P.imltshddmr);
  P.ibinhmlr = 0;
  P.ibinhlmlr = 0;
  P.dnu(P.lhl) = P.alphahl;
  P.dmu(P.lhl) = P.dmuhl;
  P.xnu(P.lhl) = (P.dnu(P.lhl) - R(2.)) / R(3.);
  P.xmu(P.lhl) = P.dmuhl / R(3.);
  P.cno(P.lc) = 1.0e+08;
  P.cno(P.li) = 1.0e+08;
  P.cno(P.lr) = P.cnor;
  P.cno(P.ls) = P.cnos;
  P.cno(P.lh) = P.cnoh;
  P.cno(P.lhl) = P.cnohl;
  for (int l = 3; l <= 30; ++l) P.xdnmx(l) = 900.0;
  P.xdnmx(P.lr) = 1000.0; P.xdnmx(P.lc) = 1000.0; P.xdnmx(P.li) = 917.0; P.xdnmx(P.ls) = 300.0;
  P.xdnmx(P.lh) = 900.0; P.xdnmx(P.lhl) = 900.0;
  for (int l = 3; l <= 30; ++l) P.xdnmn(l) = 900.0;
  P.xdnmn(P.lr) = 1000.0; P.xdnmn(P.lc) = 1000.0; P.xdnmn(P.li) = 100.0; P.xdnmn(P.ls) = 100.0;
  P.xdnmn(P.lh) = P.hdnmn; P.xdnmn(P.lhl) = P.hldnmn;
  for (int l = 3; l <= 30; ++l) P.xdn0(l) = 900.0;
  P.xdn0(P.lc) = 1000.0; P.xdn0(P.li) = 900.0; P.xdn0(P.lr) = 1000.0; P.xdn0(P.ls) = P.rho_qs;
  P.xdn0(P.lh) = P.rho_qh; P.xdn0(P.lhl) = P.rho_qhl;
  P.cdx(P.lr) = 0.60; P.cdx(P.lh) = 0.8; P.cdx(P.ls) = 2.00; P.cdx(P.lhl) = 0.45;
  P.ido(P.lc) = P.idocw; P.ido(P.lr) = P.idorw; P.ido(P.li) = P.idoci; P.ido(P.ls) = P.idosw;
  P.ido(P.lh) = P.idohw; P.ido(P.lhl) = P.idohl;
  if (P.irfall < 0) P.irfall = P.infall;  // lzr = 0
  for (int l = 3; l <= 30; ++l) P.linfall(l) = P.infall;
  P.linfall(P.lc) = 0;
  P.linfall(P.lr) = P.irfall;
  P.linfall(P.ls) = P.isfall;
  P.linfall(P.li) = P.iifall;
  P.qccn = P.ccn / P.rho00;
  P.qccnuf = P.ccnuf / P.rho00;
  P.old_qccn = P.qccn;
  P.xvrmx = P.xvrmx0;
  if (P.dhmn <= R(0.0)) {
    P.xvhmn = P.xvhmn0;
  } else {
    P.xvhmn = R(0.523599) * ipow<3>(P.dhmn);
  }
  P.xvhmx = P.xvhmx0;
  P.xvhlmx = P.xvhlmx0;
  if (P.qhdpvdn < R(0.)) P.qhdpvdn = P.xdnmn(P.lh);
  if (P.qhacidn < R(0.)) P.qhacidn = P.xdnmn(P.lh);
  P.xvmn(P.lc) = P.xvcmn; P.xvmn(P.li) = P.xvimn; P.xvmn(P.lr) = P.xvrmn; P.xvmn(P.ls) = P.xvsmn;
  P.xvmn(P.lh) = P.xvhmn;
  P.xvmx(P.lc) = P.xvcmx; P.xvmx(P.li) = P.xvimx; P.xvmx(P.lr) = P.xvrmx; P.xvmx(P.ls) = P.xvsmx;
  P.xvmx(P.lh) = P.xvhmx;
  P.xvmn(P.lhl) = P.xvhlmn;
  P.xvmx(P.lhl) = P.xvhlmx;
  P.ventr = 1.;
  P.ventr = gamma_sp(R(2.) + P.alphar);  // imurain /= 3
  P.ventrn = gamma_sp(P.alphar + R(2.5) + P.br / R(2.)) / gamma_sp(P.alphar + R(1.));
  P.ventc = gamma_sp(P.cnu + R(4.) / R(3.)) / pow(P.cnu + R(1.), R(1.) / R(3.)) / gamma_sp(P.cnu + R(1.));
  // (snu + 1.0)**(-1./3.): snu is a PARAMETER, so gfortran folds the power (MPFR, correctly rounded)
  P.c1sw = gamma_sp(P.snu + R(4.) / R(3.)) * hexd(nssl_c1sw_pow_bits) / gamma_sp(P.snu + R(1.0));
  for (int l = 3; l <= 30; ++l) P.qxmin(l) = 1.0e-12;
  P.qxmin(P.lc) = 1.e-9; P.qxmin(P.lr) = 1.e-7; P.qxmin(P.li) = 1.e-12; P.qxmin(P.ls) = 1.e-7;
  P.qxmin(P.lh) = 1.e-7; P.qxmin(P.lhl) = 1.e-7;
  P.qxmin(P.lc) = 1.0e-13; P.qxmin(P.lr) = 1.0e-12; P.qxmin(P.li) = 1.0e-13; P.qxmin(P.ls) = 1.0e-13;
  P.qxmin(P.lh) = 1.0e-12; P.qxmin(P.lhl) = 1.0e-12;
  for (int l = 3; l <= 30; ++l) P.qxmin_init(l) = 1.0e-8;
  P.cckm = P.cck - R(1.);
  P.ccnefac = pow(R(1.63) / (P.cck * nssl_beta(R(3.) / R(2.), P.cck / R(2.))), P.cck / (P.cck + R(2.0)));
  P.cnexp = (R(3.) / R(2.)) * P.cck / (P.cck + R(2.0));
  P.ccne = P.ccnefac * R(1.e6) * pow(R(1.e-6) * fabs(P.cwccn), R(2.) / (R(2.) + P.cck));
  P.ccne0 = P.ccnefac * R(1.e6) * pow(R(1.e-6), R(2.) / (R(2.) + P.cck));
  if (P.cwccn < R(0.0)) {
    P.cwccn = fabs(P.cwccn);
    P.ccwmx = 50.e9;
  } else {
    P.ccwmx = 50.e9;
  }
  for (int l = 3; l <= 30; ++l) P.bb(l) = R(1.0) / R(3.0);
  P.bb(P.li) = 0.3429;
  for (int il = P.lc; il <= P.lhab; ++il) {
    P.da0(il) = delbk(P, P.bb(il), P.xnu(il), P.xmu(il), 0);
    P.da1(il) = delbk(P, P.bb(il), P.xnu(il), P.xmu(il), 1);
  }
  for (int a = 3; a <= 30; ++a)
    for (int b = 3; b <= 30; ++b) P.dab0(a, b) = P.dab1(a, b) = 0.0;
  for (int il = P.lc; il <= P.lhab; ++il)
    for (int j = P.lc; j <= P.lhab; ++j)
      if (il != j) {
        P.dab0(il, j) = delabk(P, P.bb(il), P.bb(j), P.xnu(il), P.xnu(j), P.xmu(il), P.xmu(j), 0);
        P.dab1(il, j) = delabk(P, P.bb(il), P.bb(j), P.xnu(il), P.xnu(j), P.xmu(il), P.xmu(j), 1);
      }
  P.gf4br = gamma_sp(R(4.0) + P.br);
  P.gf4ds = gamma_sp(R(4.0) + P.ds);
  P.gf4p5 = gamma_sp(R(4.0) + R(0.5));
  P.gfcinu1 = gamma_sp(P.cinu + R(1.0));
  P.gfcinu1p47 = gamma_sp(P.cinu + R(1.47167));
  P.gfcinu2p47 = gamma_sp(P.cinu + R(2.47167));
  P.gfcinu1p22 = gamma_sp(P.cinu + R(1.22117));
  P.gfcinu2p22 = gamma_sp(P.cinu + R(2.22117));
  P.gfcinu1p18 = gamma_sp(P.cinu + R(1.18333));
  P.gfcinu2p18 = gamma_sp(P.cinu + R(2.18333));
  P.gsnow1 = gamma_sp(P.snu + R(1.0));
  P.gsnow53 = gamma_sp(P.snu + R(5.) / R(3.));
  P.gsnow73 = gamma_sp(P.snu + R(7.) / R(3.));
  P.cwchtmp0 = R(6.0) / P.pi * gamma_sp((P.xnu(P.lh) + R(1.)) / P.xmu(P.lh)) /
               gamma_sp((P.xnu(P.lh) + R(2.)) / P.xmu(P.lh));
  P.cwchltmp0 = R(6.0) / P.pi * gamma_sp((P.xnu(P.lhl) + 1) / P.xmu(P.lhl)) /
                gamma_sp((P.xnu(P.lhl) + 2) / P.xmu(P.lhl));
  for (int a = 3; a <= 30; ++a)
    for (int b = 3; b <= 30; ++b) P.iexy(a, b) = 0;
  P.iexy(P.ls, P.li) = P.ieswi;
  P.iexy(P.ls, P.lc) = P.ieswc; P.iexy(P.ls, P.lr) = P.ieswr;
  P.iexy(P.lh, P.ls) = P.iehwsw; P.iexy(P.lh, P.li) = P.iehwi;
  P.iexy(P.lh, P.lc) = P.iehwc; P.iexy(P.lh, P.lr) = P.iehwr;
  P.iexy(P.lhl, P.ls) = P.iehlsw; P.iexy(P.lhl, P.li) = P.iehli;
  P.iexy(P.lhl, P.lc) = P.iehlc; P.iexy(P.lhl, P.lr) = P.iehlr;
  P.tabqvs_p = T.tabqvs.data();
  P.tabqis_p = T.tabqis.data();
  P.dtabqvs_p = T.dtabqvs.data();
  P.dtabqis_p = T.dtabqis.data();
  P.gamxinflu_p = T.gamxinflu.data();
  P.ciacrratio_p = T.ciacrratio.data();
  P.qiacrratio_p = T.qiacrratio.data();
  P.ziacrratio_p = T.ziacrratio.data();
  return ccn_is_ccna;
}

}  // namespace cppwrf::phys::mp_nssl
