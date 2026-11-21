// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_tables.hpp"
#include <algorithm>
#include <cmath>

namespace cppwrf::phys {

CPPWRF_FORCEINLINE void reftra_sw_elem(
    bool chk,
    rb zg,
    rb prmuz,
    rb zto1,
    rb zw,
    rb& pref,
    rb& prefd,
    rb& ptra,
    rb& ptrad,
    const RrtmgSwTables& CPPWRF_RESTRICT tables
) {
  constexpr rb tblint = rb(10000.0);
  constexpr rb od_lo = rb(0.06);
  constexpr rb pade = rb(0.278);
  constexpr rb bpade = rb(1.0) / pade;
  constexpr rb zwcrit = rb(0.9999995);
  constexpr rb eps = rb(1.0e-8);

  if (!chk) {
    pref = rb(0.0);
    ptra = rb(1.0);
    prefd = rb(0.0);
    ptrad = rb(1.0);
    return;
  }

  rb zg3 = rb(3.0) * zg;
  // kmodts == 2
  rb zgamma1 = (rb(8.0) - zw * (rb(5.0) + zg3)) * rb(0.25);
  rb zgamma2 = rb(3.0) * (zw * (rb(1.0) - zg)) * rb(0.25);
  rb zgamma3 = (rb(2.0) - zg3 * prmuz) * rb(0.25);
  rb zgamma4 = rb(1.0) - zgamma3;

  rb zwo = rb(0.0);
  rb denom = rb(1.0);
  if (zg != rb(1.0)) {
    rb ratio = zg / (rb(1.0) - zg);
    denom = rb(1.0) - (rb(1.0) - zw) * (ratio * ratio);  // (zg/(1-zg))**2
  }
  if (zw > rb(0.0) && denom != rb(0.0)) {
    zwo = zw / denom;
  }

  if (zwo >= zwcrit) {
    // Conservative scattering
    rb za = zgamma1 * prmuz;
    rb za1 = za - zgamma3;
    rb zgt = zgamma1 * zto1;

    rb ze1 = std::min(zto1 / prmuz, rb(500.0));
    rb ze2;
    if (ze1 <= od_lo) {
      ze2 = rb(1.0) - ze1 + rb(0.5) * ze1 * ze1;
    } else {
      rb tblind = ze1 / (bpade + ze1);
      int itind = static_cast<int>(tblint * tblind + rb(0.5));
      ze2 = tables.exp_tbl[itind];
    }

    pref = (zgt - za1 * (rb(1.0) - ze2)) / (rb(1.0) + zgt);
    ptra = rb(1.0) - pref;

    prefd = zgt / (rb(1.0) + zgt);
    ptrad = rb(1.0) - prefd;

    if (ze2 == rb(1.0)) {
      pref = rb(0.0);
      ptra = rb(1.0);
      prefd = rb(0.0);
      ptrad = rb(1.0);
    }
  } else {
    // Non-conservative scattering
    rb za1 = zgamma1 * zgamma4 + zgamma2 * zgamma3;
    rb za2 = zgamma1 * zgamma3 + zgamma2 * zgamma4;
    rb zrk = std::sqrt(zgamma1 * zgamma1 - zgamma2 * zgamma2);
    rb zrp = zrk * prmuz;
    rb zrp1 = rb(1.0) + zrp;
    rb zrm1 = rb(1.0) - zrp;
    rb zrk2 = rb(2.0) * zrk;
    rb zrpp = rb(1.0) - zrp * zrp;
    rb zrkg = zrk + zgamma1;
    rb zr1 = zrm1 * (za2 + zrk * zgamma3);
    rb zr2 = zrp1 * (za2 - zrk * zgamma3);
    rb zr3 = zrk2 * (zgamma3 - za2 * prmuz);
    rb zr4 = zrpp * zrkg;
    rb zr5 = zrpp * (zrk - zgamma1);
    rb zt1 = zrp1 * (za1 + zrk * zgamma4);
    rb zt2 = zrm1 * (za1 - zrk * zgamma4);
    rb zt3 = zrk2 * (zgamma4 + za1 * prmuz);
    rb zt4 = zr4;
    rb zt5 = zr5;

    rb zbeta = (zgamma1 - zrk) / zrkg;

    rb ze1 = std::min(zrk * zto1, rb(500.0));
    rb ze2 = std::min(zto1 / prmuz, rb(500.0));

    rb zem1, zep1, zem2, zep2;
    if (ze1 <= od_lo) {
      zem1 = rb(1.0) - ze1 + rb(0.5) * ze1 * ze1;
      zep1 = rb(1.0) / zem1;
    } else {
      rb tblind = ze1 / (bpade + ze1);
      int itind = static_cast<int>(tblint * tblind + rb(0.5));
      zem1 = tables.exp_tbl[itind];
      zep1 = rb(1.0) / zem1;
    }

    if (ze2 <= od_lo) {
      zem2 = rb(1.0) - ze2 + rb(0.5) * ze2 * ze2;
      zep2 = rb(1.0) / zem2;
    } else {
      rb tblind = ze2 / (bpade + ze2);
      int itind = static_cast<int>(tblint * tblind + rb(0.5));
      zem2 = tables.exp_tbl[itind];
      zep2 = rb(1.0) / zem2;
    }

    rb zdenr = zr4 * zep1 + zr5 * zem1;
    rb zdent = zt4 * zep1 + zt5 * zem1;
    if (zdenr >= -eps && zdenr <= eps) {
      pref = eps;
      ptra = zem2;
    } else {
      pref = zw * (zr1 * zep1 - zr2 * zem1 - zr3 * zem2) / zdenr;
      ptra = zem2 - zem2 * zw * (zt1 * zep1 - zt2 * zem1 - zt3 * zep2) / zdent;
    }

    rb zemm = zem1 * zem1;
    rb zdend = rb(1.0) / ((rb(1.0) - zbeta * zemm) * zrkg);
    prefd = zgamma2 * (rb(1.0) - zemm) * zdend;
    ptrad = zrk2 * zem1 * zdend;
  }
}

}  // namespace cppwrf::phys
