// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// RRTMG_LWRAD (WRF module_ra_rrtmg_lw.F) for one column as independent stages, every per-column
// array a strided view (src/phys/ra_rrtmg_common/strided.hpp):
//   rrtmg_lw_prep     per column : RRTMG_LWRAD profiles, cloud properties, McICA subcolumns
//                                  (mcica_subcol_lw, max-random overlap), inatm, secdiff, setcoef
//   rrtmg_lw_setcoef_stage per layer: setcoef of one layer (GPU batch: prep leaves setcoef to it)
//   rrtmg_lw_layer    per layer  : taumol, cldprmc, cloudy-layer flag
//   rrtmg_lw_bandstage per band  : rtrnmc g-point loop of the band (rrtmg_lw_rtrnmc.hpp)
//   rrtmg_lw_level    per level  : rtrnmc band sums (GPU batch: rrtmg_lw_bandsum_level)
//   rrtmg_lw_finish   per column : heating rates and the RRTMG_LWRAD outputs
// The CPU column driver (rrtmg_lw_column) runs them in this order on stride-1 arrays; the GPU
// batch driver runs each stage for all columns (layers, g-points, levels) at once, with the band
// stage as rrtmg_lw_gpointstage per g-point and rrtmg_lw_bandsum_level per level. The arithmetic of
// every value is WRF's (checked bit for bit against WRF dumps on the CPU path).
#include <algorithm>
#include <cmath>
#include <cstddef>

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/kissvec.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_cldprmc.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_rtrnmc.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_setcoef.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_tables.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_taumol.hpp"

namespace cppwrf::phys {

// Constants of RRTMG_LWRAD / inirad (module data in WRF), device-copyable.
struct RrtmgLwConsts {
  rb pprof[60], tprof[60];  // standard profile above the model top
  rb retab[95];             // ice effective radius table (reicalc)
  rb o3ann[31], ppwrkh[32]; // inirad annual ozone and its pressure edges
};
void rrtmg_lw_fill_consts(RrtmgLwConsts& k);  // rrtmg_lw.cpp

// Configuration of one radiation call (all columns).
struct RrtmgLwCallConfig {
  int num_levels, nlayers;
  int o3input;
  rb co2, n2o, ch4, cfc11, cfc12;
  rb g;                    // gravmks of the water paths
  rb heatfac;              // rrtmg_lw heatfac = grav*86400/(cp*100)
  int has_reqc, has_reqi, has_reqs;
  int icld, idcor, juldat;     // McICA overlap (cldovrlp), decorrelation length type, day of year
  int icloud;                  // icloud = 0: RRTMG_LWRAD keeps CLDFRA1D = 0 (a clear column)
};

// Per-column views. nl = layer dimension (>= nlayers), all level arrays nl + 1.
struct LwViews {
  // inputs of the stages after prep / outputs of prep (layer index 0 = surface layer)
  SD pz, tz, pavel, tavel, coldry, wbrodl, semiss, plankbnd, secdiff, reicmc, relqmc, resnmc;
  SD cldfrac, clwpth, ciwpth, cswpth, o3mmr;
  SD2 wkl, wx, planklay, planklev;
  SI jp, jt, jt1, indself, indfor, indminor, icldlyr;
  SD colh2o, colco2, colo3, coln2o, colco, colch4, colo2, colbrd, fac00, fac01, fac10, fac11;
  SD rat_h2oco2, rat_h2oco2_1, rat_h2oo3, rat_h2oo3_1, rat_h2on2o, rat_h2on2o_1, rat_h2och4, rat_h2och4_1;
  SD rat_n2oco2, rat_n2oco2_1, rat_o3co2, rat_o3co2_1, selffac, selffrac, forfac, forfrac, minorfrac, scaleminor,
      scaleminorn2;
  SD2 cldfmc, taucmc;                          // [140][nl]
  SD2 fracs, taug;                             // [nl][140]
  SD2 bu, bd, bcu, bcd;                        // [16][nl + 1] band sums of rtrnmc (not in the GPU batch's layout)
  SD gu0, gcu0;                                // [140] the g-points' level-0 radiances (GPU batch)
  SD totuflux, totdflux, totuclfl, totdclfl, fnet, fnetc, htr, htrc;  // [nl + 1]
  SI iscal;  // [0] laytrop, [1] inflag, [2] iceflag, [3] liqflag, [4] 1 + the top cloudy layer (0: a clear column)
  SD dscal;  // [0] pwvcm
};

// Storage of the views: element e of column col of a batch of ncol columns at base[e*ncol + col]
// (doubles and ints separately). nl = layer dimension (>= nlayers).
// band_sums = false (the GPU batch, whose levels add the band sums in registers): no bu / bd / bcu / bcd.
struct LwLayout {
  int nl;
  bool band_sums = true;
  std::size_t nd = 0, ni = 0;  // doubles and ints per column
  CPPWRF_HD explicit LwLayout(int nl_, bool band_sums_ = true) : nl(nl_), band_sums(band_sums_) {
    views(nullptr, nullptr, 0, 1, &nd, &ni);
  }
  CPPWRF_FORCEINLINE LwViews views(rb* d, int* ip, int col, int ncol, std::size_t* nd_out = nullptr,
                                   std::size_t* ni_out = nullptr) const {
    LwViews v{};
    const int L = nl, L1 = nl + 1, s = ncol;
    std::size_t od = 0, oi = 0;
    auto take = [&](std::size_t n) {
      rb* q = d ? d + od * std::size_t(s) + col : nullptr;
      od += n;
      return q;
    };
    auto v1 = [&](int n) { return SD{take(n), s}; };
    auto v2 = [&](int a, int b) { return SD2{take(std::size_t(a) * b), s, b}; };
    auto i1 = [&](int n) {
      int* q = ip ? ip + oi * std::size_t(s) + col : nullptr;
      oi += n;
      return SI{q, s};
    };
    v.pz = v1(L1); v.tz = v1(L1); v.pavel = v1(L1); v.tavel = v1(L1); v.coldry = v1(L1); v.wbrodl = v1(L1);
    v.semiss = v1(16); v.plankbnd = v1(16); v.secdiff = v1(16);
    v.reicmc = v1(L1); v.relqmc = v1(L1); v.resnmc = v1(L1);
    v.cldfrac = v1(L1); v.clwpth = v1(L1); v.ciwpth = v1(L1); v.cswpth = v1(L1); v.o3mmr = v1(L1);
    v.wkl = v2(7, L); v.wx = v2(4, L); v.planklay = v2(L, 16); v.planklev = v2(L1, 16);
    v.colh2o = v1(L); v.colco2 = v1(L); v.colo3 = v1(L); v.coln2o = v1(L); v.colco = v1(L); v.colch4 = v1(L);
    v.colo2 = v1(L); v.colbrd = v1(L);
    v.fac00 = v1(L); v.fac01 = v1(L); v.fac10 = v1(L); v.fac11 = v1(L);
    v.rat_h2oco2 = v1(L); v.rat_h2oco2_1 = v1(L); v.rat_h2oo3 = v1(L); v.rat_h2oo3_1 = v1(L);
    v.rat_h2on2o = v1(L); v.rat_h2on2o_1 = v1(L); v.rat_h2och4 = v1(L); v.rat_h2och4_1 = v1(L);
    v.rat_n2oco2 = v1(L); v.rat_n2oco2_1 = v1(L); v.rat_o3co2 = v1(L); v.rat_o3co2_1 = v1(L);
    v.selffac = v1(L); v.selffrac = v1(L); v.forfac = v1(L); v.forfrac = v1(L); v.minorfrac = v1(L);
    v.scaleminor = v1(L); v.scaleminorn2 = v1(L);
    v.cldfmc = v2(140, L); v.taucmc = v2(140, L);
    v.fracs = v2(L, 140); v.taug = v2(L, 140);
    if (band_sums) { v.bu = v2(16, L1); v.bd = v2(16, L1); v.bcu = v2(16, L1); v.bcd = v2(16, L1); }
    v.gu0 = v1(140); v.gcu0 = v1(140);
    v.totuflux = v1(L1); v.totdflux = v1(L1); v.totuclfl = v1(L1); v.totdclfl = v1(L1);
    v.fnet = v1(L1); v.fnetc = v1(L1); v.htr = v1(L1); v.htrc = v1(L1);
    v.dscal = v1(1);
    v.jp = i1(L); v.jt = i1(L); v.jt1 = i1(L); v.indself = i1(L); v.indfor = i1(L); v.indminor = i1(L);
    v.icldlyr = i1(L); v.iscal = i1(5);
    if (nd_out) *nd_out = od;
    if (ni_out) *ni_out = oi;
    return v;
  }
};

// Per-column inputs of RRTMG_LWRAD (k = 0 at the surface), model memory views.
struct LwColumnIn {
  rb tsk, emiss, xland, xice, snow, xlat;
  SD p8w, t8w, p3d, pi3d, dz8w, t3d, rho3d, qv3d, qc3d, qi3d, qs3d, cldfra3d, o33d;
  SD re_cloud, re_ice, re_snow;  // p == nullptr: has_req* = 0
  bool has_qc, has_qi, has_qs;   // F_QC, F_QI, F_QS (absent species are zero)
  // RRTMG_LWRAD / RRTMG_SWRAD "for MP option 3": without F_QI and warm_rain, below 273.15 K the cloud water is
  // ice and the rain snow (QI1D = QC1D, QS1D = QR1D, QC1D = 0)
  SD qr3d;
  bool split3;
  SD o3_out;  // o3input != 2, p != nullptr: receives the ozone vmr of the model levels (WRF's O33D = O31D)
};

// inatm of rrtmg_lw: wkl holds the volume mixing ratios on entry, the column amounts on exit.
CPPWRF_HD inline void rrtmg_lw_inatm(int nlayers, const LwViews& v) {
  constexpr rb amd = rb(28.9660);
  constexpr rb amw = rb(18.0160);
  constexpr rb avogad = rb(6.02214199e23);
  constexpr rb grav = rb(9.8066);
  for (int l = 0; l < nlayers; ++l) {
    rb amm = (rb(1.0) - v.wkl[0][l]) * amd + v.wkl[0][l] * amw;
    v.coldry[l] = (v.pz[l] - v.pz[l + 1]) * rb(1.0e3) * avogad / (rb(1.0e2) * grav * amm * (rb(1.0) + v.wkl[0][l]));
  }
  rb amttl = rb(0.0);
  rb wvttl = rb(0.0);
  for (int l = 0; l < nlayers; ++l) {
    rb summol = rb(0.0);
    for (int imol = 1; imol < 7; ++imol) summol += v.wkl[imol][l];
    v.wbrodl[l] = v.coldry[l] * (rb(1.0) - summol);
    for (int imol = 0; imol < 7; ++imol) v.wkl[imol][l] = v.coldry[l] * v.wkl[imol][l];
    amttl = amttl + v.coldry[l] + v.wkl[0][l];  // WRF order: (amttl + coldry) + wkl
    wvttl += v.wkl[0][l];
    for (int ix = 0; ix < 4; ++ix) v.wx[ix][l] = v.coldry[l] * v.wx[ix][l] * rb(1.0e-20);
  }
  rb wvsh = (amw * wvttl) / (amd * amttl);
  v.dscal[0] = wvsh * (rb(1.0e3) * v.pz[0]) / (rb(1.0e2) * grav);  // pwvcm
}

// ---- stage 1: per column ------------------------------------------------------------------------
// setcoef = false: setcoef is left to rrtmg_lw_setcoef_stage (one call per layer, the GPU batch) and laytrop to 0
CPPWRF_FORCEINLINE void rrtmg_lw_prep(const RrtmgLwCallConfig& c, const LwColumnIn& in, const LwViews& v,
                                      const RrtmgLwTables& CPPWRF_RESTRICT tables,
                                      const RrtmgLwConsts& CPPWRF_RESTRICT kc, bool setcoef = true) {
  const int num_levels = c.num_levels, nlayers = c.nlayers;
  constexpr rb deltap = rb(4.0);  // mb
  const rb xland = in.xland;
  auto qc_of = [&](int k) { return in.has_qc ? in.qc3d[k] : rb(0.0); };
  auto qi_of = [&](int k) { return in.has_qi ? in.qi3d[k] : rb(0.0); };
  auto qs_of = [&](int k) { return in.has_qs ? in.qs3d[k] : rb(0.0); };

  // cloud optics flags (RRTMG_LWRAD, icloud != 0)
  int inflglw = 2, iceflglw = 3;
  const int liqflglw = 1;
  const bool hrc = in.re_cloud.p != nullptr, hri = in.re_ice.p != nullptr, hrs = in.re_snow.p != nullptr;
  if (hrc) inflglw = 3;
  if (hri) { inflglw = 4; iceflglw = 4; }
  if (hrs) { inflglw = 5; iceflglw = 5; }
  const bool p3case = !hrs && hri && hrc;  // P3: ice into the snow category for the optics
  if (p3case) { inflglw = 5; iceflglw = 5; }

  // CAM relcalc inputs
  const rb landfrac = rb(2.0) - xland;
  const rb landm = landfrac;
  const rb snowh = rb(0.001) * in.snow;
  const rb icefrac = in.xice;
  constexpr rb rliqland = rb(8.0), rliqocean = rb(14.0), rliqice = rb(14.0), tmelt = rb(273.16);
  // relcalc / reicalc at the layer temperature t (RRTMG_LWRAD passes them tlay)
  auto relcalc = [&](rb t) {
    rb reliq = rliqland + (rliqocean - rliqland) * std::min(rb(1.0), std::max(rb(0.0), (tmelt - t) * rb(0.05)));
    reliq += (rliqocean - reliq) * std::min(rb(1.0), std::max(rb(0.0), snowh * rb(10.0)));
    reliq += (rliqocean - reliq) * std::min(rb(1.0), std::max(rb(0.0), rb(1.0) - landm));
    reliq += (rliqice - reliq) * std::min(rb(1.0), std::max(rb(0.0), icefrac));
    return reliq;
  };
  auto reicalc = [&](rb t) {
    int idx = static_cast<int>(t - rb(179.0));
    idx = std::min(std::max(idx, 1), 94);
    const rb corr = t - static_cast<int>(t);
    return kc.retab[idx - 1] * (rb(1.0) - corr) + kc.retab[idx] * corr;
  };

  // base layer vertical variables, gases, cloud physical properties
  v.pz[0] = in.p8w[0] / rb(100.0);
  v.tz[0] = in.t8w[0];
  for (int k = 0; k < num_levels; ++k) {
    const rb t3 = in.t3d[k];
    v.pavel[k] = in.p3d[k] / rb(100.0);
    v.pz[k + 1] = in.p8w[k + 1] / rb(100.0);
    const rb pdel = v.pz[k] - v.pz[k + 1];
    v.tavel[k] = t3;
    v.tz[k + 1] = in.t8w[k + 1];

    rb qv_val = std::max(in.qv3d[k], rb(1.0e-12));
    v.wkl[0][k] = qv_val * rb(1.607793);  // h2ovmr
    v.wkl[1][k] = c.co2;
    v.wkl[2][k] = rb(0.0);                // o3vmr below
    v.wkl[3][k] = c.n2o;
    v.wkl[4][k] = rb(0.0);                // co
    v.wkl[5][k] = c.ch4;
    v.wkl[6][k] = rb(0.209488);           // o2
    v.wx[0][k] = rb(0.093e-9);            // ccl4
    v.wx[1][k] = c.cfc11;
    v.wx[2][k] = c.cfc12;
    v.wx[3][k] = rb(0.169e-9);            // cfc22

    // hydrometeors (max(0, q)) and effective radii
    rb qc1 = std::max(rb(0.0), qc_of(k)), qi1 = std::max(rb(0.0), qi_of(k)), qs1 = std::max(rb(0.0), qs_of(k));
    if (in.split3 && t3 < rb(273.15)) {  // module_ra_rrtmg_lw.F "For MP option 3" (WSM3)
      qi1 = qc1;
      qs1 = std::max(rb(0.0), in.qr3d[k]);
      qc1 = rb(0.);
    }
    // RRTMG_LWRAD reads CLDFRA3D only IF (ICLOUD .ne. 0) (module_ra_rrtmg_lw.F); bmj_rad_feedback sets CLDFRA with
    // icloud = 0 too
    const rb cf3 = c.icloud != 0 ? in.cldfra3d[k] : rb(0.0);
    rb recloud1d = rb(5.0), reice1d = rb(10.0), resnow1d = rb(10.0);
    if (hrc) {
      rb r = std::max(rb(2.5), in.re_cloud[k] * rb(1.E6));
      if (r <= rb(2.5) && cf3 > rb(0.) && (xland - rb(1.5)) > rb(0.)) r = rb(10.5);       // ocean
      else if (r <= rb(2.5) && cf3 > rb(0.) && (xland - rb(1.5)) < rb(0.)) r = rb(7.5);   // land
      recloud1d = r;
    }
    if (hri) {
      rb r = std::max(rb(5.), in.re_ice[k] * rb(1.E6));
      if (r <= rb(5.) && cf3 > rb(0.)) {
        int idx_rei = int(t3 - rb(179.));
        idx_rei = std::min(std::max(idx_rei, 1), 75);
        const rb corr = t3 - int(t3);
        r = kc.retab[idx_rei - 1] * (rb(1.) - corr) + kc.retab[idx_rei] * corr;
        r = std::max(r, rb(5.0));
      }
      reice1d = r;
    }
    if (hrs) resnow1d = std::max(rb(10.), in.re_snow[k] * rb(1.E6));
    if (p3case) {
      resnow1d = std::max(rb(10.), in.re_ice[k] * rb(1.E6));
      qs1 = qi_of(k);
      qi1 = rb(0.);
      reice1d = rb(10.);
    }
    // relcalc / reicalc (the top model layer again below, after the buffer changed its temperature)
    rb reliq = inflglw >= 3 ? recloud1d : relcalc(t3);
    rb reice = iceflglw >= 4 ? reice1d : reicalc(t3);
    if (iceflglw == 3) {  // Fu (1996) generalized effective size
      reice = reice * rb(1.0315);
      reice = std::min(rb(140.0), reice);
    }
    // grid box and in-cloud water paths (g/m2), CAM method
    rb gicewp = (qi1 + qs1) * pdel * rb(100.0) / c.g * rb(1000.0);
    const rb gliqwp = qc1 * pdel * rb(100.0) / c.g * rb(1000.0);
    rb cicewp = gicewp / std::max(rb(0.01), cf3);
    const rb cliqwp = gliqwp / std::max(rb(0.01), cf3);
    if (iceflglw >= 4) {
      gicewp = qi1 * pdel * rb(100.0) / c.g * rb(1000.0);
      cicewp = gicewp / std::max(rb(0.01), cf3);
    }
    rb csnowp = rb(0.0);
    if (iceflglw == 5) {
      rb snow_mass_factor = rb(0.99);
      if (resnow1d > rb(130.)) {
        snow_mass_factor = std::min(snow_mass_factor, (rb(130.0) / resnow1d) * (rb(130.0) / resnow1d));
        resnow1d = rb(130.0);
      }
      const rb gsnowp = qs1 * snow_mass_factor * pdel * rb(100.0) / c.g * rb(1000.0);
      csnowp = gsnowp / std::max(rb(0.01), cf3);
    }
    v.clwpth[k] = cliqwp;
    v.ciwpth[k] = cicewp;
    v.relqmc[k] = reliq;
    v.reicmc[k] = reice;
    if (inflglw == 5) {
      v.cswpth[k] = csnowp;
      v.resnmc[k] = resnow1d;
    } else {
      v.cswpth[k] = rb(0.);
      v.resnmc[k] = rb(10.);
    }
    v.cldfrac[k] = cf3;
  }

  // buffer layers above the model top (to 0 mb), Cavallo (2010)
  for (int L = num_levels; L < nlayers; ++L) {
    v.pz[L + 1] = v.pz[L] - deltap;
    v.pavel[L] = rb(0.5) * (v.pz[L] + v.pz[L + 1]);
  }
  v.pz[nlayers] = rb(0.00);
  v.pavel[nlayers - 1] = rb(0.5) * (v.pz[nlayers - 1] + v.pz[nlayers]);
  // standard profile temperatures at the column pressure levels, matched to the model top
  auto varint = [&](int L) {
    const rb p_val = v.pz[L];
    int klev = 59;
    if (kc.pprof[59] < p_val) {
      for (int LL = 1; LL < 60; ++LL) {
        if (kc.pprof[LL] < p_val) {
          klev = LL - 1;
          break;
        }
      }
    }
    rb wght = rb(0.0);
    rb vark = kc.tprof[klev];
    rb vark1 = (klev < 59) ? kc.tprof[klev + 1] : kc.tprof[klev];
    if (klev < 59) wght = (p_val - kc.pprof[klev]) / (kc.pprof[klev + 1] - kc.pprof[klev]);
    return wght * (vark1 - vark) + vark;
  };
  const rb vtop = varint(num_levels - 1);
  for (int L = num_levels; L <= nlayers; ++L) {
    v.tz[L] = varint(L) + (v.tz[num_levels - 1] - vtop);
    v.tavel[L - 1] = rb(0.5) * (v.tz[L] + v.tz[L - 1]);
  }
  // RRTMG_LWRAD calls relcalc / reicalc after this adjustment: the top model layer's sizes use its new tlay
  {
    const int k = num_levels - 1;
    if (inflglw < 3) v.relqmc[k] = relcalc(v.tavel[k]);
    if (iceflglw == 3) v.reicmc[k] = std::min(rb(140.0), reicalc(v.tavel[k]) * rb(1.0315));
  }
  for (int L = num_levels; L < nlayers; ++L) {
    for (int m = 0; m < 7; ++m) v.wkl[m][L] = v.wkl[m][num_levels - 1];
    for (int m = 0; m < 4; ++m) v.wx[m][L] = v.wx[m][num_levels - 1];
    v.clwpth[L] = rb(0.0);
    v.ciwpth[L] = rb(0.0);
    v.cswpth[L] = rb(0.0);
    v.relqmc[L] = rb(10.0);
    v.reicmc[L] = rb(10.0);
    v.resnmc[L] = rb(10.0);
    v.cldfrac[L] = rb(0.0);
  }

  // ozone: inirad climatology, o3input = 2 CAM ozone on the model levels, shifted above (o3input = 2 reads the
  // climatology from level num_levels - 1 up only)
  for (int k = 0; k <= nlayers - 1; ++k) {
    if (c.o3input == 2 && k < num_levels - 1) continue;
    rb o3 = rb(0.0);
    const rb pr_k = v.pz[k], pr_k1 = v.pz[k + 1];
    for (int jj = 0; jj < 31; ++jj) {
      rb pb1 = (pr_k > kc.ppwrkh[jj]) ? (pr_k - kc.ppwrkh[jj]) : rb(0.0);
      rb pb2 = (pr_k > kc.ppwrkh[jj + 1]) ? (pr_k - kc.ppwrkh[jj + 1]) : rb(0.0);
      rb pt1 = (pr_k1 > kc.ppwrkh[jj]) ? (pr_k1 - kc.ppwrkh[jj]) : rb(0.0);
      rb pt2 = (pr_k1 > kc.ppwrkh[jj + 1]) ? (pr_k1 - kc.ppwrkh[jj + 1]) : rb(0.0);
      o3 += (pb2 - pb1 - pt2 + pt1) * kc.o3ann[jj];
    }
    v.o3mmr[k] = o3 / (pr_k - pr_k1);
  }
  constexpr rb amdo = rb(0.603461);
  for (int k = 0; k < nlayers; ++k) {
    rb o3vmr;
    if (c.o3input == 2) {
      if (k < num_levels) {
        o3vmr = in.o33d[k];
      } else {
        o3vmr = in.o33d[num_levels - 1] - v.o3mmr[num_levels - 1] * amdo + v.o3mmr[k] * amdo;
        if (o3vmr <= rb(0.)) o3vmr = v.o3mmr[k] * amdo;
      }
    } else {
      o3vmr = v.o3mmr[k] * amdo;
      // o31d(k) = o3vmr(ncol,k), then O33D(I,K,J) = O31D(K) ("output o3 for o3input=0")
      if (in.o3_out.p && k < num_levels) in.o3_out[k] = o3vmr;
    }
    v.wkl[2][k] = o3vmr;
  }

  for (int nb = 0; nb < 16; ++nb) v.semiss[nb] = in.emiss;

  // McICA subcolumns (mcica_subcol_lw / generate_stochastic_clouds, KISS, changeSeed = 150, overlap icld);
  // icld = 0: mcica_subcol_lw returns at once and inatm keeps its zeroed cloud arrays, a clear column
  bool any_cloud = false;
  for (int ilev = 0; ilev < nlayers; ++ilev) {
    if (c.icld != 0 && RSD(v.cldfrac)[ilev] >= rb(1.0e-20)) any_cloud = true;
  }
  const bool run_mcica = CPPWRF_WARP_ANY(any_cloud);
  std::uint64_t icld_mask_lo = 0, icld_mask_hi = 0;
  if (run_mcica && any_cloud) {
    KissRng rng;
    rng.init_from_pressure(RSD(v.pavel)[0] * rb(100.0), RSD(v.pavel)[1] * rb(100.0), RSD(v.pavel)[2] * rb(100.0),
                           RSD(v.pavel)[3] * rb(100.0));
    rng.permute(150);
    // layer mid-point heights from dz8w (exponential overlaps): above the model top the top layer's dz
    rb hgt[kMcicaMaxLayers];
    rb zo_inv = rb(0.0);
    if (c.icld == 4 || c.icld == 5) {
      rb dzsum = rb(0.0), dz = rb(0.0);
      for (int k = 0; k < nlayers; ++k) {
        if (k < num_levels) dz = in.dz8w[k];
        hgt[k] = dzsum + rb(0.5) * dz;
        dzsum = dzsum + dz;
      }
      zo_inv = rb(1.0) / mcica_decorr_len(c.idcor, c.juldat, in.xlat);
    }
    auto cldf_of = [&](int l) { const rb cf = RSD(v.cldfrac)[l]; return cf < rb(1.0e-20) ? rb(0.0) : cf; };
    auto alpha_of = [&](int l) {
      if (c.icld == 5 && cldf_of(l) == rb(0.0) && cldf_of(l - 1) > rb(0.0)) return rb(0.0);
      return libm::exp(-(hgt[l] - hgt[l - 1]) * zo_inv);
    };
    // cloudy subcolumns get the layer's in-cloud paths (cldprmc reads them through cldfmc)
    mcica_subcolumns(c.icld, rng, 140, nlayers, cldf_of, alpha_of, [&](int isubcol, int ilev, bool cld) {
      if (cldf_of(ilev) > rb(0.0)) {
        v.cldfmc[isubcol][ilev] = cld ? rb(1.0) : rb(0.0);
        if (cld) {
          if (ilev < 64) icld_mask_lo |= (std::uint64_t(1) << ilev);
          else icld_mask_hi |= (std::uint64_t(1) << (ilev - 64));
        }
      }
    });
  }
  int top_cld_plus1 = 0;  // 1 + the top cloudy layer, 0 without one
  for (int ilev = 0; ilev < nlayers; ++ilev) {
    const int bit = (ilev < 64) ? int((icld_mask_lo >> ilev) & 1u) : int((icld_mask_hi >> (ilev - 64)) & 1u);
    v.icldlyr[ilev] = bit;
    if (bit) top_cld_plus1 = ilev + 1;
  }
  v.iscal[1] = inflglw;
  v.iscal[2] = iceflglw;
  v.iscal[3] = liqflglw;
  v.iscal[4] = top_cld_plus1;

  // inatm
  rrtmg_lw_inatm(nlayers, v);
  rrtmg_lw_secdiff(v.dscal[0], v.secdiff);
  if (!setcoef) {
    v.iscal[0] = 0;
    return;
  }
  // setcoef
  int laytrop = 0;
  rrtmg_lw_setcoef(nlayers, 1, v.pavel, v.tavel, v.tz, in.tsk, v.semiss, v.coldry, v.wkl, v.wbrodl, laytrop, v.jp,
                   v.jt, v.jt1, v.planklay, v.planklev, v.plankbnd, v.colh2o, v.colco2, v.colo3, v.coln2o, v.colco,
                   v.colch4, v.colo2, v.colbrd, v.fac00, v.fac01, v.fac10, v.fac11, v.rat_h2oco2, v.rat_h2oco2_1,
                   v.rat_h2oo3, v.rat_h2oo3_1, v.rat_h2on2o, v.rat_h2on2o_1, v.rat_h2och4, v.rat_h2och4_1,
                   v.rat_n2oco2, v.rat_n2oco2_1, v.rat_o3co2, v.rat_o3co2_1, v.selffac, v.selffrac, v.indself,
                   v.forfac, v.forfrac, v.indfor, v.minorfrac, v.scaleminor, v.scaleminorn2, v.indminor, tables);
  v.iscal[0] = laytrop;
}

// ---- stage 1b (GPU batch): setcoef for layer lay after rrtmg_lw_prep(..., false); laytrop counted into iscal[0]
CPPWRF_FORCEINLINE void rrtmg_lw_setcoef_stage(int lay, rb tsk, const LwViews& v,
                                               const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  const bool lower = rrtmg_lw_setcoef_layer(
      lay, 1, v.pavel, v.tavel, v.tz, tsk, v.semiss, v.coldry, v.wkl, v.wbrodl, v.jp, v.jt, v.jt1, v.planklay,
      v.planklev, v.plankbnd, v.colh2o, v.colco2, v.colo3, v.coln2o, v.colco, v.colch4, v.colo2, v.colbrd, v.fac00,
      v.fac01, v.fac10, v.fac11, v.rat_h2oco2, v.rat_h2oco2_1, v.rat_h2oo3, v.rat_h2oo3_1, v.rat_h2on2o,
      v.rat_h2on2o_1, v.rat_h2och4, v.rat_h2och4_1, v.rat_n2oco2, v.rat_n2oco2_1, v.rat_o3co2, v.rat_o3co2_1,
      v.selffac, v.selffrac, v.indself, v.forfac, v.forfrac, v.indfor, v.minorfrac, v.scaleminor, v.scaleminorn2,
      v.indminor, tables);
  if (lower) {
#if defined(CPPWRF_DEVICE_CODE)
    atomicAdd(&v.iscal[0], 1);
#else
#pragma omp atomic
    v.iscal[0] += 1;
#endif
  }
}

// ---- stage 2: per layer -------------------------------------------------------------------------
CPPWRF_FORCEINLINE void rrtmg_lw_layer(int nlayers, int lay, const LwViews& v,
                                       const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  const int laytrop = CSI(v.iscal)[0];
  if (CSI(v.icldlyr)[lay] != 0) {
    rrtmg_lw_cldprmc(nlayers, CSI(v.iscal)[1], CSI(v.iscal)[2], CSI(v.iscal)[3], v.cldfmc, v.ciwpth, v.clwpth,
                     v.cswpth, v.reicmc, v.relqmc, v.resnmc, v.taucmc, tables, lay, lay + 1);
  }
  rrtmg_lw_taumol(nlayers, v.pavel, v.wx, v.coldry, laytrop, v.jp, v.jt, v.jt1, v.colh2o, v.colco2, v.colo3,
                  v.coln2o, v.colco, v.colch4, v.colo2, v.colbrd, v.fac00, v.fac01, v.fac10, v.fac11, v.rat_h2oco2,
                  v.rat_h2oco2_1, v.rat_h2oo3, v.rat_h2oo3_1, v.rat_h2on2o, v.rat_h2on2o_1, v.rat_h2och4,
                  v.rat_h2och4_1, v.rat_n2oco2, v.rat_n2oco2_1, v.rat_o3co2, v.rat_o3co2_1, v.selffac, v.selffrac,
                  v.indself, v.forfac, v.forfrac, v.indfor, v.minorfrac, v.scaleminor, v.scaleminorn2, v.indminor,
                  v.fracs, v.taug, tables, lay, lay + 1);
}

// ---- stage 3: per band (taut = taug: no aerosols) ----------------------------------------------
template <int MAX_LAY = RRTMG_LW_SWEEP_MAX>
CPPWRF_FORCEINLINE void rrtmg_lw_bandstage(int nlayers, int iband, const LwViews& v,
                                           const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  rrtmg_lw_band<MAX_LAY>(nlayers, iband, v.secdiff, v.semiss, v.cldfmc, v.taucmc, v.planklay, v.planklev, v.plankbnd,
                         v.fracs, v.taug, v.icldlyr, v.bu, v.bd, v.bcu, v.bcd, tables, CSI(v.iscal)[4] == 0);
}

// ---- stage 3 (GPU batch): per g-point igc (0..139) ------------------------------------------------
// rtrnmc's sweeps of one g-point (rrtmg_lw_gpoint) store the radiances of each level in place of the g-point's
// inputs: the down sweep's radld / radclrd of level l in taug[l][igc] / fracs[l][igc] (read at layer l just before),
// the up sweep's radlu / radclru of level lev >= 1 in cldfmc[igc][lev - 1] / taucmc[igc][lev - 1] (cldfmc read at
// layer lev - 1 just before), level 0 in gu0 / gcu0. rrtmg_lw_bandsum_level adds them per band in g-point order.
// The clear-sky radiances below the first cloud (iclddn = 0) are not stored: there rtrnmc's clear-sky sums take the
// all-sky ones (rrtmg_lw_bandsum_level reads fracs at level l only with a cloudy layer at or above it, taucmc only in
// a column with a cloudy layer).
struct LwGpointStore {
  SD2 taug, fracs, cldfmc, taucmc;
  SD gu0, gcu0;
  int igc;
  CPPWRF_FORCEINLINE void down(int l, rb radld, rb radclrd, int iclddn) const {
    taug[l][igc] = radld;
    if (iclddn == 1) fracs[l][igc] = radclrd;
  }
  CPPWRF_FORCEINLINE void up(int lev, rb radlu, rb radclru, int iclddn) const {
    if (lev == 0) {
      gu0[igc] = radlu;
      gcu0[igc] = radclru;
    } else {
      cldfmc[igc][lev - 1] = radlu;
      if (iclddn == 1) taucmc[igc][lev - 1] = radclru;
    }
  }
};
template <int MAX_LAY = RRTMG_LW_SWEEP_MAX>
CPPWRF_FORCEINLINE void rrtmg_lw_gpointstage(int nlayers, int igc, const LwViews& v,
                                             const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  const int iband = tables.ngb[igc] - 1;
  const rb reflect = rb(1.0) - CSD(v.semiss)[iband];
  const rb pbnd = CSD(v.plankbnd)[iband];
  LwGpointStore st{v.taug, v.fracs, v.cldfmc, v.taucmc, v.gu0, v.gcu0, igc};
  const bool fast_clear = CPPWRF_WARP_ALL(CSI(v.iscal)[4] == 0);
  if (fast_clear)
    rrtmg_lw_gpoint<MAX_LAY, true, SD2>(nlayers, iband, igc, v.secdiff, reflect, pbnd, v.cldfmc, v.taucmc,
                                        v.planklay, v.planklev, v.fracs, v.taug, v.icldlyr, tables, st);
  else
    rrtmg_lw_gpoint<MAX_LAY, false, SD2>(nlayers, iband, igc, v.secdiff, reflect, pbnd, v.cldfmc, v.taucmc,
                                         v.planklay, v.planklev, v.fracs, v.taug, v.icldlyr, tables, st);
}

// ---- stage 3b (GPU batch): per level, after rrtmg_lw_gpointstage of every g-point ---------------------------
// The band sums of level lev as rtrnmc forms them (LwBandAcc: each band's g-points in order; below the first cloud
// clrdrad = drad and clrurad = urad, at level 0 clrurad adds radclru), then the level's fluxes from them as
// rrtmg_lw_level (rrtmg_lw_accum: the same operations in the same order) with the band sums in registers.
CPPWRF_FORCEINLINE void rrtmg_lw_bandsum_level(int nlayers, int lev, const LwViews& v,
                                               const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  // rtrnmc's iclddn after the down sweep's layer lev (a cloudy layer at or above it) and at the end of it
  const int top_cld_plus1 = CSI(v.iscal)[4];
  const int cld_dn = top_cld_plus1 > lev ? 1 : 0, cld_col = top_cld_plus1 > 0 ? 1 : 0;
  constexpr rb pi = rb(3.14159265358979323846);
  constexpr rb fluxfac = pi * rb(2.0e4);
  constexpr rb wtdiff = rb(0.5);
  rb tu = rb(0.0), td = rb(0.0), tuc = rb(0.0), tdc = rb(0.0);
  for (int iband = 0; iband < 16; ++iband) {
    const int igc0 = iband == 0 ? 0 : tables.ngs[iband - 1];
    const int igc1 = tables.ngs[iband];
    rb u = rb(0.0), d = rb(0.0), cu = rb(0.0), cd = rb(0.0);
    for (int igc = igc0; igc < igc1; ++igc) {
      if (lev < nlayers) {
        d += CSD(v.taug[lev])[igc];
        if (cld_dn == 1) cd += CSD(v.fracs[lev])[igc];
        else cd = d;
      }
      if (lev == 0) {
        u += CSD(v.gu0)[igc];
        cu += CSD(v.gcu0)[igc];
      } else {
        u += CSD(v.cldfmc[igc])[lev - 1];
        if (cld_col == 1) cu += CSD(v.taucmc[igc])[lev - 1];
        else cu = u;
      }
    }
    // rrtmg_lw_accum
    const rb dw = tables.delwave[iband];
    rb uflux = u * wtdiff;
    rb dflux = d * wtdiff;
    tu += uflux * dw;
    td += dflux * dw;
    rb uclfl = cu * wtdiff;
    rb dclfl = cd * wtdiff;
    tuc += uclfl * dw;
    tdc += dclfl * dw;
  }
  tu *= fluxfac;
  td *= fluxfac;
  tuc *= fluxfac;
  tdc *= fluxfac;
  v.totuflux[lev] = tu; v.totdflux[lev] = td; v.totuclfl[lev] = tuc; v.totdclfl[lev] = tdc;
  v.fnet[lev] = tu - td; v.fnetc[lev] = tuc - tdc;
}

// ---- stage 4: per level -------------------------------------------------------------------------
CPPWRF_FORCEINLINE void rrtmg_lw_level(int lev, const LwViews& v, const RrtmgLwTables& CPPWRF_RESTRICT tables) {
  rb tu, td, tuc, tdc, fn, fnc;
  rrtmg_lw_accum(lev, v.bu, v.bd, v.bcu, v.bcd, tables, tu, td, tuc, tdc, fn, fnc);
  v.totuflux[lev] = tu; v.totdflux[lev] = td; v.totuclfl[lev] = tuc; v.totdclfl[lev] = tdc;
  v.fnet[lev] = fn; v.fnetc[lev] = fnc;
}

// ---- stage 5: per column ------------------------------------------------------------------------
struct LwColumnOut {
  rb glw, olr, lwcf, lwupt, lwuptc, lwdnt, lwdntc, lwupb, lwupbc, lwdnb, lwdnbc;
};
CPPWRF_FORCEINLINE LwColumnOut rrtmg_lw_finish(const RrtmgLwCallConfig& c, const LwViews& v, CSD pi3d,
                                               SD rthratenlw, SD rthratenlwc) {
  const int nlayers = c.nlayers;
  rrtmg_lw_heating(nlayers, v.pz, v.fnet, v.fnetc, v.htr, v.htrc, c.heatfac);
  LwColumnOut o;
  o.glw = v.totdflux[0];
  o.olr = v.totuflux[nlayers];
  o.lwcf = v.totuclfl[nlayers] - v.totuflux[nlayers];
  o.lwupt = v.totuflux[nlayers];
  o.lwuptc = v.totuclfl[nlayers];
  o.lwdnt = v.totdflux[nlayers];
  o.lwdntc = v.totdclfl[nlayers];
  o.lwupb = v.totuflux[0];
  o.lwupbc = v.totuclfl[0];
  o.lwdnb = v.totdflux[0];
  o.lwdnbc = v.totdclfl[0];
  for (int k = 0; k < c.num_levels; ++k) {
    rthratenlw[k] = (v.htr[k] / rb(86400.0)) / pi3d[k];
    if (rthratenlwc.p) rthratenlwc[k] = (v.htrc[k] / rb(86400.0)) / pi3d[k];
  }
  return o;
}

}  // namespace cppwrf::phys
