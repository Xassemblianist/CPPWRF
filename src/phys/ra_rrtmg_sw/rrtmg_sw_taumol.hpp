// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_tables.hpp"
#include <algorithm>
#include <cmath>

namespace cppwrf::phys {

// taumol_sw's solar source: WRF zeroes sfluxzen first (module_ra_rrtmg_sw.F taumol_sw): with laytrop = nlayers
// (model top at more than ~191 hPa) bands 16, 17, 27, 28 and 29 never reach their laysolfr and keep 0. Band 26's
// sfluxzen is constant. The other bands write it at the layer where lay == laysolfr inside their layer loops, with
// laysolfr updated in the same loop from jp; taumol_sw_solfr runs those integer loops alone, in WRF's order, and
// stores per band the last layer that writes (WRF's later writes replace the earlier ones) or -1 in solfr[band - 16];
// taumol_sw_layers writes sfluxzen at that layer with the layer's own values, the same writes WRF's last one makes.
// laysolfr: WRF reads jp(nlayers+1) (laytrop = nlayers; never set, and either outcome gives laytrop) and jp(0)
// (laytrop = 0, every layer above 95.6 hPa), outside the layers; here neither is read (lay > 0, lay + 1 < nlayers)
template <class JP>
CPPWRF_FORCEINLINE void taumol_sw_solfr(int nlayers, int laytrop, JP jp, SD sfluxzen, SI solfr,
                                        const RrtmgSwTables& CPPWRF_RESTRICT tables) {
  for (int ig = 0; ig < 112; ++ig) sfluxzen[ig] = rb(0.0);
  for (int ig = 1; ig <= 6; ++ig) sfluxzen[80 + ig - 1] = tables.b26.sfluxref[ig - 1];
  // bands 16, 17, 27, 28, 29: laysolfr from the upper-atmosphere loop
  auto upper = [&](int layreffr) {
    int laysolfr = nlayers - 1, w = -1;
    for (int lay = laytrop; lay < nlayers; ++lay) {
      if (lay > 0 && jp[lay - 1] < layreffr && jp[lay] >= layreffr) laysolfr = lay;
      if (lay == laysolfr) w = lay;
    }
    return w;
  };
  // bands 18 - 25: laysolfr from the lower-atmosphere loop
  auto lower = [&](int layreffr) {
    int laysolfr = laytrop - 1, w = -1;
    for (int lay = 0; lay < laytrop; ++lay) {
      if (lay + 1 < nlayers && jp[lay] < layreffr && jp[lay + 1] >= layreffr)
        laysolfr = std::min(lay + 1, laytrop - 1);
      if (lay == laysolfr) w = lay;
    }
    return w;
  };
  solfr[0] = upper(tables.b16.layreffr);
  solfr[1] = upper(tables.b17.layreffr);
  solfr[2] = lower(tables.b18.layreffr);
  solfr[3] = lower(tables.b19.layreffr);
  solfr[4] = lower(tables.b20.layreffr);
  solfr[5] = lower(tables.b21.layreffr);
  solfr[6] = lower(tables.b22.layreffr);
  solfr[7] = lower(tables.b23.layreffr);
  solfr[8] = lower(tables.b24.layreffr);
  solfr[9] = lower(tables.b25.layreffr);
  solfr[10] = -1;  // band 26: constant, above
  solfr[11] = upper(tables.b27.layreffr);
  solfr[12] = upper(tables.b28.layreffr);
  solfr[13] = upper(tables.b29.layreffr);
}

// taumol_sw for the layers [lay0, lay1) of a column: taug, taur of those layers and sfluxzen where one of them is the
// band's solfr layer (taumol_sw_solfr first). Each value has WRF's operations in WRF's order; only the layer loops
// are cut to [lay0, lay1). The column driver runs [0, nlayers); the GPU batch one layer per thread (KSwLayer).
// Inputs from setcoef_sw: coherent loads (RSD / RSI; the column driver writes them in the same kernel)
CPPWRF_FORCEINLINE void taumol_sw_layers(
    int nlayers,
    int laytrop,
    RSI jp,
    RSI jt,
    RSI jt1,
    RSD colh2o,
    RSD colco2,
    RSD colch4,
    RSD colo2,
    RSD colo3,
    RSD colmol,
    RSD fac00,
    RSD fac01,
    RSD fac10,
    RSD fac11,
    RSD selffac,
    RSD selffrac,
    RSI indself,
    RSD forfac,
    RSD forfrac,
    RSI indfor,
    SD2 taug,                 // (nlayers, 112)
    SD2 taur,                 // (nlayers, 112)
    SD sfluxzen,         // (112)
    RSI solfr,           // (14) taumol_sw_solfr
    const RrtmgSwTables& CPPWRF_RESTRICT tables,
    int lay0, int lay1
) {
  constexpr rb oneminus = rb(1.0) - rb(1.0e-6);

  // Band 16 (2600-3250 cm-1): ng16 = 6, offset = 0
  {
    const auto& b = tables.b16;
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat1 * colch4[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 6; ++ig) {
        taug[lay][ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)])));
        taur[lay][ig - 1] = tauray;
      }
    }

    const int laysolfr = solfr[0];
    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 6; ++ig) {
        taug[lay][ig - 1] = colch4[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]);
        if (lay == laysolfr) sfluxzen[ig - 1] = b.sfluxref[ig - 1];
        taur[lay][ig - 1] = tauray;
      }
    }
  }

  // Band 17 (3250-4000 cm-1): ng17 = 12, offset = 6
  {
    const auto& b = tables.b17;
    int offset = 6;
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colco2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 12; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)])));
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    const int laysolfr = solfr[1];
    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colco2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(4.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 5 + js;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 5 + js;
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 12; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.kb[(ig - 1) * 1175 + (ind0 - 1)] +
            fac100 * b.kb[(ig - 1) * 1175 + (ind0 + 1 - 1)] +
            fac010 * b.kb[(ig - 1) * 1175 + (ind0 + 5 - 1)] +
            fac110 * b.kb[(ig - 1) * 1175 + (ind0 + 6 - 1)] +
            fac001 * b.kb[(ig - 1) * 1175 + (ind1 - 1)] +
            fac101 * b.kb[(ig - 1) * 1175 + (ind1 + 1 - 1)] +
            fac011 * b.kb[(ig - 1) * 1175 + (ind1 + 5 - 1)] +
            fac111 * b.kb[(ig - 1) * 1175 + (ind1 + 6 - 1)]) +
            colh2o[lay] * forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)]));
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 12 + (ig - 1)] +
              fs * (b.sfluxref[js * 12 + (ig - 1)] - b.sfluxref[(js - 1) * 12 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 18 (4000-4650 cm-1): ng18 = 8, offset = 18
  {
    const auto& b = tables.b18;
    int offset = 18;
    const int laysolfr = solfr[2];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colch4[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 8; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)])));
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 8 + (ig - 1)] +
              fs * (b.sfluxref[js * 8 + (ig - 1)] - b.sfluxref[(js - 1) * 8 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 8; ++ig) {
        taug[lay][offset + ig - 1] = colch4[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]);
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 19 (4650-5150 cm-1): ng19 = 8, offset = 26
  {
    const auto& b = tables.b19;
    int offset = 26;
    const int laysolfr = solfr[3];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colco2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 8; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)])));
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 8 + (ig - 1)] +
              fs * (b.sfluxref[js * 8 + (ig - 1)] - b.sfluxref[(js - 1) * 8 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 8; ++ig) {
        taug[lay][offset + ig - 1] = colco2[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]);
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 20 (5150-6150 cm-1): ng20 = 10, offset = 34
  {
    const auto& b = tables.b20;
    int offset = 34;
    const int laysolfr = solfr[4];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 1 + 1;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 10; ++ig) {
        taug[lay][offset + ig - 1] = colh2o[lay] * (
            fac00[lay] * b.ka[(ig - 1) * 65 + (ind0 - 1)] +
            fac10[lay] * b.ka[(ig - 1) * 65 + (ind0 + 1 - 1)] +
            fac01[lay] * b.ka[(ig - 1) * 65 + (ind1 - 1)] +
            fac11[lay] * b.ka[(ig - 1) * 65 + (ind1 + 1 - 1)] +
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)]))) +
            colch4[lay] * b.absch4[ig - 1];
        if (lay == laysolfr) sfluxzen[offset + ig - 1] = b.sfluxref[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 10; ++ig) {
        taug[lay][offset + ig - 1] = colh2o[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)] +
            forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)]))) +
            colch4[lay] * b.absch4[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 21 (6150-7700 cm-1): ng21 = 10, offset = 44
  {
    const auto& b = tables.b21;
    int offset = 44;
    const int laysolfr = solfr[5];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colco2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 10; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)])));
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 10 + (ig - 1)] +
              fs * (b.sfluxref[js * 10 + (ig - 1)] - b.sfluxref[(js - 1) * 10 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colco2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(4.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 5 + js;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 5 + js;
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 10; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.kb[(ig - 1) * 1175 + (ind0 - 1)] +
            fac100 * b.kb[(ig - 1) * 1175 + (ind0 + 1 - 1)] +
            fac010 * b.kb[(ig - 1) * 1175 + (ind0 + 5 - 1)] +
            fac110 * b.kb[(ig - 1) * 1175 + (ind0 + 6 - 1)] +
            fac001 * b.kb[(ig - 1) * 1175 + (ind1 - 1)] +
            fac101 * b.kb[(ig - 1) * 1175 + (ind1 + 1 - 1)] +
            fac011 * b.kb[(ig - 1) * 1175 + (ind1 + 5 - 1)] +
            fac111 * b.kb[(ig - 1) * 1175 + (ind1 + 6 - 1)]) +
            colh2o[lay] * forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)]));
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 22 (7700-8050 cm-1): ng22 = 2, offset = 54
  {
    const auto& b = tables.b22;
    int offset = 54;
    const int laysolfr = solfr[6];
    constexpr rb o2adj = rb(1.6);
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb o2cont = rb(4.35e-4) * colo2[lay] / (rb(350.0) * rb(2.0));
      rb speccomb = colh2o[lay] + o2adj * b.strrat * colo2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 2; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)]))) +
            o2cont;
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 2 + (ig - 1)] +
              fs * (b.sfluxref[js * 2 + (ig - 1)] - b.sfluxref[(js - 1) * 2 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      rb o2cont = rb(4.35e-4) * colo2[lay] / (rb(350.0) * rb(2.0));
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 2; ++ig) {
        taug[lay][offset + ig - 1] = colo2[lay] * o2adj * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]) +
            o2cont;
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 23 (8050-12850 cm-1): ng23 = 10, offset = 56
  {
    const auto& b = tables.b23;
    int offset = 56;
    const int laysolfr = solfr[7];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 1 + 1;
      int inds = indself[lay];
      int indf = indfor[lay];

      for (int ig = 1; ig <= 10; ++ig) {
        rb tauray = colmol[lay] * b.rayl[ig - 1];
        taug[lay][offset + ig - 1] = colh2o[lay] * (
            b.givfac * (
            fac00[lay] * b.ka[(ig - 1) * 65 + (ind0 - 1)] +
            fac10[lay] * b.ka[(ig - 1) * 65 + (ind0 + 1 - 1)] +
            fac01[lay] * b.ka[(ig - 1) * 65 + (ind1 - 1)] +
            fac11[lay] * b.ka[(ig - 1) * 65 + (ind1 + 1 - 1)]) +
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)])));
        if (lay == laysolfr) sfluxzen[offset + ig - 1] = b.sfluxref[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      for (int ig = 1; ig <= 10; ++ig) {
        taug[lay][offset + ig - 1] = rb(0.0);
        taur[lay][offset + ig - 1] = colmol[lay] * b.rayl[ig - 1];
      }
    }
  }

  // Band 24 (12850-16000 cm-1): ng24 = 8, offset = 66
  {
    const auto& b = tables.b24;
    int offset = 66;
    const int laysolfr = solfr[8];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colh2o[lay] + b.strrat * colo2[lay];
      rb specparm = colh2o[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      int inds = indself[lay];
      int indf = indfor[lay];

      for (int ig = 1; ig <= 8; ++ig) {
        rb tauray = colmol[lay] * (b.rayla[(js - 1) * 8 + (ig - 1)] +
            fs * (b.rayla[js * 8 + (ig - 1)] - b.rayla[(js - 1) * 8 + (ig - 1)]));
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]) +
            colo3[lay] * b.abso3a[ig - 1] +  // WRF order: ozone before the water vapour continuum
            colh2o[lay] * (
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 3 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 3 + (indf + 1 - 1)] - b.forref[(ig - 1) * 3 + (indf - 1)])));
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 8 + (ig - 1)] +
              fs * (b.sfluxref[js * 8 + (ig - 1)] - b.sfluxref[(js - 1) * 8 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;

      for (int ig = 1; ig <= 8; ++ig) {
        rb tauray = colmol[lay] * b.raylb[ig - 1];
        taug[lay][offset + ig - 1] = colo2[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]) +
            colo3[lay] * b.abso3b[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 25 (16000-22650 cm-1): ng25 = 6, offset = 74
  {
    const auto& b = tables.b25;
    int offset = 74;
    const int laysolfr = solfr[9];
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 1 + 1;

      for (int ig = 1; ig <= 6; ++ig) {
        rb tauray = colmol[lay] * b.rayl[ig - 1];
        taug[lay][offset + ig - 1] = colh2o[lay] * (
            fac00[lay] * b.ka[(ig - 1) * 65 + (ind0 - 1)] +
            fac10[lay] * b.ka[(ig - 1) * 65 + (ind0 + 1 - 1)] +
            fac01[lay] * b.ka[(ig - 1) * 65 + (ind1 - 1)] +
            fac11[lay] * b.ka[(ig - 1) * 65 + (ind1 + 1 - 1)]) +
            colo3[lay] * b.abso3a[ig - 1];
        if (lay == laysolfr) sfluxzen[offset + ig - 1] = b.sfluxref[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      for (int ig = 1; ig <= 6; ++ig) {
        rb tauray = colmol[lay] * b.rayl[ig - 1];
        taug[lay][offset + ig - 1] = colo3[lay] * b.abso3b[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 26 (22650-29000 cm-1): ng26 = 6, offset = 80
  {
    const auto& b = tables.b26;
    int offset = 80;
    for (int lay = lay0; lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      for (int ig = 1; ig <= 6; ++ig) {
        taug[lay][offset + ig - 1] = rb(0.0);
        taur[lay][offset + ig - 1] = colmol[lay] * b.rayl[ig - 1];
      }
    }
    // sfluxzen: taumol_sw_solfr
  }

  // Band 27 (29000-38000 cm-1): ng27 = 8, offset = 86
  {
    const auto& b = tables.b27;
    int offset = 86;
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 1 + 1;

      for (int ig = 1; ig <= 8; ++ig) {
        rb tauray = colmol[lay] * b.rayl[ig - 1];
        taug[lay][offset + ig - 1] = colo3[lay] * (
            fac00[lay] * b.ka[(ig - 1) * 65 + (ind0 - 1)] +
            fac10[lay] * b.ka[(ig - 1) * 65 + (ind0 + 1 - 1)] +
            fac01[lay] * b.ka[(ig - 1) * 65 + (ind1 - 1)] +
            fac11[lay] * b.ka[(ig - 1) * 65 + (ind1 + 1 - 1)]);
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    const int laysolfr = solfr[11];
    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;

      for (int ig = 1; ig <= 8; ++ig) {
        rb tauray = colmol[lay] * b.rayl[ig - 1];
        taug[lay][offset + ig - 1] = colo3[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]);
        if (lay == laysolfr) sfluxzen[offset + ig - 1] = b.scalekur * b.sfluxref[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 28 (38000-50000 cm-1): ng28 = 6, offset = 94
  {
    const auto& b = tables.b28;
    int offset = 94;
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      rb speccomb = colo3[lay] + b.strrat * colo2[lay];
      rb specparm = colo3[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(8.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 9 + js;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 9 + js;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 6; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.ka[(ig - 1) * 585 + (ind0 - 1)] +
            fac100 * b.ka[(ig - 1) * 585 + (ind0 + 1 - 1)] +
            fac010 * b.ka[(ig - 1) * 585 + (ind0 + 9 - 1)] +
            fac110 * b.ka[(ig - 1) * 585 + (ind0 + 10 - 1)] +
            fac001 * b.ka[(ig - 1) * 585 + (ind1 - 1)] +
            fac101 * b.ka[(ig - 1) * 585 + (ind1 + 1 - 1)] +
            fac011 * b.ka[(ig - 1) * 585 + (ind1 + 9 - 1)] +
            fac111 * b.ka[(ig - 1) * 585 + (ind1 + 10 - 1)]);
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    const int laysolfr = solfr[12];
    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      rb speccomb = colo3[lay] + b.strrat * colo2[lay];
      rb specparm = colo3[lay] / speccomb;
      if (specparm >= oneminus) specparm = oneminus;
      rb specmult = rb(4.0) * specparm;
      int js = 1 + static_cast<int>(specmult);
      rb fs = specmult - static_cast<rb>(js - 1);

      rb fac000 = (rb(1.0) - fs) * fac00[lay];
      rb fac010 = (rb(1.0) - fs) * fac10[lay];
      rb fac100 = fs * fac00[lay];
      rb fac110 = fs * fac10[lay];
      rb fac001 = (rb(1.0) - fs) * fac01[lay];
      rb fac011 = (rb(1.0) - fs) * fac11[lay];
      rb fac101 = fs * fac01[lay];
      rb fac111 = fs * fac11[lay];

      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 5 + js;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 5 + js;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 6; ++ig) {
        taug[lay][offset + ig - 1] = speccomb * (
            fac000 * b.kb[(ig - 1) * 1175 + (ind0 - 1)] +
            fac100 * b.kb[(ig - 1) * 1175 + (ind0 + 1 - 1)] +
            fac010 * b.kb[(ig - 1) * 1175 + (ind0 + 5 - 1)] +
            fac110 * b.kb[(ig - 1) * 1175 + (ind0 + 6 - 1)] +
            fac001 * b.kb[(ig - 1) * 1175 + (ind1 - 1)] +
            fac101 * b.kb[(ig - 1) * 1175 + (ind1 + 1 - 1)] +
            fac011 * b.kb[(ig - 1) * 1175 + (ind1 + 5 - 1)] +
            fac111 * b.kb[(ig - 1) * 1175 + (ind1 + 6 - 1)]);
        if (lay == laysolfr) {
          sfluxzen[offset + ig - 1] = b.sfluxref[(js - 1) * 6 + (ig - 1)] +
              fs * (b.sfluxref[js * 6 + (ig - 1)] - b.sfluxref[(js - 1) * 6 + (ig - 1)]);
        }
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }

  // Band 29 (820-2600 cm-1): ng29 = 12, offset = 100
  {
    const auto& b = tables.b29;
    int offset = 100;
    for (int lay = lay0; lay < (laytrop < lay1 ? laytrop : lay1); ++lay) {
      int ind0 = ((jp[lay] - 1) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = (jp[lay] * 5 + (jt1[lay] - 1)) * 1 + 1;
      int inds = indself[lay];
      int indf = indfor[lay];
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 12; ++ig) {
        taug[lay][offset + ig - 1] = colh2o[lay] * (
            fac00[lay] * b.ka[(ig - 1) * 65 + (ind0 - 1)] +
            fac10[lay] * b.ka[(ig - 1) * 65 + (ind0 + 1 - 1)] +
            fac01[lay] * b.ka[(ig - 1) * 65 + (ind1 - 1)] +
            fac11[lay] * b.ka[(ig - 1) * 65 + (ind1 + 1 - 1)] +
            selffac[lay] * (b.selfref[(ig - 1) * 10 + (inds - 1)] +
            selffrac[lay] * (b.selfref[(ig - 1) * 10 + (inds + 1 - 1)] - b.selfref[(ig - 1) * 10 + (inds - 1)])) +
            forfac[lay] * (b.forref[(ig - 1) * 4 + (indf - 1)] +
            forfrac[lay] * (b.forref[(ig - 1) * 4 + (indf + 1 - 1)] - b.forref[(ig - 1) * 4 + (indf - 1)]))) +
            colco2[lay] * b.absco2[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }

    const int laysolfr = solfr[13];
    for (int lay = (laytrop > lay0 ? laytrop : lay0); lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
      int ind0 = ((jp[lay] - 13) * 5 + (jt[lay] - 1)) * 1 + 1;
      int ind1 = ((jp[lay] - 12) * 5 + (jt1[lay] - 1)) * 1 + 1;
      rb tauray = colmol[lay] * b.rayl;

      for (int ig = 1; ig <= 12; ++ig) {
        taug[lay][offset + ig - 1] = colco2[lay] * (
            fac00[lay] * b.kb[(ig - 1) * 235 + (ind0 - 1)] +
            fac10[lay] * b.kb[(ig - 1) * 235 + (ind0 + 1 - 1)] +
            fac01[lay] * b.kb[(ig - 1) * 235 + (ind1 - 1)] +
            fac11[lay] * b.kb[(ig - 1) * 235 + (ind1 + 1 - 1)]) +
            colh2o[lay] * b.absh2o[ig - 1];
        if (lay == laysolfr) sfluxzen[offset + ig - 1] = b.sfluxref[ig - 1];
        taur[lay][offset + ig - 1] = tauray;
      }
    }
  }
}

// taumol_sw for a whole column (the column driver); solfr: 14 ints of work space
CPPWRF_FORCEINLINE void taumol_sw(int nlayers, int laytrop, SI jp, SI jt, SI jt1, SD colh2o, SD colco2, SD colch4,
                                  SD colo2, SD colo3, SD colmol, SD fac00, SD fac01, SD fac10, SD fac11, SD selffac,
                                  SD selffrac, SI indself, SD forfac, SD forfrac, SI indfor, SD2 taug, SD2 taur,
                                  SD sfluxzen, SI solfr, const RrtmgSwTables& CPPWRF_RESTRICT tables) {
  taumol_sw_solfr(nlayers, laytrop, RSI(jp), sfluxzen, solfr, tables);
  taumol_sw_layers(nlayers, laytrop, jp, jt, jt1, colh2o, colco2, colch4, colo2, colo3, colmol, fac00, fac01, fac10,
                   fac11, selffac, selffrac, indself, forfac, forfrac, indfor, taug, taur, sfluxzen, solfr, tables, 0,
                   nlayers);
}

}  // namespace cppwrf::phys
