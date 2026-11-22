// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_tables.hpp"
#include <algorithm>
#include <cmath>

namespace cppwrf::phys {

// cldprmc_sw for one (layer, g-point): the McICA subcolumn's cloud fraction (0 or 1) and paths,
// the layer's effective sizes; taucm/ssacm/asmcm hold the input optical properties (0, 1, 0 from
// the subcolumn generator) and return the cloud's, taorm the unscaled optical depth.
CPPWRF_HD inline void cldprmc_sw_elem(int inflag, int iceflag, int liqflag, int ig, rb cldf, rb ciwp,
                                      rb clwp, rb cswp, rb radice, rb radliq, rb radsno,
                                      rb fsfcm, rb& taorm, rb& taucm, rb& ssacm, rb& asmcm,
                                      const RrtmgSwTables& tables) {
  constexpr rb eps = rb(1.0e-6);
  constexpr rb cldmin = rb(1.0e-20);
  (void)eps;
  taorm = taucm;
  rb cwp = ciwp + clwp + cswp;

  if (cldf >= cldmin && (cwp >= cldmin || taucm >= cldmin)) {
    if (inflag == 0) {
      rb taucldorig_a = taucm;
      rb ffp = fsfcm;
      rb ffp1 = rb(1.0) - ffp;
      rb ffpssa = rb(1.0) - ffp * ssacm;
      rb ssacloud_a = ffp1 * ssacm / ffpssa;
      rb taucloud_a = ffpssa * taucldorig_a;

      taorm = taucldorig_a;
      ssacm = ssacloud_a;
      taucm = taucloud_a;
      asmcm = (asmcm - ffp) / ffp1;
    } else if (inflag >= 2) {
      rb extcoice = rb(0.0), ssacoice = rb(0.0), gice = rb(0.0), forwice = rb(0.0);
      rb extcosno = rb(0.0), ssacosno = rb(0.0), gsno = rb(0.0), forwsno = rb(0.0);

      if (ciwp + cswp > rb(0.0)) {
        int ib = tables.ngb[ig]; // 16 to 29
        int ib_idx = ib - 16;   // 0 to 13

        if (iceflag == 1) {
          int icx = 4;
          if (tables.wavenum2[ib_idx] > rb(1.43e04)) icx = 0;
          else if (tables.wavenum2[ib_idx] > rb(7.7e03)) icx = 1;
          else if (tables.wavenum2[ib_idx] > rb(5.3e03)) icx = 2;
          else if (tables.wavenum2[ib_idx] > rb(4.0e03)) icx = 3;
          else if (tables.wavenum2[ib_idx] >= rb(2.5e03)) icx = 4;

          extcoice = tables.abari[icx] + tables.bbari[icx] / radice;
          ssacoice = rb(1.0) - tables.cbari[icx] - tables.dbari[icx] * radice;
          gice = tables.ebari[icx] + tables.fbari[icx] * radice;
          if (gice >= rb(1.0)) gice = rb(1.0) - eps;
          forwice = gice * gice;
        } else if (iceflag == 2) {
          rb factor = (radice - rb(2.0)) / rb(3.0);
          int index = static_cast<int>(factor);
          if (index < 1) index = 1;  // WRF stops ('RADIUS OUT OF BOUNDS') below 5 um
          if (index == 43) index = 42;
          rb fint = factor - static_cast<rb>(index);
          int i0 = index - 1;
          extcoice = tables.extice2[ib_idx * 43 + i0] + fint * (tables.extice2[ib_idx * 43 + i0 + 1] - tables.extice2[ib_idx * 43 + i0]);
          ssacoice = tables.ssaice2[ib_idx * 43 + i0] + fint * (tables.ssaice2[ib_idx * 43 + i0 + 1] - tables.ssaice2[ib_idx * 43 + i0]);
          gice = tables.asyice2[ib_idx * 43 + i0] + fint * (tables.asyice2[ib_idx * 43 + i0 + 1] - tables.asyice2[ib_idx * 43 + i0]);
          forwice = gice * gice;
        } else if (iceflag >= 3) {
          rb factor = (radice - rb(2.0)) / rb(3.0);
          int index = static_cast<int>(factor);
          if (index < 1) index = 1;  // WRF stops ('RADIUS OUT OF BOUNDS') below 5 um
          if (index >= 46) index = 45;
          rb fint = factor - static_cast<rb>(index);
          int i0 = index - 1;
          extcoice = tables.extice3[ib_idx * 46 + i0] + fint * (tables.extice3[ib_idx * 46 + i0 + 1] - tables.extice3[ib_idx * 46 + i0]);
          ssacoice = tables.ssaice3[ib_idx * 46 + i0] + fint * (tables.ssaice3[ib_idx * 46 + i0 + 1] - tables.ssaice3[ib_idx * 46 + i0]);
          gice = tables.asyice3[ib_idx * 46 + i0] + fint * (tables.asyice3[ib_idx * 46 + i0 + 1] - tables.asyice3[ib_idx * 46 + i0]);
          rb fdelta = tables.fdlice3[ib_idx * 46 + i0] + fint * (tables.fdlice3[ib_idx * 46 + i0 + 1] - tables.fdlice3[ib_idx * 46 + i0]);
          forwice = fdelta + rb(0.5) / ssacoice;
          if (forwice > gice) forwice = gice;
        }

        if (cswp > rb(0.0) && iceflag == 5) {
          rb factor = (radsno - rb(2.0)) / rb(3.0);
          int index = static_cast<int>(factor);
          if (index < 1) index = 1;  // WRF stops ('RADIUS OUT OF BOUNDS') below 5 um
          if (index >= 46) index = 45;
          rb fint = factor - static_cast<rb>(index);
          int i0 = index - 1;
          extcosno = tables.extice3[ib_idx * 46 + i0] + fint * (tables.extice3[ib_idx * 46 + i0 + 1] - tables.extice3[ib_idx * 46 + i0]);
          ssacosno = tables.ssaice3[ib_idx * 46 + i0] + fint * (tables.ssaice3[ib_idx * 46 + i0 + 1] - tables.ssaice3[ib_idx * 46 + i0]);
          gsno = tables.asyice3[ib_idx * 46 + i0] + fint * (tables.asyice3[ib_idx * 46 + i0 + 1] - tables.asyice3[ib_idx * 46 + i0]);
          rb fdelta = tables.fdlice3[ib_idx * 46 + i0] + fint * (tables.fdlice3[ib_idx * 46 + i0 + 1] - tables.fdlice3[ib_idx * 46 + i0]);
          forwsno = fdelta + rb(0.5) / ssacosno;
          if (forwsno > gsno) forwsno = gsno;
        }
      }

      // Liquid water clouds
      rb extcoliq = rb(0.0), ssacoliq = rb(0.0), gliq = rb(0.0), forwliq = rb(0.0);
      if (clwp > rb(0.0) && liqflag == 1) {
        int ib = tables.ngb[ig];
        int ib_idx = ib - 16;
        int index = static_cast<int>(radliq - rb(1.5));
        if (index <= 0) index = 1;
        if (index >= 58) index = 57;
        rb fint = radliq - rb(1.5) - static_cast<rb>(index);
        int i0 = index - 1;
        extcoliq = tables.extliq1[ib_idx * 58 + i0] + fint * (tables.extliq1[ib_idx * 58 + i0 + 1] - tables.extliq1[ib_idx * 58 + i0]);
        ssacoliq = tables.ssaliq1[ib_idx * 58 + i0] + fint * (tables.ssaliq1[ib_idx * 58 + i0 + 1] - tables.ssaliq1[ib_idx * 58 + i0]);
        if (fint < rb(0.0) && ssacoliq > rb(1.0)) ssacoliq = tables.ssaliq1[ib_idx * 58 + i0];
        gliq = tables.asyliq1[ib_idx * 58 + i0] + fint * (tables.asyliq1[ib_idx * 58 + i0 + 1] - tables.asyliq1[ib_idx * 58 + i0]);
        forwliq = gliq * gliq;
      }

      rb tauliqorig = clwp * extcoliq;
      rb tauiceorig = ciwp * extcoice;
      rb ssaliq = (clwp > rb(0.0)) ? ssacoliq * (rb(1.0) - forwliq) / (rb(1.0) - forwliq * ssacoliq) : rb(0.0);
      rb tauliq = (rb(1.0) - forwliq * ssacoliq) * tauliqorig;
      rb ssaice = (ciwp > rb(0.0)) ? ssacoice * (rb(1.0) - forwice) / (rb(1.0) - forwice * ssacoice) : rb(0.0);
      rb tauice = (rb(1.0) - forwice * ssacoice) * tauiceorig;
      rb scatliq = ssaliq * tauliq;
      rb scatice = ssaice * tauice;

      if (iceflag < 5) {
        taorm = tauliqorig + tauiceorig;
        taucm = tauliq + tauice;
        if (taucm == rb(0.0)) taucm = cldmin;
        if (scatice == rb(0.0)) scatice = cldmin;
        ssacm = (scatliq + scatice) / taucm;

        if (iceflag == 3 || iceflag == 4) {
          // WRF grouping: scatliq*(gliq - forwliq)/(1 - forwliq) + scatice*((gice - forwice)/(1 - forwice))
          asmcm = (rb(1.0) / (scatliq + scatice)) *
                        (scatliq * (gliq - forwliq) / (rb(1.0) - forwliq) +
                         scatice * ((gice - forwice) / (rb(1.0) - forwice)));
        } else {
          asmcm = (scatliq * (gliq - forwliq) / (rb(1.0) - forwliq) +
                         scatice * (gice - forwice) / (rb(1.0) - forwice)) / (scatliq + scatice);
        }
      } else {
        rb tausnoorig = cswp * extcosno;
        taorm = tauliqorig + tauiceorig + tausnoorig;
        rb ssasno = (cswp > rb(0.0)) ? ssacosno * (rb(1.0) - forwsno) / (rb(1.0) - forwsno * ssacosno) : rb(0.0);
        rb tausno = (rb(1.0) - forwsno * ssacosno) * tausnoorig;
        rb scatsno = ssasno * tausno;
        taucm = tauliq + tauice + tausno;
        if (taucm == rb(0.0)) taucm = cldmin;
        if (scatice == rb(0.0)) scatice = cldmin;
        if (scatsno == rb(0.0)) scatsno = cldmin;
        ssacm = (scatliq + scatice + scatsno) / taucm;

        asmcm = (rb(1.0) / (scatliq + scatice + scatsno)) *
                      (scatliq * (gliq - forwliq) / (rb(1.0) - forwliq) +
                       scatice * ((gice - forwice) / (rb(1.0) - forwice)) +
                       scatsno * ((gsno - forwsno) / (rb(1.0) - forwsno)));
      }
    }
  }
}

}  // namespace cppwrf::phys
