// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_tables.hpp"

namespace cppwrf::phys {

// Computes in-cloud optical depth taucmc[ig][lay] for McICA subcolumns.
// Matches WRF module_ra_rrtmg_lw subroutine cldprmc.
CPPWRF_FORCEINLINE void rrtmg_lw_cldprmc(
    int nlayers,
    int inflag,
    int iceflag,
    int liqflag,
    CSD2 cldfmc,   // [140][nlayers] McICA cloud flags (0 or 1)
    CSD ciwpth,    // [nlayers] in-cloud paths: the McICA subcolumn paths are these where cldfmc = 1
    CSD clwpth,    //           and 0 elsewhere (generate_stochastic_clouds)
    CSD cswpth,
    CSD reicmc,          // [nlayers]
    CSD relqmc,          // [nlayers]
    CSD resnmc,          // [nlayers]
    SD2 taucmc,          // [140][nlayers] (inout)
    const RrtmgLwTables& CPPWRF_RESTRICT tables,
    int lay0, int lay1  // layers [lay0, lay1)
) {
  constexpr rb cldmin = rb(1.0e-20);
  const int icb[16] = {0, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4};

  for (int lay = lay0; lay < (nlayers < lay1 ? nlayers : lay1); ++lay) {
    const rb ciwp_l = ciwpth[lay], clwp_l = clwpth[lay], cswp_l = cswpth[lay];
    const rb radice = reicmc[lay], radliq = relqmc[lay], radsno = resnmc[lay];
#pragma unroll 1
    for (int ig = 0; ig < 140; ++ig) {
      const rb cf = cldfmc[ig][lay];
      const bool cl = cf == rb(1.0);
      const rb ciwp = cl ? ciwp_l : rb(0.0), clwp = cl ? clwp_l : rb(0.0), cswp = cl ? cswp_l : rb(0.0);
      rb cwp = ciwp + clwp + cswp;
      if (cf >= cldmin && (cwp >= cldmin || (inflag == 0 && taucmc[ig][lay] >= cldmin))) {
        if (inflag == 0) {
          // Cloud optical depth already defined in taucmc
          return;
        } else if (inflag >= 2) {
          rb abscoice = rb(0.0);
          rb abscosno = rb(0.0);

          if ((ciwp + cswp) > rb(0.0)) {
            if (iceflag == 0) {
              abscoice = tables.absice0[0] + tables.absice0[1] / radice;
            } else if (iceflag == 1) {
              int ib = icb[tables.ngb[ig] - 1];
              abscoice = tables.absice1[ib * 2 + 0] + tables.absice1[ib * 2 + 1] / radice;
            } else if (iceflag == 2) {
              rb factor = (radice - rb(2.0)) / rb(3.0);
              int index = static_cast<int>(factor);
              if (index >= 43) index = 42;
              if (index < 1) index = 1;
              rb fint = factor - static_cast<rb>(index);
              int ib = tables.ngb[ig] - 1;
              int idx0 = index - 1;
              abscoice = tables.absice2[ib * 43 + idx0] +
                         fint * (tables.absice2[ib * 43 + idx0 + 1] - tables.absice2[ib * 43 + idx0]);
            } else if (iceflag >= 3) {
              // Fu (1996) generalized effective size
              rb factor = (radice - rb(2.0)) / rb(3.0);
              int index = static_cast<int>(factor);
              if (index >= 46) index = 45;
              if (index < 1) index = 1;
              rb fint = factor - static_cast<rb>(index);
              int ib = tables.ngb[ig] - 1;
              int idx0 = index - 1;
              abscoice = tables.absice3[ib * 46 + idx0] +
                         fint * (tables.absice3[ib * 46 + idx0 + 1] - tables.absice3[ib * 46 + idx0]);
            }
          }

          // Incorporate snow if iceflag == 5
          if (cswp > rb(0.0) && iceflag == 5) {
            rb factor = (radsno - rb(2.0)) / rb(3.0);
            int index = static_cast<int>(factor);
            if (index >= 46) index = 45;
            if (index < 1) index = 1;
            rb fint = factor - static_cast<rb>(index);
            int ib = tables.ngb[ig] - 1;
            int idx0 = index - 1;
            abscosno = tables.absice3[ib * 46 + idx0] +
                       fint * (tables.absice3[ib * 46 + idx0 + 1] - tables.absice3[ib * 46 + idx0]);
          }

          // Liquid water droplets
          rb abscoliq = rb(0.0);
          if (clwp > rb(0.0)) {
            if (liqflag == 0) {
              abscoliq = tables.absliq0;
            } else if (liqflag == 1) {
              // Hu and Stamnes (1993)
              int index = static_cast<int>(radliq - rb(1.5));
              if (index < 1) index = 1;
              if (index >= 58) index = 57;
              rb fint = radliq - rb(1.5) - static_cast<rb>(index);
              int ib = tables.ngb[ig] - 1;
              int idx0 = index - 1;
              abscoliq = tables.absliq1[ib * 58 + idx0] +
                         fint * (tables.absliq1[ib * 58 + idx0 + 1] - tables.absliq1[ib * 58 + idx0]);
            }
          }

          taucmc[ig][lay] = ciwp * abscoice +
                            clwp * abscoliq +
                            cswp * abscosno;
        }
      } else {
        taucmc[ig][lay] = rb(0.0);
      }
    }
  }
}

}  // namespace cppwrf::phys
