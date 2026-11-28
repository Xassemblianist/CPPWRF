// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"

namespace cppwrf::phys {

// vrtqdr_sw for one g-point. ptdbt (the direct beam transmittance from the TOA down to each level) is formed here
// as the callers did (ptdbt[0] = 1, ptdbt[jk + 1] = pdbt[jk] * ptdbt[jk]), and the downward pass and the level fluxes
// run as one loop over jk with ztdn[jk], prdnd[jk], ptdbt[jk] in registers: each value has the same operands and the
// same operations in the same order as WRF's three separate loops (only the stack arrays are gone).
template <int ML>
CPPWRF_FORCEINLINE void vrtqdr_sw_direct(
    int klev,
    const rb* pref,       // (klev+1)
    const rb* prefd,      // (klev+1)
    const rb* ptra,       // (klev+1)
    const rb* ptrad,      // (klev+1)
    const rb* pdbt,       // (klev+1)
    rb zincflx,
    SD2 pfd_out,
    SD2 pfu_out,
    int iw
) {
  rb prup[ML + 1], prupd[ML + 1];

  // Link lowest layer with surface
  rb zreflect = rb(1.0) / (rb(1.0) - prefd[klev] * prefd[klev - 1]);
  prup[klev - 1] = pref[klev - 1] + (ptrad[klev - 1] *
      ((ptra[klev - 1] - pdbt[klev - 1]) * prefd[klev] +
       pdbt[klev - 1] * pref[klev])) * zreflect;
  prupd[klev - 1] = prefd[klev - 1] + ptrad[klev - 1] * ptrad[klev - 1] *
      prefd[klev] * zreflect;

  // Pass from bottom to top
  for (int ikx = klev - 2; ikx >= 0; --ikx) {
    int ikp = ikx + 1;
    zreflect = rb(1.0) / (rb(1.0) - prupd[ikp] * prefd[ikx]);
    prup[ikx] = pref[ikx] + (ptrad[ikx] *
        ((ptra[ikx] - pdbt[ikx]) * prupd[ikp] +
         pdbt[ikx] * prup[ikp])) * zreflect;
    prupd[ikx] = prefd[ikx] + ptrad[ikx] * ptrad[ikx] *
        prupd[ikp] * zreflect;
  }
  prup[klev] = pref[klev];
  prupd[klev] = prefd[klev];

  // Upper boundary conditions: ztdn(1) = 1, prdnd(1) = 0, ptdbt(1) = 1
  rb ztdn = rb(1.0), prdnd = rb(0.0), ptdbt = rb(1.0);
  for (int jk = 0; jk <= klev; ++jk) {
    // Up- and down-welling fluxes at level jk
    const int ikl = klev - jk;
    const rb zrf = rb(1.0) / (rb(1.0) - prdnd * prupd[jk]);
    rb pfu = (ptdbt * prup[jk] +
        (ztdn - ptdbt) * prupd[jk]) * zrf;
    rb pfd = ptdbt + (ztdn - ptdbt +
        ptdbt * prup[jk] * prdnd) * zrf;
    pfu_out[ikl][iw] = zincflx * pfu;
    pfd_out[ikl][iw] = zincflx * pfd;
    if (jk == klev) break;
    // Pass from top to bottom: level jk + 1 (ztdn(2) = ptra(1), prdnd(2) = prefd(1))
    rb ztdn_n, prdnd_n;
    if (jk == 0) {
      ztdn_n = ptra[0];
      prdnd_n = prefd[0];
    } else {
      zreflect = rb(1.0) / (rb(1.0) - prefd[jk] * prdnd);
      ztdn_n = ptdbt * ptra[jk] +
          (ptrad[jk] * ((ztdn - ptdbt) +
           ptdbt * pref[jk] * prdnd)) * zreflect;
      prdnd_n = prefd[jk] + ptrad[jk] * ptrad[jk] *
          prdnd * zreflect;
    }
    ztdn = ztdn_n;
    prdnd = prdnd_n;
    ptdbt = pdbt[jk] * ptdbt;  // the callers' ztdbt(jk + 1) = zdbt(jk) * ztdbt(jk)
  }
}

}  // namespace cppwrf::phys
