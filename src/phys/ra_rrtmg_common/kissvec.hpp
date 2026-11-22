// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include <cmath>
#include <cstdint>

namespace cppwrf::phys {

// Marsaglia's 32-bit KISS ("Keep It Simple Stupid") PRNG
// Bit-exact equivalent to WRF module_ra_rrtmg_lw / module_ra_rrtmg_sw kissvec.
struct KissRng {
  uint32_t seed1{0};
  uint32_t seed2{0};
  uint32_t seed3{0};
  uint32_t seed4{0};

  // WRF McICA seed derivation from bottom 4 layer mid-pressures (in Pa)
  // Fortran: seed1 = (pmid(1) - int(pmid(1))) * 1000000000_im
  CPPWRF_HD void init_from_pressure(rb pmid0, rb pmid1, rb pmid2, rb pmid3) {
    seed1 = static_cast<uint32_t>((pmid0 - static_cast<int64_t>(pmid0)) * rb(1.0e9));
    seed2 = static_cast<uint32_t>((pmid1 - static_cast<int64_t>(pmid1)) * rb(1.0e9));
    seed3 = static_cast<uint32_t>((pmid2 - static_cast<int64_t>(pmid2)) * rb(1.0e9));
    seed4 = static_cast<uint32_t>((pmid3 - static_cast<int64_t>(pmid3)) * rb(1.0e9));
  }

  // Advance PRNG by one step and return uniform float in [0, 1): kissvec's
  // ran_arr = kiss * 2.328306e-10_rb + 0.5_rb in rb
  CPPWRF_HD rb step() {
    seed1 = 69069u * seed1 + 1327217885u;
    seed2 ^= (seed2 << 13);
    seed2 ^= (seed2 >> 17);
    seed2 ^= (seed2 << 5);
    seed3 = 18000u * (seed3 & 65535u) + (seed3 >> 16);
    seed4 = 30903u * (seed4 & 65535u) + (seed4 >> 16);

    uint32_t kiss_u = seed1 + seed2 + (seed3 << 16) + seed4;
    int32_t kiss = static_cast<int32_t>(kiss_u);
    return rb(kiss) * rb(2.328306e-10) + rb(0.5);
  }

  // Advance PRNG by count steps without returning values (e.g. changeSeed = 150)
  CPPWRF_HD void permute(int count) {
    for (int i = 0; i < count; ++i) step();
  }
};

constexpr int kMcicaMaxLayers = 128;  // RRTMG_LW/SW_MAX_LAYERS (layer heights of the exponential overlaps)

// generate_stochastic_clouds (module_ra_rrtmg_lw.F / _sw.F, irng = 0): the cloud overlap (icld = cldovrlp)
// 1 random, 2 maximum-random, 3 maximum, 4 exponential, 5 exponential-random. cldf(l): layer cloud fraction
// (0 below cldmin), alpha(l): exponential overlap parameter of layer l (icld 4/5, l >= 1), out(isub, l, cloudy).
// The random numbers are drawn subcolumn by subcolumn, level by level (icld 3: one per subcolumn; icld 4/5:
// CDF then CDF2 at every level), and each level's CDF transform reads the previous level's final CDF.
template <class CF, class AL, class OUT>
CPPWRF_HD void mcica_subcolumns(int icld, KissRng& rng, int nsub, int nlay, CF cldf, AL alpha, OUT out) {
  for (int isub = 0; isub < nsub; ++isub) {
    rb cdf_prev = rb(0.0), cldf_prev = rb(0.0);
    const rb r3 = icld == 3 ? rng.step() : rb(0.0);
    for (int l = 0; l < nlay; ++l) {
      const rb cf = cldf(l);
      rb cdf;
      if (icld == 3) {
        cdf = r3;
      } else if (icld == 4 || icld == 5) {
        cdf = rng.step();
        const rb cdf2 = rng.step();
        if (l > 0 && cdf2 < alpha(l)) cdf = cdf_prev;
      } else {
        cdf = rng.step();
        if (icld == 2 && l > 0) {
          if (cdf_prev > rb(1.0) - cldf_prev) cdf = cdf_prev;
          else cdf = cdf * (rb(1.0) - cldf_prev);
        }
      }
      out(isub, l, cdf >= rb(1.0) - cf);
      cdf_prev = cdf;
      cldf_prev = cf;
    }
  }
}

// decorr_len of mcica_subcol_lw / _sw: idcor 1 from latitude (degrees) and day of year, else 2500 m
CPPWRF_HD inline rb mcica_decorr_len(int idcor, int juldat, rb lat) {
  if (idcor != 1) return rb(2500.0);
  constexpr rb am1 = rb(1.4315), am2 = rb(2.1219), am4 = -rb(25.584), amr = rb(7.0);
  const rb am3 = juldat > 181 ? -rb(4.0) * amr / rb(365.0) * rb(juldat - 272) : rb(4.0) * amr / rb(365.0) * rb(juldat - 91);
  const rb d = lat - am3;
  return (am1 + am2 * libm::exp(-(d * d) / (am4 * am4))) * rb(1.e3);
}

}  // namespace cppwrf::phys
