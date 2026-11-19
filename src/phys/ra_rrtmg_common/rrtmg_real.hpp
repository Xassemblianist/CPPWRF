// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// RRTMG's arithmetic type (src/phys/ra_rrtmg_lw, ra_rrtmg_sw, ra_rrtmg_common, the radiation driver, Dudhia SW):
// the model's real, as RRTMG's rb in WRF (parkind: kind_rb = kind(1.0), the native real): float in the FP32 builds
// (WRF real*4, CPU and CUDA alike), double in the FP64 builds (rr(x) is x).
#include "core/precision.hpp"
namespace cppwrf::phys {
using rr = ::cppwrf::real;
}  // namespace cppwrf::phys
