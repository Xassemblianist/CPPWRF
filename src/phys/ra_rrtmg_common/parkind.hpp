// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// RRTMG's real kind (module parkind of WRF's module_ra_rrtmg_lw.F, which the LW and SW modules use as
// "use parkind, only : im => kind_im, rb => kind_rb"). WRF builds it with kind_rb = kind(1.0), the default REAL:
// double in WRF -r8, float in WRF real*4. RRTMG's arithmetic here (LW, SW, the shared code in ra_rrtmg_common, the
// radiation driver's staging, Dudhia SW) is in rb, the model's real: float in the FP32 builds, double in the FP64
// builds, on the host and on the device. A constant written rb(c) is the double c rounded to rb (c itself in the
// FP64 builds).
#include "core/precision.hpp"

namespace cppwrf::phys {

using rb = ::cppwrf::real;  // kind_rb

}  // namespace cppwrf::phys
