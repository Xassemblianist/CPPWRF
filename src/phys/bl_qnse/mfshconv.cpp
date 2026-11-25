// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/bl_qnse/mfshconv.hpp"

namespace cppwrf::phys::mfshconv {

void run_mfshconv(const MFSHCONVKernel& kernel, const Range2D& range) {
#if defined(CPPWRF_LOCAL_ROUNDING)  // the column arrays in the work buffer (as MYJ's)
  column_for_work(range, kernel);
#else
  column_for(range, kernel);
#endif
}

void mfshconv_reserve([[maybe_unused]] const Range2D& range) {
#if defined(CPPWRF_LOCAL_ROUNDING)
  column_work_reserve(range, MFSHCONVKernel::kWorkBytes);
#endif
}

}  // namespace cppwrf::phys::mfshconv
