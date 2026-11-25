// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/bl_qnse/qnsepbl.hpp"

namespace cppwrf::phys::qnsepbl {

void run_qnsepbl(const QNSEPBLKernel& kernel, const Range2D& range) {
#if defined(CPPWRF_LOCAL_ROUNDING)  // the column arrays in the work buffer (as MYJ's)
  column_for_work(range, kernel);
#else
  column_for(range, kernel);
#endif
}

void qnsepbl_reserve([[maybe_unused]] const Range2D& range) {
#if defined(CPPWRF_LOCAL_ROUNDING)
  column_work_reserve(range, QNSEPBLKernel::kWorkBytes);
#endif
}

}  // namespace cppwrf::phys::qnsepbl
