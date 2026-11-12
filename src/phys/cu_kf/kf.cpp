// SPDX-License-Identifier: GPL-3.0-or-later
#include "kf.hpp"
#include "core/limits.hpp"

namespace cppwrf::phys::cu_kf {

template <typename T>
void run_kf(KFKernel<T> kernel, const Range2D& range) {
    check_limit("Kain-Fritsch", "nz", kernel.nz, kMaxLevels);
#if defined(CPPWRF_DOUBLE)  // the column arrays in the work buffer (kf.hpp)
    if (kernel.nz <= 48) {
        column_for_work(range, KFRunner<T, 48>{kernel});
    } else {
        column_for_work(range, KFRunner<T, kMaxLevels>{kernel});
    }
#else
    if (kernel.nz <= 48) {
        column_for(range, KFRunner<T, 48>{kernel});
    } else {
        column_for(range, KFRunner<T, kMaxLevels>{kernel});
    }
#endif
    fatal_check();
}

template <typename T>
void kf_reserve([[maybe_unused]] const Range2D& range, [[maybe_unused]] int nz) {
#if defined(CPPWRF_DOUBLE)
    column_work_reserve(range, nz <= 48 ? KFRunner<T, 48>::kWorkBytes : KFRunner<T, kMaxLevels>::kWorkBytes);
#endif
}

template void run_kf<double>(KFKernel<double>, const Range2D&);
template void run_kf<float>(KFKernel<float>, const Range2D&);
template void kf_reserve<double>(const Range2D&, int);
template void kf_reserve<float>(const Range2D&, int);

} // namespace cppwrf::phys::cu_kf
