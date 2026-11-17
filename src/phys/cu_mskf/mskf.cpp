// SPDX-License-Identifier: GPL-3.0-or-later
#include "mskf.hpp"
#include "core/limits.hpp"

namespace cppwrf::phys::cu_mskf {

// MSKF_CPS over the columns of range (the column arrays in the work buffer the suite reserved, mskf_work_bytes)
template <typename T>
void run_mskf(MSKFKernel<T> kernel, const Range2D& range) {
    check_limit("Multi-scale Kain-Fritsch", "nz", kernel.nz, kMaxLevels);
    if (kernel.nz <= 48) {
        column_for_work(range, MSKFRunner<T, 48>{kernel});
    } else {
        column_for_work(range, MSKFRunner<T, kMaxLevels>{kernel});
    }
    fatal_check();
}

template void run_mskf<double>(MSKFKernel<double>, const Range2D&);
template void run_mskf<float>(MSKFKernel<float>, const Range2D&);

} // namespace cppwrf::phys::cu_mskf
