// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Shared by the PhysicsSuite translation units (suite.cpp: construction, part1 / part2 and the carried
// state; surface_driver.cpp, pbl_driver.cpp, cumulus_driver.cpp: WRF's module_surface_driver,
// module_pbl_driver and module_cumulus_driver).

#include <cstddef>

#include "core/field.hpp"
#include "phys/lsm_noah/noah.hpp"

namespace cppwrf::phys::driver::suite_detail {

using V2 = View2<real>;
using C2 = View2<const real>;
using V3 = View3<real>;
using C3 = View3<const real>;
using I2 = View2<int>;
using CI2 = View2<const int>;

constexpr int kMaxSoil = noah::kMaxSoilLevels;

// cumulus_driver.cpp: the bytes of column work arrays one column of the cumulus scheme keeps in the GPU's work
// buffer (core/column_work.hpp; 0: none), for the reserve at set-up
std::size_t cumulus_work_bytes(int cu_physics, int nz);
// pbl_driver.cpp: the same for the gravity wave drag (gwd_opt = 1)
std::size_t gwdo_work_bytes();

inline C2 slice0(const Field3& f) { return C2{f.data(), int(f.layout().sj)}; }  // level k = 0 (WRF kts)

}  // namespace cppwrf::phys::driver::suite_detail
