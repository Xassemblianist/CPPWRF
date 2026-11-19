// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "phys/ra_rrtmg_common/rrtmg_real.hpp"
// WRF phys/module_ra_clWRF_support.F read_CAMgases (ghg_input = 1): yearly trace-gas table
// CAMtr_volume_mixing_ratio (read once), interpolated in time (year, julian day) at every call.
#include <string>
#include <vector>

namespace cppwrf::phys {

struct CamGases {
  std::vector<int> yr;
  // juldata is REAL, co2r .. cfc12r and the interpolated values REAL(r8), r8 = 8 (double in WRF real*4 too)
  std::vector<rr> jul;
  std::vector<double> co2, n2o, ch4, cfc11, cfc12;
  // read the table (WRF reads it in phy_init); throws if missing
  void load(const std::string& path);
  // read_CAMgases(yr, julian, .false., "RRTMG", ...): volume mixing ratios
  void interpolate(int year, rr julian, double& co2v, double& n2ov, double& ch4v, double& cfc11v,
                   double& cfc12v) const;
};

}  // namespace cppwrf::phys
