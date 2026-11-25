// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// lakeini (phys/module_sf_lake.F:4886-5397) on the host for a set of points (a patch, row by row as WRF's tile):
// the -999 fill of the non-lake points, the lake points of LAKEFLAG 0 (water with HGT >= lake_min_elev) or 1
// (LAKEMASK = 1, real.exe's ISLAKE category), WRF's changes of IVGTYP / XLAND / XICE at frozen lake points, the depth
// choice (LAKE_DEPTH, lakedepth_default or the 1 m spval column) and lakeini_point. Not called on restarts (WRF
// returns at once).
#include <stdexcept>
#include <string>
#include <vector>

#include "phys/sf_lake/lake.hpp"

namespace cppwrf::phys::lake {

struct LakeIniOptions {
  real lake_min_elev = real(5.), lakedepth_default = real(50.), xice_threshold = real(0.5);
  int lakeflag = 1, use_lakedepth = 1, iswater = 17;
  int lake_depth_flag = 0;  // INOUT: set to 0 when use_lakedepth = 0 and a lake point exists
};

// One point's inputs and the arrays lakeini writes (p: every lake array of the point)
struct LakeIniIn {
  real ht, snow, tsk, lake_depth;
  int isltyp;
};

// pts: the points of the patch in WRF's loop order (j outer, i inner; the order matters only for the fatal stop);
// ivgtyp, xland, xice, lakemask: the points' values, changed as WRF changes them
inline void lakeini_points(LakeIniOptions& o, const std::vector<LakeIniIn>& pts, std::vector<int>& ivgtyp,
                           std::vector<real>& xland, std::vector<real>& xice, std::vector<real>& lakemask,
                           std::vector<LakeInitPoint>& out) {
  const std::size_t n = pts.size();
  out.assign(n, LakeInitPoint{});
  std::vector<char> lake(n, 0);
  const real defval = real(-999.0);
  for (std::size_t p = 0; p < n; ++p) {
    LakeInitPoint& q = out[p];
    q.snowdp2d = pts[p].snow * real(0.005);  // 4987
    q.h2osno2d = pts[p].snow;
    q.lakedepth2d = defval;
    q.snl2d = defval;
    for (int k = 0; k < 15; ++k) q.h2osoi_liq[k] = q.h2osoi_ice[k] = q.t_soisno[k] = q.z[k] = q.dz[k] = defval;
    for (int k = 1; k <= nlevlake; ++k) q.t_lake[k] = q.lake_icefrac[k] = q.z_lake[k] = q.dz_lake[k] = defval;
  }
  for (std::size_t p = 0; p < n; ++p) {  // 5017-5070 (the i / j order does not matter here)
    if (o.lakeflag == 0) {
      if (pts[p].ht >= o.lake_min_elev) {
        if (xice[p] > o.xice_threshold) {
          ivgtyp[p] = o.iswater;
          xland[p] = real(2.);
          out[p].lake_icefrac[1] = xice[p];
          xice[p] = real(0.0);
        }
      }
      if (ivgtyp[p] == o.iswater && pts[p].ht >= o.lake_min_elev) {
        lake[p] = 1;
        lakemask[p] = real(1);
      } else {
        lake[p] = 0;
        lakemask[p] = real(0);
      }
    } else {
      if (lakemask[p] == real(1)) {
        lake[p] = 1;
        if (xice[p] > o.xice_threshold) {
          ivgtyp[p] = o.iswater;
          xland[p] = real(2.);
          out[p].lake_icefrac[1] = xice[p];
          xice[p] = real(0.0);
        }
      } else {
        lake[p] = 0;
      }
    }
  }
  for (std::size_t p = 0; p < n; ++p) {  // 5076-5121
    if (!lake[p]) continue;
    LakeInitPoint& q = out[p];
    if (o.use_lakedepth == 1 && o.lake_depth_flag == 0)
      throw std::runtime_error("STOP: You need lake-depth information. Rerun WPS or set use_lakedepth = 0");
    if (o.use_lakedepth == 0 && o.lake_depth_flag == 1) o.lake_depth_flag = 0;
    if (o.lake_depth_flag == 1) {
      if (pts[p].lake_depth > real(0.0)) {
        q.lakedepth2d = pts[p].lake_depth;
      } else {
        q.lakedepth2d = o.lakedepth_default > real(0.0) ? o.lakedepth_default : real(spval);
      }
    } else {
      q.lakedepth2d = o.lakedepth_default > real(0.0) ? o.lakedepth_default : real(spval);
    }
  }
  const LakeGrid g = lake_grid();
  for (std::size_t p = 0; p < n; ++p) {  // 5221-5395
    if (!lake[p]) continue;
    if (pts[p].isltyp < 1 || pts[p].isltyp > 19)
      throw std::runtime_error("lakeini: ISLTYP = " + std::to_string(pts[p].isltyp) +
                               " at a lake point: outside WRF's 19 sand / clay categories");
    lakeini_point(g, pts[p].tsk, pts[p].isltyp, out[p]);
  }
}

}  // namespace cppwrf::phys::lake
