// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// urban_param_init (WRF v4.8.0 phys/module_sf_urban.F:2941-3519): URBPARM.TBL (use_wudapt_lcz = 0) or
// URBPARM_LCZ.TBL (use_wudapt_lcz = 1) read as WRF reads it, and the derived canyon parameters (Macdonald 1998
// roughness and displacement, sky view factor). Throws with WRF's message where WRF stops, and for the table
// options whose branches of SUBROUTINE urban are not ported (urban.hpp).
#include <string>

#include "phys/sf_urban/urban.hpp"

namespace cppwrf::phys::urban {

struct UrbanTableOptions {  // the scalar switches of the table (module variables of module_sf_urban.F)
  int boundr = 0, boundb = 0, boundg = 0, ch_scheme = 0, ts_scheme = 0, ahoption = 0, alhoption = 0, imp_scheme = 0,
      iri_scheme = 0, groption = 0, treeoption = 0, ttscheme = 0;
  double oasis = 0, fgr = 0;
  bool has_boundr = false, has_boundb = false, has_boundg = false, has_ch = false, has_ts = false, has_imp = false;
};

// path: the table file; num_soil_layers: the number of roof / wall / road layers (DDZR, DDZB, DDZG values)
// sf_urban_physics: 1 (Z0R from SIGMA_ZED). Fills t (DZR / DZB / DZG in cm) and opt.
void read_urban_table(const std::string& path, int num_soil_layers, int sf_urban_physics, UrbanTables& t,
                      UrbanTableOptions& opt);
// The table options the port supports (urban.hpp); throws naming the entry otherwise
void check_urban_table_options(const UrbanTableOptions& opt, const std::string& path);

}  // namespace cppwrf::phys::urban
