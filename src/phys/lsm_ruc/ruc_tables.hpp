// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// RUCLSM_SOILVEGPARM (WRF v4.8.0 phys/module_sf_ruclsm.F): VEGPARM.TBL section USGS-RUC (MMINLU USGS) or MODI-RUC
// (MODIS, MODIFIED_IGBP_MODIS_NOAH), SOILPARM.TBL section STAS-RUC (the first two sections of the file, as WRF reads
// them), GENPARM.TBL (must exist; RUC uses none of its values); and LSMRUC's TBQ table.
#include <string>

#include "phys/lsm_ruc/ruc.hpp"

namespace cppwrf::phys::ruc {

// "" on success, else WRF's error message
std::string read_ruc_tables(const std::string& table_dir, const std::string& mminlu, RucTables& t);

}  // namespace cppwrf::phys::ruc
