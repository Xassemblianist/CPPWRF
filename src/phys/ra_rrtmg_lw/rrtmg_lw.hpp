// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/kissvec.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_cldprmc.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_rtrnmc.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_setcoef.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_tables.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_taumol.hpp"
#include "phys/ra_rrtmg_lw/rrtmg_lw_stages.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cppwrf::phys {

constexpr int RRTMG_LW_MAX_LAYERS = 128;

// Per-thread column workspace: the stride-1 storage of the stage views (rrtmg_lw_stages.hpp) of the
// column driver, and the McICA cloud flags / optical depths that rrtmg_lw_core takes as input
struct RrtmgLwWorkspace {
  rb cldfmc[140][RRTMG_LW_MAX_LAYERS];
  rb taucmc[140][RRTMG_LW_MAX_LAYERS];
  std::vector<rb> d;
  std::vector<int> i;
};

struct RrtmgLwConfig {
  int icloud{1};
  int cldovrlp{2};
  int idcor{0};
  int mp_physics{0};
  rb p_top{rb(5000.0)};
  rb yr{rb(2000.0)};
  rb julian{rb(1.0)};
  rb g{rb(9.81)};
  rb r_d{rb(287.0)};
  rb cp{rb(1004.5)};
  int o3input{0};  // 2: o33d (CAM climatology from the radiation driver) on model levels
  // trace gases (volume mixing ratios): ghg_input = 1 -> read_CAMgases, else rrtmg_default_gases
  rb co2{0}, n2o{0}, ch4{0}, cfc11{0}, cfc12{0};
};

// call configuration of the stages (nlayers, heatfac, gases) and the RRTMG_LWRAD constants
RrtmgLwCallConfig rrtmg_lw_call_config(int num_levels, const RrtmgLwConfig& config, bool hrc, bool hri, bool hrs);
const RrtmgLwConsts& rrtmg_lw_consts();

// RRTMG_LWRAD trace gases without ghg_input (WRF's fixed values, CO2 from the year)
void rrtmg_default_gases(RrtmgLwConfig& c);

// WRF rrtmg_lw (inatm, cldprmc, setcoef, taumol, rtrnmc) for one column whose McICA subcolumn
// cloud arrays are already in ws (cldfmc, taucmc). Layer arrays have nlayers
// entries, level arrays nlayers + 1 (index 0 = surface); pressures in mb, emis per band.
void rrtmg_lw_core(int nlayers, rb cp, const rb* play, const rb* plev, const rb* tlay,
                   const rb* tlev, rb tsk, const rb* h2ovmr, const rb* o3vmr, const rb* co2vmr,
                   const rb* ch4vmr, const rb* n2ovmr, const rb* o2vmr, const rb* cfc11vmr,
                   const rb* cfc12vmr, const rb* cfc22vmr, const rb* ccl4vmr, const rb* semiss,
                   const rb* rei, const rb* rel, const rb* res, rb* totuflux, rb* totdflux,
                   rb* htr, rb* totuclfl, rb* totdclfl, rb* htrc, RrtmgLwWorkspace& ws,
                   const RrtmgLwTables& tables, int inflag = 2, int iceflag = 3, int liqflag = 1);

// Single column execution
void rrtmg_lw_column(
    int num_levels, // model nz (e.g. 50)
    const RrtmgLwConfig& config,
    const RrtmgLwTables& tables,
    // 2D surface variables for this column
    rb tsk,
    rb emiss,
    rb xland,
    rb xice,
    rb snow,
    rb xlat,
    // 3D vertical profiles for this column
    const rb* p8w,   // [nz + 1] Pa
    const rb* t8w,   // [nz + 1] K
    const rb* p3d,   // [nz] Pa
    const rb* pi3d,  // [nz]
    const rb* dz8w,  // [nz] m
    const rb* t3d,   // [nz] K
    const rb* rho3d, // [nz] kg/m3
    const rb* qv3d,  // [nz] kg/kg
    const rb* qc3d,  // [nz] kg/kg
    const rb* qr3d,  // [nz] kg/kg
    const rb* qi3d,  // [nz] kg/kg
    const rb* qs3d,  // [nz] kg/kg
    const rb* qg3d,  // [nz] kg/kg
    const rb* cldfra3d, // [nz]
    const rb* o33d,     // [nz] ozone vmr (o3input = 2), else unused
    // 2D outputs
    rb& glw,
    rb& olr,
    rb& lwcf,
    rb& lwupt,
    rb& lwuptc,
    rb& lwdnt,
    rb& lwdntc,
    rb& lwupb,
    rb& lwupbc,
    rb& lwdnb,
    rb& lwdnbc,
    // 3D outputs
    rb* rthratenlw,  // [nz] K/s
    rb* rthratenlwc, // [nz] K/s
    // Workspace
    RrtmgLwWorkspace& ws,
    // effective radii from the microphysics [nz] (m): WRF has_reqc / has_reqi / has_reqs != 0
    // when given (nullptr: CAM relcalc / reicalc sizes, no separate snow)
    const rb* re_cloud = nullptr,
    const rb* re_ice = nullptr,
    const rb* re_snow = nullptr,
    // [nz] o3input != 2: the climatology ozone vmr on the model levels, WRF's O33D output (nullptr: not wanted)
    rb* o3_out = nullptr
);

} // namespace cppwrf::phys
