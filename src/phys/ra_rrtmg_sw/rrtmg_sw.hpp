// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/kissvec.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_tables.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_cldprmc.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_setcoef.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_reftra.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_vrtqdr.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_taumol.hpp"
#include "phys/ra_rrtmg_sw/rrtmg_sw_stages.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cppwrf::phys {

// Per-thread column workspace: the stage views of rrtmg_sw_stages.hpp (stride 1); cldfmc holds the McICA
// subcolumn flags rrtmg_sw_core takes ([layer][g-point])
struct RrtmgSwWorkspace {
  rb cldfmc[RRTMG_SW_COL_MAX * 112];
  std::vector<rb> d;
  std::vector<int> i;
};

struct RrtmgSwConfig {
  int icloud{1};
  int cldovrlp{2};
  int idcor{0};
  rb p_top{rb(5000.0)};
  rb yr{rb(2000.0)};
  rb julian{rb(1.0)};
  rb solcon{rb(1367.0)};
  rb g{rb(9.81)};
  rb r_d{rb(287.0)};
  rb cp{rb(1004.5)};
  int o3input{0};                     // 2: o33d on model levels
  rb co2{0}, n2o{0}, ch4{0};      // trace gases (vmr), as for RrtmgLwConfig
  // rrsw_con heatfac = grav * secdy / (cpdair * 1.e2_rb) (rrtmg_sw_ini), checked against WRF; rb arithmetic (float in
  // the FP32 builds, as WRF real*4 computes it)
  rb heatfac{sizeof(rb) == 4 ? rb(9.8066f * 86400.0f / (1004.5f * 100.0f)) : rb(8.4349451468392243)};
};

RrtmgSwCallConfig rrtmg_sw_call_config(int num_levels, const RrtmgSwConfig& config);

void rrtmg_sw_core(int nlayers, rb heatfac, const rb* play, const rb* plev, const rb* tlay,
                   const rb* tlev, const rb* h2ovmr, const rb* o3vmr, const rb* co2vmr,
                   const rb* ch4vmr, const rb* n2ovmr, const rb* o2vmr, rb asdir, rb asdif,
                   rb aldir, rb aldif, rb coszen, rb adjes, rb scon, const rb* reicmcl,
                   const rb* relqmcl, const rb* resnmcl, rb* swuflx, rb* swdflx, rb* swhr,
                   rb* swuflxc, rb* swdflxc, rb* swhrc, rb* swdkdir, rb* swdkdif,
                   rb* sibvisdir, rb* sibvisdif, rb* sibnirdir, rb* sibnirdif, RrtmgSwWorkspace& ws,
                   const RrtmgSwTables& tables, int inflag = 2, int iceflag = 3, int liqflag = 1);

// Single column execution
void rrtmg_sw_column(
    int num_levels, // model nz (e.g. 50)
    const RrtmgSwConfig& config,
    const RrtmgSwTables& tables,
    // 2D surface / solar variables for this column
    rb coszen,
    rb albedo,
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
    // 2D outputs (gsw, swcf, the TOA / surface fluxes, the surface direct / diffuse components)
    SwColumnOut& out,
    // 3D outputs
    rb* rthratensw,  // [nz] K/s
    rb* rthratenswc, // [nz] K/s
    // Workspace
    RrtmgSwWorkspace& ws,
    // effective radii from the microphysics [nz] (m): WRF has_reqc / has_reqi / has_reqs != 0
    const rb* re_cloud = nullptr,
    const rb* re_ice = nullptr,
    const rb* re_snow = nullptr
);

} // namespace cppwrf::phys
