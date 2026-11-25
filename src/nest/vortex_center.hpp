// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// ============================================================================
// CPPWRF vortex-following nests: the numerics of WRF 4.8.0's time_for_move2 when wrf.exe is built with vortex
// following (configure nesting 3: -DMOVE_NESTS -DVORTEX_CENTER; share/mediation_nest_move.F:349-700) and of
// compute_500mb_height / interp_p (share/module_compute_geop.F), in WRF's REAL (cppwrf::real: real*8 in the -r8
// build, real*4 in the real*4 build), statement by statement:
//   heights      compute_500mb_height: the geopotential height (m) of the pressure track_level (Pa) per mass column;
//   search       the alarm branch: the first-guess centre (the lowest height of the search box: the whole domain the
//                first time, else max_vortex_speed * vortex_interval around the last centre), the smoothed centre
//                (centroid of vmax - height over 100 km around the first guess), xi / xj, vc_i / vc_j, and the
//                ATCF line's minimum sea-level-adjusted surface pressure;
//   decide       the move after the alarm branch: the displacement of vc from the domain centre, limited to 6
//                points, in parent cells limited to one, and vc_i / vc_j shifted by the move.
// Every reduction (min, max, sums) runs over WRF's whole-domain copy (wrf_patch_to_global_real) on the monitor task
// in WRF's loop order (j outer, i inner); a nest that moves is not decomposed in CPPWRF (one patch), so the loops
// below run on the host over the domain's points in that order. Ties: the first point in that order (strict < / >).
// Arrays: the domain's mass points, 0-based (i - 1) + (j - 1) * nx for WRF's 1-based (i, j), i = 1 .. ide - 1.
// ============================================================================

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/backend.hpp"
#include "core/precision.hpp"

namespace cppwrf::nest::vortex {

// interp_p (module_compute_geop.F:62-104): a(p_loc) in a column of nk mass levels (a, p 0-based: WRF's ks .. ke =
// 1 .. nk); a column whose lowest pressure is below p_loc gets 9.81e5 (cys_change). pk is an INTEGER in WRF: the
// pressure is truncated before the comparison. Returns false for WRF's fatal "interp too high".
template <class R>
CPPWRF_HD inline bool interp_p(const R* a, const R* p, R p_loc, int nk, R& a_interp) {
  const int ks = 1, ke = nk;  // WRF's 1-based bounds
  auto P = [&](int k) { return p[k - 1]; };
  if (P(ks) < p_loc) {
    a_interp = R(9.81e5);
  } else {
    int kp = ks + 1;
    int pk = static_cast<int>(P(kp));  // INTEGER = REAL: truncation
    const R pressure = p_loc;
    while (R(pk) > pressure) {  // pk .gt. pressure: the INTEGER converted to REAL
      kp = kp + 1;
      if (kp > ke) return false;
      pk = static_cast<int>(P(kp));
    }
    const R dp = P(kp - 1) - P(kp);
    const R wght2 = (P(kp - 1) - pressure) / dp;
    const R wght1 = R(1.) - wght2;
    a_interp = wght1 * a[kp - 2] + wght2 * a[kp - 1];
  }
  return true;
}

// compute_500mb_height (module_compute_geop.F:6-58) for one column: pressure(k) = p + pb, geopotential(k) the mean
// of the full levels k and k + 1 (WRF's order of the four terms), interp_p at track_level, / 9.81. ph / phb: nk + 1
// full levels; p / pb: nk mass levels (0-based). False: "interp too high".
template <class R>
CPPWRF_HD inline bool height_column(const R* ph, const R* phb, const R* p, const R* pb, int nk, int track_level,
                                    R* pressure, R* geopotential, R& height) {
  const R track_level_p = R(track_level);  // float(track_level): exact for |track_level| < 2^24
  for (int k = 0; k < nk; ++k) {
    pressure[k] = p[k] + pb[k];
    geopotential[k] = R(0.5) * (ph[k] + phb[k] + ph[k + 1] + phb[k + 1]);
  }
  R interp_output = R(0);
  if (!interp_p(geopotential, pressure, track_level_p, nk, interp_output)) return false;
  height = interp_output / R(9.81);  // 500 mb height in meters
  return true;
}

// the nest's vortex-centre state (Registry state xi, xj, vc_i, vc_j; module_domain.F:798-801 sets -1)
struct Centre {
  real xi = real(-1.0), xj = real(-1.0), vc_i = real(-1.0), vc_j = real(-1.0);
};

// What the alarm branch found (for the ATCF line and the messages)
struct Found {
  int imploc = -1, jmploc = -1;  // the minimum of psfc + 11.38 * terrain
  real pmin = real(0);
  int imloc = -1, jmloc = -1;  // the first guess (lowest height)
  int is = 0, ie = 0, js = 0, je = 0;  // the smoothing window
  bool centred = false;  // pbar /= 0: xi, xj, vc_i, vc_j set
  int i_atcf = 0, j_atcf = 0;  // INT(xi + .5), INT(xj + .5)
};

// The alarm branch of time_for_move2 (share/mediation_nest_move.F:471-603). height, psfc, terrain: the domain's mass
// points (nx = ide - 1, ny = jde - 1, see above); psfc is modified in the search box as WRF modifies its global copy;
// dx: nl_get_dx of the nest (REAL). Throws WRF's fatal messages.
inline Found search(const std::vector<real>& height, std::vector<real>& psfc, const std::vector<real>& terrain,
                    int nx, int ny, real dx, int vortex_interval, int max_vortex_speed, Centre& c) {
  const int ids = 1, ide = nx + 1, jds = 1, jde = ny + 1;
  auto H = [&](int i, int j) { return height[std::size_t(j - 1) * std::size_t(nx) + std::size_t(i - 1)]; };
  auto PS = [&](int i, int j) -> real& { return psfc[std::size_t(j - 1) * std::size_t(nx) + std::size_t(i - 1)]; };
  auto TER = [&](int i, int j) { return terrain[std::size_t(j - 1) * std::size_t(nx) + std::size_t(i - 1)]; };
  Found f;
  int is, ie, js, je;
  if (c.vc_i < real(0.) && c.vc_j < real(0.)) {
    // first time through
    is = ids;
    ie = ide - 1;
    js = jds;
    je = jde - 1;
  } else {
    // limit the search to an area around the vortex that is limited by max_vortex_speed (meters per second) from the
    // previous location over vortex_interval (minutes): 60 * vortex_interval * max_vortex_speed is an INTEGER product
    const real r = real(60 * vortex_interval * max_vortex_speed) / dx;
    is = static_cast<int>(std::max(c.vc_i - r, real(1.0) * real(ids)));
    js = static_cast<int>(std::max(c.vc_j - r, real(1.0) * real(jds)));
    ie = static_cast<int>(std::min(c.vc_i + r, real(1.0) * real(ide - 1)));
    je = static_cast<int>(std::min(c.vc_j + r, real(1.0) * real(jde - 1)));
  }

  // find minimum psfc, adjusted approximately to sea level pressure (ATCF)
  f.imploc = -1;
  f.jmploc = -1;
  real pmin = real(99999999.0);  // make this very large to be sure we find a minumum
  for (int j = js; j <= je; ++j)
    for (int i = is; i <= ie; ++i) {
      PS(i, j) = PS(i, j) + real(11.38) * TER(i, j);
      if (PS(i, j) < pmin) {
        pmin = PS(i, j);
        f.imploc = i;
        f.jmploc = j;
      }
    }
  f.pmin = pmin;
  // wrf_message of the search box, then wrf_error_fatal
  auto fatal = [&](const char* names, const std::string& found, const char* msg) {
    std::fprintf(stderr, " i,j,is,ie,js,je,%s %d %d %d %d %s\n", names, is, ie, js, je, found.c_str());
    return std::runtime_error(std::string("time_for_move2: Method failure searching ") + msg);
  };
  if (f.imploc == -1 || f.jmploc == -1)
    throw fatal("imploc,jmploc", std::to_string(f.imploc) + " " + std::to_string(f.jmploc), "for minimum psfc.");

  int imloc = -1, jmloc = -1, maxi = -1, maxj = -1;
  // find local min, max
  real vmin = real(99999999.0);
  real vmax = real(-99999999.0);
  for (int j = js; j <= je; ++j)
    for (int i = is; i <= ie; ++i) {
      if (H(i, j) < vmin) {
        vmin = H(i, j);
        imloc = i;
        jmloc = j;
      }
      if (H(i, j) > vmax) {
        vmax = H(i, j);
        maxi = i;
        maxj = j;
      }
    }
  if (imloc == -1 || jmloc == -1 || maxi == -1 || maxj == -1)
    throw fatal("imloc,jmloc,maxi,maxj",
                std::to_string(imloc) + " " + std::to_string(jmloc) + " " + std::to_string(maxi) + " " +
                    std::to_string(maxj),
                "max/min of height.");
  f.imloc = imloc;
  f.jmloc = jmloc;

  const int fimloc = imloc, fjmloc = jmloc;  // INTEGERs in WRF
  if (c.xi == real(-1.)) c.xi = real(fimloc);
  if (c.xj == real(-1.)) c.xj = real(fjmloc);

  const real rsmooth = real(100000.);  // in meters
  const real dijsmooth = rsmooth / dx;
  const int fjs = static_cast<int>(std::max(real(fjmloc) - dijsmooth, real(1.0)));
  const int fje = static_cast<int>(std::min(real(fjmloc) + dijsmooth, real(jde) - real(2.0)));
  const int fis = static_cast<int>(std::max(real(fimloc) - dijsmooth, real(1.0)));
  const int fie = static_cast<int>(std::min(real(fimloc) + dijsmooth, real(ide) - real(2.0)));
  js = fjs;
  je = fje;
  is = fis;
  ie = fie;
  f.is = is;
  f.ie = ie;
  f.js = js;
  f.je = je;

  vmin = real(1000000.0);
  vmax = real(-1000000.0);
  for (int j = js; j <= je; ++j)
    for (int i = is; i <= ie; ++i)
      if (H(i, j) > vmax) vmax = H(i, j);

  real pbar = real(0.0), ipbar = real(0.0), jpbar = real(0.0);
  for (int j = js; j <= je; ++j)
    for (int i = is; i <= ie; ++i) {
      const real fact = vmax - H(i, j);
      pbar = pbar + fact;
      ipbar = ipbar + fact * real(i - is);
      jpbar = jpbar + fact * real(j - js);
    }

  if (pbar != real(0.)) {
    // an adjusted, smoothed vortex centre in cross point index space; time average with a (old) = 0, b (new) = 1
    const real a = real(0.0), b = real(1.0);
    c.xi = (a * c.xi + b * (real(is) + ipbar / pbar)) / (a + b);
    c.xj = (a * c.xj + b * (real(js) + jpbar / pbar)) / (a + b);
    c.vc_i = c.xi + real(.5);
    c.vc_j = c.xj + real(.5);
    f.centred = true;
  }
  f.i_atcf = static_cast<int>(c.xi + real(.5));
  f.j_atcf = static_cast<int>(c.xj + real(.5));
  return f;
}

// time_for_move2 after the alarm branch (share/mediation_nest_move.F:653-697): ide, jde = e_we, e_sn of the nest,
// pgr its parent_grid_ratio. Returns move_cd_x, move_cd_y in parent cells (-1, 0, 1); vc_i / vc_j are shifted by the
// move. disp / dc: the values WRF prints.
struct Decision {
  real dc_i = 0, dc_j = 0, disp_x = 0, disp_y = 0;
  int move_cd_x = 0, move_cd_y = 0;
};
inline Decision decide(int ide, int jde, int pgr, Centre& c) {
  const int ids = 1, jds = 1;
  Decision d;
  d.dc_i = real(ide - ids + 1) / real(2.);  // domain center
  d.dc_j = real(jde - jds + 1) / real(2.);
  real disp_x = c.vc_i - d.dc_i * real(1.0);
  real disp_y = c.vc_j - d.dc_j * real(1.0);
  const int irad = 6;  // vortex_search_radius, hard coded (JM 20050721)
  const real radius = real(irad);
  if (disp_x > real(0)) disp_x = std::min(disp_x, radius);
  if (disp_y > real(0)) disp_y = std::min(disp_y, radius);
  if (disp_x < real(0)) disp_x = std::max(disp_x, -radius);
  if (disp_y < real(0)) disp_y = std::max(disp_y, -radius);
  d.disp_x = disp_x;
  d.disp_y = disp_y;
  int move_cd_x = static_cast<int>(disp_x / real(pgr));
  int move_cd_y = static_cast<int>(disp_y / real(pgr));
  if (move_cd_x > 0) move_cd_x = std::min(move_cd_x, 1);
  if (move_cd_y > 0) move_cd_y = std::min(move_cd_y, 1);
  if (move_cd_x < 0) move_cd_x = std::max(move_cd_x, -1);
  if (move_cd_y < 0) move_cd_y = std::max(move_cd_y, -1);
  d.move_cd_x = move_cd_x;
  d.move_cd_y = move_cd_y;
  c.vc_i = c.vc_i - real(move_cd_x * pgr);
  c.vc_j = c.vc_j - real(move_cd_y * pgr);
  return d;
}

}  // namespace cppwrf::nest::vortex
