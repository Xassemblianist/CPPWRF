// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Observation nudging, obs_nudge_opt = 1 (statement ports, WRF v4.8.0):
//   ObsNudging::init      phys/module_fddaobs_rtfdda.F fddaobs_init (phy_init's fdob_init)
//   ObsNudging::in4dob    share/wrf_fddaobs_in.F wrf_fddaobs_in / in4dob (med_before_solve_io, every step): the
//                         OBS_DOMAIN<id>01 reader, its time window and duplicate rules, share/module_llxy.F (Lambert)
//   PhysicsSuite::fddaobs_driver  phys/module_fddaobs_driver.F (first_rk_step_part2 after update_phy_ten): errob
//                         (the innovations at the observations), nudob (their weights and the tendencies)
// The observation arrays (WRF grid%fdob) are on the host (in4dob changes them); errob and nudob run as kernels on
// copies of them: one thread per observation (errob, which reads only its own observation) and one per column and
// variable (nudob, which adds the observations of a column in WRF's order n = 1 ... nstat).
#include <array>
#include <fstream>
#include <string>
#include <vector>

#include "core/backend.hpp"
#include "core/field.hpp"
#include "core/grid.hpp"

namespace cppwrf::phys::driver {

// &fdda obs_* options of this domain (WRF Registry defaults; max_domains entries are this domain's, the others
// domain 1's)
struct ObsNudgeOptions {
  int obs_nudge_opt = 0;
  int nudge_wind = 0, nudge_temp = 0, nudge_mois = 0, nudge_pstr = 0;  // per domain
  double coef_wind = 0, coef_temp = 0, coef_mois = 0, coef_pstr = 0;   // per domain
  double rinxy = 0, twindo = 0;                                         // per domain (km, hours)
  double twindo_cg = 0;            // domain 1's obs_twindo (a nest's window when its own is 0)
  int ionf = 1;                    // per domain: the steps (of domain 1) between the error computations
  int prt_freq = 1000;             // per domain (collect_obs_info stops for <= 0)
  int no_pbl_nudge_uv = 0, no_pbl_nudge_t = 0, no_pbl_nudge_q = 0;  // per domain
  double fdda_end = 0;             // per domain (minutes); fdda_start has no effect in WRF 4.8
  int idynin = 0;
  double dtramp = 0;               // minutes
  int sfc_scheme_horiz = 0, sfc_scheme_vert = 0;
  double max_sndng_gap = 20;       // cb
  double sfcfact = 1.0, sfcfacr = 1.0, dpsmx = 7.5;
  std::array<double, 6> vif_uv{50, 50, 50, 50, -5000, 50};  // obs_nudgezfullr1_uv, rampr1, fullr2, rampr2, fullr4, rampr4
  std::array<double, 6> vif_t{50, 50, 50, 50, -5000, 50};
  std::array<double, 6> vif_q{50, 50, 50, 50, -5000, 50};
  double nudgezfullmin = 50, nudgezrampmin = 50, nudgezmax = 3000;
  int scl_neg_qv_innov = 0;
  int max_obs = 0;                 // NIOBF
  int grid_id = 1;                 // INEST
  int levidn = 0;                  // the domain's nest level (0: domain 1; fddaobs_init's levidn(inest))
  int parent_grid_ratio = 1;       // grid%parent_grid_ratio (the domain's ratio to its parent)
  std::string file;                // OBS_DOMAIN<id>01 in the case directory
  bool diff_z = false;             // diff_opt 1 or 2: grid%z is compute_diff_metrics' at the driver call
};

// the map projection of set_projection (share/module_llxy.F proj_info, PROJ_LC)
struct ObsProj {
  real truelat1 = 0, truelat2 = 0, stdlon = 0, lat1 = 0, lon1 = 0, knowni = 0, knownj = 0, dx = 0;
  real hemi = 0, rebydx = 0, cone = 0, rsw = 0, polei = 0, polej = 0;
};

// the device copies of the observations (n = 0 ... cap - 1; varobs(kn, n) at kn + 6 n, errf(kn, n) at kn + 10 n)
struct ObsDevArrays {
  real *varobs = nullptr, *errf = nullptr, *timeob = nullptr, *nlevs_ob = nullptr, *lev_in_ob = nullptr,
       *rio = nullptr, *rjo = nullptr, *rko = nullptr;
};

struct ObsNudging {
  static constexpr int kNdgv = 6, kErrf = 10;  // nobs_ndg_vars, nobs_err_flds (Registry derived)
  ObsNudging(const Grid& g, const ObsNudgeOptions& o);
  ~ObsNudging();
  ObsNudging(const ObsNudging&) = delete;
  ObsNudging& operator=(const ObsNudging&) = delete;

  // fddaobs_init: ktaur (itimestep of a resumed run, else 0), the run's start date (namelist start_*, YYYY-MM-DD_hh:mm:ss),
  // the base state (p00, t00, tlp, znu, p_top), XLAT(1,1), XLONG(1,1) and the projection (wrfinput attributes)
  void init(bool restart_in, long ktaur_in, const std::string& start_date, double p00, double t00, double tlp,
            const std::vector<double>& znu, double p_top, double xlat11, double xlong11, int map_proj,
            double truelat1, double truelat2, double stand_lon);
  // wrf_fddaobs_in at the start of the step after `ktau` steps (grid%itimestep), xtime = grid%xtime (minutes)
  void in4dob(long ktau, real xtime);
  // the wrfinput values init needs (init_real_physics); start: init at the first step of the run (DomainRunner)
  struct Inputs {
    double p00 = 0, t00 = 0, tlp = 0, p_top = 0, xlat11 = 0, xlong11 = 0, truelat1 = 0, truelat2 = 0, stand_lon = 0;
    std::vector<double> znu;
    int map_proj = 0;
  } in;
  void start(bool restart_in, long ktaur_in, const std::string& start_date) {
    init(restart_in, ktaur_in, start_date, in.p00, in.t00, in.tlp, in.znu, in.p_top, in.xlat11, in.xlong11,
         in.map_proj, in.truelat1, in.truelat2, in.stand_lon);
  }
  void latlon_to_ij(real lat, real lon, real& i, real& j) const;
  void rotate_vector(real lon, real& u, real& v) const;
  void upload(int nsta);    // the host arrays of the first nsta observations to the device
  void download(int nsta);  // errob's varobs, errf and rko back to the host

  Grid g_;
  ObsNudgeOptions o_;
  ObsProj proj_;
  bool initialised = false;
  bool restart = false;  // config_flags%restart (a run resumed from a WRF restart file)
  long ktaur = 0;
  real tback_ = 0;       // in4dob's TBACK of the current call
  std::string sdate;
  real xtime_at_rest = 0, rtlast = 0, datend = 0, tfaci = 1, window = 0;
  int ieodi = 0, ieof = 0, ifon = 0, nstat = 0, nstaw = 0, iwtsig = 0;
  real known_lat = 0, known_lon = 0, dcon = 0, dpsmx = 0, pfree = 0, rinfmn = 0, rinfmx = 0;
  std::array<real, 6> vif_uv{}, vif_t{}, vif_q{};
  real vif_max = 0, vif_fullmin = 0, vif_rampmin = 0;
  bool nudge_uv_pbl = true, nudge_t_pbl = true, nudge_q_pbl = true;
  std::vector<real> base_state;
  // grid%fdob arrays (NIOBF = max_obs entries)
  std::vector<real> varobs, errf, timeob, nlevs_ob, lev_in_ob, plfo, elevob, rio, rjo, rko;
  std::ifstream file;
  bool file_open = false;
  // device side
  ObsDevArrays d;
  int dcap = 0;
  Field2 vih[6];      // compute_VIH: vih_uv(1:2), vih_t(1:2), vih_q(1:2)
  Field3 zdiff;       // compute_diff_metrics' z (diff_opt 1, 2)
  Field3 savwt[4];    // grid%obs_savwt(ipl = 1 ... 4): U, V (at their points), T, QV
};

}  // namespace cppwrf::phys::driver
