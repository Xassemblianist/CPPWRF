// SPDX-License-Identifier: GPL-3.0-or-later
// Grid (analysis) nudging, grid_fdda = 1 (statement ports, WRF v4.8.0, C++ k = WRF k - 1):
//   fddagd_driver   phys/module_fddagd_driver.F (first_rk_step_part1, after the cumulus driver)
//   fddagd          phys/module_fdda_psufddagd.F (PSUFDDAGD: tendencies toward the analyses of wrffdda_d0N,
//                   interpolated in time; the weights of the PBL, of the lowest layers and of the ramp)
// The tendencies are coupled, added and decoupled in tendency_coupling.cpp (calculate_phy_tend, phy_fg_ten,
// phy_prep_part2); the analyses are read when the auxinput10 alarm rings (DomainRunner, read_wrffdda).
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "core/wrf_time.hpp"
#include "phys/driver/suite.hpp"

namespace cppwrf::phys::driver {

namespace {

using V3 = View3<real>;
using C3 = View3<const real>;
using V2 = View2<real>;
using C2 = View2<const real>;
using B2 = BView2<const real>;

// itimestep = 1: the old analyses are the model state (module_fddagd_driver.F:283-296), k = kts..kte
struct KFddaOldFromState {
  V3 u_old, v_old, t_old, ph_old, q_old;
  V2 mu_old;
  C3 u3d, v3d, th_phy, ph, qv;
  CPPWRF_HD void operator()(int i, int j, int k) const {
    u_old(i, j, k) = u3d(i, j, k);
    v_old(i, j, k) = v3d(i, j, k);
    t_old(i, j, k) = th_phy(i, j, k) - real(300.0);
    ph_old(i, j, k) = ph(i, j, k);
    q_old(i, j, k) = qv(i, j, k);
    if (k == 0) mu_old(i, j) = real(0.0);
  }
};

// RTHNDGDTEN = RUNDGDTEN = RVNDGDTEN = RQVNDGDTEN = 0 (the driver's zeroing, fddagd's after the end of the nudging);
// RPHNDGDTEN and RMUNDGDTEN stay 0 with grid_fdda = 1 (not held). The driver also zeroes level kde, which nothing reads.
struct KFddaZero {
  V3 ru, rv, rth, rqv;
  CPPWRF_HD void operator()(int i, int j, int k) const {
    rth(i, j, k) = real(0.);
    ru(i, j, k) = real(0.);
    rv(i, j, k) = real(0.);
    rqv(i, j, k) = real(0.);
  }
};

// fddagd_driver: the regime of the surface layers that do not set it (module_fddagd_driver.F:503-514)
struct KFddaRegime {
  V2 regime;
  C2 pblh;
  C3 z_at_w;
  B2 ht;
  CPPWRF_HD void operator()(int i, int j) const {
    if (pblh(i, j) > z_at_w(i, j, 1) - ht(i, j)) regime(i, j) = real(4.0);
    else regime(i, j) = real(1.0);
  }
};

// The scalars of fddagd for this call (computed on the host as WRF computes them once per tile)
struct FddaScalars {
  real coef, tfac, guv, gt, gq;
  int nz;                                       // ktf = kde - 1 levels
  int no_pbl_uv, no_pbl_t, no_pbl_q;            // if_no_pbl_nudging_* (grid_sfdda = 0)
  int if_zfac_uv, k_zfac_uv, if_zfac_t, k_zfac_t, if_zfac_q, k_zfac_q;
};

// wzfac(k): 0 up to k_zfac, 0.1 at k_zfac + 1, 1 above (k = WRF level, 1-based); 1 without if_zfac
CPPWRF_HD inline real wzfac(int kw, int if_zfac, int k_zfac) {
  real w = real(1.0);
  if (if_zfac == 1) {
    if (kw <= k_zfac) w = real(0.0);
    if (kw == k_zfac + 1) w = real(0.1);
    if (kw > k_zfac + 1) w = real(1.0);
  }
  return w;
}

// wpbl(k) from the PBL level kpbl (WRF level): 0 up to kpbl, 0.1 at kpbl + 1, 1 above
CPPWRF_HD inline real wpbl_of(int kw, int kpbl) {
  real w = real(1.0);
  if (kw <= kpbl) w = real(0.0);
  if (kw == kpbl + 1) w = real(0.1);
  if (kw > kpbl + 1) w = real(1.0);
  return w;
}

// fddagd: RUNDGDTEN at the u points i = itsu..itf (C++ 1..nx-1), all rows (module_fdda_psufddagd.F:351-380, 621-629)
struct KFddagdU {
  FddaScalars c;
  V3 ru;
  C3 u_old, u_new, u3d, z_at_w;
  C2 pblh;
  B2 ht;
  CPPWRF_HD void operator()(int i, int j) const {
    int kpbl = 1;
    if (c.no_pbl_uv == 1) {
      const real zpbl = real(0.5) * (pblh(i - 1, j) + pblh(i, j));
      for (int kw = 1; kw <= c.nz; ++kw) {
        const int k = kw - 1;
        const real zagl_bot = real(0.5) * (z_at_w(i - 1, j, k) - ht(i - 1, j) + z_at_w(i, j, k) - ht(i, j));
        const real zagl_top = real(0.5) * (z_at_w(i - 1, j, k + 1) - ht(i - 1, j) + z_at_w(i, j, k + 1) - ht(i, j));
        if (zpbl >= zagl_bot && zpbl < zagl_top) {
          kpbl = kw;
          break;
        }
      }
    }
    for (int kw = 1; kw <= c.nz; ++kw) {
      const int k = kw - 1;
      const real wpbl = c.no_pbl_uv == 1 ? wpbl_of(kw, kpbl) : real(1.0);
      const real val_analysis = u_old(i, j, k) * (real(1.0) - c.coef) + u_new(i, j, k) * c.coef;
      ru(i, j, k) = ru(i, j, k) + c.guv * wpbl * wzfac(kw, c.if_zfac_uv, c.k_zfac_uv) * c.tfac * (val_analysis - u3d(i, j, k));
    }
  }
};

// fddagd: RVNDGDTEN at the v points j = jtsv..jtf (C++ 1..ny-1), all columns (module_fdda_psufddagd.F:378-401, 631-639)
struct KFddagdV {
  FddaScalars c;
  V3 rv;
  C3 v_old, v_new, v3d, z_at_w;
  C2 pblh;
  B2 ht;
  CPPWRF_HD void operator()(int i, int j) const {
    int kpbl = 1;
    if (c.no_pbl_uv == 1) {
      const real zpbl = real(0.5) * (pblh(i, j - 1) + pblh(i, j));
      for (int kw = 1; kw <= c.nz; ++kw) {
        const int k = kw - 1;
        const real zagl_bot = real(0.5) * (z_at_w(i, j - 1, k) - ht(i, j - 1) + z_at_w(i, j, k) - ht(i, j));
        const real zagl_top = real(0.5) * (z_at_w(i, j - 1, k + 1) - ht(i, j - 1) + z_at_w(i, j, k + 1) - ht(i, j));
        if (zpbl >= zagl_bot && zpbl < zagl_top) {
          kpbl = kw;
          break;
        }
      }
    }
    for (int kw = 1; kw <= c.nz; ++kw) {
      const int k = kw - 1;
      const real wpbl = c.no_pbl_uv == 1 ? wpbl_of(kw, kpbl) : real(1.0);
      const real val_analysis = v_old(i, j, k) * (real(1.0) - c.coef) + v_new(i, j, k) * c.coef;
      rv(i, j, k) = rv(i, j, k) + c.guv * wpbl * wzfac(kw, c.if_zfac_uv, c.k_zfac_uv) * c.tfac * (val_analysis - v3d(i, j, k));
    }
  }
};

// the PBL level of a mass column (the loops loop_kt / loop_kq)
CPPWRF_HD inline int kpbl_mass(real zpbl, const C3& z_at_w, real ht, int i, int j, int nz) {
  int kpbl = 1;
  for (int kw = 1; kw <= nz; ++kw) {
    const int k = kw - 1;
    const real zagl_bot = z_at_w(i, j, k) - ht;
    const real zagl_top = z_at_w(i, j, k + 1) - ht;
    if (zpbl >= zagl_bot && zpbl < zagl_top) {
      kpbl = kw;
      break;
    }
  }
  return kpbl;
}

// fddagd: RTHNDGDTEN, HFX_FDDA and RQVNDGDTEN at the mass points (module_fdda_psufddagd.F:405-461, 641-670)
struct KFddagdTQ {
  FddaScalars c;
  V3 rth, rqv, hfx_fdda;
  C3 t_old, t_new, q_old, q_new, th3d, qv3d, p3d, dz8w, z_at_w;
  C2 pblh;
  B2 ht;
  CPPWRF_HD void operator()(int i, int j) const {
    const int kpbl_t = c.no_pbl_t == 1 ? kpbl_mass(pblh(i, j), z_at_w, ht(i, j), i, j, c.nz) : 1;
    const int kpbl_q = c.no_pbl_q == 1 ? kpbl_mass(pblh(i, j), z_at_w, ht(i, j), i, j, c.nz) : 1;
    for (int kw = 1; kw <= c.nz; ++kw) {
      const int k = kw - 1;
      const real wpbl_t = c.no_pbl_t == 1 ? wpbl_of(kw, kpbl_t) : real(1.0);
      const real wpbl_q = c.no_pbl_q == 1 ? wpbl_of(kw, kpbl_q) : real(1.0);
      real val_analysis = t_old(i, j, k) * (real(1.0) - c.coef) + t_new(i, j, k) * c.coef;
      rth(i, j, k) = rth(i, j, k) + c.gt * wpbl_t * wzfac(kw, c.if_zfac_t, c.k_zfac_t) * c.tfac *
                                        (val_analysis - th3d(i, j, k) + real(300.0));
      // TWG 2015 pseudo radiative flux
      const real rho_air = p3d(i, j, k) / (real(287.0) * th3d(i, j, k));
      hfx_fdda(i, j, k) = rho_air * real(1004.0) * rth(i, j, k) * dz8w(i, j, k);
      val_analysis = q_old(i, j, k) * (real(1.0) - c.coef) + q_new(i, j, k) * c.coef;
      rqv(i, j, k) = rqv(i, j, k) + c.gq * wpbl_q * wzfac(kw, c.if_zfac_q, c.k_zfac_q) * c.tfac *
                                        (val_analysis - qv3d(i, j, k));
    }
  }
};

}  // namespace

void PhysicsSuite::fddagd_driver(const dyn::State& s, const dyn::BaseState& b) {
  const FddaOptions& f = o_.fdda;
  if (f.grid_fdda == 0) return;  // (grid_sfdda: not ported, always 0)
  const int nx = g_.nx, ny = g_.ny, nz = g_.nz;
  const Range3D mr{0, nx, 0, ny, 0, nz};
  const KFddaZero zero{pt.rundgdten.view(), pt.rvndgdten.view(), pt.rthndgdten.view(), pt.rqvndgdten.view()};
  if (itimestep == 1)  // grid_fdda = 1
    parallel_for(mr, KFddaOldFromState{u_ndg_old.view(), v_ndg_old.view(), t_ndg_old.view(), ph_ndg_old.view(),
                                       q_ndg_old.view(), mu_ndg_old.view(), s.u_2.cview(), s.v_2.cview(),
                                       prep.th_phy.cview(), s.ph_2.cview(), s.q_2[s.P_QV].cview()});
  // fgdtzero = 1: the tendencies are 0 between the calls
  if ((itimestep - 1) % stepfg == 0 && f.fgdtzero == 1) parallel_for(mr, zero);
  if (!(itimestep == 1 || itimestep % stepfg == 0)) return;
  parallel_for(mr, zero);

  // PSUFDDAGD (the begin / end / interval unit checks of the driver: tools/wrf2cppwrf.py and the model's options)
  if (o_.sf_sfclay_physics != 1 && o_.sf_sfclay_physics != 5 && o_.sf_sfclay_physics != 7 && o_.sf_sfclay_physics != 91)
    column_for(g_.columns(), KFddaRegime{regime.view(), pblh.cview(), prep.z_at_w.cview(), b.ht.view()});

  // fddagd: the step's times (XTIME since the simulation start; curr_mins2 since this run's start)
  const real xtime = wrf_time::minutes<real>(clock_since_start());
  // solve_em: curr_mins2 = REAL(real_time_r8(current - start) / 60.0d0)
  const real curr_mins2 = real(wrf_time::real_time<double>(clock_since_run_start()) / 60.0);
  const int end_h = f.end_h, end_h_sfc = 0;  // auxinput9_end_h: check_a_mundo sets it to 0 without grid_sfdda
  const real dtramp = real(f.dtramp_min);
  real actual_end_fdda_min = real(end_h) * real(60.0);
  real actual_end_fdda_min_sfc = real(end_h) * real(60.0);
  if (f.if_ramping == 1 && dtramp > real(0.0)) actual_end_fdda_min = real(end_h) * real(60.0) + std::fabs(dtramp);
  // (WRF: this statement follows the one-line IF above and runs always)
  actual_end_fdda_min_sfc = real(end_h_sfc) * real(60.0) + std::fabs(dtramp);
  if (xtime > actual_end_fdda_min && xtime > actual_end_fdda_min_sfc) {
    // after the end time: the tendencies 0 (RMUNDGDTEN too), no nudging
    parallel_for(mr, zero);
    return;
  }
  const int analysis_interval = f.interval_m;
  const real xtime_old = real(int(std::floor(curr_mins2 / real(analysis_interval))) * analysis_interval) * real(1.0);
  const real xtime_new = xtime_old + real(analysis_interval);
  real coef = (curr_mins2 - xtime_old) / (xtime_new - xtime_old);  // int4 = 1: temporal interpolation

  // the ramp of the nudging coefficients near the end of the nudging period
  real tfac = real(1.0);
  if (f.if_ramping == 1 && std::fabs(dtramp) > real(0.0)) {
    if (dtramp <= real(0.0)) actual_end_fdda_min = real(end_h) * real(60.0);
    else actual_end_fdda_min = real(end_h) * real(60.0) + dtramp;
    if (xtime < actual_end_fdda_min - std::fabs(dtramp)) {
      tfac = real(1.0);
    } else if (xtime >= actual_end_fdda_min - std::fabs(dtramp) && xtime <= actual_end_fdda_min) {
      tfac = (actual_end_fdda_min - xtime) / std::fabs(dtramp);
      if (dtramp > real(0.0)) coef = real(0.0);  // the rampdown nudges toward the past analysis only
    } else {
      tfac = real(0.0);
    }
  }

  FddaScalars c{};
  c.coef = coef; c.tfac = tfac;
  c.guv = real(f.guv); c.gt = real(f.gt); c.gq = real(f.gq);
  c.nz = nz;
  c.no_pbl_uv = f.if_no_pbl_nudging_uv; c.no_pbl_t = f.if_no_pbl_nudging_t; c.no_pbl_q = f.if_no_pbl_nudging_q;
  c.if_zfac_uv = f.if_zfac_uv; c.k_zfac_uv = f.k_zfac_uv; c.if_zfac_t = f.if_zfac_t; c.k_zfac_t = f.k_zfac_t;
  c.if_zfac_q = f.if_zfac_q; c.k_zfac_q = f.k_zfac_q;
  // the u / v weights read the west / south neighbours' PBL heights and levels (WRF: HALO_EM_FDDA_SFC)
  if (c.no_pbl_uv == 1) tile::exchange(pblh, prep.z_at_w);
  column_for(Range2D{1, nx, 0, ny}, KFddagdU{c, pt.rundgdten.view(), u_ndg_old.cview(), u_ndg_new.cview(),
                                             s.u_2.cview(), prep.z_at_w.cview(), pblh.cview(), b.ht.view()});
  column_for(Range2D{0, nx, 1, ny}, KFddagdV{c, pt.rvndgdten.view(), v_ndg_old.cview(), v_ndg_new.cview(),
                                             s.v_2.cview(), prep.z_at_w.cview(), pblh.cview(), b.ht.view()});
  column_for(g_.columns(),
             KFddagdTQ{c, pt.rthndgdten.view(), pt.rqvndgdten.view(), hfx_fdda.view(), t_ndg_old.cview(),
                       t_ndg_new.cview(), q_ndg_old.cview(), q_ndg_new.cview(), prep.th_phy.cview(),
                       s.q_2[s.P_QV].cview(), prep.p_phy.cview(), prep.dz8w.cview(), prep.z_at_w.cview(), pblh.cview(),
                       b.ht.view()});
}

}  // namespace cppwrf::phys::driver
