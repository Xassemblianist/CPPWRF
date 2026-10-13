# T8.3 (Agent 8): the real-data physics suite behind dyn::ColumnPhysics

The dycore hook is committed (e94c05e): src/dyn/column_physics.hpp. RK3 calls
  part1(s, b, t)   RK stage 1, after rk_step_prep and init_zero_tendency
                   = WRF first_rk_step_part1: phy_prep, radiation_driver, surface_driver,
                     pbl_driver, cumulus_driver
  part2(s, b, t)   RK stage 1, after the stage-1 diffusion, before rk_tendency
                   = first_rk_step_part2: calculate_phy_tend + update_phy_ten into
                     t.ru_tendf, t.rv_tendf, t.t_tendf, t.q_tendf (moist species in State order)
  after_rk(s, b)   after the RK loop, before microphysics = phy_prep_part2
Read src/dyn/rk3.cpp (RK3::step) and src/dyn/large_step.cpp (LargeStep::tendencies) to see
exactly where they run.

Write src/phys/driver/suite.{hpp,cpp}: class PhysicsSuite : public dyn::ColumnPhysics for
  sf_sfclay_physics 1 (sfclayrev), sf_surface_physics 2 (Noah), bl_pbl_physics 1 (YSU),
  cu_physics 1 (KF), ra_lw/ra_sw_physics 4 (RRTMG, Agent 2's port; add it last, behind a flag,
  when Agent 2's LW/SW are verified), each switchable to 0.
- Port statement by statement ONLY the branches of phys/module_surface_driver.F,
  module_pbl_driver.F, module_cumulus_driver.F, module_radiation_driver.F (and
  module_physics_addtendc.F, dyn_em/module_big_step_utilities_em.F phy_prep_part2) that these
  options execute in WRF 4.6.1 with the namelists of ~/wrf_real/run_P1..P4 (Agent 4, T4.5).
  Every pre/post-processing line counts (u_phytmp, v_phytmp, qv_curr, psfc, chklowq, the
  KF stepcu/NCA logic, bldt/cudt = 0 -> every step, radt, ...). List skipped branches.
- Persistent state: the surface/soil fields (TSK, TSLB, SMOIS, SH2O, SNOW, ..., UST, ZNT, MOL,
  PBLH, HFX, QFX, ..., RAINC, NCA, W0AVG, the R*TEN fields) as device fields; initialised from
  wrfinput (io_netcdf::read_wrfinput; extend Agent 6's reader if fields are missing) with your
  T8.2 init code.
- Everything device-resident: parallel_for kernels over columns using the existing scheme
  kernels (sfclayrev, noah, ysu, kf: they are CPPWRF_HD). No per-step host copies.
- A factory std::unique_ptr<dyn::ColumnPhysics> make_physics_suite(const Config&, const Grid&,
  ...) plus the config keys (phys.sf_sfclay_physics, phys.sf_surface_physics,
  phys.bl_pbl_physics, phys.cu_physics, phys.ra_lw_physics, phys.ra_sw_physics, phys.radt,
  phys.table_dir). Agent 0 wires it into src/cases/model.cpp; do not edit src/dyn/ or
  src/cases/model.cpp (tell me the 3 lines you need).
- Output: the surface fields and R*TEN for comparison with WRF (HFX, QFX, UST, PBLH, TSK,
  RAINC, RTHBLTEN, RTHCUTEN, RTHRATEN, ...) through the existing snapshot/wrfout writers.
Acceptance (Agent 0 runs it after wiring, you prepare the configs and script): configs
real_p1.ini .. real_p4.ini matching Agent 4's P1..P4; step 1 against the r8 references
(steps/) to roundoff in CPU FP64, the 1 h FP32 runs at FP32 level. Extend
tests/verify/compare_real.py for the 2D surface fields.
Report in docs/team/AGENT8_REPORT.md, section T8.3. Rules as always (heavy.sh, no git writes).
