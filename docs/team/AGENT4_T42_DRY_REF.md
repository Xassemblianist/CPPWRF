# Agent 4 task T4.2: WRF real-data reference runs with all physics off (dycore validation)

Your T4.1 work is committed (166a615). Agent 0 is now adding map factors, Coriolis/curvature,
terrain and lateral boundaries to the CPPWRF dycore. These runs are the ground truth for it.
Work in ~/wrf_real only (new run directories), every WRF run through tools/heavy.sh.

## Runs (same domain, same met_em / wrfinput / wrfbdy as T4.1, start 2026-10-12_00)
Common namelist changes from your T4.1 namelist:
- &physics: mp_physics=0, ra_lw_physics=0, ra_sw_physics=0, sf_sfclay_physics=0,
  sf_surface_physics=0, bl_pbl_physics=0, cu_physics=0, num_soil_layers unchanged
- &dynamics: diff_opt=0, km_opt=0, diff_6th_opt=0, damp_opt=0, w_damping=0, epssm=0.1,
  non_hydrostatic=.true., moist_adv_opt=1, scalar_adv_opt=1, h_mom_adv_order=5,
  v_mom_adv_order=3, h_sca_adv_order=5, v_sca_adv_order=3, use_theta_m=1
- run_hours=1, restart=.false.
Runs:
- R0: hybrid_opt=0 (rerun real.exe with hybrid_opt=0), history every 10 min.
- R1: as R0, but history_interval_s = the time step, run 5 steps only (debugging output).
- R2: as R0 with hybrid_opt=2.
- R3: as R2 with damp_opt=3, zdamp=5000, dampcoef=0.2, w_damping=1.
Write every namelist used verbatim into your report. Record dt, time_step_sound (as WRF
chose it), and the "Timing for main" of each run (timings: tools/heavy.sh --exclusive).
If any run blows up (CFL, NaN), report it with the rsl output; do not tune silently.

## Then: WRF built with double precision
Build a second WRF em_real in ~/wrf_real/WRF-4.6.1-r8 configured for real*8 (configure -r8 or
the RWORDSIZE=8 setting; serial GNU), and redo R0 and R1 with it (R0_r8, R1_r8). This gives
Agent 0 a reference without FP32 noise. Build with tools/heavy.sh, -j4.

Report: new section T4.2 in docs/team/AGENT4_REPORT.md (paths of every wrfout, namelists,
timings, any problem). Then stop.
