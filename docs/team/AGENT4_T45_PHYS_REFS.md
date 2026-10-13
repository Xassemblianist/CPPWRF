# T4.5 (Agent 4): WRF real-data reference runs with physics

Goal: references for coupling the column physics into CPPWRF real-data runs, one scheme group
at a time, so each coupling step can be checked against WRF in isolation. Same domain and input
as R0 (wrfinput_d01/wrfbdy_d01 from ~/wrf_real/run_R0), WRF v4.6.1, dt = 60 s.

Base namelist = run_R3 (hybrid_opt 2, w_damping 1, damp_opt 3, diff_opt 0, khdif/kvdif 0)
with the changes per run below. Every run in FP32 (WRF-4.6.1) AND real*8 (WRF-4.6.1-r8).
Directories ~/wrf_real/run_P1, run_P1_r8, ... (outside the repo).

| run | physics |
|---|---|
| P1 | sf_sfclay_physics 1 (sfclayrev), sf_surface_physics 2 (Noah, num_soil_layers 4), bl_pbl_physics 1 (YSU), bldt 0; mp 0, cu 0, ra 0 |
| P2 | P1 + cu_physics 1 (KF), cudt 0 |
| P3 | P1 + ra_lw_physics 4, ra_sw_physics 4 (RRTMG), radt 1 (every step) |
| P4 | P1 + mp_physics 6 + cu 1 + RRTMG radt 1 + diff_opt 1, km_opt 4 (the operational suite) |
| P4_12h | P4, 12 h, both boundary records (as run_bdy12), FP32 only |

Output: 1 h runs, history every 10 min, plus a second short run of each (3 steps,
history_interval_s 60) in <run>/steps/ as run_mp6_r8/steps. Add to the history (iofields_filename,
"+:h:0:" lines) every physics tendency and surface field the schemes produce: RUBLTEN, RVBLTEN,
RTHBLTEN, RQVBLTEN, RQCBLTEN, RQIBLTEN, RTHCUTEN, RQVCUTEN, RQCCUTEN, RQRCUTEN, RQICUTEN,
RQSCUTEN, RTHRATEN, RTHRATLW, RTHRATSW, HFX, QFX, LH, UST, ZNT, PBLH, TSK, TSLB, SMOIS, SH2O,
GRDFLX, CHS, CHS2, CQS2, FLHC, FLQC, QSFC, GLW, SWDOWN, GSW, RAINC, RAINCV, RAINNC, NCA, W0AVG.

Record in docs/team/AGENT4_REPORT_T45.md: the exact namelist diffs vs R3, wall times (timed
with tools/heavy.sh --exclusive, 4 OpenMP threads, FP32 and r8), and anything WRF printed
(rsl warnings, w-cfl messages). Also check the namelist.output values of every physics
option that WRF changes silently (e.g. sf_sfclay with isfflx, num_land_cat, usemonalb,
sst_update) and list them.

Machine rules: tools/heavy.sh for every build/run, -j4; nothing in the repo root; no git writes.
