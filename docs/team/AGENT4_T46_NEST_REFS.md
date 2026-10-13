# T4.6 (Agent 4): WRF two-domain (nested) reference runs

For Agent 0's nesting implementation. Same parent domain and GFS data as R0 (15 km, 119x89,
44 levels). Add d02: parent_grid_ratio 3 (5 km), parent_time_step_ratio 3, i_parent_start /
j_parent_start such that d02 covers central Turkey with at least 20 parent cells to every
parent boundary, e_we/e_sn around 121x91 (nest dims must satisfy (e_we-1) % 3 == 0).
WPS: geogrid + metgrid for max_dom 2 (in a copy of ~/wrf_real/wps_run), real.exe max_dom 2
(wrfinput_d01, wrfinput_d02, wrfbdy_d01), WRF-4.6.1-r8 and FP32.

Runs (namelist = R3 dynamics: hybrid_opt 2, w_damping 1, damp_opt 3, diff_opt 0, no physics,
qv only as in R0, input_from_file .true. for d02, smooth_option 0):
| run | feedback | notes |
|---|---|---|
| N1 | 0 (one-way) | 1 h, history every 10 min for both domains, and steps/ (every d01 step for 3 d01 steps = 9 d02 steps, history_interval_s per domain) |
| N2 | 1 (two-way) | same |
Each in r8 (~/wrf_real/run_N1_r8, run_N2_r8) and FP32 (run_N1, run_N2).
Record: namelist diffs, the WRF log lines about nest init (blend_terrain, input_from_file,
which fields come from wrfinput_d02 vs interpolated), timings. Note in the report whether WRF
modified d02 terrain (HGT) at start (compare wrfinput_d02 HGT with wrfout_d02 HGT at t=0).
Rules: tools/heavy.sh (WRF runs are long: fine), nothing in the repo root, no git writes.
Report: docs/team/AGENT4_REPORT_T46.md.
