# Agent 7 audit A3: Agent 0's dycore commits and the physics ports since your last audit

Audit, do not fix. Write findings to docs/audit/ (one file per item) and update docs/audit/INDEX.md.
1. Commit 3801f2a (map factors, Coriolis, curvature, terrain, damp_opt=3), 7e84567 (diffusion
   BCs), 854ede1 (WSM6 coupling): line-by-line against the WRF v4.6.1 source in
   ~/wrf_ref/WRF/dyn_em (couple_momentum, calc_ww_cp, advect_u/v/w/scalar 5th/3rd order,
   advect_scalar_pd, horizontal_pressure_gradient, pg_buoy_w, rhs_ph, coriolis, curvature,
   rk_addtend_dry, rk_update_scalar, small_step_prep, advance_uv, advance_mu_t, sumflux,
   advance_w, small_step_finish, vertical_diffusion*, microphysics_driver wsm6 call). For each:
   every map factor / index / sign / loop range that differs from WRF, with file:line on both
   sides. Note also deviations in evaluation order that change rounding.
2. Mutation-test tests/verify/dycore_checks.py: flip one map factor at a time (e.g. remove
   msfty in KTendPh, msftx in KMuT, the 1/msfty on h_diabatic) in a scratch copy
   (git worktree in /tmp, never in the shared tree; never git write in the repo) and confirm
   the map_factor_invariance check fails for each. List any mutation that survives.
3. Re-run Agent 2's YSU and sfclayrev tests and Agent 6's restart acceptance; VERIFIED / NOT
   REPRODUCED with commands and output.
Then stop.
