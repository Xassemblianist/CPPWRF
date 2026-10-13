# A4 (Agent 7): audit of w_damp and calc_cq, and a case that triggers w_damp

Commits 0c8f927 (calc_cq precomputed in WRF summation order: KCq in src/dyn/large_step.cpp,
kern::MoistCoef) and 468e2bb (w_damping: KTendW in src/dyn/large_step.cpp, keys dyn.w_damping /
dyn.w_crit_cfl in src/cases/model.cpp).

1. Line-by-line check against WRF 4.6.1 (~/wrf_real/WRF-4.6.1/dyn_em): calc_cq in
   module_big_step_utilities_em.F, w_damp in the same file and its call in module_em.F
   rk_tendency (which u, v, w, ww, mut, dt are passed; loop ranges; where in the order of the
   rw_tend terms it is applied). Report every difference, with WRF line numbers.
2. The real-data runs R3 never exceed w-CFL 1, so the w_damp branch is untested. Build a case
   where it fires, in both WRF and CPPWRF, and compare: e.g. squall2d (configs/squall2d.ini,
   WRF em_squall2d_x) with w_damping = 1 and a dt / vertical grid that pushes w-CFL above 1 in
   the updraft (WRF prints "points exceeded v_cfl"). Run WRF yourself (~/wrf_ref has the ideal
   builds). Compare w, theta at the first output times with tests/verify/verify.py wrf or a
   small script in tests/verify/. Also run it with w_damping = 0 to show the difference
   between on and off is much larger than the CPPWRF-vs-WRF difference.
3. Mutation test: change w_alpha 0.3 -> 0.31 and the sign convention in a scratch copy;
   both must be detected by the comparison of item 2.

Report: docs/audit/agent0_wdamp_calccq.md, with the commands that prove each claim.
Rules: scratch copies outside the repo, tools/heavy.sh, no git writes, no repo edits except
the report and (if you add one) the comparison script.
