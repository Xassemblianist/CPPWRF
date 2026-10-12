# Agent 3 task T3.6: dycore extensions (Coriolis, upper damping, 3D cases)

QUEUED: start only after Agent 1 writes "M2 done" in docs/STATUS.md (you will edit Agent 1's
files, and nobody else may edit them at the same time). Agent 1 reviews your diff afterwards.

You are Agent 3 in CPPWRF. Read docs/team/PLAN.md, docs/STATUS.md, and the WRF source in
~/wrf_ref/WRF/dyn_em/. For this task only, you may edit: src/dyn/large_step.{hpp,cpp},
src/dyn/small_step.cpp (only the advance_w damping part), src/main.cpp (new config keys,
run.json), configs/ (new files), tests/unit/ (new test files). Port WRF formulas exactly
(cite routine names in comments); keep the existing kernel structure. Never use git write
commands or sudo.

1. Coriolis on an f-plane: WRF `coriolis` (module_big_step_utilities_em.F), config `dyn.f`
   (default 0) and `dyn.e` (default 0), added to ru_tend/rv_tend/rw_tend in rk_tendency
   order. Test: inertial oscillation in a 3D periodic box with a uniform initial u. The period
   must be 2*pi/f within 1% (FP64), and the amplitude must be conserved within 1% over 3
   periods. Also run configs/igw.ini with f = 1e-4 and check it with tests/verify/verify.py.
2. damp_opt = 3 (implicit Rayleigh damping of w at the end of WRF advance_w; keys
   dyn.damp_opt, dyn.zdamp, dyn.dampcoef) and w_damping = 1 (WRF w_damp in rk_tendency).
   Test: a gravity-wave pulse in a deep domain (write a config). The wave energy reflected
   from the lid must be smaller with damping on; report the ratio.
3. 3D configs: configs/warm_bubble_3d.ini (ny = nx, symmetric y) and configs/rest_state.ini
   is already 3D. The test checks x<->y swap symmetry of th_pert at the end (FP64 max rel
   diff ~1e-12) as a new executable in tests/unit/.
All existing ctests and tests/verify/verify.py checks on the four M1 cases must still pass
(CPU FP32 and FP64, and CUDA). Report in docs/team/AGENT3_REPORT_T36.md, with diff stats per
file, then stop.
