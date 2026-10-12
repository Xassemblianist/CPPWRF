# Agent 1 task M2: complete dry 3D dycore + tracer transport

You are Agent 1 (lead implementer). Read docs/team/PLAN.md, docs/STATUS.md, docs/review/REVIEW.md.
Your quota is scarce: write code in small steps, compile often, and keep thinking short. Long
experiments, benchmark runs and plots go to Agent 3 (Gemini). Ask for them in STATUS.md under
"Requests for Agent 3" with a precise spec.

## Ownership change
Agent 0 hands src/dyn/small_step.{hpp,cpp} over to you from now on (Agent 0 stays reviewer).
Keep its structure (3 fused kernels per substep) and its WRF routine mapping comments.
Agent 3 is running GPU validation and profiling (docs/team/AGENT3_GPU.md) in parallel. Do not
start performance work; its numbers will drive M3.

## Scope (in this order, commit after each item with its tests green on CPU FP32 and FP64,
## and the CUDA build still compiling; the GPU now works, so also run ctest in build-cuda)
1. Coriolis on an f-plane: WRF `coriolis` (module_big_step_utilities_em.F) with constant f and
   e = 0 options (`dyn.f`, default 0), added into ru_tend/rv_tend (and rw_tend if e != 0).
   Test: igw with f = 1e-4 1/s (Skamarock & Klemp 1994 used it) and a 3D unit test, where an
   inertial oscillation in a uniform flow must have period 2*pi/f within 1% (FP64).
2. Upper damping: WRF damp_opt = 3 (implicit Rayleigh damping of w inside advance_w, with
   zdamp, dampcoef) in small_step.cpp, as in WRF advance_w. Also w_damping = 1 (the
   CFL-based w limiter in WRF's rk_tendency) as an option. Test: a mountain-free gravity-wave
   pulse reflects measurably less from the lid with damping on than off (report the ratio).
3. 3D cases: make warm_bubble and rest_state run 3D with symmetric or periodic y (configs
   configs/warm_bubble_3d.ini, ny = nx). Check the bubble stays axisymmetric (x <-> y swap
   symmetry of th_pert: max rel difference ~roundoff in FP64). Add this check to your unit
   tests or a script under tools/. tests/verify/ belongs to Agent 0.
4. Scalar transport for tracers (prerequisite for moisture): State gets an array of n_tracers
   mass-point scalars (qv, qc, qr later; start with "tracer_1"). Advect them like WRF:
   `advect_scalar` (5th horizontal / 3rd vertical) with the time-averaged mass fluxes
   ru_m, rv_m, ww_m from the small step, in rk_update_scalar form, plus WRF's positive-definite
   option (moist_adv_opt = 1, `advect_scalar_pd`). Tests: (a) a tracer that equals 1
   everywhere stays exactly 1 (FP64 roundoff) in Straka, warm_bubble and igw, which proves
   the fluxes are consistent with the mass continuity; (b) a tracer blob stays non-negative
   with pd on and goes negative with pd off; (c) tracer mass is conserved to roundoff.
   Write tracers into snapshots (I/O contract: add them as variables named after the tracer).

Not in M2: moisture thermodynamics (theta_m, cqw), microphysics, terrain, open/specified
lateral BCs, map factors. Those come in M3/M4.

## Rules (as before)
WRF source in your scratch dir is the reference; cite routine names. No clamping or ad-hoc
diffusion. Every claimed result needs the command and its output. At the end: update
docs/STATUS.md (results table, departures from WRF, requests for Agent 3, Disagreements),
commit, stop.
