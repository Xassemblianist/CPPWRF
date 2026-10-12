# Agent 1 task M2: moist dynamics core + tracer transport + physics coupling

You are Agent 1 (lead implementer). Read docs/team/PLAN.md, docs/STATUS.md, docs/review/REVIEW.md.
Your quota is scarce, so you do ONLY the formula-critical core below. Everything mechanical
(runs, sweeps, benchmarks, plots, physics ports, Coriolis/damping/3D configs) belongs to Agent 3
(Gemini). Ask it for things under "Requests for Agent 3" in STATUS.md, with a precise spec.
Keep thinking short; write code in small steps; compile often.

## Ownership change
src/dyn/small_step.{hpp,cpp} is now yours (written by Agent 0, who stays reviewer). Keep the
3-fused-kernels-per-substep structure and the WRF routine mapping comments.
Parallel work by others: Agent 3 does GPU validation (T3.4, no source edits), Agent 2 the physics
library port into src/phys/, Agent 4 real-data init (src/init_real/). After you write "M2 done" in STATUS.md, Agent 3 adds
Coriolis, damping and 3D configs (T3.6) to your files, so leave those out of M2.

## Scope (commit after each item: CPU FP32 + FP64 ctest green, CUDA build compiles and its ctest is green)
1. Scalar transport. State gets n_tracers mass-point scalars (moist array: qv, qc, qr, ... plus
   passive tracers). Advect them as WRF does: rk_scalar_tend / advect_scalar (5th horizontal,
   3rd vertical) with the time-averaged fluxes ru_m, rv_m, ww_m, then rk_update_scalar; plus
   moist_adv_opt = 1 (advect_scalar_pd). Tests: (a) a tracer that is 1 everywhere stays 1 to
   roundoff (FP64) in straka, warm_bubble and igw, which proves the fluxes match mass
   continuity; (b) with PD on, a blob stays >= 0; with PD off it does not; (c) tracer mass is
   conserved to roundoff. Tracers go into snapshots under their names.
2. Moist dynamics (TN 2.2, use_theta_m = 1): theta_m = theta (1 + Rv/Rd qv); cqu/cqv/cqw
   moisture coefficients in advance_uv/advance_w and in the large-step PGF/buoyancy
   (pg_buoy_w with qtot); moist alt/al in calc_p_rho_phi and calc_p_rho; c2a with the moist
   alt; moist initialisation (sounding with qv). Reference: solve_em, module_small_step_em,
   module_big_step_utilities_em (the moist paths). Test: a saturated-free moist rest state
   stays at rest (roundoff), and dry results are bitwise unchanged when qv = 0.
3. Physics coupling interface (src/dyn/physics_coupling.{hpp,cpp}): where and how WRF calls
   microphysics (after the RK3 loop in solve_em: moist_physics_prep_em, microphysics_driver,
   moist_physics_finish_em, h_diabatic for the next step's small_step_finish), and how
   tendency-based physics (PBL, radiation) enters through *_tendf / RTHRATEN-style arrays in
   stage 1. Define the C++ interface that Agent 3's src/phys/ kernels plug into (views of
   theta, qv, qc, qr, rho, pii, p, dz8w, z, dt, rainnc, ...), with a no-op scheme and a
   test. Write the interface down in docs/PHYSICS_INTERFACE.md so Agent 3 can target it.
4. Moist bubble case (Bryan & Fritsch 2002 moist benchmark), runnable once Agent 3's Kessler
   (T3.5 scheme 1) passes its differential test. If Kessler isn't ready yet, leave the case
   wired to the no-op scheme and say so.

Not in M2: terrain, open/specified lateral BCs, map factors, performance work.

## Rules (as before)
The WRF source is the reference; cite routine names. No clamping or ad-hoc diffusion. Every
claimed result needs the command and its output. At the end: update docs/STATUS.md (results,
departures from WRF, requests for Agent 3, Disagreements), write "M2 done", commit, stop.
