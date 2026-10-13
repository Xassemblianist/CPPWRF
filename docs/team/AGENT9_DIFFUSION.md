# Agent 9 task T9.1: WRF diffusion and damping kernels, differential-tested against Fortran

You are Agent 9 (Gemini 3.8 Flash High), new. Read docs/team/PLAN.md, HIERARCHY.md and
AGENT2_PHYSICS.md (conventions and the Fortran differential-test method: compile the original
routines with gfortran, drive Fortran and C++ with identical inputs, PASS = bit-exact or
<= 1e-12 rel in FP64). Read src/dyn/large_step.cpp (KDiff*, VCoef, BaseState map factors) and
src/core/field.hpp (View3, BView2/3, parallel_for, CPPWRF_HD, C++ k = Fortran k - 1, halo 3).

Ownership (granted): src/dyn/diffusion/ and tests/dyn/diffusion/ (new). Do not edit other
files; Agent 0 wires your kernels into the time loop.

Port statement by statement from WRF v4.6.1 (~/wrf_ref/WRF/dyn_em), with all map factors and
c1/c2 hybrid coefficients, for periodic and for specified (real-data) lateral boundaries (the
config_flags%specified loop ranges and edge copies):
1. module_diffusion_em.F: compute_diff_metrics (z, rdz, rdzw, zx, zy), cal_deform_and_div
   (defor11/22/33/12/13/23, div), calculate_km_kh for km_opt = 1 and 4 (smag2d_km incl. the
   diff_opt=2 terrain alpha), and for diff_opt = 2: horizontal_diffusion_2 and
   vertical_diffusion_2 (stress tensor form) with km_opt 1 and 4, bl_pbl_physics = 0 path.
2. module_big_step_utilities_em.F: horizontal_diffusion ('u','v','w','m'),
   horizontal_diffusion_3dmp, vertical_diffusion / _u / _v / _3dmp / _mp (diff_opt = 1),
   sixth_order_diffusion (diff_6th_opt = 1 and 2, diff_6th_factor, diff_6th_slopeopt=0),
   w_damp (w_damping = 1, max_vert_cfl), as kernels taking K as a 3D view (xkmh, xkhh, ...).
3. Differential tests in tests/dyn/diffusion (gfortran, tests/dyn/diffusion/build), on a 3D
   tile with terrain (non-flat ph/phb), non-unit varying map factors, hybrid c1..c4 != (1,0),
   random but physical u, v, w, theta, moisture; both periodic and specified boundaries.
4. A short header doc (src/dyn/diffusion/README.md): for each kernel the WRF routine, inputs,
   outputs and in which RK stage WRF calls it (rk_step == 1 only for the *_tendf terms).
Report: docs/team/AGENT9_REPORT.md. Then stop.
