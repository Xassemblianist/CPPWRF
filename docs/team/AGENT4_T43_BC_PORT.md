# Agent 4 task T4.3: WRF specified lateral boundary conditions, ported with a differential test

Your T4.2 reference runs (R0..R3, r8) are the ground truth for the real-data dycore. Agent 0
will integrate the lateral boundaries into the time loop; you port the routines.

Ownership (granted): src/bdy/ and tests/bdy/ (new). Read src/dyn/*.hpp and src/core/field.hpp
for the conventions (View3, BView2, parallel_for/column_for, CPPWRF_HD, C++ k = Fortran k - 1,
halo 3). Do not edit src/dyn.

1. Call map first (docs/team/AGENT4_BC_CALLMAP.md): for a real-data run with spec_bdy_width=5,
   spec_zone=1, relax_zone=4 and physics off, list in order every boundary-related call in
   solve_em.F, module_first_rk_step_part1/2.F, module_em.F, module_small_step_em.F,
   module_bc_em.F, share/module_bc.F (relax_bdy_dry, spec_bdy_dry, relax_bdy_scalar,
   spec_bdy_scalar, spec_bdyupdate, spec_bdyupdate_ph, zero_grad_bdy, set_physical_bc*,
   flow_dep_bdy, spec_bdy_final, the time interpolation of *_BXS/*_BTXS, ...): which RK
   stage / small step, which fields, which arguments, and every place where the *interior*
   dycore code changes near the boundary because config_flags%specified is true (loop
   ranges in advect_*, rhs_ph, horizontal_pressure_gradient, coriolis, curvature,
   calc_mu_uv, advance_*; degraded advection orders near the edges). Be complete: this is
   what Agent 0 implements from.
2. Port the routines of module_bc_em.F / module_bc.F used above as C++ kernels in src/bdy/
   (single source, FP32/FP64), plus a reader-independent struct for the boundary arrays
   (west/east/south/north, width 5, values and tendencies).
3. Differential test: compile the Fortran routines with gfortran in tests/bdy/build, drive both
   with the same random but physically scaled inputs (several tiles: corners included), PASS =
   bit-exact or <= 1e-12 rel in FP64. Also a check against your R0 wrfbdy: the boundary values
   at t = 0 and the tendencies reproduce WRF's own (from wrfbdy_d01 and wrfinput).
Report: docs/team/AGENT4_REPORT.md, section T4.3. Then stop.
