# Agent 8 task T8.1: physics driver glue (WRF phy_prep / tendency coupling), differential-tested

You are Agent 8 (Gemini 3.8 Flash High), new. Read docs/team/PLAN.md, HIERARCHY.md and
AGENT2_PHYSICS.md (conventions, Fortran differential harness). The column schemes exist
(Kessler, WSM6, KF, Noah, sfclayrev, YSU); what is missing is WRF's glue between the dycore
state and the schemes, which Agent 0 will call from the time loop.

Ownership (granted): src/phys/driver/ and tests/phys/driver/ (new). Do not edit other paths.
1. Port, statement by statement, from dyn_em/module_big_step_utilities_em.F and
   phys/module_physics_addtendc.F / module_first_rk_step_part1.F:
   phy_prep (th_phy, p_phy, pi_phy, rho, t_phy, z, z_at_w, dz8w, p8w, t8w, u_phy, v_phy with
   the WRF averaging and the surface/top extrapolations), and the tendency coupling
   (update_phy_ten / add_a2a, add_a2c_u, add_a2c_v and the mu-coupling of RTHBLTEN, RUBLTEN,
   RVBLTEN, RQVBLTEN, RTHCUTEN, RTHRATEN ... into t_tendf, ru_tendf, rv_tendf, moist_tend,
   including the map-factor divisions). Use the current BaseState/State field names
   (src/dyn/state.hpp) in the C++ interface; FP32/FP64; CPU and CUDA.
2. Differential test against those Fortran subroutines compiled with gfortran
   (tests/phys/driver/build), random but physical input on a 3D tile with terrain and
   non-unit map factors; PASS = bit-exact or <= 1e-12 rel (FP64).
3. Write docs/PHYSICS_DRIVER.md: for each scheme (sfclayrev, YSU, KF, Noah, WSM6), the exact
   inputs it gets from phy_prep and where its outputs go, in WRF order.
Report: docs/team/AGENT8_REPORT.md. Then stop.
