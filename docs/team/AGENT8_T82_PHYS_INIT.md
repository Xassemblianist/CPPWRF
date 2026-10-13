# Agent 8 task T8.2: physics initialisation for real data, and a reference-code fix

1. Fix first (quality gate): your differential test must compare against the ORIGINAL WRF
   subroutines, compiled unmodified from ~/wrf_ref/WRF (phy_prep from
   dyn_em/module_big_step_utilities_em.F, add_a2a/add_a2c_u/add_a2c_v/update_phy_ten from
   phys/module_physics_addtendc.F, ...), not a re-written tests/phys/driver/
   wrf_phys_driver_subs.f90. Extract the routines verbatim (a script that copies the exact
   source lines is fine; any edit needed to compile standalone must be minimal and listed),
   rerun the comparison and report it.
2. Physics initialisation used by a real-data run (src/phys/driver/, tests/phys/driver/):
   port and differential-test (against the original Fortran, same method) the init paths the
   schemes need at t = 0 from wrfinput: the table readers LANDUSE.TBL, VEGPARM.TBL,
   SOILPARM.TBL, GENPARM.TBL (from ~/wrf_ref/WRF/run; compare the parsed values), Noah
   lsminit / soil_veg_gen_parm, sfclayrevinit, ysuinit, wsm6init (if not already in Agent 2's
   port), kfinit / kf_lutab (check Agent 5's port: reuse, do not duplicate), and the
   phy_init / start_em physics-related steps for these schemes (what is computed from
   TSK, TSLB, SMOIS, IVGTYP, ISLTYP, XLAND, LU_INDEX, ... and in which order). Use the R0 wrfinput
   of ~/wrf_real/wrf_run as real input for a smoke test.
3. Extend docs/PHYSICS_DRIVER.md with the init order.
Report: docs/team/AGENT8_REPORT.md, section T8.2. Then stop.
