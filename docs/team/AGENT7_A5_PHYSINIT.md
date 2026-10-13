# A5 (Agent 7): physics initialisation (Agent 8, commit 6d64875) against WRF's own t = 0 state

Agent 8 ported landuse_init, lsminit (Noah SH2O via frh2o, SNOALB) and the LANDUSE/VEGPARM/
SOILPARM/GENPARM readers (src/phys/driver/physics_init.*, landuse_tables.*), verified against
a Fortran reader and a smoke test only. Check them end to end against what WRF itself
computes at t = 0:
1. Make your own run directory ~/wrf_real/run_A5_r8 (WRF-4.6.1-r8): real.exe with
   sf_surface_physics 2 (Noah), num_soil_layers 4, sf_sfclay_physics 1, bl_pbl_physics 1, from
   the same met_em files as R0 (see ~/wrf_real/run_R0 / wps_run), then wrf.exe for one step with
   history at t = 0 (and iofields adding every surface/soil field WRF initialises: ALBEDO,
   ALBBCK, EMISS, EMBCK?, ZNT, Z0, MAVAIL, THC, XLAND, SNOALB, SH2O, SMOIS, TSLB, LU_INDEX,
   IVGTYP, ISLTYP, VEGFRA, SHDMAX/SHDMIN, ...). (Agent 4 is making similar runs for P1-P4; do not
   touch their directories.)
2. Drive Agent 8's init code (a small C++ test in tests/phys/driver/, FP64) with that wrfinput
   and compare every initialised field with WRF's t = 0 history values. Expected: equal to
   roundoff; list every difference with the WRF source line that explains it.
3. Line-by-line review of physics_init.cpp / landuse_tables.cpp against WRF 4.6.1
   (phys/module_physics_init.F landuse_init, lsminit; module_sf_noahlsm.F soil_veg_gen_parm),
   with file:line on both sides; which branches are ported, which are skipped and whether a
   skipped branch is executed by this case.
Report: docs/audit/agent8_physinit.md. Rules as always; no edits to Agent 8's files (only your
test and report).
