# Agent 5 task T5.1: second physics track (land surface, cumulus, Thompson)

Long task. You are Agent 5 (Gemini) in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md
and docs/team/AGENT2_PHYSICS.md. You follow exactly the same method and C++ conventions as
Agent 2 (port statement by statement, a single-source column_for kernel, and a differential
test against the original Fortran routine compiled with gfortran on >= 2000 columns, with
PASS = max rel diff <= 1e-12 in FP64). Agent 2 owns the shared harness design; until it exists
(tests/phys/README or similar), build your own and switch to Agent 2's when it appears.

## Ownership (only these paths)
src/phys/lsm_noah/, src/phys/cu_kf/, src/phys/mp_thompson/ and the matching
tests/phys/lsm_noah/, tests/phys/cu_kf/, tests/phys/mp_thompson/, plus
docs/team/AGENT5_REPORT.md. Each directory has its own CMakeLists.txt. Do NOT edit
src/phys/CMakeLists.txt or any other Agent 2 file: write in your report the one-line
add_subdirectory request for Agent 2. Never use git write commands or sudo. WRF source:
~/wrf_ref/WRF (read only; compile standalone copies under your tests/phys/<scheme>/fortran/).

## Schemes, in this order
1. sf_surface_physics = 2, the Noah LSM (phys/module_sf_noahlsm.F, called through
   module_surface_driver / sflx). Includes the soil-temperature and soil-moisture solvers and
   the snow model. It needs the VEGPARM.TBL, SOILPARM.TBL and GENPARM.TBL tables: port their
   reading (host side) and pass the parameters to the kernel as a device-side table struct.
   Input regimes: bare soil, vegetation, snow cover, frozen soil, water points (skipped),
   dry and saturated soils, day and night forcing.
2. cu_physics = 1, the Kain-Fritsch scheme (phys/module_cu_kfeta.F, the version called in
   v4.6.1). Note its trigger depends on the time step and on the cudt and stepcu logic:
   reproduce the driver behaviour in the test (call frequency, the NCA counter).
3. mp_physics = 8, Thompson (phys/module_mp_thompson.F). Its lookup tables are generated at
   initialisation (thompson_init: qr_acr_qg, freezeH2O, ...) and are expensive. Port the table
   generation, verify the tables themselves against the Fortran-generated tables (binary
   dump), then verify the column kernel.
For each scheme, also measure ns/column on CPU-12 and on the GPU for 10^5 columns x 50
levels, and report the GPU vs CPU max rel diff (FP64 and FP32).

## Report: docs/team/AGENT5_REPORT.md
Update after each scheme: files, standalone-compilation changes to the Fortran, input regimes,
the comparison table (variable | max abs | max rel | worst column), timings, open issues.
Continue to the next scheme only if the current one PASSES.
