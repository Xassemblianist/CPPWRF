# Agent 6 task T6.2: wire NetCDF output and restarts into the model; extend regression

You are Agent 6 (infrastructure lead). Your T6.1 work (src/io_netcdf, tests/io_netcdf) is
committed (6e8f708). tools/regression/ is not committed yet: Agent 7 audits it first.

## Ownership exception for this task (granted by Agent 0)
You may edit: src/main.cpp, the top-level CMakeLists.txt (only NetCDF/io_netcdf lines),
src/cases/model.{hpp,cpp} (only output/restart config plumbing), configs/ (new keys), plus your
own paths. Keep each change small; do not touch numerics in src/dyn or src/core.

## Work
1. CMake: `find_package` / pkg-config for NetCDF-C. If found, build cppwrf_io_netcdf and link
   it into cppwrf with a compile definition CPPWRF_HAVE_NETCDF. If not found, everything builds
   as before.
2. Output: config `output.format = native | wrfout | both` (default native, so verify.py and
   the regression golden files keep working). wrfout files go to the run's output dir, named
   like WRF (wrfout_d01_<date>_<time>; start date from `run.start_date`, default
   2000-01-01_00:00:00). Include moist species (QVAPOR, QCLOUD, QRAIN) and RAINNC when present.
3. Restart: `restart.interval` (s, 0 = off) writes restart files; `run.restart_file = <path>`
   starts from one. Acceptance (must be bitwise, FP32 and FP64, CPU and CUDA):
   run straka 600 s straight vs 300 s + restart + 300 s, then `cmp` of the final snap .bin and
   diag.csv columns 1-10 identical. Do the same for squall2d (moisture, h_diabatic, rainnc must be
   in the restart file, or it will not be bitwise: check this).
4. Regression: add squall2d (FP64 symmetric check + water budget via verify.py), CUDA
   backends when a GPU is present, and run every job through tools/heavy.sh (timings with
   --exclusive). Update golden files only after Agent 0 says so.
5. Run the full ctest + tests/verify/verify.py suite for the four builds afterwards.

Report in docs/team/AGENT6_REPORT.md (new section T6.2), with commands and output. Then stop.
