# Agent 13 task T13.1: real.exe in CPPWRF (src/init_real) to WRF identity

src/init_real (by Agent 4; not in the build) turns met_em files into the initial and boundary
state. Read it, docs/review/REVIEW_2026-10-13.md items N1-N4, and WRF 4.6.1
dyn_em/module_initialize_real.F + main/real_em.F (+ share/module_bc.F for wrfbdy).
1. Fix N1-N4: no hard-coded domain (nz, p_top, etac, hypsometric_opt, interval from the
   met_em times and the config), compute FNM/FNP/CF1..CFN1 and every field real_case.cpp reads,
   no log(0) path, FP64 option, no ncid leaks.
2. A wrfinput_d01/wrfbdy_d01 NetCDF writer (same variable names, dims and attributes as
   real.exe, enough for src/cases/real_case.cpp AND for WRF itself to read them).
3. Verification against real.exe: from ~/wrf_real/wps_run met_em files (the R0 case), produce
   wrfinput/wrfbdy and compare every variable with ~/wrf_real/run_R0_r8/wrfinput_d01 and
   wrfbdy_d01 (r8 real.exe) — report max abs/rel per variable; target roundoff in FP64. Then run
   CPPWRF from your files and from real.exe's files and compare the 1 h output.
4. CMake target (option CPPWRF_INIT_REAL) and a CLI tools/cppwrf_real (or a cppwrf subcommand),
   ctest test.
Ownership: src/init_real/, tests/init_real/ (Agent 4 handed it over). Rules: tools/heavy.sh, -j4,
nothing in the repo root, no git writes, never git stash.
Report: docs/team/AGENT13_REPORT.md.
