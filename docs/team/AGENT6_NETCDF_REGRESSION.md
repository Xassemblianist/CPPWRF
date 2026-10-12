# Agent 6 task T6.1: WRF-format NetCDF output, restarts, and a regression system

You are Agent 6 (Gemini) in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md,
docs/STATUS.md and src/main.cpp (read only) to see how output is written today
(cppwrf_io: run.json, diag.csv, snap_*.bin/.json).

## Ownership (only these paths)
src/io_netcdf/ (new library `cppwrf_io_netcdf`, built only when NetCDF-C is found), tests/io_netcdf/,
tools/regression/, docs/team/AGENT6_REPORT.md. Do not edit src/main.cpp, src/io/ or any other
file: write the exact integration snippet (a few lines for main.cpp) into your report for
Agent 1. Never use git write commands or sudo. If libnetcdf-dev is missing, stop and give me the
apt command.

## Part A: wrfout writer (src/io_netcdf/)
Write CPPWRF state to NetCDF files that WRF's tools read as WRF output: the same dimension
names (Time, south_north, west_east, bottom_top, *_stag, DateStrLen), variable names, units,
`MemoryOrder`/`stagger`/`description`/`units` attributes, and the global attributes that
wrf-python and ncview need (DX, DY, MAP_PROJ, CEN_LAT, TRUELAT1/2, STAND_LON, MOAD_CEN_LAT,
DT, BOTTOM-TOP_GRID_DIMENSION, ..., SIMULATION_START_DATE, WEST-EAST_GRID_DIMENSION, ...).
Variables: U, V, W, PH, PHB, T (theta - 300), MU, MUB, P, PB, QVAPOR (when present), ZNU, ZNW,
C1H/C2H/C1F/C2F/C3H/C4H/C3F/C4F, RDX, RDY, P_TOP, Times, XLAT/XLONG (idealized: 0), HGT.
The API takes host arrays + metadata (like VarView in src/io/output.hpp). It must not depend on
the dycore headers. Reference: a real wrfout from WRF (Agent 3 has one under ~/wrf_ref; Agent 4
under ~/wrf_real). Compare `ncdump -h` of yours and theirs and list every remaining difference.
Tests: (1) round trip of all variables through netCDF4-python; (2) wrf-python (`pip install
--user wrf-python` is fine; if it fails, record why) computes getvar("theta"), getvar("pressure"),
getvar("z") on our file and they match our own diagnostics to float32 precision.

## Part B: restart files
Design a restart format (NetCDF, all prognostic and the RK3 *_1/*_2 state needed for bitwise
continuation) and the read/write functions. Acceptance test (needs Agent 1 to wire it, so
prepare a standalone test with a fake state now): write -> read -> bitwise identical arrays.
Write the main.cpp integration snippet and the test plan for "run 600 s = run 300 s + restart
+ run 300 s, bitwise" into your report for Agent 1.

## Part C: regression system (tools/regression/)
A single command, `tools/regression/run.sh [--quick|--full] [--gpu]`, that:
1. builds the four configurations (cpu, cpu-dp, cuda, cuda-dp) in build-reg-*/ (separate from
   the team's build dirs), runs ctest in each;
2. runs every config in configs/ (quick = short run_time overrides; full = the real ones) on
   every available backend and precision;
3. runs tests/verify/verify.py check on each output, and compares every run against stored
   golden outputs (tools/regression/golden/<case>_<backend>_<prec>/: diag.csv + the last
   snapshot) with tolerances per precision; it also compares CPU vs GPU;
4. records ns/cell/step per run in tools/regression/perf_history.csv (date, git commit, case,
   backend, precision, value) and flags slowdowns > 10% against the previous entry;
5. writes one Markdown summary (tools/regression/last_report.md): PASS/FAIL per check, perf
   deltas, and the git commit tested.
`tools/regression/update_golden.sh` refreshes the golden files, and only on explicit request.
Make it robust: missing GPU => skip GPU rows with a note; a crash in one case must not stop the
others; exit code != 0 if anything fails.
Generate the first golden set from the current HEAD and record the commit in the report.

## Report: docs/team/AGENT6_REPORT.md (update after each part)
Files, commands with real output, `ncdump -h` diff lists, wrf-python results, the main.cpp
snippets for Agent 1, and the first regression report. Then stop.
