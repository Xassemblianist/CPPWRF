# Agent 12 task T12.1: namelist.input translator and user guide

Users come with a WRF namelist.input. Write tools/wrf2cppwrf.py: reads a WRF namelist.input
(f90nml-free parser of your own: Fortran namelist syntax, multiple domains -> domain 1 only, with
a warning), and writes a CPPWRF config (.ini) with every option CPPWRF supports mapped
(read src/cases/model.cpp, src/cases/real_case.cpp, src/main.cpp, src/dyn/*.hpp parameter structs
and configs/*.ini to find every key and its WRF meaning), and a clear error/warning for every
namelist option that CPPWRF does not support or that differs from the WRF default (never silently
drop one). Test it on ~/wrf_real/run_*/namelist.input (R0..R3, P1..P4, mp6) and ~/wrf_ref/run_*:
for the real runs, the generated .ini must reproduce configs/real_r0.ini ... (compare key by key)
and run (tools/heavy.sh build-a0-cpu/cppwrf <ini> for 2 steps). Unit tests in
tests/tools/test_wrf2cppwrf.py.
Then docs/USER_GUIDE.md: build (CPU/CUDA/FP64), running ideal and real cases (wrfinput/wrfbdy from
WPS + real.exe), the config keys (generated table from the code, with WRF equivalents),
outputs (snapshots, wrfout, restart), verification commands (tests/verify), performance notes.
Accuracy: every statement checked against the code; cite file:line in a hidden comment.
Ownership: tools/wrf2cppwrf.py, tests/tools/, docs/USER_GUIDE.md. No git writes.
Report: docs/team/AGENT12_REPORT.md.
