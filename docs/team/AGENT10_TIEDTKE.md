# Agent 10 task T10.1: New Tiedtke cumulus (cu_physics = 16) and Tiedtke (cu_physics = 6)

WRF's tropical and CONUS physics suites use the (new) Tiedtke scheme. Port
phys/module_cu_ntiedtke.F (cu_physics 16) first, then module_cu_tiedtke.F (6), including their
init routines (tables) and the cumulus_driver glue for these options
(phys/module_cumulus_driver.F: what is passed in, how the tendencies come out, stepcu/cudt).
Ownership: src/phys/cu_ntiedtke/, src/phys/cu_tiedtke/, tests/phys/cu_ntiedtke/, tests/phys/cu_tiedtke/ (new).
Method (binding, as docs/team/AGENT2_PHYSICS.md): statement-by-statement C++20 port as single-source column kernels (CPPWRF_HD structs launched with column_for/parallel_for; read src/phys/bl_ysu/ysu.hpp and src/phys/cu_kf/kf.hpp as the house style; C++ k = Fortran k - 1). No fixed 128-level local arrays sized beyond need: use kMaxLevels guarded by a runtime nz check at launch, and keep per-column local arrays to the minimum (review item S3 in docs/review/REVIEW_2026-10-13.md). All literals as real(...) (S4). Tables on the device (no host pointers in device code, S5). Differential test against the ORIGINAL WRF 4.6.1 Fortran (~/wrf_real/WRF-4.6.1-r8/phys, compiled unmodified with gfortran -fdefault-real-8, driven with identical inputs): PASS = bit-exact or <= 1e-12 rel in CPU FP64 AND CUDA FP64, plus an FP32 build that compiles and runs; inputs from a realistic column set (take columns from ~/wrf_real/run_P4_r8 wrfout or the R0 wrfinput) incl. edge cases. Every build/run via tools/heavy.sh, -j4; nothing in the repo root; no git writes; never git stash.
Report: docs/team/AGENT10_REPORT.md (per routine: ported lines, test results, skipped branches).
