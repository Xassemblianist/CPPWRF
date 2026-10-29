# New Tiedtke (cu_physics = 16) column checks

The port (`src/phys/cu_ntiedtke/ntiedtke.hpp`) is checked against WRF's own object file.

1. Dump the scheme's column inputs and outputs from a CPU build (one thread):

       CPPWRF_NTIEDTKE_DUMP=nt.bin CPPWRF_NTIEDTKE_DUMP_STEP=2 OMP_NUM_THREADS=1 \
         build/cppwrf configs/real_p7_r8.ini --set run.run_time=120

   Each record holds `qfx, hfx, slimsk, delt, dx, nz`, the 11 level arrays and the 2 half-level arrays
   passed to `cu_ntiedtke_run` (top to bottom), then the 6 updated arrays and the rain. `km = 44` is
   fixed in the harness.

2. Replay the inputs through WRF's `cu_ntiedtke_run` (the objects of the r8d8 build that made the P7
   references, `WRF_R8D8_DIR`; `tests/phys/wrf_fortran.sh` links the harness to them):

       . tests/phys/wrf_fortran.sh; wrf_need r8d8; wrf_fortran harness tests/phys/cu_ntiedtke/harness.f90
       ./harness nt.bin                 # columns whose u, v, t, qv, qc, qi or rain differ (any bit)

   `run_test.sh` (ctest `diff_phys_cu_ntiedtke`) does both for steps 2 and 4 of P7: every column identical.

3. `sensitivity.f90 nt.bin <column> <eps>` applies 500 random relative perturbations of size eps to
   one column's inputs and reports the largest change of the convective temperature change. At step 4
   column 1524 (i = 95, j = 12) moves by 1.3e-4 K under 1e-15 noise: the scheme's thresholds turn
   roundoff into O(1e-6) relative differences, which then grow to ~1e-6 (U) over an hour.
