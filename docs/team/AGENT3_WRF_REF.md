# Agent 3 task T3.3: build real WRF and run the Straka case (em_grav2d_x)

You are Agent 3 in the CPPWRF project (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md.
Goal: a reference run of the original Fortran WRF on the same Straka density current that
CPPWRF runs, to get (a) WRF's own front position and min theta', and (b) WRF's wall time on
this machine. CPPWRF is never compared against anything except measured numbers.

## Hard rules
1. Work ONLY in ~/wrf_ref/ (WRF source, build, runs) plus, inside the repo, scripts/ and
   docs/team/AGENT3_REPORT_T33.md. Do not touch anything else in the repo.
2. Never run any git command that writes inside /home/ostriquetrum/CPPWRF. (Running git clone
   inside ~/wrf_ref is fine.)
3. Never use sudo. If a system package is missing, STOP and list the exact apt command the
   user must run. Expected to be installed already: gfortran, libnetcdff-dev, netcdf-bin,
   csh, m4, python3-netcdf4.
4. Do not modify WRF Fortran source code. Configuration choices go into namelist.input only.
5. Never claim a number you did not measure. Paste real command output.

## Steps
1. Get WRF v4.6.1: `git clone --depth 1 --branch v4.6.1 https://github.com/wrf-model/WRF ~/wrf_ref/WRF`.
2. Configure a **serial** GNU build (gfortran/gcc, the "serial" option in ./configure,
   nesting 0). NETCDF must point to the directory containing include/netcdf.inc and the
   netcdff library. On Debian: find them with `nc-config --prefix` and `nf-config --all`.
   If they are in different prefixes, create ~/wrf_ref/netcdf with symlinks for include/ and lib/.
   Compile: `./compile em_grav2d_x >& compile.log`. Success = main/ideal.exe and main/wrf.exe
   exist. On failure, show the first real error in compile.log (grep -n "Error" | head).
3. Run in ~/wrf_ref/run_straka/ (copy test/em_grav2d_x/*). Keep the case's own
   namelist.input and input_sounding unchanged, except: run_seconds = 900,
   history_interval_s = 300. Record every namelist value in the report (dx, e_we, e_vert,
   time_step, km_opt, diff_opt, khdif, kvdif, h/v adv orders, epssm, smdiv, emdiv,
   hypsometric_opt, top_lid, boundary conditions). Run ideal.exe then wrf.exe; time wrf.exe with
   `/usr/bin/time -v`. Also write down the wall time WRF itself reports per time step
   (rsl / stdout "Timing for main").
4. If the serial build works, also configure and build an OpenMP ("smpar") build in a
   separate copy (~/wrf_ref/WRF_omp), run with OMP_NUM_THREADS=12, and time it the same way.
5. scripts/compare_wrf_straka.py (python3, numpy, netCDF4):
   `python3 scripts/compare_wrf_straka.py <wrfout file> <cppwrf run dir>`.
   From wrfout at t = 900 s: theta' = T (perturbation potential temperature) minus its
   initial value far from the bubble (or the T at time 0 at the right edge; state which).
   Report min theta' and the front position = outermost x where theta' <= -1 K at the
   lowest level, linearly interpolated, measured from the bubble centre (in em_grav2d_x the
   bubble sits at the left, symmetric boundary; check the initial field to confirm). Read the
   CPPWRF run with scripts/cppwrf_io.py and compute the same two numbers the same way
   (CPPWRF uses a full domain with the bubble in the middle: measure from the centre).
   Print one table: model | dx | front_km | min_th_pert | wall_s.
   Also save out/plots/wrf_vs_cppwrf_straka.png: theta' at 900 s, WRF on top, CPPWRF below
   (CPPWRF cropped to the right half), same contour levels as plot_case.py.
6. Run it with the WRF output and out/a0_straka_dp (FP64) and out/a0_straka (FP32).

## Report: docs/team/AGENT3_REPORT_T33.md (max 50 lines)
Build commands and result, the full namelist values from step 3, timing output
(/usr/bin/time and WRF's "Timing for main" lines: first, typical, total), the comparison
table, and every difference you noticed between WRF's em_grav2d_x setup and CPPWRF's
configs/straka.ini (domain, BCs, diffusion, advection order, eta levels, time step,
sound steps). Then stop.
