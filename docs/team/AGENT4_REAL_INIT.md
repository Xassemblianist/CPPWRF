# Agent 4 task T4.1: real-data initialisation (WPS + real.exe equivalent), differential-tested

Long task. You are Agent 4 (Gemini) in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md.
Today CPPWRF runs only idealized cases. Goal: build initial and lateral-boundary conditions
from real global-model data, the way WRF's real.exe does, and prove they match real.exe
field by field.

## Hard rules
1. Paths you own: ~/wrf_real/ (everything WRF/WPS: source, builds, data, runs); in the repo:
   src/init_real/ (new), tests/init_real/ (new), docs/team/AGENT4_REPORT.md. Edit nothing else.
   Do not touch ~/wrf_ref (Agent 3), src/phys or tests/phys (Agent 2), src/dyn, src/core (Agent 1).
2. Never use git write commands in the CPPWRF repo. Never use sudo. If a system package is
   missing (e.g. libjasper/libpng for ungrib GRIB2, libnetcdf-dev), STOP and list the apt
   command for the user.
3. Do not modify WRF/WPS Fortran sources. Configuration goes in namelists.
4. Downloads: print each URL and its size before downloading. Total budget: about 5 GB. Use
   official sources only (www2.mmm.ucar.edu for WPS geog data, NOAA NOMADS or the AWS open data
   bucket noaa-gfs-bdp-pds for GFS 0.25 deg GRIB2).
5. Every claimed result needs the command and its output.

## Part A: reference run with the real WRF (in ~/wrf_real)
1. Clone WRF v4.6.1 and WPS v4.6.0. Build WRF for em_real (serial is enough; smpar if easy),
   then WPS (serial, GRIB2 support).
2. Static data: the WPS "mandatory low resolution" geographic dataset.
3. Case: one GFS 0.25 deg cycle, 24 h at 6-hourly intervals (5 files), a recent date.
   Domain: 1 domain over Türkiye, Lambert conformal, dx = 15 km, e_we = 120, e_sn = 90,
   e_vert = 45, p_top_requested = 5000 Pa. Physics: mp_physics = 1 (Kessler), all other physics
   off (ra_*, bl_pbl, sf_* = 0, cu_physics = 0), so the reference stays close to what CPPWRF
   can run soon. Keep the default dynamics (hybrid_opt = 2 is v4's default: record it, and
   also produce a second wrfinput with hybrid_opt = 0, the one CPPWRF uses now).
4. Run geogrid, ungrib, metgrid, real.exe, then wrf.exe for 6 h (timed with /usr/bin/time -v,
   plus WRF's "Timing for main" lines). Keep met_em*, wrfinput_d01, wrfbdy_d01 and wrfout.

## Part B: C++ port of real.exe's core (src/init_real/)
Port the parts of dyn_em/module_initialize_real.F (and the routines it calls in
share/module_*.F, dyn_em/nest-free paths) that turn met_em fields into WRF's initial state:
- reading met_em (NetCDF; you may link libnetcdf here only: an optional CMake target
  `cppwrf_init_real`, built only when NetCDF is found);
- eta levels (auto-generated as real.exe does for the namelist above), hybrid coefficients
  c1..c4 (hybrid_opt 0 and 2), base state (mub, pb, phb, alb, t_init with the ideal
  base_pres/base_temp/base_lapse/iso_temp settings real.exe uses), dry surface pressure and mu,
  vertical interpolation of u, v, t, qv (vert_interp with the default options), the
  hydrostatic integration of ph and the al/p diagnosis, and the map factors, f, e, sina, cosa.
- lateral boundary data: compute the same boundary values and tendencies as wrfbdy_d01
  (spec_bdy_width = 5) for every 6-h interval.
Host-side C++20 is fine (this runs once at start-up). Output: a struct of plain arrays in
WRF's variable names and units (staggering as in wrfinput: U on west faces, V on south
faces, W/PH on full levels). This is NOT wired into the model yet: Agent 1 does that when
terrain support arrives. Document the struct in docs/team/AGENT4_REPORT.md.

## Part C: differential tests (tests/init_real/)
A program `compare_wrfinput` that runs your port on the met_em files and compares every
field with wrfinput_d01 (and wrfbdy_d01) from real.exe: max abs, max rel, RMS and location of
the worst point, per variable. real.exe stores float32, so expect differences of ~1e-6
relative. Anything larger needs an explanation (show the input values at the worst point).
Run it for both hybrid_opt = 0 and 2. Register a ctest that runs the comparison when the
reference files exist (skip otherwise).

## Report: docs/team/AGENT4_REPORT.md (update after each part)
Build steps and problems, the namelists used (full text of namelist.wps and namelist.input),
download list with sizes, WRF timings, the comparison tables, the struct definition, and open
issues. Then stop.
