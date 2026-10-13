# Agent 6 task T6.3: wrfinput / wrfbdy reader, golden files, regression additions

Your T6.2 work is committed (8eca021). Agent 0 changed the dycore (3801f2a: map factors,
Coriolis, curvature terms, terrain BC), so most golden files are out of date.

1. Regenerate ALL golden files from HEAD (authorized), CPU and CUDA, FP32 and FP64; commit
   nothing (Agent 0 commits). Report the old-vs-new difference per case in one line each.
2. Add to tools/regression: tests/verify/dycore_checks.py (all four builds), configs/straka_wrf.ini
   checked with `verify.py wrf ~/wrf_ref/run_straka_d1/wrfout_d01_0001-01-01_00:00:00`, and
   configs/squall2d_wsm6.ini with verify.py check squall2d. Every job through tools/heavy.sh.
3. Reader (src/io_netcdf, your ownership): read_wrfinput(path) and read_wrfbdy(path) returning
   plain host structs (std::vector<double>, explicit dims) with every field the dynamics needs:
   ZNW, ZNU, DN, DNW, RDN, RDNW, FNM, FNP, CF1-3, CFN, CFN1, C1H..C4H, C1F..C4F, P_TOP, MUB,
   MU, PB, PHB, PH, T, T_INIT, ALB, AL, P, U, V, W, QVAPOR (and the other moist species if
   present), HGT, MAPFAC_MX/MY/UX/UY/VX/VY, F, E, SINALPHA, COSALPHA, XLAT, XLONG, DX, DY, DT,
   global attributes (HYBRID_OPT, ...); for wrfbdy all *_BXS/BXE/BYS/BYE and *_BTXS... with
   times. Document the C++ index order (C++ k = Fortran k - 1; i fastest). Unit test against
   ~/wrf_real/wrf_run R0 files (values spot-checked with ncdump / netCDF4-python).
Report: docs/team/AGENT6_REPORT.md, section T6.3. Then stop.
