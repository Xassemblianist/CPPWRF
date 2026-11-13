# Changelog

All notable changes of CPPWRF. Versions follow [Semantic Versioning](https://semver.org/): a change of the results of
a verified configuration, of a configuration key or of an output file is a new major version.

## Unreleased

### Added
- Verification tiers of options: `data/option_tiers.txt` lists option values merged before their whole-model WRF
  comparison as column_verified (bit-identical to WRF's Fortran in a column driver) or unverified (ported only).
  The model and cppwrf_real stop on such a value, naming option, value and tier, unless `[cppwrf] allow_unverified =
  true` (namelist.input: `&cppwrf allow_unverified = .true. /`, passed on by tools/wrf2cppwrf.py, which warns on
  them); with the switch each one is logged and the wrfout / wrfrst files get the global attribute
  `CPPWRF_UNVERIFIED_OPTIONS`. The table has no row: every option is verified, and no output byte changes. A new
  option's entry in this changelog names its tier.

### Fixed
- WRF's clock values on the fixed time step: the physics' xtime (radiation's coszen and hour angle, grid nudging),
  julian and the year (radiation, tmn_update, Noah-MP), solve_em's curr_secs2 / curr_mins2 (buckets, prec_acc_dt,
  grid nudging), clwrf's xtime and the history files' XTIME now come from the exact clock, n times the domain's
  rational time step (a nest's: the parent's / parent_time_step_ratio), as WRF's ESMF clock gives them and as the
  adaptive time step already did. Before, they were computed from (itimestep - 1) * dt, which differs in the last
  bit past one day (XTIME = days*1440 + seconds/60; e.g. dt 16 s with radt 10 min, from the radiation call of
  27.7 h on) and from the first steps on with a nest step that is a fraction of a second (a parent dt not divisible
  by parent_time_step_ratio). New suite cases P4D16 (30 h, dt 16 s, radt 10 min) and N3D50 (N3 with d01 dt 50 s,
  nest dt 50/3 s), CPU and CUDA FP64 at tolerance 0 (data port_af1_data.tar). Results of the suite's earlier cases
  are unchanged.

### Tools and documentation
- `cppwrf-post` (docs/POST.md): forecaster fields from wrfout files with wrf-python's methods (ported to numpy,
  identical to wrf-python on single precision files), written as CF-1.8 NetCDF and / or GRIB2 (Lambert, polar
  stereographic, Mercator and latitude-longitude grids; WMO parameters; simple packing, JPEG 2000 / CCSDS through
  ecCodes); a post.yaml / post.ini configuration.
- `cppwrf-verify-obs` (docs/VERIFY_OBS.md): a run against surface observations (IEM METARs, NCEI ISD or a CSV
  file), with continuous, wind and precipitation contingency scores per lead time and station, SVG plots, a
  Markdown summary, and the comparison of two runs.
- docs/FP32_ACCURACY.md: how close GPU FP32 is to WRF real*8 compared with stock WRF real*4 (R = FP32's difference /
  WRF real*4's, measured on eleven domains at 1 h and over 24 h / 72 h), its method and when to use FP32 or FP64.
- tools/verify/fp32_accuracy.py: max |d|, RMSE, relative L2 and bias of any two sets of wrfout files per frame and
  variable (surface fields, accumulated and hourly rain with category scores, 3-D state, hydrometeors), and the report
  of the ratio between pairs with growth figures (SVG).
- tools/bench/colab_fp32.sh (with fp32_body.sh, fp32_gpu.sh, fp32_run.py): the study on one Colab runtime: WRF
  v4.8.0 real*4 and real*8 built from the release source, CPPWRF CUDA FP32 and FP64, twelve configurations
  (1 h; 24 h and 72 h from the suite's long reference inputs), statistics on the runtime.
- The FP32 cases real_acm7_fp32_1h, real_morr_fp32_1h and real_skv_fp32_1h have measured tolerances (about 3x the
  values of the study); real_skv_fp32_1h no longer lists TSK, which tests/verify/compare_real.py does not compare.

## 1.0.0

The first release: a C++20 / CUDA port of WRF-ARW v4.8.0 whose CPU FP64 and GPU FP64 output is bit-identical to WRF
v4.8.0 real*8 for every verified configuration. The release notes ([docs/RELEASE_NOTES.md](docs/RELEASE_NOTES.md))
give the exact verified scope.

### Model
- Dynamical core: RK3 split-explicit time integration with acoustic substeps, hybrid vertical coordinate
  (`hybrid_opt` 0 / 2), `hypsometric_opt` 1 / 2, map projections (Lambert, Mercator, polar stereographic), terrain,
  Coriolis and curvature terms; 5th / 6th-order horizontal and 3rd / 4th-order vertical advection, positive-definite
  moisture, scalar and TKE advection; specified, open, periodic and symmetric lateral boundaries; `w_damping`,
  `damp_opt` 1-3; the adaptive time step for single-domain real-data runs.
- Diffusion: `diff_opt` 1 and 2 with `km_opt` 1-4 (constant K, 1.5-order TKE, 3-D and 2-D Smagorinsky),
  6th-order diffusion (`diff_6th_opt` 1, 2).
- Microphysics: Kessler (1), WSM6 (6), Thompson (8), Morrison two-moment (10, `morr_rimed_ice` 0 / 1),
  aerosol-aware Thompson (28); radar reflectivity (`do_radar_ref`).
- Cumulus: Kain-Fritsch (1), Grell-Freitas-Li (3), Tiedtke (6), New Tiedtke (16); cumulus clouds in the radiation
  (`cu_rad_feedback`).
- PBL and surface layer: YSU (1), MYJ (2), MYNN-EDMF (5) with its options, ACM2 (7); revised MM5 (1), Eta (2),
  MYNN (5) and Pleim-Xiu (7) surface layers; fog deposition (`grav_settling`); gravity wave drag (`gwd_opt` 1).
- Land surface: 5-layer thermal diffusion (1), Noah (2), Noah-MP v5.1.2 (4, default options); fractional sea ice,
  SST update (`sst_update`, wrflowinp).
- Radiation: RRTMG LW and SW (4), RRTM LW (1), Dudhia SW (1); McICA (`cldovrlp` 0-5), CAM ozone and greenhouse
  gases.
- Nests: static one-way and two-way nests (feedback, smoothing, terrain blending), several nests and nests in nests.
- Four-dimensional data assimilation: grid analysis nudging (`grid_fdda = 1`, wrffdda_d0N).
- Tiles: one domain as P x Q tiles (`run.tiles`) with the same bits as without tiles.
- Diagnostics: accumulation buckets (`bucket_mm`, `bucket_J`), precipitation windows (`prec_acc_dt`),
  `output_diagnostics` (auxhist3).
- Pre-processing: `cppwrf_real` (WRF's real.exe: wrfinput, wrfbdy, wrflowinp from WPS met_em files) and the port of
  WRF's ideal.exe for `em_hill2d_x`, `em_quarter_ss`, `em_squall2d_x/y`, `em_grav2d_x`.
- I/O: WRF's wrfout, wrfrst and auxhist3 NetCDF files (the WRF v4.8.0 I/O schema, `iofields_filename`, file names,
  frames per file, NetCDF classic), NetCDF-4 compression on host threads with the same bytes as HDF5's filter;
  restarts from WRF's or CPPWRF's restart files; CPPWRF's own restart files record the format version, the commit
  and the options.
- Backends: OpenMP CPU and CUDA GPU from one source, FP64 and FP32; on the GPU, CUDA graphs for the time step.

### Tools
- `tools/wrf2cppwrf.py`: WRF `namelist.input` to CPPWRF configuration, with `physics_suite`, WRF's check_a_mundo
  resets and an error for every option without a port and every unknown key.
- `examples/quickstart/run.sh`: GFS -> WPS -> cppwrf_real -> cppwrf -> wrfout with WRF's default domains and namelist.
- Verification: `tests/verify/wrf_suite.py` (model runs against WRF at tolerance 0), the WRF output files tests,
  the differential tests against WRF's compiled Fortran (`ctest -L diff`), unit tests, and the Colab / Google Cloud
  drivers in `tools/colab` and `tools/gcp`; `tools/bench` (the benchmark against WRF v4.8.0 in docs/PERF.md).

### Not included
MPI runs, moving nests, vertical nest refinement, ndown.exe, I/O forms other than NetCDF, and the schemes and options
listed in docs/USER_GUIDE.md ("Rejected Options and Why"); each is rejected with an error.
