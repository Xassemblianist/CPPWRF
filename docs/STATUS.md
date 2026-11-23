# CPPWRF: status and verification

Version 1.3.0 (docs/RELEASE_NOTES.md: the verified scope of the release).

CPPWRF reproduces WRF-ARW v4.8.0 statement by statement. This page lists what is implemented, how it
is verified against WRF, and the current results. All numbers come from `tests/verify/wrf_suite.py`
(at 318159a, version 1.3.0: 1153 cases, 618 CPU FP64, 404 CUDA FP64, 43 CUDA FP32, 44 CPU FP32 (`cpu_sp`, against
WRF real*4) and 44 CUDA FP32 identity (`cuda_sp_id`), and 18 restart configurations. CPU CI at 318159a reported 527
passes, including the 464 CPU FP64 cases outside `--slow`, 18 restart checks, the legacy `straka_wrf` RMS check
against WRF real*4 at 1e-4 K, and the 44 `cpu_sp` cases. The `ideal_grav2d_x` cases verify the Straka density current
against WRF real*8 at tolerance 0 on CPU and CUDA. The 154 slow CPU cases are the 9 runs of 24 h, 30 h and 72 h and
145 Noah-MP option cases. For 1.1.0 the Colab G4 reported 338 CUDA FP64 passes, including all 319 cases at tolerance 0,
18 GPU restart checks and the legacy `straka_wrf_cuda` RMS check. The 43 CUDA FP32 cases with
restart_p1_fp32_cuda gave 44 runs within their tolerances. The 1.2.0,
1.2.1, 1.2.2, 1.2.3 and 1.3.0 GPU runs (1.3.0: all 423 CUDA FP64 checks run) are in docs/RELEASE_NOTES.md) and `tests/verify/run_wrf_files_test.sh` (WRF output files).

## Implemented

| Component | WRF options | Tier |
|---|---|---|
| Dynamics | RK3 split-explicit, adaptive time step (use_adaptive_time_step, single-domain real-data runs: target_cfl, target_hcfl, max_step_increase_pct, starting / max / min_time_step, step_to_output_time, restarts), hybrid_opt 0/2, hypsometric_opt 1/2, map factors, Coriolis and curvature, terrain, specified (spec_zone 1, relax_zone 4), open, periodic and symmetric lateral boundaries (symmetric: no wind across the walls), w_damping, damp_opt 1, 2 and 3, h_adv_order 5/6, v_adv_order 3/4, positive-definite moisture, scalar and TKE advection (h_adv_order 5 and 6), monotonic (2) and WENO (3, 4) moist / scalar / TKE advection (moist_adv_opt, scalar_adv_opt, tke_adv_opt), WENO momentum advection (momentum_adv_opt 3) | verified |
| Diffusion | diff_opt 1 with km_opt 1 (constant K), 3 (3-D Smagorinsky) and 4 (2-D Smagorinsky); diff_opt 2 with km_opt 2 (1.5-order TKE, advected TKE); 6th-order diffusion (diff_6th_opt 1, 2) | verified |
| Microphysics | Kessler (1), WSM6 (6, with effective radii), Thompson (8), Morrison two-moment (10: morr_rimed_ice 0 / 1, progn 0, real data), aerosol-aware Thompson (28: aer_init_opt 0, wif_input_opt 0 / 1; single domain, not with MYNN), P3 with one ice category (50, 51: predicted droplet number; real data, single domain, not with do_radar_ref 1 or MYNN), NSSL 2-moment with hail, CCN and graupel / hail volume (18 with the default nssl_* options, and 17; real data, single domain) | verified |
| Microphysics | WSM3 (3, simple ice: qc and qr hold the ice and the snow below 0 C) and WSM5 (4, with qi and qs), both with the effective radii for RRTMG; real data, not with do_radar_ref 1: module_mp_wsm3.F / module_mp_wsm5.F bit for bit in the column driver tests/phys/mp_wsm35 (wsm3init / wsm5init constants and every output); suite cases W3 / W5 | verified |
| Microphysics | WDM6 (16, `hail_opt` 0 / 1) and WDM5 (14): double-moment warm rain with the CCN, cloud and rain numbers QNCCN / QNCLOUD / QNRAIN (start_em's ccn_conc fill, the first step's CCN reset, flow_dep_bdy_qnn at the boundaries), the effective radii for RRTMG; real data, one domain, not with do_radar_ref 1: module_mp_wdm6.F / module_mp_wdm5.F bit for bit in the column driver tests/phys/mp_wdm; suite cases D6, D6H, D6C, D5 | verified |
| Cumulus | Kain-Fritsch (1), multi-scale Kain-Fritsch (11: scale-aware trigger, entrainment and closure, cloud fractions in the radiation, momentum transport RUCUTEN / RVCUTEN; aercu_opt 0, kf_edrates 0; column driver tests/phys/cu_mskf), Grell-Freitas-Li (3, deep and shallow), Tiedtke (6), New Tiedtke (16), Betts-Miller-Janjic (2: deep and shallow convection, CLDEFI, BMJINIT's tables; column driver tests/phys/cu_bmj) with bmj_rad_feedback (its convective clouds in the radiation) | verified |
| Cumulus | Grell-3D ensemble (5: G3DRV, CUP_enss_3d with its 144-member closure ensemble and the 3 x 3 neighbour forcing, ishallow 0 / 1, cugd_avedx 1 / 3, clos_choice 0-16, CONV_GRELL_SPREAD3D; patch driver tests/phys/cu_g3 bit-identical with WRF's module_cu_g3.o). Grell-Devenyi (93) is not ported; suite cases G3P8, G3P4, G3P4A3 | verified |
| PBL, surface layer, land surface | YSU (1), MYJ (2), ACM2 (7, over the surface layer 1 or 7), MYNN-EDMF (5, icloud_bl 0 / 1, with the surface layers 1, 2 and 5; bl_mynn_tkeadvect, bl_mynn_cloudpdf -2..2, bl_mynn_mixlength 0..2, bl_mynn_closure 2.5 / 2.6 / 3.0, bl_mynn_edmf 0 / 1, bl_mynn_edmf_mom, bl_mynn_edmf_tke, bl_mynn_cloudmix, bl_mynn_mixqt, bl_mynn_ess, bl_mynn_mixscalars, bl_mynn_output 0 / 1, tke_budget 0 / 1), MYNN surface layer options sf_mynn_sfcflux_land 0..3 and sf_mynn_sfcflux_water 0..4, fog deposition (grav_settling 1, 2), revised MM5 surface layer (1), Eta surface layer (2), MYNN surface layer (5), Pleim-Xiu surface layer (7, with ACM2 and Noah), 5-layer thermal diffusion (1, ifsnow 0 / 1; also in cppwrf_real), Noah LSM (2, including glacial and sea-ice points; usemonalb and rdlai2d: background albedo and LAI from wrfinput), Noah-MP v5.1.2 (4; dveg 1-9, opt_crs, opt_btr, opt_runsrf / opt_runsub 1-4, 6, 7, opt_sfc, opt_frz, opt_inf, opt_rad, opt_alb 1 / 2, opt_snf, opt_tbot, opt_stc, opt_gla, opt_rsf, opt_tksno, opt_compact, opt_scf, opt_wetland 0 / 1, soiltstep), fractional sea ice (fractional_seaice = 1), SST update (sst_update = 1, wrflowinp), skin SST (sst_skin = 1: warm layer and cool skin, with Noah), deep soil temperature update (tmn_update = 1, lagday, with Noah), gravity wave drag (gwd_opt = 1, GWDO) | verified |
| Land surface | RUC LSM (3, 6 or 9 soil levels; land, snow, sea ice, land ice and water points; USGS-RUC / MODI-RUC tables; with sfclayrev or MYJSFC; not with mosaic_lu / mosaic_soil, fractional_seaice, sst_update, MYNNSFC or nests; input from WRF's real.exe): RUCLSMINIT, LSMRUC and SFCDIAGS_RUCLSM bit for bit in the column driver tests/phys/lsm_ruc; suite cases RUC4 (9 levels) and RUC4J (MYJSFC + MYJ, 6 levels), restarted at 30 min | verified |
| PBL | BouLac (8, Bougeault-Lacarrere TKE scheme, over the surface layer 1 or 2; without the BEP urban terms, as CPPWRF has no BEP urban model): module_bl_boulac.F bit for bit in the column driver tests/phys/bl_boulac; suite cases BL1, BL4, BL9 | verified |
| PBL and surface layer | QNSE PBL (4) with the QNSE surface layer (sf_sfclay_physics 4), mfshconv = 0 or 1 (the EDKF mass-flux shallow convection, WRF's default), fractional_seaice 0: module_bl_qnsepbl.F and module_sf_qnsesfc.F (with QNSESFCINIT's tables) bit for bit in the column driver tests/phys/bl_qnse, module_bl_mfshconvpbl.F's MFSHCONVPBL in tests/phys/bl_mfshconv; suite cases Q1, Q4 (mfshconv 0) and Q1M, Q4M (mfshconv 1) | verified |
| Radiation | RRTMG LW and SW (4), RRTM LW (1, with RRTMG or Dudhia SW), Dudhia SW (1, with RRTMG or RRTM LW; swrad_scat), CAM ozone and greenhouse gases, cal_cldfra1, McICA (cldovrlp 1-5) and clear sky only (cldovrlp 0), cumulus clouds in the radiation (cu_rad_feedback: Kain-Fritsch, Grell-Freitas; no effect with Tiedtke / New Tiedtke as in WRF) | verified |
| Terrain options | YSU topographic wind correction (topo_wind 1: start_em's CTOPO / CTOPO2 from VAR_SSO and the terrain Laplacian, stops without VAR_SSO as WRF; 2: from VAR), slope-dependent surface shortwave (slope_rad 1, TOPO_RAD_ADJ_DRVR) and terrain shadows (topo_shading 1 with shadlen, toposhad); real data, one domain, fixed time step; topo_shading not with tiles or MPI ranks: column test tests/phys/topo (radconst, toposhad, TOPO_RAD_ADJ_DRVR bit for bit with WRF's objects); suite cases TW1, TW2, TW4 (1 h) and SR1-SR4 (5 h through sunrise), restarted at 30 / 150 min, tiles | verified |
| Urban canopy | single-layer urban canopy model (sf_urban_physics = 1, module_sf_urban.F) with the Noah LSM: URBPARM.TBL (use_wudapt_lcz = 0) and URBPARM_LCZ.TBL (use_wudapt_lcz = 1) with their default options, one domain, no restart; column test tests/phys/sf_urban (urban_param_init and SUBROUTINE urban bit-identical to WRF) | verified |
| Nudging | grid (analysis) nudging, grid_fdda = 1 (PSUFDDAGD: guv, gt, gq, if_no_pbl_nudging_uv / t / q, if_zfac_* / k_zfac_*, fgdt, fgdtzero, if_ramping / dtramp_min, gfdda_end_h, gfdda_interval_m, per domain; wrffdda_d0N from real.exe or cppwrf_real; restarts, nests) | verified |
| Nesting | static one-way and two-way nests (interp_fcn_sint, bdy_interp1, copy_fcn feedback, sm121/smdsm smoothers, blend_terrain, adjust_tempqv) | verified |
| Namelist | physics_suite CONUS and tropical (the -1 options), in tools/wrf2cppwrf.py and in the WRF I/O schema | verified |
| Diagnostics | accumulation buckets bucket_mm and bucket_J (I_RAINC, I_RAINNC, I_ACSWUPT ... I_ACLWDNBC) and the precipitation windows prec_acc_dt (PREC_ACC_C, PREC_ACC_NC, SNOW_ACC_NC, per domain), output_diagnostics = 1 (T2MIN ... RAINNCVSTD over auxhist3_interval, the auxhist3 files), with the column physics; radar reflectivity (do_radar_ref); nwp_diagnostics is not ported | verified |
| I/O | reads wrfinput_d0N / wrfbdy_d01; writes WRF v4.8.0 wrfout_d0N and wrfrst_d0N files (WRF I/O schema, iofields_filename, history_outname / rst_outname, frames_per_outfile, use_netcdf_classic) and its own restart files; native binary output; cppwrf_real (real.exe) | verified |
| Domain decomposition | one domain and its nests as P x Q patches, each with its own memory (its points and a halo): tiles (`run.tiles`, host threads with their own fields and CUDA streams) or MPI ranks (a `CPPWRF_MPI` build under `mpirun`, `run.ranks`); halo exchanges, layout-independent reductions, one writer of the output; the same bits for every worker count and layout (docs/MPI.md: 13 layouts of up to 8 ranks on eight cases, 53 suite cases in three layouts, up to 48 ranks against the one-process run; suite cases `*_tPxQ`). Not run: several GPUs, several nodes | verified |
| Ideal cases | WRF's ideal.exe for em_hill2d_x, em_quarter_ss, em_squall2d_x/y, em_grav2d_x; CPPWRF's own analytic tests (rest state, warm bubble, Straka, inertia-gravity wave, 2-D squall line) | verified |

Tier: every row is **verified**, its whole-model output identical to WRF's at tolerance 0 (CPU FP64 and CUDA FP64,
the suite below). A port may be merged before that proof at a lower tier: **column_verified** (the scheme's routines
match WRF's Fortran bit for bit in a column driver, tests/phys) or **unverified** (ported and compiling only, where a
column driver is impossible). Such options are listed in `data/option_tiers.txt`, the one table the model, cppwrf_real
and tools/wrf2cppwrf.py read; the model stops on them unless `[cppwrf] allow_unverified = true` (namelist.input:
`&cppwrf allow_unverified = .true. /`), and a run with the switch logs a warning per option and writes the global
attribute `CPPWRF_UNVERIFIED_OPTIONS` (e.g. `mp_physics=4:column_verified`) into its wrfout and wrfrst files. The
table has no row. A whole-model proof removes the option's row.

## Verification method

1. **Column and kernel tests** (`tests/phys`, `tests/dyn`, `tests/nest`, ctest label `diff`): each scheme
   and kernel runs on the same inputs in C++ and in WRF v4.8.0's own compiled code: the test's Fortran driver
   is linked to the objects of the WRF build that made the model references (`tests/phys/wrf_fortran.sh`;
   `WRF_DIR` = the -r8 build for Kessler, WSM6, KF, Noah, Noah-MP, MYJ, the revised MM5 surface layer,
   YSU, the diffusion kernels, the nest numerics and the physics suite driver; `WRF_R8D8_DIR` = -r8
   -fdefault-double-8 for Thompson, MYNN and the cumulus replays). No WRF source is copied or compiled by
   a test. Every output must be bit-identical in CPU FP64 (tolerance 0, compared as tests/verify/exact.py:
   NaN equals only NaN, -0 differs from +0; an empty or truncated output fails). This covers Kessler
   (one and several fall steps), WSM6 (hail_opt 0 and 1, effective radii), Thompson (its tables and the
   table files it writes), YSU (ysu_topdown_pblmix 0 and 1, with and without the cloud tendencies), MYJ and
   the Eta surface layer, the revised MM5 surface layer (isfflx 0/1, isftcflx 0-2, iz0tlnd 0-2, kim_tofd),
   Noah (opt_thcnd 1 and 2) and its sea-ice path, Noah-MP, MYNN-EDMF and the MYNN surface layer (default
   options; the other options are covered by the model runs, 2.), Kain-Fritsch (its tables bit by bit, both
   triggers, three moisture sets), the diffusion and damping kernels (km_opt 1-4), the nest interpolation
   (the model's KSintView3D and the host references; feedback, blending, the nest boundary and coupling are
   covered by the nest model runs N1-N3), and the physics suite driver. Grell-Freitas-Li and New Tiedtke are
   checked by replaying the model's own column inputs through WRF's objects (the scheme call; their pre- and
   post-processing by the model runs P8 and P7). RRTMG has no ctest (its checks need WRF debug dumps); the
   model runs P3 and P4 cover it. Run them with `cmake -DCPPWRF_DOUBLE=ON -DCPPWRF_ENABLE_CUDA=OFF
   -DCPPWRF_DIFF_TESTS=ON` and `ctest -L diff`, with `WRF_DIR`, `WRF_R8D8_DIR` and `CPPWRF_WRF_RUNS` set;
   without them a test is skipped (77). Every full VM suite runs them (`tools/gcp/run_diff_tests.sh`
   with `diff_data.tar`: the WRF objects and the input subsets; a skipped test fails the suite);
   `tools/gcp/gcp_difftests.sh --faults DIR` and `tools/colab/colab_suite.sh --diff-only [--tests REGEX]
   [--faults DIR]` run them alone, with deliberate faults that must be caught. WRF's side is computed on the test
   machine, so the Fortran drivers get glibc libmvec's vector log / log10 as on Intel
   (`tests/phys/libmvec_intel.cpp`, linked by `wrf_fortran.sh`; see below): the tests give the same bits on Intel
   and AMD CPUs.
2. **Model runs against WRF** (`tests/verify/wrf_suite.py`): WRF v4.8.0 and CPPWRF are run from the same
   wrfinput/wrfbdy files (119 x 89 x 44, 15 km, dt 60 s; nests 120 x 90 x 44 at 5 km). The CPU FP64 cases
   compare CPPWRF's wrfout files with WRF's at tolerance 0 (tests/verify/exact.py: NaN equals only NaN, -0
   differs from +0) in every variable the reference holds, every reference frame up to the case's time, types and
   attributes, and check that the configuration sets the options of the reference run's namelist.input; so do the
   CUDA FP64 cases. The CPU FP32 cases (`cpu_sp`) and their GPU twins (`cuda_sp_id`, the default CUDA FP32 identity
   build) compare with WRF real*4's history files in the same way, at tolerance 0 (section "FP32 against WRF real*4"
   below). The fast CUDA FP32 cases (`cuda`, `-DCPPWRF_CUDA_FP32_IDENTITY=OFF`) compare fields by relative L2 over the
   domain. The comparators' own test
   (tests/verify/selftest.py: one ulp, NaN, -0, a missing variable or frame must fail) runs before the VM suite.
3. **WRF output files** (`tests/verify/run_wrf_files_test.sh`, ctest `wrf_files_vs_wrf`): the
   wrfout and wrfrst files of the CPU FP64 runs, and the runs resumed from WRF's restart files,
   compared with WRF's files at tolerance 0 (values, variables, attributes, global attributes).
   With `CPPWRF_FILES_TRANSLATED=1` the CPPWRF configuration is generated from the WRF run's
   `namelist.input` by `tools/wrf2cppwrf.py`.
4. **Checkpoint dumps** (`tests/verify/wrf_dycore_dumps`): a WRF build that writes intermediate arrays
   inside solve_em and the physics drivers, used to locate the first differing operation.
5. **Quick start** (`tools/colab/colab_quickstart.sh`): the builds and runs of README.md and docs/USER_GUIDE.md from
   a clean source tree on a Colab G4 runtime (CPU FP64, GPU FP32 and FP64 builds, ctest, the translator's tests, the
   Straka and em_quarter_ss cases, `examples/quickstart/run.sh` from GFS through WPS and cppwrf_real to wrfout), with
   the example's GPU FP64 wrfout files compared with the CPU FP64 ones at tolerance 0.

Bit identity on the CPU requires reproducing WRF's arithmetic exactly: the order of every sum and
product, constant folding done by the Fortran compiler, the halo refresh schedule, and the vector math
functions (glibc libmvec) that gfortran uses in vectorised loops. CPPWRF calls the same libmvec
functions for the same loop iterations (`cppwrf::vm` in src/core/backend.hpp). The physics reference
runs of the schemes with DOUBLE PRECISION variables (Thompson, MYNN, radar reflectivity, New Tiedtke,
Grell-Freitas, WENO advection, NSSL, P3) use WRF built with `-fdefault-real-8 -fdefault-double-8`, so that these
variables are 8-byte as in CPPWRF.

## Results

CPU, FP64 against WRF real*8 (tolerance 0: every compared value identical):

<!-- generated: tests/verify/wrf_suite.py --status-table docs/STATUS.md (rows: STATUS in tests/verify/cases) -->
| Case | Configuration | Duration |
|---|---|---|
| R1 | dynamics only | 5 steps |
| T1 | R1 with diff_opt 2 / km_opt 2 (TKE) | 5 steps |
| R1D6 | R1 with diff_opt 1 / km_opt 4, damp_opt 1, diff_6th_opt 2 | 10 min |
| P1 | sfclayrev + Noah + YSU | 1 h |
| A01 / A01V | P1 / V2 with usemonalb and rdlai2d (wrfinput ALBBCK and LAI; V2: snow-covered points start with ALBEDO = SNOALB) | 1 h |
| P5 | P1 + WSM6 + diff_opt 1 / km_opt 4 | 1 h |
| P2 | P1 + Kain-Fritsch | 1 h |
| P3 | P1 + RRTMG LW/SW (radt 1 min) | 3 steps |
| P4 | P2 + WSM6 + RRTMG (cloudy, effective radii) + diff_opt 1 / km_opt 4 | 1 h |
| A05 / A05C / A05G | P4 with cu_rad_feedback (Kain-Fritsch cloud fraction and water in RRTMG) / P4 with cldovrlp 0 (RRTMG without clouds) / P8 + WSM6 + RRTMG with cu_rad_feedback (Grell-Freitas QC_CU / QI_CU) | 1 h |
| A08 | P4 with bucket_mm 0.05, bucket_J 1e4 and prec_acc_dt 2 min (RAINC / RAINNC and the LW accumulations tip at the first step: I_RAINC, I_ACLW*; PREC_ACC_C, PREC_ACC_NC, SNOW_ACC_NC) | 3 min, 1 h |
| A09 | P4 with output_diagnostics = 1, auxhist3_interval 3 min (the auxhist3 files T2MIN ... RAINNCVSTD compared as wrfout) | 3 min, 1 h, 1 h restarted at 30 min |
| C01 | P6 with the aerosol-aware Thompson (mp_physics 28, aer_init_opt 0: thompson_init's QNWFA / QNIFA profiles and QNWFA2D emission) | 1 h |
| RADW / RADT | P4 (WSM6) / P6 (Thompson) with do_radar_ref 1 (REFL_10CM at the history times: radar_init's bins, Rayleigh scattering of rain, snow and graupel, melting snow and graupel as Maxwell-Garnett water-ice spheroids) | 3 min, 1 h |
| KESSKF | P4 with Kessler instead of WSM6 (Kain-Fritsch rain in Kessler's split sedimentation steps) | 1 h |
| DUD | V1 with the Dudhia shortwave and RRTMG longwave (00 UTC: sunrise over the Bay of Bengal, the whole domain sunlit within the hour) | 1 h |
| RRTM | V1 with the RRTM longwave and RRTMG shortwave | 3 steps, 1 h |
| FD1 | P4 with grid nudging (grid_fdda 1, wrffdda from real.exe every 6 h; restart and resumed run) | 1 h |
| FD2 / FD3 | FD1 without PBL / lowest-level nudging, fgdt 3 / other coefficients, fgdt 2, fgdtzero 1 | 1 h |
| FD4 / FD5 | FD1 ending at 1 h with if_ramping 1 (dtramp_min -30 / +30) | 90 / 100 min |
| FD7 | FD1 nudging until 12 h (the 6 h analyses read during the run) | 7 h |
| FD9 | P9 (MYJ surface layer, REGIME from PBLH) with grid nudging | 1 h |
| FDN | N3 with grid nudging on both domains (wrffdda_d02) | 1 h |
| ACM1 | P4 with the ACM2 PBL over the revised MM5 surface layer (bl_pbl_physics 7; restarted at 30 min) | 1 h |
| ACM7 | P4 with ACM2 over the Pleim-Xiu surface layer (sf_sfclay_physics 7; restarted at 30 min) | 1 h |
| ACMB | ACM7 with bldt 3 | 1 h |
| ACMN | N3 with ACM2 and the Pleim-Xiu surface layer on both domains | 1 h |
| MORR | P4 with the Morrison two-moment microphysics (mp_physics 10, hail) and KF (restarted at 30 min) | 1 h |
| MORG | P4 with Morrison with graupel (morr_rimed_ice 0), no cumulus | 1 h |
| MORN | N3 with Morrison on both domains | 1 h |
| P6 | P5 with Thompson | 1 h |
| P7 | P2 with New Tiedtke | 1 h |
| P8 | P2 with Grell-Freitas-Li (deep and shallow) | 1 h |
| P9 | P1 with MYJ PBL and the Eta surface layer | 1 h |
| P3MP | P4 with P3, one ice category (mp_physics 50) and KF (restarted at 30 min) | 1 h |
| P3NC | P4 with P3 with predicted droplet number (mp_physics 51) and KF | 1 h |
| D6 | P4 with WDM6 (mp_physics 16) instead of WSM6; restarted at 30 min | 1 h |
| D6H | D6 with hail_opt 1 | 1 h |
| D6C | D6 with ccn_conc 3e8 | 10 min |
| D5 | P4 with WDM5 (mp_physics 14) instead of WSM6 | 1 h |
| MYNN | P6 with the MYNN surface layer and the MYNN-EDMF PBL (icloud_bl 1) + RRTMG (radt 10 min) | 1 h |
| MYNNSF1 / MYNNSF2 | MYNN with the revised MM5 / Eta surface layer instead of the MYNN one | 1 h |
| MYNNX1 / MYNNX2 | MYNN with non-default MYNN options (X1: tkeadvect, cloudpdf 1, mixlength 2, closure 3.0, edmf_tke 1, mixqt 1, output 1, tke_budget 1, sfcflux land 2 / water 3, grav_settling 2; X2: edmf 0, cloudmix 0, cloudpdf 0, mixlength 0, closure 2.5, ess 0, mixscalars 1, sfcflux land 3 / water 4, grav_settling 1) | 1 h |
| P9BMJ | P9 with the Betts-Miller-Janjic cumulus (cu_physics 2; restarted at 30 min; cudt 5) | 1 h |
| P4BMJ | P4 with BMJ instead of Kain-Fritsch, without and with bmj_rad_feedback | 1 h |
| P4BMJFI0 | P4BMJF with icloud 0 (no cloud fraction in RRTMG) | 1 h |
| BL1 | P1 with the BouLac PBL (bl_pbl_physics 8; restarted at 30 min) | 1 h |
| BL4 | P4 with the BouLac PBL | 1 h |
| BL9 | P9 with the BouLac PBL over the Eta surface layer | 1 h |
| W5 | P4 with WSM5 (mp_physics 4) instead of WSM6; restarted at 30 min | 1 h |
| W3 | P4 with WSM3 (mp_physics 3) instead of WSM6; restarted at 30 min | 1 h |
| G3P8 | P8 with the Grell-3D cumulus (cu_physics 5, ishallow 1; restarted at 30 min) | 1 h |
| G3P4 | P4 with Grell-3D instead of Kain-Fritsch (cugd_avedx 1 and 3) | 1 h |
| V1 | tropics (Bay of Bengal), Mercator, 15 km, P4 physics | 2 h |
| V2 | Arctic (Svalbard), polar stereographic, 15 km, P4 physics (sea ice, glacier land ice) | 2 h |
| V3 | Alps, Lambert, 3 km, Thompson + MYJ + Eta + Noah-MP + RRTMG (radt 5), no cumulus | 30 min |
| Q1 | P1 with the QNSE PBL and surface layer (bl_pbl_physics 4, sf_sfclay_physics 4, mfshconv 0) | 1 h |
| Q4 | P4 with the QNSE PBL and surface layer (restarted at 30 min) | 1 h |
| Q1M | Q1 with the EDKF mass flux of the QNSE PBL (mfshconv 1; restarted at 30 min) | 1 h |
| Q4M | Q4 with the EDKF mass flux of the QNSE PBL (mfshconv 1; restarted at 30 min) | 1 h |
| U4 | P4 with the single-layer urban canopy model (sf_urban_physics 1, URBPARM.TBL) | 1 h |
| U4L | U4 with use_wudapt_lcz 1 (URBPARM_LCZ.TBL) | 1 h |
| M4 | P4 with the multi-scale Kain-Fritsch (cu_physics 11) instead of KF; restarted at 30 min | 1 h |
| M4C5 | M4 with cudt = 5 | 1 h |
| M9 | P9 (MYJ, no microphysics) with the multi-scale Kain-Fritsch | 1 h |
| M3N | N3 two-way nest with the multi-scale Kain-Fritsch on both domains | 1 h |
| T6 | P2 with Tiedtke (cu_physics 6) | 3 h |
| G2 | P2 with gravity wave drag (gwd_opt 1; ELVMAX = HGT + 3 VAR, WPS 4.6 writes none) | 3 h |
| F1 / F2 | V2 with fractional sea ice (real.exe with fractional_seaice 1), sfclayrev + YSU / Eta + MYJ | 3 h |
| D0 / D0V | WRF's default em_real namelist (physics_suite CONUS, fractional sea ice, gwd_opt 1, diff_opt 2 / km_opt 4, w_damping 0) on the P2 / V2 domains, configuration from tools/wrf2cppwrf.py | 6 h |
| SU1 / SU2 | D0V with sst_update 1 (wrflowinp from real.exe, read every 6 h); SU2: changed sea ice at the start (water/ice transitions) | 7 h / 20 min |
| MO4 | P4 with monotonic moist / scalar advection (moist_adv_opt 2, scalar_adv_opt 2) | 1 h |
| MOK4 | P4 with diff_opt 2 / km_opt 2, monotonic moist and TKE advection (moist_adv_opt 2, tke_adv_opt 2) | 1 h |
| MOMY | MYNN with monotonic moist / scalar advection (qni, qnr, qke_adv) | 1 h |
| MON3 | two-way nest N3 with monotonic moist / scalar advection on both domains (3 min: both domains compared; 1 h: d01 only) | 3 min, 1 h |
| WE4 | P4 with WENO moist / scalar advection (moist_adv_opt 3, scalar_adv_opt 3) | 1 h |
| WP4 | P4 with positive-definite WENO moist / scalar advection (4, 4) | 1 h |
| WM4 | P4 with WENO momentum advection (momentum_adv_opt 3) | 1 h |
| WK4 | P4 with km_opt 2, WENO moist (3) and positive-definite WENO TKE (tke_adv_opt 4) advection | 1 h |
| WMY | MYNN with positive-definite WENO moist (4) and WENO scalar (3) advection | 1 h |
| WN3 | two-way nest N3 with positive-definite WENO moist / scalar and WENO momentum advection | 1 h |
| SKV | V1 with the skin SST (sst_skin 1, without sst_update) and the deep soil temperature update (tmn_update 1; restarted at 3 h, against WRF's run restarted there) | 6 h |
| SKU | SU1 (sst_update 1, fractional sea ice) with sst_skin 1 and tmn_update 1 | 7 h |
| SKN | N3 with sst_skin 1, tmn_update 1 and lagday 5 on both domains (restarted at 30 min) | 1 h |
| N18 | P4 with NSSL 2-moment (mp_physics 18: hail, CCN, graupel / hail volume) and KF; restarted at 30 min | 1 h |
| N17 | P4 with WRF mp_physics 17 (= 18 with its defaults) | 3 steps |
| N18V | V3 (3 km, Noah-MP, no cumulus) with NSSL 2-moment (mp_physics 18) | 30 min |
| TW1 | P1 with YSU topo_wind = 1 (VAR_SSO = VAR**2); restarted at 30 min | 1 h |
| TW2 | P1 with YSU topo_wind = 2; restarted at 30 min | 1 h |
| TW4 | P4 with YSU topo_wind = 1 (VAR_SSO = VAR**2) | 1 h |
| SR1 | P4 with slope_rad = 1 and topo_shading = 1 (radt 10, through sunrise); restarted at 150 min, and at 245 min (between two radiation calls) from its own restart file | 5 h |
| SR2 | SR1 with shadlen = 100 km | 5 h |
| SR3 | P4 with slope_rad = 1 alone (radt 1) | 5 h |
| SR4 | SR1 with the Dudhia shortwave (ra_sw_physics 1) | 5 h |
| HY1 | P1 with hypsometric_opt 1 (real.exe and wrf.exe) | 1 h |
| SLAB1 / SLAB4 | P1 / P4 with the 5-layer thermal diffusion land surface (sf_surface_physics 1; inputs from real.exe) | 1 h |
| NMP1 | P1 with Noah-MP | 1 h |
| RUC4 | P4 with the RUC LSM (sf_surface_physics 3, 9 levels) instead of Noah; restarted at 30 min | 1 h |
| RUC4J | RUC4 with MYJSFC + MYJ PBL and 6 soil levels; restarted at 30 min | 1 h |
| N* (37 runs) | NMP1 with one Noah-MP option changed (dveg 1-3/5-9, opt_crs 2, opt_btr 2/3, opt_runsrf/sub 1/2/4/6/7 and 1/2, opt_sfc 2, opt_frz 2, opt_inf 2, opt_rad 1/2, opt_alb 1, opt_tbot 1, opt_stc 2/3, opt_rsf 2-4, opt_tksno 2-5, opt_compact 1, opt_scf 2, opt_wetland 1, soiltstep 180; 5 restarted at 30 min) | 1 h |
| X4 | NMP1 inputs with P4 physics (WSM6, KF, RRTMG radt 10) and Noah-MP; restarted at 30 min | 1 h |
| X4L, X4* (23 runs) | X4 with dveg 1-3/5-9, opt_crs 2, opt_btr 2/3, opt_rad 1/2, opt_rsf 2/3, opt_sfc 2, opt_runsrf/sub 1/2/4/6/7, opt_wetland 1 (X4WET1 also restarted at 3 h) | 6 h |
| X4SNF* (4 runs) | X4 with opt_snf 2-5 | 1 h |
| VB, V* (20 runs) | V3 (Alps snow and glacier) with opt_tksno 2-5, opt_compact 1, opt_scf 2, opt_gla 2, opt_alb 1, opt_stc 2/3, opt_frz 2, opt_inf 2, opt_tbot 1, opt_rsf 4, dveg 2, opt_snf 2-5; VB restarted at 15 min | 30 min |
| N1 | one-way nest (both domains) | 3 parent steps |
| N2 | two-way nest (both domains) | 1 h |
| N2S1 / N2S2 | N2 with smooth_option 1 / 2 (smoothing of the fed-back fields, ht included) | 1 h |
| N3F1 / N3M | N3 with d02 land use differing at the fed-back points / with MYNN and km_opt 4, 2 | 3 parent steps |
| N3 | two-way nest with the P4 physics on both domains (physics feedback, nest ozone from the parent) | 1 h |
| P4, P6, P9, N3 long runs | inputs from real.exe for 24 h / 72 h (wrfbdy every 6 h; 24-72 h from the GFS forecast through WPS), history every 3 h, CPPWRF restarted in the middle from its own wrfrst files | 24 h; 72 h (P4, P6, P9) |
| D0, P4, G2, P8, NMP1, SR1, SLAB1, N3 chains | resumed from CPPWRF's wrfrst and native restart files (SR1: native only) between the radiation, PBL and cumulus calls (radt, bldt, cudt > dt): every later wrfout frame and wrfrst file equals the straight run's | 15-20 min (SR1: 280 min) |
| P4D16 | P4 with dt = 16 s and radt = 10 min on the 72 h inputs, history every hour (WRF's clock past one day: XTIME, coszen) | 30 h |
| N3D50 | N3 with d01 dt = 50 s, so a nest step of 50/3 s (WRF's clock with a fraction of a second: XTIME, julian) | 1 h |
| RSA | P4 with dt = 50 s and restart_interval = 2 min (not a multiple of dt), history every 150 s; the wrfrst files too; resumed from its wrfrst file of 150 s with restart_interval = 2 and = 0 (WRF's restart alarm of a resumed run: the first ring at the file's seconds to the next ring) | 600 s |
| OVL1 | P4 with cldovrlp 1 (RRTMG McICA random overlap) | 1 h |
| OVL3 | P4 with cldovrlp 3 (maximum overlap) | 1 h |
| OVL4I | P4 with cldovrlp 4 (exponential overlap) and idcor 1 (latitude-dependent decorrelation length) | 1 h |
| OVL5 | P4 with cldovrlp 5 (exponential-random overlap, constant decorrelation length) | 1 h |
| ICL0 | P4 with icloud 0 (no cloud fraction in RRTMG) | 1 h |
| D6O1 | P4 with diff_6th_opt 1 (sixth-order diffusion without the flux limiter) | 1 h |
| KM3 | P4 with diff_opt 2 / km_opt 3 (3-D Smagorinsky; WRF is NaN from the third step on) | 2 steps |
| KM3H | P4 with diff_opt 1 / km_opt 3 (3-D Smagorinsky Km, horizontal diffusion along the model surfaces) | 1 h |
| TKA0 | P4 with diff_opt 2 / km_opt 2 (1.5-order TKE) and tke_adv_opt 0 | 1 h |
| ADV0 | MYNN with moist_adv_opt 0 and scalar_adv_opt 0 | 1 h |
| MPDFM1 | MYNN with bl_mynn_cloudpdf -1 (subgrid clouds zeroed) | 1 h |
| MPDFM2 | MYNN with bl_mynn_cloudpdf -2 (subgrid clouds zeroed) | 1 h |
| G9F6C | N3 with the multi-scale Kain-Fritsch on d01 only and icloud = 0 (WRF: icloud 1 on both domains) | 3 steps |
| G9F2 | P4 with output_diagnostics and QVAPOR / QCLOUD added to auxhist3 by iofields_filename | 3 steps |
| W1 | active w_damping (dt 450 s) | 10 steps |

V1-V3, T6, G2, F1, F2, D0, D0V, SU1 and SU2 are compared in every wrfout variable over the whole run; V1-V3 start from the
global GFS of 2026-10-12 00 UTC through WPS, so the domains, projections and surface types differ from the P
cases. The Thompson, MYNN and radar reflectivity references (P6, RADW, RADT, MYNN, MYNNSF1, MYNNSF2, MYNNX1, MYNNX2, mp8, V3, D0, D0V, SU1, SU2, and the MYNN nest N3M) come from WRF built with -r8 -double-8: with -r8 alone
gfortran makes the DOUBLE PRECISION variables of Thompson and of MYNN's locals 16-byte (and the reflectivity code of
WSM6 and Thompson). N3M's first reference came from the stock -r8 build and differed for that reason; against its
WRF-4.8.0-r8d8 reference it is bit-identical.

The long runs compare every 3-hourly frame of both halves (the run stopped one step after 12 h / 36 h and the run
resumed from its wrfrst files) and the wrfrst files at the restart with WRF's uninterrupted run, every variable at
tolerance 0: 24 h for P4, P6, P9 and the two-way nest N3 (both domains), 72 h for P4, P6 and P9. N3's 72 h reference
comes from wrf.exe in 12 h parts resumed from WRF's own wrfrst files (run.segment; each part runs 3 h past its end,
since at a run's last step WRF's nest history differs in the last bits); the N3 72 h comparison is still open. The references come
from tools/gcp/gcp_wrfref.sh (real.exe inputs, run.real), the met_em files beyond 24 h from tools/gcp/gcp_wps.sh
(GFS 0.25 degree, NOAA open data; the 24 h met_em file it makes is identical to the existing one). All long-run cases
are slow (`wrf_suite.py --slow`, `tools/gcp/gcp_run.sh --slow` with long_data.tar, long72_data.tar and long72n3_data.tar
in `--data`): the 24 h ones alone made the default VM suite 65 instead of 43 minutes.

Bit identity holds for references made with gfortran 14.2 and glibc 2.41 on x86-64 (another gfortran version
vectorises other loops, e.g. in Thompson): WRF's vectorised loops and CPPWRF call the same glibc libmvec
functions. glibc's vector log and log10 (`_ZGVbN2v_log`, `_ZGVbN2v_log10`, their SSE4 variants on every CPU with
SSE4.1) take an approximate reciprocal with `rcpps`, which Intel and AMD compute differently, so WRF itself
differs between Intel and AMD EPYC (about 1e-14 after one step); the references are made on Intel. CPPWRF
computes these two as libmvec does on Intel on every x86-64 CPU (src/core/libmvec_x86.hpp: glibc's SSE4
algorithms statement by statement with Intel's rcpps from a table, src/core/libmvec_tables.hpp made by
tools/libmvec_tables.py; CMake option `CPPWRF_PORTABLE_LIBMVEC`, default ON), so its CPU result is the Intel one
on AMD too. The other libmvec functions it calls (exp, pow, sin, cos, atan) and glibc's scalar libm use no
approximate instructions (the scalar ifunc variants depend only on FMA / AVX2, which both vendors have).
The differential tests run WRF's objects on the test machine: their Fortran drivers link
tests/phys/libmvec_intel.cpp, whose `_ZGVbN2v_log` / `_ZGVbN2v_log10` are these Intel-equivalent functions
(on Intel identical to libmvec), so WRF's side gives Intel's bits on AMD too (with libmvec's own log / log10 it
differed on AMD EPYC by 1 ulp in KF, Noah-MP, the MYNN surface layer and the WSM6 / Thompson reflectivity).
The full CPU suite therefore runs on AMD CPUs: on every push to main on GitHub-hosted runners (AMD EPYC or Intel Xeon; for 1.1.0
at 78e46c2: suite PASS 342, WRF output files 42, ideal files 14, `ctest -L diff` 32/32, ctest 33/33; for
1.2.0 at 0bc4652, in a Debian 13 container: suite PASS 386 with the 44 `cpu_sp` cases, WRF output files 42, ideal
files 14, `ctest -L diff` 32/32, ctest 35/35; for 1.2.1 at 4a57df7: suite PASS 387, WRF output files 42,
ideal files 14, `ctest -L diff` 32/32, ctest 36/36; for 1.2.2 at fbbcf57: suite PASS 423, WRF output files
42, ideal files 14, `ctest -L diff` 32/32, ctest 40/40; for 1.2.3 at c29a32d / ca4de31: suite PASS 527, WRF
output files 42, ideal files 14, `ctest -L diff` 32/32, ctest 40/40; for 1.3.0 at 318159a: suite PASS 527,
WRF output files 42, ideal files 14, `ctest -L diff` 32/32, ctest 39/40 (the generated option tables, fixed in c0de113)), and
on the AMD CPU of Colab's G4 runtime (`tools/colab/colab_suite.sh --gpu G4 --any-cpu --jobs 12 --threads 4`: the CPU
suite, the output files, ideal files, differential and ctest tests); the Intel L4 runtime and the GCP VMs stay
possible. The historical CPU suite totals include the legacy `straka_wrf` RMS check at 1e-4 K. The CPU FP64 cases
against WRF real*8, including `ideal_grav2d_x`, use tolerance 0.
The CPU result does not depend on the OpenMP thread count or on the Intel machine: P4, P6, P8, P9, MYNN,
P1 with Noah-MP and the two-way nest N3s (1 h, both domains) and em_quarter_ss (30 min) with 1, 16 and 44
threads are bit-identical to serial WRF on c3 (Sapphire Rapids) and n2 machines (`run.threads` in
tools/gcp/gcp_matrix.sh). The required build flags are in [USER_GUIDE.md](USER_GUIDE.md#required-flags-and-determinism).

Restarts reproduce an uninterrupted run bit for bit (P1, A01, A05, A05G, P7, P8, P9, NMP1, SLAB1, ACM7, MYNN, MYNNSF1, MYNNSF2, MYNNX1, MYNNX2, T1, MORR, P3MP; P1 also in FP32 on the GPU), and a run
resumed from WRF's own wrfrst files continues as WRF does (also the two-way physics nest N3, both domains).
The restart variants of tests/verify/matrix_restart_io.txt (tools/gcp/gcp_matrix.sh) write restart files in the
middle of a run and resume both models from WRF's: the wrfrst files of the straight and the resumed runs (data and
alarm attributes) and the resumed history are identical to WRF's for P1, P4 (radt 10), Kessler, Thompson, New
Tiedtke, Tiedtke, Grell-Freitas, MYJ, MYNN, Noah-MP, G2, V2, diff_opt 2 / km_opt 2, diff_6th_opt, A01, the 5-layer thermal diffusion LSM (P1), em_quarter_ss
(also diff_opt 2), SU1 across the 6 h wrflowinp and boundary update, and history intervals that are not a divisor
of the restart time (after a restart WRF's input_wrf takes the seconds to the next history time as the history
interval, which the resumed runs reproduce); the same holds for the two-way nest N3s (P4 physics, also with
Kessler, Thompson, New Tiedtke, Tiedtke, Grell-Freitas, MYJ, MYNN, diff_opt 2 / km_opt 2, A01, feedback 0), the
sibling nests N3sib and the nest in a nest N3gc. Where WRF's resumed run differs from its straight
run (sst_update, the history quirk), CPPWRF's differs in the same way.

Output files: the `wrfout_d0N` files of P1-P9, NMP1, MYNN, mp8, V1-V3, T6, G2, F1, F2, D0, D0V, SU1, HY1, R1, T1, N1, N2 and N3 (both
domains) and the `wrfrst_d0N` files of P1, P3, P4, P6, P7, P8, P9, NMP1, MYNN, mp8, N2, N3, T6, G2, F1, F2, D0, D0V and SU1 are
identical to WRF v4.8.0's. This covers
every variable, value, dimension, attribute and global attribute
(`tests/verify/run_wrf_files_test.sh`). Runs of these cases resumed from WRF's
`wrfrst` files, give the same files as WRF's resumed runs. The exceptions are values WRF leaves uninitialised:
RQCBLTEN / RQIBLTEN in runs without qc / qi, and QZ0 / USTM at fractional sea-ice points with the revised MM5
surface layer (restart fields). `cppwrf_real` output is identical to real.exe for 43 cases
([REAL.md](REAL.md)).

WRF's clock: XTIME, julian and the year come from the exact clock (n times the rational time step; a nest's is the
parent's / parent_time_step_ratio), as WRF's ESMF clock gives them. P4D16 (P4 with dt 16 s and radt 10 min on the 72 h
inputs, 30 h, every hourly frame) covers XTIME past one day (days*1440 + seconds/60; the radiation call of 27.7 h is
the first whose coszen depends on it) and N3D50 (N3 with d01 dt 50 s, a nest step of 50/3 s, 1 h) a step that is a
fraction of a second; both pass at tolerance 0 on the CPU (Colab L4, Intel) and in CUDA FP64 (L4).

CUDA FP64 is bit-identical to WRF and to the CPU build: every `cuda_dp` case of the suite passes with tolerance 0
(tests/verify/exact.py, every frame and variable of the history). For 1.1.0 the Colab G4 (RTX PRO 6000 Blackwell
Server Edition, sm_120, CUDA 13.0, 2026-11-20) ran all of them at fb158f3, which has the model code of 78e46c2: 319
cases, 18 GPU restart checks and the legacy `straka_wrf_cuda` RMS check, 338 PASS, 0 FAIL; the device glibc math proof was identical
and ctest passed 35/35 in the FP64 and the FP32 CUDA build. For 1.2.2 (fbbcf57, 2026-11-22): 337 cases, 18 GPU
restart checks and the legacy RMS check, 356 PASS, 0 FAIL. For 1.2.3 (c29a32d, 2026-11-22): 404 cases, 18 GPU
restart checks and the legacy RMS check, 422 PASS, 0 FAIL, 1 not run (its data tar was missing from the GPU data list).
For 1.3.0 (318159a, 2026-11-22): 404 cases, 18 GPU restart checks and the legacy RMS check, 423 PASS, 0 FAIL, 0 SKIP.
The legacy RMS check used WRF real*4 and a tolerance of 1e-4 K. The `ideal_grav2d_x` case uses tolerance 0 against
WRF real*8. Earlier subsets also ran on Colab's L4 (sm_89) and A100
(sm_80); H100 was not available. The FP64 device code
computes as WRF's gfortran real*8 build on x86-64:

- no FMA contraction (`--fmad=false` for every CUDA FP64 kernel; gfortran without -march emits none);
- glibc 2.41's libm (the FMA variants WRF's executable selects) and libmvec (SSE4 2-lane pow, exp, log, atan, sin,
  cos, log10; the lane of each call as gfortran vectorises the loop) ported to the device (src/core/glibc_math.hpp,
  CMake option `CPPWRF_CUDA_GLIBC_MATH`, default ON); tools/glibc_math_proof.cpp and tools/glibc_math_proof_cuda.cu
  prove the port equal to glibc (10^9 random inputs per function on the CPU; 10^7 per function on each GPU against
  the host, the `CPPWRF_JOB_GLIBC_PROOF` default of tools/colab/remote_job.py that the release GPU proofs ran, e.g.
  the 1.2.1 G4 proof; the tool's own default without an argument is 10^8);
- the folds gcc and gfortran make: x**2.0 with a constant exponent as x * x (`rpow2c`), functions of constants as
  constants;
- no atomics in floating-point sums; division and sqrt correctly rounded (`-prec-div`, `-prec-sqrt`).

CUDA 13.0's ptxas for sm_120 compiles an integer add of an immediate to the bits of a double register as a
floating-point add (`mov.b64; add.s64` became `DADD`); log's |x - 1| test then sent inputs near 1 to its table path.
The device code makes these adds with two 32-bit adds with carry (`glibc::detail::addk`). FP64 stepping time against
main before the FP64 bit-identity work (a862af3, CUDA's math library and FMA contraction; the work, e09fe5e to
86d8aa7: glibc's math on the device, `--fmad=false`, this work-around and the folds, measured together; 1 h, no
history; P1 / P4 / D0): L4 +7 % / +2 % / +3 %, G4 +6 % / -4 % / 0 %, A100 +5 % / +2 % / +6 %. No other measurement
of the device math's cost exists (in particular none of `CPPWRF_CUDA_GLIBC_MATH=OFF` against `ON` alone). The fast
CUDA FP32 build (`-DCPPWRF_CUDA_FP32_IDENTITY=OFF`: CUDA's math library, FMA contraction) is not affected by this. Its
relative L2 at 1 h in the 1.1.0 G4 run (fb158f3; docs/FP32_ACCURACY.md, section 1.1.0, has the values that moved
with this release's FP32 fixes):

| Case | Precision | U | W | THM | QV |
|---|---|---|---|---|---|
| R0 (dynamics) | FP32 vs WRF FP32 | 1.6e-4 | 4.9e-3 | 1.0e-6 | 2.0e-5 |
| P1 | FP32 vs WRF FP32 | 1.9e-4 | 3.9e-3 | 7.5e-7 | 4.7e-5 |
| P3 | FP32 vs WRF FP32 | 1.9e-4 | 4.0e-3 | 7.4e-7 | 4.5e-5 |
| P4 / P6 / P8 | FP32 vs WRF FP32 | 2.1e-4 to 6.3e-4 | up to 2.1e-2 | up to 9.4e-6 | up to 5.4e-4 |
| N1 (1 h) | FP32 vs WRF FP32 | 1.9e-4 (d01), 2.8e-4 (d02) | 3.6e-3 / 4.7e-3 | 6.5e-7 / 9.7e-7 | 2.0e-5 / 3.1e-5 |

On the GPU a run restarted after its first step reproduces the uninterrupted run bit for bit
(restart_<case>_fp64_cuda for P1, A01, A05, A05G, P7, P8, P9, NMP1, SLAB1, ACM7, MYNN, MYNNSF1, MYNNSF2, MYNNX1, MYNNX2, T1, MORR, P3MP), and the
wrfout / wrfrst files of a GPU run (P1, P4, MYNN, N2, also resumed from WRF's wrfrst) have WRF's variables,
dimensions, attributes, global attributes and times (run_wrf_files_test.sh with CPPWRF_FILES_NO_VALUES=1).

Ideal cases (CPPWRF's port of WRF's ideal.exe, `run.case = em_*`), CPU FP64 against WRF v4.8.0
real*8: em_hill2d_x (1 h), em_grav2d_x (Straka, 900 s), em_quarter_ss (Kessler supercell, 2 h, periodic and with open boundaries in x and y) and
em_squall2d_x / em_squall2d_y (open lateral boundaries in x / y, TKE diffusion, 10 min) are bit-identical
in every field; em_quarter_ss also with diff_opt 2 / km_opt 2 (and isfflx 0). Their wrfout files equal WRF's in every
variable and attribute (tests/verify/ideal_files_test.py).
Symmetric lateral boundaries (WRF symmetric_xs/xe/ys/ye, rigid walls) with soundings without wind are bit-identical
too, with the storm and the cold pools against the walls: em_quarter_ss on all four sides (21 x 21 cells, 1 h), the
Straka density current and em_squall2d_x (TKE) in x (40 min); in the variant matrix also x or y alone, diff_opt 2 /
km_opt 2, km_opt 3 / 4, 6th-order diffusion, h_adv_order 6, moist_adv_opt 0, em_hill2d_x and winds along the walls. A
wind across a symmetric wall is rejected: WRF keeps the wall value of the normal velocity while the sign of muu flips
there, and its solution breaks down within a minute.
On the GPU the ideal cases (also symmetric, open and diff_opt 2) are bit-identical in FP64, and em_squall2d_x in fast
FP32 is within WRF's own single/double difference (U 6.8e-5; WRF real*8 vs. WRF FP32: 8.2e-5).

Namelist variants (tools/gcp/gcp_matrix.sh, 3-minute runs from the P1, P4, P6, P8 and P9 inputs, every wrfout
variable at tolerance 0): kfeta_trigger 3, ysu_topdown_pblmix 0, icloud 0, cldovrlp 0-5 (1-5 with idcor 0 and 1; 0 also with restart files, MYNN and the N3s nest), cu_rad_feedback (Kain-Fritsch: also cudt 5, Kessler, icloud 0, cldovrlp 0, wrfrst at 30 min; Grell-Freitas with WSM6 or Kessler and RRTMG, ishallow 0 and 1, wrfrst at 30 min; Tiedtke and New Tiedtke 1 h; the N3s nest on both or one domain; 3 min and 1 h), shallowcu_forced_ra,
o3input 0, ghg_input 0, use_mp_re 0, rdmaxalb false, ifsnow 0, kim_tofd, isfflx 0, Kessler on real data (also with
Kain-Fritsch: cudt, kfeta_trigger 3, dt 30 s and 90 s, radiation off, 4.5 h with restart files),
radt / bldt / cudt > dt, Grell-Freitas clos_choice 1, 4, 10 and ishallow 0, hybrid_opt (input-file check),
diff_6th_opt 1 and 2, moist/scalar_adv_opt 0, w_damping 0 and w_crit_cfl, damp_opt 0, dampcoef / zdamp,
epssm, time_step_sound, smdiv / emdiv, c_s, km_opt 1 / 3 / 4 with diff_opt 1 (km_opt 3 also on em_quarter_ss with
diff_opt 2), diff_opt 2 / km_opt 4 (also mix_full_fields), mix_isotropic 1, PD advection with h_adv_order 6,
moist_mix2_off, the RRTM longwave with RRTMG and Dudhia shortwave (P4 at night and V1 in daylight: 3 min, 1 h, 3 h, radt 15, icloud 0, o3input 0, ghg_input 0, Kessler, Thompson, MYNN with icloud_bl 1, cu_rad_feedback with Kain-Fritsch and Grell-Freitas, wrfrst at 30 min; two-way nest N3s with both pairs and wrfrst), the Dudhia shortwave with RRTMG longwave (P4 at night and V1 in daylight: radt 1 and 15, icloud 0, swrad_scat, Kessler (warm_rain), Thompson, MYNN with icloud_bl 1, 3 min / 1 h / 3 h, wrfrst at 30 min; two-way nest N3s with wrfrst), MYNN (P6 inputs) with RRTMG, icloud_bl 0, bldt 2, WSM6, Kain-Fritsch and restart_interval 1 (the restart alarm in the first step), the MYNN PBL on the revised MM5 and Eta surface layers (P6 inputs, 3 min: without radiation, RRTMG with icloud_bl 0, bldt 2, WSM6, Kain-Fritsch, Noah-MP, isfflx 0, iz0tlnd 1, isftcflx 1, wrfrst with restart_interval 1; 1 h with RRTMG and wrfrst at 30 min; two-way nest N3s on d01 and d02), every non-default MYNN option value with a port (P6 inputs, MYNN surface layer, RRTMG: 3 min and 1 h each, wrfrst at 30 min with bl_mynn_tkeadvect, closure 3.0, grav_settling 2 and the combined X1 / X2 options, bldt 2, and the N3s two-way nest with tkeadvect, edmf 0 / closure 3.0 / mixlength 2, the MYNN surface-layer forms and grav_settling), grav_settling 1 and 2 with YSU, usemonalb and rdlai2d (together and alone, also false) on P1, P4, P9, P1 with Noah-MP, V2 (snow, sea ice, glacier) and the N3s nest (both domains), 3 min and 1 h, with wrfrst files at 30 min (P1) and 15 min (V2), and on SU1 over 6 h 20 min across the 6 h wrflowinp update of LAI and ALBBCK, open-boundary 6th-order diffusion (em_squall2d_x, em_quarter_ss open), v_adv_order 4, and surface layer + LSM without a PBL scheme (bl_pbl_physics 0, with
diff_opt 1 or diff_opt 2 / km_opt 2) are bit-identical to WRF. output_diagnostics = 1 (clwrf_output_calc: T2MIN ... RAINNCVSTD and the auxhist3 files) is bit-identical on P4 (3 min, 1 h, auxhist3 every 2 / 3 / 10 / 30 min, frames_per_auxhist3 1 with an auxhist3_outname, io_form_auxhist3 0), P1, P6 and the two-way nest N3s (10 / 5 min), with wrfrst files at 25 and 30 min and runs resumed from them (the auxhist3 alarm of WRF's restart file). The accumulation buckets and precipitation windows (bucket_mm, bucket_J, prec_acc_dt) are bit-identical on P1 (no microphysics, no radiation: WRF stops with bucket_J there and CPPWRF rejects it), P4 (also Kessler, the adaptive time step, bucket_J alone, which does nothing as in WRF, and 61 min: the buckets of the step at 1 h), P6 (also with RRTMG), P8, V1 (daytime: the SW accumulations tip) and the two-way nest N3s (prec_acc_dt 10 / 5 and 0 / 5), 3 min and 1 h, with wrfrst files at 30 min and runs resumed from them (WRF's resumed run tips the buckets at its first step, its curr_secs2 starting at 0; CPPWRF's resumed run does the same). Two-way nest N3 variants: smooth_option 0 / 1,
feedback 0, parent_time_step_ratio 2, per-domain cu_physics (cumulus off on d02), cudt, radt and bldt. New Tiedtke
(cu_physics 16, 30-minute runs) on the P1, P4 (also cudt 5), P6 and P9 inputs and on the N3s (also off on d02), N3sib
and N3gc nests, whose dx below 15 km takes the scheme's log scale factor, is bit-identical. (WRF leaves RQCBLTEN / RQIBLTEN of runs without qc / qi
and LH with isfflx 0 uninitialised; they are not compared.)

### FP32 against WRF real*4

The CPU FP32 build (`-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF`) is bit-identical to stock WRF v4.8.0 real*4 (gfortran 14.2, glibc 2.41,
the default real*4 build) on 24 configurations: P1, P2, P3, P4, P9, NMP1 (Noah-MP), P6 (Thompson), MYNN, C01
(aerosol-aware Thompson), D6 (WDM6), D5 (WDM5), MORR and MORG (Morrison with hail / graupel), G2 (GWDO), the nests N1
(one-way) and N3 (two-way, P4 physics), MON3 (monotonic advection on N3), KESSKF (Kessler), ACM7 (ACM2 + Pleim-Xiu),
Q4 (QNSE), T6 (Tiedtke), P7 (New Tiedtke), DUD4 (Dudhia shortwave) and SLAB1 (5-layer slab LSM). The 44 `cpu_sp`
cases run each of them for 1 h and 20 of them also for 3 minutes with history every step (every minute on the nests of N3 and MON3), every variable of the WRF
real*4 history at tolerance 0. The default CUDA FP32 build (`-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF`,
`CPPWRF_CUDA_FP32_IDENTITY=ON` by default: no FMA contraction, glibc 2.41's float functions on the device) gives the
same bits on the GPU: the 44 `cuda_sp_id` cases pass on an RTX 2060 (sm_75) and on a Colab G4 (sm_120). The FP32
executables define glibc 2.41's float functions themselves (src/core/glibc_mathf_host.cpp), so binaries built on Ubuntu
24.04 (glibc 2.39) pass these cases too. The opt-in fast CUDA FP32 build (`-DCPPWRF_CUDA_FP32_IDENTITY=OFF`) is compared
by relative L2 (above and docs/FP32_ACCURACY.md). In the [v1.3.0 paper benchmark](#performance), the identity build
takes 2.1 to 18.0 % longer in steady stepping than the fast build (identity time / fast time - 1). docs/FP32_IDENTITY.md
has the method, the case table, the proofs of the ported functions, the cost and what is not identical yet (moving
nests, more than two domains, restarts, configurations without a WRF real*4 reference, other compilers for WRF).

### Verification coverage

Option values that CPPWRF accepts and whose code paths are checked at model level against WRF (the coverage audit of
2026-11, closed in tests/verify/cases/69_coverage.py): RRTMG `cldovrlp` 1, 3, 4, 5 and `idcor` 1, `icloud` 0,
`diff_6th_opt` 1, `km_opt` 3 (with `diff_opt` 1 for 1 h; with `diff_opt` 2 two steps, after which WRF itself is NaN
on the 15 km test domain), `tke_adv_opt` 0, `moist_adv_opt` / `scalar_adv_opt` 0 and `bl_mynn_cloudpdf` -1 / -2, each
CPU and CUDA FP64 at tolerance 0. WRF's MYNN defaults (`bl_mynn_edmf` 1, `bl_mynn_edmf_mom` 1, `bl_mynn_mixlength` 1,
`bl_mynn_cloudmix` 1, `bl_mynn_cloudpdf` 2, `bl_mynn_closure` 2.6) are what the MYNN, MYNNSF1 and MYNNSF2 cases run
(their WRF namelists leave them at the Registry defaults); MYNNX1 / MYNNX2 run the other values with a port.
`moist_adv_opt` / `scalar_adv_opt` / `tke_adv_opt` 2 (monotonic, advect_scalar_mono), 3 (WENO) and 4 (positive-definite
WENO), and `momentum_adv_opt` 3 (WENO, advect_weno_u / _v / _w) are verified at model level (tests/verify/cases/46_advection.py:
MO4, MOK4, MOMY, MON3, WE4, WP4, WM4, WK4, WMY, WN3, 3 min and 1 h at tolerance 0) and column by column against WRF's
advection routines (tests/dyn/advection). The WENO references come from WRF built with -r8 -double-8: in the plain -r8
build the WENO routines' DOUBLE PRECISION locals are real*16. Monotonic advection needs h_sca_adv_order 5 and
v_sca_adv_order 3 (WRF stops with other orders).

Order and initialisation: every CPU FP64 suite configuration, including the 56 whose inputs the audit did not have,
gives the same bytes over 3 steps in the normal OpenMP run, serially with every launch's points in reversed order,
serially with i outermost, and in a build with `-ftrivial-auto-var-init=pattern -fsanitize=bounds` (no bounds
report). GPU: NVIDIA compute-sanitizer memcheck, initcheck, synccheck and racecheck report no error or hazard on
CUDA FP64 and FP32 builds (one step each) of P4 (with and without CUDA graphs), MYNN-EDMF, Thompson, Morrison, RRTMG
with cldovrlp 5, the two-way nest N3, P4 on 2x2 tiles and em_quarter_ss (racecheck of the FP32 2x2 tiled run did
not finish within 40 min and reported nothing up to then).

Not reached at model level: the option values of "Not yet implemented" below, and the GPU-only code paths of
schemes that no suite case runs on the GPU.

## Performance

The v1.3.0 paper benchmark (CPPWRF commit `9f9bb77`) ran WRF v4.8.0 and CPPWRF on a Google Colab G4 with an AMD
EPYC 9B45 (24 cores, 48 hardware threads) and an NVIDIA RTX PRO 6000 Blackwell. Each case ran for 6 to 24 h. WRF uses
its stock GNU real*4 and real*8 builds and the fastest of eight screened MPI or MPI + OpenMP layouts for each case
and precision. The CPU FP64 column uses the selected thread count for each case.

The table gives steady stepping seconds per simulated hour with history off from the unrounded `step_s` medians in
the benchmark's `runs.csv` and `summary.json`.

| Case | WRF real*4 | WRF real*8 | CPPWRF GPU FP32 (fast) | CPPWRF GPU FP64 | CPPWRF CPU FP64 |
|---|---|---|---|---|---|
| C1: P4, radt 15 min | 2.90 | 4.36 | 0.46 | 1.54 | 5.49 |
| C2: L (359 x 269 x 44), radt 15 min | 28.60 | 50.38 | 2.78 | 9.16 | 49.83 |
| C3: MORR, radt 1 min | 11.77 | 14.43 | 1.61 | 5.00 | 11.99 |
| C4: N3 | 30.00 | 39.63 | 3.32 | 9.78 | 36.05 |

Across the four cases, the steady stepping medians give speed-ups over WRF real*4 of 6.33x to 10.28x for fast GPU
FP32 and 5.36x to 10.07x for identity GPU FP32. GPU FP64 runs 2.83x to 5.50x faster than WRF real*8.

The earlier `c9c95c5` benchmark (2026-11-16, docs/PERF.md) also measured these 1 h cases with history off, before
RRTMG computed in single precision in CUDA FP32 builds. `tools/bench/colab_bench.sh` reproduces that benchmark.

| Case | WRF real*4 | WRF real*8 | CPPWRF GPU FP32 (fast) | CPPWRF GPU FP64 | CPPWRF CPU FP64 (48 threads) |
|---|---|---|---|---|---|
| R1 | 0.84 | 1.99 | 0.11 | 0.44 | 4.69 |
| P4 | 7.46 | 10.71 | 1.47 | 2.35 | 11.40 |

No new benchmark was run for 1.1.0 or 1.2.0 (docs/PERF.md says what was and was not measured again; 1.2.0's GPU
changes left the G4's 1 h stepping times within a second, docs/RELEASE_NOTES.md).

Agreement in the five `c9c95c5` cases:
- The CPU FP64 output is bit-identical to the suite's WRF real*8 references for P4 and N3 (both domains).
- The GPU FP64 output is bit-identical to the CPU FP64 output in all five cases.
- With this CPU's own libmvec log (`CPPWRF_PORTABLE_LIBMVEC=OFF`), the CPU FP64 output is bit-identical over the
  hour to WRF real*8 built and run on the runtime in one process, for R1, P4 and L.

WRF run with MPI ranks differs from its own one-process run, so exact comparisons use one-process WRF.

The NetCDF-4 history and restart files are compressed on host threads (the same bytes as HDF5's own filter): with a
frame every 10 minutes and a restart file, L's hour takes 13 s in GPU FP32 and 33 s in FP64 (58 s and 123 s with
serial compression). Wall times, GPU memory and the details are in docs/PERF.md.

## Not yet implemented

Milbrandt-Yau (mp_physics 9), WSM7 (24), WDM7 (26), WSM6R (106), WSM3 / WSM5 / WDM5 / WDM6 with do_radar_ref 1, in the ideal cases or (WDM) with nests, KF-CuP (cu_physics 10), Grell-Devenyi (93), the old Kain-Fritsch (99), multi-scale Kain-Fritsch with aercu_opt > 0 or kf_edrates 1, Shin-Hong (bl_pbl_physics 11), shcu_physics, the QNSE surface layer with fractional_seaice = 1, BEP and BEP+BEM (sf_urban_physics 2, 3), the single-layer urban model with a land surface model other than Noah, on nests or with a restart, the lake model (sf_lake_physics), CLM and SSiB (sf_surface_physics 5, 8), the RUC LSM with mosaic_lu / mosaic_soil, fractional_seaice, sst_update, the MYNN surface layer, nests or cppwrf_real input, CAM radiation (ra_lw_physics / ra_sw_physics 3), NSSL other than mp_physics 18 / 17 with the default nssl_* options (19, 21, 22, nssl_3moment, nssl_hail_on 0, non-default nssl_cccn ... nssl_icdxhl, nssl_ssat_output, &nssl_mp_params, nests, the ideal cases), P3 with two ice categories (52) or three-moment ice (53), P3 with nests, ideal cases or do_radar_ref 1 (REFD_MAX), Morrison with aerosols (mp_physics 40), Morrison's prognostic droplet number (progn 1) and its 10 cm reflectivity (do_radar_ref 1 with mp_physics 10), cumulus, radiation or PBL physics without both a surface layer and a land surface model (wrf.exe runs them), the Dudhia shortwave with cu_rad_feedback and Grell-Freitas (wrf.exe writes NaN cloud fractions there), the PX land surface (sf_surface_physics 7, pxlsm_* options and PX soil nudging), the Pleim-Xiu surface layer without ACM2 or Noah or with fractional_seaice = 1, sf_sfclay_physics 91, km_opt 5, the MYNN PBL with fractional_seaice = 1, the MYNN options bl_mynn_edmf 2, bl_mynn_edmf_dd, bl_mynn_mixnumcon, bl_mynn_mixaerosols 0, bl_mynn_output 2, other bl_mynn_closure values and the GFS surface-layer forms (sf_mynn_sfcflux_land 4, sf_mynn_sfcflux_water 5), the Noah-MP options that need input fields (opt_soil 2-4 / opt_pedo, opt_crop, opt_irr / opt_irrm, opt_tdrn, opt_wetland 2, opt_runsrf / opt_runsub 5), the dynamic VIC runoff (opt_runsrf / opt_runsub 8, opt_infdv: WRF 4.8.0 reads an uninitialised local) and SNICAR (opt_alb 3), sst_skin and tmn_update with land surface models other than Noah, the adaptive time step with nests, sst_update, sst_skin, tmn_update, grid_fdda or ideal cases, spectral nudging (grid_fdda = 2),
surface analysis nudging (grid_sfdda, needs OBSGRID's wrfsfdda_d0N), observation nudging (obs_nudge_opt), moving nests, ndown.exe (one-way nesting from a parent run's wrfout), I/O forms other than NetCDF
(io_form 2) and auxiliary history / input streams other than sst_update's wrflowinp and grid_fdda's wrffdda (WRF writes none by default),
and runs on several nodes or more than two GPUs (docs/MPI.md, section 7). Options without a port are rejected by `tools/wrf2cppwrf.py` and by the model with an
error; docs/USER_GUIDE.md, "Rejected Options and Why", lists each with its reason. Noah-MP with nests is rejected because wrf.exe 4.8.0 aborts with it (heap corruption in the nest run's
first history output).

Restart and input limits that remain (each refused with an error):
- the single-layer urban model resumed from any restart file (WRF's urban_var_init on a restart is not ported);
- CPPWRF's own restart files (`restart_<step>.nc`) with nests, the adaptive time step or `tmn_update`: resume from
  the run's wrfrst files;
- two-way nest feedback with Noah and the NLCD40 land use (phy_init after the feedback);
- the debug dump `CPPWRF_VM_DUMP` with `run.tiles` or MPI ranks.

`CPPWRF_KTIME=1` with `run.tiles` on one GPU synchronises the whole device around each launch: a tile's kernel times
include the other tiles' kernels running at the same time (use it without tiles for kernel timings).
