# Changelog

All notable changes of CPPWRF. Versions follow [Semantic Versioning](https://semver.org/): a change of the results of
a verified configuration, of a configuration key or of an output file is a new major version. CPPWRF's native
restart files (`restart_<step>.nc`) are not a stable interface: they are bound to the commit that wrote them; a run
resumes across versions from WRF's wrfrst files.

## Unreleased

### Added
- CPU FP32 builds bit-identical to WRF v4.8.0 real*4 on P1 and P4 (Kain-Fritsch, WSM6, RRTMG; 3 minutes with
  history every step, and 1 h) and P2, P3, P9 (1 h): base state, boundary coefficients, Noah init, history T, WSM6's init
  products and moist_physics_prep_em's R_v/R_d rounded where WRF real*4 rounds them, and glibc libmvec's 4-lane SSE4
  expf / logf / powf / sinf / cosf ported lane by lane for the iterations WRF's vectorised loops give them
  (src/core/libmvec_f4.hpp, test_libmvec_f4; sinf / cosf for radiation_driver's COSZEN). The suite's CPU FP32 backend
  `cpu_sp` (`--cpu-sp`) compares them at tolerance 0: `real_p1_fp32_cpu`, `real_p1_fp32_cpu_1h`,
  `real_p2_fp32_cpu_1h`, `real_p3_fp32_cpu_1h`, `real_p9_fp32_cpu_1h`, `real_p4_fp32_cpu`, `real_p4_fp32_cpu_1h`
  (new data tar fp32id_data.tar), and Noah-MP NMP1 (1 h, `real_nmp1_fp32_cpu_1h`: the MOST resistances' four logarithms
  through the 4-lane logf) and Thompson P6 (3 minutes and 1 h, `real_p6_fp32_cpu`, `real_p6_fp32_cpu_1h`, new data tar
  fp32id2_data.tar: thompson_init's constants, coefficients and table builders in REAL where WRF real*4 has them,
  the runtime's rain slope and sedimentation powers scalar). docs/FP32_IDENTITY.md. FP64 and CUDA results unchanged.

## 1.1.0 (2026-11-20)

208 commits (not counting merges) after 1.0.0, release candidate main 78e46c2; docs/RELEASE_NOTES.md has the
verified scope. New microphysics (WSM3, WSM5, WDM5, WDM6, NSSL 2-moment, P3), cumulus (BMJ, Grell-3D, multi-scale
Kain-Fritsch), PBL and surface layers (QNSE with EDKF, BouLac), the RUC land surface model, the single-layer urban
canopy model, skin SST and deep soil temperature update, monotonic and WENO advection, Noah-MP option values and
terrain options, all verified against WRF v4.8.0 real*8 at tolerance 0 in CPU FP64 and CUDA FP64. Domain
decomposition with tiles and MPI ranks; drop-in `cppwrf-wrf` / `cppwrf-realexe` / `cppwrf-check`; packaging, the
Python package and user tools. The fixes of an independent code review (dropped namelist options, GPU paths,
decomposed runs, restart state, `cppwrf_real` inputs, namelist checks, nests); FP32 builds follow WRF real*4's
single-precision arithmetic more closely; RRTMG computes in single precision in CUDA FP32 builds. Verified FP64
results are unchanged. Release checks: full CPU CI on 78e46c2 (suite PASS 342, WRF output files 42, ideal files 14,
`ctest -L diff` 32/32, ctest 33/33, all exact) and the Colab G4 GPU proof at fb158f3 (CUDA FP64 suite PASS 338 at
tolerance 0, CUDA FP32 suite 44 runs within their measured tolerances, ctest 35/35 in both CUDA builds).

### Added
- Terrain options, tier **verified**: YSU's topographic wind correction `topo_wind = 1 / 2` (start_em's CTOPO /
  CTOPO2: 1 from VAR_SSO and the terrain Laplacian, a wrfinput without VAR_SSO (GOT_VAR_SSO = 0) stops as in WRF;
  2 from VAR), the slope-dependent surface shortwave `slope_rad = 1` (TOPO_RAD_ADJ_DRVR: SWDOWN and GSW by the slope,
  its azimuth and the shadow at each surface call, SWNORM, restored at the end of the surface driver) and the
  terrain shadows `topo_shading = 1` with `shadlen` (pre_radiation_driver's toposhad at the radiation steps,
  HT_SHAD). Real data, one domain, fixed time step; topo_shading not with tiles or MPI ranks. Suite cases TW1, TW2,
  TW4 (1 h) and SR1-SR4 (5 h through sunrise) against WRF real*8, restarted at 30 / 150 min, CPU and CUDA FP64 at
  tolerance 0; column test tests/phys/topo (radconst, toposhad and TOPO_RAD_ADJ_DRVR bit for bit with WRF's
  compiled code, including the edge search over start_em's boundary copy of HGT beyond the domain). The translator
  passes the options and stops on nests, ideal cases and adaptive time steps with them.
- WDM6 and WDM5 microphysics, `mp_physics = 16` (`hail_opt` 0 / 1) and `14` (phys/module_mp_wdm6.F,
  module_mp_wdm5.F): double-moment warm rain with the CCN, cloud and rain numbers (scalars QNCCN, QNCLOUD, QNRAIN),
  `[phys] ccn_conc` (start_em's fill when the input has no CCN, the CCN reset of the first step, the inflow value of
  flow_dep_bdy_qnn), WDM6's land-mask dependent autoconversion threshold, WRF's libmvec lanes of the vectorised
  loops, SNOWNC / GRAUPELNC / SR, the effective radii for RRTMG; real data, one domain. Tier: verified (suite cases
  D6, D6H, D6C, D5 against WRF real*8 at tolerance 0; column driver tests/phys/mp_wdm, bit for bit with the -r8
  build's wdm6 / wdm5). Not ported (errors): `do_radar_ref = 1` with them, nests, the ideal cases, WDM6 without the
  column physics, WDM7 (26).
- Multi-scale Kain-Fritsch cumulus (cu_physics = 11, WRF's module_cu_mskf.F): MSKF_eta_PARA's scale-aware trigger
  (DX < 25 km), entrainment, dynamic adjustment time and cloud fractions (CLDFRA_DP / CLDFRA_SH, QC_CU / QI_CU in
  RRTMG: icloud_cu = 2 and icloud = 1 as check_a_mundo sets them), the momentum transport MSKF_CMT (RUCUTEN /
  RVCUTEN, staggered with add_a2c_u / add_a2c_v), cudt; aercu_opt = 0 and kf_edrates = 0 only. Column driver
  tests/phys/cu_mskf against WRF's MSKF_CPS; suite cases M4 (also restarted at 30 min), M4C5, M9 and the nest M3N
  against WRF real*8. Tier: verified (CPU FP64 and CUDA FP64 at tolerance 0). KF-CuP (10) and the old Kain-Fritsch
  (99) are rejected by tools/wrf2cppwrf.py and the model.
- Monotonic and WENO advection: `moist_adv_opt` / `scalar_adv_opt` / `tke_adv_opt` 2 (WRF's advect_scalar_mono, the
  Smolarkiewicz flux limiter after rk_update_scalar_pd), 3 (advect_scalar_weno; theta also takes the WENO fluxes on
  the last RK stage) and 4 (advect_scalar_wenopd), and `momentum_adv_opt` 3 (advect_weno_u / _v / _w), all on the
  last RK stage as in module_em.F. Tier: verified (suite cases MO4, MOK4, MOMY, MON3, WE4, WP4, WM4, WK4, WMY, WN3
  against WRF real*8 at tolerance 0, the WENO ones against WRF -r8 -double-8; column test tests/dyn/advection
  bit-identical with WRF's advection routines). Monotonic advection with other orders than h 5 / v 3 is an error.
- NSSL 2-moment microphysics kernel (WRF's module_mp_nssl_2mom.F, mp_physics = 18 with its defaults and 17: 2-moment
  cloud water, rain, ice, snow, graupel and hail with predicted graupel / hail volume and activated CCN):
  src/phys/mp_nssl (nssl_2mom_init's tables and module values; nssl_2mom_driver for one j row: ice nucleation,
  calcnfromq / calcnfromcuten, the adaptive two-moment sedimentation, nssl_2mom_gs, NUCOND, smallvalues, ssat,
  radardd02, calc_eff_radius; the loops gfortran vectorises with libmvec's pow take the vm:: lanes of WRF's 64-point
  gather batches). Column test tests/phys/mp_nssl (ctest test_phys_nssl) against WRF's nssl_2mom_init /
  nssl_2mom_driver of the -r8 -fdefault-double-8 build: every output bit-identical over several calls. Wired into the
  real-data model as `[phys] mp_physics = 18` (the translator maps WRF's 17 and 18 with the default nssl_* options to
  it): QHAIL, QNDROP, QNHAIL, QNCCN, QVGRAUPEL, QVHAIL, HAILNC / HAILNCV (also Noah-MP's MP_HAIL), REFL_10CM, the
  effective radii to RRTMG, the cumulus tendencies of the rain / snow / ice / cloud water, one row per thread over
  WRF's tile i-range. Tier: verified (suite cases 48_nssl: N18, N17, N18V against WRF real*8 of the -r8
  -fdefault-double-8 build, CPU FP64 and CUDA FP64 at tolerance 0). 19, 21, 22, non-default nssl_* options, nests, the ideal
  cases and a decomposition in x stay errors.
- Noah-MP option values of WRF 4.8.0 (v5.1.2) beyond the defaults: dveg 1-9 (dveg 2 / 5 / 6 with the dynamic
  vegetation carbon of BiochemNatureVegMain / CarbonFluxNatureVeg), opt_crs 2, opt_btr 2 / 3, opt_runsrf / opt_runsub
  1, 2, 4, 6 (VIC), 7 (Xinanjiang), opt_sfc 2, opt_frz 2, opt_inf 2, opt_rad 1 / 2, opt_alb 1, opt_snf 2-5, opt_tbot
  1, opt_stc 2 / 3, opt_gla 2, opt_rsf 2-4, opt_tksno 2-5, opt_compact 1, opt_scf 2, opt_wetland 1 (Zhang 2022, FSAT /
  WSURF restart state), soiltstep; suite cases tests/verify/cases/57_noahmp_opts.py (NMP1, NMP4 = NMP1 with P4's
  physics over 6 h, V3 Alps snow and glacier). opt_runsrf / opt_runsub 5 and 8, opt_infdv, opt_alb 3, opt_soil 2-4,
  opt_crop, opt_irr, opt_tdrn, opt_wetland 2 and dveg 10 stay errors (the model accepted only the defaults before).
- Grell-3D ensemble cumulus (cu_physics = 5, WRF's module_cu_g3.F): G3DRV with the 3 x 3 neighbours' vertical
  motion and advective forcing (ens4 = 9), CUP_enss_3d with its closure ensemble (maxens 3, maxens2 3, maxens3 16),
  the shallow convection (ishallow = 1), clos_choice, cugd_avedx 1 / 3 (`[phys] cugd_avedx`), APR_* /
  MASS_FLUX / XMB_SHALLOW, XF_ENS / PR_ENS (ensemble_stag = 144), K22 / KBCON / KTOP_SHALLOW and CUGD_* in the
  restart files (as G3DRV leaves them; read back on restart), CONV_GRELL_SPREAD3D (the subsidence spreading and the
  smoothing over the neighbours, halo exchanges for patched runs), phy_cu_ten, the cumulus clouds of cu_diag = 1.
  Tier: verified (suite cases G3P8, G3P4, G3P4A3 against WRF real*8, CPU and CUDA FP64 at tolerance 0; patch driver
  tests/phys/cu_g3 bit-identical with WRF's module_cu_g3.o on synthetic and model-dumped patches). Grell-Devenyi (93) stays an error.
- docs/FP32_ACCURACY.md "Root causes of the first-hour outliers": ACM7, MORR and SKV's FP32 excess at 1 h traced
  step by step to Kain-Fritsch deep / shallow and trigger decisions that are ties at the last bit (e.g. `TU95 >
  TV0` at 1 ulp in the first step), measured over five builds of the same code (GPU with and without FMA, CPU on
  two CPUs), perturbed starts and 24 h runs: no FP32 port defect, no output change. tools/bench/colab_fp32.sh
  --extra (GPU FP32 without FMA, CPU FP32) and --data; fp32_run.py's cases P4S / ACM7S / MORRS / SKVS, ACM7L / MORRL
  / SKVL and the perturbed-start E cases (data fp2_data.tar).
- EDKF mass-flux shallow convection of the QNSE PBL (`mfshconv = 1`, WRF's default with bl_pbl_physics = 4), tier
  **verified** (suite cases Q1M / Q4M against WRF real*8, CPU and CUDA FP64): WRF's module_bl_mfshconvpbl.F (theta_l / r_t,
  the dry and cloudy updraft from the surface buoyancy flux with the BL89 upward length, the entrainment and
  detrainment of COMPUTE_ENTR_DETR, the explicit mass-flux transport of theta_l and r_t, the convective cloud water
  RC_MF, in the restart files) before QNSEPBL, with libmvec's pow lanes over the tile. Bit for bit with WRF's
  compiled MFSHCONVPBL in tests/phys/bl_mfshconv (ctest test_phys_bl_mfshconv). A QNSE namelist without mfshconv
  now runs WRF's default instead of stopping.
- BouLac PBL (bl_pbl_physics = 8), tier **verified**: WRF's module_bl_boulac.F (pbl_height, the length
  scales up / down of Bougeault and Lacarrere, the TKE equation and the implicit diffusion of u, v, theta, qv and qc
  with the Troen-Mahrt counter-gradient term) for one column, over the revised MM5 or the Eta surface layer, without
  the BEP urban terms; TKE_PBL, EL_PBL, EXCH_H / EXCH_M and the restart fluxes WU_TUR .. WQ_TUR as WRF writes them; no
  qi tendency (phy_bl_ten). Bit for bit with WRF's compiled routine in tests/phys/bl_boulac (ctest
  test_phys_bl_boulac) and suite cases BL1 / BL4 / BL9 against WRF real*8 (CPU and CUDA FP64 at tolerance 0).
  Shin-Hong (bl_pbl_physics = 11) stays an error.
- QNSE PBL (bl_pbl_physics = 4) and surface layer (sf_sfclay_physics = 4), tier **verified**: WRF's
  module_bl_qnsepbl.F (MYJ's TKE scheme with the QNSE mixing length in stable layers, the QNSE stability functions
  of the exchange coefficients, the production / dissipation of TKE with libmvec's pow lanes) and module_sf_qnsesfc.F
  (QNSESFCINIT's QNSE stability-function tables, PRT0, the topographic Zilitinkevich factor, THZ0 from the surface
  heat flux over land), with mfshconv = 0 (the EDKF mass flux: the entry above); QNSESFC_SEAICE_WRAPPER stays an
  error. Bit for bit with WRF's compiled routines in
  tests/phys/bl_qnse (ctest test_phys_bl_qnse) and suite cases Q1 / Q4 against WRF real*8 (CPU and CUDA FP64). New key `[phys] mfshconv` (default 1, as WRF's Registry).
- Verification tiers of options: `data/option_tiers.txt` lists option values merged before their whole-model WRF
  comparison as column_verified (bit-identical to WRF's Fortran in a column driver) or unverified (ported only).
  The model and cppwrf_real stop on such a value, naming option, value and tier, unless `[cppwrf] allow_unverified =
  true` (namelist.input: `&cppwrf allow_unverified = .true. /`, passed on by tools/wrf2cppwrf.py, which warns on
  them); with the switch each one is logged and the wrfout / wrfrst files get the global attribute
  `CPPWRF_UNVERIFIED_OPTIONS`. The table has no row: every option is verified, and no output byte changes. A new
  option's entry in this changelog names its tier.
- WSM3 and WSM5 microphysics, `mp_physics = 3` and `4` (phys/module_mp_wsm3.F, module_mp_wsm5.F): wsm3init /
  wsm5init, the semi-Lagrangian sedimentation, WRF's libmvec lanes of the vectorised saturation and diffusion
  loops, SNOWNC / SR, the effective radii for RRTMG (has_reqc = has_reqi = has_reqs = 1); real data. Tier:
  verified (suite cases W3 / W5 against WRF real*8, CPU and CUDA FP64 at tolerance 0; column driver
  tests/phys/mp_wsm35, bit for bit with the -r8 build's wsm3 / wsm5). Not ported (errors): `do_radar_ref = 1` with them, the ideal cases, WSM7 (24), WSM6R (106).
- RUC land surface model, `sf_surface_physics = 3` with 6 or 9 soil levels (phys/module_sf_ruclsm.F,
  module_sf_sfcdiags_ruclsm.F): RUCLSMINIT, LSMRUC's land, snow (one- and two-layer snow), sea-ice, land-ice and
  water points, the frozen-soil and snow-melt paths, SFCDIAGS_RUCLSM; the ruclsmscheme fields (SOILT1, SMFR3D,
  KEEPFR3DFLAG, RHOSNF, SNOWFALLAC, ACRUNOFF) and QVG / QSG / QCG / TSNAV / DEW in the history and restart files;
  USGS-RUC / MODI-RUC tables; input from WRF's real.exe. **Tier verified**: suite cases RUC4 (9 levels) and RUC4J
  (MYJSFC + MYJ, 6 levels), 3 steps, 1 h and restarted at 30 min, against WRF real*8 (CPU FP64 and CUDA FP64 at
  tolerance 0); column driver tests/phys/lsm_ruc (bit for bit with the -r8 build over six configurations). Not ported (errors): mosaic_lu /
  mosaic_soil 1, fractional_seaice 1, sst_update 1, the MYNN surface layer, nests, cppwrf_real's RUC soil levels.
- P3 microphysics with one ice category, `mp_physics = 50` (constant droplet number) and `51` (predicted droplet
  number QNCLOUD), phys/module_mp_p3.F: rime mass and volume QIR / QIB, TH_OLD / QV_OLD in the restart files, P3's
  REFL_10CM and effective radii for RRTMG; real data, one domain. Tier: verified (suite cases P3MP and P3NC against
  WRF real*8 of the -fdefault-double-8 build; column test tests/phys/mp_p3). Milbrandt-Yau (9), P3 52 / 53, P3 with
  nests, ideal cases or `do_radar_ref = 1` stay errors.
- Betts-Miller-Janjic cumulus (cu_physics = 2, WRF's module_cu_bmj.F): BMJINIT's look-up tables and SPLINE, BMJ's
  deep and shallow convection with the cloud efficiency CLDEFI (restart field), BMJDRV with its convective clouds
  (CCLDFRA, QCCONV, QICONV, CONVCLD; restart fields), phy_cu_ten (theta and qv), advance_ppt's HTOP / HBOT, and
  the adaptive time step's cudt = 0 (check_a_mundo); gfortran's libmvec lanes of the four vectorised loops.
  **Tier verified**: suite cases P9BMJ (3 steps, 1 h, restarted, cudt 5) and P4BMJ against WRF real*8 (CPU FP64
  and CUDA FP64 at tolerance 0); column driver tests/phys/cu_bmj (ctest test_phys_cu_bmj, label diff).
- bmj_rad_feedback (per domain, with BMJ): BMJ's clouds in the radiation (module_radiation_driver.F: QCCONV / QICONV
  added to qc / qi, CCLDFRA combined with CLDFRA); needs a microphysics scheme with cloud water and ice when the
  radiation is on. **Tier verified** (suite case P4BMJF, CPU and CUDA FP64 at tolerance 0).

- Single-layer urban canopy model, `sf_urban_physics = 1` (phys/module_sf_urban.F urban_param_init,
  urban_var_init and SUBROUTINE urban with SFCDIF_URB, called by the Noah LSM as module_sf_noahdrv.F does), with
  URBPARM.TBL or, with `use_wudapt_lcz = 1`, URBPARM_LCZ.TBL and their default options; the surface driver's urban
  10 m wind, PSIM / PSIH / GZ1OZ0, AKHS and AKMS; the UCM state, FRC_URB2D and UTYPE_URB in the wrfout / wrfrst
  files. One domain, Noah only, no restart. **Tier verified**: suite cases U4 (URBPARM.TBL) and U4L (URBPARM_LCZ.TBL),
  3 steps and 1 h against WRF real*8 (CPU FP64 and CUDA FP64 at tolerance 0); column driver tests/phys/sf_urban
  (WRF's urban_param_init and urban of the -r8 build, 9000 point-steps per table, tolerance 0). BEP / BEP+BEM (2, 3), slucm_distributed_drag, NUDAPT morphology, the other
  table options, nests, restarts and cppwrf_real's urban input stay errors.
- Domain decomposition with divided memory, and MPI (docs/MPI.md, USER_GUIDE section 7): a domain and its nests run
  as P x Q patches, each worker holding its patch and a halo of 3 points under the domain's indices. The workers
  are tiles (`run.tiles`, as before, now also with nests) or MPI ranks: a build with `-DCPPWRF_MPI=ON` (off by
  default) under `mpirun -np N`, layout `run.ranks = PxQ` / `CPPWRF_RANKS` (default the most square with P <= Q),
  `CPPWRF_RANK_DEVICES` for the GPUs of a CUDA build. One worker writes every output file. The result has the same
  bits for every worker count and layout (CPU FP64 = one-process WRF real*8): proved on one machine for 1 to 8
  ranks in 13 layouts on eight cases, 53 suite cases in three layouts, and up to 48 ranks against the one-process
  run; not run on several GPUs or nodes. A real-data start reads only the worker's window of wrfinput's 3-D
  variables. Results without tiles and outside `mpirun` are unchanged.
- tools/colab/colab_mpi.sh (suite cases under `mpirun` with several layouts on a Colab runtime, CPU and GPU, each
  against the WRF references and byte for byte against one rank) and tools/bench/colab_bench.sh `--scaling` (strong
  scaling next to WRF real*8 dmpar).
- Skin SST (`sst_skin = 1`) and deep soil temperature update (`tmn_update = 1`, `lagday`) as WRF's
  module_sf_sstskin.F, module_sf_tmnupdate.F and their surface_driver calls, with the Noah LSM: SSTSK / DTW and TYR,
  TYRA, TDLY, TLAG, NYEAR, NDAY in the restart files (tmn_update resumes from wrfrst files only). Tier: verified
  (suite cases SKV, SKU, SKN against WRF real*8, SKV also restarted at 3 h against WRF's run restarted there). The
  other land surface models, `lagday = 1` and the adaptive time step stay errors.
- Drop-in `wrf.exe` and `real.exe`: `cppwrf-wrf`, `cppwrf-realexe` and `cppwrf-check` run in a WRF run directory and
  write WRF's file names there, with rsl logs, WRF's timing lines and SUCCESS COMPLETE; `output.wrf_dir`,
  `output.wrf_timing`, `restart.native` and `cppwrf --version` for them; suite cases dropin_*. With `restart =
  .true.`, `override_restart_timers` and `write_hist_at_0h_rst` stop with an error (WRF's resumed alarms with them
  are not ported).
- Install and packaging: `cmake --install` (bin/cppwrf-<flavour>, share/cppwrf data, tools, python, verify, docs),
  the data directory found from the binary (`CPPWRF_DATA_DIR` overrides), CPack tarballs per flavour and CUDA
  architectures, Docker and Apptainer recipes, a Spack recipe, tools/release/make_release.sh.
- Python package `cppwrf` (python/, docs/PYTHON.md): namelists (read, write, translate through tools/wrf2cppwrf.py,
  check), runs (start, progress from the rsl log, cancel, wait, typed errors), `open_wrfout` (xarray or numpy) and
  diagnostics (T2 in C, 10 m wind, wrf-python's slp, precipitation with buckets).
- User tools (docs/TOOLS.md): `cppwrf-compare` (two sets of WRF output files with the suite's comparison),
  `cppwrf-bench` (an em_quarter_ss benchmark per installed mode), `cppwrf-fit` (memory and time of a namelist from
  the bench results), bash completion.
- User documentation: docs/INSTALL.md, QUICKSTART.md, MIGRATING_FROM_WRF.md (option tables generated from the
  translator, ctest docs_options_current), FAQ.md, Turkish KURULUM.md and HIZLI_BASLANGIC.md; examples/quickstart
  through cppwrf-realexe and cppwrf-wrf; examples/selftest with the FP64 value hash.
- Repository files: issue and pull request templates, CONTRIBUTING, SECURITY and the Code of Conduct.

### Fixed
- Namelist options that changed WRF's run and were dropped or misread: `scalar_adv_opt` 0 / 2 / 3 / 4 now reaches the
  model also without the column physics (ideal cases, real runs with microphysics only; 3 / 4 advect theta with WENO
  on the last RK stage in dry runs too); a nest's `diff_6th_factor` reaches it when domain 1 has `diff_6th_opt = 0`;
  `h_sca_adv_order` / `v_sca_adv_order` different from the momentum orders on a nest is an error (it was checked on
  domain 1 only); a nest's missing `start_*` / `end_*` entry is WRF's Registry default (an error, as every per-domain
  start or end time), a nest whose `end_*` is before domain 1's stop time with `run_*` is an error (WRF stops it
  there), and without `run_*` the run length takes the Registry default for a date component the namelist leaves
  out (WRFU_TimeSet carries minutes and seconds over); `sst_update = 1` without the column physics is an error in the
  translator and the model (WRF reads wrflowinp into the history fields; CPPWRF did not). cppwrf_real stops on
  `sf_lake_physics` /= 0, `sf_ocean_physics = 2`, `seaice_albedo_opt = 2`, `seaice_snowdepth_opt = 1` and
  `seaice_thickness_opt = 1` instead of writing real.exe's LAKE_DEPTH, OM_*, ALBSI, SNOWSI, ICEDEPTH fields as zeros.
  No verified configuration changes.
- Restarts between the radiation, PBL and cumulus calls (radt, bldt, cudt > dt): the resumed run now carries, from
  WRF's wrfrst files and CPPWRF's own restart file, the restart-stream state of the last calls that WRF carries:
  O3RAD (RRTMG; a nest's radiation reads domain 1's, so a nest whose radiation ran before domain 1's after the
  restart used zero ozone and left the uninterrupted run), HRANG, DECLIN / SOLCON (also without slope_rad), URATX,
  VRATX, TRATX, RA, RS (Noah, slab, RUC), GWDO's DTAUX3D / DTAUY3D / DUSFCG / DVSFCG, KF's SHALL and AKPBL, which
  is now KPBL of the last cumulus call as in cumulus_driver (it was the current KPBL). They were 0 in the wrfrst
  files and history of the resumed run until the next call. A wrfrst restart leaves grid%ch at 0 as WRF (it read
  Noah-MP's CH = CHXY into it); DTBC of an ideal domain grows from the start as solve_em's (it wrapped at
  interval_seconds). CPPWRF's restart file (format version 4; version 3 files are refused) also holds WW, WW_M,
  PHP and ALT, as the wrfrst file, and (one patch) the halo points of its fields as the run had them (HALO_<name>):
  the dycore fills them at fixed points of the step, so after the step's last updates they are not a copy of the
  domain's values, and a run resumed off a radiation step read them (SR1 at 245 min: one boundary column). Suite: restart chains (tests/verify/cases/66_restart_chain.py:
  D0, P4, G2, P8, NMP1, SLAB1, SR1, N3 resumed from both files against the straight run, every later wrfout frame
  and wrfrst file at tolerance 0) and SR1 resumed from its own restart file at 245 min (`real_sr1_r8_fp64_cpu_5h_nrst`). From a wrfrst file SR1 resumes
  as WRF's resumed run does: WRF's SHADOWMASK (topo_shading) is in no restart stream, so its resumed run casts no
  shadow until the next radiation call; the SR1 chain therefore checks the native resume only.
- CPPWRF's own restart file (`restart_<step>.nc`, format version 3; files of version 2 are refused, the run's
  wrfrst files still resume): it now holds the scalars of WRF's restart stream, NSTEPS (output_diagnostics' means)
  and DECLIN / SOLCON of the last radiation call (read by slope_rad's TOPO_RAD_ADJ and the urban canopy model until
  the next call: a run resumed between two radiation calls used 0), as global attributes SCALAR_<name>, and
  topo_shading's SHADOWMASK (WRF's restart stream has none: computed again at the next radiation call);
  tests/io_netcdf/test_restart.cpp checks the scalars round trip. A run resumed from that file between two
  radiation calls was not yet identical to the uninterrupted run; format version 4 (entry above) resolves it.
- The restart alarm of a run resumed from a restart file (WRF's or CPPWRF's own) with a restart interval that is not
  a multiple of the time step: the first ring is at the restart time + WRF_ALARM_SECS_TIL_NEXT_RING_51 and the later
  ones are counted from that ring's step (share/input_wrf.F, ESMF_ClockAdvance), so the wrfrst files and their alarm
  attributes are WRF's; with restart_interval = 0 the resumed run writes the one set WRF writes at that ring. CPPWRF's
  restart_<step>.nc files are format version 2 (they carry WRF_ALARM_SECS_TIL_NEXT_RING_51). New suite cases RSA
  (P4, dt 50 s, restart every 2 min, the wrfrst files compared too) and its resumed runs, CPU and CUDA FP64 at
  tolerance 0 (data port_af2_data.tar); ctest restart_alarm.
- auxinput4_interval (sst_update) and gfdda_interval_m (grid_fdda) that are not multiples of the domain's time step:
  WRF stops at the first read ("Could not find matching time in input file"); tools/wrf2cppwrf.py and the model now
  refuse them (before, the model ran on with the first record). ctest input_interval.
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
- Acoustic step count with `time_step_sound = 0`: computed in REAL as solve_em does, so in float in the FP32 build,
  as in WRF real*4 (dx 10 km, dt 67 s: 6 steps, not 4); FP64 results unchanged (tests/unit/test_sound_steps.cpp).
- FP32 builds: WSM6, the nest interpolation (SINT, boundary slabs) and the vm math functions compute in float in
  CUDA builds, and MYNN-EDMF's DOUBLE PRECISION locals stay double, as in WRF real*4. FP64 and CPU builds unchanged.
- RRTMG LW / SW with WSM3: WRF's "MP option 3" hydrometeor split (no F_QI, not warm_rain) on the host and CUDA
  paths.
- The whole CUDA build compiles with CUDA 12.4 (real_case's namelist / input checks without a range-for over a braced
  list).
- Python test fixtures and the self-test read the Times characters with numpy 2.3.
- The translator names CAM radiation (`ra_lw_physics` / `ra_sw_physics = 3`, module_ra_cam.F not ported) in its
  error.
- `cppwrf_real` with met_em input from other sources than the suite's GFS sets, each as module_initialize_real.F /
  input_wrf.F do it: sea ice together with an SST field (adjust_for_seaice_post took grid%tsk / grid%tsk_save the
  other way round; TSK and TSLB at the ice points differed by up to 16.4 K); land use without an inland-lake category
  (`ISLAKE = -1`: USGS 24, MODIS 20, NLCD40; lakeflag 0 and the SST repair at water points); specific humidity
  (`FLAG_SH`, SPECHUMD) and mixing ratio (`FLAG_QV`) instead of RH, and `use_sh_qv`; sfcprs2 (`sfcp_to_sfcp`, top-down
  level input) and GHT rebuilt hypsometrically where it is missing; PSFC / PMSL read only when present; the met_em
  fields real.exe copies through unchanged (CANWAT, LAKE_DEPTH, FRC_URB2D, ELVMAX, the gwd_opt 3 fields, Noah-MP's
  static fields, ...); liquid soil water SW. Checks real.exe makes: BOTTOM-TOP / WEST-EAST / SOUTH-NORTH_GRID_DIMENSION
  against the namelist (the level check read an attribute metgrid does not write and always passed), DX / DY,
  MMINLU and its category defaults, one time per met_em file, the land use and soil category counts,
  num_land_cat / num_metgrid_soil_levels, keys a namelist group does not have, a reference pressure that does not
  decrease in a column on the hybrid coordinate; met_em reads are bounds-checked. check_a_mundo's rewrites
  (`lagrange_order` 1 for few levels, `default_soiltype`, `hypsometric_opt = 2`). Existing references are identical;
  the new inputs have WRF real.exe references (ctest real_vs_wrf_runs, below). Still refused: surface pressure from
  PMSL alone (WRF's sfcprs path) and TAVGSFC.
- Namelist checks, translator and model: values `wrf.exe` runs differently are refused (`use_theta_m` other than 1,
  `km_opt = 0` with `diff_opt` /= 0, `wif_input_opt` 2 with any `mp_physics`, `spec_bdy_width` other than 5 with
  nests, `s_we` / `s_sn` / `s_vert` other than 1, a tslist file, `shadlen <= 0`, `io_form_restart` other than 2 and
  `sf_urban_physics` with `restart = .true.`); check_a_mundo's derivations are applied (`num_soil_layers` from domain
  1's land surface model, `icloud = 1` on every domain when one runs MSKF, `cu_diag`, Registry defaults of
  `interval_seconds`, `e_vert` and the start date); `sf_surface_physics = 0` with a surface layer or PBL scheme is an
  error, as the model stops there. wrfout / wrfrst global attributes as WRF writes them (d01's PARENT_ID /
  PARENT_GRID_RATIO, MMINLU, WRF_ALARM_SECS_TIL_NEXT_RING_55 by whole clock seconds, the Noah-MP OPT_* of INI runs)
  and the stream contents of `iofields` files and auxiliary streams; WRF references for these namelists (files test
  cases g9f*, suite cases G9F2 and G9F6C).
- Nests: the parent's phy_init runs again after the feedback at its first step, as start_em does (landuse_init,
  TSK_SAVE, XICEM, SNOALB from the fed-back land use; suite case N3F1); the parent's and the nest's scalars are paired
  by name in med_nest_force / med_nest_feedback (tke and MYNN's qke_adv with per-domain `km_opt`; suite case N3M);
  per-domain physics WRF does not run are refused by the model and the translator.
- Decomposed runs (tiles, MPI ranks): the scalars' halo exchanged after spec_bdy_final, as solve_em; the periodic
  wrap of a direction of 6 or 7 points in two patches (ctest tiles_wrap); `diff_opt = 2` stresses at the halo point
  beyond a symmetric side; the base state's halos after a start from a wrfrst file filled as start_em does.
- GPU and kernel paths: MSKF's out-of-range levels and `NINT` (gfortran's on x86-64, also on the device); WENO
  weights, eps and fluxes in double as WRF's DOUBLE PRECISION locals; NSSL over the worker's rows with a bounded work
  space; RRTMG LW / SW with `icloud = 0` take a clear column also when `bmj_rad_feedback` set CLDFRA (suite case
  P4BMJFI0); the radiation's boundary-layer clouds without QI (CPU and CUDA as WRF); Thompson's ice fall speed folded
  as gfortran folds it on the device too; MYNN's ktop_plume initialised; coherent loads for the arrays an RRTMG batch
  kernel writes before it reads them; BouLac's and the TKE kernels' column arrays sized and checked by the level
  count; WSM5's and the urban table's set-up through the model's libm on the host.
- FP32 builds follow WRF real*4 more closely: the RRTMG LW / SW tables as WRF real*4 builds them, Thompson's
  REAL(KIND=R8SIZE) tables and DOUBLE PRECISION process rates in double, MSKF's momentum transport in REAL(r8), P3's
  double temporaries, WSM / WDM rgmma in single precision, grid%julian from the double day of year and the CLWRF trace
  gases in double, and WRF real*4's single-precision constants and products (small_step's cof, sfclayrev's 1./1.1,
  x**2.0, NSSL's dgam, Noah-MP groundwater, RRTMG SW heatfac). RRTMG (`rr`), WSM6's literals, the nest SINT and the
  `vm` atan / sin / cos / log10 are single precision in CPU FP32 builds too. FP64 output is unchanged; the GPU FP32
  values of the suite moved (docs/FP32_ACCURACY.md, section 1.1.0).

### Performance
- RRTMG LW / SW (and the radiation driver's RRTMG paths, Dudhia SW) compute in single precision in CUDA FP32 builds:
  the type `rr` (src/phys/ra_rrtmg_common/rrtmg_real.hpp) is float there, as RRTMG's `rb` in WRF real*4, and double
  in CUDA FP64 and in every CPU build, whose results are unchanged (CUDA FP64 at tolerance 0, CPU FP32
  byte-identical). RRTM LW stays double. GPU FP32 output bytes change; its distance from WRF real*8 stays at WRF
  real*4's (P4 72 h, N3 24 h). GPU FP32 on the Colab G4: P4 72 h 2.0x, N3 24 h 1.65x faster than the GPU FP32
  build with RRTMG in double precision (the commit before this change).
- GPU memory: the column arrays of Tiedtke, New Tiedtke, Grell-Freitas-Li and GWDO (FP32 and FP64) and of
  Kain-Fritsch and MYJ (FP64) live in the column work buffer instead of the kernels' stack frames, which CUDA
  reserves for every thread the GPU can hold. The frames of 20-80 KB per thread are gone; the largest left is
  YSU's / Noah's (11-12 KB in FP64). A 1 h run on 120 x 90 x 45 peaks 4.7-15 GB lower on an RTX PRO 6000 with these
  schemes (WRF's CONUS suite in FP64: 20.7 -> 8.7 GB) and 1.1-3.1 GB lower on an L4; every verified suite fits a
  6 GB GPU in FP32 and FP64 (docs/USER_GUIDE.md "GPU memory" has the minimum per suite, docs/PERF.md the tables).
  The moved kernels are up to 11 % (FP64) and 30 % (FP32) slower on the RTX PRO 6000; the steps of Grell-Freitas
  and Tiedtke runs 3-7 %. Results are unchanged: CUDA FP64 at tolerance 0, FP32 byte-identical.
- `CPPWRF_DEVICE_MEMORY_MIB=N`: the work buffers (column physics, RRTMG) are sized as on a device of N MiB.
- tools/colab/colab_run.sh `--base REV` (both builds in one session: times, memory, kernel times, outputs byte for
  byte) and `--device-mib N` (the runs see N MiB of free device memory).

### Changed
- Tiles no longer write `output.dir.tileN` copies: tile 0 writes the run's files from gathered fields.
- An MPI run prints the peak host memory of the writer, of the largest other rank and of all ranks.
- The CUDA build needs CUDA 12.1 or later and a GPU of compute capability 7.0 or later: the batched launches pass
  kernel parameters above 4 KB. CMake stops on an older CUDA with that reason.

### Verification
- New suite cases against WRF v4.8.0 real*8 (tests/verify/cases/69_coverage.py, data port_cov1_data.tar and
  port_cov1b_data.tar; 3 steps and 1 h on the CPU, 1 h on the GPU, every variable at tolerance 0) for option values
  that had no suite case: RRTMG `cldovrlp` 1, 3, 4 (with `idcor` 1) and 5 (OVL1, OVL3, OVL4I, OVL5), `icloud` 0
  (ICL0), `diff_6th_opt` 1 (D6O1), `km_opt` 3 with `diff_opt` 1 (KM3H), `tke_adv_opt` 0 with `km_opt` 2 (TKA0),
  `moist_adv_opt` / `scalar_adv_opt` 0 (ADV0) and `bl_mynn_cloudpdf` -1 / -2 (MPDFM1, MPDFM2). `km_opt` 3 with
  `diff_opt` 2 (KM3) is compared over two steps only: on this 15 km domain WRF itself blows up in the third step
  (NaN in every field, which wrf.exe does not stop on); CPPWRF is identical up to that point and then stops with its
  non-finite check. All verified tier.
- Verification coverage (docs/STATUS.md): the audit's order check (serial reversed and i-outer launch order against
  the OpenMP run) and the pattern-initialised, bounds-checked build on the configurations whose inputs were not on
  the workstation, and an NVIDIA compute-sanitizer pass (memcheck, racecheck, initcheck, synccheck; FP64 and FP32)
  over P4, MYNN, Thompson, Morrison, RRTMG, the two-way nest, a 2x2 tiled run and an ideal case.

### Tools and documentation
- The CPU FP64 suite on free GitHub-hosted runners (docs/USER_GUIDE.md, "The CPU Suite on GitHub"): the reference
  data (the tars of tests/verify/vm_data.txt) are public release assets of github.com/Xassemblianist/cppwrf-data,
  whose workflow suite.yml runs wrf_suite.py's CPU cases, the output files tests, the ideal files test, the
  differential and ctest tests, the translator tests and the comparators' self-test in a Debian 13 container, a
  gate failing unless every expected case PASSed. tools/ci/ci_submit.sh runs it for any local commit (a git bundle
  of the commits origin/main lacks) and prints suite_status.sh's summary; .github/workflows/wrf_suite.yml runs it on
  every push to main. Its workflow wrfref.yml builds WRF v4.8.0 as the references were built (wrf.exe with the Intel
  vector log / log10 of tests/phys/libmvec_intel.cpp preloaded) and makes WRF references or runs existing ones again
  (R1 1 h reproduced byte for byte on an AMD runner). Colab (tools/colab) stays the other way, and the one for GPU
  runs.
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
- FP32 tolerances of ACM7 (W) and Q4 (U, QV) re-measured on the Colab G4 at fb158f3, after the FP32 fixes above, and
  set to about 3x the measured values (W 3.0e-2; U 6.5e-4, QV 5.6e-4).
- Suite cases for the nest and radiation fixes against WRF v4.8.0 real*8: P4BMJFI0 (P4BMJF with `icloud = 0`, CPU
  and CUDA FP64), N3F1 (d02 land use differing at the fed-back points) and N3M (MYNN on the two-way nest with
  `km_opt` 4 / 2). N3M's reference is from WRF-4.8.0-r8d8 (`-fdefault-real-8 -fdefault-double-8`), as every MYNN
  case: the stock `-r8` build computes MYNN's DOUBLE PRECISION locals in real*16.
- CUDA FP64 twins of the CPU FP64 cases (tolerance 0): the Noah-MP option cases, the terrain cases TW2, SR2, SR3,
  NSSL's N17, the advection, P3, urban, Morrison, Grell-3D, FDDA, skin SST, ACM2 and BouLac cases added since 1.0.0.
- ctest real_vs_wrf_runs: `cppwrf_real` against WRF real.exe on 14 runs (plain P1 met_em, `lagrange_order` 1,
  `default_soiltype`, VAR_SSO input, and ten met_em input variants: sea ice with SST, `ISLAKE = -1`, specific
  humidity with `use_sh_qv` false and true, no GHT, top-down levels, `sfcp_to_sfcp`, pass-through fields, soil
  water). For `ISLAKE = -1` WRF's wrf.exe stops on that artificial input ("too many input landuse types"), so only
  real.exe's output is compared. tests/verify/ctest_data.txt names the data tars of the ctest tests.

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
