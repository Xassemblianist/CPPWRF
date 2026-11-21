# CPPWRF 1.2.0 release notes

CPPWRF 1.2.0 is a C++20 / CUDA port of WRF-ARW v4.8.0. As before, the CPU FP64 and CUDA FP64 builds reproduce WRF
v4.8.0 real*8 bit for bit in every verified configuration, and their results are unchanged from 1.1.1. New in this
release: the FP32 build is bit-identical to WRF v4.8.0 real*4 on 24 configurations, on the CPU and, with an opt-in
CUDA build, on the GPU, and its result no longer depends on the host's libm. It covers 31 commits (not counting
merges) from `v1.1.1` (7629981) to `main` 0bc4652. The 1.1.1 notes follow below.

## Highlights

- **FP32 bit-identical to WRF real*4.** The CPU FP32 build (`-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF`) gives the output of stock WRF
  v4.8.0 real*4 (gfortran, WRF's default real*4 build), every history variable and every bit, on 24 configurations:
  P1, P2, P3, P4, P9, Noah-MP (NMP1), Thompson (P6), MYNN, the aerosol-aware Thompson (C01), WDM6, WDM5, Morrison
  with hail and with graupel (MORR, MORG), the gravity wave drag (G2), the one-way nest N1, the two-way nest N3,
  monotonic advection on N3 (MON3), Kessler, ACM2 + Pleim-Xiu, QNSE, Tiedtke, New Tiedtke, the Dudhia shortwave and
  the slab LSM. docs/FP32_IDENTITY.md is the full description: the method, the build switches, the case list, the
  ported glibc functions and what is not identical yet.
- **The same bits on the GPU:** the CUDA FP32 identity build (`-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF
  -DCPPWRF_CUDA_FP32_IDENTITY=ON`) gives the CPU FP32 build's results, and so WRF real*4's on these configurations.
  It turns FMA contraction off in every kernel and runs glibc 2.41's float functions on the device. On an RTX 2060
  it costs 3 % of kernel time on P4. The default CUDA FP32 build stays the fast one, checked against measured
  tolerances (docs/FP32_ACCURACY.md).
- **FP32 independent of the host's libm:** the FP32 executables define glibc 2.41's float functions themselves
  (src/core/glibc_mathf_host.cpp), each proven equal to glibc 2.41's libm on every float input. Binaries built on
  Ubuntu 24.04 (glibc 2.39) reproduce the WRF real*4 references, which were made against Debian 13's glibc 2.41.
- **GPU performance work with the same results:** fewer kernels and less memory traffic in the RK3 step, the vertical
  solve, the nest feedback, Noah, RRTMG LW and the surface driver (Changed). FP64 output is bit-identical to before.

## Added

- **CPU FP32 identity with WRF v4.8.0 real*4.** The FP32 build rounds where WRF real*4 rounds: start_em's base state,
  the lateral boundary coefficients, Noah's initialisation, history T, the initialisation constants of WSM6,
  Thompson, WDM5 / WDM6 and Morrison, the MYNN surface layer's tables, moist_physics_prep_em's R_v/R_d, the nest start
  (blend_terrain, adjust_tempqv, press_adj) and the feedback (copy_fcn's average, sm121, smdsm). gfortran's vectorised
  loops call glibc libmvec's 4-lane SSE4 float functions (expf, logf, powf, sinf, cosf, atanf); CPPWRF ports them lane
  by lane (src/core/libmvec_f4.hpp) and calls them for the same loop iterations.
- **Suite backend `cpu_sp`** (`tests/verify/wrf_suite.py --cpu-sp BIN`): 44 cases, the 24 configurations above at
  1 h and 20 of them also over 3 minutes with history every step (every minute on the nests of N3 and MON3),
  compared with WRF real*4 history files at tolerance 0. New data tars fp32id_data.tar, fp32id2_data.tar,
  fp32id3_data.tar and fp32id4_data.tar hold the new WRF real*4 references; the 1 h references of P1, P2, P3, P9 and
  NMP1 are earlier real*4 runs in suite_data.tar.
- **CUDA FP32 identity build**, CMake option `CPPWRF_CUDA_FP32_IDENTITY` (default `OFF`; needs `CPPWRF_ENABLE_CUDA=ON`,
  `CPPWRF_DOUBLE=OFF` and `CPPWRF_CUDA_GLIBC_MATH=ON`): `--fmad=false` in every kernel, glibc 2.41's scalar float
  functions on the device (from libm's FMA variants and from CORE-MATH's correctly rounded sources), glibc's double
  functions as in the CUDA FP64 build, libmvec's 4-lane float functions with the CPU build's lane rule, and
  `Reducer::sum` in the CPU build's order. It installs as the `gpu-fp32` flavour.
- **Suite backend `cuda_sp_id`** (`--cuda-sp-id BIN`): the 44 `cpu_sp` cases on the GPU (`..._cuda_id...`) against
  the same WRF real*4 references at tolerance 0.
- **Proofs of the math ports:** tools/glibc_mathf_proof.cpp (each port against glibc 2.41's libm on every float input,
  2^32 pairs plus special values for powf and atan2f), tools/glibc_mathf_proof_cuda.cu (the device against the host),
  test_libmvec_f4 and test_glibc_mathf in ctest; the ctest `fp32_host_libm` checks that the FP32 `cppwrf` imports none
  of the float functions it defines.

## Changed

- **GPU performance** (the same operations in the same order; FP64 bit-identical on the Colab G4, 338 / 338 checks at
  b2d320c):
  - KWSolve carries the neighbour level of w in a register; pi = 4 atan(1) is computed once on the host.
  - An RK3 step with microphysics and lateral boundaries runs calc_p_rho_phi once, as WRF's solve_em does.
  - The nest feedback synchronises the device only when CUDA graphs are off, or at the run's last root step.
  - Noah reads its parameter tables through a device pointer instead of a by-value copy in the kernel arguments.
  - RRTMG LW's batch kernel forms the band sums and the level fluxes in registers.
  - The surface driver no longer zeroes u_phytmp / v_phytmp.

  On the G4 the 1 h stepping times stayed within a second (P4 FP64 9 s; N3 FP64 16 -> 15 s; P4 FP32 3 s; N2 FP32
  3 -> 2 s); the gains showed on the smaller RTX 2060. A change of NSSL's kernel launch in the same work made NSSL
  3 to 8 times slower on the G4 (P4N18 1 h: 47 -> 135 s FP64, 19 -> 165 s FP32) and was reverted: NSSL keeps 1.1's
  launch. No new benchmark was run; the numbers of docs/PERF.md and README.md are those of 1.1.
- **FP32 results:** the FP32 builds now round as WRF real*4 does in the places listed under Added, so their output
  differs from 1.1.1's. The fast CUDA FP32 build shares these code paths; its suite tolerances are unchanged. The
  FP64 builds are unchanged.

## Fixed

- **FP32 results depended on the host's libm.** glibc 2.40 and 2.41 replaced several float functions with CORE-MATH's
  correctly rounded ones; an FP32 build on Ubuntu 24.04 (glibc 2.39) differed from WRF real*4 from t = 0. The FP32
  executables (`cppwrf`, `cppwrf_real`; the CPU build and the host code of the CUDA builds) now define glibc 2.41's
  expf, logf, powf, sinf, cosf, sincosf, tanf, atanf, atan2f, asinf, acosf, sinhf, tanhf, log10f, erff and tgammaf
  themselves. On a Colab G4 (Ubuntu-built binaries: gcc 13.3, glibc 2.39) the FP32 cases failed before this change
  and pass after it.

## Verification

Full CPU CI on 0bc4652 (GitHub-hosted runners, Debian 13 container, run 44860146094), all exact (tolerance 0), and
the GPU proofs:

| Check | Result |
|---|---|
| `tests/verify/wrf_suite.py`: CPU FP64 against WRF v4.8.0 real*8 and CPU FP32 (`cpu_sp`) against WRF v4.8.0 real*4, at tolerance 0 | PASS 386, FAIL 0, SKIP 0 (342 CPU FP64 checks and the 44 `cpu_sp` cases) |
| WRF output files (wrfout / wrfrst values, attributes, metadata) | PASS 42 |
| Ideal-case output files | PASS 14 |
| Differential tests against WRF's compiled Fortran (`ctest -L diff`) | PASS 32 / 32 |
| ctest | PASS 35 / 35 |
| Host-libm independence: Colab G4, NVIDIA RTX PRO 6000 Blackwell Server Edition (sm_120), CUDA 13.0, binaries built on Ubuntu 24.04 (gcc 13.3, glibc 2.39), 2026-11-21 | `cpu_sp` 44 / 44 and `cuda_sp_id` 44 / 44 at tolerance 0 |
| GPU: Colab G4 at 0bc4652 (NVIDIA RTX PRO 6000 Blackwell Server Edition, sm_120, CUDA 13.0), 2026-11-21 | CUDA FP64 338 / 338 at tolerance 0 (319 cases, 18 restart checks, Straka); CUDA FP32 identity (`cuda_sp_id`) 44 / 44 at tolerance 0; fast CUDA FP32 44 / 44 within the measured tolerances; ctest 37 / 37 (FP64) and 38 / 38 (FP32); device glibc math proof identical |

How the suite counts relate (`tests/verify/wrf_suite.py --list` at 0bc4652): the suite defines 927 cases, 477 CPU
FP64, 319 CUDA FP64, 43 CUDA FP32, 44 CPU FP32 (`cpu_sp`) and 44 CUDA FP32 identity (`cuda_sp_id`), and 18 restart
configurations, each with a CPU and a CUDA FP64 check. The CI's 342 CPU FP64 checks are, as in 1.1, the 323 CPU FP64
cases outside `--slow`, the 18 CPU restart checks and the Straka check; the 154 slow CPU cases run on request. The
`cuda_sp_id` cases also all passed on an RTX 2060 (sm_75) during development (docs/FP32_IDENTITY.md).

## Known limitations

- **Tiles with the cumulus diagnostics:** `run.tiles` (host threads of one process) together with `cu_diag = 1`
  (Grell-Freitas, Grell-3D) shares one process-wide counter of convtrans_prep's averaging between the tile threads;
  such a run is not checked and should not be used. One tile, or MPI ranks, are not affected.
- **The fast CUDA FP32 build** (the default) is not bit-identical to anything: FMA contraction and CUDA's math
  library. It is checked against WRF with measured tolerances (docs/FP32_ACCURACY.md).
- **On the identity build's device** hypotf is CUDA's (no FP32 code calls it).
- **FP32 identity needs a WRF real*4 reference:** only the configurations of docs/FP32_IDENTITY.md are checked bit for
  bit. Other configurations are checked against WRF real*8 with tolerances, or not at all in FP32.
- **Not covered by FP32 identity:** moving nests, more than two domains, vertical refinement, restarts, and the
  options listed in docs/FP32_IDENTITY.md, "What is not identical yet".
- **The WRF reference compiler:** the references are gfortran builds of WRF; WRF built with other compilers (or with
  AVX2, whose loops call 8-lane functions) is not covered.
- **MPI:** several GPUs and several nodes have not been run. Several MPI ranks on one GPU are not proved
  (docs/MPI.md).
- The limitations of 1.1.0 below otherwise hold (scheme scope, the N3 72 h comparison, the options not ported).

## Outlook

Next come a new benchmark report against WRF on current hardware, runs on several GPUs, and later a port to AMD GPUs
through HIP.

## Requirements

As 1.1.0: a C++20 compiler, CMake >= 3.24, OpenMP and NetCDF-C. For the GPU builds, the CUDA toolkit >= 12.1 and an
NVIDIA GPU of compute capability 7.0 or later. An MPI library for `-DCPPWRF_MPI=ON`. Python 3 with `netCDF4` and
`numpy` for the tools.

---

# CPPWRF 1.1.1 release notes

CPPWRF 1.1.1 is a fix release of 1.1.0. Model results are unchanged: the CPU FP64 and CUDA FP64 builds give the same
output as 1.1.0 for every namelist 1.1.0 runs.

## Added

- **`cppwrf_real`: TAVGSFC input** (`FLAG_TAVGSFC = 1`), as real.exe uses it: the surface pressure of `sfcp_to_sfcp`
  (sfcprs2's ez_method), the SST and skin temperature of inland-lake points, the SST repair of water points without
  lake information (`ISLAKE = -1`), the slab's TMN and its terrain adjustment. `use_tavg_for_tsk` stops only when the
  input has neither TSK nor TAVGSFC, as in WRF.
- **`cppwrf_real`: surface pressure from PMSL** (sfcprs) when the met_em files have no PSFC or no SOILHGT, with WRF's
  850 / 700 / 500 hPa lapse rates, its fallback without those levels, and the TAVGSFC variant.
- Both are compared with WRF v4.8.0 real.exe (real*8) at tolerance 0 in eight new runs of ctest `real_vs_wrf_runs`
  (wrfinput and wrfbdy identical): V111TAV, V111TAVT (`use_tavg_for_tsk`), V111TS2P (`sfcp_to_sfcp`), V111TSLAB (the
  slab), V111TNL (`ISLAKE = -1`), V111PMS (no PSFC), V111PMT (no PSFC, with TAVGSFC), V111NSH (no SOILHGT).

## Fixed

- **Translator:** a namelist with cumulus or radiation physics but no surface layer and land surface model was
  translated, and `cppwrf` then stopped at its start. It is now an error at translation time with the model's
  message. wrf.exe runs such namelists; CPPWRF's column physics needs both schemes (docs/USER_GUIDE.md, "Rejected
  Options and Why").

## Verification

Full CPU CI on the release commit's parent ade4b90 (GitHub-hosted runners): suite PASS 342, WRF output files 42,
ideal-case files 14, differential tests 32 / 32, ctest 33 / 33 (real_vs_wrf_runs: 22 WRF real.exe runs), all at
tolerance 0. The CUDA code is unchanged from 1.1.0 (its G4 proof holds).

---

# CPPWRF 1.1.0 release notes

CPPWRF 1.1.0 is a C++20 / CUDA port of WRF-ARW v4.8.0. As in 1.0.0, the CPU FP64 and CUDA FP64 builds reproduce
WRF v4.8.0 real*8 bit for bit in every verified configuration. This release adds microphysics, cumulus, PBL,
land-surface, urban and advection options, domain decomposition with MPI ranks, drop-in replacements for `wrf.exe`
and `real.exe`, Noah-MP option values and terrain options, `cppwrf_real` support for more met_em inputs, the fixes
of an independent code review, and single-precision RRTMG in CUDA FP32 builds. It covers 208 commits (not counting
merges) from `v1.0.0` (e849149) to `main` 78e46c2. The 1.0.0 notes follow below.

## Highlights

- **New schemes, each compared with WRF v4.8.0 real*8 at tolerance 0 (CPU FP64 and CUDA FP64):** WSM3, WSM5, WDM5,
  WDM6, NSSL 2-moment and P3 (one ice category) microphysics; Betts-Miller-Janjic, Grell-3D ensemble and multi-scale
  Kain-Fritsch cumulus; QNSE PBL and surface layer with the EDKF mass flux; BouLac PBL; RUC land surface model;
  single-layer urban canopy model; skin SST and deep soil temperature update; monotonic and WENO advection.
- **Noah-MP option values beyond the defaults** (dynamic vegetation, runoff including VIC and Xinanjiang, the
  wetland model, snow, radiation, soil and glacier options) and **terrain options** (YSU `topo_wind` 1 / 2,
  `slope_rad`, `topo_shading`), verified the same way.
- **Code review fixes:** an independent review of the code against WRF found namelist options that were silently
  dropped, GPU-only defects, halo and boundary defects of decomposed runs, restart state that was not carried, and
  `cppwrf_real` paths that refused or misread common met_em inputs. Each finding was checked against WRF's source and
  runs; the confirmed ones are fixed and have suite cases or WRF real.exe references (see Fixes).
- **Restart chains:** a new check resumes a run from WRF's wrfrst and from CPPWRF's own restart file at a mid-run
  step and requires every later output frame and restart file to equal the straight run, bit for bit.
- **Domain decomposition and MPI:** a domain and its nests run as P x Q patches, each holding its own memory, as
  thread tiles or as MPI ranks (`-DCPPWRF_MPI=ON`). Every worker count and layout gives the same bits.
- **Drop-in use in a WRF run directory:** `cppwrf-wrf`, `cppwrf-realexe` and `cppwrf-check` replace `wrf.exe` and
  `real.exe` and use WRF's file names, `rsl` logs and `SUCCESS COMPLETE WRF`.
- **Performance:** RRTMG LW / SW compute in single precision in CUDA FP32 builds. CUDA FP64 and all CPU builds are
  unchanged. On the Colab G4, GPU FP32 runs P4 72 h 2.0x faster and N3 24 h 1.65x faster than the GPU FP32 build
  with RRTMG in double precision (the build before this change).
- **Packaging and tools:** `cmake --install`, CPack tarballs, Docker / Apptainer / Spack recipes, the `cppwrf`
  Python package, `cppwrf-compare`, `cppwrf-bench`, `cppwrf-fit`, `cppwrf-post` and `cppwrf-verify-obs`.

## New physics and dynamics options

Every option below is verified: its whole-model output equals WRF v4.8.0 real*8 at tolerance 0 in the named suite
cases. `data/option_tiers.txt` has no rows at 78e46c2, so no option is below the verified tier.

### Microphysics (`&physics mp_physics`)

| Value | Scheme | Scope | Suite cases |
|---|---|---|---|
| 3 | WSM3 (module_mp_wsm3.F) | real data; not with `do_radar_ref = 1` or ideal cases | W3 |
| 4 | WSM5 (module_mp_wsm5.F) | real data; not with `do_radar_ref = 1` or ideal cases | W5 |
| 14 | WDM5 (module_mp_wdm5.F), `ccn_conc` | real data, one domain; not with `do_radar_ref = 1` | D5 |
| 16 | WDM6 (module_mp_wdm6.F), `hail_opt` 0 / 1, `ccn_conc` | real data, one domain; not with `do_radar_ref = 1` | D6, D6H, D6C |
| 18 | NSSL 2-moment with hail, CCN and graupel / hail volume (module_mp_nssl_2mom.F), default `nssl_*` options | real data, one domain, no decomposition in x | N18, N18V |
| 17 | NSSL as WRF's check_a_mundo maps it (= 18 with its defaults) | as 18 | N17 |
| 50 | P3, one ice category, constant droplet number | real data, one domain; not with `do_radar_ref = 1` | P3MP |
| 51 | P3, one ice category, predicted droplet number | as 50 | P3NC |

New fields: QNCCN / QNCLOUD / QNRAIN (WDM), QHAIL, QNDROP, QNHAIL, QVGRAUPEL, QVHAIL, HAILNC / HAILNCV (NSSL),
QIR / QIB, TH_OLD / QV_OLD (P3). Still rejected: Milbrandt-Yau (9), NSSL 19 / 21 / 22 and non-default `nssl_*`
options, WSM7 (24), WDM7 (26), Morrison with aerosols (40), P3 52 / 53, WSM6R (106).

### Cumulus (`&physics cu_physics`)

| Value | Scheme | Options | Suite cases |
|---|---|---|---|
| 2 | Betts-Miller-Janjic (module_cu_bmj.F) | `bmj_rad_feedback` (per domain), `cudt` | P9BMJ, P4BMJ, P4BMJF, P4BMJFI0 |
| 5 | Grell-3D ensemble (module_cu_g3.F) | `ishallow` 0 / 1, `clos_choice` 0-16, `cugd_avedx` 1 / 3 | G3P8, G3P4, G3P4A3 |
| 11 | Multi-scale Kain-Fritsch (module_cu_mskf.F) | `aercu_opt = 0`, `kf_edrates = 0`, `cudt`; nests | M4, M4C5, M9, M3N |

Still rejected: KF-CuP (10), Grell-Devenyi (93), old Kain-Fritsch (99).

### PBL, surface layer, land surface, urban

| Option | Values | Scope | Suite cases |
|---|---|---|---|
| `bl_pbl_physics` / `sf_sfclay_physics` | 4 / 4 (QNSE), `mfshconv` 0 / 1 (EDKF mass flux, WRF's default 1) | `fractional_seaice = 0` | Q1, Q4, Q1M, Q4M |
| `bl_pbl_physics` | 8 (BouLac) | over surface layer 1 or 2; no BEP terms | BL1, BL4, BL9 |
| `sf_surface_physics` | 3 (RUC LSM), `num_soil_layers` 6 / 9 | with `sf_sfclay_physics` 1 or 2; one domain; input from WRF's real.exe; not with mosaic, `fractional_seaice`, `sst_update` | RUC4, RUC4J |
| `sf_urban_physics`, `use_wudapt_lcz` | 1 (single-layer UCM); 0 / 1 (URBPARM.TBL / URBPARM_LCZ.TBL) | Noah only, one domain, no restart, default table options | U4, U4L |
| `sst_skin`, `tmn_update`, `lagday` | 1; 1; any but 1 | Noah only | SKV, SKU, SKN |

Noah-MP (`sf_surface_physics = 4`), option values new in 1.1: `dveg` 1-9, `opt_crs` 2, `opt_btr` 2 / 3,
`opt_runsrf` / `opt_runsub` 1, 2, 4, 6 (VIC), 7 (Xinanjiang), `opt_sfc` 2, `opt_frz` 2, `opt_inf` 2, `opt_rad` 1 / 2,
`opt_alb` 1, `opt_snf` 2-5, `opt_tbot` 1, `opt_stc` 2 / 3, `opt_gla` 2, `opt_rsf` 2-4, `opt_tksno` 2-5,
`opt_compact` 1, `opt_scf` 2, `opt_wetland` 1 (Zhang 2022), `soiltstep` (suite cases N*, X4*, V*, NMP1, V3).
`opt_runsrf` / `opt_runsub` 5 and 8, `opt_infdv`, `opt_alb` 3, `opt_soil` 2-4, crops, irrigation, tile drainage,
`opt_wetland` 2 and `dveg` 10 stay errors.

Terrain (`&physics`): `topo_wind` 1 / 2 with YSU, `slope_rad` 1, `topo_shading` 1 with `shadlen`; real data, one
domain, fixed time step; `topo_shading` not with tiles or MPI ranks (suite cases TW1, TW2, TW4, SR1-SR4).

Still rejected: Shin-Hong (`bl_pbl_physics = 11`), BEP / BEP+BEM (`sf_urban_physics` 2 / 3), the lake model, the
Pleim-Xiu, CLM and SSiB land surface models.

### Advection (`&dynamics`)

| Option | New values | Suite cases |
|---|---|---|
| `moist_adv_opt`, `scalar_adv_opt`, `tke_adv_opt` | 2 (monotonic; needs `h_sca_adv_order = 5`, `v_sca_adv_order = 3`), 3 (WENO), 4 (positive-definite WENO) | MO4, MOK4, MOMY, MON3, WE4, WP4, WK4, WMY, WN3 |
| `momentum_adv_opt` | 3 (WENO) | WM4, WN3 |

### Domain decomposition and MPI

- `run.tiles = PxQ` now also with nests. Each tile holds its patch and a 3-point halo; tile 0 writes the output.
- MPI ranks: build with `-DCPPWRF_MPI=ON` (off by default) and run under `mpirun -np N`. Set the layout with
  `run.ranks = PxQ` or `CPPWRF_RANKS`, and the GPUs of a CUDA build with `CPPWRF_RANK_DEVICES`. A real-data start
  reads only the worker's window of wrfinput's 3-D variables.
- The CPU FP64 result is the same for every worker count and layout. This was checked on one machine: 1 to 8 ranks
  in 13 layouts on eight cases, 53 suite cases in three layouts, and up to 48 ranks against the one-process run
  (docs/MPI.md).

## Performance

- **RRTMG in single precision (CUDA FP32 only).** RRTMG's arithmetic type `rr` (src/phys/ra_rrtmg_common/
  rrtmg_real.hpp) is `float` in CUDA FP32 builds, as RRTMG's `rb` in WRF real*4, and `double` in CUDA FP64 and every
  CPU build. CUDA FP64 output is unchanged (tolerance 0), and CPU FP32 output was byte-identical at that change.
  GPU FP32 output bytes change. Its distance from WRF real*8 stays at WRF real*4's level (P4 72 h, N3 24 h). GPU
  FP32 on the Colab G4: P4 72 h 2.0x and N3 24 h 1.65x faster than the same build with RRTMG in double precision.
  The RRTM longwave stays in double.
- **FP32 math in CUDA builds.** WSM6, the nest interpolation (SINT, boundary slabs) and the `vm` math functions
  have float paths in CUDA FP32 builds. MYNN-EDMF's DOUBLE PRECISION locals stay double, as in WRF real*4. FP64
  builds are unchanged. (A later change, under Fixes, makes CPU FP32 take the same single-precision paths.)
- **GPU memory.** The column arrays of Tiedtke, New Tiedtke, Grell-Freitas-Li and GWDO (FP32 and FP64), and of
  Kain-Fritsch and MYJ (FP64), moved from the kernels' stack frames to a column work buffer. On an RTX PRO 6000,
  peak memory of a 1 h run on 120 x 90 x 45 is 4.7-15 GB lower (WRF's CONUS suite in FP64: 20.7 -> 8.7 GB). Every
  verified suite fits a 6 GB GPU in FP32 and FP64. The moved kernels are up to 11 % (FP64) and 30 % (FP32) slower
  on the RTX PRO 6000, and the steps of Grell-Freitas and Tiedtke runs 3-7 % slower. Results are unchanged.
- `CPPWRF_DEVICE_MEMORY_MIB=N` sizes the work buffers as for a device with N MiB of memory.
- **Benchmark.** No new benchmark was run for 1.1.0. The latest one is the rerun at c9c95c5 on the Colab G4
  (stepping time per simulated hour, before the RRTMG FP32 change): GPU FP32 is 5.1x to 10.3x faster than WRF
  real*4 at its best MPI layout, and GPU FP64 is 3.1x to 6.0x faster than WRF real*8 (docs/PERF.md). A new
  benchmark will come with a later release.

## Verification

Full CPU CI run on 78e46c2 (GitHub-hosted runners, AMD EPYC, run 44689341507), all exact (tolerance 0), and the GPU
proof on the Colab G4:

| Check | Result |
|---|---|
| `tests/verify/wrf_suite.py`, CPU FP64 against WRF v4.8.0 real*8 at tolerance 0 | PASS 342, FAIL 0, SKIP 0 |
| WRF output files (wrfout / wrfrst values, attributes, metadata) | PASS 42 |
| Ideal-case output files | PASS 14 |
| Differential tests against WRF's compiled Fortran (`ctest -L diff`) | PASS 32 / 32 |
| ctest (includes `real_vs_wrf_runs`: `cppwrf_real` against WRF real.exe on 14 runs) | PASS 33 / 33 |
| GPU: Colab G4, NVIDIA RTX PRO 6000 Blackwell Server Edition (sm_120), CUDA 13.0, at fb158f3 (the model code of 78e46c2) | CUDA FP64 suite PASS 338, FAIL 0 at tolerance 0; device glibc math proof identical; ctest 35 / 35 in the FP64 and in the FP32 CUDA build; CUDA FP32 suite 44 runs within their measured tolerances (see below) |

How the suite counts relate (tests/verify/cases at 78e46c2): the suite defines 839 cases, 477 CPU FP64, 319 CUDA
FP64 and 43 CUDA FP32, and 18 restart configurations, each with a CPU and a CUDA FP64 check.
- CPU: 154 of the 477 CPU FP64 cases are `--slow`: the 9 runs of 24 h, 30 h and 72 h, and 145 Noah-MP option cases
  whose reference data do not fit on one GitHub runner next to the rest. The CI's 342 checks are the other 323
  cases, the 18 CPU restart checks and the Straka check. The slow cases run on request (`wrf_suite.py --slow`) and
  were not part of this CI run.
- CUDA FP64: the G4's 338 checks are all 319 `cuda_dp` cases (including the 137 slow ones, the CUDA twins of the
  Noah-MP option cases), the 18 GPU restart checks and the GPU Straka check.
- CUDA FP32: the 44 runs are the 43 `cuda` cases and `restart_p1_fp32_cuda`.
- v1.0.0 had 242 cases (153 CPU FP64, 71 CUDA FP64, 18 CUDA FP32; 8 slow) and 17 restart configurations.

fb158f3 and 78e46c2 have the same model code: 78e46c2 only adds the CPU case N3M and changes two FP32 tolerances.
In the G4 run, ACM7's W (relative L2 3.02e-2) was just above its earlier tolerance of 3e-2, and Q4's U and QV had
moved up as well (6.5e-4, 5.6e-4). The FP32 tolerances of ACM7 (W) and Q4 (U, QV) were then set to about 3x those
measured values (tests/verify/cases/28_acm.py, 41_qnse.py), so all 44 FP32 runs are within their tolerances. The
FP32 distances changed because of the FP32 fixes of this release (docs/FP32_ACCURACY.md, section 1.1.0).

The `real_vs_wrf_runs` test compares `cppwrf_real` with WRF's real.exe on four runs (plain P1 met_em,
`lagrange_order` 1, `default_soiltype`, VAR_SSO input) and ten met_em input variants: sea ice with SST (two runs),
no inland-lake category (`ISLAKE = -1`), specific humidity (with `use_sh_qv` false and true), no GHT, top-down
levels, `sfcp_to_sfcp`, pass-through fields and soil water. For the variant without an inland-lake category, WRF's
own wrf.exe stops on that artificial input ("too many input landuse types"), so only real.exe's output is compared.

The conditions of bit identity are those of 1.0.0 (below): WRF v4.8.0 built with gfortran 14.2 and glibc 2.41 and run
in one process, `configure -r8`, and WRF-4.8.0-r8d8 (`-fdefault-real-8 -fdefault-double-8`) for the schemes with
DOUBLE PRECISION variables (Thompson, MYNN, radar reflectivity, New Tiedtke, Grell-Freitas, WENO advection, NSSL, P3).

New suite coverage since 1.0.0:
- Every new scheme above has CPU FP64 and CUDA FP64 cases against WRF real*8, most with a run restarted at 30 min.
  Each scheme's column or patch driver is bit-identical to WRF's compiled routine: tests/phys/mp_wsm35, mp_wdm,
  mp_nssl, mp_p3, cu_bmj, cu_g3, cu_mskf, bl_qnse, bl_mfshconv, bl_boulac, lsm_ruc, sf_urban, and tests/dyn/advection.
- Coverage cases for option values that had no case before: `cldovrlp` 1, 3, 4 (`idcor` 1), 5; `icloud` 0;
  `diff_6th_opt` 1; `km_opt` 3 with `diff_opt` 1 and 2; `tke_adv_opt` 0; `moist_adv_opt` / `scalar_adv_opt` 0;
  `bl_mynn_cloudpdf` -1 / -2.
- Nests: N3F1 (land use of d02 differing at the fed-back points) and N3M (MYNN on the two-way nest with `km_opt`
  4 / 2). N3M's first WRF reference came from the stock `-r8` build, which computes MYNN's DOUBLE PRECISION locals in
  real*16; like every MYNN case it now uses WRF-4.8.0-r8d8 (`-fdefault-real-8 -fdefault-double-8`) and is
  bit-identical.
- WRF's clock: P4D16 (30 h, dt 16 s) and N3D50 (nest step 50/3 s). Restart alarm: RSA and its resumed runs.
- Order and initialisation checks: reversed and i-outer serial launch order, and a pattern-initialised,
  bounds-checked build. NVIDIA compute-sanitizer (memcheck, racecheck, initcheck, synccheck) passed on CUDA FP64 and
  FP32 builds.
- The CPU FP64 suite runs on GitHub-hosted runners on every push to `main` (`tools/ci/ci_submit.sh` for any local
  commit). The reference data are public assets of the cppwrf-data repository. The GPU suite runs on a Colab G4
  once per release.

## Fixes

From the code review (every finding verified against WRF v4.8.0's source and runs before it was fixed):
- **Namelist options WRF honours that were dropped:** `scalar_adv_opt` without physics, a nest's
  `diff_6th_factor`, per-domain scalar advection orders, nest start / end dates, `sst_update` without the column
  physics; `cppwrf_real` now stops on lake, PWP ocean and sea-ice input options instead of ignoring them.
- **GPU paths:** MSKF's out-of-range levels and `NINT` as on x86-64 on the device; WENO weights and eps in double as
  WRF's DOUBLE PRECISION locals (no NaN in FP32); NSSL over the worker's rows with a bounded work space.
- **Decomposed runs:** the scalars' halo exchanged after `spec_bdy_final`; the periodic wrap of short directions;
  `diff_opt 2` stresses at the halo point beyond a symmetric side; the base state's halos after a wrfrst start.
- **Restart:** the restart files now carry the state WRF carries that was missing (O3RAD, HRANG, DECLIN / SOLCON,
  URATX / VRATX / TRATX, RA / RS, GWDO tendencies, Kain-Fritsch's SHALL, AKPBL); CPPWRF's own restart file
  (format 4) also holds WW / WW_M / PHP / ALT and, for one-patch runs, the halo points, so a resume equals the
  straight run.
- **`cppwrf_real` with met_em input from other sources:** sea ice together with an SST field (the skin and soil
  temperatures at the ice points, which could differ by up to 16 K), land use without an inland-lake category
  (`ISLAKE = -1`, e.g. USGS 24 or MODIS 20), specific-humidity and mixing-ratio input (`FLAG_SH` / `FLAG_QV`,
  `use_sh_qv`), `sfcp_to_sfcp` and top-down level input, GHT rebuilt when it is missing, and the met_em fields
  real.exe copies through (CANWAT, LAKE_DEPTH, FRC_URB2D, ...). The met_em level and dimension checks of WRF's
  input_wrf.F, bounds-checked reads, and WRF's stop on a non-monotonic reference pressure on the hybrid coordinate.
  Ten new WRF real.exe references cover these inputs, each identical. Surface pressure from PMSL alone is refused.
- **Namelist checks:** values that `wrf.exe` runs differently or refuses (per-domain physics, `sf_surface_physics
  = 0` with a surface layer, `num_soil_layers` derived as check_a_mundo does, `wif_input_opt`, MSKF's `icloud`) are
  refused or derived by the translator and the model as WRF does. The wrfout / wrfrst global attributes and the
  stream contents (`iofields` files, auxiliary streams) follow WRF's output for the checked namelists.
- **Nests:** the parent's physics initialisation after the first feedback, scalars paired by name in the force and
  feedback steps, and refusal of per-domain physics WRF does not run.
- **FP32 builds follow WRF real*4 more closely:** the RRTMG tables as real*4 builds them, Thompson's double tables
  and process rates, MSKF's momentum transport, P3's double temporaries, julian from the double day of year, and
  WRF's single-precision constants (small step, sfclayrev, NSSL, Noah-MP, RRTMG SW heating). CPU FP32 now uses the
  same single-precision paths as CUDA FP32. FP64 output is unchanged.
- **GPU paths:** RRTMG with `icloud = 0` and BMJ cloud feedback, radiation boundary-layer clouds without QI,
  Thompson's ice fall speed, MYNN's plume top, coherent loads in the RRTMG batch kernels.

Other fixes:
- **Restart alarm of a resumed run.** When the restart interval is not a multiple of the time step, the first ring
  now comes at the restart time plus `WRF_ALARM_SECS_TIL_NEXT_RING_51`, and later rings count from that step, as in
  WRF.
- **`auxinput4_interval` and `gfdda_interval_m`** that are not multiples of the domain's time step are refused by
  the translator and the model, because WRF stops on them. Before this fix, the model ran on with the first record.
- **WRF's clock on the fixed time step.** xtime, julian, the year, curr_secs2 / curr_mins2 and the history XTIME
  now come from the exact rational clock. Before this fix, they could differ in the last bit after one day, or with
  nest steps that are a fraction of a second. Earlier suite results are unchanged.
- **Acoustic step count with `time_step_sound = 0`:** computed in REAL as solve_em does, so it uses float in the
  FP32 build (e.g. dx 10 km, dt 67 s: 6 steps, not 4). FP64 results are unchanged.
- **Grell-3D restart fields** (XF_ENS / PR_ENS, K22 / KBCON / KTOP_SHALLOW) are written and read as WRF does.
- **RRTMG with WSM3:** WRF's "MP option 3" hydrometeor split is applied on the host and CUDA paths.
- **CUDA versions:** the whole CUDA build compiles with CUDA 12.4; CMake now stops below CUDA 12.1, whose kernel
  parameter limit the batched launches need.
- **Python:** the test fixtures and the self-test read the Times variable correctly with numpy 2.3.
- **Column-test drivers:** they use `-march=native` only with `CPPWRF_NATIVE`, which prevents an illegal
  instruction on other CPUs that share a compiler cache.

## Changed behaviour

- Tiles no longer write `output.dir.tileN` copies. Tile 0 writes the run's files from gathered fields.
- The drop-in `cppwrf-wrf` stops with an error on `override_restart_timers` or `write_hist_at_0h_rst` with
  `restart = .true.`. These were ignored before.
- A QNSE namelist without `mfshconv` now runs WRF's default (`mfshconv = 1`).
- CAM radiation (`ra_lw_physics` / `ra_sw_physics = 3`) is named in the translator's error.
- The CUDA build needs CUDA 12.1 or later and a GPU of compute capability 7.0 or later.
- CPPWRF's own restart files (`restart_<step>.nc`) are now format version 4 and are not read by earlier versions;
  as CHANGELOG.md states, they are bound to the commit that wrote them and are not a stable interface. WRF's
  wrfout / wrfrst files are unchanged in form, so 1.1.0 is a minor version. WRF's own restart does not carry
  SHADOWMASK; a run with `topo_shading = 1` resumed from a wrfrst between radiation steps therefore shades nothing
  until the next radiation call, in WRF and in CPPWRF alike.

## Known limitations

- **Verification tiers:** `data/option_tiers.txt` has no rows at 78e46c2, so no option is column_verified or
  unverified. `[cppwrf] allow_unverified` is kept for later ports.
- **FP32** is checked against WRF with measured tolerances (relative L2), not for bit identity.
- **Translator and model disagree on two namelist shapes:** the translator still accepts cumulus-only and
  radiation-only namelists, which the model then refuses.
- **`cppwrf_real`** does not read TAVGSFC and refuses surface pressure from PMSL alone (WRF's sfcprs path).
- **MPI:** several GPUs and several nodes have not been run. Several MPI ranks on one GPU are not proved
  (docs/MPI.md).
- **Scheme scope:** NSSL, P3, WDM5 / WDM6, aerosol-aware Thompson and the urban canopy model run on one domain.
  NSSL, P3, WDM and WSM3 / WSM5 run in real-data cases only. The urban canopy model cannot be restarted. RUC needs
  input from WRF's real.exe (not `cppwrf_real`).
- The N3 72 h comparison is still open (docs/STATUS.md).
- Not ported: moving nests, vertical nest refinement, ndown.exe, I/O forms other than NetCDF, spectral / surface /
  observation nudging, the adaptive time step with nests, and the options listed in docs/USER_GUIDE.md, "Rejected
  Options and Why". Each is rejected with an error.

## Tools and documentation

- Drop-ins: `cppwrf-wrf`, `cppwrf-realexe`, `cppwrf-check`, and `cppwrf --version`.
- Installation: `cmake --install`, CPack tarballs per flavour and CUDA architectures, Docker, Apptainer and Spack
  recipes, `tools/release/make_release.sh`. The data directory is found from the binary (`CPPWRF_DATA_DIR`
  overrides it).
- The `cppwrf` Python package (`pip install ./python`): namelists, runs, wrfout files, diagnostics (docs/PYTHON.md).
- `cppwrf-compare`, `cppwrf-bench`, `cppwrf-fit` (docs/TOOLS.md); `cppwrf-post` (CF NetCDF and GRIB2,
  docs/POST.md); `cppwrf-verify-obs` (scores against METAR / ISD / CSV observations, docs/VERIFY_OBS.md).
- New documents: docs/INSTALL.md, QUICKSTART.md, MIGRATING_FROM_WRF.md (option tables generated from the
  translator), FAQ.md, MPI.md, FP32_ACCURACY.md, Turkish translations in docs/tr. Repository files: CONTRIBUTING,
  SECURITY and Code of Conduct, plus issue and pull request templates.
- FP32 accuracy study: tools/verify/fp32_accuracy.py and tools/bench/colab_fp32.sh. The first-hour FP32 outliers
  (ACM7, MORR, SKV) trace back to Kain-Fritsch decisions that are ties at the last bit, not to a port defect
  (docs/FP32_ACCURACY.md). The study (2026-11-09 to 11-15) predates the RRTMG FP32 change and the FP32 real*4
  fixes of this release; its 1.1.0 section gives the suite's FP32 values before and after those fixes.

## Outlook

The goal of the next release is GPU FP32 output bit-identical to WRF real*4, as FP64 already is to WRF real*8. A
new performance benchmark will come with a later release.

## Requirements

A C++20 compiler, CMake >= 3.24, OpenMP and NetCDF-C. For the GPU builds, the CUDA toolkit >= 12.1 and an NVIDIA
GPU of compute capability 7.0 or later. An MPI library for `-DCPPWRF_MPI=ON`. Python 3 with `netCDF4` and `numpy`
for the tools.

## Licence

GPL-3.0-or-later (LICENSE, NOTICE.md).

---

# CPPWRF 1.0.0 release notes

CPPWRF 1.0.0 is a C++20 / CUDA port of the WRF-ARW model v4.8.0. It reads WRF's inputs and writes WRF's output
files, and its CPU FP64 and GPU FP64 builds give WRF v4.8.0 real*8's results bit for bit in every verified
configuration. CHANGELOG.md lists the contents; docs/USER_GUIDE.md explains the use; docs/STATUS.md lists every
verified case.

## Verified scope

What "verified" means here: WRF v4.8.0 (gfortran 14.2, glibc 2.41, Debian 13, `configure -r8`, serial build, one
process) and CPPWRF run from the same inputs, and every value of every variable of every output time is compared at
tolerance 0 (NaN equals only NaN, -0 differs from +0).

| Check | Scope |
|---|---|
| Model runs against WRF (`tests/verify/wrf_suite.py`) | 242 cases: 153 CPU FP64 and 71 GPU FP64 cases at tolerance 0, 18 GPU FP32 cases by relative L2; 8 of them are the 24 h and 72 h runs of `--slow` |
| Restarts | a run restarted from its own restart files equals the uninterrupted run: 17 CPU and 17 GPU FP64 configurations, also one FP32, and the Straka checks |
| WRF output files | the wrfout and wrfrst files of the CPU FP64 runs equal WRF's in values, variables, dimensions, attributes and global attributes (66 runs), also when resumed from WRF's wrfrst files; the wrfout files of WRF's ideal cases (14 runs) |
| GPU FP64 | the 71 GPU FP64 cases and 17 GPU restart checks (88 runs) at tolerance 0 on NVIDIA RTX PRO 6000 Blackwell (sm_120); earlier subsets also on L4 (sm_89) and A100 (sm_80) |
| Schemes against WRF's own objects | the differential tests (`ctest -L diff`) link each scheme's driver to the objects of the WRF build that made the references: Kessler, WSM6, Thompson, YSU, MYJ, MYNN, the surface layers, Noah, Noah-MP, Kain-Fritsch, diffusion, nest numerics, the physics driver, Grell-Freitas and New Tiedtke column replays |
| real.exe | `cppwrf_real` output equals real.exe's for 43 cases (single domains, nests, `sst_update`, hybrid and hypsometric options) |
| Namelist variants | several hundred 3-minute and 1-hour variants of the namelist options against WRF (docs/STATUS.md, "Namelist variants") |

The configurations cover real-data domains in the mid-latitudes (15 km), the tropics (Bay of Bengal, Mercator), the
Arctic (Svalbard, polar stereographic, sea ice) and the Alps (3 km); 24 h and 72 h forecasts with restarts in the
middle; WRF's default `namelist.input` (physics_suite CONUS); one-way and two-way nests; WRF's idealized cases. The
CPU result does not depend on the OpenMP thread count, and it is the same on Intel and AMD CPUs
(`CPPWRF_PORTABLE_LIBMVEC`).

The quick start of README.md and docs/USER_GUIDE.md (the CPU FP64, GPU FP32 and GPU FP64 builds, `ctest`, the
translator's tests, the Straka and em_quarter_ss cases and the end-to-end example from the GFS analysis through WPS
and `cppwrf_real` to wrfout files) runs from a clean source tree: `tools/colab/colab_quickstart.sh` did it on a
Google Colab G4 runtime, where the example's GPU FP64 output equalled the CPU FP64 output bit for bit.

## Conditions of bit identity

- CPU FP64 builds with the flags the CMake files set (no FMA contraction, no fast-math; docs/USER_GUIDE.md,
  "Required Flags and Determinism"). Any x86-64 CPU with SSE4.1, FMA and AVX2.
- The comparison is with WRF real*8 (`configure -r8`; `-r8 -fdefault-double-8` for Thompson, MYNN and radar
  reflectivity, whose DOUBLE PRECISION variables are 16-byte with `-r8` alone) built with gfortran 14.2 and glibc
  2.41 and run in one process. WRF's own result changes with its MPI process layout, its compiler version
  (another gfortran vectorises other loops) and `-march` (FMA contraction).
- GPU FP64 builds with `--fmad=false` and `CPPWRF_CUDA_GLIBC_MATH` (the CMake defaults).
- GPU FP32 is not bit-identical to anything. docs/FP32_ACCURACY.md compares its difference from WRF real*8 with stock
  WRF real*4's (the usual production build), R = FP32's / real*4's, over eleven domains at 1 h (and 24 h / 72 h):
  in the KF / WSM6 / Thompson / MYJ, nest, Arctic and large-domain cases R is 0.4 to 1.1 for the state and surface
  fields (as close as WRF real*4 or closer); with ACM2 + Pleim-Xiu, Morrison and the skin SST single runs reach up to
  2 at 1 h (2.3 for QVAPOR in ACM7) and higher for sparse hydrometeors. These were traced to Kain-Fritsch decisions
  that are ties at the last bit, not to a port defect: other builds of the same code give 1.1 to 2.0, and from 6 h on
  R is 0.8 to 1.1 there too.

## Performance

On one Google Colab G4 runtime (AMD EPYC 9B45 with 48 hardware threads, NVIDIA RTX PRO 6000 Blackwell), stepping
time per simulated hour against WRF v4.8.0 at its fastest MPI layout on the same machine (docs/PERF.md):

| Case | WRF real*4 | WRF real*8 | CPPWRF GPU FP32 | CPPWRF GPU FP64 |
|---|---|---|---|---|
| P4: 119 x 89 x 44, 15 km, full physics, radt 1 min | 7.84 s | 9.72 s | 1.66 s | 2.78 s |
| N3: P4 with a two-way 5 km nest | 23.8 s | 34.0 s | 5.25 s | 10.4 s |
| L: 359 x 269 x 44, radt 15 min | 24.8 s | 45.1 s | 3.01 s | 9.59 s |

## Not included

MPI runs (a run is one process: OpenMP threads or one GPU), moving nests, vertical nest refinement, ndown.exe, I/O
forms other than NetCDF, and the options listed in docs/USER_GUIDE.md ("Rejected Options and Why"): urban and lake
models, other microphysics, cumulus, PBL and land surface schemes (among them the Pleim-Xiu land surface model), Noah-MP's non-default options, spectral and observation nudging, stochastic
physics, chemistry, the adaptive time step with nests. Each is rejected with an error by the translator and the model.

## Requirements

A C++20 compiler, CMake >= 3.24, OpenMP, NetCDF-C, Python 3 with `netCDF4` and `numpy`; the CUDA toolkit >= 12
and an NVIDIA GPU for the GPU builds. WRF's `run/` directory (WRF v4.8.0 source) for the physics tables, and WPS or
WRF's real.exe output for real-data inputs.

## Licence

GPL-3.0-or-later (LICENSE). NOTICE.md carries the notices of WRF (UCAR), RRTMG (AER), Noah-MP (UCAR) and the GNU C
Library, from which parts of CPPWRF are derived.
