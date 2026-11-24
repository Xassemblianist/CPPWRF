# Changelog

All notable changes of CPPWRF. Versions follow [Semantic Versioning](https://semver.org/): a change of the results of
a verified configuration, of a configuration key or of an output file is a new major version. CPPWRF's native
restart files (`restart_<step>.nc`) are not a stable interface: they are bound to the commit that wrote them; a run
resumes across versions from WRF's wrfrst files.

## Unreleased

### Added
- Ensembles in one process (docs/ENSEMBLE.md, USER_GUIDE section 7): an `[ensemble]` section (`members`,
  `memberI` = '|'-separated overrides, `mode` = threads | sequential) runs every member of a single-domain
  configuration; with threads each member has its own host thread, CUDA stream, graphs and fields on the one GPU,
  sharing the stack reserve and the device tables; the members' host work runs one member at a time. Every member's
  files are byte-identical to its single run (ctest `ensemble`; `--set ensemble.member=I` runs member I alone).
  Single runs are unchanged.

### Changed
- Halo exchanges of a decomposed run (tiles, MPI ranks; docs/MPI.md section 3.4): each exchange is planned once and
  the plan kept (the boxes, their offsets in one send and one receive buffer, on the GPU a device table of them); the
  boxes go out with one pack launch and come in with one unpack launch on the worker's stream (were one launch per
  field and peer) and no device-wide wait (were two per exchange); MPI ranks post every receive before the sends and
  wait once (`MPI_Irecv` / `MPI_Isend` / `MPI_Waitall` on a duplicate of `MPI_COMM_WORLD`; were a blocking
  `MPI_Probe` / `MPI_Recv` per peer), from and into persistent buffers: device buffers handed to a GPU-aware MPI
  (`run.gpu_aware_mpi = auto | on | off`, `CPPWRF_GPU_AWARE_MPI`; auto: when the library reports at run time that it
  takes device memory) or pinned host buffers with one asynchronous copy per direction (were pageable buffers copied
  per message). The maxima, fatal codes, per-row reductions and owner broadcasts of MPI ranks are one `MPI_Allgather`
  each (were a message from every rank to every other), combined in worker order as before. An MPI rank of a GPU
  build runs its time steps as CUDA graphs (segments between its exchanges and fatal checks) and prepares the next
  step while the device runs the last, as one process does (`CPPWRF_GRAPHS=0`: without). The values are bytes copied
  between the same points as before: every output is unchanged in every layout and build. `CPPWRF_HALO_STATS=1`
  prints each worker's exchanges; `CPPWRF_TILE_DEVICE_BUFFERS=1` runs tiles over the device-buffer path (a check).
  tools/kaggle/kaggle_mpi.sh: layouts `L@on` / `L@off` force the transport, env.txt names whether Open MPI was built
  with CUDA.
- WRF-format output off the model's thread (files unchanged, byte for byte): the values of a wrfout / auxhist3 /
  wrfrst record are copied from the model's fields as before, then a writer thread makes the record's NetCDF calls
  (Times, the puts in order, nc_sync) while the model steps on. One NetCDF lock for the process: the model's thread
  holds it outside its time steps (its readers of wrfinput, wrflowinp, wrffdda, restart files), the writer takes it per
  call. New key `output.write_queue_mb` (default 1024: the memory of the records waiting; 0 writes in the model's
  thread as before); a writer error stops the run at the next step or at the end with the same message. Decomposed runs
  (tiles, MPI ranks) write in the model's thread. The precomputed chunks are kept per record (a wrfrst file and the
  history frame of the same time no longer take each other's), and the replacement of HDF5's deflate filter is used
  only with HDF5 1.10 to 1.14 (another version keeps HDF5's own filter: the same bytes); docs/PERF.md IO2.

### Fixed
- Ensembles: convtrans_prep's averaging counter (WRF's SAVE `stepave_count`) is each run's own (one per run, shared
  by its domains) instead of one per host thread, so a sequential member no longer inherits the previous member's
  count (in its restart file, or as a divisor after a WRF restart). WRF fatal flags are per member (a slot per member
  thread; on CUDA the slot travels with each launch): one member's fatal code no longer stops another member, and a
  stopping member reads its pending checks before its destructors. `output.wrf_dir` is a per-member root as
  `output.dir`; member directories are compared as resolved paths; stream names that leave the WRF directory and
  debugging dumps are refused. Members run with the output writer thread (one NetCDF lock for the members' host work
  and the writer). A member thread that cannot be started stops the ensemble cleanly; a member thread frees its
  stream, graphs, flag slots and work buffer at its end.
- Decomposed runs: CPPWRF_TILE_TRACE only prints (a rank-local setting changed the transport of the collectives and
  could hang an MPI run); halo messages carry a fingerprint of their boxes (fields, element sizes, extents; device
  buffers: a host header beside each message) checked before any byte is unpacked, and periodic lines and per-level
  maxima have kinds of their own; `run.gpu_aware_mpi = auto` hands device buffers to MPI only when every rank's
  library takes them (rank 0's mode for all); counts and request storage are checked before the first post and a
  failed post or wait aborts with the buffers alive; a field given twice to one exchange is exchanged once; alignment
  padding is sent as zeros; the duplicated communicator is freed after a successful run.
- Fatal checks: every flag copy, reset and wait is checked (a failed transfer stops the run instead of reading 0);
  a pinned slot is cleared before its copy; the flag of Noah's land preprocessing (urban LAI) is checked right after
  it, before land SFLX and the dumps use its columns.
- Output writer thread (output.write_queue_mb): a file whose path is still open on the writer thread (a fixed
  history_outname with frames_per_outfile = 1) waits for the queued records before it is created (HDF5 refused it);
  the queue is configured once per process (a nest or member with another value is refused; a synchronous record
  waits for the queued ones); a writer error that surfaces only at the end of a run stopped by non-finite values
  gives exit status 2; no new stream or restart file after a known writer error; the NetCDF lock's ownership is one
  state per thread; the end of the process joins the writer whatever it threw; an oversized write_queue_mb is
  refused instead of wrapping.
- NetCDF-4 output: the staged bytes of a variable belong to the record before their chunks are queued, and a chunk
  that cannot be queued or compressed is no longer counted, so an error between staging and writing waits for the
  queued compression and then frees the bytes.

## 1.4.0 (2026-11-24)

### Changed
- CPPWRF's own restart file (`restart_<step>.nc`) no longer depends on the decomposition: a decomposed run (tiles,
  MPI ranks) writes the halo points of its fields beyond the domain's edges (`HALO_<name>`, gathered from the edge
  patches) as a run of one patch does, and a decomposed run resumed from the file sets them in every patch that holds
  them. Before, a decomposed run's file had no `HALO_<name>` and a decomposed resume refilled those points from the
  domain's values. WRF-format wrfrst files and every model output are unchanged. Suite: `real_p4_r8_fp64_cpu_rst_t2x2` and
  `ideal_quarter_ss_k2_fp64_cpu_rst_t2x2` (every restart file of a 2 x 2 run identical to the one-patch run's, all
  variables and attributes; quarter_ss periodic) and
  `real_p4_r8_fp64_cpu_chain_t2x2` (P4's restart chain in 2 x 2 tiles).
- docs/MPI.md section 6: the CUDA FP64 build with MPI on two T4 GPUs of one Kaggle node (`tools/kaggle/kaggle_mpi.sh`):
  seven cuda_dp checks in seven layouts (one process, one rank, one rank per GPU, two ranks per GPU, tiles over the
  GPUs) equal the WRF references at tolerance 0.
- Verification: every suite check says what it compared (`[vs WRF tol 0]`, `[vs WRF rel L2]` or `[self only]`) and
  requires complete coverage: every scheduled frame, stream, domain and restart file of the configuration, one time
  window for compared and reference counts, no duplicate or clock-only records, integers compared exactly, variables
  without a Time dimension and the metadata of every file compared; a check with zero comparisons fails. Restart
  checks resume from WRF's own wrfrst file as WRF's resumed run was configured, and the P4, G2, P8, NMP1, SLAB1, N3
  and D0 restart chains are compared with WRF's runs of their configurations at tolerance 0 (were self-consistency
  checks). tools/gcp/suite_status.sh fails on any FAIL, SKIP or ERROR and reconciles the results with the cases
  requested. Mutation tests of the comparator and the harness: tests/verify/test_exact_mutations.py,
  test_harness_mutations.py, test_cdx9_mutations.py.
- Suite: N3T, three domains (d03 at 1.67 km inside d02 inside d01, two-way feedback, smooth_option 2, P4 physics),
  against WRF real*8 at tolerance 0 on every domain, CPU and CUDA FP64. straka_wrf / straka_wrf_cuda (an RMS check
  against WRF real*4) leave the suite; WRF's em_grav2d_x real*8 run is ideal_grav2d_x_fp64_cpu / _cuda at tolerance 0.
- Build: CMake refuses x87 arithmetic (-mfpmath=387, -m32, -mno-sse2, -fexcess-precision=fast) and, in the CUDA FP64
  and FP32 identity builds, --fmad=true; precision.hpp asserts `__FLT_EVAL_METHOD__ == 0`.
- GPU nests and stepping (results unchanged): history-only feedback runs when the parent next reads the fields,
  smooth_option 2 without the field copies, the boundary slabs of a field in one launch, force and feedback as CUDA
  graphs; a step segment's graph is launched before the wait for the previous one; host uploads through a reusable
  per-thread buffer.
- GPU RRTMG SW (results unchanged): the SW prep's inatm_sw / setcoef_sw and taumol_sw run per (column, layer)
  (`KSwSetcoef`, `KSwLayer`), the layer of each band's solar source per column (`KSwSolfr`); docs/PERF.md PERF14.
- sfclayrev / YSU: `phys.shalwater_z0`, `phys.scm_force_flux != 0` and a nonzero U_FRAME / V_FRAME in wrfinput are
  refused (not ported; were ignored). The two options are checked with the model's options whichever schemes the
  run has, and the error names the value given; the translator names WRF's group (`&physics.shalwater_z0`,
  `&scm.scm_force_flux`). A run resumed from a wrfrst file refuses a nonzero U_FRAME / V_FRAME in that file too
  (WRF's Registry `ir`: the resumed wrf.exe takes the file's frame); the errors name the file and the value.
- CPPWRF's native restart format version is 7 (`sfc_var2d` / VAR and `HALO_<name>` in decomposed runs' files): a
  file of version 6 or older is refused with a pointer to the run's wrfrst file (native restart files belong to the
  sources that wrote them). Every `HALO_<name>` of a field the file has is required on reading. The file is written
  as `restart_<step>.nc.partial` and renamed once complete (fields and halo points).

### Fixed
- sfclayrev with fractional_seaice = 1: USTM_SEA is 0 on entry to every open-water call (WRF's automatic array has
  no defined value; CPPWRF carried the last call's) and ch = chs only in the non-fractional branch, as
  module_surface_driver.F.
- UOCE / VOCE (ocean currents) are read from wrfinput on a cold start (were 0); VAR (orographic variance) is carried
  in CPPWRF's restart file and read from WRF's wrfrst. Native restart files of earlier versions lack VAR and are not
  read (CPPWRF's native restart files are bound to the commit that wrote them; resume from wrfrst instead).
- YSU and sfclayrev use Fortran's MIN / MAX (the second argument on ties and NaN) instead of std::min / std::max
  (bit-neutral on every suite case); ysu_topdown_pblmix defaults to WRF's 1 in ysu::run_ysu.
- GPU step segments: a failed graph launch or a later device error reports the fatal flag the previous segment raised
  first; tile workers free their upload staging buffer at exit.
- ctest: real_vs_wrf and real_vs_wrf_runs each use a work directory of their own (they ran at once in one directory).
- Input and restart readers: the `HALO_<name>` variables of CPPWRF's restart file and the wrfinput's UOCE, VOCE, VAR,
  U_FRAME and V_FRAME are read only after their rank, dimension names and extents are checked (one record of a
  Time variable, `nc_get_vara_double`; every NetCDF return code checked): a rank-2 `HALO_` variable, a staggered
  UOCE or a frame value of several records was a buffer overrun and is now an error naming the file and the
  variable. The other wrfinput fields of the column physics read one record of their expected size. Tests:
  test_restart (the bounded read), ctest restart_guard (a rank-2 or missing `HALO_` variable and a version-6 file
  refused, one patch and 2 x 2), restart_resume (U_FRAME in a wrfrst file, V_FRAME and a staggered UOCE in wrfinput).
- backend.hpp: `c_min` / `c_max` are described as CPPWRF's explicit policy (the second operand on ties and NaN, the
  Fortran source order), not as every compiled WRF site's behaviour on NaN or signed zero.

## 1.3.0 (2026-11-22)

### Changed
- The GPU FP32 builds are WRF real*4's arithmetic by default: CPPWRF_CUDA_FP32_IDENTITY (and CPPWRF_HIP_FP32_IDENTITY)
  default to ON, so `-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF` builds the FP32 identity build, bit-identical to the
  CPU FP32 build and to stock WRF real*4 on the configurations of docs/FP32_IDENTITY.md. The fast FP32 build (FMA
  contraction, CUDA's math library; 2 to 17 % faster in the paper benchmark) is `-DCPPWRF_CUDA_FP32_IDENTITY=OFF`.
  The options have no effect on FP64 and CPU builds. The suite, benchmark, tiles and MPI tools that compare the fast
  build name it explicitly.
- Translator: aligned WRF namelist validation and automatic resets with WRF v4.8.0 (`share/module_check_a_mundo.F`,
  `share/input_wrf.F` and the physics/dynamics drivers). Automatic resets and inactive options now match WRF with a
  note instead of failing: ideal `hypsometric_opt = 2` resets to 1; `sf_urban_physics` copies the innermost domain's
  value on nested runs; MSKF (`cu_physics = 11`) resets any `icloud` to 1 and sets `cu_rad_feedback = .true.` before
  checking `cu_diag`; `o3input` resets to 0 for non-RRTMG radiation; `sst_update = 0` zeroes `io_form_auxinput4` and
  `auxinput4_interval*`; non-MYNN PBL resets `bl_mynn_edmf`, `bl_mynn_edmf_dd`, `bl_mynn_output` and `icloud_bl = 1`
  to 0; `bl_pbl_physics = 5, bl_mynn_edmf = 0, ishallow = 1` is accepted; `mp_physics = 28` resets any non-zero
  `grav_settling` to 0 before range checks; `ghg_input`, `idcor` and `hail_opt` values other than 1 follow WRF's 0
  branch; `nssl_*` flags accept any `< 0` as the default; `tice2tsk_if2cold` with `fractional_seaice = 0`, `lagday`
  with `tmn_update = 0`, `opt_thcnd` outside Noah, `cugd_avedx` / ensemble sizes outside Grell-3D, `shadlen` with
  `topo_shading = 0`, `scalar_pblmix = 1` with WSM3 / WSM5 / no PBL, `gwd_dx_factor` / `gwd_if_nonhyd` on domains
  with `gwd_opt = 0`, `chem_adv_opt` and moving-nest options at their Registry defaults are accepted. MYNN
  (`bl_pbl_physics = 5`) now accepts all ported microphysics with `qc, qi, qs` (4, 6, 8, 10, 14, 16, 17, 18) and
  ACM2 (`bl_pbl_physics = 7`) all ported microphysics with `qc, qi` (4, 6, 8, 10, 14, 16, 17, 18, 28, 50, 51).
  Stricter checks matching WRF: `zdamp >= ztop` is enforced only for `damp_opt = 2` (with ideal default `ztop = 15000`);
  omitted nest `parent_id` defaults to Registry 0 (rejected) instead of `dom_id - 1`; `vert_refine_fact = 0` is
  rejected; non-`max_domains` array options (`press_levels`, `z_levels`, `ocean_z`, ...) are checked across all
  supplied elements; child `s_we`/`s_sn` no longer inherit domain 1's value; fixed `DEFAULT_ONLY_OPTIONS` key
  `"grid_allowed"`.
- Licensing: SPDX expressions name the third-party terms of the files derived from other code (AER's RRTMG / RRTM:
  LicenseRef-AER-RRTMG, verbatim from the WRF source; UCAR's Noah-MP: LicenseRef-UCAR-NoahMP; glibc: LGPL-2.1-or-later;
  CORE-MATH: MIT; wrf-python: Apache-2.0); licenses/ holds each text, NOTICE.md lists every derived file,
  docs/LICENSING.md explains the combination. No code change.

### Fixed
- CPPWRF's own restart file records the simulation start (SIMULATION_START_DATE), and a run resumed from it dates
  its files from it, as from a wrfrst file: in a chain wrfrst -> restart_NNNNNN.nc -> resumed run with run.start_date
  = the restart time (the translation of a resumed run's namelist), every date of the last run shifted by the first
  restart time.
- CPPWRF's own restart file at a last step that does not ring, with a resumed run's RingTime still pending: the
  seconds to the alarm's next ring are those to the earlier of RingTime and PrevRingTime + interval (ESMF_ClockAdvance
  rings at either; was the seconds to RingTime), so that a run resumed from it with another restart.interval rings
  there. With the adaptive time step the file's seconds come from the clock (were from step * the current time
  step; such files are not read back, results unchanged).
- WRF_ALARM_SECS_TIL_NEXT_RING_51 with a time step that is not a whole number of seconds (time.dt_den, a nest's
  parent_dt / parent_time_step_ratio): the whole seconds of the exact clock (60 / 11 s and a 55 s interval: 50 s at
  60 s, was 51 s; n * dt lay just below the whole second).
- A run without column physics writes STEPBL and STEPCU of the configuration's phys.bldt / phys.cudt into its wrfrst
  files (bl_init, cu_init), as STEPRA and STEPFG; was the namelist's when the two differ.
- HIP builds: the tiled and MPI runs' halo exchange and gathers through the device staging buffer, the fast FP32
  build's 256-lane ordered sums and the refusal of CPPWRF_TILE_CHECK in src/core/tiles.cpp, as in CUDA builds (the
  host path copied the device fields with memcpy); MPI builds take the device UUID from hipDeviceGetUuid, which
  ROCm 5 has as well (hipDeviceProp_t has its uuid member from ROCm 6.0 on). CUDA and CPU builds unchanged.
- Translator: use_adaptive_time_step rounds the starting time step to 1/100 s before max_time_step and
  min_time_step, as adapt_timestep (time.dt 0.33 for 1/3 s; a starting time step below 0.005 s with min_time_step =
  0 is refused); a negative starting / max / min_time_step_den is refused; time_step = 0 without the adaptive time
  step is an error instead of a crash of the translator.
- Benchmark tools (tools/bench): failed builds, apt, report and Drive steps and failed runs fail the session (were
  reported with a time); release assets whole or in parts; WPS only for C2 without --data-lday; a timeout kills the
  run's process group and an OOM kill is not reported as a timeout; 1-based blocks of the history-on regime;
  linear-time power integration; the post-step write charged at idle power; at least one thread; relative SVG links.
- Suite: the CUDA tiled twin of the KESSKF case reads kesskf_data.tar (was skipped for its missing data).
- HIP builds: NSSL's power table as a constexpr function and adapt_timestep's static_assert outside device code
  (hipcc 6.4 refused both). CUDA and CPU builds unchanged.

## 1.2.3 (2026-11-22)

### Added
- Coverage waves 3 to 5 against WRF 4.8.0 runs (CPU FP64 exact, each with a CUDA FP64 twin): 17 references / 51
  cases (Noah-MP with fractional sea ice and sst_update, LES-like setups without a PBL scheme, Noah-MP with MYNN,
  ACM2, BouLac and QNSE, RUC with rdlai2d, grav_settling with other PBLs, the diffusion skip flags and
  mix_isotropic, Morrison and NSSL with other cumulus schemes, isftcflx 2 and iz0tlnd 2), 16 references / 48 cases
  with 57 and 89 levels (eight physics sets: the GPU kernels' level tiers above 48 levels on real data), 19
  references / 57 cases (YSU ysu_topdown_pblmix 0, kfeta_trigger 3, Grell-Freitas and Grell-3 without shallow
  convection and with clos_choice 1, 4, 7, 10, New Tiedtke on a nest, opt_thcnd 2, the slab scheme with ifsnow 0,
  kim_tofd, damp_opt 1, top_lid, Dudhia SW with WSM3 and RRTMG LW); data tars port_cov3_data.tar, port_cov4_data.tar, port_cov5_data.tar.
- CUDA FP64 twins of the 15 CPU-only tiled real cases (run.tiles layouts on the GPU).
- An opt-in AMD HIP / ROCm build path (CPPWRF_ENABLE_HIP, off by default; docs/HIP.md) through a CUDA / HIP runtime
  portability header. Not yet built or run on AMD hardware. CUDA and CPU builds unchanged.

### Fixed
- Files test: restart.end only for fixed time step runs (an adaptive run refuses it); CI planner (cppwrf-data
  ci/plan.py): at most 90 GiB of data per suite job.
- tools/bench/bench_stats.py: the speed-up table no longer fails from its second case (a loop variable hid the
  cases table).
- wrfrst files of a resumed run whose restart.interval is shorter than the restart file's seconds to the next ring:
  WRF_ALARM_SECS_TIL_NEXT_RING_51 is the interval - (time - PrevRingTime) of output_wrf at every ring, also at the
  rings by the interval before the file's pending RingTime (P1 resumed at 5 min from a 5-min interval with 1 min:
  60 s at 6, 7, 8, 9 min, was 240, 180, 120, 60 s; a run resumed from such a file rang again at the old RingTime).
  CPPWRF's own restart file of such a ring carries the same seconds.
- A run resumed from a wrfrst file dates its files from the file's SIMULATION_START_DATE (input_wrf), not from
  run.start_date: the translation of a resumed ideal run's namelist (start_* = the restart time) run with
  run.restart_file wrote SIMULATION_START_DATE = the restart time and every date shifted by the restart time. The
  adaptive resume's clock from Times counts from that date as well. run.run_time still counts from the simulation
  start.
- FP32 builds: STEPRA, STEPBL, STEPCU and STEPFG of the wrfrst file (phy_init without the column physics, STEPRA
  from phys.radt, STEPFG without grid nudging) and topo_shading's STEPRA without radiation in WRF's REAL as ra_init,
  bl_init, cu_init and fg_init (radt 2.1 min, dt 12 s: 10, was 11). FP64 builds unchanged.
- Translator, use_adaptive_time_step: time.dt is adapt_timestep's first time step, starting_time_step /
  starting_time_step_den between max_time_step and min_time_step (their NINT(4, 8, 3 * MIN(dx, dy) / 1000) defaults
  with dx, dy of the Registry, 200 m, when the namelist has none), whatever time_step is; time_step = 0 without dx /
  dy or with a starting time step of 0 s raised to min_time_step was refused. The model computed the time step from
  [time] already; results unchanged.
- Nests of an INI: parent_grid_ratio >= 2 and the nest from parent cell 3 to 2 cells inside its parent's edge, as
  the translator checks (a ratio of 1 at the parent's edge read the parent's fields beyond their halo in the
  boundary interpolation).
- Tiedtke: the comment on cududv's level 0 (times pmfu(1) = cmfctop * pmfu(2) with a cloud top at level 2, where
  WRF's value is undefined; times 0 at level 3). No code change.

## 1.2.2 (2026-11-22)

### Documentation
- NOTICE.md: the parts derived from RRTMG / RRTM (AER) and Noah-MP (UCAR) stay under their own terms, which do not
  allow selling them or a work that contains them; a copy of CPPWRF with them may be redistributed free of charge,
  not sold. Nothing else changes.
- NOTICE.md: an additional permission under GPLv3 section 7 from the copyright holder of CPPWRF's own code: CPPWRF's
  code may be conveyed combined with those parts with the parts under AER's and UCAR's terms instead of the GPL.

### Added
- Suite: coverage wave 2, 18 WRF v4.8.0 r8 references (tests/verify/refs/cov2_refs.txt, cov2b_refs.txt; data
  port_cov2_data.tar, port_cov2b_data.tar) and 54 cases (36 CPU FP64, 18 CUDA FP64; tests/verify/cases/
  72_coverage2.py): WRF's tropical suite on P4 (TROP) and CONUS suite on the two-way nest N3 (CONL), the adaptive
  time step with cudt / bldt (ADCB, ADGT), cudt 5 / bldt 3 (CB53), MYNN with KF + cu_rad_feedback (MYKR) and BMJ +
  bmj_rad_feedback (MYBR), the UCM under MYJ (UMYJ), RRTM LW + Dudhia SW in daylight (RRDD), Grell-3D and
  Grell-Freitas-Li with cu_rad_feedback on a two-way nest (G3N, GFN), a one-way N3 (N3OW), use_mp_re 0 (MPRE0),
  o3input 0 / ghg_input 0 (O3G0), isfflx 0 (ISF0), advection orders 6 / 4 (ADV64), hail_opt 1 with radar
  reflectivity (HAIL), isftcflx 1 / iz0tlnd 1 (SFT1).
- `tools/fp32_double_rounding.py`: every decimal literal of src/ (and, with --data, text tables and namelists) whose
  rounding through binary64 to binary32 differs from gfortran's single rounding of a REAL literal; none does in
  src/, data/ or WRF v4.8.0's run directory (ctest fp32_double_rounding, fp32_double_rounding_tool).
- `tools/colab/colab_wrfref.sh` runs without a cloud bucket: data from Drive, else from the data-v1 release (parts
  checked against SHA256SUMS); `--dry-run` checks the reference list and where each tar is; a tar is staged only
  when every WRF run of the list succeeded.

### Changed
- RRTMG LW / SW, their shared code, the radiation driver and Dudhia SW compute in WRF's kind `rb` (the model's real:
  float in FP32 builds, double in FP64 builds; src/phys/ra_rrtmg_common/parkind.hpp); the table structs are templates
  on the real type and the WRF -r8 table dump is rounded member by member when an FP32 build reads it. FP64 builds
  unchanged.
- FP32 builds read the tables WRF READs into REAL (LANDUSE.TBL, Noah SOILPARM / VEGPARM / GENPARM.TBL, RUC
  VEGPARM.TBL, URBPARM.TBL, the CAM ozone files, P3's itab) with strtof, as libgfortran, instead of strtod and a
  rounding to float (src/core/parse_real.hpp); CAMtr_volume_mixing_ratio and P3's itabcoll keep double. No value of
  the shipped or reference tables rounds differently, so no result changes.

### Fixed
- Indexing outside the arrays in columns WRF itself reads outside its arrays (WRF's values there are undefined;
  defined values here, identical wherever WRF's are defined): MYNN's PBL-height and mixing-length `DO WHILE` loops
  stop at the model top; ACM2 at a mixed layer or CBL reaching the top (U3D / V3D and VCI at kte+1 read as 0);
  Kain-Fritsch and multi-scale KF with the LCL at the lowest level (a saturated one-layer source of more than 50 hPa:
  K = KLCL-1 = 0 of WRF, here the lowest two levels); Tiedtke with a cloud top at level 2 (the levels from 1; the
  [jk-1] = -1 writes are gone) and the level-0 winds of `cududv` (0, not the column buffer's bytes); YSU's PBL height
  at kpbl = 0 of the theta-li loop (the ground, 0); RRTMG SW's `jp(nlayers+1)` / `jp(0)` (model top above 191 hPa,
  every layer above 95.6 hPa); RRTM LW's Planck rows below 160 K / from 340 K and CORR1 / CORR2 below ~1 Pa (the edge
  row). None of these columns occurs in the verified cases: P1, P4, Tiedtke, ACM2, MYNN, MSKF, RRTM and adaptive runs
  of 3 steps are byte-identical.
- Start-up checks for grids the kernels would index outside their halos: e_vert >= 4 (real, WRF ideal, cppwrf_real),
  specified boundaries of at least 9 x 9 points (e_we, e_sn >= 10, WRF's smallest decomposed patch), a symmetric
  axis of at least 3 points (a narrower mirror read the other side's halo while it was written), RRTM LW with at most
  1022 layers (a fixed level array; WRF's are automatic), cppwrf_real with e_we, e_sn >= 3 and num_metgrid_levels
  >= 2; a nest's e_vert equal to its parent's, e_we - s_we and e_sn - s_sn divisible by parent_grid_ratio, and the
  nest inside its parent (the translator checked these; an INI written by hand was not).
- CUDA FP64: `diag.csv` sums in the CPU build's order (one thread per level, as the FP32 identity build), so a CUDA
  FP64 run's diag.csv equals the CPU build's; the fast FP32 build keeps the tree.
- Adaptive time step: `time_step = 0` (start_em replaces it by `starting_time_step`) stopped the translator with a
  division by zero, and `time.dt = 0` divided by zero in the model; a run resumed from a real*4 wrfrst file whose
  XTIME rounds below the restart second (3601 s: 60.016666 min) started 1 s early and missed the next boundary time
  (the resumed clock now takes the second of the file's Times); a WRF_ALARM_SECS_TIL_NEXT_RING_01 < 0 disabled the
  history output where WRF keeps the namelist's alarm; a resumed run that ends at or before the restart time took
  one step. STEPRA, STEPBL, STEPCU and the boundary time dtbc in WRF's REAL kind (float in FP32 builds; FP64
  unchanged).
- wrfout / wrfrst / auxhist3: an empty variable attribute (stagger, units, description) is one NUL byte as WRF's
  `ext_ncd_put_var_ti_char` writes it (was 0 bytes; netCDF4-python and ncdump read both as ""). The translator gives
  WRF ideal cases the namelist's start date (Registry default 1993-03-13_12:00:00), as ideal.exe and wrf.exe date
  their files; it was always 0001-01-01_00:00:00. `cppwrf-wrf` gives a resumed run the simulation start of its
  wrfrst file (the namelist's start is the restart time there).
- A two-way parent without column physics wrote CTOPO / CTOPO2 of its t = 0 frame from the initial feedback (1 + 1 ulp
  in double precision at the 1064 fed-back points of N2) where WRF's start_domain(parent) resets them to 1
  (start_em.F:1620-1621); visible only where an `iofields_filename` puts them in a history or auxiliary stream (the
  Registry writes them to restart files only). N2 for 3 minutes with `+:h:0:ctopo,ctopo2`: both domains' wrfout
  files identical to WRF r8's.
- Fatal stops: of several codes raised in one physics call the smallest stops the run (an atomic minimum on host
  and device), not the first thread's; a decomposed run (tiles, MPI ranks) takes the smallest over its workers at
  each check (`tile::first_code`, which docs/MPI.md described but nothing called), so every worker count and layout
  stops with the same code and message as one patch. A decomposed CUDA run reads each check at once. No other
  result changes.
- CPPWRF's own restart files (format version 6; version 5 files are refused) continue the uninterrupted run also in
  the alarms and attributes of the WRF files: a run resumed from `restart_<step>.nc` wrote START_DATE and
  WRF_ALARM_SECS_TIL_NEXT_RING_55 from the restart time instead of the start of the run that wrote the file (also
  without column physics, whose start step the file did not carry); with a `restart.interval` that is not a multiple
  of `time.dt` its restart alarm counted from its first ring (5 min resumed at 2 min with 100 s and dt 60 s: 4, 6 min
  instead of 4, 5 min); the file now carries the alarm's PrevRingTime and one-shot RingTime. A resumed run with
  another `output.out_interval` or `phys.auxhist3_interval` kept the file's history / auxhist3 alarm and ignored the
  new interval (CLWRF's statistics then used the new one): the alarm now rings at the new interval from the start of
  the run that wrote the file, where CLWRF's period counts from.
- A run resumed from wrfrst files takes STEPRA / STEPBL / STEPCU / STEPFG, FCX / GCX and LANDUSE_ISICE from its own
  options as wrf.exe's phy_init and start_em do, not from the file (P1 resumed with `phys.bldt = 2`: STEPBL 1 in the
  next wrfrst file, now 2); a two-way parent writes the phy_init fields of its wrfrst file (TOPOSLPX, SLOPE, CTOPO, ...)
  instead of those of the initial feedback, which med_nest_initial of a restart does not run.
- `phys.sst_update = 1`: domain 1 with `phys.auxinput4_interval = 0` and runs of `run.run_time = 0` without the
  interval stop as WRF does (they ran without reading wrflowinp, or with a 360-minute alarm in the wrfrst files); an
  interval below 0 is refused (model and translator).
- WRF's files of a run without a namelist entry `interval_seconds` take `time.interval_seconds` or the wrfbdy interval
  (WE_JUST_D01_LBC, DTBC), not the Registry default 10800 s.
- The single-layer UCM with `use_wudapt_lcz = 0` and WUDAPT LCZ classes stops with WRF's message ("USING 10 WUDAPT LCZ
  WITHOUT URBPARM_LCZ.TBL") instead of the generic urban input error; a class beyond the table is refused after it.
- cppwrf_real stops on a `physics_suite` other than none, CONUS or tropical ("Unrecognized physics suite"), also when
  every physics option is given (unit test).
- `tools/wrf2cppwrf.py`: iofields_filename lines `-:i:-0:...` (stream 0) and lines of an unknown stream type after an
  input line (WRF keeps the stream type) are refused like the other input stream changes; a stream id such as `++0`
  is an error instead of a translator crash (translator tests).
- `cppwrf_real --precision float` computed its vectorised logf / expf with the host's libmvec (`_ZGVbN4v_logf`,
  `_ZGVbN4v_expf` imported in every x86-64 build): now with CPPWRF's port of glibc 2.41's libmvec, as the model; the
  ctest `fp32_host_libm_real` checks that cppwrf_real imports none of the float functions or their vector variants.
- `CPPWRF_RANK_DEVICES`: only the entry a rank takes must be below the number of devices it sees (a list for a node
  with more devices stopped the ranks of a smaller node; unit test). K processes or tiles on one GPU also divide the
  RRTMG batch's minimum of 2048 columns and NSSL's 1 GiB row work space by K; `run.tiles` on one GPU now divides the
  work budget among the tiles as MPI ranks do (launch sizes only, the values do not change).
- `CPPWRF_VM_DUMP` with `run.tiles` or MPI ranks stops with an error (it read past the patch's fields).
- The unit tests link `${CMAKE_DL_LIBS}` (test_glibc_mathf's dlopen on glibc < 2.34).
- A run resumed from CPPWRF's own restart file with another `restart.interval` kept the file's PrevRingTime with the
  new interval (5 steps of 60 s with 3 min, resumed from the 5-min file with 1 min: a ring every step with -60 s to
  the next one in the wrfrst files); the file now also records its interval, and another one sets the alarm as a run
  resumed from WRF's file has it (RingTime = the restart time + WRF_ALARM_SECS_TIL_NEXT_RING_51, PrevRingTime = the
  restart time). Another `phys.auxhist3_interval` with `output_diagnostics = 1` is refused there (CLWRF's sums, or the
  means just finished, belong to the file's interval; resume from the wrfrst file instead). Tests restart_resume,
  restart_alarm (resumed from the native file with 30 s and 0: the rings of WRF's rule).
- A run resumed from wrfrst files wrote STEPRA from the namelist's `radt`, not `phys.radt` (and STEPFG without grid
  nudging from the namelist's `fgdt`, not `fdda.fgdt`): the configuration's values where it has them, as ra_init /
  fg_init (test restart_resume: `phys.radt = 2`, STEPRA 2).
- The single-layer UCM with LCZ classes beyond URBPARM.TBL and FRC_URB2D 0 or missing (WPS without NUDAPT) stopped
  with the generic urban input error before WRF's message; it now stops with WRF's ("USING 10 WUDAPT LCZ WITHOUT
  URBPARM_LCZ.TBL") as for a given fraction.
- `physics_suite`: TRIM drops trailing blanks only, so `' conus'` or `'co nus'` is "Unrecognized physics suite" as in
  WRF (they were taken as CONUS); cppwrf_real checks the suite before it reads the physics options (an unknown suite
  without `sf_surface_physics` stopped on `sf_surface_physics = -1`); the translator keeps a quoted value's leading
  blanks (unit and translator tests).
- `tools/wrf2cppwrf.py`: iofields_filename lines are split as WRF's get_fieldstr (an empty field swallows the next
  `:`), the stream type of a line with a bad operation still counts, the stream id is read also on a line without a
  field list (`+:i:++0`, `+:i::tsk`: WRF's read error), and only ASCII digits are digits (`²` crashed the translator)
  (translator tests).
- `run.tiles` with `--check`: the reference run counts among the processes sharing its GPU's work budget.

## 1.2.1 (2026-11-21)

### Added
- CUDA builds with `CPPWRF_MPI`: a rank's device is chosen by its rank among the ranks of its node, not by its global
  rank: `CPPWRF_RANK_DEVICES=d0,d1,...` gives node-local rank `r` device `d[r mod n]` (an entry that is not a device
  number stops the run with an error); without it the ranks of a node spread over the visible devices
  (`r mod count`); with one visible device nothing changes. Rank 0 prints the mapping unless every rank is on
  device 0. The ranks of a node that share a device divide its work-buffer budget, and every rank sizes the column
  physics' work buffer (and NSSL's row work space) for its own patch instead of the whole domain. Results unchanged
  (test_rank_device; docs/MPI.md).

### Fixed
- Tiles (`run.tiles`) with the cumulus diagnostics (`cu_diag = 1`: Grell-Freitas, Grell-3D): convtrans_prep's
  averaging counter (module_convtrans_prep.F's SAVE stepave_count) was one for the process, so the tile threads raced
  on it and counted once per tile; at the end of an averaging period QC_CU / QI_CU / RAINCV_B took another divisor
  than in the run without tiles (with cu_rad_feedback, RRTMG's clouds and all fields after it). It is now one per
  tile thread, which gives the run without tiles (WRF's own numtiles > 1 counts once per tile; the references run one
  tile). New suite case real_a05g_r8_fp64_cpu_1h_t2x2. MPI ranks were not affected.
- `CPPWRF_KTIME=1` with tiles: the tile threads updated the kernel-time table without a lock (it could crash); the
  debug flag of `CPPWRF_VM_DUMP` was shared by the tile threads.
- The FP64 builds no longer contain glibc 2.41's float functions (src/core/glibc_mathf_host.cpp came in through the
  core library); they call the host's libm, as documented. The FP64 results are unchanged.
- `cppwrf_real --precision float` uses glibc 2.41's float functions in every x86-64 build, not only in the FP32
  builds (one `cppwrf_real` is installed into a prefix: that of the flavour installed last).
- test_glibc_mathf compared the ports with themselves (the executable's own expf ... from the core library); it now
  looks the system libm's functions up in libm.so.6. The ctest `fp32_host_libm` also rejects libmvec vector variants
  of the ported functions (none are imported).
- Noah (`sf_surface_physics = 2`) with the NLCD40 land use: IVGTYP's urban categories 23 and 25 become 24 before
  LSMINIT, as WRF's lsm_init does (the Noah tables, SNOALB with `rdmaxalb = false`, the urban model, the IVGTYP of
  the history); CPPWRF kept 23 and 25. Two-way nest feedback with NLCD40 and Noah is refused (the model keeps no
  LU_INDEX apart from the remapped IVGTYP for phy_init after the feedback).
- `cppwrf_real` stops as real.exe does (check_nml_consistency) when a nest's `sf_surface_physics`,
  `sf_sfclay_physics`, `ra_lw_physics` or `ra_sw_physics` differs from domain 1's, or its `bl_pbl_physics`,
  `cu_physics` or `gwd_opt` differs and is not 0; it wrote the files.
- The single-layer urban model stops as WRF's phy_init does with `use_wudapt_lcz = 1` when no urban point of the
  domain has an LCZ class above 3 (URBPARM_LCZ.TBL with the 3 old urban classes); it ran.
- `tools/wrf2cppwrf.py`: `iofields_filename` changes of the input stream `i:10` (wrffdda, grid_fdda) are an error as
  those of `i:0` and `i:4`; with `--format native` the model read wrffdda whole without a word.
- `phys.num_soil_layers` follows `sf_surface_physics` as WRF's set_physics_rconfigs does (5 for the slab, 4 for Noah
  and Noah-MP, 6 or 9 for RUC); without the key the slab and RUC took 4 and read the 5- or 6-layer soil fields into
  4-layer arrays. Another value is replaced, with a note (the translator writes WRF's value).
- CPPWRF's own restart files (`restart_<step>.nc`, format version 5; version 4 files are refused): a run resumed from
  one now continues the uninterrupted run bit for bit also with `bucket_mm` / `bucket_J`, `prec_acc_dt`,
  `output_diagnostics`, `grid_fdda` (the interpolation weight and the end of the nudging), the Grell-Freitas /
  Grell-3D cumulus averaging (convtrans_prep's counter), `sst_update` and `grid_fdda` at a read time (the record was
  read again), Noah-MP's CROPCAT and land-point flag, and REFL_10CM in the native snapshots. The resumed run counted
  solve_em's curr_secs2 from the restart step, as wrf.exe's run resumed from a wrfrst file does (which stays so).
- A run resumed from wrfrst files takes the history's phy_init fields (THC, LU_STATE; without column physics also
  ALBBCK, ALBEDO, ZNT, EMISS, Z0, EMBCK, MAVAIL) from the restart file, as wrf.exe does (its landuse_init runs only
  when not restarting). They came from wrfinput_d01, so a two-way nested parent whose fed-back land use differs from
  its wrfinput wrote its wrfinput values into the resumed run's wrfout and wrfrst files (and a run resumed from those
  started from them: the slab's THC).

### Changed
- `phys.sst_update = 1` needs `phys.auxinput4_interval` (WRF stops without an auxinput4 interval); the
  configuration's default was 360 minutes. The translator writes it.
- `time.interval_seconds` of the adaptive time step defaults to the wrfbdy file's interval (it was 21600 s; a value
  that differs from the file's was and is an error).

## 1.2.0 (2026-11-21)

### Added
- CPU FP32 builds (`-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF`) bit-identical to WRF v4.8.0 real*4 (gfortran, the default real*4 build)
  on 24 configurations, every history variable at tolerance 0 (docs/FP32_IDENTITY.md):
  - P1 and P4 (Kain-Fritsch, WSM6, RRTMG; 3 minutes with history every step, and 1 h) and P2, P3, P9 (1 h): start_em's
    base state, al and p, the lbc_fcx_gcx relaxation coefficients, Noah init (SNOALB, snow albedo, the TSLB freezing
    test), history T, WSM6's mp_wsm6_init products and moist_physics_prep_em's R_v/R_d rounded where WRF real*4 rounds
    them (new data tar fp32id_data.tar; the 1 h references of P1, P2, P3 and P9 are earlier real*4 runs in
    suite_data.tar).
  - Noah-MP NMP1 (1 h; the MOST resistances' four logarithms through the 4-lane logf) and Thompson P6 (3 minutes and
    1 h; thompson_init's constants, coefficients and table builders in REAL where WRF real*4 has them, the runtime's
    rain slope and sedimentation powers scalar; new data tar fp32id2_data.tar).
  - MYNN (MYNN surface layer + MYNN-EDMF PBL + RRTMG) and the aerosol-aware Thompson C01 (mp_physics 28), 3 minutes
    and 1 h: the MYNN psi tables as the real*4 build vectorises them (entry 0 a constant, entries 1 .. 1000 four lanes
    at a time), powf(x, 0.5) in their scalar fallback, MYNN-EDMF's plume powers through the 4-lane powf;
    thompson_init's QNWFA / QNIFA profiles and QNWFA2D emission in REAL (new data tar fp32id3_data.tar).
  - WDM6 / WDM5, Morrison with hail and with graupel, GWDO, the one-way nest N1, the two-way nest N3 with the P4
    physics, monotonic advection on N3 (MON3), Kessler, ACM2 + Pleim-Xiu, QNSE, Tiedtke, New Tiedtke, the Dudhia
    shortwave and the slab LSM, each 3 minutes and 1 h (new data tar fp32id4_data.tar): wdm6init's and
    MORR_TWO_MOMENT_INIT's folded powers and products in REAL (all their module variables compared with WRF real*4's),
    WDM's (24.)**(.3333333) and (actr*1.E-6)**3; the nest start (start_em's base state with max_dom > 1,
    blend_terrain, adjust_tempqv, press_adj) and the feedback (copy_fcn's average, sm121, smdsm) in REAL.
  - glibc 2.41 libmvec's 4-lane SSE4 float functions (`_ZGVbN4v_expf`, `_ZGVbN4v_logf`, `_ZGVbN4vv_powf` with Intel's
    rcpps table, `_ZGVbN4v_sinf`, `_ZGVbN4v_cosf`, `_ZGVbN4v_atanf`) ported lane by lane (src/core/libmvec_f4.hpp,
    the expf / logf / sinf / cosf tables from tools/libmvec_f4_tables.py) and called for the iterations WRF's vectorised loops give them
    (test_libmvec_f4 `all`: expf, logf, sinf, cosf and atanf over every float input, 0 differ).
- The suite's CPU FP32 backend `cpu_sp` (`tests/verify/wrf_suite.py --cpu-sp BIN`): 44 cases (`real_*_fp32_cpu`,
  `real_*_fp32_cpu_1h`) against WRF real*4 history files at tolerance 0; each also checks its configuration against
  the reference's namelist.
- CUDA FP32 identity build, CMake option `CPPWRF_CUDA_FP32_IDENTITY` (CUDA, FP32; default OFF): the CPU FP32 build's
  results on the GPU, bit for bit. `--fmad=false --ftz=false --prec-div=true --prec-sqrt=true` in every kernel; on
  the device glibc 2.41's scalar float functions (src/core/glibc_mathf.hpp: expf, logf, powf, sinf, cosf from libm's
  FMA variants; atanf, atan2f, tanf, tanhf, log10f, asinf, acosf, sinhf, erff, tgammaf from CORE-MATH's correctly
  rounded sources), glibc's double functions as in the CUDA FP64 build, libmvec's 4-lane float functions with the CPU
  build's lane rule, and `Reducer::sum` in the CPU build's order. Proofs: tools/glibc_mathf_proof.cpp (each port
  against the system libm over every float input, 2^32 pairs plus special values for powf and atan2f) and
  tools/glibc_mathf_proof_cuda.cu (device against host, RTX 2060); test_glibc_mathf samples the scalar ports in every FP32 build. The
  suite's backend `cuda_sp_id` (`--cuda-sp-id BIN`) runs the 44 `cpu_sp` cases on the GPU (`real_*_fp32_cuda_id*`)
  against the same references at tolerance 0. Cost on an RTX 2060 (P4, 10 steps): kernel time +3 %. The default
  fast CUDA FP32 build is unchanged.

### Changed
- GPU performance, the same operations in the same order (FP64 bit-identical on the Colab G4: 338 / 338 at b2d320c):
  KWSolve's Thomas sweeps carry the neighbour level of w in a register and pi = 4 atan(1) is computed once on the
  host; an RK3 step with microphysics and lateral boundaries runs one interior calc_p_rho_phi (no second one after
  spec_bdy_final, as solve_em); the nest feedback synchronises the device only when CUDA graphs are off or at the run's last root step; the Noah
  parameter tables by device pointer instead of a by-value copy in the kernel arguments; RRTMG LW's batch kernel
  KLwLevel forms the band sums and the level fluxes in registers; the surface driver no longer zeroes u_phytmp /
  v_phytmp and KSfcPrep fills only the levels the surface layer schemes read. On the G4 the 1 h stepping times were
  unchanged within a second (P4 FP64 9 s, N3 FP64 16 -> 15 s, P4 FP32 3 s, N2 FP32 3 -> 2 s); the gains showed on an
  RTX 2060. No new benchmark: docs/PERF.md's numbers stay.
- NSSL microphysics keeps 1.1's launch (the row on the launch's second index): a launch with the row on the first
  index, tried in the same work, was 3-8x slower on the G4 (real_p4n18 1 h: 47 -> 135 s FP64, 19 -> 165 s FP32) and
  was reverted.
- FP32 results change where the FP32 builds now round as WRF real*4 does (Added); the fast CUDA FP32 build shares
  these code paths, its suite tolerances are unchanged. FP64 results are unchanged.

### Fixed
- FP32 results no longer depend on the host's libm: they were WRF real*4's only on a host with glibc 2.41 (Debian 13,
  where the references were made; glibc 2.40 / 2.41 replaced several float functions with CORE-MATH's correctly
  rounded ones), and an Ubuntu 24.04 build (glibc 2.39) differed from t = 0. The FP32 builds' executables (`cppwrf`,
  `cppwrf_real`; CPU, and the host code of the CUDA builds) define glibc 2.41's float functions themselves
  (src/core/glibc_mathf_host.cpp), in place of libm's for every call: expf, logf, powf, sinf, cosf, sincosf, tanf,
  atanf, atan2f, asinf, acosf, sinhf, tanhf, log10f, erff, tgammaf. Each port equals glibc 2.41's libm on every float
  input (tools/glibc_mathf_proof.cpp, also built without `-march`) and the device equals the host
  (tools/glibc_mathf_proof_cuda.cu, RTX 2060). The ctest `fp32_host_libm` checks that `cppwrf` imports none of them.
  Proven on a Colab G4 with binaries built on Ubuntu 24.04 (gcc 13.3, glibc 2.39): `cpu_sp` 44 / 44 and `cuda_sp_id`
  44 / 44 at tolerance 0.

### Verification
- CPU CI on 0bc4652 (GitHub-hosted runners, Debian 13 container, run 44860146094), all exact: suite PASS 386, FAIL 0,
  SKIP 0 (the 342 CPU FP64 checks and the 44 `cpu_sp` cases), WRF output files 42, ideal files 14, differential tests
  32 / 32, ctest 35 / 35.

## 1.1.1 (2026-11-20)

### Added
- `cppwrf_real`: TAVGSFC input (`FLAG_TAVGSFC = 1`), as real.exe uses it: the surface pressure of sfcprs2 with
  `sfcp_to_sfcp` (ez_method, Tv from TAVGSFC; module_initialize_real.F:1409-1418, 8457-8515), the SST and skin
  temperature of the inland-lake points (`ISLAKE` >= 0) and the SST repair of water points without lake information
  (`ISLAKE = -1`; :2814-2861), the slab's TMN and its terrain adjustment (module_soil_pre.F:650-686, 1043-1055).
  `use_tavg_for_tsk` stops only when neither TSK nor TAVGSFC is in the input (:607-612); its TSK = TAVGSFC is
  overwritten with SKINTEMP in the metgrid branch (:704-713), as in WRF. Verified: wrfinput / wrfbdy identical to WRF
  real.exe's (ctest real_vs_wrf_runs: V111TAV, V111TAVT with use_tavg_for_tsk, V111TS2P with sfcp_to_sfcp, V111TSLAB
  with the slab, V111TNL with ISLAKE = -1).
- `cppwrf_real`: the surface pressure from PMSL (sfcprs, module_initialize_real.F:1419-1426, 8147-8453) when the
  input has no PSFC or no SOILHGT, with the 850 / 700 / 500 hPa lapse rates, the lapse-rate formula over the lowest
  level without those levels, or TAVGSFC (ez_method); without PSFC the computed surface pressure goes into the lowest
  input level (:1436-1446). Verified: identical to WRF real.exe's (ctest real_vs_wrf_runs: V111PMS without PSFC,
  V111PMT without PSFC with TAVGSFC, V111NSH without SOILHGT). The error without enough input names FLAG_PSFC, FLAG_SOILHGT, FLAG_SLP and sfcp_to_sfcp, as
  real.exe's message.

### Fixed
- Translator: cumulus (`cu_physics`) or radiation (`ra_lw_physics` / `ra_sw_physics`) without a surface layer and a
  land surface model was translated, and `cppwrf` then stopped at its start. It is now an error at translation time
  with the model's message (PhysicsSuite's with a cumulus scheme, model.cpp's with radiation only). wrf.exe runs these
  namelists (module_physics_init.F:3316-3409 and 3414-3851 do nothing for `sf_sfclay_physics` /
  `sf_surface_physics = 0`); CPPWRF's column physics needs both schemes. Unit tests in tests/tools.

### Verification
- tests/unit/test_fatal: a land-use category beyond the VEGPARM.TBL dataset of MMINLU (a synthetic table read by
  read_noah_tables) stops the Noah LSM with REDPRM's "Warning: too many input landuse types"
  (module_sf_noahlsm.F:2436-2438), as wrf.exe stops on MODIS input whose lake category 21 stays land with
  `ISLAKE = -1` (VEGPARM.TBL lists 20 MODIS categories). The check was already in the model (src/phys/lsm_noah).

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
  x**2.0, NSSL's dgam, Noah-MP groundwater, RRTMG SW heatfac). RRTMG (`rb`), WSM6's literals, the nest SINT and the
  `vm` atan / sin / cos / log10 are single precision in CPU FP32 builds too. FP64 output is unchanged; the GPU FP32
  values of the suite moved (docs/FP32_ACCURACY.md, section 1.1.0).

### Performance
- RRTMG LW / SW (and the radiation driver's RRTMG paths, Dudhia SW) compute in single precision in CUDA FP32 builds:
  RRTMG's real kind `rb` (src/phys/ra_rrtmg_common/parkind.hpp) is float there, as in WRF real*4, and double
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
