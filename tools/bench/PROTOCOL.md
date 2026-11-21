# Paper benchmark protocol

This is the benchmark protocol of the paper and of the vendor material: what runs, how it is timed, how often, how
the numbers are summarised, what "cost" and "energy" mean, and how the same runs show bit identity. The tools are
in this directory; `tools/bench/colab_paper.sh` runs the whole protocol in one Colab session. The older benchmark
(`colab_bench.sh`, `bench_run.py`, `bench_report.py`, docs/PERF.md) is unchanged and stays reproducible; it is the
1-hour comparison of releases 1.0 to 1.2.

## Sessions

| Session | Runtime | What | Estimate | Compute units |
|---|---|---|---|---|
| 1 (required) | Colab G4: AMD EPYC 9B45 (24 cores, 48 vCPUs) + RTX PRO 6000 Blackwell (96 GB) | WRF v4.8.0 (stock and tuned real*4) and every CPPWRF build, cases C1-C4 | 7.9 h (`paper_run.py plan --nc0`), at most 10 h | about 79, at most 100 (10.05 per hour in November 2026) |
| 2 (optional) | Colab L4, then Colab A100 | CPPWRF's three CUDA builds against its CPU builds, cases C1, C2 | about 2-3 h each (not measured) | `colab usage`'s rate x hours (the L4 and A100 rates were not checked here) |

The estimate comes from the G4 times of docs/PERF.md (stepping per simulated hour of every model, WRF in one
process from the b30ec5b session's logs, the write time per history frame) and the b30ec5b session's set-up (WRF's
four builds took 13 minutes, the CUDA builds 1-2 minutes each). `paper_run.py plan [OPTIONS]` prints it for any
option set. The job stops waiting after 11 hours (`--hours`).

## Cases

Every case is a configuration the suite already proves bit-identical in FP64 (docs/STATUS.md), run longer than the
suite does. Inputs are existing data tars on Drive (and the GitHub release cppwrf-data data-v1): fp2_data.tar's
run_FP2_P4_24h_r8 (real.exe inputs of P4's domain with boundaries for 24 h; its wrfinput is run_P4_r8's) for C1 and
C3, n3_data.tar's run_N3h_r8 (the suite's N3 inputs, boundaries valid for 6 h) for C4. C2 is made on the runtime as
the older benchmark's large domain L (its tables and namelist from run_FP2_P4_24h_r8).

| Case | Configuration | Grid (cells) | Steps | Length | Why | Verified before (suite) |
|---|---|---|---|---|---|---|
| C1 | P4 physics (WSM6, Kain-Fritsch, YSU, revised MM5 surface layer, Noah, RRTMG LW/SW) with radt 15 min; run_FP2_P4_24h_r8 | 119 x 89 x 44 (466 k) | 1440 | 24 h from 2026-10-12 00 UTC | a whole day: the shortwave works for 11 h; the production radiation step | FP64 24 h / 72 h (radt 1, the long-run inputs); FP32 identity 1 h (radt 1) |
| C2 | the daytime large domain: 15 km Lambert at 39 N 35 E (tools/bench/namelist.wps), WPS 4.6.0 + GFS 0.25 degree (6 h and 12 h forecasts of 2026-10-12 00 UTC) + real.exe of WRF real*8; P4 physics with radt 15 min | 359 x 269 x 44 (4.25 M) | 360 | 6 h from 06 UTC (08:20 - 14:20 local solar time) | a GPU-sized domain; full sun | FP64: the older benchmark's L (same domain, night) against WRF in one process on a G4 |
| C3 | MORR: P4 with Morrison two-moment microphysics (mp_physics 10), radiation every minute (radt 1); run_FP2_P4_24h_r8 | 119 x 89 x 44 | 360 | 6 h from 00 UTC | physics-heavy: radiation every step and a two-moment scheme | FP64 1 h; FP32 identity 1 h |
| C4 | N3: two-way nest, d02 120 x 90 x 44 at 5 km (ratio 3, feedback, smooth_option 2), P4 physics with radt 1 on both domains; run_N3h_r8 | 466 k + 475 k | 360 + 1080 | 6 h from 00 UTC | nesting and its feedback | FP64 1 h (and 24 h from the long-run inputs); FP32 identity 1 h |

Sunrise at 39 N 35 E on 12 October is about 03:45 UTC, so C3 and C4 have about 2 h of daylight at their end, C1 a
whole day, C2 only daylight. A Thompson or MYNN physics-heavy case is not in session 1: its WRF real*8 reference
needs the `-r8 -fdefault-double-8` build (with `-r8` alone gfortran makes Thompson's DOUBLE PRECISION variables
16-byte, docs/STATUS.md, which would make WRF real*8 a straw man), and its real*4 runs compute Thompson's tables for
about three minutes at every start. Both are possible later (another WRF build and cached tables).

## Builds

All builds are made on the runtime from the commit under test (`git archive`) and WRF's release tarball.

- WRF v4.8.0 with its own `configure` for GNU gfortran / gcc and MPICH in the Debian 13 chroot (gfortran 14.2,
  glibc 2.41: the toolchain of the suite's references): dmpar (option 34) and dm+sm (35), real*4 and real*8
  (`configure -r8`), stock flags (`FCOPTIM = -O2 -ftree-vectorize -funroll-loops`, no `-march`). The compile is
  retried with `-j 4` and `-j 1` when WRF's parallel make loses a module (the c9c95c5 session's real*4 dmpar link).
  Also WRF real*4 tuned for the CPU (model `wrf_r4t`, in session 1 by default): `FCOPTIM = -O3 -march=native
  -funroll-loops` (FMA contraction allowed, gfortran's default), dmpar and dm+sm, screened like the others; it
  answers "how much of the speed-up is compiler flags". `--models` without wrf_r4t leaves it out.
- CPPWRF CUDA on the runtime's Ubuntu (CUDA 13): `cuda` (FP32, the fast default: FMA, CUDA's math), `cuda_sp_id`
  (`-DCPPWRF_CUDA_FP32_IDENTITY=ON`: the CPU FP32 build's bits), `cuda_dp` (FP64, `--fmad=false`, glibc's math on
  the device).
- CPPWRF CPU in the chroot: `cpu-dp` (FP64, the release build: `-O3 -march=native`, `CPPWRF_PORTABLE_LIBMVEC` on:
  Intel's libmvec log on any CPU), `cpu-dp-hwlog` (`CPPWRF_PORTABLE_LIBMVEC` off: this CPU's libmvec, as the WRF
  executables built here call it) and `cpu-sp` (FP32).

### Manifest

`manifest.py collect` writes, for each side (chroot "cpu", runtime "gpu"), the commit, the CPU (model, vCPUs, cores,
sockets, NUMA nodes, AVX-512), memory, kernel, OS, glibc, compilers (gcc, gfortran, nvcc, MPICH), the GPU (name,
driver, memory, compute capability, power limit, maximum clocks), the energy counters found, and per build: the
CMakeCache options (every `CPPWRF_*`, flags, CUDA architectures), the flavour they make (e.g. "cuda-fp32 identity",
"cpu-fp64, portable libmvec on, native on"), `cppwrf --version` and the SHA-256 of the executable; per WRF build the
configure.wrf lines DMPARALLEL, OMPCPP, FCOPTIM, PROMOTION, ... and the SHA-256 of wrf.exe and real.exe. The two are
merged into `paper/manifest.json`; the report prints its summary.

## What is timed

Both models print WRF's timing lines: WRF in rsl.out.0000, CPPWRF with `output.wrf_timing = true` ("Timing for main:
time T on domain N: S elapsed seconds" after each step, "Timing for Writing F for domain N: S elapsed seconds" after
each write). `bench_parse.py` splits both the same way:

| Quantity | Definition |
|---|---|
| wall | the process, launch to exit (`time.monotonic()` around `mpiexec` / `cppwrf`): MPI start-up, input, initialisation, steps, output, finalisation |
| stepping | sum of domain 1's "Timing for main" minus the writes inside them (a nest's steps run inside its parent's step) |
| step 1 | domain 1's first step minus its writes: CUDA graph capture, first-touch allocations, the nest's start (WRF and CPPWRF both open the nest in d01's first step) |
| steady | stepping minus step 1, over steps 2 .. N |
| I/O | every "Timing for Writing" line of every domain |
| other | wall - stepping - I/O |

WRF writes the frame at t = 0 inside its first step (med_before_solve_io), CPPWRF writes domain 1's frame at t = 0 in
its set-up; the parser counts each where it happens. The older benchmark's CPPWRF stepping time (the "cppwrf: done"
line) contained N3's d02 output; this split does not.

Settings of every CPPWRF timed run: `output.wrf_timing = true`, `output.diag_interval = 1000000` (no diag.csv
reductions; WRF computes none), `CPPWRF_MEMINFO` only in the warm-up runs (it synchronises the device). Neither
setting changes the results: P1, 3 steps, with and without them, CPU and CUDA FP64, every variable of every frame
identical (the dry run below). No restart files: `restart_interval = 0` (WRF writes none and tools/wrf2cppwrf.py then
gives CPPWRF no `[restart]` section; the older benchmark's `restart_interval = 1440` made CPPWRF write a restart file
at the last step that WRF did not write, inside its wall time).

Two history settings ("regimes"), the same namelist for both models apart from them:

- off: `history_interval = 99999`, only the frame at t = 0: the compute comparison;
- on: a frame every hour (`--hist-min 60`), NetCDF-4 with WRF's default chunking, shuffle and deflate level 2 in
  both models. CPPWRF compresses the chunks on host threads (the same bytes, docs/PERF.md "NetCDF output"); WRF
  compresses serially on rank 0 (no quilting). `--nc0` (in the recommended command) also times CPPWRF GPU FP32
  with `CPPWRF_NC_THREADS=0`, HDF5's own serial filter as WRF uses it, to show how much of the history-on gain is
  the threaded compression.

On the GPU the steps run asynchronously (CUDA graphs, no wait per step): a "Timing for main" line is the host's time
for the step, and a write waits for the steps queued before it. With history off only the last step waits, so
stepping and steady are the device's times (step 1's device work falls into steady: one step of hundreds). With
history on, the GPU's split between stepping and I/O is not meaningful: the report gives the wall time and the
speed-ups of the wall time for that regime.

## Process layouts

WRF (dmN: N MPI ranks of the dmpar build; smNxT: N ranks x T OpenMP threads of the dm+sm build), screened per case
and precision with 1 simulated hour, history off: dm12, dm24, dm48, sm24x2, sm12x4 (the older benchmark's) and the
physical-core hybrids sm12x2, sm6x4, sm8x3. Each rank is bound to cores / N physical cores (`mpiexec -bind-to
core:K`; beyond 24 ranks one hardware thread each), its threads with `OMP_PROC_BIND=close OMP_PLACES=cores`,
`OMP_STACKSIZE=512M`, `ulimit -s unlimited`. The layouts run in turn (`--screen-reps N` repeats the round with the
order turned); the layout with the smallest median stepping time is WRF's layout for that case and precision in the
timed blocks. The report prints the whole screening table.

CPPWRF CPU FP64: OpenMP on 24 threads (`OMP_PLACES=cores`) or 48 (`OMP_PLACES=threads`), `OMP_PROC_BIND=close`,
screened the same way. CPPWRF GPU: 4 host threads (`OMP_NUM_THREADS=4`, as the older benchmark); NetCDF compression
threads at their default (`CPPWRF_NC_THREADS`: the hardware threads).

## Repetitions and order

- History off: 1 warm-up block (not counted: page cache, the GPU's module load and clocks; it records
  `CPPWRF_MEMINFO`), then 4 timed blocks (`--reps 4`: with the tuned WRF, 5 blocks take the estimate from 7.9 h
  to 8.5 h, too close to 10 h). History on: 3 timed blocks (the history-off blocks warmed up).
- A block runs every model of every case once. Within a case the order of the models turns by one each block
  (block k starts with model k), so no model always runs first or after the same neighbour; WRF and CPPWRF of one
  block share the machine's state of that moment.
- CPPWRF CPU FP64 runs in history-off blocks 1-3 only (`--cpu-reps 3`): it is the port's CPU comparison, not the
  headline, and its runs are the longest; the warm-up block before has read the same inputs.
- Each GPU run is preceded by a 3 s pause (thermal settling) and 1.5 s of idle power sampling.
- Before every run the number of other running processes is recorded (`/proc/stat` procs_running) with the mean
  CPU clock; runs with other processes running or with GPU throttling samples are flagged in the report, not
  dropped. Nothing else runs during the screening and the timed blocks: builds and downloads end first, and the
  screening waits until the GPU side's builds are done.

## Statistics

Per case, model and regime over the timed repetitions: median, quartiles (q25 - q75) and minimum - maximum of wall,
stepping, step 1, steady, I/O, other, GPU energy, mean power and SM clock. From the medians: seconds per simulated
day, SYPD (simulated years per wall-clock day, from the wall time and from the steady stepping), simulated time per
wall time, ns per cell and step (steady stepping over the cells x steps of all domains after step 1).

Speed-up against WRF real*4 and real*8 (and the tuned real*4): the ratio of WRF's run to CPPWRF's run of the same
block, reported as the median over the blocks with the smallest and largest block ratio, and the ratio of the
medians; history off for stepping, steady and wall, history on for wall.

## Cost

Colab compute units per simulated day = median wall / 3600 x 86400 / simulated seconds x the session's rate
(`colab usage`, read at the end of the session: 10.05 units per hour for the G4 in November 2026). The runtime costs
the same whichever processor a run uses, so for WRF this is its price on the GPU runtime with the GPU idle: an
upper bound for WRF. `--cpu-rate R` adds WRF's price at R units per hour, a CPU-only runtime with the same CPU, as a
stated assumption; it is not measured in these sessions (no CPU-only runtime of the same CPU was checked) and the
report says so. No cloud list prices are given: they change and are not verified here; the report's wall times and
the formula let a reader apply their own.

## Power and energy

During every GPU run `telemetry.py` samples `nvidia-smi --query-gpu=timestamp,power.draw,clocks.sm,clocks.mem,
temperature.gpu,utilization.gpu,memory.used,pstate,clocks_event_reasons.active` every 100 ms from 1.5 s before the
start to 0.3 s after the end. Energy = the trapezoidal integral of power.draw over the run's window (interpolated at
its ends); also mean and peak power, the idle power before the start, the median and minimum SM clock, the maximum
temperature, the peak device memory and the samples with throttle reasons (power cap, hardware or thermal
slowdown). This is the GPU board's energy only: not the host CPU, memory, disks or the data centre. CPU energy
counters (RAPL / powercap, perf) are probed on both sides and recorded in the manifest; they are expected to be
hidden in the Colab VM. When none is readable no CPU energy and no GPU-versus-CPU energy ratio is reported.

## Bit identity in the same runs

The history-on outputs of the last timed block (WRF at its fastest layouts, the three GPU builds) are kept, and the
proof phase adds untimed runs with history on, several at a time on the CPU while the GPU idles: WRF real*8 and
real*4 in one process (`mpiexec -n 1`, unbound), CPPWRF CPU FP64, CPU FP64 with this CPU's log, CPU FP32 (8
threads each). Then `tests/verify/exact.py` (every variable of every frame at tolerance 0, NaN and signed zero
exact) with the SHA-256 of every compared file:

| Comparison | Expected | Meaning |
|---|---|---|
| GPU FP64 vs CPU FP64 | identical | the GPU's FP64 bits are the CPU's |
| CPU FP64 (this CPU's log) vs WRF real*8, one process, this runtime | identical | CPPWRF FP64 = WRF real*8 on the benchmarked case and length; on AMD the default CPU build computes Intel's vector log and differs from a WRF built here by that function only (docs/STATUS.md) |
| CPU FP64 (default) vs WRF real*8, one process | identical on Intel, differs on AMD | shown for completeness |
| GPU FP32 identity vs CPU FP32 | identical | the identity build's GPU bits are the CPU FP32 build's |
| CPU FP32 vs WRF real*4, one process, this runtime | informative | CPPWRF reproduces WRF real*4 on references made on Intel (glibc's powf starts from an `rcpps` estimate that AMD computes differently, docs/FP32_IDENTITY.md), so on the G4's AMD CPU a difference is expected where that estimate matters; the WRF real*4 identity proof stays the suite's 44 cases |

Accuracy of the fast FP32 build: `tools/verify/fp32_accuracy.py compare` for the pairs fp32-r8 (GPU FP32 against GPU
FP64, the bit-identical real*8 result), r4-r8 (WRF real*4 in one process against it), fp32id-r8, r4best-r8 (WRF
real*4 at its fastest layout) and r8best-r8dm1 (WRF real*8 at its fastest layout against one process: WRF's own
layout effect), and `report` with the ratio R = metric(fp32-r8) / metric(r4-r8) per variable and hour
(`paper/accuracy.md`, `paper/svg_accuracy/`). The relative L2 at the last frame against GPU FP64 is in the report.

## Outputs

`paper/raw_runs.jsonl` (one line per run: the command, environment, exit, wall, the parsed times, load, CPU clock,
GPU telemetry summary, energy counter), `runs.csv`, `verify.jsonl`, `cases.json`, `best.json` (the chosen
layouts), `manifest.json`, `accuracy/`, `logs/CASE/` (each run's rsl.out.0000 or CPPWRF log, its GPU samples),
`report.md` and `summary.json` (`bench_stats.py`, with the session's rate) and `svg/` (`bench_plot.py`: speed-ups
against WRF real*4 and real*8 with the block range, the wall-time split per model, GPU energy per simulated day, the
layout screening).

## Commands

No cloud bucket and no gcloud. The workstation needs the Colab CLI and the rclone remote cppwrf-drive; the runtime
gets only that remote's section (deleted at the end). `tools/bench/paper_job.sh` on the runtime streams each data
tar from Drive (`rclone cat | tar x`), else from the GitHub release data-v1 (512 MiB parts checked against its
SHA256SUMS). The results come back with `colab download` and are also copied to
`cppwrf-drive:cppwrf-g4results/paper_<date>/` (paper.tar.gz, results.tar.gz), from where the launcher takes them
if the download fails. `--dry-run` prints the job's arguments, the estimate and where each data tar is, and starts
nothing.

Session 1 (the release manager, on the workstation):

    tools/bench/colab_paper.sh --gpu G4 --save-lday -- --nc0

`--save-lday` puts C2's inputs on Drive (cppwrf-data/paper_lday_data.tar) for session 2.

Session 2 (after session 1):

    tools/bench/colab_paper.sh --gpu L4 --data-lday
    tools/bench/colab_paper.sh --gpu A100 --data-lday

(`--set gpu` is the default off the G4: cases C1, C2; models gpu_fp32, gpu_fp32id, gpu_fp64; proof CPU FP64 and
CPU FP32 with half the vCPUs each; no WRF builds unless C2's inputs must be made.)

Locally, a dry run of the pipeline (P1, 3 steps; no WRF executables, so WRF's runs are recorded as missing):

    tools/bench/bench_exec.py agent ROOT/q --bins BINS &      # BINS/cuda_dp/cppwrf, BINS/cuda_sp_id/cppwrf
    touch ROOT/gpu_ready
    tools/heavy.sh --exclusive tools/bench/paper_run.py all --root ROOT --data DIR --cases P1 \
      --case P1=run_P1:3m --cpu-bin cpu-dp=... --cpu-bin cpu-sp=... --layout dm2 --threads 6 --hist-min 1 \
      --reps 3 --reps-on 2 --cpu-reps 2 --proof cpu_fp64,cpu_fp32
    touch ROOT/q/stop

## Known limitations

- One virtual machine per session, shared physical hosts: interleaving, medians, quartiles, the load check and
  the GPU clock telemetry bound the variation but do not remove it. Earlier G4 sessions agreed within 3.5 % (GPU)
  and 3-9 % (WRF at its best layout); single WRF runs at other layouts varied up to 15 % (docs/PERF.md).
- The screening runs each layout once for 1 h: a layout within that noise of the fastest may be chosen; the
  screening table is printed.
- WRF is built with GNU compilers only (stock flags, and the tuned real*4 build); Intel, NVIDIA or AMD Fortran
  compilers are not measured. WRF uses no I/O quilting and no parallel NetCDF.
- WRF runs on the G4's CPU only; its cost on a CPU-only runtime is an assumption (`--cpu-rate`), not a measurement.
- Single GPU; CPPWRF's multi-GPU MPI runs are not part of this protocol (docs/MPI.md).
- GPU energy is the board's; host energy is not measured.
- WRF real*4's bits on the G4's AMD CPU are not those of the Intel references (see above); the FP32 identity in
  the same runs is GPU identity build = CPU FP32 build, with the WRF real*4 identity from the suite.
- The physics-heavy case is Morrison with radiation every minute; Thompson / MYNN cases need the r8d8 WRF build
  and cached tables (later).
- C3 and C4 start at 00 UTC (the existing inputs) and are mostly night; C1 covers a whole day and C2 is daytime.
- Session 2's CPUs differ from the G4's (and its WRF is not run): its numbers are CPPWRF GPU builds across GPUs.
