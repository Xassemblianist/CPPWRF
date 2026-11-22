# Paper benchmark protocol

This is the benchmark protocol of the paper and of the vendor material: what runs, how it is timed, how often, how
the numbers are summarised, what "cost" and "energy" mean, how the same runs show bit identity, and what the data
cannot show. The tools are in this directory; `tools/bench/colab_paper.sh` runs the whole protocol in one Colab
session. The older benchmark (`colab_bench.sh`, `bench_run.py`, `bench_report.py`, docs/PERF.md) is unchanged and
stays reproducible; it is the 1-hour comparison of releases 1.0 to 1.2.

## Scope: what the numbers can and cannot support

- **One node, one GPU.** Session 1 compares CPPWRF on one RTX PRO 6000 Blackwell (96 GB) with WRF v4.8.0 on the
  same virtual machine's CPU: one AMD EPYC 9B45 slice of 24 cores / 48 vCPUs. Every speed-up is "this GPU against
  WRF on this 24-core CPU slice with GNU compilers", and is written that way in tables and figure captions. It is
  not a claim about WRF on a full two-socket server, with other compilers or on several nodes. CPPWRF's multi-GPU
  MPI path (docs/MPI.md: host-staged halos, no CUDA-aware MPI) is not measured here; no scaling claim is made.
- **The WRF baseline.** WRF is built from its release with its own `configure` (GNU, MPICH), stock flags, plus a
  tuned real*4 build (`-O3 -march=native -funroll-loops`) that answers "how much is compiler flags". No tuned
  real*8 build, no Intel / AMD / NVIDIA Fortran compiler, no I/O quilting, no parallel NetCDF: the baseline is WRF
  as its default GNU build runs, at its fastest screened process layout.
- **Cases.** Four configurations the suite already proves bit-identical (below). They are not a community
  benchmark grid (CONUS 12 km / 2.5 km); the speed-ups hold for these domains and physics.
- **Energy** is the GPU board's (nvidia-smi), never the host's: no CPU energy counter is readable in the Colab VM
  (probed and recorded). No energy ratio between GPU and CPU runs is reported, and no energy saving against WRF is
  claimed from these sessions.
- **Cost** in Colab compute units is the price of the runtime used; for WRF it is the GPU runtime with the GPU
  idle (an upper bound). A CPU-only price is a stated assumption (`--cpu-rate`), not a measurement.
- **Variation.** One VM per session on shared hosts: medians, bootstrap intervals, interleaving and the logged
  steal time describe the variation within the session; they do not measure the variation between VMs ("Known
  limitations").

## Sessions

| Session | Runtime | What | Estimate | Compute units |
|---|---|---|---|---|
| 1 (required) | Colab G4: AMD EPYC 9B45 (24 cores, 48 vCPUs) + RTX PRO 6000 Blackwell (96 GB) | WRF v4.8.0 (stock real*4 / real*8 and tuned real*4) and every CPPWRF build, cases C1-C4 | 8.6 h (`paper_run.py plan --nc0`; 8.0 h with `--reps 4`), at most 10 h | about 86, at most 100 (10.05 per hour in November 2026) |
| 2 (optional) | Colab L4, then Colab A100 | CPPWRF's three CUDA builds timed, cases C1, C2; CPU FP64 / FP32 only as untimed proof runs | about 2-3 h each (not measured) | `colab usage`'s rate x hours (the L4 and A100 rates were not checked here) |

The estimate comes from the G4 times of docs/PERF.md (stepping per simulated hour of every model, WRF in one
process from the b30ec5b session's logs, the write time per history frame), 30 s per GPU cool-down after a GPU run,
and the b30ec5b session's set-up (WRF's four builds took 13 minutes, the CUDA builds 1-2 minutes each; 7 more
minutes for the tuned builds). `paper_run.py plan [OPTIONS]` prints it for any option set. The job stops waiting
after 11 hours (`--hours`).

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
whole day, C2 only daylight. radt 1 in C3 / C4 is the suite's verified setting, not a production choice (it weights
radiation heavily; C1 and C2 use radt 15). A Thompson or MYNN physics-heavy case is not in session 1: its WRF real*8
reference needs the `-r8 -fdefault-double-8` build (with `-r8` alone gfortran makes Thompson's DOUBLE PRECISION
variables 16-byte, docs/STATUS.md, which would make WRF real*8 a straw man), and its real*4 runs compute Thompson's
tables for about three minutes at every start. Both are possible later (another WRF build and cached tables). Every
model reads the same real*8 real.exe output; WRF real*4 converts it on input, which is outside the steady steps.

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

### Manifest and what is pinned

`manifest.py collect` writes, for each side (chroot "cpu", runtime "gpu"), the commit, the CPU (model, vCPUs, cores,
sockets, NUMA nodes, AVX-512), memory, kernel, OS, glibc, compilers (gcc, gfortran, nvcc, MPICH), the GPU (name,
driver, memory, compute capability, power limit, maximum clocks), the energy counters found, and per build: the
CMakeCache options (every `CPPWRF_*`, flags, CUDA architectures), the flavour they make (e.g. "cuda-fp32 identity",
"cpu-fp64, portable libmvec on, native on"), `cppwrf --version` and the SHA-256 of the executable; per WRF build the
configure.wrf lines DMPARALLEL, OMPCPP, FCOPTIM, PROMOTION, ... and the SHA-256 of wrf.exe and real.exe. The two are
merged into `paper/manifest.json`; the report prints its summary.

Pinned: the CPPWRF commit; WRF v4.8.0's release tarball; WPS 4.6.0; the GFS files of one cycle; gfortran 14.2 and
glibc 2.41 (Debian 13's, the suite's toolchain). Not pinnable on Colab and therefore recorded: the runtime image
(`host.txt`: os-release, kernel, driver, nvcc), every package version on both sides (`host_packages.txt`,
`paper/packages_chroot.txt`; Debian trixie is a point-release stream), and the SHA-256 of every input as used: each
data tar as streamed (`data.txt`, whichever of Drive or the GitHub release served it; the GitHub parts are also
checked against the release's SHA256SUMS), the WRF source tarball, wps.tar.gz (the WPS executables the launcher
uploads), the geography tarball and the two GFS files (`paper/inputs_sha256.txt`).

## What is timed

Both models print WRF's timing lines: WRF in rsl.out.0000, CPPWRF with `output.wrf_timing = true` ("Timing for main:
time T on domain N: S elapsed seconds" after each step, "Timing for Writing F for domain N: S elapsed seconds" after
each write; WRF also "Timing for processing wrfinput file / lateral boundary"). `bench_parse.py` splits both the same
way. Domain 1's main lines cut the log into step intervals, and what the two models do at different places is
placed explicitly:

- WRF opens a nest inside domain 1's first step (nests_to_open: allocation, the nest's wrfinput, the interpolation,
  start_domain), reads the lateral boundaries inside steps (at t = 0 and every `interval_seconds`) and writes the
  frame at t = 0 inside step 1.
- CPPWRF opens the nests and reads all boundaries in its set-up, before its first step's clock starts, and writes
  domain 1's frame at t = 0 there too (a nest's frame at t = 0 inside domain 1's first step).

| Quantity | Definition | Comparable across models |
|---|---|---|
| wall | the process, launch to exit (`time.monotonic()` around `mpiexec` / `cppwrf`) | yes: everything |
| steady | domain 1's steps 2 .. N minus the writes and input reads inside them: the computation of the steps (a nest's steps run inside its parent's) | yes: the same work in both |
| step_s | steady / steady_steps: seconds per domain-1 step | yes: **the headline compute metric** |
| step 1 | domain 1's first step minus its writes and inputs: WRF's nest opening and first boundary read, CPPWRF's CUDA graph capture | no: part of the fixed cost |
| I/O | every "Timing for Writing" line | yes with history off; see below for history on |
| input in steps | WRF's input lines inside the steps (the boundaries every 6 h; the nest's wrfinput in step 1); CPPWRF: none | (reported) |
| other | wall - step 1 - steady - I/O - input in steps: start-up, the inputs before the first step, initialisation, finish | no: the models read inputs and open nests at different places |
| marginal | step_s x N + the writes after t = 0 + the input reads in steps 2 .. N: what grows with the simulated time | yes |
| fixed | wall - marginal: start-up, input, the frame at t = 0, step 1's set-up, the finish, once per run | yes (once per run in both) |

Speed-ups are computed for step_s (history off) and the wall time (history off and on). The older "stepping"
(step 1 + steady) is printed but no speed-up is computed from it, because step 1 holds different work in the two
models.

On the GPU the steps run asynchronously (CUDA graphs): a step's main line is printed when the host has launched it,
the device's work of step n is waited for when step n + 1 launches, and the last step is waited for before its line.
With history off, the intervals 2 .. N of a single-domain CUDA run therefore hold the device work of steps 1 .. N,
and its steady_steps is N (N - 1 for WRF and CPPWRF CPU). A nested CUDA run waits inside domain 1's first step (the
nest's frame at t = 0 reads the device), so its intervals 2 .. N hold steps 2 .. N plus at most the last part of
step 1 still in flight (a nest substep and the feedback); it uses N - 1, an error below one step of 360. With
history on, a write waits for the steps queued before it, so the CUDA builds' split between steps and writes is not
meaningful: the report prints "n/p" for it and gives the wall time, other, fixed and marginal (whose sums are
exact), and speed-ups of the wall time only; the figures draw those runs' steps and output as one bar.

Settings of every CPPWRF timed run: `output.wrf_timing = true`, `output.diag_interval = 1000000` (no diag.csv
reductions; WRF computes none), `CPPWRF_MEMINFO` only in the warm-up runs (it synchronises the device). Neither
setting changes the results: P1, 3 steps, with and without them, CPU and CUDA FP64, every variable of every frame
identical (the dry run below). No restart files: `restart_interval = 0` (WRF writes none and tools/wrf2cppwrf.py then
gives CPPWRF no `[restart]` section; the older benchmark's `restart_interval = 1440` made CPPWRF write a restart file
at the last step that WRF did not write, inside its wall time).

Two history settings ("regimes"), the same namelist for both models apart from them:

- **off, the primary comparison:** `history_interval = 99999`, only the frame at t = 0;
- on: a frame every hour (`--hist-min 60`), NetCDF-4 with WRF's default chunking, shuffle and deflate level 2 in
  both models (the same format and settings). CPPWRF compresses the chunks on host threads (the same bytes,
  docs/PERF.md "NetCDF output"); WRF compresses serially on rank 0 (no quilting). `--nc0` (in the recommended
  command) also times CPPWRF GPU FP32 with `CPPWRF_NC_THREADS=0`, HDF5's own serial filter as WRF uses it: that row
  is the like-for-like history-on comparison, and its difference to the default row is the threaded compression.
  WRF's I/O quilting, PnetCDF and classic NetCDF are not measured; history-on numbers are secondary.

## Process layouts

WRF (dmN: N MPI ranks of the dmpar build; smNxT: N ranks x T OpenMP threads of the dm+sm build), screened per case
and precision, history off, 30 simulated minutes (`--screen-min`): dm12, dm24, dm48, sm24x2, sm12x4 (the older
benchmark's) and the physical-core hybrids sm12x2, sm6x4, sm8x3. Each rank is bound to cores / N physical cores
(`mpiexec -bind-to core:K`, probed once: an MPI without `-bind-to` runs unbound; beyond 24 ranks one hardware thread
each), its threads with `OMP_PROC_BIND=close`, `OMP_PLACES=cores` while ranks x threads fit the 24 cores and
`OMP_PLACES=threads` beyond, `OMP_STACKSIZE=512M`, `ulimit -s unlimited`. WRF's tiling is its default. Per case: one
warm-up run of each precision (not counted), then 2 rounds (`--screen-reps 2`) over all layouts, the order turned
and reversed in the second round, the precisions alternated; the layout with the smallest median seconds per step
(steady) is that precision's layout in the timed blocks. The report prints the whole table (median and range per
layout).

CPPWRF CPU FP64: OpenMP on 24 threads (`OMP_PLACES=cores`) or 48 (`OMP_PLACES=threads`), `OMP_PROC_BIND=close`,
screened the same way (a warm-up, then 2 rounds); CPPWRF's own CPU MPI mode is not screened. CPPWRF GPU: 4 host
threads (`OMP_NUM_THREADS=4`, as the older benchmark); NetCDF compression threads at their default
(`CPPWRF_NC_THREADS`: the hardware threads).

## Repetitions and order

- Warm-up: every model of every case once (CPPWRF CPU FP64 too), 30 simulated minutes (`--warm-min`), history off,
  not counted: the executables and libraries in the page cache, the GPU's module load and clocks,
  `CPPWRF_MEMINFO`. Before it, every input file of every case is read once (page cache).
- History off: 5 timed blocks (`--reps 5`); CPPWRF CPU FP64 in blocks 1-3 (`--cpu-reps 3`: it is the port's CPU
  comparison, not the headline, and its runs are the longest). History on: 3 timed blocks (`--reps-on 3`), after
  the history-off blocks (everything warm; the `--nc0` row is the warm GPU FP32 binary with another setting).
- A block runs every model of every case once. The order of the models in block k of case number i is row
  (k x cases + i) of a Williams design over the regime's models (`paper_run.py run_order`): over a regime's blocks
  every model runs first and runs right after every other model (session 1: all 42 ordered neighbour pairs of the
  7 models occur in both regimes; every model runs first 2-3 times with history off, 1-2 times with history on),
  so neither a position nor a predecessor is systematic. Each run records its position and predecessor.
- GPU cool-down: before each GPU run at least 3 s (`--pause`) and until the GPU is within 5 C (`--cool-delta`) of
  its idle temperature (measured by the agent at its start), at most 90 s (`--cool-max`); a WRF or CPU run before a
  GPU run is natural cool-down. The wait and the temperatures are recorded; a cool-down that ends at its limit is
  flagged.
- Each GPU run has 1.5 s of idle power sampling before it (the sampler's fields are probed once per session, not
  before each run).
- Recorded per run and flagged in the report, never dropped: other running processes before the start
  (`/proc/stat` procs_running), the CPU's steal, iowait and busy time during the run (`/proc/stat`; steal above 1 %
  is flagged), the CPU clock before and once a second during the run (`/proc/cpuinfo`; a VM may show a fixed
  value), the GPU's thermal / hardware slowdown samples, and a system clock step (the epoch window against the
  monotonic wall). Nothing else runs during the screening and the timed blocks: builds and downloads end first, and
  the screening waits until the GPU side's builds are done.

## Statistics

Per case, model and regime over the timed repetitions: median, quartiles (q25 - q75), minimum - maximum, a 95 %
percentile bootstrap interval of the median (10000 resamples, fixed seed, 3 or more values) and the coefficient of
variation, of wall, step_s, steady, step 1, I/O, input in steps, other, fixed, marginal, GPU energy (run and steady
window), mean and steady power and SM clock. From the medians: SYPD (simulated years per wall-clock day) from the
wall time and from step_s, simulated time per wall time, ns per cell and step (step_s over the cell updates of all
domains per domain-1 step).

Speed-up against WRF real*4, tuned real*4 and real*8: per block, WRF's run over CPPWRF's run of that block, over the
blocks both ran in; reported as the median block ratio with its 95 % bootstrap interval and its range (smallest -
largest block ratio), and the ratio of the two models' medians over the same blocks. History off: step_s (headline)
and wall; history on: wall. With 5 blocks the bootstrap interval of a median is coarse (it cannot be narrower than
the spread of the middle values): it is printed so that a reader sees the spread, not as a significance claim. No
hypothesis test is made.

## Cost

A run of S simulated seconds is split into its fixed part (start-up, input, the frame at t = 0, the finish: once
per run) and its marginal part (the steps at the steady rate and the frames after t = 0). One simulated day costs
fixed + marginal x 86400 / S: the start-up is counted once per forecast, not scaled with the run length (scaling
the whole wall time by 86400 / S would count a 6 h case's start-up four times). The report gives, at the session's
rate (`colab usage`, read at the end: 10.05 compute units per hour for the G4 in November 2026), the compute units
of the measured run, of the fixed part, of the marginal part per simulated day and of one forecast day. The runtime
costs the same whichever processor a run uses, so for WRF this is its price on the GPU runtime with the GPU idle:
an upper bound for WRF, not a CPU-only price. `--cpu-rate R` adds WRF's and CPPWRF CPU's price at R units per hour,
a CPU-only runtime with the same CPU, as a stated assumption; it is not measured in these sessions and the report
says so. No cloud list prices are given: they change and are not verified here; the wall times and the formula let
a reader apply their own.

## Power and energy

During every GPU run `telemetry.py` samples `nvidia-smi --query-gpu=timestamp,power.draw,clocks.sm,clocks.mem,
temperature.gpu,utilization.gpu,memory.used,pstate,clocks_event_reasons.active` every 100 ms (line-buffered, so no
sample is lost when the sampler stops) from 1.5 s before the start to 0.3 s after the end. The run's log reaches the
agent through a pipe; the arrival times of domain 1's first and last "Timing for main" lines mark the steady window.
Energy = the trapezoidal integral of power.draw over the run's window (interpolated at its ends) and over the steady
window; also mean, steady and peak power, the idle power before the start, the median and minimum SM clock, the
maximum temperature, the peak device memory, the gaps between the window's ends and the first / last samples, the
samples with a thermal or hardware slowdown (NVML clock event reasons 0x8 HwSlowdown, 0x20 SwThermalSlowdown, 0x40
HwThermalSlowdown, 0x80 HwPowerBrakeSlowdown: "throttled") and, counted on their own, the samples at the software
power cap (0x4 SwPowerCap: normal at full load). 0x1 GpuIdle, 0x2 ApplicationsClocksSetting, 0x10 SyncBoost and
0x100 DisplayClockSetting are not throttling.

The energy of one simulated day = the fixed energy (the run's energy minus steady power x the marginal time) once +
steady power x the marginal time of a day. This is the GPU board's energy only: not the host CPU, memory, disks or
the data centre. CPU energy counters (RAPL / powercap, perf) are probed on both sides and recorded in the manifest;
they are expected to be hidden in the Colab VM. When none is readable no CPU energy and no GPU-versus-CPU energy
ratio is reported.

## Bit identity in the same runs

The history-on outputs of the last timed block (WRF at its fastest layouts, the three GPU builds) are kept, and the
proof phase adds untimed runs with history on, several at a time on the CPU while the GPU idles: WRF real*8 and
real*4 in one process (`mpiexec -n 1`, unbound), CPPWRF CPU FP64, CPU FP64 with this CPU's log, CPU FP32 (8 threads
each). Then `tests/verify/exact.py` (every variable of every frame at tolerance 0, NaN and signed zero exact;
dimensions, types, variable attributes and the global attributes but TITLE) with the SHA-256 of every compared
file:

| Comparison | Expected | Meaning |
|---|---|---|
| GPU FP64 vs CPU FP64 | identical | the GPU's FP64 bits are the CPU build's (both compute Intel's vector log) |
| CPU FP64 (this CPU's log) vs WRF real*8, one process, this runtime | identical | CPPWRF FP64 = WRF real*8 built and run on this machine, on the benchmarked case and length |
| CPU FP64 (default) vs CPU FP64 (this CPU's log) | identical on Intel, differs on AMD | the two builds differ in glibc's vector log / log10 only (`rcpps`, which Intel and AMD compute differently) |
| CPU FP64 (default) vs WRF real*8, one process | identical on Intel, differs on AMD | shown for completeness |
| GPU FP32 identity vs CPU FP32 | identical | the identity build's GPU bits are the CPU FP32 build's |
| CPU FP32 vs WRF real*4, one process, this runtime | informative | CPPWRF reproduces WRF real*4 on references made on Intel (glibc's powf starts from an `rcpps` estimate that AMD computes differently, docs/FP32_IDENTITY.md), so on the G4's AMD CPU a difference is expected where that estimate matters |

The chain on the G4's AMD CPU, stated precisely. In these runs: GPU FP64 = CPU FP64 (default build), and CPU FP64
with this CPU's log = WRF real*8 built and run here. The default build and the this-CPU's-log build differ only in
two vector functions, and on AMD they give different results. So on this machine the GPU's FP64 result is the
result WRF real*8 gives on an Intel x86-64 CPU (where the default CPU build equals WRF: the suite's references,
docs/STATUS.md), not the one WRF gives on this AMD CPU; the two WRF results differ by WRF's own Intel / AMD
difference (about 1e-14 after one step). The sessions run no WRF on an Intel CPU, so for these case lengths the
Intel link is the suite's (the same configurations at 1 h, C1's physics at 24 h / 72 h), not a measurement in the
session. An Intel WRF real*8 run of C1-C4 (one process, from the published inputs) closes it and is planned before
the paper.

Accuracy of the fast FP32 build: `tools/verify/fp32_accuracy.py compare` for the pairs fp32-r8 (GPU FP32 against GPU
FP64, the bit-identical real*8 result), r4-r8 (WRF real*4 in one process against it), fp32id-r8, r4best-r8 (WRF
real*4 at its fastest layout), r4tuned-r8 (the tuned WRF real*4) and r8best-r8dm1 (WRF real*8 at its fastest layout
against one process: WRF's own layout effect), and `report` with the ratio R = metric(fp32-r8) / metric(r4-r8) per
variable and hour (`paper/accuracy.md`, `paper/svg_accuracy/`, every pair drawn). The relative L2 at the last frame
against GPU FP64 is in the report.

## Outputs

`paper/raw_runs.jsonl` (one line per run: the command, environment, exit, wall, the parsed times, block, position
and predecessor, load, steal / iowait, CPU clock, cool-down, GPU telemetry summary, energy counter), `runs.csv`,
`verify.jsonl`, `cases.json`, `best.json` (the chosen layouts), `manifest.json`, `inputs_sha256.txt`,
`packages_chroot.txt`, `accuracy/`, `logs/CASE/` (each run's rsl.out.0000 or CPPWRF log, its GPU samples); the job's
`results.tar.gz` (data.txt with the data tars' digests, host.txt, host_packages.txt, the build and fetch logs). The
launcher writes `report.md` and `summary.json` (`bench_stats.py` with the session's rate: the final report; the
chroot's `paper/report.md` is the same without the rate) and `svg/` (`bench_plot.py`: steady and wall speed-ups
against WRF real*4, tuned real*4 and real*8 with the bootstrap interval, history off and on; the wall-time split of
the median run of each model; the GPU energy of one simulated day; the layout screening relative to each case's
fastest layout).

## Data and code availability

During the work the inputs are on Drive (cppwrf-drive:cppwrf-data) and in the public GitHub release
Xassemblianist/cppwrf-data data-v1, the results on Drive (cppwrf-g4results/paper_<date>/). For the paper, the tagged
CPPWRF release, the three input tars (fp2_data.tar, n3_data.tar, paper_lday_data.tar), every session's paper.tar.gz
and results.tar.gz (raw runs, logs, manifests, accuracy statistics, the kept outputs of the comparisons as far as
the archive's size limit allows) and this protocol are to be deposited on Zenodo with DOIs, and the launcher and
paper_job.sh are to take the archive's URLs and SHA-256 sums as their public source. Until then Drive and the GitHub
release are working copies, not the archive.

## Commands

No cloud bucket and no gcloud. The workstation needs the Colab CLI and the rclone remote cppwrf-drive; the runtime
gets only that remote's section (deleted at the end). `tools/bench/paper_job.sh` on the runtime streams each data
tar from Drive (`rclone cat | tar x`), else from the GitHub release data-v1 (512 MiB parts checked against its
SHA256SUMS), recording each tar's SHA-256. The results come back with `colab download` and are also copied to
`cppwrf-drive:cppwrf-g4results/paper_<date>/` (paper.tar.gz, results.tar.gz), from where the launcher takes them if
the download fails. A failed phase (prepare, screen, timed, proof, verify, report), translation, build or download,
or a failed local report makes the launcher exit non-zero. `--dry-run` prints the job's arguments, the estimate and
where each data tar is, and starts nothing.

Session 1 (the release manager, on the workstation):

    tools/bench/colab_paper.sh --gpu G4 --save-lday -- --nc0

`--save-lday` puts C2's inputs on Drive (cppwrf-data/paper_lday_data.tar) for session 2.

Session 2 (after session 1):

    tools/bench/colab_paper.sh --gpu L4 --data-lday
    tools/bench/colab_paper.sh --gpu A100 --data-lday

(`--set gpu` is the default off the G4: cases C1, C2; timed models gpu_fp32, gpu_fp32id, gpu_fp64; untimed proof
runs CPU FP64 and CPU FP32 with half the vCPUs each, for the bit identity on that GPU; no WRF builds unless C2's
inputs must be made.)

Locally, a dry run of the pipeline (P1, 3 steps; no WRF executables, so WRF's runs are recorded as missing):

    tools/bench/bench_exec.py agent ROOT/q --bins BINS &      # BINS/cuda_dp/cppwrf, BINS/cuda_sp_id/cppwrf
    touch ROOT/gpu_ready
    tools/heavy.sh --exclusive tools/bench/paper_run.py all --root ROOT --data DIR --cases P1 \
      --case P1=run_P1:3m --cpu-bin cpu-dp=... --cpu-bin cpu-sp=... --layout dm2 --threads 6 --hist-min 1 \
      --reps 3 --reps-on 2 --cpu-reps 2 --proof cpu_fp64,cpu_fp32
    touch ROOT/q/stop

## Known limitations

- One virtual machine per session on shared physical hosts: the variation within the session is measured
  (quartiles, bootstrap intervals, steal time, clocks), the variation between VMs is not. Earlier G4 sessions agreed
  within 3.5 % (GPU) and 3-9 % (WRF at its best layout); single WRF runs at other layouts varied up to 15 %
  (docs/PERF.md). Repeating session 1 on another day is the check; it is outside this session's budget.
- 5 history-off and 3 history-on blocks are what a 10-hour session holds with four cases and seven models; small
  differences (a few per cent) between CPPWRF builds or between layouts are not resolved.
- The screening runs each layout twice for 30 minutes after a warm-up: layouts within that noise of the fastest are
  not separated (the table shows the ranges). WRF's tiling (`numtiles`, `nproc_x` / `nproc_y`) is its default.
- WRF is built with GNU compilers only (stock flags and the tuned real*4 build; no tuned real*8 build); Intel,
  NVIDIA or AMD Fortran compilers are not measured. WRF uses no I/O quilting and no parallel NetCDF.
- WRF runs on the G4's CPU slice only; its cost on a CPU-only runtime is an assumption (`--cpu-rate`), not a
  measurement.
- Single GPU; CPPWRF's multi-GPU MPI runs are not part of this protocol (docs/MPI.md).
- GPU energy is the board's; host energy is not measured; no energy comparison with WRF.
- WRF's bits on the G4's AMD CPU are not those of the Intel references (see above); for these case lengths the
  GPU FP64 = Intel WRF real*8 link comes from the suite until the planned Intel run.
- The physics-heavy case is Morrison with radiation every minute; Thompson / MYNN cases need the r8d8 WRF build and
  cached tables (later). radt 1 in C3 / C4 is the suite's setting, not a production choice.
- C3 and C4 start at 00 UTC (the existing inputs) and are mostly night; C1 covers a whole day and C2 is daytime.
- Session 2's CPUs differ from the G4's (and its WRF is not run): its numbers are CPPWRF GPU builds across GPUs.
