# Ensembles on one GPU (ENS1: design and first phase)

An ensemble (perturbed initial or boundary conditions, several physics members) is many runs of the same domain.
One CPPWRF run of a regional domain often does not fill a large GPU. This document compares three ways to run N
members on one GPU, says which one CPPWRF implements first and why, and records what was measured. The hard rule:
every member's output is byte-identical to that member run alone with the same build, so a member in an FP64 or
FP32-identity build stays bit-identical to WRF wherever its single run is.

## 1. Use

```ini
[ensemble]
members = 3
member1 =                                    ; empty: the configuration as it is
member2 = phys.mp_physics=6 | moist.species=qv,qc,qr,qi,qs,qg
member3 = real.dir=/data/run_perturbed_b | output.out_interval=30
mode = threads                               ; threads (default) | sequential
```

- `memberI` lists `section.key=value` overrides separated by `|` (`#` and `;` start comments in the file). They
  apply after the command line's `--set`, so `--set` changes every member and `memberI` changes one. A member without
  a `memberI` key is the configuration as it is.
- Member I writes to `<output.dir>/memberI`, where `output.dir` is the configuration's (default `out/<case>`). An
  `output.dir` in `memberI` wins. A WRF directory of the configuration (`output.wrf_dir`) is a root in the same way:
  member I writes its WRF files to `<output.wrf_dir>/memberI` (an `output.wrf_dir` in `memberI` wins). Every
  directory a member writes into is its own: its `output.dir` (run.json, diag.csv, native snapshots and restart files,
  in every `output.format`, `wrfout` too), its WRF directory, and the directories a namelist `history_outname`,
  `rst_outname` or `auxhist3_outname` names below that (`sub/wrfout_d<domain>_<date>`). Two members with one of them
  in common, of the same kind or not (one member's `output.dir` as another's WRF directory), are refused; the
  directories are compared as the file system resolves them (`a/x/../b`, symbolic links). A stream name with a `..`
  component (a file outside the member's WRF directory) or a date tag in its directory part is refused, and so are
  the debugging dumps (`CPPWRF_VM_DUMP`, `CPPWRF_NOAH_DUMP`, `CPPWRF_GF_DUMP`, `CPPWRF_GFL_DUMP`,
  `CPPWRF_NTIEDTKE_DUMP`, `CPPWRF_G3_DUMP`: one file name for every member) in an ensemble of more than one member.
- The member's configuration is the file plus the overrides, without the `[ensemble]` section. That is also what
  its restart files record, so a member's restart file resumes as a single run.
- `cppwrf ens.ini --set ensemble.member=I` runs member I alone, as a plain single run, into the same directory
  (to repeat one member, or to check it).
- `mode = threads`: every member on its own host thread, with its own CUDA stream, CUDA graphs and fields on the
  current GPU, and `OMP_NUM_THREADS / N` OpenMP threads (CPU builds). `mode = sequential`: one member after the other
  in the main thread.
- Exit status: the largest member status (3: a member stopped on non-finite values; the others run on). An error
  in a member (a WRF fatal stop, a configuration error, a CUDA error) stops every member at its next step and is
  reported as the error of the run; the log names the member that failed first.
- A member thread that cannot be started (a system thread limit) is that member's error: the members already
  started stop at their next step and are joined; members never started are reported as such.
- Not yet: nested members (`domains.max_dom > 1`), tiles (`run.tiles`) or MPI ranks in an ensemble (refused at
  start-up).

## 2. What makes the members identical to their single runs

A member runs the same code as a single run: `Model::build`, `DomainRunner::start`, then for every step
`before_step`, `RK3::step`, `after_step`, then `finish` (src/cases/ensemble.cpp, `run_member`, the loop of
src/main.cpp). The members share no field. What they share is either read-only or per thread:

| State | Where | In an ensemble |
|---|---|---|
| CUDA stream of all launches and copies | `gpu_stream_raw`, thread_local (core/backend.cpp) | one per member thread (made for tiles, P03) |
| CUDA graphs of the step, their cache, the pinned fatal-flag slots | `graphs()`, thread_local | one per member thread; capture mode `ThreadLocal`; released at the thread's end (`release_thread_device`, with its stream) |
| column physics work buffer, RRTMG batch scratch | core/column_work.cpp, ra_rrtmg_common/batch_memory.cpp, thread_local | one per member thread; the column buffer released at the thread's end |
| convtrans_prep's averaging counter (WRF's SAVE `stepave_count`) | `PhysicsSuite::stepave_count_`, one per run (the domains of a nested run share it) | each member's own, 0 at its start (was one per host thread: a sequential member inherited the previous member's count) |
| upload staging buffer | `thread_staging`, thread_local | one per member thread, freed at its end |
| Thompson / P3 device tables | physics_coupling.cpp, a mutex-guarded cache keyed by options and device | one copy per process for the members with the same tables |
| RRTMG / glibc / libmvec device tables | `static __device__ const` | read-only, one per process |
| local-memory (stack) reserve | `reserve_local_memory`, mutex-guarded, the largest frame per device | once per process (cudaLimitStackSize is per context) |
| device work budget share | `set_device_work_share` | N while the members run (as ranks on one device): chunk sizes, never values |
| NetCDF (not thread-safe), host-side readers and writers | input, boundaries, output, restart | one member at a time: the host work between steps runs under the process's NetCDF lock (below) |
| WRF fatal flags | `fatal_code_host[slot]`, the per translation unit `fatal_code_dev[slot]` | one slot per member thread (below) |

The time steps of the members run at the same time; their host work between the steps (`before_step`,
`after_step`, the start, the end) runs one member at a time under the ensemble's lock. In NetCDF builds that lock is
the process's NetCDF lock of the output writer thread (wrf_io/output_queue.hpp, docs/PERF.md IO2): the writer's
NetCDF calls fall between the members' host work while the members step, a member waiting for the writer's queue
lets the lock go as a single run does, and there is no second lock whose order could deadlock. Ensembles therefore
run with `output.write_queue_mb > 0` (the default) as single runs do. Two changes make the
threads independent on the device, neither of which changes a single run's work:

- `dev_sync` in a member thread waits for the member's own stream (`set_thread_stream_sync`), not for the whole
  device, so one member's wait (output, the step's end, WSM6's buffer) does not wait for the others' work. A single
  run, tiles and MPI ranks keep the device-wide synchronisation.
- The graphs' end-marker word is copied to the device on the thread's stream instead of the legacy stream 0
  (a legacy-stream copy while another thread captures its blocking stream would join that capture).

Results do not depend on the chunk sizes of the column physics and RRTMG batches (each column is computed alone;
the budget is divided among ranks or tiles on one device the same way), nor on the number of OpenMP threads.

The fatal flags are per member (core/backend.hpp): member i's thread uses slot i (slot 0 outside an ensemble and in
sequential mode). On the host the CPU kernels' OpenMP threads raise into the launching thread's slot; on a CUDA
device each translation unit's flag has a word per slot, and a member's launches carry the slot as the bytes of
dynamic shared memory they ask for (0, the launches unchanged, outside an ensemble), which `raise_fatal` reads back
(`%dynamic_smem_size`) only when it raises. A member's check reads and clears its own slot on its own stream, with
every transfer checked. A member that stops (another member failed) reads its pending checks at a point where a
raised code is its own error, before the destructors that free device memory; an error already on its way drops
them unread. HIP: a launch carries the slot as a kernel argument instead, which the kernel's threads keep in a word
of shared memory for `raise_fatal`: the same per-member flags as on CUDA.

The output writer thread keeps each member's records apart: a writer error stops the member whose file it was
writing (its message names the file and the member), the writer goes on with the other members' records, and a
member that ends waits for its own queued records only (then frees its device memory), not for the records the other
members queue after them. The queue's cap (`output.write_queue_mb`) is one for the process and is checked again by
every waiting member after it wakes.

Proof: ctest `ensemble` (tests/verify/ensemble_test.py, CPU builds and GPU builds alike): a tiny squall2d as an
ensemble of three members (Kessler; WSM6 with other species; a warmer bubble in shear), in threads and in
sequence, every file of every member (native snapshots, wrfout, wrfrst, CPPWRF's restart files; diag.csv without
its wall-time column) byte-identical to the plain single run of squall2d.ini with the member's overrides as `--set`,
and to `--set ensemble.member=I` for every member (all files, recursively; run.json without the output directory
of its configuration), with the output writer thread on (`write_queue_mb` 1024) and off (0); refused: an unknown
ensemble key, a member index above `members`, two members with one `output.dir` (also as `dir/x/../member1`) or one
`output.wrf_dir`, an unknown mode. ctest `ensemble_cumulus` (data: run_P8_r8): P8 with Grell-Freitas and P8 without
cumulus, both writing restart files every step, sequential and threads, against their single runs (a
convtrans_prep counter kept from the first member changes the second member's restart file). Section 5 lists the real-data and GPU checks.

## 3. The options compared

Numbers below are from docs/PERF.md (Colab G4, RTX PRO 6000 Blackwell, 188 SMs, 96 GB) unless stated.

**How much of a large GPU one member leaves idle.** L has 9.1 times P4's cells (359 x 269 x 44 against 119 x 89 x 44)
and steps an hour in 2.50 s (GPU FP32) and 7.52 s (FP64), against P4's 1.47 s and 2.35 s. So 9.1 times the work
takes 1.7 times (FP32) and 3.2 times (FP64) the time: one P4 uses the G4 at about a fifth (FP32) to a third (FP64)
of the rate it reaches on a large grid. The per-kernel tables say the same from the other side: the column physics
of P4 / D0 run at 5-8 % achieved occupancy (one thread per column, ~10^4 columns on 188 SMs), Thompson is 25-33 % of
a D0 step, and P4 / D0 launch 633-635 kernels per step. The upper bound of the gain for N concurrent P4-sized
members on a G4 is therefore about 5x (FP32) and 3x (FP64) in throughput, the rate of one grid N times as large.

**(1) N processes sharing the GPU.** Possible today without any change: N `cppwrf` processes on one device (a shell
loop or a job array). Nothing in CPPWRF conflicts across processes (each has its own context, stream, tables,
flags). But:
- Without MPS, kernels of different processes do not run at the same time; the GPU time-slices between the
  contexts. P4 / D0 keep the GPU 97-99 % busy with kernels already, so the gain is only the host gaps (a few %).
- With MPS (`nvidia-cuda-mps-control -d`, needs control of the machine; not available on Colab or Kaggle) the
  kernels overlap, but every process still pays its own context, its own local-memory reserve and its own tables:
  P4 FP64 on the G4 peaks at 13.7 GB, 8.0 GB of it the stack reserve (docs/PERF.md, history-off benchmark), so a
  96 GB GPU holds 6 P4 FP64 processes; Thompson's tables are ~380 MB per FP64 process. Each process also sizes its
  work buffers (`device_work_budget`: min(total / 8, free / 4)) without knowing the others (CPPWRF_DEVICE_MEMORY_MIB
  is the manual cap).
- Effort: none (docs, a launcher script). Benefit: small without MPS, memory-bound with it.

**(2) One process, N members, each on its own host thread, stream, graphs and buffers (implemented).** The members'
kernels overlap on the device as MPS would let them, but in one context: one stack reserve (set to the largest
frame of all members), one copy of the read-only device tables, one CUDA context. The machinery exists because the
tiles of P03 run one domain as host threads with their own streams on one device (section 2's table); what was
missing was a driver that runs whole runs instead of patches, the host lock for NetCDF, and the per-thread
synchronisation. Effort: M (about 300 lines and a test). Benefit: the overlap of concurrent kernels (bounded by the
5x / 3x above; streams overlap less well than one large grid: a kernel that fills the GPU still waits for one that
runs), plus N - 1 stack reserves and table copies saved against (1) with MPS. Each member keeps its own launches
(N x 633 per step for P4), its own graphs and its own host work, and the host work of the members is serialised
(output and boundary reads of N members per output time).

**(3) True batching: a member dimension in the kernels.** Every kernel indexes (member, i, j, k); every Field gets
a member stride; the halo fills, lateral boundaries, reductions (the CFL maxima of each member), fatal checks, the
physics drivers' column batches, tiles and I/O learn the dimension. That touches most of the 127 000 lines of
src/; the members must also share every option that changes the kernel sequence (physics members could not be
batched at all, only IC/LBC members). Benefit beyond (2): one launch per kernel for all members (fewer, larger grids;
the full occupancy of the L numbers), and the column physics of N members in one launch. Effort: XL, with the risk of
touching every bit-identical path. Not now: (2) gives most of the benefit for regional domains of P4's size and
larger; (3) pays off for many small members (a few thousand columns each), where launch counts dominate.

**Decision: (2).** It is the one with a large benefit for an M effort, it keeps every single-run path as it is,
and it reuses machinery that is already proved bit-identical (tiles on one device). (1) stays available and is
the right choice for members on different GPUs (one process per GPU, or `CUDA_VISIBLE_DEVICES` per member).

## 4. Memory per member

Measured on the RTX 2060 (6 GB, 30 SMs, CUDA FP32 identity build, P4 for 30 minutes, nvidia-smi's peak every 0.2 s):

| Run | Peak device memory (MiB) |
|---|---|
| P4 alone | 2138 |
| P4 without cumulus, radt 2 alone | 1980 |
| both as an ensemble of 2 | 3080 |

The second member adds 940-1100 MiB, about 2.2 KB per cell of the 119 x 89 x 44 grid: its fields, its column work
buffer and its RRTMG batch scratch (budget / N). It does not add a CUDA context (~300-500 MiB), a stack reserve
(434 MiB here, the largest frame x 30720 resident threads) or the device tables; under (1) every process pays those
again. On the G4 the stack reserve is the large part (288768 resident threads: P4 FP64 8.0 GB of its 13.7 GB peak),
so option (2) saves 8 GB per member there against N processes.

Estimates for one more member (fields scale with the precision, the scratch with the GPU's budget):

| Case | FP32 per member | FP64 per member | Shared once per process |
|---|---|---|---|
| P4 (WSM6, KF, RRTMG, YSU, Noah; 6 moist species) | ~1.0 GB (measured) | ~2.0 GB | context, stack reserve, RRTMG tables |
| D0 (Thompson, Tiedtke, RRTMG radt 15, MYJ, Noah, GWDO; 8 moist species incl. 2 numbers) | ~1.1 GB | ~2.2 GB | as P4, plus Thompson's tables (~190 MB FP32, ~380 MB FP64) |

D0 is on the same grid as P4 with two more 3D moist arrays (QNI, QNR, ~2 x 466k cells in each of the state,
tendency and boundary arrays), so its member footprint is P4's plus a few percent; its Thompson tables are shared by
all members with the same Thompson options. An 80-96 GB GPU therefore holds tens of P4 / D0 members in FP64 under (2)
(memory is no longer the limit; the compute is), against 6 under (1) with MPS.

## 5. Measurements

Local checks on the RTX 2060 mobile (sm_75, nvcc 12.4, CUDA FP32 identity build, one run each):

- ctest `ensemble` (section 2) passes on the GPU build: three squall2d members, threads and sequential, every file
  byte-identical to the single runs. It passes on the CPU FP64 build too.
- P4 (real data, WSM6, KF, RRTMG, Noah, YSU), 30 minutes, history every 10 minutes (wrfout and snapshots): an
  ensemble of 2 members (P4 as it is; P4 with `cu_physics = 0`, `radt = 2`) against the two single runs: all 9
  output files of each member byte-identical (`cmp`). CPU FP64 build, 3 steps, history every step: the same, 9
  files per member identical.

Wall time (P4 FP32, 30 minutes, whole process including start-up and output):

| Run | Wall (s) | Stepping (s) |
|---|---|---|
| P4 alone | 5.11 | 3.02 |
| P4 without cumulus alone | 4.04 | 1.95 |
| the two in sequence | 9.15 | 4.97 |
| ensemble of the two (threads) | 8.33 | 4.14 / 6.47 (the members overlap) |

1.10x on this GPU. A 30-SM laptop GPU is much closer to full with one P4 than the G4 with its 188 SMs (P4's
~10^4 columns are ~330 warps: ~11 per SM here, ~2 on the G4), so there is little idle capacity for the second member
here, and the members' host work (start-up, output) runs one member at a time. The gain is expected where one member leaves most of the GPU idle (section 3: a fifth to a
third used on the G4); that measurement is the first item of section 6. CPU FP64, P4 3 steps (12 cores): the
ensemble of the two 11.5 s against 8.5 + 8.1 s in sequence (each member with half the OpenMP threads).

## 6. What is left

- Per-member fatal flags (a flag word per stream or per member), so a member's WRF fatal stop names the right member
  and the other members could run on.
- Nested members (`run_nested` per member thread: its host phases under the ensemble's lock), tiles inside a member,
  and members over several GPUs (`ensemble.devices = d0,d1,...`, member i on `d[i % n]`, the same code).
- The host work of the members in parallel where it is not NetCDF (diagnostics, the precompression of output
  chunks), and a member's output written while the other members step (it is already: only the lock holder's host
  work waits).
- A G4 measurement: P4 and D0 FP32 / FP64 with N = 1, 2, 4, 8 members against N sequential runs (throughput,
  peak memory), and the same under MPS with N processes, to place (2) against the 5x / 3x bound.
- (3) for many small members, if the G4 numbers show the launch count as the limit.
