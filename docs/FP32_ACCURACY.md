# CPPWRF GPU FP32 accuracy

CPPWRF's CPU FP64 and GPU FP64 builds are bit-identical to WRF v4.8.0 real*8. The GPU FP32 build, the fast mode on
GPUs with slow FP64 (RTX, L4, the RTX PRO 6000 Blackwell of a Colab G4), cannot be bit-identical to anything. The fair
question is whether it is as trustworthy as the WRF most people run, stock WRF real*4:

> Is the difference between CPPWRF GPU FP32 and WRF real*8 not larger than the difference between WRF real*4 and
> WRF real*8?

The measure is the ratio, per case, variable and output time,

    R = metric(CPPWRF GPU FP32 vs WRF real*8) / metric(WRF real*4 vs WRF real*8)

R <= 1: CPPWRF FP32 is at least as close to the double-precision answer as WRF's own single-precision build.

## Status

- **Measured (1 h, six domains, eight fields):** the benchmark of docs/PERF.md (tools/bench/colab_bench.sh, Colab G4,
  CPPWRF b30ec5b) compared the relative L2 difference at 1 h of U, V, W, T, QVAPOR, PH, MU and PSFC. R is 0.35 to 1.08
  (table below): CPPWRF FP32 is as close to WRF real*8 as WRF real*4 is, or closer, except in the two-way nest's d02,
  where T, QVAPOR and PH are 1 to 8 % further.
- **Not yet measured:** the full study of this page (tools/bench/colab_fp32.sh: twelve configurations, the surface
  fields, rain and its categories, the hydrometeors, 24 h and 72 h growth). Its tooling is in the repository; the
  first attempts (2026-11-09) found no Colab runtime (G4 and L4 "Service Unavailable" for two hours), so its tables
  and figures are still to come. The section "Running the study" says how to make them.

## The 1 h measurement (docs/PERF.md's benchmark)

One Colab G4 runtime (AMD EPYC 9B45, RTX PRO 6000 Blackwell). WRF v4.8.0 real*4 and real*8 built from the release
source with WRF's configure for GNU (gfortran 14.2, MPICH), each run at its fastest MPI layout (24 ranks, or 24 x 2 /
12 x 4 ranks x threads); CPPWRF's CUDA FP32 release build; all from the same real*8 `wrfinput` / `wrfbdy`; the real*4
runs with WRF's single-precision RRTMG tables. Relative L2 difference ||a - b|| / ||b|| over the whole domain at 1 h.
Each cell: WRF real*4 vs WRF real*8 / CPPWRF FP32 vs WRF real*8 (R).

| Case | U | V | W | T | QVAPOR | PH | MU | PSFC |
|---|---|---|---|---|---|---|---|---|
| R1 (dynamics only) | 1.9e-04 / 8.7e-05 (0.46) | 1.5e-04 / 7.2e-05 (0.49) | 3.8e-03 / 3.7e-03 (0.98) | 3.8e-06 / 2.9e-06 (0.75) | 1.9e-05 / 7.7e-06 (0.41) | 1.3e-05 / 1.2e-05 (0.93) | 2.7e-05 / 2.5e-05 (0.93) | 2.0e-08 / 2.0e-08 (1.00) |
| P4 (KF, WSM6, YSU, Noah, RRTMG radt 1) | 6.5e-04 / 6.2e-04 (0.96) | 6.5e-04 / 6.3e-04 (0.97) | 1.7e-02 / 1.7e-02 (0.96) | 3.7e-05 / 3.5e-05 (0.96) | 5.5e-04 / 5.2e-04 (0.95) | 3.0e-05 / 2.9e-05 (0.98) | 1.8e-04 / 1.7e-04 (0.96) | 2.1e-06 / 1.9e-06 (0.94) |
| P4R15 (P4, radt 15) | 7.7e-04 / 4.6e-04 (0.59) | 7.6e-04 / 4.5e-04 (0.60) | 1.9e-02 / 1.2e-02 (0.60) | 4.9e-05 / 3.9e-05 (0.80) | 5.6e-04 / 4.2e-04 (0.76) | 3.3e-05 / 1.9e-05 (0.56) | 1.9e-04 / 7.8e-05 (0.41) | 2.0e-06 / 1.1e-06 (0.52) |
| N3 d01 (two-way nest, parent) | 6.6e-04 / 3.1e-04 (0.47) | 6.6e-04 / 3.2e-04 (0.48) | 1.7e-02 / 9.2e-03 (0.54) | 4.1e-05 / 2.7e-05 (0.65) | 5.4e-04 / 2.9e-04 (0.54) | 3.1e-05 / 1.4e-05 (0.45) | 1.9e-04 / 6.6e-05 (0.35) | 2.0e-06 / 9.6e-07 (0.47) |
| N3 d02 (5 km nest) | 3.6e-04 / 2.8e-04 (0.77) | 3.1e-04 / 2.2e-04 (0.70) | 5.9e-03 / 5.3e-03 (0.90) | 1.1e-05 / 1.2e-05 (**1.08**) | 9.5e-05 / 9.6e-05 (**1.01**) | 9.4e-06 / 9.8e-06 (**1.04**) | 3.2e-05 / 2.9e-05 (0.93) | 5.1e-07 / 4.6e-07 (0.90) |
| L (359 x 269 x 44, P4 physics, radt 15) | 2.9e-04 / 2.7e-04 (0.93) | 3.8e-04 / 4.0e-04 (**1.04**) | 1.1e-02 / 1.1e-02 (0.96) | 3.5e-05 / 3.4e-05 (0.97) | 2.5e-04 / 2.1e-04 (0.83) | 1.3e-05 / 1.3e-05 (0.98) | 4.5e-05 / 4.4e-05 (0.97) | 7.5e-07 / 7.0e-07 (0.93) |

What the numbers say, and what they do not:
- With physics, both differences are of the same size after one hour (R 0.9 to 1.0 in P4 and L): the physics'
  thresholds (convective triggers, saturation, the radiation every step) turn last-bit differences of any origin
  into differences of this size. Where they are smaller (R1 without physics, radiation every 15 minutes, N3's parent)
  CPPWRF FP32 is clearly closer to real*8 than WRF real*4.
- The largest R, 1.08 (T of the nest), is 8 %: within the spread of single-precision runs. WRF's own real*8 differs
  from itself by the same order of magnitude when its MPI layout changes (docs/PERF.md, "Agreement").
- The real*8 reference here is WRF real*8 at its fastest MPI layout on an AMD CPU, not the suite's one-process Intel
  reference; numerator and denominator use the same run. The full study uses CPPWRF GPU FP64 (= one-process WRF
  real*8) as the reference.
- One hour, eight 3-D / 2-D state fields. Rain, the surface fields and the growth over a day are the full study's.

## The full study

### Method

**Models.** One Colab runtime (tools/bench/colab_fp32.sh):
- WRF v4.8.0 from the official release source, WRF's own `configure` for GNU (dmpar, option 34) in the Debian 13
  chroot of the suite (gfortran 14.2, MPICH): real*4 (no `-r8`: the usual production build) and real*8
  (`configure -r8`). The real*4 runs read WRF's single-precision RRTMG tables and compute their own Thompson tables.
- CPPWRF's release builds of README.md: CUDA FP32 (`-DCPPWRF_DOUBLE=OFF`: CUDA's math library, FMA contraction) and
  CUDA FP64.
- The real*8 reference of every comparison is the CPPWRF GPU FP64 run. It is bit-identical to WRF real*8 run in one
  process on Intel (the suite's references); the same session re-runs the suite's tolerance-0 GPU FP64 cases of the
  studied configurations to show it. Using it rather than a new WRF real*8 run removes two effects that have nothing
  to do with precision: WRF's dependence on its MPI layout, and on the CPU's libmvec (an AMD CPU gives other last
  bits of `log` than the Intel CPUs that made the references).
- Every model runs the same namelist from the same real*8 `wrfinput` / `wrfbdy` files (WRF real*4 converts them on
  reading), history every 10 minutes (1 h runs) or every hour (long runs), restart off.

**Cases** (tools/bench/fp32_run.py, CASES):

| Case | Configuration | Length | WRF real*4 runs |
|---|---|---|---|
| P4 | 15 km, 119 x 89 x 44, KF + WSM6 + YSU + Noah + RRTMG (radt 1) | 1 h | one process; also 8 MPI ranks |
| P6 | P4 domain, Thompson | 1 h | one process |
| P9 | P4 domain, MYJ + Eta | 1 h | one process |
| N3 | P4 with a two-way nest (5 km, 120 x 90) | 1 h | one process; also 12 MPI ranks |
| V1 | tropics (Bay of Bengal) | 1 h | one process |
| T1 | Arctic | 1 h | one process |
| SKV | V1 with `sst_skin` = 1, `tmn_update` = 1 | 1 h | one process |
| MORR | P4 with Morrison (mp 10, hail) and KF | 1 h | one process |
| ACM7 | P4 with ACM2 over the Pleim-Xiu surface layer | 1 h | one process |
| L | 359 x 269 x 44 at 15 km (WPS and real.exe on the runtime from GFS), P4 physics, radt 15 | 1 h | one process |
| P4L | P4 from the real.exe inputs of the suite's 72 h reference | 72 h | 8 MPI ranks; first 24 h also in one process |
| N3L | N3 from the inputs of the suite's 24 h reference | 24 h | 12 MPI ranks |

The long real*4 runs use MPI ranks: one process would need about 2.5 h for each. WRF's result depends on its process
layout; P4 and N3 in both layouts for 1 h, and P4L's first 24 h in one process, show how much.

**Pairs** (columns of the tables):
- `r4-r8`: WRF real*4 (one process; long runs: MPI) against WRF real*8 (= CPPWRF FP64).
- `fp32-r8`: CPPWRF GPU FP32 against WRF real*8.
- `fp32-r4`: CPPWRF GPU FP32 against WRF real*4.
- `r4m-r8`, `r4s-r8`: WRF real*4 in the other layout (MPI for P4 and N3, one process for P4L).
- `r8n-r8` (long runs, on runtimes with 32 or more vCPUs): WRF real*8 with MPI ranks on the runtime's CPU against
  the one-process Intel answer, a perturbation of the last bits in double precision: how fast any perturbation grows.
- `fp64-ref`: CPPWRF GPU FP64 against the suite's WRF real*8 reference files (must be 0).

**Statistics** (tools/verify/fp32_accuracy.py, numpy and netCDF4 only, any two sets of wrfout files): per output
time and variable the max |d|, the RMSE, the relative L2 difference and the bias of T2, Q2, U10, V10, PSFC, TSK,
SWDOWN, HFX, LH, PBLH, the accumulated rain RAINNC + RAINC and the hourly rain, the 3-D T, QVAPOR, U, V, W, PH, the
hydrometeor mixing ratios and number concentrations; for rain the fraction of points whose category differs (hourly
rain >= 0.1, 1 and 5 mm, accumulated rain >= 1 and 10 mm). `report` turns the statistics of several pairs into the
tables (R per variable, the growth table at 1, 3, 6, 12, 24, 48, 72 h) and SVG figures of the growth (log scale).

```bash
tools/verify/fp32_accuracy.py compare FP32_RUN_DIR R8_RUN_DIR --domain d01 --case P4 --pair fp32-r8 --out p4.jsonl
tools/verify/fp32_accuracy.py compare WRF_R4_DIR R8_RUN_DIR --domain d01 --case P4 --pair r4-r8 --out p4.jsonl
tools/verify/fp32_accuracy.py report p4.jsonl --metric rmse --svg-dir docs/img --svg-rel img
```

### Running the study

```bash
CPPWRF_GCP_BUCKET=<bucket> tools/bench/colab_fp32.sh            # a Colab G4; --gpu L4 or A100 if no G4 is free
```

One session: the WRF builds (about 13 minutes), WPS for L, the WRF runs several at a time (the long real*4 runs set
the length: about 1 h on a G4), the GPU runs, the suite's tolerance-0 GPU FP64 cases of the studied configurations,
the FP32 suite cases (staged with tools/colab/stage_data.py, as a GPU proof stages them) and the statistics, all on
the runtime. Only the statistics, the three reports (`report_rmse.md`, `report_max_abs.md`, `report_rel_l2.md`) and
the SVG figures come back. Expected cost on a G4: about 1.5 h, 15 compute units.

### Results

To be filled from the first run of tools/bench/colab_fp32.sh: the summary (median and largest R per case), the 1 h
tables per case, the rain categories, the growth over 24 h (N3L) and 72 h (P4L) with the figures under docs/img, and
the verdict per case with the likely cause wherever R > 1.

## When to use FP32 and when FP64

- FP64 (CPU or GPU) when the answer must be WRF's: verification against WRF real*8, studies that compare against
  WRF runs, restarts that must continue a WRF run exactly.
- GPU FP32 for production forecasts and ensembles on GPUs with slow FP64: on a G4 it steps 1.7 to 4.3 times faster
  than GPU FP64 (docs/PERF.md). Its distance from WRF real*8 is that of WRF real*4, the build most WRF users run.
