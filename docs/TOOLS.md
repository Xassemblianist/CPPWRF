# CPPWRF user tools

Besides the drop-ins `cppwrf-wrf`, `cppwrf-realexe` and `cppwrf-check` (docs/QUICKSTART.md,
docs/MIGRATING_FROM_WRF.md), an installation has tools for checking results, planning runs and using the output. All are
part of the `cppwrf` Python package (docs/PYTHON.md) and need only `numpy` and `netCDF4`. `--help` lists every
option. A bash completion for all the tools is installed in `share/bash-completion/completions`; from a source tree,
`source tools/completion/cppwrf.bash`.

## cppwrf-compare: two sets of output files

```bash
cppwrf-compare WRF_RUN_DIR CPPWRF_RUN_DIR             # every stream and domain found in both
cppwrf-compare a/wrfout_d01_2026-10-12_00:00:00 b/wrfout_d01_2026-10-12_00:00:00 -v T,U,QVAPOR
```

It uses the verification suite's comparison code (`tests/verify/exact.py`):

- frames are paired by their `Times`;
- values are identical when their bits are, except that NaN equals NaN (-0 and +0 differ);
- the global attributes and dimensions are compared too (`--no-metadata` skips that).

For each variable that differs it prints the number of frames and points that differ, max |a - b|, the RMSE over
all points, the largest distance in units in the last place, and the first differing point with both values.
`--json` writes all of it. The exit status is 0 if everything is identical, 1 if something differs, and 2 if a
stream has no common frame or nothing can be compared.

## cppwrf-bench: this machine's speed

```bash
cppwrf-bench                         # every mode installed
cppwrf-bench --modes gpu-fp32 --size 200x200x40 --json
```

It runs a built-in case through `cppwrf-wrf` in each mode: WRF's em_quarter_ss supercell (dynamics and Kessler
microphysics; no input files), 100 x 100 x 40 cells and 20 steps of 12 s by default, then the same on a quarter of
the horizontal grid. It prints, per mode:

- ns per cell and step: the model's stepping time over cells x steps;
- simulated time per wall time, for the stepping alone and for the whole run;
- peak memory: the device's memory in use (GPU modes, from `CPPWRF_MEMINFO`) or the process's peak RSS (CPU modes);
- the ratio to docs/PERF.md's dynamics-only case R1 on the reference machine. R1 is a real-data case, so the ratio
  is indicative.

The results are saved in `~/.cache/cppwrf/bench.json` (`--save`, `--no-save`) for `cppwrf-fit`. Run it on an idle
machine with the thread count you will use (`--threads`, or `OMP_NUM_THREADS`). Small grids cost more per cell, so
benchmark at a size near your runs' if they are much larger than the default.

## cppwrf-fit: will a run fit, and how long will it take?

```bash
cppwrf-fit namelist.input                            # every mode cppwrf-bench measured
cppwrf-fit --grid 359x269x44 --steps 60 --dt 60 --physics full --radt 15
```

From the namelist it takes each domain's cells, its time step (with `parent_time_step_ratio`), the run length, the
physics options and `radt`. With the bench results it estimates, per mode:

- **memory:** the bench's memory per cell times the cells of all domains, plus the bench's fixed part. With full
  physics (any radiation, PBL, surface or cumulus scheme, or a `physics_suite`), the memory per cell is scaled by
  docs/PERF.md's full-physics over dynamics-only ratio, and on the GPU the stack reserve by its ratio as well;
- **time:** the bench's ns per cell and step times each domain's cells and steps, times the cost of the physics
  relative to the dynamics. That cost is docs/PERF.md's: the P4 case calls the radiation every step, P4R15 every
  15th; other radt values are interpolated by the fraction of steps that call it. The bench's start-up time is
  added;
- **verdict:** "fits" below 85 % of the GPU's memory (GPU modes) or of the host's available memory (CPU modes),
  "tight" below 100 %, else "does not fit". The exit status is 0 if any mode fits or is tight, 1 if none does,
  2 for an error.

History output, reading the input files and restart files are not in the estimate.

### Accuracy

Measured on an Intel i7-10750H (4 threads), CPU FP64: `cppwrf-bench` at its default size, then em_quarter_ss runs of
four other sizes against `cppwrf-fit`'s estimate for them:

| Grid | Steps | Time: estimate / measured | Peak RSS: estimate / measured |
|---|---|---|---|
| 60 x 60 x 30 | 40 | 6.8 s / 6.6 s (+2 %) | 246 / 288 MiB (-15 %) |
| 150 x 150 x 40 | 20 | 26.8 s / 27.4 s (-2 %) | 1597 / 1589 MiB (+0 %) |
| 200 x 200 x 40 | 15 | 35.6 s / 35.5 s (+0 %) | 2790 / 2507 MiB (+11 %) |
| 250 x 250 x 50 | 10 | 46.3 s / 52.2 s (-11 %) | 5391 / 5283 MiB (+2 %) |

So for the dynamics and microphysics the time is within about 10 % and the memory within about 15 %, the largest
errors on the smallest and largest grids. These runs check the scaling with the grid only. The physics factors come
from docs/PERF.md's measurements of one physics suite (KF, WSM6, YSU, Noah, RRTMG) on the reference machine, and
other schemes cost differently. With physics the estimate rests on those factors and has not been checked
against runs on another machine, so take it as a guide. On the
CPU the physics memory factor is the GPU FP64 one (not measured on the CPU). The GPU modes use the same method but
have not been measured against estimates yet.

## cppwrf-post: forecaster fields as NetCDF and GRIB2

```bash
cppwrf-post run/wrfout_d01_* -o post --format both -j 4
```

Surface, pressure-level and height fields (2 m temperature and humidity, earth-relative wind, MSLP, precipitation
and its type, clouds, CAPE / CIN, helicities, reflectivity, ...) computed with wrf-python's methods, one CF-1.8
NetCDF and / or GRIB2 file per output time; a post.yaml / post.ini chooses fields, levels, units and packing.
docs/POST.md.

## cppwrf-verify-obs: a run against surface observations

```bash
cppwrf-verify-obs run/wrfout_d01_* -o verify
cppwrf-verify-obs gpu/wrfout_d01_* --compare cpu/wrfout_d01_*
```

Downloads (and caches) the METARs (IEM) or ISD reports of the stations in the domain, or reads a CSV file; writes
bias, MAE, RMSE, correlation, wind and precipitation scores per lead time and station, plots and a summary.
docs/VERIFY_OBS.md.
