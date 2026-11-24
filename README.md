# CPPWRF

Version 1.5.0 ([release notes](docs/RELEASE_NOTES.md)). The Zenodo concept DOI for all versions is
[doi:10.5281/zenodo.24442751](https://doi.org/10.5281/zenodo.24442751). The 1.4.0 archive is
[doi:10.5281/zenodo.24488201](https://doi.org/10.5281/zenodo.24488201).

CPPWRF is a C++20 / CUDA port of the WRF-ARW v4.8.0 model: the dynamical core, the physics, static nesting,
`real.exe` and `ideal.exe`. One source tree builds for the CPU (OpenMP) and for NVIDIA GPUs (CUDA), in FP64
or FP32. It reads WRF's `wrfinput` / `wrfbdy` files and writes WRF's `wrfout` and `wrfrst` files.

The port follows WRF statement by statement, and every result is checked against WRF itself:

- **CPU, FP64: bit-identical to WRF v4.8.0 real*8.** Every compared value of every variable and output time
  is equal at tolerance 0, for the real-data cases, the physics suites, nests, restarts and WRF's idealized cases.
- **GPU, FP64: bit-identical to WRF too.** For v1.5.0, all 423 CUDA FP64 checks matched WRF real*8 at tolerance 0
  on the Colab G4 at `40628eb` (RTX PRO 6000 Blackwell, sm_120, CUDA 13.0). The CUDA FP32 identity build matched
  WRF real*4 at tolerance 0 in all 44 cases. Earlier G4 runs reported 423 passes for v1.4.0 and v1.3.0, 422 for v1.2.3,
  356 for v1.2.2 and 338 for v1.2.0 and v1.2.1. The v1.1.0 G4 total of 338 included 319 cases, 18 restart checks
  and the legacy `straka_wrf_cuda` RMS check against WRF real*4 at 1e-4 K. The WRF real*8 comparisons passed at tolerance 0.
  Earlier subsets also ran on L4 (sm_89) and A100 (sm_80).
- **CPU and GPU, FP32: bit-identical to WRF v4.8.0 real*4** (the default gfortran real*4 build) on 24 configurations,
  among them P1-P4, Thompson, MYNN, WDM6, Morrison, Noah-MP and the nests N1 / N3. The CUDA FP32 build defaults to
  this identity mode (`CPPWRF_CUDA_FP32_IDENTITY=ON`), and the result does not depend on the host's libm. See
  [docs/FP32_IDENTITY.md](docs/FP32_IDENTITY.md).
- **GPU, FP32 (fast mode):** opt-in (`-DCPPWRF_CUDA_FP32_IDENTITY=OFF`, FMA contraction and CUDA's math library).
  In the [v1.3.0 paper benchmark](#performance), the identity build takes 2.1 to 18.0 % longer in steady stepping
  than the fast build (identity time / fast time - 1). The fast build is compared with WRF by relative L2 error.
  The v1.5.0 G4 run passed 43 of 44 checks. The remaining case passed on an RTX 2060 with the harness fix at `091ffae`
  ([release notes](docs/RELEASE_NOTES.md)).

The details are in [docs/STATUS.md](docs/STATUS.md).

## Install, quick start, migrating from WRF

- **Install:** [docs/INSTALL.md](docs/INSTALL.md) (builds, `cmake --install`, release tarballs, containers, the
  self-test). Turkish: [docs/tr/KURULUM.md](docs/tr/KURULUM.md).
- **Quick start:** [docs/QUICKSTART.md](docs/QUICKSTART.md) goes from GFS through WPS, `cppwrf-realexe` and
  `cppwrf-wrf` to a map. Turkish: [docs/tr/HIZLI_BASLANGIC.md](docs/tr/HIZLI_BASLANGIC.md).
- **Migrating from WRF:** [docs/MIGRATING_FROM_WRF.md](docs/MIGRATING_FROM_WRF.md) and [docs/FAQ.md](docs/FAQ.md).
  In a WRF run directory, `cppwrf-wrf` replaces `wrf.exe` and `cppwrf-realexe` replaces `real.exe`: the same file
  names, `rsl` logs and `SUCCESS COMPLETE WRF`. `cppwrf-check namelist.input` says which options are ported.
- **Python:** [docs/PYTHON.md](docs/PYTHON.md) covers `pip install ./python`: namelists, runs and wrfout files with
  diagnostics.
- **Tools:** [docs/TOOLS.md](docs/TOOLS.md): `cppwrf-compare` compares two runs' output, `cppwrf-bench` measures
  this machine, and `cppwrf-fit namelist.input` estimates a run's memory and time.
- **Post-processing:** [docs/POST.md](docs/POST.md): `cppwrf-post` writes forecaster fields (wrf-python's methods)
  as CF NetCDF and GRIB2. Turkish: [docs/tr/SON_ISLEM.md](docs/tr/SON_ISLEM.md).
- **Verification:** [docs/VERIFY_OBS.md](docs/VERIFY_OBS.md): `cppwrf-verify-obs` scores a run against METAR, ISD
  or your own station observations. Turkish: [docs/tr/GOZLEM_DOGRULAMA.md](docs/tr/GOZLEM_DOGRULAMA.md).

## What is ported

| Area | WRF options |
|---|---|
| Dynamics | RK3 split-explicit, adaptive time step, hybrid coordinate, map projections (Lambert, Mercator, polar), terrain; specified, open, periodic and symmetric boundaries; damping, 5th/6th-order, monotonic and WENO advection, positive-definite moisture |
| Diffusion | diff_opt 1 and 2; km_opt 1-4 (constant K, Smagorinsky, 1.5-order TKE); 6th-order diffusion |
| Microphysics | Kessler, WSM3, WSM5, WSM6, WDM5, WDM6, Thompson, aerosol-aware Thompson (mp 28), Morrison two-moment, P3 (one ice category), NSSL 2-moment with hail (mp 18 / 17); radar reflectivity |
| Cumulus | Kain-Fritsch, multi-scale Kain-Fritsch, Betts-Miller-Janjic, Grell-Freitas-Li, Grell-3D ensemble, Tiedtke, New Tiedtke |
| PBL and surface layer | YSU, MYJ, ACM2, MYNN-EDMF with most of its options, BouLac, QNSE with the EDKF mass flux; revised MM5, Eta, Pleim-Xiu, MYNN and QNSE surface layers; gravity wave drag |
| Land surface | 5-layer slab, Noah, Noah-MP (its options without extra input fields), RUC; single-layer urban canopy model; fractional sea ice, SST update |
| Radiation | RRTMG LW and SW, RRTM LW, Dudhia SW, cumulus clouds in radiation, McICA |
| Nesting and nudging | static one-way and two-way nests with feedback and smoothing; grid (analysis) nudging |
| Pre-processing | `cppwrf_real` (real.exe from WPS met_em files) and WRF's ideal.exe cases |
| Diagnostics and I/O | buckets, precipitation windows, output_diagnostics (auxhist3), WRF NetCDF wrfout / wrfrst, iofields |

WRF's `namelist.input` works as is: `tools/wrf2cppwrf.py` translates it, including `physics_suite`. An option
without a port is reported as an error and never silently ignored. MPI runs give WRF's bits in every tested layout
on one machine: up to 48 CPU ranks, and CUDA FP64 on two GPUs; runs across several nodes use the same code and have
not been tested yet ([docs/MPI.md](docs/MPI.md)). Moving nests, BEP urban and lake models, and other missing schemes
are listed in [docs/STATUS.md](docs/STATUS.md#not-yet-implemented) and in
[docs/USER_GUIDE.md](docs/USER_GUIDE.md#rejected-options-and-why).

## How it is verified

- **Column and kernel tests** (`tests/phys`, `tests/dyn`, `tests/nest`): each scheme runs on the same inputs in
  C++ and in WRF's own compiled Fortran objects, and the outputs must be bit-identical.
- **Model runs and restart checks** (`tests/verify/wrf_suite.py`): CPU CI runs these checks on every push. For
  v1.5.0 at `ac13ebe`, all 533 suite checks passed against WRF at tolerance 0 (489 FP64 and 44 FP32). CPU CI also
  passed 42 WRF output file checks, 14 ideal file checks, `ctest -L diff` 32/32 and CTest 44/44. For v1.3.0 at
  `318159a`, the suite reported 483 FP64 passes (464 cases, 18 restart checks and the legacy `straka_wrf` RMS check)
  and 44 FP32 passes. The `ideal_grav2d_x` cases verify the Straka density current against WRF real*8 at tolerance 0
  on CPU and CUDA. Another 154 CPU FP64 cases run with `--slow`. The CUDA FP64 checks and the CUDA FP32 identity
  cases run once per release on a GPU. The cases cover real-data domains in the mid-latitudes, the tropics (Bay of
  Bengal), the Arctic (Svalbard) and the Alps at 3 km. They include 24 h and 72 h forecasts and WRF's default CONUS
  namelist. CPPWRF's wrfout files are compared with WRF's in every variable and output time.
- **WRF output files:** wrfout and wrfrst match WRF's files in values, attributes and metadata. Runs resumed
  from WRF's restart files match too.

To be bit-identical, the port must repeat WRF's arithmetic exactly. That means the order of every sum, the
constants gfortran folds, the halo schedule, and the glibc vector math (libmvec) that gfortran calls. On the
GPU, CPPWRF runs a device port of the same glibc functions and turns FMA contraction off. The CPU result is
the same on Intel and AMD processors (`CPPWRF_PORTABLE_LIBMVEC`). On x86-64 the FP32 builds carry glibc 2.41's
float functions in the executable, so their result does not depend on the host's glibc version.

## Performance

The v1.3.0 paper benchmark (CPPWRF commit `9f9bb77`) ran WRF v4.8.0 and CPPWRF on a Google Colab G4 with an AMD
EPYC 9B45 (24 cores, 48 hardware threads) and an NVIDIA RTX PRO 6000 Blackwell (97887 MiB). WRF used its stock GNU
real*4 and real*8 builds, with the fastest of eight screened MPI or MPI + OpenMP layouts for each case and precision.
The table gives steady stepping seconds per simulated hour for the 6 to 24 h runs with history off. Times and
speed-ups use the unrounded `step_s` medians in the benchmark's `runs.csv` and `summary.json`.

| Case | WRF real*4, best layout | WRF real*8, best layout | CPPWRF GPU FP32 (fast) | CPPWRF GPU FP64 |
|---|---|---|---|---|
| C1: P4, 119 x 89 x 44 at 15 km, radt 15 min | 2.90 | 4.36 | 0.46 (6.33x) | 1.54 (2.83x) |
| C2: L, 359 x 269 x 44 at 15 km, radt 15 min | 28.60 | 50.38 | 2.78 (10.28x) | 9.16 (5.50x) |
| C3: MORR, P4 with Morrison microphysics and radt 1 min | 11.77 | 14.43 | 1.61 (7.31x) | 5.00 (2.89x) |
| C4: N3, P4 with a two-way 5 km nest | 30.00 | 39.63 | 3.32 (9.05x) | 9.78 (4.05x) |

The speed-ups in brackets are against WRF real*4 for FP32 and against WRF real*8 for FP64. Across the four cases,
steady stepping speed-ups are 6.33x to 10.28x for fast GPU FP32, 5.36x to 10.07x for identity GPU FP32 and 2.83x
to 5.50x for GPU FP64. The speed-up is larger on the bigger P4 domain (C2 vs C1). With history off, the median
`wall` times give speed-ups over WRF real*4 of 5.92x to 8.73x for fast GPU FP32 and 5.06x to 8.32x for identity GPU FP32.

The earlier 1 h benchmark at `c9c95c5` ([docs/PERF.md](docs/PERF.md)) measured the following case, before RRTMG
computed in single precision in CUDA FP32 builds.

| Case | WRF real*4, best layout | WRF real*8, best layout | CPPWRF GPU FP32 (fast) | CPPWRF GPU FP64 |
|---|---|---|---|---|
| P4: 119 x 89 x 44, 15 km, full physics, radt 1 min | 7.46 | 10.71 | 1.47 (5.1x) | 2.35 (4.6x) |

In the 1 h pre-1.1 benchmark of docs/PERF.md, GPU FP32 was 1.5x to 3.5x faster than WRF real*4 in wall time with history off.
CPPWRF compresses the NetCDF-4 chunks of its history and restart files on host threads, and the files keep every byte:
with a frame every 10 minutes and a restart file, L's hour takes 13 s in GPU FP32 (58 s with serial compression) and
33 s in FP64 (123 s).

In the earlier `c9c95c5` runs, the GPU FP64 results are bit-identical to the CPU FP64 results. The CPU FP64 results are
bit-identical to WRF real*8 run in one process, which is how the references are made: WRF's own result changes
with its MPI layout. In these runs, the fast GPU FP32 build differed from WRF real*4 by about as much as WRF real*4
differs from WRF real*8 (the default GPU FP32 identity build matches WRF real*4 on the configurations of
docs/FP32_IDENTITY.md). In the v1.3.0 paper benchmark, the identity build takes 2.1 to 18.0 % longer in steady stepping
than the fast build (identity time / fast time - 1).

## Build

```bash
# CPU build, FP64 (the configuration that is bit-identical to WRF real*8)
cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON
cmake --build build-cpu -j "$(nproc)"

# GPU build, FP32 (default: identity mode, bit-identical to CPU FP32 and to WRF real*4 on docs/FP32_IDENTITY.md)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF
cmake --build build -j "$(nproc)"

# GPU build, FP64 (bit-identical to WRF real*8 and to the CPU build)
cmake -S . -B build-gpu-dp -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON
cmake --build build-gpu-dp -j "$(nproc)"

# CPU build, FP32 (bit-identical to WRF real*4 on the cases of docs/FP32_IDENTITY.md)
cmake -S . -B build-cpu-sp -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF
cmake --build build-cpu-sp -j "$(nproc)"

# GPU build, FP32 fast mode (opt-in, with FMA contraction and CUDA math)
# In the v1.3.0 paper benchmark, identity takes 2.1 to 18.0 % longer in steady stepping (identity time / fast time - 1).
cmake -S . -B build-gpu-sp-fast -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF \
  -DCPPWRF_CUDA_FP32_IDENTITY=OFF
cmake --build build-gpu-sp-fast -j "$(nproc)"
```

You need a C++20 compiler, CMake >= 3.24, OpenMP and NetCDF-C; the CUDA toolkit >= 12.1 and a GPU of compute
capability 7.0 or later for the GPU builds; Python 3
with `netCDF4` and `numpy` for the tools. `ctest --test-dir build-cpu` runs the unit tests.

## Run

From a directory prepared for WRF's wrf.exe (`namelist.input`, `wrfinput_d0N`, `wrfbdy_d01` and the tables of
WRF's `run/` directory), with `CPPWRF` the source tree:

```bash
python3 $CPPWRF/tools/wrf2cppwrf.py namelist.input -o run.ini --format wrfout
$CPPWRF/build-cpu/cppwrf run.ini --set output.dir=out
```

The wrfout and wrfrst files go to `out` (a nested run: `out/d01`, `out/d02`, ...). `cppwrf_real` makes `wrfinput` and
`wrfbdy` from WPS met_em files instead of real.exe ([docs/REAL.md](docs/REAL.md)):

```bash
$CPPWRF/build-cpu/src/init_real/cppwrf_real --namelist namelist.input --met-dir /path/to/met_em
```

`examples/quickstart/run.sh` goes the whole way from the GFS analysis through WPS, `cppwrf_real` and the model to
wrfout files, with WRF's default domains and namelist ([docs/USER_GUIDE.md](docs/USER_GUIDE.md#4-from-wrf-inputs-to-wrfout)).
The builds above and this example were run from a clean source tree on a Google Colab G4 runtime
(`tools/colab/colab_quickstart.sh`); the GPU FP64 output equals the CPU FP64 output bit for bit.

The configurations in `configs/` include idealized cases (`build-cpu/cppwrf configs/straka.ini`; WRF's em_* cases
with WRF's `input_sounding`) and the suite's real-data cases, which read the WRF run directories from
`$CPPWRF_WRF_RUNS`.

## Documentation

- [docs/USER_GUIDE.md](docs/USER_GUIDE.md): building, the way from WPS to wrfout, the translator, every supported
  and rejected option, nests, GPU runs, restarts, output streams, configuration keys
- [CHANGELOG.md](CHANGELOG.md), [docs/RELEASE_NOTES.md](docs/RELEASE_NOTES.md): versions and the verified scope of
  v1.5.0 (and of the earlier releases)
- [docs/STATUS.md](docs/STATUS.md): what is ported, how it is verified, every case and result
- [docs/PERF.md](docs/PERF.md): the benchmark against WRF, the GPU profile and the performance roadmap
- [docs/FP32_IDENTITY.md](docs/FP32_IDENTITY.md): the FP32 builds that are bit-identical to WRF real*4
- [docs/FP32_ACCURACY.md](docs/FP32_ACCURACY.md): how close the fast GPU FP32 build is to WRF real*8, against stock
  WRF real*4
- [docs/PHYSICS_SUITE.md](docs/PHYSICS_SUITE.md), [docs/PHYSICS_DRIVER.md](docs/PHYSICS_DRIVER.md),
  [docs/PHYSICS_INTERFACE.md](docs/PHYSICS_INTERFACE.md): the physics port
- [docs/NESTING.md](docs/NESTING.md), [docs/NEST_NUMERICS.md](docs/NEST_NUMERICS.md): nesting
- [docs/REAL.md](docs/REAL.md): `cppwrf_real`

## Contributing

Bug reports, questions and requests for WRF options go to the issue tracker, with its templates. See
[CONTRIBUTING.md](CONTRIBUTING.md) for how changes are tested, [SECURITY.md](SECURITY.md) for reporting security
problems, and the [Code of Conduct](CODE_OF_CONDUCT.md).

## License

Copyright (C) 2026 Ömer Kaan Yılmaz. GPL-3.0-or-later; see [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md), which
includes the WRF notice. WRF is
developed by NCAR/MMM and its partners. CPPWRF is an independent port and is not affiliated with or endorsed
by NCAR or UCAR.
