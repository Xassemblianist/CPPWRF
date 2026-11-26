# CPPWRF: frequently asked questions

### Is it really identical to WRF?

Yes, in FP64, for the configurations CPPWRF verifies. `cpu-fp64` and `gpu-fp64` give every value of every variable
at every output time exactly as WRF v4.8.0 real*8 does, at tolerance 0. NaN must equal NaN, and -0 must differ from
+0.

- **What was compared.** For v1.5.0 at `ac13ebe`, CPU CI passed 533 suite checks against WRF at tolerance 0
  (489 FP64 and 44 FP32), 42 WRF output file checks, 14 ideal file checks, `ctest -L diff` 32/32 and CTest 44/44.
  Another 154 CPU FP64 cases run with `--slow`. The v1.5.0 Colab G4 run at `40628eb` passed all 423 CUDA FP64
  checks against WRF at tolerance 0. The v1.3.0 runs at `318159a` reported 483 CPU FP64 and 423 CUDA FP64 passes,
  including the legacy Straka RMS checks. The `ideal_grav2d_x` cases verify the Straka density current on CPU and
  CUDA against WRF real*8 at tolerance 0. The suite runs CPU checks on each push and GPU checks for each release.
  - real-data domains in the mid-latitudes, the tropics, the Arctic and the Alps;
  - 24 h and 72 h forecasts;
  - nests, restarts, and WRF's idealized cases;
  - several hundred namelist variants.

  The wrfout and wrfrst files also match WRF's in their metadata. docs/RELEASE_NOTES.md has the exact scope, and
  docs/STATUS.md every case.
- **"Identical" means identical to one particular WRF:** WRF v4.8.0 built with gfortran 14.2 and glibc 2.41,
  `configure -r8`, and run in one process. WRF itself gives different last bits with another compiler, other flags
  (`-march`) or another MPI layout, and those differences grow during a forecast.
- **The method.** CPPWRF repeats WRF's arithmetic exactly: the order of every sum, the constants gfortran folds,
  the halo schedule, and the glibc math functions gfortran calls, with ports of them on the GPU.
- **To check it yourself,** run the self-test (docs/INSTALL.md) and compare a run of your own with
  `cppwrf-compare` (docs/MIGRATING_FROM_WRF.md).

### Why FP32?

Speed. In the v1.3.0 paper benchmark (`9f9bb77`), the fast GPU FP32 build (`-DCPPWRF_CUDA_FP32_IDENTITY=OFF`) is 2.9 to
3.4 times faster than GPU FP64 in steady stepping. Both FP32 builds use 60 to 70 % of FP64's peak GPU memory across
the benchmark cases. On a 359 x 269 x 44 domain (C2, 6 h run), steady stepping takes 2.78 s per simulated hour in fast
FP32, 2.84 s in identity FP32, and 9.16 s in GPU FP64. WRF real*4 at its best stock layout on the same machine
(48 hardware threads) takes 28.60 s. These figures use the history-off medians from `runs.csv` and `summary.json`.

The default `gpu-fp32` build (`CPPWRF_CUDA_FP32_IDENTITY=ON`) is bit-identical to `cpu-fp32` and to stock WRF v4.8.0
real*4 on the 24 configurations of docs/FP32_IDENTITY.md. In the v1.3.0 paper benchmark the identity build takes
2.1 to 18.0 % longer in steady stepping than the fast build (identity time / fast time - 1). The opt-in fast build
(`-DCPPWRF_CUDA_FP32_IDENTITY=OFF`) is not bit-identical to WRF real*4. Its difference from WRF real*8 is about the
size of the difference between WRF's own real*4 and real*8 builds. The benchmark's accuracy files give a relative L2 error of
3.2e-4 to 7.9e-4 for U after an hour against CPPWRF GPU FP64 (see also docs/FP32_ACCURACY.md). Most operational
WRF runs use the real*4 build, so FP32 is the natural choice for forecasts and ensembles. Use FP64 when you need WRF
real*8's exact numbers.

### What GPU do I need?

An NVIDIA GPU with a CUDA 12 or 13 driver. The runs that prove bit identity used:

- the A100 (sm_80);
- the L4 (sm_89);
- the RTX PRO 6000 Blackwell (sm_120).

Other architectures compile for their `CMAKE_CUDA_ARCHITECTURES` value but have not been verified. One run uses one
GPU.

FP64 is fast only on data-centre GPUs (A100, H100). Consumer GeForce cards run FP64 at 1/32 or 1/64 of their FP32
speed, so use `gpu-fp32` on them. Intel GPUs are not supported; AMD GPUs have an opt-in HIP / ROCm build path
(`-DCPPWRF_ENABLE_HIP=ON`, docs/HIP.md) that has not been run or verified on AMD hardware. Without a GPU, `cpu-fp64`
runs on any x86-64 CPU with SSE4.1, FMA and AVX2.

### How big a domain fits?

On the GPU, the domain has to fit in the GPU's memory. Measured peaks with full P4 physics on an RTX PRO 6000 Blackwell
in the v1.3.0 paper benchmark (`9f9bb77`), from the history-off `mem_used_peak_mib` medians in `runs.csv`:

| Domain | FP32 (identity, default) | FP32 (fast) | FP64 |
|---|---|---|---|
| 119 x 89 x 44 (0.47 M cells) | 6.1 GiB (6209 MiB) | 6.1 GiB (6205 MiB) | 8.8 GiB (9051 MiB) |
| the same with a 120 x 90 x 44 nest | 7.0 GiB (7191 MiB) | 7.0 GiB (7187 MiB) | 10.4 GiB (10667 MiB) |
| 359 x 269 x 44 (4.25 M cells) | 16.5 GiB (16935 MiB) | 16.5 GiB (16931 MiB) | 27.5 GiB (28115 MiB) |

Part of each peak is a stack reserve. CUDA reserves a kernel's local arrays for every thread the GPU can hold.
The benchmark's `CPPWRF_MEMINFO` warm-up logs give 3018 MiB in FP32 and 2965 MiB in FP64 on the RTX PRO 6000.
The earlier RTX 2060 estimate is about 0.4 GB. The large per-column arrays of the column physics live in a work
buffer of at most 1/8 of the device memory (docs/USER_GUIDE.md, "GPU memory").
From the difference between the two single-domain P4 cases (C2 and C1), peak memory grows by about 2.9 KiB per cell
in FP32 and 5.2 KiB in FP64.

`cppwrf-fit namelist.input` estimates the memory and the run time of your namelist on your machine. On the CPU, the
limit is the machine's RAM. All verified MPI and tile layouts ran on one node. The CUDA FP64 checks covered up to
two GPUs. See [MPI.md](MPI.md).

### Can I use my WRF namelist?

Yes, as it is. Run `cppwrf-check namelist.input` in your run directory. It lists every option as ported, ignored
(no effect, in WRF too or for CPPWRF), or not ported, with the reason and a suggestion. Then run `cppwrf-wrf` instead
of `wrf.exe`. An option without a port always stops the run at start-up with a clear message; it is never ignored
silently. `physics_suite` and the `-1` options work as in WRF.

### What is not ported?

- moving nests, vertical nest refinement, ndown.exe;
- the urban canopy models other than the single-layer UCM (sf_urban_physics = 1 with Noah), and the lake model;
- the schemes outside the ported list: other microphysics, cumulus, PBL, surface-layer and land-surface schemes,
  including the Pleim-Xiu land surface model;
- Noah-MP's options outside the listed values;
- spectral nudging, observation nudging on grids other than Lambert conformal or with tiles / MPI ranks, stochastic physics, chemistry;
- the adaptive time step with nests;
- I/O forms other than NetCDF.

docs/MIGRATING_FROM_WRF.md lists every ported value, generated from the translator, and docs/STATUS.md keeps the
list of what is missing. Requests for a WRF option use the "feature request" issue template.

### Does it work with WPS, real.exe and my post-processing?

Yes:

- WPS is unchanged.
- `cppwrf-realexe` replaces real.exe, and WRF's own real.exe works too.
- The wrfout files are WRF's, so wrf-python, NCL, UPP and ncview read them.
- The `cppwrf` Python package (docs/PYTHON.md) reads them too, with a few standard diagnostics.

### Does the CPU result depend on the number of threads?

No. Every kernel writes its own grid points, and the few sums over the domain use a fixed order whatever the thread
count. `cpu-fp64` with 1 thread and with 48 threads gives the same bits.

### Can I restart a run?

Yes, as in WRF. `restart_interval` writes `wrfrst_d0N_<date>` files, and `restart = .true.` with the start date of
a restart file resumes from it. WRF's and CPPWRF's restart files are interchangeable: a `cpu-fp64` run resumed
from WRF's restart file gives WRF's resumed results bit for bit (verified in the suite).

### How do I report a bug?

Open an issue at <https://github.com/Xassemblianist/CPPWRF/issues> with the "bug report" template. Include:

- the CPPWRF version and commit (`cppwrf --version`);
- the build flags, the compiler and the CUDA version;
- the GPU or CPU;
- your `namelist.input`, the output of `cppwrf-check namelist.input`, and `rsl.error.0000`.

If CPPWRF and WRF differ on the same case, say how WRF was built and run (real*4 or real*8, the compiler, MPI or not),
and include the output of `cppwrf-compare`. A small case that shows the problem in a few steps helps the most.
Security problems go by e-mail instead (SECURITY.md).
