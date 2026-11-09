# CPPWRF: frequently asked questions

### Is it really identical to WRF?

Yes, in FP64, for the configurations CPPWRF verifies. `cpu-fp64` and `gpu-fp64` give every value of every variable
at every output time exactly as WRF v4.8.0 real*8 does, at tolerance 0. NaN must equal NaN, and -0 must differ from
+0.

- **What was compared.** The verification suite runs 242 cases against WRF:
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

Speed. On the GPU, FP32 is 2 to 3 times faster than FP64, and it needs about 60 % of the memory. On a 359 x 269 x 44
domain, an hour of forecast steps in 3.0 s in FP32 against 9.6 s in FP64. WRF real*4 on 48 CPU threads takes 24.8 s
(docs/PERF.md).

FP32 is not bit-identical to anything. Its difference from WRF real*8 is about the size of the difference between
WRF's own real*4 and real*8 builds. For U after an hour, that is a relative L2 error of 2e-4 to 8e-4. Most
operational WRF runs use the real*4 build, so FP32 is the natural choice for forecasts and ensembles. Use FP64 when
you need WRF real*8's exact numbers.

### What GPU do I need?

An NVIDIA GPU with a CUDA 12 or 13 driver. The runs that prove bit identity used:

- the A100 (sm_80);
- the L4 (sm_89);
- the RTX PRO 6000 Blackwell (sm_120).

Other architectures compile for their `CMAKE_CUDA_ARCHITECTURES` value but have not been verified. One run uses one
GPU.

FP64 is fast only on data-centre GPUs (A100, H100). Consumer GeForce cards run FP64 at 1/32 or 1/64 of their FP32
speed, so use `gpu-fp32` on them. AMD and Intel GPUs are not supported. Without a GPU, `cpu-fp64` runs on any x86-64
CPU with SSE4.1, FMA and AVX2.

### How big a domain fits?

On the GPU, the domain has to fit in the GPU's memory. Measured peaks with full physics (docs/PERF.md):

| Domain | FP32 | FP64 |
|---|---|---|
| 119 x 89 x 44 (0.47 M cells) | 8.3 GB | 13.7 GB |
| the same with a 120 x 90 x 44 nest | 9.5 GB | 15.4 GB |
| 359 x 269 x 44 (4.25 M cells) | 20.8 GB | 30.0 GB |

Part of each peak is a fixed stack reserve for the column physics. It is 3 GB (FP32) and 8 GB (FP64) on a 96 GB
Blackwell card, and it scales with the number of threads the GPU can hold, so a 6 GB card reserves 9 times less.
Beyond that reserve, large domains take about 4 KB per cell in FP32 and 5 KB in FP64.

`cppwrf-fit namelist.input` estimates the memory and the run time of your namelist on your machine. On the CPU, the
limit is the machine's RAM. Each process is one machine, since CPPWRF has no MPI.

### Can I use my WRF namelist?

Yes, as it is. Run `cppwrf-check namelist.input` in your run directory. It lists every option as ported, ignored
(no effect, in WRF too or for CPPWRF), or not ported, with the reason and a suggestion. Then run `cppwrf-wrf` instead
of `wrf.exe`. An option without a port always stops the run at start-up with a clear message; it is never ignored
silently. `physics_suite` and the `-1` options work as in WRF.

### What is not ported?

- MPI runs (one process: OpenMP threads or one GPU);
- moving nests, vertical nest refinement, ndown.exe;
- the urban and lake models;
- the schemes outside the ported list: other microphysics, cumulus, PBL, surface-layer and land-surface schemes,
  including the Pleim-Xiu land surface model;
- Noah-MP's options outside the listed values;
- spectral and observation nudging, stochastic physics, chemistry;
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
