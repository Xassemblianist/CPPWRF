# CPPWRF User Guide

Welcome to **CPPWRF**, a modern, high-performance C++20 reimplementation of the Weather Research and Forecasting (WRF) Advanced Research WRF (ARW) dynamical core and physics suite.

This guide provides instructions for building CPPWRF on CPU and GPU architectures, running idealized and real-data cases, configuring model options via INI configuration files, translating WRF `namelist.input` files, interpreting output datasets, and executing the independent verification suite.

---

## Table of Contents
1. [Architecture & Design Principles](#1-architecture--design-principles)
2. [Building CPPWRF](#2-building-cppwrf)
   - [Prerequisites](#prerequisites)
   - [CPU Build (OpenMP, FP32 / FP64)](#cpu-build-openmp-fp32--fp64)
   - [CUDA GPU Build](#cuda-gpu-build)
   - [Build Flags & CMake Options](#build-flags--cmake-options)
3. [Running Simulations](#3-running-simulations)
   - [Command-Line Invocation](#command-line-invocation)
   - [Translating WRF Namelists (`wrf2cppwrf.py`)](#translating-wrf-namelists-wrf2cppwrfpy)
   - [Idealized Cases](#idealized-cases)
   - [Real-Data Simulations (WPS + real.exe)](#real-data-simulations-wps--realexe)
4. [Configuration Reference (INI Keys)](#4-configuration-reference-ini-keys)
   - [Summary Table](#configuration-keys-table)
   - [Section Details](#section-details)
5. [Outputs & Data Formats](#5-outputs--data-formats)
   - [Native Format (`run.json`, `diag.csv`, `snap_*.bin`)](#native-format)
   - [WRF NetCDF Output (`wrfout_d01_*`)](#wrf-netcdf-output)
   - [Restart Files & Resuming Simulations](#restart-files--resuming-simulations)
6. [Verification & Validation](#6-verification--validation)
   - [Independent Verification Harness (`verify.py`)](#independent-verification-harness-verifypy)
   - [Real-Data WRF Output Comparison (`compare_real.py`)](#real-data-wrf-output-comparison-compare_realpy)
   - [Unit & Differential Physics Tests](#unit--differential-physics-tests)
7. [Performance & Shared-Machine Operational Rules](#7-performance--shared-machine-operational-rules)

---

## 1. Architecture & Design Principles

CPPWRF is engineered to match WRF ARW (v4.6.1) numerics and equations discretely:
<!-- cite: docs/team/PLAN.md:90-95 -->
- **Equation Set:** Fully compressible, non-hydrostatic Euler equations in flux form with hybrid terrain-following vertical coordinates ($\eta$).
- **Time Integration:** 3rd-order Runge-Kutta (RK3) with split-explicit acoustic substeps.
<!-- cite: src/dyn/rk3.hpp:2-7 -->
- **Spatial Discretization:** Arakawa C-grid staggering; 5th- or 6th-order horizontal advection; 3rd- or 4th-order vertical advection.
<!-- cite: src/dyn/large_step.hpp:15-16 -->
- **Thermodynamic Consistency:** Diagnostic pressure and inverse density using moist potential temperature $\theta_m = \theta(1 + (R_v/R_d)q_v)$.
<!-- cite: src/cases/real_case.cpp:6-7, src/dyn/large_step.hpp:28-33 -->
- **Single-Source Kernels:** Dynamics and physics kernels run identically on CPU (OpenMP parallel loops) and GPU (CUDA device kernels) without code duplication.
<!-- cite: CMakeLists.txt:80-94 -->
- **Strict IEEE Floating-Point:** Built with `-ffp-contract=off` (no fused multiply-add contraction) to maintain bitwise determinism and exact verification against Fortran reference models.
<!-- cite: CMakeLists.txt:32-39 -->

---

## 2. Building CPPWRF

### Prerequisites
- Modern C++20 compiler (`g++` >= 11 or `clang++` >= 14) <!-- cite: CMakeLists.txt:4-5 -->
- CMake >= 3.24 <!-- cite: CMakeLists.txt:1 -->
- OpenMP runtime library <!-- cite: CMakeLists.txt:29,93 -->
- NetCDF-C development headers (`libnetcdf-dev`) <!-- cite: CMakeLists.txt:60-78 -->
- *(Optional)* NVIDIA CUDA Toolkit >= 12.0 for GPU offload <!-- cite: CMakeLists.txt:12-27 -->

All builds and tests should be wrapped with `tools/heavy.sh` to respect machine slot limits on shared systems.
<!-- cite: tools/heavy.sh:1-22, docs/team/PLAN.md:11-15 -->

### CPU Build (OpenMP, FP32 / FP64)
To build standard single-precision (float32) for CPU:
```bash
cmake -B build-cpu -S . -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF
tools/heavy.sh cmake --build build-cpu -j4
```
<!-- cite: CMakeLists.txt:18,29,96-100 -->

To build double-precision (float64, `real = double`):
```bash
cmake -B build-cpu-dp -S . -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON
tools/heavy.sh cmake --build build-cpu-dp -j4
```
<!-- cite: CMakeLists.txt:10,56-58, src/core/precision.hpp:6-18 -->

### CUDA GPU Build
To build the GPU-accelerated backend:
```bash
cmake -B build-cuda -S . -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=native
tools/heavy.sh cmake --build build-cuda -j4
```
<!-- cite: CMakeLists.txt:18-28,87-94 -->

### Build Flags & CMake Options
| CMake Option | Default | Description | Citation |
|---|---|---|---|
| `CMAKE_BUILD_TYPE` | `Release` | Compiler optimization level (`Release`, `Debug`, `RelWithDebInfo`) | <!-- cite: CMakeLists.txt:7 --> |
| `CPPWRF_DOUBLE` | `OFF` | Use double-precision floats (`real = double`) across dycore and physics | <!-- cite: CMakeLists.txt:10,56-58 --> |
| `CPPWRF_ENABLE_CUDA` | Auto | Enable CUDA compilation for NVIDIA GPUs | <!-- cite: CMakeLists.txt:18-28 --> |
| `CMAKE_CUDA_ARCHITECTURES` | `native` | Target GPU architecture (e.g. `80;90` for Ampere/Hopper) | <!-- cite: CMakeLists.txt:21-23 --> |
| `CPPWRF_NATIVE` | `ON` | Optimize for host CPU architecture (`-march=native -ffp-contract=off`) | <!-- cite: CMakeLists.txt:34-39 --> |
| `CPPWRF_DIFF_TESTS` | `OFF` | Register WRF Fortran differential test suites with `ctest` | <!-- cite: CMakeLists.txt:114-123 --> |

---

## 3. Running Simulations

### Command-Line Invocation
Simulations are launched by providing an INI configuration file to the executable:
```bash
tools/heavy.sh ./build-cpu/cppwrf configs/real_r0.ini
```
<!-- cite: src/main.cpp:1,170-172 -->

Individual configuration parameters can be overridden dynamically from the command line without modifying the `.ini` file:
```bash
# Override total simulation time to 2 minutes (120 s)
tools/heavy.sh ./build-cpu/cppwrf configs/real_r0.ini --set run.run_time=120

# Multiple overrides
tools/heavy.sh ./build-cpu/cppwrf configs/real_p4.ini --set output.format=both --set output.out_interval=5
```
<!-- cite: src/main.cpp:173-180 -->

### Translating WRF Namelists (`wrf2cppwrf.py`)
CPPWRF includes a standalone translator that converts WRF `namelist.input` files into valid CPPWRF `.ini` configurations:
```bash
# Basic conversion
python3 tools/wrf2cppwrf.py /path/to/wrf/namelist.input -o run.ini

# Real-data conversion with specified directory and NetCDF wrfout format
python3 tools/wrf2cppwrf.py ~/wrf_real/run_R0_r8/namelist.input -o run_r0.ini --format wrfout
```
<!-- cite: tools/wrf2cppwrf.py:465-475 -->

Key translator features:
- **f90nml-Free Parser:** Pure Python tokenizer supporting standard Fortran namelist syntax, repetition counts (`3*0`), exponents (`1.0d-4`), and inline comments. <!-- cite: tools/wrf2cppwrf.py:27-233 -->
- **Single-Domain Filtering:** Extracts domain 1 settings; issues descriptive warnings when nested domain arrays (`max_dom > 1`) are present. <!-- cite: tools/wrf2cppwrf.py:410-413 -->
- **Non-Silent Diagnostics:** Reports clear warnings on unsupported schemes (e.g. `diff_opt=2`, `mp_physics=2`) and informational notices for ignored WPS/quilt metadata. <!-- cite: tools/wrf2cppwrf.py:252-349,530-538 -->

### Idealized Cases
CPPWRF supports standard ARW idealized benchmarks:
- **`rest_state`:** Stably stratified atmosphere at rest testing hydrostatic balance. <!-- cite: src/cases/cases.cpp:95-101 -->
- **`warm_bubble`:** Bryan & Fritsch (2002) dry rising thermal bubble in neutral atmosphere. <!-- cite: src/cases/cases.cpp:113-122 -->
- **`straka`:** Straka et al. (1993) cold density current benchmark. <!-- cite: src/cases/cases.cpp:102-112 -->
- **`igw`:** Skamarock & Klemp (1994) inertia-gravity wave. <!-- cite: src/cases/cases.cpp:123-133 -->
- **`squall2d`:** Weisman & Klemp (1982) squall line with Kessler or WSM6 microphysics. <!-- cite: src/cases/cases.cpp:134-178 -->
- **`hill2d`:** Flow over a 2D bell-shaped mountain from WRF `input_sounding`. <!-- cite: src/cases/hill2d.cpp:11-235 -->
- **`quarter_ss`:** 3D supercell storm case from `input_sounding`. <!-- cite: src/cases/quarter_ss.cpp:11-255 -->

Run an idealized case:
```bash
tools/heavy.sh ./build-cpu/cppwrf configs/straka_wrf.ini
```

### Real-Data Simulations (WPS + real.exe)
Real-data simulations use initial conditions (`wrfinput_d01`) and lateral boundary updates (`wrfbdy_d01`) generated by the standard WRF Preprocessing System (WPS) and `real.exe`:
<!-- cite: src/cases/real_case.cpp:1-8 -->
1. Run `geogrid.exe`, `ungrib.exe`, and `metgrid.exe` in WPS.
2. Run `real.exe` in WRF to generate `wrfinput_d01` and `wrfbdy_d01`.
3. In your `.ini` file:
   ```ini
   [run]
   case = real
   run_time = 3600

   [real]
   dir = /path/to/wrf_real/run_dir
   wrfinput = wrfinput_d01
   wrfbdy = wrfbdy_d01

   [time]
   dt = 60

   [output]
   format = wrfout
   out_interval = 10
   ```
<!-- cite: src/cases/real_case.cpp:46-56,184-192 -->
4. Execute with `tools/heavy.sh ./build-cpu/cppwrf my_real_run.ini`.

---

## 4. Configuration Reference (INI Keys)

### Configuration Keys Table
Every option supported by the CPPWRF runtime is cataloged below, along with its type, default, WRF equivalent, and code source.

| Section | Key | Type | Default | Description | WRF Equivalent | Citation |
|---|---|---|---|---|---|---|
| `[run]` | `case` | string | *required* | Simulation case (`real`, `hill2d`, `quarter_ss`, `straka`, `squall2d`, `warm_bubble`, `rest_state`, `igw`) | `ideal_case` | <!-- cite: src/main.cpp:181, src/cases/model.cpp:155 --> |
| `[run]` | `run_time` | double | `0.0` | Total simulation duration in seconds | `run_days/hours/minutes/seconds` | <!-- cite: src/cases/model.cpp:78, src/cases/real_case.cpp:55 --> |
| `[run]` | `restart_file` | string | `""` | Path to NetCDF restart file (`restart_*.nc`) to resume execution | `restart` / restart filename | <!-- cite: src/main.cpp:206,222 --> |
| `[run]` | `start_date` | string | `"2000-01-01_00:00:00"` | UTC start timestamp string | `start_year/month/day/hour/etc` | <!-- cite: src/main.cpp:209 --> |
| `[real]` | `dir` | string | *required (real)* | Directory containing `wrfinput_d01` and `wrfbdy_d01` | WRF run directory | <!-- cite: src/cases/real_case.cpp:47 --> |
| `[real]` | `wrfinput` | string | `"wrfinput_d01"` | Name of initial condition NetCDF file | `wrfinput_d01` | <!-- cite: src/cases/real_case.cpp:48 --> |
| `[real]` | `wrfbdy` | string | `"wrfbdy_d01"` | Name of boundary condition NetCDF file | `wrfbdy_d01` | <!-- cite: src/cases/real_case.cpp:185 --> |
| `[real]` | `bdy_interval` | double | `0.0` (auto) | Boundary condition update interval in seconds (0 = auto-detected from file) | `interval_seconds` | <!-- cite: src/cases/real_case.cpp:188-191 --> |
| `[domain]` | `nx`, `ny`, `nz` | integer | *required (ideal)* | Grid dimensions in mass-point coordinates | `e_we - 1`, `e_sn - 1`, `e_vert - 1` | <!-- cite: src/cases/model.cpp:69-71 --> |
| `[domain]` | `dx`, `dy` | double | *required (ideal)* | Horizontal grid spacing in meters | `dx`, `dy` | <!-- cite: src/cases/model.cpp:72-73 --> |
| `[domain]` | `ztop` | double | *required (ideal)* | Model top elevation in meters | `ztop` | <!-- cite: src/cases/model.cpp:74 --> |
| `[domain]` | `p_top` | double | Sounding $p(z_{top})$ | Pressure at top boundary in Pa | `p_top_requested` | <!-- cite: src/cases/model.cpp:94 --> |
| `[domain]` | `bc_x`, `bc_y` | string | `"periodic"` | Lateral boundary condition (`periodic`, `symmetric`, `specified`) | `periodic_x`, `symmetric_xs`, etc. | <!-- cite: src/cases/model.cpp:75-76 --> |
| `[domain]` | `eta` | string | `"uniform_z"` | Vertical coordinate distribution (`uniform_z`, `uniform_eta`, `list`) | `eta_levels` | <!-- cite: src/cases/cases.cpp:203-216 --> |
| `[domain]` | `eta_levels` | float list | `[]` | Explicit comma-separated list of full $\eta$ levels | `eta_levels` | <!-- cite: src/cases/cases.cpp:213 --> |
| `[domain]` | `map_factor` | double | `1.0` | Constant horizontal map projection scale factor | Map scale factor ($m$) | <!-- cite: src/cases/model.cpp:85-103 --> |
| `[domain]` | `f0`, `e0` | double | `0.0` | Coriolis parameters $f$ and $e$ in $s^{-1}$ ($f$-plane) | Coriolis parameters | <!-- cite: src/cases/model.cpp:104-109 --> |
| `[case]` | `sounding_file` | string | `""` | Path to WRF format sounding file | `input_sounding` | <!-- cite: src/cases/hill2d.cpp:65, src/cases/quarter_ss.cpp:65 --> |
| `[case]` | `hm` | double | `100.0` | Mountain height in meters (for `hill2d`) | Hill height | <!-- cite: src/cases/hill2d.cpp:174 --> |
| `[case]` | `xa` | double | `5.0` | Mountain half-width in km (for `hill2d`) | Hill half-width | <!-- cite: src/cases/hill2d.cpp:175 --> |
| `[case]` | `u0` | double | `10.0` | Uniform inflow wind speed in m/s | Mean wind | <!-- cite: src/cases/hill2d.cpp:229 --> |
| `[case]` | `delt` | double | `3.0` | Thermal perturbation magnitude in K (for `quarter_ss`) | Bubble perturbation | <!-- cite: src/cases/quarter_ss.cpp:249 --> |
| `[time]` | `dt` | double | *required* | Large time step in seconds ($\Delta t$) | `time_step` | <!-- cite: src/cases/model.cpp:77 --> |
| `[time]` | `time_step_sound` | integer | `0` (auto) | Acoustic small steps per RK stage (0 = auto-computed CFL rule) | `time_step_sound` | <!-- cite: src/cases/model.cpp:202-203 --> |
| `[dyn]` | `khdif`, `kvdif` | double | `0.0` | Constant horizontal/vertical eddy diffusion in $m^2/s$ | `khdif`, `kvdif` | <!-- cite: src/cases/model.cpp:159-160 --> |
| `[dyn]` | `diff_opt` | integer | `0` | Diffusion option (0 = off, 1 = coordinate/slope diffusion) | `diff_opt` | <!-- cite: src/cases/model.cpp:191 --> |
| `[dyn]` | `km_opt` | integer | `1` | Eddy viscosity closure (1 = constant, 4 = 2D Smagorinsky) | `km_opt` | <!-- cite: src/cases/model.cpp:192 --> |
| `[dyn]` | `c_s` | double | `0.25` | Smagorinsky diffusion constant | `c_s` | <!-- cite: src/cases/model.cpp:194 --> |
| `[dyn]` | `diff_6th_opt` | integer | `0` | 6th-order spatial diffusion (0 = off, 1 = standard, 2 = monotonic) | `diff_6th_opt` | <!-- cite: src/cases/model.cpp:195 --> |
| `[dyn]` | `diff_6th_factor` | double | `0.12` | 6th-order numerical diffusion coefficient | `diff_6th_factor` | <!-- cite: src/cases/model.cpp:196 --> |
| `[dyn]` | `pr_inv` | double | `3.0` | Inverse turbulent Prandtl number ($Pr^{-1} = K_\theta / K_m$) | $1 / \text{prandtl}$ | <!-- cite: src/cases/model.cpp:161 --> |
| `[dyn]` | `top_lid` | boolean | `false` | Rigid lid at top boundary (extrapolates $p'$) | `top_lid` | <!-- cite: src/cases/model.cpp:162 --> |
| `[dyn]` | `h_adv_order` | integer | `5` | Horizontal advection order (5 = upwind, 6 = centered) | `h_mom_adv_order` | <!-- cite: src/cases/model.cpp:163 --> |
| `[dyn]` | `v_adv_order` | integer | `3` | Vertical advection order (3 = upwind, 4 = centered) | `v_mom_adv_order` | <!-- cite: src/cases/model.cpp:164 --> |
| `[dyn]` | `w_damping` | integer | `0` | Vertical velocity damping where $w$-CFL > 1 (0 = off, 1 = on) | `w_damping` | <!-- cite: src/cases/model.cpp:165 --> |
| `[dyn]` | `w_crit_cfl` | double | `1.0` | Critical Courant number threshold for $w$-damping | `w_crit_cfl` | <!-- cite: src/cases/model.cpp:166 --> |
| `[dyn]` | `damp_opt` | integer | `0` | Upper damping layer (0 = off, 3 = implicit Rayleigh damping of $w$) | `damp_opt` | <!-- cite: src/cases/model.cpp:176 --> |
| `[dyn]` | `zdamp` | double | `5000.0` | Depth of upper Rayleigh damping sponge layer in meters | `zdamp` | <!-- cite: src/cases/model.cpp:178 --> |
| `[dyn]` | `dampcoef` | double | `0.2` | Rayleigh damping coefficient ($s^{-1}$) | `dampcoef` | <!-- cite: src/cases/model.cpp:179 --> |
| `[dyn]` | `moist_adv_opt` | integer | `1` | Moisture advection scheme (0 = standard, 1 = positive-definite) | `moist_adv_opt` | <!-- cite: src/cases/model.cpp:181 --> |
| `[dyn]` | `scalar_adv_opt` | integer | `1` | Passive tracer advection (0 = standard, 1 = positive-definite) | `scalar_adv_opt` | <!-- cite: src/cases/model.cpp:182 --> |
| `[dyn]` | `epssm` | double | `0.1` | Acoustic small-step forward-centering factor | `epssm` | <!-- cite: src/cases/model.cpp:169 --> |
| `[dyn]` | `smdiv`, `emdiv` | double | `0.1`, `0.01` | Acoustic divergence and external mode filters | `smdiv`, `emdiv` | <!-- cite: src/cases/model.cpp:170-171 --> |
| `[moist]` | `species` | string list | `""` | Comma-separated list of active water species (`qv,qc,qr,qi,qs,qg`) | WRF moisture array | <!-- cite: src/cases/model.cpp:113, src/cases/real_case.cpp:126 --> |
| `[phys]` | `mp_physics` | integer | `0` | Microphysics scheme (0 = off, 1 = Kessler, 6 = WSM6) | `mp_physics` | <!-- cite: src/cases/model.cpp:208, src/dyn/physics_coupling.hpp:14 --> |
| `[phys]` | `hail_opt` | integer | `0` | WSM6 6th class density (0 = graupel, 1 = hail) | `hail_opt` | <!-- cite: src/cases/model.cpp:210, src/dyn/physics_coupling.hpp:15 --> |
| `[phys]` | `mp_tend_lim` | double | `10.0` | Heating rate limiter for microphysics ($K/s$) | `mp_tend_lim` | <!-- cite: src/cases/model.cpp:209, src/dyn/physics_coupling.hpp:16 --> |
| `[phys]` | `sf_sfclay_physics` | integer | `1` | Surface layer formulation (0 = off, 1 = Revised MM5 / sfclayrev) | `sf_sfclay_physics` | <!-- cite: src/phys/driver/suite.cpp:393 --> |
| `[phys]` | `sf_surface_physics` | integer | `2` | Land surface model (0 = off, 2 = Unified Noah LSM 4-layer) | `sf_surface_physics` | <!-- cite: src/phys/driver/suite.cpp:394 --> |
| `[phys]` | `bl_pbl_physics` | integer | `1` | Planetary boundary layer scheme (0 = off, 1 = YSU) | `bl_pbl_physics` | <!-- cite: src/phys/driver/suite.cpp:395 --> |
| `[phys]` | `cu_physics` | integer | `0` | Cumulus parameterization (0 = off, 1 = Kain-Fritsch) | `cu_physics` | <!-- cite: src/phys/driver/suite.cpp:396 --> |
| `[phys]` | `ra_lw_physics` | integer | `0` | Longwave radiation scheme (0 = off, 4 = RRTMG LW) | `ra_lw_physics` | <!-- cite: src/phys/driver/suite.cpp:397 --> |
| `[phys]` | `ra_sw_physics` | integer | `0` | Shortwave radiation scheme (0 = off, 4 = RRTMG SW) | `ra_sw_physics` | <!-- cite: src/phys/driver/suite.cpp:398 --> |
| `[phys]` | `radt` | double | `0.0` | Radiation physics call interval (minutes; 0 = every step) | `radt` | <!-- cite: src/phys/driver/suite.cpp:399 --> |
| `[phys]` | `table_dir` | string | `"."` | Path to directory containing physics tables (`LANDUSE.TBL`, etc.) | WRF run directory | <!-- cite: src/phys/driver/suite.cpp:408 --> |
| `[output]` | `dir` | string | `"out/<case>"` | Directory for simulation output files | Output path | <!-- cite: src/main.cpp:198 --> |
| `[output]` | `format` | string | `"native"` | Output data format (`native`, `wrfout`, `both`) | `io_form_history` | <!-- cite: src/main.cpp:200 --> |
| `[output]` | `diag_interval` | integer | `10` | Frequency (in steps) for writing rows to `diag.csv` | Diagnostic interval | <!-- cite: src/main.cpp:196 --> |
| `[output]` | `out_interval` | integer | `nsteps` | Frequency (in steps) for writing field snapshots | `history_interval` | <!-- cite: src/main.cpp:197 --> |
| `[restart]` | `interval` | double | `0.0` | Restart snapshot write frequency in seconds (0 = disabled) | `restart_interval` | <!-- cite: src/main.cpp:207 --> |
| `[tracers]` | `names` | string list | `""` | Comma-separated names of passive scalar tracers | Tracer variables | <!-- cite: src/cases/model.cpp:117 --> |

---

## 5. Outputs & Data Formats

### Native Format
When `output.format = native` (or `both`), CPPWRF writes raw binary dumps:
<!-- cite: src/main.cpp:201,353-357 -->
1. **`run.json`:** Comprehensive simulation metadata containing grid parameters, vertical levels, physical constants, execution precision, and resolved configuration keys. <!-- cite: src/main.cpp:234-272 -->
2. **`diag.csv`:** Tabular time-series diagnostics recorded every `output.diag_interval` steps. Columns include:
   - `step`: Time step index
   - `time`: Elapsed simulation seconds
   - `max_abs_u`, `max_abs_v`, `max_abs_w`: Peak velocity magnitudes ($m/s$)
   - `min_th_pert`, `max_th_pert`: Extrema of potential temperature perturbation $\theta'$ ($K$)
   - `dry_mass`, `theta_mass`: Total domain mass integrals
   - `total_energy`: Total dry energy
   - `wall_s`: Cumulative wall clock execution time
   <!-- cite: src/main.cpp:295-318 -->
3. **`snap_<step>.bin` and `snap_<step>.json`:** Multi-field binary array dumps with companion JSON headers detailing variable offsets, shapes, and IEEE precision. <!-- cite: src/io/output.hpp:64-71 -->

### WRF NetCDF Output
When `output.format = wrfout` (or `both`), CPPWRF produces files named `wrfout_d01_<YYYY-MM-DD_HH:MM:SS>`.
<!-- cite: src/main.cpp:331-372, src/io_netcdf/wrfout_writer.hpp:21-25 -->
- Fully compatible with standard WRF post-processing tools: `ncview`, Python (`xarray`, `netCDF4`, `wrf-python`), and NCL.
- Contains 3D prognostic state fields (`U`, `V`, `W`, `T`, `PH`, `P`, `PB`, `PHB`, `QVAPOR`, `QCLOUD`, `QRAIN`, etc.) and 2D surface diagnostics (`TSK`, `HFX`, `QFX`, `LH`, `UST`, `PBLH`, `RAINNC`).

### Restart Files & Resuming Simulations
NetCDF restart files (`restart_<step>.nc`) store the complete prognostic state at bitwise precision.
<!-- cite: src/main.cpp:375-383, src/io_netcdf/restart.hpp:19-27 -->

To enable periodic restart snapshots:
```ini
[restart]
interval = 3600   # Write restart file every 3600 seconds (1 hour)
```
<!-- cite: src/main.cpp:207-208 -->

To resume a simulation from a restart file:
```ini
[run]
case = real
run_time = 7200
restart_file = out/real/restart_000060.nc
```
<!-- cite: src/main.cpp:206,220-231 -->
When resuming, pre-restart rows in `diag.csv` are preserved automatically.
<!-- cite: src/main.cpp:274-300 -->

---

## 6. Verification & Validation

CPPWRF includes an independent, black-box verification suite located in `tests/verify/`.
<!-- cite: tests/verify/verify.py:1-14 -->

### Independent Verification Harness (`verify.py`)
Run standard verification commands:
```bash
# Check idealized simulation physics & conservation laws
python3 tests/verify/verify.py check straka out/straka
python3 tests/verify/verify.py check warm_bubble out/warm_bubble
python3 tests/verify/verify.py check rest_state out/rest_state

# Field-by-field differential comparison against WRF reference run
python3 tests/verify/verify.py wrf /path/to/wrfout_d01 out/straka

# Compare two CPPWRF model runs (e.g. CPU vs GPU or FP32 vs FP64)
python3 tests/verify/verify.py compare out/cpu out/cuda --rtol 1e-4

# Grid convergence test (dx, dx/2, dx/4)
python3 tests/verify/verify.py convergence out/dx100 out/dx50 out/dx25
```
<!-- cite: tests/verify/verify.py:9-14 -->

### Real-Data WRF Output Comparison (`compare_real.py`)
Compare CPPWRF real-data simulations directly against WRF reference runs:
```bash
python3 tests/verify/compare_real.py /path/to/wrfout_d01 out/real --interior 6
```
<!-- cite: tests/verify/compare_real.py:4-9 -->
Reports max absolute difference and relative $L_2$ errors across all 3D fields (`U`, `V`, `W`, `PH`, `MU`, `P`, `THM`, `QVAPOR`) and 2D surface variables (`TSK`, `HFX`, `QFX`, `LH`, `UST`, `PBLH`, `RAINNC`).

### Unit & Differential Physics Tests
Run the automated CTest unit and regression suites:
```bash
# Core C++ unit tests
tools/heavy.sh ctest --test-dir build-cpu --output-on-failure

# Python translation tool tests
tools/heavy.sh python3 tests/tools/test_wrf2cppwrf.py -v
```
<!-- cite: CMakeLists.txt:42,102-110 -->

---

## 7. Performance & Shared-Machine Operational Rules

When developing and running on shared compute resources (12 cores, 15 GB RAM, 1 GPU), adherence to team operational rules is mandatory:
<!-- cite: docs/team/PLAN.md:10-16, docs/team/HIERARCHY.md:36-43, tools/heavy.sh:1-78 -->
- **Job Slots (`tools/heavy.sh`):** Wrap builds, model runs, and tests with `tools/heavy.sh <cmd>` (runs in one of 4 slots with `OMP_NUM_THREADS=3 OMP_WAIT_POLICY=passive`). <!-- cite: tools/heavy.sh:3-4,15,23 -->
- **Quick Jobs (`--light`):** For small compiles, ncdump, short Python checks, or quick 1-2 step test runs, use `tools/heavy.sh --light <cmd>` (runs immediately without waiting for a slot, `OMP_NUM_THREADS=2`, `nice -n 5`). <!-- cite: tools/heavy.sh:5-6,18-22 -->
- **Exclusive Profiling (`--exclusive`):** Formal performance benchmarks, GPU profiling, and timings must run under `tools/heavy.sh --exclusive <cmd>`. It waits until all 4 slots are simultaneously free and blocks new heavy jobs. Timings measured while other jobs run are invalid. <!-- cite: tools/heavy.sh:7-10,32-52, docs/team/HIERARCHY.md:39 -->
- **Build Concurrency:** Restrict build concurrency to `-j4` at most. <!-- cite: docs/team/PLAN.md:12 -->
- **Measured Stepping Speeds:**
  - Dry dycore on CPU (12-core x86-64, AVX2, OMP=3): ~30–50 ns/cell/step in FP32. <!-- cite: docs/STATUS.md:18-35 -->
  - Dry dycore on GPU (NVIDIA RTX 3050): ~3–8 ns/cell/step. <!-- cite: tools/gpu/ -->
  - Real-data simulations with active physics suite (sfclayrev + Noah LSM + YSU PBL + Kain-Fritsch + RRTMG + WSM6): ~4000–5000 ns/cell/step on CPU.
