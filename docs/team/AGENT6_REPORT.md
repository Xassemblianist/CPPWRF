# Agent 6 Report: NetCDF Output, Restarts, and Regression System

- **Owner**: Agent 6 (Infrastructure Lead: NetCDF I/O, Restarts, Regression System)
- **Task Reference**: `docs/team/AGENT6_NETCDF_REGRESSION.md`
- **Date**: 2026-10-13
- **Git Commit HEAD**: `9e5586e6dbd1d2011a6cfbe2b934fca9de2ddcb9`
- **Status**: Completed (Parts A, B, and C)

---

## Machine Notices & Rule Compliance (HIERARCHY.md)

1. **Warning Remediation**:
   - As noted in `docs/team/HIERARCHY.md` (warning on `build-reg-*.log` in repo root), all build and run logs have been relocated to `tools/regression/logs/`.
   - Regression run outputs are confined to `tools/regression/runs/`.
   - Unit test outputs are placed in `tests/io_netcdf/test_out/`.
   - The repository root contains **zero** generated logs, binary files, object files, or module files.
2. **Heavy Job & Timing Protocol**:
   - All compilation, CTest executions, and model runs in `tools/regression/run.sh` are wrapped with `tools/heavy.sh`.
   - All model execution and performance measurements strictly use `tools/heavy.sh --exclusive` to ensure valid timing isolation.
   - Multi-threading parameters conform to `OMP_NUM_THREADS=4` and `OMP_WAIT_POLICY=passive`.
3. **Git Cleanliness**:
   - Zero git write commands (`add`, `commit`, `stash`, `checkout`, `reset`, etc.) or `sudo` invoked. Only Agent 0 commits code.

---

## Part A: WRF-Format NetCDF Output (`src/io_netcdf/`)

### 1. Architecture & Implementation
A modular NetCDF-C library target `cppwrf_io_netcdf` was created under `src/io_netcdf/`:
- [`netcdf_common.hpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/netcdf_common.hpp):
  - Defines `DType` (`F32`, `F64`), `VarView` (zero-copy view into host memory buffers with strides and stagger flags), and `WrfoutMeta` (simulation spatial and projection metadata).
  - Implements `NC_CHECK(call)` macro with descriptive error messages and source location tracking.
- [`wrfout_writer.hpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/wrfout_writer.hpp) & [`wrfout_writer.cpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/wrfout_writer.cpp):
  - Full WRF-compliant NetCDF dataset creator and frame appender.
  - Implements 64-bit offset format (`NC_64BIT_OFFSET`) supporting multi-gigabyte outputs.
  - Unlimited `Time` dimension allowing multiple snapshot frames in a single file (`wrfout_d01_<date>`).
  - Strict WRF dimension schema: `Time`, `DateStrLen` (19), `west_east`, `south_north`, `bottom_top`, `west_east_stag`, `south_north_stag`, `bottom_top_stag`.
  - Comprehensive variables list (all 29 requested variables):
    - Prognostic / state: `U`, `V`, `W`, `PH`, `PHB`, `T` ($\theta - 300\text{ K}$), `MU`, `MUB`, `P`, `PB`, `QVAPOR`.
    - Coordinate / vertical: `ZNU`, `ZNW`, `C1H`, `C2H`, `C1F`, `C2F`, `C3H`, `C4H`, `C3F`, `C4F`, `P_TOP`, `RDX`, `RDY`.
    - Horizontal coordinates & time: `Times`, `XLAT`, `XLONG`, `HGT`, `XTIME`.
  - Required WRF metadata attributes: `FieldType = 104`, `MemoryOrder` (`XYZ`, `XY `, `Z  `, `0  `), `stagger` (`X`, `Y`, `Z`, `""`), `description`, `units`, `coordinates`.
  - Global attributes essential for `wrf-python` and `ncview`: `TITLE`, `START_DATE`, `SIMULATION_START_DATE`, `WEST-EAST_GRID_DIMENSION`, `SOUTH-NORTH_GRID_DIMENSION`, `BOTTOM-TOP_GRID_DIMENSION`, `DX`, `DY`, `DT`, `GRIDTYPE`, `DIFF_OPT`, `KM_OPT`, `KHDIF`, `KVDIF`, `MAP_PROJ`, `CEN_LAT`, `TRUELAT1`, `TRUELAT2`, `STAND_LON`, `MOAD_CEN_LAT`, `HYBRID_OPT`, `ETAC`, and patch indices.
- [`CMakeLists.txt`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/CMakeLists.txt):
  - Discovers NetCDF-C using CMake config, pkg-config, or standard include/library paths.
  - Builds `cppwrf_io_netcdf` as a modern CMake target.

### 2. Verification & Testing
- **C++ Unit Test** ([`tests/io_netcdf/test_wrfout_writer.cpp`](file:///home/ostriquetrum/CPPWRF/tests/io_netcdf/test_wrfout_writer.cpp)):
  - Synthesizes 3D test grids ($nx=10, ny=10, nz=8$).
  - Writes two time frames ($t = 0\text{ s}$ and $t = 300\text{ s}$) to `tests/io_netcdf/test_out/wrfout_d01_test.nc`.
  - Validates file creation, dimension attributes, frame counts, variable attributes (`FieldType=104`), and floating point data integrity via the NetCDF C API.
  - Result: **PASS**.
- **Python Verification** ([`tests/io_netcdf/test_wrfout_python.py`](file:///home/ostriquetrum/CPPWRF/tests/io_netcdf/test_wrfout_python.py)):
  - Reads output file via `netCDF4.Dataset`.
  - Verifies round-trip integrity across all 29 variables, dimension shapes, and global attributes.
  - Result: **PASS**.
- **`wrf-python` Investigation**:
  - `wrf-python` installation was attempted via `pip install --user wrf-python`.
  - Installation fails under Python 3.13.5 (Debian trixie) during setup:
    ```text
    ModuleNotFoundError: No module named 'numpy.distutils'
    ```
    **Root Cause**: `wrf-python 1.3.4.1` imports `numpy.distutils` in `setup.py`. `numpy.distutils` was deprecated in NumPy 1.23 and removed completely in NumPy 2.0+ (the system environment provides NumPy 2.2.4).
  - **Diagnostic Verification**: In lieu of `wrf-python`'s C-accelerated extensions, we validated the exact mathematical transformations used by `wrf.getvar`:
    - Potential temperature: $\theta = T + 300.0\text{ K}$
    - Full pressure: $P = (P_{pert} + P_B) / 100.0\text{ hPa}$
    - Geopotential height: $Z = 0.5 \times (PH + PH_B) / 9.81\text{ m}$
    All computations matched expected float32 precision (`rtol=1e-5, atol=1e-5`).
- **Header Comparison (`ncdump -h`)**:
  - Script [`tests/io_netcdf/compare_ncdump.py`](file:///home/ostriquetrum/CPPWRF/tests/io_netcdf/compare_ncdump.py) checks structural conformity.
  - Dimensions: All 8 standard WRF dimensions match 1:1.
  - Variables: All 29 standard WRF diagnostic/prognostic variables present with exact `FieldType = 104`.
  - Global attributes: Matches WRF v4.6.1 registry specifications (`Registry.EM_COMMON`, `registry.hyb_coord`).

---

## Part B: Restart Files (`src/io_netcdf/restart.*`)

### 1. Architecture & Design
- Designed restart format in [`restart.hpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/restart.hpp) and [`restart.cpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/restart.cpp):
  - Preserves complete dycore state for bitwise continuation:
    - Prognostic velocities and thermodynamic fields: `u_1`, `u_2`, `v_1`, `v_2`, `w_1`, `w_2`, `t_1`, `t_2`, `ph_1`, `ph_2`, `mu_1`, `mu_2`.
    - Scalar and moisture fields: `q1_*`, `q2_*` dynamically registered from simulation state.
    - Coordinate & grid parameters: $nx, ny, nz, dx, dy, dt, ztop, p\_top, rdx, rdy, step, time$.
  - Precision flexibility: Full support for both single precision (`NC_FLOAT`) and double precision (`NC_DOUBLE`).
  - Classic NetCDF format portability: Uses `NC_INT` attributes for step indexing to guarantee compatibility with all NetCDF-3/4 and 64-bit offset readers.

### 2. Acceptance Test
- Executed standalone acceptance test [`tests/io_netcdf/test_restart.cpp`](file:///home/ostriquetrum/CPPWRF/tests/io_netcdf/test_restart.cpp):
  - Fills arrays with arbitrary high-entropy floating-point data patterns.
  - Invokes `write_restart(...)` -> zero-fills memory buffers -> invokes `read_restart(...)`.
  - Compares original and restored arrays byte-for-byte using `memcmp == 0`.
  - Execution Output:
    ```text
    Running test_restart acceptance test...
    Testing restart bitwise identity for FP32...
    PASS: Bitwise identity verified for FP32 (memcmp == 0)
    Testing restart bitwise identity for FP64...
    PASS: Bitwise identity verified for FP64 (memcmp == 0)
    RESTART BITWISE TEST PASSED!
    ```
  - Result: **PASS** (100% bitwise identity verified).

---

## Part C: Regression System (`tools/regression/`)

### 1. Architecture & Tools Implemented
A complete, autonomous regression testing harness has been established in `tools/regression/`:
- **Regression Runner** ([`tools/regression/run.sh`](file:///home/ostriquetrum/CPPWRF/tools/regression/run.sh)):
  - Synopsis: `tools/regression/run.sh [--quick|--full] [--gpu]`
  - Builds all required configurations in dedicated directories (`build-reg-cpu`, `build-reg-cpu-dp`, `build-reg-cuda`, `build-reg-cuda-dp`) using `tools/heavy.sh cmake ...` with `-j4`.
  - Executes `ctest` within each configuration under `tools/heavy.sh`.
  - Executes simulation test cases (`rest_state`, `straka`, `warm_bubble`, `igw`) on all available backends and precisions using `tools/heavy.sh --exclusive`.
  - Automatically isolates runs in `tools/regression/runs/<case>_<backend>_<prec>/` and logs to `tools/regression/logs/`.
  - Tracks performance metrics (`ns/cell/step`) in `tools/regression/perf_history.csv` and flags slowdowns greater than 10% relative to previous runs.
  - Verifies run physics with `tests/verify/verify.py check`.
  - Validates numerical reproducibility against stored golden references via `compare_golden.py`.
  - Performs CPU vs. GPU comparisons via `tests/verify/verify.py compare` when GPU is active.
  - Writes Markdown summary to `tools/regression/last_report.md`.
- **Golden Comparison Engine** ([`tools/regression/compare_golden.py`](file:///home/ostriquetrum/CPPWRF/tools/regression/compare_golden.py)):
  - Compares diagnostics in `diag.csv` (excluding non-deterministic `wall_s`).
  - Compares final snapshot binary and metadata arrays (`snap_*.bin`, `snap_*.json`).
  - Precision tolerances: FP64 (`rtol=1e-12, atol=1e-12`), FP32 (`rtol=1e-5, atol=1e-5`).
- **Golden Reference Updater** ([`tools/regression/update_golden.sh`](file:///home/ostriquetrum/CPPWRF/tools/regression/update_golden.sh)):
  - Refreshes reference baselines under `tools/regression/golden/<case>_<backend>_<prec>/` on explicit request.
  - Initial golden dataset captured at commit `9e5586e6dbd1d2011a6cfbe2b934fca9de2ddcb9`.

### 2. First Regression Run Summary
Results from `tools/regression/last_report.md`:

#### Build and CTest Status
| Configuration | Build | CTest |
|---|---|---|
| `build-reg-cpu` | PASS | PASS |
| `build-reg-cpu-dp` | PASS | PASS |

#### Simulation Cases & Golden Reference Comparison
| Case | Backend | Precision | Execution | Verifier | Golden Match | Performance (ns/cell/step) |
|---|---|---|---|---|---|---|
| `rest_state` | cpu | fp32 | PASS | PASS | **PASS** | 1149.40 |
| `straka` | cpu | fp32 | PASS | FAIL* | **PASS** | 739.05 |
| `warm_bubble` | cpu | fp32 | PASS | PASS | **PASS** | 773.71 |
| `igw` | cpu | fp32 | PASS | PASS | **PASS** | 1139.33 |
| `rest_state` | cpu | fp64 | PASS | PASS | **PASS** | 1560.61 |
| `straka` | cpu | fp64 | PASS | FAIL* | **PASS** | 995.21 |
| `warm_bubble` | cpu | fp64 | PASS | PASS | **PASS** | 885.92 |
| `igw` | cpu | fp64 | PASS | PASS | **PASS** | 1055.49 |

> \* **Note on `straka` in `--quick` mode**:
> In `--quick` mode, simulation time is truncated ($t = 2\text{ s}$). `tests/verify/verify.py check straka` specifically validates the cold pool front position and minimum $\theta'$ perturbation at $t = 900\text{ s}$ (`abs(m["time"] - 900.0) <= 1.0`). Finite numbers, mass/energy conservation, and left-right symmetry passed completely, and the simulation matched the golden baseline with 100% numerical identity. In `--full` mode, `straka` runs to $t = 900\text{ s}$ where the front verification passes.

---

## Integration Snippets for Agent 0

### 1. Top-Level `CMakeLists.txt`
Add the following block to top-level `CMakeLists.txt`:
```cmake
# NetCDF-C library (Agent 6)
find_package(netCDF CONFIG QUIET)
if(NOT TARGET netCDF::netcdf)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(NETCDF netcdf)
  endif()
endif()
find_path(NETCDF_INCDIR netcdf.h HINTS /usr/include /usr/local/include)
if(TARGET netCDF::netcdf OR NETCDF_FOUND OR NETCDF_INCDIR)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/io_netcdf)
    add_subdirectory(src/io_netcdf)
    target_link_libraries(cppwrf_core PUBLIC cppwrf_io_netcdf)
    target_compile_definitions(cppwrf_core PUBLIC CPPWRF_HAVE_NETCDF)
  endif()
  if(EXISTS ${CMAKE_SOURCE_DIR}/tests/io_netcdf)
    add_subdirectory(tests/io_netcdf)
  endif()
endif()
```

### 2. `src/main.cpp` Integration Snippet
```cpp
#ifdef CPPWRF_HAVE_NETCDF
#include "io_netcdf/wrfout_writer.hpp"
#include "io_netcdf/restart.hpp"
#endif

// In run():
#ifdef CPPWRF_HAVE_NETCDF
  const bool write_netcdf = cfg.get_bool("output.netcdf", true);
  std::unique_ptr<io_netcdf::WrfoutWriter> wrfout;
  if (write_netcdf) {
    io_netcdf::WrfoutMeta wm;
    wm.nx = g.nx; wm.ny = g.ny; wm.nz = g.nz;
    wm.dx = g.dx; wm.dy = g.dy; wm.dt = g.dt;
    wm.ztop = g.ztop; wm.p_top = g.p_top;
    wm.diff_opt = int(lp.khdif > 0 ? 1 : 0);
    wm.khdif = double(lp.khdif); wm.kvdif = double(lp.kvdif);
    wm.start_date = cfg.get_string("time.start_date", "0001-01-01_00:00:00");
    wm.simulation_start_date = wm.start_date;
    wrfout = std::make_unique<io_netcdf::WrfoutWriter>(out_dir, wm);
  }
#endif

// Inside do_snap(n):
#ifdef CPPWRF_HAVE_NETCDF
  if (wrfout) {
    std::vector<io_netcdf::VarView> nc_vars;
    for (const auto& v : vars) {
      nc_vars.emplace_back(v.name, v.stagger, v.nx, v.ny, v.nz, v.data,
                           v.dtype == io::DType::F64 ? io_netcdf::DType::F64 : io_netcdf::DType::F32,
                           v.sj, v.sk);
    }
    wrfout->write_frame(n, n * g.dt, nc_vars);
  }
#endif
```

### 3. Bitwise Continuation Test Plan ("600 s = 300 s + restart + 300 s")
Once Agent 0 wires `read_restart` and `write_restart` into `src/main.cpp`:
1. Run continuous simulation to 600 s:
   ```bash
   tools/heavy.sh ./build-cpu-dp/cppwrf configs/straka.ini --set run.run_time=600 --set output.dir=out/straka_cont
   ```
2. Run split segment 1 (0 to 300 s with restart write at 300 s):
   ```bash
   tools/heavy.sh ./build-cpu-dp/cppwrf configs/straka.ini --set run.run_time=300 --set output.dir=out/straka_p1 --set output.restart_interval=300
   ```
3. Run split segment 2 (restart from 300 s, run to 600 s):
   ```bash
   tools/heavy.sh ./build-cpu-dp/cppwrf configs/straka.ini --set run.run_time=600 --set run.restart_file=out/straka_p1/restart_000300.nc --set output.dir=out/straka_p2
   ```
4. Verify bitwise identity:
   ```bash
   cmp out/straka_cont/snap_000600.bin out/straka_p2/snap_000600.bin
   ```
   Must exit with code 0 (`cmp` produces no output on bitwise identity).

---

## Task T6.2: NetCDF WRF Output & Restart Integration Report

- **Date**: 2026-10-13
- **Scope**: Integration into `src/main.cpp`, Top-Level `CMakeLists.txt`, `src/io_netcdf/`, and `tools/regression/` (under limited ownership exception).
- **Status**: Completed and fully verified.

### 1. Architectural Changes & Scope Summary

1. **Top-Level `CMakeLists.txt`**:
   - Added automatic NetCDF-C discovery (`find_package(netCDF CONFIG QUIET)` and `pkg_check_modules(NETCDF netcdf)`).
   - Conditionally integrates `src/io_netcdf` and links `cppwrf_io_netcdf` into `cppwrf_core`.
   - Defines `CPPWRF_HAVE_NETCDF` across the core and driver when NetCDF is detected.
   - Adds `tests/io_netcdf` to CTest suite.
2. **`src/io_netcdf/`**:
   - [`wrfout_writer.cpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/wrfout_writer.cpp): Added output support for moist species (`QVAPOR`, `QCLOUD`, `QRAIN`, with aliases `QC` and `QR`) and cumulative grid-scale precipitation (`RAINNC`). Default start date set to `2000-01-01_00:00:00`.
   - [`restart.cpp`](file:///home/ostriquetrum/CPPWRF/src/io_netcdf/restart.cpp): Fixed 2D domain dimension handling (`dim_sn` vs `dim_sn_stag`) preventing memory overflow during restart read; correctly handled stride logic for zero-stride 2D axes; allowed skipping optional/missing variables gracefully.
3. **`src/main.cpp` Integration**:
   - Config option `output.format`: Supports `native` (default), `wrfout`, and `both`. Validates option and raises descriptive runtime errors on unsupported values or missing NetCDF support.
   - Base state variables (`PB`, `PHB`, `MUB`) downloaded and broadcast to WRF NetCDF outputs.
   - Config option `restart.interval`: Triggers NetCDF restart generation (`restart_%06ld.nc`) every specified seconds.
   - Config option `run.restart_file`: Restores prognostic state (`u_1, u_2, v_1, v_2, w_1, w_2, t_1, t_2, ph_1, ph_2, mu_1, mu_2, p, al`), scalar/moisture fields, `h_diabatic`, and microphysics state (`rainnc`, `rainncv`). Halos are recomputed via `fill_halo`.
   - Preserves pre-restart diagnostic rows in `diag.csv` when continuing in an existing directory.
4. **`tools/regression/run.sh`**:
   - Added `squall2d` to regression suite with quick-mode parameters (`run.run_time=12`, `out_interval=2`, `diag_interval=1`).
   - Added auto GPU detection (`HAS_GPU=1` if `nvcc` and `nvidia-smi` available, with `--no-gpu` / `--gpu` flag support).
   - Runs all builds and simulation executions under `tools/heavy.sh` (`--exclusive` for model timings).

---

### 2. Output Format Verification (`output.format`)

Three output format modes were verified on `squall2d`:
1. `output.format = native` (default):
   - Generates native binary snapshots `snap_*.bin` and `snap_*.json`. No NetCDF files generated.
2. `output.format = wrfout`:
   - Generates only WRF NetCDF file `wrfout_d01_2000-01-01_00:00:00`. No `snap_*.bin` files created.
3. `output.format = both`:
   - Command:
     ```bash
     tools/heavy.sh --exclusive build-cpu/cppwrf configs/squall2d.ini --set=run.run_time=12 --set=output.dir=tests/io_netcdf/test_out/squall_both --set=output.format=both --set=output.out_interval=12
     ```
   - Both `snap_000000.bin`, `snap_000002.bin`, and `wrfout_d01_2000-01-01_00:00:00` are written.
   - NetCDF inspection via `ncdump -h` confirms presence and shapes of:
     - `float QVAPOR(Time, bottom_top, south_north, west_east)`
     - `float QCLOUD(Time, bottom_top, south_north, west_east)`
     - `float QRAIN(Time, bottom_top, south_north, west_east)`
     - `float RAINNC(Time, south_north, west_east)`
     - All 29 WRF standard variables, grid dimensions, and global attributes.

---

### 3. Bitwise Restart Acceptance Test Results

Bitwise acceptance was verified across all 8 required configurations using the acceptance suite [`tests/io_netcdf/run_acceptance_restart.py`](file:///home/ostriquetrum/CPPWRF/tests/io_netcdf/run_acceptance_restart.py).
Each configuration executes:
- Continuous run: 0 to 600 s.
- Segment 1: 0 to 300 s (generates `restart_*.nc` at 300 s).
- Segment 2: Restarts from 300 s NetCDF file, runs to 600 s.
- Bitwise verification: `cmp` on final snapshot `.bin` files (byte-for-byte exit code 0) and exact match of `diag.csv` columns 1-10 for steps >= 300.

| Test Case | Build Target | Precision | Backend | Final Snapshot Size | `cmp` Bitwise Match | `diag.csv` Cols 1-10 | Status |
|---|---|---|---|---|---|---|---|
| **straka** | `build-cpu` | FP32 | CPU (OpenMP) | 1,188,096 B | Identical (exit code 0) | 31/31 rows identical | **PASS** |
| **straka** | `build-cpu-dp` | FP64 | CPU (OpenMP) | 2,376,192 B | Identical (exit code 0) | 31/31 rows identical | **PASS** |
| **straka** | `build-cuda` | FP32 | CUDA | 1,188,096 B | Identical (exit code 0) | 31/31 rows identical | **PASS** |
| **straka** | `build-cuda-dp` | FP64 | CUDA | 2,376,192 B | Identical (exit code 0) | 31/31 rows identical | **PASS** |
| **squall2d** | `build-cpu` | FP32 | CPU (OpenMP) | 252,640 B | Identical (exit code 0) | 6/6 rows identical | **PASS** |
| **squall2d** | `build-cpu-dp` | FP64 | CPU (OpenMP) | 505,280 B | Identical (exit code 0) | 6/6 rows identical | **PASS** |
| **squall2d** | `build-cuda` | FP32 | CUDA | 252,640 B | Identical (exit code 0) | 6/6 rows identical | **PASS** |
| **squall2d** | `build-cuda-dp` | FP64 | CUDA | 505,280 B | Identical (exit code 0) | 6/6 rows identical | **PASS** |

Execution command:
```bash
python3 tests/io_netcdf/run_acceptance_restart.py
```
Output:
```text
==================================================
ALL RESTART ACCEPTANCE TESTS PASSED BITWISE!
==================================================
```

---

### 4. Regression Runner & CTest Suite

#### 4.1 CTest Results (10/10 PASS across all 4 builds)
Executed across all configurations:
- `build-cpu`: 100% tests passed (10/10)
- `build-cpu-dp`: 100% tests passed (10/10)
- `build-cuda`: 100% tests passed (10/10)
- `build-cuda-dp`: 100% tests passed (10/10)

Unit tests include:
- `test_wrfout_writer`: Validates NetCDF WRF output creation and frames.
- `test_restart`: In-memory byte-for-byte validation of restart writing and reading.
- `test_wrfout_python`: `netCDF4` verification of all 29 fields and metadata.

#### 4.2 Squall2D Physics & Budget Verification via `tests/verify/verify.py`
Running `python3 tests/verify/verify.py check squall2d <dir>` validates:
- **FP64 Symmetry**:
  ```text
  PASS  symmetry  th' 2.86e-14  u 1.52e-13  w 1.09e-12  (tol 1e-06, t=12s)
  ```
- **Water Budget Conservation**:
  ```text
  PASS  moist_nonnegative  min over ['qv', 'qc', 'qr']: 0.000e+00
  PASS  water_budget       (water + rain - W0)/W0 = -1.66e-16 (tol 1e-03, mp_physics=1)
  ```
- **Dry Mass Conservation**:
  ```text
  PASS  dry_mass_drift     rel drift 0.000e+00 (tol 1e-10)
  ```

#### 4.3 Full Performance & Regression Sweep
All 20 simulation runs (5 cases $\times$ 4 targets) execute without crashes.
CPU vs GPU agreement confirms bitwise/tolerance parity:
- `squall2d`: PASS in both FP32 and FP64.
- `straka`, `warm_bubble`, `igw`: PASS in both FP32 and FP64.

---

## Conclusion & Next Steps
Agent 6 has completed task T6.2 in full. All code, restart acceptance tests, WRF format integrations, regression enhancements, and documentation are in place and verified under the multi-tenant shared-machine guidelines. Ready for review and integration by Agent 0. Execution stopped.
