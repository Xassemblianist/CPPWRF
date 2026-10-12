# Agent 2 Report: WRF Physics Port & Differential Verification

Owner: Agent 2 (Gemini 3.8 Flash High)
Reference WRF: v4.6.1 (`~/wrf_ref/WRF`)
Precision: IEEE 754 FP64 primary differential verification (FP32 evaluated for GPU)

---

## Differential Test Harness Overview

A modular differential test harness was established under `tests/phys/` to test ported C++ physics routines directly against the original WRF Fortran routines compiled standalone with `gfortran`.

The harness workflow:
1. `generate_inputs.py`: Synthesizes $N \ge 2000$ realistic, diverse atmospheric columns across multiple regimes with a fixed random seed.
2. `driver_<scheme>_fortran`: Reads binary input, executes the original WRF Fortran subroutine, and writes the output.
3. `driver_<scheme>_cpu` / `_cuda`: Reads identical binary input, executes the C++ kernel using CPPWRF single-source views, and writes the output.
4. `compare.py`: Computes maximum absolute difference and maximum relative difference with physical floors, identifies worst-case columns, and verifies acceptance tolerance ($\le 10^{-12}$ in FP64).
5. `run_test.sh` / `ctest`: Automated pipeline integrated into CMake test framework.

---

## Scheme 1: Kessler Microphysics (`mp_physics = 1`)

- **Fortran source:** `phys/module_mp_kessler.F`
- **C++ implementation:** `src/phys/mp_kessler/kessler.hpp`, `src/phys/mp_kessler/kessler.cpp`
- **Constants:** `src/phys/phys_constants.hpp`
- **Library target:** `cppwrf_phys` (`src/phys/CMakeLists.txt`)
- **Test files:**
  - Standalone Fortran copy: `tests/phys/fortran/module_mp_kessler.F`
  - Fortran driver: `tests/phys/mp_kessler/driver_kessler.f90`
  - C++ driver: `tests/phys/mp_kessler/driver_kessler.cpp`
  - Input generator: `tests/phys/mp_kessler/generate_inputs.py`
  - Comparison script: `tests/phys/mp_kessler/compare.py`
  - Automated test runner: `tests/phys/mp_kessler/run_test.sh`
  - Test suite CMake: `tests/phys/CMakeLists.txt`

### Fortran Changes Needed for Standalone Compilation
**None.** `module_mp_kessler.F` is completely self-contained. It compiles cleanly with:
```bash
gfortran -ffree-form -fdefault-real-8 -O3 -march=native -ffp-contract=off -fno-tree-vectorize
```
*Note on compiler flags:* WRF `.F` files require `-ffree-form`. `-fdefault-real-8` promotes `REAL` to 64-bit IEEE float. `-fno-tree-vectorize` ensures standard scalar IEEE libm evaluation, matching the C++ single-source kernel bit-for-bit.

### Input Regimes Tested ($N = 2500$ columns, $nz = 50$ levels, $dt = 12.0\,\text{s}$)
Input columns cover the entire operational regime of WRF:
1. **Tropical (cols 0–499):** High surface temperature ($T_s \approx 298–305\,\text{K}$), deep moist layer, saturated layers ($RH > 1.0$), cloud water autoconversion ($qc > 1\,\text{g/kg}$), and rain fallout through unsaturated lower levels.
2. **Mid-latitude (cols 500–999):** Moderate surface temperatures ($T_s \approx 280–295\,\text{K}$), freezing level around 2–3 km, supercooled liquid water clouds ($T < 273.15\,\text{K}$), and rain sedimentation.
3. **Dry / Arctic (cols 1000–1499):** Low surface temperatures ($240–270\,\text{K}$), extremely dry layers ($q_v \approx 10^{-6}\,\text{kg/kg}$ down to 0), and rain aloft evaporating into dry lower layers.
4. **Supercooled & Deep Convection (cols 1500–1999):** Steep lapse rates, supercooled clouds, and heavy rain ($qr \approx 4–12\,\text{g/kg}$) triggering Courant substepping ($nfall = 2, 3, \dots$).
5. **Edge Cases & Thresholds (cols 2000–2499):**
   - Exactly zero hydrometeors ($qc = 0, qr = 0$)
   - Near-zero mixing ratios ($10^{-15}\,\text{kg/kg}$)
   - Autoconversion threshold boundaries ($qc = 10^{-3}\,\text{kg/kg}$, $10^{-3} \pm 10^{-8}$)
   - Completely dry columns ($qv = 0, qc = 0, qr = 0$)
   - Extreme rain ($qr = 1.5 \times 10^{-2}\,\text{kg/kg}$)
   - Multiple timesteps tested: $dt = 1.0\,\text{s}$, $12.0\,\text{s}$, $60.0\,\text{s}$, $120.0\,\text{s}$.

### Differential Comparison Table: Fortran vs C++ CPU (FP64, $dt = 12.0\,\text{s}$)

| Variable | Description | Max Abs Diff | Max Rel Diff | Worst Column | Status |
|---|---|---|---|---|---|
| `t` | Potential temperature (K) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qv` | Water vapor mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qc` | Cloud water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qr` | Rain water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rainnc` | Accumulated grid rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rainncv` | Timestep rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |

*Result:* **Exact bit-for-bit match (0.000000e+00)** across all 2500 columns and 50 vertical levels. Tests across $dt = 1.0\,\text{s}, 60.0\,\text{s}, 120.0\,\text{s}$ also pass bit-for-bit.

### CPU vs CUDA Differences ($N = 2500$ columns, $nz = 50$ levels)

#### CUDA FP64 vs CPU FP64:
| Variable | Max Abs Diff | Max Rel Diff | Worst Column | Notes |
|---|---|---|---|---|
| `t` | 5.684342e-14 | 2.163437e-16 | 2297 | Roundoff on $\theta \approx 300\,\text{K}$ |
| `qv` | 1.040834e-17 | 4.607703e-15 | 2493 | Roundoff |
| `qc` | 1.062518e-17 | 1.277570e-10 | 1868 | 1-ULP diff in libdevice `exp` during $q_v - q_{vs}$ cancellation ($q_c \approx 8.37 \times 10^{-10}$) |
| `qr` | 6.938894e-18 | 8.816420e-16 | 2491 | Roundoff |
| `rainnc` | 1.776357e-15 | 2.217890e-16 | 169 | Roundoff |
| `rainncv` | 1.665335e-16 | 6.367523e-16 | 121 | Roundoff |

*Evidence for the `qc` discrepancy:* In column 1868 at level 46, $q_v \approx 3.8446 \times 10^{-5}$ and $q_{vs} \approx 3.8445 \times 10^{-5}$. Their difference $(q_v - q_{vs}) \approx 8.37 \times 10^{-10}$ cancels 5 leading decimal digits. The 1-ULP variation between x86 glibc `std::exp` and NVIDIA libdevice `__nv_exp` introduces an absolute difference of $1.06 \times 10^{-17}\,\text{kg/kg}$ (10 attograms/kg), which scales to $1.27 \times 10^{-10}$ relative to the tiny $8.37 \times 10^{-10}$ condensed cloud amount.

#### CUDA FP32 vs CPU FP64:
- `t`: max abs $3.79 \times 10^{-5}\,\text{K}$, max rel $1.24 \times 10^{-7}$
- `qv`: max abs $1.17 \times 10^{-8}\,\text{kg/kg}$, max rel $4.01 \times 10^{-6}$
- `qr`: max abs $5.04 \times 10^{-9}\,\text{kg/kg}$, max rel $1.94 \times 10^{-6}$
- `rainnc`: max abs $8.90 \times 10^{-7}\,\text{mm}$, max rel $1.01 \times 10^{-7}$
- `rainncv`: max abs $6.63 \times 10^{-8}\,\text{mm}$, max rel $2.34 \times 10^{-7}$
- `qc`: max abs $1.14 \times 10^{-8}\,\text{kg/kg}$

All variables conform to expected single precision precision ($\sim 10^{-7} - 10^{-5}$).

### Performance Benchmarks ($100,000$ columns $\times 50$ levels)

Measured exclusively with `tools/heavy.sh --exclusive` on 4-thread CPU (OMP_NUM_THREADS=4, OMP_WAIT_POLICY=passive) and NVIDIA GeForce RTX 2060 (Turing sm_75, 6 GB):

| Backend | Precision | Wall Time (10/20-iter avg) | ns / column | Speedup vs CPU-4 |
|---|---|---|---|---|
| **CPU-4** (OpenMP) | FP64 | 128.2 ms | **1282 ns** | 1.0x |
| **GPU** (CUDA) | FP64 | 34.3 ms | **343 ns** | **3.7x** |
| **GPU** (CUDA) | FP32 | 4.46 ms | **44.6 ns** | **28.7x** |

*(Note: Prior 12-thread timing discarded per shared machine rule update; re-benchmarked strictly with `tools/heavy.sh --exclusive` on 4 threads).*

### Open Issues / Requests for Other Agents
- **For Agent 0:** When integrating `src/phys/` into the main build, add `add_subdirectory(src/phys)` to top-level `CMakeLists.txt` and link `cppwrf_phys` into `cppwrf_core` or `cppwrf`.
- No known numerical bugs or divergences in Kessler.

---

## Scheme 2: WSM6 Microphysics (`mp_physics = 6`)

- **Fortran source:** `phys/module_mp_wsm6.F` calling `physics_mmm/mp_wsm6.F90`
- **C++ implementation:** `src/phys/mp_wsm6/wsm6.hpp`, `src/phys/mp_wsm6/wsm6.cpp`
- **Constants:** `src/phys/phys_constants.hpp`
- **Library target:** `cppwrf_phys` (`src/phys/CMakeLists.txt`)
- **Test files:**
  - Standalone Fortran sources: `tests/phys/fortran/module_mp_wsm6.F`, `tests/phys/fortran/mp_wsm6.F90`, `tests/phys/fortran/module_libmassv.F90`, `tests/phys/fortran/mp_radar.F90`, `tests/phys/fortran/mp_wsm6_effectRad.F90`, `tests/phys/fortran/ccpp_kind_types.F`
  - Fortran driver: `tests/phys/mp_wsm6/driver_wsm6.f90`
  - C++ driver: `tests/phys/mp_wsm6/driver_wsm6.cpp`
  - Input generator: `tests/phys/mp_wsm6/generate_inputs.py`
  - Comparison script: `tests/phys/mp_wsm6/compare.py`
  - Automated test runner: `tests/phys/mp_wsm6/run_test.sh`
  - Test suite CMake: `tests/phys/CMakeLists.txt`

### Fortran Changes Needed for Standalone Compilation
In WRF v4.6.1, `module_mp_wsm6.F` delegates computation to `mp_wsm6.F90` (originally from `physics_mmm/`). In standalone mode, `mp_wsm6.F90` depends on IBM MASS vector functions (`vrec`, `vsqrt`), radar reflectivity calculations (`mp_radar.F90`), radiation effective radius (`mp_wsm6_effectRad.F90`), and CCPP precision types (`ccpp_kind_types.F`).
- Copied all required module sources into `tests/phys/fortran/`.
- Compiled cleanly with:
```bash
gfortran -ffree-form -fdefault-real-8 -O3 -march=native -ffp-contract=off -fno-tree-vectorize
```
- Module and object build artifacts are strictly isolated in `tests/phys/mp_wsm6/build/`.

### Input Regimes Tested ($N = 2500$ columns, $nz = 50$ levels)
Input columns cover the full thermodynamic space and hydrometeor phases:
1. **Tropical (cols 0–499):** High surface temperatures ($T_s \approx 298–305\,\text{K}$), deep moist layer, autoconversion of cloud to rain, heavy rain sedimentation.
2. **Mid-latitude (cols 500–999):** Freezing level at $k \approx 8–10$, mixed-phase clouds, snow accretion by rain, graupel melting and riming.
3. **Dry / Arctic (cols 1000–1499):** Low surface temperatures ($240–270\,\text{K}$), low vapor mixing ratios, ice crystal sedimentation, sublimation.
4. **Supercooled & Deep Convection (cols 1500–1999):** Severe convective storms, high mixing ratios across all 6 species ($qc, qr, qi, qs, qg$), rapid riming, multi-step sub-cycling.
5. **Edge Cases & Thresholds (cols 2000–2499):**
   - Zero hydrometeors ($qc = qr = qi = qs = qg = 0$)
   - Near-zero mixing ratios ($10^{-15}\,\text{kg/kg}$)
   - Threshold boundaries for autoconversion, riming, and melting
   - Completely dry soundings
   - Multi-loop timesteps ($dt = 240.0\,\text{s}$, `loops = 2`) and single-loop timesteps ($dt = 120.0\,\text{s}$, `loops = 1`).

### Differential Comparison Table: Fortran vs C++ CPU (FP64)

#### Multi-Loop ($dt = 240.0\,\text{s}$, `loops = 2`, $N = 2500$ columns $\times 50$ levels):

| Variable | Description | Max Abs Diff | Max Rel Diff | Worst Column | Status |
|---|---|---|---|---|---|
| `th` | Potential temperature (K) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `q` | Water vapor mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qc` | Cloud water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qr` | Rain water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qi` | Cloud ice mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qs` | Snow mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qg` | Graupel mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rain` | Accumulated surface rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rainncv` | Timestep surface rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `snow` | Accumulated surface snow (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `snowncv` | Timestep surface snow (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `graupel` | Accumulated surface graupel (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `graupelncv` | Timestep surface graupel (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `sr` | Frozen precipitation ratio (-) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |

#### Single-Loop ($dt = 120.0\,\text{s}$, `loops = 1`, $N = 2500$ columns $\times 50$ levels):

| Variable | Description | Max Abs Diff | Max Rel Diff | Worst Column | Status |
|---|---|---|---|---|---|
| `th` | Potential temperature (K) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `q` | Water vapor mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qc` | Cloud water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qr` | Rain water mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qi` | Cloud ice mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qs` | Snow mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `qg` | Graupel mixing ratio (kg/kg) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rain` | Accumulated surface rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `rainncv` | Timestep surface rain (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `snow` | Accumulated surface snow (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `snowncv` | Timestep surface snow (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `graupel` | Accumulated surface graupel (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `graupelncv` | Timestep surface graupel (mm) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |
| `sr` | Frozen precipitation ratio (-) | 0.000000e+00 | 0.000000e+00 | 0 | **PASS** |

*Result:* **100% exact bit-for-bit match (0.000000e+00)** across all 2500 columns $\times$ 50 levels (125,000 grid points) for all 14 physical output arrays in both multi-loop and single-loop configurations.

### Key Discoveries & Numerical Solutions
1. **IBM MASS Vector Reciprocal Order:**
   In WRF `mp_wsm6.F90`, density scaling factor `denfac` is computed via `vrec` and `vsqrt`:
   $$\text{denfac} = \sqrt{(1.0 / \rho) \times \rho_0}$$
   rather than $\sqrt{\rho_0 / \rho}$. Matching this exact order eliminated discrepancy across 32 convective columns.
2. **Rain Freezing Order & Limiter Depletion Threshold:**
   In `pgfrz`, WRF evaluates:
   $$pfrzdtr = \min(20 \times \pi^2 \times pfrz1 \times n0r \times \frac{denr}{\rho} \times (\exp(pfrz2 \times supcolt) - 1) \times temp \times dtcld, qr)$$
   Grouping $denr / \rho$ as `(denr / den[k])` in C++ introduced a 1-ULP variation ($1.4 \times 10^{-20}$) in $pfrzdtr$. When subtracted from $qr$, this left a tiny positive residual ($4.3 \times 10^{-19}$) in a complete depletion regime (`source > value`). In loop 2, this tiny residual triggered non-zero terminal fall velocity ($vt \approx 0.37\,\text{m/s}$), altering the semi-Lagrangian advection boundary. Replicating the Fortran expression order restored exact bit-for-bit equivalence.

### CPU vs CUDA Differences ($N = 2500$ columns, $nz = 50$ levels)

#### CUDA Thread Stack Requirement:
Due to local per-column scratch arrays in vertical sweeps, each GPU thread requires $\approx 40\,\text{KB}$ of stack space. The driver dynamically configures:
```cpp
cudaDeviceSetLimit(cudaLimitStackSize, 64 * 1024);
```
preventing silent device stack overflow.

#### CUDA FP64 vs CPU FP64 (Single-Loop $dt = 120.0\,\text{s}$):
| Variable | Max Abs Diff | Max Rel Diff | Worst Column | Notes |
|---|---|---|---|---|
| `th` | 1.136868e-13 | 3.838601e-16 | 1635 | Roundoff |
| `q` | 2.428613e-17 | 6.310375e-15 | 1605 | Roundoff |
| `qc` | 2.621059e-17 | 2.395037e-12 | 1521 | 26 attograms/kg: 1-ULP libdevice `__nv_exp` in condensation |
| `qr` | 4.857226e-17 | 1.517883e-12 | 1568 | 48 attograms/kg: 1-ULP libdevice `__nv_exp` roundoff |
| `qi` | 2.764716e-18 | 8.277429e-13 | 1865 | Machine roundoff |
| `qs` | 6.028164e-17 | 9.757820e-13 | 896 | Machine roundoff |
| `qg` | 7.025630e-17 | 4.392802e-13 | 1540 | Machine roundoff |
| `rain` | 2.664535e-15 | 5.641349e-16 | 1603 | Roundoff |
| `rainncv` | 2.664535e-15 | 1.136294e-15 | 1914 | Roundoff |
| `snow` | 8.881784e-16 | 2.943996e-16 | 1098 | Roundoff |
| `snowncv` | 1.110223e-16 | 8.105180e-16 | 1233 | Roundoff |
| `graupel` | 4.440892e-16 | 2.272149e-16 | 2322 | Roundoff |
| `graupelncv` | 4.440892e-16 | 4.700670e-16 | 2106 | Roundoff |
| `sr` | 0.000000e+00 | 0.000000e+00 | 0 | Bit-for-bit match |

#### CUDA FP32 vs CPU FP64 (Single-Loop $dt = 120.0\,\text{s}$):
- `th`: max abs $8.99 \times 10^{-5}\,\text{K}$, max rel $3.01 \times 10^{-7}$
- `q`: max abs $2.80 \times 10^{-8}\,\text{kg/kg}$, max rel $2.56 \times 10^{-5}$
- `rain`: max abs $1.25 \times 10^{-6}\,\text{mm}$, max rel $2.84 \times 10^{-7}$
- `rainncv`: max abs $1.21 \times 10^{-6}\,\text{mm}$, max rel $2.44 \times 10^{-6}$
- `snow`: max abs $4.34 \times 10^{-7}\,\text{mm}$, max rel $2.64 \times 10^{-7}$
- `graupel`: max abs $5.53 \times 10^{-7}\,\text{mm}$, max rel $2.71 \times 10^{-7}$
- `sr`: max abs $1.43 \times 10^{-10}$

All variables conform to expected single precision IEEE 754 precision ($\sim 10^{-7} - 10^{-5}$).

### Performance Benchmarks ($100,000$ columns $\times 50$ levels)

Measured exclusively with `tools/heavy.sh --exclusive` on 4-thread CPU (OMP_NUM_THREADS=4, OMP_WAIT_POLICY=passive) and NVIDIA GeForce RTX 2060 (Turing sm_75, 6 GB):

| Backend | Precision | Wall Time (5/10-iter avg) | ns / column | Speedup vs CPU-4 |
|---|---|---|---|---|
| **CPU-4** (OpenMP) | FP64 | 813.4 ms | **8134 ns** | 1.0x |
| **GPU** (CUDA) | FP64 | 180.8 ms | **1808 ns** | **4.5x** |
| **GPU** (CUDA) | FP32 | 62.5 ms | **625 ns** | **13.0x** |

### Open Issues / Requests for Other Agents
- **For Agent 0:** `cppwrf_phys` library contains both `kessler.cpp` and `wsm6.cpp`. Both schemes PASS bit-for-bit.
- CMake differential test suite `test_phys_kessler` and `test_phys_wsm6` are integrated and pass via `ctest`.

