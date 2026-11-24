# AMD HIP Backend (Phase 1)

CPPWRF's kernels are written once over `parallel_for`, `column_for`, `LaunchBatch`, `parallel_fused` and `Reducer`
(`src/core/backend.hpp`), with `CPPWRF_HD` functors shared between CPU and GPU builds. Phase 1 adds an opt-in
AMD HIP / ROCm build path (`CPPWRF_ENABLE_HIP=ON`, `OFF` by default) over a thin runtime portability header
(`src/core/gpu_runtime.hpp`) without changing the behaviour of existing CUDA or CPU builds.

The HIP FP64, FP32 identity and fast FP32 builds compile with ROCm 6.4.1 (gfx90a, gfx942) but have not yet run on an
AMD GPU. Every place where HIP's semantics could change a bit against CUDA is listed under
[Bit-identity review](#bit-identity-review) with what the code does about it; what only an AMD GPU can show is checked
by the [first run on AMD hardware](#first-run-on-amd-hardware) (`tools/amd/amd_run.sh`, one command).

## What phase 1 contains

- **CMake build path (`CMakeLists.txt`, `cmake/Install.cmake`)**:
  - `CPPWRF_ENABLE_HIP` (`OFF` by default), mutually exclusive with `CPPWRF_ENABLE_CUDA`. When `OFF`, CMake
    configures CUDA and CPU builds identically to before.
  - Native CMake HIP support (`enable_language(HIP)`, C++20 / HIP20, `find_package(hip REQUIRED)`), defaulting
    `CMAKE_HIP_ARCHITECTURES` to `gfx90a;gfx942;gfx1100` (MI200, MI300, RDNA 3) when not set by the caller.
  - Compiles the same single-source core, dynamics, case, physics, `src/main.cpp` and `tests/unit/*.cpp` files as
    `LANGUAGE HIP` via `cppwrf_device_sources()`, keeping the 8 host-only RRTMG/RRTM files and
    `src/core/glibc_mathf_host.cpp` as `LANGUAGE CXX`.
  - Links `hip::host` on `cppwrf_core` (with native `enable_language(HIP)`, CMake drives device compilation and
    `--hip-link` from `CMAKE_HIP_ARCHITECTURES`, while `hip::host` provides `amdhip64` and ROCm headers without
    injecting `-x hip` into the `LANGUAGE CXX` sources of `cppwrf_core`).
  - Strict IEEE flags for HIP sources: `-ffp-contract=off` and `-fno-gpu-flush-denormals-to-zero`, plus a
    `CMAKE_HIP_FLAGS` guard rejecting fast-math, FMA contraction, flush-to-zero and approximate-transcendental
    flags.
  - Matching HIP math options `CPPWRF_HIP_GLIBC_MATH` (`ON` by default) and `CPPWRF_HIP_FP32_IDENTITY` (`ON` by
    default).
  - Install and CPack flavour naming (`hip-fp64`, `hip-fp32`).
- **Portability header (`src/core/gpu_runtime.hpp`)**:
  - Central backend macros: `CPPWRF_USE_GPU` (`CPPWRF_USE_CUDA || CPPWRF_USE_HIP`), `CPPWRF_GPUCC`
    (`__CUDACC__ || __HIP__`), and `CPPWRF_DEVICE_CODE` (`__CUDA_ARCH__ || __HIP_DEVICE_COMPILE__`).
  - Runtime type, constant and API mappings (`gpuMalloc`, `gpuFree`, `gpuMemsetAsync`, `gpuMemcpy*`,
    `gpuHostAlloc`, `gpuStream*`, `gpuEvent*`, `gpuGetDevice*`, `gpuSetDevice`, `gpuDeviceGetAttribute`,
    `gpuDeviceGetLimit`/`SetLimit`, `gpuFuncGetAttributes`, `gpuGetErrorString`, `gpuGetLastError`,
    `gpuMemcpy*Symbol*` with `HIP_SYMBOL(...)` on HIP).
  - Under `CPPWRF_USE_CUDA`, every runtime macro expands to the exact CUDA call used previously, and
    `<cuda_runtime.h>` is not pulled into `LANGUAGE CXX` host translation units.
  - Guarded device helper macros for non-1:1 intrinsics: `CPPWRF_IS_GPU_KERNEL_TYPE`, `CPPWRF_LDG`,
    `CPPWRF_WARP_ANY`, `CPPWRF_WARP_ALL`, `CPPWRF_TRAP`, `CPPWRF_FMA_RN`, `CPPWRF_FMAF_RN`, `CPPWRF_DSQRT_RN`.
- **Backend layer and GPU-aware sources**:
  - `src/core/backend.hpp`, `src/core/backend.cpp`, `src/core/column_work.hpp`, `src/core/column_work.cpp`,
    `src/core/tiles.cpp`, `src/core/tiles_mpi.cpp`, the glibc/libmvec math headers (`libm.hpp`, `glibc_math.hpp`,
    `glibc_math_tables.hpp`, `glibc_mathf.hpp`, `libmvec_x86.hpp`, `libmvec_f4.hpp`), the physics files with
    device-pass or GPU-path guards, and `tests/unit/test_glibc_mathf.cpp` now route through
    `src/core/gpu_runtime.hpp`.
  - CUDA-specific features without an exact drop-in HIP equivalent (CUDA 12 graph capture/update/instantiation,
    CUDA resident-thread `cudaLimitStackSize` reservation, `__ldg`, `__any_sync`/`__all_sync` with
    `__activemask()`, inline PTX `addk`, `nv_diagnostic` pragmas) stay guarded on the CUDA path while the HIP
    path takes a safe, bit-identical fallback.

## Building with HIP

Requirements on an AMD GPU machine:

- ROCm 6.0 or later (6.4.1 is the version compiled so far) with `amdclang++` and `hip-config.cmake` (typically
  under `/opt/rocm`). CMake refuses an older HIP compiler: LLVM 17 (ROCm 6.0) is the first to round the device's
  double square root correctly.
- CMake 3.24 or later, g++ (C++20) with OpenMP for host C++ sources, NetCDF-C development headers, and Python 3
  (`numpy`, `netCDF4` for tools and verification).

Ensure CMake finds ROCm (`CMAKE_PREFIX_PATH=/opt/rocm` if not already in the environment) and disable CUDA
explicitly if `nvcc` is also installed on the host:

```bash
# HIP FP64 (bit-identical target, glibc math on device)
cmake -S . -B build-hip-dp \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/rocm \
  -DCPPWRF_ENABLE_CUDA=OFF \
  -DCPPWRF_ENABLE_HIP=ON \
  -DCPPWRF_DOUBLE=ON
cmake --build build-hip-dp -j "$(nproc)"
```

Targeting a single GPU architecture (faster build) or adding MPI:

```bash
# Single architecture (e.g. MI300X = gfx942, MI250X = gfx90a, RX 7900 = gfx1100)
cmake -S . -B build-hip-dp \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/rocm \
  -DCMAKE_HIP_ARCHITECTURES=gfx942 \
  -DCPPWRF_ENABLE_CUDA=OFF \
  -DCPPWRF_ENABLE_HIP=ON \
  -DCPPWRF_DOUBLE=ON \
  -DCPPWRF_MPI=ON
cmake --build build-hip-dp -j "$(nproc)"
```

FP32 builds:

```bash
# HIP FP32 (default: identity mode, glibc float/double + 4-lane libmvec ports on device)
cmake -S . -B build-hip-sp \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/rocm \
  -DCPPWRF_ENABLE_CUDA=OFF \
  -DCPPWRF_ENABLE_HIP=ON \
  -DCPPWRF_DOUBLE=OFF

# HIP FP32 fast mode (opt-in: device libm instead of the glibc ports)
cmake -S . -B build-hip-sp-fast \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/rocm \
  -DCPPWRF_ENABLE_CUDA=OFF \
  -DCPPWRF_ENABLE_HIP=ON \
  -DCPPWRF_DOUBLE=OFF \
  -DCPPWRF_HIP_FP32_IDENTITY=OFF
```

### HIP CMake options

| Option | Default | Meaning |
|---|---|---|
| `CPPWRF_ENABLE_HIP` | `OFF` | Build the AMD HIP backend (mutually exclusive with `CPPWRF_ENABLE_CUDA`). |
| `CMAKE_HIP_ARCHITECTURES` | `gfx90a;gfx942;gfx1100` | Semicolon-separated AMDGPU target ISA list passed to `amdclang++`. |
| `CPPWRF_HIP_GLIBC_MATH` | `ON` | Use `src/core/glibc_math.hpp` on device and host in HIP FP64 builds (`OFF` defines `CPPWRF_HIP_LIBM`). |
| `CPPWRF_HIP_FP32_IDENTITY` | `ON` | HIP FP32 identity mode (`CPPWRF_FP32_IDENTITY`, default): glibc float/double and 4-lane libmvec ports on device; `OFF` is the fast build (device libm). |

## Floating-point behaviour and bit-identity notes

- **Device math ports**: In HIP FP64 (`CPPWRF_HIP_GLIBC_MATH=ON`, default), `CPPWRF_GLIBC_MATH` is enabled just
  as in CUDA FP64, routing all transcendental `cppwrf::libm` and `cppwrf::vm` calls through the in-tree Debian 13
  glibc 2.41 ports (`src/core/glibc_math.hpp`, `src/core/libmvec_x86.hpp`). In HIP FP32 identity mode
  (`CPPWRF_HIP_FP32_IDENTITY=ON`, default), `src/core/glibc_mathf.hpp` and `src/core/libmvec_f4.hpp` are likewise
  enabled on the device. These ports avoid ROCm OCML transcendental ULP differences against glibc/CUDA on every ported
  function.
- **Compiler flags**: `CMakeLists.txt` passes `-ffp-contract=off` (no implicit FMA contraction on host or device
  passes, equivalent to GCC `-ffp-contract=off` and nvcc `--fmad=false`), `-fno-gpu-flush-denormals-to-zero`
  (preserve IEEE subnormals, countering AMDGPU FP32 FTZ defaults), `-fhip-fp32-correctly-rounded-divide-sqrt`
  (clang's default for HIP, made explicit) and `-fno-builtin-exp2 -fno-builtin-exp2f` (the host pass of a HIP source
  is clang's, which turns `pow(2, x)` into `exp2(x)`; gcc, the host compiler of the CPU and CUDA builds, keeps the
  `pow` call). `CMAKE_HIP_FLAGS` that change the arithmetic (fast math, `-ffp-model=fast`, `-fapprox-func`,
  flushed denormals, `-fno-hip-fp32-correctly-rounded-divide-sqrt`, `-munsafe-fp-atomics`, ...) are refused. Explicit
  `fma` / `fmaf` inside the glibc ports map through `CPPWRF_FMA_RN` / `CPPWRF_FMAF_RN` to `::fma` / `::fmaf`.
- **Remaining device math / intrinsics still using AMDGPU / OCML**:
  - Correctly rounded IEEE primitives: `::fma`, `::fmaf`, `::sqrt`, `std::sqrt`, `::rint`, `::floor`, `::floorf`,
    `std::ceil`, `std::trunc`, `std::round`, `std::frexp`, `std::fabs`, `std::copysign`, `std::fmod`.
  - `::lround` / `::lroundf` inside `lround_x86` (`src/core/backend.hpp`), for arguments in long's range only (the
    HIP device returns `LONG_MIN` outside it itself, as x86 does).
  - `std::hypot(float, float)` in `libm::hypot` (`src/core/libm.hpp:112`, FP32 only; FP64 uses `glibc::hypot`).
  - Non-identity FP32 (`CPPWRF_HIP_FP32_IDENTITY=OFF`) and `CPPWRF_HIP_GLIBC_MATH=OFF` call OCML `exp`/`log`/`pow`/trig
    directly; OCML ULP rounding differs from both glibc and CUDA libdevice, so only `CPPWRF_HIP_GLIBC_MATH=ON`
    (FP64) and `CPPWRF_HIP_FP32_IDENTITY=ON` (FP32) target tolerance-0 bit-identity.
- **Wavefront 64 vs warp 32**: `Reducer` (`kred` and `kred_seq` in `src/core/backend.hpp:565-592`) is independent
  of wavefront size (fixed 256-thread shared-memory binary tree with `__syncthreads()`, or 1-thread sequential
  sum in FP64 and FP32-identity builds). `column_work.hpp` and `column_block()` currently retain a 32-lane tile
  width (functionally bit-identical on both wave32 and wave64, with a CDNA performance cost noted below).

## Bit-identity review

Where a HIP build could compute other bits than the CUDA and CPU builds (HIP2 review). "Fixed" changes the source;
"compiled" was checked on the AMDGPU code (gfx942, LLVM 19 with ROCm 6.4.1's device libraries); "AMD run" is left to
the first run on AMD hardware, which checks it (the test or step named).

| Item | Where | HIP behaviour and what CPPWRF does | Status |
|---|---|---|---|
| FMA contraction | `CMakeLists.txt` | `-ffp-contract=off` on both passes; the device code keeps `a * b + c` as `v_mul` + `v_add`; explicit `fma` is `v_fma` (one rounding). Division and square root expand to FMA sequences that are correctly rounded. | compiled; AMD run: `test_device_ieee`, math proofs |
| Subnormals | `CMakeLists.txt` | `-fno-gpu-flush-denormals-to-zero`: kernels carry `float_denorm_mode_32 = 3` and `float_denorm_mode_16_64 = 3` (IEEE), round to nearest even, IEEE mode on. | compiled; AMD run: `test_device_ieee` (subnormal results) |
| Float division and square root | `CMakeLists.txt` | Correctly rounded (`-fhip-fp32-correctly-rounded-divide-sqrt`, explicit); an option that turns it off is refused. | fixed; AMD run: `test_device_ieee` |
| Double square root | `CMakeLists.txt` | Correctly rounded from LLVM 17 on (an `rsq` + Newton sequence); CMake refuses an older compiler. | fixed (ROCm >= 6.0) |
| `fmax` / `fmin` | `src/core/backend.hpp` (`fmax_x86`, `fmin_x86`), `src/phys/cu_ntiedtke/ntiedtke.hpp`, `src/nest/nest_interp.hpp` | ROCm's `fmax` / `fmin` are the GPU's `v_max` / `v_min`, whose result for two zeros of opposite sign need not be glibc's (glibc 2.41: the first argument). The HIP device writes glibc's comparisons out; host and CUDA unchanged. | fixed; AMD run: `test_device_ieee` |
| `lround_x86` below `-2^63` | `src/core/backend.hpp` | An AMD GPU has no 64-bit float to integer conversion (LLVM expands it; out of range is undefined): the HIP device returns `LONG_MIN` there as x86 does. | fixed; AMD run: `test_lround_x86` (device part now on HIP too) |
| Float to int conversion out of range | kernels | `v_cvt_i32_f64` saturates (NaN to 0) as CUDA does; x86 gives `INT_MIN`. The code converts in range (or through `lround_x86`), as for CUDA. | as CUDA |
| 128-bit integer division (`glibc::tgamma`'s `x87_div`) | `src/core/glibc_math.hpp` | Expanded inline by LLVM (no `__udivti3` call). | compiled; AMD run: math proof (`tgamma`) |
| Host pass compiler | every HIP source | clang, not gcc: `pow(2, x)` to `exp2(x)` stopped (`-fno-builtin-exp2`, `-fno-builtin-exp2f`). A math call with constant arguments folds with the build machine's libm under clang, with MPFR (correctly rounded) under gcc: the device code has no such folds (`CPPWRF_LM_CONST` is false there) and the host code's are rare (initialisation). | fixed (exp2); AMD run: suite |
| Wavefront 64 | `kred`, `kred_seq`, `column_work.hpp`, RRTMG votes | No shuffles or warp-synchronous code; the reductions are fixed trees or sequential; column work slots are a memory layout in 32-thread groups (independent of the hardware's wavefront); RRTMG's vote hints take the per-column predicate on HIP. | as CUDA (performance only) |
| Atomics | `raise_fatal`, RRTMG `iscal` counters | Integer only (a CAS minimum, counts): the result does not depend on the order. No floating-point atomics. | as CUDA |
| Fatal flag per launch (ensembles) | `src/core/backend.hpp` (`CPPWRF_KSLOT_PARAM`), `src/cases/ensemble.cpp` | A HIP kernel cannot read the size of its dynamic shared memory (CUDA's carrier of the slot). HIP launches pass the slot as a last kernel argument, which each thread stores into a shared-memory word that `raise_fatal` reads (where `raise_fatal` is inlined the compiler forwards the argument and the kernel keeps no shared memory); the HIP threads ensemble's shared-word caveat is gone. CUDA and CPU kernels unchanged. | fixed; compiled; AMD run: `test_fatal` (e) runs the two slot threads at once, `ensemble` |
| Device symbols of the fatal flags | `fatal_code_dev`, `HIP_SYMBOL` | The TU-local `static __device__` array is emitted as a global symbol of its TU's code object (clang externalises device variables the host uses), so `hipMemcpy*Symbol*` can find it. | compiled; AMD run: `test_fatal` |
| Kernel parameter size | `LaunchBatch`, `parallel_fused` | ROCm's limit is not documented above 4 KiB: a HIP batch or fused launch with parameters above `CPPWRF_HIP_MAX_KERNARG` (4096) launches its parts one by one (the same results). A single kernel's parameters are far below. | fixed |
| Device halt (`NSSL_STOP`) | `src/core/gpu_runtime.hpp` | `abort()` on the device (`s_trap`): the HIP runtime ends the process; the NSSL guards are unreachable in the supported configurations. | as before |
| HIP graphs | `src/core/backend.cpp` | Off on HIP (`graphs_mode()` 0): each step's launches run directly, fatal checks through pinned slots. Same kernels, same order. | phase 2 (performance) |
| Stack / scratch | `reserve_local_memory` | No-op on HIP (scratch is allocated per queue by the runtime). A launch that cannot get its scratch fails with the runtime's error. | AMD run: suite (MYNN, KF, NSSL frames) |
| `__ldg` | `CPPWRF_LDG` | A plain load on HIP. | performance only |
| `libm::hypot(float, float)` | `src/core/libm.hpp` | Device `hypotf` (no caller in the model). | open (no caller) |
| Pageable async copies | `copy_to_dev_async` | Uploads are staged from pinned buffers (`HostStaging`); downloads wait on the stream. | AMD run: suite |
| MPI device UUID | `src/core/tiles_mpi.cpp` | `hipDeviceGetUuid`. | AMD run (multi-GPU MPI, not in `amd_run.sh`) |

## First run on AMD hardware

`tools/amd/amd_run.sh` does the whole verification on a fresh Ubuntu node with ROCm (6.0 or later) and AMD Instinct
GPUs, in one command from a checkout of the commit to test:

```bash
git clone https://github.com/Xassemblianist/CPPWRF && cd CPPWRF
tools/amd/amd_run.sh                     # results in ~/cppwrf_amd/res/summary.txt; last line AMD RUN PASS / FAIL
```

1. **Dependencies**: g++, CMake 3.24 or later (pip's on an older Ubuntu), NetCDF, Python with numpy and netCDF4
   (apt); ROCm itself must be there (`--install-rocm 6.4.1` installs it from repo.radeon.com when only the driver is).
2. **Builds** for the first GPU's architecture (`rocminfo`; `--arch` to choose): `b-hip-dp` (HIP FP64), `b-hip-id`
   (HIP FP32 identity), `b-hip-sp` (fast HIP FP32), and `b-cpu-dp` (CPU FP64, for triage: `CPPWRF_VM_DUMP` runs of
   both builds find the first kernel that differs). Each binary must report `backend hip` and its precision.
3. **Data** (while the programs build): the tars of the selected checks (`wrf_suite.py --data-for`) from the public
   release `data-v1` of cppwrf-data (its `ci/fetch_data.sh`), and the files of `v05_files_data.tar` from the packs of
   `data-v2` (`tools/amd/amd_cases.py members`), as the CUDA proofs take them: about 240 GB for the full suite.
4. **ctest** of each HIP build, with the reference data in place: `test_device_ieee` (the device's IEEE arithmetic
   against the host's), `test_lround_x86`, `test_fatal` (per-launch slots), `test_glibc_mathf`, `ensemble`, ...
5. **Device math proofs**: `tools/glibc_math_proof_cuda.cu` (glibc's double functions and libmvec's, 10^7 random
   inputs each plus the edge values) and `tools/glibc_mathf_proof_cuda.cu` (every float input of the float
   functions, 2^28 pairs for `powf` / `atan2f`), compiled for HIP (`tools/proof_gpu_runtime.hpp`): 0 differences.
6. **The GPU suite** (`tests/verify/wrf_suite.py --hip-dp / --hip-sp-id / --hip`): every check of the CUDA proofs
   (FP64: the `cuda_dp` cases with the slow ones and the restart checks, 423; FP32 identity: 44; fast FP32: 44),
   FP64 and FP32 identity against WRF at tolerance 0. The checks are shared out over the node's GPUs
   (`HIP_VISIBLE_DEVICES`, `--gpus N`).
7. **Summary**: `res/summary.txt` (machine, builds, ctest, proofs, every FAIL / SKIP line), ending `AMD RUN PASS` only
   when everything passed.

The run is resumable: a finished step is not repeated (`--redo STEP` repeats one), and a suite started again runs only
its checks without a PASS line. `--cases NAME,...` runs some checks only (`tools/amd/amd_cases.py list cuda_dp` names
them), `--flavours dp` one build, `--compile-only --arch gfx942` builds without a GPU. Multi-GPU MPI and tiled runs
(`tools/kaggle/kg_body.sh`'s layouts) and HIP graphs / wave64 tuning (phase 2) come after this run.
