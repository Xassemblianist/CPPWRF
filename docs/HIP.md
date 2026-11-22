# AMD HIP Backend (Phase 1)

CPPWRF's kernels are written once over `parallel_for`, `column_for`, `LaunchBatch`, `parallel_fused` and `Reducer`
(`src/core/backend.hpp`), with `CPPWRF_HD` functors shared between CPU and GPU builds. Phase 1 adds an opt-in
AMD HIP / ROCm build path (`CPPWRF_ENABLE_HIP=ON`, `OFF` by default) over a thin runtime portability header
(`src/core/gpu_runtime.hpp`) without changing the behaviour of existing CUDA or CPU builds.

Phase 1 has not yet been compiled or run on AMD ROCm hardware; items that require verification or AMD-specific
follow-up on an AMD machine are guarded where their HIP semantics differ from CUDA and listed under
[Open items](#open-items).

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

- ROCm 5.3 or later (ROCm 6.x recommended) with `amdclang++` and `hip-config.cmake` (typically under
  `/opt/rocm`).
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
  passes, equivalent to GCC `-ffp-contract=off` and nvcc `--fmad=false`) and `-fno-gpu-flush-denormals-to-zero`
  (preserve IEEE subnormals, countering AMDGPU FP32 FTZ defaults). Explicit `fma` / `fmaf` inside the glibc
  ports map through `CPPWRF_FMA_RN` / `CPPWRF_FMAF_RN` to `::fma` / `::fmaf`.
- **Remaining device math / intrinsics still using AMDGPU / OCML**:
  - Correctly rounded IEEE primitives: `::fma`, `::fmaf`, `::sqrt`, `std::sqrt`, `::rint`, `::floor`, `::floorf`,
    `std::ceil`, `std::trunc`, `std::round`, `std::frexp`, `std::fabs`, `std::copysign`, `std::fmod`.
  - `::lround` / `::lroundf` inside `lround_x86` (`src/core/backend.hpp:97-112`).
  - `std::hypot(float, float)` in `libm::hypot` (`src/core/libm.hpp:112`, FP32 only; FP64 uses `glibc::hypot`).
  - Non-identity FP32 (`CPPWRF_HIP_FP32_IDENTITY=OFF`) and `CPPWRF_HIP_GLIBC_MATH=OFF` call OCML `exp`/`log`/`pow`/trig
    directly; OCML ULP rounding differs from both glibc and CUDA libdevice, so only `CPPWRF_HIP_GLIBC_MATH=ON`
    (FP64) and `CPPWRF_HIP_FP32_IDENTITY=ON` (FP32) target tolerance-0 bit-identity.
- **Wavefront 64 vs warp 32**: `Reducer` (`kred` and `kred_seq` in `src/core/backend.hpp:565-592`) is independent
  of wavefront size (fixed 256-thread shared-memory binary tree with `__syncthreads()`, or 1-thread sequential
  sum in FP64 and FP32-identity builds). `column_work.hpp` and `column_block()` currently retain a 32-lane tile
  width (functionally bit-identical on both wave32 and wave64, with a CDNA performance cost noted below).

## Open items

Every CUDA-only or unverified ROCm construct is guarded on the CUDA path or flagged below for bring-up on AMD
hardware:

| Item | Location | Status in phase 1 & why it is open |
|---|---|---|
| HIP Graphs (`StepScope`) | `src/core/backend.cpp:91-96,120-222` | Guarded to CUDA (`graphs_mode()` returns `0` on HIP). CUDA 12 uses 3-arg `cudaGraphExecUpdate(..., &info)` and 3-arg `cudaGraphInstantiate(&e, g, 0)`, whereas HIP uses 4-arg `hipGraphExecUpdate(..., &errNode, &res)` and 5-arg `hipGraphInstantiate` (or `hipGraphInstantiateWithFlags`), and stream-capturing `hipMemcpyFromSymbolAsync` on TU-local `static __device__` symbols must be validated on ROCm. `StepScope` still runs pinned async `fatal_check` with `hipEventRecord`/`hipEventSynchronize`. |
| Local-memory / stack reservation | `src/core/backend.cpp:289-340,400-407` | `reserve_local_memory` is guarded to CUDA (no-op on HIP); `device_memory_report` ignores `hipDeviceGetLimit(hipLimitStackSize)` errors on HIP. AMDGPU scratch is managed per queue by HSA/KFD rather than permanently reserved per resident thread; confirm `hipFuncAttributes::localSizeBytes` and large-frame kernel launch behaviour on ROCm. |
| Read-only cache load `__ldg` | `src/core/gpu_runtime.hpp:149-153`, `src/phys/ra_rrtmg_common/strided.hpp:53,64` | `CPPWRF_LDG` uses `__ldg` on `__CUDA_ARCH__` and plain dereference `*(ptr)` on HIP. Verify whether AMD L1/L2 caching matches performance or benefits from `__builtin_nontemporal_load` / HIP `__ldg`. |
| Warp-vote skip hints (`__any_sync` / `__all_sync`) | `src/core/gpu_runtime.hpp:157-163`, `src/phys/ra_rrtmg_lw/rrtmg_lw_stages.hpp:396,537`, `src/phys/ra_rrtmg_lw/rrtmg_lw_rtrnmc.hpp:309`, `src/phys/ra_rrtmg_sw/rrtmg_sw_stages.hpp:322` | `CPPWRF_WARP_ANY` / `CPPWRF_WARP_ALL` keep `__any_sync(__activemask(), ...)` / `__all_sync(__activemask(), ...)` on CUDA and fall back to the scalar per-column predicate on HIP (bit-identical). Re-enable wave-agnostic `__any`/`__all` after testing on wave32 (`gfx1100`) and wave64 (`gfx90a`/`gfx942`). |
| Wavefront 64 vs warp 32 work-size & coalescing | `src/core/backend.cpp:486-497`, `src/core/column_work.hpp:33,55,72-73,83-84,88,94` | `column_block()` defaults to `{32, 1}` (requires `x % 32 == 0`) and `ColumnSlots` interleaves threads in 32-lane groups. Bit-identical on wave64, but leaves half of each CDNA wavefront idle and splits coalesced loads across two 32-lane tiles. Phase 2 should default `column_block()` to `{64, 1}` (or `hipDeviceAttributeWarpSize`) and parameterize `column_work.hpp` stride for wave64. |
| Device symbol transfers (`HIP_SYMBOL`) | `src/core/gpu_runtime.hpp:79-83`, `src/core/backend.hpp:861,884,892,908,912,923` | `fatal_code_dev` is `static __device__ int` per TU, mapped via `HIP_SYMBOL(...)`. Verify ROCm resolves TU-local static device symbols across objects linked from static library `cppwrf_core` (otherwise migrate to a single device-allocated `int*`). |
| `static __device__ const` math tables | `src/core/glibc_math_tables.hpp:3468-3481`, `src/core/glibc_mathf.hpp:266-274`, `src/core/libmvec_x86.hpp:67-69`, `src/core/libmvec_f4.hpp:29` | 27 `static __device__ const` tables initialized from `constexpr` helpers. Verify `amdclang++ -x hip` accepts them without requiring `__constant__` or explicit `constexpr` qualifiers. |
| Kernel lambda closure trait & 32 KB kernargs | `src/core/gpu_runtime.hpp:141-146`, `src/core/backend.hpp:596,613,756,803` | `CPPWRF_IS_GPU_KERNEL_TYPE` checks `std::is_trivially_copyable_v<K>` on HIP; verify all kernel call sites compile under `amdclang++`. Verify HSA kernarg limits accept up to 32 KB `BatchArgs`/`FusedArgs` packets (`LaunchBatch` / `parallel_fused`). |
| Device rounding & IEEE verification | `src/core/gpu_runtime.hpp:173-181`, `src/core/backend.hpp:97-112`, `src/core/glibc_math.hpp:76-90`, `src/core/glibc_mathf.hpp:308-314,989-1017`, `src/core/libm.hpp:112`, `CMakeLists.txt:106-107` | Verify on ROCm that `-ffp-contract=off` disables implicit FMA while `::fma`/`::fmaf` remain single-rounding hardware FMA; verify `-fno-gpu-flush-denormals-to-zero` preserves FP32 subnormals (check if `-fdenormal-fp-math=ieee` / `-fdenormal-fp-math-f32=ieee` is also needed); verify `::sqrt`, `::rint`, `::floor`/`::floorf`, `std::fmod`, `::lround`/`::lroundf` edge cases (NaN, `±Inf`, `±2^63`, ties), and `libm::hypot(float, float)` in FP32 identity mode. |
| 128-bit division in `glibc::tgamma` | `src/core/glibc_math.hpp:1644-1647` | `x87_div` uses `unsigned __int128` `/` and `%`. Verify whether AMDGPU device-libs provide `__udivti3`/`__umodti3`; if linking fails, replace with a portable 128-by-64 division helper. |
| Device abnormal halt (`NSSL_STOP` / `overflow`) | `src/core/gpu_runtime.hpp:166-170`, `src/phys/mp_nssl/nssl_util.hpp:66,108` | Mapped to `abort()` on HIP device code. Unlike CUDA `__trap()` (kernel fault), HIP `abort()` terminates the host process; acceptable for fatal internal guards, verify compilation on ROCm. |
| MPI multi-GPU UUID deduplication | `src/core/tiles_mpi.cpp:149-160` | Uses `hipDeviceProp_t::uuid` (`sizeof(prop.uuid) == 16`, ROCm >= 5.2). Verify `prop.uuid` is uniquely populated per physical GPU on the target ROCm driver rather than zero-filled. |
| CMake `hip::host` vs `hip::device` | `CMakeLists.txt:346-352` | Links `hip::host` with native `enable_language(HIP)` so the 9 `LANGUAGE CXX` files in `cppwrf_core` are not passed `-x hip`. Verify link step on target ROCm version (and `fp32_host_libm` in HIP FP32 builds). |
| Suite, CLI & proof-tool integration | `python/cppwrf/_locate.py:19`, `tests/verify/wrf_suite.py`, `tools/glibc_math_proof_cuda.cu`, `tools/glibc_mathf_proof_cuda.cu` | `backend_name()` returns `"hip"` on HIP builds. `_locate.py` (`MODES`) currently expects `"cuda"` for `gpu-fp32`/`gpu-fp64`, and `wrf_suite.py` does not yet have `--hip-dp` / `--hip` / `--hip-sp-id` flags (phase 2). |

## Verification plan

Run these stages in order on an AMD ROCm machine:

1. **Regression check on existing builds**:
   - Confirm CPU FP64 (`-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_ENABLE_HIP=OFF -DCPPWRF_DOUBLE=ON`) and CUDA builds
     (where available) configure, build and pass `ctest -L unit` with unchanged behaviour.
2. **HIP configure, build and unit tests**:
   - Configure and build `build-hip-dp` (`-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_ENABLE_HIP=ON -DCPPWRF_DOUBLE=ON`)
     and `build-hip-sp-id` (`-DCPPWRF_DOUBLE=OFF`; `CPPWRF_HIP_FP32_IDENTITY=ON` is the default).
   - Run `ctest --test-dir build-hip-dp -L unit --output-on-failure` and
     `ctest --test-dir build-hip-sp-id -L unit --output-on-failure` (exercises `test_glibc_mathf` device vs host,
     `lint_fatal_check`, `fp32_double_rounding`, `fp32_host_libm`, and `robustness`).
   - Check `./build-hip-dp/cppwrf --version` reports `backend hip precision float64`.
3. **Exhaustive device math proofs (0 bit differences)**:
   - Port/compile `tools/glibc_math_proof_cuda.cu` and `tools/glibc_mathf_proof_cuda.cu` with `amdclang++ -x hip`
     using `-ffp-contract=off -fno-gpu-flush-denormals-to-zero` and require **0 differences** device vs host
     across all FP64 functions (scalar + 2-lane libmvec) and all $2^{32}$ FP32 inputs (scalar + 4-lane libmvec,
     including subnormals).
   - Add a quick device vs host check of `lround_x86` on edge values (`NaN`, `±Inf`, `±2^63`, half-integers) and
     `fatal_check` (`raise_fatal` / `HIP_SYMBOL(fatal_code_dev)`).
4. **First end-to-end model runs (tolerance 0)**:
   - Run the self-test and compare against CPU FP64 (`./build-hip-dp/cppwrf --self-test` vs
     `./build-cpu-dp/cppwrf --self-test`).
   - Run dry dynamics (e.g. `R1`) and 3 steps of `configs/real_p1_r8.ini` with `build-hip-dp/cppwrf`, comparing
     every variable and frame against WRF v4.8.0 `real*8` and CPU FP64 via `tests/verify/exact.py` at
     **tolerance 0**.
5. **Full verification suite (`cuda_dp` cases as a `hip` backend, tolerance 0)**:
   - Add a `hip_dp` (and optionally `hip_sp_id`) backend selector to `tests/verify/wrf_suite.py` (mirroring
     `cuda_dp` and `cuda_sp_id`, keeping all reference data and tolerances unchanged).
   - Run every case in the `cuda_dp` set against `build-hip-dp/cppwrf` at **tolerance 0** (exact bit-for-bit
     match of every `wrfout` variable, frame, and attribute, plus restart checks `restart_*_fp64` and
     `straka_wrf`).
   - For HIP FP32 identity (`build-hip-sp-id/cppwrf`), run the `cuda_sp_id` case set at **tolerance 0**.
   - Exercise MPI multi-GPU decomposition (`-DCPPWRF_MPI=ON`, `mpirun -n 2` / `4` with `CPPWRF_RANK_DEVICES`
     and shared-GPU ranks) and verify bit-identity against single-rank HIP FP64.
6. **Phase 2 (HIP Graphs and wave64 tuning)**:
   - Wire HIP graph capture/update/instantiation in `src/core/backend.cpp` (`hipGraphExecUpdate`,
     `hipGraphInstantiateWithFlags`), test `CPPWRF_GRAPHS=stats`, and verify outputs are byte-identical
     (`sha256`) to `CPPWRF_GRAPHS=0` across the `hip_dp` suite.
   - Re-enable wave-agnostic RRTMG votes (`__any`/`__all`), tune `column_block()` / `column_work.hpp` for
     wave64 on CDNA (`gfx90a`/`gfx942`), and re-verify tolerance 0 on the full suite.
