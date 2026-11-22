// SPDX-License-Identifier: GPL-3.0-or-later
// Thin portability layer between CUDA and AMD HIP runtime APIs and device macros.
// When building with CUDA (CPPWRF_USE_CUDA), every runtime macro and type alias expands to the
// exact CUDA call or symbol used directly, leaving CUDA builds unchanged in behaviour.

#if !defined(CPPWRF_GPU_RUNTIME_HEADER_INCLUDED) && \
    (defined(__CUDACC__) || defined(__HIP__) || defined(CPPWRF_GPU_RUNTIME_HOST))
#if defined(CPPWRF_USE_HIP) || (defined(__HIP__) && !defined(CPPWRF_USE_CUDA))
#include <hip/hip_runtime.h>
#define CPPWRF_GPU_RUNTIME_HEADER_INCLUDED 1
#elif defined(CPPWRF_USE_CUDA) || defined(__CUDACC__)
#include <cuda_runtime.h>
#define CPPWRF_GPU_RUNTIME_HEADER_INCLUDED 1
#endif
#endif

#ifndef CPPWRF_CORE_GPU_RUNTIME_HPP
#define CPPWRF_CORE_GPU_RUNTIME_HPP

#if defined(CPPWRF_USE_CUDA) || defined(CPPWRF_USE_HIP)
#define CPPWRF_USE_GPU 1
#endif

#if defined(__CUDACC__) || defined(__HIP__)
#define CPPWRF_GPUCC 1
#endif

#if defined(__CUDA_ARCH__) || defined(__HIP_DEVICE_COMPILE__)
#define CPPWRF_DEVICE_CODE 1
#endif

#if defined(CPPWRF_USE_HIP) || (defined(__HIP__) && !defined(CPPWRF_USE_CUDA))

#define CPPWRF_GPU_BACKEND_NAME "hip"
#define CPPWRF_GPU_API_NAME "HIP"

// Runtime types and constants
#define gpuError_t hipError_t
#define gpuSuccess hipSuccess
#define gpuStream_t hipStream_t
#define gpuEvent_t hipEvent_t
#define gpuEventDisableTiming hipEventDisableTiming
#define gpuMemcpyDeviceToHost hipMemcpyDeviceToHost
#define gpuMemcpyHostToDevice hipMemcpyHostToDevice
#define gpuMemcpyDeviceToDevice hipMemcpyDeviceToDevice
#define gpuHostAllocDefault hipHostMallocDefault
#define gpuDeviceProp hipDeviceProp_t
#define gpuFuncAttributes hipFuncAttributes
#define gpuLimitStackSize hipLimitStackSize
#define gpuDevAttrMultiProcessorCount hipDeviceAttributeMultiprocessorCount
#define gpuDevAttrMaxThreadsPerMultiProcessor hipDeviceAttributeMaxThreadsPerMultiProcessor

// Runtime API calls
#define gpuGetErrorString hipGetErrorString
#define gpuGetLastError hipGetLastError
#define gpuDeviceSynchronize hipDeviceSynchronize
#define gpuStreamCreate hipStreamCreate
#define gpuStreamSynchronize hipStreamSynchronize
#define gpuEventCreateWithFlags hipEventCreateWithFlags
#define gpuEventRecord hipEventRecord
#define gpuEventSynchronize hipEventSynchronize
#define gpuMalloc hipMalloc
#define gpuFree hipFree
#define gpuMemsetAsync hipMemsetAsync
#define gpuMemcpy hipMemcpy
#define gpuMemcpyAsync hipMemcpyAsync
#define gpuHostAlloc hipHostMalloc
#define gpuMemGetInfo hipMemGetInfo
#define gpuGetDevice hipGetDevice
#define gpuSetDevice hipSetDevice
#define gpuGetDeviceCount hipGetDeviceCount
#define gpuGetDeviceProperties hipGetDeviceProperties
#define gpuDeviceGetAttribute hipDeviceGetAttribute
#define gpuDeviceGetLimit hipDeviceGetLimit
#define gpuDeviceSetLimit hipDeviceSetLimit
#define gpuFuncGetAttributes hipFuncGetAttributes

// Device symbol transfers (HIP requires HIP_SYMBOL around the symbol identifier)
#define gpuMemcpyToSymbol(sym, src, count) hipMemcpyToSymbol(HIP_SYMBOL(sym), (src), (count))
#define gpuMemcpyToSymbolAsync(sym, src, count, offset, kind, stream) \
  hipMemcpyToSymbolAsync(HIP_SYMBOL(sym), (src), (count), (offset), (kind), (stream))
#define gpuMemcpyFromSymbolAsync(dst, sym, count, offset, kind, stream) \
  hipMemcpyFromSymbolAsync((dst), HIP_SYMBOL(sym), (count), (offset), (kind), (stream))

#elif defined(CPPWRF_USE_CUDA) || defined(__CUDACC__)

#define CPPWRF_GPU_BACKEND_NAME "cuda"
#define CPPWRF_GPU_API_NAME "CUDA"

// Runtime types and constants (expand to the exact CUDA names)
#define gpuError_t cudaError_t
#define gpuSuccess cudaSuccess
#define gpuStream_t cudaStream_t
#define gpuEvent_t cudaEvent_t
#define gpuEventDisableTiming cudaEventDisableTiming
#define gpuMemcpyDeviceToHost cudaMemcpyDeviceToHost
#define gpuMemcpyHostToDevice cudaMemcpyHostToDevice
#define gpuMemcpyDeviceToDevice cudaMemcpyDeviceToDevice
#define gpuHostAllocDefault cudaHostAllocDefault
#define gpuDeviceProp cudaDeviceProp
#define gpuFuncAttributes cudaFuncAttributes
#define gpuLimitStackSize cudaLimitStackSize
#define gpuDevAttrMultiProcessorCount cudaDevAttrMultiProcessorCount
#define gpuDevAttrMaxThreadsPerMultiProcessor cudaDevAttrMaxThreadsPerMultiProcessor

// Runtime API calls (expand to the exact CUDA calls)
#define gpuGetErrorString cudaGetErrorString
#define gpuGetLastError cudaGetLastError
#define gpuDeviceSynchronize cudaDeviceSynchronize
#define gpuStreamCreate cudaStreamCreate
#define gpuStreamSynchronize cudaStreamSynchronize
#define gpuEventCreateWithFlags cudaEventCreateWithFlags
#define gpuEventRecord cudaEventRecord
#define gpuEventSynchronize cudaEventSynchronize
#define gpuMalloc cudaMalloc
#define gpuFree cudaFree
#define gpuMemsetAsync cudaMemsetAsync
#define gpuMemcpy cudaMemcpy
#define gpuMemcpyAsync cudaMemcpyAsync
#define gpuHostAlloc cudaHostAlloc
#define gpuMemGetInfo cudaMemGetInfo
#define gpuGetDevice cudaGetDevice
#define gpuSetDevice cudaSetDevice
#define gpuGetDeviceCount cudaGetDeviceCount
#define gpuGetDeviceProperties cudaGetDeviceProperties
#define gpuDeviceGetAttribute cudaDeviceGetAttribute
#define gpuDeviceGetLimit cudaDeviceGetLimit
#define gpuDeviceSetLimit cudaDeviceSetLimit
#define gpuFuncGetAttributes cudaFuncGetAttributes

// Device symbol transfers (expand to the exact CUDA calls)
#define gpuMemcpyToSymbol(sym, src, count) cudaMemcpyToSymbol(sym, src, count)
#define gpuMemcpyToSymbolAsync(sym, src, count, offset, kind, stream) \
  cudaMemcpyToSymbolAsync(sym, src, count, offset, kind, stream)
#define gpuMemcpyFromSymbolAsync(dst, sym, count, offset, kind, stream) \
  cudaMemcpyFromSymbolAsync(dst, sym, count, offset, kind, stream)

#endif

// Kernel-type check: nvcc extended __host__ __device__ lambdas are not always std::is_trivially_copyable
#if defined(__CUDACC__) && !defined(__HIP__)
#define CPPWRF_IS_GPU_KERNEL_TYPE(K) \
  (__nv_is_extended_host_device_lambda_closure_type(K) || std::is_trivially_copyable_v<K>)
#else
#define CPPWRF_IS_GPU_KERNEL_TYPE(K) (std::is_trivially_copyable_v<K>)
#endif

// Read-only cache load: keep CUDA __ldg; on HIP use a plain load until verified on AMD hardware.
#if defined(__CUDA_ARCH__)
#define CPPWRF_LDG(ptr) __ldg(ptr)
#else
#define CPPWRF_LDG(ptr) (*(ptr))
#endif

// Warp-vote skip hints (RRTMG clear/cloudy checks): CUDA uses 32-bit __activemask(); on HIP fall back
// to the per-column predicate (bit-identical) until wave32/wave64 vote intrinsics are verified on AMD hardware.
#if defined(__CUDA_ARCH__)
#define CPPWRF_WARP_ANY(pred) __any_sync(__activemask(), (pred))
#define CPPWRF_WARP_ALL(pred) __all_sync(__activemask(), (pred))
#else
#define CPPWRF_WARP_ANY(pred) (pred)
#define CPPWRF_WARP_ALL(pred) (pred)
#endif

// Device abnormal halt (NSSL overflow / stop guards): CUDA __trap() faults the kernel; HIP abort() halts execution.
#if defined(__CUDA_ARCH__)
#define CPPWRF_TRAP() __trap()
#elif defined(__HIP_DEVICE_COMPILE__)
#define CPPWRF_TRAP() abort()
#endif

// Explicit round-to-nearest-even FMA and double sqrt on the device (glibc math ports)
#if defined(__CUDA_ARCH__)
#define CPPWRF_FMA_RN(a, b, c) __fma_rn((a), (b), (c))
#define CPPWRF_FMAF_RN(a, b, c) __fmaf_rn((a), (b), (c))
#define CPPWRF_DSQRT_RN(x) __dsqrt_rn(x)
#elif defined(__HIP_DEVICE_COMPILE__)
#define CPPWRF_FMA_RN(a, b, c) ::fma((a), (b), (c))
#define CPPWRF_FMAF_RN(a, b, c) ::fmaf((a), (b), (c))
#define CPPWRF_DSQRT_RN(x) ::sqrt(x)
#endif

#endif  // CPPWRF_CORE_GPU_RUNTIME_HPP
