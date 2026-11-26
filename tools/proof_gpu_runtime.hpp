// SPDX-License-Identifier: GPL-3.0-or-later
// The few CUDA runtime calls of the device math proofs (tools/glibc_math_proof_cuda.cu,
// tools/glibc_mathf_proof_cuda.cu), also for an AMD HIP compile of the same sources:
//   hipcc -x hip --offload-arch=gfx942 -std=c++20 -O3 -ffp-contract=off -fno-gpu-flush-denormals-to-zero \
//         -fhip-fp32-correctly-rounded-divide-sqrt -fopenmp -DCPPWRF_GLIBC_MATH -Isrc -Itools \
//         tools/glibc_math_proof_cuda.cu -o glibc_math_proof_hip
// (tools/amd/amd_run.sh builds both). nvcc: <cuda_runtime.h> itself, the tools unchanged.
#pragma once
#if defined(__HIP__)
#include <hip/hip_runtime.h>
#define cudaMalloc hipMalloc
#define cudaFree hipFree
#define cudaMemcpy hipMemcpy
#define cudaMemcpyHostToDevice hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost hipMemcpyDeviceToHost
#define cudaDeviceProp hipDeviceProp_t
#define cudaGetDeviceProperties hipGetDeviceProperties
#define cudaGetLastError hipGetLastError
#define cudaGetErrorString hipGetErrorString
#define cudaError_t hipError_t
#define cudaSuccess hipSuccess
#else
#include <cuda_runtime.h>
#endif
