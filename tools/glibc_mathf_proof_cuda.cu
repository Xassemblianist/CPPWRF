// SPDX-License-Identifier: GPL-3.0-or-later
// Proof of src/core/glibc_mathf.hpp and src/core/libmvec_f4.hpp on the device: every float input of the scalar
// expf, logf, sinf, cosf, atanf, tanf, tanhf, log10f and of the 4-lane expf, logf, sinf, cosf, atanf, and powf (scalar
// and 4-lane) and atan2f on 2^N pseudo-random pairs, device against host, bit for bit (two NaNs count as equal). The
// host functions are proven against the system libm / libmvec by tools/glibc_mathf_proof.cpp and
// tests/unit/test_libmvec_f4.cpp.
//   nvcc -std=c++20 -O3 --fmad=false -arch=native -DCPPWRF_GLIBC_MATH -DCPPWRF_FP32_IDENTITY -Isrc \
//        -Xcompiler -fopenmp,-ffp-contract=off --expt-relaxed-constexpr tools/glibc_mathf_proof_cuda.cu -o proof_cuda
//   ./proof_cuda [log2 of the number of pairs, default 32] [names, e.g. "tanf,atan2f": only those]
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/glibc_mathf.hpp"
#include "core/libmvec_f4.hpp"

namespace {
constexpr long long kBatch = 1ll << 26;
enum Fn { kExp, kLog, kSin, kCos, kAtan, kTan, kTanh, kLog10, kVExp, kVLog, kVSin, kVCos, kVAtan, kPow, kVPow, kAtan2,
          kNFn };
const char* kName[kNFn] = {"expf", "logf", "sinf", "cosf", "atanf", "tanf", "tanhf", "log10f", "v4 expf", "v4 logf",
                           "v4 sinf", "v4 cosf", "v4 atanf", "powf", "v4 powf", "atan2f"};
__host__ __device__ inline std::uint64_t mix(std::uint64_t z) {
  z += 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}
__host__ __device__ inline float asf(std::uint32_t b) { return cppwrf::glibcf::detail::asf(b); }
__host__ __device__ inline void pair(long long i, float& x, float& y) {
  const std::uint64_t r = mix(std::uint64_t(i));
  x = asf(std::uint32_t(r));
  y = asf(std::uint32_t(r >> 32));
  if (i & 1) {  // x in (0, 8), |y| in [2^-7, 2^5)
    x = asf(0x3f800000u + std::uint32_t(r % 0x1800000u) - 0x800000u);
    y = asf((std::uint32_t(r >> 32) & 0x80000000u) | (0x3c000000u + std::uint32_t((r >> 32) % 0x6000000u)));
  }
}
__host__ __device__ inline float eval(int fn, long long i) {
  namespace g = cppwrf::glibcf;
  namespace v = cppwrf::mvec4;
  if (fn == kPow || fn == kVPow) {
    float x, y;
    pair(i, x, y);
    return fn == kPow ? g::powf(x, y) : v::powf(x, y);
  }
  if (fn == kAtan2) {  // any two floats
    const std::uint64_t r = mix(std::uint64_t(i) ^ 0x5bd1e995ull);
    return g::atan2f(asf(std::uint32_t(r)), asf(std::uint32_t(r >> 32)));
  }
  const float x = asf(std::uint32_t(i));
  switch (fn) {
    case kExp: return g::expf(x);
    case kLog: return g::logf(x);
    case kSin: return g::sinf(x);
    case kCos: return g::cosf(x);
    case kAtan: return g::atanf(x);
    case kTan: return g::tanf(x);
    case kTanh: return g::tanhf(x);
    case kLog10: return g::log10f(x);
    case kVExp: return v::expf(x);
    case kVLog: return v::logf(x);
    case kVSin: return v::sinf(x);
    case kVAtan: return v::atanf(x);
    default: return v::cosf(x);
  }
}
__global__ void kern(int fn, long long i0, long long n, float* out) {
  const long long t = blockIdx.x * (long long)blockDim.x + threadIdx.x;
  if (t < n) out[t] = eval(fn, i0 + t);
}
std::uint32_t u(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, 4);
  return b;
}
}  // namespace

int main(int argc, char** argv) {
  const int lg = argc > 1 ? std::atoi(argv[1]) : 32;
  float* d = nullptr;
  cudaMalloc(&d, kBatch * sizeof(float));
  std::vector<float> dev(kBatch);
  long long total = 0;
  for (int fn = 0; fn < kNFn; ++fn) {
    if (argc > 2 && !std::strstr(argv[2], kName[fn])) continue;  // a list of names: only those
    const bool pairs = fn == kPow || fn == kVPow || fn == kAtan2;
    const long long count = pairs ? (1ll << lg) : (1ll << 32);
    long long bad = 0;
    long long first = -1;
    for (long long i0 = 0; i0 < count; i0 += kBatch) {
      const long long n = std::min(kBatch, count - i0);
      kern<<<unsigned((n + 255) / 256), 256>>>(fn, i0, n, d);
      cudaMemcpy(dev.data(), d, n * sizeof(float), cudaMemcpyDeviceToHost);
#pragma omp parallel for schedule(static, 1 << 16) reduction(+ : bad)
      for (long long t = 0; t < n; ++t) {
        const float h = eval(fn, i0 + t);
        if (!(std::isnan(h) && std::isnan(dev[t])) && u(h) != u(dev[t])) {
          ++bad;
#pragma omp critical
          if (first < 0) first = i0 + t;
        }
      }
    }
    const cudaError_t e = cudaGetLastError();
    if (e != cudaSuccess) {
      std::printf("CUDA error: %s\n", cudaGetErrorString(e));
      return 2;
    }
    std::printf("%-8s %s: %lld differ", kName[fn], pairs ? "pairs" : "all 2^32 inputs", bad);
    if (first >= 0) std::printf(" (first index %lld)", first);
    std::printf("\n");
    total += bad;
  }
  std::printf(total ? "glibc_mathf_proof_cuda FAILED\n" : "glibc_mathf_proof_cuda passed (device = host)\n");
  return total ? 1 : 0;
}
