// SPDX-License-Identifier: GPL-3.0-or-later
// src/core/glibc_mathf.hpp (glibc 2.41's scalar expf, logf, powf, sinf, cosf, atanf, tanf, tanhf, log10f, atan2f,
// asinf, acosf, erff, tgammaf, sinhf)
// against the system libm, bit for bit (two NaNs count as equal): every 61st float bit pattern of the unary functions
// and 2^20 pseudo-random powf / atan2f pairs. The ports reproduce Debian's glibc 2.41 (FMA variants): with another
// glibc version the differences are reported, not failed (tools/glibc_mathf_proof.cpp checks every input).
// CUDA builds: also the device against the host on the same inputs, and in the FP32 identity build
// (CPPWRF_FP32_IDENTITY) the 4-lane functions of src/core/libmvec_f4.hpp.
#include <dlfcn.h>
#include <gnu/libc-version.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/backend.hpp"
#include "core/glibc_mathf.hpp"

namespace {
// 0-6 unary scalar, 7 powf, 8-12 the 4-lane expf, logf, powf, sinf, cosf, 13 tanf, 14 atan2f, 15-19 asinf, acosf,
// erff, tgammaf, sinhf
constexpr int kNFn = 20;
const char* kName[kNFn] = {"expf",    "logf",    "sinf",    "cosf",    "atanf",   "tanhf", "log10f", "powf",
                           "v4 expf", "v4 logf", "v4 powf", "v4 sinf", "v4 cosf", "tanf",  "atan2f", "asinf",
                           "acosf",   "erff",    "tgammaf", "sinhf"};
// the scalar functions (host and device) and, in the identity build, the 4-lane ones (device)
constexpr int kScalar[] = {0, 1, 2, 3, 4, 5, 6, 7, 13, 14, 15, 16, 17, 18, 19};
CPPWRF_HD inline bool is_pair(int fn) { return fn == 7 || fn == 10 || fn == 14; }
CPPWRF_HD inline std::uint64_t mix(std::uint64_t z) {
  z += 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}
CPPWRF_HD inline float input(int fn, long long i, float* y) {
  using cppwrf::glibcf::detail::asf;
  if (is_pair(fn)) {
    const std::uint64_t r = mix(std::uint64_t(i));
    float x = asf(std::uint32_t(r));
    *y = asf(std::uint32_t(r >> 32));
    if ((i & 1) && fn != 14) {  // x in (0, 8), |y| in [2^-7, 2^5): the arguments of a model
      x = asf(0x3f800000u + std::uint32_t(r % 0x1800000u) - 0x800000u);
      *y = asf((std::uint32_t(r >> 32) & 0x80000000u) | (0x3c000000u + std::uint32_t((r >> 32) % 0x6000000u)));
    }
    return x;
  }
  *y = 0.0f;
  return asf(std::uint32_t(i * 61));
}
CPPWRF_HD inline float port(int fn, float x, float y) {
  namespace g = cppwrf::glibcf;
  switch (fn) {
    case 0: return g::expf(x);
    case 1: return g::logf(x);
    case 2: return g::sinf(x);
    case 3: return g::cosf(x);
    case 4: return g::atanf(x);
    case 5: return g::tanhf(x);
    case 6: return g::log10f(x);
    case 7: return g::powf(x, y);
    case 13: return g::tanf(x);
    case 14: return g::atan2f(x, y);
    case 15: return g::asinf(x);
    case 16: return g::acosf(x);
    case 17: return g::erff(x);
    case 18: return g::tgammaf(x);
    case 19: return g::sinhf(x);
#if defined(CPPWRF_HAVE_MVEC_F4)
    case 8: return cppwrf::mvec4::expf(x);
    case 9: return cppwrf::mvec4::logf(x);
    case 10: return cppwrf::mvec4::powf(x, y);
    case 11: return cppwrf::mvec4::sinf(x);
    case 12: return cppwrf::mvec4::cosf(x);
#endif
    default: return 0.0f;
  }
}
// the system libm's own functions, looked up in libm.so.6: the FP32 builds' executables define expf ... themselves
// (src/core/glibc_mathf_host.cpp, also in cppwrf_core), so ::expf would be the port (volatile pointers: no folding)
float (*volatile sys1[kNFn])(float) = {};
float (*volatile sys_powf)(float, float) = nullptr;
float (*volatile sys_tanf)(float) = nullptr;
float (*volatile sys_atan2f)(float, float) = nullptr;
bool load_libm();
template <class F> bool libm_fn(F volatile& f, const char* name) {
  static void* h = dlopen("libm.so.6", RTLD_NOW | RTLD_LOCAL);
  f = h ? reinterpret_cast<F>(dlsym(h, name)) : nullptr;
  Dl_info in{}, self{};
  const bool own = f != nullptr && dladdr(reinterpret_cast<void*>(f), &in) &&
                   dladdr(reinterpret_cast<void*>(&load_libm), &self) && in.dli_fbase == self.dli_fbase;
  if (f == nullptr || own) {
    std::printf("%s: %s\n", name, f == nullptr ? "not found in libm.so.6" : "the program's own function, not libm's");
    return false;
  }
  return true;
}
bool load_libm() {
  constexpr int kUnary[] = {0, 1, 2, 3, 4, 5, 6, 15, 16, 17, 18, 19};
  bool ok = libm_fn(sys_powf, "powf") & libm_fn(sys_tanf, "tanf") & libm_fn(sys_atan2f, "atan2f");
  for (int fn : kUnary) ok &= libm_fn(sys1[fn], kName[fn]);
  return ok;
}
std::uint32_t bits(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, 4);
  return b;
}
bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || bits(a) == bits(b); }
long long count(int fn) { return is_pair(fn) ? (1ll << 20) : (1ll << 32) / 61 + 1; }
#if defined(__CUDACC__)
__global__ void kport(int fn, long long n, float* out) {
  const long long t = blockIdx.x * (long long)blockDim.x + threadIdx.x;
  if (t >= n) return;
  float y;
  const float x = input(fn, t, &y);
  out[t] = port(fn, x, y);
}
#endif
}  // namespace

int main() {
  const bool strict = std::strcmp(gnu_get_libc_version(), "2.41") == 0;
  long long fails = 0;
  if (!load_libm()) {
    std::printf("test_glibc_mathf FAILED: the system libm's functions are not reachable\n");
    return 1;
  }
  for (int fn : kScalar) {  // host ports against the system libm
    long long bad = 0;
    const long long n = count(fn);
    for (long long i = 0; i < n; ++i) {
      float y;
      const float x = input(fn, i, &y);
      const float s = fn == 7 ? sys_powf(x, y) : fn == 13 ? sys_tanf(x) : fn == 14 ? sys_atan2f(x, y) : sys1[fn](x);
      if (!same(port(fn, x, y), s)) ++bad;
    }
    std::printf("%-8s host vs libm %s: %lld of %lld differ\n", kName[fn], gnu_get_libc_version(), bad, n);
    if (strict) fails += bad;
  }
#if defined(__CUDACC__)
  for (int fn = 0; fn < kNFn; ++fn) {  // the device against the host
#if !defined(CPPWRF_HAVE_MVEC_F4)
    if (fn >= 8 && fn <= 12) continue;
#endif
    const long long n = count(fn);
    float* d = nullptr;
    if (cudaMalloc(&d, n * sizeof(float)) != cudaSuccess) {
      std::printf("no CUDA device: the device check is skipped\n");
      break;
    }
    kport<<<unsigned((n + 255) / 256), 256>>>(fn, n, d);
    std::vector<float> h(static_cast<std::size_t>(n));
    cudaMemcpy(h.data(), d, n * sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(d);
    if (cudaGetLastError() != cudaSuccess) {
      std::printf("CUDA error: the device check fails\n");
      return 1;
    }
    long long bad = 0;
    for (long long i = 0; i < n; ++i) {
      float y;
      const float x = input(fn, i, &y);
      if (!same(port(fn, x, y), h[std::size_t(i)])) ++bad;
    }
    std::printf("%-8s device vs host: %lld of %lld differ\n", kName[fn], bad, n);
    fails += bad;
  }
#endif
  std::printf(fails ? "test_glibc_mathf FAILED\n" : "test_glibc_mathf passed\n");
  return fails ? 1 : 0;
}
