// SPDX-License-Identifier: GPL-3.0-or-later
// The math library as the model calls it, on special inputs, against this machine's libm / libmvec bit for bit:
// +-0, subnormals, +-DBL_MIN / DBL_MAX, +-inf, quiet and signalling NaNs, |x| just above and below 1, negative
// integers and half-integers, the overflow / underflow edges of exp, pow and tgamma. glibc's public asin, acos,
// log10 and tgamma (and asinf ...) are SVID wrappers that return +NaN on a domain error where the kernels return
// -NaN (core/glibc_math_compat.hpp, core/glibc_mathf_compat.hpp); every other function returns its kernel's value.
// Checked (host): cppwrf::libm's double functions as the CUDA / HIP FP64 build's host computes them
// (CPPWRF_GLIBC_MATH, set here in every build), libmvec's 2-lane exp / pow / sin / cos / atan and the vector log /
// log10 of core/libmvec_x86.hpp, cppwrf::vm::log10, glibc's float functions (core/glibc_mathf.hpp and the wrappers)
// and the C functions an FP32 executable defines (core/glibc_mathf_host.cpp, linked into this test), libmvec's 4-lane
// float functions. The vector log / log10 and 4-lane powf lanes that take rcpps (CPU-defined) are compared on Intel
// only, where WRF's references are made (elsewhere: SKIP, counted); their scalar-callout lanes and every other
// function are compared on any CPU. GPU builds: the same functions on the device against the
// system libm. A result must have the same bits; when an input is a NaN only NaN-ness is compared (the operand whose
// NaN propagates is not fixed by C, and the device returns its canonical NaN). Meaningful where WRF's references are
// made (glibc 2.41, x86-64): another glibc reports the differences and fails.
#if !defined(CPPWRF_GLIBC_MATH)
#define CPPWRF_GLIBC_MATH 1  // the FP64 GPU build's host path, here also in the CPU builds
#endif
#include <dlfcn.h>
#include <gnu/libc-version.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include "core/backend.hpp"
#include "core/glibc_math_compat.hpp"
#include "core/glibc_mathf_compat.hpp"

#if defined(__x86_64__)
#include <cpuid.h>
#include <emmintrin.h>
#include <xmmintrin.h>
extern "C" __m128d _ZGVbN2v_exp(__m128d);
extern "C" __m128d _ZGVbN2vv_pow(__m128d, __m128d);
extern "C" __m128d _ZGVbN2v_log(__m128d);
extern "C" __m128d _ZGVbN2v_log10(__m128d);
extern "C" __m128d _ZGVbN2v_sin(__m128d);
extern "C" __m128d _ZGVbN2v_cos(__m128d);
extern "C" __m128d _ZGVbN2v_atan(__m128d);
#if defined(CPPWRF_HAVE_MVEC_F4)
extern "C" __m128 _ZGVbN4v_expf(__m128);
extern "C" __m128 _ZGVbN4v_logf(__m128);
extern "C" __m128 _ZGVbN4vv_powf(__m128, __m128);
extern "C" __m128 _ZGVbN4v_sinf(__m128);
extern "C" __m128 _ZGVbN4v_cosf(__m128);
extern "C" __m128 _ZGVbN4v_atanf(__m128);
#endif

namespace {
// libmvec's vector log / log10 and 4-lane powf take rcpps, whose result the CPU defines: the ports reproduce Intel's
// (core/libmvec_x86.hpp, libmvec_f4.hpp; WRF's references are made on Intel). Their lanes that use it are compared
// with this machine's libmvec only on Intel (elsewhere counted as skipped); their scalar-callout lanes always.
bool is_intel() {
  if (const char* e = std::getenv("CPPWRF_TEST_NOT_INTEL"); e && *e == '1') return false;  // the other CPUs' path
  unsigned a, b, c, d;
  if (!__get_cpuid(0, &a, &b, &c, &d)) return false;
  char v[13];
  std::memcpy(v, &b, 4);
  std::memcpy(v + 4, &d, 4);
  std::memcpy(v + 8, &c, 4);
  v[12] = 0;
  return std::strcmp(v, "GenuineIntel") == 0;
}
const bool g_intel = is_intel();
long g_skip = 0;
// the lanes of _ZGVbN2v_log / log10 that do not call the scalar function: x in [DBL_MIN, DBL_MAX]
bool log_rcpps_lane(double x) { return x >= DBL_MIN && x <= DBL_MAX; }
// the lanes of _ZGVbN4vv_powf past its special_x / special_y masks (libmvec_f4.hpp: x in [FLT_MIN, FLT_MAX), y finite)
bool powf_rcpps_lane(float x, float y) { return x >= FLT_MIN && x < FLT_MAX && std::isfinite(y); }
std::uint64_t bits(double v) {
  std::uint64_t b;
  std::memcpy(&b, &v, 8);
  return b;
}
std::uint32_t bits(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, 4);
  return b;
}
double dbl(std::uint64_t b) {
  double v;
  std::memcpy(&v, &b, 8);
  return v;
}
float flt(std::uint32_t b) {
  float v;
  std::memcpy(&v, &b, 4);
  return v;
}

long g_n = 0, g_bad = 0;
template <class T>
void check(const char* name, T x, T y, T got, T want, bool pair = false) {
  ++g_n;
  const bool nan_in = std::isnan(x) || (pair && std::isnan(y));
  const bool ok = nan_in ? std::isnan(got) == std::isnan(want) : bits(got) == bits(want);
  if (ok) return;
  if (g_bad++ < 40) {
    if (pair)
      std::printf("  %s(%a, %a): port %a (0x%llx) libm %a (0x%llx)\n", name, double(x), double(y), double(got),
                  (unsigned long long)bits(got), double(want), (unsigned long long)bits(want));
    else
      std::printf("  %s(%a): port %a (0x%llx) libm %a (0x%llx)\n", name, double(x), double(got),
                  (unsigned long long)bits(got), double(want), (unsigned long long)bits(want));
  }
}

// the system libm's own functions (looked up in libm.so.6: this program defines asinf ... itself)
void* libm_handle() {
  static void* h = dlopen("libm.so.6", RTLD_NOW | RTLD_LOCAL);
  return h;
}
template <class F>
F sys(const char* name) {
  void* f = libm_handle() ? dlsym(libm_handle(), name) : nullptr;
  if (f == nullptr) {
    std::printf("%s: not found in libm.so.6\n", name);
    ++g_bad;
  }
  return reinterpret_cast<F>(f);
}
// a function of this program (not of a shared library)
bool own(void* f) {
  Dl_info in{}, self{};
  return dladdr(f, &in) && dladdr(reinterpret_cast<void*>(&libm_handle), &self) && in.dli_fbase == self.dli_fbase;
}

std::vector<double> special_doubles() {
  const double inf = HUGE_VAL;
  std::vector<double> sp = {0.0, DBL_TRUE_MIN, 2 * DBL_TRUE_MIN, DBL_MIN / 3, DBL_MIN, 0x1p-60, 0x1p-27, 0.25,
                            0.5, 0.75, 1.0 - 0x1p-53, 1.0, 1.0 + 0x1p-52, 1.0001, 1.5, 2.0, 3.0, 10.0, 100.0,
                            171.5, 171.7, 172.0, 184.0, 700.0, 709.8, 710.0, 745.2, 750.0, 1024.0, 1075.0, 1e10,
                            1e300, DBL_MAX, inf, dbl(0x7ff8000000000000ull), dbl(0x7ff4000000000000ull),
                            dbl(0x7ff8000000000123ull)};
  std::vector<double> xs;
  for (double x : sp) {
    xs.push_back(x);
    xs.push_back(-x);
  }
  for (int k = 1; k <= 200; ++k) {  // tgamma's poles, its sign changes and its underflow below -184
    xs.push_back(-double(k));
    xs.push_back(-double(k) - 0.5);
    xs.push_back(std::nextafter(-double(k), 0.0));
  }
  return xs;
}
std::vector<float> special_floats() {
  std::vector<float> xf;
  for (double x : special_doubles()) xf.push_back(float(x));
  for (float x : {FLT_TRUE_MIN, FLT_MIN, FLT_MAX, 1.0f + FLT_EPSILON, 1.0f - FLT_EPSILON / 2, 35.04f, 35.1f, 36.0f,
                  88.7f, 89.0f, 104.0f, 0x1p-24f}) {
    xf.push_back(x);
    xf.push_back(-x);
  }
  for (std::uint32_t b : {0x7fc00000u, 0xffc00000u, 0x7fa00000u, 0x7fc00123u}) xf.push_back(flt(b));
  return xf;
}

// the functions the device computes (also on the host, for the device comparison)
constexpr int kDevD = 6, kDevF = 5;
const char* kDevDName[kDevD] = {"libm::asin", "libm::acos", "libm::log10", "libm::tgamma", "mvec::log10",
                                "vm::log10"};
const char* kDevFName[kDevF] = {"compat::asinf", "compat::acosf", "compat::log10f", "compat::tgammaf", "vm::log10f"};
CPPWRF_HD inline double dev_d(int fn, double x) {
  switch (fn) {
    case 0: return cppwrf::libm::asin(x);
    case 1: return cppwrf::libm::acos(x);
    case 2: return cppwrf::libm::log10(x);
    case 3: return cppwrf::libm::tgamma(x);
    case 4: return cppwrf::mvec::log10(x);
    default: return cppwrf::vm::log10(x, 0, 1);  // a 1-trip loop: the scalar log10
  }
}
CPPWRF_HD inline float dev_f(int fn, float x) {
  namespace c = cppwrf::glibcf::compat;
  switch (fn) {
    case 0: return c::asinf(x);
    case 1: return c::acosf(x);
    case 2: return c::log10f(x);
    case 3: return c::tgammaf(x);
    default: return cppwrf::vm::log10(x, 0, 1);  // libm's log10f (FP32 builds)
  }
}
#if defined(CPPWRF_GPUCC)
__global__ void kdev(int n, const double* xd, const float* xf, double* yd, float* yf) {
  const int t = blockIdx.x * blockDim.x + threadIdx.x;
  if (t >= n) return;
  for (int fn = 0; fn < kDevD; ++fn) yd[fn * n + t] = dev_d(fn, xd[t]);
  for (int fn = 0; fn < kDevF; ++fn) yf[fn * n + t] = dev_f(fn, xf[t]);
}
#endif
}  // namespace

int main() {
  const bool strict = std::strcmp(gnu_get_libc_version(), "2.41") == 0;
  const std::vector<double> xs = special_doubles();
  const std::vector<float> xf = special_floats();

  // ---- double: cppwrf::libm (glibc's algorithms and wrappers) against libm
  using D1 = double (*)(double);
  using D2 = double (*)(double, double);
  struct U {
    const char* name;
    D1 port;
  };
  const U unary[] = {
      {"asin", [](double x) { return cppwrf::libm::asin(x); }},
      {"acos", [](double x) { return cppwrf::libm::acos(x); }},
      {"log10", [](double x) { return cppwrf::libm::log10(x); }},
      {"tgamma", [](double x) { return cppwrf::libm::tgamma(x); }},
      {"exp", [](double x) { return cppwrf::libm::exp(x); }},
      {"log", [](double x) { return cppwrf::libm::log(x); }},
      {"atan", [](double x) { return cppwrf::libm::atan(x); }},
      {"tanh", [](double x) { return cppwrf::libm::tanh(x); }},
      {"sin", [](double x) { return cppwrf::libm::sin(x); }},
      {"cos", [](double x) { return cppwrf::libm::cos(x); }},
      {"sinh", [](double x) { return cppwrf::libm::sinh(x); }},
      {"erf", [](double x) { return cppwrf::libm::erf(x); }},
      {"tan", [](double x) { return cppwrf::libm::tan(x); }},
  };
  for (const U& u : unary) {
    D1 volatile s = sys<D1>(u.name);
    if (s == nullptr) continue;
    for (double x : xs) check(u.name, x, 0.0, u.port(x), s(x));
  }
  {
    D2 volatile s_atan2 = sys<D2>("atan2"), s_hypot = sys<D2>("hypot"), s_pow = sys<D2>("pow");
    if (s_atan2 && s_hypot && s_pow)
      for (double x : xs)
        for (double y : xs) {
          check("atan2", x, y, cppwrf::libm::atan2(x, y), s_atan2(x, y), true);
          check("hypot", x, y, cppwrf::libm::hypot(x, y), s_hypot(x, y), true);
          check("pow", x, y, cppwrf::libm::pow(x, y), s_pow(x, y), true);
          double o[2];  // libmvec pow, distinct lanes
          _mm_storeu_pd(o, _ZGVbN2vv_pow(_mm_set_pd(y, x), _mm_set_pd(x, y)));
          check("mvec pow", x, y, cppwrf::glibc::mvec::pow(x, y), o[0], true);
          check("mvec pow", y, x, cppwrf::glibc::mvec::pow(y, x), o[1], true);
        }
  }
  // ---- libmvec's 2-lane functions (lane 0; lane 1 holds -x)
  struct V {
    const char* name;
    __m128d (*vec)(__m128d);
    D1 port;
    bool rcpps;  // core/libmvec_x86.hpp: Intel's rcpps in the normal range (log_rcpps_lane)
  };
  const V vec[] = {
      {"mvec exp", _ZGVbN2v_exp, [](double x) { return cppwrf::glibc::mvec::exp(x); }, false},
      {"mvec sin", _ZGVbN2v_sin, [](double x) { return cppwrf::glibc::mvec::sin(x); }, false},
      {"mvec cos", _ZGVbN2v_cos, [](double x) { return cppwrf::glibc::mvec::cos(x); }, false},
      {"mvec atan", _ZGVbN2v_atan, [](double x) { return cppwrf::glibc::mvec::atan(x); }, false},
      {"mvec log", _ZGVbN2v_log, [](double x) { return cppwrf::mvec::log(x); }, true},
      {"mvec log10", _ZGVbN2v_log10, [](double x) { return cppwrf::mvec::log10(x); }, true},
      {"vm log10", _ZGVbN2v_log10, [](double x) { return cppwrf::vm::log10(x, 0, 2); }, true},
  };
  for (const V& v : vec)
    for (double x : xs) {
      double o[2];
      _mm_storeu_pd(o, v.vec(_mm_set_pd(-x, x)));
      for (int l = 0; l < 2; ++l) {
        const double xl = l ? -x : x;
        if (v.rcpps && log_rcpps_lane(xl) && !g_intel) {
          ++g_skip;
          continue;
        }
        check(v.name, xl, 0.0, v.port(xl), o[l]);
      }
    }
  {  // the scalar log10 of a trailing iteration
    D1 volatile s = sys<D1>("log10");
    if (s)
      for (double x : xs) check("vm log10 (scalar)", x, 0.0, cppwrf::vm::log10(x, 0, 1), s(x));
  }

  // ---- float: glibc's float functions (kernels and wrappers), this program's C functions, libmvec's 4 lanes
  using F1 = float (*)(float);
  using F2 = float (*)(float, float);
  struct UF {
    const char* name;
    F1 port;
    F1 program;  // the C function this program defines (core/glibc_mathf_host.cpp)
  };
  namespace g = cppwrf::glibcf;
  const UF unaryf[] = {
      {"asinf", [](float x) { return g::compat::asinf(x); }, ::asinf},
      {"acosf", [](float x) { return g::compat::acosf(x); }, ::acosf},
      {"log10f", [](float x) { return g::compat::log10f(x); }, ::log10f},
      {"tgammaf", [](float x) { return g::compat::tgammaf(x); }, ::tgammaf},
      {"expf", [](float x) { return g::expf(x); }, ::expf},
      {"logf", [](float x) { return g::logf(x); }, ::logf},
      {"sinf", [](float x) { return g::sinf(x); }, ::sinf},
      {"cosf", [](float x) { return g::cosf(x); }, ::cosf},
      {"tanf", [](float x) { return g::tanf(x); }, ::tanf},
      {"atanf", [](float x) { return g::atanf(x); }, ::atanf},
      {"tanhf", [](float x) { return g::tanhf(x); }, ::tanhf},
      {"sinhf", [](float x) { return g::sinhf(x); }, ::sinhf},
      {"erff", [](float x) { return g::erff(x); }, ::erff},
  };
  for (const UF& u : unaryf) {
    F1 volatile s = sys<F1>(u.name);
    if (s == nullptr) continue;
    F1 volatile p = u.program;
    if (!own(reinterpret_cast<void*>(u.program))) {
      std::printf("%s: libm's, not this program's (core/glibc_mathf_host.cpp not linked)\n", u.name);
      ++g_bad;
    }
    for (float x : xf) {
      check(u.name, x, 0.0f, u.port(x), s(x));
      check(u.name, x, 0.0f, p(x), s(x));
    }
  }
  {
    F2 volatile s_powf = sys<F2>("powf"), s_atan2f = sys<F2>("atan2f");
    F2 volatile p_powf = ::powf, p_atan2f = ::atan2f;
    if (s_powf && s_atan2f)
      for (float x : xf)
        for (float y : xf) {
          check("powf", x, y, g::powf(x, y), s_powf(x, y), true);
          check("powf", x, y, p_powf(x, y), s_powf(x, y), true);
          check("atan2f", x, y, g::atan2f(x, y), s_atan2f(x, y), true);
          check("atan2f", x, y, p_atan2f(x, y), s_atan2f(x, y), true);
#if defined(CPPWRF_HAVE_MVEC_F4)
          float o[4];
          _mm_storeu_ps(o, _ZGVbN4vv_powf(_mm_set_ps(1.0f, 1.0f, y, x), _mm_set_ps(1.0f, 1.0f, x, y)));
          if (g_intel || !powf_rcpps_lane(x, y)) check("v4 powf", x, y, cppwrf::mvec4::powf(x, y), o[0], true);
          else ++g_skip;
          if (g_intel || !powf_rcpps_lane(y, x)) check("v4 powf", y, x, cppwrf::mvec4::powf(y, x), o[1], true);
          else ++g_skip;
#endif
        }
  }
  {  // libm's log10f as the FP32 builds' vectorised loops call it (cppwrf::vm::log10(float))
    F1 volatile s = sys<F1>("log10f");
    if (s)
      for (float x : xf) check("vm log10f", x, 0.0f, cppwrf::vm::log10(x, 0, 1), s(x));
  }
#if defined(CPPWRF_HAVE_MVEC_F4)
  struct VF {
    const char* name;
    __m128 (*vec)(__m128);
    F1 port;
  };
  const VF vecf[] = {
      {"v4 expf", _ZGVbN4v_expf, [](float x) { return cppwrf::mvec4::expf(x); }},
      {"v4 logf", _ZGVbN4v_logf, [](float x) { return cppwrf::mvec4::logf(x); }},
      {"v4 sinf", _ZGVbN4v_sinf, [](float x) { return cppwrf::mvec4::sinf(x); }},
      {"v4 cosf", _ZGVbN4v_cosf, [](float x) { return cppwrf::mvec4::cosf(x); }},
      {"v4 atanf", _ZGVbN4v_atanf, [](float x) { return cppwrf::mvec4::atanf(x); }},
  };
  for (const VF& v : vecf)
    for (float x : xf) {
      float o[4];
      _mm_storeu_ps(o, v.vec(_mm_set_ps(1.0f, 0.5f, -x, x)));
      check(v.name, x, 0.0f, v.port(x), o[0]);
      check(v.name, -x, 0.0f, v.port(-x), o[1]);
    }
#endif
  const long host_n = g_n, host_bad = g_bad;
  std::printf("host: %ld results, %ld differ from libm %s\n", host_n, host_bad, gnu_get_libc_version());
  if (g_skip)
    std::printf("SKIP %ld libmvec lanes that use rcpps (log, log10, 4-lane powf): not an Intel CPU, the ports are "
                "Intel's (test_libmvec / test_libmvec_f4 check them against Intel's results)\n", g_skip);

#if defined(CPPWRF_GPUCC)
  {  // the device against the system libm (and the host ports)
    const int n = int(std::min(xs.size(), xf.size()));
    double *dxd = nullptr, *dyd = nullptr;
    float *dxf = nullptr, *dyf = nullptr;
    if (gpuMalloc(&dxd, n * sizeof(double)) != gpuSuccess) {
      std::printf("no " CPPWRF_GPU_API_NAME " device: the device check is skipped\n");
    } else {
      gpuMalloc(&dyd, kDevD * n * sizeof(double));
      gpuMalloc(&dxf, n * sizeof(float));
      gpuMalloc(&dyf, kDevF * n * sizeof(float));
      gpuMemcpy(dxd, xs.data(), n * sizeof(double), gpuMemcpyHostToDevice);
      gpuMemcpy(dxf, xf.data(), n * sizeof(float), gpuMemcpyHostToDevice);
      kdev<<<unsigned((n + 127) / 128), 128>>>(n, dxd, dxf, dyd, dyf);
      std::vector<double> yd(std::size_t(kDevD) * n);
      std::vector<float> yf(std::size_t(kDevF) * n);
      gpuMemcpy(yd.data(), dyd, yd.size() * sizeof(double), gpuMemcpyDeviceToHost);
      gpuMemcpy(yf.data(), dyf, yf.size() * sizeof(float), gpuMemcpyDeviceToHost);
      gpuFree(dxd);
      gpuFree(dyd);
      gpuFree(dxf);
      gpuFree(dyf);
      if (gpuGetLastError() != gpuSuccess) {
        std::printf(CPPWRF_GPU_API_NAME " error: the device check fails\n");
        return 1;
      }
      const char* sd[kDevD] = {"asin", "acos", "log10", "tgamma", "log10", "log10"};
      const char* sf[kDevF] = {"asinf", "acosf", "log10f", "tgammaf", "log10f"};
      for (int fn = 0; fn < kDevD; ++fn) {
        D1 volatile s = sys<D1>(sd[fn]);
        for (int i = 0; s && i < n; ++i) {
          const double x = xs[std::size_t(i)];
          if (fn == 4 && !(x < DBL_MIN) && !(x > DBL_MAX) && !std::isnan(x)) continue;  // vector lanes
          check(kDevDName[fn], x, 0.0, yd[std::size_t(fn) * n + i], s(x));
        }
      }
      for (int fn = 0; fn < kDevF; ++fn) {
#if !defined(CPPWRF_FP32_IDENTITY)
        if (fn == 4) continue;  // vm::log10(float) on the device: the device's log10f outside the identity build
#endif
        F1 volatile s = sys<F1>(sf[fn]);
        for (int i = 0; s && i < n; ++i)
          check(kDevFName[fn], xf[std::size_t(i)], 0.0f, yf[std::size_t(fn) * n + i], s(xf[std::size_t(i)]));
      }
      std::printf("device: %ld results, %ld differ from libm\n", g_n - host_n, g_bad - host_bad);
    }
  }
#endif
  if (!strict) {
    std::printf("test_libm_special FAILED: glibc %s, not 2.41 (the version WRF's references use)\n",
                gnu_get_libc_version());
    return 1;
  }
  std::printf(g_bad ? "test_libm_special FAILED\n" : "test_libm_special passed\n");
  return g_bad ? 1 : 0;
}
#else
int main() {
  std::printf("test_libm_special: not x86-64, skipped\n");
  return 0;
}
#endif
