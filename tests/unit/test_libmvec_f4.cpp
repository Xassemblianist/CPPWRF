// SPDX-License-Identifier: GPL-3.0-or-later
// The 4-lane float expf / logf / powf / sinf / cosf / atanf of src/core/libmvec_f4.hpp (glibc 2.41 libmvec's SSE4
// variants, powf with Intel's rcpps) against glibc's libmvec (_ZGVbN4v_expf, _ZGVbN4v_logf, _ZGVbN4vv_powf,
// _ZGVbN4v_sinf, _ZGVbN4v_cosf, _ZGVbN4v_atanf), bit for bit (`test_libmvec_f4 all`: every 2^32 input of the
// unary functions):
//  expf, logf, sinf, cosf, atanf: every 61st float bit pattern (about 7*10^7, all signs, exponents, NaN / Inf / denormal classes) and
//              the edges of the range masks; four different arguments per vector call (lanes independent);
//  powf:       x over every 4099th positive bit pattern and the edges, each with 8 exponents y (0, +-1, +-0.5,
//              WRF's rcp = 2/7, cvpm, cpovcv), and 2^22 random (x, y) pairs with x in [1e-3, 1e6) and |y| < 4;
//              special x / y (0, negatives, denormals, Inf, NaN) with random partners.
// sinf / cosf also every 97th bit pattern with |x| <= 2 pi. expf, logf, sinf, cosf and atanf use IEEE operations only
// (any x86-64 CPU); powf's results equal libmvec's only on Intel (rcpps):
// elsewhere the powf differences are reported, not failed. The ports are glibc 2.41's, and their out-of-range lanes
// call glibc 2.41's scalar functions (glibc_mathf_host.cpp) where the system's libmvec calls the system's: on another
// glibc (an Ubuntu node of tools/amd/amd_run.sh) every difference is reported, none fails the test (as
// test_glibc_mathf). The vm:: lane checks fail on any glibc.
#include <cpuid.h>
#include <gnu/libc-version.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/backend.hpp"

#if defined(CPPWRF_HAVE_MVEC_F4)
#include <xmmintrin.h>

extern "C" __m128 _ZGVbN4v_expf(__m128);
extern "C" __m128 _ZGVbN4v_logf(__m128);
extern "C" __m128 _ZGVbN4vv_powf(__m128, __m128);
extern "C" __m128 _ZGVbN4v_sinf(__m128);
extern "C" __m128 _ZGVbN4v_cosf(__m128);
extern "C" __m128 _ZGVbN4v_atanf(__m128);

static std::uint32_t fbits(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}
static float ffrom(std::uint32_t b) {
  float v;
  std::memcpy(&v, &b, sizeof v);
  return v;
}
static bool is_intel() {
  unsigned a, b, c, d;
  char v[13] = {};
  if (!__get_cpuid(0, &a, &b, &c, &d)) return false;
  std::memcpy(v, &b, 4);
  std::memcpy(v + 4, &d, 4);
  std::memcpy(v + 8, &c, 4);
  return std::strcmp(v, "GenuineIntel") == 0;
}
// results equal as bit patterns (any NaN equals any NaN: the scalar fallback's NaN payloads are glibc's own)
static bool same(float a, float b) { return fbits(a) == fbits(b) || (a != a && b != b); }

template <class Port, class Vec>
static long check1(const char* name, const std::vector<std::uint32_t>& in, Port port, Vec vec) {
  long diff = 0;
  for (std::size_t n = 0; n + 4 <= in.size(); n += 4) {
    alignas(16) float x[4], y[4];
    for (int l = 0; l < 4; ++l) x[l] = ffrom(in[n + std::size_t(l)]);
    _mm_store_ps(y, vec(_mm_load_ps(x)));
    for (int l = 0; l < 4; ++l)
      if (!same(port(x[l]), y[l]) && diff++ < 5)
        std::printf("  %s(%a = 0x%08x): port %a, libmvec %a\n", name, double(x[l]), fbits(x[l]), double(port(x[l])),
                    double(y[l]));
  }
  std::printf("%s: %zu inputs, %ld differ\n", name, in.size() / 4 * 4, diff);
  return diff;
}

template <class Port, class Vec>
static long check_all(const char* name, Port port, Vec vec) {  // every float bit pattern, in parallel chunks
  long diff = 0;
#pragma omp parallel for schedule(dynamic, 1) reduction(+ : diff)
  for (std::int64_t c = 0; c < (std::int64_t(1) << 16); ++c)
    for (std::uint32_t k = 0; k < (1u << 16); k += 4) {
      alignas(16) float x[4], y[4];
      for (std::uint32_t l = 0; l < 4; ++l) x[l] = ffrom(std::uint32_t(c << 16) + k + l);
      _mm_store_ps(y, vec(_mm_load_ps(x)));
      for (int l = 0; l < 4; ++l)
        if (!same(port(x[l]), y[l])) ++diff;
    }
  std::printf("%s: all 2^32 inputs, %ld differ\n", name, diff);
  return diff;
}

int main(int argc, char** argv) {
  const bool intel = is_intel();
  const bool strict = std::strcmp(gnu_get_libc_version(), "2.41") == 0;
  if (!strict)
    std::printf("glibc %s, not 2.41: the differences from the system's libmvec are reported, not failed\n",
                gnu_get_libc_version());
  if (argc > 1 && std::strcmp(argv[1], "all") == 0) {
    long d = check_all("expf", [](float x) { return cppwrf::mvec4::expf(x); }, [](__m128 x) { return _ZGVbN4v_expf(x); });
    d += check_all("logf", [](float x) { return cppwrf::mvec4::logf(x); }, [](__m128 x) { return _ZGVbN4v_logf(x); });
    d += check_all("sinf", [](float x) { return cppwrf::mvec4::sinf(x); }, [](__m128 x) { return _ZGVbN4v_sinf(x); });
    d += check_all("cosf", [](float x) { return cppwrf::mvec4::cosf(x); }, [](__m128 x) { return _ZGVbN4v_cosf(x); });
    d += check_all("atanf", [](float x) { return cppwrf::mvec4::atanf(x); }, [](__m128 x) { return _ZGVbN4v_atanf(x); });
    if (!strict) d = 0;
    std::printf("%s\n", d ? "test_libmvec_f4 all FAILED" : "test_libmvec_f4 all passed");
    return d ? 1 : 0;
  }
  std::vector<std::uint32_t> in;
  for (std::uint64_t b = 0; b < (1ull << 32); b += 61) in.push_back(std::uint32_t(b));
  for (std::uint32_t c : {0x00000000u, 0x80000000u, 0x00000001u, 0x007fffffu, 0x00800000u, 0x7f7fffffu, 0x7f800000u,
                          0xff800000u, 0x7fc00000u, 0xffc00000u, 0x7f800001u, 0x3f800000u, 0xbf800000u, 0x42aeac4fu,
                          0x42aeac50u, 0xc2aeac4fu, 0xc2aeac50u, 0x42b17218u, 0xc2cff1b5u, 0x3f2aaaabu, 0x3f2aaaaau})
    for (int r = 0; r < 4; ++r) in.push_back(c);
  long fails = 0;
  fails += check1("expf", in, [](float x) { return cppwrf::mvec4::expf(x); }, [](__m128 x) { return _ZGVbN4v_expf(x); });
  fails += check1("logf", in, [](float x) { return cppwrf::mvec4::logf(x); }, [](__m128 x) { return _ZGVbN4v_logf(x); });
  fails += check1("sinf", in, [](float x) { return cppwrf::mvec4::sinf(x); }, [](__m128 x) { return _ZGVbN4v_sinf(x); });
  fails += check1("cosf", in, [](float x) { return cppwrf::mvec4::cosf(x); }, [](__m128 x) { return _ZGVbN4v_cosf(x); });
  fails += check1("atanf", in, [](float x) { return cppwrf::mvec4::atanf(x); }, [](__m128 x) { return _ZGVbN4v_atanf(x); });
  {  // sin / cos: the arguments of WRF's loops (|x| <= 2 pi: latitudes, hour angles) more densely
    std::vector<std::uint32_t> t;
    for (std::uint32_t b = 0; b <= 0x40c90fdbu; b += 97) { t.push_back(b); t.push_back(b | 0x80000000u); }
    fails += check1("sinf |x|<=2pi", t, [](float x) { return cppwrf::mvec4::sinf(x); }, [](__m128 x) { return _ZGVbN4v_sinf(x); });
    fails += check1("cosf |x|<=2pi", t, [](float x) { return cppwrf::mvec4::cosf(x); }, [](__m128 x) { return _ZGVbN4v_cosf(x); });
  }

  std::uint64_t s = 0;
  auto sm = [&s] {  // splitmix64
    std::uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
  };
  std::vector<float> px, py;
  const float ys[8] = {0.f, 1.f, -1.f, 0.5f, -0.5f, 287.f / (7.f * 287.f / 2.f),
                       -(7.f * 287.f / 2.f - 287.f) / (7.f * 287.f / 2.f), (7.f * 287.f / 2.f) / (7.f * 287.f / 2.f - 287.f)};
  for (std::uint64_t b = 0; b < 0x80000000ull; b += 4099)
    for (float y : ys) { px.push_back(ffrom(std::uint32_t(b))); py.push_back(y); }
  for (int n = 0; n < (1 << 22); ++n) {
    const double u = double(sm() >> 11) * 0x1p-53, v = double(sm() >> 11) * 0x1p-53;
    px.push_back(float(1e-3 * __builtin_pow(1e9, u)));
    py.push_back(float(8.0 * v - 4.0));
  }
  for (std::uint32_t c : {0x00000000u, 0x80000000u, 0x00000001u, 0x007fffffu, 0x00800000u, 0x7f7fffffu, 0x7f800000u,
                          0xff800000u, 0x7fc00000u, 0xbf800000u, 0xc0000000u, 0x3f800000u})
    for (int n = 0; n < 64; ++n) {
      const float r = ffrom(std::uint32_t(sm()));
      px.push_back(ffrom(c)); py.push_back(r);
      px.push_back(r); py.push_back(ffrom(c));
      px.push_back(float(double(sm() >> 11) * 0x1p-53 * 100.0)); py.push_back(ffrom(c));
    }
  for (int n = 0; n < 1 << 16; ++n) {  // overflow / underflow region of y*log(x)
    px.push_back(float(1.0 + double(sm() >> 11) * 0x1p-53 * 1e3));
    py.push_back(float((double(sm() >> 11) * 0x1p-53 - 0.5) * 60.0));
  }
  long dpow = 0;
  for (std::size_t n = 0; n + 4 <= px.size(); n += 4) {
    alignas(16) float y[4];
    _mm_store_ps(y, _ZGVbN4vv_powf(_mm_loadu_ps(&px[n]), _mm_loadu_ps(&py[n])));
    for (int l = 0; l < 4; ++l) {
      const float p = cppwrf::mvec4::powf(px[n + std::size_t(l)], py[n + std::size_t(l)]);
      if (!same(p, y[l]) && dpow++ < 5)
        std::printf("  powf(%a, %a): port %a, libmvec %a\n", double(px[n + std::size_t(l)]),
                    double(py[n + std::size_t(l)]), double(p), double(y[l]));
    }
  }
  std::printf("powf: %zu inputs, %ld differ (%s CPU)\n", px.size() / 4 * 4, dpow, intel ? "Intel" : "non-Intel");
  if (intel) fails += dpow;
  if (!strict) fails = 0;  // (another glibc: reported above)
  // the vm:: lanes: iterations 0 ... 4*(n/4)-1 vector, the rest scalar
  for (int n = 0; n < 12; ++n)
    for (int i = 0; i < n; ++i)
      if (cppwrf::vm::lane4(i, n) != (i < n / 4 * 4)) ++fails;
  std::printf("%s\n", fails ? "test_libmvec_f4 FAILED" : "test_libmvec_f4 passed");
  return fails ? 1 : 0;
}
#else
int main() {
  std::printf("test_libmvec_f4: no x86-64 CPU build (skipped)\n");
  return 0;
}
#endif
