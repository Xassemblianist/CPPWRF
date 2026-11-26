// SPDX-License-Identifier: GPL-3.0-or-later
// glibc 2.41 libmvec's SSE4 acos / asin / tan (2 doubles) and acosf / asinf / tanf (4 floats) of
// src/core/libmvec_sse4_trig.hpp (the routines instruction by instruction with Intel's rsqrtps / rcpps; used by the
// WRF reference runs through tests/phys/libmvec_intel.cpp):
//  (a) on any x86-64 CPU with glibc 2.41 the ports' results over a fixed input set hash to the values glibc's
//      libmvec gives on Intel (`test_libmvec_trig --hash` on an Intel CPU prints them);
//  (b) on Intel also: every port equals libmvec's _ZGVbN2v_acos ... _ZGVbN4v_tanf bit for bit on that set (NaN
//      payloads included), and the rsqrtps / rcpps emulation equals the CPU's for every 13th float bit pattern
//      (`test_libmvec_trig all`: every float input of the float functions and of rsqrtps / rcpps);
//  (c) elsewhere (AMD) the test reports how many of libmvec's own results and of the CPU's rsqrtps / rcpps differ
//      (information, no failure). Run with LD_PRELOAD=libmvec_intel.so (tests/phys/libmvec_intel.cpp as a shared
//      object), "libmvec" is the preload: on AMD it then differs for none and its hash is Intel's.
// The double set: the special classes (zeros, denormals, DBL_MIN / MAX, +-Inf, quiet / signalling NaNs of both
// signs), |x| at and next to 0.5 and 1, 1 - 2^-k (Y = 0.5 - 0.5|x| down to the SmallNorm mask 2^-255), every rsqrtps
// input class of Y (float exponents 2^-60 ... 2^-2, 10-bit classes, the double neighbours of the class edges),
// 2^20 uniform values in [-1, 1], tan's main / large-argument edge (|x| = 128000) and its neighbours, 2^18 values
// in [-2e5, 2e5], 2^18 huge values (exponents 2^17 ... 2^1023), the neighbours of 2^16 multiples of pi/2, 2^18
// random bit patterns. The float set: every 61st bit pattern and the edges (`all`: every bit pattern).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/backend.hpp"

#if defined(CPPWRF_HAVE_LIBMVEC)
#include <cpuid.h>
#include <gnu/libc-version.h>
#include <xmmintrin.h>

#include "core/libmvec_sse4_trig.hpp"

extern "C" __m128d _ZGVbN2v_acos(__m128d);
extern "C" __m128d _ZGVbN2v_asin(__m128d);
extern "C" __m128d _ZGVbN2v_tan(__m128d);
extern "C" __m128 _ZGVbN4v_acosf(__m128);
extern "C" __m128 _ZGVbN4v_asinf(__m128);
extern "C" __m128 _ZGVbN4v_tanf(__m128);

namespace ms = cppwrf::mvec_sse4;

static std::uint64_t bits_of(double v) {
  std::uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}
static double from_bits(std::uint64_t b) {
  double v;
  std::memcpy(&v, &b, sizeof v);
  return v;
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

static std::vector<double> dinputs() {
  std::uint64_t s = 0;
  auto sm = [&s] {  // splitmix64
    std::uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
  };
  auto u01 = [&sm] { return double(sm() >> 11) * 0x1p-53; };
  std::vector<double> x;
  auto both = [&x](double v) {
    x.push_back(v);
    x.push_back(-v);
  };
  for (std::uint64_t b : {0x0ull, 0x1ull, 0x000fffffffffffffull, 0x0010000000000000ull, 0x0010000000000001ull,
                          0x7fefffffffffffffull, 0x7ff0000000000000ull, 0x7ff8000000000000ull, 0x7ff0000000000001ull,
                          0x7ff4000000000000ull, 0x7fffffffffffffffull, 0x3ff0000000000000ull, 0x3fefffffffffffffull,
                          0x3ff0000000000001ull, 0x3fe0000000000000ull, 0x3fdfffffffffffffull, 0x3fe0000000000001ull,
                          0x3000000000000000ull, 0x2fffffffffffffffull, 0x3e40000000000000ull, 0x40fF400000000000ull,
                          0x40ff3fffffffffffull, 0x40ff400000000001ull, 0x3eb0000000000000ull, 0x3eafffffffffffffull})
    both(from_bits(b));
  for (int k = 1; k <= 60; ++k) both(1.0 - __builtin_ldexp(1.0, -k));
  for (int k = 1; k <= 60; ++k) both(1.0 - 3.0 * __builtin_ldexp(1.0, -k - 2));
  // every rsqrtps class of Y = 0.5 - 0.5|x| = float(Y): |x| = 1 - 2Y at and next to the class edges
  for (std::uint32_t fe = 67; fe <= 125; ++fe)
    for (std::uint32_t t = 0; t < 1024; ++t) {
      const double y = double(ffrom(fe << 23 | t << 13));
      for (double v : {y, __builtin_nextafter(y, 0.0), __builtin_nextafter(y, 1.0), y * (1.0 + 0x1p-25),
                       y * (1.0 - 0x1p-25)})
        both(1.0 - 2.0 * v);
    }
  for (int i = 0; i < 1 << 20; ++i) x.push_back(2.0 * u01() - 1.0);
  for (int i = 0; i < 1 << 12; ++i) {  // around tan's large-argument edge 128000
    const double v = 128000.0 * (1.0 + (u01() - 0.5) * 0x1p-30);
    both(v);
  }
  for (int i = 0; i < 1 << 18; ++i) x.push_back(4e5 * (u01() - 0.5));
  for (int i = 0; i < 1 << 18; ++i)
    x.push_back(from_bits((sm() & 0x800fffffffffffffull) | (std::uint64_t(1040 + sm() % 1007) << 52)));
  for (int i = 0; i < 1 << 16; ++i) {  // near multiples of pi/2, small and large
    const double k = double(sm() % (i & 1 ? 1000000000ull : 100000ull));
    const double v = k * 1.5707963267948966;
    both(v);
    both(__builtin_nextafter(v, 1e300));
  }
  for (int i = 0; i < 1 << 18; ++i) x.push_back(from_bits(sm()));
  if (x.size() % 2) x.push_back(0.25);
  return x;
}

static std::vector<std::uint32_t> finputs(std::uint64_t step) {
  std::vector<std::uint32_t> in;
  for (std::uint64_t b = 0; b < (1ull << 32); b += step) in.push_back(std::uint32_t(b));
  for (std::uint32_t c : {0x00000000u, 0x80000000u, 0x00000001u, 0x007fffffu, 0x00800000u, 0x7f7fffffu, 0x7f800000u,
                          0xff800000u, 0x7fc00000u, 0xffc00000u, 0x7f800001u, 0xffa00000u, 0x3f800000u, 0xbf800000u,
                          0x3f7fffffu, 0x3f800001u, 0x3f000000u, 0x3effffffu, 0x3f000001u, 0x2f800000u, 0x461c4000u,
                          0x461c4001u, 0x461c3fffu, 0x47c35000u})
    for (std::uint32_t d = 0; d < 4; ++d) {
      in.push_back(c + d);
      in.push_back((c - d) ^ 0x80000000u);
    }
  while (in.size() % 4) in.push_back(0x3e800000u);
  return in;
}

struct Res {
  std::uint64_t hash_port = 0xcbf29ce484222325ull, hash_mvec = 0xcbf29ce484222325ull;
  long differ = 0;
};
static void fold(std::uint64_t& h, std::uint64_t v) { h = (h ^ v) * 0x100000001b3ull; }

template <class Port, class Vec>
static Res run_d(const char* name, const std::vector<double>& x, Port port, Vec vec) {
  Res r;
  long shown = 0;
  for (std::size_t i = 0; i + 2 <= x.size(); i += 2) {
    const __m128d v = _mm_set_pd(x[i + 1], x[i]);
    alignas(16) double p[2], m[2];
    _mm_store_pd(p, _mm_castsi128_pd(port(_mm_castpd_si128(v))));
    _mm_store_pd(m, vec(v));
    for (int l = 0; l < 2; ++l) {
      fold(r.hash_port, bits_of(p[l]));
      fold(r.hash_mvec, bits_of(m[l]));
      if (bits_of(p[l]) != bits_of(m[l])) {
        ++r.differ;
        if (shown++ < 3)
          std::printf("  %s(%a = 0x%016llx): port %a, libmvec %a\n", name, x[i + std::size_t(l)],
                      (unsigned long long)bits_of(x[i + std::size_t(l)]), p[l], m[l]);
      }
    }
  }
  return r;
}

template <class Port, class Vec>
static Res run_f(const char* name, const std::vector<std::uint32_t>& x, Port port, Vec vec) {
  Res r;
  long shown = 0;
  for (std::size_t i = 0; i + 4 <= x.size(); i += 4) {
    alignas(16) std::uint32_t a[4], p[4], m[4];
    std::memcpy(a, &x[i], sizeof a);
    const __m128 v = _mm_load_ps(reinterpret_cast<const float*>(a));
    _mm_store_si128(reinterpret_cast<__m128i*>(p), port(_mm_castps_si128(v)));
    _mm_store_ps(reinterpret_cast<float*>(m), vec(v));
    for (int l = 0; l < 4; ++l) {
      fold(r.hash_port, p[l]);
      fold(r.hash_mvec, m[l]);
      if (p[l] != m[l]) {
        ++r.differ;
        if (shown++ < 3)
          std::printf("  %s(0x%08x): port 0x%08x, libmvec 0x%08x\n", name, a[l], p[l], m[l]);
      }
    }
  }
  return r;
}

// every float bit pattern, in parallel chunks: the number of differing lanes
template <class Port, class Vec>
static long run_f_all(Port port, Vec vec) {
  long diff = 0;
#pragma omp parallel for schedule(dynamic, 1) reduction(+ : diff)
  for (std::int64_t c = 0; c < (std::int64_t(1) << 16); ++c)
    for (std::uint32_t k = 0; k < (1u << 16); k += 4) {
      alignas(16) std::uint32_t a[4], p[4], m[4];
      for (std::uint32_t l = 0; l < 4; ++l) a[l] = std::uint32_t(c << 16) + k + l;
      const __m128 v = _mm_load_ps(reinterpret_cast<const float*>(a));
      _mm_store_si128(reinterpret_cast<__m128i*>(p), port(_mm_castps_si128(v)));
      _mm_store_ps(reinterpret_cast<float*>(m), vec(v));
      for (int l = 0; l < 4; ++l) diff += p[l] != m[l];
    }
  return diff;
}

// the rsqrtps / rcpps emulation against this CPU's instructions, every step-th float bit pattern
static void emu_vs_cpu(std::uint64_t step, long& drs, long& drc) {
  long a = 0, b = 0;
#pragma omp parallel for schedule(dynamic, 1) reduction(+ : a, b)
  for (std::int64_t c = 0; c < (std::int64_t(1) << 16); ++c)
    for (std::uint64_t k = std::uint64_t(c) << 16; k < (std::uint64_t(c) + 1) << 16; k += step) {
      const std::uint32_t w = std::uint32_t(k);
      const __m128i v = _mm_set1_epi32(int(w));
      const std::uint32_t hs = std::uint32_t(_mm_cvtsi128_si32(_mm_castps_si128(_mm_rsqrt_ps(_mm_castsi128_ps(v)))));
      const std::uint32_t hc = std::uint32_t(_mm_cvtsi128_si32(_mm_castps_si128(_mm_rcp_ps(_mm_castsi128_ps(v)))));
      a += hs != ms::intel_rsqrt1(w);
      b += hc != ms::intel_rcp1(w);
    }
  drs = a;
  drc = b;
}

// glibc 2.41 libmvec's results on Intel over the input sets (`test_libmvec_trig --hash` on an Intel CPU)
constexpr std::uint64_t kHash[6] = {0x630c6fc75765175full, 0x4b29829bc33ed224ull, 0x59fd5c8f53fc00f9ull,
                                    0xf2ddb04ce11cfabbull, 0x626ca38e5d10fb73ull, 0xc18562bfc674d4bbull};
const char* const kName[6] = {"acos", "asin", "tan", "acosf", "asinf", "tanf"};

int main(int argc, char** argv) {
  const bool intel = is_intel();
  const bool strict = std::strcmp(gnu_get_libc_version(), "2.41") == 0;
  const bool all = argc > 1 && std::strcmp(argv[1], "all") == 0;
  const bool hash = argc > 1 && std::strcmp(argv[1], "--hash") == 0;
  if (!strict)
    std::printf("glibc %s, not 2.41: the scalar fallback lanes and the hashes are another glibc's; reported, "
                "not failed\n", gnu_get_libc_version());
  long fails = 0;
  long drs = 0, drc = 0;
  emu_vs_cpu(all ? 1 : 13, drs, drc);
  std::printf("%s CPU: rsqrtps differs from Intel's for %ld, rcpps for %ld of %s float bit patterns\n",
              intel ? "Intel" : "non-Intel", drs, drc, all ? "all 2^32" : "every 13th of the 2^32");
  if (intel && (drs || drc)) {
    std::printf("FAIL: the rsqrtps / rcpps emulation differs from this Intel CPU\n");
    ++fails;
  }
  if (all) {
    const long d[3] = {
        run_f_all([](__m128i v) { return ms::acosf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_acosf(v); }),
        run_f_all([](__m128i v) { return ms::asinf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_asinf(v); }),
        run_f_all([](__m128i v) { return ms::tanf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_tanf(v); })};
    for (int k = 0; k < 3; ++k) {
      std::printf("%s: all 2^32 inputs, this CPU's libmvec differs from the port for %ld\n", kName[3 + k], d[k]);
      if (intel && strict && d[k]) ++fails;
    }
  }
  const std::vector<double> xd = dinputs();
  const std::vector<std::uint32_t> xf = finputs(61);
  Res r[6] = {
      run_d("acos", xd, [](__m128i v) { return ms::acos2_sse4(v); }, [](__m128d v) { return _ZGVbN2v_acos(v); }),
      run_d("asin", xd, [](__m128i v) { return ms::asin2_sse4(v); }, [](__m128d v) { return _ZGVbN2v_asin(v); }),
      run_d("tan", xd, [](__m128i v) { return ms::tan2_sse4(v); }, [](__m128d v) { return _ZGVbN2v_tan(v); }),
      run_f("acosf", xf, [](__m128i v) { return ms::acosf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_acosf(v); }),
      run_f("asinf", xf, [](__m128i v) { return ms::asinf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_asinf(v); }),
      run_f("tanf", xf, [](__m128i v) { return ms::tanf4_sse4(v); }, [](__m128 v) { return _ZGVbN4v_tanf(v); })};
  if (hash) {
    if (!intel || !strict) {
      std::printf("--hash: needs an Intel CPU with glibc 2.41\n");
      return 1;
    }
    std::printf("constexpr std::uint64_t kHash[6] = {");
    for (int k = 0; k < 6; ++k) std::printf("%s0x%016llxull", k ? ", " : "", (unsigned long long)r[k].hash_mvec);
    std::printf("};\n");
  }
  for (int k = 0; k < 6; ++k) {
    std::printf("%s: %zu inputs, hash %016llx (Intel %016llx), this CPU's libmvec (hash %016llx) differs for %ld\n",
                kName[k], k < 3 ? xd.size() : xf.size(), (unsigned long long)r[k].hash_port,
                (unsigned long long)kHash[k], (unsigned long long)r[k].hash_mvec, r[k].differ);
    if (strict && r[k].hash_port != kHash[k]) {
      std::printf("FAIL: %s differs from Intel's libmvec\n", kName[k]);
      ++fails;
    }
    if (intel && strict && r[k].differ) {
      std::printf("FAIL: %s differs from this Intel CPU's libmvec\n", kName[k]);
      ++fails;
    }
  }
  std::printf("%s\n", fails ? "test_libmvec_trig FAILED" : "test_libmvec_trig passed");
  return fails ? 1 : 0;
}
#else
int main() {
  std::printf("test_libmvec_trig: no libmvec in this build, skipped\n");
  return 0;
}
#endif
