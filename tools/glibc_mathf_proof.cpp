// SPDX-License-Identifier: GPL-3.0-or-later
// Proof of src/core/glibc_mathf.hpp on the CPU: every float input of expf, logf, sinf, cosf, atanf, tanf, tanhf and
// log10f, and powf and atan2f on 2^32 pseudo-random pairs plus grids of special values, against the system libm
// (glibc 2.41, FMA variants), bit for bit (two NaNs count as equal).
//   g++ -std=c++20 -O2 -ffp-contract=off -fopenmp -Isrc tools/glibc_mathf_proof.cpp -o glibc_mathf_proof
//   ./glibc_mathf_proof [log2 of the number of powf / atan2f pairs, default 32]
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "core/glibc_mathf.hpp"

namespace {
float f(std::uint32_t b) {
  float v;
  std::memcpy(&v, &b, 4);
  return v;
}
std::uint32_t u(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, 4);
  return b;
}
bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || u(a) == u(b); }
// system libm through volatile function pointers (no folding, no builtins)
float (*volatile sys_expf)(float) = ::expf;
float (*volatile sys_logf)(float) = ::logf;
float (*volatile sys_sinf)(float) = ::sinf;
float (*volatile sys_cosf)(float) = ::cosf;
float (*volatile sys_atanf)(float) = ::atanf;
float (*volatile sys_tanf)(float) = ::tanf;
float (*volatile sys_atan2f)(float, float) = ::atan2f;
float (*volatile sys_tanhf)(float) = ::tanhf;
float (*volatile sys_log10f)(float) = ::log10f;
float (*volatile sys_powf)(float, float) = ::powf;

template <class P, class S>
long long unary(const char* name, P port, S sys) {
  long long bad = 0;
  std::uint32_t first = 0;
  bool have = false;
#pragma omp parallel for schedule(static, 1 << 16) reduction(+ : bad)
  for (long long i = 0; i < (1ll << 32); ++i) {
    const float x = f(std::uint32_t(i));
    if (!same(port(x), sys(x))) {
      ++bad;
#pragma omp critical
      if (!have) { have = true; first = std::uint32_t(i); }
    }
  }
  std::printf("%-7s all 2^32 inputs: %lld differ", name, bad);
  if (bad) std::printf(" (first 0x%08x: %a vs %a)", first, double(port(f(first))), double(sys(f(first))));
  std::printf("\n");
  return bad;
}
std::uint64_t mix(std::uint64_t z) {
  z += 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}
}  // namespace

int main(int argc, char** argv) {
  namespace g = cppwrf::glibcf;
  long long bad = 0;
  bad += unary("expf", [](float x) { return g::expf(x); }, [](float x) { return sys_expf(x); });
  bad += unary("logf", [](float x) { return g::logf(x); }, [](float x) { return sys_logf(x); });
  bad += unary("sinf", [](float x) { return g::sinf(x); }, [](float x) { return sys_sinf(x); });
  bad += unary("cosf", [](float x) { return g::cosf(x); }, [](float x) { return sys_cosf(x); });
  bad += unary("atanf", [](float x) { return g::atanf(x); }, [](float x) { return sys_atanf(x); });
  bad += unary("tanf", [](float x) { return g::tanf(x); }, [](float x) { return sys_tanf(x); });
  bad += unary("tanhf", [](float x) { return g::tanhf(x); }, [](float x) { return sys_tanhf(x); });
  bad += unary("log10f", [](float x) { return g::log10f(x); }, [](float x) { return sys_log10f(x); });
  const int lg = argc > 1 ? std::atoi(argv[1]) : 32;
  long long pbad = 0;
#pragma omp parallel for schedule(static, 1 << 16) reduction(+ : pbad)
  for (long long i = 0; i < (1ll << lg); ++i) {
    const std::uint64_t r = mix(std::uint64_t(i));
    float x = f(std::uint32_t(r)), y = f(std::uint32_t(r >> 32));
    // half the pairs with x in (0, 8) and |y| < 64: the arguments of a model
    if (i & 1) {
      x = f(0x3f800000u + std::uint32_t(r % 0x1800000u) - 0x800000u);
      y = f((std::uint32_t(r >> 32) & 0x80000000u) | (0x3c000000u + std::uint32_t((r >> 32) % 0x6000000u)));
    }
    if (!same(g::powf(x, y), sys_powf(x, y))) ++pbad;
  }
  // special values: 0, subnormals, 1, integers, Inf, NaN, negative bases
  const float sp[] = {0.0f, -0.0f, 1e-45f, -1e-45f, 1e-40f, 1.0f, -1.0f, 2.0f, -2.0f, 0.5f, 3.0f, -3.0f, 1e30f,
                      -1e30f, INFINITY, -INFINITY, NAN, 1.0000001f, 0.99999994f, 100.0f, 0.1f, -0.1f, 126.0f, -150.0f};
  for (float x : sp)
    for (float y : sp)
      if (!same(g::powf(x, y), sys_powf(x, y))) {
        ++pbad;
        std::printf("powf(%a, %a): %a vs %a\n", double(x), double(y), double(g::powf(x, y)), double(sys_powf(x, y)));
      }
  std::printf("powf    2^%d pairs + special values: %lld differ\n", lg, pbad);
  bad += pbad;
  // atan2f: 2^lg pseudo-random pairs (half of them with exponents within 2^-30 .. 2^30 of each other) and the special
  // values' pairs
  long long abad = 0;
#pragma omp parallel for schedule(static, 1 << 16) reduction(+ : abad)
  for (long long i = 0; i < (1ll << lg); ++i) {
    const std::uint64_t r = mix(std::uint64_t(i) ^ 0x9e3779b97f4a7c15ull);
    std::uint32_t a = std::uint32_t(r), b = std::uint32_t(r >> 32);
    if (i & 1) b = (b & 0x807fffffu) | ((((a >> 23) & 0xffu) + ((b >> 23) % 61u) - 30u) & 0xffu) << 23;
    const float y = f(a), x = f(b);
    if (!same(g::atan2f(y, x), sys_atan2f(y, x))) ++abad;
  }
  for (float x : sp)
    for (float y : sp)
      if (!same(g::atan2f(y, x), sys_atan2f(y, x))) {
        ++abad;
        std::printf("atan2f(%a, %a): %a vs %a\n", double(y), double(x), double(g::atan2f(y, x)), double(sys_atan2f(y, x)));
      }
  std::printf("atan2f  2^%d pairs + special values: %lld differ\n", lg, abad);
  bad += abad;
  std::printf(bad ? "glibc_mathf_proof FAILED\n" : "glibc_mathf_proof passed\n");
  return bad ? 1 : 0;
}
