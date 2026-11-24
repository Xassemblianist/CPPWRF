// SPDX-License-Identifier: GPL-3.0-or-later
// The device's IEEE arithmetic against the host's, bit for bit, in double and float: + - * /, square root, fma,
// the conversions between double, float and int, floor / ceil / trunc / round / rint, fmod, ldexp, fabs / copysign,
// fmax_x86 / fmin_x86 and lround_x86 (core/backend.hpp), on edge values (zeros of both signs, subnormals, the smallest
// and largest normals, infinities, NaN), on operands whose results are subnormal and on pseudo-random bit patterns.
// What the model's kernels assume of a GPU (no FMA contraction, no flush to zero, correctly rounded division and
// square root): first of all for the AMD HIP build, whose device arithmetic is not proven anywhere else; the CUDA
// builds pass it too. Two NaNs count as equal (their payloads may differ; exact.py compares NaN with NaN alike).
// CPU builds: the host alone (nothing to compare).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/backend.hpp"

namespace {
constexpr int kOpsD = 22, kOpsF = 22;
[[maybe_unused]] const char* kNamesD[kOpsD] = {
    "d +",    "d -",    "d *",    "d /",     "d sqrt",  "d fma",  "d->f",   "d floor", "d ceil",   "d trunc", "d round",
    "d rint", "d fmod", "d ldexp", "d fabs", "d copysign", "d fmax", "d fmin", "d fmax2", "d fmin2", "d lround",
    "d->int"};
[[maybe_unused]] const char* kNamesF[kOpsF] = {
    "f +",    "f -",    "f *",    "f /",     "f sqrt",  "f fma",  "f->d",   "f floor", "f ceil",   "f trunc", "f round",
    "f rint", "f fmod", "f ldexp", "f fabs", "f copysign", "f fmax", "f fmin", "f fmax2", "f fmin2", "f lround",
    "f->int"};

CPPWRF_HD inline double as_d(std::uint64_t b) {
  double d;
  std::memcpy(&d, &b, 8);
  return d;
}
CPPWRF_HD inline float as_f(std::uint32_t b) {
  float f;
  std::memcpy(&f, &b, 4);
  return f;
}
// an int conversion of a value in int's range (C++ leaves the others undefined; WRF's code converts in range)
CPPWRF_HD inline double to_int_d(double x) { return std::fabs(x) < 2.0e9 ? double(int(x)) : 0.0; }
CPPWRF_HD inline float to_int_f(float x) { return std::fabs(x) < 2.0e9f ? float(int(x)) : 0.0f; }

// one input: a, b, c in double and float; every result of the operations above
CPPWRF_HD inline void eval(const double* in, const float* inf, int i, double* od, float* of) {
  const double a = in[3 * i], b = in[3 * i + 1], c = in[3 * i + 2];
  double* r = od + std::size_t(i) * kOpsD;
  r[0] = a + b;
  r[1] = a - b;
  r[2] = a * b;
  r[3] = a / b;
  r[4] = std::sqrt(a);
  r[5] = std::fma(a, b, c);
  r[6] = double(float(a));
  r[7] = std::floor(a);
  r[8] = std::ceil(a);
  r[9] = std::trunc(a);
  r[10] = std::round(a);
  r[11] = std::rint(a);
  r[12] = std::fmod(a, b);
  r[13] = std::ldexp(a, i % 64 - 32);
  r[14] = std::fabs(a);
  r[15] = std::copysign(a, b);
  r[16] = fmax_x86(a, b);
  r[17] = fmin_x86(a, b);
  r[18] = fmax_x86(b, a);
  r[19] = fmin_x86(b, a);
  r[20] = double(lround_x86(a));
  r[21] = to_int_d(a);
  const float x = inf[3 * i], y = inf[3 * i + 1], z = inf[3 * i + 2];
  float* q = of + std::size_t(i) * kOpsF;
  q[0] = x + y;
  q[1] = x - y;
  q[2] = x * y;
  q[3] = x / y;
  q[4] = std::sqrt(x);
  q[5] = std::fma(x, y, z);
  q[6] = float(double(x) * 0.5);  // float -> double, an exact operation, back to float
  q[7] = std::floor(x);
  q[8] = std::ceil(x);
  q[9] = std::trunc(x);
  q[10] = std::round(x);
  q[11] = std::rint(x);
  q[12] = std::fmod(x, y);
  q[13] = std::ldexp(x, i % 64 - 32);
  q[14] = std::fabs(x);
  q[15] = std::copysign(x, y);
  q[16] = fmax_x86(x, y);
  q[17] = fmin_x86(x, y);
  q[18] = fmax_x86(y, x);
  q[19] = fmin_x86(y, x);
  q[20] = float(int(lround_x86(x) & 0xffffff));  // (the whole long: test_lround_x86)
  q[21] = to_int_f(x);
}

struct K {
  const double* in;
  const float* inf;
  double* od;
  float* of;
  CPPWRF_HD void operator()(int i, int) const { eval(in, inf, i, od, of); }
};

std::uint64_t mix(std::uint64_t z) {
  z += 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}
bool same_d(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) return true;
  std::uint64_t x, y;
  std::memcpy(&x, &a, 8);
  std::memcpy(&y, &b, 8);
  return x == y;
}
bool same_f(float a, float b) {
  if (std::isnan(a) && std::isnan(b)) return true;
  std::uint32_t x, y;
  std::memcpy(&x, &a, 4);
  std::memcpy(&y, &b, 4);
  return x == y;
}
}  // namespace

int main() {
  // edge values: every ordered pair of them (c from the list as well), then products with subnormal results, then
  // random bit patterns (a fourth of them with small exponents: subnormal sums and differences)
  const double ed[] = {0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 1.5, -2.5, 2.5, 3.0, 1e300, -1e300, 4.9e-324, -4.9e-324,
                       2.2250738585072014e-308, -2.2250738585072014e-308, 2.225073858507201e-308,
                       1.7976931348623157e308, INFINITY, -INFINITY, NAN, 9.3e18, -9.3e18, 2147483647.5,
                       -2147483648.5, 0.49999999999999994};
  const float ef[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, -0.5f, 1.5f, -2.5f, 2.5f, 3.0f, 1e38f, -1e38f, 1.4e-45f,
                      -1.4e-45f, 1.17549435e-38f, -1.17549435e-38f, 1.1754942e-38f, 3.4028235e38f, INFINITY,
                      -INFINITY, NAN, 9.3e18f, -9.3e18f, 16777217.0f, -8388609.5f, 0.49999997f};
  constexpr int ne = int(sizeof ed / sizeof ed[0]);
  static_assert(sizeof ef / sizeof ef[0] == sizeof ed / sizeof ed[0]);
  const int nedge = ne * ne, nsub = 4096, nrand = 1 << 17;
  const int n = nedge + nsub + nrand;
  std::vector<double> in(3 * std::size_t(n));
  std::vector<float> inf(3 * std::size_t(n));
  std::uint64_t s = 0x1eee0000ull;
  for (int t = 0; t < n; ++t) {
    double* a = &in[3 * std::size_t(t)];
    float* x = &inf[3 * std::size_t(t)];
    const std::uint64_t r0 = mix(s++), r1 = mix(s++), r2 = mix(s++);
    if (t < nedge) {
      a[0] = ed[t / ne], a[1] = ed[t % ne], a[2] = ed[(t / ne + t) % ne];
      x[0] = ef[t / ne], x[1] = ef[t % ne], x[2] = ef[(t / ne + t) % ne];
    } else if (t < nedge + nsub) {  // |a b| near 2^-1040 and |x y| near 2^-140: subnormal products and quotients
      a[0] = std::ldexp(1.0 + double(r0 >> 12) * 0x1p-52, -520 - int(r2 % 16)) * ((r0 & 1) ? -1 : 1);
      a[1] = std::ldexp(1.0 + double(r1 >> 12) * 0x1p-52, -520 - int((r2 >> 8) % 16));
      a[2] = std::ldexp(1.0 + double(r2 >> 12) * 0x1p-52, -1060);
      x[0] = std::ldexp(1.0f + float(r0 >> 41) * 0x1p-23f, -70 - int(r2 % 8)) * ((r0 & 2) ? -1.0f : 1.0f);
      x[1] = std::ldexp(1.0f + float(r1 >> 41) * 0x1p-23f, -70 - int((r2 >> 8) % 8));
      x[2] = std::ldexp(1.0f + float(r2 >> 41) * 0x1p-23f, -145);
    } else {
      const bool small = (r2 & 3) == 0;
      const std::uint64_t md = small ? 0x801fffffffffffffull : ~0ull;  // small: exponent bits 0 or 1
      const std::uint32_t mf = small ? 0x80ffffffu : ~0u;
      a[0] = as_d(r0 & md), a[1] = as_d(r1 & md), a[2] = as_d(r2 & md);
      x[0] = as_f(std::uint32_t(r0 >> 32) & mf), x[1] = as_f(std::uint32_t(r1 >> 32) & mf);
      x[2] = as_f(std::uint32_t(r2 >> 32) & mf);
    }
  }
  std::vector<double> hd(std::size_t(n) * kOpsD);
  std::vector<float> hf(std::size_t(n) * kOpsF);
  for (int t = 0; t < n; ++t) eval(in.data(), inf.data(), t, hd.data(), hf.data());
  int fails = 0;
#if defined(CPPWRF_USE_GPU)
  const std::size_t bi = in.size() * sizeof(double), bf = inf.size() * sizeof(float);
  const std::size_t bod = hd.size() * sizeof(double), bof = hf.size() * sizeof(float);
  double* din = static_cast<double*>(cppwrf::dev_alloc(bi));
  float* dinf = static_cast<float*>(cppwrf::dev_alloc(bf));
  double* dod = static_cast<double*>(cppwrf::dev_alloc(bod));
  float* dof = static_cast<float*>(cppwrf::dev_alloc(bof));
  cppwrf::copy_to_dev(din, in.data(), bi);
  cppwrf::copy_to_dev(dinf, inf.data(), bf);
  cppwrf::column_for(cppwrf::Range2D{0, n, 0, 1}, K{din, dinf, dod, dof});
  std::vector<double> gd(hd.size());
  std::vector<float> gf(hf.size());
  cppwrf::copy_to_host(gd.data(), dod, bod);
  cppwrf::copy_to_host(gf.data(), dof, bof);
  cppwrf::dev_free(din);
  cppwrf::dev_free(dinf);
  cppwrf::dev_free(dod);
  cppwrf::dev_free(dof);
  for (int op = 0; op < kOpsD; ++op) {
    int bad = 0;
    for (int t = 0; t < n; ++t) {
      const std::size_t k = std::size_t(t) * kOpsD + std::size_t(op);
      if (same_d(hd[k], gd[k])) continue;
      if (bad++ < 3)
        std::printf("%-10s a=%a b=%a c=%a: host %a device %a\n", kNamesD[op], in[3 * std::size_t(t)],
                    in[3 * std::size_t(t) + 1], in[3 * std::size_t(t) + 2], hd[k], gd[k]);
    }
    if (bad) std::printf("%-10s %d of %d differ\n", kNamesD[op], bad, n);
    fails += bad;
  }
  for (int op = 0; op < kOpsF; ++op) {
    int bad = 0;
    for (int t = 0; t < n; ++t) {
      const std::size_t k = std::size_t(t) * kOpsF + std::size_t(op);
      if (same_f(hf[k], gf[k])) continue;
      if (bad++ < 3)
        std::printf("%-10s x=%a y=%a z=%a: host %a device %a\n", kNamesF[op], double(inf[3 * std::size_t(t)]),
                    double(inf[3 * std::size_t(t) + 1]), double(inf[3 * std::size_t(t) + 2]), double(hf[k]),
                    double(gf[k]));
    }
    if (bad) std::printf("%-10s %d of %d differ\n", kNamesF[op], bad, n);
    fails += bad;
  }
  std::printf("%s: device against host, %d inputs, %d + %d operations\n", cppwrf::backend_name(), n, kOpsD, kOpsF);
#else
  (void)same_d;
  (void)same_f;
  std::printf("CPU build: %d inputs evaluated on the host (no device to compare)\n", n);
#endif
  std::printf(fails ? "FAIL (%d)\n" : "PASS\n", fails);
  return fails ? 1 : 0;
}
