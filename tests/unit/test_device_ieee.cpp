// SPDX-License-Identifier: GPL-3.0-or-later
// The device's IEEE arithmetic against the host's, bit for bit, in double and float: + - * /, square root, fma,
// the conversions between double, float and int, floor / ceil / trunc / round / rint, fmod, ldexp, fabs / copysign,
// fmax_x86 / fmin_x86 and lround_x86 (core/backend.hpp), a * b + c written out (no implicit contraction) and
// a + 0 (signed zeros kept), on edge values (zeros of both signs, subnormals, the smallest and largest normals,
// infinities, quiet and signaling NaNs), on operands whose results are subnormal and on pseudo-random bit patterns.
// What the model's kernels assume of a GPU (no FMA contraction, no flush to zero, correctly rounded division and
// square root, signed zeros): first of all for the AMD HIP build, whose device arithmetic is not proven anywhere
// else; the CUDA builds pass it too. Two NaNs count as equal (their payloads may differ; exact.py compares NaN with
// NaN alike).
// Explicit oracles, in every build, on the host (and the device): fmax_x86 / fmin_x86 are x86's maxsd / minsd as
// glibc 2.41's fmax / fmin use them (x when x > y, fmin x < y, else y; a quiet NaN gives the other operand, a
// signaling NaN x + y); a * b + c rounds twice (a sentinel triple whose fused value differs) and -0 + 0 is +0.
// Where the builds differ by design (core/backend.hpp): the CPU build's and the CUDA host pass's std::fmax is
// glibc's call with g++'s choice of operand order (it may swap two variables), so their host results are not
// checked on two zeros of opposite sign; the CUDA device's fmax / fmin are PTX max / min (+0 for fmax and -0 for
// fmin of opposite zeros, the number for a signaling NaN and a number), checked as such. HIP builds: the oracle
// everywhere. CPU builds: the host alone (no device to compare).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "core/backend.hpp"

namespace {
constexpr int kOpsD = 24, kOpsF = 24;
[[maybe_unused]] const char* kNamesD[kOpsD] = {
    "d +",    "d -",    "d *",    "d /",     "d sqrt",  "d fma",  "d->f",   "d floor", "d ceil",   "d trunc", "d round",
    "d rint", "d fmod", "d ldexp", "d fabs", "d copysign", "d fmax", "d fmin", "d fmax2", "d fmin2", "d lround",
    "d->int", "d a*b+c", "d a+0"};
[[maybe_unused]] const char* kNamesF[kOpsF] = {
    "f +",    "f -",    "f *",    "f /",     "f sqrt",  "f fma",  "f->d",   "f floor", "f ceil",   "f trunc", "f round",
    "f rint", "f fmod", "f ldexp", "f fabs", "f copysign", "f fmax", "f fmin", "f fmax2", "f fmin2", "f lround",
    "f->int", "f x*y+z", "f x+0"};

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
  r[22] = a * b + c;  // two roundings (-ffp-contract=off, --fmad=false)
  r[23] = a + 0.0;    // -0 + 0 = +0 (no -fno-signed-zeros)
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
  q[22] = x * y + z;
  q[23] = x + 0.0f;
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
// x86's maxsd / minsd with x as the destination, glibc 2.41's fmax / fmin around them (s_fmax.S, s_fmin.S)
bool snan_d(double v) {
  std::uint64_t u;
  std::memcpy(&u, &v, 8);
  return std::isnan(v) && !((u >> 51) & 1u);
}
bool snan_f(float v) {
  std::uint32_t u;
  std::memcpy(&u, &v, 4);
  return std::isnan(v) && !((u >> 22) & 1u);
}
template <class T>
T x86_minmax(T x, T y, bool max, bool (*snan)(T)) {
  const bool xn = std::isnan(x), yn = std::isnan(y);
  if (!xn && !yn) return (max ? x > y : x < y) ? x : y;
  if (yn && !xn) return snan(y) ? T(NAN) : x;
  if (xn && !yn) return snan(x) ? T(NAN) : y;
  return T(NAN);
}
template <class T>
bool opposite_zeros(T x, T y) {
  return x == T(0) && y == T(0) && std::signbit(x) != std::signbit(y);
}
// the fmax / fmin operations (16..19): their operands and which is a max
template <class T>
void minmax_operands(int op, T a, T b, T& x, T& y, bool& max) {
  max = op == 16 || op == 18;
  x = op < 18 ? a : b;
  y = op < 18 ? b : a;
}
// the oracle's value of operation op on input (a, b, c), when it has one (fmax / fmin, the sentinels)
template <class T>
bool oracle(int op, T a, T b, bool (*snan)(T), T& want) {
  if (op >= 16 && op <= 19) {
    T x, y;
    bool max;
    minmax_operands(op, a, b, x, y, max);
    want = x86_minmax(x, y, max, snan);
    return true;
  }
  if (op == 23 && a == T(0)) {  // -0 + 0 and +0 + 0: +0
    want = T(0);
    return true;
  }
  return false;
}
}  // namespace

// the build's rule for operation op on input (a, b, c) of type T: what the host's result must be (oracle) and
// whether the device's must equal the host's or its own value (CUDA's PTX max / min)
enum class Rule { kSame, kOracle, kSkipHost };
template <class T>
int check(const char* name, int op, int nops, int n, const T* in, const std::vector<T>& h,
          [[maybe_unused]] const std::vector<T>* g, bool (*snan)(T), bool (*same)(T, T), int nfix0) {
  int bad = 0, skipped = 0;
  for (int t = 0; t < n; ++t) {
    const std::size_t k = std::size_t(t) * std::size_t(nops) + std::size_t(op);
    const T a = in[3 * std::size_t(t)], b = in[3 * std::size_t(t) + 1], c = in[3 * std::size_t(t) + 2];
    const bool mm = op >= 16 && op <= 19;
    T want{};
    const bool has = oracle(op, a, b, snan, want) || (op == 22 && t == nfix0 && ((want = T(0)), true));
    // the host's result
    bool host_checked = has;
#if !defined(CPPWRF_USE_HIP)
    if (mm && opposite_zeros(a, b)) host_checked = false, ++skipped;  // g++'s operand order (glibc's call)
#endif
    if (host_checked && !same(h[k], want)) {
      if (bad++ < 3)
        std::printf("%-10s a=%a b=%a c=%a: host %a, oracle %a\n", name, double(a), double(b), double(c),
                    double(h[k]), double(want));
    }
#if defined(CPPWRF_USE_GPU)
    // the device's result: the host's (HIP: also the oracle's), CUDA's fmax / fmin their PTX value
    T dev_want = has ? want : h[k];
#if defined(CPPWRF_USE_CUDA)
    if (mm) {
      T x, y;
      bool max;
      minmax_operands(op, a, b, x, y, max);
      if (opposite_zeros(x, y)) dev_want = max ? T(0) : -T(0);
      else if (std::isnan(x) != std::isnan(y)) dev_want = std::isnan(x) ? y : x;
    }
#endif
    if (!same((*g)[k], dev_want)) {
      if (bad++ < 3)
        std::printf("%-10s a=%a b=%a c=%a: host %a device %a (want %a)\n", name, double(a), double(b), double(c),
                    double(h[k]), double((*g)[k]), double(dev_want));
    }
#endif
  }
  if (bad) std::printf("%-10s %d of %d differ\n", name, bad, n);
  if (skipped) std::printf("%-10s %d host results of opposite zeros not checked (g++'s operand order)\n", name, skipped);
  return bad;
}

int main() {
  // edge values: every ordered pair of them (c from the list as well), then the fixed triples (nfix: the contraction
  // sentinel, whose a * b + c is +0 with two roundings and -2^-104 (float: -2^-46) fused), then products with
  // subnormal results, then random bit patterns (a fourth of them with small exponents: subnormal sums and
  // differences)
  const double ed[] = {0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 1.5, -2.5, 2.5, 3.0, 1e300, -1e300, 4.9e-324, -4.9e-324,
                       2.2250738585072014e-308, -2.2250738585072014e-308, 2.225073858507201e-308,
                       1.7976931348623157e308, INFINITY, -INFINITY, NAN, 9.3e18, -9.3e18, 2147483647.5,
                       -2147483648.5, 0.49999999999999994, as_d(0x7ff0000000000001ull), as_d(0xfff4000000000000ull)};
  const float ef[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, -0.5f, 1.5f, -2.5f, 2.5f, 3.0f, 1e38f, -1e38f, 1.4e-45f,
                      -1.4e-45f, 1.17549435e-38f, -1.17549435e-38f, 1.1754942e-38f, 3.4028235e38f, INFINITY,
                      -INFINITY, NAN, 9.3e18f, -9.3e18f, 16777217.0f, -8388609.5f, 0.49999997f, as_f(0x7f800001u),
                      as_f(0xffa00000u)};
  constexpr int ne = int(sizeof ed / sizeof ed[0]);
  static_assert(sizeof ef / sizeof ef[0] == sizeof ed / sizeof ed[0]);
  const int nedge = ne * ne, nfix = 1, nsub = 4096, nrand = 1 << 17;
  const int n = nedge + nfix + nsub + nrand;
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
    } else if (t < nedge + nfix) {  // (1 + 2^-52)(1 - 2^-52) - 1, (1 + 2^-23)(1 - 2^-23) - 1
      a[0] = 0x1.0000000000001p0, a[1] = 0x1.ffffffffffffep-1, a[2] = -1.0;
      x[0] = 0x1.000002p0f, x[1] = 0x1.fffffcp-1f, x[2] = -1.0f;
    } else if (t < nedge + nfix + nsub) {  // |a b| near 2^-1040 and |x y| near 2^-140: subnormal products, quotients
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
  const std::vector<double>* gdp = nullptr;
  const std::vector<float>* gfp = nullptr;
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
  gdp = &gd;
  gfp = &gf;
#endif
  for (int op = 0; op < kOpsD; ++op) fails += check(kNamesD[op], op, kOpsD, n, in.data(), hd, gdp, snan_d, same_d, nedge);
  for (int op = 0; op < kOpsF; ++op) fails += check(kNamesF[op], op, kOpsF, n, inf.data(), hf, gfp, snan_f, same_f, nedge);
#if defined(CPPWRF_USE_GPU)
  std::printf("%s: device against host and the oracles, %d inputs, %d + %d operations\n", cppwrf::backend_name(), n,
              kOpsD, kOpsF);
#else
  std::printf("CPU build: %d inputs on the host against the oracles (no device to compare)\n", n);
#endif
  std::printf(fails ? "FAIL (%d)\n" : "PASS\n", fails);
  return fails ? 1 : 0;
}
