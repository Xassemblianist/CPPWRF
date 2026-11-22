// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// glibc 2.41 libmvec's 4-lane float expf, logf, powf, sinf, cosf and atanf (_ZGVbN4v_expf, _ZGVbN4v_logf,
// _ZGVbN4vv_powf, _ZGVbN4v_sinf, _ZGVbN4v_cosf, _ZGVbN4v_atanf; the SSE4
// variants that glibc selects on every CPU with SSE4.1) re-implemented statement by statement, one lane, from
// sysdeps/x86_64/fpu/multiarch/svml_s_{exp,log,pow,sin,cos,atan}f4_core_sse4.S.
// WRF's real*4 build (gfortran -O2 -ftree-vectorize, SSE2 ABI: 4 floats per call) calls them in its vectorised loops.
// expf, logf, sinf, cosf and atanf are IEEE single-precision operations only; powf works in double and takes an approximate reciprocal
// with `rcpps`, whose result the hardware defines: here Intel's (mvec::detail::kRcp, the table the double log uses;
// WRF's references are made on Intel), so the results are Intel libmvec's on any x86-64 CPU. Every other operation
// is an IEEE operation in the assembly's order (the build has -ffp-contract=off: no FMA contraction). Lanes the
// assembly sends to the scalar libm (its range masks) call the scalar function here too.
// CPU FP32 builds on x86-64, and the CUDA FP32 identity build (CPPWRF_FP32_IDENTITY) on the device and the host: one
// lane per call (a thread computes its lane as the SSE4 code does), kPowfT and kLogLA read from device copies, the
// masked lanes through glibc's scalar functions (core/glibc_mathf.hpp on the device).
#include <cmath>
#include <cstdint>
#include <cstring>

#include "core/gpu_runtime.hpp"
#include "core/libmvec_f4_tables.hpp"
#include "core/libmvec_x86.hpp"
#if defined(CPPWRF_FP32_IDENTITY)
#include "core/glibc_mathf.hpp"
#endif

#if defined(CPPWRF_GPUCC)
namespace cppwrf::mvec::detail {
static __device__ const DevTable<std::uint64_t, 2048> kPowfTDev = dev_table(::cppwrf::mvec_tables::kPowfT);
}  // namespace cppwrf::mvec::detail
#endif

namespace cppwrf::mvec4 {
namespace detail {
CPPWRF_MV float f(std::uint32_t b) {
#if defined(CPPWRF_DEVICE_CODE)
  return __uint_as_float(b);
#else
  float v;
  std::memcpy(&v, &b, sizeof v);
  return v;
#endif
}
CPPWRF_MV std::uint32_t fb(float v) {
#if defined(CPPWRF_DEVICE_CODE)
  return __float_as_uint(v);
#else
  std::uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
#endif
}
CPPWRF_MV std::int32_t s32(std::uint32_t b) { return static_cast<std::int32_t>(b); }
// the scalar functions of the masked lanes: glibc's (the system libm on the host, its port on the device)
#if defined(CPPWRF_DEVICE_CODE)
CPPWRF_MV float sc_exp(float x) { return ::cppwrf::glibcf::expf(x); }
CPPWRF_MV float sc_log(float x) { return ::cppwrf::glibcf::logf(x); }
CPPWRF_MV float sc_pow(float x, float y) { return ::cppwrf::glibcf::powf(x, y); }
CPPWRF_MV float sc_sin(float x) { return ::cppwrf::glibcf::sinf(x); }
CPPWRF_MV float sc_cos(float x) { return ::cppwrf::glibcf::cosf(x); }
#else
inline float sc_exp(float x) { return std::exp(x); }
inline float sc_log(float x) { return std::log(x); }
inline float sc_pow(float x, float y) { return std::pow(x, y); }
inline float sc_sin(float x) { return std::sin(x); }
inline float sc_cos(float x) { return std::cos(x); }
#endif
}  // namespace detail

// _ZGVbN4v_expf_sse4: exp(x) = 2^N * exp(r), N = rint(x/ln2), r = x - N*ln2 (hi + lo), degree-5 polynomial
CPPWRF_MV float expf(float x) {
  using namespace mvec_tables;
  using detail::f;
  const std::uint32_t ix = detail::fb(x);
  if (detail::s32(ix & kExpf_iAbsMask) > detail::s32(kExpf_iDomainRange)) return detail::sc_exp(x);
  const float m = x * f(kExpf_sInvLn2) + f(kExpf_sShifter);  // mulps; addps
  const float n = m - f(kExpf_sShifter);
  float r = x - n * f(kExpf_sLn2hi);
  r = r - n * f(kExpf_sLn2lo);
  const float scale = f((detail::fb(m) + kExpf_iBias) << 23);  // paddd; pslld $23
  float p = f(kExpf_sPC5) * r + f(kExpf_sPC4);
  p = p * r + f(kExpf_sPC3);
  p = p * r + f(kExpf_sPC2);
  p = p * r + f(kExpf_sPC1);
  const float e = r * p + f(kExpf_sPC0);
  return scale * e;
}

// _ZGVbN4v_logf_sse4: log(x) = k*ln2 + log(1+R), R = mantissa (in [2/3, 4/3)) - 1, degree-9 polynomial
CPPWRF_MV float logf(float x) {
  using namespace mvec_tables;
  using detail::f;
  const std::uint32_t ix = detail::fb(x);
  // denormals, zero, negatives, Inf, NaN: pcmpgtd (LoRange > x + HiDelta, signed)
  if (detail::s32(kLogf_iLoRange) > detail::s32(ix + kLogf_iHiDelta)) return detail::sc_log(x);
  const std::uint32_t t = ix - kLogf_iBrkValue;
  const float r = f((t & kLogf_iOffExpoMask) + kLogf_iBrkValue) - f(kLogf_sOne);
  const float k = static_cast<float>(detail::s32(t) >> 23) * f(kLogf_sLn2);  // psrad $23; cvtdq2ps; mulps
  float p = f(kLogf_sPoly_7) * r + f(kLogf_sPoly_6);
  p = p * r + f(kLogf_sPoly_5);
  p = p * r + f(kLogf_sPoly_4);
  p = p * r + f(kLogf_sPoly_3);
  p = p * r + f(kLogf_sPoly_2);
  p = p * r + f(kLogf_sPoly_1);
  p = p * r;
  p = p * r;
  return (r + p) + k;
}

// _ZGVbN4vv_powf_sse4: pow(x, y) = exp(y * log(x)) in double: log(x) = k*ln2 - log(Rcp) + log1p(q) with Rcp an
// integer reciprocal of x's mantissa scaled to [2^-10, 2^-9) (rcpps), then 2^(t / 2048) = 2^N * T[j] * (1 + r)
CPPWRF_MV float powf(float x, float y) {
  using namespace mvec_tables;
  using mvec::detail::d;
  using mvec::detail::bits;
  const std::uint32_t ix = detail::fb(x), iy = detail::fb(y);
  const std::uint32_t xm = ix - kPowf_NMINNORM, ay = iy & kPowf_ABSMASK;
  const bool special_x = detail::s32(xm) > detail::s32(kPowf_NMAXVAL) || xm == kPowf_NMAXVAL;  // x <= 0, not normal, >= FLT_MAX
  const bool special_y = detail::s32(ay) > detail::s32(kPowf_INF) || ay == kPowf_INF;          // y Inf or NaN
  const double xd = double(x), yd = double(y);
  const std::uint64_t xb = bits(xd);
  const double m = d((xb & kPowf_ExpMask) | kPowf_Two10);
  const double e = double(std::int32_t(xb >> 52));  // biased exponent (with the sign bit)
  const double rcp = mvec::detail::rcp_rounded(m);
  const double q = m * rcp - d(kPowf_One);
  const double bias = d(kPowf_Threshold) < rcp ? d(kPowf_Bias) : d(kPowf_Bias1);  // (mask & Bias) | Bias1
  const double ke = (e - bias) * d(kPowf_L2);
  const double q2 = q * q;
  double p = d(kPowf_poly_coeff_3) * q + d(kPowf_poly_coeff_4);
  p = p * q2;
  const double lq = q + p;
  const double l = d(CPPWRF_MVT(kLogLA, kLogLADev)[int(rcp) - 512]) + lq;
  const double lx = ke + l;
  const double yl = lx * yd;
  // iRangeMask: |hi word of y*log(x)| > DomainRange
  const bool special_r = detail::s32(std::uint32_t(bits(yl) >> 32) & kPowf_iAbsMask) > detail::s32(kPowf_iDomainRange);
  if (special_x || special_y || special_r) return detail::sc_pow(x, y);
  const double t = d(kPowf_dbInvLn2) * yl;
  const double s = (t - d(kPowf_dbHALF)) + d(kPowf_dbShifter);
  const double n = s - d(kPowf_dbShifter);
  const double r = (t - n) * d(kPowf_dbC1);
  const std::uint64_t sb = bits(s);
  const double tj = d(CPPWRF_MVT(kPowfT, kPowfTDev)[sb & kPowf_lbLOWKBITS]);
  const double res = tj + r * tj;
  return static_cast<float>(d(bits(res) + ((sb >> 11) << 52)));  // paddq of the exponent; cvtpd2ps
}

// _ZGVbN4v_sinf_sse4: x = N*pi + r (pi in 4 parts), sin(x) = (-1)^N sin(r), odd degree-9 polynomial; |x| > 10000
// (and NaN, Inf) through the scalar sinf
CPPWRF_MV float sinf(float x) {
  using namespace mvec_tables;
  using detail::f;
  using detail::fb;
  const std::uint32_t ix = fb(x), sgn = ix & ~kTrigf_sAbsMask;
  const float ax = f(ix & kTrigf_sAbsMask);
  if (!(ax <= f(kTrigf_sRangeReductionVal))) return detail::sc_sin(x);  // cmpnleps
  const float y = ax * f(kTrigf_sInvPI) + f(kTrigf_sRShifter);
  const std::uint32_t s = fb(y) << 31;
  const float n = y - f(kTrigf_sRShifter);
  float r = ax - n * f(kTrigf_sPI1);
  r = r - n * f(kTrigf_sPI2);
  r = r - n * f(kTrigf_sPI3);
  r = r - n * f(kTrigf_sPI4);
  const float r2 = r * r;
  r = f(fb(r) ^ s);
  float p = f(kTrigf_sA9) * r2 + f(kTrigf_sA7);
  p = p * r2 + f(kTrigf_sA5);
  p = p * r2 + f(kTrigf_sA3);
  const float t = (r2 * p) * r;
  return f(fb(r + t) ^ sgn);
}

// _ZGVbN4v_cosf_sse4: cos(x) = sin(x + pi/2) with N = rint((x + pi/2)/pi) - 1/2 and the same polynomial
CPPWRF_MV float cosf(float x) {
  using namespace mvec_tables;
  using detail::f;
  using detail::fb;
  if (!(f(fb(x) & kTrigf_sAbsMask) <= f(kTrigf_sRangeReductionVal))) return detail::sc_cos(x);
  const float y = (f(kTrigf_sHalfPI) + x) * f(kTrigf_sInvPI) + f(kTrigf_sRShifter);
  const std::uint32_t s = fb(y) << 31;
  const float n = (y - f(kTrigf_sRShifter)) - f(kTrigf_sOneHalf);
  float r = x - n * f(kTrigf_sPI1);
  r = r - n * f(kTrigf_sPI2);
  r = r - n * f(kTrigf_sPI3);
  r = r - n * f(kTrigf_sPI4);
  const float r2 = r * r;
  r = f(fb(r) ^ s);
  float p = f(kTrigf_sA9) * r2 + f(kTrigf_sA7);
  p = p * r2 + f(kTrigf_sA5);
  p = p * r2 + f(kTrigf_sA3);
  const float t = (r2 * p) * r;
  return r + t;
}
// _ZGVbN4v_atanf_sse4: s = |x| <= 1 ? x : -1/x (minps / maxps / divps, the sign of x applied), atan(x) = c + s*P(s^2)
// with c = 0 or +-pi/2, two interleaved polynomial halves in s^4 (the assembly's order); no scalar fallback lanes
// (NaN propagates through the polynomial)
CPPWRF_MV float atanf(float x) {
  using detail::f;
  using detail::fb;
  const std::uint32_t ix = fb(x), sgn = ix & 0x80000000u;
  const float ax = f(ix & 0x7fffffffu), one = f(0x3f800000u);
  const bool le = ax <= one;                // cmpleps (false for NaN)
  const float mx = one > ax ? one : ax;     // maxps %xmm2(|x|), %xmm3(1): NaN gives |x|
  const float mn = one < ax ? one : ax;     // minps %xmm2(|x|), %xmm1(1)
  const float q = mn / mx;                  // divps
  const float s = f(fb(q) ^ ((le ? 0u : 0x80000000u) ^ sgn));
  const float x2 = s * s, x4 = x2 * x2;
  float p6 = f(0x3B322CC0u) * x4;          // PC8
  float p5 = f(0xBC7F2631u) * x4;          // PC7
  p6 = p6 + f(0x3D2BC384u);                // PC6
  p6 = p6 * x4;
  p5 = p5 + f(0xBD987629u);                // PC5
  p5 = p5 * x4;
  p6 = p6 + f(0x3DD96474u);                // PC4
  p6 = p6 * x4;
  p5 = p5 + f(0xBE1161F8u);                // PC3
  float p7 = x4 * p5;
  p6 = p6 + f(0x3E4CB79Fu);                // PC2
  p6 = p6 * x2;
  p7 = p7 + f(0xBEAAAA49u);                // PC1
  const float c = f((le ? 0u : 0x3FC90FDBu) ^ sgn);  // andnps PIO2; pxor sign
  p7 = p7 + p6;
  float pp = x2 * p7;
  pp = pp + f(0x3f800000u);                // PC0
  return c + s * pp;
}
}  // namespace cppwrf::mvec4
