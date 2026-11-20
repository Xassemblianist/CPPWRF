// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// glibc 2.41 libmvec's 4-lane float expf, logf and powf (_ZGVbN4v_expf, _ZGVbN4v_logf, _ZGVbN4vv_powf; the SSE4
// variants that glibc selects on every CPU with SSE4.1) re-implemented statement by statement, one lane, from
// sysdeps/x86_64/fpu/multiarch/svml_s_expf4_core_sse4.S, svml_s_logf4_core_sse4.S and svml_s_powf4_core_sse4.S.
// WRF's real*4 build (gfortran -O2 -ftree-vectorize, SSE2 ABI: 4 floats per call) calls them in its vectorised loops.
// expf and logf are IEEE single-precision operations only; powf works in double and takes an approximate reciprocal
// with `rcpps`, whose result the hardware defines: here Intel's (mvec::detail::kRcp, the table the double log uses;
// WRF's references are made on Intel), so the results are Intel libmvec's on any x86-64 CPU. Every other operation
// is an IEEE operation in the assembly's order (the build has -ffp-contract=off: no FMA contraction). Lanes the
// assembly sends to the scalar libm (its range masks) call the scalar function here too.
// Host only: the FP32 CUDA build keeps CUDA's float functions on the device (a port of these to the device, with
// glibc's scalar expf / logf / powf for the masked lanes, is open).
#include <cstdint>
#include <cstring>

#include "core/libmvec_f4_tables.hpp"
#include "core/libmvec_x86.hpp"

namespace cppwrf::mvec4 {
namespace detail {
inline float f(std::uint32_t b) {
  float v;
  std::memcpy(&v, &b, sizeof v);
  return v;
}
inline std::uint32_t fb(float v) {
  std::uint32_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}
inline std::int32_t s32(std::uint32_t b) { return static_cast<std::int32_t>(b); }
}  // namespace detail

// _ZGVbN4v_expf_sse4: exp(x) = 2^N * exp(r), N = rint(x/ln2), r = x - N*ln2 (hi + lo), degree-5 polynomial
inline float expf(float x) {
  using namespace mvec_tables;
  using detail::f;
  const std::uint32_t ix = detail::fb(x);
  if (detail::s32(ix & kExpf_iAbsMask) > detail::s32(kExpf_iDomainRange)) return std::exp(x);
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
inline float logf(float x) {
  using namespace mvec_tables;
  using detail::f;
  const std::uint32_t ix = detail::fb(x);
  // denormals, zero, negatives, Inf, NaN: pcmpgtd (LoRange > x + HiDelta, signed)
  if (detail::s32(kLogf_iLoRange) > detail::s32(ix + kLogf_iHiDelta)) return std::log(x);
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
inline float powf(float x, float y) {
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
  const double l = d(kLogLA[int(rcp) - 512]) + lq;
  const double lx = ke + l;
  const double yl = lx * yd;
  // iRangeMask: |hi word of y*log(x)| > DomainRange
  const bool special_r = detail::s32(std::uint32_t(bits(yl) >> 32) & kPowf_iAbsMask) > detail::s32(kPowf_iDomainRange);
  if (special_x || special_y || special_r) return std::pow(x, y);
  const double t = d(kPowf_dbInvLn2) * yl;
  const double s = (t - d(kPowf_dbHALF)) + d(kPowf_dbShifter);
  const double n = s - d(kPowf_dbShifter);
  const double r = (t - n) * d(kPowf_dbC1);
  const std::uint64_t sb = bits(s);
  const double tj = d(kPowfT[sb & kPowf_lbLOWKBITS]);
  const double res = tj + r * tj;
  return static_cast<float>(d(bits(res) + ((sb >> 11) << 52)));  // paddq of the exponent; cvtpd2ps
}
}  // namespace cppwrf::mvec4
