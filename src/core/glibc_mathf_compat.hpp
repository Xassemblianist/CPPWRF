// SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later
#pragma once
// The public asinf, acosf, log10f and tgammaf of glibc 2.41's x86-64 libm (host and device): the SVID compatibility
// wrappers math/w_asinf_compat.c, w_acosf_compat.c, w_log10f_compat.c and w_tgammaf_compat.c around the CORE-MATH
// kernels of core/glibc_mathf.hpp, as core/glibc_math_compat.hpp for the double functions: a domain error returns
// (float) __kernel_standard(...) = NAN (+qNaN, 0x7fc00000) where the kernel returns -qNaN (CUDA's canonical NaN on
// the device). In-domain inputs take the kernel itself. The other float functions (expf, logf, powf, sinf, cosf: the
// GLIBC_2.27 symbols; tanf, atanf, atan2f, tanhf, sinhf, erff) and libmvec's 4-lane ones return the kernels' values
// for every non-NaN input (tests/unit/test_libm_special.cpp). The FP32 builds' executables define the C functions
// with these (core/glibc_mathf_host.cpp); the CUDA / HIP FP32 identity build calls them on the device
// (core/libm.hpp, core/backend.hpp).
#include "core/glibc_mathf.hpp"

namespace cppwrf::glibcf::compat {
// w_asinf_compat.c:28-39: asinf(|x| > 1) is __kernel_standard_f(x, x, 102) = NAN
CPPWRF_GMF float asinf(float x) {
  return ::cppwrf::glibcf::detail::fabsff(x) > 1.0f ? __builtin_nanf("") : ::cppwrf::glibcf::asinf(x);
}
// w_acosf_compat.c:28-39: acosf(|x| > 1) is __kernel_standard_f(x, x, 101) = NAN
CPPWRF_GMF float acosf(float x) {
  return ::cppwrf::glibcf::detail::fabsff(x) > 1.0f ? __builtin_nanf("") : ::cppwrf::glibcf::acosf(x);
}
// w_log10f_compat.c:28-45: log10f(0) (x, x, 118) = -HUGE_VAL, log10f(x < 0) (x, x, 119) = NAN
CPPWRF_GMF float log10f(float x) {
  if (x <= 0.0f) {
    if (x == 0.0f) return -__builtin_huge_valf();
    return __builtin_nanf("");
  }
  return ::cppwrf::glibcf::log10f(x);
}
// w_tgammaf_compat.c:24-45: pole (x, x, 150) and overflow (x, x, 140) copysign(HUGE_VAL, x), a negative integer or
// -inf (x, x, 141) NAN, an underflow sets errno only
CPPWRF_GMF float tgammaf(float x) {
  namespace d = ::cppwrf::glibcf::detail;
  const float y = ::cppwrf::glibcf::tgammaf(x);
  const bool y_finite = (d::asuf(y) & 0x7fffffffu) < 0x7f800000u;
  const bool x_finite = (d::asuf(x) & 0x7fffffffu) < 0x7f800000u;
  const bool x_inf = (d::asuf(x) & 0x7fffffffu) == 0x7f800000u;
  if ((!y_finite || y == 0) && (x_finite || (x_inf && x < 0.0))) {
    if (x == (float)0.0) return float(d::copysignd(__builtin_huge_val(), double(x)));  // tgammaf pole
    if (d::floorff(x) == x && x < 0.0f) return __builtin_nanf("");                    // tgammaf domain
    if (y == 0) return y;                                                             // tgammaf underflow (ERANGE)
    return float(d::copysignd(__builtin_huge_val(), double(x)));                      // tgammaf overflow
  }
  return y;
}
}  // namespace cppwrf::glibcf::compat
