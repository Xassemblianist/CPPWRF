// SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later
#pragma once
// The public asin, acos, log10 and tgamma of glibc 2.41's x86-64 libm (host and device): the SVID compatibility
// wrappers math/w_asin_compat.c, w_acos_compat.c, w_log10_compat.c and w_tgamma_compat.c (LIBM_SVID_COMPAT,
// _LIB_VERSION = _POSIX_) around the kernels of core/glibc_math.hpp. The default symbols asin@@GLIBC_2.2.5 ...
// are these wrappers: a domain error returns __kernel_standard's NAN (+qNaN, 0x7ff8000000000000), not the kernel's
// (x - x) / (x - x) (-qNaN on x86-64; CUDA's canonical NaN on the device). In-domain inputs take the kernel itself,
// so every other result is the kernel's bit for bit. exp, log, pow (GLIBC_2.29 symbols), sin, cos, tan, atan, atan2,
// tanh, sinh, erf, hypot and libmvec's lanes return the kernels' values for every input (tests/unit/
// test_libm_special.cpp); libmvec's log10 calls the public log10 for its special lanes (core/libmvec_x86.hpp).
// errno and the exception flags are not set (as in the kernels). core/glibc_math.hpp itself stays the kernels
// (the code CPPMPAS shares; its src/core/libm.hpp applies the same asin / acos wrappers).
#include <cmath>

#include "core/glibc_math.hpp"

namespace cppwrf::glibc::compat {
// w_asin_compat.c:28-39: asin(|x| > 1) is __kernel_standard(x, x, 2) = NAN
CPPWRF_GM double asin(double x) {
  return ::cppwrf::glibc::detail::fabs_(x) > 1.0 ? __builtin_nan("") : ::cppwrf::glibc::asin(x);
}
// w_acos_compat.c:28-39: acos(|x| > 1) is __kernel_standard(x, x, 1) = NAN
CPPWRF_GM double acos(double x) {
  return ::cppwrf::glibc::detail::fabs_(x) > 1.0 ? __builtin_nan("") : ::cppwrf::glibc::acos(x);
}
// w_log10_compat.c:28-45: log10(0) is __kernel_standard(x, x, 18) = -HUGE_VAL, log10(x < 0) (x, x, 19) = NAN
CPPWRF_GM double log10(double x) {
  if (x <= 0.0) {
    if (x == 0.0) return -__builtin_huge_val();
    return __builtin_nan("");
  }
  return ::cppwrf::glibc::log10(x);
}
namespace detail {
CPPWRF_GM double floor_(double x) {
#if defined(CPPWRF_DEVICE_CODE)
  return ::floor(x);
#else
  return std::floor(x);
#endif
}
}  // namespace detail
// w_tgamma_compat.c:26-44 (glibc::tgamma is __ieee754_gamma_r with local_signgam applied): the pole x = +-0
// (x, x, 50) and an overflow (x, x, 40) are copysign(HUGE_VAL, x), a negative integer or -inf (x, x, 41) NAN, an
// underflow sets errno only
CPPWRF_GM double tgamma(double x) {
  namespace d = ::cppwrf::glibc::detail;
  const double y = ::cppwrf::glibc::tgamma(x);
  if ((!d::finite_(y) || y == 0) && (d::finite_(x) || (d::inf_(x) && x < 0.0))) {
    if (x == 0.0) return d::copysign(__builtin_huge_val(), x);             // tgamma pole
    if (detail::floor_(x) == x && x < 0.0) return __builtin_nan("");       // tgamma domain
    if (y == 0) return y;                                                  // tgamma underflow (ERANGE)
    return d::copysign(__builtin_huge_val(), x);                           // tgamma overflow
  }
  return y;
}
}  // namespace cppwrf::glibc::compat
