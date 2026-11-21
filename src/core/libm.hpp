// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// cppwrf::libm: the C math library as the kernels call it (host and device), with std::'s overloads and promotions.
// CPU and CUDA FP32 builds: the std:: functions themselves (using-declarations: glibc's libm on the host, CUDA's math
// library on the device, exactly as a call of std::exp). The FP32 builds' executables define glibc 2.41's float
// functions themselves (core/glibc_mathf_host.cpp), so that a float call on the host computes those whatever libm the
// host has.
// CUDA FP64 build (CPPWRF_GLIBC_MATH, core/backend.hpp): the double functions are glibc's own algorithms
// (core/glibc_math.hpp) on the device and on the host, so that the GPU computes what WRF's real*8 build and the CPU
// build compute; float arguments still call std:: (CUDA's float functions on the device). Compile-time constants: gcc
// folds a math call with constant arguments (correctly rounded, as WRF's gfortran build and the CPU build) and
// pow(x, c) for a constant c = 2, 1, -1, 0 into x * x, x, 1 / x, 1; the host side of the FP64 CUDA build keeps these
// folds (__builtin_constant_p). Device code has no such test: a kernel writes them as WRF's compiled code computes
// them (x * x for x**2.0, a constant for a function of constants).
#include <cmath>
#include <type_traits>

#if defined(CPPWRF_GLIBC_MATH)
#include "core/glibc_math.hpp"
#endif
#if defined(CPPWRF_FP32_IDENTITY)
#include "core/glibc_mathf.hpp"
#endif

namespace cppwrf::libm {
#if !defined(CPPWRF_GLIBC_MATH)
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::cos;
using std::erf;
using std::exp;
using std::hypot;
using std::log;
using std::log10;
using std::pow;
using std::sin;
using std::sinh;
using std::tan;
using std::tanh;
using std::tgamma;
#else
#if defined(__CUDACC__)
#define CPPWRF_LM __host__ __device__ __forceinline__
#else
#define CPPWRF_LM inline
#endif
#if defined(__CUDA_ARCH__)
#define CPPWRF_LM_CONST(x) false
#else
#define CPPWRF_LM_CONST(x) __builtin_constant_p(x)
#endif
// one argument: float -> std:: (as before; the CUDA FP32 identity build: glibc's float function on the device, Gf),
// double -> glibc (a host constant: std::, folded), integral -> double
#if defined(CPPWRF_FP32_IDENTITY) && defined(__CUDA_ARCH__)
#define CPPWRF_LM_FLOAT(F, Gf) return Gf(a);
#else
#define CPPWRF_LM_FLOAT(F, Gf) return std::F(a);
#endif
#define CPPWRF_LM_FUNC1F(F, G, Gf)                                     \
  CPPWRF_LM float F(float a) { CPPWRF_LM_FLOAT(F, Gf) }                \
  CPPWRF_LM double F(double a) {                                        \
    if (CPPWRF_LM_CONST(a)) return std::F(a);                           \
    return G(a);                                                        \
  }                                                                     \
  template <class T, std::enable_if_t<std::is_integral_v<T>, int> = 0> \
  CPPWRF_LM double F(T a) {                                             \
    return F(double(a));                                                \
  }
CPPWRF_LM_FUNC1F(exp, glibc::exp, glibcf::expf)
CPPWRF_LM_FUNC1F(log, glibc::log, glibcf::logf)
CPPWRF_LM_FUNC1F(log10, glibc::log10, glibcf::log10f)
CPPWRF_LM_FUNC1F(atan, glibc::atan, glibcf::atanf)
CPPWRF_LM_FUNC1F(tanh, glibc::tanh, glibcf::tanhf)
CPPWRF_LM_FUNC1F(sin, glibc::sin, glibcf::sinf)
CPPWRF_LM_FUNC1F(cos, glibc::cos, glibcf::cosf)
CPPWRF_LM_FUNC1F(sinh, glibc::sinh, glibcf::sinhf)
CPPWRF_LM_FUNC1F(erf, glibc::erf, glibcf::erff)
CPPWRF_LM_FUNC1F(asin, glibc::asin, glibcf::asinf)
CPPWRF_LM_FUNC1F(acos, glibc::acos, glibcf::acosf)
CPPWRF_LM_FUNC1F(tan, glibc::tan, glibcf::tanf)
CPPWRF_LM_FUNC1F(tgamma, glibc::tgamma, glibcf::tgammaf)
#undef CPPWRF_LM_FUNC1F
#undef CPPWRF_LM_FLOAT
// atan2: two arguments as std:: (float, float) -> float, otherwise double
CPPWRF_LM double atan2_d(double a, double b) {
  if (CPPWRF_LM_CONST(a) && CPPWRF_LM_CONST(b)) return std::atan2(a, b);
  return glibc::atan2(a, b);
}
template <class A, class B, std::enable_if_t<std::is_arithmetic_v<A> && std::is_arithmetic_v<B>, int> = 0>
CPPWRF_LM auto atan2(A a, B b) {
  if constexpr (std::is_same_v<A, float> && std::is_same_v<B, float>) {
#if defined(CPPWRF_FP32_IDENTITY) && defined(__CUDA_ARCH__)
    return glibcf::atan2f(a, b);
#else
    return std::atan2(a, b);
#endif
  } else {
    return atan2_d(double(a), double(b));
  }
}
// hypot: as atan2
CPPWRF_LM double hypot_d(double a, double b) {
  if (CPPWRF_LM_CONST(a) && CPPWRF_LM_CONST(b)) return std::hypot(a, b);
  return glibc::hypot(a, b);
}
template <class A, class B, std::enable_if_t<std::is_arithmetic_v<A> && std::is_arithmetic_v<B>, int> = 0>
CPPWRF_LM auto hypot(A a, B b) {
  if constexpr (std::is_same_v<A, float> && std::is_same_v<B, float>) return std::hypot(a, b);
  else return hypot_d(double(a), double(b));
}

// pow: std::'s promotion (float if both arguments are float, double otherwise); gcc folds pow(x, 2.0) = x * x,
// pow(x, 1.0) = x, pow(x, -1.0) = 1 / x and pow(x, 0.0) = 1 for a constant exponent
CPPWRF_LM double pow_d(double a, double b) {
  if (CPPWRF_LM_CONST(b) && (b == 2.0 || b == 1.0 || b == -1.0 || b == 0.0 || CPPWRF_LM_CONST(a)))
    return std::pow(a, b);
  return glibc::pow(a, b);
}
template <class A, class B, std::enable_if_t<std::is_arithmetic_v<A> && std::is_arithmetic_v<B>, int> = 0>
CPPWRF_LM auto pow(A a, B b) {
  if constexpr (std::is_same_v<A, float> && std::is_same_v<B, float>) {
#if defined(CPPWRF_FP32_IDENTITY) && defined(__CUDA_ARCH__)
    return glibcf::powf(a, b);
#else
    return std::pow(a, b);
#endif
  } else {
    return pow_d(double(a), double(b));
  }
}
#endif
}  // namespace cppwrf::libm
