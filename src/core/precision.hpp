// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Working precision. CPPWRF_DOUBLE is set by CMake option CPPWRF_DOUBLE.
namespace cppwrf {
#ifdef CPPWRF_DOUBLE
using real = double;
inline constexpr const char* precision_name = "float64";
#else
using real = float;
inline constexpr const char* precision_name = "float32";
#endif
}  // namespace cppwrf

// CPPWRF_LOCAL_ROUNDING: every floating-point operation of a kernel is rounded on its own, whatever the code around
// it: the CPU builds (-ffp-contract=off), CUDA FP64 and FP32 identity (--fmad=false), HIP (-ffp-contract=off). A
// kernel may then keep a value it computed instead of computing it again. Not in the fast CUDA FP32 build, where
// ptxas contracts a*b + c into an fma depending on the code around it: there kernels keep the form they had.
#if !(defined(CPPWRF_USE_CUDA) && !defined(CPPWRF_DOUBLE) && !defined(CPPWRF_FP32_IDENTITY))
#define CPPWRF_LOCAL_ROUNDING 1
#endif
