// SPDX-License-Identifier: GPL-3.0-or-later
// The float functions of the C math library as glibc 2.41 computes them (core/glibc_mathf.hpp), defined in the
// executable of an FP32 build (CMake: CPPWRF_DOUBLE=OFF on x86-64; the CPU build and the host code of the CUDA
// builds). A definition in the executable takes the place of libm's for every call of the program (std::exp(float),
// libm::exp(float), the scalar lanes of core/libmvec_f4.hpp, the sincosf that g++ makes of a sinf and a cosf of one
// argument, table and initialisation code), so the FP32 results do not depend on the libm of the host: WRF real*4's
// references were made against glibc 2.41, whose float functions differ from older versions' (2.40 and 2.41 replaced
// several with CORE-MATH's correctly rounded ones). Calls with constant arguments are folded by g++ (correctly
// rounded) as before. sqrtf, fmodf, roundf, lroundf, llroundf, fminf, fmaxf are exact by definition and stay libm's.
// errno is not set (no caller reads it). Proof: tools/glibc_mathf_proof.cpp (every input of each function against
// glibc 2.41), tests/unit/test_glibc_mathf.cpp; the ctest fp32_host_libm checks that the executable imports none of
// these from libm. asinf, acosf, log10f and tgammaf are libm's SVID wrappers around the kernels
// (core/glibc_mathf_compat.hpp: +NaN for a domain error).
#include "core/glibc_mathf_compat.hpp"

namespace g = cppwrf::glibcf;

extern "C" {
float expf(float x) noexcept { return g::expf(x); }
float logf(float x) noexcept { return g::logf(x); }
float powf(float x, float y) noexcept { return g::powf(x, y); }
float sinf(float x) noexcept { return g::sinf(x); }
float cosf(float x) noexcept { return g::cosf(x); }
void sincosf(float x, float* s, float* c) noexcept {  // glibc's sincosf: sinf's and cosf's results (all inputs)
  *s = g::sinf(x);
  *c = g::cosf(x);
}
float tanf(float x) noexcept { return g::tanf(x); }
float atanf(float x) noexcept { return g::atanf(x); }
float atan2f(float y, float x) noexcept { return g::atan2f(y, x); }
float asinf(float x) noexcept { return g::compat::asinf(x); }
float acosf(float x) noexcept { return g::compat::acosf(x); }
float sinhf(float x) noexcept { return g::sinhf(x); }
float tanhf(float x) noexcept { return g::tanhf(x); }
float log10f(float x) noexcept { return g::compat::log10f(x); }
float erff(float x) noexcept { return g::erff(x); }
float tgammaf(float x) noexcept { return g::compat::tgammaf(x); }
}
