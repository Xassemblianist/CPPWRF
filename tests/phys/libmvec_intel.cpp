// SPDX-License-Identifier: GPL-3.0-or-later
// glibc libmvec's 2-lane vector log and log10 (_ZGVbN2v_log, _ZGVbN2v_log10) as they are on an Intel CPU, for the
// Fortran drivers of the differential tests (tests/phys/wrf_fortran.sh links this object before -lmvec, so the
// vectorised loops of WRF's objects call these instead of libmvec's). glibc's two functions take an approximate
// reciprocal with rcpps, whose result the hardware defines: on an AMD CPU libmvec's log / log10 differ from Intel's
// in the last bit for some inputs, and WRF's side of a test with them. Each lane here is cppwrf::mvec::log / log10
// (src/core/libmvec_x86.hpp: libmvec's SSE4 code statement by statement with Intel's rcpps table; on Intel equal to
// libmvec for every input, tools/colab/colab_mvec.sh), so WRF's code gives on every x86-64 CPU the bits it gives on
// Intel. On an AMD EPYC, libmvec's log made KF, Noah-MP and the MYNN surface layer differ, its log10 the WSM6 and
// Thompson reflectivity. Compiled without FMA contraction and without -march (IEEE double operations in libmvec's
// order). -DCPPWRF_MVEC_INTEL_NO_LOG / -DCPPWRF_MVEC_INTEL_NO_LOG10 leave one of them out (to find which one a
// difference comes from, e.g. as an LD_PRELOAD library).
// The float 4-lane powf (_ZGVbN4vv_powf) takes rcpps as well: WRF's real*4 build calls it in its vectorised REAL
// x**y loops (WSM6, the surface layer, ...), and on an AMD CPU it gives other bits than on Intel. Each lane here is
// cppwrf::mvec4::powf (src/core/libmvec_f4.hpp: libmvec's SSE4 code with Intel's rcpps table, the lanes CPPWRF's
// FP32 build computes), so the real*4 references made on a GitHub runner (AMD EPYC or Intel Xeon) are Intel's.
// -DCPPWRF_MVEC_INTEL_NO_POWF leaves it out.
// The other SSE4 vector functions of WRF's objects that take rsqrtps / rcpps: acos, asin (rsqrtps of 0.5 - 0.5|x|)
// and tan (rcpps in the branch of |x| > 128000), and the float acosf, asinf, tanf (|x| > 10000 ...) (exp, pow, sin,
// cos, atan, erf, sinh, cosh, tanh, atan2 and their float variants take neither). Here they are
// src/core/libmvec_sse4_trig.hpp (glibc's routines instruction by instruction with Intel's rsqrtps / rcpps; on
// Intel equal to libmvec, tests/unit/test_libmvec_trig.cpp). WRF calls them in curvature (map_proj 6 or polar:
// tan), gausll / lggaus (Gaussian grids: acos), the FARMS shortwave (acos) and the urban tree view factors (real*4:
// asinf), none of them a configuration CPPWRF runs; the real*8 and real*4 builds reference only the SSE2-ABI
// variants (_ZGVb*: FCOPTIM has no -march, so the AVX2 _ZGVd* variants are never called).
// -DCPPWRF_MVEC_INTEL_NO_ACOS, _NO_ASIN, _NO_TAN, _NO_ACOSF, _NO_ASINF, _NO_TANF leave one of them out.
#include <emmintrin.h>
#include <xmmintrin.h>

#include "core/libmvec_f4.hpp"
#include "core/libmvec_sse4_trig.hpp"
#include "core/libmvec_x86.hpp"

extern "C" {
#if !defined(CPPWRF_MVEC_INTEL_NO_LOG)
__m128d _ZGVbN2v_log(__m128d x) {
  alignas(16) double v[2];
  _mm_store_pd(v, x);
  v[0] = cppwrf::mvec::log(v[0]);
  v[1] = cppwrf::mvec::log(v[1]);
  return _mm_load_pd(v);
}
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_LOG10)
__m128d _ZGVbN2v_log10(__m128d x) {
  alignas(16) double v[2];
  _mm_store_pd(v, x);
  v[0] = cppwrf::mvec::log10(v[0]);
  v[1] = cppwrf::mvec::log10(v[1]);
  return _mm_load_pd(v);
}
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_POWF)
__m128 _ZGVbN4vv_powf(__m128 x, __m128 y) {
  alignas(16) float a[4], b[4];
  _mm_store_ps(a, x);
  _mm_store_ps(b, y);
  for (int l = 0; l < 4; ++l) a[l] = cppwrf::mvec4::powf(a[l], b[l]);
  return _mm_load_ps(a);
}
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_ACOS)
__m128d _ZGVbN2v_acos(__m128d x) { return _mm_castsi128_pd(cppwrf::mvec_sse4::acos2_sse4(_mm_castpd_si128(x))); }
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_ASIN)
__m128d _ZGVbN2v_asin(__m128d x) { return _mm_castsi128_pd(cppwrf::mvec_sse4::asin2_sse4(_mm_castpd_si128(x))); }
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_TAN)
__m128d _ZGVbN2v_tan(__m128d x) { return _mm_castsi128_pd(cppwrf::mvec_sse4::tan2_sse4(_mm_castpd_si128(x))); }
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_ACOSF)
__m128 _ZGVbN4v_acosf(__m128 x) { return _mm_castsi128_ps(cppwrf::mvec_sse4::acosf4_sse4(_mm_castps_si128(x))); }
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_ASINF)
__m128 _ZGVbN4v_asinf(__m128 x) { return _mm_castsi128_ps(cppwrf::mvec_sse4::asinf4_sse4(_mm_castps_si128(x))); }
#endif
#if !defined(CPPWRF_MVEC_INTEL_NO_TANF)
__m128 _ZGVbN4v_tanf(__m128 x) { return _mm_castsi128_ps(cppwrf::mvec_sse4::tanf4_sse4(_mm_castps_si128(x))); }
#endif
}
