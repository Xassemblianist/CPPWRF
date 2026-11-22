// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Helpers of the NSSL microphysics port (WRF 4.8.0 phys/module_mp_nssl_2mom.F): Fortran-indexed arrays (column-major,
// declared bounds), the work space of one row (WRF calls the scheme for one j row at a time: nssl_2mom_driver's jy
// loop), gfortran's integer powers and NINT.
#include <cmath>
#include <cstddef>

#include "core/backend.hpp"
#include "core/libm.hpp"

namespace cppwrf::phys::mp_nssl {

// a fixed-size module array with Fortran bounds
template <class T, int LO, int HI>
struct FArr1 {
  T v[HI - LO + 1];
  CPPWRF_HD T& operator()(int i) { return v[i - LO]; }
  CPPWRF_HD const T& operator()(int i) const { return v[i - LO]; }
};
template <class T, int LO1, int HI1, int LO2, int HI2>
struct FArr2 {
  T v[(HI1 - LO1 + 1) * (HI2 - LO2 + 1)];
  CPPWRF_HD T& operator()(int i, int j) { return v[(i - LO1) + (HI1 - LO1 + 1) * (j - LO2)]; }
  CPPWRF_HD const T& operator()(int i, int j) const { return v[(i - LO1) + (HI1 - LO1 + 1) * (j - LO2)]; }
};

// a Fortran array of up to 4 dimensions over memory owned elsewhere (column-major, lower bounds l*, extents n*)
template <class T>
struct FA {
  T* p = nullptr;
  int l1 = 1, n1 = 1, l2 = 1, n2 = 1, l3 = 1, n3 = 1, l4 = 1;
  CPPWRF_HD T& operator()(int i) const { return p[i - l1]; }
  CPPWRF_HD T& operator()(int i, int j) const { return p[(i - l1) + n1 * (j - l2)]; }
  CPPWRF_HD T& operator()(int i, int j, int k) const { return p[(i - l1) + n1 * ((j - l2) + n2 * (k - l3))]; }
  CPPWRF_HD T& operator()(int i, int j, int k, int l) const {
    return p[(i - l1) + n1 * ((j - l2) + n2 * ((k - l3) + n3 * (l - l4)))];
  }
  // the same memory with other bounds (a Fortran dummy array of explicit shape)
  CPPWRF_HD FA reshape(int a1, int b1, int a2 = 1, int b2 = 1, int a3 = 1, int b3 = 1, int a4 = 1) const {
    FA r;
    r.p = p; r.l1 = a1; r.n1 = b1 - a1 + 1; r.l2 = a2; r.n2 = b2 - a2 + 1; r.l3 = a3; r.n3 = b3 - a3 + 1; r.l4 = a4;
    return r;
  }
  // the array that starts at element (i, j, k, l) (Fortran passes a(i,j,k,l) as the first element of a dummy array)
  CPPWRF_HD T* at(int i, int j = 1, int k = 1, int l = 1) const { return &(*this)(i, j, k, l); }
};

// the work space of one row: arrays taken in order (a callee takes its arrays after the caller's: Work by value)
struct Work {
  char* p = nullptr;
  char* end = nullptr;
  template <class T>
  CPPWRF_HD FA<T> arr(int a1, int b1, int a2 = 1, int b2 = 1, int a3 = 1, int b3 = 1, int a4 = 1, int b4 = 1) {
    FA<T> r;
    r.l1 = a1; r.n1 = b1 - a1 + 1; r.l2 = a2; r.n2 = b2 - a2 + 1; r.l3 = a3; r.n3 = b3 - a3 + 1; r.l4 = a4;
    const long n = long(r.n1) * r.n2 * r.n3 * (b4 - a4 + 1);
    r.p = reinterpret_cast<T*>(p);
    p += ((std::size_t(n > 0 ? n : 0) * sizeof(T) + 15) / 16) * 16;
    if (p > end) overflow();
    for (long q = 0; q < n; ++q) r.p[q] = T(0);
    return r;
  }
  CPPWRF_HD static void overflow() {
#if defined(CPPWRF_DEVICE_CODE)
    CPPWRF_TRAP();
#else
    throw_overflow();
#endif
  }
  static void throw_overflow();
};

// gfortran's x**n for an integer constant n (powi_as_mults: GCC's power tree, tree-ssa-math-opts.cc powi_table)
template <int N>
struct PowiTab {
  static constexpr int t[] = {0, 1, 1, 2, 2, 3, 3, 4, 4, 6, 5, 6, 6, 10, 7, 9, 8};
};
template <int N, class T>
CPPWRF_HD inline T ipow_pos(T x) {
  if constexpr (N == 0) {
    return T(1);
  } else if constexpr (N == 1) {
    return x;
  } else {
    static_assert(N <= 16, "ipow: extend the power table");
    constexpr int tn = PowiTab<0>::t[N];
    return ipow_pos<N - tn>(x) * ipow_pos<tn>(x);
  }
}
template <int N, class T>
CPPWRF_HD inline T ipow(T x) {
  if constexpr (N < 0) {
    return T(1) / ipow_pos<-N>(x);
  } else {
    return ipow_pos<N>(x);
  }
}

// NINT (gfortran: lround; x86-64's result also on the device for NaN, +-Inf and |x| >= 2^63)
CPPWRF_HD inline int nint(double x) { return int(::lround_x86(x)); }
CPPWRF_HD inline int nint(float x) { return int(::lround_x86(x)); }
// SIGN(a, b)
template <class T>
CPPWRF_HD inline T fsign(T a, T b) { return std::copysign(a, b); }

#if defined(CPPWRF_DEVICE_CODE)
#define NSSL_STOP() CPPWRF_TRAP()
#else
#define NSSL_STOP() ::cppwrf::phys::mp_nssl::nssl_stop(__LINE__)
#endif
[[noreturn]] void nssl_stop(int line);

}  // namespace cppwrf::phys::mp_nssl

namespace cppwrf::phys::mp_nssl {
template <class T>
CPPWRF_HD inline FA<T> mkfa(T* p, int a1, int b1, int a2 = 1, int b2 = 1, int a3 = 1, int b3 = 1, int a4 = 1, int b4 = 1) {
  FA<T> r;
  r.p = p; r.l1 = a1; r.n1 = b1 - a1 + 1; r.l2 = a2; r.n2 = b2 - a2 + 1; r.l3 = a3; r.n3 = b3 - a3 + 1; r.l4 = a4;
  (void)b4;
  return r;
}
template <class T>
CPPWRF_HD inline FA<T> mkfa(const T* p, int a1, int b1, int a2 = 1, int b2 = 1, int a3 = 1, int b3 = 1, int a4 = 1,
                            int b4 = 1) {
  return mkfa(const_cast<T*>(p), a1, b1, a2, b2, a3, b3, a4, b4);
}
}  // namespace cppwrf::phys::mp_nssl
namespace cppwrf::phys::mp_nssl {
// a branch of a configuration that the port rejects (ipconc 8: the 3-moment tables dab0lu / dab1lu)
template <class R>
CPPWRF_HD inline R nssl_trap() {
  NSSL_STOP();
  return R(0);
}
}  // namespace cppwrf::phys::mp_nssl
