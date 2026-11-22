// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Strided per-column array views for the RRTMG ports. One code path serves both layouts:
//  - a single column in local arrays (stride 1, the CPU column driver);
//  - a batch of columns in device memory, column index fastest (stride = number of columns), so a
//    warp of threads working on neighbouring columns reads and writes coalesced.
// SD/SI: x[i] = p[i*s]; SD2: x[a][b] = p[(a*n + b)*s] (row length n).
#include <cstddef>

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"

namespace cppwrf::phys {

struct SD {
  rb* p;
  int s;
  CPPWRF_FORCEINLINE rb& operator[](int i) const { return p[std::ptrdiff_t(i) * s]; }
};

struct SI {
  int* p;
  int s;
  CPPWRF_FORCEINLINE int& operator[](int i) const { return p[std::ptrdiff_t(i) * s]; }
};

// read-only arrays seen through the (non-const) views: one column (stride 1) and column col of a
// batch of ncol columns
CPPWRF_FORCEINLINE SD sd_const(const rb* p) { return SD{const_cast<rb*>(p), 1}; }
CPPWRF_FORCEINLINE SD col_view(const rb* a, int col, int ncol) { return SD{const_cast<rb*>(a) + col, ncol}; }

// grows a workspace's rb and int buffers to a layout's sizes (lay.nd, lay.ni)
template <class Workspace, class Layout>
inline void ensure_workspace(Workspace& ws, const Layout& lay) {
  if (ws.d.size() < lay.nd) ws.d.assign(lay.nd, rb(0.0));
  if (ws.i.size() < lay.ni) ws.i.assign(lay.ni, 0);
}

struct SD2 {
  rb* p;
  int s;
  int n;
  CPPWRF_FORCEINLINE SD operator[](int a) const { return SD{p + std::ptrdiff_t(a) * n * s, s}; }
};

struct CSD {
  const rb* p;
  int s;
  CPPWRF_FORCEINLINE CSD() : p(nullptr), s(0) {}
  CPPWRF_FORCEINLINE CSD(const rb* p_, int s_) : p(p_), s(s_) {}
  CPPWRF_FORCEINLINE CSD(SD v) : p(v.p), s(v.s) {}
  CPPWRF_FORCEINLINE rb operator[](int i) const {
    return CPPWRF_LDG(p + std::ptrdiff_t(i) * s);
  }
};

struct CSI {
  const int* p;
  int s;
  CPPWRF_FORCEINLINE CSI() : p(nullptr), s(0) {}
  CPPWRF_FORCEINLINE CSI(const int* p_, int s_) : p(p_), s(s_) {}
  CPPWRF_FORCEINLINE CSI(SI v) : p(v.p), s(v.s) {}
  CPPWRF_FORCEINLINE int operator[](int i) const {
    return CPPWRF_LDG(p + std::ptrdiff_t(i) * s);
  }
};

struct CSD2 {
  const rb* p;
  int s;
  int n;
  CPPWRF_FORCEINLINE CSD2() : p(nullptr), s(0), n(0) {}
  CPPWRF_FORCEINLINE CSD2(const rb* p_, int s_, int n_) : p(p_), s(s_), n(n_) {}
  CPPWRF_FORCEINLINE CSD2(SD2 v) : p(v.p), s(v.s), n(v.n) {}
  CPPWRF_FORCEINLINE CSD operator[](int a) const { return CSD{p + std::ptrdiff_t(a) * n * s, s}; }
};

// Read-only views through ordinary (coherent) loads, for arrays the same kernel launch writes before it reads
// them: CSD / CSI / CSD2 load through __ldg (ld.global.nc), which is defined only for data no thread writes during
// the kernel's lifetime (CUDA C++ Programming Guide, __ldg; PTX ISA, ld.global.nc).
struct RSD {
  const rb* p;
  int s;
  CPPWRF_FORCEINLINE RSD(const rb* p_, int s_) : p(p_), s(s_) {}
  CPPWRF_FORCEINLINE RSD(SD v) : p(v.p), s(v.s) {}
  CPPWRF_FORCEINLINE rb operator[](int i) const { return p[std::ptrdiff_t(i) * s]; }
};

struct RSI {
  const int* p;
  int s;
  CPPWRF_FORCEINLINE RSI(SI v) : p(v.p), s(v.s) {}
  CPPWRF_FORCEINLINE int operator[](int i) const { return p[std::ptrdiff_t(i) * s]; }
};

struct RSD2 {
  const rb* p;
  int s;
  int n;
  CPPWRF_FORCEINLINE RSD2(SD2 v) : p(v.p), s(v.s), n(v.n) {}
  CPPWRF_FORCEINLINE RSD operator[](int a) const { return RSD{p + std::ptrdiff_t(a) * n * s, s}; }
};

// view of a local stride-1 array
CPPWRF_FORCEINLINE SD sd(rb* p) { return SD{p, 1}; }

}  // namespace cppwrf::phys
