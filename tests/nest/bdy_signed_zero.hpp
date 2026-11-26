// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Lateral-boundary forcing on fields of mostly zeros of both signs (the CPU and the CUDA nest tests): the model's
// slab kernel KBdySlabView3D against WRF's bdy_interp1 (fortran_nest_bridge.f90) on the one-process nest patch
// (nits:nite x njts:njte = nids:nide x njds:njde, the staggered ends, as WRF's nest_forcedown_interp.inc passes
// nips:nipe x njps:njpe). The nest field is +0, so each tendency rdt*(psca - nfld) keeps the sign of WRF's SINTB value.

#include <cstdint>
#include <vector>
#include "nest/nest_bdy.hpp"
#include "tests/nest/fortran_nest_bridge.h"

namespace cppwrf::nest::test {

struct SzRng {
  unsigned long long state;
  explicit SzRng(unsigned long long s) : state(s) {}
  unsigned next() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<unsigned>(state >> 33);
  }
  // 9 in 10 a zero of random sign, else a small or O(1) value of either sign
  template <class T>
  T value() {
    static const double nz[] = {1.0, -1.0, 0.5, -2.0, 1.0e-12, -1.0e-12};
    const unsigned r = next() % 1000;
    if (r < 900) return (r & 1) ? T(0.0) : T(-0.0);
    return T(nz[r % 6]);
  }
  // rows of period 4 (+0, -0, a zero of random sign, a nonzero value), from row `phase`: the y sweep window
  // (Z(-1), Z(0), Z(1), Z(2)) = (+0, -0, -0 or +0, nonzero) is where the loop copies' MN differ in the sign of zero
  // (the REAL*8 vector body's Z(1) against the scalar remainder's Z(-1); REAL*4 quads' Z(1) against W)
  template <class T>
  T row(int j, int phase) {
    switch ((j + phase) & 3) {
      case 0: return T(0.0);
      case 1: return T(-0.0);
      case 2: return (next() & 1) ? T(0.0) : T(-0.0);
      default: {
        static const double nz[] = {1.0, -1.0, 0.5, -2.0, 1.0e-12, -1.0e-12};
        return T(nz[next() % 6]);
      }
    }
  }
};

struct BdyZeroCase {
  int ratio, n_nx, n_ny, nk, ipos, jpos, c_nx, c_ny;
  bool xstag, ystag;
  static constexpr int kWidth = 5, kSz = 5;  // spec_bdy_width; MIN(MAX(spec_zone 1, relax_zone 4 + 1), 5)
  static constexpr double kCdt = 60.0;
  std::vector<double> cfld;  // cfld(1:c_nx+1, 1:nk, 1:c_ny+1)
  std::vector<double> nfld;  // nfld(1:n_nx+1, 1:nk, 1:n_ny+1), +0
  std::vector<double> f_xs, f_txs, f_xe, f_txe, f_ys, f_tys, f_ye, f_tye;  // WRF's (1:n?+1, 1:nk, 1:5)

  int cni() const { return c_nx + 1; }
  int cnj() const { return c_ny + 1; }
  int nni() const { return n_nx + 1; }
  int nnj() const { return n_ny + 1; }
  int npos(int side) const { return side < 2 ? (ystag ? n_ny + 1 : n_ny) : (xstag ? n_nx + 1 : n_nx); }

  // rows: each parent row constant in i, rows of SzRng::row (each II of a SINTB call sees the same columns: each
  // loop copy shows in the signs of zero), else every value drawn
  void make(SzRng& rng, bool rows = false) {
    cfld.resize(static_cast<std::size_t>(cni()) * nk * cnj());
    const int phase = static_cast<int>(rng.next() & 3);
    for (int j = 0; j < cnj(); ++j)
      for (int k = 0; k < nk; ++k) {
        const std::size_t r = (static_cast<std::size_t>(j) * nk + k) * cni();
        const double v = rng.row<double>(j, phase);
        for (int i = 0; i < cni(); ++i) cfld[r + i] = rows ? v : rng.value<double>();
      }
    nfld.assign(static_cast<std::size_t>(nni()) * nk * nnj(), 0.0);
    const std::size_t nxs = static_cast<std::size_t>(nnj()) * nk * kWidth, nys = static_cast<std::size_t>(nni()) * nk * kWidth;
    for (auto* v : {&f_xs, &f_txs, &f_xe, &f_txe}) v->assign(nxs, 7.0);
    for (auto* v : {&f_ys, &f_tys, &f_ye, &f_tye}) v->assign(nys, 7.0);
    std::vector<int> imask(static_cast<std::size_t>(nni()) * nnj(), 0);
    wrf_bridge_bdy_interp1(cfld.data(), 1, c_nx + 1, 1, nk + 1, 1, c_ny + 1,
                           1, cni(), 1, nk, 1, cnj(),
                           1, c_nx, 1, nk, 1, c_ny,
                           nfld.data(), kWidth,
                           1, n_nx + 1, 1, nk + 1, 1, n_ny + 1,
                           1, nni(), 1, nk, 1, nnj(),
                           1, n_nx + 1, 1, nk, 1, n_ny + 1,  // WRF's patch: the whole nest
                           0, imask.data(), xstag ? 1 : 0, ystag ? 1 : 0, ipos, jpos, ratio, ratio,
                           f_xs.data(), f_txs.data(), f_xe.data(), f_txe.data(),
                           f_ys.data(), f_tys.data(), f_ye.data(), f_tye.data(), kCdt, kCdt / ratio);
  }

  // the model's kernel on views of cfld / nfld (on the device: pointers to device copies)
  KBdySlabView3D functor(int side, const real* c, const real* n, real* val, real* tend) const {
    KBdySlabView3D k{};
    k.cfld = View3<const real>{c, cni() * nk, cni()};
    k.nfld = View3<const real>{n, nni() * nk, nni()};
    k.val_slab = val;
    k.tend_slab = tend;
    k.n_nx = n_nx;
    k.n_ny = n_ny;
    k.ipos = ipos;
    k.jpos = jpos;
    k.nri = ratio;
    k.nrj = ratio;
    k.cdt = kCdt;
    k.xstag = xstag;
    k.ystag = ystag;
    k.sz = kSz;
    k.side = side;
    k.npos = npos(side);
    k.nk = nk;
    return k;
  }

  // WRF's value of the model's slab entry (pos, k, b) of side: bdy_xs(nj,k,b+1), bdy_xe(nj,k,b+1), ...
  double wrf(int side, bool tend, int pos, int k, int b) const {
    const bool x = side < 2;
    const std::size_t n = x ? nnj() : nni();
    const std::size_t i = static_cast<std::size_t>(pos) + static_cast<std::size_t>(k) * n +
                          static_cast<std::size_t>(b) * n * nk;
    const std::vector<double>* v[4][2] = {{&f_xs, &f_txs}, {&f_xe, &f_txe}, {&f_ys, &f_tys}, {&f_ye, &f_tye}};
    return (*v[side][tend ? 1 : 0])[i];
  }
};

// The cases: ratios 3 and 5, mass / U / V staggering, nests of WRF's sizes (e_we - 1 and e_sn - 1 multiples of the
// ratio) whose SINTB trip counts (south / north: n_nx / ratio + 3) take every value modulo 4
inline std::vector<BdyZeroCase> bdy_zero_cases() {
  std::vector<BdyZeroCase> cs;
  for (int ratio : {3, 5})
    for (int st = 0; st < 3; ++st)
      for (int mx : {10, 11, 12, 13})
        for (int my : {10, 11}) {
          const int nx = (ratio == 3 ? mx : mx - 4) * ratio, ny = (ratio == 3 ? my : my - 4) * ratio;
          BdyZeroCase c{};
          c.ratio = ratio;
          c.n_nx = nx;
          c.n_ny = ny;
          c.nk = 2;
          c.ipos = 6;
          c.jpos = 5;
          c.c_nx = c.ipos + (nx + 1) / ratio + 8;
          c.c_ny = c.jpos + (ny + 1) / ratio + 8;
          c.xstag = st == 1;
          c.ystag = st == 2;
          cs.push_back(c);
        }
  return cs;
}

}  // namespace cppwrf::nest::test
