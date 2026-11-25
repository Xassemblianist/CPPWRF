// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// ============================================================================
// CPPWRF moving nests: the host-side numerics of a nest move (WRF 4.8.0 built with "preset moves", -DMOVE_NESTS)
//   shift_domain_em (dyn_em/shift_domain_em.F, frame/loop_based_x_shift_code.h / _y_): every state array of the
//     nest moves by one parent cell (parent_grid_ratio nest points) in x, then in y, with Fortran array semantics
//     over the whole memory rows (the points brought in from beyond the domain are the memory halo's values);
//   the interpolation masks imask_nostag / _xstag / _ystag / _xystag: 1 everywhere, 0 on the patch, then shifted
//     with the fields, so that 1 marks the new points;
//   the nest interpolation functions of share/interp_fcn.F as med_interp_domain runs them after a move (only the
//     points with imask = 1, except interp_fcnm and interp_mask_water_field, which write every point).
// The arrays are host copies of the fields' whole allocations (core/field.hpp Layout, halo included). The functions
// index them with WRF's 1-based (i, k, j): nest point (ni, nk, nj) is element (ni-1, nj-1, nk-1) of the nest's
// Layout, coarse point (ci, ck, cj) element (ci-1, cj-1, ck-1) of the parent's. A serial wrf.exe has one patch: the
// tile ranges are nits = 1, nite = ide (every stagger), njts = 1, njte = jde.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/field.hpp"
#include "nest/nest_interp.hpp"

namespace cppwrf::nest::move {

// a host copy of a field's allocation, indexed as WRF's (i, k, j), 1-based
template <class T>
struct HView {
  T* p = nullptr;
  const Layout* L = nullptr;
  T& operator()(int i, int k, int j) const { return p[L->index(i - 1, j - 1, k - 1)]; }
};

// The four interpolation masks over the nest's memory, 0-based i in [-halo, nx + 1 + halo), j likewise (WRF's
// memory ims:ime, jms:jme hold at least the 3 points beyond ide / jde that the shift reads)
struct Masks {
  enum Which { NOSTAG = 0, XSTAG = 1, YSTAG = 2, XYSTAG = 3 };
  static constexpr int h = Grid::halo;
  int nx = 0, ny = 0;  // the nest's mass points
  std::vector<int> m[4];
  int ni() const { return nx + 1 + 2 * h; }
  int nj() const { return ny + 1 + 2 * h; }
  // 0-based index
  int& at(int w, int i0, int j0) { return m[w][std::size_t(j0 + h) * std::size_t(ni()) + std::size_t(i0 + h)]; }
  int at(int w, int i0, int j0) const { return m[w][std::size_t(j0 + h) * std::size_t(ni()) + std::size_t(i0 + h)]; }
  // WRF's 1-based (ni, nj)
  int f(int w, int i, int j) const { return at(w, i - 1, j - 1); }
  bool in_mem(int i, int j) const { return i - 1 >= -h && i - 1 < nx + 1 + h && j - 1 >= -h && j - 1 < ny + 1 + h; }

  // shift_domain_em.F:64-72: 1 everywhere, 0 on the patch (ips:min(ide-1,ipe) for the unstaggered direction,
  // ips:ipe for the staggered one; one patch: ipe = ide)
  void reset(int nx_, int ny_) {
    nx = nx_;
    ny = ny_;
    for (auto& v : m) v.assign(std::size_t(ni()) * std::size_t(nj()), 1);
    for (int w = 0; w < 4; ++w) {
      const int ie = (w == XSTAG || w == XYSTAG) ? nx : nx - 1;  // 0-based last
      const int je = (w == YSTAG || w == XYSTAG) ? ny : ny - 1;
      for (int j = 0; j <= je; ++j)
        for (int i = 0; i <= ie; ++i) at(w, i, j) = 0;
    }
  }
  // loop_based_x_shift_code.h for the masks themselves (Registry state: imask_xstag "X", imask_ystag "Y",
  // imask_xystag "XY"): ips:ipf over every memory row, ipf = ide if the stagger has X, else ide-1
  void shift_x(int px) {
    for (int w = 0; w < 4; ++w) {
      const int ipf = (w == XSTAG || w == XYSTAG) ? nx : nx - 1;
      std::vector<int> row(static_cast<std::size_t>(ni()));
      for (int j = -h; j < ny + 1 + h; ++j) {
        for (int i = -h; i < nx + 1 + h; ++i) row[std::size_t(i + h)] = at(w, i, j);
        for (int i = 0; i <= ipf; ++i) at(w, i, j) = row[std::size_t(i + px + h)];
      }
    }
  }
  void shift_y(int py) {
    for (int w = 0; w < 4; ++w) {
      const int jpf = (w == YSTAG || w == XYSTAG) ? ny : ny - 1;
      std::vector<int> col(static_cast<std::size_t>(nj()));
      for (int i = -h; i < nx + 1 + h; ++i) {
        for (int j = -h; j < ny + 1 + h; ++j) col[std::size_t(j + h)] = at(w, i, j);
        for (int j = 0; j <= jpf; ++j) at(w, i, j) = col[std::size_t(j + py + h)];
      }
    }
  }
};

// frame/loop_based_x_shift_code.h on one field's host copy: p(ips:ipf, :, jms:jme) = p(ips+px:ipf+px, :, jms:jme),
// ipf = ide for a field staggered in x (Layout nx = ide), else ide-1 (Layout nx = ide-1): ipf = L.nx - 1, 0-based
inline void shift_x(unsigned char* buf, const Layout& L, int esz, int px) {
  const int ipf = L.nx - 1, nz = std::max(1, L.nz);
  if (L.si != 1) throw std::logic_error("nest move: a field with a horizontal stride other than 1");
  for (int k = 0; k < nz; ++k)
    for (int j = L.wj0; j < L.wj1; ++j) {
      unsigned char* row = buf + L.index(0, j, k) * std::size_t(esz);
      std::memmove(row, row + std::ptrdiff_t(px) * esz, std::size_t(ipf + 1) * std::size_t(esz));
    }
}
// loop_based_y_shift_code.h: p(ims:ime, :, jps:jpf) = p(ims:ime, :, jps+py:jpf+py), jpf = L.ny - 1 (0-based)
inline void shift_y(unsigned char* buf, const Layout& L, int esz, int py) {
  const int jpf = L.ny - 1, nz = std::max(1, L.nz);
  const std::size_t n = std::size_t(L.wi1 - L.wi0) * std::size_t(esz);
  for (int k = 0; k < nz; ++k) {
    auto row = [&](int j) { return buf + L.index(L.wi0, j, k) * std::size_t(esz); };
    if (py > 0)
      for (int j = 0; j <= jpf; ++j) std::memmove(row(j), row(j + py), n);
    else
      for (int j = jpf; j >= 0; --j) std::memmove(row(j), row(j + py), n);
  }
}

// share/interp_fcn.F:814 nest_loc_of_cg (REAL)
inline real nest_loc_of_cg(int ci, int ipos, int nri, int ioff) {
  return real((ci - ipos) * nri) + real(1 - ioff) * real(nri + 1) / real(2.) + real(ioff);
}
// share/interp_fcn.F:836 compute_CGLL
inline int compute_cgll(int ni, int ipos, int nri, int istag) {
  const int inc = (ni - 1) / nri, ioff = nri % 2;
  int loc;
  if (istag == 0) loc = ((ni - 1) % nri) / (nri + ioff) - istag;
  else loc = (((ni - 1) % nri) + ioff) / ((nri + ioff) / 2) - istag;
  return ipos + inc + loc;
}

// The geometry of one interpolation: the nest's domain end (ide, jde: mass points + 1), its position and ratio
struct Geo {
  int nide = 0, njde = 0;  // nest ide / jde (1-based)
  int ipos = 1, jpos = 1, nri = 3, nrj = 3;
  int nims() const { return 1 - Grid::halo; }
  int nime() const { return nide + Grid::halo; }
  int njms() const { return 1 - Grid::halo; }
  int njme() const { return njde + Grid::halo; }
};

// share/interp_fcn.F:874 interp_fcn_sint (interp_method_type 2), nk levels. The SINT of a coarse cell depends only
// on the 5 x 5 coarse values around it, so a fine point takes the cell's SINT value when the cell's icmask is set and
// the coarse value otherwise, as psca after share/sint.F.
inline void interp_fcn_sint(const HView<real>& c, const HView<real>& n, int nk, const Masks& M, int mw, bool xstag,
                            bool ystag, const Geo& G) {
  const int nri = G.nri, nrj = G.nrj, ipos = G.ipos, jpos = G.jpos;
  int ioff = 0, joff = 0, nioff = 0, njoff = 0;
  if (xstag) { ioff = (nri - 1) / 2; nioff = nri; }
  if (ystag) { joff = (nrj - 1) / 2; njoff = nrj; }
  const int nrio2 = nri / 2, nrjo2 = nrj / 2;
  const int nits = 1, nite = G.nide, njts = 1, njte = G.njde;
  const int nims = G.nims(), nime = G.nime(), njms = G.njms(), njme = G.njme();
  auto icmask = [&](int i, int j) {
    const int ni = (i - ipos) * nri + (nrio2 + 1), nj = (j - jpos) * nrj + (nrjo2 + 1);
    bool r = false;
    if (ni >= nits - nioff - nrio2 && ni <= nite + nioff + nrio2 && nj >= njts - njoff - nrjo2 &&
        nj <= njte + njoff + nrjo2) {
      if (ni >= nims && ni <= nime && nj >= njms && nj <= njme)
        if (M.f(mw, ni, nj) == 1) r = true;
      if (ni - nioff >= nims && ni <= nime && nj - njoff >= njms && nj <= njme)
        if (ni >= nits - nioff && nj >= njts - njoff)
          if (M.f(mw, ni - nioff, nj - njoff) == 1) r = true;
    }
    return r;
  };
  for (int k = 1; k <= nk; ++k)
    for (int nj = njts; nj <= njte + joff; ++nj) {
      const int cj = jpos + (nj - 1) / nrj, jp = (nj - 1) % nrj;
      for (int ni = nits; ni <= nite + ioff; ++ni) {
        const int ci = ipos + (ni - 1) / nri, ip = (ni - 1) % nri;
        if (!(ni - ioff >= nits && nj - joff >= njts)) continue;
        if (!(M.f(mw, ni, nj) == 1 || M.f(mw, ni - ioff, nj - joff) == 1)) continue;
        real v;
        if (icmask(ci, cj)) {
          nreal ax, ay, patch[5][5];
          compute_sint_subgrid_offsets(nri, xstag, ystag, ip, jp, ax, ay);
          for (int dj = -2; dj <= 2; ++dj)
            for (int di = -2; di <= 2; ++di) patch[dj + 2][di + 2] = nreal(c(ci + di, k, cj + dj));
          v = real(sint_point_2d(patch, ax, ay));
        } else {
          v = c(ci, k, cj);
        }
        n(ni - ioff, k, nj - joff) = v;
      }
    }
}

// interp_fcni / interp_fcnm_imask / interp_fcnm_lu (without TERRAIN_AND_LANDUSE): the coarse value of the cell, at the
// points with imask = 1; interp_fcnm (masked = false): at every point of the nest
template <class T>
void interp_nearest(const HView<T>& c, const HView<T>& n, int nk, const Masks& M, int mw, const Geo& G, bool masked) {
  for (int nj = 1; nj <= G.njde; ++nj) {
    const int cj = G.jpos + (nj - 1) / G.nrj;
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        if (masked && M.f(mw, ni, nj) != 1) continue;
        n(ni, k, nj) = c(G.ipos + (ni - 1) / G.nri, k, cj);
      }
  }
}

// share/interp_fcn.F:4075 interp_mask_field (unstaggered fields): clu / nlu the land use (lu_index), iswater the
// nest's flag (nflag)
inline void interp_mask_field(const HView<real>& c, const HView<real>& n, int nk, const Masks& M, int mw,
                              const HView<real>& clu, const HView<real>& nlu, int iswater, const Geo& G) {
  const int nri = G.nri, nrj = G.nrj;
  auto nint = [](real x) { return int(std::lround(x)); };
  for (int nj = 1; nj <= G.njde; ++nj) {
    const int cj = nrj % 2 == 0 ? (nj + (nrj / 2) - 1) / nrj + G.jpos - 1 : (nj + (nrj - 1) / 2) / nrj + G.jpos - 1;
    for (int nk_ = 1; nk_ <= nk; ++nk_)
      for (int ni = 1; ni <= G.nide; ++ni) {
        if (M.f(mw, ni, nj) != 1) continue;
        const int ci = nri % 2 == 0 ? (ni + (nri / 2) - 1) / nri + G.ipos - 1 : (ni + (nri - 1) / 2) / nri + G.ipos - 1;
        const real dx = nri % 2 == 0 ? (real((ni + (nri - 1) / 2) % nri) + real(0.5)) / real(nri)
                                     : real((ni + (nri - 1) / 2) % nri) / real(nri);
        const real dy = nrj % 2 == 0 ? (real((nj + (nrj - 1) / 2) % nrj) + real(0.5)) / real(nrj)
                                     : real((nj + (nrj - 1) / 2) % nrj) / real(nrj);
        // ALL( clu(ci:ci+1,cj:cj+1) == iswater ) / ALL( ... /= iswater ): REAL compared with the INTEGER flag
        const real fw = real(iswater);
        const bool all_w = clu(ci, 1, cj) == fw && clu(ci + 1, 1, cj) == fw && clu(ci, 1, cj + 1) == fw &&
                           clu(ci + 1, 1, cj + 1) == fw;
        const bool all_nw = clu(ci, 1, cj) != fw && clu(ci + 1, 1, cj) != fw && clu(ci, 1, cj + 1) != fw &&
                            clu(ci + 1, 1, cj + 1) != fw;
        const int k = nk_;
        if (all_w || all_nw) {
          n(ni, k, nj) = (real(1.) - dx) * ((real(1.) - dy) * c(ci, k, cj) + dy * c(ci, k, cj + 1)) +
                         dx * ((real(1.) - dy) * c(ci + 1, k, cj) + dy * c(ci + 1, k, cj + 1));
          continue;
        }
        const bool water = nint(nlu(ni, 1, nj)) == iswater;
        int icount = 0;
        real sum = 0;
        auto take = [&](int a, int b) {
          if ((nint(clu(a, 1, b)) == iswater) == water) {
            ++icount;
            sum = sum + c(a, k, b);
          }
        };
        take(ci, cj);
        take(ci + 1, cj);
        take(ci, cj + 1);
        take(ci + 1, cj + 1);
        n(ni, k, nj) = sum / real(icount);
      }
  }
}

// share/interp_fcn.F:4278 interp_mask_soil (INTEGER isltyp): iswater = nl_get_iswater(1, ...) (domain 1's)
template <class T>
void interp_mask_soil(const HView<T>& c, const HView<T>& n, int nk, const Masks& M, int mw, const HView<real>& clu,
                      const HView<real>& nlu, int iswater, const Geo& G) {
  constexpr int isoilwater = 14;
  auto nint = [](real x) { return int(std::lround(x)); };
  for (int nj = 1; nj <= G.njde; ++nj) {
    const int cj = G.jpos + (nj - 1) / G.nrj;
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        const int ci = G.ipos + (ni - 1) / G.nri;
        if (M.f(mw, ni, nj) != 1) continue;
        if (nint(nlu(ni, 1, nj)) == iswater) n(ni, k, nj) = nint(clu(ci, 1, cj)) == iswater ? c(ci, k, cj) : T(-1);
        else n(ni, k, nj) = nint(clu(ci, 1, cj)) != iswater ? c(ci, k, cj) : T(-1);
      }
  }
  for (int nj = 1; nj <= G.njde; ++nj)
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        if (M.f(mw, ni, nj) != 1) continue;
        if (n(ni, k, nj) == T(-1) && nint(nlu(ni, 1, nj)) == iswater) n(ni, k, nj) = T(isoilwater);
      }
  // IF ( ANY(nfld .EQ. -1) ): a masked point still -1 makes it true, and only those change
  for (int nj = 1; nj <= G.njde; ++nj)
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        if (M.f(mw, ni, nj) != 1) continue;
        if (n(ni, k, nj) == T(-1)) n(ni, k, nj) = T(8);
      }
}

// share/interp_fcn.F:3382 interp_mask_water_field: every nest point (the mask test is commented out in WRF);
// nflag the nest's flag (islake / iswater); cits..cite, cjts..cjte the intermediate domain's patch and cide, cjde its
// end (the search of a lake point without lake cells around it)
inline void interp_mask_water_field(const HView<real>& c, const HView<real>& n, int nk, const HView<real>& clu,
                                    const HView<real>& nlu, int nflag, const Geo& G, int cits, int cite, int cide,
                                    int cjts, int cjte, int cjde) {
  const int nri = G.nri, nrj = G.nrj;
  auto nint = [](real x) { return int(std::lround(x)); };
  auto cell = [&](int ni, int nj, int& ci, int& cj) {
    cj = nrj % 2 == 0 ? (nj + (nrj / 2) - 1) / nrj + G.jpos - 1 : (nj + (nrj - 1) / 2) / nrj + G.jpos - 1;
    ci = nri % 2 == 0 ? (ni + (nri / 2) - 1) / nri + G.ipos - 1 : (ni + (nri - 1) / 2) / nri + G.ipos - 1;
  };
  for (int nj = 1; nj <= G.njde; ++nj)
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        int ci, cj;
        cell(ni, nj, ci, cj);
        const real dx = nri % 2 == 0 ? (real((ni + (nri - 1) / 2) % nri) + real(0.5)) / real(nri)
                                     : real((ni + (nri - 1) / 2) % nri) / real(nri);
        const real dy = nrj % 2 == 0 ? (real((nj + (nrj - 1) / 2) % nrj) + real(0.5)) / real(nrj)
                                     : real((nj + (nrj - 1) / 2) % nrj) / real(nrj);
        const bool nw = nint(nlu(ni, 1, nj)) == nflag;
        const bool c00 = nint(clu(ci, 1, cj)) == nflag, c10 = nint(clu(ci + 1, 1, cj)) == nflag,
                   c01 = nint(clu(ci, 1, cj + 1)) == nflag, c11 = nint(clu(ci + 1, 1, cj + 1)) == nflag;
        if (!nw) {
          n(ni, k, nj) = c(ci, k, cj);
        } else if (c00 && c10 && c01 && c11) {
          n(ni, k, nj) = (real(1.) - dx) * ((real(1.) - dy) * c(ci, k, cj) + dy * c(ci, k, cj + 1)) +
                         dx * ((real(1.) - dy) * c(ci + 1, k, cj) + dy * c(ci + 1, k, cj + 1));
        } else if (!c00 && !c10 && !c01 && !c11) {
          n(ni, k, nj) = real(-4);
        } else {
          int icount = 0;
          real sum = 0;
          if (c00) { ++icount; sum = sum + c(ci, k, cj); }
          if (c10) { ++icount; sum = sum + c(ci + 1, k, cj); }
          if (c01) { ++icount; sum = sum + c(ci, k, cj + 1); }
          if (c11) { ++icount; sum = sum + c(ci + 1, k, cj + 1); }
          n(ni, k, nj) = sum / real(icount);
        }
      }
  // the average of the nest's water points that are not -1 (one patch: wrf_dm_sum_reals is the sum itself)
  std::vector<real> avg_n(std::size_t(nk) + 1, real(0));
  {
    std::vector<real> sum_n(std::size_t(nk) + 1, real(0));
    std::vector<int> icount_n(std::size_t(nk) + 1, 0);
    for (int nj = 1; nj <= G.njde; ++nj)
      for (int k = 1; k <= nk; ++k)
        for (int ni = 1; ni <= G.nide; ++ni)
          if (n(ni, k, nj) != real(-1) && nint(nlu(ni, 1, nj)) == nflag) {
            ++icount_n[std::size_t(k)];
            sum_n[std::size_t(k)] = sum_n[std::size_t(k)] + n(ni, k, nj);
          }
    for (int k = 1; k <= nk; ++k)
      if (icount_n[std::size_t(k)] > 0) avg_n[std::size_t(k)] = sum_n[std::size_t(k)] / real(icount_n[std::size_t(k)]);
  }
  constexpr int max_search = 5;
  for (int nj = 1; nj <= G.njde; ++nj)
    for (int k = 1; k <= nk; ++k)
      for (int ni = 1; ni <= G.nide; ++ni) {
        if (n(ni, k, nj) != real(-4)) continue;
        int ci, cj;
        cell(ni, nj, ci, cj);
        const int ist = std::max(ci - max_search, cits), ien = std::min({ci + max_search, cite, cide - 1});
        const int jst = std::max(cj - max_search, cjts), jen = std::min({cj + max_search, cjte, cjde - 1});
        int icount = 0;
        real sum = 0;
        for (int jj = jst; jj <= jen; ++jj)
          for (int ii = ist; ii <= ien; ++ii)
            if (nint(clu(ii, 1, jj)) == nflag) {
              ++icount;
              sum = sum + c(ii, k, jj);
            }
        n(ni, k, nj) = icount > 0 ? sum / real(icount) : avg_n[std::size_t(k)];
      }
}

// share/interp_fcn.F:283 interp_fcn_blint_ll (longitudes): nlat the nest's latitudes (already interpolated),
// ninput_from_file the nest's input_from_file
inline void interp_fcn_blint_ll(const HView<real>& cinp, const HView<real>& n, const Masks& M, int mw, bool xstag,
                                bool ystag, const HView<real>& nlat, bool ninput_from_file, const Geo& G) {
  const int istag = xstag ? 0 : 1, ioff = xstag ? 1 : 0, jstag = ystag ? 0 : 1, joff = ystag ? 1 : 0;
  const int nri = G.nri, nrj = G.nrj, ipos = G.ipos, jpos = G.jpos;
  const int ie = G.nide - istag, je = G.njde - jstag;
  real max_lat = -90, min_lat = 90;
  for (int nj = 1; nj <= je; ++nj)
    for (int ni = 1; ni <= ie; ++ni) {
      max_lat = std::max(nlat(ni, 1, nj), max_lat);
      min_lat = std::min(nlat(ni, 1, nj), min_lat);
    }
  if ((max_lat > 85 || std::abs(min_lat) > 85) && !ninput_from_file)
    throw std::runtime_error("Nest over the pole, single input domain, longitudes will be wrong");
  real max_lon = -180, min_lon = 180;
  for (int nj = 1; nj <= je; ++nj) {
    const int cj = compute_cgll(nj, jpos, nrj, jstag);
    for (int ni = 1; ni <= ie; ++ni) {
      const int ci = compute_cgll(ni, ipos, nri, istag);
      max_lon = std::max(cinp(ci, 1, cj), max_lon);
      min_lon = std::min(cinp(ci, 1, cj), min_lon);
    }
  }
  const bool dateline = max_lon - min_lon > 300;
  auto cf = [&](int ci, int cj) {
    const real x = cinp(ci, 1, cj);
    return x < 0 && dateline ? real(360) + x : x;
  };
  for (int nj = 1; nj <= je; ++nj) {
    const int cj = compute_cgll(nj, jpos, nrj, jstag);
    const real ny = real(nj);
    const real cyp0 = nest_loc_of_cg(cj, jpos, nrj, joff), cyp1 = nest_loc_of_cg(cj + 1, jpos, nrj, joff);
    const real wy = (cyp1 - ny) / (cyp1 - cyp0);
    for (int ni = 1; ni <= ie; ++ni) {
      if (M.f(mw, ni, nj) != 1) continue;
      const int ci = compute_cgll(ni, ipos, nri, istag);
      const real nx = real(ni);
      const real cxp0 = nest_loc_of_cg(ci, ipos, nri, ioff), cxp1 = nest_loc_of_cg(ci + 1, ipos, nri, ioff);
      const real wx = (cxp1 - nx) / (cxp1 - cxp0);
      const real ll = cf(ci, cj), lr = cf(ci + 1, cj), ul = cf(ci, cj + 1), ur = cf(ci + 1, cj + 1);
      n(ni, 1, nj) = wy * (ll * wx + lr * (real(1.) - wx)) + (real(1.) - wy) * (ul * wx + ur * (real(1.) - wx));
    }
  }
  for (int nj = 1; nj <= je; ++nj)
    for (int ni = 1; ni <= ie; ++ni)
      if (n(ni, 1, nj) > 180) n(ni, 1, nj) = real(-360) + n(ni, 1, nj);
}

}  // namespace cppwrf::nest::move
