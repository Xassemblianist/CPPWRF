// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Lateral halo fill (periodic or symmetric) for x and, if ny>1, y. Halo width 3.
// Symmetric (rigid wall, mirror at the boundary face, WRF symmetric_xs/xe):
//   normal velocity (U in x, V in y): odd about the face, whose value is left as is;  everything else: even.
// Fill order x then y, so corners are consistent.
#include <initializer_list>
#include <vector>

#include "core/field.hpp"

namespace cppwrf {

// OddFace: odd about the face, whose value changes sign too (WRF set_physical_bc2d 'u' in x: dat(ids-i) =
// -dat(ids+i) from i = 0, i.e. muu at the walls; the 3-D 'u' and the 'v' loops start at i = 1)
enum class Parity { Even, Odd, OddFace };

void fill_halo(Field3& f, const Grid& g, Parity px, Parity py);
void fill_halo(Field2& f, const Grid& g, Parity px, Parity py);
// Default parity from staggering: U odd in x, V odd in y, else even.
void fill_halo(Field3& f, const Grid& g);
// 2D fields (mu, muu, ...) are always even.
void fill_halo(Field2& f, const Grid& g);
// Default-parity fill of several fields with one launch per direction (same result as one by one).
void fill_halos(std::initializer_list<Field3*> fs, const Grid& g);
void fill_halos(std::initializer_list<Field2*> fs, const Grid& g);
void fill_halos(const std::vector<Field3*>& fs, const Grid& g);
// One field of a batched fill: 3-D or 2-D, the parities of fill_halo(f, g) or the given ones.
struct HaloField {
  real* p;
  Layout L;
  Stagger st;
  Parity px, py;
  HaloField(Field3& f, Parity x, Parity y) : p(f.data()), L(f.layout()), st(f.stagger()), px(x), py(y) {}
  HaloField(Field2& f, Parity x, Parity y) : p(f.data()), L(f.layout()), st(f.stagger()), px(x), py(y) {}
  HaloField(Field3& f)  // NOLINT: implicit, as the field itself in a list
      : HaloField(f, f.stagger() == Stagger::U ? Parity::Odd : Parity::Even,
                  f.stagger() == Stagger::V ? Parity::Odd : Parity::Even) {}
  HaloField(Field2& f) : HaloField(f, Parity::Even, Parity::Even) {}  // NOLINT
  // a base-state field (both parities even: WRF set_physical_bc2d / 3d types 'r', 't', 'x', 'y', 'W')
  template <int R>
  explicit HaloField(BaseFieldT<R>& f)
      : p(f.data()), L(f.layout()), st(f.stagger()), px(Parity::Even), py(Parity::Even) {}
};
// Several fields of either rank and any parities, one launch per direction for up to 8 fields: the same values
// as fill_halo of each field in turn (each field's x pass before its y pass; the fields must be distinct).
void fill_halos(std::initializer_list<HaloField> fs, const Grid& g);
void fill_halos(const std::vector<HaloField>& fs, const Grid& g);

// A fill whose transfer overlaps launches that do not need it (a decomposed GPU run, docs/MPI.md section 3.4):
//   FillAsync f;  f.begin({...}, g);       // the exchange packed and its copy queued; nothing waits
//   launch_interior(g, w, [&] { ... });   // the patch's interior points: run during the transfer
//   f.finish();                            // the transfer, the unpack, the boundary fill (as fill_halos)
//   launch_rest(g, w, [&] { ... });        // the other points, after the fill
// The kernels launched between begin and finish must not read the fields' halo points nor write any point of the
// fields; the same launch given to launch_interior and launch_rest computes each point once, with the operations and
// inputs of the one launch over all points, when no point of the interior box (the patch's points at least w from
// its edges) reads a halo point of the fields within the launch's stencil. Without a decomposition, with
// CPPWRF_TILE_CHECK, or with CPPWRF_HALO_OVERLAP=0: begin does nothing, finish is fill_halos, launch_interior
// launches nothing and launch_rest launches all points at once (the launches of a run without it). Other collectives
// (fills, exchanges, fatal checks) must not be called between begin and finish.
bool halo_overlap_on();
class FillAsync {
 public:
  FillAsync() = default;
  ~FillAsync();
  FillAsync(const FillAsync&) = delete;
  FillAsync& operator=(const FillAsync&) = delete;
  void begin(std::vector<HaloField> fs, const Grid& g);
  void finish();
  bool open() const { return open_; }

 private:
  std::vector<HaloField> fs_;
  const Grid* g_ = nullptr;
  bool open_ = false, exchanged_ = false;
};
// the interior box of the calling worker's patch shrunk by w on every side (empty: no split)
struct InteriorBox {
  bool on;
  int i0, i1, j0, j1;
};
InteriorBox interior_box(const Grid& g, int w);
template <class F>
void launch_interior(const Grid& g, int w, F&& launch) {
  const InteriorBox b = interior_box(g, w);
  if (!b.on) return;
  const tile::Region r(b.i0, b.i1, b.j0, b.j1);
  launch();
}
template <class F>
void launch_rest(const Grid& g, int w, F&& launch) {
  const InteriorBox b = interior_box(g, w);
  if (!b.on) {
    launch();
    return;
  }
  constexpr int lo = -(1 << 30), hi = 1 << 30;  // (the worker's own bounds clip these)
  {
    const tile::Region r(lo, hi, lo, b.j0);  // south strip, every column
    launch();
  }
  {
    const tile::Region r(lo, hi, b.j1, hi);  // north strip
    launch();
  }
  {
    const tile::Region r(lo, b.i0, b.j0, b.j1);  // west strip, the rows between
    launch();
  }
  {
    const tile::Region r(b.i1, hi, b.j0, b.j1);  // east strip
    launch();
  }
}

}  // namespace cppwrf
