// SPDX-License-Identifier: GPL-3.0-or-later
// Grell-3D (cu_physics = 5) patch test, the C++ side: cu_g3::g3drv and cu_g3::g3_spread_column (src/phys/cu_g3/g3.hpp,
// the functions the model's kernels call) on the patch of driver_g3.f90, its output in the same layout.
//   driver_g3_cpu IN OUT
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include "core/precision.hpp"
#include "phys/cu_g3/g3.hpp"

using cppwrf::real;

using cppwrf::phys::cu_g3::ENSDIM;
using R = real;

namespace {
constexpr int H = 3;  // halo
int NX, NY, NZ;
// a field of nk levels with a halo of H points: (i, j, k), i, j from -H
struct F3 {
  int nk = 0;
  std::vector<R> v;
  void init(int k) {
    nk = k;
    v.assign(std::size_t(NX + 2 * H) * (NY + 2 * H) * k, R(0));
  }
  R& operator()(int i, int j, int k) const {
    return const_cast<R&>(v[(std::size_t(j + H) * nk + k) * (NX + 2 * H) + (i + H)]);
  }
};
struct F2 {
  std::vector<R> v;
  void init() { v.assign(std::size_t(NX + 2 * H) * (NY + 2 * H), R(0)); }
  R& operator()(int i, int j) const { return const_cast<R&>(v[std::size_t(j + H) * (NX + 2 * H) + (i + H)]); }
};
struct I2 {
  std::vector<int> v;
  void init() { v.assign(std::size_t(NX + 2 * H) * (NY + 2 * H), 0); }
  int operator()(int i, int j) const { return v[std::size_t(j + H) * (NX + 2 * H) + (i + H)]; }
};

std::FILE* fin;
void rd3(F3& a, int nk) {
  a.init(nk);
  std::vector<double> row(NX);
  for (int j = 0; j < NY; ++j)
    for (int k = 0; k < nk; ++k) {
      if (std::fread(row.data(), 8, NX, fin) != std::size_t(NX)) std::exit(2);
      for (int i = 0; i < NX; ++i) a(i, j, k) = R(row[i]);
    }
}
void rd2(F2& a) {
  a.init();
  std::vector<double> b(std::size_t(NX) * NY);
  if (std::fread(b.data(), 8, b.size(), fin) != b.size()) std::exit(2);
  for (int j = 0; j < NY; ++j)
    for (int i = 0; i < NX; ++i) a(i, j) = R(b[std::size_t(j) * NX + i]);
}
void halo3(F3& a) {
  for (int j = -H; j < NY + H; ++j)
    for (int k = 0; k < a.nk; ++k)
      for (int i = -H; i < NX + H; ++i) {
        const int ic = i < 0 ? 0 : (i >= NX ? NX - 1 : i), jc = j < 0 ? 0 : (j >= NY ? NY - 1 : j);
        if (ic != i || jc != j) a(i, j, k) = a(ic, jc, k);
      }
}

struct Fields {
  F3 u, v, t, q, p, pi, rho, rthften, rqvften, rthblten, rqvblten, w, p8w;
  F2 ht, xland, gsw;
  I2 kpbl;
};
struct Extra {
  F2 edt_out, xmb, k22, kbcon3, ktop3, ktopd, apr[8];
  std::vector<F2> xf, pr;
};
struct Outs {
  F3 cugd_tten, cugd_qvten, cugd_ttens, cugd_qvtens, cugd_qcten, rqccuten, rqicuten, gdc, gdc2, rthcuten, rqvcuten;
  F2 raincv, pratec, htop, hbot;
  F2 apr0[8];
  Extra* ex;
  R apr_in(int i, int j, int n) const { return apr0[n](i, j); }
  template <class S, class O, class W>
  void extra(int i, int j, const S& s, const O& o, W& w) const {
    ex->edt_out(i, j) = o.edt_out;
    ex->xmb(i, j) = o.xmb_shallow;
    ex->k22(i, j) = R(o.k22_shallow);
    ex->kbcon3(i, j) = R(o.kbcon_shallow);
    ex->ktop3(i, j) = R(o.ktop_shallow);
    ex->ktopd(i, j) = R(o.ktop_deep);
    for (int n = 0; n < 8; ++n) ex->apr[n](i, j) = s.apr[n];
    for (int n = 0; n < ENSDIM; ++n) {
      ex->xf[n](i, j) = w.xf_ens[n + 1];
      ex->pr[n](i, j) = w.pr_ens[n + 1];
    }
  }
};
struct Spread {
  const Outs* o;
  const F3* qv_;
  const F3* pi_;
  R tten(int i, int j, int k) const { return o->cugd_tten(i, j, k); }
  R qvten(int i, int j, int k) const { return o->cugd_qvten(i, j, k); }
  R ttens(int i, int j, int k) const { return o->cugd_ttens(i, j, k); }
  R qvtens(int i, int j, int k) const { return o->cugd_qvtens(i, j, k); }
  R qv(int i, int j, int k) const { return (*qv_)(i, j, k); }
  R pi(int i, int j, int k) const { return (*pi_)(i, j, k); }
};

std::FILE* fout;
void wr3(const F3& a) {
  for (int j = 0; j < NY; ++j)
    for (int k = 0; k < NZ; ++k)
      for (int i = 0; i < NX; ++i) {
        const double x = double(a(i, j, k));
        std::fwrite(&x, 8, 1, fout);
      }
}
void wr2(const F2& a) {
  for (int j = 0; j < NY; ++j)
    for (int i = 0; i < NX; ++i) {
      const double x = double(a(i, j));
      std::fwrite(&x, 8, 1, fout);
    }
}
}  // namespace

int main(int argc, char** argv) {
  namespace g3 = cppwrf::phys::cu_g3;
  if (argc != 3) return 2;
  fin = std::fopen(argv[1], "rb");
  if (!fin) return 2;
  int32_t hd[8];
  double sc[2];
  if (std::fread(hd, 4, 8, fin) != 8 || std::fread(sc, 8, 2, fin) != 2) return 2;
  NX = hd[0]; NY = hd[1]; NZ = hd[2];
  if (NZ > g3::kMaxLevels) return 3;
  g3::G3Call<R> c{};
  c.nx = NX; c.ny = NY; c.nz = NZ;
  c.dt = R(sc[0]); c.dx = R(sc[1]);
  // module_model_constants (as WRF's -r8 build evaluates them)
  c.xlv = R(2.5E6); c.cp = R(7.) * R(287.) / R(2.); c.g = R(9.81); c.r_v = R(461.6);
  c.cugd_avedx = hd[3]; c.ishallow = hd[4]; c.ichoice = hd[5];
  c.periodic_x = c.periodic_y = false;
  c.f_qc = hd[6] != 0; c.f_qi = hd[7] != 0;
  auto f = std::make_unique<Fields>();
  rd3(f->u, NZ); rd3(f->v, NZ); rd3(f->t, NZ); rd3(f->q, NZ); rd3(f->p, NZ); rd3(f->pi, NZ); rd3(f->rho, NZ);
  rd3(f->rthften, NZ); rd3(f->rqvften, NZ); rd3(f->rthblten, NZ); rd3(f->rqvblten, NZ);
  rd3(f->w, NZ + 1); rd3(f->p8w, NZ + 1);
  rd2(f->ht); rd2(f->xland); rd2(f->gsw);
  F2 kp;
  rd2(kp);
  f->kpbl.init();
  for (int j = 0; j < NY; ++j)
    for (int i = 0; i < NX; ++i) f->kpbl.v[std::size_t(j + H) * (NX + 2 * H) + (i + H)] = int(std::lround(kp(i, j)));
  auto o = std::make_unique<Outs>();
  auto ex = std::make_unique<Extra>();
  o->ex = ex.get();
  for (int n = 0; n < 8; ++n) {
    rd2(o->apr0[n]);
    ex->apr[n] = o->apr0[n];
  }
  std::fclose(fin);
  halo3(f->rthften); halo3(f->rqvften); halo3(f->w);
  for (F3* a : {&o->cugd_tten, &o->cugd_qvten, &o->cugd_ttens, &o->cugd_qvtens, &o->cugd_qcten, &o->rqccuten,
                &o->rqicuten, &o->gdc, &o->gdc2, &o->rthcuten, &o->rqvcuten})
    a->init(NZ);
  for (F2* a : {&o->raincv, &o->pratec, &o->htop, &o->hbot, &ex->edt_out, &ex->xmb, &ex->k22, &ex->kbcon3, &ex->ktop3,
                &ex->ktopd})
    a->init();
  ex->xf.resize(ENSDIM);
  ex->pr.resize(ENSDIM);
  for (int n = 0; n < ENSDIM; ++n) {
    ex->xf[n].init();
    ex->pr[n].init();
  }
  auto w = std::make_unique<g3::G3Work<R>>();
  for (int j = 0; j < NY; ++j)
    for (int i = 0; i < NX; ++i) g3::g3drv<R>(c, *f, *o, i, j, *w);
  fout = std::fopen(argv[2], "wb");
  for (const F3* a : {&o->cugd_tten, &o->cugd_qvten, &o->cugd_ttens, &o->cugd_qvtens, &o->cugd_qcten, &o->rqccuten,
                      &o->rqicuten, &o->gdc, &o->gdc2})
    wr3(*a);
  for (const F2* a : {&o->raincv, &o->pratec, &o->htop, &o->hbot, &ex->edt_out, &ex->xmb, &ex->k22, &ex->kbcon3,
                      &ex->ktop3, &ex->ktopd})
    wr2(*a);
  for (int n = 0; n < 8; ++n) wr2(ex->apr[n]);
  for (int n = 0; n < ENSDIM; ++n) wr2(ex->xf[n]);
  for (int n = 0; n < ENSDIM; ++n) wr2(ex->pr[n]);
  Spread sp{o.get(), &f->q, &f->pi};
  auto ws = std::make_unique<g3::G3SpreadWork<R>>();
  for (int j = 0; j < NY; ++j)
    for (int i = 0; i < NX; ++i) g3::g3_spread_column<R>(c, sp, *o, i, j, *ws);
  for (const F3* a : {&o->rthcuten, &o->rqvcuten, &o->rqccuten, &o->rqicuten}) wr3(*a);
  wr2(o->raincv);
  wr2(o->pratec);
  std::fclose(fout);
  return 0;
}
