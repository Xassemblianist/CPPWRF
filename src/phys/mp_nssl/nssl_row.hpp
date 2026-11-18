// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// nssl_2mom_driver (WRF 4.8.0 phys/module_mp_nssl_2mom.F) for one j row, as module_microphysics_driver.F calls it
// (CASE NSSL_2MOM): QH = qg (graupel), QHL = qh (hail), CCW = qndrop, CRW = qnr, CCI = qni, CSW = qns, CHW = qng,
// CHL = qnh, VHW = qvolg, VHL = qvolh, CN = qnn (f_cn = F_QNN), no cna / cni / cnuf / 3rd moments, dbz = REFL_10CM,
// re_cloud / re_ice / re_snow with has_reqc / has_reqi / has_reqs, cu_used and the decoupled cumulus tendencies,
// ssat / ssati with nssl_ssat_output. Not passed by WRF (so absent here): f_qi (flag_qi true), dx / dy, ntmul,
// vzf, compdbz, pcc2, axtra, rainncw2 / rainnci2, has_reqr / has_reqg / has_reqh, ipelectmp (ipelec 0), isedonly_in.
// The row's arrays are copies in the work space laid out as WRF's an(its:ite, 1, kts:kte, na), t0 ... (Fortran ix =
// i - its + 1, kz = k + 1).
#include "phys/mp_nssl/nssl_args.hpp"
#include "phys/mp_nssl/nssl_kernel.hpp"

namespace cppwrf::phys::mp_nssl {


template <class Real>
CPPWRF_HD inline void nssl_row(const NsslParams<Real>& P, Work W, const NsslRowArgs<Real>& A, int jy) {
  const int its = A.its, ite = A.ite;
  const int nx = ite - its + 1, ny = 1, nz = A.nz, na = P.na;
  const int ngs = 64;
  const Real dtp = A.dtp;
  const int itimestep = A.itimestep;
  const bool flag_ccn = A.flag_ccn;
  const bool flag_qi = true;
  const bool flag_qndrop = false;
  const int lastloop = 1;
  (void)lastloop;
  auto I = [&](int ix) { return its + ix - 1; };  // Fortran ix -> model i
  auto K = [&](int kz) { return kz - 1; };        // Fortran kz -> model k
  // dx1 = dy1 = 1 (dx / dy not passed)
  const Real dx1 = NR(1.0), dy1 = NR(1.0);
  const bool makediag = A.diagflag || itimestep == 1;
  const Real cnin20 = NR(1.0e3), cnin2a = NR(12.96), cnin2b = NR(0.639);
  // infdo: infall 4 (/= 1) -> 1; any linfall >= 3 or ipconc >= 6 -> 2
  int infdo = 1;
  {
    bool anyl = false;
    for (int l = P.lc; l <= P.lqmx; ++l) anyl = anyl || P.linfall(l) >= 3;
    if (anyl || P.ipconc >= 6) infdo = 2;
  }
  const int lnb = c_max(P.lh, P.lhl) + 1;
  // the row's arrays (an(its:ite,1,kts:kte,na) and the 3d temporaries of the driver)
  auto an = W.template arr<Real>(1, nx, 1, 1, 1, nz, 1, na);
  auto ancuten = W.template arr<Real>(1, nx, 1, 1, 1, nz, 1, na);
  auto t0 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t1 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t2 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t3 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t4 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t5 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t6 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t7 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t8 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t9 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto dn1 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t00 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto t77 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto ssat = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto pn = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto wn = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto dz2d = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto dz2dinv = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto dbz2d = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto vzf2d = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto tke2d = W.template arr<Real>(1, nx, 1, nz);
  auto rainprod2d = W.template arr<Real>(1, nx, 1, nz);
  auto evapprod2d = W.template arr<Real>(1, nx, 1, nz);
  auto elec2 = W.template arr<Real>(1, nx, 1, 1, 1, nz);
  auto axtra2d = W.template arr<Real>(1, nx, 1, 1, 1, nz, 1, P.nxtra);
  auto alpha2d = W.template arr<Real>(1, nx, 1, 1, 1, nz, 1, 3);
  auto xfall = W.template arr<Real>(1, nx, 1, 1, 1, na);
  auto thproclocal = W.template arr<Real>(1, nz, 1, 1);
  double timevtcalc = 0, timesetvt = 0, timesed1 = 0, timesed2 = 0, timesed3 = 0, zmaxsed = 0;

  // itimestep 1, flag_ccn, not invertccn: lccna > 1 (cna absent) -> cn = 0 (the driver's loop over all rows,
  // here row by row before the row is used)
  if (itimestep == 1 && !P.invertccn && flag_ccn) {
    if (P.lccna > 1) {
      for (int kz = 1; kz <= nz; ++kz)
        for (int ix = 1; ix <= nx; ++ix) A.cn(I(ix), jy, K(kz)) = NR(0.0);
    } else {
      // cn((ite+its)/2, (kte+kts)/2, (jte+jts)/2) < 10: the tile's middle point (not reached with lccna > 1)
      NSSL_STOP();
    }
  }
  if (P.lhl < 1) {
    for (int ix = 1; ix <= nx; ++ix) A.hailncv(I(ix), jy) = NR(0.0);
  }
  // an from the model fields
  for (int kz = 1; kz <= nz; ++kz) {
    for (int ix = 1; ix <= nx; ++ix) {
      const int i = I(ix), k = K(kz);
      an(ix, 1, kz, P.lt) = A.th(i, jy, k);
      an(ix, 1, kz, P.lv) = A.qv(i, jy, k);
      an(ix, 1, kz, P.lc) = A.qc(i, jy, k);
      an(ix, 1, kz, P.lr) = A.qr(i, jy, k);
      an(ix, 1, kz, P.li) = flag_qi ? A.qi(i, jy, k) : NR(0.0);
      an(ix, 1, kz, P.ls) = A.qs(i, jy, k);
      an(ix, 1, kz, P.lh) = A.qh(i, jy, k);
      if (P.lhl > 1) an(ix, 1, kz, P.lhl) = A.qhl(i, jy, k);
      if (P.lccn > 1) {
        if (flag_ccn) {
          if (P.lccna > 1) {
            an(ix, 1, kz, P.lccna) = A.cn(i, jy, k);
            an(ix, 1, kz, P.lccn) = P.qccn;
          } else {
            an(ix, 1, kz, P.lccn) = A.cn(i, jy, k);
          }
        } else {
          if (P.lccna == 0) {
            an(ix, 1, kz, P.lccn) = P.qccn - A.ccw(i, jy, k);
          } else {
            an(ix, 1, kz, P.lccn) = P.qccn;
          }
        }
      }
      if (P.ipconc >= 5) {
        an(ix, 1, kz, P.lnc) = A.ccw(i, jy, k);
        if (P.constccw > NR(0.0)) an(ix, 1, kz, P.lnc) = P.constccw;
        an(ix, 1, kz, P.lnr) = A.crw(i, jy, k);
        an(ix, 1, kz, P.lni) = A.cci(i, jy, k);
        an(ix, 1, kz, P.lns) = A.csw(i, jy, k);
        an(ix, 1, kz, P.lnh) = A.chw(i, jy, k);
        if (P.lhl > 1) an(ix, 1, kz, P.lnhl) = A.chl(i, jy, k);
      }
      if (P.lvh > 0) an(ix, 1, kz, P.lvh) = A.vhw(i, jy, k);
      if (P.lvhl > 0) an(ix, 1, kz, P.lvhl) = A.vhl(i, jy, k);
    }
  }
  for (int kz = 1; kz <= nz; ++kz) {
    for (int ix = 1; ix <= nx; ++ix) {
      const int i = I(ix), k = K(kz);
      t0(ix, 1, kz) = A.th(i, jy, k) * A.pii(i, jy, k);
      t00(ix, 1, kz) = NR(380.0) / A.p(i, jy, k);
      t77(ix, 1, kz) = A.pii(i, jy, k);
      dbz2d(ix, 1, kz) = NR(0.0);
      vzf2d(ix, 1, kz) = NR(0.0);
    }
  }
  for (int ix = 1; ix <= nx; ++ix) {
    A.rainncv(I(ix), jy) = NR(0.0);
    A.grplncv(I(ix), jy) = NR(0.0);
    A.hailncv(I(ix), jy) = NR(0.0);
    A.snowncv(I(ix), jy) = NR(0.0);
  }
  // loopmax = 1
  for (int kz = 1; kz <= nz; ++kz) {
    for (int ix = 1; ix <= nx; ++ix) {
      const int i = I(ix), k = K(kz);
      t1(ix, 1, kz) = NR(0.0); t2(ix, 1, kz) = NR(0.0); t3(ix, 1, kz) = NR(0.0);
      t4(ix, 1, kz) = NR(0.0); t5(ix, 1, kz) = NR(0.0); t6(ix, 1, kz) = NR(0.0);
      t7(ix, 1, kz) = NR(0.0); t8(ix, 1, kz) = NR(0.0); t9(ix, 1, kz) = NR(0.0);
      pn(ix, 1, kz) = A.p(i, jy, k);
      wn(ix, 1, kz) = A.w(i, jy, k);
      dn1(ix, 1, kz) = A.dn(i, jy, k);
      dz2d(ix, 1, kz) = A.dz(i, jy, k);
      dz2dinv(ix, 1, kz) = NR(1.0) / A.dz(i, jy, k);
      int ltemq = int((t0(ix, 1, kz) - NR(163.15)) / P.fqsat + NR(1.5));
      ltemq = c_min(P.nqsat, c_max(1, ltemq));
      Real t8s;
      if (P.iqvsopt == 0) {
        t8s = t00(ix, 1, kz) * P.tabqvs(ltemq);
      } else {
        t8s = P.rdorv * P.esbolton * P.tabqvs(ltemq) / (pn(ix, 1, kz) - P.esbolton * P.tabqvs(ltemq));
      }
      const Real t9s = t00(ix, 1, kz) * P.tabqis(ltemq);
      const Real ssival = c_min(t8s, c_max(an(ix, 1, kz, P.lv), NR(0.0))) / t9s;
      if (ssival > NR(1.0)) {
        // icenucopt 1, imeyers5 false
        if (t0(ix, 1, kz) <= NR(268.15)) {
          const double dp1 =
              dn1(ix, 1, kz) / P.rho00 * cnin20 * exp(c_min(NR(57.0), cnin2a * (ssival - NR(1.0)) - cnin2b));
          t7(ix, 1, kz) = c_min(dp1, 1.0e30);
        }
      }
    }
  }
  for (int il = lnb; il <= na; ++il) {
    if (P.denscale(il) == 1) {
      for (int kz = 1; kz <= nz; ++kz)
        for (int ix = 1; ix <= nx; ++ix) an(ix, 1, kz, il) = an(ix, 1, kz, il) * dn1(ix, 1, kz);
    }
  }
  // xfall = 0 (W.arr)
  if (itimestep == 1 && P.ipconc > 0) calcnfromq(P, W, nx, ny, nz, an.p, na, 0, 0, dn1.p);
  if (A.cu_used) {
    for (int kz = 1; kz <= nz; ++kz) {
      for (int ix = 1; ix <= nx; ++ix) {
        const int i = I(ix), k = K(kz);
        ancuten(ix, 1, kz, P.lr) = dtp * A.qrcuten(i, jy, k);
        ancuten(ix, 1, kz, P.ls) = dtp * A.qscuten(i, jy, k);
        ancuten(ix, 1, kz, P.li) = dtp * A.qicuten(i, jy, k);
        ancuten(ix, 1, kz, P.lc) = dtp * A.qccuten(i, jy, k);
      }
    }
    calcnfromcuten(P, W, nx, ny, nz, ancuten.p, an.p, na, 0, 0, dn1.p);
  }
  // isedonly_local = 0
  sediment1d(P, W, dtp, nx, ny, nz, an.p, na, 0, 0, xfall.p, dn1.p, dz2d.p, dz2dinv.p, t0.p, t7.p, infdo, jy, its, jy,
             timesed1, timesed2, timesed3, zmaxsed, timesetvt);
  for (int ix = 1; ix <= nx; ++ix) {
    const int i = I(ix);
    if (P.lhl > 1) {
      A.rainncv(i, jy) = A.rainncv(i, jy) +
                         dtp * dn1(ix, 1, 1) *
                             (xfall(ix, 1, P.lr) + xfall(ix, 1, P.ls) * NR(1000.0) / P.xdn0(P.lr) +
                              xfall(ix, 1, P.lh) * NR(1000.0) / P.xdn0(P.lr) + xfall(ix, 1, P.lhl) * NR(1000.0) / P.xdn0(P.lr));
    } else {
      A.rainncv(i, jy) = A.rainncv(i, jy) + dtp * dn1(ix, 1, 1) *
                                                (xfall(ix, 1, P.lr) + xfall(ix, 1, P.ls) * NR(1000.0) / P.xdn0(P.lr) +
                                                 xfall(ix, 1, P.lh) * NR(1000.0) / P.xdn0(P.lr));
    }
    A.snowncv(i, jy) = A.snowncv(i, jy) + dtp * dn1(ix, 1, 1) * xfall(ix, 1, P.ls) * NR(1000.0) / P.xdn0(P.lr);
    // HAILNC present: graupel only
    A.grplncv(i, jy) = A.grplncv(i, jy) + dtp * dn1(ix, 1, 1) * xfall(ix, 1, P.lh) * NR(1000.0) / P.xdn0(P.lr);
    A.rainnc(i, jy) = A.rainnc(i, jy) + A.rainncv(i, jy);
    A.snownc(i, jy) = A.snownc(i, jy) + A.snowncv(i, jy);
    if (P.lhl > 1) {
      A.hailncv(i, jy) = dtp * dn1(ix, 1, 1) * xfall(ix, 1, P.lhl) * NR(1000.0) / P.xdn0(P.lr);
      A.hailnc(i, jy) = A.hailnc(i, jy) + A.hailncv(i, jy);
    }
    A.grplnc(i, jy) = A.grplnc(i, jy) + A.grplncv(i, jy);
    A.sr(i, jy) = (A.snowncv(i, jy) + A.hailncv(i, jy) + A.grplncv(i, jy)) / (A.rainncv(i, jy) + NR(1.e-12));
  }
  // isedonly /= 1
  nssl_2mom_gs(P, W, nx, ny, nz, na, jy, 0, 0, dtp, dz2d.p, t0.p, t1.p, t2.p, t3.p, t4.p, t5.p, t6.p, t7.p, t8.p, t9.p,
               an.p, dn1.p, t77.p, pn.p, wn.p, 0, t00.p, t77.p, P.ventr, P.ventc, P.c1sw, 1, const_cast<int*>(P.ido.v),
               CP(P.xdnmx.v), CP(P.xdnmn.v), CP(P.cdx.v), CP(P.xdn0.v), dbz2d.p, tke2d.p, thproclocal.p, 1, dx1, dy1,
               ngs, timevtcalc, axtra2d.p, makediag, false, rainprod2d.p, evapprod2d.p, alpha2d.p, elec2.p, its, 1, 1, 1,
               1);
  nucond(P, W, nx, ny, nz, na, jy, 0, 0, dtp, nx, dz2d.p, t0.p, t9.p, an.p, dn1.p, t77.p, pn.p, wn.p, ngs, axtra2d.p,
         makediag, ssat.p, t00.p, t77.p, flag_qndrop);
  smallvalues(P, W, nx, ny, nz, na, jy, 0, 0, dtp, nx, t0.p, an.p, dn1.p, wn.p, t77.p, flag_qndrop);
  // ssat3d / ssati (nssl_ssat_output >= 1 / 2; WRF passes the arrays always, present or not as packages)
  if (A.nssl_ssat_output >= 1 && makediag) {
    for (int kz = 1; kz <= nz; ++kz) {
      for (int ix = 1; ix <= nx; ++ix) {
        const Real temp1 = t0(ix, 1, kz);
        int ltemq = int((temp1 - NR(163.15)) / P.fqsat + NR(1.5));
        ltemq = c_min(P.nqsat, c_max(1, ltemq));
        Real c1 = 0;
        if (P.iqvsopt == 0) {
          c1 = NR(380.0) / pn(ix, 1, kz) * P.tabqvs(ltemq);
        } else if (P.iqvsopt == 1) {
          c1 = P.rdorv * P.esbolton * P.tabqvs(ltemq) / (pn(ix, 1, kz) - P.esbolton * P.tabqvs(ltemq));
        }
        if (c1 > NR(0.0)) A.ssat3d(I(ix), jy, K(kz)) = NR(100.) * (an(ix, 1, kz, P.lv) / c1 - NR(1.0));
        if (A.nssl_ssat_output >= 2) {
          const Real t9s = NR(380.0) / pn(ix, 1, kz) * P.tabqis(ltemq);
          A.ssati(I(ix), jy, K(kz)) = NR(100.) * (an(ix, 1, kz, P.lv) / t9s - NR(1.0));
        }
      }
    }
  }
  if (A.do_dbz && makediag) {
    const int kediagloc = A.ke_diag;
    radardd02(P, W, nx, ny, nz, 0, na, an.p, t0.p, dbz2d.p, dn1.p, nz, P.cnoh, P.rho_qh, P.ipconc, kediagloc, 0);
    for (int kz = 1; kz <= kediagloc; ++kz)
      for (int ix = 1; ix <= nx; ++ix) A.dbz(I(ix), jy, K(kz)) = dbz2d(ix, 1, kz);
  }
  if (A.has_reqc != 0 || A.has_reqi != 0 || A.has_reqs != 0) {
    for (int kz = 1; kz <= nz; ++kz) {
      for (int ix = 1; ix <= nx; ++ix) {
        const int i = I(ix), k = K(kz);
        A.re_cloud(i, jy, k) = NR(2.51E-6);
        A.re_ice(i, jy, k) = NR(10.01E-6);
        A.re_snow(i, jy, k) = NR(25.E-6);
        t1(ix, 1, kz) = NR(2.51E-6);
        t2(ix, 1, kz) = NR(10.01E-6);
        t3(ix, 1, kz) = NR(25.E-6);
        t4(ix, 1, kz) = NR(50.e-6);
      }
    }
    calc_eff_radius(P, W, nx, ny, nz, na, jy, 0, 0, t1.p, t2.p, t3.p, t4.p, t5.p, t6.p, false, false, false, an.p,
                    dn1.p);
    for (int kz = 1; kz <= nz; ++kz) {
      for (int ix = 1; ix <= nx; ++ix) {
        const int i = I(ix), k = K(kz);
        A.re_cloud(i, jy, k) = c_max(NR(2.51E-6), c_min(t1(ix, 1, kz), NR(50.E-6)));
        A.re_ice(i, jy, k) = c_max(NR(10.01E-6), c_min(t2(ix, 1, kz), NR(125.E-6)));
        A.re_snow(i, jy, k) = c_max(NR(25.E-6), c_min(t3(ix, 1, kz), NR(999.E-6)));
      }
    }
  }
  // nwp_diagnostics (hailmaxd) is rejected with mp_physics 18
  for (int il = lnb; il <= na; ++il) {
    if (P.denscale(il) == 1) {
      for (int kz = 1; kz <= nz; ++kz)
        for (int ix = 1; ix <= nx; ++ix) an(ix, 1, kz, il) = an(ix, 1, kz, il) / dn1(ix, 1, kz);
    }
  }
  for (int kz = 1; kz <= nz; ++kz) {
    for (int ix = 1; ix <= nx; ++ix) {
      const int i = I(ix), k = K(kz);
      A.th(i, jy, k) = an(ix, 1, kz, P.lt);
      A.qv(i, jy, k) = an(ix, 1, kz, P.lv);
      A.qc(i, jy, k) = an(ix, 1, kz, P.lc);
      A.qr(i, jy, k) = an(ix, 1, kz, P.lr);
      A.qi(i, jy, k) = an(ix, 1, kz, P.li);
      A.qs(i, jy, k) = an(ix, 1, kz, P.ls);
      A.qh(i, jy, k) = an(ix, 1, kz, P.lh);
      if (P.lhl > 1) A.qhl(i, jy, k) = an(ix, 1, kz, P.lhl);
      if (flag_ccn && P.lccn > 1 && !flag_qndrop) {
        if (P.lccna > 1) {
          A.cn(i, jy, k) = c_max(NR(0.0), an(ix, 1, kz, P.lccna));
        } else {
          A.cn(i, jy, k) = an(ix, 1, kz, P.lccn);
        }
      }
      if (P.ipconc >= 5) {
        A.ccw(i, jy, k) = an(ix, 1, kz, P.lnc);
        A.crw(i, jy, k) = an(ix, 1, kz, P.lnr);
        A.cci(i, jy, k) = an(ix, 1, kz, P.lni);
        A.csw(i, jy, k) = an(ix, 1, kz, P.lns);
        A.chw(i, jy, k) = an(ix, 1, kz, P.lnh);
        if (P.lhl > 1) A.chl(i, jy, k) = an(ix, 1, kz, P.lnhl);
      }
      if (P.lvh > 0) A.vhw(i, jy, k) = an(ix, 1, kz, P.lvh);
      if (P.lvhl > 0) A.vhl(i, jy, k) = an(ix, 1, kz, P.lvhl);
    }
  }
}

}  // namespace cppwrf::phys::mp_nssl
#undef NR
#undef CP
