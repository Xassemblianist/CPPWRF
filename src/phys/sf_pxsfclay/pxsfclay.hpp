// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Pleim-Xiu surface layer (sf_sfclay_physics = 7): WRF v4.8.0 phys/module_sf_pxsfclay.F PXSFCLAY / PXSFCLAY1D for one
// point (WRF loops over the rows j and the points its..ite of a row; every point is independent; fractional_seaice = 0,
// PXSFCLAY_SEAICE_WRAPPER is not ported). Every statement follows the Fortran in order and parenthesisation (real*8).
// libm: gfortran vectorises two of PXSFCLAY1D's loops, which then call libmvec's 2-lane functions over its..ite
// (vm:: with the point's i of the nx-point row): CPOT = (100/PSFC)**ROVCP, and GZ1OZ0 = ALOG(ZA/ZNT) with
// VCONV = (G/TG*PBLH*FLUXC)**.33; VSGD = 0.32*MAX(DX/5000-1, 0)**.33 is loop-invariant (one scalar pow), and the
// exp / log / pow / atan of the other loops are scalar libm (x**0.5 and x**0.25 are pow calls in WRF's build).
// ISFFLX is passed but not read: the fluxes are always computed. DX is the namelist dx (a scalar in WRF's call).
#include <cmath>

#include "core/backend.hpp"
#include "core/field.hpp"

namespace cppwrf::phys::pxsfclay {

// module_sf_pxsfclay.F parameters
inline constexpr real ricrit = real(0.25);  // critical Richardson number
inline constexpr real betah = real(5.0);
inline constexpr real betam = real(5.0);
inline constexpr real bm = real(13.0);
inline constexpr real bh = real(15.7);
inline constexpr real gamam = real(19.3);
inline constexpr real gamah = real(11.6);
inline constexpr real pr0 = real(0.95);
inline constexpr real czo = real(0.032);
inline constexpr real ozo = real(1.E-4);
inline constexpr real vconvc = real(1.0);

struct PxSfclayKernel {
  // level kts of u_phytmp, v_phytmp, t_phy, th_phy, qv_curr, p_phy (grid%p_hyd), dz8w
  View2<const real> us, vs, t1d, theta1, qv1d, p1d, dz8w1d;
  View2<const real> psfcpa, pblh, mavail, xland, tg;  // TG = TSK
  View2<real> chs, chs2, cqs2, cpm, rmol, znt, ust, zol, mol, regime, psim, psih, hfx, qfx, u10, v10, flhc, flqc, qgh,
      qsfc, lh, gz1oz0, wspd, br;
  real cp, g, rovcp, r, xlv, dx, svp1, svp2, svp3, svpt0, ep1, ep2, karman;
  int itimestep;
  int nx;  // the row's points its..ite (the vectorised loops' lanes)

  CPPWRF_HD void operator()(int i, int j) const {
    using std::abs;
    using std::sqrt;
    const real us_ = us(i, j), vs_ = vs(i, j), t1 = t1d(i, j), th1 = theta1(i, j), qv1 = qv1d(i, j);
    const real ps = psfcpa(i, j), tgi = tg(i, j), xl = xland(i, j);
    // loop 1
    const real psfc = ps / real(1000.);
    real tvcon = real(1.0) + ep1 * qv1;
    const real thetav1 = th1 * tvcon;
    const real rhox = ps / (r * t1 * tvcon);
    // virtual potential temperature at the surface
    real e1;
    if (tgi < real(273.15)) {
      // saturation vapor pressure wrt ice (SVP1 = .6112; 10*mb)
      e1 = svp1 * libm::exp(real(4648) * (real(1.) / real(273.15) - real(1.) / tgi) -
                            real(11.64) * libm::log(real(273.15) / tgi) + real(0.02265) * (real(273.15) - tgi));
    } else {
      // saturation vapor pressure wrt water (Bolton 1980)
      e1 = svp1 * libm::exp(svp2 * (tgi - svpt0) / (tgi - svp3));
    }
    // water or the initial time step: the saturation mixing ratio for qsfc, otherwise the LSM's
    real qs = qsfc(i, j);
    if (xl > real(1.5) || qs <= real(0.0) || itimestep == 1) {
      qs = ep2 * e1 / (psfc - e1);
      qsfc(i, j) = qs;
    }
    // QGH from the lowest-level air temperature (Q2SAT = QGH in the LSM)
    e1 = svp1 * libm::exp(svp2 * (t1 - svpt0) / (t1 - svp3));
    const real pl = p1d(i, j) / real(1000.);
    qgh(i, j) = ep2 * e1 / (pl - e1);
    const real cpmv = cp * (real(1.) + real(0.8) * qv1);
    cpm(i, j) = cpmv;
    // thetav at the ground (vectorised: CPOT)
    const real tv0 = tgi * (real(1.0) + ep1 * qs);
    const real cpot = vm::pow(real(100.) / psfc, rovcp, i, nx);
    const real th0 = tv0 * cpot;
    const real thetag = cpot * tgi;
    // height of the lowest half level (DZ8W1D is dz between the full levels)
    const real za = real(0.5) * dz8w1d(i, j);
    const real ws = sqrt(us_ * us_ + vs_ * vs_);
    // bulk Richardson number of the surface layer, AKB(1976) eq. (12) (vectorised: GZ1OZ0, VCONV)
    const real ricriti = real(1.0) / ricrit;
    real znti = znt(i, j);
    real gz = vm::log(za / znti, i, nx);
    const real dthvdz = thetav1 - th0;
    const real fluxc = c_max(hfx(i, j) / rhox / cp + ep1 * th0 * qfx(i, j) / rhox, real(0.));
    const real vconv = vconvc * vm::pow(g / tgi * pblh(i, j) * fluxc, real(.33), i, nx);
    const real vsgd = real(0.32) * libm::pow(c_max(dx / real(5000.) - real(1.), real(0.)), real(.33));
    real wspdv = sqrt(ws * ws + vconv * vconv + vsgd * vsgd);
    wspdv = c_max(wspdv, real(0.1));
    wspd(i, j) = wspdv;
    const real govrth = g / th1;
    const real brv = govrth * za * dthvdz / (wspdv * wspdv);
    br(i, j) = brv;
    const real ricut = real(1.0) / (ricriti + gz);
    // regimes (the classes of HIRPBL changed)
    real zoll = real(0.0), psimv, psihv;
    if (brv >= ricut) {
      // class 1: very stable, z/L > 1
      regime(i, j) = real(1.0);
      zoll = brv * gz / (real(1.0) - ricriti * ricut);
      psimv = real(1.0) - betam - zoll;
      psihv = real(1.0) - betah - zoll;
    } else if (brv >= real(0.0)) {
      // class 2: stable, 1 > z/L > 0
      regime(i, j) = real(2.0);
      zoll = brv * gz / (real(1.0) - ricriti * brv);
      psimv = -betam * zoll;
      psihv = -betah * zoll;
    } else {
      // class 3 or 4: unstable (class 4 is for ACM non-local convection, H/L < -3)
      regime(i, j) = real(3.0);
      const real am = real(0.031) + real(0.276) * libm::log(gz);
      const real ah = real(0.04) + real(0.355) * libm::log(gz);
      const real sqlnzz0 = sqrt(gz);
      psimv = am * libm::log(real(1.0) - bm * sqlnzz0 * brv);
      psihv = ah * libm::log(real(1.0) - bh * sqlnzz0 * brv);
    }
    psim(i, j) = psimv;
    psih(i, j) = psihv;
    // frictional velocity and surface fluxes
    const real dtg = th1 - thetag;
    real psix = gz - psimv;
    real ustv = real(0.5) * ust(i, j) + real(0.5) * karman * wspdv / psix;
    real ustm = ustv;
    // over water: the roughness length from the wind (UST)
    if ((xl - real(1.5)) >= real(0.0)) {
      znti = czo * ustm * ustm / g + ozo;
      gz = libm::log(za / znti);
      psix = gz - psimv;
      ustv = karman * wspdv / psix;
      ustm = ustv;
    }
    znt(i, j) = znti;
    gz1oz0(i, j) = gz;
    ust(i, j) = ustv;
    const real ra = pr0 * (gz - psihv) / (karman * ustv);
    const real rbh = real(5.0) / ustv;
    const real rbw = real(4.503) / ustv;
    const real chsv = real(1.) / (ra + rbh);
    chs(i, j) = chsv;
    const real cqs = real(1.) / (ra + rbw);
    mol(i, j) = dtg * chsv / ustv;
    real tstv = (thetav1 - th0) * chsv / ustv;
    if (abs(tstv) < real(1.E-5)) tstv = real(1.E-5);
    const real molength = thetav1 * ustv * ustv / (karman * g * tstv);
    // 2 m surface exchange coefficients for heat and moisture
    real xmol = molength;
    if (molength > real(0.0)) xmol = c_max(molength, real(2.0));
    const real rmolv = real(1) / xmol;
    rmol(i, j) = rmolv;
    zol(i, j) = za * rmolv;
    const real zobol = real(1.5) * rmolv;
    const real z10ol = real(10.0) * rmolv;
    const real zntol = znti * rmolv;
    real psih2, psim10;
    if (xmol < real(0.0)) {
      const real ynt = libm::pow(real(1.0) - gamah * zntol, real(0.5));
      const real yob = libm::pow(real(1.0) - gamah * zobol, real(0.5));
      psih2 = real(2.) * libm::log((yob + real(1.0)) / (ynt + real(1.0)));
      const real x1 = libm::pow(real(1.0) - gamam * z10ol, real(0.25));
      const real x2 = libm::pow(real(1.0) - gamam * zntol, real(0.25));
      psim10 = real(2.0) * libm::log((real(1.0) + x1) / (real(1.0) + x2)) +
               libm::log((real(1.0) + x1 * x1) / (real(1.0) + x2 * x2)) - real(2.0) * libm::atan(x1) +
               real(2.0) * libm::atan(x2);
    } else {
      if ((zobol - zntol) <= real(1.0))
        psih2 = -betah * (zobol - zntol);
      else
        psih2 = real(1.) - betah - (zobol - zntol);
      if ((z10ol - zntol) <= real(1.0))
        psim10 = -betam * (z10ol - zntol);
      else
        psim10 = real(1.) - betam - (z10ol - zntol);
    }
    const real g2oz0 = libm::log(real(1.5) / znti);
    const real g10oz0 = libm::log(real(10.0) / znti);
    const real ra2 = pr0 * (g2oz0 - psih2) / (karman * ustv);
    chs2(i, j) = real(1.0) / (ra2 + rbh);
    cqs2(i, j) = real(1.0) / (ra2 + rbw);
    u10(i, j) = us_ * (g10oz0 - psim10) / psix;
    v10(i, j) = vs_ * (g10oz0 - psim10) / psix;
    // surface heat and moisture fluxes
    const real flhcv = cpmv * rhox * chsv;
    const real flqcv = rhox * cqs * mavail(i, j);
    flhc(i, j) = flhcv;
    flqc(i, j) = flqcv;
    real qfxv = flqcv * (qs - qv1);
    qfxv = c_max(qfxv, real(0.));
    qfx(i, j) = qfxv;
    lh(i, j) = xlv * qfxv;
    if (xl - real(1.5) > real(0.)) {
      hfx(i, j) = -flhcv * dtg;
    } else if (xl - real(1.5) < real(0.)) {
      real h = -flhcv * dtg;
      h = c_max(h, real(-250.));
      hfx(i, j) = h;
    }
  }
};

}  // namespace cppwrf::phys::pxsfclay
