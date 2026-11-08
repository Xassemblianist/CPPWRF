// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// WRF v4.8.0 phys/module_mp_morr_two_moment.F: the Morrison two-moment microphysics (mp_physics = 10) as WRF runs it
// without WRF-Chem: MORR_TWO_MOMENT_INIT (morr_rimed_ice), MP_MORR_TWO_MOMENT (the column driver) and
// MORR_TWO_MOMENT_MICRO (one column), with POLYSVP and GAMMA. Mixing ratios qv qc qr qi qs qg and number
// concentrations ni ns nr ng are predicted; the droplet number is the constant NDCNST = 250 cm-3 (iinum = 1: no
// QNDROP without WRF-Chem / progn); the compile-time switches are WRF's: IACT 2 (unused with iinum 1), ILIQ 0, INUC
// 0, IGRAUP 0, IHAIL = morr_rimed_ice. Kain-Fritsch's decoupled tendencies RQRCUTEN, RQSCUTEN, RQICUTEN add rain,
// snow and ice number (the driver passes them as qrcuten, qscuten, qicuten). Not ported: refl10cm_hm (do_radar_ref =
// 1, REAL*16 DOUBLE PRECISION in a plain -r8 build), the effective radii (EFFC ... EFFG are not coupled to the
// radiation in WRF: has_reqc = 0 with mp_physics = 10), the WRF-Chem outputs (QLSINK, PRECR ...).
//
// Arithmetic as gfortran -O2 compiles the real*8 build (module_mp_morr_two_moment.o calls exp, log, log10, pow and
// no libmvec function: every loop is scalar): x**2 = x*x, x**3 = (x*x)*x, x**4 = (x*x)*(x*x), a REAL exponent a
// pow call (x**0.5, x**1.5, ... too), Fortran's left-to-right order. MORR_TWO_MOMENT_INIT calls neither exp nor log
// nor pow: gfortran folds those of constants correctly rounded ((25.E-6)**3, PI**((1.-BS)/3.), ...; the values below).
// C++ k = Fortran k - 1 (kts = 1).
#include <cmath>
#include <cstddef>

#include "core/backend.hpp"
#include "core/column_work.hpp"
#include "core/field.hpp"
#include "core/libm.hpp"
#include "phys/phys_constants.hpp"

namespace cppwrf::phys::mp_morrison {

constexpr int kMaxLevels = 128;

// FUNCTION GAMMA (W. J. Cody's algorithm), as in the module
template <typename Real>
CPPWRF_HD inline Real gamma_fn(Real x) {
  constexpr double PI = 3.1415926535897932384626434;
  constexpr double xxx = 0.9189385332046727417803297;
  constexpr double P[8] = {-1.71618513886549492533811E+0, 2.47656508055759199108314E+1,
                           -3.79804256470945635097577E+2, 6.29331155312818442661052E+2,
                           8.66966202790413211295064E+2,  -3.14512729688483675254357E+4,
                           -3.61444134186911729807069E+4, 6.64561438202405440627855E+4};
  constexpr double Q[8] = {-3.08402300119738975254353E+1, 3.15350626979604161529144E+2,
                           -1.01515636749021914166146E+3, -3.10777167157231109440444E+3,
                           2.25381184209801510330112E+4,  4.75584627752788110767815E+3,
                           -1.34659959864969306392456E+5, -1.15132259675553483497211E+5};
  constexpr double C[7] = {-1.910444077728E-03,         8.4171387781295E-04,
                           -5.952379913043012E-04,      7.93650793500350248E-04,
                           -2.777777777777681622553E-03, 8.333333333333333331554247E-02,
                           5.7083835261E-03};
  const Real one = Real(1.0), half = Real(0.5), twelve = Real(12.0), two = Real(2.0), zero = Real(0.0);
  const Real xbig = Real(35.040), xminin = Real(1.18E-38), eps = Real(1.19E-7), xinf = Real(3.4E38);
  bool parity = false;
  Real fact = one, res, y = x, y1, z;
  int n = 0;
  if (y <= zero) {  // negative argument (not reached by the scheme)
    y = -x;
    y1 = Real((long long)(y));  // AINT
    res = y - y1;
    if (res != zero) {
      if (y1 != Real((long long)(y1 * half)) * two) parity = true;
      fact = -Real(PI) / libm::sin(Real(PI) * res);
      y = y + one;
    } else {
      return xinf;
    }
  }
  if (y < eps) {
    if (y >= xminin) res = one / y;
    else return xinf;
  } else if (y < twelve) {
    y1 = y;
    if (y < one) {
      z = y;
      y = y + one;
    } else {
      n = int(y) - 1;
      y = y - Real(n);
      z = y - one;
    }
    Real xnum = zero, xden = one;
    for (int i = 0; i < 8; ++i) {
      xnum = (xnum + Real(P[i])) * z;
      xden = xden * z + Real(Q[i]);
    }
    res = xnum / xden + one;
    if (y1 < y) {
      res = res / y1;
    } else if (y1 > y) {
      for (int i = 1; i <= n; ++i) {
        res = res * y;
        y = y + one;
      }
    }
  } else {
    if (y <= xbig) {
      const Real ysq = y * y;
      Real sum = Real(C[6]);
      for (int i = 0; i < 6; ++i) sum = sum / ysq + Real(C[i]);
      sum = sum / y - y + Real(xxx);
      sum = sum + (y - half) * libm::log(y);
      res = libm::exp(sum);
    } else {
      return xinf;
    }
  }
  if (parity) res = -res;
  if (fact != one) res = fact / res;
  return res;
}

// FUNCTION POLYSVP: saturation vapour pressure [Pa] over liquid (type 0) or ice (1): Flatau et al. (1992) at and
// above 202.0 K / 195.8 K, Goff-Gratch below (ALOG10(6.1071), ALOG10(1013.246): constants gfortran folds)
template <typename Real>
CPPWRF_HD inline Real polysvp(Real t, int type) {
  Real p = 0;
  if (type == 1) {
    if (t >= Real(195.8)) {
      const Real dt = t - Real(273.15);
      p = Real(6.11147274) +
          dt * (Real(0.503160820) +
                dt * (Real(0.188439774e-1) +
                      dt * (Real(0.420895665e-3) +
                            dt * (Real(0.615021634e-5) +
                                  dt * (Real(0.602588177e-7) +
                                        dt * (Real(0.385852041e-9) +
                                              dt * (Real(0.146898966e-11) + Real(0.252751365e-14) * dt)))))));
      p = p * Real(100.);
    } else {
      p = libm::pow(Real(10.), Real(-9.09718) * (Real(273.16) / t - Real(1.)) -
                                   Real(3.56654) * libm::log10(Real(273.16) / t) +
                                   Real(0.876793) * (Real(1.) - t / Real(273.16)) + Real(0x1.9258f81f79246p-1)) *
          Real(100.);
    }
  }
  if (type == 0) {
    if (t >= Real(202.0)) {
      const Real dt = t - Real(273.15);
      p = Real(6.11239921) +
          dt * (Real(0.443987641) +
                dt * (Real(0.142986287e-1) +
                      dt * (Real(0.264847430e-3) +
                            dt * (Real(0.302950461e-5) +
                                  dt * (Real(0.206739458e-7) +
                                        dt * (Real(0.640689451e-10) +
                                              dt * (Real(-0.952447341e-13) + Real(-0.976195544e-15) * dt)))))));
      p = p * Real(100.);
    } else {
      p = libm::pow(Real(10.),
                    Real(-7.90298) * (Real(373.16) / t - Real(1.)) + Real(5.02808) * libm::log10(Real(373.16) / t) -
                        Real(1.3816e-7) * (libm::pow(Real(10.), Real(11.344) * (Real(1.) - t / Real(373.16))) - Real(1.)) +
                        Real(8.1328e-3) * (libm::pow(Real(10.), Real(-3.49149) * (Real(373.16) / t - Real(1.))) - Real(1.)) +
                        Real(0x1.80bb4409e541bp+1)) *
          Real(100.);
    }
  }
  return p;
}

// MORR_TWO_MOMENT_INIT's constants (the module variables the micro routine reads)
template <typename Real>
struct MorrisonParams {
  Real ai, ac, as, ar, ag, bi, bc, bs, br, bg;
  Real rhosu, rhow, rhoi, rhosn, rhog, aimm, bimm, ecr, dcs, mi0, mg0, f1s, f2s, f1r, f2r, qsmall, eii, eci, rin,
      cpw, ci, di, cs, ds, cg, dg, mmult, ndcnst;
  Real lammaxi, lammini, lammaxr, lamminr, lammaxs, lammins, lammaxg, lamming;
  Real cons1, cons2, cons3, cons4, cons5, cons6, cons7, cons8, cons9, cons10, cons11, cons12, cons13, cons14, cons15,
      cons16, cons17, cons18, cons19, cons20, cons21, cons22, cons23, cons24, cons25, cons26, cons27, cons28, cons29,
      cons30, cons31, cons32, cons33, cons34, cons35, cons36, cons37, cons38, cons39, cons40, cons41;
};

// MORR_TWO_MOMENT_INIT(morr_rimed_ice): IHAIL = 1 (hail) if morr_rimed_ice = 1, else 0 (graupel)
template <typename Real>
MorrisonParams<Real> morrison_init(int morr_rimed_ice) {
  const bool hail = morr_rimed_ice == 1;
  const Real PI = Real(3.1415926535897932384626434);
  MorrisonParams<Real> c{};
  c.ndcnst = Real(250.);
  c.ai = Real(700.);
  c.ac = Real(3.E7);
  c.as = Real(11.72);
  c.ar = Real(841.99667);
  c.bi = Real(1.);
  c.bc = Real(2.);
  c.bs = Real(0.41);
  c.br = Real(0.8);
  c.ag = hail ? Real(114.5) : Real(19.3);
  c.bg = hail ? Real(0.5) : Real(0.37);
  c.rhosu = Real(85000.) / (Real(287.15) * Real(273.15));
  c.rhow = Real(997.);
  c.rhoi = Real(500.);
  c.rhosn = Real(100.);
  c.rhog = hail ? Real(900.) : Real(400.);
  c.aimm = Real(0.66);
  c.bimm = Real(100.);
  c.ecr = Real(1.);
  c.dcs = Real(125.E-6);
  // (10.E-6)**3, (5.E-6)**3, (25.E-6)**3 of literals: gfortran's correctly rounded powers
  c.mi0 = Real(4.) / Real(3.) * PI * c.rhoi * Real(0x1.203af9ee75617p-50);
  c.mg0 = Real(1.6E-10);
  c.f1s = Real(0.86);
  c.f2s = Real(0.28);
  c.f1r = Real(0.78);
  c.f2r = Real(0.308);
  c.qsmall = Real(1.E-14);
  c.eii = Real(0.1);
  c.eci = Real(0.7);
  c.cpw = Real(4187.);
  c.ci = c.rhoi * PI / Real(6.);
  c.di = Real(3.);
  c.cs = c.rhosn * PI / Real(6.);
  c.ds = Real(3.);
  c.cg = c.rhog * PI / Real(6.);
  c.dg = Real(3.);
  c.rin = Real(0.1E-6);
  c.mmult = Real(4.) / Real(3.) * PI * c.rhoi * Real(0x1.203af9ee75617p-53);
  c.lammaxi = Real(1.) / Real(1.E-6);
  c.lammini = Real(1.) / (Real(2.) * c.dcs + Real(100.E-6));
  c.lammaxr = Real(1.) / Real(20.E-6);
  c.lamminr = Real(1.) / Real(2800.E-6);
  c.lammaxs = Real(1.) / Real(10.E-6);
  c.lammins = Real(1.) / Real(2000.E-6);
  c.lammaxg = Real(1.) / Real(20.E-6);
  c.lamming = Real(1.) / Real(2000.E-6);
  auto G = [](Real x) { return gamma_fn<Real>(x); };
  c.cons1 = G(Real(1.) + c.ds) * c.cs;
  c.cons2 = G(Real(1.) + c.dg) * c.cg;
  c.cons3 = G(Real(4.) + c.bs) / Real(6.);
  c.cons4 = G(Real(4.) + c.br) / Real(6.);
  c.cons5 = G(Real(1.) + c.bs);
  c.cons6 = G(Real(1.) + c.br);
  c.cons7 = G(Real(4.) + c.bg) / Real(6.);
  c.cons8 = G(Real(1.) + c.bg);
  c.cons9 = G(Real(5.) / Real(2.) + c.br / Real(2.));
  c.cons10 = G(Real(5.) / Real(2.) + c.bs / Real(2.));
  c.cons11 = G(Real(5.) / Real(2.) + c.bg / Real(2.));
  c.cons12 = G(Real(1.) + c.di) * c.ci;
  c.cons13 = G(c.bs + Real(3.)) * PI / Real(4.) * c.eci;
  c.cons14 = G(c.bg + Real(3.)) * PI / Real(4.) * c.eci;
  // PI**((1.-BS)/3.) and RHOSN**((-2.-BS)/3.) with BS = 0.41, RHOSN = 100: folded, correctly rounded
  c.cons15 = Real(-1108.) * c.eii * Real(0x1.40a2e8af05d65p+0) * Real(0x1.954739e4529e6p-6) / (Real(4.) * Real(720.));
  c.cons16 = G(c.bi + Real(3.)) * PI / Real(4.) * c.eci;
  c.cons17 = Real(4.) * Real(2.) * Real(3.) * c.rhosu * PI * c.eci * c.eci * G(Real(2.) * c.bs + Real(2.)) /
             (Real(8.) * (c.rhog - c.rhosn));
  c.cons18 = c.rhosn * c.rhosn;
  c.cons19 = c.rhow * c.rhow;
  c.cons20 = Real(20.) * PI * PI * c.rhow * c.bimm;
  c.cons21 = Real(4.) / (c.dcs * c.rhoi);
  c.cons22 = PI * c.rhoi * (c.dcs * c.dcs * c.dcs) / Real(6.);
  c.cons23 = PI / Real(4.) * c.eii * G(c.bs + Real(3.));
  c.cons24 = PI / Real(4.) * c.ecr * G(c.br + Real(3.));
  c.cons25 = PI * PI / Real(24.) * c.rhow * c.ecr * G(c.br + Real(6.));
  c.cons26 = PI / Real(6.) * c.rhow;
  c.cons27 = G(Real(1.) + c.bi);
  c.cons28 = G(Real(4.) + c.bi) / Real(6.);
  c.cons29 = Real(4.) / Real(3.) * PI * c.rhow * Real(0x1.19799812dea12p-46);
  c.cons30 = Real(4.) / Real(3.) * PI * c.rhow;
  c.cons31 = PI * PI * c.ecr * c.rhosn;
  c.cons32 = PI / Real(2.) * c.ecr;
  c.cons33 = PI * PI * c.ecr * c.rhog;
  c.cons34 = Real(5.) / Real(2.) + c.br / Real(2.);
  c.cons35 = Real(5.) / Real(2.) + c.bs / Real(2.);
  c.cons36 = Real(5.) / Real(2.) + c.bg / Real(2.);
  c.cons37 = Real(4.) * PI * Real(1.38E-23) / (Real(6.) * PI * c.rin);
  c.cons38 = PI * PI / Real(3.) * c.rhow;
  c.cons39 = PI * PI / Real(36.) * c.rhow * c.bimm;
  c.cons40 = PI / Real(6.) * c.bimm;
  c.cons41 = PI * PI * c.ecr * c.rhow;
  return c;
}

// MORR_TWO_MOMENT_MICRO's column arrays that live across its level loops (the process rates are per level: scalars)
#define CPPWRF_MORRISON_COLUMN(X)                                                                                    \
  X(Real, t3d, NL) X(Real, nc3d, NL) X(Real, rho, NL) X(Real, xxlv, NL) X(Real, xxls, NL) X(Real, cpm, NL)          \
  X(Real, xlf, NL) X(Real, lami, NL) X(Real, ain, NL) X(Real, arn, NL) X(Real, asn, NL) X(Real, acn, NL)            \
  X(Real, agn, NL) X(Real, qc3dten, NL) X(Real, qi3dten, NL) X(Real, qni3dten, NL) X(Real, qr3dten, NL)             \
  X(Real, ni3dten, NL) X(Real, ns3dten, NL) X(Real, nr3dten, NL) X(Real, t3dten, NL) X(Real, qv3dten, NL)          \
  X(Real, qg3dten, NL) X(Real, ng3dten, NL) X(Real, nc3dten, NL) X(Real, qgsten, NL) X(Real, qrsten, NL)            \
  X(Real, qisten, NL) X(Real, qnisten, NL) X(Real, qcsten, NL) X(Real, dumi, NL) X(Real, dumr, NL)                  \
  X(Real, dumfni, NL) X(Real, dumg, NL) X(Real, dumfng, NL) X(Real, dumqs, NL) X(Real, dumfns, NL)                  \
  X(Real, dumc, NL) X(Real, dumfnc, NL) X(Real, dumfnr, NL) X(Real, fr, NL) X(Real, fi, NL) X(Real, fni, NL)        \
  X(Real, fg, NL) X(Real, fng, NL) X(Real, fs, NL) X(Real, fns, NL) X(Real, fc, NL) X(Real, fnc, NL)                \
  X(Real, fnr, NL) X(Real, faloutr, NL) X(Real, falouti, NL) X(Real, faloutni, NL) X(Real, falouts, NL)            \
  X(Real, faloutns, NL) X(Real, faloutnr, NL) X(Real, faloutc, NL) X(Real, faloutnc, NL) X(Real, faloutg, NL)      \
  X(Real, faloutng, NL)

// MP_MORR_TWO_MOMENT for one column (i, j): TH, the mixing ratios and numbers in place, the precipitation
// accumulations and SR; NL >= nz sizes the column arrays (local arrays on the CPU, a shared device work buffer on
// the GPU: core/column_work.hpp)
template <typename Real, int NL = kMaxLevels>
struct MorrisonKernel {
  MorrisonParams<Real> c;
  Real dt;
  int nz;
  View3<Real> th, qv, qc, qr, qi, qs, qg, ni, ns, nr, ng;
  View3<const Real> pii, p, dz, qrcuten, qscuten, qicuten;  // p: full pressure (p_phy); cumulus: decoupled
  bool has_cu = false;  // the cumulus tendencies (zero arrays in WRF without cumulus)
  View2<Real> rainnc, rainncv, snownc, snowncv, graupelnc, graupelncv, sr;
  ColumnWork work{};
  static constexpr std::size_t kWorkBytes = 0 CPPWRF_MORRISON_COLUMN(CPPWRF_COLUMN_BYTES);

  CPPWRF_HD void operator()(int i, int j) const {
    if (nz > NL) return;
    CPPWRF_COLUMN_SLOTS(work, i, j)
    CPPWRF_MORRISON_COLUMN(CPPWRF_COLUMN_ARRAY)
    namespace pc = phys::cst;
    const Real CP = Real(pc::cp), G = Real(pc::g), R = Real(pc::r_d), RV = Real(pc::r_v), EP_2 = Real(pc::ep_2);
    const Real PI = Real(3.1415926535897932384626434);
    const Real QSMALL = c.qsmall;
    const Real DT = dt;
    // the driver: T = TH*PII, zero tendencies, nc1d = 0 (iinum = 1)
    for (int k = 0; k < nz; ++k) {
      t3d[k] = th(i, j, k) * pii(i, j, k);
      qc3dten[k] = 0; qi3dten[k] = 0; qni3dten[k] = 0; qr3dten[k] = 0; ni3dten[k] = 0; ns3dten[k] = 0;
      nr3dten[k] = 0; t3dten[k] = 0; qv3dten[k] = 0; nc3dten[k] = 0; qg3dten[k] = 0; ng3dten[k] = 0;
      nc3d[k] = 0;
    }
    int ltrue = 0;
    for (int k = 0; k < nz; ++k) {
      Real& QV3D = qv(i, j, k);
      Real& QC3D = qc(i, j, k);
      Real& QR3D = qr(i, j, k);
      Real& QI3D = qi(i, j, k);
      Real& QNI3D = qs(i, j, k);
      Real& QG3D = qg(i, j, k);
      Real& NI3D = ni(i, j, k);
      Real& NS3D = ns(i, j, k);
      Real& NR3D = nr(i, j, k);
      Real& NG3D = ng(i, j, k);
      Real& NC3D = nc3d[k];
      Real& T3D = t3d[k];
      const Real PRES = p(i, j, k);
      Real DUM, DUM1, RATIO;
      nc3dten[k] = 0;
      xxlv[k] = Real(3.1484E6) - Real(2370.) * T3D;
      xxls[k] = Real(3.15E6) - Real(2370.) * T3D + Real(0.3337E6);
      cpm[k] = CP * (Real(1.) + Real(0.887) * QV3D);
      Real EVS = c_min(Real(0.99) * PRES, polysvp(T3D, 0));
      Real EIS = c_min(Real(0.99) * PRES, polysvp(T3D, 1));
      if (EIS > EVS) EIS = EVS;
      const Real QVS = EP_2 * EVS / (PRES - EVS);
      const Real QVI = EP_2 * EIS / (PRES - EIS);
      const Real QVQVS = QV3D / QVS;
      const Real QVQVSI = QV3D / QVI;
      rho[k] = PRES / (R * T3D);
      const Real RHO = rho[k];
      // number from the cumulus tendencies: N0 1e7 m-4 for rain, 2e7 m-4 for snow, 80 micron detrained ice
      if (has_cu) {
        if (qrcuten(i, j, k) >= Real(1.E-10)) {
          DUM = Real(1.8e5) *
                libm::pow(qrcuten(i, j, k) * DT / (PI * c.rhow * (RHO * RHO * RHO)), Real(0.25));
          NR3D = NR3D + DUM;
        }
        if (qscuten(i, j, k) >= Real(1.E-10)) {
          DUM = Real(3.e5) *
                libm::pow(qscuten(i, j, k) * DT / (c.cons1 * (RHO * RHO * RHO)), Real(1.) / (c.ds + Real(1.)));
          NS3D = NS3D + DUM;
        }
        if (qicuten(i, j, k) >= Real(1.E-10)) {
          DUM = qicuten(i, j, k) * DT / (c.ci * libm::pow(Real(80.E-6), c.di));
          NI3D = NI3D + DUM;
        }
      }
      // at subsaturation, small amounts of cloud and precipitation evaporate
      if (QVQVS < Real(0.9)) {
        if (QR3D < Real(1.E-8)) {
          QV3D = QV3D + QR3D;
          T3D = T3D - QR3D * xxlv[k] / cpm[k];
          QR3D = 0;
        }
        if (QC3D < Real(1.E-8)) {
          QV3D = QV3D + QC3D;
          T3D = T3D - QC3D * xxlv[k] / cpm[k];
          QC3D = 0;
        }
      }
      if (QVQVSI < Real(0.9)) {
        if (QI3D < Real(1.E-8)) {
          QV3D = QV3D + QI3D;
          T3D = T3D - QI3D * xxls[k] / cpm[k];
          QI3D = 0;
        }
        if (QNI3D < Real(1.E-8)) {
          QV3D = QV3D + QNI3D;
          T3D = T3D - QNI3D * xxls[k] / cpm[k];
          QNI3D = 0;
        }
        if (QG3D < Real(1.E-8)) {
          QV3D = QV3D + QG3D;
          T3D = T3D - QG3D * xxls[k] / cpm[k];
          QG3D = 0;
        }
      }
      xlf[k] = xxls[k] - xxlv[k];
      if (QC3D < QSMALL) { QC3D = 0; NC3D = 0; }
      if (QR3D < QSMALL) { QR3D = 0; NR3D = 0; }
      if (QI3D < QSMALL) { QI3D = 0; NI3D = 0; }
      if (QNI3D < QSMALL) { QNI3D = 0; NS3D = 0; }
      if (QG3D < QSMALL) { QG3D = 0; NG3D = 0; }
      qrsten[k] = 0; qisten[k] = 0; qnisten[k] = 0; qcsten[k] = 0; qgsten[k] = 0;
      const Real MU = Real(1.496E-6) * libm::pow(T3D, Real(1.5)) / (T3D + Real(120.));
      DUM = libm::pow(c.rhosu / RHO, Real(0.54));
      ain[k] = libm::pow(c.rhosu / RHO, Real(0.35)) * c.ai;
      arn[k] = DUM * c.ar;
      asn[k] = DUM * c.as;
      acn[k] = G * c.rhow / (Real(18.) * MU);
      agn[k] = DUM * c.ag;
      lami[k] = 0;
      // no cloud / precipitation water and subsaturated: skip the level (GOTO 200)
      if (QC3D < QSMALL && QI3D < QSMALL && QNI3D < QSMALL && QR3D < QSMALL && QG3D < QSMALL) {
        if (T3D < Real(273.15) && QVQVSI < Real(0.999)) continue;
        if (T3D >= Real(273.15) && QVQVS < Real(0.999)) continue;
      }
      const Real KAP = Real(1.414E3) * MU;
      const Real DV = Real(8.794E-5) * libm::pow(T3D, Real(1.81)) / PRES;
      const Real SC = MU / (RHO * DV);
      DUM = (RV * (T3D * T3D));
      const Real DQSDT = xxlv[k] * QVS / DUM;
      const Real DQSIDT = xxls[k] * QVI / DUM;
      const Real ABI = Real(1.) + DQSIDT * xxls[k] / cpm[k];
      const Real AB = Real(1.) + DQSDT * xxlv[k] / cpm[k];
      const Real ARN = arn[k], ASN = asn[k], AGN = agn[k], AIN = ain[k];
      const Real XXLV = xxlv[k], XXLS = xxls[k], CPM = cpm[k], XLF = xlf[k];
      Real LAMR = 0, N0RR = 0, LAMC = 0, PGAM = 0, LAMS = 0, N0S = 0, LAMG = 0, N0G = 0, N0I = 0, CDIST1 = 0;
      Real LAMMIN, LAMMAX;
      Real PCC;
      // MARTIN ET AL. (1994) PGAM and the droplet slope (both temperature branches)
      auto droplets = [&]() {
        DUM = PRES / (Real(287.15) * T3D);
        PGAM = Real(0.0005714) * (NC3D / Real(1.E6) * DUM) + Real(0.2714);
        PGAM = Real(1.) / (PGAM * PGAM) - Real(1.);
        PGAM = c_max(PGAM, Real(2.));
        PGAM = c_min(PGAM, Real(10.));
        LAMC = libm::pow(c.cons26 * NC3D * gamma_fn(PGAM + Real(4.)) / (QC3D * gamma_fn(PGAM + Real(1.))),
                         Real(1.) / Real(3.));
        LAMMIN = (PGAM + Real(1.)) / Real(60.E-6);
        LAMMAX = (PGAM + Real(1.)) / Real(1.E-6);
        if (LAMC < LAMMIN) {
          LAMC = LAMMIN;
          NC3D = libm::exp(Real(3.) * libm::log(LAMC) + libm::log(QC3D) + libm::log(gamma_fn(PGAM + Real(1.))) -
                           libm::log(gamma_fn(PGAM + Real(4.)))) /
                 c.cons26;
        } else if (LAMC > LAMMAX) {
          LAMC = LAMMAX;
          NC3D = libm::exp(Real(3.) * libm::log(LAMC) + libm::log(QC3D) + libm::log(gamma_fn(PGAM + Real(1.))) -
                           libm::log(gamma_fn(PGAM + Real(4.)))) /
                 c.cons26;
        }
      };
      auto rain = [&]() {
        LAMR = libm::pow(PI * c.rhow * NR3D / QR3D, Real(1.) / Real(3.));
        N0RR = NR3D * LAMR;
        if (LAMR < c.lamminr) {
          LAMR = c.lamminr;
          N0RR = (LAMR * LAMR) * (LAMR * LAMR) * QR3D / (PI * c.rhow);
          NR3D = N0RR / LAMR;
        } else if (LAMR > c.lammaxr) {
          LAMR = c.lammaxr;
          N0RR = (LAMR * LAMR) * (LAMR * LAMR) * QR3D / (PI * c.rhow);
          NR3D = N0RR / LAMR;
        }
      };
      auto snow = [&]() {
        LAMS = libm::pow(c.cons1 * NS3D / QNI3D, Real(1.) / c.ds);
        N0S = NS3D * LAMS;
        if (LAMS < c.lammins) {
          LAMS = c.lammins;
          N0S = (LAMS * LAMS) * (LAMS * LAMS) * QNI3D / c.cons1;
          NS3D = N0S / LAMS;
        } else if (LAMS > c.lammaxs) {
          LAMS = c.lammaxs;
          N0S = (LAMS * LAMS) * (LAMS * LAMS) * QNI3D / c.cons1;
          NS3D = N0S / LAMS;
        }
      };
      auto graupel = [&]() {
        LAMG = libm::pow(c.cons2 * NG3D / QG3D, Real(1.) / c.dg);
        N0G = NG3D * LAMG;
        if (LAMG < c.lamming) {
          LAMG = c.lamming;
          N0G = (LAMG * LAMG) * (LAMG * LAMG) * QG3D / c.cons2;
          NG3D = N0G / LAMG;
        } else if (LAMG > c.lammaxg) {
          LAMG = c.lammaxg;
          N0G = (LAMG * LAMG) * (LAMG * LAMG) * QG3D / c.cons2;
          NG3D = N0G / LAMG;
        }
      };
      // the collection rate of rain by snow / graupel (Ikawa and Saito 1991): CONS41*(...)
      auto collect = [&](Real cons, Real UMR, Real UMX, Real N0X, Real LAMX) {
        const Real d = Real(1.2) * UMR - Real(0.95) * UMX;
        return cons * (libm::pow(d * d + Real(0.08) * UMX * UMR, Real(0.5)) * RHO * N0RR * N0X / (LAMR * LAMR * LAMR) *
                       (Real(5.) / ((LAMR * LAMR * LAMR) * LAMX) + Real(2.) / ((LAMR * LAMR) * (LAMX * LAMX)) +
                        Real(0.5) / (LAMR * (LAMX * LAMX * LAMX))));
      };
      auto ncollect = [&](Real UNR, Real UNX, Real N0X, Real LAMX) {
        const Real d = UNR - UNX;
        return c.cons32 * RHO * libm::pow(Real(1.7) * (d * d) + Real(0.3) * UNR * UNX, Real(0.5)) * N0RR * N0X *
               (Real(1.) / ((LAMR * LAMR * LAMR) * LAMX) + Real(1.) / ((LAMR * LAMR) * (LAMX * LAMX)) +
                Real(1.) / (LAMR * (LAMX * LAMX * LAMX)));
      };
      // the evaporation / deposition factor 2 pi N0 rho Dv (f1/lam^2 + f2 (a rho/mu)^0.5 Sc^(1/3) cons/lam^cons)
      auto eps_vent = [&](Real N0X, Real LAMX, Real f1, Real f2, Real AX, Real cg, Real ce) {
        return Real(2.) * PI * N0X * RHO * DV *
               (f1 / (LAMX * LAMX) + f2 * libm::pow(AX * RHO / MU, Real(0.5)) * libm::pow(SC, Real(1.) / Real(3.)) * cg /
                                         (libm::pow(LAMX, ce)));
      };
      if (T3D >= Real(273.15)) {
        // ---- above freezing ----
        NC3D = c.ndcnst * Real(1.E6) / RHO;  // iinum = 1
        // melt very small snow and graupel mixing ratios, add to rain
        if (QNI3D < Real(1.E-6)) {
          QR3D = QR3D + QNI3D;
          NR3D = NR3D + NS3D;
          T3D = T3D - QNI3D * XLF / CPM;
          QNI3D = 0;
          NS3D = 0;
        }
        if (QG3D < Real(1.E-6)) {
          QR3D = QR3D + QG3D;
          NR3D = NR3D + NG3D;
          T3D = T3D - QG3D * XLF / CPM;
          QG3D = 0;
          NG3D = 0;
        }
        if (!(QC3D < QSMALL && QNI3D < Real(1.E-8) && QR3D < QSMALL && QG3D < Real(1.E-8))) {  // else GOTO 300
          NS3D = c_max(Real(0.), NS3D);
          NC3D = c_max(Real(0.), NC3D);
          NR3D = c_max(Real(0.), NR3D);
          NG3D = c_max(Real(0.), NG3D);
          if (QR3D >= QSMALL) rain();
          if (QC3D >= QSMALL) droplets();
          if (QNI3D >= QSMALL) snow();
          if (QG3D >= QSMALL) graupel();
          Real PRC = 0, NPRC = 0, NPRC1 = 0, PRA = 0, NPRA = 0, NRAGG = 0, NSMLTS = 0, NSMLTR = 0, EVPMS = 0;
          Real PRE = 0, NSUBR = 0, PRACG = 0, NPRACG = 0, PSMLT = 0, PGMLT = 0, EVPMG = 0, PRACS = 0;
          Real NGMLTG = 0, NGMLTR = 0;
          Real UMS, UMR, UNS, UNR, UMG, UNG, EPSR, EPSS, EPSG;
          // autoconversion (Khairoutdinov and Kogan 2000)
          if (QC3D >= Real(1.E-6)) {
            PRC = Real(1350.) * libm::pow(QC3D, Real(2.47)) * libm::pow(NC3D / Real(1.e6) * RHO, Real(-1.79));
            NPRC1 = PRC / c.cons29;
            NPRC = PRC / (QC3D / NC3D);
            NPRC = c_min(NPRC, NC3D / DT);
            NPRC1 = c_min(NPRC1, NPRC);
          }
          // collection of snow by rain above freezing (only for the melting)
          if (QR3D >= Real(1.E-8) && QNI3D >= Real(1.E-8)) {
            UMS = ASN * c.cons3 / (libm::pow(LAMS, c.bs));
            UMR = ARN * c.cons4 / (libm::pow(LAMR, c.br));
            UNS = ASN * c.cons5 / libm::pow(LAMS, c.bs);
            UNR = ARN * c.cons6 / libm::pow(LAMR, c.br);
            DUM = libm::pow(c.rhosu / RHO, Real(0.54));
            UMS = c_min(UMS, Real(1.2) * DUM);
            UNS = c_min(UNS, Real(1.2) * DUM);
            UMR = c_min(UMR, Real(9.1) * DUM);
            UNR = c_min(UNR, Real(9.1) * DUM);
            PRACS = collect(c.cons41, UMR, UMS, N0S, LAMS);
          }
          // collection of graupel by rain above freezing, shed as 1 mm drops
          if (QR3D >= Real(1.E-8) && QG3D >= Real(1.E-8)) {
            UMG = AGN * c.cons7 / (libm::pow(LAMG, c.bg));
            UMR = ARN * c.cons4 / (libm::pow(LAMR, c.br));
            UNG = AGN * c.cons8 / libm::pow(LAMG, c.bg);
            UNR = ARN * c.cons6 / libm::pow(LAMR, c.br);
            DUM = libm::pow(c.rhosu / RHO, Real(0.54));
            UMG = c_min(UMG, Real(20.) * DUM);
            UNG = c_min(UNG, Real(20.) * DUM);
            UMR = c_min(UMR, Real(9.1) * DUM);
            UNR = c_min(UNR, Real(9.1) * DUM);
            PRACG = collect(c.cons41, UMR, UMG, N0G, LAMG);
            DUM = PRACG / Real(5.2E-7);
            NPRACG = ncollect(UNR, UNG, N0G, LAMG);
            NPRACG = NPRACG - DUM;
          }
          // accretion of cloud water by rain
          if (QR3D >= Real(1.E-8) && QC3D >= Real(1.E-8)) {
            DUM = (QC3D * QR3D);
            PRA = Real(67.) * libm::pow(DUM, Real(1.15));
            NPRA = PRA / (QC3D / NC3D);
          }
          // self-collection and breakup of rain
          if (QR3D >= Real(1.E-8)) {
            DUM1 = Real(300.e-6);
            if (Real(1.) / LAMR < DUM1) DUM = Real(1.);
            else if (Real(1.) / LAMR >= DUM1) DUM = Real(2.) - libm::exp(Real(2300.) * (Real(1.) / LAMR - DUM1));
            NRAGG = Real(-5.78) * DUM * NR3D * QR3D * RHO;
          }
          // evaporation of rain (Rutledge and Hobbs 1983)
          if (QR3D >= QSMALL) EPSR = eps_vent(N0RR, LAMR, c.f1r, c.f2r, ARN, c.cons9, c.cons34);
          else EPSR = 0;
          if (QV3D < QVS) {
            PRE = EPSR * (QV3D - QVS) / AB;
            PRE = c_min(PRE, Real(0.));
          } else {
            PRE = 0;
          }
          // melting of snow (Rutledge and Hobbs 1984), accelerated by the collection of rain
          if (QNI3D >= Real(1.E-8)) {
            DUM = -c.cpw / XLF * (T3D - Real(273.15)) * PRACS;
            PSMLT = Real(2.) * PI * N0S * KAP * (Real(273.15) - T3D) / XLF *
                        (c.f1s / (LAMS * LAMS) + c.f2s * libm::pow(ASN * RHO / MU, Real(0.5)) *
                                                     libm::pow(SC, Real(1.) / Real(3.)) * c.cons10 /
                                                     (libm::pow(LAMS, c.cons35))) +
                    DUM;
            if (QVQVS < Real(1.)) {
              EPSS = eps_vent(N0S, LAMS, c.f1s, c.f2s, ASN, c.cons10, c.cons35);
              EVPMS = (QV3D - QVS) * EPSS / AB;
              EVPMS = c_max(EVPMS, PSMLT);
              PSMLT = PSMLT - EVPMS;
            }
          }
          // melting of graupel
          if (QG3D >= Real(1.E-8)) {
            DUM = -c.cpw / XLF * (T3D - Real(273.15)) * PRACG;
            PGMLT = Real(2.) * PI * N0G * KAP * (Real(273.15) - T3D) / XLF *
                        (c.f1s / (LAMG * LAMG) + c.f2s * libm::pow(AGN * RHO / MU, Real(0.5)) *
                                                     libm::pow(SC, Real(1.) / Real(3.)) * c.cons11 /
                                                     (libm::pow(LAMG, c.cons36))) +
                    DUM;
            if (QVQVS < Real(1.)) {
              EPSG = eps_vent(N0G, LAMG, c.f1s, c.f2s, AGN, c.cons11, c.cons36);
              EVPMG = (QV3D - QVS) * EPSG / AB;
              EVPMG = c_max(EVPMG, PGMLT);
              PGMLT = PGMLT - EVPMG;
            }
          }
          PRACG = 0;
          PRACS = 0;
          // conservation of qc, snow, graupel, rain
          DUM = (PRC + PRA) * DT;
          if (DUM > QC3D && QC3D >= QSMALL) {
            RATIO = QC3D / DUM;
            PRC = PRC * RATIO;
            PRA = PRA * RATIO;
          }
          DUM = (-PSMLT - EVPMS + PRACS) * DT;
          if (DUM > QNI3D && QNI3D >= QSMALL) {
            RATIO = QNI3D / DUM;
            PSMLT = PSMLT * RATIO;
            EVPMS = EVPMS * RATIO;
            PRACS = PRACS * RATIO;
          }
          DUM = (-PGMLT - EVPMG + PRACG) * DT;
          if (DUM > QG3D && QG3D >= QSMALL) {
            RATIO = QG3D / DUM;
            PGMLT = PGMLT * RATIO;
            EVPMG = EVPMG * RATIO;
            PRACG = PRACG * RATIO;
          }
          DUM = (-PRACS - PRACG - PRE - PRA - PRC + PSMLT + PGMLT) * DT;
          if (DUM > QR3D && QR3D >= QSMALL) {
            RATIO = (QR3D / DT + PRACS + PRACG + PRA + PRC - PSMLT - PGMLT) / (-PRE);
            PRE = PRE * RATIO;
          }
          qv3dten[k] = qv3dten[k] + (-PRE - EVPMS - EVPMG);
          t3dten[k] = t3dten[k] +
                      (PRE * XXLV + (EVPMS + EVPMG) * XXLS + (PSMLT + PGMLT - PRACS - PRACG) * XLF) / CPM;
          qc3dten[k] = qc3dten[k] + (-PRA - PRC);
          qr3dten[k] = qr3dten[k] + (PRE + PRA + PRC - PSMLT - PGMLT + PRACS + PRACG);
          qni3dten[k] = qni3dten[k] + (PSMLT + EVPMS - PRACS);
          qg3dten[k] = qg3dten[k] + (PGMLT + EVPMG - PRACG);
          nc3dten[k] = nc3dten[k] + (-NPRA - NPRC);
          nr3dten[k] = nr3dten[k] + (NPRC1 + NRAGG - NPRACG);
          if (PRE < Real(0.)) {
            DUM = PRE * DT / QR3D;
            DUM = c_max(Real(-1.), DUM);
            NSUBR = DUM * NR3D / DT;
          }
          if (EVPMS + PSMLT < Real(0.)) {
            DUM = (EVPMS + PSMLT) * DT / QNI3D;
            DUM = c_max(Real(-1.), DUM);
            NSMLTS = DUM * NS3D / DT;
          }
          if (PSMLT < Real(0.)) {
            DUM = PSMLT * DT / QNI3D;
            DUM = c_max(Real(-1.0), DUM);
            NSMLTR = DUM * NS3D / DT;
          }
          if (EVPMG + PGMLT < Real(0.)) {
            DUM = (EVPMG + PGMLT) * DT / QG3D;
            DUM = c_max(Real(-1.), DUM);
            NGMLTG = DUM * NG3D / DT;
          }
          if (PGMLT < Real(0.)) {
            DUM = PGMLT * DT / QG3D;
            DUM = c_max(Real(-1.0), DUM);
            NGMLTR = DUM * NG3D / DT;
          }
          ns3dten[k] = ns3dten[k] + (NSMLTS);
          ng3dten[k] = ng3dten[k] + (NGMLTG);
          nr3dten[k] = nr3dten[k] + (NSUBR - NSMLTR - NGMLTR);
        }
        // 300: saturation adjustment for liquid
        {
          const Real DUMT = T3D + DT * t3dten[k];
          const Real DUMQV = QV3D + DT * qv3dten[k];
          DUM = c_min(Real(0.99) * PRES, polysvp(DUMT, 0));
          const Real DUMQSS = EP_2 * DUM / (PRES - DUM);
          Real DUMQC = QC3D + DT * qc3dten[k];
          DUMQC = c_max(DUMQC, Real(0.));
          const Real DUMS = DUMQV - DUMQSS;
          PCC = DUMS / (Real(1.) + (XXLV * XXLV) * DUMQSS / (CPM * RV * (DUMT * DUMT))) / DT;
          if (PCC * DT + DUMQC < Real(0.)) PCC = -DUMQC / DT;
          qv3dten[k] = qv3dten[k] - PCC;
          t3dten[k] = t3dten[k] + PCC * XXLV / CPM;
          qc3dten[k] = qc3dten[k] + PCC;
        }
      } else {
        // ---- below freezing ----
        NC3D = c.ndcnst * Real(1.E6) / RHO;  // iinum = 1
        NI3D = c_max(Real(0.), NI3D);
        NS3D = c_max(Real(0.), NS3D);
        NC3D = c_max(Real(0.), NC3D);
        NR3D = c_max(Real(0.), NR3D);
        NG3D = c_max(Real(0.), NG3D);
        if (QI3D >= QSMALL) {
          lami[k] = libm::pow(c.cons12 * NI3D / QI3D, Real(1.) / c.di);
          N0I = NI3D * lami[k];
          if (lami[k] < c.lammini) {
            lami[k] = c.lammini;
            N0I = (lami[k] * lami[k]) * (lami[k] * lami[k]) * QI3D / c.cons12;
            NI3D = N0I / lami[k];
          } else if (lami[k] > c.lammaxi) {
            lami[k] = c.lammaxi;
            N0I = (lami[k] * lami[k]) * (lami[k] * lami[k]) * QI3D / c.cons12;
            NI3D = N0I / lami[k];
          }
        }
        const Real LAMI = lami[k];
        if (QR3D >= QSMALL) rain();
        if (QC3D >= QSMALL) {
          droplets();
          CDIST1 = NC3D / gamma_fn(PGAM + Real(1.));  // for the droplet freezing
        }
        if (QNI3D >= QSMALL) snow();
        if (QG3D >= QSMALL) graupel();
        Real MNUCCC = 0, NNUCCC = 0, PRC = 0, NPRC = 0, NPRC1 = 0, NSAGG = 0, PSACWS = 0, NPSACWS = 0, PSACWI = 0;
        Real NPSACWI = 0, PRACS = 0, NPRACS = 0, NMULTS = 0, QMULTS = 0, NMULTR = 0, QMULTR = 0, NMULTG = 0;
        Real QMULTG = 0, NMULTRG = 0, QMULTRG = 0, MNUCCR = 0, NNUCCR = 0, PRA = 0, NPRA = 0, NRAGG = 0, PRCI = 0;
        Real NPRCI = 0, PRAI = 0, NPRAI = 0, NNUCCD = 0, MNUCCD = 0, PRE = 0, PRD = 0, PRDS = 0;
        Real EPRD = 0, EPRDS = 0, NSUBI = 0, NSUBS = 0, NSUBR = 0, PIACR = 0, NIACR = 0, PRACI = 0;
        Real PIACRS = 0, NIACRS = 0, PRACIS = 0, PRACG = 0, PSACR = 0, PSACWG = 0, PGSACW = 0, PGRACS = 0;
        Real PRDG = 0, EPRDG = 0, NPRACG = 0, NPSACWG = 0, NSCNG = 0, NGRACS = 0, NSUBG = 0;
        Real UMS, UMR, UNS, UNR, UMG, UNG, FMULT = 0, EPSI, EPSS, EPSG, EPSR;
        const Real PCC0 = 0;  // PCC(K) is zeroed with the process rates and enters QC3DTEN before the adjustment
        // freezing of cloud droplets below -4 C: contact (Meyers et al. 1992 nuclei) and immersion (Bigg 1953)
        if (QC3D >= QSMALL && T3D < Real(269.15)) {
          const Real NACNT = libm::exp(Real(-2.80) + Real(0.262) * (Real(273.15) - T3D)) * Real(1000.);
          DUM = Real(7.37) * T3D / (Real(288.) * Real(10.) * PRES) / Real(100.);
          const Real DAP = c.cons37 * T3D * (Real(1.) + DUM / c.rin) / MU;
          MNUCCC = c.cons38 * DAP * NACNT *
                   libm::exp(libm::log(CDIST1) + libm::log(gamma_fn(PGAM + Real(5.))) - Real(4.) * libm::log(LAMC));
          NNUCCC = Real(2.) * PI * DAP * NACNT * CDIST1 * gamma_fn(PGAM + Real(2.)) / LAMC;
          MNUCCC = MNUCCC + c.cons39 *
                                libm::exp(libm::log(CDIST1) + libm::log(gamma_fn(Real(7.) + PGAM)) -
                                          Real(6.) * libm::log(LAMC)) *
                                (libm::exp(c.aimm * (Real(273.15) - T3D)) - Real(1.));
          NNUCCC = NNUCCC + c.cons40 *
                                libm::exp(libm::log(CDIST1) + libm::log(gamma_fn(PGAM + Real(4.))) -
                                          Real(3.) * libm::log(LAMC)) *
                                (libm::exp(c.aimm * (Real(273.15) - T3D)) - Real(1.));
          NNUCCC = c_min(NNUCCC, NC3D / DT);
        }
        // autoconversion (Khairoutdinov and Kogan 2000)
        if (QC3D >= Real(1.E-6)) {
          PRC = Real(1350.) * libm::pow(QC3D, Real(2.47)) * libm::pow(NC3D / Real(1.e6) * RHO, Real(-1.79));
          NPRC1 = PRC / c.cons29;
          NPRC = PRC / (QC3D / NC3D);
          NPRC = c_min(NPRC, NC3D / DT);
          NPRC1 = c_min(NPRC1, NPRC);
        }
        // snow aggregation (Passarelli 1978)
        if (QNI3D >= Real(1.E-8)) {
          NSAGG = c.cons15 * ASN * libm::pow(RHO, (Real(2.) + c.bs) / Real(3.)) *
                  libm::pow(QNI3D, (Real(2.) + c.bs) / Real(3.)) *
                  libm::pow(NS3D * RHO, (Real(4.) - c.bs) / Real(3.)) / (RHO);
        }
        // accretion of cloud droplets onto snow, graupel and cloud ice
        if (QNI3D >= Real(1.E-8) && QC3D >= QSMALL) {
          PSACWS = c.cons13 * ASN * QC3D * RHO * N0S / libm::pow(LAMS, c.bs + Real(3.));
          NPSACWS = c.cons13 * ASN * NC3D * RHO * N0S / libm::pow(LAMS, c.bs + Real(3.));
        }
        if (QG3D >= Real(1.E-8) && QC3D >= QSMALL) {
          PSACWG = c.cons14 * AGN * QC3D * RHO * N0G / libm::pow(LAMG, c.bg + Real(3.));
          NPSACWG = c.cons14 * AGN * NC3D * RHO * N0G / libm::pow(LAMG, c.bg + Real(3.));
        }
        if (QI3D >= Real(1.E-8) && QC3D >= QSMALL) {
          if (Real(1.) / LAMI >= Real(100.E-6)) {
            PSACWI = c.cons16 * AIN * QC3D * RHO * N0I / libm::pow(LAMI, c.bi + Real(3.));
            NPSACWI = c.cons16 * AIN * NC3D * RHO * N0I / libm::pow(LAMI, c.bi + Real(3.));
          }
        }
        // accretion of rain by snow (Ikawa and Saito 1991)
        if (QR3D >= Real(1.E-8) && QNI3D >= Real(1.E-8)) {
          UMS = ASN * c.cons3 / (libm::pow(LAMS, c.bs));
          UMR = ARN * c.cons4 / (libm::pow(LAMR, c.br));
          UNS = ASN * c.cons5 / libm::pow(LAMS, c.bs);
          UNR = ARN * c.cons6 / libm::pow(LAMR, c.br);
          DUM = libm::pow(c.rhosu / RHO, Real(0.54));
          UMS = c_min(UMS, Real(1.2) * DUM);
          UNS = c_min(UNS, Real(1.2) * DUM);
          UMR = c_min(UMR, Real(9.1) * DUM);
          UNR = c_min(UNR, Real(9.1) * DUM);
          PRACS = collect(c.cons41, UMR, UMS, N0S, LAMS);
          NPRACS = ncollect(UNR, UNS, N0S, LAMS);
          PRACS = c_min(PRACS, QR3D / DT);
          // collection of snow by rain, for the conversion to graupel (qs and qr >= 0.1 g/kg)
          if (QNI3D >= Real(0.1E-3) && QR3D >= Real(0.1E-3)) {
            const Real d = Real(1.2) * UMR - Real(0.95) * UMS;
            PSACR = c.cons31 * (libm::pow(d * d + Real(0.08) * UMS * UMR, Real(0.5)) * RHO * N0RR * N0S /
                                (LAMS * LAMS * LAMS) *
                                (Real(5.) / ((LAMS * LAMS * LAMS) * LAMR) + Real(2.) / ((LAMS * LAMS) * (LAMR * LAMR)) +
                                 Real(0.5) / (LAMS * (LAMR * LAMR * LAMR))));
          }
        }
        // collection of rain by graupel
        if (QR3D >= Real(1.E-8) && QG3D >= Real(1.E-8)) {
          UMG = AGN * c.cons7 / (libm::pow(LAMG, c.bg));
          UMR = ARN * c.cons4 / (libm::pow(LAMR, c.br));
          UNG = AGN * c.cons8 / libm::pow(LAMG, c.bg);
          UNR = ARN * c.cons6 / libm::pow(LAMR, c.br);
          DUM = libm::pow(c.rhosu / RHO, Real(0.54));
          UMG = c_min(UMG, Real(20.) * DUM);
          UNG = c_min(UNG, Real(20.) * DUM);
          UMR = c_min(UMR, Real(9.1) * DUM);
          UNR = c_min(UNR, Real(9.1) * DUM);
          PRACG = collect(c.cons41, UMR, UMG, N0G, LAMG);
          NPRACG = ncollect(UNR, UNG, N0G, LAMG);
          PRACG = c_min(PRACG, QR3D / DT);
        }
        // rime splintering (Hallett-Mossop 1974) on snow
        auto fmult = [&]() {
          if (T3D > Real(270.16)) FMULT = 0;
          else if (T3D <= Real(270.16) && T3D > Real(268.16)) FMULT = (Real(270.16) - T3D) / Real(2.);
          else if (T3D >= Real(265.16) && T3D <= Real(268.16)) FMULT = (T3D - Real(265.16)) / Real(3.);
          else if (T3D < Real(265.16)) FMULT = 0;
        };
        if (QNI3D >= Real(0.1E-3)) {
          if (QC3D >= Real(0.5E-3) || QR3D >= Real(0.1E-3)) {
            if (PSACWS > Real(0.) || PRACS > Real(0.)) {
              if (T3D < Real(270.16) && T3D > Real(265.16)) {
                fmult();
                if (PSACWS > Real(0.)) {
                  NMULTS = Real(35.E4) * PSACWS * FMULT * Real(1000.);
                  QMULTS = NMULTS * c.mmult;
                  QMULTS = c_min(QMULTS, PSACWS);
                  PSACWS = PSACWS - QMULTS;
                }
                if (PRACS > Real(0.)) {
                  NMULTR = Real(35.E4) * PRACS * FMULT * Real(1000.);
                  QMULTR = NMULTR * c.mmult;
                  QMULTR = c_min(QMULTR, PRACS);
                  PRACS = PRACS - QMULTR;
                }
              }
            }
          }
        }
        // rime splintering on graupel
        if (QG3D >= Real(0.1E-3)) {
          if (QC3D >= Real(0.5E-3) || QR3D >= Real(0.1E-3)) {
            if (PSACWG > Real(0.) || PRACG > Real(0.)) {
              if (T3D < Real(270.16) && T3D > Real(265.16)) {
                fmult();
                if (PSACWG > Real(0.)) {
                  NMULTG = Real(35.E4) * PSACWG * FMULT * Real(1000.);
                  QMULTG = NMULTG * c.mmult;
                  QMULTG = c_min(QMULTG, PSACWG);
                  PSACWG = PSACWG - QMULTG;
                }
                if (PRACG > Real(0.)) {
                  NMULTRG = Real(35.E4) * PRACG * FMULT * Real(1000.);
                  QMULTRG = NMULTRG * c.mmult;
                  QMULTRG = c_min(QMULTRG, PRACG);
                  PRACG = PRACG - QMULTRG;
                }
              }
            }
          }
        }
        // conversion of rimed cloud water onto snow to graupel (Reisner et al. 1998)
        if (PSACWS > Real(0.)) {
          if (QNI3D >= Real(0.1E-3) && QC3D >= Real(0.5E-3)) {
            PGSACW = c_min(PSACWS, c.cons17 * DT * N0S * QC3D * QC3D * ASN * ASN /
                                       (RHO * libm::pow(LAMS, Real(2.) * c.bs + Real(2.))));
            DUM = c_max(c.rhosn / (c.rhog - c.rhosn) * PGSACW, Real(0.));
            NSCNG = DUM / c.mg0 * RHO;
            NSCNG = c_min(NSCNG, NS3D / DT);
            PSACWS = PSACWS - PGSACW;
          }
        }
        // conversion of rimed rain water onto snow to graupel
        if (PRACS > Real(0.)) {
          if (QNI3D >= Real(0.1E-3) && QR3D >= Real(0.1E-3)) {
            const Real s4 = Real(4.) / LAMS, r4 = Real(4.) / LAMR;
            const Real s43 = (s4 * s4) * s4, r43 = (r4 * r4) * r4;
            DUM = c.cons18 * s43 * s43 / (c.cons18 * s43 * s43 + c.cons19 * r43 * r43);
            DUM = c_min(DUM, Real(1.));
            DUM = c_max(DUM, Real(0.));
            PGRACS = (Real(1.) - DUM) * PRACS;
            NGRACS = (Real(1.) - DUM) * NPRACS;
            NGRACS = c_min(NGRACS, NR3D / DT);
            NGRACS = c_min(NGRACS, NS3D / DT);
            PRACS = PRACS - PGRACS;
            NPRACS = NPRACS - NGRACS;
            PSACR = PSACR * (Real(1.) - DUM);
          }
        }
        // freezing of rain drops below -4 C (Bigg 1953)
        if (T3D < Real(269.15) && QR3D >= QSMALL) {
          const Real L3 = (LAMR * LAMR) * LAMR;
          MNUCCR = c.cons20 * NR3D * (libm::exp(c.aimm * (Real(273.15) - T3D)) - Real(1.)) / L3 / L3;
          NNUCCR = PI * NR3D * c.bimm * (libm::exp(c.aimm * (Real(273.15) - T3D)) - Real(1.)) / L3;
          NNUCCR = c_min(NNUCCR, NR3D / DT);
        }
        // accretion of cloud water by rain
        if (QR3D >= Real(1.E-8) && QC3D >= Real(1.E-8)) {
          DUM = (QC3D * QR3D);
          PRA = Real(67.) * libm::pow(DUM, Real(1.15));
          NPRA = PRA / (QC3D / NC3D);
        }
        // self-collection and breakup of rain
        if (QR3D >= Real(1.E-8)) {
          DUM1 = Real(300.e-6);
          if (Real(1.) / LAMR < DUM1) DUM = Real(1.);
          else if (Real(1.) / LAMR >= DUM1) DUM = Real(2.) - libm::exp(Real(2300.) * (Real(1.) / LAMR - DUM1));
          NRAGG = Real(-5.78) * DUM * NR3D * QR3D * RHO;
        }
        // autoconversion of cloud ice to snow (Harrington et al. 1995), in ice supersaturation
        if (QI3D >= Real(1.E-8) && QVQVSI >= Real(1.)) {
          NPRCI = c.cons21 * (QV3D - QVI) * RHO * N0I * libm::exp(-LAMI * c.dcs) * DV / ABI;
          PRCI = c.cons22 * NPRCI;
          NPRCI = c_min(NPRCI, NI3D / DT);
        }
        // accretion of cloud ice by snow
        if (QNI3D >= Real(1.E-8) && QI3D >= QSMALL) {
          PRAI = c.cons23 * ASN * QI3D * RHO * N0S / libm::pow(LAMS, c.bs + Real(3.));
          NPRAI = c.cons23 * ASN * NI3D * RHO * N0S / libm::pow(LAMS, c.bs + Real(3.));
          NPRAI = c_min(NPRAI, NI3D / DT);
        }
        // collision of rain and ice to produce snow or graupel (Reisner et al. 1998)
        if (QR3D >= Real(1.E-8) && QI3D >= Real(1.E-8) && T3D <= Real(273.15)) {
          if (QR3D >= Real(0.1E-3)) {
            NIACR = c.cons24 * NI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) * RHO;
            PIACR = c.cons25 * NI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) / ((LAMR * LAMR) * LAMR) * RHO;
            PRACI = c.cons24 * QI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) * RHO;
            NIACR = c_min(NIACR, NR3D / DT);
            NIACR = c_min(NIACR, NI3D / DT);
          } else {
            NIACRS = c.cons24 * NI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) * RHO;
            PIACRS = c.cons25 * NI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) / ((LAMR * LAMR) * LAMR) * RHO;
            PRACIS = c.cons24 * QI3D * N0RR * ARN / libm::pow(LAMR, c.br + Real(3.)) * RHO;
            NIACRS = c_min(NIACRS, NR3D / DT);
            NIACRS = c_min(NIACRS, NI3D / DT);
          }
        }
        // ice nucleation (INUC = 0: Cooper curve, limited to 500 L-1)
        if ((QVQVS >= Real(0.999) && T3D <= Real(265.15)) || QVQVSI >= Real(1.08)) {
          Real kc2 = Real(0.005) * libm::exp(Real(0.304) * (Real(273.15) - T3D)) * Real(1000.);
          kc2 = c_min(kc2, Real(500.e3));
          kc2 = c_max(kc2 / RHO, Real(0.));
          if (kc2 > NI3D + NS3D + NG3D) {
            NNUCCD = (kc2 - NI3D - NS3D - NG3D) / DT;
            MNUCCD = NNUCCD * c.mi0;
          }
        }
        // 101: evaporation / sublimation / deposition of qi, qs, qg, qr (no ventilation for cloud ice)
        if (QI3D >= QSMALL) EPSI = Real(2.) * PI * N0I * RHO * DV / (LAMI * LAMI);
        else EPSI = 0;
        if (QNI3D >= QSMALL) EPSS = eps_vent(N0S, LAMS, c.f1s, c.f2s, ASN, c.cons10, c.cons35);
        else EPSS = 0;
        if (QG3D >= QSMALL) EPSG = eps_vent(N0G, LAMG, c.f1s, c.f2s, AGN, c.cons11, c.cons36);
        else EPSG = 0;
        if (QR3D >= QSMALL) EPSR = eps_vent(N0RR, LAMR, c.f1r, c.f2r, ARN, c.cons9, c.cons34);
        else EPSR = 0;
        // only the part of the ice size distribution below DCS (Harrington et al. 1995)
        if (QI3D >= QSMALL) {
          DUM = (Real(1.) - libm::exp(-LAMI * c.dcs) * (Real(1.) + LAMI * c.dcs));
          PRD = EPSI * (QV3D - QVI) / ABI * DUM;
        } else {
          DUM = 0;
        }
        if (QNI3D >= QSMALL) {
          PRDS = EPSS * (QV3D - QVI) / ABI + EPSI * (QV3D - QVI) / ABI * (Real(1.) - DUM);
        } else {
          PRD = PRD + EPSI * (QV3D - QVI) / ABI * (Real(1.) - DUM);
        }
        PRDG = EPSG * (QV3D - QVI) / ABI;
        if (QV3D < QVS) {
          PRE = EPSR * (QV3D - QVS) / AB;
          PRE = c_min(PRE, Real(0.));
        } else {
          PRE = 0;
        }
        // not into ice supersaturation / subsaturation (Reisner 2)
        DUM = (QV3D - QVI) / DT;
        const Real FUDGEF = Real(0.9999);
        const Real SUM_DEP = PRD + PRDS + MNUCCD + PRDG;
        if ((DUM > Real(0.) && SUM_DEP > DUM * FUDGEF) || (DUM < Real(0.) && SUM_DEP < DUM * FUDGEF)) {
          MNUCCD = FUDGEF * MNUCCD * DUM / SUM_DEP;
          PRD = FUDGEF * PRD * DUM / SUM_DEP;
          PRDS = FUDGEF * PRDS * DUM / SUM_DEP;
          PRDG = FUDGEF * PRDG * DUM / SUM_DEP;
        }
        if (PRD < Real(0.)) {
          EPRD = PRD;
          PRD = 0;
        }
        if (PRDS < Real(0.)) {
          EPRDS = PRDS;
          PRDS = 0;
        }
        if (PRDG < Real(0.)) {
          EPRDG = PRDG;
          PRDG = 0;
        }
        // conservation of qc, qi, qr, qs (IGRAUP = 0), qg
        DUM = (PRC + PRA + MNUCCC + PSACWS + PSACWI + QMULTS + PSACWG + PGSACW + QMULTG) * DT;
        if (DUM > QC3D && QC3D >= QSMALL) {
          RATIO = QC3D / DUM;
          PRC = PRC * RATIO;
          PRA = PRA * RATIO;
          MNUCCC = MNUCCC * RATIO;
          PSACWS = PSACWS * RATIO;
          PSACWI = PSACWI * RATIO;
          QMULTS = QMULTS * RATIO;
          QMULTG = QMULTG * RATIO;
          PSACWG = PSACWG * RATIO;
          PGSACW = PGSACW * RATIO;
        }
        DUM = (-PRD - MNUCCC + PRCI + PRAI - QMULTS - QMULTG - QMULTR - QMULTRG - MNUCCD + PRACI + PRACIS - EPRD -
               PSACWI) *
              DT;
        if (DUM > QI3D && QI3D >= QSMALL) {
          RATIO = (QI3D / DT + PRD + MNUCCC + QMULTS + QMULTG + QMULTR + QMULTRG + MNUCCD + PSACWI) /
                  (PRCI + PRAI + PRACI + PRACIS - EPRD);
          PRCI = PRCI * RATIO;
          PRAI = PRAI * RATIO;
          PRACI = PRACI * RATIO;
          PRACIS = PRACIS * RATIO;
          EPRD = EPRD * RATIO;
        }
        DUM = ((PRACS - PRE) + (QMULTR + QMULTRG - PRC) + (MNUCCR - PRA) + PIACR + PIACRS + PGRACS + PRACG) * DT;
        if (DUM > QR3D && QR3D >= QSMALL) {
          RATIO = (QR3D / DT + PRC + PRA) /
                  (-PRE + QMULTR + QMULTRG + PRACS + MNUCCR + PIACR + PIACRS + PGRACS + PRACG);
          PRE = PRE * RATIO;
          PRACS = PRACS * RATIO;
          QMULTR = QMULTR * RATIO;
          QMULTRG = QMULTRG * RATIO;
          MNUCCR = MNUCCR * RATIO;
          PIACR = PIACR * RATIO;
          PIACRS = PIACRS * RATIO;
          PGRACS = PGRACS * RATIO;
          PRACG = PRACG * RATIO;
        }
        DUM = (-PRDS - PSACWS - PRAI - PRCI - PRACS - EPRDS + PSACR - PIACRS - PRACIS) * DT;
        if (DUM > QNI3D && QNI3D >= QSMALL) {
          RATIO = (QNI3D / DT + PRDS + PSACWS + PRAI + PRCI + PRACS + PIACRS + PRACIS) / (-EPRDS + PSACR);
          EPRDS = EPRDS * RATIO;
          PSACR = PSACR * RATIO;
        }
        DUM = (-PSACWG - PRACG - PGSACW - PGRACS - PRDG - MNUCCR - EPRDG - PIACR - PRACI - PSACR) * DT;
        if (DUM > QG3D && QG3D >= QSMALL) {
          RATIO = (QG3D / DT + PSACWG + PRACG + PGSACW + PGRACS + PRDG + MNUCCR + PSACR + PIACR + PRACI) / (-EPRDG);
          EPRDG = EPRDG * RATIO;
        }
        // tendencies
        qv3dten[k] = qv3dten[k] + (-PRE - PRD - PRDS - MNUCCD - EPRD - EPRDS - PRDG - EPRDG);
        t3dten[k] = t3dten[k] + (PRE * XXLV + (PRD + PRDS + MNUCCD + EPRD + EPRDS + PRDG + EPRDG) * XXLS +
                                 (PSACWS + PSACWI + MNUCCC + MNUCCR + QMULTS + QMULTG + QMULTR + QMULTRG + PRACS +
                                  PSACWG + PRACG + PGSACW + PGRACS + PIACR + PIACRS) *
                                     XLF) /
                                    CPM;
        qc3dten[k] = qc3dten[k] +
                     (-PRA - PRC - MNUCCC + PCC0 - PSACWS - PSACWI - QMULTS - QMULTG - PSACWG - PGSACW);
        qi3dten[k] = qi3dten[k] + (PRD + EPRD + PSACWI + MNUCCC - PRCI - PRAI + QMULTS + QMULTG + QMULTR + QMULTRG +
                                   MNUCCD - PRACI - PRACIS);
        qr3dten[k] = qr3dten[k] + (PRE + PRA + PRC - PRACS - MNUCCR - QMULTR - QMULTRG - PIACR - PIACRS - PRACG -
                                   PGRACS);
        qni3dten[k] = qni3dten[k] + (PRAI + PSACWS + PRDS + PRACS + PRCI + EPRDS - PSACR + PIACRS + PRACIS);
        ns3dten[k] = ns3dten[k] + (NSAGG + NPRCI - NSCNG - NGRACS + NIACRS);
        qg3dten[k] = qg3dten[k] + (PRACG + PSACWG + PGSACW + PGRACS + PRDG + EPRDG + MNUCCR + PIACR + PRACI + PSACR);
        ng3dten[k] = ng3dten[k] + (NSCNG + NGRACS + NNUCCR + NIACR);
        nc3dten[k] = nc3dten[k] + (-NNUCCC - NPSACWS - NPRA - NPRC - NPSACWI - NPSACWG);
        ni3dten[k] = ni3dten[k] + (NNUCCC - NPRCI - NPRAI + NMULTS + NMULTG + NMULTR + NMULTRG + NNUCCD - NIACR -
                                   NIACRS);
        nr3dten[k] = nr3dten[k] + (NPRC1 - NPRACS - NNUCCR + NRAGG - NIACR - NIACRS - NPRACG - NGRACS);
        // saturation adjustment for liquid
        {
          const Real DUMT = T3D + DT * t3dten[k];
          const Real DUMQV = QV3D + DT * qv3dten[k];
          DUM = c_min(Real(0.99) * PRES, polysvp(DUMT, 0));
          const Real DUMQSS = EP_2 * DUM / (PRES - DUM);
          Real DUMQC = QC3D + DT * qc3dten[k];
          DUMQC = c_max(DUMQC, Real(0.));
          const Real DUMS = DUMQV - DUMQSS;
          PCC = DUMS / (Real(1.) + (XXLV * XXLV) * DUMQSS / (CPM * RV * (DUMT * DUMT))) / DT;
          if (PCC * DT + DUMQC < Real(0.)) PCC = -DUMQC / DT;
          qv3dten[k] = qv3dten[k] - PCC;
          t3dten[k] = t3dten[k] + PCC * XXLV / CPM;
          qc3dten[k] = qc3dten[k] + PCC;
        }
        // sublimation of number with the mass
        if (EPRD < Real(0.)) {
          DUM = EPRD * DT / QI3D;
          DUM = c_max(Real(-1.), DUM);
          NSUBI = DUM * NI3D / DT;
        }
        if (EPRDS < Real(0.)) {
          DUM = EPRDS * DT / QNI3D;
          DUM = c_max(Real(-1.), DUM);
          NSUBS = DUM * NS3D / DT;
        }
        if (PRE < Real(0.)) {
          DUM = PRE * DT / QR3D;
          DUM = c_max(Real(-1.), DUM);
          NSUBR = DUM * NR3D / DT;
        }
        if (EPRDG < Real(0.)) {
          DUM = EPRDG * DT / QG3D;
          DUM = c_max(Real(-1.), DUM);
          NSUBG = DUM * NG3D / DT;
        }
        ni3dten[k] = ni3dten[k] + NSUBI;
        ns3dten[k] = ns3dten[k] + NSUBS;
        ng3dten[k] = ng3dten[k] + NSUBG;
        nr3dten[k] = nr3dten[k] + NSUBR;
      }
      ltrue = 1;
    }
    Real PRECRT = 0, SNOWRT = 0, SNOWPRT = 0, GRPLPRT = 0;
    if (ltrue != 0) {
      // sedimentation (Reisner et al. 1998) in NSTEP split steps (Courant number < 1)
      int NSTEP = 1;
      for (int k = nz - 1; k >= 0; --k) {
        const Real RHO = rho[k];
        dumi[k] = qi(i, j, k) + qi3dten[k] * DT;
        dumqs[k] = qs(i, j, k) + qni3dten[k] * DT;
        dumr[k] = qr(i, j, k) + qr3dten[k] * DT;
        dumfni[k] = ni(i, j, k) + ni3dten[k] * DT;
        dumfns[k] = ns(i, j, k) + ns3dten[k] * DT;
        dumfnr[k] = nr(i, j, k) + nr3dten[k] * DT;
        dumc[k] = qc(i, j, k) + qc3dten[k] * DT;
        dumfnc[k] = nc3d[k] + nc3dten[k] * DT;
        dumg[k] = qg(i, j, k) + qg3dten[k] * DT;
        dumfng[k] = ng(i, j, k) + ng3dten[k] * DT;
        dumfnc[k] = nc3d[k];  // iinum = 1
        dumfni[k] = c_max(Real(0.), dumfni[k]);
        dumfns[k] = c_max(Real(0.), dumfns[k]);
        dumfnc[k] = c_max(Real(0.), dumfnc[k]);
        dumfnr[k] = c_max(Real(0.), dumfnr[k]);
        dumfng[k] = c_max(Real(0.), dumfng[k]);
        Real DLAMI = 0, DLAMR = 0, DLAMC = 0, DLAMS = 0, DLAMG = 0, PGAM = 0;
        if (dumi[k] >= QSMALL) {
          DLAMI = libm::pow(c.cons12 * dumfni[k] / dumi[k], Real(1.) / c.di);
          DLAMI = c_max(DLAMI, c.lammini);
          DLAMI = c_min(DLAMI, c.lammaxi);
        }
        if (dumr[k] >= QSMALL) {
          DLAMR = libm::pow(PI * c.rhow * dumfnr[k] / dumr[k], Real(1.) / Real(3.));
          DLAMR = c_max(DLAMR, c.lamminr);
          DLAMR = c_min(DLAMR, c.lammaxr);
        }
        if (dumc[k] >= QSMALL) {
          const Real DUM = p(i, j, k) / (Real(287.15) * t3d[k]);
          PGAM = Real(0.0005714) * (nc3d[k] / Real(1.E6) * DUM) + Real(0.2714);
          PGAM = Real(1.) / (PGAM * PGAM) - Real(1.);
          PGAM = c_max(PGAM, Real(2.));
          PGAM = c_min(PGAM, Real(10.));
          DLAMC = libm::pow(c.cons26 * dumfnc[k] * gamma_fn(PGAM + Real(4.)) / (dumc[k] * gamma_fn(PGAM + Real(1.))),
                            Real(1.) / Real(3.));
          const Real LAMMIN = (PGAM + Real(1.)) / Real(60.E-6);
          const Real LAMMAX = (PGAM + Real(1.)) / Real(1.E-6);
          DLAMC = c_max(DLAMC, LAMMIN);
          DLAMC = c_min(DLAMC, LAMMAX);
        }
        if (dumqs[k] >= QSMALL) {
          DLAMS = libm::pow(c.cons1 * dumfns[k] / dumqs[k], Real(1.) / c.ds);
          DLAMS = c_max(DLAMS, c.lammins);
          DLAMS = c_min(DLAMS, c.lammaxs);
        }
        if (dumg[k] >= QSMALL) {
          DLAMG = libm::pow(c.cons2 * dumfng[k] / dumg[k], Real(1.) / c.dg);
          DLAMG = c_max(DLAMG, c.lamming);
          DLAMG = c_min(DLAMG, c.lammaxg);
        }
        // number- and mass-weighted fall speeds
        Real UNC, UMC, UNI, UMI, UNR, UMR, UMS, UNS, UMG, UNG;
        if (dumc[k] >= QSMALL) {
          UNC = acn[k] * gamma_fn(Real(1.) + c.bc + PGAM) / (libm::pow(DLAMC, c.bc) * gamma_fn(PGAM + Real(1.)));
          UMC = acn[k] * gamma_fn(Real(4.) + c.bc + PGAM) / (libm::pow(DLAMC, c.bc) * gamma_fn(PGAM + Real(4.)));
        } else {
          UMC = 0;
          UNC = 0;
        }
        if (dumi[k] >= QSMALL) {
          UNI = ain[k] * c.cons27 / libm::pow(DLAMI, c.bi);
          UMI = ain[k] * c.cons28 / (libm::pow(DLAMI, c.bi));
        } else {
          UMI = 0;
          UNI = 0;
        }
        if (dumr[k] >= QSMALL) {
          UNR = arn[k] * c.cons6 / libm::pow(DLAMR, c.br);
          UMR = arn[k] * c.cons4 / (libm::pow(DLAMR, c.br));
        } else {
          UMR = 0;
          UNR = 0;
        }
        if (dumqs[k] >= QSMALL) {
          UMS = asn[k] * c.cons3 / (libm::pow(DLAMS, c.bs));
          UNS = asn[k] * c.cons5 / libm::pow(DLAMS, c.bs);
        } else {
          UMS = 0;
          UNS = 0;
        }
        if (dumg[k] >= QSMALL) {
          UMG = agn[k] * c.cons7 / (libm::pow(DLAMG, c.bg));
          UNG = agn[k] * c.cons8 / libm::pow(DLAMG, c.bg);
        } else {
          UMG = 0;
          UNG = 0;
        }
        const Real DUM = libm::pow(c.rhosu / RHO, Real(0.54));
        UMS = c_min(UMS, Real(1.2) * DUM);
        UNS = c_min(UNS, Real(1.2) * DUM);
        UMI = c_min(UMI, Real(1.2) * libm::pow(c.rhosu / RHO, Real(0.35)));
        UNI = c_min(UNI, Real(1.2) * libm::pow(c.rhosu / RHO, Real(0.35)));
        UMR = c_min(UMR, Real(9.1) * DUM);
        UNR = c_min(UNR, Real(9.1) * DUM);
        UMG = c_min(UMG, Real(20.) * DUM);
        UNG = c_min(UNG, Real(20.) * DUM);
        fr[k] = UMR;
        fi[k] = UMI;
        fni[k] = UNI;
        fs[k] = UMS;
        fns[k] = UNS;
        fnr[k] = UNR;
        fc[k] = UMC;
        fnc[k] = UNC;
        fg[k] = UMG;
        fng[k] = UNG;
        // V3.3: below the precipitation level the fall speed of the level above
        if (k <= nz - 2) {
          if (fr[k] < Real(1.E-10)) fr[k] = fr[k + 1];
          if (fi[k] < Real(1.E-10)) fi[k] = fi[k + 1];
          if (fni[k] < Real(1.E-10)) fni[k] = fni[k + 1];
          if (fs[k] < Real(1.E-10)) fs[k] = fs[k + 1];
          if (fns[k] < Real(1.E-10)) fns[k] = fns[k + 1];
          if (fnr[k] < Real(1.E-10)) fnr[k] = fnr[k + 1];
          if (fc[k] < Real(1.E-10)) fc[k] = fc[k + 1];
          if (fnc[k] < Real(1.E-10)) fnc[k] = fnc[k + 1];
          if (fg[k] < Real(1.E-10)) fg[k] = fg[k + 1];
          if (fng[k] < Real(1.E-10)) fng[k] = fng[k + 1];
        }
        // MAX(FR, FI, FS, FC, FNI, FNR, FNS, FNC, FG, FNG)
        Real RGVM = fr[k];
        RGVM = c_max(RGVM, fi[k]);
        RGVM = c_max(RGVM, fs[k]);
        RGVM = c_max(RGVM, fc[k]);
        RGVM = c_max(RGVM, fni[k]);
        RGVM = c_max(RGVM, fnr[k]);
        RGVM = c_max(RGVM, fns[k]);
        RGVM = c_max(RGVM, fnc[k]);
        RGVM = c_max(RGVM, fg[k]);
        RGVM = c_max(RGVM, fng[k]);
        const int ns_k = int(RGVM * DT / dz(i, j, k) + Real(1.));
        NSTEP = ns_k > NSTEP ? ns_k : NSTEP;
        dumr[k] = dumr[k] * RHO;
        dumi[k] = dumi[k] * RHO;
        dumfni[k] = dumfni[k] * RHO;
        dumqs[k] = dumqs[k] * RHO;
        dumfns[k] = dumfns[k] * RHO;
        dumfnr[k] = dumfnr[k] * RHO;
        dumc[k] = dumc[k] * RHO;
        dumfnc[k] = dumfnc[k] * RHO;
        dumg[k] = dumg[k] * RHO;
        dumfng[k] = dumfng[k] * RHO;
      }
      const Real RNSTEP = Real(NSTEP);
      for (int n = 1; n <= NSTEP; ++n) {
        for (int k = 0; k < nz; ++k) {
          faloutr[k] = fr[k] * dumr[k];
          falouti[k] = fi[k] * dumi[k];
          faloutni[k] = fni[k] * dumfni[k];
          falouts[k] = fs[k] * dumqs[k];
          faloutns[k] = fns[k] * dumfns[k];
          faloutnr[k] = fnr[k] * dumfnr[k];
          faloutc[k] = fc[k] * dumc[k];
          faloutnc[k] = fnc[k] * dumfnc[k];
          faloutg[k] = fg[k] * dumg[k];
          faloutng[k] = fng[k] * dumfng[k];
        }
        {  // top of the model
          const int k = nz - 1;
          const Real DZQ = dz(i, j, k), RHO = rho[k];
          const Real FALTNDR = faloutr[k] / DZQ, FALTNDI = falouti[k] / DZQ, FALTNDNI = faloutni[k] / DZQ;
          const Real FALTNDS = falouts[k] / DZQ, FALTNDNS = faloutns[k] / DZQ, FALTNDNR = faloutnr[k] / DZQ;
          const Real FALTNDC = faloutc[k] / DZQ, FALTNDNC = faloutnc[k] / DZQ, FALTNDG = faloutg[k] / DZQ;
          const Real FALTNDNG = faloutng[k] / DZQ;
          qrsten[k] = qrsten[k] - FALTNDR / RNSTEP / RHO;
          qisten[k] = qisten[k] - FALTNDI / RNSTEP / RHO;
          ni3dten[k] = ni3dten[k] - FALTNDNI / RNSTEP / RHO;
          qnisten[k] = qnisten[k] - FALTNDS / RNSTEP / RHO;
          ns3dten[k] = ns3dten[k] - FALTNDNS / RNSTEP / RHO;
          nr3dten[k] = nr3dten[k] - FALTNDNR / RNSTEP / RHO;
          qcsten[k] = qcsten[k] - FALTNDC / RNSTEP / RHO;
          nc3dten[k] = nc3dten[k] - FALTNDNC / RNSTEP / RHO;
          qgsten[k] = qgsten[k] - FALTNDG / RNSTEP / RHO;
          ng3dten[k] = ng3dten[k] - FALTNDNG / RNSTEP / RHO;
          dumr[k] = dumr[k] - FALTNDR * DT / RNSTEP;
          dumi[k] = dumi[k] - FALTNDI * DT / RNSTEP;
          dumfni[k] = dumfni[k] - FALTNDNI * DT / RNSTEP;
          dumqs[k] = dumqs[k] - FALTNDS * DT / RNSTEP;
          dumfns[k] = dumfns[k] - FALTNDNS * DT / RNSTEP;
          dumfnr[k] = dumfnr[k] - FALTNDNR * DT / RNSTEP;
          dumc[k] = dumc[k] - FALTNDC * DT / RNSTEP;
          dumfnc[k] = dumfnc[k] - FALTNDNC * DT / RNSTEP;
          dumg[k] = dumg[k] - FALTNDG * DT / RNSTEP;
          dumfng[k] = dumfng[k] - FALTNDNG * DT / RNSTEP;
        }
        for (int k = nz - 2; k >= 0; --k) {
          const Real DZQ = dz(i, j, k), RHO = rho[k];
          const Real FALTNDR = (faloutr[k + 1] - faloutr[k]) / DZQ;
          const Real FALTNDI = (falouti[k + 1] - falouti[k]) / DZQ;
          const Real FALTNDNI = (faloutni[k + 1] - faloutni[k]) / DZQ;
          const Real FALTNDS = (falouts[k + 1] - falouts[k]) / DZQ;
          const Real FALTNDNS = (faloutns[k + 1] - faloutns[k]) / DZQ;
          const Real FALTNDNR = (faloutnr[k + 1] - faloutnr[k]) / DZQ;
          const Real FALTNDC = (faloutc[k + 1] - faloutc[k]) / DZQ;
          const Real FALTNDNC = (faloutnc[k + 1] - faloutnc[k]) / DZQ;
          const Real FALTNDG = (faloutg[k + 1] - faloutg[k]) / DZQ;
          const Real FALTNDNG = (faloutng[k + 1] - faloutng[k]) / DZQ;
          qrsten[k] = qrsten[k] + FALTNDR / RNSTEP / RHO;
          qisten[k] = qisten[k] + FALTNDI / RNSTEP / RHO;
          ni3dten[k] = ni3dten[k] + FALTNDNI / RNSTEP / RHO;
          qnisten[k] = qnisten[k] + FALTNDS / RNSTEP / RHO;
          ns3dten[k] = ns3dten[k] + FALTNDNS / RNSTEP / RHO;
          nr3dten[k] = nr3dten[k] + FALTNDNR / RNSTEP / RHO;
          qcsten[k] = qcsten[k] + FALTNDC / RNSTEP / RHO;
          nc3dten[k] = nc3dten[k] + FALTNDNC / RNSTEP / RHO;
          qgsten[k] = qgsten[k] + FALTNDG / RNSTEP / RHO;
          ng3dten[k] = ng3dten[k] + FALTNDNG / RNSTEP / RHO;
          dumr[k] = dumr[k] + FALTNDR * DT / RNSTEP;
          dumi[k] = dumi[k] + FALTNDI * DT / RNSTEP;
          dumfni[k] = dumfni[k] + FALTNDNI * DT / RNSTEP;
          dumqs[k] = dumqs[k] + FALTNDS * DT / RNSTEP;
          dumfns[k] = dumfns[k] + FALTNDNS * DT / RNSTEP;
          dumfnr[k] = dumfnr[k] + FALTNDNR * DT / RNSTEP;
          dumc[k] = dumc[k] + FALTNDC * DT / RNSTEP;
          dumfnc[k] = dumfnc[k] + FALTNDNC * DT / RNSTEP;
          dumg[k] = dumg[k] + FALTNDG * DT / RNSTEP;
          dumfng[k] = dumfng[k] + FALTNDNG * DT / RNSTEP;
        }
        // precipitation and snowfall of the split step [mm] (1000 mm/m over the water density 1000 kg/m3)
        PRECRT = PRECRT + (faloutr[0] + faloutc[0] + falouts[0] + falouti[0] + faloutg[0]) * DT / RNSTEP;
        SNOWRT = SNOWRT + (falouts[0] + falouti[0] + faloutg[0]) * DT / RNSTEP;
        SNOWPRT = SNOWPRT + (falouti[0] + falouts[0]) * DT / RNSTEP;
        GRPLPRT = GRPLPRT + (faloutg[0]) * DT / RNSTEP;
      }
      for (int k = 0; k < nz; ++k) {
        Real& QV3D = qv(i, j, k);
        Real& QC3D = qc(i, j, k);
        Real& QR3D = qr(i, j, k);
        Real& QI3D = qi(i, j, k);
        Real& QNI3D = qs(i, j, k);
        Real& QG3D = qg(i, j, k);
        Real& NI3D = ni(i, j, k);
        Real& NS3D = ns(i, j, k);
        Real& NR3D = nr(i, j, k);
        Real& NG3D = ng(i, j, k);
        Real& T3D = t3d[k];
        const Real PRES = p(i, j, k);
        qr3dten[k] = qr3dten[k] + qrsten[k];
        qi3dten[k] = qi3dten[k] + qisten[k];
        qc3dten[k] = qc3dten[k] + qcsten[k];
        qg3dten[k] = qg3dten[k] + qgsten[k];
        qni3dten[k] = qni3dten[k] + qnisten[k];
        // all cloud ice to snow where its mean diameter exceeds 2 DCS
        if (QI3D >= QSMALL && T3D < Real(273.15) && lami[k] >= Real(1.E-10)) {
          if (Real(1.) / lami[k] >= Real(2.) * c.dcs) {
            qni3dten[k] = qni3dten[k] + QI3D / DT + qi3dten[k];
            ns3dten[k] = ns3dten[k] + NI3D / DT + ni3dten[k];
            qi3dten[k] = -QI3D / DT;
            ni3dten[k] = -NI3D / DT;
          }
        }
        QC3D = QC3D + qc3dten[k] * DT;
        QI3D = QI3D + qi3dten[k] * DT;
        QNI3D = QNI3D + qni3dten[k] * DT;
        QR3D = QR3D + qr3dten[k] * DT;
        nc3d[k] = nc3d[k] + nc3dten[k] * DT;
        NI3D = NI3D + ni3dten[k] * DT;
        NS3D = NS3D + ns3dten[k] * DT;
        NR3D = NR3D + nr3dten[k] * DT;
        QG3D = QG3D + qg3dten[k] * DT;
        NG3D = NG3D + ng3dten[k] * DT;
        T3D = T3D + t3dten[k] * DT;
        QV3D = QV3D + qv3dten[k] * DT;
        Real EVS = c_min(Real(0.99) * PRES, polysvp(T3D, 0));
        Real EIS = c_min(Real(0.99) * PRES, polysvp(T3D, 1));
        if (EIS > EVS) EIS = EVS;
        const Real QVS = EP_2 * EVS / (PRES - EVS);
        const Real QVI = EP_2 * EIS / (PRES - EIS);
        const Real QVQVS = QV3D / QVS;
        const Real QVQVSI = QV3D / QVI;
        if (QVQVS < Real(0.9)) {
          if (QR3D < Real(1.E-8)) {
            QV3D = QV3D + QR3D;
            T3D = T3D - QR3D * xxlv[k] / cpm[k];
            QR3D = 0;
          }
          if (QC3D < Real(1.E-8)) {
            QV3D = QV3D + QC3D;
            T3D = T3D - QC3D * xxlv[k] / cpm[k];
            QC3D = 0;
          }
        }
        if (QVQVSI < Real(0.9)) {
          if (QI3D < Real(1.E-8)) {
            QV3D = QV3D + QI3D;
            T3D = T3D - QI3D * xxls[k] / cpm[k];
            QI3D = 0;
          }
          if (QNI3D < Real(1.E-8)) {
            QV3D = QV3D + QNI3D;
            T3D = T3D - QNI3D * xxls[k] / cpm[k];
            QNI3D = 0;
          }
          if (QG3D < Real(1.E-8)) {
            QV3D = QV3D + QG3D;
            T3D = T3D - QG3D * xxls[k] / cpm[k];
            QG3D = 0;
          }
        }
        if (QC3D < QSMALL) { QC3D = 0; nc3d[k] = 0; }
        if (QR3D < QSMALL) { QR3D = 0; NR3D = 0; }
        if (QI3D < QSMALL) { QI3D = 0; NI3D = 0; }
        if (QNI3D < QSMALL) { QNI3D = 0; NS3D = 0; }
        if (QG3D < QSMALL) { QG3D = 0; NG3D = 0; }
        if (!(QC3D < QSMALL && QI3D < QSMALL && QNI3D < QSMALL && QR3D < QSMALL && QG3D < QSMALL)) {  // else GOTO 500
          // instantaneous processes: melting of cloud ice, homogeneous freezing of cloud water and rain
          if (QI3D >= QSMALL && T3D >= Real(273.15)) {
            QR3D = QR3D + QI3D;
            T3D = T3D - QI3D * xlf[k] / cpm[k];
            QI3D = 0;
            NR3D = NR3D + NI3D;
            NI3D = 0;
          }
          if (T3D <= Real(233.15) && QC3D >= QSMALL) {
            QI3D = QI3D + QC3D;
            T3D = T3D + QC3D * xlf[k] / cpm[k];
            QC3D = 0;
            NI3D = NI3D + nc3d[k];
            nc3d[k] = 0;
          }
          if (T3D <= Real(233.15) && QR3D >= QSMALL) {
            QG3D = QG3D + QR3D;
            T3D = T3D + QR3D * xlf[k] / cpm[k];
            QR3D = 0;
            NG3D = NG3D + NR3D;
            NR3D = 0;
          }
          NI3D = c_max(Real(0.), NI3D);
          NS3D = c_max(Real(0.), NS3D);
          nc3d[k] = c_max(Real(0.), nc3d[k]);
          NR3D = c_max(Real(0.), NR3D);
          NG3D = c_max(Real(0.), NG3D);
          // the numbers within the slope limits (the droplet number is reset below: iinum = 1)
          if (QI3D >= QSMALL) {
            Real LAMI = libm::pow(c.cons12 * NI3D / QI3D, Real(1.) / c.di);
            if (LAMI < c.lammini) {
              LAMI = c.lammini;
              const Real N0I = (LAMI * LAMI) * (LAMI * LAMI) * QI3D / c.cons12;
              NI3D = N0I / LAMI;
            } else if (LAMI > c.lammaxi) {
              LAMI = c.lammaxi;
              const Real N0I = (LAMI * LAMI) * (LAMI * LAMI) * QI3D / c.cons12;
              NI3D = N0I / LAMI;
            }
          }
          if (QR3D >= QSMALL) {
            Real LAMR = libm::pow(PI * c.rhow * NR3D / QR3D, Real(1.) / Real(3.));
            if (LAMR < c.lamminr) {
              LAMR = c.lamminr;
              const Real N0RR = (LAMR * LAMR) * (LAMR * LAMR) * QR3D / (PI * c.rhow);
              NR3D = N0RR / LAMR;
            } else if (LAMR > c.lammaxr) {
              LAMR = c.lammaxr;
              const Real N0RR = (LAMR * LAMR) * (LAMR * LAMR) * QR3D / (PI * c.rhow);
              NR3D = N0RR / LAMR;
            }
          }
          if (QNI3D >= QSMALL) {
            Real LAMS = libm::pow(c.cons1 * NS3D / QNI3D, Real(1.) / c.ds);
            if (LAMS < c.lammins) {
              LAMS = c.lammins;
              const Real N0S = (LAMS * LAMS) * (LAMS * LAMS) * QNI3D / c.cons1;
              NS3D = N0S / LAMS;
            } else if (LAMS > c.lammaxs) {
              LAMS = c.lammaxs;
              const Real N0S = (LAMS * LAMS) * (LAMS * LAMS) * QNI3D / c.cons1;
              NS3D = N0S / LAMS;
            }
          }
          if (QG3D >= QSMALL) {
            Real LAMG = libm::pow(c.cons2 * NG3D / QG3D, Real(1.) / c.dg);
            if (LAMG < c.lamming) {
              LAMG = c.lamming;
              const Real N0G = (LAMG * LAMG) * (LAMG * LAMG) * QG3D / c.cons2;
              NG3D = N0G / LAMG;
            } else if (LAMG > c.lammaxg) {
              LAMG = c.lammaxg;
              const Real N0G = (LAMG * LAMG) * (LAMG * LAMG) * QG3D / c.cons2;
              NG3D = N0G / LAMG;
            }
          }
        }
        // 500: upper bound on the ice number (0.3 cm-3)
        NI3D = c_min(NI3D, Real(0.3E6) / rho[k]);
      }
    }
    // back to the driver: TH = T/PII, the precipitation of the step
    for (int k = 0; k < nz; ++k) th(i, j, k) = t3d[k] / pii(i, j, k);
    rainnc(i, j) = rainnc(i, j) + PRECRT;
    rainncv(i, j) = PRECRT;
    snownc(i, j) = snownc(i, j) + SNOWPRT;
    snowncv(i, j) = SNOWPRT;
    graupelnc(i, j) = graupelnc(i, j) + GRPLPRT;
    graupelncv(i, j) = GRPLPRT;
    sr(i, j) = SNOWRT / (PRECRT + Real(1.E-12));
  }
};

// The GPU work buffer of run_morrison over range (outside the time loop; CPU build: no-op)
template <typename Real>
void morrison_reserve(const Range2D& range, int nz);

// MP_MORR_TWO_MOMENT over the columns of range
template <typename Real>
void run_morrison(const MorrisonParams<Real>& c, Real dt, const Range2D& range, int nz, View3<Real> th, View3<Real> qv,
                  View3<Real> qc, View3<Real> qr, View3<Real> qi, View3<Real> qs, View3<Real> qg, View3<Real> ni,
                  View3<Real> ns, View3<Real> nr, View3<Real> ng, View3<const Real> pii, View3<const Real> p,
                  View3<const Real> dz, View3<const Real> qrcuten, View3<const Real> qscuten,
                  View3<const Real> qicuten, View2<Real> rainnc, View2<Real> rainncv, View2<Real> snownc,
                  View2<Real> snowncv, View2<Real> graupelnc, View2<Real> graupelncv, View2<Real> sr);

}  // namespace cppwrf::phys::mp_morrison
