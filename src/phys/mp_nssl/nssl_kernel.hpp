// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// NSSL 2-moment microphysics (WRF 4.8.0 phys/module_mp_nssl_2mom.F, mp_physics = 18): the routines of one j row, ported
// statement by statement (the module's subroutines keep their names, arguments and local names; Fortran arrays are FA
// views with the declared bounds over the row's work space). Module variables are NsslParams members (P.name).
#include "phys/mp_nssl/nssl_params.hpp"
#include "phys/mp_nssl/nssl_util.hpp"

namespace cppwrf::phys::mp_nssl {
using libm::atan;
using libm::exp;
using libm::log;
using libm::log10;
using libm::pow;
using libm::sin;
using libm::tanh;
using std::fabs;
using std::sqrt;
#define NR(x) static_cast<Real>(x)
#define CP(x) const_cast<Real*>(x)
// x**y inside a loop that gfortran vectorises (_ZGVbN2vv_pow): WRF's value for iteration i of an n-trip loop
template <class R>
CPPWRF_HD inline R vpow(R a, R b, int i, int n) {
  if constexpr (sizeof(R) == sizeof(double)) {
    return cppwrf::vm::pow(a, b, i, n);
  } else {
    (void)i; (void)n;
    return pow(a, b);
  }
}

// ---- the module's functions (GAMMA_SP, GAMMA_DP, GAMMA_DPR, GAMXINFDP, gaminterp, delbk, delabk; -r8 -fdefault-double-8:
// REAL and DOUBLE PRECISION are both real*8, so GAMMA_SP and GAMMA_DP compute the same)
CPPWRF_HD inline double nssl_gamma_lanczos(double x) {
  const double cof[6] = {76.18009172947146e+0, -86.50532032941677e0, 24.01409824083091e0, -1.231739572450155e0,
                         0.1208650973866179e-2, -0.5395239384953e-5};
  const double stp = 2.5066282746310005e0;
  double y = x;
  double tmp = x + 5.5e0;
  tmp = (x + 0.5e0) * log(tmp) - tmp;
  double ser = 1.000000000190015e0;
  for (int j = 0; j < 6; ++j) {
    y = y + 1.0e0;
    ser = ser + cof[j] / y;
  }
  return exp(tmp + log(stp * ser / x));
}
template <class R>
CPPWRF_HD inline R gamma_sp(R xx) {
  if (xx <= R(0.0)) NSSL_STOP();  // 'Argument to gamma must be > 0!!'
  return R(nssl_gamma_lanczos(double(xx)));
}
CPPWRF_HD inline double gamma_dp(double xx) { return nssl_gamma_lanczos(xx); }
template <class R>
CPPWRF_HD inline double gamma_dpr(R x) { return gamma_dp(double(x)); }
template <class R>
CPPWRF_HD inline double gamxinfdp(R a1, R x1) {
  const double a = a1, x = x1;
  if (x1 <= R(0.0)) return gamma_dp(a);
  const double xam = -x + a * log(x);
  if (xam > 700.0 || a > 170.0) NSSL_STOP();  // 'a and/or x too large'
  double gim = 0;
  if (x == 0.0) {
    gim = gamma_dp(a);
  } else if (x <= 1.0 + a) {
    double s = 1.0 / a;
    double r = s;
    for (int k = 1; k <= 60; ++k) {
      r = r * x / (a + k);
      s = s + r;
      if (fabs(r / s) < 1.0e-15) break;
    }
    const double gin = exp(xam) * s;
    const double ga = gamma_dp(a);
    gim = ga - gin;
  } else if (x > 1.0 + a) {
    double t0 = 0.0;
    for (int k = 60; k >= 1; --k) t0 = (k - a) / (1.0 + k / (x + t0));
    gim = exp(xam) / (x + t0);
  }
  return gim;
}
template <class Real>
CPPWRF_HD inline Real gaminterp(const NsslParams<Real>& P, Real ratio, Real alp, int luindex, int ilh) {
  const int i = c_min(P.nqiacrratio, int(ratio * P.dqiacrratioinv));
  const int j = int(c_max(Real(0.0), c_min(P.maxalphalu, alp)) * P.dqiacralphainv);
  const Real delx = c_min(P.maxratiolu, ratio) - Real(i) * P.dqiacrratio;
  const Real dely = alp - Real(j) * P.dqiacralpha;
  const int ip1 = c_min(i + 1, P.nqiacrratio);
  const int jp1 = c_min(j + 1, P.nqiacralpha);
  const Real tmp1 = P.gamxinflu(i, j, luindex, ilh) +
                    delx * P.dqiacrratioinv * (P.gamxinflu(ip1, j, luindex, ilh) - P.gamxinflu(i, j, luindex, ilh));
  const Real tmp2 = P.gamxinflu(i, jp1, luindex, ilh) +
                    delx * P.dqiacrratioinv * (P.gamxinflu(ip1, jp1, luindex, ilh) - P.gamxinflu(i, jp1, luindex, ilh));
  return tmp1 + dely * P.dqiacralphainv * (tmp2 - tmp1);
}
template <class Real>
CPPWRF_HD inline Real nssl_gmoi_interp(const NsslParams<Real>& P, Real tmp) {
  const int i = int(P.dgami * tmp);
  const Real del = tmp - P.dgam * i;
  return P.gmoi(i) + (P.gmoi(i + 1) - P.gmoi(i)) * del * P.dgami;
}
template <class Real>
CPPWRF_HD inline Real delbk(const NsslParams<Real>& P, Real bb, Real nu, Real mu, int k) {
  const Real x1 = nssl_gmoi_interp(P, Real((Real(1.0) + nu) / mu));
  const Real x2 = nssl_gmoi_interp(P, Real((Real(2.0) + nu) / mu));
  const Real x3 = nssl_gmoi_interp(P, Real((Real(1.0) + Real(2.0) * bb + k + nu) / mu));
  return (pow(x1 / x2, Real(2.0) * bb + k) * x3) / x1;
}
template <class Real>
CPPWRF_HD inline Real delabk(const NsslParams<Real>& P, Real ba, Real bb, Real nua, Real nub, Real mua, Real mub, int k) {
  const Real g1pnua = nssl_gmoi_interp(P, Real((Real(1.) + nua) / mua));
  const Real g2pnua = nssl_gmoi_interp(P, Real((Real(2.) + nua) / mua));
  const Real g1pbapnua = nssl_gmoi_interp(P, Real((Real(1.) + ba + nua) / mua));
  const Real g1pnub = nssl_gmoi_interp(P, Real((Real(1.) + nub) / mub));
  const Real g2pnub = nssl_gmoi_interp(P, Real((2 + nub) / mub));
  const Real g1pbbpk = nssl_gmoi_interp(P, Real((Real(1.) + bb + k + nub) / mub));
  return (Real(2.) * pow(g1pnua / g2pnua, ba) * g1pbapnua * pow(g1pnub / g2pnub, bb + k) * g1pbbpk) / (g1pnua * g1pnub);
}

template <class Real>
CPPWRF_HD inline void fallout1d(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int nor, int na, Real dtp, Real dtfrac, int jgs, Real* vt_p, Real* a_p, Real* db1_p, int ia, int id, Real* xfall_p, Real* dtz1_p, int ixcol) {
  const int ng1 = 1;
  FA<Real> vt = mkfa<Real>(vt_p, 1, (nz + 1), 1, nx);
  FA<Real> a = mkfa<Real>(a_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor), 1, na);
  FA<Real> db1 = mkfa<Real>(db1_p, 1, nx, 1, (nz + 1));
  FA<Real> xfall = mkfa<Real>(xfall_p, 1, nx, 1, ny, 1, na);
  FA<Real> dtz1 = mkfa<Real>(dtz1_p, 1, (nz + 1), 1, nx, 0, 1);
  int ngt = 0;  // 4860
  Real cmax = 0;  // 4871
  int ix = 0;  // 4877
  int jy = 0;  // 4877
  int kz = 0;  // 4877
  int n = 0;  // 4877
  int k = 0;  // 4877
  int iv1 = 0;  // 4878
  int iv2 = 0;  // 4878
  Real tmp = 0;  // 4879
  int imn = 0;  // 4880
  int imx = 0;  // 4880
  int kmn = 0;  // 4880
  int kmx = 0;  // 4880
  auto qtmp1 = W.template arr<Real>(1, (nz + 1));  // 4881 
  int ixb = 0;  // 4885
  int jyb = 0;  // 4885
  int kzb = 0;  // 4885
  int ixe = 0;  // 4886
  int jye = 0;  // 4886
  int kze = 0;  // 4886
  jy = 1;  // 4890
  iv1 = 0;  // 4892
  iv2 = 0;  // 4893
  imn = nx;  // 4895
  imx = 1;  // 4896
  kmn = nz;  // 4897
  kmx = 1;  // 4898
  cmax = NR(0.0);  // 4900
  kzb = 1;  // 4902
  kze = nz;  // 4903
  ixb = ixcol;  // 4905
  ixe = ixcol;  // 4906
  ix = ixcol;  // 4907
  qtmp1((nz + 1)) = NR(0.0);  // 4909
  for (kz = kzb; kz <= kze; ++kz) {  // 4911
    if ((id == 1)) {  // 4915
      qtmp1(kz) = ((a(ix, jgs, kz, ia) * vt(kz, ix)) * db1(ix, kz));  // 4916
    } else {  // 4917
      qtmp1(kz) = (a(ix, jgs, kz, ia) * vt(kz, ix));  // 4918
    }  // 4919
    if ((a(ix, jgs, kz, ia) != NR(0.0))) {  // 4921
      kmn = c_min(kz, kmn);  // 4924
      kmx = c_max(kz, kmx);  // 4925
    }  // 4926
  }  // 4928
  kmn = c_max(1, (kmn - 1));  // 4930
  if ((kmn == 1)) {  // 4937
    kz = 1;  // 4939
    xfall(ix, jy, ia) = (xfall(ix, jy, ia) + ((a(ix, jgs, kz, ia) * vt(kz, ix)) * dtfrac));  // 4941
  }  // 4944
  for (kz = 1; kz <= nz; ++kz) {  // 4946
    a(ix, jgs, kz, ia) = (a(ix, jgs, kz, ia) + ((dtp * dtz1(kz, ix, id)) * ((qtmp1((kz + 1)) - qtmp1(kz)))));  // 4948
  }  // 4950
  return;  // 4953

}
template <class Real>
CPPWRF_HD inline void calczgr1d(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int nor, int na, Real* a_p, int ixe, int kze, Real* z_p, Real* db_p, int jgs, int ipconc, Real alpha, int l, int ln, Real qmin, Real xvmn, Real xvmx, int lvol, Real rho_qx, int ixcol) {
  const int norz = 3;  // 4967
  FA<Real> a = mkfa<Real>(a_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-nor) + 1), (nz + nor), 1, na);
  FA<Real> z = mkfa<Real>(z_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (nz + nor), P.lr, P.lhab);
  FA<Real> db = mkfa<Real>(db_p, 1, nx, 1, (nz + 1));
  int ngt = 0;  // 4965
  int ix = 0;  // 4984
  int jy = 0;  // 4984
  int kz = 0;  // 4984
  Real vr = 0;  // 4985
  Real qr = 0;  // 4985
  Real nrx = 0;  // 4985
  Real rd = 0;  // 4985
  Real xv = 0;  // 4985
  Real g1 = 0;  // 4985
  Real zx = 0;  // 4985
  Real chw = 0;  // 4985
  Real xdn = 0;  // 4985
  Real ynu = 0;  // 4985
  jy = jgs;  // 4988
  ix = ixcol;  // 4989
  if (((((l == P.lh) || (l == P.lhl)) || (((l == P.lr) && (P.imurain == 1)))) || (((l == P.ls) && (P.imusnow == 1))))) {  // 4991
    for (kz = 1; kz <= kze; ++kz) {  // 4995
      if (((a(ix, jy, kz, l) > qmin) && (a(ix, jy, kz, ln) > NR(1.0e-15)))) {  // 4999
        if ((lvol > 1)) {  // 5001
          if ((a(ix, jy, kz, lvol) > NR(0.0))) {  // 5002
            xdn = ((db(ix, kz) * a(ix, jy, kz, l)) / a(ix, jy, kz, lvol));  // 5003
            xdn = c_min(NR(900.0), c_max(P.hdnmn, xdn));  // 5004
          } else {  // 5005
            xdn = rho_qx;  // 5006
          }  // 5007
        } else {  // 5008
          xdn = rho_qx;  // 5009
        }  // 5010
        if ((l == P.lr)) { xdn = NR(1000.0); }  // 5012
        qr = a(ix, jy, kz, l);  // 5014
        xv = ((db(ix, kz) * a(ix, jy, kz, l)) / ((xdn * a(ix, jy, kz, ln))));  // 5015
        chw = a(ix, jy, kz, ln);  // 5016
        if (((xv < xvmn) || (xv > xvmx))) {  // 5018
          xv = c_min(xvmx, c_max(xvmn, xv));  // 5019
          chw = ((db(ix, kz) * a(ix, jy, kz, l)) / ((xv * xdn)));  // 5020
        }  // 5021
        g1 = (((((NR(6.0) + alpha)) * ((NR(5.0) + alpha))) * ((NR(4.0) + alpha))) / (((((NR(3.0) + alpha)) * ((NR(2.0) + alpha))) * ((NR(1.0) + alpha)))));  // 5023
        zx = ((((g1 * ipow<2>(db(ix, kz))) * (a(ix, jy, kz, l))) * a(ix, jy, kz, l)) / chw);  // 5025
        z(ix, kz, l) = (zx * ipow<2>(((NR(6.0) / ((P.pi * NR(1000.0)))))));  // 5027
      } else {  // 5034
        z(ix, kz, l) = NR(0.0);  // 5036
      }  // 5038
    }  // 5040
  } else if (((((l == P.ls) && (P.imusnow == 3))) || (((l == P.lr) && (P.imurain == 3))))) {  // 5042
    xdn = rho_qx;  // 5044
    if ((l == P.ls)) { ynu = P.snu; }  // 5045
    if ((l == P.lr)) { ynu = P.rnu; }  // 5046
    for (kz = 1; kz <= kze; ++kz) {  // 5048
      if (((a(ix, jy, kz, l) > qmin) && (a(ix, jy, kz, ln) > NR(1.0e-15)))) {  // 5050
        vr = ((db(ix, kz) * a(ix, jy, kz, l)) / ((xdn * a(ix, jy, kz, ln))));  // 5052
        z(ix, kz, l) = ((((NR(3.6) * ((ynu + NR(2.0)))) * a(ix, jy, kz, ln)) * ipow<2>(vr)) / ((ynu + NR(1.0))));  // 5054
      } else {  // 5058
        z(ix, kz, l) = NR(0.0);  // 5060
      }  // 5062
    }  // 5065
  }  // 5067
  return;  // 5069

}
template <class Real>
CPPWRF_HD inline void calcnfromz1d(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int nor, int na, Real* a_p, Real* t0_p, int ixe, int kze, Real* z0_p, Real* db_p, int jgs, int ipconc, Real alpha, int l, int ln, Real qmin, Real xvmn, Real xvmx, Real* t1_p, int lvol, Real rho_qx, int infall, int ixcol) {

  FA<Real> a = mkfa<Real>(a_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-nor) + 1), (nz + nor), 1, na);
  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-nor) + 1), (nz + nor));
  FA<Real> z0 = mkfa<Real>(z0_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (nz + nor), P.lr, P.lhab);
  FA<Real> db = mkfa<Real>(db_p, 1, nx, 1, (nz + 1));
  FA<Real> t1 = mkfa<Real>(t1_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-nor) + 1), (nz + nor));
  int ngt = 0;  // 5088
  int ix = 0;  // 5110
  int jy = 0;  // 5110
  int kz = 0;  // 5110
  double vr = 0;  // 5111
  double qr = 0;  // 5111
  double nrx = 0;  // 5111
  double rd = 0;  // 5111
  double g1 = 0;  // 5111
  double zx = 0;  // 5111
  double chw = 0;  // 5111
  double z = 0;  // 5111
  double znew = 0;  // 5111
  double zt = 0;  // 5111
  double zxt = 0;  // 5111
  Real xv = 0;  // 5112
  Real xdn = 0;  // 5112
  int ndbz = 0;  // 5113
  int nmwgt = 0;  // 5113
  int nnwgt = 0;  // 5113
  int nwlessthanz = 0;  // 5113
  ndbz = 0;  // 5115
  nmwgt = 0;  // 5116
  nnwgt = 0;  // 5117
  nwlessthanz = 0;  // 5118
  jy = jgs;  // 5122
  ix = ixcol;  // 5123
  if ((((l == P.lh) || (l == P.lhl)) || (((l == P.lr) && (P.imurain == 1))))) {  // 5125
    g1 = (((((NR(6.0) + alpha)) * ((NR(5.0) + alpha))) * ((NR(4.0) + alpha))) / (((((NR(3.0) + alpha)) * ((NR(2.0) + alpha))) * ((NR(1.0) + alpha)))));  // 5127
    for (kz = 1; kz <= kze; ++kz) {  // 5130
      if ((t0(ix, jy, kz) > NR(0.0))) {  // 5133
        if ((lvol > 1)) {  // 5135
          if ((a(ix, jy, kz, lvol) > NR(0.0))) {  // 5136
            xdn = ((db(ix, kz) * a(ix, jy, kz, l)) / a(ix, jy, kz, lvol));  // 5137
            xdn = c_min(NR(900.0), c_max(P.hdnmn, xdn));  // 5138
          } else {  // 5139
            xdn = rho_qx;  // 5140
          }  // 5141
        } else {  // 5142
          xdn = rho_qx;  // 5143
        }  // 5144
        if ((l == P.lr)) { xdn = NR(1000.0); }  // 5146
        qr = a(ix, jy, kz, l);  // 5148
        xv = ((db(ix, kz) * a(ix, jy, kz, l)) / ((xdn * a(ix, jy, kz, ln))));  // 5149
        chw = a(ix, jy, kz, ln);  // 5150
        if (((xv < xvmn) || (xv > xvmx))) {  // 5152
          xv = c_min(xvmx, c_max(xvmn, xv));  // 5153
          chw = ((db(ix, kz) * a(ix, jy, kz, l)) / ((xv * xdn)));  // 5154
        }  // 5155
        zx = ((((g1 * ipow<2>(db(ix, kz))) * (a(ix, jy, kz, l))) * a(ix, jy, kz, l)) / chw);  // 5157
        z = (zx * ipow<2>(((NR(6.0) / ((P.pi * NR(1000.0)))))));  // 5158
        if (((((z > t0(ix, jy, kz)) && (z > NR(0.0))) && (t0(ix, jy, kz) > z0(ix, kz, l))))) {  // 5161
          zx = (t0(ix, jy, kz) / (ipow<2>(((NR(6.0) / ((P.pi * NR(1000.0))))))));  // 5164
          nrx = ((((g1 * ipow<2>(db(ix, kz))) * (a(ix, jy, kz, l))) * a(ix, jy, kz, l)) / zx);  // 5166
          if ((infall == 3)) {  // 5167
            if ((nrx > a(ix, jy, kz, ln))) {  // 5168
              ndbz = (ndbz + 1);  // 5169
              if ((t1(ix, jy, kz) < ndbz)) { nwlessthanz = (nwlessthanz + 1); }  // 5170
            } else {  // 5171
              nnwgt = (nnwgt + 1);  // 5172
            }  // 5173
            a(ix, jy, kz, ln) = c_max(NR(nrx), a(ix, jy, kz, ln));  // 5174
          } else {  // 5175
            if (((nrx > a(ix, jy, kz, ln)) && (t1(ix, jy, kz) > a(ix, jy, kz, ln)))) {  // 5176
              if ((nrx < t1(ix, jy, kz))) {  // 5177
                ndbz = (ndbz + 1);  // 5178
              } else {  // 5179
                nmwgt = (nmwgt + 1);  // 5180
                if ((t1(ix, jy, kz) < ndbz)) { nwlessthanz = (nwlessthanz + 1); }  // 5181
              }  // 5182
            } else {  // 5183
              nnwgt = (nnwgt + 1);  // 5184
            }  // 5185
            a(ix, jy, kz, ln) = c_max(c_min(NR(nrx), t1(ix, jy, kz)), a(ix, jy, kz, ln));  // 5187
          }  // 5188
        } else {  // 5190
          if (((t1(ix, jy, kz) > 0) && (a(ix, jy, kz, ln) > 0))) {  // 5191
            if ((t1(ix, jy, kz) > a(ix, jy, kz, ln))) {  // 5192
              nmwgt = (nmwgt + 1);  // 5193
            } else {  // 5194
              nnwgt = (nnwgt + 1);  // 5195
            }  // 5196
          }  // 5197
          a(ix, jy, kz, ln) = c_max(t1(ix, jy, kz), a(ix, jy, kz, ln));  // 5198
          nrx = a(ix, jy, kz, ln);  // 5199
        }  // 5203
      } else {  // 5206
        if (((t1(ix, jy, kz) > 0) && (a(ix, jy, kz, ln) > 0))) {  // 5207
          if ((t1(ix, jy, kz) > a(ix, jy, kz, ln))) {  // 5208
            nmwgt = (nmwgt + 1);  // 5209
          } else {  // 5210
            nnwgt = (nnwgt + 1);  // 5211
          }  // 5212
        }  // 5213
      }  // 5214
    }  // 5216
  }  // 5257
  return;  // 5259

}
template <class Real>
CPPWRF_HD inline void setvtz(const NsslParams<Real>& P, Work W, int ngscnt, Real* qx_p, Real* qxmin_p, Real* qxw_p, Real* cx_p, Real* rho0_p, Real* rhovt_p, Real* xdia_p, Real* cno_p, Real* cnostmp_p, Real* xmas_p, Real* vtxbar_p, Real* xdn_p, Real* xvmn0_p, Real* xvmx0_p, Real* xv_p, Real* cdx_p, Real* cdxgs_p, int ipconc1, int ndebug1, int ngs, int nz, int* igs_p, int* kgs_p, Real* fadvisc_p, Real cwmasn, Real cwmasx, Real cwradn, Real* cnina_p, Real cimna, Real cimxa, int itype1a, int itype2a, Real* temcg_p, int infdo, Real* alpha_p, Real* axx_p, Real* bxx_p, int ildo) {
  const Real rho00 = NR(1.225);  // 6410
  const Real bta1 = NR(0.6);
  const Real cnit = NR(1.0e-02);
  FA<Real> qx = mkfa<Real>(qx_p, 1, ngs, P.lv, P.lhab);
  FA<Real> qxmin = mkfa<Real>(qxmin_p, P.lc, P.lhab);
  FA<Real> qxw = mkfa<Real>(qxw_p, 1, ngs, P.ls, P.lhab);
  FA<Real> cx = mkfa<Real>(cx_p, 1, ngs, P.lc, P.lhab);
  FA<Real> rho0 = mkfa<Real>(rho0_p, 1, ngs);
  FA<Real> rhovt = mkfa<Real>(rhovt_p, 1, ngs);
  FA<Real> xdia = mkfa<Real>(xdia_p, 1, ngs, P.lc, P.lhab, 1, 3);
  FA<Real> cno = mkfa<Real>(cno_p, P.lc, P.lhab);
  FA<Real> cnostmp = mkfa<Real>(cnostmp_p, 1, ngs);
  FA<Real> xmas = mkfa<Real>(xmas_p, 1, ngs, P.lc, P.lhab);
  FA<Real> vtxbar = mkfa<Real>(vtxbar_p, 1, ngs, P.lc, P.lhab, 1, 3);
  FA<Real> xdn = mkfa<Real>(xdn_p, 1, ngs, P.lc, P.lhab);
  FA<Real> xvmn0 = mkfa<Real>(xvmn0_p, P.lc, P.lhab);
  FA<Real> xvmx0 = mkfa<Real>(xvmx0_p, P.lc, P.lhab);
  FA<Real> xv = mkfa<Real>(xv_p, 1, ngs, P.lc, P.lhab);
  FA<Real> cdx = mkfa<Real>(cdx_p, P.lc, P.lhab);
  FA<Real> cdxgs = mkfa<Real>(cdxgs_p, 1, ngs, P.lc, P.lhab);
  FA<int> igs = mkfa<int>(igs_p, 1, ngs);
  FA<int> kgs = mkfa<int>(kgs_p, 1, ngs);
  FA<Real> fadvisc = mkfa<Real>(fadvisc_p, 1, ngs);
  FA<Real> cnina = mkfa<Real>(cnina_p, 1, ngs);
  FA<Real> temcg = mkfa<Real>(temcg_p, 1, ngs);
  FA<Real> alpha = mkfa<Real>(alpha_p, 1, ngs, P.lc, P.lhab);
  FA<Real> axx = mkfa<Real>(axx_p, 1, ngs, P.lh, P.lhab);
  FA<Real> bxx = mkfa<Real>(bxx_p, 1, ngs, P.lh, P.lhab);
  int ngs0 = 0;  // 6333
  Real cwc1 = 0;  // 6354
  Real fsw = 0;  // 6358
  Real swmasmx = 0;  // 6374
  Real dtmp = 0;  // 6374
  Real cd = 0;  // 6375
  Real cwc0 = 0;  // 6376
  auto cwch = W.template arr<Real>(1, ngscnt);  // 6377 
  auto cwchl = W.template arr<Real>(1, ngscnt);  // 6377 
  Real cwchtmp = 0;  // 6378
  Real cwchltmp = 0;  // 6378
  Real xnutmp = 0;  // 6378
  Real pii = 0;  // 6379
  Real cimasx = 0;  // 6380
  Real cimasn = 0;  // 6380
  Real cwrad = 0;  // 6382
  Real vr = 0;  // 6383
  Real rnux = 0;  // 6383
  Real alp = 0;  // 6384
  Real ccimx = 0;  // 6386
  int mgs = 0;  // 6388
  Real arx = 0;  // 6390
  Real frx = 0;  // 6390
  Real vtrain = 0;  // 6390
  Real fw = 0;  // 6390
  Real fwlo = 0;  // 6391
  Real fwhi = 0;  // 6391
  Real rfwdiff = 0;  // 6391
  Real ar = 0;  // 6392
  Real br = 0;  // 6392
  Real cs = 0;  // 6392
  Real ds = 0;  // 6392
  Real gr = 0;  // 6395
  Real rwrad = 0;  // 6396
  Real rwdia = 0;  // 6396
  Real mwfac = 0;  // 6397
  int il = 0;  // 6398
  Real x = 0;  // 6406
  Real y = 0;  // 6406
  Real tmp = 0;  // 6406
  Real del = 0;  // 6406
  Real aax = 0;  // 6407
  Real bbx = 0;  // 6407
  Real delrho = 0;  // 6407
  int indxr = 0;  // 6408
  Real mwt = 0;  // 6409
  Real nwt = 0;  // 6409
  Real zwt = 0;  // 6409
  int i = 0;  // 6411
  Real xvbarmax = 0;  // 6412
  int l1 = 0;  // 6414
  int l2 = 0;  // 6414
  fwlo = NR(0.2);  // 6424
  fwhi = NR(0.4);  // 6425
  rfwdiff = (NR(1.0) / ((fwhi - fwlo)));  // 6426
  pii = P.piinv;  // 6429
  arx = NR(10.0);  // 6431
  frx = NR(516.575);  // 6432
  ar = NR(841.99666);  // 6434
  br = NR(0.8);  // 6435
  gr = NR(9.8);  // 6436
  cs = NR(12.42);  // 6438
  ds = NR(0.42);  // 6439
  if ((ildo == 0)) {  // 6441
    l1 = P.lc;  // 6442
    l2 = P.lhab;  // 6443
  } else {  // 6444
    l1 = ildo;  // 6445
    l2 = ildo;  // 6446
  }  // 6447
  cwchtmp = pow((((((NR(3.0) + P.dnu(P.lh))) * ((NR(2.0) + P.dnu(P.lh)))) * ((NR(1.0) + P.dnu(P.lh))))), ((-(NR(1.0) / NR(3.0)))));  // 6461
  if ((P.lhl > 1)) {  // 6466
    cwchltmp = pow((((((NR(3.0) + P.dnu(P.lhl))) * ((NR(2.0) + P.dnu(P.lhl)))) * ((NR(1.0) + P.dnu(P.lhl))))), ((-(NR(1.0) / NR(3.0)))));  // 6468
  }  // 6472
  if ((P.ipconc <= 5)) {  // 6474
    for (int i_0 = 1; i_0 <= ngscnt; ++i_0) cwch(i_0) = cwchtmp;  // 6475
    if ((P.lhl > 1)) { for (int i_0 = 1; i_0 <= ngscnt; ++i_0) cwchl(i_0) = cwchltmp; }  // 6476
  } else {  // 6477
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6478
      if (((P.lh > 1) && (((ildo == 0) || (ildo == P.lh))))) {  // 6480
        if ((qx(mgs, P.lh) > qxmin(P.lh))) {  // 6481
          cwch(mgs) = pow((((((NR(3.0) + alpha(mgs, P.lh))) * ((NR(2.0) + alpha(mgs, P.lh)))) * ((NR(1.0) + alpha(mgs, P.lh))))), ((-(NR(1.0) / NR(3.0)))));  // 6483
        } else {  // 6488
          cwch(mgs) = cwchtmp;  // 6489
        }  // 6490
      }  // 6491
      if (((P.lhl > 1) && (((ildo == 0) || (ildo == P.lhl))))) {  // 6492
        if ((qx(mgs, P.lhl) > qxmin(P.lhl))) {  // 6493
          cwchl(mgs) = pow((((((NR(3.0) + alpha(mgs, P.lhl))) * ((NR(2.0) + alpha(mgs, P.lhl)))) * ((NR(1.0) + alpha(mgs, P.lhl))))), ((-(NR(1.0) / NR(3.0)))));  // 6495
        } else {  // 6500
          cwchl(mgs) = cwchltmp;  // 6501
        }  // 6502
      }  // 6503
    }  // 6505
  }  // 6507
  cimasn = c_min(P.cimas0, NR(6.88e-13));  // 6510
  cimasx = NR(1.0e-8);  // 6511
  ccimx = NR(5000.0e3);  // 6512
  cwc1 = (NR(6.0) / ((P.pi * NR(1000.0))));  // 6514
  cwc0 = pii;  // 6515
  mwfac = pow(NR(6.0), ((NR(1.0) / NR(3.0))));  // 6516
  // if (ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set scale diameter'
  // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set cloud water variables'
  if (((ildo == 0) || (ildo == P.lc))) {  // 6532
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6534
      xv(mgs, P.lc) = NR(0.0);  // 6535
      if ((qx(mgs, P.lc) > qxmin(P.lc))) {  // 6537
        if ((P.ipconc >= 2)) {  // 6539
          if ((cx(mgs, P.lc) > P.cxmin)) {  // 6540
            xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), cwmasn), cwmasx);  // 6541
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6543
          } else {  // 6544
            cx(mgs, P.lc) = c_max(P.cxmin, ((rho0(mgs) * qx(mgs, P.lc)) / cwmasx));  // 6545
            xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), cwmasn), cwmasx);  // 6546
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6547
          }  // 6549
        } else {  // 6550
          if ((P.ipconc < 2)) {  // 6551
            cx(mgs, P.lc) = ((rho0(mgs) * P.ccn) / rho00);  // 6552
          }  // 6553
          if (((qx(mgs, P.lc) > qxmin(P.lc)) && (cx(mgs, P.lc) > NR(0.01)))) {  // 6554
            xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), (xdn(mgs, P.lc) * P.xvmn(P.lc))), (xdn(mgs, P.lc) * P.xvmx(P.lc)));  // 6555
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6559
            cx(mgs, P.lc) = ((qx(mgs, P.lc) * rho0(mgs)) / xmas(mgs, P.lc));  // 6560
          } else if (((qx(mgs, P.lc) > qxmin(P.lc)) && (cx(mgs, P.lc) <= NR(1.0e-9)))) {  // 6562
            cx(mgs, P.lc) = c_max(P.cxmin, ((rho0(mgs) * qx(mgs, P.lc)) / cwmasx));  // 6563
            xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), cwmasn), cwmasx);  // 6564
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6566
          } else if (((qx(mgs, P.lc) > qxmin(P.lc)) && (cx(mgs, P.lc) <= NR(0.01)))) {  // 6568
            xmas(mgs, P.lc) = ((((xdn(mgs, P.lc) * NR(4.0)) * P.pi) / NR(3.0)) * NR(hexd(0x3CA203AF9EE75617ull)));  // 6569
            cx(mgs, P.lc) = ((rho0(mgs) * qx(mgs, P.lc)) / xmas(mgs, P.lc));  // 6570
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6571
          } else {  // 6573
            xmas(mgs, P.lc) = cwmasn;  // 6574
            xv(mgs, P.lc) = (xmas(mgs, P.lc) / NR(1000.0));  // 6575
          }  // 6577
        }  // 6578
        xdia(mgs, P.lc, 1) = pow(((xmas(mgs, P.lc) * cwc1)), ((NR(1.0) / NR(3.0))));  // 6587
        xdia(mgs, P.lc, 2) = ipow<2>(xdia(mgs, P.lc, 1));  // 6588
        xdia(mgs, P.lc, 3) = xdia(mgs, P.lc, 1);  // 6589
        cwrad = (NR(0.5) * xdia(mgs, P.lc, 1));  // 6590
        if ((fadvisc(mgs) > NR(0.0))) {  // 6591
          vtxbar(mgs, P.lc, 1) = (((((NR(2.0) * gr) * xdn(mgs, P.lc)) * (ipow<2>(cwrad)))) / ((NR(9.0) * fadvisc(mgs))));  // 6592
        } else {  // 6595
          vtxbar(mgs, P.lc, 1) = NR(0.0);  // 6596
        }  // 6597
      } else {  // 6600
        xmas(mgs, P.lc) = cwmasn;  // 6601
        xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 6602
        if ((qx(mgs, P.lc) <= NR(0.0))) { cx(mgs, P.lc) = NR(0.0); }  // 6603
        if ((P.ipconc <= 1)) { cx(mgs, P.lc) = NR(0.01); }  // 6604
        xdia(mgs, P.lc, 1) = (NR(2.0) * cwradn);  // 6605
        xdia(mgs, P.lc, 2) = (NR(4.0) * ipow<2>(cwradn));  // 6606
        xdia(mgs, P.lc, 3) = xdia(mgs, P.lc, 1);  // 6607
        vtxbar(mgs, P.lc, 1) = NR(0.0);  // 6608
      }  // 6610
    }  // 6612
  }  // 6614
  // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set cip'
  if (((P.li > 1) && (((ildo == 0) || (ildo == P.li))))) {  // 6628
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6629
      xdn(mgs, P.li) = NR(900.0);  // 6630
      if ((P.ipconc == 0)) {  // 6631
        cx(mgs, P.li) = cnina(mgs);  // 6633
        if ((cimna > NR(1.0))) {  // 6634
          cx(mgs, P.li) = c_max(cimna, cx(mgs, P.li));  // 6635
        }  // 6636
        if ((cimxa > NR(1.0))) {  // 6637
          cx(mgs, P.li) = c_min(cimxa, cx(mgs, P.li));  // 6638
        }  // 6639
        if (((itype1a >= 1) || (itype2a >= 1))) {  // 6641
          cx(mgs, P.li) = c_max(cx(mgs, P.li), ((qx(mgs, P.li) * rho0(mgs)) / cimasx));  // 6642
          cx(mgs, P.li) = c_min(cx(mgs, P.li), ((qx(mgs, P.li) * rho0(mgs)) / cimasn));  // 6643
        }  // 6644
        cx(mgs, P.li) = c_max(NR(1.0e-20), cx(mgs, P.li));  // 6646
      } else if ((P.ipconc >= 1)) {  // 6650
        if ((qx(mgs, P.li) > qxmin(P.li))) {  // 6651
          cx(mgs, P.li) = c_max(cx(mgs, P.li), ((qx(mgs, P.li) * rho0(mgs)) / cimasx));  // 6652
          cx(mgs, P.li) = c_min(cx(mgs, P.li), ((qx(mgs, P.li) * rho0(mgs)) / cimasn));  // 6653
        }  // 6655
      }  // 6656
      if ((qx(mgs, P.li) > qxmin(P.li))) {  // 6658
        xmas(mgs, P.li) = c_max(((qx(mgs, P.li) * rho0(mgs)) / cx(mgs, P.li)), cimasn);  // 6659
        if ((xmas(mgs, P.li) > NR(0.0))) {  // 6666
          xdia(mgs, P.li, 1) = (NR(0.1871) * (pow(xmas(mgs, P.li), (NR(0.3429)))));  // 6672
          xdia(mgs, P.li, 3) = (NR(0.1871) * (pow(xmas(mgs, P.li), (NR(0.3429)))));  // 6673
        }  // 6678
        if ((P.ipconc >= 0)) {  // 6683
          xv(mgs, P.li) = (xmas(mgs, P.li) / xdn(mgs, P.li));  // 6686
          tmp = (((NR(47.6273) * rhovt(mgs))) / ((pow(((((NR(1.0) + P.cinu)) / xv(mgs, P.li))), NR(0.18333)) * P.gfcinu1)));  // 6710
          vtxbar(mgs, P.li, 2) = (tmp * P.gfcinu1p18);  // 6712
          vtxbar(mgs, P.li, 1) = ((tmp * P.gfcinu2p18) / ((NR(1.0) + P.cinu)));  // 6713
          vtxbar(mgs, P.li, 3) = vtxbar(mgs, P.li, 1);  // 6714
          xdia(mgs, P.li, 2) = ipow<2>(xdia(mgs, P.li, 1));  // 6720
        } else {  // 6722
          xdia(mgs, P.li, 1) = c_max(xdia(mgs, P.li, 1), NR(10.0e-6));  // 6723
          xdia(mgs, P.li, 1) = c_min(xdia(mgs, P.li, 1), NR(1000.0e-6));  // 6724
          vtxbar(mgs, P.li, 1) = ((NR(4.942e4)) * (pow(xdia(mgs, P.li, 1), (NR(1.4150)))));  // 6725
          xdn(mgs, P.li) = NR(900.0);  // 6727
          xdia(mgs, P.li, 2) = ipow<2>(xdia(mgs, P.li, 1));  // 6728
          vtxbar(mgs, P.li, 1) = (vtxbar(mgs, P.li, 1) * rhovt(mgs));  // 6729
          xv(mgs, P.li) = (xmas(mgs, P.li) / xdn(mgs, P.li));  // 6730
        }  // 6731
      } else {  // 6732
        xmas(mgs, P.li) = NR(1.0e-13);  // 6733
        if ((qx(mgs, P.li) <= NR(0.0))) { cx(mgs, P.li) = NR(0.0); }  // 6734
        xdn(mgs, P.li) = NR(900.0);  // 6735
        xdia(mgs, P.li, 1) = NR(1.0e-7);  // 6736
        xdia(mgs, P.li, 2) = (NR(1.0e-14));  // 6737
        xdia(mgs, P.li, 3) = NR(1.0e-7);  // 6738
        vtxbar(mgs, P.li, 1) = NR(0.0);  // 6739
      }  // 6742
    }  // 6752
  }  // 6754
  if (((ildo == 0) || (ildo == P.lr))) {  // 6763
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6764
      if ((qx(mgs, P.lr) > qxmin(P.lr))) {  // 6765
        if ((P.ipconc >= 3)) {  // 6770
          xv(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((xdn(mgs, P.lr) * c_max(NR(1.0e-11), cx(mgs, P.lr)))));  // 6771
          xvbarmax = P.xvmx(P.lr);  // 6772
          xvbarmax = (P.xvmx(P.lr) / ((ipow<3>(((NR(4.0) + alpha(mgs, P.lr)))) / (((((NR(3.0) + alpha(mgs, P.lr))) * ((NR(2.0) + alpha(mgs, P.lr)))) * ((NR(1.0) + alpha(mgs, P.lr))))))));  // 6783
          if ((xv(mgs, P.lr) > xvbarmax)) {  // 6789
            xv(mgs, P.lr) = xvbarmax;  // 6790
            cx(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((xvbarmax * xdn(mgs, P.lr))));  // 6791
          } else if ((xv(mgs, P.lr) < P.xvmn(P.lr))) {  // 6792
            xv(mgs, P.lr) = P.xvmn(P.lr);  // 6793
            cx(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((P.xvmn(P.lr) * xdn(mgs, P.lr))));  // 6794
          }  // 6795
          xmas(mgs, P.lr) = (xv(mgs, P.lr) * xdn(mgs, P.lr));  // 6798
          xdia(mgs, P.lr, 3) = pow(((xmas(mgs, P.lr) * cwc1)), ((NR(1.0) / NR(3.0))));  // 6799
          xdia(mgs, P.lr, 1) = pow(((((NR(6.0) * pii) * xv(mgs, P.lr)) / (((((alpha(mgs, P.lr) + NR(3.0))) * ((alpha(mgs, P.lr) + NR(2.0)))) * ((alpha(mgs, P.lr) + NR(1.0))))))), ((NR(1.0) / NR(3.0))));  // 6804
        } else {  // 6812
          xdia(mgs, P.lr, 1) = pow((((qx(mgs, P.lr) * rho0(mgs)) / (((P.pi * xdn(mgs, P.lr)) * cno(P.lr))))), (NR(0.25)));  // 6813
          xmas(mgs, P.lr) = ((xdn(mgs, P.lr) * ((P.pi / NR(6.0)))) * ipow<3>(xdia(mgs, P.lr, 1)));  // 6815
          xdia(mgs, P.lr, 3) = pow(((xmas(mgs, P.lr) * cwc1)), ((NR(1.0) / NR(3.0))));  // 6816
          cx(mgs, P.lr) = (cno(P.lr) * xdia(mgs, P.lr, 1));  // 6817
          xv(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((xdn(mgs, P.lr) * cx(mgs, P.lr))));  // 6818
        }  // 6819
      } else {  // 6820
        xdia(mgs, P.lr, 1) = NR(1.0e-9);  // 6821
        xdia(mgs, P.lr, 3) = NR(1.0e-9);  // 6822
        xmas(mgs, P.lr) = ((xdn(mgs, P.lr) * ((P.pi / NR(6.0)))) * ipow<3>(xdia(mgs, P.lr, 1)));  // 6823
      }  // 6825
      xdia(mgs, P.lr, 2) = ipow<2>(xdia(mgs, P.lr, 1));  // 6826
    }  // 6828
  }  // 6830
  if (((P.ls > 1) && (((ildo == 0) || (ildo == P.ls))))) {  // 6836
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6838
      if ((qx(mgs, P.ls) > qxmin(P.ls))) {  // 6839
        if ((P.ipconc >= 4)) {  // 6840
          xmas(mgs, P.ls) = ((rho0(mgs) * qx(mgs, P.ls)) / (c_max(NR(1.0e-9), cx(mgs, P.ls))));  // 6842
          swmasmx = NR(13.7e-6);  // 6843
          xv(mgs, P.ls) = ((rho0(mgs) * qx(mgs, P.ls)) / ((xdn(mgs, P.ls) * c_max(NR(1.0e-9), cx(mgs, P.ls)))));  // 6864
          dtmp = pow((((xv(mgs, P.ls) * cwc0) * NR(6.0))), ((NR(1.0) / NR(3.0))));  // 6865
          xdia(mgs, P.ls, 1) = dtmp;  // 6868
          if (((xv(mgs, P.ls) < P.xvmn(P.ls)) && (P.isnowdens == 1))) {  // 6870
            xv(mgs, P.ls) = c_max(P.xvmn(P.ls), xv(mgs, P.ls));  // 6871
            xmas(mgs, P.ls) = (xv(mgs, P.ls) * xdn(mgs, P.ls));  // 6872
            cx(mgs, P.ls) = ((rho0(mgs) * qx(mgs, P.ls)) / (xmas(mgs, P.ls)));  // 6873
            xdia(mgs, P.ls, 1) = pow((((xv(mgs, P.ls) * cwc0) * NR(6.0))), ((NR(1.0) / NR(3.0))));  // 6874
          }  // 6875
          if ((xv(mgs, P.ls) > (P.xvmx(P.ls) * c_max(NR(1.0), (NR(100.0) / c_min(NR(100.0), xdn(mgs, P.ls))))))) {  // 6877
            xv(mgs, P.ls) = c_min(P.xvmx(P.ls), c_max(P.xvmn(P.ls), xv(mgs, P.ls)));  // 6878
            xmas(mgs, P.ls) = (NR(0.106214) * pow(xv(mgs, P.ls), ((NR(2.0) / NR(3.0)))));  // 6879
            cx(mgs, P.ls) = ((rho0(mgs) * qx(mgs, P.ls)) / (xmas(mgs, P.ls)));  // 6880
            xdn(mgs, P.ls) = (NR(0.0346159) * sqrt((cx(mgs, P.ls) / ((qx(mgs, P.ls) * rho0(mgs))))));  // 6881
            xdia(mgs, P.ls, 1) = sqrt((xmas(mgs, P.ls) / NR(0.069)));  // 6882
          }  // 6883
          xdia(mgs, P.ls, 3) = xdia(mgs, P.ls, 1);  // 6885
        } else {  // 6887
          xdia(mgs, P.ls, 1) = pow((((qx(mgs, P.ls) * rho0(mgs)) / (((P.pi * xdn(mgs, P.ls)) * cnostmp(mgs))))), (NR(0.25)));  // 6888
          cx(mgs, P.ls) = (cnostmp(mgs) * xdia(mgs, P.ls, 1));  // 6890
          xv(mgs, P.ls) = ((rho0(mgs) * qx(mgs, P.ls)) / ((xdn(mgs, P.ls) * cx(mgs, P.ls))));  // 6891
          xdia(mgs, P.ls, 3) = pow((((xv(mgs, P.ls) * cwc0) * NR(6.0))), ((NR(1.0) / NR(3.0))));  // 6892
        }  // 6893
      } else {  // 6894
        xdia(mgs, P.ls, 1) = NR(1.0e-9);  // 6895
        xdia(mgs, P.ls, 3) = NR(1.0e-9);  // 6896
        cx(mgs, P.ls) = NR(0.0);  // 6897
      }  // 6903
      xdia(mgs, P.ls, 2) = ipow<2>(xdia(mgs, P.ls, 1));  // 6904
    }  // 6907
  }  // 6909
  if (((P.lh > 1) && (((ildo == 0) || (ildo == P.lh))))) {  // 6917
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6919
      if ((qx(mgs, P.lh) > qxmin(P.lh))) {  // 6920
        if ((P.ipconc >= 5)) {  // 6921
          xv(mgs, P.lh) = ((rho0(mgs) * qx(mgs, P.lh)) / ((xdn(mgs, P.lh) * c_max(NR(1.0e-9), cx(mgs, P.lh)))));  // 6923
          xmas(mgs, P.lh) = (xv(mgs, P.lh) * xdn(mgs, P.lh));  // 6924
          if (((xv(mgs, P.lh) < P.xvmn(P.lh)) || (xv(mgs, P.lh) > P.xvmx(P.lh)))) {  // 6926
            xv(mgs, P.lh) = c_min(P.xvmx(P.lh), c_max(P.xvmn(P.lh), xv(mgs, P.lh)));  // 6927
            xmas(mgs, P.lh) = (xv(mgs, P.lh) * xdn(mgs, P.lh));  // 6928
            cx(mgs, P.lh) = ((rho0(mgs) * qx(mgs, P.lh)) / (xmas(mgs, P.lh)));  // 6929
          }  // 6930
          xdia(mgs, P.lh, 3) = pow((((xv(mgs, P.lh) * NR(6.0)) * pii)), ((NR(1.0) / NR(3.0))));  // 6932
          xdia(mgs, P.lh, 1) = (cwch(mgs) * xdia(mgs, P.lh, 3));  // 6934
        } else {  // 6939
          xdia(mgs, P.lh, 1) = pow((((qx(mgs, P.lh) * rho0(mgs)) / (((P.pi * xdn(mgs, P.lh)) * cno(P.lh))))), (NR(0.25)));  // 6940
          cx(mgs, P.lh) = (cno(P.lh) * xdia(mgs, P.lh, 1));  // 6942
          xv(mgs, P.lh) = c_max(P.xvmn(P.lh), ((rho0(mgs) * qx(mgs, P.lh)) / ((xdn(mgs, P.lh) * cx(mgs, P.lh)))));  // 6943
          xdia(mgs, P.lh, 3) = pow((((xv(mgs, P.lh) * NR(6.0)) / P.pi)), ((NR(1.0) / NR(3.0))));  // 6944
        }  // 6945
      } else {  // 6946
        xdia(mgs, P.lh, 1) = NR(1.0e-9);  // 6947
        xdia(mgs, P.lh, 3) = NR(1.0e-9);  // 6948
      }  // 6949
      xdia(mgs, P.lh, 2) = ipow<2>(xdia(mgs, P.lh, 1));  // 6950
    }  // 6953
  }  // 6955
  if (((P.lhl > 1) && (((ildo == 0) || (ildo == P.lhl))))) {  // 6963
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 6965
      if ((qx(mgs, P.lhl) > qxmin(P.lhl))) {  // 6966
        if ((P.ipconc >= 5)) {  // 6967
          xv(mgs, P.lhl) = ((rho0(mgs) * qx(mgs, P.lhl)) / ((xdn(mgs, P.lhl) * c_max(NR(1.0e-9), cx(mgs, P.lhl)))));  // 6969
          xmas(mgs, P.lhl) = (xv(mgs, P.lhl) * xdn(mgs, P.lhl));  // 6970
          if (((xv(mgs, P.lhl) < P.xvmn(P.lhl)) || (xv(mgs, P.lhl) > P.xvmx(P.lhl)))) {  // 6973
            xv(mgs, P.lhl) = c_min(P.xvmx(P.lhl), c_max(P.xvmn(P.lhl), xv(mgs, P.lhl)));  // 6974
            xmas(mgs, P.lhl) = (xv(mgs, P.lhl) * xdn(mgs, P.lhl));  // 6975
            cx(mgs, P.lhl) = ((rho0(mgs) * qx(mgs, P.lhl)) / (xmas(mgs, P.lhl)));  // 6976
          }  // 6977
          xdia(mgs, P.lhl, 3) = pow((((xv(mgs, P.lhl) * NR(6.0)) / P.pi)), ((NR(1.0) / NR(3.0))));  // 6979
          xdia(mgs, P.lhl, 1) = (cwchl(mgs) * xdia(mgs, P.lhl, 3));  // 6981
        } else {  // 6987
          xdia(mgs, P.lhl, 1) = pow((((qx(mgs, P.lhl) * rho0(mgs)) / (((P.pi * xdn(mgs, P.lhl)) * cno(P.lhl))))), (NR(0.25)));  // 6988
          cx(mgs, P.lhl) = (cno(P.lhl) * xdia(mgs, P.lhl, 1));  // 6990
          xv(mgs, P.lhl) = c_max(P.xvmn(P.lhl), ((rho0(mgs) * qx(mgs, P.lhl)) / ((xdn(mgs, P.lhl) * cx(mgs, P.lhl)))));  // 6991
          xdia(mgs, P.lhl, 3) = pow((((xv(mgs, P.lhl) * NR(6.0)) / P.pi)), ((NR(1.0) / NR(3.0))));  // 6992
        }  // 6993
      } else {  // 6994
        xdia(mgs, P.lhl, 1) = NR(1.0e-9);  // 6995
        xdia(mgs, P.lhl, 3) = NR(1.0e-9);  // 6996
      }  // 6997
      xdia(mgs, P.lhl, 2) = ipow<2>(xdia(mgs, P.lhl, 1));  // 6998
    }  // 7001
  }  // 7003
  // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set terminal velocities'
  if (((ildo == 0) || (ildo == P.lr))) {  // 7026
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7027
      if ((qx(mgs, P.lr) > qxmin(P.lr))) {  // 7028
        if ((P.ipconc < 3)) {  // 7029
          vtxbar(mgs, P.lr, 1) = (((P.rainfallfac * (((ar * P.gf4br) / NR(6.0)))) * (pow(xdia(mgs, P.lr, 1), br))) * rhovt(mgs));  // 7030
        } else {  // 7032
          alp = alpha(mgs, P.lr);  // 7041
          vtxbar(mgs, P.lr, 1) = ((rhovt(mgs) * arx) * ((NR(1.0) - pow(((NR(1.0) + (frx * xdia(mgs, P.lr, 1)))), (((-alp) - NR(4.0)))))));  // 7043
          if (((infdo >= 1) && (P.rssflg == 1))) {  // 7045
            vtxbar(mgs, P.lr, 2) = ((rhovt(mgs) * arx) * ((NR(1.0) - pow(((NR(1.0) + (frx * xdia(mgs, P.lr, 1)))), (((-alp) - NR(1.0)))))));  // 7046
          } else {  // 7047
            vtxbar(mgs, P.lr, 2) = vtxbar(mgs, P.lr, 1);  // 7048
          }  // 7049
          if (((infdo >= 2) && (P.rssflg == 1))) {  // 7051
            vtxbar(mgs, P.lr, 3) = ((rhovt(mgs) * arx) * ((NR(1.0) - pow(((NR(1.0) + (frx * xdia(mgs, P.lr, 1)))), (((-alp) - NR(7.0)))))));  // 7052
          } else {  // 7053
            vtxbar(mgs, P.lr, 3) = vtxbar(mgs, P.lr, 1);  // 7054
          }  // 7055
        }  // 7152
      } else {  // 7153
        vtxbar(mgs, P.lr, 1) = NR(0.0);  // 7154
        vtxbar(mgs, P.lr, 2) = NR(0.0);  // 7155
      }  // 7156
    }  // 7157
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set rain vt'
  }  // 7160
  if (((P.ls > 1) && (((ildo == 0) || (ildo == P.ls))))) {  // 7166
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7167
      if ((qx(mgs, P.ls) > qxmin(P.ls))) {  // 7168
        if ((P.ipconc >= 4)) {  // 7169
          vtxbar(mgs, P.ls, 1) = ((NR(11.9495) * rhovt(mgs)) * pow((xv(mgs, P.ls)), (NR(0.14))));  // 7178
          vtxbar(mgs, P.ls, 2) = ((NR(7.02909) * rhovt(mgs)) * pow((xv(mgs, P.ls)), (NR(0.14))));  // 7193
          if ((infdo >= 2)) {  // 7204
            vtxbar(mgs, P.ls, 3) = ((NR(13.3436) * rhovt(mgs)) * pow((xv(mgs, P.ls)), (NR(0.14))));  // 7208
          }  // 7213
        } else {  // 7221
          vtxbar(mgs, P.ls, 1) = (((((cs * P.gf4ds) / NR(6.0))) * (pow(xdia(mgs, P.ls, 1), ds))) * rhovt(mgs));  // 7222
          vtxbar(mgs, P.ls, 2) = vtxbar(mgs, P.ls, 1);  // 7223
        }  // 7224
      } else {  // 7225
        vtxbar(mgs, P.ls, 1) = NR(0.0);  // 7226
      }  // 7227
    }  // 7236
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set snow vt'
  }  // 7239
  if (((P.lh > 1) && (((ildo == 0) || (ildo == P.lh))))) {  // 7246
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7248
      vtxbar(mgs, P.lh, 1) = NR(0.0);  // 7249
      if ((qx(mgs, P.lh) > qxmin(P.lh))) {  // 7250
        cd = cdx(P.lh);  // 7251
        indxr = (int((((xdn(mgs, P.lh) - NR(50.0))) / NR(100.0))) + 1);  // 7268
        indxr = c_min(P.ngdnmm, c_max(1, indxr));  // 7269
        delrho = c_max(NR(0.0), (NR(0.01) * ((xdn(mgs, P.lh) - P.mmgraupvt(indxr, 1)))));  // 7272
        if ((indxr < P.ngdnmm)) {  // 7273
          axx(mgs, P.lh) = (P.mmgraupvt(indxr, 2) + (delrho * ((P.mmgraupvt((indxr + 1), 2) - P.mmgraupvt(indxr, 2)))));  // 7275
          bxx(mgs, P.lh) = (P.mmgraupvt(indxr, 3) + (delrho * ((P.mmgraupvt((indxr + 1), 3) - P.mmgraupvt(indxr, 3)))));  // 7276
        } else {  // 7279
          axx(mgs, P.lh) = P.mmgraupvt(indxr, 2);  // 7280
          bxx(mgs, P.lh) = P.mmgraupvt(indxr, 3);  // 7281
        }  // 7282
        aax = axx(mgs, P.lh);  // 7284
        bbx = bxx(mgs, P.lh);  // 7285
        cd = c_max(NR(0.45), c_min(NR(1.2), (NR(0.45) + ((NR(0.55) * ((NR(800.0) - c_max(P.hdnmn, c_min(NR(800.0), xdn(mgs, P.lh)))))) / ((NR(800.0) - NR(170.0)))))));  // 7287
        cdxgs(mgs, P.lh) = cd;  // 7297
        tmp = ((NR(4.0) + alpha(mgs, P.lh)) + bbx);  // 7310
        i = int((P.dgami * (tmp)));  // 7311
        del = (tmp - (P.dgam * i));  // 7312
        x = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7313
        tmp = (NR(4.0) + alpha(mgs, P.lh));  // 7315
        i = int((P.dgami * (tmp)));  // 7316
        del = (tmp - (P.dgam * i));  // 7317
        y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7318
        vtxbar(mgs, P.lh, 1) = ((((rhovt(mgs) * aax) * pow(xdia(mgs, P.lh, 1), bbx)) * x) / y);  // 7329
      }  // 7344
    }  // 7345
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set hail vt'
  }  // 7348
  if (((P.lhl > 1) && (((ildo == 0) || (ildo == P.lhl))))) {  // 7355
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7357
      vtxbar(mgs, P.lhl, 1) = NR(0.0);  // 7358
      if ((qx(mgs, P.lhl) > qxmin(P.lhl))) {  // 7359
        indxr = (int((((xdn(mgs, P.lhl) - NR(50.0))) / NR(100.0))) + 1);  // 7372
        indxr = c_min(P.ngdnmm, c_max(1, indxr));  // 7373
        delrho = c_max(NR(0.0), (NR(0.01) * ((xdn(mgs, P.lhl) - P.mmgraupvt(indxr, 1)))));  // 7376
        if ((indxr < P.ngdnmm)) {  // 7377
          axx(mgs, P.lhl) = (P.mmgraupvt(indxr, 2) + (delrho * ((P.mmgraupvt((indxr + 1), 2) - P.mmgraupvt(indxr, 2)))));  // 7379
          bxx(mgs, P.lhl) = (P.mmgraupvt(indxr, 3) + (delrho * ((P.mmgraupvt((indxr + 1), 3) - P.mmgraupvt(indxr, 3)))));  // 7380
        } else {  // 7383
          axx(mgs, P.lhl) = P.mmgraupvt(indxr, 2);  // 7384
          bxx(mgs, P.lhl) = P.mmgraupvt(indxr, 3);  // 7385
        }  // 7386
        aax = axx(mgs, P.lhl);  // 7388
        bbx = bxx(mgs, P.lhl);  // 7389
        cd = c_max(NR(0.45), c_min(NR(1.2), (NR(0.45) + ((NR(0.55) * ((NR(800.0) - c_max(P.hldnmn, c_min(NR(800.0), xdn(mgs, P.lhl)))))) / ((NR(800.0) - NR(170.0)))))));  // 7391
        cdxgs(mgs, P.lhl) = cd;  // 7400
        tmp = ((NR(4.0) + alpha(mgs, P.lhl)) + bbx);  // 7411
        i = int((P.dgami * (tmp)));  // 7412
        del = (tmp - (P.dgam * i));  // 7413
        x = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7414
        tmp = (NR(4.0) + alpha(mgs, P.lhl));  // 7416
        i = int((P.dgami * (tmp)));  // 7417
        del = (tmp - (P.dgam * i));  // 7418
        y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7419
        vtxbar(mgs, P.lhl, 1) = ((((rhovt(mgs) * aax) * pow((xdia(mgs, P.lhl, 1)), bbx)) * x) / y);  // 7427
      }  // 7438
    }  // 7439
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set hail vt'
  }  // 7442
  if ((infdo >= 1)) {  // 7445
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7449
      if (((ildo == 0) || (ildo == P.lc))) {  // 7450
        vtxbar(mgs, P.lc, 2) = vtxbar(mgs, P.lc, 1);  // 7451
      }  // 7452
    }  // 7466
    for (il = P.lg; il <= P.lhab; ++il) {  // 7470
      if (((ildo == 0) || (ildo == il))) {  // 7471
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7473
          if ((qx(mgs, il) > qxmin(il))) {  // 7474
            if (((((il == P.lh) && (P.hssflg == 1))) || ((((P.lhl > 1) && (il == P.lhl)) && (P.hlssflg == 1))))) {  // 7475
              if ((il == P.lh)) {  // 7480
                aax = axx(mgs, P.lh);  // 7498
                bbx = bxx(mgs, P.lh);  // 7499
              } else if (((P.lhl > 1) && (il == P.lhl))) {  // 7505
                aax = axx(mgs, P.lhl);  // 7520
                bbx = bxx(mgs, P.lhl);  // 7521
              }  // 7527
              tmp = ((NR(1.0) + alpha(mgs, il)) + bbx);  // 7538
              i = int((P.dgami * (tmp)));  // 7539
              del = (tmp - (P.dgam * i));  // 7540
              x = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7541
              tmp = (NR(1.0) + alpha(mgs, il));  // 7543
              i = int((P.dgami * (tmp)));  // 7544
              del = (tmp - (P.dgam * i));  // 7545
              y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7546
              if (((il == P.lh) || (il == P.lhl))) {  // 7548
                if ((((il == P.lh) && (P.icdx > 0)))) {  // 7549
                  vtxbar(mgs, il, 2) = ((((rhovt(mgs) * aax) * pow(xdia(mgs, il, 1), bbx)) * x) / y);  // 7554
                } else if ((((il == P.lhl) && (P.icdxhl > 0)))) {  // 7557
                  vtxbar(mgs, il, 2) = ((((rhovt(mgs) * aax) * pow(xdia(mgs, il, 1), bbx)) * x) / y);  // 7562
                } else {  // 7564
                  aax = P.ax(il);  // 7565
                  vtxbar(mgs, il, 2) = (((rhovt(mgs) * P.ax(il)) * ((pow(xdia(mgs, il, 1), P.bx(il)) * x))) / y);  // 7566
                }  // 7567
                if ((infdo >= 2)) {  // 7574
                  tmp = ((NR(7.0) + alpha(mgs, il)) + bbx);  // 7576
                  i = int((P.dgami * (tmp)));  // 7577
                  del = (tmp - (P.dgam * i));  // 7578
                  x = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7579
                  tmp = (NR(7.0) + alpha(mgs, il));  // 7581
                  i = int((P.dgami * (tmp)));  // 7582
                  del = (tmp - (P.dgam * i));  // 7583
                  y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 7584
                  vtxbar(mgs, il, 3) = ((rhovt(mgs) * (((aax * pow((xdia(mgs, il, 1)), bbx)) * x))) / y);  // 7586
                  if (((!(((vtxbar(mgs, il, 1) > (-NR(1.0))) && (vtxbar(mgs, il, 1) < NR(250.0))))) || (!(((vtxbar(mgs, il, 3) > (-NR(1.0))) && (vtxbar(mgs, il, 3) < NR(250.0))))))) {  // 7590
                    // write(0,*) 'Setvtz: problem with vtxbar1/3: ',il,vtxbar(mgs,il,1),vtxbar(mgs,il,3),aax,bbx,x,y
                    // write(0,*) 'q, number, diam1,3(mm) = ', qx(mgs,il),cx(mgs,il),1000.*xdia(mgs,il,1),1000.*xdia(mgs,il,3)
                  }  // 7595
                }  // 7598
                // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set hail vt3'
              } else {  // 7602
                vtxbar(mgs, il, 2) = ((rhovt(mgs) * (((P.ax(il) * pow(xdia(mgs, il, 1), P.bx(il))) * x))) / y);  // 7603
                if ((infdo >= 2)) {  // 7607
                  vtxbar(mgs, il, 3) = ((rhovt(mgs) * (((aax * pow(((NR(1.0) / xdia(mgs, il, 1))), ((-bbx)))) * gamma_sp(((NR(7.0) + alpha(mgs, il)) + bbx))))) / gamma_sp((NR(7.0) + alpha(mgs, il))));  // 7608
                }  // 7613
                // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set hail vt4'
              }  // 7617
            } else {  // 7633
              vtxbar(mgs, il, 2) = sqrt((((((xdn(mgs, il) * xdia(mgs, il, 1)) * P.pi) * gr)) / (((NR(3.0) * cdx(il)) * c_max(NR(0.05), rho0(mgs))))));  // 7634
              vtxbar(mgs, il, 3) = vtxbar(mgs, il, 1);  // 7637
              // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set graupel vt5'
            }  // 7642
          } else {  // 7643
            vtxbar(mgs, il, 2) = NR(0.0);  // 7644
            // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set graupel vt6'
          }  // 7648
        }  // 7649
        // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set graupel vt7'
      }  // 7653
    }  // 7654
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set graupel vt8'
    // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: Set graupel vt9'
  }  // 7672
  // if ( ndebug1 .gt. 0 ) write(0,*) 'SETVTZ: END OF ROUTINE'
  return;  // 7696

}
template <class Real>
CPPWRF_HD inline void ziegfall1d(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int nor, int norz, int na, Real dtp, int jgs, int ixcol, Real* xvt_p, Real* rhovtzx_p, Real* an_p, Real* dn_p, int ipconc0, Real* t0_p, Real* t7_p, Real cwmasn, Real cwmasx, Real cwradn, Real* qxmin_p, Real* xdnmx_p, Real* xdnmn_p, Real* cdx_p, Real* cno_p, Real* xdn0_p, Real* xvmn_p, Real* xvmx_p, int ngs, Real* qx_p, Real* qxw_p, Real* cx_p, Real* xv_p, Real* vtxbar_p, Real* xmas_p, Real* xdn_p, Real* xdia_p, Real* vx_p, Real* alpha_p, Real* zx_p, int* igs_p, int* kgs_p, Real* rho0_p, Real* temcg_p, Real* temg_p, Real* rhovt_p, Real* cwnc_p, Real* cinc_p, Real* fadvisc_p, Real* cwdia_p, Real* cipmas_p, Real* cnina_p, Real* cimas_p, Real* cnostmp_p, int infdo, int ildo, double timesetvt) {
  const Real c1r = NR(19.0);  // 7832
  const Real c2r = NR(0.6);  // 7832
  const Real c3r = NR(1.8);  // 7832
  const Real c4r = NR(17.0);  // 7832
  const Real c1h = NR(5.5);  // 7833
  const Real c2h = NR(0.7);  // 7833
  const Real c3h = NR(4.5);  // 7833
  const Real c4h = NR(8.5);  // 7833
  const Real c1hl = NR(3.7);  // 7834
  const Real c2hl = NR(0.3);  // 7834
  const Real c3hl = NR(9.0);  // 7834
  const Real c4hl = NR(6.5);  // 7834
  const Real c5hl = NR(1.0);  // 7834
  const Real c6hl = NR(6.5);  // 7834
  const int ng1 = 1;
  const int ndebugzf = 0;
  const int kstag = 1;
  FA<Real> xvt = mkfa<Real>(xvt_p, 1, (nz + 1), 1, nx, 1, 3, P.lc, P.lhab);
  FA<Real> rhovtzx = mkfa<Real>(rhovtzx_p, 1, nz, 1, nx);
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  FA<Real> t7 = mkfa<Real>(t7_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  FA<Real> qxmin = mkfa<Real>(qxmin_p, P.lc, P.lhab);
  FA<Real> xdnmx = mkfa<Real>(xdnmx_p, P.lc, P.lhab);
  FA<Real> xdnmn = mkfa<Real>(xdnmn_p, P.lc, P.lhab);
  FA<Real> cdx = mkfa<Real>(cdx_p, P.lc, P.lhab);
  FA<Real> cno = mkfa<Real>(cno_p, P.lc, P.lhab);
  FA<Real> xdn0 = mkfa<Real>(xdn0_p, P.lc, P.lhab);
  FA<Real> xvmn = mkfa<Real>(xvmn_p, P.lc, P.lhab);
  FA<Real> xvmx = mkfa<Real>(xvmx_p, P.lc, P.lhab);
  FA<Real> qx = mkfa<Real>(qx_p, 1, ngs, P.lv, P.lhab);
  FA<Real> qxw = mkfa<Real>(qxw_p, 1, ngs, P.ls, P.lhab);
  FA<Real> cx = mkfa<Real>(cx_p, 1, ngs, P.lc, P.lhab);
  FA<Real> xv = mkfa<Real>(xv_p, 1, ngs, P.lc, P.lhab);
  FA<Real> vtxbar = mkfa<Real>(vtxbar_p, 1, ngs, P.lc, P.lhab, 1, 3);
  FA<Real> xmas = mkfa<Real>(xmas_p, 1, ngs, P.lc, P.lhab);
  FA<Real> xdn = mkfa<Real>(xdn_p, 1, ngs, P.lc, P.lhab);
  FA<Real> xdia = mkfa<Real>(xdia_p, 1, ngs, P.lc, P.lhab, 1, 3);
  FA<Real> vx = mkfa<Real>(vx_p, 1, ngs, P.li, P.lhab);
  FA<Real> alpha = mkfa<Real>(alpha_p, 1, ngs, P.lc, P.lhab);
  FA<Real> zx = mkfa<Real>(zx_p, 1, ngs, P.lr, P.lhab);
  FA<int> igs = mkfa<int>(igs_p, 1, ngs);
  FA<int> kgs = mkfa<int>(kgs_p, 1, ngs);
  FA<Real> rho0 = mkfa<Real>(rho0_p, 1, ngs);
  FA<Real> temcg = mkfa<Real>(temcg_p, 1, ngs);
  FA<Real> temg = mkfa<Real>(temg_p, 1, ngs);
  FA<Real> rhovt = mkfa<Real>(rhovt_p, 1, ngs);
  FA<Real> cwnc = mkfa<Real>(cwnc_p, 1, ngs);
  FA<Real> cinc = mkfa<Real>(cinc_p, 1, ngs);
  FA<Real> fadvisc = mkfa<Real>(fadvisc_p, 1, ngs);
  FA<Real> cwdia = mkfa<Real>(cwdia_p, 1, ngs);
  FA<Real> cipmas = mkfa<Real>(cipmas_p, 1, ngs);
  FA<Real> cnina = mkfa<Real>(cnina_p, 1, ngs);
  FA<Real> cimas = mkfa<Real>(cimas_p, 1, ngs);
  FA<Real> cnostmp = mkfa<Real>(cnostmp_p, 1, ngs);
  int ngt = 0;  // 7737
  Real dtz1 = 0;  // 7742
  int ix = 0;  // 7749
  int jy = 0;  // 7749
  int kz = 0;  // 7749
  int i = 0;  // 7749
  int j = 0;  // 7749
  int k = 0;  // 7749
  int il = 0;  // 7749
  int ngscnt = 0;  // 7761
  int mgs = 0;  // 7761
  auto cdxgs = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 7771 
  auto axx = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 7778 
  auto bxx = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 7778 
  Real cwccn0 = 0;  // 7790
  int nxmpb = 0;  // 7793
  int nzmpb = 0;  // 7793
  int nxz = 0;  // 7793
  int numgs = 0;  // 7793
  int inumgs = 0;  // 7793
  bool flag = false;  // 7823
  bool ldoliq = false;  // 7824
  Real chw = 0;  // 7827
  Real qr = 0;  // 7827
  Real z = 0;  // 7827
  Real rd = 0;  // 7827
  Real alp = 0;  // 7827
  Real z1 = 0;  // 7827
  Real g1 = 0;  // 7827
  Real vr = 0;  // 7827
  Real nrx = 0;  // 7827
  Real tmp = 0;  // 7827
  Real tmpc = 0;  // 7827
  Real tmpz = 0;  // 7827
  Real vtmax = 0;  // 7829
  Real xvbarmax = 0;  // 7830
  int l1 = 0;  // 7836
  int l2 = 0;  // 7836
  double dpt1 = 0;  // 7838
  double dpt2 = 0;  // 7838
  int ixb = 0;  // 7844
  int jyb = 0;  // 7844
  int kzb = 0;  // 7844
  int ixe = 0;  // 7845
  int jye = 0;  // 7845
  int kze = 0;  // 7845
  bool debug_mpi = false;  // 7847
  // if (ndebugzf .gt. 0 ) write(0,*) "ZIEGFALL1D: ENTERED SUBROUTINE"
  ldoliq = false;  // 7860
  for (il = P.ls; il <= P.lhab; ++il) {  // 7862
    ldoliq = (ldoliq || ((P.lliq(il) > 1)));  // 7863
  }  // 7864
  jy = jgs;  // 7899
  nxmpb = ixcol;  // 7900
  nzmpb = 1;  // 7901
  nxz = (1 * nz);  // 7902
  numgs = 1;  // 7904
  if ((ildo == 0)) {  // 7906
    l1 = P.lc;  // 7907
    l2 = P.lhab;  // 7908
  } else {  // 7909
    l1 = ildo;  // 7910
    l2 = ildo;  // 7911
  }  // 7912
  for (inumgs = 1; inumgs <= numgs; ++inumgs) {  // 7915
    ngscnt = 0;  // 7916
    for (kz = 1; kz <= nz; ++kz) {  // 7919
      for (ix = ixcol; ix <= ixcol; ++ix) {  // 7920
        flag = false;  // 7921
        for (il = l1; il <= l2; ++il) {  // 7923
          flag = (flag || ((an(ix, jy, kz, il) > NR(0.0))));  // 7924
        }  // 7925
        if (flag) {  // 7927
          ngscnt = (ngscnt + 1);  // 7930
          igs(ngscnt) = ix;  // 7931
          kgs(ngscnt) = kz;  // 7932
          if ((ngscnt == nz)) { goto L1100; }  // 7933
        }  // 7934
      }  // 7935
      nxmpb = 1;  // 7936
    }  // 7937
    L1100:;  // 7941
    if ((ngscnt == 0)) { goto L9998; }  // 7943
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7952
      rho0(mgs) = dn(igs(mgs), jy, kgs(mgs));  // 7954
      rhovt(mgs) = rhovtzx(kgs(mgs), ixcol);  // 7955
      temg(mgs) = t0(igs(mgs), jy, kgs(mgs));  // 7956
      temcg(mgs) = (temg(mgs) - P.tfr);  // 7957
    }  // 7961
    if (((P.lc > 1) && (((ildo == 0) || (ildo == P.lc))))) {  // 7964
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7965
        fadvisc(mgs) = ((P.advisc0 * ((NR(416.16) / ((temg(mgs) + NR(120.0)))))) * vpow(((temg(mgs) / NR(296.0))), (NR(1.5)), mgs - 1, ngscnt));  // 7966  // libmvec 2-lane pow (vectorised loop)
      }  // 7968
    }  // 7969
    if ((P.ipconc == 0)) {  // 7971
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7972
        cnina(mgs) = t7(igs(mgs), jgs, kgs(mgs));  // 7973
      }  // 7974
    }  // 7975
    if ((ildo > 0)) {  // 7978
      for (int i_2 = 1; i_2 <= 3; ++i_2) for (int i_0 = 1; i_0 <= ngs; ++i_0) vtxbar(i_0, ildo, i_2) = NR(0.0);  // 7979
    } else {  // 7980
      for (int i_2 = 1; i_2 <= 3; ++i_2) for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) vtxbar(i_0, i_1, i_2) = NR(0.0);  // 7981
    }  // 7982
    for (il = l1; il <= l2; ++il) {  // 7987
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 7988
        qx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), il), NR(0.0));  // 7989
      }  // 7990
    }  // 7991
    for (int i_0 = 1; i_0 <= ngs; ++i_0) cnostmp(i_0) = cno(P.ls);  // 7993
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) cx(i_0, i_1) = NR(0.0);  // 8005
    if ((((P.ipconc >= 1) && (P.li > 1)) && (((ildo == 0) || (ildo == P.li))))) {  // 8007
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8008
        cx(mgs, P.li) = c_max(an(igs(mgs), jy, kgs(mgs), P.lni), NR(0.0));  // 8009
      }  // 8010
    }  // 8011
    if ((((P.ipconc >= 2) && (P.lc > 1)) && (((ildo == 0) || (ildo == P.lc))))) {  // 8013
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8014
        cx(mgs, P.lc) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnc), NR(0.0));  // 8015
      }  // 8016
    }  // 8017
    if ((((P.ipconc >= 3) && (P.lr > 1)) && (((ildo == 0) || (ildo == P.lr))))) {  // 8019
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8020
        cx(mgs, P.lr) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnr), NR(0.0));  // 8021
      }  // 8022
    }  // 8023
    if ((((P.ipconc >= 4) && (P.ls > 1)) && (((ildo == 0) || (ildo == P.ls))))) {  // 8025
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8026
        cx(mgs, P.ls) = c_max(an(igs(mgs), jy, kgs(mgs), P.lns), NR(0.0));  // 8027
      }  // 8028
    }  // 8029
    if ((((P.ipconc >= 5) && (P.lh > 1)) && (((ildo == 0) || (ildo == P.lh))))) {  // 8031
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8032
        cx(mgs, P.lh) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnh), NR(0.0));  // 8033
      }  // 8034
    }  // 8035
    if ((((P.ipconc >= 5) && (P.lhl > 1)) && (((ildo == 0) || (ildo == P.lhl))))) {  // 8037
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8038
        cx(mgs, P.lhl) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnhl), NR(0.0));  // 8039
      }  // 8040
    }  // 8041
    for (il = l1; il <= l2; ++il) {  // 8044
      if (((P.lz(il) < 1) && (P.ln(il) > 1))) {  // 8045
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8046
          if (((cx(mgs, il) <= P.cxmin) || (qx(mgs, il) < qxmin(il)))) {  // 8047
            cx(mgs, il) = NR(0.0);  // 8048
            an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 8049
            qx(mgs, il) = NR(0.0);  // 8050
            an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 8051
            an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 8052
          }  // 8053
        }  // 8054
      }  // 8055
    }  // 8056
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8058
      xdn(mgs, P.lc) = xdn0(P.lc);  // 8059
      xdn(mgs, P.lr) = xdn0(P.lr);  // 8060
      xdn(mgs, P.li) = xdn0(P.li);  // 8063
      xdn(mgs, P.ls) = xdn0(P.ls);  // 8064
      xdn(mgs, P.lh) = xdn0(P.lh);  // 8065
      if ((P.lhl > 1)) { xdn(mgs, P.lhl) = xdn0(P.lhl); }  // 8066
    }  // 8067
    if ((P.ldovol && (((ildo == 0) || (ildo >= P.li))))) {  // 8072
      for (int i_1 = P.li; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) vx(i_0, i_1) = NR(0.0);  // 8074
      for (il = l1; il <= l2; ++il) {  // 8076
        if ((P.lvol(il) >= 1)) {  // 8078
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8080
            vx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), P.lvol(il)), NR(0.0));  // 8081
            if (((vx(mgs, il) > ((rho0(mgs) * qxmin(il)) * NR(1.0e-3))) && (qx(mgs, il) > qxmin(il)))) {  // 8082
              xdn(mgs, il) = c_min(xdnmx(il), c_max(xdnmn(il), ((rho0(mgs) * qx(mgs, il)) / vx(mgs, il))));  // 8083
            }  // 8084
          }  // 8085
        }  // 8087
      }  // 8089
    }  // 8091
    for (il = P.lg; il <= P.lhab; ++il) {  // 8093
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8094
        alpha(mgs, il) = P.dnu(il);  // 8095
      }  // 8096
    }  // 8097
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.lr) = P.alphar;  // 8100
    if (((P.ipconc >= 6) || (P.lzr > 1))) {  // 8138
      for (int i_1 = P.lr; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) zx(i_0, i_1) = NR(0.0);  // 8140
      for (il = l1; il <= l2; ++il) {  // 8143
        if ((P.lz(il) >= 1)) {  // 8145
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8147
            zx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), P.lz(il)), NR(0.0));  // 8148
          }  // 8149
        }  // 8152
      }  // 8154
    }  // 8156
    if ((P.ipconc >= 6)) {  // 8349
      for (il = P.lr; il <= P.lhab; ++il) {  // 8353
        if ((((P.lz(il) > 1) && (((ildo == 0) || (ildo == il)))) && ((!(((il == P.lr) && (P.imurain == 3))))))) {  // 8355
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8357
            if ((((P.iresetmoments == 1) || (P.iresetmoments == il)) || (P.iresetmoments == (-1)))) {  // 8359
              if ((zx(mgs, il) <= P.zxmin)) {  // 8360
                qx(mgs, il) = NR(0.0);  // 8361
                cx(mgs, il) = NR(0.0);  // 8362
                an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 8363
                an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 8364
                an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 8365
              } else if (((cx(mgs, il) <= P.cxmin) && (P.iresetmoments != (-1)))) {  // 8376
                zx(mgs, il) = NR(0.0);  // 8378
                qx(mgs, il) = NR(0.0);  // 8379
                an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 8380
                an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 8381
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 8382
              }  // 8383
            }  // 8384
            if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= P.cxmin))) {  // 8386
              zx(mgs, il) = NR(0.0);  // 8387
              cx(mgs, il) = NR(0.0);  // 8388
              an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 8389
              qx(mgs, il) = NR(0.0);  // 8390
              an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 8391
              an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 8392
              an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 8393
            }  // 8394
            if ((qx(mgs, il) > qxmin(il))) {  // 8396
              xv(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / ((xdn(mgs, il) * c_max(NR(1.0e-9), cx(mgs, il)))));  // 8398
              xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 8399
              if ((xv(mgs, il) < xvmn(il))) {  // 8401
                xv(mgs, il) = c_min(xvmx(il), c_max(xvmn(il), xv(mgs, il)));  // 8403
                xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 8404
                cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 8405
              }  // 8413
              if (((zx(mgs, il) > NR(0.0)) && (cx(mgs, il) <= NR(0.0)))) {  // 8415
                g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 8417
                z = zx(mgs, il);  // 8419
                qr = qx(mgs, il);  // 8420
                cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((6 * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 8421
                an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 8422
              } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) > P.cxmin))) {  // 8424
                g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 8426
                chw = cx(mgs, il);  // 8428
                qr = qx(mgs, il);  // 8429
                zx(mgs, il) = c_min((P.zxmin * NR(1.1)), (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((6 * qr)))) / ((chw * ipow<2>(((P.pi * xdn(mgs, il))))))));  // 8431
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 8432
              } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= NR(0.0)))) {  // 8433
                if ((qx(mgs, il) < NR(1.0e-8))) {  // 8439
                  qx(mgs, il) = NR(0.0);  // 8440
                  an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 8441
                  an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 8442
                } else {  // 8443
                  zx(mgs, il) = ((NR(1.0e-19) / NR(0.224)) * ipow<2>(((xdn0(P.lr) / xdn0(il)))));  // 8447
                  an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 8448
                  g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 8450
                  z = zx(mgs, il);  // 8452
                  qr = qx(mgs, il);  // 8453
                  cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((6 * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 8454
                  an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 8455
                }  // 8456
              }  // 8457
            }  // 8458
            if (((qx(mgs, il) > qxmin(il)) && (cx(mgs, il) > P.cxmin))) {  // 8460
              chw = cx(mgs, il);  // 8461
              qr = qx(mgs, il);  // 8462
              z = zx(mgs, il);  // 8463
              if ((zx(mgs, il) > NR(0.0))) {  // 8465
                rd = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((dn(igs(mgs), jy, kgs(mgs)) * qr)))));  // 8468
                alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rd))) - NR(1.0));  // 8470
                for (i = 1; i <= 10; ++i) {  // 8472
                  if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 8473
                  alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 8474
                  alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rd))) - NR(1.0));  // 8475
                  alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 8478
                }  // 8479
                if (((P.imaxdiaopt == 1) || (il != P.lr))) {  // 8485
                  xvbarmax = xvmx(il);  // 8486
                } else {  // 8489
                  xvbarmax = (xvmx(il) / ((ipow<3>(((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))))));  // 8490
                }  // 8491
                if ((xv(mgs, il) > xvbarmax)) {  // 8493
                  tmp = cx(mgs, il);  // 8494
                  xv(mgs, il) = c_min(xvbarmax, c_max(xvmn(il), xv(mgs, il)));  // 8495
                  xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 8496
                  cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 8497
                  if ((tmp < cx(mgs, il))) {  // 8498
                    g1 = ((((NR(36.0) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / ((((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il)))) * ipow<2>(P.pi))));  // 8499
                    tmpz = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / tmp);  // 8503
                    if ((tmpz > zx(mgs, il))) {  // 8504
                      tmpc = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / zx(mgs, il));  // 8506
                      cx(mgs, il) = c_max(cx(mgs, il), tmpc);  // 8507
                    }  // 8508
                    zx(mgs, il) = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / cx(mgs, il));  // 8509
                    an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 8511
                    chw = cx(mgs, il);  // 8513
                    qr = qx(mgs, il);  // 8514
                    z = zx(mgs, il);  // 8515
                    rd = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((rho0(mgs) * qr)))));  // 8517
                    alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rd))) - NR(1.0));  // 8518
                    for (i = 1; i <= 10; ++i) {  // 8520
                      if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 8521
                      alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 8522
                      alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rd))) - NR(1.0));  // 8523
                      alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 8525
                    }  // 8526
                  }  // 8529
                }  // 8530
                if ((((P.rescale_low_alpha || P.rescale_high_alpha)) && ((((alpha(mgs, il) <= P.alphamin) || (alp == P.alphamin)) || (alp == P.alphamax))))) {  // 8536
                  g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 8539
                  if (((P.rescale_low_alpha && (alp <= P.alphamin)) && (!(((il == P.lh) && (P.icvhl2h > 0)))))) {  // 8546
                    z1 = ((((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * (qr)) * qr) / chw);  // 8549
                    z = (z1 * ipow<2>(((NR(6.0) / ((P.pi * xdn(mgs, il)))))));  // 8550
                    zx(mgs, il) = z;  // 8551
                    an(igs(mgs), jy, kgs(mgs), P.lz(il)) = z;  // 8552
                  }  // 8553
                }  // 8554
              } else {  // 8555
              }  // 8556
            }  // 8557
          }  // 8558
        }  // 8560
      }  // 8562
    }  // 8566
    // if (ndebugzf .gt. 0 ) write(0,*)  'ZIEGFALL: call setvtz'
    setvtz(P, W, ngscnt, qx.p, qxmin.p, qxw.p, cx.p, rho0.p, rhovt.p, xdia.p, cno.p, cnostmp.p, xmas.p, vtxbar.p, xdn.p, xvmn.p, xvmx.p, xv.p, cdx.p, cdxgs.p, P.ipconc, ndebugzf, ngs, nz, igs.p, kgs.p, fadvisc.p, cwmasn, cwmasx, cwradn, cnina.p, P.cimn, P.cimx, P.itype1, P.itype2, temcg.p, infdo, alpha.p, axx.p, bxx.p, ildo);  // 8582
    for (il = l1; il <= l2; ++il) {  // 8594
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 8595
        vtmax = NR(150.0);  // 8597
        if (((vtxbar(mgs, il, 2) > vtxbar(mgs, il, 1)) || (((vtxbar(mgs, il, 1) > vtxbar(mgs, il, 3)) && (vtxbar(mgs, il, 3) > NR(0.0)))))) {  // 8600
          vtxbar(mgs, il, 1) = c_max(vtxbar(mgs, il, 1), vtxbar(mgs, il, 2));  // 8618
          vtxbar(mgs, il, 3) = c_max(vtxbar(mgs, il, 3), vtxbar(mgs, il, 1));  // 8619
        }  // 8621
        if ((((vtxbar(mgs, il, 1) > vtmax) || (vtxbar(mgs, il, 2) > vtmax)) || (vtxbar(mgs, il, 3) > vtmax))) {  // 8624
          vtxbar(mgs, il, 1) = c_min(vtmax, vtxbar(mgs, il, 1));  // 8639
          vtxbar(mgs, il, 2) = c_min(vtmax, vtxbar(mgs, il, 2));  // 8640
          vtxbar(mgs, il, 3) = c_min(vtmax, vtxbar(mgs, il, 3));  // 8641
        }  // 8644
        xvt(kgs(mgs), igs(mgs), 1, il) = vtxbar(mgs, il, 1);  // 8647
        xvt(kgs(mgs), igs(mgs), 2, il) = vtxbar(mgs, il, 2);  // 8648
        if ((infdo >= 2)) {  // 8649
          xvt(kgs(mgs), igs(mgs), 3, il) = vtxbar(mgs, il, 3);  // 8650
        } else {  // 8651
          xvt(kgs(mgs), igs(mgs), 3, il) = NR(0.0);  // 8652
        }  // 8653
      }  // 8657
    }  // 8658
    // if (ndebugzf .gt. 0 ) write(0,*)  'ZIEGFALL: COPIED FALL SPEEDS'
    L9998:;  // 8665
    // if (ndebugzf .gt. 0 ) write(0,*)  'ZIEGFALL: DONE WITH LOOP'
    if ((kz > (nz - 1))) {  // 8669
      goto L1200;  // 8670
    } else {  // 8671
      nzmpb = kz;  // 8672
    }  // 8673
    // if (ndebugzf .gt. 0 ) write(0,*) 'ZIEGFALL: SET NZMPB'
  }  // 8677
  // if (ndebugzf .gt. 0 ) write(0,*) 'ZIEGFALL: SET NXMPB'
  L1200:;  // 8681
  // if (ndebugzf .gt. 0 ) write(0,*) "ZIEGFALL: EXITING SUBROUTINE"
  return;  // 8691

}
template <class Real>
CPPWRF_HD inline void sediment1d(const NsslParams<Real>& P, Work W, Real dtp, int nx, int ny, int nz, Real* an_p, int na, int nor, int norz, Real* xfall_p, Real* dn_p, Real* dz3d_p, Real* dz3dinv_p, Real* t0_p, Real* t7_p, int infdo, int jslab, int its, int jts, double timesed1, double timesed2, double timesed3, double zmaxsed, double timesetvt) {
  const Real dz = NR(200.0);  // 4435
  const int ng1 = 1;
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> xfall = mkfa<Real>(xfall_p, 1, nx, 1, ny, 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> dz3d = mkfa<Real>(dz3d_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> dz3dinv = mkfa<Real>(dz3dinv_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t7 = mkfa<Real>(t7_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  int ngt = 0;  // 4413
  int jgs = 0;  // 4413
  int ia = 0;  // 4413
  int id = 0;  // 4414
  int ix = 0;  // 4433
  int jy = 0;  // 4433
  int kz = 0;  // 4433
  int ndfall = 0;  // 4433
  int n = 0;  // 4433
  int k = 0;  // 4433
  int il = 0;  // 4433
  int in = 0;  // 4433
  Real tmp = 0;  // 4434
  Real vtmax = 0;  // 4434
  Real dtptmp = 0;  // 4434
  Real dtfrac = 0;  // 4434
  double dummy = 0;  // 4441
  double dt1 = 0;  // 4442
  double dt2 = 0;  // 4442
  double dt3 = 0;  // 4442
  double dt4 = 0;  // 4442
  int ngs = 0;  // 4444
  int ngscnt = 0;  // 4445
  int mgs = 0;  // 4445
  int ipconc0 = 0;  // 4445
  Real cimasn = 0;  // 4476
  Real cimasx = 0;  // 4476
  int ixb = 0;  // 4481
  int jyb = 0;  // 4481
  int kzb = 0;  // 4481
  int ixe = 0;  // 4482
  int jye = 0;  // 4482
  int kze = 0;  // 4482
  int plo = 0;  // 4483
  int phi = 0;  // 4483
  auto db1 = W.template arr<Real>(1, nx, 1, (nz + 1));
  auto dtz1 = W.template arr<Real>(1, (nz + 1), 1, nx, 0, 1);
  auto dz2dinv = W.template arr<Real>(1, (nz + 1), 1, nx);
  auto db1inv = W.template arr<Real>(1, nx, 1, (nz + 1));
  auto rhovtzx = W.template arr<Real>(1, nz, 1, nx);
  auto xfall0 = W.template arr<Real>(1, nx, 1, ny);
  auto xvt = W.template arr<Real>(1, (nz + 1), 1, nx, 1, 3, P.lc, P.lhab);
  auto tmpn = W.template arr<Real>(1, nx, 1, ny, 1, nz);
  auto tmpn2 = W.template arr<Real>(1, nx, 1, ny, 1, nz);
  auto z = W.template arr<Real>(1, nx, 1, nz, P.lr, P.lhab);
  ngs = (nz + 3);  // 4492
  auto qx = W.template arr<Real>(1, ngs, P.lv, P.lhab);
  auto qxw = W.template arr<Real>(1, ngs, P.ls, P.lhab);
  auto cx = W.template arr<Real>(1, ngs, P.lc, P.lhab);
  auto xv = W.template arr<Real>(1, ngs, P.lc, P.lhab);
  auto vtxbar = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);
  auto xmas = W.template arr<Real>(1, ngs, P.lc, P.lhab);
  auto xdn = W.template arr<Real>(1, ngs, P.lc, P.lhab);
  auto xdia = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);
  auto vx = W.template arr<Real>(1, ngs, P.li, P.lhab);
  auto alpha = W.template arr<Real>(1, ngs, P.lc, P.lhab);
  auto zx = W.template arr<Real>(1, ngs, P.lr, P.lhab);
  auto hasmass = W.template arr<bool>(1, nx, (P.lc + 1), P.lhab);
  auto igs = W.template arr<int>(1, ngs);
  auto kgs = W.template arr<int>(1, ngs);
  auto rho0 = W.template arr<Real>(1, ngs);
  auto temcg = W.template arr<Real>(1, ngs);
  auto temg = W.template arr<Real>(1, ngs);
  auto rhovt = W.template arr<Real>(1, ngs);
  auto cwnc = W.template arr<Real>(1, ngs);
  auto cinc = W.template arr<Real>(1, ngs);
  auto fadvisc = W.template arr<Real>(1, ngs);
  auto cwdia = W.template arr<Real>(1, ngs);
  auto cipmas = W.template arr<Real>(1, ngs);
  auto cnina = W.template arr<Real>(1, ngs);
  auto cimas = W.template arr<Real>(1, ngs);
  auto cnostmp = W.template arr<Real>(1, ngs);
  kzb = 1;  // 4513
  kze = nz;  // 4514
  ixb = 1;  // 4516
  ixe = nx;  // 4517
  jy = 1;  // 4520
  jgs = jy;  // 4521
  // xvt(:,:,:,:) = 0.0: zero from W.arr  // 4528
  for (kz = kzb; kz <= kze; ++kz) {  // 4533
    for (ix = ixb; ix <= ixe; ++ix) {  // 4534
      db1(ix, kz) = dn(ix, jy, kz);  // 4535
      db1inv(ix, kz) = (NR(1.0) / dn(ix, jy, kz));  // 4536
      rhovtzx(kz, ix) = sqrt((P.rho00 * c_min((NR(1.0) / NR(0.05)), db1inv(ix, kz))));  // 4537
    }  // 4538
  }  // 4539
  for (kz = kzb; kz <= kze; ++kz) {  // 4541
    for (ix = ixb; ix <= ixe; ++ix) {  // 4542
      dtz1(kz, ix, 0) = dz3dinv(ix, jy, kz);  // 4543
      dtz1(kz, ix, 1) = (dz3dinv(ix, jy, kz) * db1inv(ix, kz));  // 4544
      dz2dinv(kz, ix) = dz3dinv(ix, jy, kz);  // 4545
    }  // 4546
  }  // 4547
  for (il = (P.lc + 1); il <= P.lhab; ++il) {  // 4558
    for (ix = ixb; ix <= ixe; ++ix) {  // 4559
    }  // 4561
  }  // 4562
  for (ix = ixb; ix <= ixe; ++ix) {  // 4570
    dummy = 0.0e0;  // 4572
    ziegfall1d(P, W, nx, ny, nz, nor, norz, na, dtp, jgs, ix, xvt.p, rhovtzx.p, an.p, dn.p, P.ipconc, t0.p, t7.p, P.cwmasn, P.cwmasx, P.cwradn, CP(P.qxmin.v), CP(P.xdnmx.v), CP(P.xdnmn.v), CP(P.cdx.v), CP(P.cno.v), CP(P.xdn0.v), CP(P.xvmn.v), CP(P.xvmx.v), ngs, qx.p, qxw.p, cx.p, xv.p, vtxbar.p, xmas.p, xdn.p, xdia.p, vx.p, alpha.p, zx.p, igs.p, kgs.p, rho0.p, temcg.p, temg.p, rhovt.p, cwnc.p, cinc.p, fadvisc.p, cwdia.p, cipmas.p, cnina.p, cimas.p, cnostmp.p, infdo, 0, timesetvt);  // call ziegfall1d
    for (il = P.lc; il <= P.lhab; ++il) {  // 4588
      if ((P.ido(il) == 0)) { continue; }  // 4589
      vtmax = NR(0.0);  // 4597
      for (kz = kzb; kz <= kze; ++kz) {  // 4599
        xvt(kz, ix, 1, il) = c_min(P.vtmaxsed, xvt(kz, ix, 1, il));  // 4602
        xvt(kz, ix, 2, il) = c_min(P.vtmaxsed, xvt(kz, ix, 2, il));  // 4603
        xvt(kz, ix, 3, il) = c_min(P.vtmaxsed, xvt(kz, ix, 3, il));  // 4604
        vtmax = c_max(vtmax, (xvt(kz, ix, 1, il) * dz2dinv(kz, ix)));  // 4606
        vtmax = c_max(vtmax, (xvt(kz, ix, 2, il) * dz2dinv(kz, ix)));  // 4607
        vtmax = c_max(vtmax, (xvt(kz, ix, 3, il) * dz2dinv(kz, ix)));  // 4608
      }  // 4620
      if ((vtmax == NR(0.0))) { continue; }  // 4622
      if (((dtp * vtmax) < NR(0.7))) {  // 4626
        ndfall = 1;  // 4627
      } else {  // 4628
        if ((dtp > NR(20.0))) {  // 4629
          ndfall = c_max(2, (int(((dtp * vtmax) / NR(0.7))) + 1));  // 4630
        } else {  // 4631
          ndfall = (1 + int(((dtp * vtmax) + NR(0.301))));  // 4632
        }  // 4633
      }  // 4634
      if ((ndfall > 1)) {  // 4636
        dtptmp = (dtp / NR(ndfall));  // 4637
      } else {  // 4640
        dtptmp = dtp;  // 4641
      }  // 4642
      dtfrac = (dtptmp / dtp);  // 4644
      for (n = 1; n <= ndfall; ++n) {  // 4647
        if ((((il >= P.lr) && (((P.linfall(il) == 3) || (P.linfall(il) == 4)))) && (P.ln(il) > 0))) {  // 4682
          calczgr1d(P, W, nx, ny, nz, nor, na, an.p, ixe, kze, z.p, db1.p, jgs, P.ipconc, P.dnu(il), il, P.ln(il), P.qxmin(il), P.xvmn(il), P.xvmx(il), P.lvol(il), P.xdn0(il), ix);  // call calczgr1d
        }  // 4685
        fallout1d(P, W, nx, ny, nz, nor, na, dtptmp, dtfrac, jgs, xvt.at(1, 1, 1, il), an.p, db1.p, il, 1, xfall.p, dtz1.p, ix);  // call fallout1d
        if ((P.ldovol && (il >= P.li))) {  // 4699
          if ((P.lvol(il) > 1)) {  // 4700
            fallout1d(P, W, nx, ny, nz, nor, na, dtptmp, dtfrac, jgs, xvt.at(1, 1, 1, il), an.p, db1.p, P.lvol(il), 0, xfall.p, dtz1.p, ix);  // call fallout1d
          }  // 4703
        }  // 4704
        if ((P.ipconc >= 6)) {  // 4708
          if ((P.lz(il) > 1)) {  // 4709
            fallout1d(P, W, nx, ny, nz, nor, na, dtptmp, dtfrac, jgs, xvt.at(1, 1, 3, il), an.p, db1.p, P.lz(il), 0, xfall.p, dtz1.p, ix);  // call fallout1d
          }  // 4712
        }  // 4713
        if ((P.ipconc > 0)) {  // 4718
          if ((P.ipconc >= P.ipc(il))) {  // 4719
            if ((((P.linfall(il) >= 2)) && (P.lz(il) < 1))) {  // 4721
              if (((P.linfall(il) == 3) || (P.linfall(il) == 4))) {  // 4727
                for (kz = kzb; kz <= kze; ++kz) {  // 4729
                  tmpn2(ix, jy, kz) = z(ix, kz, il);  // 4730
                }  // 4731
                for (kz = kzb; kz <= kze; ++kz) {  // 4732
                  tmpn(ix, jy, kz) = an(ix, jy, kz, P.ln(il));  // 4733
                }  // 4734
              } else {  // 4736
                for (kz = kzb; kz <= kze; ++kz) {  // 4738
                  tmpn(ix, jy, kz) = an(ix, jy, kz, P.ln(il));  // 4739
                }  // 4740
              }  // 4742
            }  // 4744
            in = 2;  // 4749
            if ((P.linfall(il) == 1)) { in = 1; }  // 4750
            fallout1d(P, W, nx, ny, nz, nor, na, dtptmp, dtfrac, jgs, xvt.at(1, 1, in, il), an.p, db1.p, P.ln(il), 0, xfall.p, dtz1.p, ix);  // call fallout1d
            if ((P.lz(il) < 1)) {  // 4756
              if ((P.linfall(il) >= 2)) {  // 4757
                for (int i_0 = 1; i_0 <= nx; ++i_0) xfall0(i_0, jgs) = NR(0.0);  // 4758
                if (((((P.infall == 3) || (P.infall == 4))) && ((((il >= P.lh) || (((il == P.lr) && (P.irfall == P.infall)))) || (((il == P.ls) && (P.isfall == P.infall))))))) {  // 4760
                  fallout1d(P, W, nx, ny, nz, nor, 1, dtptmp, dtfrac, jgs, xvt.at(1, 1, 3, il), tmpn2.p, db1.p, 1, 0, xfall0.p, dtz1.p, ix);  // call fallout1d
                  fallout1d(P, W, nx, ny, nz, nor, 1, dtptmp, dtfrac, jgs, xvt.at(1, 1, 1, il), tmpn.p, db1.p, 1, 0, xfall0.p, dtz1.p, ix);  // call fallout1d
                } else {  // 4767
                  fallout1d(P, W, nx, ny, nz, nor, 1, dtptmp, dtfrac, jgs, xvt.at(1, 1, 1, il), tmpn.p, db1.p, 1, 0, xfall0.p, dtz1.p, ix);  // call fallout1d
                }  // 4770
                if (((P.linfall(il) == 3) || (P.linfall(il) == 4))) {  // 4772
                  calcnfromz1d(P, W, nx, ny, nz, nor, na, an.p, tmpn2.p, ixe, kze, z.p, db1.p, jgs, P.ipconc, P.dnu(il), il, P.ln(il), P.qxmin(il), P.xvmn(il), P.xvmx(il), tmpn.p, P.lvol(il), P.xdn0(il), P.infall, ix);  // call calcnfromz1d
                } else if ((((P.linfall(il) == 5) && (il >= P.lh)) || (((il == P.lr) && (P.irfall == 5))))) {  // 4782
                  for (kz = kzb; kz <= kze; ++kz) {  // 4784
                    an(ix, jgs, kz, P.ln(il)) = c_max(an(ix, jgs, kz, P.ln(il)), (NR(0.5) * ((an(ix, jgs, kz, P.ln(il)) + tmpn(ix, jy, kz)))));  // 4785
                  }  // 4786
                } else {  // 4788
                  for (kz = kzb; kz <= kze; ++kz) {  // 4791
                    an(ix, jgs, kz, P.ln(il)) = c_max(an(ix, jgs, kz, P.ln(il)), tmpn(ix, jy, kz));  // 4792
                  }  // 4793
                }  // 4794
              }  // 4795
            }  // 4798
          }  // 4799
        }  // 4802
      }  // 4805
    }  // 4806
  }  // 4808
  return;  // 4834

}
template <class Real>
CPPWRF_HD inline void calcnfromq(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, Real* an_p, int na, int nor, int norz, Real* dn_p) {
  const Real xn0s = NR(3.0e8);  // 5307
  const Real xn0r = NR(8.0e6);  // 5307
  const Real xn0h = NR(2.0e5);  // 5307
  const Real xn0hl = NR(4.0e4);  // 5307
  const Real xdnr = NR(1000.0);  // 5308
  const Real xdns = NR(100.0);  // 5308
  const Real xdnh = NR(700.0);  // 5308
  const Real xdnhl = NR(900.0);  // 5308
  const Real zhlfac = (NR(1.0) / (((P.pi * xdnhl) * xn0hl)));  // 5309
  const Real zhfac = (NR(1.0) / (((P.pi * xdnh) * xn0h)));  // 5310
  const Real zrfac = (NR(1.0) / (((P.pi * xdnr) * xn0r)));  // 5311
  const Real zsfac = (NR(1.0) / (((P.pi * xdns) * xn0s)));  // 5312
  const Real g0 = ((((NR(6.0)) * (NR(5.0))) * (NR(4.0))) / ((((NR(3.0)) * (NR(2.0))) * (NR(1.0)))));  // 5313
  const Real xims = ((NR(900.0) * NR(0.523599)) * NR(hexd(0x3D719799812DEA12ull)));  // 5314
  const Real xgms = ((xdnh * NR(0.523599)) * NR(hexd(0x3DBDAFD309FD7AFBull)));  // 5315
  const Real cwmas09 = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3CFA43D6953FE15Aull)));  // 5316
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, 1, nx, 1, (nz + 1));
  int ngt = 0;  // 5283
  int jgs = 0;  // 5283
  int ixcol = 0;  // 5283
  FA<Real> qcw;  // 5289 optional, absent
  FA<Real> qci;  // 5289 optional, absent
  FA<Real> qsw;  // 5289 optional, absent
  FA<Real> qrw;  // 5289 optional, absent
  FA<Real> qhw;  // 5289 optional, absent
  FA<Real> qhl;  // 5289 optional, absent
  FA<Real> ccw;  // 5289 optional, absent
  FA<Real> cci;  // 5289 optional, absent
  FA<Real> csw;  // 5289 optional, absent
  FA<Real> crw;  // 5289 optional, absent
  FA<Real> chw;  // 5289 optional, absent
  FA<Real> chl;  // 5289 optional, absent
  FA<Real> cccn;  // 5289 optional, absent
  FA<Real> cccna;  // 5289 optional, absent
  FA<Real> vhw;  // 5289 optional, absent
  FA<Real> vhl;  // 5289 optional, absent
  FA<Real> qv;  // 5289 optional, absent
  FA<Real> spechum;  // 5289 optional, absent
  bool invertccn_flag = false;  // 5292 , optional, intent(in)
  Real cwmasin = 0;  // 5293 , optional
  int ixe = 0;  // 5295
  int kze = 0;  // 5295
  Real alpha = 0;  // 5296
  Real qmin = 0;  // 5297
  Real xvmn = 0;  // 5298
  Real xvmx = 0;  // 5298
  int ipconc = 0;  // 5299
  int lvol = 0;  // 5300
  int infall = 0;  // 5301
  int ix = 0;  // 5304
  int jy = 0;  // 5304
  int kz = 0;  // 5304
  double vr = 0;  // 5305
  double q = 0;  // 5305
  double nrx = 0;  // 5305
  double nrx2 = 0;  // 5305
  double rd = 0;  // 5305
  double g1h = 0;  // 5305
  double g1hl = 0;  // 5305
  double g1r = 0;  // 5305
  double g1s = 0;  // 5305
  double zx = 0;  // 5305
  double z = 0;  // 5305
  double znew = 0;  // 5305
  double zt = 0;  // 5305
  double zxt = 0;  // 5305
  double n1 = 0;  // 5305
  double laminv1 = 0;  // 5305
  double zr = 0;  // 5306
  double zs = 0;  // 5306
  double zh = 0;  // 5306
  double dninv = 0;  // 5306
  Real xv = 0;  // 5318
  Real xdn = 0;  // 5318
  Real cwmasinv = 0;  // 5318
  int ndbz = 0;  // 5319
  int nmwgt = 0;  // 5319
  int nnwgt = 0;  // 5319
  int nwlessthanz = 0;  // 5319
  double mixconv = 0;  // 5320
  double mixconvqv = 0;  // 5320
  double qsmax = 0;  // 5320
  double qsmax2 = 0;  // 5320
  double qsmax3 = 0;  // 5320
  double qsmax4 = 0;  // 5320
  bool invertccn_local = false;  // 5321
  if (false) {  // 5325
    invertccn_local = invertccn_flag;  // 5326
  } else {  // 5327
    invertccn_local = false;  // 5328
  }  // 5329
  if (false) {  // 5331
    cwmasinv = (NR(1.0) / cwmasin);  // 5332
  } else {  // 5333
    cwmasinv = (NR(1.0) / cwmas09);  // 5334
  }  // 5335
  jy = 1;  // 5337
  g1h = (((((NR(6.0) + P.alphah)) * ((NR(5.0) + P.alphah))) * ((NR(4.0) + P.alphah))) / (((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))));  // 5340
  g1hl = (((((NR(6.0) + P.alphahl)) * ((NR(5.0) + P.alphahl))) * ((NR(4.0) + P.alphahl))) / (((((NR(3.0) + P.alphahl)) * ((NR(2.0) + P.alphahl))) * ((NR(1.0) + P.alphahl)))));  // 5343
  g1r = (((((NR(6.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(4.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 5349
  g1s = (((P.snu + NR(2.0))) / ((P.snu + NR(1.0))));  // 5353
  qsmax = 0;  // 5354
  qsmax2 = 0;  // 5355
  qsmax3 = 0;  // 5356
  qsmax4 = 0;  // 5357
  for (kz = 1; kz <= nz; ++kz) {  // 5359
    for (ix = 1; ix <= nx; ++ix) {  // 5360
      mixconv = 1;  // 5365
      if (false) {  // 5366
        an(ix, jy, kz, P.lv) = (spechum(ix, kz) / ((1.0e0 - spechum(ix, kz))));  // 5367
        mixconv = (1.0e0 / ((1.0e0 - spechum(ix, kz))));  // 5368
      } else {  // 5369
        mixconv = 1.0e0;  // 5370
      }  // 5371
      if (false) { an(ix, jy, kz, P.lv) = qv(ix, kz); }  // 5372
      if (false) { an(ix, jy, kz, P.lc) = (qcw(ix, kz) * mixconv); }  // 5373
      if (false) { an(ix, jy, kz, P.lr) = (qrw(ix, kz) * mixconv); }  // 5374
      if (false) { an(ix, jy, kz, P.li) = (qci(ix, kz) * mixconv); }  // 5375
      if (false) {  // 5376
        an(ix, jy, kz, P.ls) = (qsw(ix, kz) * mixconv);  // 5377
      }  // 5380
      if (false) { an(ix, jy, kz, P.lh) = (qhw(ix, kz) * mixconv); }  // 5381
      if (((P.lhl > 1) && false)) { an(ix, jy, kz, P.lhl) = (qhl(ix, kz) * mixconv); }  // 5382
      if (false) { an(ix, jy, kz, P.lnc) = ((ccw(ix, kz) * mixconv) * dn(ix, kz)); }  // 5383
      if (false) { an(ix, jy, kz, P.lnr) = ((crw(ix, kz) * mixconv) * dn(ix, kz)); }  // 5384
      if (false) { an(ix, jy, kz, P.lni) = ((cci(ix, kz) * mixconv) * dn(ix, kz)); }  // 5385
      if (false) { an(ix, jy, kz, P.lns) = ((csw(ix, kz) * mixconv) * dn(ix, kz)); }  // 5386
      if (false) { an(ix, jy, kz, P.lnh) = ((chw(ix, kz) * mixconv) * dn(ix, kz)); }  // 5387
      if (((P.lhl > 1) && false)) { an(ix, jy, kz, P.lnhl) = ((chl(ix, kz) * mixconv) * dn(ix, kz)); }  // 5388
      if (((P.lvh > 1) && false)) { an(ix, jy, kz, P.lvh) = (vhw(ix, kz) * mixconv); }  // 5389
      if (((P.lvhl > 1) && false)) { an(ix, jy, kz, P.lvhl) = (vhl(ix, kz) * mixconv); }  // 5390
      if (((P.lccn > 1) && false)) { an(ix, jy, kz, P.lccn) = ((cccn(ix, kz) * mixconv) * dn(ix, kz)); }  // 5391
      if (((P.lccna > 1) && false)) { an(ix, jy, kz, P.lccna) = (cccna(ix, kz) * mixconv); }  // 5392
      dninv = (NR(1.0) / dn(ix, kz));  // 5394
      if ((P.lnc > 1)) {  // 5399
        if (((an(ix, jy, kz, P.lnc) <= P.cxmin) && (an(ix, jy, kz, P.lc) > P.qxmin_init(P.lc)))) {  // 5400
          an(ix, jy, kz, P.lnc) = (c_min(P.qccn, (an(ix, jy, kz, P.lc) * cwmasinv)) * dn(ix, kz));  // 5402
          if (invertccn_local) {  // 5404
            an(ix, jy, kz, P.lccn) = (an(ix, jy, kz, P.lccn) + an(ix, jy, kz, P.lnc));  // 5405
          } else {  // 5406
            if (((P.lccn > 1) && (P.lccna < 1))) {  // 5408
              an(ix, jy, kz, P.lccn) = (an(ix, jy, kz, P.lccn) - an(ix, jy, kz, P.lnc));  // 5409
            }  // 5410
            if ((P.lccna > 1)) {  // 5411
              an(ix, jy, kz, P.lccna) = (an(ix, jy, kz, P.lccna) + an(ix, jy, kz, P.lnc));  // 5412
            }  // 5413
          }  // 5414
        } else if (((an(ix, jy, kz, P.lc) <= P.qxmin(P.lc)) || (((an(ix, jy, kz, P.lnc) <= P.cxmin) && (an(ix, jy, kz, P.lc) <= P.qxmin_init(P.lc)))))) {  // 5416
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lc));  // 5419
          an(ix, jy, kz, P.lnc) = NR(0.0);  // 5420
          an(ix, jy, kz, P.lc) = NR(0.0);  // 5421
        }  // 5423
      }  // 5424
      if ((P.lni > 1)) {  // 5428
        if (((an(ix, jy, kz, P.lni) <= P.cxmin) && (an(ix, jy, kz, P.li) > P.qxmin_init(P.li)))) {  // 5429
          an(ix, jy, kz, P.lni) = ((dn(ix, kz) * an(ix, jy, kz, P.li)) / xims);  // 5430
        } else if (((an(ix, jy, kz, P.li) <= P.qxmin(P.li)) || (((an(ix, jy, kz, P.lni) <= P.cxmin) && (an(ix, jy, kz, P.li) <= P.qxmin_init(P.li)))))) {  // 5432
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.li));  // 5434
          an(ix, jy, kz, P.lni) = NR(0.0);  // 5435
          an(ix, jy, kz, P.li) = NR(0.0);  // 5436
        }  // 5437
      }  // 5438
      if ((P.lnr > 1)) {  // 5442
        if (((an(ix, jy, kz, P.lnr) <= (NR(0.1) * P.cxmin)) && (an(ix, jy, kz, P.lr) > P.qxmin_init(P.lr)))) {  // 5443
          q = an(ix, jy, kz, P.lr);  // 5445
          laminv1 = pow((((dn(ix, kz) * q) * zrfac)), (NR(0.25)));  // 5447
          n1 = (laminv1 * xn0r);  // 5449
          nrx = ((n1 * g1r) / g0);  // 5451
          an(ix, jy, kz, P.lnr) = nrx;  // 5453
        } else if (((an(ix, jy, kz, P.lr) <= P.qxmin(P.lr)) || (((an(ix, jy, kz, P.lnr) <= P.cxmin) && (an(ix, jy, kz, P.lr) <= P.qxmin_init(P.lr)))))) {  // 5455
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lr));  // 5457
          an(ix, jy, kz, P.lnr) = NR(0.0);  // 5458
          an(ix, jy, kz, P.lr) = NR(0.0);  // 5459
        }  // 5460
      }  // 5461
      if ((P.lns > 1)) {  // 5473
        if (((an(ix, jy, kz, P.lns) <= (NR(0.1) * P.cxmin)) && (an(ix, jy, kz, P.ls) > P.qxmin_init(P.ls)))) {  // 5474
          q = an(ix, jy, kz, P.ls);  // 5476
          laminv1 = pow((((dn(ix, kz) * q) * zsfac)), (NR(0.25)));  // 5478
          n1 = (laminv1 * xn0s);  // 5480
          nrx = ((n1 * g1s) / g0);  // 5482
          an(ix, jy, kz, P.lns) = nrx;  // 5484
        } else if (((an(ix, jy, kz, P.ls) <= P.qxmin(P.ls)) || (((an(ix, jy, kz, P.lns) <= P.cxmin) && (an(ix, jy, kz, P.ls) <= P.qxmin_init(P.ls)))))) {  // 5486
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.ls));  // 5488
          an(ix, jy, kz, P.lns) = NR(0.0);  // 5489
          an(ix, jy, kz, P.ls) = NR(0.0);  // 5490
        }  // 5492
      }  // 5493
      if ((P.lnh > 1)) {  // 5497
        if (((an(ix, jy, kz, P.lnh) <= (NR(0.1) * P.cxmin)) && (an(ix, jy, kz, P.lh) > P.qxmin_init(P.lh)))) {  // 5498
          if ((P.lvh > 1)) {  // 5499
            if ((an(ix, jy, kz, P.lvh) <= NR(0.0))) {  // 5500
              an(ix, jy, kz, P.lvh) = (an(ix, jy, kz, P.lh) / xdnh);  // 5501
            }  // 5502
          }  // 5503
          q = an(ix, jy, kz, P.lh);  // 5505
          laminv1 = pow((((dn(ix, kz) * q) * zhfac)), (NR(0.25)));  // 5507
          n1 = (laminv1 * xn0h);  // 5509
          nrx = ((n1 * g1h) / g0);  // 5511
          nrx2 = ((dn(ix, kz) * q) / xgms);  // 5513
          nrx = c_min(nrx, nrx2);  // 5515
          if ((nrx > P.cxmin)) {  // 5517
            an(ix, jy, kz, P.lnh) = nrx;  // 5518
          } else {  // 5519
            an(ix, jy, kz, P.lh) = NR(0.0);  // 5520
            an(ix, jy, kz, P.lnh) = NR(0.0);  // 5521
            an(ix, jy, kz, P.lvh) = NR(0.0);  // 5522
          }  // 5523
        } else if (((an(ix, jy, kz, P.lh) <= P.qxmin(P.lh)) || (((an(ix, jy, kz, P.lnh) <= P.cxmin) && (an(ix, jy, kz, P.lh) <= P.qxmin_init(P.lh)))))) {  // 5525
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lh));  // 5528
          an(ix, jy, kz, P.lh) = NR(0.0);  // 5529
        }  // 5531
      }  // 5532
      if (((P.lnhl > 1) && (P.lhl > 1))) {  // 5545
        if (((an(ix, jy, kz, P.lnhl) <= (NR(0.1) * P.cxmin)) && (an(ix, jy, kz, P.lhl) > P.qxmin_init(P.lhl)))) {  // 5546
          if ((P.lvhl > 1)) {  // 5547
            if ((an(ix, jy, kz, P.lvhl) <= NR(0.0))) {  // 5548
              an(ix, jy, kz, P.lvhl) = (an(ix, jy, kz, P.lhl) / xdnhl);  // 5549
            }  // 5550
          }  // 5551
          q = an(ix, jy, kz, P.lhl);  // 5553
          laminv1 = pow((((dn(ix, kz) * q) * zhlfac)), (NR(0.25)));  // 5555
          n1 = (laminv1 * xn0hl);  // 5557
          nrx = ((n1 * g1hl) / g0);  // 5559
          an(ix, jy, kz, P.lnhl) = nrx;  // 5561
        } else if (((an(ix, jy, kz, P.lhl) <= P.qxmin(P.lhl)) || (((an(ix, jy, kz, P.lnhl) <= P.cxmin) && (an(ix, jy, kz, P.lhl) <= P.qxmin_init(P.lhl)))))) {  // 5563
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lhl));  // 5566
          an(ix, jy, kz, P.lhl) = NR(0.0);  // 5567
        }  // 5569
      }  // 5570
      mixconvqv = 1;  // 5587
      if (false) {  // 5588
        mixconvqv = (1.0e0 / ((1.0e0 + an(ix, jy, kz, P.lv))));  // 5590
        spechum(ix, kz) = (an(ix, jy, kz, P.lv) * mixconvqv);  // 5591
      } else {  // 5592
        mixconvqv = 1.0e0;  // 5593
      }  // 5594
      if (false) { qv(ix, kz) = an(ix, jy, kz, P.lv); }  // 5596
      if (false) { qcw(ix, kz) = (an(ix, jy, kz, P.lc) * mixconvqv); }  // 5597
      if (false) { qrw(ix, kz) = (an(ix, jy, kz, P.lr) * mixconvqv); }  // 5598
      if (false) { qci(ix, kz) = (an(ix, jy, kz, P.li) * mixconvqv); }  // 5599
      if (false) {  // 5600
        qsw(ix, kz) = (an(ix, jy, kz, P.ls) * mixconvqv);  // 5601
      }  // 5604
      if (false) { qhw(ix, kz) = (an(ix, jy, kz, P.lh) * mixconvqv); }  // 5605
      if (((P.lhl > 1) && false)) { qhl(ix, kz) = (an(ix, jy, kz, P.lhl) * mixconvqv); }  // 5606
      if (false) { ccw(ix, kz) = ((an(ix, jy, kz, P.lnc) * mixconvqv) * dninv); }  // 5607
      if (false) { crw(ix, kz) = ((an(ix, jy, kz, P.lnr) * mixconvqv) * dninv); }  // 5608
      if (false) { cci(ix, kz) = ((an(ix, jy, kz, P.lni) * mixconvqv) * dninv); }  // 5609
      if (false) { csw(ix, kz) = ((an(ix, jy, kz, P.lns) * mixconvqv) * dninv); }  // 5610
      if (false) { chw(ix, kz) = ((an(ix, jy, kz, P.lnh) * mixconvqv) * dninv); }  // 5611
      if (((P.lhl > 1) && false)) { chl(ix, kz) = ((an(ix, jy, kz, P.lnhl) * mixconvqv) * dninv); }  // 5612
      if (((P.lvh > 1) && false)) { vhw(ix, kz) = (an(ix, jy, kz, P.lvh) * mixconvqv); }  // 5613
      if (((P.lvhl > 1) && false)) { vhl(ix, kz) = (an(ix, jy, kz, P.lvhl) * mixconvqv); }  // 5614
      if (((P.lccn > 1) && false)) { cccn(ix, kz) = ((an(ix, jy, kz, P.lccn) * mixconvqv) * dninv); }  // 5615
      if (((P.lccna > 1) && false)) { cccna(ix, kz) = (an(ix, jy, kz, P.lccna) * mixconvqv); }  // 5616
    }  // 5619
  }  // 5620
  return;  // 5631

}
template <class Real>
CPPWRF_HD inline void calcnfromcuten(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, Real* an_p, Real* anold_p, int na, int nor, int norz, Real* dn_p) {
  const Real xn0s = NR(3.0e6);  // 5669
  const Real xn0r = NR(8.0e6);  // 5669
  const Real xn0h = NR(4.0e4);  // 5669
  const Real xn0hl = NR(4.0e4);  // 5669
  const Real xdnr = NR(1000.0);  // 5670
  const Real xdns = NR(100.0);  // 5670
  const Real xdnh = NR(700.0);  // 5670
  const Real xdnhl = NR(900.0);  // 5670
  const Real zhlfac = (NR(1.0) / (((P.pi * xdnhl) * xn0hl)));  // 5671
  const Real zhfac = (NR(1.0) / (((P.pi * xdnh) * xn0h)));  // 5672
  const Real zrfac = (NR(1.0) / (((P.pi * xdnr) * xn0r)));  // 5673
  const Real zsfac = (NR(1.0) / (((P.pi * xdns) * xn0s)));  // 5674
  const Real g0 = ((((NR(6.0)) * (NR(5.0))) * (NR(4.0))) / ((((NR(3.0)) * (NR(2.0))) * (NR(1.0)))));  // 5675
  const Real xims = ((NR(900.0) * NR(0.523599)) * NR(hexd(0x3D719799812DEA12ull)));  // 5676
  const Real xcms = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3CEE66385C266145ull)));  // 5677
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> anold = mkfa<Real>(anold_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, 1, nx, 1, (nz + 1));
  int ngt = 0;  // 5650
  int jgs = 0;  // 5650
  int ixcol = 0;  // 5650
  int ixe = 0;  // 5657
  int kze = 0;  // 5657
  Real alpha = 0;  // 5658
  Real qmin = 0;  // 5659
  Real xvmn = 0;  // 5660
  Real xvmx = 0;  // 5660
  int ipconc = 0;  // 5661
  int lvol = 0;  // 5662
  int infall = 0;  // 5663
  int ix = 0;  // 5666
  int jy = 0;  // 5666
  int kz = 0;  // 5666
  double vr = 0;  // 5667
  double q = 0;  // 5667
  double nrx = 0;  // 5667
  double rd = 0;  // 5667
  double g1h = 0;  // 5667
  double g1hl = 0;  // 5667
  double g1r = 0;  // 5667
  double g1s = 0;  // 5667
  double zx = 0;  // 5667
  double chw = 0;  // 5667
  double z = 0;  // 5667
  double znew = 0;  // 5667
  double zt = 0;  // 5667
  double zxt = 0;  // 5667
  double n1 = 0;  // 5667
  double laminv1 = 0;  // 5667
  double zr = 0;  // 5668
  double zs = 0;  // 5668
  double zh = 0;  // 5668
  double dninv = 0;  // 5668
  Real xmass = 0;  // 5679
  Real xv = 0;  // 5679
  Real xdn = 0;  // 5679
  int ndbz = 0;  // 5680
  int nmwgt = 0;  // 5680
  int nnwgt = 0;  // 5680
  int nwlessthanz = 0;  // 5680
  jy = 1;  // 5685
  g1h = (((((NR(6.0) + P.alphah)) * ((NR(5.0) + P.alphah))) * ((NR(4.0) + P.alphah))) / (((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))));  // 5688
  g1hl = (((((NR(6.0) + P.alphahl)) * ((NR(5.0) + P.alphahl))) * ((NR(4.0) + P.alphahl))) / (((((NR(3.0) + P.alphahl)) * ((NR(2.0) + P.alphahl))) * ((NR(1.0) + P.alphahl)))));  // 5691
  g1r = (((((NR(6.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(4.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 5697
  g1s = (((P.snu + NR(2.0))) / ((P.snu + NR(1.0))));  // 5701
  for (kz = 1; kz <= nz; ++kz) {  // 5703
    for (ix = 1; ix <= nx; ++ix) {  // 5704
      dninv = (NR(1.0) / dn(ix, kz));  // 5706
      if ((P.lnc > 1)) {  // 5710
        if ((an(ix, jy, kz, P.lnc) > P.qxmin(P.lc))) {  // 5712
          anold(ix, jy, kz, P.lnc) = (anold(ix, jy, kz, P.lnc) + (an(ix, jy, kz, P.lc) / xcms));  // 5713
        }  // 5714
      }  // 5715
      if ((P.lni > 1)) {  // 5719
        if ((an(ix, jy, kz, P.lni) > P.qxmin(P.li))) {  // 5720
          anold(ix, jy, kz, P.lni) = (anold(ix, jy, kz, P.lni) + (an(ix, jy, kz, P.li) / xims));  // 5721
        }  // 5722
      }  // 5723
      if ((P.lnr > 1)) {  // 5727
        if ((an(ix, jy, kz, P.lr) > P.qxmin(P.lr))) {  // 5728
          q = an(ix, jy, kz, P.lr);  // 5732
          laminv1 = pow((((dn(ix, kz) * q) * zrfac)), (NR(0.25)));  // 5734
          n1 = (laminv1 * xn0r);  // 5736
          nrx = ((n1 * g1r) / g0);  // 5738
          anold(ix, jy, kz, P.lnr) = (anold(ix, jy, kz, P.lnr) + nrx);  // 5740
        }  // 5751
      }  // 5752
      if ((P.lns > 1)) {  // 5755
        if ((an(ix, jy, kz, P.ls) > P.qxmin(P.ls))) {  // 5756
          q = an(ix, jy, kz, P.ls);  // 5762
          laminv1 = pow((((dn(ix, kz) * q) * zsfac)), (NR(0.25)));  // 5764
          n1 = (laminv1 * xn0s);  // 5766
          nrx = ((n1 * g1s) / g0);  // 5768
          anold(ix, jy, kz, P.lns) = (anold(ix, jy, kz, P.lns) + nrx);  // 5770
        }  // 5778
      }  // 5779
    }  // 5833
  }  // 5834
  return;  // 5836

}
template <class Real>
CPPWRF_HD inline void qvexcess(const NsslParams<Real>& P, Work W, int ngs, int mgs, Real* qwvp0_p, Real* qv0_p, Real* qcw1_p, Real* pres_p, Real* thetap0_p, Real* theta0_p, Real& qvex, Real* pi0_p, Real* tabqvs_p, int nqsat, Real fqsat, Real cbw, Real* fcqv1_p, Real* felvcp_p, Real ss1, Real* pk_p, int ngscnt) {
  const Real tfr = NR(273.15);
  FA<Real> qwvp0 = mkfa<Real>(qwvp0_p, 1, mgs);
  FA<Real> qv0 = mkfa<Real>(qv0_p, 1, ngs);
  FA<Real> qcw1 = mkfa<Real>(qcw1_p, 1, ngscnt);
  FA<Real> pres = mkfa<Real>(pres_p, 1, ngs);
  FA<Real> thetap0 = mkfa<Real>(thetap0_p, 1, ngs);
  FA<Real> theta0 = mkfa<Real>(theta0_p, 1, ngs);
  FA<Real> pi0 = mkfa<Real>(pi0_p, 1, ngs);
  FA<Real> tabqvs = mkfa<Real>(tabqvs_p, 1, nqsat);
  FA<Real> fcqv1 = mkfa<Real>(fcqv1_p, 1, ngs);
  FA<Real> felvcp = mkfa<Real>(felvcp_p, 1, ngs);
  FA<Real> pk = mkfa<Real>(pk_p, 1, ngs);
  Real theta2temp = 0;  // 6165
  int itertd = 0;  // 6186
  int ltemq = 0;  // 6187
  Real gamss = 0;  // 6188
  Real tmp = 0;  // 6188
  auto theta = W.template arr<Real>(1, ngs);  // 6189 
  auto qvap = W.template arr<Real>(1, ngs);  // 6189 
  auto pqs = W.template arr<Real>(1, ngs);  // 6189 
  auto qcw = W.template arr<Real>(1, ngs);  // 6189 
  auto qwv = W.template arr<Real>(1, ngs);  // 6189 
  auto qcwtmp = W.template arr<Real>(1, ngs);  // 6190 
  auto qss = W.template arr<Real>(1, ngs);  // 6190 
  auto qvs = W.template arr<Real>(1, ngs);  // 6190 
  auto qwvp = W.template arr<Real>(1, ngs);  // 6190 
  auto dqcw = W.template arr<Real>(1, ngs);  // 6191 
  auto dqwv = W.template arr<Real>(1, ngs);  // 6191 
  auto dqvcnd = W.template arr<Real>(1, ngs);  // 6191 
  auto temg = W.template arr<Real>(1, ngs);  // 6192 
  auto temcg = W.template arr<Real>(1, ngs);  // 6192 
  auto thetap = W.template arr<Real>(1, ngs);  // 6192 
  pqs(mgs) = ((NR(380.0)) / (pres(mgs)));  // 6207
  thetap(mgs) = thetap0(mgs);  // 6208
  theta(mgs) = (thetap(mgs) + theta0(mgs));  // 6209
  qwvp(mgs) = qwvp0(mgs);  // 6210
  qvap(mgs) = c_max(((qwvp0(mgs) + qv0(mgs))), NR(0.0));  // 6211
  temg(mgs) = (theta(mgs) * pk(mgs));  // 6212
  qwv(mgs) = c_max(NR(0.0), qvap(mgs));  // 6220
  qcw(mgs) = c_max(NR(0.0), qcw1(mgs));  // 6221
  qcwtmp(mgs) = qcw(mgs);  // 6224
  temcg(mgs) = (temg(mgs) - tfr);  // 6225
  ltemq = ((((temg(mgs) - NR(163.15))) / fqsat) + NR(1.5));  // 6226
  ltemq = c_min(nqsat, c_max(1, ltemq));  // 6227
  qvs(mgs) = (((P.rdorv * P.esbolton) * tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * tabqvs(ltemq)))));  // 6233
  qss(mgs) = ((((NR(0.01) * ss1) + NR(1.0))) * qvs(mgs));  // 6236
  for (itertd = 1; itertd <= 2; ++itertd) {  // 6240
    dqcw(mgs) = NR(0.0);  // 6245
    dqwv(mgs) = ((qwv(mgs) - qss(mgs)));  // 6246
    if ((dqwv(mgs) < NR(0.0))) {  // 6250
      if ((qcw(mgs) > (-dqwv(mgs)))) {  // 6251
        dqcw(mgs) = dqwv(mgs);  // 6252
        dqwv(mgs) = NR(0.0);  // 6253
      } else {  // 6254
        dqcw(mgs) = (-qcw(mgs));  // 6255
        dqwv(mgs) = (dqwv(mgs) + qcw(mgs));  // 6256
      }  // 6257
      qwvp(mgs) = (qwvp(mgs) - (dqcw(mgs)));  // 6259
      qcw(mgs) = (qcw(mgs) + dqcw(mgs));  // 6261
      thetap(mgs) = (thetap(mgs) + ((NR(1.0) / pi0(mgs)) * ((felvcp(mgs) * dqcw(mgs)))));  // 6263
    }  // 6267
    if ((dqwv(mgs) >= NR(0.0))) {  // 6271
      dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv1(mgs) * qss(mgs)) / (ipow<2>(((temg(mgs) - cbw))))))));  // 6273
      dqcw(mgs) = dqvcnd(mgs);  // 6277
      thetap(mgs) = (thetap(mgs) + (((felvcp(mgs) * dqcw(mgs))) / (pi0(mgs))));  // 6279
      qwvp(mgs) = (qwvp(mgs) - (dqvcnd(mgs)));  // 6282
      qcw(mgs) = (qcw(mgs) + dqcw(mgs));  // 6283
    }  // 6285
    theta(mgs) = (thetap(mgs) + theta0(mgs));  // 6287
    temg(mgs) = (theta(mgs) * pk(mgs));  // 6288
    qvap(mgs) = c_max(((qwvp(mgs) + qv0(mgs))), NR(0.0));  // 6290
    temcg(mgs) = (temg(mgs) - tfr);  // 6291
    ltemq = ((((temg(mgs) - NR(163.15))) / fqsat) + NR(1.5));  // 6293
    ltemq = c_min(nqsat, c_max(1, ltemq));  // 6294
    qvs(mgs) = (((P.rdorv * P.esbolton) * tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * tabqvs(ltemq)))));  // 6298
    qcw(mgs) = c_max(NR(0.0), qcw(mgs));  // 6300
    qwv(mgs) = c_max(NR(0.0), qvap(mgs));  // 6301
    qss(mgs) = ((((NR(0.01) * ss1) + NR(1.0))) * qvs(mgs));  // 6302
  }  // 6303
  qvex = c_max(NR(0.0), (qcw(mgs) - qcw1(mgs)));  // 6308
  return;  // 6310

}
template <class Real>
CPPWRF_HD inline void nssl_2mom_gs(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int na, int jyslab, int nor, int norz, Real dtp, Real* gz_p, Real* t0_p, Real* t1_p, Real* t2_p, Real* t3_p, Real* t4_p, Real* t5_p, Real* t6_p, Real* t7_p, Real* t8_p, Real* t9_p, Real* an_p, Real* dn_p, Real* p2_p, Real* pn_p, Real* w_p, int iunit, Real* t00_p, Real* t77_p, Real ventr, Real ventc, Real c1sw, int jgs, int* ido_p, Real* xdnmx_p, Real* xdnmn_p, Real* cdx_p, Real* xdn0_p, Real* tmp3d_p, Real* tkediss_p, Real* thproc_p, int numproc, Real dx1, Real dy1, int ngs, double timevtcalc, Real* axtra_p, bool io_flag, bool has_wetscav, Real* rainprod2d_p, Real* evapprod2d_p, Real* alpha2d_p, Real* elec_p, int its, int ids, int ide, int jds, int jde) {
  const int ng1 = 1;  // 12676
  const int myprock = 1;  // 12690
  const int nprock = 1;  // 12690
  const Real tfrdry = NR(243.15);  // 12701
  const bool gammacheck = false;  // 12731
  const bool usegamxinfcnu = false;  // 12734
  const bool usegamxinf = false;  // 12735
  const bool usegamxinf2 = false;  // 12736
  const bool usegamxinf3 = false;  // 12737
  const bool alp0flag = false;  // 12781
  const int ngsz = 500;  // 12846
  const Real thnuc = NR(235.15);  // 12860
  const Real shedalp = NR(3.0);  // 12886
  const Real aa1 = NR(9.44e15);  // 12898
  const Real aa2 = NR(5.78e3);  // 12898
  const Real cexs = NR(0.1);  // 12900
  const Real cecs = NR(0.5);  // 12900
  const Real rvt = NR(0.104);  // 12901
  const Real kfrag = NR(1.0e-6);  // 12902
  const Real mfrag = NR(1.0e-10);  // 12903
  const Real vr1mm = NR(5.23599e-10);  // 12925
  const Real vr3mm = (NR(5.23599e-10) * NR(hexd(0x403B000000000000ull)));  // 12926
  const Real vr4p5mm = (NR(5.23599e-10) * NR(hexd(0x4056C80000000000ull)));  // 12927
  const Real rimedens = NR(500.0);  // 12962
  const Real cpv = NR(1885.0);  // 13032
  const bool dosublimationfix = true;  // 13085
  const Real d1t = pow((((NR(6.0) * NR(0.268e-3)) / ((NR(917.0) * P.pi)))), ((NR(1.0) / NR(3.0))));  // 13106
  const Real srasheym = NR(0.1389);  // 13107
  const int ndiam = 10;  // 13113
  const Real cwmas30 = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3D4E66385C266145ull)));  // 13594
  const Real cwmas20 = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3D3203AF9EE75617ull)));  // 13595
  const Real c1r = NR(19.0);  // 13680
  const Real c2r = NR(0.6);  // 13680
  const Real c3r = NR(1.8);  // 13680
  const Real c4r = NR(17.0);  // 13680
  const Real c1h = NR(5.5);  // 13681
  const Real c2h = NR(0.7);  // 13681
  const Real c3h = NR(4.5);  // 13681
  const Real c4h = NR(8.5);  // 13681
  const Real c1hl = NR(3.7);  // 13682
  const Real c2hl = NR(0.3);  // 13682
  const Real c3hl = NR(9.0);  // 13682
  const Real c4hl = NR(6.5);  // 13682
  const Real c5hl = NR(1.0);  // 13682
  const Real c6hl = NR(6.5);  // 13682
  const bool newton = false;  // 13688
  const int iraincv = 1;
  const int icgxconv = 1;
  const int itest = 1;
  const int nidx = 10;
  const int id1 = 1;
  const int jd1 = 1;
  const int kd1 = 1;
  const Real eps = NR(1.0e-20);
  const Real eps2 = NR(1.0e-5);
  const int mu = 1;
  const int mv = 2;
  const int mw = 3;
  const int mqcw = 21;
  const int mqxw = 21;
  const int mtem = 21;
  const int mrho = 5;
  const int mtim = 6;
  const Real xftim = NR(0.05);
  const Real xftimi = (NR(1.0) / xftim);
  const Real yftim = NR(1.0);
  const Real xftem = NR(0.5);
  const Real yftem = NR(1.0);
  const Real xfqcw = NR(2000.0);
  const Real yfqcw = NR(1.0);
  const Real xfqxw = NR(2000.0);
  const Real yfqxw = NR(1.0);
  const Real dtfac = NR(1.0);
  const int ntt = 300;
  const int ncuse = 0;
  const Real sscb = NR(2.0);
  const int idecss = 1;
  const int iba = 1;
  const int ifilt = 0;
  const Real bfnu0 = (((P.rnu + NR(2.0))) / ((P.rnu + NR(1.0))));
  const Real rhosm = NR(500.0);
  const Real vgra = (NR(0.523599) * NR(hexd(0x3E112E0BE826D695ull)));
  const Real raero = NR(3.0e-7);
  const Real kaero = NR(5.39e-3);
  const Real kb = NR(1.3807e-23);
  const Real rwradmn = NR(50.0e-6);
  const Real clionpmx = NR(1.0e9);
  const Real clionnmx = NR(1.0e9);
  const int iholef = 1;
  const int iholen = 1;
  FA<Real> gz = mkfa<Real>(gz_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t1 = mkfa<Real>(t1_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t2 = mkfa<Real>(t2_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t3 = mkfa<Real>(t3_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t4 = mkfa<Real>(t4_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t5 = mkfa<Real>(t5_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t6 = mkfa<Real>(t6_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t7 = mkfa<Real>(t7_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t8 = mkfa<Real>(t8_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t9 = mkfa<Real>(t9_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> p2 = mkfa<Real>(p2_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> pn = mkfa<Real>(pn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> w = mkfa<Real>(w_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t00 = mkfa<Real>(t00_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> t77 = mkfa<Real>(t77_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<int> ido = mkfa<int>(ido_p, P.lc, P.lqmx);
  FA<Real> xdnmx = mkfa<Real>(xdnmx_p, P.lc, P.lhab);
  FA<Real> xdnmn = mkfa<Real>(xdnmn_p, P.lc, P.lhab);
  FA<Real> cdx = mkfa<Real>(cdx_p, P.lc, P.lhab);
  FA<Real> xdn0 = mkfa<Real>(xdn0_p, P.lc, P.lhab);
  FA<Real> tmp3d = mkfa<Real>(tmp3d_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> tkediss = mkfa<Real>(tkediss_p, ((-nor) + 1), (nx + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> thproc = mkfa<Real>(thproc_p, 1, nz, 1, numproc);
  FA<Real> axtra = mkfa<Real>(axtra_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz), 1, P.nxtra);
  FA<Real> rainprod2d = mkfa<Real>(rainprod2d_p, ((-nor) + 1), (nx + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> evapprod2d = mkfa<Real>(evapprod2d_p, ((-nor) + 1), (nx + nor), ((-norz) + ng1), (nz + norz));
  FA<Real> alpha2d = mkfa<Real>(alpha2d_p, ((-nor) + 1), (nx + nor), ((-norz) + ng1), (nz + norz), 1, 3);
  FA<Real> elec = mkfa<Real>(elec_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz));
  int nba = 0;  // 12678
  int nv = 0;  // 12678
  int istag = 0;  // 12679
  int jstag = 0;  // 12679
  int kstag = 0;  // 12679
  int iwrite = 0;  // 12680
  Real dx = 0;  // 12681
  Real dy = 0;  // 12681
  Real dz = 0;  // 12681
  int itile = 0;  // 12685
  int jtile = 0;  // 12685
  int ktile = 0;  // 12685
  int ixbeg = 0;  // 12686
  int jybeg = 0;  // 12686
  int ixend = 0;  // 12687
  int jyend = 0;  // 12687
  int kzend = 0;  // 12687
  int kzbeg = 0;  // 12687
  int nxend = 0;  // 12688
  int nyend = 0;  // 12688
  int nzend = 0;  // 12688
  int nzbeg = 0;  // 12688
  int my_rank = 0;  // 12689
  auto lrescalelow = W.template arr<bool>(P.lc, P.lhab);  // 12703 
  Real galpharaut = 0;  // 12707
  Real xvbarmax = 0;  // 12708
  Real qvex = 0;  // 12712
  Real ffrz = 0;  // 12715
  Real ffrzh = NR(1.0);  // 12716
  Real qcitmp = 0;  // 12718
  Real cirdiatmp = 0;  // 12718
  Real ccwtmp = 0;  // 12719
  Real ccitmp = 0;  // 12719
  Real cpqc = 0;  // 12720
  Real cpci = 0;  // 12720
  Real cpqc0 = 0;  // 12721
  Real cpci0 = 0;  // 12721
  Real scfac = 0;  // 12722
  double dp1 = 0;  // 12724
  double frac = 0;  // 12726
  double frach = 0;  // 12726
  double xvfrz = 0;  // 12726
  double xvbiggsnow = 0;  // 12726
  double dpt1 = 0;  // 12729
  double dpt2 = 0;  // 12729
  int luindex = 0;  // 12732
  double tmpgam = 0;  // 12733
  Real vtmax = 0;  // 12741
  int n = 0;  // 12742
  int ndfall = 0;  // 12742
  double chgneg = 0;  // 12744
  double chgpos = 0;  // 12744
  double sctot = 0;  // 12744
  Real temgtmp = 0;  // 12746
  auto pb = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 12748 
  auto pinit = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 12749 
  Real qimax = 0;  // 12753
  Real xni0 = 0;  // 12753
  Real roqi0 = 0;  // 12753
  Real dv = 0;  // 12756
  Real dtptmp = 0;  // 12758
  int ierr = 0;  // 12763
  int iend = 0;  // 12764
  int ix = 0;  // 12766
  int kz = 0;  // 12766
  int il = 0;  // 12766
  int ic = 0;  // 12766
  int ir = 0;  // 12766
  int icp1 = 0;  // 12766
  int irp1 = 0;  // 12766
  int ip1 = 0;  // 12766
  int jp1 = 0;  // 12766
  int kp1 = 0;  // 12766
  int jy = 0;  // 12767
  int i = 0;  // 12768
  int j = 0;  // 12768
  int k = 0;  // 12768
  int i1 = 0;  // 12768
  int kzb = 0;  // 12769
  int kze = 0;  // 12769
  Real slope1 = 0;  // 12770
  Real slope2 = 0;  // 12770
  Real x1 = 0;  // 12771
  Real x2 = 0;  // 12771
  Real x3 = 0;  // 12771
  Real y1 = 0;  // 12771
  Real temele = 0;  // 12777
  Real trev = 0;  // 12778
  bool ldovol = false;  // 12780
  bool ishail = false;  // 12780
  bool ltest = false;  // 12780
  bool wtest = false;  // 12780
  auto delqnxa = W.template arr<Real>(P.lc, P.lqmx);  // 12814 
  auto delqxxa = W.template arr<Real>(P.lc, P.lqmx);  // 12815 
  int nxmpb = 0;  // 12844
  int nzmpb = 0;  // 12844
  int nxz = 0;  // 12844
  int mgs = 0;  // 12845
  int numgs = 0;  // 12845
  auto dvmgs = W.template arr<Real>(1, ngs);  // 12850 
  int ngscnt = 0;  // 12852
  auto igs = W.template arr<int>(1, ngs);  // 12852 
  auto kgs = W.template arr<int>(1, ngs);  // 12852 
  auto kgsp = W.template arr<int>(1, ngs);  // 12853 
  auto kgsm = W.template arr<int>(1, ngs);  // 12853 
  auto kgsm2 = W.template arr<int>(1, ngs);  // 12853 
  auto il0 = W.template arr<int>(1, ngs);  // 12856 
  auto il5 = W.template arr<int>(1, ngs);  // 12856 
  auto il2 = W.template arr<int>(1, ngs);  // 12856 
  auto il3 = W.template arr<int>(1, ngs);  // 12856 
  Real tdtol = 0;  // 12859
  Real temsav = 0;  // 12859
  Real tfrcbw = 0;  // 12859
  Real tfrcbi = 0;  // 12859
  auto fimt1 = W.template arr<Real>(1, ngs);  // 12864 
  auto fimta = W.template arr<Real>(1, ngs);  // 12864 
  auto fimt2 = W.template arr<Real>(1, ngs);  // 12864 
  Real xcwmas = 0;  // 12865
  auto ccnc = W.template arr<Real>(1, ngs);  // 12872 
  auto ccin = W.template arr<Real>(1, ngs);  // 12872 
  auto cina = W.template arr<Real>(1, ngs);  // 12872 
  auto ccna = W.template arr<Real>(1, ngs);  // 12872 
  auto cwnccn = W.template arr<Real>(1, ngs);  // 12873 
  Real temp1 = 0;  // 12884
  Real temp2 = 0;  // 12884
  Real mwat = 0;  // 12885
  Real mice = 0;  // 12885
  Real dice = 0;  // 12885
  Real mwshed = 0;  // 12885
  Real fwmax = 0;  // 12885
  Real fw = 0;  // 12885
  Real mwcrit = 0;  // 12885
  Real massfactor = 0;  // 12885
  Real tmpdiam = 0;  // 12885
  auto ssmax = W.template arr<Real>(1, ngs);  // 12887 
  Real ssmx = 0;  // 12888
  Real dnnet = 0;  // 12889
  Real dqnet = 0;  // 12889
  Real bfnu = 0;  // 12892
  Real bfnu1 = 0;  // 12892
  Real volb = 0;  // 12895
  double t2s = 0;  // 12896
  double xdp = 0;  // 12896
  auto xl2p = W.template arr<double>(1, ngs);  // 12897 
  auto rb = W.template arr<double>(1, ngs);  // 12897 
  auto cautn = W.template arr<double>(1, ngs);  // 12904 
  auto rh = W.template arr<double>(1, ngs);  // 12904 
  auto nh = W.template arr<double>(1, ngs);  // 12904 
  Real ex1 = 0;  // 12905
  Real ft = 0;  // 12905
  auto rhoinv = W.template arr<Real>(1, ngs);  // 12905 
  auto ec0 = W.template arr<Real>(1, ngs);  // 12906 
  Real ac1 = 0;  // 12908
  Real bc = 0;  // 12908
  Real c1 = 0;  // 12908
  Real d1 = 0;  // 12908
  Real e1 = 0;  // 12908
  Real f1 = 0;  // 12908
  Real p380 = 0;  // 12908
  Real tmp = 0;  // 12908
  Real tmp1 = 0;  // 12908
  Real tmp2 = 0;  // 12908
  Real tmp3 = 0;  // 12908
  Real tmp4 = 0;  // 12908
  Real tmp5 = 0;  // 12908
  Real tmp6 = 0;  // 12908
  Real temp3 = 0;  // 12908
  Real flim = 0;  // 12909
  Real xmass = 0;  // 12909
  Real dw = 0;  // 12910
  Real dwr = 0;  // 12910
  double tmpz = 0;  // 12911
  double tmpzmlt = 0;  // 12911
  Real tmpc = 0;  // 12912
  Real ratio = 0;  // 12913
  Real delx = 0;  // 12913
  Real dely = 0;  // 12913
  Real dbigg = 0;  // 12914
  Real volt = 0;  // 12914
  Real chgtmp = 0;  // 12915
  Real fac = 0;  // 12915
  Real mixedphasefac = 0;  // 12915
  Real x = 0;  // 12916
  Real y = 0;  // 12916
  Real y2 = 0;  // 12916
  Real del = 0;  // 12916
  Real r = 0;  // 12916
  Real rtmp = 0;  // 12916
  Real alpr = 0;  // 12916
  double vent1 = 0;  // 12917
  double vent2 = 0;  // 12917
  double g1palp = 0;  // 12918
  double g4palp = 0;  // 12918
  double g1palpinf = 0;  // 12919
  double g4palpinf = 0;  // 12919
  Real fqt = 0;  // 12920
  Real bs = 0;  // 12921
  Real v1 = 0;  // 12922
  Real v2 = 0;  // 12922
  Real d1r = 0;  // 12923
  Real d1i = 0;  // 12923
  Real d1s = 0;  // 12923
  Real e1i = 0;  // 12923
  Real vmlt = 0;  // 12928
  Real vshd = 0;  // 12928
  auto vshdgs = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 12928 
  auto maxmassfac = W.template arr<Real>(P.lc, P.lhab);  // 12928 
  int nc = 0;  // 12931
  Real dtcon = 0;  // 12932
  Real dtcon1 = 0;  // 12932
  Real dtcon2 = 0;  // 12932
  Real delta = 0;  // 12933
  int ltemq1 = 0;  // 12934
  int ltemq1m = 0;  // 12934
  Real dqv = 0;  // 12935
  Real qv1 = 0;  // 12935
  Real ss1 = 0;  // 12935
  Real ss2 = 0;  // 12935
  Real qvs1 = 0;  // 12935
  Real dqvs = 0;  // 12935
  Real dtemp = 0;  // 12935
  Real dt1 = 0;  // 12935
  Real ssi1 = 0;  // 12936
  Real ssi2 = 0;  // 12936
  Real dqvi = 0;  // 12936
  Real dqvis = 0;  // 12936
  Real dqvii = 0;  // 12936
  Real qis1 = 0;  // 12936
  Real dqvr = 0;  // 12937
  Real dqc = 0;  // 12937
  Real dqr = 0;  // 12937
  Real dqi = 0;  // 12937
  Real dqs = 0;  // 12937
  Real qv1m = 0;  // 12938
  Real qvs1m = 0;  // 12938
  Real ss1m = 0;  // 12938
  Real ssi1m = 0;  // 12938
  Real qis1m = 0;  // 12938
  Real cwmastmp = 0;  // 12939
  Real dcloud = 0;  // 12940
  Real dcloud2 = 0;  // 12940
  auto cn = W.template arr<Real>(1, ngs);  // 12941 
  double xvc = 0;  // 12942
  double xvr = 0;  // 12942
  Real mwfac = 0;  // 12943
  Real rwmasn = 0;  // 12947
  Real rwmasx = 0;  // 12947
  Real vfrz = 0;  // 12949
  Real d = 0;  // 12954
  Real dold = 0;  // 12954
  Real denom = 0;  // 12954
  Real denominv = 0;  // 12954
  Real vth = 0;  // 12954
  double h1 = 0;  // 12955
  double h2 = 0;  // 12955
  double h3 = 0;  // 12955
  double h4 = 0;  // 12955
  double denomdp = 0;  // 12955
  double denominvdp = 0;  // 12955
  Real r1 = 0;  // 12956
  Real qevap = 0;  // 12956
  Real vr = 0;  // 12958
  Real nrx = 0;  // 12958
  Real chw = 0;  // 12958
  Real g1 = 0;  // 12958
  Real qr = 0;  // 12958
  Real z = 0;  // 12958
  Real z1 = 0;  // 12958
  Real rdi = 0;  // 12958
  Real alp = 0;  // 12958
  Real xnutmp = 0;  // 12958
  Real xnuc = 0;  // 12958
  Real g1r = 0;  // 12958
  Real rd1 = 0;  // 12958
  Real rdia = 0;  // 12958
  Real rmas = 0;  // 12958
  Real snowmeltmass = 0;  // 12959
  auto knud = W.template arr<Real>(1, ngs);  // 12973 
  auto knuda = W.template arr<Real>(1, ngs);  // 12973 
  auto gtp = W.template arr<Real>(1, ngs);  // 12974 
  auto dfar = W.template arr<Real>(1, ngs);  // 12975 
  auto fn1 = W.template arr<Real>(1, ngs);  // 12976 
  auto fn2 = W.template arr<Real>(1, ngs);  // 12976 
  auto fnft = W.template arr<Real>(1, ngs);  // 12976 
  auto ccia = W.template arr<Real>(1, ngs);  // 12978 
  auto ctfzbd = W.template arr<Real>(1, ngs);  // 12979 
  auto ctfzth = W.template arr<Real>(1, ngs);  // 12979 
  auto ctfzdi = W.template arr<Real>(1, ngs);  // 12979 
  Real ni = 0;  // 12983
  Real nis = 0;  // 12983
  Real nr = 0;  // 12983
  Real d0 = 0;  // 12983
  auto dqvcnd = W.template arr<Real>(1, ngs);  // 12984 
  auto dqwv = W.template arr<Real>(1, ngs);  // 12984 
  auto dqcw = W.template arr<Real>(1, ngs);  // 12984 
  auto dqci = W.template arr<Real>(1, ngs);  // 12984 
  auto dqcitmp = W.template arr<Real>(1, ngs);  // 12984 
  auto dqwvtmp = W.template arr<Real>(1, ngs);  // 12984 
  auto tempc = W.template arr<Real>(1, ngs);  // 12985 
  auto temg = W.template arr<Real>(1, ngs);  // 12986 
  auto temcg = W.template arr<Real>(1, ngs);  // 12986 
  auto theta = W.template arr<Real>(1, ngs);  // 12986 
  auto qvap = W.template arr<Real>(1, ngs);  // 12986 
  auto temgkm1 = W.template arr<Real>(1, ngs);  // 12987 
  auto temgkm2 = W.template arr<Real>(1, ngs);  // 12987 
  auto temgx = W.template arr<Real>(1, ngs);  // 12988 
  auto temcgx = W.template arr<Real>(1, ngs);  // 12988 
  auto qvs = W.template arr<Real>(1, ngs);  // 12989 
  auto qis = W.template arr<Real>(1, ngs);  // 12989 
  auto qss = W.template arr<Real>(1, ngs);  // 12989 
  auto pqs = W.template arr<Real>(1, ngs);  // 12989 
  auto elv = W.template arr<Real>(1, ngs);  // 12990 
  auto elf = W.template arr<Real>(1, ngs);  // 12990 
  auto els = W.template arr<Real>(1, ngs);  // 12990 
  auto tsqr = W.template arr<Real>(1, ngs);  // 12991 
  auto ssi = W.template arr<Real>(1, ngs);  // 12991 
  auto ssw = W.template arr<Real>(1, ngs);  // 12991 
  auto ssi0 = W.template arr<Real>(1, ngs);  // 12991 
  auto qcwtmp = W.template arr<Real>(1, ngs);  // 12992 
  Real qtmp = 0;  // 12992
  auto qtot = W.template arr<Real>(1, ngs);  // 12992 
  auto qcond = W.template arr<Real>(1, ngs);  // 12993 
  Real ctmp = 0;  // 12994
  Real sctmp = 0;  // 12994
  Real cimasn = 0;  // 12995
  Real cimasx = 0;  // 12995
  Real ccimx = 0;  // 12995
  Real pid4 = 0;  // 12996
  Real cs = 0;  // 12997
  Real ds = 0;  // 12997
  Real gf7 = 0;  // 12997
  Real gf6 = 0;  // 12997
  Real gf5 = 0;  // 12997
  Real gf4 = 0;  // 12997
  Real gf3 = 0;  // 12997
  Real gf2 = 0;  // 12997
  Real gf1 = 0;  // 12997
  Real gcnup1 = 0;  // 12998
  Real gcnup2 = 0;  // 12998
  Real gf73rds = 0;  // 12999
  Real gf83rds = 0;  // 12999
  Real gamice73fac = 0;  // 13000
  Real gamsnow73fac = 0;  // 13000
  Real gf43rds = 0;  // 13001
  Real gf53rds = 0;  // 13001
  Real aradcw = 0;  // 13002
  Real bradcw = 0;  // 13002
  Real cradcw = 0;  // 13002
  Real dradcw = 0;  // 13002
  Real cwrad = 0;  // 13002
  Real rwrad = 0;  // 13002
  Real dh0 = 0;  // 13004
  auto dg0 = W.template arr<Real>(1, ngs);  // 13005 
  auto df0 = W.template arr<Real>(1, ngs);  // 13005 
  auto dhwet = W.template arr<Real>(1, ngs);  // 13006 
  auto dhlwet = W.template arr<Real>(1, ngs);  // 13006 
  auto dfwet = W.template arr<Real>(1, ngs);  // 13006 
  auto fwet1 = W.template arr<Real>(1, ngs);  // 13013 
  auto fwet2 = W.template arr<Real>(1, ngs);  // 13013 
  auto fmlt1 = W.template arr<Real>(1, ngs);  // 13014 
  auto fmlt2 = W.template arr<Real>(1, ngs);  // 13014 
  auto fmlt1e = W.template arr<Real>(1, ngs);  // 13014 
  auto fvds = W.template arr<Real>(1, ngs);  // 13015 
  auto fvce = W.template arr<Real>(1, ngs);  // 13015 
  auto fiinit = W.template arr<Real>(1, ngs);  // 13015 
  auto fvent = W.template arr<Real>(1, ngs);  // 13016 
  auto fraci = W.template arr<Real>(1, ngs);  // 13016 
  auto fracl = W.template arr<Real>(1, ngs);  // 13016 
  auto fai = W.template arr<Real>(1, ngs);  // 13018 
  auto fav = W.template arr<Real>(1, ngs);  // 13018 
  auto fbi = W.template arr<Real>(1, ngs);  // 13018 
  auto fbv = W.template arr<Real>(1, ngs);  // 13018 
  auto felv = W.template arr<Real>(1, ngs);  // 13019 
  auto fels = W.template arr<Real>(1, ngs);  // 13019 
  auto felf = W.template arr<Real>(1, ngs);  // 13019 
  auto felvcp = W.template arr<Real>(1, ngs);  // 13020 
  auto felscp = W.template arr<Real>(1, ngs);  // 13020 
  auto felfcp = W.template arr<Real>(1, ngs);  // 13020 
  auto felvpi = W.template arr<Real>(1, ngs);  // 13021 
  auto felspi = W.template arr<Real>(1, ngs);  // 13021 
  auto felfpi = W.template arr<Real>(1, ngs);  // 13021 
  auto felvs = W.template arr<Real>(1, ngs);  // 13022 
  auto felss = W.template arr<Real>(1, ngs);  // 13022 
  auto fwvdf = W.template arr<Real>(1, ngs);  // 13023 
  auto ftka = W.template arr<Real>(1, ngs);  // 13023 
  auto fthdf = W.template arr<Real>(1, ngs);  // 13023 
  auto fadvisc = W.template arr<Real>(1, ngs);  // 13024 
  auto fakvisc = W.template arr<Real>(1, ngs);  // 13024 
  auto fci = W.template arr<Real>(1, ngs);  // 13025 
  auto fcw = W.template arr<Real>(1, ngs);  // 13025 
  auto fschm = W.template arr<Real>(1, ngs);  // 13026 
  auto fpndl = W.template arr<Real>(1, ngs);  // 13026 
  auto fgamw = W.template arr<Real>(1, ngs);  // 13027 
  auto fgams = W.template arr<Real>(1, ngs);  // 13027 
  auto fcqv1 = W.template arr<Real>(1, ngs);  // 13028 
  auto fcqv2 = W.template arr<Real>(1, ngs);  // 13028 
  auto fcc3 = W.template arr<Real>(1, ngs);  // 13028 
  Real cvm = 0;  // 13030
  Real cpm = 0;  // 13030
  Real rmm = 0;  // 13030
  auto fcci = W.template arr<Real>(1, ngs);  // 13034 
  auto fcip = W.template arr<Real>(1, ngs);  // 13034 
  auto sfm1 = W.template arr<Real>(1, ngs);  // 13036 
  auto sfm2 = W.template arr<Real>(1, ngs);  // 13036 
  auto gfm1 = W.template arr<Real>(1, ngs);  // 13037 
  auto gfm2 = W.template arr<Real>(1, ngs);  // 13037 
  auto ffm1 = W.template arr<Real>(1, ngs);  // 13038 
  auto ffm2 = W.template arr<Real>(1, ngs);  // 13038 
  auto hfm1 = W.template arr<Real>(1, ngs);  // 13039 
  auto hfm2 = W.template arr<Real>(1, ngs);  // 13039 
  auto wetsfc = W.template arr<bool>(1, ngs);  // 13041 
  auto wetsfchl = W.template arr<bool>(1, ngs);  // 13041 
  auto wetsfcf = W.template arr<bool>(1, ngs);  // 13041 
  auto wetgrowth = W.template arr<bool>(1, ngs);  // 13042 
  auto wetgrowthhl = W.template arr<bool>(1, ngs);  // 13042 
  auto wetgrowthf = W.template arr<bool>(1, ngs);  // 13042 
  auto qitmp = W.template arr<Real>(1, ngs);  // 13044 
  auto qistmp = W.template arr<Real>(1, ngs);  // 13044 
  auto rzxh = W.template arr<Real>(1, ngs);  // 13046 
  auto rzxhl = W.template arr<Real>(1, ngs);  // 13046 
  auto rzxhlh = W.template arr<Real>(1, ngs);  // 13046 
  auto rzxhlf = W.template arr<Real>(1, ngs);  // 13046 
  auto rzxs = W.template arr<Real>(1, ngs);  // 13047 
  auto rzxf = W.template arr<Real>(1, ngs);  // 13047 
  auto cdh = W.template arr<Real>(1, ngs);  // 13049 
  auto cdhl = W.template arr<Real>(1, ngs);  // 13049 
  auto axx = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 13050 
  auto bxx = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 13050 
  auto vt2ave = W.template arr<Real>(1, ngs);  // 13051 
  auto qcwresv = W.template arr<Real>(1, ngs);  // 13053 
  auto ccwresv = W.template arr<Real>(1, ngs);  // 13053 
  auto lfsave = W.template arr<Real>(1, ngs, 1, 6);  // 13055 
  auto qx = W.template arr<Real>(1, ngs, P.lv, P.lhab);  // 13056 
  auto qxw = W.template arr<Real>(1, ngs, P.ls, P.lhab);  // 13057 
  auto qxwlg = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 13058 
  auto chxf = W.template arr<Real>(1, ngs, P.lh, P.lhab);  // 13059 
  auto cx = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13060 
  auto cxmxd = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13061 
  auto qxmxd = W.template arr<Real>(1, ngs, P.lv, P.lhab);  // 13062 
  auto scx = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13063 
  auto xv = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13064 
  auto vtxbar = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);  // 13065 
  auto xmas = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13066 
  auto xdn = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13067 
  auto xdntmp = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13068 
  auto cdxgs = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13069 
  auto xdia = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);  // 13070 
  auto vtwtdia = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 13071 
  auto rarx = W.template arr<Real>(1, ngs, P.ls, P.lhab);  // 13072 
  auto vx = W.template arr<Real>(1, ngs, P.li, P.lhab);  // 13073 
  auto rimdn = W.template arr<Real>(1, ngs, P.li, P.lhab);  // 13074 
  auto raindn = W.template arr<Real>(1, ngs, P.li, P.lhab);  // 13075 
  auto alpha = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13076 
  auto dab0lh = W.template arr<Real>(1, ngs, P.lc, P.lhab, P.lc, P.lhab);  // 13077 
  auto dab1lh = W.template arr<Real>(1, ngs, P.lc, P.lhab, P.lc, P.lhab);  // 13078 
  auto zx = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 13079 
  auto zxmxd = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 13080 
  auto g1x = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 13081 
  Real g1xmax = 0;  // 13082
  Real g1xmin = 0;  // 13082
  auto qsimxdep = W.template arr<Real>(1, ngs);  // 13083 
  auto qsimxsub = W.template arr<Real>(1, ngs);  // 13084 
  auto qrtmp = W.template arr<Real>(1, ngs);  // 13086 
  auto qvtmp = W.template arr<Real>(1, ngs);  // 13086 
  auto qctmp = W.template arr<Real>(1, ngs);  // 13086 
  Real felvcptmp = 0;  // 13087
  Real felscptmp = 0;  // 13087
  Real qsstmp = 0;  // 13087
  Real thetatmp = 0;  // 13088
  Real thetaptmp = 0;  // 13088
  Real temcgtmp = 0;  // 13088
  Real qvaptmp = 0;  // 13088
  Real qvstmp = 0;  // 13089
  Real qisstmp = 0;  // 13089
  Real qvptmp = 0;  // 13089
  Real qitmp1 = 0;  // 13089
  Real qctmp1 = 0;  // 13089
  Real galphrout = 0;  // 13091
  auto ventrx = W.template arr<Real>(1, ngs);  // 13093 
  auto ventrxn = W.template arr<Real>(1, ngs);  // 13094 
  Real g1shr = 0;  // 13095
  Real alphashr = 0;  // 13095
  Real g1mlr = 0;  // 13096
  Real alphamlr = 0;  // 13096
  Real g1smlr = 0;  // 13097
  Real alphasmlr = 0;  // 13097
  Real massfacshr = 0;  // 13098
  Real massfacmlr = 0;  // 13098
  Real qhgt8mm = 0;  // 13100
  Real qhwgt8mm = 0;  // 13101
  Real qhgt10mm = 0;  // 13102
  Real qhgt20mm = 0;  // 13103
  Real fwmhtmp = 0;  // 13104
  Real dtmp = 0;  // 13108
  auto swvent = W.template arr<Real>(1, ngs);  // 13110 
  auto hwvent = W.template arr<Real>(1, ngs);  // 13110 
  auto rwvent = W.template arr<Real>(1, ngs);  // 13110 
  auto hlvent = W.template arr<Real>(1, ngs);  // 13110 
  auto hwventy = W.template arr<Real>(1, ngs);  // 13110 
  auto hlventy = W.template arr<Real>(1, ngs);  // 13110 
  auto rwventz = W.template arr<Real>(1, ngs);  // 13110 
  Real hxventtmp = 0;  // 13111
  auto hlventinc = W.template arr<Real>(1, ngs);  // 13112 
  auto hwventinc = W.template arr<Real>(1, ngs);  // 13112 
  int numdiam = 0;  // 13114
  auto hwvent0 = W.template arr<Real>(1, (ndiam + 4));  // 13115 
  Real hlvent0 = 0;  // 13115
  Real hwvent1 = 0;  // 13116
  Real hlvent1 = 0;  // 13116
  Real hwvent2 = 0;  // 13117
  Real hlvent2 = 0;  // 13117
  Real gama0 = 0;  // 13118
  Real gamb0 = 0;  // 13118
  Real gama1 = 0;  // 13119
  Real gamb1 = 0;  // 13119
  Real gama2 = 0;  // 13120
  Real gamb2 = 0;  // 13120
  auto mltdiam = W.template arr<Real>(1, (ndiam + 4));  // 13122 
  Real mltmass0inv = 0;  // 13123
  Real mltmass1inv = 0;  // 13123
  Real mltmass2inv = 0;  // 13123
  Real mltmass1cgs = 0;  // 13123
  Real mltmass2cgs = 0;  // 13123
  Real mltmass3inv = 0;  // 13123
  Real mltmass3cgs = 0;  // 13123
  Real qhmlr0 = 0;  // 13124
  Real qhmlr05 = 0;  // 13124
  Real qhmlr1 = 0;  // 13124
  Real qhmlr2 = 0;  // 13124
  Real qhmlr3 = 0;  // 13124
  Real qhmlr12 = 0;  // 13124
  Real qhmlr23 = 0;  // 13124
  Real qhlmlr0 = 0;  // 13125
  Real qhlmlr05 = 0;  // 13125
  Real qhlmlr1 = 0;  // 13125
  Real qhlmlr2 = 0;  // 13125
  Real qhlmlr3 = 0;  // 13125
  Real qhlmlr12 = 0;  // 13125
  Real qhlmlr23 = 0;  // 13125
  Real qxd1 = 0;  // 13126
  Real cxd1 = 0;  // 13126
  Real zxd1 = 0;  // 13126
  Real qxd05 = 0;  // 13127
  Real cxd05 = 0;  // 13127
  Real qrbreak = 0;  // 13128
  Real crbreaksmall = 0;  // 13128
  Real crbreak = 0;  // 13128
  Real zrbreak = 0;  // 13128
  Real breakbin = 0;  // 13128
  auto qxd = W.template arr<Real>(1, (ndiam + 4));  // 13130 
  auto cxd = W.template arr<Real>(1, (ndiam + 4));  // 13130 
  auto qhml = W.template arr<Real>(1, (ndiam + 4));  // 13130 
  auto qhml0 = W.template arr<Real>(1, (ndiam + 4));  // 13130 
  auto dqxd = W.template arr<Real>(1, (ndiam + 4));  // 13131 
  auto dcxd = W.template arr<Real>(1, (ndiam + 4));  // 13131 
  auto dqhml = W.template arr<Real>(1, (ndiam + 4));  // 13131 
  auto civent = W.template arr<Real>(1, ngs);  // 13134 
  auto isvent = W.template arr<Real>(1, ngs);  // 13135 
  auto xmascw = W.template arr<Real>(1, ngs);  // 13137 
  Real dnmx = 0;  // 13139
  auto xdiamxmas = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 13140 
  auto cilen = W.template arr<Real>(1, ngs);  // 13142 
  auto rwcap = W.template arr<Real>(1, ngs);  // 13145 
  auto swcap = W.template arr<Real>(1, ngs);  // 13145 
  auto hwcap = W.template arr<Real>(1, ngs);  // 13146 
  auto hlcap = W.template arr<Real>(1, ngs);  // 13147 
  auto cicap = W.template arr<Real>(1, ngs);  // 13148 
  auto iscap = W.template arr<Real>(1, ngs);  // 13149 
  auto qvimxd = W.template arr<Real>(1, ngs);  // 13151 
  auto qimxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qismxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qcmxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qrmxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qsmxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qhmxd = W.template arr<Real>(1, ngs);  // 13152 
  auto qhlmxd = W.template arr<Real>(1, ngs);  // 13152 
  auto cimxd = W.template arr<Real>(1, ngs);  // 13153 
  auto ccmxd = W.template arr<Real>(1, ngs);  // 13153 
  auto crmxd = W.template arr<Real>(1, ngs);  // 13153 
  auto csmxd = W.template arr<Real>(1, ngs);  // 13153 
  auto chmxd = W.template arr<Real>(1, ngs);  // 13153 
  auto cionpmxd = W.template arr<Real>(1, ngs);  // 13154 
  auto cionnmxd = W.template arr<Real>(1, ngs);  // 13154 
  auto clionpmxd = W.template arr<Real>(1, ngs);  // 13155 
  auto clionnmxd = W.template arr<Real>(1, ngs);  // 13155 
  auto chmul1 = W.template arr<Real>(1, ngs);  // 13163 
  auto chlmul1 = W.template arr<Real>(1, ngs);  // 13163 
  auto csmul1 = W.template arr<Real>(1, ngs);  // 13163 
  auto csmul = W.template arr<Real>(1, ngs);  // 13163 
  auto qhmul1 = W.template arr<Real>(1, ngs);  // 13164 
  auto qhlmul1 = W.template arr<Real>(1, ngs);  // 13164 
  auto qsmul1 = W.template arr<Real>(1, ngs);  // 13164 
  auto qsmul = W.template arr<Real>(1, ngs);  // 13164 
  auto csplinter = W.template arr<Real>(1, ngs);  // 13167 
  auto qsplinter = W.template arr<Real>(1, ngs);  // 13167 
  auto csplinter2 = W.template arr<Real>(1, ngs);  // 13168 
  auto qsplinter2 = W.template arr<Real>(1, ngs);  // 13168 
  auto chlcnh = W.template arr<Real>(1, ngs);  // 13173 
  auto vhlcnh = W.template arr<Real>(1, ngs);  // 13173 
  auto vhlcnhl = W.template arr<Real>(1, ngs);  // 13173 
  auto chlcnhhl = W.template arr<Real>(1, ngs);  // 13174 
  auto cracif = W.template arr<Real>(1, ngs);  // 13175 
  auto ciacrf = W.template arr<Real>(1, ngs);  // 13175 
  auto cracr = W.template arr<Real>(1, ngs);  // 13176 
  auto ciint = W.template arr<Real>(1, ngs);  // 13179 
  auto crfrz = W.template arr<Real>(1, ngs);  // 13179 
  auto crfrzf = W.template arr<Real>(1, ngs);  // 13179 
  auto crfrzs = W.template arr<Real>(1, ngs);  // 13179 
  auto cicint = W.template arr<Real>(1, ngs);  // 13180 
  auto cipint = W.template arr<Real>(1, ngs);  // 13181 
  auto ciacw = W.template arr<Real>(1, ngs);  // 13182 
  auto cwacii = W.template arr<Real>(1, ngs);  // 13182 
  auto ciacr = W.template arr<Real>(1, ngs);  // 13183 
  auto craci = W.template arr<Real>(1, ngs);  // 13183 
  auto csacw = W.template arr<Real>(1, ngs);  // 13184 
  auto csacr = W.template arr<Real>(1, ngs);  // 13185 
  auto csaci = W.template arr<Real>(1, ngs);  // 13186 
  auto csacs = W.template arr<Real>(1, ngs);  // 13186 
  auto cracw = W.template arr<Real>(1, ngs);  // 13187 
  auto chacw = W.template arr<Real>(1, ngs);  // 13188 
  auto chacr = W.template arr<Real>(1, ngs);  // 13188 
  auto chlacw = W.template arr<Real>(1, ngs);  // 13189 
  auto chaci = W.template arr<Real>(1, ngs);  // 13190 
  auto chacs = W.template arr<Real>(1, ngs);  // 13190 
  auto chlacr = W.template arr<Real>(1, ngs);  // 13192 
  auto chlaci = W.template arr<Real>(1, ngs);  // 13193 
  auto chlacs = W.template arr<Real>(1, ngs);  // 13193 
  auto crcnw = W.template arr<Real>(1, ngs);  // 13194 
  auto cidpv = W.template arr<Real>(1, ngs);  // 13195 
  auto cisbv = W.template arr<Real>(1, ngs);  // 13195 
  auto cisdpv = W.template arr<Real>(1, ngs);  // 13196 
  auto cissbv = W.template arr<Real>(1, ngs);  // 13196 
  auto cimlr = W.template arr<Real>(1, ngs);  // 13197 
  auto cismlr = W.template arr<Real>(1, ngs);  // 13197 
  auto chlsbv = W.template arr<Real>(1, ngs);  // 13199 
  auto chldpv = W.template arr<Real>(1, ngs);  // 13199 
  auto chlmlr = W.template arr<Real>(1, ngs);  // 13200 
  auto chlmlrr = W.template arr<Real>(1, ngs);  // 13200 
  auto chlfmlr = W.template arr<Real>(1, ngs);  // 13201 
  auto chlshr = W.template arr<Real>(1, ngs);  // 13203 
  auto chlshrr = W.template arr<Real>(1, ngs);  // 13203 
  auto chdpv = W.template arr<Real>(1, ngs);  // 13206 
  auto chsbv = W.template arr<Real>(1, ngs);  // 13206 
  auto chmlr = W.template arr<Real>(1, ngs);  // 13207 
  auto chcev = W.template arr<Real>(1, ngs);  // 13207 
  auto chmlrr = W.template arr<Real>(1, ngs);  // 13208 
  auto chshr = W.template arr<Real>(1, ngs);  // 13209 
  auto chshrr = W.template arr<Real>(1, ngs);  // 13209 
  auto csdpv = W.template arr<Real>(1, ngs);  // 13211 
  auto cssbv = W.template arr<Real>(1, ngs);  // 13211 
  auto csmlr = W.template arr<Real>(1, ngs);  // 13212 
  auto csmlrr = W.template arr<Real>(1, ngs);  // 13212 
  auto cscev = W.template arr<Real>(1, ngs);  // 13212 
  auto csshr = W.template arr<Real>(1, ngs);  // 13213 
  auto csshrr = W.template arr<Real>(1, ngs);  // 13213 
  auto crcev = W.template arr<Real>(1, ngs);  // 13215 
  auto crshr = W.template arr<Real>(1, ngs);  // 13216 
  auto cwshw = W.template arr<Real>(1, ngs);  // 13217 
  auto qwshw = W.template arr<Real>(1, ngs);  // 13217 
  auto qrcnw = W.template arr<Real>(1, ngs);  // 13224 
  auto qwcnr = W.template arr<Real>(1, ngs);  // 13224 
  auto zrcnw = W.template arr<Real>(1, ngs);  // 13225 
  auto zracr = W.template arr<Real>(1, ngs);  // 13225 
  auto zracw = W.template arr<Real>(1, ngs);  // 13225 
  auto zrcev = W.template arr<Real>(1, ngs);  // 13225 
  auto qracw = W.template arr<Real>(1, ngs);  // 13227 
  auto qiacw = W.template arr<Real>(1, ngs);  // 13228 
  auto qsacw = W.template arr<Real>(1, ngs);  // 13230 
  auto qhacw = W.template arr<Real>(1, ngs);  // 13231 
  auto qhlacw = W.template arr<Real>(1, ngs);  // 13232 
  Real qxacwtmp = 0;  // 13232
  Real qxacrtmp = 0;  // 13232
  Real qxacitmp = 0;  // 13232
  Real qxacstmp = 0;  // 13232
  auto vhacw = W.template arr<Real>(1, ngs);  // 13233 
  auto vsacw = W.template arr<Real>(1, ngs);  // 13233 
  auto vhlacw = W.template arr<Real>(1, ngs);  // 13233 
  auto vhlacr = W.template arr<Real>(1, ngs);  // 13233 
  auto qfcev = W.template arr<Real>(1, ngs);  // 13235 
  auto qfmul1 = W.template arr<Real>(1, ngs);  // 13236 
  auto cfmul1 = W.template arr<Real>(1, ngs);  // 13236 
  auto qsacws = W.template arr<Real>(1, ngs);  // 13238 
  auto qsacr = W.template arr<Real>(1, ngs);  // 13243 
  auto qracs = W.template arr<Real>(1, ngs);  // 13243 
  auto qhacr = W.template arr<Real>(1, ngs);  // 13244 
  auto qhacrmlr = W.template arr<Real>(1, ngs);  // 13244 
  auto qhacwmlr = W.template arr<Real>(1, ngs);  // 13244 
  auto vhacr = W.template arr<Real>(1, ngs);  // 13245 
  auto zhacr = W.template arr<Real>(1, ngs);  // 13245 
  auto zhacrf = W.template arr<Real>(1, ngs);  // 13245 
  auto zrach = W.template arr<Real>(1, ngs);  // 13245 
  auto zrachl = W.template arr<Real>(1, ngs);  // 13245 
  auto qiacr = W.template arr<Real>(1, ngs);  // 13246 
  auto qraci = W.template arr<Real>(1, ngs);  // 13246 
  auto ziacr = W.template arr<Real>(1, ngs);  // 13248 
  auto qracif = W.template arr<Real>(1, ngs);  // 13250 
  auto qiacrf = W.template arr<Real>(1, ngs);  // 13250 
  auto qiacrs = W.template arr<Real>(1, ngs);  // 13250 
  auto ciacrs = W.template arr<Real>(1, ngs);  // 13250 
  auto qhlacr = W.template arr<Real>(1, ngs);  // 13252 
  auto qhlacrmlr = W.template arr<Real>(1, ngs);  // 13252 
  auto qhlacwmlr = W.template arr<Real>(1, ngs);  // 13252 
  auto qsacrs = W.template arr<Real>(1, ngs);  // 13253 
  auto qsaci = W.template arr<Real>(1, ngs);  // 13257 
  auto qsacis = W.template arr<Real>(1, ngs);  // 13258 
  auto csacis = W.template arr<Real>(1, ngs);  // 13259 
  auto qhaci = W.template arr<Real>(1, ngs);  // 13260 
  auto qhacs = W.template arr<Real>(1, ngs);  // 13261 
  auto qhacis = W.template arr<Real>(1, ngs);  // 13263 
  auto chacis = W.template arr<Real>(1, ngs);  // 13264 
  auto chacis0 = W.template arr<Real>(1, ngs);  // 13265 
  auto csaci0 = W.template arr<Real>(1, ngs);  // 13267 
  auto csacis0 = W.template arr<Real>(1, ngs);  // 13268 
  auto chaci0 = W.template arr<Real>(1, ngs);  // 13269 
  auto chacs0 = W.template arr<Real>(1, ngs);  // 13270 
  auto chlaci0 = W.template arr<Real>(1, ngs);  // 13271 
  auto chlacis = W.template arr<Real>(1, ngs);  // 13272 
  auto chlacis0 = W.template arr<Real>(1, ngs);  // 13273 
  auto chlacs0 = W.template arr<Real>(1, ngs);  // 13274 
  auto qsaci0 = W.template arr<Real>(1, ngs);  // 13276 
  auto qsacis0 = W.template arr<Real>(1, ngs);  // 13277 
  auto qhaci0 = W.template arr<Real>(1, ngs);  // 13278 
  auto qhacis0 = W.template arr<Real>(1, ngs);  // 13279 
  auto qhacs0 = W.template arr<Real>(1, ngs);  // 13280 
  auto qhlaci0 = W.template arr<Real>(1, ngs);  // 13281 
  auto qhlacis0 = W.template arr<Real>(1, ngs);  // 13282 
  auto qhlacs0 = W.template arr<Real>(1, ngs);  // 13283 
  auto qhlaci = W.template arr<Real>(1, ngs);  // 13285 
  auto qhlacis = W.template arr<Real>(1, ngs);  // 13286 
  auto qhlacs = W.template arr<Real>(1, ngs);  // 13287 
  auto qrfrz = W.template arr<Real>(1, ngs);  // 13291 
  auto zrfrz = W.template arr<Real>(1, ngs);  // 13292 
  auto zrfrzf = W.template arr<Real>(1, ngs);  // 13292 
  auto zrfrzs = W.template arr<Real>(1, ngs);  // 13292 
  auto ziacrf = W.template arr<Real>(1, ngs);  // 13293 
  auto zhcnsh = W.template arr<Real>(1, ngs);  // 13293 
  auto zhcnih = W.template arr<Real>(1, ngs);  // 13293 
  auto zhacw = W.template arr<Real>(1, ngs);  // 13294 
  auto zhacs = W.template arr<Real>(1, ngs);  // 13294 
  auto zhaci = W.template arr<Real>(1, ngs);  // 13294 
  auto zhmlr = W.template arr<Real>(1, ngs);  // 13295 
  auto zhdsv = W.template arr<Real>(1, ngs);  // 13295 
  auto zhsbv = W.template arr<Real>(1, ngs);  // 13295 
  auto zhlcnh = W.template arr<Real>(1, ngs);  // 13295 
  auto zhshr = W.template arr<Real>(1, ngs);  // 13295 
  auto zfacw = W.template arr<Real>(1, ngs);  // 13296 
  auto zfacs = W.template arr<Real>(1, ngs);  // 13296 
  auto zfaci = W.template arr<Real>(1, ngs);  // 13296 
  auto zfmlr = W.template arr<Real>(1, ngs);  // 13297 
  auto zfdsv = W.template arr<Real>(1, ngs);  // 13297 
  auto zfsbv = W.template arr<Real>(1, ngs);  // 13297 
  auto zhlcnf = W.template arr<Real>(1, ngs);  // 13297 
  auto zfshr = W.template arr<Real>(1, ngs);  // 13297 
  auto zfshrr = W.template arr<Real>(1, ngs);  // 13297 
  Real zhmlrtmp = 0;  // 13298
  Real zhmlr0inf = 0;  // 13298
  Real zhlmlr0inf = 0;  // 13298
  auto zhmlrr = W.template arr<Real>(1, ngs);  // 13299 
  auto zhlmlrr = W.template arr<Real>(1, ngs);  // 13299 
  auto zhshrr = W.template arr<Real>(1, ngs);  // 13299 
  auto zhlshrr = W.template arr<Real>(1, ngs);  // 13299 
  auto zfmlrr = W.template arr<Real>(1, ngs);  // 13299 
  auto zsmlrr = W.template arr<Real>(1, ngs);  // 13301 
  auto zsshr = W.template arr<Real>(1, ngs);  // 13301 
  auto zsshrr = W.template arr<Real>(1, ngs);  // 13301 
  auto zhcns = W.template arr<Real>(1, ngs);  // 13302 
  auto zhcni = W.template arr<Real>(1, ngs);  // 13302 
  auto zhwdn = W.template arr<Real>(1, ngs);  // 13303 
  auto zfwdn = W.template arr<Real>(1, ngs);  // 13303 
  auto zhldn = W.template arr<Real>(1, ngs);  // 13304 
  auto zhlacw = W.template arr<Real>(1, ngs);  // 13306 
  auto zhlacs = W.template arr<Real>(1, ngs);  // 13306 
  auto zhlacr = W.template arr<Real>(1, ngs);  // 13306 
  auto zhlmlr = W.template arr<Real>(1, ngs);  // 13307 
  auto zhldsv = W.template arr<Real>(1, ngs);  // 13307 
  auto zhlsbv = W.template arr<Real>(1, ngs);  // 13307 
  auto zhlshr = W.template arr<Real>(1, ngs);  // 13307 
  auto vrfrzf = W.template arr<Real>(1, ngs);  // 13310 
  auto viacrf = W.template arr<Real>(1, ngs);  // 13310 
  auto qrfrzs = W.template arr<Real>(1, ngs);  // 13311 
  auto qrfrzf = W.template arr<Real>(1, ngs);  // 13311 
  auto qwfrz = W.template arr<Real>(1, ngs);  // 13312 
  auto qwctfz = W.template arr<Real>(1, ngs);  // 13312 
  auto cwfrz = W.template arr<Real>(1, ngs);  // 13313 
  auto cwctfz = W.template arr<Real>(1, ngs);  // 13313 
  auto qwfrzis = W.template arr<Real>(1, ngs);  // 13314 
  auto qwctfzis = W.template arr<Real>(1, ngs);  // 13314 
  auto cwfrzis = W.template arr<Real>(1, ngs);  // 13315 
  auto cwctfzis = W.template arr<Real>(1, ngs);  // 13315 
  auto qwfrzc = W.template arr<Real>(1, ngs);  // 13316 
  auto qwctfzc = W.template arr<Real>(1, ngs);  // 13316 
  auto cwfrzc = W.template arr<Real>(1, ngs);  // 13317 
  auto cwctfzc = W.template arr<Real>(1, ngs);  // 13317 
  auto qwfrzp = W.template arr<Real>(1, ngs);  // 13318 
  auto qwctfzp = W.template arr<Real>(1, ngs);  // 13318 
  auto cwfrzp = W.template arr<Real>(1, ngs);  // 13319 
  auto cwctfzp = W.template arr<Real>(1, ngs);  // 13319 
  auto xcolmn = W.template arr<Real>(1, ngs);  // 13320 
  auto xplate = W.template arr<Real>(1, ngs);  // 13320 
  auto ciihr = W.template arr<Real>(1, ngs);  // 13321 
  auto qiihr = W.template arr<Real>(1, ngs);  // 13321 
  auto cicichr = W.template arr<Real>(1, ngs);  // 13322 
  auto qicichr = W.template arr<Real>(1, ngs);  // 13322 
  auto cipiphr = W.template arr<Real>(1, ngs);  // 13323 
  auto qipiphr = W.template arr<Real>(1, ngs);  // 13323 
  auto qscni = W.template arr<Real>(1, ngs);  // 13324 
  auto cscni = W.template arr<Real>(1, ngs);  // 13324 
  auto cscnis = W.template arr<Real>(1, ngs);  // 13324 
  auto qscnvi = W.template arr<Real>(1, ngs);  // 13325 
  auto cscnvi = W.template arr<Real>(1, ngs);  // 13325 
  auto cscnvis = W.template arr<Real>(1, ngs);  // 13325 
  auto qhcns = W.template arr<Real>(1, ngs);  // 13326 
  auto chcns = W.template arr<Real>(1, ngs);  // 13326 
  auto chcnsh = W.template arr<Real>(1, ngs);  // 13326 
  auto vhcns = W.template arr<Real>(1, ngs);  // 13326 
  auto qscnh = W.template arr<Real>(1, ngs);  // 13327 
  auto cscnh = W.template arr<Real>(1, ngs);  // 13327 
  auto vscnh = W.template arr<Real>(1, ngs);  // 13327 
  auto qhcni = W.template arr<Real>(1, ngs);  // 13328 
  auto chcni = W.template arr<Real>(1, ngs);  // 13328 
  auto chcnih = W.template arr<Real>(1, ngs);  // 13328 
  auto vhcni = W.template arr<Real>(1, ngs);  // 13328 
  auto qiint = W.template arr<Real>(1, ngs);  // 13329 
  auto qipipnt = W.template arr<Real>(1, ngs);  // 13329 
  auto qicicnt = W.template arr<Real>(1, ngs);  // 13329 
  auto cninm = W.template arr<Real>(1, ngs);  // 13330 
  auto cnina = W.template arr<Real>(1, ngs);  // 13330 
  auto cninp = W.template arr<Real>(1, ngs);  // 13330 
  auto wvel = W.template arr<Real>(1, ngs);  // 13330 
  auto wvelkm1 = W.template arr<Real>(1, ngs);  // 13330 
  auto tke = W.template arr<Real>(1, ngs);  // 13331 
  auto uvel = W.template arr<Real>(1, ngs);  // 13332 
  auto vvel = W.template arr<Real>(1, ngs);  // 13332 
  auto qidpv = W.template arr<Real>(1, ngs);  // 13334 
  auto qisbv = W.template arr<Real>(1, ngs);  // 13334 
  auto qimlr = W.template arr<Real>(1, ngs);  // 13335 
  auto qidsv = W.template arr<Real>(1, ngs);  // 13335 
  auto qisdsv = W.template arr<Real>(1, ngs);  // 13335 
  auto qidsvp = W.template arr<Real>(1, ngs);  // 13335 
  auto qismlr = W.template arr<Real>(1, ngs);  // 13336 
  auto qhldpv = W.template arr<Real>(1, ngs);  // 13340 
  auto qhlsbv = W.template arr<Real>(1, ngs);  // 13340 
  auto qhlmlr = W.template arr<Real>(1, ngs);  // 13341 
  auto qhldsv = W.template arr<Real>(1, ngs);  // 13341 
  auto qhlmlrsave = W.template arr<Real>(1, ngs);  // 13341 
  auto qhlwet = W.template arr<Real>(1, ngs);  // 13342 
  auto qhldry = W.template arr<Real>(1, ngs);  // 13342 
  auto qhlshr = W.template arr<Real>(1, ngs);  // 13342 
  Real qxwettmp = 0;  // 13342
  auto qrfz = W.template arr<Real>(1, ngs);  // 13344 
  auto qsfz = W.template arr<Real>(1, ngs);  // 13344 
  auto qhfz = W.template arr<Real>(1, ngs);  // 13344 
  auto qhlfz = W.template arr<Real>(1, ngs);  // 13344 
  auto qffz = W.template arr<Real>(1, ngs);  // 13345 
  auto qhdpv = W.template arr<Real>(1, ngs);  // 13347 
  auto qhsbv = W.template arr<Real>(1, ngs);  // 13347 
  auto qhmlr = W.template arr<Real>(1, ngs);  // 13348 
  auto qhdsv = W.template arr<Real>(1, ngs);  // 13348 
  auto qhcev = W.template arr<Real>(1, ngs);  // 13348 
  auto qhcndv = W.template arr<Real>(1, ngs);  // 13348 
  auto qhevv = W.template arr<Real>(1, ngs);  // 13348 
  auto qhlcev = W.template arr<Real>(1, ngs);  // 13349 
  auto chlcev = W.template arr<Real>(1, ngs);  // 13349 
  auto qhwet = W.template arr<Real>(1, ngs);  // 13350 
  auto qhdry = W.template arr<Real>(1, ngs);  // 13350 
  auto qhshr = W.template arr<Real>(1, ngs);  // 13350 
  auto qhshrp = W.template arr<Real>(1, ngs);  // 13351 
  auto qhshh = W.template arr<Real>(1, ngs);  // 13352 
  auto qhmlh = W.template arr<Real>(1, ngs);  // 13353 
  auto qhfzh = W.template arr<Real>(1, ngs);  // 13354 
  auto qffzf = W.template arr<Real>(1, ngs);  // 13355 
  auto qhlfzhl = W.template arr<Real>(1, ngs);  // 13356 
  auto qhmlrlg = W.template arr<Real>(1, ngs);  // 13358 
  auto qhlmlrlg = W.template arr<Real>(1, ngs);  // 13358 
  auto qhfzhlg = W.template arr<Real>(1, ngs);  // 13359 
  auto qhlfzhllg = W.template arr<Real>(1, ngs);  // 13360 
  auto qhlcevlg = W.template arr<Real>(1, ngs);  // 13361 
  auto chlcevlg = W.template arr<Real>(1, ngs);  // 13361 
  auto qhcevlg = W.template arr<Real>(1, ngs);  // 13362 
  auto chcevlg = W.template arr<Real>(1, ngs);  // 13362 
  auto vhfzh = W.template arr<Real>(1, ngs);  // 13364 
  auto vffzf = W.template arr<Real>(1, ngs);  // 13364 
  auto vhlfzhl = W.template arr<Real>(1, ngs);  // 13365 
  auto vhshdr = W.template arr<Real>(1, ngs);  // 13367 
  auto vhlshdr = W.template arr<Real>(1, ngs);  // 13368 
  auto vhmlr = W.template arr<Real>(1, ngs);  // 13369 
  auto vhlmlr = W.template arr<Real>(1, ngs);  // 13370 
  auto vhsoak = W.template arr<Real>(1, ngs);  // 13371 
  auto vhlsoak = W.template arr<Real>(1, ngs);  // 13372 
  auto qsdpv = W.template arr<Real>(1, ngs);  // 13375 
  auto qssbv = W.template arr<Real>(1, ngs);  // 13375 
  auto qsmlr = W.template arr<Real>(1, ngs);  // 13376 
  auto qsdsv = W.template arr<Real>(1, ngs);  // 13376 
  auto qscev = W.template arr<Real>(1, ngs);  // 13376 
  auto qscndv = W.template arr<Real>(1, ngs);  // 13376 
  auto qsevv = W.template arr<Real>(1, ngs);  // 13376 
  auto qswet = W.template arr<Real>(1, ngs);  // 13377 
  auto qsdry = W.template arr<Real>(1, ngs);  // 13377 
  auto qsshr = W.template arr<Real>(1, ngs);  // 13377 
  auto qsshrp = W.template arr<Real>(1, ngs);  // 13378 
  auto qsfzs = W.template arr<Real>(1, ngs);  // 13379 
  auto qipdpv = W.template arr<Real>(1, ngs);  // 13382 
  auto qipsbv = W.template arr<Real>(1, ngs);  // 13382 
  auto qipmlr = W.template arr<Real>(1, ngs);  // 13383 
  auto qipdsv = W.template arr<Real>(1, ngs);  // 13383 
  auto qirdpv = W.template arr<Real>(1, ngs);  // 13385 
  auto qirsbv = W.template arr<Real>(1, ngs);  // 13385 
  auto qirmlr = W.template arr<Real>(1, ngs);  // 13386 
  auto qirdsv = W.template arr<Real>(1, ngs);  // 13386 
  auto qirmlw = W.template arr<Real>(1, ngs);  // 13386 
  auto qgldpv = W.template arr<Real>(1, ngs);  // 13388 
  auto qglsbv = W.template arr<Real>(1, ngs);  // 13388 
  auto qglmlr = W.template arr<Real>(1, ngs);  // 13389 
  auto qgldsv = W.template arr<Real>(1, ngs);  // 13389 
  auto qglwet = W.template arr<Real>(1, ngs);  // 13390 
  auto qgldry = W.template arr<Real>(1, ngs);  // 13390 
  auto qglshr = W.template arr<Real>(1, ngs);  // 13390 
  auto qglshrp = W.template arr<Real>(1, ngs);  // 13391 
  auto qgmdpv = W.template arr<Real>(1, ngs);  // 13393 
  auto qgmsbv = W.template arr<Real>(1, ngs);  // 13393 
  auto qgmmlr = W.template arr<Real>(1, ngs);  // 13394 
  auto qgmdsv = W.template arr<Real>(1, ngs);  // 13394 
  auto qgmwet = W.template arr<Real>(1, ngs);  // 13395 
  auto qgmdry = W.template arr<Real>(1, ngs);  // 13395 
  auto qgmshr = W.template arr<Real>(1, ngs);  // 13395 
  auto qgmshrp = W.template arr<Real>(1, ngs);  // 13396 
  auto qghdpv = W.template arr<Real>(1, ngs);  // 13397 
  auto qghsbv = W.template arr<Real>(1, ngs);  // 13397 
  auto qghmlr = W.template arr<Real>(1, ngs);  // 13398 
  auto qghdsv = W.template arr<Real>(1, ngs);  // 13398 
  auto qghwet = W.template arr<Real>(1, ngs);  // 13399 
  auto qghdry = W.template arr<Real>(1, ngs);  // 13399 
  auto qghshr = W.template arr<Real>(1, ngs);  // 13399 
  auto qghshrp = W.template arr<Real>(1, ngs);  // 13400 
  auto qrztot = W.template arr<Real>(1, ngs);  // 13402 
  auto qrzmax = W.template arr<Real>(1, ngs);  // 13402 
  auto qrzfac = W.template arr<Real>(1, ngs);  // 13402 
  auto qrcev = W.template arr<Real>(1, ngs);  // 13403 
  auto qrshr = W.template arr<Real>(1, ngs);  // 13404 
  auto fsw = W.template arr<Real>(1, ngs);  // 13405 
  auto fhw = W.template arr<Real>(1, ngs);  // 13405 
  auto fhlw = W.template arr<Real>(1, ngs);  // 13405 
  auto ffw = W.template arr<Real>(1, ngs);  // 13405 
  auto fswmax = W.template arr<Real>(1, ngs);  // 13406 
  auto fhwmax = W.template arr<Real>(1, ngs);  // 13406 
  auto fhlwmax = W.template arr<Real>(1, ngs);  // 13406 
  auto ffwmax = W.template arr<Real>(1, ngs);  // 13407 
  auto qhcnf = W.template arr<Real>(1, ngs);  // 13408 
  auto qhlcnh = W.template arr<Real>(1, ngs);  // 13409 
  auto qhcngh = W.template arr<Real>(1, ngs);  // 13410 
  auto qhcngm = W.template arr<Real>(1, ngs);  // 13410 
  auto qhcngl = W.template arr<Real>(1, ngs);  // 13410 
  auto qhcnhl = W.template arr<Real>(1, ngs);  // 13412 
  auto chcnhl = W.template arr<Real>(1, ngs);  // 13412 
  auto zhcnhl = W.template arr<Real>(1, ngs);  // 13412 
  auto vhcnhl = W.template arr<Real>(1, ngs);  // 13412 
  auto eiw = W.template arr<Real>(1, ngs);  // 13414 
  auto eii = W.template arr<Real>(1, ngs);  // 13414 
  auto eiri = W.template arr<Real>(1, ngs);  // 13414 
  auto eipir = W.template arr<Real>(1, ngs);  // 13414 
  auto eisw = W.template arr<Real>(1, ngs);  // 13414 
  auto erw = W.template arr<Real>(1, ngs);  // 13415 
  auto esw = W.template arr<Real>(1, ngs);  // 13415 
  auto eglw = W.template arr<Real>(1, ngs);  // 13415 
  auto eghw = W.template arr<Real>(1, ngs);  // 13415 
  auto efw = W.template arr<Real>(1, ngs);  // 13415 
  auto ehxw = W.template arr<Real>(1, ngs);  // 13416 
  auto ehlw = W.template arr<Real>(1, ngs);  // 13416 
  auto egmw = W.template arr<Real>(1, ngs);  // 13416 
  auto ehw = W.template arr<Real>(1, ngs);  // 13416 
  auto err = W.template arr<Real>(1, ngs);  // 13417 
  auto esr = W.template arr<Real>(1, ngs);  // 13417 
  auto eglr = W.template arr<Real>(1, ngs);  // 13417 
  auto eghr = W.template arr<Real>(1, ngs);  // 13417 
  auto efr = W.template arr<Real>(1, ngs);  // 13417 
  auto ehxr = W.template arr<Real>(1, ngs);  // 13418 
  auto ehlr = W.template arr<Real>(1, ngs);  // 13418 
  auto egmr = W.template arr<Real>(1, ngs);  // 13418 
  auto eri = W.template arr<Real>(1, ngs);  // 13419 
  auto esi = W.template arr<Real>(1, ngs);  // 13419 
  auto esis = W.template arr<Real>(1, ngs);  // 13419 
  auto egli = W.template arr<Real>(1, ngs);  // 13419 
  auto eghi = W.template arr<Real>(1, ngs);  // 13419 
  auto efi = W.template arr<Real>(1, ngs);  // 13419 
  auto efis = W.template arr<Real>(1, ngs);  // 13419 
  auto ehxi = W.template arr<Real>(1, ngs);  // 13420 
  auto ehli = W.template arr<Real>(1, ngs);  // 13420 
  auto egmi = W.template arr<Real>(1, ngs);  // 13420 
  auto ehi = W.template arr<Real>(1, ngs);  // 13420 
  auto ehis = W.template arr<Real>(1, ngs);  // 13420 
  auto ehlis = W.template arr<Real>(1, ngs);  // 13420 
  auto ers = W.template arr<Real>(1, ngs);  // 13421 
  auto ess = W.template arr<Real>(1, ngs);  // 13421 
  auto egls = W.template arr<Real>(1, ngs);  // 13421 
  auto eghs = W.template arr<Real>(1, ngs);  // 13421 
  auto efs = W.template arr<Real>(1, ngs);  // 13421 
  auto ehs = W.template arr<Real>(1, ngs);  // 13421 
  auto ehsfac = W.template arr<Real>(1, ngs);  // 13421 
  auto ehscnv = W.template arr<Real>(1, ngs);  // 13422 
  auto ehxs = W.template arr<Real>(1, ngs);  // 13423 
  auto ehls = W.template arr<Real>(1, ngs);  // 13423 
  auto egms = W.template arr<Real>(1, ngs);  // 13423 
  auto egmip = W.template arr<Real>(1, ngs);  // 13423 
  auto ehsclsn = W.template arr<Real>(1, ngs);  // 13425 
  auto ehiclsn = W.template arr<Real>(1, ngs);  // 13425 
  auto ehisclsn = W.template arr<Real>(1, ngs);  // 13425 
  auto efsclsn = W.template arr<Real>(1, ngs);  // 13426 
  auto eficlsn = W.template arr<Real>(1, ngs);  // 13426 
  auto efisclsn = W.template arr<Real>(1, ngs);  // 13426 
  auto ehlsclsn = W.template arr<Real>(1, ngs);  // 13427 
  auto ehliclsn = W.template arr<Real>(1, ngs);  // 13427 
  auto ehlisclsn = W.template arr<Real>(1, ngs);  // 13427 
  auto esiclsn = W.template arr<Real>(1, ngs);  // 13428 
  auto esisclsn = W.template arr<Real>(1, ngs);  // 13428 
  Real ehs_collsn = NR(0.5);  // 13430
  Real ehi_collsn = NR(1.0);  // 13430
  Real efs_collsn = NR(0.5);  // 13431
  Real efi_collsn = NR(1.0);  // 13431
  Real ehls_collsn = NR(1.0);  // 13432
  Real ehli_collsn = NR(1.0);  // 13432
  Real esi_collsn = NR(1.0);  // 13433
  auto ew = W.template arr<Real>(1, 8, 1, 6);  // 13435 
  auto cwr = W.template arr<Real>(1, 8, 1, 2);  // 13436 
  auto icwr = W.template arr<int>(1, ngs);  // 13439 
  auto igwr = W.template arr<int>(1, ngs);  // 13439 
  auto irwr = W.template arr<int>(1, ngs);  // 13439 
  auto ihlr = W.template arr<int>(1, ngs);  // 13439 
  auto ifwr = W.template arr<int>(1, ngs);  // 13439 
  auto grad = W.template arr<Real>(1, 6, 1, 2);  // 13440 
  auto da0lr = W.template arr<Real>(1, ngs);  // 13454 
  auto da1lr = W.template arr<Real>(1, ngs);  // 13454 
  auto da0lc = W.template arr<Real>(1, ngs);  // 13455 
  auto da1lc = W.template arr<Real>(1, ngs);  // 13455 
  auto da0lh = W.template arr<Real>(1, ngs);  // 13456 
  auto da0lhl = W.template arr<Real>(1, ngs);  // 13457 
  auto da0lf = W.template arr<Real>(1, ngs);  // 13458 
  auto da0lx = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 13459 
  auto va0 = W.template arr<Real>(P.lc, P.lqmx);  // 13461 
  auto vab0 = W.template arr<Real>(P.lc, P.lqmx, P.lc, P.lqmx);  // 13462 
  auto vab1 = W.template arr<Real>(P.lc, P.lqmx, P.lc, P.lqmx);  // 13463 
  auto va1 = W.template arr<Real>(P.lc, P.lqmx);  // 13464 
  auto ehip = W.template arr<Real>(1, ngs);  // 13465 
  auto ehlip = W.template arr<Real>(1, ngs);  // 13465 
  auto ehlir = W.template arr<Real>(1, ngs);  // 13465 
  auto erir = W.template arr<Real>(1, ngs);  // 13466 
  auto esir = W.template arr<Real>(1, ngs);  // 13466 
  auto eglir = W.template arr<Real>(1, ngs);  // 13466 
  auto egmir = W.template arr<Real>(1, ngs);  // 13466 
  auto eghir = W.template arr<Real>(1, ngs);  // 13466 
  auto efir = W.template arr<Real>(1, ngs);  // 13467 
  auto ehir = W.template arr<Real>(1, ngs);  // 13467 
  auto eirw = W.template arr<Real>(1, ngs);  // 13467 
  auto eirir = W.template arr<Real>(1, ngs);  // 13467 
  auto ehr = W.template arr<Real>(1, ngs);  // 13467 
  auto erip = W.template arr<Real>(1, ngs);  // 13468 
  auto esip = W.template arr<Real>(1, ngs);  // 13468 
  auto eglip = W.template arr<Real>(1, ngs);  // 13468 
  auto eghip = W.template arr<Real>(1, ngs);  // 13468 
  auto efip = W.template arr<Real>(1, ngs);  // 13469 
  auto eipi = W.template arr<Real>(1, ngs);  // 13469 
  auto eipw = W.template arr<Real>(1, ngs);  // 13469 
  auto eipip = W.template arr<Real>(1, ngs);  // 13469 
  auto ptotal = W.template arr<Real>(1, ngs);  // 13473 
  auto pqcwi = W.template arr<Real>(1, ngs);  // 13475 
  auto pqcii = W.template arr<Real>(1, ngs);  // 13475 
  auto pqrwi = W.template arr<Real>(1, ngs);  // 13475 
  auto pqisi = W.template arr<Real>(1, ngs);  // 13475 
  auto pqswi = W.template arr<Real>(1, ngs);  // 13476 
  auto pqhwi = W.template arr<Real>(1, ngs);  // 13476 
  auto pqwvi = W.template arr<Real>(1, ngs);  // 13476 
  auto pqgli = W.template arr<Real>(1, ngs);  // 13477 
  auto pqghi = W.template arr<Real>(1, ngs);  // 13477 
  auto pqfwi = W.template arr<Real>(1, ngs);  // 13477 
  auto pqgmi = W.template arr<Real>(1, ngs);  // 13478 
  auto pqhli = W.template arr<Real>(1, ngs);  // 13478 
  auto pqiri = W.template arr<Real>(1, ngs);  // 13479 
  auto pqipi = W.template arr<Real>(1, ngs);  // 13479 
  auto pqlwsi = W.template arr<Real>(1, ngs);  // 13480 
  auto pqlwhi = W.template arr<Real>(1, ngs);  // 13480 
  auto pqlwhli = W.template arr<Real>(1, ngs);  // 13480 
  auto pqlwfi = W.template arr<Real>(1, ngs);  // 13480 
  auto pqlwlghi = W.template arr<Real>(1, ngs);  // 13482 
  auto pqlwlghli = W.template arr<Real>(1, ngs);  // 13482 
  auto pqlwlghd = W.template arr<Real>(1, ngs);  // 13483 
  auto pqlwlghld = W.template arr<Real>(1, ngs);  // 13483 
  auto pvhwi = W.template arr<Real>(1, ngs);  // 13488 
  auto pvhwd = W.template arr<Real>(1, ngs);  // 13488 
  auto pvfwi = W.template arr<Real>(1, ngs);  // 13489 
  auto pvfwd = W.template arr<Real>(1, ngs);  // 13489 
  auto pvhli = W.template arr<Real>(1, ngs);  // 13490 
  auto pvhld = W.template arr<Real>(1, ngs);  // 13490 
  auto pvswi = W.template arr<Real>(1, ngs);  // 13491 
  auto pvswd = W.template arr<Real>(1, ngs);  // 13491 
  auto pqcwd = W.template arr<Real>(1, ngs);  // 13493 
  auto pqcid = W.template arr<Real>(1, ngs);  // 13493 
  auto pqrwd = W.template arr<Real>(1, ngs);  // 13493 
  auto pqisd = W.template arr<Real>(1, ngs);  // 13493 
  auto pqcwdacc = W.template arr<Real>(1, ngs);  // 13493 
  auto pqswd = W.template arr<Real>(1, ngs);  // 13494 
  auto pqhwd = W.template arr<Real>(1, ngs);  // 13494 
  auto pqwvd = W.template arr<Real>(1, ngs);  // 13494 
  auto pqgld = W.template arr<Real>(1, ngs);  // 13495 
  auto pqghd = W.template arr<Real>(1, ngs);  // 13495 
  auto pqfwd = W.template arr<Real>(1, ngs);  // 13495 
  auto pqgmd = W.template arr<Real>(1, ngs);  // 13496 
  auto pqhld = W.template arr<Real>(1, ngs);  // 13496 
  auto pqird = W.template arr<Real>(1, ngs);  // 13497 
  auto pqipd = W.template arr<Real>(1, ngs);  // 13497 
  auto pqlwsd = W.template arr<Real>(1, ngs);  // 13498 
  auto pqlwhd = W.template arr<Real>(1, ngs);  // 13498 
  auto pqlwhld = W.template arr<Real>(1, ngs);  // 13498 
  auto pqlwfd = W.template arr<Real>(1, ngs);  // 13498 
  auto pctot = W.template arr<Real>(1, ngs);  // 13502 
  auto pcipi = W.template arr<Real>(1, ngs);  // 13503 
  auto pcipd = W.template arr<Real>(1, ngs);  // 13503 
  auto pciri = W.template arr<Real>(1, ngs);  // 13504 
  auto pcird = W.template arr<Real>(1, ngs);  // 13504 
  auto pccwi = W.template arr<Real>(1, ngs);  // 13505 
  auto pccwd = W.template arr<Real>(1, ngs);  // 13505 
  auto pccwdacc = W.template arr<Real>(1, ngs);  // 13505 
  auto pccii = W.template arr<Real>(1, ngs);  // 13506 
  auto pccid = W.template arr<Real>(1, ngs);  // 13506 
  auto pcisi = W.template arr<Real>(1, ngs);  // 13507 
  auto pcisd = W.template arr<Real>(1, ngs);  // 13507 
  auto pccin = W.template arr<Real>(1, ngs);  // 13508 
  auto pcrwi = W.template arr<Real>(1, ngs);  // 13509 
  auto pcrwd = W.template arr<Real>(1, ngs);  // 13509 
  auto pcswi = W.template arr<Real>(1, ngs);  // 13510 
  auto pcswd = W.template arr<Real>(1, ngs);  // 13510 
  auto pchwi = W.template arr<Real>(1, ngs);  // 13511 
  auto pchwd = W.template arr<Real>(1, ngs);  // 13511 
  auto pchli = W.template arr<Real>(1, ngs);  // 13512 
  auto pchld = W.template arr<Real>(1, ngs);  // 13512 
  auto pcfwi = W.template arr<Real>(1, ngs);  // 13513 
  auto pcfwd = W.template arr<Real>(1, ngs);  // 13513 
  auto pcgli = W.template arr<Real>(1, ngs);  // 13514 
  auto pcgld = W.template arr<Real>(1, ngs);  // 13514 
  auto pcgmi = W.template arr<Real>(1, ngs);  // 13515 
  auto pcgmd = W.template arr<Real>(1, ngs);  // 13515 
  auto pcghi = W.template arr<Real>(1, ngs);  // 13516 
  auto pcghd = W.template arr<Real>(1, ngs);  // 13516 
  auto pzrwi = W.template arr<Real>(1, ngs);  // 13518 
  auto pzrwd = W.template arr<Real>(1, ngs);  // 13518 
  auto pzhwi = W.template arr<Real>(1, ngs);  // 13519 
  auto pzhwd = W.template arr<Real>(1, ngs);  // 13519 
  auto pzfwi = W.template arr<Real>(1, ngs);  // 13520 
  auto pzfwd = W.template arr<Real>(1, ngs);  // 13520 
  auto pzhli = W.template arr<Real>(1, ngs);  // 13521 
  auto pzhld = W.template arr<Real>(1, ngs);  // 13521 
  auto pzswi = W.template arr<Real>(1, ngs);  // 13522 
  auto pzswd = W.template arr<Real>(1, ngs);  // 13522 
  auto dqisdt = W.template arr<Real>(1, ngs);  // 13527 
  auto qss0 = W.template arr<Real>(1, ngs);  // 13529 
  auto qsacip = W.template arr<Real>(1, ngs);  // 13531 
  auto pres = W.template arr<Real>(1, ngs);  // 13532 
  auto pipert = W.template arr<Real>(1, ngs);  // 13532 
  auto pk = W.template arr<Real>(1, ngs);  // 13533 
  auto rho0 = W.template arr<Real>(1, ngs);  // 13534 
  auto pi0 = W.template arr<Real>(1, ngs);  // 13534 
  auto rhovt = W.template arr<Real>(1, ngs);  // 13535 
  Real sqrtrhovt = 0;  // 13535
  auto thetap = W.template arr<Real>(1, ngs);  // 13536 
  auto theta0 = W.template arr<Real>(1, ngs);  // 13536 
  auto qwvp = W.template arr<Real>(1, ngs);  // 13536 
  auto qv0 = W.template arr<Real>(1, ngs);  // 13536 
  auto thsave = W.template arr<Real>(1, ngs);  // 13537 
  auto ptwfzi = W.template arr<Real>(1, ngs);  // 13538 
  auto ptimlw = W.template arr<Real>(1, ngs);  // 13538 
  auto psub = W.template arr<Real>(1, ngs);  // 13539 
  auto pvap = W.template arr<Real>(1, ngs);  // 13539 
  auto pfrz = W.template arr<Real>(1, ngs);  // 13539 
  auto ptem = W.template arr<Real>(1, ngs);  // 13539 
  auto pmlt = W.template arr<Real>(1, ngs);  // 13539 
  auto pevap = W.template arr<Real>(1, ngs);  // 13539 
  auto pdep = W.template arr<Real>(1, ngs);  // 13539 
  auto ptem2 = W.template arr<Real>(1, ngs);  // 13539 
  auto cnostmp = W.template arr<Real>(1, ngs);  // 13541 
  Real cqtotn = 0;  // 13555
  Real cqtotn1 = 0;  // 13555
  Real cctotn = 0;  // 13556
  Real citotn = 0;  // 13557
  Real crtotn = 0;  // 13558
  Real cstotn = 0;  // 13559
  Real cvtotn = 0;  // 13560
  Real cftotn = 0;  // 13561
  Real cgltotn = 0;  // 13562
  Real cghtotn = 0;  // 13563
  Real chtotn = 0;  // 13564
  Real cqtotp = 0;  // 13565
  Real cqtotp1 = 0;  // 13565
  Real cctotp = 0;  // 13566
  Real citotp = 0;  // 13567
  Real ciptotp = 0;  // 13568
  Real crtotp = 0;  // 13569
  Real cstotp = 0;  // 13570
  Real cvtotp = 0;  // 13571
  Real cftotp = 0;  // 13572
  Real chltotp = 0;  // 13573
  Real cgltotp = 0;  // 13574
  Real cgmtotp = 0;  // 13575
  Real cghtotp = 0;  // 13576
  Real chtotp = 0;  // 13577
  Real cqfac = 0;  // 13578
  Real ccfac = 0;  // 13579
  Real cifac = 0;  // 13580
  Real cipfac = 0;  // 13581
  Real crfac = 0;  // 13582
  Real csfac = 0;  // 13583
  Real cvfac = 0;  // 13584
  Real cffac = 0;  // 13585
  Real cglfac = 0;  // 13586
  Real cghfac = 0;  // 13587
  Real chfac = 0;  // 13588
  Real ssifac = 0;  // 13590
  Real qvapor = 0;  // 13590
  int ireadqf = 0;  // 13596
  int lrho = 0;  // 13596
  int lqsw = 0;  // 13596
  int lqgl = 0;  // 13596
  int lqgm = 0;  // 13596
  int lqgh = 0;  // 13596
  int lqrw = 0;  // 13597
  Real vt = 0;  // 13598
  Real arg = 0;  // 13599
  Real erbnd1 = 0;  // 13600
  Real fdgt1 = 0;  // 13600
  Real costhe1 = 0;  // 13600
  Real qeps = 0;  // 13601
  Real dyi2 = 0;  // 13602
  Real dzi2 = 0;  // 13602
  Real bta1 = 0;  // 13602
  Real cnit = 0;  // 13602
  Real dragh = 0;  // 13602
  Real dnz00 = 0;  // 13602
  Real pii = 0;  // 13602
  Real qccrit = 0;  // 13603
  Real gf4br = 0;  // 13603
  Real gf4ds = 0;  // 13603
  Real gf4p5 = 0;  // 13603
  Real gf3ds = 0;  // 13603
  Real gf1ds = 0;  // 13603
  auto gf1palp = W.template arr<Real>(1, ngs);  // 13604 
  Real xdn_new = 0;  // 13608
  Real drhodt = 0;  // 13608
  int l = 0;  // 13610
  int ltemq = 0;  // 13610
  int inumgs = 0;  // 13610
  int idelq = 0;  // 13610
  Real brz = 0;  // 13612
  Real arz = 0;  // 13612
  Real temq = 0;  // 13612
  Real ssival = 0;  // 13614
  Real tqvcon = 0;  // 13614
  Real cnox = 0;  // 13616
  Real cval = 0;  // 13617
  Real aval = 0;  // 13617
  Real eval = 0;  // 13617
  Real fval = 0;  // 13617
  Real gval = 0;  // 13617
  Real qsign = 0;  // 13617
  Real ftelwc = 0;  // 13617
  Real qconkq = 0;  // 13617
  Real elecfac = 0;  // 13617
  Real altelecfac = 0;  // 13617
  Real qconm = 0;  // 13618
  Real qconn = 0;  // 13618
  Real cfce15 = 0;  // 13618
  Real gf8 = 0;  // 13618
  Real gf4i = 0;  // 13618
  Real gf3p5 = 0;  // 13618
  Real gf1a = 0;  // 13618
  Real gf1p5 = 0;  // 13618
  Real qdiff = 0;  // 13618
  Real argrcnw = 0;  // 13618
  Real c4 = 0;  // 13619
  Real bradp = 0;  // 13619
  Real bl2 = 0;  // 13619
  Real bt2 = 0;  // 13619
  Real dthr = 0;  // 13619
  Real hrifac = 0;  // 13619
  Real hdia0 = 0;  // 13619
  Real hdia1 = 0;  // 13619
  Real civenta = 0;  // 13619
  Real civentb = 0;  // 13619
  Real civentc = 0;  // 13620
  Real civentd = 0;  // 13620
  Real civente = 0;  // 13620
  Real civentf = 0;  // 13620
  Real civentg = 0;  // 13620
  Real cireyn = 0;  // 13620
  Real xcivent = 0;  // 13620
  Real cipventa = 0;  // 13621
  Real cipventb = 0;  // 13621
  Real cipventc = 0;  // 13621
  Real cipventd = 0;  // 13621
  Real cipreyn = 0;  // 13621
  Real cirventa = 0;  // 13621
  Real cirventb = 0;  // 13622
  int igmrwa = 0;  // 13623
  int igmrwb = 0;  // 13623
  int igmswa = 0;  // 13623
  int igmswb = 0;  // 13623
  int igmfwa = 0;  // 13623
  int igmfwb = 0;  // 13623
  int igmhwa = 0;  // 13623
  int igmhwb = 0;  // 13623
  Real rwventa = 0;  // 13624
  Real rwventb = 0;  // 13624
  Real swventa = 0;  // 13624
  Real swventb = 0;  // 13624
  Real fwventa = 0;  // 13624
  Real fwventb = 0;  // 13624
  Real fwventc = 0;  // 13624
  Real hwventa = 0;  // 13625
  Real hwventb = 0;  // 13625
  Real hwventc = 0;  // 13626
  Real hlventa = 0;  // 13626
  Real hlventb = 0;  // 13626
  Real hlventc = 0;  // 13626
  Real glventa = 0;  // 13627
  Real glventb = 0;  // 13627
  Real glventc = 0;  // 13627
  Real gmventa = 0;  // 13628
  Real gmventb = 0;  // 13628
  Real gmventc = 0;  // 13628
  Real ghventa = 0;  // 13628
  Real ghventb = 0;  // 13628
  Real ghventc = 0;  // 13628
  Real dzfacp = 0;  // 13629
  Real dzfacm = 0;  // 13629
  Real cmassin = 0;  // 13629
  Real cwdiar = 0;  // 13629
  Real rimmas = 0;  // 13630
  Real rhobar = 0;  // 13630
  Real argtim = 0;  // 13631
  Real argqcw = 0;  // 13631
  Real argqxw = 0;  // 13631
  Real argtem = 0;  // 13631
  Real frcswsw = 0;  // 13632
  Real frcswgl = 0;  // 13632
  Real frcswgm = 0;  // 13632
  Real frcswgh = 0;  // 13632
  Real frcswfw = 0;  // 13632
  Real frcswsw1 = 0;  // 13632
  Real frcglgl = 0;  // 13633
  Real frcglgm = 0;  // 13633
  Real frcglgh = 0;  // 13633
  Real frcglfw = 0;  // 13633
  Real frcglgl1 = 0;  // 13633
  Real frcgmgl = 0;  // 13634
  Real frcgmgm = 0;  // 13634
  Real frcgmgh = 0;  // 13634
  Real frcgmfw = 0;  // 13634
  Real frcgmgm1 = 0;  // 13634
  Real frcghgl = 0;  // 13635
  Real frcghgm = 0;  // 13635
  Real frcghgh = 0;  // 13635
  Real frcghfw = 0;  // 13635
  Real frcghgh1 = 0;  // 13635
  Real frcfwgl = 0;  // 13636
  Real frcfwgm = 0;  // 13636
  Real frcfwgh = 0;  // 13636
  Real frcfwfw = 0;  // 13636
  Real frcfwfw1 = 0;  // 13636
  Real frcswrsw = 0;  // 13637
  Real frcswrgl = 0;  // 13637
  Real frcswrgm = 0;  // 13637
  Real frcswrgh = 0;  // 13637
  Real frcswrfw = 0;  // 13637
  Real frcswrsw1 = 0;  // 13638
  Real frcrswsw = 0;  // 13639
  Real frcrswgl = 0;  // 13639
  Real frcrswgm = 0;  // 13639
  Real frcrswgh = 0;  // 13639
  Real frcrswfw = 0;  // 13639
  Real frcrswsw1 = 0;  // 13640
  Real frcglrgl = 0;  // 13641
  Real frcglrgm = 0;  // 13641
  Real frcglrgh = 0;  // 13641
  Real frcglrfw = 0;  // 13641
  Real frcglrgl1 = 0;  // 13641
  Real frcrglgl = 0;  // 13642
  Real frcrglgm = 0;  // 13643
  Real frcrglgh = 0;  // 13643
  Real frcrglfw = 0;  // 13643
  Real frcrglgl1 = 0;  // 13643
  Real frcgmrgl = 0;  // 13644
  Real frcgmrgm = 0;  // 13644
  Real frcgmrgh = 0;  // 13644
  Real frcgmrfw = 0;  // 13644
  Real frcgmrgm1 = 0;  // 13644
  Real frcrgmgl = 0;  // 13645
  Real frcrgmgm = 0;  // 13645
  Real frcrgmgh = 0;  // 13645
  Real frcrgmfw = 0;  // 13645
  Real frcrgmgm1 = 0;  // 13645
  Real total = 0;  // 13646
  Real qweps = 0;  // 13646
  Real gf2a = 0;  // 13646
  Real gf4a = 0;  // 13646
  Real dqldt = 0;  // 13646
  Real dqidt = 0;  // 13646
  Real dqdt = 0;  // 13646
  Real frcghrgl = 0;  // 13647
  Real frcghrgm = 0;  // 13647
  Real frcghrgh = 0;  // 13647
  Real frcghrfw = 0;  // 13647
  Real frcghrgh1 = 0;  // 13647
  Real frcrghgl = 0;  // 13647
  Real frcrghgm = 0;  // 13648
  Real frcrghgh = 0;  // 13648
  Real frcrghfw = 0;  // 13648
  Real frcrghgh1 = 0;  // 13648
  Real a1 = 0;  // 13649
  Real a2 = 0;  // 13649
  Real a3 = 0;  // 13649
  Real a4 = 0;  // 13649
  Real a5 = 0;  // 13649
  Real a6 = 0;  // 13649
  Real gamss = 0;  // 13650
  Real cdw = 0;  // 13651
  Real cdi = 0;  // 13651
  Real denom1 = 0;  // 13651
  Real denom2 = 0;  // 13651
  Real delqci1 = 0;  // 13651
  Real delqip1 = 0;  // 13651
  Real cirtotn = 0;  // 13652
  Real ciptotn = 0;  // 13652
  Real cgmtotn = 0;  // 13652
  Real chltotn = 0;  // 13652
  Real cirtotp = 0;  // 13652
  Real cgmfac = 0;  // 13653
  Real chlfac = 0;  // 13653
  Real cirfac = 0;  // 13653
  int igmhla = 0;  // 13654
  int igmhlb = 0;  // 13654
  int igmgla = 0;  // 13654
  int igmglb = 0;  // 13654
  int igmgma = 0;  // 13654
  int igmgmb = 0;  // 13654
  int igmgha = 0;  // 13655
  int igmghb = 0;  // 13655
  int idqis = 0;  // 13656
  int item = 0;  // 13656
  int itim0 = 0;  // 13656
  int iqgl = 0;  // 13657
  int iqgm = 0;  // 13657
  int iqgh = 0;  // 13657
  int iqrw = 0;  // 13657
  int iqsw = 0;  // 13657
  int itertd = 0;  // 13658
  int ia = 0;  // 13658
  int infdo = 0;  // 13660
  Real tau = 0;  // 13662
  Real ewtmp = 0;  // 13662
  int cntnic_noliq = 0;  // 13664
  Real q_noliqmn = 0;  // 13665
  Real q_noliqmx = 0;  // 13665
  Real scsacimn = 0;  // 13666
  Real scsacimx = 0;  // 13666
  Real dtpinv = 0;  // 13668
  Real xden = 0;  // 13672
  Real xmlt = 0;  // 13672
  Real cmlt = 0;  // 13672
  Real cmlttot = 0;  // 13672
  Real fventm = 0;  // 13672
  Real fventh = 0;  // 13672
  Real am = 0;  // 13672
  Real ah = 0;  // 13672
  Real felfinv = 0;  // 13672
  Real dmwdt = 0;  // 13672
  Real qhmlrtmp = 0;  // 13674
  Real qhmlrtmp2 = 0;  // 13674
  Real chmlrtmp = 0;  // 13674
  Real chmlrtmpd1inf = 0;  // 13674
  Real chlmlrtmp = 0;  // 13674
  Real zhlmlrtmp = 0;  // 13674
  Real zhlmlrrtmp = 0;  // 13674
  Real qvs0 = 0;  // 13674
  Real tmpcmlt = 0;  // 13674
  Real term1 = 0;  // 13676
  Real term2 = 0;  // 13676
  Real term3 = 0;  // 13676
  Real term4 = 0;  // 13676
  Real qaacw = 0;  // 13677
  Real cwchtmp = 0;  // 13678
  Real a_in = 0;  // 13687
  // data cwr / 2.0, 3.0, 4.0, 6.0,  8.0,  10.0, 15.0,  20.0 , 1.0, 1.0, 0.5, 0.5,  0.5,   0.2,  0.2,  1.  /
  // data grad / 100., 200., 300., 400., 600., 1000., 1.e-2,1.e-2,1.e-2,5.e-3,2.5e-3, 1.    /
  // data ew /0.03, 0.07, 0.17, 0.41, 0.58, 0.69, 0.82, 0.88, 0.10, 0.20, 0.34, 0.58, 0.70, 0.78, 0.88, 0.92, 0.15, 0.31, 0.44, 0.65, 0.75, 0.83, 0.96, 0.91, 0.17, 0.37, 0.50, 0.70, 0.81, 0.87, 0.93, 0.96, 0.17, 0.40, 0.54, 0.71, 0.83, 0.88, 0.94, 0.98, 0.15, 0.37, 0.52, 0.74, 0.82, 0.88, 0.94, 0.98 /
  auto galpha = [&](Real a_in) -> Real { return ((((((NR(4.0) + a_in)) * ((NR(5.0) + a_in))) * ((NR(6.0) + a_in)))) / (((((NR(1.0) + a_in)) * ((NR(2.0) + a_in))) * ((NR(3.0) + a_in))))); };  // 13691 statement function
  auto dgalpha = [&](Real a_in) -> Real { return ((((((NR(876.0) + (NR(1260.0) * a_in)) + (NR(621.0) * ipow<2>(a_in))) + (NR(126.0) * ipow<3>(a_in))) + (NR(9.0) * ipow<4>(a_in)))) / (((((((NR(36.0) + (NR(132.0) * a_in)) + (NR(193.0) * ipow<2>(a_in))) + (NR(144.0) * ipow<3>(a_in))) + (NR(58.0) * ipow<4>(a_in))) + (NR(12.0) * ipow<5>(a_in))) + ipow<6>(a_in)))); };  // 13692 statement function
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pb(i_0) = NR(0.0);  // 13705
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pinit(i_0) = NR(0.0);  // 13706
  itile = nx;  // 13707
  jtile = ny;  // 13708
  ktile = nz;  // 13709
  ixend = nx;  // 13710
  jyend = ny;  // 13711
  kzend = nz;  // 13712
  nxend = (nx + 1);  // 13713
  nyend = (ny + 1);  // 13714
  nzend = nz;  // 13715
  kzbeg = 1;  // 13716
  nzbeg = 1;  // 13717
  istag = 0;  // 13719
  jstag = 0;  // 13720
  kstag = 1;  // 13721
  for (int i_0 = P.lc; i_0 <= P.lhab; ++i_0) lrescalelow(i_0) = P.rescale_low_alpha;  // 13723
  lrescalelow(P.lr) = (P.rescale_low_alphar && P.rescale_low_alpha);  // 13724
  lrescalelow(P.lh) = (P.rescale_low_alphah && P.rescale_low_alpha);  // 13725
  if ((P.lhl > 1)) { lrescalelow(P.lhl) = (P.rescale_low_alphahl && P.rescale_low_alpha); }  // 13727
  if ((ngs < nz)) {  // 13734
  }  // 13737
  cntnic_noliq = 0;  // 13739
  q_noliqmn = NR(0.0);  // 13740
  q_noliqmx = NR(0.0);  // 13741
  scsacimn = NR(0.0);  // 13742
  scsacimx = NR(0.0);  // 13743
  ldovol = false;  // 13745
  for (il = P.lc; il <= P.lhab; ++il) {  // 13747
    ldovol = (ldovol || ((P.lvol(il) > 1)));  // 13748
  }  // 13749
  ffrzh = 1;  // 13752
  dtpinv = (1.0e0 / dtp);  // 13766
  qeps = NR(1.0e-20);  // 13775
  aradcw = (-NR(0.27544));  // 13785
  bradcw = NR(0.26249e+06);  // 13786
  cradcw = (-NR(1.8896e+10));  // 13787
  dradcw = NR(4.4626e+14);  // 13788
  bta1 = NR(0.6);  // 13789
  cnit = NR(1.0e-02);  // 13790
  dragh = NR(0.60);  // 13791
  dnz00 = NR(1.225);  // 13792
  cs = NR(12.42);  // 13796
  ds = NR(0.42);  // 13797
  pii = P.piinv;  // 13798
  pid4 = (P.pi / NR(4.0));  // 13799
  gf1 = NR(1.0);  // 13801
  gf1p5 = NR(0.8862269255);  // 13802
  gf2 = NR(1.0);  // 13803
  gf3 = NR(2.0);  // 13804
  gf3p5 = NR(3.32335097);  // 13805
  gf4 = NR(6.00);  // 13806
  gf5 = NR(24.0);  // 13807
  gf6 = NR(120.0);  // 13808
  gf7 = NR(720.0);  // 13809
  gf4br = NR(17.837861981813607);  // 13810
  gf4ds = NR(10.41688578110938);  // 13811
  gf4p5 = NR(11.63172839656745);  // 13812
  gf3ds = NR(3.0458730354120997);  // 13813
  gf1ds = NR(0.8863557896089221);  // 13814
  gf43rds = NR(0.8929795116);  // 13816
  gf53rds = NR(0.9027452930);  // 13817
  gf73rds = NR(1.190639349);  // 13818
  gf83rds = NR(1.504575488);  // 13819
  gamice73fac = (ipow<3>((gamma_sp(((NR(7.0) / NR(3.0)) + P.cinu)))) / ((ipow<3>(gamma_sp((NR(1.0) + P.cinu))) * ipow<4>(((NR(1.0) + P.cinu))))));  // 13821
  gamsnow73fac = (ipow<3>((gamma_sp(((NR(7.0) / NR(3.0)) + P.snu)))) / ((ipow<3>(gamma_sp((NR(1.0) + P.snu))) * ipow<4>(((NR(1.0) + P.snu))))));  // 13822
  brz = NR(100.0);  // 13832
  arz = NR(0.66);  // 13833
  bfnu1 = (((((NR(4.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(6.0) + P.alphar))) / (((((NR(1.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(3.0) + P.alphar)))));  // 13835
  galpharaut = (((((NR(6.0) + P.alpharaut)) * ((NR(5.0) + P.alpharaut))) * ((NR(4.0) + P.alpharaut))) / (((((NR(3.0) + P.alpharaut)) * ((NR(2.0) + P.alpharaut))) * ((NR(1.0) + P.alpharaut)))));  // 13838
  vfrz = (NR(0.523599) * ipow<3>((P.dfrz)));  // 13841
  vmlt = c_min(P.xvmx(P.lr), (NR(0.523599) * ipow<3>((P.dmlt))));  // 13842
  vshd = c_min(P.xvmx(P.lr), (NR(0.523599) * ipow<3>((P.dshd))));  // 13843
  tdtol = NR(1.0e-05);  // 13849
  tfrcbw = (P.tfr - P.cbw);  // 13850
  tfrcbi = (P.tfr - P.cbi);  // 13851
  mltmass0inv = (NR(1.0) / ((NR(1000.0) * P.xvmx(P.lr))));  // 13872
  mltmass1inv = (NR(1.0) / (((NR(1000.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>((((NR(0.01) * NR(0.5)) * P.takshedsize1)))))));  // 13873
  mltmass2inv = (NR(1.0) / (((NR(1000.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>((((NR(0.01) * NR(0.5)) * P.takshedsize2)))))));  // 13874
  mltmass3inv = (NR(1.0) / (((NR(1000.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>((((NR(0.01) * NR(0.5)) * P.takshedsize3)))))));  // 13875
  mltmass1cgs = ((NR(1.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>(((NR(0.5) * P.takshedsize1)))));  // 13876
  mltmass2cgs = ((NR(1.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>(((NR(0.5) * P.takshedsize2)))));  // 13877
  mltmass3cgs = ((NR(1.0) * (((NR(4.0) * P.pi) / NR(3.0)))) * (ipow<3>(((NR(0.5) * P.takshedsize3)))));  // 13878
  numdiam = 2;  // 13886
  mltdiam(1) = (P.mltdiam1 / NR(6.0));  // 13887
  mltdiam(2) = (P.mltdiam1 / NR(2.0));  // 13888
  mltdiam((ndiam + 1)) = P.mltdiam1;  // 13910
  mltdiam((ndiam + 2)) = P.mltdiam2;  // 13911
  mltdiam((ndiam + 3)) = P.mltdiam3;  // 13912
  mltdiam((ndiam + 4)) = P.mltdiam4;  // 13913
  kzb = 1;  // 13916
  kze = ktile;  // 13917
  mwfac = pow(NR(6.0), ((NR(1.0) / NR(3.0))));  // 13924
  if ((P.ipconc >= 2)) {  // 13925
  }  // 13929
  rwmasn = (P.xvmn(P.lr) * NR(1000.0));  // 13930
  rwmasx = (P.xvmx(P.lr) * NR(1000.0));  // 13931
  xvbiggsnow = P.xvmn(P.lh);  // 13936
  cimasn = c_min(P.cimas0, P.cimas1);  // 13942
  cimasx = NR(1.0e-8);  // 13943
  ccimx = NR(5000.0e3);  // 13944
  iend = 0;  // 13953
  jy = jgs;  // 13969
  if ((P.ipconc < 2)) {  // 13980
    for (kz = 1; kz <= kze; ++kz) {  // 13981
      for (ix = 1; ix <= itile; ++ix) {  // 13982
        t9(ix, jy, kz) = an(ix, jy, kz, P.lc);  // 13983
      }  // 13984
    }  // 13985
  }  // 13986
  nxmpb = 1;  // 13995
  nzmpb = 1;  // 13996
  nxz = (itile * nz);  // 13997
  numgs = ((nxz / ngs) + 1);  // 13998
  for (inumgs = 1; inumgs <= numgs; ++inumgs) {  // 14001
    ngscnt = 0;  // 14002
    for (kz = nzmpb; kz <= kze; ++kz) {  // 14004
      for (ix = nxmpb; ix <= itile; ++ix) {  // 14005
        pqs(1) = t00(ix, jy, kz);  // 14007
        pres(1) = (pn(ix, jy, kz) + pb(kz));  // 14008
        theta(1) = an(ix, jy, kz, P.lt);  // 14010
        temg(1) = t0(ix, jy, kz);  // 14011
        temcg(1) = (temg(1) - P.tfr);  // 14012
        tqvcon = (temg(1) - P.cbw);  // 14013
        ltemq = ((((temg(1) - NR(163.15))) / P.fqsat) + NR(1.5));  // 14014
        ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 14015
        qvs(1) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(1) - (P.esbolton * P.tabqvs(ltemq)))));  // 14019
        if (((P.iqis0 == 1) || (temg(1) <= (P.tfr + NR(0.5))))) {  // 14022
          qis(1) = (pqs(1) * P.tabqis(ltemq));  // 14023
        } else {  // 14024
          ltemq = ((((P.tfr - NR(163.15))) / P.fqsat) + NR(1.5));  // 14025
          qis(1) = (pqs(1) * P.tabqis(ltemq));  // 14026
        }  // 14027
        qss(1) = qvs(1);  // 14029
        if ((temg(1) < P.tfr)) {  // 14031
          qss(1) = qis(1);  // 14032
        }  // 14033
        ishail = false;  // 14035
        if ((P.lhl > 1)) {  // 14036
          if ((an(ix, jy, kz, P.lhl) > P.qxmin(P.lhl))) { ishail = true; }  // 14037
        }  // 14038
        if ((((((((an(ix, jy, kz, P.lv) > qss(1)) || (an(ix, jy, kz, P.lc) > P.qxmin(P.lc))) || (an(ix, jy, kz, P.li) > P.qxmin(P.li))) || (an(ix, jy, kz, P.lr) > P.qxmin(P.lr))) || (an(ix, jy, kz, P.ls) > P.qxmin(P.ls))) || (an(ix, jy, kz, P.lh) > P.qxmin(P.lh))) || ishail)) {  // 14042
          ngscnt = (ngscnt + 1);  // 14048
          igs(ngscnt) = ix;  // 14049
          kgs(ngscnt) = kz;  // 14050
          if ((ngscnt == ngs)) { goto L1100; }  // 14051
        }  // 14052
      }  // 14053
      nxmpb = 1;  // 14054
    }  // 14055
    L1100:;  // 14056
    if ((ngscnt == 0)) { goto L9998; }  // 14058
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) xv(i_0, i_1) = NR(0.0);  // 14065
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) xmas(i_0, i_1) = NR(0.0);  // 14066
    for (int i_2 = 1; i_2 <= 3; ++i_2) for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) vtxbar(i_0, i_1, i_2) = NR(0.0);  // 14067
    for (int i_2 = 1; i_2 <= 3; ++i_2) for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) xdia(i_0, i_1, i_2) = NR(0.0);  // 14068
    for (int i_1 = P.li; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) raindn(i_0, i_1) = NR(900.0);  // 14069
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) cx(i_0, i_1) = NR(0.0);  // 14070
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, i_1) = NR(0.0);  // 14072
    for (il = P.li; il <= P.lhab; ++il) {  // 14073
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14074
        rimdn(mgs, il) = rimedens;  // 14075
      }  // 14076
    }  // 14077
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14082
      kgsm(mgs) = c_max((kgs(mgs) - 1), 1);  // 14083
      kgsp(mgs) = c_min((kgs(mgs) + 1), (nz - 1));  // 14084
      kgsm2(mgs) = c_max((kgs(mgs) - 2), 1);  // 14085
      theta0(mgs) = an(igs(mgs), jy, kgs(mgs), P.lt);  // 14086
      thetap(mgs) = (an(igs(mgs), jy, kgs(mgs), P.lt) - theta0(mgs));  // 14087
      theta(mgs) = an(igs(mgs), jy, kgs(mgs), P.lt);  // 14088
      qv0(mgs) = an(igs(mgs), jy, kgs(mgs), P.lv);  // 14089
      qwvp(mgs) = (an(igs(mgs), jy, kgs(mgs), P.lv) - qv0(mgs));  // 14090
      pres(mgs) = (pn(igs(mgs), jy, kgs(mgs)) + pb(kgs(mgs)));  // 14092
      pipert(mgs) = p2(igs(mgs), jy, kgs(mgs));  // 14093
      rho0(mgs) = dn(igs(mgs), jy, kgs(mgs));  // 14094
      rhoinv(mgs) = (NR(1.0) / rho0(mgs));  // 14095
      rhovt(mgs) = sqrt((P.rho00 / c_max(NR(0.05), rho0(mgs))));  // 14096
      pi0(mgs) = (p2(igs(mgs), jy, kgs(mgs)) + pinit(kgs(mgs)));  // 14097
      temg(mgs) = t0(igs(mgs), jy, kgs(mgs));  // 14098
      temgkm1(mgs) = t0(igs(mgs), jy, kgsm(mgs));  // 14099
      temgkm2(mgs) = t0(igs(mgs), jy, kgsm2(mgs));  // 14100
      pk(mgs) = (p2(igs(mgs), jy, kgs(mgs)) + pinit(kgs(mgs)));  // 14101
      temcg(mgs) = (temg(mgs) - P.tfr);  // 14102
      qss0(mgs) = ((NR(380.0)) / (pres(mgs)));  // 14103
      pqs(mgs) = ((NR(380.0)) / (pres(mgs)));  // 14104
      ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 14105
      ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 14106
      qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 14111
      if (((P.iqis0 == 1) || (temg(mgs) <= (P.tfr + NR(0.5))))) {  // 14114
        qis(mgs) = (pqs(mgs) * P.tabqis(ltemq));  // 14115
      } else {  // 14116
        ltemq = ((((P.tfr - NR(163.15))) / P.fqsat) + NR(1.5));  // 14117
        qis(mgs) = (pqs(mgs) * P.tabqis(ltemq));  // 14118
      }  // 14119
      qss(mgs) = qvs(mgs);  // 14120
      cnostmp(mgs) = P.cno(P.ls);  // 14123
      il5(mgs) = 0;  // 14126
      if ((temg(mgs) < P.tfr)) {  // 14127
        il5(mgs) = 1;  // 14128
      }  // 14129
    }  // 14130
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14143
      qhshr(mgs) = NR(0.0);  // 14144
    }  // 14145
    for (il = P.lv; il <= P.lhab; ++il) {  // 14149
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14150
        qx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), il), NR(0.0));  // 14151
      }  // 14152
    }  // 14153
    for (int i_1 = P.ls; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) qxw(i_0, i_1) = NR(0.0);  // 14155
    for (int i_1 = P.lh; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) qxwlg(i_0, i_1) = NR(0.0);  // 14156
    if ((P.ipconc >= 1)) {  // 14169
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14170
        cx(mgs, P.li) = c_max(an(igs(mgs), jy, kgs(mgs), P.lni), NR(0.0));  // 14171
        if ((qx(mgs, P.li) <= P.qxmin(P.li))) {  // 14172
          cx(mgs, P.li) = NR(0.0);  // 14173
        }  // 14174
        cina(mgs) = cx(mgs, P.li);  // 14179
      }  // 14184
    }  // 14185
    if ((P.ipconc >= 2)) {  // 14186
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14187
        cx(mgs, P.lc) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnc), NR(0.0));  // 14188
        if ((qx(mgs, P.lc) <= P.qxmin(P.lc))) {  // 14190
          cx(mgs, P.lc) = NR(0.0);  // 14191
        }  // 14192
        if ((P.lccn > 1)) {  // 14196
          ccnc(mgs) = an(igs(mgs), jy, kgs(mgs), P.lccn);  // 14197
        } else {  // 14198
          ccnc(mgs) = NR(0.0);  // 14199
        }  // 14200
        if ((P.lccna > 1)) {  // 14201
          ccna(mgs) = an(igs(mgs), jy, kgs(mgs), P.lccna);  // 14202
        } else {  // 14203
          ccna(mgs) = cx(mgs, P.lc);  // 14204
        }  // 14205
      }  // 14206
    }  // 14209
    if ((P.ipconc >= 3)) {  // 14210
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14211
        cx(mgs, P.lr) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnr), NR(0.0));  // 14212
        if ((qx(mgs, P.lr) <= P.qxmin(P.lr))) {  // 14213
        } else if (((cx(mgs, P.lr) == NR(0.0)) && (qx(mgs, P.lr) < (NR(3.0) * P.qxmin(P.lr))))) {  // 14215
          qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, P.lr));  // 14216
          qx(mgs, P.lr) = NR(0.0);  // 14217
        } else {  // 14218
          cx(mgs, P.lr) = c_max(NR(1.0e-9), cx(mgs, P.lr));  // 14219
        }  // 14220
      }  // 14221
    }  // 14222
    if ((P.ipconc >= 4)) {  // 14223
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14224
        cx(mgs, P.ls) = c_max(an(igs(mgs), jy, kgs(mgs), P.lns), NR(0.0));  // 14225
        if ((qx(mgs, P.ls) <= P.qxmin(P.ls))) {  // 14226
        } else if (((cx(mgs, P.ls) == NR(0.0)) && (qx(mgs, P.ls) < (NR(3.0) * P.qxmin(P.ls))))) {  // 14228
          qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, P.ls));  // 14229
          qx(mgs, P.ls) = NR(0.0);  // 14230
        } else {  // 14231
          cx(mgs, P.ls) = c_max(NR(1.0e-9), cx(mgs, P.ls));  // 14232
          if ((P.ilimit >= P.ipc(P.ls))) {  // 14234
            tmp = (((xdn0(P.ls) * cx(mgs, P.ls))) / ((rho0(mgs) * qx(mgs, P.ls))));  // 14235
            tmp2 = pow(((tmp * (NR(3.14159)))), ((NR(1.0) / NR(3.0))));  // 14236
            cnox = (cx(mgs, P.ls) * (tmp2));  // 14237
            if ((cnox > (NR(3.0) * P.cno(P.ls)))) {  // 14238
              cx(mgs, P.ls) = ((NR(3.0) * P.cno(P.ls)) / tmp2);  // 14239
            }  // 14240
          }  // 14241
        }  // 14242
      }  // 14243
    }  // 14244
    if ((P.ipconc >= 5)) {  // 14245
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14246
        cx(mgs, P.lh) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnh), NR(0.0));  // 14248
        if ((qx(mgs, P.lh) <= P.qxmin(P.lh))) {  // 14249
        } else if (((cx(mgs, P.lh) == NR(0.0)) && (qx(mgs, P.lh) < (NR(3.0) * P.qxmin(P.lh))))) {  // 14251
          qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, P.lh));  // 14252
          qx(mgs, P.lh) = NR(0.0);  // 14253
        } else {  // 14254
          cx(mgs, P.lh) = c_max(NR(1.0e-9), cx(mgs, P.lh));  // 14255
          if ((P.ilimit >= P.ipc(P.lh))) {  // 14256
            tmp = (((xdn0(P.lh) * cx(mgs, P.lh))) / ((rho0(mgs) * qx(mgs, P.lh))));  // 14257
            tmp2 = pow(((tmp * (NR(3.14159)))), ((NR(1.0) / NR(3.0))));  // 14258
            cnox = (cx(mgs, P.lh) * (tmp2));  // 14259
            if ((cnox > (NR(3.0) * P.cno(P.lh)))) {  // 14260
              cx(mgs, P.lh) = ((NR(3.0) * P.cno(P.lh)) / tmp2);  // 14261
            }  // 14262
          }  // 14263
        }  // 14264
      }  // 14267
    }  // 14270
    if (((P.lhl > 1) && (P.ipconc >= 5))) {  // 14272
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14273
        cx(mgs, P.lhl) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnhl), NR(0.0));  // 14275
        if ((qx(mgs, P.lhl) <= P.qxmin(P.lhl))) {  // 14276
          cx(mgs, P.lhl) = NR(0.0);  // 14277
        } else if (((cx(mgs, P.lhl) == NR(0.0)) && (qx(mgs, P.lhl) < (NR(3.0) * P.qxmin(P.lhl))))) {  // 14278
          qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, P.lhl));  // 14279
          qx(mgs, P.lhl) = NR(0.0);  // 14280
        } else {  // 14281
          cx(mgs, P.lhl) = c_max(NR(1.0e-9), cx(mgs, P.lhl));  // 14282
          if ((P.ilimit >= P.ipc(P.lhl))) {  // 14283
            tmp = (((xdn0(P.lhl) * cx(mgs, P.lhl))) / ((rho0(mgs) * qx(mgs, P.lhl))));  // 14284
            tmp2 = pow(((tmp * (NR(3.14159)))), ((NR(1.0) / NR(3.0))));  // 14285
            cnox = (cx(mgs, P.lhl) * (tmp2));  // 14286
            if ((cnox > (NR(3.0) * P.cno(P.lhl)))) {  // 14287
              cx(mgs, P.lhl) = ((NR(3.0) * P.cno(P.lhl)) / tmp2);  // 14288
            }  // 14289
          }  // 14290
        }  // 14291
      }  // 14294
    }  // 14295
    if (ldovol) {  // 14300
      for (int i_1 = P.li; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) vx(i_0, i_1) = NR(0.0);  // 14302
      for (il = P.li; il <= P.lhab; ++il) {  // 14304
        if ((P.lvol(il) >= 1)) {  // 14306
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14308
            vx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), P.lvol(il)), NR(0.0));  // 14309
          }  // 14310
        }  // 14312
      }  // 14314
    }  // 14316
    for (int i_0 = 1; i_0 <= ngs; ++i_0) fhw(i_0) = NR(0.0);  // 14322
    for (int i_0 = 1; i_0 <= ngs; ++i_0) fsw(i_0) = NR(0.0);  // 14323
    for (int i_0 = 1; i_0 <= ngs; ++i_0) fhlw(i_0) = NR(0.0);  // 14324
    if ((P.ipconc >= 6)) {  // 14332
      for (int i_1 = P.lr; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) zx(i_0, i_1) = NR(0.0);  // 14333
      for (il = P.lr; il <= P.lhab; ++il) {  // 14334
        if ((P.lz(il) > 1)) {  // 14335
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14336
            zx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), P.lz(il)), NR(0.0));  // 14337
          }  // 14338
        }  // 14339
      }  // 14340
    }  // 14342
    if ((P.ipconc >= 6)) {  // 14344
      tmp = (P.alphamax - NR(1.0));  // 14346
      g1xmax = (((((NR(6.0) + tmp)) * ((NR(5.0) + tmp))) * ((NR(4.0) + tmp))) / (((((NR(3.0) + tmp)) * ((NR(2.0) + tmp))) * ((NR(1.0) + tmp)))));  // 14347
      g1xmin = (((((NR(6.0) + P.alphamin)) * ((NR(5.0) + P.alphamin))) * ((NR(4.0) + P.alphamin))) / (((((NR(3.0) + P.alphamin)) * ((NR(2.0) + P.alphamin))) * ((NR(1.0) + P.alphamin)))));  // 14349
      for (int i_0 = 1; i_0 <= ngs; ++i_0) g1x(i_0, P.lr) = (((((NR(6.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(4.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 14353
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14357
        if (((cx(mgs, P.lr) > NR(0.0)) && (qx(mgs, P.lr) > P.qxmin(P.lr)))) {  // 14358
          vr = ((rho0(mgs) * qx(mgs, P.lr)) / ((NR(1000.0) * cx(mgs, P.lr))));  // 14360
          zx(mgs, P.lr) = (((NR(3.6476) * g1x(mgs, P.lr)) * cx(mgs, P.lr)) * ipow<2>(vr));  // 14365
        }  // 14369
      }  // 14370
    }  // 14373
    if ((P.ipconc == 5)) {  // 14376
      for (int i_0 = 1; i_0 <= ngs; ++i_0) g1x(i_0, P.lr) = (((((NR(6.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(4.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 14378
      for (int i_0 = 1; i_0 <= ngs; ++i_0) g1x(i_0, P.lh) = (((((NR(6.0) + P.alphah)) * ((NR(5.0) + P.alphah))) * ((NR(4.0) + P.alphah))) / (((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))));  // 14380
      if ((P.lhl > 0)) {  // 14382
        for (int i_0 = 1; i_0 <= ngs; ++i_0) g1x(i_0, P.lhl) = (((((NR(6.0) + P.alphahl)) * ((NR(5.0) + P.alphahl))) * ((NR(4.0) + P.alphahl))) / (((((NR(3.0) + P.alphahl)) * ((NR(2.0) + P.alphahl))) * ((NR(1.0) + P.alphahl)))));  // 14383
      }  // 14385
    }  // 14386
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) scx(i_0, i_1) = NR(0.0);  // 14388
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.lr) = P.alphar;  // 14394
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.li) = P.xnu(P.li);  // 14399
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.lc) = P.xnu(P.lc);  // 14400
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.ls) = P.xnu(P.ls);  // 14405
    for (il = P.lr; il <= P.lhab; ++il) {  // 14410
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14411
        if ((il >= P.lg)) { alpha(mgs, il) = P.dnu(il); }  // 14412
        for (ic = P.lc; ic <= P.lhab; ++ic) {  // 14415
          dab0lh(mgs, il, ic) = P.dab0(il, ic);  // 14416
          dab1lh(mgs, il, ic) = P.dab1(il, ic);  // 14417
        }  // 14418
      }  // 14419
    }  // 14420
    for (il = P.lr; il <= P.lhab; ++il) {  // 14424
      for (int i_0 = 1; i_0 <= ngs; ++i_0) da0lx(i_0, il) = P.da0(il);  // 14425
    }  // 14426
    for (int i_0 = 1; i_0 <= ngs; ++i_0) da0lh(i_0) = P.da0(P.lh);  // 14427
    for (int i_0 = 1; i_0 <= ngs; ++i_0) da0lr(i_0) = P.da0(P.lr);  // 14428
    for (int i_0 = 1; i_0 <= ngs; ++i_0) da1lr(i_0) = P.da1(P.lr);  // 14429
    for (int i_0 = 1; i_0 <= ngs; ++i_0) da0lc(i_0) = P.da0(P.lc);  // 14430
    for (int i_0 = 1; i_0 <= ngs; ++i_0) da1lc(i_0) = P.da1(P.lc);  // 14431
    for (int i_0 = 1; i_0 <= ngs; ++i_0) rzxhlh(i_0) = (P.rzhl / P.rz);  // 14436
    for (int i_0 = 1; i_0 <= ngs; ++i_0) rzxh(i_0) = P.rz;  // 14444
    for (int i_0 = 1; i_0 <= ngs; ++i_0) rzxhl(i_0) = P.rzhl;  // 14445
    for (int i_0 = 1; i_0 <= ngs; ++i_0) rzxs(i_0) = P.rzs;  // 14449
    if ((P.lhl > 1)) {  // 14455
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14456
        da0lhl(mgs) = P.da0(P.lhl);  // 14457
      }  // 14458
    }  // 14459
    for (int i_0 = 1; i_0 <= ngs; ++i_0) ventrx(i_0) = ventr;  // 14461
    for (int i_0 = 1; i_0 <= ngs; ++i_0) ventrxn(i_0) = P.ventrn;  // 14462
    for (int i_0 = 1; i_0 <= ngs; ++i_0) gf1palp(i_0) = gamma_sp((NR(1.0) + P.alphar));  // 14463
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14468
      ssi(mgs) = (qx(mgs, P.lv) / qis(mgs));  // 14470
      ssw(mgs) = (qx(mgs, P.lv) / qvs(mgs));  // 14471
      tsqr(mgs) = ipow<2>(temg(mgs));  // 14473
      temgx(mgs) = c_min(temg(mgs), NR(313.15));  // 14475
      temgx(mgs) = c_max(temgx(mgs), NR(233.15));  // 14476
      felv(mgs) = (NR(2500837.367) * pow(((NR(273.15) / temgx(mgs))), (((NR(0.167)) + ((NR(3.67e-4)) * temgx(mgs))))));  // 14477
      temcgx(mgs) = c_min(temg(mgs), NR(273.15));  // 14479
      temcgx(mgs) = c_max(temcgx(mgs), NR(223.15));  // 14480
      temcgx(mgs) = (temcgx(mgs) - NR(273.15));  // 14481
      felf(mgs) = ((NR(333690.6098) + ((NR(2030.61425)) * temcgx(mgs))) - ((NR(10.46708312)) * ipow<2>(temcgx(mgs))));  // 14484
      fels(mgs) = (felv(mgs) + felf(mgs));  // 14486
      felvs(mgs) = (felv(mgs) * felv(mgs));  // 14488
      felss(mgs) = (fels(mgs) * fels(mgs));  // 14489
      felvcp(mgs) = (felv(mgs) * P.cpi);  // 14492
      felscp(mgs) = (fels(mgs) * P.cpi);  // 14493
      felfcp(mgs) = (felf(mgs) * P.cpi);  // 14494
      fgamw(mgs) = (felvcp(mgs) / pi0(mgs));  // 14529
      fgams(mgs) = (felscp(mgs) / pi0(mgs));  // 14530
      fcqv1(mgs) = ((NR(4098.0258) * pi0(mgs)) * fgamw(mgs));  // 14532
      fcqv2(mgs) = ((NR(5807.6953) * pi0(mgs)) * fgams(mgs));  // 14533
      fcc3(mgs) = (felfcp(mgs) / pi0(mgs));  // 14534
      fwvdf(mgs) = (((NR(2.11e-05)) * (pow(((temg(mgs) / P.tfr)), NR(1.94)))) * ((NR(101325.0) / (pres(mgs)))));  // 14537
      fadvisc(mgs) = ((P.advisc0 * ((NR(416.16) / ((temg(mgs) + NR(120.0)))))) * pow(((temg(mgs) / NR(296.0))), (NR(1.5))));  // 14541
      fakvisc(mgs) = (fadvisc(mgs) * rhoinv(mgs));  // 14543
      temcgx(mgs) = c_min(temg(mgs), NR(273.15));  // 14545
      temcgx(mgs) = c_max(temcgx(mgs), NR(233.15));  // 14546
      temcgx(mgs) = (temcgx(mgs) - NR(273.15));  // 14547
      fci(mgs) = (((NR(2.118636) + (NR(0.007371) * (temcgx(mgs))))) * (NR(1.0e+03)));  // 14548
      if ((temg(mgs) < NR(273.15))) {  // 14550
        temcgx(mgs) = c_min(temg(mgs), NR(273.15));  // 14551
        temcgx(mgs) = c_max(temcgx(mgs), NR(233.15));  // 14552
        temcgx(mgs) = (temcgx(mgs) - NR(273.15));  // 14553
        fcw(mgs) = ((NR(4203.1548) + ((NR(1.30572e-2)) * (ipow<2>(((temcgx(mgs) - NR(35.0))))))) + ((NR(1.60056e-5)) * (ipow<4>(((temcgx(mgs) - NR(35.0)))))));  // 14554
      }  // 14556
      if ((temg(mgs) >= NR(273.15))) {  // 14557
        temcgx(mgs) = c_min(temg(mgs), NR(308.15));  // 14558
        temcgx(mgs) = c_max(temcgx(mgs), NR(273.15));  // 14559
        temcgx(mgs) = (temcgx(mgs) - NR(273.15));  // 14560
        fcw(mgs) = (NR(4243.1688) + ((NR(3.47104e-1)) * (ipow<2>(temcgx(mgs)))));  // 14561
      }  // 14562
      ftka(mgs) = ((P.tka0 * fadvisc(mgs)) / P.advisc1);  // 14564
      fthdf(mgs) = ((ftka(mgs) * P.cpi) * rhoinv(mgs));  // 14565
      fschm(mgs) = ((fakvisc(mgs) / fwvdf(mgs)));  // 14567
      fpndl(mgs) = ((fakvisc(mgs) / fthdf(mgs)));  // 14568
      fai(mgs) = ((ipow<2>(fels(mgs))) / (((ftka(mgs) * P.rw) * ipow<2>(temg(mgs)))));  // 14570
      fbi(mgs) = ((NR(1.0) / (((rho0(mgs) * fwvdf(mgs)) * qis(mgs)))));  // 14571
      fav(mgs) = ((ipow<2>(felv(mgs))) / (((ftka(mgs) * P.rw) * ipow<2>(temg(mgs)))));  // 14572
      fbv(mgs) = ((NR(1.0) / (((rho0(mgs) * fwvdf(mgs)) * qvs(mgs)))));  // 14573
      kp1 = c_min(nz, (kgs(mgs) + 1));  // 14575
      wvel(mgs) = ((NR(0.5)) * ((w(igs(mgs), jgs, kp1) + w(igs(mgs), jgs, kgs(mgs)))));  // 14576
    }  // 14580
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 14592
      xdn(mgs, P.li) = xdn0(P.li);  // 14593
      xdn(mgs, P.lc) = xdn0(P.lc);  // 14594
      xdn(mgs, P.lr) = xdn0(P.lr);  // 14595
      xdn(mgs, P.ls) = xdn0(P.ls);  // 14596
      xdn(mgs, P.lh) = xdn0(P.lh);  // 14597
      if ((P.lvol(P.ls) > 1)) {  // 14598
        if (((vx(mgs, P.ls) > NR(0.0)) && (qx(mgs, P.ls) > P.qxmin(P.ls)))) {  // 14599
          xdn(mgs, P.ls) = c_min(xdnmx(P.ls), c_max(xdnmn(P.ls), ((rho0(mgs) * qx(mgs, P.ls)) / vx(mgs, P.ls))));  // 14600
        }  // 14601
      }  // 14602
      if ((P.lvol(P.lh) > 1)) {  // 14604
        if (((vx(mgs, P.lh) > NR(0.0)) && (qx(mgs, P.lh) > P.qxmin(P.lh)))) {  // 14605
          dnmx = xdnmx(P.lh);  // 14608
          xdn(mgs, P.lh) = c_min(dnmx, c_max(xdnmn(P.lh), ((rho0(mgs) * qx(mgs, P.lh)) / vx(mgs, P.lh))));  // 14610
          vx(mgs, P.lh) = ((rho0(mgs) * qx(mgs, P.lh)) / xdn(mgs, P.lh));  // 14611
        } else if (((vx(mgs, P.lh) == NR(0.0)) && (qx(mgs, P.lh) > P.qxmin(P.lh)))) {  // 14613
          vx(mgs, P.lh) = ((rho0(mgs) * qx(mgs, P.lh)) / xdn(mgs, P.lh));  // 14615
        }  // 14617
      }  // 14618
      if ((P.lhl > 1)) {  // 14621
        xdn(mgs, P.lhl) = xdn0(P.lhl);  // 14623
        xdntmp(mgs, P.lhl) = xdn0(P.lhl);  // 14624
        if ((P.lvol(P.lhl) > 1)) {  // 14626
          if (((vx(mgs, P.lhl) > NR(0.0)) && (qx(mgs, P.lhl) > P.qxmin(P.lhl)))) {  // 14627
            dnmx = xdnmx(P.lhl);  // 14631
            xdn(mgs, P.lhl) = c_min(dnmx, c_max(xdnmn(P.lhl), ((rho0(mgs) * qx(mgs, P.lhl)) / vx(mgs, P.lhl))));  // 14634
            vx(mgs, P.lhl) = ((rho0(mgs) * qx(mgs, P.lhl)) / xdn(mgs, P.lhl));  // 14635
            xdntmp(mgs, P.lhl) = xdn(mgs, P.lhl);  // 14636
          } else if (((vx(mgs, P.lhl) == NR(0.0)) && (qx(mgs, P.lhl) > P.qxmin(P.lhl)))) {  // 14638
            vx(mgs, P.lhl) = ((rho0(mgs) * qx(mgs, P.lhl)) / xdn(mgs, P.lhl));  // 14640
          }  // 14642
        }  // 14643
      }  // 14645
    }  // 14648
    alphashr = P.alphar;  // 14775
    alphamlr = P.alphar;  // 14776
    alphasmlr = P.alphar;  // 14777
    massfacshr = (ipow<3>(((NR(3.0) + alphashr))) / (((((NR(3.0) + alphashr)) * ((NR(2.0) + alphashr))) * ((NR(1.0) + alphashr)))));  // 14781
    massfacmlr = (ipow<3>(((NR(3.0) + alphamlr))) / (((((NR(3.0) + alphamlr)) * ((NR(2.0) + alphamlr))) * ((NR(1.0) + alphamlr)))));  // 14782
    g1shr = NR(1.0);  // 14787
    g1mlr = NR(1.0);  // 14788
    g1smlr = NR(1.0);  // 14789
    if ((P.ipconc >= 6)) {  // 14793
    }  // 15053
    if ((P.ipconc >= 6)) {  // 15056
      for (il = P.lr; il <= P.lhab; ++il) {  // 15058
        if ((((!(((il == P.lr) && (P.imurain == 3))))) && (((((il == P.lr) || (il == P.lh)) || (il == P.lhl)) || (il == P.lf))))) {  // 15061
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15062
            g1x(mgs, il) = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 15063
          }  // 15065
        }  // 15066
        if (((P.lz(il) > 1) && ((!(((il == P.lr) && (P.imurain == 3))))))) {  // 15068
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15070
            if ((((P.iresetmoments == 1) || (P.iresetmoments == il)) || (P.iresetmoments == (-1)))) {  // 15073
              if ((zx(mgs, il) <= P.zxmin)) {  // 15074
                qx(mgs, il) = NR(0.0);  // 15076
                cx(mgs, il) = NR(0.0);  // 15077
                zx(mgs, il) = NR(0.0);  // 15078
                an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 15079
                an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 15080
                an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 15081
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15082
              } else if (((cx(mgs, il) <= P.cxmin) && (P.iresetmoments != (-1)))) {  // 15093
                qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, il));  // 15094
                zx(mgs, il) = NR(0.0);  // 15095
                cx(mgs, il) = NR(0.0);  // 15096
                qx(mgs, il) = NR(0.0);  // 15097
                an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 15098
                an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 15099
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15100
                an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 15101
              }  // 15102
            }  // 15103
            if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= P.cxmin))) {  // 15105
              zx(mgs, il) = NR(0.0);  // 15106
              cx(mgs, il) = NR(0.0);  // 15107
              an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 15108
              qx(mgs, il) = NR(0.0);  // 15110
              an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 15111
              an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 15112
              an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15113
            }  // 15114
            if ((qx(mgs, il) > P.qxmin(il))) {  // 15116
              xv(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / ((xdn(mgs, il) * c_max(NR(1.0e-9), cx(mgs, il)))));  // 15118
              xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 15119
              if ((xv(mgs, il) < P.xvmn(il))) {  // 15121
                xv(mgs, il) = c_min(P.xvmx(il), c_max(P.xvmn(il), xv(mgs, il)));  // 15122
                xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 15123
                cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 15124
              }  // 15125
              if (((zx(mgs, il) > NR(0.0)) && (cx(mgs, il) <= NR(0.0)))) {  // 15127
                g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 15129
                z = zx(mgs, il);  // 15131
                qr = qx(mgs, il);  // 15132
                cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((NR(6.0) * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 15134
              } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) > P.cxmin))) {  // 15136
                chw = cx(mgs, il);  // 15140
                qr = qx(mgs, il);  // 15141
                g1 = (((((NR(6.0) + P.alphamax)) * ((NR(5.0) + P.alphamax))) * ((NR(4.0) + P.alphamax))) / (((((NR(3.0) + P.alphamax)) * ((NR(2.0) + P.alphamax))) * ((NR(1.0) + P.alphamax)))));  // 15144
                zx(mgs, il) = c_max((P.zxmin * NR(1.1)), (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((6 * qr)))) / ((chw * ipow<2>(((P.pi * xdn(mgs, il))))))));  // 15146
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15147
              } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= NR(0.0)))) {  // 15149
                zx(mgs, il) = ((NR(1.0e-19) / NR(0.224)) * ipow<2>(((xdn0(P.lr) / xdn0(il)))));  // 15153
                an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15154
                g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 15156
                z = zx(mgs, il);  // 15158
                qr = qx(mgs, il);  // 15159
                cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((NR(6.0) * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 15161
                an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 15162
              } else {  // 15163
                chw = cx(mgs, il);  // 15165
                qr = qx(mgs, il);  // 15166
                z = zx(mgs, il);  // 15167
                if ((zx(mgs, il) > NR(0.0))) {  // 15169
                  rdi = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((rho0(mgs) * qr)))));  // 15172
                  alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 15176
                  alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 15179
                  for (i = 1; i <= 10; ++i) {  // 15190
                    if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 15192
                    alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 15193
                    alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 15196
                    alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 15199
                  }  // 15200
                  if (((P.imaxdiaopt == 1) || (il != P.lr))) {  // 15205
                    xvbarmax = P.xvmx(il);  // 15206
                  } else {  // 15209
                    xvbarmax = (P.xvmx(il) / ((ipow<3>(((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))))));  // 15210
                  }  // 15213
                  if (((xv(mgs, il) > xvbarmax) || ((((il == P.lr) && (P.ioldlimiter >= 2)) && (xv(mgs, il) > (P.xvmx(il) / NR(8.0))))))) {  // 15215
                    tmp = cx(mgs, il);  // 15216
                    xv(mgs, il) = c_min(xvbarmax, c_max(P.xvmn(il), xv(mgs, il)));  // 15225
                    xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 15226
                    cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 15227
                    if ((tmp < cx(mgs, il))) {  // 15229
                      g1 = ((((NR(36.0) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / ((((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il)))) * ipow<2>(P.pi))));  // 15230
                      tmpz = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / tmp);  // 15235
                      if ((tmpz > zx(mgs, il))) {  // 15236
                        tmpc = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / zx(mgs, il));  // 15237
                        cx(mgs, il) = c_max(cx(mgs, il), tmpc);  // 15238
                      }  // 15240
                      zx(mgs, il) = (((g1 / ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * (ipow<2>(((rho0(mgs) * qx(mgs, il)))))) / cx(mgs, il));  // 15241
                      an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 15242
                      qr = qx(mgs, il);  // 15244
                      chw = cx(mgs, il);  // 15245
                      z = zx(mgs, il);  // 15246
                      rdi = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((rho0(mgs) * qr)))));  // 15248
                      alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 15249
                      for (i = 1; i <= 10; ++i) {  // 15251
                        if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 15252
                        alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 15253
                        alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 15254
                        alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 15256
                      }  // 15257
                    }  // 15260
                  }  // 15261
                  g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 15267
                  if ((((lrescalelow(il) || P.rescale_high_alpha)) && ((((alpha(mgs, il) <= P.alphamin) || (alp == P.alphamin)) || (alp == P.alphamax))))) {  // 15270
                    if ((((lrescalelow(il) && (alp <= P.alphamin)) && (!(((il == P.lh) && (P.icvhl2h > 0))))) && (!(((il == P.lr) && (!P.rescale_low_alphar)))))) {  // 15279
                      wtest = false;  // 15281
                      wtest = (((temcg(mgs) > P.rescale_tempthresh) && (qx(mgs, P.lc) > P.qxmin(P.lc))) && (wvel(mgs) < P.rescale_wthresh));  // 15289
                      if (((il == P.lr) && (wtest))) {  // 15292
                        chw = (((((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * (qr)) * qr) / z) * ipow<2>(((NR(6.0) / ((P.pi * xdn(mgs, il)))))));  // 15296
                        cx(mgs, il) = chw;  // 15297
                        an(igs(mgs), jy, kgs(mgs), P.ln(il)) = chw;  // 15298
                      } else {  // 15299
                        z1 = ((((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * (qr)) * qr) / chw);  // 15302
                        z = (z1 * ipow<2>(((NR(6.0) / ((P.pi * xdn(mgs, il)))))));  // 15303
                        zx(mgs, il) = z;  // 15304
                        an(igs(mgs), jy, kgs(mgs), P.lz(il)) = z;  // 15305
                      }  // 15306
                    }  // 15307
                  }  // 15308
                  if ((alp >= (P.alphamax - NR(0.5)))) {  // 15317
                    g1x(mgs, il) = (((ipow<2>(((P.pi * xdn(mgs, il)))) * zx(mgs, il)) * cx(mgs, il)) / (ipow<2>((((NR(6.0) * rho0(mgs)) * qx(mgs, il))))));  // 15320
                  } else {  // 15321
                    g1x(mgs, il) = g1;  // 15322
                  }  // 15323
                }  // 15325
              }  // 15334
              if ((il == P.lr)) {  // 15336
                tmp = (alpha(mgs, P.lr) + NR(1.0));  // 15352
                i = int((P.dgami * (tmp)));  // 15353
                del = (tmp - (P.dgam * i));  // 15354
                y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 15355
                gf1palp(mgs) = y;  // 15357
                tmp = ((alpha(mgs, P.lr) + NR(2.5)) + (P.br / NR(2.0)));  // 15361
                i = int((P.dgami * (tmp)));  // 15362
                del = (tmp - (P.dgam * i));  // 15363
                x = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 15364
                ventrxn(mgs) = (x / y);  // 15367
              }  // 15371
            } else {  // 15374
            }  // 15382
          }  // 15387
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15398
            if ((qx(mgs, il) > P.qxmin(il))) {  // 15399
              xnutmp = (((alpha(mgs, il) - NR(2.0))) / NR(3.0));  // 15400
              for (ic = P.lc; ic <= (P.lh - 1); ++ic) {  // 15403
                if (((il != ic) && (qx(mgs, ic) > P.qxmin(ic)))) {  // 15404
                  xnuc = P.xnu(ic);  // 15405
                  i = nint((alpha(mgs, il) * P.dqiacralphainv));  // 15421
                  if (((((ic == P.lc) || (ic == P.li)) || (ic == P.ls)) || (((ic == P.lr) && (P.imurain == 3))))) {  // 15422
                    alp = (((NR(3.0) * alpha(mgs, ic)) + NR(2.0)));  // 15423
                    j = nint(((((NR(3.0) * alpha(mgs, ic)) + NR(2.0))) * P.dqiacralphainv));  // 15424
                  } else {  // 15425
                    alp = alpha(mgs, ic);  // 15426
                    j = nint((alpha(mgs, ic) * P.dqiacralphainv));  // 15427
                  }  // 15428
                  dab0lh(mgs, ic, il) = nssl_trap<Real>();  // 15430
                  dab1lh(mgs, ic, il) = nssl_trap<Real>();  // 15431
                  dab0lh(mgs, il, ic) = nssl_trap<Real>();  // 15432
                  dab1lh(mgs, il, ic) = nssl_trap<Real>();  // 15433
                }  // 15457
              }  // 15458
              da0lx(mgs, il) = delbk(P, P.bb(il), xnutmp, P.xmu(il), 0);  // 15462
              if ((il == P.lh)) {  // 15463
                da0lh(mgs) = delbk(P, P.bb(il), xnutmp, P.xmu(il), 0);  // 15464
                rzxh(mgs) = (((((((NR(4.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(1.0) + P.xnu(P.lr))))) / ((((((NR(1.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(3.0) + alpha(mgs, il)))) * ((NR(2.0) + P.xnu(P.lr))))));  // 15468
                rzxhlh(mgs) = (rzxhl(mgs) / ((((((((NR(4.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(1.0) + P.xnu(P.lr))))) / ((((((NR(1.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(3.0) + alpha(mgs, il)))) * ((NR(2.0) + P.xnu(P.lr))))))));  // 15473
              } else if ((il == P.lhl)) {  // 15476
                da0lhl(mgs) = delbk(P, P.bb(il), xnutmp, P.xmu(il), 0);  // 15477
                rzxhl(mgs) = (((((((NR(4.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(1.0) + P.xnu(P.lr))))) / ((((((NR(1.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(3.0) + alpha(mgs, il)))) * ((NR(2.0) + P.xnu(P.lr))))));  // 15481
              } else if ((il == P.lr)) {  // 15484
                xnutmp = (((alpha(mgs, il) - NR(2.0))) / NR(3.0));  // 15485
                da0lr(mgs) = delbk(P, P.bb(il), xnutmp, P.xmu(il), 0);  // 15486
                da1lr(mgs) = delbk(P, P.bb(il), xnutmp, P.xmu(il), 1);  // 15487
              }  // 15488
            }  // 15490
          }  // 15491
        }  // 15497
      }  // 15499
    }  // 15501
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15508
      kp1 = c_min(nz, (kgs(mgs) + 1));  // 15509
      wvelkm1(mgs) = ((NR(0.5)) * ((w(igs(mgs), jgs, kgs(mgs)) + w(igs(mgs), jgs, kgsm(mgs)))));  // 15514
      cninm(mgs) = t7(igs(mgs), jgs, kgsm(mgs));  // 15516
      cnina(mgs) = t7(igs(mgs), jgs, kgs(mgs));  // 15517
      cninp(mgs) = t7(igs(mgs), jgs, kgsp(mgs));  // 15518
    }  // 15519
    infdo = 1;  // 15535
    setvtz(P, W, ngscnt, qx.p, CP(P.qxmin.v), qxw.p, cx.p, rho0.p, rhovt.p, xdia.p, CP(P.cno.v), cnostmp.p, xmas.p, vtxbar.p, xdn.p, CP(P.xvmn.v), CP(P.xvmx.v), xv.p, cdx.p, cdxgs.p, P.ipconc, P.ndebug, ngs, nz, igs.p, kgs.p, fadvisc.p, P.cwmasn, P.cwmasx, P.cwradn, cnina.p, P.cimn, P.cimx, P.itype1, P.itype2, temcg.p, infdo, alpha.p, axx.p, bxx.p, 0);  // 15538
    if ((P.ipconc < 1)) {  // 15563
      for (int i_0 = 1; i_0 <= ngscnt; ++i_0) cina(i_0) = cx(i_0, P.li);  // 15564
    }  // 15565
    if ((P.ipconc < 5)) {  // 15566
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15567
        if ((P.ipconc < 3)) {  // 15570
          if ((qx(mgs, P.lr) > P.qxmin(P.lh))) {  // 15572
          }  // 15575
        }  // 15576
        if ((P.ipconc < 4)) {  // 15578
          if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 15581
          }  // 15584
        }  // 15585
        if ((P.ipconc < 5)) {  // 15587
          if ((qx(mgs, P.lh) > P.qxmin(P.lh))) {  // 15591
          }  // 15595
        }  // 15597
      }  // 15599
    }  // 15600
    if ((P.ipconc >= 2)) {  // 15602
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15603
        rb(mgs) = ((NR(0.5) * xdia(mgs, P.lc, 1)) * pow(((NR(1.0) / ((NR(1.0) + alpha(mgs, P.lc))))), ((NR(1.0) / NR(6.0)))));  // 15605
        xl2p(mgs) = c_max(0.0e0, ((((NR(2.7e-2) * xdn(mgs, P.lc)) * cx(mgs, P.lc)) * xv(mgs, P.lc)) * (((((NR(0.5e20) * ipow<3>(rb(mgs))) * xdia(mgs, P.lc, 1))) - NR(0.4)))));  // 15606
        if ((rb(mgs) > NR(3.51e-6))) {  // 15608
          rh(mgs) = c_max(41.0e-6, (6.3e-4 / ((1.0e6 * ((rb(mgs) - 3.5e-6))))));  // 15610
        } else {  // 15611
          rh(mgs) = 41.0e-6;  // 15612
        }  // 15613
        if ((xl2p(mgs) > NR(0.0))) {  // 15614
          nh(mgs) = (4.2e9 * xl2p(mgs));  // 15615
        } else {  // 15616
          nh(mgs) = NR(1.0e30);  // 15617
        }  // 15618
      }  // 15619
    }  // 15620
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15632
      qvimxd(mgs) = ((NR(0.70) * ((qx(mgs, P.lv) - qis(mgs)))) * dtpinv);  // 15633
      if ((qx(mgs, P.lc) < P.qxmin(P.lc))) { qvimxd(mgs) = ((NR(0.99) * ((qx(mgs, P.lv) - qis(mgs)))) * dtpinv); }  // 15635
      qvimxd(mgs) = c_max(qvimxd(mgs), NR(0.0));  // 15637
      frac = 0.1e0;  // 15639
      qimxd(mgs) = ((frac * qx(mgs, P.li)) * dtpinv);  // 15640
      qcmxd(mgs) = ((frac * qx(mgs, P.lc)) * dtpinv);  // 15641
      qrmxd(mgs) = ((frac * qx(mgs, P.lr)) * dtpinv);  // 15642
      qsmxd(mgs) = ((frac * qx(mgs, P.ls)) * dtpinv);  // 15643
      qhmxd(mgs) = ((frac * qx(mgs, P.lh)) * dtpinv);  // 15644
      if ((P.lhl > 1)) { qhlmxd(mgs) = ((frac * qx(mgs, P.lhl)) * dtpinv); }  // 15645
    }  // 15646
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15652
      if ((qx(mgs, P.lc) <= P.qxmin(P.lc))) {  // 15654
        ccmxd(mgs) = ((NR(0.20) * cx(mgs, P.lc)) * dtpinv);  // 15655
      } else {  // 15656
        if ((P.ipconc >= 2)) {  // 15657
          ccmxd(mgs) = ((frac * cx(mgs, P.lc)) * dtpinv);  // 15658
        } else {  // 15659
          ccmxd(mgs) = ((frac * qx(mgs, P.lc)) / (((xmas(mgs, P.lc) * rho0(mgs)) * dtp)));  // 15660
        }  // 15661
      }  // 15662
      if ((qx(mgs, P.li) <= P.qxmin(P.li))) {  // 15664
        cimxd(mgs) = ((frac * cx(mgs, P.li)) * dtpinv);  // 15665
      } else {  // 15666
        if ((P.ipconc >= 1)) {  // 15667
          cimxd(mgs) = ((frac * cx(mgs, P.li)) * dtpinv);  // 15668
        } else {  // 15669
          cimxd(mgs) = ((frac * qx(mgs, P.li)) / (((xmas(mgs, P.li) * rho0(mgs)) * dtp)));  // 15670
        }  // 15671
      }  // 15672
      crmxd(mgs) = ((NR(0.10) * cx(mgs, P.lr)) * dtpinv);  // 15675
      csmxd(mgs) = ((frac * cx(mgs, P.ls)) * dtpinv);  // 15676
      chmxd(mgs) = ((frac * cx(mgs, P.lh)) * dtpinv);  // 15677
      ccmxd(mgs) = ((frac * cx(mgs, P.lc)) * dtpinv);  // 15679
      cimxd(mgs) = ((frac * cx(mgs, P.li)) * dtpinv);  // 15680
      crmxd(mgs) = ((frac * cx(mgs, P.lr)) * dtpinv);  // 15681
      csmxd(mgs) = ((frac * cx(mgs, P.ls)) * dtpinv);  // 15682
      chmxd(mgs) = ((frac * cx(mgs, P.lh)) * dtpinv);  // 15683
      qxmxd(mgs, P.lv) = c_max(NR(0.0), ((NR(0.1) * ((qx(mgs, P.lv) - qvs(mgs)))) * dtpinv));  // 15685
      for (il = P.lc; il <= P.lhab; ++il) {  // 15687
        qxmxd(mgs, il) = ((frac * qx(mgs, il)) * dtpinv);  // 15688
        cxmxd(mgs, il) = ((frac * cx(mgs, il)) * dtpinv);  // 15689
      }  // 15690
    }  // 15692
    if ((P.ipconc >= 6)) {  // 15697
      frac = 0.4e0;  // 15698
      for (int i_1 = P.lr; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) zxmxd(i_0, i_1) = NR(0.0);  // 15699
      for (il = P.lr; il <= P.lhab; ++il) {  // 15700
        if (((P.lz(il) > 0) || ((il == P.lr)))) {  // 15701
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15702
            zxmxd(mgs, il) = ((frac * zx(mgs, il)) * dtpinv);  // 15703
          }  // 15704
        }  // 15705
      }  // 15706
    }  // 15707
    maxmassfac(P.lc) = ((ipow<3>(((NR(2.0) + (NR(3.0) * ((NR(1.0) + P.xnu(P.lc))))))) / ((NR(3.0) * ((NR(1.0) + P.xnu(P.lc)))))));  // 15713
    maxmassfac(P.li) = ((ipow<3>(((NR(2.0) + (NR(3.0) * ((NR(1.0) + P.xnu(P.li))))))) / ((NR(3.0) * ((NR(1.0) + P.xnu(P.li)))))));  // 15714
    maxmassfac(P.lr) = (ipow<3>(((NR(3.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 15719
    maxmassfac(P.ls) = ((ipow<3>(((NR(2.0) + (NR(3.0) * ((NR(1.0) + P.alphas)))))) / ((NR(3.0) * ((NR(1.0) + P.alphas))))));  // 15724
    maxmassfac(P.lh) = (ipow<3>(((NR(3.0) + P.alphah))) / (((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))));  // 15730
    if ((P.lhl > 1)) {  // 15733
      maxmassfac(P.lhl) = (ipow<3>(((NR(3.0) + P.alphahl))) / (((((NR(3.0) + P.alphahl)) * ((NR(2.0) + P.alphahl))) * ((NR(1.0) + P.alphahl)))));  // 15734
    }  // 15736
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15740
      for (il = P.lh; il <= P.lhab; ++il) {  // 15741
        vshdgs(mgs, il) = vshd;  // 15743
        if (((qx(mgs, il) > P.qxmin(il)) && (P.ivshdgs > 0))) {  // 15745
          tmpdiam = (((shedalp + NR(0.0))) * xdia(mgs, il, 1));  // 15749
          if ((tmpdiam > P.sheddiam0)) {  // 15751
            vshdgs(mgs, il) = ((NR(0.523599) * NR(hexd(0x3E2CFDB417C18A1Bull))) / massfacshr);  // 15752
          } else if ((tmpdiam > P.sheddiam)) {  // 15753
            vshdgs(mgs, il) = ((NR(0.523599) * NR(hexd(0x3E5CFDB417C18A1Bull))) / massfacshr);  // 15754
          } else {  // 15755
            vshdgs(mgs, il) = (c_min(P.xvmx(P.lr), ((((NR(6.0) / P.pi) * xdn(mgs, il)) * NR(0.001)) * ipow<3>(tmpdiam))) / massfacshr);  // 15757
          }  // 15758
        }  // 15759
      }  // 15760
    }  // 15761
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 15773
      qcwresv(mgs) = NR(0.0);  // 15777
      ccwresv(mgs) = NR(0.0);  // 15778
      erw(mgs) = NR(0.0);  // 15780
      esw(mgs) = NR(0.0);  // 15781
      ehw(mgs) = NR(0.0);  // 15782
      efw(mgs) = NR(0.0);  // 15783
      ehlw(mgs) = NR(0.0);  // 15784
      err(mgs) = NR(0.0);  // 15787
      esr(mgs) = NR(0.0);  // 15788
      il2(mgs) = 0;  // 15789
      il3(mgs) = 0;  // 15790
      ehr(mgs) = NR(0.0);  // 15791
      ehlr(mgs) = NR(0.0);  // 15792
      eri(mgs) = NR(0.0);  // 15795
      esi(mgs) = NR(0.0);  // 15796
      ehi(mgs) = NR(0.0);  // 15797
      ehis(mgs) = NR(0.0);  // 15798
      ehli(mgs) = NR(0.0);  // 15799
      ehlis(mgs) = NR(0.0);  // 15800
      ers(mgs) = NR(0.0);  // 15803
      ess(mgs) = NR(0.0);  // 15804
      ehs(mgs) = NR(0.0);  // 15805
      ehsfac(mgs) = NR(1.0);  // 15806
      ehls(mgs) = NR(0.0);  // 15807
      ehscnv(mgs) = NR(0.0);  // 15808
      eiw(mgs) = NR(0.0);  // 15811
      eii(mgs) = NR(0.0);  // 15812
      ehsclsn(mgs) = NR(0.0);  // 15813
      ehiclsn(mgs) = NR(0.0);  // 15814
      ehlsclsn(mgs) = NR(0.0);  // 15815
      ehliclsn(mgs) = NR(0.0);  // 15816
      esiclsn(mgs) = NR(0.0);  // 15817
      icwr(mgs) = 1;  // 15840
      if ((qx(mgs, P.lc) > P.qxmin(P.lc))) {  // 15841
        cwrad = (NR(0.5) * xdia(mgs, P.lc, 1));  // 15842
        for (il = 1; il <= 8; ++il) {  // 15843
          if ((cwrad >= (NR(1.0e-6) * cwr(il, 1)))) { icwr(mgs) = il; }  // 15844
        }  // 15845
      }  // 15846
      irwr(mgs) = 1;  // 15849
      if ((qx(mgs, P.lr) > P.qxmin(P.lr))) {  // 15850
        rwrad = (NR(0.5) * xdia(mgs, P.lr, 3));  // 15851
        for (il = 1; il <= 6; ++il) {  // 15852
          if ((rwrad >= (NR(1.0e-6) * grad(il, 1)))) { irwr(mgs) = il; }  // 15853
        }  // 15854
      }  // 15855
      igwr(mgs) = 1;  // 15858
      if ((qx(mgs, P.lh) > P.qxmin(P.lh))) {  // 15862
        rwrad = (NR(0.5) * xdia(mgs, P.lh, 3));  // 15863
        for (il = 1; il <= 6; ++il) {  // 15864
          if ((rwrad >= (NR(1.0e-6) * grad(il, 1)))) { igwr(mgs) = il; }  // 15865
        }  // 15866
      }  // 15867
      if ((P.lhl > 1)) {  // 15870
        ihlr(mgs) = 1;  // 15871
        if ((qx(mgs, P.lhl) > P.qxmin(P.lhl))) {  // 15872
          rwrad = (NR(0.5) * xdia(mgs, P.lhl, 3));  // 15873
          for (il = 1; il <= 6; ++il) {  // 15874
            if ((rwrad >= (NR(1.0e-6) * grad(il, 1)))) { ihlr(mgs) = il; }  // 15875
          }  // 15876
        }  // 15877
      }  // 15878
      if ((qx(mgs, P.li) > P.qxmin(P.li))) {  // 15885
        eii(mgs) = exp((NR(0.025) * c_min(temcg(mgs), NR(0.0))));  // 15893
        if ((temg(mgs) > NR(273.15))) { eii(mgs) = NR(1.0); }  // 15895
      }  // 15896
      eiw(mgs) = NR(0.0);  // 15903
      if (((qx(mgs, P.li) > P.qxmin(P.li)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 15904
        if (((xdia(mgs, P.lc, 1) > P.ewi_dcmin) && (xdia(mgs, P.li, 1) > P.ewi_dimin))) {  // 15907
          eiw(mgs) = P.eiw0;  // 15910
        }  // 15911
        if ((temg(mgs) >= NR(273.15))) { eiw(mgs) = NR(0.0); }  // 15912
      }  // 15913
      if (((qx(mgs, P.lr) > P.qxmin(P.lr)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 15921
        if ((P.lnr > 1)) {  // 15923
          erw(mgs) = NR(1.0);  // 15924
        } else {  // 15926
          ic = icwr(mgs);  // 15937
          icp1 = c_min(8, (ic + 1));  // 15938
          ir = irwr(mgs);  // 15939
          irp1 = c_min(6, (ir + 1));  // 15940
          cwrad = (NR(0.5) * xdia(mgs, P.lc, 3));  // 15941
          rwrad = (NR(0.5) * xdia(mgs, P.lr, 3));  // 15942
          slope1 = (((ew(icp1, ir) - ew(ic, ir))) * cwr(ic, 2));  // 15944
          slope2 = (((ew(icp1, irp1) - ew(ic, irp1))) * cwr(ic, 2));  // 15945
          x1 = (ew(ic, ir) + (slope1 * c_max(NR(0.0), ((cwrad - cwr(ic, 1))))));  // 15949
          x2 = (ew(icp1, ir) + (slope2 * c_max(NR(0.0), ((cwrad - cwr(ic, 1))))));  // 15950
          slope1 = (((x2 - x1)) * grad(ir, 2));  // 15952
          erw(mgs) = c_max(NR(0.0), (x1 + (slope1 * c_max(NR(0.0), ((rwrad - grad(ir, 1)))))));  // 15954
          erw(mgs) = c_max(NR(0.0), erw(mgs));  // 15959
          if ((rwrad < NR(50.0e-6))) {  // 15960
            erw(mgs) = NR(0.0);  // 15961
          } else if ((rwrad < NR(100.0e-6))) {  // 15962
            erw(mgs) = ((erw(mgs) * ((rwrad - NR(50.0e-6)))) / NR(50.0e-6));  // 15963
          }  // 15964
        }  // 15966
      }  // 15967
      if ((cx(mgs, P.lc) <= NR(0.0))) { erw(mgs) = NR(0.0); }  // 15968
      if (((qx(mgs, P.lr) > P.qxmin(P.lr)) && (qx(mgs, P.lr) > P.qxmin(P.lr)))) {  // 15970
        err(mgs) = NR(1.0);  // 15971
      }  // 15972
      if (((qx(mgs, P.lr) > P.qxmin(P.lr)) && (qx(mgs, P.ls) > P.qxmin(P.ls)))) {  // 15974
        ers(mgs) = NR(1.0);  // 15975
      }  // 15976
      if (((qx(mgs, P.lr) > P.qxmin(P.lr)) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 15978
        eri(mgs) = P.eri0;  // 15981
        if ((xdia(mgs, P.li, 3) < P.eri_cimin)) { eri(mgs) = NR(0.0); }  // 15988
      }  // 15989
      esw(mgs) = NR(0.0);  // 15999
      if (((qx(mgs, P.ls) > P.qxmin(P.ls)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 16000
        esw(mgs) = NR(0.5);  // 16001
        if (((xdia(mgs, P.lc, 1) > NR(15.0e-6)) && (xdia(mgs, P.ls, 1) > NR(100.0e-6)))) {  // 16002
          esw(mgs) = NR(0.5);  // 16003
        } else if ((xdia(mgs, P.ls, 1) >= NR(500.0e-6))) {  // 16004
          esw(mgs) = c_min(NR(0.5), (NR(0.05) + ((((NR(0.8) - NR(0.05))) / (NR(40.0e-6))) * xdia(mgs, P.lc, 1))));  // 16005
        }  // 16006
      }  // 16007
      if ((((qx(mgs, P.ls) > P.qxmin(P.ls)) && (qx(mgs, P.lr) > P.qxmin(P.lr))) && (temg(mgs) < (P.tfr - NR(1.0))))) {  // 16009
        esr(mgs) = (exp((-(NR(hexd(0x3D3203AF9EE75617ull)) / xv(mgs, P.lr)))) * exp((-(NR(40.0e-6) / xdia(mgs, P.ls, 1)))));  // 16012
        if (((qx(mgs, P.ls) < NR(1.0e-4)) && (qx(mgs, P.lr) < NR(1.0e-4)))) { il2(mgs) = 1; }  // 16013
      }  // 16014
      if (((((P.ipconc < 3) && (temg(mgs) < P.tfr)) && (qx(mgs, P.lr) > P.qxmin(P.lr))) && (qx(mgs, P.lr) < NR(1.0e-4)))) {  // 16016
        il3(mgs) = 1;  // 16017
      }  // 16018
      if ((temcg(mgs) < NR(0.0))) {  // 16021
        if (((P.ipconc < 4) || (temcg(mgs) < P.esstem1))) {  // 16023
          ess(mgs) = NR(0.0);  // 16024
        } else {  // 16028
          fac = fabs(P.ess0);  // 16030
          if (((temcg(mgs) > P.esstem1) && (temcg(mgs) < P.esstem2))) {  // 16057
            ess(mgs) = (((fac * exp((P.ess1 * (P.esstem2)))) * ((temcg(mgs) - P.esstem1))) / ((P.esstem2 - P.esstem1)));  // 16058
          } else if ((temcg(mgs) >= P.esstem2)) {  // 16059
            ess(mgs) = (fac * exp((P.ess1 * c_min(temcg(mgs), NR(0.0)))));  // 16060
          }  // 16061
        }  // 16063
      }  // 16064
      if (((qx(mgs, P.ls) > P.qxmin(P.ls)) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 16066
        esiclsn(mgs) = esi_collsn;  // 16067
        esi(mgs) = (P.esi0 * exp((NR(0.1) * c_min(temcg(mgs), NR(0.0)))));  // 16072
        esi(mgs) = c_min(NR(0.1), esi(mgs));  // 16073
        if ((P.ipconc <= 3)) {  // 16075
          esi(mgs) = exp((NR(0.025) * c_min(temcg(mgs), NR(0.0))));  // 16076
        }  // 16079
        if ((temg(mgs) > NR(273.15))) { esi(mgs) = NR(0.0); }  // 16083
      }  // 16084
      xmascw(mgs) = xmas(mgs, P.lc);  // 16093
      if (((qx(mgs, P.lh) > P.qxmin(P.lh)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 16094
        ehw(mgs) = NR(1.0);  // 16095
        cwrad = (NR(0.5) * xdia(mgs, P.lc, 1));  // 16099
        ehw(mgs) = c_min(P.ehw0, (P.ewfac * c_min(((aradcw + (cwrad * ((bradcw + (cwrad * ((cradcw + (cwrad * (dradcw)))))))))), NR(1.0))));  // 16100
        if ((xdia(mgs, P.lc, 1) < NR(2.4e-06))) { ehw(mgs) = NR(0.0); }  // 16144
        ehw(mgs) = c_min(P.ehw0, ehw(mgs));  // 16146
      }  // 16152
      if (((qx(mgs, P.lh) > P.qxmin(P.lh)) && (qx(mgs, P.lr) > P.qxmin(P.lr)))) {  // 16154
        ehr(mgs) = (exp((-((NR(40.0e-6)) / xdia(mgs, P.lr, 3)))) * exp((-(NR(40.0e-6) / xdia(mgs, P.lh, 3)))));  // 16159
        ehr(mgs) = c_min(P.ehr0, ehr(mgs));  // 16160
      }  // 16161
      if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 16163
        if ((P.ipconc >= 4)) {  // 16164
          ehscnv(mgs) = (P.ehs0 * exp((P.ehs1 * c_min(temcg(mgs), NR(0.0)))));  // 16165
        } else {  // 16166
          ehscnv(mgs) = exp((NR(0.09) * c_min(temcg(mgs), NR(0.0))));  // 16167
        }  // 16168
        if (((qx(mgs, P.lh) > P.qxmin(P.lh)) && (qx(mgs, P.lc) >= P.qxmin(P.lc)))) {  // 16170
          ehsclsn(mgs) = ehs_collsn;  // 16174
          if ((xdia(mgs, P.ls, 3) < NR(40.0e-6))) {  // 16175
            ehsclsn(mgs) = NR(0.0);  // 16176
          } else if ((xdia(mgs, P.ls, 3) < NR(150.0e-6))) {  // 16177
            ehsclsn(mgs) = ((ehs_collsn * ((xdia(mgs, P.ls, 3) - NR(40.0e-6)))) / ((NR(150.0e-6) - NR(40.0e-6))));  // 16178
          } else {  // 16179
            ehsclsn(mgs) = ehs_collsn;  // 16180
          }  // 16181
          ehs(mgs) = (ehscnv(mgs) * c_min(NR(1.0), (c_max(NR(0.0), (xdn(mgs, P.lh) - NR(300.0))) / NR(300.0))));  // 16183
          ehs(mgs) = c_min(ehs(mgs), P.ehsmax);  // 16185
        }  // 16186
      }  // 16187
      if (((qx(mgs, P.lh) > P.qxmin(P.lh)) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 16189
        ehiclsn(mgs) = ehi_collsn;  // 16190
        ehi(mgs) = (P.eii0 * exp((P.eii1 * c_min(temcg(mgs), NR(0.0)))));  // 16191
        ehi(mgs) = c_min(P.ehimax, c_max(ehi(mgs), P.ehimin));  // 16192
      }  // 16194
      if ((P.lhl > 1)) {  // 16211
        if (((qx(mgs, P.lhl) > P.qxmin(P.lhl)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 16213
          ehlw(mgs) = P.ehlw0;  // 16216
          cwrad = (NR(0.5) * xdia(mgs, P.lc, 1));  // 16220
          ehlw(mgs) = c_min(P.ehlw0, (P.ewfac * c_min(((aradcw + (cwrad * ((bradcw + (cwrad * ((cradcw + (cwrad * (dradcw)))))))))), NR(1.0))));  // 16221
          if ((xdia(mgs, P.lc, 1) < NR(2.4e-06))) { ehlw(mgs) = NR(0.0); }  // 16259
          ehlw(mgs) = c_min(P.ehlw0, ehlw(mgs));  // 16260
        }  // 16266
        if (((qx(mgs, P.lhl) > P.qxmin(P.lhl)) && (qx(mgs, P.lr) > P.qxmin(P.lr)))) {  // 16268
          ehlr(mgs) = NR(1.0);  // 16271
          ehlr(mgs) = c_min(P.ehlr0, ehlr(mgs));  // 16272
        }  // 16273
        if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 16275
          if ((qx(mgs, P.lhl) > P.qxmin(P.lhl))) {  // 16276
            ehlsclsn(mgs) = ehls_collsn;  // 16277
            ehls(mgs) = ehscnv(mgs);  // 16278
            ehls(mgs) = c_min(ehls(mgs), P.ehsmax);  // 16279
          }  // 16280
        }  // 16281
        if (((qx(mgs, P.lhl) > P.qxmin(P.lhl)) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 16283
          ehliclsn(mgs) = ehli_collsn;  // 16284
          ehli(mgs) = (P.eii0hl * exp((P.eii1hl * c_min(temcg(mgs), NR(0.0)))));  // 16285
          ehli(mgs) = c_min(P.ehimax, c_max(ehli(mgs), P.ehimin));  // 16286
          if (((temg(mgs) > NR(273.15)) || ((qx(mgs, P.lc) < P.qxmin(P.lc))))) { ehli(mgs) = NR(0.0); }  // 16287
        }  // 16288
      }  // 16300
    }  // 16302
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16310
      xplate(mgs) = NR(0.0);  // 16312
      xcolmn(mgs) = NR(1.0);  // 16313
    }  // 16335
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16347
      qracw(mgs) = NR(0.0);  // 16348
      if (((qx(mgs, P.lr) > P.qxmin(P.lr)) && (erw(mgs) > NR(0.0)))) {  // 16349
        if ((P.ipconc < 3)) {  // 16350
          if (((erw(mgs) > NR(0.0)) && (qx(mgs, P.lr) > NR(1.0e-7)))) {  // 16351
            vt = (((P.ar * (pow(xdia(mgs, P.lc, 1), P.br)))) * rhovt(mgs));  // 16352
            qracw(mgs) = (((((((NR(0.25)) * P.pi) * erw(mgs)) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * cx(mgs, P.lr)) * c_max(NR(0.0), (vtxbar(mgs, P.lr, 1) - vt))) * ((((gf3 * xdia(mgs, P.lr, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lr, 1)) * xdia(mgs, P.lc, 1))) + (gf1 * xdia(mgs, P.lc, 2)))));  // 16353
          }  // 16365
        } else {  // 16366
          rwrad = (NR(0.5) * xdia(mgs, P.lr, 3));  // 16369
          if ((rwrad > rh(mgs))) {  // 16370
            if ((rwrad > rwradmn)) {  // 16371
              qracw(mgs) = ((((((erw(mgs) * aa2) * cx(mgs, P.lr)) * cx(mgs, P.lc)) * xmas(mgs, P.lc)) * ((((((alpha(mgs, P.lc) + NR(2.0))) * xv(mgs, P.lc)) / ((alpha(mgs, P.lc) + NR(1.0)))) + xv(mgs, P.lr)))) / rho0(mgs));  // 16374
            } else {  // 16376
              qracw(mgs) = (((aa1 * cx(mgs, P.lr)) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * (((((((alpha(mgs, P.lc) + NR(3.0))) * ((alpha(mgs, P.lc) + NR(2.0)))) * ipow<2>(xv(mgs, P.lc))) / ipow<2>(((alpha(mgs, P.lc) + NR(1.0))))) + ((((((alpha(mgs, P.lr) + NR(6.0))) * ((alpha(mgs, P.lr) + NR(5.0)))) * ((alpha(mgs, P.lr) + NR(4.0)))) * ipow<2>(xv(mgs, P.lr))) / (((((alpha(mgs, P.lr) + NR(3.0))) * ((alpha(mgs, P.lr) + NR(2.0)))) * ((alpha(mgs, P.lr) + NR(1.0)))))))));  // 16393
            }  // 16400
          }  // 16401
        }  // 16403
        qracw(mgs) = c_min(qracw(mgs), qcmxd(mgs));  // 16405
      }  // 16406
    }  // 16407
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16409
      qraci(mgs) = NR(0.0);  // 16410
      craci(mgs) = NR(0.0);  // 16411
      qracs(mgs) = NR(0.0);  // 16412
      if ((((eri(mgs) > NR(0.0)) && (P.iacr >= 1)) && (xdia(mgs, P.lr, 3) > (NR(2.0) * rwradmn)))) {  // 16413
        if ((P.ipconc >= 3)) {  // 16414
          tmp = ((((eri(mgs) * aa2) * cx(mgs, P.lr)) * cx(mgs, P.li)) * ((((((P.cinu + NR(2.0))) * xv(mgs, P.li)) / ((P.cinu + NR(1.0)))) + xv(mgs, P.lr))));  // 16416
          qraci(mgs) = c_min(qxmxd(mgs, P.li), ((tmp * xmas(mgs, P.li)) * rhoinv(mgs)));  // 16419
          craci(mgs) = c_min(cxmxd(mgs, P.li), tmp);  // 16420
        } else {  // 16442
          qraci(mgs) = c_min((((((((NR(0.25)) * P.pi) * eri(mgs)) * qx(mgs, P.li)) * cx(mgs, P.lr)) * fabs((vtxbar(mgs, P.lr, 1) - vtxbar(mgs, P.li, 1)))) * ((((gf3 * xdia(mgs, P.lr, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lr, 1)) * xdia(mgs, P.li, 1))) + (gf1 * xdia(mgs, P.li, 2))))), qimxd(mgs));  // 16443
        }  // 16451
        if ((temg(mgs) > NR(268.15))) {  // 16452
          qraci(mgs) = NR(0.0);  // 16453
        }  // 16454
      }  // 16455
    }  // 16456
    if ((P.ipconc < 3)) {  // 16458
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16459
        qracs(mgs) = NR(0.0);  // 16460
        if (((ers(mgs) > NR(0.0)) && (P.ipconc < 3))) {  // 16461
          vt = vtxbar(mgs, P.ls, 1);  // 16465
          qracs(mgs) = c_min((((((((((NR(0.25)) * P.pi) / gf4)) * ers(mgs)) * qx(mgs, P.ls)) * cx(mgs, P.lr)) * fabs((vtxbar(mgs, P.lr, 1) - vt))) * (((((gf6 * gf1) * xdia(mgs, P.ls, 2)) + ((((NR(2.0) * gf5) * gf2) * xdia(mgs, P.ls, 1)) * xdia(mgs, P.lr, 1))) + ((gf4 * gf3) * xdia(mgs, P.lr, 2))))), qsmxd(mgs));  // 16467
        }  // 16475
      }  // 16476
    }  // 16477
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16483
      qsacw(mgs) = NR(0.0);  // 16484
      csacw(mgs) = NR(0.0);  // 16485
      vsacw(mgs) = NR(0.0);  // 16486
      if ((esw(mgs) > NR(0.0))) {  // 16487
        if ((P.ipconc >= 4)) {  // 16489
          tmp = (((((NR(1.0) * rvt) * aa2) * cx(mgs, P.ls)) * cx(mgs, P.lc)) * ((((((alpha(mgs, P.lc) + NR(2.0))) * xv(mgs, P.lc)) / ((alpha(mgs, P.lc) + NR(1.0)))) + xv(mgs, P.ls))));  // 16495
          qsacw(mgs) = c_min(qxmxd(mgs, P.lc), ((tmp * xmas(mgs, P.lc)) * rhoinv(mgs)));  // 16498
          csacw(mgs) = c_min(cxmxd(mgs, P.lc), tmp);  // 16499
          if ((P.lvol(P.ls) > 1)) {  // 16501
            if ((temg(mgs) < NR(273.15))) {  // 16502
              rimdn(mgs, P.ls) = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lc, 1))) * (((NR(0.60)) * vtxbar(mgs, P.ls, 1)))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 16503
              rimdn(mgs, P.ls) = c_min(c_max(P.rimc3, rimdn(mgs, P.ls)), P.rimc4);  // 16506
            } else {  // 16507
              rimdn(mgs, P.ls) = NR(1000.0);  // 16508
            }  // 16509
            vsacw(mgs) = ((rho0(mgs) * qsacw(mgs)) / rimdn(mgs, P.ls));  // 16511
          }  // 16513
        } else {  // 16518
          vt = fabs((vtxbar(mgs, P.ls, 1) - vtxbar(mgs, P.lc, 1)));  // 16528
          qsacw(mgs) = ((((((NR(0.25) * P.pi) * esw(mgs)) * cx(mgs, P.ls)) * qx(mgs, P.lc)) * vt) * ((((P.da0(P.ls) * ipow<2>(xdia(mgs, P.ls, 3))) + ((P.dab1(P.ls, P.lc) * xdia(mgs, P.ls, 3)) * xdia(mgs, P.lc, 3))) + (da1lc(mgs) * ipow<2>(xdia(mgs, P.lc, 3))))));  // 16530
          qsacw(mgs) = c_min(qsacw(mgs), qxmxd(mgs, P.ls));  // 16534
          csacw(mgs) = ((rho0(mgs) * qsacw(mgs)) / xmas(mgs, P.lc));  // 16535
        }  // 16536
      }  // 16537
    }  // 16538
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16541
      qsaci(mgs) = NR(0.0);  // 16542
      csaci(mgs) = NR(0.0);  // 16543
      csaci0(mgs) = NR(0.0);  // 16544
      if ((P.ipconc >= 4)) {  // 16545
        if (((esi(mgs) > NR(0.0)) || (((P.ipelec > 0) && (esiclsn(mgs) > NR(0.0)))))) {  // 16546
          tmp = (((((esiclsn(mgs) * rvt) * aa2) * cx(mgs, P.ls)) * cx(mgs, P.li)) * ((((((P.cinu + NR(2.0))) * xv(mgs, P.li)) / ((P.cinu + NR(1.0)))) + xv(mgs, P.ls))));  // 16550
          qsaci(mgs) = c_min(qxmxd(mgs, P.li), (((esi(mgs) * tmp) * xmas(mgs, P.li)) * rhoinv(mgs)));  // 16553
          csaci0(mgs) = tmp;  // 16554
          csaci(mgs) = c_min(cxmxd(mgs, P.li), (esi(mgs) * tmp));  // 16555
        }  // 16565
      } else {  // 16566
        if ((esi(mgs) > NR(0.0))) {  // 16567
          qsaci(mgs) = c_min(((((((((NR(0.25)) * P.pi)) * esi(mgs)) * qx(mgs, P.li)) * cx(mgs, P.ls)) * fabs((vtxbar(mgs, P.ls, 1) - vtxbar(mgs, P.li, 1)))) * ((((gf3 * xdia(mgs, P.ls, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.ls, 1)) * xdia(mgs, P.li, 1))) + (gf1 * xdia(mgs, P.li, 2))))), qimxd(mgs));  // 16568
        }  // 16576
      }  // 16577
    }  // 16578
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16582
      qsacr(mgs) = NR(0.0);  // 16583
      qsacrs(mgs) = NR(0.0);  // 16584
      csacr(mgs) = NR(0.0);  // 16585
      if ((esr(mgs) > NR(0.0))) {  // 16586
        if ((P.ipconc >= 3)) {  // 16587
        } else {  // 16598
          vt = vtxbar(mgs, P.ls, 1);  // 16602
          qsacr(mgs) = c_min((((((((((NR(0.25)) * P.pi) / gf4)) * esr(mgs)) * qx(mgs, P.lr)) * cx(mgs, P.ls)) * fabs((vtxbar(mgs, P.lr, 1) - vt))) * (((((gf6 * gf1) * xdia(mgs, P.lr, 2)) + ((((NR(2.0) * gf5) * gf2) * xdia(mgs, P.lr, 1)) * xdia(mgs, P.ls, 1))) + ((gf4 * gf3) * xdia(mgs, P.ls, 2))))), qrmxd(mgs));  // 16605
        }  // 16613
      }  // 16614
    }  // 16615
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16622
      qhacw(mgs) = NR(0.0);  // 16623
      qhacwmlr(mgs) = NR(0.0);  // 16624
      rarx(mgs, P.lh) = NR(0.0);  // 16625
      vhacw(mgs) = NR(0.0);  // 16626
      vhsoak(mgs) = NR(0.0);  // 16627
      zhacw(mgs) = NR(0.0);  // 16628
      if ((ehw(mgs) > NR(0.0))) {  // 16636
        if ((P.ipconc >= 2)) {  // 16638
          vt = fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.lc, 1)));  // 16648
          qhacw(mgs) = ((((((NR(0.25) * P.pi) * ehw(mgs)) * cx(mgs, P.lh)) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab1lh(mgs, P.lh, P.lc) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.lc, 3))) + (da1lc(mgs) * ipow<2>(xdia(mgs, P.lc, 3))))));  // 16650
          qhacw(mgs) = c_min(qhacw(mgs), ((NR(0.5) * qx(mgs, P.lc)) * dtpinv));  // 16656
        } else {  // 16669
          qhacw(mgs) = c_min(((((((((NR(0.25)) * P.pi)) * ehw(mgs)) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.lc, 1)))) * ((((gf3 * xdia(mgs, P.lh, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lh, 1)) * xdia(mgs, P.lc, 1))) + (gf1 * xdia(mgs, P.lc, 2))))), ((NR(0.5) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * dtpinv));  // 16670
        }  // 16689
        qhacwmlr(mgs) = qhacw(mgs);  // 16691
        if (((P.lvol(P.lh) > 1) || (P.lhl > 1))) {  // 16696
          if ((temg(mgs) < NR(273.15))) {  // 16698
            vt = (((((NR(1.0) - P.rimdenvwgt)) * vtxbar(mgs, P.lh, 1)) + (P.rimdenvwgt * vtxbar(mgs, P.lh, 2))));  // 16700
            rimdn(mgs, P.lh) = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lc, 1))) * (((NR(0.60)) * vt))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 16702
            rimdn(mgs, P.lh) = c_min(c_max(P.rimc3, rimdn(mgs, P.lh)), P.rimc4);  // 16706
          } else {  // 16739
            rimdn(mgs, P.lh) = NR(1000.0);  // 16740
          }  // 16741
          if ((P.lvol(P.lh) > 1)) { vhacw(mgs) = ((rho0(mgs) * qhacw(mgs)) / rimdn(mgs, P.lh)); }  // 16743
        }  // 16745
      }  // 16752
    }  // 16753
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16756
      qhaci(mgs) = NR(0.0);  // 16757
      qhaci0(mgs) = NR(0.0);  // 16758
      if ((ehi(mgs) > NR(0.0))) {  // 16759
        if ((P.ipconc >= 5)) {  // 16760
          vt = sqrt((ipow<2>(((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.li, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lh, 1)) * vtxbar(mgs, P.li, 1))));  // 16762
          qhaci0(mgs) = ((((((NR(0.25) * P.pi) * ehiclsn(mgs)) * cx(mgs, P.lh)) * qx(mgs, P.li)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab1lh(mgs, P.lh, P.li) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.li, 3))) + (P.da1(P.li) * ipow<2>(xdia(mgs, P.li, 3))))));  // 16765
          qhaci(mgs) = c_min((ehi(mgs) * qhaci0(mgs)), qimxd(mgs));  // 16769
        } else {  // 16770
          qhaci(mgs) = c_min((((((((((NR(0.25)) * P.pi)) * ehi(mgs)) * ehiclsn(mgs)) * qx(mgs, P.li)) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.li, 1)))) * ((((gf3 * xdia(mgs, P.lh, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lh, 1)) * xdia(mgs, P.li, 1))) + (gf1 * xdia(mgs, P.li, 2))))), qimxd(mgs));  // 16771
        }  // 16779
      }  // 16780
    }  // 16781
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16786
      qhacs(mgs) = NR(0.0);  // 16787
      qhacs0(mgs) = NR(0.0);  // 16788
      if ((ehs(mgs) > NR(0.0))) {  // 16789
        if ((P.ipconc >= 5)) {  // 16790
          vt = sqrt((ipow<2>(((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.ls, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lh, 1)) * vtxbar(mgs, P.ls, 1))));  // 16792
          qhacs0(mgs) = ((((((NR(0.25) * P.pi) * ehsclsn(mgs)) * cx(mgs, P.lh)) * qx(mgs, P.ls)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab1lh(mgs, P.lh, P.ls) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.ls, 3))) + (P.da1(P.ls) * ipow<2>(xdia(mgs, P.ls, 3))))));  // 16795
          qhacs(mgs) = c_min((ehs(mgs) * qhacs0(mgs)), qsmxd(mgs));  // 16800
        } else {  // 16802
          qhacs(mgs) = c_min(((((((((((NR(0.25)) * P.pi) / gf4)) * ehs(mgs)) * ehsclsn(mgs)) * qx(mgs, P.ls)) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.ls, 1)))) * (((((gf6 * gf1) * xdia(mgs, P.ls, 2)) + ((((NR(2.0) * gf5) * gf2) * xdia(mgs, P.ls, 1)) * xdia(mgs, P.lh, 1))) + ((gf4 * gf3) * xdia(mgs, P.lh, 2))))), qsmxd(mgs));  // 16803
        }  // 16811
      }  // 16812
    }  // 16813
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16815
      qhacr(mgs) = NR(0.0);  // 16816
      qhacrmlr(mgs) = NR(0.0);  // 16817
      vhacr(mgs) = NR(0.0);  // 16818
      chacr(mgs) = NR(0.0);  // 16819
      zhacr(mgs) = NR(0.0);  // 16820
      if ((temg(mgs) > P.tfr)) { raindn(mgs, P.lh) = NR(1000.0); }  // 16821
      if ((ehr(mgs) > NR(0.0))) {  // 16823
        if ((P.ipconc >= 3)) {  // 16824
          vt = sqrt((ipow<2>(((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.lr, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lh, 1)) * vtxbar(mgs, P.lr, 1))));  // 16825
          qhacr(mgs) = ((((((NR(0.25) * P.pi) * ehr(mgs)) * cx(mgs, P.lh)) * qx(mgs, P.lr)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab1lh(mgs, P.lh, P.lr) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.lr, 3))) + (da1lr(mgs) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 16833
          qhacr(mgs) = c_min(qhacr(mgs), qxmxd(mgs, P.lr));  // 16843
          qhacrmlr(mgs) = qhacr(mgs);  // 16845
          if (((temg(mgs) > P.tfr) && (P.iehr0c == 0))) {  // 16847
            qhacr(mgs) = NR(0.0);  // 16848
          } else {  // 16854
            chacr(mgs) = ((((((NR(0.25) * P.pi) * ehr(mgs)) * cx(mgs, P.lh)) * cx(mgs, P.lr)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab0lh(mgs, P.lh, P.lr) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.lr, 3))) + (da0lr(mgs) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 16863
            chacr(mgs) = c_min(chacr(mgs), crmxd(mgs));  // 16871
          }  // 16884
        } else {  // 16886
          vt = vtxbar(mgs, P.lh, 1);  // 16890
          qhacr(mgs) = c_min((((((((((NR(0.25)) * P.pi) / gf4)) * ehr(mgs)) * qx(mgs, P.lr)) * cx(mgs, P.lh)) * fabs((vt - vtxbar(mgs, P.lr, 1)))) * (((((gf6 * gf1) * xdia(mgs, P.lr, 2)) + ((((NR(2.0) * gf5) * gf2) * xdia(mgs, P.lr, 1)) * xdia(mgs, P.lh, 1))) + ((gf4 * gf3) * xdia(mgs, P.lh, 2))))), qrmxd(mgs));  // 16893
          if ((temg(mgs) > P.tfr)) {  // 16902
            qhacrmlr(mgs) = qhacr(mgs);  // 16903
            qhacr(mgs) = NR(0.0);  // 16904
          }  // 16905
        }  // 16907
        if (((P.lvol(P.lh) > 1) || (P.lhl > 1))) {  // 16908
          if ((temg(mgs) < NR(273.15))) {  // 16910
            raindn(mgs, P.lh) = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lr, 3))) * (((NR(0.60)) * vt))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 16911
            raindn(mgs, P.lh) = c_min(c_max(P.rimc3, rimdn(mgs, P.lh)), P.rimc4);  // 16915
          } else {  // 16916
            raindn(mgs, P.lh) = NR(1000.0);  // 16917
          }  // 16918
          if ((P.lvol(P.lh) > 1)) { vhacr(mgs) = ((rho0(mgs) * qhacr(mgs)) / raindn(mgs, P.lh)); }  // 16920
        }  // 16921
      }  // 16922
    }  // 16923
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 16930
      qhlacw(mgs) = NR(0.0);  // 16931
      qhlacwmlr(mgs) = NR(0.0);  // 16932
      vhlacw(mgs) = NR(0.0);  // 16933
      vhlsoak(mgs) = NR(0.0);  // 16934
      if (((P.lhl > 1) && true)) {  // 16935
        vtmax = ((gz(igs(mgs), jgs, kgs(mgs)) * dtpinv));  // 16936
        vtxbar(mgs, P.lhl, 1) = c_min(vtmax, vtxbar(mgs, P.lhl, 1));  // 16937
        vtxbar(mgs, P.lhl, 2) = c_min(vtmax, vtxbar(mgs, P.lhl, 2));  // 16938
        vtxbar(mgs, P.lhl, 3) = c_min(vtmax, vtxbar(mgs, P.lhl, 3));  // 16939
      }  // 16940
      if ((P.lhl > 0)) {  // 16942
        rarx(mgs, P.lhl) = NR(0.0);  // 16943
      }  // 16944
      if (((P.lhl > 1) && (ehlw(mgs) > NR(0.0)))) {  // 16946
        vt = fabs((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.lc, 1)));  // 16951
        qhlacw(mgs) = ((((((NR(0.25) * P.pi) * ehlw(mgs)) * cx(mgs, P.lhl)) * ((qx(mgs, P.lc) - qcwresv(mgs)))) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((dab1lh(mgs, P.lhl, P.lc) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.lc, 3))) + (da1lc(mgs) * ipow<2>(xdia(mgs, P.lc, 3))))));  // 16953
        qhlacw(mgs) = c_min(qhlacw(mgs), ((NR(0.5) * qx(mgs, P.lc)) * dtpinv));  // 16959
        qhlacwmlr(mgs) = qhlacw(mgs);  // 16961
        if ((P.lvol(P.lhl) > 1)) {  // 16966
          if ((temg(mgs) < NR(273.15))) {  // 16968
            rimdn(mgs, P.lhl) = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lc, 1))) * (((NR(0.60)) * (((((NR(1.0) - P.rimdenvwgt)) * vtxbar(mgs, P.lhl, 1)) + (P.rimdenvwgt * vtxbar(mgs, P.lhl, 2))))))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 16970
            rimdn(mgs, P.lhl) = c_min(c_max(c_max(P.hldnmn, P.rimc3), rimdn(mgs, P.lhl)), P.rimc4);  // 16973
          } else {  // 16996
            rimdn(mgs, P.lhl) = NR(1000.0);  // 16997
          }  // 16998
          vhlacw(mgs) = ((rho0(mgs) * qhlacw(mgs)) / rimdn(mgs, P.lhl));  // 17000
        }  // 17002
      }  // 17010
    }  // 17011
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlaci(i_0) = NR(0.0);  // 17013
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlaci0(i_0) = NR(0.0);  // 17014
    if ((P.lhl > 1)) {  // 17015
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17016
        if ((ehli(mgs) > NR(0.0))) {  // 17017
          if ((P.ipconc >= 5)) {  // 17018
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.li, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lhl, 1)) * vtxbar(mgs, P.li, 1))));  // 17020
            qhlaci0(mgs) = ((((((NR(0.25) * P.pi) * ehliclsn(mgs)) * cx(mgs, P.lhl)) * qx(mgs, P.li)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((dab1lh(mgs, P.lhl, P.li) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.li, 3))) + (P.da1(P.li) * ipow<2>(xdia(mgs, P.li, 3))))));  // 17023
            qhlaci(mgs) = c_min((ehli(mgs) * qhlaci0(mgs)), qimxd(mgs));  // 17028
          }  // 17029
        }  // 17030
      }  // 17031
    }  // 17032
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlacis(i_0) = NR(0.0);  // 17034
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlacis0(i_0) = NR(0.0);  // 17035
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlacs(i_0) = NR(0.0);  // 17037
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlacs0(i_0) = NR(0.0);  // 17038
    if ((P.lhl > 1)) {  // 17039
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17040
        if ((ehls(mgs) > NR(0.0))) {  // 17041
          if ((P.ipconc >= 5)) {  // 17042
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.ls, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lhl, 1)) * vtxbar(mgs, P.ls, 1))));  // 17044
            qhlacs0(mgs) = ((((((NR(0.25) * P.pi) * ehlsclsn(mgs)) * cx(mgs, P.lhl)) * qx(mgs, P.ls)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((dab1lh(mgs, P.lhl, P.ls) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.ls, 3))) + (P.da1(P.ls) * ipow<2>(xdia(mgs, P.ls, 3))))));  // 17047
            qhlacs(mgs) = c_min((ehls(mgs) * qhlacs0(mgs)), qsmxd(mgs));  // 17052
          }  // 17053
        }  // 17054
      }  // 17055
    }  // 17056
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17059
      qhlacr(mgs) = NR(0.0);  // 17060
      qhlacrmlr(mgs) = NR(0.0);  // 17061
      chlacr(mgs) = NR(0.0);  // 17062
      vhlacr(mgs) = NR(0.0);  // 17063
      if (((P.lhl > 1) && (temg(mgs) > P.tfr))) { raindn(mgs, P.lhl) = NR(1000.0); }  // 17064
      if (((P.lhl > 1) && (ehlr(mgs) > NR(0.0)))) {  // 17066
        if ((P.ipconc >= 3)) {  // 17067
          vt = sqrt((ipow<2>(((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.lr, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lhl, 1)) * vtxbar(mgs, P.lr, 1))));  // 17068
          qhlacr(mgs) = ((((((NR(0.25) * P.pi) * ehlr(mgs)) * cx(mgs, P.lhl)) * qx(mgs, P.lr)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((dab1lh(mgs, P.lhl, P.lr) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.lr, 3))) + (da1lr(mgs) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 17071
          qhlacr(mgs) = c_min(qhlacr(mgs), qxmxd(mgs, P.lr));  // 17081
          qhlacrmlr(mgs) = qhlacr(mgs);  // 17084
          if (((temg(mgs) > P.tfr) && (P.iehlr0c == 0))) {  // 17086
            qhlacr(mgs) = NR(0.0);  // 17087
          } else {  // 17091
            chlacr(mgs) = ((((((NR(0.25) * P.pi) * ehlr(mgs)) * cx(mgs, P.lhl)) * cx(mgs, P.lr)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((dab0lh(mgs, P.lhl, P.lr) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.lr, 3))) + (da0lr(mgs) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 17092
            chlacr(mgs) = c_min(chlacr(mgs), crmxd(mgs));  // 17097
            if ((P.lvol(P.lhl) > 1)) {  // 17099
              vhlacr(mgs) = ((rho0(mgs) * qhlacr(mgs)) / raindn(mgs, P.lhl));  // 17100
            }  // 17101
          }  // 17102
        }  // 17103
      }  // 17104
    }  // 17105
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17117
      qiacw(mgs) = NR(0.0);  // 17118
      if ((eiw(mgs) > NR(0.0))) {  // 17119
        vt = sqrt((ipow<2>(((vtxbar(mgs, P.li, 1) - vtxbar(mgs, P.lc, 1)))) + ((NR(0.04) * vtxbar(mgs, P.li, 1)) * vtxbar(mgs, P.lc, 1))));  // 17121
        qiacw(mgs) = ((((((NR(0.25) * P.pi) * eiw(mgs)) * cx(mgs, P.li)) * qx(mgs, P.lc)) * vt) * ((((P.da0(P.li) * ipow<2>(xdia(mgs, P.li, 3))) + ((P.dab1(P.li, P.lc) * xdia(mgs, P.li, 3)) * xdia(mgs, P.lc, 3))) + (da1lc(mgs) * ipow<2>(xdia(mgs, P.lc, 3))))));  // 17124
        qiacw(mgs) = c_min(qiacw(mgs), qxmxd(mgs, P.lc));  // 17129
      }  // 17130
    }  // 17131
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17138
      qiacr(mgs) = NR(0.0);  // 17139
      qiacrf(mgs) = NR(0.0);  // 17140
      qiacrs(mgs) = NR(0.0);  // 17141
      ciacrs(mgs) = NR(0.0);  // 17142
      ciacr(mgs) = NR(0.0);  // 17143
      ciacrf(mgs) = NR(0.0);  // 17144
      viacrf(mgs) = NR(0.0);  // 17145
      csplinter(mgs) = NR(0.0);  // 17146
      qsplinter(mgs) = NR(0.0);  // 17147
      csplinter2(mgs) = NR(0.0);  // 17148
      qsplinter2(mgs) = NR(0.0);  // 17149
      if ((((P.iacr >= 1) && (eri(mgs) > NR(0.0))) && (temg(mgs) <= NR(270.15)))) {  // 17150
        if ((P.ipconc >= 3)) {  // 17152
          ni = NR(0.0);  // 17153
          if ((xdia(mgs, P.li, 1) >= NR(10.0e-6))) {  // 17154
            ni = (ni + (cx(mgs, P.li) * exp((-ipow<3>(((NR(40.0e-6) / xdia(mgs, P.li, 1))))))));  // 17155
          }  // 17156
          ratio = (NR(150.0e-6) / xdia(mgs, P.lr, 1));  // 17166
          i = c_min(P.nqiacrratio, int((ratio * P.dqiacrratioinv)));  // 17168
          j = int((c_max(NR(0.0), c_min(NR(15.0), alpha(mgs, P.lr))) * P.dqiacralphainv));  // 17169
          delx = (ratio - (NR(i) * P.dqiacrratio));  // 17171
          dely = (alpha(mgs, P.lr) - (NR(j) * P.dqiacralpha));  // 17172
          ip1 = c_min((i + 1), P.nqiacrratio);  // 17173
          jp1 = c_min((j + 1), P.nqiacralpha);  // 17174
          tmp1 = (P.ciacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, j) - P.ciacrratio(i, j)))));  // 17177
          tmp2 = (P.ciacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, jp1) - P.ciacrratio(i, jp1)))));  // 17178
          nr = (((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * cx(mgs, P.lr));  // 17182
          tmp1 = (P.qiacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, j) - P.qiacrratio(i, j)))));  // 17185
          tmp2 = (P.qiacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, jp1) - P.qiacrratio(i, jp1)))));  // 17186
          qr = (((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * qx(mgs, P.lr));  // 17190
          vt = sqrt((ipow<2>(((vtxbar(mgs, P.lr, 1) - vtxbar(mgs, P.li, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lr, 1)) * vtxbar(mgs, P.li, 1))));  // 17197
          qiacr(mgs) = ((((((NR(0.25) * P.pi) * eri(mgs)) * ni) * qr) * vt) * ((((P.da0(P.li) * ipow<2>(xdia(mgs, P.li, 3))) + ((dab1lh(mgs, P.li, P.lr) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.li, 3))) + (P.da1(P.lr) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 17200
          qiacr(mgs) = c_min(qrmxd(mgs), qiacr(mgs));  // 17205
          ciacr(mgs) = ((((((NR(0.25) * P.pi) * eri(mgs)) * ni) * nr) * vt) * ((((P.da0(P.li) * ipow<2>(xdia(mgs, P.li, 3))) + ((dab0lh(mgs, P.li, P.lr) * xdia(mgs, P.lr, 3)) * xdia(mgs, P.li, 3))) + (P.da0(P.lr) * ipow<2>(xdia(mgs, P.lr, 3))))));  // 17208
          ciacr(mgs) = c_min(crmxd(mgs), ciacr(mgs));  // 17213
          ciacrf(mgs) = ciacr(mgs);  // 17263
        }  // 17270
      } else {  // 17273
        qiacr(mgs) = c_min((((((((((NR(0.25) / gf4)) * P.pi)) * eri(mgs)) * cx(mgs, P.li)) * qx(mgs, P.lr)) * fabs((vtxbar(mgs, P.lr, 1) - vtxbar(mgs, P.li, 1)))) * (((((gf6 * gf1) * xdia(mgs, P.lr, 2)) + ((((NR(2.0) * gf5) * gf2) * xdia(mgs, P.lr, 1)) * xdia(mgs, P.li, 1))) + ((gf4 * gf3) * xdia(mgs, P.li, 2))))), qrmxd(mgs));  // 17274
      }  // 17282
      if ((P.ipconc >= 1)) {  // 17288
        csplinter(mgs) = (P.nsplinter * ciacr(mgs));  // 17303
        qsplinter(mgs) = c_min((NR(0.1) * qiacr(mgs)), ((csplinter(mgs) * P.splintermass) / rho0(mgs)));  // 17307
      }  // 17308
      frach = NR(1.0);  // 17310
      if ((ciacr(mgs) > P.qxmin(P.lh))) {  // 17312
        xvfrz = ((rho0(mgs) * qiacr(mgs)) / ((ciacr(mgs) * NR(900.0))));  // 17313
        frach = (NR(0.5) * ((NR(1.0) + tanh((NR(0.2e12) * ((xvfrz - (NR(1.15) * xvbiggsnow))))))));  // 17314
        qiacrs(mgs) = (((NR(1.0) - frach)) * qiacr(mgs));  // 17316
        ciacrs(mgs) = (((NR(1.0) - frach)) * ciacrf(mgs));  // 17317
      }  // 17319
      qiacrf(mgs) = (frach * qiacr(mgs));  // 17322
      ciacrf(mgs) = (frach * ciacrf(mgs));  // 17323
      if ((P.lvol(P.lh) > 1)) {  // 17325
        viacrf(mgs) = ((rho0(mgs) * qiacrf(mgs)) / P.rhofrz);  // 17326
      }  // 17327
    }  // 17329
    if ((P.ipconc >= 4)) {  // 17336
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17337
        csacs(mgs) = NR(0.0);  // 17338
        if (((qx(mgs, P.ls) > P.qxmin(P.ls)) && (ess(mgs) > NR(0.0)))) {  // 17339
          ec0(mgs) = NR(1.0);  // 17342
          csacs(mgs) = (((((ec0(mgs) * rvt) * aa2) * ess(mgs)) * ipow<2>(cx(mgs, P.ls))) * c_min(xv(mgs, P.ls), (((NR(4.0) * pii) / NR(3.0)) * ipow<3>(P.essrmax))));  // 17354
          csacs(mgs) = c_min(csacs(mgs), csmxd(mgs));  // 17356
        }  // 17357
      }  // 17358
    }  // 17359
    if (((P.ipconc >= 2) || (P.ipelec >= 9))) {  // 17363
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17364
        ciacw(mgs) = NR(0.0);  // 17365
        if (((eiw(mgs) > NR(0.0)) && (xmas(mgs, P.lc) > NR(0.0)))) {  // 17366
          ciacw(mgs) = ((qiacw(mgs) * rho0(mgs)) / xmas(mgs, P.lc));  // 17367
          ciacw(mgs) = c_min(ciacw(mgs), ccmxd(mgs));  // 17368
        }  // 17369
      }  // 17370
    }  // 17372
    if (((P.ipconc >= 2) || (P.ipelec >= 1))) {  // 17375
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17376
        tmp1 = NR(0.0);  // 17377
        cracw(mgs) = NR(0.0);  // 17378
        cracr(mgs) = NR(0.0);  // 17379
        ec0(mgs) = NR(1.0e9);  // 17380
        if ((((qx(mgs, P.lc) > P.qxmin(P.lc)) && (qx(mgs, P.lr) > P.qxmin(P.lr))) && (qracw(mgs) > NR(0.0)))) {  // 17381
          if ((P.ipconc < 3)) {  // 17384
            if ((erw(mgs) > NR(0.0))) {  // 17385
              cracw(mgs) = ((((((((NR(0.25)) * P.pi)) * erw(mgs)) * ((cx(mgs, P.lc) - ccwresv(mgs)))) * cx(mgs, P.lr)) * fabs((vtxbar(mgs, P.lr, 1) - vtxbar(mgs, P.lc, 1)))) * ((((gf1 * xdia(mgs, P.lc, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lc, 1)) * xdia(mgs, P.lr, 1))) + (gf3 * xdia(mgs, P.lr, 2)))));  // 17386
            }  // 17392
          } else {  // 17393
            if (((NR(0.5) * xdia(mgs, P.lr, 3)) > rh(mgs))) {  // 17395
              if (((NR(0.5) * xdia(mgs, P.lr, 3)) > rwradmn)) {  // 17397
                cracw(mgs) = (((aa2 * cx(mgs, P.lr)) * ((cx(mgs, P.lc) - ccwresv(mgs)))) * ((xv(mgs, P.lc) + xv(mgs, P.lr))));  // 17400
              } else {  // 17401
                cracw(mgs) = (((aa1 * cx(mgs, P.lr)) * ((cx(mgs, P.lc) - ccwresv(mgs)))) * ((((((alpha(mgs, P.lc) + NR(2.0))) * ipow<2>(xv(mgs, P.lc))) / ((alpha(mgs, P.lc) + NR(1.0)))) + ((((((alpha(mgs, P.lr) + NR(6.0))) * ((alpha(mgs, P.lr) + NR(5.0)))) * ((alpha(mgs, P.lr) + NR(4.0)))) * ipow<2>(xv(mgs, P.lr))) / (((((alpha(mgs, P.lr) + NR(3.0))) * ((alpha(mgs, P.lr) + NR(2.0)))) * ((alpha(mgs, P.lr) + NR(1.0)))))))));  // 17408
              }  // 17413
            }  // 17414
          }  // 17416
        }  // 17417
        ec0(mgs) = NR(1.0);  // 17422
        if ((qx(mgs, P.lr) > P.qxmin(P.lr))) {  // 17423
          rwrad = (NR(0.5) * xdia(mgs, P.lr, 3));  // 17424
          tmp = (xdia(mgs, P.lr, 3) - NR(0.1e-3));  // 17435
          if ((((((tmp > NR(1.9e-3)) && (P.irainbreak != 10)) && (P.irainbreak != 20))) || (P.icracr <= 0))) {  // 17441
            ec0(mgs) = NR(0.0);  // 17442
            cracr(mgs) = NR(0.0);  // 17443
          } else {  // 17449
            if (((xdia(mgs, P.lr, 3) < NR(6.1e-4)) || (P.irainbreak == 10))) {  // 17452
              ec0(mgs) = NR(1.0);  // 17453
            } else {  // 17454
              ec0(mgs) = exp((-(NR(2500.0) * ((xdia(mgs, P.lr, 3) - NR(6.0e-4))))));  // 17455
            }  // 17456
            if ((rwrad >= NR(50.0e-6))) {  // 17460
              tmp1 = ((aa2 * ipow<2>(cx(mgs, P.lr))) * xv(mgs, P.lr));  // 17461
              cracr(mgs) = (ec0(mgs) * tmp1);  // 17462
            } else {  // 17466
              tmp1 = (((((aa1 * ipow<2>(((cx(mgs, P.lr) * xv(mgs, P.lr))))) * ((alpha(mgs, P.lr) + NR(6.0)))) * ((alpha(mgs, P.lr) + NR(5.0)))) * ((alpha(mgs, P.lr) + NR(4.0)))) / (((((alpha(mgs, P.lr) + NR(3.0))) * ((alpha(mgs, P.lr) + NR(2.0)))) * ((alpha(mgs, P.lr) + NR(1.0))))));  // 17471
              cracr(mgs) = (ec0(mgs) * tmp1);  // 17474
            }  // 17479
          }  // 17482
        }  // 17492
        crbreak = NR(0.0);  // 17495
      }  // 17552
    }  // 17553
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chacw(i_0) = NR(0.0);  // 17561
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17562
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17563
        if ((P.ipconc >= 5)) {  // 17565
          if (((qhacw(mgs) > NR(0.0)) && (xmas(mgs, P.lc) > NR(0.0)))) {  // 17566
            chacw(mgs) = ((qhacw(mgs) * rho0(mgs)) / xmascw(mgs));  // 17580
            chacw(mgs) = c_min(chacw(mgs), ((NR(0.5) * ((cx(mgs, P.lc) - ccwresv(mgs)))) * dtpinv));  // 17582
          } else {  // 17583
            qhacw(mgs) = NR(0.0);  // 17584
          }  // 17585
        } else {  // 17586
          chacw(mgs) = ((((((((NR(0.25)) * P.pi)) * ehw(mgs)) * cx(mgs, P.lc)) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.lc, 1)))) * ((((gf1 * xdia(mgs, P.lc, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.lc, 1)) * xdia(mgs, P.lh, 1))) + (gf3 * xdia(mgs, P.lh, 2)))));  // 17588
          chacw(mgs) = c_min(chacw(mgs), ((NR(0.5) * cx(mgs, P.lc)) * dtpinv));  // 17594
        }  // 17597
      }  // 17598
    }  // 17599
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chaci(i_0) = NR(0.0);  // 17602
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chaci0(i_0) = NR(0.0);  // 17603
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17604
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17605
        if (((ehi(mgs) > NR(0.0)) || (((ehiclsn(mgs) > NR(0.0)) && (P.ipelec > 0))))) {  // 17606
          if ((P.ipconc >= 5)) {  // 17607
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.li, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lh, 1)) * vtxbar(mgs, P.li, 1))));  // 17609
            chaci0(mgs) = ((((((NR(0.25) * P.pi) * ehiclsn(mgs)) * cx(mgs, P.lh)) * cx(mgs, P.li)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab0lh(mgs, P.lh, P.li) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.li, 3))) + (P.da0(P.li) * ipow<2>(xdia(mgs, P.li, 3))))));  // 17612
          } else {  // 17617
            chaci0(mgs) = ((((((((NR(0.25)) * P.pi)) * ehiclsn(mgs)) * cx(mgs, P.li)) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.li, 1)))) * ((((gf1 * xdia(mgs, P.li, 2)) + (((NR(2.0) * gf2) * xdia(mgs, P.li, 1)) * xdia(mgs, P.lh, 1))) + (gf3 * xdia(mgs, P.lh, 2)))));  // 17618
          }  // 17624
          chaci(mgs) = c_min((ehi(mgs) * chaci0(mgs)), cimxd(mgs));  // 17626
        }  // 17627
      }  // 17628
    }  // 17629
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chacs(i_0) = NR(0.0);  // 17635
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chacs0(i_0) = NR(0.0);  // 17636
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17637
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17638
        if ((ehs(mgs) > 0)) {  // 17639
          if (((P.ipconc >= 5) || (((ehsclsn(mgs) > NR(0.0)) && (P.ipelec > 0))))) {  // 17640
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.ls, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lh, 1)) * vtxbar(mgs, P.ls, 1))));  // 17642
            chacs0(mgs) = ((((((NR(0.25) * P.pi) * ehsclsn(mgs)) * cx(mgs, P.lh)) * cx(mgs, P.ls)) * vt) * ((((da0lh(mgs) * ipow<2>(xdia(mgs, P.lh, 3))) + ((dab0lh(mgs, P.lh, P.ls) * xdia(mgs, P.lh, 3)) * xdia(mgs, P.ls, 3))) + (P.da0(P.ls) * ipow<2>(xdia(mgs, P.ls, 3))))));  // 17645
          } else {  // 17650
            chacs0(mgs) = ((((((((NR(0.25)) * P.pi)) * ehsclsn(mgs)) * cx(mgs, P.ls)) * cx(mgs, P.lh)) * fabs((vtxbar(mgs, P.lh, 1) - vtxbar(mgs, P.ls, 1)))) * (((((gf3 * gf1) * xdia(mgs, P.ls, 2)) + ((((NR(2.0) * gf2) * gf2) * xdia(mgs, P.ls, 1)) * xdia(mgs, P.lh, 1))) + ((gf1 * gf3) * xdia(mgs, P.lh, 2)))));  // 17651
          }  // 17657
          chacs(mgs) = c_min((ehs(mgs) * chacs0(mgs)), csmxd(mgs));  // 17658
        }  // 17659
      }  // 17660
    }  // 17661
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlacw(i_0) = NR(0.0);  // 17669
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17670
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17671
        if (((P.lhl > 1) && (P.ipconc >= 5))) {  // 17673
          if (((qhlacw(mgs) > NR(0.0)) && (xmas(mgs, P.lc) > NR(0.0)))) {  // 17674
            chlacw(mgs) = ((qhlacw(mgs) * rho0(mgs)) / xmascw(mgs));  // 17688
            chlacw(mgs) = c_min(chlacw(mgs), ((NR(0.5) * cx(mgs, P.lc)) * dtpinv));  // 17690
          } else {  // 17691
            qhlacw(mgs) = NR(0.0);  // 17692
          }  // 17693
        }  // 17704
      }  // 17705
    }  // 17706
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlaci(i_0) = NR(0.0);  // 17709
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlaci0(i_0) = NR(0.0);  // 17710
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17711
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17712
        if (((P.lhl > 1) && (((ehli(mgs) > NR(0.0)) || (((P.ipelec > 0) && (ehliclsn(mgs) > NR(0.0)))))))) {  // 17713
          if ((P.ipconc >= 5)) {  // 17714
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.li, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lhl, 1)) * vtxbar(mgs, P.li, 1))));  // 17716
            chlaci0(mgs) = ((((((NR(0.25) * P.pi) * ehliclsn(mgs)) * cx(mgs, P.lhl)) * cx(mgs, P.li)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((P.dab0(P.lhl, P.li) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.li, 3))) + (P.da0(P.li) * ipow<2>(xdia(mgs, P.li, 3))))));  // 17719
          }  // 17731
          chlaci(mgs) = c_min((ehli(mgs) * chlaci0(mgs)), cimxd(mgs));  // 17733
        }  // 17734
      }  // 17735
    }  // 17736
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlacs(i_0) = NR(0.0);  // 17742
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlacs0(i_0) = NR(0.0);  // 17743
    if (((P.ipconc >= 1) || (P.ipelec >= 1))) {  // 17744
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17745
        if (((P.lhl > 1) && (((ehls(mgs) > NR(0.0)) || (((P.ipelec > 0) && (ehlsclsn(mgs) > NR(0.0)))))))) {  // 17746
          if ((P.ipconc >= 5)) {  // 17747
            vt = sqrt((ipow<2>(((vtxbar(mgs, P.lhl, 1) - vtxbar(mgs, P.ls, 1)))) + ((NR(0.04) * vtxbar(mgs, P.lhl, 1)) * vtxbar(mgs, P.ls, 1))));  // 17749
            chlacs0(mgs) = ((((((NR(0.25) * P.pi) * ehlsclsn(mgs)) * cx(mgs, P.lhl)) * cx(mgs, P.ls)) * vt) * ((((da0lhl(mgs) * ipow<2>(xdia(mgs, P.lhl, 3))) + ((P.dab0(P.lhl, P.ls) * xdia(mgs, P.lhl, 3)) * xdia(mgs, P.ls, 3))) + (P.da0(P.ls) * ipow<2>(xdia(mgs, P.ls, 3))))));  // 17752
          }  // 17764
          chlacs(mgs) = c_min((ehls(mgs) * chlacs0(mgs)), csmxd(mgs));  // 17765
        }  // 17766
      }  // 17767
    }  // 17768
    if ((P.ipconc >= 2)) {  // 17774
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17777
        zrcnw(mgs) = NR(0.0);  // 17778
        qrcnw(mgs) = NR(0.0);  // 17779
        crcnw(mgs) = NR(0.0);  // 17780
        cautn(mgs) = NR(0.0);  // 17781
      }  // 17782
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17785
        if ((((qx(mgs, P.lc) > P.qxmin(P.lc)) && (cx(mgs, P.lc) > NR(1000.0))) && (temg(mgs) > (P.tfrh + NR(4.0))))) {  // 17788
          volb = (xv(mgs, P.lc) * pow(((NR(1.0) / ((NR(1.0) + alpha(mgs, P.lc))))), ((NR(1.0) / NR(2.0)))));  // 17790
          cautn(mgs) = c_min(ccmxd(mgs), (((((((alpha(mgs, P.lc) + NR(2.0))) / ((alpha(mgs, P.lc) + NR(1.0))))) * aa1) * ipow<2>(cx(mgs, P.lc))) * ipow<2>(xv(mgs, P.lc))));  // 17791
          cautn(mgs) = c_max(0.0e0, cautn(mgs));  // 17793
          if (((rb(mgs) <= 7.51e-6) || (P.dmrauto == (-1)))) {  // 17794
            t2s = 1.0e30;  // 17795
          } else {  // 17797
            t2s = (NR(3.72) / ((((NR(1.0e6) * ((rb(mgs) - 7.500e-6))) * rho0(mgs)) * qx(mgs, P.lc))));  // 17803
            qrcnw(mgs) = c_max(0.0e0, (xl2p(mgs) / ((t2s * rho0(mgs)))));  // 17805
            crcnw(mgs) = c_max(0.0e0, c_min(((NR(3.5e9) * xl2p(mgs)) / t2s), (NR(0.5) * cautn(mgs))));  // 17806
            if ((((qx(mgs, P.lr) * rho0(mgs)) > (NR(1.2) * xl2p(mgs))) && (cx(mgs, P.lr) > P.cxmin))) {  // 17809
              crcnw(mgs) = ((cx(mgs, P.lr) / qx(mgs, P.lr)) * qrcnw(mgs));  // 17810
            }  // 17841
            if ((crcnw(mgs) < NR(1.0e-30))) { qrcnw(mgs) = NR(0.0); }  // 17857
            if ((P.ipconc >= 6)) {  // 17859
            }  // 17899
          }  // 17924
        }  // 17927
      }  // 17928
    } else {  // 17934
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 17963
        qrcnw(mgs) = NR(0.0);  // 17964
        qrcnw(mgs) = NR(0.0);  // 17965
        qccrit = (((((P.pi / NR(6.0))) * ((cx(mgs, P.lc) * ipow<3>(P.cwdiap)))) * xdn(mgs, P.lc)) / rho0(mgs));  // 17966
        qdiff = c_max(((qx(mgs, P.lc) - qccrit)), NR(0.0));  // 17967
        if (((qdiff > NR(0.0)) && (cx(mgs, P.lc) > NR(1.0)))) {  // 17968
          argrcnw = (((NR(1.2e-4)) + ((((NR(1.596e-12)) * cx(mgs, P.lc)) * NR(1.0e-3)) / (((P.cwdisp * rho0(mgs)) * qdiff)))));  // 17969
          qrcnw(mgs) = (((NR(1.0e-3) * rho0(mgs)) * (ipow<2>(qdiff))) / argrcnw);  // 17972
          qrcnw(mgs) = c_min(qxmxd(mgs, P.lc), (c_max(qrcnw(mgs), NR(0.0))));  // 17975
        }  // 17978
      }  // 17979
    }  // 18017
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qrfrz(i_0) = NR(0.0);  // 18025
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qrfrzs(i_0) = NR(0.0);  // 18026
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qrfrzf(i_0) = NR(0.0);  // 18027
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vrfrzf(i_0) = NR(0.0);  // 18028
    for (int i_0 = 1; i_0 <= ngs; ++i_0) crfrz(i_0) = NR(0.0);  // 18029
    for (int i_0 = 1; i_0 <= ngs; ++i_0) crfrzs(i_0) = NR(0.0);  // 18030
    for (int i_0 = 1; i_0 <= ngs; ++i_0) crfrzf(i_0) = NR(0.0);  // 18031
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zrfrz(i_0) = NR(0.0);  // 18032
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zrfrzs(i_0) = NR(0.0);  // 18033
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zrfrzf(i_0) = NR(0.0);  // 18034
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qwcnr(i_0) = NR(0.0);  // 18035
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18039
      if ((((qx(mgs, P.lr) > P.qxmin(P.lr)) && (temcg(mgs) < (-NR(5.0)))) && (P.ibiggopt > 0))) {  // 18040
        if ((P.ipconc < 3)) {  // 18043
          qrfrz(mgs) = c_min((((((((NR(20.0)) * (ipow<2>(P.pi))) * brz) * ((xdn(mgs, P.lr) / rho0(mgs)))) * cx(mgs, P.lr)) * (ipow<6>(xdia(mgs, P.lr, 1)))) * ((exp(c_max((-(arz * temcg(mgs))), NR(0.0))) - NR(1.0)))), qrmxd(mgs));  // 18044
          qrfrzf(mgs) = qrfrz(mgs);  // 18050
        } else if ((P.ipconc >= 3)) {  // 18053
          frach = 1.0e0;  // 18057
          volt = (exp((NR(16.2) + (NR(1.0) * temcg(mgs)))) * NR(1.0e-6));  // 18063
          dbigg = pow((((NR(6.0) / P.pi) * volt)), ((NR(1.0) / NR(3.0))));  // 18066
          if ((dbigg < NR(8.0e-3))) {  // 18069
            ratio = c_min(P.maxratiolu, (dbigg / xdia(mgs, P.lr, 1)));  // 18071
            i = c_min(P.nqiacrratio, int((ratio * P.dqiacrratioinv)));  // 18073
            j = int((c_max(P.minalphalu, c_min(P.maxalphalu, alpha(mgs, P.lr))) * P.dqiacralphainv));  // 18077
            delx = (ratio - (NR(i) * P.dqiacrratio));  // 18079
            dely = (alpha(mgs, P.lr) - (NR(j) * P.dqiacralpha));  // 18080
            ip1 = c_min((i + 1), P.nqiacrratio);  // 18081
            jp1 = c_min((j + 1), P.nqiacralpha);  // 18082
            tmp1 = (P.ciacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, j) - P.ciacrratio(i, j)))));  // 18085
            tmp2 = (P.ciacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, jp1) - P.ciacrratio(i, jp1)))));  // 18086
            crfrz(mgs) = ((((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * cx(mgs, P.lr)) * dtpinv);  // 18090
            crfrzf(mgs) = crfrz(mgs);  // 18091
            tmp1 = (P.qiacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, j) - P.qiacrratio(i, j)))));  // 18093
            tmp2 = (P.qiacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, jp1) - P.qiacrratio(i, jp1)))));  // 18094
            qrfrz(mgs) = ((((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * qx(mgs, P.lr)) * dtpinv);  // 18098
            qrfrzf(mgs) = qrfrz(mgs);  // 18099
            if ((((qrfrz(mgs) * dtp) < P.qxmin(P.lh)) || ((crfrz(mgs) * dtp) < P.cxmin))) {  // 18101
              crfrz(mgs) = NR(0.0);  // 18103
              qrfrz(mgs) = NR(0.0);  // 18104
              qrfrzf(mgs) = NR(0.0);  // 18105
            } else {  // 18107
              if (((P.ipconc >= 5) || (P.lzr > 1))) {  // 18110
                cxd1 = (crfrz(mgs) * dtp);  // 18112
                qxd1 = (qrfrz(mgs) * dtp);  // 18113
                tmp1 = (P.ziacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.ziacrratio(ip1, j) - P.ziacrratio(i, j)))));  // 18116
                tmp2 = (P.ziacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.ziacrratio(ip1, jp1) - P.ziacrratio(i, jp1)))));  // 18117
                tmp5 = ((g1x(mgs, P.lr) * ipow<2>(((rho0(mgs) * qx(mgs, P.lr))))) / ((ipow<2>((((P.pi * xdn(mgs, P.lr)) / NR(6.0)))) * cx(mgs, P.lr))));  // 18136
                zxd1 = (((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * tmp5);  // 18137
                tmp3 = ((g1x(mgs, P.lr) * ipow<2>(((rho0(mgs) * qxd1)))) / (ipow<2>((((P.pi * xdn(mgs, P.lr)) / NR(6.0))))));  // 18140
                tmp4 = (tmp3 / cxd1);  // 18141
                if ((tmp4 > zxd1)) {  // 18142
                  crfrzf(mgs) = ((tmp3 / zxd1) * dtpinv);  // 18143
                }  // 18144
              }  // 18146
              if (((dbigg < c_max(P.biggsnowdiam, c_max(P.dfrz, P.dhmn))) && (((P.ibiggsnow == 1) || (P.ibiggsnow == 3))))) {  // 18161
                crfrzs(mgs) = crfrz(mgs);  // 18164
                qrfrzs(mgs) = qrfrz(mgs);  // 18165
                ratio = c_min(P.maxratiolu, (c_max(P.dfrz, P.dhmn) / xdia(mgs, P.lr, 1)));  // 18179
                i = c_min(P.nqiacrratio, int((ratio * P.dqiacrratioinv)));  // 18181
                j = int((c_max(P.minalphalu, c_min(P.maxalphalu, alpha(mgs, P.lr))) * P.dqiacralphainv));  // 18187
                delx = (ratio - (NR(i) * P.dqiacrratio));  // 18189
                dely = (alpha(mgs, P.lr) - (NR(j) * P.dqiacralpha));  // 18190
                ip1 = c_min((i + 1), P.nqiacrratio);  // 18191
                jp1 = c_min((j + 1), P.nqiacralpha);  // 18192
                tmp1 = (P.ciacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, j) - P.ciacrratio(i, j)))));  // 18195
                tmp2 = (P.ciacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.ciacrratio(ip1, jp1) - P.ciacrratio(i, jp1)))));  // 18196
                crfrzf(mgs) = ((((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * cx(mgs, P.lr)) * dtpinv);  // 18201
                tmp1 = (P.qiacrratio(i, j) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, j) - P.qiacrratio(i, j)))));  // 18204
                tmp2 = (P.qiacrratio(i, jp1) + ((delx * P.dqiacrratioinv) * ((P.qiacrratio(ip1, jp1) - P.qiacrratio(i, jp1)))));  // 18205
                qrfrzf(mgs) = ((((tmp1 + ((dely * P.dqiacralphainv) * ((tmp2 - tmp1))))) * qx(mgs, P.lr)) * dtpinv);  // 18209
                crfrzs(mgs) = (crfrzs(mgs) - crfrzf(mgs));  // 18212
                qrfrzs(mgs) = (qrfrzs(mgs) - qrfrzf(mgs));  // 18213
              } else {  // 18228
                crfrzs(mgs) = NR(0.0);  // 18229
                qrfrzs(mgs) = NR(0.0);  // 18230
                zrfrzs(mgs) = NR(0.0);  // 18231
              }  // 18232
            }  // 18234
            if ((((qrfrz(mgs)) * dtp) > qx(mgs, P.lr))) {  // 18236
              fac = (((qrfrz(mgs)) * dtp) / qx(mgs, P.lr));  // 18237
              qrfrz(mgs) = (fac * qrfrz(mgs));  // 18238
              qrfrzs(mgs) = (fac * qrfrzs(mgs));  // 18239
              qrfrzf(mgs) = (fac * qrfrzf(mgs));  // 18240
              crfrz(mgs) = (fac * crfrz(mgs));  // 18241
              crfrzs(mgs) = (fac * crfrzs(mgs));  // 18242
              crfrzf(mgs) = (fac * crfrzf(mgs));  // 18243
            }  // 18248
          }  // 18250
          if ((P.lvol(P.lh) > 1)) {  // 18359
            vrfrzf(mgs) = ((rho0(mgs) * qrfrzf(mgs)) / P.rhofrz);  // 18360
          }  // 18361
        }  // 18400
      } else {  // 18402
      }  // 18404
    }  // 18405
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18413
      qwfrz(mgs) = NR(0.0);  // 18414
      cwfrz(mgs) = NR(0.0);  // 18415
      qwfrzc(mgs) = NR(0.0);  // 18416
      cwfrzc(mgs) = NR(0.0);  // 18417
      qwfrzp(mgs) = NR(0.0);  // 18418
      cwfrzp(mgs) = NR(0.0);  // 18419
      if ((((P.ibfc >= 1) && (P.ibfc != 3)) && (temg(mgs) < NR(268.15)))) {  // 18420
        if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (cx(mgs, P.lc) > P.cxmin))) {  // 18423
          if ((P.ipconc < 2)) {  // 18424
            qwfrz(mgs) = (((((((NR(2.0)) * (brz)) / ((xdn(mgs, P.lc) * cx(mgs, P.lc))))) * ((exp(c_max((-(arz * temcg(mgs))), NR(0.0))) - NR(1.0)))) * rho0(mgs)) * (ipow<2>(qx(mgs, P.lc))));  // 18425
            qwfrz(mgs) = c_max(qwfrz(mgs), NR(0.0));  // 18428
            qwfrz(mgs) = c_min(qwfrz(mgs), qcmxd(mgs));  // 18429
            cwfrz(mgs) = ((qwfrz(mgs) * rho0(mgs)) / xmas(mgs, P.li));  // 18430
          } else if ((P.ipconc >= 2)) {  // 18431
            if ((xdia(mgs, P.lc, 3) > NR(0.0e-6))) {  // 18432
              volt = (exp((NR(16.2) + (NR(1.0) * temcg(mgs)))) * NR(1.0e-6));  // 18433
              if ((alpha(mgs, P.lc) == NR(0.0))) {  // 18438
                cwfrz(mgs) = ((cx(mgs, P.lc) * exp((-(volt / xv(mgs, P.lc))))) * dtpinv);  // 18439
                qwfrz(mgs) = (((cwfrz(mgs) * xdn0(P.lc)) * rhoinv(mgs)) * ((volt + xv(mgs, P.lc))));  // 18443
              } else {  // 18444
                ratio = ((((NR(1.0) + alpha(mgs, P.lc))) * volt) / xv(mgs, P.lc));  // 18445
                ratio = c_min(P.maxratiolu, ratio);  // 18459
                tmp = gaminterp(P, ratio, alpha(mgs, P.lc), 1, 1);  // 18463
                cwfrz(mgs) = ((cx(mgs, P.lc) * tmp) * dtpinv);  // 18465
                tmp = gaminterp(P, ratio, alpha(mgs, P.lc), 12, 1);  // 18467
                qwfrz(mgs) = (((((cx(mgs, P.lc) * xdn0(P.lc)) * xv(mgs, P.lc)) * rhoinv(mgs)) * dtpinv) * tmp);  // 18469
              }  // 18473
            }  // 18475
          }  // 18476
          if ((temg(mgs) > NR(268.15))) {  // 18477
            qwfrz(mgs) = NR(0.0);  // 18478
            cwfrz(mgs) = NR(0.0);  // 18479
          }  // 18480
        }  // 18481
      }  // 18482
      if ((xplate(mgs) == 1)) {  // 18484
        qwfrzp(mgs) = qwfrz(mgs);  // 18485
        cwfrzp(mgs) = cwfrz(mgs);  // 18486
      }  // 18487
      if ((xcolmn(mgs) == 1)) {  // 18489
        qwfrzc(mgs) = qwfrz(mgs);  // 18490
        cwfrzc(mgs) = cwfrz(mgs);  // 18491
      }  // 18492
    }  // 18498
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18505
      ccia(mgs) = NR(0.0);  // 18507
      cwctfz(mgs) = NR(0.0);  // 18509
      qwctfz(mgs) = NR(0.0);  // 18510
      ctfzbd(mgs) = NR(0.0);  // 18511
      ctfzth(mgs) = NR(0.0);  // 18512
      ctfzdi(mgs) = NR(0.0);  // 18513
      cwctfzc(mgs) = NR(0.0);  // 18515
      qwctfzc(mgs) = NR(0.0);  // 18516
      cwctfzp(mgs) = NR(0.0);  // 18517
      qwctfzp(mgs) = NR(0.0);  // 18518
      if (((temg(mgs) < NR(271.15)) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 18521
        ccia(mgs) = exp((NR(4.11) - ((NR(0.262)) * temcg(mgs))));  // 18526
        knud(mgs) = ((NR(2.28e-5) * temg(mgs)) / ((pres(mgs) * raero)));  // 18532
        knuda(mgs) = (NR(1.257) + (NR(0.4) * exp((-(NR(1.1) / knud(mgs))))));  // 18533
        gtp(mgs) = (NR(1.0) / ((fai(mgs) + fbi(mgs))));  // 18534
        dfar(mgs) = (((kb * temg(mgs)) * ((NR(1.0) + (knuda(mgs) * knud(mgs))))) / ((((NR(6.0) * P.pi) * fadvisc(mgs)) * raero)));  // 18535
        fn1(mgs) = ((((NR(2.0) * P.pi) * xdia(mgs, P.lc, 1)) * cx(mgs, P.lc)) * ccia(mgs));  // 18536
        fn2(mgs) = (-(((gtp(mgs) * ((ssw(mgs) - NR(1.0)))) * felv(mgs)) / pres(mgs)));  // 18537
        fnft(mgs) = (((NR(0.4) * (((NR(1.0) + (NR(1.45) * knud(mgs))) + ((NR(0.4) * knud(mgs)) * exp((-(NR(1.0) / knud(mgs)))))))) * ((ftka(mgs) + ((NR(2.5) * knud(mgs)) * kaero)))) / ((((NR(1.0) + (NR(3.0) * knud(mgs)))) * ((((2 * ftka(mgs)) + ((NR(5.0) * knud(mgs)) * kaero)) + kaero)))));  // 18538
        ctfzbd(mgs) = (fn1(mgs) * dfar(mgs));  // 18543
        ctfzth(mgs) = (((fn1(mgs) * fn2(mgs)) * fnft(mgs)) / rho0(mgs));  // 18546
        ctfzdi(mgs) = ((((fn1(mgs) * fn2(mgs)) * P.rw) * temg(mgs)) / ((felv(mgs) * rho0(mgs))));  // 18549
        cwctfz(mgs) = c_max(((ctfzbd(mgs) + ctfzth(mgs)) + ctfzdi(mgs)), NR(0.0));  // 18551
        if ((P.ipconc >= 2)) {  // 18568
          cwctfz(mgs) = c_min((cwctfz(mgs) * dtpinv), ccmxd(mgs));  // 18569
          qwctfz(mgs) = ((xmas(mgs, P.lc) * cwctfz(mgs)) / rho0(mgs));  // 18570
        } else {  // 18571
          qwctfz(mgs) = (((cimasn) * cwctfz(mgs)) / ((dtp * rho0(mgs))));  // 18572
          qwctfz(mgs) = c_max(qwctfz(mgs), NR(0.0));  // 18573
          qwctfz(mgs) = c_min(qwctfz(mgs), qcmxd(mgs));  // 18574
        }  // 18575
        if ((xplate(mgs) == 1)) {  // 18578
          qwctfzp(mgs) = qwctfz(mgs);  // 18579
          cwctfzp(mgs) = cwctfz(mgs);  // 18580
        }  // 18581
        if ((xcolmn(mgs) == 1)) {  // 18583
          qwctfzc(mgs) = qwctfz(mgs);  // 18584
          cwctfzc(mgs) = cwctfz(mgs);  // 18585
        }  // 18586
      }  // 18596
    }  // 18600
    dthr = NR(300.0);  // 18607
    hrifac = ((NR(1.0e-3)) * (((NR(0.044)) * (NR(hexd(0x3EB0C6F7A0B5ED8Eull))))));  // 18608
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18609
      ciihr(mgs) = NR(0.0);  // 18610
      qiihr(mgs) = NR(0.0);  // 18611
      cicichr(mgs) = NR(0.0);  // 18612
      qicichr(mgs) = NR(0.0);  // 18613
      cipiphr(mgs) = NR(0.0);  // 18614
      qipiphr(mgs) = NR(0.0);  // 18615
    }  // 18655
    hdia0 = NR(300.0e-6);  // 18680
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18681
      qscnvi(mgs) = NR(0.0);  // 18682
      cscnvi(mgs) = NR(0.0);  // 18683
      cscnvis(mgs) = NR(0.0);  // 18684
      if (((temg(mgs) < P.tfr) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 18687
        if ((P.ipconc < 4)) {  // 18707
          qscnvi(mgs) = ((NR(0.001) * eii(mgs)) * c_max(((qx(mgs, P.li) - NR(1.0e-3))), NR(0.0)));  // 18709
          qscnvi(mgs) = c_min(qscnvi(mgs), qxmxd(mgs, P.li));  // 18710
          cscnvi(mgs) = ((qscnvi(mgs) * rho0(mgs)) / xmas(mgs, P.li));  // 18711
          cscnvis(mgs) = (NR(0.5) * cscnvi(mgs));  // 18712
        }  // 18714
      }  // 18715
    }  // 18717
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18724
      fvent(mgs) = ((vpow(fschm(mgs), ((NR(1.0) / NR(3.0))), mgs - 1, ngscnt)) * (vpow(fakvisc(mgs), ((-NR(0.5))), mgs - 1, ngscnt)));  // 18725  // libmvec 2-lane pow (vectorised loop)
    }  // 18726
    civenta = NR(1.258e4);  // 18731
    civentb = NR(2.331);  // 18732
    civentc = NR(5.662e4);  // 18733
    civentd = NR(2.373);  // 18734
    civente = NR(0.8241);  // 18735
    civentf = (-NR(0.042));  // 18736
    civentg = NR(1.70);  // 18737
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18739
      if ((qx(mgs, P.li) > P.qxmin(P.li))) {  // 18742
        cireyn = ((((civenta * pow(xdia(mgs, P.li, 1), civentb)) + (civentc * pow(xdia(mgs, P.li, 1), civentd)))) / (((civente * pow(xdia(mgs, P.li, 1), civentf)) + civentg)));  // 18743
        xcivent = ((pow(fschm(mgs), ((NR(1.0) / NR(3.0))))) * (pow(((cireyn / fakvisc(mgs))), NR(0.5))));  // 18748
        if ((xcivent < NR(1.0))) {  // 18749
          civent(mgs) = (NR(1.0) + (NR(0.14) * ipow<2>(xcivent)));  // 18750
        }  // 18751
        if ((xcivent >= NR(1.0))) {  // 18752
          civent(mgs) = (NR(0.86) + (NR(0.28) * xcivent));  // 18753
        }  // 18754
      } else {  // 18755
        civent(mgs) = NR(0.0);  // 18756
      }  // 18757
    }  // 18761
    igmrwa = (NR(100.0) * NR(2.0));  // 18765
    igmrwb = (NR(100.0) * ((((NR(5.0) + P.br)) / NR(2.0))));  // 18766
    rwventa = ((NR(0.78)) * P.gmoi(igmrwa));  // 18767
    rwventb = ((NR(0.308)) * P.gmoi(igmrwb));  // 18768
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18769
      if ((qx(mgs, P.lr) > P.qxmin(P.lr))) {  // 18770
        if ((P.ipconc >= 3)) {  // 18771
          x = (NR(1.0) + alpha(mgs, P.lr));  // 18834
          rwvent(mgs) = (((NR(0.78) * x) + ((((NR(0.308) * ventrxn(mgs)) * fvent(mgs)) * sqrt(((P.ar * rhovt(mgs))))) * (pow(xdia(mgs, P.lr, 1), ((((NR(1.0) + P.br)) / NR(2.0))))))));  // 18836
          if ((P.ipconc >= 7)) {  // 18842
            alpr = c_min(P.alpharmax, alpha(mgs, P.lr));  // 18844
            tmp = ((alpr + NR(5.5)) + (P.br / NR(2.0)));  // 18846
            i = int((P.dgami * (tmp)));  // 18847
            del = (tmp - (P.dgam * i));  // 18848
            y = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 18849
            rwventz(mgs) = (((((NR(0.78) * ((NR(4.0) + alpr))) * ((NR(3.0) + alpr))) * ((NR(2.0) + alpr))) * ((NR(1.0) + alpr))) + ((((NR(0.308) * fvent(mgs)) * sqrt((P.ax(P.lr) * rhovt(mgs)))) * ((y / gf1palp(mgs)))) * (pow(xdia(mgs, P.lr, 1), ((((NR(1.0) + P.br)) / NR(2.0)))))));  // 18853
          }  // 18858
        } else {  // 18864
          rwvent(mgs) = ((rwventa + (((rwventb * fvent(mgs)) * sqrt(((P.ar * rhovt(mgs))))) * (pow(xdia(mgs, P.lr, 1), ((((NR(1.0) + P.br)) / NR(2.0))))))));  // 18865
        }  // 18869
      } else {  // 18870
        rwvent(mgs) = NR(0.0);  // 18871
      }  // 18872
    }  // 18873
    igmswa = (NR(100.0) * NR(2.0));  // 18875
    igmswb = (NR(100.0) * ((((NR(5.0) + ds)) / NR(2.0))));  // 18876
    swventa = ((NR(0.78)) * P.gmoi(igmswa));  // 18877
    swventb = ((NR(0.308)) * P.gmoi(igmswb));  // 18878
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18879
      if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 18880
        if ((P.ipconc >= 4)) {  // 18881
          swvent(mgs) = (NR(0.65) + ((NR(0.44) * fvent(mgs)) * sqrt((vtxbar(mgs, P.ls, 1) * xdia(mgs, P.ls, 1)))));  // 18882
        } else {  // 18883
          swvent(mgs) = ((swventa + (((swventb * fvent(mgs)) * sqrt(((cs * rhovt(mgs))))) * (pow(xdia(mgs, P.ls, 1), ((((NR(1.0) + ds)) / NR(2.0))))))));  // 18885
        }  // 18889
      } else {  // 18890
        swvent(mgs) = NR(0.0);  // 18891
      }  // 18892
    }  // 18893
    igmhwa = (NR(100.0) * NR(2.0));  // 18897
    igmhwb = (NR(100.0) * NR(2.75));  // 18898
    hwventa = ((NR(0.78)) * P.gmoi(igmhwa));  // 18899
    hwventb = ((NR(0.308)) * P.gmoi(igmhwb));  // 18900
    for (int i_0 = 1; i_0 <= ngs; ++i_0) hwvent(i_0) = NR(0.0);  // 18902
    for (int i_0 = 1; i_0 <= ngs; ++i_0) hwventy(i_0) = NR(0.0);  // 18903
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18905
      if ((qx(mgs, P.lh) > P.qxmin(P.lh))) {  // 18906
        hwventc = pow((((NR(4.0) * P.gr) / ((NR(3.0) * cdxgs(mgs, P.lh))))), (NR(0.25)));  // 18907
        if ((false || (alpha(mgs, P.lh) == NR(0.0)))) {  // 18908
          hwvent(mgs) = ((hwventa + ((((hwventb * hwventc) * fvent(mgs)) * (pow(((xdn(mgs, P.lh) / rho0(mgs))), (NR(0.25))))) * (pow(xdia(mgs, P.lh, 1), (NR(0.75)))))));  // 18909
        } else {  // 18913
          x = (NR(1.0) + alpha(mgs, P.lh));  // 18922
          tmp = (1 + alpha(mgs, P.lh));  // 18924
          i = int((P.dgami * (tmp)));  // 18925
          del = (tmp - (P.dgam * i));  // 18926
          g1palp = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 18927
          tmp = ((NR(2.5) + alpha(mgs, P.lh)) + (NR(0.5) * bxx(mgs, P.lh)));  // 18929
          i = int((P.dgami * (tmp)));  // 18930
          del = (tmp - (P.dgam * i));  // 18931
          y = (((P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami))) / g1palp);  // 18932
          hwventy(mgs) = (((NR(0.308) * fvent(mgs)) * (pow(xdia(mgs, P.lh, 1), ((NR(0.5) + (NR(0.5) * bxx(mgs, P.lh))))))) * sqrt((axx(mgs, P.lh) * rhovt(mgs))));  // 18935
          hwvent(mgs) = (((NR(0.78) * x) + (y * hwventy(mgs))));  // 18936
        }  // 18941
      } else {  // 18942
        hwvent(mgs) = NR(0.0);  // 18943
        hwventy(mgs) = NR(0.0);  // 18944
      }  // 18945
    }  // 18946
    for (int i_0 = 1; i_0 <= ngs; ++i_0) hlvent(i_0) = NR(0.0);  // 18949
    for (int i_0 = 1; i_0 <= ngs; ++i_0) hlventy(i_0) = NR(0.0);  // 18950
    if ((P.lhl > 1)) {  // 18952
      igmhwa = (NR(100.0) * NR(2.0));  // 18953
      igmhwb = (NR(100.0) * NR(2.75));  // 18954
      hwventa = ((NR(0.78)) * P.gmoi(igmhwa));  // 18955
      hwventb = ((NR(0.308)) * P.gmoi(igmhwb));  // 18956
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 18958
        if ((qx(mgs, P.lhl) > P.qxmin(P.lhl))) {  // 18959
          hwventc = pow((((NR(4.0) * P.gr) / ((NR(3.0) * cdxgs(mgs, P.lhl))))), (NR(0.25)));  // 18960
          if ((false || (alpha(mgs, P.lhl) == NR(0.0)))) {  // 18962
            hlvent(mgs) = ((hwventa + ((((hwventb * hwventc) * fvent(mgs)) * (pow(((xdn(mgs, P.lhl) / rho0(mgs))), (NR(0.25))))) * (pow(xdia(mgs, P.lhl, 1), (NR(0.75)))))));  // 18963
          } else {  // 18967
            x = (NR(1.0) + alpha(mgs, P.lhl));  // 18977
            tmp = (1 + alpha(mgs, P.lhl));  // 18979
            i = int((P.dgami * (tmp)));  // 18980
            del = (tmp - (P.dgam * i));  // 18981
            g1palp = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 18982
            tmp = ((NR(2.5) + alpha(mgs, P.lhl)) + (NR(0.5) * bxx(mgs, P.lhl)));  // 18984
            i = int((P.dgami * (tmp)));  // 18985
            del = (tmp - (P.dgam * i));  // 18986
            y = (((P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami))) / g1palp);  // 18987
            hlventy(mgs) = (((NR(0.308) * fvent(mgs)) * (pow(xdia(mgs, P.lhl, 1), ((NR(0.5) + (NR(0.5) * bxx(mgs, P.lhl))))))) * sqrt((axx(mgs, P.lhl) * rhovt(mgs))));  // 18989
            hlvent(mgs) = ((NR(0.78) * x) + (y * hlventy(mgs)));  // 18991
          }  // 18996
        }  // 18997
      }  // 18998
    }  // 18999
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19006
      fwet1(mgs) = ((((NR(2.0) * P.pi)) * (((((felv(mgs) * fwvdf(mgs)) * rho0(mgs)) * ((qss0(mgs) - qx(mgs, P.lv)))) - (ftka(mgs) * temcg(mgs))))) / ((rho0(mgs) * ((felf(mgs) + (fcw(mgs) * temcg(mgs)))))));  // 19007
      fwet2(mgs) = ((NR(1.0)) - ((fci(mgs) * temcg(mgs)) / ((felf(mgs) + (fcw(mgs) * temcg(mgs))))));  // 19012
    }  // 19015
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19019
      fmlt1(mgs) = ((((NR(2.0) * P.pi)) * ((((felv(mgs) * fwvdf(mgs)) * ((qss0(mgs) - qx(mgs, P.lv)))) - ((ftka(mgs) * temcg(mgs)) / rho0(mgs))))) / (felf(mgs)));  // 19020
      fmlt2(mgs) = (-((fcw(mgs) * temcg(mgs)) / felf(mgs)));  // 19024
      fmlt1e(mgs) = ((((NR(2.0) * P.pi)) * (((felv(mgs) * fwvdf(mgs)) * ((qss0(mgs) - qx(mgs, P.lv)))))) / (felf(mgs)));  // 19025
    }  // 19027
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19031
      fvds(mgs) = (((((NR(4.0) * P.pi) / rho0(mgs))) * ((ssi(mgs) - NR(1.0)))) * ((NR(1.0) / ((fai(mgs) + fbi(mgs))))));  // 19032
    }  // 19035
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19036
      fvce(mgs) = (((((NR(4.0) * P.pi) / rho0(mgs))) * ((ssw(mgs) - NR(1.0)))) * ((NR(1.0) / ((fav(mgs) + fbv(mgs))))));  // 19037
    }  // 19040
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qsmlr(i_0) = NR(0.0);  // 19045
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qimlr(i_0) = NR(0.0);  // 19046
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhmlr(i_0) = NR(0.0);  // 19047
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlmlr(i_0) = NR(0.0);  // 19048
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhfzh(i_0) = NR(0.0);  // 19053
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qffzf(i_0) = NR(0.0);  // 19054
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlfzhl(i_0) = NR(0.0);  // 19055
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhfzhlg(i_0) = NR(0.0);  // 19056
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlfzhllg(i_0) = NR(0.0);  // 19057
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhfzh(i_0) = NR(0.0);  // 19058
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vffzf(i_0) = NR(0.0);  // 19059
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlfzhl(i_0) = NR(0.0);  // 19060
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qsfzs(i_0) = NR(0.0);  // 19061
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhmlr(i_0) = NR(0.0);  // 19063
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhmlrr(i_0) = NR(0.0);  // 19064
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zsmlrr(i_0) = NR(0.0);  // 19065
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhshr(i_0) = NR(0.0);  // 19066
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhlmlr(i_0) = NR(0.0);  // 19067
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhlshr(i_0) = NR(0.0);  // 19068
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhshrr(i_0) = NR(0.0);  // 19070
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhlmlrr(i_0) = NR(0.0);  // 19071
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhlshrr(i_0) = NR(0.0);  // 19072
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csmlr(i_0) = NR(0.0);  // 19074
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csmlrr(i_0) = NR(0.0);  // 19075
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chmlr(i_0) = NR(0.0);  // 19076
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chmlrr(i_0) = NR(0.0);  // 19077
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlmlr(i_0) = NR(0.0);  // 19078
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlfmlr(i_0) = NR(0.0);  // 19079
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlmlrr(i_0) = NR(0.0);  // 19084
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19088
      if ((temg(mgs) > P.tfr)) {  // 19090
        if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 19092
          qsmlr(mgs) = c_min((((((c1sw * fmlt1(mgs)) * cx(mgs, P.ls)) * swvent(mgs)) * xdia(mgs, P.ls, 1))), NR(0.0));  // 19093
        }  // 19097
        if ((qx(mgs, P.lh) > P.qxmin(P.lh))) {  // 19111
          qhmlr(mgs) = (P.meltfac * c_min(((((fmlt1(mgs) * cx(mgs, P.lh)) * hwvent(mgs)) * xdia(mgs, P.lh, 1)) + (fmlt2(mgs) * ((qhacrmlr(mgs) + qhacwmlr(mgs))))), NR(0.0)));  // 19114
          if (((((P.ivhmltsoak > 0) && (qhmlr(mgs) < NR(0.0))) && (P.lvol(P.lh) > 1)) && (xdn(mgs, P.lh) < xdnmx(P.lh)))) {  // 19129
            v1 = ((((NR(1.0) - (xdn(mgs, P.lh) / xdnmx(P.lh)))) * ((vx(mgs, P.lh) + ((rho0(mgs) * qhmlr(mgs)) / xdn(mgs, P.lh))))) / (dtp));  // 19131
            v2 = (-(((NR(1.0) * rho0(mgs)) * qhmlr(mgs)) / xdnmx(P.lh)));  // 19132
            vhsoak(mgs) = c_min(v1, v2);  // 19134
          }  // 19136
        }  // 19138
        if (((P.lhl > 1) && (P.lhlw < 1))) {  // 19141
          if ((qx(mgs, P.lhl) > P.qxmin(P.lhl))) {  // 19143
            qhlmlr(mgs) = (P.meltfac * c_min(((((fmlt1(mgs) * cx(mgs, P.lhl)) * hlvent(mgs)) * xdia(mgs, P.lhl, 1)) + (fmlt2(mgs) * ((qhlacrmlr(mgs) + qhlacwmlr(mgs))))), NR(0.0)));  // 19145
            if (((((P.ivhmltsoak > 0) && (qhlmlr(mgs) < NR(0.0))) && (P.lvol(P.lhl) > 1)) && (xdn(mgs, P.lhl) < xdnmx(P.lhl)))) {  // 19159
              v1 = ((((NR(1.0) - (xdn(mgs, P.lhl) / xdnmx(P.lhl)))) * ((vx(mgs, P.lhl) + ((rho0(mgs) * qhlmlr(mgs)) / xdn(mgs, P.lhl))))) / (dtp));  // 19161
              v2 = (-(((NR(1.0) * rho0(mgs)) * qhlmlr(mgs)) / xdnmx(P.lhl)));  // 19162
              vhlsoak(mgs) = c_min(v1, v2);  // 19164
            }  // 19166
          }  // 19168
        }  // 19169
      }  // 19171
      qsmlr(mgs) = c_max(qsmlr(mgs), c_min((-qsmxd(mgs)), (-((NR(0.7) * qx(mgs, P.ls)) * dtpinv))));  // 19177
      qhmlr(mgs) = c_max(qhmlr(mgs), c_min((-qhmxd(mgs)), (-((NR(0.95) * qx(mgs, P.lh)) * dtpinv))));  // 19179
      chmlr(mgs) = c_max(chmlr(mgs), c_min((-chmxd(mgs)), (-((NR(0.95) * cx(mgs, P.lh)) * dtpinv))));  // 19180
      qhmlh(mgs) = NR(0.0);  // 19183
      if (((P.lhl > 1) && (P.lhlw < 1))) {  // 19189
        qhlmlr(mgs) = c_max(qhlmlr(mgs), c_min((-qxmxd(mgs, P.lhl)), (-((NR(0.95) * qx(mgs, P.lhl)) * dtpinv))));  // 19190
        chlmlr(mgs) = c_max(chlmlr(mgs), c_min((-cxmxd(mgs, P.lhl)), (-((NR(0.95) * cx(mgs, P.lhl)) * dtpinv))));  // 19191
      }  // 19192
    }  // 19195
    if ((P.ipconc >= 1)) {  // 19199
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19200
        cimlr(mgs) = (((cx(mgs, P.li) / ((qx(mgs, P.li) + NR(1.0e-20))))) * qimlr(mgs));  // 19201
        if ((((xdia(mgs, P.ls, 1) > NR(1.0e-6)) && ((-qsmlr(mgs)) >= (NR(0.5) * P.qxmin(P.ls)))) && (P.ipconc >= 4))) {  // 19203
          csmlr(mgs) = (((cx(mgs, P.ls) / (qx(mgs, P.ls)))) * qsmlr(mgs));  // 19205
        } else if ((qx(mgs, P.ls) > P.qxmin(P.ls))) {  // 19206
          csmlr(mgs) = (((cx(mgs, P.ls) / (qx(mgs, P.ls)))) * qsmlr(mgs));  // 19207
        }  // 19208
        csmlrr(mgs) = (csmlr(mgs) / rzxs(mgs));  // 19210
        chmlr(mgs) = (((cx(mgs, P.lh) / ((qx(mgs, P.lh) + NR(1.0e-20))))) * qhmlr(mgs));  // 19225
        if (((chmlr(mgs) < NR(0.0)) && (((P.ibinhmlr < 1) || (P.lzh < 1))))) {  // 19260
          if (((xv(mgs, P.lh) > NR(0.0)) && (chmlr(mgs) < NR(0.0)))) {  // 19274
            tmp = (-((rho0(mgs) * qhmlr(mgs)) / (c_min((xdn(mgs, P.lr) * P.xvmx(P.lr)), (xdn(mgs, P.lh) * xv(mgs, P.lh))))));  // 19280
            tmp2 = (-((rho0(mgs) * qhmlr(mgs)) / ((xdn(mgs, P.lr) * vr3mm))));  // 19281
            chmlrr(mgs) = (((tmp * ((P.sheddiam0 - xdia(mgs, P.lh, 3)))) / ((P.sheddiam0 - P.sheddiam))) + ((tmp2 * ((xdia(mgs, P.lh, 3) - P.sheddiam))) / ((P.sheddiam0 - P.sheddiam))));  // 19283
            chmlrr(mgs) = (-c_max(tmp, c_min(tmp2, chmlrr(mgs))));  // 19284
          } else {  // 19292
            chmlrr(mgs) = chmlr(mgs);  // 19293
          }  // 19294
        }  // 19303
        if (((((P.lhl > 1) && (P.lhlw < 1)) && (!P.mixedphase)) && (qhlmlr(mgs) < NR(0.0)))) {  // 19305
          chlmlr(mgs) = (((cx(mgs, P.lhl) / ((qx(mgs, P.lhl) + NR(1.0e-20))))) * qhlmlr(mgs));  // 19312
          if (((xv(mgs, P.lhl) > NR(0.0)) && (chlmlr(mgs) < NR(0.0)))) {  // 19347
            tmp = (-((rho0(mgs) * qhlmlr(mgs)) / (c_min((xdn(mgs, P.lr) * P.xvmx(P.lr)), (xdn(mgs, P.lhl) * xv(mgs, P.lhl))))));  // 19351
            tmp2 = (-((rho0(mgs) * qhlmlr(mgs)) / ((xdn(mgs, P.lr) * vr3mm))));  // 19352
            chlmlrr(mgs) = (((tmp * ((NR(20.0e-3) - xdia(mgs, P.lhl, 3)))) / ((NR(20.0e-3) - P.sheddiam))) + ((tmp2 * ((xdia(mgs, P.lhl, 3) - P.sheddiam))) / ((NR(20.0e-3) - P.sheddiam))));  // 19353
            chlmlrr(mgs) = (-c_max(tmp, c_min(tmp2, chlmlrr(mgs))));  // 19354
          } else {  // 19362
            chlmlrr(mgs) = chlmlr(mgs);  // 19363
          }  // 19364
        }  // 19385
      }  // 19392
    }  // 19393
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19398
      rwcap(mgs) = ((NR(0.5)) * xdia(mgs, P.lr, 1));  // 19400
      swcap(mgs) = ((NR(0.5)) * xdia(mgs, P.ls, 1));  // 19401
      hwcap(mgs) = ((NR(0.5)) * xdia(mgs, P.lh, 1));  // 19402
      if ((P.lhl > 1)) { hlcap(mgs) = ((NR(0.5)) * xdia(mgs, P.lhl, 1)); }  // 19403
      if (((qx(mgs, P.li) > P.qxmin(P.li)) && (xdia(mgs, P.li, 1) > NR(0.0)))) {  // 19405
        cilen(mgs) = (NR(0.4764) * pow((xdia(mgs, P.li, 1)), (NR(0.958))));  // 19409
        cval = xdia(mgs, P.li, 1);  // 19410
        aval = cilen(mgs);  // 19411
        eval = sqrt((NR(1.0) - ((ipow<2>(aval)) / (ipow<2>(cval)))));  // 19412
        fval = c_min(NR(0.99), eval);  // 19413
        gval = log(fabs((((NR(1.0) + fval)) / ((NR(1.0) - fval)))));  // 19414
        cicap(mgs) = ((cval * fval) / gval);  // 19415
      } else {  // 19416
        cicap(mgs) = NR(0.0);  // 19417
      }  // 19418
    }  // 19419
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhdsv(i_0) = NR(0.0);  // 19422
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhldsv(i_0) = NR(0.0);  // 19423
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19425
      qidsv(mgs) = ((((fvds(mgs) * cx(mgs, P.li)) * civent(mgs)) * cicap(mgs)) * P.depfac);  // 19428
      qsdsv(mgs) = ((((fvds(mgs) * cx(mgs, P.ls)) * swvent(mgs)) * swcap(mgs)) * P.depfac);  // 19430
      qhdsv(mgs) = ((((fvds(mgs) * cx(mgs, P.lh)) * hwvent(mgs)) * hwcap(mgs)) * P.depfac);  // 19442
      if ((P.lhl > 1)) { qhldsv(mgs) = ((((fvds(mgs) * cx(mgs, P.lhl)) * hlvent(mgs)) * hlcap(mgs)) * P.depfac); }  // 19445
    }  // 19448
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19461
      qitmp(mgs) = ((qx(mgs, P.li) + qx(mgs, P.ls)) + qx(mgs, P.lh));  // 19463
      if ((P.lhl > 1)) { qitmp(mgs) = (qitmp(mgs) + qx(mgs, P.lhl)); }  // 19465
      qrtmp(mgs) = qx(mgs, P.lr);  // 19466
      qctmp(mgs) = qx(mgs, P.lc);  // 19467
      qsimxdep(mgs) = NR(0.0);  // 19468
      qsimxsub(mgs) = NR(0.0);  // 19469
      dqcitmp(mgs) = NR(0.0);  // 19470
      if ((qitmp(mgs) > P.qxmin(P.li))) {  // 19474
        qitmp1 = qitmp(mgs);  // 19476
        qctmp1 = qctmp(mgs);  // 19477
        felvcptmp = felvcp(mgs);  // 19478
        felscptmp = felscp(mgs);  // 19479
        qvtmp(mgs) = qx(mgs, P.lv);  // 19480
        qss(mgs) = qvs(mgs);  // 19481
        qsstmp = qvs(mgs);  // 19482
        qvstmp = qvs(mgs);  // 19483
        qisstmp = qis(mgs);  // 19484
        thetatmp = theta(mgs);  // 19485
        thetaptmp = thetap(mgs);  // 19486
        temgtmp = temg(mgs);  // 19487
        temcgtmp = temcg(mgs);  // 19488
        qvaptmp = qx(mgs, P.lv);  // 19489
        qvptmp = NR(0.0);  // 19490
        qsstmp = qisstmp;  // 19492
        dqwvtmp(mgs) = ((qvtmp(mgs) - qsstmp));  // 19495
        for (itertd = 1; itertd <= 2; ++itertd) {  // 19497
          if ((itertd == 1)) {  // 19502
          } else {  // 19504
            dqcitmp(mgs) = dqci(mgs);  // 19505
          }  // 19507
          dqcw(mgs) = NR(0.0);  // 19509
          dqci(mgs) = NR(0.0);  // 19510
          dqwv(mgs) = ((qvtmp(mgs) - qsstmp));  // 19511
          if ((dqwv(mgs) < NR(0.0))) {  // 19515
            if ((qitmp(mgs) > (-dqwv(mgs)))) {  // 19516
              dqci(mgs) = dqwv(mgs);  // 19517
              dqwv(mgs) = NR(0.0);  // 19518
            } else {  // 19519
              dqci(mgs) = (-qitmp(mgs));  // 19520
              dqwv(mgs) = (dqwv(mgs) + qitmp(mgs));  // 19521
            }  // 19522
            qvptmp = (qvptmp - ((dqcw(mgs) + dqci(mgs))));  // 19524
            qctmp(mgs) = (qctmp(mgs) + dqcw(mgs));  // 19539
            qitmp(mgs) = (qitmp(mgs) + dqci(mgs));  // 19540
            thetaptmp = (thetaptmp + ((NR(1.0) / pi0(mgs)) * (((felvcp(mgs) * dqcw(mgs)) + (felscp(mgs) * dqci(mgs))))));  // 19541
          }  // 19546
          if ((dqwv(mgs) >= NR(0.0))) {  // 19550
            fracl(mgs) = NR(0.0);  // 19555
            fraci(mgs) = NR(1.0);  // 19556
            if (((temg(mgs) < P.tfr) && (temg(mgs) > thnuc))) {  // 19557
            }  // 19560
            if ((temg(mgs) <= thnuc)) {  // 19561
              fraci(mgs) = NR(1.0);  // 19562
              fracl(mgs) = NR(0.0);  // 19563
            }  // 19564
            gamss = ((((felvcp(mgs) * fracl(mgs)) + (felscp(mgs) * fraci(mgs)))) / (pi0(mgs)));  // 19567
            dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv2(mgs) * qsstmp) / (ipow<2>(((temg(mgs) - P.cbi))))))));  // 19570
            if ((temg(mgs) >= P.tfr)) {  // 19573
              dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv1(mgs) * qsstmp) / (ipow<2>(((temg(mgs) - P.cbw))))))));  // 19574
            }  // 19576
            delqci1 = qx(mgs, P.li);  // 19578
            dqcw(mgs) = (dqvcnd(mgs) * fracl(mgs));  // 19581
            dqci(mgs) = (dqvcnd(mgs) * fraci(mgs));  // 19582
            thetaptmp = (thetaptmp + ((((felvcp(mgs) * dqcw(mgs)) + (felscp(mgs) * dqci(mgs)))) / (pi0(mgs))));  // 19584
            qvptmp = (qvptmp - (dqvcnd(mgs)));  // 19588
            qctmp(mgs) = (qctmp(mgs) + dqcw(mgs));  // 19589
            qitmp(mgs) = (qitmp(mgs) + dqci(mgs));  // 19590
          }  // 19611
          if ((itertd == 1)) {  // 19615
            thetatmp = (thetaptmp + theta0(mgs));  // 19618
            temgtmp = (thetatmp * pk(mgs));  // 19619
            qvaptmp = c_max(((qvptmp + qv0(mgs))), NR(0.0));  // 19620
            temcgtmp = (temgtmp - P.tfr);  // 19621
            tqvcon = (temgtmp - P.cbw);  // 19622
            ltemq = ((((temgtmp - NR(163.15))) / P.fqsat) + NR(1.5));  // 19623
            ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 19624
            qvstmp = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 19629
            qisstmp = (pqs(mgs) * P.tabqis(ltemq));  // 19632
            qctmp(mgs) = c_max(NR(0.0), qctmp(mgs));  // 19633
            qitmp(mgs) = c_max(NR(0.0), qitmp(mgs));  // 19634
            qvtmp(mgs) = c_max(NR(0.0), qvaptmp);  // 19635
            qsstmp = qisstmp;  // 19638
          } else {  // 19640
            qctmp(mgs) = c_max(NR(0.0), qctmp(mgs));  // 19642
            qitmp(mgs) = c_max(NR(0.0), qitmp(mgs));  // 19643
            if ((qitmp(mgs) < qitmp1)) {  // 19645
              qsimxsub(mgs) = (((qitmp1 - qitmp(mgs))) * dtpinv);  // 19646
            } else if ((qitmp(mgs) > qitmp1)) {  // 19647
              qsimxdep(mgs) = (((qitmp(mgs) - qitmp1)) * dtpinv);  // 19648
            }  // 19649
          }  // 19652
        }  // 19658
      }  // 19660
    }  // 19662
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhcev(i_0) = NR(0.0);  // 19675
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chcev(i_0) = NR(0.0);  // 19676
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlcev(i_0) = NR(0.0);  // 19677
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlcev(i_0) = NR(0.0);  // 19678
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qfcev(i_0) = NR(0.0);  // 19679
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19681
      qisbv(mgs) = NR(0.0);  // 19682
      qssbv(mgs) = NR(0.0);  // 19683
      qidpv(mgs) = NR(0.0);  // 19684
      qsdpv(mgs) = NR(0.0);  // 19685
      qhsbv(mgs) = NR(0.0);  // 19686
      qscev(mgs) = NR(0.0);  // 19687
      cscev(mgs) = NR(0.0);  // 19688
      qisbv(mgs) = c_max(c_min(qidsv(mgs), NR(0.0)), c_min((-qimxd(mgs)), (-((NR(0.5) * qx(mgs, P.li)) * dtpinv))));  // 19694
      if (((temg(mgs) < P.tfr) || (!(qsmlr(mgs) < NR(0.0))))) {  // 19695
        qssbv(mgs) = c_max(c_min(qsdsv(mgs), NR(0.0)), c_min((-qsmxd(mgs)), (-((NR(0.5) * qx(mgs, P.ls)) * dtpinv))));  // 19696
      }  // 19697
      qidpv(mgs) = c_max(qidsv(mgs), NR(0.0));  // 19698
      qsdpv(mgs) = c_max(qsdsv(mgs), NR(0.0));  // 19699
      if (((qsmlr(mgs) < NR(0.0)) && (!P.mixedphase))) {  // 19701
        qscev(mgs) = (((((((P.evapfac * NR(4.0)) * P.pi) * ((qx(mgs, P.lv) - qss0(mgs)))) * cx(mgs, P.ls)) * swcap(mgs)) * swvent(mgs)) / ((qss0(mgs) * ((fav(mgs) + fbv(mgs))))));  // 19703
        qscev(mgs) = c_max(c_min(NR(0.0), qscev(mgs)), c_min((-qsmxd(mgs)), (-((NR(0.5) * qx(mgs, P.ls)) * dtpinv))));  // 19705
      } else {  // 19706
      }  // 19708
      qhsbv(mgs) = NR(0.0);  // 19719
      qhdpv(mgs) = NR(0.0);  // 19720
      if ((qx(mgs, P.lh) > P.qxmin(P.lh))) {  // 19721
        if (((temg(mgs) < P.tfr) || (!(qhmlr(mgs) < NR(0.0))))) {  // 19722
          qhsbv(mgs) = c_max(c_min(qhdsv(mgs), NR(0.0)), (-qhmxd(mgs)));  // 19724
          qhdpv(mgs) = c_max(qhdsv(mgs), NR(0.0));  // 19725
        }  // 19726
        if (((true && (qhmlr(mgs) < NR(0.0))) && (!P.mixedphase))) {  // 19728
          qhcev(mgs) = (((((((P.evapfac * NR(2.0)) * P.pi) * ((qx(mgs, P.lv) - qss0(mgs)))) * cx(mgs, P.lh)) * xdia(mgs, P.lh, 1)) * hwvent(mgs)) / ((qss0(mgs) * ((fav(mgs) + fbv(mgs))))));  // 19734
          qhcev(mgs) = c_max(qhcev(mgs), (-qhmxd(mgs)));  // 19737
          if ((temg(mgs) > P.tfr)) { qhcev(mgs) = c_min(NR(0.0), qhcev(mgs)); }  // 19738
        }  // 19740
      }  // 19741
      qhlsbv(mgs) = NR(0.0);  // 19744
      qhldpv(mgs) = NR(0.0);  // 19745
      if ((P.lhl > 1)) {  // 19746
        if ((qx(mgs, P.lhl) > P.qxmin(P.lhl))) {  // 19747
          if (((temg(mgs) < P.tfr) || (!(qhlmlr(mgs) < NR(0.0))))) {  // 19748
            qhlsbv(mgs) = c_max(c_min(qhldsv(mgs), NR(0.0)), (-qxmxd(mgs, P.lhl)));  // 19749
            qhldpv(mgs) = c_max(qhldsv(mgs), NR(0.0));  // 19750
          }  // 19751
          if (((qhlmlr(mgs) < NR(0.0)) && (!P.mixedphase))) {  // 19752
            qhlcev(mgs) = (((((((P.evapfac * NR(2.0)) * P.pi) * ((qx(mgs, P.lv) - qss0(mgs)))) * cx(mgs, P.lhl)) * xdia(mgs, P.lhl, 1)) * hlvent(mgs)) / ((qss0(mgs) * ((fav(mgs) + fbv(mgs))))));  // 19754
            qhlcev(mgs) = c_max(qhlcev(mgs), (-qhlmxd(mgs)));  // 19757
            if ((temg(mgs) > P.tfr)) { qhlcev(mgs) = c_min(NR(0.0), qhlcev(mgs)); }  // 19758
          }  // 19760
        }  // 19761
      }  // 19762
      temp1 = (((qidpv(mgs) + qsdpv(mgs)) + qhdpv(mgs)) + qhldpv(mgs));  // 19764
      if ((temp1 > qsimxdep(mgs))) {  // 19770
        frac = (qsimxdep(mgs) / temp1);  // 19771
        qidpv(mgs) = (frac * qidpv(mgs));  // 19773
        qsdpv(mgs) = (frac * qsdpv(mgs));  // 19774
        qhdpv(mgs) = (frac * qhdpv(mgs));  // 19775
        qhldpv(mgs) = (frac * qhldpv(mgs));  // 19776
      }  // 19783
      temp1 = (((qisbv(mgs) + qssbv(mgs)) + qhsbv(mgs)) + qhlsbv(mgs));  // 19785
      if ((temp1 < (-qsimxsub(mgs)))) {  // 19788
        frac = (-(qsimxsub(mgs) / temp1));  // 19789
        qisbv(mgs) = (frac * qisbv(mgs));  // 19791
        qssbv(mgs) = (frac * qssbv(mgs));  // 19792
        qhsbv(mgs) = (frac * qhsbv(mgs));  // 19793
        qhlsbv(mgs) = (frac * qhlsbv(mgs));  // 19794
      }  // 19801
    }  // 19804
    if ((P.ipconc >= 1)) {  // 19807
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19808
        cssbv(mgs) = (((cx(mgs, P.ls) / ((qx(mgs, P.ls) + NR(1.0e-20))))) * qssbv(mgs));  // 19809
        cisbv(mgs) = (((cx(mgs, P.li) / ((qx(mgs, P.li) + NR(1.0e-20))))) * qisbv(mgs));  // 19810
        chsbv(mgs) = (((cx(mgs, P.lh) / ((qx(mgs, P.lh) + NR(1.0e-20))))) * qhsbv(mgs));  // 19811
        if ((P.lhl > 1)) { chlsbv(mgs) = (((cx(mgs, P.lhl) / ((qx(mgs, P.lhl) + NR(1.0e-20))))) * qhlsbv(mgs)); }  // 19812
        csdpv(mgs) = NR(0.0);  // 19813
        cidpv(mgs) = NR(0.0);  // 19814
        cisdpv(mgs) = NR(0.0);  // 19815
        chdpv(mgs) = NR(0.0);  // 19816
        chldpv(mgs) = NR(0.0);  // 19817
      }  // 19818
    }  // 19819
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 19825
      qscni(mgs) = NR(0.0);  // 19826
      cscni(mgs) = NR(0.0);  // 19827
      cscnis(mgs) = NR(0.0);  // 19828
      if ((((P.ipconc >= 4) && (P.iscni >= 1)) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 19829
        if ((((P.iscni != 5) && (qidpv(mgs) > NR(0.0))) && (xdia(mgs, P.li, 3) >= NR(100.0e-6)))) {  // 19838
          qscni(mgs) = (c_min(NR(0.5), (xdia(mgs, P.li, 3) / NR(200.0e-6))) * qidpv(mgs));  // 19843
          cscni(mgs) = (((P.fscni * qscni(mgs)) * rho0(mgs)) / c_max((P.rho_qs * P.xvmn(P.ls)), xmas(mgs, P.li)));  // 19847
          cscnis(mgs) = cscni(mgs);  // 19851
        }  // 19856
      } else if ((P.ipconc < 4)) {  // 19877
        qscni(mgs) = ((NR(0.001) * eii(mgs)) * c_max(((qx(mgs, P.li) - NR(1.0e-3))), NR(0.0)));  // 19882
        qscni(mgs) = c_min(qscni(mgs), qxmxd(mgs, P.li));  // 19883
      } else {  // 19885
        if (((P.iscni > 0) && (qx(mgs, P.li) > P.qxmin(P.li)))) {  // 19886
          qscni(mgs) = (((((((P.pi * rho0(mgs)) * (((NR(0.25)) / (NR(6.0))))) * eii(mgs)) * (ipow<2>(qx(mgs, P.li)))) * (xdia(mgs, P.li, 2))) * vtxbar(mgs, P.li, 1)) / xmas(mgs, P.li));  // 19887
          cscni(mgs) = c_min(cimxd(mgs), ((qscni(mgs) * rho0(mgs)) / xmas(mgs, P.li)));  // 19891
        }  // 19892
      }  // 19894
    }  // 19895
    for (int i_0 = 1; i_0 <= ngs; ++i_0) dhwet(i_0) = d1t;  // 19898
    for (int i_0 = 1; i_0 <= ngs; ++i_0) dhlwet(i_0) = d1t;  // 19899
    for (int i_0 = 1; i_0 <= ngs; ++i_0) dfwet(i_0) = d1t;  // 19900
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20031
      qsdry(mgs) = ((qsacr(mgs) + qsacw(mgs)) + qsaci(mgs));  // 20033
      qhdry(mgs) = (((qhaci(mgs) + qhacs(mgs)) + qhacr(mgs)) + qhacw(mgs));  // 20036
      qhldry(mgs) = NR(0.0);  // 20041
      if ((P.lhl > 1)) {  // 20042
        qhldry(mgs) = (((qhlaci(mgs) + qhlacs(mgs)) + qhlacr(mgs)) + qhlacw(mgs));  // 20043
      }  // 20046
    }  // 20047
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20051
      if (((tfrdry < temg(mgs)) && (temg(mgs) < P.tfr))) {  // 20053
        qhwet(mgs) = (((((xdia(mgs, P.lh, 1) * hwvent(mgs)) * cx(mgs, P.lh)) * fwet1(mgs)) + (fwet2(mgs) * ((qhaci(mgs) + qhacs(mgs))))));  // 20065
        qhwet(mgs) = c_max(NR(0.0), qhwet(mgs));  // 20068
        qhlwet(mgs) = NR(0.0);  // 20151
        if ((P.lhl > 1)) {  // 20152
          qhlwet(mgs) = (((((xdia(mgs, P.lhl, 1) * hlvent(mgs)) * cx(mgs, P.lhl)) * fwet1(mgs)) + (fwet2(mgs) * ((qhlaci(mgs) + qhlacs(mgs))))));  // 20154
          qhlwet(mgs) = c_max(NR(0.0), qhlwet(mgs));  // 20157
        }  // 20236
      } else {  // 20238
        qhwet(mgs) = qhdry(mgs);  // 20240
        qhlwet(mgs) = qhldry(mgs);  // 20241
      }  // 20242
    }  // 20246
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qsshr(i_0) = NR(0.0);  // 20251
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhshr(i_0) = NR(0.0);  // 20252
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlshr(i_0) = NR(0.0);  // 20253
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhshh(i_0) = NR(0.0);  // 20254
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csshr(i_0) = NR(0.0);  // 20255
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csshrr(i_0) = NR(0.0);  // 20256
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chshr(i_0) = NR(0.0);  // 20257
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlshr(i_0) = NR(0.0);  // 20258
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chshrr(i_0) = NR(0.0);  // 20259
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlshrr(i_0) = NR(0.0);  // 20260
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhshdr(i_0) = NR(0.0);  // 20261
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlshdr(i_0) = NR(0.0);  // 20262
    for (int i_0 = 1; i_0 <= ngs; ++i_0) wetsfc(i_0) = false;  // 20263
    for (int i_0 = 1; i_0 <= ngs; ++i_0) wetgrowth(i_0) = false;  // 20264
    for (int i_0 = 1; i_0 <= ngs; ++i_0) wetsfchl(i_0) = false;  // 20265
    for (int i_0 = 1; i_0 <= ngs; ++i_0) wetgrowthhl(i_0) = false;  // 20266
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20268
      qhshr(mgs) = c_min(NR(0.0), (qhwet(mgs) - qhdry(mgs)));  // 20272
      qhlshr(mgs) = c_min(NR(0.0), (qhlwet(mgs) - qhldry(mgs)));  // 20276
      qsshr(mgs) = NR(0.0);  // 20281
      if ((temg(mgs) < NR(243.15))) {  // 20286
        qsshr(mgs) = NR(0.0);  // 20287
        qhshr(mgs) = NR(0.0);  // 20288
        qhlshr(mgs) = NR(0.0);  // 20289
        vhshdr(mgs) = NR(0.0);  // 20290
        vhlshdr(mgs) = NR(0.0);  // 20291
        wetsfc(mgs) = false;  // 20292
        wetgrowth(mgs) = false;  // 20293
        wetsfchl(mgs) = false;  // 20294
        wetgrowthhl(mgs) = false;  // 20295
      }  // 20296
      if ((temg(mgs) > P.tfr)) {  // 20300
        qsshr(mgs) = ((-qsacr(mgs)) - qsacw(mgs));  // 20308
        qhlshr(mgs) = ((-qhlacw(mgs)) - qhlacr(mgs));  // 20309
        qhshr(mgs) = ((-qhacw(mgs)) - qhacr(mgs));  // 20310
        vhshdr(mgs) = ((-vhacw(mgs)) - vhacr(mgs));  // 20314
        vhlshdr(mgs) = ((-vhlacw(mgs)) - vhlacr(mgs));  // 20315
        qhwet(mgs) = NR(0.0);  // 20316
        qhlwet(mgs) = NR(0.0);  // 20317
      }  // 20318
      wetsfc(mgs) = ((((qhshr(mgs) < NR(0.0)) && (temg(mgs) < P.tfr))) || (((qhmlr(mgs) < (-P.qxmin(P.lh))) && (temg(mgs) > P.tfr))));  // 20321
      wetgrowth(mgs) = (((qhshr(mgs) < NR(0.0)) && (temg(mgs) < P.tfr)));  // 20322
      if (((qhlshr(mgs) < NR(0.0)) && (temg(mgs) < P.tfr))) {  // 20324
        wetsfchl(mgs) = ((((qhlshr(mgs) < NR(0.0)) && (temg(mgs) < P.tfr))) || (((qhlmlr(mgs) < (-P.qxmin(P.lhl))) && (temg(mgs) > P.tfr))));  // 20325
        wetgrowthhl(mgs) = (((qhlshr(mgs) < NR(0.0)) && (temg(mgs) < P.tfr)));  // 20326
      }  // 20327
    }  // 20329
    if ((P.ipconc >= 1)) {  // 20331
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20332
        csshr(mgs) = NR(0.0);  // 20333
        chshr(mgs) = NR(0.0);  // 20335
        chshrr(mgs) = ((rho0(mgs) * qhshr(mgs)) / ((xdn(mgs, P.lr) * vshdgs(mgs, P.lh))));  // 20341
        chlshr(mgs) = NR(0.0);  // 20345
        chlshrr(mgs) = NR(0.0);  // 20346
        if ((P.lhl > 1)) {  // 20347
          chlshr(mgs) = NR(0.0);  // 20351
          chlshrr(mgs) = ((rho0(mgs) * qhlshr(mgs)) / ((xdn(mgs, P.lr) * vshdgs(mgs, P.lhl))));  // 20357
        }  // 20359
      }  // 20362
    }  // 20363
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20370
      if ((qsshr(mgs) < NR(0.0))) {  // 20374
        qsdpv(mgs) = NR(0.0);  // 20375
        qssbv(mgs) = NR(0.0);  // 20376
      } else {  // 20377
        qsshr(mgs) = NR(0.0);  // 20378
      }  // 20379
      if ((wetgrowth(mgs) || (((P.mixedphase && (fhw(mgs) > NR(0.05))) && (temg(mgs) > NR(243.15)))))) {  // 20391
        if (((P.lvol(P.lh) > 1) && (!P.mixedphase))) {  // 20396
          rimdn(mgs, P.lh) = xdnmx(P.lh);  // 20400
          raindn(mgs, P.lh) = xdnmx(P.lh);  // 20401
          vhacw(mgs) = ((qhacw(mgs) * rho0(mgs)) / rimdn(mgs, P.lh));  // 20402
          vhacr(mgs) = ((qhacr(mgs) * rho0(mgs)) / raindn(mgs, P.lh));  // 20403
          if ((xdn(mgs, P.lh) < xdnmx(P.lh))) {  // 20405
            v1 = ((((NR(1.0) - (xdn(mgs, P.lh) / xdnmx(P.lh)))) * vx(mgs, P.lh)) / (dtp));  // 20408
            v2 = ((rho0(mgs) * qhwet(mgs)) / xdnmx(P.lh));  // 20410
            vhsoak(mgs) = c_min(v1, v2);  // 20412
          }  // 20415
          vhshdr(mgs) = c_min(NR(0.0), ((((rho0(mgs) * qhwet(mgs)) / xdnmx(P.lh)) - vhacw(mgs)) - vhacr(mgs)));  // 20419
        }  // 20424
        qhdpv(mgs) = NR(0.0);  // 20427
        chdpv(mgs) = NR(0.0);  // 20429
        if ((ehi(mgs) > NR(0.0))) {  // 20434
          qhaci(mgs) = c_min(qimxd(mgs), qhaci0(mgs));  // 20435
          chaci(mgs) = c_min(cimxd(mgs), chaci0(mgs));  // 20436
        }  // 20437
        if ((ehs(mgs) > NR(0.0))) {  // 20438
          qhacs(mgs) = c_min(qsmxd(mgs), qhacs0(mgs));  // 20440
          chacs(mgs) = c_min(csmxd(mgs), chacs0(mgs));  // 20441
          ehs(mgs) = P.ehsmax;  // 20442
          qhacs(mgs) = c_min(qsmxd(mgs), qhacs(mgs));  // 20443
        }  // 20444
        wetsfc(mgs) = true;  // 20447
      } else {  // 20449
      }  // 20451
      if (((P.lhl > 1) && ((wetgrowthhl(mgs) || (((P.mixedphase && (fhlw(mgs) > NR(0.05))) && (temg(mgs) > NR(243.15)))))))) {  // 20457
        qhldpv(mgs) = NR(0.0);  // 20461
        chldpv(mgs) = NR(0.0);  // 20463
        if (((P.lvol(P.lhl) > 1) && (!P.mixedphase))) {  // 20469
          rimdn(mgs, P.lhl) = xdnmx(P.lhl);  // 20474
          raindn(mgs, P.lhl) = xdnmx(P.lhl);  // 20475
          vhlacw(mgs) = ((qhlacw(mgs) * rho0(mgs)) / rimdn(mgs, P.lhl));  // 20476
          vhlacr(mgs) = ((qhlacr(mgs) * rho0(mgs)) / raindn(mgs, P.lhl));  // 20477
          if ((xdn(mgs, P.lhl) < xdnmx(P.lhl))) {  // 20479
            v1 = ((((NR(1.0) - (xdn(mgs, P.lhl) / xdnmx(P.lhl)))) * vx(mgs, P.lhl)) / (dtp));  // 20482
            v2 = ((rho0(mgs) * qhlwet(mgs)) / xdnmx(P.lhl));  // 20484
            if ((v1 > v2)) {  // 20485
              vhlsoak(mgs) = v2;  // 20486
            } else {  // 20487
              vhlsoak(mgs) = v1;  // 20488
            }  // 20489
          } else {  // 20492
            vhlsoak(mgs) = NR(0.0);  // 20493
          }  // 20497
          vhlshdr(mgs) = c_min(NR(0.0), ((((rho0(mgs) * qhlwet(mgs)) / xdnmx(P.lhl)) - vhlacw(mgs)) - vhlacr(mgs)));  // 20501
        }  // 20507
        if ((ehli(mgs) > NR(0.0))) {  // 20509
          qhlaci(mgs) = c_min(qimxd(mgs), qhlaci0(mgs));  // 20510
          chlaci(mgs) = c_min(cimxd(mgs), chlaci0(mgs));  // 20511
        }  // 20512
        if ((ehls(mgs) > NR(0.0))) {  // 20517
          qhlacs(mgs) = c_min(qsmxd(mgs), qhlacs0(mgs));  // 20518
          chlacs(mgs) = c_min(csmxd(mgs), chlacs0(mgs));  // 20519
          ehls(mgs) = P.ehsmax;  // 20520
        }  // 20522
        wetsfchl(mgs) = true;  // 20528
      } else {  // 20531
      }  // 20534
    }  // 20536
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20540
      qhcni(mgs) = NR(0.0);  // 20542
      chcni(mgs) = NR(0.0);  // 20543
      chcnih(mgs) = NR(0.0);  // 20544
      vhcni(mgs) = NR(0.0);  // 20545
      if (((temg(mgs) < NR(273.0)) && ((qiacw(mgs) - qidpv(mgs)) > NR(0.0)))) {  // 20548
        tmp = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lc, 1))) * (((NR(0.60)) * vtxbar(mgs, P.li, 1)))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 20551
        tmp = c_min(c_max(P.rimc3, tmp), NR(900.0));  // 20554
        if (((tmp >= NR(200.0)) || (P.iglcnvi >= 2))) {  // 20562
          r = c_max((NR(0.5) * ((xdn(mgs, P.li) + tmp))), xdnmn(P.lh));  // 20563
          qhcni(mgs) = ((qiacw(mgs) - qidpv(mgs)));  // 20565
          chcni(mgs) = ((cx(mgs, P.li) * qhcni(mgs)) / qx(mgs, P.li));  // 20566
          chcnih(mgs) = c_min(chcni(mgs), ((rho0(mgs) * qhcni(mgs)) / ((r * P.xvmn(P.lh)))));  // 20568
          vhcni(mgs) = ((rho0(mgs) * qhcni(mgs)) / r);  // 20570
        }  // 20571
      }  // 20604
    }  // 20608
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlcnh(i_0) = NR(0.0);  // 20611
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlcnh(i_0) = NR(0.0);  // 20612
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlcnhhl(i_0) = NR(0.0);  // 20613
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlcnh(i_0) = NR(0.0);  // 20614
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlcnhl(i_0) = NR(0.0);  // 20615
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhlcnh(i_0) = NR(0.0);  // 20616
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhcnhl(i_0) = NR(0.0);  // 20618
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chcnhl(i_0) = NR(0.0);  // 20619
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhcnhl(i_0) = NR(0.0);  // 20620
    for (int i_0 = 1; i_0 <= ngs; ++i_0) zhcnhl(i_0) = NR(0.0);  // 20621
    if ((P.lhl > 1)) {  // 20624
      if (((P.ihlcnh == 1) || (P.ihlcnh == 3))) {  // 20626
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20631
          ltest = (xdia(mgs, P.lh, 3) > P.hlcnhdia);  // 20637
          if (((P.iusedw == 0) && (P.ihlcnh == 1))) {  // 20644
            dg0(mgs) = (-NR(1.0));  // 20645
          } else {  // 20646
            if (((temg(mgs) <= (P.tfr + P.hailcnvtoffset)) && (((((((((qhacw(mgs) + qhacr(mgs))) * dtp) > P.qxmin(P.lh)) && (qx(mgs, P.lh) > P.hlcnhqmin)) && (temg(mgs) > P.dwtempmin))) || ((wetgrowth(mgs) && (qx(mgs, P.lh) > P.hlcnhqmin))))))) {  // 20647
              x = ((((NR(1.1e4) * rho0(mgs)) * (((ehw(mgs) * qx(mgs, P.lc)) + (ehr(mgs) * qx(mgs, P.lr))))) - ((NR(1.3e3) * rho0(mgs)) * qx(mgs, P.li))) + NR(1.0));  // 20657
              if ((x > NR(1.0e-20))) {  // 20659
                arg = c_min(NR(70.0), ((-(temcg(mgs) / x))));  // 20660
                dwr = (NR(0.01) * ((exp(arg) - NR(1.0))));  // 20661
              } else {  // 20662
                dwr = NR(1.0e30);  // 20663
              }  // 20664
              d = c_min(dwr, (P.dg0thresh + NR(0.0001)));  // 20665
              if ((((dwr < NR(0.2)) && (dwr > NR(0.0))) && ((rho0(mgs) * ((qx(mgs, P.lc) + qx(mgs, P.lr)))) > NR(1.0e-4)))) {  // 20666
                sqrtrhovt = sqrt(rhovt(mgs));  // 20667
                fventh = ((sqrtrhovt * (pow(fpndl(mgs), ((NR(1.0) / NR(3.0)))))) * pow((fakvisc(mgs)), ((-NR(0.5)))));  // 20668
                fventm = ((sqrtrhovt * (pow(fschm(mgs), ((NR(1.0) / NR(3.0)))))) * pow((fakvisc(mgs)), ((-NR(0.5)))));  // 20669
                ltemq = ((((P.tfr - NR(163.15))) / P.fqsat) + NR(1.5));  // 20670
                qvs0 = (pqs(mgs) * P.tabqvs(ltemq));  // 20671
                denomdp = (felf(mgs) + (fcw(mgs) * temcg(mgs)));  // 20672
                denominvdp = (1.0e0 / ((felf(mgs) + (fcw(mgs) * temcg(mgs)))));  // 20673
                h1 = (((-(ftka(mgs) * temcg(mgs))) - (((felv(mgs) * fwvdf(mgs)) * rho0(mgs)) * ((qx(mgs, P.lv) - qvs0)))));  // 20676
                h2 = ((((ehi(mgs) * qx(mgs, P.li)) * rho0(mgs)) * fci(mgs)) * temcg(mgs));  // 20677
                h3 = (c_max(P.dwehwmin, ehw(mgs)) * qx(mgs, P.lc));  // 20678
                h4 = (ehr(mgs) * qx(mgs, P.lr));  // 20679
                for (n = 1; n <= 10; ++n) {  // 20681
                  d = c_max(d, NR(1.0e-4));  // 20682
                  dold = d;  // 20683
                  vth = (axx(mgs, P.lh) * pow(d, bxx(mgs, P.lh)));  // 20684
                  x2 = ((fventh * sqrtrhovt) * sqrt((d * vth)));  // 20685
                  if ((x2 > NR(1.4))) {  // 20686
                    ah = (NR(0.78) + (NR(0.308) * x2));  // 20687
                  } else {  // 20688
                    ah = (NR(1.0) + (NR(0.108) * ipow<2>(x2)));  // 20689
                  }  // 20690
                  d = (((NR(8.0) * ah) * h1) / (((((((c_max(NR(0.001), (vth - vtxbar(mgs, P.lc, 1))) * h3) + (c_max(NR(0.001), (vth - vtxbar(mgs, P.lr, 1))) * h4))) * rho0(mgs)) * denomdp) + (c_max(NR(0.001), (vth - vtxbar(mgs, P.li, 1))) * h2))));  // 20709
                  if ((((fabs((dold - d)) / dold) < NR(0.05)) || (((n > 3) && (d > P.dg0thresh))))) { break; }  // 20715
                }  // 20717
                d = c_min(d, (P.dg0thresh + NR(0.0001)));  // 20719
              }  // 20720
              dg0(mgs) = c_max(d, P.dwmin);  // 20724
            } else {  // 20725
              dg0(mgs) = (P.dg0thresh + NR(0.0001));  // 20729
            }  // 20731
            if ((((((P.ihlcnh == 3) && ((((qhacw(mgs) + qhacr(mgs))) * dtp) > P.qxmin(P.lh))) && (qx(mgs, P.lh) > P.hlcnhqmin)) && (temg(mgs) <= (P.tfr + P.hailcnvtoffset))) && (temg(mgs) > NR(238.0)))) {  // 20733
              dg0(mgs) = c_min(dg0(mgs), P.dwmax);  // 20737
            }  // 20738
          }  // 20740
          wtest = (((dg0(mgs) > NR(0.0)) && (dg0(mgs) < P.dg0thresh)));  // 20742
          if ((P.ihlcnh == 1)) {  // 20744
            if (((((((wetgrowth(mgs) && (((xdn(mgs, P.lh) > P.hldnmn) || (P.lvh < 1)))) && (rimdn(mgs, P.lh) > NR(800.0))) && ltest) && (qx(mgs, P.lh) > P.hlcnhqmin))) || wtest)) {  // 20746
              if ((((qhacw(mgs) > NR(0.0)) && (qhacw(mgs) > qhaci(mgs))) && (temg(mgs) <= (P.tfr + P.hailcnvtoffset)))) {  // 20751
                if (wtest) {  // 20755
                  dh0 = dg0(mgs);  // 20756
                } else {  // 20757
                  x = ((((NR(1.1e4) * ((rho0(mgs) * qx(mgs, P.lc)))) - ((NR(1.3e3) * rho0(mgs)) * qx(mgs, P.li))) + NR(1.0e-3)));  // 20758
                  if ((x > NR(1.0e-20))) {  // 20759
                    arg = c_min(NR(70.0), ((-(temcg(mgs) / x))));  // 20760
                    dh0 = (NR(0.01) * ((exp(arg) - NR(1.0))));  // 20761
                  } else {  // 20762
                    dh0 = NR(1.0e30);  // 20763
                  }  // 20764
                  dg0(mgs) = c_min(dh0, (P.dg0thresh + NR(0.0001)));  // 20765
                }  // 20766
                if (((xdia(mgs, P.lh, 3) / dh0) > NR(0.1))) {  // 20771
                  tmp = (((qhacw(mgs) + qhacr(mgs)) + qhaci(mgs)) + qhacs(mgs));  // 20773
                  qtmp = (c_min(NR(100.0), (xdia(mgs, P.lh, 3) / ((NR(2.0) * dh0)))) * (tmp));  // 20775
                  qhlcnh(mgs) = c_min(qxmxd(mgs, P.lh), qtmp);  // 20776
                  if ((P.ipconc >= 5)) {  // 20778
                    if ((!wtest)) { dh0 = c_min(dh0, NR(10.0e-3)); }  // 20780
                    if ((qx(mgs, P.lhl) > NR(0.1e-3))) { dh0 = c_max(dh0, xdia(mgs, P.lhl, 3)); }  // 20781
                    chlcnhhl(mgs) = c_min(cxmxd(mgs, P.lh), ((rho0(mgs) * qhlcnh(mgs)) / ((((P.pi * xdn(mgs, P.lh)) * ipow<3>(dh0)) / NR(6.0)))));  // 20782
                    r = ((rho0(mgs) * qhlcnh(mgs)) / ((xdn(mgs, P.lh) * xv(mgs, P.lh))));  // 20784
                    chlcnh(mgs) = c_max(chlcnhhl(mgs), r);  // 20785
                  }  // 20786
                  vhlcnh(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / xdn(mgs, P.lh));  // 20788
                  vhlcnhl(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / c_max(xdnmn(P.lhl), xdn(mgs, P.lh)));  // 20789
                }  // 20791
              }  // 20793
            }  // 20794
          } else if ((P.ihlcnh == 3)) {  // 20796
            if ((wtest && (((((qhacw(mgs) * dtp) > P.qxmin(P.lh)) && (temg(mgs) < (P.tfr + P.hailcnvtoffset))) && (qx(mgs, P.lh) > P.hlcnhqmin))))) {  // 20799
              if ((P.ipconc == 5)) {  // 20802
              }  // 20805
              ratio = c_min(P.maxratiolu, (dg0(mgs) / xdia(mgs, P.lh, 1)));  // 20807
              tmp2 = gaminterp(P, ratio, alpha(mgs, P.lh), 4, 1);  // 20811
              if ((P.ipconc == 5)) {  // 20812
              }  // 20814
              qxd1 = (qx(mgs, P.lh) * (tmp2));  // 20815
              qhlcnh(mgs) = (dtpinv * qxd1);  // 20816
              flim = NR(1.0);  // 20817
              tmp3 = qxmxd(mgs, P.lh);  // 20818
              if ((qxd1 > tmp3)) {  // 20819
              }  // 20822
              if (((((qxd1 > P.qxmin(P.lhl)) && (P.ipconc > 5))) || (((qxd1 > (NR(10.0) * P.qxmin(P.lhl))) && (P.ipconc == 5))))) {  // 20826
                tmp = gaminterp(P, ratio, alpha(mgs, P.lh), 1, 1);  // 20829
                if ((P.ipconc == 5)) {  // 20830
                }  // 20832
                cxd1 = ((flim * cx(mgs, P.lh)) * (tmp));  // 20833
                chlcnh(mgs) = (dtpinv * cxd1);  // 20834
                chlcnhhl(mgs) = chlcnh(mgs);  // 20835
                zxd1 = 0;  // 20865
                if ((P.ipconc == 5)) {  // 20867
                  tmp3 = gaminterp(P, ratio, alpha(mgs, P.lh), 11, 1);  // 20868
                  tmp5 = ((g1x(mgs, P.lh) * ipow<2>(((rho0(mgs) * qx(mgs, P.lh))))) / ((ipow<2>((((P.pi * xdn(mgs, P.lh)) / NR(6.0)))) * cx(mgs, P.lh))));  // 20870
                  zxd1 = ((flim * (tmp3)) * tmp5);  // 20871
                  tmp3 = ((g1x(mgs, P.lh) * ipow<2>(((rho0(mgs) * qxd1)))) / (ipow<2>((((P.pi * xdn(mgs, P.lh)) / NR(6.0))))));  // 20874
                  tmp4 = (tmp3 / cxd1);  // 20875
                  if ((tmp4 > zxd1)) {  // 20876
                    cxd1 = (tmp3 / zxd1);  // 20884
                    chlcnhhl(mgs) = (dtpinv * cxd1);  // 20885
                  }  // 20886
                }  // 20887
              } else {  // 20890
                qhlcnh(mgs) = NR(0.0);  // 20891
              }  // 20892
              vhlcnh(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / xdn(mgs, P.lh));  // 20894
              vhlcnhl(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / c_max(xdnmn(P.lhl), xdn(mgs, P.lh)));  // 20895
            }  // 20897
          }  // 20900
        }  // 20902
      } else if ((P.ihlcnh == 2)) {  // 20904
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20914
          if (((((qhacw(mgs) * dtp) > P.qxmin(P.lh)) && (temg(mgs) < (P.tfr + P.hailcnvtoffset))) && (qx(mgs, P.lh) > P.qxmin(P.lh)))) {  // 20915
            ratio = c_min(P.maxratiolu, (P.hldia1 / xdia(mgs, P.lh, 1)));  // 20916
            tmp = gaminterp(P, ratio, alpha(mgs, P.lh), 1, 1);  // 20919
            cxd1 = (cx(mgs, P.lh) * (tmp));  // 20920
            chlcnh(mgs) = (dtpinv * cxd1);  // 20921
            chlcnhhl(mgs) = chlcnh(mgs);  // 20922
            tmp2 = gaminterp(P, ratio, alpha(mgs, P.lh), 4, 1);  // 20925
            qxd1 = (qx(mgs, P.lh) * (tmp2));  // 20926
            qhlcnh(mgs) = (dtpinv * qxd1);  // 20927
            zxd1 = 0;  // 20935
            vhlcnh(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / xdn(mgs, P.lh));  // 20937
            vhlcnhl(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / c_max(xdnmn(P.lhl), xdn(mgs, P.lh)));  // 20938
          }  // 20940
        }  // 20942
      } else if ((P.ihlcnh == 0)) {  // 20944
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20946
          if (((wetgrowth(mgs) && (temg(mgs) < (P.tfr - NR(5.0)))) && (qx(mgs, P.lh) > P.qxmin(P.lh)))) {  // 20949
            if (((qhacw(mgs) > NR(1.0e-6)) && (((xdn(mgs, P.lh) > NR(700.0)) || (P.lvh == 0))))) {  // 20950
              qhlcnh(mgs) = (((((((P.pi * xdn(mgs, P.lh)) * cx(mgs, P.lh))) / (((NR(6.0) * rho0(mgs)) * dtp)))) * exp((-(P.hldia1 / xdia(mgs, P.lh, 1))))) * (((((ipow<3>(P.hldia1)) + ((NR(3.0) * (ipow<2>(P.hldia1))) * xdia(mgs, P.lh, 1))) + ((NR(6.0) * (P.hldia1)) * (ipow<2>(xdia(mgs, P.lh, 1))))) + (NR(6.0) * (ipow<3>(xdia(mgs, P.lh, 1)))))));  // 20951
              qhlcnh(mgs) = c_min(qhlcnh(mgs), qhmxd(mgs));  // 20956
              if ((P.ipconc >= 5)) {  // 20957
                chlcnh(mgs) = c_min(cxmxd(mgs, P.lh), (cx(mgs, P.lh) * exp((-(P.hldia1 / xdia(mgs, P.lh, 1))))));  // 20958
                chlcnhhl(mgs) = chlcnh(mgs);  // 20959
              }  // 20961
              vhlcnh(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / xdn(mgs, P.lh));  // 20962
              vhlcnhl(mgs) = ((rho0(mgs) * qhlcnh(mgs)) / c_max(xdnmn(P.lhl), xdn(mgs, P.lh)));  // 20963
            }  // 20964
          }  // 20965
        }  // 20966
      }  // 20970
    }  // 20986
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 20994
      qhcns(mgs) = NR(0.0);  // 20996
      chcns(mgs) = NR(0.0);  // 20997
      chcnsh(mgs) = NR(0.0);  // 20998
      vhcns(mgs) = NR(0.0);  // 20999
      qscnh(mgs) = NR(0.0);  // 21001
      cscnh(mgs) = NR(0.0);  // 21002
      vscnh(mgs) = NR(0.0);  // 21003
      if ((P.ipconc >= 5)) {  // 21005
        if (((((temg(mgs) < P.tfr) && (qx(mgs, P.lh) > P.qxmin(P.lh))) && (qhdpv(mgs) > (P.qxmin(P.lh) * dtpinv))) && (qhacw(mgs) < (P.qxmin(P.lh) * dtpinv)))) {  // 21008
          if ((xdn(mgs, P.lh) < NR(290.0))) {  // 21010
          }  // 21014
        }  // 21015
        if (((qx(mgs, P.ls) > P.qxmin(P.ls)) && (qsacw(mgs) > NR(0.0)))) {  // 21018
          if (((temg(mgs) < NR(273.0)) && ((((qsacw(mgs) - qsdpv(mgs)) > NR(0.0)) || ((((P.iglcnvs >= 3) && ((qsacw(mgs) * dtp) > (NR(2.0) * P.qxmin(P.lh)))) && ((gamsnow73fac * xmas(mgs, P.ls)) > (xdnmn(P.lh) * P.xvmn(P.lh))))))))) {  // 21059
            tmp = (P.rimc1 * pow(((-((((((NR(0.5)) * (NR(1.0e+06))) * xdia(mgs, P.lc, 1))) * (((NR(0.60)) * vtxbar(mgs, P.ls, 1)))) / ((temg(mgs) - NR(273.15)))))), (P.rimc2)));  // 21063
            tmp = c_min(tmp, NR(900.0));  // 21067
            if ((tmp >= NR(200.0))) {  // 21076
              r = c_max((NR(0.5) * ((xdn(mgs, P.ls) + tmp))), xdnmn(P.lh));  // 21077
              qhcns(mgs) = ((qsacw(mgs) - qsdpv(mgs)));  // 21079
              chcns(mgs) = ((cx(mgs, P.ls) * qhcns(mgs)) / qx(mgs, P.ls));  // 21080
              chcnsh(mgs) = c_min(chcns(mgs), ((rho0(mgs) * qhcns(mgs)) / ((r * P.xvmn(P.lh)))));  // 21082
              vhcns(mgs) = ((rho0(mgs) * qhcns(mgs)) / r);  // 21084
            }  // 21085
          }  // 21104
        }  // 21109
      } else {  // 21111
        qhcns(mgs) = ((NR(0.001) * ehscnv(mgs)) * c_max(((qx(mgs, P.ls) - NR(6.0e-4))), NR(0.0)));  // 21113
        qhcns(mgs) = c_min(qhcns(mgs), qxmxd(mgs, P.ls));  // 21114
        if ((P.lvol(P.lh) >= 1)) { vhcns(mgs) = ((rho0(mgs) * qhcns(mgs)) / c_max(xdn(mgs, P.ls), NR(400.0))); }  // 21115
      }  // 21117
    }  // 21118
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21127
      qrztot(mgs) = ((qrfrz(mgs) + qiacr(mgs)) + qsacr(mgs));  // 21131
      qrzmax(mgs) = ((((xdia(mgs, P.lr, 1) * rwvent(mgs)) * cx(mgs, P.lr)) * fwet1(mgs)));  // 21136
      qrzmax(mgs) = c_max(qrzmax(mgs), NR(0.0));  // 21138
      qrzmax(mgs) = c_min(qrztot(mgs), qrzmax(mgs));  // 21139
      qrzmax(mgs) = c_min((qx(mgs, P.lr) * dtpinv), qrzmax(mgs));  // 21140
      if ((temcg(mgs) < (-NR(30.0)))) {  // 21142
        qrzmax(mgs) = (qx(mgs, P.lr) * dtpinv);  // 21143
      }  // 21144
      if (((qrztot(mgs) > qrzmax(mgs)) && (qrztot(mgs) > P.qxmin(P.lr)))) {  // 21150
        qrzfac(mgs) = (qrzmax(mgs) / (qrztot(mgs)));  // 21151
      } else {  // 21152
        qrzfac(mgs) = NR(1.0);  // 21153
      }  // 21154
      qrzfac(mgs) = c_min(NR(1.0), qrzfac(mgs));  // 21155
    }  // 21157
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21163
      if (((temg(mgs) <= NR(273.15)) && (qrzfac(mgs) < NR(1.0)))) {  // 21164
        qrfrz(mgs) = (qrzfac(mgs) * qrfrz(mgs));  // 21165
        qrfrzs(mgs) = (qrzfac(mgs) * qrfrzs(mgs));  // 21166
        qrfrzf(mgs) = (qrzfac(mgs) * qrfrzf(mgs));  // 21167
        qiacr(mgs) = (qrzfac(mgs) * qiacr(mgs));  // 21168
        qsacr(mgs) = (qrzfac(mgs) * qsacr(mgs));  // 21169
        qiacrf(mgs) = (qrzfac(mgs) * qiacrf(mgs));  // 21170
        qiacrs(mgs) = (qrzfac(mgs) * qiacrs(mgs));  // 21171
        crfrz(mgs) = (qrzfac(mgs) * crfrz(mgs));  // 21172
        crfrzf(mgs) = (qrzfac(mgs) * crfrzf(mgs));  // 21173
        crfrzs(mgs) = (qrzfac(mgs) * crfrzs(mgs));  // 21174
        ciacr(mgs) = (qrzfac(mgs) * ciacr(mgs));  // 21175
        ciacrf(mgs) = (qrzfac(mgs) * ciacrf(mgs));  // 21176
        ciacrs(mgs) = (qrzfac(mgs) * ciacrs(mgs));  // 21177
        vrfrzf(mgs) = (qrzfac(mgs) * vrfrzf(mgs));  // 21184
        viacrf(mgs) = (qrzfac(mgs) * viacrf(mgs));  // 21185
      }  // 21186
    }  // 21187
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qrcev(i_0) = NR(0.0);  // 21199
    for (int i_0 = 1; i_0 <= ngs; ++i_0) crcev(i_0) = NR(0.0);  // 21200
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21203
      if ((qx(mgs, P.lr) > P.qxmin(P.lr))) {  // 21205
        qrcev(mgs) = ((((fvce(mgs) * cx(mgs, P.lr)) * rwvent(mgs)) * rwcap(mgs)) * P.evapfac);  // 21207
        qrcev(mgs) = c_min(qrcev(mgs), NR(0.0));  // 21214
        qrcev(mgs) = c_max(qrcev(mgs), (-qrmxd(mgs)));  // 21217
        if (((qrcev(mgs) < NR(0.0)) && (P.lnr > 1))) {  // 21219
          crcev(mgs) = (((cx(mgs, P.lr) / (qx(mgs, P.lr)))) * qrcev(mgs));  // 21223
        } else {  // 21229
          crcev(mgs) = NR(0.0);  // 21230
        }  // 21231
      }  // 21234
    }  // 21236
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chmul1(i_0) = NR(0.0);  // 21257
    for (int i_0 = 1; i_0 <= ngs; ++i_0) chlmul1(i_0) = NR(0.0);  // 21258
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csmul1(i_0) = NR(0.0);  // 21259
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhmul1(i_0) = NR(0.0);  // 21261
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qhlmul1(i_0) = NR(0.0);  // 21262
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qsmul1(i_0) = NR(0.0);  // 21263
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21264
      ltest = (qx(mgs, P.lh) > P.qxmin(P.lh));  // 21266
      if ((P.lhl > 1)) { ltest = (ltest || (qx(mgs, P.lhl) > P.qxmin(P.lhl))); }  // 21267
      if (((((P.itype1 >= 1) || (P.itype2 >= 1))) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 21269
        if (((temg(mgs) >= NR(265.15)) && (temg(mgs) <= NR(271.15)))) {  // 21271
          if ((P.ipconc >= 2)) {  // 21272
            if (((xv(mgs, P.lc) > NR(0.0)) && ltest)) {  // 21273
              if ((alpha(mgs, P.lc) == NR(0.0))) {  // 21280
                ex1 = (((NR(1.0) / NR(250.0))) * exp((-(NR(7.23e-15) / xv(mgs, P.lc)))));  // 21281
              } else {  // 21282
                ratio = ((((NR(1.0) + alpha(mgs, P.lc))) * (NR(7.23e-15))) / xv(mgs, P.lc));  // 21284
                ratio = c_min(P.maxratiolu, ratio);  // 21291
                tmp = gaminterp(P, ratio, alpha(mgs, P.lc), 1, 1);  // 21292
                ex1 = (((NR(1.0) / NR(250.0))) * tmp);  // 21293
              }  // 21295
              ft = c_max(NR(0.0), c_min(NR(1.0), (((-(NR(0.11) * ipow<2>(temcg(mgs)))) - (NR(1.1) * temcg(mgs))) - NR(1.7))));  // 21297
              if ((ft > NR(0.0))) {  // 21312
                if (((qx(mgs, P.lh) > P.qxmin(P.lh)) && ((!wetsfc(mgs))))) {  // 21315
                  chmul1(mgs) = ((ft * ex1) * chacw(mgs));  // 21316
                  qhmul1(mgs) = ((P.cimas0 * chmul1(mgs)) * rhoinv(mgs));  // 21318
                }  // 21319
                if ((P.lhl > 1)) {  // 21320
                  if (((qx(mgs, P.lhl) > P.qxmin(P.lhl)) && ((!wetsfchl(mgs))))) {  // 21321
                    chlmul1(mgs) = (((ft * ex1) * chlacw(mgs)));  // 21322
                    qhlmul1(mgs) = ((P.cimas0 * chlmul1(mgs)) * rhoinv(mgs));  // 21323
                  }  // 21324
                }  // 21325
              }  // 21344
            }  // 21346
          } else {  // 21348
            fimt1(mgs) = NR(0.0);  // 21352
            if (((temg(mgs) >= NR(268.15)) && (temg(mgs) <= NR(270.15)))) {  // 21356
              fimt1(mgs) = (NR(1.0) - (((temg(mgs) - NR(268.15))) / NR(2.0)));  // 21357
            } else if (((temg(mgs) <= NR(268.15)) && (temg(mgs) >= NR(265.15)))) {  // 21358
              fimt1(mgs) = (NR(1.0) + (((temg(mgs) - NR(268.15))) / NR(3.0)));  // 21359
            } else {  // 21360
              fimt1(mgs) = NR(0.0);  // 21361
            }  // 21362
            if (((temg(mgs) >= NR(265.15)) && (temg(mgs) <= NR(267.15)))) {  // 21366
              fimt1(mgs) = NR(0.5);  // 21367
            } else if (((temg(mgs) >= NR(267.15)) && (temg(mgs) <= NR(269.15)))) {  // 21368
              fimt1(mgs) = NR(1.0);  // 21369
            } else if (((temg(mgs) >= NR(269.15)) && (temg(mgs) <= NR(271.15)))) {  // 21370
              fimt1(mgs) = NR(0.5);  // 21371
            } else {  // 21372
              fimt1(mgs) = NR(0.0);  // 21373
            }  // 21374
            fimta(mgs) = NR(0.0);  // 21384
            fimt2(mgs) = NR(0.0);  // 21394
            xcwmas = (xmas(mgs, P.lc) * NR(1000.0));  // 21395
            if ((xcwmas < NR(1.26e-9))) {  // 21398
              fimt2(mgs) = NR(0.0);  // 21399
            }  // 21400
            if (((xcwmas <= NR(3.55e-9)) && (xcwmas >= NR(1.26e-9)))) {  // 21401
              fimt2(mgs) = (((NR(2.27)) * log(xcwmas)) + NR(13.39));  // 21402
            }  // 21403
            if ((xcwmas > NR(3.55e-9))) {  // 21404
              fimt2(mgs) = NR(1.0);  // 21405
            }  // 21406
            fimt2(mgs) = c_min(fimt2(mgs), NR(1.0));  // 21408
            fimt2(mgs) = c_max(fimt2(mgs), NR(0.0));  // 21409
            if ((!wetsfc(mgs))) {  // 21423
              chmul1(mgs) = ((fimt1(mgs) * ((fimta(mgs) + ((NR(4.0e-03)) * fimt2(mgs))))) * qhacw(mgs));  // 21424
            }  // 21426
            qhmul1(mgs) = (chmul1(mgs) * ((P.cimas0 / rho0(mgs))));  // 21428
            if ((P.lhl > 1)) {  // 21430
              if (((qx(mgs, P.lhl) > P.qxmin(P.lhl)) && ((!wetsfchl(mgs))))) {  // 21431
                tmp = ((fimt1(mgs) * ((fimta(mgs) + ((NR(4.0e-03)) * fimt2(mgs))))) * qhlacw(mgs));  // 21432
                chlmul1(mgs) = tmp;  // 21434
                qhlmul1(mgs) = ((P.cimas0 * tmp) * rhoinv(mgs));  // 21435
              }  // 21436
            }  // 21437
          }  // 21441
        }  // 21443
      }  // 21445
    }  // 21447
    for (int i_0 = 1; i_0 <= ngs; ++i_0) csmul(i_0) = NR(0.0);  // 21460
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qsmul(i_0) = NR(0.0);  // 21461
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21488
      qracif(mgs) = qraci(mgs);  // 21489
      cracif(mgs) = craci(mgs);  // 21490
    }  // 21492
    cmassin = cimasn;  // 21516
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21517
      qiint(mgs) = NR(0.0);  // 21518
      ciint(mgs) = NR(0.0);  // 21519
      qicicnt(mgs) = NR(0.0);  // 21520
      cicint(mgs) = NR(0.0);  // 21521
      qipipnt(mgs) = NR(0.0);  // 21522
      cipint(mgs) = NR(0.0);  // 21523
      ccitmp = NR(0.0);  // 21524
      if (((((temg(mgs) < NR(268.15)) || (((P.imeyers5 && (temg(mgs) < NR(272.0))) && (temgkm2(mgs) < P.tfr))))) && (P.ciintmx > ((cx(mgs, P.li) + ccitmp))))) {  // 21526
        fiinit(mgs) = ((ipow<2>(felv(mgs))) / ((P.cp * P.rw)));  // 21532
        dqisdt(mgs) = (((qx(mgs, P.lv) - qis(mgs))) / ((NR(1.0) + ((fiinit(mgs) * qis(mgs)) / tsqr(mgs)))));  // 21533
        idqis = 0;  // 21536
        if ((ssi(mgs) > NR(1.0))) {  // 21537
          idqis = 1;  // 21538
          dzfacp = c_max(NR((kgsp(mgs) - kgs(mgs))), NR(0.0));  // 21539
          dzfacm = c_max(NR((kgs(mgs) - kgsm(mgs))), NR(0.0));  // 21540
          qiint(mgs) = ((((((idqis * il5(mgs)) * ((cmassin / rho0(mgs)))) * c_max(NR(0.0), wvel(mgs))) * c_max(((cninp(mgs) - cninm(mgs))), NR(0.0))) / gz(igs(mgs), jgs, kgs(mgs))) / (((dzfacp + dzfacm))));  // 21541
          qiint(mgs) = c_min(qiint(mgs), c_max((NR(0.25) * dqisdt(mgs)), NR(0.0)));  // 21548
          ciint(mgs) = ((qiint(mgs) * rho0(mgs)) / cmassin);  // 21549
          if (((P.icenucopt == 1) && (ciint(mgs) > (c_max(NR(0.0), ((P.ciintmx - cx(mgs, P.li)) - ccitmp)) * dtpinv)))) {  // 21567
            ciint(mgs) = (c_max(NR(0.0), (P.ciintmx - (cx(mgs, P.li)))) * dtpinv);  // 21568
            qiint(mgs) = ((ciint(mgs) * cmassin) / rho0(mgs));  // 21569
          }  // 21575
        }  // 21578
      }  // 21579
      if ((xplate(mgs) == 1)) {  // 21617
        qipipnt(mgs) = qiint(mgs);  // 21618
        cipint(mgs) = ciint(mgs);  // 21619
      }  // 21620
      if ((xcolmn(mgs) == 1)) {  // 21622
        qicicnt(mgs) = qiint(mgs);  // 21623
        cicint(mgs) = ciint(mgs);  // 21624
      }  // 21625
    }  // 21630
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21659
      qrshr(mgs) = NR(0.0);  // 21660
      qwshw(mgs) = NR(0.0);  // 21661
      cwshw(mgs) = NR(0.0);  // 21662
      qsshrp(mgs) = NR(0.0);  // 21663
      qhshrp(mgs) = NR(0.0);  // 21664
    }  // 21665
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21671
      qrshr(mgs) = ((qsshr(mgs) + qhshr(mgs)) + qhlshr(mgs));  // 21672
      crshr(mgs) = ((chshrr(mgs) / rzxh(mgs)) + (chlshrr(mgs) / rzxhl(mgs)));  // 21673
      if ((P.ipconc >= 3)) {  // 21676
      }  // 21678
    }  // 21679
    if ((P.ipconc >= 1)) {  // 21689
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccwi(i_0) = NR(0.0);  // 21698
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccwd(i_0) = NR(0.0);  // 21699
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccwdacc(i_0) = NR(0.0);  // 21700
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccii(i_0) = NR(0.0);  // 21701
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccin(i_0) = NR(0.0);  // 21702
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pccid(i_0) = NR(0.0);  // 21703
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcisi(i_0) = NR(0.0);  // 21704
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcisd(i_0) = NR(0.0);  // 21705
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcrwi(i_0) = NR(0.0);  // 21706
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcrwd(i_0) = NR(0.0);  // 21707
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcswi(i_0) = NR(0.0);  // 21708
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pcswd(i_0) = NR(0.0);  // 21709
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pchwi(i_0) = NR(0.0);  // 21710
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pchwd(i_0) = NR(0.0);  // 21711
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pchli(i_0) = NR(0.0);  // 21712
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pchld(i_0) = NR(0.0);  // 21713
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21723
        pccii(mgs) = (((((((il5(mgs) * cicint(mgs)) + (il5(mgs) * ((((((NR(1.0) - P.cwfrz2snowfrac)) * cwfrzc(mgs)) + cwctfzc(mgs)) + cicichr(mgs))))) + chmul1(mgs)) + chlmul1(mgs)) + csplinter(mgs)) + csplinter2(mgs)) + csmul(mgs));  // 21724
        pccii(mgs) = (pccii(mgs) * ((NR(1.0) - P.ffrzs)));  // 21733
        pccid(mgs) = (((il5(mgs) * ((((((((-cscni(mgs)) - cscnvi(mgs)) - craci(mgs)) - csaci(mgs)) - chaci(mgs)) - chlaci(mgs)) - chcni(mgs)))) + (il5(mgs) * cisbv(mgs))) - (((NR(1.0) - il5(mgs))) * cimlr(mgs)));  // 21736
        pccin(mgs) = ciint(mgs);  // 21745
      }  // 21748
      if ((P.ipconc >= 2)) {  // 21786
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21788
          pccwi(mgs) = ((NR(0.0)) - cwshw(mgs));  // 21789
          pccwd(mgs) = ((((((-cautn(mgs)) + (il5(mgs) * (((((-ciacw(mgs)) - cwfrz(mgs)) - cwctfzp(mgs)) - cwctfzc(mgs))))) - cracw(mgs)) - csacw(mgs)) - chacw(mgs)) - chlacw(mgs));  // 21792
          if (((-(pccwd(mgs) * dtp)) > cx(mgs, P.lc))) {  // 21861
            frac = (-(cx(mgs, P.lc) / ((pccwd(mgs) * dtp))));  // 21868
            pccwd(mgs) = (-(cx(mgs, P.lc) * dtpinv));  // 21869
            ciacw(mgs) = (frac * ciacw(mgs));  // 21871
            cwfrz(mgs) = (frac * cwfrz(mgs));  // 21872
            cwfrzp(mgs) = (frac * cwfrzp(mgs));  // 21873
            cwctfzp(mgs) = (frac * cwctfzp(mgs));  // 21874
            cwfrzc(mgs) = (frac * cwfrzc(mgs));  // 21875
            cwctfzc(mgs) = (frac * cwctfzc(mgs));  // 21876
            cwctfz(mgs) = (frac * cwctfz(mgs));  // 21877
            cracw(mgs) = (frac * cracw(mgs));  // 21878
            csacw(mgs) = (frac * csacw(mgs));  // 21879
            chacw(mgs) = (frac * chacw(mgs));  // 21880
            cautn(mgs) = (frac * cautn(mgs));  // 21881
            pccii(mgs) = (pccii(mgs) - (((((NR(1.0) - frac)) * il5(mgs)) * ((cwfrzc(mgs) + cwctfzc(mgs)))) * ((NR(1.0) - P.ffrzs))));  // 21883
            if ((P.lhl > 1)) { chlacw(mgs) = (frac * chlacw(mgs)); }  // 21884
          }  // 21888
        }  // 21890
      }  // 21892
      if ((P.ipconc >= 3)) {  // 21897
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21899
          pcrwi(mgs) = (((crcnw(mgs) + (((1 - il5(mgs))) * (((((-(chmlrr(mgs) / rzxh(mgs))) - (chlmlrr(mgs) / rzxhl(mgs))) - csmlrr(mgs)) - cimlr(mgs))))) - c_min(NR(0.0), cracr(mgs))) - crshr(mgs));  // 21902
          pcrwd(mgs) = (((((il5(mgs) * (((-ciacr(mgs)) - crfrz(mgs)))) - chacr(mgs)) - chlacr(mgs)) + crcev(mgs)) - c_max(NR(0.0), cracr(mgs)));  // 21913
          frac = NR(0.0);  // 21952
          if (((-(pcrwd(mgs) * dtp)) > cx(mgs, P.lr))) {  // 21953
            frac = (-(cx(mgs, P.lr) / ((pcrwd(mgs) * dtp))));  // 21961
            pcrwd(mgs) = (-(cx(mgs, P.lr) * dtpinv));  // 21962
            ciacr(mgs) = (frac * ciacr(mgs));  // 21964
            ciacrf(mgs) = (frac * ciacrf(mgs));  // 21965
            ciacrs(mgs) = (frac * ciacrs(mgs));  // 21966
            crfrz(mgs) = (frac * crfrz(mgs));  // 21967
            crfrzf(mgs) = (frac * crfrzf(mgs));  // 21968
            crfrzs(mgs) = (frac * crfrzs(mgs));  // 21969
            chacr(mgs) = (frac * chacr(mgs));  // 21970
            chlacr(mgs) = (frac * chlacr(mgs));  // 21971
            crcev(mgs) = (frac * crcev(mgs));  // 21972
            cracr(mgs) = (frac * cracr(mgs));  // 21973
          }  // 21976
        }  // 21978
      }  // 21980
      if ((P.ipconc >= 4)) {  // 21988
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 21990
          pcswi(mgs) = (((il5(mgs) * ((cscnis(mgs) + cscnvis(mgs)))) + ((P.cwfrz2snowfrac * cwfrz(mgs)) / P.cwfrz2snowratio)) + cscnh(mgs));  // 21991
          pcswd(mgs) = (((((((-chacs(mgs)) - chlacs(mgs)) - chcns(mgs)) + (((1 - il5(mgs))) * csmlr(mgs))) + csshr(mgs)) + cssbv(mgs)) - csacs(mgs));  // 22012
          frac = NR(0.0);  // 22021
          if (((cx(mgs, P.ls) + (dtp * ((pcswi(mgs) + pcswd(mgs))))) < NR(0.0))) {  // 22023
            frac = ((((-cx(mgs, P.ls)) + (pcswi(mgs) * dtp))) / ((pcswd(mgs) * dtp)));  // 22024
            pcswd(mgs) = (frac * pcswd(mgs));  // 22026
            chacs(mgs) = (frac * chacs(mgs));  // 22028
            chlacs(mgs) = (frac * chlacs(mgs));  // 22029
            chcns(mgs) = (frac * chcns(mgs));  // 22030
            csmlr(mgs) = (frac * csmlr(mgs));  // 22031
            csshr(mgs) = (frac * csshr(mgs));  // 22032
            cssbv(mgs) = (frac * cssbv(mgs));  // 22033
            csacs(mgs) = (frac * csacs(mgs));  // 22034
          }  // 22036
          pccii(mgs) = ((pccii(mgs) + (((NR(1.0) - P.ifrzs)) * crfrzs(mgs))) + (((NR(1.0) - P.ifrzs)) * ciacrs(mgs)));  // 22041
          pcswi(mgs) = ((pcswi(mgs) + ((P.ifrzs) * crfrzs(mgs))) + ((P.ifrzs) * ciacrs(mgs)));  // 22045
        }  // 22049
      }  // 22051
      if ((P.ipconc >= 5)) {  // 22056
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22057
          pchwi(mgs) = (((((((ffrzh * P.ifrzg) * crfrzf(mgs)) + (((il5(mgs) * ffrzh) * P.ifiacrg) * (ciacrf(mgs))))) + (P.f2h * chcnsh(mgs))) + (P.f2h * chcnih(mgs))) + chcnhl(mgs));  // 22058
          pchwd(mgs) = ((((((1 - il5(mgs))) * chmlr(mgs)) + chsbv(mgs)) - (il5(mgs) * chlcnh(mgs))) - cscnh(mgs));  // 22063
        }  // 22070
        if (((P.lhl > 1) && (P.lnhl > 1))) {  // 22079
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22080
            pchli(mgs) = (((((ffrzh * ((NR(1.0) - P.ifrzg))) * crfrzf(mgs)) + (((il5(mgs) * ffrzh) * ((NR(1.0) - P.ifiacrg))) * (ciacrf(mgs))))) + (chlcnhhl(mgs) * rzxhlh(mgs)));  // 22081
            pchld(mgs) = (((((1 - il5(mgs))) * chlmlr(mgs)) + chlsbv(mgs)) - chcnhl(mgs));  // 22084
            frac = NR(0.0);  // 22090
            if (((cx(mgs, P.lhl) + (dtp * ((pchli(mgs) + pchld(mgs))))) < NR(0.0))) {  // 22091
              frac = ((((-cx(mgs, P.lhl)) + (pchli(mgs) * dtp))) / ((pchld(mgs) * dtp)));  // 22094
              chlmlr(mgs) = (frac * chlmlr(mgs));  // 22096
              chlsbv(mgs) = (frac * chlsbv(mgs));  // 22097
              chcnhl(mgs) = (frac * chcnhl(mgs));  // 22098
              pchld(mgs) = (frac * pchld(mgs));  // 22100
            }  // 22102
          }  // 22105
        }  // 22107
      }  // 22110
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22155
        pctot(mgs) = (((((((((((pccwi(mgs) + pccwd(mgs)) + pccii(mgs)) + pccid(mgs)) + pcrwi(mgs)) + pcrwd(mgs)) + pcswi(mgs)) + pcswd(mgs)) + pchwi(mgs)) + pchwd(mgs)) + pchli(mgs)) + pchld(mgs));  // 22156
      }  // 22162
    }  // 22165
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqwvi(i_0) = NR(0.0);  // 22175
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqwvd(i_0) = NR(0.0);  // 22176
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqcwi(i_0) = NR(0.0);  // 22177
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqcwd(i_0) = NR(0.0);  // 22178
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqcwdacc(i_0) = NR(0.0);  // 22179
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqcii(i_0) = NR(0.0);  // 22180
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqcid(i_0) = NR(0.0);  // 22181
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqrwi(i_0) = NR(0.0);  // 22182
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqrwd(i_0) = NR(0.0);  // 22183
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqswi(i_0) = NR(0.0);  // 22184
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqswd(i_0) = NR(0.0);  // 22185
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqhwi(i_0) = NR(0.0);  // 22186
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqhwd(i_0) = NR(0.0);  // 22187
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqhli(i_0) = NR(0.0);  // 22188
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqhld(i_0) = NR(0.0);  // 22189
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwsi(i_0) = NR(0.0);  // 22190
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwsd(i_0) = NR(0.0);  // 22191
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwhi(i_0) = NR(0.0);  // 22192
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwhd(i_0) = NR(0.0);  // 22193
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwlghi(i_0) = NR(0.0);  // 22194
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwlghd(i_0) = NR(0.0);  // 22195
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwlghli(i_0) = NR(0.0);  // 22196
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwlghld(i_0) = NR(0.0);  // 22197
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwhli(i_0) = NR(0.0);  // 22198
    for (int i_0 = 1; i_0 <= ngs; ++i_0) pqlwhld(i_0) = NR(0.0);  // 22199
    if ((P.ipconc > 5)) {  // 22200
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzhwi(i_0) = NR(0.0);  // 22201
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzhwd(i_0) = NR(0.0);  // 22202
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzrwi(i_0) = NR(0.0);  // 22203
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzrwd(i_0) = NR(0.0);  // 22204
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzhli(i_0) = NR(0.0);  // 22205
      for (int i_0 = 1; i_0 <= ngs; ++i_0) pzhld(i_0) = NR(0.0);  // 22206
    }  // 22207
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22214
      pqwvi(mgs) = ((((((((-c_min(NR(0.0), qrcev(mgs))) - c_min(NR(0.0), qhcev(mgs))) - c_min(NR(0.0), qhlcev(mgs))) - c_min(NR(0.0), qscev(mgs))) - qhsbv(mgs)) - qhlsbv(mgs)) - qssbv(mgs)) - (il5(mgs) * qisbv(mgs)));  // 22217
      pqwvd(mgs) = ((((((-c_max(NR(0.0), qrcev(mgs))) - c_max(NR(0.0), qhcev(mgs))) - c_max(NR(0.0), qhlcev(mgs))) - c_max(NR(0.0), qscev(mgs))) + (il5(mgs) * (((((-qiint(mgs)) - qhdpv(mgs)) - qsdpv(mgs)) - qhldpv(mgs))))) - (il5(mgs) * qidpv(mgs)));  // 22227
    }  // 22236
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22264
      pqcwi(mgs) = (((NR(0.0)) + qwcnr(mgs)) - qwshw(mgs));  // 22266
      pqcwd(mgs) = (((((((il5(mgs) * ((((-qiacw(mgs)) - qwfrz(mgs)) - qwctfz(mgs)))) - (il5(mgs) * (qiihr(mgs)))) - qracw(mgs)) - qsacw(mgs)) - qrcnw(mgs)) - qhacw(mgs)) - qhlacw(mgs));  // 22269
      if (((pqcwd(mgs) < NR(0.0)) && ((-(pqcwd(mgs) * dtp)) > qx(mgs, P.lc)))) {  // 22285
        frac = (-(c_max(NR(0.0), qx(mgs, P.lc)) / ((pqcwd(mgs) * dtp))));  // 22287
        pqcwd(mgs) = (-(qx(mgs, P.lc) * dtpinv));  // 22288
        qiacw(mgs) = (frac * qiacw(mgs));  // 22290
        qwfrzc(mgs) = (frac * qwfrzc(mgs));  // 22293
        qwfrz(mgs) = (frac * qwfrz(mgs));  // 22294
        qwctfzc(mgs) = (frac * qwctfzc(mgs));  // 22295
        qwctfz(mgs) = (frac * qwctfz(mgs));  // 22296
        qracw(mgs) = (frac * qracw(mgs));  // 22297
        qsacw(mgs) = (frac * qsacw(mgs));  // 22298
        qhacw(mgs) = (frac * qhacw(mgs));  // 22299
        vhacw(mgs) = (frac * vhacw(mgs));  // 22300
        qrcnw(mgs) = (frac * qrcnw(mgs));  // 22301
        qwfrzp(mgs) = (frac * qwfrzp(mgs));  // 22302
        if ((P.lhl > 1)) {  // 22303
          qhlacw(mgs) = (frac * qhlacw(mgs));  // 22304
          vhlacw(mgs) = (frac * vhlacw(mgs));  // 22305
        }  // 22306
      }  // 22310
    }  // 22313
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22319
      pqcii(mgs) = ((((((((il5(mgs) * qicicnt(mgs)) + (il5(mgs) * (((((NR(1.0) - P.cwfrz2snowfrac)) * qwfrzc(mgs)) + qwctfzc(mgs))))) + (il5(mgs) * (qicichr(mgs)))) + qsmul(mgs)) + qhmul1(mgs)) + qhlmul1(mgs)) + qsplinter(mgs)) + qsplinter2(mgs));  // 22321
      pqcii(mgs) = (((pqcii(mgs) * ((NR(1.0) - P.ffrzs))) + (il5(mgs) * qidpv(mgs))) + (il5(mgs) * qiacw(mgs)));  // 22331
      pqcid(mgs) = ((((((il5(mgs) * (((((-qscni(mgs)) - qscnvi(mgs)) - qraci(mgs)) - qsaci(mgs)))) - qhaci(mgs)) - qhlaci(mgs)) + (il5(mgs) * qisbv(mgs))) + (((NR(1.0) - il5(mgs))) * qimlr(mgs))) - qhcni(mgs));  // 22335
    }  // 22344
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22382
      pqrwi(mgs) = ((((qracw(mgs) + qrcnw(mgs)) + c_max(NR(0.0), qrcev(mgs))) + (((1 - il5(mgs))) * (((((-qhmlr(mgs)) - qsmlr(mgs)) - qhlmlr(mgs)) - qimlr(mgs))))) - qrshr(mgs));  // 22384
      pqrwd(mgs) = ((((((il5(mgs) * (((-qiacr(mgs)) - qrfrz(mgs)))) - qsacr(mgs)) - qhacr(mgs)) - qhlacr(mgs)) - qwcnr(mgs)) + c_min(NR(0.0), qrcev(mgs)));  // 22395
      if (((pqrwd(mgs) < NR(0.0)) && ((-(((pqrwd(mgs) + pqrwi(mgs))) * dtp)) > qx(mgs, P.lr)))) {  // 22420
        frac = ((((-qx(mgs, P.lr)) + (pqrwi(mgs) * dtp))) / ((pqrwd(mgs) * dtp)));  // 22422
        pqwvi(mgs) = ((pqwvi(mgs) + c_min(NR(0.0), qrcev(mgs))) - (frac * c_min(NR(0.0), qrcev(mgs))));  // 22425
        pqwvd(mgs) = ((pqwvd(mgs) + c_max(NR(0.0), qrcev(mgs))) - (frac * c_max(NR(0.0), qrcev(mgs))));  // 22428
        qiacr(mgs) = (frac * qiacr(mgs));  // 22432
        qiacrf(mgs) = (frac * qiacrf(mgs));  // 22433
        qiacrs(mgs) = (frac * qiacrs(mgs));  // 22434
        viacrf(mgs) = (frac * viacrf(mgs));  // 22435
        qrfrz(mgs) = (frac * qrfrz(mgs));  // 22436
        qrfrzs(mgs) = (frac * qrfrzs(mgs));  // 22437
        qrfrzf(mgs) = (frac * qrfrzf(mgs));  // 22438
        vrfrzf(mgs) = (frac * vrfrzf(mgs));  // 22439
        qsacr(mgs) = (frac * qsacr(mgs));  // 22440
        qhacr(mgs) = (frac * qhacr(mgs));  // 22441
        vhacr(mgs) = (frac * vhacr(mgs));  // 22442
        qrcev(mgs) = (frac * qrcev(mgs));  // 22443
        qhlacr(mgs) = (frac * qhlacr(mgs));  // 22444
        vhlacr(mgs) = (frac * vhlacr(mgs));  // 22445
        qhcev(mgs) = (frac * qhcev(mgs));  // 22446
        qhlcev(mgs) = (frac * qhlcev(mgs));  // 22447
        pqrwd(mgs) = (((((il5(mgs) * ((((-qiacr(mgs)) - qrfrz(mgs)) - qsacr(mgs)))) - qhacr(mgs)) - qhlacr(mgs)) - qwcnr(mgs)) + c_min(NR(0.0), qrcev(mgs)));  // 22451
        if ((qrcev(mgs) != NR(0.0))) {  // 22468
          pqwvi(mgs) = ((((((((-c_min(NR(0.0), qrcev(mgs))) - c_min(NR(0.0), qhcev(mgs))) - c_min(NR(0.0), qhlcev(mgs))) - c_min(NR(0.0), qscev(mgs))) - qhsbv(mgs)) - qhlsbv(mgs)) - qssbv(mgs)) - (il5(mgs) * qisbv(mgs)));  // 22469
          pqwvd(mgs) = ((((((-c_max(NR(0.0), qrcev(mgs))) - c_max(NR(0.0), qhcev(mgs))) - c_max(NR(0.0), qhlcev(mgs))) - c_max(NR(0.0), qscev(mgs))) + (il5(mgs) * (((((-qiint(mgs)) - qhdpv(mgs)) - qsdpv(mgs)) - qhldpv(mgs))))) - (il5(mgs) * qidpv(mgs)));  // 22479
        }  // 22488
      }  // 22492
    }  // 22495
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22502
      pqswi(mgs) = (((((((il5(mgs) * (((((((qscni(mgs) + qsaci(mgs)) + qsdpv(mgs)) + qscnvi(mgs)) + (P.ifrzs * ((qiacrs(mgs) + qrfrzs(mgs))))) + (il5(mgs) * ((((((qwfrzc(mgs) + qwctfzc(mgs)) + qicichr(mgs))) * P.ffrzs) + ((((NR(1.0) - P.ffrzs)) * P.cwfrz2snowfrac) * qwfrz(mgs)))))) + (il2(mgs) * qsacr(mgs))))) + ((il5(mgs) * qicicnt(mgs)) * P.ffrzs)) + (il3(mgs) * ((qiacrf(mgs) + qracif(mgs))))) + c_max(NR(0.0), qscev(mgs))) + qsacw(mgs)) + qscnh(mgs)) + (P.ffrzs * (((((qsmul(mgs) + qhmul1(mgs)) + qhlmul1(mgs)) + qsplinter(mgs)) + qsplinter2(mgs)))));  // 22503
      pqswd(mgs) = (((((((((-(qracs(mgs) * ((1 - il2(mgs))))) - qhacs(mgs)) - qhlacs(mgs)) - qhcns(mgs)) + (((1 - il5(mgs))) * qsmlr(mgs))) + qsshr(mgs)) + qssbv(mgs)) + c_min(NR(0.0), qscev(mgs))) - qsmul(mgs));  // 22517
      if (((P.imixedphase == 0) && (pqswd(mgs) < NR(0.0)))) {  // 22528
        if (((qx(mgs, P.ls) + (dtp * ((pqswi(mgs) + pqswd(mgs))))) < NR(0.0))) {  // 22529
          frac = ((((-qx(mgs, P.ls)) + (pqswi(mgs) * dtp))) / ((pqswd(mgs) * dtp)));  // 22530
          pqswd(mgs) = (frac * pqswd(mgs));  // 22532
          qracs(mgs) = (frac * qracs(mgs));  // 22534
          qhacs(mgs) = (frac * qhacs(mgs));  // 22535
          qhlacs(mgs) = (frac * qhlacs(mgs));  // 22536
          qhcns(mgs) = (frac * qhcns(mgs));  // 22537
          qsmlr(mgs) = (frac * qsmlr(mgs));  // 22538
          qsshr(mgs) = (frac * qsshr(mgs));  // 22539
          qssbv(mgs) = (frac * qssbv(mgs));  // 22540
          qsmul(mgs) = (frac * qsmul(mgs));  // 22541
          if ((qscev(mgs) < NR(0.0))) { qscev(mgs) = (frac * qscev(mgs)); }  // 22542
        }  // 22544
      }  // 22545
      pqcii(mgs) = ((pqcii(mgs) + (((NR(1.0) - P.ifrzs)) * qrfrzs(mgs))) + (((NR(1.0) - P.ifrzs)) * qiacrs(mgs)));  // 22547
    }  // 22551
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22556
      pqhwi(mgs) = (((((((((((il5(mgs) * ((((ffrzh * P.ifrzg) * qrfrzf(mgs)) + (((((1 - il3(mgs))) * ffrzh) * P.ifiacrg) * ((qiacrf(mgs) + qracif(mgs))))))) + (((1 - il2(mgs))) * ((qracs(mgs) + qsacr(mgs))))) + (il5(mgs) * (qhdpv(mgs)))) + c_max(NR(0.0), qhcev(mgs))) + qhacr(mgs)) + qhacw(mgs)) + qhacs(mgs)) + qhaci(mgs)) + (P.f2h * qhcns(mgs))) + (P.f2h * qhcni(mgs))) + qhcnhl(mgs));  // 22557
      pqhwd(mgs) = (((((((qhshr(mgs) + (((1 - il5(mgs))) * qhmlr(mgs))) + qhsbv(mgs)) + c_min(NR(0.0), qhcev(mgs))) - qhmul1(mgs)) - qhlcnh(mgs)) - qscnh(mgs)) - (ffrzh * ((qsplinter(mgs) + qsplinter2(mgs)))));  // 22565
    }  // 22575
    if ((P.lhl > 1)) {  // 22581
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22583
        pqhli(mgs) = (((((((il5(mgs) * ((qhldpv(mgs) + (((((NR(1.0) - P.ifrzg)) * qrfrzf(mgs)) + (((NR(1.0) - P.ifiacrg)) * ((qiacrf(mgs) + qracif(mgs))))))))) + c_max(NR(0.0), qhlcev(mgs))) + qhlacr(mgs)) + qhlacw(mgs)) + qhlacs(mgs)) + qhlaci(mgs)) + qhlcnh(mgs));  // 22584
        pqhld(mgs) = (((((qhlshr(mgs) + (((1 - il5(mgs))) * qhlmlr(mgs))) + qhlsbv(mgs)) + c_min(NR(0.0), qhlcev(mgs))) - qhlmul1(mgs)) - qhcnhl(mgs));  // 22590
        frac = NR(0.0);  // 22599
        if (((qx(mgs, P.lhl) + (dtp * ((pqhli(mgs) + pqhld(mgs))))) < NR(0.0))) {  // 22600
          frac = ((((-qx(mgs, P.lhl)) + (pqhli(mgs) * dtp))) / ((pqhld(mgs) * dtp)));  // 22603
          qhlmlr(mgs) = (frac * qhlmlr(mgs));  // 22605
          qhlsbv(mgs) = (frac * qhlsbv(mgs));  // 22606
          qhcnhl(mgs) = (frac * qhcnhl(mgs));  // 22607
          qhlmul1(mgs) = (frac * qhlmul1(mgs));  // 22608
          if ((qhlcev(mgs) < NR(0.0))) { qhlcev(mgs) = (frac * qhlcev(mgs)); }  // 22609
          pqhld(mgs) = (frac * pqhld(mgs));  // 22611
        }  // 22613
      }  // 22617
    }  // 22619
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhmlr(i_0) = NR(0.0);  // 22667
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlmlr(i_0) = NR(0.0);  // 22668
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhfzh(i_0) = NR(0.0);  // 22669
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlfzhl(i_0) = NR(0.0);  // 22670
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhmlr(i_0) = qhmlr(i_0);  // 22676
    for (int i_0 = 1; i_0 <= ngs; ++i_0) vhlmlr(i_0) = qhlmlr(i_0);  // 22680
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22693
      zhdsv(mgs) = NR(0.0);  // 22699
      if ((ffrzh > NR(0.0))) {  // 22701
        ziacr(mgs) = NR(0.0);  // 22702
        ziacrf(mgs) = NR(0.0);  // 22703
      }  // 22704
      zhcns(mgs) = NR(0.0);  // 22706
      zhcni(mgs) = NR(0.0);  // 22707
      zhacs(mgs) = NR(0.0);  // 22708
      zhaci(mgs) = NR(0.0);  // 22709
    }  // 22711
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 22933
      zhldsv(mgs) = NR(0.0);  // 22935
      zhlacr(mgs) = NR(0.0);  // 22936
      zhlacw(mgs) = NR(0.0);  // 22937
    }  // 22939
    if ((P.lvol(P.ls) > 1)) {  // 23251
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23252
        pvswi(mgs) = ((rho0(mgs) * ((((il5(mgs) * ((((((qscni(mgs) + qsaci(mgs)) + qsdpv(mgs)) + qscnvi(mgs)) + (((NR(1.0) - P.ifrzs)) * qiacrs(mgs))) + (((NR(1.0) - P.ifrzs)) * qrfrzs(mgs))))) / xdn0(P.ls)) + ((qsacr(mgs)) / rimdn(mgs, P.ls))))) + vsacw(mgs));  // 23255
        pvswd(mgs) = (((rho0(mgs) * (pqswd(mgs))) / xdn0(P.ls)) - ((rho0(mgs) * qsmul(mgs)) / xdn0(P.ls)));  // 23264
      }  // 23273
    }  // 23280
    if ((P.lvol(P.lh) > 1)) {  // 23284
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23285
        pvhwi(mgs) = ((((((((rho0(mgs) * ((((il5(mgs) * (((P.ifiacrg * ffrzh) * qracif(mgs)))) / P.rhofrz) + ((((il5(mgs) * qhdpv(mgs)) / P.qhdpvdn) + (((qhacs(mgs) + qhaci(mgs))) / P.qhacidn)))))) + ((rho0(mgs) * c_max(NR(0.0), qhcev(mgs))) / NR(1000.0))) + (P.f2h * vhcns(mgs))) + vhacr(mgs)) + vhacw(mgs)) + vhfzh(mgs)) + (P.f2h * vhcni(mgs))) + ((((P.ifiacrg * viacrf(mgs)) + (P.ifrzg * vrfrzf(mgs)))) * ffrzh));  // 23291
        pvhwd(mgs) = (((((rho0(mgs) * (((((((((1 - il5(mgs))) * vhmlr(mgs)) + qhsbv(mgs)) + c_min(NR(0.0), qhcev(mgs))) - qhmul1(mgs))) / xdn(mgs, P.lh)))) - vhlcnh(mgs)) + vhshdr(mgs)) - vhsoak(mgs)) - vscnh(mgs));  // 23306
      }  // 23399
    }  // 23401
    if ((P.lhl > 1)) {  // 23409
      if ((P.lvol(P.lhl) > 1)) {  // 23410
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23411
          pvhli(mgs) = (((((((rho0(mgs) * ((((((il5(mgs) * ((((((((NR(1.0) - P.ifiacrg)) * ffrzh) * qracif(mgs))) / P.rhofrz) + qhldpv(mgs)))) + qhlacs(mgs)) + qhlaci(mgs))) / NR(500.0)))) + ((rho0(mgs) * c_max(NR(0.0), qhlcev(mgs))) / NR(1000.0))) + vhlcnhl(mgs)) + ((((((NR(1.0) - P.ifiacrg)) * ffrzh) * viacrf(mgs)) + ((((NR(1.0) - P.ifrzg)) * ffrzh) * vrfrzf(mgs))))) + vhlacr(mgs)) + vhlacw(mgs)) + vhlfzhl(mgs));  // 23413
          pvhld(mgs) = ((((rho0(mgs) * (((((qhlsbv(mgs) + c_min(NR(0.0), qhlcev(mgs))) - qhlmul1(mgs))) / xdn(mgs, P.lhl)))) + (((rho0(mgs) * ((1 - il5(mgs)))) * vhlmlr(mgs)) / xdn(mgs, P.lhl))) + vhlshdr(mgs)) - vhlsoak(mgs));  // 23423
        }  // 23459
      }  // 23461
    }  // 23462
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23760
      pfrz(mgs) = (((((1 - il5(mgs))) * (((qhmlr(mgs) + qsmlr(mgs)) + qhlmlr(mgs)))) + ((il5(mgs) * ((1 - P.imixedphase))) * (((((((((((qsacw(mgs) + qhacw(mgs)) + qhlacw(mgs)) + qsacr(mgs)) + qhacr(mgs)) + qhlacr(mgs)) + qsshr(mgs)) + qhshr(mgs)) + qhlshr(mgs)) + qrfrz(mgs)) + qiacr(mgs))))) + (il5(mgs) * ((((qwfrz(mgs) + qwctfz(mgs)) + qiihr(mgs)) + qiacw(mgs)))));  // 23762
      pmlt(mgs) = (((1 - il5(mgs))) * (((qhmlr(mgs) + qsmlr(mgs)) + qhlmlr(mgs))));  // 23777
      psub(mgs) = (((((il5(mgs) * (((((qsdpv(mgs) + qhdpv(mgs)) + qhldpv(mgs)) + qidpv(mgs)) + qisbv(mgs)))) + qssbv(mgs)) + qhsbv(mgs)) + qhlsbv(mgs)) + (il5(mgs) * (qiint(mgs))));  // 23782
      pvap(mgs) = ((((qrcev(mgs) + qhcev(mgs)) + qscev(mgs)) + qhlcev(mgs)) + qfcev(mgs));  // 23790
      pevap(mgs) = ((((c_min(NR(0.0), qrcev(mgs)) + c_min(NR(0.0), qhcev(mgs))) + c_min(NR(0.0), qscev(mgs))) + c_min(NR(0.0), qhlcev(mgs))) + c_min(NR(0.0), qfcev(mgs)));  // 23792
      pdep(mgs) = ((il5(mgs) * ((((qsdpv(mgs) + qhdpv(mgs)) + qhldpv(mgs)) + qidpv(mgs)))) + (il5(mgs) * (qiint(mgs))));  // 23796
      ptem(mgs) = (((NR(1.0) / pi0(mgs))) * ((((felfcp(mgs) * pfrz(mgs)) + (felscp(mgs) * psub(mgs))) + (felvcp(mgs) * pvap(mgs)))));  // 23828
      thetap(mgs) = (thetap(mgs) + (dtp * ptem(mgs)));  // 23833
      ptem2(mgs) = ptem(mgs);  // 23834
    }  // 23840
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23849
      qwvp(mgs) = (qwvp(mgs) + (dtp * ((pqwvi(mgs) + pqwvd(mgs)))));  // 23852
      qx(mgs, P.lc) = (qx(mgs, P.lc) + (dtp * ((pqcwi(mgs) + pqcwd(mgs)))));  // 23855
      qx(mgs, P.lr) = (qx(mgs, P.lr) + (dtp * ((pqrwi(mgs) + pqrwd(mgs)))));  // 23857
      qx(mgs, P.li) = (qx(mgs, P.li) + (dtp * ((pqcii(mgs) + pqcid(mgs)))));  // 23859
      qx(mgs, P.ls) = (qx(mgs, P.ls) + (dtp * ((pqswi(mgs) + pqswd(mgs)))));  // 23861
      qx(mgs, P.lh) = (qx(mgs, P.lh) + (dtp * ((pqhwi(mgs) + pqhwd(mgs)))));  // 23863
      if ((P.lhl > 1)) {  // 23866
        qx(mgs, P.lhl) = (qx(mgs, P.lhl) + (dtp * ((pqhli(mgs) + pqhld(mgs)))));  // 23867
      }  // 23869
    }  // 23872
    if (ldovol) {  // 23876
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23878
        if ((P.lvol(P.ls) > 1)) {  // 23880
          vx(mgs, P.ls) = (vx(mgs, P.ls) + (dtp * ((pvswi(mgs) + pvswd(mgs)))));  // 23881
        }  // 23883
        if ((P.lvol(P.lh) > 1)) {  // 23885
          vx(mgs, P.lh) = (vx(mgs, P.lh) + (dtp * ((pvhwi(mgs) + pvhwd(mgs)))));  // 23886
        }  // 23889
        if ((P.lhl > 1)) {  // 23891
          if ((P.lvol(P.lhl) > 1)) {  // 23892
            vx(mgs, P.lhl) = (vx(mgs, P.lhl) + (dtp * ((pvhli(mgs) + pvhld(mgs)))));  // 23893
          }  // 23896
        }  // 23897
      }  // 23899
    }  // 23901
    if ((P.ipconc >= 1)) {  // 23908
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23909
        cx(mgs, P.li) = (cx(mgs, P.li) + (dtp * ((pccii(mgs) + pccid(mgs)))));  // 23910
        cina(mgs) = (cina(mgs) + (pccin(mgs) * dtp));  // 23912
        if ((P.ipconc >= 2)) {  // 23913
          cx(mgs, P.lc) = (cx(mgs, P.lc) + (dtp * ((pccwi(mgs) + pccwd(mgs)))));  // 23914
        }  // 23916
        if ((P.ipconc >= 3)) {  // 23917
          cx(mgs, P.lr) = (cx(mgs, P.lr) + (dtp * ((pcrwi(mgs) + pcrwd(mgs)))));  // 23918
        }  // 23920
        if ((P.ipconc >= 4)) {  // 23921
          cx(mgs, P.ls) = (cx(mgs, P.ls) + (dtp * ((pcswi(mgs) + pcswd(mgs)))));  // 23922
        }  // 23924
        if ((P.ipconc >= 5)) {  // 23925
          cx(mgs, P.lh) = (cx(mgs, P.lh) + (dtp * ((pchwi(mgs) + pchwd(mgs)))));  // 23926
          if ((P.lhl > 1)) {  // 23928
            cx(mgs, P.lhl) = (cx(mgs, P.lhl) + (dtp * ((pchli(mgs) + pchld(mgs)))));  // 23929
          }  // 23935
        }  // 23936
        if ((P.ipconc >= 6)) {  // 23937
        }  // 23957
      }  // 23958
    }  // 23959
    if (has_wetscav) {  // 23961
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23962
        evapprod2d(igs(mgs), kgs(mgs)) = (-((((qrcev(mgs) + qssbv(mgs)) + qhsbv(mgs)) + qhlsbv(mgs))));  // 23963
        rainprod2d(igs(mgs), kgs(mgs)) = (((((((((qrcnw(mgs) + qracw(mgs)) + qsacw(mgs)) + qhacw(mgs)) + qhlacw(mgs)) + qraci(mgs)) + qsaci(mgs)) + qhaci(mgs)) + qhlaci(mgs)) + qscni(mgs));  // 23964
      }  // 23966
    }  // 23967
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23984
      pqs(mgs) = ((NR(380.0)) / (pres(mgs)));  // 23985
      theta(mgs) = (thetap(mgs) + theta0(mgs));  // 23986
      qvap(mgs) = c_max(((qwvp(mgs) + qv0(mgs))), NR(0.0));  // 23987
      temg(mgs) = (theta(mgs) * pk(mgs));  // 23988
    }  // 23989
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23993
      qcwtmp(mgs) = qx(mgs, P.lc);  // 23994
      ptimlw(mgs) = NR(0.0);  // 23995
    }  // 23996
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 23998
      qitmp(mgs) = qx(mgs, P.li);  // 23999
      if (((temg(mgs) > P.tfr) && (qitmp(mgs) > NR(0.0)))) {  // 24000
        qx(mgs, P.lc) = (qx(mgs, P.lc) + qitmp(mgs));  // 24002
        ptem(mgs) = (ptem(mgs) + ((((NR(1.0) / pi0(mgs))) * felfcp(mgs)) * ((-(qitmp(mgs) * dtpinv)))));  // 24004
        pmlt(mgs) = (pmlt(mgs) - (qitmp(mgs) * dtpinv));  // 24010
        scx(mgs, P.lc) = (scx(mgs, P.lc) + scx(mgs, P.li));  // 24011
        thetap(mgs) = (thetap(mgs) - (fcc3(mgs) * qitmp(mgs)));  // 24012
        ptimlw(mgs) = (-((fcc3(mgs) * qitmp(mgs)) * dtpinv));  // 24014
        cx(mgs, P.lc) = (cx(mgs, P.lc) + cx(mgs, P.li));  // 24015
        qx(mgs, P.li) = NR(0.0);  // 24016
        cx(mgs, P.li) = NR(0.0);  // 24017
        scx(mgs, P.li) = NR(0.0);  // 24018
        vx(mgs, P.li) = NR(0.0);  // 24019
        qitmp(mgs) = NR(0.0);  // 24020
      }  // 24021
    }  // 24022
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24036
      qcwtmp(mgs) = qx(mgs, P.lc);  // 24037
      ptwfzi(mgs) = NR(0.0);  // 24038
    }  // 24039
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24041
      ctmp = NR(0.0);  // 24047
      frac = NR(0.0);  // 24048
      qtmp = NR(0.0);  // 24049
      if (((((((temg(mgs) < (thnuc + NR(0.0)))) || (((temg(mgs) < (thnuc + NR(2.0))) && (P.ibfc >= 3))))) && (qx(mgs, P.lc) > NR(0.0))) && ((((P.ipconc < 2) || (P.ibfc == 0)) || (P.ibfc == 2))))) {  // 24056
        frac = c_max(NR(0.25), c_min(NR(1.0), (((((thnuc + NR(1.0))) - temg(mgs))) / NR(4.0))));  // 24062
        qtmp = (frac * qx(mgs, P.lc));  // 24076
        qx(mgs, P.li) = (qx(mgs, P.li) + qtmp);  // 24081
        pfrz(mgs) = (pfrz(mgs) + (qtmp * dtpinv));  // 24083
        ptem(mgs) = (ptem(mgs) + ((((NR(1.0) / pi0(mgs))) * felfcp(mgs)) * ((qtmp * dtpinv))));  // 24084
        if ((P.lvol(P.li) > 1)) { vx(mgs, P.li) = (vx(mgs, P.li) + ((rho0(mgs) * qtmp) / xdn0(P.li))); }  // 24093
        if ((P.ipconc >= 2)) {  // 24095
          ctmp = (frac * cx(mgs, P.lc));  // 24096
          cx(mgs, P.li) = (cx(mgs, P.li) + ctmp);  // 24101
        } else {  // 24103
          ctmp = NR(0.0);  // 24104
          if ((t9(igs(mgs), jgs, (kgs(mgs) - 1)) > qx(mgs, P.lc))) {  // 24105
            qtmp = (frac * t9(igs(mgs), jgs, (kgs(mgs) - 1)));  // 24106
            ctmp = (((cx(mgs, P.lc) * qx(mgs, P.lc)) * rho0(mgs)) / qtmp);  // 24109
          } else {  // 24110
            cx(mgs, P.lc) = (((c_max(NR(0.0), wvel(mgs)) * dtp) * P.cwccn) / gz(igs(mgs), jgs, kgs(mgs)));  // 24111
            cx(mgs, P.lc) = P.cwccn;  // 24113
          }  // 24114
          if ((P.ipconc >= 1)) { cx(mgs, P.li) = c_min(ccimx, (cx(mgs, P.li) + cx(mgs, P.lc))); }  // 24116
        }  // 24117
        sctmp = (frac * scx(mgs, P.lc));  // 24119
        scx(mgs, P.li) = (scx(mgs, P.li) + sctmp);  // 24121
        thetap(mgs) = (thetap(mgs) + (fcc3(mgs) * qtmp));  // 24127
        ptwfzi(mgs) = ((fcc3(mgs) * qtmp) * dtpinv);  // 24128
        qx(mgs, P.lc) = (qx(mgs, P.lc) - qtmp);  // 24129
        cx(mgs, P.lc) = (cx(mgs, P.lc) - ctmp);  // 24130
        scx(mgs, P.lc) = (scx(mgs, P.lc) - sctmp);  // 24131
      }  // 24132
    }  // 24133
    for (int i_0 = 1; i_0 <= ngs; ++i_0) qcond(i_0) = NR(0.0);  // 24143
    if (((P.ipconc <= 1) && (!P.lwsm6))) {  // 24183
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24186
        qx(mgs, P.lv) = c_max(NR(0.0), qvap(mgs));  // 24187
        qx(mgs, P.lc) = c_max(NR(0.0), qx(mgs, P.lc));  // 24188
        qx(mgs, P.li) = c_max(NR(0.0), qx(mgs, P.li));  // 24189
        qitmp(mgs) = qx(mgs, P.li);  // 24190
      }  // 24191
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24194
        qcwtmp(mgs) = qx(mgs, P.lc);  // 24195
        qitmp(mgs) = qx(mgs, P.li);  // 24196
        theta(mgs) = (thetap(mgs) + theta0(mgs));  // 24197
        temgtmp = temg(mgs);  // 24198
        temg(mgs) = (theta(mgs) * ((pinit(kgs(mgs)) + p2(igs(mgs), jgs, kgs(mgs)))));  // 24199
        temsav = temg(mgs);  // 24200
        thsave(mgs) = thetap(mgs);  // 24201
        temcg(mgs) = (temg(mgs) - P.tfr);  // 24202
        tqvcon = (temg(mgs) - P.cbw);  // 24203
        ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 24204
        ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 24205
        qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 24210
        qis(mgs) = (pqs(mgs) * P.tabqis(ltemq));  // 24212
        qss(mgs) = qvs(mgs);  // 24213
        if ((temg(mgs) < P.tfr)) {  // 24214
          if (((qx(mgs, P.lc) >= NR(0.0)) && (qitmp(mgs) <= P.qxmin(P.li)))) { qss(mgs) = qvs(mgs); }  // 24215
          if (((qx(mgs, P.lc) <= P.qxmin(P.lc)) && (qitmp(mgs) > P.qxmin(P.li)))) { qss(mgs) = qis(mgs); }  // 24217
          if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (qitmp(mgs) > P.qxmin(P.li)))) { qss(mgs) = ((((qx(mgs, P.lc) * qvs(mgs)) + (qitmp(mgs) * qis(mgs)))) / ((qx(mgs, P.lc) + qitmp(mgs)))); }  // 24219
        }  // 24222
      }  // 24223
      for (itertd = 1; itertd <= 2; ++itertd) {  // 24227
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24229
          qitmp(mgs) = qx(mgs, P.li);  // 24233
          fcci(mgs) = NR(0.0);  // 24234
          fcip(mgs) = NR(0.0);  // 24235
          dqcw(mgs) = NR(0.0);  // 24236
          dqci(mgs) = NR(0.0);  // 24237
          dqwv(mgs) = ((qx(mgs, P.lv) - qss(mgs)));  // 24238
          if ((dqwv(mgs) < NR(0.0))) {  // 24242
            if ((qx(mgs, P.lc) > (-dqwv(mgs)))) {  // 24243
              dqcw(mgs) = dqwv(mgs);  // 24244
              dqwv(mgs) = NR(0.0);  // 24245
            } else {  // 24246
              dqcw(mgs) = (-qx(mgs, P.lc));  // 24247
              dqwv(mgs) = (dqwv(mgs) + qx(mgs, P.lc));  // 24248
            }  // 24249
            if ((qitmp(mgs) > (-dqwv(mgs)))) {  // 24251
              dqci(mgs) = dqwv(mgs);  // 24252
              dqwv(mgs) = NR(0.0);  // 24253
            } else {  // 24254
              dqci(mgs) = (-qitmp(mgs));  // 24255
              dqwv(mgs) = (dqwv(mgs) + qitmp(mgs));  // 24256
            }  // 24257
            qwvp(mgs) = (qwvp(mgs) - ((dqcw(mgs) + dqci(mgs))));  // 24259
            qitmp(mgs) = qx(mgs, P.li);  // 24264
            if ((qitmp(mgs) >= P.qxmin(P.li))) {  // 24265
              fcci(mgs) = (qx(mgs, P.li) / (qitmp(mgs)));  // 24266
            } else {  // 24267
              fcci(mgs) = NR(1.0);  // 24268
            }  // 24269
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dqcw(mgs));  // 24270
            qx(mgs, P.li) = (qx(mgs, P.li) + (dqci(mgs) * fcci(mgs)));  // 24271
            thetap(mgs) = (thetap(mgs) + ((NR(1.0) / pi0(mgs)) * (((felvcp(mgs) * dqcw(mgs)) + (felscp(mgs) * dqci(mgs))))));  // 24272
          }  // 24282
          if ((dqwv(mgs) >= NR(0.0))) {  // 24286
            qitmp(mgs) = qx(mgs, P.li);  // 24290
            fracl(mgs) = NR(1.0);  // 24291
            fraci(mgs) = NR(0.0);  // 24292
            if (((temg(mgs) < P.tfr) && (temg(mgs) > thnuc))) {  // 24293
              fracl(mgs) = c_max(c_min(NR(1.0), (((temg(mgs) - NR(233.15))) / (NR(20.0)))), NR(0.0));  // 24294
              fraci(mgs) = (NR(1.0) - fracl(mgs));  // 24295
            }  // 24296
            if ((temg(mgs) <= thnuc)) {  // 24297
              fraci(mgs) = NR(1.0);  // 24298
              fracl(mgs) = NR(0.0);  // 24299
            }  // 24300
            fraci(mgs) = (NR(1.0) - fracl(mgs));  // 24301
            gamss = ((((felvcp(mgs) * fracl(mgs)) + (felscp(mgs) * fraci(mgs)))) / (pi0(mgs)));  // 24303
            if ((temg(mgs) < P.tfr)) {  // 24306
              if (((qx(mgs, P.lc) >= NR(0.0)) && (qitmp(mgs) <= P.qxmin(P.li)))) {  // 24307
                dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv1(mgs) * qss(mgs)) / (ipow<2>(((temg(mgs) - P.cbw))))))));  // 24308
              }  // 24310
              if (((qx(mgs, P.lc) == NR(0.0)) && (qitmp(mgs) > P.qxmin(P.li)))) {  // 24311
                dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv2(mgs) * qss(mgs)) / (ipow<2>(((temg(mgs) - P.cbi))))))));  // 24312
              }  // 24314
              if (((qx(mgs, P.lc) > NR(0.0)) && (qitmp(mgs) > P.qxmin(P.li)))) {  // 24315
                cdw = (((P.caw * pi0(mgs)) * tfrcbw) / (ipow<2>(((temg(mgs) - P.cbw)))));  // 24316
                cdi = (((P.cai * pi0(mgs)) * tfrcbi) / (ipow<2>(((temg(mgs) - P.cbi)))));  // 24317
                denom1 = (qx(mgs, P.lc) + qitmp(mgs));  // 24318
                denom2 = (NR(1.0) + ((gamss * ((((qx(mgs, P.lc) * qvs(mgs)) * cdw) + ((qitmp(mgs) * qis(mgs)) * cdi)))) / denom1));  // 24319
                dqvcnd(mgs) = (dqwv(mgs) / denom2);  // 24321
              }  // 24322
            }  // 24324
            if ((temg(mgs) >= P.tfr)) {  // 24326
              dqvcnd(mgs) = (dqwv(mgs) / ((NR(1.0) + ((fcqv1(mgs) * qss(mgs)) / (ipow<2>(((temg(mgs) - P.cbw))))))));  // 24327
            }  // 24329
            delqci1 = qx(mgs, P.li);  // 24331
            if ((qitmp(mgs) > P.qxmin(P.li))) {  // 24333
              fcci(mgs) = (qx(mgs, P.li) / (qitmp(mgs)));  // 24334
            } else {  // 24335
              fcci(mgs) = NR(1.0);  // 24336
            }  // 24337
            dqcw(mgs) = (dqvcnd(mgs) * fracl(mgs));  // 24339
            dqci(mgs) = (dqvcnd(mgs) * fraci(mgs));  // 24340
            thetap(mgs) = (thetap(mgs) + ((((felvcp(mgs) * dqcw(mgs)) + (felscp(mgs) * dqci(mgs)))) / (pi0(mgs))));  // 24342
            qwvp(mgs) = (qwvp(mgs) - (dqvcnd(mgs)));  // 24352
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dqcw(mgs));  // 24353
            qx(mgs, P.li) = (qx(mgs, P.li) + (dqci(mgs) * fcci(mgs)));  // 24355
            qitmp(mgs) = qx(mgs, P.li);  // 24356
          }  // 24361
        }  // 24362
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24364
          qitmp(mgs) = qx(mgs, P.li);  // 24365
          theta(mgs) = (thetap(mgs) + theta0(mgs));  // 24366
          temg(mgs) = (theta(mgs) * pk(mgs));  // 24367
          qvap(mgs) = c_max(((qwvp(mgs) + qv0(mgs))), NR(0.0));  // 24368
          temcg(mgs) = (temg(mgs) - P.tfr);  // 24369
          tqvcon = (temg(mgs) - P.cbw);  // 24370
          ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 24371
          ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 24372
          qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 24377
          qis(mgs) = (pqs(mgs) * P.tabqis(ltemq));  // 24379
          qx(mgs, P.lc) = c_max(NR(0.0), qx(mgs, P.lc));  // 24380
          qitmp(mgs) = c_max(NR(0.0), qitmp(mgs));  // 24381
          qx(mgs, P.lv) = c_max(NR(0.0), qvap(mgs));  // 24382
          qss(mgs) = qvs(mgs);  // 24396
          if ((temg(mgs) < P.tfr)) {  // 24397
            if (((qx(mgs, P.lc) >= NR(0.0)) && (qitmp(mgs) <= P.qxmin(P.li)))) { qss(mgs) = qvs(mgs); }  // 24398
            if (((qx(mgs, P.lc) <= P.qxmin(P.lc)) && (qitmp(mgs) > P.qxmin(P.li)))) { qss(mgs) = qis(mgs); }  // 24400
            if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (qitmp(mgs) > P.qxmin(P.li)))) { qss(mgs) = ((((qx(mgs, P.lc) * qvs(mgs)) + (qitmp(mgs) * qis(mgs)))) / ((qx(mgs, P.lc) + qitmp(mgs)))); }  // 24402
          }  // 24405
        }  // 24408
      }  // 24412
    }  // 24414
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24431
      t0(igs(mgs), jy, kgs(mgs)) = temg(mgs);  // 24432
    }  // 24433
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24458
      an(igs(mgs), jy, kgs(mgs), P.lt) = (theta0(mgs) + thetap(mgs));  // 24460
      an(igs(mgs), jy, kgs(mgs), P.lv) = (qwvp(mgs) + qv0(mgs));  // 24462
      for (il = P.lc; il <= P.lhab; ++il) {  // 24469
        if ((ido(il) == 1)) {  // 24470
          an(igs(mgs), jy, kgs(mgs), il) = (qx(mgs, il) + c_min(an(igs(mgs), jy, kgs(mgs), il), NR(0.0)));  // 24475
          qx(mgs, il) = an(igs(mgs), jy, kgs(mgs), il);  // 24477
        }  // 24478
      }  // 24479
      if ((P.ipconc >= 6)) {  // 24493
        for (il = P.lr; il <= P.lhab; ++il) {  // 24494
          if ((P.lz(il) > 1)) {  // 24495
            an(igs(mgs), jy, kgs(mgs), P.lz(il)) = (zx(mgs, il) + c_min(an(igs(mgs), jy, kgs(mgs), P.lz(il)), NR(0.0)));  // 24501
            zx(mgs, il) = an(igs(mgs), jy, kgs(mgs), P.lz(il));  // 24503
          }  // 24505
        }  // 24506
      }  // 24508
    }  // 24510
    if ((P.ipconc >= 1)) {  // 24513
      for (il = P.lc; il <= P.lhab; ++il) {  // 24514
        if (((P.ipconc >= P.ipc(il)) && (ido(il) > 0))) {  // 24518
          if (((P.ipconc >= 4) && (P.ipc(il) >= 1))) {  // 24520
            if (((P.lz(il) <= 1) || (P.ioldlimiter == 1))) {  // 24525
              for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24528
                if ((qx(mgs, il) <= NR(0.0))) {  // 24529
                  cx(mgs, il) = NR(0.0);  // 24530
                } else {  // 24531
                  if (((cx(mgs, il) > P.cxmin) && (qx(mgs, il) > P.qxmin(il)))) {  // 24532
                    xv(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / ((xdn(mgs, il) * cx(mgs, il))));  // 24535
                    if (((((((P.imaxdiaopt == 1) || (il == P.lc)) || (il == P.li)) || (((il == P.lr) && (P.imurain == 3)))) || (((il == P.ls) && (P.imusnow == 3)))) || (((il >= P.lh) && (P.lh > 0))))) {  // 24542
                      xvbarmax = P.xvmx(il);  // 24545
                    } else {  // 24548
                      xvbarmax = (P.xvmx(il) / ((ipow<3>(((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))))));  // 24549
                    }  // 24552
                    tmp = NR(1.0);  // 24554
                    if ((il == P.ls)) {  // 24555
                      xvbarmax = (xvbarmax * c_max(NR(1.0), (NR(100.0) / c_min(NR(100.0), xdn(mgs, P.ls)))));  // 24556
                    }  // 24557
                    if (((xv(mgs, il) < P.xvmn(il)) || (xv(mgs, il) > xvbarmax))) {  // 24559
                      xv(mgs, il) = c_min(xvbarmax, xv(mgs, il));  // 24560
                      xv(mgs, il) = c_max(P.xvmn(il), xv(mgs, il));  // 24561
                      cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / ((xv(mgs, il) * xdn(mgs, il))));  // 24562
                    }  // 24563
                  }  // 24565
                }  // 24571
              }  // 24572
            } else {  // 24574
              if (((((il == P.lh) || (il == P.lhl)) || (il == P.lf)) || (((il == P.lr) && (P.imurain == 1))))) {  // 24736
                for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 24740
                  if ((((P.iresetmoments == 1) || (P.iresetmoments == il)) || (P.iresetmoments == (-1)))) {  // 24765
                    if ((zx(mgs, il) <= P.zxmin)) {  // 24766
                      qx(mgs, il) = NR(0.0);  // 24768
                      cx(mgs, il) = NR(0.0);  // 24769
                      an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 24770
                      an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 24771
                      an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 24772
                    } else if (((cx(mgs, il) <= P.cxmin) && (P.iresetmoments != (-1)))) {  // 24783
                      qx(mgs, P.lv) = (qx(mgs, P.lv) + qx(mgs, il));  // 24784
                      zx(mgs, il) = NR(0.0);  // 24785
                      qx(mgs, il) = NR(0.0);  // 24786
                      an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 24787
                      an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 24788
                      an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 24789
                    }  // 24790
                  } else {  // 24791
                    if ((zx(mgs, il) < NR(0.0))) {  // 24792
                      zx(mgs, il) = NR(0.0);  // 24793
                    }  // 24794
                  }  // 24795
                  if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= P.cxmin))) {  // 24798
                    zx(mgs, il) = NR(0.0);  // 24799
                    cx(mgs, il) = NR(0.0);  // 24800
                    an(igs(mgs), jgs, kgs(mgs), P.lv) = (an(igs(mgs), jgs, kgs(mgs), P.lv) + an(igs(mgs), jgs, kgs(mgs), il));  // 24801
                    qx(mgs, il) = NR(0.0);  // 24802
                    an(igs(mgs), jgs, kgs(mgs), il) = qx(mgs, il);  // 24803
                    an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 24804
                    an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 24805
                  }  // 24806
                  if ((qx(mgs, il) > P.qxmin(il))) {  // 24808
                    xv(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / ((xdn(mgs, il) * c_max(NR(1.0e-9), cx(mgs, il)))));  // 24810
                    xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 24811
                    if ((xv(mgs, il) < P.xvmn(il))) {  // 24813
                      xv(mgs, il) = c_min(P.xvmx(il), c_max(P.xvmn(il), xv(mgs, il)));  // 24814
                      xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 24815
                      cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 24816
                    }  // 24817
                    if (((zx(mgs, il) > NR(0.0)) && (cx(mgs, il) <= NR(0.0)))) {  // 24819
                      g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 24821
                      z = zx(mgs, il);  // 24823
                      qr = qx(mgs, il);  // 24824
                      cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((NR(6.0) * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 24826
                    } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) > NR(0.0)))) {  // 24829
                      chw = cx(mgs, il);  // 24833
                      qr = qx(mgs, il);  // 24834
                      g1 = (((((NR(6.0) + P.alphamax)) * ((NR(5.0) + P.alphamax))) * ((NR(4.0) + P.alphamax))) / (((((NR(3.0) + P.alphamax)) * ((NR(2.0) + P.alphamax))) * ((NR(1.0) + P.alphamax)))));  // 24837
                      zx(mgs, il) = c_max((P.zxmin * NR(1.1)), (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((6 * qr)))) / ((chw * ipow<2>(((P.pi * xdn(mgs, il))))))));  // 24839
                      an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 24840
                    } else if (((zx(mgs, il) <= P.zxmin) && (cx(mgs, il) <= NR(0.0)))) {  // 24842
                      zx(mgs, il) = ((NR(1.0e-19) / NR(0.224)) * ipow<2>(((xdn0(P.lr) / xdn0(il)))));  // 24849
                      an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 24850
                      g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 24852
                      z = zx(mgs, il);  // 24854
                      qr = qx(mgs, il);  // 24855
                      cx(mgs, il) = (((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * ipow<2>(((NR(6.0) * qr)))) / ((z * ipow<2>(((P.pi * xdn(mgs, il)))))));  // 24857
                      an(igs(mgs), jgs, kgs(mgs), P.ln(il)) = cx(mgs, il);  // 24858
                    } else {  // 24862
                      chw = cx(mgs, il);  // 24864
                      qr = qx(mgs, il);  // 24865
                      z = zx(mgs, il);  // 24866
                      if ((((zx(mgs, il) > P.zxmin) && (qr > P.qxmin(il))) && (chw > P.cxmin))) {  // 24868
                        rdi = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((rho0(mgs) * qr)))));  // 24871
                        alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 24875
                        for (i = 1; i <= 10; ++i) {  // 24878
                          if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 24880
                          alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 24881
                          alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 24884
                          alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 24887
                        }  // 24888
                        if ((xv(mgs, il) > P.xvmx(il))) {  // 24892
                          tmp = cx(mgs, il);  // 24893
                          xv(mgs, il) = c_min(P.xvmx(il), c_max(P.xvmn(il), xv(mgs, il)));  // 24896
                          xmas(mgs, il) = (xv(mgs, il) * xdn(mgs, il));  // 24897
                          cx(mgs, il) = ((rho0(mgs) * qx(mgs, il)) / (xmas(mgs, il)));  // 24898
                          if ((tmp < cx(mgs, il))) {  // 24899
                            g1 = ((((NR(36.0) * ((NR(6.0) + alpha(mgs, il)))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / ((((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il)))) * ipow<2>(P.pi))));  // 24900
                            zx(mgs, il) = (zx(mgs, il) + ((g1 * ipow<2>(((rho0(mgs) / xdn(mgs, il))))) * ((ipow<2>(((qx(mgs, il) / tmp))) * ((tmp - cx(mgs, il)))))));  // 24902
                            an(igs(mgs), jgs, kgs(mgs), P.lz(il)) = zx(mgs, il);  // 24903
                            chw = cx(mgs, il);  // 24905
                            qr = qx(mgs, il);  // 24906
                            z = zx(mgs, il);  // 24907
                            rdi = (((z * ipow<2>((((P.pi / NR(6.0)) * xdn(mgs, il))))) * chw) / (ipow<2>(((rho0(mgs) * qr)))));  // 24909
                            alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 24910
                            for (i = 1; i <= 10; ++i) {  // 24912
                              if ((fabs((alp - alpha(mgs, il))) < NR(0.01))) { break; }  // 24913
                              alpha(mgs, il) = c_max(P.alphamin, c_min(P.alphamax, alp));  // 24914
                              alp = ((((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * rdi))) - NR(1.0));  // 24915
                              alp = c_max(P.alphamin, c_min(P.alphamax, alp));  // 24917
                            }  // 24918
                          }  // 24921
                        }  // 24922
                        g1 = (((((NR(6.0) + alpha(mgs, il))) * ((NR(5.0) + alpha(mgs, il)))) * ((NR(4.0) + alpha(mgs, il)))) / (((((NR(3.0) + alpha(mgs, il))) * ((NR(2.0) + alpha(mgs, il)))) * ((NR(1.0) + alpha(mgs, il))))));  // 24928
                        if ((((lrescalelow(il) || P.rescale_high_alpha)) && ((((alpha(mgs, il) <= P.alphamin) || (alp == P.alphamin)) || (alp == P.alphamax))))) {  // 24931
                          if ((((lrescalelow(il) && (alp <= P.alphamin)) && (!(((il == P.lh) && (P.icvhl2h > 0))))) && (!(((il == P.lr) && (!P.rescale_low_alphar)))))) {  // 24938
                            wtest = false;  // 24941
                            wtest = (((temcg(mgs) > P.rescale_tempthresh) && (qx(mgs, P.lc) > P.qxmin(P.lc))) && (wvel(mgs) < P.rescale_wthresh));  // 24949
                            if (((il == P.lr) && ((wtest || (!P.rescale_low_alphar))))) {  // 24952
                              chw = (((((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * (qr)) * qr) / z) * ipow<2>(((NR(6.0) / ((P.pi * xdn(mgs, il)))))));  // 24955
                              cx(mgs, il) = chw;  // 24956
                              an(igs(mgs), jy, kgs(mgs), P.ln(il)) = chw;  // 24957
                            } else {  // 24958
                              z1 = ((((g1 * ipow<2>(dn(igs(mgs), jy, kgs(mgs)))) * (qr)) * qr) / chw);  // 24960
                              z = (z1 * ipow<2>(((NR(6.0) / ((P.pi * xdn(mgs, il)))))));  // 24961
                              zx(mgs, il) = z;  // 24962
                              an(igs(mgs), jy, kgs(mgs), P.lz(il)) = z;  // 24963
                            }  // 24964
                          }  // 24970
                        }  // 24972
                      }  // 24975
                    }  // 24978
                  }  // 24982
                }  // 25011
              }  // 25021
            }  // 25023
          }  // 25024
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 25026
            if ((il == P.lh)) {  // 25028
            }  // 25032
            if ((il == P.lhl)) {  // 25034
            }  // 25040
            an(igs(mgs), jy, kgs(mgs), P.ln(il)) = c_max(cx(mgs, il), NR(0.0));  // 25041
          }  // 25042
        }  // 25043
      }  // 25044
      if ((P.ipconc >= 2)) {  // 25052
        for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 25053
          if ((P.lccn > 1)) {  // 25058
            an(igs(mgs), jy, kgs(mgs), P.lccn) = c_max(NR(0.0), ccnc(mgs));  // 25059
          }  // 25060
        }  // 25061
      }  // 25062
    } else if (((P.ipconc == 0) && (P.lni > 1))) {  // 25064
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 25066
        an(igs(mgs), jy, kgs(mgs), P.lni) = c_max(cx(mgs, P.li), NR(0.0));  // 25067
      }  // 25068
    }  // 25071
    if (ldovol) {  // 25073
      for (il = P.li; il <= P.lhab; ++il) {  // 25075
        if ((P.lvol(il) >= 1)) {  // 25077
          for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 25079
            an(igs(mgs), jy, kgs(mgs), P.lvol(il)) = c_max(NR(0.0), vx(mgs, il));  // 25081
          }  // 25082
        }  // 25084
      }  // 25086
    }  // 25088
    L9998:;  // 25100
    if (((kz > (nz - 1)) && (ix >= itile))) {  // 25102
      if ((ix >= itile)) {  // 25103
        goto L1200;  // 25104
      } else {  // 25105
        nzmpb = kz;  // 25106
      }  // 25107
    } else {  // 25108
      nzmpb = kz;  // 25109
    }  // 25110
    if ((ix >= itile)) {  // 25112
      nxmpb = 1;  // 25113
      nzmpb = (kz + 1);  // 25114
    } else {  // 25115
      nxmpb = (ix + 1);  // 25116
    }  // 25117
    L1000:; }  // 25119
    L1200:;  // 25120
    return;  // 25126

}
template <class Real>
CPPWRF_HD inline void nucond(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int na, int jyslab, int nor, int norz, Real dtp, int nxi, Real* dz3d_p, Real* t0_p, Real* t9_p, Real* an_p, Real* dn_p, Real* p2_p, Real* pn_p, Real* w_p, int ngs, Real* axtra_p, bool io_flag, Real* ssfilt_p, Real* t00_p, Real* t77_p, bool flag_qndrop) {
  const int ng1 = 1;  // 9746
  const Real cwmas30 = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3D4E66385C266145ull)));  // 9796
  const Real cwmas20 = ((NR(1000.0) * NR(0.523599)) * NR(hexd(0x3D3203AF9EE75617ull)));  // 9797
  const Real aa1 = NR(9.44e15);  // 9839
  const Real aa2 = NR(5.78e3);  // 9839
  const Real ssfcut = NR(4.0);  // 9876
  const int iunit = 0;  // 9944
  const Real cpv = NR(1885.0);  // 9950
  const Real mair = NR(0.0284);  // 9951
  const Real mwwater = NR(0.01801528);  // 9959
  const Real rhowater = NR(997.0);  // 9960
  const Real gasconst = NR(8.3144598);  // 9961
  const Real sscb = NR(2.0);
  const int idecss = 1;
  const int iba = 1;
  const int ifilt = 0;
  const Real supcb = NR(0.5);
  const Real supmx = NR(238.0);
  const Real epsi = NR(0.622);
  const Real d = NR(0.266);
  FA<Real> dz3d = mkfa<Real>(dz3d_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t9 = mkfa<Real>(t9_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> p2 = mkfa<Real>(p2_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> pn = mkfa<Real>(pn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> w = mkfa<Real>(w_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> axtra = mkfa<Real>(axtra_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-norz) + ng1), (nz + norz), 1, P.nxtra);
  FA<Real> ssfilt = mkfa<Real>(ssfilt_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t00 = mkfa<Real>(t00_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t77 = mkfa<Real>(t77_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  auto pb = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 9778 
  auto pinit = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 9779 
  Real dv = 0;  // 9790
  Real ccnefactwo = 0;  // 9791
  Real sstmp = 0;  // 9791
  Real cn1 = 0;  // 9791
  Real cnuctmp = 0;  // 9791
  int nxmpb = 0;  // 9798
  int nzmpb = 0;  // 9798
  int nxz = 0;  // 9798
  int mgs = 0;  // 9799
  int numgs = 0;  // 9799
  int inumgs = 0;  // 9799
  int ngscnt = 0;  // 9800
  auto igs = W.template arr<int>(1, ngs);  // 9800 
  auto kgs = W.template arr<int>(1, ngs);  // 9800 
  auto kgsp = W.template arr<int>(1, ngs);  // 9801 
  auto kgsm = W.template arr<int>(1, ngs);  // 9801 
  int nsvcnt = 0;  // 9802
  int ix = 0;  // 9804
  int kz = 0;  // 9804
  int i = 0;  // 9804
  int n = 0;  // 9804
  int kp1 = 0;  // 9804
  int km1 = 0;  // 9804
  int jy = 0;  // 9805
  int jgs = 0;  // 9805
  int ixb = 0;  // 9806
  int ixe = 0;  // 9806
  int jyb = 0;  // 9806
  int jye = 0;  // 9806
  int kzb = 0;  // 9806
  int kze = 0;  // 9806
  int itile = 0;  // 9808
  int jtile = 0;  // 9808
  int ktile = 0;  // 9808
  int ixend = 0;  // 9809
  int jyend = 0;  // 9809
  int kzend = 0;  // 9809
  int kzbeg = 0;  // 9809
  int nxend = 0;  // 9810
  int nyend = 0;  // 9810
  int nzend = 0;  // 9810
  int nzbeg = 0;  // 9810
  auto ccnc = W.template arr<Real>(1, ngs);  // 9817 
  auto ccna = W.template arr<Real>(1, ngs);  // 9817 
  auto cnuc = W.template arr<Real>(1, ngs);  // 9817 
  auto cwnccn = W.template arr<Real>(1, ngs);  // 9817 
  auto ccnaco = W.template arr<Real>(1, ngs);  // 9817 
  auto ccnanu = W.template arr<Real>(1, ngs);  // 9817 
  auto ccnc_nu = W.template arr<Real>(1, ngs);  // 9818 
  auto ccnc_ac = W.template arr<Real>(1, ngs);  // 9818 
  auto ccnc_co = W.template arr<Real>(1, ngs);  // 9818 
  auto ccncuf = W.template arr<Real>(1, ngs);  // 9819 
  Real temp1 = 0;  // 9830
  Real temp2 = 0;  // 9830
  auto ssmax = W.template arr<Real>(1, ngs);  // 9831 
  Real ssmx = 0;  // 9832
  Real dnnet = 0;  // 9833
  Real dqnet = 0;  // 9833
  auto ventrx = W.template arr<Real>(1, ngs);  // 9836 
  auto ventrxn = W.template arr<Real>(1, ngs);  // 9837 
  Real volb = 0;  // 9838
  Real t2s = 0;  // 9838
  auto rhoinv = W.template arr<Real>(1, ngs);  // 9841 
  Real chw = 0;  // 9843
  Real g1 = 0;  // 9843
  Real rd1 = 0;  // 9843
  Real ac1 = 0;  // 9845
  Real bc = 0;  // 9845
  Real taus = 0;  // 9845
  Real c1 = 0;  // 9845
  Real d1 = 0;  // 9845
  Real e1 = 0;  // 9845
  Real f1 = 0;  // 9845
  Real p380 = 0;  // 9845
  Real tmp = 0;  // 9845
  Real tmp2 = 0;  // 9845
  Real tmpmx = 0;  // 9846
  Real fw = 0;  // 9846
  Real qctmp = 0;  // 9846
  Real x = 0;  // 9847
  Real y = 0;  // 9847
  Real del = 0;  // 9847
  Real r = 0;  // 9847
  Real alpr = 0;  // 9847
  double vent1 = 0;  // 9848
  double vent2 = 0;  // 9848
  Real g1palp = 0;  // 9849
  Real bs = 0;  // 9850
  Real v1 = 0;  // 9851
  Real v2 = 0;  // 9851
  Real d1r = 0;  // 9852
  Real d1i = 0;  // 9852
  Real d1s = 0;  // 9852
  Real e1i = 0;  // 9852
  int nc = 0;  // 9853
  Real dtcon = 0;  // 9854
  Real dtcon1 = 0;  // 9854
  Real dtcon2 = 0;  // 9854
  Real delta = 0;  // 9855
  int ltemq1 = 0;  // 9856
  int ltemq1m = 0;  // 9856
  Real dqv = 0;  // 9857
  Real qv1 = 0;  // 9857
  Real ss1 = 0;  // 9857
  Real ss2 = 0;  // 9857
  Real qvs1 = 0;  // 9857
  Real dqvs = 0;  // 9857
  Real dtemp = 0;  // 9857
  Real dt1 = 0;  // 9857
  Real ssi1 = 0;  // 9859
  Real ssi2 = 0;  // 9859
  Real dqvi = 0;  // 9859
  Real dqvis = 0;  // 9859
  Real dqvii = 0;  // 9859
  Real qis1 = 0;  // 9859
  Real dqvr = 0;  // 9860
  Real dqc = 0;  // 9860
  Real dqr = 0;  // 9860
  Real dqi = 0;  // 9860
  Real dqs = 0;  // 9860
  Real qv1m = 0;  // 9861
  Real qvs1m = 0;  // 9861
  Real ss1m = 0;  // 9861
  Real ssi1m = 0;  // 9861
  Real qis1m = 0;  // 9861
  Real cwmastmp = 0;  // 9862
  Real dcloud = 0;  // 9863
  Real dcloud2 = 0;  // 9863
  Real dcrit = 0;  // 9864
  auto cn = W.template arr<Real>(1, ngs);  // 9865 
  auto cnuf = W.template arr<Real>(1, ngs);  // 9865 
  Real ccwmax = 0;  // 9866
  int ltemq = 0;  // 9869
  int il = 0;  // 9871
  auto es = W.template arr<Real>(1, ngs);  // 9873 
  auto ssf = W.template arr<Real>(1, ngs);  // 9875 
  auto ssfkp1 = W.template arr<Real>(1, ngs);  // 9875 
  auto ssfkm1 = W.template arr<Real>(1, ngs);  // 9875 
  auto ssat0 = W.template arr<Real>(1, ngs);  // 9875 
  auto ssfjp1 = W.template arr<Real>(1, ngs);  // 9877 
  auto ssfjm1 = W.template arr<Real>(1, ngs);  // 9877 
  auto ssfip1 = W.template arr<Real>(1, ngs);  // 9878 
  auto ssfim1 = W.template arr<Real>(1, ngs);  // 9878 
  Real r2dxm = 0;  // 9882
  Real r2dym = 0;  // 9882
  Real r2dzm = 0;  // 9882
  Real dssdz = 0;  // 9883
  Real dssdy = 0;  // 9883
  Real dssdx = 0;  // 9883
  Real r1 = 0;  // 9887
  Real qevap = 0;  // 9887
  Real vr = 0;  // 9889
  Real nrx = 0;  // 9889
  Real qr = 0;  // 9889
  Real z1 = 0;  // 9889
  Real z2 = 0;  // 9889
  Real rdi = 0;  // 9889
  Real alp = 0;  // 9889
  Real xnutmp = 0;  // 9889
  Real xnuc = 0;  // 9889
  Real ctmp = 0;  // 9890
  Real ccwtmp = 0;  // 9890
  Real f5 = 0;  // 9891
  Real qvs0 = 0;  // 9891
  Real t0p1 = 0;  // 9892
  Real t0p3 = 0;  // 9892
  Real qvex = 0;  // 9893
  auto dqvcnd = W.template arr<Real>(1, ngs);  // 9896 
  auto dqwv = W.template arr<Real>(1, ngs);  // 9896 
  auto dqcw = W.template arr<Real>(1, ngs);  // 9896 
  auto dqci = W.template arr<Real>(1, ngs);  // 9896 
  auto temp = W.template arr<Real>(1, ngs);  // 9897 
  auto tempc = W.template arr<Real>(1, ngs);  // 9897 
  auto temg = W.template arr<Real>(1, ngs);  // 9898 
  auto temcg = W.template arr<Real>(1, ngs);  // 9898 
  auto theta = W.template arr<Real>(1, ngs);  // 9898 
  auto qvap = W.template arr<Real>(1, ngs);  // 9898 
  auto temgx = W.template arr<Real>(1, ngs);  // 9899 
  auto temcgx = W.template arr<Real>(1, ngs);  // 9899 
  auto qvs = W.template arr<Real>(1, ngs);  // 9900 
  auto qis = W.template arr<Real>(1, ngs);  // 9900 
  auto qss = W.template arr<Real>(1, ngs);  // 9900 
  auto pqs = W.template arr<Real>(1, ngs);  // 9900 
  auto felv = W.template arr<Real>(1, ngs);  // 9901 
  auto felf = W.template arr<Real>(1, ngs);  // 9901 
  auto fels = W.template arr<Real>(1, ngs);  // 9901 
  auto felvcp = W.template arr<Real>(1, ngs);  // 9902 
  auto felvpi = W.template arr<Real>(1, ngs);  // 9902 
  auto gamw = W.template arr<Real>(1, ngs);  // 9903 
  auto gams = W.template arr<Real>(1, ngs);  // 9903 
  auto tsqr = W.template arr<Real>(1, ngs);  // 9904 
  auto ssi = W.template arr<Real>(1, ngs);  // 9904 
  auto ssw = W.template arr<Real>(1, ngs);  // 9904 
  auto cc3 = W.template arr<Real>(1, ngs);  // 9905 
  auto cqv1 = W.template arr<Real>(1, ngs);  // 9905 
  auto cqv2 = W.template arr<Real>(1, ngs);  // 9905 
  auto qcwtmp = W.template arr<Real>(1, ngs);  // 9906 
  Real qtmp = 0;  // 9906
  auto fvent = W.template arr<Real>(1, ngs);  // 9908 
  auto fwvdf = W.template arr<Real>(1, ngs);  // 9909 
  auto ftka = W.template arr<Real>(1, ngs);  // 9909 
  auto fthdf = W.template arr<Real>(1, ngs);  // 9909 
  auto fadvisc = W.template arr<Real>(1, ngs);  // 9910 
  auto fakvisc = W.template arr<Real>(1, ngs);  // 9910 
  auto fci = W.template arr<Real>(1, ngs);  // 9911 
  auto fcw = W.template arr<Real>(1, ngs);  // 9911 
  auto fschm = W.template arr<Real>(1, ngs);  // 9912 
  auto fpndl = W.template arr<Real>(1, ngs);  // 9912 
  auto pres = W.template arr<Real>(1, ngs);  // 9914 
  auto pipert = W.template arr<Real>(1, ngs);  // 9914 
  auto pk = W.template arr<Real>(1, ngs);  // 9915 
  auto rho0 = W.template arr<Real>(1, ngs);  // 9916 
  auto pi0 = W.template arr<Real>(1, ngs);  // 9916 
  auto rhovt = W.template arr<Real>(1, ngs);  // 9917 
  auto thetap = W.template arr<Real>(1, ngs);  // 9918 
  auto theta0 = W.template arr<Real>(1, ngs);  // 9918 
  auto qwvp = W.template arr<Real>(1, ngs);  // 9918 
  auto qv0 = W.template arr<Real>(1, ngs);  // 9918 
  auto thsave = W.template arr<Real>(1, ngs);  // 9919 
  auto qss0 = W.template arr<Real>(1, ngs);  // 9920 
  auto fcqv1 = W.template arr<Real>(1, ngs);  // 9921 
  auto wvel = W.template arr<Real>(1, ngs);  // 9922 
  auto wvelkm1 = W.template arr<Real>(1, ngs);  // 9922 
  auto wvdf = W.template arr<Real>(1, ngs);  // 9924 
  auto tka = W.template arr<Real>(1, ngs);  // 9924 
  auto advisc = W.template arr<Real>(1, ngs);  // 9925 
  auto rwvent = W.template arr<Real>(1, ngs);  // 9927 
  auto qx = W.template arr<Real>(1, ngs, P.lv, P.lhab);  // 9930 
  auto cx = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 9931 
  auto xv = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 9932 
  auto xmas = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 9933 
  auto xdn = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 9934 
  auto xdia = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);  // 9935 
  auto alpha = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 9936 
  auto zx = W.template arr<Real>(1, ngs, P.lr, P.lhab);  // 9937 
  auto zerocx = W.template arr<bool>(P.lc, P.lqmx);  // 9940 
  bool lprint = false;  // 9942
  Real frac = 0;  // 9946
  Real hwdn = 0;  // 9946
  Real tmpg = 0;  // 9946
  Real xdia1 = 0;  // 9946
  Real xdia3 = 0;  // 9946
  Real cwch = 0;  // 9946
  Real xvol = 0;  // 9946
  Real cvm = 0;  // 9948
  Real cpm = 0;  // 9948
  Real rmm = 0;  // 9948
  int kstag = 0;  // 9954
  int count = 0;  // 9956
  Real sswater = 0;  // 9962
  Real sigvl = 0;  // 9963
  Real aact = 0;  // 9963
  Real alpha_ar = 0;  // 9965
  Real gamma_ar = 0;  // 9965
  Real g_ar = 0;  // 9965
  Real evs = 0;  // 9965
  Real zeta = 0;  // 9965
  Real smax = 0;  // 9965
  Real f_ac = 0;  // 9966
  Real g_ac = 0;  // 9966
  Real eta_ac = 0;  // 9966
  Real f_nu = 0;  // 9967
  Real g_nu = 0;  // 9967
  Real eta_nu = 0;  // 9967
  Real f_co = 0;  // 9968
  Real g_co = 0;  // 9968
  Real eta_co = 0;  // 9968
  Real sm_nu = 0;  // 9970
  Real sm_ac = 0;  // 9970
  Real sm_co = 0;  // 9970
  Real ss_ac = 0;  // 9970
  Real ss_nu = 0;  // 9970
  Real ss_co = 0;  // 9970
  Real uu_nu = 0;  // 9971
  Real uu_ac = 0;  // 9971
  Real uu_co = 0;  // 9971
  Real cn_ac = 0;  // 9973
  Real cn_co = 0;  // 9973
  Real cn_nu = 0;  // 9973
  itile = nxi;  // 9976
  jtile = ny;  // 9977
  ktile = nz;  // 9978
  ixend = nxi;  // 9979
  jyend = ny;  // 9980
  kzend = nz;  // 9981
  nxend = (nxi + 1);  // 9982
  nyend = (ny + 1);  // 9983
  nzend = nz;  // 9984
  kzbeg = 1;  // 9985
  nzbeg = 1;  // 9986
  f5 = (((NR(237.3) * NR(17.27)) * NR(2.5e6)) / P.cp);  // 9989
  jy = 1;  // 9991
  kstag = 0;  // 9992
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pb(i_0) = NR(0.0);  // 9993
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pinit(i_0) = NR(0.0);  // 9994
  if (((P.ipconc <= 1) || (P.isedonly == 2))) { goto L2200; }  // 9996
  ssmx = 0;  // 10003
  count = 0;  // 10004
  for (kz = 1; kz <= (nz - kstag); ++kz) {  // 10006
    for (ix = 1; ix <= nxi; ++ix) {  // 10007
      temp1 = (an(ix, jy, kz, P.lt) * t77(ix, jy, kz));  // 10009
      t0(ix, jy, kz) = temp1;  // 10010
      ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 10011
      ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 10012
      c1 = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / (((pn(ix, jy, kz) + pb(kz)) - (P.esbolton * P.tabqvs(ltemq)))));  // 10018
      if ((c1 > NR(0.0))) {  // 10021
        ssfilt(ix, jy, kz) = (NR(100.0) * (((an(ix, jy, kz, P.lv) / c1) - NR(1.0))));  // 10022
      }  // 10023
    }  // 10025
  }  // 10026
  jgs = jy;  // 10033
  nxmpb = 1;  // 10040
  nzmpb = 1;  // 10041
  nxz = (nxi * nz);  // 10042
  numgs = ((nxz / ngs) + 1);  // 10043
  for (inumgs = 1; inumgs <= numgs; ++inumgs) {  // 10046
    ngscnt = 0;  // 10048
    kzb = nzmpb;  // 10051
    kze = (nz - kstag);  // 10052
    ixb = nxmpb;  // 10055
    ixe = itile;  // 10056
    for (kz = kzb; kz <= kze; ++kz) {  // 10058
      for (ix = nxmpb; ix <= nxi; ++ix) {  // 10059
        pres(1) = (pn(ix, jy, kz) + pb(kz));  // 10061
        pqs(1) = (NR(380.0) / ((pn(ix, jy, kz) + pb(kz))));  // 10062
        theta(1) = an(ix, jy, kz, P.lt);  // 10063
        temg(1) = t0(ix, jy, kz);  // 10064
        temcg(1) = (temg(1) - P.tfr);  // 10066
        ltemq = ((((temg(1) - NR(163.15))) / P.fqsat) + NR(1.5));  // 10067
        ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 10068
        qvs(1) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(1) - (P.esbolton * P.tabqvs(ltemq)))));  // 10073
        qis(1) = (pqs(1) * P.tabqis(ltemq));  // 10075
        qss(1) = qvs(1);  // 10077
        if ((temg(1) < P.tfr)) {  // 10080
        }  // 10081
        if (((((temg(1) > P.tfrh) || ((an(ix, jy, kz, P.lv) / qvs(1)) > P.maxlowtempss))) && ((((an(ix, jy, kz, P.lv) > qss(1)) || (an(ix, jy, kz, P.lc) > P.qxmin(P.lc))) || (((an(ix, jy, kz, P.lr) > P.qxmin(P.lr)) && (P.rcond == 2))))))) {  // 10083
          ngscnt = (ngscnt + 1);  // 10088
          igs(ngscnt) = ix;  // 10089
          kgs(ngscnt) = kz;  // 10090
          if ((ngscnt == ngs)) { goto L2100; }  // 10091
        }  // 10092
      }  // 10094
      nxmpb = 1;  // 10096
    }  // 10097
    L2100:;  // 10099
    if ((ngscnt == 0)) { goto L29998; }  // 10101
    for (int i_1 = P.lv; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) qx(i_0, i_1) = NR(0.0);  // 10108
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) cx(i_0, i_1) = NR(0.0);  // 10109
    for (int i_1 = P.lr; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) zx(i_0, i_1) = NR(0.0);  // 10110
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) xv(i_0, i_1) = NR(0.0);  // 10112
    for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) for (int i_0 = 1; i_0 <= ngs; ++i_0) xmas(i_0, i_1) = NR(0.0);  // 10113
    for (int i_0 = 1; i_0 <= ngs; ++i_0) alpha(i_0, P.lr) = P.alphar;  // 10116
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10124
      qx(mgs, P.lv) = an(igs(mgs), jy, kgs(mgs), P.lv);  // 10125
      for (il = P.lc; il <= P.lhab; ++il) {  // 10126
        qx(mgs, il) = c_max(an(igs(mgs), jy, kgs(mgs), il), NR(0.0));  // 10127
      }  // 10128
      qcwtmp(mgs) = qx(mgs, P.lc);  // 10130
      theta0(mgs) = an(igs(mgs), jy, kgs(mgs), P.lt);  // 10133
      thetap(mgs) = NR(0.0);  // 10134
      theta(mgs) = an(igs(mgs), jy, kgs(mgs), P.lt);  // 10135
      qv0(mgs) = qx(mgs, P.lv);  // 10136
      qwvp(mgs) = (qx(mgs, P.lv) - qv0(mgs));  // 10137
      pres(mgs) = (pn(igs(mgs), jy, kgs(mgs)) + pb(kgs(mgs)));  // 10139
      pipert(mgs) = p2(igs(mgs), jy, kgs(mgs));  // 10140
      rho0(mgs) = dn(igs(mgs), jy, kgs(mgs));  // 10141
      rhoinv(mgs) = (NR(1.0) / rho0(mgs));  // 10142
      rhovt(mgs) = sqrt((P.rho00 / rho0(mgs)));  // 10143
      pi0(mgs) = (p2(igs(mgs), jy, kgs(mgs)) + pinit(kgs(mgs)));  // 10144
      temg(mgs) = t0(igs(mgs), jy, kgs(mgs));  // 10145
      pk(mgs) = (p2(igs(mgs), jy, kgs(mgs)) + pinit(kgs(mgs)));  // 10147
      temcg(mgs) = (temg(mgs) - P.tfr);  // 10148
      qss0(mgs) = ((NR(380.0)) / (pres(mgs)));  // 10149
      pqs(mgs) = ((NR(380.0)) / (pres(mgs)));  // 10150
      ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 10151
      ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 10152
      qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 10157
      qis(mgs) = (pqs(mgs) * P.tabqis(ltemq));  // 10159
      qvap(mgs) = c_max(((qwvp(mgs) + qv0(mgs))), NR(0.0));  // 10161
      es(mgs) = (P.esbolton * P.tabqvs(ltemq));  // 10165
      qss(mgs) = qvs(mgs);  // 10168
      temgx(mgs) = c_min(temg(mgs), NR(313.15));  // 10171
      temgx(mgs) = c_max(temgx(mgs), NR(233.15));  // 10172
      felv(mgs) = (NR(2500837.367) * pow(((NR(273.15) / temgx(mgs))), (((NR(0.167)) + ((NR(3.67e-4)) * temgx(mgs))))));  // 10173
      felvcp(mgs) = (felv(mgs) * P.cpi);  // 10176
      temcgx(mgs) = c_min(temg(mgs), NR(273.15));  // 10198
      temcgx(mgs) = c_max(temcgx(mgs), NR(223.15));  // 10199
      temcgx(mgs) = (temcgx(mgs) - NR(273.15));  // 10200
      felf(mgs) = ((NR(333690.6098) + ((NR(2030.61425)) * temcgx(mgs))) - ((NR(10.46708312)) * ipow<2>(temcgx(mgs))));  // 10201
      fels(mgs) = (felv(mgs) + felf(mgs));  // 10203
      fcqv1(mgs) = ((NR(4098.0258) * felv(mgs)) * P.cpi);  // 10204
      wvdf(mgs) = (((NR(2.11e-05)) * (pow(((temg(mgs) / P.tfr)), NR(1.94)))) * ((NR(101325.0) / ((pb(kgs(mgs)) + pn(igs(mgs), jgs, kgs(mgs)))))));  // 10206
      advisc(mgs) = ((P.advisc0 * ((NR(416.16) / ((temg(mgs) + NR(120.0)))))) * pow(((temg(mgs) / NR(296.0))), (NR(1.5))));  // 10208
      tka(mgs) = ((P.tka0 * advisc(mgs)) / P.advisc1);  // 10210
    }  // 10213
    if ((P.ipconc >= 1)) {  // 10220
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10221
        cx(mgs, P.li) = c_max(an(igs(mgs), jy, kgs(mgs), P.lni), NR(0.0));  // 10222
      }  // 10223
    }  // 10224
    if ((P.ipconc >= 2)) {  // 10225
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10226
        cx(mgs, P.lc) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnc), NR(0.0));  // 10227
        cwnccn(mgs) = ((P.cwccn * rho0(mgs)) / P.rho00);  // 10228
        cn(mgs) = NR(0.0);  // 10229
        ssmax(mgs) = NR(0.0);  // 10233
        if (((P.lccn > 1) && (P.ac_opt == 0))) {  // 10235
          ccnc(mgs) = an(igs(mgs), jy, kgs(mgs), P.lccn);  // 10239
          if ((P.lccna > 1)) {  // 10240
            cnuc(mgs) = ccnc(mgs);  // 10241
          }  // 10242
          ccnaco(mgs) = NR(0.0);  // 10253
          ccnanu(mgs) = NR(0.0);  // 10258
        } else {  // 10271
          ccnc(mgs) = cwnccn(mgs);  // 10272
        }  // 10273
        ccncuf(mgs) = NR(0.0);  // 10277
        cnuf(mgs) = NR(0.0);  // 10279
        if ((P.lccna > 1)) {  // 10280
          ccna(mgs) = an(igs(mgs), jy, kgs(mgs), P.lccna);  // 10281
        } else {  // 10294
          if ((P.lccn > 1)) {  // 10295
            ccna(mgs) = NR(0.0);  // 10296
          } else {  // 10297
            ccna(mgs) = cx(mgs, P.lc);  // 10298
          }  // 10299
        }  // 10300
      }  // 10301
    }  // 10302
    if ((P.ipconc >= 3)) {  // 10303
      for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10304
        cx(mgs, P.lr) = c_max(an(igs(mgs), jy, kgs(mgs), P.lnr), NR(0.0));  // 10305
      }  // 10306
    }  // 10307
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10310
      if ((P.irenuc != 6)) {  // 10312
        if ((P.irenuc == 2)) {  // 10313
          cnuc(mgs) = ((c_max(ccnc(mgs), cwnccn(mgs)) * ((NR(1.0) - P.renucfrac))) + (ccnc(mgs) * P.renucfrac));  // 10314
        } else {  // 10315
          cnuc(mgs) = ((ccnc(mgs) * ((NR(1.0) - P.renucfrac))) + (ccnc(mgs) * P.renucfrac));  // 10316
        }  // 10317
      } else {  // 10318
        cnuc(mgs) = ((ccnc(mgs) * ((NR(1.0) - P.renucfrac))) + (c_max(NR(0.0), (ccnc(mgs) - ccna(mgs))) * P.renucfrac));  // 10319
      }  // 10320
    }  // 10330
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10336
      xdn(mgs, P.lc) = P.xdn0(P.lc);  // 10337
      xdn(mgs, P.lr) = P.xdn0(P.lr);  // 10338
    }  // 10339
    for (int i_0 = 1; i_0 <= ngs; ++i_0) ventrx(i_0) = P.ventr;  // 10341
    for (int i_0 = 1; i_0 <= ngs; ++i_0) ventrxn(i_0) = P.ventrn;  // 10342
    ssmx = NR(0.0);  // 10578
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10579
      kp1 = c_min(nz, (kgs(mgs) + 1));  // 10581
      wvel(mgs) = ((NR(0.5)) * ((w(igs(mgs), jgs, kp1) + w(igs(mgs), jgs, kgs(mgs)))));  // 10582
      wvelkm1(mgs) = ((NR(0.5)) * ((w(igs(mgs), jgs, kgs(mgs)) + w(igs(mgs), jgs, c_max(1, (kgs(mgs) - 1))))));  // 10584
      ssat0(mgs) = ssfilt(igs(mgs), jgs, kgs(mgs));  // 10587
      ssf(mgs) = ssfilt(igs(mgs), jgs, kgs(mgs));  // 10588
      ssfkp1(mgs) = ssfilt(igs(mgs), jgs, c_min((nz - 1), (kgs(mgs) + 1)));  // 10592
      ssfkm1(mgs) = ssfilt(igs(mgs), jgs, c_max(1, (kgs(mgs) - 1)));  // 10593
    }  // 10598
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10608
      xv(mgs, P.lc) = NR(0.0);  // 10609
      if (((P.ipconc >= 2) && (cx(mgs, P.lc) > NR(1.0e6)))) {  // 10610
        xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), P.cwmasn), P.cwmasx);  // 10611
        xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 10613
      } else {  // 10614
        if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (cx(mgs, P.lc) > P.cxmin))) {  // 10615
          xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), (xdn(mgs, P.lc) * P.xvmn(P.lc))), (xdn(mgs, P.lc) * P.xvmx(P.lc)));  // 10616
          cx(mgs, P.lc) = ((qx(mgs, P.lc) * rho0(mgs)) / xmas(mgs, P.lc));  // 10620
        } else if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (cx(mgs, P.lc) <= P.cxmin))) {  // 10622
          cx(mgs, P.lc) = c_max(P.cxmin, ((rho0(mgs) * qx(mgs, P.lc)) / P.cwmasx));  // 10625
          xmas(mgs, P.lc) = c_min(c_max(((qx(mgs, P.lc) * rho0(mgs)) / cx(mgs, P.lc)), P.cwmasn), P.cwmasx);  // 10626
          xv(mgs, P.lc) = (xmas(mgs, P.lc) / xdn(mgs, P.lc));  // 10628
        } else {  // 10630
          xmas(mgs, P.lc) = P.cwmasn;  // 10631
        }  // 10632
      }  // 10633
      xdia(mgs, P.lc, 1) = pow(((xmas(mgs, P.lc) * P.cwc1)), P.c1f3);  // 10634
    }  // 10637
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10641
      if ((qx(mgs, P.lr) > P.qxmin(P.lr))) {  // 10642
        if ((P.ipconc >= 3)) {  // 10644
          xv(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((xdn(mgs, P.lr) * c_max(NR(1.0e-9), cx(mgs, P.lr)))));  // 10645
          if ((xv(mgs, P.lr) > P.xvmx(P.lr))) {  // 10647
            xv(mgs, P.lr) = P.xvmx(P.lr);  // 10648
            cx(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((P.xvmx(P.lr) * xdn(mgs, P.lr))));  // 10649
          } else if ((xv(mgs, P.lr) < P.xvmn(P.lr))) {  // 10650
            xv(mgs, P.lr) = P.xvmn(P.lr);  // 10651
            cx(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((P.xvmn(P.lr) * xdn(mgs, P.lr))));  // 10652
          }  // 10653
          xmas(mgs, P.lr) = (xv(mgs, P.lr) * xdn(mgs, P.lr));  // 10655
          xdia(mgs, P.lr, 3) = pow(((xmas(mgs, P.lr) * P.cwc1)), ((NR(1.0) / NR(3.0))));  // 10656
          xdia(mgs, P.lr, 1) = pow(((((NR(6.0) * P.piinv) * xv(mgs, P.lr)) / (((((alpha(mgs, P.lr) + NR(3.0))) * ((alpha(mgs, P.lr) + NR(2.0)))) * ((alpha(mgs, P.lr) + NR(1.0))))))), ((NR(1.0) / NR(3.0))));  // 10661
        } else {  // 10669
          xdia(mgs, P.lr, 1) = pow((((qx(mgs, P.lr) * rho0(mgs)) / (((P.pi * xdn(mgs, P.lr)) * P.cno(P.lr))))), (NR(0.25)));  // 10670
        }  // 10672
      } else {  // 10673
        xdia(mgs, P.lr, 1) = NR(1.0e-9);  // 10674
      }  // 10676
    }  // 10678
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10684
      fadvisc(mgs) = ((P.advisc0 * ((NR(416.16) / ((temg(mgs) + NR(120.0)))))) * vpow(((temg(mgs) / NR(296.0))), (NR(1.5)), mgs - 1, ngscnt));  // 10687  // libmvec 2-lane pow (vectorised loop)
      fakvisc(mgs) = (fadvisc(mgs) * rhoinv(mgs));  // 10690
      fwvdf(mgs) = (((NR(2.11e-05)) * (vpow(((temg(mgs) / P.tfr)), NR(1.94), mgs - 1, ngscnt))) * ((NR(101325.0) / (pres(mgs)))));  // 10692  // libmvec 2-lane pow (vectorised loop)
      fschm(mgs) = ((fakvisc(mgs) / fwvdf(mgs)));  // 10695
      fvent(mgs) = ((vpow(fschm(mgs), ((NR(1.0) / NR(3.0))), mgs - 1, ngscnt)) * (vpow(fakvisc(mgs), ((-NR(0.5))), mgs - 1, ngscnt)));  // 10697  // libmvec 2-lane pow (vectorised loop)
    }  // 10699
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 10708
      dcloud = NR(0.0);  // 10709
      if (((temg(mgs) <= P.tfrh) && ((qx(mgs, P.lv) / qvs(mgs)) < P.maxlowtempss))) {  // 10712
        continue;  // 10713
      }  // 10714
      if (((ssat0(mgs) > NR(0.0)) || (ssf(mgs) > NR(0.0)))) { goto L620; }  // 10716
      if ((qx(mgs, P.lc) <= NR(0.0))) { goto L631; }  // 10722
      r1 = (NR(1.0) / ((NR(1.0) + ((((P.caw * ((NR(273.15) - P.cbw))) * qss(mgs)) * felv(mgs)) / ((P.cp * ipow<2>(((temg(mgs) - P.cbw)))))))));  // 10725
      qevap = c_min(qx(mgs, P.lc), (r1 * ((qss(mgs) - qvap(mgs)))));  // 10727
      if ((qx(mgs, P.lc) <= qevap)) {  // 10730
        qwvp(mgs) = (qwvp(mgs) + qx(mgs, P.lc));  // 10731
        thetap(mgs) = (thetap(mgs) - ((felvcp(mgs) * qx(mgs, P.lc)) / (pi0(mgs))));  // 10732
        qx(mgs, P.lc) = NR(0.0);  // 10736
        if ((P.lccna > 1)) {  // 10738
          tmp = (P.restoreccnfrac * cx(mgs, P.lc));  // 10739
          ccna(mgs) = (ccna(mgs) - tmp);  // 10749
        } else if ((P.irenuc <= 2)) {  // 10751
          ccnc(mgs) = c_max(ccnc(mgs), c_min((P.qccn * rho0(mgs)), (ccnc(mgs) + (P.restoreccnfrac * cx(mgs, P.lc)))));  // 10753
        }  // 10757
        cx(mgs, P.lc) = NR(0.0);  // 10759
      } else {  // 10760
        qctmp = qx(mgs, P.lc);  // 10761
        qwvp(mgs) = (qwvp(mgs) + qevap);  // 10762
        qx(mgs, P.lc) = (qx(mgs, P.lc) - qevap);  // 10763
        if ((qx(mgs, P.lc) <= NR(0.0))) {  // 10764
          if ((P.lccna > 1)) {  // 10766
            tmp = (P.restoreccnfrac * cx(mgs, P.lc));  // 10767
            ccna(mgs) = (ccna(mgs) - tmp);  // 10777
          } else if ((P.irenuc <= 2)) {  // 10779
            ccnc(mgs) = c_max(ccnc(mgs), c_min((P.qccn * rho0(mgs)), (ccnc(mgs) + (P.restoreccnfrac * cx(mgs, P.lc)))));  // 10783
          }  // 10787
          cx(mgs, P.lc) = NR(0.0);  // 10789
        } else {  // 10790
          tmp = (((NR(0.9) * qevap) * cx(mgs, P.lc)) / qctmp);  // 10791
          if ((P.lccna > 1)) {  // 10793
            tmp = (P.restoreccnfrac * tmp);  // 10794
            ccna(mgs) = (ccna(mgs) - tmp);  // 10804
          } else if ((P.irenuc <= 2)) {  // 10807
            ccnc(mgs) = c_max(ccnc(mgs), c_min((P.qccn * rho0(mgs)), (ccnc(mgs) + (P.restoreccnfrac * tmp))));  // 10811
          }  // 10815
          cx(mgs, P.lc) = (cx(mgs, P.lc) - tmp);  // 10817
        }  // 10818
        thetap(mgs) = (thetap(mgs) - ((felvcp(mgs) * qevap) / (pi0(mgs))));  // 10819
      }  // 10824
      goto L631;  // 10826
      L620:;  // 10829
      if (((qx(mgs, P.lc) > P.qxmin(P.lc)) && (cx(mgs, P.lc) >= NR(1.0)))) {  // 10833
        ac1 = (ipow<2>(felv(mgs)) / (((tka(mgs) * P.rw) * ipow<2>(temg(mgs)))));  // 10840
        bc = ((P.rw * temg(mgs)) / ((wvdf(mgs) * es(mgs))));  // 10846
        if (((ssf(mgs) > NR(0.0)) || (ssat0(mgs) > NR(0.0)))) {  // 10855
          if ((ny <= 2)) {  // 10856
          }  // 10859
          if ((qx(mgs, P.lc) > P.qxmin(P.lc))) {  // 10863
            if ((xdia(mgs, P.lc, 1) <= NR(0.0))) {  // 10865
              xmas(mgs, P.lc) = P.cwmasn;  // 10866
              xdia(mgs, P.lc, 1) = pow(((xmas(mgs, P.lc) * P.cwc1)), P.c1f3);  // 10867
            }  // 10868
            d1 = (((((((((NR(1.0) / ((ac1 + bc)))) * NR(4.0)) * P.pi) * P.ventc) * NR(0.5)) * xdia(mgs, P.lc, 1)) * cx(mgs, P.lc)) * rhoinv(mgs));  // 10869
          } else {  // 10872
            d1 = NR(0.0);  // 10873
          }  // 10874
          if ((((P.rcond == 2) && (qx(mgs, P.lr) > P.qxmin(P.lr))) && (cx(mgs, P.lr) > NR(1.0e-9)))) {  // 10876
            x = (NR(1.0) + alpha(mgs, P.lr));  // 10919
            rwvent(mgs) = (((NR(0.78) * x) + ((((NR(0.308) * ventrxn(mgs)) * fvent(mgs)) * sqrt(((P.ar * rhovt(mgs))))) * (pow(xdia(mgs, P.lr, 1), ((((NR(1.0) + P.br)) / NR(2.0))))))));  // 10921
            d1r = (((((((((NR(1.0) / ((ac1 + bc)))) * NR(4.0)) * P.pi) * rwvent(mgs)) * NR(0.5)) * xdia(mgs, P.lr, 1)) * cx(mgs, P.lr)) * rhoinv(mgs));  // 10931
          } else {  // 10933
            d1r = NR(0.0);  // 10934
          }  // 10935
          e1 = (felvcp(mgs) / (pi0(mgs)));  // 10938
          f1 = pk(mgs);  // 10939
          ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 10944
          ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 10945
          ltemq1 = ltemq;  // 10946
          temp1 = temg(mgs);  // 10947
          p380 = ((P.esbolton * P.rdorv) / ((pres(mgs) - es(mgs))));  // 10951
          ss1 = (qx(mgs, P.lv) / qvs(mgs));  // 10957
          ss2 = ss1;  // 10958
          temp2 = temp1;  // 10959
          qv1 = qx(mgs, P.lv);  // 10960
          qvs1 = qvs(mgs);  // 10961
          qis1 = qis(mgs);  // 10962
          dt1 = NR(0.0);  // 10963
          ltemq1 = ltemq;  // 10969
          if ((fabs((ss1 - NR(1.0))) > NR(1.0e-5))) {  // 10974
            delta = ((NR(0.5) * ((qv1 - qvs1))) / ((d1 * ((ss1 - NR(1.0))))));  // 10975
          } else {  // 10976
            delta = (NR(0.1) * dtp);  // 10977
          }  // 10978
          dtcon1 = c_min(NR(0.05), (NR(0.2) * delta));  // 10982
          nc = c_max(5, (2 * nint((((dtp - (NR(4.0) * dtcon1))) / delta))));  // 10983
          dtcon2 = (((dtp - (NR(4.0) * dtcon1))) / nc);  // 10984
          n = 1;  // 10986
          dt1 = NR(0.0);  // 10987
          nc = 0;  // 10988
          dqc = NR(0.0);  // 10989
          dqr = NR(0.0);  // 10990
          dqi = NR(0.0);  // 10991
          dqs = NR(0.0);  // 10992
          dqvii = NR(0.0);  // 10993
          dqvis = NR(0.0);  // 10994
          while (dt1 < dtp) {  // RK2c  // 10996
          nc = 0;  // 10997
          if ((n <= 4)) {  // 10998
            dtcon = dtcon1;  // 10999
          } else {  // 11000
            dtcon = dtcon2;  // 11001
          }  // 11002
          L609: dqv = (-((((ss1 - NR(1.0))) * d1) * dtcon));  // 11003
          dqvr = (-((((ss1 - NR(1.0))) * d1r) * dtcon));  // 11004
          dtemp = (-(((NR(0.5) * e1) * f1) * ((dqv + dqvr))));  // 11005
          ltemq1m = (((((temp1 + dtemp) - NR(163.15))) * P.fqsati) + NR(1.5));  // 11011
          ltemq1m = c_min(P.nqsat, c_max(1, ltemq1m));  // 11012
          if (((ltemq1m < 1) || (ltemq1m > P.nqsat))) {  // 11014
            // write(0,*) 'STOP in nucond line 1192 '
            // write(0,*) ' ltemq1m,icond = ',ltemq1m,icond
            // write(0,*) ' dtemp,e1,f1,dqv,dqvr = ', dtemp,e1,f1,dqv,dqvr
            // write(0,*) ' d1,d1r,dtcon,ss1 = ',d1,d1r,dtcon,ss1
            // write(0,*) ' dqc, dqr = ',dqc,dqr
            // write(0,*) ' qv,qc,qr = ',qx(mgs,lv)*1000.,qx(mgs,lc)*1000.,qx(mgs,lr)*1000.
            // write(0,*) ' i, j, k = ',igs(mgs),jy,kgs(mgs)
            // write(0,*) ' dtcon1,dtcon2,delta = ',dtcon1,dtcon2,delta
            // write(0,*) ' nc,dtp = ',nc,dtp
            // write(0,*) ' rwvent,xdia,crw,ccw = ', rwvent(mgs),xdia(mgs,lr,1),cx(mgs,lr),cx(mgs,lc)
            // write(0,*) ' fvent,alphar = ',fvent(mgs),alpha(mgs,lr)
            // write(0,*) ' xvr,xmasr,xdnr,cwc1 = ',xv(mgs,lr),xmas(mgs,lr),xdn(mgs,lr),cwc1
          }  // 11027
          dqvs = ((dtemp * p380) * P.dtabqvs(ltemq1m));  // 11028
          qv1m = ((qv1 + dqv) + dqvr);  // 11029
          qvs1m = (qvs1 + dqvs);  // 11032
          ss1m = (qv1m / qvs1m);  // 11033
          if (((ss1m < NR(1.0)) && (((dqvii + dqvis)) == NR(0.0)))) {  // 11036
            dtcon = ((NR(0.5) * dtcon));  // 11037
            if ((dtcon >= dtcon1)) {  // 11038
              goto L609;  // 11039
            } else {  // 11040
              break;  // 11041
            }  // 11042
          }  // 11043
          dqv = (-((((ss1m - NR(1.0))) * d1) * dtcon));  // 11045
          dqvr = (-((((ss1m - NR(1.0))) * d1r) * dtcon));  // 11046
          dtemp = (-((e1 * f1) * ((dqv + dqvr))));  // 11050
          ltemq1 = (((((temp1 + dtemp) - NR(163.15))) * P.fqsati) + NR(1.5));  // 11055
          ltemq1 = c_min(P.nqsat, c_max(1, ltemq1));  // 11056
          if (((ltemq1 < 1) || (ltemq1 > P.nqsat))) {  // 11058
            // write(0,*) 'STOP in nucond line 1230 '
            // write(0,*) ' ltemq1m,icond = ',ltemq1m,icond
            // write(0,*) ' dtemp,e1,dqv,dqvr = ', dtemp,e1,dqv,dqvr
          }  // 11062
          dqvs = ((dtemp * p380) * P.dtabqvs(ltemq1));  // 11063
          qv1 = ((qv1 + dqv) + dqvr);  // 11065
          dqc = (dqc - dqv);  // 11067
          dqr = (dqr - dqvr);  // 11068
          qvs1 = (qvs1 + dqvs);  // 11070
          ss1 = (qv1 / qvs1);  // 11071
          temp1 = (temp1 + dtemp);  // 11072
          if (((((temp2 == temp1) || (ss2 == ss1)) || (ss1 == NR(1.00))) || (((n > 10) && (ss1 < NR(1.0005)))))) {  // 11073
            break;  // 11077
          } else {  // 11078
            ss2 = ss1;  // 11079
            temp2 = temp1;  // 11080
            dt1 = (dt1 + dtcon);  // 11081
            n = (n + 1);  // 11082
          }  // 11083
          }  // RK2c  // 11084
          dcloud = dqc;  // 11087
          thetap(mgs) = (thetap(mgs) + (e1 * ((dcloud + dqr))));  // 11088
          qwvp(mgs) = (qwvp(mgs) - ((dcloud + dqr)));  // 11098
          qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11099
          qx(mgs, P.lr) = (qx(mgs, P.lr) + dqr);  // 11100
          theta(mgs) = (thetap(mgs) + theta0(mgs));  // 11118
          temg(mgs) = (theta(mgs) * f1);  // 11119
          ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 11120
          ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11121
          qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 11125
        }  // 11131
      } else {  // 11134
        if ((ssf(mgs) > NR(0.0))) {  // 11137
          ssmx = P.ssmxinit;  // 11156
          if ((((ssf(mgs) > ssmx) && (ssf(mgs) < NR(20.0))) && (((ccnc(mgs) > (NR(0.05) * cwnccn(mgs))) || (((P.ac_opt > 0) && ((ccnc(mgs) - cx(mgs, P.lc)) > NR(0.0)))))))) {  // 11163
            qvexcess(P, W, ngs, mgs, qwvp.p, qv0.p, qx.at(1, P.lc), pres.p, thetap.p, theta0.p, dcloud, pi0.p, CP(P.tabqvs_p), P.nqsat, P.fqsat, P.cbw, fcqv1.p, felvcp.p, ssmx, pk.p, ngscnt);  // 11166
          } else {  // 11168
            dcloud = NR(0.0);  // 11169
          }  // 11170
        } else {  // 11172
          dcloud = NR(0.0);  // 11173
        }  // 11174
        thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11176
        qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11177
        qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11178
        theta(mgs) = (thetap(mgs) + theta0(mgs));  // 11182
        temg(mgs) = (theta(mgs) * pk(mgs));  // 11183
        ltemq = ((((temg(mgs) - NR(163.15))) / P.fqsat) + NR(1.5));  // 11185
        ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11186
        qvs(mgs) = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 11191
        cn(mgs) = NR(0.0);  // 11197
        if ((!flag_qndrop)) {  // 11203
          cnuctmp = cnuc(mgs);  // 11206
          if (((dcloud > P.qxmin(P.lc)) && (wvel(mgs) > NR(0.0)))) {  // 11212
            cn(mgs) = ((P.ccne0 * pow(cnuctmp, ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(wvel(mgs), P.cnexp));  // 11214
            cn(mgs) = c_min(((P.cwccn * rho0(mgs)) / P.rho00), c_max(cn(mgs), ((rho0(mgs) * qx(mgs, P.lc)) / (((xdn(mgs, P.lc) * (((NR(4.0) * P.pi) / NR(3.0)))) * NR(hexd(0x3C92725DD1D243ABull)))))));  // 11225
          } else {  // 11228
            cn(mgs) = NR(0.0);  // 11229
            dcloud = NR(0.0);  // 11230
          }  // 11233
          if ((cn(mgs) > NR(0.0))) {  // 11235
            if ((cn(mgs) > ccnc(mgs))) {  // 11237
              cn(mgs) = ccnc(mgs);  // 11238
            }  // 11240
            if (((P.irenuc <= 2) && (P.lccna < 1))) { ccnc(mgs) = c_max(NR(0.0), (ccnc(mgs) - cn(mgs))); }  // 11245
            ccna(mgs) = (ccna(mgs) + cn(mgs));  // 11246
          }  // 11247
          if ((cn(mgs) > cx(mgs, P.lc))) { cx(mgs, P.lc) = cn(mgs); }  // 11251
          if (((cx(mgs, P.lc) > NR(0.0)) && (qx(mgs, P.lc) <= P.qxmin(P.lc)))) {  // 11252
            cx(mgs, P.lc) = NR(0.0);  // 11253
          } else {  // 11254
            cx(mgs, P.lc) = c_min(cx(mgs, P.lc), ((rho0(mgs) * c_max(NR(0.0), qx(mgs, P.lc))) / P.cwmasn));  // 11255
          }  // 11256
        }  // 11258
        goto L613;  // 11260
      }  // 11262
      if ((wvel(mgs) <= NR(0.0))) { goto L616; }  // 11271
      if ((cx(mgs, P.lc) <= NR(0.0))) { goto L613; }  // 11272
      if (((((kzbeg - 1) + kgs(mgs)) > 1) && (qx(mgs, P.lc) <= P.qxmin(P.lc)))) { goto L613; }  // 11273
      if (((((kzbeg - 1) + kgs(mgs)) == 1) && (wvel(mgs) > NR(0.0)))) { goto L613; }  // 11274
      L616: if (((ssf(mgs) <= supcb) && (wvel(mgs) > NR(0.0)))) { goto L631; }  // 11276
      if ((((((kzbeg - 1) + kgs(mgs)) > 1) && (((kzbeg - 1) + kgs(mgs)) < (nzend - 1))) && ((((ssfkp1(mgs) >= supmx) || (ssf(mgs) >= supmx)) || (ssfkm1(mgs) >= supmx))))) { goto L631; }  // 11277
      if (((ssf(mgs) < NR(1.0e-10)) || (ssf(mgs) >= supmx))) { goto L631; }  // 11281
      dssdz = NR(0.0);  // 11289
      r2dzm = (NR(0.50) / dz3d(igs(mgs), jy, kgs(mgs)));  // 11290
      if ((((P.irenuc >= 0) && (P.ac_opt == 0)) && (!flag_qndrop))) {  // 11292
        if ((P.irenuc < 2)) {  // 11294
          if ((kzend == nzend)) {  // 11296
            t0p3 = t0(igs(mgs), jgs, c_min(kze, (kgs(mgs) + 3)));  // 11297
            t0p1 = t0(igs(mgs), jgs, c_min(kze, (kgs(mgs) + 1)));  // 11298
          } else {  // 11299
            t0p3 = t0(igs(mgs), jgs, (kgs(mgs) + 3));  // 11300
            t0p1 = t0(igs(mgs), jgs, (kgs(mgs) + 1));  // 11301
          }  // 11302
          if (((((ssf(mgs) > ssmax(mgs)) || (P.irenuc == 1))) && (((((P.lccn < 1) && (cx(mgs, P.lc) < (P.cwccn * (c_min(NR(1.0), rho0(mgs))))))) || (((P.lccn > 1) && (ccnc(mgs) > NR(0.0)))))))) {  // 11304
            if ((((((((((((kzbeg - 1) + kgs(mgs)) > 1) && (((kzbeg - 1) + kgs(mgs)) < (nzend - 1))) && (ssf(mgs) > NR(0.0))) && (ssfkp1(mgs) < supmx)) && (ssfkp1(mgs) >= NR(0.0))) && (ssfkm1(mgs) < supmx)) && (ssfkm1(mgs) >= NR(0.0))) && (ssfkp1(mgs) > ssfkm1(mgs))) && (t0p3 > NR(233.2)))) {  // 11309
              dssdz = (((ssfkp1(mgs) - ssfkm1(mgs))) * r2dzm);  // 11315
            } else if ((((((((((((kzbeg - 1) + kgs(mgs)) > 1) && (((kzbeg - 1) + kgs(mgs)) < (nzend - 1))) && (ssf(mgs) > NR(0.0))) && (wvel(mgs) > NR(0.0))) && (ssfkp1(mgs) > NR(0.0))) && (ssfkm1(mgs) <= NR(0.0))) && (wvelkm1(mgs) > NR(0.0))) && (ssf(mgs) > ssfkm1(mgs))) && (t0p1 > NR(233.2)))) {  // 11319
              dssdz = ((NR(2.0) * ((ssf(mgs) - ssfkm1(mgs)))) * r2dzm);  // 11326
            }  // 11327
          }  // 11329
          c1 = c_max(NR(0.0), ((rho0(mgs) * ((qx(mgs, P.lv) - qss(mgs)))) / (((xdn(mgs, P.lc) * (((NR(4.0) * P.pi) / NR(3.0)))) * NR(hexd(0x3C92725DD1D243ABull))))));  // 11333
          if ((P.lccn < 1)) {  // 11335
            cn(mgs) = ((((((P.cwccn * rho0(mgs)) / P.rho00) * P.cck) * pow(ssf(mgs), P.cckm)) * dtp) * c_max(NR(0.0), ((wvel(mgs) * dssdz))));  // 11336
          } else {  // 11339
            cn(mgs) = c_min(ccnc(mgs), ((((cnuc(mgs) * P.cck) * pow(ssf(mgs), P.cckm)) * dtp) * c_max(NR(0.0), ((wvel(mgs) * dssdz)))));  // 11340
          }  // 11345
          if ((cn(mgs) > NR(0.0))) {  // 11347
            if (((ccnc(mgs) < NR(5.0e7)) && (cn(mgs) >= NR(5.0e7)))) {  // 11348
              cn(mgs) = NR(5.0e7);  // 11349
              ccnc(mgs) = NR(0.0);  // 11350
            } else if ((cn(mgs) > ccnc(mgs))) {  // 11351
              cn(mgs) = ccnc(mgs);  // 11352
              ccnc(mgs) = NR(0.0);  // 11353
            }  // 11354
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11355
            ccnc(mgs) = c_max(NR(0.0), (ccnc(mgs) - cn(mgs)));  // 11356
          }  // 11357
        } else if ((P.irenuc == 2)) {  // 11359
          cn(mgs) = ((P.ccne0 * pow(cnuc(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp));  // 11362
          cn(mgs) = c_min(cn(mgs), ccnc(mgs));  // 11367
          cn(mgs) = c_min(cn(mgs), ((NR(0.5) * dqc) / P.cwmasn));  // 11368
          cn(mgs) = c_min(cn(mgs), c_max(NR(0.0), ((cnuc(mgs) - ccna(mgs)))));  // 11369
          cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11395
          if ((P.lccna < 1)) { ccnc(mgs) = c_max(NR(0.0), (ccnc(mgs) - cn(mgs))); }  // 11397
        } else if ((P.irenuc == 3)) {  // 11399
          temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11405
          ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11406
          ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11407
          c1 = (pqs(mgs) * P.tabqvs(ltemq));  // 11409
          ssf(mgs) = NR(0.0);  // 11411
          if ((c1 > NR(0.0))) {  // 11412
            ssf(mgs) = (NR(100.0) * (((qx(mgs, P.lv) / c1) - NR(1.0))));  // 11413
          }  // 11414
          cn(mgs) = (cnuc(mgs) * c_min(NR(1.0), pow((ssf(mgs)), P.cck)));  // 11415
          cn(mgs) = c_max(NR(0.0), (cn(mgs) - ccna(mgs)));  // 11417
          cn(mgs) = c_min(cn(mgs), ccnc(mgs));  // 11420
          cn(mgs) = c_min(cn(mgs), ((NR(0.5) * dqc) / P.cwmasn));  // 11421
          cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11423
          ccnc(mgs) = c_max(NR(0.0), (ccnc(mgs) - cn(mgs)));  // 11427
        } else if ((P.irenuc == 4)) {  // 11429
          temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11435
          ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11436
          ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11437
          c1 = (pqs(mgs) * P.tabqvs(ltemq));  // 11439
          if ((c1 > NR(0.0))) {  // 11440
            ssf(mgs) = c_max(NR(0.0), (NR(100.0) * (((((qv0(mgs) + qwvp(mgs))) / c1) - NR(1.0)))));  // 11441
          } else {  // 11442
            ssf(mgs) = NR(0.0);  // 11443
          }  // 11444
          cn(mgs) = (cnuc(mgs) * c_min(P.ssf2kmax, pow(ssf(mgs), P.cck)));  // 11445
          cn(mgs) = c_max(NR(0.0), (cn(mgs) - ccna(mgs)));  // 11447
          cn(mgs) = c_min(cn(mgs), ((NR(0.5) * dqc) / P.cwmasn));  // 11451
          if ((cn(mgs) > NR(0.0))) {  // 11453
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11454
            dcrit = (NR(2.0) * NR(2.5e-7));  // 11457
            dcloud = ((((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0)) * cn(mgs));  // 11459
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11460
            thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11461
            qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11462
          }  // 11463
        } else if ((P.irenuc == 6)) {  // 11470
          cn(mgs) = NR(0.0);  // 11474
          if ((ccna(mgs) < (NR(0.7) * cnuc(mgs)))) {  // 11476
            cn(mgs) = c_min((NR(0.9) * cnuc(mgs)), ((P.ccne0 * pow(cnuc(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp)));  // 11477
            cn(mgs) = c_min(cn(mgs), c_max(NR(0.0), (((NR(0.7) * cnuc(mgs)) - ccna(mgs)))));  // 11480
          } else {  // 11483
            temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11486
            ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11488
            ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11489
            c1 = (pqs(mgs) * P.tabqvs(ltemq));  // 11492
            if ((c1 > NR(0.0))) {  // 11493
              ssf(mgs) = c_max(NR(0.0), (NR(100.0) * (((((qv0(mgs) + qwvp(mgs))) / c1) - NR(1.0)))));  // 11494
            } else {  // 11495
              ssf(mgs) = NR(0.0);  // 11496
            }  // 11497
            cn(mgs) = (cnuc(mgs) * c_min(NR(2.0), pow(c_max(NR(0.0), ssf(mgs)), P.cck)));  // 11500
            cn(mgs) = c_min((NR(0.01) * cnuc(mgs)), c_max(NR(0.0), (cn(mgs) - ccna(mgs))));  // 11504
          }  // 11506
          if ((cn(mgs) > NR(0.0))) {  // 11514
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11515
            dcrit = (NR(2.0) * NR(2.5e-7));  // 11519
            dcloud = ((((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0)) * cn(mgs));  // 11521
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11522
            thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11523
            qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11524
          }  // 11526
        } else if ((P.irenuc == 5)) {  // 11527
          cn(mgs) = c_min(cnuc(mgs), ((P.ccne0 * pow(cnuc(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp)));  // 11532
          if ((ccna(mgs) >= cnuc(mgs))) {  // 11535
            temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11536
            ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11537
            ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11538
            c1 = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 11543
            if ((c1 > NR(0.0))) {  // 11545
              ssf(mgs) = c_max(NR(0.0), (NR(100.0) * (((((qv0(mgs) + qwvp(mgs))) / c1) - NR(1.0)))));  // 11546
            } else {  // 11547
              ssf(mgs) = NR(0.0);  // 11548
            }  // 11549
            cn(mgs) = c_max(cn(mgs), (cnuc(mgs) * c_min(P.ssf2kmax, pow(ssf(mgs), P.cck))));  // 11552
            cn(mgs) = c_max(NR(0.0), (cn(mgs) - ccna(mgs)));  // 11557
          } else {  // 11559
            cn(mgs) = c_min(cn(mgs), (cnuc(mgs) - ccna(mgs)));  // 11560
          }  // 11561
          dcrit = (NR(2.0) * NR(2.0e-6));  // 11566
          dcloud = (((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0));  // 11567
          tmp = c_max(NR(0.0), (((rho0(mgs) * qx(mgs, P.lc)) / dcloud) - cx(mgs, P.lc)));  // 11571
          cn(mgs) = c_min(tmp, cn(mgs));  // 11572
          if ((cn(mgs) > NR(0.0))) {  // 11575
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11576
            dcrit = NR(2.5e-7);  // 11578
            dcloud = ((((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0)) * cn(mgs));  // 11580
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11581
            thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11582
            qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11583
          }  // 11584
        } else if (((P.irenuc == 7) || (P.irenuc == 17))) {  // 11588
          cn(mgs) = NR(0.0);  // 11592
          if ((P.irenuc == 7)) {  // 11593
            frac = NR(0.9);  // 11594
          } else {  // 11595
            frac = NR(0.98);  // 11596
          }  // 11597
          if ((ccna(mgs) < (frac * cnuc(mgs)))) {  // 11599
            cn(mgs) = c_min((((frac + NR(0.01))) * cnuc(mgs)), ((P.ccne0 * pow(cnuc(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp)));  // 11600
            cn(mgs) = c_min(cn(mgs), c_max(NR(0.0), (((frac * cnuc(mgs)) - ccna(mgs)))));  // 11603
          } else {  // 11613
            temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11616
            ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11618
            ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11619
            c1 = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 11625
            ssf(mgs) = NR(0.0);  // 11628
            if ((c1 > NR(0.0))) {  // 11629
              ssf(mgs) = (NR(100.0) * (((qx(mgs, P.lv) / c1) - NR(1.0))));  // 11630
            }  // 11631
            if ((ssf(mgs) <= NR(1.0))) {  // 11634
              cn(mgs) = (cnuc(mgs) * c_min(NR(1.0), pow(c_max(NR(0.0), ssf(mgs)), P.cck)));  // 11635
            } else {  // 11636
              cn(mgs) = (cnuc(mgs) * c_min(NR(2.0), pow(c_max(NR(0.0), ((NR(0.03) * ((ssf(mgs) - NR(1.0)))) + NR(1.0))), P.cck)));  // 11637
            }  // 11640
            if ((((ccncuf(mgs) > NR(0.0)) && (ssf(mgs) > P.ssmxuf)) && (((ssmax(mgs) > P.ssmxuf) || (P.lss < 1))))) {  // 11645
              cnuf(mgs) = c_min(ccncuf(mgs), ((P.ccne0 * pow(ccncuf(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp)));  // 11646
            }  // 11648
            cn(mgs) = c_min((NR(0.01) * cnuc(mgs)), c_max(NR(0.0), (cn(mgs) - ccna(mgs))));  // 11654
          }  // 11656
          if (((cn(mgs) + cnuf(mgs)) > NR(0.0))) {  // 11678
            dcrit = (NR(2.0) * NR(2.0e-6));  // 11680
            dcloud = (((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0));  // 11681
            tmp = c_max(NR(0.0), (((rho0(mgs) * qx(mgs, P.lc)) / dcloud) - cx(mgs, P.lc)));  // 11685
            cn(mgs) = c_min(tmp, cn(mgs));  // 11686
            cx(mgs, P.lc) = ((cx(mgs, P.lc) + cn(mgs)) + cnuf(mgs));  // 11688
            dcrit = (NR(2.0) * NR(2.5e-7));  // 11694
            dcloud = ((((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0)) * ((cn(mgs) + cnuf(mgs))));  // 11695
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11696
            thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11697
            qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11698
            ccncuf(mgs) = c_max(NR(0.0), (ccncuf(mgs) - cnuf(mgs)));  // 11700
          }  // 11701
        } else if ((P.irenuc == 8)) {  // 11703
          cn(mgs) = NR(0.0);  // 11707
          if ((ccnc(mgs) > NR(0.0))) {  // 11709
            cn(mgs) = ((P.ccne0 * pow(ccnc(mgs), ((NR(2.0) / ((NR(2.0) + P.cck)))))) * pow(c_max(NR(0.0), wvel(mgs)), P.cnexp));  // 11710
            cn(mgs) = c_min(cn(mgs), ccnc(mgs));  // 11715
          } else if ((cx(mgs, P.lc) < NR(0.01e9))) {  // 11717
            temp1 = (((theta0(mgs) + thetap(mgs))) * pk(mgs));  // 11721
            ltemq = int(((((temp1 - NR(163.15))) / P.fqsat) + NR(1.5)));  // 11723
            ltemq = c_min(P.nqsat, c_max(1, ltemq));  // 11724
            c1 = (((P.rdorv * P.esbolton) * P.tabqvs(ltemq)) / ((pres(mgs) - (P.esbolton * P.tabqvs(ltemq)))));  // 11730
            ssf(mgs) = NR(0.0);  // 11733
            if ((c1 > NR(0.0))) {  // 11734
              ssf(mgs) = (NR(100.0) * (((qx(mgs, P.lv) / c1) - NR(1.0))));  // 11735
            }  // 11736
            if ((ssf(mgs) <= NR(1.0))) {  // 11739
              cn(mgs) = NR(0.0);  // 11740
            } else {  // 11741
              cn(mgs) = ((NR(0.01e9) * c_min(NR(2.0), pow(c_max(NR(0.0), ((NR(0.03) * ((ssf(mgs) - NR(1.0)))) + NR(1.0))), P.cck))) - cx(mgs, P.lc));  // 11743
            }  // 11744
          }  // 11746
          if ((cn(mgs) > NR(0.0))) {  // 11748
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11749
            dcrit = (NR(2.0) * NR(2.5e-7));  // 11755
            dcloud = ((((NR(1000.0) * ipow<3>(dcrit)) * P.pi) / NR(6.0)) * cn(mgs));  // 11757
            qx(mgs, P.lc) = (qx(mgs, P.lc) + dcloud);  // 11758
            thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * dcloud) / (pi0(mgs))));  // 11759
            qwvp(mgs) = (qwvp(mgs) - dcloud);  // 11760
          }  // 11762
        } else if (((P.irenuc == 9) || (P.irenuc == 10))) {  // 11765
          // write(0,*) 'irenuc=9 requires nuwrfmods=1'
        } else if ((P.irenuc == 11)) {  // 11770
          // write(0,*) 'irenuc=11 requires nuwrfmods=1'
        }  // 11773
        ccna(mgs) = (ccna(mgs) + cn(mgs));  // 11776
      }  // 11780
      goto L631;  // 11783
      L613:;  // 11786
      L631:;  // 11788
      ssmx = P.maxsupersat;  // 11793
      qv1 = (qv0(mgs) + qwvp(mgs));  // 11794
      qvs1 = qvs(mgs);  // 11795
      if ((qv1 > ((ssmx * qvs1)))) {  // 11799
        ss1 = (qv1 / qvs1);  // 11803
        ssmx = (NR(100.0) * ((ssmx - NR(1.0))));  // 11805
        qvex = NR(0.0);  // 11807
        qvexcess(P, W, ngs, mgs, qwvp.p, qv0.p, qx.at(1, P.lc), pres.p, thetap.p, theta0.p, qvex, pi0.p, CP(P.tabqvs_p), P.nqsat, P.fqsat, P.cbw, fcqv1.p, felvcp.p, ssmx, pk.p, ngscnt);  // 11809
        if ((qvex > NR(0.0))) {  // 11814
          thetap(mgs) = (thetap(mgs) + ((felvcp(mgs) * qvex) / (pi0(mgs))));  // 11815
          qwvp(mgs) = (qwvp(mgs) - qvex);  // 11819
          qx(mgs, P.lc) = (qx(mgs, P.lc) + qvex);  // 11820
          if ((!flag_qndrop)) {  // 11821
            cn(mgs) = c_min(c_max(ccnc(mgs), cwnccn(mgs)), ((rho0(mgs) * qvex) / c_max(P.cwmasn5, c_max(cwmas20, xmas(mgs, P.lc)))));  // 11830
            if ((P.lccna > 1)) {  // 11833
              ccna(mgs) = (ccna(mgs) + cn(mgs));  // 11835
            } else {  // 11837
              ccnc(mgs) = c_max(NR(0.0), (ccnc(mgs) - cn(mgs)));  // 11838
            }  // 11839
            cx(mgs, P.lc) = (cx(mgs, P.lc) + cn(mgs));  // 11841
          }  // 11843
        }  // 11845
      }  // 11847
      if (((cx(mgs, P.lc) > P.cxmin) && (qx(mgs, P.lc) > P.qxmin(P.lc)))) {  // 11857
        xmas(mgs, P.lc) = ((rho0(mgs) * qx(mgs, P.lc)) / (cx(mgs, P.lc)));  // 11859
        if (((xmas(mgs, P.lc) < P.cwmasn) || (xmas(mgs, P.lc) > P.cwmasx))) {  // 11861
          tmp = cx(mgs, P.lc);  // 11862
          xmas(mgs, P.lc) = c_min(xmas(mgs, P.lc), P.cwmasx);  // 11863
          xmas(mgs, P.lc) = c_max(xmas(mgs, P.lc), P.cwmasn);  // 11864
          cx(mgs, P.lc) = ((rho0(mgs) * qx(mgs, P.lc)) / xmas(mgs, P.lc));  // 11865
        }  // 11868
      }  // 11869
      if (((P.ipconc >= 3) && (P.rcond == 2))) {  // 11894
        if (((cx(mgs, P.lr) > NR(0.0)) && (qx(mgs, P.lr) > P.qxmin(P.lr)))) { xv(mgs, P.lr) = ((rho0(mgs) * qx(mgs, P.lr)) / ((xdn(mgs, P.lr) * cx(mgs, P.lr)))); }  // 11897
        if ((xv(mgs, P.lr) > P.xvmx(P.lr))) { xv(mgs, P.lr) = P.xvmx(P.lr); }  // 11899
        if ((xv(mgs, P.lr) < P.xvmn(P.lr))) { xv(mgs, P.lr) = P.xvmn(P.lr); }  // 11900
      }  // 11902
    }  // 11906
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 11910
    }  // 11915
    for (mgs = 1; mgs <= ngscnt; ++mgs) {  // 11918
      an(igs(mgs), jy, kgs(mgs), P.lt) = (theta0(mgs) + thetap(mgs));  // 11919
      an(igs(mgs), jy, kgs(mgs), P.lv) = (qv0(mgs) + qwvp(mgs));  // 11920
      an(igs(mgs), jy, kgs(mgs), P.lc) = (qx(mgs, P.lc) + c_min(an(igs(mgs), jy, kgs(mgs), P.lc), NR(0.0)));  // 11928
      an(igs(mgs), jy, kgs(mgs), P.lr) = (qx(mgs, P.lr) + c_min(an(igs(mgs), jy, kgs(mgs), P.lr), NR(0.0)));  // 11935
      if ((P.ipconc >= 2)) {  // 11946
        an(igs(mgs), jy, kgs(mgs), P.lnc) = c_max(cx(mgs, P.lc), NR(0.0));  // 11947
        if (((P.lccn > 1) && (P.lccna < 1))) {  // 11954
          an(igs(mgs), jy, kgs(mgs), P.lccn) = c_max(NR(0.0), ccnc(mgs));  // 11955
        }  // 11956
        if ((P.lccna > 1)) {  // 11973
          an(igs(mgs), jy, kgs(mgs), P.lccna) = c_max(NR(0.0), ccna(mgs));  // 11974
        }  // 11975
      }  // 11976
      if (((P.ipconc >= 3) && (P.rcond == 2))) {  // 11977
        an(igs(mgs), jy, kgs(mgs), P.lnr) = c_max(cx(mgs, P.lr), NR(0.0));  // 11978
      }  // 11979
    }  // 11980
    L29998:;  // 11983
    if (((kz > (nz - 1)) && (ix >= nxi))) {  // 11986
      if ((ix >= nxi)) {  // 11987
        goto L2200;  // 11988
      } else {  // 11989
        nzmpb = kz;  // 11990
      }  // 11991
    } else {  // 11992
      nzmpb = kz;  // 11993
    }  // 11994
    if ((ix >= nxi)) {  // 11996
      nxmpb = 1;  // 11997
      nzmpb = (kz + 1);  // 11998
    } else {  // 11999
      nxmpb = (ix + 1);  // 12000
    }  // 12001
    L2000:; }  // 12003
    L2200:;  // 12004
    L9999: return;  // 12018

}
template <class Real>
CPPWRF_HD inline void smallvalues(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int na, int jyslab, int nor, int norz, Real dtp, int nxi, Real* t0_p, Real* an_p, Real* dn_p, Real* w_p, Real* t77_p, bool flag_qndrop) {

  FA<Real> t0 = mkfa<Real>(t0_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> w = mkfa<Real>(w_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t77 = mkfa<Real>(t77_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  auto zerocx = W.template arr<bool>(P.lc, P.lqmx);  // 12058 
  Real frac = 0;  // 12060
  Real hwdn = 0;  // 12060
  Real tmpg = 0;  // 12060
  Real xdia1 = 0;  // 12060
  Real xdia3 = 0;  // 12060
  Real cwch = 0;  // 12060
  Real xvol = 0;  // 12060
  int ix = 0;  // 12062
  int kz = 0;  // 12062
  int i = 0;  // 12062
  int n = 0;  // 12062
  int km1 = 0;  // 12062
  int il = 0;  // 12063
  int jy = 0;  // 12064
  int jgs = 0;  // 12064
  Real chw = 0;  // 12065
  Real g1 = 0;  // 12065
  Real z1 = 0;  // 12065
  Real tmp = 0;  // 12065
  Real tmp2 = 0;  // 12065
  Real fw = 0;  // 12065
  Real tmpmx = 0;  // 12065
  Real qr = 0;  // 12065
  jy = 1;  // 12072
  frac = NR(1.0);  // 12074
  cwch = pow((((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))), ((-(NR(1.0) / NR(3.0)))));  // 12076
  for (kz = 1; kz <= nz; ++kz) {  // 12082
    for (ix = 1; ix <= nxi; ++ix) {  // 12084
      t0(ix, jy, kz) = (an(ix, jy, kz, P.lt) * t77(ix, jy, kz));  // 12086
      for (int i_0 = P.lc; i_0 <= P.lqmx; ++i_0) zerocx(i_0) = false;  // 12088
      for (il = P.lc; il <= P.lhab; ++il) {  // 12089
        if (((P.iresetmoments == 1) || (P.iresetmoments == il))) {  // 12090
          if ((P.ln(il) > 1)) { zerocx(il) = ((an(ix, jy, kz, P.ln(il)) < P.cxmin)); }  // 12091
          if ((P.lz(il) > 1)) { zerocx(il) = ((zerocx(il) || ((an(ix, jy, kz, P.lz(il)) < P.zxmin)))); }  // 12092
        } else {  // 12093
          if ((il == P.lc)) {  // 12094
            if ((P.ln(il) > 1)) {  // 12095
              zerocx(il) = (((an(ix, jy, kz, P.ln(il)) <= NR(0.0))) && (!flag_qndrop));  // 12096
            }  // 12097
          } else {  // 12098
            if ((P.ln(il) > 1)) { zerocx(il) = ((an(ix, jy, kz, P.ln(il)) <= NR(0.0))); }  // 12099
          }  // 12100
        }  // 12101
      }  // 12102
      if ((P.lhl > 1)) {  // 12104
        if ((((an(ix, jy, kz, P.lhl) < (frac * P.qxmin(P.lhl)))) || zerocx(P.lhl))) {  // 12143
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lhl));  // 12146
          an(ix, jy, kz, P.lhl) = NR(0.0);  // 12147
          if ((P.ipconc >= 5)) {  // 12150
            an(ix, jy, kz, P.lnhl) = NR(0.0);  // 12151
          }  // 12152
          if ((P.lvhl > 1)) {  // 12154
            an(ix, jy, kz, P.lvhl) = NR(0.0);  // 12155
          }  // 12156
        } else {  // 12170
          if ((P.lvol(P.lhl) > 1)) {  // 12171
            if ((an(ix, jy, kz, P.lvhl) > NR(0.0))) {  // 12172
              tmp = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 12173
            } else {  // 12174
              tmp = P.rho_qhl;  // 12175
              an(ix, jy, kz, P.lvhl) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / tmp);  // 12176
            }  // 12177
            if ((tmp < P.xdnmn(P.lhl))) {  // 12179
              tmp = c_max(P.xdnmn(P.lhl), tmp);  // 12180
              an(ix, jy, kz, P.lvhl) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / tmp);  // 12181
            }  // 12182
            if (((tmp > P.xdnmx(P.lhl)) && (P.lhlw <= 0))) {  // 12184
              tmp = c_min(P.xdnmx(P.lhl), tmp);  // 12185
              an(ix, jy, kz, P.lvhl) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / tmp);  // 12186
            }  // 12206
          }  // 12215
          if ((P.lvhl > 1)) {  // 12217
            if ((an(ix, jy, kz, P.lvhl) > NR(0.0))) {  // 12218
              hwdn = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 12219
            } else {  // 12220
              hwdn = P.xdn0(P.lhl);  // 12221
            }  // 12222
            hwdn = c_max(P.xdnmn(P.lhl), hwdn);  // 12223
          } else {  // 12224
            hwdn = P.xdn0(P.lhl);  // 12225
          }  // 12226
          if (((P.ipconc >= 5) && (an(ix, jy, kz, P.lhl) > P.qxmin(P.lhl)))) {  // 12228
            qr = an(ix, jy, kz, P.lhl);  // 12229
            xvol = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / ((hwdn * an(ix, jy, kz, P.lnhl))));  // 12230
            chw = an(ix, jy, kz, P.lnhl);  // 12231
            if (((xvol < P.xvmn(P.lhl)) || (xvol > P.xvmx(P.lhl)))) {  // 12233
              xvol = c_min(P.xvmx(P.lhl), c_max(P.xvmn(P.lhl), xvol));  // 12234
              chw = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / ((xvol * hwdn)));  // 12235
              an(ix, jy, kz, P.lnhl) = chw;  // 12236
            }  // 12237
          }  // 12238
        }  // 12258
      }  // 12260
      if ((((an(ix, jy, kz, P.lh) < (frac * P.qxmin(P.lh)))) || zerocx(P.lh))) {  // 12301
        an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lh));  // 12304
        an(ix, jy, kz, P.lh) = NR(0.0);  // 12305
        if ((P.ipconc >= 5)) {  // 12308
          an(ix, jy, kz, P.lnh) = NR(0.0);  // 12309
        }  // 12310
        if ((P.lvh > 1)) {  // 12312
          an(ix, jy, kz, P.lvh) = NR(0.0);  // 12313
        }  // 12314
      } else {  // 12328
        if ((P.lvol(P.lh) > 1)) {  // 12329
          if ((an(ix, jy, kz, P.lvh) > NR(0.0))) {  // 12330
            tmp = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / an(ix, jy, kz, P.lvh));  // 12331
          } else {  // 12332
            tmp = P.rho_qh;  // 12333
            an(ix, jy, kz, P.lvh) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / tmp);  // 12334
          }  // 12335
          if ((tmp < P.xdnmn(P.lh))) {  // 12337
            tmp = c_max(P.xdnmn(P.lh), tmp);  // 12338
            an(ix, jy, kz, P.lvh) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / tmp);  // 12339
          }  // 12340
          if (((tmp > P.xdnmx(P.lh)) && (P.lhw <= 0))) {  // 12342
            tmp = c_min(P.xdnmx(P.lh), tmp);  // 12343
            an(ix, jy, kz, P.lvh) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / tmp);  // 12344
          }  // 12364
        }  // 12373
        if ((P.lvh > 1)) {  // 12375
          if ((an(ix, jy, kz, P.lvh) > NR(0.0))) {  // 12376
            hwdn = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / an(ix, jy, kz, P.lvh));  // 12377
          } else {  // 12378
            hwdn = P.xdn0(P.lh);  // 12379
          }  // 12380
          hwdn = c_max(P.xdnmn(P.lh), hwdn);  // 12381
        } else {  // 12382
          hwdn = P.xdn0(P.lh);  // 12383
        }  // 12384
        if (((P.ipconc >= 5) && (an(ix, jy, kz, P.lh) > P.qxmin(P.lh)))) {  // 12386
          qr = an(ix, jy, kz, P.lh);  // 12387
          xvol = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((hwdn * an(ix, jy, kz, P.lnh))));  // 12388
          chw = an(ix, jy, kz, P.lnh);  // 12389
          if (((xvol < P.xvmn(P.lh)) || (xvol > P.xvmx(P.lh)))) {  // 12391
            xvol = c_min(P.xvmx(P.lh), c_max(P.xvmn(P.lh), xvol));  // 12392
            chw = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((xvol * hwdn)));  // 12393
            an(ix, jy, kz, P.lnh) = chw;  // 12394
          }  // 12395
        }  // 12396
        if ((((((P.ipconc == 5) && (an(ix, jy, kz, P.lh) > P.qxmin(P.lh))) && (P.alphah <= NR(0.1))) && (P.lnh > 1)) && (P.lzh == 0))) {  // 12399
          tmp = (((hwdn * an(ix, jy, kz, P.lnh))) / ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh))));  // 12401
          tmpg = (an(ix, jy, kz, P.lnh) * pow(((tmp * P.pi)), ((NR(1.0) / NR(3.0)))));  // 12402
          if ((tmpg < P.cnohmn)) {  // 12403
            tmp = pow(((((hwdn) / ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)))) * P.pi)), ((NR(1.0) / NR(3.0))));  // 12406
            an(ix, jy, kz, P.lnh) = pow(((P.cnohmn / tmp)), ((NR(3.0) / NR(4.0))));  // 12407
          }  // 12408
        }  // 12410
      }  // 12426
      if ((((an(ix, jy, kz, P.ls) < (frac * P.qxmin(P.ls)))) || zerocx(P.ls))) {  // 12429
        if ((t0(ix, jy, kz) < NR(273.15))) {  // 12430
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.ls));  // 12432
          an(ix, jy, kz, P.ls) = NR(0.0);  // 12433
          if ((P.ipconc >= 4)) {  // 12436
            an(ix, jy, kz, P.lns) = NR(0.0);  // 12438
          }  // 12439
        } else {  // 12449
          an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.ls));  // 12451
          an(ix, jy, kz, P.ls) = NR(0.0);  // 12452
          if ((P.ipconc >= 4)) {  // 12463
            an(ix, jy, kz, P.lns) = NR(0.0);  // 12465
          }  // 12466
        }  // 12468
      } else if ((P.lvol(P.ls) > 1)) {  // 12471
        if ((an(ix, jy, kz, P.lvs) > NR(0.0))) {  // 12472
          tmp = ((dn(ix, jy, kz) * an(ix, jy, kz, P.ls)) / an(ix, jy, kz, P.lvs));  // 12473
          if (((tmp > P.xdnmx(P.ls)) || (tmp < P.xdnmn(P.ls)))) {  // 12474
            tmp = c_min(P.xdnmx(P.ls), c_max(P.xdnmn(P.ls), tmp));  // 12475
            an(ix, jy, kz, P.lvs) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.ls)) / tmp);  // 12476
          }  // 12477
        } else {  // 12478
          tmp = P.rho_qs;  // 12479
          an(ix, jy, kz, P.lvs) = ((dn(ix, jy, kz) * an(ix, jy, kz, P.ls)) / tmp);  // 12480
        }  // 12481
      }  // 12484
      if ((((an(ix, jy, kz, P.lr) < (frac * P.qxmin(P.lr)))) || zerocx(P.lr))) {  // 12490
        an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lr));  // 12491
        an(ix, jy, kz, P.lr) = NR(0.0);  // 12492
        if ((P.ipconc >= 3)) {  // 12493
          an(ix, jy, kz, P.lnr) = NR(0.0);  // 12495
        }  // 12496
      }  // 12502
      if ((((an(ix, jy, kz, P.li) <= (frac * P.qxmin(P.li)))) || zerocx(P.li))) {  // 12507
        an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.li));  // 12508
        an(ix, jy, kz, P.li) = NR(0.0);  // 12509
        if ((P.ipconc >= 1)) {  // 12510
          an(ix, jy, kz, P.lni) = NR(0.0);  // 12511
        }  // 12512
      }  // 12513
      if ((((an(ix, jy, kz, P.lc) <= (frac * P.qxmin(P.lc)))) || zerocx(P.lc))) {  // 12519
        an(ix, jy, kz, P.lv) = (an(ix, jy, kz, P.lv) + an(ix, jy, kz, P.lc));  // 12520
        an(ix, jy, kz, P.lc) = NR(0.0);  // 12521
        if ((P.ipconc >= 2)) {  // 12522
          if (((P.lccn > 1) || (P.ac_opt == 1))) {  // 12523
            if (((P.irenuc < 5) && (P.lccna <= 1))) {  // 12524
              an(ix, jy, kz, P.lccn) = (an(ix, jy, kz, P.lccn) + c_max(NR(0.0), an(ix, jy, kz, P.lnc)));  // 12526
            } else if ((P.lccna > 1)) {  // 12530
              tmp = c_max(NR(0.0), an(ix, jy, kz, P.lnc));  // 12531
              an(ix, jy, kz, P.lccna) = c_max(NR(0.0), (an(ix, jy, kz, P.lccna) - tmp));  // 12541
            }  // 12543
          }  // 12544
          an(ix, jy, kz, P.lnc) = NR(0.0);  // 12545
          if ((P.lccn > 1)) { an(ix, jy, kz, P.lccn) = c_max(NR(0.0), an(ix, jy, kz, P.lccn)); }  // 12546
          if ((P.lccna > 0)) {  // 12549
            tmp = (an(ix, jy, kz, P.li) + an(ix, jy, kz, P.ls));  // 12551
            if ((tmp < P.qxmin(P.li))) {  // 12552
              if (((an(ix, jy, kz, P.lccna) > NR(1.0)) && (tmp < P.qxmin(P.li)))) { an(ix, jy, kz, P.lccna) = (an(ix, jy, kz, P.lccna) * exp((-(dtp / P.ccntimeconst)))); }  // 12553
            }  // 12556
          } else if ((((P.lccn > 1) && P.restoreccn) && (P.ac_opt == 0))) {  // 12558
            tmp = (an(ix, jy, kz, P.li) + an(ix, jy, kz, P.ls));  // 12560
            if ((((an(ix, jy, kz, P.lccn) > NR(1.0)) && (tmp < P.qxmin(P.li))) && (((an(ix, jy, kz, P.lccn) < (dn(ix, jy, kz) * P.qccn)) || (!P.invertccn))))) {  // 12565
              an(ix, jy, kz, P.lccn) = ((dn(ix, jy, kz) * P.qccn) - (c_max(NR(0.0), ((dn(ix, jy, kz) * P.qccn) - an(ix, jy, kz, P.lccn))) * exp((-(dtp / P.ccntimeconst)))));  // 12570
            }  // 12572
          }  // 12574
        }  // 12576
      }  // 12578
    }  // 12580
  }  // 12582

}
template <class Real>
CPPWRF_HD inline void radardd02(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int nor, int na, Real* an_p, Real* temk_p, Real* dbz_p, Real* db_p, int nzdbz, Real cnoh0t, Real hwdn1t, int ipconc, int ke_diag, int iunit) {
  const int printyn = 0;  // 8755
  const int norz = 3;  // 8768
  const Real cwmasn = NR(5.23e-13);  // 8835
  const Real cwmasx = NR(5.25e-10);  // 8836
  const Real cwradn = NR(5.0e-6);  // 8837
  const int ng1 = 1;
  const Real dbzmin = 0;
  const Real rhos = NR(0.1);
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor), 1, na);
  FA<Real> temk = mkfa<Real>(temk_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  FA<Real> dbz = mkfa<Real>(dbz_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  FA<Real> db = mkfa<Real>(db_p, ((-nor) + ng1), (nx + nor), ((-nor) + ng1), (ny + nor), ((-nor) + ng1), (nz + nor));
  int ngt = 0;  // 8750
  int n10 = 0;  // 8753
  Real vr = 0;  // 8762
  int imapz = 0;  // 8765
  int mzdist = 0;  // 8765
  int vzflag = 0;  // 8767
  auto gz = W.template arr<Real>(((-nor) + 1), (nz + nor));  // 8774 
  Real cr1 = 0;  // 8777
  Real cr2 = 0;  // 8777
  Real hwdnsq = 0;  // 8777
  Real swdnsq = 0;  // 8777
  Real rwdnsq = 0;  // 8778
  Real dhmin = 0;  // 8778
  Real qrmin = 0;  // 8778
  Real qsmin = 0;  // 8778
  Real qhmin = 0;  // 8778
  Real qhlmin = 0;  // 8778
  Real tfr = 0;  // 8778
  Real tfrh = 0;  // 8778
  Real zrc = 0;  // 8778
  Real reflectmin = 0;  // 8779
  Real kw_sq = 0;  // 8779
  Real const_ki_sn = 0;  // 8780
  Real const_ki_h = 0;  // 8780
  Real ki_sq_sn = 0;  // 8780
  Real ki_sq_h = 0;  // 8781
  Real dielf_sn = 0;  // 8781
  Real dielf_h = 0;  // 8781
  Real pi = 0;  // 8782
  bool ltest = false;  // 8783
  auto gtmp = W.template arr<Real>(1, nx, 1, nz);  // 8786 
  auto dtmp = W.template arr<Real>(1, nx, 1, nz);  // 8787 
  Real tmp = 0;  // 8788
  double dtmps = 0;  // 8790
  double dtmpr = 0;  // 8790
  double dtmph = 0;  // 8790
  double dtmphl = 0;  // 8790
  double g1 = 0;  // 8790
  double zx = 0;  // 8790
  double ze = 0;  // 8790
  double x = 0;  // 8790
  int i = 0;  // 8792
  int j = 0;  // 8792
  int k = 0;  // 8792
  int ix = 0;  // 8792
  int jy = 0;  // 8792
  int kz = 0;  // 8792
  int ihcnt = 0;  // 8792
  double xcnoh = 0;  // 8794
  double xcnos = 0;  // 8794
  double dadh = 0;  // 8794
  double dads = 0;  // 8794
  double zhdryc = 0;  // 8794
  double zsdryc = 0;  // 8794
  double zhwetc = 0;  // 8794
  double zswetc = 0;  // 8794
  double dadr = 0;  // 8795
  Real dbzmax = 0;  // 8796
  Real cnow = 0;  // 8799
  Real cnoi = 0;  // 8799
  Real cnoip = 0;  // 8799
  Real cnoir = 0;  // 8799
  Real cnor = 0;  // 8799
  Real cnos = 0;  // 8799
  Real cnogl = 0;  // 8800
  Real cnogm = 0;  // 8800
  Real cnogh = 0;  // 8800
  Real cnof = 0;  // 8800
  Real cnoh = 0;  // 8800
  Real cnohl = 0;  // 8800
  Real swdn = 0;  // 8802
  Real rwdn = 0;  // 8802
  Real hwdn = 0;  // 8802
  Real gldn = 0;  // 8802
  Real gmdn = 0;  // 8802
  Real ghdn = 0;  // 8802
  Real fwdn = 0;  // 8802
  Real hldn = 0;  // 8802
  Real swdn0 = 0;  // 8803
  Real rwdnmx = 0;  // 8805
  Real cwdnmx = 0;  // 8805
  Real cidnmx = 0;  // 8805
  Real xidnmx = 0;  // 8805
  Real swdnmx = 0;  // 8805
  Real gldnmx = 0;  // 8805
  Real gmdnmx = 0;  // 8805
  Real ghdnmx = 0;  // 8806
  Real fwdnmx = 0;  // 8806
  Real hwdnmx = 0;  // 8806
  Real hldnmx = 0;  // 8806
  Real rwdnmn = 0;  // 8807
  Real cwdnmn = 0;  // 8807
  Real cidnmn = 0;  // 8807
  Real xidnmn = 0;  // 8807
  Real swdnmn = 0;  // 8807
  Real gldnmn = 0;  // 8807
  Real gmdnmn = 0;  // 8807
  Real ghdnmn = 0;  // 8808
  Real fwdnmn = 0;  // 8808
  Real hwdnmn = 0;  // 8808
  Real hldnmn = 0;  // 8808
  Real gldnsq = 0;  // 8810
  Real gmdnsq = 0;  // 8810
  Real ghdnsq = 0;  // 8810
  Real fwdnsq = 0;  // 8810
  Real hldnsq = 0;  // 8810
  Real dadgl = 0;  // 8812
  Real dadgm = 0;  // 8812
  Real dadgh = 0;  // 8812
  Real dadhl = 0;  // 8812
  Real dadf = 0;  // 8812
  Real zgldryc = 0;  // 8813
  Real zglwetc = 0;  // 8813
  Real zgmdryc = 0;  // 8813
  Real zgmwetc = 0;  // 8813
  Real zghdryc = 0;  // 8813
  Real zghwetc = 0;  // 8813
  Real zhldryc = 0;  // 8814
  Real zhlwetc = 0;  // 8814
  Real zfdryc = 0;  // 8814
  Real zfwetc = 0;  // 8814
  Real dielf_gl = 0;  // 8816
  Real dielf_gm = 0;  // 8816
  Real dielf_gh = 0;  // 8816
  Real dielf_hl = 0;  // 8816
  Real dielf_fw = 0;  // 8816
  int imx = 0;  // 8818
  int jmx = 0;  // 8818
  int kmx = 0;  // 8818
  Real swdia = 0;  // 8820
  Real gldia = 0;  // 8820
  Real gmdia = 0;  // 8820
  Real ghdia = 0;  // 8820
  Real fwdia = 0;  // 8820
  Real hwdia = 0;  // 8820
  Real hldia = 0;  // 8820
  Real csw = 0;  // 8822
  Real cgl = 0;  // 8822
  Real cgm = 0;  // 8822
  Real cgh = 0;  // 8822
  Real cfw = 0;  // 8822
  Real chw = 0;  // 8822
  Real chl = 0;  // 8822
  Real xvs = 0;  // 8823
  Real xvgl = 0;  // 8823
  Real xvgm = 0;  // 8823
  Real xvgh = 0;  // 8823
  Real xvf = 0;  // 8823
  Real xvh = 0;  // 8823
  Real xvhl = 0;  // 8823
  Real cwc0 = 0;  // 8825
  int izieg = 0;  // 8826
  int ice10 = 0;  // 8827
  Real qxw = 0;  // 8831
  Real qxw1 = 0;  // 8831
  Real dnsnow = 0;  // 8832
  Real qh = 0;  // 8833
  auto cwnccn = W.template arr<Real>(1, nz);  // 8839 
  Real vzsnow = 0;  // 8841
  Real vzrain = 0;  // 8841
  Real vzgraupel = 0;  // 8841
  Real vzhail = 0;  // 8841
  Real ksq = 0;  // 8842
  Real dtp = 0;  // 8843
  vzflag = 0;  // 8848
  izieg = 0;  // 8850
  ice10 = 0;  // 8851
  pi = (NR(4.0) * atan(NR(1.0)));  // 8860
  cwc0 = P.piinv;  // 8861
  cnoh = cnoh0t;  // 8863
  hwdn = hwdn1t;  // 8864
  rwdn = NR(1000.0);  // 8866
  swdn = NR(100.0);  // 8867
  qrmin = NR(1.0e-05);  // 8869
  qsmin = NR(1.0e-06);  // 8870
  qhmin = NR(1.0e-05);  // 8871
  cnow = NR(1.0e+08);  // 8876
  cnoi = NR(1.0e+08);  // 8877
  cnoip = NR(1.0e+08);  // 8878
  cnoir = NR(1.0e+08);  // 8879
  cnor = NR(8.0e+06);  // 8880
  cnos = NR(8.0e+06);  // 8881
  cnogl = NR(4.0e+05);  // 8882
  cnogm = NR(4.0e+05);  // 8883
  cnogh = NR(4.0e+05);  // 8884
  cnof = NR(4.0e+05);  // 8885
  cnohl = NR(1.0e+03);  // 8886
  imx = 1;  // 8889
  jmx = 1;  // 8890
  kmx = 1;  // 8891
  i = 1;  // 8892
  if (false) {  // microp = 'ZVD'  // 8895
    izieg = 1;  // 8898
    hwdn = hwdn1t;  // 8900
    cnor = P.cno(P.lr);  // 8903
    cnos = P.cno(P.ls);  // 8904
    cnoh = P.cno(P.lh);  // 8905
    qrmin = P.qxmin(P.lr);  // 8906
    qsmin = P.qxmin(P.ls);  // 8907
    qhmin = P.qxmin(P.lh);  // 8908
    if ((P.lhl > 1)) {  // 8909
      cnohl = P.cno(P.lhl);  // 8910
      qhlmin = P.qxmin(P.lhl);  // 8911
    }  // 8912
  } else if (true) {  // 8914
    izieg = 1;  // 8916
    swdn0 = swdn;  // 8918
    cnor = P.cno(P.lr);  // 8920
    cnos = P.cno(P.ls);  // 8921
    cnoh = P.cno(P.lh);  // 8922
    qrmin = P.qxmin(P.lr);  // 8924
    qsmin = P.qxmin(P.ls);  // 8925
    qhmin = P.qxmin(P.lh);  // 8926
    if ((P.lhl > 1)) {  // 8927
      cnohl = P.cno(P.lhl);  // 8928
      qhlmin = P.qxmin(P.lhl);  // 8929
    }  // 8930
  }  // 8934
  rwdnmx = NR(1000.0);  // 9014
  cwdnmx = NR(1000.0);  // 9015
  cidnmx = NR(917.0);  // 9016
  xidnmx = NR(917.0);  // 9017
  swdnmx = NR(200.0);  // 9018
  gldnmx = NR(400.0);  // 9019
  gmdnmx = NR(600.0);  // 9020
  ghdnmx = NR(800.0);  // 9021
  fwdnmx = NR(900.0);  // 9022
  hwdnmx = NR(900.0);  // 9023
  hldnmx = NR(900.0);  // 9024
  rwdnmn = NR(1000.0);  // 9026
  cwdnmn = NR(1000.0);  // 9027
  xidnmn = NR(001.0);  // 9028
  cidnmn = NR(001.0);  // 9029
  swdnmn = NR(001.0);  // 9030
  gldnmn = NR(200.0);  // 9031
  gmdnmn = NR(400.0);  // 9032
  ghdnmn = NR(600.0);  // 9033
  fwdnmn = NR(700.0);  // 9034
  hwdnmn = NR(700.0);  // 9035
  hldnmn = NR(900.0);  // 9036
  gldn = ((NR(0.5)) * ((gldnmn + gldnmx)));  // 9039
  gmdn = ((NR(0.5)) * ((gmdnmn + gmdnmx)));  // 9040
  ghdn = ((NR(0.5)) * ((ghdnmn + ghdnmx)));  // 9041
  fwdn = ((NR(0.5)) * ((fwdnmn + fwdnmx)));  // 9042
  hldn = ((NR(0.5)) * ((hldnmn + hldnmx)));  // 9043
  cr1 = NR(7.2e+20);  // 9046
  cr2 = NR(7.295e+19);  // 9047
  hwdnsq = ipow<2>(hwdn);  // 9048
  swdnsq = ipow<2>(swdn);  // 9049
  rwdnsq = ipow<2>(rwdn);  // 9050
  gldnsq = ipow<2>(gldn);  // 9052
  gmdnsq = ipow<2>(gmdn);  // 9053
  ghdnsq = ipow<2>(ghdn);  // 9054
  fwdnsq = ipow<2>(fwdn);  // 9055
  hldnsq = ipow<2>(hldn);  // 9056
  dhmin = NR(0.005);  // 9058
  tfr = NR(273.16);  // 9059
  tfrh = (tfr - NR(8.0));  // 9060
  zrc = (cr1 * cnor);  // 9061
  reflectmin = NR(0.0);  // 9062
  kw_sq = NR(0.93);  // 9063
  dbzmax = dbzmin;  // 9064
  ihcnt = 0;  // 9066
  const_ki_sn = (NR(0.5) - ((((NR(0.5) - NR(0.46))) / ((NR(917.0) - NR(220.0)))) * ((swdn - NR(220.0)))));  // 9076
  const_ki_h = (NR(0.5) - ((((NR(0.5) - NR(0.46))) / ((NR(917.0) - NR(220.0)))) * ((hwdn - NR(220.0)))));  // 9077
  ki_sq_sn = (((swdnsq / rwdnsq)) * ipow<2>(const_ki_sn));  // 9078
  ki_sq_h = (((hwdnsq / rwdnsq)) * ipow<2>(const_ki_h));  // 9079
  dielf_sn = (ki_sq_sn / kw_sq);  // 9080
  dielf_h = (ki_sq_h / kw_sq);  // 9081
  dielf_sn = ((((swdnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9088
  dielf_h = ((((hwdnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9089
  dielf_gl = ((((gldnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9091
  dielf_gm = ((((gmdnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9092
  dielf_gh = ((((ghdnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9093
  dielf_hl = ((((hldnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9094
  dielf_fw = ((((fwdnsq / rwdnsq)) * NR(0.21)) / kw_sq);  // 9095
  for (jy = 1; jy <= 1; ++jy) {  // 9121
    for (kz = 1; kz <= ke_diag; ++kz) {  // 9123
      for (ix = 1; ix <= nx; ++ix) {  // 9125
        dbz(ix, jy, kz) = NR(0.0);  // 9126
        vzsnow = NR(0.0);  // 9128
        vzrain = NR(0.0);  // 9129
        vzgraupel = NR(0.0);  // 9130
        vzhail = NR(0.0);  // 9131
        dtmph = NR(0.0);  // 9133
        dtmps = NR(0.0);  // 9134
        dtmphl = NR(0.0);  // 9135
        dtmpr = NR(0.0);  // 9136
        dadr = pow(((db(ix, jy, kz) / (((pi * rwdn) * cnor)))), (NR(0.25)));  // 9137
        dtmp(ix, kz) = NR(0.0);  // 9142
        gtmp(ix, kz) = NR(0.0);  // 9143
        if ((an(ix, jy, kz, P.lr) >= qrmin)) {  // 9144
          if ((ipconc <= 2)) {  // 9145
            gtmp(ix, kz) = (dadr * pow(an(ix, jy, kz, P.lr), (NR(0.25))));  // 9146
            dtmp(ix, kz) = (zrc * ipow<7>(gtmp(ix, kz)));  // 9147
          } else if ((an(ix, jy, kz, P.lnr) > NR(1.0e-3))) {  // 9150
            g1 = (((((NR(6.0) + P.alphar)) * ((NR(5.0) + P.alphar))) * ((NR(4.0) + P.alphar))) / (((((NR(3.0) + P.alphar)) * ((NR(2.0) + P.alphar))) * ((NR(1.0) + P.alphar)))));  // 9155
            zx = ((g1 * ipow<2>(((db(ix, jy, kz) * an(ix, jy, kz, P.lr))))) / an(ix, jy, kz, P.lnr));  // 9156
            ze = ((NR(1.0e18) * zx) * ipow<2>(((NR(6.0) / ((pi * NR(1000.0)))))));  // 9157
            dtmp(ix, kz) = ze;  // 9158
          }  // 9160
          dtmpr = dtmp(ix, kz);  // 9161
        }  // 9162
        if ((P.lhab > P.lr)) {  // 9170
          xcnoh = cnoh;  // 9197
          xcnos = cnos;  // 9198
          if ((ipconc >= 5)) {  // 9228
            xvs = ((db(ix, jy, kz) * an(ix, jy, kz, P.ls)) / ((swdn * c_max(NR(1.0e-3), an(ix, jy, kz, P.lns)))));  // 9230
            if (((xvs < P.xvsmn) || (xvs > P.xvsmx))) {  // 9232
              xvs = c_min(P.xvsmx, c_max(P.xvsmn, xvs));  // 9233
              csw = ((db(ix, jy, kz) * an(ix, jy, kz, P.ls)) / ((xvs * swdn)));  // 9234
            }  // 9235
            swdia = pow(((xvs * cwc0)), ((NR(1.0) / NR(3.0))));  // 9237
            xcnos = ((an(ix, jy, kz, P.ls) * db(ix, jy, kz)) / (((xvs * swdn) * swdia)));  // 9238
          }  // 9240
          if ((P.lvh > 1)) {  // 9257
            if ((an(ix, jy, kz, P.lvh) > NR(0.0))) {  // 9258
              hwdn = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / an(ix, jy, kz, P.lvh));  // 9259
              hwdn = c_min(NR(900.0), c_max(P.hdnmn, hwdn));  // 9260
            } else {  // 9261
              hwdn = NR(500.0);  // 9262
            }  // 9263
          } else {  // 9264
            hwdn = hwdn1t;  // 9265
          }  // 9266
          if ((ipconc >= 5)) {  // 9268
            xvh = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((hwdn * c_max(NR(1.0e-3), an(ix, jy, kz, P.lnh)))));  // 9270
            if (((xvh < P.xvhmn) || (xvh > P.xvhmx))) {  // 9272
              xvh = c_min(P.xvhmx, c_max(P.xvhmn, xvh));  // 9273
              chw = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((xvh * hwdn)));  // 9274
            }  // 9275
            hwdia = pow(((xvh * cwc0)), ((NR(1.0) / NR(3.0))));  // 9277
            xcnoh = ((an(ix, jy, kz, P.lh) * db(ix, jy, kz)) / (((xvh * hwdn) * hwdia)));  // 9278
          }  // 9280
          dadh = NR(0.0);  // 9284
          dadhl = NR(0.0);  // 9285
          dads = NR(0.0);  // 9286
          if ((xcnoh > NR(0.0))) {  // 9287
            dadh = pow(((db(ix, jy, kz) / (((pi * hwdn) * xcnoh)))), (NR(0.25)));  // 9288
            zhdryc = (((NR(0.224) * cr2) * ipow<2>(((db(ix, jy, kz) / rwdn)))) / xcnoh);  // 9289
          } else {  // 9293
            dadh = NR(0.0);  // 9294
            zhdryc = NR(0.0);  // 9295
          }  // 9296
          if ((xcnos > NR(0.0))) {  // 9298
            dads = pow(((db(ix, jy, kz) / (((pi * swdn) * xcnos)))), (NR(0.25)));  // 9299
            zsdryc = (((NR(0.224) * cr2) * ipow<2>(((db(ix, jy, kz) / rwdn)))) / xcnos);  // 9300
          } else {  // 9301
            dads = NR(0.0);  // 9302
            zsdryc = NR(0.0);  // 9303
          }  // 9304
          zhwetc = zhdryc;  // 9305
          zswetc = zsdryc;  // 9306
          gtmp(ix, kz) = NR(0.0);  // 9312
          qxw = NR(0.0);  // 9313
          qxw1 = NR(0.0);  // 9314
          dtmps = NR(0.0);  // 9315
          if ((an(ix, jy, kz, P.ls) >= qsmin)) {  // 9316
            if ((ipconc >= 4)) {  // 9317
              vr = xvs;  // 9328
              ksq = NR(0.189);  // 9331
              if ((an(ix, jy, kz, P.lns) > NR(1.0e-7))) {  // 9332
                if (((qxw > qsmin) || (P.iusewetsnow >= 2))) {  // 9334
                  gtmp(ix, kz) = (((((NR(3.6e18) * ((P.snu + NR(2.0)))) * (((NR(0.224) * ((an(ix, jy, kz, P.ls) + qxw1))) + (NR(0.776) * qxw)))) * ((an(ix, jy, kz, P.ls) + qxw1))) / (((an(ix, jy, kz, P.lns) * ((P.snu + NR(1.0)))) * ipow<2>(rwdn)))) * ipow<2>(db(ix, jy, kz)));  // 9337
                } else {  // 9340
                  dnsnow = (NR(0.0346159) * sqrt((an(ix, jy, kz, P.lns) / ((an(ix, jy, kz, P.ls) * db(ix, jy, kz))))));  // 9342
                  gtmp(ix, kz) = (((((((NR(1.0e18) * NR(323.3226)) * ipow<2>(NR(0.106214))) * (((ksq * an(ix, jy, kz, P.ls)) + (((NR(1.0) - ksq)) * qxw)))) * an(ix, jy, kz, P.ls)) * ipow<2>(db(ix, jy, kz))) * P.gsnow73) / ((((an(ix, jy, kz, P.lns) * ipow<2>((NR(917.0)))) * P.gsnow1) * pow(((NR(1.0) + P.snu)), ((NR(4.0) / NR(3.0)))))));  // 9344
                }  // 9352
              }  // 9354
              dtmps = gtmp(ix, kz);  // 9358
              dtmp(ix, kz) = (dtmp(ix, kz) + gtmp(ix, kz));  // 9359
            } else {  // 9360
              gtmp(ix, kz) = (dads * pow(an(ix, jy, kz, P.ls), (NR(0.25))));  // 9361
              if ((gtmp(ix, kz) > NR(0.0))) {  // 9363
                dtmps = ((zsdryc * ipow<2>(an(ix, jy, kz, P.ls))) / gtmp(ix, kz));  // 9364
                if ((temk(ix, jy, kz) < tfr)) {  // 9365
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zsdryc * ipow<2>(an(ix, jy, kz, P.ls))) / gtmp(ix, kz)));  // 9366
                } else {  // 9368
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zswetc * ipow<2>(an(ix, jy, kz, P.ls))) / gtmp(ix, kz)));  // 9369
                }  // 9371
              }  // 9372
            }  // 9373
          }  // 9375
          if (((P.idbzci == 1) && (P.lni > 0))) {  // 9385
            if (((an(ix, jy, kz, P.li) > P.qxmin(P.li)) && (an(ix, jy, kz, P.lni) > NR(1.0)))) {  // 9387
              vr = ((db(ix, jy, kz) * an(ix, jy, kz, P.li)) / ((NR(900.0) * an(ix, jy, kz, P.lni))));  // 9388
              dtmp(ix, kz) = (dtmp(ix, kz) + ((((((NR(0.224) * NR(3.6e18)) * ((P.cinu + NR(2.0)))) * an(ix, jy, kz, P.lni)) * ipow<2>(vr)) / ((P.cinu + NR(1.0)))) * ipow<2>(((NR(900.0) / NR(1000.0))))));  // 9389
            }  // 9391
          }  // 9403
          gtmp(ix, kz) = NR(0.0);  // 9411
          dtmph = NR(0.0);  // 9412
          qxw = NR(0.0);  // 9413
          if (((izieg >= 1) && (ipconc >= 5))) {  // 9415
            ltest = false;  // 9417
            if ((ltest || (((an(ix, jy, kz, P.lh) >= qhmin) && (an(ix, jy, kz, P.lnh) >= P.cxmin))))) {  // 9423
              if ((P.lvh > 1)) {  // 9425
                if ((an(ix, jy, kz, P.lvh) > NR(0.0))) {  // 9427
                  hwdn = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / an(ix, jy, kz, P.lvh));  // 9428
                  hwdn = c_min(NR(900.0), c_max(NR(100.0), hwdn));  // 9429
                } else {  // 9430
                  hwdn = NR(500.0);  // 9431
                }  // 9432
              }  // 9434
              chw = an(ix, jy, kz, P.lnh);  // 9436
              if ((chw > NR(0.0))) {  // 9437
                xvh = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((hwdn * c_max(NR(1.0e-3), chw))));  // 9438
                if (((xvh < P.xvhmn) || (xvh > P.xvhmx))) {  // 9439
                  xvh = c_min(P.xvhmx, c_max(P.xvhmn, xvh));  // 9440
                  chw = ((db(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((xvh * hwdn)));  // 9441
                }  // 9442
                qh = an(ix, jy, kz, P.lh);  // 9444
                g1 = (((((NR(6.0) + P.alphah)) * ((NR(5.0) + P.alphah))) * ((NR(4.0) + P.alphah))) / (((((NR(3.0) + P.alphah)) * ((NR(2.0) + P.alphah))) * ((NR(1.0) + P.alphah)))));  // 9471
                zx = ((((g1 * ipow<2>(db(ix, jy, kz))) * (((NR(0.224) * qh) + (NR(0.776) * qxw)))) * qh) / chw);  // 9474
                ze = ((NR(1.0e18) * zx) * ipow<2>(((NR(6.0) / ((pi * NR(1000.0)))))));  // 9475
                dtmp(ix, kz) = (dtmp(ix, kz) + ze);  // 9476
                dtmph = ze;  // 9477
              }  // 9480
            }  // 9483
          } else {  // 9485
            dtmph = NR(0.0);  // 9487
            if ((an(ix, jy, kz, P.lh) >= qhmin)) {  // 9489
              gtmp(ix, kz) = (dadh * pow(an(ix, jy, kz, P.lh), (NR(0.25))));  // 9490
              if ((gtmp(ix, kz) > NR(0.0))) {  // 9491
                dtmph = ((zhdryc * ipow<2>(an(ix, jy, kz, P.lh))) / gtmp(ix, kz));  // 9492
                if ((temk(ix, jy, kz) < tfr)) {  // 9493
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zhdryc * ipow<2>(an(ix, jy, kz, P.lh))) / gtmp(ix, kz)));  // 9494
                } else {  // 9496
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zhdryc * ipow<2>(an(ix, jy, kz, P.lh))) / gtmp(ix, kz)));  // 9498
                }  // 9505
              }  // 9506
            }  // 9507
          }  // 9511
        }  // 9516
        if (((izieg >= 1) && (P.lhl > 1))) {  // 9519
          hldn = NR(900.0);  // 9521
          gtmp(ix, kz) = NR(0.0);  // 9522
          dtmphl = NR(0.0);  // 9523
          qxw = NR(0.0);  // 9524
          if ((P.lvhl > 1)) {  // 9527
            if ((an(ix, jy, kz, P.lvhl) > NR(0.0))) {  // 9528
              hldn = ((db(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 9529
              hldn = c_min(NR(900.0), c_max(NR(300.0), hldn));  // 9530
            } else {  // 9531
              hldn = NR(900.0);  // 9532
            }  // 9533
          } else {  // 9534
            hldn = P.rho_qhl;  // 9535
          }  // 9536
          if ((ipconc >= 5)) {  // 9539
            ltest = false;  // 9541
            if ((ltest || (((an(ix, jy, kz, P.lhl) >= qhlmin) && (an(ix, jy, kz, P.lnhl) > NR(0.0)))))) {  // 9547
              chl = an(ix, jy, kz, P.lnhl);  // 9548
              if ((chl > NR(0.0))) {  // 9549
                xvhl = ((db(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / ((hldn * c_max(NR(1.0e-9), an(ix, jy, kz, P.lnhl)))));  // 9550
                if (((xvhl < P.xvhlmn) || (xvhl > P.xvhlmx))) {  // 9552
                  xvhl = c_min(P.xvhlmx, c_max(P.xvhlmn, xvhl));  // 9553
                  chl = ((db(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / ((xvhl * hldn)));  // 9554
                }  // 9556
                g1 = (((((NR(6.0) + P.alphahl)) * ((NR(5.0) + P.alphahl))) * ((NR(4.0) + P.alphahl))) / (((((NR(3.0) + P.alphahl)) * ((NR(2.0) + P.alphahl))) * ((NR(1.0) + P.alphahl)))));  // 9574
                zx = ((((g1 * ipow<2>(db(ix, jy, kz))) * (((NR(0.224) * an(ix, jy, kz, P.lhl)) + (NR(0.776) * qxw)))) * an(ix, jy, kz, P.lhl)) / chl);  // 9575
                ze = ((NR(1.0e18) * zx) * ipow<2>(((NR(6.0) / ((pi * NR(1000.0)))))));  // 9577
                dtmp(ix, kz) = (dtmp(ix, kz) + ze);  // 9578
                dtmphl = ze;  // 9579
              }  // 9582
            }  // 9584
          } else {  // 9587
            if ((an(ix, jy, kz, P.lhl) >= qhlmin)) {  // 9590
              dadhl = pow(((db(ix, jy, kz) / (((pi * hldn) * cnohl)))), (NR(0.25)));  // 9591
              gtmp(ix, kz) = (dadhl * pow(an(ix, jy, kz, P.lhl), (NR(0.25))));  // 9592
              if ((gtmp(ix, kz) > NR(0.0))) {  // 9593
                zhldryc = (((NR(0.224) * cr2) * ipow<2>(((db(ix, jy, kz) / rwdn)))) / cnohl);  // 9595
                dtmphl = ((zhldryc * ipow<2>(an(ix, jy, kz, P.lhl))) / gtmp(ix, kz));  // 9597
                if ((temk(ix, jy, kz) < tfr)) {  // 9599
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zhldryc * ipow<2>(an(ix, jy, kz, P.lhl))) / gtmp(ix, kz)));  // 9600
                } else {  // 9602
                  dtmp(ix, kz) = (dtmp(ix, kz) + ((zhldryc * ipow<2>(an(ix, jy, kz, P.lhl))) / gtmp(ix, kz)));  // 9604
                }  // 9611
              }  // 9612
            }  // 9614
          }  // 9616
        }  // 9619
        if ((dtmp(ix, kz) > NR(0.0))) {  // 9623
          dbz(ix, jy, kz) = c_max(dbzmin, (NR(10.0) * log10(dtmp(ix, kz))));  // 9624
          if ((dbz(ix, jy, kz) > dbzmax)) {  // 9626
            dbzmax = c_max(dbzmax, dbz(ix, jy, kz));  // 9627
            imx = ix;  // 9628
            jmx = jy;  // 9629
            kmx = kz;  // 9630
          }  // 9631
        } else {  // 9632
          dbz(ix, jy, kz) = dbzmin;  // 9633
          if (((P.lh > 1) && (P.lhl > 1))) {  // 9634
            if ((an(ix, jy, kz, P.lh) > NR(1.0e-3))) {  // 9635
              // write(0,*) 'radardbz: qr,qh,qhl = ',an(ix,jy,kz,lr), an(ix,jy,kz,lh),an(ix,jy,kz,lhl)
              // write(0,*) 'radardbz: dtmps,dtmph,dadh,dadhl,dtmphl = ',dtmps,dtmph,dadh,dadhl,dtmphl
            }  // 9640
          }  // 9641
        }  // 9642
        if (((!(dtmp(ix, kz) < NR(1.0e30))) || (dbz(ix, jy, kz) > NR(190.0)))) {  // 9652
          // write(0,*) 'ix,jy,kz = ',ix,jy,kz
          // write(0,*) 'dbz = ',dbz(ix,jy,kz)
          // write(0,*) 'db, zhdryc = ',db(ix,jy,kz),zhdryc
          // write(0,*) 'Hail intercept: ',xcnoh,ix,kz
          // write(0,*) 'Hail,snow q: ',an(ix,jy,kz,lh),an(ix,jy,kz,ls)
          // write(0,*) 'graupel density hwdn = ',hwdn
          // write(0,*) 'rain q: ',an(ix,jy,kz,lr)
          // write(0,*) 'ice q: ',an(ix,jy,kz,li)
          if ((P.lhl > 1)) { }  // 9663
          if ((ipconc >= 3)) { }  // 9664
          if ((ipconc >= 5)) {  // 9666
            // write(0,*) 'Hail,snow c: ',an(ix,jy,kz,lnh),an(ix,jy,kz,lns)
            if ((P.lhl > 1)) { }  // 9668
          }  // 9674
          // write(0,*) 'chw,xvh = ', chw,xvh
          // write(0,*) 'dtmps,dtmph,dadh,dadhl,dtmphl = ',dtmps,dtmph,dadh,dadhl,dtmphl
          // write(0,*) 'dtmpr = ',dtmpr
          // write(0,*) 'gtmp = ',gtmp(ix,kz),dtmp(ix,kz)
          if ((!(((dbz(ix, jy, kz) > (-100)) && (dbz(ix, jy, kz) < 200))))) {  // 9679
            // write(0,*) 'dbz out of bounds!'
          }  // 9682
        }  // 9683
      }  // 9686
    }  // 9687
  }  // 9688
  return;  // 9710

}
template <class Real>
CPPWRF_HD inline void calc_eff_radius(const NsslParams<Real>& P, Work W, int nx, int ny, int nz, int na, int jyslab, int nor, int norz, Real* t1_p, Real* t2_p, Real* t3_p, Real* t4_p, Real* t5_p, Real* t6_p, bool f_t4, bool f_t5, bool f_t6, Real* an_p, Real* dn_p) {
  FA<Real> qcw, qci, qsw, qrw, ccw, cci, csw, crw;  // optional, absent
  const int ng1 = 1;  // 5853
  const Real qmin = NR(1.0e-8);  // 5916
  const Real volmin = NR(1.0e-30);  // 5917
  const int ngs = 1;
  FA<Real> t1 = mkfa<Real>(t1_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t2 = mkfa<Real>(t2_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t3 = mkfa<Real>(t3_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t4 = mkfa<Real>(t4_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t5 = mkfa<Real>(t5_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> t6 = mkfa<Real>(t6_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  int ng = 0;  // 5855
  Real dtp = 0;  // 5857
  auto pb = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 5879 
  auto pinit = W.template arr<Real>(((-norz) + ng1), (nz + norz));  // 5880 
  int nxmpb = 0;  // 5885
  int nzmpb = 0;  // 5885
  int nxz = 0;  // 5885
  int mgs = 0;  // 5886
  int numgs = 0;  // 5886
  int inumgs = 0;  // 5886
  int ngscnt = 0;  // 5888
  auto igs = W.template arr<int>(1, ngs);  // 5888 
  auto kgs = W.template arr<int>(1, ngs);  // 5888 
  auto rho0 = W.template arr<Real>(1, ngs);  // 5889 
  int ix = 0;  // 5891
  int kz = 0;  // 5891
  int i = 0;  // 5891
  int n = 0;  // 5891
  int kp1 = 0;  // 5891
  int jy = 0;  // 5892
  int jgs = 0;  // 5892
  int ixb = 0;  // 5893
  int ixe = 0;  // 5893
  int jyb = 0;  // 5893
  int jye = 0;  // 5893
  int kzb = 0;  // 5893
  int kze = 0;  // 5893
  int itile = 0;  // 5895
  int jtile = 0;  // 5895
  int ktile = 0;  // 5895
  int ixend = 0;  // 5896
  int jyend = 0;  // 5896
  int kzend = 0;  // 5896
  int kzbeg = 0;  // 5896
  int nxend = 0;  // 5897
  int nyend = 0;  // 5897
  int nzend = 0;  // 5897
  int nzbeg = 0;  // 5897
  auto qx = W.template arr<Real>(1, ngs, P.lv, P.lhab);  // 5899 
  auto cx = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 5900 
  auto xv = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 5901 
  auto xmas = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 5902 
  auto xdn = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 5903 
  auto xdia = W.template arr<Real>(1, ngs, P.lc, P.lhab, 1, 3);  // 5904 
  auto alpha = W.template arr<Real>(1, ngs, P.lc, P.lhab);  // 5905 
  Real gamc1 = 0;  // 5907
  Real gamc2 = 0;  // 5907
  Real gami1 = 0;  // 5907
  Real gami2 = 0;  // 5907
  Real gams1 = 0;  // 5907
  Real gams2 = 0;  // 5907
  Real factor_c = 0;  // 5907
  Real factor_i = 0;  // 5907
  Real factor_s = 0;  // 5907
  Real lam_c = 0;  // 5908
  Real lam_i = 0;  // 5908
  Real lam_s = 0;  // 5908
  Real lam_r = 0;  // 5908
  Real lam_h = 0;  // 5908
  Real lam_hl = 0;  // 5908
  Real gamr1 = 0;  // 5909
  Real gamr2 = 0;  // 5909
  Real gamh1 = 0;  // 5909
  Real gamh2 = 0;  // 5909
  Real factor_r = 0;  // 5909
  Real factor_h = 0;  // 5909
  Real factor_hl = 0;  // 5909
  int il = 0;  // 5910
  Real hwdn = 0;  // 5911
  Real hldn = 0;  // 5911
  double numh = 0;  // 5912
  double numhl = 0;  // 5912
  double denomh = 0;  // 5912
  double denomhl = 0;  // 5912
  bool flag_t4 = false;  // 5914
  bool flag_t5 = false;  // 5914
  bool flag_t6 = false;  // 5914
  itile = nx;  // 5921
  jtile = ny;  // 5922
  ktile = nz;  // 5923
  ixend = nx;  // 5924
  jyend = ny;  // 5925
  kzend = nz;  // 5926
  nxend = (nx + 1);  // 5927
  nyend = (ny + 1);  // 5928
  nzend = nz;  // 5929
  kzbeg = 1;  // 5930
  nzbeg = 1;  // 5931
  flag_t4 = false;  // 5933
  flag_t5 = false;  // 5934
  flag_t6 = false;  // 5935
  if (true) {  // 5937
    if (true) {  // 5938
      flag_t4 = f_t4;  // 5939
    }  // 5940
  }  // 5941
  if (true) {  // 5943
    if (true) {  // 5944
      flag_t5 = f_t5;  // 5945
    }  // 5946
  }  // 5947
  if (true) {  // 5949
    if (true) {  // 5950
      flag_t6 = f_t6;  // 5951
    }  // 5952
  }  // 5953
  jy = 1;  // 5955
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pb(i_0) = NR(0.0);  // 5956
  for (int i_0 = ((-norz) + ng1); i_0 <= (nz + norz); ++i_0) pinit(i_0) = NR(0.0);  // 5957
  gamc1 = gamma_sp((NR(2.0) + P.cnu));  // 5959
  gamc2 = NR(1.0);  // 5960
  gami1 = gamma_sp((NR(2.0) + P.cinu));  // 5961
  gami2 = NR(1.0);  // 5962
  gams1 = gamma_sp((NR(2.0) + P.snu));  // 5963
  gams2 = gamma_sp((NR(1.0) + P.snu));  // 5964
  gamr1 = gamma_sp((NR(2.0) + P.rnu));  // 5965
  gamr2 = gamma_sp((NR(1.0) + P.rnu));  // 5966
  factor_c = ((((NR(1.0) + P.cnu)) * gamma_sp((NR(1.0) + P.cnu))) / gamma_sp(((NR(5.0) / NR(3.0)) + P.cnu)));  // 5968
  factor_i = ((((NR(1.0) + P.cinu)) * gamma_sp((NR(1.0) + P.cinu))) / gamma_sp(((NR(5.0) / NR(3.0)) + P.cinu)));  // 5969
  factor_s = ((((NR(1.0) + P.snu)) * gamma_sp((NR(1.0) + P.snu))) / gamma_sp(((NR(5.0) / NR(3.0)) + P.snu)));  // 5970
  if (true) {  // 5972
    factor_r = pow(((((((P.pi * ((P.alphar + NR(3.0)))) * ((P.alphar + NR(1.0)))) * ((P.alphar + NR(1.0))))) / NR(6.0))), ((NR(1.0) / NR(3.0))));  // 5976
  }  // 5978
  factor_h = pow(((((((P.pi * ((P.alphah + NR(3.0)))) * ((P.alphah + NR(1.0)))) * ((P.alphah + NR(1.0))))) / NR(6.0))), ((NR(1.0) / NR(3.0))));  // 5980
  factor_hl = pow(((((((P.pi * ((P.alphahl + NR(3.0)))) * ((P.alphahl + NR(1.0)))) * ((P.alphahl + NR(1.0))))) / NR(6.0))), ((NR(1.0) / NR(3.0))));  // 5981
  jgs = jy;  // 5987
  mgs = 1;  // 5989
  for (kz = 1; kz <= nz; ++kz) {  // 5990
    for (ix = 1; ix <= nx; ++ix) {  // 5991
      rho0(mgs) = dn(ix, jy, kz);  // 5993
      if (true) {  // 5994
        for (il = P.lc; il <= P.lhab; ++il) {  // 5995
          qx(mgs, il) = c_max(an(ix, jy, kz, il), NR(0.0));  // 5996
          cx(mgs, il) = c_max(an(ix, jy, kz, P.ln(il)), NR(0.0));  // 5997
        }  // 5998
      } else {  // 5999
        for (int i_1 = P.lv; i_1 <= P.lhab; ++i_1) qx(mgs, i_1) = NR(0.0);  // 6000
        for (int i_1 = P.lc; i_1 <= P.lhab; ++i_1) cx(mgs, i_1) = NR(0.0);  // 6001
        if (false) { qx(mgs, P.lc) = qcw(ix, kz); }  // 6002
        if (false) { qx(mgs, P.li) = qci(ix, kz); }  // 6003
        if (false) { qx(mgs, P.ls) = qsw(ix, kz); }  // 6004
        if (false) { qx(mgs, P.lr) = qrw(ix, kz); }  // 6005
        if (false) { cx(mgs, P.lc) = (ccw(ix, kz) * rho0(mgs)); }  // 6006
        if (false) { cx(mgs, P.li) = (cci(ix, kz) * rho0(mgs)); }  // 6007
        if (false) { cx(mgs, P.ls) = (csw(ix, kz) * rho0(mgs)); }  // 6008
        if (false) { cx(mgs, P.lr) = (crw(ix, kz) * rho0(mgs)); }  // 6009
      }  // 6011
      if (((true && (qx(mgs, P.lc) > P.qxmin(P.lc))) && (cx(mgs, P.lc) > P.cxmin))) {  // 6013
        lam_c = pow(((((((cx(mgs, P.lc) * ((P.pi / NR(6.0)))) * P.xdn0(P.lc)) * gamc1)) / (((qx(mgs, P.lc) * rho0(mgs)) * gamc2)))), ((NR(1.0) / NR(3.0))));  // 6015
        t1(ix, jy, kz) = ((NR(0.5) * factor_c) / lam_c);  // 6016
      }  // 6017
      if (((true && (qx(mgs, P.li) > P.qxmin(P.li))) && (cx(mgs, P.li) > P.cxmin))) {  // 6019
        lam_i = pow(((((((cx(mgs, P.li) * ((P.pi / NR(6.0)))) * P.xdn0(P.li)) * gami1)) / (((qx(mgs, P.li) * rho0(mgs)) * gami2)))), ((NR(1.0) / NR(3.0))));  // 6021
        t2(ix, jy, kz) = ((NR(0.5) * factor_i) / lam_i);  // 6022
      }  // 6023
      if (((true && (qx(mgs, P.ls) > P.qxmin(P.ls))) && (cx(mgs, P.ls) > P.cxmin))) {  // 6025
        lam_s = pow(((((((cx(mgs, P.ls) * ((P.pi / NR(6.0)))) * P.xdn0(P.ls)) * gams1)) / (((qx(mgs, P.ls) * rho0(mgs)) * gams2)))), ((NR(1.0) / NR(3.0))));  // 6027
        t3(ix, jy, kz) = ((NR(0.5) * factor_s) / lam_s);  // 6028
      }  // 6029
      if ((true && ((((false && false)) || flag_t4)))) {  // 6031
        if (((qx(mgs, P.lr) > c_max(qmin, P.qxmin(P.lr))) && (cx(mgs, P.lr) > P.cxmin))) {  // 6032
          lam_r = (factor_r * pow(((((P.xdn0(P.lr) * cx(mgs, P.lr))) / ((qx(mgs, P.lr) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6035
          t4(ix, jy, kz) = ((NR(0.5) * ((P.alphar + NR(3.0)))) / lam_r);  // 6036
        }  // 6042
      }  // 6043
      if ((true && flag_t5)) {  // 6045
        if (((P.lhl < 1) || flag_t6)) {  // 6049
          if ((qx(mgs, P.lh) > c_max(qmin, P.qxmin(P.lh)))) {  // 6051
            hwdn = P.xdn0(P.lh);  // 6053
            if ((P.lvh > 1)) {  // 6054
              if ((an(ix, jy, kz, P.lvh) > volmin)) {  // 6055
                hwdn = ((rho0(mgs) * qx(mgs, P.lh)) / an(ix, jy, kz, P.lvh));  // 6056
              }  // 6057
            }  // 6058
            lam_h = (factor_h * pow(((((hwdn * cx(mgs, P.lh))) / ((qx(mgs, P.lh) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6060
            t5(ix, jy, kz) = ((NR(0.5) * ((P.alphah + NR(3.0)))) / lam_h);  // 6061
          }  // 6062
        } else {  // 6064
          if (((qx(mgs, P.lh) > c_max(qmin, P.qxmin(P.lh))) && (qx(mgs, P.lhl) < c_max(qmin, P.qxmin(P.lhl))))) {  // 6066
            hwdn = P.xdn0(P.lh);  // 6068
            if ((P.lvh > 1)) {  // 6069
              if ((an(ix, jy, kz, P.lvh) > volmin)) {  // 6070
                hwdn = ((rho0(mgs) * qx(mgs, P.lh)) / an(ix, jy, kz, P.lvh));  // 6071
              }  // 6072
            }  // 6073
            lam_h = (factor_h * pow(((((hwdn * cx(mgs, P.lh))) / ((qx(mgs, P.lh) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6075
            t5(ix, jy, kz) = ((NR(0.5) * ((P.alphah + NR(3.0)))) / lam_h);  // 6076
          } else if (((qx(mgs, P.lh) < c_max(qmin, P.qxmin(P.lh))) && (qx(mgs, P.lhl) > c_max(qmin, P.qxmin(P.lhl))))) {  // 6078
            hldn = P.xdn0(P.lhl);  // 6080
            if ((P.lvhl > 1)) {  // 6081
              if ((an(ix, jy, kz, P.lvhl) > volmin)) {  // 6082
                hldn = ((rho0(mgs) * qx(mgs, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 6083
              }  // 6084
            }  // 6085
            lam_hl = (factor_hl * pow(((((hldn * cx(mgs, P.lhl))) / ((qx(mgs, P.lhl) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6087
            t5(ix, jy, kz) = ((NR(0.5) * ((P.alphahl + NR(3.0)))) / lam_hl);  // 6088
          } else if (((qx(mgs, P.lh) > c_max(qmin, P.qxmin(P.lh))) && (qx(mgs, P.lhl) > c_max(qmin, P.qxmin(P.lhl))))) {  // 6090
            hldn = P.xdn0(P.lhl);  // 6093
            if ((P.lvhl > 1)) {  // 6094
              if ((an(ix, jy, kz, P.lvhl) > volmin)) {  // 6095
                hldn = ((rho0(mgs) * qx(mgs, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 6096
              }  // 6097
            }  // 6098
            hwdn = P.xdn0(P.lh);  // 6100
            if ((P.lvh > 1)) {  // 6101
              if ((an(ix, jy, kz, P.lvh) > volmin)) {  // 6102
                hwdn = ((rho0(mgs) * qx(mgs, P.lh)) / an(ix, jy, kz, P.lvh));  // 6103
              }  // 6104
            }  // 6105
            lam_h = (factor_h * pow(((((hwdn * cx(mgs, P.lh))) / ((qx(mgs, P.lh) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6107
            lam_hl = (factor_hl * pow(((((hldn * cx(mgs, P.lhl))) / ((qx(mgs, P.lhl) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6108
            numh = ((((cx(mgs, P.lh) * ((P.alphah + NR(3.0)))) * ((P.alphah + NR(2.0)))) * ((P.alphah + NR(1.0)))) / ipow<3>(lam_h));  // 6110
            numhl = ((((cx(mgs, P.lhl) * ((P.alphahl + NR(3.0)))) * ((P.alphahl + NR(2.0)))) * ((P.alphahl + NR(1.0)))) / ipow<3>(lam_hl));  // 6111
            denomh = (((cx(mgs, P.lh) * ((P.alphah + NR(2.0)))) * ((P.alphah + NR(1.0)))) / ipow<2>(lam_h));  // 6113
            denomhl = (((cx(mgs, P.lhl) * ((P.alphahl + NR(2.0)))) * ((P.alphahl + NR(1.0)))) / ipow<2>(lam_hl));  // 6114
            t5(ix, jy, kz) = ((NR(0.5) * ((numh + numhl))) / ((denomh + denomhl)));  // 6116
          }  // 6119
        }  // 6121
      }  // 6123
      if (((true && flag_t6) && (P.lhl > 1))) {  // 6125
        if ((qx(mgs, P.lhl) > c_max(qmin, P.qxmin(P.lhl)))) {  // 6127
          hldn = P.xdn0(P.lhl);  // 6129
          if ((P.lvhl > 1)) {  // 6130
            if ((an(ix, jy, kz, P.lvhl) > volmin)) {  // 6131
              hldn = ((rho0(mgs) * qx(mgs, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 6132
            }  // 6133
          }  // 6134
          lam_hl = (factor_hl * pow(((((hldn * cx(mgs, P.lhl))) / ((qx(mgs, P.lhl) * rho0(mgs))))), ((NR(1.0) / NR(3.0)))));  // 6136
          t6(ix, jy, kz) = ((NR(0.5) * ((P.alphahl + NR(3.0)))) / lam_hl);  // 6137
        }  // 6139
      }  // 6141
    }  // 6144
  }  // 6145
  return;  // 6147

}
template <class Real>
CPPWRF_HD inline void hailmaxd(const NsslParams<Real>& P, Work W, Real dtp, int nx, int ny, int nz, Real* an_p, int na, int nor, int norz, Real* alpha2d_p, Real* dn_p, Real* hailmax1d_p, Real* hailmaxk1_p, int jslab) {
  const Real dz = NR(200.0);  // 4206
  const double thresh_conc = 0.0005e0;  // 4215
  const int ng1 = 1;
  FA<Real> an = mkfa<Real>(an_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz), 1, na);
  FA<Real> alpha2d = mkfa<Real>(alpha2d_p, ((-nor) + 1), (nx + nor), 1, 1, ((-norz) + 1), (nz + norz), 1, 3);
  FA<Real> dn = mkfa<Real>(dn_p, ((-nor) + 1), (nx + nor), ((-nor) + 1), (ny + nor), ((-norz) + 1), (nz + norz));
  FA<Real> hailmax1d = mkfa<Real>(hailmax1d_p, 1, nx, 1, ny);
  FA<Real> hailmaxk1 = mkfa<Real>(hailmaxk1_p, 1, nx, 1, ny);
  int ngt = 0;  // 4187
  int jgs = 0;  // 4187
  int ia = 0;  // 4187
  int id = 0;  // 4188
  int infdo = 0;  // 4201
  int ix = 0;  // 4204
  int jy = 0;  // 4204
  int kz = 0;  // 4204
  int ndfall = 0;  // 4204
  int n = 0;  // 4204
  int k = 0;  // 4204
  int il = 0;  // 4204
  int in = 0;  // 4204
  double tmp = 0;  // 4205
  double ratio = 0;  // 4205
  double del = 0;  // 4205
  double g1palp = 0;  // 4205
  auto db1 = W.template arr<Real>(1, nx, 1, (nz + 1));  // 4208 
  auto dtz1 = W.template arr<Real>(1, (nz + 1), 1, nx, 0, 1);  // 4208 
  auto dz2dinv = W.template arr<Real>(1, (nz + 1), 1, nx);  // 4208 
  auto db1inv = W.template arr<Real>(1, nx, 1, (nz + 1));  // 4208 
  auto rhovtzx = W.template arr<Real>(1, nz, 1, nx);  // 4210 
  Real alp = 0;  // 4212
  Real diam = 0;  // 4212
  Real diam1 = 0;  // 4212
  Real hwdn = 0;  // 4212
  Real cwchtmp = 0;  // 4216
  Real cwchltmp = 0;  // 4216
  Real maxdia = 0;  // 4216
  int ixb = 0;  // 4220
  int jyb = 0;  // 4220
  int kzb = 0;  // 4220
  int ixe = 0;  // 4221
  int jye = 0;  // 4221
  int kze = 0;  // 4221
  int plo = 0;  // 4222
  int phi = 0;  // 4222
  int ialp = 0;  // 4223
  int i = 0;  // 4223
  int j = 0;  // 4223
  bool debug_mpi = false;  // 4225
  cwchtmp = pow((((((NR(3.0) + P.dnu(P.lh))) * ((NR(2.0) + P.dnu(P.lh)))) * ((NR(1.0) + P.dnu(P.lh))))), ((-(NR(1.0) / NR(3.0)))));  // 4231
  if ((P.lhl > 1)) {  // 4233
    cwchltmp = pow((((((NR(3.0) + P.dnu(P.lhl))) * ((NR(2.0) + P.dnu(P.lhl)))) * ((NR(1.0) + P.dnu(P.lhl))))), ((-(NR(1.0) / NR(3.0)))));  // 4234
  }  // 4235
  kzb = 1;  // 4238
  kze = nz;  // 4239
  ixb = 1;  // 4241
  ixe = nx;  // 4242
  jy = jslab;  // 4245
  jgs = jy;  // 4246
  if (((P.lh > 1) && (P.lnh > 1))) {  // 4256
    for (kz = kzb; kz <= kze; ++kz) {  // 4257
      for (ix = ixb; ix <= ixe; ++ix) {  // 4258
        if (((an(ix, jy, kz, P.lh) > P.qxmin(P.lh)) && (an(ix, jy, kz, P.lnh) > thresh_conc))) {  // 4259
          if ((P.lvh > 1)) {  // 4260
            hwdn = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / an(ix, jy, kz, P.lvh));  // 4261
          } else {  // 4262
            hwdn = P.rho_qh;  // 4263
          }  // 4264
          tmp = (NR(1.0) + alpha2d(ix, 1, kz, 2));  // 4266
          i = int((P.dgami * (tmp)));  // 4267
          del = (tmp - (P.dgam * i));  // 4268
          g1palp = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 4269
          tmp = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lh)) / ((hwdn * an(ix, jy, kz, P.lnh))));  // 4271
          diam = pow((((NR(6.0) * tmp) / P.pi)), ((NR(1.0) / NR(3.0))));  // 4272
          diam1 = (diam * cwchtmp);  // 4276
          tmp = ((thresh_conc * g1palp) / an(ix, jy, kz, P.lnh));  // 4282
          alp = alpha2d(ix, 1, kz, 2);  // 4283
          j = int((c_max(NR(0.0), c_min(P.maxalphalu, alp)) * P.dqiacralphainv));  // 4285
          ratio = NR(0.0);  // 4286
          maxdia = NR(0.0);  // 4287
          for (i = 0; i <= (P.nqiacrratio - 1); ++i) {  // 4290
            if (((P.gamxinflu(i, j, 1, 1) >= tmp) && (tmp >= P.gamxinflu((i + 1), j, 1, 1)))) {  // 4291
              ratio = (i * P.dqiacrratio);  // 4293
              del = (tmp - P.gamxinflu(i, j, 1, 1));  // 4294
              ratio = (((NR(i) + (del / ((P.gamxinflu((i + 1), j, 1, 1) - P.gamxinflu(i, j, 1, 1)))))) * P.dqiacrratio);  // 4295
              break;  // 4296
            }  // 4297
          }  // 4298
          if ((ratio > NR(0.0))) {  // 4300
            maxdia = (ratio * diam1);  // 4301
          }  // 4302
          if ((kz == kzb)) {  // 4304
            hailmaxk1(ix, jy) = c_max(maxdia, hailmaxk1(ix, jy));  // 4305
          }  // 4313
          hailmax1d(ix, jy) = c_max(maxdia, hailmax1d(ix, jy));  // 4315
        }  // 4319
      }  // 4321
    }  // 4322
  }  // 4324
  if (((P.lhl > 1) && (P.lnhl > 1))) {  // 4327
    for (kz = kzb; kz <= kze; ++kz) {  // 4328
      for (ix = ixb; ix <= ixe; ++ix) {  // 4329
        if (((an(ix, jy, kz, P.lhl) > P.qxmin(P.lhl)) && (an(ix, jy, kz, P.lnhl) > thresh_conc))) {  // 4330
          if ((P.lvhl > 1)) {  // 4331
            hwdn = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / an(ix, jy, kz, P.lvhl));  // 4332
          } else {  // 4333
            hwdn = P.rho_qhl;  // 4334
          }  // 4335
          tmp = (NR(1.0) + alpha2d(ix, 1, kz, 3));  // 4337
          i = int((P.dgami * (tmp)));  // 4338
          del = (tmp - (P.dgam * i));  // 4339
          g1palp = (P.gmoi(i) + ((((P.gmoi((i + 1)) - P.gmoi(i))) * del) * P.dgami));  // 4340
          tmp = ((dn(ix, jy, kz) * an(ix, jy, kz, P.lhl)) / ((hwdn * an(ix, jy, kz, P.lnhl))));  // 4342
          diam = pow((((NR(6.0) * tmp) / P.pi)), ((NR(1.0) / NR(3.0))));  // 4343
          diam1 = (diam * cwchltmp);  // 4347
          tmp = ((thresh_conc * g1palp) / an(ix, jy, kz, P.lnhl));  // 4353
          alp = alpha2d(ix, 1, kz, 3);  // 4354
          j = int((c_max(NR(0.0), c_min(P.maxalphalu, alp)) * P.dqiacralphainv));  // 4356
          ratio = NR(0.0);  // 4357
          maxdia = NR(0.0);  // 4358
          for (i = 0; i <= (P.nqiacrratio - 1); ++i) {  // 4361
            if (((P.gamxinflu(i, j, 1, 1) >= tmp) && (tmp >= P.gamxinflu((i + 1), j, 1, 1)))) {  // 4362
              ratio = (i * P.dqiacrratio);  // 4364
              del = (tmp - P.gamxinflu(i, j, 1, 1));  // 4365
              ratio = (((NR(i) + (del / ((P.gamxinflu((i + 1), j, 1, 1) - P.gamxinflu(i, j, 1, 1)))))) * P.dqiacrratio);  // 4366
              break;  // 4367
            }  // 4368
          }  // 4369
          if ((ratio > NR(0.0))) {  // 4371
            maxdia = (ratio * diam1);  // 4372
          }  // 4373
          if ((kz == kzb)) {  // 4375
            hailmaxk1(ix, jy) = c_max(maxdia, hailmaxk1(ix, jy));  // 4376
          }  // 4384
          hailmax1d(ix, jy) = c_max(maxdia, hailmax1d(ix, jy));  // 4386
        }  // 4390
      }  // 4392
    }  // 4393
  }  // 4395

}



}  // namespace cppwrf::phys::mp_nssl
