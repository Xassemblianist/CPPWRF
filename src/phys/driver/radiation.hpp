// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// WRF phys/module_radiation_driver.F radiation_driver for ra_lw_physics = 4 (RRTMG) or 1 (RRTM, module_ra_rrtm.F)
// with ra_sw_physics = 4 (RRTMG) or 1 (Dudhia, module_ra_sw.F) with o3input = 2 (CAM ozone), ghg_input = 1
// (CAMtr_volume_mixing_ratio), aer_opt = 0, swint_opt = 0, icloud = 1, cldovrlp, idcor, cu_rad_feedback,
// single domain; slope_rad / topo_shading: pre_radiation_driver's toposhad and the surface driver's
// TOPO_RAD_ADJ_DRVR in src/phys/driver/topo.hpp (called from suite.cpp / surface_driver.cpp).
// CPU builds: one column at a time per OpenMP thread (rrtmg_lw_column / rrtm_lw_column / rrtmg_sw_column).
// CUDA builds: everything on the device: inputs staged column-fastest, coszen, CAM ozone and
// cal_cldfra1 per column, the RRTMG stages as kernels (RrtmgLwBatch / RrtmgSwBatch); RRTM LW on the host
// (rrtm_lw_batch) from the staged columns.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "core/field.hpp"
#include "core/libm.hpp"
#include "core/grid.hpp"
#include "core/wrf_time.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include "phys/ra_rrtmg_common/strided.hpp"

namespace cppwrf::phys::driver {

struct RadiationOptions {
  int ra_lw_physics = 4, ra_sw_physics = 4;
  rb radt = 0;          // minutes; stepra = max(nint(radt*60/dt), 1)
  int ra_call_offset = 0;
  int icloud = 1, cldovrlp = 2, idcor = 0, o3input = 2, ghg_input = 1, aer_opt = 0;
  // cu_rad_feedback (phy_init's icloud_cu): 1 = Grell schemes (QC_CU / QI_CU added to qc / qi), 2 = KF (CLDFRA_DP
  // + CLDFRA_SH into CLDFRA, QC_CU / QI_CU times that fraction into qc / qi), 0 = none
  int icloud_cu = 0;
  // bmj_rad_feedback with BMJ (cu_physics = 2; module_radiation_driver.F:1578-1591): QCCONV / QICONV added to qc /
  // qi and the convective cloud fraction CCLDFRA combined with CLDFRA (after every other cloud change)
  bool bmj_feedback = false;
  // oznini and the ozone interpolation run on domain 1 only (module_physics_init.F, module_radiation_driver.F,
  // EM_CORE): a nest's radiation uses o3rad, which med_nest_force interpolates from the parent (Registry f=p2c)
  bool oznini = true;
  rb swrad_scat = 1;   // Dudhia: swinit's CSSCA = swrad_scat * 1.e-5
  bool warm_rain = false;  // mp_init: Kessler (Dudhia keeps its cloud and rain liquid without qi)
  int julyr = 0, julday = 1;  // start (wrfinput JULYR/JULDAY), gmt start hour
  rb gmt = 0;
  rb p_top = 0;
  std::string table_dir = ".";  // RRTMG_*_DATA tables dump, ozone*.formatted, CAMtr_volume_mixing_ratio
};

struct RadiationInputs {  // device fields (mass points) read by the radiation
  const Field3 *t_phy, *p_hyd, *p_hyd_w, *pi_phy, *rho, *dz8w, *t8w, *qv;
  const Field3 *qc, *qi, *qs;  // moist species (nullptr when the scheme has none: WRF F_QC, F_QI, F_QS)
  const Field3 *re_cloud, *re_ice, *re_snow;  // microphysics effective radii (m): has_reqc/i/s, else nullptr
  const Field2 *tsk, *albedo, *emiss, *xland, *xice, *snow;
  // icloud_bl = 1 with the MYNN PBL: subgrid clouds (module_radiation_driver.F, ICLOUD == 1): after step 1
  // CLDFRA = CLDFRA_BL; QC_BL / QI_BL are added where qc < 1e-6 / qi < 1e-8 and CLDFRA_BL > 0.001
  const Field3 *cldfra_bl = nullptr, *qc_bl = nullptr, *qi_bl = nullptr;
  // icloud_cu > 0: the cumulus scheme's clouds of its last call (radiation runs before cumulus_driver)
  const Field3 *qc_cu = nullptr, *qi_cu = nullptr, *cldfra_dp = nullptr, *cldfra_sh = nullptr;
  // bmj_feedback: BMJ's clouds of its last call
  const Field3 *qcconv = nullptr, *qiconv = nullptr, *ccldfra = nullptr;
  // Dudhia SW: rain and graupel (nullptr without F_QR / F_QG) and the terrain height of the direct / diffuse split
  const Field3 *qr = nullptr, *qg = nullptr;
  const BaseField2* ht = nullptr;
};

struct RadiationOutputs {
  Field3* rthraten;  // uncoupled theta tendency (K/s), LW + SW
  Field2 *glw, *gsw, *swdown, *coszen;
  Field3* cldfra;    // cloud fraction (icloud = 1: cal_cldfra1), may be nullptr
  // diagnostics (may be nullptr): RTHRATLW, RTHRATSW; LWUPT, LWUPTC, LWDNT, LWDNTC, LWUPB, LWUPBC, LWDNB,
  // LWDNBC, OLR; SWUPT, SWUPTC, SWDNT, SWDNTC, SWUPB, SWUPBC, SWDNB, SWDNBC (0 at night)
  Field3 *rthratlw = nullptr, *rthratsw = nullptr;
  Field3 *rthratlwc = nullptr, *rthratswc = nullptr, *o3rad = nullptr;  // clear-sky heating, ozone (o3input = 2)
  Field2 *lwcf = nullptr, *swcf = nullptr, *hrang = nullptr;
  Field2* lw[9] = {};
  Field2* sw[8] = {};
  // RRTMG_SWRAD's surface components (restart fields, may be nullptr; 0 at night): SWVISDIR, SWVISDIF, SWNIRDIR,
  // SWNIRDIF, SWDDIR, SWDDNI, SWDDIF, SWDDIRC, SWDDNIC (DNI = the direct flux / coszen); radiation_driver's
  // DIFFUSE_FRAC = min(SWDDIF / SWDOWN, 1) where SWDOWN > 0.001, else 0
  Field2* swsfc[10] = {};
};

// grid%julian at the start of a step, the clock's time el since the simulation start (julyr, julday, gmt):
// domain_clock_get's REAL(dayOfYear_r8) - 1 with ESMF_TimeGet's dayOfYear_r8 = (S + Sn/Sd) / 86400 + 1, S the whole
// seconds since 1 January of the current year (0 again after a year change)
inline rb wrf_julian(int julyr, int julday, rb gmt, const wrf_time::Interval& el) {
  long long s = (long long)(julday - 1) * 86400 + std::llround(gmt * rb(3600.0)) + el.S;
  for (int yr = julyr;; ++yr) {
    const long long ylen = (((yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0) ? 366LL : 365LL) * 86400LL;
    if (s < ylen) break;
    s -= ylen;
  }
  // dayOfYear_r8 is REAL(8) (ESMF), currentDayOfYearReal = REAL(currentDayOfYearR8) - 1.0 a REAL
  double rsec = double(s);
  if (el.Sd != 0) rsec = rsec + double(el.Sn) / double(el.Sd);
  return rb(rsec / 86400.0 + 1.0) - rb(1.0);
}
inline int wrf_year(int julyr, int julday, rb gmt, const wrf_time::Interval& el) {
  long long s = (long long)(julday - 1) * 86400 + std::llround(gmt * rb(3600.0)) + el.S;
  int yr = julyr;
  for (;; ++yr) {
    const long long ylen = (((yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0) ? 366LL : 365LL) * 86400LL;
    if (s < ylen) break;
    s -= ylen;
  }
  return yr;
}

// calc_coszen (module_radiation_driver.F) for one column; xtime in minutes, angles in degrees
// li, ln: lane li of WRF's n = ln-trip i loop (its..ite = 0..nx-1), which gfortran vectorises: sin(xxlat),
// cos(xxlat) and cos(hrang) through libmvec's 2-lane functions (cppwrf::vm); ln = 1: scalar libm
CPPWRF_HD inline rb calc_coszen(rb julian, rb xtime, rb gmt, rb declin, rb xlon,
                                    rb xlat, int li = 0, int ln = 1, rb* hrang_out = nullptr) {
  constexpr rb piconst = rb(3.1415926535897932384626433);
  constexpr rb degrad = piconst / rb(180.);
  const rb da = rb(6.2831853071795862) * (julian - rb(0.5)) / rb(365.);
  const rb eot = (rb(0.000075) + rb(0.001868) * libm::cos(da) - rb(0.032077) * libm::sin(da) - rb(0.014615) * libm::cos(2 * da) -
                      rb(0.040849) * libm::sin(2 * da)) * (rb(229.18));
  const rb xt24 = std::fmod(xtime, rb(1440.)) + eot;
  const rb tloctm = gmt + xt24 / rb(60.) + xlon / rb(15.);
  const rb hrang = rb(15.) * (tloctm - rb(12.)) * degrad;
  if (hrang_out) *hrang_out = hrang;
  const rb xxlat = xlat * degrad;
  rb c = vm::sin(xxlat, li, ln) * libm::sin(declin) + vm::cos(xxlat, li, ln) * libm::cos(declin) * vm::cos(hrang, li, ln);
  return std::min(std::max(c, -rb(1.0)), rb(1.0));
}

// module_radiation_driver.F cal_cldfra1 for one column (bottom to top): qi / qs with p == nullptr
// when the scheme has no F_QI / F_QS; t (K) and p (Pa) are t_phy and p_hyd. mp_physics !=
// FER_MP_HIRES; F_QC and not F_QI but F_QS needs F_ICE_PHY (not ported).
CPPWRF_HD inline void cal_cldfra1(int nz, SD qv, SD qc, SD qi, SD qs, SD t, SD p, SD cldfra) {
  constexpr rb ALPHA0 = rb(100.), GAMMA = rb(0.49), QCLDMIN = rb(1.E-12), PEXP = rb(0.25), RHGRID = rb(1.0);
  constexpr rb SVP1 = rb(0.61078), SVP2 = rb(17.2693882), SVPI2 = rb(21.8745584), SVP3 = rb(35.86), SVPI3 = rb(7.66),
                   SVPT0 = rb(273.15), r_d = rb(287.), r_v = rb(461.6), ep_2 = r_d / r_v;
  for (int k = 0; k < nz; ++k) {
    const rb tk = t[k], pk = p[k];
    const rb tc = tk - SVPT0;
    const rb esw = rb(1000.0) * SVP1 * libm::exp(SVP2 * tc / (tk - SVP3));
    const rb esi = rb(1000.0) * SVP1 * libm::exp(SVPI2 * tc / (tk - SVPI3));
    const rb QVSW = ep_2 * esw / (pk - esw);
    const rb QVSI = ep_2 * esi / (pk - esi);
    rb QCLD = 0, weight = 0;
    if (qi.p && qs.p) {
      QCLD = qi[k] + qc[k] + qs[k];
      weight = QCLD < QCLDMIN ? rb(0.) : (qi[k] + qs[k]) / QCLD;
    } else if (qi.p) {
      QCLD = qi[k] + qc[k];
      weight = QCLD < QCLDMIN ? rb(0.) : qi[k] / QCLD;
    } else {  // F_QC only
      QCLD = qc[k];
      if (QCLD < QCLDMIN) weight = rb(0.);
      else weight = tk > rb(273.15) ? rb(0.) : rb(1.);
    }
    const rb QVS_WEIGHT = (1 - weight) * QVSW + weight * QVSI;
    rb RHUM = qv[k] / QVS_WEIGHT;
    rb cf;
    if (QCLD < QCLDMIN) {
      cf = rb(0.);
    } else if (RHUM >= RHGRID) {
      cf = rb(1.);
    } else {
      const rb SUBSAT = std::max(rb(1.E-10), RHGRID * QVS_WEIGHT - qv[k]);
      const rb DENOM = libm::pow(SUBSAT, GAMMA);
      const rb ARG = std::max(-rb(6.9), -ALPHA0 * QCLD / DENOM);
      RHUM = std::max(rb(1.E-10), RHUM);
      cf = libm::pow(RHUM / RHGRID, PEXP) * (rb(1.) - libm::exp(ARG));
      if (cf < rb(.01)) cf = rb(0.);
    }
    cldfra[k] = cf;
  }
}

// radconst (module_radiation_driver.F): the solar declination (rad) and the solar constant of the julian day
inline void radconst(rb julian, rb& declin, rb& solcon) {
  constexpr rb piconst = rb(3.1415926535897932384626433), degrad = piconst / rb(180.), dpd = rb(360.) / rb(365.);
  const rb obecl = rb(23.5) * degrad, sinob = libm::sin(obecl);
  rb sxlong = julian >= rb(80.) ? dpd * (julian - rb(80.)) : dpd * (julian + rb(285.));
  sxlong = sxlong * degrad;
  const rb arg = sinob * libm::sin(sxlong);
  declin = libm::asin(arg);
  const rb djul = julian * rb(360.) / rb(365.), rjul = djul * degrad;
  const rb eccfac = rb(1.000110) + rb(0.034221) * libm::cos(rjul) + rb(0.001280) * libm::sin(rjul) +
                    rb(0.000719) * libm::cos(2 * rjul) + rb(0.000077) * libm::sin(2 * rjul);
  solcon = rb(1370.) * eccfac;
}

class RadiationDriver {
 public:
  RadiationDriver(const Grid& g, const RadiationOptions& o, const std::vector<rb>& xlat,
                  const std::vector<rb>& xlong);
  ~RadiationDriver();
  int stepra() const { return stepra_; }
  // radiation_driver for time step itimestep (1-based) at xtime minutes after the start,
  // julian (day of year - 1 + fraction) and yr (the year of that time, RRTMG's YR); returns false
  // when the radiation is not called. adapt_step_flag: the adaptive time step's decision by
  // curr_secs >= radtacttime (radtacttime is updated as WRF does)
  bool run(long itimestep, rb xtime, rb julian, int yr, const RadiationInputs& in,
           const RadiationOutputs& out, bool adapt_step_flag = false, real curr_secs = 0,
           real* radtacttime = nullptr);
  rb declin() const { return declin_; }  // radconst of the last call (DECLINX, SOLCONX)
  rb solcon() const { return solcon_; }
  // a restart: grid%declin and grid%solcon of the restart file (the values of the last call before it)
  void set_solar(rb declin, rb solcon) { declin_ = declin; solcon_ = solcon; }
  // a moved nest (share/mediation_nest_move.F): its latitudes and longitudes after the shift and the interpolation
  // (the copies calc_coszen and the RRTMG columns read; the CAM ozone of oznini is domain 1's only)
  void set_latlon(const std::vector<rb>& xlat, const std::vector<rb>& xlong);

 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
  Grid g_;
  RadiationOptions o_;
  int stepra_ = 1;
  rb declin_ = 0, solcon_ = 0;
};

}  // namespace cppwrf::phys::driver
