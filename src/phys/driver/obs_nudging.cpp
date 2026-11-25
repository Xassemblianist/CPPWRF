// SPDX-License-Identifier: GPL-3.0-or-later
// Observation nudging, obs_nudge_opt = 1 (statement ports, WRF v4.8.0; WRF indices in the comments, C++ i = I - 1,
// j = J - 1, k = K - 1):
//   fddaobs_init     phys/module_fddaobs_rtfdda.F:42-513 (get_base_state_height_column :2983-3030)
//   in4dob           share/wrf_fddaobs_in.F:35-1375 (rh2r :1459, set_projection :1551, rotate_vector :1902;
//                    share/module_llxy.F map_set, set_lc, lc_cone, llij_lc; share/module_date_time.F geth_idts)
//   fddaobs_driver   phys/module_fddaobs_driver.F:35-470 (compute_VIH, get_vif_hts_slab)
//   errob            phys/module_fddaobs_rtfdda.F:517-1398 (upoint, vpoint, ht_to_p, get_height_column)
//   nudob            phys/module_fddaobs_rtfdda.F:1522-2744 (get_timewt :3032)
// WRF's reference build is serial (DM_PARALLEL with STUBMPI): the DM_PARALLEL && !STUBMPI blocks of errob and nudob
// (get_full_obs_vector, ERRF(9) / kpbl_obs from ERRF) are not part of it. nudob's local kpbl_obs is then never set:
// a single-level upper-air observation whose layer komin ... komax holds the PBL top level reads it undefined (WRF
// 4.8.0 module_fddaobs_rtfdda.F:2269-2284); that case stops the run here (kFatalObsKpblObs).
#include "phys/driver/obs_nudging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "core/constants.hpp"
#include "core/libm.hpp"
#include "core/parse_real.hpp"
#include "core/tiles.hpp"
#include "core/wrf_time.hpp"
#include "phys/driver/suite.hpp"

namespace cppwrf::phys::driver {

namespace {

using V3 = View3<real>;
using C3 = View3<const real>;
using V2 = View2<real>;
using C2 = View2<const real>;
using B2 = BView2<const real>;
using B3 = BView3<const real>;
using IC2 = View2<const int>;
using C1 = View1<const real>;

constexpr int kMaxObsLevels = 128;  // the column arrays of nudob (WRF: automatic arrays of kte)

[[noreturn]] void obs_fatal(const std::string& msg) {
  for (std::FILE* f : {stdout, stderr}) {
    std::fprintf(f, "-------------- FATAL CALLED ---------------\n%s\n-------------------------------------------\n",
                 msg.c_str());
    std::fflush(f);
  }
  throw std::runtime_error(msg);
}

// ---------------------------------------------------------------------------------------------------------------
// formatted input as gfortran reads it (one record per READ; a short record is padded with blanks)

std::string field(const std::string& rec, std::size_t pos, std::size_t w) {
  std::string s = pos < rec.size() ? rec.substr(pos, w) : std::string();
  s.resize(w, ' ');
  return s;
}

// Fw.d: blanks ignored (BN), all blanks 0, without a decimal point the last d digits are the fraction; one correctly
// rounded conversion to the kind (libgfortran read_f: strtod / strtof)
real read_f(const std::string& rec, std::size_t pos, std::size_t w, int d) {
  std::string s;
  for (char c : field(rec, pos, w))
    if (c != ' ') s += (c == 'd' || c == 'D' || c == 'q' || c == 'Q') ? 'e' : c;
  if (s.empty()) return real(0);
  if (s.find('.') == std::string::npos) {
    std::size_t e = s.find_first_of("eE");
    std::string mant = s.substr(0, e), ex = e == std::string::npos ? std::string() : s.substr(e);
    std::string sign;
    if (!mant.empty() && (mant[0] == '+' || mant[0] == '-')) {
      sign = mant.substr(0, 1);
      mant = mant.substr(1);
    }
    while (int(mant.size()) <= d) mant = "0" + mant;
    s = sign + mant.substr(0, mant.size() - d) + "." + mant.substr(mant.size() - d) + ex;
  }
  char* end = nullptr;
  const real v = strto_kind<real>(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0')
    obs_fatal("obs nudging: cannot read the F" + std::to_string(w) + "." + std::to_string(d) + " field '" +
              field(rec, pos, w) + "' of an OBS_DOMAIN record");
  return v;
}

bool read_l(const std::string& rec, std::size_t pos, std::size_t w) {
  std::string s;
  for (char c : field(rec, pos, w))
    if (c != ' ') s += c;
  if (!s.empty() && s[0] == '.') s = s.substr(1);
  if (!s.empty() && (s[0] == 'T' || s[0] == 't')) return true;
  if (!s.empty() && (s[0] == 'F' || s[0] == 'f')) return false;
  obs_fatal("obs nudging: cannot read the L" + std::to_string(w) + " field '" + field(rec, pos, w) +
            "' of an OBS_DOMAIN record");
}

int read_i(const std::string& rec, std::size_t pos, std::size_t w) {
  std::string s;
  for (char c : field(rec, pos, w))
    if (c != ' ') s += c;
  if (s.empty()) return 0;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (*end != '\0')
    obs_fatal("obs nudging: cannot read the I" + std::to_string(w) + " field '" + field(rec, pos, w) +
              "' of an OBS_DOMAIN record");
  return int(v);
}

// seconds of YYYY-MM-DD_hh:mm:ss since 0000-03-01 (proleptic Gregorian, as geth_idts counts)
long long date_seconds(const std::string& d) {
  auto num = [&](std::size_t p, std::size_t n) { return std::stoll(d.substr(p, n)); };
  long long y = num(0, 4), m = num(5, 2), dd = num(8, 2);
  y -= m <= 2;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const long long yoe = y - era * 400;
  const long long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + dd - 1;
  const long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long long days = era * 146097 + doe;
  return days * 86400 + num(11, 2) * 3600 + num(14, 2) * 60 + num(17, 2);
}

// ---------------------------------------------------------------------------------------------------------------
// kernels

// get_timewt (module_fddaobs_rtfdda.F:3032-3070)
CPPWRF_HD inline real get_timewt(real xtime, real dtmin, real twindo, real scalef, real obtime) {
  const real fdtim = xtime - dtmin;
  const real tw1 = twindo / real(2.) * real(60.) * scalef;
  const real tw2 = twindo * real(60.) * scalef;
  const real tconst = real(1.) / tw1;
  real timewt = real(0.0);
  const real ttim = obtime * real(60.);
  const real dift = std::fabs(fdtim - ttim);
  if (dift <= tw1) timewt = real(1.0);
  if (dift > tw1 && dift <= tw2) {
    if (fdtim < ttim) timewt = (fdtim - (ttim - tw2)) * tconst;
    if (fdtim > ttim) timewt = ((ttim + tw2) - fdtim) * tconst;
  }
  return timewt;
}

CPPWRF_HD inline real fmaxr(real a, real b) { return a > b ? a : b; }  // gfortran MAX(a, b), a first
CPPWRF_HD inline real fminr(real a, real b) { return a < b ? a : b; }

// the driver's REGIME for surface layers that do not set it (module_fddaobs_driver.F:286-299)
struct KObsRegime {
  V2 regime;
  C2 pblh;
  C3 z_at_w;
  B2 ht;
  CPPWRF_HD void operator()(int i, int j) const {
    if (pblh(i, j) > z_at_w(i, j, 1) - ht(i, j)) regime(i, j) = real(4.0);
    else regime(i, j) = real(1.0);
  }
};

struct VifSet {
  real vif[6];
  real hmax, fullmin, rampmin;
};

// compute_VIH + get_vif_hts_slab (module_fddaobs_driver.F:478-603)
struct KObsVih {
  VifSet v;
  V2 vih1, vih2;
  C2 regime, pblh;
  CPPWRF_HD void operator()(int i, int j) const {
    real fullr, rampr;
    const real r = regime(i, j);
    if (r == real(1.0)) {
      fullr = v.vif[0];
      rampr = v.vif[1];
    } else if (r == real(2.0)) {
      fullr = v.vif[2];
      rampr = v.vif[3];
    } else if (r == real(3.0) || r == real(4.0)) {
      fullr = v.vif[4];
      rampr = v.vif[5];
    } else {
      raise_fatal(kFatalObsRegime);
      return;
    }
    real ht1, ht2;
    if (fullr >= real(0.0)) ht1 = fullr;
    else ht1 = pblh(i, j) - (fullr + real(5000.));
    ht1 = fmaxr(v.fullmin, ht1);
    if (rampr >= real(0.0)) ht2 = ht1 + fmaxr(v.rampmin, rampr);
    else ht2 = fmaxr(ht1 + v.rampmin, pblh(i, j) - (rampr + real(5000.)));
    ht1 = fminr(ht1, v.hmax - v.rampmin);
    ht2 = fminr(ht2, v.hmax);
    vih1(i, j) = ht1;
    vih2(i, j) = ht2;
  }
};

// compute_diff_metrics' z (module_diffusion_em.F; dyn/diffusion/diff_metrics.hpp KDiffMetricsZ)
struct KObsDiffZ {
  C3 ph;
  B3 phb;
  V3 z;
  int nz;
  CPPWRF_HD void operator()(int i, int j) const {
    for (int k = 0; k < nz; ++k)
      z(i, j, k) = real(0.5) * (ph(i, j, k) + phb(i, j, k) + ph(i, j, k + 1) + phb(i, j, k + 1)) / real(cst::g);
  }
};

struct ErrobScalars {
  int ide, jde, k_end;  // k_end = min(kte, kde - 1) = nz
  int iswind, istemp, ismois;
  real t0, rcp;
};

// errob for one observation n (module_fddaobs_rtfdda.F:701-1390; the observations are independent)
struct KErrob {
  ErrobScalars c;
  ObsDevArrays o;
  C3 ub, vb, tb, qvb, pp, z;
  B3 pbase;
  C2 uratx, vratx, tratx;
  IC2 kpbl;

  // WRF (I, K, J) of a mass / u / v field
  CPPWRF_HD real U(int I, int K, int J) const { return ub(I - 1, J - 1, K - 1); }
  CPPWRF_HD real Vv(int I, int K, int J) const { return vb(I - 1, J - 1, K - 1); }
  CPPWRF_HD real T(int I, int K, int J) const { return tb(I - 1, J - 1, K - 1); }
  CPPWRF_HD real Q(int I, int K, int J) const { return qvb(I - 1, J - 1, K - 1); }
  CPPWRF_HD real PB(int I, int K, int J) const { return pbase(I - 1, J - 1, K - 1); }
  CPPWRF_HD real PP(int I, int K, int J) const { return pp(I - 1, J - 1, K - 1); }
  CPPWRF_HD real Z(int I, int K, int J) const { return z(I - 1, J - 1, K - 1); }
  // upoint / vpoint (module_fddaobs_rtfdda.F:1426-1477): the ratios at the u points I = 2 ... ide - 1 and at the
  // v points J = 2 ... jde - 1 (errob reads no others)
  CPPWRF_HD real uratio(int I, int J) const { return real(0.5) * (uratx(I - 1, J - 1) + uratx(I - 2, J - 1)); }
  CPPWRF_HD real vratio(int I, int J) const { return real(0.5) * (vratx(I - 1, J - 1) + vratx(I - 1, J - 2)); }

  template <class F>
  CPPWRF_HD static real tri(const F& f, int iob, int job, int kob, int kobp, real dxob, real dyob, real dzob) {
    return ((real(1.) - dzob) *
                ((real(1.) - dyob) * ((real(1.) - dxob) * f(iob, kob, job) + dxob * f(iob + 1, kob, job)) +
                 dyob * ((real(1.) - dxob) * f(iob, kob, job + 1) + dxob * f(iob + 1, kob, job + 1))) +
            dzob * ((real(1.) - dyob) * ((real(1.) - dxob) * f(iob, kobp, job) + dxob * f(iob + 1, kobp, job)) +
                    dyob * ((real(1.) - dxob) * f(iob, kobp, job + 1) + dxob * f(iob + 1, kobp, job + 1))));
  }

  // ht_to_p + get_height_column (module_fddaobs_rtfdda.F:2879-2981): pbbc, ppbc as errob passes them (ppbo, pbbo)
  CPPWRF_HD real ht_to_p(real h, const real* pbbc, const real* ppbc, int ic, int jc, real dx, real dy) const {
    real z_at_p[kMaxObsLevels + 1];
    for (int k = 1; k <= c.k_end; ++k)  // kds ... kde in WRF; the level kde is never read
      z_at_p[k] = (real(1.) - dy) * ((real(1.) - dx) * Z(ic, k, jc) + dx * Z(ic + 1, k, jc)) +
                  dy * ((real(1.) - dx) * Z(ic, k, jc + 1) + dx * Z(ic + 1, k, jc + 1));
    int klo = 1;
    for (int k = 2; k <= c.k_end; ++k) {
      klo = k - 1;
      if (h <= z_at_p[k]) break;
    }
    const real zlo = z_at_p[klo], zhi = z_at_p[klo + 1];
    const real ln_plo = rlog(pbbc[klo + 1] + ppbc[klo + 1]);
    const real ln_phi = rlog(pbbc[klo] + ppbc[klo]);
    real ln_p;
    if (h <= zlo) ln_p = ln_phi;
    else if (h >= zhi) ln_p = ln_plo;
    else ln_p = ln_plo + (ln_phi - ln_plo) * ((zhi - h) / (zhi - zlo));
    return rexp(ln_p);
  }

  CPPWRF_HD void operator()(int n, int) const {
    real* errf = o.errf + ObsNudging::kErrf * n - 1;      // errf[kn], kn = 1 ... 10
    real* varobs = o.varobs + ObsNudging::kNdgv * n - 1;  // varobs[kn], kn = 1 ... 6
    real& rko = o.rko[n];
    const real rio = o.rio[n], rjo = o.rjo[n];
    const real dummy = real(99999.);
    const real gridx_t = real(0.5), gridy_t = real(0.5);
    for (int kn = 1; kn <= ObsNudging::kErrf; ++kn) errf[kn] = real(0.0);
    int iobmg = 0, jobmg = 0;
    real dxobmg = 0, dyobmg = 0;
    for (int ityp = 1; ityp <= 3; ++ityp) {
      real gridx, gridy;
      if (ityp == 1) {
        gridx = real(0.0);
        gridy = real(0.5);
      } else if (ityp == 2) {
        gridx = real(0.5);
        gridy = real(0.0);
      } else {
        gridx = gridx_t;
        gridy = gridy_t;
      }
      real ra = rio - gridx, rb = rjo - gridy;
      int ia = int(ra), ib = int(rb);
      if (ityp == 1) {
        iobmg = std::min(std::max(1, int(rio - gridx_t)), c.ide - 1);
        jobmg = std::min(std::max(1, int(rjo - gridy_t)), c.jde - 1);
        dxobmg = rio - gridx_t - real(int(rio - gridx_t));
        dyobmg = rjo - gridy_t - real(int(rjo - gridy_t));
      }
      const int iob_ms = iobmg, job_ms = jobmg;
      const real dxob_ms = dxobmg, dyob_ms = dyobmg;
      real pob = real(0.0);
      if (std::fabs(rko + real(99)) < real(1.)) {
        real pbbo[kMaxObsLevels + 1], ppbo[kMaxObsLevels + 1];
        for (int k = 1; k <= c.k_end; ++k) {  // kds ... kde in WRF; the level kde is never read
          pbbo[k] = real(.001) * ((real(1.) - dyob_ms) * ((real(1.) - dxob_ms) * PB(iob_ms, k, job_ms) +
                                                          dxob_ms * PB(iob_ms + 1, k, job_ms)) +
                                  dyob_ms * ((real(1.) - dxob_ms) * PB(iob_ms, k, job_ms + 1) +
                                             dxob_ms * PB(iob_ms + 1, k, job_ms + 1)));
          ppbo[k] = real(.001) * ((real(1.) - dyob_ms) * ((real(1.) - dxob_ms) * PP(iob_ms, k, job_ms) +
                                                          dxob_ms * PP(iob_ms + 1, k, job_ms)) +
                                  dyob_ms * ((real(1.) - dxob_ms) * PP(iob_ms, k, job_ms + 1) +
                                             dxob_ms * PP(iob_ms + 1, k, job_ms + 1)));
        }
        pob = varobs[5];
        if (pob == real(-888888.)) {
          const real hob = varobs[6];
          if (hob > real(-800000.)) pob = ht_to_p(hob, ppbo, pbbo, iob_ms, job_ms, dxob_ms, dyob_ms);
        }
        if (pob > real(-800000.)) {
          int kbot = 1;
          for (int k = c.k_end - 1; k >= 1; --k) {
            kbot = k;
            if (pob <= pbbo[k] + ppbo[k]) break;
          }
          const real pphi = ppbo[kbot + 1], pbhi = pbbo[kbot + 1];
          rko = real(kbot + 1) - ((pob - pbhi - pphi) / (pbbo[kbot] + ppbo[kbot] - pbhi - pphi));
          rko = fmaxr(rko, real(1.0));
        }
      }
      if (varobs[5] == real(-888888.) && varobs[6] > real(-800000.)) varobs[5] = pob;  // wrf_dm_sum_real(pob)
      real rc = rko;
      const int nlb = ityp == 1 ? 1 : ityp == 2 ? 2 : 3, nle = ityp == 1 ? 1 : ityp == 2 ? 2 : 5;
      for (int ivar = nlb; ivar <= nle; ++ivar) {
        if ((ra - real(1.)) < real(0)) errf[ivar] = errf[ivar] + dummy;
        if ((rb - real(1.)) < real(0)) errf[ivar] = errf[ivar] + dummy;
        if ((real(c.ide) - real(2.0) * gridx - ra - real(1.e-10)) < real(0)) errf[ivar] = errf[ivar] + dummy;
        if ((real(c.jde) - real(2.0) * gridy - rb - real(1.e-10)) < real(0)) errf[ivar] = errf[ivar] + dummy;
        if (rc < real(1.)) errf[ivar] = errf[ivar] + dummy;
      }
      ia = int(ra);
      ib = int(rb);
      const int ic = int(rc);
      ra = ra - real(ia);
      rb = rb - real(ib);
      rc = rc - real(ic);
      // an observation without a vertical position (no pressure and no height: rko = -99) has errf + dummy from
      // rc < 1 and is never nudged; WRF reads its innovations below the column (k = ic < 1), here the lowest level
      const int kc = ic < 1 ? 1 : ic;
      if (ityp == 1 && c.iswind == 1) {
        int iob = std::max(2, ia);
        iob = std::min(iob, c.ide - 1);
        const int iobm = std::max(1, iob - 1), iobp = std::min(c.ide - 1, iob + 1);
        int job = std::max(2, ib);
        job = std::min(job, c.jde - 1);
        const int jobp = std::min(c.jde - 1, job + 1);
        const int kob = std::min(c.k_end, kc);
        const int kobp = std::min(kob + 1, c.k_end);
        const real dxob = ra, dyob = rb, dzob = rc;
        const real pug1 = real(.5) * (PB(iobm, 1, job) + PB(iob, 1, job));
        const real pug2 = real(.5) * (PB(iob, 1, job) + PB(iobp, 1, job));
        real uratiob;
        if (rko == real(1.0))
          uratiob = ((real(1.) - dyob) * ((real(1.) - dxob) * uratio(iob, job) + dxob * uratio(iobp, job)) +
                     dyob * ((real(1.) - dxob) * uratio(iob, jobp) + dxob * uratio(iobp, jobp)));
        else
          uratiob = real(1.);
        if (std::fabs(uratiob) < real(1.0e-3)) uratiob = real(1.);
        auto f = [this](int I, int K, int J) { return U(I, K, J); };
        errf[1] = errf[1] + uratiob * varobs[1] - tri(f, iob, job, kob, kobp, dxob, dyob, dzob);
        errf[7] = real(.001) * ((real(1.) - dxob) * pug1 + dxob * pug2);
      }
      if (ityp == 2 && c.iswind == 1) {
        int iob = std::max(2, ia);
        iob = std::min(iob, c.ide - 1);
        const int iobp = std::min(c.ide - 1, iob + 1);
        int job = std::max(2, ib);
        job = std::min(job, c.jde - 1);
        const int jobm = std::max(1, job - 1), jobp = std::min(c.jde - 1, job + 1);
        const int kob = std::min(c.k_end, kc);
        const int kobp = std::min(kob + 1, c.k_end);
        const real dxob = ra, dyob = rb, dzob = rc;
        const real pvg1 = real(.5) * (PB(iob, 1, jobm) + PB(iob, 1, job));
        const real pvg2 = real(.5) * (PB(iob, 1, job) + PB(iob, 1, jobp));
        real vratiob;
        if (rko == real(1.0))
          vratiob = ((real(1.) - dyob) * ((real(1.) - dxob) * vratio(iob, job) + dxob * vratio(iobp, job)) +
                     dyob * ((real(1.) - dxob) * vratio(iob, jobp) + dxob * vratio(iobp, jobp)));
        else
          vratiob = real(1.);
        if (std::fabs(vratiob) < real(1.0e-3)) vratiob = real(1.);
        auto f = [this](int I, int K, int J) { return Vv(I, K, J); };
        errf[2] = errf[2] + vratiob * varobs[2] - tri(f, iob, job, kob, kobp, dxob, dyob, dzob);
        errf[8] = real(.001) * ((real(1.) - dyob) * pvg1 + dyob * pvg2);
      }
      if (ityp == 3) {
        int iob = std::max(1, ia);
        iob = std::min(iob, c.ide - 1);
        int job = std::max(1, ib);
        job = std::min(job, c.jde - 1);
        if (c.istemp == 1 || c.ismois == 1) {
          const int kob = std::min(c.k_end, kc);
          const int kobp = std::min(kob + 1, c.k_end);
          const real dxob = ra, dyob = rb, dzob = rc;
          real tratxob;
          if (rko == real(1.0))
            tratxob = ((real(1.) - dyob) * ((real(1.) - dxob) * tratx(iob - 1, job - 1) + dxob * tratx(iob, job - 1)) +
                       dyob * ((real(1.) - dxob) * tratx(iob - 1, job) + dxob * tratx(iob, job)));
          else
            tratxob = real(1.);
          if (std::fabs(tratxob) < real(1.0E-3)) tratxob = real(1.);
          real obs_pottemp = real(-888888.);
          if (varobs[3] > real(-800000.) && varobs[5] > real(-800000))
            obs_pottemp = varobs[3] * rpow(real(100.) / varobs[5], c.rcp) - c.t0;
          auto ft = [this](int I, int K, int J) { return T(I, K, J); };
          auto fq = [this](int I, int K, int J) { return Q(I, K, J); };
          errf[3] = errf[3] + tratxob * obs_pottemp - tri(ft, iob, job, kob, kobp, dxob, dyob, dzob);
          errf[4] = errf[4] + varobs[4] - tri(fq, iob, job, kob, kobp, dxob, dyob, dzob);
          errf[10] = errf[10] + tri(fq, iob, job, kob, kobp, dxob, dyob, dzob);
          errf[6] = real(.001) * ((real(1.) - dyob) * ((real(1.) - dxob) * PB(iob, 1, job) + dxob * PB(iob + 1, 1, job)) +
                                  dyob * ((real(1.) - dxob) * PB(iob, 1, job + 1) + dxob * PB(iob + 1, 1, job + 1)));
        }
        errf[5] = real(kpbl(iob + int(lround_x86(ra)) - 1, job + int(lround_x86(rb)) - 1));
      }
    }
    // the gross error check of the temperature (module_fddaobs_rtfdda.F:1377-1390)
    if ((std::fabs(errf[3]) > real(20.)) && (errf[3] > real(-800000.))) {
      errf[1] = real(-888888.);
      errf[2] = real(-888888.);
      errf[3] = real(-888888.);
      errf[4] = real(-888888.);
      varobs[1] = real(-888888.);
      varobs[2] = real(-888888.);
      varobs[3] = real(-888888.);
      varobs[4] = real(-888888.);
    }
  }
};

struct NudobScalars {
  int ivar, inest, ide, jde, kte, icut, nstat;
  int recompute;       // MOD(KTAU, INFR) == 0 .OR. (IFREST .AND. KTAU == KTAUR)
  int nudge_pbl, sfc_scheme_horiz, sfc_scheme_vert, scl_neg_qv_innov;
  int isw;             // ISWIND / ISTEMP / ISMOIS of the variable
  real xtime, dtmin, twindo, sfcfact, sfcfacr, rinfmn, rinfmx, pfree, dcon, tfaci, maxsnd_gap;
  real rindx, gfactor, coef;  // RINDX, GFACTOR, GIV / GITQ
};

// nudob for one column I of the row J of one variable (module_fddaobs_rtfdda.F:1719-2744)
struct KNudob {
  NudobScalars c;
  ObsDevArrays o;
  B3 pbase;
  C3 pp, qvb, zslab;
  C2 vih1, vih2, mu;
  B2 terrh, msf;  // msf: msfuy (U), msfvx (V)
  IC2 kpblt;
  C1 c1h, c2h;
  V3 aten, savwt;

  CPPWRF_HD real VAR(int kn, int n) const { return o.varobs[(kn - 1) + ObsNudging::kNdgv * (n - 1)]; }
  CPPWRF_HD real ERRF(int kn, int n) const { return o.errf[(kn - 1) + ObsNudging::kErrf * (n - 1)]; }

  // the ERFIVR of nudob's surface branch after the column I (k loop) with obs_scl_neg_qv_innov: WRF scales it in
  // place, so that a column sees the factors of the columns before it (module_fddaobs_rtfdda.F:2015-2040, 2092-2119)
  CPPWRF_HD real qv_scale(int i, int j, int k, real erfivr, real qvb_at_ob_loc) const {
    const real qvb_at_cur_loc = fmaxr(qvb(i, j, k), real(0.0));
    if ((erfivr < real(0)) && (qvb_at_cur_loc < qvb_at_ob_loc)) {
      const real scale = fminr(real(1.0), std::fabs(qvb_at_cur_loc / erfivr));
      erfivr = erfivr * scale;
    }
    return erfivr;
  }

  // the surface branch's vertical loop at column i: adds to wt / wt2err (add) and returns ERFIVR after it
  CPPWRF_HD real sfc_column(int i, int j, real timewt, real wtij, real erfivr, real qvb_at_ob_loc, bool add, real* wt,
                            real* wt2err) const {
    const bool scale = c.ivar == 4 && c.scl_neg_qv_innov > 0;
    if (c.sfc_scheme_vert == 1) {
      int komax = std::max(3, kpblt(i, j));
      if (kpblt(i, j) > 25) komax = 3;
      const int komin = 1;
      const real dk = real(komax);
      for (int k = komin; k <= komax; ++k) {
        const real wtsig = real(komax - k + 1) / dk;
        if (add) wt[k] = wt[k] + timewt * wtsig * wtij;
        if (scale) erfivr = qv_scale(i, j, k - 1, erfivr, qvb_at_ob_loc);
        if (add) wt2err[k] = wt2err[k] + timewt * timewt * wtij * wtij * wtsig * wtsig * erfivr;
      }
    } else {
      const real dz_ramp = real(1.0) / fmaxr(real(1.0), vih2(i, j) - vih1(i, j));
      for (int k = 1; k <= c.kte; ++k) {
        real wtsig = fminr(real(1.0), real(1.0) + (vih1(i, j) - zslab(i, j, k - 1) + terrh(i, j)) * dz_ramp);
        wtsig = fmaxr(real(0.0), wtsig);
        if (wtsig <= real(0.0)) break;
        if (add) wt[k] = wt[k] + timewt * wtsig * wtij;
        if (scale) erfivr = qv_scale(i, j, k - 1, erfivr, qvb_at_ob_loc);
        if (add) wt2err[k] = wt2err[k] + timewt * timewt * wtij * wtij * wtsig * wtsig * erfivr;
      }
    }
    return erfivr;
  }

  CPPWRF_HD void operator()(int i, int j) const {
    const int I = i + 1, J = j + 1, ivar = c.ivar;
    // fddaobs_driver: T and QV only for 2 < J < jde - 1 (module_fddaobs_driver.F:351); nudob's returns (:1733-1742)
    if (ivar >= 3 && !(J > 2 && J < c.jde - 1)) return;
    if (c.inest == 1 && ivar < 3 && (J <= 2 || J >= c.jde - 1)) return;
    if (c.inest == 1 && ivar >= 3 && (J <= 2 || J >= c.jde - 2)) return;
    const int its = 1, ite = c.ide - 1;  // one tile (the driver passes min(ite, ide - 1))
    const int i_s = std::max(2 + c.icut, its), i_e = std::min(c.ide - 1 - c.icut, ite);
    const int kte = c.kte;
    real wt[kMaxObsLevels + 1], wt2err[kMaxObsLevels + 1];
    if (c.recompute) {
      real gridx = real(0.0), gridy = real(0.0);
      int igrid = 0;
      if (ivar >= 3) {
        gridx = real(0.5);
        gridy = real(0.5);
        igrid = 1;
      } else if (ivar == 1) {
        gridy = real(0.5);
        gridx = real(0.0);
        igrid = 1;
      } else if (ivar == 2) {
        gridx = real(0.5);
        gridy = real(0.0);
        igrid = 1;
      }
      const real rindx = c.rindx;
      for (int k = 1; k <= kte; ++k) {
        wt[k] = real(0.0);
        wt2err[k] = real(0.0);
      }
      const int maxj = J + int(rindx * c.rinfmx + real(0.99));
      const int minj = J - int(rindx * c.rinfmx + real(0.99));
      int n = 1;
      for (int nnn = 1; nnn <= c.nstat; ++nnn) {
        int nsndlev = int(o.nlevs_ob[n - 1] - o.lev_in_ob[n - 1]) + 1;
        const int njcsnd = nsndlev;
        const real pob = VAR(5, n);
        const real ra = o.rio[n - 1] - gridx, rb = o.rjo[n - 1] - gridy;
        if (std::fabs(ERRF(ivar, n)) > real(9.E4) && njcsnd == 1) {
        } else if (rb < real(minj) || rb > real(maxj)) {
        } else {
          int kob = int(lround_x86(o.rko[n - 1] + real(0.45)));
          kob = std::min(kte, kob);
          kob = std::max(1, kob);
          const real psurf = ivar >= 3 ? ERRF(6, n) : ivar == 1 ? ERRF(7, n) : ERRF(8, n);
          if (kob == 1 && ivar <= 4 && o.nlevs_ob[n - 1] < real(1.5)) {
            // the surface branch (module_fddaobs_rtfdda.F:1903-2131)
            const real timewt = get_timewt(c.xtime, c.dtmin, c.twindo, c.sfcfact, o.timeob[n - 1]);
            int maxi = int(ra + real(0.99) + rindx * c.sfcfacr);
            maxi = std::min(c.ide - 1, maxi);
            int mini = int(ra - rindx * c.sfcfacr - real(0.99));
            mini = std::max(2, mini);
            if (ra >= (real(0.) - rindx * c.sfcfacr / real(3)) && ra <= real(c.ide) + rindx * c.sfcfacr / real(3) &&
                rb >= (real(0.) - rindx * c.sfcfacr / real(3)) && rb <= real(c.jde) + rindx * c.sfcfacr / real(3)) {
              const real rj = real(J);
              const real rx = rj - rb;
              real erfivr = ERRF(ivar, n);
              const real qvb_at_ob_loc = ERRF(10, n);
              const int lo = std::max(its, mini), hi = std::min(ite, maxi);
              if (c.nudge_pbl && I >= lo && I <= hi) {
                if (ivar == 4 && c.scl_neg_qv_innov > 0)  // the factors of the columns lo ... I - 1
                  for (int i2 = lo; i2 < I; ++i2)
                    erfivr = sfc_column(i2 - 1, j, 0, 0, erfivr, qvb_at_ob_loc, false, wt, wt2err);
                const real ri = real(I);
                const real ry = ri - ra;
                const real ris = rindx * rindx * c.sfcfacr * c.sfcfacr;
                const real rsq = rx * rx + ry * ry;
                real wtij;
                if (c.sfc_scheme_horiz == 1) {
                  const real dprim = std::sqrt(rsq);
                  const real d = dprim + rindx * c.dcon * std::fabs(psurf - real(.001) * pbase(i, j, 0));
                  const real dsq = d * d;
                  wtij = (ris - dsq) / (ris + dsq);
                  wtij = fmaxr(real(0.0), wtij);
                } else {
                  wtij = (ris - rsq) / (ris + rsq);
                  const real scratch = (std::fabs(psurf - real(.001) * pbase(i, j, 0)) * c.dcon);
                  const real pdfac = real(1.) - fminr(real(1.0), scratch);
                  wtij = wtij * pdfac;
                  wtij = fmaxr(real(0.0), wtij);
                }
                sfc_column(i, j, timewt, wtij, erfivr, qvb_at_ob_loc, true, wt, wt2err);
              }
            }
          } else {
            // the upper-air branch (module_fddaobs_rtfdda.F:2134-2558)
            const real timewt = get_timewt(c.xtime, c.dtmin, c.twindo, real(1.), o.timeob[n - 1]);
            const real slope0 = (c.rinfmn - c.rinfmx) / (psurf - c.pfree);
            real rinfac = slope0 * pob + c.rinfmx - slope0 * c.pfree;
            rinfac = fmaxr(rinfac, c.rinfmn);
            rinfac = fminr(rinfac, c.rinfmx);
            if (nsndlev > 1) rinfac = c.rinfmx;
            int maxi = int(ra + real(0.99) + rindx * rinfac);
            maxi = std::min(c.ide - igrid, maxi);
            int mini = int(ra - rindx * rinfac - real(0.99));
            mini = std::max(1, mini);
            if (ra >= (real(0.) - rinfac * rindx / real(3)) && ra <= real(c.ide) + rinfac * rindx / real(3) &&
                rb >= (real(0.) - rinfac * rindx / real(3)) && rb <= real(c.jde) + rinfac * rindx / real(3)) {
              const real rj = real(J);
              const real rx = rj - rb;
              const real erfivr = ERRF(ivar, n);
              const real qvb_at_ob_loc = ERRF(10, n);
              nsndlev = int(o.nlevs_ob[n - 1] - o.lev_in_ob[n - 1]) + 1;
              if (I >= std::max(its, mini) && I <= std::min(ite, maxi)) {
                const real ri = real(I);
                const real ry = ri - ra;
                const real ris = rindx * rinfac * rindx * rinfac;
                const real rsq = rx * rx + ry * ry;
                real wtij = (ris - rsq) / (ris + rsq);
                wtij = fmaxr(real(0.0), wtij);
                const real rinprs = nsndlev == 1 ? real(7.5) : real(3.0);
                if (nsndlev == 1) {
                  int komin = 1, komax = kte;
                  for (int k = kte; k >= 1; --k) {
                    const real pijk = real(.001) * (pbase(i, j, k - 1) + pp(i, j, k - 1));
                    if (pijk >= (pob + rinprs)) {
                      komin = k;
                      break;
                    }
                    if (k == 1) komin = 1;
                  }
                  for (int k = 3; k <= kte; ++k) {
                    const real pijk = real(.001) * (pbase(i, j, k - 1) + pp(i, j, k - 1));
                    if (pijk <= (pob - rinprs)) {
                      komax = k;
                      break;
                    }
                    if (k == kte) komax = kte;
                  }
                  // WRF reads its never set kpbl_obs(n) here (serial build)
                  if ((kpblt(i, j) <= komax) && (kpblt(i, j) >= komin)) raise_fatal(kFatalObsKpblObs);
                  if (c.nudge_pbl || komin >= kpblt(i, j)) {
                    for (int k = komin; k <= komax; ++k) {
                      const real pijk = real(.001) * (pbase(i, j, k - 1) + pp(i, j, k - 1));
                      real reserf = erfivr;
                      real wtsig = real(1.) - std::fabs(pijk - pob) / rinprs;
                      wtsig = fmaxr(wtsig, real(0.0));
                      wt[k] = wt[k] + timewt * wtij * wtsig;
                      if ((ivar == 4) && (c.scl_neg_qv_innov > 0)) {
                        const real qvb_at_cur_loc = fmaxr(qvb(i, j, k - 1), real(0.0));
                        if ((erfivr < real(0)) && (qvb_at_cur_loc < qvb_at_ob_loc))
                          reserf = reserf * fminr(real(1.0), std::fabs(qvb_at_cur_loc / erfivr));
                      }
                      wt2err[k] = wt2err[k] + timewt * timewt * wtij * wtij * reserf * wtsig * wtsig;
                    }
                  }
                } else {
                  // a multi-level sounding (module_fddaobs_rtfdda.F:2348-2547)
                  const int nlast = n + nsndlev - 1;
                  if (o.nlevs_ob[nlast - 1] != o.lev_in_ob[nlast - 1]) raise_fatal(kFatalObsSoundingLevels);
                  const int komin = 1, komax = kte - 2;
                  for (int k = komax; k >= komin; --k) {
                    real reserf = real(0.0), wtsig = real(0.0);
                    const real pijk = real(.001) * (pbase(i, j, k - 1) + pp(i, j, k - 1));
                    do {  // goto 1501: break
                      if (pijk > VAR(5, n)) break;
                      if (pijk <= VAR(5, nlast)) break;
                      const real slope = (c.rinfmn - c.rinfmx) / (real(.001) * pbase(i, j, 0) - c.pfree);
                      rinfac = slope * pijk + c.rinfmx - slope * c.pfree;
                      rinfac = fmaxr(rinfac, c.rinfmn);
                      rinfac = fminr(rinfac, c.rinfmx);
                      const real ris2 = rindx * rinfac * rindx * rinfac;
                      wtij = (ris2 - rsq) / (ris2 + rsq);
                      wtij = fmaxr(real(0.0), wtij);
                      real abs_pdiff_above = c.maxsnd_gap + real(1.0);
                      real abs_pdiff_below = c.maxsnd_gap + real(1.0);
                      real pobhi = real(-888888.), poblo = real(-888888.);
                      int nhi = 0, nlo = 0;
                      bool found = false;
                      for (int nn = 2; nn <= nsndlev; ++nn) {
                        pobhi = real(-888888.);
                        if (VAR(ivar, n + nn - 1) > real(-800000.) && VAR(5, n + nn - 1) > real(-800000.)) {
                          pobhi = VAR(5, n + nn - 1);
                          nhi = n + nn - 1;
                          abs_pdiff_above = std::fabs(pobhi - pijk);
                          if (pobhi <= pijk && abs_pdiff_above <= c.maxsnd_gap) {
                            found = true;
                            break;
                          }
                        }
                      }
                      if (!found) break;
                      nlo = nhi - 1;
                      found = false;
                      for (int nnjc = nhi - 1; nnjc >= n; --nnjc) {
                        if (VAR(ivar, nnjc) > real(-800000.) && VAR(5, nnjc) > real(-800000.)) {
                          poblo = VAR(5, nnjc);
                          nlo = nnjc;
                          abs_pdiff_below = std::fabs(poblo - pijk);
                          if (poblo >= pijk && abs_pdiff_below <= c.maxsnd_gap) {
                            found = true;
                            break;
                          }
                        }
                      }
                      if (!found) break;
                      if ((abs_pdiff_below + abs_pdiff_above) > c.maxsnd_gap) break;
                      real pdiffj;
                      if (std::fabs(pobhi - poblo) < real(0.000001)) pdiffj = real(0);
                      else pdiffj = rlog(pijk / poblo) / rlog(pobhi / poblo);
                      reserf = ERRF(ivar, nlo) + (ERRF(ivar, nhi) - ERRF(ivar, nlo)) * pdiffj;
                      if ((ivar == 4) && (c.scl_neg_qv_innov > 0)) {
                        const real qvb_at_cur_loc = qvb(i, j, k - 1);
                        const real qob = ERRF(10, nlo) + (ERRF(10, nhi) - ERRF(10, nlo)) * pdiffj;
                        if ((reserf < real(0)) && (qvb_at_cur_loc < qob))
                          reserf = reserf * fminr(real(1.0), std::fabs(qvb_at_cur_loc / reserf));
                      }
                      wtsig = real(1.);
                    } while (false);
                    // 1501
                    if (c.nudge_pbl || k > kpblt(i, j)) {
                      wt[k] = wt[k] + timewt * wtij * wtsig;
                      wt2err[k] = wt2err[k] + timewt * timewt * wtij * wtij * reserf * wtsig * wtsig;
                    }
                  }
                }
              }
            }
          }
        }
        n = n + njcsnd;
        if (n > c.nstat) break;
      }
      for (int k = 1; k <= kte; ++k) {
        if (wt[k] == real(0)) wt2err[k] = real(0.0);
        if (wt[k] == real(0)) wt[k] = real(1.0);
      }
    }
    // the tendencies (module_fddaobs_rtfdda.F:2596-2743)
    if (I >= i_s && I <= i_e) {
      for (int k = 1; k <= kte; ++k) {
        const real w2eowt = c.recompute ? wt2err[k] / wt[k] : savwt(i, j, k - 1);
        if (ivar < 3) {
          const real rscale = mu(i, j) / msf(i, j);
          aten(i, j, k - 1) = aten(i, j, k - 1) + c.coef * rscale * w2eowt * c.tfaci * real(c.isw) * c.gfactor;
        } else {
          aten(i, j, k - 1) = aten(i, j, k - 1) +
                              c.coef * (c1h(k - 1) * mu(i, j) + c2h(k - 1)) * w2eowt * c.tfaci * real(c.isw) * c.gfactor;
        }
      }
    }
    if (c.recompute)
      for (int k = 1; k <= kte; ++k) savwt(i, j, k - 1) = wt2err[k] / wt[k];
  }
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------

ObsNudging::ObsNudging(const Grid& g, const ObsNudgeOptions& o) : g_(g), o_(o) {
  if (g.nz > kMaxObsLevels) throw std::runtime_error("obs nudging: more than 128 levels are not supported");
  const std::size_t n = std::size_t(std::max(o.max_obs, 0));
  varobs.assign(n * kNdgv, real(0));
  errf.assign(n * kErrf, real(0));
  for (auto* v : {&timeob, &nlevs_ob, &lev_in_ob, &plfo, &elevob, &rio, &rjo, &rko}) v->assign(n, real(0));
  for (int v = 0; v < 6; ++v) vih[v] = Field2(g, Stagger::M, "obs_vih");
  if (o.diff_z) zdiff = Field3(g, Stagger::M, "obs_z");
  for (int v = 0; v < 4; ++v) savwt[v] = Field3(g, Stagger::M, "obs_savwt");
}

ObsNudging::~ObsNudging() {
  for (real* p : {d.varobs, d.errf, d.timeob, d.nlevs_ob, d.lev_in_ob, d.rio, d.rjo, d.rko})
    if (p) dev_free(p);
}

void ObsNudging::upload(int nsta) {
  if (nsta > dcap) {
    for (real* p : {d.varobs, d.errf, d.timeob, d.nlevs_ob, d.lev_in_ob, d.rio, d.rjo, d.rko})
      if (p) dev_free(p);
    dcap = std::max(nsta, 2 * dcap);
    auto al = [&](int per) { return static_cast<real*>(dev_alloc(std::size_t(dcap) * per * sizeof(real))); };
    d.varobs = al(kNdgv);
    d.errf = al(kErrf);
    d.timeob = al(1);
    d.nlevs_ob = al(1);
    d.lev_in_ob = al(1);
    d.rio = al(1);
    d.rjo = al(1);
    d.rko = al(1);
  }
  const std::size_t b = std::size_t(nsta) * sizeof(real);
  copy_to_dev(d.varobs, varobs.data(), b * kNdgv);
  copy_to_dev(d.errf, errf.data(), b * kErrf);
  copy_to_dev(d.timeob, timeob.data(), b);
  copy_to_dev(d.nlevs_ob, nlevs_ob.data(), b);
  copy_to_dev(d.lev_in_ob, lev_in_ob.data(), b);
  copy_to_dev(d.rio, rio.data(), b);
  copy_to_dev(d.rjo, rjo.data(), b);
  copy_to_dev(d.rko, rko.data(), b);
}

void ObsNudging::download(int nsta) {
  const std::size_t b = std::size_t(nsta) * sizeof(real);
  copy_to_host(varobs.data(), d.varobs, b * kNdgv);
  copy_to_host(errf.data(), d.errf, b * kErrf);
  copy_to_host(rko.data(), d.rko, b);
}

// fddaobs_init (module_fddaobs_rtfdda.F:173-511)
void ObsNudging::init(bool restart_in, long ktaur_in, const std::string& start_date, double p00, double t00, double tlp,
                      const std::vector<double>& znu, double p_top, double xlat11, double xlong11, int map_proj,
                      double truelat1, double truelat2, double stand_lon) {
  const ObsNudgeOptions& o = o_;
  restart = restart_in;
  ktaur = ktaur_in;
  sdate = start_date.substr(0, 19);
  iwtsig = 0;
  datend = real(0.);
  ieodi = 0;
  if (o.idynin == 1) {
    if (real(o.dtramp) > real(0.)) datend = real(o.fdda_end) - real(o.dtramp);
    else datend = real(o.fdda_end);
  }
  if (o.sfc_scheme_horiz != 0 && o.sfc_scheme_horiz != 1)
    obs_fatal("fddaobs_init: module_fddaobs_rtfdda STOP (obs_sfc_scheme_horiz = " + std::to_string(o.sfc_scheme_horiz) +
              ": 0 or 1)");
  if (o.sfc_scheme_vert != 0 && o.sfc_scheme_vert != 1)
    obs_fatal("fddaobs_init: module_fddaobs_rtfdda STOP (obs_sfc_scheme_vert = " + std::to_string(o.sfc_scheme_vert) +
              ": 0 or 1)");
  window = real(o.twindo);
  if (o.grid_id != 1 && real(o.twindo) == real(0.)) window = real(o.twindo_cg);
  if (real(o.dpsmx) > real(0.)) {
    dpsmx = real(o.dpsmx);
    dcon = real(1.0) / dpsmx;
  } else {
    obs_fatal("fddaobs_init: Namelist variable dpsmx must be greater than zero!");
  }
  // get_base_state_height_column (k = kts ... kte: the vectorised log of WRF's build)
  {
    const int nz = g_.nz;
    const real r_d = real(cst::r_d), g = real(cst::g), ginv = real(1) / g, p00r = real(p00), ptop = real(p_top);
    const real a = real(tlp), t00r = real(t00);
    base_state.assign(std::size_t(nz), real(0));
    std::vector<real> pb(static_cast<std::size_t>(nz));
    for (int k = 0; k < nz; ++k) pb[k] = real(znu[k]) * (p00r - ptop) + ptop;
    for (int k = 0; k < nz; ++k) {
      const real logterm = vm::log(pb[k] / p00r, k, nz);
      base_state[k] = real(.5) * r_d * a * ginv * logterm * logterm - r_d * t00r * ginv * logterm;
    }
  }
  nudge_uv_pbl = o.no_pbl_nudge_uv != 1;
  nudge_t_pbl = o.no_pbl_nudge_t != 1;
  nudge_q_pbl = o.no_pbl_nudge_q != 1;
  if (o.sfc_scheme_vert == 0) {
    for (int v = 0; v < 6; ++v) {
      vif_uv[v] = real(o.vif_uv[v]);
      vif_t[v] = real(o.vif_t[v]);
      vif_q[v] = real(o.vif_q[v]);
    }
    const real zmax = real(o.nudgezmax), fullmin = real(o.nudgezfullmin), rampmin = real(o.nudgezrampmin);
    if (zmax <= real(0.)) obs_fatal("fddaobs_init: STOP on bad obs_nudgemax value");
    if (fullmin < real(0.)) obs_fatal("fddaobs_init: STOP on bad obs_nudgefullmin value");
    if (rampmin < real(0.)) obs_fatal("fddaobs_init: STOP on bad obs_nudgerampmin value");
    if (zmax < fullmin + rampmin) obs_fatal("fddaobs_init: STOP on inconsistent namelist values");
    vif_fullmin = fullmin;
    vif_rampmin = rampmin;
    vif_max = zmax;
    if (fullmin > real(0.0))
      if (fullmin < real(1.1) * base_state[0]) vif_fullmin = real(1.1) * base_state[0];
  }
  pfree = real(50.0);
  rinfmn = real(1.0);
  rinfmx = real(2.0);
  known_lat = real(xlat11);
  known_lon = real(xlong11);
  // set_projection (wrf_fddaobs_in.F:1582-1604): map_init + map_set(PROJ_LC, ...) of module_llxy.F
  if (map_proj != 1)
    obs_fatal("obs nudging: map_proj = " + std::to_string(map_proj) + " is not supported (Lambert conformal, 1, only)");
  {
    ObsProj& p = proj_;
    const real rad_per_deg = real(3.141592653589793) / real(180.);
    auto wrap = [](real x) {  // map_set's longitude normalisation
      if (std::fabs(x) > real(180.)) {
        int iter = 0;
        while (std::fabs(x) > real(180.) && iter < 10) {
          if (x < real(-180.)) x = x + real(360.);
          if (x > real(180.)) x = x - real(360.);
          ++iter;
        }
      }
      return x;
    };
    p.truelat1 = real(truelat1);
    p.truelat2 = real(truelat2);
    p.stdlon = wrap(real(stand_lon));
    p.lat1 = known_lat;
    p.lon1 = wrap(known_lon);
    p.knowni = real(1.);
    p.knownj = real(1.);
    p.dx = real(g_.dx);
    p.hemi = p.truelat1 < real(0.) ? real(-1.0) : real(1.0);
    p.rebydx = real(6370000.) / p.dx;
    if (std::fabs(p.truelat2) > real(90.)) p.truelat2 = p.truelat1;
    // lc_cone
    if (std::fabs(p.truelat1 - p.truelat2) > real(0.1)) {
      p.cone = libm::log10(libm::cos(p.truelat1 * rad_per_deg)) - libm::log10(libm::cos(p.truelat2 * rad_per_deg));
      p.cone = p.cone / (libm::log10(libm::tan((real(45.0) - std::fabs(p.truelat1) / real(2.0)) * rad_per_deg)) -
                         libm::log10(libm::tan((real(45.0) - std::fabs(p.truelat2) / real(2.0)) * rad_per_deg)));
    } else {
      p.cone = libm::sin(std::fabs(p.truelat1) * rad_per_deg);
    }
    // set_lc
    real deltalon1 = p.lon1 - p.stdlon;
    if (deltalon1 > real(+180.)) deltalon1 = deltalon1 - real(360.);
    if (deltalon1 < real(-180.)) deltalon1 = deltalon1 + real(360.);
    const real tl1r = p.truelat1 * rad_per_deg;
    const real ctl1r = libm::cos(tl1r);
    p.rsw = p.rebydx * ctl1r / p.cone *
            rpow(libm::tan((real(90.) * p.hemi - p.lat1) * rad_per_deg / real(2.)) /
                     libm::tan((real(90.) * p.hemi - p.truelat1) * rad_per_deg / real(2.)),
                 p.cone);
    const real arg = p.cone * (deltalon1 * rad_per_deg);
    p.polei = p.hemi * p.knowni - p.hemi * p.rsw * libm::sin(arg);
    p.polej = p.hemi * p.knownj + p.rsw * libm::cos(arg);
  }
  initialised = true;
}

// llij_lc (module_llxy.F)
void ObsNudging::latlon_to_ij(real lat, real lon, real& i, real& j) const {
  const ObsProj& p = proj_;
  const real rad_per_deg = real(3.141592653589793) / real(180.);
  real deltalon = lon - p.stdlon;
  if (deltalon > real(+180.)) deltalon = deltalon - real(360.);
  if (deltalon < real(-180.)) deltalon = deltalon + real(360.);
  const real tl1r = p.truelat1 * rad_per_deg;
  const real ctl1r = libm::cos(tl1r);
  const real rm = p.rebydx * ctl1r / p.cone *
                  rpow(libm::tan((real(90.) * p.hemi - lat) * rad_per_deg / real(2.)) /
                           libm::tan((real(90.) * p.hemi - p.truelat1) * rad_per_deg / real(2.)),
                       p.cone);
  const real arg = p.cone * (deltalon * rad_per_deg);
  i = p.polei + p.hemi * rm * libm::sin(arg);
  j = p.polej - rm * libm::cos(arg);
  i = p.hemi * i;
  j = p.hemi * j;
}

// rotate_vector (wrf_fddaobs_in.F:1902-1946), PROJ_LC
void ObsNudging::rotate_vector(real lon, real& u, real& v) const {
  const real rad_per_deg = real(3.141592653589793) / real(180.);
  real diff = proj_.stdlon - lon;
  if (diff > real(180.)) diff = diff - real(360.);
  else if (diff < real(-180.)) diff = diff + real(360.);
  const real alpha = diff * proj_.cone * rad_per_deg * proj_.hemi;
  const real udbl = v * libm::sin(alpha) + u * libm::cos(alpha);
  const real vdbl = v * libm::cos(alpha) - u * libm::sin(alpha);
  u = udbl;
  v = vdbl;
}

namespace {
// rh2r (wrf_fddaobs_in.F:1459-1501), iice = 0
real rh2r(real rh, real t, real p) {
  const real eps = real(0.62197), e0 = real(6.1078), eslcon1 = real(17.2693882), eslcon2 = real(35.86);
  const real rh1 = rh * real(.01);
  const real esat = e0 * libm::exp(eslcon1 * (t - real(273.16)) / (t - eslcon2));
  const real rsat = eps * esat / (p - esat);
  return rh1 * rsat;
}
}  // namespace

// in4dob (wrf_fddaobs_in.F:137-1375) with wrf_fddaobs_in's condition (:72-73)
void ObsNudging::in4dob(long ktau, real xtime) {
  const ObsNudgeOptions& o = o_;
  const long krest = ktaur;
  const int infreq = o.ionf * powi(o.parent_grid_ratio, o.levidn);
  if (!(((ktau > krest && ktau % infreq == 0) || ktau == krest) && xtime <= real(o.fdda_end))) return;
  const int niobf = std::max(o.max_obs, 0);
  const int e_we = g_.nx + 1, e_sn = g_.ny + 1;
  const real rovg = real(cst::r_d) / real(cst::g);
  // 1-based accessors of the fdob arrays (WRF's indices)
  auto VAR = [&](int kn, int n) -> real& { return varobs[std::size_t(kn - 1) + std::size_t(kNdgv) * (n - 1)]; };
  auto TOB = [&](int n) -> real& { return timeob[n - 1]; };
  auto shift_or_fill = [&](int ndum, bool full) {  // the shifts of the removed old observations (:371-404, :1216-1243)
    const int nmove = niobf - ndum;
    if (nmove > 0 && ndum != 0) {
      for (int n = 1; n <= nmove; ++n) {
        for (int kn = 1; kn <= kNdgv; ++kn) VAR(kn, n) = VAR(kn, n + ndum);
        rjo[n - 1] = rjo[n + ndum - 1];
        rio[n - 1] = rio[n + ndum - 1];
        rko[n - 1] = rko[n + ndum - 1];
        TOB(n) = TOB(n + ndum);
        nlevs_ob[n - 1] = nlevs_ob[n + ndum - 1];
        lev_in_ob[n - 1] = lev_in_ob[n + ndum - 1];
        plfo[n - 1] = plfo[n + ndum - 1];
        if (full) elevob[n - 1] = elevob[n + ndum - 1];
      }
    }
    const int nopen = nmove + 1;
    if (nopen <= niobf) {
      for (int n = nopen; n <= niobf; ++n) {
        for (int kn = 1; kn <= kNdgv; ++kn) VAR(kn, n) = real(99999.);
        rio[n - 1] = real(99999.);
        rjo[n - 1] = real(99999.);
        rko[n - 1] = real(99999.);
        TOB(n) = real(99999.);
        if (full) {
          nlevs_ob[n - 1] = real(99999.);
          lev_in_ob[n - 1] = real(99999.);
          plfo[n - 1] = real(99999.);
          elevob[n - 1] = real(99999.);
        }
      }
    }
  };
  auto old_obs = [&](int nsta) {  // the observations now before the window (:353-361, :1197-1207)
    int ndum = 0;
    for (int n = 1; n <= nsta + 1 && n <= niobf; ++n) {
      if ((TOB(n) - tback_) < real(0)) TOB(n) = real(99999.);
      if (TOB(n) < real(9.E4)) break;
      ndum = n;
    }
    return std::abs(ndum);
  };

  if (ktau == 0 || ktau == ktaur) {
    for (int n = 1; n <= niobf; ++n) TOB(n) = real(99999.);
    xtime_at_rest = xtime;
  }
  if (ktau == ktaur) nstat = 0;
  int nsta = nstat;
  real xhour = xtime / real(60.);
  xhour = fmaxr(xhour, real(0.0));
  tback_ = xhour - window;
  const real tforwd = xhour + window;
  if (nsta != 0) shift_or_fill(old_obs(nsta), true);

  int nlast = 0;
  for (int n = 1; n <= niobf; ++n) {
    if (TOB(n) > real(9.E4)) break;
    nlast = n;
  }
  if (ktau == 0 || ktau == ktaur) {
    rtlast = real(-999.);
    ifon = 1;
    const std::string name = o.file;
    if (!file_open) {
      file.open(name);
      if (!file) {  // no observation file: no nudging from this domain's observations (WRF returns here)
        std::printf("obs nudging: no %s, there are no fdda obs files to open\n", name.c_str());
        return;
      }
      file_open = true;
    }
  }

  // the read loop (:475-1151) as WRF's labels
  enum class L { L1001, L100, L110, L111, L130 };
  int N = nlast;
  L at = N == 0 ? L::L110 : L::L1001;
  std::string rec;
  auto getrec = [&](std::string& r) {
    if (!file_open || !std::getline(file, r)) return false;
    if (!r.empty() && r.back() == '\r') r.pop_back();
    return true;
  };
  auto need = [&](std::string& r) {
    if (!getrec(r)) obs_fatal("obs nudging: " + o.file + " ends inside an observation (WRF: end of file in READ)");
  };
  while (at != L::L130) {
    switch (at) {
      case L::L1001:
        at = ieof > 1 ? L::L130 : L::L100;
        break;
      case L::L100:
        if (N != 0 && TOB(N) > tforwd && TOB(N) < real(99999.)) {
          at = L::L130;
          break;
        }
        if (ieof == 1) {
          ieof = 2;
          at = L::L130;
          break;
        }
        at = L::L110;
        break;
      case L::L111:
        ieof = 1;
        if (file_open) file.close();
        file_open = false;
        at = L::L1001;
        break;
      case L::L110: {
        if (N > niobf - 1)
          obs_fatal("wrf_fddaobs_in: in4dob STOP 122 (more observations than max_obs = " + std::to_string(niobf) + ")");
        if (ieodi == 1 || !getrec(rec)) {
          at = L::L111;
          break;
        }
        const std::string date_char = field(rec, 1, 14);
        N = N + 1;
        // fmt_date + geth_idts
        const std::string obs_date = date_char.substr(0, 4) + "-" + date_char.substr(4, 2) + "-" + date_char.substr(6, 2) +
                                     "_" + date_char.substr(8, 2) + ":" + date_char.substr(10, 2) + ":" +
                                     date_char.substr(12, 2);
        long long idts = 0;
        try {
          idts = date_seconds(obs_date) - date_seconds(sdate);
        } catch (const std::exception&) {
          obs_fatal("obs nudging: bad date '" + date_char + "' in " + o.file);
        }
        idts = idts + lround_x86(xtime_at_rest * real(60.));
        real rtimob = real(idts) / real(3600.);
        TOB(N) = rtimob;
        if (o.idynin == 1 && TOB(N) * real(60.) > datend) {
          ieodi = 1;
          TOB(N) = real(99999.);
          rtimob = TOB(N);
        }
        need(rec);
        const real latitude = read_f(rec, 2, 9, 4), longitude = read_f(rec, 12, 9, 4);
        real ri, rj;
        latlon_to_ij(latitude, longitude, ri, rj);
        ri = ri + real(.5);
        rj = rj + real(.5);
        rio[N - 1] = ri;
        rjo[N - 1] = rj;
        need(rec);
        const std::string id = field(rec, 2, 40), namef = field(rec, 45, 40);
        need(rec);
        std::string platform = field(rec, 2, 16);
        const real elevation = read_f(rec, 38, 8, 0);
        const bool is_sound = read_l(rec, 48, 4);
        (void)read_l(rec, 54, 4);  // bogus
        const int meas_count = read_i(rec, 60, 5);
        elevob[N - 1] = elevation;
        if (namef.substr(1, 8) == "PROFILER") platform.replace(6, 8, "PROFILER");
        if (namef.substr(1, 5) == "ACARS") platform.replace(6, 5, "ACARS");
        if (namef.substr(0, 7) == "SATWNDS") platform.replace(0, 11, "SATWNDS    ");
        if (namef.substr(0, 8) == "CLASS DA") platform.replace(6, 4, "TEMP");
        rko[N - 1] = real(-99.);
        if (!is_sound) rko[N - 1] = real(1.0);
        real& pl = plfo[N - 1];
        pl = real(99.);
        if (platform.substr(6, 5) == "METAR") pl = real(1.);
        if (platform.substr(6, 5) == "SPECI") pl = real(2.);
        if (platform.substr(6, 4) == "SHIP") pl = real(3.);
        if (platform.substr(6, 5) == "SYNOP") pl = real(4.);
        if (platform.substr(6, 4) == "TEMP") pl = real(5.);
        if (platform.substr(6, 5) == "PILOT") pl = real(6.);
        if (platform.substr(0, 7) == "SATWNDS") pl = real(7.);
        if (platform.substr(6, 5) == "SATWI") pl = real(7.);
        if (platform.substr(0, 4) == "SAMS") pl = real(8.);
        if (platform.substr(6, 8) == "PROFILER") pl = real(9.);
        if (platform.substr(6, 5) == "ACARS") pl = real(7.);
        if (is_sound) {
          nlevs_ob[N - 1] = real(meas_count);
          lev_in_ob[N - 1] = real(1.);
          for (int imc = 1; imc <= meas_count; ++imc) {
            if (imc > 1) {
              N = N + 1;
              if (N > niobf)
                obs_fatal("wrf_fddaobs_in: in4dob STOP 122 (more observations than max_obs = " + std::to_string(niobf) +
                          ")");
              nlevs_ob[N - 1] = real(meas_count);
              lev_in_ob[N - 1] = real(imc);
              TOB(N) = rtimob;
              rio[N - 1] = ri;
              rjo[N - 1] = rj;
              rko[N - 1] = real(-99.);
              plfo[N - 1] = plfo[N - imc];
              elevob[N - 1] = elevation;
            }
            need(rec);
            real pressure_data = read_f(rec, 1, 11, 3), pressure_qc = read_f(rec, 13, 11, 3);
            real height_data = read_f(rec, 25, 11, 3), height_qc = read_f(rec, 37, 11, 3);
            real temperature_data = read_f(rec, 49, 11, 3), temperature_qc = read_f(rec, 61, 11, 3);
            real u_met_data = read_f(rec, 73, 11, 3), u_met_qc = read_f(rec, 85, 11, 3);
            real v_met_data = read_f(rec, 97, 11, 3), v_met_qc = read_f(rec, 109, 11, 3);
            real rh_data = read_f(rec, 121, 11, 3), rh_qc = read_f(rec, 133, 11, 3);
            const real m7 = real(-777777.), m8 = real(-888888.);
            if (temperature_qc == m7) temperature_qc = real(0.);
            if (pressure_qc == m7) pressure_qc = real(0.);
            if (height_qc == m7) height_qc = real(0.);
            if (u_met_qc == m7) u_met_qc = real(0.);
            if (v_met_qc == m7) v_met_qc = real(0.);
            if (rh_qc == m7) rh_qc = real(0.);
            if (temperature_data == m8) temperature_qc = m8;
            if (pressure_data == m8) pressure_qc = m8;
            if (height_data == m8) height_qc = m8;
            if (u_met_data == m8) u_met_qc = m8;
            if (v_met_data == m8) v_met_qc = m8;
            if (rh_data == m8) rh_qc = m8;
            if (plfo[N - 1] == real(5.) && (u_met_qc == real(256.) || v_met_qc == real(256.))) {
              u_met_data = m8;
              v_met_data = m8;
              u_met_qc = m8;
              v_met_qc = m8;
            }
            if (plfo[N - 1] == real(6.)) {
              temperature_data = m8;
              rh_data = m8;
              temperature_qc = m8;
              rh_qc = m8;
            }
            auto ok = [](real qc) { return qc >= real(0.) && qc < real(30000.); };
            if (ok(temperature_qc)) {
              if (ok(pressure_qc) || ok(height_qc)) VAR(3, N) = temperature_data;
              else VAR(3, N) = m8;
            } else {
              VAR(3, N) = m8;
            }
            if (ok(height_qc)) VAR(6, N) = height_data;
            else VAR(6, N) = m8;
            if (ok(pressure_qc)) VAR(5, N) = pressure_data;
            else VAR(5, N) = m8;
            if (VAR(5, N) >= real(0.)) VAR(5, N) = VAR(5, N) * real(1.e-3);
            if ((VAR(5, N) > real(0.)) && (VAR(5, N) <= real(8.))) {
              u_met_data = m8;
              v_met_data = m8;
              u_met_qc = m8;
              v_met_qc = m8;
              temperature_data = m8;
              temperature_qc = m8;
              rh_data = m8;
              rh_qc = m8;
            }
            if (ok(u_met_qc) && ok(v_met_qc) && (u_met_data != real(0.) || v_met_data != real(0.))) {
              if (u_met_qc == real(129.) && v_met_qc == real(129.)) rotate_vector(longitude, u_met_data, v_met_data);
              VAR(1, N) = u_met_data;
              VAR(2, N) = v_met_data;
            } else {
              VAR(1, N) = m8;
              VAR(2, N) = m8;
            }
            real r_data = m8;
            if (ok(rh_qc)) {
              if ((pressure_qc >= real(0.)) && (temperature_qc >= real(0.)) && (pressure_qc < real(30000.)) &&
                  (temperature_qc < real(30000.)))
                r_data = rh2r(rh_data, temperature_data, pressure_data * real(.01));
              else
                r_data = m8;
            }
            VAR(4, N) = r_data;
          }
        } else {
          nlevs_ob[N - 1] = real(1.);
          lev_in_ob[N - 1] = real(1.);
          need(rec);
          real slp_data = read_f(rec, 1, 11, 3), slp_qc = read_f(rec, 13, 11, 3);
          (void)read_f(rec, 25, 11, 3);  // ref_pres
          (void)read_f(rec, 37, 11, 3);
          (void)read_f(rec, 49, 11, 3);  // height
          (void)read_f(rec, 61, 11, 3);
          real temperature_data = read_f(rec, 73, 11, 3), temperature_qc = read_f(rec, 85, 11, 3);
          real u_met_data = read_f(rec, 97, 11, 3), u_met_qc = read_f(rec, 109, 11, 3);
          real v_met_data = read_f(rec, 121, 11, 3), v_met_qc = read_f(rec, 133, 11, 3);
          real rh_data = read_f(rec, 145, 11, 3), rh_qc = read_f(rec, 157, 11, 3);
          real psfc_data = read_f(rec, 169, 11, 3), psfc_qc = read_f(rec, 181, 11, 3);
          (void)read_f(rec, 193, 11, 3);  // precip
          (void)read_f(rec, 205, 11, 3);
          const real m8 = real(-888888.);
          auto ok = [](real qc) { return qc >= real(0.) && qc < real(30000.); };
          if ((psfc_qc < real(0.)) && ok(slp_qc) && ok(temperature_qc) && (slp_data > real(90000.))) {
            const real tbar = temperature_data + real(0.5) * elevation * real(.0065);
            psfc_data = slp_data * libm::exp(-elevation / (rovg * tbar));
            VAR(5, N) = psfc_data;
            psfc_qc = real(0.);
          }
          if ((psfc_qc < real(0.)) && ok(temperature_qc)) {
            const real tbar = temperature_data + real(0.5) * elevation * real(.0065);
            psfc_data = real(100000.) * libm::exp(-elevation / (rovg * tbar));
            VAR(5, N) = psfc_data;
            psfc_qc = real(0.);
          }
          if (ok(psfc_qc) && (psfc_data > real(70000.) && psfc_data < real(105000.))) VAR(5, N) = psfc_data;
          else VAR(5, N) = m8;
          if (VAR(5, N) >= real(0.)) VAR(5, N) = VAR(5, N) * real(1.e-3);
          if (ok(temperature_qc)) {
            if (ok(psfc_qc) && (psfc_data > real(70000.) && psfc_data < real(105000.))) VAR(3, N) = temperature_data;
            else VAR(3, N) = m8;
          } else {
            VAR(3, N) = m8;
          }
          if (ok(u_met_qc) && ok(v_met_qc) && (u_met_data != real(0.) || v_met_data != real(0.))) {
            if (u_met_qc == real(129.) && v_met_qc == real(129.)) rotate_vector(longitude, u_met_data, v_met_data);
            VAR(1, N) = u_met_data;
            VAR(2, N) = v_met_data;
          } else {
            VAR(1, N) = m8;
            VAR(2, N) = m8;
          }
          if (plfo[N - 1] == real(3.) && rh_qc >= real(0.) && rh_data < real(70.)) {
            rh_qc = m8;
            rh_data = m8;
          }
          real r_data = m8;
          if (ok(rh_qc)) {
            if (ok(psfc_qc) && ok(temperature_qc)) r_data = rh2r(rh_data, temperature_data, psfc_data * real(.01));
            else r_data = m8;
          }
          VAR(4, N) = r_data;
          // varobs(6): the height of a surface report stays what the array held (WRF sets it for soundings only)
        }
        if (rtimob < tback_) {  // too old for the window (:949-955)
          N = N - meas_count;
          at = L::L110;
          break;
        }
        // duplicates (:959-1093)
        int njend = N - 1;
        if (is_sound) njend = N - meas_count;
        bool next = false;
        for (int njc = 1; njc <= njend && !next; ++njc) {
          if ((TOB(N) == TOB(njc)) && (rio[N - 1] == rio[njc - 1]) && (rjo[N - 1] == rjo[njc - 1]) &&
              (plfo[njc - 1] != real(99.))) {
            if (((plfo[N - 1] <= real(4.)) && (plfo[njc - 1] <= real(4.))) || (plfo[N - 1] == plfo[njc - 1])) {
              if ((!is_sound) && (rko[njc - 1] == rko[N - 1])) {
                for (int kn = 1; kn <= kNdgv; ++kn) VAR(kn, njc) = VAR(kn, N);
                N = N - 1;
                next = true;
              } else if ((is_sound) && (plfo[njc - 1] == plfo[N - 1]) &&
                         ((plfo[njc - 1] == real(5.)) || (plfo[njc - 1] == real(9.)) ||
                          ((plfo[njc - 1] == real(6.)) && (nlevs_ob[N - 1] >= nlevs_ob[njc - 1])))) {
                const int nl = int(lround_x86(nlevs_ob[njc - 1]));
                for (int njcc = njc; njcc <= njc + nl - 1; ++njcc) {
                  for (int kn = 1; kn <= kNdgv; ++kn) VAR(kn, njcc) = real(-888888.);
                  plfo[njcc - 1] = real(99.);
                }
                next = true;
              } else if ((is_sound) && (plfo[njc - 1] == plfo[N - 1]) && (plfo[njc - 1] == real(6.)) &&
                         (nlevs_ob[N - 1] < nlevs_ob[njc - 1])) {
                N = N - meas_count;
                for (int imc = N + 1; imc <= N + meas_count; ++imc) TOB(imc) = real(99999.);
                next = true;
              } else if ((is_sound) && (nlevs_ob[njc - 1] == real(1.)) && (nlevs_ob[N - 1] == real(1.)) &&
                         (VAR(5, njc) == VAR(5, N)) && (plfo[njc - 1] == real(7.)) && (plfo[N - 1] == real(7.))) {
                for (int kn = 1; kn <= kNdgv; ++kn) VAR(kn, njc) = VAR(kn, N);
                N = N - 1;
                next = true;
              }
            }
          }
        }
        if (next) {
          at = L::L100;
          break;
        }
        if (plfo[N - 1] == real(4.) && (platform.substr(6, 10) == "SYNOP PRET") && (id.substr(6, 9) == "METNET= 3")) {
          N = N - 1;
          at = L::L100;
          break;
        }
        if ((ri < real(2.)) || (ri > real(e_we - 1)) || (rj < real(2.)) || (rj > real(e_sn - 1))) {
          N = N - meas_count;
          for (int imc = N + 1; imc <= N + meas_count; ++imc) TOB(imc) = real(99999.);
          at = L::L100;
          break;
        }
        if (TOB(N) < rtlast) obs_fatal("wrf_fddaobs_in: in4dob STOP 111 (2 OBS ARE NOT IN CHRONOLOGICAL ORDER)");
        rtlast = TOB(N);
        if (o.prt_freq <= 0)  // collect_obs_info
          obs_fatal("wrf_fddaobs_in: in4dob STOP (obs_prt_freq must be greater than zero)");
        at = L::L100;
        break;
      }
      case L::L130:
        break;
    }
  }
  // 130 (:1164-1346)
  if (ktau == ktaur) {
    nsta = 0;
    for (int n = 1; n <= niobf; ++n) {
      if (TOB(n) > real(9.e4)) break;
      nsta = n;
    }
    shift_or_fill(old_obs(nsta), false);
  }
  nsta = 0;
  for (int n = 1; n <= niobf; ++n) {
    if (TOB(n) > real(9.e4)) break;
    nsta = n;
  }
  nstaw = nsta;  // nvals_le_limit (printouts only)
  if (ktau == ktaur && iwtsig != 1) {
    if (rinfmn > rinfmx) obs_fatal("wrf_fddaobs_in: in4dob STOP 556");
    if (dpsmx > real(10.)) obs_fatal("wrf_fddaobs_in: in4dob STOP 557 (obs_dpsmx > 10)");
  }
  nstat = nsta;
}

// ---------------------------------------------------------------------------------------------------------------
// fddaobs_driver (module_fddaobs_driver.F:35-470) at first_rk_step_part2, after update_phy_ten

void PhysicsSuite::fddaobs_driver(const dyn::State& s, const dyn::BaseState& b, dyn::Tendencies& t) {
  ObsNudging& ob = *obs_;
  const ObsNudgeOptions& o = o_.obs;
  const real xtime = wrf_time::minutes<real>(clock_since_start());
  if (!(xtime <= real(o.fdda_end))) return;  // first_rk_step_part2: grid%xtime <= grid%fdda_end
  const Grid& g = g_;
  const int nx = g.nx, ny = g.ny, nz = g.nz;
  const real dtmin = real(g.dt) / real(60.);
  const long ktau = itimestep - 1;
  const int infr = o.ionf * powi(o.parent_grid_ratio, o.levidn);
  const int nsta = ob.nstat;
  const bool restart = ob.restart;
  const int idarst = (restart && ktau == ob.ktaur) ? 1 : 0;
  const Range2D cols{0, nx, 0, ny};
  // REGIME (module_fddaobs_driver.F:286-299)
  if (o_.sf_sfclay_physics != 1 && o_.sf_sfclay_physics != 5 && o_.sf_sfclay_physics != 7 &&
      o_.sf_sfclay_physics != 91)
    column_for(cols, KObsRegime{regime.view(), pblh.cview(), prep.z_at_w.cview(), b.ht.view()});
  // grid%z: compute_diff_metrics' (diff_opt 1, 2) or phy_prep's
  const Field3* zf = &prep.z;
  if (o.diff_z) {
    column_for(cols, KObsDiffZ{s.ph_2.cview(), b.phb.view(), ob.zdiff.view(), nz});
    zf = &ob.zdiff;
  }
  // compute_VIH
  if (ob.o_.sfc_scheme_vert == 0) {
    const int flags[3] = {o.nudge_wind, o.nudge_temp, o.nudge_mois};
    const std::array<real, 6>* vifs[3] = {&ob.vif_uv, &ob.vif_t, &ob.vif_q};
    for (int v = 0; v < 3; ++v) {
      if (!(flags[v] == 1 && nsta > 0)) continue;
      VifSet vs{};
      for (int q = 0; q < 6; ++q) vs.vif[q] = (*vifs[v])[q];
      vs.hmax = ob.vif_max;
      vs.fullmin = ob.vif_fullmin;
      vs.rampmin = ob.vif_rampmin;
      column_for(cols, KObsVih{vs, ob.vih[2 * v].view(), ob.vih[2 * v + 1].view(), regime.cview(), pblh.cview()});
    }
    fatal_check();
  }
  // errob
  if (nsta > 0 && (ktau % infr == 0 || idarst == 1)) {
    ob.upload(nsta);
    ErrobScalars ec{};
    ec.ide = nx + 1;
    ec.jde = ny + 1;
    ec.k_end = nz;
    ec.iswind = o.nudge_wind;
    ec.istemp = o.nudge_temp;
    ec.ismois = o.nudge_mois;
    ec.t0 = real(cst::t0);
    ec.rcp = real(cst::r_d) / (real(7.) * real(cst::r_d) / real(2.));
    const tile::Whole whole;  // one thread per observation
    column_for(Range2D{0, nsta, 0, 1},
               KErrob{ec, ob.d, s.u_2.cview(), s.v_2.cview(), prep.th_phy_m_t0.cview(), s.q_2[s.P_QV].cview(),
                      s.p.cview(), zf->cview(), b.pb.view(), uratx.cview(), vratx.cview(), tratx.cview(), kpbl.cview()});
    ob.download(nsta);
    if (std::getenv("CPPWRF_OBS_DEBUG")) {  // the observations after errob (a diagnostic of the port)
      for (int n = 0; n < nsta; ++n) {
        std::printf("obs ktau %ld n %d t %.17g ri %.17g rj %.17g rk %.17g nl %g lv %g pl %g var", ktau, n + 1,
                    double(ob.timeob[n]), double(ob.rio[n]), double(ob.rjo[n]), double(ob.rko[n]),
                    double(ob.nlevs_ob[n]), double(ob.lev_in_ob[n]), double(ob.plfo[n]));
        for (int kn = 0; kn < ObsNudging::kNdgv; ++kn) std::printf(" %.17g", double(ob.varobs[kn + 6 * n]));
        std::printf(" errf");
        for (int kn = 0; kn < ObsNudging::kErrf; ++kn) std::printf(" %.17g", double(ob.errf[kn + 10 * n]));
        std::printf("\n");
      }
    }
  }
  // tfaci (dynamic initialisation ramp)
  ob.tfaci = real(1.0);
  if (o.idynin == 1) {
    const real dtr = std::fabs(real(o.dtramp));
    const real tconst = real(1.) / dtr;
    const real fdaend = real(o.fdda_end);
    if (xtime < fdaend - dtr) ob.tfaci = real(1.);
    else if (xtime >= fdaend - dtr && xtime <= fdaend) ob.tfaci = (fdaend - xtime) * tconst;
    else ob.tfaci = real(0.0);
  }
  if (nsta <= 0) return;
  // nudob for T, QV, U, V (the rows of each variable are independent; one launch per variable)
  NudobScalars nc{};
  nc.inest = o.grid_id;
  nc.ide = nx + 1;
  nc.jde = ny + 1;
  nc.kte = nz;
  nc.icut = o.grid_id > 1 ? 1 : 0;
  nc.nstat = nsta;
  nc.recompute = (ktau % infr == 0 || (restart && ktau == ob.ktaur)) ? 1 : 0;
  nc.sfc_scheme_horiz = ob.o_.sfc_scheme_horiz;
  nc.sfc_scheme_vert = ob.o_.sfc_scheme_vert;
  nc.scl_neg_qv_innov = o.scl_neg_qv_innov;
  nc.xtime = xtime;
  nc.dtmin = dtmin;
  nc.twindo = ob.window;
  nc.sfcfact = real(o.sfcfact);
  nc.sfcfacr = real(o.sfcfacr);
  nc.rinfmn = ob.rinfmn;
  nc.rinfmx = ob.rinfmx;
  nc.pfree = ob.pfree;
  nc.dcon = ob.dcon;
  nc.tfaci = ob.tfaci;
  nc.maxsnd_gap = real(o.max_sndng_gap);
  const int nscan = 0;
  nc.gfactor = real(1.) + real(nscan) * (real(-1.) + real(0.33333));
  const real rfactor = real(1.) + real(nscan) * (real(-1.) + real(3.0));
  nc.rindx = real(o.rinxy) * real(1000.) / real(g.dx) * rfactor;
  auto launch = [&](int ivar, Field3& aten, const Field2& mu, const B2& msf, int isw, double coef, bool pbl, int v) {
    NudobScalars c = nc;
    c.ivar = ivar;
    c.isw = isw;
    c.coef = real(coef);
    c.nudge_pbl = pbl ? 1 : 0;
    column_for(cols, KNudob{c, ob.d, b.pb.view(), s.p.cview(), s.q_2[s.P_QV].cview(), zf->cview(),
                            ob.vih[2 * v].cview(), ob.vih[2 * v + 1].cview(), mu.cview(), b.ht.view(), msf,
                            kpbl.cview(), b.c1h.view(), b.c2h.view(), aten.view(), ob.savwt[ivar - 1].view()});
    fatal_check();  // nudob's STOPs (kpbl_obs, sounding levels)
  };
  if (o.nudge_temp == 1) launch(3, t.t_tendf, s.mut, b.msftx.view(), o.nudge_temp, o.coef_temp, ob.nudge_t_pbl, 1);
  if (o.nudge_mois == 1)
    launch(4, t.q_tendf[s.P_QV], s.mut, b.msftx.view(), o.nudge_mois, o.coef_mois, ob.nudge_q_pbl, 2);
  if (o.nudge_wind == 1) {
    launch(1, t.ru_tendf, s.muu, b.msfuy.view(), o.nudge_wind, o.coef_wind, ob.nudge_uv_pbl, 0);
    launch(2, t.rv_tendf, s.muv, b.msfvx.view(), o.nudge_wind, o.coef_wind, ob.nudge_uv_pbl, 0);
  }
}

}  // namespace cppwrf::phys::driver
