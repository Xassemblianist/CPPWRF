// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The model fields of one call of nssl_2mom_driver (nssl_row.hpp), and the scheme's host interface (nssl.cpp).
#include <cstddef>
#include <memory>

#include "core/field.hpp"

namespace cppwrf::phys::mp_nssl {

template <class Real>
struct NsslRowArgs {
  int its, ite, nz;  // the row's i range (WRF its:ite of the tile) and levels
  int itimestep;
  bool diagflag;      // solve_em diag_flag (history alarm, or nwp_diagnostics)
  int ke_diag;        // solve_em ke_diag (Fortran level: kms = 1, or the domain top at a history step)
  Real dtp;
  bool flag_ccn;      // F_QNN (qnn present: nssl_ccn_on)
  bool cu_used;       // config_flags%cu_used
  int has_reqc, has_reqi, has_reqs;
  int nssl_ssat_output;
  bool do_dbz;        // REFL_10CM allocated (compute_radar_ref)
  View3<Real> th, qv, qc, qr, qi, qs, qh, qhl;               // qh = WRF qg (graupel), qhl = WRF qh (hail)
  View3<Real> ccw, crw, cci, csw, chw, chl, vhw, vhl, cn;    // qndrop qnr qni qns qng qnh qvolg qvolh qnn
  View3<const Real> pii, p, w, dz, dn;
  View3<const Real> qrcuten, qscuten, qicuten, qccuten;
  View2<Real> rainnc, rainncv, snownc, snowncv, hailnc, hailncv, grplnc, grplncv, sr;
  View3<Real> dbz, re_cloud, re_ice, re_snow, ssat3d, ssati;
};

// mp_init's arguments of nssl_2mom_init (nssl_params(1:10), nssl_icdx, nssl_icdxhl)
struct NsslConfig {
  double cccn = 0.5e9, alphah = 0.0, alphahl = 1.0, cnoh = 4.e5, cnohl = 4.e4, cnor = 8.e5, cnos = 3.e6;
  double rho_qh = 500., rho_qhl = 900., rho_qs = 100.;
  int icdx = 6, icdxhl = 6;
};

// nssl_2mom_init once (module values and tables, on the device in a CUDA build) and nssl_2mom_driver over rows:
// run() calls nssl_row for the rows j0 .. j1-1 of A (one thread per row, each with its work space), the row being
// WRF's tile i-range A.its .. A.ite
template <typename Real>
class NsslScheme {
 public:
  NsslScheme(const NsslConfig& c, int nx_row, int nz, int max_rows = 4096);  // max_rows: the domain's rows
  void run(const NsslRowArgs<Real>& A, int j0, int j1) const;
  int ccn_is_ccna() const;  // nssl_2mom_init's ccn_is_ccna (1: QNN holds the activated CCN)
  static std::size_t work_bytes(int nx_row, int nz);  // one row's work space

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace cppwrf::phys::mp_nssl
