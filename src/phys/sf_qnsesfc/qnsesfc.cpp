// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/sf_qnsesfc/qnsesfc.hpp"

#include <vector>

namespace cppwrf::phys::sf_qnsesfc {

// QNSESFCINIT's function definition loop (module_sf_qnsesfc.F:1123-1277): Paulson's unstable functions bounded
// below by Sukoriansky's QNSE polynomials, the QNSE stable functions; ZETA**2 = ZETA*ZETA, ZETA**3 = (ZETA*ZETA)*ZETA,
// X**5 = ((X*X)*X)*(X*X) (gfortran's powi expansion)
void QNSESFCTableManager::compute(real* psim1, real* psih1, real* psim2, real* psih2, Tables& tb) {
  const real pihf = real(3.1415926) / real(2.);
  const real eps = real(1.e-6);
  real ztmin1 = real(-5.0), ztmax1 = real(2.3);
  real ztmin2 = real(-5.0), ztmax2 = real(2.3);
  const real zrng1 = ztmax1 - ztmin1, zrng2 = ztmax2 - ztmin2;
  const real dzeta1 = zrng1 / real(KZTM - 1), dzeta2 = zrng2 / real(KZTM - 1);
  auto unstable = [&](real zeta, real& psim, real& psih) {
    const real x = std::sqrt(std::sqrt(real(1.) - real(16.) * zeta));
    psim = real(-2.) * libm::log((x + real(1.)) / real(2.)) - libm::log((x * x + real(1.)) / real(2.)) +
           real(2.) * libm::atan(x) - pihf;
    psih = real(-2.) * libm::log((x * x + real(1.)) / real(2.));
    const real z2 = zeta * zeta, z3 = z2 * zeta;
    const real psimq = real(2.25) * zeta + z2 + real(45.9) * z3;
    const real psihq = real(2.0) * zeta + real(10.3) * z2 + real(130) * z3;
    psim = c_max(psimq, psim);
    psih = c_max(psihq, psih);
  };
  auto stable = [&](real zeta, real& psim, real& psih) {
    psim = real(2.25) * zeta - real(0.2) * zeta * zeta;
    const real x = zeta - real(0.5), x2 = x * x, x5 = (x2 * x) * x2;
    psih = real(2.0) * zeta + real(0.14) * (x5 + real(0.03125));
  };
  real zeta1 = ztmin1, zeta2 = ztmin2;
  for (int k = 1; k <= KZTM; ++k) {
    if (zeta1 < real(0.)) unstable(zeta1, psim1[k - 1], psih1[k - 1]);
    else stable(zeta1, psim1[k - 1], psih1[k - 1]);
    if (zeta2 < real(0.)) unstable(zeta2, psim2[k - 1], psih2[k - 1]);
    else stable(zeta2, psim2[k - 1], psih2[k - 1]);
    if (k == KZTM) {
      ztmax1 = zeta1;
      ztmax2 = zeta2;
    }
    zeta1 = zeta1 + dzeta1;
    zeta2 = zeta2 + dzeta2;
  }
  tb.ztmin1 = ztmin1;
  tb.ztmax1 = ztmax1 - eps;
  tb.dzeta1 = dzeta1;
  tb.fh01 = real(1.);
  tb.ztmin2 = ztmin2;
  tb.ztmax2 = ztmax2 - eps;
  tb.dzeta2 = dzeta2;
  tb.fh02 = real(1.);
}

QNSESFCTableManager::QNSESFCTableManager() {
  std::vector<real> h[4];
  for (auto& v : h) v.assign(KZTM, real(0.));
  compute(h[0].data(), h[1].data(), h[2].data(), h[3].data(), tables_);
  for (int n = 0; n < 4; ++n) {
    dev_[n] = dev_alloc(KZTM * sizeof(real));
    copy_to_dev(dev_[n], h[n].data(), KZTM * sizeof(real));
  }
  tables_.psim1 = View1<const real>{static_cast<const real*>(dev_[0])};
  tables_.psih1 = View1<const real>{static_cast<const real*>(dev_[1])};
  tables_.psim2 = View1<const real>{static_cast<const real*>(dev_[2])};
  tables_.psih2 = View1<const real>{static_cast<const real*>(dev_[3])};
}

QNSESFCTableManager::~QNSESFCTableManager() {
  for (auto& p : dev_)
    if (p) {
      dev_free(p);
      p = nullptr;
    }
}

void run_qnsesfc(const QNSESFCKernel& kernel, const Range2D& range) { column_for(range, kernel); }

}  // namespace cppwrf::phys::sf_qnsesfc
