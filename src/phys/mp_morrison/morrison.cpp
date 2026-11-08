// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/mp_morrison/morrison.hpp"

#include <type_traits>

#include "core/limits.hpp"

namespace cppwrf::phys::mp_morrison {

template <typename Real>
void run_morrison(const MorrisonParams<Real>& c, Real dt, const Range2D& range, int nz, View3<Real> th, View3<Real> qv,
                  View3<Real> qc, View3<Real> qr, View3<Real> qi, View3<Real> qs, View3<Real> qg, View3<Real> ni,
                  View3<Real> ns, View3<Real> nr, View3<Real> ng, View3<const Real> pii, View3<const Real> p,
                  View3<const Real> dz, View3<const Real> qrcuten, View3<const Real> qscuten,
                  View3<const Real> qicuten, View2<Real> rainnc, View2<Real> rainncv, View2<Real> snownc,
                  View2<Real> snowncv, View2<Real> graupelnc, View2<Real> graupelncv, View2<Real> sr) {
  check_limit("Morrison", "nz", nz, kMaxLevels);
  // the column arrays sized to the next tier above nz (on the GPU in the shared work buffer: morrison_reserve)
  auto launch = [&](auto nl) {
    MorrisonKernel<Real, decltype(nl)::value> k{c, dt, nz, th, qv, qc, qr, qi, qs, qg, ni, ns, nr, ng, pii, p, dz,
                                                qrcuten, qscuten, qicuten, qrcuten.p != nullptr, rainnc, rainncv,
                                                snownc, snowncv, graupelnc, graupelncv, sr};
    column_for_work(range, k);
  };
  if (nz <= 48) launch(std::integral_constant<int, 48>{});
  else if (nz <= 64) launch(std::integral_constant<int, 64>{});
  else launch(std::integral_constant<int, kMaxLevels>{});
}

template <typename Real>
void morrison_reserve(const Range2D& range, int nz) {
  const std::size_t b = nz <= 48   ? MorrisonKernel<Real, 48>::kWorkBytes
                        : nz <= 64 ? MorrisonKernel<Real, 64>::kWorkBytes
                                   : MorrisonKernel<Real, kMaxLevels>::kWorkBytes;
  column_work_reserve(range, b);
}

#define CPPWRF_MORRISON_INST(R)                                                                                      \
  template void morrison_reserve<R>(const Range2D&, int);                                                            \
  template void run_morrison<R>(const MorrisonParams<R>&, R, const Range2D&, int, View3<R>, View3<R>, View3<R>,      \
                                View3<R>, View3<R>, View3<R>, View3<R>, View3<R>, View3<R>, View3<R>, View3<R>,      \
                                View3<const R>, View3<const R>, View3<const R>, View3<const R>, View3<const R>,      \
                                View3<const R>, View2<R>, View2<R>, View2<R>, View2<R>, View2<R>, View2<R>, View2<R>);
CPPWRF_MORRISON_INST(double)
CPPWRF_MORRISON_INST(float)
#undef CPPWRF_MORRISON_INST

}  // namespace cppwrf::phys::mp_morrison
