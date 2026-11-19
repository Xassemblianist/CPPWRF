// SPDX-License-Identifier: GPL-3.0-or-later
// NSSL 2-moment microphysics: the work space of one row (NsslScheme::work_bytes) holds what nssl_row takes, also for
// many levels (the row's arrays and sediment1d's are live together: about 97 nx nz doubles). The row runs on the host
// over a buffer of twice the bound filled with a marker; the last byte the row changed is its high-water mark. The
// bound is the same in every build; a CUDA build (this file compiled as CUDA) skips the host run.
#include <cstdio>
#if defined(CPPWRF_USE_CUDA)
int main() {
  std::printf("skipped in the CUDA build (the CPU build runs the row)\n");
  return 0;
}
#else
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "core/backend.hpp"
#include "phys/mp_nssl/nssl_args.hpp"
#include "phys/mp_nssl/nssl_init.hpp"
#include "phys/mp_nssl/nssl_row.hpp"

namespace mn = cppwrf::phys::mp_nssl;
using cppwrf::View2;
using cppwrf::View3;

namespace {

using R = double;
mn::NsslParams<R> P;
mn::NsslTables<R> T;

// a convective column per point: cloud water, rain, ice, snow, graupel and hail with numbers, an updraft
std::size_t high_water(int nx, int nz) {
  const int sk = nx, sj = nx * nz;
  const std::size_t n3 = std::size_t(nx) * nz, pad3 = 1 + sj, pad2 = 1 + nx;
  constexpr int N3 = 35, N2 = 9;
  std::vector<std::vector<double>> a3(N3, std::vector<double>(pad3 + n3, 0.0));
  std::vector<std::vector<double>> a2(N2, std::vector<double>(pad2 + nx, 0.0));
  auto at = [&](int q, int i, int k) -> double& { return a3[q][pad3 + std::size_t(k) * nx + (i - 1)]; };
  const double ztop = 17000.0;
  for (int i = 1; i <= nx; ++i) {
    for (int k = 0; k < nz; ++k) {
      const double zw0 = ztop * k / nz, zw1 = ztop * (k + 1) / nz, zm = 0.5 * (zw0 + zw1);
      const double t = std::fmax(300.0 - 0.0068 * zm, 210.0), p = 100000.0 * std::exp(-zm / 7600.0);
      const double tc = t - 273.15, es = 611.2 * std::exp(17.67 * tc / (t - 29.65));
      const double qv = 0.95 * 0.622 * es / (p - es), rho = p / (287.04 * t * (1 + 0.608 * qv));
      const double pii = std::pow(p / 1.0e5, 287.04 / 1004.0);
      const bool cloud = zm > 1000.0 && zm < 12000.0 && (i + k) % 5 != 0;
      const double s = 1.0 + 0.1 * ((i * 7 + k * 3) % 11);
      const double qc = cloud && t > 235.0 ? 5e-4 * s : 0.0, qr = cloud ? 2e-4 * s : 0.0;
      const double qi = cloud && t < 273.15 ? 1e-4 * s : 0.0, qs = cloud && t < 278.0 ? 2e-4 * s : 0.0;
      const double qg = cloud ? 3e-4 * s : 0.0, qh = cloud && (i % 2 == 0) ? 4e-4 * s : 0.0;
      auto num = [](double q, double d, double rx) { return q > 0 ? q / (rx * M_PI / 6 * d * d * d) : 0.0; };
      at(0, i, k) = t / pii; at(1, i, k) = qv; at(2, i, k) = qc; at(3, i, k) = qr; at(4, i, k) = qi;
      at(5, i, k) = qs; at(6, i, k) = qg; at(7, i, k) = qh;
      at(8, i, k) = num(qc, 1e-5, 1000.0); at(9, i, k) = num(qr, 1e-3, 1000.0); at(10, i, k) = num(qi, 5e-5, 500.0);
      at(11, i, k) = num(qs, 1e-3, 100.0); at(12, i, k) = num(qg, 2e-3, 400.0); at(13, i, k) = num(qh, 1e-2, 800.0);
      at(14, i, k) = qg / 400.0; at(15, i, k) = qh / 800.0; at(16, i, k) = 1.0e8;
      at(17, i, k) = pii; at(18, i, k) = p; at(19, i, k) = cloud ? 8.0 * std::sin(M_PI * zm / ztop) : 0.0;
      at(20, i, k) = zw1 - zw0; at(21, i, k) = rho;
    }
  }
  auto V3 = [&](int q) { return View3<double>{a3[q].data() + pad3 - 1 - sj, sj, sk}; };
  auto C3 = [&](int q) { return View3<const double>{a3[q].data() + pad3 - 1 - sj, sj, sk}; };
  auto V2 = [&](int q) { return View2<double>{a2[q].data() + pad2 - 1 - nx, nx}; };
  mn::NsslRowArgs<double> A{};
  A.its = 1; A.ite = nx; A.nz = nz; A.dtp = 20.0; A.flag_ccn = true; A.cu_used = false;
  A.has_reqc = A.has_reqi = A.has_reqs = 1; A.nssl_ssat_output = 0; A.do_dbz = true;
  A.th = V3(0); A.qv = V3(1); A.qc = V3(2); A.qr = V3(3); A.qi = V3(4); A.qs = V3(5); A.qh = V3(6); A.qhl = V3(7);
  A.ccw = V3(8); A.crw = V3(9); A.cci = V3(10); A.csw = V3(11); A.chw = V3(12); A.chl = V3(13); A.vhw = V3(14);
  A.vhl = V3(15); A.cn = V3(16); A.pii = C3(17); A.p = C3(18); A.w = C3(19); A.dz = C3(20); A.dn = C3(21);
  A.qrcuten = C3(22); A.qscuten = C3(23); A.qicuten = C3(24); A.qccuten = C3(25);
  A.dbz = V3(26); A.ssat3d = V3(27); A.ssati = V3(28); A.re_cloud = V3(29); A.re_ice = V3(30); A.re_snow = V3(31);
  A.rainnc = V2(0); A.rainncv = V2(1); A.snownc = V2(2); A.snowncv = V2(3); A.hailnc = V2(4); A.hailncv = V2(5);
  A.grplnc = V2(6); A.grplncv = V2(7); A.sr = V2(8);
  const std::size_t cap = 2 * mn::NsslScheme<double>::work_bytes(nx, nz);
  std::unique_ptr<unsigned char[]> buf(new unsigned char[cap]);
  std::memset(buf.get(), 0xA5, cap);
  for (int step = 1; step <= 2; ++step) {  // the cold start (numbers from mass) and a later step with diagnostics
    A.itimestep = step; A.diagflag = step == 2; A.ke_diag = step == 2 ? nz : 1;
    mn::Work W{reinterpret_cast<char*>(buf.get()), reinterpret_cast<char*>(buf.get()) + cap};
    mn::nssl_row(P, W, A, 1);
  }
  std::size_t hw = cap;
  while (hw > 0 && buf[hw - 1] == 0xA5) --hw;
  return hw;
}

}  // namespace

int main() {
  mn::nssl_init(P, T, mn::NsslConfig{});
  int fails = 0;
  const int nxs[] = {1, 8, 64, 65, 200};
  const int nzs[] = {35, 60, 120, 250, 400, 600};
  for (int nx : nxs) {
    for (int nz : nzs) {
      if (std::size_t(nx) * nz > 40000) continue;
      const std::size_t hw = high_water(nx, nz), wb = mn::NsslScheme<double>::work_bytes(nx, nz);
      const bool ok = hw <= wb;
      std::printf("nx %3d nz %3d: high-water %10zu bytes, work_bytes %10zu (%5.1f %%)%s\n", nx, nz, hw, wb,
                  100.0 * double(hw) / double(wb), ok ? "" : "  EXCEEDED");
      if (!ok) ++fails;
    }
  }
  if (fails) {
    std::printf("FAIL: %d row(s) beyond work_bytes\n", fails);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
#endif
