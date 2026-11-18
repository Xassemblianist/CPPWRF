// SPDX-License-Identifier: GPL-3.0-or-later
// The C++ NSSL 2-moment microphysics (nssl_init, nssl_row for every j row of the tile) on the input of
// driver_nssl.f90; writes the same output.
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include "phys/mp_nssl/nssl_init.hpp"
#include "phys/mp_nssl/nssl_row.hpp"

using namespace cppwrf::phys::mp_nssl;
using cppwrf::View2;
using cppwrf::View3;

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  FILE* f = std::fopen(argv[1], "rb");
  if (!f) return 1;
  int hdr[6];
  double dt;
  if (std::fread(hdr, 4, 6, f) != 6 || std::fread(&dt, 8, 1, f) != 1) return 1;
  const int nx = hdr[0], ny = hdr[1], nz = hdr[2], nsteps = hdr[3], cu_used = hdr[4], ssat_out = hdr[5];
  const std::size_t n3 = std::size_t(nx) * nz * ny, n2 = std::size_t(nx) * ny;
  // Fortran (1:nx, 1:nz, 1:ny) storage behind a pad so that the views take i = 1..nx, j = 1..ny, k = 0..nz-1
  const int sk = nx, sj = nx * nz;
  const std::size_t pad3 = 1 + sj, pad2 = 1 + nx;
  constexpr int N3 = 35, N2 = 9;
  std::vector<std::vector<double>> a3(N3, std::vector<double>(pad3 + n3, 0.0));
  std::vector<std::vector<double>> a2(N2, std::vector<double>(pad2 + n2, 0.0));
  for (int q = 0; q < 26; ++q)
    if (std::fread(a3[q].data() + pad3, 8, n3, f) != n3) return 1;
  // rainnc snownc hailnc grplnc
  const int acc_ids[4] = {0, 2, 4, 6};
  for (int q : acc_ids)
    if (std::fread(a2[q].data() + pad2, 8, n2, f) != n2) return 1;
  auto V3 = [&](int q) { return View3<double>{a3[q].data() + pad3 - 1 - sj, sj, sk}; };
  auto C3 = [&](int q) { return View3<const double>{a3[q].data() + pad3 - 1 - sj, sj, sk}; };
  auto V2 = [&](int q) { return View2<double>{a2[q].data() + pad2 - 1 - nx, nx}; };

  static NsslParams<double> P;
  static NsslTables<double> T;
  nssl_init(P, T, NsslConfig{});

  NsslRowArgs<double> A{};
  A.its = 1; A.ite = nx; A.nz = nz; A.dtp = dt; A.flag_ccn = true; A.cu_used = cu_used != 0;
  A.has_reqc = A.has_reqi = A.has_reqs = 1; A.nssl_ssat_output = ssat_out; A.do_dbz = true;
  A.th = V3(0); A.qv = V3(1); A.qc = V3(2); A.qr = V3(3); A.qi = V3(4); A.qs = V3(5); A.qh = V3(6); A.qhl = V3(7);
  A.ccw = V3(8); A.crw = V3(9); A.cci = V3(10); A.csw = V3(11); A.chw = V3(12); A.chl = V3(13); A.vhw = V3(14);
  A.vhl = V3(15); A.cn = V3(16); A.pii = C3(17); A.p = C3(18); A.w = C3(19); A.dz = C3(20); A.dn = C3(21);
  A.qrcuten = C3(22); A.qscuten = C3(23); A.qicuten = C3(24); A.qccuten = C3(25);
  A.dbz = V3(26); A.ssat3d = V3(27); A.ssati = V3(28); A.re_cloud = V3(29); A.re_ice = V3(30); A.re_snow = V3(31);
  A.rainnc = V2(0); A.rainncv = V2(1); A.snownc = V2(2); A.snowncv = V2(3); A.hailnc = V2(4); A.hailncv = V2(5);
  A.grplnc = V2(6); A.grplncv = V2(7); A.sr = V2(8);

  const std::size_t wbytes = std::size_t(256) << 20;
  std::unique_ptr<char[]> wbuf(new char[wbytes]);
  FILE* o = std::fopen(argv[2], "wb");
  for (int s = 0; s < nsteps; ++s) {
    int st[3];
    if (std::fread(st, 4, 3, f) != 3) return 1;
    A.itimestep = st[0]; A.diagflag = st[1] != 0; A.ke_diag = st[2];
    for (int jy = 1; jy <= ny; ++jy) {
      Work W{wbuf.get(), wbuf.get() + wbytes};
      nssl_row(P, W, A, jy);
    }
    const int outs[23] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 26, 27, 28, 29, 30, 31};
    for (int q : outs) std::fwrite(a3[q].data() + pad3, 8, n3, o);
    for (int q = 0; q < N2; ++q) std::fwrite(a2[q].data() + pad2, 8, n2, o);
  }
  std::fclose(o);
  std::fclose(f);
  return 0;
}
