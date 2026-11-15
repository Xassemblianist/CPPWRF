// SPDX-License-Identifier: GPL-3.0-or-later
// Single-layer urban canopy model column driver (C++ side of tests/phys/sf_urban): read_urban_table and
// urban_point on the points of urban_input.bin, in the output format of driver_urban.f90.
//   driver_urban_cpu INPUT OUTPUT        (URBPARM.TBL / URBPARM_LCZ.TBL in the working directory)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "phys/sf_urban/urban.hpp"
#include "phys/sf_urban/urban_tables.hpp"

using namespace cppwrf::phys::urban;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: driver_urban_cpu INPUT OUTPUT\n");
    return 2;
  }
  std::FILE* in = std::fopen(argv[1], "rb");
  std::FILE* out = std::fopen(argv[2], "wb");
  if (!in || !out) return 1;
  std::int32_t hdr[3];
  if (std::fread(hdr, 4, 3, in) != 3) return 1;
  const int npts = hdr[0], nstep = hdr[1], lcz = hdr[2];
  constexpr int nl = 4;
  UrbanTables tb;
  UrbanTableOptions opt;
  try {
    const char* path = lcz == 0 ? "URBPARM.TBL" : "URBPARM_LCZ.TBL";
    read_urban_table(path, nl, 1, tb, opt);
    check_urban_table_options(opt, path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  std::fwrite(tb.dzr, 8, nl, out);
  std::fwrite(tb.dzb, 8, nl, out);
  std::fwrite(tb.dzg, 8, nl, out);
  for (int ip = 0; ip < npts; ++ip) {
    std::int32_t utype;
    double f[18], st[29];
    if (std::fread(&utype, 4, 1, in) != 1 || std::fread(f, 8, 18, in) != 18 || std::fread(st, 8, 29, in) != 29)
      return 1;
    UrbanState s{};
    s.tr = st[0]; s.tb = st[1]; s.tg = st[2]; s.tc = st[3]; s.qc = st[4]; s.uc = st[5];
    s.xxxr = st[6]; s.xxxb = st[7]; s.xxxg = st[8]; s.xxxc = st[9];
    for (int k = 0; k < nl; ++k) { s.trl[k] = st[10 + k]; s.tbl[k] = st[14 + k]; s.tgl[k] = st[18 + k]; }
    s.flxhumr = st[22]; s.flxhumb = st[23]; s.flxhumg = st[24];
    s.cmr = st[25]; s.chr = st[26]; s.cmc = st[27]; s.chc = st[28];
    for (int istep = 0; istep < nstep; ++istep) {
      s.chs = f[16];
      s.chs2 = f[17];
      UrbanOut o{};
      if (!urban_point(tb, nl, utype, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11],
                       f[12], f[13], f[14], s, o)) {
        std::fprintf(stderr, "point %d: ZDC + Z0C + 2m is larger than the 1st WRF level\n", ip);
        return 1;
      }
      const double ov[18] = {o.ts, o.qs, o.sh, o.lh, o.lh_kinematic, o.sw, o.alb, o.lw, o.g, o.rn,
                             o.psim, o.psih, o.gz1oz0, o.u10, o.v10, o.th2, o.q2, o.ust};
      std::fwrite(ov, 8, 18, out);
      double sv[29] = {s.tr, s.tb, s.tg, s.tc, s.qc, s.uc, s.xxxr, s.xxxb, s.xxxg, s.xxxc};
      for (int k = 0; k < nl; ++k) { sv[10 + k] = s.trl[k]; sv[14 + k] = s.tbl[k]; sv[18 + k] = s.tgl[k]; }
      sv[22] = s.flxhumr; sv[23] = s.flxhumb; sv[24] = s.flxhumg;
      sv[25] = s.cmr; sv[26] = s.chr; sv[27] = s.cmc; sv[28] = s.chc;
      std::fwrite(sv, 8, 29, out);
    }
  }
  std::fclose(in);
  std::fclose(out);
  return 0;
}
