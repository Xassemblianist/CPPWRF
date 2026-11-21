// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include <cstdint>
#include <string>

namespace cppwrf::phys {

// The tables as packed structs of reals R (WRF's rb) and int32 members, in the order the dump program writes
// them (tests/phys/ra_rrtmg_lw/dump_tables.f90): RrtmgLwTablesOf<double> is the layout of the WRF -r8 dump, RrtmgLwTablesOf<float> that of
// the WRF real*4 dump (data/rrtmg/README.md).
#pragma pack(push, 1)

template <class R>
struct Band01TablesOf {
  R fracrefa[10];
  R fracrefb[10];
  R ka[10 * 65];
  R kb[10 * 235];
  R ka_mn2[10 * 19];
  R kb_mn2[10 * 19];
  R selfref[10 * 10];
  R forref[10 * 4];
};

template <class R>
struct Band02TablesOf {
  R fracrefa[12];
  R fracrefb[12];
  R ka[12 * 65];
  R kb[12 * 235];
  R selfref[12 * 10];
  R forref[12 * 4];
};

template <class R>
struct Band03TablesOf {
  R fracrefa[9 * 16];
  R fracrefb[5 * 16];
  R ka[16 * 585];
  R kb[16 * 1175];
  R ka_mn2o[16 * 9 * 19];
  R kb_mn2o[16 * 5 * 19];
  R selfref[16 * 10];
  R forref[16 * 4];
};

template <class R>
struct Band04TablesOf {
  R fracrefa[9 * 14];
  R fracrefb[5 * 14];
  R ka[14 * 585];
  R kb[14 * 1175];
  R selfref[14 * 10];
  R forref[14 * 4];
};

template <class R>
struct Band05TablesOf {
  R fracrefa[9 * 16];
  R fracrefb[5 * 16];
  R ka[16 * 585];
  R kb[16 * 1175];
  R ka_mo3[16 * 9 * 19];
  R ccl4[16];
  R selfref[16 * 10];
  R forref[16 * 4];
};

template <class R>
struct Band06TablesOf {
  R fracrefa[8];
  R ka[8 * 65];
  R ka_mco2[8 * 19];
  R cfc11adj[8];
  R cfc12[8];
  R selfref[8 * 10];
  R forref[8 * 4];
};

template <class R>
struct Band07TablesOf {
  R fracrefa[9 * 12];
  R fracrefb[12];
  R ka[12 * 585];
  R kb[12 * 235];
  R ka_mco2[12 * 9 * 19];
  R kb_mco2[12 * 19];
  R selfref[12 * 10];
  R forref[12 * 4];
};

template <class R>
struct Band08TablesOf {
  R fracrefa[8];
  R fracrefb[8];
  R ka[8 * 65];
  R kb[8 * 235];
  R ka_mco2[8 * 19];
  R kb_mco2[8 * 19];
  R ka_mn2o[8 * 19];
  R kb_mn2o[8 * 19];
  R ka_mo3[8 * 19];
  R cfc12[8];
  R cfc22adj[8];
  R selfref[8 * 10];
  R forref[8 * 4];
};

template <class R>
struct Band09TablesOf {
  R fracrefa[9 * 12];
  R fracrefb[12];
  R ka[12 * 585];
  R kb[12 * 235];
  R ka_mn2o[12 * 9 * 19];
  R kb_mn2o[12 * 19];
  R selfref[12 * 10];
  R forref[12 * 4];
};

template <class R>
struct Band10TablesOf {
  R fracrefa[6];
  R fracrefb[6];
  R ka[6 * 65];
  R kb[6 * 235];
  R selfref[6 * 10];
  R forref[6 * 4];
};

template <class R>
struct Band11TablesOf {
  R fracrefa[8];
  R fracrefb[8];
  R ka[8 * 65];
  R kb[8 * 235];
  R ka_mo2[8 * 19];
  R kb_mo2[8 * 19];
  R selfref[8 * 10];
  R forref[8 * 4];
};

template <class R>
struct Band12TablesOf {
  R fracrefa[9 * 8];
  R ka[8 * 585];
  R selfref[8 * 10];
  R forref[8 * 4];
};

template <class R>
struct Band13TablesOf {
  R fracrefa[9 * 4];
  R fracrefb[4];
  R ka[4 * 585];
  R ka_mco2[4 * 9 * 19];
  R ka_mco[4 * 9 * 19];
  R kb_mo3[4 * 19];
  R selfref[4 * 10];
  R forref[4 * 4];
};

template <class R>
struct Band14TablesOf {
  R fracrefa[2];
  R fracrefb[2];
  R ka[2 * 65];
  R kb[2 * 235];
  R selfref[2 * 10];
  R forref[2 * 4];
};

template <class R>
struct Band15TablesOf {
  R fracrefa[9 * 2];
  R ka[2 * 585];
  R ka_mn2[2 * 9 * 19];
  R selfref[2 * 10];
  R forref[2 * 4];
};

template <class R>
struct Band16TablesOf {
  R fracrefa[9 * 2];
  R fracrefb[2];
  R ka[2 * 585];
  R kb[2 * 235];
  R selfref[2 * 10];
  R forref[2 * 4];
};

template <class R>
struct RrtmgLwTablesOf {
  // Pade approximation tables (rrlw_tbl)
  R tau_tbl[10001];
  R exp_tbl[10001];
  R tfn_tbl[10001];

  // Spectral / Planck tables (rrlw_wvn)
  R totplnk[16 * 181];
  R totplk16[181];
  R delwave[16];
  R wavenum1[16];
  R wavenum2[16];

  int32_t ngc[16];
  int32_t ngs[16];
  int32_t ngn[140];
  int32_t ngb[140];
  int32_t ngm[256];
  R wt[16];
  R rwgt[256];

  // Cloud property tables (rrlw_cld)
  R abscld1;
  R absice0[2];
  R absice1[2 * 5];
  R absice2[43 * 16];
  R absice3[46 * 16];
  R absliq0;
  R absliq1[58 * 16];

  // Reference profiles (rrlw_ref)
  R preflog[59];
  R tref[59];
  R chi_mls[7 * 59];

  // 16 longwave absorption bands
  Band01TablesOf<R> b01;
  Band02TablesOf<R> b02;
  Band03TablesOf<R> b03;
  Band04TablesOf<R> b04;
  Band05TablesOf<R> b05;
  Band06TablesOf<R> b06;
  Band07TablesOf<R> b07;
  Band08TablesOf<R> b08;
  Band09TablesOf<R> b09;
  Band10TablesOf<R> b10;
  Band11TablesOf<R> b11;
  Band12TablesOf<R> b12;
  Band13TablesOf<R> b13;
  Band14TablesOf<R> b14;
  Band15TablesOf<R> b15;
  Band16TablesOf<R> b16;
};

#pragma pack(pop)

using RrtmgLwTables = RrtmgLwTablesOf<rb>;  // the tables in the build's real (WRF's rb)
// the dumps' sizes (data/rrtmg/README.md): packed in every compiler pass (g++, nvcc's host and device passes)
static_assert(sizeof(RrtmgLwTablesOf<double>) == 1435576 && sizeof(RrtmgLwTablesOf<float>) == 718924);
static_assert(alignof(RrtmgLwTablesOf<double>) == 1 && alignof(RrtmgLwTablesOf<float>) == 1);


// Loads the tables dump (tests/phys/ra_rrtmg_lw/dump_tables.f90, data/rrtmg/rrtmg_lw_tables.bin)
bool load_rrtmg_lw_tables(const std::string& filepath, RrtmgLwTables& tables);

}  // namespace cppwrf::phys
