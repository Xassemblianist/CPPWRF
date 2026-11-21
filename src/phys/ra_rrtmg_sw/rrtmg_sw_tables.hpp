// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "core/backend.hpp"
#include "phys/ra_rrtmg_common/parkind.hpp"
#include <cstdint>
#include <string>

namespace cppwrf::phys {

// The tables as packed structs of reals R (WRF's rb) and int32 members, in the order the dump program writes
// them (tests/phys/ra_rrtmg_sw/dump_sw_tables.f90): RrtmgSwTablesOf<double> is the layout of the WRF -r8 dump, RrtmgSwTablesOf<float> that of
// the WRF real*4 dump (data/rrtmg/README.md).
#pragma pack(push, 1)

template <class R>
struct SwBand16TablesOf {
  R ka[9 * 5 * 13 * 6];       // (9,5,13,6)
  R kb[5 * 47 * 6];           // (5,13:59,6)
  R selfref[10 * 6];          // (10,6)
  R forref[3 * 6];            // (3,6)
  R sfluxref[6];              // (6)
  R rayl;
  R strrat1;
  int32_t layreffr;
};

template <class R>
struct Band17TablesOf {
  R ka[9 * 5 * 13 * 12];      // (9,5,13,12)
  R kb[5 * 5 * 47 * 12];      // (5,5,13:59,12)
  R selfref[10 * 12];         // (10,12)
  R forref[4 * 12];           // (4,12)
  R sfluxref[12 * 5];         // (12,5)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band18TablesOf {
  R ka[9 * 5 * 13 * 8];       // (9,5,13,8)
  R kb[5 * 47 * 8];           // (5,13:59,8)
  R selfref[10 * 8];          // (10,8)
  R forref[3 * 8];            // (3,8)
  R sfluxref[8 * 9];          // (8,9)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band19TablesOf {
  R ka[9 * 5 * 13 * 8];       // (9,5,13,8)
  R kb[5 * 47 * 8];           // (5,13:59,8)
  R selfref[10 * 8];          // (10,8)
  R forref[3 * 8];            // (3,8)
  R sfluxref[8 * 9];          // (8,9)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band20TablesOf {
  R ka[5 * 13 * 10];          // (5,13,10)
  R kb[5 * 47 * 10];          // (5,13:59,10)
  R selfref[10 * 10];         // (10,10)
  R forref[4 * 10];           // (4,10)
  R sfluxref[10];             // (10)
  R absch4[10];               // (10)
  R rayl;
  int32_t layreffr;
};

template <class R>
struct Band21TablesOf {
  R ka[9 * 5 * 13 * 10];      // (9,5,13,10)
  R kb[5 * 5 * 47 * 10];      // (5,5,13:59,10)
  R selfref[10 * 10];         // (10,10)
  R forref[4 * 10];           // (4,10)
  R sfluxref[10 * 9];         // (10,9)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band22TablesOf {
  R ka[9 * 5 * 13 * 2];       // (9,5,13,2)
  R kb[5 * 47 * 2];           // (5,13:59,2)
  R selfref[10 * 2];          // (10,2)
  R forref[3 * 2];            // (3,2)
  R sfluxref[2 * 9];          // (2,9)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band23TablesOf {
  R ka[5 * 13 * 10];          // (5,13,10)
  R selfref[10 * 10];         // (10,10)
  R forref[3 * 10];           // (3,10)
  R sfluxref[10];             // (10)
  R rayl[10];                 // (10)
  R givfac;
  int32_t layreffr;
};

template <class R>
struct Band24TablesOf {
  R ka[9 * 5 * 13 * 8];       // (9,5,13,8)
  R kb[5 * 47 * 8];           // (5,13:59,8)
  R selfref[10 * 8];          // (10,8)
  R forref[3 * 8];            // (3,8)
  R sfluxref[8 * 9];          // (8,9)
  R abso3a[8];                // (8)
  R abso3b[8];                // (8)
  R rayla[8 * 9];             // (8,9)
  R raylb[8];                 // (8)
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band25TablesOf {
  R ka[5 * 13 * 6];           // (5,13,6)
  R sfluxref[6];              // (6)
  R abso3a[6];                // (6)
  R abso3b[6];                // (6)
  R rayl[6];                  // (6)
  int32_t layreffr;
};

template <class R>
struct Band26TablesOf {
  R sfluxref[6];              // (6)
  R rayl[6];                  // (6)
};

template <class R>
struct Band27TablesOf {
  R ka[5 * 13 * 8];           // (5,13,8)
  R kb[5 * 47 * 8];           // (5,13:59,8)
  R sfluxref[8];              // (8)
  R rayl[8];                  // (8)
  R scalekur;
  int32_t layreffr;
};

template <class R>
struct Band28TablesOf {
  R ka[9 * 5 * 13 * 6];       // (9,5,13,6)
  R kb[5 * 5 * 47 * 6];       // (5,5,13:59,6)
  R sfluxref[6 * 5];          // (6,5)
  R rayl;
  R strrat;
  int32_t layreffr;
};

template <class R>
struct Band29TablesOf {
  R ka[5 * 13 * 12];          // (5,13,12)
  R kb[5 * 47 * 12];          // (5,13:59,12)
  R selfref[10 * 12];         // (10,12)
  R forref[4 * 12];           // (4,12)
  R sfluxref[12];             // (12)
  R absh2o[12];               // (12)
  R absco2[12];               // (12)
  R rayl;
  int32_t layreffr;
};

template <class R>
struct RrtmgSwTablesOf {
  // 1. Pade approximation lookup table (rrsw_tbl)
  R exp_tbl[10001];

  // 2. Spectral info (rrsw_wvn)
  R wavenum1[14];
  R wavenum2[14];
  R delwave[14];
  int32_t ngc[14];
  int32_t ngs[14];
  int32_t ngn[112];
  int32_t ngb[112];
  int32_t ngm[224];
  R wt[16];
  R rwgt[224];

  // 3. Cloud property tables (rrsw_cld)
  R extliq1[58 * 14];
  R ssaliq1[58 * 14];
  R asyliq1[58 * 14];
  R extice2[43 * 14];
  R ssaice2[43 * 14];
  R asyice2[43 * 14];
  R extice3[46 * 14];
  R ssaice3[46 * 14];
  R asyice3[46 * 14];
  R fdlice3[46 * 14];
  R abari[5];
  R bbari[5];
  R cbari[5];
  R dbari[5];
  R ebari[5];
  R fbari[5];

  // 4. Reference profiles (rrsw_ref)
  R preflog[59];
  R tref[59];

  // 5. Aerosol properties (rrsw_aer)
  R rsrtaua[14 * 6];
  R rsrpiza[14 * 6];
  R rsrasya[14 * 6];

  // 6. 14 shortwave absorption bands
  SwBand16TablesOf<R> b16;
  Band17TablesOf<R> b17;
  Band18TablesOf<R> b18;
  Band19TablesOf<R> b19;
  Band20TablesOf<R> b20;
  Band21TablesOf<R> b21;
  Band22TablesOf<R> b22;
  Band23TablesOf<R> b23;
  Band24TablesOf<R> b24;
  Band25TablesOf<R> b25;
  Band26TablesOf<R> b26;
  Band27TablesOf<R> b27;
  Band28TablesOf<R> b28;
  Band29TablesOf<R> b29;
};

#pragma pack(pop)

using RrtmgSwTables = RrtmgSwTablesOf<rb>;  // the tables in the build's real (WRF's rb)
// the dumps' sizes (data/rrtmg/README.md): packed in every compiler pass (g++, nvcc's host and device passes)
static_assert(sizeof(RrtmgSwTablesOf<double>) == 841044 && sizeof(RrtmgSwTablesOf<float>) == 421500);
static_assert(alignof(RrtmgSwTablesOf<double>) == 1 && alignof(RrtmgSwTablesOf<float>) == 1);


// Loads the tables dump (tests/phys/ra_rrtmg_sw/dump_sw_tables.f90, data/rrtmg/rrtmg_sw_tables.bin)
bool load_rrtmg_sw_tables(const std::string& filepath, RrtmgSwTables& tables);

}  // namespace cppwrf::phys
