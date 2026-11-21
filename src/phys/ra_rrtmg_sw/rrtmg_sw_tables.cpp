// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/ra_rrtmg_sw/rrtmg_sw_tables.hpp"

#include <cstddef>
#include <iostream>

#include "phys/ra_rrtmg_common/table_dump.hpp"

namespace cppwrf::phys {

namespace {

// the int32 members of RrtmgSwTablesOf<R>: ngc .. ngm (one run before wt), then layreffr of every band but 26
template <class R>
constexpr std::array<TableBytes, 14> sw_table_ints() {
  using T = RrtmgSwTablesOf<R>;
  constexpr std::size_t i4 = sizeof(int32_t);
  return {TableBytes{offsetof(T, ngc), offsetof(T, wt)},
          table_member(offsetof(T, b16), offsetof(SwBand16TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b17), offsetof(Band17TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b18), offsetof(Band18TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b19), offsetof(Band19TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b20), offsetof(Band20TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b21), offsetof(Band21TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b22), offsetof(Band22TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b23), offsetof(Band23TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b24), offsetof(Band24TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b25), offsetof(Band25TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b27), offsetof(Band27TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b28), offsetof(Band28TablesOf<R>, layreffr), i4),
          table_member(offsetof(T, b29), offsetof(Band29TablesOf<R>, layreffr), i4)};
}

}  // namespace

bool load_rrtmg_sw_tables(const std::string& filepath, RrtmgSwTables& tables) {
  if (read_table_dump<RrtmgSwTablesOf<double>>(filepath, tables, sw_table_ints<double>(), sw_table_ints<rb>()))
    return true;
  std::cerr << "Error: could not read rrtmg_sw_tables.bin from " << filepath << "\n";
  return false;
}

}  // namespace cppwrf::phys
