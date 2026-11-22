// SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG
#include "phys/ra_rrtmg_lw/rrtmg_lw_tables.hpp"

#include <cstddef>
#include <iostream>

#include "phys/ra_rrtmg_common/table_dump.hpp"

namespace cppwrf::phys {

namespace {

// the int32 members of RrtmgLwTablesOf<R>: ngc .. ngm, one run before wt
template <class R>
constexpr std::array<TableBytes, 1> lw_table_ints() {
  using T = RrtmgLwTablesOf<R>;
  return {TableBytes{offsetof(T, ngc), offsetof(T, wt)}};
}

}  // namespace

bool load_rrtmg_lw_tables(const std::string& filepath, RrtmgLwTables& tables) {
  if (read_table_dump<RrtmgLwTablesOf<double>>(filepath, tables, lw_table_ints<double>(), lw_table_ints<rb>()))
    return true;
  std::cerr << "Error: could not read rrtmg_lw_tables.bin from " << filepath << "\n";
  return false;
}

}  // namespace cppwrf::phys
