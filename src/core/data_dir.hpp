// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The directory of CPPWRF's data (the RRTMG tables, the WRF I/O schema): $CPPWRF_DATA_DIR, else the share/cppwrf/data
// of an installed binary (<prefix>/bin/../share/cppwrf/data), else the source tree's data/ (CPPWRF_DATA_DIR at build
// time).
#include <cstdlib>
#include <filesystem>
#include <string>

namespace cppwrf {

inline std::string data_dir() {
  if (const char* e = std::getenv("CPPWRF_DATA_DIR"); e && *e) return e;
  std::error_code ec;
  const std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (!ec) {
    const std::filesystem::path d = exe.parent_path().parent_path() / "share" / "cppwrf" / "data";
    if (std::filesystem::exists(d / "wrf_schema", ec)) return d.string();
  }
#ifdef CPPWRF_DATA_DIR
  return CPPWRF_DATA_DIR;
#else
  return "data";
#endif
}

}  // namespace cppwrf
