// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/lsm_ruc/ruc_tables.hpp"
#include "core/parse_real.hpp"

#include <cctype>
#include <fstream>
#include <vector>

namespace cppwrf::phys::ruc {

namespace {

std::vector<std::string> tokens(const std::string& line) {  // list-directed: blanks and commas separate
  std::vector<std::string> v;
  std::string cur;
  for (char c : line) {
    if (c == ',' || std::isspace(static_cast<unsigned char>(c))) {
      if (!cur.empty()) v.push_back(cur), cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) v.push_back(cur);
  return v;
}

std::string a8(const std::string& line) {  // FORMAT (A8), trailing blanks dropped
  std::string s = line.substr(0, 8);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\r')) s.pop_back();
  return s;
}

std::string trimmed(std::string s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
  std::size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  return s.substr(b);
}

}  // namespace

std::string read_ruc_tables(const std::string& table_dir, const std::string& mminlu_in, RucTables& t) {
  t = RucTables{};
  std::string mminlu = trimmed(mminlu_in);
  std::string want;
  if (mminlu == "USGS") want = "USGS-RUC";
  else if (mminlu == "MODIS" || mminlu == "MODIFIED_IGBP_MODIS_NOAH") want = "MODI-RUC";
  else return "RUC LSM: MMINLU '" + mminlu + "' has no RUC table (WRF: USGS, MODIS or MODIFIED_IGBP_MODIS_NOAH)";
  const std::string dir = table_dir.empty() ? std::string(".") : table_dir;
  auto num = [](const std::string& s) { return sto_kind(s); };  // READ into REAL: one rounding
  try {
    std::ifstream f(dir + "/VEGPARM.TBL");
    if (!f) return "module_sf_ruclsm.F: soil_veg_gen_parm: failure opening VEGPARM.TBL";
    std::string line;
    std::getline(f, line);
    bool found = false;
    while (std::getline(f, line)) {
      const std::string lutype = a8(line);
      if (!std::getline(f, line)) break;
      auto tk = tokens(line);
      if (tk.empty()) break;
      const int lucats = std::stoi(tk[0]);
      if (lutype != want) {
        for (int lc = 0; lc < lucats; ++lc) std::getline(f, line);
        bool next = false;
        while (std::getline(f, line))
          if (trimmed(line) == "Vegetation Parameters") { next = true; break; }
        if (!next) break;
        continue;
      }
      found = true;
      if (lucats > kNlus) return "RUC LSM: too many land-use categories in VEGPARM.TBL";
      t.lucats = lucats;
      for (int lc = 0; lc < lucats; ++lc) {
        if (!std::getline(f, line)) return "RUC LSM: VEGPARM.TBL ends inside the " + want + " table";
        auto v = tokens(line);
        if (v.size() < 13) return "RUC LSM: short line in the " + want + " table of VEGPARM.TBL";
        t.albtbl[lc] = num(v[1]); t.z0tbl[lc] = num(v[2]); t.lemitbl[lc] = num(v[3]); t.pctbl[lc] = num(v[4]);
        t.shdtbl[lc] = num(v[5]); t.ifortbl[lc] = std::stoi(v[6]); t.rstbl[lc] = num(v[7]); t.rgltbl[lc] = num(v[8]);
        t.hstbl[lc] = num(v[9]); t.snuptbl[lc] = num(v[10]); t.laitbl[lc] = num(v[11]); t.maxalb[lc] = num(v[12]);
      }
      auto value = [&](std::string& out) {  // READ (19,*) then READ (19,*) X
        std::getline(f, line);
        if (!std::getline(f, line)) return false;
        auto v = tokens(line);
        if (v.empty()) return false;
        out = v[0];
        return true;
      };
      std::string s;
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks TOPT_DATA";
      t.topt_data = num(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks CMCMAX_DATA";
      t.cmcmax_data = num(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks CFACTR_DATA";
      t.cfactr_data = num(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks RSMAX_DATA";
      t.rsmax_data = num(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks BARE";
      t.bare = std::stoi(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks NATURAL";
      t.natural = std::stoi(s);
      if (!value(s)) return "RUC LSM: VEGPARM.TBL lacks CROP";
      t.crop = std::stoi(s);
      if (!value(s)) return "Problem read URBAN from VEGPARM.TBL";
      t.urban = std::stoi(s);
      break;
    }
    if (!found) return "Land Use Dataset '" + want + "' not found in VEGPARM.TBL.";

    std::ifstream fs(dir + "/SOILPARM.TBL");
    if (!fs) return "module_sf_ruclsm.F: soil_veg_gen_parm: failure opening SOILPARM.TBL";
    bool sfound = false;
    for (int sec = 0; sec < 2 && !sfound; ++sec) {  // WRF looks at the first two sections only
      std::getline(fs, line);
      if (!std::getline(fs, line)) break;
      const std::string sltype = a8(line);
      if (!std::getline(fs, line)) break;
      auto tk = tokens(line);
      if (tk.empty()) break;
      const int slcats = std::stoi(tk[0]);
      if (slcats > kNsltype) return "RUC LSM: too many soil categories in SOILPARM.TBL";
      const bool match = sltype == "STAS-RUC";
      for (int lc = 0; lc < slcats; ++lc) {
        if (!std::getline(fs, line)) return "RUC LSM: SOILPARM.TBL ends inside a table";
        if (!match) continue;
        auto v = tokens(line);
        if (v.size() < 11) return "RUC LSM: short line in the STAS-RUC table of SOILPARM.TBL";
        t.bb[lc] = num(v[1]); t.drysmc[lc] = num(v[2]); t.hc[lc] = num(v[3]); t.maxsmc[lc] = num(v[4]);
        t.refsmc[lc] = num(v[5]); t.satpsi[lc] = num(v[6]); t.satdk[lc] = num(v[7]); t.satdw[lc] = num(v[8]);
        t.wltsmc[lc] = num(v[9]); t.qtz[lc] = num(v[10]);
      }
      if (match) {
        sfound = true;
        t.slcats = slcats;
      }
    }
    if (!sfound) return "INCONSISTENT OR MISSING SOILPARM FILE";
    std::ifstream fg(dir + "/GENPARM.TBL");
    if (!fg) return "module_sf_ruclsm.F: soil_veg_gen_parm: failure opening GENPARM.TBL";
  } catch (const std::exception& e) {
    return std::string("RUC LSM: cannot parse the tables: ") + e.what();
  }
  make_tbq(t.tbq);
  return "";
}

}  // namespace cppwrf::phys::ruc
