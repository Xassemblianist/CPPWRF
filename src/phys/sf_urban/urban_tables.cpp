// SPDX-License-Identifier: GPL-3.0-or-later
#include "phys/sf_urban/urban_tables.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "core/libm.hpp"  // the Macdonald lengths with the kernels' pow / exp (glibc's in the CUDA FP64 build)

namespace cppwrf::phys::urban {
namespace {

std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r");
  if (a == std::string::npos) return "";
  const auto b = s.find_last_not_of(" \t\r");
  return s.substr(a, b - a + 1);
}

// Fortran list-directed read of n values from one record (separators: blanks and commas; r*c repeats);
// fewer values than n: WRF's internal read hits the end of the record and stops
std::vector<double> list_read(const std::string& rec, std::size_t n, const std::string& what) {
  std::vector<double> v;
  std::string tok;
  auto flush = [&]() {
    if (tok.empty()) return;
    const auto star = tok.find('*');
    if (star != std::string::npos) {
      const long r = std::strtol(tok.substr(0, star).c_str(), nullptr, 10);
      const double c = std::strtod(tok.substr(star + 1).c_str(), nullptr);
      for (long k = 0; k < r; ++k) v.push_back(c);
    } else {
      char* end = nullptr;
      const double c = std::strtod(tok.c_str(), &end);
      if (end == tok.c_str() || *end != '\0')
        throw std::runtime_error("URBPARM: cannot read '" + tok + "' of " + what);
      v.push_back(c);
    }
    tok.clear();
  };
  for (char ch : rec) {
    if (v.size() >= n) break;
    if (ch == '/') { flush(); break; }  // list-directed read: a slash ends the record
    if (ch == ',' || ch == ' ' || ch == '\t' || ch == '\r') flush();
    else tok += ch;
  }
  flush();
  if (v.size() < n) throw std::runtime_error("URBPARM: " + what + " has fewer than " + std::to_string(n) + " values");
  v.resize(n);
  return v;
}

}  // namespace

void read_urban_table(const std::string& path, int nl, int sf_urban_physics, UrbanTables& t, UrbanTableOptions& opt) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Error opening " + path + ". Make sure the URBPARM table (found in run/) is linked "
                                   "to the running directory.");
  if (nl < 2 || nl > kMaxUrbLayers) throw std::runtime_error("urban: num_soil_layers out of range");
  t = UrbanTables{};
  t.nlayers = nl;
  int icate = 0;
  std::vector<double> roof_width, road_width;
  std::map<std::string, std::vector<double>> per;  // name -> icate values
  std::vector<int> numhgt;
  std::vector<std::vector<double>> hpct;
  bool have_dzr = false, have_dzb = false, have_dzg = false;
  std::string s;
  auto need = [&](const std::string& name) {
    if (icate <= 0) throw std::runtime_error("URBPARM: " + name + " before 'Number of urban categories'");
  };
  while (std::getline(f, s)) {
    if (!s.empty() && s.back() == '\r') s.pop_back();
    if (s.size() > 512) s.resize(512);  // read(11,'(A512)')
    if (!s.empty() && s[0] == '#') continue;
    if (trim(s).empty()) continue;
    const auto indx = s.find(':');
    if (indx == std::string::npos) continue;
    const std::string name = trim(s.substr(0, indx));
    const std::string rest = s.substr(indx + 1);
    auto cat = [&](const char* key) {
      need(key);
      per[key] = list_read(rest, std::size_t(icate), key);
    };
    auto ival = [&](const char* key) { return int(std::lround(list_read(rest, 1, key)[0])); };
    if (name == "Number of urban categories") {
      icate = ival("Number of urban categories");
      if (icate < 1 || icate > kMaxUrbCat) throw std::runtime_error("URBPARM: number of urban categories out of range");
      numhgt.assign(std::size_t(icate), 0);
      hpct.assign(std::size_t(icate), {});
    } else if (name == "ROOF_WIDTH") {
      need(name);
      roof_width = list_read(rest, std::size_t(icate), name);
    } else if (name == "ROAD_WIDTH") {
      need(name);
      road_width = list_read(rest, std::size_t(icate), name);
    } else if (name == "ZR" || name == "SIGMA_ZED" || name == "AH" || name == "ALH" || name == "FRC_URB" ||
               name == "CAPR" || name == "CAPB" || name == "CAPG" || name == "AKSR" || name == "AKSB" ||
               name == "AKSG" || name == "ALBR" || name == "ALBB" || name == "ALBG" || name == "EPSR" ||
               name == "EPSB" || name == "EPSG" || name == "AKANDA_URBAN" || name == "Z0B" || name == "Z0G" ||
               name == "Z0R" || name == "TRLEND" || name == "TBLEND" || name == "TGLEND" || name == "FVG" ||
               name == "RTREE" || name == "HTREE" || name == "DTREE" || name == "LAI_TREE" || name == "LAI_VEG" ||
               name == "Z0VG" || name == "Z0HVG" || name == "CAPVG" || name == "EPSVG" || name == "EPST" ||
               name == "ALBVG" || name == "ALBT" || name == "COP" || name == "BLDAC_FRC" || name == "COOLED_FRC" ||
               name == "PWIN" || name == "BETA" || name == "SW_COND" || name == "TIME_ON" || name == "TIME_OFF" ||
               name == "TARGTEMP" || name == "GAPTEMP" || name == "TARGHUM" || name == "GAPHUM" ||
               name == "PERFLO" || name == "HSEQUIP_SCALE_FACTOR" || name == "PV_FRAC_ROOF" ||
               name == "GR_FRAC_ROOF") {
      cat(name.c_str());
    } else if (name == "DDZR" || name == "DDZB" || name == "DDZG") {
      // below
    } else if (name == "BOUNDR") {
      opt.boundr = ival("BOUNDR"); opt.has_boundr = true;
    } else if (name == "BOUNDB") {
      opt.boundb = ival("BOUNDB"); opt.has_boundb = true;
    } else if (name == "BOUNDG") {
      opt.boundg = ival("BOUNDG"); opt.has_boundg = true;
    } else if (name == "CH_SCHEME") {
      opt.ch_scheme = ival("CH_SCHEME"); opt.has_ch = true;
    } else if (name == "TS_SCHEME") {
      opt.ts_scheme = ival("TS_SCHEME"); opt.has_ts = true;
    } else if (name == "AHOPTION") {
      opt.ahoption = ival("AHOPTION");
    } else if (name == "AHDIUPRF") {
      list_read(rest, 24, name);
    } else if (name == "ALHOPTION") {
      opt.alhoption = ival("ALHOPTION");
    } else if (name == "ALHSEASON") {
      list_read(rest, 4, name);
    } else if (name == "ALHDIUPRF") {
      list_read(rest, 48, name);
    } else if (name == "PORIMP" || name == "DENGIMP") {
      list_read(rest, 3, name);
    } else if (name == "IMP_SCHEME") {
      opt.imp_scheme = ival("IMP_SCHEME"); opt.has_imp = true;
    } else if (name == "IRI_SCHEME") {
      opt.iri_scheme = ival("IRI_SCHEME");
    } else if (name == "OASIS") {
      opt.oasis = list_read(rest, 1, name)[0];
    } else if (name == "GROPTION") {
      opt.groption = ival("GROPTION");
    } else if (name == "FGR") {
      opt.fgr = list_read(rest, 1, name)[0];
    } else if (name == "DZGR") {
      list_read(rest, 4, name);
    } else if (name == "TREEOPTION") {
      opt.treeoption = ival("TREEOPTION");
    } else if (name == "TTscheme") {
      opt.ttscheme = ival("TTscheme");
    } else if (name == "HSEQUIP" || name == "IRHO") {
      list_read(rest, 24, name);
    } else if (name == "GR_FLAG" || name == "GR_TYPE") {
      ival(name.c_str());
    } else if (name == "STREET PARAMETERS") {
      need(name);
      while (std::getline(f, s)) {
        if (!s.empty() && s.back() == '\r') s.pop_back();
        if (!s.empty() && s[0] == '#') continue;
        if (trim(s).empty()) continue;
        if (s == "END STREET PARAMETERS") break;
        const std::vector<double> v = list_read(s, 4, "STREET PARAMETERS");
        const int k = int(std::lround(v[0]));
        if (k < 1 || k > icate) throw std::runtime_error("URBPARM: STREET PARAMETERS category out of range");
      }
    } else if (name == "BUILDING HEIGHTS") {
      need(name);
      const int k = ival("BUILDING HEIGHTS");
      if (k < 1 || k > icate) throw std::runtime_error("URBPARM: BUILDING HEIGHTS category out of range");
      while (std::getline(f, s)) {
        if (!s.empty() && s.back() == '\r') s.pop_back();
        if (!s.empty() && s[0] == '#') continue;
        if (trim(s).empty()) continue;
        if (s == "END BUILDING HEIGHTS") break;
        const std::vector<double> v = list_read(s, 2, "BUILDING HEIGHTS");
        hpct[std::size_t(k - 1)].push_back(v[1]);
      }
      // pctsum = sum(hpercent_bin_tbl(:,k), mask=(hpercent_bin_tbl(:,k) > -1.E25)): Fortran SUM in order
      real pctsum = real(0);
      for (double p : hpct[std::size_t(k - 1)])
        if (real(p) > real(-1.E25)) pctsum = pctsum + real(p);
      if (pctsum != real(100.))
        throw std::runtime_error("pctsum is not equal to 100. (building height percentages for category " +
                                 std::to_string(k) + ")");
    } else {
      throw std::runtime_error("URBPARM.TBL: Unrecognized NAME = \"" + name + "\" in Subr URBAN_PARAM_INIT");
    }
    if (name == "DDZR" || name == "DDZB" || name == "DDZG") {
      const std::vector<double> v = list_read(rest, std::size_t(nl), name);
      real* dst = name == "DDZR" ? t.dzr : name == "DDZB" ? t.dzb : t.dzg;
      for (int k = 0; k < nl; ++k) dst[k] = real(v[std::size_t(k)]) * real(100.0);  // m -> cm
      (name == "DDZR" ? have_dzr : name == "DDZB" ? have_dzb : have_dzg) = true;
    }
  }
  if (icate <= 0) throw std::runtime_error(path + ": no 'Number of urban categories'");
  if (roof_width.empty() || road_width.empty())
    throw std::runtime_error(path + ": ROOF_WIDTH and ROAD_WIDTH are needed (WRF deallocates them unallocated)");
  if (!have_dzr || !have_dzb || !have_dzg) throw std::runtime_error(path + ": DDZR, DDZB and DDZG are needed");
  for (const char* k : {"ZR", "SIGMA_ZED", "AH", "ALH", "FRC_URB", "CAPR", "CAPB", "CAPG", "AKSR", "AKSB", "AKSG",
                        "ALBR", "ALBB", "ALBG", "EPSR", "EPSB", "EPSG", "AKANDA_URBAN", "Z0B", "Z0G", "TRLEND",
                        "TBLEND", "TGLEND"})
    if (!per.count(k)) throw std::runtime_error(path + ": " + k + " is missing (WRF uses an undefined value)");
  if (sf_urban_physics != 1 && !per.count("Z0R")) throw std::runtime_error(path + ": Z0R is missing");
  t.icate = icate;
  auto P = [&](const char* k, int lc) { return real(per[k][std::size_t(lc)]); };
  constexpr real vonk = real(0.4);
  for (int lc = 0; lc < icate; ++lc) {
    t.zr[lc] = P("ZR", lc);
    t.sigma_zed[lc] = P("SIGMA_ZED", lc);
    t.ah[lc] = P("AH", lc);
    t.alh[lc] = P("ALH", lc);
    t.frc_urb[lc] = P("FRC_URB", lc);
    // the unit conversions of urban_param_init (whole-array statements after each read)
    t.capr[lc] = P("CAPR", lc) * (real(1.0) / real(4.1868)) * real(1.E-6);
    t.capb[lc] = P("CAPB", lc) * (real(1.0) / real(4.1868)) * real(1.E-6);
    t.capg[lc] = P("CAPG", lc) * (real(1.0) / real(4.1868)) * real(1.E-6);
    t.aksr[lc] = P("AKSR", lc) * (real(1.0) / real(4.1868)) * real(1.E-2);
    t.aksb[lc] = P("AKSB", lc) * (real(1.0) / real(4.1868)) * real(1.E-2);
    t.aksg[lc] = P("AKSG", lc) * (real(1.0) / real(4.1868)) * real(1.E-2);
    t.albr[lc] = P("ALBR", lc);
    t.albb[lc] = P("ALBB", lc);
    t.albg[lc] = P("ALBG", lc);
    t.epsr[lc] = P("EPSR", lc);
    t.epsb[lc] = P("EPSB", lc);
    t.epsg[lc] = P("EPSG", lc);
    t.akanda_urban[lc] = P("AKANDA_URBAN", lc);
    t.z0b[lc] = P("Z0B", lc);
    t.z0g[lc] = P("Z0G", lc);
    t.z0r[lc] = per.count("Z0R") ? P("Z0R", lc) : real(0);
    t.trlend[lc] = P("TRLEND", lc);
    t.tblend[lc] = P("TBLEND", lc);
    t.tglend[lc] = P("TGLEND", lc);
    t.z0hb[lc] = real(0.1) * t.z0b[lc];
    t.z0hg[lc] = real(0.1) * t.z0g[lc];
    const real roof = real(roof_width[std::size_t(lc)]), road = real(road_width[std::size_t(lc)]);
    t.hgt[lc] = t.zr[lc] / (road + roof);
    t.r[lc] = roof / (road + roof);
    t.rw[lc] = real(1.0) - t.r[lc];
    t.betr[lc] = real(0.0);
    t.betb[lc] = real(0.0);
    t.betg[lc] = real(0.0);
    const real lambda_p = t.r[lc], lambda_f = t.hgt[lc];
    const real cd = real(1.2), alpha_macd = real(4.43), beta_macd = real(1.0);
    t.zdc[lc] = t.zr[lc] * (real(1.0) + (libm::pow(alpha_macd, -lambda_p)) * (lambda_p - real(1.0)));
    t.z0c[lc] = t.zr[lc] * (real(1.0) - t.zdc[lc] / t.zr[lc]) *
                libm::exp(-libm::pow(real(0.5) * beta_macd * cd / (vonk * vonk) * (real(1.0) - t.zdc[lc] / t.zr[lc]) *
                                       lambda_f,
                                   real(-0.5)));
    if (sf_urban_physics == 1) {  // roof height variability in Macdonald: Z0R from ZR_SD
      const real lambda_fr = t.sigma_zed[lc] / (road + roof);
      t.z0r[lc] = t.zr[lc] * (real(1.0) - t.zdc[lc] / t.zr[lc]) *
                  libm::exp(-libm::pow(real(0.5) * beta_macd * cd / (vonk * vonk) *
                                         (real(1.0) - t.zdc[lc] / t.zr[lc]) * lambda_fr,
                                     real(-0.5)));
    }
    t.z0hc[lc] = real(0.1) * t.z0c[lc];
    // sky view factor
    const real dhgt = t.hgt[lc] / real(100.);
    real hgt = t.hgt[lc] - dhgt / real(2.);
    real vfws = real(0.);
    for (int k = 1; k <= 99; ++k) {
      hgt = hgt - dhgt;
      vfws = vfws + real(0.25) * (real(1.) - hgt / std::sqrt(hgt * hgt + t.rw[lc] * t.rw[lc]));
    }
    vfws = vfws / real(99.);
    vfws = vfws * real(2.);
    t.svf[lc] = real(1.) - real(2.) * vfws * t.hgt[lc] / t.rw[lc];
  }
}

void check_urban_table_options(const UrbanTableOptions& o, const std::string& path) {
  auto bad = [&](const std::string& what) {
    throw std::runtime_error(path + ": " + what +
                             " is not ported to CPPWRF (sf_urban_physics = 1 supports the default URBPARM options "
                             "CH_SCHEME 2, TS_SCHEME 1, IMP_SCHEME 1, IRI_SCHEME 0, AHOPTION 0, ALHOPTION 0, "
                             "GROPTION 0, TREEOPTION 0, BOUNDR/BOUNDB/BOUNDG 1, OASIS 1.0)");
  };
  if (!o.has_ch || o.ch_scheme != 2) bad("CH_SCHEME = " + std::to_string(o.ch_scheme));
  if (!o.has_ts || o.ts_scheme != 1) bad("TS_SCHEME = " + std::to_string(o.ts_scheme));
  if (!o.has_imp || o.imp_scheme != 1) bad("IMP_SCHEME = " + std::to_string(o.imp_scheme));
  if (o.iri_scheme != 0) bad("IRI_SCHEME = " + std::to_string(o.iri_scheme));
  if (o.ahoption != 0) bad("AHOPTION = " + std::to_string(o.ahoption));
  if (o.alhoption != 0) bad("ALHOPTION = " + std::to_string(o.alhoption));
  if (o.groption != 0) bad("GROPTION = " + std::to_string(o.groption));
  if (o.treeoption != 0) bad("TREEOPTION = " + std::to_string(o.treeoption));
  if (!o.has_boundr || !o.has_boundb || !o.has_boundg || o.boundr != 1 || o.boundb != 1 || o.boundg != 1)
    bad("BOUNDR/BOUNDB/BOUNDG other than 1");
  if (o.oasis != 1.0) bad("OASIS other than 1.0");
}

}  // namespace cppwrf::phys::urban
