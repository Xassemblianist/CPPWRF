// SPDX-License-Identifier: GPL-3.0-or-later
// The WRF output schema (src/wrf_io): what WRF 4.8.0's check_nml_consistency derives from the namelist
// (wrf_io::derive_config), which variables a stream holds (WrfSchema::stream_vars with iofields_filename), and the
// global attribute MMINLU. Each check names the WRF source it follows.
#include "wrf_io/global_atts.hpp"
#include "wrf_io/namelist.hpp"
#include "wrf_io/nc_out.hpp"
#include "wrf_io/wrf_schema.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#ifdef CPPWRF_HAVE_NETCDF
#include <netcdf.h>
#endif

using namespace cppwrf::wrf_io;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
  std::cout << (ok ? "ok   " : "FAIL ") << what << std::endl;
  if (!ok) ++failures;
}

const WrfSchema& schema() {
  static const WrfSchema s = WrfSchema::load("data/wrf_schema/wrf-4.8.0.tsv");
  return s;
}

// the namelist of domain `dom` after derive_config, as wrf.exe has it
struct Run {
  Namelist nl;
  WrfConfig c;
  Run(const std::string& text, int dom)
      : nl(Namelist::parse(text)), c(nl, schema().defaults(), dom, &schema().entries()) {
    derive_config(c, std::max(1, c.integer("max_dom")), false);
  }
};

bool has(const std::vector<OutVar>& v, const std::string& name) {
  return std::any_of(v.begin(), v.end(), [&](const OutVar& o) { return o.name == name; });
}

std::string iofields_file(const std::string& name, const std::string& text) {
  const std::filesystem::path p = std::filesystem::temp_directory_path() / name;
  std::ofstream(p) << text;
  return p.string();
}

}  // namespace

int main() {
  // check_nml_consistency (module_check_a_mundo.F:753-943, after setup_physics_suite): the cross-domain physics checks
  // real.exe and wrf.exe stop on (cppwrf_real: physics_domain_consistency)
  {
    auto cons = [](const std::string& physics, const std::string& dynamics = "") {
      const Namelist nl = Namelist::parse("&domains\n max_dom = 2,\n/\n&physics\n" + physics + "\n/\n&dynamics\n" +
                                          dynamics + "\n/\n");
      const WrfConfig c(nl, schema().defaults(), 1, &schema().entries());
      return physics_domain_consistency(c, 2);
    };
    const std::string ok = "sf_surface_physics = 2, 2, sf_sfclay_physics = 1, 1, ra_lw_physics = 4, 4, "
                           "ra_sw_physics = 4, 4, bl_pbl_physics = 1, 0, cu_physics = 1, 0,";
    check(cons(ok, "gwd_opt = 1, 0,").empty(), "equal physics, nest PBL / cumulus / GWDO 0: consistent");
    check(cons("physics_suite = 'CONUS', sf_surface_physics = -1, 2, cu_physics = -1, 6,").empty(),
          "physics_suite CONUS: -1 is the suite's scheme on every domain");
    const std::pair<std::string, std::string> bad[] = {
        {"sf_surface_physics = 2, 4,", "sf_surface_physics must be equal for all domains"},
        {"sf_sfclay_physics = 1, 2,", "sf_sfclay_physics must be equal for all domains"},
        {"ra_lw_physics = 4, 1,", "ra_lw_physics must be equal for all domains"},
        {"ra_sw_physics = 4, 1,", "ra_sw_physics must be equal for all domains"},
        {"bl_pbl_physics = 1, 2,", "bl_pbl_physics must be equal for all domains (or = zero)"},
        {"cu_physics = 1, 6,", "cu_physics must be equal for all domains (or = zero)"},
        {"cu_physics = 0, 1,", "cu_physics must be equal for all domains (or = zero)"}};
    for (const auto& [p, msg] : bad) check(cons(p) == msg, p + " -> " + msg);
    check(cons("", "gwd_opt = 1, 3,") == "gwd_opt must be equal for all domains (or = zero)", "gwd_opt = 1, 3: fatal");
  }
  // module_check_a_mundo.F:1852-1890: cu_used / gwd_used when any domain uses the scheme; a nest without cumulus
  // or GWDO keeps the cumulus tendencies (restart) and the GWDO fields (history, restart)
  {
    Run r("&domains\n max_dom = 2,\n/\n&physics\n cu_physics = 1, 0,\n/\n&dynamics\n gwd_opt = 1, 0,\n/\n", 2);
    check(r.c.integer("cu_used") == 1, "cu_physics = 1, 0: cu_used = 1 on d02");
    check(r.c.integer("gwd_used") == 1, "gwd_opt = 1, 0: gwd_used = 1 on d02");
    const auto h = schema().history_vars(r.c, 41, 41, 31, 8);
    const auto rst = schema().restart_vars(r.c, 41, 41, 31, 8);
    check(has(h, "OA1") && has(h, "DTAUX3D") && has(rst, "ELVMAX"), "d02 history / restart hold the GWDO fields");
    check(has(rst, "RTHCUTEN") && has(rst, "RUCUTEN"), "d02 restart holds the cumulus tendencies");
    Run n("&domains\n max_dom = 2,\n/\n&physics\n cu_physics = 0, 0,\n/\n&dynamics\n gwd_opt = 0, 0,\n/\n", 2);
    check(n.c.integer("cu_used") == 0 && n.c.integer("gwd_used") == 0, "no domain with cumulus / GWDO: unused");
  }
  // check_nml_consistency (module_check_a_mundo.F:974-992, after setup_physics_suite): ghg_input = 0 when a domain
  // has no shortwave; d01's own resolved suite value does not stand for d02's entry
  {
    const std::string nl = "&domains\n max_dom = 2,\n/\n&physics\n physics_suite = 'CONUS',\n ra_sw_physics = -1, 0,\n"
                           " ghg_input = 1,\n/\n";
    Run d1(nl, 1), d2(nl, 2);
    check(d1.c.integer("ra_sw_physics") == 4, "CONUS: d01 ra_sw_physics = 4");
    check(d1.c.integer("ghg_input") == 0 && d2.c.integer("ghg_input") == 0, "ra_sw_physics = -1, 0: GHG_INPUT = 0");
    Run s1("&domains\n max_dom = 2,\n/\n&physics\n physics_suite = 'CONUS',\n ra_sw_physics = -1, -1,\n/\n", 1);
    check(s1.c.integer("ghg_input") == 1, "ra_sw_physics = -1, -1 (CONUS): GHG_INPUT stays 1");
  }
  // output_wrf.F:1453-1460: a 4-D array's members by their own masks, which iofields_filename changes
  {
    Run r("&fdda\n grid_fdda = 1,\n/\n&physics\n mp_physics = 1,\n/\n", 1);
    WrfSchema s = schema();
    check(!has(s.history_vars(r.c, 41, 41, 31, 8), "U_NDG_OLD"), "grid_fdda: U_NDG_OLD not in history by default");
    const IofieldsResult a = s.apply_iofields(
        iofields_file("gem9_iofields_4d.txt", "+:h:0:u_ndg_old,mu_ndg_old\n+:h:3:qvapor,qcloud\n"), r.c);
    check(!a.warning, "iofields file accepted");
    const auto h = s.history_vars(r.c, 41, 41, 31, 8);
    check(has(h, "U_NDG_OLD") && has(h, "MU_NDG_OLD"), "+:h:0:u_ndg_old: U_NDG_OLD in history");
    const auto a3 = s.stream_vars(r.c, 41, 41, 31, 8, 0, 3);
    check(has(a3, "QVAPOR") && has(a3, "QCLOUD"), "+:h:3:qvapor: QVAPOR in auxhist3");
    check(!has(a3, "QRAIN"), "auxhist3: QRAIN not added");
  }
  // io_netcdf GetDim (wrf_io.F90:434-446): memory orders 'cz', 'ccz' are not written, 'c' and 'cc' are
  {
    Run r("&physics\n mp_physics = 1,\n/\n", 1);
    WrfSchema s = schema();
    const IofieldsResult a = s.apply_iofields(iofields_file("gem9_iofields_cz.txt", "+:h:0:ts_qv_profile,hcoeff\n"), r.c);
    check(!a.warning, "iofields file accepted");
    const auto h = s.history_vars(r.c, 41, 41, 31, 8);
    check(!has(h, "TS_QV_PROFILE"), "CCZ variable TS_QV_PROFILE not written");
    check(has(h, "HCOEFF"), "C variable HCOEFF written");
  }
  // check_nml_consistency rewrites of the namelist
  {
    Run m("&physics\n bl_pbl_physics = 5,\n scalar_pblmix = 1,\n/\n", 1);  // bl_mynn_mixaerosols default 1
    check(m.c.integer("scalar_pblmix") == 0, "MYNN mixing aerosols: scalar_pblmix = 0 (check_a_mundo.F:2514-2526)");
    Run y("&physics\n bl_pbl_physics = 1,\n scalar_pblmix = 1,\n/\n", 1);
    check(y.c.integer("scalar_pblmix") == 1, "YSU: scalar_pblmix kept");
    Run k("&domains\n max_dom = 2,\n/\n&physics\n cu_physics = 11, 0,\n icloud = 0,\n/\n", 2);
    check(k.c.integer("icloud") == 1, "cu_physics = 11, 0: icloud = 1 on d02 (check_a_mundo.F:1390-1398)");
    check(!k.c.logical("cu_rad_feedback"), "cu_physics = 11, 0: d02 cu_rad_feedback stays false");
    Run b("&bdy_control\n specified = .true.,\n spec_bdy_width = 5,\n spec_zone = 2,\n relax_zone = 3,\n/\n", 1);
    check(b.c.integer("spec_zone") == 1 && b.c.integer("relax_zone") == 4,
          "specified: spec_zone = 1, relax_zone = spec_bdy_width - 1 (check_a_mundo.F:248-251)");
    Run i("&bdy_control\n spec_zone = 2,\n relax_zone = 3,\n/\n", 1);
    check(i.c.integer("spec_zone") == 2 && i.c.integer("relax_zone") == 3, "not specified: namelist zones kept");
    Run d("&domains\n max_dom = 2,\n/\n&dynamics\n diff_opt = 1, -1,\n km_opt = 4, -1,\n/\n", 2);
    check(d.c.integer("diff_opt") == 1 && d.c.integer("km_opt") == 4, "d02 diff_opt / km_opt -1: d01's (check_a_mundo.F:289-312)");
    Run g("&physics\n ra_lw_physics = 4,\n ra_sw_physics = 4,\n/\n", 1);
    check(g.c.integer("levsiz") == 59 && g.c.integer("alevsiz") == 12 && g.c.integer("no_src_types") == 6,
          "RRTMG: levsiz 59, alevsiz 12, no_src_types 6 (check_a_mundo.F:3566-3580)");
    Run n("&physics\n ra_lw_physics = 1,\n ra_sw_physics = 1,\n/\n", 1);
    check(n.c.integer("levsiz") == 1, "RRTM / Dudhia: levsiz Registry default 1");
  }
#ifdef CPPWRF_HAVE_NETCDF
  // output_wrf.F:965-973 and ext_ncd_put_dom_ti_char (len_trim): MMINLU without trailing blanks, a blank one empty
  {
    Run r("", 1);
    auto mminlu = [&](const std::string& in, bool ideal) {
      const std::string path = (std::filesystem::temp_directory_path() / "gem9_mminlu.nc").string();
      {
        NcOut nc(path, false);
        DomainAtts d;
        d.mminlu = in;
        d.ideal = ideal;
        d.start_date = d.simulation_start_date = "2026-10-12_00:00:00";
        d.have_julgmt = true;
        d.ide = d.jde = 3;
        d.kde = 3;
        write_global_atts(nc, r.c, d, Stream::history);
        nc.enddef();
      }
      int id = -1;
      std::size_t len = 0;
      std::string s;
      if (nc_open(path.c_str(), NC_NOWRITE, &id) == NC_NOERR) {
        if (nc_inq_attlen(id, NC_GLOBAL, "MMINLU", &len) == NC_NOERR) {
          s.assign(len, '?');
          if (len) nc_get_att_text(id, NC_GLOBAL, "MMINLU", s.data());
        } else {
          s = "<none>";
        }
        nc_close(id);
      }
      std::filesystem::remove(path);
      return s;
    };
    check(mminlu("MODIFIED_IGBP_MODIS_NOAH", false) == "MODIFIED_IGBP_MODIS_NOAH", "MMINLU MODIS kept");
    check(mminlu("USGS", false) == "USGS" && mminlu("USGS_LAKES", false) == "USGS", "MMINLU USGS*: USGS");
    check(mminlu("NLCD40  ", false) == "NLCD40", "MMINLU trailing blanks trimmed");
    check(mminlu("    ", false).empty() && mminlu("", false).empty(), "MMINLU blank: empty attribute");
    check(mminlu("", true).empty(), "MMINLU ideal: empty attribute");
  }
#endif
  if (failures) std::cout << failures << " failure(s)\n";
  return failures ? 1 : 0;
}
