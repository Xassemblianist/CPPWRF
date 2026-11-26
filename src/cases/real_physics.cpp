// SPDX-License-Identifier: GPL-3.0-or-later
// Column physics of a real-data run (P1 suite: sfclayrev + Noah + YSU): the suite is created
// and its initial surface state filled as WRF's start_domain does for a non-restart run:
//   - the wrfinput fields (TSK, TMN, TSLB, SMOIS, SEAICE, LU_INDEX, ISLTYP, VEGFRA, ...),
//   - phy_init (module_physics_init.F): UST = 1e-4, MOL = PBLH = HFX = QFX = RAINBL = ACSNOW = 0,
//     IVGTYP = nint(LU_INDEX), compute_2d_dx_area (DX2D = dx), landuse_init (ALBBCK, ALBEDO,
//     THC, Z0, ZNT, EMBCK, EMISS, MAVAIL, XLAND from LANDUSE.TBL for the season of JULDAY),
//   - bl_init: ACHFX = ACLHF = ACGRDFLX = SFCEVP = 0, LSMINIT (module_sf_noahdrv.F): SH2O from
//     SMOIS/TSLB (FRH2O for frozen layers), SNOWH = 0.005*SNOW without input SNOWH, CANWAT = 0,
//     SNOALB = MAXALB/100 when rdmaxalb = false,
//   - start_em: CTOPO = CTOPO2 = 1 (topo_wind = 0; topo_wind 1 / 2, slope_rad, topo_shading: PhysicsSuite::init_topo).
// Every other surface field starts at zero (WRF allocates grid fields zeroed).
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <type_traits>
#include <vector>

#include "cases/model.hpp"
#include "core/tiles.hpp"
#include "phys/driver/landuse_tables.hpp"
#include "phys/driver/suite.hpp"
#include "phys/lsm_noah/noah.hpp"
#include "phys/lsm_ruc/ruc.hpp"
#include "phys/lsm_ruc/ruc_tables.hpp"
#include "wrf_io/namelist.hpp"
#ifdef CPPWRF_HAVE_NETCDF
#include <netcdf.h>

#include "io_netcdf/nc_read.hpp"
#endif

namespace cppwrf::cases {

bool wants_column_physics(const io::Config& cfg) {
  // sf_lake_physics: the suite then refuses the lake without Noah / Noah-MP instead of skipping it
  return cfg.get_int("phys.sf_sfclay_physics", 0) != 0 || cfg.get_int("phys.sf_surface_physics", 0) != 0 ||
         cfg.get_int("phys.bl_pbl_physics", 0) != 0 || cfg.get_int("phys.cu_physics", 0) != 0 ||
         cfg.get_int("phys.sf_lake_physics", 0) != 0;
}

LanduseInit landuse_init(const std::string& table_dir, const std::string& mminlu, int julday, double cen_lat,
                         int iswater, int isice, const std::vector<double>& lu, const std::vector<double>& xice,
                         const std::vector<double>& snowc, int fractional_seaice, bool usemonalb,
                         const std::vector<double>& albbck_in, const std::vector<double>& snoalb_in) {
  phys::driver::LanduseTables lt;
  if (!phys::driver::read_landuse_table(table_dir + "/LANDUSE.TBL", mminlu, lt))
    throw std::runtime_error("real physics: LANDUSE.TBL lacks " + mminlu);
  int isn = 1;
  if (julday < 105 || julday > 288) isn = 2;
  if (cen_lat < 0.0) isn = 3 - isn;
  if (lt.n_seasons == 1) isn = 1;
  const double xice_threshold = fractional_seaice == 0 ? 0.5 : 0.02;
  const std::size_t n2 = lu.size();
  LanduseInit r;
  std::vector<double>&albbck = r.albbck, &albedo = r.albedo, &z0 = r.z0, &embck = r.embck, &emiss = r.emiss,
                     &mavail = r.mavail, &xland = r.xland;
  for (auto* v : {&albbck, &albedo, &z0, &embck, &emiss, &mavail, &xland, &r.thc}) v->assign(n2, 0.0);
  r.isn = isn;
  r.lucats = lt.n_cats;
  r.luseas = lt.n_seasons;
  {  // landuse_init's saved table (max_cats = 100, max_seas = 12)
    r.lu_state.assign(7501, 0.0);
    std::size_t curs = 0;
    for (int cats = 1; cats <= 100; ++cats) {
      const bool in = cats < phys::driver::kMaxLanduseCats && cats <= lt.n_cats;
      // SCFX(LC) is read again for every season: the last season's value stays
      r.lu_state[curs++] = in ? double(lt.categories[std::max(0, lt.n_seasons - 1)][cats].scfx) : 0.0;
      for (int seas = 1; seas <= 12; ++seas) {
        const bool ok = in && seas <= lt.n_seasons;
        const phys::driver::LanduseCategory* c = ok ? &lt.categories[seas - 1][cats] : nullptr;
        for (real x : {c ? c->albd : real(0), c ? c->slmo : real(0), c ? c->sfem : real(0), c ? c->sfz0 : real(0),
                       c ? c->sfhc : real(0), c ? c->therin : real(0)})
          r.lu_state[curs++] = double(x);
      }
    }
  }
  for (std::size_t p = 0; p < n2; ++p) {
    int is = int(std::lround(lu[p]));
    if (is < 0 || is > lt.n_cats) throw std::runtime_error("real physics: LANDUSE OUTSIDE RANGE");
    if (is == 0) is = iswater;
    const auto& c = lt.categories[isn - 1][is];
    albbck[p] = usemonalb ? albbck_in[p] : c.albd / 100.;
    albedo[p] = albbck[p];
    // SCFX(IS) comes from lu_state: the last season read (see above), not season ISN
    if (snowc[p] > 0.5)
      albedo[p] = usemonalb ? snoalb_in[p]
                  : std::is_same_v<real, float>  // WRF real*4: ALBBCK*(1.+SCFX(IS)) in REAL
                      ? double(real(albbck[p]) * (real(1.) + lt.categories[std::max(0, lt.n_seasons - 1)][is].scfx))
                      : albbck[p] * (1. + lt.categories[std::max(0, lt.n_seasons - 1)][is].scfx);
    z0[p] = c.sfz0 / 100.;
    embck[p] = c.sfem;
    emiss[p] = embck[p];
    mavail[p] = c.slmo;
    r.thc[p] = double(c.therin / real(100.));
    xland[p] = is != iswater ? 1.0 : 2.0;
    if (xice[p] >= xice_threshold) {
      const auto& ci = lt.categories[isn - 1][isice];
      xland[p] = 1.0;
      albbck[p] = ci.albd / 100.;
      embck[p] = ci.sfem;
      if (fractional_seaice == 1) {  // open water: albedo 0.08, emissivity 0.98
        albedo[p] = (xice[p] * albbck[p]) + ((1.0 - xice[p]) * 0.08);
        emiss[p] = (xice[p] * embck[p]) + ((1.0 - xice[p]) * 0.98);
      } else {
        albedo[p] = albbck[p];
        emiss[p] = embck[p];
      }
      z0[p] = ci.sfz0 / 100.;
      mavail[p] = ci.slmo;
      r.thc[p] = double(ci.therin / real(100.));
    }
  }
  return r;
}

#ifndef CPPWRF_HAVE_NETCDF
void init_real_physics(const io::Config&, Model&) { throw std::runtime_error("column physics needs NetCDF"); }
void phy_init_after_feedback(const io::Config&, Model&) {}
void check_phy_init_moved(const io::Config&, const Model&, const std::string&) {
  throw std::runtime_error("moving nests need NetCDF support");
}
std::vector<double> phy_init_moved(const io::Config&, Model&, const std::vector<unsigned char>&,
                                   const std::vector<double>&) {
  throw std::runtime_error("moving nests need NetCDF support");
}
void read_wrflowinp(const std::string&, const std::string&, phys::driver::PhysicsSuite&) {
  throw std::runtime_error("sst_update needs NetCDF");
}
void read_wrffdda(const std::string&, const std::string&, phys::driver::PhysicsSuite&) {
  throw std::runtime_error("grid_fdda needs NetCDF");
}
#else
namespace {
class NcIn {
 public:
  explicit NcIn(const std::string& path) : path_(path) {
    if (nc_open(path.c_str(), NC_NOWRITE, &id_) != NC_NOERR) throw std::runtime_error("real physics: cannot open " + path);
  }
  ~NcIn() { nc_close(id_); }
  NcIn(const NcIn&) = delete;
  NcIn& operator=(const NcIn&) = delete;
  bool has(const char* name) const {
    int v = 0;
    return nc_inq_varid(id_, name, &v) == NC_NOERR;
  }
  // name's n values: the whole variable holds n values, or its first record does (a record dimension Time of more
  // records); any other size is an error before anything is read
  std::vector<double> var(const char* name, std::size_t n) const {
    int v = 0, nd = 0;
    if (nc_inq_varid(id_, name, &v) != NC_NOERR || nc_inq_varndims(id_, v, &nd) != NC_NOERR)
      throw std::runtime_error(std::string("real physics: ") + path_ + " lacks " + name);
    std::vector<int> dids(std::size_t(nd) + 1);
    std::vector<std::size_t> len(std::size_t(nd) + 1, 1);
    if (nc_inq_vardimid(id_, v, dids.data()) != NC_NOERR)
      throw std::runtime_error(std::string("real physics: ") + path_ + ": cannot read the dimensions of " + name);
    std::size_t total = 1;
    for (int d = 0; d < nd; ++d) {
      if (nc_inq_dimlen(id_, dids[std::size_t(d)], &len[std::size_t(d)]) != NC_NOERR)
        throw std::runtime_error(std::string("real physics: ") + path_ + ": cannot read the dimensions of " + name);
      total *= len[std::size_t(d)];
    }
    std::vector<double> out(n);
    int rc = NC_NOERR;
    if (total == n) {
      rc = nc_get_var_double(id_, v, out.data());
    } else {
      char d0[NC_MAX_NAME + 1] = {};
      if (nd < 1 || nc_inq_dimname(id_, dids[0], d0) != NC_NOERR || std::string(d0) != "Time" || len[0] == 0 ||
          total / len[0] != n)
        throw std::runtime_error(std::string("real physics: ") + path_ + ": " + name + " holds " +
                                 std::to_string(total) + " values, expected " + std::to_string(n));
      std::vector<std::size_t> st(std::size_t(nd), 0), ct(len.begin(), len.begin() + nd);
      ct[0] = 1;
      rc = nc_get_vara_double(id_, v, st.data(), ct.data(), out.data());
    }
    if (rc != NC_NOERR) throw std::runtime_error(std::string("real physics: ") + path_ + ": cannot read " + name);
    return out;
  }
  // record 0 of name with exactly the dimensions dims (io_netcdf::read_record)
  std::vector<double> rec(const char* name, const std::vector<io_netcdf::DimSpec>& dims) const {
    return io_netcdf::read_record(id_, path_, name, dims);
  }
  const std::string& path() const { return path_; }
  std::string att_str(const char* name, const std::string& def) const {
    std::size_t len = 0;
    if (nc_inq_attlen(id_, NC_GLOBAL, name, &len) != NC_NOERR) return def;
    std::string s(len, '\0');
    if (nc_get_att_text(id_, NC_GLOBAL, name, s.data()) != NC_NOERR) return def;
    while (!s.empty() && (s.back() == '\0' || s.back() == ' ')) s.pop_back();
    return s;
  }
  double att_num(const char* name, double def) const {
    double x = 0;
    return nc_get_att_double(id_, NC_GLOBAL, name, &x) == NC_NOERR ? x : def;
  }

 private:
  std::string path_;
  int id_ = -1;
};
}  // namespace

// med_auxinput_in for auxinput4 (wrflowinp_d0N): the record at `date` overwrites the fields of the stream
// that the file holds (SST, SEAICE, VEGFRA, ALBBCK, LAI, UOCE, VOCE)
void read_wrflowinp(const std::string& path, const std::string& date, phys::driver::PhysicsSuite& su) {
  int id = -1;
  if (nc_open(path.c_str(), NC_NOWRITE, &id) != NC_NOERR) throw std::runtime_error("sst_update: cannot open " + path);
  int vt = -1, dims[2];
  std::size_t nt = 0, sl = 0;
  if (nc_inq_varid(id, "Times", &vt) != NC_NOERR) throw std::runtime_error("sst_update: " + path + " has no Times");
  nc_inq_vardimid(id, vt, dims);
  nc_inq_dimlen(id, dims[0], &nt);
  nc_inq_dimlen(id, dims[1], &sl);
  long rec = -1;
  for (std::size_t t = 0; t < nt && rec < 0; ++t) {
    std::string s(sl, '\0');
    const std::size_t st[2] = {t, 0}, ct[2] = {1, sl};
    nc_get_vara_text(id, vt, st, ct, s.data());
    if (s.compare(0, date.size(), date) == 0) rec = long(t);
  }
  if (rec < 0) {
    nc_close(id);
    throw std::runtime_error("sst_update: " + path + " has no record for " + date);
  }
  const Layout& L = su.sst.layout();
  const int nx = L.nx, ny = L.ny;
  const std::pair<const char*, Field2*> fields[] = {
      {"SST", &su.sst}, {"SEAICE", &su.xice}, {"VEGFRA", &su.vegfra}, {"ALBBCK", &su.albbck},
      {"LAI", &su.lai}, {"UOCE", &su.uoce}, {"VOCE", &su.voce}};
  std::vector<double> buf(std::size_t(nx) * ny);
  for (const auto& [name, f] : fields) {
    int v = -1;
    if (nc_inq_varid(id, name, &v) != NC_NOERR) continue;
    const std::size_t st[3] = {std::size_t(rec), 0, 0}, ct[3] = {1, std::size_t(ny), std::size_t(nx)};
    if (nc_get_vara_double(id, v, st, ct, buf.data()) != NC_NOERR)
      throw std::runtime_error(std::string("sst_update: cannot read ") + name + " of " + path);
    f->fill_host([&](int i, int j, int) { return buf[i + std::size_t(nx) * j]; });
  }
  nc_close(id);
}

// med_auxinput_in for auxinput10 (gfdda_inname, wrffdda_d0N): the record at `date` overwrites the members of the
// 4-D arrays fdda3d and fdda2d (grid_fdda = 1: U/V/T/Q/PH_NDG_OLD/NEW on the mass dimensions, MU_NDG_OLD/NEW)
void read_wrffdda(const std::string& path, const std::string& date, phys::driver::PhysicsSuite& su) {
  int id = -1;
  if (nc_open(path.c_str(), NC_NOWRITE, &id) != NC_NOERR) throw std::runtime_error("grid_fdda: cannot open " + path);
  int vt = -1, dims[2];
  std::size_t nt = 0, sl = 0;
  if (nc_inq_varid(id, "Times", &vt) != NC_NOERR) {
    nc_close(id);
    throw std::runtime_error("grid_fdda: " + path + " has no Times");
  }
  nc_inq_vardimid(id, vt, dims);
  nc_inq_dimlen(id, dims[0], &nt);
  nc_inq_dimlen(id, dims[1], &sl);
  long rec = -1;
  for (std::size_t t = 0; t < nt && rec < 0; ++t) {
    std::string s(sl, '\0');
    const std::size_t st[2] = {t, 0}, ct[2] = {1, sl};
    nc_get_vara_text(id, vt, st, ct, s.data());
    if (s.compare(0, date.size(), date) == 0) rec = long(t);
  }
  if (rec < 0) {
    nc_close(id);
    throw std::runtime_error("grid_fdda: " + path + " has no record for " + date);
  }
  const Layout& L = su.t_ndg_old.layout();
  const int nx = L.nx, ny = L.ny, nz = L.nz;
  const std::pair<const char*, Field3*> f3[] = {
      {"U_NDG_OLD", &su.u_ndg_old}, {"V_NDG_OLD", &su.v_ndg_old}, {"T_NDG_OLD", &su.t_ndg_old},
      {"Q_NDG_OLD", &su.q_ndg_old}, {"PH_NDG_OLD", &su.ph_ndg_old}, {"U_NDG_NEW", &su.u_ndg_new},
      {"V_NDG_NEW", &su.v_ndg_new}, {"T_NDG_NEW", &su.t_ndg_new}, {"Q_NDG_NEW", &su.q_ndg_new},
      {"PH_NDG_NEW", &su.ph_ndg_new}};
  std::vector<double> buf(std::size_t(nx) * ny * nz);
  for (const auto& [name, f] : f3) {
    int v = -1;
    const std::size_t st[4] = {std::size_t(rec), 0, 0, 0}, ct[4] = {1, std::size_t(nz), std::size_t(ny), std::size_t(nx)};
    if (nc_inq_varid(id, name, &v) != NC_NOERR || nc_get_vara_double(id, v, st, ct, buf.data()) != NC_NOERR) {
      nc_close(id);
      throw std::runtime_error(std::string("grid_fdda: cannot read ") + name + " of " + path);
    }
    f->fill_host([&](int i, int j, int k) { return buf[i + std::size_t(nx) * (j + std::size_t(ny) * k)]; });
  }
  const std::pair<const char*, Field2*> f2[] = {{"MU_NDG_OLD", &su.mu_ndg_old}, {"MU_NDG_NEW", &su.mu_ndg_new}};
  for (const auto& [name, f] : f2) {
    int v = -1;
    const std::size_t st[4] = {std::size_t(rec), 0, 0, 0}, ct[4] = {1, 1, std::size_t(ny), std::size_t(nx)};
    if (nc_inq_varid(id, name, &v) != NC_NOERR || nc_get_vara_double(id, v, st, ct, buf.data()) != NC_NOERR) {
      nc_close(id);
      throw std::runtime_error(std::string("grid_fdda: cannot read ") + name + " of " + path);
    }
    f->fill_host([&](int i, int j, int) { return buf[i + std::size_t(nx) * j]; });
  }
  nc_close(id);
}

void init_real_physics(const io::Config& cfg, Model& m) {
  using phys::driver::PhysicsSuite;
  using phys::driver::SuiteOptions;
  const Grid& g = m.g;
  const int nx = g.nx, ny = g.ny;
  const std::string dir = cfg.get_string("real.dir");
  const NcIn in(dir + "/" + cfg.get_string("real.wrfinput", "wrfinput_d01"));

  SuiteOptions o;
  o.sf_sfclay_physics = int(cfg.get_int("phys.sf_sfclay_physics", 0));
  o.sf_surface_physics = int(cfg.get_int("phys.sf_surface_physics", 0));
  o.bl_pbl_physics = int(cfg.get_int("phys.bl_pbl_physics", 0));
  o.mfshconv = int(cfg.get_int("phys.mfshconv", 1));  // QNSE's EDKF (Registry default 1; check_a_mundo: 0 unless 4)
  if (o.bl_pbl_physics != 4) o.mfshconv = 0;
  o.icloud_bl = int(cfg.get_int("phys.icloud_bl", 1));
  o.mynn.cloudpdf = int(cfg.get_int("phys.bl_mynn_cloudpdf", 2));  // MYNN-EDMF options (WRF's Registry names)
  o.mynn.mixlength = int(cfg.get_int("phys.bl_mynn_mixlength", 1));
  o.mynn.edmf = int(cfg.get_int("phys.bl_mynn_edmf", 1));
  o.mynn.edmf_mom = int(cfg.get_int("phys.bl_mynn_edmf_mom", 1));
  o.mynn.edmf_tke = int(cfg.get_int("phys.bl_mynn_edmf_tke", 0));
  o.mynn.cloudmix = int(cfg.get_int("phys.bl_mynn_cloudmix", 1));
  o.mynn.mixqt = int(cfg.get_int("phys.bl_mynn_mixqt", 0));
  o.mynn.ess = int(cfg.get_int("phys.bl_mynn_ess", 1));
  o.mynn.tkeadvect = cfg.get_bool("phys.bl_mynn_tkeadvect", false);
  o.mynn.closure = real(cfg.get_double("phys.bl_mynn_closure", 2.6));
  o.bl_mynn_mixscalars = int(cfg.get_int("phys.bl_mynn_mixscalars", 0));
  o.bl_mynn_output = int(cfg.get_int("phys.bl_mynn_output", 0));
  o.tke_budget = int(cfg.get_int("phys.tke_budget", 0));
  o.grav_settling = int(cfg.get_int("phys.grav_settling", 0));
  // check_a_mundo: mp_physics = 28 has its own fog settling, grav_settling is reset to 0
  if (cfg.get_int("phys.mp_physics", 0) == 28) o.grav_settling = 0;
  o.cu_physics = int(cfg.get_int("phys.cu_physics", 0));
  // set_physics_rconfigs (module_check_a_mundo.F:3586-3603) sets num_soil_layers from domain 1's land surface model
  // whatever the namelist says: 5 without one and for the slab (real.exe writes 5-layer TSLB / SMOIS), 4 for Noah and
  // Noah-MP, 6 or 9 for RUC (another value: 6). The translator writes that value; another one in the configuration
  // is replaced as wrf.exe replaces it (the wrfinput soil fields have that many layers)
  {
    const int lsm = o.sf_surface_physics, given = int(cfg.get_int("phys.num_soil_layers", -1));
    const int wrf = lsm == 0 || lsm == 1 ? 5 : lsm == 3 ? (given == 6 || given == 9 ? given : 6)
                    : lsm == 5 ? 10 : lsm == 7 ? 2 : lsm == 8 ? 3 : 4;  // CLM, Pleim-Xiu, SSiB (not ported)
    if (given != -1 && given != wrf && tile::writer())
      std::cerr << "cppwrf: NOTE: phys.num_soil_layers = " << given << " is set to " << wrf
                << " for sf_surface_physics = " << lsm << ", as wrf.exe does (set_physics_rconfigs)" << std::endl;
    o.num_soil_layers = wrf;
  }
  o.bldt = cfg.get_double("phys.bldt", 0.0);
  o.cudt = cfg.get_double("phys.cudt", 0.0);
  // check_a_mundo (module_check_a_mundo.F:2156-2174): the adaptive time step runs BMJ, the SAS schemes and Tiedtke
  // every step (cudt = 0)
  if (cfg.get_bool("time.adaptive", false) && (o.cu_physics == 2 || o.cu_physics == 6)) o.cudt = 0.0;
  // bmj_rad_feedback (Registry max_domains, default .false.): BMJ's convective clouds (CCLDFRA, QCCONV, QICONV,
  // CONVCLD) and their use in the radiation; other cumulus schemes ignore it
  o.bmj_rad_feedback = o.cu_physics == 2 && cfg.get_bool("phys.bmj_rad_feedback", false);
  {
    const std::string nl_path = cfg.get_string("output.wrf_namelist", dir + "/namelist.input");
    const double nl_avg = std::filesystem::exists(nl_path)
                              ? wrf_io::Namelist::read(nl_path).get_real("physics", "convtrans_avglen_m", 30.0)
                              : 30.0;
    o.convtrans_avglen_m = cfg.get_double("phys.convtrans_avglen_m", nl_avg);
  }
  o.kfeta_trigger = int(cfg.get_int("phys.kfeta_trigger", 1));
  o.ishallow = int(cfg.get_int("phys.ishallow", 0));          // Grell-Freitas shallow convection
  o.gf_ichoice = int(cfg.get_int("phys.clos_choice", 0));     // Grell-Freitas / Grell-3D closure
  o.cugd_avedx = int(cfg.get_int("phys.cugd_avedx", 1));      // Grell-3D subsidence spreading
  o.warm_rain = cfg.get_int("phys.mp_physics", 0) == 1;  // mp_init: true only for KESSLERSCHEME
  o.isfflx = int(cfg.get_int("phys.isfflx", 1));
  o.ifsnow = int(cfg.get_int("phys.ifsnow", 1));        // the slab's snow cover effects (Registry default 1)
  o.opt_thcnd = int(cfg.get_int("phys.opt_thcnd", 1));
  o.rdlai2d = cfg.get_bool("phys.rdlai2d", false);      // Noah: LAI from the input instead of the table
  o.usemonalb = cfg.get_bool("phys.usemonalb", false);  // landuse_init and Noah: ALBBCK from the input
  o.sf_urban_physics = int(cfg.get_int("phys.sf_urban_physics", 0));  // 1: single-layer UCM (with Noah)
  o.use_wudapt_lcz = int(cfg.get_int("phys.use_wudapt_lcz", 0));      // 1: URBPARM_LCZ.TBL
  if (o.sf_urban_physics != 0 && cfg.get_bool("real.nested", false))
    throw std::runtime_error("sf_urban_physics = " + std::to_string(o.sf_urban_physics) +
                             " on a nest is not supported");
  // the lake model (module_sf_lake.F): one domain (the nest interpolation of the lake fields, interp_mask_water_field
  // with ISLAKE, is not ported)
  o.sf_lake_physics = int(cfg.get_int("phys.sf_lake_physics", 0));
  o.use_lakedepth = int(cfg.get_int("phys.use_lakedepth", 1));
  o.lakedepth_default = cfg.get_double("phys.lakedepth_default", 50.);
  o.lake_min_elev = cfg.get_double("phys.lake_min_elev", 5.);
  if (o.sf_lake_physics != 0 && cfg.get_bool("real.nested", false))
    throw std::runtime_error("sf_lake_physics = " + std::to_string(o.sf_lake_physics) + " on a nest is not supported");
  if (o.sf_lake_physics != 0 && cfg.get_bool("time.adaptive", false))
    throw std::runtime_error("sf_lake_physics = " + std::to_string(o.sf_lake_physics) +
                             " with the adaptive time step (use_adaptive_time_step) is not supported");
  // terrain options (start_em, pre_radiation_driver, surface_driver): one domain, a fixed time step
  o.topo_wind = int(cfg.get_int("phys.topo_wind", 0));
  o.slope_rad = int(cfg.get_int("phys.slope_rad", 0));
  o.topo_shading = int(cfg.get_int("phys.topo_shading", 0));
  o.shadlen = cfg.get_double("phys.shadlen", 25000.);
  if ((o.topo_wind != 0 || o.slope_rad != 0 || o.topo_shading != 0) && cfg.get_bool("real.nested", false))
    throw std::runtime_error("topo_wind / slope_rad / topo_shading on a nest are not supported");
  if ((o.slope_rad != 0 || o.topo_shading != 0) && cfg.get_bool("time.adaptive", false))
    throw std::runtime_error("slope_rad / topo_shading with the adaptive time step (use_adaptive_time_step) are not "
                             "supported");
  o.ysu_topdown_pblmix = int(cfg.get_int("phys.ysu_topdown_pblmix", 1));
  o.kim_tofd = cfg.get_bool("phys.kim_tofd", false);
  o.isftcflx = int(cfg.get_int("phys.isftcflx", 0));  // sfclayrev: z0t / z0q and roughness over water
  // phys.shalwater_z0 and phys.scm_force_flux: refused in build_model's option checks (model.cpp)
  o.iz0tlnd = int(cfg.get_int("phys.iz0tlnd", 0));    // sfclayrev: z0t over land (czil)
  o.sf_mynn_sfcflux_land = int(cfg.get_int("phys.sf_mynn_sfcflux_land", 0));    // MYNN surface layer
  o.sf_mynn_sfcflux_water = int(cfg.get_int("phys.sf_mynn_sfcflux_water", 1));
  o.fractional_seaice = int(cfg.get_int("phys.fractional_seaice", 0));
  o.tice2tsk_if2cold = cfg.get_bool("phys.tice2tsk_if2cold", false);
  o.sst_update = int(cfg.get_int("phys.sst_update", 0));
  o.sst_skin = int(cfg.get_int("phys.sst_skin", 0));        // skin SST (module_sf_sstskin.F)
  o.tmn_update = int(cfg.get_int("phys.tmn_update", 0));    // deep soil temperature update (module_sf_tmnupdate.F)
  o.lagday = int(cfg.get_int("phys.lagday", 150));
  if ((o.sst_skin != 0 || o.tmn_update != 0) && cfg.get_bool("time.adaptive", false))
    throw std::runtime_error("sst_skin / tmn_update with the adaptive time step (use_adaptive_time_step) are not supported");
  // grid (analysis) nudging, this domain's entries (namelist &fdda; gfdda_interval_m / gfdda_end_h are WRF's
  // auxinput10_interval_m / auxinput10_end_h; if_ramping and dtramp_min are domain 1's)
  o.fdda.grid_fdda = int(cfg.get_int("fdda.grid_fdda", 0));
  o.fdda.interval_m = int(cfg.get_int("fdda.gfdda_interval_m", 0));
  o.fdda.end_h = int(cfg.get_int("fdda.gfdda_end_h", 0));
  o.fdda.if_no_pbl_nudging_uv = int(cfg.get_int("fdda.if_no_pbl_nudging_uv", 0));
  o.fdda.if_no_pbl_nudging_t = int(cfg.get_int("fdda.if_no_pbl_nudging_t", 0));
  o.fdda.if_no_pbl_nudging_q = int(cfg.get_int("fdda.if_no_pbl_nudging_q", 0));
  o.fdda.if_zfac_uv = int(cfg.get_int("fdda.if_zfac_uv", 0));
  o.fdda.k_zfac_uv = int(cfg.get_int("fdda.k_zfac_uv", 0));
  o.fdda.if_zfac_t = int(cfg.get_int("fdda.if_zfac_t", 0));
  o.fdda.k_zfac_t = int(cfg.get_int("fdda.k_zfac_t", 0));
  o.fdda.if_zfac_q = int(cfg.get_int("fdda.if_zfac_q", 0));
  o.fdda.k_zfac_q = int(cfg.get_int("fdda.k_zfac_q", 0));
  o.fdda.guv = cfg.get_double("fdda.guv", 0.0);
  o.fdda.gt = cfg.get_double("fdda.gt", 0.0);
  o.fdda.gq = cfg.get_double("fdda.gq", 0.0);
  o.fdda.if_ramping = int(cfg.get_int("fdda.if_ramping", 0));
  o.fdda.dtramp_min = cfg.get_double("fdda.dtramp_min", 0.0);
  o.fdda.fgdt = cfg.get_double("fdda.fgdt", 0.0);
  o.fdda.fgdtzero = int(cfg.get_int("fdda.fgdtzero", 0));
  if (int(cfg.get_int("fdda.grid_sfdda", 0)) != 0)
    throw std::runtime_error("grid_sfdda (surface analysis nudging) is not supported: it needs wrfsfdda_d0N from OBSGRID");
  if (o.fdda.grid_fdda != 0 && cfg.get_bool("time.adaptive", false))
    throw std::runtime_error("grid_fdda with the adaptive time step (use_adaptive_time_step) is not supported");
  {  // observation nudging (&fdda obs_*; max_domains entries this domain's, fdda.obs_* of [d0N] for a nest)
    auto& ob = o.obs;
    auto gi = [&](const char* k, int d) { return int(cfg.get_int(k, d)); };
    auto gd = [&](const char* k, double d) { return cfg.get_double(k, d); };
    ob.obs_nudge_opt = gi("fdda.obs_nudge_opt", 0);
    if (ob.obs_nudge_opt != 0) {
      if (ob.obs_nudge_opt != 1) throw std::runtime_error("obs_nudge_opt = " + std::to_string(ob.obs_nudge_opt) + " (0 or 1)");
      ob.nudge_wind = gi("fdda.obs_nudge_wind", 0);
      ob.nudge_temp = gi("fdda.obs_nudge_temp", 0);
      ob.nudge_mois = gi("fdda.obs_nudge_mois", 0);
      ob.nudge_pstr = gi("fdda.obs_nudge_pstr", 0);
      ob.coef_wind = gd("fdda.obs_coef_wind", 0);
      ob.coef_temp = gd("fdda.obs_coef_temp", 0);
      ob.coef_mois = gd("fdda.obs_coef_mois", 0);
      ob.coef_pstr = gd("fdda.obs_coef_pstr", 0);
      ob.rinxy = gd("fdda.obs_rinxy", 0);
      ob.twindo = gd("fdda.obs_twindo", 0);
      ob.twindo_cg = gd("fdda.obs_twindo_cg", ob.twindo);
      ob.ionf = gi("fdda.obs_ionf", 1);
      ob.prt_freq = gi("fdda.obs_prt_freq", 1000);
      ob.no_pbl_nudge_uv = gi("fdda.obs_no_pbl_nudge_uv", 0);
      ob.no_pbl_nudge_t = gi("fdda.obs_no_pbl_nudge_t", 0);
      ob.no_pbl_nudge_q = gi("fdda.obs_no_pbl_nudge_q", 0);
      ob.fdda_end = gd("fdda.fdda_end", 0);
      ob.idynin = gi("fdda.obs_idynin", 0);
      ob.dtramp = gd("fdda.obs_dtramp", 0);
      ob.sfc_scheme_horiz = gi("fdda.obs_sfc_scheme_horiz", 0);
      ob.sfc_scheme_vert = gi("fdda.obs_sfc_scheme_vert", 0);
      ob.max_sndng_gap = gd("fdda.obs_max_sndng_gap", 20);
      ob.sfcfact = gd("fdda.obs_sfcfact", 1.0);
      ob.sfcfacr = gd("fdda.obs_sfcfacr", 1.0);
      ob.dpsmx = gd("fdda.obs_dpsmx", 7.5);
      // the full / ramp heights of the regimes 1, 2, 4 (the keys as literals: src/cases/config_keys.cmake)
      const char* const vk[3][3][2] = {
          {{"fdda.obs_nudgezfullr1_uv", "fdda.obs_nudgezrampr1_uv"}, {"fdda.obs_nudgezfullr1_t", "fdda.obs_nudgezrampr1_t"},
           {"fdda.obs_nudgezfullr1_q", "fdda.obs_nudgezrampr1_q"}},
          {{"fdda.obs_nudgezfullr2_uv", "fdda.obs_nudgezrampr2_uv"}, {"fdda.obs_nudgezfullr2_t", "fdda.obs_nudgezrampr2_t"},
           {"fdda.obs_nudgezfullr2_q", "fdda.obs_nudgezrampr2_q"}},
          {{"fdda.obs_nudgezfullr4_uv", "fdda.obs_nudgezrampr4_uv"}, {"fdda.obs_nudgezfullr4_t", "fdda.obs_nudgezrampr4_t"},
           {"fdda.obs_nudgezfullr4_q", "fdda.obs_nudgezrampr4_q"}}};
      decltype(&ob.vif_uv) const arrs[3] = {&ob.vif_uv, &ob.vif_t, &ob.vif_q};
      for (int r = 0; r < 3; ++r) {
        for (int v = 0; v < 3; ++v) {
          (*arrs[v])[2 * r] = gd(vk[r][v][0], r == 2 ? -5000 : 50);
          (*arrs[v])[2 * r + 1] = gd(vk[r][v][1], 50);
        }
      }
      ob.nudgezfullmin = gd("fdda.obs_nudgezfullmin", 50);
      ob.nudgezrampmin = gd("fdda.obs_nudgezrampmin", 50);
      ob.nudgezmax = gd("fdda.obs_nudgezmax", 3000);
      ob.scl_neg_qv_innov = gi("fdda.obs_scl_neg_qv_innov", 0);
      ob.max_obs = gi("fdda.max_obs", 0);
      ob.grid_id = gi("fdda.obs_grid_id", 1);
      ob.levidn = gi("fdda.obs_levidn", 0);
      ob.parent_grid_ratio = gi("fdda.obs_parent_grid_ratio", 1);
      ob.file = cfg.get_string("fdda.obs_file", dir + "/OBS_DOMAIN" + std::to_string(ob.grid_id) + "01");
      const int diff_opt = int(cfg.get_int("dyn.diff_opt", 0));
      ob.diff_z = diff_opt == 1 || diff_opt == 2;
      // options without data proving them stay errors (tools/wrf2cppwrf.py says the same)
      auto bad = [](const std::string& m) { throw std::runtime_error("obs nudging: " + m); };
      for (auto [v, n] : {std::pair{ob.nudge_wind, "obs_nudge_wind"}, std::pair{ob.nudge_temp, "obs_nudge_temp"},
                          std::pair{ob.nudge_mois, "obs_nudge_mois"}, std::pair{ob.no_pbl_nudge_uv, "obs_no_pbl_nudge_uv"},
                          std::pair{ob.no_pbl_nudge_t, "obs_no_pbl_nudge_t"}, std::pair{ob.no_pbl_nudge_q, "obs_no_pbl_nudge_q"},
                          std::pair{ob.idynin, "obs_idynin"}, std::pair{ob.scl_neg_qv_innov, "obs_scl_neg_qv_innov"}})
        if (v != 0 && v != 1) bad(std::string(n) + " = " + std::to_string(v) + " is not supported (0 or 1)");
      if (ob.nudge_pstr != 0) bad("obs_nudge_pstr = 1 has no effect in WRF 4.8 and is not supported");
      if (ob.ionf < 1) bad("obs_ionf must be >= 1");
      if (ob.prt_freq <= 0) bad("obs_prt_freq must be greater than zero (WRF stops)");
      if (ob.dpsmx > 10.) bad("obs_dpsmx > 10 (WRF stops, in4dob STOP 557)");
      if (ob.idynin == 1 && ob.dtramp == 0) bad("obs_idynin = 1 with obs_dtramp = 0 (WRF divides by 0)");
      if (ob.max_obs <= 0) bad("max_obs must be > 0 (WRF stops at the first observation: in4dob STOP 122)");
      if (cfg.get_bool("time.adaptive", false)) bad("the adaptive time step (use_adaptive_time_step) is not supported");
      if (ob.grid_id > 9) bad("more than 9 domains: OBS_DOMAIN<id>01 has one digit");
    }
  }
  o.bucket_mm = cfg.get_double("phys.bucket_mm", -1.0);  // diagnostic_output_calc (module_diag_misc.F)
  o.bucket_j = cfg.get_double("phys.bucket_j", -1.0);
  o.prec_acc_dt = cfg.get_double("phys.prec_acc_dt", 0.0);  // this domain's entry (minutes)
  o.prec_acc_opt = cfg.get_bool("phys.prec_acc_opt", o.prec_acc_dt > 0);  // check_a_mundo: some domain's > 0
  o.output_diagnostics = int(cfg.get_int("phys.output_diagnostics", 0));  // clwrf_output_calc (module_diag_cl.F)
  o.auxhist3_interval = int(cfg.get_int("phys.auxhist3_interval", 0));    // this domain's entry (minutes)
  o.gwd_opt = int(cfg.get_int("dyn.gwd_opt", 0));
  o.gwd_dx_factor = cfg.get_double("dyn.gwd_dx_factor", 2.0);
  o.gwd_if_nonhyd = cfg.get_bool("dyn.gwd_if_nonhyd", true);
  o.tofd_factor = cfg.get_double("phys.tofd_factor", 0.003);
  o.mminlu = in.att_str("MMINLU", o.mminlu);
  o.iswater = int(in.att_num("ISWATER", o.iswater));
  o.isice = int(in.att_num("ISICE", o.isice));
  o.isurban = int(in.att_num("ISURBAN", o.isurban));
  o.table_dir = cfg.get_string("phys.table_dir", dir);
  const bool rdmaxalb = cfg.get_bool("phys.rdmaxalb", true);
  {  // radiation (radiation_driver; RRTMG tables in phys.rrtmg_tables or data/rrtmg, ozone and CAMtr from phys.rrtmg_tables)
    auto& r = o.rad;
    r.ra_lw_physics = int(cfg.get_int("phys.ra_lw_physics", 0));
    r.ra_sw_physics = int(cfg.get_int("phys.ra_sw_physics", 0));
    r.radt = cfg.get_double("phys.radt", 0.0);
    r.icloud = int(cfg.get_int("phys.icloud", 1));
    r.cldovrlp = int(cfg.get_int("phys.cldovrlp", 2));
    r.idcor = int(cfg.get_int("phys.idcor", 0));
    r.o3input = int(cfg.get_int("phys.o3input", 2));
    // check_a_mundo: o3input = 0 for radiation schemes other than RRTMG (RRTM LW with the Dudhia SW)
    if (r.ra_lw_physics != 4 && r.ra_sw_physics != 4) r.o3input = 0;
    r.oznini = !cfg.get_bool("real.nested", false);
    r.ghg_input = int(cfg.get_int("phys.ghg_input", 1));
    r.aer_opt = int(cfg.get_int("phys.aer_opt", 0));
    // phy_init (module_physics_init.F:1192-1206): cu_rad_feedback gives icloud_cu = 2 with KF, 1 with the Grell
    // schemes (GF 3, G3 5, GD 93) and 0 with the others (Tiedtke 6, New Tiedtke 16: no feedback)
    if (cfg.get_bool("phys.cu_rad_feedback", false))
      r.icloud_cu = o.cu_physics == 1 ? 2 : (o.cu_physics == 3 || o.cu_physics == 5 || o.cu_physics == 93) ? 1 : 0;
    // MSKF: icloud_cu = 2 whatever cu_rad_feedback (module_physics_init.F:1205; check_a_mundo also sets
    // cu_rad_feedback = .true. and icloud = 1)
    if (o.cu_physics == 11) {
      r.icloud_cu = 2;
      r.icloud = 1;
    }
    r.bmj_feedback = o.bmj_rad_feedback;  // module_radiation_driver.F:1578 (cu_physics == BMJSCHEME)
    r.swrad_scat = cfg.get_double("phys.swrad_scat", 1.0);
    r.warm_rain = o.warm_rain;
    r.julday = int(in.att_num("JULDAY", 1));
    r.julyr = int(in.att_num("JULYR", 0));
    r.gmt = in.att_num("GMT", 0.0);
    r.p_top = g.p_top;
    r.table_dir = cfg.get_string("phys.rrtmg_tables", o.table_dir);
  }
  if (o.sf_surface_physics == 4) {
    auto& nm = o.noahmp;
    nm.dveg = int(cfg.get_int("phys.dveg", nm.dveg));
    nm.opt_crs = int(cfg.get_int("phys.opt_crs", nm.opt_crs));
    nm.opt_btr = int(cfg.get_int("phys.opt_btr", nm.opt_btr));
    nm.opt_runsrf = int(cfg.get_int("phys.opt_runsrf", nm.opt_runsrf));
    nm.opt_runsub = int(cfg.get_int("phys.opt_runsub", nm.opt_runsub));
    nm.opt_sfc = int(cfg.get_int("phys.opt_sfc", nm.opt_sfc));
    nm.opt_frz = int(cfg.get_int("phys.opt_frz", nm.opt_frz));
    nm.opt_inf = int(cfg.get_int("phys.opt_inf", nm.opt_inf));
    nm.opt_rad = int(cfg.get_int("phys.opt_rad", nm.opt_rad));
    nm.opt_alb = int(cfg.get_int("phys.opt_alb", nm.opt_alb));
    nm.opt_snf = int(cfg.get_int("phys.opt_snf", nm.opt_snf));
    nm.opt_tbot = int(cfg.get_int("phys.opt_tbot", nm.opt_tbot));
    nm.opt_stc = int(cfg.get_int("phys.opt_stc", nm.opt_stc));
    nm.opt_gla = int(cfg.get_int("phys.opt_gla", nm.opt_gla));
    nm.opt_rsf = int(cfg.get_int("phys.opt_rsf", nm.opt_rsf));
    nm.opt_soil = int(cfg.get_int("phys.opt_soil", nm.opt_soil));
    nm.opt_pedo = int(cfg.get_int("phys.opt_pedo", nm.opt_pedo));
    nm.opt_crop = int(cfg.get_int("phys.opt_crop", nm.opt_crop));
    nm.opt_irr = int(cfg.get_int("phys.opt_irr", nm.opt_irr));
    nm.opt_irrm = int(cfg.get_int("phys.opt_irrm", nm.opt_irrm));
    nm.opt_infdv = int(cfg.get_int("phys.opt_infdv", nm.opt_infdv));
    nm.opt_tdrn = int(cfg.get_int("phys.opt_tdrn", nm.opt_tdrn));
    nm.opt_tksno = int(cfg.get_int("phys.opt_tksno", nm.opt_tksno));
    nm.opt_compact = int(cfg.get_int("phys.opt_compact", nm.opt_compact));
    nm.opt_scf = int(cfg.get_int("phys.opt_scf", nm.opt_scf));
    nm.opt_wetland = int(cfg.get_int("phys.opt_wetland", nm.opt_wetland));
    nm.sf_urban_physics = int(cfg.get_int("phys.sf_urban_physics", 0));
    nm.iz0tlnd = o.iz0tlnd;
    nm.soiltstep = cfg.get_double("phys.soiltstep", 0.0);
  }
  m.suite = std::make_unique<PhysicsSuite>(g, o);
  PhysicsSuite& su = *m.suite;

  const std::size_t n2 = std::size_t(nx) * ny;
  const int ns = o.num_soil_layers;
  auto put2 = [&](Field2& f, const std::vector<double>& a) { f.fill_host([&](int i, int j, int) { return a[i + std::size_t(nx) * j]; }); };
  auto put_const = [&](Field2& f, double c) { f.fill_host([=](int, int, int) { return c; }); };
  auto rd2 = [&](const char* name) { return in.var(name, n2); };

  // ---- wrfinput fields
  const std::vector<double> lu = rd2("LU_INDEX"), xice = rd2("SEAICE"), isl = rd2("ISLTYP");
  const std::vector<double> tslb = in.var("TSLB", n2 * ns), smois = in.var("SMOIS", n2 * ns);
  std::vector<double> snow = rd2("SNOW"), snowc = rd2("SNOWC"), snoalb = rd2("SNOALB");
  put2(su.tsk, rd2("TSK")); put2(su.tmn, rd2("TMN")); put2(su.xice, xice);
  put2(su.vegfra, rd2("VEGFRA")); put2(su.shdmax, rd2("SHDMAX")); put2(su.shdmin, rd2("SHDMIN"));
  put2(su.snow, snow); put2(su.snowc, snowc); put2(su.lai, rd2("LAI"));
  put2(su.lakemask, in.has("LAKEMASK") ? rd2("LAKEMASK") : std::vector<double>(n2, 0.0));
  // VAR, UOCE, VOCE: mass-grid fields of one record (Registry VAR2D, UOCE, VOCE: ij, no staggering), checked shape
  const std::vector<io_netcdf::DimSpec> mass = {{"Time", 0}, {"south_north", std::size_t(ny)},
                                                {"west_east", std::size_t(nx)}};
  if (in.has("VAR")) put2(su.var2d, in.rec("VAR", mass));  // subgrid orography std. dev. (kim_tofd; history VAR)
  // sea-surface currents (Registry UOCE / VOCE i0124rd: wrfinput, auxinput4, wrfrst); the surface driver subtracts
  // them from the lowest-level wind, YSU and MYNN take them as the ocean's velocity. real.exe writes zeros unless the
  // met_em files hold them
  if (in.has("UOCE")) put2(su.uoce, in.rec("UOCE", mass));
  if (in.has("VOCE")) put2(su.voce, in.rec("VOCE", mass));
  // U_FRAME / V_FRAME (Registry ir, idealized moving frames): the drivers take 0 (u_phytmp = u_phy + u_frame); a
  // run resumed from a wrfrst file checks the file's values as well (DomainRunner::load_restart_file)
  for (const char* n : {"U_FRAME", "V_FRAME"})
    if (in.has(n)) {
      const double f = in.rec(n, {{"Time", 0}})[0];
      if (f != 0.0)
        throw std::runtime_error("real physics: " + in.path() + ": " + n + " = " + io_netcdf::value_str(f) +
                                 ", not 0: a moving frame (u_frame / v_frame) is not supported");
    }
  if (o.gwd_opt == 1) {  // package gwd_used_1
    const std::pair<const char*, Field2*> gw[] = {{"CON", &su.oc12d}, {"OA1", &su.oa1}, {"OA2", &su.oa2}, {"OA3", &su.oa3},
                                                  {"OA4", &su.oa4}, {"OL1", &su.ol1}, {"OL2", &su.ol2}, {"OL3", &su.ol3},
                                                  {"OL4", &su.ol4}, {"ELVMAX", &su.elvmax}};
    for (const auto& [n, f] : gw) {
      if (!in.has(n)) throw std::runtime_error(std::string("gwd_opt = 1: wrfinput lacks ") + n);
      put2(*f, rd2(n));
    }
  }
  if (o.fractional_seaice == 1 || o.sst_update == 1 || o.sst_skin == 1) put2(su.sst, rd2("SST"));
  if (o.fractional_seaice == 1) put2(su.tsk_save, rd2("TSK"));  // phy_init: TSK_SAVE = TSK
  if (o.sst_update == 1 || o.sf_lake_physics == 1) put2(su.xicem, xice);  // landuse_init: XICEM = XICE
  su.init_tmn_update();  // phy_init (tmn_update = 1): TYR, TLAG = the input's TMN, before the LSM's init
  put2(su.water_depth, in.has("WATER_DEPTH") ? rd2("WATER_DEPTH") : std::vector<double>(n2, 0.0));
  const std::pair<const char*, Field2*> diag[] = {{"T2", &su.t2}, {"TH2", &su.th2}, {"Q2", &su.q2},
                                                   {"U10", &su.u10}, {"V10", &su.v10}, {"PSFC", &su.psfc}};
  for (const auto& [n, f] : diag)
    if (in.has(n)) put2(*f, rd2(n));
  if (o.sf_sfclay_physics == 2 || o.sf_sfclay_physics == 4) {  // phy_init: u10e = u10, v10e = v10 for MYJSFC, QNSESFC
    if (in.has("U10")) put2(su.u10e, rd2("U10"));
    if (in.has("V10")) put2(su.v10e, rd2("V10"));
  }
  const std::vector<double> dzs = in.var("DZS", std::size_t(ns));
  for (int k = 0; k < ns; ++k) su.dzs[k] = dzs[k];
  if (o.sf_surface_physics == 1 || o.sf_surface_physics == 3) {  // the slab's diffusion, RUC's levels
    const std::vector<double> zs = in.var("ZS", std::size_t(ns));
    for (int k = 0; k < ns; ++k) su.zs[k] = zs[k];
  }

  // ---- phy_init / myjsfcinit
  put_const(su.ust, o.sf_sfclay_physics == 2 || o.sf_sfclay_physics == 4 ? 0.1 : 0.0001);  // myjsfcinit, qnsesfcinit
  put_const(su.dx2d, g.dx);
  put_const(su.ctopo, 1.0); put_const(su.ctopo2, 1.0);
  // IVGTYP = NINT(LU_INDEX); lsm_init (module_physics_init.F:3438-3448) then gives Noah with the NLCD40 land use one
  // urban category: 23 and 25 become 24 (before LSMINIT, the urban model and every later use of IVGTYP; landuse_init
  // reads LU_INDEX)
  const bool nlcd40_noah = o.sf_surface_physics == 2 && o.mminlu == "NLCD40";
  auto ivgtyp_of = [nlcd40_noah](double lu_index) {
    const int v = int(std::lround(lu_index));
    return nlcd40_noah && (v == 23 || v == 25) ? 24 : v;
  };
  {
    std::vector<int> h(su.ivgtyp.layout().total, 0), hs(su.isltyp.layout().total, 0);
    const Layout& Li = su.ivgtyp.layout();
    for (int j = Li.j0(); j < Li.j1(); ++j)  // the points the allocation holds (a decomposed run: the patch's)
      for (int i = Li.i0(); i < Li.i1(); ++i) {
        h[su.ivgtyp.layout().index(i, j, 0)] = ivgtyp_of(lu[i + std::size_t(nx) * j]);
        hs[su.isltyp.layout().index(i, j, 0)] = int(std::lround(isl[i + std::size_t(nx) * j]));
      }
    su.ivgtyp.upload(h);
    su.isltyp.upload(hs);
  }
  // landuse_init
  const LanduseInit li = landuse_init(o.table_dir, o.mminlu, int(in.att_num("JULDAY", 0)), in.att_num("CEN_LAT", 0.0),
                                      o.iswater, o.isice, lu, xice, snowc, o.fractional_seaice, o.usemonalb,
                                      o.usemonalb ? rd2("ALBBCK") : std::vector<double>(),
                                      o.usemonalb ? snoalb : std::vector<double>());
  const std::vector<double>&albbck = li.albbck, &albedo = li.albedo, &z0 = li.z0, &embck = li.embck,
                            &emiss = li.emiss, &mavail = li.mavail, &xland = li.xland;
  put2(su.albbck, albbck); put2(su.albedo, albedo); put2(su.z0, z0); put2(su.znt, z0);
  put2(su.embck, embck); put2(su.emiss, emiss); put2(su.mavail, mavail); put2(su.xland, xland);
  std::vector<double> xland_phy = xland;  // grid%xland after phy_init (lakeini changes frozen lake points)
  if (o.sf_surface_physics == 1) put2(su.thc, li.thc);

  auto put_soil = [&](Field3& f, const std::vector<double>& a) {
    f.fill_host([&](int i, int j, int k) { return a[i + std::size_t(nx) * j + n2 * k]; });
  };

  if (o.sf_surface_physics == 1) {
    // ---- bl_init / slabinit (no statement): the soil, snow and canopy fields stay as in wrfinput
    put2(su.snoalb, snoalb);
    put_soil(su.tslb, tslb); put_soil(su.smois, smois);
    if (in.has("SH2O")) put_soil(su.sh2o, in.var("SH2O", n2 * ns));
    if (in.has("SMCREL")) put_soil(su.smcrel, in.var("SMCREL", n2 * ns));
    if (in.has("SNOWH")) put2(su.snowh, rd2("SNOWH"));
    if (in.has("CANWAT")) put2(su.canwat, rd2("CANWAT"));
  } else if (o.sf_surface_physics == 3) {
    // ---- bl_init / RUCLSMINIT (not a restart): ZNT from the RUC table, MAVAIL, SH2O and SMFR3D; the snow, canopy
    // and SNOALB fields stay as in wrfinput, as do SOILT1, RHOSNF, SNOWFALLAC and ACRUNOFF (Registry i); LSMRUC's
    // first step initialises the rest
    phys::ruc::RucTables* tb = new phys::ruc::RucTables;
    const std::unique_ptr<phys::ruc::RucTables> tb_own(tb);
    const std::string err = phys::ruc::read_ruc_tables(o.table_dir, o.mminlu, *tb);
    if (!err.empty()) throw std::runtime_error("real physics: " + err);
    put2(su.snoalb, snoalb);
    if (in.has("SNOWH")) put2(su.snowh, rd2("SNOWH"));
    if (in.has("CANWAT")) put2(su.canwat, rd2("CANWAT"));
    const std::pair<const char*, Field2*> ruc_in[] = {
        {"SOILT1", &su.soilt1}, {"RHOSNF", &su.rhosnf}, {"SNOWFALLAC", &su.snowfallac}, {"ACRUNOFF", &su.acrunoff}};
    for (const auto& [n, f] : ruc_in)
      if (in.has(n)) put2(*f, rd2(n));
    std::vector<double> sh2o(n2 * ns), smfr(n2 * ns), mav(mavail), znt(n2);
    for (std::size_t p = 0; p < n2; ++p) {
      const int vt = int(std::lround(lu[p])), st = int(std::lround(isl[p]));
      if (vt < 1 || vt > tb->lucats || st < 1 || st > tb->slcats)
        throw std::runtime_error("real physics: RUCLSMINIT: IVGTYP / ISLTYP out of the RUC tables' range");
      real ts[phys::ruc::kA] = {}, sm[phys::ruc::kA] = {}, sw[phys::ruc::kA] = {}, fr[phys::ruc::kA] = {};
      for (int k = 0; k < ns; ++k) { ts[k + 1] = real(tslb[p + n2 * k]); sm[k + 1] = real(smois[p + n2 * k]); }
      real ma = real(mav[p]), z = real(z0[p]);
      phys::ruc::ruclsminit_point(*tb, ns, vt, st, real(xice[p]), ts, sm, sw, fr, ma, z);
      for (int k = 0; k < ns; ++k) { sh2o[p + n2 * k] = sw[k + 1]; smfr[p + n2 * k] = fr[k + 1]; }
      mav[p] = ma; znt[p] = z;
    }
    put2(su.mavail, mav); put2(su.znt, znt);
    put_soil(su.tslb, tslb); put_soil(su.smois, smois); put_soil(su.sh2o, sh2o); put_soil(su.smfr3d, smfr);
    if (in.has("SMCREL")) put_soil(su.smcrel, in.var("SMCREL", n2 * ns));
  } else if (o.sf_surface_physics == 2) {
    // ---- bl_init / LSMINIT (Noah)
    phys::noah::NoahTables nt;
    if (!phys::noah::read_noah_tables(o.table_dir, o.mminlu, o.mminsl, nt))
      throw std::runtime_error("real physics: cannot read the Noah tables from " + o.table_dir);
    if (!rdmaxalb)
      for (std::size_t p = 0; p < n2; ++p)  // WRF real*4: MAXALB(IVGTYP)*0.01 in REAL
        snoalb[p] = std::is_same_v<real, float> ? double(real(nt.maxalb[ivgtyp_of(lu[p]) - 1]) * real(0.01))
                                                : nt.maxalb[ivgtyp_of(lu[p]) - 1] * 0.01;
    put2(su.snoalb, snoalb);
    std::vector<double> sh2o(n2 * ns);
    for (std::size_t p = 0; p < n2; ++p) {
      const int st = int(std::lround(isl[p]));
      if (st < 1) throw std::runtime_error("real physics: lsminit: out of range value of ISLTYP");
      real bx = nt.bb[st - 1];
      const real smcmax = nt.maxsmc[st - 1], psisat = nt.satpsi[st - 1];
      for (int k = 0; k < ns; ++k) {
        const std::size_t q = p + n2 * k;
        if (bx > 0 && smcmax > 0 && psisat > 0) {
          if (std::is_same_v<real, float> ? real(tslb[q]) < real(273.149) : tslb[q] < 273.149) {  // REAL TSLB
            bx = nt.bb[st - 1];
            if (bx > real(5.5)) bx = real(5.5);
            const real t = real(tslb[q]);
            real fk = libm::pow((real(3.335e5) / (real(9.81) * (-psisat))) * ((t - real(273.15)) / t), real(-1) / bx) * smcmax;
            if (fk < real(0.02)) fk = real(0.02);
            real s = std::min(fk, real(smois[q]));
            real free_w = 0;
            phys::noah::NoahKernel::frh2o(free_w, t, real(smois[q]), s, smcmax, bx, psisat);
            sh2o[q] = free_w;
          } else {
            sh2o[q] = smois[q];
          }
        } else {
          sh2o[q] = smois[q];
        }
      }
    }
    put_soil(su.tslb, tslb); put_soil(su.smois, smois); put_soil(su.sh2o, sh2o);
    if (in.has("SMCREL")) put_soil(su.smcrel, in.var("SMCREL", n2 * ns));
    // phy_init passes FNDSNOWH = .true. to LSMINIT: SNOWH is never derived from SNOW (0 when not in the input)
    if (in.has("SNOWH")) put2(su.snowh, rd2("SNOWH"));
    // phy_init: urban_param_init (the PhysicsSuite constructor) and urban_var_init after LSMINIT
    if (su.urb_) {
      // input_wrf.F:805-822: the wrfinput's SF_URBAN_PHYSICS attribute must equal the namelist's value
      const double att = in.att_num("SF_URBAN_PHYSICS", -1.0);
      if (att >= 0 && int(att) != o.sf_urban_physics)
        throw std::runtime_error("---- ERROR: Mismatch between namelist and global attribute SF_URBAN_PHYSICS "
                                 "(namelist " + std::to_string(o.sf_urban_physics) + ", wrfinput " +
                                 std::to_string(int(att)) + ")");
      put2(su.urb_->xlat_urb, m.xlat);
      su.init_urban(in.has("FRC_URB2D") ? rd2("FRC_URB2D") : std::vector<double>(),
                    in.has("HGT_URB2D") ? rd2("HGT_URB2D") : std::vector<double>());
    }
  } else if (o.sf_surface_physics == 4) {
    // ---- bl_init / NoahmpWRFinit (Noah-MP)
    put2(su.snoalb, snoalb);
    for (std::size_t p = 0; p < n2; ++p)  // NoahmpInitMain: stop on any ISLTYP < 1
      if (std::lround(isl[p]) < 1) throw std::runtime_error("real physics: lsminit: out of range ISLTYP");
    put2(su.xlat_sfc, m.xlat);
    put2(su.xlong_sfc, m.xlong);
    put_soil(su.tslb, tslb);
    put_soil(su.smois, smois);
    if (in.has("SH2O")) put_soil(su.sh2o, in.var("SH2O", n2 * ns));
    if (in.has("SNOWH")) put2(su.snowh, rd2("SNOWH"));
    su.init_noahmp(in.has("SNOWH"));
  }
  if (su.lake_) {
    // bl_init's lakeini after the land surface model's init. The proven inputs: LAKEFLAG = 1 (real.exe with the ISLAKE
    // category: LAKEMASK from LU_INDEX) and no lake depth data in use (LAKE_DEPTH_FLAG = 0, or use_lakedepth = 0)
    auto scalar = [&](const char* n) {
      if (!in.has(n)) throw std::runtime_error(std::string("sf_lake_physics = 1: wrfinput lacks ") + n);
      return int(std::lround(in.var(n, 1)[0]));
    };
    const int lakeflag = scalar("LAKEFLAG"), lake_depth_flag = scalar("LAKE_DEPTH_FLAG");
    if (lakeflag != 1)
      throw std::runtime_error("sf_lake_physics = 1 with LAKEFLAG = " + std::to_string(lakeflag) +
                               " (an input without the ISLAKE land-use category: lakes from IVGTYP and lake_min_elev) "
                               "is not supported (not proven against a WRF run)");
    if (lake_depth_flag == 1 && o.use_lakedepth == 1)
      throw std::runtime_error("sf_lake_physics = 1 with LAKE_DEPTH from the input (LAKE_DEPTH_FLAG = 1, use_lakedepth "
                               "= 1) is not supported (not proven against a WRF run): set use_lakedepth = 0");
    put2(su.lake_->xlat, m.xlat);
    su.init_lake(rd2("HGT"), in.has("LAKE_DEPTH") ? rd2("LAKE_DEPTH") : std::vector<double>(), lakeflag,
                 lake_depth_flag, &xland_phy);
  }
  // CANWAT = 0 (lsminit / NoahmpInitMain; the slab's from wrfinput), the accumulations and fluxes stay zero
  su.init_radiation(m.xlat, m.xlong);
  if (su.obs_) {  // fddaobs_init's inputs: wrfinput's base state, XLAT(1,1), XLONG(1,1) and projection
    auto& ip = su.obs_->in;
    ip.p00 = in.var("P00", 1)[0];
    ip.t00 = in.var("T00", 1)[0];
    ip.tlp = in.var("TLP", 1)[0];
    ip.p_top = m.base->p_top;
    ip.znu.resize(std::size_t(g.nz));
    for (int k = 0; k < g.nz; ++k) ip.znu[k] = m.base->znu[k];
    ip.xlat11 = m.xlat.at(0);
    ip.xlong11 = m.xlong.at(0);
    ip.map_proj = int(in.att_num("MAP_PROJ", 0));
    ip.truelat1 = in.att_num("TRUELAT1", 0);
    ip.truelat2 = in.att_num("TRUELAT2", 0);
    ip.stand_lon = in.att_num("STAND_LON", 0);
  }
  if (o.topo_wind != 0 || su.topo_) {
    // start_em (not a restart): CTOPO / CTOPO2 and the slope-radiation arrays from the terrain and phy_init's XLAND
    if (o.topo_wind == 1 && !(in.has("GOT_VAR_SSO") && in.var("GOT_VAR_SSO", 1)[0] != 0.0))
      throw std::runtime_error("topo_wind requires VAR_SSO data");  // start_em.F:236-239 (grid%got_var_sso)
    const std::vector<double> zero(n2, 0.0);
    su.init_topo(rd2("HGT"), rd2("MAPFAC_MX"), rd2("MAPFAC_MY"), rd2("SINALPHA"), rd2("COSALPHA"), m.xlat, m.xlong,
                 xland_phy, in.has("VAR_SSO") ? rd2("VAR_SSO") : zero, in.has("VAR") ? in.rec("VAR", mass) : zero);
  }
  if (auto* pc = m.rk->physics(); pc && pc->params().microp_re)
    su.set_effective_radii(&pc->re_cloud, &pc->re_ice, pc->p3() ? nullptr : &pc->re_snow);  // P3: has_reqs = 0
  if (auto* pc = m.rk->physics(); pc && pc->params().mp_physics != 0) {
    const bool has_ice_precip = pc->ice_precip();
    su.set_microphysics_accum(&pc->rainnc);
    su.set_microphysics_precip(&pc->rainncv, has_ice_precip ? &pc->sr : nullptr,
                               has_ice_precip ? &pc->snowncv : nullptr,
                               has_ice_precip ? &pc->graupelncv : nullptr, pc->nssl() ? &pc->hailncv : nullptr);
    if (!has_ice_precip) pc->set_grid_sr(&su.sr);  // Kessler: solve_em.F:3852 zeroes the suite's grid%sr
  }
  m.rk->set_column_physics(m.suite.get());
}

// share/mediation_integrate.F:791-856: med_nest_initial feeds the nest back (feedback /= 0), then calls
// start_domain(parent); the parent's itimestep is 0 at its first step, so start_em runs phy_init again
// (start_em.F:246-247, 1033) on the fed-back LU_INDEX, XICE, SNOWC, TSK and IVGTYP: landuse_init's ALBBCK, ALBEDO,
// Z0, ZNT, EMBCK, EMISS, MAVAIL, XLAND (and the slab's THC), which have no Registry feedback function, TSK_SAVE
// (fractional_seaice), XICEM (sst_update) and Noah's LSMINIT SNOALB (rdmaxalb = .false.). LSMINIT's SH2O is a
// function of the point's ISLTYP, TSLB and SMOIS, which are fed back from the same nest point as SH2O (copy_fcni /
// copy_fcnm): it is the nest's value already. Pointwise: each worker its patch.
void phy_init_after_feedback(const io::Config& cfg, Model& m) {
  if (!m.suite) return;
  phys::driver::PhysicsSuite& su = *m.suite;
  const phys::driver::SuiteOptions& o = su.options();
  if (o.sf_surface_physics == 3 || o.sf_surface_physics == 4 || su.urb_ || su.lake_)  // not ported for nests (refused)
    throw std::runtime_error("phy_init after the nest feedback: RUC, Noah-MP, the UCM and the lake model are not ported "
                             "for nests");
  // the model keeps no LU_INDEX of its own (IVGTYP stands for it below): Noah's NLCD40 remapping of IVGTYP (23, 25 ->
  // 24, init_real_physics) would give landuse_init other categories than WRF's fed-back LU_INDEX
  if (o.sf_surface_physics == 2 && o.mminlu == "NLCD40")
    throw std::runtime_error("two-way nest feedback with Noah (sf_surface_physics = 2) and the NLCD40 land use is not "
                             "supported (phy_init after the feedback needs the fed-back LU_INDEX, which CPPWRF does not "
                             "keep apart from the remapped IVGTYP)");
  const Grid& g = m.g;
  const int nx = g.nx;
  const std::size_t n2 = std::size_t(nx) * g.ny;
  const std::string dir = cfg.get_string("real.dir");
  const NcIn in(dir + "/" + cfg.get_string("real.wrfinput", "wrfinput_d01"));
  // the patch's points of a field (the other points stay 0: landuse_init takes category 0 as water)
  auto get2 = [&](const Field2& f) {
    std::vector<real> h;
    f.download(h);
    const Layout& L = f.layout();
    std::vector<double> a(n2, 0.0);
    for (int j = L.j0(); j < L.j1(); ++j)
      for (int i = L.i0(); i < L.i1(); ++i) a[i + std::size_t(nx) * j] = double(h[L.index(i, j, 0)]);
    return a;
  };
  std::vector<double> lu(n2, 0.0);
  {
    std::vector<int> h;
    su.ivgtyp.download(h);
    const Layout& L = su.ivgtyp.layout();
    for (int j = L.j0(); j < L.j1(); ++j)
      for (int i = L.i0(); i < L.i1(); ++i) lu[i + std::size_t(nx) * j] = double(h[L.index(i, j, 0)]);
  }
  auto put2 = [&](Field2& f, const std::vector<double>& a) {  // the patch's points; the halo points as they are
    std::vector<real> h;
    f.download(h);
    const Layout& L = f.layout();
    for (int j = L.j0(); j < L.j1(); ++j)
      for (int i = L.i0(); i < L.i1(); ++i) h[L.index(i, j, 0)] = real(a[i + std::size_t(nx) * j]);
    copy_to_dev(f.data() - L.offset, h.data(), f.bytes());
  };
  // the model's IVGTYP is LU_INDEX (init_real_physics), fed back from the same nest point
  const LanduseInit li = landuse_init(o.table_dir, o.mminlu, int(in.att_num("JULDAY", 0)), in.att_num("CEN_LAT", 0.0),
                                      o.iswater, o.isice, lu, get2(su.xice), get2(su.snowc), o.fractional_seaice,
                                      o.usemonalb, o.usemonalb ? get2(su.albbck) : std::vector<double>(),
                                      o.usemonalb ? get2(su.snoalb) : std::vector<double>());
  put2(su.albbck, li.albbck); put2(su.albedo, li.albedo); put2(su.z0, li.z0); put2(su.znt, li.z0);
  put2(su.embck, li.embck); put2(su.emiss, li.emiss); put2(su.mavail, li.mavail); put2(su.xland, li.xland);
  if (o.sf_surface_physics == 1) put2(su.thc, li.thc);
  if (o.fractional_seaice == 1) su.tsk_save.copy_from(su.tsk);  // phy_init: TSK_SAVE = TSK
  if (o.sst_update == 1) su.xicem.copy_from(su.xice);           // landuse_init: XICEM = XICE
  if (o.sf_surface_physics == 2 && !cfg.get_bool("phys.rdmaxalb", true)) {  // LSMINIT: SNOALB = MAXALB(IVGTYP) / 100
    phys::noah::NoahTables nt;
    if (!phys::noah::read_noah_tables(o.table_dir, o.mminlu, o.mminsl, nt))
      throw std::runtime_error("real physics: cannot read the Noah tables from " + o.table_dir);
    std::vector<double> snoalb = get2(su.snoalb);
    const Layout& L = su.snoalb.layout();
    for (int j = L.j0(); j < L.j1(); ++j)
      for (int i = L.i0(); i < L.i1(); ++i) {
        const std::size_t p = i + std::size_t(nx) * j;
        snoalb[p] = nt.maxalb[int(std::lround(lu[p])) - 1] * 0.01;
      }
    put2(su.snoalb, snoalb);
  }
}

// dyn_em/start_em.F:1007-1101 for a moved nest (share/mediation_nest_move.F:264-271, WRF dmpar): set_tiles with
// imask_nostag gives phy_init (phys/module_physics_init.F, moved = .TRUE., restart = .FALSE., start_of_simulation =
// .FALSE.) the new points only. Ported for mp_physics 0 / 6, cu_physics 0 / 1, bl_pbl_physics 0 / 1,
// sf_sfclay_physics 0 / 1, sf_surface_physics 0 / 2, ra_lw / ra_sw_physics 0 / 4 without the options that change
// phy_init (urban, fractional_seaice, sst_update, tmn_update, usemonalb, the terrain options); there:
//  - 1256-1278 (IF .NOT. moved: XLAND, GSW, UST, ...) is skipped; IVGTYP = NINT(LU_INDEX); the XLAND < 1.5 defaults
//    are overwritten by landuse_init (1423-1437: ALBBCK, ALBEDO, Z0, ZNT, EMBCK, EMISS, MAVAIL, XLAND, THC);
//    compute_2d_dx_area: DX2D = dx; RAINCV_A = RAINCV_B = 0; RE_CLOUD, RE_ICE, RE_SNOW = 2.49e-6, 4.99e-6, 9.99e-6
//    (has_reqc / has_reqi / has_reqs);
//  - ra_init, mp_init (WSM6): nothing without start_of_simulation;
//  - bl_init: ACHFX = ACLHF = ACGRDFLX = SFCEVP = 0; sfclayinit, ysuinit: nothing; Noah (lsm_init's NLCD40 IVGTYP
//    23, 25 -> 24, LSMINIT): SNOALB = MAXALB(IVGTYP) / 100 without rdmaxalb, SH2O from TSLB, SMOIS and ISLTYP,
//    CANWAT = 0 (FNDSNOWH: SNOWH stays);
//  - cu_init (start_of_simulation false: RAINC, RAINCV stay), kf_eta_init: RTHCUTEN, RQVCUTEN, RQCCUTEN, RQRCUTEN,
//    RQICUTEN (P_QI), RQSCUTEN (P_QS) = 0, NCA = -100, W0AVG = 0.
void check_phy_init_moved(const io::Config& cfg, const Model& m, const std::string& tag) {
  // phy_init on the new points runs for any physics (WRF calls it without column physics too); the column-physics
  // suite (PhysicsSuite) holds the fields it sets for the options below only
  if (!m.suite)
    throw std::runtime_error(tag + ": moving nests need the column physics (sf_sfclay_physics, sf_surface_physics, "
                             "bl_pbl_physics or cu_physics != 0): phy_init of a moved nest without them is not ported");
  const phys::driver::PhysicsSuite& su = *m.suite;
  const phys::driver::SuiteOptions& o = su.options();
  auto in = [](long v, std::initializer_list<long> ok) { return std::find(ok.begin(), ok.end(), v) != ok.end(); };
  const long mp = cfg.get_int("phys.mp_physics", 0);
  if (!in(mp, {0, 6}) || !in(o.cu_physics, {0, 1}) || !in(o.bl_pbl_physics, {0, 1}) ||
      !in(o.sf_sfclay_physics, {0, 1}) || !in(o.sf_surface_physics, {0, 2}) || !in(o.rad.ra_lw_physics, {0, 4}) ||
      !in(o.rad.ra_sw_physics, {0, 4}) || o.sf_urban_physics != 0 || su.urb_ || o.fractional_seaice != 0 ||
      o.sst_update != 0 || o.tmn_update != 0 || o.usemonalb || o.topo_wind != 0 || o.slope_rad != 0 ||
      o.topo_shading != 0 || o.gwd_opt != 0)
    throw std::runtime_error(tag + ": moving nests: phy_init of a moved nest is ported for mp_physics 0 / 6, "
                             "cu_physics 0 / 1, bl_pbl_physics 0 / 1, sf_sfclay_physics 0 / 1, sf_surface_physics 0 / "
                             "2, ra_lw / ra_sw_physics 0 / 4 without urban, fractional_seaice, sst_update, tmn_update, "
                             "usemonalb, gwd_opt and the terrain options");
  // phy_init's fg_init (module_physics_init.F:4961-4986) -> fddagdinit zeroes the nudging tendencies of the new
  // points (module_fdda_psufddagd.F:1516-1530); not ported
  if (o.fdda.grid_fdda != 0)
    throw std::runtime_error(tag + ": moving nests with grid_fdda (analysis nudging) are not supported");
}

std::vector<double> phy_init_moved(const io::Config& cfg, Model& m, const std::vector<unsigned char>& strip,
                                   const std::vector<double>& lu_index) {
  check_phy_init_moved(cfg, m, "moving nests");
  phys::driver::PhysicsSuite& su = *m.suite;
  const phys::driver::SuiteOptions& o = su.options();
  const Grid& g = m.g;
  const int nx = g.nx, ny = g.ny;
  const std::size_t n2 = std::size_t(nx) * ny;
  if (strip.size() != n2 || lu_index.size() != n2) throw std::logic_error("phy_init_moved: the arrays' sizes");
  auto each = [&](auto fn) {
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i)
        if (strip[std::size_t(j) * nx + i]) fn(i, j, std::size_t(j) * nx + i);
  };
  auto get2 = [&](const Field2& f) {
    std::vector<real> h;
    f.download(h);
    std::vector<double> a(n2, 0.0);
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) a[std::size_t(j) * nx + i] = double(h[f.layout().index(i, j, 0)]);
    return a;
  };
  auto put2 = [&](Field2& f, const std::vector<double>& a) {  // the new points
    std::vector<real> h;
    f.download(h);
    each([&](int i, int j, std::size_t p) { h[f.layout().index(i, j, 0)] = real(a[p]); });
    copy_to_dev(f.data() - f.layout().offset, h.data(), f.bytes());
  };
  auto set2 = [&](Field2& f, double v) { put2(f, std::vector<double>(n2, v)); };
  auto set3 = [&](Field3& f, double v) {  // the mass levels kts..ktf of the new points
    if (f.layout().total == 0) return;
    std::vector<real> h;
    f.download(h);
    const Layout& L = f.layout();
    for (int k = 0; k < std::min(L.nz, g.nz); ++k) each([&](int i, int j, std::size_t) { h[L.index(i, j, k)] = real(v); });
    copy_to_dev(f.data() - L.offset, h.data(), f.bytes());
  };

  // IVGTYP = NINT(LU_INDEX); Noah with NLCD40: 23, 25 -> 24 (lsm_init)
  const bool nlcd40_noah = o.sf_surface_physics == 2 && o.mminlu == "NLCD40";
  std::vector<int> ivg;
  su.ivgtyp.download(ivg);
  each([&](int i, int j, std::size_t p) {
    const int v = int(std::lround(lu_index[p]));
    ivg[su.ivgtyp.layout().index(i, j, 0)] = nlcd40_noah && (v == 23 || v == 25) ? 24 : v;
  });
  su.ivgtyp.upload(ivg);

  // landuse_init on the moved LU_INDEX, XICE and SNOWC (julday, cen_lat: the nest's wrfinput, as phy_init's)
  // (read once per file: a moving nest's phy_init runs at every move)
  int julday = 0;
  double cen_lat = 0.0;
  {
    static std::mutex mu;
    static std::map<std::string, std::pair<int, double>> cache;
    const std::string path = cfg.get_string("real.dir") + "/" + cfg.get_string("real.wrfinput", "wrfinput_d01");
    const std::lock_guard<std::mutex> lk(mu);
    auto it = cache.find(path);
    if (it == cache.end()) {
      const NcIn inp(path);
      it = cache.emplace(path, std::make_pair(int(inp.att_num("JULDAY", 0)), inp.att_num("CEN_LAT", 0.0))).first;
    }
    julday = it->second.first;
    cen_lat = it->second.second;
  }
  std::vector<double> lu(n2, 0.0);
  each([&](int, int, std::size_t p) { lu[p] = lu_index[p]; });
  const LanduseInit li = landuse_init(o.table_dir, o.mminlu, julday, cen_lat,
                                      o.iswater, o.isice, lu, get2(su.xice), get2(su.snowc), o.fractional_seaice,
                                      false, std::vector<double>(), std::vector<double>());
  put2(su.albbck, li.albbck); put2(su.albedo, li.albedo); put2(su.z0, li.z0); put2(su.znt, li.z0);
  put2(su.embck, li.embck); put2(su.emiss, li.emiss); put2(su.mavail, li.mavail); put2(su.xland, li.xland);
  set2(su.dx2d, g.dx);
  set2(su.raincv_a, 0.0);
  set2(su.raincv_b, 0.0);
  if (Field3* f = su.re_cloud_field()) set3(*f, 2.49e-6);  // RE_QC_BG
  if (Field3* f = su.re_ice_field()) set3(*f, 4.99e-6);    // RE_QI_BG
  if (Field3* f = su.re_snow_field()) set3(*f, 9.99e-6);   // RE_QS_BG

  // bl_init
  set2(su.achfx, 0.0); set2(su.aclhf, 0.0); set2(su.acgrdflx, 0.0); set2(su.sfcevp, 0.0);
  if (o.sf_surface_physics == 2) {  // LSMINIT
    phys::noah::NoahTables nt;
    if (!phys::noah::read_noah_tables(o.table_dir, o.mminlu, o.mminsl, nt))
      throw std::runtime_error("real physics: cannot read the Noah tables from " + o.table_dir);
    std::vector<int> isl;
    su.isltyp.download(isl);
    each([&](int i, int j, std::size_t) {
      if (isl[su.isltyp.layout().index(i, j, 0)] < 1)
        throw std::runtime_error("module_sf_noahdrv.F: lsminit: out of range value of ISLTYP");
    });
    if (!cfg.get_bool("phys.rdmaxalb", true)) {
      std::vector<double> snoalb(n2, 0.0);
      each([&](int i, int j, std::size_t p) {
        const int vt = ivg[su.ivgtyp.layout().index(i, j, 0)];
        snoalb[p] = std::is_same_v<real, float> ? double(real(nt.maxalb[vt - 1]) * real(0.01)) : nt.maxalb[vt - 1] * 0.01;
      });
      put2(su.snoalb, snoalb);
    }
    std::vector<real> tslb, smois, sh2o;
    su.tslb.download(tslb);
    su.smois.download(smois);
    su.sh2o.download(sh2o);
    const Layout& L = su.sh2o.layout();
    each([&](int i, int j, std::size_t) {
      const int st = isl[su.isltyp.layout().index(i, j, 0)];
      real bx = nt.bb[st - 1];
      const real smcmax = nt.maxsmc[st - 1], psisat = nt.satpsi[st - 1];
      for (int k = 0; k < L.nz; ++k) {
        const std::size_t q = L.index(i, j, k);
        if (bx > 0 && smcmax > 0 && psisat > 0) {
          if (tslb[q] < real(273.149)) {
            bx = nt.bb[st - 1];
            if (bx > real(5.5)) bx = real(5.5);
            const real t = tslb[q];
            real fk = libm::pow((real(3.335e5) / (real(9.81) * (-psisat))) * ((t - real(273.15)) / t), real(-1) / bx) * smcmax;
            if (fk < real(0.02)) fk = real(0.02);
            real s = std::min(fk, smois[q]);
            real free_w = 0;
            phys::noah::NoahKernel::frh2o(free_w, t, smois[q], s, smcmax, bx, psisat);
            sh2o[q] = free_w;
          } else {
            sh2o[q] = smois[q];
          }
        } else {
          sh2o[q] = smois[q];
        }
      }
    });
    copy_to_dev(su.sh2o.data() - L.offset, sh2o.data(), su.sh2o.bytes());
    set2(su.canwat, 0.0);
  }
  if (o.cu_physics == 1) {  // kf_eta_init
    const dyn::State& s = *m.st;
    set3(su.pt.rthcuten, 0.0); set3(su.pt.rqvcuten, 0.0); set3(su.pt.rqccuten, 0.0); set3(su.pt.rqrcuten, 0.0);
    if (s.P_QI >= 0) set3(su.pt.rqicuten, 0.0);
    if (s.P_QS >= 0) set3(su.pt.rqscuten, 0.0);
    set2(su.nca, -100.0);
    set3(su.w0avg, 0.0);
  }
  dev_sync();
  return li.thc;  // landuse_init's THC (a history field without a model array: the caller stores the new points)
}
#endif

}  // namespace cppwrf::cases
