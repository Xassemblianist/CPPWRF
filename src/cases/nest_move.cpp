// SPDX-License-Identifier: GPL-3.0-or-later
#include "cases/nest_move.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include "core/backend.hpp"
#include "core/data_dir.hpp"
#include "core/strings.hpp"
#include "core/tiles.hpp"
#include "dyn/state.hpp"
#include "nest/nest_move.hpp"
#include "phys/driver/suite.hpp"

#ifdef CPPWRF_HAVE_NETCDF
#include <netcdf.h>

#include "cases/wrf_history.hpp"
#endif

namespace cppwrf::cases::nestmove {

namespace {

// ---------------------------------------------------------------- the field tracker (tile::track)
thread_local FieldRegistry* g_cur = nullptr;
thread_local const tile::Track* g_prev = nullptr;

std::vector<FieldRegistry*>& registries() {
  static std::vector<FieldRegistry*> r;
  return r;
}

// the registries are shared by the domains' threads (fields made, freed and swapped concurrently)
std::mutex& reg_mutex() {
  static std::mutex m;
  return m;
}

void track_add(const void* base, const Layout& L, int esz, const std::string& name) {
  if (g_cur) {
    std::lock_guard<std::mutex> lk(reg_mutex());
    g_cur->fields.push_back(Tracked{const_cast<void*>(base), L, esz, name});
  }
  if (g_prev) g_prev->add(base, L, esz, name);
}

void track_remove(const void* base) {
  {
    std::lock_guard<std::mutex> lk(reg_mutex());
    for (FieldRegistry* r : registries()) {
      auto& f = r->fields;
      f.erase(std::remove_if(f.begin(), f.end(), [&](const Tracked& t) { return t.base == base; }), f.end());
    }
  }
  if (g_prev) g_prev->remove(base);
}

// two fields exchanged their allocations (a scalar's time levels: ScalarTransport swaps q_2 with its work array):
// each name stays with its field object, so the tracked addresses are exchanged
void track_swapped(const void* a, const void* b) {
  if (a != b) {
    std::lock_guard<std::mutex> lk(reg_mutex());
    for (FieldRegistry* r : registries())
      for (Tracked& t : r->fields) {
        if (t.base == a) t.base = const_cast<void*>(b);
        else if (t.base == b) t.base = const_cast<void*>(a);
      }
  }
  if (g_prev && g_prev->swapped) g_prev->swapped(a, b);
}

const tile::Track kTrack{&track_add, &track_remove, &track_swapped};

bool debug() {
  static const bool d = [] {
    const char* e = std::getenv("CPPWRF_MOVE_DEBUG");
    return e && *e && std::string(e) != "0";
  }();
  return d;
}

std::vector<int> int_list(const std::string& s, const std::string& key) {
  std::vector<int> r;
  std::stringstream ss(s);
  std::string t;
  while (std::getline(ss, t, ',')) {
    t = trim(t);
    if (t.empty()) continue;
    try {
      std::size_t pos = 0;
      r.push_back(std::stoi(t, &pos));
      if (pos != t.size()) throw std::invalid_argument(t);
    } catch (const std::exception&) {
      throw std::runtime_error(key + " = " + s + ": a comma-separated list of integers");
    }
  }
  return r;
}

}  // namespace

FieldRegistry::FieldRegistry() {
  std::lock_guard<std::mutex> lk(reg_mutex());
  registries().push_back(this);
}
FieldRegistry::~FieldRegistry() {
  std::lock_guard<std::mutex> lk(reg_mutex());
  auto& r = registries();
  r.erase(std::remove(r.begin(), r.end(), this), r.end());
}

Install::Install() : saved_(tile::track) {
  if (tile::track == &kTrack) throw std::logic_error("nestmove::Install: installed twice");
  g_prev = tile::track;
  tile::track = &kTrack;
}
Install::~Install() {
  tile::track = static_cast<const tile::Track*>(saved_);
  g_prev = nullptr;
}

Scope::Scope(FieldRegistry* reg) : saved_(g_cur) { g_cur = reg; }
Scope::~Scope() { g_cur = saved_; }

// ---------------------------------------------------------------- the namelist's moves
Moves parse_moves(const io::Config& root, int max_dom) {
  Moves mv;
  constexpr int max_moves = 50;  // frame/module_driver_constants.F
  mv.num_moves = int(root.get_int("domains.num_moves", 0));
  mv.corral_dist.assign(std::size_t(std::max(1, max_dom)), 8);  // Registry default
  for (int id = 1; id <= max_dom; ++id) {
    char tag[16];
    std::snprintf(tag, sizeof tag, "d%02d", id);
    for (const std::string k : {std::string(tag) + ".corral_dist", "domains." + std::string(tag) + "_corral_dist"})
      if (root.has(k)) {
        mv.corral_dist[std::size_t(id - 1)] = int(root.get_int(k));
        // a nest nearer domain 1's edge than 2 parent cells reads WRF's intermediate grid beyond the parent (not
        // verified; tools/wrf2cppwrf.py: i_parent_start >= 3)
        if (mv.corral_dist[std::size_t(id - 1)] < 1) throw std::runtime_error(k + " < 1 is not supported");
      }
  }
  if (mv.num_moves <= 0) {
    mv.num_moves = 0;
    return mv;
  }
  // time_for_move2: num_moves > max_moves stops the run when a nest asks for a move
  if (mv.num_moves > max_moves)
    throw std::runtime_error("domains.num_moves = " + std::to_string(mv.num_moves) + " > max_moves (50)");
  auto list = [&](const char* name, int def) {
    const std::string key = std::string("domains.") + name;
    std::vector<int> v = root.has(key) ? int_list(root.get_string(key), key) : std::vector<int>{};
    // Registry defaults of the entries the namelist does not give
    if (int(v.size()) < mv.num_moves) v.resize(std::size_t(mv.num_moves), def);
    v.resize(std::size_t(mv.num_moves));
    return v;
  };
  mv.move_id = list("move_id", 0);
  mv.move_interval = list("move_interval", 999999999);
  mv.move_cd_x = list("move_cd_x", 0);
  mv.move_cd_y = list("move_cd_y", 0);
  for (int i = 0; i < mv.num_moves; ++i) {
    const int iv = mv.move_interval[std::size_t(i)], id = mv.move_id[std::size_t(i)];
    if (iv < 0)
      throw std::runtime_error("domains.move_interval = " + std::to_string(iv) + " < 0 (minutes from the start) is not "
                               "supported");
    if (iv < 999999999 && id >= 2 && id <= max_dom) mv.active = true;
  }
  return mv;
}

// ---------------------------------------------------------------- time_for_move
bool time_for_move(Moves& mv, const Domain& parent, const Domain& nest, const wrf_time::Interval& now, int& dx,
                   int& dy) {
  dx = dy = 0;
  if (debug()) std::fprintf(stderr, "moving nests: time_for_move d%02d at %lld s (moves %d)\n", nest.id, (long long)now.S, mv.num_moves);
  // time_for_move2 (prescribed moves): the first move of this nest whose time has come, used once
  // (nl_set_move_interval( i, 999999999 ))
  bool t2 = false;
  for (int i = 0; i < mv.num_moves; ++i) {
    if (mv.move_id[std::size_t(i)] != nest.id) continue;
    const int iv = mv.move_interval[std::size_t(i)];
    if (iv >= 999999999) continue;
    if (now >= wrf_time::make(std::int64_t(iv) * 60)) {  // ct .GE. st + ti (WRFU_TimeIntervalSet( ti, M=... ))
      dx = mv.move_cd_x[std::size_t(i)];
      dy = mv.move_cd_y[std::size_t(i)];
      mv.move_interval[std::size_t(i)] = 999999999;
      t2 = true;
      break;
    }
  }
  if (!t2) return false;
  // the innermost nest: would the parent (domain 1, which cannot move) have to move? (corral_dist)
  const int corral = mv.corral_dist[std::size_t(nest.id - 1)];
  const Grid& ng = nest.m->g;
  const Grid& cg = parent.m->g;
  const int nwe = ng.nx + 1, nsn = ng.ny + 1;  // nide - nids + 1
  const int cids = 1, cide = cg.nx + 1, cjds = 1, cjde = cg.ny + 1;
  const int pgr = nest.ratio;
  const int dw = nest.i_parent_start + dx - cids;
  const int ds = nest.j_parent_start + dy - cjds;
  const int de = cide - (nest.i_parent_start + nwe / pgr + dx);
  const int dn = cjde - (nest.j_parent_start + nsn / pgr + dy);
  int wx = 0, wy = 0;
  if (dw <= corral) wx -= 1;
  if (de <= corral) wx += 1;
  if (ds <= corral) wy -= 1;
  if (dn <= corral) wy += 1;
  if (wx != 0 && dx != 0) {
    std::fprintf(stderr, "MOAD can not move. Cancelling nest move in X\n");
    dx = 0;
  }
  if (wy != 0 && dy != 0) {
    std::fprintf(stderr, "MOAD can not move. Cancelling nest move in Y\n");
    dy = 0;
  }
  return dx != 0 || dy != 0;
}

// ---------------------------------------------------------------- the window the interpolation reads
// med_interp_domain's intermediate domain (external/RSL_LITE/module_dm.F:480-483, 625-660; one task; shw = 2): c_ids =
// i_parent_start - 2, c_ide = c_ids + e_we / ratio + 5, its patch and tile 2 more points each way; interp_mask_water_field
// searches up to that tile (cits = i_parent_start - 4 .. cite = i_parent_start + e_we / ratio + 5), SINT reads 2
// parent points beyond the nest. CPPWRF serves domain 1's mass points 1 .. e_we - 1 only (no halo of the parent's
// history fields), so the nest must keep the tile inside them.
void check_window(const Grid& cg, const Grid& ng, int ratio, int ipos, int jpos, const std::string& what) {
  const int cide = cg.nx + 1, cjde = cg.ny + 1, nide = ng.nx + 1, njde = ng.ny + 1;
  const bool ok = ipos - 4 >= 1 && jpos - 4 >= 1 && ipos + nide / ratio + 5 <= cide - 1 &&
                  jpos + njde / ratio + 5 <= cjde - 1;
  if (!ok)
    throw std::runtime_error("moving nests: " + what + " at i_parent_start " + std::to_string(ipos) +
                             ", j_parent_start " + std::to_string(jpos) + " is too near domain 1's edge: i_parent_start, "
                             "j_parent_start >= 5 and i_parent_start + e_we / parent_grid_ratio + 5 <= e_we - 1 of "
                             "domain 1 (and in y) are supported (corral_dist keeps a moving nest there)");
}

// ---------------------------------------------------------------- the move
#ifndef CPPWRF_HAVE_NETCDF
void move_nest(Domain&, Domain&, int, int) { throw std::runtime_error("moving nests need NetCDF support"); }
#else
namespace {

using nest::move::HView;
using nest::move::Masks;

// one table line of data/wrf_schema/nest_interpdown-4.8.0.tsv
struct InterpRow {
  std::string fn, field, mask;
  bool xstag = false, ystag = false;
  std::vector<std::string> extra;
};

const std::vector<InterpRow>& interp_table() {
  static const std::vector<InterpRow> t = [] {
    const std::string path = data_dir() + "/wrf_schema/nest_interpdown-4.8.0.tsv";
    std::ifstream in(path);
    if (!in) throw std::runtime_error("moving nests: cannot read " + path);
    std::vector<InterpRow> r;
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::vector<std::string> f;
      std::stringstream ss(line);
      std::string c;
      while (std::getline(ss, c, '\t')) f.push_back(c);
      if (f.size() != 6) throw std::runtime_error(path + ": a line without 6 fields: " + line);
      InterpRow row;
      row.fn = f[0];
      row.field = f[1];
      row.xstag = f[2] == "T";
      row.ystag = f[3] == "T";
      row.mask = f[4];
      if (f[5] != "-") {
        std::stringstream es(f[5]);
        while (std::getline(es, c, ',')) row.extra.push_back(c);
      }
      r.push_back(std::move(row));
    }
    return r;
  }();
  return t;
}

int mask_index(const std::string& m) {
  if (m == "imask_nostag") return Masks::NOSTAG;
  if (m == "imask_xstag") return Masks::XSTAG;
  if (m == "imask_ystag") return Masks::YSTAG;
  if (m == "imask_xystag") return Masks::XYSTAG;
  throw std::runtime_error("moving nests: unknown interpolation mask " + m);
}

bool on_grid(const Layout& L, const Grid& g) {
  return L.si == 1 && L.dom < 0 && (L.nx == g.nx || L.nx == g.nx + 1) && (L.ny == g.ny || L.ny == g.ny + 1) &&
         L.wi0 == -Grid::halo && L.wj0 == -g.hy() && L.total > 0;
}

// a host copy of one array of a domain: a model field (registry), a history move array (in place) or the model's
// host latitudes / longitudes (Model::xlat, xlong)
struct Arr {
  Layout L;
  int esz = 0;
  std::vector<unsigned char> h;
  void* dev = nullptr;                          // the registry field to upload
  WrfHistory::MoveArray* hist = nullptr;        // in place
  std::vector<double>* vec = nullptr;           // Model::xlat / xlong (the domain's points, no halo)
  bool registry = false;                        // a model field
  bool integer = false;                         // an IntField2 (int elements)
  unsigned char* data() { return hist ? reinterpret_cast<unsigned char*>(hist->v.data()) : h.data(); }
  template <class T>
  HView<T> view() {
    if (sizeof(T) != std::size_t(esz)) throw std::logic_error("moving nests: an array viewed with another type");
    return HView<T>{reinterpret_cast<T*>(data()), &L};
  }
  void commit() {
    if (dev) copy_to_dev(dev, h.data(), h.size());
    if (vec) {
      const real* p = reinterpret_cast<const real*>(h.data());
      for (int j = 0; j < L.ny; ++j)
        for (int i = 0; i < L.nx; ++i) (*vec)[std::size_t(j) * std::size_t(L.nx) + std::size_t(i)] = double(p[L.index(i, j, 0)]);
    }
  }
};

Arr from_tracked(const Tracked& t) {
  Arr a;
  a.L = t.L;
  a.esz = t.esz;
  a.h.resize(t.L.total * std::size_t(t.esz));
  copy_to_host(a.h.data(), t.base, a.h.size());
  a.dev = t.base;
  a.registry = true;
  a.integer = phys::driver::is_int_field2(t.base);
  return a;
}

Arr from_vec(std::vector<double>& v, const Grid& g) {
  Arr a;
  a.L = Layout::make(g, Stagger::M, 2);
  a.esz = int(sizeof(real));
  a.h.assign(a.L.total * sizeof(real), 0);
  real* p = reinterpret_cast<real*>(a.h.data());
  for (int j = 0; j < g.ny; ++j)
    for (int i = 0; i < g.nx; ++i) p[a.L.index(i, j, 0)] = real(v[std::size_t(j) * std::size_t(g.nx) + std::size_t(i)]);
  a.vec = &v;
  return a;
}

// the wrfinput attribute of a domain (ISWATER, ISICE, ISLAKE), read at the first move (WRF reads them at the input)
int input_att(const Domain& d, const char* att, int def) {
  const std::string path = d.cfg->get_string("real.dir") + "/" + d.cfg->get_string("real.wrfinput", "wrfinput_d01");
  static std::mutex mu;
  static std::map<std::string, int> cache;
  const std::lock_guard<std::mutex> lk(mu);
  const std::string key = path + '\n' + att + '\n' + std::to_string(def);
  auto it = cache.find(key);
  if (it != cache.end()) return it->second;
  int nc = 0, v = def;
  if (nc_open(path.c_str(), NC_NOWRITE, &nc) != NC_NOERR) throw std::runtime_error("moving nests: cannot open " + path);
  if (nc_get_att_int(nc, NC_GLOBAL, att, &v) != NC_NOERR) v = def;
  nc_close(nc);
  cache.emplace(key, v);
  return v;
}

// a move array as an Arr (a copy)
Arr from_move_array(const WrfHistory::MoveArray& a) {
  Arr out;
  out.L = a.L;
  out.esz = int(sizeof(real));
  out.h.resize(a.v.size() * sizeof(real));
  std::memcpy(out.h.data(), a.v.data(), out.h.size());
  return out;
}

// the arrays of a domain named `name` (every registry field of that name on the grid, else the history's move
// array; the model's host latitudes / longitudes as well for xlat / xlong): the nest's destinations
std::vector<Arr> nest_arrays(Domain& d, const std::string& name) {
  std::vector<Arr> r;
  // grid%th_phy_m_t0 of moist_physics_finish_em (WRF's T): the history's array of the last solve (move_t_begin); the
  // suite's phy_prep field of that name is recomputed at every step's start
  if (name == "th_phy_m_t0" && d.hist)
    if (WrfHistory::MoveArray* a = d.hist->move_array(name)) {
      Arr x;
      x.L = a->L;
      x.esz = int(sizeof(real));
      x.hist = a;
      r.push_back(std::move(x));
      return r;
    }
  for (const Tracked& t : d.reg->fields)
    if (t.name == name && on_grid(t.L, d.m->g)) r.push_back(from_tracked(t));
  if (r.empty() && d.hist)
    if (WrfHistory::MoveArray* a = d.hist->move_array(name)) {
      Arr x;
      x.L = a->L;
      x.esz = int(sizeof(real));
      x.hist = a;
      r.push_back(std::move(x));
    }
  if ((name == "xlat" && !d.m->xlat.empty()) || (name == "xlong" && !d.m->xlong.empty()))
    r.push_back(from_vec(name == "xlat" ? d.m->xlat : d.m->xlong, d.m->g));
  return r;
}

// the parent's array named `name` (the occurrence-th registry field of that name, else its first, else the history's
// current values, else the model's host latitudes / longitudes); false: the parent has none
bool parent_array(Domain& p, const std::string& name, std::size_t occurrence, Arr& out) {
  if (name == "th_phy_m_t0" && p.hist) {  // the parent's grid%th_phy_m_t0 of its last solve (before the feedback)
    WrfHistory::MoveArray a;
    if (!p.hist->move_t_source(a)) throw std::runtime_error("moving nests: the parent has no th_phy_m_t0 of its last step");
    out = from_move_array(a);
    return true;
  }
  std::vector<const Tracked*> ts;
  for (const Tracked& t : p.reg->fields)
    if (t.name == name && on_grid(t.L, p.m->g)) ts.push_back(&t);
  if (!ts.empty()) {
    out = from_tracked(*ts[occurrence < ts.size() ? occurrence : 0]);
    out.dev = nullptr;  // read only
    out.registry = true;
    return true;
  }
  if (p.hist) {
    auto a = std::make_unique<WrfHistory::MoveArray>();
    if (p.hist->move_source(name, *a)) {
      out = from_move_array(*a);
      return true;
    }
  }
  if ((name == "xlat" && !p.m->xlat.empty()) || (name == "xlong" && !p.m->xlong.empty())) {
    out = from_vec(name == "xlat" ? p.m->xlat : p.m->xlong, p.m->g);
    out.vec = nullptr;
    return true;
  }
  return false;
}

}  // namespace

void move_nest(Domain& P, Domain& N, int dx, int dy) {
  using namespace nest::move;
  const Grid& g = N.m->g;
  if (g.patched() || P.m->g.patched()) throw std::runtime_error("moving nests in a decomposed run are not supported");
  if (dx < -1 || dx > 1 || dy < -1 || dy > 1) {  // med_nest_move (EM_CORE): wrf_error_fatal
    std::ostringstream os;
    os << " invalid move: dx, dy " << dx << " " << dy;
    throw std::runtime_error(os.str());
  }
  // shift_domain_em moves the arrays by parent_grid_ratio points, which come from the halo (WRF's halo is at least
  // that wide; CPPWRF's Grid::halo)
  if (N.ratio > Grid::halo)
    throw std::runtime_error("moving nests: parent_grid_ratio " + std::to_string(N.ratio) + " > " +
                             std::to_string(Grid::halo) + " is not supported");
  std::fprintf(stderr, " moving %d %d %d\n", N.id, dx, dy);
  dev_sync();
  // the parent's queued history feedback reads this nest's fields as they are now
  if (P.hist) P.hist->move_flush();
  if (N.hist) {
    N.hist->move_flush();
    N.hist->move_begin();
    N.hist->move_t_begin();
  }
  const int r = N.ratio;
  // shift_domain_em: px = isign(parent_grid_ratio, disp_x); one shift (|disp| <= 1)
  const int px = dx > 0 ? r : dx < 0 ? -r : 0, py = dy > 0 ? r : dy < 0 ? -r : 0;

  // 1. the masks (shift_domain_em.F:64-72), shifted with the fields
  Masks M;
  M.reset(g.nx, g.ny);
  if (px != 0) M.shift_x(px);
  if (py != 0) M.shift_y(py);

  // 2. every state array of the nest: x, then y (the model's fields, the history's arrays). Model::xlat / xlong
  // (host copies without a halo) are interpolated in step 4 over the whole strip the shift brings in.
  int nshift = 0;
  for (const Tracked& t : N.reg->fields) {
    if (!on_grid(t.L, g)) continue;
    Arr a = from_tracked(t);
    if (px != 0) shift_x(a.h.data(), a.L, a.esz, px);
    if (py != 0) shift_y(a.h.data(), a.L, a.esz, py);
    a.commit();
    ++nshift;
  }
  if (N.hist) N.hist->move_shift(px, py);
  for (std::vector<double>* v : {&N.m->xlat, &N.m->xlong}) {
    if (v->empty()) continue;
    Arr a = from_vec(*v, g);
    if (px != 0) shift_x(a.h.data(), a.L, a.esz, px);
    if (py != 0) shift_y(a.h.data(), a.L, a.esz, py);
    a.commit();
  }

  // 3. grid%i_parent_start / j_parent_start (med_nest_move: + px / parent_grid_ratio)
  N.i_parent_start += px / r;
  N.j_parent_start += py / r;
  check_window(P.m->g, g, r, N.i_parent_start, N.j_parent_start, "d" + std::string(N.id < 10 ? "0" : "") +
               std::to_string(N.id) + " after its move");

  // 4. med_interp_domain: every "d" field in WRF's order, where imask = 1
  Geo G;
  G.nide = g.nx + 1;
  G.njde = g.ny + 1;
  G.ipos = N.i_parent_start;
  G.jpos = N.j_parent_start;
  G.nri = G.nrj = r;
  const int iswater_n = input_att(N, "ISWATER", 16), isice_n = input_att(N, "ISICE", 24),
            islake_n = input_att(N, "ISLAKE", -1), iswater_1 = input_att(P, "ISWATER", 16);  // P: domain 1
  // the intermediate domain (external/RSL_LITE/module_dm.F:480-483, 625-660; one task): c_ids = i_parent_start - shw,
  // c_ide = c_ids + e_we / ratio + 1 + 2 shw, patch c_ids - shw .. c_ide + shw (shw = 2)
  constexpr int shw = 2;
  const int c_ids = G.ipos - shw, c_ide = c_ids + G.nide / r + 1 + 2 * shw, c_jds = G.jpos - shw,
            c_jde = c_jds + G.njde / r + 1 + 2 * shw;
  const int cits = c_ids - shw, cite = c_ide + shw, cjts = c_jds - shw, cjte = c_jde + shw;
  int ninterp = 0;
  std::vector<std::string> missing;
  for (const InterpRow& row : interp_table()) {
    std::vector<std::string> names;
    if (N.hist && !N.hist->schema_members(row.field).empty()) {
      names = N.hist->schema_members(row.field);  // a 4-D array: its members in use
    } else {
      if (N.hist && !N.hist->schema_in_use(row.field)) continue;  // not allocated in this configuration
      names = {row.field};
    }
    const int mw = mask_index(row.mask);
    auto has_extra = [&](const char* e) { return std::find(row.extra.begin(), row.extra.end(), e) != row.extra.end(); };
    // an extra array argument: c.NAME the parent's, n.NAME the nest's current values (an earlier line of the list
    // interpolated them); an integer array as REAL values (a read-only copy)
    auto extra_arr = [&](const std::string& e, Arr& out) {
      const std::string nm = e.substr(2);
      bool ok = false;
      if (e.compare(0, 2, "c.") == 0) {
        ok = parent_array(P, nm, 0, out);
      } else {
        std::vector<Arr> a = nest_arrays(N, nm);
        if (!a.empty()) {
          out = std::move(a[0]);
          ok = true;
        }
      }
      if (!ok) throw std::runtime_error("moving nests: the interpolation of " + row.field + " needs " + e);
      out.dev = nullptr;
      out.vec = nullptr;
      if (out.hist) {  // a copy: the extra is read only
        out.h.resize(out.hist->v.size() * sizeof(real));
        std::memcpy(out.h.data(), out.hist->v.data(), out.h.size());
        out.hist = nullptr;
      }
      if (out.integer) {
        std::vector<unsigned char> r(out.L.total * sizeof(real));
        const int* src = reinterpret_cast<const int*>(out.h.data());
        real* dst = reinterpret_cast<real*>(r.data());
        for (std::size_t q = 0; q < out.L.total; ++q) dst[q] = real(src[q]);
        out.h = std::move(r);
        out.esz = int(sizeof(real));
        out.integer = false;
      }
    };
    for (const std::string& name : names) {
      std::vector<Arr> dst = nest_arrays(N, name);
      if (dst.empty()) {
        missing.push_back(name);
        continue;
      }
      // an INTEGER Registry field the model holds as an IntField2 on both domains; else REAL values (the
      // history's arrays hold the integers as reals)
      const bool integer = !N.hist || N.hist->schema_type(name) == 'i';
      auto int_path = [&](const Arr& c, const Arr& d) {
        if (c.integer != d.integer)
          throw std::runtime_error("moving nests: " + name + " is an integer field on one domain only");
        if (d.integer && !integer)
          throw std::runtime_error("moving nests: " + name + " is an IntField2 but a REAL Registry field");
        return d.integer;
      };
      for (std::size_t o = 0; o < dst.size(); ++o) {
        Arr& d = dst[o];
        Arr c;
        if (!parent_array(P, name, o, c))
          throw std::runtime_error("moving nests: the nest has " + name + ", its parent does not");
        const int nk = std::max(1, d.L.nz);
        const std::string& fn = row.fn;
        if (d.integer && fn != "interp_fcni" && fn != "interp_fcnm" && fn != "interp_mask_soil")
          throw std::runtime_error("moving nests: " + fn + " of the integer field " + name);
        if (fn == "interp_fcn") {
          interp_fcn_sint(c.view<real>(), d.view<real>(), nk, M, mw, row.xstag, row.ystag, G);
        } else if (fn == "interp_fcnm" || fn == "interp_fcni" || fn == "interp_fcnm_imask" || fn == "interp_fcnm_lu") {
          const bool masked = fn != "interp_fcnm";
          if (int_path(c, d)) interp_nearest(c.view<int>(), d.view<int>(), nk, M, mw, G, masked);
          else interp_nearest(c.view<real>(), d.view<real>(), nk, M, mw, G, masked);
        } else if (fn == "interp_mask_field") {
          Arr clu, nlu;
          extra_arr("c.lu_index", clu);
          extra_arr("n.lu_index", nlu);
          const int flag = has_extra("n.isice") ? isice_n : iswater_n;
          interp_mask_field(c.view<real>(), d.view<real>(), nk, M, mw, clu.view<real>(), nlu.view<real>(), flag, G);
        } else if (fn == "interp_mask_soil") {
          Arr clu, nlu;
          extra_arr("c.lu_index", clu);
          extra_arr("n.lu_index", nlu);
          if (int_path(c, d))
            interp_mask_soil(c.view<int>(), d.view<int>(), nk, M, mw, clu.view<real>(), nlu.view<real>(), iswater_1, G);
          else
            interp_mask_soil(c.view<real>(), d.view<real>(), nk, M, mw, clu.view<real>(), nlu.view<real>(), iswater_1, G);
        } else if (fn == "interp_mask_water_field") {
          Arr clu, nlu;
          extra_arr("c.lu_index", clu);
          extra_arr("n.lu_index", nlu);
          const int flag = has_extra("n.islake") ? islake_n : iswater_n;
          interp_mask_water_field(c.view<real>(), d.view<real>(), nk, clu.view<real>(), nlu.view<real>(), flag, G, cits,
                                  cite, c_ide, cjts, cjte, c_jde);
        } else if (fn == "interp_fcn_blint_ll") {
          // the nest's latitudes of the same stagger (n.xlat, n.xlat_u, n.xlat_v), interpolated before
          std::string lat = "n.xlat";
          for (const std::string& e : row.extra)
            if (e.compare(0, 6, "n.xlat") == 0) lat = e;
          Arr nlat;
          extra_arr(lat, nlat);
          interp_fcn_blint_ll(c.view<real>(), d.view<real>(), M, mw, row.xstag, row.ystag, nlat.view<real>(), true, G);
        } else {
          throw std::runtime_error("moving nests: the nest interpolation " + fn + " of " + name + " is not ported");
        }
        d.commit();
        ++ninterp;
      }
    }
  }

  // 4b. start_domain(nest, .FALSE.) with nest%moved = .TRUE. (WRF dmpar; mediation_nest_move.F:264-271): the
  // dynamics (start_domain_moved), the terrain slopes, T and P_HYD of the frame written after the move, phy_init on
  // the new points (phy_init_moved)
  {
    std::vector<unsigned char> strip(std::size_t(g.nx) * std::size_t(g.ny), 0);
    for (int j = 0; j < g.ny; ++j)
      for (int i = 0; i < g.nx; ++i) strip[std::size_t(j) * std::size_t(g.nx) + std::size_t(i)] = M.at(Masks::NOSTAG, i, j) == 1;
    StartMoved sm = start_domain_moved(*N.cfg, *N.m, strip);
    for (const char* nm : {"p_hyd", "p_hyd_w"}) {
      const std::vector<double>& v = nm[5] == '_' ? sm.p_hyd_w : sm.p_hyd;
      for (const Tracked& t : N.reg->fields) {
        if (t.name != nm || !on_grid(t.L, g) || t.L.nx != g.nx || t.L.ny != g.ny) continue;
        Arr a = from_tracked(t);
        real* q = reinterpret_cast<real*>(a.h.data());
        const int nk = std::max(1, a.L.nz);
        if (std::size_t(nk) * std::size_t(g.nx) * std::size_t(g.ny) != v.size())
          throw std::logic_error(std::string("moving nests: the levels of ") + nm);
        for (int k = 0; k < nk; ++k)
          for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i)
              q[a.L.index(i, j, k)] = real(v[(std::size_t(k) * std::size_t(g.ny) + std::size_t(j)) * std::size_t(g.nx) + std::size_t(i)]);
        a.commit();
      }
    }
    if (N.hist) {
      N.hist->move_set_p_hyd(std::move(sm.p_hyd));
      N.hist->move_start_em();
      N.hist->move_t_end();
    }
    {
      // phy_init runs for any physics; check_phy_init_moved (at the run's start) admits the suite's options only
      std::vector<Arr> lu = nest_arrays(N, "lu_index");
      if (lu.empty()) throw std::runtime_error("moving nests: phy_init of the moved nest needs its LU_INDEX");
      std::vector<double> luv(strip.size(), 0.0);
      const real* q = reinterpret_cast<const real*>(lu[0].data());
      for (int j = 0; j < g.ny; ++j)
        for (int i = 0; i < g.nx; ++i) luv[std::size_t(j) * std::size_t(g.nx) + std::size_t(i)] = double(q[lu[0].L.index(i, j, 0)]);
      const std::vector<double> thc = phy_init_moved(*N.cfg, *N.m, strip, luv);
      // landuse_init's THC (module_physics_init.F:2093, 2119): a Registry "r" field (restart, or a history field
      // through iofields) the model does not carry
      if (N.hist)
        if (WrfHistory::MoveArray* a = N.hist->move_array("thc"))
          if (a->L.nx == g.nx && a->L.ny == g.ny && thc.size() == strip.size())
            for (int j = 0; j < g.ny; ++j)
              for (int i = 0; i < g.nx; ++i) {
                const std::size_t p = std::size_t(j) * std::size_t(g.nx) + std::size_t(i);
                if (strip[p]) a->v[a->L.index(i, j, 0)] = real(thc[p]);
              }
    }
    // start_em.F:1620-1621 (not a restart; topo_wind = 0, the only value a move admits): grid%ctopo = 1.,
    // grid%ctopo2 = 1. over the whole array (the strip's SINT values from the parent's fed-back 1 + 1 ulp go)
    if (N.m->suite) {
      for (Field2* f : {&N.m->suite->ctopo, &N.m->suite->ctopo2}) {
        if (f->layout().total == 0) continue;
        std::vector<real> h(f->layout().total, real(1));
        copy_to_dev(const_cast<void*>(f->base()), h.data(), f->bytes());
      }
    }
    if (N.hist)
      for (const char* nm : {"ctopo", "ctopo2"})
        if (WrfHistory::MoveArray* a = N.hist->move_array(nm)) std::fill(a->v.begin(), a->v.end(), real(1));
  }

  // 5. time level 2 to time level 1 in the new points (med_nest_move: where ( imask .EQ. 1 ) over the memory)
  {
    dyn::State& s = *N.m->st;
    auto copy_level = [&](Field3* f1, const Field3* f2, int mw) {
      if (!f1 || !f2 || f1->layout().total == 0) return;
      std::vector<real> a, b;
      f1->download(a);
      f2->download(b);
      const Layout& L = f1->layout();
      for (int k = 0; k < std::max(1, L.nz); ++k)
        for (int j = L.wj0; j < L.wj1; ++j)
          for (int i = L.wi0; i < L.wi1; ++i)
            if (M.at(mw, i, j) == 1) a[L.index(i, j, k)] = b[L.index(i, j, k)];
      copy_to_dev(const_cast<void*>(f1->base()), a.data(), a.size() * sizeof(real));
    };
    copy_level(&s.u_1, &s.u_2, Masks::XSTAG);
    copy_level(&s.v_1, &s.v_2, Masks::YSTAG);
    copy_level(&s.t_1, &s.t_2, Masks::NOSTAG);
    copy_level(&s.w_1, &s.w_2, Masks::NOSTAG);
    copy_level(&s.ph_1, &s.ph_2, Masks::NOSTAG);
    if (s.P_TKE >= 0) copy_level(&s.q_1[std::size_t(s.P_TKE)], &s.q_2[std::size_t(s.P_TKE)], Masks::NOSTAG);
    {
      std::vector<real> a, b;
      s.mu_1.download(a);
      s.mu_2.download(b);
      const Layout& L = s.mu_1.layout();
      for (int j = L.wj0; j < L.wj1; ++j)
        for (int i = L.wi0; i < L.wi1; ++i)
          if (M.at(Masks::NOSTAG, i, j) == 1) a[L.index(i, j, 0)] = b[L.index(i, j, 0)];
      copy_to_dev(const_cast<void*>(s.mu_1.base()), a.data(), a.size() * sizeof(real));
    }
  }

  // the suite's copy of grid%t_1 for first_rk_step_part2 (conv_t_tendf_to_moist) is grid%t_1 as the move left it
  if (N.m->suite && N.m->suite->t1_lag_set_) N.m->suite->t1_lag_.copy_from(N.m->st->t_1);

  // 6. the radiation driver's copies of the latitudes and longitudes
  if (N.m->suite) N.m->suite->move_latlon(N.m->xlat, N.m->xlong);
  // the output's I_PARENT_START / J_PARENT_START (WRF dmpar: nl_set_i/j_parent_start)
  if (N.hist) N.hist->set_parent_start(N.i_parent_start, N.j_parent_start);
  dev_sync();

  std::fprintf(stderr, " Grid %d New SW corner (in parent x and y): %d %d\n", N.id, N.i_parent_start, N.j_parent_start);
  if (debug()) {
    std::fprintf(stderr, "cppwrf move: %d arrays shifted, %d interpolated\n", nshift, ninterp);
    std::sort(missing.begin(), missing.end());
    missing.erase(std::unique(missing.begin(), missing.end()), missing.end());
    for (const std::string& m : missing) std::fprintf(stderr, "cppwrf move: \"d\" field without an array: %s\n", m.c_str());
    std::map<std::string, int> names;
    for (const Tracked& t : N.reg->fields)
      if (on_grid(t.L, g)) ++names[t.name];
    for (const auto& [n, c] : names)
      std::fprintf(stderr, "cppwrf move: field %s%s%s\n", n.c_str(), c > 1 ? " x" : "", c > 1 ? std::to_string(c).c_str() : "");
  }
}
#endif

}  // namespace cppwrf::cases::nestmove
