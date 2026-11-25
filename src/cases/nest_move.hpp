// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Moving nests with prescribed moves: WRF 4.8.0's med_nest_move (share/mediation_nest_move.F) as a dmpar wrf.exe
// (one MPI rank) built with "preset moves" (-DMOVE_NESTS, -DDM_PARALLEL) runs it, after each nest's feedback in the
// parent's step (frame/module_integrate.F:446-452). A serial build (-DSTUBMPI) is not the reference: its
// med_nest_move skips nl_set_i/j_parent_start, which the interpolation, forcing and feedback read, and start_domain.
//   time_for_move / time_for_move2: the namelist's num_moves, move_id, move_interval (minutes from the run's start:
//     the clock's start time, which a restarted run resets), move_cd_x, move_cd_y; a move is used once; the corral
//     check against domain 1 (corral_dist);
//   shift_domain_em: every state array of the nest by one parent cell (nest/nest_move.hpp);
//   grid%i_parent_start / j_parent_start += the move, and nl_set_i/j_parent_start (the output's I/J_PARENT_START);
//   med_interp_domain: every Registry "d" field of the configuration (data/wrf_schema/nest_interpdown-4.8.0.tsv,
//     WRF's order) from the parent where the interpolation mask is 1;
//   start_domain(nest) with nest%moved: start_em's base state, t_1 / ph_1 / mu_1, al, p, alt, p_hyd, p_hyd_w, the
//     terrain slopes and phy_init on the new points (start_domain_moved, phy_init_moved);
//   u_1 = u_2, v_1 = v_2, t_1 = t_2, w_1 = w_2, ph_1 = ph_2, tke_1 = tke_2, mu_1 = mu_2 where the mask is 1.
// The nest's arrays are its model's fields (every field allocated while the domain's FieldRegistry scope is active)
// and the history fields the model does not carry (WrfHistory's move arrays). The move runs on the host (the fields
// are copied there and back), so CPU, CUDA and both precisions share it; decomposed runs are not supported.
#include <memory>
#include <string>
#include <vector>

#include "cases/model.hpp"
#include "core/field.hpp"
#include "io/config.hpp"

namespace cppwrf::cases {
class WrfHistory;

namespace nestmove {

// one field of a domain: its allocation, layout, element size and name
struct Tracked {
  void* base = nullptr;
  Layout L;
  int esz = 0;
  std::string name;
};

// The fields allocated while a Scope of this registry is active on the thread (an Install must be active)
class FieldRegistry {
 public:
  FieldRegistry();
  ~FieldRegistry();
  FieldRegistry(const FieldRegistry&) = delete;
  FieldRegistry& operator=(const FieldRegistry&) = delete;
  std::vector<Tracked> fields;  // allocation order
};

// The field tracker of this thread for the run (tile::track; a previous tracker keeps receiving the calls)
class Install {
 public:
  Install();
  ~Install();
  Install(const Install&) = delete;
  Install& operator=(const Install&) = delete;

 private:
  const void* saved_ = nullptr;
};

// The allocations of this thread go to reg while the scope is alive (nullptr: to none); scopes nest
class Scope {
 public:
  explicit Scope(FieldRegistry* reg);
  ~Scope();
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;

 private:
  FieldRegistry* saved_;
};

// The namelist's prescribed moves (WRF's model_config_rec: one list for the run; a used move's interval becomes
// 999999999, as nl_set_move_interval)
struct Moves {
  int num_moves = 0;
  std::vector<int> move_id, move_interval, move_cd_x, move_cd_y;
  std::vector<int> corral_dist;  // per domain (index id - 1), Registry default 8
  bool active = false;  // a move of a nest (2 <= move_id <= max_dom) with move_interval < 999999999
  bool any() const { return active; }
};
// [domains] num_moves, move_id, move_interval, move_cd_x, move_cd_y (comma lists), corral_dist (d0N / domains.d0N_);
// checks max_moves (50), the moving domains (no children, parent domain 1, ratio, the nest inside its parent)
Moves parse_moves(const io::Config& root, int max_dom);

// One domain as the move sees it
struct Domain {
  Model* m = nullptr;
  FieldRegistry* reg = nullptr;
  WrfHistory* hist = nullptr;  // null: no history fields
  const io::Config* cfg = nullptr;
  int id = 1;
  int i_parent_start = 1, j_parent_start = 1, ratio = 1;  // a nest's (the grid's copy)
};

// time_for_move (share/mediation_nest_move.F:657-758) for a nest without children whose parent is domain 1:
// time_for_move2's prescribed move of this nest due at the nest's clock (now: since the run's start), then the
// corral check against the parent (a move that would bring the nest within corral_dist parent cells of domain 1's
// edge is cancelled: "MOAD can not move"). dx, dy: the move in parent cells (-1, 0, 1).
bool time_for_move(Moves& mv, const Domain& parent, const Domain& nest, const wrf_time::Interval& now, int& dx, int& dy);

// med_nest_move's work after time_for_move: shift, i/j_parent_start, interpolation, time levels. nest's
// i_parent_start / j_parent_start are updated.
void move_nest(Domain& parent, Domain& nest, int dx, int dy);

// the nest's start_domain after a move (cases/nest_driver.cpp): the dynamics' part of start_em with grid%moved, strip =
// imask_nostag at the domain's points (j * nx + i); returns p_hyd and p_hyd_w at the domain's points ((k, j, i))
struct StartMoved {
  std::vector<double> p_hyd, p_hyd_w;
};
StartMoved start_domain_moved(const io::Config& cfg, Model& m, const std::vector<unsigned char>& strip);

// a nest's position that WRF's interpolation can serve: the intermediate domain's search box (interp_mask_water_field,
// SINT's stencil) inside domain 1's mass points (cases/nest_move.cpp); throws otherwise
void check_window(const Grid& parent, const Grid& nest, int ratio, int i_parent_start, int j_parent_start,
                  const std::string& what);

}  // namespace nestmove
}  // namespace cppwrf::cases
