// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// One domain as several tiles (domain decomposition). Each tile is a host thread that runs the whole model with its
// own copy of every field, its own stream and its own device (a configuration: run.tiles / CPPWRF_TILE_DEVICES), and
// computes only the points it owns: inside a time step every launch over (i, j) is clipped to the tile's patch
// (core/backend.hpp, tile::ctx). The tiles' copies meet through halo exchanges (every core/halo.hpp fill, and
// tile::exchange where a kernel reads neighbours that no fill refreshes): a tile pulls from its neighbours' copies the
// points they own within Grid::halo of its patch (and the sources of the periodic wrap), host memcpy on the CPU,
// cudaMemcpy3D (peer copies between devices) on the GPU. The exchanges only copy values and every point is computed by
// one tile with the inputs of a run without tiles, so the result does not depend on the number of tiles.
//
// Phase 1: every tile allocates the whole domain (the copies of the points it does not own are stale between
// exchanges; allgather() completes them before the host reads fields for output); the start (Model::build, the
// input) and the output run in every tile, one tile at a time (Serial: NetCDF is not thread-safe), without clipping.
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "core/field.hpp"

namespace cppwrf::tile {

struct Options {
  int px = 1, py = 1;        // tiles along x and y
  std::vector<int> devices;  // CUDA device of tile t: devices[t % size] (empty: the current device)
  bool check = false;        // compare every launch of a step with a run without tiles (CPU build; slow)
  int count() const { return px * py; }
  bool on() const { return count() > 1 || check; }
};
// run.tiles ("P", "PxQ"; the environment CPPWRF_TILES overrides it), CPPWRF_TILE_DEVICES=d0,d1,... and
// CPPWRF_TILE_CHECK=1 (the reference run without tiles is compared with every tile after every launch of a time step:
// the first point that differs names the launch and the field)
Options options(const std::string& run_tiles);

// body(t) for t = 0 .. count - 1, each on its own host thread (and CUDA device), and with o.check also body(-1), the
// reference run. The first error stops every tile and is rethrown here; else body(0)'s value.
int run(const Options& o, const std::function<int(int)>& body);

int id();      // the calling thread's tile (0 without tiles, -1 for the check's reference run)
bool tiled();  // the calling thread is a tile of a run with tiles

// The calling tile's domain, before its first step (every tile the same): nx x ny mass points and the periodic
// directions. Tile t = tx + px ty owns the mass points [tx nx / px, (tx + 1) nx / px) x [ty ny / py, (ty + 1) ny / py)
// and the faces with the same indices; the edge tiles also own the last faces and the lateral halos beyond the edge.
void set_domain(int nx, int ny, bool periodic_x, bool periodic_y);

// A time step: launches clipped to the tile's points (tiles only), the check's comparisons on.
class Step {
 public:
  Step();
  ~Step();
  Step(const Step&) = delete;
  Step& operator=(const Step&) = delete;
};

// Every tile's copy of every field gets the points the other tiles own (collective; before the host reads fields
// for output or diagnostics). gather(): the same for some fields only.
void allgather();
void gather(const void* const* bases, int n);
template <class... F>
void gather(const F&... f) {
  if (!tiled()) return;
  const void* const b[] = {f.base()...};
  gather(b, int(sizeof...(F)));
}

// The host phases outside the time step (input, output) one tile at a time; no collective call inside.
class Serial {
 public:
  Serial();
  void unlock() {
    if (lk_.owns_lock()) lk_.unlock();
  }

 private:
  std::unique_lock<std::mutex> lk_;
};

}  // namespace cppwrf::tile
