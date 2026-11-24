// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The transport between the workers of a decomposed run (core/tiles.cpp): whole messages, in order per (sender,
// receiver) pair. Tiles: in-process mailboxes (tiles.cpp); MPI ranks: core/tiles_mpi.cpp.
#include <cstddef>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/tiles.hpp"

namespace cppwrf::tile::detail {

struct Aborted : std::runtime_error {
  Aborted() : std::runtime_error("another worker stopped") {}
};

// One message of a bulk exchange: `bytes` bytes at `p`, to or from worker `peer`
struct Post {
  int peer;
  char* p;
  std::size_t bytes;
};

struct Comm {
  virtual ~Comm() = default;
  // never blocks on the receiver (the message is queued); the next message of this worker to `to`
  virtual void send(int to, std::vector<char>&& m) = 0;
  // waits for the next message from `from`; Aborted when another worker stopped
  virtual std::vector<char> recv(int from) = 0;
  // every queued message has left this worker's buffers (after each collective)
  virtual void flush() {}
  // A halo exchange's messages (core/tiles.cpp transfer): sends every message of `out` and receives every message of
  // `in` into its buffer (the sizes are known to both sides); returns when all have arrived and every buffer of
  // `out` may be reused. `device`: the buffers are device memory (only when device_buffers()). The messages of the
  // exchanges are in order per (sender, receiver) pair, apart from those of send / recv. Default: send / recv.
  virtual void exchange(const std::vector<Post>& out, const std::vector<Post>& in, bool device) {
    (void)device;
    for (const Post& m : out) send(m.peer, std::vector<char>(m.p, m.p + m.bytes));
    for (const Post& m : in) {
      const std::vector<char> r = recv(m.peer);
      if (r.size() != m.bytes)
        throw std::logic_error("decomposition: a halo message of " + std::to_string(r.size()) + " bytes from worker " +
                               std::to_string(m.peer) + ", " + std::to_string(m.bytes) + " expected (the workers' calls differ)");
      if (m.bytes) std::memcpy(m.p, r.data(), m.bytes);
    }
  }
  // exchange() takes device buffers (a GPU-aware MPI: docs/MPI.md, run.gpu_aware_mpi)
  virtual bool device_buffers() const { return false; }
  // every worker's `bytes` bytes at `mine` into `all` (worker order, `bytes` each), as one collective of the
  // transport; false: the transport has none (core/tiles.cpp sends them to every worker itself)
  virtual bool allgather(const void* mine, std::size_t bytes, void* all) {
    (void)mine;
    (void)bytes;
    (void)all;
    return false;
  }
};

// The calling thread as worker `id` of n = o.px * o.py over the transport c: body(id) with the decomposition's hooks.
int run_worker(Comm& c, int id, const Options& o, const std::function<int(int)>& body);

}  // namespace cppwrf::tile::detail
