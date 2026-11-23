// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/column_work.hpp"

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <string>

namespace cppwrf {

#if defined(CPPWRF_USE_GPU)
namespace {
// per host thread: the tiles of a tiled run (src/core/tiles.cpp) launch their column physics at the same time
std::mutex mtx;
thread_local char* buf = nullptr;
thread_local std::size_t buf_bytes = 0;
// grows the buffer to `bytes` (its contents are not kept: every kernel writes its arrays before it reads them)
void grow(std::size_t bytes) {
  if (bytes <= buf_bytes) return;
  dev_free(buf);
  buf = nullptr;
  buf_bytes = 0;
  try {
    buf = static_cast<char*>(dev_alloc(bytes));
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("the column physics' work arrays: ") + e.what());
  }
  buf_bytes = bytes;
}
}  // namespace

void column_work_reserve(const Range2D& r0, std::size_t bytes_per_col) {
  // the worker's own columns (a decomposed run): column_work_chunks launches those only
  Range2D r = r0;
  tile::clip(r);
  if (r.i1 <= r.i0 || r.j1 <= r.j0 || bytes_per_col == 0) return;
  const int ni = r.i1 - r.i0, nj = r.j1 - r.j0;
  const std::size_t bpc = detail::column_work_bpc(bytes_per_col);
  const std::size_t budget = device_work_budget();
  std::size_t want = detail::column_for_threads(ni, nj) * bpc;
  if (want > budget) {  // whole block rows (a block covers column_block().y rows), at least one
    const std::size_t pair = detail::column_for_threads(ni, column_block().y) * bpc;
    want = std::max<std::size_t>(budget / pair, 1) * pair;
  }
  std::lock_guard<std::mutex> lk(mtx);
  grow(want);
}

void column_work_release() noexcept {
  try {
    std::lock_guard<std::mutex> lk(mtx);
    dev_free(buf);
  } catch (...) {
  }
  buf = nullptr;
  buf_bytes = 0;
}

namespace detail {
char* column_work_buffer(std::size_t bytes, std::size_t* have) {
  std::lock_guard<std::mutex> lk(mtx);
  grow(bytes);
  *have = buf_bytes;
  return buf;
}
}  // namespace detail
#else
void column_work_reserve(const Range2D&, std::size_t) {}
void column_work_release() noexcept {}
namespace detail {
char* column_work_buffer(std::size_t, std::size_t* have) {
  *have = 0;
  return nullptr;
}
}  // namespace detail
#endif

}  // namespace cppwrf
