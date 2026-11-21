// SPDX-License-Identifier: GPL-3.0-or-later
// tile::rank_device: the CUDA device of an MPI rank from CPPWRF_RANK_DEVICES, its rank on the node and the devices
// it sees (docs/MPI.md).
#include <cstdio>
#include <stdexcept>
#include <string>

#include "core/tiles.hpp"

using cppwrf::tile::rank_device;
static int fails = 0;

static void expect(const char* spec, int local, int count, int want) {
  int got = -2;
  try {
    got = rank_device(spec, local, count);
  } catch (const std::exception& e) {
    std::printf("FAIL rank_device(%s, %d, %d): threw '%s', want %d\n", spec ? spec : "null", local, count, e.what(),
                want);
    ++fails;
    return;
  }
  if (got != want) {
    std::printf("FAIL rank_device(%s, %d, %d) = %d, want %d\n", spec ? spec : "null", local, count, got, want);
    ++fails;
  }
}

static void expect_throw(const char* spec, int local, int count) {
  try {
    const int got = rank_device(spec, local, count);
    std::printf("FAIL rank_device(%s, %d, %d) = %d, want an error\n", spec, local, count, got);
    ++fails;
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()).find("CPPWRF_RANK_DEVICES") == std::string::npos) {
      std::printf("FAIL rank_device(%s, ...): the error does not name CPPWRF_RANK_DEVICES: %s\n", spec, e.what());
      ++fails;
    }
  }
}

int main() {
  // unset: one device (or none) keeps the current device; several: the node-local rank modulo the count
  expect(nullptr, 0, 1, -1);
  expect(nullptr, 3, 1, -1);
  expect("", 2, 0, -1);
  expect(nullptr, 0, 2, 0);
  expect(nullptr, 1, 2, 1);
  expect(nullptr, 2, 2, 0);
  expect(nullptr, 5, 4, 1);
  // set: entry local mod n
  expect("0,0", 0, 1, 0);
  expect("0,0", 3, 1, 0);
  expect("1", 0, 2, 1);
  expect("1,0", 0, 2, 1);
  expect("1,0", 1, 2, 0);
  expect("2,0,1", 4, 3, 0);
  expect("3", 7, 0, 3);  // count unknown (0): not checked against it
  // invalid entries
  expect_throw("a", 0, 2);
  expect_throw("0,,1", 0, 2);
  expect_throw("0,", 0, 2);
  expect_throw("-1", 0, 2);
  expect_throw("1x", 0, 2);
  expect_throw(" 1", 0, 2);
  expect_throw("99999999999", 0, 2);
  expect_throw("2", 0, 2);  // not below the visible count
  // only the entry the rank takes is checked against its count (a node with fewer devices than the list names)
  expect("0,1,2,3", 1, 2, 1);
  expect_throw("0,1,2,3", 2, 2);
  if (fails) {
    std::printf("test_rank_device: %d failures\n", fails);
    return 1;
  }
  std::printf("test_rank_device: OK\n");
  return 0;
}
