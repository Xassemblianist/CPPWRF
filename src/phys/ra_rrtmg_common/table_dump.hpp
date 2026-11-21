// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// The RRTMG tables dumps (data/rrtmg/README.md): memory images of the packed table structs (RrtmgLwTablesOf<R>,
// RrtmgSwTablesOf<R>: reals R and int32 members) as an instrumented WRF build writes them, R = double for WRF -r8
// (rrtmg_*_tables.bin), R = float for WRF real*4 (rrtmg_*_tables_r4.bin).
//
// read_table_dump<Dump64>(path, t, ints64, ints) fills t, a struct of the build's rb:
//  - a file of exactly sizeof(t) bytes is t's own image (the dump of the WRF build of the same precision);
//  - otherwise the file's first sizeof(Dump64) bytes are the WRF -r8 dump. In the FP64 builds that is t's layout;
//    in the FP32 builds every real of it is rounded once to float and the int32 members are copied.
// ints64 / ints: the byte ranges of the int32 members in the -r8 layout and in t's, in ascending order (the same
// members); everything between them is reals.
#include <array>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "phys/ra_rrtmg_common/parkind.hpp"

namespace cppwrf::phys {

struct TableBytes {
  std::size_t begin, end;  // [begin, end) bytes from the start of the struct
};

// the bytes of a member at offset `at` of a struct that sits at offset `base` of the enclosing one
constexpr TableBytes table_member(std::size_t base, std::size_t at, std::size_t size) {
  return {base + at, base + at + size};
}

// the -r8 image src (n64 bytes) into the rb image dst (n bytes); false if the two layouts do not pair up
template <std::size_t N>
bool narrow_table_image(const unsigned char* src, std::size_t n64, unsigned char* dst, std::size_t n,
                        const std::array<TableBytes, N>& ints64, const std::array<TableBytes, N>& ints) {
  std::size_t s = 0, d = 0;
  for (std::size_t k = 0; k <= N; ++k) {
    const std::size_t s_end = k < N ? ints64[k].begin : n64;
    const std::size_t d_end = k < N ? ints[k].begin : n;
    for (; s + sizeof(double) <= s_end && d + sizeof(rb) <= d_end; s += sizeof(double), d += sizeof(rb)) {
      double v;
      std::memcpy(&v, src + s, sizeof(double));
      const rb r = rb(v);
      std::memcpy(dst + d, &r, sizeof(rb));
    }
    if (s != s_end || d != d_end) return false;
    if (k < N) {
      const std::size_t len = ints64[k].end - ints64[k].begin;
      if (ints[k].end - ints[k].begin != len) return false;
      std::memcpy(dst + d, src + s, len);
      s += len;
      d += len;
    }
  }
  return true;
}

template <class Dump64, class Tables, std::size_t N>
bool read_table_dump(const std::string& path, Tables& t, const std::array<TableBytes, N>& ints64,
                     const std::array<TableBytes, N>& ints) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) return false;
  file.seekg(0, std::ios::end);
  const std::streamoff size = file.tellg();
  file.seekg(0, std::ios::beg);
  if (size == static_cast<std::streamoff>(sizeof(Tables))) {
    file.read(reinterpret_cast<char*>(&t), sizeof(Tables));
    if (file.gcount() == static_cast<std::streamsize>(sizeof(Tables))) return true;
    file.clear();
    file.seekg(0, std::ios::beg);
  }
  if constexpr (sizeof(Dump64) == sizeof(Tables)) {
    file.read(reinterpret_cast<char*>(&t), sizeof(Tables));
    return file.gcount() == static_cast<std::streamsize>(sizeof(Tables));
  } else {
    std::vector<unsigned char> image(sizeof(Dump64));
    file.read(reinterpret_cast<char*>(image.data()), sizeof(Dump64));
    if (file.gcount() != static_cast<std::streamsize>(sizeof(Dump64))) return false;
    return narrow_table_image(image.data(), sizeof(Dump64), reinterpret_cast<unsigned char*>(&t), sizeof(Tables),
                              ints64, ints);
  }
}

}  // namespace cppwrf::phys
