// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Debug: CPPWRF_VM_DUMP=dir writes dycore checkpoints of the first time step for comparison with a
// WRF build that dumps the same arrays (fields in logical order: i fastest, then k, then j; each field
// preceded by its logical nx, ny, nz). Host-side, off unless the variable is set.
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <vector>

#include "core/field.hpp"

namespace cppwrf::vmdump {

inline bool& on() {
  static bool v = false;
  return v;
}

template <class F>
inline void write_field(std::FILE* f, const F& x) {
  std::vector<real> h;
  x.download(h);
  const Layout& L = x.layout();
  const int nz = L.nz > 0 ? L.nz : 1;
  const int hdr[3] = {L.nx, L.ny, nz};
  std::fwrite(hdr, sizeof(int), 3, f);
  for (int j = 0; j < L.ny; ++j)
    for (int k = 0; k < nz; ++k)
      for (int i = 0; i < L.nx; ++i) {
        const double v = double(h[L.index(i, j, k)]);
        std::fwrite(&v, sizeof(double), 1, f);
      }
}

inline std::FILE* open(const char* name, int rk) {
  const char* dir = std::getenv("CPPWRF_VM_DUMP");
  if (!dir || !on()) return nullptr;
  char fn[512];
  std::snprintf(fn, sizeof fn, "%s/%s%d.bin", dir, name, rk);
  return std::fopen(fn, "wb");
}

inline void dump(const char* name, int rk, std::initializer_list<const Field3*> f3,
                 std::initializer_list<const Field2*> f2 = {}, std::initializer_list<const BaseField3*> b3 = {}) {
  std::FILE* f = open(name, rk);
  if (!f) return;
  for (const Field3* x : f3) if (x) write_field(f, *x);  // null: field absent in this run (no qv)
  for (const Field2* x : f2) if (x) write_field(f, *x);
  for (const BaseField3* x : b3) if (x) write_field(f, *x);
  std::fclose(f);
}

// the whole allocation (halo included): header wi0, wi1, wj0, wj1, nz, then sj, sk, offset, total (int64), then
// every value of the buffer (point (i, j, k) at offset + i + j * sj + k * sk)
template <class F>
inline void write_raw(std::FILE* f, const F& x) {
  std::vector<real> h;
  x.download(h);
  const Layout& L = x.layout();
  const int hdr[5] = {L.wi0, L.wi1, L.wj0, L.wj1, L.nz > 0 ? L.nz : 1};
  const long long st[4] = {(long long)L.sj, (long long)L.sk, (long long)L.offset, (long long)h.size()};
  std::fwrite(hdr, sizeof(int), 5, f);
  std::fwrite(st, sizeof(long long), 4, f);
  for (real v : h) {
    const double d = double(v);
    std::fwrite(&d, sizeof(double), 1, f);
  }
}

inline void dump_raw(const char* name, int rk, std::initializer_list<const Field3*> f3,
                     std::initializer_list<const Field2*> f2 = {}) {
  std::FILE* f = open(name, rk);
  if (!f) return;
  for (const Field3* x : f3) if (x) write_raw(f, *x);
  for (const Field2* x : f2) if (x) write_raw(f, *x);
  std::fclose(f);
}

}  // namespace cppwrf::vmdump
