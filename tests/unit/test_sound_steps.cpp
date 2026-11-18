// SPDX-License-Identifier: GPL-3.0-or-later
// time_step_sound = 0: solve_em's number of acoustic steps, max(2 * (INT(300. * dt / spacing - 0.01) + 1), 4), is
// computed in REAL as in WRF: double in the FP64 build (WRF -r8), float in the FP32 build (WRF real*4). dx = 10 km,
// dt = 67 s gives 6 steps in single precision and 4 in double precision.
#include <cstdio>

#include "core/precision.hpp"
#include "dyn/rk3.hpp"

using namespace cppwrf;

int main() {
  int fails = 0;
  auto check = [&](double dt, double dx, double dy, int want) {
    const int got = dyn::RK3::auto_sound_steps(dt, dx, dy);
    std::printf("%s dt %g dx %g dy %g: %d steps (want %d)\n", got == want ? "ok  " : "FAIL", dt, dx, dy, got, want);
    if (got != want) ++fails;
  };
  const bool single = sizeof(real) == 4;
  check(67.0, 10000.0, 10000.0, single ? 6 : 4);
  check(6.7, 1000.0, 1000.0, single ? 6 : 4);
  check(180.0, 30000.0, 30000.0, 4);   // 300 * 180 / 30000 = 1.8
  check(60.0, 9000.0, 12000.0, 4);     // the smaller spacing counts: 2.0 - 0.01 -> 1
  check(60.0, 12000.0, 4000.0, 10);    // 4.5 - 0.01 -> 4
  check(6.0, 2000.0, 2000.0, 4);       // at least 4
  check(40.0, 2000.0, 2000.0, 12);     // 6 - 0.01 -> 5
  std::printf("%s\n", fails ? "test_sound_steps FAILED" : "test_sound_steps passed");
  return fails ? 1 : 0;
}
