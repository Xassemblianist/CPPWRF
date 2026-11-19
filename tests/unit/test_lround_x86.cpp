// SPDX-License-Identifier: GPL-3.0-or-later
// lround_x86 (core/backend.hpp): gfortran's NINT as on x86-64 (LONG_MIN for NaN, +-Inf, |x| >= 2^63), also on the
// device, whose own conversion saturates (+Inf: LONG_MAX, int -1 where the CPU's int is 0). A CUDA build checks the
// device against the host.
#include <climits>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "core/backend.hpp"

namespace {
constexpr int kN = 12;
const double kX[kN] = {INFINITY, -INFINITY, NAN, 1e19, -1e19, 9.3e18, 3e9, -3e9, 2.5, -2.5, 0.49999999999999994, 7.0};

struct K {
  const double* x;
  long* out;
  CPPWRF_HD void operator()(int i, int) const {
    out[2 * i] = lround_x86(x[i]);
    out[2 * i + 1] = lround_x86(float(x[i]));
  }
};
}  // namespace

int main() {
  int fails = 0;
  const long want[kN] = {LONG_MIN, LONG_MIN, LONG_MIN, LONG_MIN, LONG_MIN, LONG_MIN, 3000000000L, -3000000000L,
                         3, -3, 0, 7};
  for (int i = 0; i < kN; ++i) {
    const long h = lround_x86(kX[i]);
    if (h != want[i]) {
      std::printf("host lround_x86(%g) = %ld, want %ld\n", kX[i], h, want[i]);
      ++fails;
    }
  }
#if defined(CPPWRF_USE_CUDA)
  double* dx = static_cast<double*>(cppwrf::dev_alloc(sizeof kX));
  long* dout = static_cast<long*>(cppwrf::dev_alloc(sizeof(long) * 2 * kN));
  cppwrf::copy_to_dev(dx, kX, sizeof kX);
  cppwrf::column_for(cppwrf::Range2D{0, kN, 0, 1}, K{dx, dout});
  std::vector<long> out(2 * kN);
  cppwrf::copy_to_host(out.data(), dout, sizeof(long) * 2 * kN);
  for (int i = 0; i < kN; ++i) {
    const long hd = lround_x86(kX[i]), hf = lround_x86(float(kX[i]));
    if (out[2 * i] != hd || out[2 * i + 1] != hf || int(out[2 * i]) != int(hd)) {
      std::printf("device lround_x86(%g) = %ld / %ld (float), host %ld / %ld\n", kX[i], out[2 * i], out[2 * i + 1], hd,
                  hf);
      ++fails;
    }
  }
  cppwrf::dev_free(dx);
  cppwrf::dev_free(dout);
#endif
  if (fails) {
    std::printf("FAIL (%d)\n", fails);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
