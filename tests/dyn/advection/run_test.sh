#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Scalar advection of the last RK3 stage (moist_adv_opt 1 positive definite, 2 monotonic, 3 WENO, 4 positive-definite
# WENO): WRF v4.8.0's own rk_update_scalar_pd, advect_scalar_pd / _mono / _weno / _wenopd, set_tend and
# rk_update_scalar (the -r8 build WRF_DIR for 1 / 2, the -r8 -fdefault-double-8 build WRF_R8D8_DIR for 3 / 4, whose
# DOUBLE PRECISION WENO locals are real*8; tests/phys/wrf_fortran.sh) called by driver_advection.f90 against CPPWRF's
# ScalarTransport (driver_advection.cpp,
# the build's own kernels), on random fields of specified, periodic and open domains: q_2, the pre-updated q_1 and
# RQVFTEN of every species, tolerance 0 in a CPU FP64 build (an FP32 or CUDA build: relative 1e-5 / 1e-12).
#   run_test.sh DRIVER   (DRIVER: the build's driver_advection)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DRV=$(readlink -f "$1")
. "${REPO_ROOT}/tests/phys/wrf_fortran.sh"
B="${DIFF_TEST_BUILD_DIR:-${SCRIPT_DIR}/build}/advection"
mkdir -p "$B/r8" "$B/r8d8"
cd "$B"
for fl in r8 r8d8; do
  wrf_need $fl
  echo "== 1. Fortran driver (WRF objects of ${W})"
  wrf_fortran "$B/$fl/driver_advection_f90" "${SCRIPT_DIR}/driver_advection.f90"
done
echo "== 2. inputs"
python3 -I "${SCRIPT_DIR}/generate_inputs.py" "$B/in" > cases.txt
echo "== 3. WRF and CPPWRF, comparison"
st=0
while read -r f; do
  c=$(basename "$f" .bin); c=${c#in_}
  case $c in *weno*) fl=r8d8 ;; *) fl=r8 ;; esac
  "$B/$fl/driver_advection_f90" "$f" "$B/wrf_$c.bin"
  "$DRV" "$f" "$B/cpp_$c.bin"
  python3 -I "${SCRIPT_DIR}/compare.py" "$B/wrf_$c.bin" "$B/cpp_$c.bin" "$f" "$c" || st=1
done < cases.txt
exit $st
