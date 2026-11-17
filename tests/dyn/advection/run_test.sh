#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Scalar advection of the last RK3 stage (moist_adv_opt 1 positive definite, 2 monotonic): WRF v4.8.0's own
# rk_update_scalar_pd, advect_scalar_pd / advect_scalar_mono, set_tend and rk_update_scalar (the -r8 build, WRF_DIR;
# tests/phys/wrf_fortran.sh) called by driver_advection.f90 against CPPWRF's ScalarTransport (driver_advection.cpp,
# the build's own kernels), on random fields of specified, periodic and open domains: q_2, the pre-updated q_1 and
# RQVFTEN of every species, tolerance 0 in a CPU FP64 build (an FP32 or CUDA build: relative 1e-5 / 1e-12).
#   run_test.sh DRIVER   (DRIVER: the build's driver_advection)
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DRV=$(readlink -f "$1")
. "${REPO_ROOT}/tests/phys/wrf_fortran.sh"
wrf_need r8
B="${DIFF_TEST_BUILD_DIR:-${SCRIPT_DIR}/build}/advection"
mkdir -p "$B"
cd "$B"
echo "== 1. Fortran driver (WRF objects of ${W})"
wrf_fortran "$B/driver_advection_f90" "${SCRIPT_DIR}/driver_advection.f90"
echo "== 2. inputs"
python3 -I "${SCRIPT_DIR}/generate_inputs.py" "$B/in" > cases.txt
echo "== 3. WRF and CPPWRF, comparison"
st=0
while read -r f; do
  c=$(basename "$f" .bin); c=${c#in_}
  "$B/driver_advection_f90" "$f" "$B/wrf_$c.bin"
  "$DRV" "$f" "$B/cpp_$c.bin"
  python3 -I "${SCRIPT_DIR}/compare.py" "$B/wrf_$c.bin" "$B/cpp_$c.bin" "$f" "$c" || st=1
done < cases.txt
exit $st
