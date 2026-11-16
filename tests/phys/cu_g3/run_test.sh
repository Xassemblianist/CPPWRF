#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Patch differential test for the Grell-3D cumulus scheme (cu_physics = 5): G3DRV and CONV_GRELL_SPREAD3D of WRF
# v4.8.0's own compiled module_cu_g3.o (the -r8 build, WRF_DIR; tests/phys/wrf_fortran.sh) against cu_g3::g3drv and
# cu_g3::g3_spread_column (src/phys/cu_g3/g3.hpp, the model's kernels' functions) on synthetic nx x ny patches
# (generate_inputs.py: moist unstable soundings, smooth in i and j, land and sea, day and night) for cugd_avedx 1 / 3,
# ishallow 0 / 1, clos_choice 0 / 3 / 10, with and without QC / QI, and odd level counts.
# Pass criterion: bit-identical (tol = 0.0) in CPU FP64 with convection in the patch; the FP32 driver runs as a smoke
# test.
#   run_test.sh [WORK_DIR]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HEAVY="${REPO_ROOT}/tools/heavy.sh"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
OUT_DIR="${1:-${PWD}/test_cu_g3}"
mkdir -p "${OUT_DIR}"
cd "${OUT_DIR}"

echo "=== Building the Fortran and C++ drivers (linking ${W}/main/libwrflib.a) ==="
wrf_fortran "${OUT_DIR}/driver_g3_fortran" "${SCRIPT_DIR}/driver_g3.f90"

for prec in fp64 fp32; do
  def=; [ "$prec" = fp64 ] && def=-DCPPWRF_DOUBLE
  "${HEAVY}" --light g++ -std=c++20 ${def} -O3 ${CPPWRF_TEST_MARCH:-} -ffp-contract=off -I"${REPO_ROOT}/src" \
    "${SCRIPT_DIR}/driver_g3.cpp" "${REPO_ROOT}/src/core/backend.cpp" -lmvec -lm -o "driver_g3_cpu_${prec}"
done

# nx ny nz cugd_avedx ishallow clos_choice f_qc f_qi seed
for cfg in "40 30 44 1 0 0 1 1 1" "41 29 33 1 1 0 1 1 2" "38 31 60 3 0 0 1 1 3" "40 30 45 3 1 0 0 0 4" \
           "36 28 44 1 1 3 1 0 5" "39 30 41 1 0 10 1 1 6"; do
  set -- $cfg
  echo "=== G3 differential test ($1 x $2 x $3, cugd_avedx $4, ishallow $5, clos_choice $6, qc $7, qi $8) ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --nx "$1" --ny "$2" --nz "$3" --avedx "$4" --ishallow "$5" \
    --ichoice "$6" --qc "$7" --qi "$8" --seed "$9" --out g3_in.bin
  "${HEAVY}" --light ./driver_g3_fortran g3_in.bin g3_wrf.bin
  "${HEAVY}" --light ./driver_g3_cpu_fp64 g3_in.bin g3_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" g3_wrf.bin g3_cpp.bin --input g3_in.bin --tol 0.0
done

echo "=== FP32 smoke run ==="
"${HEAVY}" --light ./driver_g3_cpu_fp32 g3_in.bin g3_fp32.bin
python3 "${SCRIPT_DIR}/compare.py" g3_wrf.bin g3_fp32.bin --input g3_in.bin --tol 1e30 > /dev/null

echo "=== G3 PATCH DIFFERENTIAL TEST PASSED (BIT-IDENTICAL IN CPU FP64) ==="
