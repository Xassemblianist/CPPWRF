#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# QNSE surface layer and PBL (sf_sfclay_physics = 4, bl_pbl_physics = 4, mfshconv = 0) column test: WRF v4.8.0's
# QNSESFCINIT, QNSESFC and QNSEPBL (module_sf_qnsesfc.o, module_bl_qnsepbl.o of the -r8 build, WRF_DIR) against the
# C++ port (driver_qnse_cpu, built here with the C++ compiler), tolerance 0; the FP32 build runs as a smoke test.
#   run_test.sh WORK_DIR
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
RUN_DIR="$(cd "$1" && pwd)"
cd "${RUN_DIR}"
wrf_fortran "${RUN_DIR}/driver_qnse_fortran" "${SCRIPT_DIR}/driver_qnse.f90"
SRCS=("${SCRIPT_DIR}/driver_qnse.cpp" "${REPO_ROOT}/src/phys/sf_qnsesfc/qnsesfc.cpp"
      "${REPO_ROOT}/src/phys/bl_qnse/qnsepbl.cpp" "${REPO_ROOT}/src/core/backend.cpp"
      "${REPO_ROOT}/src/core/column_work.cpp")
MARCH=${CPPWRF_TEST_MARCH--march=native}
g++ -std=c++20 -O3 ${MARCH} -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" "${SRCS[@]}" -lmvec -lm \
  -o "${RUN_DIR}/driver_qnse_cpu"
g++ -std=c++20 -O3 ${MARCH} -ffp-contract=off -I"${REPO_ROOT}/src" "${SRCS[@]}" -lmvec -lm \
  -o "${RUN_DIR}/driver_qnse_cpu_fp32"
# columns, levels, itimestep, dt: 44 levels (odd PRODQ2 loop: the scalar pow after the libmvec pairs), 45, the
# first step (USTAR = 0.1, the sea points' first-step branch) and 60 levels
for cfg in "3000 44 3 60" "3000 45 2 30" "2000 44 1 60" "1500 60 5 20"; do
  set -- $cfg
  echo "=== QNSE column test: $1 columns x $2 levels, itimestep $3, dt $4 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --cols "$1" --nz "$2" --itimestep "$3" --dt "$4" --seed "$2$3" \
    --out qnse_in.bin
  ./driver_qnse_fortran qnse_in.bin qnse_out_fortran.bin
  ./driver_qnse_cpu qnse_in.bin qnse_out_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" qnse_out_fortran.bin qnse_out_cpp.bin --cols "$1" --nz "$2"
done
./driver_qnse_cpu_fp32 qnse_in.bin /dev/null
echo "=== QNSE column test PASSED ==="
