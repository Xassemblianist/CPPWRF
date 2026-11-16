#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# EDKF mass flux of the QNSE PBL (mfshconv = 1) column test: WRF v4.8.0's MFSHCONVPBL (module_bl_mfshconvpbl.o of
# the -r8 build, WRF_DIR) on random tiles against the C++ port (driver_mf_cpu, built here with the C++ compiler),
# tolerance 0; the FP32 build runs as a smoke test.
#   run_test.sh WORK_DIR
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
RUN_DIR="$(cd "$1" && pwd)"
cd "${RUN_DIR}"
ulimit -s unlimited 2>/dev/null || ulimit -s 1048576 2>/dev/null || true  # WRF's automatic arrays of the tile
wrf_fortran "${RUN_DIR}/driver_mf_fortran" "${SCRIPT_DIR}/driver_mf.f90"
SRCS=("${SCRIPT_DIR}/driver_mf.cpp" "${REPO_ROOT}/src/phys/bl_qnse/mfshconv.cpp" "${REPO_ROOT}/src/core/backend.cpp"
      "${REPO_ROOT}/src/core/column_work.cpp")
MARCH=${CPPWRF_TEST_MARCH--march=native}
g++ -std=c++20 -O3 ${MARCH} -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" "${SRCS[@]}" -lmvec -lm \
  -o "${RUN_DIR}/driver_mf_cpu"
g++ -std=c++20 -O3 ${MARCH} -ffp-contract=off -I"${REPO_ROOT}/src" "${SRCS[@]}" -lmvec -lm \
  -o "${RUN_DIR}/driver_mf_cpu_fp32"
# tiles nx x ny (the libmvec lanes of the exner run over the tile: odd counts end with the scalar pow), levels, dt,
# stepbl
for cfg in "61 49 44 60 1" "40 50 45 30 2" "37 27 60 20 1" "3 1 50 90 1" "1 1 44 60 1"; do
  set -- $cfg
  echo "=== EDKF column test: tile $1 x $2, $3 levels, dt $4, stepbl $5 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --nx "$1" --ny "$2" --nz "$3" --dt "$4" --stepbl "$5" --seed "$1$3" \
    --out mf_in.bin
  ./driver_mf_fortran mf_in.bin mf_out_fortran.bin
  ./driver_mf_cpu mf_in.bin mf_out_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" mf_out_fortran.bin mf_out_cpp.bin --cols $(($1 * $2)) --nz "$3"
done
./driver_mf_cpu_fp32 mf_in.bin /dev/null
echo "=== EDKF column test PASSED ==="
