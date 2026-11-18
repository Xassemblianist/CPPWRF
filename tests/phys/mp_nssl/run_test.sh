#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# NSSL 2-moment microphysics (mp_physics = 18 with its defaults, = 17) column test: WRF v4.8.0's nssl_2mom_init and
# nssl_2mom_driver (module_mp_nssl_2mom.o of the -r8 -fdefault-double-8 build, WRF_R8D8_DIR) against the C++
# nssl_init / nssl_row (driver_nssl_cpu of the CMake build) on (i, k, j) tiles, several calls in a row, tolerance 0
# in every output.
#   run_test.sh BUILD_DIR      (BUILD_DIR: the directory of driver_nssl_cpu; the work files go there)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(cd "${1:?usage: run_test.sh BUILD_DIR}" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HEAVY="${REPO_ROOT}/tools/heavy.sh"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8d8
cd "${RUN_DIR}"   # nssl_2mom_init opens namelist.input and namelist.output here (the drivers write empty ones)

FORT_BIN="${RUN_DIR}/driver_nssl_fortran"
CPP_BIN="${RUN_DIR}/driver_nssl_cpu"
wrf_fortran "${FORT_BIN}" "${SCRIPT_DIR}/driver_nssl.f90"

INPUT_BIN="${RUN_DIR}/nssl_input.bin"
FORT_OUT="${RUN_DIR}/nssl_output_fortran.bin"
CPP_OUT="${RUN_DIR}/nssl_output_cpp.bin"
# rows of 1 to 65 points (odd / even gather batches of 64 points: the scalar tail of the vectorised pow), 35 to 60
# levels, dt 6 to 60 s (sedimentation sub-steps), the cold start (itimestep 1: QNN reset, numbers from mass) and
# later steps, history steps (diagflag: REFL_10CM to the top, ke_diag) and others, cu_used with the cumulus
# tendencies, nssl_ssat_output 0 / 1 / 2
for cfg in "7 2 40 12 1 3 1 0 2 7" "33 3 50 30 4 5 010 1 1 11" "65 2 60 60 1 3 1 0 0 12" "1 4 45 20 7 3 01 1 2 13" \
           "16 2 35 6 1 4 1001 1 2 14"; do
  set -- $cfg
  echo "=== NSSL Differential Test: $1 x $2 points, $3 levels, dt $4 s, first itimestep $5, $6 calls ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --nx "$1" --ny "$2" --nz "$3" --dt "$4" --start-it "$5" --steps "$6" \
    --diag "$7" --cu-used "$8" --ssat "$9" --seed "${10}" --out "${INPUT_BIN}"
  "${HEAVY}" "${FORT_BIN}" "${INPUT_BIN}" "${FORT_OUT}" > /dev/null
  "${HEAVY}" "${CPP_BIN}" "${INPUT_BIN}" "${CPP_OUT}"
  python3 "${SCRIPT_DIR}/compare.py" "${INPUT_BIN}" "${FORT_OUT}" "${CPP_OUT}"
done
echo "=== NSSL Differential Test PASSED ==="
