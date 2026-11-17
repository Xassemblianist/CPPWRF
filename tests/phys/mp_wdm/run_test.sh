#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# WDM6 (mp_physics = 16) column test: WRF v4.8.0's wdm6init / wdm6 (module_mp_wdm6.o of the -r8 build, WRF_DIR)
# against the C++ Wdm6Kernel + WdmEffRadKernel (driver_wdm_cpu of the CMake build), three calls in a row, tolerance 0
# in every output (NN / NC / NR, the effective radii, the precipitation) and in wdm6init's constants.
#   run_test.sh BUILD_DIR      (BUILD_DIR: the directory of driver_wdm_cpu; the work files go there)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(cd "${1:?usage: run_test.sh BUILD_DIR}" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HEAVY="${REPO_ROOT}/tools/heavy.sh"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
cd "${RUN_DIR}"

FORT_BIN="${RUN_DIR}/driver_wdm_fortran"
CPP_BIN="${RUN_DIR}/driver_wdm_cpu"
wrf_fortran "${FORT_BIN}" "${SCRIPT_DIR}/driver_wdm.f90"

INPUT_BIN="${RUN_DIR}/wdm_input.bin"
FORT_OUT="${RUN_DIR}/wdm_output_fortran.bin"
CPP_OUT="${RUN_DIR}/wdm_output_cpp.bin"
# ni x nj x nz, dt, seed, hail_opt, itimestep of the first call (1: NN = ccn0 first), ccn_conc: odd and even ni (the
# scalar tail of the vectorised diffac loop; ni = 1: no vector lanes), levels in the 48 / 64 / 128 tiers, dt 60 - 300 s
# (one to three microphysics loops)
for cfg in "41 25 44 60 7 0 1 1e8" "40 20 50 180 8 1 1 3e8" "1 30 60 120 9 0 2 1e8" "33 9 90 300 10 1 2 5e9" \
           "64 12 35 240 11 0 1 1e8"; do
  set -- $cfg
  echo "=== WDM6 Differential Test: $1 x $2 columns, $3 levels, dt $4 s, hail_opt $6, itimestep $7, ccn $8 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --ni "$1" --nj "$2" --nz "$3" --dt "$4" --seed "$5" --hail "$6" \
    --it0 "$7" --ccn "$8" --steps 3 --scheme 6 --out "${INPUT_BIN}"
  "${HEAVY}" "${FORT_BIN}" "${INPUT_BIN}" "${FORT_OUT}"
  "${HEAVY}" "${CPP_BIN}" "${INPUT_BIN}" "${CPP_OUT}"
  python3 "${SCRIPT_DIR}/compare.py" "${INPUT_BIN}" "${FORT_OUT}" "${CPP_OUT}"
done
echo "=== WDM6 Differential Test PASSED ==="
