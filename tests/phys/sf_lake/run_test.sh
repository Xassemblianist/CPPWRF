#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Lake model (sf_lake_physics = 1) column test: WRF v4.8.0's lakeini and Lake (module_sf_lake.o of the -r8 build,
# WRF_DIR) against lakeini_points and lake_point (src/phys/sf_lake), tolerance 0: every state array after lakeini
# and after each of 240 steps, and Lake's outputs, byte for byte. Configurations A (use_lakedepth 0), B (LAKE_DEPTH
# input) and C (LAKEFLAG 0): warm, deep, shallow, freezing, snow-covered and ice-covered lake points.
#   run_test.sh WORK_DIR      (the drivers are built there)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
WORK_DIR="$(cd "$1" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
cd "${WORK_DIR}"

FORT_BIN="${WORK_DIR}/driver_lake_fortran"
CPP_BIN="${WORK_DIR}/driver_lake_cpu"
wrf_fortran "${FORT_BIN}" "${SCRIPT_DIR}/driver_lake.f90"
g++ -std=c++20 -O3 ${CPPWRF_TEST_MARCH--march=native} -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" \
  "${SCRIPT_DIR}/driver_lake.cpp" "${REPO_ROOT}/src/core/backend.cpp" -o "${CPP_BIN}" -fopenmp -lmvec

for cfg in "A 1" "A 2" "B 3" "C 4"; do  # configuration, seed
  set -- $cfg
  echo "=== Lake Differential Test: configuration $1, seed $2 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" lake_input.bin --config "$1" --seed "$2"
  "${FORT_BIN}" lake_input.bin lake_output_fortran.bin
  "${CPP_BIN}" lake_input.bin lake_output_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" lake_output_fortran.bin lake_output_cpp.bin
done
echo "=== Lake Differential Test PASSED ==="
