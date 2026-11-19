#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Terrain options column test: WRF v4.8.0's radconst, toposhad_init / toposhad (pre_radiation_driver with one patch)
# and TOPO_RAD_ADJ_DRVR of the -r8 build (WRF_DIR) against src/phys/driver/topo.hpp's KTopoShad and KTopoRadAdj,
# tolerance 0 (shadow mask, shadow height, SWDOWN, GSW, SWNORM, GSWSAVE byte for byte).
#   run_test.sh WORK_DIR
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
WORK_DIR="$(cd "$1" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
cd "${WORK_DIR}"
FORT_BIN="${WORK_DIR}/driver_topo_fortran"
CPP_BIN="${WORK_DIR}/driver_topo_cpu"
wrf_fortran "${FORT_BIN}" "${SCRIPT_DIR}/driver_topo.f90"
g++ -std=c++20 -O2 -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" "${SCRIPT_DIR}/driver_topo.cpp" \
  "${REPO_ROOT}/src/core/backend.cpp" -o "${CPP_BIN}" -fopenmp
for cfg in "1 15000 25000 39 0.2" "2 15000 100000 39 -0.3" "3 3000 25000 -35 2.0" "4 1000 8000 62 -2.5"; do
  set -- $cfg
  echo "=== terrain options: seed $1, dx $2, shadlen $3, lat $4, rotation $5 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" topo_input.bin --seed "$1" --dx "$2" --shadlen "$3" --lat "$4" --rot "$5"
  "${FORT_BIN}" topo_input.bin topo_output_fortran.bin
  "${CPP_BIN}" topo_input.bin topo_output_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" topo_output_fortran.bin topo_output_cpp.bin 48 36 4000
done
echo "=== terrain options column test PASSED ==="
