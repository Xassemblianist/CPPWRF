#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Single-layer urban canopy model (sf_urban_physics = 1) column test: WRF v4.8.0's urban_param_init and SUBROUTINE
# urban (module_sf_urban.o of the -r8 build, WRF_DIR; WRF's run/URBPARM.TBL and URBPARM_LCZ.TBL) against the C++
# read_urban_table and urban_point (src/phys/sf_urban), tolerance 0 (every output and state value, byte for byte).
#   run_test.sh WORK_DIR      (the drivers are built there)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
WORK_DIR="$(cd "$1" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
cd "${WORK_DIR}"
for t in URBPARM.TBL URBPARM_LCZ.TBL; do cp -L "${W}/run/${t}" .; done

FORT_BIN="${WORK_DIR}/driver_urban_fortran"
CPP_BIN="${WORK_DIR}/driver_urban_cpu"
wrf_fortran "${FORT_BIN}" "${SCRIPT_DIR}/driver_urban.f90"
g++ -std=c++20 -O3 -march=native -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" \
  "${SCRIPT_DIR}/driver_urban.cpp" "${REPO_ROOT}/src/phys/sf_urban/urban_tables.cpp" \
  "${REPO_ROOT}/src/core/backend.cpp" -o "${CPP_BIN}" -fopenmp

for cfg in "0 14" "0 15" "1 16"; do  # use_wudapt_lcz, seed
  set -- $cfg
  tbl=URBPARM.TBL; [ "$1" = 1 ] && tbl=URBPARM_LCZ.TBL
  echo "=== SLUCM Differential Test: ${tbl}, seed $2 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" "${tbl}" urban_input.bin --lcz "$1" --seed "$2"
  "${FORT_BIN}" urban_input.bin urban_output_fortran.bin
  "${CPP_BIN}" urban_input.bin urban_output_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" urban_output_fortran.bin urban_output_cpp.bin
done
echo "=== SLUCM Differential Test PASSED ==="
