#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# RUC LSM (sf_surface_physics = 3) column test: WRF v4.8.0's RUCLSMINIT, LSMRUC and SFCDIAGS_RUCLSM
# (module_sf_ruclsm.o, module_sf_sfcdiags_ruclsm.o of the -r8 build, WRF_DIR; WRF's run/ tables) against the C++
# port (src/phys/lsm_ruc, driver_ruc_cpu built here), tolerance 0.
#   run_test.sh WORK_DIR
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
mkdir -p "${1:?usage: run_test.sh WORK_DIR}"
RUN_DIR="$(cd "$1" && pwd)"
cd "${RUN_DIR}"
for t in VEGPARM.TBL SOILPARM.TBL GENPARM.TBL; do cp -L "${W}/run/${t}" .; done
wrf_fortran "${RUN_DIR}/driver_ruc_fortran" "${SCRIPT_DIR}/driver_ruc.f90"
g++ -std=c++20 -O2 -ffp-contract=off -fopenmp -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" "${SCRIPT_DIR}/driver_ruc.cpp" \
  "${REPO_ROOT}/src/phys/lsm_ruc/ruc_tables.cpp" "${REPO_ROOT}/src/core/backend.cpp" -o "${RUN_DIR}/driver_ruc_cpu"
# points, soil levels, steps, myj, frpcpn, rdlai2d, MODIS land use, dt, seed
for cfg in "3000 9 5 0 1 0 0 60 1" "3000 6 5 0 1 0 0 60 2" "3000 9 5 1 1 1 0 30 3" "3000 9 5 0 0 0 1 60 4" \
           "3000 6 5 1 1 0 1 20 5" "3000 9 8 0 1 0 0 180 6"; do
  set -- $cfg
  echo "=== RUC LSM column test: $1 points, $2 levels, $3 steps, myj $4, frpcpn $5, rdlai2d $6, modis $7, dt $8 ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --n "$1" --nsl "$2" --steps "$3" --myj "$4" --frpcpn "$5" \
    --rdlai2d "$6" --modis "$7" --dt "$8" --seed "$9" --out ruc_in.bin
  ./driver_ruc_fortran ruc_in.bin ruc_out_fortran.bin > fortran.log
  ./driver_ruc_cpu ruc_in.bin ruc_out_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" ruc_out_fortran.bin ruc_out_cpp.bin "$1" "$2" "$3"
done
echo "=== RUC LSM column test PASSED ==="
