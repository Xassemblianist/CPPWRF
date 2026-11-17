#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Column differential test of the multi-scale Kain-Fritsch cumulus (cu_physics = 11): WRF v4.8.0's MSKF_CPS
# (module_cu_mskf.o of the -r8 build, WRF_DIR; tests/phys/wrf_fortran.sh; MSKF_LUTAB through mskf_init) against
# cu_mskf::run_mskf (src/phys/cu_mskf) on generate_inputs.py's soundings: DX 15, 5, 3 km (the scale-aware
# branches), 25 and 30 km (none) and 24.999.5 km (no Scale_Fac but the unscaled trigger), kfeta_trigger 1 and 3, the
# three moisture sets, odd column / level counts. Every output (rain, NCA, cloud levels, W0AVG, the T / moisture /
# momentum tendencies, the cloud fractions and the cumulus condensate) bit-identical in CPU FP64 (tolerance 0).
#   run_test.sh [WORK_DIR]
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HEAVY="${REPO_ROOT}/tools/heavy.sh"
. "${SCRIPT_DIR}/../wrf_fortran.sh"
wrf_need r8
OUT_DIR="${1:-${PWD}/test_cu_mskf}"
mkdir -p "${OUT_DIR}"
cd "${OUT_DIR}"
wrf_fortran "${OUT_DIR}/driver_mskf_fortran" "${SCRIPT_DIR}/driver_mskf.f90"
S="${REPO_ROOT}/src"
"${HEAVY}" --light g++ -std=c++20 -DCPPWRF_DOUBLE -O3 ${CPPWRF_TEST_MARCH:--march=native} -ffp-contract=off -fopenmp \
  -I"${S}" "${SCRIPT_DIR}/driver_mskf.cpp" "${S}/phys/cu_mskf/mskf.cpp" "${S}/phys/cu_kf/kf_tables.cpp" \
  "${S}/core/backend.cpp" "${S}/core/column_work.cpp" -lmvec -lm -o driver_mskf_cpu
# columns, levels, kfeta_trigger, species, DX, seed
for cfg in "2500 50 1 ice 15000 7" "2499 51 1 ice 5000 3" "2500 50 1 kessler 30000 4" "2500 44 1 none 24999.5 5" \
           "2501 50 3 ice 3000 6" "2500 45 1 ice 25000 8"; do
  set -- $cfg
  echo "=== MSKF ($1 columns x $2 levels, kfeta_trigger $3, species $4, DX $5 m) ==="
  python3 "${SCRIPT_DIR}/generate_inputs.py" --ncols "$1" --nlevels "$2" --trigger "$3" --mp "$4" --dx "$5" \
    --seed "$6" --out mskf_in.bin
  "${HEAVY}" --light ./driver_mskf_fortran mskf_in.bin mskf_wrf.bin
  OMP_NUM_THREADS=2 "${HEAVY}" --light ./driver_mskf_cpu mskf_in.bin mskf_cpp.bin
  python3 "${SCRIPT_DIR}/compare.py" mskf_wrf.bin mskf_cpp.bin mskf_in.bin --tol 0
done
echo "=== MSKF COLUMN DIFFERENTIAL TEST PASSED (BIT-IDENTICAL IN CPU FP64) ==="
