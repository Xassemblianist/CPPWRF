#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Vortex-following nests: WRF v4.8.0's compute_500mb_height (the -r8 build, WRF_DIR) with time_for_move2's vortex
# search and move (driver_vortex.f90) against src/nest/vortex_center.hpp (driver_vortex.cpp, CPU FP64), byte for
# byte, on the scenarios of generate_inputs.py.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
. "${REPO_ROOT}/tests/phys/wrf_fortran.sh"
wrf_need r8
WORK_DIR="${DIFF_TEST_BUILD_DIR:-${CPPWRF_SCRATCH:-${HOME}/cppwrf_scratch}}/nest_vortex"
mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"
wrf_fortran "${WORK_DIR}/driver_vortex_fortran" "${SCRIPT_DIR}/driver_vortex.f90"
g++ -std=c++20 -O2 -ffp-contract=off -DCPPWRF_DOUBLE -I"${REPO_ROOT}/src" "${SCRIPT_DIR}/driver_vortex.cpp" \
  -o "${WORK_DIR}/driver_vortex_cpu"
n=0
for sc in vortex edge mountain flat box trunc; do
  for seed in 1 2; do
    python3 "${SCRIPT_DIR}/generate_inputs.py" in.bin --scenario "$sc" --seed "$seed"
    ./driver_vortex_fortran in.bin out_fortran.bin
    ./driver_vortex_cpu in.bin out_cpp.bin
    if ! cmp -s out_fortran.bin out_cpp.bin; then
      echo "FAIL: scenario $sc seed $seed: the outputs differ"
      python3 - out_fortran.bin out_cpp.bin <<'PY'
import sys
import numpy as np
a, b = (np.fromfile(f, "<f8") for f in sys.argv[1:3])
if a.size != b.size:
    print("sizes", a.size, b.size)
else:
    d = np.nonzero(a.view("<u8") != b.view("<u8"))[0]
    print("first differences at", d[:10], a[d[:10]], b[d[:10]])
PY
      exit 1
    fi
    tail -c $((8 * 8)) out_cpp.bin | python3 -c "import sys,struct; v=struct.unpack('<8d', sys.stdin.buffer.read()); print('$sc $seed: last move', int(v[4]), int(v[5]), 'vc', v[6], v[7])"
    n=$((n + 1))
  done
done
echo "=== vortex-following nest test PASSED ($n inputs, byte for byte) ==="
