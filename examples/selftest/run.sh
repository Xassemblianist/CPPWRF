#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The check of a CPPWRF installation: a tiny em_quarter_ss run (10 x 10 x 10 points, 5 steps, 1 s of CPU) through the
# drop-in wrf.exe, and the SHA-256 of its last frame's values (cppwrf.wrfout.values_hash). CPU FP64 and GPU FP64 builds
# must give expected_fp64.txt exactly (they are bit-identical); GPU FP32 gives its own hash and is checked for a
# successful, finite run.
#   examples/selftest/run.sh [cpu-fp64|gpu-fp64|gpu-fp32] [WORK]   (WORK: default a new temporary directory)
# cppwrf-wrf from PATH, else the source tree's tools/; the binary as cppwrf-wrf finds it (CPPWRF_EXE, bin/, build*/).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODE=${1:-cpu-fp64}
WORK=${2:-$(mktemp -d)}
WRF=$(command -v cppwrf-wrf || echo "$HERE/../../tools/cppwrf-wrf")
mkdir -p "$WORK"
cp "$HERE/namelist.input" "$HERE/input_sounding" "$WORK/"
cd "$WORK"
OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} "$WRF" --mode "$MODE" > /dev/null || { cat rsl.error.0000; exit 1; }
grep -q "SUCCESS COMPLETE WRF" rsl.out.0000
PY=$(dirname "$(readlink -f "$WRF")")
hash=$(python3 - "$PY" <<'PYEOF'
import os, sys
for p in (os.path.join(sys.argv[1], "..", "python"), os.path.join(sys.argv[1], "..", "share", "cppwrf", "python")):
    if os.path.isdir(os.path.join(p, "cppwrf")):
        sys.path.insert(0, p)
        break
import numpy as np
from cppwrf import open_wrfout
from cppwrf.wrfout import values_hash
w = open_wrfout(".", xarray=False)
assert all(np.isfinite(w[v]).all() for v in ("U", "V", "W", "T", "PH", "MU")), "non-finite values"
print(values_hash("."))
PYEOF
)
echo "selftest $MODE: $hash ($WORK)"
case $MODE in
  *fp64)
    want=$(cat "$HERE/expected_fp64.txt")
    if [ "$hash" = "$want" ]; then echo "PASS: identical to the reference values (expected_fp64.txt)"
    else echo "FAIL: expected $want"; exit 1; fi ;;
  *) echo "PASS: the run succeeded with finite values (FP32 has no reference hash)" ;;
esac
