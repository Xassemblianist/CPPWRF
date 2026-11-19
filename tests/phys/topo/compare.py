#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF_OUT CPP_OUT NX NY N: the outputs byte for byte; prints what differs and the coverage."""
import sys
import numpy as np
a, b = (np.fromfile(x, dtype="<f8") for x in sys.argv[1:3])
nx, ny, n = map(int, sys.argv[3:6])
if a.shape != b.shape:
    sys.exit(f"FAIL: sizes {a.shape} {b.shape}")
bad = np.flatnonzero(a.view("<u8") != b.view("<u8"))
m = nx * ny
per = 1 + 2 * m
nt = (a.size - 4 * n) // per
shaded = sum(int(a[t * per + 1:t * per + 1 + m].sum()) for t in range(nt))
sw = a[nt * per: nt * per + n]
print(f"{a.size} values, {shaded} shaded point-times over {nt} times, {int((sw > 1e-3).sum())} daytime SWDOWN "
      f"points")
if bad.size:
    sys.exit(f"FAIL: {bad.size} values differ, first at {bad[:5]}: {a[bad[:5]]} vs {b[bad[:5]]}")
if shaded == 0:
    sys.exit("FAIL: no shaded point (the test does not reach the shading)")
print("PASS (byte for byte)")
