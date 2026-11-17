# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF.bin CPP.bin IN.bin NAME: the outputs of driver_advection.f90 / driver_advection.cpp (per species
q_2 and q_1, then RQVFTEN; float64) bit for bit; prints the differing values and exits 1 on any difference."""
import struct
import sys

import numpy as np

wrf, cpp, inp, name = sys.argv[1:5]
with open(inp, "rb") as fh:
    nx, ny, nz, bc, nspec, p_qv, adv_opt = struct.unpack("<7i", fh.read(28))
a = np.fromfile(wrf, "<f8")
b = np.fromfile(cpp, "<f8")
n3 = nx * ny * nz
if a.size != b.size or a.size != (2 * nspec + 1) * n3:
    sys.exit(f"{name}: sizes {a.size} {b.size}, expected {(2 * nspec + 1) * n3}")
labels = [f"{f}{s}" for s in range(nspec) for f in ("q_2.", "q_1.")] + ["rqvften"]
bad = 0
for m, lab in enumerate(labels):
    x, y = a[m * n3:(m + 1) * n3].reshape(nz, ny, nx), b[m * n3:(m + 1) * n3].reshape(nz, ny, nx)
    d = x.view("<u8") != y.view("<u8")
    d &= ~((x == 0) & (y == 0))  # +0 / -0
    if d.any():
        bad += 1
        k, j, i = np.argwhere(d)[0]
        print(f"{name} adv_opt {adv_opt} {lab}: {d.sum()} of {n3} differ, first (i,j,k)=({i},{j},{k}) "
              f"wrf {x[k, j, i]!r} cppwrf {y[k, j, i]!r}, max |diff| {np.abs(x - y)[d].max():.3e}")
print(f"{name} adv_opt {adv_opt} ({['specified', 'periodic', 'open'][bc]}): "
      f"{'IDENTICAL' if not bad else 'DIFFERENT'} ({len(labels)} fields, {n3} points each)")
sys.exit(1 if bad else 0)
