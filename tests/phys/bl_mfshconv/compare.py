#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF.bin CPP.bin --cols N --nz K: MFSHCONVPBL's outputs bit for bit (NaN equals only NaN, -0 differs
from +0), with what the tile exercised (updrafts, clouds, too deep clouds); exit 1 on any difference."""
import argparse
import sys
import numpy as np

NAMES = ["rthblten", "rqvblten", "rqcblten", "rublten", "rvblten", "rc_mf", "massflux", "entr", "thl_up", "rt_up",
         "frac_up"]
p = argparse.ArgumentParser()
p.add_argument("fortran")
p.add_argument("cpp")
p.add_argument("--cols", type=int, required=True)
p.add_argument("--nz", type=int, required=True)
a = p.parse_args()
n3 = a.cols * a.nz
f = np.fromfile(a.fortran, dtype="<f8")
c = np.fromfile(a.cpp, dtype="<f8")
if f.size != n3 * len(NAMES) or c.size != f.size:
    print(f"ERROR: sizes {f.size} / {c.size}, expected {n3 * len(NAMES)}")
    sys.exit(1)
ok = True
for m, name in enumerate(NAMES):
    x, y = f[m * n3:(m + 1) * n3], c[m * n3:(m + 1) * n3]
    diff = x.view("<u8") != y.view("<u8")
    nd = int(diff.sum())
    mx = float(np.nanmax(np.abs(x[diff] - y[diff]))) if nd else 0.0
    print(f"{name:<10} nonzero {int((x != 0).sum()):>8}  differing {nd:>8}  max|d| {mx:.3e}  "
          f"{'PASS' if nd == 0 else 'FAIL'}")
    ok = ok and nd == 0
fr = f[10 * n3:11 * n3].reshape(a.nz, a.cols)
rc = f[5 * n3:6 * n3].reshape(a.nz, a.cols)
top = np.where(fr > 0, np.arange(a.nz)[:, None], -1).max(axis=0)
print(f"tile: {int((fr[0] > 0).sum())} of {a.cols} columns with an updraft, {int((rc > 0).any(axis=0).sum())} with "
      f"MF clouds, updraft tops at levels {np.bincount(top[top >= 0] + 1).nonzero()[0].min() if (top >= 0).any() else 0}"
      f"..{top.max() + 1}")
print("EDKF column test", "PASSED" if ok else "FAILED")
sys.exit(0 if ok else 1)
