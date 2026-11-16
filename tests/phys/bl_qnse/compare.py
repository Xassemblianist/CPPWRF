#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF.bin CPP.bin --cols N --nz K: QNSESFCINIT's tables, QNSESFC's and QNSEPBL's outputs bit for bit
(NaN equals only NaN, -0 differs from +0); exit 1 on any difference."""
import argparse
import sys
import numpy as np

KZTM = 10001
SFC = ["qsfc", "thz0", "qz0", "uz0", "vz0", "ust", "znt", "pblh", "rmol", "akhs", "akms", "rib", "chs", "chs2",
       "cqs2", "hfx", "qfx", "flx_lh", "flhc", "flqc", "qgh", "cpm", "ct", "u10", "v10", "t02", "th02", "tshltr",
       "th10", "q02", "qshltr", "q10", "pshltr", "u10e", "v10e"]
PBL3 = ["tke", "exch_h", "exch_m", "el_pbl", "rublten", "rvblten", "rthblten", "rqvblten", "rqcblten"]
PBL2 = ["pbl:pblh", "pbl:kpbl", "pbl:ct", "pbl:qsfc", "pbl:thz0", "pbl:qz0"]
p = argparse.ArgumentParser()
p.add_argument("fortran")
p.add_argument("cpp")
p.add_argument("--cols", type=int, required=True)
p.add_argument("--nz", type=int, required=True)
a = p.parse_args()
n, n3 = a.cols, a.cols * a.nz
blocks = [(x, KZTM) for x in ("psim1", "psih1", "psim2", "psih2")] + [("bounds", 6)] + [(x, n) for x in SFC] + \
    [(x, n3) for x in PBL3] + [(x, n) for x in PBL2]
f = np.fromfile(a.fortran, dtype="<f8")
c = np.fromfile(a.cpp, dtype="<f8")
total = sum(m for _, m in blocks)
if f.size != total or c.size != total:
    print(f"ERROR: sizes {f.size} / {c.size}, expected {total}")
    sys.exit(1)
ok, lo = True, 0
for name, m in blocks:
    x, y = f[lo:lo + m], c[lo:lo + m]
    lo += m
    diff = x.view("<u8") != y.view("<u8")
    nd = int(diff.sum())
    mx = float(np.nanmax(np.abs(x[diff] - y[diff]))) if nd else 0.0
    print(f"{name:<10} nonzero {int((x != 0).sum()):>8}  differing {nd:>8}  max|d| {mx:.3e}  "
          f"{'PASS' if nd == 0 else 'FAIL'}")
    ok = ok and nd == 0
print("QNSE column test", "PASSED" if ok else "FAILED")
sys.exit(0 if ok else 1)
