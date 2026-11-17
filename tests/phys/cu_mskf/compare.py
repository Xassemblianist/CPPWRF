#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF.bin CPP.bin INPUT.bin [--tol T]: the outputs of driver_mskf.f90 and driver_mskf.cpp, tolerance 0
as tests/verify/exact.py (tests/phys/colcmp.py); prints how many columns convect and carry a momentum tendency."""
import argparse
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import colcmp  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("wrf")
ap.add_argument("cpp")
ap.add_argument("inp")
ap.add_argument("--tol", type=float, default=0.0)
a = ap.parse_args()
with open(a.inp, "rb") as f:
    n, nl = struct.unpack("=ii", f.read(8))
names = [("raincv", 1), ("pratec", 1), ("nca", 1), ("cutop", 1), ("cubot", 1)] + \
        [(x, nl) for x in ("w0avg", "rthcuten", "rqvcuten", "rqccuten", "rqrcuten", "rqicuten", "rqscuten", "rucuten",
                           "rvcuten", "cldfra_dp", "cldfra_sh", "qc_kf", "qi_kf")]
total = sum(m for _, m in names) * n
fw = colcmp.read_exact(a.wrf, np.float64, total)
fc = colcmp.read_exact(a.cpp, np.float64, total)
for b in (a.wrf, a.cpp):
    if os.path.getsize(b) != total * 8:
        print(f"{b}: {os.path.getsize(b)} bytes, expected {total * 8}")
        sys.exit(1)
ok, o = True, 0
for name, m in names:
    x, y = fw[o:o + m * n], fc[o:o + m * n]
    o += m * n
    if a.tol == 0:
        ok = colcmp.same(name, x, y) and ok
    else:
        d = np.max(np.abs(x - y) / np.maximum(np.abs(x), 1e-30)) if x.size else 0.0
        print(f"{name}: max rel {d:.3e}")
        ok = ok and d <= a.tol
    if name == "raincv":
        print(f"  {np.count_nonzero(x > 0)} of {n} columns rain")
    if name == "rucuten":
        print(f"  {np.count_nonzero(np.any(x.reshape(nl, n) != 0, axis=0))} columns with RUCUTEN")
    if name == "cldfra_sh":
        print(f"  {np.count_nonzero(np.any(x.reshape(nl, n) > 0, axis=0))} columns with shallow cloud")
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
