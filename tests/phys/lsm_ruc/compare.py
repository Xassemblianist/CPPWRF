#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tolerance-0 comparison of the RUC LSM column test outputs (driver_ruc.f90 vs driver_ruc.cpp)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import colcmp  # noqa: E402

f, c, n, nsl, steps = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
F2 = ("SNOW SNOWH SNOWC CANWAT SNOALB ALBEDO EMISS LAI MAVAIL TSK SOILT1 TSNAV QVG QCG QSG HFX QFX LH ZNT VEGFRA Z0 "
      "SFCEXC SFCEVP SFCRUNOFF UDRUNOFF ACRUNOFF GRDFLX ACSNOM DEW QSFC CHKLOWQ SMSTAV SMSTOT RHOSNF PRECIPFR "
      "SNOWFALLAC CQS CHS T2 TH2 Q2").split()
F3 = "SMOIS SH2O TSLB SMFR3D KEEPFR3DFLAG".split()
names = [("init.ZNT", n), ("init.MAVAIL", n), ("init.SH2O", n * nsl), ("init.SMFR3D", n * nsl)]
for s in range(1, steps + 1):
    names += [(f"step{s}.{x}", n) for x in F2] + [(f"step{s}.{x}", n * nsl) for x in F3]
total = sum(m for _, m in names)
a = colcmp.read_exact(f, "<f8", total)
b = colcmp.read_exact(c, "<f8", total)
ok, o = True, 0
for name, m in names:
    ok &= colcmp.same(name, a[o:o + m], b[o:o + m], quiet=True)
    o += m
print(f"{len(names)} fields x {n} points: {'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
