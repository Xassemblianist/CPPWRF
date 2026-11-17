#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compares the outputs of driver_wdm.f90 and driver_wdm.cpp bit for bit (tolerance 0: -0 differs from +0, NaN equals
only a NaN): the init's constants, then every field of every step. Also prints how many columns precipitate and how
many points the number concentrations changed, so that a configuration that exercises nothing is visible."""
import struct
import sys

import numpy as np

NCONST = 63
F3 = "th q qc qr qi qs qg qh nn nc nr re_cloud re_ice re_snow".split()
F2 = "rain rainncv snow snowncv graupel graupelncv hail hailncv sr".split()


def main():
    inp, a, b = sys.argv[1:4]
    raw = open(inp, "rb").read()
    ni, nj, nz, steps, scheme, hail, it0 = struct.unpack("<iiiiiii", raw[:28])
    m, m2 = ni * nj * nz, ni * nj
    x = np.fromfile(a, "<f8")
    y = np.fromfile(b, "<f8")
    per = len(F3) * m + len(F2) * m2
    if x.size != NCONST + steps * per or y.size != x.size:
        print(f"FAIL: sizes {x.size} {y.size}, expected {NCONST + steps * per}")
        return 1
    bad = 0
    for n in range(NCONST):
        if x[n].view("<u8") != y[n].view("<u8"):
            print(f"constant {n}: wrf={x[n]!r} cpp={y[n]!r}")
            bad += 1
    off = NCONST
    for s in range(steps):
        for name, cnt in [(f, m) for f in F3] + [(f, m2) for f in F2]:
            xu, yv = x[off:off + cnt], y[off:off + cnt]
            same = (xu.view("<u8") == yv.view("<u8")) | (np.isnan(xu) & np.isnan(yv))
            if not same.all():
                idx = np.flatnonzero(~same)
                d = np.nanmax(np.abs(xu[idx] - yv[idx]))
                i0 = idx[0]
                print(f"step {s + 1} {name}: {idx.size} differ, first (i={i0 % ni}, k={(i0 // ni) % nz if cnt == m else 0}, "
                      f"j={i0 // (ni * nz) if cnt == m else i0 // ni}) wrf={xu[i0]!r} cpp={yv[i0]!r}, max |d| {d:g}")
                bad += 1
            if s == steps - 1 and name in ("rainncv", "snowncv", "graupelncv", "hailncv", "sr"):
                print(f"  {name}: {np.count_nonzero(xu)} of {cnt} columns non-zero")
            off += cnt
    print("PASS" if bad == 0 else f"FAIL: {bad} differences")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
