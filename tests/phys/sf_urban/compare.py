#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""compare.py WRF.bin CPP.bin: the SLUCM column outputs (driver_urban.f90 format), tolerance 0: every value
identical bit for bit (NaN equals only NaN, -0 differs from +0); an empty or truncated output fails."""
import sys

import numpy as np

NAMES = ["ts", "qs", "sh", "lh", "lh_kinematic", "sw", "alb", "lw", "g", "rn", "psim", "psih", "gz1oz0", "u10",
         "v10", "th2", "q2", "ust", "tr", "tb", "tg", "tc", "qc", "uc", "xxxr", "xxxb", "xxxg", "xxxc"] + \
        [f"trl{k}" for k in range(1, 5)] + [f"tbl{k}" for k in range(1, 5)] + [f"tgl{k}" for k in range(1, 5)] + \
        ["flxhumr", "flxhumb", "flxhumg", "cmr", "chr", "cmc", "chc"]


def main():
    a = np.fromfile(sys.argv[1], dtype="<u8")
    b = np.fromfile(sys.argv[2], dtype="<u8")
    if a.size == 0 or a.size != b.size or (a.size - 12) % len(NAMES):
        print(f"FAIL: sizes {a.size} / {b.size}")
        return 1
    if not np.array_equal(a[:12], b[:12]):
        print("FAIL: DZR / DZB / DZG differ")
        return 1
    ra = a[12:].reshape(-1, len(NAMES))
    rb = b[12:].reshape(-1, len(NAMES))
    fa, fb = ra.view("<f8"), rb.view("<f8")
    same = (ra == rb) | (np.isnan(fa) & np.isnan(fb))
    if same.all():
        print(f"PASS: {ra.shape[0]} point-steps, {len(NAMES)} values each, bit-identical")
        return 0
    bad = np.argwhere(~same)
    print(f"FAIL: {bad.shape[0]} values differ in {np.unique(bad[:, 0]).size} of {ra.shape[0]} point-steps")
    for c in np.unique(bad[:, 1]):
        rows = bad[bad[:, 1] == c][:, 0]
        d = np.abs(fa[rows, c] - fb[rows, c])
        print(f"  {NAMES[c]:14s} {rows.size:6d} first row {rows[0]} max |d| {np.nanmax(d):.3e} "
              f"({fa[rows[0], c]!r} vs {fb[rows[0], c]!r})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
