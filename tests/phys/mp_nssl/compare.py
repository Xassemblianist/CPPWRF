#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compares the NSSL test outputs of WRF and the C++ port bit for bit (tolerance 0) and prints the first differences."""
import struct
import sys

import numpy as np

N3 = "th qv qc qr qi qs qh qhl ccw crw cci csw chw chl vhw vhl cn dbz ssat3d ssati re_cloud re_ice re_snow".split()
N2 = "rainnc rainncv snownc snowncv hailnc hailncv grplnc grplncv sr".split()


def main():
    inp, fo, co = sys.argv[1:4]
    with open(inp, "rb") as f:
        nx, ny, nz, nsteps = struct.unpack("<6id", f.read(32))[:4]
    a = np.fromfile(fo, "<u8")
    b = np.fromfile(co, "<u8")
    n3, n2 = nx * nz * ny, nx * ny
    per = len(N3) * n3 + len(N2) * n2
    if a.size != per * nsteps or b.size != a.size:
        print(f"FAIL: output sizes {a.size} {b.size}, expected {per * nsteps}")
        sys.exit(1)
    bad = 0
    for s in range(nsteps):
        off = s * per
        for q, nm in enumerate(N3 + N2):
            n = n3 if q < len(N3) else n2
            x, y = a[off:off + n], b[off:off + n]
            off += n
            d = np.nonzero(x != y)[0]
            if d.size:
                bad += 1
                xf, yf = x.view("<f8"), y.view("<f8")
                k = d[0]
                if q < len(N3):
                    i, kk, j = k % nx + 1, (k // nx) % nz + 1, k // (nx * nz) + 1
                    where = f"i={i} k={kk} j={j}"
                else:
                    where = f"i={k % nx + 1} j={k // nx + 1}"
                print(f"step {s + 1} {nm}: {d.size} differ; first {where}: WRF {xf[k]!r} C++ {yf[k]!r}")
    if bad:
        print("FAIL")
        sys.exit(1)
    print(f"PASS: {nsteps} calls, {nx} x {ny} x {nz}, all fields bit-identical")


if __name__ == "__main__":
    main()
