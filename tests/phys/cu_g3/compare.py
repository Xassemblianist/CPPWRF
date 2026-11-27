#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compares the outputs of driver_g3.f90 (WRF) and driver_g3.cpp: every value bit for bit (--tol 0) or within a
relative tolerance; prints per field the differing points and the largest difference. The shallow levels (K22 /
KBCON / KTOP_SHALLOW) are undefined in WRF without ishallow = 1 and are then not compared."""
import argparse

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wrf")
    ap.add_argument("cpp")
    ap.add_argument("--input", required=True)
    ap.add_argument("--tol", type=float, default=0.0)
    ap.add_argument("--model", action="store_true",
                    help="CPP is the model's CPPWRF_G3_DUMP file.out (RTHCUTEN ... PRATEC after the spreading only)")
    a = ap.parse_args()
    nx, ny, nz, avedx, ishallow, ichoice, fqc, fqi = np.fromfile(a.input, dtype="<i4", count=8)
    w = np.fromfile(a.wrf, dtype="<f8")
    c = np.fromfile(a.cpp, dtype="<f8")
    if a.model:
        w = w[w.size - c.size:]
    if w.size != c.size:
        print(f"FAIL: sizes differ ({w.size} vs {c.size})")
        return 1
    n3, n2 = nx * ny * nz, nx * ny
    names = [(n, n3) for n in ("CUGD_TTEN", "CUGD_QVTEN", "CUGD_TTENS", "CUGD_QVTENS", "CUGD_QCTEN", "RQCCUTEN_G3",
                               "RQICUTEN_G3", "GD_CLOUD", "GD_CLOUD2")]
    names += [(n, n2) for n in ("RAINCV_G3", "PRATEC_G3", "CUTOP", "CUBOT", "EDT_OUT", "XMB_SHALLOW", "K22_SHALLOW",
                                "KBCON_SHALLOW", "KTOP_SHALLOW", "KTOP_DEEP", "APR_GR", "APR_W", "APR_MC", "APR_ST",
                                "APR_AS", "APR_CAPMA", "APR_CAPME", "APR_CAPMI")]
    names += [("XF_ENS", 144 * n2), ("PR_ENS", 144 * n2)]
    names += [(n, n3) for n in ("RTHCUTEN", "RQVCUTEN", "RQCCUTEN", "RQICUTEN")]
    names += [(n, n2) for n in ("RAINCV", "PRATEC")]
    if a.model:
        names = names[-6:]
    if w.size != sum(m for _, m in names):  # an empty or truncated pair of files would compare empty slices
        print(f"FAIL: {w.size} values, expected {sum(m for _, m in names)}")
        return 1
    off, bad = 0, 0
    act = None
    for n, m in names:
        x, y = w[off:off + m], c[off:off + m]
        off += m
        if n == "RAINCV":
            act = int(np.count_nonzero(x))
        if ishallow != 1 and n in ("K22_SHALLOW", "KBCON_SHALLOW", "KTOP_SHALLOW"):
            print(f"  {n}: not compared (undefined in WRF without ishallow = 1)")
            continue
        if a.tol == 0:
            d = (x.view("<u8") != y.view("<u8")) & ~(np.isnan(x) & np.isnan(y))
        else:  # NaN or an infinity on one side differs at any tolerance (NaN > t is False)
            with np.errstate(invalid="ignore", over="ignore"):
                d = ~(np.abs(x - y) <= a.tol * np.maximum(np.abs(x), 1e-30)) & ~(x == y) & ~(np.isnan(x) & np.isnan(y))
        nonzero = int(np.count_nonzero(x))
        if d.any():
            bad += 1
            print(f"  {n}: {int(d.sum())} of {m} differ, max |d| {np.nanmax(np.abs(x - y)):.3e} (nonzero {nonzero})")
        else:
            print(f"  {n}: identical (nonzero {nonzero})")
    print(f"  columns with convective rain: {act} of {(nx - 8) * (ny - 8)} interior columns")
    if bad:
        print("FAIL")
        return 1
    if not act:
        print("FAIL: no convection in the patch")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
