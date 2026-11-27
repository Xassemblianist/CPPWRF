#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""CPPWRF real-data run vs a WRF wrfout with the same input.

  compare_real.py <wrfout> <cppwrf_out_dir> [--interior N]

Per output time: max abs / relative L2 difference of U, V, W, PH (full), MU (full), P (full),
THM (theta_m), QVAPOR (and the hydrometeors, RAINNC, REFL_10CM when present), over the whole domain and over the interior (N points away from the
lateral boundaries, default 6). The caller sets the tolerances. Invalid comparisons exit nonzero, and so does a
NaN or infinity in any compared field of either run at any matched time (the table is printed first): the caller
gates only some fields at one time, and a non-finite value is never a tolerance question.
--y-slab: a periodic-y slab that WRF ran with e_sn = 3 (two mass rows, three V rows) and CPPWRF with one row: every
WRF row must equal the first bit for bit, and the first row is compared; any other shape difference still fails.
"""
import math
import os
import sys

import netCDF4 as nc
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify as V  # noqa: E402


def main(wrfout, d, inner, y_slab=False):
    ds = nc.Dataset(wrfout)
    steps = V.list_steps(d)
    xt = np.array(ds.variables["XTIME"][:], dtype=float) * 60.0
    if not np.isfinite(xt).all() or len(np.unique(xt)) != len(xt):
        sys.exit(f"{wrfout}: invalid or duplicate WRF XTIME")
    snaps = {}
    for step in steps:
        path = os.path.join(d, f"snap_{step:06d}.json")
        snap = V.read_snap(d, step)
        stamp = snap["_time"]
        if not math.isfinite(stamp) or snap["_step"] != step:
            sys.exit(f"{path}: invalid snapshot time or step")
        if stamp in snaps:
            sys.exit(f"{path}: duplicate snapshot time {stamp:g}s (also step {snaps[stamp]})")
        snaps[stamp] = step
    if not set(xt) & snaps.keys():
        sys.exit("no matched WRF and CPPWRF snapshot times")
    nonfinite = []
    print(f"{'t':>6} {'var':>6} {'max|d|':>11} {'relL2':>24} {'relL2_in':>10} {'max|wrf|':>10}")
    for it, t in enumerate(xt):
        if t not in snaps:
            continue
        sn = V.read_snap(d, snaps[t])
        def g(n):
            return np.array(ds.variables[n][it], dtype=float)

        def plane(n):
            a = sn[n]
            if a.ndim != 3 or a.shape[0] != 1:
                sys.exit(f"t={t:g} {n}: expected one CPPWRF level, got shape {a.shape}")
            return a[0]

        def full(pert, base):
            a, b = g(pert), g(base)
            if a.shape != b.shape:
                sys.exit(f"t={t:g} {pert}/{base}: WRF shapes {a.shape} and {b.shape} differ")
            return a + b

        pairs = {
            "U": (g("U"), sn["u"]), "V": (g("V"), sn["v"]), "W": (g("W"), sn["w"]),
            "PH": (full("PH", "PHB"), sn["ph"]), "MU": (full("MU", "MUB"), plane("mu")),
            "P": (full("P", "PB"), sn["p"]),
        }
        if "qv" in sn:  # dry ideal cases carry no vapour
            pairs["QV"] = (g("QVAPOR"), sn["qv"])
        for wn, cn in (("QCLOUD", "qc"), ("QRAIN", "qr"), ("QICE", "qi"), ("QSNOW", "qs"), ("QGRAUP", "qg"),
                       ("QNICE", "qni"), ("QNRAIN", "qnr"), ("TKE", "tke")):
            if wn in ds.variables and cn in sn:
                pairs[wn] = (g(wn), sn[cn])
        if "REFL_10CM" in ds.variables and "refl_10cm" in sn:  # do_radar_ref = 1
            pairs["REFL_10CM"] = (g("REFL_10CM"), sn["refl_10cm"])
        if "RAINNC" in ds.variables and "rainnc" in sn:
            pairs["RAINNC"] = (g("RAINNC"), plane("rainnc"))
        if "THM" in ds.variables and "theta_m" in sn:
            pairs["THM"] = (g("THM") + 300.0, sn["theta_m"])
        for k, (a, b) in pairs.items():
            if y_slab and a.ndim >= 2 and b.ndim == a.ndim and b.shape[-2] == 1 and a.shape[-2] > 1 \
                    and a.shape[:-2] == b.shape[:-2] and a.shape[-1] == b.shape[-1]:
                if not all(np.array_equal(a[..., j, :], a[..., 0, :]) for j in range(1, a.shape[-2])):
                    sys.exit(f"t={t:g} {k}: --y-slab but WRF's rows differ")
                a = a[..., :1, :]
            if a.shape != b.shape:
                sys.exit(f"t={t:g} {k}: WRF shape {a.shape} differs from CPPWRF shape {b.shape}")
            for side, x in (("WRF", a), ("CPPWRF", b)):
                n = int(np.size(x) - np.isfinite(x).sum())
                if n:
                    nonfinite.append(f"t={t:g} {k}: {n} non-finite values in {side}")
            e = b - a
            sl = (Ellipsis, slice(inner, -inner), slice(inner, -inner))
            rl = np.sqrt(np.sum(e * e) / max(np.sum(a * a), 1e-300))
            ri = np.sqrt(np.sum(e[sl] ** 2) / max(np.sum(a[sl] ** 2), 1e-300))
            print(f"{t:6.17g} {k:>6} {np.abs(e).max():11.3e} {rl:24.17g} {ri:10.2e} {np.abs(a).max():10.3e}")
    if nonfinite:
        sys.exit("; ".join(nonfinite))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    inner = int(sys.argv[sys.argv.index("--interior") + 1]) if "--interior" in sys.argv else 6
    main(sys.argv[1], sys.argv[2], inner, "--y-slab" in sys.argv)
