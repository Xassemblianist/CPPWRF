#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the multi-scale Kain-Fritsch column test (cu_physics = 11): the Kain-Fritsch test's soundings
(tests/phys/cu_kf/generate_inputs.py: deep, capped, shallow, dry, sheared, moderate and relaxing columns) on a grid
length DX, plus the fields of the dynamic convective time scale (HFX, UST, PBLH, ZOL: unstable, neutral and stable
surface layers, ZOL = 0 exactly in some columns) and stronger running-mean W in a third of the columns (MSKF below
25 km does not scale W to the 25 km grid).

Header: ncols, nlevels (int32), dt, dx, cudt (float64), stepcu, ktau, trigger, warm_rain, flag_qi, flag_qs, flag_qr
(int32); then u, v, th, t, qv, dz8w, pcps, rho, pi, w0avg (nlevels x ncols), w (nlevels+1 x ncols), raincv, pratec,
nca, cutop, cubot, hfx, ust, pblh, zol, xland (ncols), all float64."""
import argparse
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cu_kf"))
import generate_inputs as kf  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ncols", type=int, default=2500)
ap.add_argument("--nlevels", type=int, default=50)
ap.add_argument("--trigger", type=int, default=1)
ap.add_argument("--mp", default="ice", choices=("ice", "kessler", "none"))
ap.add_argument("--dx", type=float, default=15000.0)
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--out", default="input_mskf.bin")
a = ap.parse_args()

d = kf.generate_soundings(ncols=a.ncols, nlevels=a.nlevels, trigger=1, mp=a.mp)
rng = np.random.default_rng(a.seed)
n = a.ncols
strong = rng.random(n) < 0.33
d["w"][:, strong] *= 3.0
d["w0avg"][:, strong] *= 3.0
hfx = rng.uniform(-60.0, 450.0, n)
ust = rng.uniform(0.03, 0.9, n)
pblh = rng.uniform(80.0, 3200.0, n)
zol = rng.uniform(-6.0, 2.5, n)
zol[rng.random(n) < 0.05] = 0.0
xland = np.where(rng.random(n) < 0.3, 2.0, 1.0)
with open(a.out, "wb") as f:
    f.write(struct.pack("=ii ddd iiii iii", n, a.nlevels, d["dt"], a.dx, d["cudt"], d["stepcu"], d["ktau"], a.trigger,
                        d["warm_rain"], d["flag_qi"], d["flag_qs"], d["flag_qr"]))
    for k in ("u", "v", "th", "t", "qv", "dz8w", "pcps", "rho", "pi", "w0avg", "w"):
        f.write(np.ascontiguousarray(d[k], dtype=np.float64).tobytes())
    for x in (d["raincv"], d["pratec"], d["nca"], d["cutop"], d["cubot"], hfx, ust, pblh, zol, xland):
        f.write(np.ascontiguousarray(x, dtype=np.float64).tobytes())
print(f"{a.out}: {n} columns x {a.nlevels} levels, dx {a.dx}, trigger {a.trigger}, species {a.mp}")
