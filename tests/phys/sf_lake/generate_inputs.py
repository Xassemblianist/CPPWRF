#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the lake model column test (driver_lake.f90 / driver_lake.cpp): a row of points (warm, deep, shallow,
freezing and snow-covered lakes, ice-covered lake points, a non-lake point) and nsteps steps of forcing.
  generate_inputs.py OUT --config A|B|C [--steps N] [--seed S]
A: LAKEFLAG 1, use_lakedepth 0, lakedepth_default 50; B: LAKE_DEPTH from the input (lake_depth_flag 1, use_lakedepth
1, lakedepth_default 0: the 1 m column where LAKE_DEPTH <= 0); C: LAKEFLAG 0 (water with HGT >= lake_min_elev 5),
lakedepth_default 10. Little-endian float64 (the -r8 build's REAL), Fortran order."""
import argparse
import math
import random
import struct


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--config", default="A")
    ap.add_argument("--steps", type=int, default=240)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    cfg = {"A": (50.0, 0, 0, 1, 5.0), "B": (0.0, 1, 1, 1, 5.0), "C": (10.0, 0, 0, 0, 5.0)}[a.config]
    ldef, use_ld, ld_flag, lakeflag, lmin = cfg
    # (tsk, snow kg/m2, xice, lakemask, ht, lake_depth, isltyp, ivgtyp, t_air offset, kind)
    pts = []
    for i, t in enumerate((299.0, 293.5, 287.0, 281.0, 278.0, 277.3, 276.0)):  # warm to the density maximum
        # air 2 K warmer (stable) or 12 K colder (unstable, zeta < -zetam in light wind)
        pts.append((t, 0.0, 0.0, 1, rnd.uniform(0, 1500), rnd.choice([3.0, 12.0, 40.0, 120.0, 300.0]), 14, 17,
                    2.0 if i % 2 else -12.0))
    for t, sn in ((273.5, 0.0), (272.0, 0.0), (270.0, 4.0), (268.0, 7.0), (266.0, 10.0), (265.0, 20.0),
                  (263.0, 30.0), (262.0, 50.0), (260.0, 70.0), (258.0, 100.0), (255.0, 200.0), (271.0, 0.5)):
        pts.append((t, sn, 0.0, 1, rnd.uniform(5, 900), rnd.choice([-2.0, 0.0, 8.0, 60.0]), 14, 17, -6.0))
    pts.append((270.5, 3.0, 0.8, 1, 400.0, 20.0, 14, 15, -8.0))   # sea ice on a lake point: IVGTYP / XLAND / XICE
    pts.append((290.0, 0.0, 0.0, 0, 3.0, -3.0, 14, 17, 1.0))      # water below lake_min_elev / not a lake
    pts.append((285.0, 0.0, 0.0, 0, 200.0, -2.0, 3, 5, 1.0))      # land
    pts.append((288.0, 0.0, 0.0, 1, 250.0, 30.0, 7, 17, -1.0))    # another soil category
    n = len(pts)
    xland = [2.0 if p[7] == 17 else (2.0 if p[2] > 0 else 1.0) for p in pts]
    hdr = [n, a.steps, 60.0, ldef, use_ld, ld_flag, lakeflag, lmin, 0.5, 17]
    with open(a.out, "wb") as f:
        w = lambda v: f.write(struct.pack("<%dd" % len(v), *v))
        w(hdr)
        w([p[4] for p in pts]); w([p[1] for p in pts]); w([p[0] for p in pts]); w([p[2] for p in pts]); w(xland)
        w([float(p[3]) for p in pts]); w([p[5] for p in pts])
        w([rnd.uniform(20, 75) for p in pts])  # latitude
        w([float(p[6]) for p in pts]); w([float(p[7]) for p in pts])
        for s in range(a.steps):
            ph = 2 * math.pi * s / 240.0
            t = [p[0] + p[8] + 3.0 * math.sin(ph + i) + rnd.uniform(-0.3, 0.3) for i, p in enumerate(pts)]
            psfc = [rnd.uniform(85000, 101500) for _ in pts]
            p2 = [x - rnd.uniform(500, 900) for x in psfc]
            dz = [rnd.uniform(40, 60) for _ in pts]
            qv = [min(0.02, 0.003 + 0.0006 * max(0.0, x - 255.0) * rnd.uniform(0.3, 1.0)) for x in t]
            wmax = [1.5 if (i < 7 and i % 2 == 0 and s % 3 == 0) else 12.0 for i in range(len(pts))]
            u = [rnd.uniform(-x, x) for x in wmax]
            v = [rnd.uniform(-x, x) for x in wmax]
            glw = [rnd.uniform(180, 380) for _ in pts]
            em = [rnd.choice([0.98, 0.97, 0.95]) for _ in pts]
            precip = s % 60 < 25 and s > 10
            rb = [rnd.uniform(0.05, 0.6) if precip and rnd.random() < 0.8 else 0.0 for _ in pts]
            sw = [max(0.0, 800.0 * math.sin(ph)) * rnd.uniform(0.5, 1.0) for _ in pts]
            alb = [rnd.choice([0.08, 0.3, 0.6]) for _ in pts]
            for vec in (t, psfc, p2, dz, qv, u, v, glw, em, rb, sw, alb):
                w(vec)


if __name__ == "__main__":
    main()
