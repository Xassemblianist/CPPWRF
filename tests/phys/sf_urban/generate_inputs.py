#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""generate_inputs.py TABLE OUT [--points N] [--steps S] [--lcz 0|1] [--seed K]: urban points for the SLUCM column
test (tests/phys/sf_urban): forcing as lsm passes it to SUBROUTINE urban, day and night, rain above and below the
1 mm/h threshold of IMP_SCHEME 1, first levels above and below ZR + 2 m (canopy wind branches), stable and unstable
canyons; the state of urban_var_init and perturbed. ZA stays above ZDC + Z0C + 2 m (WRF stops otherwise)."""
import argparse
import math
import struct

import numpy as np


def table_morphology(path):
    """ZR, ROOF_WIDTH, ROAD_WIDTH of the table and urban_param_init's ZDC, Z0C (Macdonald 1998)."""
    vals = {}
    with open(path) as f:
        for line in f:
            if line.startswith("#") or ":" not in line:
                continue
            name, rest = line.split(":", 1)
            name = name.strip()
            if name in ("Number of urban categories", "ZR", "ROOF_WIDTH", "ROAD_WIDTH"):
                vals[name] = [float(x) for x in rest.replace(",", " ").split()]
    n = int(vals["Number of urban categories"][0])
    out = []
    for lc in range(n):
        zr, roof, road = vals["ZR"][lc], vals["ROOF_WIDTH"][lc], vals["ROAD_WIDTH"][lc]
        lp, lf = roof / (road + roof), zr / (road + roof)
        zdc = zr * (1.0 + 4.43 ** (-lp) * (lp - 1.0))
        z0c = zr * (1.0 - zdc / zr) * math.exp(-(0.5 * 1.2 / 0.16 * (1.0 - zdc / zr) * lf) ** (-0.5))
        out.append((zr, zdc, z0c))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("table")
    ap.add_argument("out")
    ap.add_argument("--points", type=int, default=3000)
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--lcz", type=int, default=0)
    ap.add_argument("--seed", type=int, default=14)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    morph = table_morphology(a.table)
    with open(a.out, "wb") as f:
        f.write(struct.pack("<3i", a.points, a.steps, a.lcz))
        for p in range(a.points):
            ut = int(rng.integers(1, len(morph) + 1))
            zr, zdc, z0c = morph[ut - 1]
            zmin = zdc + z0c + 2.0 + 0.05
            if p % 4 == 0 and zr + 2.0 > zmin:  # first level within the canopy (UC = UA/2)
                za = rng.uniform(zmin, zr + 2.0)
            else:
                za = rng.uniform(max(zmin, zr + 2.01), max(zmin, zr + 2.01) + 60.0)
            ta = rng.uniform(250.0, 315.0)
            qa = rng.uniform(1e-4, 0.022)
            u1, v1 = rng.uniform(-15, 15), rng.uniform(-15, 15)
            ua = max(math.sqrt(u1 * u1 + v1 * v1), 1.0)
            day = p % 3 != 0
            cosz = rng.uniform(0.02, 0.99) if day else rng.uniform(-0.99, -0.01)
            ssg = rng.uniform(5.0, 1000.0) * max(cosz, 0.05) if day else 0.0
            llg = rng.uniform(180.0, 450.0)
            rain = 0.0 if p % 5 else rng.choice([rng.uniform(0.0, 1.0), rng.uniform(1.0, 30.0)])
            psfc = rng.uniform(70000.0, 103000.0)
            rhoo = psfc / (287.04 * ta * (1.0 + 0.61 * qa))
            declin = rng.uniform(-0.409, 0.409)
            omg = rng.uniform(-math.pi, math.pi)
            xlat = rng.uniform(-60.0, 70.0)
            delt = float(rng.choice([30.0, 60.0, 90.0]))
            znt = rng.uniform(0.1, 2.0)
            chs = max(rng.uniform(0.0, 0.08), 1e-2)
            chs2 = max(rng.uniform(0.0, 0.05), 1e-2)
            tsk = ta + rng.uniform(-8.0, 12.0)
            tl = [tsk + rng.uniform(-6, 6) for _ in range(3)]
            trl = [tl[0], 0.5 * (tl[0] + tl[1]), tl[1], tl[1] + (tl[2] - tl[1]) * 0.29]
            pert = lambda x: x + rng.uniform(-3.0, 3.0)  # noqa: E731
            st = [pert(tsk), pert(tsk), pert(tsk), pert(tsk), rng.uniform(0.005, 0.02), rng.uniform(0.5, 5.0),
                  0.0, 0.0, 0.0, 0.0]
            st += [pert(x) for x in trl] + [pert(x) for x in trl] + [pert(x) for x in tl + [tl[2]]]
            st += [0.0, 0.0, 0.0]
            if p % 2:  # SFCDIF_URB's AKMS / AKHS of an earlier step
                st += [rng.uniform(0.0, 0.5), rng.uniform(0.0, 0.3), rng.uniform(0.0, 0.5), rng.uniform(0.0, 0.3)]
            else:
                st += [0.0, 0.0, 0.0, 0.0]
            fv = [ta, qa, ua, u1, v1, ssg, llg, rain, rhoo, za, declin, cosz, omg, xlat, delt, znt, chs, chs2]
            f.write(struct.pack("<i", ut))
            f.write(struct.pack("<18d", *fv))
            f.write(struct.pack("<29d", *st))


if __name__ == "__main__":
    main()
