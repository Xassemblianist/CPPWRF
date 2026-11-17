#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Slabs for the WDM5 / WDM6 / WDM7 differential test (ni x nj columns, nz levels): warm, mixed-phase and cold
soundings below and above saturation with cloud water, rain, cloud ice, snow, graupel (and hail) at every height and
their numbers (CCN NN, cloud NC, rain NR) from tiny to large (rain drops below and above the size limits, cloud
droplets that self-collect or autoconvert), amounts and numbers at the thresholds (qmin 1e-15, qcrmin 1e-9, ncmin 10,
nrmin 1e-2) and zeros, clear columns, land and water points (xland 1 / 2: the autoconversion threshold), the
precipitation accumulations of an earlier step."""
import argparse
import struct

import numpy as np

F3 = "th q qc qr qi qs qg qh nn nc nr den pii p delz".split()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ni", type=int, default=41)
    ap.add_argument("--nj", type=int, default=25)
    ap.add_argument("--nz", type=int, default=44)
    ap.add_argument("--dt", type=float, default=60.0)
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--scheme", type=int, default=6)
    ap.add_argument("--hail", type=int, default=0)
    ap.add_argument("--it0", type=int, default=1, help="itimestep of the first call (1: NN = ccn0 first)")
    ap.add_argument("--ccn", type=float, default=1.0e8)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    ni, nj, nz = a.ni, a.nj, a.nz
    shape = (nj, nz, ni)  # C order (j, k, i) = Fortran (i, k, j)
    f = {n: np.zeros(shape) for n in F3}
    acc = {n: rng.uniform(0.0, 20.0, (nj, ni)) * (rng.random((nj, ni)) < 0.5) for n in ("rain", "snow", "graupel", "hail")}
    xland = np.where(rng.random((nj, ni)) < 0.4, 2.0, 1.0)

    def esat(t, ice):
        tc = t - 273.15
        return np.where(ice & (t < 273.15), 611.2 * np.exp(22.46 * tc / (tc + 272.62)),
                        611.2 * np.exp(17.62 * tc / (tc + 243.12)))

    def amount(lo, hi, frac=0.5):
        v = np.where(rng.random(nz) < frac, 10 ** rng.uniform(lo, hi, nz), 0.0)
        m = rng.random(nz)
        v[m < 0.03] = 1.0e-15 * rng.uniform(0.5, 2.0)
        v[(m >= 0.03) & (m < 0.06)] = 1.0e-9 * rng.uniform(0.5, 2.0)
        v[(m >= 0.06) & (m < 0.08)] = 10 ** rng.uniform(-14, -9)
        return v

    for j in range(nj):
        for i in range(ni):
            ztop = rng.uniform(14000.0, 20000.0)
            zw = ztop * np.linspace(0, 1, nz + 1) ** rng.uniform(1.0, 1.4)
            dz = np.diff(zw)
            zm = 0.5 * (zw[1:] + zw[:-1])
            ts = rng.uniform(255.0, 305.0)
            t = np.maximum(ts - rng.uniform(0.005, 0.0085) * zm, rng.uniform(195.0, 215.0))
            t += rng.normal(0.0, 0.4, nz)
            ps = rng.uniform(80000.0, 103000.0)
            pc = ps * np.exp(-zm / rng.uniform(7000.0, 8200.0))
            rh = np.clip(rng.uniform(0.5, 1.1, nz) + rng.normal(0, 0.04, nz), 0.02, 1.25)
            es = esat(t, np.ones(nz, bool))
            qsat = 0.622 * es / (pc - es)
            qv = rh * qsat
            cloudy = rh > rng.uniform(0.8, 1.0)
            lc = np.where(cloudy, amount(-6.5, -2.6, 0.8), 0.0)
            lr, li, ls, lg, lh = amount(-8.5, -2.4), amount(-8.5, -3.0), amount(-8.5, -2.5), amount(-8.5, -2.5), \
                amount(-8.5, -2.5, 0.3)
            # numbers: CCN around ccn0 (and outside the 1e8..2e10 clamp), cloud droplets 1e5..1e10 /kg, rain drops
            # 1e-1..1e7 /kg; zeros and the thresholds
            nn = 10 ** rng.uniform(6.5, 10.5, nz) * (rng.random(nz) < 0.9)
            nc = np.where(rng.random(nz) < 0.85, 10 ** rng.uniform(5.0, 10.0, nz), 0.0)
            nr = np.where(rng.random(nz) < 0.85, 10 ** rng.uniform(-1.0, 7.0, nz), 0.0)
            m = rng.random(nz)
            nc[m < 0.04] = 10.0 * rng.uniform(0.5, 2.0)
            nr[(m >= 0.04) & (m < 0.08)] = 1.0e-2 * rng.uniform(0.5, 2.0)
            if (i + 3 * j) % 13 == 0:  # clear column
                lc[:] = lr[:] = li[:] = ls[:] = lg[:] = lh[:] = 0.0
                nc[:] = nr[:] = 0.0
            if (i + j) % 11 == 0:  # heavy precipitation
                lr *= 10.0
                ls *= 10.0
                lg *= 10.0
                lh *= 10.0
            pi_ = (pc / 1.0e5) ** (287.0 / 1004.5)
            f["th"][j, :, i] = t / pi_
            f["pii"][j, :, i] = pi_
            f["p"][j, :, i] = pc
            f["q"][j, :, i] = qv
            f["qc"][j, :, i] = lc
            f["qr"][j, :, i] = lr
            f["qi"][j, :, i] = li
            f["qs"][j, :, i] = ls
            f["qg"][j, :, i] = lg if a.scheme != 5 else 0.0
            f["qh"][j, :, i] = lh if a.scheme == 7 else 0.0
            f["nn"][j, :, i] = nn
            f["nc"][j, :, i] = nc
            f["nr"][j, :, i] = nr
            f["den"][j, :, i] = pc / (287.0 * t * (1.0 + 0.608 * qv))
            f["delz"][j, :, i] = dz
    with open(a.out, "wb") as o:
        o.write(struct.pack("<iiiiiiidd", ni, nj, nz, a.steps, a.scheme, a.hail, a.it0, a.dt, a.ccn))
        for n in F3:
            o.write(np.ascontiguousarray(f[n], dtype="<f8").tobytes())
        for arr in (xland, acc["rain"], acc["snow"], acc["graupel"], acc["hail"]):
            o.write(np.ascontiguousarray(arr, dtype="<f8").tobytes())


if __name__ == "__main__":
    main()
