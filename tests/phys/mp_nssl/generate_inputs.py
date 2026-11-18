#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tiles for the NSSL 2-moment (mp_physics 18 / 17) differential test: convective soundings with up- and downdrafts,
supersaturation (droplet activation, ice nucleation), cloud water, rain, ice, snow, graupel and hail with their numbers
and graupel / hail volumes across the density limits, activated CCN, zeros and tiny amounts near qxmin, and the
cumulus tendencies of cu_used. Several calls follow (itimestep 1 resets QNN and diagnoses numbers from mass)."""
import argparse
import struct

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nx", type=int, default=7)
    ap.add_argument("--ny", type=int, default=2)
    ap.add_argument("--nz", type=int, default=40)
    ap.add_argument("--dt", type=float, default=12.0)
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--start-it", type=int, default=1)
    ap.add_argument("--diag", default="1")       # per step 0/1, cycled
    ap.add_argument("--cu-used", type=int, default=0)
    ap.add_argument("--ssat", type=int, default=2)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    nx, ny, nz = a.nx, a.ny, a.nz
    shp = (ny, nz, nx)  # C order (j, k, i) = Fortran (i, k, j)
    zw = 17000.0 * np.linspace(0, 1, nz + 1) ** 1.2
    dzc = np.diff(zw)
    zm = 0.5 * (zw[1:] + zw[:-1])
    F = {k: np.zeros(shp) for k in ("th qv qc qr qi qs qh qhl ccw crw cci csw chw chl vhw vhl cn pii p w dz dn "
                                     "qrcuten qscuten qicuten qccuten").split()}

    def esw(t):
        tc = t - 273.15
        return 611.2 * np.exp(17.67 * tc / (t - 29.65))

    def maybe(n, prob, lo, hi):
        v = np.where(rng.random(n) < prob, 10 ** rng.uniform(lo, hi, n), 0.0)
        m = rng.random(n)
        v[m < 0.04] = 10 ** rng.uniform(-15, -12)
        return v

    for j in range(ny):
        for i in range(nx):
            ts = rng.uniform(285.0, 305.0)
            t = np.maximum(ts - rng.uniform(0.006, 0.0075) * zm, rng.uniform(200.0, 215.0))
            ps = rng.uniform(90000.0, 101500.0)
            p = ps * np.exp(-zm / 7600.0)
            rh = np.clip(rng.uniform(0.5, 1.04, nz) + rng.normal(0, 0.03, nz), 0.05, 1.08)
            es = esw(t)
            qv = rh * 0.622 * es / (p - es)
            rho = p / (287.04 * t * (1 + 0.608 * qv))
            pii = (p / 1.0e5) ** (287.04 / 1004.0)
            wmax = rng.choice([0.0, rng.uniform(1, 8), rng.uniform(10, 40), -rng.uniform(1, 10)])
            w = wmax * np.sin(np.pi * zm / zw[-1]) + rng.normal(0, 0.3, nz)
            warm, cold = t > 235.0, t < 273.15
            qc = np.where(warm, maybe(nz, 0.6, -6.0, -2.6), 0.0)
            qr = maybe(nz, 0.5, -7.0, -2.3)
            qi = np.where(cold, maybe(nz, 0.6, -8.0, -3.0), 0.0)
            qs = np.where(t < 278.0, maybe(nz, 0.5, -7.5, -2.8), 0.0)
            qg = maybe(nz, 0.45, -7.0, -2.2)
            qh = maybe(nz, 0.3, -7.0, -2.0)
            # numbers from mean diameters (per kg)
            def num(q, dlo, dhi, rhox):
                d = 10 ** rng.uniform(np.log10(dlo), np.log10(dhi), nz)
                return np.where(q > 0, q / (rhox * np.pi / 6 * d ** 3), 0.0)
            rhog = rng.uniform(170.0, 900.0, nz)
            rhoh = rng.uniform(500.0, 900.0, nz)
            ccw = num(qc, 4e-6, 4e-5, 1000.0)
            crw = num(qr, 8e-5, 4e-3, 1000.0)
            cci = num(qi, 1e-5, 4e-4, 500.0)
            csw = num(qs, 1e-4, 6e-3, 100.0)
            chw = num(qg, 3e-4, 8e-3, rhog)
            chl = num(qh, 2e-3, 3e-2, rhoh)
            # a few number-only / mass-only points
            for arr in (ccw, crw, cci, csw, chw, chl):
                m = rng.random(nz)
                arr[m < 0.03] = 0.0
            vhw = np.where(qg > 0, qg / rhog, 0.0)
            vhl = np.where(qh > 0, qh / rhoh, 0.0)
            cn = rng.uniform(0, 4.0e8, nz) * (rng.random(nz) < 0.8)
            F["th"][j, :, i] = t / pii; F["qv"][j, :, i] = qv; F["qc"][j, :, i] = qc; F["qr"][j, :, i] = qr
            F["qi"][j, :, i] = qi; F["qs"][j, :, i] = qs; F["qh"][j, :, i] = qg; F["qhl"][j, :, i] = qh
            F["ccw"][j, :, i] = ccw; F["crw"][j, :, i] = crw; F["cci"][j, :, i] = cci; F["csw"][j, :, i] = csw
            F["chw"][j, :, i] = chw; F["chl"][j, :, i] = chl; F["vhw"][j, :, i] = vhw; F["vhl"][j, :, i] = vhl
            F["cn"][j, :, i] = cn; F["pii"][j, :, i] = pii; F["p"][j, :, i] = p; F["w"][j, :, i] = w
            F["dz"][j, :, i] = dzc; F["dn"][j, :, i] = rho
            if a.cu_used:
                for nm, sc in (("qrcuten", 1e-6), ("qscuten", 5e-7), ("qicuten", 3e-7), ("qccuten", 1e-6)):
                    F[nm][j, :, i] = np.where(rng.random(nz) < 0.4, rng.uniform(-0.2, 1.0, nz) * sc, 0.0)
    order = ("th qv qc qr qi qs qh qhl ccw crw cci csw chw chl vhw vhl cn pii p w dz dn "
             "qrcuten qscuten qicuten qccuten").split()
    acc = [rng.uniform(0, 30, (ny, nx)) * (rng.random((ny, nx)) < 0.7) for _ in range(4)]
    diag = [int(c) for c in a.diag]
    with open(a.out, "wb") as f:
        f.write(struct.pack("<6id", nx, ny, nz, a.steps, a.cu_used, a.ssat, a.dt))
        for nm in order:
            f.write(np.ascontiguousarray(F[nm], dtype="<f8").tobytes())
        for x in acc:
            f.write(np.ascontiguousarray(x, dtype="<f8").tobytes())
        for s in range(a.steps):
            d = diag[s % len(diag)]
            f.write(struct.pack("<3i", a.start_it + s, d, nz if d else 1))


if __name__ == "__main__":
    main()
