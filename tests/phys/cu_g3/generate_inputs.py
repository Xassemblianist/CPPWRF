#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Synthetic nx x ny x nz patch for the Grell-3D test (driver_g3.f90 / driver_g3.cpp): moist conditionally unstable
soundings (surface 293-305 K, 8.3 K/km below 3 km and 6-7.5 K/km above to a 12-14 km tropopause, relative humidity
60-98 % in the lowest 1.5 km), smooth in i and j (the scheme reads the 3 x 3 neighbours), with advective / PBL forcing that moistens and
cools, vertical motion, land and sea, day and night (gsw), a few dry and cold columns.
Layout (little endian): int32 nx ny nz cugd_avedx ishallow ichoice f_qc f_qi; float64 dt dx; 3D fields (j, k, i
order) u v t q p pi rho rthften rqvften rthblten rqvblten (nz levels), w p8w (nz+1); 2D ht xland gsw kpbl apr_* (8)."""
import argparse

import numpy as np

RD, CP, G = 287.0, 7.0 * 287.0 / 2.0, 9.81


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nx", type=int, default=40)
    ap.add_argument("--ny", type=int, default=30)
    ap.add_argument("--nz", type=int, default=44)
    ap.add_argument("--avedx", type=int, default=1)
    ap.add_argument("--ishallow", type=int, default=0)
    ap.add_argument("--ichoice", type=int, default=0)
    ap.add_argument("--qc", type=int, default=1)
    ap.add_argument("--qi", type=int, default=1)
    ap.add_argument("--dt", type=float, default=60.0)
    ap.add_argument("--dx", type=float, default=12000.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    nx, ny, nz = a.nx, a.ny, a.nz
    ii, jj = np.meshgrid(np.arange(nx), np.arange(ny))  # (ny, nx)

    def smooth(amp, base=0.0):
        ph = rng.uniform(0, 2 * np.pi, 4)
        f = (np.sin(2 * np.pi * ii / rng.uniform(8, 20) + ph[0]) * np.cos(2 * np.pi * jj / rng.uniform(8, 20) + ph[1])
             + 0.5 * np.sin(2 * np.pi * (ii + jj) / rng.uniform(6, 15) + ph[2]))
        return base + amp * f / 1.5 + 0.1 * amp * rng.standard_normal((ny, nx))

    ht = np.clip(smooth(500.0, 300.0), 0.0, 1500.0)
    xland = np.where(smooth(1.0) > 0.4, 2.0, 1.0)
    ht = np.where(xland > 1.5, 0.0, ht)
    gsw = np.where(smooth(1.0) > -0.2, np.clip(smooth(200.0, 450.0), 0.0, None), 0.0)
    tsfc = smooth(5.0, 299.0)
    lapse = smooth(0.0007, 0.0068)
    ztrop = smooth(800.0, 13000.0)
    rh0 = np.clip(smooth(0.1, 0.9), 0.6, 0.98)
    dry = rng.random((ny, nx)) < 0.08
    rh0 = np.where(dry, 0.3, rh0)
    tsfc = np.where(rng.random((ny, nx)) < 0.05, 268.0, tsfc)
    psfc = 101000.0 * np.exp(-ht / 8000.0) + smooth(500.0)
    ptop = 5000.0
    # eta-like full levels (bottom to top), faces nz+1
    eta_w = np.linspace(1.0, 0.0, nz + 1) ** 1.4
    p8w = ptop + (psfc[None] - ptop) * eta_w[:, None, None]  # (nz+1, ny, nx)
    p = 0.5 * (p8w[:-1] + p8w[1:])
    # temperature from the height of each level (iterate the hypsometric equation)
    z8w = np.zeros_like(p8w)
    z8w[0] = ht
    t = np.zeros_like(p)
    q = np.zeros_like(p)
    for k in range(nz):
        zk = z8w[k] + 50.0
        for _ in range(3):
            zag = zk - ht
            zl = np.minimum(zag, 3000.0)  # a steep lower lapse rate (8.3 K/km) under the mean one
            tk = np.where(zag < ztrop, tsfc - 0.0083 * zl - lapse * (zag - zl), tsfc - 0.0083 * 3000.0 - lapse * (ztrop - 3000.0))
            rh = np.clip(np.where(zag < 1500.0, rh0, rh0 * np.exp(-(zag - 1500.0) / 3500.0) + 0.08), 0.05, 0.99)
            es = 610.78 * np.exp(17.27 * (tk - 273.16) / (tk - 35.86))
            qk = np.clip(rh * 0.622 * es / (p[k] - es), 1e-7, 0.03)
            tvk = tk * (1.0 + 0.608 * qk)
            dz = RD * tvk / G * np.log(p8w[k] / p8w[k + 1])
            zk = z8w[k] + 0.5 * dz
        t[k], q[k] = tk, qk
        z8w[k + 1] = z8w[k] + dz
    tv = t * (1.0 + 0.608 * q)
    rho = p / (RD * tv)
    pi = (p / 1.0e5) ** (RD / CP)
    zm = 0.5 * (z8w[:-1] + z8w[1:])
    shear = smooth(0.0015, 0.002)
    u = smooth(3.0) + shear * (zm - ht)
    v = smooth(3.0, 1.0) + 0.5 * shear * (zm - ht)
    wamp = smooth(0.4, 0.05)
    w = wamp[None] * np.sin(np.pi * np.clip((z8w - ht) / 12000.0, 0.0, 1.0)) + 0.02 * rng.standard_normal(z8w.shape)
    w[0] = 0.0
    prof = np.exp(-((zm - ht) / 3000.0) ** 2)
    rthften = (smooth(1.5e-4, -0.5e-4))[None] * prof + 1e-6 * rng.standard_normal(p.shape)
    rqvften = (smooth(1.0e-7, 0.6e-7))[None] * prof + 1e-9 * rng.standard_normal(p.shape)
    pbl = np.exp(-((zm - ht) / 1000.0) ** 2)
    rthblten = (smooth(2.0e-4, 1.0e-4))[None] * pbl
    rqvblten = (smooth(1.0e-7, 0.5e-7))[None] * pbl
    kpbl = np.clip(np.rint(smooth(4.0, 6.0)), 1, nz // 2).astype(float)
    aprs = [np.abs(smooth(0.5, 0.3)) for _ in range(8)]
    with open(a.out, "wb") as f:
        np.array([nx, ny, nz, a.avedx, a.ishallow, a.ichoice, a.qc, a.qi], "<i4").tofile(f)
        np.array([a.dt, a.dx], "<f8").tofile(f)
        for x in (u, v, t, q, p, pi, rho, rthften, rqvften, rthblten, rqvblten, w, p8w):
            np.ascontiguousarray(np.transpose(x, (1, 0, 2)), "<f8").tofile(f)  # (j, k, i)
        for x in [ht, xland, gsw, kpbl] + aprs:
            np.ascontiguousarray(x, "<f8").tofile(f)  # (j, i)


if __name__ == "__main__":
    main()
