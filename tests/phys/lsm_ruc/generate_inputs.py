#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the RUC LSM column test (driver_ruc.f90 / driver_ruc.cpp): land with and without snow (thin, one- and
two-layer snow, melting), frozen and wet soils, rain and snowfall, sea ice with and without snow, land ice, water and
lakes, over several steps (the first one initialises the point as LSMRUC does at ktau = 1)."""
import argparse

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=2000)
ap.add_argument("--nsl", type=int, default=9)
ap.add_argument("--steps", type=int, default=4)
ap.add_argument("--myj", type=int, default=0)
ap.add_argument("--frpcpn", type=int, default=1)
ap.add_argument("--rdlai2d", type=int, default=0)
ap.add_argument("--modis", type=int, default=0)
ap.add_argument("--dt", type=float, default=60.0)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--out", required=True)
a = ap.parse_args()
r = np.random.default_rng(a.seed)
n, nsl = a.n, a.nsl
iswater, isice, nlu = (17, 15, 20) if a.modis else (16, 24, 27)
zs = [0, .01, .04, .10, .30, .60, 1.0, 1.6, 3.0] if nsl == 9 else [0, .05, .20, .40, 1.60, 3.00]
U = lambda lo, hi: r.uniform(lo, hi, n)
kind = r.choice(6, n, p=[.45, .3, .08, .07, .05, .05])  # 0 land, 1 land+snow, 2 sea ice, 3 sea ice+snow, 4 land ice, 5 water
ivg = r.integers(1, nlu + 1, n).astype(float)
ivg[ivg == iswater] = 1
ivg[ivg == isice] = 2
isl = r.integers(1, 20, n).astype(float)
isl[isl == 14] = 6
xland = np.ones(n); xice = np.zeros(n)
ivg[kind >= 2] = isice; isl[kind >= 2] = 16
xice[(kind == 2) | (kind == 3)] = U(0.6, 1.0)[(kind == 2) | (kind == 3)]
w = kind == 5
xland[w] = 2; ivg[w] = iswater; isl[w] = 14
if not a.modis:
    lake = w & (r.random(n) < .3)
    ivg[lake] = 28
t3d0 = U(240, 300)
cold = (kind >= 2) | (kind == 1)
t3d0[cold] = U(245, 276)[cold]
tsk = t3d0 + U(-6, 6)
tbot = t3d0 + U(-10, 5)
shdmin = U(0, 30); shdmax = shdmin + U(0, 70)
m = r.random(n) < .2
shdmax[m] = shdmin[m] + U(0, .9)[m]  # (shdmax - shdmin) < 1: factor = 1
vegfra = shdmin + (shdmax - shdmin) * U(0, 1)
vegfra[r.random(n) < .1] = 0.5
albbck = U(.1, .3)
snowh = np.zeros(n)
sn = (kind == 1) | (kind == 3)
c = r.random(n)
snowh = np.where(sn, np.where(c < .3, U(.002, .03), np.where(c < .6, U(.03, .2), U(.2, 1.5))), 0.0)
rhos = U(80, 400)
snow = snowh * rhos
snow[sn & (r.random(n) < .05)] = 0  # snow depth without water equivalent
snowc = np.where(snow > 0, U(.3, 1), 0.0)
canwat = np.where(r.random(n) < .5, U(0, .5), 0.0)
snoalb = U(.5, .8); alb = U(.1, .3); emiss = U(.9, .98); lai = U(.3, 5); mavail = U(.2, 1)
soilt1 = np.where(r.random(n) < .5, 0.0, tsk + U(-2, 0))
tsnav = np.where(r.random(n) < .5, 0.0, U(-10, 0))
qvg = np.where(r.random(n) < .5, 0.0, U(.001, .015))
qcg = np.where(r.random(n) < .2, U(0, 1e-3), 0.0)
qsg = np.zeros(n)
hfx = U(-50, 200); qfx = U(-1e-5, 1e-4); lh = qfx * 2.5e6
znt = U(.01, .5)
frac = np.array(zs) / 3.0
tslb = np.array([tsk + (tbot - tsk) * f + r.uniform(-1, 1, n) for f in frac])
smois = np.array([U(.05, .45) for _ in zs])
hdr = [n, nsl, a.steps, a.myj, a.frpcpn, a.rdlai2d, 2 if a.modis else 1, a.dt, 0.5, iswater, isice]
with open(a.out, "wb") as f:
    w8 = lambda x: np.asarray(x, dtype="<f8").tofile(f)
    w8(hdr); w8(zs)
    for x in (ivg, isl, xland, xice, tbot, shdmin, shdmax, albbck, vegfra, snow, snowh, snowc, canwat, snoalb, alb,
              emiss, lai, mavail, tsk, soilt1, tsnav, qvg, qcg, qsg, hfx, qfx, lh, znt):
        w8(x)
    w8(smois); w8(tslb)
    p0, dz, ch0, glw0, gsw0, qr = U(70000, 101000), U(20, 60), U(.002, .04), U(150, 400), U(0, 800), U(.3, 1.05)
    gsw0[r.random(n) < .3] = 0
    for s in range(a.steps):
        t3d = t3d0 + r.uniform(-.5, .5, n)
        p8w = p0 + r.uniform(-20, 20, n)
        es = 611.2 * np.exp(17.67 * (t3d - 273.15) / (t3d - 29.65))
        qv = 0.622 * es / p8w * qr
        qc = np.where(r.random(n) < .3, U(0, 1e-3), 0.0)
        rho = p8w / (287 * t3d * (1 + .608 * qv))
        rain = np.where(r.random(n) < .4, U(0, 2) * a.dt / 60, 0.0)
        rainnc = rain * np.where(r.random(n) < .5, 1.0, U(0, 1))
        sr = np.where(t3d < 275, U(0, 1), np.where(r.random(n) < .1, U(0, .3), 0.0))
        snownc = rainnc * sr * U(0, 1)
        graupnc = rainnc * sr * U(0, .3)
        glw = glw0 + r.uniform(-5, 5, n)
        gsw = gsw0 * r.uniform(.95, 1.05, n)
        ch = ch0 * r.uniform(.9, 1.1, n)
        cp = 1004.5 * (1 + .84 * qv)
        flhc = rho * cp * ch
        flqc = rho * mavail * ch
        chs = ch
        cqs2 = np.where(r.random(n) < .05, 1e-6, U(.002, .04))
        chs2 = np.where(r.random(n) < .05, 1e-6, U(.002, .04))
        psfc = p8w + U(100, 600)
        for x in (rain, rainnc, snownc, graupnc, sr, glw, gsw, flhc, flqc, chs, t3d, qv, qc, p8w, rho, dz, cp, cqs2,
                  chs2, psfc):
            w8(x)
