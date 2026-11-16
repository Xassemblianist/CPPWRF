#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""generate_inputs.py --nx NX --ny NY --nz K --dt DT --stepbl S --seed S --out FILE: a random tile for the EDKF mass
flux of the QNSE PBL (mfshconv = 1): mixed layers of 0.2 to 3 km under conditionally unstable or stable air, dry to
saturated (clouds from the first level to deep ones), warm and freezing columns, surface heat fluxes of both signs,
RC_MF of a previous step. Stream (little endian): int32 nx, ny, nz, stepbl; float64 dt; 3D arrays (i fastest, then
j, then k; nz levels) dz, rho, pmid, th, exner, qv, qc, u, v, tke, rc_mf; 2D arrays hfx, qfx, ht."""
import argparse
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--nx", type=int, default=61)
p.add_argument("--ny", type=int, default=49)
p.add_argument("--nz", type=int, default=44)
p.add_argument("--dt", type=float, default=60.0)
p.add_argument("--stepbl", type=int, default=1)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--out", required=True)
a = p.parse_args()
rng = np.random.default_rng(a.seed)
n, nz = a.nx * a.ny, a.nz
rd, cp = 287.0, 7.0 * 287.0 / 2.0

dz0 = rng.uniform(25.0, 60.0, n)
grow = rng.uniform(1.04, 1.09, n)
dz = np.array([dz0 * grow ** k * rng.uniform(0.97, 1.03, n) for k in range(nz)])
dz = np.minimum(dz, 900.0)
zf = np.vstack([np.zeros(n), np.cumsum(dz, axis=0)])
z = 0.5 * (zf[:-1] + zf[1:])
psfc = rng.uniform(85000.0, 102500.0, n)
pmid = psfc[None, :] * np.exp(-z / rng.uniform(7800.0, 8600.0, n)[None, :])
exner = (pmid / 1.0e5) ** (rd / cp)
th0 = np.where(rng.uniform(size=n) < 0.2, rng.uniform(262.0, 280.0, n), rng.uniform(284.0, 306.0, n))
zi = rng.uniform(200.0, 3000.0, n)
gam = np.where(rng.uniform(size=n) < 0.5, rng.uniform(0.0003, 0.003, n), rng.uniform(0.003, 0.008, n))
jump = np.where(rng.uniform(size=n) < 0.5, 0.0, rng.uniform(0.0, 2.5, n))
sup = rng.uniform(0.0, 2.5, n)  # superadiabatic surface layer
th = th0 + np.where(z > zi, jump + gam * (z - zi), 0.0) + sup * np.exp(-z / 60.0) + rng.normal(0.0, 0.05, (nz, n))
t = th * exner
es = 611.2 * np.exp(17.67 * (t - 273.15) / (t - 29.65))
qs = 0.622 * es / (pmid - es)
k_zi = np.argmin(np.abs(z - zi), axis=0)
rh = rng.uniform(0.3, 1.08, n)
qml = rh * qs[k_zi, np.arange(n)]
hq = rng.uniform(1200.0, 4000.0, n)
qv = np.where(z <= zi, qml, qml * np.exp(-(z - zi) / hq)) * rng.uniform(0.98, 1.02, (nz, n))
qv = np.clip(qv, 1e-7, 1.03 * qs)
qc = np.where(rng.uniform(size=(nz, n)) < 0.1, rng.uniform(0.0, 5e-4, (nz, n)), 0.0)
rho = pmid / (rd * t * (1.0 + 0.608 * qv))
u = rng.normal(5.0, 5.0, (nz, n))
v = rng.normal(0.0, 5.0, (nz, n))
tke = np.where(rng.uniform(size=(nz, n)) < 0.2, 0.005, rng.lognormal(-1.0, 1.2, (nz, n)) + 0.005)
rc_mf = np.where((rng.uniform(size=n) < 0.4)[None, :] & (z < zi[None, :] + 2500.0),
                 rng.uniform(0.0, 3e-4, (nz, n)), 0.0)
rc_mf = np.minimum(rc_mf, 0.5 * qv)
hfx = rng.normal(150.0, 160.0, n)
qfx = np.where(rng.uniform(size=n) < 0.15, rng.uniform(-3e-5, 0.0, n), rng.uniform(0.0, 3e-4, n))
ht = rng.uniform(0.0, 1500.0, n)

with open(a.out, "wb") as f:
    np.array([a.nx, a.ny, nz, a.stepbl], dtype="<i4").tofile(f)
    np.array([a.dt], dtype="<f8").tofile(f)
    for x in (dz, rho, pmid, th, exner, qv, qc, u, v, tke, rc_mf, hfx, qfx, ht):
        np.ascontiguousarray(x, dtype="<f8").tofile(f)
