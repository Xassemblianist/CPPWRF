#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""generate_inputs.py --cols N --nz K --itimestep T --out FILE [--seed S]: random columns for the QNSE surface layer
and PBL (land and sea points, stable / neutral / convective profiles, cloud water, TKE from the minimum to a few
m2 s-2, terrain up to 2.5 km, both hemispheres) in the stream layout of driver_qnse.f90 / driver_qnse.cpp:
int32 n, nz, itimestep, stepbl; float64 dt; 3D arrays (i fastest, nz levels) dz, pmid, th, t, exner, qv, qc, u, v,
tke, then pint (nz + 1 levels); 2D arrays in the order of NAMES2."""
import argparse
import numpy as np

NAMES2 = ["ht", "tsk", "xland", "z0base", "mavail", "znt", "ust", "qsfc", "thz0", "qz0", "uz0", "vz0", "akhs",
          "akms", "hfx", "qfx", "corf", "chklowq", "sice", "snow", "ct"]
p = argparse.ArgumentParser()
p.add_argument("--cols", type=int, default=2000)
p.add_argument("--nz", type=int, default=44)
p.add_argument("--itimestep", type=int, default=3)
p.add_argument("--dt", type=float, default=60.0)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--out", required=True)
a = p.parse_args()
rng = np.random.default_rng(a.seed)
n, nz = a.cols, a.nz
rd, cp, g = 287.0, 7.0 * 287.0 / 2.0, 9.81

dz = np.empty((nz, n))
dz0 = rng.uniform(20.0, 70.0, n)
for k in range(nz):
    dz[k] = dz0 * (1.0 + 0.1 * k) * rng.uniform(0.97, 1.03, n)
z = np.cumsum(dz, axis=0) - 0.5 * dz
ht = np.where(rng.uniform(size=n) < 0.3, 0.0, rng.uniform(0.0, 2500.0, n))
xland = np.where(rng.uniform(size=n) < 0.35, 2.0, 1.0)
ht[xland == 2.0] = 0.0
zi = rng.uniform(100.0, 2500.0, n)
lapse = rng.uniform(0.003, 0.01, n)
th0 = rng.uniform(280.0, 305.0, n)
kind = rng.integers(0, 4, n)  # 0 convective, 1 stable, 2 neutral, 3 noisy
th = np.empty((nz, n))
for k in range(nz):
    t_ = th0 + lapse * np.maximum(z[k] - zi, 0.0) + 2.0 * (z[k] > zi)
    t_ = np.where(kind == 1, th0 + 0.02 * np.minimum(z[k], 300.0) + lapse * z[k], t_)
    t_ = np.where(kind == 2, th0 + 0.0 * z[k], t_)
    t_ = np.where(kind == 3, th0 + lapse * z[k] + rng.normal(0.0, 0.4, n), t_)
    t_ = np.where(kind == 0, t_ - 0.8 * np.exp(-z[k] / 50.0), t_)
    th[k] = t_
psfc = 101325.0 * np.exp(-ht / 8000.0) * rng.uniform(0.98, 1.01, n)
zf = np.vstack([np.zeros(n), np.cumsum(dz, axis=0)])
pint = psfc[None, :] * np.exp(-zf / 8000.0)
pmid = 0.5 * (pint[:-1] + pint[1:])
exner = (pmid / 1.0e5) ** (rd / cp)
t = th * exner
qv = np.maximum(0.014 * np.exp(-z / 2500.0) * rng.uniform(0.2, 1.1, n)[None, :], 1e-6)
qc = np.where(rng.uniform(size=(nz, n)) < 0.15, rng.uniform(0.0, 6e-4, (nz, n)), 0.0)
u = rng.normal(5.0, 4.0, n)[None, :] + np.log1p(z / 10.0) * rng.uniform(0.3, 2.0, n)[None, :] + \
    rng.normal(0.0, 0.5, (nz, n))
v = rng.normal(0.0, 4.0, n)[None, :] + rng.normal(0.0, 0.5, (nz, n))
u[:, :6] = 0.0  # calm columns (EPSU2, EPSGM)
v[:, :6] = 0.0
tke = np.where(rng.uniform(size=(nz, n)) < 0.25, 0.005, rng.lognormal(-1.5, 1.5, (nz, n)))
tke[:, 6:12] = 0.005  # first-step columns: TKE at its minimum (EPSQ2L / 2)
tsk = th[0] * (psfc / 1.0e5) ** (rd / cp) + rng.normal(0.0, 3.0, n)
z0base = np.where(xland == 2.0, 1e-4, rng.uniform(0.01, 1.0, n))
znt = z0base * rng.uniform(1.0, 1.2, n)
ust = rng.uniform(0.02, 1.2, n)
ust[xland == 2.0] = rng.uniform(0.02, 0.9, int((xland == 2.0).sum()))  # all three sea branches
mavail = np.where(xland == 2.0, 1.0, rng.uniform(0.1, 1.0, n))
qsfc = qv[0] * rng.uniform(0.9, 1.3, n)
thz0 = (tsk * (1.0e5 / psfc) ** (rd / cp)) * rng.uniform(0.999, 1.001, n)
qz0 = qsfc * rng.uniform(0.95, 1.05, n)
uz0 = np.where(xland == 2.0, rng.uniform(-0.2, 0.2, n), 0.0)
vz0 = np.where(xland == 2.0, rng.uniform(-0.2, 0.2, n), 0.0)
akhs = rng.uniform(0.002, 0.05, n)
akms = rng.uniform(0.002, 0.08, n)
hfx = rng.normal(60.0, 120.0, n)
hfx[kind == 1] = -np.abs(hfx[kind == 1]) * 0.3
qfx = rng.normal(5e-5, 6e-5, n)
corf = rng.uniform(-1.4e-4, 1.4e-4, n)
chklowq = np.where(rng.uniform(size=n) < 0.2, 0.0, 1.0)
sice = np.where(rng.uniform(size=n) < 0.1, 1.0, 0.0)
snow = np.where(rng.uniform(size=n) < 0.15, rng.uniform(1.0, 50.0, n), 0.0)
ct = np.where(rng.uniform(size=n) < 0.3, rng.uniform(-0.5, 0.5, n), 0.0)

with open(a.out, "wb") as f:
    np.array([n, nz, a.itimestep, 1], dtype="<i4").tofile(f)
    np.array([a.dt], dtype="<f8").tofile(f)
    for x in (dz, pmid, th, t, exner, qv, qc, u, v, tke, pint):
        np.ascontiguousarray(x, dtype="<f8").tofile(f)
    loc = dict(ht=ht, tsk=tsk, xland=xland, z0base=z0base, mavail=mavail, znt=znt, ust=ust, qsfc=qsfc, thz0=thz0,
               qz0=qz0, uz0=uz0, vz0=vz0, akhs=akhs, akms=akms, hfx=hfx, qfx=qfx, corf=corf, chklowq=chklowq,
               sice=sice, snow=snow, ct=ct)
    for name in NAMES2:
        np.ascontiguousarray(loc[name], dtype="<f8").tofile(f)
