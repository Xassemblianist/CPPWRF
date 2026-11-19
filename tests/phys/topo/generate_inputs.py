#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the terrain-option column test: a rough terrain (0 to ~3000 m, valleys and ridges) on a rotated
Lambert-like grid at mid latitudes, times around sunrise and sunset (the sun 0.5 to 10 degrees high), and random
points for TOPO_RAD_ADJ_DRVR (night, flat, all-diffuse, shaded and sloping cases).
  generate_inputs.py OUT [--seed S] [--dx DX] [--shadlen L] [--lat LAT] [--rot R]"""
import argparse
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--dx", type=float, default=15000.)
ap.add_argument("--shadlen", type=float, default=25000.)
ap.add_argument("--lat", type=float, default=39.)
ap.add_argument("--rot", type=float, default=0.2)  # rotation of the grid (rad): sina, cosa (cosa < 0 if |rot| > pi/2)
a = ap.parse_args()
r = np.random.default_rng(a.seed)
nx, ny = 48, 36
x, y = np.meshgrid(np.arange(nx), np.arange(ny))
ht = 1500. + 900. * np.sin(x / 3.1) * np.cos(y / 4.3) + 600. * r.standard_normal((ny, nx))
ht = np.maximum(ht, 0.) * (x > 3)  # a sea strip on the west edge (HGT 0)
ht[5:9, 20:30] -= 25.  # depressions below sea level
# below-sea-level bands on every edge: WRF's search reads HT up to 2 points beyond the domain (start_em's copy of the
# edge point there; a zero halo would shade these bands)
ht[-2:, 8:40] = -400.
ht[:2, 8:40] = -400.
ht[6:30, -2:] = -400.
ht[6:30, :2] = -400.
xlat = a.lat + (y - ny / 2) * a.dx / 111e3 + 0.01 * r.standard_normal((ny, nx))
xlong = 35. + (x - nx / 2) * a.dx / 85e3 + 0.01 * r.standard_normal((ny, nx))
rot = a.rot + 0.05 * (x - nx / 2) / nx
sina, cosa = np.sin(rot), np.cos(rot)
times = np.concatenate([np.arange(150., 300., 3.), np.arange(840., 990., 3.), [0., 1430.]])  # around sunrise, sunset
julian = 284.0
n = 4000
p = dict(xlat=a.lat + 10 * r.standard_normal(n), coszen=r.uniform(-0.2, 1.0, n), diff=r.uniform(0, 1, n),
         sw=r.uniform(0, 900, n), hr=r.uniform(-np.pi, np.pi, n), slope=np.abs(r.normal(0, 0.15, n)),
         azi=r.uniform(0, 2 * np.pi, n), shadow=(r.uniform(0, 1, n) < 0.2).astype(float))
p["gsw"] = p["sw"] * r.uniform(0.7, 0.95, n)
p["diff"][::17] = 1.0
p["slope"][::13] = 0.0
p["sw"][::11] = r.uniform(0, 1e-3, len(p["sw"][::11]))
p["coszen"][::19] = r.uniform(0, 1e-4, len(p["coszen"][::19]))
with open(a.out, "wb") as f:
    np.array([nx, ny, a.dx, a.dx, a.shadlen, 0., 10., julian, len(times), n], dtype="<f8").tofile(f)
    times.astype("<f8").tofile(f)
    for v in (ht, xlat, xlong, sina, cosa):
        v.astype("<f8").tofile(f)
    np.array([-0.1291], dtype="<f8").tofile(f)
    for k in ("xlat", "coszen", "diff", "sw", "gsw", "hr", "slope", "azi", "shadow"):
        p[k].astype("<f8").tofile(f)
