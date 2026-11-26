#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the vortex-following nest test (driver_vortex.f90 / driver_vortex.cpp): a header of 8 float64 (nx, ny,
nz, track_level, vortex_interval, max_vortex_speed, parent_grid_ratio, dx), then two field sets A and B, each ph,
phb (nx, nz + 1, ny), p, pb (nx, nz, ny), psfc, ht (nx, ny) in Fortran order, float64.

Scenarios (--scenario):
  vortex    a low of the track_level height inside the domain (A), moved by 2, 1 points in B: the interior
            centroid, the search box around the last centre;
  edge      a height gradient only (the lowest point on the domain's edge, as in the N3 domain): the smoothing
            window clipped at 1 and ide - 2;
  mountain  track_level 85000 Pa with terrain whose lowest level is above it (interp_p's 9.81e5), equal heights in
            a quantised field (ties: the first point in WRF's loop order);
  flat      one height everywhere in A (pbar = 0: xi, xj stay, vc stays -1, then the box around vc moved by decide);
  box       vortex_interval 1, max_vortex_speed 20, dx 3000, ratio 2, track_level 70000: a one-point search box;
  trunc     pressures with fractions next to track_level (interp_p's INTEGER pk: 50000.7 stops the search at that
            level)."""
import argparse
import struct

import numpy as np

G = 9.81


def column_fields(nx, ny, nz, rng, ps, ter, dz_pert):
    """ph, phb, p, pb for surface pressure ps(j, i), terrain ter(j, i), geopotential perturbation dz_pert(j, i) (m,
    on every full level above the ground)."""
    eta = np.linspace(1.0, 0.0, nz + 1)  # full levels
    ptop = 5000.0
    pfull = ptop + eta[None, None, :] * (ps[:, :, None] - ptop)  # (ny, nx, nz + 1)
    pmass = 0.5 * (pfull[..., :-1] + pfull[..., 1:])
    # hydrostatic heights with T = 288 - 6.5 K/km, roughly
    H = 287.0 * 260.0 / G
    zfull = ter[:, :, None] + H * np.log(ps[:, :, None] / pfull)
    zfull[..., 1:] += dz_pert[:, :, None] * np.minimum(1.0, np.arange(1, nz + 1)[None, None, :] / 4.0)
    phi = G * zfull
    pb = pmass * 0.97 + rng.normal(0, 0.5, pmass.shape)
    p = pmass - pb
    phb = phi * 0.99 + rng.normal(0, 0.05, phi.shape)
    ph = phi - phb
    return ph, phb, p, pb


def fortran(a):  # (ny, nx, nk) -> Fortran (i, k, j) order bytes
    return np.ascontiguousarray(np.transpose(a, (0, 2, 1))).astype("<f8").tobytes()


def main():
    cli = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument("out")
    cli.add_argument("--scenario", required=True,
                     choices=("vortex", "edge", "mountain", "flat", "box", "trunc"))
    cli.add_argument("--seed", type=int, default=1)
    a = cli.parse_args()
    rng = np.random.default_rng(a.seed)
    nx, ny, nz, tl, vi, mvs, pgr, dx = 60, 51, 20, 50000, 15, 40, 3, 5000.0
    if a.scenario == "box":
        nx, ny, tl, vi, mvs, pgr, dx = 48, 40, 70000, 1, 20, 2, 3000.0
    if a.scenario == "mountain":
        tl = 85000
    jj, ii = np.mgrid[1:ny + 1, 1:nx + 1].astype(float)
    sets = []
    for s in range(2):
        ter = np.clip(rng.normal(150, 80, (ny, nx)), 0, None)
        ps = 101000.0 - 11.5 * ter + rng.normal(0, 30, (ny, nx))
        dz = rng.normal(0, 0.3, (ny, nx))
        if a.scenario in ("vortex", "box", "trunc"):
            ic, jc = (40 + 2 * s, 30 + s) if a.scenario != "box" else (30 + s, 18 - s)
            low = np.exp(-((ii - ic) ** 2 + (jj - jc) ** 2) / (2 * 6.0 ** 2))
            dz -= 80.0 * low
            ps -= 2500.0 * low
        elif a.scenario == "edge":
            dz += 0.9 * (ii - nx) - 0.6 * jj
        elif a.scenario == "mountain":
            ter += 1800.0 * np.exp(-((ii - 20) ** 2 + (jj - 25) ** 2) / (2 * 5.0 ** 2))
            ps = 101000.0 - 11.5 * ter
            dz += 0.5 * jj
        if a.scenario == "flat" and s == 0:
            dz[:] = 0.0
        ph, phb, p, pb = column_fields(nx, ny, nz, rng, ps, ter, dz)
        if a.scenario == "flat" and s == 0:
            # one column everywhere: every height equal
            for arr in (ph, phb, p, pb):
                arr[:] = arr[0, 0]
        if a.scenario == "mountain":
            # quantised geopotential: equal heights at many points (ties)
            ph = np.round((ph + phb) / 50.0) * 50.0 - phb
        if a.scenario == "trunc":
            # level pressures p + pb with fractions around track_level
            for j in range(ny):
                for i in range(nx):
                    tot = p[j, i] + pb[j, i]
                    k = int(np.argmin(np.abs(tot - tl)))
                    if k > 0:
                        frac = (0.7, 0.0, 0.3, 0.999)[(i + j) % 4]
                        p[j, i, k] = (tl + frac) - pb[j, i, k]
        psfc = ps
        sets.append((ph, phb, p, pb, psfc, ter))
    with open(a.out, "wb") as f:
        f.write(struct.pack("<8d", nx, ny, nz, tl, vi, mvs, pgr, dx))
        for ph, phb, p, pb, psfc, ter in sets:
            for arr in (ph, phb, p, pb):
                f.write(fortran(arr))
            for arr in (psfc, ter):
                f.write(np.ascontiguousarray(arr).astype("<f8").tobytes())


if __name__ == "__main__":
    main()
