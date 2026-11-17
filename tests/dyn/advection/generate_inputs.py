# SPDX-License-Identifier: GPL-3.0-or-later
"""Inputs of the scalar-advection differential test (tests/dyn/advection/run_test.sh): one file per case, the last
RK3 stage of the moist species of a small domain with random fields (signs of the mass fluxes mixed, exact zeros
among them, species with exact zeros and sharp gradients so that the limiters act).

    generate_inputs.py OUT_PREFIX     ->  OUT_PREFIX_<case>.bin for the cases of CASES

Layout (little-endian, stream): int32 nx ny nz bc nspec p_qv adv_opt, float64 dx dy dt; then float64 arrays, each
over the padded index range [-H, n + H) of its staggering (x fastest, then y, then k; H = 5), in this order:
c1h c2h rdnw (nz), fnm fnp (nz + 1), msftx msfty mub mu_1 mu_2 muts (2-D, mass points), ru_m (U), rv_m (V),
ww_m (W), then per species q_1 (WRF moist_old), q_2 (moist), q_tendf (moist_tend). Periodic directions are
periodic over the whole padded range; elsewhere the halo values are random (WRF and CPPWRF must not read them).
bc: 0 specified, 1 periodic, 2 open."""
import struct
import sys

import numpy as np

H = 5
# name, nx, ny, nz, bc, adv_opt (moist_adv_opt of the stage; 3 / 4: the WENO cases, -r8 -double-8 WRF), seed
CASES = [("spec", 14, 12, 10, 0, 2, 1), ("peri", 12, 11, 9, 1, 2, 2), ("open", 13, 12, 10, 2, 2, 3),
         ("spec_b", 17, 15, 12, 0, 2, 4), ("spec_pd", 14, 12, 10, 0, 1, 5), ("peri_pd", 12, 11, 9, 1, 1, 6),
         ("spec_weno", 14, 12, 12, 0, 3, 7), ("peri_weno", 12, 11, 10, 1, 3, 8), ("open_weno", 13, 12, 11, 2, 3, 9),
         ("spec_wenopd", 14, 12, 12, 0, 4, 10), ("peri_wenopd", 12, 11, 10, 1, 4, 11),
         ("open_wenopd", 13, 12, 11, 2, 4, 12)]


def padded(rng, nx, ny, nz, sx, sy, periodic, f):
    """f(shape) -> values on the padded range; periodic: the period nx (ny) of the interior repeated."""
    px, py = nx + sx, ny + sy
    a = f((nz, py + 2 * H, px + 2 * H))
    if periodic:
        ix = (np.arange(px + 2 * H) - H) % nx + H
        iy = (np.arange(py + 2 * H) - H) % ny + H
        a = a[:, iy][:, :, ix]
    return a


def write_case(path, name, nx, ny, nz, bc, adv_opt, seed):
    rng = np.random.default_rng(seed)
    per = bc == 1
    dx, dy, dt = 3000.0, 3000.0, 18.0
    nspec, p_qv = 3, 0
    out = [struct.pack("<7i3d", nx, ny, nz, bc, nspec, p_qv, adv_opt, dx, dy, dt)]

    def put(a):
        out.append(np.ascontiguousarray(a, dtype="<f8").tobytes())

    eta = np.sort(rng.uniform(0.0, 1.0, nz - 1))[::-1]
    znw = np.concatenate([[1.0], eta, [0.0]])
    dnw = znw[1:] - znw[:-1]
    c1h = rng.uniform(0.3, 1.0, nz)
    c2h = rng.uniform(0.0, 5000.0, nz)
    put(c1h), put(c2h), put(1.0 / dnw)
    fnm = np.zeros(nz + 1)
    fnm[1:nz] = rng.uniform(0.3, 0.7, nz - 1)
    fnp = np.zeros(nz + 1)
    fnp[1:nz] = 1.0 - fnm[1:nz]
    put(fnm), put(fnp)

    def f2(lo, hi):
        return padded(rng, nx, ny, 1, 0, 0, per, lambda s: rng.uniform(lo, hi, s))[0]

    put(f2(0.95, 1.05)), put(f2(0.95, 1.05))       # msftx, msfty
    mub = f2(85000.0, 95000.0)
    put(mub), put(f2(-600.0, 600.0)), put(f2(-600.0, 600.0))
    put(mub + f2(-600.0, 600.0))                   # muts

    def flux(sx, sy, scale, nzs):
        def gen(s):
            v = rng.normal(0.0, scale, s)
            v[rng.uniform(size=s) < 0.08] = 0.0    # cr = 0 faces (WRF's else branches)
            return v
        return padded(rng, nx, ny, nzs, sx, sy, per, gen)

    put(flux(1, 0, 9.0e4 * 25.0, nz))              # ru_m = mu u
    put(flux(0, 1, 9.0e4 * 25.0, nz))              # rv_m
    ww = flux(0, 0, 400.0, nz + 1)
    ww[0], ww[nz] = 0.0, 0.0
    put(ww)

    def species(kind):
        def gen(s):
            if kind == 0:
                return 0.012 * (1.0 + 0.4 * rng.uniform(-1, 1, s))
            v = rng.lognormal(-9.0, 1.5, s)
            v[rng.uniform(size=s) < (0.5 if kind == 1 else 0.75)] = 0.0
            return v
        return gen

    for kind in range(nspec):
        q1 = padded(rng, nx, ny, nz, 0, 0, per, species(kind))
        q2 = np.maximum(q1 * (1.0 + 0.2 * padded(rng, nx, ny, nz, 0, 0, per, lambda s: rng.uniform(-1, 1, s))), 0.0)
        tend = padded(rng, nx, ny, nz, 0, 0, per, lambda s: rng.normal(0.0, 1.0, s)) * 9.0e4 * 1e-7
        put(q1), put(q2), put(tend)
    with open(path, "wb") as fh:
        fh.write(b"".join(out))


def main():
    prefix = sys.argv[1]
    for c in CASES:
        write_case(f"{prefix}_{c[0]}.bin", *c)
        print(f"{prefix}_{c[0]}.bin")


if __name__ == "__main__":
    main()
