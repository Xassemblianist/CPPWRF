#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The quick start's map: sea-level pressure (contours) and 2 m temperature (colours) of the last frame of domain 1,
with the cppwrf package (docs/PYTHON.md).   plot.py RUN_DIR [OUT.png]"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "python"))  # the source tree's python/ or share/cppwrf/python

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import cppwrf  # noqa: E402
from cppwrf import diagnostics as dg  # noqa: E402

run = sys.argv[1] if len(sys.argv) > 1 else "."
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(run, "slp_t2_d01.png")
w = cppwrf.open_wrfout(run, domain=1, xarray=False)
slp, t2 = dg.slp(w)[-1], dg.t2_celsius(w)[-1]
fig, ax = plt.subplots(figsize=(8, 6.5))
c = ax.pcolormesh(w.lon, w.lat, t2, cmap="RdYlBu_r", shading="auto")
fig.colorbar(c, ax=ax, label="2 m temperature (°C)", shrink=0.85)
cs = ax.contour(w.lon, w.lat, slp, levels=range(940, 1060, 2), colors="k", linewidths=0.7)
ax.clabel(cs, fmt="%d", fontsize=7)
ax.set_title(f"CPPWRF d01, {w.times[-1]:%Y-%m-%d %H:%M} UTC: sea-level pressure (hPa) and 2 m temperature")
ax.set_xlabel("longitude")
ax.set_ylabel("latitude")
fig.tight_layout()
fig.savefig(out, dpi=120)
print(f"{out}: slp {slp.min():.1f} to {slp.max():.1f} hPa, T2 {t2.min():.1f} to {t2.max():.1f} °C")
