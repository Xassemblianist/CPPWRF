# SPDX-License-Identifier: GPL-3.0-or-later
"""Tiles (run.tiles, docs/USER_GUIDE.md): cases of the other groups run as several tiles of one domain, compared
with the same WRF references at tolerance 0 (the tiles' output must be that of the run without tiles).

CASES: see tests/verify/cases/__init__.py. Each case here is NAME_tLAYOUT: the case NAME of its group with
run.tiles = LAYOUT added to its --set list (same configuration, reference, time, tolerances and data)."""
import glob
import importlib.util
import os

_DIR = os.path.dirname(os.path.abspath(__file__))


def _groups():
    for p in sorted(glob.glob(os.path.join(_DIR, "[0-9][0-9]_*.py"))):
        if os.path.basename(p) == os.path.basename(__file__):
            continue
        spec = importlib.util.spec_from_file_location(f"cases._tiles_{os.path.basename(p)[:-3]}", p)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        yield m


_G = list(_groups())
_BASE = {c[0]: c for g in _G for c in g.CASES}
_BASE_DATA = {n: t for g in _G for n, t in getattr(g, "DATA", {}).items()}

# (case, layout): the dynamics with specified and periodic boundaries, the physics suites whose drivers gather or
# exchange between the tiles (radiation by rows, MYNN's advected qke_adv, Noah, cumulus), the 2-D domains
_TILED = [
    ("real_r1_fp64_cpu", "2x2"),
    ("real_p4_r8_fp64_cpu", "2x2"),
    ("real_p6_r8_fp64_cpu", "1x3"),
    ("real_kesskf_r8_fp64_cpu", "3x1"),
    ("real_fd2_r8_fp64_cpu", "2x2"),
    ("real_mynnx1_r8_fp64_cpu", "3x1"),
    ("real_morr_r8_fp64_cpu", "2x2"),
    ("real_morr_r8_fp64_cpu_1h", "3x1"),
    ("real_skv_r8_fp64_cpu", "2x2"),
    ("real_skv_r8_fp64_cpu_1h", "1x3"),
    ("ideal_quarter_ss_k2_fp64_cpu", "2x2"),
    ("ideal_squall2d_x_fp64_cpu", "3x1"),
    ("ideal_squall2d_y_fp64_cpu", "1x3"),
    ("real_p4_r8_fp64_cuda_1h", "2x2"),
    ("real_mynnx1_r8_fp64_cuda_1h", "2x1"),
    ("ideal_quarter_ss_k2_fp64_cuda", "2x2"),
    ("ideal_squall2d_x_fp64_cuda", "2x1"),
]

CASES = [(f"{n}_t{lay}", c[1], c[2], list(c[3]) + [f"run.tiles={lay}"], c[4], c[5], dict(c[6]))
         for n, lay in _TILED for c in [_BASE[n]]]

# the data tars (tests/verify/vm_data.txt) each case reads: those of the case it tiles
DATA = {f"{n}_t{lay}": _BASE_DATA[n] for n, lay in _TILED}
