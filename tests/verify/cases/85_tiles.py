# SPDX-License-Identifier: GPL-3.0-or-later
"""Tiles (run.tiles, docs/USER_GUIDE.md, docs/MPI.md): cases of the other groups run as several tiles of one domain,
compared with the same WRF references at tolerance 0 (the tiles' output must be that of the run without tiles).
MPI ranks use the same decomposition: tools/colab/colab_mpi.sh runs any case under mpirun with several layouts.

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
    # start_em's slopes and ctopo from the whole domain's HGT; TOPO_RAD_ADJ point by point (topo_shading: one patch)
    ("real_tw1_r8_fp64_cpu", "2x2"),
    ("real_sr3_r8_fp64_cpu_5h", "3x1"),
    # two-way nests: each domain on its own patches, whole copies between them (1x5: the couplings of the nest
    # forcing read mu_2 across the patches' edges), land-ice points (Noah's carried RC), Tiedtke's row flag
    ("real_n3_r8_fp64_cpu", "1x5"),
    ("real_n3_r8_fp64_cpu", "2x2"),
    ("real_d0v_r8_fp64_cpu", "3x2"),
    ("real_d0_r8_fp64_cpu_6h", "4x2"),  # Tiedtke's row flag across the patches of a row (it matters after 1 - 2 h)
    # Grell-Freitas with cu_rad_feedback: convtrans_prep's averaging counter (one per tile thread) at the end of the
    # first 30-minute period, QC_CU / QI_CU into RRTMG
    ("real_a05g_r8_fp64_cpu_1h", "2x2"),
    ("ideal_quarter_ss_k2_fp64_cpu", "2x2"),
    ("ideal_squall2d_x_fp64_cpu", "3x1"),
    ("ideal_squall2d_y_fp64_cpu", "1x3"),
    ("real_p4_r8_fp64_cuda_1h", "2x2"),
    ("real_mynnx1_r8_fp64_cuda_1h", "2x1"),
    ("ideal_quarter_ss_k2_fp64_cuda", "2x2"),
    ("ideal_squall2d_x_fp64_cuda", "2x1"),
    # P4's restart chain (tests/verify/cases/66_restart_chain.py) as 2 x 2 tiles: straight and resumed from the wrfrst
    # file and from CPPWRF's own restart file (its halo points HALO_<name> go to the patches that hold them)
    ("real_p4_r8_fp64_cpu_chain", "2x2"),
]

CASES = [(f"{n}_t{lay}", c[1], c[2], list(c[3]) + [f"run.tiles={lay}"], c[4], c[5], dict(c[6]))
         for n, lay in _TILED for c in [_BASE[n]]]

# CUDA FP64 twins of the CPU-only tiled cases above (the decomposed device paths: patch clipping, whole-field
# gathers of nests, Tiedtke's row flag, the land-ice Noah carry, FDDA, terrain options, Morrison, Grell-Freitas's
# averaging): the CPU case's configuration, reference, time and tolerances with the cuda_dp backend
_TILED_CUDA = [(n, lay) for n, lay in _TILED if n.startswith("real_") and _BASE[n][1] == "cpu_dp"
               and n not in ("real_p4_r8_fp64_cpu", "real_mynnx1_r8_fp64_cpu")  # these two have CUDA tiled cases
               and not n.endswith("_chain")]  # the CUDA restart chains run as one patch
_CUDA_NAME = {n: n.replace("_fp64_cpu", "_fp64_cuda") for n, _ in _TILED_CUDA}
CASES += [(f"{_CUDA_NAME[n]}_t{lay}", "cuda_dp", c[2], list(c[3]) + [f"run.tiles={lay}"], c[4], c[5], dict(c[6]))
          for n, lay in _TILED_CUDA for c in [_BASE[n]]]

# CPU tiles with CPPWRF_TILE_CHECK=1 (suite.tile_check: every launch of every step of the tiles against the check's
# run with one patch, then the output against WRF): (name, case NAME, LAYOUT, TIME): NAME's configuration for TIME s.
# The drivers' host copies of a decomposed run stand for the one-patch run's launches (Noah's RC at the land-ice
# points: P8 and D0V; Tiedtke's row flag: D0V)
_CHECKED = [("real_p8_r8_fp64_cpu_t2x2_chk", "real_p8_r8_fp64_cpu_10min", "2x2", 180),
            ("real_d0v_r8_fp64_cpu_10min_t2x2_chk", "real_d0v_r8_fp64_cpu", "2x2", 600)]
CASES += [(name, c[1], c[2], [x for x in c[3] if not x.startswith("run.run_time=")]
           + [f"run.run_time={t:g}", f"run.tiles={lay}", "suite.tile_check"], c[4], t, dict(c[6]))
          for name, n, lay, t in _CHECKED for c in [_BASE[n]]]

# Cases for the layout comparisons of tools/colab/colab_mpi.sh (every output file of a layout against one rank's):
# NAME_wrst is NAME with wrfrst files written, which hold fields no wrfout file has (RS at the land-ice points of
# the D0V domain: Noah's RC carried from the last SFLX point, across the patches in a decomposed run).
_WRST = [("real_d0v_r8_fp64_cpu", ["restart.interval=10"])]
CASES += [(f"{n}_wrst", c[1], c[2], list(c[3]) + sets, c[4], c[5], dict(c[6])) for n, sets in _WRST for c in [_BASE[n]]]

# The restart files do not depend on the decomposition (suite.layout_restart=LAYOUT,INTERVAL:
# wrf_suite.layout_restart): the configuration of the case NAME (a restart chain: without suite.chain) runs as one
# patch and as LAYOUT tiles with restart files every INTERVAL s, and CPPWRF's own restart_<step>.nc files
# (HALO_<name>: the halo points beyond the domain's edges, gathered from the edge patches) and the wrfrst files of the
# two runs must be identical, every variable, dimension and attribute. NAME_rst_tLAYOUT (NAME without _chain).
# quarter_ss: the halo points of periodic directions (the wrap across the patches).
_LAYOUT_RST = [("real_p4_r8_fp64_cpu_chain", "2x2", 180), ("ideal_quarter_ss_k2_fp64_cpu", "2x2", 120)]
_RST_NAME = {n: f"{n.removesuffix('_chain')}_rst_t{lay}" for n, lay, _ in _LAYOUT_RST}
CASES += [(_RST_NAME[n], c[1], c[2], [x for x in c[3] if not x.startswith("suite.chain=")]
           + [f"suite.layout_restart={lay},{iv:g}"], c[4], c[5], dict(c[6]))
          for n, lay, iv in _LAYOUT_RST for c in [_BASE[n]]]

# the data tars (tests/verify/vm_data.txt) each case reads: those of the case it tiles
DATA = {f"{n}_t{lay}": _BASE_DATA[n] for n, lay in _TILED}
DATA.update({f"{_CUDA_NAME[n]}_t{lay}": _BASE_DATA[n] for n, lay in _TILED_CUDA})
DATA.update({f"{n}_wrst": _BASE_DATA[n] for n, _ in _WRST})
DATA.update({_RST_NAME[n]: _BASE_DATA[n] for n, _, _ in _LAYOUT_RST})
DATA.update({name: _BASE_DATA[n] for name, n, _, _ in _CHECKED})
# the GPU runs take run_KESSKF_r8 from kesskf_data.tar (v05_files_data.tar's absolute links are for the CPU suite)
DATA["real_kesskf_r8_fp64_cuda_t3x1"] = "kesskf_data.tar,v05_iofields.tar"
