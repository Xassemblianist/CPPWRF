# SPDX-License-Identifier: GPL-3.0-or-later
"""Vortex-following moving nests (src/cases/nest_move.cpp, src/nest/vortex_center.hpp) against WRF v4.8.0 dmpar (one
MPI rank) built with vortex following (configure nesting 3, -DMOVE_NESTS -DVORTEX_CENTER; references
tests/verify/refs/vortexnest_refs.txt, data vortexnest_data.tar), at tolerance 0 on both domains:
- VX1: N3 (d02 at (40, 30) of d01, ratio 3, the P4 physics) with the Registry defaults (vortex_interval 15,
  max_vortex_speed 40, track_level 50000, corral_dist 8): d02 follows the lowest 500 hPa height (its north-west
  corner) by one parent cell per parent step; 3 parent steps with history every step (the moves at 1 and 2 min) and
  1 h, on the CPU and in CUDA FP64; in FP32 against WRF real*4's run (CPU, and on the GPU as the derived cuda_sp_id
  case);
- VX2: vortex_interval 1, max_vortex_speed 20, track_level 70000, time_to_move 2, corral_dist 28 for d02 (the search
  box around the last centre every minute, no move before 2 min, the corral cancelling a move toward d01's edge):
  10 min, history every minute.
A resumed run with moving nests is refused by CPPWRF (no restart cases).
The option is "unverified" (data/option_tiers.txt) until these cases have passed; they set cppwrf.allow_unverified.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_DOMS = ("d01", "d02")
_VARS = ("U", "V", "W", "PH", "MU", "THM", "QV", "QCLOUD", "QRAIN", "HGT", "XLAT", "XLONG")
_TOL = {f"{d}.{x}": 0.0 for d in _DOMS for x in _VARS}
_ALLOW = ["cppwrf.allow_unverified=true"]
_VX1 = ["domains.vortex_following=true"] + _ALLOW
_VX2 = ["domains.vortex_following=true", "d02.vortex_interval=1", "d02.max_vortex_speed=20",
        "domains.track_level=70000", "d02.time_to_move=2", "d02.corral_dist=28"] + _ALLOW


def _steps(run, moves):
    return ["real.dir=" + f"{WR}/{run}/steps", "phys.table_dir=" + f"{WR}/{run}/steps"] + moves


def _run(run, moves, extra=()):
    return ["real.dir=" + f"{WR}/{run}", "phys.table_dir=" + f"{WR}/{run}"] + list(extra) + moves


def _refs(path, date=_D):
    return {d: f"{path}/wrfout_{d}_{date}" for d in _DOMS}


CASES = [
    *[(f"real_vx1_r8_fp64_{b}", f"{b}_dp", "configs/real_n3_steps.ini", _steps("run_VX1_r8", _VX1),
       _refs(f"{WR}/run_VX1_r8/steps"), 180, _TOL) for b in ("cpu", "cuda")],
    *[(f"real_vx1_r8_fp64_{b}_1h", f"{b}_dp", "configs/real_n3.ini", _run("run_VX1_r8", _VX1),
       _refs(f"{WR}/run_VX1_r8"), 3600, _TOL) for b in ("cpu", "cuda")],
    # 10 min with history every minute on both domains (the steps config's output interval: one parent step)
    *[(f"real_vx2_r8_fp64_{b}", f"{b}_dp", "configs/real_n3_steps.ini", _run("run_VX2_r8", _VX2),
       _refs(f"{WR}/run_VX2_r8"), 600, _TOL) for b in ("cpu", "cuda")],
    ("real_vx1_fp32_cpu", "cpu_sp", "configs/real_n3_steps.ini", _steps("run_VX1_cpu_sp", _VX1),
     _refs(f"{WR}/run_VX1_cpu_sp/steps"), 180, _TOL),
    ("real_vx1_fp32_cpu_1h", "cpu_sp", "configs/real_n3.ini", _run("run_VX1_cpu_sp", _VX1),
     _refs(f"{WR}/run_VX1_cpu_sp"), 3600, _TOL),
]

# no STATUS rows yet: docs/STATUS.md's table lists verified cases; VX1 / VX2 rows come with the proof (the option's
# row of data/option_tiers.txt removed)

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order
DATA = {c[0]: 'vortexnest_data.tar' for c in CASES}
