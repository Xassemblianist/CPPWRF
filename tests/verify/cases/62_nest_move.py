# SPDX-License-Identifier: GPL-3.0-or-later
"""Moving nests with prescribed moves (src/cases/nest_move.cpp) against WRF v4.8.0 dmpar (one MPI rank) built with
"preset moves" (configure nesting 2, -DMOVE_NESTS; tools/colab/colab_movenest.sh, references
tests/verify/refs/movenest_refs.txt, data movenest_data.tar), at tolerance 0 on both domains:
- MV1: N3 (d02 at (40, 30) of d01, ratio 3, the P4 physics) with d02 moved by one parent cell at 1 min (+1, 0),
  2 min (0, +1), 20 min (-1, -1) and 40 min (+1, -1): 3 parent steps with history every step (the first two moves)
  and 1 h, on the CPU and in CUDA FP64; in FP32 against WRF real*4's run of the same moves (CPU, and on the GPU as the
  derived cuda_sp_id case);
- MV2: the corral (d02's corral_dist 39): the x part of the move at 1 min and the whole move at 2 min are cancelled
  ("MOAD can not move").
A resumed run with moving nests is refused by CPPWRF (no restart cases).
The option is "unverified" (data/option_tiers.txt) until these cases have passed; they set cppwrf.allow_unverified.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_DOMS = ("d01", "d02")
_VARS = ("U", "V", "W", "PH", "MU", "THM", "QV", "QCLOUD", "QRAIN", "HGT", "XLAT", "XLONG")
_TOL = {f"{d}.{x}": 0.0 for d in _DOMS for x in _VARS}
_ALLOW = ["cppwrf.allow_unverified=true"]
_MV1 = ["domains.num_moves=4", "domains.move_id=2,2,2,2", "domains.move_interval=1,2,20,40",
        "domains.move_cd_x=1,0,-1,1", "domains.move_cd_y=0,1,-1,-1"] + _ALLOW
_MV2 = ["domains.num_moves=2", "domains.move_id=2,2", "domains.move_interval=1,2", "domains.move_cd_x=-1,1",
        "domains.move_cd_y=1,0", "d02.corral_dist=39"] + _ALLOW


def _steps(run, moves):
    return ["real.dir=" + f"{WR}/{run}/steps", "phys.table_dir=" + f"{WR}/{run}/steps"] + moves


def _hour(run, moves):
    return ["real.dir=" + f"{WR}/{run}", "phys.table_dir=" + f"{WR}/{run}"] + moves


def _refs(path, date=_D):
    return {d: f"{path}/wrfout_{d}_{date}" for d in _DOMS}


CASES = [
    *[(f"real_mv1_r8_fp64_{b}", f"{b}_dp", "configs/real_n3_steps.ini", _steps("run_MV1_r8", _MV1),
       _refs(f"{WR}/run_MV1_r8/steps"), 180, _TOL) for b in ("cpu", "cuda")],
    *[(f"real_mv1_r8_fp64_{b}_1h", f"{b}_dp", "configs/real_n3.ini", _hour("run_MV1_r8", _MV1),
       _refs(f"{WR}/run_MV1_r8"), 3600, _TOL) for b in ("cpu", "cuda")],
    *[(f"real_mv2_r8_fp64_{b}", f"{b}_dp", "configs/real_n3_steps.ini", _steps("run_MV2_r8", _MV2),
       _refs(f"{WR}/run_MV2_r8/steps"), 180, _TOL) for b in ("cpu", "cuda")],
    ("real_mv1_fp32_cpu", "cpu_sp", "configs/real_n3_steps.ini", _steps("run_MV1_cpu_sp", _MV1),
     _refs(f"{WR}/run_MV1_cpu_sp/steps"), 180, _TOL),
    ("real_mv1_fp32_cpu_1h", "cpu_sp", "configs/real_n3.ini", _hour("run_MV1_cpu_sp", _MV1),
     _refs(f"{WR}/run_MV1_cpu_sp"), 3600, _TOL),
]

# no STATUS rows yet: docs/STATUS.md's table lists verified cases; MV1 / MV2 rows come with the proof (the option's
# row of data/option_tiers.txt removed)

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order
DATA = {c[0]: 'movenest_data.tar' for c in CASES}
