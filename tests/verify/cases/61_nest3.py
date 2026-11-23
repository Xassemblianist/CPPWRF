# SPDX-License-Identifier: GPL-3.0-or-later
"""Three domains, a nest of a nest: N3T (configs/real_n3t_r8.ini: d03 at 1.67 km inside d02 at 5 km inside d01 at
15 km, ratio 3 each, two-way feedback with smooth_option 2, the P4 physics on every domain) against WRF's run N3T_r8
(real*8; WPS 4.6.0 geogrid / metgrid and real.exe of the three domains, tests/verify/refs/nest3_refs.txt) at
tolerance 0 on every domain: 3 steps with history every d01 step (run_N3T_r8/steps) and 1 h with history every
10 min, on the CPU and the GPU. d03's step is 60/9 s (WRF's clock: 6 + 2/3 s); d02's feedback into d01 follows d03's
into d02 at each d01 step.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_DOMS = ("d01", "d02", "d03")
_VARS = ("U", "W", "THM", "QV", "QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")

CASES = [
    ("real_n3t_r8_fp64_cpu", "cpu_dp", "configs/real_n3t_r8.ini",
     ["real.dir=" + f"{WR}/run_N3T_r8/steps", "phys.table_dir=" + f"{WR}/run_N3T_r8/steps", "run.run_time=180",
      "output.out_interval=1", "output.diag_interval=1", "d02.out_interval=3", "d02.diag_interval=3",
      "d03.out_interval=9", "d03.diag_interval=9"],
     {d: f"{WR}/run_N3T_r8/steps/wrfout_{d}_{_D}" for d in _DOMS}, 180,
     {f"{d}.{x}": 0.0 for d in _DOMS for x in _VARS}),
    *[(f"real_n3t_r8_fp64_{b}_1h", f"{b}_dp", "configs/real_n3t_r8.ini", [],
       {d: f"{WR}/run_N3T_r8/wrfout_{d}_{_D}" for d in _DOMS}, 3600,
       {f"{d}.{x}": 0.0 for d in _DOMS for x in _VARS}) for b in ("cpu", "cuda")],
]

STATUS = [
    ('N3T', 'three domains, d03 (1.67 km) inside d02 (5 km) inside d01 (15 km), two-way feedback, smooth_option 2, '
     'P4 physics on every domain: every domain bit-identical with WRF (3 steps and 1 h)', '1 h'),
]

DATA = {
    'real_n3t_r8_fp64_cpu': 'nest3_data.tar',
    'real_n3t_r8_fp64_cpu_1h': 'nest3_data.tar',
    'real_n3t_r8_fp64_cuda_1h': 'nest3_data.tar',
}
