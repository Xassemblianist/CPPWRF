# SPDX-License-Identifier: GPL-3.0-or-later
"""CLM lake model (sf_lake_physics = 1, phys/module_sf_lake.F) with Noah / Noah-MP, use_lakedepth = 0 (the inputs
carry no lake depth: LAKE_DEPTH_FLAG = 0): LK1 = P4 (WSM6, Kain-Fritsch, RRTMG, YSU, Noah; 1715 lake points of
LAKEMASK, LAKEFLAG = 1) with lakedepth_default 50 m, LK2 = LK1 with 10 m, LK3 = LK1 with 0 (lakeini's spval
column), LKV = V2 (Svalbard, polar night, cold lakes), LKN = NMP1 (Noah-MP); each against its WRF v4.8.0 real*8 run
(tests/verify/refs/b13_refs.txt; data port_b13_data.tar). LK1 also has the restart check restart_lk1 (restarts.py).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "TSK": 0.0, "HFX": 0.0, "LH": 0.0,
      "T2": 0.0, "Q2": 0.0, "T_LAKE3D": 0.0, "T_SOISNO3D": 0.0, "LAKE_ICEFRAC3D": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n, _s in (("LK1", "lk1"), ("LK2", "lk2"), ("LK3", "lk3"), ("LKV", "lkv"), ("LKN", "lkn")):
    _cfg, _run = f"configs/real_{_s}_r8.ini", f"{WR}/run_{_n}_r8"
    CASES += [
        # 3 steps (history every step) and 1 h, CPU and CUDA FP64, tolerance 0
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_s}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _T),
        (f"real_{_s}_r8_fp64_cuda_1h", "cuda_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600,
         _T),
    ]
CASES += [
    # the fast CUDA FP32 build against the real*8 run: P4's FP32 bound (real_p4_fp32_1h) until a measurement
    ("real_lk1_fp32_1h", "cuda", "configs/real_lk1_r8.ini", [f"real.dir={WR}/run_LK1_r8"] + _1H,
     f"{WR}/run_LK1_r8/wrfout_d01_{_D}", 3600, {"U": 3e-3, "W": 7e-2, "THM": 3e-5, "QV": 2e-3}),
]

STATUS = [
    ('LK1', 'P4 with the CLM lake model (sf_lake_physics 1, use_lakedepth 0, lakedepth_default 50 m)', '1 h'),
    ('LK2 / LK3', 'LK1 with lakedepth_default 10 m / 0 (spval column)', '1 h'),
    ('LKV', 'V2 (Svalbard, polar night) with the CLM lake model', '1 h'),
    ('LKN', 'NMP1 (Noah-MP) with the CLM lake model', '1 h'),
]

DATA = {c[0]: 'port_b13_data.tar' for c in CASES}
