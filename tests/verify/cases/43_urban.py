# SPDX-License-Identifier: GPL-3.0-or-later
"""Single-layer urban canopy model
(sf_urban_physics = 1 with Noah, phys/module_sf_urban.F): U4 = P4 (WSM6, Kain-Fritsch, RRTMG, YSU, Noah) with the
UCM on URBPARM.TBL, U4L = U4 with use_wudapt_lcz = 1 (URBPARM_LCZ.TBL), each against its WRF v4.8.0 real*8 run
(wrfref workflow of Xassemblianist/cppwrf-data, tag b14-urban: run_P4_r8's inputs with the wrfinput attribute
SF_URBAN_PHYSICS = 1 and WRF 4.8.0's URBPARM tables, directory base_U4; U4 also writes wrfrst at 30 min, which no
case reads yet: restarts with the UCM are not ported).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0,
      "TSK": 0.0, "HFX": 0.0, "LH": 0.0, "T2": 0.0, "U10": 0.0, "RAINNC": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n, _s in (("U4", "p4u"), ("U4L", "p4ul")):
    _cfg, _run = f"configs/real_{_s}_r8.ini", f"{WR}/run_{_n}_r8"
    CASES += [
        # 3 steps (history every step) and 1 h
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_s}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H,
         f"{_run}/wrfout_d01_{_D}", 3600, _T),
    ]
CASES += [
    # CUDA FP64 (exact) and FP32 (about 3x the G4 measurement: U 6.2e-4 W 1.6e-2 THM 8.3e-6 QV 5.2e-4)
    ("real_p4u_r8_fp64_cuda_1h", "cuda_dp", "configs/real_p4u_r8.ini", [f"real.dir={WR}/run_U4_r8"] + _1H,
     f"{WR}/run_U4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "TSK": 0.0}),
    ("real_p4u_fp32_1h", "cuda", "configs/real_p4u_r8.ini", [f"real.dir={WR}/run_U4_r8"] + _1H,
     f"{WR}/run_U4_r8/wrfout_d01_{_D}", 3600, {"U": 1.9e-3, "W": 5.1e-2, "THM": 2.6e-5, "QV": 1.6e-3}),
]

STATUS = [
    ('U4', 'P4 with the single-layer urban canopy model (sf_urban_physics 1, URBPARM.TBL)', '1 h'),
    ('U4L', 'U4 with use_wudapt_lcz 1 (URBPARM_LCZ.TBL)', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_b14_data.tar' for c in CASES}
