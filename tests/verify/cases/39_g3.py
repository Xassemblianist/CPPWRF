# SPDX-License-Identifier: GPL-3.0-or-later
"""Grell-3D cumulus (cu_physics = 5, phys/module_cu_g3.F): G3P8 (P8's inputs and physics with Grell-3D and
ishallow = 1 instead of Grell-Freitas-Li; qv only), G3P4 (P4: WSM6, RRTMG, YSU; Grell-3D instead of Kain-Fritsch: the
QC / QI branches and the cumulus clouds) and G3P4A3 (cugd_avedx = 3), each against its WRF v4.8.0 real*8 run (the
wrfref workflow of the CI repository, run c12-g3: the inputs of run_P8_r8 / run_P4_r8).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "RAINC": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_P4 = ["phys.cu_physics=5"]

CASES = [
    # G3P8: 3 steps (history every step), 1 h, 1 h restarted at 30 min
    ("real_p8g3_r8_fp64_cpu", "cpu_dp", "configs/real_p8g3_r8.ini", [],
     f"{WR}/run_G3P8_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_p8g3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_p8g3_r8.ini", ["real.dir=" + f"{WR}/run_G3P8_r8"] + _1H,
     f"{WR}/run_G3P8_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_p8g3_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_p8g3_r8.ini",
     ["real.dir=" + f"{WR}/run_G3P8_r8"] + _1H + ["suite.restart_at=1800"],
     f"{WR}/run_G3P8_r8/wrfout_d01_{_D}", 3600, _T),
    # G3P4 / G3P4A3: P4 with Grell-3D instead of KF (cugd_avedx 1 / 3)
    ("real_p4g3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_p4_r8.ini", ["real.dir=" + f"{WR}/run_G3P4_r8"] + _P4 + _1H,
     f"{WR}/run_G3P4_r8/wrfout_d01_{_D}", 3600, dict(_T, QCLOUD=0.0, QICE=0.0)),
    ("real_p4g3a3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_p4_r8.ini",
     ["real.dir=" + f"{WR}/run_G3P4A3_r8", "phys.cugd_avedx=3"] + _P4 + _1H,
     f"{WR}/run_G3P4A3_r8/wrfout_d01_{_D}", 3600, dict(_T, QCLOUD=0.0, QICE=0.0)),
    # CUDA FP64 (tolerance 0; SKIP without a GPU) and FP32 (P8's tolerances until measured on the G4)
    ("real_p8g3_r8_fp64_cuda_1h", "cuda_dp", "configs/real_p8g3_r8.ini", ["real.dir=" + f"{WR}/run_G3P8_r8"] + _1H,
     f"{WR}/run_G3P8_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    ("real_p4g3_r8_fp64_cuda_1h", "cuda_dp", "configs/real_p4_r8.ini", ["real.dir=" + f"{WR}/run_G3P4_r8"] + _P4 + _1H,
     f"{WR}/run_G3P4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    # FP32: measured on the G4 2026-11-16, tolerances about 3x (CUDA FP64 exact)
    ("real_p8g3_fp32_1h", "cuda", "configs/real_p8g3_r8.ini", ["real.dir=" + f"{WR}/run_G3P8_r8"] + _1H,
     f"{WR}/run_G3P8_r8/wrfout_d01_{_D}", 3600, {"U": 9.3e-4, "W": 2.9e-2, "THM": 1.8e-5, "QV": 1.1e-3}),
]

STATUS = [
    ('G3P8', 'P8 with the Grell-3D cumulus (cu_physics 5, ishallow 1; restarted at 30 min)', '1 h'),
    ('G3P4', 'P4 with Grell-3D instead of Kain-Fritsch (cugd_avedx 1 and 3)', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_c12_data.tar' for c in CASES}  # the inputs and tables: links into run_P8_r8 / run_P4_r8
