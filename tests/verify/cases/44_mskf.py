# SPDX-License-Identifier: GPL-3.0-or-later
"""Multi-scale Kain-Fritsch cumulus (cu_physics = 11, phys/module_cu_mskf.F): M4 = P4 (WSM6, RRTMG, YSU, sfclayrev,
Noah, 15 km) with MSKF instead of Kain-Fritsch (its cloud fractions in RRTMG: icloud_cu = 2), M4C5 = M4 with
cudt = 5, M9 = P9 (MYJ, no microphysics) with MSKF, M3N = the N3 two-way nest (d02 5 km) with MSKF on both domains.
Each against its WRF v4.8.0 real*8 run (wrfref, tag c14-mskf: run_P4_r8 / run_P9_r8 / run_N3s_r8 inputs, 1 h,
history every 10 min, wrfrst at 30 min; M4's steps/ = 3 steps, history every step).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_X = ("U", "V", "W", "PH", "THM", "QV", "MU", "QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP", "RAINC", "RAINNC",
      "NCA", "W0AVG", "RTHCUTEN", "RQVCUTEN")
_T = {x: 0.0 for x in _X}
_T9 = {x: 0.0 for x in ("U", "V", "W", "PH", "THM", "QV", "MU", "RAINC", "NCA", "W0AVG", "RTHCUTEN", "RQVCUTEN")}
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "QCLOUD", "QRAIN", "RAINC", "RTHCUTEN")}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_M4 = f"{WR}/run_M4_r8"

CASES = [
    # 3 steps (history every step), 1 h, and 1 h restarted at 30 min (its wrfrst identical to WRF's)
    ("real_m4_r8_fp64_cpu", "cpu_dp", "configs/real_m4_r8.ini", [], f"{_M4}/steps/wrfout_d01_{_D}", 180, _T),
    ("real_m4_r8_fp64_cpu_1h", "cpu_dp", "configs/real_m4_r8.ini", ["real.dir=" + _M4] + _1H,
     f"{_M4}/wrfout_d01_{_D}", 3600, _T),
    ("real_m4_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_m4_r8.ini", ["real.dir=" + _M4] + _1H +
     ["suite.restart_at=1800"], f"{_M4}/wrfout_d01_{_D}", 3600, _T),
    # cudt = 5: MSKF every 5 steps (NCA counts the steps, TIMEC / shallow NCA = cudt*60)
    ("real_m4c5_r8_fp64_cpu_1h", "cpu_dp", "configs/real_m4_r8.ini",
     ["real.dir=" + f"{WR}/run_M4C5_r8", "phys.cudt=5"] + _1H, f"{WR}/run_M4C5_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_m9_r8_fp64_cpu_1h", "cpu_dp", "configs/real_m9_r8.ini", [], f"{WR}/run_M9_r8/wrfout_d01_{_D}", 3600, _T9),
    ("real_m3n_r8_fp64_cpu_1h", "cpu_dp", "configs/real_m3n_r8.ini", [],
     {d: f"{WR}/run_M3N_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
    # CUDA FP64 (tolerance 0)
    ("real_m4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_m4_r8.ini", ["real.dir=" + _M4] + _1H,
     f"{_M4}/wrfout_d01_{_D}", 3600, _T),
    ("real_m9_r8_fp64_cuda_1h", "cuda_dp", "configs/real_m9_r8.ini", [], f"{WR}/run_M9_r8/wrfout_d01_{_D}", 3600,
     _T9),
    ("real_m3n_r8_fp64_cuda_1h", "cuda_dp", "configs/real_m3n_r8.ini", [],
     {d: f"{WR}/run_M3N_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
    # FP32 against the real*8 runs (tolerances: about 3x the rel L2 measured on the Colab G4 at 37213be: M4 U 4.8e-4
    # W 1.0e-2 THM 5.6e-6 QV 3.2e-4; M9 U 7.0e-4 W 9.4e-3 THM 9.7e-6 QV 1.6e-3)
    ("real_m4_fp32_1h", "cuda", "configs/real_m4_r8.ini", ["real.dir=" + _M4] + _1H, f"{_M4}/wrfout_d01_{_D}", 3600,
     {"U": 1.5e-3, "W": 3e-2, "THM": 1.7e-5, "QV": 1e-3}),
    ("real_m9_fp32_1h", "cuda", "configs/real_m9_r8.ini", [], f"{WR}/run_M9_r8/wrfout_d01_{_D}", 3600,
     {"U": 2.1e-3, "W": 2.8e-2, "THM": 2.9e-5, "QV": 4.8e-3}),
]

STATUS = [
    ('M4', 'P4 with the multi-scale Kain-Fritsch (cu_physics 11) instead of KF; restarted at 30 min', '1 h'),
    ('M4C5', 'M4 with cudt = 5', '1 h'),
    ('M9', 'P9 (MYJ, no microphysics) with the multi-scale Kain-Fritsch', '1 h'),
    ('M3N', 'N3 two-way nest with the multi-scale Kain-Fritsch on both domains', '1 h'),
]

DATA = {c[0]: 'n3_data.tar,port_c14_data.tar' if '_m3n_' in c[0] else 'suite_data.tar,port_c14_data.tar' for c in CASES}
