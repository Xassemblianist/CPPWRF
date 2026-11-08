# SPDX-License-Identifier: GPL-3.0-or-later
"""The Morrison two-moment microphysics (mp_physics = 10, phys/module_mp_morr_two_moment.F): MORR (hail,
morr_rimed_ice = 1, with Kain-Fritsch's rain / snow / ice tendencies) and MORG (graupel, morr_rimed_ice = 0, no
cumulus) on the P4 physics, MORN on the N3 nest, each against its WRF v4.8.0 real*8 run (tools/colab/colab_wrfref.sh,
the inputs of run_P4_r8 / run_N3s_r8).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0, "QICE": 0.0, "QSNOW": 0.0,
      "QGRAUP": 0.0, "QNICE": 0.0, "QNSNOW": 0.0, "QNRAIN": 0.0, "QNGRAUPEL": 0.0, "RAINNC": 0.0}
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "QRAIN", "QNRAIN", "QNSNOW", "RAINNC")}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = [
    # MORR = P4 with Morrison (hail) and KF: 3 steps (history every step), 1 h, 1 h restarted at 30 min (wrfrst identical)
    ("real_morr_r8_fp64_cpu", "cpu_dp", "configs/real_morr_r8.ini", [],
     f"{WR}/run_MORR_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_morr_r8_fp64_cpu_1h", "cpu_dp", "configs/real_morr_r8.ini", ["real.dir=" + f"{WR}/run_MORR_r8"] + _1H,
     f"{WR}/run_MORR_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_morr_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_morr_r8.ini",
     ["real.dir=" + f"{WR}/run_MORR_r8"] + _1H + ["suite.restart_at=1800"],
     f"{WR}/run_MORR_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_morr_r8_fp64_cuda_1h", "cuda_dp", "configs/real_morr_r8.ini", ["real.dir=" + f"{WR}/run_MORR_r8"] + _1H,
     f"{WR}/run_MORR_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QNRAIN": 0.0}),
    # FP32 GPU against the real*8 run (no real*4 WRF run of MORR): bounds of the P4 FP32 case's order
    ("real_morr_fp32_1h", "cuda", "configs/real_morr_r8.ini", ["real.dir=" + f"{WR}/run_MORR_r8"] + _1H,
     f"{WR}/run_MORR_r8/wrfout_d01_{_D}", 3600, {"U": 1e-2, "W": 2e-1, "THM": 1e-4, "QV": 1e-2}),
    # MORG = P4 with Morrison (graupel), no cumulus: 3 steps, 1 h
    ("real_morg_r8_fp64_cpu", "cpu_dp", "configs/real_morg_r8.ini", [],
     f"{WR}/run_MORG_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_morg_r8_fp64_cpu_1h", "cpu_dp", "configs/real_morg_r8.ini", ["real.dir=" + f"{WR}/run_MORG_r8"] + _1H,
     f"{WR}/run_MORG_r8/wrfout_d01_{_D}", 3600, _T),
    # MORN = N3 with Morrison on both domains: 3 minutes (history every minute), 1 h
    ("real_morn_r8_fp64_cpu", "cpu_dp", "configs/real_morn_r8.ini", [],
     {d: f"{WR}/run_MORN_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_morn_r8_fp64_cpu_1h", "cpu_dp", "configs/real_morn_r8.ini",
     ["real.dir=" + f"{WR}/run_MORN_r8", "phys.table_dir=" + f"{WR}/run_MORN_r8", "run.run_time=3600",
      "output.out_interval=10", "output.diag_interval=10", "d02.out_interval=30", "d02.diag_interval=30"],
     {d: f"{WR}/run_MORN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('MORR', 'P4 with the Morrison two-moment microphysics (mp_physics 10, hail) and KF (restarted at 30 min)', '1 h'),
    ('MORG', 'P4 with Morrison with graupel (morr_rimed_ice 0), no cumulus', '1 h'),
    ('MORN', 'N3 with Morrison on both domains', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_c04_data.tar' for c in CASES}
