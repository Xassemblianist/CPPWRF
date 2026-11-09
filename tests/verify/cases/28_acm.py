# SPDX-License-Identifier: GPL-3.0-or-later
"""The ACM2 PBL (bl_pbl_physics = 7, phys/module_bl_acm.F) over the revised MM5 (sf_sfclay_physics = 1) and the
Pleim-Xiu (7, phys/module_sf_pxsfclay.F) surface layers: ACM1 / ACM7 / ACMB on the P4 physics and ACMN on the N3 nest,
each against its WRF v4.8.0 real*8 run (tools/colab/colab_wrfref.sh, the inputs of run_P4_r8 / run_N3s_r8).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "PBLH": 0.0, "HFX": 0.0, "UST": 0.0}
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "PBLH", "HFX")}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = [
    # ACM1 = P4 with ACM2 over sfclayrev: 3 steps (history every step), 1 h, 1 h restarted at 30 min (wrfrst identical)
    ("real_acm1_r8_fp64_cpu", "cpu_dp", "configs/real_acm1_r8.ini", [],
     f"{WR}/run_ACM1_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_acm1_r8_fp64_cpu_1h", "cpu_dp", "configs/real_acm1_r8.ini", ["real.dir=" + f"{WR}/run_ACM1_r8"] + _1H,
     f"{WR}/run_ACM1_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_acm1_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_acm1_r8.ini",
     ["real.dir=" + f"{WR}/run_ACM1_r8"] + _1H + ["suite.restart_at=1800"],
     f"{WR}/run_ACM1_r8/wrfout_d01_{_D}", 3600, _T),
    # ACM7 = P4 with ACM2 over the Pleim-Xiu surface layer: the same three cases
    ("real_acm7_r8_fp64_cpu", "cpu_dp", "configs/real_acm7_r8.ini", [],
     f"{WR}/run_ACM7_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_acm7_r8_fp64_cpu_1h", "cpu_dp", "configs/real_acm7_r8.ini", ["real.dir=" + f"{WR}/run_ACM7_r8"] + _1H,
     f"{WR}/run_ACM7_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_acm7_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_acm7_r8.ini",
     ["real.dir=" + f"{WR}/run_ACM7_r8"] + _1H + ["suite.restart_at=1800"],
     f"{WR}/run_ACM7_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_acm7_r8_fp64_cuda_1h", "cuda_dp", "configs/real_acm7_r8.ini", ["real.dir=" + f"{WR}/run_ACM7_r8"] + _1H,
     f"{WR}/run_ACM7_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    # FP32 GPU against the real*8 run: about 3x the values measured on an L4 (sm_89) by tools/bench/colab_fp32.sh
    # (2026-11-09, docs/FP32_ACCURACY.md): U 5.1e-4, W 1.0e-2, THM 9.9e-6, QV 4.8e-4
    ("real_acm7_fp32_1h", "cuda", "configs/real_acm7_r8.ini", ["real.dir=" + f"{WR}/run_ACM7_r8"] + _1H,
     f"{WR}/run_ACM7_r8/wrfout_d01_{_D}", 3600, {"U": 2e-3, "W": 3e-2, "THM": 3e-5, "QV": 2e-3}),
    # ACMB = ACM7 with bldt = 3 (the surface layer, Noah and ACM2 every third step with dtbl = 180 s)
    ("real_acmb_r8_fp64_cpu_1h", "cpu_dp", "configs/real_acm7_r8.ini",
     ["real.dir=" + f"{WR}/run_ACMB_r8", "phys.bldt=3"] + _1H,
     f"{WR}/run_ACMB_r8/wrfout_d01_{_D}", 3600, _T),
    # ACMN = N3 with ACM2 and the Pleim-Xiu surface layer on both domains: 3 minutes (history every minute), 1 h
    ("real_acmn_r8_fp64_cpu", "cpu_dp", "configs/real_acmn_r8.ini", [],
     {d: f"{WR}/run_ACMN_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_acmn_r8_fp64_cpu_1h", "cpu_dp", "configs/real_acmn_r8.ini",
     ["real.dir=" + f"{WR}/run_ACMN_r8", "phys.table_dir=" + f"{WR}/run_ACMN_r8", "run.run_time=3600",
      "output.out_interval=10", "output.diag_interval=10", "d02.out_interval=30", "d02.diag_interval=30"],
     {d: f"{WR}/run_ACMN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('ACM1', 'P4 with the ACM2 PBL over the revised MM5 surface layer (bl_pbl_physics 7; restarted at 30 min)', '1 h'),
    ('ACM7', 'P4 with ACM2 over the Pleim-Xiu surface layer (sf_sfclay_physics 7; restarted at 30 min)', '1 h'),
    ('ACMB', 'ACM7 with bldt 3', '1 h'),
    ('ACMN', 'N3 with ACM2 and the Pleim-Xiu surface layer on both domains', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_c16_data.tar' for c in CASES}
