# SPDX-License-Identifier: GPL-3.0-or-later
"""Grid (analysis) nudging, grid_fdda = 1 (phys/module_fddagd_driver.F, module_fdda_psufddagd.F): FD1 ... FD9 on the
P4 / P9 physics and FDN on the N3 nest, each against its WRF v4.8.0 real*8 run with real.exe's wrffdda_d0N.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "HFX_FDDA": 0.0}
_T9 = _T  # REGIME is in the restart stream only
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "HFX_FDDA")}
# the [fdda] options of FD2 ... FD7 (the namelists of their WRF runs: tools/colab/colab_wrfref.sh refs, B03 report)
_FD2 = ["fdda.if_no_pbl_nudging_uv=1", "fdda.if_no_pbl_nudging_t=1", "fdda.if_no_pbl_nudging_q=1", "fdda.if_zfac_uv=1",
        "fdda.k_zfac_uv=10", "fdda.if_zfac_t=1", "fdda.k_zfac_t=5", "fdda.if_zfac_q=1", "fdda.k_zfac_q=8", "fdda.fgdt=3"]
_FD3 = ["fdda.guv=0.001", "fdda.gt=0.0005", "fdda.gq=0.0002", "fdda.fgdt=2", "fdda.fgdtzero=1"]
_FD4 = ["fdda.gfdda_end_h=1", "fdda.if_ramping=1", "fdda.dtramp_min=-30"]
_FD5 = ["fdda.gfdda_end_h=1", "fdda.if_ramping=1", "fdda.dtramp_min=30"]
_FD7 = ["fdda.gfdda_end_h=12"]
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = [
    # FD1 = P4 with grid_fdda = 1 (guv = gt = gq = 3e-4, wrffdda_d01 every 6 h, nudging until 6 h): 3 steps (history every
    # step), 1 h; the restart (wrfrst at minute 1) and the run resumed from it: run_wrf_files_test.sh fd1
    ("real_fd1_r8_fp64_cpu", "cpu_dp", "configs/real_fd1_r8.ini", [],
     f"{WR}/run_FD1_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_fd1_r8_fp64_cpu_1h", "cpu_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD1_r8"] + _1H,
     f"{WR}/run_FD1_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_fd1_r8_fp64_cuda_1h", "cuda_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD1_r8"] + _1H,
     f"{WR}/run_FD1_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    # FP32 GPU against the real*8 run (no real*4 WRF run of FD1): bounds of the P4 FP32 case's order
    ("real_fd1_fp32_1h", "cuda", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD1_r8"] + _1H,
     f"{WR}/run_FD1_r8/wrfout_d01_{_D}", 3600, {"U": 1e-2, "W": 2e-1, "THM": 1e-4, "QV": 1e-2}),
    # FD2 = FD1 without nudging in the PBL (if_no_pbl_nudging_uv / t / q), not in the lowest levels (if_zfac_*, k_zfac_*
    # 10 / 5 / 8) and every 3 steps (fgdt = 3 min: STEPFG 3, the tendencies kept between the calls)
    ("real_fd2_r8_fp64_cpu", "cpu_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD2_r8/steps"] + _FD2,
     f"{WR}/run_FD2_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_fd2_r8_fp64_cpu_1h", "cpu_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD2_r8"] + _FD2 + _1H,
     f"{WR}/run_FD2_r8/wrfout_d01_{_D}", 3600, _T),
    # FD3 = FD1 with guv 1e-3, gt 5e-4, gq 2e-4, fgdt = 2 min and fgdtzero = 1 (the tendencies 0 between the calls)
    ("real_fd3_r8_fp64_cpu", "cpu_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD3_r8/steps"] + _FD3,
     f"{WR}/run_FD3_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_fd3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_fd1_r8.ini", ["real.dir=" + f"{WR}/run_FD3_r8"] + _FD3 + _1H,
     f"{WR}/run_FD3_r8/wrfout_d01_{_D}", 3600, _T),
    # FD4 / FD5 = FD1 ending at 1 h with if_ramping = 1: dtramp_min = -30 (the ramp from 30 to 60 min) / +30 (from 60 to
    # 90 min toward the old analysis only); then no nudging (90 / 100 min)
    ("real_fd4_r8_fp64_cpu_90m", "cpu_dp", "configs/real_fd1_r8.ini",
     ["real.dir=" + f"{WR}/run_FD4_r8"] + _FD4 + _1H[1:] + ["run.run_time=5400"],
     f"{WR}/run_FD4_r8/wrfout_d01_{_D}", 5400, _T),
    ("real_fd5_r8_fp64_cpu_100m", "cpu_dp", "configs/real_fd1_r8.ini",
     ["real.dir=" + f"{WR}/run_FD5_r8"] + _FD5 + _1H[1:] + ["run.run_time=6000"],
     f"{WR}/run_FD5_r8/wrfout_d01_{_D}", 6000, _T),
    # FD7 = FD1 nudging until 12 h for 7 h (real.exe to 12 h: wrffdda_d01 at 0 and 6 h; the analyses of 6 and 12 h read
    # at 6 h)
    ("real_fd7_r8_fp64_cpu_7h", "cpu_dp", "configs/real_fd1_r8.ini",
     ["real.dir=" + f"{WR}/run_FD7_r8", "run.run_time=25200", "output.out_interval=60", "output.diag_interval=60"] + _FD7,
     f"{WR}/run_FD7_r8/wrfout_d01_{_D}", 25200, _T),
    # FD9 = P9 (MYJ surface layer: fddagd_driver's REGIME from PBLH) with FD1's nudging
    ("real_fd9_r8_fp64_cpu", "cpu_dp", "configs/real_fd9_r8.ini", [],
     f"{WR}/run_FD9_r8/steps/wrfout_d01_{_D}", 180, _T9),
    ("real_fd9_r8_fp64_cpu_1h", "cpu_dp", "configs/real_fd9_r8.ini", ["real.dir=" + f"{WR}/run_FD9_r8"] + _1H,
     f"{WR}/run_FD9_r8/wrfout_d01_{_D}", 3600, _T9),
    # FDN = N3 with nudging on both domains (d02: guv 6e-4, its own wrffdda_d02); restart: run_wrf_files_test.sh fdn
    ("real_fdn_r8_fp64_cpu", "cpu_dp", "configs/real_fdn_r8.ini", [],
     {d: f"{WR}/run_FDN_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_fdn_r8_fp64_cpu_1h", "cpu_dp", "configs/real_fdn_r8.ini",
     ["real.dir=" + f"{WR}/run_FDN_r8", "run.run_time=3600", "output.out_interval=10", "output.diag_interval=10",
      "d02.out_interval=30", "d02.diag_interval=30"],
     {d: f"{WR}/run_FDN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('FD1', 'P4 with grid nudging (grid_fdda 1, wrffdda from real.exe every 6 h; restart and resumed run)', '1 h'),
    ('FD2 / FD3', 'FD1 without PBL / lowest-level nudging, fgdt 3 / other coefficients, fgdt 2, fgdtzero 1', '1 h'),
    ('FD4 / FD5', 'FD1 ending at 1 h with if_ramping 1 (dtramp_min -30 / +30)', '90 / 100 min'),
    ('FD7', 'FD1 nudging until 12 h (the 6 h analyses read during the run)', '7 h'),
    ('FD9', 'P9 (MYJ surface layer, REGIME from PBLH) with grid nudging', '1 h'),
    ('FDN', 'N3 with grid nudging on both domains (wrffdda_d02)', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_b03n_data.tar' if c[0].startswith('real_fdn') else 'port_b03_data.tar' for c in CASES}
