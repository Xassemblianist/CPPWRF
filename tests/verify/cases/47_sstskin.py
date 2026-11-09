# SPDX-License-Identifier: GPL-3.0-or-later
"""The skin SST (sst_skin = 1, phys/module_sf_sstskin.F) and the deep soil temperature update (tmn_update = 1,
phys/module_sf_tmnupdate.F): SKV on V1 (Bay of Bengal, P4 physics, sunrise at 00 UTC: warm layer and cool skin;
without sst_update), SKU on SU1 (sst_update = 1, fractional sea ice, Thompson / MYJ), SKN on the N3 nest (lagday = 5),
each against its WRF v4.8.0 real*8 run (tools/colab/colab_wrfref.sh, the inputs of run_V1_r8 / run_SU1_r8 /
run_N3s_r8). The runs start at 00 UTC: tmnupdate's end of a day falls on the first step (TMN, TLAG, TYRA change).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "TSK": 0.0, "SSTSK": 0.0, "TMN": 0.0, "HFX": 0.0,
      "LH": 0.0}
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "TSK", "SSTSK", "TMN", "HFX")}
_G = {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "TSK": 0.0, "SSTSK": 0.0}

CASES = [
    # SKV = V1 with sst_skin and tmn_update: 3 steps (history every step), 1 h, 6 h restarted at 3 h (against WRF's restarted run)
    ("real_skv_r8_fp64_cpu", "cpu_dp", "configs/real_skv_r8.ini", [],
     f"{WR}/run_SKV_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_skv_r8_fp64_cpu_1h", "cpu_dp", "configs/real_skv_r8.ini",
     ["real.dir=" + f"{WR}/run_SKV_r8", "run.run_time=3600", "output.out_interval=30", "output.diag_interval=30"],
     f"{WR}/run_SKV_r8/wrfout_d01_{_D}", 3600, _T),
    # 6 h restarted at 3 h against WRF's run restarted at 3 h (run_SKVR_r8: run_SKV_r8's namelist in two parts,
    # tools/gcp/wrf_ref_run.py run.segment=10800): WRF's own restart changes W at one point of the north boundary
    # row in the first step after it (1 ulp), so the uninterrupted run_SKV_r8 is not the reference of a restarted run
    ("real_skvr_r8_fp64_cpu_6h_rst", "cpu_dp", "configs/real_skv_r8.ini",
     ["real.dir=" + f"{WR}/run_SKVR_r8", "run.run_time=21600", "output.out_interval=30", "output.diag_interval=30",
      "suite.restart_at=10800"],
     f"{WR}/run_SKVR_r8/wrfout_d01_{_D}", 21600, _T),
    ("real_skv_r8_fp64_cuda_1h", "cuda_dp", "configs/real_skv_r8.ini",
     ["real.dir=" + f"{WR}/run_SKV_r8", "run.run_time=3600", "output.out_interval=30", "output.diag_interval=30"],
     f"{WR}/run_SKV_r8/wrfout_d01_{_D}", 3600, _G),
    # FP32 GPU against the real*8 run (no real*4 WRF run of SKV): bounds of the P4 FP32 case's order
    ("real_skv_fp32_1h", "cuda", "configs/real_skv_r8.ini",
     ["real.dir=" + f"{WR}/run_SKV_r8", "run.run_time=3600", "output.out_interval=30", "output.diag_interval=30"],
     f"{WR}/run_SKV_r8/wrfout_d01_{_D}", 3600, {"U": 1e-2, "W": 2e-1, "THM": 1e-4, "QV": 1e-2, "TSK": 1e-4}),
    # SKU = SU1 with sst_skin and tmn_update: 3 steps, 7 h (the 6 h wrflowinp update included)
    ("real_sku_r8_fp64_cpu", "cpu_dp", "configs/real_sku_r8.ini", [],
     f"{WR}/run_SKU_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_sku_r8_fp64_cpu_7h", "cpu_dp", "configs/real_sku_r8.ini",
     ["real.dir=" + f"{WR}/run_SKU_r8", "run.run_time=25200", "output.out_interval=30", "output.diag_interval=30"],
     f"{WR}/run_SKU_r8/wrfout_d01_{_D}", 25200, _T),
    # SKN = N3 with sst_skin, tmn_update and lagday 5 on both domains: 3 minutes, 1 h, 1 h restarted at 30 min
    ("real_skn_r8_fp64_cpu", "cpu_dp", "configs/real_skn_r8.ini", [],
     {d: f"{WR}/run_SKN_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_skn_r8_fp64_cpu_1h", "cpu_dp", "configs/real_skn_r8.ini",
     ["real.dir=" + f"{WR}/run_SKN_r8", "phys.table_dir=" + f"{WR}/run_SKN_r8", "run.run_time=3600",
      "output.out_interval=10", "output.diag_interval=10", "d02.out_interval=30", "d02.diag_interval=30"],
     {d: f"{WR}/run_SKN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
    ("real_skn_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_skn_r8.ini",
     ["real.dir=" + f"{WR}/run_SKN_r8", "phys.table_dir=" + f"{WR}/run_SKN_r8", "run.run_time=3600",
      "output.out_interval=10", "output.diag_interval=10", "d02.out_interval=30", "d02.diag_interval=30",
      "suite.restart_at=1800"],
     {d: f"{WR}/run_SKN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('SKV', 'V1 with the skin SST (sst_skin 1, without sst_update) and the deep soil temperature update (tmn_update 1; '
     "restarted at 3 h, against WRF's run restarted there)", '6 h'),
    ('SKU', 'SU1 (sst_update 1, fractional sea ice) with sst_skin 1 and tmn_update 1', '7 h'),
    ('SKN', 'N3 with sst_skin 1, tmn_update 1 and lagday 5 on both domains (restarted at 30 min)', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_a11r_data.tar' if c[0].startswith('real_skvr') else 'port_a11_data.tar' for c in CASES}
