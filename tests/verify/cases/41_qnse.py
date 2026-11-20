# SPDX-License-Identifier: GPL-3.0-or-later
"""The QNSE PBL and surface layer (bl_pbl_physics = 4, sf_sfclay_physics = 4, mfshconv = 0; phys/module_bl_qnsepbl.F,
phys/module_sf_qnsesfc.F): Q1 / Q4 = P1 / P4 with QNSE (P1 without microphysics, P4 with WSM6 / KF / RRTMG), each
against its WRF v4.8.0 real*8 run (wrfref workflow of Xassemblianist/cppwrf-data, tag c17-qnse: the inputs of
run_P1_r8 / run_P4_r8 with physics.bl_pbl_physics = 4, physics.sf_sfclay_physics = 4, physics.mfshconv = 0; Q4 with
wrfrst at 30 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "PBLH": 0.0, "HFX": 0.0, "LH": 0.0,
      "UST": 0.0, "TKE_PBL": 0.0, "EL_PBL": 0.0, "T2": 0.0, "Q2": 0.0, "U10": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n in ("1", "4"):
    _cfg, _run = f"configs/real_q{_n}_r8.ini", f"{WR}/run_Q{_n}_r8"
    CASES += [
        # 3 steps (history every step) and 1 h
        (f"real_q{_n}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_q{_n}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}",
         3600, _T),
    ]
CASES += [
    # Q4 restarted at 30 min (its wrfrst identical to WRF's)
    ("real_q4_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_q4_r8.ini",
     ["real.dir=" + f"{WR}/run_Q4_r8"] + _1H + ["suite.restart_at=1800"], f"{WR}/run_Q4_r8/wrfout_d01_{_D}", 3600, _T),
    # CUDA FP64 and FP32: tolerances not measured yet (BouLac's placeholders; the merge train measures them, ~3x the
    # relative L2 of a G4 run)
    ("real_q4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_q4_r8.ini", ["real.dir=" + f"{WR}/run_Q4_r8"] + _1H,
     f"{WR}/run_Q4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    # FP32: measured on the G4 2026-11-16, tolerances about 3x (CUDA FP64 exact); U and QV again 3x of the G4 run
    # at fb158f3 (the real*4 constants of the FP32 builds, 2026-11-20: U 6.5e-4, W 1.8e-2, THM 9.1e-6, QV 5.6e-4)
    ("real_q4_fp32_1h", "cuda", "configs/real_q4_r8.ini", ["real.dir=" + f"{WR}/run_Q4_r8"] + _1H,
     f"{WR}/run_Q4_r8/wrfout_d01_{_D}", 3600, {"U": 2.0e-3, "W": 3.0e-2, "THM": 1.6e-5, "QV": 1.7e-3}),
]

STATUS = [
    ('Q1', 'P1 with the QNSE PBL and surface layer (bl_pbl_physics 4, sf_sfclay_physics 4, mfshconv 0)', '1 h'),
    ('Q4', 'P4 with the QNSE PBL and surface layer (restarted at 30 min)', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_c17_data.tar' for c in CASES}  # the inputs and tables: links into run_P1_r8 / P4
