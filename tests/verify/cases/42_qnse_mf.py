# SPDX-License-Identifier: GPL-3.0-or-later
"""The EDKF mass-flux shallow convection of the QNSE PBL (mfshconv = 1, WRF's default with bl_pbl_physics = 4;
phys/module_bl_mfshconvpbl.F before QNSEPBL): Q1M / Q4M = Q1 / Q4 (P1 / P4 with the QNSE PBL and surface layer) with
mfshconv = 1, each against its WRF v4.8.0 real*8 run (wrfref workflow of Xassemblianist/cppwrf-data, tag
c17b-mfshconv: the inputs of run_P1_r8 / run_P4_r8 with physics.bl_pbl_physics = 4, physics.sf_sfclay_physics = 4,
physics.mfshconv = 1; wrfrst at 30 min). The restarts carry RC_MF, the scheme's state.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "PBLH": 0.0, "HFX": 0.0, "LH": 0.0,
      "UST": 0.0, "TKE_PBL": 0.0, "EL_PBL": 0.0, "T2": 0.0, "Q2": 0.0, "U10": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n in ("1", "4"):
    _cfg, _run = f"configs/real_q{_n}m_r8.ini", f"{WR}/run_Q{_n}M_r8"
    CASES += [
        # 3 steps (history every step), 1 h, and 1 h restarted at 30 min (its wrfrst identical to WRF's)
        (f"real_q{_n}m_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_q{_n}m_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}",
         3600, _T),
        (f"real_q{_n}m_r8_fp64_cpu_1h_rst", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H + ["suite.restart_at=1800"],
         f"{_run}/wrfout_d01_{_D}", 3600, _T),
    ]
CASES += [
    # CUDA FP64 (exact) and FP32 (about 3x the G4 measurement: U 3.8e-4 W 1.8e-2 THM 5.4e-6 QV 3.5e-4)
    ("real_q4m_r8_fp64_cuda_1h", "cuda_dp", "configs/real_q4m_r8.ini", ["real.dir=" + f"{WR}/run_Q4M_r8"] + _1H,
     f"{WR}/run_Q4M_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    ("real_q4m_fp32_1h", "cuda", "configs/real_q4m_r8.ini", ["real.dir=" + f"{WR}/run_Q4M_r8"] + _1H,
     f"{WR}/run_Q4M_r8/wrfout_d01_{_D}", 3600, {"U": 1.1e-3, "W": 5.4e-2, "THM": 1.6e-5, "QV": 1.1e-3}),
]

STATUS = [
    ('Q1M', 'Q1 with the EDKF mass flux of the QNSE PBL (mfshconv 1; restarted at 30 min)', '1 h'),
    ('Q4M', 'Q4 with the EDKF mass flux of the QNSE PBL (mfshconv 1; restarted at 30 min)', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_c17b_data.tar' for c in CASES}  # the inputs and tables: links into run_P1_r8 / P4
