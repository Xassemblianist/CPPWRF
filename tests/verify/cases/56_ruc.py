# SPDX-License-Identifier: GPL-3.0-or-later
"""RUC land surface model
(sf_surface_physics = 3, phys/module_sf_ruclsm.F): RUC4 = P4 (WSM6, Kain-Fritsch, RRTMG, YSU, sfclayrev) with RUC
and 9 soil levels instead of Noah; RUC4J = RUC4 with MYJSFC, the MYJ PBL (LSMRUC's myj branch) and 6 soil levels.
Each against its WRF v4.8.0 real*8 run on real.exe's RUC input from run_P4_r8's met_em files (wrfref with
run.real=real.exe:run_P4_r8; wrfrst at 30 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0,
      "QICE": 0.0, "QSNOW": 0.0, "QGRAUP": 0.0, "RAINNC": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

# FP32 tolerances: about 3x the rel L2 measured on the Colab G4 (RUC4 U 6.3e-4 W 1.7e-2 THM 8.6e-6 QV 5.3e-4;
# RUC4J U 5.1e-4 W 6.9e-3 THM 1.5e-5 QV 1.8e-3)
_SP = {"RUC4": {"U": 1.9e-3, "W": 5.1e-2, "THM": 2.6e-5, "QV": 1.6e-3},
       "RUC4J": {"U": 1.5e-3, "W": 2.1e-2, "THM": 4.5e-5, "QV": 5.4e-3}}

CASES = []
for _n in ("RUC4", "RUC4J"):
    _cfg, _run, _c = f"configs/real_{_n.lower()}_r8.ini", f"{WR}/run_{_n}_r8", _n.lower()
    CASES += [
        # 3 steps (history every step), 1 h, and 1 h restarted at 30 min (its wrfrst identical to WRF's)
        (f"real_{_c}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_c}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _T),
        (f"real_{_c}_r8_fp64_cpu_1h_rst", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H + ["suite.restart_at=1800"],
         f"{_run}/wrfout_d01_{_D}", 3600, _T),
        # CUDA FP64 (tolerance 0) and FP32 (tolerances measured on the G4, about 3x)
        (f"real_{_c}_r8_fp64_cuda_1h", "cuda_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}",
         3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
        (f"real_{_c}_fp32_1h", "cuda", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _SP[_n]),
    ]

STATUS = [
    ('RUC4', 'P4 with the RUC LSM (sf_surface_physics 3, 9 levels) instead of Noah; restarted at 30 min', '1 h'),
    ('RUC4J', 'RUC4 with MYJSFC + MYJ PBL and 6 soil levels; restarted at 30 min', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_c23_data.tar' for c in CASES}
