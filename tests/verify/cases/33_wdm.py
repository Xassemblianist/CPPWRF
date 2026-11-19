# SPDX-License-Identifier: GPL-3.0-or-later
"""WDM6 and WDM5 microphysics
(mp_physics = 16, phys/module_mp_wdm6.F; 14, module_mp_wdm5.F): D6 = P4 (Kain-Fritsch, RRTMG with WDM6's effective
radii, YSU, Noah) with WDM6 instead of WSM6; D6H the same with hail_opt = 1; D6C with ccn_conc = 3e8 (10 min); D5 =
P4 with WDM5 (no graupel: QGRAUP and GRAUPELNC stay 0, as in WRF). Each against its WRF v4.8.0
real*8 run (wrfref workflow of Xassemblianist/cppwrf-data, tag c03-wdm: the inputs of run_P4_r8, which have no QNCCN /
QNCLOUD / QNRAIN, so NN starts from ccn_conc and NC, NR from 0 as in WRF; wrfrst of D6 at 30 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T6 = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0,
       "QICE": 0.0, "QSNOW": 0.0, "QGRAUP": 0.0, "QNCCN": 0.0, "QNCLOUD": 0.0, "QNRAIN": 0.0, "RAINNC": 0.0,
       "SNOWNC": 0.0, "GRAUPELNC": 0.0, "SR": 0.0, "RE_CLOUD": 0.0, "RE_ICE": 0.0, "RE_SNOW": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n, _rst in (("d6", True), ("d6h", False), ("d5", False)):
    _cfg, _run = f"configs/real_p4{_n}_r8.ini", f"{WR}/run_{_n.upper()}_r8"
    CASES += [
        # 3 steps (history every step), 1 h (D6: also restarted at 30 min, its wrfrst identical to WRF's)
        (f"real_p4{_n}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T6),
        (f"real_p4{_n}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}",
         3600, _T6),
    ]
    if _rst:
        CASES += [(f"real_p4{_n}_r8_fp64_cpu_1h_rst", "cpu_dp", _cfg,
                   ["real.dir=" + _run] + _1H + ["suite.restart_at=1800"], f"{_run}/wrfout_d01_{_D}", 3600, _T6)]
    CASES += [
        # CUDA FP64 (exact on the G4, 2026-11-17) and FP32 (measured on the G4 on top of the RRTMG float change:
        # D6 U 1.5e-3, W 8.4e-2, THM 3.1e-5, QV 8.2e-4; D6H U 1.6e-3, W 9.2e-2, THM 3.2e-5, QV 8.5e-4; D5 about D6's;
        # tolerances about 3x the largest)
        (f"real_p4{_n}_r8_fp64_cuda_1h", "cuda_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}",
         3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QNCCN": 0.0, "QNCLOUD": 0.0, "QNRAIN": 0.0}),
        (f"real_p4{_n}_fp32_1h", "cuda", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600,
         {"U": 4.8e-3, "W": 2.8e-1, "THM": 9.6e-5, "QV": 2.6e-3}),
    ]
_cfg, _run = "configs/real_p4d6c_r8.ini", f"{WR}/run_D6C_r8"
CASES += [
    ("real_p4d6c_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T6),
    ("real_p4d6c_r8_fp64_cpu_10m", "cpu_dp", _cfg,
     ["real.dir=" + _run, "run.run_time=600", "output.out_interval=10", "output.diag_interval=10"],
     f"{_run}/wrfout_d01_{_D}", 600, _T6),
]
# CUDA FP64 twins (tolerance 0) of D6C's CPU cases: the same configuration, reference and variables, kind cuda_dp
CASES += [(c[0].replace("_fp64_cpu", "_fp64_cuda"), "cuda_dp") + tuple(c[2:]) for c in list(CASES)
          if c[1] == "cpu_dp" and c[0].startswith("real_p4d6c_")]

STATUS = [
    ('D6', 'P4 with WDM6 (mp_physics 16) instead of WSM6; restarted at 30 min', '1 h'),
    ('D6H', 'D6 with hail_opt 1', '1 h'),
    ('D6C', 'D6 with ccn_conc 3e8', '10 min'),
    ('D5', 'P4 with WDM5 (mp_physics 14) instead of WSM6', '1 h'),
]

DATA = {c[0]: 'suite_data.tar,port_c03_data.tar' for c in CASES}
