# SPDX-License-Identifier: GPL-3.0-or-later
"""Terrain options
YSU's topographic wind correction (topo_wind = 1 / 2; dyn_em/start_em.F's ctopo / ctopo2, phys/module_bl_ysu.F) and
the slope / shading radiation (slope_rad = 1: module_surface_driver.F's TOPO_RAD_ADJ_DRVR; topo_shading = 1:
module_radiation_driver.F's toposhad): TW1 / TW2 = P1 with topo_wind 1 / 2, TW4 = P4 with topo_wind 1 (TW1 and TW4 on
the wrfinput with VAR_SSO = VAR**2, GOT_VAR_SSO = 1); SR1 = P4 with slope_rad 1 + topo_shading 1 (radt 10), SR2 = SR1
with shadlen 100 km, SR3 = P4 with slope_rad 1 alone (radt 1), SR4 = SR1 with the Dudhia shortwave. SR runs 5 h from
00 UTC (the sunrise over the domain, with shadows, is near 03-04 UTC). Each against its WRF v4.8.0 real*8 run
(wrfref; TW1 / TW2 wrfrst at 30 min, SR1 at 150 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "TSK": 0.0, "HFX": 0.0}
_TS = dict(_T, SWDOWN=0.0, GSW=0.0, SWNORM=0.0, QCLOUD=0.0, QRAIN=0.0)
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_5H = ["run.run_time=18000", "output.out_interval=30", "output.diag_interval=30"]
_G = {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}

CASES = []
for _n, _rst in (("TW1", 1800), ("TW2", 1800), ("TW4", None)):
    _cfg, _run, _c = f"configs/real_{_n.lower()}_r8.ini", f"{WR}/run_{_n}_r8", _n.lower()
    CASES += [  # 3 steps (history every step), 1 h, and 1 h restarted at 30 min (its wrfrst identical to WRF's)
        (f"real_{_c}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_c}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _T)]
    if _rst:
        CASES += [(f"real_{_c}_r8_fp64_cpu_1h_rst", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H +
                   [f"suite.restart_at={_rst}"], f"{_run}/wrfout_d01_{_D}", 3600, _T)]
for _n in ("SR1", "SR2", "SR3", "SR4"):
    _cfg, _run, _c = f"configs/real_{_n.lower()}_r8.ini", f"{WR}/run_{_n}_r8", _n.lower()
    CASES += [
        (f"real_{_c}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _TS),
        (f"real_{_c}_r8_fp64_cpu_5h", "cpu_dp", _cfg, ["real.dir=" + _run] + _5H, f"{_run}/wrfout_d01_{_D}", 18000,
         _TS)]
CASES += [
    ("real_sr1_r8_fp64_cpu_5h_rst", "cpu_dp", "configs/real_sr1_r8.ini",
     [f"real.dir={WR}/run_SR1_r8"] + _5H + ["suite.restart_at=9000"], f"{WR}/run_SR1_r8/wrfout_d01_{_D}", 18000, _TS),
    # SR1 resumed from CPPWRF's own restart file at 245 min, between two radiation calls (steps 241 and 251)
    ("real_sr1_r8_fp64_cpu_5h_nrst", "cpu_dp", "configs/real_sr1_r8.ini",
     [f"real.dir={WR}/run_SR1_r8"] + _5H + ["suite.restart_at=14700", "suite.native_restart"],
     f"{WR}/run_SR1_r8/wrfout_d01_{_D}", 18000, _TS),
    # CUDA FP64 (tolerance 0)
    ("real_tw1_r8_fp64_cuda_1h", "cuda_dp", "configs/real_tw1_r8.ini", [f"real.dir={WR}/run_TW1_r8"] + _1H,
     f"{WR}/run_TW1_r8/wrfout_d01_{_D}", 3600, _G),
    ("real_tw4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_tw4_r8.ini", [f"real.dir={WR}/run_TW4_r8"] + _1H,
     f"{WR}/run_TW4_r8/wrfout_d01_{_D}", 3600, _G),
    ("real_sr1_r8_fp64_cuda_5h", "cuda_dp", "configs/real_sr1_r8.ini", [f"real.dir={WR}/run_SR1_r8"] + _5H,
     f"{WR}/run_SR1_r8/wrfout_d01_{_D}", 18000, dict(_G, SWDOWN=0.0, SWNORM=0.0)),
    ("real_sr4_r8_fp64_cuda_5h", "cuda_dp", "configs/real_sr4_r8.ini", [f"real.dir={WR}/run_SR4_r8"] + _5H,
     f"{WR}/run_SR4_r8/wrfout_d01_{_D}", 18000, dict(_G, SWDOWN=0.0, SWNORM=0.0)),
]
# CUDA FP64 twins (tolerance 0) of the CPU cases of TW2, SR2 and SR3 (the options without a CUDA FP64 case), not the
# restarted ones: the same configuration, overrides, reference and variables
_HAS = {c[0].split("_fp64_cuda")[0] for c in CASES if c[1] == "cuda_dp"}
CASES += [(c[0].replace("_fp64_cpu", "_fp64_cuda"), "cuda_dp") + tuple(c[2:]) for c in list(CASES)
          if c[1] == "cpu_dp" and not c[0].endswith("_rst") and c[0].split("_fp64_cpu")[0] not in _HAS]
# FP32 tolerances: about 3x the rel L2 measured on the Colab G4 (TW1 U 7.2e-5 W 3.0e-3 THM 5.0e-7 QV 3.2e-5; TW4 U
# 8.5e-4 W 2.1e-2 THM 1.1e-5 QV 6.3e-4; SR1 U 6.0e-3 W 1.4e-1 THM 7.7e-5 QV 3.9e-3; SR4 U 7.2e-3 W 1.5e-1 THM 9.2e-5
# QV 4.9e-3)
_SP = {"TW1": {"U": 2.2e-4, "W": 9.0e-3, "THM": 1.5e-6, "QV": 1.0e-4},
       "TW4": {"U": 2.6e-3, "W": 6.3e-2, "THM": 3.3e-5, "QV": 1.9e-3},
       "SR1": {"U": 1.8e-2, "W": 4.2e-1, "THM": 2.3e-4, "QV": 1.2e-2},
       "SR4": {"U": 2.2e-2, "W": 4.5e-1, "THM": 2.8e-4, "QV": 1.5e-2}}
for _n, _t in (("TW1", _1H), ("TW4", _1H), ("SR1", _5H), ("SR4", _5H)):
    CASES += [(f"real_{_n.lower()}_fp32_{'1h' if _t is _1H else '5h'}", "cuda", f"configs/real_{_n.lower()}_r8.ini",
               [f"real.dir={WR}/run_{_n}_r8"] + _t, f"{WR}/run_{_n}_r8/wrfout_d01_{_D}", 3600 if _t is _1H else 18000,
               _SP[_n])]

STATUS = [
    ('TW1', 'P1 with YSU topo_wind = 1 (VAR_SSO = VAR**2); restarted at 30 min', '1 h'),
    ('TW2', 'P1 with YSU topo_wind = 2; restarted at 30 min', '1 h'),
    ('TW4', 'P4 with YSU topo_wind = 1 (VAR_SSO = VAR**2)', '1 h'),
    ('SR1', 'P4 with slope_rad = 1 and topo_shading = 1 (radt 10, through sunrise); restarted at 150 min, and at 245 min (between two radiation calls) from its own restart file', '5 h'),
    ('SR2', 'SR1 with shadlen = 100 km', '5 h'),
    ('SR3', 'P4 with slope_rad = 1 alone (radt 1)', '5 h'),
    ('SR4', 'SR1 with the Dudhia shortwave (ra_sw_physics 1)', '5 h'),
]

DATA = {c[0]: 'suite_data.tar,port_a0204_data.tar' for c in CASES}
