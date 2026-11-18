# SPDX-License-Identifier: GPL-3.0-or-later
"""The NSSL 2-moment microphysics (phys/module_mp_nssl_2mom.F, mp_physics = 18 with its defaults: hail, CCN, graupel
and hail volume; mp_physics = 17 is the same): N18 = P4 (Kain-Fritsch with cu_used, RRTMG with NSSL's effective
radii, YSU, Noah) and N18V = V3 (3 km, no cumulus, Noah-MP with HAILNCV) with NSSL instead of WSM6 / Thompson; N17 =
P4 with mp_physics = 17 in WRF's namelist (check_a_mundo makes it 18), against the N18 configuration. WRF v4.8.0
real*8 runs of the -fdefault-double-8 build (wrfref workflow of Xassemblianist/cppwrf-data, tag c08-nssl: the inputs
of run_P4_r8 / run_V3_r8; N18's wrfrst at 30 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0,
      "QICE": 0.0, "QSNOW": 0.0, "QGRAUP": 0.0, "QHAIL": 0.0, "QNDROP": 0.0, "QNRAIN": 0.0, "QNICE": 0.0,
      "QNSNOW": 0.0, "QNGRAUPEL": 0.0, "QNHAIL": 0.0, "QNCCN": 0.0, "QVGRAUPEL": 0.0, "QVHAIL": 0.0,
      "REFL_10CM": 0.0, "RAINNC": 0.0, "SNOWNC": 0.0, "GRAUPELNC": 0.0, "HAILNC": 0.0}
_TR = dict(_T, RE_CLOUD=0.0, RE_ICE=0.0, RE_SNOW=0.0)
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_C, _R = "configs/real_p4n18_r8.ini", f"{WR}/run_N18_r8"
_CV, _RV = "configs/real_v3n18_r8.ini", f"{WR}/run_N18V_r8"

CASES = [
    # N18: 3 steps (history every step), 1 h, 1 h restarted at 30 min (its wrfrst identical to WRF's)
    ("real_p4n18_r8_fp64_cpu", "cpu_dp", _C, [], f"{_R}/steps/wrfout_d01_{_D}", 180, _TR),
    ("real_p4n18_r8_fp64_cpu_1h", "cpu_dp", _C, ["real.dir=" + _R] + _1H, f"{_R}/wrfout_d01_{_D}", 3600, _TR),
    ("real_p4n18_r8_fp64_cpu_1h_rst", "cpu_dp", _C, ["real.dir=" + _R] + _1H + ["suite.restart_at=1800"],
     f"{_R}/wrfout_d01_{_D}", 3600, _TR),
    # N17: WRF's mp_physics = 17 run, 3 steps, with the translated configuration (mp_physics = 18)
    ("real_p4n17_r8_fp64_cpu", "cpu_dp", _C, ["real.dir=" + f"{WR}/run_N17_r8/steps"],
     f"{WR}/run_N17_r8/steps/wrfout_d01_{_D}", 180, _TR),
    # N18V: 3 steps (history every step), 30 min
    ("real_v3n18_r8_fp64_cpu", "cpu_dp", _CV, ["real.dir=" + f"{_RV}/steps", "run.run_time=54",
                                               "output.out_interval=1", "output.diag_interval=1"],
     f"{_RV}/steps/wrfout_d01_{_D}", 54, _TR),
    ("real_v3n18_r8_fp64_cpu_30min", "cpu_dp", _CV, [], f"{_RV}/wrfout_d01_{_D}", 1800, _TR),
    # CUDA FP64 (exact) and FP32 (provisional tolerances: P4's and V3's microphysics cases, to be measured on the G4)
    ("real_p4n18_r8_fp64_cuda_1h", "cuda_dp", _C, ["real.dir=" + _R] + _1H, f"{_R}/wrfout_d01_{_D}", 3600,
     {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QHAIL": 0.0, "QNCCN": 0.0}),
    ("real_v3n18_r8_fp64_cuda_30min", "cuda_dp", _CV, [], f"{_RV}/wrfout_d01_{_D}", 1800,
     {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QHAIL": 0.0, "QNCCN": 0.0}),
    ("real_p4n18_fp32_1h", "cuda", _C, ["real.dir=" + _R] + _1H, f"{_R}/wrfout_d01_{_D}", 3600,
     {"U": 1.3e-2, "W": 0.4, "THM": 1.7e-4, "QV": 6e-3}),
]

STATUS = [
    ('N18', 'P4 with NSSL 2-moment (mp_physics 18: hail, CCN, graupel / hail volume) and KF; restarted at 30 min',
     '1 h'),
    ('N17', 'P4 with WRF mp_physics 17 (= 18 with its defaults)', '3 steps'),
    ('N18V', 'V3 (3 km, Noah-MP, no cumulus) with NSSL 2-moment (mp_physics 18)', '30 min'),
]

DATA = {c[0]: 'suite_data.tar,port_c08_data.tar' for c in CASES}
