# SPDX-License-Identifier: GPL-3.0-or-later
"""Monotonic scalar advection (moist_adv_opt / scalar_adv_opt / tke_adv_opt = 2: dyn_em/module_advect_em.F
advect_scalar_mono on the last RK stage after rk_update_scalar_pd): MO4 = P4 (WSM6, Kain-Fritsch, RRTMG, YSU, Noah)
with moist and scalar 2, MOK4 = P4 with diff_opt 2 / km_opt 2, moist and TKE 2, MOMY = the MYNN physics (Thompson,
MYNN, qni / qnr / qke_adv scalars) with moist and scalar 2, MON3 = the two-way nest N3 with moist and scalar 2 on
both domains (the 1 h WRF run writes no d02 history: d01, which the nest feeds back, is compared); each against
its WRF v4.8.0 run (real*8; MOMY -r8 -double-8; wrfref workflow of Xassemblianist/cppwrf-data, tag b05-mono, inputs of run_P4_r8 / run_P6_r8 / run_N3h_r8).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "QCLOUD": 0.0, "QRAIN": 0.0,
      "QICE": 0.0, "QSNOW": 0.0, "QGRAUP": 0.0, "T2": 0.0, "RAINNC": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]

CASES = []
for _n, _s in (("MO4", "mo4"), ("MOK4", "mok4"), ("MOMY", "momy")):
    _cfg, _run = f"configs/real_{_s}_r8.ini", f"{WR}/run_{_n}_r8"
    CASES += [
        # 3 steps (history every step) and 1 h
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_s}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _T),
    ]
_N3 = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "QCLOUD", "QRAIN", "QSNOW")}
CASES += [
    ("real_mon3_r8_fp64_cpu", "cpu_dp", "configs/real_mon3_r8.ini", [f"real.dir={WR}/run_MON3_r8/steps",
     f"phys.table_dir={WR}/run_MON3_r8/steps", "run.run_time=180", "output.out_interval=1",
     "output.diag_interval=1", "d02.out_interval=3", "d02.diag_interval=3"],
     {d: f"{WR}/run_MON3_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _N3),
    ("real_mon3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_mon3_r8.ini", [],
     {"d01": f"{WR}/run_MON3_r8/wrfout_d01_{_D}"}, 3600, {k: v for k, v in _N3.items() if k.startswith("d01")}),
    # CUDA FP64 (exact) and FP32 (tolerances to be measured on the G4: P4-like placeholders)
    ("real_mo4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_mo4_r8.ini", [f"real.dir={WR}/run_MO4_r8"] + _1H,
     f"{WR}/run_MO4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QRAIN": 0.0}),
    ("real_mok4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_mok4_r8.ini", [f"real.dir={WR}/run_MOK4_r8"] + _1H,
     f"{WR}/run_MOK4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    ("real_mo4_fp32_1h", "cuda", "configs/real_mo4_r8.ini", [f"real.dir={WR}/run_MO4_r8"] + _1H,
     f"{WR}/run_MO4_r8/wrfout_d01_{_D}", 3600, {"U": 1.9e-3, "W": 5.1e-2, "THM": 2.6e-5, "QV": 1.6e-3}),
]

STATUS = [
    ('MO4', 'P4 with monotonic moist / scalar advection (moist_adv_opt 2, scalar_adv_opt 2)', '1 h'),
    ('MOK4', 'P4 with diff_opt 2 / km_opt 2, monotonic moist and TKE advection (moist_adv_opt 2, tke_adv_opt 2)',
     '1 h'),
    ('MOMY', 'MYNN with monotonic moist / scalar advection (qni, qnr, qke_adv)', '1 h'),
    ('MON3', 'two-way nest N3 with monotonic moist / scalar advection on both domains', '1 h'),
]

DATA = {c[0]: ('suite_data.tar,n3_data.tar,port_b05_data.tar' if '_mon3_' in c[0] else
               'suite_data.tar,port_b05_data.tar') for c in CASES}
