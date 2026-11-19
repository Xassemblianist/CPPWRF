# SPDX-License-Identifier: GPL-3.0-or-later
"""Monotonic and WENO advection. Monotonic scalar advection (moist_adv_opt / scalar_adv_opt / tke_adv_opt = 2: dyn_em/module_advect_em.F
advect_scalar_mono on the last RK stage after rk_update_scalar_pd): MO4 = P4 (WSM6, Kain-Fritsch, RRTMG, YSU, Noah)
with moist and scalar 2, MOK4 = P4 with diff_opt 2 / km_opt 2, moist and TKE 2, MOMY = the MYNN physics (Thompson,
MYNN, qni / qnr / qke_adv scalars) with moist and scalar 2, MON3 = the two-way nest N3 with moist and scalar 2 on
both domains (the 1 h WRF run writes no d02 history: d01, which the nest feeds back, is compared); each against
its WRF v4.8.0 run (real*8; MOMY -r8 -double-8; wrfref workflow of Xassemblianist/cppwrf-data, tag b05-mono, inputs of run_P4_r8 / run_P6_r8 / run_N3h_r8).
WENO (advect_scalar_weno / _wenopd for adv_opt 3 / 4, theta with advect_scalar_weno; advect_weno_u / _v / _w for
momentum_adv_opt 3; all on the last RK stage): WE4 = P4 with moist and scalar 3, WP4 = 4 / 4, WM4 = P4 with
momentum_adv_opt 3, WK4 = MOK4's TKE run with moist 3 and TKE 4, WMY = MYNN with moist 4 and scalar 3, WN3 = the
nest N3 with moist / scalar 4 and momentum 3 on both domains; WRF -r8 -double-8 (the WENO routines' DOUBLE
PRECISION locals are real*16 in the plain -r8 build), tag b05-weno, data port_b05w_data.tar.

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
    # CUDA FP64 (exact) and FP32 (about 3x the G4 measurement at 37213be: MO4 U 3.2e-4 W 9.4e-3 THM 6.5e-6 QV 3.2e-4;
    # WM4 U 2.9e-4 W 7.8e-3 THM 8.7e-6 QV 2.6e-4)
    ("real_mo4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_mo4_r8.ini", [f"real.dir={WR}/run_MO4_r8"] + _1H,
     f"{WR}/run_MO4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QRAIN": 0.0}),
    ("real_mok4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_mok4_r8.ini", [f"real.dir={WR}/run_MOK4_r8"] + _1H,
     f"{WR}/run_MOK4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    ("real_mo4_fp32_1h", "cuda", "configs/real_mo4_r8.ini", [f"real.dir={WR}/run_MO4_r8"] + _1H,
     f"{WR}/run_MO4_r8/wrfout_d01_{_D}", 3600, {"U": 1.0e-3, "W": 2.8e-2, "THM": 2.0e-5, "QV": 1.0e-3}),
]

for _n, _s in (("WE4", "we4"), ("WP4", "wp4"), ("WM4", "wm4"), ("WK4", "wk4"), ("WMY", "wmy")):
    _cfg, _run = f"configs/real_{_s}_r8.ini", f"{WR}/run_{_n}_r8"
    CASES += [
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_s}_r8_fp64_cpu_1h", "cpu_dp", _cfg, ["real.dir=" + _run] + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _T),
    ]
CASES += [
    ("real_wn3_r8_fp64_cpu", "cpu_dp", "configs/real_wn3_r8.ini", [f"real.dir={WR}/run_WN3_r8/steps",
     f"phys.table_dir={WR}/run_WN3_r8/steps", "run.run_time=180", "output.out_interval=1",
     "output.diag_interval=1", "d02.out_interval=3", "d02.diag_interval=3"],
     {d: f"{WR}/run_WN3_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _N3),
    ("real_wn3_r8_fp64_cpu_1h", "cpu_dp", "configs/real_wn3_r8.ini", [],
     {d: f"{WR}/run_WN3_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _N3),
    ("real_we4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_we4_r8.ini", [f"real.dir={WR}/run_WE4_r8"] + _1H,
     f"{WR}/run_WE4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QRAIN": 0.0}),
    ("real_wm4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_wm4_r8.ini", [f"real.dir={WR}/run_WM4_r8"] + _1H,
     f"{WR}/run_WM4_r8/wrfout_d01_{_D}", 3600, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
    ("real_wm4_fp32_1h", "cuda", "configs/real_wm4_r8.ini", [f"real.dir={WR}/run_WM4_r8"] + _1H,
     f"{WR}/run_WM4_r8/wrfout_d01_{_D}", 3600, {"U": 9.0e-4, "W": 2.4e-2, "THM": 2.6e-5, "QV": 8.0e-4}),
]
# CUDA FP64 twins (tolerance 0) of the CPU cases of MOMY, MON3, WP4, WK4, WMY and WN3: the same configuration,
# overrides, WRF reference and variables, kind cuda_dp
CASES += [(c[0].replace("_fp64_cpu", "_fp64_cuda"), "cuda_dp") + tuple(c[2:]) for c in list(CASES)
          if c[1] == "cpu_dp" and c[0].split("_")[1] in ("momy", "mon3", "wp4", "wk4", "wmy", "wn3")]

STATUS = [
    ('MO4', 'P4 with monotonic moist / scalar advection (moist_adv_opt 2, scalar_adv_opt 2)', '1 h'),
    ('MOK4', 'P4 with diff_opt 2 / km_opt 2, monotonic moist and TKE advection (moist_adv_opt 2, tke_adv_opt 2)',
     '1 h'),
    ('MOMY', 'MYNN with monotonic moist / scalar advection (qni, qnr, qke_adv)', '1 h'),
    ('MON3', 'two-way nest N3 with monotonic moist / scalar advection on both domains', '1 h'),
    ('WE4', 'P4 with WENO moist / scalar advection (moist_adv_opt 3, scalar_adv_opt 3)', '1 h'),
    ('WP4', 'P4 with positive-definite WENO moist / scalar advection (4, 4)', '1 h'),
    ('WM4', 'P4 with WENO momentum advection (momentum_adv_opt 3)', '1 h'),
    ('WK4', 'P4 with km_opt 2, WENO moist (3) and positive-definite WENO TKE (tke_adv_opt 4) advection', '1 h'),
    ('WMY', 'MYNN with positive-definite WENO moist (4) and WENO scalar (3) advection', '1 h'),
    ('WN3', 'two-way nest N3 with positive-definite WENO moist / scalar and WENO momentum advection', '1 h'),
]

def _data(name):
    tar = 'port_b05w_data.tar' if name.startswith('real_w') else 'port_b05_data.tar'
    return f'suite_data.tar,n3_data.tar,{tar}' if '_mon3_' in name or '_wn3_' in name else f'suite_data.tar,{tar}'


DATA = {c[0]: _data(c[0]) for c in CASES}
