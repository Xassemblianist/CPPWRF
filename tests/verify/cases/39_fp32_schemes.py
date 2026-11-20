# SPDX-License-Identifier: GPL-3.0-or-later
"""CPU FP32 against WRF real*4, bit for bit, for more column schemes: KESSKF (Kessler + Kain-Fritsch), ACM7 (ACM2 PBL
over the Pleim-Xiu surface layer), Q4 (QNSE PBL and surface layer), T6 (Tiedtke), P7 (New Tiedtke), DUD4 (P4 with
the Dudhia shortwave, ra_sw_physics 1), SLAB1 (the 5-layer thermal diffusion land surface). Each reference
(fp32id4_data.tar) is WRF v4.8.0 real*4's run on the workstation from run_P4_cpu_sp's (KESSKF, ACM7, Q4, DUD4) or
run_P2's (T6, P7) real*4 inputs, or (SLAB1) real.exe real*4's 5-layer input from run_P1's met_em files, with the
configuration's namelist; steps/: 3 minutes, history every step; 1 h, history every 10 minutes.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_V = ("U", "V", "W", "PH", "MU", "THM", "QV")

CASES = []
for _n, _c, _cfg, _x in (("KESSKF", "real_kesskf", "real_kesskf_r8", []), ("ACM7", "real_acm7", "real_acm7_r8", []),
                         ("Q4", "real_q4", "real_q4_r8", []), ("T6", "real_t6", "real_t6_r8", []),
                         ("P7", "real_p7", "real_p7_r8", []), ("SLAB1", "real_slab1", "real_slab1_r8", []),
                         ("DUD", "real_dud4", "real_p4_r8", ["phys.ra_sw_physics=1"])):
    _run = f"{WR}/run_{_n}_cpu_sp"
    CASES += [
        (f"{_c}_fp32_cpu", "cpu_sp", f"configs/{_cfg}.ini", _x + ["real.dir=" + f"{_run}/steps", "run.run_time=180",
                                                                  "output.out_interval=1", "output.diag_interval=1"],
         f"{_run}/steps/wrfout_d01_{_D}", 180, {v: 0.0 for v in _V}),
        (f"{_c}_fp32_cpu_1h", "cpu_sp", f"configs/{_cfg}.ini", _x + ["real.dir=" + _run] + _1H,
         f"{_run}/wrfout_d01_{_D}", 3600, {v: 0.0 for v in _V}),
    ]

DATA = {c[0]: 'fp32id4_data.tar' for c in CASES}
