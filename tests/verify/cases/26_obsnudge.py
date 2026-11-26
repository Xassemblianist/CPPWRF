# SPDX-License-Identifier: GPL-3.0-or-later
"""Observation nudging, obs_nudge_opt = 1 (phys/module_fddaobs_driver.F, module_fddaobs_rtfdda.F): OB1 ... OB9 on the
P4 / P9 physics and OBN on the N3 nest, each against its WRF v4.8.0 real*8 run with the synthetic OBS_DOMAIN101 of
tests/data/obs (tools/obs_nudging/make_obs_domain.py; the WRF runs: tools/obs_nudging/refs_b04.txt).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0}
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV")}
# the [fdda] options of OB2 ... OB4 (the namelists of their WRF runs: tools/obs_nudging/refs_b04.txt)
_OB2 = ["fdda.obs_ionf=1", "fdda.obs_sfc_scheme_horiz=1", "fdda.obs_sfc_scheme_vert=1", "fdda.obs_no_pbl_nudge_uv=1",
        "fdda.obs_no_pbl_nudge_t=1", "fdda.obs_no_pbl_nudge_q=1", "fdda.obs_scl_neg_qv_innov=1", "fdda.obs_sfcfact=1.5",
        "fdda.obs_sfcfacr=0.8", "fdda.obs_dpsmx=5", "fdda.obs_max_sndng_gap=10"]
_OB3 = ["fdda.fdda_end=60", "fdda.obs_coef_wind=4.E-4", "fdda.obs_coef_temp=8.E-4", "fdda.obs_coef_mois=1.E-3",
        "fdda.obs_rinxy=150", "fdda.obs_twindo=1.0", "fdda.obs_ionf=3", "fdda.obs_idynin=1", "fdda.obs_dtramp=30",
        "fdda.obs_scl_neg_qv_innov=1", "fdda.obs_nudgezfullr1_uv=-4900", "fdda.obs_nudgezrampr1_t=-4800",
        "fdda.obs_nudgezfullr1_q=0", "fdda.obs_nudgezfullmin=0", "fdda.obs_nudgezmax=2000"]
_OB4 = ["fdda.obs_nudge_temp=0", "fdda.obs_coef_temp=0", "fdda.obs_nudge_mois=0", "fdda.obs_coef_mois=0",
        "fdda.obs_coef_wind=1.E-3", "fdda.obs_rinxy=300", "fdda.obs_twindo=0.5", "fdda.obs_ionf=1", "fdda.obs_sfc_scheme_horiz=1", "fdda.obs_sfcfacr=2"]
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]


def _ob(n, sets):
    """3 steps (history every step) and 1 h of OB<n> = OB1's configuration with `sets`."""
    return [(f"real_ob{n}_r8_fp64_cpu", "cpu_dp", "configs/real_ob1_r8.ini",
             ["real.dir=" + f"{WR}/run_OB{n}_r8/steps"] + sets, f"{WR}/run_OB{n}_r8/steps/wrfout_d01_{_D}", 180, _T),
            (f"real_ob{n}_r8_fp64_cpu_1h", "cpu_dp", "configs/real_ob1_r8.ini",
             ["real.dir=" + f"{WR}/run_OB{n}_r8"] + sets + _1H, f"{WR}/run_OB{n}_r8/wrfout_d01_{_D}", 3600, _T)]


CASES = [
    # OB1 = P4 with obs nudging of wind, temperature and moisture (6e-4 s-1, rinxy 240 km, a 40 min window, errob every
    # 2 steps, until 6 h): 3 steps, 1 h
    ("real_ob1_r8_fp64_cpu", "cpu_dp", "configs/real_ob1_r8.ini", [],
     f"{WR}/run_OB1_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_ob1_r8_fp64_cpu_1h", "cpu_dp", "configs/real_ob1_r8.ini", ["real.dir=" + f"{WR}/run_OB1_r8"] + _1H,
     f"{WR}/run_OB1_r8/wrfout_d01_{_D}", 3600, _T),
    # OB1R = OB1 with obs_ionf 4, 1 h restarted at 30 min against WRF's run restarted there (run_OB1R_r8,
    # run.segment=1800, restart_interval 30): the resumed run reads OBS_DOMAIN101 again from the restart time (in4dob
    # at ktaur) and calls errob at its first step (idarst; ktaur 30 is no multiple of obs_ionf), the model levels of
    # the reports recomputed there
    ("real_ob1r_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_ob1_r8.ini",
     ["real.dir=" + f"{WR}/run_OB1R_r8", "fdda.obs_ionf=4"] + _1H + ["suite.restart_at=1800"],
     f"{WR}/run_OB1R_r8/wrfout_d01_{_D}", 3600, _T),
    # FP32 GPU against the real*8 run (no real*4 WRF run of OB1): bounds of the P4 FP32 case's order
    ("real_ob1_fp32_1h", "cuda", "configs/real_ob1_r8.ini", ["real.dir=" + f"{WR}/run_OB1_r8"] + _1H,
     f"{WR}/run_OB1_r8/wrfout_d01_{_D}", 3600, {"U": 1e-2, "W": 2e-1, "THM": 1e-4, "QV": 1e-2}),
]
# OB2 = OB1 with the MM5 surface schemes (obs_sfc_scheme_horiz / vert 1), no nudging in the PBL, the scaled negative
# qv innovations, errob every step, sfcfact 1.5 / sfcfacr 0.8, dpsmx 5 hPa, a 10 hPa sounding gap
CASES += _ob(2, _OB2)
# OB3 = OB1 with dynamic initialisation (idynin 1, dtramp 30 min, fdda_end 60 min), the PBL-relative and negative
# vertical ramps, nudgezfullmin 0, nudgezmax 2000 m, a 1 h window, rinxy 150 km, other coefficients, errob every 3 steps
CASES += _ob(3, _OB3)
# OB4 = wind nudging only (1e-3 s-1, rinxy 300 km, a 30 min window) with the MM5 horizontal weights, sfcfacr 2
CASES += _ob(4, _OB4)
CASES += [
    # OB9 = P9 (MYJ surface layer: fddaobs_driver's REGIME from PBLH; no diffusion: phy_prep's z) with OB1's nudging
    ("real_ob9_r8_fp64_cpu", "cpu_dp", "configs/real_ob9_r8.ini", [],
     f"{WR}/run_OB9_r8/steps/wrfout_d01_{_D}", 180, _T),
    ("real_ob9_r8_fp64_cpu_1h", "cpu_dp", "configs/real_ob9_r8.ini", ["real.dir=" + f"{WR}/run_OB9_r8"] + _1H,
     f"{WR}/run_OB9_r8/wrfout_d01_{_D}", 3600, _T),
    # OBN = N3 with obs nudging on both domains (d02: rinxy 80 km, domain 1's window, OBS_DOMAIN201)
    ("real_obn_r8_fp64_cpu", "cpu_dp", "configs/real_obn_r8.ini", [],
     {d: f"{WR}/run_OBN_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_obn_r8_fp64_cpu_1h", "cpu_dp", "configs/real_obn_r8.ini",
     ["real.dir=" + f"{WR}/run_OBN_r8", "phys.table_dir=" + f"{WR}/run_OBN_r8", "run.run_time=3600",
      "output.out_interval=10", "output.diag_interval=10", "d02.out_interval=30", "d02.diag_interval=30"],
     {d: f"{WR}/run_OBN_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 3600, _TN),
]
# CUDA FP64 twins (tolerance 0) of every CPU case but the restarted one: the same configuration, overrides, reference
# and variables, kind cuda_dp
CASES += [(c[0].replace("_fp64_cpu", "_fp64_cuda"), "cuda_dp") + tuple(c[2:]) for c in list(CASES)
          if c[1] == "cpu_dp" and "restart_at" not in " ".join(c[3])]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('OB1', 'P4 with observation nudging (obs_nudge_opt 1, synthetic OBS_DOMAIN101; OB1R: obs_ionf 4, restarted at 30 min)', '1 h'),
    ('OB2', 'OB1 with the MM5 surface schemes, no PBL nudging, scaled qv innovations, sfcfact / sfcfacr, dpsmx', '1 h'),
    ('OB3', 'OB1 with dynamic initialisation (idynin, dtramp 30 min), regime-based vertical ramps, nudgezmax', '1 h'),
    ('OB4', 'OB1 with wind nudging only, MM5 horizontal weights', '1 h'),
    ('OB9', 'P9 (MYJ surface layer, REGIME from PBLH) with observation nudging', '1 h'),
    ('OBN', 'N3 with observation nudging on both domains (OBS_DOMAIN201)', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order
DATA = {c[0]: 'port_b04_data.tar' for c in CASES}
