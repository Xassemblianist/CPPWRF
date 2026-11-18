# SPDX-License-Identifier: GPL-3.0-or-later
"""Noah-MP v5.1.2 option values (sf_surface_physics = 4, phys/noahmp in WRF 4.8.0) other than the defaults, each
against its WRF v4.8.0 run (wrfref workflow of Xassemblianist/cppwrf-data, tags a14-n1a/b/c, a14-x4a/b, a14-v3a/b):
  N*   NMP1 (P1 column physics + Noah-MP, no radiation or microphysics) with one option changed: 1 h, history every
       10 min (some with wrfrst at 30 min: the *_1h_rst cases restart there), steps/ = 3 steps, history every step;
  X4*  NMP4 = NMP1's inputs with P4's physics (WSM6, Kain-Fritsch, RRTMG with radt 10, diff_opt 1 / km_opt 4):
       X4 1 h (wrfrst at 30 min) and 3 steps, X4L and the option runs 6 h (past sunrise at 03:50 UTC), history
       every hour, X4SNF* 1 h;
  V*   V3 (Alps, 3 km, Thompson, MYJ, Eta, RRTMG; snow, glacier points) with one option changed: 30 min, history
       every 10 min, steps/ = 3 steps (VB = V3 itself, wrfrst at 15 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_T = {"U": 0.0, "V": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0, "TSK": 0.0, "HFX": 0.0, "LH": 0.0,
      "T2": 0.0, "SMOIS": 0.0, "TSLB": 0.0}
_1H = ["run.run_time=3600", "output.out_interval=10", "output.diag_interval=10"]
_6H = ["run.run_time=21600", "output.out_interval=60", "output.diag_interval=60"]
_30M = ["run.run_time=1800", "output.out_interval=100", "output.diag_interval=100"]  # V3: dt = 18 s

# NAME: the option keys ([phys] of the config); RST: the runs with wrfrst at 30 min (V3: 15 min)
_N = {f"NDV{v}": {"dveg": v} for v in (1, 2, 3, 5, 6, 7, 8, 9)}
_N.update({"NCRS2": {"opt_crs": 2}, "NBTR2": {"opt_btr": 2}, "NBTR3": {"opt_btr": 3}})
_N.update({f"NRUN{v}": {"opt_runsrf": v, "opt_runsub": v} for v in (1, 2, 4, 6, 7)})
_N.update({"NRUNX": {"opt_runsrf": 1, "opt_runsub": 2}, "NSFC2": {"opt_sfc": 2}, "NFRZ2": {"opt_frz": 2},
           "NINF2": {"opt_inf": 2}, "NRAD1": {"opt_rad": 1}, "NRAD2": {"opt_rad": 2}, "NALB1": {"opt_alb": 1},
           "NTBOT1": {"opt_tbot": 1}, "NSTC2": {"opt_stc": 2}, "NSTC3": {"opt_stc": 3}})
_N.update({f"NRSF{v}": {"opt_rsf": v} for v in (2, 3, 4)})
_N.update({f"NTKS{v}": {"opt_tksno": v} for v in (2, 3, 4, 5)})
_N.update({"NCMP1": {"opt_compact": 1}, "NSCF2": {"opt_scf": 2}, "NWET1": {"opt_wetland": 1},
           "NSTS": {"soiltstep": 180}})
_NRST = ("NDV2", "NDV5", "NRUN1", "NWET1", "NSTS")

_X = {"X4L": {}}
_X.update({f"X4DV{v}": {"dveg": v} for v in (1, 2, 3, 5, 6, 7, 8, 9)})
_X.update({"X4CRS2": {"opt_crs": 2}, "X4BTR2": {"opt_btr": 2}, "X4BTR3": {"opt_btr": 3}, "X4RAD1": {"opt_rad": 1},
           "X4RAD2": {"opt_rad": 2}, "X4RSF2": {"opt_rsf": 2}, "X4RSF3": {"opt_rsf": 3}, "X4SFC2": {"opt_sfc": 2}})
_XS = {f"X4SNF{v}": {"opt_snf": v} for v in (2, 3, 4, 5)}

_V = {f"VTKS{v}": {"opt_tksno": v} for v in (2, 3, 4, 5)}
_V.update({"VCMP1": {"opt_compact": 1}, "VSCF2": {"opt_scf": 2}, "VGLA2": {"opt_gla": 2}, "VALB1": {"opt_alb": 1},
           "VSTC2": {"opt_stc": 2}, "VSTC3": {"opt_stc": 3}, "VFRZ2": {"opt_frz": 2}, "VINF2": {"opt_inf": 2},
           "VTBOT1": {"opt_tbot": 1}, "VRSF4": {"opt_rsf": 4}, "VDV2": {"dveg": 2}})
_V.update({f"VSNF{v}": {"opt_snf": v} for v in (2, 3, 4, 5)})


def _opts(o):
    return [f"phys.{k}={v}" for k, v in o.items()]


CASES = []
for _n, _o in _N.items():
    _run, _s = f"{WR}/run_{_n}_r8", _n.lower()
    CASES += [
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", "configs/real_nmp1_r8.ini", [f"real.dir={_run}/steps"] + _opts(_o),
         f"{_run}/steps/wrfout_d01_{_D}", 180, _T),
        (f"real_{_s}_r8_fp64_cpu_1h", "cpu_dp", "configs/real_nmp1_r8.ini", [f"real.dir={_run}"] + _opts(_o) + _1H,
         f"{_run}/wrfout_d01_{_D}", 3600, _T),
    ]
    if _n in _NRST:
        CASES.append((f"real_{_s}_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_nmp1_r8.ini",
                      [f"real.dir={_run}"] + _opts(_o) + _1H + ["suite.restart_at=1800"],
                      f"{_run}/wrfout_d01_{_D}", 3600, _T))
_X4 = f"{WR}/run_X4_r8"
_TX = dict(_T, QCLOUD=0.0, QRAIN=0.0, RAINNC=0.0, RAINC=0.0, SWDOWN=0.0, ALBEDO=0.0)
CASES += [
    ("real_x4_r8_fp64_cpu", "cpu_dp", "configs/real_nmp4_r8.ini", [], f"{_X4}/steps/wrfout_d01_{_D}", 180, _TX),
    ("real_x4_r8_fp64_cpu_1h", "cpu_dp", "configs/real_nmp4_r8.ini", [f"real.dir={_X4}"] + _1H,
     f"{_X4}/wrfout_d01_{_D}", 3600, _TX),
    ("real_x4_r8_fp64_cpu_1h_rst", "cpu_dp", "configs/real_nmp4_r8.ini", [f"real.dir={_X4}"] + _1H +
     ["suite.restart_at=1800"], f"{_X4}/wrfout_d01_{_D}", 3600, _TX),
]
for _n, _o in _X.items():
    _run = f"{WR}/run_{_n}_r8"
    CASES.append((f"real_{_n.lower()}_r8_fp64_cpu_6h", "cpu_dp", "configs/real_nmp4_r8.ini",
                  [f"real.dir={_run}"] + _opts(_o) + _6H, f"{_run}/wrfout_d01_{_D}", 21600, _TX))
for _n, _o in _XS.items():
    _run = f"{WR}/run_{_n}_r8"
    CASES += [
        (f"real_{_n.lower()}_r8_fp64_cpu", "cpu_dp", "configs/real_nmp4_r8.ini", [f"real.dir={_run}/steps"] + _opts(_o),
         f"{_run}/steps/wrfout_d01_{_D}", 180, _TX),
        (f"real_{_n.lower()}_r8_fp64_cpu_1h", "cpu_dp", "configs/real_nmp4_r8.ini",
         [f"real.dir={_run}"] + _opts(_o) + _1H, f"{_run}/wrfout_d01_{_D}", 3600, _TX),
    ]
_TV = dict(_T, QCLOUD=0.0, QRAIN=0.0, QSNOW=0.0, RAINNC=0.0, SNOWNC=0.0, SNOW=0.0, SNOWH=0.0, ALBEDO=0.0)
CASES += [
    ("real_vb_r8_fp64_cpu", "cpu_dp", "configs/real_v3_r8.ini", [f"real.dir={WR}/run_VB_r8/steps",
     "run.run_time=54", "output.out_interval=1", "output.diag_interval=1"], f"{WR}/run_VB_r8/steps/wrfout_d01_{_D}",
     54, _TV),
    ("real_vb_r8_fp64_cpu_30min_rst", "cpu_dp", "configs/real_v3_r8.ini", [f"real.dir={WR}/run_VB_r8"] + _30M +
     ["suite.restart_at=900"], f"{WR}/run_VB_r8/wrfout_d01_{_D}", 1800, _TV),
]
for _n, _o in _V.items():
    _run, _s = f"{WR}/run_{_n}_r8", _n.lower()
    CASES += [
        (f"real_{_s}_r8_fp64_cpu", "cpu_dp", "configs/real_v3_r8.ini", [f"real.dir={_run}/steps", "run.run_time=54",
         "output.out_interval=1", "output.diag_interval=1"] + _opts(_o), f"{_run}/steps/wrfout_d01_{_D}", 54, _TV),
        (f"real_{_s}_r8_fp64_cpu_30min", "cpu_dp", "configs/real_v3_r8.ini", [f"real.dir={_run}"] + _opts(_o) + _30M,
         f"{_run}/wrfout_d01_{_D}", 1800, _TV),
    ]
CASES += [
    # CUDA FP64 (exact) and FP32 (tolerances to be measured on the G4)
    ("real_x4_r8_fp64_cuda_1h", "cuda_dp", "configs/real_nmp4_r8.ini", [f"real.dir={_X4}"] + _1H,
     f"{_X4}/wrfout_d01_{_D}", 3600, _TX),
    ("real_x4dv2_r8_fp64_cuda_6h", "cuda_dp", "configs/real_nmp4_r8.ini",
     [f"real.dir={WR}/run_X4DV2_r8", "phys.dveg=2"] + _6H, f"{WR}/run_X4DV2_r8/wrfout_d01_{_D}", 21600, _TX),
    ("real_nrun7_r8_fp64_cuda_1h", "cuda_dp", "configs/real_nmp1_r8.ini",
     [f"real.dir={WR}/run_NRUN7_r8", "phys.opt_runsrf=7", "phys.opt_runsub=7"] + _1H,
     f"{WR}/run_NRUN7_r8/wrfout_d01_{_D}", 3600, _T),
    ("real_x4_fp32_1h", "cuda", "configs/real_nmp4_r8.ini", [f"real.dir={_X4}"] + _1H, f"{_X4}/wrfout_d01_{_D}", 3600,
     {"U": 3e-3, "W": 5e-2, "THM": 3e-5, "QV": 3e-3}),
]

STATUS = [
    ('N* (36 runs)', 'NMP1 with one Noah-MP option changed (dveg 1-3/5-9, opt_crs 2, opt_btr 2/3, opt_runsrf/sub 1/2/4/6/7 '
     'and 1/2, opt_sfc 2, opt_frz 2, opt_inf 2, opt_rad 1/2, opt_alb 1, opt_tbot 1, opt_stc 2/3, opt_rsf 2-4, '
     'opt_tksno 2-5, opt_compact 1, opt_scf 2, opt_wetland 1, soiltstep 180; 5 restarted at 30 min)', '1 h'),
    ('X4', 'NMP1 inputs with P4 physics (WSM6, KF, RRTMG radt 10) and Noah-MP; restarted at 30 min', '1 h'),
    ('X4L, X4* (17 runs)', 'X4 with dveg 1-3/5-9, opt_crs 2, opt_btr 2/3, opt_rad 1/2, opt_rsf 2/3, opt_sfc 2', '6 h'),
    ('X4SNF* (4 runs)', 'X4 with opt_snf 2-5', '1 h'),
    ('VB, V* (16 runs)', 'V3 (Alps snow and glacier) with opt_tksno 2-5, opt_compact 1, opt_scf 2, opt_gla 2, opt_alb 1, '
     'opt_stc 2/3, opt_frz 2, opt_inf 2, opt_tbot 1, opt_rsf 4, dveg 2, opt_snf 2-5; VB restarted at 15 min', '30 min'),
]

# the reference runs' data tars (staged from the wrfref assets of each tag)
_TAR = {n: "port_a14n1_data.tar" for n in ("NDV1", "NDV2", "NDV3", "NDV5", "NDV6", "NDV7", "NDV8", "NDV9", "NCRS2",
                                           "NBTR2", "NBTR3", "NRUN1", "NRUN2", "NRUN4")}
_TAR.update({n: "port_a14n2_data.tar" for n in ("NRUN6", "NRUN7", "NRUNX", "NSFC2", "NFRZ2", "NINF2", "NRAD1", "NRAD2",
                                                "NALB1", "NTBOT1")})
_TAR.update({n: "port_a14n3_data.tar" for n in ("NSTC2", "NSTC3", "NRSF2", "NRSF3", "NRSF4", "NTKS2", "NTKS3", "NTKS4",
                                                "NTKS5", "NCMP1", "NSCF2", "NWET1", "NSTS")})


def _data(c):
    run = c[4].split("/run_")[1].split("_r8/")[0]
    if run.startswith("V"):
        return "suite_data.tar,port_a14v_data.tar,v05_files_links.tar"
    return "suite_data.tar," + _TAR.get(run, "port_a14x_data.tar")


DATA = {c[0]: _data(c) for c in CASES}
