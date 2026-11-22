# SPDX-License-Identifier: GPL-3.0-or-later
"""Supported options and branches that no suite case compared with WRF before (the coverage audit's fifth wave):
YSU PBL with ysu_topdown_pblmix = 0 in daylight (YSUT0: P4 from 06 UTC, real.exe on the P4 met_em files),
Kain-Fritsch with the RH-dependent trigger kfeta_trigger = 3 (KFT3 on P4), Grell-Freitas-Li on P8 with ishallow = 0
(GFSH0) and single-closure selections clos_choice = 1, 4, 7, 10 (GFC1, GFC4, GFC7, GFC10), Grell-3D on P8 with
ishallow = 0 (G3SH0) and clos_choice = 1, 4, 7, 10 (G3C1, G3C4, G3C7, G3C10), New Tiedtke on the two-way nest N3
(NTN3: d01 dx 15 km, d02 dx 5 km < 15 km scale-aware branch), Noah LSM with opt_thcnd = 2 in daylight (THCND2: P4
from 06 UTC), 5-layer thermal diffusion (slab) LSM with ifsnow = 0 in daylight (SLAB0: P4 from 06 UTC, 5 soil
layers), revised MM5 surface layer with turbulent orographic form drag kim_tofd = .true. in daylight (KTOFD: P4 from
06 UTC), real-data full physics with damp_opt = 1 (DMP1 on P4, dampcoef 0.01) and top_lid = .true. (TOPLID on P4),
and Dudhia shortwave with WSM3 in daylight (DUDW3: P4 from 06 UTC, mp_physics = 3 without QI / QS fields).

Each against its WRF v4.8.0 run (tests/verify/refs/cov5_refs.txt in port_cov5_data.tar; tools/colab/colab_wrfref.sh
with --data matrix_data.tar,matrix_nest.tar,matrix_real.tar; r8 or r8d8 as the schemes need): 3 steps
with history every step (nest: 3 parent steps, history every minute) and 10 min with history every 5 min, CPU FP64
and CUDA FP64, every variable at tolerance 0.

Not covered here: kfeta_trigger = 2 (not ported in CPPWRF; tools/wrf2cppwrf.py supports 1 and 3 only); Grell-3D
clos_choice = 16 (WRF v4.8.0 phys/module_cu_g3.F reads xf_ens(nall+16) before setting it); multi-scale Kain-Fritsch
with dx >= 25 km (no base run or met_em dataset with dx >= 25 km exists, and WRF's input_wrf forbids changing dx from
wrfinput); grav_settling = 2 on USGS land use (all available base runs and met_em datasets use MODIS land use);
tmn_update = 1 year-end rollover (all available base runs and met_em datasets start 2026-10-12, JULDAY = 285);
gwd_if_nonhyd = .false. (the gwd_opt = 1 base run G2 is not in the reference job's base-run tars).

Base-run and input-tar dependencies to verify when staging references:
- run_P4_r8 (matrix_data.tar): LANDUSE.TBL, VEGPARM.TBL, SOILPARM.TBL, GENPARM.TBL, RRTMG_LW_DATA, RRTMG_SW_DATA,
  ozone*.formatted, CAMtr_volume_mixing_ratio for YSUT0, KFT3, THCND2, SLAB0, KTOFD, DMP1, TOPLID, DUDW3.
- run_P8_r8 (matrix_data.tar): LANDUSE.TBL, VEGPARM.TBL, SOILPARM.TBL, GENPARM.TBL and wrfinput_d01 / wrfbdy_d01
  for GFSH0, GFC1, GFC4, GFC7, GFC10, G3SH0, G3C1, G3C4, G3C7, G3C10.
- run_N3s_r8 (matrix_nest.tar): wrfinput_d01, wrfinput_d02, wrfbdy_d01 and tables for NTN3.
- real_P4_r8 and real_r8.exe (matrix_real.tar): met_em.d01.2026-10-12_06:00:00.nc and 12:00:00.nc plus real_r8.exe
  for the 06 UTC references (YSUT0, THCND2, SLAB0, KTOFD, DUDW3); VAR > 5 m in wrfinput_d01 for KTOFD; ISLTYP 3 or 4
  points for THCND2; snow-covered points (SNOWC > 0) with TSK / TSLB(1) > 273.16 K for the SLAB0 clamp divergence.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_D6 = "2026-10-12_06:00:00"
_D6_TAGS = {"ysut0", "thcnd2", "slab0", "ktofd", "dudw3"}  # daylight runs (06 UTC, real.exe on real_P4_r8)
_DOM = ("d01", "d02")
_BASE = ("U", "V", "W", "PH", "MU", "THM", "QV")
_MP = ("QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")
_SFC = ("TSK", "HFX", "LH", "UST", "PBLH", "RAINC", "RAINNC")
_P8_SFC = ("TSK", "HFX", "LH", "UST", "PBLH", "RAINC", "RTHCUTEN")


def _tol(*names):
    return {v: 0.0 for v in names}


# (tag, minutes of the long run, history interval of the long run [min], nest, tolerances)
_SETS = [
    ("ysut0", 10, 5, False, _tol(*_BASE, *_MP, *_SFC, "RTHBLTEN", "SWDOWN", "GLW")),
    ("kft3", 10, 5, False, _tol(*_BASE, *_MP, *_SFC, "RTHCUTEN")),
    ("gfsh0", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("gfc1", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("gfc4", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("gfc7", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("gfc10", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("g3sh0", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("g3c1", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("g3c4", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("g3c7", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("g3c10", 10, 5, False, _tol(*_BASE, *_P8_SFC)),
    ("ntn3", 10, 5, True, _tol(*_BASE, *_MP, *_SFC, "RTHCUTEN", "SWDOWN", "GLW")),
    ("thcnd2", 10, 5, False, _tol(*_BASE, *_MP, *_SFC, "TSLB", "SMOIS", "SWDOWN")),
    ("slab0", 10, 5, False, _tol(*_BASE, *_MP, *_SFC, "TSLB", "SWDOWN", "GLW")),
    ("ktofd", 10, 5, False, _tol(*_BASE, *_MP, "TSK", "HFX", "LH", "UST", "ZNT", "PBLH", "SWDOWN")),
    ("dmp1", 10, 5, False, _tol(*_BASE, *_MP, *_SFC)),
    ("toplid", 10, 5, False, _tol(*_BASE, *_MP, *_SFC)),
    ("dudw3", 10, 5, False, _tol(*_BASE, "QCLOUD", "QRAIN", *_SFC, "SWDOWN", "GLW", "RTHRATEN")),
]


def _refs(run, nest, date=_D):
    return {d: f"{run}/wrfout_{d}_{date}" for d in _DOM} if nest else f"{run}/wrfout_d01_{date}"


def _ntol(tol, nest):
    return {f"{d}.{v}": t for d in _DOM for v, t in tol.items()} if nest else tol


CASES = []
for _tag, _m, _h, _nest, _t in _SETS:
    _cfg, _run, _tl = f"configs/real_{_tag}_r8.ini", f"{WR}/run_{_tag.upper()}_r8", _ntol(_t, _nest)
    _long = ["real.dir=" + _run, f"run.run_time={_m * 60}", f"output.out_interval={_h}", f"output.diag_interval={_h}"]
    _long += [f"d02.out_interval={3 * _h}", f"d02.diag_interval={3 * _h}"] if _nest else []
    _dt = _D6 if _tag in _D6_TAGS else _D
    CASES += [
        (f"real_{_tag}_r8_fp64_cpu", "cpu_dp", _cfg, [], _refs(f"{_run}/steps", _nest, _dt), 180, _tl),
        (f"real_{_tag}_r8_fp64_cpu_{_m}m", "cpu_dp", _cfg, _long, _refs(_run, _nest, _dt), _m * 60, _tl),
        (f"real_{_tag}_r8_fp64_cuda_{_m}m", "cuda_dp", _cfg, _long, _refs(_run, _nest, _dt), _m * 60, _tl),
    ]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('YSUT0', 'P4 in daylight (06 UTC) with YSU ysu_topdown_pblmix = 0', '10 min'),
    ('KFT3', 'P4 with Kain-Fritsch kfeta_trigger = 3 (RH-dependent trigger)', '10 min'),
    ('GFSH0', 'P8 (Grell-Freitas-Li) with ishallow = 0', '10 min'),
    ('GFC1', 'P8 (Grell-Freitas-Li) with clos_choice = 1 (Grell stability closure)', '10 min'),
    ('GFC4', 'P8 (Grell-Freitas-Li) with clos_choice = 4 (low-level vertical velocity closure)', '10 min'),
    ('GFC7', 'P8 (Grell-Freitas-Li) with clos_choice = 7 (moisture-convergence closure)', '10 min'),
    ('GFC10', 'P8 (Grell-Freitas-Li) with clos_choice = 10 (CAPE / Arakawa-Schubert closure)', '10 min'),
    ('G3SH0', 'P8 with Grell-3D (cu_physics = 5) and ishallow = 0', '10 min'),
    ('G3C1', 'P8 with Grell-3D (cu_physics = 5) and clos_choice = 1 (Grell stability closure)', '10 min'),
    ('G3C4', 'P8 with Grell-3D (cu_physics = 5) and clos_choice = 4 (low-level vertical velocity closure)', '10 min'),
    ('G3C7', 'P8 with Grell-3D (cu_physics = 5) and clos_choice = 7 (moisture-convergence closure)', '10 min'),
    ('G3C10', 'P8 with Grell-3D (cu_physics = 5) and clos_choice = 10 (CAPE / Arakawa-Schubert closure)', '10 min'),
    ('NTN3', 'N3 (two-way nest, d02 dx 5 km < 15 km) with New Tiedtke (cu_physics = 16) on both domains', '10 min'),
    ('THCND2', 'P4 in daylight (06 UTC) with Noah opt_thcnd = 2 (McCumber-Pielke thermal conductivity)', '10 min'),
    ('SLAB0', 'P4 in daylight (06 UTC) with the 5-layer thermal diffusion (slab) LSM and ifsnow = 0', '10 min'),
    ('KTOFD', 'P4 in daylight (06 UTC) with sfclayrev kim_tofd = true (turbulent orographic form drag)', '10 min'),
    ('DMP1', 'P4 with damp_opt = 1 and dampcoef 0.01 (upper diffusive damping, cal_dampkm)', '10 min'),
    ('TOPLID', 'P4 with top_lid = true (rigid-lid top boundary condition)', '10 min'),
    ('DUDW3', 'P4 in daylight (06 UTC) with WSM3 (mp_physics = 3), RRTMG LW and Dudhia SW (qi absent)', '10 min'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_cov5_data.tar' for c in CASES}
