# SPDX-License-Identifier: GPL-3.0-or-later
"""Supported options and pairings that no case compared with WRF before (the coverage audit's second wave): WRF's
tropical suite on P4 (TROP: New Tiedtke with WSM6 and RRTMG), WRF's CONUS suite on the two-way nest N3 (CONL:
Thompson, Tiedtke, MYJ / Eta, radt 15 with its second call at 15 min, gwd_opt 1 on d01, fractional_seaice 1, diff_opt
2), the adaptive time step with cudt / bldt > 0 (ADCB on P4; ADGT on P8 with Grell-Freitas-Li and target_cfl 1.0), the
fixed step with cudt 5 / bldt 3 (CB53), MYNN-EDMF with cumulus and its radiation feedback, RRTMG every step (MYKR:
Kain-Fritsch + cu_rad_feedback; MYBR: BMJ + bmj_rad_feedback, icloud_bl 0), the urban canopy model under MYJ (UMYJ),
RRTM LW with Dudhia SW in daylight (RRDD: P4 from 06 UTC, swrad_scat 1.5),
Grell-3D / Grell-Freitas-Li on the N3 nest with cu_rad_feedback (G3N, GFN), the one-way N3 nest with physics (N3OW),
and on P4 use_mp_re 0 (MPRE0), o3input 0 / ghg_input 0 (O3G0), isfflx 0 (ISF0), the centered advection orders 6 / 4
(ADV64), hail_opt 1 with WSM6 and the radar reflectivity (HAIL), isftcflx 1 / iz0tlnd 1 (SFT1).

Each against its WRF v4.8.0 run (tests/verify/refs/cov2_refs.txt in port_cov2_data.tar, cov2b_refs.txt in
port_cov2b_data.tar; tools/colab/colab_wrfref.sh; r8 or r8d8 as the schemes need): 3 steps with history every step
(nests: 3 parent steps, history every minute) and 10 min with history every 5 min (20 min, every 10 min, for the
cudt / bldt cases and CONL), CPU FP64 and CUDA FP64, every variable at tolerance 0 (but those WRF leaves uninitialised:
LH with isfflx 0, wrf_suite.uninitialised).

Not covered here: step_to_output_time = .false. (WRF's adaptive steps then end on no fixed time, and the suite compares
the frame at the case's time); gwd_opt 1 with drag on the nest (the WPS 4.6 met_em files have no ELVMAX, so GWDO gives
no drag, as in WRF; G2 covers the drag on one domain); fractional_seaice with sea ice on a nest (none on N3).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_D6 = "2026-10-12_06:00:00"  # RRDD starts at 06 UTC
_B = {"conl", "adgt", "mykr", "mybr", "rrdd"}  # the references of cov2b_refs.txt (port_cov2b_data.tar)
_DOM = ("d01", "d02")
_BASE = ("U", "V", "W", "PH", "MU", "THM", "QV")
_MP = ("QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")
_SFC = ("TSK", "HFX", "LH", "UST", "PBLH", "RAINC", "RAINNC")


def _tol(*names):
    return {v: 0.0 for v in names}


# (tag, minutes of the long run, history interval of the long run [min], nest, adaptive, tolerances)
_SETS = [
    ("trop", 10, 5, False, False, _tol(*_BASE, *_MP, *_SFC, "RTHCUTEN", "SWDOWN", "GLW")),
    ("conl", 20, 10, True, False, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "TKE_PBL", "TSK", "HFX", "RAINC", "RAINNC",
                                     "SWDOWN", "GLW")),
    ("adcb", 20, 10, False, True, _tol(*_BASE, *_MP, *_SFC, "RTHCUTEN", "RTHBLTEN")),
    ("adgt", 20, 10, False, True, _tol(*_BASE, *_SFC, "RTHCUTEN", "RTHBLTEN")),
    ("cb53", 20, 10, False, False, _tol(*_BASE, *_MP, *_SFC, "RTHCUTEN", "RTHBLTEN")),
    ("mykr", 10, 5, False, False, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "CLDFRA", "RAINC", "RAINNC", "SWDOWN")),
    ("mybr", 10, 5, False, False, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "CLDFRA", "RAINC", "RAINNC", "SWDOWN")),
    ("umyj", 10, 5, False, False, _tol(*_BASE, *_MP, "TSK", "HFX", "LH", "T2", "U10", "TKE_PBL")),
    ("rrdd", 10, 5, False, False, _tol(*_BASE, *_MP, "SWDOWN", "GLW", "RTHRATEN", "TSK")),
    ("g3n", 10, 5, True, False, _tol(*_BASE, *_MP, "RTHCUTEN", "RAINC", "CLDFRA")),
    ("gfn", 10, 5, True, False, _tol(*_BASE, *_MP, "RTHCUTEN", "RAINC", "CLDFRA")),
    ("n3ow", 10, 5, True, False, _tol(*_BASE, *_MP, "TSK", "RAINC", "RAINNC")),
    ("mpre0", 10, 5, False, False, _tol(*_BASE, *_MP, "SWDOWN", "GLW", "CLDFRA")),
    ("o3g0", 10, 5, False, False, _tol(*_BASE, *_MP, "SWDOWN", "GLW", "RTHRATEN")),
    ("isf0", 10, 5, False, False, _tol(*_BASE, *_MP, "HFX", "LH", "TSK", "PBLH")),
    ("adv64", 10, 5, False, False, _tol(*_BASE, *_MP)),
    ("hail", 10, 5, False, False, _tol(*_BASE, *_MP, "RAINNC", "GRAUPELNC", "REFL_10CM")),
    ("sft1", 10, 5, False, False, _tol(*_BASE, "TSK", "HFX", "LH", "UST", "ZNT")),
]


def _refs(run, nest, date=_D):
    return {d: f"{run}/wrfout_{d}_{date}" for d in _DOM} if nest else f"{run}/wrfout_d01_{date}"


def _ntol(tol, nest):
    return {f"{d}.{v}": t for d in _DOM for v, t in tol.items()} if nest else tol


CASES = []
for _tag, _m, _h, _nest, _adapt, _t in _SETS:
    _cfg, _run, _tl = f"configs/real_{_tag}_r8.ini", f"{WR}/run_{_tag.upper()}_r8", _ntol(_t, _nest)
    # the long run: history every _h minutes (out_interval in steps of 60 s; d02 in its 20 s steps; the adaptive
    # time step writes on history_interval_s)
    _long = ["real.dir=" + _run, f"run.run_time={_m * 60}"]
    _long += ([f"time.history_interval_s={_h * 60}"] if _adapt else
              [f"output.out_interval={_h}", f"output.diag_interval={_h}"])
    _long += [f"d02.out_interval={3 * _h}", f"d02.diag_interval={3 * _h}"] if _nest else []
    _dt = _D6 if _tag == "rrdd" else _D
    CASES += [
        (f"real_{_tag}_r8_fp64_cpu", "cpu_dp", _cfg, [], _refs(f"{_run}/steps", _nest, _dt), 180, _tl),
        (f"real_{_tag}_r8_fp64_cpu_{_m}m", "cpu_dp", _cfg, _long, _refs(_run, _nest, _dt), _m * 60, _tl),
        (f"real_{_tag}_r8_fp64_cuda_{_m}m", "cuda_dp", _cfg, _long, _refs(_run, _nest, _dt), _m * 60, _tl),
    ]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('TROP', "WRF's tropical suite on P4 (WSM6, New Tiedtke, RRTMG, YSU, sfclayrev, Noah)", '10 min'),
    ('CONL', "WRF's CONUS suite on the two-way nest N3 (Thompson, Tiedtke, MYJ / Eta, radt 15, gwd_opt 1 on d01, "
             "fractional_seaice 1, diff_opt 2 / km_opt 4)", '20 min'),
    ('ADCB', 'P4 with the adaptive time step and cudt 5 / bldt 2', '20 min'),
    ('ADGT', 'P8 (Grell-Freitas-Li) with the adaptive time step, target_cfl 1.0, cudt 5 / bldt 3', '20 min'),
    ('CB53', 'P4 with cudt 5 and bldt 3', '20 min'),
    ('MYKR', 'MYNN with Kain-Fritsch and cu_rad_feedback, RRTMG every step', '10 min'),
    ('MYBR', 'MYNN with BMJ, bmj_rad_feedback and icloud_bl 0, RRTMG every step', '10 min'),
    ('UMYJ', 'P4 with the urban canopy model under MYJ / Eta', '10 min'),
    ('RRDD', 'P4 in daylight (06 UTC) with RRTM LW and Dudhia SW (swrad_scat 1.5)', '10 min'),
    ('G3N / GFN', 'N3 (two-way nest) with Grell-3D / Grell-Freitas-Li and cu_rad_feedback on both domains', '10 min'),
    ('N3OW', 'N3 one-way (feedback 0) with the P4 physics on both domains', '10 min'),
    ('MPRE0', 'P4 with use_mp_re 0', '10 min'),
    ('O3G0', 'P4 with o3input 0 and ghg_input 0', '10 min'),
    ('ISF0', 'P4 with isfflx 0', '10 min'),
    ('ADV64', 'P4 with the advection orders 6 (horizontal) and 4 (vertical)', '10 min'),
    ('HAIL', 'P4 with hail_opt 1 (WSM6) and do_radar_ref 1', '10 min'),
    ('SFT1', 'P4 with isftcflx 1 and iz0tlnd 1', '10 min'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_cov2b_data.tar' if c[0].split("_")[1] in _B else 'port_cov2_data.tar' for c in CASES}
