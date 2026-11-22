"""Coverage wave 4: real-data cases with more vertical levels (WRF v4.8.0 r8 references from
tests/verify/refs/cov4_refs.txt, data port_cov4_data.tar).

Every other real-data case has 44 layers, so the GPU kernel variants chosen for nz > 48 (and RRTMG LW nlayers > 60,
RRTMG SW nlayers > 48) never ran on real data. These cases repeat eight physics sets with e_vert 58 (57 layers: the
64-level variants) and e_vert 90 (89 layers: the 128-level variants); real.exe (real_r8.exe on the P4 met_em files)
makes the input with WRF's default level generation. Physics as in P4 (WSM6, KF, YSU, Noah, RRTMG), MOGF (Morrison,
Grell-Freitas-Li), NSMK (NSSL 2-moment, multi-scale KF), MYF10 (Thompson, MYNN surface layer and PBL, RRTMG every
step, from 06 UTC), NMPBL / NMPAC / NMPQN (Noah-MP with BouLac, ACM2, QNSE + EDKF, from 06 UTC) and GS2J (MYJ PBL,
Eta surface layer, grav_settling 2). Each: 3 steps and 10 min on the CPU, 10 min with CUDA FP64, tolerance 0.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_D6 = "2026-10-12_06:00:00"
_BASE = ("U", "V", "W", "PH", "MU", "THM", "QV")
_MP = ("QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")
_SFC = ("TSK", "HFX", "LH", "UST", "PBLH", "RAINC", "RAINNC")
_NSSL = ("QHAIL", "QNDROP", "QNRAIN", "QNICE", "QNSNOW", "QNGRAUPEL", "QNHAIL", "QNCCN", "QVGRAUPEL", "QVHAIL",
         "REFL_10CM", "RAINC", "RAINNC", "RTHCUTEN", "RE_CLOUD", "RE_ICE", "RE_SNOW")


def _tol(*names):
    return {v: 0.0 for v in names}


# (base, start, tolerances)
_BASES = [
    ("p4", _D, _tol(*_BASE, *_MP, *_SFC, "SWDOWN", "GLW", "RTHCUTEN")),
    ("mogf", _D, _tol(*_BASE, *_MP, "QNICE", "QNSNOW", "QNRAIN", "QNGRAUPEL", "RAINC", "RAINNC", "RTHCUTEN")),
    ("nsmk", _D, _tol(*_BASE, *_MP, *_NSSL)),
    ("myf10", _D6, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "TSK", "HFX", "LH", "UST", "ZNT", "SWDOWN", "GLW")),
    ("nmpbl", _D6, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL", "EL_PBL", "T2", "SMOIS", "TSLB", "SWDOWN")),
    ("nmpac", _D6, _tol(*_BASE, *_MP, *_SFC, "T2", "SMOIS", "TSLB", "RTHBLTEN", "SWDOWN")),
    ("nmpqn", _D6, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL", "T2", "SMOIS", "TSLB", "SWDOWN")),
    ("gs2j", _D, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL")),
]
_LEVELS = (58, 90)

CASES = []
for _lv in _LEVELS:
    for _b, _dt, _tl in _BASES:
        _tag = f"l{_lv}{_b}"
        _cfg, _run = f"configs/real_{_tag}_r8.ini", f"{WR}/run_{_tag.upper()}_r8"
        _long = ["real.dir=" + _run, "run.run_time=600", "output.out_interval=5", "output.diag_interval=5"]
        CASES += [
            (f"real_{_tag}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_dt}", 180, _tl),
            (f"real_{_tag}_r8_fp64_cpu_10m", "cpu_dp", _cfg, _long, f"{_run}/wrfout_d01_{_dt}", 600, _tl),
            (f"real_{_tag}_r8_fp64_cuda_10m", "cuda_dp", _cfg, _long, f"{_run}/wrfout_d01_{_dt}", 600, _tl),
        ]

_DESC = {"p4": "P4", "mogf": "MOGF (Morrison, Grell-Freitas-Li)", "nsmk": "NSMK (NSSL 2-moment, multi-scale KF)",
         "myf10": "MYF10 (Thompson, MYNN, RRTMG every step; 06 UTC)", "nmpbl": "NMPBL (Noah-MP, BouLac; 06 UTC)",
         "nmpac": "NMPAC (Noah-MP, ACM2; 06 UTC)", "nmpqn": "NMPQN (Noah-MP, QNSE + EDKF; 06 UTC)",
         "gs2j": "GS2J (MYJ, Eta surface layer, grav_settling 2)"}
# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [(f"L{_lv}{_b.upper()}", f"{_DESC[_b]} with {_lv - 1} layers (e_vert {_lv}, real.exe)", "10 min")
          for _lv in _LEVELS for _b, _, _ in _BASES]

# the data tars (tests/verify/vm_data.txt) each case reads
DATA = {c[0]: 'port_cov4_data.tar' for c in CASES}
