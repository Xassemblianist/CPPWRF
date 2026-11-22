# SPDX-License-Identifier: GPL-3.0-or-later
"""Supported options and pairings that no case compared with WRF before (the coverage audit's third wave): LES-like
no-PBL with diff_opt 2 / km_opt 2 and the Noah LSM on P4 (LESK2), Noah-MP in daylight (06 UTC, real.exe inputs from
the P4 met_em files) paired with MYNN (NMPMY: P6 + RRTMG radt 1), ACM2 (NMPAC), BouLac (NMPBL) and QNSE with EDKF mass
flux (NMPQN), the RUC LSM with rdlai2d in daylight (RUCL: P4 from 06 UTC, 9 soil levels), gravitational fog-water
settling with non-MYNN PBLs on P4 (GS1Y: grav_settling 1 with YSU; GS2J: grav_settling 2 with MYJ / Eta), the
diffusion skip flags (*_mix2_off, *_mix6_off) and mix_isotropic 1 on P6 with Thompson moist and scalar arrays (MX2I:
diff_opt 2 / km_opt 2 / diff_6th_opt 2; MX1I: diff_opt 1 / km_opt 3 / diff_6th_opt 2), Morrison and NSSL 2-moment
microphysics with non-KF cumulus on P4 (MOGF: Morrison + Grell-Freitas-Li; MOMK: Morrison + MSKF; NSNT: NSSL + New
Tiedtke; NSMK: NSSL + MSKF), and the remaining surface-layer flux options (SFT2: sfclayrev isftcflx 2 / iz0tlnd 2 on
P4; MYF10 / MYFW2: MYNN sf_mynn_sfcflux_land 1 / _water 0 and _water 2 in daylight from 06 UTC on P6 with RRTMG every
step).

Each against its WRF v4.8.0 run (tests/verify/refs/cov3_refs.txt in port_cov3_data.tar; tools/colab/colab_wrfref.sh;
r8 or r8d8 as the schemes need): 3 steps with history every step and 10 min with history every 5 min, CPU FP64 and
CUDA FP64, every variable at tolerance 0.

Not covered here: Noah-MP with fractional_seaice = 1 (the matrix base runs P4 / P6 / N3 have no sea ice; Svalbard
V2 / F1 / SU1 are not in matrix_data.tar / matrix_nest.tar / matrix_real.tar); Noah-MP with sst_update = 1 (no
sst_update base run in those tars, auxinput4_inname cannot be quoted through wrfref_body.sh, and the 6 h wrflowinp
update exceeds the wave's 10 min window); diff_opt 2 / km_opt 3 with bl_pbl_physics = 0 (WRF blows up from step 3 on
the 15 km P4 domain, as in KM3); grav_settling 2 on USGS land use (all matrix base runs use MODIS land use).

Base-run and input-tar dependencies to verify when staging references from matrix_data.tar and matrix_real.tar:
- run_P4_r8 (matrix_data.tar):
  - NoahmpTable.TBL for NMPAC, NMPBL, NMPQN;
  - LANDUSE.TBL, VEGPARM.TBL, SOILPARM.TBL, GENPARM.TBL, RRTMG_LW_DATA, RRTMG_SW_DATA, ozone*.formatted,
    CAMtr_volume_mixing_ratio for the P4-based references (LESK2, NMPAC, NMPBL, NMPQN, RUCL, GS1Y, GS2J, MOGF, MOMK,
    NSNT, NSMK, SFT2);
  - open-water points (XLAND == 2) in wrfinput_d01 to exercise isftcflx = 2 in SFT2;
  - lowest-level cloud water (QCLOUD(k=1) > 1e-12) during 00:00-00:10 UTC to exercise sf_fogdes surface deposition
    (including FogDES vegetation/LAI resistance for grav_settling = 2) in GS1Y and GS2J (otherwise only bl_fogdes
    aloft runs).
- run_P6_r8 (matrix_data.tar):
  - NoahmpTable.TBL for NMPMY;
  - RRTMG_LW_DATA, RRTMG_SW_DATA, ozone*.formatted, CAMtr_volume_mixing_ratio for NMPMY, MYF10, MYFW2 (P6 itself
    runs without radiation);
  - Thompson .dat tables (qr_acr_qg_V4.dat, qr_acr_qsV2.dat, freezeH2O.dat) for NMPMY, MX2I, MX1I, MYF10, MYFW2
    (stage_matrix_data.sh omits freezeH2O.dat, so wrf.exe computes it if absent);
  - LANDUSE.TBL, VEGPARM.TBL, SOILPARM.TBL, GENPARM.TBL and real.exe namelist keys (start/end year, month, day,
    interval_seconds, num_metgrid_levels, num_metgrid_soil_levels) for NMPMY, MYF10, MYFW2.
- real_P4_r8 and real_r8.exe (matrix_real.tar):
  - met_em.d01.2026-10-12_06:00:00.nc and met_em.d01.2026-10-12_12:00:00.nc plus real_r8.exe for all 06 UTC
    references (NMPMY, NMPAC, NMPBL, NMPQN, RUCL, MYF10, MYFW2);
  - non-zero LAI12M in met_em to exercise rdlai2d = .true. (LAI kept by RUC) in RUCL;
  - open-water points (XLAND == 2) in the 06 UTC wrfinput_d01 to exercise sf_mynn_sfcflux_water = 0 in MYF10 and
    sf_mynn_sfcflux_water = 2 (its sole non-default option) in MYFW2.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_D6 = "2026-10-12_06:00:00"
_D6_TAGS = {"nmpmy", "nmpac", "nmpbl", "nmpqn", "rucl", "myf10", "myfw2"}  # daylight runs (06 UTC, real.exe)
_BASE = ("U", "V", "W", "PH", "MU", "THM", "QV")
_MP = ("QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")
_SFC = ("TSK", "HFX", "LH", "UST", "PBLH", "RAINC", "RAINNC")
_NSSL = ("QHAIL", "QNDROP", "QNRAIN", "QNICE", "QNSNOW", "QNGRAUPEL", "QNHAIL", "QNCCN", "QVGRAUPEL", "QVHAIL",
         "REFL_10CM", "RAINC", "RAINNC", "RTHCUTEN", "RE_CLOUD", "RE_ICE", "RE_SNOW")


def _tol(*names):
    return {v: 0.0 for v in names}


# (tag, minutes of the long run, history interval of the long run [min], tolerances)
_SETS = [
    ("lesk2", 10, 5, _tol(*_BASE, *_MP, "TKE", "TSK", "HFX", "LH", "UST", "RAINC", "RAINNC")),
    ("nmpmy", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "CLDFRA", "TSK", "HFX", "LH", "T2", "SMOIS", "TSLB",
                          "RAINNC", "SWDOWN", "GLW")),
    ("nmpac", 10, 5, _tol(*_BASE, *_MP, *_SFC, "T2", "SMOIS", "TSLB", "RTHBLTEN", "SWDOWN")),
    ("nmpbl", 10, 5, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL", "EL_PBL", "T2", "SMOIS", "TSLB", "SWDOWN")),
    ("nmpqn", 10, 5, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL", "T2", "SMOIS", "TSLB", "SWDOWN")),
    ("rucl", 10, 5, _tol(*_BASE, *_MP, *_SFC, "LAI", "TSLB", "SMOIS", "SWDOWN")),
    ("gs1y", 10, 5, _tol(*_BASE, *_MP, *_SFC)),
    ("gs2j", 10, 5, _tol(*_BASE, *_MP, *_SFC, "TKE_PBL")),
    ("mx2i", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "TKE", "RAINNC")),
    ("mx1i", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "RAINNC")),
    ("mogf", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNSNOW", "QNRAIN", "QNGRAUPEL", "RAINC", "RAINNC", "RTHCUTEN")),
    ("momk", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNSNOW", "QNRAIN", "QNGRAUPEL", "RAINC", "RAINNC", "RTHCUTEN")),
    ("nsnt", 10, 5, _tol(*_BASE, *_MP, *_NSSL)),
    ("nsmk", 10, 5, _tol(*_BASE, *_MP, *_NSSL)),
    ("sft2", 10, 5, _tol(*_BASE, "TSK", "HFX", "LH", "UST", "ZNT")),
    ("myf10", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "TSK", "HFX", "LH", "UST", "ZNT", "SWDOWN", "GLW")),
    ("myfw2", 10, 5, _tol(*_BASE, *_MP, "QNICE", "QNRAIN", "QKE", "TSK", "HFX", "LH", "UST", "ZNT", "SWDOWN", "GLW")),
]

CASES = []
for _tag, _m, _h, _tl in _SETS:
    _cfg, _run = f"configs/real_{_tag}_r8.ini", f"{WR}/run_{_tag.upper()}_r8"
    _long = ["real.dir=" + _run, f"run.run_time={_m * 60}", f"output.out_interval={_h}", f"output.diag_interval={_h}"]
    _dt = _D6 if _tag in _D6_TAGS else _D
    CASES += [
        (f"real_{_tag}_r8_fp64_cpu", "cpu_dp", _cfg, [], f"{_run}/steps/wrfout_d01_{_dt}", 180, _tl),
        (f"real_{_tag}_r8_fp64_cpu_{_m}m", "cpu_dp", _cfg, _long, f"{_run}/wrfout_d01_{_dt}", _m * 60, _tl),
        (f"real_{_tag}_r8_fp64_cuda_{_m}m", "cuda_dp", _cfg, _long, f"{_run}/wrfout_d01_{_dt}", _m * 60, _tl),
    ]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('LESK2', 'P4 with bl_pbl_physics 0 (no PBL) and diff_opt 2 / km_opt 2 (1.5-order TKE) over the Noah LSM', '10 min'),
    ('NMPMY', 'P6 in daylight (06 UTC) with Noah-MP, MYNN surface layer and MYNN-EDMF PBL, RRTMG every step', '10 min'),
    ('NMPAC', 'P4 in daylight (06 UTC) with Noah-MP and the ACM2 PBL over sfclayrev', '10 min'),
    ('NMPBL', 'P4 in daylight (06 UTC) with Noah-MP and the BouLac PBL over sfclayrev', '10 min'),
    ('NMPQN', 'P4 in daylight (06 UTC) with Noah-MP and the QNSE surface layer and PBL (mfshconv 1)', '10 min'),
    ('RUCL', 'P4 in daylight (06 UTC) with the RUC LSM (9 soil levels) and rdlai2d', '10 min'),
    ('GS1Y', 'P4 with grav_settling 1 and the YSU PBL', '10 min'),
    ('GS2J', 'P4 with grav_settling 2 and the MYJ PBL / Eta surface layer', '10 min'),
    ('MX2I', 'P6 with diff_opt 2 / km_opt 2, diff_6th_opt 2, mix_isotropic 1 and all *_mix2_off / *_mix6_off flags', '10 min'),
    ('MX1I', 'P6 with diff_opt 1 / km_opt 3, diff_6th_opt 2, mix_isotropic 1 and moist/scalar *_mix2_off / *_mix6_off', '10 min'),
    ('MOGF', 'P4 with Morrison two-moment microphysics and Grell-Freitas-Li cumulus (ishallow 1)', '10 min'),
    ('MOMK', 'P4 with Morrison two-moment microphysics and multi-scale Kain-Fritsch cumulus', '10 min'),
    ('NSNT', 'P4 with NSSL 2-moment microphysics and New Tiedtke cumulus', '10 min'),
    ('NSMK', 'P4 with NSSL 2-moment microphysics and multi-scale Kain-Fritsch cumulus', '10 min'),
    ('SFT2', 'P4 with isftcflx 2 and iz0tlnd 2', '10 min'),
    ('MYF10', 'MYNN in daylight (06 UTC) with sf_mynn_sfcflux_land 1 and sf_mynn_sfcflux_water 0, RRTMG every step', '10 min'),
    ('MYFW2', 'MYNN in daylight (06 UTC) with sf_mynn_sfcflux_water 2, RRTMG every step', '10 min'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_cov3_data.tar' for c in CASES}
