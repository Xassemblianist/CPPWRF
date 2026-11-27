# SPDX-License-Identifier: GPL-3.0-or-later
"""Restart files of nests, moving nests, fractional time steps and grid nudging against WRF v4.8.0 (drop-in runs of
WRF's own run directories, tools/cppwrf-wrf; the wrfrst files compared too, suite.wrfrst: the same set of files, each
identical, the alarms' WRF_ALARM_* attributes among them). References tests/verify/refs/restart_audit_refs.txt, data
auda1_data.tar; N3s (d02 at (40, 30) of d01, ratio 3, the P4 physics) and FD7 (P4 with grid nudging, analyses every 6 h to 12 h):
- (not a case: moving nests with restart files are unverified, behind cppwrf.allow_unverified) AU1MV, N3s with d02
  moved at 1 min (+1, 0) and 2 min (0, +1), restart files every minute (WRF dmpar, one rank, preset moves): CPPWRF
  writes wrfrst_d02 after med_nest_move as WRF does (wrfout identical; 206 variables and I_PARENT_START of
  wrfrst_d02 at 1 min differed while it was written before the move), but the restart-only fields of a moved nest
  still differ (RHO, WW, RUBLTEN / RVBLTEN of the new strip; DEFOR11 / DEFOR22 / DIV at the top; d01's HT_COARSE; the
  preset-moves build's XI / XJ / VC_I / VC_J and alarm 55);
- AU1DT: N3s with dt = 50 s (d02: 50 / 3 s), restart files every 250 s: DTBC of wrfrst_d02 is solve_em's REAL sum
  of the nest's three steps (50), not the time since the parent step in double (250.00000000000003 - 250);
- (not a case: tools/wrf2cppwrf.py refuses time_step_fract for real data) AU1FR, N3s's d01 alone with dt = 60 / 11 s,
  restart files every 55 s, history every 60 s: WRF writes WRF_ALARM_SECS_TIL_NEXT_RING_01 = 6 at 114 6/11 s (whole
  seconds of the exact clock, truncated), as tests/verify/restart_alarm_test.py checks on em_quarter_ss;
- AU1RS1: N3s with restart_interval_s = 100 (not a multiple of d01's 60 s): only domain 1's alarm writes restart files
  (120 s), each nest's own alarm rings on its clock (d02 at 100 s, no file); AU1RS2: WRF's restart run from AU1RS1's
  files of 120 s with restart_interval_s = 120: domain 1 rings at 240 s (RingTime 200 s), d02's own alarm at 200 s;
  files of both domains at 240 and 360 s (d02's WRF_ALARM_ISRINGING_51 = 0, its seconds to the next ring 80);
- AU1FD2: WRF's restart run of FD7 with gfdda_end_h = 1 and radt 30 from its wrfrst file of 30 min, to 6 h 30 min (restart_interval 330 min: wrfrst at 1 h and 6 h 30 min): input_wrf
  sets AUXINPUT10_ALARM without StopTime (ESMF_AlarmSet clears StopTimeSet), so it reads the 6 h analyses past
  gfdda_end_h (the *_NDG_* fields and WRF_ALARM_SECS_TIL_NEXT_RING_36 of the wrfrst file of 6 h 30 min).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_B = f"{WR}/base_AUA1"
_BF = f"{WR}/base_AUA1F"
_D = "2026-10-12"
_T = {v: 0.0 for v in ("U", "W", "THM", "QV", "MU")}
_TN = {f"{d}.{v}": 0.0 for d in ("d01", "d02") for v in ("U", "W", "THM", "QV", "MU")}


def _two(path, date):
    return {d: f"{path}/wrfout_{d}_{_D}_{date}" for d in ("d01", "d02")}


def _rsm(first, second, at, doms=("d01",), base=_B):
    return ["suite.dropin_namelist=" + f"{base}/run_{second}", "suite.wrfrst"] + \
        ["suite.dropin_link=" + f"{base}/run_{first}/wrfrst_{d}_{_D}_{at}" for d in doms]


_CPU = [
    ("dropin_au1dt_r8_fp64_cpu", "cpu_dp", f"dropin:{WR}/run_AU1DT_r8", ["suite.wrfrst"],
     _two(f"{WR}/run_AU1DT_r8", "00:00:00"), 500, _TN),
    ("dropin_au1rs1_r8_fp64_cpu", "cpu_dp", f"dropin:{_B}/run_AU1RS1", ["suite.wrfrst"],
     _two(f"{_B}/run_AU1RS1", "00:00:00"), 120, _TN),
    ("dropin_au1rs2_r8_fp64_cpu", "cpu_dp", f"dropin:{_B}/run_AU1RS1",
     _rsm("AU1RS1", "AU1RS2", "00:02:00", ("d01", "d02")), _two(f"{_B}/run_AU1RS2", "00:03:00"), 360, _TN),
    ("dropin_au1fd2_r8_fp64_cpu", "cpu_dp", f"dropin:{_BF}/run_AU1FD1", _rsm("AU1FD1", "AU1FD2", "00:30:00", base=_BF),
     f"{_BF}/run_AU1FD2/wrfout_d01_{_D}_01:00:00", 23400, _T),
]
CASES = _CPU + [(c[0].replace("_cpu", "_cuda"), "cuda_dp") + tuple(c[2:]) for c in _CPU]

STATUS = [
    ('AU1DT', 'N3s with dt = 50 s (d02 50 / 3 s), restart files every 250 s (DTBC of wrfrst_d02)', '500 s'),
    ('AU1RS', 'N3s with restart_interval_s = 100 (domain 1\'s alarm writes every domain\'s files), resumed at 120 s with '
     '120 s (d02\'s own alarm rings at 200 s, files at 240 and 360 s)', '6 min'),
    ('AU1FD', 'FD7 with gfdda_end_h = 1 resumed from its wrfrst file of 30 min to 6 h 30 min (the auxinput10 alarm '
     'without StopTime reads the 6 h analyses)', '6 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order
DATA = {c[0]: 'auda1_data.tar' for c in CASES}
