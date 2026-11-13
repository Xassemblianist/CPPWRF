# SPDX-License-Identifier: GPL-3.0-or-later
"""WRF's restart alarm with an interval that is not a multiple of the time step (configs/real_rsa_r8.ini: P4 with
dt = 50 s, restart_interval = 2 min, 700 s, history every 150 s), the wrfrst files compared too (suite.wrfrst: the
same set of files, each identical, WRF_ALARM_SECS_TIL_NEXT_RING_51 among the attributes):
- RSA: the uninterrupted run, wrfrst files at 150, 250, 400, 500, 600 s (ESMF_ClockAdvance: the first step at or
  past each multiple of the interval);
- RSA_RSM: WRF's restart = .true. run (cppwrf-wrf) resumed from RSA's wrfrst file of 150 s: input_wrf sets the alarm's
  RingTime to 150 s + WRF_ALARM_SECS_TIL_NEXT_RING_51 (90 s), it rings at the step of 250 s and counts the interval
  from there: 250, 400, 500, 650 s (650 s instead of 600 s);
- RSA_RSM0: the same with restart_interval = 0: the alarm (forever interval) still rings once at that RingTime, one
  wrfrst file of 250 s.

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_RST150 = f"{WR}/run_RSA_r8/wrfrst_d01_2026-10-12_00:02:30"


def _rsm(d):
    return ["suite.dropin_namelist=" + f"{WR}/{d}", "suite.dropin_link=" + _RST150, "suite.wrfrst"]


CASES = [
    *[(f"real_rsa_r8_fp64_{b}", f"{b}_dp", "configs/real_rsa_r8.ini", ["suite.wrfrst"],
       f"{WR}/run_RSA_r8/wrfout_d01_2026-10-12_00:00:00", 600,
       {v: 0.0 for v in ("U", "W", "THM", "QV", "QCLOUD", "QRAIN")}) for b in ("cpu", "cuda")],
    *[(f"dropin_rsa_{r}_r8_fp64_{b}", f"{b}_dp", f"dropin:{WR}/run_RSA_r8", _rsm(f"{r}_RSA_r8"),
       f"{WR}/{r}_RSA_r8/wrfout_d01_2026-10-12_00:05:00", 600,
       {v: 0.0 for v in ("U", "W", "THM", "QV", "QCLOUD", "QRAIN")}) for r in ("rsm", "rsm0") for b in ("cpu", "cuda")],
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('RSA', 'P4 with dt = 50 s and restart_interval = 2 min (not a multiple of dt), history every 150 s; the wrfrst '
     'files too; resumed from its wrfrst file of 150 s with restart_interval = 2 and = 0 (WRF\'s restart alarm of a '
     'resumed run: the first ring at the file\'s seconds to the next ring)', '700 s'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_af2_data.tar' for c in CASES}
