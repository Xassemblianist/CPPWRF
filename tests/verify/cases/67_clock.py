# SPDX-License-Identifier: GPL-3.0-or-later
"""WRF's clock values (XTIME, julian, the year) on the fixed time step: P4D16 (past one day) and N3D50 (a nest step
that is a fraction of a second).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_N3D50_VARS = ("U", "W", "THM", "QV", "QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP")

CASES = [
    # P4D16: P4 on the 72 h inputs with dt = 16 s and radt = 10 min for 30 h, history every hour: past one day WRF's
    # XTIME (days*1440 + seconds/60) differs in the last bit from (itimestep - 1) * dt / 60, first at the radiation call
    # of 27.7 h (coszen, hour angle). Every hourly frame and variable at tolerance 0. The CPU case is slow (about
    # 100 min); the CUDA FP64 one takes a few minutes.
    *[(f"real_p4d16_r8_fp64_{b}_30h", f"{b}_dp", "configs/real_p4d16_r8.ini", [] if b == "cuda" else ["suite.slow"],
       f"{WR}/run_P4D16_r8/wrfout_d01_2026-10-12_00:00:00", 108000,
       {v: 0.0 for v in ("U", "W", "THM", "QV", "QCLOUD", "QRAIN", "QICE")})
      for b in ("cpu", "cuda")],
    # N3D50: the N3 two-way nest with d01's dt = 50 s, the nest's 50/3 s (WRF's clock: 16 + 2/3 s), 1 h, history
    # every 10 min on both domains: XTIME and julian of the exact time from the first steps on
    *[(f"real_n3d50_r8_fp64_{b}_1h", f"{b}_dp", "configs/real_n3d50_r8.ini", [],
       {d: f"{WR}/run_N3D50_r8/wrfout_{d}_2026-10-12_00:00:00" for d in ("d01", "d02")}, 3600,
       {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in _N3D50_VARS})
      for b in ("cpu", "cuda")],
]

# the rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py --status-table): case, configuration,
# duration
STATUS = [
    ('P4D16', 'P4 with dt = 16 s and radt = 10 min on the 72 h inputs, history every hour (WRF\'s clock past one '
     'day: XTIME, coszen)', '30 h'),
    ('N3D50', 'N3 with d01 dt = 50 s, so a nest step of 50/3 s (WRF\'s clock with a fraction of a second: XTIME, '
     'julian)', '1 h'),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {c[0]: 'port_af1_data.tar' for c in CASES}
