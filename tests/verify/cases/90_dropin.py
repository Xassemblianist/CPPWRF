# SPDX-License-Identifier: GPL-3.0-or-later
"""The drop-in wrf.exe (tools/cppwrf-wrf, python/cppwrf/dropin.py): WRF's own run directories run by cppwrf-wrf, which
reads their namelist.input, writes wrfout_d0N_<date> files there and the rsl logs; the files are compared with WRF's
at tolerance 0 as the other cases. P1 (3 steps), the two-way nest N1 (3 steps of d01) and P1 resumed from WRF's restart
file at minute 1 with WRF's restart = .true. namelist (rsm_P1_r8, tests/verify/make_resumed_refs.sh).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_N1 = {d: f"{WR}/run_N1_r8/steps/wrfout_{d}_2026-10-12_00:00:00" for d in ("d01", "d02")}
_RSM = ["suite.dropin_namelist=" + f"{WR}/rsm_P1_r8", "suite.dropin_link=" + f"{WR}/rst_P1_r8/wrfrst_d01_2026-10-12_00:01:00"]

CASES = [
    ("dropin_p1_r8_fp64_cpu", "cpu_dp", f"dropin:{WR}/run_P1_r8/steps", [],
     f"{WR}/run_P1_r8/steps/wrfout_d01_2026-10-12_00:00:00", 180,
     {"U": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0}),
    ("dropin_p1_r8_fp64_cuda", "cuda_dp", f"dropin:{WR}/run_P1_r8/steps", [],
     f"{WR}/run_P1_r8/steps/wrfout_d01_2026-10-12_00:00:00", 180,
     {"U": 0.0, "W": 0.0, "PH": 0.0, "THM": 0.0, "QV": 0.0, "MU": 0.0}),
    ("dropin_n1_r8_fp64_cpu", "cpu_dp", f"dropin:{WR}/run_N1_r8/steps", [], _N1, 180,
     {"d01.U": 0.0, "d01.THM": 0.0, "d02.U": 0.0, "d02.THM": 0.0}),
    ("dropin_n1_r8_fp64_cuda", "cuda_dp", f"dropin:{WR}/run_N1_r8/steps", [], _N1, 180,
     {"d01.U": 0.0, "d01.THM": 0.0, "d02.U": 0.0, "d02.THM": 0.0}),
    ("dropin_rsm_p1_r8_fp64_cpu", "cpu_dp", f"dropin:{WR}/run_P1_r8/steps", _RSM,
     f"{WR}/rsm_P1_r8/wrfout_d01_2026-10-12_00:02:00", 180, {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0}),
]

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {
    'dropin_p1_r8_fp64_cpu': 'v05_files_data.tar,v05_files_links.tar',
    'dropin_p1_r8_fp64_cuda': 'v05_files_data.tar,v05_files_links.tar',
    'dropin_n1_r8_fp64_cpu': 'suite_data.tar,v05_files_data.tar',
    'dropin_n1_r8_fp64_cuda': 'suite_data.tar,v05_files_data.tar',
    'dropin_rsm_p1_r8_fp64_cpu': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
}
