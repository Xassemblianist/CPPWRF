# SPDX-License-Identifier: GPL-3.0-or-later
"""Output-file contents that depend on WRF's namelist checks (module_check_a_mundo.F) and the I/O streams, each against a
WRF v4.8.0 real*8 run of the wrfref workflow (tags gem9-refs, gem9-refs2; 3 steps, history every step):
G9F6C = the N3 two-way nest with the multi-scale Kain-Fritsch on d01 only (cu_physics = 11, 0) and icloud = 0: WRF
sets icloud = 1 for every domain when any has cu_physics 11, so d02's radiation (no cumulus) runs with icloud 1;
G9F2 = P4 with output_diagnostics (auxhist3 every minute) and iofields_filename adding QVAPOR / QCLOUD to stream h:3
(4-D array members by their own stream masks; the auxhist3 files compared as the wrfout files).
The attribute / variable-list checks of the other GEM9 references (G9F1, G9F5, G9F6A0, G9F6A5, G9F6B, G9F6D, G9F6E,
G9F7) are the files test's (tests/verify/run_wrf_files_test.sh).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
_TN = {f"{d}.{x}": 0.0 for d in ("d01", "d02") for x in ("U", "W", "THM", "QV", "QCLOUD", "RTHRATEN", "GLW", "SWDOWN")}

CASES = [
    ("real_g9f6c_r8_fp64_cpu", "cpu_dp", "configs/real_n3_steps.ini",
     [f"real.dir={WR}/run_G9F6C_r8/steps", f"phys.table_dir={WR}/run_G9F6C_r8/steps", "phys.cu_physics=11",
      "d02.phys_cu_physics=0"],
     {d: f"{WR}/run_G9F6C_r8/steps/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 180, _TN),
    ("real_g9f2_r8_fp64_cpu", "cpu_dp", "configs/real_a09_r8.ini",
     [f"real.dir={WR}/run_G9F2_r8/steps", "output.auxhist3=true", "phys.auxhist3_interval=1"],
     f"{WR}/run_G9F2_r8/steps/wrfout_d01_{_D}", 180,
     {"U": 0.0, "W": 0.0, "THM": 0.0, "QV": 0.0, "QCLOUD": 0.0}),
]

STATUS = [
    ('G9F6C', 'N3 with the multi-scale Kain-Fritsch on d01 only and icloud = 0 (WRF: icloud 1 on both domains)',
     '3 steps'),
    ('G9F2', 'P4 with output_diagnostics and QVAPOR / QCLOUD added to auxhist3 by iofields_filename', '3 steps'),
]

DATA = {
    'real_g9f6c_r8_fp64_cpu': 'n3_data.tar,port_gem9_data.tar',
    'real_g9f2_r8_fp64_cpu': 'suite_data.tar,port_gem9b_data.tar',
}
