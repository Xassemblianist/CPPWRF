# SPDX-License-Identifier: GPL-3.0-or-later
"""Restart chains (wrf_suite.restart_chain, suite.chain=MID,INTERVAL): CPPWRF's run with restart files every
INTERVAL s and the runs resumed at MID s from its wrfrst files (and, for one domain, its own restart_<step>.nc) must
write the same wrfout frames and wrfrst files after MID at tolerance 0. The physics calls run less often than every
step (radt, bldt, cudt > dt) and MID and the wrfrst files after it fall between their calls, so the resumed runs
must carry what WRF's restart stream (Registry r) carries between the calls. The WRF runs named give the inputs and
namelist.input; they are not compared (the options differ from theirs).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
# P4's physics (sfclayrev, Noah, YSU, Kain-Fritsch, WSM6, RRTMG) with radiation every 10 min, PBL and cumulus every
# 5 min (dt 60 s): resumed at 6 min, wrfrst at 9 min before the next PBL, cumulus and radiation calls
_P4 = ["real.dir=" + f"{WR}/run_P4_r8", "phys.radt=10", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3",
       "output.diag_interval=3"]
# the nest N3 (P4's physics on both domains) with radiation every 4 min on d01 and every 2 min on d02: resumed at
# 10 min, where d02's radiation runs before d01's (d02 reads the ozone d01 computed at 8 min)
_N3 = ["real.dir=" + f"{WR}/run_N3h_r8", "phys.table_dir=" + f"{WR}/run_N3h_r8", "phys.radt=4", "d02.phys_radt=2",
       "output.out_interval=5", "output.diag_interval=5", "d02.out_interval=15", "d02.diag_interval=15"]

CASES = [
    # WRF's default namelist (CONUS: Thompson, Tiedtke, RRTMG radt 15, MYJ, Noah, GWDO, fractional sea ice) resumed
    # at 6 min, wrfrst every 3 min before the next radiation call at 15 min
    ("real_d0_r8_fp64_cpu_chain", "cpu_dp", "configs/real_d0_r8.ini",
     ["real.dir=" + f"{WR}/run_D0_r8", "output.out_interval=3", "output.diag_interval=3", "suite.chain=360,180"],
     f"{WR}/run_D0_r8/wrfout_d01_{_D}", 1200, {}),
    ("real_p4_r8_fp64_cpu_chain", "cpu_dp", "configs/real_p4_r8.ini", _P4 + ["suite.chain=360,180"],
     f"{WR}/run_P4_r8/wrfout_d01_{_D}", 900, {}),
    # G2 (P2 with GWDO, gwd_opt 1) with PBL and cumulus every 5 min
    ("real_g2_r8_fp64_cpu_chain", "cpu_dp", "configs/real_g2_r8.ini",
     ["real.dir=" + f"{WR}/run_G2_r8", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180"], f"{WR}/run_G2_r8/wrfout_d01_{_D}", 900, {}),
    # P8 (Grell-Freitas-Li, ishallow 1) and NMP1 (Noah-MP) with PBL and cumulus every 5 min
    ("real_p8_r8_fp64_cpu_chain", "cpu_dp", "configs/real_p8_r8.ini",
     ["real.dir=" + f"{WR}/run_P8_r8", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180"], f"{WR}/run_P8_r8/wrfout_d01_{_D}", 900, {}),
    ("real_nmp1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_nmp1_r8.ini",
     ["real.dir=" + f"{WR}/run_NMP1_r8", "phys.bldt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180"], f"{WR}/run_NMP1_r8/wrfout_d01_{_D}", 900, {}),
    # SR1 (slope_rad, topo_shading, radt 10) resumed at 245 min, in daylight between the radiation calls of 240 and
    # 250 min
    ("real_sr1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_sr1_r8.ini",
     ["real.dir=" + f"{WR}/run_SR1_r8", "output.out_interval=30", "output.diag_interval=30", "suite.chain=14700,2100"],
     f"{WR}/run_SR1_r8/wrfout_d01_{_D}", 16800, {}),
    # SLAB1 (the slab LSM: THC, CAPG) with PBL every 5 min
    ("real_slab1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_slab1_r8.ini",
     ["real.dir=" + f"{WR}/run_SLAB1_r8", "phys.bldt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180"], f"{WR}/run_SLAB1_r8/wrfout_d01_{_D}", 900, {}),
    ("real_n3_r8_fp64_cpu_chain", "cpu_dp", "configs/real_n3.ini", _N3 + ["suite.chain=600,300"],
     {d: f"{WR}/run_N3h_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 1200, {}),
    # the same on the GPU (CUDA FP64: deterministic, resumed = straight bit for bit)
    ("real_p4_r8_fp64_cuda_chain", "cuda_dp", "configs/real_p4_r8.ini", _P4 + ["suite.chain=360,180"],
     f"{WR}/run_P4_r8/wrfout_d01_{_D}", 900, {}),
    ("real_n3_r8_fp64_cuda_chain", "cuda_dp", "configs/real_n3.ini", _N3 + ["suite.chain=600,300"],
     {d: f"{WR}/run_N3h_r8/wrfout_{d}_{_D}" for d in ("d01", "d02")}, 1200, {}),
]

STATUS = [
    ('D0, P4, G2, P8, NMP1, SR1, SLAB1, N3 chains', 'resumed from CPPWRF\'s wrfrst (and native) restart files between the radiation, '
     'PBL and cumulus calls (radt, bldt, cudt > dt): every later wrfout frame and wrfrst file equals the straight run\'s',
     '15-20 min'),
]

DATA = {
    'real_d0_r8_fp64_cpu_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_p4_r8_fp64_cpu_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_g2_r8_fp64_cpu_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_p8_r8_fp64_cpu_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_nmp1_r8_fp64_cpu_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_n3_r8_fp64_cpu_chain': 'n3_data.tar,v05_suite_refs.tar',
    'real_sr1_r8_fp64_cpu_chain': 'suite_data.tar,port_a0204_data.tar',
    'real_slab1_r8_fp64_cpu_chain': 'port_c22_data.tar',
    'real_p4_r8_fp64_cuda_chain': 'suite_data.tar,v05_suite_refs.tar',
    'real_n3_r8_fp64_cuda_chain': 'n3_data.tar,v05_suite_refs.tar',
}
