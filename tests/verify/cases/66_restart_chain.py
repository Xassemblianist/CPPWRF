# SPDX-License-Identifier: GPL-3.0-or-later
"""Restart chains (wrf_suite.restart_chain, suite.chain=MID,INTERVAL): CPPWRF's run with restart files every
INTERVAL s and the runs resumed at MID s from its wrfrst files (and, for one domain, its own restart_<step>.nc) must
write the same wrfout frames and wrfrst files after MID at tolerance 0. The physics calls run less often than every
step (radt, bldt, cudt > dt) and MID and the wrfrst files after it fall between their calls, so the resumed runs
must carry what WRF's restart stream (Registry r) carries between the calls. Each chain also compares the straight
and the resumed runs with WRF's uninterrupted run of the chain's very configuration (suite.chain_wrf, every domain)
at tolerance 0: D0 and SR1 with their WRF runs; P4, G2, P8, NMP1, SLAB1 and N3, whose physics intervals (radt, bldt,
cudt) differ from their 1 h WRF runs', with the WRF runs CHP4_r8 ... CHN3_r8 of tests/verify/refs/chain_refs.txt
(tools/colab/colab_wrfref.sh; their directories give the inputs, tables and namelist.input; data chain_wrf_data.tar).

CASES: see tests/verify/cases/__init__.py."""
from cases.common import WR  # noqa: F401

_D = "2026-10-12_00:00:00"
# P4's physics (sfclayrev, Noah, YSU, Kain-Fritsch, WSM6, RRTMG) with radiation every 10 min, PBL and cumulus every
# 5 min (dt 60 s): resumed at 6 min, wrfrst at 9 min before the next PBL, cumulus and radiation calls
_P4 = ["real.dir=" + f"{WR}/run_CHP4_r8", "phys.radt=10", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3",
       "output.diag_interval=3"]
# the nest N3 (P4's physics on both domains) with radiation every 4 min on d01 and every 2 min on d02: resumed at
# 10 min, where d02's radiation runs before d01's (d02 reads the ozone d01 computed at 8 min)
_N3 = ["real.dir=" + f"{WR}/run_CHN3_r8", "phys.table_dir=" + f"{WR}/run_CHN3_r8", "phys.radt=4", "d02.phys_radt=2",
       "output.out_interval=5", "output.diag_interval=5", "d02.out_interval=15", "d02.diag_interval=15"]


def _wrf(n):  # WRF's run of the chain's configuration (tests/verify/refs/chain_refs.txt), d01's file (every domain compared)
    return f"suite.chain_wrf={WR}/run_{n}_r8/wrfout_d01_{_D}"


def _ref(n, doms=None):  # its directory gives the inputs, tables and namelist.input
    return {d: f"{WR}/run_{n}_r8/wrfout_{d}_{_D}" for d in doms} if doms else f"{WR}/run_{n}_r8/wrfout_d01_{_D}"


CASES = [
    # WRF's default namelist (CONUS: Thompson, Tiedtke, RRTMG radt 15, MYJ, Noah, GWDO, fractional sea ice) resumed
    # at 6 min, wrfrst every 3 min before the next radiation call at 15 min
    # (its run_D0_r8/steps inputs; against WRF's run of that directory, history every step, too)
    ("real_d0_r8_fp64_cpu_chain", "cpu_dp", "configs/real_d0_r8.ini",
     ["output.out_interval=3", "output.diag_interval=3", "suite.chain=360,180",
      f"suite.chain_wrf={WR}/run_D0_r8/steps/wrfout_d01_{_D}"],
     f"{WR}/run_D0_r8/steps/wrfout_d01_{_D}", 1200, {}),
    # P4 (against WRF's run CHP4_r8 of this configuration: radt 10, bldt 5, cudt 5, history every 3 min)
    ("real_p4_r8_fp64_cpu_chain", "cpu_dp", "configs/real_p4_r8.ini", _P4 + ["suite.chain=360,180", _wrf("CHP4")],
     _ref("CHP4"), 900, {}),
    # G2 (P2 with GWDO, gwd_opt 1) with PBL and cumulus every 5 min (WRF: CHG2_r8)
    ("real_g2_r8_fp64_cpu_chain", "cpu_dp", "configs/real_g2_r8.ini",
     ["real.dir=" + f"{WR}/run_CHG2_r8", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180", _wrf("CHG2")], _ref("CHG2"), 900, {}),
    # P8 (Grell-Freitas-Li, ishallow 1; WRF r8d8: CHP8_r8) and NMP1 (Noah-MP; WRF: CHNMP1_r8) with PBL (and
    # cumulus) every 5 min
    ("real_p8_r8_fp64_cpu_chain", "cpu_dp", "configs/real_p8_r8.ini",
     ["real.dir=" + f"{WR}/run_CHP8_r8", "phys.bldt=5", "phys.cudt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180", _wrf("CHP8")], _ref("CHP8"), 900, {}),
    ("real_nmp1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_nmp1_r8.ini",
     ["real.dir=" + f"{WR}/run_CHNMP1_r8", "phys.bldt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180", _wrf("CHNMP1")], _ref("CHNMP1"), 900, {}),
    # SR1 (slope_rad, topo_shading, radt 10) resumed at 245 min, in daylight between the radiation calls of 240 and
    # 250 min, from CPPWRF's own restart file only: WRF's SHADOWMASK is in no stream (Registry '-') and its
    # pre_radiation_driver sets it on the first step and the radiation steps only, so WRF's own run resumed there
    # (and CPPWRF's from the wrfrst file) shades nothing until the next radiation call; CPPWRF's file keeps it
    ("real_sr1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_sr1_r8.ini",
     ["real.dir=" + f"{WR}/run_SR1_r8", "output.out_interval=30", "output.diag_interval=30",
      "suite.chain=14700,2100,native", f"suite.chain_wrf={WR}/run_SR1_r8/wrfout_d01_{_D}"],
     f"{WR}/run_SR1_r8/wrfout_d01_{_D}", 16800, {}),
    # SLAB1 (the slab LSM: THC, CAPG) with PBL every 5 min (WRF: CHSLAB1_r8)
    ("real_slab1_r8_fp64_cpu_chain", "cpu_dp", "configs/real_slab1_r8.ini",
     ["real.dir=" + f"{WR}/run_CHSLAB1_r8", "phys.bldt=5", "output.out_interval=3", "output.diag_interval=3",
      "suite.chain=360,180", _wrf("CHSLAB1")], _ref("CHSLAB1"), 900, {}),
    # N3 (WRF: CHN3_r8, both domains)
    ("real_n3_r8_fp64_cpu_chain", "cpu_dp", "configs/real_n3.ini", _N3 + ["suite.chain=600,300", _wrf("CHN3")],
     _ref("CHN3", ("d01", "d02")), 1200, {}),
    # the same on the GPU (CUDA FP64: deterministic, resumed = straight bit for bit, and = WRF)
    ("real_p4_r8_fp64_cuda_chain", "cuda_dp", "configs/real_p4_r8.ini", _P4 + ["suite.chain=360,180", _wrf("CHP4")],
     _ref("CHP4"), 900, {}),
    ("real_n3_r8_fp64_cuda_chain", "cuda_dp", "configs/real_n3.ini", _N3 + ["suite.chain=600,300", _wrf("CHN3")],
     _ref("CHN3", ("d01", "d02")), 1200, {}),
]

STATUS = [
    ('D0, P4, G2, P8, NMP1, SR1, SLAB1, N3 chains', 'resumed from CPPWRF\'s wrfrst and native restart files (SR1: native only) between the radiation, '
     'PBL and cumulus calls (radt, bldt, cudt > dt): every later wrfout frame and wrfrst file equals the straight run\'s, '
     'and the straight and resumed runs equal WRF\'s run of the same configuration (every domain) at tolerance 0',
     '15-20 min'),
]

DATA = {
    'real_d0_r8_fp64_cpu_chain': 'v05_files_data.tar,v05_files_links.tar',
    'real_p4_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_g2_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_p8_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_nmp1_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_n3_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_sr1_r8_fp64_cpu_chain': 'suite_data.tar,port_a0204_data.tar',
    'real_slab1_r8_fp64_cpu_chain': 'chain_wrf_data.tar',
    'real_p4_r8_fp64_cuda_chain': 'chain_wrf_data.tar',
    'real_n3_r8_fp64_cuda_chain': 'chain_wrf_data.tar',
}
