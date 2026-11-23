# SPDX-License-Identifier: GPL-3.0-or-later
"""restart_<tag>_fp64: the configuration of a tolerance-0 suite case, 3 steps (t1: 5, p8: 10) straight vs restarted after
step 1 from CPPWRF's own restart_000001.nc (the snapshots of step 3 bit for bit), and against WRF at tolerance 0
(wrf_suite.restart_check): the run resumed from that file in WRF history output against WRF's uninterrupted run
(every frame after the restart, every variable and global attribute: a run resumed from CPPWRF's own file
continues the run that wrote it, START_DATE included) and, where WRF's resumed run exists (rsm_<C>_r8, tests/verify/make_resumed_refs.sh), CPPWRF resumed
from WRF's own wrfrst file at minute 1 (rst_<C>_r8, make_restart_refs.sh) against it (wrfout and wrfrst files,
every global attribute). restart_<tag>_fp64_cuda: the same in CUDA FP64. (tag, config, input run directory under
${CPPWRF_WRF_RUNS})."""
RESTARTS = (("p1", "configs/real_p1_r8.ini", "run_P1_r8"), ("a01", "configs/real_a01_r8.ini", "run_P1_r8"),
            ("p7", "configs/real_p7_r8.ini", "run_P7_r8"),
            ("p8", "configs/real_p8_r8.ini", "run_P8_r8"), ("p9", "configs/real_p9_r8.ini", "run_P9_r8"),
            ("nmp1", "configs/real_nmp1_r8.ini", "run_NMP1_r8"), ("mynn", "configs/real_mynn_r8.ini", "run_MYNN_r8"),
            ("mynnsf1", "configs/real_mynnsf1_r8.ini", "run_P6_r8"), ("mynnsf2", "configs/real_mynnsf2_r8.ini", "run_P6_r8"),
            ("mynnx1", "configs/real_mynnx1_r8.ini", "run_P6_r8"), ("mynnx2", "configs/real_mynnx2_r8.ini", "run_P6_r8"),
            ("t1", "configs/real_t1.ini", "run_T1_r8"),
            ("a05", "configs/real_a05_r8.ini", "run_A05_r8"), ("a05g", "configs/real_a05g_r8.ini", "run_A05G_r8"),
            ("slab1", "configs/real_slab1_r8.ini", "run_SLAB1_r8"), ("acm7", "configs/real_acm7_r8.ini", "run_ACM7_r8"),
            ("morr", "configs/real_morr_r8.ini", "run_MORR_r8"), ("p3mp", "configs/real_p3mp_r8.ini", "run_P3MP_r8"))

# tag: (the suite case whose configuration (config, --set list) and WRF reference the check uses, WRF's run with
# restart files every minute (rst_<C>_r8: the uninterrupted reference of a straight run that writes restart files at
# each step, which the MYNN surface layer sees at its first step, solve_em's restart_flag) or None (the case's
# reference), WRF's run resumed from it at minute 1 (rsm_<C>_r8) or None, the restart time [s] of the comparison with
# WRF: 60, or 120 where the MYNN surface layer's first step would see the restart alarm and no rst_<C>_r8 exists)
RESTART_WRF = {
    "p1": ("real_p1_r8_fp64_cpu", "rst_P1_r8", "rsm_P1_r8", 60),
    "a01": ("real_a01_r8_fp64_cpu", None, None, 60),
    "p7": ("real_p7_r8_fp64_cpu", "rst_P7_r8", "rsm_P7_r8", 60),
    "p8": ("real_p8_r8_fp64_cpu_10min", "rst_P8_r8", "rsm_P8_r8", 60),
    "p9": ("real_p9_r8_fp64_cpu", "rst_P9_r8", "rsm_P9_r8", 60),
    "nmp1": ("real_nmp1_r8_fp64_cpu", "rst_NMP1_r8", "rsm_NMP1_r8", 60),
    "mynn": ("real_mynn_r8_fp64_cpu", "rst_MYNN_r8", "rsm_MYNN_r8", 60),
    "mynnsf1": ("real_mynnsf1_r8_fp64_cpu", None, None, 60),
    "mynnsf2": ("real_mynnsf2_r8_fp64_cpu", None, None, 60),
    "mynnx1": ("real_mynnx1_r8_fp64_cpu", None, None, 120),
    "mynnx2": ("real_mynnx2_r8_fp64_cpu", None, None, 120),
    "t1": ("real_t1_fp64_cpu", None, None, 60),
    "a05": ("real_a05_r8_fp64_cpu", None, None, 60),
    "a05g": ("real_a05g_r8_fp64_cpu", None, None, 60),
    "slab1": ("real_slab1_r8_fp64_cpu", None, None, 60),
    "acm7": ("real_acm7_r8_fp64_cpu", None, None, 60),
    "morr": ("real_morr_r8_fp64_cpu", None, None, 60),
    "p3mp": ("real_p3mp_r8_fp64_cpu", None, None, 60),
}

# the data tars (tests/verify/vm_data.txt) each case reads, in that file's order (tests/verify/case_data.py
# derives them from the tars' file lists; tools/colab/colab_suite.sh --cases fetches only these)
DATA = {
    'restart_p1_fp32_cuda': 'suite_data.tar,v05_files_links.tar',
    'restart_p1_fp64': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_a01_fp64': 'suite_data.tar,port_a01_data.tar,v05_files_links.tar',
    'restart_p7_fp64': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_p8_fp64': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_p9_fp64': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_nmp1_fp64': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynn_fp64': 'mynn_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnsf1_fp64': 'port_a06_data.tar,port_c01_data.tar,v05_iofields.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnsf2_fp64': 'port_a06_data.tar,port_c01_data.tar,v05_iofields.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnx1_fp64': 'port_a06_data.tar,port_c01_data.tar,v05_suite_refs2.tar,v05_iofields.tar',
    'restart_mynnx2_fp64': 'port_a06_data.tar,port_c01_data.tar,v05_suite_refs2.tar,v05_iofields.tar',
    'restart_t1_fp64': 'suite_data.tar,dycore_data.tar,v05_suite_refs.tar',
    'restart_a05_fp64': 'port_a05_data.tar',
    'restart_a05g_fp64': 'port_a05_data.tar',
    'restart_slab1_fp64': 'port_c22_data.tar',
    'restart_acm7_fp64': 'port_c16_data.tar',
    'restart_morr_fp64': 'port_c04_data.tar',
    'restart_p3mp_fp64': 'port_c09_data.tar',
    'restart_p1_fp64_cuda': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_a01_fp64_cuda': 'suite_data.tar,port_a01_data.tar,v05_files_links.tar',
    'restart_p7_fp64_cuda': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_p8_fp64_cuda': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_p9_fp64_cuda': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_nmp1_fp64_cuda': 'suite_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynn_fp64_cuda': 'mynn_data.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnsf1_fp64_cuda': 'port_a06_data.tar,port_c01_data.tar,v05_iofields.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnsf2_fp64_cuda': 'port_a06_data.tar,port_c01_data.tar,v05_iofields.tar,v05_files_data.tar,v05_files_links.tar',
    'restart_mynnx1_fp64_cuda': 'port_a06_data.tar,port_c01_data.tar,v05_suite_refs2.tar,v05_iofields.tar',
    'restart_mynnx2_fp64_cuda': 'port_a06_data.tar,port_c01_data.tar,v05_suite_refs2.tar,v05_iofields.tar',
    'restart_t1_fp64_cuda': 'suite_data.tar,dycore_data.tar,v05_suite_refs.tar',
    'restart_a05_fp64_cuda': 'port_a05_data.tar',
    'restart_a05g_fp64_cuda': 'port_a05_data.tar',
    'restart_slab1_fp64_cuda': 'port_c22_data.tar',
    'restart_acm7_fp64_cuda': 'port_c16_data.tar',
    'restart_morr_fp64_cuda': 'port_c04_data.tar',
    'restart_p3mp_fp64_cuda': 'port_c09_data.tar',
}
