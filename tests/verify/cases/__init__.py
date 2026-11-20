# SPDX-License-Identifier: GPL-3.0-or-later
"""The cases of tests/verify/wrf_suite.py, one file per group: NN_<group>.py (loaded in the order of their names, so
the case order is fixed; a new group takes a free number) defines CASES, a list of
  (name, binary key, config, extra --set, WRF wrfout, time [s], {var: max rel L2 (whole domain)})
(binary key cpu_dp, cpu_sp (the CPU FP32 build, against WRF real*4 references; each also derived as a cuda_sp_id case, below), cuda_dp or cuda; WRF wrfout: a file, or {domain: file} for nests; tolerances all 0.0: compared
exactly in every variable of the wrfout file, the list names the variables that matter most for the case). A config
"dropin:<WRF run directory>" runs tools/cppwrf-wrf in a copy of that directory instead (wrf_suite.dropin_spec).
A group file may define STATUS, its rows of docs/STATUS.md's table of the CPU FP64 cases (wrf_suite.py
--status-table), and DATA, the data tars (tests/verify/vm_data.txt) each case reads, {name: "TAR,TAR"} in
vm_data.txt's order (tests/verify/case_data.py derives and checks them; tools/colab/colab_suite.sh --cases fetches
only those). restarts.py defines RESTARTS, the restart checks, and DATA for wrf_suite.py's special checks (Straka, the
restart checks). Case names are unique."""
import glob
import importlib.util
import os
import sys

_DIR = os.path.dirname(os.path.abspath(__file__))
if os.path.dirname(_DIR) not in sys.path:  # the group files import cases.common
    sys.path.insert(0, os.path.dirname(_DIR))


def _load(path):
    spec = importlib.util.spec_from_file_location(f"cases._{os.path.basename(path)[:-3]}", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


GROUPS = [_load(p) for p in sorted(glob.glob(os.path.join(_DIR, "[0-9][0-9]_*.py")))]
CASES = [c for g in GROUPS for c in g.CASES]
STATUS = [r for g in GROUPS for r in getattr(g, "STATUS", [])]
_dup = sorted({c[0] for c in CASES if [x[0] for x in CASES].count(c[0]) > 1})
if _dup:
    raise SystemExit(f"tests/verify/cases: case names defined twice: {', '.join(_dup)}")
from cases.restarts import RESTARTS  # noqa: E402,F401
import cases.restarts as _restarts  # noqa: E402
# the data tars each case and special check reads ("TAR,TAR" in tests/verify/vm_data.txt's order)
DATA = {n: t for g in GROUPS + [_restarts] for n, t in getattr(g, "DATA", {}).items()}
# The CUDA FP32 identity build (CMake CPPWRF_CUDA_FP32_IDENTITY; wrf_suite.py --cuda-sp-id): every cpu_sp case again on
# the GPU (key cuda_sp_id), against the same WRF real*4 reference at tolerance 0; real_p1_fp32_cpu ->
# real_p1_fp32_cuda_id. Its data are the cpu_sp case's.


def _id_name(n):
    return n.replace("_cpu", "_cuda_id", 1) if "_cpu" in n else n + "_cuda_id"


_ID = [(_id_name(c[0]), "cuda_sp_id") + tuple(c[2:]) for c in CASES if c[1] == "cpu_sp"]
CASES += _ID
DATA.update({_id_name(c[0]): DATA[c[0]] for c in CASES if c[1] == "cpu_sp" and c[0] in DATA})
