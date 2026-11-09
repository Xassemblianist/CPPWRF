# SPDX-License-Identifier: GPL-3.0-or-later
"""Where CPPWRF's files are: the namelist translator (tools/), the data (data/) and the binaries of each mode.

The same lookup works in the source tree (python/cppwrf next to tools/ and data/), in an installed prefix
(share/cppwrf/python/cppwrf next to share/cppwrf/tools and share/cppwrf/data, the binaries in bin/) and for a
pip-installed package (the binaries on PATH, their prefix's share/cppwrf). Environment variables override it:
CPPWRF_HOME (a source tree or share/cppwrf directory), CPPWRF_EXE (the model binary), CPPWRF_REAL_EXE
(cppwrf_real), CPPWRF_BIN_DIR (a directory with the binaries)."""
import functools
import glob
import os
import shutil
import subprocess
from typing import Dict, List, Optional

PKG = os.path.dirname(os.path.abspath(__file__))

# the run modes and the (backend, precision) of the binary that runs them
MODES = {"gpu-fp32": ("cuda", "float32"), "gpu-fp64": ("cuda", "float64"), "cpu-fp64": ("cpu", "float64"),
         "cpu-fp32": ("cpu", "float32")}


class NotFound(RuntimeError):
    """A file or binary CPPWRF needs is not there."""


def _homes() -> List[str]:
    """Candidate CPPWRF homes (a directory with tools/ and data/), most specific first."""
    out = []
    if os.environ.get("CPPWRF_HOME"):
        h = os.environ["CPPWRF_HOME"]
        out += [h, os.path.join(h, "share", "cppwrf")]
    out.append(os.path.normpath(os.path.join(PKG, "..", "..")))  # source tree, or share/cppwrf/python/cppwrf
    for exe in ("cppwrf", "cppwrf-cpu-fp64", "cppwrf-gpu-fp32", "cppwrf-gpu-fp64"):
        p = shutil.which(exe)
        if p:
            out.append(os.path.join(os.path.dirname(os.path.realpath(p)), "..", "share", "cppwrf"))
    return out


def home() -> str:
    """The CPPWRF home: the directory with tools/wrf2cppwrf.py and data/."""
    for h in _homes():
        if os.path.isfile(os.path.join(h, "tools", "wrf2cppwrf.py")):
            return os.path.normpath(h)
    raise NotFound("CPPWRF's tools/wrf2cppwrf.py not found (set CPPWRF_HOME to the source tree or to "
                   "<prefix>/share/cppwrf)")


def tools_dir() -> str:
    return os.path.join(home(), "tools")


def _bin_dirs() -> List[str]:
    dirs = []
    if os.environ.get("CPPWRF_BIN_DIR"):
        dirs.append(os.environ["CPPWRF_BIN_DIR"])
    h = None
    try:
        h = home()
    except NotFound:
        pass
    if h:
        dirs.append(os.path.normpath(os.path.join(h, "..", "..", "bin")))  # <prefix>/share/cppwrf -> <prefix>/bin
        # a source tree: its build directories (README.md's build-cpu, build, build-gpu-dp, ...)
        dirs += sorted(d for d in glob.glob(os.path.join(h, "build*")) if os.path.isdir(d))
    dirs += os.environ.get("PATH", "").split(os.pathsep)
    seen, out = set(), []
    for d in dirs:
        if d and os.path.isdir(d) and os.path.realpath(d) not in seen:
            seen.add(os.path.realpath(d))
            out.append(d)
    return out


@functools.lru_cache(maxsize=None)
def describe(exe: str) -> Optional[Dict[str, str]]:
    """`cppwrf --version` of a binary: {version, commit, backend, precision}, or None when it does not answer."""
    try:
        r = subprocess.run([exe, "--version"], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None
    f = r.stdout.split()
    if r.returncode != 0 or len(f) < 8 or f[0] != "cppwrf":
        return None
    return {"version": f[1], "commit": f[3], "backend": f[5], "precision": f[7]}


def mode_of(exe: str) -> Optional[str]:
    d = describe(exe)
    if not d:
        return None
    return next((m for m, bp in MODES.items() if bp == (d["backend"], d["precision"])), None)


def model_binaries() -> List[str]:
    """Every cppwrf binary found (CPPWRF_EXE first), without duplicates."""
    found = []
    if os.environ.get("CPPWRF_EXE"):
        found.append(os.environ["CPPWRF_EXE"])
    names = ["cppwrf-" + m for m in MODES] + ["cppwrf"]
    for d in _bin_dirs():
        for n in names:
            p = os.path.join(d, n)
            if os.path.isfile(p) and os.access(p, os.X_OK):
                found.append(p)
    seen, out = set(), []
    for p in found:
        r = os.path.realpath(p)
        if r not in seen:
            seen.add(r)
            out.append(p)
    return out


def model_binary(mode: Optional[str] = None, exe: Optional[str] = None) -> str:
    """The model binary of a mode (None: the first one found). `exe` names it directly; its mode must then match."""
    if mode is not None and mode not in MODES:
        raise ValueError(f"unknown mode '{mode}'; the modes are {', '.join(MODES)}")
    cands = [exe] if exe else model_binaries()
    for p in cands:
        m = mode_of(p)
        if m is None:
            continue
        if mode is None or m == mode:
            return p
    have = sorted({m for m in (mode_of(p) for p in cands) if m})
    what = f"a cppwrf binary for mode {mode}" if mode else "a cppwrf binary"
    raise NotFound(f"{what} was not found" + (f" ({exe} is {mode_of(exe) or 'not a cppwrf binary'})" if exe else "")
                   + (f"; this installation has {', '.join(have)}" if have else "")
                   + ". Build it (docs/INSTALL.md) or name it with --exe / CPPWRF_EXE.")


def real_binary(exe: Optional[str] = None) -> str:
    """cppwrf_real (WRF's real.exe)."""
    if exe:
        return exe
    if os.environ.get("CPPWRF_REAL_EXE"):
        return os.environ["CPPWRF_REAL_EXE"]
    for d in _bin_dirs():
        for p in (os.path.join(d, "cppwrf_real"), os.path.join(d, "src", "init_real", "cppwrf_real")):
            if os.path.isfile(p) and os.access(p, os.X_OK):
                return p
    raise NotFound("cppwrf_real was not found (build it, or name it with --exe / CPPWRF_REAL_EXE)")
