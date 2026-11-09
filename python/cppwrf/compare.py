# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-compare: two sets of WRF output files (wrfout, wrfrst, auxhist), variable by variable.

A front-end to the verification suite's comparison code (tests/verify/exact.py; installed in share/cppwrf/verify):
the same pairing of frames by their Times, the same notion of identical values (NaN equals NaN, -0 differs from +0)
and the same metadata check. It adds, per variable over the common frames: the frames that differ, max |a - b|, the
RMSE, the largest distance in units in the last place (ulp) and the first differing point.

  cppwrf-compare A B [-v VAR,VAR] [--exact] [--json] [--no-metadata]
A and B: two directories (each stream and domain found in both is compared: wrfout_d01, wrfout_d02, wrfrst_d01,
...), two files, or two glob patterns. Exit status: 0 identical, 1 different, 2 an error (no common frame, missing
files)."""
import argparse
import glob
import importlib
import json
import math
import os
import re
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import _locate
from ._version import __version__

STREAM = re.compile(r"^(wrfout|wrfrst|auxhist\d+)_(d\d\d)_\d{4}-\d\d-\d\d_\d\d:\d\d:\d\d")


def exact():
    """tests/verify/exact.py (the source tree) or share/cppwrf/verify/exact.py (an installation)."""
    h = _locate.home()
    for d in (os.path.join(h, "tests", "verify"), os.path.join(h, "verify")):
        if os.path.isfile(os.path.join(d, "exact.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            return importlib.import_module("exact")
    raise _locate.NotFound(f"exact.py (the verification suite's comparison code) not found under {h}")


@dataclass
class VarResult:
    name: str
    frames_compared: int = 0
    frames_differing: int = 0
    points_differing: int = 0
    points: int = 0
    max_abs: float = 0.0
    rmse: float = 0.0
    max_ulp: float = 0.0
    first: Optional[str] = None  # "Times (dim=index, ...): a vs b"
    note: Optional[str] = None  # shape / type differences

    @property
    def identical(self) -> bool:
        return self.points_differing == 0 and self.note is None


@dataclass
class StreamResult:
    label: str  # e.g. wrfout_d01
    files_a: List[str]
    files_b: List[str]
    common_times: List[str] = field(default_factory=list)
    only_a: List[str] = field(default_factory=list)
    only_b: List[str] = field(default_factory=list)
    only_vars_a: List[str] = field(default_factory=list)
    only_vars_b: List[str] = field(default_factory=list)
    metadata: List[str] = field(default_factory=list)
    variables: List[VarResult] = field(default_factory=list)

    @property
    def identical(self) -> bool:
        return (bool(self.common_times) and not self.only_a and not self.only_b and not self.only_vars_a
                and not self.only_vars_b and not self.metadata and all(v.identical for v in self.variables))


def _ordered(x: np.ndarray) -> np.ndarray:
    """IEEE values as integers in the order of the numbers (adjacent floats differ by 1), -0 and +0 both 0."""
    if x.dtype == np.float32:
        i = x.view(np.int32).astype(np.int64)
        return np.where(i < 0, np.int64(-(2**31)) - i, i)
    i = x.astype(np.float64).view(np.int64)
    return np.where(i < 0, np.int64(-(2**63)) - i, i)


def ulp_distance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """|a - b| in units in the last place of the type of a (float32 or float64), as float64; NaN where either is NaN."""
    a = np.asarray(a)
    b = np.asarray(b).astype(a.dtype if a.dtype in (np.float32, np.float64) else np.float64)
    if a.dtype not in (np.float32, np.float64):
        a = a.astype(np.float64)
    ia, ib = _ordered(a), _ordered(b)
    # exact in int64 where both have the same sign; across zero the distance may overflow int64 and float64 suffices
    same = (ia < 0) == (ib < 0)
    d = np.where(same, np.abs(np.where(same, ia - ib, 0)).astype(np.float64),
                 np.abs(ia.astype(np.float64) - ib.astype(np.float64)))
    return np.where(np.isnan(a) | np.isnan(b), np.nan, d)


def _streams(path: str) -> Dict[str, List[str]]:
    """{stream_dNN: files} of a directory, a file or a glob pattern."""
    if os.path.isdir(path):
        files = [os.path.join(path, f) for f in os.listdir(path)]
    else:
        files = glob.glob(path) or ([path] if os.path.exists(path) else [])
    out: Dict[str, List[str]] = {}
    for f in files:
        m = STREAM.match(os.path.basename(f))
        if m and os.path.isfile(f):
            out.setdefault(f"{m.group(1)}_{m.group(2)}", []).append(f)
    return {k: sorted(v) for k, v in out.items()}


def pairs(a: str, b: str) -> List[Tuple[str, List[str], List[str]]]:
    """(label, files of a, files of b) of each stream and domain to compare."""
    sa, sb = _streams(a), _streams(b)
    if not os.path.isdir(a) and not os.path.isdir(b) and len(sa) == 1 and len(sb) == 1:
        (la, fa), (lb, fb) = next(iter(sa.items())), next(iter(sb.items()))
        return [(la if la == lb else f"{la} / {lb}", fa, fb)]
    return [(k, sa[k], sb[k]) for k in sorted(set(sa) & set(sb))]


def compare_stream(label: str, files_a: Sequence[str], files_b: Sequence[str], variables: Optional[Sequence[str]] = None,
                   metadata: bool = True) -> StreamResult:
    """Every variable of every frame both have (by Times); one frame of one variable in memory at a time."""
    ex = exact()
    res = StreamResult(label, list(files_a), list(files_b))
    opened: list = []
    try:
        A, B = ex.frames(files_a, opened), ex.frames(files_b, opened)
        common = sorted(set(A) & set(B))
        res.common_times, res.only_a, res.only_b = common, sorted(set(A) - set(B)), sorted(set(B) - set(A))
        if not common:
            return res
        da, db = A[common[0]][0], B[common[0]][0]
        res.only_vars_a = sorted(v for v in da.variables if v not in db.variables)
        res.only_vars_b = sorted(v for v in db.variables if v not in da.variables)
        if metadata:
            res.metadata = ex.metadata(da, db)
        names = [v for v in db.variables if v in da.variables and "Time" in db[v].dimensions and v != "Times"
                 and (not variables or v in variables)]
        for v in names:
            r = VarResult(v)
            sq, n_fin = 0.0, 0
            for t in common:
                (a, ta), (b, tb) = A[t], B[t]
                if a[v].dtype.kind == "S" or b[v].dtype.kind == "S":
                    r.frames_compared += 1
                    if ex.text(a[v], ta) != ex.text(b[v], tb):
                        r.frames_differing += 1
                        r.points_differing += 1
                        r.first = r.first or f"{t}: text differs"
                    continue
                x, y = np.asarray(a[v][ta]), np.asarray(b[v][tb])
                if x.shape != y.shape:
                    r.note = f"shape {x.shape} vs {y.shape}"
                    break
                if a[v].dtype != b[v].dtype:
                    r.note = f"type {a[v].dtype} vs {b[v].dtype}"
                r.frames_compared += 1
                r.points += x.size
                d = ex.differ(x, y)
                nd = int(d.sum())
                if not nd:
                    continue
                r.frames_differing += 1
                r.points_differing += nd
                xf, yf = x.astype(np.float64), y.astype(np.float64)
                fin = np.isfinite(xf) & np.isfinite(yf)
                diff = np.abs(xf - yf)[fin]
                if diff.size:
                    r.max_abs = max(r.max_abs, float(diff.max()))
                    sq += float(np.sum(diff * diff))
                    n_fin += int(fin.sum())
                if x.dtype.kind == "f":
                    u = ulp_distance(x, y)[fin]
                    if u.size:
                        r.max_ulp = max(r.max_ulp, float(u.max()))
                if r.first is None:
                    idx = np.unravel_index(int(np.flatnonzero(d)[0]), x.shape)
                    dims = b[v].dimensions[1:]
                    where = ", ".join(f"{dn}={i}" for dn, i in zip(dims, idx))
                    r.first = f"{t} ({where}): {float(x[idx])!r} vs {float(y[idx])!r}"
            # RMSE over every compared point (identical points count as 0)
            r.rmse = math.sqrt(sq / r.points) if r.points else 0.0
            res.variables.append(r)
    finally:
        for ds in opened:
            ds.close()
    return res


def format_stream(r: StreamResult, all_vars: bool = False) -> str:
    out = [f"== {r.label}: {len(r.common_times)} common frames ({len(r.files_a)} vs {len(r.files_b)} files)"]
    if r.only_a or r.only_b:
        out.append(f"   times only in A: {r.only_a or '-'}; only in B: {r.only_b or '-'}")
    if r.only_vars_a or r.only_vars_b:
        out.append(f"   variables only in A: {r.only_vars_a or '-'}; only in B: {r.only_vars_b or '-'}")
    for m in r.metadata:
        out.append(f"   metadata: {m}")
    diff = [v for v in r.variables if not v.identical]
    if diff:
        out.append(f"   {'variable':12s} {'frames':>9s} {'points':>12s} {'max|d|':>10s} {'rmse':>10s} {'max ulp':>10s}  first")
        for v in diff:
            if v.note:
                out.append(f"   {v.name:12s} {v.note}")
                continue
            out.append(f"   {v.name:12s} {v.frames_differing:>4d}/{v.frames_compared:<4d} "
                       f"{v.points_differing:>12d} {v.max_abs:>10.3e} {v.rmse:>10.3e} {v.max_ulp:>10.3g}  {v.first}")
    n_same = sum(1 for v in r.variables if v.identical)
    out.append(f"   {n_same} of {len(r.variables)} variables identical"
               + (": IDENTICAL" if r.identical else ""))
    if all_vars:
        out.append("   identical: " + " ".join(v.name for v in r.variables if v.identical))
    return "\n".join(out)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="cppwrf-compare", description=__doc__.split("\n\n")[0] + " See --help epilog.",
                                 epilog="Exit status: 0 identical, 1 different, 2 error.")
    ap.add_argument("a", help="directory, file or glob pattern (e.g. WRF's run directory)")
    ap.add_argument("b", help="directory, file or glob pattern (e.g. CPPWRF's run directory)")
    ap.add_argument("-v", "--variables", help="comma-separated variables (default: all)")
    ap.add_argument("--exact", action="store_true", help="stop at the first stream that differs")
    ap.add_argument("--json", action="store_true", help="the results as JSON")
    ap.add_argument("--no-metadata", action="store_true", help="values only (no attribute and dimension checks)")
    ap.add_argument("--all", action="store_true", help="also list the identical variables")
    ap.add_argument("--version", action="version", version=f"cppwrf-compare {__version__}")
    a = ap.parse_args(argv)
    try:
        ps = pairs(a.a, a.b)
    except _locate.NotFound as e:
        sys.stderr.write(f"cppwrf-compare: {e}\n")
        return 2
    if not ps:
        sys.stderr.write(f"cppwrf-compare: no stream (wrfout_dNN_*, wrfrst_dNN_*, auxhistN_dNN_*) in both {a.a} and "
                         f"{a.b}\n")
        return 2
    variables = a.variables.split(",") if a.variables else None
    results, status = [], 0
    for label, fa, fb in ps:
        r = compare_stream(label, fa, fb, variables, metadata=not a.no_metadata)
        results.append(r)
        if not r.common_times:
            status = 2
        elif not r.identical:
            status = max(status, 1)
        if not a.json:
            print(format_stream(r, a.all), flush=True)
        if a.exact and status:
            break
    if a.json:
        print(json.dumps([asdict(r) | {"identical": r.identical} for r in results], indent=1))
    else:
        print("IDENTICAL" if status == 0 else ("DIFFERENT" if status == 1 else "ERROR: a stream has no common frame"))
    return status


if __name__ == "__main__":
    sys.exit(main())
