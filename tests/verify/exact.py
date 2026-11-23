# SPDX-License-Identifier: GPL-3.0-or-later
"""Tolerance-0 comparison of model output, shared by the harness (wrf_suite.py, compare_real.py,
tests/init_real/compare_real.py, tools/gcp/matrix_variant.py).

Two values are identical when they are the same number: NaN equals NaN, NaN differs from any number, -0 differs
from +0 (WRF writes -0, e.g. in W, C1H, C1F), infinities equal themselves. netCDF variables are read as stored
(no masking of fill values, no scaling)."""
import numpy as np


CLOCK_VARS = {"Times", "XTIME"}


def raw(ds):
    """The netCDF4 dataset `ds` with masking and scaling off (values as stored)."""
    ds.set_auto_maskandscale(False)
    return ds


def _ints(x, y):
    """Both arrays integer (or boolean): compared in their own types, not via float64 (exact beyond 2**53)."""
    return x.dtype.kind in "iub" and y.dtype.kind in "iub"


def differ(x, y):
    """Mask of the points where x and y (same shape) are not the same value."""
    x, y = np.asarray(x), np.asarray(y)
    if _ints(x, y):
        return x != y
    x, y = x.astype(np.float64, copy=False), y.astype(np.float64, copy=False)
    nx, ny = np.isnan(x), np.isnan(y)
    d = (x != y) & ~(nx & ny)
    return d | ((x == 0) & (y == 0) & (np.signbit(x) != np.signbit(y)))


def nan_matched(x, y):
    """Points where x and y are both NaN (identical by the rule above, counted in the comparisons' summaries)."""
    x, y = np.asarray(x), np.asarray(y)
    if x.dtype.kind != "f" or y.dtype.kind != "f" or x.shape != y.shape:
        return 0
    return int((np.isnan(x) & np.isnan(y)).sum())


def describe(x, y):
    """None when x and y are identical, else 'n/size max|d| (first: a vs b at index)' with the NaN-vs-number and
    signed-zero point counts. Integer arrays are compared in their own types."""
    x, y = np.atleast_1d(np.asarray(x)), np.atleast_1d(np.asarray(y))
    if x.shape != y.shape:
        return f"shape {x.shape} vs {y.shape}"
    if _ints(x, y):
        d = x != y
        n = int(d.sum())
        if not n:
            return None
        i = int(np.flatnonzero(d)[0])
        return f"{n}/{x.size} integers differ (first {int(x.flat[i])} vs {int(y.flat[i])} at {i})"
    x, y = x.astype(np.float64, copy=False), y.astype(np.float64, copy=False)
    d = differ(x, y)
    n = int(d.sum())
    if not n:
        return None
    i = int(np.flatnonzero(d)[0])
    nan = int((np.isnan(x) != np.isnan(y)).sum())
    sz = int(((x == 0) & (y == 0) & (np.signbit(x) != np.signbit(y))).sum())
    with np.errstate(invalid="ignore"):
        m = np.abs(x - y)[d & ~np.isnan(x) & ~np.isnan(y)]
    return (f"{n}/{x.size} max|d| {m.max() if m.size else 0.0:.3e} (first {float(x.flat[i])!r} vs {float(y.flat[i])!r} at {i})"
            + (f" nan-vs-number {nan}" if nan else "") + (f" signed-zero {sz}" if sz else ""))


def text(v, t=None):
    """Bytes of a character variable (all of it, or frame t)."""
    a = np.asarray(v[:] if t is None else v[t])
    return b"".join(a.ravel().tolist()) if a.dtype.kind == "S" else a.tobytes()


def metadata(a, b, ignore=("TITLE",), only=None):
    """Differences of two netCDF datasets other than values: variable list and order, dimensions, each variable's
    dimensions, type and attributes, the global attributes (names in `ignore` aside) and their order.
    only=[variables of b]: b holds these variables of a full file (in any order; the dimensions they use)."""
    out = []
    va, vb = list(a.variables), list(b.variables)
    if only is not None:  # b's variable order is its writer's (stage_data.py), not WRF's
        va, vb = sorted(x for x in va if x in only), sorted(vb)
    if va != vb:
        out.append(f"variables: missing {[x for x in vb if x not in va]} extra {[x for x in va if x not in vb]}"
                   + ("" if set(va) != set(vb) else " (order)"))
    dims = lambda f: [(d, "U" if f.dimensions[d].isunlimited() else len(f.dimensions[d])) for d in f.dimensions]  # noqa: E731
    da, db = dims(a), dims(b)
    if only is not None:
        used = {d for n in vb for d in b.variables[n].dimensions}
        da, db = [d for d in da if d[0] in used], [d for d in db if d[0] in used]
    if da != db:
        out.append(f"dimensions {da} vs {db}")
    for n in vb:
        if n not in va:
            continue
        x, y = a.variables[n], b.variables[n]
        if x.dimensions != y.dimensions or x.dtype != y.dtype:
            out.append(f"{n}: dims/type {x.dimensions} {x.dtype} vs {y.dimensions} {y.dtype}")
        ax = [(k, repr(x.getncattr(k))) for k in x.ncattrs()]
        ay = [(k, repr(y.getncattr(k))) for k in y.ncattrs()]
        if ax != ay:
            out.append(f"{n}: attributes {ax} vs {ay}")
    ga = [(k, repr(a.getncattr(k)), type(a.getncattr(k)).__name__) for k in a.ncattrs() if k not in ignore]
    gb = [(k, repr(b.getncattr(k)), type(b.getncattr(k)).__name__) for k in b.ncattrs() if k not in ignore]
    if ga != gb:
        ma, mb = {k: v for k, *v in ga}, {k: v for k, *v in gb}
        dif = {k: f"{ma.get(k)} vs {mb.get(k)}" for k in sorted(set(ma) | set(mb)) if ma.get(k) != mb.get(k)}
        out.append(f"global attributes {dif}" if dif else "global attribute order differs")
    return out


def uncached(ds):
    """The dataset `ds` without HDF5 chunk caches (netCDF-4 files): each frame of a variable is read once, and the
    caches (64 MiB per variable by default) kept every chunk read until the file was closed, the whole decompressed
    file per open dataset (31 GB in the comparing process of a full suite). Values read are the same."""
    if ds.data_model.startswith("NETCDF4"):
        for v in ds.variables.values():
            v.set_var_chunk_cache(0, 0, 1.0)
    return ds


def frames(files, opened=None, duplicates=None):
    """Index a stream by Times. Record repeated timestamps when the caller requests a coverage check."""
    import netCDF4
    m = {}
    for f in sorted(files):
        ds = uncached(raw(netCDF4.Dataset(f)))
        if opened is not None:
            opened.append(ds)
        for t in range(len(ds["Times"])):
            stamp = text(ds["Times"], t).decode()
            if duplicates is not None and stamp in m:
                duplicates.append(f"{stamp}: {m[stamp][0].filepath()} and {f}")
            m[stamp] = (ds, t)
    return m


def compare_stream(cfiles, wfiles, subset=False, until=None, global_atts=True, after=None, ignore_atts=(),
                   stats=None, common_only=False, *, continuation=(), required_times=(), wrf_reference=False):
    """Compare values and metadata for one stream and domain, matching by Times.

    The coverage window includes `until` and excludes `after`. Shared frames after `until`
    must also match, although they are outside the coverage counts. With subset=True, CPPWRF may have
    additional variables or frames. With common_only=True, reference frames outside the candidate's cadence
    are optional. Every
    required_times timestamp must exist on both sides. Duplicate Times within either stream fail, including
    duplicates across input files. A continuation is a separate CPPWRF run whose frames take precedence where
    it overlaps the first leg at a restart. A WRF reference may not identify itself as CPPWRF in TITLE.
    """
    opened, dup_c, dup_w = [], [], []
    try:
        A = frames(cfiles, opened, dup_c)
        A.update(frames(continuation, opened, dup_c))
        B = frames(wfiles, opened, dup_w)
        if after is not None:
            A = {k: v for k, v in A.items() if k > after}
            B = {k: v for k, v in B.items() if k > after}
        bad, n = _compare_stream(A, B, subset, until, global_atts, ignore_atts, stats, common_only)
        if until is not None:
            extra = {k for k in A.keys() & B.keys() if k > until}
            if extra:
                extra_bad, _ = _compare_stream({k: A[k] for k in extra}, {k: B[k] for k in extra},
                                               subset, None, global_atts, ignore_atts)
                for key, detail in extra_bad.items():
                    bad.setdefault(key, detail)
        if dup_c or dup_w:
            bad["(records)"] = [f"duplicate Times in {side}: {d}"
                                for side, duplicates in (("CPPWRF", dup_c), ("reference", dup_w))
                                for d in duplicates]
        expected = set(required_times)
        missing = []
        for side, records in (("CPPWRF", A), ("WRF", B)):
            absent = expected - {k for k in records if until is None or k <= until}
            if absent:
                missing.append(f"{side} missing {sorted(absent)}")
        if missing:
            bad["(scheduled frames)"] = "; ".join(missing)
        if wrf_reference:
            wrong = [ds.filepath() for ds in {ds for ds, _ in B.values()}
                     if "CPPWRF" in str(getattr(ds, "TITLE", "")).upper()]
            if wrong:
                bad["(reference)"] = f"CPPWRF TITLE in WRF reference: {', '.join(sorted(wrong))}"
        return bad, n
    finally:
        for ds in opened:
            ds.close()


def _compare_stream(A, B, subset, until, global_atts, ignore_atts=(), stats=None, common_only=False):
    bad, nframes = {}, {}
    extra_c = sorted(A.keys() - B.keys()) if not subset else []
    if until is not None:
        A = {k: v for k, v in A.items() if k <= until}
        B = {k: v for k, v in B.items() if k <= until}
    common = sorted(set(A) & set(B))
    only_c = sorted(set(A) - set(B))
    only_w = [] if common_only else sorted(set(B) - set(A))
    if extra_c or only_w:
        bad["(times)"] = f"only-cppwrf {extra_c} only-wrf {only_w}"
    wrf_only, cppwrf_only, compared, static, nan = set(), set(), set(), set(), {}
    pairs = {}  # file pairs in frame order
    for k in common:
        (a, ta), (b, tb) = A[k], B[k]
        pairs.setdefault((a, b), None)
        cppwrf_only |= {v for v in a.variables if v not in b.variables}
        wrf_only |= {v for v in b.variables if v not in a.variables}
        for v in b.variables:
            if v not in a.variables or "Time" not in b[v].dimensions:
                continue
            compared.add(v)
            if b[v].dtype.kind == "S":
                if v != "Times" and text(a[v], ta) != text(b[v], tb):
                    bad.setdefault(v, f"{k} text differs")
                continue
            if a[v].dimensions[:1] != ("Time",):  # a variable with a Time dimension in WRF's file only
                bad.setdefault(v, f"dimensions cppwrf {a[v].dimensions} wrf {b[v].dimensions}")
                continue
            x, y = np.asarray(a[v][ta]), np.asarray(b[v][tb])  # one frame of one variable
            if x.shape != y.shape:
                if v not in bad:  # WRF's values per leading index (e.g. soil layers)
                    yl = y.reshape(y.shape[0], -1) if y.ndim > 1 else y.reshape(-1, 1)
                    bad[v] = (f"shape cppwrf {x.shape} wrf {y.shape} wrf min/max per level "
                              + " ".join(f"{r.min():.4g}/{r.max():.4g}" for r in yl))
                continue
            if a[v].dtype != b[v].dtype:
                bad.setdefault(v, f"type cppwrf {a[v].dtype} wrf {b[v].dtype}")
            dif = describe(x, y)
            if dif:
                nframes[v] = nframes.get(v, 0) + 1
                bad.setdefault(v, f"{k} {dif}")
            m = nan_matched(x, y)
            if m:
                nan[v] = nan.get(v, 0) + m
    for a, b in pairs:  # the variables without a Time dimension, of every pair of files
        for v in b.variables:
            if v not in a.variables or "Time" in b[v].dimensions:
                continue
            static.add(v)
            x, y = np.asarray(a[v][:]), np.asarray(b[v][:])
            if b[v].dtype.kind == "S" or a[v].dtype.kind == "S":
                if text(a[v]) != text(b[v]):
                    bad.setdefault(v, "text differs")
                continue
            if x.shape != y.shape or a[v].dtype != b[v].dtype:
                bad.setdefault(v, f"shape/type cppwrf {x.shape} {a[v].dtype} wrf {y.shape} {b[v].dtype}")
                continue
            dif = describe(x, y)
            if dif:
                bad.setdefault(v, f"(no Time dimension) {dif}")
            m = nan_matched(x, y)
            if m:
                nan[v] = nan.get(v, 0) + m
    if wrf_only:  # variables WRF writes and CPPWRF does not
        bad["(wrf-only-vars)"] = sorted(wrf_only)
    if cppwrf_only and not subset:  # variables CPPWRF writes and WRF does not
        bad["(cppwrf-only-vars)"] = sorted(cppwrf_only)
    for v in nframes:
        if nframes[v] > 1:
            bad[v] += f" [{nframes[v]} frames]"
    meta = []
    for a, b in pairs:
        ignore = ("TITLE",) + tuple(ignore_atts) if global_atts else set(a.ncattrs()) | set(b.ncattrs())
        m = metadata(a, b, ignore, only=list(b.variables) if subset else None)
        meta += [x for x in m if x not in meta]
    if meta:
        bad["(metadata)"] = meta
    nvars = len((compared | static) - {"Times"})
    if not common:
        bad["(frames)"] = "no common frame"
    else:
        for b in {b for _, b in pairs}:
            if not any(v not in CLOCK_VARS and b[v].dtype.kind in "fiub" for v in b.variables):
                bad.setdefault("(variables)", []).append(f"{b.filepath()}: no model fields (Times and XTIME are clocks)")
    if stats is not None:
        ref_vars = set().union(*(set(ds.variables) for ds, _ in B.values()))
        add(stats, frames=len(common), ref_frames=len(B), ref_vars=len(ref_vars - {"Times"}),
            vars=nvars, static=len(static), cppwrf_only_vars=len(cppwrf_only),
            cppwrf_only_frames=len(only_c), files=len(pairs), nan=sum(nan.values()), nan_vars=len(nan),
            global_atts=int(bool(global_atts)), streams=1)
        stats.setdefault("nan_by_var", {})
        for v, m in nan.items():
            stats["nan_by_var"][v] = stats["nan_by_var"].get(v, 0) + m
        stats.setdefault("ignored_atts", set()).update(ignore_atts if global_atts else ())
    return bad, len(common)


def add(stats, **counts):
    """Adds the counts to the dict `stats` (several streams' comparisons summed)."""
    for k, n in counts.items():
        stats[k] = stats.get(k, 0) + n


def summary(stats, removed=()):
    """One line of what compare_stream compared (the counts in `stats`, summed over streams): WRF's frames and
    variables compared of those WRF wrote, the variables without a Time dimension, CPPWRF-only variables and
    frames, the NaN points identical in both, the global attributes (or that they were not compared) and the
    variables whose differences the caller does not count (`removed`: [(variables, reason)])."""
    s = stats
    if not s.get("streams"):
        return "nothing compared"
    out = (f"{s['frames']}/{s['ref_frames']} WRF frames, {s['vars']}/{s['ref_vars']} WRF variables"
           f" ({s['static']} without Time) in {s['files']} file pairs")
    if s["cppwrf_only_vars"] or s["cppwrf_only_frames"]:
        out += f"; CPPWRF-only {s['cppwrf_only_vars']} variables, {s['cppwrf_only_frames']} frames"
    if s["nan"]:
        out += (f"; NaN at the same {s['nan']} points in both ("
                + ", ".join(f"{v} {n}" for v, n in sorted(s["nan_by_var"].items())) + ")")
    if s["global_atts"] < s["streams"]:
        out += f"; global attributes not compared in {s['streams'] - s['global_atts']} of {s['streams']} streams"
    elif s.get("ignored_atts"):
        out += f"; global attributes but {', '.join(sorted(s['ignored_atts']))}"
    for vs, why in removed:
        if vs:
            out += f"; differences not counted: {', '.join(sorted(vs))} ({why})"
    return out
