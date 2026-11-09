# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-post: forecaster fields from wrfout files, as NetCDF (CF-1.8) and / or GRIB2 (docs/POST.md).

    from cppwrf import post
    files = post.process("run/wrfout_d01_*", config="post.yaml", output_dir="post", fmt="both")

The diagnostics are wrf-python 1.3.4.1's, ported to numpy (wrfpy.py, frame.py); the fields and their GRIB2 / CF
identification are in products.py; the configuration in config.py."""
import argparse
import datetime
import glob
import os
import sys
import time as _time

import numpy as np

from . import config as cfgmod
from . import grib2, ncout, products
from .frame import Frame
from .grid import Grid

__all__ = ["process", "main", "Context", "fields"]


class Context:
    """What a product's compute function sees: the frame, the options, the accumulations of this time (`acc`) and
    over the last precip_hours (`interval`, None when that earlier time is not in the input), the levels."""

    def __init__(self, frame, opt, acc, interval, plevels, hlevels):
        self.frame = frame
        self.opt = opt
        self.acc = acc
        self.interval = interval
        self.plevels = plevels
        self.hlevels = hlevels
        self.shape2 = frame.raw("XLAT").shape
        self.memo = {}


def fields(cfg=None):
    """{kind: [names]} of every field cppwrf-post knows (cfg: a loaded configuration, for precip_hours)."""
    cfg = cfg or cfgmod.load()
    return {k: [p.name for p in v] for k, v in products.catalogue(cfg["options"]).items()}


def _select(cat, cfg):
    out = {}
    for kind, key in (("surface", "surface"), ("pressure", "pressure"), ("height", "height")):
        want = cfg["fields"][key]
        known = {p.name: p for p in cat[kind]}
        if want == "all" or want == ["all"]:
            out[kind] = list(cat[kind]) if kind == "surface" else list(cat[kind])
            continue
        want = [want] if isinstance(want, str) else list(want)
        bad = [n for n in want if n not in known]
        if bad:
            raise cfgmod.ConfigError(f"fields.{key}: unknown {', '.join(bad)} (known: {', '.join(known)})")
        out[kind] = [known[n] for n in want]
    if not cfg["levels"]["pressure"]:
        out["pressure"] = []
    if not cfg["levels"]["height"]:
        out["height"] = []
    return out


def _times(path):
    import netCDF4
    from ..wrfout import decode_times
    with netCDF4.Dataset(path) as ds:
        return decode_times(ds["Times"][:])


def _domain(path, attrs):
    gid = attrs.get("GRID_ID")
    if gid is not None:
        return f"d{int(gid):02d}"
    base = os.path.basename(path)
    i = base.find("_d")
    return base[i + 1:i + 4] if i >= 0 else "d01"


def process(files, config=None, output_dir=None, fmt=None, overrides=None, log=None, jobs=1):
    """Post-process wrfout files (paths, globs or a list); returns the paths written. config: a path (.yaml / .ini) or a
    configuration dict from config.load; output_dir / fmt override output.dir / output.format. log: a callable
    taking a line (default: none). jobs > 1: that many processes, each a contiguous share of the times."""
    cfg = config if isinstance(config, dict) else cfgmod.load(config, overrides)
    if output_dir is not None:
        cfg["output"]["dir"] = output_dir
    if fmt is not None:
        cfg["output"]["format"] = fmt
    say = log or (lambda s: None)
    paths = []
    for f in ([files] if isinstance(files, str) else list(files)):
        paths += sorted(glob.glob(f)) or ([f] if os.path.exists(f) else [])
    if not paths:
        raise FileNotFoundError(f"no wrfout files in {files!r}")
    # every (time, file, index), in time order, so that interval accumulations find the earlier times
    frames = sorted((t, p, i) for p in dict.fromkeys(paths) for i, t in enumerate(_times(p)))
    _select(products.catalogue(cfg["options"]), cfg)  # unknown field names fail here, before any work
    os.makedirs(cfg["output"]["dir"], exist_ok=True)
    jobs = max(1, min(int(jobs or 1), len(frames)))
    if jobs == 1:
        return _run(frames, 0, len(frames), cfg, say)
    import multiprocessing as mp
    bounds = [round(k * len(frames) / jobs) for k in range(jobs + 1)]
    with mp.get_context("fork").Pool(jobs) as pool:
        parts = pool.starmap(_run, [(frames, bounds[k], bounds[k + 1], cfg, None) for k in range(jobs)])
    for line in (ln for part in parts for ln in part[1]):
        say(line)
    return [p for part in parts for p in part[0]]


def _run(frames, start, end, cfg, say):
    """Frames[start:end] written; the earlier frames within precip_hours only read for their accumulations. Returns
    the paths (and, in a worker (say None), the log lines with them)."""
    import netCDF4
    lines = []
    out_say = say or lines.append
    opt = cfg["options"]
    sel = _select(products.catalogue(opt), cfg)
    plevels = [float(x) for x in cfg["levels"]["pressure"]]
    hlevels = [float(x) for x in cfg["levels"]["height"]]
    fmt = cfg["output"]["format"]
    outdir = cfg["output"]["dir"]
    hours = float(opt.get("precip_hours", 1))
    first = start
    while first > 0 and frames[first - 1][0] >= frames[start][0] - datetime.timedelta(hours=hours):
        first -= 1
    if first > 0:
        first -= 1  # the frame before, for the output interval
    history = {}  # (domain, time) -> accumulations
    prev_time = {}
    written = []
    skipped = set()
    open_ds = {}
    try:
        for n in range(first, end):
            t, path, idx = frames[n]
            if path not in open_ds:
                for p in list(open_ds):
                    open_ds.pop(p).close()
                open_ds[path] = netCDF4.Dataset(path)
            ds = open_ds[path]
            t0 = _time.time()
            F = Frame(ds, idx, emulate=bool(opt.get("emulate_wrfpython", True)))
            dom = _domain(path, F.attrs)
            acc = products.accumulations(F) if F.has("RAINNC") else None
            earlier = history.get((dom, t - datetime.timedelta(hours=hours)))
            interval = None
            if acc is not None and earlier is not None:
                interval = {k: acc[k] - earlier[k] for k in acc}
            if acc is not None:
                history[(dom, t)] = acc
                for key in [k for k in history if k[0] == dom and k[1] < t - datetime.timedelta(hours=hours)]:
                    del history[key]
            out_interval = (t - prev_time[dom]).total_seconds() if dom in prev_time else 0.0
            prev_time[dom] = t
            if n < start:
                continue
            ref = grib2.reference_time(F.attrs)
            ctx = Context(F, opt, acc, interval, plevels, hlevels)
            results = []
            for kind in ("surface", "pressure", "height"):
                for prod in sel[kind]:
                    if not prod.available(F):
                        if (kind, prod.name) not in skipped:
                            out_say(f"skipping {prod.name} ({kind}): the file has no {', '.join(prod.needs)}")
                            skipped.add((kind, prod.name))
                        continue
                    results.append((prod, prod.compute(ctx)))
            grid = Grid(F.attrs, F.raw("XLAT"), F.raw("XLONG"))
            lead_h = (t - ref).total_seconds() / 3600.0
            stem = cfg["output"]["filename"].format(domain=dom, time=t.strftime("%Y-%m-%d_%H:%M:%S"),
                                                    lead=f"{int(round(lead_h)):03d}")
            hist = f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} cppwrf-post {os.path.basename(path)}"
            if fmt in ("netcdf", "both"):
                out = os.path.join(outdir, stem + ".nc")
                ncout.write(out, grid, ref, t, results, plevels if sel["pressure"] else [],
                            hlevels if sel["height"] else [], cfg, hist)
                written.append(out)
            if fmt in ("grib2", "both"):
                out = os.path.join(outdir, stem + ".grib2")
                _write_grib(out, grid, ref, t, results, plevels, hlevels, cfg, F, hours * 3600.0, out_interval)
                written.append(out)
            out_say(f"{t:%Y-%m-%d_%H:%M:%S} {dom} +{lead_h:g} h: {len(results)} fields, {_time.time() - t0:.2f} s")
    finally:
        for d in open_ds.values():
            d.close()
    return written if say is not None else (written, lines)


def _write_grib(path, grid, ref, t, results, plevels, hlevels, cfg, F, interval_s, out_interval_s):
    g = cfg["grib2"]
    prec = g.get("precision") or {}
    wr = grib2.Writer(path, grid, ref, t, int(g["centre"]), int(g["subcentre"]), int(g["tables_version"]),
                      int(g["production_status"]), g.get("packing", "simple"))
    try:
        for prod, values in results:
            gr = prod.grib
            if gr is None:
                continue
            key = prod.name if prod.kind == "2d" else f"{prod.name}_{'P' if prod.kind == 'pressure' else 'Z'}"
            D = int(prec.get(key, prec.get(prod.name, prod.precision)))
            data = products.convert(np.asarray(values, dtype=np.float64), prod.units, gr["units"]) * gr["sign"]
            if gr["add_terrain"]:
                data = data + F.raw("HGT").astype(np.float64)
            if gr["stat"] == "acc_interval":
                ival = interval_s
            elif gr["stat"] == "max_output":
                ival = out_interval_s
            else:
                ival = 0
            if prod.kind == "2d":
                if gr["stat"] == "acc_interval" and not np.isfinite(data).any():
                    continue
                wr.write(data, gr, None, D, ival)
            else:
                levs = [p * 100.0 for p in plevels] if prod.kind == "pressure" else hlevels
                for k, lv in enumerate(levs):
                    wr.write(data[k], gr, lv, D, ival)
    finally:
        wr.close()


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="cppwrf-post",
        description="Forecaster fields from wrfout files (wrf-python's methods), as CF NetCDF and / or GRIB2. "
                    "docs/POST.md describes the fields, methods and the configuration file.")
    ap.add_argument("files", nargs="*", help="wrfout files (globs allowed)")
    ap.add_argument("-c", "--config", help="post.yaml or post.ini (default: the built-in defaults)")
    ap.add_argument("-o", "--output-dir", help="output directory (default: output.dir, '.')")
    ap.add_argument("--format", choices=("netcdf", "grib2", "both"), help="output format (default: output.format)")
    ap.add_argument("-j", "--jobs", type=int, default=1, help="processes, each a share of the times (default 1)")
    ap.add_argument("--list-fields", action="store_true", help="list the known fields and exit")
    ap.add_argument("--print-config", action="store_true", help="print the configuration in effect (.ini) and exit")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)
    try:
        cfg = cfgmod.load(a.config)
        if a.print_config:
            print(cfgmod.dump_ini(cfg))
            return 0
        if a.list_fields:
            cat = products.catalogue(cfg["options"])
            for kind in ("surface", "pressure", "height"):
                print(f"[{kind}]")
                for p in cat[kind]:
                    print(f"  {p.name:16s} {p.units:10s} {p.long_name}")
            return 0
        if not a.files:
            ap.error("no wrfout files")
        t0 = _time.time()
        out = process(a.files, cfg, a.output_dir, a.format, log=None if a.quiet else print, jobs=a.jobs)
        if not a.quiet:
            print(f"cppwrf-post: {len(out)} files in {_time.time() - t0:.1f} s")
        return 0
    except (cfgmod.ConfigError, FileNotFoundError, ValueError, RuntimeError) as e:
        print(f"cppwrf-post: error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
