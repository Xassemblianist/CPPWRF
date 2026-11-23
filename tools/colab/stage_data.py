#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Pack the WRF reference data that the selected wrf_suite.py cases read, for a run on another machine.

  stage_data.py --only NAME[,NAME] [--files NAME[,NAME]] [--no-thompson-tables] [--whole] [--refs-only] [--no-refs]
                [--no-steps] OUT.tar|-

--only takes exact wrf_suite.py case (or special check) names. Each case's run directory (real.dir of its
configuration or of its --set overrides) is written under its own
name, so the remote side sets CPPWRF_WRF_RUNS to the unpacked directory and runs the same suite. Links are
followed. Left out: other history and restart files, rsl/log files, met_em input, executables and the
large tables no ported scheme reads; the Thompson .dat tables only for mp_physics = 8 and 28 (and not with
--no-thompson-tables: CPPWRF then computes them, close to but not bit-identical with WRF's files). The reference
wrfout files keep only the frames the cases compare, and every variable for a tolerance-0 case (wrf_suite.py
compares all of them) or else the variables tests/verify/compare_real.py compares. The ideal cases' references
come with their namelist.input (wrf_suite.py checks the configuration and the global attributes with it).
--whole: the reference files whole (every frame and variable: wrf_suite.py then compares every frame up to the case's
time; it needs no work, so OUT can be - for a tar on stdout, e.g. | gcloud storage cp - gs://BUCKET/X.tar, which
writes nothing locally); --refs-only: only the reference files (and the ideal namelists), not the run directories;
--no-refs: only the run directories, no reference file (timing runs: tools/colab/colab_run.sh --bench without
--cases); --no-steps: no reference in a steps/ directory (tools/gcp/stage_files_data.py packs those whole). A case
restarted in the middle (suite.restart_at) also takes WRF's wrfrst files at the restart, a tolerance-0 case the
auxhist3 files (output_diagnostics) next to its reference wrfout, whole. Split runs, restart checks and chains keep
the full history they compare, even when an ordinary case shares the reference.
--files: the cases of tests/verify/run_wrf_files_test.sh run with CPPWRF_FILES_NO_VALUES=1 (a GPU build): their input
directories, and their wrfout / wrfrst references (run, restart and resumed directories) as skeletons, with every
variable, dimension, attribute and time but zero numbers; whole only the wrfrst files one minute after each case's
start (derived from namelist.input), which the resumed runs start from.
"""
import argparse
import datetime
import fnmatch
import glob
import os
import re
import sys
import tarfile
import tempfile

import netCDF4 as nc
import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tests", "verify"))
import wrf_suite  # noqa: E402
from case_data import needs  # noqa: E402

WR = os.environ["CPPWRF_WRF_RUNS"]
WREF = os.environ.get("CPPWRF_WRF_REF", os.path.expanduser("~/wrf_ref480"))
SKIP = ["wrfout*", "wrfrst*", "rsl.*", "met_em*", "*.exe", "*.log", "namelist.output", "steps", "README*",
        "p3_lookup*", "ishmael*", "snicar*", "CAM_ABS_DATA", "CAM_AEROPT_DATA", "aerosol*.formatted",
        "CLM_*", "BROADBAND*", "ETAMPNEW*", "kernels*", "*.f90*", "eclipse*", "HLC.TBL",
        "tr*", "co2_trans", "grib*", "bulk*", "capacity.asc", "coeff_*", "constants.asc", "masses.asc",
        "termvels.asc", "*.nc"]
THOMPSON = ["freezeH2O.dat", "qr_acr_qg_V4.dat", "qr_acr_qsV2.dat"]
P3 = "p3_lookupTable_1.dat-v5.4_2momI"  # p3_init's table of mp_physics 50 / 51 (the other P3 tables: SKIP)
COMPARED = ["Times", "XTIME", "U", "V", "W", "PH", "PHB", "MU", "MUB", "P", "PB", "QVAPOR", "QCLOUD", "QRAIN", "QICE",
            "QSNOW", "QGRAUP", "QNICE", "QNRAIN", "TKE", "RAINNC", "THM", "REFL_10CM"]


def trimmed(src, times, dst, every_variable):
    """src with only the frames at `times` (seconds) and the compared variables (every_variable: all)."""
    with nc.Dataset(src) as a, nc.Dataset(dst, "w", format="NETCDF4") as b:
        xt = np.array(a["XTIME"][:], dtype=float) * 60.0
        frames = [i for i, t in enumerate(xt) if any(abs(t - u) < 1e-6 for u in times)]
        missing = sorted(t for t in times if not any(abs(xt[i] - t) < 1e-6 for i in frames))
        if missing:
            raise ValueError(f"{src}: no staging reference frames at {missing}")
        for n, d in a.dimensions.items():
            b.createDimension(n, None if d.isunlimited() else len(d))
        b.setncatts({k: a.getncattr(k) for k in a.ncattrs()})
        for n in (list(a.variables) if every_variable else COMPARED):
            if n not in a.variables:
                continue
            v = a[n]
            w = b.createVariable(n, v.dtype, v.dimensions, zlib=True, complevel=1)
            w.setncatts({k: v.getncattr(k) for k in v.ncattrs()})
            w[:] = v[frames] if v.dimensions and v.dimensions[0] == "Time" else v[:]


def skeleton(src, dst):
    """src with every dimension, variable, attribute and text value, its numbers zero (compressed to almost nothing)."""
    with nc.Dataset(src) as a, nc.Dataset(dst, "w", format="NETCDF4") as b:
        for n, d in a.dimensions.items():
            b.createDimension(n, None if d.isunlimited() else len(d))
        b.setncatts({k: a.getncattr(k) for k in a.ncattrs()})
        for n, v in a.variables.items():
            w = b.createVariable(n, v.dtype, v.dimensions, zlib=True, complevel=1)
            w.setncatts({k: v.getncattr(k) for k in v.ncattrs()})
            w[:] = v[:] if v.dtype.kind in "SU" else np.zeros(v.shape, v.dtype)


def files_cases():
    """{name: (config, wrfout reference dir, restart dir or None, extra --set)} of run_wrf_files_test.sh's CASES."""
    with open(os.path.join(REPO, "tests", "verify", "run_wrf_files_test.sh")) as f:
        text = f.read()
    out = {}
    for line in text.split('CASES="', 1)[1].split('\n"', 1)[0].strip().splitlines():
        name, cfg, ref, rst, extra = line.split("|")
        out[name] = (cfg, ref.replace("$R", WR), None if rst == "-" else rst.replace("$R", WR), extra.replace("$R", WR))
    return out


def config_value(cfg, section, key):
    return wrf_suite.configuration(cfg, []).get(section, {}).get(key)


def first_restart_stamp(d):
    """The YYYY-MM-DD_HH:MM:SS timestamp one minute after the start in d/namelist.input (None without one)."""
    nml = os.path.join(d, "namelist.input")
    if not os.path.isfile(nml):
        return None
    with open(nml) as f:
        nl = f.read()
    vals = []
    for k in ("year", "month", "day", "hour", "minute", "second"):
        m = re.search(rf"^\s*start_{k}\s*=\s*(\d+)", nl, re.M | re.I)
        if m:
            vals.append(int(m.group(1)))
        elif k in ("minute", "second"):
            vals.append(0)
        else:
            return None
    return (datetime.datetime(*vals) + datetime.timedelta(minutes=1)).strftime("%Y-%m-%d_%H:%M:%S")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", required=True)
    ap.add_argument("--files", default="")
    ap.add_argument("--no-thompson-tables", action="store_true")
    ap.add_argument("--whole", action="store_true")
    ap.add_argument("--refs-only", action="store_true")
    ap.add_argument("--no-refs", action="store_true")
    ap.add_argument("--no-steps", action="store_true")
    ap.add_argument("out")
    a = ap.parse_args()
    log = (lambda *x: print(*x, file=sys.stderr)) if a.out == "-" else print  # stdout may be the tar
    want = {o for o in a.only.split(",") if o}
    known = {c[0] for c in wrf_suite.CASES} | set(wrf_suite.SPECIAL)
    unknown = sorted(want - known)
    if unknown:
        sys.exit(f"no case named {', '.join(unknown)}")
    dirs = {}  # run directory -> whether Thompson tables are needed
    p3dirs = set()  # run directories of P3 cases (its lookup table)
    ref_times = {}  # reference wrfout -> compared times
    full_refs = set()  # chains and split runs need every scheduled history frame
    exact = set()  # reference wrfout files of tolerance-0 cases
    restarts = set()  # WRF's wrfrst files at the restart of restarted cases
    ideal = set()  # files under CPPWRF_WRF_REF: ideal-case soundings and reference wrfout files
    for name, _, cfg, sets, wo, t, tol in wrf_suite.CASES:
        if name not in want:
            continue
        dropin = cfg.startswith("dropin:")
        snd = None if dropin else config_value(cfg, "case", "sounding_file")
        refs = list(wo.values()) if isinstance(wo, dict) else [wo]
        if snd and "${CPPWRF_WRF_REF}" in snd or any(r.startswith(WREF) for r in refs):
            if snd:
                ideal.add(snd.replace("${CPPWRF_WRF_REF}", WREF))
            if not a.no_refs:
                ideal.update(r for r in refs if r.startswith(WREF))
                ideal.update(os.path.join(os.path.dirname(r), "namelist.input") for r in refs if r.startswith(WREF))
            log(f"{name}: ideal ({len(refs)} reference files)")
            continue
        if dropin:
            rd, nl_dir, links = wrf_suite.dropin_spec(cfg, sets)
            dirs.setdefault(nl_dir, False)
            nml = os.path.join(nl_dir, "namelist.input")
            if not os.path.isfile(nml):
                sys.exit(f"{name}: missing drop-in namelist {nml}")
            restarts.update(links)  # seeds are inputs, including for --no-refs and --refs-only
            mp = "0"
        else:
            conf = wrf_suite.configuration(cfg, sets)
            rd = conf.get("real", {}).get("dir")
            if not rd or "${CPPWRF_WRF_RUNS}" not in rd and not rd.startswith(WR):
                log(f"{name}: skipped (not a real-data case of CPPWRF_WRF_RUNS)")
                continue
            rd = rd.replace("${CPPWRF_WRF_RUNS}", WR)
            mp = conf.get("phys", {}).get("mp_physics", "0")
        chain = wrf_suite.chain_at(sets)
        if chain and chain[3]:
            chain_refs = wrf_suite.chain_references(cfg, sets, chain[3])
            refs.extend(chain_refs.values())
            for path in chain_refs.values():
                if not a.no_refs and not (a.no_steps and "/steps/" in path) and not os.path.isfile(
                        os.path.join(os.path.dirname(path), "namelist.input")):
                    sys.exit(f"{name}: missing chain namelist beside {path}")
        refs = {r for r in refs if not a.no_refs and not (a.no_steps and "/steps/" in r)}
        mid = wrf_suite.restart_at(sets)
        for r in sorted(refs):
            dirs.setdefault(os.path.dirname(r), False)
            ref_times.setdefault(r, set()).add(float(t))
            if chain or mid is not None:
                full_refs.add(r)
            dom = re.search(r"wrfout_(d\d\d)_", os.path.basename(r)).group(1)
            if mid is not None and "suite.native_restart" not in sets:
                at = wrf_suite.times_at(wrf_suite.simulation_start(r), mid)
                restarts.add(os.path.join(os.path.dirname(r), f"wrfrst_{dom}_{at}"))
            if all(x == 0.0 for x in tol.values()):
                exact.add(r)
                streams = [("auxhist3", wrf_suite.auxhist3_check(sets, cfg)),
                           ("wrfrst", wrf_suite.wrfrst_check(sets))]
                for kind, required in streams:
                    if kind == "wrfrst" and not required:
                        continue
                    pattern = os.path.join(os.path.dirname(r), f"{kind}_{dom}_*")
                    paths = glob.glob(pattern)
                    if required and not paths:
                        sys.exit(f"{name}: missing reference stream {pattern}")
                    restarts.update(paths)
        if not a.refs_only:
            dirs[rd] = dirs.get(rd, False) or (mp.strip() in ("8", "28") and not a.no_thompson_tables)
        if mp.strip() in ("50", "51") and not a.refs_only:
            p3dirs.add(rd)
        log(f"{name}: {os.path.relpath(rd, WR)}")
    for tag, cfg, _ in wrf_suite.RESTARTS:  # restart_<tag>_*: the input directory of the configuration
        if not a.refs_only and any(n in want for n in wrf_suite.SPECIAL if n.startswith(f"restart_{tag}_")):
            rd = config_value(cfg, "real", "dir").replace("${CPPWRF_WRF_RUNS}", WR)
            mp = (config_value(cfg, "phys", "mp_physics") or "0").strip()
            dirs[rd] = dirs.get(rd, False) or (mp in ("8", "28") and not a.no_thompson_tables)
            if mp in ("50", "51"):
                p3dirs.add(rd)
            log(f"restart_{tag}: {os.path.relpath(rd, WR)}")
    rwhole = {}  # the restart checks' WRF references (wrf_suite.restart_check): whole files
    for tag, cfg, _ in wrf_suite.RESTARTS:
        if a.no_refs or not any(n in want for n in wrf_suite.SPECIAL if n.startswith(f"restart_{tag}_fp64")):
            continue
        case_name, rst_ref, rsm_ref, _ = wrf_suite.RESTART_WRF[tag]
        case = next(c for c in wrf_suite.CASES if c[0] == case_name)
        rd = next((s.split("=", 1)[1] for s in case[3] if s.startswith("real.dir=")), None)
        if rd:  # the case's own input directory (restart_t1: run_T1_r8)
            dirs.setdefault(rd.replace("${CPPWRF_WRF_RUNS}", WR), False)
        ref = os.path.join(WR, rst_ref, os.path.basename(case[4])) if rst_ref else case[4]
        if not os.path.isfile(ref):
            sys.exit(f"missing restart history reference: {ref}")
        if rsm_ref:
            selected = next(n for n in sorted(want) if n.startswith(f"restart_{tag}_fp64"))
            required, _ = needs(selected)
            for pattern, must_exist in required.items():
                if not must_exist:
                    continue
                path = os.path.join(WR, os.path.relpath(pattern, "wrf_runs"))
                if not any(os.path.isfile(f) for f in glob.glob(path)):
                    sys.exit(f"{selected}: missing restart reference {path}")
        for x in [ref, os.path.join(os.path.dirname(ref), "namelist.input"),
                  os.path.join(os.path.dirname(ref), "myoutfields.txt")]:
            if os.path.isfile(x):
                rwhole[os.path.join("wrf_runs", os.path.relpath(x, WR))] = x
        for x in ([os.path.join(WR, rst_ref)] if rsm_ref else []) + ([os.path.join(WR, rsm_ref)] if rsm_ref else []):
            for f in sorted(os.listdir(x)) if os.path.isdir(x) else []:
                if re.fullmatch(r"(wrfout|wrfrst)_d\d\d_.*|namelist\.input|myoutfields\.txt", f):
                    rwhole[os.path.join("wrf_runs", os.path.relpath(os.path.join(x, f), WR))] = os.path.join(x, f)
        log(f"restart_{tag}: WRF {os.path.relpath(ref, WR)}" + (f", {rst_ref}, {rsm_ref}" if rsm_ref else ""))
    skel, whole = {}, dict(rwhole)  # files test references: skeletons, whole files (path in the tar -> file)
    fcases = files_cases()
    for name in [f for f in a.files.split(",") if f]:
        cfg, ref, rst, extra = fcases[name]
        m = re.search(r"real\.dir=(\S+)", extra)
        rd = m.group(1) if m else config_value(cfg, "real", "dir").replace("${CPPWRF_WRF_RUNS}", WR)
        mp = (config_value(cfg, "phys", "mp_physics") or "0").strip()
        dirs[rd] = dirs.get(rd, False) or (mp in ("8", "28") and not a.no_thompson_tables)
        if mp in ("50", "51"):
            p3dirs.add(rd)
        rsm = rst.replace("/rst_", "/rsm_") if rst else None
        for directory, pattern in ((ref, "wrfout_d??_*"), (rst, "wrfrst_d??_*"), (rsm, "wrfout_d??_*")):
            if directory and not any(os.path.isfile(p) for p in glob.glob(os.path.join(directory, pattern))):
                sys.exit(f"files {name}: missing {pattern} in {directory}")
        rst_at = None
        if rst and os.path.isdir(rst):
            rst_at = first_restart_stamp(rst) or (first_restart_stamp(ref) if os.path.isdir(ref) else None)
            if not rst_at:
                sys.exit(f"{rst}: cannot derive minute-1 restart time (no namelist.input)")
            if not os.path.isfile(os.path.join(rst, f"wrfrst_d01_{rst_at}")):
                sys.exit(f"{rst}: missing minute-1 restart seed at {rst_at}")
        for x in (ref, rst, rsm):
            if not x or not os.path.isdir(x):
                continue
            for f in sorted(os.listdir(x)):
                p, arc = os.path.join(x, f), os.path.join("wrf_runs", os.path.relpath(os.path.join(x, f), WR))
                if x == rst and rst_at and re.fullmatch(rf"wrfrst_d\d\d_{re.escape(rst_at)}", f):
                    whole[arc] = p
                elif re.fullmatch(r"(wrfout|wrfrst)_d\d\d_.*", f):
                    if p in ref_times:
                        sys.exit(f"{p}: a reference of the suite cases and of the files test {name}: stage them apart")
                    skel.setdefault(arc, p)
                elif f in ("namelist.input", "myoutfields.txt"):
                    whole[arc] = p
        log(f"files {name}: {os.path.relpath(rd, WR)}, {os.path.relpath(ref, WR)}"
            + (f", {os.path.relpath(rst, WR)}, {os.path.relpath(rsm, WR)}" if rst else ""))
    missing_dirs = [d for d in sorted(dirs) if not os.path.isdir(d)]
    if missing_dirs:
        sys.exit("missing run directory: " + ", ".join(missing_dirs))
    missing_refs = [p for p in sorted(set(ref_times) | restarts | ideal) if not os.path.isfile(p)]
    if missing_refs:
        sys.exit("missing reference file: " + ", ".join(missing_refs))
    tf = tarfile.open(fileobj=sys.stdout.buffer, mode="w|", dereference=True) if a.out == "-" else \
        tarfile.open(a.out, "w", dereference=True)
    added = set()
    with tf:
        for d, thompson in sorted(dirs.items()):
            for f in sorted(os.listdir(d)):
                p = os.path.join(d, f)
                if os.path.isdir(p) or (f in THOMPSON and not thompson) or \
                        (a.refs_only and p not in ref_times and p not in restarts
                         and f not in ("namelist.input", "myoutfields.txt")):
                    continue
                arc = os.path.join("wrf_runs", os.path.relpath(p, WR))
                added.add(arc)
                if arc in whole or (p in ref_times and (a.whole or p in full_refs)):
                    tf.add(p, arcname=arc, recursive=False)
                elif p in ref_times:
                    with tempfile.TemporaryDirectory() as tmp:
                        q = os.path.join(tmp, f)
                        trimmed(p, ref_times[p], q, p in exact)
                        tf.add(q, arcname=arc)
                elif p in restarts or f in THOMPSON or (f == P3 and d in p3dirs) or \
                        not any(fnmatch.fnmatch(f, s) for s in SKIP):
                    tf.add(p, arcname=arc, recursive=False)
                else:
                    added.discard(arc)
        for arc, p in sorted(whole.items()):
            if arc not in added:
                tf.add(p, arcname=arc, recursive=False)
                added.add(arc)
        for p in sorted(restarts):
            arc = os.path.join("wrf_runs", os.path.relpath(p, WR))
            if arc not in added:
                tf.add(p, arcname=arc, recursive=False)
                added.add(arc)
        for arc, p in sorted(skel.items()):
            if arc in added:
                continue
            with tempfile.TemporaryDirectory() as tmp:
                q = os.path.join(tmp, os.path.basename(p))
                skeleton(p, q)
                tf.add(q, arcname=arc)
        for p in sorted(ideal):
            tf.add(p, arcname=os.path.join("wrf_ref", os.path.relpath(p, WREF)), recursive=False)
    if a.out != "-":
        print(f"{a.out}: {os.path.getsize(a.out) / 2**20:.0f} MiB, {len(dirs)} directories")


if __name__ == "__main__":
    main()
