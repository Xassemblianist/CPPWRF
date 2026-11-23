#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""WRF-identity regression suite: every CPPWRF configuration that has a WRF
reference run, with the tolerance that the current code meets. Run after every change to the
dycore, boundaries, diffusion or physics coupling.

  wrf_suite.py --cpu-dp <build>/cppwrf --cuda <build>/cppwrf [--cuda-dp <build>/cppwrf] [--cpu-sp <build>/cppwrf]
               [--cuda-sp-id <build>/cppwrf]
               [--out DIR] [--only NAME[,NAME]]
               [--exact] [--jobs N]   (N CPU cases at once, each with OMP_NUM_THREADS threads) [--allow-skip] [--slow]
  wrf_suite.py --list   (every case with all its fields, then the special and restart checks)
  wrf_suite.py --status-table [FILE]   (docs/STATUS.md's table of the CPU FP64 cases; FILE: replace it there)
  wrf_suite.py --data-for NAME[,NAME]  (the data tars these cases read, comma-joined in tests/verify/vm_data.txt's
                                        order; tools/colab/colab_suite.sh --cases without --data)
The cases are defined in tests/verify/cases/ (one file per group, see its __init__.py).

Binaries: --cpu-dp the CPU FP64 build (cases cpu_dp, against WRF's real*8 build), --cpu-sp the CPU FP32 build
(-DCPPWRF_DOUBLE=OFF; cases cpu_sp, against WRF's real*4 build, also at tolerance 0), --cuda the fast CUDA FP32
build (-DCPPWRF_DOUBLE=OFF -DCPPWRF_CUDA_FP32_IDENTITY=OFF; cases cuda, compared by relative L2), --cuda-dp the CUDA
FP64 build, --cuda-sp-id the default CUDA FP32 identity build (-DCPPWRF_DOUBLE=OFF, CPPWRF_CUDA_FP32_IDENTITY=ON by
default; cases cuda_sp_id: the cpu_sp cases on the GPU, against the same WRF real*4 references at tolerance 0).
Cases with tolerance 0 (CPU FP64, CPU FP32) run with WRF history output and their wrfout files are compared with WRF's at
tolerance 0 (exact.py: NaN equals only NaN, -0 differs from +0, integers in their own types): every variable of
the reference file (those without a Time dimension too), every reference frame up to the case's time (the final one
must be there), types and variable attributes of every file, and the global attributes when the reference's
namelist.input is there (CPPWRF's history then reads it; the PASS line says when it is not). The PASS line counts
what was compared (WRF's frames and variables, CPPWRF-only variables and frames, NaN at the same points in both,
variables whose differences are not counted). The other cases compare the model state with the reference's by
relative L2 at the case's time (compare_real.py). The restart checks (cases/restarts.py) compare their resumed runs
with WRF at tolerance 0 too; the restart chains (cases/66_restart_chain.py) do where WRF has a run of their
configuration (suite.chain_wrf), else they are CPPWRF against CPPWRF only. Each PASS / FAIL line ends with what it
compared with ([vs WRF tol 0], [vs WRF rel L2], [self only]) before its wall time, and the SPLIT line before the
SUITE line counts the PASSes by it (tools/gcp/suite_status.sh counts a merged suite.log the same way).
With the reference's namelist.input a tolerance-0 case FAILs when its configuration (config file and --set) sets
an option otherwise than tools/wrf2cppwrf.py's translation of that namelist.
Only the cases of the given binaries are selected; a given binary that does not exist is an error (exit 2).
Exit status 0 = every selected case PASSed; a case whose reference is missing is a SKIP and fails the suite unless
--allow-skip. The last line counts the results.
Slow cases (suite.slow in their --set list: the 24 h and 72 h runs) are left out unless --slow is given or --only --exact
names them.
"""
import argparse
import filecmp
import glob
import os
import re
import shutil
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
# the cases, one file per group (tests/verify/cases/NN_<group>.py), and the restart checks (cases/restarts.py);
# WRF reference runs under ${CPPWRF_WRF_RUNS}, ideal-case references under ${CPPWRF_WRF_REF} (cases/common.py)
from cases import CASES, DATA, RESTARTS, STATUS  # noqa: E402
from cases.restarts import RESTART_WRF  # noqa: E402
from cases.common import WR, WREF  # noqa: E402,F401

# the checks that are not in CASES: the restart checks. (The Straka density current of configs/straka_wrf.ini, an
# analytic initial state with WRF's eta levels and p_top as rounded decimals, is no suite check: WRF's em_grav2d_x
# run_straka_d1 is the case ideal_grav2d_x_fp64_cpu / _cuda, the ideal.exe port against WRF's real*8 run at
# tolerance 0; tests/verify/verify.py wrf compares the analytic run with it by RMS.)
SPECIAL = (["restart_p1_fp32_cuda"] + [f"restart_{t}_fp64" for t, _, _ in RESTARTS]
           + [f"restart_{t}_fp64_cuda" for t, _, _ in RESTARTS])


def run(binary, cfg, sets, out, extra_env=None):
    shutil.rmtree(out, ignore_errors=True)
    cmd = [binary, os.path.join(REPO, cfg), "--set", f"output.dir={out}"]
    for s in sets:
        cmd += ["--set", s]
    env = dict(os.environ, OMP_NUM_THREADS=os.environ.get("OMP_NUM_THREADS", "3"), **(extra_env or {}))
    r = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


# a drop-in case (config "dropin:<WRF run directory>"): cppwrf-wrf in a copy of the run directory
DROPIN_OUTPUT = re.compile(r"(wrfout|wrfrst|auxhist\d+)_d\d\d_.*|rsl\..*|.*\.log|namelist\.(input|output)|(wrf|real|ideal)\.exe")


def dropin_spec(cfg, sets):
    """A drop-in case's sources: the WRF run directory whose input files it links (every file but the outputs, the
    logs, the executables and the namelists), the directory whose namelist.input and *.txt files (iofields) it copies
    (suite.dropin_namelist=DIR, default the run directory) and further files it links (suite.dropin_link=FILE: a
    restart run's wrfrst file)."""
    src = cfg[len("dropin:"):]
    nl_dir = next((x.split("=", 1)[1] for x in sets if x.startswith("suite.dropin_namelist=")), src)
    return src, nl_dir, [x.split("=", 1)[1] for x in sets if x.startswith("suite.dropin_link=")]


def dropin_run(binary, cfg, sets, out):
    """cppwrf-wrf --exe binary in `out`, a WRF-style run directory made from the case's sources (dropin_spec); the
    --set list (without suite.*) is passed on. (exit status, the end of rsl.error.0000 or the tool's output)."""
    src, nl_dir, links = dropin_spec(cfg, sets)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    for f in sorted(os.listdir(src)):
        if not DROPIN_OUTPUT.fullmatch(f):
            os.symlink(os.path.realpath(os.path.join(src, f)), os.path.join(out, f))
    for f in sorted(os.listdir(nl_dir)):
        if f == "namelist.input" or f.endswith(".txt"):
            if os.path.lexists(os.path.join(out, f)):
                os.remove(os.path.join(out, f))
            shutil.copyfile(os.path.join(nl_dir, f), os.path.join(out, f))
    for f in links:
        os.symlink(os.path.realpath(f), os.path.join(out, os.path.basename(f)))
    cmd = [sys.executable, os.path.join(REPO, "tools", "cppwrf-wrf"), "--exe", binary, "--dir", out]
    for x in sets:
        if not x.startswith("suite."):
            cmd += ["--set", x]
    env = dict(os.environ, OMP_NUM_THREADS=os.environ.get("OMP_NUM_THREADS", "3"))
    r = subprocess.run(cmd, env=env, capture_output=True, text=True)
    rsl_out, rsl_err = os.path.join(out, "rsl.out.0000"), os.path.join(out, "rsl.error.0000")
    log = open(rsl_err).read() if os.path.exists(rsl_err) else r.stdout + r.stderr
    if r.returncode == 0 and not (os.path.exists(rsl_out) and "SUCCESS COMPLETE WRF" in open(rsl_out).read()):
        return 1, "exit status 0 without SUCCESS COMPLETE WRF in rsl.out.0000"
    return r.returncode, log


def configuration(cfg, sets):
    """Effective INI options, including the last --set assignment of each key."""
    conf, sec = {}, None
    with open(os.path.join(REPO, cfg)) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if re.fullmatch(r"\[.+\]", line):
                sec = line[1:-1]
            elif sec and "=" in line:
                key, value = line.split("=", 1)
                conf.setdefault(sec, {})[key.strip()] = value.strip()
    for x in sets:
        if "=" not in x:
            continue
        qualified, value = x.split("=", 1)
        section, key = qualified.split(".", 1)
        conf.setdefault(section, {})[key] = value.strip()

    return conf


def config_vs_namelist(cfg, sets, nml):
    """Options of the case's configuration (cfg and its --set) that differ from tools/wrf2cppwrf.py's translation of
    the namelist.input of the WRF run that made the reference (options both set; paths by their real path; the run
    length, output and restart settings aside), or the translation's errors. A case with output.auxhist3=true (the
    translation of io_form_auxhist3 = 2, which needs WRF history output) is translated with --format wrfout."""
    nl = read_namelist(nml)
    from tools.wrf2cppwrf import WrfTranslator
    tr = WrfTranslator(nl, namelist_path=nml, output_format="wrfout" if auxhist3_check(sets, cfg) else "native")
    gen = tr.translate()
    if tr.errors:
        return [f"{nml} does not translate: {tr.errors}"]
    conf = configuration(cfg, sets)

    def norm(v):
        v = os.path.expandvars(str(v)).strip().strip("'\"").replace(" ", "")
        if v.startswith("/"):
            return os.path.realpath(v)
        try:
            return repr(float(v.replace("d", "e").replace("D", "e")))
        except ValueError:
            return {".true.": "true", ".false.": "false", "t": "true", "f": "false"}.get(v.lower(), v.lower())
    # a reference directory without its own wrfinput (tools/gcp/gcp_wrfref.sh links the base run's) does not name the
    # input directory
    skip = {("run", "run_time")} | (set() if os.path.exists(os.path.join(os.path.dirname(nml), "wrfinput_d01"))
                                    else {("real", "dir")})
    # nor does one without its own tables (a wrfref run on a base_* directory, whose links the data tar drops)
    if not os.path.exists(os.path.join(os.path.dirname(nml), "LANDUSE.TBL")):
        skip |= {("phys", "table_dir")}
    def same(a, b):  # files by their content (a reference directory's copy of a linked input_sounding)
        a, b = norm(a), norm(b)
        return a == b or (a.startswith("/") and os.path.isfile(a) and os.path.isfile(b) and filecmp.cmp(a, b, False))
    return [f"{s}.{k}: namelist {v} configuration {conf[s][k]}" for s, kv in gen.items() if s not in ("output", "restart")
            for k, v in kv.items() if (s, k) not in skip and k in conf.get(s, {}) and not same(v, conf[s][k])]


def restart_at(sets):
    """The time [s] of a case's restart (suite.restart_at=S in its --set list: the run stops there, writes wrfrst
    files and resumes from them), or None."""
    return next((float(x.split("=", 1)[1]) for x in sets if x.startswith("suite.restart_at=")), None)


def slow(sets):
    """A slow case (suite.slow in its --set list): run only with --slow or when --only --exact names it."""
    return "suite.slow" in sets


# netCDF4-python (1.7) releases the GIL in its netCDF-C calls and netCDF-C / HDF5 are not thread-safe: the case
# threads' in-process reads of the references and outputs (exact_case, restarted_case) take this lock (two restarted
# cases starting together read their references at once: "NetCDF: HDF error", "XTIME not found", a segfault)
_NC = threading.RLock()


def uninitialised(ds):
    """The variables of WRF's file `ds` that WRF leaves uninitialised, not compared: RQCBLTEN / RQIBLTEN without qc /
    qi, and LH with isfflx = 0 (sf_sfclayrev skips lh = xlv * qfx and copies its uninitialised local lh_hv; the LSM
    is skipped too; the file's global attribute ISFFLX)."""
    out = [x for x, q in (("RQCBLTEN", "QCLOUD"), ("RQIBLTEN", "QICE")) if q not in ds.variables]
    if "ISFFLX" in ds.ncattrs() and int(ds.getncattr("ISFFLX")) == 0:
        out.append("LH")
    return out


def stream_files(out, kind, dom):
    pattern = f"{kind}_{dom}_*"
    return sorted(glob.glob(os.path.join(out, pattern)) + glob.glob(os.path.join(out, dom, pattern)))


def auxhist3_check(sets, cfg=None):
    """Whether the case explicitly enables auxhist3 output."""
    value = "false"
    if cfg and not os.fspath(cfg).startswith("dropin:"):
        value = configuration(cfg, sets).get("output", {}).get("auxhist3", value)
    else:
        for entry in sets:
            if entry.startswith("output.auxhist3="):
                value = entry.split("=", 1)[1]
    return value.strip().lower() in ("true", "1", "yes")


def exact_case(refs, out, t, sets, after=None, ignore_atts=(), *, cfg=None, required_times=None):
    """Compare history and auxhist3 with WRF up to t, optionally requiring a specific history cadence.

    A list of output directories gives the first and resumed legs of a restarted case.
    """
    with _NC:
        return _exact_case(refs, out, t, sets, after, ignore_atts, cfg, required_times)


def _exact_case(refs, out, t, sets, after, ignore_atts, cfg, required_times):
    import netCDF4
    import exact
    bad, stats, noise, no_nl = [], {}, set(), []
    first, *resumed = out if isinstance(out, list) else [out]
    require_aux = auxhist3_check(sets, cfg)
    for dom, w in refs.items():
        d = dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1)
        with netCDF4.Dataset(w) as ds:
            xt = [float(x) * 60.0 for x in ds["XTIME"][:]]
            final = [exact.text(ds["Times"], i).decode() for i, x in enumerate(xt) if abs(x - t) < 1e-6]
            unin = [v for v in uninitialised(ds) if v in ds.variables]
        if not final:
            bad.append(f"{d}: the reference has no frame at t={t:g}s")
            continue
        has_nl = os.path.exists(os.path.join(os.path.dirname(w), "namelist.input"))
        if not has_nl:
            no_nl.append(d)
        mine = stream_files(first, "wrfout", d)
        continuation = stream_files(resumed[0], "wrfout", d) if resumed else ()
        scheduled = required_times or ()
        dif, _ = exact.compare_stream(mine, [w], subset=True, until=final[0], global_atts=has_nl, after=after,
                                      ignore_atts=ignore_atts, stats=stats, common_only=required_times is not None,
                                      continuation=continuation, required_times=scheduled, wrf_reference=True)
        for v in unin:  # the variables WRF leaves uninitialised
            if dif.pop(v, None) is not None:
                noise.add(v)
        bad += [f"{d}.{k}: {v}" for k, v in dif.items()]
        # output_diagnostics: the auxhist3 files next to the reference wrfout, compared as the wrfout files
        aux = sorted(glob.glob(os.path.join(os.path.dirname(w), f"auxhist3_{d}_*")))
        if not aux and require_aux:
            bad.append(f"{d}.auxhist3: no WRF reference files")
        if aux:
            mine = stream_files(first, "auxhist3", d)
            continuation = stream_files(resumed[0], "auxhist3", d) if resumed else ()
            dif, n = exact.compare_stream(mine, aux, subset=True, until=final[0], global_atts=has_nl, after=after,
                                          ignore_atts=ignore_atts, stats=stats, common_only=required_times is not None,
                                          continuation=continuation, wrf_reference=True)
            if not n:
                dif.setdefault("(times)", "no auxhist3 frame in common")
            bad += [f"{d}.auxhist3.{k}: {v}" for k, v in dif.items()]
    what = exact.summary(stats, [(noise, "WRF leaves them uninitialised")])
    if no_nl:
        what += f" (no namelist.input next to the {', '.join(no_nl)} reference: global attributes not compared)"
    return bad, what


def wrfrst_check(sets):
    """A case that also compares its wrfrst files (suite.wrfrst in its --set list): wrfrst_case."""
    return "suite.wrfrst" in sets


def wrfrst_case(refs, out, skip=()):
    """The wrfrst files of a run against WRF's: the reference run's directory (next to its wrfout) and `out` (or
    out/<domain>) must hold the same set of wrfrst_<domain>_<date> files (the restart alarm's times), each identical at
    tolerance 0 (every variable and attribute, WRF_ALARM_SECS_TIL_NEXT_RING_* among them; RQCBLTEN / RQIBLTEN not when
    WRF's file has no QCLOUD / QICE). `skip`: file names that are inputs (a resumed run's linked restart file):
    (differences, summary)."""
    import exact
    bad, n, stats, noise = [], 0, {}, set()
    for dom, w in refs.items():
        d = dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1)
        ref = {os.path.basename(f): f for f in glob.glob(os.path.join(os.path.dirname(w), f"wrfrst_{d}_*"))
               if os.path.basename(f) not in skip}
        mine = {os.path.basename(f): f for f in stream_files(out, "wrfrst", d) if os.path.basename(f) not in skip}
        if not ref:
            bad.append(f"{d}: no WRF wrfrst reference files")
        if sorted(ref) != sorted(mine):
            bad.append(f"{d}: wrfrst files {sorted(x[-8:] for x in mine)}, WRF's {sorted(x[-8:] for x in ref)}")
        for name in sorted(set(ref) & set(mine)):
            b2, _ = wrfrst_pair(mine[name], ref[name], stats, noise)
            bad += [f"{name}.{x}" for x in b2]
            n += 1
    return bad, f"{n} wrfrst files" + (" identical" if not bad else "") + f" ({exact.summary(stats, [(noise, 'WRF leaves them uninitialised')])})"


def wrfrst_pair(mine, ref, stats, noise):
    """One wrfrst file of CPPWRF's against WRF's at tolerance 0 (every variable, attribute and global attribute;
    the variables WRF leaves uninitialised, wrf_suite.uninitialised, not counted: added to `noise` when they
    differ): ([differences], number of frames); `stats`: exact.compare_stream's counts."""
    import netCDF4
    import exact
    with _NC:
        with netCDF4.Dataset(ref) as ds:
            unin = uninitialised(ds)
        dif, n = exact.compare_stream([mine], [ref], stats=stats, wrf_reference=True)
    noise.update(k for k in dif if k in unin)
    return [f"{k}: {v}" for k, v in dif.items() if k not in unin], n


def start_dates(files, want, *, restart_alarm=False):
    """Report incorrect START_DATE and, for a restart stream, its run-start alarm."""
    from datetime import datetime
    import netCDF4
    import exact
    out = []
    with _NC:
        for f in files:
            with netCDF4.Dataset(f) as ds:
                v = getattr(ds, "START_DATE", None)
                if v != want:
                    out.append(f"{os.path.basename(f)} START_DATE {v!r}, the expected start time {want}")
                if restart_alarm:
                    stamp = exact.text(ds["Times"], 0).decode()
                    elapsed = (datetime.fromisoformat(stamp) - datetime.fromisoformat(want)).total_seconds()
                    key = "WRF_ALARM_SECS_TIL_NEXT_RING_55"
                    actual = getattr(ds, key, None)
                    if actual != -elapsed:
                        out.append(f"{os.path.basename(f)} {key} {actual}, expected {-elapsed:g}")
    return out


def restarted_case(b, cfg, sets, refs, t, mid, out, native=False):
    """A run to `t` s restarted at `mid` s from its own wrfrst files: the first run (to one step past mid, wrfrst files
    at mid only: written within the run, as WRF's, not at its last step, where WRF's history alarm is still ringing)
    and the resumed one (mid to t) together against WRF's uninterrupted run at tolerance 0 (every reference frame;
    START_DATE, which a run resumed from a wrfrst file sets to the restart time as WRF's resumed runs do, is checked
    to be that; a run resumed from CPPWRF's own file continues the run that wrote it: every attribute compared),
    and the wrfrst files at mid against WRF's (the reference run's restart_interval writes them there):
    (differences, summary). As run_wrf_files_test.sh, RQCBLTEN / RQIBLTEN are not compared when WRF's restart file
    has no QCLOUD / QICE (WRF leaves them uninitialised)."""
    import netCDF4
    import exact
    first, resumed = os.path.join(out, "a_first"), os.path.join(out, "b_resumed")  # sorted: resumed frames win
    w0 = next(iter(refs.values()))

    def resumed_files():
        return glob.glob(os.path.join(resumed, "wrfout_d??_*")) + glob.glob(os.path.join(resumed, "d??", "wrfout_d??_*"))
    with _NC, netCDF4.Dataset(w0) as ds:
        at = [exact.text(ds["Times"], i).decode() for i, x in enumerate(ds["XTIME"][:]) if abs(float(x) * 60.0 - mid) < 1e-6]
    if native:  # suite.native_restart: resumed from CPPWRF's own restart_<step>.nc (mid need not be a frame time)
        nl = os.path.join(os.path.dirname(w0), "namelist.input")
        dt = float(re.search(r"^\s*time_step\s*=\s*(\d+)", open(nl).read(), re.M).group(1))
        rc, log = run(b, cfg, sets + [f"run.run_time={mid + dt:g}", f"restart.interval={mid:g}", f"restart.end={mid:g}"],
                      first)
        if rc != 0:
            return [f"run to the restart failed: {log[-200:]}"], ""
        rf = os.path.join(first, f"restart_{round(mid / dt):06d}.nc")
        if not os.path.exists(rf):
            return [f"no {rf}"], ""
        rc, log = run(b, cfg, sets + [f"run.restart_file={rf}"], resumed)
        if rc != 0:
            return [f"resumed run failed: {log[-200:]}"], ""
        # (CPPWRF's own file continues the run that wrote it: START_DATE is that run's, every attribute compared)
        bad, what = exact_case(refs, [first, resumed], t, sets, cfg=cfg)
        return bad, f"{what}, restarted at {mid:g}s from restart_{round(mid / dt):06d}.nc"
    if not at:
        return [f"{w0} has no frame at the restart time {mid:g}s"], ""
    nl = os.path.join(os.path.dirname(w0), "namelist.input")
    dt = float(re.search(r"^\s*time_step\s*=\s*(\d+)", open(nl).read(), re.M).group(1))
    rc, log = run(b, cfg, sets + [f"run.run_time={mid + dt:g}", f"restart.interval={mid:g}", f"restart.end={mid:g}"],
                  first)
    if rc != 0:
        return [f"run to the restart failed: {log[-200:]}"], ""
    # a nested run writes each domain's files in first/<domain>; the resumed run reads every domain's wrfrst file
    # from the directory of d01's (as WRF's run directory)
    rst = os.path.join(out, "rst")
    os.makedirs(rst, exist_ok=True)
    for pat in (f"wrfrst_d??_{at[0]}", f"d??/wrfrst_d??_{at[0]}"):
        for f in glob.glob(os.path.join(first, pat)):
            os.symlink(f, os.path.join(rst, os.path.basename(f)))
    rc, log = run(b, cfg, sets + [f"run.restart_file={rst}/wrfrst_d01_{at[0]}"], resumed)
    if rc != 0:
        return [f"resumed run failed: {log[-200:]}"], ""
    bad, what = exact_case(refs, [first, resumed], t, sets, ignore_atts=("START_DATE",), cfg=cfg)
    bad += start_dates(resumed_files(), at[0])
    stats, noise = {}, set()
    for dom, w in refs.items():
        d = dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1)
        wr = os.path.join(os.path.dirname(w), f"wrfrst_{d}_{at[0]}")
        mine = glob.glob(os.path.join(first, f"wrfrst_{d}_{at[0]}")) + glob.glob(os.path.join(first, d, f"wrfrst_{d}_{at[0]}"))
        if not os.path.exists(wr) or not mine:
            bad.append(f"{d}: no wrfrst_{d}_{at[0]} from {'WRF' if not os.path.exists(wr) else 'CPPWRF'}")
            continue
        b2, _ = wrfrst_pair(mine[0], wr, stats, noise)
        bad += [f"{d}.wrfrst.{x}" for x in b2]
    return bad, (f"{what}, restarted at {mid:g}s; wrfrst at {mid:g}s vs WRF's: "
                 f"{exact.summary(stats, [(noise, 'WRF leaves them uninitialised')])}")


def times_at(start, secs):
    """The Times string `secs` seconds after `start` (a Times string)."""
    import datetime
    t0 = datetime.datetime.strptime(start, "%Y-%m-%d_%H:%M:%S")
    return (t0 + datetime.timedelta(seconds=secs)).strftime("%Y-%m-%d_%H:%M:%S")


def chain_at(sets):
    """A restart-chain case (suite.chain=MID,INTERVAL[,native] in its --set list: restart_chain), (MID, INTERVAL
    [s], native only, the WRF reference of the same configuration (suite.chain_wrf=WRFOUT) or None), or None."""
    v = next((x.split("=", 1)[1] for x in sets if x.startswith("suite.chain=")), None)
    if not v:
        return None
    f = v.split(",")
    return float(f[0]), float(f[1]), f[2:] == ["native"], \
        next((x.split("=", 1)[1] for x in sets if x.startswith("suite.chain_wrf=")), None)


def chain_history(conf, doms, t):
    """History cadence in seconds on each configured chain domain."""
    dt = {"d01": float(conf["time"]["dt"])}
    cadence = {"d01": dt["d01"] * int(conf.get("output", {}).get("out_interval", round(t / dt["d01"])))}
    for d in doms[1:]:
        domains = conf.get("domains", {})
        opts = {k.removeprefix(d + "_"): v for k, v in domains.items() if k.startswith(d + "_")}
        opts.update(conf.get(d, {}))
        parent = f"d{int(opts.get('parent_id', 1)):02d}"
        ratio = opts.get("parent_time_step_ratio", domains.get("parent_time_step_ratio", 1))
        dt[d] = float(opts.get("dt", dt[parent] / int(ratio)))
        cadence[d] = dt[d] * int(opts["out_interval"]) if "out_interval" in opts else cadence[parent]
    return cadence


def chain_self_stream(straight, resumed, mid, *, restart, wrfrst_resume):
    """Compare all records after the split and the restart alarm metadata."""
    import exact
    # Alarm 55 counts from the start of its leg. A wrfrst resume sets START_DATE to that time.
    ignore = ("START_DATE", "WRF_ALARM_SECS_TIL_NEXT_RING_55") if wrfrst_resume else ()
    with _NC:
        return exact.compare_stream(resumed, straight, after=mid, global_atts=restart, ignore_atts=ignore)


def restart_chain(b, cfg, sets, refs, t, mid, iv, out, native_only=False, wrf=None):
    """The restart contract without a WRF restarted run: CPPWRF's run to `t` s with restart files every `iv` s
    (wrfrst and, for one domain, CPPWRF's own restart_<step>.nc) and the runs resumed at `mid` s from them (each
    domain's wrfrst file; the native file) must write the same files after `mid`: every wrfout frame and every
    wrfrst file later than `mid`, each variable at tolerance 0 (restart-stream state a resumed run lost shows in
    its next wrfrst file even when the history does not use it). `refs`: the WRF run whose inputs (and
    namelist.input, when there) the case reads. native_only (suite.chain=MID,INTERVAL,native): the native resume
    alone, where WRF's own resumed run is not the straight one. wrf (suite.chain_wrf=WRFOUT): WRF's run of the same
    configuration (the case FAILs when its namelist.input translates otherwise): the straight run's frames and each
    resumed run's frames after `mid` against its frames up to `t` at tolerance 0 too (the wrfrst resume's START_DATE
    is the restart time, as in WRF's resumed runs, checked; the native resume continues the straight run: every
    attribute compared); without it the chain is CPPWRF against CPPWRF only and its
    PASS line says so. (differences, summary)."""
    import netCDF4
    import exact
    nl = os.path.join(os.path.dirname(next(iter(refs.values()))), "namelist.input")
    common = sets + ["output.format=wrfout", f"run.run_time={t:g}", f"restart.interval={iv:g}"] + \
        ([f"output.wrf_namelist={nl}"] if os.path.exists(nl) else [])
    straight = os.path.join(out, "a_straight")
    rc, log = run(b, cfg, common, straight)
    if rc != 0:
        return [f"straight run failed: {log[-200:]}"], ""

    conf = configuration(cfg, sets)
    doms = sorted({f"d{i:02d}" for i in range(1, int(conf.get("domains", {}).get("max_dom", 1)) + 1)}
                  | {d or "d01" for d in refs})
    emitted = {os.path.basename(f).split("_")[1]
               for f in glob.glob(os.path.join(straight, "**", "wrfrst_d??_*"), recursive=True)}
    unexpected = emitted - set(doms)
    if unexpected:
        return [f"straight run wrote wrfrst files for unexpected domains {', '.join(sorted(unexpected))}"], ""
    at = {}
    start = None
    for d in doms:
        for f in stream_files(straight, "wrfrst", d):
            with _NC, netCDF4.Dataset(f) as ds:
                if abs(float(ds["XTIME"][0]) * 60.0 - mid) < 1e-6:
                    if d in at:
                        return [f"{d}: multiple straight-run wrfrst files at {mid:g}s"], ""
                    at[d] = f
                    if d == "d01":
                        start = exact.text(ds["Times"], 0).decode()
    missing = sorted(set(doms) - at.keys())
    if missing:
        return [f"the straight run wrote no wrfrst files at {mid:g}s for {', '.join(missing)}"], ""
    rst = os.path.join(out, "rst")
    os.makedirs(rst, exist_ok=True)
    for d in doms:
        os.symlink(at[d], os.path.join(rst, os.path.basename(at[d])))
    resumed = [] if native_only else [("wrfrst", os.path.join(out, "b_resumed"),
                                       os.path.join(rst, os.path.basename(at["d01"])))]
    if doms == ["d01"]:  # CPPWRF's own restart file of the same step (nests resume from wrfrst files only)
        dt = float(conf["time"]["dt"])
        resumed.append(("native", os.path.join(out, "c_native"),
                        os.path.join(straight, f"restart_{round(mid / dt):06d}.nc")))
    bad, nfr, nrst, bad_cropcat = [], 0, 0, []
    for what, d_out, f in resumed:
        rc, log = run(b, cfg, common + [f"run.restart_file={f}"], d_out)
        if rc != 0:
            bad.append(f"{what}: resumed run failed: {log[-200:]}")
            continue
        for dom in doms:
            for kind in ("wrfout", "wrfrst"):
                dif, n = chain_self_stream(stream_files(straight, kind, dom), stream_files(d_out, kind, dom),
                                           start, restart=kind == "wrfrst", wrfrst_resume=what == "wrfrst")
                nfr, nrst = (nfr + n, nrst) if kind == "wrfout" else (nfr, nrst + n)
                # NOAHMP_INIT on a restart: CROPCAT (an output) restarts at undefined_int (PhysicsSuite::restarted)
                bad += [f"{what}: {dom}.{kind}.{k}: {v}" for k, v in dif.items() if k != "CROPCAT"]
                bad_cropcat += [k for k in dif if k == "CROPCAT"]
    cropcat = any("CROPCAT" in x for x in bad_cropcat)
    if bad:  # where it starts: CPPWRF's own restart files one step after mid, straight and resumed (every variable)
        diag = chain_first_step(b, cfg, common, resumed, mid, out)
        print("\n".join(f"  [{cfg}] {x}" for x in diag), flush=True)
        bad = diag + bad
    what = (f"self: resumed at {mid:g}s ({', '.join(w for w, _, _ in resumed)}) = CPPWRF straight: {nfr} wrfout frames, "
            f"{nrst} wrfrst files" + ("; CROPCAT differences not counted (NOAHMP_INIT on a restart)" if cropcat else ""))
    if not wrf:
        return bad, what + "; vs WRF: none (no WRF run of this configuration: self-consistency only)"
    # against WRF's run of the same configuration: the straight run, and each resumed run after mid
    nlw = os.path.join(os.path.dirname(wrf), "namelist.input")
    dif = config_vs_namelist(cfg, sets, nlw) if os.path.exists(nlw) else [f"no {nlw}"]
    if dif:
        return bad + [f"configuration differs from {nlw}: {'; '.join(dif)}"], what
    # Every configured domain, each against WRF's file of that domain next to d01's (wrfout_d02_... beside
    # wrfout_d01_...) up to its last frame by t (a nest's history interval may be longer than d01's)
    wdom = {d: wrf.replace("wrfout_d01_", f"wrfout_{d}_") for d in doms}
    missing = [w for w in wdom.values() if not os.path.exists(w)]
    if missing:
        return bad + [f"no WRF reference {', '.join(missing)}"], what
    cadence = chain_history(conf, doms, t)
    tw, scheduled, split, starts = {}, {}, {}, {}
    for d, w in wdom.items():
        with _NC, netCDF4.Dataset(w) as ds:
            xt = [float(x) * 60.0 for x in ds["XTIME"][:]]
            reference_start = ds.getncattr("SIMULATION_START_DATE")
        within = [x for x in xt if x <= t + 1e-6]
        if not within:
            return bad + [f"{d}: no WRF history through {t:g}s"], what
        tw[d] = max(within)
        starts[d] = reference_start
        interval = cadence[d]
        scheduled[d] = [times_at(reference_start, i * interval) for i in range(int((t + 1e-6) / interval) + 1)]
        split[d] = times_at(reference_start, mid)
    tag = (lambda d: f"{d} ") if len(wdom) > 1 else (lambda d: "")
    w2 = []
    for d, w in wdom.items():
        b2, s2 = exact_case({d: w}, straight, tw[d], common, cfg=cfg, required_times=scheduled[d])
        bad += [f"straight vs WRF: {x}" for x in b2]
        w2.append(tag(d) + s2)
    for d in doms:
        bad += start_dates(stream_files(straight, "wrfrst", d), starts[d], restart_alarm=True)
    what += f"; vs WRF {os.path.relpath(wrf, WR)}{' (every domain)' if len(wdom) > 1 else ''} at tolerance 0: " \
        f"straight {', '.join(w2)}"
    for w, d_out, _ in resumed:  # a wrfrst resume's START_DATE is the restart time (WRF's), a native one's the run's
        w2 = []
        for d, wf in wdom.items():
            b2, s2 = exact_case({d: wf}, d_out, tw[d], common, after=split[d], cfg=cfg,
                                ignore_atts=("START_DATE",) if w == "wrfrst" else (),
                                required_times=[x for x in scheduled[d] if x > split[d]])
            bad += [f"{w} vs WRF: {x}" for x in b2]
            w2.append(tag(d) + s2)
        for d in doms:
            if w == "wrfrst":
                bad += start_dates(stream_files(d_out, "wrfout", d), split[d])
            bad += start_dates(stream_files(d_out, "wrfrst", d), split[d] if w == "wrfrst" else starts[d],
                               restart_alarm=True)
        what += f"; {w} resumed {', '.join(w2)}"
    return bad, what


def chain_first_step(b, cfg, common, resumed, mid, out):
    """restart_chain's diagnosis of a difference: the runs to one step past `mid` (the straight one, and each resumed
    one from its file) write CPPWRF's restart file there (the last step); their variables that differ."""
    import netCDF4
    import numpy as np
    ini_dt = [x.split("=", 1)[1] for x in common if x.startswith("time.dt=")]
    import configparser
    ini = configparser.ConfigParser(interpolation=None)
    ini.read(os.path.join(REPO, cfg))
    dt = float(ini_dt[0] if ini_dt else ini.get("time", "dt"))
    one = [x for x in common if not x.startswith(("run.run_time=", "restart.interval="))] + \
        [f"run.run_time={mid + dt:g}", f"restart.interval={mid:g}", "output.out_interval=100000"]
    name = f"restart_{round(mid / dt) + 1:06d}.nc"
    ref = os.path.join(out, "x_straight1")
    dumps = {"straight": os.path.join(out, "dump_straight")}
    os.makedirs(dumps["straight"], exist_ok=True)
    rc, log = run(b, cfg, one, ref, {"CPPWRF_VM_DUMP": dumps["straight"],
                                     "CPPWRF_VM_DUMP_STEP": str(round(mid / dt) + 1)})
    if rc != 0 or not os.path.exists(os.path.join(ref, name)):
        return [f"first step: straight run failed: {log[-200:]}"]
    res = []
    for what, _, f in resumed:
        d = os.path.join(out, f"x_{what}1")
        dumps[what] = os.path.join(out, f"dump_{what}")
        os.makedirs(dumps[what], exist_ok=True)
        rc, log = run(b, cfg, one + [f"run.restart_file={f}"], d,
                      {"CPPWRF_VM_DUMP": dumps[what], "CPPWRF_VM_DUMP_STEP": "1"})
        if rc != 0 or not os.path.exists(os.path.join(d, name)):
            res.append(f"first step: {what} run failed: {log[-200:]}")
            continue
        with _NC, netCDF4.Dataset(os.path.join(ref, name)) as A, netCDF4.Dataset(os.path.join(d, name)) as B:
            dif = []
            for v in A.variables:
                if v not in B.variables:
                    continue
                x, y = np.asarray(A[v][:]), np.asarray(B[v][:])
                if x.shape != y.shape or np.array_equal(x, y, equal_nan=True):
                    continue
                bad_at = np.argwhere(~((x == y) | (np.isnan(x) & np.isnan(y))))
                lo, hi = bad_at.min(axis=0), bad_at.max(axis=0)
                dif.append(f"{v}[{bad_at.shape[0]} pts, idx {tuple(int(a) for a in lo)}..{tuple(int(a) for a in hi)} "
                           f"of {x.shape}, max|d| {float(np.nanmax(np.abs(x - y))):.1e}]")
        res.append(f"first step after {mid:g}s ({what}): {' '.join(dif) if dif else 'identical'}")
        res += [f"stage dumps ({what}): {x}" for x in compare_dumps(dumps["straight"], dumps[what])]
    return res


def compare_dumps(a, b):
    """The CPPWRF_VM_DUMP files (core/vm_dump.hpp) of two runs, in the order the first run wrote them: for each file
    whose arrays differ, the arrays (by position) with their differing points ((i, j, k) ranges; raw dumps include
    the halos)."""
    import numpy as np

    def arrays(path):
        buf, out, p = open(path, "rb").read(), [], 0
        raw = os.path.basename(path).startswith("vm_raw")
        while p < len(buf):
            if raw and len(buf) - p >= 52:
                wi0, wi1, wj0, wj1, nz = np.frombuffer(buf, np.int32, 5, p)
                sj, sk, off, tot = np.frombuffer(buf, np.int64, 4, p + 20)
                v = np.frombuffer(buf, np.float64, int(tot), p + 52)
                p += 52 + 8 * int(tot)
                out.append(("raw", (int(wi0), int(wi1), int(wj0), int(wj1), int(nz), int(sj), int(sk), int(off)), v))
            elif not raw and len(buf) - p >= 12:
                nx, ny, nz = np.frombuffer(buf, np.int32, 3, p)
                n = int(nx) * int(ny) * int(nz)
                if n <= 0 or p + 12 + 8 * n > len(buf):
                    out.append(("tail", None, np.frombuffer(buf[p:], np.uint8)))
                    break
                out.append(("log", (int(nx), int(ny), int(nz)), np.frombuffer(buf, np.float64, n, p + 12)))
                p += 12 + 8 * n
            else:
                out.append(("tail", None, np.frombuffer(buf[p:], np.uint8)))
                break
        return out

    def where(kind, shape, idx):
        if kind == "log":  # j, k, i order
            nx, ny, nz = shape
            j, r = divmod(idx, nz * nx)
            k, i = divmod(r, nx)
            return np.stack([i, j, k], 1)
        if kind == "raw":
            wi0, wi1, wj0, wj1, nz, sj, sk, off = shape
            pts = [(i, j, k) for k in range(nz) for j in range(wj0, wj1) for i in range(wi0, wi1)]
            m = {off + i + j * sj + k * sk: (i, j, k) for i, j, k in pts}
            return np.array([m.get(int(q), (99999, 99999, 99999)) for q in idx])
        return np.stack([idx, idx, idx], 1)

    res = []
    files = sorted(glob.glob(os.path.join(a, "*.bin")), key=lambda f: os.stat(f).st_mtime_ns)
    for fa in files:
        fb = os.path.join(b, os.path.basename(fa))
        if not os.path.exists(fb):
            res.append(f"{os.path.basename(fa)}: missing")
            continue
        A, B = arrays(fa), arrays(fb)
        d = []
        for n, ((ka, sa, xa), (kb, sb, xb)) in enumerate(zip(A, B)):
            if sa != sb or xa.shape != xb.shape:
                d.append(f"#{n} shape {sa} vs {sb}")
                continue
            bad = np.flatnonzero(~((xa == xb) | (np.isnan(xa) & np.isnan(xb))))
            if bad.size:
                w = where(ka, sa, bad)
                d.append(f"#{n}[{bad.size} pts i {w[:, 0].min()}..{w[:, 0].max()} j {w[:, 1].min()}..{w[:, 1].max()} "
                         f"k {w[:, 2].min()}..{w[:, 2].max()} max|d| {float(np.nanmax(np.abs(xa[bad] - xb[bad]))):.1e}]")
        if len(A) != len(B):
            d.append(f"{len(A)} vs {len(B)} arrays")
        if d:
            res.append(f"{os.path.basename(fa)}: {' '.join(d)}")
    return res or ["identical"]


def compare(wrfout, out, t):
    r = subprocess.run([sys.executable, os.path.join(REPO, "tests/verify/compare_real.py"), wrfout, out],
                       capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(f"L2 comparator exited {r.returncode}: {r.stderr.strip() or r.stdout.strip()}")
    res = {}
    for line in r.stdout.splitlines():
        f = line.split()
        if len(f) < 5:
            continue
        try:
            row_time = float(f[0])
        except ValueError:  # the header
            continue
        if abs(row_time - t) < 1e-6:
            res[f[1]] = float(f[3])
    return res


def restart_p1(binary, out, cfg="configs/real_p1_r8.ini", run_dir="run_P1_r8"):
    """Physics/TKE restart: 3 steps straight vs restarted after step 1 must be bit-identical
    (dycore, TKE, surface/soil state, R*BLTEN, lagged t_1; for P7 also R*CUTEN and RTHFTEN/RQVFTEN)."""
    if not os.path.exists(f"{WR}/{run_dir}/steps/wrfinput_d01") and not os.path.exists(f"{WR}/{run_dir}/wrfinput_d01"):
        return None, "input missing"
    a, b = os.path.join(out, "straight"), os.path.join(out, "restarted")
    rc, log = run(binary, cfg, ["restart.interval=60"], a)
    if rc != 0:
        return False, log[-300:]
    rc, log = run(binary, cfg, [f"run.restart_file={a}/restart_000001.nc"], b)
    if rc != 0:
        return False, log[-300:]
    same = open(f"{a}/snap_000003.bin", "rb").read() == open(f"{b}/snap_000003.bin", "rb").read()
    return same, "step 3 bit-identical" if same else "step 3 differs after restart"


def read_namelist(path):
    sys.path.insert(0, REPO)
    from tools.wrf2cppwrf import parse_namelist
    with open(path) as f:
        return parse_namelist(f.read())


def namelist_restart_interval(nml):
    """Return the translated restart interval in seconds or fail on unsupported settings."""
    nl = read_namelist(nml)
    from tools.wrf2cppwrf import WrfTranslator
    tr = WrfTranslator(nl, namelist_path=nml, output_format="wrfout")
    gen = tr.translate()
    errors = [err for err in tr.errors if not err.startswith("WRF restart = .true. is not translated:")]
    if errors:
        raise ValueError(f"{nml}: {'; '.join(errors)}")
    return float(gen.get("restart", {}).get("interval", 0))


def resumed_namelist(rsm, rst, out):
    """Stage the resumed namelist beside its effective iofields files when some come from rst."""
    nml = os.path.join(rsm, "namelist.input")
    names = read_namelist(nml).get("time_control", {}).get("iofields_filename", [])
    sources = {}
    for name in names:
        if not name or name.upper() == "NONE_SPECIFIED":
            continue
        src = os.path.join(rsm, name)
        if not os.path.isfile(src):
            src = os.path.join(rst, name)
            if not os.path.isfile(src):
                return None, f"{nml} names iofields_filename {name}: in neither {rsm} nor {rst}"
        if not os.path.isabs(name):
            sources[name] = src
    if all(src == os.path.join(rsm, name) for name, src in sources.items()):
        return nml, ""
    d = os.path.join(out, "rsm_namelist")
    os.makedirs(d, exist_ok=True)
    shutil.copyfile(nml, os.path.join(d, "namelist.input"))
    for name, src in sources.items():
        dst = os.path.join(d, name)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src, dst)
    return os.path.join(d, "namelist.input"), ""


def compare_seeded_restart(resumed, rsm, start, t):
    """Compare the minute-1 WRF-seeded leg with the references made by make_resumed_refs.sh."""
    import netCDF4
    import exact
    # These runs write history every minute. The seed's restart alarm first rings at minute 2.
    end = times_at(start, t)
    seed = times_at(start, 60)
    expected = [times_at(start, sec) for sec in range(120, int(t) + 1, 60)]
    history = stream_files(rsm, "wrfout", "d01")
    stats, noise = {}, set()
    with _NC:
        ignored_by_file = []
        for w in history:
            with netCDF4.Dataset(w) as ds:
                ignored_by_file.append(set(uninitialised(ds)))
        ignored = set.intersection(*ignored_by_file) if ignored_by_file else set()
        dif, _ = exact.compare_stream(stream_files(resumed, "wrfout", "d01"), history, subset=True,
                                      until=end, stats=stats, required_times=expected, wrf_reference=True)
    noise.update(ignored.intersection(dif))
    bad = [f"wrfout {k}: {v}" for k, v in dif.items() if k not in ignored]
    ref, mine = [{os.path.basename(f): f for f in stream_files(directory, "wrfrst", "d01") if seed < f[-19:] <= end}
                 for directory in (rsm, resumed)]
    required = f"wrfrst_d01_{times_at(start, 120)}"
    if required not in ref:
        bad.append(f"missing WRF restart reference {required}")
    if ref.keys() != mine.keys():
        bad.append(f"wrfrst missing from CPPWRF {sorted(ref.keys() - mine.keys())}, "
                   f"missing from WRF {sorted(mine.keys() - ref.keys())}")
    shared = sorted(ref.keys() & mine.keys())
    for name in shared:
        dif, _ = wrfrst_pair(mine[name], ref[name], stats, noise)
        bad += [f"{name}.{x}" for x in dif]
    return bad, (f"{len(history)} wrfout, {len(shared)} wrfrst files: "
                 f"{exact.summary(stats, [(noise, 'WRF leaves them uninitialised')])}")


def restart_check(binary, out, tag, cfg, run_dir):
    """restart_<tag>_fp64[_cuda] (cases/restarts.py, RESTART_WRF): the configuration of the suite case named there,
    (1) straight vs restarted after step 1 from CPPWRF's restart_000001.nc (the step-3 snapshots bit for bit), (2)
    the run resumed from CPPWRF's restart file at the comparison's restart time in WRF history output against WRF's
    uninterrupted run at tolerance 0 (every reference frame after the restart up to the run's end), (3) where WRF's
    resumed run exists: CPPWRF resumed from WRF's own wrfrst file at minute 1 against it (wrfout frames and the
    wrfrst files WRF wrote). (good or None when an input is missing, summary)."""
    import configparser
    import netCDF4
    case_name, rst_ref, rsm_ref, mid = RESTART_WRF[tag]
    case = next(c for c in CASES if c[0] == case_name)
    if case[2] != cfg:
        return False, f"{case_name} has configuration {case[2]}, not {cfg}"
    ini = configparser.ConfigParser(interpolation=None)
    ini.read(os.path.join(REPO, cfg))
    t, dt = float(ini.get("run", "run_time")), float(ini.get("time", "dt"))
    sets = [x for x in case[3] if not x.startswith("suite.")] + [f"run.run_time={t:g}"]
    ref = f"{WR}/{rst_ref}/wrfout_d01_{os.path.basename(case[4]).split('_', 2)[2]}" if rst_ref else case[4]
    rsm = f"{WR}/{rsm_ref}" if rsm_ref else None
    need = [ref] + ([rsm, f"{WR}/{rst_ref}"] if rsm else [])
    if not os.path.exists(f"{WR}/{run_dir}/steps/wrfinput_d01") and not os.path.exists(f"{WR}/{run_dir}/wrfinput_d01"):
        return None, "input missing"
    if [x for x in need if not os.path.exists(x)]:
        return None, f"WRF reference missing: {', '.join(x for x in need if not os.path.exists(x))}"
    nl = os.path.join(os.path.dirname(ref), "namelist.input")
    dif = config_vs_namelist(cfg, sets, nl) if os.path.exists(nl) else [f"no {nl}"]
    if dif:
        return False, f"configuration differs from {nl}: {'; '.join(dif)}"
    # (1) CPPWRF straight vs CPPWRF restarted after step 1 (the native snapshots)
    a, b = os.path.join(out, "straight"), os.path.join(out, "restarted")
    rc, log = run(binary, cfg, sets + [f"restart.interval={dt:g}"], a)
    if rc != 0:
        return False, f"straight run failed: {log[-300:]}"
    rc, log = run(binary, cfg, sets + [f"run.restart_file={a}/restart_000001.nc"], b)
    if rc != 0:
        return False, f"restarted run failed: {log[-300:]}"
    with open(f"{a}/snap_000003.bin", "rb") as straight, open(f"{b}/snap_000003.bin", "rb") as resumed:
        bad = [] if straight.read() == resumed.read() else ["self: step 3 differs after the restart"]
    wrf = sets + ["output.format=wrfout", f"output.wrf_namelist={nl}"]
    # (2) resumed from CPPWRF's restart file at mid against WRF's uninterrupted run
    first = a
    if mid != dt:  # a run of its own whose restart alarm rings at mid only
        first = os.path.join(out, "first")
        rc, log = run(binary, cfg, sets + [f"restart.interval={mid:g}"], first)
        if rc != 0:
            return False, f"run to the restart at {mid:g}s failed: {log[-300:]}"
    c = os.path.join(out, "resumed_wrfout")
    rc, log = run(binary, cfg, wrf + [f"run.restart_file={first}/restart_{round(mid / dt):06d}.nc"], c)
    if rc != 0:
        return False, f"resumed run (WRF history output) failed: {log[-300:]}"
    with _NC, netCDF4.Dataset(ref) as ds:
        start = ds.getncattr("SIMULATION_START_DATE")
    # (a run resumed from CPPWRF's own file continues the run that wrote it: START_DATE is that run's, WRF's
    # uninterrupted run's; every global attribute is compared)
    b2, w2 = exact_case({"": ref}, c, t, wrf, after=times_at(start, mid), cfg=cfg)
    bad += [f"vs WRF {os.path.basename(os.path.dirname(ref))}: {x}" for x in b2]
    what = (f"step 3 bit-identical after the restart at {dt:g}s; resumed at {mid:g}s vs WRF "
            f"{os.path.relpath(ref, WR)} at tolerance 0: {w2}")
    # (3) resumed from WRF's own wrfrst file at minute 1 against WRF's resumed run
    if rsm:
        r = os.path.join(out, "resumed_from_wrf")
        rnl, why = resumed_namelist(rsm, f"{WR}/{rst_ref}", out)
        if rnl is None:
            return False, why
        # WRF's resumed run as configured: its namelist's restart_interval (1000 min); the restart alarm rings at
        # minute 2 by the WRF_ALARM_SECS_TIL_NEXT_RING_51 of rst_<C>'s file (input_wrf), and the wrfrst file
        # written there carries that interval
        try:
            interval = namelist_restart_interval(rnl)
        except ValueError as e:
            return False, str(e)
        rsets = sets + ["output.format=wrfout", f"output.wrf_namelist={rnl}",
                        f"restart.interval={interval:g}",
                        f"run.restart_file={WR}/{rst_ref}/wrfrst_d01_{times_at(start, 60)}"]
        rc, log = run(binary, cfg, rsets, r)
        if rc != 0:
            return False, f"run resumed from WRF's wrfrst file failed: {log[-300:]}"
        b3, w3 = compare_seeded_restart(r, rsm, start, t)
        bad += [f"vs WRF {rsm_ref}: {x}" for x in b3]
        what += (f"; resumed from WRF's {rst_ref}/wrfrst_d01 at 60s vs WRF's resumed run {rsm_ref} at tolerance 0 "
                 f"({w3})")
    else:
        what += "; WRF's resumed run of this configuration: none (rsm_<C>_r8 not made)"
    return not bad, what if not bad else f"{'; '.join(x[:300] for x in bad[:6])}{' ...' if len(bad) > 6 else ''}"


# what a check compares with, the tag its PASS / FAIL line carries before the wall time (tools/gcp/suite_status.sh
# counts the PASS lines by it: the SPLIT line)
KINDS = {"wrf0": "[vs WRF tol 0]", "wrfl2": "[vs WRF rel L2]", "self": "[self only]"}


def kind(name):
    """What the check `name` compares with: 'wrf0' (WRF's output at tolerance 0: the tolerance-0 cases, the
    restart checks (cases/restarts.py: their resumed runs too), the restart chains with suite.chain_wrf), 'wrfl2'
    (WRF's by relative L2: the cuda cases of the fast FP32 build) or 'self' (CPPWRF against CPPWRF only: the
    restart chains without a WRF run of their configuration, restart_p1_fp32_cuda)."""
    case = next((c for c in CASES if c[0] == name), None)
    if case is None:
        return "self" if name == "restart_p1_fp32_cuda" else "wrf0"
    sets = case[3]
    if chain_at(sets):
        return "wrf0" if chain_at(sets)[3] else "self"
    return "wrf0" if all(x == 0.0 for x in case[6].values()) else "wrfl2"


def split_line(split):
    """The SPLIT line: the PASSes by what they compared with (KINDS)."""
    return (f"SPLIT PASS vs WRF at tolerance 0: {split['wrf0']}; vs WRF by relative L2: {split['wrfl2']}; "
            f"CPPWRF self-consistency only: {split['self']}")


STATUS_MARK = "<!-- generated: tests/verify/wrf_suite.py --status-table docs/STATUS.md (rows: STATUS in tests/verify/cases) -->"


def status_table(path=None):
    """The markdown table of STATUS (the groups' rows); with `path`, it replaces the table after STATUS_MARK there."""
    table = ["| Case | Configuration | Duration |", "|---|---|---|"] + [f"| {c} | {d} | {t} |" for c, d, t in STATUS]
    if not path:
        print("\n".join(table))
        return
    lines = open(path).read().split("\n")
    i = lines.index(STATUS_MARK) + 1
    j = i
    while j < len(lines) and lines[j].startswith("|"):
        j += 1
    open(path, "w").write("\n".join(lines[:i] + table + lines[j:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cpu-dp")
    ap.add_argument("--cpu-sp", help="the CPU FP32 build (-DCPPWRF_DOUBLE=OFF): the cpu_sp cases")
    ap.add_argument("--cuda-sp-id", help="the default CUDA FP32 identity build (CPPWRF_CUDA_FP32_IDENTITY=ON, "
                    "default): the cuda_sp_id cases (the cpu_sp cases on the GPU)")
    ap.add_argument("--cuda", help="the fast CUDA FP32 build (-DCPPWRF_CUDA_FP32_IDENTITY=OFF): the cuda cases")
    ap.add_argument("--cuda-dp")
    ap.add_argument("--out", default=os.path.expanduser("~/cppwrf_scratch/wrf_suite"))
    ap.add_argument("--only", help="comma-separated substrings of case names")
    ap.add_argument("--exact", action="store_true", help="--only names are exact case names")
    ap.add_argument("--jobs", type=int, default=1, help="CPU cases run at the same time (GPU cases: one at a time)")
    ap.add_argument("--keep", action="store_true", help="keep the outputs of passing cases (default: deleted)")
    ap.add_argument("--allow-skip", action="store_true", help="cases without their reference do not fail the suite")
    ap.add_argument("--slow", action="store_true", help="also the slow cases (suite.slow: the 24 h and 72 h runs)")
    ap.add_argument("--list", action="store_true", help="print every case (all its fields) and the restart and special "
                    "checks, and exit")
    ap.add_argument("--status-table", nargs="?", const="", metavar="FILE",
                    help="print docs/STATUS.md's case table (FILE: replace it there) and exit")
    ap.add_argument("--data-for", metavar="NAME[,NAME]", help="print the data tars these cases (exact names) read and exit")
    a = ap.parse_args()
    if a.data_for is not None:
        names = [n for n in a.data_for.split(",") if n]
        unknown = [n for n in names if n not in [c[0] for c in CASES] + SPECIAL]
        none = [n for n in names if n not in unknown and not DATA.get(n)]  # '': its data are in no tar
        if unknown or none:  # a misspelt case or one without its tars: no partial data list
            print(f"ERROR {'no case named ' + ', '.join(unknown) if unknown else ''}"
                  f"{'; ' if unknown and none else ''}{'no data tar recorded for ' + ', '.join(none) + ' (pass --data)' if none else ''}",
                  file=sys.stderr)
            sys.exit(2)
        want = {t for n in names for t in DATA[n].split(",") if t}
        vm = [x.strip() for x in open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "vm_data.txt"))
              if x.strip() and not x.startswith("#")]
        print(",".join([t for t in vm if t in want] + sorted(want - set(vm))))
        return
    if a.status_table is not None:
        status_table(a.status_table)
        return
    if a.list:
        for c in CASES:
            print(" | ".join(repr(x) for x in c))
        for n in SPECIAL:
            print(n)
        for r in RESTARTS:
            print(" | ".join(repr(x) for x in r))
        return
    only = a.only.split(",") if a.only else None

    def wanted(name):
        return not only or (name in only if a.exact else any(o in name for o in only))

    unknown = [o for o in only or [] if not any(o == n if a.exact else o in n for n in [c[0] for c in CASES] + SPECIAL)]
    if unknown:  # a misspelt case would otherwise make an empty suite that passes
        print(f"ERROR no case named {', '.join(unknown)}", flush=True)
        sys.exit(2)

    def done(path, good):  # the outputs of passing cases are deleted (they take tens of GB)
        if good and not a.keep:
            shutil.rmtree(path, ignore_errors=True)

    def case(name, b, cfg, sets, refs, t, tol):
        out = os.path.join(a.out, name)
        if cfg.startswith("dropin:"):  # cppwrf-wrf in a WRF run directory, its files against WRF's at tolerance 0
            rc, log = dropin_run(b, cfg, sets, out)
            if rc != 0:
                return False, f"FAIL  {name:22s} drop-in run failed: {log[-200:]}"
            bad, what = exact_case(refs, out, t, sets, cfg=cfg)
            if wrfrst_check(sets):
                b2, w2 = wrfrst_case(refs, out, [os.path.basename(f) for f in dropin_spec(cfg, sets)[2]])
                bad, what = bad + b2, f"{what}, {w2}"
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact (cppwrf-wrf): {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:6])}{' ...' if len(bad) > 6 else ''}]" if bad else ""))
        mid = restart_at(sets)
        rst_check = wrfrst_check(sets)
        native = "suite.native_restart" in sets
        chain = chain_at(sets)
        sets = [x for x in sets if not x.startswith("suite.")]
        if chain:  # CPPWRF resumed against CPPWRF straight (restart_chain)
            bad, what = restart_chain(b, cfg, sets, refs, t, chain[0], chain[1], out, chain[2], chain[3])
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact: {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:8])}{' ...' if len(bad) > 8 else ''}]" if bad else ""))
        exact = all(x == 0.0 for x in tol.values())
        if exact:  # WRF history output; with the reference's namelist.input CPPWRF writes WRF's global attributes
            nl = os.path.join(os.path.dirname(next(iter(refs.values()))), "namelist.input")
            if os.path.exists(nl):  # the configuration is the WRF run's that made the reference
                dif = config_vs_namelist(cfg, sets, nl)
                if dif:
                    return False, f"FAIL  {name:22s} configuration differs from {nl}: {'; '.join(dif)}"
            sets = sets + ["output.format=wrfout"] + ([f"output.wrf_namelist={nl}"] if os.path.exists(nl) else [])
        if exact and mid:
            bad, what = restarted_case(b, cfg, sets, refs, t, mid, out, native)
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact: {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:6])}{' ...' if len(bad) > 6 else ''}]" if bad else ""))
        rc, log = run(b, cfg, sets, out)
        if rc != 0:
            return False, f"FAIL  {name:22s} run failed: {log[-200:]}"
        if exact:
            bad, what = exact_case(refs, out, t, sets, cfg=cfg)
            if rst_check:
                b2, w2 = wrfrst_case(refs, out)
                bad, what = bad + b2, f"{what}, {w2}"
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact: {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:6])}{' ...' if len(bad) > 6 else ''}]" if bad else ""))
        res = {}
        for dom, w in refs.items():  # nested: out/<domain>, tolerance keys "<domain>.<var>"
            for v, x in compare(w, os.path.join(out, dom) if dom else out, t).items():
                res[f"{dom}.{v}" if dom else v] = x
        bad = [f"{v} {res.get(v, float('nan'))!r}>{tl!r}" for v, tl in tol.items()
               if not (res.get(v, float("inf")) <= tl)]
        txt = " ".join(f"{v}={res.get(v, float('nan')):.1e}" for v in tol)
        done(out, not bad)
        return not bad, f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s {txt}" + (f"  [{'; '.join(bad)}]" if bad else "")

    def special(name, fn, path):
        good, msg = fn()
        if good is None:
            return None, f"SKIP  {name:22s} ({msg})"
        done(path, good)
        return good, f"{'PASS' if good else 'FAIL'}  {name:22s} {msg}"

    bins = {"cpu_dp": a.cpu_dp, "cpu_sp": a.cpu_sp, "cuda": a.cuda, "cuda_dp": a.cuda_dp, "cuda_sp_id": a.cuda_sp_id}
    for k, b in bins.items():  # a binary that was asked for and is not there (a failed build) is no SKIP
        if b and not os.path.exists(b):
            print(f"ERROR --{k.replace('_', '-')} {b}: no such file (build failed?)", flush=True)
            sys.exit(2)
    cpu, gpu = [], []
    count = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    split = {k: 0 for k in KINDS}
    for name, key, cfg, sets, wo, t, tol in CASES:
        if not wanted(name) or not bins.get(key):  # cases of binaries not given are not selected
            continue
        if slow(sets) and not a.slow and not (a.exact and only and name in only):  # slow cases only on request
            continue
        b = bins[key]
        refs = wo if isinstance(wo, dict) else {"": wo}  # nested runs: {domain: wrfout}
        missing = [w for w in refs.values() if not os.path.exists(w)]
        if missing:
            print(f"SKIP  {name:22s} (reference missing: {', '.join(missing)})", flush=True)
            count["SKIP"] += 1
            continue
        (cpu if key in ("cpu_dp", "cpu_sp") else gpu).append((t, name, lambda n=name, b=b, c=cfg, s=sets, r=refs, t=t, tl=tol: case(n, b, c, s, r, t, tl)))
    if a.cpu_dp and os.path.exists(a.cpu_dp):
        for tag, cfg, rd in RESTARTS:
            rname = f"restart_{tag}_fp64"
            if not wanted(rname):
                continue
            p = os.path.join(a.out, f"restart_{tag}")
            cpu.append((0, rname, lambda r=rname, g=tag, c=cfg, d=rd, p=p: special(r, lambda: restart_check(a.cpu_dp, p, g, c, d), p)))
    if a.cuda and os.path.exists(a.cuda) and wanted("restart_p1_fp32_cuda"):  # single precision, GPU
        p = os.path.join(a.out, "restart_p1_fp32_cuda")
        gpu.append((0, "restart_p1_fp32_cuda", lambda p=p: special("restart_p1_fp32_cuda", lambda: restart_p1(a.cuda, p), p)))
    if a.cuda_dp and os.path.exists(a.cuda_dp):
        for tag, cfg, rd in RESTARTS:
            rname = f"restart_{tag}_fp64_cuda"
            if not wanted(rname):
                continue
            p = os.path.join(a.out, f"restart_{tag}_cuda")
            gpu.append((0, rname, lambda r=rname, g=tag, c=cfg, d=rd, p=p: special(r, lambda: restart_check(a.cuda_dp, p, g, c, d), p)))
    if a.only is not None and not (cpu or gpu or count["SKIP"]):
        ap.error("--only selected no checks for the supplied binaries and options")
    # longest simulated time first; each task prints its line as soon as it ends; GPU cases share the one GPU
    cpu = sorted(cpu, key=lambda x: -x[0])
    from concurrent.futures import ThreadPoolExecutor, as_completed

    def timed(fn, name):  # the case's line ends with what it compared with (KINDS) and its wall time
        t0 = time.monotonic()
        good, line = fn()
        return good, f"{line}  {KINDS[kind(name)]}  ({time.monotonic() - t0:.0f} s)"
    with ThreadPoolExecutor(max(1, a.jobs)) as pc, ThreadPoolExecutor(1) as pg:
        futs = {pc.submit(timed, f, n): n for _, n, f in cpu} | {pg.submit(timed, f, n): n for _, n, f in gpu}
        for f in as_completed(futs):
            try:
                good, line = f.result()
            except Exception as e:  # noqa: BLE001 - a case the harness cannot finish fails, the others go on
                good, line = False, f"FAIL  {futs[f]:22s} harness error: {type(e).__name__}: {e}"
            print(line, flush=True)
            count["SKIP" if good is None else "PASS" if good else "FAIL"] += 1
            if good:
                split[kind(futs[f])] += 1
    print(split_line(split), flush=True)
    print(f"SUITE PASS {count['PASS']} FAIL {count['FAIL']} SKIP {count['SKIP']}", flush=True)
    sys.exit(0 if not count["FAIL"] and (a.allow_skip or not count["SKIP"]) else 1)


if __name__ == "__main__":
    main()
