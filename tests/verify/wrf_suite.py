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
Exit status 0 requires at least one result and a result for each name matched by --only. Missing reference data
produces a SKIP. --allow-skip permits reported SKIPs, including a selection with every case reported as SKIP.
Slow cases (suite.slow in their --set list: the 24 h and 72 h runs) are left out unless --slow is given or --only --exact
names them.
"""
import argparse
import filecmp
import glob
import json
import math
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


def run(binary, cfg, sets, out, extra_env=None, *, clear_env=()):
    shutil.rmtree(out, ignore_errors=True)
    cmd = [binary, os.path.join(REPO, cfg), "--set", f"output.dir={out}"]
    for s in sets:
        cmd += ["--set", s]
    env = dict(os.environ, OMP_NUM_THREADS=os.environ.get("OMP_NUM_THREADS", "3"), **(extra_env or {}))
    for name in clear_env:
        env.pop(name, None)
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
            line = re.split(r"[#;]", line, maxsplit=1)[0].strip()
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


def auxhist3_settings(sets, cfg=None, dom="d01"):
    """The opt-in and seconds between auxhist3 records from the effective inputs."""
    value, minutes = "false", None
    if cfg and os.fspath(cfg).startswith("dropin:"):
        _, directory, _ = dropin_spec(cfg, sets)
        path = os.path.join(directory, "namelist.input")
        if os.path.isfile(path):
            tc = read_namelist(path).get("time_control", {})
            if tc.get("output_diagnostics", [0])[0] == 1 and tc.get("io_form_auxhist3", [0])[0] == 2:
                value = "true"
                intervals = tc.get("auxhist3_interval", [])
                idx = int(dom[1:]) - 1
                minutes = intervals[idx] if idx < len(intervals) else 0
    elif cfg:
        conf = configuration(cfg, sets)
        value = conf.get("output", {}).get("auxhist3", value)
        minutes = conf.get("phys", {}).get("auxhist3_interval")
        opts = domain_options(conf, dom)
        minutes = opts.get("phys.auxhist3_interval", opts.get("phys_auxhist3_interval", minutes))
    for entry in sets:
        if entry.startswith("output.auxhist3="):
            value = entry.split("=", 1)[1]
        elif (not cfg or os.fspath(cfg).startswith("dropin:")) and entry.startswith("phys.auxhist3_interval="):
            minutes = entry.split("=", 1)[1]
    return str(value).strip().lower() in ("true", "1", "yes"), 60 * int(minutes) if minutes is not None else None


def auxhist3_check(sets, cfg=None):
    return auxhist3_settings(sets, cfg)[0]


def exact_case(refs, out, t, sets, after=None, ignore_atts=(), *, cfg=None, required_times=None, split=None,
               aux_schedules=None, reference_schedules=None):
    """Compare history and auxhist3 with WRF up to t, optionally requiring a specific history cadence.

    A list of two output directories gives the first and resumed legs of a restarted case. The first leg owns
    records through the split, and the resumed leg owns later records.
    """
    with _NC:
        return _exact_case(refs, out, t, sets, after, ignore_atts, cfg, required_times, split, aux_schedules,
                           reference_schedules)


def _exact_case(refs, out, t, sets, after, ignore_atts, cfg, required_times, split, aux_schedules, reference_schedules):
    import netCDF4
    import exact
    bad, stats, noise, no_nl = [], {}, set(), []
    first, *resumed = out if isinstance(out, list) else [out]
    for dom, w in refs.items():
        d = dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1)
        with netCDF4.Dataset(w) as ds:
            xt = [float(x) * 60.0 for x in ds["XTIME"][:]]
            final = [exact.text(ds["Times"], i).decode() for i, x in enumerate(xt) if abs(x - t) < 1e-6]
            origin = getattr(ds, "SIMULATION_START_DATE", None)
        if not final:
            bad.append(f"{d}: the reference has no frame at t={t:g}s")
            continue
        has_nl = os.path.exists(os.path.join(os.path.dirname(w), "namelist.input"))
        if not has_nl:
            no_nl.append(d)
        mine = stream_files(first, "wrfout", d)
        continuation = stream_files(resumed[0], "wrfout", d) if resumed else ()
        scheduled = (required_times[d] if split is not None else required_times) or ()
        reference_times = reference_schedules[d] if reference_schedules else set()
        optional_times = reference_times - set(scheduled) if split is not None else ()
        dif, _ = exact.compare_stream(mine, [w], subset=True, until=final[0], global_atts=has_nl, after=after,
                                      ignore_atts=ignore_atts, stats=stats,
                                      common_only=required_times is not None and split is None,
                                      continuation=continuation, split=split, required_times=scheduled, wrf_reference=True,
                                      required_reference_times=reference_times, optional_reference_times=optional_times,
                                      waive=uninitialised, noise=noise)
        bad += [f"{d}.{k}: {v}" for k, v in dif.items()]
        # output_diagnostics: the auxhist3 files next to the reference wrfout, compared as the wrfout files
        require_aux, interval = auxhist3_settings(sets, cfg, d)
        if require_aux and cfg and (interval is None or interval <= 0):
            bad.append(f"{d}.auxhist3: missing positive auxhist3 interval")
        if require_aux and origin is None:
            bad.append(f"{d}.auxhist3: WRF reference missing SIMULATION_START_DATE")
        aux = sorted(glob.glob(os.path.join(os.path.dirname(w), f"auxhist3_{d}_*")))
        if not aux and require_aux:
            bad.append(f"{d}.auxhist3: no WRF reference files")
        if aux:
            mine = stream_files(first, "auxhist3", d)
            continuation = stream_files(resumed[0], "auxhist3", d) if resumed else ()
            aux_end = final[0]
            if aux_schedules is not None:
                aux_end, aux_required, aux_ref = aux_schedules[d]
            else:
                aux_ref = set()
                if require_aux and interval and origin:
                    clock, lower = (origin, interval), after
                    if cfg and os.fspath(cfg).startswith("dropin:"):
                        _, _, links = dropin_spec(cfg, sets)
                        seed = next((f for f in links if os.path.basename(f).startswith(f"wrfrst_{d}_")), None)
                        if seed:
                            split_stamp = os.path.basename(seed)[-19:]
                            clock = resumed_history_clock(seed, interval, origin, split_stamp, alarm=AUX_ALARM)
                            lower = max(split_stamp, lower) if lower else split_stamp
                    aux_ref = {s for s in history_schedule(*clock, aux_end) if lower is None or s > lower}
                aux_required = aux_ref
            aux_optional = aux_ref - aux_required if split is not None else ()
            dif, n = exact.compare_stream(mine, aux, subset=True, until=aux_end, global_atts=has_nl, after=after,
                                          ignore_atts=ignore_atts, stats=stats,
                                          common_only=aux_schedules is not None and split is None,
                                          continuation=continuation, split=split, required_times=aux_required,
                                          required_reference_times=aux_ref, optional_reference_times=aux_optional,
                                          wrf_reference=True)
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


def tile_check_env(sets):
    """A tiled CPU case run with CPPWRF_TILE_CHECK=1 (suite.tile_check in its --set list: every launch of every step
    of the tiles compared with a run with one patch; docs/MPI.md): the run's extra environment, else None."""
    return {"CPPWRF_TILE_CHECK": "1"} if "suite.tile_check" in sets else None


RESTART_ALARM = "WRF_ALARM_SECS_TIL_NEXT_RING_51"


def restart_calendar(dt, interval, end, *, begin=0, pending=None):
    """Fixed-step restart alarm, including the seed's pending RingTime on WRF-format resume."""
    previous = begin
    rings = {}
    for step in range(round(begin / dt) + 1, round(end / dt) + 1):
        now = step * dt
        if pending is not None and now + 1e-6 * dt >= pending:
            previous, pending = now, None
        elif interval > 0 and now + 1e-6 * dt >= previous + interval:
            previous += interval
        else:
            continue
        elapsed = now - previous
        seconds = round(elapsed) if abs(elapsed - round(elapsed)) <= 1e-6 else math.trunc(elapsed)
        rings[now] = math.trunc(interval) - seconds if interval > 0 else 1700000000
    return rings


def restart_schedule(dt, interval, end, *, begin=0, pending=None):
    return restart_calendar(dt, interval, end, begin=begin, pending=pending).keys()


def configured_restart_times(conf, origin, *, begin=0, pending=None):
    dt = float(conf["time"]["dt"])
    rst = conf.get("restart", {})
    interval = float(rst.get("interval", conf.get("output", {}).get("restart_interval", 0)))
    end = float(conf["run"]["run_time"])
    cutoff = float(rst.get("end", 0))
    if cutoff > 0:
        end = min(end, round(cutoff / dt) * dt)
    return {times_at(origin, s) for s in restart_schedule(dt, interval, end, begin=begin, pending=pending)}


def simulation_start(path):
    import netCDF4
    with _NC, netCDF4.Dataset(path) as ds:
        return ds.getncattr("SIMULATION_START_DATE")


def xtime_matches(minutes, seconds):
    return math.isclose(float(minutes) * 60, seconds, rel_tol=1e-7, abs_tol=1e-6)


def restart_inventory(files, dom, origin, expected, *, side, seed=None):
    """Each scheduled time has one file with one matching Times/XTIME record."""
    import netCDF4
    import exact
    records, bad = {}, []
    prefix = f"wrfrst_{dom}_"
    with _NC:
        for f in files:
            stamp = os.path.basename(f)[len(prefix):]
            if stamp in records:
                bad.append(f"duplicate {side} restart {os.path.basename(f)}: {records[stamp]} and {f}")
            records[stamp] = f
            with netCDF4.Dataset(f) as ds:
                if len(ds["Times"]) != 1 or len(ds["XTIME"]) != 1:
                    bad.append(f"{f}: expected one restart record")
                    continue
                actual = exact.text(ds["Times"], 0).decode()
                elapsed = seconds_between(origin, stamp)
                if actual != stamp or not xtime_matches(ds["XTIME"][0], elapsed):
                    bad.append(f"{f}: restart clock disagrees with filename: Times {actual}, XTIME {ds['XTIME'][0]}")
    records.pop(seed, None)  # a WRF-format input may be linked in the output directory
    missing, extra = expected - records.keys(), records.keys() - expected
    if missing:
        bad.append(f"missing {side} restart {' '.join(prefix + s for s in sorted(missing))}")
    if extra:
        bad.append(f"unexpected {side} restart {' '.join(prefix + s for s in sorted(extra))}")
    return records, bad


def wrfrst_case(refs, out, cfg, sets):
    """The wrfrst files of a run against WRF's: the reference run's directory (next to its wrfout) and `out` (or
    out/<domain>) must hold the same set of wrfrst_<domain>_<date> files (the restart alarm's times), each identical at
    tolerance 0 (every variable and attribute, WRF_ALARM_SECS_TIL_NEXT_RING_* among them; RQCBLTEN / RQIBLTEN not when
    WRF's file has no QCLOUD / QICE). Check each side against the effective restart schedule:
    (differences, summary)."""
    import exact
    import netCDF4
    bad, n, stats, noise = [], 0, {}, set()
    origin = simulation_start(next(iter(refs.values())))
    seeds, begin, pending = {}, 0, None
    if os.fspath(cfg).startswith("dropin:"):
        _, nl_dir, links = dropin_spec(cfg, sets)
        nml = os.path.join(nl_dir, "namelist.input")
        nl = read_namelist(nml)
        from tools.wrf2cppwrf import WrfTranslator
        conf = WrfTranslator(nl, namelist_path=nml, output_format="wrfout").translate()
        for f in links:
            if match := re.fullmatch(r"wrfrst_(d\d\d)_(.+)", os.path.basename(f)):
                seeds[match[1]] = match[2]
                _, errors = restart_inventory([f], match[1], origin, {match[2]}, side="seed")
                bad += errors
                if match[1] == "d01" and not errors:
                    begin = seconds_between(origin, match[2])
                    with _NC, netCDF4.Dataset(f) as ds:
                        left = int(ds.getncattr(RESTART_ALARM))
                    if 0 <= left < 1700000000:
                        pending = begin + left
        conf["run"]["run_time"] += begin
        for entry in sets:
            if "=" in entry:
                key, value = entry.split("=", 1)
                section, key = key.split(".", 1)
                conf.setdefault(section, {})[key] = value
    else:
        conf = configuration(cfg, sets)
    required = configured_restart_times(conf, origin, begin=begin, pending=pending)
    if not required:
        bad.append("configured restart schedule has no files")
    for dom, w in refs.items():
        d = dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1)
        ref, errors = restart_inventory(stream_files(os.path.dirname(w), "wrfrst", d), d, origin, required,
                                        side="WRF reference", seed=seeds.get(d))
        mine, errors_c = restart_inventory(stream_files(out, "wrfrst", d), d, origin, required,
                                           side="CPPWRF", seed=seeds.get(d))
        bad += errors + errors_c
        for name in sorted(required & ref.keys() & mine.keys()):
            b2, _ = wrfrst_pair(mine[name], ref[name], stats, noise)
            bad += [f"{name}.{x}" for x in b2]
            n += 1
    return bad, f"{n} wrfrst files" + (" identical" if not bad else "") + f" ({exact.summary(stats, [(noise, 'WRF leaves them uninitialised')])})"


def wrfrst_pair(mine, ref, stats, noise):
    """One wrfrst file of CPPWRF's against WRF's at tolerance 0 (every variable, attribute and global attribute;
    the variables WRF leaves uninitialised, wrf_suite.uninitialised, not counted: added to `noise` when they
    differ): ([differences], number of frames); `stats`: exact.compare_stream's counts."""
    import exact
    with _NC:
        dif, n = exact.compare_stream([mine], [ref], stats=stats, wrf_reference=True,
                                      waive=uninitialised, noise=noise)
    return [f"{k}: {v}" for k, v in dif.items()], n


HISTORY_ALARM = "WRF_ALARM_SECS_TIL_NEXT_RING_01"
HISTORY_RINGING = "WRF_ALARM_ISRINGING_01"
AUX_ALARM = "WRF_ALARM_SECS_TIL_NEXT_RING_04"
AUX_RINGING = "WRF_ALARM_ISRINGING_04"
RUN_START_ALARM = "WRF_ALARM_SECS_TIL_NEXT_RING_55"


def start_dates(files, want, *, restart_times=None, history_clock=None, aux_clock=None, end=None):
    """Report incorrect START_DATE and, for a restart stream, its run-start alarm."""
    import netCDF4
    import exact
    out = []
    with _NC:
        for f in files:
            with netCDF4.Dataset(f) as ds:
                v = getattr(ds, "START_DATE", None)
                if v != want:
                    out.append(f"{os.path.basename(f)} START_DATE {v!r}, the expected start time {want}")
                if restart_times is not None:
                    stamp = exact.text(ds["Times"], 0).decode()
                    elapsed = seconds_between(want, stamp)
                    actual = getattr(ds, RUN_START_ALARM, None)
                    if actual != -elapsed:
                        out.append(f"{os.path.basename(f)} {RUN_START_ALARM} {actual}, expected {-elapsed:g}")
                    expected_rst = restart_times.get(stamp)
                    if expected_rst is not None and getattr(ds, RESTART_ALARM, None) != expected_rst:
                        actual = getattr(ds, RESTART_ALARM, None)
                        out.append(f"{os.path.basename(f)} {RESTART_ALARM} {actual}, expected {expected_rst}")
                    for clock, secs_key, ring_key in ((history_clock, HISTORY_ALARM, HISTORY_RINGING),
                                                       (aux_clock, AUX_ALARM, AUX_RINGING)):
                        if clock is None:
                            continue
                        origin, interval = clock
                        since = round(seconds_between(origin, stamp))
                        due = interval > 0 and since % interval == 0
                        left = interval - since % interval if interval > 0 else 1700000000
                        ringing = int(due and (stamp == end or "wrfrst_d01_" not in os.path.basename(f)))
                        for key, expected in ((secs_key, left), (ring_key, ringing)):
                            actual = getattr(ds, key, None)
                            if actual != expected:
                                out.append(f"{os.path.basename(f)} {key} {actual}, expected {expected}")
    return out


def case_domains(refs):
    return {dom or re.search(r"wrfout_(d\d\d)_", os.path.basename(w)).group(1): w for dom, w in refs.items()}


def configured_domains(conf):
    return {f"d{i:02d}" for i in range(1, int(conf.get("domains", {}).get("max_dom", 1)) + 1)}


def chain_references(cfg, sets, wrf):
    return {d: wrf.replace("wrfout_d01_", f"wrfout_{d}_") for d in sorted(configured_domains(configuration(cfg, sets)))}


def reference_domain_errors(cfg, sets, refs):
    if os.fspath(cfg).startswith("dropin:"):
        _, directory, _ = dropin_spec(cfg, sets)
        nl = read_namelist(os.path.join(directory, "namelist.input"))
        count = int(nl.get("domains", {}).get("max_dom", [1])[0])
    else:
        count = int(configuration(cfg, sets).get("domains", {}).get("max_dom", 1))
    expected = {f"d{i:02d}" for i in range(1, count + 1)}
    scoped = next((s.split("=", 1)[1] for s in sets if s.startswith("suite.history_domains=")), None)
    if scoped is not None:
        declared = set(scoped.split(","))
        if not declared <= expected:
            return [f"suite.history_domains has unknown domains {sorted(declared - expected)}"]
        expected = declared
    actual = set(case_domains(refs))
    if actual != expected:
        return [f"history reference domains {sorted(actual)}, expected {sorted(expected)}"]
    return [f"{d} history reference filename disagrees with its domain: {w}" for d, w in case_domains(refs).items()
            if not os.path.basename(w).startswith(f"wrfout_{d}_")]


def restarted_case(b, cfg, sets, refs, t, mid, out, native=False):
    """A run to `t` s restarted at `mid` s from its own wrfrst files: the first run (to one step past mid, wrfrst files
    at mid only: written within the run, as WRF's, not at its last step, where WRF's history alarm is still ringing)
    and the resumed one (mid to t) together against WRF's uninterrupted run at tolerance 0 (every reference frame;
    START_DATE, which a run resumed from a wrfrst file sets to the restart time as WRF's resumed runs do, is checked
    to be that; a run resumed from CPPWRF's own file continues the run that wrote it: every attribute compared),
    and the wrfrst files at mid against WRF's (the reference run's restart_interval writes them there):
    (differences, summary). As run_wrf_files_test.sh, RQCBLTEN / RQIBLTEN are not compared when WRF's restart file
    has no QCLOUD / QICE (WRF leaves them uninitialised)."""
    import exact
    first, resumed = os.path.join(out, "a_first"), os.path.join(out, "b_resumed")
    w0 = next(iter(refs.values()))
    origin = simulation_start(w0)
    split, end = times_at(origin, mid), times_at(origin, t)
    conf = configuration(cfg, sets)
    dt = float(conf["time"]["dt"])
    doms = case_domains(refs)
    rc, log = run(b, cfg, sets + [f"run.run_time={mid + dt:g}", f"restart.interval={mid:g}", f"restart.end={mid:g}"],
                  first)
    if rc != 0:
        return [f"run to the restart failed: {log[-200:]}"], ""
    stats, noise, bad, seeds = {}, set(), [], {}
    if native:
        seed = os.path.join(first, f"restart_{round(mid / dt):06d}.nc")
        if not os.path.isfile(seed):
            return [f"missing native restart seed {seed}"], ""
    else:
        rst = os.path.join(out, "rst")
        os.makedirs(rst, exist_ok=True)
        for d, w in doms.items():
            name = f"wrfrst_{d}_{split}"
            # The WRF directory can hold other scheduled restarts as well.
            ref = [f for f in stream_files(os.path.dirname(w), "wrfrst", d) if os.path.basename(f) == name]
            _, errors = restart_inventory(ref, d, origin, {split}, side="WRF reference")
            mine, errors_c = restart_inventory(stream_files(first, "wrfrst", d), d, origin, {split}, side="first leg")
            bad += errors + errors_c
            if errors or errors_c:
                continue
            b2, _ = wrfrst_pair(mine[split], ref[0], stats, noise)
            bad += [f"{d}.wrfrst.{x}" for x in b2]
            seeds[d] = mine[split]
            os.symlink(seeds[d], os.path.join(rst, name))
        if bad:
            return bad, ""
        seed = os.path.join(rst, f"wrfrst_d01_{split}")
    rc, log = run(b, cfg, sets + [f"run.restart_file={seed}"], resumed)
    if rc != 0:
        return [f"resumed run failed: {log[-200:]}"], ""
    cadence = chain_history(conf, sorted(doms), t)
    first_cadence = chain_history(conf, sorted(doms), mid + dt)
    shared, reference_schedules, aux_schedules = {}, {}, {}
    for d in doms:
        history_first = history_schedule(origin, first_cadence[d], times_at(origin, mid + dt), initial=True)
        clock = (origin, cadence[d]) if native else resumed_history_clock(seeds[d], cadence[d], origin, split)
        history_resumed = {s for s in history_schedule(*clock, end) if s > split}
        if not history_resumed:
            bad.append(f"{d}: resumed history schedule has no post-split frame")
        for path, due, label in ((first, history_first, "first leg"), (resumed, history_resumed, "resumed leg")):
            bad += [f"{d} {label}: {error}" for error in history_inventory(stream_files(path, "wrfout", d), due, origin)]
        if not native:
            bad += start_dates(stream_files(resumed, "wrfout", d), split)
        straight = history_schedule(origin, cadence[d], end, initial=True)
        shared[d] = straight & ({s for s in history_first if s <= split} | history_resumed)
        reference_schedules[d] = straight
        aux_enabled, aux_iv = auxhist3_settings(sets, cfg, d)
        aux_ref, aux_shared = set(), set()
        if aux_enabled and aux_iv:
            aux_ref = history_schedule(origin, aux_iv, end)
            aux_clock = ((origin, aux_iv) if native else
                         resumed_history_clock(seeds[d], aux_iv, origin, split, alarm=AUX_ALARM))
            aux_resumed = {s for s in history_schedule(*aux_clock, end) if s > split}
            aux_shared = aux_ref & ({s for s in aux_ref if s <= split} | aux_resumed)
            for path, due in ((first, history_schedule(origin, aux_iv, times_at(origin, mid + dt), initial=True)),
                              (resumed, aux_resumed)):
                bad += [f"{d} {path} auxhist3: {x}" for x in
                        history_inventory(stream_files(path, "auxhist3", d), due, origin, require_xtime=False)]
        aux_schedules[d] = end, aux_shared, aux_ref
    b2, what = exact_case(refs, [first, resumed], t, sets, cfg=cfg, split=split, required_times=shared,
                          ignore_atts=() if native else ("START_DATE",), aux_schedules=aux_schedules,
                          reference_schedules=reference_schedules)
    detail = (f"wrfrst at {mid:g}s vs WRF's: {exact.summary(stats, [(noise, 'WRF leaves them uninitialised')])}"
              if seeds else f"native seed {os.path.basename(seed)}")
    return bad + b2, f"{what}, restarted at {mid:g}s; {detail}"


def times_at(start, secs):
    """The Times string `secs` seconds after `start` (a Times string)."""
    import datetime
    t0 = datetime.datetime.strptime(start, "%Y-%m-%d_%H:%M:%S")
    return (t0 + datetime.timedelta(seconds=secs)).isoformat(sep="_", timespec="seconds")


def seconds_between(origin, stamp):
    from datetime import datetime
    return (datetime.fromisoformat(stamp) - datetime.fromisoformat(origin)).total_seconds()


def chain_at(sets):
    """A restart-chain case (suite.chain=MID,INTERVAL[,native] in its --set list: restart_chain), (MID, INTERVAL
    [s], native only, the WRF reference of the same configuration (suite.chain_wrf=WRFOUT) or None), or None."""
    v = next((x.split("=", 1)[1] for x in sets if x.startswith("suite.chain=")), None)
    if not v:
        return None
    f = v.split(",")
    return float(f[0]), float(f[1]), f[2:] == ["native"], \
        next((x.split("=", 1)[1] for x in sets if x.startswith("suite.chain_wrf=")), None)


def domain_options(conf, dom):
    opts = {k.removeprefix(dom + "_"): v for k, v in conf.get("domains", {}).items() if k.startswith(dom + "_")}
    opts.update(conf.get(dom, {}))
    return opts


def chain_history(conf, doms, t):
    """History cadence in seconds on each configured chain domain."""
    dt = {"d01": float(conf["time"]["dt"])}
    cadence = {"d01": dt["d01"] * int(conf.get("output", {}).get("out_interval", round(t / dt["d01"])))}
    for d in doms[1:]:
        opts = domain_options(conf, d)
        parent = f"d{int(opts.get('parent_id', int(d[1:]) - 1)):02d}"
        ratio = opts.get("parent_time_step_ratio", conf.get("domains", {}).get("parent_time_step_ratio", 1) if d == "d02" else 1)
        dt[d] = float(opts.get("dt", dt[parent] / int(ratio)))
        cadence[d] = dt[d] * int(opts["out_interval"]) if "out_interval" in opts else cadence[parent]
    return cadence


def history_schedule(start, interval, end, *, initial=False):
    """A fixed-step history alarm with its phase at start."""
    if interval <= 0:
        return {start} if initial else set()
    count = int((seconds_between(start, end) + 1e-6) / interval)
    return {times_at(start, i * interval) for i in range(0 if initial else 1, count + 1)}


def history_inventory(files, expected, origin, *, require_xtime=True):
    """Check one leg's Times against its schedule. The default auxhist3 field list omits XTIME."""
    import netCDF4
    import exact
    seen, errors = set(), []
    with _NC:
        for f in files:
            with netCDF4.Dataset(f) as ds:
                times = ds["Times"]
                xtime = ds.variables.get("XTIME")
                if not len(times):
                    errors.append(f"{f}: zero records")
                if xtime is not None and xtime.dimensions == ("Time",) and xtime.shape == (len(times),):
                    minutes = xtime[:]
                else:
                    minutes = None
                    if require_xtime or xtime is not None:
                        errors.append(f"{f}: expected one XTIME per history record")
                for i in range(len(times)):
                    stamp = exact.text(times, i).decode()
                    if stamp in seen:
                        errors.append(f"duplicate Times in CPPWRF: {stamp} in {f}")
                    seen.add(stamp)
                    if minutes is not None and not xtime_matches(minutes[i], seconds_between(origin, stamp)):
                        errors.append(f"{f}: XTIME {minutes[i]} disagrees with history Times {stamp}")
    if expected - seen:
        errors.append(f"CPPWRF missing {sorted(expected - seen)}")
    if seen - expected:
        errors.append(f"only-cppwrf {sorted(seen - expected)} outside its scheduled frames")
    return errors


def resumed_history_clock(seed, interval, original_start, split, *, alarm=HISTORY_ALARM):
    """WRF-format resume takes the interval and phase from the seed's history alarm."""
    import netCDF4
    with _NC, netCDF4.Dataset(seed) as ds:
        left = int(ds.getncattr(alarm))
    if left >= 1700000000:
        return split, 0
    if left < 0:
        return original_start, interval
    if left == 0:
        raise ValueError(f"{seed}: {alarm} must be positive")
    return split, left


def chain_self_stream(straight, resumed, mid, *, restart, wrfrst_resume, noise, aux=False):
    """Compare shared history states and all post-split restart states."""
    import exact
    ignore = ("START_DATE", RUN_START_ALARM, HISTORY_ALARM, HISTORY_RINGING, RESTART_ALARM) if wrfrst_resume else ()
    if wrfrst_resume and aux:
        ignore += (AUX_ALARM, AUX_RINGING)
    with _NC:
        # NOAHMP_INIT reinitialises this output on restart (PhysicsSuite::restarted).
        return exact.compare_stream(resumed, straight, after=mid, global_atts=restart,
                                    ignore_atts=ignore, common_only=True, waive=lambda _: ("CROPCAT",), noise=noise)


def restart_chain(b, cfg, sets, refs, t, mid, iv, out, native_only=False, wrf=None):
    """The restart contract without a WRF restarted run: CPPWRF's run to `t` s with restart files every `iv` s
    (wrfrst and, for one domain, CPPWRF's own restart_<step>.nc) and the runs resumed at `mid` s from them (each
    domain's wrfrst file; the native file) must write their scheduled records. Compare shared history frames and
    post-split wrfrst files at tolerance 0 (restart-stream state a resumed run lost shows in
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
    doms = sorted(configured_domains(conf))
    if set(case_domains(refs)) != set(doms):
        return [f"chain input reference domains {sorted(case_domains(refs))}, expected {doms}"], ""
    if native_only and len(doms) > 1:
        return ["native-only chain requires a single domain and a native resumed leg"], ""
    emitted = {os.path.basename(f).split("_")[1]
               for f in glob.glob(os.path.join(straight, "**", "wrfrst_d??_*"), recursive=True)}
    unexpected = emitted - set(doms)
    if unexpected:
        return [f"straight run wrote wrfrst files for unexpected domains {', '.join(sorted(unexpected))}"], ""
    origin = simulation_start(next(iter(refs.values())))
    start, end = times_at(origin, mid), times_at(origin, t)
    dt = float(conf["time"]["dt"])
    restart_times = {times_at(origin, sec): left for sec, left in restart_calendar(dt, iv, t).items()}
    if start not in restart_times:
        return [f"split at {mid:g}s is outside the restart schedule"], ""
    cadence = chain_history(conf, doms, t)
    straight_schedule = {d: history_schedule(origin, cadence[d], end, initial=True) for d in doms}
    aux_intervals = {d: iv3 for d in doms for enabled, iv3 in [auxhist3_settings(sets, cfg, d)] if enabled and iv3}
    straight_aux = {d: history_schedule(origin, iv3, end, initial=True) for d, iv3 in aux_intervals.items()}
    bad, seed_errors, at, nfr, nrst, cropcat = [], [], {}, 0, 0, set()
    for d in doms:
        files = stream_files(straight, "wrfrst", d)
        records, errors = restart_inventory(files, d, origin, set(restart_times), side="straight")
        seed_errors += errors
        if not errors:
            at[d] = records[start]
            seed_errors += start_dates(files, origin, restart_times=restart_times, history_clock=(origin, cadence[d]),
                                       aux_clock=(origin, aux_intervals[d]) if d in aux_intervals else None, end=end)
        bad += [f"straight: {d}.wrfout: {x}" for x in
                history_inventory(stream_files(straight, "wrfout", d), straight_schedule[d], origin)]
        if d in straight_aux:
            bad += [f"straight: {d}.auxhist3: {x}" for x in
                    history_inventory(stream_files(straight, "auxhist3", d), straight_aux[d], origin, require_xtime=False)]
    if seed_errors:
        return seed_errors + bad, ""
    rst = os.path.join(out, "rst")
    os.makedirs(rst, exist_ok=True)
    for d in doms:
        os.symlink(at[d], os.path.join(rst, os.path.basename(at[d])))
    resumed = [] if native_only else [("wrfrst", os.path.join(out, "b_resumed"),
                                       os.path.join(rst, os.path.basename(at["d01"])))]
    if doms == ["d01"]:  # CPPWRF's own restart file of the same step (nests resume from wrfrst files only)
        resumed.append(("native", os.path.join(out, "c_native"),
                        os.path.join(straight, f"restart_{round(mid / dt):06d}.nc")))
    schedules, resumed_aux = {}, {}
    for what, d_out, f in resumed:
        schedules[what] = {}
        resumed_aux[what] = {}
        if not os.path.isfile(f):
            bad.append(f"{what}: missing restart seed {f}")
            continue
        rc, log = run(b, cfg, common + [f"run.restart_file={f}"], d_out)
        if rc != 0:
            bad.append(f"{what}: resumed run failed: {log[-200:]}")
            continue
        wrfrst_resume = what == "wrfrst"
        resumed_restarts = {s: left for s, left in restart_times.items() if s > start}
        if wrfrst_resume:
            pending = mid + restart_times[start]
            resumed_restarts = {times_at(origin, sec): left for sec, left in
                                restart_calendar(dt, iv, t, begin=mid, pending=pending).items()}
        for dom in doms:
            clock = resumed_history_clock(at[dom], cadence[dom], origin, start) if wrfrst_resume else (origin, cadence[dom])
            scheduled = {stamp for stamp in history_schedule(*clock, end) if stamp > start}
            schedules[what][dom] = scheduled
            bad += [f"{what}: {dom}.wrfout: {x}" for x in
                    history_inventory(stream_files(d_out, "wrfout", dom), scheduled, origin)]
            if wrfrst_resume:
                bad += start_dates(stream_files(d_out, "wrfout", dom), start)
            restart_files = stream_files(d_out, "wrfrst", dom)
            _, errors = restart_inventory(restart_files, dom, origin, set(resumed_restarts), side=what)
            bad += errors
            aux_clock = None
            streams = ["wrfout", "wrfrst"]
            if dom in aux_intervals:
                iv3 = aux_intervals[dom]
                aux_clock = resumed_history_clock(at[dom], iv3, origin, start, alarm=AUX_ALARM) if wrfrst_resume else (origin, iv3)
                due = {s for s in history_schedule(*aux_clock, end) if s > start}
                resumed_aux[what][dom] = due
                bad += [f"{what}: {dom}.auxhist3: {x}" for x in
                        history_inventory(stream_files(d_out, "auxhist3", dom), due, origin, require_xtime=False)]
                streams.append("auxhist3")
            bad += start_dates(restart_files, start if wrfrst_resume else origin, restart_times=resumed_restarts,
                               history_clock=clock, aux_clock=aux_clock, end=end)
            for kind in streams:
                dif, n = chain_self_stream(stream_files(straight, kind, dom), stream_files(d_out, kind, dom),
                                           start, restart=kind == "wrfrst", wrfrst_resume=wrfrst_resume,
                                           noise=cropcat, aux=aux_clock is not None)
                nfr += n if kind == "wrfout" else 0
                nrst += n if kind == "wrfrst" else 0
                bad += [f"{what}: {dom}.{kind}.{k}: {v}" for k, v in dif.items()]
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
    wdom = chain_references(cfg, sets, wrf)
    missing = [w for w in wdom.values() if not os.path.exists(w)]
    if missing:
        return bad + [f"no WRF reference {', '.join(missing)}"], what
    tw, scheduled, split = {}, {}, {}
    for d, w in wdom.items():
        with _NC, netCDF4.Dataset(w) as ds:
            xt = [float(x) * 60.0 for x in ds["XTIME"][:]]
            reference_start = ds.getncattr("SIMULATION_START_DATE")
        within = [x for x in xt if x <= t + 1e-6]
        if not within:
            return bad + [f"{d}: no WRF history through {t:g}s"], what
        tw[d] = max(within)
        scheduled[d] = history_schedule(reference_start, cadence[d], times_at(reference_start, t), initial=True)
        split[d] = times_at(reference_start, mid)
    tag = (lambda d: f"{d} ") if len(wdom) > 1 else (lambda d: "")
    w2 = []
    for d, w in wdom.items():
        aux_ref = straight_aux.get(d, set()) - {origin}
        b2, s2 = exact_case({d: w}, straight, tw[d], common, cfg=cfg, required_times=scheduled[d],
                            aux_schedules={d: (end, aux_ref, aux_ref)})
        bad += [f"straight vs WRF: {x}" for x in b2]
        w2.append(tag(d) + s2)
    what += f"; vs WRF {os.path.relpath(wrf, WR)}{' (every domain)' if len(wdom) > 1 else ''} at tolerance 0: " \
        f"straight {', '.join(w2)}"
    for w, d_out, _ in resumed:  # a wrfrst resume's START_DATE is the restart time (WRF's), a native one's the run's
        if set(schedules[w]) != set(doms):
            continue
        w2 = []
        for d, wf in wdom.items():
            aux_ref = {s for s in straight_aux.get(d, ()) if s > start}
            aux_shared = aux_ref & resumed_aux[w].get(d, set())
            b2, s2 = exact_case({d: wf}, d_out, tw[d], common, after=split[d], cfg=cfg,
                                ignore_atts=("START_DATE",) if w == "wrfrst" else (),
                                required_times=scheduled[d] & schedules[w][d],
                                aux_schedules={d: (end, aux_shared, aux_ref)})
            bad += [f"{w} vs WRF: {x}" for x in b2]
            w2.append(tag(d) + s2)
        what += f"; {w} resumed {', '.join(w2)}"
    return bad, what


def layout_restart_at(sets):
    """A decomposition restart case (suite.layout_restart=LAYOUT,INTERVAL in its --set list: layout_restart),
    (LAYOUT, INTERVAL [s]), or None."""
    v = next((x.split("=", 1)[1] for x in sets if x.startswith("suite.layout_restart=")), None)
    if not v:
        return None
    lay, iv = v.split(",")
    return lay.strip(), float(iv)


_LAYOUT_ENV = ("CPPWRF_TILES", "CPPWRF_TILE_DEVICES", "CPPWRF_TILE_CHECK", "CPPWRF_TILE_CHECK_LOG",
               "CPPWRF_TILE_TRACE", "CPPWRF_RANKS", "CPPWRF_RANK_DEVICES")


def layout_restart_worker_error(log, layout):
    tiles = re.findall(r"^cppwrf: (\d+) x (\d+) tiles(.*), \d+ OpenMP threads each$", log, re.MULTILINE)
    ranks = re.findall(r"^cppwrf: \d+ MPI ranks as (\d+) x (\d+) patches$", log, re.MULTILINE)
    requested = tuple(map(str, layout))
    # main.cpp takes the serial branch without a tile banner. DomainRunner still reports its setup.
    if not (tiles or ranks) and re.search(r"^cppwrf: case=\S+ grid=", log, re.MULTILINE):
        actual = ("1", "1")
    elif len(tiles) == 1 and not ranks and not tiles[0][2]:
        actual = tiles[0][:2]
    else:
        actual = None
    if actual != requested:
        report = "x".join(actual) if actual else "no matching worker report"
        return f"expected effective layout {'x'.join(requested)}, log shows {report}"
    return None


def layout_restart_schedule(conf, refs):
    """Expected restart filenames keyed by domain, using the fixed-step alarm and the native final checkpoint."""
    from fractions import Fraction

    ref_by_domain = case_domains(refs)
    domains = sorted(configured_domains(conf))
    if not domains:
        raise ValueError("layout_restart requires at least one domain")
    dts, expected = {}, set()
    for dom in domains:
        opts = domain_options(conf, dom) if len(domains) > 1 else {}
        base = configuration(os.path.expandvars(opts["config"]), []) if opts.get("config") else conf
        run_opts, time_opts, rst, output = (base.get(s, {}) for s in ("run", "time", "restart", "output"))
        output_interval = opts.get("output.restart_interval", opts.get("restart_interval",
                                   output.get("restart_interval", "0")))
        interval = float(opts.get("restart.interval", rst.get("interval", output_interval)))
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError(f"{dom}: layout_restart requires a positive restart.interval")
        if opts.get("run.restart_file", run_opts.get("restart_file")) or \
                opts.get("time.adaptive", time_opts.get("adaptive", "false")).lower() in ("1", "true", "yes"):
            raise ValueError(f"{dom}: layout_restart requires a fixed-step cold start")
        if dom == "d01":
            dt = float(opts.get("time.dt", opts.get("dt", time_opts.get("dt", "0"))))
        else:
            parent = f"d{int(opts.get('parent_id', int(dom[1:]) - 1)):02d}"
            default_ratio = conf["domains"].get("parent_time_step_ratio", 1) if dom == "d02" else 1
            ratio = int(opts.get("parent_time_step_ratio", default_ratio))
            if ratio < 1 or parent not in dts:
                raise ValueError(f"{dom}: invalid parent time step ratio or parent domain")
            dt = dts[parent] / ratio
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError(f"{dom}: layout_restart requires a positive time.dt")
        dts[dom] = dt
        end = float(opts.get("run.run_time", run_opts.get("run_time", "0")))
        cutoff = math.floor(float(opts.get("restart.end", rst.get("end", 0))) / dt + 0.5)
        last = math.floor(end / dt + 0.5)
        end_step = min(last, cutoff) if cutoff > 0 else last
        rings = {round(sec / dt) for sec in restart_schedule(dt, interval, end_step * dt)}
        if not rings:
            raise ValueError(f"{dom}: layout_restart schedules no wrfrst files")
        native = rings | ({last} if last == end_step else set())
        origin = opts.get("run.start_date", run_opts.get("start_date")) or simulation_start(ref_by_domain[dom])
        # WRF names its files with the whole seconds of the rational model clock.
        clock_step = Fraction(dt).limit_denominator(100000)
        stamps = {times_at(origin[:19], math.floor(step * clock_step)) for step in rings}
        if len(stamps) != len(rings):
            raise ValueError(f"{dom}: multiple restart ticks have the same wrfrst timestamp")
        expected.update((dom, f"restart_{step:06d}.nc") for step in native)
        expected.update((dom, f"wrfrst_{dom}_{stamp}") for stamp in stamps)
    return expected


def layout_restart_inventory(out, expected):
    """Find both streams in the output root and its domain directories."""
    files, bad = {}, []
    for pattern in ("restart_*.nc", "wrfrst_d[0-9][0-9]_*"):
        paths = glob.glob(os.path.join(out, pattern)) + glob.glob(os.path.join(out, "d[0-9][0-9]", pattern))
        for path in sorted(paths):
            if not os.path.isfile(path):
                continue
            directory, name = os.path.split(os.path.relpath(path, out))
            domain = name.split("_")[1] if name.startswith("wrfrst_") else directory or "d01"
            key = domain, name
            if directory and directory != domain:
                bad.append(f"misplaced restart {path}, expected domain {domain}")
            if key in files:
                bad.append(f"duplicate restart {files[key]} and {path}")
            files[key] = path
    for label, keys in (("missing", expected - files.keys()), ("unexpected", files.keys() - expected)):
        bad += [f"{os.path.basename(out)} {label} {dom}/{name}" for dom, name in sorted(keys)]
    return files, bad


def layout_restart_halos(ds):
    """Required HALO_ variables of a native restart, including the real fields carried by this run."""
    # DomainRunner::save_restart_file always saves the dycore fields. PhysicsSuite::carried_int
    # serializes integers as reals in the native payload without saving their halos.
    required = {"u_1", "u_2", "v_1", "v_2", "w_1", "w_2", "t_1", "t_2", "ph_1", "ph_2", "mu_1", "mu_2",
                "p", "al", "ww", "ww_m", "php", "alt"}
    carried_ints = {"sfc_ivgtyp", "sfc_isltyp", "sfc_kpbl", "shadowmask", "noahmp_ranland", "sfc_lowlyr",
                    "sfc_isnowxy", "sfc_cropcat", "sfc_pgsxy", "sfc_i_rainc", "sfc_i_rainnc",
                    "sfc_k22_shallow", "sfc_kbcon_shallow", "sfc_ktop_shallow"}
    required.update(name for name, var in ds.variables.items()
                    if var.dimensions[:1] == ("Time",) and var.dtype.kind == "f" and name not in carried_ints
                    and not name.startswith(("sfc_i_acsw", "sfc_i_aclw")))
    halos = {"HALO_" + name for name in required}
    bad = [f"missing {name}" for name in sorted(halos - ds.variables.keys())]
    for name in sorted(halos & ds.variables.keys()):
        v = ds[name]
        if v.ndim != 1 or not v.size or v.dtype.name != "float64":
            bad.append(f"{name}: expected a nonempty 1-D double array, got {v.shape} {v.dtype}")
    return len(halos), bad


def layout_restart(b, cfg, sets, refs, t, lay, iv, out):
    """The restart files do not depend on the decomposition: CPPWRF's run to `t` s with restart files every `iv` s as
    one patch and as run.tiles=`lay` patches must write the same files, CPPWRF's own restart_<step>.nc (with the
    halo points HALO_<name>) and the wrfrst files, each identical (exact.py: every variable at tolerance 0, the
    dimensions, types, variable and global attributes, all of them). `refs`: the WRF run whose inputs (and
    namelist.input, when there) the case reads; not compared. (differences, summary)"""
    import netCDF4
    import exact
    match = re.fullmatch(r"([1-9]\d*)(?:[xX]([1-9]\d*))?", lay)
    if not match or (layout := (int(match[1]), int(match[2] or 1))) == (1, 1):
        return [f"layout_restart needs a decomposed comparison layout, got {lay!r}"], ""
    lay = f"{layout[0]}x{layout[1]}"
    nl = os.path.join(os.path.dirname(next(iter(refs.values()))), "namelist.input")
    common = sets + ["output.format=wrfout", f"run.run_time={t}", f"restart.interval={iv}"] + \
        ([f"output.wrf_namelist={nl}"] if os.path.exists(nl) else [])
    try:
        expected = layout_restart_schedule(configuration(cfg, common), refs)
    except (AttributeError, KeyError, OSError, ValueError) as e:
        return [f"layout_restart schedule: {e}"], ""
    one, tiled = os.path.join(out, "a_one"), os.path.join(out, "b_tiled")
    for d, workers in ((one, (1, 1)), (tiled, layout)):
        rc, log = run(b, cfg, common + [f"run.tiles={workers[0]}x{workers[1]}"], d, clear_env=_LAYOUT_ENV)
        if rc != 0:
            return [f"{os.path.basename(d)} run failed: {log[-200:]}"], ""
        error = layout_restart_worker_error(log, workers)
        if error:
            return [f"{os.path.basename(d)}: {error}"], ""

    A, inventory_a = layout_restart_inventory(one, expected)
    B, inventory_b = layout_restart_inventory(tiled, expected)
    bad = inventory_a + inventory_b
    for key in sorted(A.keys() & B.keys()):
        path_a, path_b = os.path.relpath(A[key], one), os.path.relpath(B[key], tiled)
        if path_a != path_b:
            bad.append(f"restart location differs: a_one {path_a}, b_tiled {path_b}")
    if bad:
        return bad, ""
    nat, nhalo = 0, 0
    for key in sorted(expected):
        dom, name = key
        label = f"{dom}/{name}"
        with _NC, netCDF4.Dataset(A[key]) as x, netCDF4.Dataset(B[key]) as y:
            exact.raw(x), exact.raw(y)
            if name.startswith("restart_"):
                nat += 1
                count, bad_x = layout_restart_halos(x)
                _, bad_y = layout_restart_halos(y)
                nhalo += count
                bad += [f"{side} {label}: {msg}" for side, errors in (("a_one", bad_x), ("b_tiled", bad_y))
                        for msg in errors]
            bad += [f"{label}: {m}" for m in exact.metadata(x, y, ignore=())]
            for v in x.variables:
                if v not in y.variables:
                    continue
                dif = exact.describe(x[v][:], y[v][:])
                if dif:
                    bad.append(f"{label}.{v}: {dif}")
    if bad:
        return bad, ""
    return [], (f"{lay} = one patch: {nat} restart_<step>.nc ({nhalo} HALO_ variables), "
                f"{len(expected) - nat} wrfrst files identical")


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


def y_slab(wrfout):
    """WRF's run of wrfout is a periodic-y slab with e_sn = 3 (namelist.input next to it): compare_real.py --y-slab."""
    nl = os.path.join(os.path.dirname(wrfout), "namelist.input")
    if not os.path.exists(nl):
        return False
    txt = open(nl).read().lower()
    per = re.search(r"^\s*periodic_y\s*=\s*\.true\.", txt, re.M)
    esn = re.search(r"^\s*e_sn\s*=\s*3\s*,", txt, re.M)
    return bool(per and esn)


def compare(wrfout, out, t):
    r = subprocess.run([sys.executable, os.path.join(REPO, "tests/verify/compare_real.py"), wrfout, out]
                       + (["--y-slab"] if y_slab(wrfout) else []), capture_output=True, text=True)
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
            if f[1] in res:
                raise RuntimeError(f"duplicate L2 metric for {f[1]} at t={t:g}s")
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
    seed = os.path.join(a, "restart_000001.nc")
    if not os.path.isfile(seed):
        return False, f"missing native restart seed {seed}"
    rc, log = run(binary, cfg, [f"run.restart_file={seed}"], b)
    if rc != 0:
        return False, log[-300:]
    error = compare_snapshots(a, b, 3, 3 * float(configuration(cfg, [])["time"]["dt"]))
    return not error, error or "step 3 bit-identical"


def compare_snapshots(first, second, step, time_sec):
    """Validate the snapshot headers and payload extent before comparing their bytes."""
    headers, binaries = [], []
    for path in (first, second):
        base = os.path.join(path, f"snap_{step:06d}")
        try:
            with open(base + ".json") as f:
                header = json.load(f)
            size = os.path.getsize(base + ".bin")
            if size < 4 or header["step"] != step or not math.isclose(header["time"], time_sec, abs_tol=1e-6):
                return f"{base}: invalid snapshot size, step or time"
            offset, names = 0, set()
            for v in header["variables"]:
                width = {"float32": 4, "float64": 8}[v["dtype"]]
                shape = v["shape"]
                if len(shape) != 3 or any(type(n) is not int or n < 1 for n in shape):
                    return f"{base}: invalid snapshot shape for {v['name']}"
                nbytes = math.prod(shape) * width
                if v["offset"] != offset or v["nbytes"] != nbytes or v["name"] in names or not v["name"]:
                    return f"{base}: invalid snapshot descriptor for {v['name']}"
                names.add(v["name"])
                offset += nbytes
            if offset != size:
                return f"{base}: snapshot descriptors cover {offset} of {size} bytes"
        except (OSError, KeyError, ValueError, TypeError) as exc:
            return f"{base}: invalid snapshot: {exc}"
        headers.append(header)
        binaries.append(base + ".bin")
    if headers[0] != headers[1]:
        return f"step {step} differs after restart"
    with open(binaries[0], "rb") as a, open(binaries[1], "rb") as b:
        for chunk in iter(lambda: a.read(1024 * 1024), b""):
            if b.read(len(chunk)) != chunk:
                return f"step {step} differs after restart"
    return ""


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


def compare_seeded_restart(resumed, rsm, start, t, *, interval=60000, pending=120, dt=60):
    """Compare the minute-1 WRF-seeded leg with the references made by make_resumed_refs.sh."""
    import exact
    # These runs write history every minute. The seed's restart alarm first rings at minute 2.
    end = times_at(start, t)
    seed = times_at(start, 60)
    expected = [times_at(start, sec) for sec in range(120, int(t) + 1, 60)]
    history = stream_files(rsm, "wrfout", "d01")
    stats, noise = {}, set()
    with _NC:
        dif, _ = exact.compare_stream(stream_files(resumed, "wrfout", "d01"), history, subset=True,
                                      until=end, stats=stats, required_times=expected, wrf_reference=True,
                                      waive=uninitialised, noise=noise)
    bad = [f"wrfout {k}: {v}" for k, v in dif.items()]
    required = {times_at(start, s) for s in restart_schedule(dt, interval, t, begin=60, pending=pending)}
    ref, errors = restart_inventory(stream_files(rsm, "wrfrst", "d01"), "d01", start, required,
                                    side="WRF restart reference", seed=seed)
    mine, errors_c = restart_inventory(stream_files(resumed, "wrfrst", "d01"), "d01", start, required,
                                       side="CPPWRF", seed=seed)
    bad += errors + errors_c
    if not required:
        bad.append("seeded restart schedule has no restart records")
    shared = sorted(required & ref.keys() & mine.keys())
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
    t, dt = (float(case[5]) if tag == "p8" else float(ini.get("run", "run_time"))), float(ini.get("time", "dt"))
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
    seed = os.path.join(a, "restart_000001.nc")
    if not os.path.isfile(seed):
        return False, f"missing native restart seed {seed}"
    rc, log = run(binary, cfg, sets + [f"run.restart_file={seed}"], b)
    if rc != 0:
        return False, f"restarted run failed: {log[-300:]}"
    error = compare_snapshots(a, b, 3, 3 * dt)
    if error:
        return False, f"self: {error}"
    bad = []
    wrf = sets + ["output.format=wrfout", f"output.wrf_namelist={nl}"]
    # (2) resumed from CPPWRF's restart file at mid against WRF's uninterrupted run
    first = a
    if mid != dt:  # a run of its own whose restart alarm rings at mid only
        first = os.path.join(out, "first")
        rc, log = run(binary, cfg, sets + [f"restart.interval={mid:g}"], first)
        if rc != 0:
            return False, f"run to the restart at {mid:g}s failed: {log[-300:]}"
    c = os.path.join(out, "resumed_wrfout")
    seed = os.path.join(first, f"restart_{round(mid / dt):06d}.nc")
    if not os.path.isfile(seed):
        return False, f"missing native restart seed {seed}"
    rc, log = run(binary, cfg, wrf + [f"run.restart_file={seed}"], c)
    if rc != 0:
        return False, f"resumed run (WRF history output) failed: {log[-300:]}"
    with _NC, netCDF4.Dataset(ref) as ds:
        start = ds.getncattr("SIMULATION_START_DATE")
    # (a run resumed from CPPWRF's own file continues the run that wrote it: START_DATE is that run's, WRF's
    # uninterrupted run's; every global attribute is compared)
    split = times_at(start, mid)
    cadence = chain_history(configuration(cfg, sets), ["d01"], t)["d01"]
    required = {s for s in history_schedule(start, cadence, times_at(start, t)) if s > split}
    bad += history_inventory(stream_files(c, "wrfout", "d01"), required, start)
    b2, w2 = exact_case({"": ref}, c, t, wrf, after=split, cfg=cfg, reference_schedules={"d01": required})
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
        stamp = times_at(start, 60)
        seed = f"{WR}/{rst_ref}/wrfrst_d01_{stamp}"
        _, errors = restart_inventory([seed] if os.path.isfile(seed) else [], "d01", start, {stamp}, side="WRF seed")
        if errors:
            return False, "; ".join(errors)
        with _NC, netCDF4.Dataset(seed) as ds:
            left = int(ds.getncattr(RESTART_ALARM))
        if left != 60:
            return False, f"{seed}: {RESTART_ALARM} {left}, expected 60"
        rsets = sets + ["output.format=wrfout", f"output.wrf_namelist={rnl}",
                        f"restart.interval={interval:g}",
                        f"run.restart_file={seed}"]
        rc, log = run(binary, cfg, rsets, r)
        if rc != 0:
            return False, f"run resumed from WRF's wrfrst file failed: {log[-300:]}"
        b3, w3 = compare_seeded_restart(r, rsm, start, t, interval=interval, pending=60 + left, dt=dt)
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
    ap.add_argument("--allow-skip", action="store_true", help="permit reported SKIPs for missing data, even if all checks skip")
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
    only = a.only.split(",") if a.only is not None else None
    if only is not None and any(not o.strip() for o in only):
        ap.error("--only contains an empty selector")

    def wanted(name):
        return not only or (name in only if a.exact else any(o in name for o in only))

    known = [c[0] for c in CASES] + SPECIAL
    unknown = [o for o in only or [] if not any(o == n if a.exact else o in n for n in known)]
    if unknown:  # a misspelt case would otherwise make an empty suite that passes
        print(f"ERROR no case named {', '.join(unknown)}", flush=True)
        sys.exit(2)

    def done(path, good):  # the outputs of passing cases are deleted (they take tens of GB)
        if good and not a.keep:
            shutil.rmtree(path, ignore_errors=True)

    def case(name, b, cfg, sets, refs, t, tol):
        out = os.path.join(a.out, name)
        if all(x == 0.0 for x in tol.values()) and not chain_at(sets):
            errors = reference_domain_errors(cfg, sets, refs)
            if errors:
                return False, f"FAIL  {name:22s} {'; '.join(errors)}"
        if cfg.startswith("dropin:"):  # cppwrf-wrf in a WRF run directory, its files against WRF's at tolerance 0
            rc, log = dropin_run(b, cfg, sets, out)
            if rc != 0:
                return False, f"FAIL  {name:22s} drop-in run failed: {log[-200:]}"
            bad, what = exact_case(refs, out, t, sets, cfg=cfg)
            if wrfrst_check(sets):
                b2, w2 = wrfrst_case(refs, out, cfg, sets)
                bad, what = bad + b2, f"{what}, {w2}"
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact (cppwrf-wrf): {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:6])}{' ...' if len(bad) > 6 else ''}]" if bad else ""))
        mid = restart_at(sets)
        rst_check = wrfrst_check(sets)
        native = "suite.native_restart" in sets
        chain = chain_at(sets)
        lrst = layout_restart_at(sets)
        tile_env = tile_check_env(sets)
        if tile_env and (mid or chain or lrst):
            return False, f"FAIL  {name:22s} suite.tile_check is for straight runs (no restart, chain or layout_restart)"
        sets = [x for x in sets if not x.startswith("suite.")]
        if lrst:  # the restart files of a decomposed run against those of one patch (layout_restart)
            bad, what = layout_restart(b, cfg, sets, refs, t, lrst[0], lrst[1], out)
            done(out, not bad)
            return not bad, (f"{'PASS' if not bad else 'FAIL'}  {name:22s} t={t:g}s exact: {what}"
                             + (f"  [{'; '.join(x[:300] for x in bad[:8])}{' ...' if len(bad) > 8 else ''}]" if bad else ""))
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
        rc, log = run(b, cfg, sets, out, tile_env)
        if tile_env:  # the check's run with one patch writes OUT.ref (src/main.cpp); not compared
            shutil.rmtree(out + ".ref", ignore_errors=True)
        if rc != 0:
            return False, f"FAIL  {name:22s} run failed: {log[-200:]}"
        if exact:
            bad, what = exact_case(refs, out, t, sets, cfg=cfg)
            if rst_check:
                b2, w2 = wrfrst_case(refs, out, cfg, sets)
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
    reported = set()
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
            reported.add(name)
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
    if not (cpu or gpu or reported):
        ap.error("selected no checks for the supplied binaries and options")
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
            reported.add(futs[f])
            count["SKIP" if good is None else "PASS" if good else "FAIL"] += 1
            if good:
                split[kind(futs[f])] += 1
    missing = sorted({n for n in known if wanted(n)} - reported) if only is not None else []
    if missing:
        print(f"ERROR no result for requested checks: {', '.join(missing)}", flush=True)
    print(split_line(split), flush=True)
    print(f"SUITE PASS {count['PASS']} FAIL {count['FAIL']} SKIP {count['SKIP']}", flush=True)
    sys.exit(0 if not missing and not count["FAIL"] and (a.allow_skip or not count["SKIP"]) else 1)


if __name__ == "__main__":
    main()
