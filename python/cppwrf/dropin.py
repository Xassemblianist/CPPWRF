# SPDX-License-Identifier: GPL-3.0-or-later
"""Drop-in wrf.exe and real.exe: cppwrf-wrf, cppwrf-realexe and cppwrf-check.

Run inside a WRF run directory, cppwrf-wrf reads namelist.input there, translates it with tools/wrf2cppwrf.py, runs
the model and writes WRF's files (wrfout_d0N_<date>, wrfrst_d0N_<date>, auxhist3_d0N_<date>, by history_outname,
rst_outname and auxhist3_outname) into that directory, with WRF's logs: rsl.out.0000 and rsl.error.0000 with the
"Timing for main" and "Timing for Writing" lines and "d01 <date> wrf: SUCCESS COMPLETE WRF" at the end. CPPWRF's
own files (the configuration run.ini, run.json, diag.csv) go to cppwrf_work/. A failure is a nonzero exit status
and WRF's "FATAL CALLED" block in the logs. The model runs exactly as `cppwrf run.ini` does: only the directory of
the WRF files, the timing lines and CPPWRF's own restart files (off by default; --native-restart) differ."""
import argparse
import datetime
import os
import signal
import subprocess
import sys
import threading
from typing import IO, List, Optional, Sequence

from . import _locate
from . import namelist as nlm
from ._version import __version__

WORK = "cppwrf_work"


class Fatal(Exception):
    """A drop-in run that cannot start or failed: the message of WRF's FATAL CALLED block."""

    def __init__(self, message: str, status: int = 1):
        super().__init__(message)
        self.status = status


def add_seconds(date: str, s: float) -> str:
    t = datetime.datetime.strptime(date[:19], "%Y-%m-%d_%H:%M:%S") + datetime.timedelta(seconds=round(s))
    return f"{t.year:04d}-{t.month:02d}-{t.day:02d}_{t.hour:02d}:{t.minute:02d}:{t.second:02d}"  # %Y: no 0001


def seconds_between(a: str, b: str) -> float:
    f = "%Y-%m-%d_%H:%M:%S"
    return (datetime.datetime.strptime(b[:19], f) - datetime.datetime.strptime(a[:19], f)).total_seconds()


def global_att(path: str, name: str) -> str:
    """A global attribute of a NetCDF file (netCDF4-python, else ncdump -h)."""
    try:
        import netCDF4
        with netCDF4.Dataset(path) as ds:
            v = ds.getncattr(name)
            return v.decode() if isinstance(v, bytes) else str(v)
    except ImportError:
        pass
    try:
        out = subprocess.run(["ncdump", "-h", path], capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        raise Fatal(f"cannot read {path} (install python3-netcdf4 or ncdump): {e}")
    for line in out.splitlines():
        s = line.strip()
        if s.startswith(f":{name} ="):
            return s.split("=", 1)[1].strip().rstrip(";").strip().strip('"')
    raise Fatal(f"{path} has no global attribute {name}")


class Logs:
    """rsl.out.0000 (the model's standard output) and rsl.error.0000 (standard output and error), as WRF's
    distributed-memory build writes them for task 0; echo: also to this process's standard output / error."""

    def __init__(self, run_dir: str, echo: bool = False):
        self.out = open(os.path.join(run_dir, "rsl.out.0000"), "w", encoding="utf-8")
        self.err = open(os.path.join(run_dir, "rsl.error.0000"), "w", encoding="utf-8")
        self.echo = echo
        self.lock = threading.Lock()
        self.tail: List[str] = []  # the last lines of standard error, for the FATAL message

    def line(self, text: str, stderr: bool = False, terminal: bool = False) -> None:
        with self.lock:
            if not stderr:
                self.out.write(text + "\n")
                self.out.flush()
            self.err.write(text + "\n")
            self.err.flush()
            if stderr:
                self.tail = (self.tail + [text])[-20:]
            if self.echo or terminal:
                s = sys.stderr if stderr else sys.stdout
                s.write(text + "\n")
                s.flush()

    def pump(self, stream: IO[str], stderr: bool) -> threading.Thread:
        def run():
            for raw in stream:
                self.line(raw.rstrip("\n"), stderr)
        th = threading.Thread(target=run, daemon=True)
        th.start()
        return th

    def fatal(self, message: str, terminal: bool = True) -> None:
        """WRF's wrf_error_fatal block (both logs and, with terminal, this process's standard error)."""
        block = ["-------------- FATAL CALLED ---------------", "FATAL CALLED FROM FILE:  <cppwrf>"]
        block += message.rstrip("\n").split("\n") + ["-------------------------------------------"]
        with self.lock:
            for b in block:
                self.out.write(b + "\n")
                self.err.write(b + "\n")
                if terminal or self.echo:
                    sys.stderr.write(b + "\n")
            self.out.flush()
            self.err.flush()

    def close(self) -> None:
        self.out.close()
        self.err.close()


def run_logged(cmd: Sequence[str], cwd: str, env: dict, logs: Logs) -> int:
    """Run a binary with its output in the logs; Ctrl-C stops it."""
    p = subprocess.Popen(list(cmd), cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                         bufsize=1, errors="replace")
    threads = [logs.pump(p.stdout, False), logs.pump(p.stderr, True)]
    try:
        rc = p.wait()
    except KeyboardInterrupt:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=30)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
        rc = 130
    for t in threads:
        t.join()
    return rc


def failure_message(logs: Logs, rc: int, what: str) -> str:
    return failure_message_from(logs.tail, rc, what)


def failure_message_from(tail: Sequence[str], rc: int, what: str) -> str:
    """The FATAL message of a failed binary: its error lines (the end of its standard error) and its exit status."""
    errs = [x for x in tail if x.strip()]
    main = [x.split("error: ", 1)[1] if "error: " in x else x for x in errs if "error" in x.lower()] or errs[-3:]
    sig = f" (signal {-rc})" if rc < 0 else ""
    return "\n".join(main[-5:] + [f"{what} stopped with exit status {rc}{sig}"])


def exit_status(rc: int) -> int:
    return rc if 0 < rc < 256 else (128 - rc if rc < 0 else 1)


# -----------------------------------------------------------------------------------------------------------------
# cppwrf-wrf

class WrfRun:
    """A drop-in wrf.exe run of a WRF run directory: its configuration and command line (prepare), then the run."""

    def __init__(self, run_dir: str = ".", mode: Optional[str] = None, threads: Optional[int] = None,
                 exe: Optional[str] = None, sets: Sequence[str] = (), native_restart: bool = False):
        self.run_dir = os.path.abspath(run_dir)
        self.mode, self.threads, self.exe_arg = mode, threads, exe
        self.user_sets = list(sets)
        self.native_restart = native_restart
        self.work = os.path.join(self.run_dir, WORK)
        self.nml_path = os.path.join(self.run_dir, "namelist.input")
        self.exe = ""
        self.cmd: List[str] = []
        self.env: dict = {}
        self.start = self.end = ""
        self.max_dom = 1
        self.translation: Optional[nlm.Translation] = None

    def prepare(self) -> List[str]:
        """Translate namelist.input and write cppwrf_work/run.ini; the command line. Raises Fatal."""
        if not os.path.isfile(self.nml_path):
            raise Fatal(f"namelist.input not found in {self.run_dir} (cppwrf-wrf runs in a WRF run directory)")
        try:
            self.exe = _locate.model_binary(self.mode, self.exe_arg)
        except (_locate.NotFound, ValueError) as e:
            raise Fatal(str(e))
        self.mode = _locate.mode_of(self.exe)
        try:
            nml = nlm.read(self.nml_path)
        except Exception as e:  # noqa: BLE001 - the parser's message names the line
            raise Fatal(f"ERROR reading namelist.input: {e}")
        nml_t, restart = nlm.without_restart(nml)
        self.max_dom = int(nlm.get(nml, "domains", "max_dom", 1))
        tables = self.run_dir if os.path.exists(os.path.join(self.run_dir, "LANDUSE.TBL")) else None
        tr = nlm.translate(nml_t, namelist_path=self.nml_path, output_format="wrfout", real_dir=self.run_dir,
                           table_dir=tables)
        self.translation = tr
        if tr.errors:
            res = nlm.check(self.nml_path)
            bad = nlm.format_check(res, errors_only=True).splitlines()
            raise Fatal("CPPWRF does not port these options of namelist.input:\n" + "\n".join(bad)
                        + "\n(cppwrf-check namelist.input lists every option)")
        sets = [f"output.dir={os.path.join(self.work, 'out')}", f"output.wrf_dir={self.run_dir}",
                "output.wrf_timing=true", "output.format=wrfout"]
        if not self.native_restart:
            sets.append("restart.native=false")
        run_time = float(tr.sections["run"].get("run_time", 0))
        self.start = nlm.start_date(nml)
        if restart:
            pat = nlm.get(nml, "time_control", "rst_inname", "wrfrst_d<domain>_<date>")
            rst = os.path.join(self.run_dir, nlm.stream_filename(pat, 1, self.start))
            if not os.path.exists(rst):
                raise Fatal(f"restart = .true.: {os.path.basename(rst)} not found in {self.run_dir}")
            sim_start = global_att(rst, "SIMULATION_START_DATE")
            # CPPWRF counts run.run_time from the simulation start, WRF's run_* from the restart time
            run_time += seconds_between(sim_start, self.start)
            sets += [f"run.restart_file={rst}", f"run.run_time={run_time:g}"]
            self.end = add_seconds(sim_start, run_time)
        else:
            self.end = add_seconds(self.start, run_time)
        os.makedirs(self.work, exist_ok=True)
        ini = os.path.join(self.work, "run.ini")
        with open(ini, "w", encoding="utf-8") as f:
            f.write(tr.ini)
        self.cmd = [self.exe, ini]
        for s in sets + self.user_sets:
            self.cmd += ["--set", s]
        self.env = dict(os.environ)
        if self.threads:
            self.env["OMP_NUM_THREADS"] = str(self.threads)
        return self.cmd

    def header(self, logs: Logs) -> None:
        d = _locate.describe(self.exe) or {}
        logs.line(f"CPPWRF V{d.get('version', __version__)} MODEL (port of WRF V4.8.0), commit {d.get('commit', '?')}")
        logs.line(f" mode {self.mode}: {self.exe}")
        if self.threads:
            logs.line(f" OMP_NUM_THREADS = {self.threads}")
        for n in self.translation.notices if self.translation else []:
            logs.line(f" namelist: {n}")
        for w in self.translation.warnings if self.translation else []:
            logs.line(f" WARNING: namelist: {w}")
        logs.line(f" command: {' '.join(self.cmd)}")

    def success_line(self) -> str:
        return f"d01 {self.end} wrf: SUCCESS COMPLETE WRF"


def wrf_main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="cppwrf-wrf",
        description="Drop-in wrf.exe: run CPPWRF in a WRF run directory (namelist.input, wrfinput_d0N, wrfbdy_d01, "
                    "the tables of WRF's run/). Writes WRF's wrfout / wrfrst files there and the logs rsl.out.0000 "
                    "and rsl.error.0000.")
    ap.add_argument("--mode", choices=list(_locate.MODES), help="the build to run (default: the first cppwrf found)")
    ap.add_argument("--threads", type=int, help="OpenMP threads (default: OMP_NUM_THREADS or all)")
    ap.add_argument("--exe", help="the cppwrf binary (default: by --mode; CPPWRF_EXE)")
    ap.add_argument("--dir", default=".", help="the run directory (default: the current one)")
    ap.add_argument("--set", action="append", default=[], metavar="SECTION.KEY=VALUE",
                    help="an extra CPPWRF configuration key (advanced; repeatable)")
    ap.add_argument("--native-restart", action="store_true",
                    help="also write CPPWRF's own restart_<step>.nc files (cppwrf_work/out)")
    ap.add_argument("--echo", action="store_true", help="also print the log on the terminal")
    ap.add_argument("--dry-run", action="store_true", help="translate and print the command, do not run")
    ap.add_argument("--version", action="version", version=f"cppwrf-wrf {__version__}")
    a = ap.parse_args(argv)
    r = WrfRun(a.dir, a.mode, a.threads, a.exe, a.set, a.native_restart)
    if not os.path.isdir(r.run_dir):
        sys.stderr.write(f"cppwrf-wrf: {r.run_dir}: no such directory\n")
        return 2
    logs = Logs(r.run_dir, a.echo)
    try:
        try:
            r.prepare()
        except Fatal as e:
            logs.fatal(str(e))
            return e.status
        r.header(logs)
        if a.dry_run:
            print(" ".join(r.cmd))
            return 0
        if not a.echo:
            print(f"cppwrf-wrf: {r.mode} run of {r.run_dir} (log: rsl.out.0000, rsl.error.0000)", flush=True)
        rc = run_logged(r.cmd, r.run_dir, r.env, logs)
        if rc != 0:
            logs.fatal(failure_message(logs, rc, "cppwrf"))
            return exit_status(rc)
        logs.line(r.success_line(), terminal=True)
        return 0
    finally:
        logs.close()


# -----------------------------------------------------------------------------------------------------------------
# cppwrf-realexe

def realexe_main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="cppwrf-realexe",
        description="Drop-in real.exe: cppwrf_real in a WRF run directory (namelist.input and the met_em files). "
                    "Writes wrfinput_d0N, wrfbdy_d01 (wrflowinp_d0N with sst_update, wrffdda_d0N with grid_fdda) "
                    "there and the logs rsl.out.0000 and rsl.error.0000.")
    ap.add_argument("--exe", help="the cppwrf_real binary (default: next to cppwrf; CPPWRF_REAL_EXE)")
    ap.add_argument("--dir", default=".", help="the run directory (default: the current one)")
    ap.add_argument("--met-dir", help="the directory of the met_em files (default: the run directory)")
    ap.add_argument("--precision", choices=["double", "float"], default="double",
                    help="the real kind of the files: double as WRF's -r8 build (default; for results identical to "
                         "WRF real*8), float as WRF's default build")
    ap.add_argument("--echo", action="store_true", help="also print the log on the terminal")
    ap.add_argument("--version", action="version", version=f"cppwrf-realexe {__version__}")
    a = ap.parse_args(argv)
    run_dir = os.path.abspath(a.dir)
    if not os.path.isdir(run_dir):
        sys.stderr.write(f"cppwrf-realexe: {run_dir}: no such directory\n")
        return 2
    logs = Logs(run_dir, a.echo)
    try:
        nml_path = os.path.join(run_dir, "namelist.input")
        if not os.path.isfile(nml_path):
            logs.fatal(f"namelist.input not found in {run_dir}")
            return 1
        try:
            exe = _locate.real_binary(a.exe)
        except _locate.NotFound as e:
            logs.fatal(str(e))
            return 1
        met = os.path.abspath(a.met_dir or run_dir)
        try:
            first = nlm.stream_filename("met_em.d<domain>.<date>.nc", 1, nlm.start_date(nlm.read(nml_path)))
        except Exception as e:  # noqa: BLE001
            logs.fatal(f"ERROR reading namelist.input: {e}")
            return 1
        if not os.path.exists(os.path.join(met, first)):
            logs.fatal(f"real_em: {first} not found in {met} (link the met_em files of metgrid.exe here or use "
                       "--met-dir)")
            return 1
        cmd = [exe, "--namelist", nml_path, "--met-dir", met, "--out-dir", run_dir, "--precision", a.precision]
        logs.line(f"CPPWRF V{__version__} REAL_EM (port of WRF V4.8.0 real.exe)")
        logs.line(f" command: {' '.join(cmd)}")
        if not a.echo:
            print(f"cppwrf-realexe: {run_dir} (log: rsl.out.0000, rsl.error.0000)", flush=True)
        rc = run_logged(cmd, run_dir, dict(os.environ), logs)
        if rc != 0:
            logs.fatal(failure_message(logs, rc, "cppwrf_real"))
            return exit_status(rc)
        logs.line("real_em: SUCCESS COMPLETE REAL_EM INIT", terminal=True)
        return 0
    finally:
        logs.close()


# -----------------------------------------------------------------------------------------------------------------
# cppwrf-check

def check_main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="cppwrf-check",
        description="Which options of a WRF namelist.input CPPWRF ports, without running anything: each option is "
                    "ported, ignored (no effect, in WRF too or for CPPWRF's run) or not ported (with the reason and a "
                    "suggestion). Exit status 0 when CPPWRF runs the namelist, 1 when it does not.")
    ap.add_argument("namelist", nargs="?", default="namelist.input")
    ap.add_argument("-v", "--verbose", action="store_true", help="also list the ported options")
    ap.add_argument("--json", action="store_true", help="the result as JSON")
    ap.add_argument("--version", action="version", version=f"cppwrf-check {__version__}")
    a = ap.parse_args(argv)
    if not os.path.isfile(a.namelist):
        sys.stderr.write(f"cppwrf-check: {a.namelist}: no such file\n")
        return 2
    try:
        res = nlm.check(a.namelist)
    except Exception as e:  # noqa: BLE001 - a namelist the parser rejects
        sys.stderr.write(f"cppwrf-check: {a.namelist}: {e}\n")
        return 2
    if a.json:
        import json
        print(json.dumps({"ok": res.ok, "general": res.general,
                          "options": [{"group": o.group, "option": o.option, "values": o.values, "status": o.status,
                                       "reason": o.reason, "suggestion": o.suggestion} for o in res.options]},
                         indent=1, default=str))
    else:
        print(nlm.format_check(res, a.verbose))
    return 0 if res.ok else 1
