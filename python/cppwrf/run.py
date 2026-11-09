# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.Run: a CPPWRF run of a WRF run directory from Python (what cppwrf-wrf does, with progress and control).

    run = cppwrf.Run("/data/case", mode="gpu-fp32")
    run.start()
    while run.running:
        print(run.progress())          # Progress(fraction=0.42, model_time='2026-10-12_01:15:00', ...)
        time.sleep(10)
    run.wait()                         # raises RunFailed with the model's message on failure

Run(...).run() does the three steps at once. Errors are typed: NamelistError (an option without a port; .check
holds the checker's result per option), RunFailed (the model stopped; .status, .log_tail), CppwrfError (the base,
also for a missing binary or file)."""
import datetime
import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from . import dropin
from . import namelist as nlm


class CppwrfError(RuntimeError):
    """A CPPWRF run that cannot start or failed."""


class NamelistError(CppwrfError):
    """namelist.input has options CPPWRF does not port (check: the per-option result, cppwrf.namelist.CheckResult)."""

    def __init__(self, message: str, check: Optional[nlm.CheckResult] = None):
        super().__init__(message)
        self.check = check


class RunFailed(CppwrfError):
    """The model stopped with an error (status: its exit status; log_tail: the last lines of rsl.error.0000)."""

    def __init__(self, message: str, status: int, log_tail: Sequence[str] = ()):
        super().__init__(message)
        self.status = status
        self.log_tail = list(log_tail)


@dataclass
class Progress:
    model_time: str  # the model time of the last finished step of domain 1 (WRF's date format)
    fraction: float  # of the run's model time, 0 to 1
    steps: int  # domain 1 steps finished
    seconds_per_step: float  # domain 1's mean wall time per step so far
    eta_seconds: Optional[float]  # wall time left at that rate
    files: List[str]  # the WRF files written so far (wrfout, wrfrst, auxhist), by their name


_MAIN = re.compile(r"Timing for main: time (\S+) on domain\s+1:\s+([0-9.]+) elapsed seconds")
_WRITE = re.compile(r"Timing for Writing (\S+) for domain")


def _sec(a: str, b: str) -> float:
    f = "%Y-%m-%d_%H:%M:%S"
    return (datetime.datetime.strptime(b, f) - datetime.datetime.strptime(a, f)).total_seconds()


class Run:
    """A run of a WRF run directory (or of the directory of a namelist.input) in one mode.

    mode: gpu-fp32, gpu-fp64, cpu-fp64 (None: the first cppwrf binary found); threads: OpenMP threads; exe: the binary;
    sets: extra configuration keys ("section.key=value"); echo: copy the log to the terminal."""

    def __init__(self, directory: str = ".", mode: Optional[str] = None, threads: Optional[int] = None,
                 exe: Optional[str] = None, sets: Sequence[str] = (), echo: bool = False):
        if os.path.isfile(directory):  # a namelist.input: its directory is the run directory
            if os.path.basename(directory) != "namelist.input":
                raise CppwrfError(f"{directory}: the namelist of a run is its directory's namelist.input")
            directory = os.path.dirname(os.path.abspath(directory))
        self.directory = os.path.abspath(directory)
        self._w = dropin.WrfRun(self.directory, mode, threads, exe, sets)
        self.echo = echo
        self._proc: Optional[subprocess.Popen] = None
        self._logs: Optional[dropin.Logs] = None
        self._threads: List[threading.Thread] = []
        self._status: Optional[int] = None
        self._lock = threading.Lock()
        self._main = []  # (model time, step seconds) of domain 1
        self._files: List[str] = []
        self._prepared = False

    # -- set-up ------------------------------------------------------------------------------------------------
    def prepare(self) -> List[str]:
        """Translate namelist.input (cppwrf_work/run.ini) and choose the binary; the command line. NamelistError when
        an option has no port, CppwrfError for the rest."""
        try:
            cmd = self._w.prepare()
        except dropin.Fatal as e:
            if self._w.translation is not None and self._w.translation.errors:
                raise NamelistError(str(e), nlm.check(self._w.nml_path)) from None
            raise CppwrfError(str(e)) from None
        self._prepared = True
        return cmd

    @property
    def mode(self) -> Optional[str]:
        return self._w.mode

    @property
    def command(self) -> List[str]:
        return list(self._w.cmd)

    @property
    def start_date(self) -> str:
        return self._w.start

    @property
    def end_date(self) -> str:
        return self._w.end

    # -- running -----------------------------------------------------------------------------------------------
    def start(self) -> "Run":
        """Start the model in the background (prepare() first when not done)."""
        if self._proc is not None:
            raise CppwrfError("this run has been started already")
        if not self._prepared:
            self.prepare()
        self._logs = dropin.Logs(self.directory, self.echo)
        self._w.header(self._logs)
        self._proc = subprocess.Popen(self._w.cmd, cwd=self.directory, env=self._w.env, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, text=True, bufsize=1, errors="replace")
        self._threads = [self._pump(self._proc.stdout, False), self._pump(self._proc.stderr, True)]
        return self

    def _pump(self, stream, stderr: bool) -> threading.Thread:
        def run():
            for raw in stream:
                line = raw.rstrip("\n")
                self._logs.line(line, stderr)
                if not stderr:
                    m = _MAIN.search(line)
                    if m:
                        with self._lock:
                            self._main.append((m.group(1), float(m.group(2))))
                    w = _WRITE.search(line)
                    if w and w.group(1) != "restart":
                        with self._lock:
                            if w.group(1) not in self._files:
                                self._files.append(w.group(1))
        th = threading.Thread(target=run, daemon=True)
        th.start()
        return th

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def progress(self) -> Progress:
        """Where the run is: domain 1's model time and the fraction of the run, from the log's timing lines."""
        with self._lock:
            main = list(self._main)
            files = list(self._files)
        total = _sec(self._w.start, self._w.end) if self._w.start and self._w.end else 0.0
        if not main:
            return Progress(self._w.start, 0.0, 0, 0.0, None, files)
        t, _ = main[-1]
        done = _sec(self._w.start, t)
        frac = min(max(done / total, 0.0), 1.0) if total > 0 else 1.0
        sps = sum(s for _, s in main) / len(main)
        left = None
        if 0 < done < total:
            left = sps * len(main) * (total - done) / done
        return Progress(t, frac, len(main), sps, left, files)

    def cancel(self, timeout: float = 30.0) -> None:
        """Stop the model (SIGTERM, then SIGKILL after timeout seconds). wait() then raises RunFailed."""
        if self.running:
            self._proc.terminate()
            try:
                self._proc.wait(timeout)
            except subprocess.TimeoutExpired:
                self._proc.kill()

    def wait(self, timeout: Optional[float] = None) -> "Run":
        """Wait for the end of the run; RunFailed when the model failed or was cancelled, TimeoutError on timeout."""
        if self._proc is None:
            raise CppwrfError("this run has not been started")
        t0 = time.monotonic()
        while self._proc.poll() is None:
            if timeout is not None and time.monotonic() - t0 > timeout:
                raise TimeoutError(f"the run of {self.directory} did not end within {timeout:g} s")
            time.sleep(0.2)
        if self._status is None:
            for th in self._threads:
                th.join()
            self._proc.stdout.close()
            self._proc.stderr.close()
            rc = self._proc.returncode
            if rc == 0:
                self._logs.line(self._w.success_line(), terminal=self.echo)
            else:
                self._logs.fatal(dropin.failure_message(self._logs, rc, "cppwrf"), terminal=False)
            tail = list(self._logs.tail)
            self._logs.close()
            self._status = rc
            self._tail = tail
        if self._status != 0:
            msg = dropin.failure_message_from(self._tail, self._status, "cppwrf")
            raise RunFailed(msg, self._status, self._tail)
        return self

    def run(self, timeout: Optional[float] = None) -> "Run":
        """prepare(), start() and wait()."""
        return self.start().wait(timeout)

    @property
    def returncode(self) -> Optional[int]:
        return self._status

    # -- results -----------------------------------------------------------------------------------------------
    def wrfout(self, domain: int = 1) -> List[str]:
        """The run's history files of a domain (history_outname), in time order."""
        nml = nlm.read(self._w.nml_path)
        pat = nlm.get(nml, "time_control", "history_outname", "wrfout_d<domain>_<date>")
        rx = re.compile("^" + re.escape(pat).replace(re.escape("<domain>"), f"{domain:02d}")
                        .replace(re.escape("<date>"), r"\d{4}-\d\d-\d\d_\d\d:\d\d:\d\d") + "$")
        return sorted(os.path.join(self.directory, f) for f in os.listdir(self.directory) if rx.match(f))

    def open(self, domain: int = 1, **kw):
        """The run's history files of a domain as cppwrf.open_wrfout gives them."""
        from .wrfout import open_wrfout
        return open_wrfout(self.wrfout(domain), **kw)
