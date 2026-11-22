#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The sequencer of the paper benchmark (tools/bench/PROTOCOL.md): layout screening, interleaved timed repetitions
with warm-up, the untimed proof runs, the bit-identity and accuracy comparisons and the report, on one machine.

  paper_run.py plan     [OPTIONS]   the runs of every phase and the session estimate (G4 rates of docs/PERF.md)
  paper_run.py prepare  [OPTIONS]   the run directories and CPPWRF configurations of the cases
  paper_run.py screen   [OPTIONS]   WRF's process layouts and CPPWRF CPU's thread counts, history off
  paper_run.py timed    [OPTIONS]   the warm-up runs, the timed blocks (history off: REPS, history on: REPS_ON)
  paper_run.py proof    [OPTIONS]   the untimed reference runs (WRF in one process, CPPWRF CPU FP64 / FP32)
  paper_run.py verify   [OPTIONS]   tolerance-0 comparisons and the FP32 accuracy statistics
  paper_run.py report   [OPTIONS]   ROOT/out/report.md, summary.json, svg/ (bench_stats.py, bench_plot.py)
  paper_run.py all      [OPTIONS]   prepare, screen, timed, proof, verify, report
  paper_run.py make-large --base DIR --met DIR --out DIR --start YYYY-MM-DD_HH --end YYYY-MM-DD_HH [--wrf DIR]
                                    a WRF run directory from met_em files: BASE's tables and namelist with the
                                    domain size of the met_em files and the dates, real.exe of the WRF real*8 build
  paper_run.py --help

OPTIONS (defaults: the Colab G4 session of tools/bench/colab_paper.sh):
  --root DIR          work directory (default /opt/paper): runs/, out/ (raw_runs.jsonl, verify.jsonl, logs/,
                      best.json, report.md), q/ (the GPU agent's queue)
  --data DIR          the WRF run directories of the data tars (default /opt/job/data/wrf_runs)
  --wrf DIR           WRF builds DIR/WRF-r4, WRF-r8, WRF-r4sm, WRF-r8sm and the CPU builds DIR/cpu-dp,
                      cpu-dp-hwlog, cpu-sp (default /opt/wrfbench)
  --src DIR           CPPWRF's source tree (default: the one of this script)
  --cases C1,C2,...   the cases (default C1,C2,C3,C4); --case NAME=BASE:LENGTH[:SECTION.key=VALUE;...] adds or
                      replaces one (BASE: a directory, or a name under --data; LENGTH: 24h, 6h, 3m, ...)
  --models M,...      the timed models (default wrf_r4,wrf_r4t,wrf_r8,gpu_fp32,gpu_fp32id,gpu_fp64,cpu_fp64)
  --proof P,...       the proof runs (default wrf_r8_dm1,wrf_r4_dm1,cpu_fp64,cpu_fp64_hwlog,cpu_fp32)
  --layouts L,...     WRF's layouts to screen (default dm12,dm24,dm48,sm24x2,sm12x4,sm12x2,sm6x4,sm8x3; dmN: N
                      ranks, smNxT: N ranks x T threads of the dm+sm build); --layout dm24 skips the screening
  --screen-reps N     rounds of the screening after one warm-up run per model (default 2, each round over all
                      layouts in a turned order)
  --screen-min M      simulated minutes of a screening run (default 30; the case's length if shorter)
  --reps N            timed blocks with history off (default 5)
  --reps-on N         timed blocks with history on (default 3: the history-off blocks warmed up)
  --cpu-reps N        CPPWRF CPU FP64 runs with history off, in blocks 1 .. N (default 3)
  --warmup 0|1        the warm-up block: every model of every case once, history off, not counted (default 1)
  --warm-min M        simulated minutes of a warm-up run (default 30)
  --hist-min M        minutes between history frames with history on (default 60)
  --pause S           seconds at least before each GPU run (default 3)
  --cool-max S        at most S seconds of cool-down before a GPU run (default 90): the wait ends when the GPU is
  --cool-delta C      within C degrees (default 5) of its idle temperature (the agent's, at its start)
  --gpu agent|local|none   GPU runs through the agent of ROOT/q (tools/bench/bench_exec.py agent, default), here
                      (--gpu-bin NAME=PATH ...) or not at all
  --cpu-bin NAME=PATH the CPU builds (default WRF_DIR/NAME/cppwrf)
  --gpu-threads N     OMP_NUM_THREADS of the GPU runs (default 4)
  --proof-threads N   OMP_NUM_THREADS of a CPPWRF proof run (default 8); proof runs share the vCPUs
  --threads N         the vCPUs this machine gives the runs (default all: os.cpu_count())
  --nc0               also time CPPWRF GPU FP32 with HDF5's own deflate filter (CPPWRF_NC_THREADS=0), history on
  --keep              keep every run directory (default: only the outputs the comparisons need)

Models: wrf_r4 / wrf_r8 (WRF v4.8.0 real*4 / real*8 at the fastest screened layout), wrf_r4t (real*4 with
FCOPTIM = -O3 -march=native -funroll-loops, builds WRF-r4t / WRF-r4tsm), gpu_fp32 (CUDA FP32, the fast
build "cuda", -DCPPWRF_CUDA_FP32_IDENTITY=OFF), gpu_fp32id (CUDA FP32 identity build "cuda_sp_id"), gpu_fp64 ("cuda_dp"), cpu_fp64
(CPU FP64 "cpu-dp" at the fastest screened thread count). CPPWRF runs with output.wrf_timing = true (WRF's timing
lines; tools/bench/bench_parse.py splits both models' times the same way) and output.diag_interval = 1000000 (no
diag.csv reductions: WRF computes none); CPPWRF_MEMINFO=1 only in the warm-up runs. Every run is one JSON line of
ROOT/out/raw_runs.jsonl (tools/bench/bench_exec.py's result with the parsed times, the block, the run's position in
its block's order and the model before it). The order of the models in block k of case number i is row
(k x cases + i) of a Williams design (run_order): over a regime's blocks every model runs first and right after
every other model.
"""
import glob
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bench_exec  # noqa: E402
import bench_parse  # noqa: E402

NP = os.cpu_count() or 1

# name: (base, length, namelist changes, description)
CASES = {
    "C1": ("run_FP2_P4_24h_r8", "24h", ["physics.radt=15"],
           "P4 physics (WSM6, KF, YSU, revised MM5 surface layer, Noah, RRTMG LW/SW) with radt 15 min, 119 x 89 x 44 "
           "at 15 km, dt 60 s, 24 h from 2026-10-12 00 UTC (a whole day and night)"),
    "C2": ("@Lday", "6h", ["physics.radt=15"],
           "L: 359 x 269 x 44 at 15 km (4.25 M cells, WPS + real.exe on the runtime), P4 physics with radt 15 min, "
           "6 h from 2026-10-12 06 UTC (daytime: 08:20 - 14:20 local solar time at 35 E)"),
    "C3": ("run_FP2_P4_24h_r8", "6h", ["physics.mp_physics=10"],
           "MORR: P4 with Morrison two-moment microphysics (mp_physics 10) and radiation every minute (radt 1), "
           "119 x 89 x 44, 6 h from 00 UTC"),
    "C4": ("run_N3h_r8", "6h", [],
           "N3: two-way nest (d02 120 x 90 x 44 at 5 km, ratio 3, feedback, smooth_option 2), P4 physics with "
           "radt 1 on both domains, 6 h from 00 UTC"),
}
MODELS = {  # name: (kind, build)
    "wrf_r4": ("wrf", "r4"), "wrf_r8": ("wrf", "r8"), "wrf_r4t": ("wrf", "r4t"),
    "gpu_fp32": ("gpu", "cuda"), "gpu_fp32id": ("gpu", "cuda_sp_id"), "gpu_fp64": ("gpu", "cuda_dp"),
    "cpu_fp64": ("cpu", "cpu-dp"),
}
DEFAULT_MODELS = ["wrf_r4", "wrf_r4t", "wrf_r8", "gpu_fp32", "gpu_fp32id", "gpu_fp64", "cpu_fp64"]
PROOF = {  # name: (kind, build)
    "wrf_r8_dm1": ("wrf", "r8"), "wrf_r4_dm1": ("wrf", "r4"),
    "cpu_fp64": ("cpu", "cpu-dp"), "cpu_fp64_hwlog": ("cpu", "cpu-dp-hwlog"), "cpu_fp32": ("cpu", "cpu-sp"),
}
# G4 estimate (docs/PERF.md, c9c95c5; WRF in one process from the b30ec5b session's logs): stepping seconds per
# simulated hour with history off, and start-up seconds
EST = {
    "C1": dict(wrf_r4t=2.43, wrf_r4=2.43, wrf_r8=4.22, gpu_fp32=0.47, gpu_fp32id=0.55, gpu_fp64=1.36, cpu_fp64=7.79, init=2.5,
               wrf_r8_dm1=100, wrf_r4_dm1=60, cpu_fp32=6),
    "C2": dict(wrf_r4t=25.7, wrf_r4=25.7, wrf_r8=45.2, gpu_fp32=2.5, gpu_fp32id=3.0, gpu_fp64=7.5, cpu_fp64=52.4, init=10,
               wrf_r8_dm1=480, wrf_r4_dm1=280, cpu_fp32=40),
    "C3": dict(wrf_r4t=9.0, wrf_r4=9.0, wrf_r8=12.9, gpu_fp32=1.5, gpu_fp32id=1.7, gpu_fp64=2.8, cpu_fp64=13.7, init=2.5,
               wrf_r8_dm1=190, wrf_r4_dm1=130, cpu_fp32=10),
    "C4": dict(wrf_r4t=24.1, wrf_r4=24.1, wrf_r8=34.1, gpu_fp32=4.5, gpu_fp32id=5.2, gpu_fp64=8.3, cpu_fp64=38.6, init=4,
               wrf_r8_dm1=450, wrf_r4_dm1=300, cpu_fp32=30),
}
L2_VARS = ("U", "V", "W", "T", "QVAPOR", "PH", "MU", "PSFC")


# ---- options
class Opt:
    def __init__(self, argv):
        self.root, self.data, self.wrf = "/opt/paper", "/opt/job/data/wrf_runs", "/opt/wrfbench"
        self.src = os.path.dirname(os.path.dirname(HERE))
        self.cases = ["C1", "C2", "C3", "C4"]
        self.case_defs = dict(CASES)
        self.models = list(DEFAULT_MODELS)
        self.proof = list(PROOF)
        self.layouts = "dm12,dm24,dm48,sm24x2,sm12x4,sm12x2,sm6x4,sm8x3".split(",")
        self.layout = None
        self.screen_reps, self.reps, self.reps_on, self.cpu_reps, self.warmup = 2, 5, 3, 3, 1
        self.hist_min, self.pause, self.gpu, self.gpu_threads, self.proof_threads = 60, 3.0, "agent", 4, 8
        self.cool_max, self.cool_delta, self.screen_min, self.warm_min = 90.0, 5.0, 30, 30
        self.gpu_bins, self.cpu_bins, self.nc0, self.keep = {}, {}, False, False
        self.np = NP
        i = 0
        while i < len(argv):
            k = argv[i]
            v = argv[i + 1] if i + 1 < len(argv) else None
            i += 2
            if k == "--root": self.root = v  # noqa: E701
            elif k == "--data": self.data = v  # noqa: E701
            elif k == "--wrf": self.wrf = v  # noqa: E701
            elif k == "--src": self.src = v  # noqa: E701
            elif k == "--cases": self.cases = [x for x in v.split(",") if x]  # noqa: E701
            elif k == "--case":
                n, spec = v.split("=", 1)
                parts = spec.split(":", 2)
                self.case_defs[n] = (parts[0], parts[1], [x for x in (parts[2].split(";") if len(parts) > 2 else [])
                                                          if x], f"{n}: {parts[0]}, {parts[1]}")
                if n not in self.cases:
                    self.cases.append(n)
            elif k == "--models": self.models = [x for x in v.split(",") if x]  # noqa: E701
            elif k == "--proof": self.proof = [x for x in v.split(",") if x]  # noqa: E701
            elif k == "--layouts": self.layouts = [x for x in v.split(",") if x]  # noqa: E701
            elif k == "--layout": self.layout = v  # noqa: E701
            elif k == "--screen-reps": self.screen_reps = int(v)  # noqa: E701
            elif k == "--reps": self.reps = int(v)  # noqa: E701
            elif k == "--reps-on": self.reps_on = int(v)  # noqa: E701
            elif k == "--cpu-reps": self.cpu_reps = int(v)  # noqa: E701
            elif k == "--warmup": self.warmup = int(v)  # noqa: E701
            elif k == "--hist-min": self.hist_min = int(v)  # noqa: E701
            elif k == "--pause": self.pause = float(v)  # noqa: E701
            elif k == "--cool-max": self.cool_max = float(v)  # noqa: E701
            elif k == "--cool-delta": self.cool_delta = float(v)  # noqa: E701
            elif k == "--screen-min": self.screen_min = int(v)  # noqa: E701
            elif k == "--warm-min": self.warm_min = int(v)  # noqa: E701
            elif k == "--gpu": self.gpu = v  # noqa: E701
            elif k == "--gpu-bin": self.gpu_bins.update([v.split("=", 1)])  # noqa: E701
            elif k == "--cpu-bin": self.cpu_bins.update([v.split("=", 1)])  # noqa: E701
            elif k == "--gpu-threads": self.gpu_threads = max(1, int(v))  # noqa: E701
            elif k == "--proof-threads": self.proof_threads = max(1, int(v))  # noqa: E701
            elif k == "--threads": self.np = max(1, int(v))  # noqa: E701
            elif k == "--nc0":
                self.nc0 = True
                i -= 1
            elif k == "--keep":
                self.keep = True
                i -= 1
            else:
                raise SystemExit(f"unknown option {k} (paper_run.py --help)")
        for m in self.models:
            if m not in MODELS:
                raise SystemExit(f"unknown model {m}")
        for m in self.proof:
            if m not in PROOF:
                raise SystemExit(f"unknown proof run {m}")
        self.out = os.path.join(self.root, "out")
        self.q = os.path.join(self.root, "q")
        os.makedirs(self.out + "/logs", exist_ok=True)

    def cpu_bin(self, b):
        return self.cpu_bins.get(b, os.path.join(self.wrf, b, "cppwrf"))

    def base(self, c):
        b = self.case_defs[c][0]
        if b == "@Lday":
            return os.path.join(self.wrf, "run_Lday")
        return b if b.startswith("/") else os.path.join(self.data, b)


LOCK = threading.Lock()  # the proof phase runs several runs at a time: one writer at a time for the log, the records
#                           and the run ids


def log(o, msg):
    with LOCK:
        with open(os.path.join(o.out, "paper.log"), "a") as f:
            f.write(time.strftime("%H:%M:%S ") + msg + "\n")
        print(time.strftime("%H:%M:%S ") + msg, flush=True)


def emit(o, rec, name="raw_runs.jsonl"):
    line = json.dumps(rec) + "\n"
    with LOCK:
        with open(os.path.join(o.out, name), "a") as f:
            f.write(line)


# ---- namelists (as tools/bench/bench_run.py)
def setkey(nl, sec, key, val):
    m = re.search(rf"&{sec}\b(.*?)\n\s*/", nl, re.S | re.I)
    if not m:
        return nl.rstrip("\n") + f"\n\n&{sec}\n {key} = {val},\n/\n"
    body = m.group(1)
    pat = re.compile(rf"^\s*{key}\s*=.*$", re.M | re.I)
    line = f" {key} = {val},"
    body = pat.sub(line, body) if pat.search(body) else body + "\n" + line
    return nl[:m.start(1)] + body + nl[m.end(1):]


def geti(nl, key, d=0, default=0):
    m = re.search(rf"^\s*{key}\s*=\s*([^\n]*)$", nl, re.M | re.I)
    v = [x.strip().strip("'\"") for x in m.group(1).split(",") if x.strip()] if m else []
    return int(float(v[d])) if len(v) > d else default


def grid(nl, hours):
    """[[cells, steps]] of each domain over the run: (e_we - 1) (e_sn - 1) (e_vert - 1), run length / dt."""
    nd = geti(nl, "max_dom", 0, 1)
    dt0 = geti(nl, "time_step") + geti(nl, "time_step_fract_num") / max(1, geti(nl, "time_step_fract_den", 0, 1))
    dts, out = [], []
    for d in range(nd):
        p = geti(nl, "parent_id", d, 0) - 1
        dt = dt0 if d == 0 else dts[p] / geti(nl, "parent_time_step_ratio", d, 1)
        dts.append(dt)
        out.append([(geti(nl, "e_we", d) - 1) * (geti(nl, "e_sn", d) - 1) * (geti(nl, "e_vert", d) - 1),
                    round(hours * 3600 / dt)])
    return out


def length_s(spec):
    m = re.match(r"([0-9.]+)([hms])$", spec)
    if not m:
        raise SystemExit(f"length {spec}: e.g. 24h, 6h, 3m")
    return float(m.group(1)) * {"h": 3600, "m": 60, "s": 1}[m.group(2)]


SKIP = re.compile(r"(wrfout.*|wrfrst.*|auxhist.*|rsl\..*|namelist\.(output|input)|.*\.log|.*\.exe|met_em.*|FILE:.*|"
                  r"GRIBFILE.*|out_.*|diag\.csv)$")


def make_dir(R, base, sets, seconds, hist_min):
    """R: BASE's inputs (links) and its namelist for `seconds` with history every hist_min minutes (None: only the
    frame at t = 0), no restart files (restart_interval = 0: WRF writes none, and tools/wrf2cppwrf.py then gives
    CPPWRF no [restart] section, so it writes no restart file at the last step either), all frames of a domain in one
    file."""
    shutil.rmtree(R, ignore_errors=True)
    os.makedirs(R)
    for f in os.listdir(base):
        p = os.path.join(base, f)
        if os.path.isdir(p) or SKIP.match(f):
            continue
        os.symlink(os.path.realpath(p), os.path.join(R, f))
    nl = open(os.path.join(base, "namelist.input")).read()
    nd = geti(nl, "max_dom", 0, 1)
    for k in ("history_interval", "history_interval_d", "history_interval_h", "history_interval_m",
              "history_interval_s", "run_days", "run_hours", "run_minutes", "run_seconds"):
        nl = re.sub(rf"^\s*{k}\s*=.*\n", "", nl, flags=re.M | re.I)
    per = lambda v: ", ".join([str(v)] * nd)  # noqa: E731
    s = int(seconds)
    for k, v in (("run_days", "0"), ("run_hours", str(s // 3600)), ("run_minutes", str(s % 3600 // 60)),
                 ("run_seconds", str(s % 60)), ("restart", ".false."), ("restart_interval", "0"),
                 ("frames_per_outfile", per(100000)), ("history_interval", per(hist_min if hist_min else 99999)),
                 ("debug_level", "0")):
        nl = setkey(nl, "time_control", k, v)
    for x in sets:
        sk, v = x.split("=", 1)
        sec, k = sk.split(".", 1)
        nl = setkey(nl, sec, k, v)
    open(os.path.join(R, "namelist.input"), "w").write(nl)
    return nl


# ---- the run directories
def regimes(o):
    """History setting of each regime (minutes between frames, None: only t = 0): screen (the layout screening),
    warm (the warm-up runs), off and on (the timed blocks)."""
    return {"screen": None, "warm": None, "off": None, "on": o.hist_min}


def regime_seconds(o, regime, sec):
    if regime == "screen":
        return min(sec, o.screen_min * 60)
    if regime == "warm":
        return min(sec, o.warm_min * 60)
    return sec


def rdir(o, c, regime):
    return os.path.join(o.root, "runs", c, f"cpp_{regime}")


def prepare(o):
    info = {}
    for c in o.cases:
        base, length, sets, desc = o.case_defs[c]
        b = o.base(c)
        if not os.path.exists(os.path.join(b, "namelist.input")):
            log(o, f"{c}: no inputs ({b})")
            info[c] = dict(ok=False, error=f"no inputs {b}")
            continue
        sec = length_s(length)
        info[c] = dict(ok=True, base=b, length=length, seconds=sec, sets=sets, desc=desc)
        for reg, hm in regimes(o).items():
            s = regime_seconds(o, reg, sec)
            R = rdir(o, c, reg)
            nl = make_dir(R, b, sets, s, hm)
            t = subprocess.run([sys.executable, o.src + "/tools/wrf2cppwrf.py", R + "/namelist.input", "--real-dir", R,
                                "--format", "wrfout", "-o", R + "/case.ini"], capture_output=True, text=True)
            open(R + "/translate.log", "w").write(t.stdout + t.stderr)
            info[c][reg] = dict(dir=R, seconds=s, grid=grid(nl, s / 3600), translate=t.returncode)
            if t.returncode:
                log(o, f"{c} {reg}: wrf2cppwrf.py FAILED: {(t.stdout + t.stderr).strip().splitlines()[-1:]}")
            os.makedirs(f"{o.out}/logs/{c}", exist_ok=True)
            for f in ("namelist.input", "case.ini", "translate.log"):
                if os.path.exists(os.path.join(R, f)):
                    shutil.copy(os.path.join(R, f), f"{o.out}/logs/{c}/{reg}.{f}")
        preread(b)
        log(o, f"{c}: prepared ({desc})")
    json.dump(info, open(o.out + "/cases.json", "w"), indent=1)
    return info


def preread(d):
    """Reads every input file of a run directory once (the page cache: the first timed run reads them as the others)."""
    for f in os.listdir(d):
        p = os.path.join(d, f)
        if os.path.isfile(p) and not SKIP.match(f):
            with open(p, "rb") as fh:
                while fh.read(1 << 24):
                    pass


def cases_info(o):
    p = o.out + "/cases.json"
    return json.load(open(p)) if os.path.exists(p) else prepare(o)


# ---- one run
CORES = None


def topology():
    global CORES
    if CORES is None:
        out = subprocess.run("lscpu -p=CORE,SOCKET | grep -v '^#' | sort -u | wc -l", shell=True, capture_output=True,
                             text=True).stdout.strip()
        CORES = int(out) if out.isdigit() and int(out) > 0 else NP
    return CORES


BIND_OK = None


def bind_ok():
    """Whether this mpiexec (MPICH's Hydra) accepts -bind-to: probed once (an MPI without it runs unbound)."""
    global BIND_OK
    if BIND_OK is None:
        try:
            BIND_OK = subprocess.run("mpiexec -n 1 -bind-to core:1 true", shell=True, capture_output=True,
                                     timeout=60).returncode == 0
        except subprocess.TimeoutExpired:
            BIND_OK = False
    return BIND_OK


def bind(n, pinned=True):
    if not pinned or not shutil.which("mpiexec") or not bind_ok():
        return ""
    k = topology() // n
    return f"-bind-to core:{k}" if k >= 1 else "-bind-to hwthread"


RUN_ID = [0]


def new_id(o, tag):
    with LOCK:
        RUN_ID[0] += 1
        n = RUN_ID[0]
    return f"{time.strftime('%H%M%S')}_{n:05d}_{tag}"


def wrf_job(o, c, regime, prec, layout, rid, pinned=True):
    """A WRF run directory and its job (None, reason without the executable)."""
    m = re.match(r"(dm|sm)(\d+)(?:x(\d+))?$", layout)
    n, t = int(m.group(2)), int(m.group(3) or 1)
    exe = f"{o.wrf}/WRF-{prec}{'sm' if m.group(1) == 'sm' else ''}/main/wrf.exe"
    if not os.path.exists(exe):
        return None, f"no executable {exe}"
    info = cases_info(o)[c]
    R = os.path.join(o.root, "runs", c, rid)
    make_dir(R, info["base"], info["sets"], info[regime]["seconds"], regimes(o)[regime])
    os.symlink(exe, R + "/wrf.exe")
    if prec.startswith("r4"):  # the run directories are real*8 runs': WRF real*4 reads the single-precision tables
        run = f"{o.wrf}/WRF-{prec}/run"
        for f in os.listdir(run):
            if os.path.exists(f"{run}/{f}_DBL") and os.path.lexists(f"{R}/{f}"):
                os.remove(f"{R}/{f}")
                os.symlink(os.path.realpath(f"{run}/{f}"), f"{R}/{f}")
    env = dict(OMP_NUM_THREADS=t, OMP_STACKSIZE="512M")
    if m.group(1) == "sm" and pinned:  # one place per core while the ranks' threads fit the cores, else per thread
        env.update(OMP_PROC_BIND="close", OMP_PLACES="cores" if n * t <= topology() else "threads")
    cmd = f"ulimit -s unlimited; exec mpiexec -n {n} {bind(n, pinned)} ./wrf.exe"
    return dict(id=rid, cwd=R, cmd=cmd, env=env, log=R + "/wrf.log", timeout=6 * 3600, ranks=n, threads=t), None


def cpp_job(o, c, regime, build, rid, threads, kind, warm=False, env_extra=None, pinned=True):
    R = rdir(o, c, regime)
    out = os.path.join(R, f"out_{rid}")
    env = dict(OMP_NUM_THREADS=threads)
    if kind == "cpu" and pinned:
        env.update(OMP_PROC_BIND="close", OMP_PLACES="cores" if threads <= topology() else "threads")
    if warm:
        env["CPPWRF_MEMINFO"] = 1
    env.update(env_extra or {})
    cmd = (f"exec {{bin:{build}}} {R}/case.ini --set output.dir={out} --set output.wrf_timing=true "
           f"--set output.diag_interval=1000000")
    job = dict(id=rid, cwd=R, cmd=cmd, env=env, log=os.path.join(R, f"{rid}.log"), timeout=6 * 3600, out=out,
               threads=threads)
    if kind == "gpu":  # the steady window of the energy: domain 1's first to last "Timing for main" line
        job.update(telemetry=True, csv=os.path.join(R, f"{rid}.gpu.csv"), pause=o.pause, pre_idle=1.5,
                   cool_max=o.cool_max, cool_delta=o.cool_delta, stamp=r"Timing for main: .* on domain +1:")
    return job


def gpu_ready(o, timeout=7200):
    if o.gpu != "agent":
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(o.root + "/gpu_ready"):
            return True
        time.sleep(10)
    return False


def execute(o, job, kind, build):
    if kind == "gpu":
        if o.gpu == "none":
            return dict(job, exit=127, wall=0.0, error="no GPU (--gpu none)")
        if o.gpu == "local":
            if build not in o.gpu_bins:
                return dict(job, exit=127, wall=0.0, error=f"no GPU build {build}")
            return bench_exec.execute(job, o.gpu_bins)
        r = bench_exec.submit(o.q, job, job["timeout"] + 600)
        if r is None:  # the agent died: no more GPU runs in this session
            o.gpu = "none"
            log(o, "no answer from the GPU agent: the remaining GPU runs are skipped")
            return dict(job, exit=125, wall=0.0, error="no answer from the GPU agent")
        return r
    if kind == "cpu":
        b = o.cpu_bin(build)
        if not os.path.exists(b):
            return dict(job, exit=127, wall=0.0, error=f"no executable {b}")
        return bench_exec.execute(job, {build: b})
    return bench_exec.execute(job, {})


def finish(o, c, res, rec, kind, keep_out=False):
    """Parses the run's log, saves it to out/logs, emits the record; removes the run's files unless kept."""
    text = ""
    if kind == "wrf":
        p = os.path.join(res["cwd"], "rsl.out.0000")
        text = open(p, errors="replace").read() if os.path.exists(p) else ""
        ok = res.get("exit") == 0 and "SUCCESS COMPLETE WRF" in text
    else:
        p = res.get("log", "")
        text = open(p, errors="replace").read() if os.path.exists(p) else ""
        ok = res.get("exit") == 0 and "cppwrf: done" in text
    # the CUDA builds' steps overlap the host (CUDA graphs, bench_parse.py --overlap); a nested run waits inside
    # domain 1's first step (the nest's frame at t = 0), so its step 1 holds its own device work
    overlap = kind == "gpu" and len(rec.get("grid") or []) == 1
    tm = bench_parse.parse(text, "wrf" if kind == "wrf" else "cppwrf", res.get("wall"), overlap)
    rec.update({k: res.get(k) for k in ("id", "exit", "wall", "t0", "t1", "load", "cpu_mhz", "cpu_mhz_run", "cpu_share",
                                        "clock_step_s", "cool", "stamps", "rapl_j", "gpu", "cmd", "env", "error")
                if k in res})
    rec.update(ok=ok and tm.get("ok", False), timing=tm)
    mem = re.findall(r"cppwrf: device memory ([^:]*): ([0-9]+) MiB used", text)
    if mem:
        rec["meminfo"] = {w: int(v) for w, v in mem}
    if not rec["ok"]:
        err = ""
        if kind == "wrf" and os.path.exists(os.path.join(res["cwd"], "rsl.error.0000")):
            err = open(os.path.join(res["cwd"], "rsl.error.0000"), errors="replace").read()
        rec.setdefault("error", [x.strip() for x in (err + text).splitlines()
                                 if re.search(r"ERROR|FATAL|signal|rror|xception", x)][-6:] or text.strip().splitlines()[-4:])
    os.makedirs(f"{o.out}/logs/{c}", exist_ok=True)
    if text:
        open(f"{o.out}/logs/{c}/{rec['id']}.log", "w").write(text)
    if res.get("csv") and os.path.exists(res["csv"]):
        shutil.copy(res["csv"], f"{o.out}/logs/{c}/{rec['id']}.gpu.csv")
    emit(o, rec)
    tmv = tm.get("stepping")
    log(o, f"{c} {rec['phase']} {rec['model']} {rec.get('layout', '')} {rec['regime']} block {rec.get('block')}: "
        f"{'ok' if rec['ok'] else 'FAILED'} wall {res.get('wall', 0):.1f} s, stepping "
        f"{tmv if tmv is None else round(tmv, 2)} s" + (f", GPU {res['gpu'].get('energy_j') or 0:.0f} J"
                                                         if res.get("gpu") else ""))
    if not o.keep and not (keep_out and rec["ok"]):
        if kind == "wrf":
            shutil.rmtree(res["cwd"], ignore_errors=True)
        elif res.get("out"):
            shutil.rmtree(res["out"], ignore_errors=True)
    return rec


def run_model(o, c, phase, model, regime, block=None, warm=False, layout=None, threads=None, kind=None, build=None,
              keep_out=False, pinned=True, env_extra=None, tag=None, extra=None):
    kind, build = (kind, build) if kind else MODELS[model]
    info = cases_info(o)[c]
    rid = new_id(o, f"{c}_{tag or model}_{regime}")
    rec = dict(phase=phase, case=c, model=tag or model, build=build, kind=kind, regime=regime, block=block, warmup=warm,
               sim_seconds=info[regime]["seconds"], grid=info[regime]["grid"], **(extra or {}))
    if kind == "wrf":
        job, why = wrf_job(o, c, regime, build, layout, rid, pinned)
        rec["layout"] = layout
        if not job:
            rec.update(id=rid, ok=False, error=why)
            emit(o, rec)
            log(o, f"{c} {phase} {rec['model']} {layout} {regime}: {why}")
            return rec, None
        rec.update(ranks=job["ranks"], threads=job["threads"])
    else:
        t = threads or (o.gpu_threads if kind == "gpu" else o.np)
        job = cpp_job(o, c, regime, build, rid, t, kind, warm, env_extra, pinned)
        rec.update(layout=("gpu" if kind == "gpu" else f"omp{t}"), threads=t)
    res = execute(o, job, kind, build)
    rec = finish(o, c, res, rec, kind, keep_out)
    return rec, (res["cwd"] if kind == "wrf" else job.get("out"))


# ---- the order of the runs
def williams(n):
    """A Williams design for n treatments: rows in which every treatment takes every position and immediately follows
    every other treatment equally often (n rows for even n, 2 n for odd n)."""
    first, lo, hi = [0], 1, n - 1
    while len(first) < n:
        first.append(lo)
        lo += 1
        if len(first) < n:
            first.append(hi)
            hi -= 1
    rows = [[(x + i) % n for x in first] for i in range(n)]
    if n % 2:
        rows += [list(reversed(r)) for r in rows]
    return rows


def run_order(models, k, ci, ncases, present=None):
    """The models of block k of case number ci in the order of row (k ncases + ci) of a Williams design over
    `models` (the regime's full list); models not in `present` are left out of that block. Over the blocks and
    cases of a regime every model runs first and right after every other model."""
    rows = williams(len(models))
    row = rows[(k * ncases + ci) % len(rows)]
    seq = [models[i] for i in row]
    return [m for m in seq if present is None or m in present]


def best_steady(rec):
    t = rec.get("timing") or {}
    return t.get("step_s")


# ---- screening
def screen(o):
    """WRF's layouts and CPPWRF CPU's thread counts, history off, screen_min simulated minutes: one warm-up run of
    each model at the first layout (not counted), then screen_reps rounds, each over every layout in a rotated order;
    the layout with the smallest median seconds per step (steady) is chosen."""
    info = cases_info(o)
    best = load_best(o)
    for c in o.cases:
        if not info.get(c, {}).get("ok"):
            continue
        wrfm = [m for m in o.models if MODELS[m][0] == "wrf"]
        if wrfm and not o.layout:
            res = {}
            for m in wrfm:
                run_model(o, c, "screen", m, "screen", block=-1, warm=True, layout=o.layouts[0])
            for rep in range(o.screen_reps):
                lays = o.layouts[rep % len(o.layouts):] + o.layouts[:rep % len(o.layouts)]
                if rep % 2:
                    lays = lays[::-1]
                for lay in lays:
                    for m in (wrfm if (rep + o.layouts.index(lay)) % 2 == 0 else wrfm[::-1]):
                        rec, _ = run_model(o, c, "screen", m, "screen", block=rep, layout=lay)
                        if rec.get("ok") and best_steady(rec):
                            res.setdefault((m, lay), []).append(best_steady(rec))
            for m in wrfm:
                cand = [(statistics.median(v), lay) for (mm, lay), v in res.items() if mm == m]
                if cand:
                    best[f"{c}:{m}"] = min(cand)[1]
                    log(o, f"{c} {m}: fastest layout {best[f'{c}:{m}']} ({min(cand)[0]:.4f} s per step, median of "
                        f"the screened runs)")
        elif wrfm:
            for m in wrfm:
                best[f"{c}:{m}"] = o.layout
        if "cpu_fp64" in o.models:
            res = {}
            ths = sorted({min(topology(), o.np), o.np})
            run_model(o, c, "screen", "cpu_fp64", "screen", block=-1, warm=True, threads=ths[0])
            for rep in range(o.screen_reps):
                for t in (ths if rep % 2 == 0 else ths[::-1]):
                    rec, _ = run_model(o, c, "screen", "cpu_fp64", "screen", block=rep, threads=t)
                    if rec.get("ok") and best_steady(rec):
                        res.setdefault(t, []).append(best_steady(rec))
            if res:
                cand = [(statistics.median(v), t) for t, v in res.items()]
                best[f"{c}:cpu_fp64"] = min(cand)[1]
                log(o, f"{c} cpu_fp64: fastest thread count {min(cand)[1]}")
        json.dump(best, open(o.out + "/best.json", "w"), indent=1)
    return best


def load_best(o):
    p = o.out + "/best.json"
    return json.load(open(p)) if os.path.exists(p) else {}


# ---- the timed blocks
def timed(o):
    """The warm-up (every model of every case once, warm_min simulated minutes, history off, not counted), the
    history-off blocks and the history-on blocks; within a regime the order of block k of case number ci is row
    (k ncases + ci) of a Williams design over the regime's models (run_order)."""
    info = cases_info(o)
    best = load_best(o)
    if any(MODELS[m][0] == "gpu" for m in o.models) and not gpu_ready(o):
        log(o, "the GPU side is not ready after 2 h: no GPU runs (--gpu none)")
        o.gpu = "none"
    cs = [c for c in o.cases if info.get(c, {}).get("ok")]
    on_models = [m for m in o.models if m != "cpu_fp64"] + (["gpu_fp32_nc0"] if o.nc0 and "gpu_fp32" in o.models
                                                            else [])
    plan = [("warm", o.models, 1 if o.warmup else 0, None), ("off", o.models, o.reps, "cpu"), ("on", on_models,
                                                                                            o.reps_on, None)]
    for regime, models, nblocks, limit in plan:
        for k in range(nblocks):
            block = 0 if regime == "warm" else k + 1
            last_on = regime == "on" and k == nblocks - 1
            for ci, c in enumerate(cs):
                present = [m for m in models if not (limit and m == "cpu_fp64" and k >= o.cpu_reps)]
                order = run_order(models, k, ci, len(cs), present)
                prev = None
                for pos, m in enumerate(order):
                    extra = dict(pos=pos, prev=prev, order=order)
                    prev = m
                    if m == "gpu_fp32_nc0":
                        run_model(o, c, "timed", "gpu_fp32", regime, block=block, tag="gpu_fp32_nc0",
                                  env_extra=dict(CPPWRF_NC_THREADS=0), extra=extra)
                        continue
                    kind = MODELS[m][0]
                    lay = best.get(f"{c}:{m}", o.layout or "dm24") if kind == "wrf" else None
                    th = best.get(f"{c}:cpu_fp64") if kind == "cpu" else None
                    keep = last_on and m in ("wrf_r4", "wrf_r4t", "wrf_r8", "gpu_fp32", "gpu_fp32id", "gpu_fp64")
                    rec, out = run_model(o, c, "timed", m, regime, block=block, warm=regime == "warm", layout=lay,
                                         threads=th, keep_out=keep, extra=extra)
                    if keep and rec.get("ok"):
                        keep_output(o, c, m, out)


def keep_output(o, c, name, path):
    """Moves a run's output to ROOT/keep/CASE/NAME (the comparisons' inputs)."""
    d = os.path.join(o.root, "keep", c, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(os.path.dirname(d), exist_ok=True)
    if os.path.isdir(path):
        os.makedirs(d)
        for f in glob.glob(path + "/wrfout_*") + glob.glob(path + "/d0*"):
            shutil.move(f, d)
        shutil.rmtree(path, ignore_errors=True)


# ---- proof runs (untimed, several at a time)
def proof(o):
    info = cases_info(o)
    jobs = []  # (cost estimate, case, name, kind, build, vCPUs)
    for c in o.cases:
        if not info.get(c, {}).get("ok"):
            continue
        for p in o.proof:
            kind, build = PROOF[p]
            est = EST.get(c, {}).get(p, EST.get(c, {}).get("cpu_fp64", 1)) * info[c]["seconds"] / 3600
            jobs.append((est, c, p, kind, build, 1 if kind == "wrf" else o.proof_threads))
    jobs.sort(reverse=True)  # the longest first
    free, running = o.np, []
    lock = threading.Lock()

    def work(c, p, kind, build, ncpu):
        nonlocal free
        try:
            if kind == "wrf":
                rec, out = run_model(o, c, "proof", None, "on", layout="dm1", kind="wrf", build=build, keep_out=True,
                                     pinned=False, tag=p)
            else:
                rec, out = run_model(o, c, "proof", None, "on", threads=ncpu, kind="cpu", build=build, keep_out=True,
                                     pinned=False, tag=p)
            if rec.get("ok"):
                keep_output(o, c, p, out)
        finally:
            with lock:
                free += ncpu

    for est, c, p, kind, build, ncpu in jobs:
        while True:
            with lock:
                if free >= ncpu or not any(t.is_alive() for t in running):
                    free -= ncpu
                    break
            time.sleep(5)
        t = threading.Thread(target=work, args=(c, p, kind, build, ncpu))
        t.start()
        running.append(t)
        log(o, f"proof {c} {p} started ({ncpu} vCPUs, about {est / 60:.0f} min on a G4)")
    for t in running:
        t.join()


# ---- comparisons
def kfiles(d, dom):
    return sorted(glob.glob(f"{d}/wrfout_{dom}_*") + glob.glob(f"{d}/{dom}/wrfout_{dom}_*"))


def sha_list(paths):
    import hashlib
    out = {}
    for p in paths:
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for b in iter(lambda: f.read(1 << 22), b""):
                h.update(b)
        out[os.path.basename(p)] = h.hexdigest()
    return out


def verify(o):
    sys.path.insert(0, o.src + "/tests/verify")
    import exact
    info = cases_info(o)
    exact_pairs = [  # (A, B, what): every variable, frame and attribute; the global attributes but TITLE
        ("gpu_fp64", "cpu_fp64", "CPPWRF GPU FP64 vs CPPWRF CPU FP64"),
        ("cpu_fp64_hwlog", "wrf_r8_dm1", "CPPWRF CPU FP64 (this CPU's libmvec log) vs WRF real*8 in one process "
         "(this runtime)"),
        ("cpu_fp64", "cpu_fp64_hwlog", "CPPWRF CPU FP64 (Intel's libmvec log, the default) vs CPPWRF CPU FP64 (this "
         "CPU's log): the two builds differ in the vector log and log10 only; identical on Intel"),
        ("cpu_fp64", "wrf_r8_dm1", "CPPWRF CPU FP64 (Intel's libmvec log, the default) vs WRF real*8 in one process "
         "(this runtime; identical on Intel, differs on AMD by the vector log, docs/STATUS.md)"),
        ("gpu_fp32id", "cpu_fp32", "CPPWRF GPU FP32 identity vs CPPWRF CPU FP32"),
        ("cpu_fp32", "wrf_r4_dm1", "CPPWRF CPU FP32 vs WRF real*4 in one process (this runtime; informative on AMD, "
         "see PROTOCOL.md)"),
    ]
    acc_pairs = [("gpu_fp32", "gpu_fp64", "fp32-r8"), ("wrf_r4_dm1", "gpu_fp64", "r4-r8"),
                 ("gpu_fp32id", "gpu_fp64", "fp32id-r8"), ("wrf_r4", "gpu_fp64", "r4best-r8"),
                 ("wrf_r4t", "gpu_fp64", "r4tuned-r8"), ("wrf_r8", "wrf_r8_dm1", "r8best-r8dm1")]
    for c in o.cases:
        if not info.get(c, {}).get("ok"):
            continue
        K = os.path.join(o.root, "keep", c)
        nd = len(info[c]["on"]["grid"])
        doms = [f"d{i:02d}" for i in range(1, nd + 1)]
        for a, b, what in exact_pairs:
            if not (os.path.isdir(f"{K}/{a}") and os.path.isdir(f"{K}/{b}")):
                continue
            rec = dict(kind="exact", case=c, a=a, b=b, what=what, domains={})
            for d in doms:
                fa, fb = kfiles(f"{K}/{a}", d), kfiles(f"{K}/{b}", d)
                if not fa or not fb:
                    rec["domains"][d] = dict(error=f"no files ({len(fa)} / {len(fb)})")
                    continue
                try:
                    bad, n = exact.compare_stream(fa, fb, global_atts=True)
                    rec["domains"][d] = dict(frames=n, identical=not bad,
                                             values_identical=not [k for k in bad if k != "(metadata)"],
                                             differ={k: str(v)[:200] for k, v in bad.items()},
                                             sha256_a=sha_list(fa), sha256_b=sha_list(fb))
                except Exception as e:  # noqa: BLE001 - a comparison that fails is a result line
                    rec["domains"][d] = dict(error=f"{type(e).__name__}: {e}")
            emit(o, rec, "verify.jsonl")
            log(o, f"{c} exact {a} vs {b}: " + "; ".join(
                f"{d} " + (v.get("error") or (f"{v['frames']} frames identical" if v["identical"] else
                                             f"{v['frames']} frames, differ {sorted(v['differ'])[:8]}"))
                for d, v in rec["domains"].items()))
        acc = os.path.join(o.out, "accuracy")
        os.makedirs(acc, exist_ok=True)
        for a, b, pair in acc_pairs:
            if not (os.path.isdir(f"{K}/{a}") and os.path.isdir(f"{K}/{b}")):
                continue
            for d in doms:
                fout = f"{acc}/{c}_{d}_{pair}.jsonl"
                if os.path.exists(fout):  # compare appends: a rerun of verify starts the file again
                    os.remove(fout)
                r = subprocess.run([sys.executable, o.src + "/tools/verify/fp32_accuracy.py", "compare", f"{K}/{a}",
                                    f"{K}/{b}", "--domain", d, "--case", c, "--pair", pair, "--out",
                                    f"{acc}/{c}_{d}_{pair}.jsonl"], capture_output=True, text=True)
                emit(o, dict(kind="accuracy", case=c, a=a, b=b, pair=pair, domain=d, exit=r.returncode,
                             file=f"accuracy/{c}_{d}_{pair}.jsonl", err=(r.stderr or "")[-300:]), "verify.jsonl")
        l2(o, c, K, doms)
    st = sorted(glob.glob(os.path.join(o.out, "accuracy", "*.jsonl")))
    if st:
        r = subprocess.run([sys.executable, o.src + "/tools/verify/fp32_accuracy.py", "report", *st, "--md",
                            os.path.join(o.out, "accuracy.md"), "--metric", "rel_l2", "--svg-dir",
                            os.path.join(o.out, "svg_accuracy"), "--svg-rel", "svg_accuracy"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            log(o, f"verify: fp32_accuracy.py report exit {r.returncode} {(r.stderr or r.stdout).strip()[-300:]}")
            raise SystemExit(r.returncode)


def l2(o, c, K, doms):
    """Relative L2 at the last common frame of the models' outputs against the GPU FP64 one (the record's 'l2')."""
    import numpy as np
    sys.path.insert(0, o.src + "/tests/verify")
    import exact
    ref = f"{K}/gpu_fp64"
    if not os.path.isdir(ref):
        return
    for a in sorted(os.listdir(K)):
        if a == "gpu_fp64":
            continue
        rec = dict(kind="l2", case=c, a=a, b="gpu_fp64", domains={})
        for d in doms:
            opened = []
            try:
                fa, fb = exact.frames(kfiles(f"{K}/{a}", d), opened), exact.frames(kfiles(ref, d), opened)
                common = sorted(set(fa) & set(fb))
                if not common:
                    rec["domains"][d] = dict(error="no common frame")
                    continue
                t = common[-1]
                (x, ix), (y, iy) = fa[t], fb[t]
                v = {}
                for k in L2_VARS:
                    if k in x.variables and k in y.variables:
                        p = np.asarray(x[k][ix], dtype=np.float64)
                        q = np.asarray(y[k][iy], dtype=np.float64)
                        n = float(np.sqrt(np.sum(q * q)))
                        v[k] = float(np.sqrt(np.sum((p - q) ** 2))) / n if n > 0 else None
                rec["domains"][d] = dict(time=t, l2=v)
            except Exception as e:  # noqa: BLE001
                rec["domains"][d] = dict(error=f"{type(e).__name__}: {e}")
            finally:
                for ds in opened:
                    ds.close()
        emit(o, rec, "verify.jsonl")


# ---- report
def report(o, rate=None):
    args = [sys.executable, HERE + "/bench_stats.py", o.out + "/raw_runs.jsonl", "--verify", o.out + "/verify.jsonl",
            "--md", o.out + "/report.md", "--json", o.out + "/summary.json", "--csv", o.out + "/runs.csv"]
    for f in ("manifest.json", "cases.json", "best.json"):
        if os.path.exists(os.path.join(o.out, f)):
            args += ["--" + f.split(".")[0], os.path.join(o.out, f)]
    if rate:
        args += ["--rate", str(rate)]
    if os.path.exists(o.out + "/accuracy.md"):
        args += ["--accuracy-md", o.out + "/accuracy.md"]
    r = subprocess.run(args, capture_output=True, text=True)
    log(o, f"report: bench_stats.py exit {r.returncode} {(r.stderr or r.stdout if r.returncode else r.stderr).strip()[-300:]}")
    if r.returncode != 0:
        raise SystemExit(r.returncode)
    r = subprocess.run([sys.executable, HERE + "/bench_plot.py", o.out + "/summary.json", "--out", o.out + "/svg"],
                       capture_output=True, text=True)
    log(o, f"report: bench_plot.py exit {r.returncode} {(r.stderr or r.stdout if r.returncode else r.stderr).strip()[-300:]}")
    if r.returncode != 0:
        raise SystemExit(r.returncode)


# ---- plan
G4_VCPUS = 48
FRAME_S = {  # seconds per history frame with history on (docs/PERF.md: wall on - off over 6 frames; GPU after IO1)
    "C1": dict(wrf_r4=0.6, wrf_r4t=0.6, wrf_r8=1.6, gpu_fp32=0.3, gpu_fp32id=0.3, gpu_fp64=0.6),
    "C2": dict(wrf_r4=5.3, wrf_r4t=5.3, wrf_r8=11.4, gpu_fp32=1.5, gpu_fp32id=1.5, gpu_fp64=3.0),
    "C3": dict(wrf_r4=0.6, wrf_r4t=0.6, wrf_r8=1.6, gpu_fp32=0.3, gpu_fp32id=0.3, gpu_fp64=0.6),
    "C4": dict(wrf_r4=0.9, wrf_r4t=0.9, wrf_r8=1.7, gpu_fp32=0.6, gpu_fp32id=0.6, gpu_fp64=1.2),
}


COOL_S = 30  # the estimate's cool-down after a GPU run that follows a GPU run (the wait ends at idle + 5 C, at most 90 s)


def plan(o):
    """The runs and a session estimate from the G4 times of docs/PERF.md (EST, FRAME_S): a guide, not a promise."""
    tot, lines, proof_long, proof_cpu = 0.0, [], 0.0, 0.0
    nwrf = sum(1 for m in o.models if MODELS[m][0] == "wrf")
    on_models = [m for m in o.models if m != "cpu_fp64"] + (["gpu_fp32_nc0"] if o.nc0 and "gpu_fp32" in o.models
                                                            else [])
    gpu = lambda m: m.startswith("gpu")  # noqa: E731
    cs = [c for c in o.cases if EST.get(c)]
    for c in o.cases:
        base, length, sets, desc = o.case_defs[c]
        h = length_s(length) / 3600
        e = EST.get(c)
        if not e:
            lines.append(f"{c}: {desc}; no estimate")
            continue
        ci = cs.index(c)
        e = dict(e, gpu_fp32_nc0=e["gpu_fp32"])
        hs, hw = min(h, o.screen_min / 60), min(h, o.warm_min / 60)
        run = lambda m, hh: e[m] * hh + e["init"] + (o.pause + 2 if gpu(m) else 0)  # noqa: E731
        lay = 0 if o.layout else len(o.layouts) * o.screen_reps
        n_screen = (lay + (1 if lay else 0)) * nwrf
        s_screen = (lay + (1 if lay else 0)) * sum(1.3 * e[m] * hs + e["init"] for m in o.models if MODELS[m][0] == "wrf")
        if "cpu_fp64" in o.models:
            nt = len({min(G4_VCPUS // 2, o.np), o.np})
            s_screen += (1 + nt * o.screen_reps) * (1.2 * e["cpu_fp64"] * hs + e["init"])
            n_screen += 1 + nt * o.screen_reps
        s_warm = sum(run(m, hw) for m in o.models) if o.warmup else 0
        frames = h * 60 / o.hist_min + 1
        s_off = sum(run(m, h) * (o.cpu_reps if m == "cpu_fp64" else o.reps) for m in o.models)
        s_on = sum((run(m, h) + FRAME_S[c].get("gpu_fp32" if m == "gpu_fp32_nc0" else m, 0) * frames *
                    (3 if m == "gpu_fp32_nc0" else 1)) * o.reps_on for m in on_models)
        cool = 0
        for regime, models, nb in (("off", o.models, o.reps), ("on", on_models, o.reps_on)):
            for k in range(nb):
                pres = [m for m in models if not (regime == "off" and m == "cpu_fp64" and k >= o.cpu_reps)]
                order = run_order(models, k, ci, len(cs), pres)
                cool += sum(1 for x, y in zip(order, order[1:]) if gpu(x) and gpu(y))
        s_cool = cool * max(0, COOL_S - o.pause)
        runs = len(o.models) * (1 if o.warmup else 0) + sum(o.cpu_reps if m == "cpu_fp64" else o.reps for m in o.models)
        runs += o.reps_on * len(on_models)
        for p in o.proof:
            if PROOF[p][0] == "wrf":
                proof_long = max(proof_long, e.get(p, 100) * h)
            else:  # o.proof_threads of the 48 vCPUs: about 0.7 of linear scaling from the 48-thread time
                t = e.get(p, e["cpu_fp64"]) * h * G4_VCPUS / o.proof_threads * 0.7
                proof_cpu += t * o.proof_threads
                proof_long = max(proof_long, t)
        tot += s_screen + s_warm + s_off + s_on + s_cool
        lines.append(f"{c}: {desc}\n    {n_screen} screening runs ({o.screen_min} min, {s_screen / 60:.0f} min), "
                     f"{runs} timed runs: warm-up {s_warm / 60:.0f} min ({o.warm_min} min each), history off "
                     f"{s_off / 60:.0f} min ({o.reps} blocks), history on {s_on / 60:.0f} min ({o.reps_on} blocks), "
                     f"{cool} GPU cool-downs ({s_cool / 60:.0f} min)")
    s_proof = max(proof_long, proof_cpu / (G4_VCPUS - 8))
    setup = 1800  # the b30ec5b session: WRF's four builds 13 min, CUDA builds 1-2 min each, data and WPS
    if "wrf_r4t" in o.models:
        setup += 420  # the tuned dmpar and dm+sm builds
    lines.append(f"G4 estimate: screening, warm-up and timed blocks {tot / 3600:.1f} h, proof runs {s_proof / 3600:.1f} h "
                 f"(after the timed blocks, several at a time on the CPU), set-up and builds {setup / 3600:.1f} h, comparisons and report 0.3 h: "
                 f"{(tot + s_proof + setup) / 3600 + 0.3:.1f} h")
    print("\n".join(lines))


# ---- the large domain
def make_large(argv):
    import netCDF4
    a = dict(zip(argv[::2], argv[1::2]))
    base, met, R, start, end = a["--base"], a["--met"], a["--out"], a["--start"], a["--end"]
    wrf = a.get("--wrf", "/opt/wrfbench")
    shutil.rmtree(R, ignore_errors=True)
    os.makedirs(R)
    for f in os.listdir(base):
        p = os.path.join(base, f)
        if os.path.isdir(p) or re.match(r"(wrfout.*|wrfrst.*|wrfinput.*|wrfbdy.*|wrflowinp.*|rsl\..*|namelist\..*|"
                                        r".*\.log|.*\.exe)$", f):
            continue
        os.symlink(os.path.realpath(p), os.path.join(R, f))
    mets = sorted(glob.glob(met + "/met_em.d01.*"))
    for f in mets:
        os.symlink(f, os.path.join(R, os.path.basename(f)))
    ds = netCDF4.Dataset(mets[0])
    we, sn = ds.getncattr("WEST-EAST_GRID_DIMENSION"), ds.getncattr("SOUTH-NORTH_GRID_DIMENSION")
    nlev = len(ds.dimensions["num_metgrid_levels"])
    nsoil = int(ds.getncattr("NUM_METGRID_SOIL_LEVELS"))
    ds.close()
    nl = open(os.path.join(base, "namelist.input")).read()
    for k, v in (("e_we", we), ("e_sn", sn), ("num_metgrid_levels", nlev), ("num_metgrid_soil_levels", nsoil)):
        nl = setkey(nl, "domains", k, v)
    (sy, smo, sd, sh), (ey, emo, ed, eh) = (re.split(r"[-_]", start), re.split(r"[-_]", end))
    for k, v in (("start_year", sy), ("start_month", smo), ("start_day", sd), ("start_hour", sh), ("end_year", ey),
                 ("end_month", emo), ("end_day", ed), ("end_hour", eh), ("interval_seconds", "21600")):
        nl = setkey(nl, "time_control", k, v)
    for k in ("run_days", "run_hours", "run_minutes", "run_seconds"):
        nl = re.sub(rf"^\s*{k}\s*=.*\n", "", nl, flags=re.M | re.I)
    open(R + "/namelist.input", "w").write(nl)
    os.symlink(wrf + "/WRF-r8/main/real.exe", R + "/real.exe")
    t0 = time.monotonic()
    n = max(1, min(16, topology()))
    rc = subprocess.run(f"ulimit -s unlimited; exec mpiexec -n {n} {bind(n)} ./real.exe > real.log 2>&1", shell=True,
                        cwd=R).returncode
    text = open(R + "/rsl.error.0000", errors="replace").read() if os.path.exists(R + "/rsl.error.0000") else ""
    ok = rc == 0 and "SUCCESS COMPLETE REAL_EM" in text and os.path.exists(R + "/wrfbdy_d01")
    for f in ("namelist.input", "rsl.error.0000", "real.log"):
        if os.path.exists(os.path.join(R, f)):
            shutil.copy(os.path.join(R, f), os.path.join(R, "..", f"{os.path.basename(R)}.real.{f}"))
    for f in glob.glob(R + "/rsl.*") + glob.glob(R + "/met_em.*") + [R + "/real.exe"]:
        os.remove(f)
    print(json.dumps(dict(kind="real", dir=R, ok=ok, wall=time.monotonic() - t0, e_we=int(we), e_sn=int(sn),
                          levels=int(nlev), start=start, end=end)))
    return 0 if ok else 1


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if a else 2)
    if a[0] == "make-large":
        sys.exit(make_large(a[1:]))
    cmd = a[0]
    rate = None
    if "--rate" in a:
        i = a.index("--rate")
        rate = a[i + 1]
        del a[i:i + 2]
    o = Opt(a[1:])
    if cmd == "plan":
        plan(o)
        return
    if cmd not in ("prepare", "screen", "timed", "proof", "verify", "report", "all"):
        print(__doc__)
        sys.exit(2)
    t0 = time.monotonic()
    log(o, f"== {cmd}: cases {','.join(o.cases)}, models {','.join(o.models)}, proof {','.join(o.proof)}, {NP} vCPUs, "
        f"{topology()} cores")
    for step in ("prepare", "screen", "timed", "proof", "verify", "report"):
        if cmd in (step, "all"):
            t1 = time.monotonic()
            if step == "report":
                report(o, rate)
            else:
                globals()[step](o)
            log(o, f"== {step} done in {(time.monotonic() - t1) / 60:.1f} min")
    log(o, f"== {cmd} done in {(time.monotonic() - t0) / 60:.1f} min")


if __name__ == "__main__":
    main()
