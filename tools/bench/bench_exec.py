#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Executes the timed runs of tools/bench/paper_run.py: one run with its clocks and telemetry, or an agent that
runs the jobs another process hands over through a directory (the GPU side of a Colab session: the CUDA builds run
on the runtime's own Ubuntu, the sequencer in the Debian 13 chroot).

  bench_exec.py run JOB.json                 run one job, print its result (JSON)
  bench_exec.py agent QDIR [--bins DIR] [--idle-exit S]
                                             run QDIR/job_<id>.json in name order as they appear, answer each with
                                             QDIR/res_<id>.json (written whole: a temporary file renamed); ends when
                                             QDIR/stop exists and no job is left, or after S seconds without a job
                                             (default 36000). QDIR/agent.json: the agent's start and the
                                             telemetry probe; QDIR/alive: touched every 20 s
  bench_exec.py --help

A job: {"id", "cwd", "cmd" (a bash command line; "{bin:NAME}" stands for the executable of build NAME: DIR/NAME/cppwrf
with --bins DIR, else the job's "bins" map), "env" (added to the environment), "log" (stdout and stderr),
"timeout" (s, default 7200), "telemetry" (true: nvidia-smi samples into "csv" from "pre_idle" s before the start,
default 1.5, to 0.3 s after the end), "pause" (s to sleep before the run, default 0: lets the GPU cool down)}.
The result: the job's fields and exit (124: timeout), wall (time.monotonic() around the process), t0 / t1 (epoch),
load (the processes running just before, this one aside: /proc/stat procs_running, read before the sampler
starts), cpu_mhz (the mean "cpu MHz" of /proc/cpuinfo just before), rapl_j (the package energy between start and
end when a RAPL counter is readable, else null) and gpu (tools/bench/telemetry.py's summary of the window, or null).
"""
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry  # noqa: E402


def procs_running():
    try:
        return max(int(x.split()[1]) for x in open("/proc/stat") if x.startswith("procs_running")) - 1
    except (OSError, ValueError):
        return None


def cpu_mhz():
    try:
        v = [float(x.split(":")[1]) for x in open("/proc/cpuinfo") if x.startswith("cpu MHz")]
        return round(sum(v) / len(v), 1) if v else None
    except (OSError, ValueError):
        return None


def resolve(cmd, bins):
    def sub(m):
        n = m.group(1)
        if isinstance(bins, str):
            return os.path.join(bins, n, "cppwrf")
        if n not in bins:
            raise KeyError(f"no build {n}")
        return bins[n]
    return re.sub(r"\{bin:([A-Za-z0-9_.-]+)\}", sub, cmd)


def execute(job, bins=None):
    """Runs one job (see the module's doc); returns the result dict."""
    res = dict(job)
    pause = float(job.get("pause", 0) or 0)
    if pause > 0:
        time.sleep(pause)
    try:
        cmd = resolve(job["cmd"], bins if bins is not None else job.get("bins", {}))
    except KeyError as e:
        res.update(exit=127, wall=0.0, error=str(e))
        return res
    res["cmd_resolved"] = cmd
    env = dict(os.environ)
    env.update({k: str(v) for k, v in (job.get("env") or {}).items()})
    res["load"], res["cpu_mhz"] = procs_running(), cpu_mhz()  # before the sampler starts
    sampler = None
    if job.get("telemetry"):
        sampler = telemetry.Sampler(job.get("csv") or os.path.join(job["cwd"], f"{job['id']}.gpu.csv"))
        if sampler.start():
            time.sleep(float(job.get("pre_idle", 1.5)))
        else:
            sampler = None
    r0 = telemetry.rapl_read()
    t0, m0 = time.time(), time.monotonic()
    try:
        with open(job["log"], "w") as f:
            rc = subprocess.run(["bash", "-c", cmd], cwd=job["cwd"], env=env, stdout=f, stderr=subprocess.STDOUT,
                                timeout=float(job.get("timeout", 7200))).returncode
    except subprocess.TimeoutExpired:
        rc = 124
    wall, t1 = time.monotonic() - m0, time.time()
    res["rapl_j"] = telemetry.rapl_delta_j(r0, telemetry.rapl_read())
    res.update(exit=rc, wall=wall, t0=t0, t1=t1)
    if sampler:
        time.sleep(0.3)
        sampler.stop()
        res["gpu"] = telemetry.summarize(sampler.csv, t0, t1, float(job.get("pre_idle", 1.5)))
    else:
        res["gpu"] = None
    return res


def write_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def agent(q, bins=None, idle_exit=36000.0):
    import threading
    os.makedirs(q, exist_ok=True)
    write_atomic(os.path.join(q, "agent.json"), dict(start=time.time(), pid=os.getpid(), probe=telemetry.probe()))

    def beat():  # QDIR/alive: the sequencer stops waiting for an agent that died
        while True:
            open(os.path.join(q, "alive"), "w").write(f"{time.time()}\n")
            time.sleep(20)
    threading.Thread(target=beat, daemon=True).start()
    last = time.monotonic()
    while True:
        jobs = sorted(f for f in os.listdir(q) if f.startswith("job_") and f.endswith(".json"))
        if not jobs:
            if os.path.exists(os.path.join(q, "stop")) or time.monotonic() - last > idle_exit:
                return 0
            time.sleep(0.2)
            continue
        p = os.path.join(q, jobs[0])
        try:
            job = json.load(open(p))
        except ValueError:  # not written whole yet
            time.sleep(0.2)
            continue
        os.remove(p)
        try:
            res = execute(job, bins)
        except Exception as e:  # noqa: BLE001 - an agent that dies leaves the sequencer waiting
            res = dict(job, exit=125, wall=0.0, error=f"{type(e).__name__}: {e}")
        write_atomic(os.path.join(q, f"res_{job['id']}.json"), res)
        last = time.monotonic()


def submit(q, job, timeout):
    """The sequencer's side: hand JOB to the agent of QDIR and wait for its result (None after timeout s, or when
    QDIR/alive, which the agent touches every 20 s, is older than 2 minutes)."""
    rp = os.path.join(q, f"res_{job['id']}.json")
    if os.path.exists(rp):
        os.remove(rp)
    write_atomic(os.path.join(q, f"job_{job['id']}.json"), job)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            if time.time() - os.path.getmtime(os.path.join(q, "alive")) > 120:
                return None  # the agent stopped beating
        except OSError:
            if time.monotonic() - t0 > 120:
                return None
        if os.path.exists(rp):
            try:
                r = json.load(open(rp))
                os.remove(rp)
                return r
            except ValueError:
                pass
        time.sleep(0.2)
    return None


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if a else 2)
    if a[0] == "run" and len(a) == 2:
        print(json.dumps(execute(json.load(open(a[1]))), indent=1))
    elif a[0] == "agent" and len(a) >= 2:
        bins = a[a.index("--bins") + 1] if "--bins" in a else None
        ie = float(a[a.index("--idle-exit") + 1]) if "--idle-exit" in a else 36000.0
        sys.exit(agent(a[1], bins, ie))
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
