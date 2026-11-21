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
default 1.5, to 0.3 s after the end), "pause" (s to wait before the run, default 0), "cool_max" / "cool_delta" /
"cool_base" (with telemetry: before the run wait at least "pause" s and until the GPU is within cool_delta C,
default 5, of its idle temperature cool_base, default the agent's measurement at its start, at most cool_max s),
"stamp" (a regular expression: the log goes through a pipe and the epoch times of the first and the last matching
line are kept, the steady window of the GPU energy)}.
The result: the job's fields and exit (124: timeout), wall (time.monotonic() around the process), t0 / t1 (epoch;
clock_step_s = (t1 - t0) - wall, nonzero when the system clock stepped), load (the processes running just before,
this one aside: /proc/stat procs_running, read before the sampler starts), cpu_mhz (the mean "cpu MHz" of
/proc/cpuinfo just before), cpu_mhz_run (its mean / min / max once a second during the run), cpu_share (steal,
iowait and busy percentages of the machine's CPU time during the run, /proc/stat), cool (the cool-down: seconds,
temperatures), stamps, rapl_j (the package energy between start and end when a RAPL counter is readable, else null)
and gpu (tools/bench/telemetry.py's summary of the window, with the steady window's energy, or null).
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


def cpu_times():
    """The aggregate cpu line of /proc/stat: {user, nice, system, idle, iowait, irq, softirq, steal} in jiffies."""
    try:
        for x in open("/proc/stat"):
            if x.startswith("cpu "):
                v = [int(y) for y in x.split()[1:9]]
                return dict(zip(("user", "nice", "system", "idle", "iowait", "irq", "softirq", "steal"), v))
    except (OSError, ValueError):
        pass
    return None


def cpu_share(a, b):
    """Percentages of the CPU time between two cpu_times(): steal (the hypervisor ran something else), iowait, busy."""
    if not a or not b:
        return None
    d = {k: b[k] - a[k] for k in a}
    tot = sum(d.values())
    if tot <= 0:
        return None
    return dict(steal_pct=100.0 * d["steal"] / tot, iowait_pct=100.0 * d["iowait"] / tot,
                busy_pct=100.0 * (tot - d["idle"] - d["iowait"] - d["steal"]) / tot)


class MhzSampler:
    """The mean "cpu MHz" of /proc/cpuinfo once a second during a run (a VM may report a fixed value)."""

    def __init__(self, every=1.0):
        import threading
        self.v, self.every, self.stop_ev = [], every, threading.Event()
        self.t = threading.Thread(target=self.loop, daemon=True)

    def loop(self):
        while not self.stop_ev.wait(self.every):
            x = cpu_mhz()
            if x:
                self.v.append(x)

    def start(self):
        self.t.start()
        return self

    def stop(self):
        self.stop_ev.set()
        self.t.join(5)
        v = self.v
        return dict(n=len(v), mean=round(sum(v) / len(v), 1), min=min(v), max=max(v)) if v else None


_COOL_BASE = []


def cool_down(job):
    """Waits until the GPU is within job["cool_delta"] C of its idle temperature (measured at the first GPU job, or
    job["cool_base"]), at least job["pause"] s and at most job["cool_max"] s. Returns (seconds, temperature at the
    start of the wait, at its end, the target)."""
    t0 = time.monotonic()
    pause = float(job.get("pause", 0) or 0)
    if not job.get("telemetry") or job.get("cool_max") is None:
        if pause > 0:
            time.sleep(pause)
        return None
    if job.get("cool_base") is not None:
        base = float(job["cool_base"])
    else:
        if not _COOL_BASE:
            _COOL_BASE.append(telemetry.temperature())
        base = _COOL_BASE[0]
    first = telemetry.temperature()
    target = base + float(job.get("cool_delta", 5)) if base is not None else None
    cur = first
    while True:
        el = time.monotonic() - t0
        if el >= pause and (target is None or cur is None or cur <= target):
            break
        if el >= float(job["cool_max"]):
            break
        time.sleep(1.0)
        cur = telemetry.temperature()
    return dict(wait_s=round(time.monotonic() - t0, 2), temp_start_c=first, temp_end_c=cur, target_c=target)


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
    res["cool"] = cool_down(job)
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
    r0, c0 = telemetry.rapl_read(), cpu_times()
    mhz = MhzSampler().start()
    stamp = re.compile(job["stamp"]) if job.get("stamp") else None
    stamps = []  # epoch times of the log lines matching job["stamp"] (the first and the last are kept)
    t0, m0 = time.time(), time.monotonic()
    try:
        with open(job["log"], "w") as f:
            if stamp is None:
                rc = subprocess.run(["bash", "-c", cmd], cwd=job["cwd"], env=env, stdout=f, stderr=subprocess.STDOUT,
                                    timeout=float(job.get("timeout", 7200))).returncode
            else:  # the log through a pipe, each line stamped as it arrives (CPPWRF flushes every timing line)
                p = subprocess.Popen(["bash", "-c", cmd], cwd=job["cwd"], env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, errors="replace", bufsize=1)
                import threading
                timer = threading.Timer(float(job.get("timeout", 7200)), p.kill)
                timer.start()
                for line in p.stdout:
                    f.write(line)
                    if stamp.search(line):
                        now = t0 + (time.monotonic() - m0)
                        if len(stamps) < 2:
                            stamps.append(now)
                        else:
                            stamps[1] = now
                        res["stamp_count"] = res.get("stamp_count", 0) + 1
                rc = p.wait()
                timer.cancel()
                if rc == -9:
                    rc = 124
    except subprocess.TimeoutExpired:
        rc = 124
    wall = time.monotonic() - m0
    t1 = time.time()
    res["rapl_j"] = telemetry.rapl_delta_j(r0, telemetry.rapl_read())
    res["cpu_share"], res["cpu_mhz_run"] = cpu_share(c0, cpu_times()), mhz.stop()
    # the epoch window must match the monotonic wall: a clock step (NTP) during the run is flagged
    res.update(exit=rc, wall=wall, t0=t0, t1=t1, clock_step_s=round((t1 - t0) - wall, 3))
    if stamps:
        res["stamps"] = stamps  # derived from the monotonic clock: [first, last] matching line
    if sampler:
        time.sleep(0.3)
        sampler.stop()
        steady = (stamps[0], stamps[1]) if len(stamps) == 2 else None
        res["gpu"] = telemetry.summarize(sampler.csv, t0, t0 + wall, float(job.get("pre_idle", 1.5)), steady)
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
    probe = telemetry.probe()  # also caches the sampler's fields: no probes before each run
    temps = [x for x in (telemetry.temperature() for _ in range(5)) if x is not None]
    if temps:  # the idle temperature the cool-down waits for
        _COOL_BASE.append(sorted(temps)[len(temps) // 2])
    write_atomic(os.path.join(q, "agent.json"), dict(start=time.time(), pid=os.getpid(), probe=probe,
                                                     idle_temp_c=_COOL_BASE[0] if _COOL_BASE else None))

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
