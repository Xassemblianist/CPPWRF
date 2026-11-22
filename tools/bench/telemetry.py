#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""GPU power / clock sampling with nvidia-smi, energy by integration, and the CPU energy counter probe.

  telemetry.py probe                         JSON: nvidia-smi's GPU, the query fields it accepts, whether a RAPL /
                                             powercap counter is readable (/sys/class/powercap) and perf is present
  telemetry.py sample CSV [--interval MS] -- CMD ...
                                             run CMD while nvidia-smi samples into CSV; prints the summary of the
                                             run's window (as `summarize`), the command's exit status and wall time
  telemetry.py summarize CSV --t0 EPOCH --t1 EPOCH [--idle-before S]
                                             the summary of the samples of CSV in [t0, t1]
  telemetry.py --help

Samples: nvidia-smi --query-gpu=timestamp,power.draw,clocks.sm,clocks.mem,temperature.gpu,utilization.gpu,
memory.used,pstate,clocks_event_reasons.active (fields a driver does not know are left out: older drivers call the
last one clocks_throttle_reasons.active) every --interval ms (default 100), one CSV line each (no header, no units).
nvidia-smi stamps each sample with the host's clock, so a run's window is given in epoch seconds.

Summary of a window [t0, t1]: energy_j (the trapezoidal integral of power.draw over the window, the power
interpolated linearly between samples and at the window's ends), power_mean_w (energy / duration), power_peak_w,
power_idle_w (the median of the samples in the idle_before seconds before t0, default 1.5 s: the sampler starts
before the run), sm_clock_median_mhz, sm_clock_min_mhz, mem_clock_median_mhz, temp_max_c, util_median_pct,
mem_used_peak_mib, mem_used_idle_mib, samples, throttled_samples (samples whose clock event reasons include a
hardware slowdown 0x8, a software thermal slowdown 0x20, a hardware thermal slowdown 0x40 or a power brake 0x80),
power_cap_samples (the software power cap 0x4: normal at full load, counted on its own; 0x1 idle, 0x2 application
clocks, 0x10 sync boost and 0x100 display clock are not throttling), reasons_seen (the union of the reason bits),
head_gap_s / tail_gap_s (window start to the first sample, last sample to the window end) and, with a steady
window [ta, tb] (the run's steady steps), energy_steady_j and power_steady_w over it. nvidia-smi's power.draw is the board's
power: not the host CPU, memory, disks or the data centre's overhead. A field the GPU does not report ([N/A]) gives
null.
"""
import bisect
import datetime
import glob
import json
import os
import shutil
import statistics
import subprocess
import sys
import time

FIELDS = ["timestamp", "power.draw", "clocks.sm", "clocks.mem", "temperature.gpu", "utilization.gpu", "memory.used",
          "pstate", "clocks_event_reasons.active"]
ALT = {"clocks_event_reasons.active": "clocks_throttle_reasons.active"}
# NVML clock event reasons (nvml.h nvmlClocksEventReason*, nvidia-smi -q "Clocks Event Reasons"): 0x1 GpuIdle,
# 0x2 ApplicationsClocksSetting, 0x4 SwPowerCap, 0x8 HwSlowdown, 0x10 SyncBoost, 0x20 SwThermalSlowdown,
# 0x40 HwThermalSlowdown, 0x80 HwPowerBrakeSlowdown, 0x100 DisplayClockSetting. A sample is "throttled" when a
# thermal or hardware slowdown is active; the power cap (normal at full load) is counted on its own.
THROTTLE_BITS = 0x8 | 0x20 | 0x40 | 0x80
POWER_CAP_BIT = 0x4
_FIELDS = None


def _ok(fields):
    return subprocess.run(["nvidia-smi", "--query-gpu=" + ",".join(fields), "--format=csv,noheader,nounits"],
                          capture_output=True, text=True, timeout=60).returncode == 0


def query_fields():
    """The fields of FIELDS this nvidia-smi accepts (in order), or [] without nvidia-smi; probed once per process
    (the probes are nvidia-smi calls, which must not run in a run's idle window)."""
    global _FIELDS
    if _FIELDS is not None:
        return list(_FIELDS)
    if not shutil.which("nvidia-smi"):
        _FIELDS = []
        return []
    out = []
    for f in FIELDS:
        if _ok(out + [f]):
            out.append(f)
        elif f in ALT and _ok(out + [ALT[f]]):
            out.append(ALT[f])
    _FIELDS = list(out)
    return out


def temperature():
    """The GPU's temperature (C) now, or None."""
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=30)
        return float(q.stdout.split()[0]) if q.returncode == 0 and q.stdout.split() else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def rapl_probe():
    """Readable powercap energy counters (RAPL) and whether perf exists: virtual machines usually hide both."""
    readable = []
    for p in sorted(glob.glob("/sys/class/powercap/*/energy_uj")):
        try:
            int(open(p).read())
            readable.append(p)
        except (OSError, ValueError):
            pass
    return dict(powercap_readable=readable, powercap_dirs=len(glob.glob("/sys/class/powercap/*")),
                perf=bool(shutil.which("perf")))


def rapl_read():
    """{counter: (microjoules, max range)} of the top-level package counters, or {} when none is readable."""
    out = {}
    for p in sorted(glob.glob("/sys/class/powercap/intel-rapl:*/energy_uj")):
        d = os.path.dirname(p)
        if os.path.basename(d).count(":") != 1:
            continue
        try:
            rng = int(open(d + "/max_energy_range_uj").read()) if os.path.exists(d + "/max_energy_range_uj") else 0
            out[d] = (int(open(p).read()), rng)
        except (OSError, ValueError):
            pass
    return out


def rapl_delta_j(a, b):
    """Joules between two rapl_read() results (counter wrap-around handled), or None."""
    if not a or not b:
        return None
    tot = 0
    for k, (e0, rng) in a.items():
        if k not in b:
            return None
        e1 = b[k][0]
        tot += e1 - e0 if e1 >= e0 else e1 + rng - e0
    return tot / 1e6


class Sampler:
    """nvidia-smi -lms INTERVAL into a CSV file while a run goes on."""

    def __init__(self, csv, interval_ms=100, fields=None):
        self.csv, self.interval = csv, interval_ms
        self.fields = fields if fields is not None else query_fields()
        self.p = None

    def start(self):
        if not self.fields:
            return False
        self.f = open(self.csv, "w")
        self.f.write("# " + ",".join(self.fields) + "\n")
        self.f.flush()
        # stdbuf -oL: one line per sample reaches the file at once (a block-buffered stdout loses its tail when the
        # sampler is terminated)
        pre = ["stdbuf", "-oL"] if shutil.which("stdbuf") else []
        self.p = subprocess.Popen(pre + ["nvidia-smi", "--query-gpu=" + ",".join(self.fields),
                                         "--format=csv,noheader,nounits", "-lms", str(self.interval)],
                                  stdout=self.f, stderr=subprocess.DEVNULL)
        return True

    def stop(self):
        if self.p:
            self.p.terminate()
            try:
                self.p.wait(10)
            except subprocess.TimeoutExpired:
                self.p.kill()
            self.f.close()
            self.p = None


def _num(x):
    x = x.strip()
    if not x or x.startswith("[") or x in ("N/A", "Not Supported"):
        return None
    try:
        return int(x, 16) if x.startswith("0x") else float(x)
    except ValueError:
        return None


def read_samples(csv):
    """[(epoch, {field: value})] of a sampler's CSV (its first line names the fields)."""
    lines = open(csv, errors="replace").read().splitlines()
    if not lines or not lines[0].startswith("# "):
        return []
    fields = lines[0][2:].split(",")
    out = []
    for line in lines[1:]:
        parts = [x.strip() for x in line.split(",")]
        if len(parts) != len(fields):
            continue
        rec = dict(zip(fields, parts))
        try:
            t = datetime.datetime.strptime(rec["timestamp"], "%Y/%m/%d %H:%M:%S.%f").timestamp()
        except (KeyError, ValueError):
            continue
        out.append((t, {k: (v if k in ("timestamp", "pstate") else _num(v)) for k, v in rec.items()}))
    return out


def _integrate(pts, t0, t1):
    """Integral of the piecewise linear function through pts [(t, p)] over [t0, t1] (constant beyond its ends)."""
    if not pts:
        return None
    if t1 <= t0:
        return 0.0
    pts = sorted(pts, key=lambda x: x[0])
    if len(pts) == 1:
        return pts[0][1] * (t1 - t0)
    ts = [t for t, _ in pts]

    def at(t):
        if t <= ts[0]:
            return pts[0][1]
        if t >= ts[-1]:
            return pts[-1][1]
        i = bisect.bisect_right(ts, t)
        ta, pa = pts[i - 1]
        tb, pb = pts[i]
        return pa + (pb - pa) * (t - ta) / (tb - ta) if tb > ta else pa

    i0 = bisect.bisect_right(ts, t0)
    i1 = bisect.bisect_left(ts, t1)
    seg = [(t0, at(t0))] + pts[i0:i1] + [(t1, at(t1))]
    return sum((pa + pb) * 0.5 * (tb - ta) for (ta, pa), (tb, pb) in zip(seg, seg[1:]))


def summarize(csv, t0, t1, idle_before=1.5, steady=None):
    """steady: (ta, tb), the epoch window of the steady steps (bench_exec.py's line stamps): energy_steady_j and
    power_steady_w over it."""
    s = read_samples(csv) if os.path.exists(csv) else []
    win = [(t, r) for t, r in s if t0 <= t <= t1]
    pre = [(t, r) for t, r in s if t0 - idle_before <= t < t0]
    reason_key = next((k for k in ("clocks_event_reasons.active", "clocks_throttle_reasons.active")
                       if s and k in s[0][1]), None)

    def col(rows, k):
        return [r[k] for _, r in rows if r.get(k) is not None]

    pw = [(t, r["power.draw"]) for t, r in s if r.get("power.draw") is not None and t0 - 2 <= t <= t1 + 2]
    e = _integrate(pw, t0, t1) if pw else None
    med = lambda v: statistics.median(v) if v else None  # noqa: E731
    reasons = col(win, reason_key) if reason_key else []
    seen = 0
    for x in reasons:
        seen |= int(x)
    out = dict(samples=len(win), duration_s=t1 - t0, energy_j=e,
               power_mean_w=(e / (t1 - t0) if e is not None and t1 > t0 else None),
               power_peak_w=max(col(win, "power.draw"), default=None), power_idle_w=med(col(pre, "power.draw")),
               sm_clock_median_mhz=med(col(win, "clocks.sm")), sm_clock_min_mhz=min(col(win, "clocks.sm"), default=None),
               mem_clock_median_mhz=med(col(win, "clocks.mem")), temp_max_c=max(col(win, "temperature.gpu"), default=None),
               util_median_pct=med(col(win, "utilization.gpu")),
               mem_used_peak_mib=max(col(win, "memory.used"), default=None),
               mem_used_idle_mib=med(col(pre, "memory.used")),
               throttled_samples=sum(1 for x in reasons if int(x) & THROTTLE_BITS) if reason_key else None,
               power_cap_samples=sum(1 for x in reasons if int(x) & POWER_CAP_BIT) if reason_key else None,
               reasons_seen=hex(seen) if reason_key else None,
               # the sampler's coverage: a gap at either end is filled with the nearest sample's power
               head_gap_s=(win[0][0] - t0) if win else None, tail_gap_s=(t1 - win[-1][0]) if win else None)
    if steady and pw and steady[1] > steady[0]:
        ta, tb = steady
        es = _integrate(pw, ta, tb)
        out.update(steady_window_s=tb - ta, energy_steady_j=es, power_steady_w=es / (tb - ta) if es is not None else None)
    return out


def probe():
    gpu = None
    if shutil.which("nvidia-smi"):
        q = subprocess.run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total,compute_cap",
                            "--format=csv,noheader"], capture_output=True, text=True, timeout=60)
        gpu = q.stdout.strip() if q.returncode == 0 else None
    return dict(gpu=gpu, fields=query_fields(), rapl=rapl_probe())


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if a else 2)
    if a[0] == "probe":
        print(json.dumps(probe(), indent=1))
    elif a[0] == "summarize":
        t0, t1 = float(a[a.index("--t0") + 1]), float(a[a.index("--t1") + 1])
        ib = float(a[a.index("--idle-before") + 1]) if "--idle-before" in a else 1.5
        print(json.dumps(summarize(a[1], t0, t1, ib), indent=1))
    elif a[0] == "sample" and "--" in a:
        csv = a[1]
        iv = int(a[a.index("--interval") + 1]) if "--interval" in a[:a.index("--")] else 100
        cmd = a[a.index("--") + 1:]
        s = Sampler(csv, iv)
        s.start()
        time.sleep(1.5)
        t0, m0 = time.time(), time.monotonic()
        rc = subprocess.run(cmd).returncode
        wall, t1 = time.monotonic() - m0, time.time()
        time.sleep(0.3)
        s.stop()
        print(json.dumps(dict(exit=rc, wall=wall, **summarize(csv, t0, t1)), indent=1))
        sys.exit(rc)
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
