#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Kaggle worker of tools/kaggle/kaggle_perf.sh: one performance session on two Tesla T4.

  kg_perf_body.py JOB OUT          (the kernel wrapper of kaggle_proof.py runs it; JOB = perf-job.json)
  kg_perf_body.py report DIR       results.json and results.md of DIR (records.jsonl, perf-job.json)

The job (written by kaggle_perf.py) names the source archive's commit, the builds (CUDA with CPPWRF_MPI, sm_75), the
data tars with the input files to extract, and the runs in their order. The worker builds, fetches the inputs while it
builds, probes the node (M8: topology, P2P, copy bandwidths, Open MPI's CUDA support, nsys, cuobjdump resource usage
and the byte loads of the RRTMG kernels), then runs every planned run one at a time: a pause and a cool-down before
each, nvidia-smi sampling of each GPU (tools/bench/telemetry.py), the log parsed by tools/bench/bench_parse.py and by
the parsers below (CPPWRF_HALO_STATS, CPPWRF_GRAPHS=stats, CPPWRF_KTIME, CPPWRF_MEMINFO, CPPWRF_IO_TIMING). Runs whose
estimate does not fit the time left are recorded as skipped. One JSON line per run goes to OUT/records.jsonl at once;
OUT/results.json and OUT/results.md (medians, ranges, paired ratios per round) are written at the end and can be
rebuilt from the records (`report`). Nothing here compares model output: the bits are the suite's and the proof's job.
"""

import gzip
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

MPIRUN = ["mpirun", "--allow-run-as-root", "--oversubscribe", "--bind-to", "none"]

# ---------------------------------------------------------------------------------------------------------------
# log parsers (pure)
RE_HALO = re.compile(r"cppwrf halo: worker (\d+): exchanges (\d+): ([0-9.]+) s in all: pack ([0-9.]+) s, wait for the "
                     r"stream ([0-9.]+) s, transfer ([0-9.]+) s, unpack ([0-9.]+) s")
RE_HALO_OVL = re.compile(r"cppwrf halo: worker (\d+): (\d+) exchanges overlapped by launches \(([0-9.]+) s")
RE_HALO_PLAN = re.compile(r"cppwrf halo: worker (\d+):\s+(\d+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+([0-9.]+)\s+"
                          r"([0-9.]+)\s+(.*)$")
RE_HALO_COLL = re.compile(r"cppwrf halo: worker (\d+): collective kind (\d+): (\d+) calls, ([0-9.]+) ms")
RE_GRAPHS = re.compile(r"cppwrf graphs: (\d+) launches, (\d+) updates, (\d+) instantiations, ([0-9.]+) nodes per launch")
RE_KTIME_TOT = re.compile(r"cppwrf ktime: ([0-9.]+) s in kernels")
RE_KTIME_ROW = re.compile(r"^\s*([0-9.]+)%\s+([0-9.]+) ms\s+(\d+)\s\s(.+)$")
RE_MEM = re.compile(r"cppwrf: device memory (.*?): ([0-9.]+) MiB used of ([0-9.]+) MiB; stack limit (\d+) bytes x (\d+) "
                    r"resident threads \(([0-9.]+) MiB\)")
RE_IO = re.compile(r"cppwrf: io (\S+) rec (\d+): values ([0-9.]+) s \(device copies ([0-9.]+) s\), staging ([0-9.]+) s, "
                   r"queued ([0-9.]+) s, puts ([0-9.]+) s .*?sync ([0-9.]+) s, total ([0-9.]+) s")


def parse_halo(text):
    """CPPWRF_HALO_STATS (stderr of every rank): per worker the totals, the overlapped count, the plans (count, ms,
    pack/wait/xfer/unpack us each, label) and the other collectives."""
    workers = {}

    def w(i):
        return workers.setdefault(i, {"worker": int(i), "plans": [], "collectives": []})

    for line in text.splitlines():
        if m := RE_HALO.search(line):
            w(m[1]).update(exchanges=int(m[2]), total_s=float(m[3]), pack_s=float(m[4]), wait_s=float(m[5]),
                           xfer_s=float(m[6]), unpack_s=float(m[7]))
        elif m := RE_HALO_OVL.search(line):
            w(m[1]).update(overlapped=int(m[2]), between_s=float(m[3]))
        elif m := RE_HALO_COLL.search(line):
            w(m[1])["collectives"].append({"kind": int(m[2]), "calls": int(m[3]), "ms": float(m[4])})
        elif (m := RE_HALO_PLAN.search(line)) and "count" not in line:
            w(m[1])["plans"].append({"count": int(m[2]), "ms": float(m[3]), "pack_us": float(m[4]),
                                     "wait_us": float(m[5]), "xfer_us": float(m[6]), "unpack_us": float(m[7]),
                                     "plan": m[8].strip()})
    for v in workers.values():
        v["plans"].sort(key=lambda p: -p["ms"])
        v["plans"] = v["plans"][:12]
    return [workers[k] for k in sorted(workers, key=int)]


def parse_graphs(text):
    return [{"launches": int(m[1]), "updates": int(m[2]), "instantiations": int(m[3]), "nodes_per_launch": float(m[4])}
            for m in RE_GRAPHS.finditer(text)]


def parse_ktime(text, top=25):
    """CPPWRF_KTIME's table: the total and the largest kernels (percent, ms, launches, name)."""
    total = [float(m[1]) for m in RE_KTIME_TOT.finditer(text)]
    rows = []
    seen = False
    for line in text.splitlines():
        if RE_KTIME_TOT.search(line):
            seen = True
            continue
        if seen and (m := RE_KTIME_ROW.match(line)):
            rows.append({"pct": float(m[1]), "ms": float(m[2]), "launches": int(m[3]), "kernel": m[4].strip()})
    rows.sort(key=lambda r: -r["ms"])
    return {"total_s": sum(total), "tables": len(total), "rows": rows[:top], "kernels": len(rows)} if total else None


def parse_meminfo(text):
    return [{"when": m[1], "used_mib": float(m[2]), "total_mib": float(m[3]), "stack_bytes": int(m[4]),
             "threads": int(m[5]), "stack_mib": float(m[6])} for m in RE_MEM.finditer(text)]


def parse_io(text):
    recs = [m for m in RE_IO.finditer(text)]
    if not recs:
        return None
    keys = ("values", "device_copies", "staging", "queued", "puts", "sync", "total")
    out = {"records": len(recs)}
    for i, k in enumerate(keys):
        out[k + "_s"] = round(sum(float(m[3 + i]) for m in recs), 6)
    return out


def parse_log(text, wall, overlap):
    import bench_parse  # tools/bench/bench_parse.py, bundled next to this file

    return {"timing": bench_parse.parse(text, "cppwrf", wall, overlap), "halo": parse_halo(text),
            "graphs": parse_graphs(text), "ktime": parse_ktime(text), "meminfo": parse_meminfo(text),
            "io": parse_io(text),
            "devices": sorted({ln.strip() for ln in text.splitlines()
                               if "device of each rank" in ln or "MPI ranks as" in ln})[:8],
            "errors": [ln.strip()[:300] for ln in text.splitlines()
                       if ln.startswith("cppwrf: error") or ln.startswith("cppwrf: rank") or "FATAL" in ln][-6:]}


# ---------------------------------------------------------------------------------------------------------------
# aggregation and report (pure)
def _stats(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    med = statistics.median(v)
    return {"n": len(v), "median": med, "min": v[0], "max": v[-1],
            "spread_pct": (100 * (v[-1] - v[0]) / med) if med else None}


def metric(rec, name):
    t = (rec.get("parsed") or {}).get("timing") or {}
    if name == "wall":
        return rec.get("wall")
    return t.get(name)


def aggregate(job, records):
    timed = [r for r in records if r.get("kind") == "timed" and r.get("exit") == 0]
    groups = {}
    for r in timed:
        g = groups.setdefault((r["group"], r["variant"]), [])
        g.append(r)
    rows = []
    for (group, variant), rs in sorted(groups.items()):
        tel = [x for r in rs for x in (r.get("telemetry") or {}).values() if x and x.get("samples")]
        busy = [x for x in tel if (x.get("util_median_pct") or 0) > 0]
        rows.append({"group": group, "variant": variant, "measures": rs[0].get("measures", []),
                     "step_s": _stats(metric(r, "step_s") for r in rs), "wall": _stats(r.get("wall") for r in rs),
                     "steady_steps": rs[0]["parsed"]["timing"].get("steady_steps") if rs[0].get("parsed") else None,
                     "sm_clock_mhz": _stats(x.get("sm_clock_median_mhz") for x in busy),
                     "power_w": _stats(x.get("power_mean_w") for x in busy),
                     "temp_max_c": max((x.get("temp_max_c") or 0 for x in tel), default=None),
                     "throttled_samples": sum(x.get("throttled_samples") or 0 for x in tel),
                     "power_cap_samples": sum(x.get("power_cap_samples") or 0 for x in tel)})
    comps = []
    for c in job.get("comparisons", []):
        by_rep = {}
        for r in timed:
            if r["group"] == c["group"] and r["variant"] in (c["base"], c["new"]):
                by_rep.setdefault(r["rep"], {})[r["variant"]] = metric(r, c["metric"])
        paired = [d[c["base"]] / d[c["new"]] for d in by_rep.values()
                  if d.get(c["base"]) and d.get(c["new"])]
        b = _stats(metric(r, c["metric"]) for r in timed if r["group"] == c["group"] and r["variant"] == c["base"])
        n = _stats(metric(r, c["metric"]) for r in timed if r["group"] == c["group"] and r["variant"] == c["new"])
        ratio = _stats(paired)
        verdict = "no data"
        if ratio and b and n:
            overlap = not (b["max"] < n["min"] or n["max"] < b["min"])
            same_sign = all(p > 1 for p in paired) or all(p < 1 for p in paired)
            verdict = ("within noise" if overlap or not same_sign or len(paired) < 2 else
                       ("faster" if ratio["median"] > 1 else "slower"))
        comps.append(dict(c, base_stats=b, new_stats=n, ratio=ratio, rounds=len(paired), verdict=verdict))
    diags = []
    for r in records:
        if r.get("kind") != "diag" or not r.get("parsed"):
            continue
        p = r["parsed"]
        diags.append({"id": r["id"], "group": r["group"], "variant": r["variant"], "exit": r.get("exit"),
                      "wall": r.get("wall"), "step_s": p["timing"].get("step_s"), "ktime": p.get("ktime"),
                      "meminfo": p.get("meminfo"), "halo": p.get("halo"), "graphs": p.get("graphs"), "io": p.get("io")})
    done = [r for r in records if r.get("kind") in ("timed", "diag")]
    return {"commit": job.get("commit"), "plan_id": job.get("id"), "rows": rows, "comparisons": comps,
            "diagnostics": diags,
            "counts": {"planned": len(job.get("runs", [])), "recorded": len(done),
                       "ok": sum(1 for r in done if r.get("exit") == 0),
                       "failed": sum(1 for r in done if r.get("exit") not in (0, None)),
                       "skipped": sum(1 for r in records if r.get("kind") == "skipped")}}


def _f(x, nd=3):
    return "-" if x is None else f"{x:.{nd}f}"


def _range(s, nd=3):
    return "-" if not s else f"{_f(s['median'], nd)} [{_f(s['min'], nd)}, {_f(s['max'], nd)}] (n {s['n']})"


def markdown(agg, job, probe=None):
    out = [f"# Kaggle 2 x T4 performance session ({agg['commit']})", "",
           f"Plan {str(agg['plan_id'])[:12]}; runs planned {agg['counts']['planned']}, recorded {agg['counts']['recorded']}, "
           f"exit 0 {agg['counts']['ok']}, failed {agg['counts']['failed']}, skipped for time {agg['counts']['skipped']}.",
           "", "step_s: seconds per domain-1 step of the steady steps (tools/bench/bench_parse.py; the history-on IO runs "
           "are compared by wall time). Values: median [min, max] (n). Ratios: base / new per round (rounds run the "
           "variants in alternating order), median [min, max]; > 1 means the new variant is faster. 'within noise': "
           "the two ranges overlap or the rounds disagree in sign.", ""]
    measures = job.get("measures", {})
    if probe:
        out += ["## M8 probe", "", "```", probe.strip()[:6000], "```", ""]
    out += ["## Comparisons", "", "| Measure | Group | base -> new | metric | base | new | ratio | verdict |",
            "|---|---|---|---|---|---|---|---|"]
    for c in agg["comparisons"]:
        out.append(f"| {c['measure']} | {c['group']} | {c['base']} -> {c['new']} | {c['metric']} | "
                   f"{_range(c['base_stats'])} | {_range(c['new_stats'])} | {_range(c['ratio'])} | {c['verdict']} |")
    out += ["", "## Timed variants", "",
            "| Group | Variant | step_s | wall s | steady steps | SM MHz | W | max C | throttled | power cap |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for r in agg["rows"]:
        out.append(f"| {r['group']} | {r['variant']} | {_range(r['step_s'], 4)} | {_range(r['wall'], 1)} | "
                   f"{r['steady_steps'] or '-'} | {_range(r['sm_clock_mhz'], 0)} | {_range(r['power_w'], 1)} | "
                   f"{r['temp_max_c'] if r['temp_max_c'] is not None else '-'} | {r['throttled_samples']} | "
                   f"{r['power_cap_samples']} |")
    out += ["", "## Diagnostic runs", ""]
    for d in agg["diagnostics"]:
        out.append(f"### {d['group']} {d['variant']} (exit {d['exit']}, wall {_f(d['wall'], 1)} s)")
        if d["ktime"]:
            k = d["ktime"]
            out.append(f"KTIME: {k['total_s']:.3f} s in {k['kernels']} kernel types; largest:")
            out += ["", "| % | ms | launches | kernel |", "|---|---|---|---|"]
            out += [f"| {x['pct']:.2f} | {x['ms']:.2f} | {x['launches']} | `{x['kernel'][:90]}` |" for x in k["rows"][:15]]
            out.append("")
        if d["meminfo"]:
            m = max(d["meminfo"], key=lambda x: x["used_mib"])
            out.append(f"MEMINFO peak: {m['used_mib']:.0f} of {m['total_mib']:.0f} MiB ({m['when']}); stack "
                       f"{m['stack_bytes']} B x {m['threads']} threads = {m['stack_mib']:.0f} MiB")
        for h in d["halo"] or []:
            out.append(f"HALO worker {h['worker']}: {h.get('exchanges')} exchanges, {_f(h.get('total_s'))} s (pack "
                       f"{_f(h.get('pack_s'))}, wait {_f(h.get('wait_s'))}, xfer {_f(h.get('xfer_s'))}, unpack "
                       f"{_f(h.get('unpack_s'))}); overlapped {h.get('overlapped', 0)}; collectives "
                       + ", ".join(f"kind {c['kind']}: {c['calls']} x {c['ms'] / max(c['calls'], 1) * 1e3:.0f} us"
                                   for c in h["collectives"]))
            for p in h["plans"][:5]:
                out.append(f"  - {p['count']} x {p['plan'][:80]}: {p['ms']:.1f} ms (wait {p['wait_us']:.0f} us, xfer "
                           f"{p['xfer_us']:.0f} us each)")
        for g in d["graphs"] or []:
            out.append(f"GRAPHS: {g['launches']} launches, {g['updates']} updates, {g['instantiations']} "
                       f"instantiations, {g['nodes_per_launch']:.1f} nodes per launch")
        if d["io"]:
            io = d["io"]
            out.append(f"IO: {io['records']} records, total {io['total_s']:.2f} s (values {io['values_s']:.2f}, "
                       f"device copies {io['device_copies_s']:.2f}, puts {io['puts_s']:.2f}, sync {io['sync_s']:.2f})")
        out.append("")
    if measures:
        out += ["## What each measurement answers", ""]
        out += [f"- **{k}**: {v}" for k, v in sorted(measures.items())]
    return "\n".join(out) + "\n"


def write_report(out, job, records):
    agg = aggregate(job, records)
    probe = (out / "probe" / "probe.txt").read_text(errors="replace") if (out / "probe" / "probe.txt").is_file() else None
    (out / "results.json").write_text(json.dumps(agg, indent=1) + "\n")
    (out / "results.md").write_text(markdown(agg, job, probe))
    return agg


def read_records(path):
    recs = []
    if path.is_file():
        for line in path.read_text().splitlines():
            try:
                recs.append(json.loads(line))
            except ValueError:
                pass
    return recs


# ---------------------------------------------------------------------------------------------------------------
# run commands (pure)
def run_command(spec, binary, config, rundir, runs_dir, mca=()):
    """(argv, environment additions) of one planned run."""
    sets = [s.replace("@RUNS@", str(runs_dir)) for s in spec["sets"]]
    argv = [str(binary), str(config), "--set", f"output.dir={rundir}"]
    for s in sets:
        argv += ["--set", s]
    env = {k: str(v) for k, v in spec.get("env", {}).items()}
    layout = spec["layout"]
    cpus = spec.get("cpus", 4)
    if layout == "plain":
        env.update(CUDA_VISIBLE_DEVICES=str(spec.get("gpu", 0)), OMP_NUM_THREADS=str(cpus))
    else:
        px, py = (int(x) for x in layout.split("x"))
        n = px * py
        argv = [*MPIRUN, *mca, "-np", str(n), *argv, "--set", f"run.ranks={layout}"]
        env.update(CUDA_VISIBLE_DEVICES="0,1", OMP_NUM_THREADS=str(max(cpus // n, 1)))
    return argv, env


def overlap_rule(spec):
    """bench_parse --overlap (steady steps = N) for single-domain GPU runs with asynchronous steps; nested runs and
    CPPWRF_KTIME (a device synchronisation after every kernel) use N - 1."""
    return not spec.get("nested") and spec.get("env", {}).get("CPPWRF_KTIME", "0") in ("0", "")


def fits(estimate, now, deadline, factor=1.5):
    return now + factor * estimate <= deadline


# ---------------------------------------------------------------------------------------------------------------
# the session
def sh(argv, log, *, env=None, cwd=None, timeout=None, append=True):
    with open(log, "ab" if append else "wb") as f:
        f.write(("$ " + " ".join(map(str, argv)) + "\n").encode())
        f.flush()
        try:
            rc = subprocess.call(argv, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=cwd, timeout=timeout)
        except subprocess.TimeoutExpired:
            rc = 124
        except OSError as e:
            f.write(f"{e}\n".encode())
            rc = 127
        f.write(f"exit {rc}\n".encode())
    return rc


def temps():
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=30)
        return [float(x) for x in q.stdout.split()] if q.returncode == 0 else []
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return []


def cool_down(idle, pause, delta, cap):
    t0 = time.monotonic()
    time.sleep(pause)
    while time.monotonic() - t0 < cap:
        t = temps()
        if not t or not idle or all(a <= b + delta for a, b in zip(t, idle)):
            break
        time.sleep(2)
    return {"cool_s": round(time.monotonic() - t0, 1), "temps_c": temps()}


def setup_mpi(out):
    if shutil.which("mpicxx") and shutil.which("mpirun"):
        return
    env = dict(os.environ, DEBIAN_FRONTEND="noninteractive")
    for i, cmd in enumerate((["apt-get", "-qq", "update"],
                             ["apt-get", "-qq", "install", "-y", "libopenmpi-dev", "openmpi-bin"])):
        if sh(cmd, out / "apt-mpi.log", env=env, append=i > 0):
            raise RuntimeError("Open MPI setup failed (see apt-mpi.log)")


def build(src, work, out, name, flags, cpus):
    b = work / f"b-{name}"
    log = out / f"build-{name}.log"
    cmake = ["cmake", "-S", str(src), "-B", str(b), "-DCMAKE_BUILD_TYPE=Release", "-DCPPWRF_ENABLE_CUDA=ON",
             "-DCMAKE_CUDA_ARCHITECTURES=75", "-DCPPWRF_MPI=ON", f"-DMPI_CXX_COMPILER={shutil.which('mpicxx')}",
             f"-DCPPWRF_DOUBLE={flags['double']}", f"-DCPPWRF_CUDA_FP32_IDENTITY={flags['identity']}"]
    t0 = time.monotonic()
    if sh(cmake, log, append=False) or sh(["cmake", "--build", str(b), "--parallel", str(cpus), "--target", "cppwrf"], log) \
            or not (b / "cppwrf").is_file():
        raise RuntimeError(f"BUILD FAILED {name} (see {log.name})")
    secs = time.monotonic() - t0
    version = subprocess.run([str(b / "cppwrf"), "--version"], capture_output=True, text=True).stdout.strip()
    return b / "cppwrf", {"seconds": round(secs), "version": version}


def mpi_transport(work, out):
    """The Open MPI transport that runs 4 ranks here: the default, else TCP (as tools/kaggle/kg_body.sh)."""
    c = work / "hello.c"
    c.write_text('#include <mpi.h>\n#include <stdio.h>\nint main(int c, char** v) { int r, n, s = 0; MPI_Init(&c, &v);\n'
                 'MPI_Comm_rank(MPI_COMM_WORLD, &r); MPI_Comm_size(MPI_COMM_WORLD, &n);\n'
                 'MPI_Allreduce(&r, &s, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD); if (r == 0) printf("ok %d %d\\n", n, s);\n'
                 'MPI_Finalize(); return 0; }\n')
    log = out / "mpi.txt"
    if sh(["mpicc", "-O1", "-o", str(work / "hello"), str(c)], log):
        raise RuntimeError("mpicc failed (mpi.txt)")
    for mca in ([], ["--mca", "btl", "self,tcp"]):
        r = subprocess.run([*MPIRUN, *mca, "-np", "4", str(work / "hello")], capture_output=True, text=True, timeout=120)
        with log.open("a") as f:
            f.write(f"transport {mca or 'default'}: {r.stdout.strip()} {r.stderr.strip()[-400:]}\n")
        if "ok 4 6" in r.stdout:
            return mca
    raise RuntimeError("no Open MPI transport (mpi.txt)")


P2P_CU = r'''
#include <cstdio>
#include <chrono>
#include <cuda_runtime.h>
#define CK(x) do { cudaError_t e = (x); if (e != cudaSuccess) { printf("error %s: %s\n", #x, cudaGetErrorString(e)); return 1; } } while (0)
static double gbps(size_t b, double s) { return b / s / 1e9; }
int main() {
  int n = 0; CK(cudaGetDeviceCount(&n)); printf("devices %d\n", n);
  if (n < 2) return 0;
  int a = 0, b = 0; CK(cudaDeviceCanAccessPeer(&a, 0, 1)); CK(cudaDeviceCanAccessPeer(&b, 1, 0));
  printf("canAccessPeer 0->1 %d, 1->0 %d\n", a, b);
  const size_t B = size_t(256) << 20; void *h, *d[2];
  CK(cudaMallocHost(&h, B));
  for (int i = 0; i < 2; ++i) { CK(cudaSetDevice(i)); CK(cudaMalloc(&d[i], B)); }
  for (int i = 0; i < 2; ++i) {
    CK(cudaSetDevice(i)); CK(cudaMemcpy(d[i], h, B, cudaMemcpyHostToDevice));
    auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < 4; ++r) CK(cudaMemcpy(d[i], h, B, cudaMemcpyHostToDevice));
    auto t1 = std::chrono::steady_clock::now();
    for (int r = 0; r < 4; ++r) CK(cudaMemcpy(h, d[i], B, cudaMemcpyDeviceToHost));
    auto t2 = std::chrono::steady_clock::now();
    printf("gpu %d pinned H2D %.2f GB/s, D2H %.2f GB/s\n", i, gbps(4 * B, std::chrono::duration<double>(t1 - t0).count()),
           gbps(4 * B, std::chrono::duration<double>(t2 - t1).count()));
  }
  CK(cudaSetDevice(0)); CK(cudaMemcpyPeer(d[1], 1, d[0], 0, B)); CK(cudaDeviceSynchronize());
  auto t0 = std::chrono::steady_clock::now();
  for (int r = 0; r < 4; ++r) CK(cudaMemcpyPeer(d[1], 1, d[0], 0, B));
  CK(cudaDeviceSynchronize()); CK(cudaSetDevice(1)); CK(cudaDeviceSynchronize());
  auto t1 = std::chrono::steady_clock::now();
  printf("cudaMemcpyPeer 0->1 %.2f GB/s (peer access not enabled)\n", gbps(4 * B, std::chrono::duration<double>(t1 - t0).count()));
  // both GPUs' host-to-device copies at once: the shared PCIe path of two ranks' halo staging
  void* h2; CK(cudaMallocHost(&h2, B)); cudaStream_t s[2];
  for (int i = 0; i < 2; ++i) { CK(cudaSetDevice(i)); CK(cudaStreamCreate(&s[i])); }
  auto t2 = std::chrono::steady_clock::now();
  for (int r = 0; r < 4; ++r) for (int i = 0; i < 2; ++i) { CK(cudaSetDevice(i)); CK(cudaMemcpyAsync(d[i], i ? h2 : h, B, cudaMemcpyHostToDevice, s[i])); }
  for (int i = 0; i < 2; ++i) { CK(cudaSetDevice(i)); CK(cudaStreamSynchronize(s[i])); }
  auto t3 = std::chrono::steady_clock::now();
  printf("both GPUs H2D at once %.2f GB/s in all\n", gbps(8 * B, std::chrono::duration<double>(t3 - t2).count()));
  return 0;
}
'''

RE_SASS_FUN = re.compile(r"^\s*Function : (\S+)")
SASS_KEYS = re.compile(r"K(Sw|Lw)[A-Z]\w*|rrtmg", re.I)


def sass_byte_loads(binary, log):
    """cuobjdump -sass: per kernel whose demangled name names an RRTMG kernel (KSw* / KLw*), the global / generic /
    constant loads and how many of them are byte loads (.U8 / .S8)."""
    p = subprocess.Popen(["timeout", "900", "cuobjdump", "-sass", str(binary)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, errors="replace")
    filt = shutil.which("cu++filt") or shutil.which("c++filt")
    counts = {}
    cur = None
    for line in p.stdout:
        if m := RE_SASS_FUN.match(line):
            cur = m[1]
            counts.setdefault(cur, [0, 0])
            continue
        if cur is None or not re.search(r"\b(LDG|LD|LDC)\.", line):
            continue
        c = counts[cur]
        c[0] += 1
        if re.search(r"\b(LDG|LD|LDC)\.[A-Z0-9.]*\b[US]8\b", line):
            c[1] += 1
    p.wait()
    names = list(counts)
    if filt and names:
        r = subprocess.run([filt], input="\n".join(names), capture_output=True, text=True)
        dem = r.stdout.splitlines() if r.returncode == 0 and len(r.stdout.splitlines()) == len(names) else names
    else:
        dem = names
    rows = sorted(((d, *counts[n]) for n, d in zip(names, dem) if SASS_KEYS.search(d)), key=lambda x: -x[2])
    with open(log, "w") as f:
        f.write(f"# {binary}: {len(names)} kernels; RRTMG kernels: loads, byte loads\n")
        for d, loads, u8 in rows:
            f.write(f"{loads:7d} {u8:7d}  {d[:200]}\n")
    return {"kernels": len(names), "rrtmg_kernels": len(rows), "rrtmg_loads": sum(r[1] for r in rows),
            "rrtmg_byte_loads": sum(r[2] for r in rows)}


def probes(work, out, binaries):
    """M8: the node's GPU topology, P2P, copy bandwidths, Open MPI's CUDA support, the profilers, the kernels' resource
    usage and the RRTMG kernels' byte loads (no model run)."""
    d = out / "probe"
    d.mkdir(exist_ok=True)
    lines = []

    def q(name, argv, timeout=120):
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
            txt = (r.stdout + r.stderr).strip()
        except (OSError, subprocess.TimeoutExpired) as e:
            txt = f"unavailable: {e}"
        (d / f"{name}.txt").write_text(txt + "\n")
        return txt

    lines.append("topo:\n" + q("topo", ["nvidia-smi", "topo", "-m"]))
    q("smi_q", ["nvidia-smi", "-q"])
    lines.append(q("clocks", ["nvidia-smi", "--query-gpu=index,name,power.limit,power.max_limit,clocks.max.sm,"
                              "clocks.max.mem,clocks.applications.graphics,pcie.link.gen.current,pcie.link.width.current",
                              "--format=csv"]))
    ompi = q("ompi_info", ["ompi_info", "--parsable", "--all"], 300)
    lines.append("ompi cuda: " + " ".join(ln for ln in ompi.splitlines() if "cuda" in ln.lower() and "value" in ln)[:600])
    lines.append("mpirun: " + (q("mpirun", ["mpirun", "--version"]).splitlines() or [""])[0])
    for tool in ("nsys", "ncu", "cuobjdump", "nvdisasm"):
        lines.append(f"{tool}: {shutil.which(tool) or 'not found'}")
    cu = work / "p2p.cu"
    cu.write_text(P2P_CU)
    if sh(["nvcc", "-O2", "-arch=sm_75", "-o", str(work / "p2p"), str(cu)], d / "p2p_build.txt") == 0:
        lines.append("p2p:\n" + q("p2p", [str(work / "p2p")], 300))
    if shutil.which("cuobjdump"):
        for name, b in binaries.items():
            t0 = time.monotonic()
            try:
                r = subprocess.run(["cuobjdump", "--dump-resource-usage", str(b)], capture_output=True, text=True,
                                   timeout=900)
                with gzip.open(d / f"resource_{name}.txt.gz", "wt") as f:
                    f.write(r.stdout)
                s = sass_byte_loads(b, d / f"sass_rrtmg_{name}.txt")
                lines.append(f"sass {name}: {s} ({time.monotonic() - t0:.0f} s)")
            except (OSError, subprocess.TimeoutExpired) as e:
                lines.append(f"cuobjdump {name}: {e}")
    (d / "probe.txt").write_text("\n".join(lines) + "\n")


def run_one(spec, ctx):
    import telemetry  # tools/bench/telemetry.py, bundled next to this file

    rid = spec["id"]
    rundir = ctx["work"] / "runs" / rid
    shutil.rmtree(rundir, ignore_errors=True)
    rundir.mkdir(parents=True)
    argv, env_add = run_command(spec, ctx["bins"][spec["build"]], ctx["src"] / spec["config"], rundir,
                                ctx["runs_dir"], ctx["mca"])
    env = dict(ctx["env"], **env_add)
    cool = cool_down(ctx["idle"], ctx["job"]["pause"], ctx["job"]["cool_delta"], ctx["job"]["cool_max"])
    tel_dir = ctx["out"] / "telemetry"
    tel_dir.mkdir(exist_ok=True)
    samplers = [telemetry.Sampler(str(ctx["work"] / f"tel_{rid}_g{i}.csv"), 100, ctx["fields"], gpu=i) for i in (0, 1)]
    for s in samplers:
        s.start()
    time.sleep(1.5)
    log = ctx["out"] / "logs" / f"{rid}.log"
    t0, m0 = time.time(), time.monotonic()
    with log.open("wb") as f:
        f.write(("$ " + " ".join(f"{k}={v}" for k, v in sorted(env_add.items())) + " " + " ".join(argv) + "\n").encode())
        f.flush()
        try:
            rc = subprocess.call(argv, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=ctx["src"],
                                 timeout=max(600, 5 * spec["estimate"]))
        except subprocess.TimeoutExpired:
            rc = 124
    wall, t1 = time.monotonic() - m0, time.time()
    time.sleep(0.3)
    for s in samplers:
        s.stop()
    text = log.read_text(errors="replace")
    tel = {}
    for i in (0, 1):
        csv = ctx["work"] / f"tel_{rid}_g{i}.csv"
        if csv.is_file():
            tel[f"gpu{i}"] = telemetry.summarize(str(csv), t0, t1)
            with csv.open("rb") as a, gzip.open(tel_dir / f"{rid}_g{i}.csv.gz", "wb") as b:
                shutil.copyfileobj(a, b)
            csv.unlink()
    rec = {k: spec[k] for k in ("id", "block", "group", "variant", "kind", "case", "build", "layout", "rep", "order",
                                "measures", "estimate")}
    rec.update(exit=rc, wall=round(wall, 3), t0=t0, t1=t1, env=env_add, cool=cool, telemetry=tel,
               parsed=parse_log(text, wall, overlap_rule(spec)))
    shutil.rmtree(rundir, ignore_errors=True)
    return rec


def main(job_path, out):
    job = json.loads(job_path.read_text())
    start = time.monotonic()
    deadline = start + job["limit"] - job["margin"]
    out.mkdir(parents=True, exist_ok=True)
    (out / "logs").mkdir(exist_ok=True)
    (out / "perf-job.json").write_text(json.dumps(job, indent=1) + "\n")
    import kg_proof_body as pb  # tools/kaggle/kg_proof_body.py: setup and checked data streaming, bundled

    scratch = pb.measured_scratch(out, job["disk"] + job["reserve"])
    pb.get_gpus(out)
    pb.setup_packages(out)
    setup_mpi(out)
    cpus = len(os.sched_getaffinity(0))
    phases = {}
    with tempfile.TemporaryDirectory(prefix="cppwrf-perf-", dir=scratch) as temp:
        work = Path(temp)
        src = work / "src"
        src.mkdir()
        subprocess.run(["tar", "xzf", str(job_path.parent / "src.tgz.dat"), "-C", str(src)], check=True)
        if (src / "src/cases/commit.txt").read_text().strip() != job["commit"]:
            raise RuntimeError("candidate commit does not match the git archive")
        data = work / "data"
        data.mkdir()
        errors = []

        def fetch():
            t0 = time.monotonic()
            try:
                for name in sorted(job["tars"]):
                    pb.fetch_tar(job, job["tars"][name], work, data, out / "data.txt")
            except Exception as e:  # noqa: BLE001 - reported after the builds
                errors.append(str(e))
            phases["data_s"] = round(time.monotonic() - t0)

        fetcher = threading.Thread(target=fetch)
        fetcher.start()
        bins, info = {}, {}
        t0 = time.monotonic()
        for name, flags in job["builds"].items():
            bins[name], info[name] = build(src, work, out, name, flags, cpus)
        phases["builds_s"] = round(time.monotonic() - t0)
        fetcher.join()
        if errors:
            raise RuntimeError("; ".join(errors))
        mca = mpi_transport(work, out)
        t0 = time.monotonic()
        probes(work, out, bins)
        phases["probe_s"] = round(time.monotonic() - t0)
        import telemetry

        ctx = {"job": job, "work": work, "src": src, "bins": bins, "out": out, "mca": mca,
               "runs_dir": data / "wrf_runs", "fields": telemetry.query_fields(), "idle": temps(),
               "env": dict(os.environ, CPPWRF_WRF_RUNS=str(data / "wrf_runs"), OMP_WAIT_POLICY="passive")}
        (out / "session.json").write_text(json.dumps({"builds": info, "phases": phases, "mca": mca, "cpus": cpus,
                                                      "idle_temps": ctx["idle"], "fields": ctx["fields"]}, indent=1))
        records = []
        bad = False
        t_runs = time.monotonic()
        with (out / "records.jsonl").open("a") as recs:
            for spec in job["runs"]:
                if not fits(spec["estimate"], time.monotonic(), deadline):
                    rec = {k: spec[k] for k in ("id", "block", "group", "variant", "case", "build", "layout", "rep",
                                                "measures", "estimate")}
                    rec.update(kind="skipped", planned_kind=spec["kind"])
                else:
                    print(f"run {spec['id']}", flush=True)
                    try:
                        rec = run_one(spec, ctx)
                    except Exception as e:  # noqa: BLE001 - one broken run does not end the session
                        rec = {k: spec[k] for k in ("id", "block", "group", "variant", "kind", "case", "build",
                                                    "layout", "rep", "measures", "estimate")}
                        rec.update(exit=-1, error=f"{type(e).__name__}: {e}")
                    bad |= rec["exit"] != 0
                records.append(rec)
                recs.write(json.dumps(rec) + "\n")
                recs.flush()
        phases["runs_s"] = round(time.monotonic() - t_runs)
        phases["total_s"] = round(time.monotonic() - start)
        (out / "session.json").write_text(json.dumps({"builds": info, "phases": phases, "mca": mca, "cpus": cpus,
                                                      "idle_temps": ctx["idle"], "fields": ctx["fields"]}, indent=1))
        write_report(out, job, records)
    return int(bad)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "report":
        d = Path(sys.argv[2])
        a = write_report(d, json.loads((d / "perf-job.json").read_text()), read_records(d / "records.jsonl"))
        print((d / "results.md").read_text(), end="")
        sys.exit(0)
    try:
        sys.exit(main(Path(sys.argv[1]), Path(sys.argv[2])))
    except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as error:
        message = f"PERF SESSION FAILED: {error}"
        print(message, file=sys.stderr)
        (Path(sys.argv[2]) / "setup.log").write_text(message + "\n")
        out = Path(sys.argv[2])
        try:  # whatever ran is still reported
            write_report(out, json.loads(Path(sys.argv[1]).read_text()), read_records(out / "records.jsonl"))
        except (OSError, ValueError, KeyError):
            pass
        sys.exit(1)
