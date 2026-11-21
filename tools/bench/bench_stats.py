#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Statistics, speed-ups, energy and cost of a paper benchmark session (tools/bench/PROTOCOL.md, "Report").

  bench_stats.py RAW_RUNS.jsonl [--verify VERIFY.jsonl] [--manifest MANIFEST.json] [--cases CASES.json]
                 [--best BEST.json] [--rate CU_PER_HOUR] [--cpu-rate CU_PER_HOUR] [--accuracy-md FILE]
                 [--md REPORT.md] [--json SUMMARY.json] [--csv RUNS.csv]
  bench_stats.py --help

RAW_RUNS.jsonl: tools/bench/paper_run.py's runs (one JSON line each). Warm-up runs, failed runs and the proof runs
are not counted (the failures are listed). For every case, model and history setting ("regime" off / on) over the
timed repetitions: the median, the quartiles (q25, q75: numpy's linear interpolation), the minimum and the maximum of

- wall: the process, launch to exit (time.monotonic());
- stepping, step1, steady, io, other: tools/bench/bench_parse.py's split of the WRF timing lines both models print
  (wall = other + step1 + steady + io);
- the GPU's board energy (J) and mean power (W) over the run, the median SM clock (nvidia-smi samples,
  tools/bench/telemetry.py);

and from the medians: seconds per simulated day (wall, and steady stepping scaled to a day), simulated days per
wall day (sim. time / wall), SYPD (simulated years per wall-clock day, from the wall time and from the steady
stepping), ns per cell and step (the steady stepping over the cells x steps of all domains after step 1), GPU energy
per simulated day (Wh). Speed-ups against WRF real*4 and real*8 at their fastest screened layouts: the ratio of the
two runs of the same block (the blocks interleave the models, so a block's runs share the machine's state), as the
median over the blocks with the smallest and largest block ratio; and the ratio of the medians.

Cost: Colab compute units per simulated day = median wall / 3600 x 86400 / simulated seconds x RATE, with RATE the
session's runtime rate from `colab usage` (--rate; the runtime costs the same whichever processor a run uses, so
for WRF this is the co-located price: an upper bound, the GPU idles). --cpu-rate: the rate of a CPU-only runtime
with the same CPU, as a STATED ASSUMPTION (printed as such): WRF's cost there. Without --cpu-rate no CPU-only price
is printed. Runs whose load (processes running just before) was above 0 or whose GPU samples show throttling are
flagged. --md: the Markdown report; --json: the summary for tools/bench/bench_plot.py; --csv: one line per run.
"""
import csv
import json
import math
import os
import sys

import numpy as np

NAMES = {"wrf_r4": "WRF real*4", "wrf_r8": "WRF real*8", "wrf_r4t": "WRF real*4 -O3 -march=native", "gpu_fp32": "CPPWRF GPU FP32 (fast)",
         "gpu_fp32id": "CPPWRF GPU FP32 identity", "gpu_fp64": "CPPWRF GPU FP64", "cpu_fp64": "CPPWRF CPU FP64",
         "gpu_fp32_nc0": "CPPWRF GPU FP32, HDF5 deflate", "wrf_r8_dm1": "WRF real*8, one process",
         "wrf_r4_dm1": "WRF real*4, one process", "cpu_fp64_hwlog": "CPPWRF CPU FP64 (this CPU's log)",
         "cpu_fp32": "CPPWRF CPU FP32"}
ORDER = ["wrf_r4", "wrf_r4t", "wrf_r8", "gpu_fp32", "gpu_fp32id", "gpu_fp64", "cpu_fp64", "gpu_fp32_nc0"]
METRICS = ("wall", "stepping", "step1", "steady", "io", "other", "energy_j", "power_mean_w", "sm_clock_median_mhz")


def stats(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return None
    a = np.asarray(v, dtype=float)
    return dict(n=len(v), median=float(np.median(a)), q25=float(np.quantile(a, 0.25)), q75=float(np.quantile(a, 0.75)),
                min=float(a.min()), max=float(a.max()))


def value(r, k):
    if k == "wall":
        return r.get("wall")
    if k in ("energy_j", "power_mean_w", "sm_clock_median_mhz"):
        return (r.get("gpu") or {}).get(k)
    return (r.get("timing") or {}).get(k)


def load(path):
    return [json.loads(x) for x in open(path) if x.strip()] if path and os.path.exists(path) else []


def summarize(runs, verify, cases_info=None, best=None, manifest=None, rate=None, cpu_rate=None):
    timed = [r for r in runs if r.get("phase") == "timed" and not r.get("warmup")]
    ok = [r for r in timed if r.get("ok")]
    failed = [r for r in runs if not r.get("ok")]
    cases = []
    for r in runs:
        if r.get("case") and r["case"] not in cases:
            cases.append(r["case"])
    groups, speed = [], []
    for c in cases:
        for reg in ("off", "on"):
            rs = [r for r in ok if r["case"] == c and r["regime"] == reg]
            models = sorted({r["model"] for r in rs}, key=lambda m: ORDER.index(m) if m in ORDER else 99)
            for m in models:
                g = [r for r in rs if r["model"] == m]
                st = {k: stats([value(r, k) for r in g]) for k in METRICS}
                r0 = g[0]
                sim = r0.get("sim_seconds") or 3600
                cs = r0.get("grid") or [[0, 1]]
                cell_steps = sum(a * b for a, b in cs)
                n1 = cs[0][1] if cs else 1
                steady_frac = (n1 - 1) / n1 if n1 > 1 else 1
                d = {}
                w = st["wall"]["median"] if st["wall"] else None
                sd = st["steady"]["median"] if st["steady"] else None
                if w:
                    d.update(s_per_sim_day_wall=w * 86400 / sim, sim_speed_wall=sim / w, sypd_wall=sim / (365.25 * w))
                if sd and steady_frac > 0:
                    d.update(s_per_sim_day_steady=sd * 86400 / (sim * steady_frac),
                             sypd_steady=sim * steady_frac / (365.25 * sd),
                             ns_cell_step=sd * 1e9 / (cell_steps * steady_frac) if cell_steps else None)
                if st["energy_j"]:
                    d["gpu_wh_per_sim_day"] = st["energy_j"]["median"] / 3600 * 86400 / sim
                if w and rate:
                    d["cu_per_sim_day"] = w / 3600 * 86400 / sim * rate
                if w and cpu_rate and m.startswith("wrf"):
                    d["cu_per_sim_day_cpu_runtime"] = w / 3600 * 86400 / sim * cpu_rate
                flags = dict(loaded=sum(1 for r in g if (r.get("load") or 0) > 0),
                             throttled=sum(1 for r in g if ((r.get("gpu") or {}).get("throttled_samples") or 0) > 0))
                groups.append(dict(case=c, regime=reg, model=m, layout=r0.get("layout"), n=len(g), sim_seconds=sim,
                                   grid=cs, stats=st, derived=d, flags=flags))
            for base in ("wrf_r4", "wrf_r4t", "wrf_r8"):
                b = {r["block"]: r for r in rs if r["model"] == base}
                if not b:
                    continue
                for m in models:
                    if m.startswith("wrf"):
                        continue
                    for k in (("stepping", "steady", "wall") if reg == "off" else ("wall",)):
                        ratios = [value(b[r["block"]], k) / value(r, k) for r in rs if r["model"] == m
                                  and r["block"] in b and value(r, k) and value(b[r["block"]], k)]
                        gm = next((x for x in groups if x["case"] == c and x["regime"] == reg and x["model"] == m), None)
                        gb = next((x for x in groups if x["case"] == c and x["regime"] == reg and x["model"] == base),
                                  None)
                        mr = (gb["stats"][k]["median"] / gm["stats"][k]["median"]
                              if gm and gb and gm["stats"][k] and gb["stats"][k] and gm["stats"][k]["median"] else None)
                        if ratios:
                            speed.append(dict(case=c, regime=reg, model=m, vs=base, metric=k, n=len(ratios),
                                              median=float(np.median(ratios)), min=min(ratios), max=max(ratios),
                                              ratio_of_medians=mr))
    layouts = []
    for r in runs:
        if r.get("phase") == "screen" and r.get("ok"):
            layouts.append(dict(case=r["case"], model=r["model"], layout=r.get("layout"),
                                stepping=r["timing"]["stepping"], wall=r["wall"]))
    lay = {}
    for x in layouts:
        lay.setdefault((x["case"], x["model"], x["layout"]), []).append(x["stepping"])
    screen = [dict(case=c, model=m, layout=layo, n=len(v), median=float(np.median(v)))
              for (c, m, layo), v in lay.items()]
    return dict(cases=cases, cases_info=cases_info or {}, groups=groups, speedups=speed, screen=screen, best=best or {},
                verify=[v for v in verify if v.get("kind") in ("exact", "l2")],
                accuracy=[v for v in verify if v.get("kind") == "accuracy"],
                failed=[dict(case=r.get("case"), phase=r.get("phase"), model=r.get("model"), layout=r.get("layout"),
                             regime=r.get("regime"), block=r.get("block"), error=str(r.get("error"))[:300])
                        for r in failed],
                n_runs=len(runs), n_timed=len(timed), rate=rate, cpu_rate=cpu_rate, manifest=manifest_summary(manifest))


def manifest_summary(m):
    if not m:
        return None
    out = {}
    for side, v in m.items():
        if not isinstance(v, dict) or "host" not in v:
            continue
        h = v["host"]
        out[side] = dict(commit=v.get("commit"), cpu=h.get("cpu"), vcpus=h.get("vcpus"), cores=h.get("cores"),
                         memory_mib=h.get("memory_mib"), os=h.get("os"), gpu=(h.get("gpu") or {}).get("value"),
                         cuda=(h.get("compilers") or {}).get("nvcc"), gfortran=(h.get("compilers") or {}).get("gfortran"),
                         gcc=(h.get("compilers") or {}).get("gcc"), mpi=(h.get("compilers") or {}).get("mpich"),
                         glibc=(h.get("compilers") or {}).get("glibc"),
                         energy_counters=h.get("energy_counters"),
                         cppwrf={n: dict(flavour=b.get("flavour"), version=b.get("version"),
                                         sha256=(b.get("sha256") or "")[:16]) for n, b in v.get("cppwrf_builds", {}).items()},
                         wrf={n: dict(fcoptim=b["configure_wrf"].get("FCOPTIM"), promotion=b["configure_wrf"].get(
                             "PROMOTION"), dm=b["configure_wrf"].get("DMPARALLEL"), omp=b["configure_wrf"].get("OMPCPP"),
                             wrf_exe=(b.get("wrf_exe_sha256") or "")[:16]) for n, b in v.get("wrf_builds", {}).items()})
    return out


def f(x, n=2):
    if x is None:
        return "-"
    if abs(x) >= 1000:
        return f"{x:.0f}"
    return f"{x:.{n}f}"


def cell(s, n=2):
    """median [q25 - q75] (min - max)"""
    if not s:
        return "-"
    if s["n"] == 1:
        return f(s["median"], n)
    return f"{f(s['median'], n)} [{f(s['q25'], n)}-{f(s['q75'], n)}] ({f(s['min'], n)}-{f(s['max'], n)})"


def markdown(S, accuracy_md=None):
    o = []
    w = o.append
    w("# CPPWRF and WRF v4.8.0: paper benchmark\n")
    w(f"{S['n_timed']} timed runs (warm-up excluded) of {S['n_runs']} runs; tools/bench/PROTOCOL.md describes the "
      "protocol. Cells: median [q25-q75] (min-max) over the timed repetitions; times in seconds.\n")
    ms = S.get("manifest") or {}
    if ms:
        w("## Machine and builds\n")
        for side, v in ms.items():
            w(f"- **{side}**: commit {v['commit']}; {v['cpu']} ({v['vcpus']} vCPUs, {v['cores']} cores, "
              f"{f((v['memory_mib'] or 0) / 1024, 0)} GiB); {v['os']}; GPU {v['gpu'] or '-'}; {v['cuda'] or ''}; "
              f"{v['gcc'] or ''}; {v['gfortran'] or ''}; {v['mpi'] or ''}; {v['glibc'] or ''}")
            for n, b in (v.get("cppwrf") or {}).items():
                w(f"  - CPPWRF {n}: {b['flavour']} ({b['version']}, sha256 {b['sha256']})")
            for n, b in (v.get("wrf") or {}).items():
                w(f"  - WRF {n}: FCOPTIM `{b['fcoptim']}`, PROMOTION `{b['promotion'] or ''}`, "
                  f"DMPARALLEL {b['dm'] or '-'}, OMPCPP `{b['omp'] or ''}` (wrf.exe sha256 {b['wrf_exe']})")
            ec = v.get("energy_counters") or {}
            w(f"  - CPU energy counters: {len(ec.get('powercap_readable', []))} readable powercap counters, perf "
              f"{'present' if ec.get('perf') else 'absent'}")
        w("")
    ci = S.get("cases_info") or {}
    for c in S["cases"]:
        info = ci.get(c, {})
        w(f"## {c}\n")
        if info.get("desc"):
            w(info["desc"] + "\n")
        g0 = next((g for g in S["groups"] if g["case"] == c), None)
        if g0:
            w("Cells x steps per domain: " + ", ".join(f"d{i + 1:02d} {a} x {b}" for i, (a, b) in enumerate(g0["grid"]))
              + f"; simulated {g0['sim_seconds'] / 3600:g} h.\n")
        cpu = [x for x in S["screen"] if x["case"] == c and not x["model"].startswith("wrf")]
        if cpu:
            w("CPPWRF CPU thread counts (1 h, history off), stepping s: " + ", ".join(
                f"{x['layout']} {x['median']:.2f}" for x in sorted(cpu, key=lambda x: x["layout"])) + ".\n")
        scr = [x for x in S["screen"] if x["case"] == c and x["model"].startswith("wrf")]
        if scr:
            w("WRF process layouts (1 h, history off), stepping s, median of the screening runs:\n")
            lays = sorted({x["layout"] for x in scr}, key=lambda s: (s[:2], int(s[2:].split("x")[0]) if s[2:].split("x")[0].isdigit() else 0, s))
            mods = sorted({x["model"] for x in scr})
            w("| layout | " + " | ".join(NAMES.get(m, m) for m in mods) + " |")
            w("|---|" + "---|" * len(mods))
            for la in lays:
                row = []
                for m in mods:
                    v = next((x for x in scr if x["layout"] == la and x["model"] == m), None)
                    b = S["best"].get(f"{c}:{m}")
                    row.append("-" if not v else (f"**{v['median']:.2f}**" if b == la else f"{v['median']:.2f}"))
                w(f"| {la} | " + " | ".join(row) + " |")
            w("")
        for reg, title in (("off", "History off (the frame at t = 0 only)"), ("on", "History on")):
            gs = [g for g in S["groups"] if g["case"] == c and g["regime"] == reg]
            if not gs:
                continue
            w(f"### {title}\n")
            if reg == "on":
                w("With history on, a GPU run's write waits for the steps queued before it (the steps run "
                  "asynchronously), so for the GPU builds the split between stepping and I/O is not meaningful; "
                  "their sum and the wall time are. The speed-ups below are of the wall time only.\n")
            w("| model | layout | n | wall | stepping | step 1 | steady | I/O | other | ns / cell / step | "
              "s / sim. day (wall) | SYPD (wall / steady) |")
            w("|---|---|---|---|---|---|---|---|---|---|---|---|")
            for g in gs:
                s, d = g["stats"], g["derived"]
                w(f"| {NAMES.get(g['model'], g['model'])} | {g['layout']} | {g['n']} | {cell(s['wall'], 1)} | "
                  f"{cell(s['stepping'])} | {cell(s['step1'])} | {cell(s['steady'])} | {cell(s['io'])} | "
                  f"{cell(s['other'])} | {f(d.get('ns_cell_step'), 1)} | {f(d.get('s_per_sim_day_wall'), 0)} | "
                  f"{f(d.get('sypd_wall'), 3)} / {f(d.get('sypd_steady'), 3)} |")
            w("")
            sp = [x for x in S["speedups"] if x["case"] == c and x["regime"] == reg]
            if sp:
                w("Speed-up (WRF time / CPPWRF time), median over the blocks (smallest - largest block):\n")
                w("| model | vs | stepping | steady | wall |")
                w("|---|---|---|---|---|")
                for m in sorted({x["model"] for x in sp}, key=lambda m: ORDER.index(m) if m in ORDER else 99):
                    for vs in ("wrf_r4", "wrf_r4t", "wrf_r8"):
                        row = []
                        for k in ("stepping", "steady", "wall"):
                            x = next((y for y in sp if y["model"] == m and y["vs"] == vs and y["metric"] == k), None)
                            row.append(f"{x['median']:.2f}x ({x['min']:.2f}-{x['max']:.2f})" if x else "-")
                        if any(r != "-" for r in row):
                            w(f"| {NAMES.get(m, m)} | {NAMES[vs]} | " + " | ".join(row) + " |")
                w("")
            eg = [g for g in gs if g["stats"]["energy_j"]]
            if eg:
                w("GPU board energy (nvidia-smi power.draw integrated over the run; the board only):\n")
                w("| model | energy J | mean power W | SM clock MHz | Wh / sim. day | runs with throttling |")
                w("|---|---|---|---|---|---|")
                for g in eg:
                    s = g["stats"]
                    w(f"| {NAMES.get(g['model'], g['model'])} | {cell(s['energy_j'], 0)} | {cell(s['power_mean_w'], 0)} | "
                      f"{cell(s['sm_clock_median_mhz'], 0)} | {f(g['derived'].get('gpu_wh_per_sim_day'), 1)} | "
                      f"{g['flags']['throttled']} |")
                w("")
    if S.get("rate"):
        w("## Cost\n")
        w(f"Colab compute units per simulated day: median wall time x the session's rate, {S['rate']} units per hour "
          "(`colab usage`). The runtime costs the same whichever processor a run uses: for WRF this is the price on "
          "the GPU runtime with the GPU idle, an upper bound." + (
              f" The CPU-only column assumes {S['cpu_rate']} units per hour for a CPU-only runtime with the same CPU: "
              "a stated assumption, not a measurement." if S.get("cpu_rate") else
              " A CPU-only runtime's price was not given (--cpu-rate) and is not shown.") + "\n")
        w("| case | history | model | CU / sim. day | CU / sim. day, CPU-only runtime |")
        w("|---|---|---|---|---|")
        for g in S["groups"]:
            d = g["derived"]
            if "cu_per_sim_day" in d:
                w(f"| {g['case']} | {g['regime']} | {NAMES.get(g['model'], g['model'])} | {d['cu_per_sim_day']:.3f} | "
                  f"{f(d.get('cu_per_sim_day_cpu_runtime'), 3)} |")
        w("")
    w("## Bit identity and accuracy in the same runs\n")
    w("Tolerance 0 (tests/verify/exact.py: every variable of every frame, NaN and signed zero exact) on the history-on "
      "outputs of the last timed block and of the proof runs:\n")
    for v in S["verify"]:
        if v["kind"] != "exact":
            continue
        parts = []
        for d, x in v["domains"].items():
            if "error" in x:
                parts.append(f"{d}: {x['error']}")
            else:
                parts.append(f"{d}: {x['frames']} frames " + ("**identical**" if x["identical"] else
                             "DIFFER (" + ", ".join(sorted(x["differ"])[:8]) + ")"))
        w(f"- {v['case']}: {v['what']}: " + "; ".join(parts))
    l2s = [v for v in S["verify"] if v["kind"] == "l2"]
    if l2s:
        w("\nRelative L2 difference ||a - b|| / ||b|| at the last frame against CPPWRF GPU FP64:\n")
        for v in l2s:
            for d, x in v["domains"].items():
                if "l2" in x:
                    w(f"- {v['case']} {d} {NAMES.get(v['a'], v['a'])}: " + ", ".join(
                        f"{k} {y:.1e}" for k, y in x["l2"].items() if y is not None))
    if accuracy_md and os.path.exists(accuracy_md):
        w(f"\nThe FP32 accuracy ratio R per variable and hour: {os.path.basename(accuracy_md)} "
          "(tools/verify/fp32_accuracy.py report).")
    if S["failed"]:
        w("\n## Failed runs\n")
        for x in S["failed"]:
            w(f"- {x['case']} {x['phase']} {x['model']} {x.get('layout') or ''} {x.get('regime') or ''} block "
              f"{x.get('block')}: {x['error']}")
    fl = [g for g in S["groups"] if g["flags"]["loaded"] or g["flags"]["throttled"]]
    if fl:
        w("\n## Flagged repetitions\n")
        for g in fl:
            w(f"- {g['case']} {g['regime']} {g['model']}: {g['flags']['loaded']} runs started with other processes "
              f"running, {g['flags']['throttled']} with GPU throttling samples")
    return "\n".join(o) + "\n"


def write_csv(runs, path):
    cols = ["id", "phase", "case", "model", "build", "layout", "regime", "block", "warmup", "ok", "exit", "wall",
            "stepping", "step1", "steady", "io", "other", "steps", "load", "cpu_mhz", "energy_j", "power_mean_w",
            "power_peak_w", "power_idle_w", "sm_clock_median_mhz", "temp_max_c", "mem_used_peak_mib",
            "throttled_samples"]
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(cols)
        for r in runs:
            t, g = r.get("timing") or {}, r.get("gpu") or {}
            wr.writerow([r.get(k) if k in r else t.get(k, g.get(k)) for k in cols])


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if a else 2)
    opt = lambda k: a[a.index(k) + 1] if k in a else None  # noqa: E731
    runs = load(a[0])
    S = summarize(runs, load(opt("--verify")), json.load(open(opt("--cases"))) if opt("--cases") else None,
                  json.load(open(opt("--best"))) if opt("--best") else None,
                  json.load(open(opt("--manifest"))) if opt("--manifest") else None,
                  float(opt("--rate")) if opt("--rate") else None, float(opt("--cpu-rate")) if opt("--cpu-rate") else None)
    md = markdown(S, opt("--accuracy-md"))
    if opt("--md"):
        open(opt("--md"), "w").write(md)
    else:
        sys.stdout.write(md)
    if opt("--json"):
        json.dump(S, open(opt("--json"), "w"), indent=1)
    if opt("--csv"):
        write_csv(runs, opt("--csv"))


if __name__ == "__main__":
    main()
