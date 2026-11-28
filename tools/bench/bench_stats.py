#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Statistics, speed-ups, energy and cost of a paper benchmark session (tools/bench/PROTOCOL.md, "Report").

  bench_stats.py RAW_RUNS.jsonl [--verify VERIFY.jsonl] [--manifest MANIFEST.json] [--cases CASES.json]
                 [--best BEST.json] [--rate CU_PER_HOUR] [--cpu-rate CU_PER_HOUR] [--accuracy-md FILE]
                 [--md REPORT.md] [--json SUMMARY.json] [--csv RUNS.csv]
  bench_stats.py --help

RAW_RUNS.jsonl: tools/bench/paper_run.py's runs (one JSON line each). Warm-up runs, failed runs and the proof runs
are not counted (the failures are listed). For every case, model and history setting ("regime" off / on) over the
timed repetitions: the median, the quartiles (q25, q75: numpy's linear interpolation), the minimum and the maximum,
a 95 % percentile bootstrap interval of the median (10000 resamples, fixed seed; 3 or more values) and the
coefficient of variation of

- wall: the process, launch to exit (time.monotonic());
- tools/bench/bench_parse.py's split of the WRF timing lines both models print: step_s (seconds per step over the
  steady steps 2 .. N: the headline compute metric), steady, step1, stepping, io, io_steady, input_in, other
  (wall = other + step1 + steady + io + input_in), marginal (the part that grows with the simulated time) and fixed
  (wall - marginal: start-up, the frame at t = 0, the finish);
- the GPU's board energy over the run and over the steady steps (J), mean and steady power (W), the median SM clock
  (nvidia-smi samples, tools/bench/telemetry.py).

With history on, the CUDA builds' split between steps and writes (step_s, steady, step1, stepping, io, io_steady)
is not published (a write waits for the queued steps): wall, other, marginal and fixed are.

From the medians: SYPD from the wall time and from the steady rate, ns per cell and step (step_s over the cell
updates of all domains per domain-1 step), and one simulated day as fixed + marginal x 86400 / simulated seconds
(the fixed start-up counted once, not scaled with the length); the GPU energy of a day likewise (the fixed energy
once + the marginal energy of a day, charging the steady-window part marginal - io_post at power_steady_w and the
post-step CPU write io_post at power_idle_w). Speed-ups against WRF real*4, tuned real*4 and real*8 at their
fastest screened layouts: per block, WRF's run over CPPWRF's run of that block (the blocks interleave the models),
over the blocks both ran in: the median block ratio with its bootstrap interval and range, and the ratio of the
two medians over those same blocks; history off for step_s and wall, history on for wall.

Cost: Colab compute units at RATE, the session's runtime rate from `tools/colab/colab_api.py usage` (--rate): the measured run, the
fixed part, the marginal part per simulated day and one forecast day (the runtime costs the same whichever
processor a run uses, so for WRF this is the co-located price: an upper bound, the GPU idles). --cpu-rate: the rate
of a CPU-only runtime with the same CPU, as a STATED ASSUMPTION (printed as such): WRF's and CPPWRF CPU's cost
there. Runs are flagged (not dropped) for load (processes running just before), thermal / hardware GPU slowdown,
CPU steal time above 1 %, a system clock step, a cool-down that ended at its limit. --md: the Markdown report;
--json: the summary for tools/bench/bench_plot.py; --csv: one line per run.
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
METRICS = ("wall", "step_s", "steady", "step1", "stepping", "io", "io_steady", "input_in", "other", "marginal", "fixed",
           "energy_j", "energy_steady_j", "power_mean_w", "power_steady_w", "sm_clock_median_mhz")
GPU_KEYS = ("energy_j", "energy_steady_j", "power_mean_w", "power_steady_w", "sm_clock_median_mhz")
# with history on, a CUDA build's write waits for the steps queued before it: its split between the steps and the
# writes is not published (their sum, the marginal and fixed parts and the wall time are)
SPLIT = ("step_s", "steady", "step1", "stepping", "io", "io_steady")
BOOT, SEED = 10000, 20261121


def masked(r, k):
    return r.get("regime") == "on" and r.get("kind") == "gpu" and k in SPLIT


def boot_ci(v, n=BOOT, q=(2.5, 97.5)):
    """Percentile bootstrap interval of the median (n resamples, a fixed seed); None below 3 values."""
    v = [x for x in v if x is not None]
    if len(v) < 3:
        return None
    rng = np.random.default_rng(SEED)
    a = np.asarray(v, dtype=float)
    med = np.median(a[rng.integers(0, len(a), size=(n, len(a)))], axis=1)
    lo, hi = np.percentile(med, q)
    return [float(lo), float(hi)]


def stats(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return None
    a = np.asarray(v, dtype=float)
    med = float(np.median(a))
    return dict(n=len(v), median=med, q25=float(np.quantile(a, 0.25)), q75=float(np.quantile(a, 0.75)),
                min=float(a.min()), max=float(a.max()), ci95=boot_ci(v),
                cv=float(a.std(ddof=1) / a.mean()) if len(v) > 1 and a.mean() else None)


def value(r, k):
    if masked(r, k):
        return None
    if k == "wall":
        return r.get("wall")
    if k in GPU_KEYS:
        return (r.get("gpu") or {}).get(k)
    return (r.get("timing") or {}).get(k)


def pos(x):
    return x is not None and x > 0


def load(path):
    return [json.loads(x) for x in open(path) if x.strip()] if path and os.path.exists(path) else []


def summarize(runs, verify, cases_info=None, best=None, manifest=None, rate=None, cpu_rate=None):
    timed = [r for r in runs if r.get("phase") == "timed" and not r.get("warmup") and r.get("regime") in ("off", "on")]
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
            dedup, seen_dup, last_idx = [], 0, {}
            for r in rs:
                bk = r.get("block")
                if bk is not None:
                    key = (r["model"], bk)
                    if key in last_idx:
                        dedup[last_idx[key]] = None
                        seen_dup += 1
                    last_idx[key] = len(dedup)
                dedup.append(r)
            if seen_dup:
                print(f"warning: {c} {reg}: {seen_dup} duplicate (model, block) record(s); kept latest", file=sys.stderr)
            rs = [r for r in dedup if r is not None]
            models = sorted({r["model"] for r in rs}, key=lambda m: ORDER.index(m) if m in ORDER else 99)
            for m in models:
                g = [r for r in rs if r["model"] == m]
                st = {k: stats([value(r, k) for r in g]) for k in METRICS}
                r0 = g[0]
                sim = r0.get("sim_seconds") or 3600
                cs = r0.get("grid") or [[0, 1]]
                n1 = cs[0][1] if cs and cs[0][1] else 1
                cells_per_step = sum(a * b for a, b in cs) / n1  # all domains' cell updates per domain-1 step
                med = lambda k: st[k]["median"] if st.get(k) else None  # noqa: E731
                d, day = {}, 86400 / sim
                w, ss, mg, fx = med("wall"), med("step_s"), med("marginal"), med("fixed")
                if w:
                    d.update(sim_speed_wall=sim / w, sypd_wall=sim / (365.25 * w))
                if ss:  # the steady rate: seconds per step, scaled to a simulated day
                    d.update(s_per_sim_day_steady=ss * n1 * day, sypd_steady=sim / (365.25 * ss * n1),
                             ns_cell_step=ss * 1e9 / cells_per_step if cells_per_step else None)
                if mg is not None and fx is not None:  # a one-day forecast: the fixed part once + a day's marginal part
                    d.update(s_marginal_per_sim_day=mg * day, s_fixed=fx, s_per_forecast_day=fx + mg * day)
                et = med("energy_j")
                mg_e, fixed_e = [], []
                for r in g:
                    gp, tm = r.get("gpu") or {}, r.get("timing") or {}
                    r_ps, r_mg = gp.get("power_steady_w"), tm.get("marginal")
                    if r_ps is None or r_mg is None:
                        continue
                    r_post = max(0.0, tm.get("io_post") or 0.0)
                    r_pi = gp.get("power_idle_w") or 0.0
                    # marginal = compute + io_steady + io_post + input_steady; power_steady_w spans steps 2..N
                    # (including io_steady), while io_post is a CPU-only write after GPU work finishes
                    e_m = r_ps * max(0.0, r_mg - r_post) + r_pi * r_post
                    mg_e.append(e_m)
                    if gp.get("energy_j") is not None:
                        fixed_e.append(gp["energy_j"] - e_m)
                if mg_e:
                    d["gpu_wh_marginal_per_sim_day"] = float(np.median(mg_e)) * day / 3600
                    if fixed_e:
                        d["gpu_wh_fixed"] = float(np.median(fixed_e)) / 3600
                        d["gpu_wh_per_forecast_day"] = d["gpu_wh_fixed"] + d["gpu_wh_marginal_per_sim_day"]
                if et is not None:
                    d["gpu_wh_run"] = et / 3600
                cpu_only = m.startswith("wrf") or m.startswith("cpu")
                for key, rt in (("cu", rate), ("cu_cpu_runtime", cpu_rate if cpu_only else None)):
                    if not rt:
                        continue
                    if w:
                        d[f"{key}_run"] = w / 3600 * rt
                    if "s_per_forecast_day" in d:
                        d[f"{key}_marginal_per_sim_day"] = d["s_marginal_per_sim_day"] / 3600 * rt
                        d[f"{key}_fixed"] = fx / 3600 * rt
                        d[f"{key}_per_forecast_day"] = d["s_per_forecast_day"] / 3600 * rt
                gpu_s = [r.get("gpu") or {} for r in g]
                flags = dict(loaded=sum(1 for r in g if (r.get("load") or 0) > 0),
                             throttled=sum(1 for x in gpu_s if (x.get("throttled_samples") or 0) > 0),
                             power_capped=sum(1 for x in gpu_s if (x.get("power_cap_samples") or 0) > 0),
                             steal=sum(1 for r in g if ((r.get("cpu_share") or {}).get("steal_pct") or 0) > 1.0),
                             clock_step=sum(1 for r in g if abs(r.get("clock_step_s") or 0) > 0.5),
                             cool_capped=sum(1 for r in g if (r.get("cool") or {}).get("temp_end_c") is not None and
                                             (r.get("cool") or {}).get("target_c") is not None and
                                             r["cool"]["temp_end_c"] > r["cool"]["target_c"]))
                # the run with the median wall time (the lower middle one of an even count): its parts add up to
                # its wall time (medians of the parts would not), for the breakdown figure
                gw = sorted((r for r in g if r.get("wall") is not None), key=lambda r: r["wall"])
                rep_run = None
                if gw:
                    rr = gw[(len(gw) - 1) // 2]
                    t = rr.get("timing") or {}
                    rep_run = dict(id=rr.get("id"), block=rr.get("block"), wall=rr["wall"],
                                   parts={k: t.get(k) for k in ("other", "input_in", "step1", "steady", "io")})
                groups.append(dict(case=c, regime=reg, model=m, kind=r0.get("kind"), layout=r0.get("layout"), n=len(g),
                                   rep_run=rep_run,
                                   sim_seconds=sim, grid=cs, stats=st, derived=d, flags=flags,
                                   steal_pct_median=(stats([(r.get("cpu_share") or {}).get("steal_pct") for r in g])
                                                     or {}).get("median")))
            for base in ("wrf_r4", "wrf_r4t", "wrf_r8"):
                b = {r["block"]: r for r in rs if r["model"] == base and r.get("block") is not None}
                if not b:
                    continue
                for m in models:
                    if m.startswith("wrf"):
                        continue
                    bm = {r["block"]: r for r in rs if r["model"] == m and r.get("block") is not None}
                    for k in ("step_s", "wall"):
                        pr = [(bk, value(b[bk], k), value(bm[bk], k)) for bk in sorted(b) if bk in bm]
                        pr = [(bk, x, y) for bk, x, y in pr if pos(x) and pos(y)]  # the blocks both models ran in
                        if not pr:
                            continue
                        ratios = [x / y for _, x, y in pr]
                        # the ratio of the two medians over the same (matched) blocks
                        mr = float(np.median([x for _, x, _ in pr]) / np.median([y for _, _, y in pr]))
                        speed.append(dict(case=c, regime=reg, model=m, vs=base, metric=k, n=len(ratios),
                                          median=float(np.median(ratios)), min=min(ratios), max=max(ratios),
                                          ci95=boot_ci(ratios), ratio_of_medians=mr,
                                          blocks=[bk for bk, _, _ in pr]))
    lay = {}
    for r in runs:
        if r.get("phase") == "screen" and r.get("ok") and not r.get("warmup"):
            v = (r.get("timing") or {}).get("step_s")
            if v:
                lay.setdefault((r["case"], r["model"], r.get("layout")), []).append(v)
    screen = [dict(case=c, model=m, layout=layo, n=len(v), median=float(np.median(v)), min=min(v), max=max(v))
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


def scale(s, k):
    if not s:
        return s
    return dict(s, **{x: (s[x] * k if s.get(x) is not None else None) for x in ("median", "q25", "q75", "min", "max")})


def cell(s, n=2):
    """median [q25 - q75] (min - max)"""
    if not s:
        return "-"
    if s["n"] == 1:
        return f(s["median"], n)
    return f"{f(s['median'], n)} [{f(s['q25'], n)}-{f(s['q75'], n)}] ({f(s['min'], n)}-{f(s['max'], n)})"


def markdown(S, accuracy_md=None, md_path=None):
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
            w("CPPWRF CPU thread counts (screening runs, history off), ms per step, median (min-max): " + ", ".join(
                f"{x['layout']} {1e3 * x['median']:.1f} ({1e3 * x['min']:.1f}-{1e3 * x['max']:.1f})"
                for x in sorted(cpu, key=lambda x: x["layout"])) + ".\n")
        scr = [x for x in S["screen"] if x["case"] == c and x["model"].startswith("wrf")]
        if scr:
            w("WRF process layouts (screening runs after a warm-up run, history off), ms per step (steady), median "
              "(min-max) of the screening runs; bold: the chosen layout:\n")
            lays = sorted({x["layout"] for x in scr}, key=lambda s: (s[:2], int(s[2:].split("x")[0]) if s[2:].split("x")[0].isdigit() else 0, s))
            mods = sorted({x["model"] for x in scr})
            w("| layout | " + " | ".join(NAMES.get(m, m) for m in mods) + " |")
            w("|---|" + "---|" * len(mods))
            for la in lays:
                row = []
                for m in mods:
                    v = next((x for x in scr if x["layout"] == la and x["model"] == m), None)
                    b = S["best"].get(f"{c}:{m}")
                    t = "-" if not v else f"{1e3 * v['median']:.1f} ({1e3 * v['min']:.1f}-{1e3 * v['max']:.1f})"
                    row.append(f"**{t}**" if v and b == la else t)
                w(f"| {la} | " + " | ".join(row) + " |")
            w("")
        for reg, title in (("off", "History off (the frame at t = 0 only): the primary comparison"),
                           ("on", "History on")):
            gs = [g for g in S["groups"] if g["case"] == c and g["regime"] == reg]
            if not gs:
                continue
            w(f"### {title}\n")
            if reg == "on":
                w("With history on, a GPU run's write waits for the steps queued before it (the steps run "
                  "asynchronously): for the CUDA builds the split between the steps and the writes is not published "
                  "(\"n/p\"); the wall time, the marginal and the fixed parts are. The speed-ups are of the wall "
                  "time.\n")
            w("| model | layout | n | wall | ms / step (steady) | step 1 | I/O | input in steps | other | fixed | "
              "marginal | ns / cell / step | SYPD (wall / steady) |")
            w("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
            for g in gs:
                s, d = g["stats"], g["derived"]
                mk = reg == "on" and g.get("kind") == "gpu"
                np_ = lambda x: "n/p" if mk else x  # noqa: E731
                w(f"| {NAMES.get(g['model'], g['model'])} | {g['layout']} | {g['n']} | {cell(s['wall'], 1)} | "
                  f"{np_(cell(scale(s['step_s'], 1e3), 1))} | {np_(cell(s['step1']))} | {np_(cell(s['io']))} | "
                  f"{cell(s['input_in'])} | {cell(s['other'])} | {cell(s['fixed'], 1)} | {cell(s['marginal'], 1)} | "
                  f"{np_(f(d.get('ns_cell_step'), 1))} | {f(d.get('sypd_wall'), 3)} / {np_(f(d.get('sypd_steady'), 3))} |")
            w("")
            sp = [x for x in S["speedups"] if x["case"] == c and x["regime"] == reg]
            if sp:
                w("Speed-up (WRF time / CPPWRF time of the same block): median of the block ratios [95 % bootstrap "
                  "interval of that median] (smallest - largest block ratio); ratio of the medians over the same "
                  "blocks. Steady: seconds per step after step 1 (the headline compute metric):\n")
                w("| model | vs | n | steady | wall | ratio of medians (steady / wall) |")
                w("|---|---|---|---|---|---|")
                for m in sorted({x["model"] for x in sp}, key=lambda m: ORDER.index(m) if m in ORDER else 99):
                    for vs in ("wrf_r4", "wrf_r4t", "wrf_r8"):
                        xs = {k: next((y for y in sp if y["model"] == m and y["vs"] == vs and y["metric"] == k), None)
                              for k in ("step_s", "wall")}
                        if not any(xs.values()):
                            continue
                        row = []
                        for k in ("step_s", "wall"):
                            x = xs[k]
                            cis = f" [{x['ci95'][0]:.2f}-{x['ci95'][1]:.2f}]" if x and x.get("ci95") else ""
                            row.append(f"{x['median']:.2f}x{cis} ({x['min']:.2f}-{x['max']:.2f})" if x else "-")
                        n = max(x["n"] for x in xs.values() if x)
                        rm = " / ".join(f"{xs[k]['ratio_of_medians']:.2f}x" if xs[k] else "-" for k in ("step_s", "wall"))
                        w(f"| {NAMES.get(m, m)} | {NAMES[vs]} | {n} | " + " | ".join(row) + f" | {rm} |")
                w("")
            fd = [g for g in gs if "s_per_forecast_day" in g["derived"]]
            if fd:
                w("One simulated day (a fixed part once per run + the marginal part scaled to 24 h; seconds, from the "
                  "medians):\n")
                w("| model | fixed s | marginal s / sim. day | s / forecast day | measured run s |")
                w("|---|---|---|---|---|")
                for g in fd:
                    d = g["derived"]
                    w(f"| {NAMES.get(g['model'], g['model'])} | {f(d['s_fixed'], 1)} | {f(d['s_marginal_per_sim_day'], 0)} | "
                      f"{f(d['s_per_forecast_day'], 0)} | {f(g['stats']['wall']['median'] if g['stats']['wall'] else None, 1)} |")
                w("")
            eg = [g for g in gs if g["stats"]["energy_j"]]
            if eg:
                w("GPU board energy (nvidia-smi power.draw integrated; the board only, not the host). Steady power: over "
                  "the steady steps (domain 1's first to last timing line); per simulated day = the fixed energy once + "
                  "the marginal energy of a day (steady power over marginal - io_post, plus idle power over the "
                  "post-step CPU write io_post):\n")
                w("| model | run energy J | steady power W | SM clock MHz | Wh fixed | Wh marginal / sim. day | "
                  "Wh / forecast day | throttled runs | power-capped runs |")
                w("|---|---|---|---|---|---|---|---|---|")
                for g in eg:
                    s, d = g["stats"], g["derived"]
                    w(f"| {NAMES.get(g['model'], g['model'])} | {cell(s['energy_j'], 0)} | {cell(s['power_steady_w'], 0)} | "
                      f"{cell(s['sm_clock_median_mhz'], 0)} | {f(d.get('gpu_wh_fixed'), 2)} | "
                      f"{f(d.get('gpu_wh_marginal_per_sim_day'), 1)} | {f(d.get('gpu_wh_per_forecast_day'), 1)} | "
                      f"{g['flags']['throttled']} | {g['flags']['power_capped']} |")
                w("")
    if S.get("rate"):
        w("## Cost\n")
        w(f"Colab compute units at the session's rate, {S['rate']} units per hour (`tools/colab/colab_api.py usage`): the measured run, "
          "and one simulated day as the fixed part once plus the marginal part of 24 h. The runtime costs the same "
          "whichever processor a run uses: for WRF this is the price on the GPU runtime with the GPU idle, an upper "
          "bound, not a CPU-only price." + (
              f" The CPU-only columns assume {S['cpu_rate']} units per hour for a CPU-only runtime with the same CPU: "
              "a stated assumption, not a measurement." if S.get("cpu_rate") else
              " A CPU-only runtime's price was not given (--cpu-rate) and is not shown.") + "\n")
        w("| case | history | model | CU run | CU fixed | CU marginal / sim. day | CU / forecast day | "
          "CPU-only CU / forecast day |")
        w("|---|---|---|---|---|---|---|---|")
        for g in S["groups"]:
            d = g["derived"]
            if "cu_run" in d:
                w(f"| {g['case']} | {g['regime']} | {NAMES.get(g['model'], g['model'])} | {f(d['cu_run'], 3)} | "
                  f"{f(d.get('cu_fixed'), 3)} | {f(d.get('cu_marginal_per_sim_day'), 3)} | "
                  f"{f(d.get('cu_per_forecast_day'), 3)} | {f(d.get('cu_cpu_runtime_per_forecast_day'), 3)} |")
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
                vi = x.get("values_identical", x["identical"])
                parts.append(f"{d}: {x['frames']} frames " + ("**identical**" if x["identical"] else
                             "values identical, metadata differ (" + str(x["differ"].get("(metadata)"))[:160] + ")"
                             if vi else "DIFFER (" + ", ".join(sorted(x["differ"])[:8]) + ")"))
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
        acc_ref = (os.path.relpath(os.path.abspath(accuracy_md), os.path.dirname(os.path.abspath(md_path))).replace(os.sep, "/")
                   if md_path else os.path.basename(accuracy_md))
        w(f"\nThe FP32 accuracy ratio R per variable and hour: [{acc_ref}]({acc_ref}) "
          "(tools/verify/fp32_accuracy.py report).")
    if S["failed"]:
        w("\n## Failed runs\n")
        for x in S["failed"]:
            w(f"- {x['case']} {x['phase']} {x['model']} {x.get('layout') or ''} {x.get('regime') or ''} block "
              f"{x.get('block')}: {x['error']}")
    fl = [g for g in S["groups"] if any(v for k, v in g["flags"].items() if k != "power_capped")]
    if fl:
        w("\n## Flagged repetitions\n")
        w("Flagged, not dropped: runs started with other processes running, with thermal / hardware GPU slowdown "
          "samples, with more than 1 % CPU steal time, with a system clock step, or whose cool-down ended at its "
          "limit above the target temperature.\n")
        for g in fl:
            fg = g["flags"]
            w(f"- {g['case']} {g['regime']} {g['model']}: {fg['loaded']} loaded, {fg['throttled']} throttled, "
              f"{fg['steal']} steal, {fg['clock_step']} clock step, {fg['cool_capped']} cool-down at its limit")
    return "\n".join(o) + "\n"


def write_csv(runs, path):
    cols = ["id", "phase", "case", "model", "build", "kind", "layout", "regime", "block", "pos", "prev", "warmup", "ok",
            "exit", "wall", "step_s", "steady", "steady_steps", "step1", "stepping", "io", "io_steady", "input_in",
            "other", "marginal", "fixed", "steps", "load", "cpu_mhz", "steal_pct", "energy_j", "energy_steady_j",
            "power_mean_w", "power_steady_w", "power_peak_w", "power_idle_w", "sm_clock_median_mhz", "temp_max_c",
            "mem_used_peak_mib", "throttled_samples", "power_cap_samples", "cool_wait_s", "clock_step_s"]
    with open(path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(cols)
        for r in runs:
            t, g = r.get("timing") or {}, r.get("gpu") or {}
            x = dict(steal_pct=(r.get("cpu_share") or {}).get("steal_pct"), cool_wait_s=(r.get("cool") or {}).get("wait_s"))
            wr.writerow([r.get(k) if k in r else x[k] if k in x else t.get(k, g.get(k)) for k in cols])


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
    md = markdown(S, opt("--accuracy-md"), opt("--md"))
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
