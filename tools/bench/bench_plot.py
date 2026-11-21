#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""SVG figures of a paper benchmark session from tools/bench/bench_stats.py's summary (plain SVG, no plotting
library: the same output on any machine).

  bench_plot.py SUMMARY.json --out DIR
  bench_plot.py --help

Writes into DIR:
- speedup_steady_vs_wrf_r4.svg, ..._wrf_r4t.svg, ..._wrf_r8.svg: the speed-up of the steady steps (seconds per
  step after step 1, history off: the same work in both models) of each CPPWRF build against WRF real*4 / tuned
  real*4 / real*8, per case: the median block ratio, whiskers its 95 % bootstrap interval (the smallest to the
  largest block ratio below 3 blocks);
- speedup_wall_vs_wrf_*_off.svg, speedup_wall_vs_wrf_*_on.svg: the same for the wall time, history off and on;
- breakdown_off.svg, breakdown_on.svg: the run with the median wall time of each model, split into other (start,
  input, initialisation, finish), input read inside the steps, step 1, the steady steps and the history output
  (bench_parse.py's split, which adds up to the wall time); with history on the CUDA builds' steps and output are
  one bar ("steps + output": their split is not meaningful);
- energy.svg: the GPU board energy of one simulated day (the fixed part + 24 h at the steady power, Wh), per case;
- layouts.svg: WRF's screened layouts, seconds per step relative to each case's and precision's fastest layout.
A figure without data is not written.
"""
import json
import os
import sys

COL = {"gpu_fp32": "#2f6fb0", "gpu_fp32id": "#7a5cc2", "gpu_fp64": "#c0504d", "cpu_fp64": "#6b8e23",
       "gpu_fp32_nc0": "#5aa6a6", "wrf_r4": "#8c8c8c", "wrf_r4t": "#a6a6a6", "wrf_r8": "#4d4d4d"}
PART = [("other", "#bdbdbd", "other (start, input, init, finish)"), ("input_in", "#8e6bb0", "input in steps"),
        ("step1", "#e6a03c", "step 1"), ("steady", "#2f6fb0", "steady steps"), ("io", "#6b8e23", "history output"),
        ("steps_io", "#5a8fa0", "steps + output (GPU, history on)")]
NAMES = {"wrf_r4": "WRF real*4", "wrf_r8": "WRF real*8", "wrf_r4t": "WRF real*4 tuned", "gpu_fp32": "GPU FP32 (fast)", "gpu_fp32id": "GPU FP32 identity",
         "gpu_fp64": "GPU FP64", "cpu_fp64": "CPU FP64", "gpu_fp32_nc0": "GPU FP32, HDF5 deflate"}
ORDER = ["wrf_r4", "wrf_r4t", "wrf_r8", "gpu_fp32", "gpu_fp32id", "gpu_fp64", "cpu_fp64", "gpu_fp32_nc0"]


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def svg(w, h, body, title):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
            f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n<rect width="100%" height="100%" fill="#ffffff"/>\n'
            f'<text x="{w / 2}" y="20" text-anchor="middle" font-size="14" font-weight="bold">{esc(title)}</text>\n'
            + "\n".join(body) + "\n</svg>\n")


def nice_max(v):
    import math
    if v <= 0:
        return 1
    e = 10 ** math.floor(math.log10(v))
    for m in (1, 2, 2.5, 5, 10):
        if v <= m * e:
            return m * e
    return 10 * e


def bars(groups, series, values, title, ylabel, errs=None, ref_line=None):
    """Grouped vertical bars: values[g][s] (None: no bar), errs[g][s] = (lo, hi)."""
    W, H, L, R, T, B = max(520, 90 + len(groups) * (len(series) * 22 + 30)), 380, 70, 20, 40, 90
    vmax = max([v for row in values for v in row if v is not None] +
               [e[1] for row in (errs or []) for e in row if e] + [ref_line or 0] + [0])
    ym = nice_max(vmax * 1.05)
    pw, ph = W - L - R, H - T - B
    gw = pw / max(1, len(groups))
    bw = min(22, (gw - 20) / max(1, len(series)))
    y = lambda v: T + ph - v / ym * ph  # noqa: E731
    out = [f'<line x1="{L}" y1="{T + ph}" x2="{W - R}" y2="{T + ph}" stroke="#333"/>',
           f'<line x1="{L}" y1="{T}" x2="{L}" y2="{T + ph}" stroke="#333"/>',
           f'<text x="16" y="{T + ph / 2}" transform="rotate(-90 16 {T + ph / 2})" text-anchor="middle">{esc(ylabel)}</text>']
    for i in range(6):
        v = ym * i / 5
        out.append(f'<line x1="{L - 4}" y1="{y(v):.1f}" x2="{W - R}" y2="{y(v):.1f}" stroke="#e5e5e5"/>')
        out.append(f'<text x="{L - 6}" y="{y(v) + 4:.1f}" text-anchor="end">{v:g}</text>')
    if ref_line:
        out.append(f'<line x1="{L}" y1="{y(ref_line):.1f}" x2="{W - R}" y2="{y(ref_line):.1f}" stroke="#999" '
                   f'stroke-dasharray="4 3"/>')
    for gi, gname in enumerate(groups):
        x0 = L + gi * gw + (gw - bw * len(series)) / 2
        out.append(f'<text x="{L + gi * gw + gw / 2:.1f}" y="{T + ph + 16}" text-anchor="middle">{esc(gname)}</text>')
        for si, (sname, color) in enumerate(series):
            v = values[gi][si]
            if v is None:
                continue
            x = x0 + si * bw
            out.append(f'<rect x="{x:.1f}" y="{y(v):.1f}" width="{bw - 2:.1f}" height="{T + ph - y(v):.1f}" '
                       f'fill="{color}"><title>{esc(sname)} {esc(gname)}: {v:.3g}</title></rect>')
            e = errs[gi][si] if errs else None
            if e:
                cx = x + (bw - 2) / 2
                out.append(f'<line x1="{cx:.1f}" y1="{y(e[0]):.1f}" x2="{cx:.1f}" y2="{y(e[1]):.1f}" stroke="#222"/>')
                for yy in (e[0], e[1]):
                    out.append(f'<line x1="{cx - 3:.1f}" y1="{y(yy):.1f}" x2="{cx + 3:.1f}" y2="{y(yy):.1f}" stroke="#222"/>')
    lx, ly = L, H - 40
    for sname, color in series:
        out.append(f'<rect x="{lx}" y="{ly}" width="10" height="10" fill="{color}"/>'
                   f'<text x="{lx + 14}" y="{ly + 9}">{esc(sname)}</text>')
        lx += 22 + 7 * len(sname)
        if lx > W - 120:
            lx, ly = L, ly + 16
    return svg(W, H, out, title)


def stacked(rows, title):
    """Horizontal stacked bars: rows [(label, {part: seconds})]."""
    W, rh, L, R, T = 760, 18, 230, 30, 40
    used = [p for p in PART if any((parts.get(p[0]) or 0) > 0 for _, parts in rows)]
    H = T + len(rows) * (rh + 6) + 70 + 16 * (len(used) // 3)
    vmax = max([sum(v for v in parts.values() if v) for _, parts in rows] + [0])
    xm = nice_max(vmax * 1.02)
    pw = W - L - R
    x = lambda v: L + v / xm * pw  # noqa: E731
    out = []
    for i in range(6):
        v = xm * i / 5
        out.append(f'<line x1="{x(v):.1f}" y1="{T - 4}" x2="{x(v):.1f}" y2="{T + len(rows) * (rh + 6)}" stroke="#e5e5e5"/>'
                   f'<text x="{x(v):.1f}" y="{T + len(rows) * (rh + 6) + 14}" text-anchor="middle">{v:g}</text>')
    out.append(f'<text x="{L + pw / 2}" y="{T + len(rows) * (rh + 6) + 30}" text-anchor="middle">seconds (median)</text>')
    for i, (label, parts) in enumerate(rows):
        yy = T + i * (rh + 6)
        out.append(f'<text x="{L - 6}" y="{yy + 13}" text-anchor="end">{esc(label)}</text>')
        acc = 0.0
        for k, color, name in PART:
            v = parts.get(k) or 0.0
            if v <= 0:  # a negative part (a clock mismatch) is not drawn; the report's tables show it
                continue
            out.append(f'<rect x="{x(acc):.1f}" y="{yy}" width="{x(acc + v) - x(acc):.1f}" height="{rh}" fill="{color}">'
                       f'<title>{esc(label)} {name}: {v:.2f} s</title></rect>')
            acc += v
    lx, ly = 20, T + len(rows) * (rh + 6) + 44
    for k, color, name in used:
        if lx + 30 + 7 * len(name) > W:
            lx, ly = 20, ly + 16
        out.append(f'<rect x="{lx}" y="{ly}" width="10" height="10" fill="{color}"/><text x="{lx + 14}" y="{ly + 9}">{name}</text>')
        lx += 30 + 7 * len(name)
    return svg(W, H, out, title)


def med(g, k):
    s = g["stats"].get(k)
    return s["median"] if s else None


def main():
    a = sys.argv[1:]
    if len(a) < 1 or a[0] in ("-h", "--help") or "--out" not in a:
        print(__doc__)
        sys.exit(0 if a and a[0] in ("-h", "--help") else 2)
    S = json.load(open(a[0]))
    out = a[a.index("--out") + 1]
    os.makedirs(out, exist_ok=True)
    written = []
    cases = S["cases"]
    for metric, reg in (("step_s", "off"), ("wall", "off"), ("wall", "on")):
        for vs in ("wrf_r4", "wrf_r4t", "wrf_r8"):
            sp = [x for x in S["speedups"] if x["vs"] == vs and x["regime"] == reg and x["metric"] == metric]
            models = sorted({x["model"] for x in sp}, key=lambda m: ORDER.index(m) if m in ORDER else 99)
            if not sp:
                continue
            cs = [c for c in cases if any(x["case"] == c for x in sp)]
            vals, errs = [], []
            for c in cs:
                row, er = [], []
                for m in models:
                    x = next((y for y in sp if y["case"] == c and y["model"] == m), None)
                    row.append(x["median"] if x else None)
                    er.append((tuple(x["ci95"]) if x.get("ci95") else (x["min"], x["max"])) if x else None)
                vals.append(row)
                errs.append(er)
            name = f"speedup_steady_vs_{vs}.svg" if metric == "step_s" else f"speedup_wall_vs_{vs}_{reg}.svg"
            what = "Steady-step" if metric == "step_s" else "Wall-time"
            p = os.path.join(out, name)
            open(p, "w").write(bars(cs, [(NAMES.get(m, m), COL.get(m, "#777")) for m in models], vals,
                                    f"{what} speed-up against {NAMES[vs]} (history {reg})", "speed-up (x)", errs, 1.0))
            written.append(p)
    for reg in ("off", "on"):
        rows = []
        for c in cases:
            for g in sorted((g for g in S["groups"] if g["case"] == c and g["regime"] == reg and g.get("rep_run")),
                            key=lambda g: ORDER.index(g["model"]) if g["model"] in ORDER else 99):
                parts = dict(g["rep_run"]["parts"])
                if reg == "on" and g.get("kind") == "gpu":
                    parts = dict(other=parts.get("other"), input_in=parts.get("input_in"),
                                 steps_io=sum(parts.get(k) or 0.0 for k in ("step1", "steady", "io")))
                rows.append((f"{c} {NAMES.get(g['model'], g['model'])}", parts))
        if rows:
            p = os.path.join(out, f"breakdown_{reg}.svg")
            open(p, "w").write(stacked(rows, f"Where the wall time goes: the median run (history {reg})"))
            written.append(p)
    eg = [g for g in S["groups"] if g["regime"] == "off" and g["derived"].get("gpu_wh_per_forecast_day") is not None]
    if eg:
        models = sorted({g["model"] for g in eg}, key=lambda m: ORDER.index(m) if m in ORDER else 99)
        cs = [c for c in cases if any(g["case"] == c for g in eg)]
        vals = [[next((g["derived"]["gpu_wh_per_forecast_day"] for g in eg if g["case"] == c and g["model"] == m),
                      None) for m in models] for c in cs]
        p = os.path.join(out, "energy.svg")
        open(p, "w").write(bars(cs, [(NAMES.get(m, m), COL.get(m, "#777")) for m in models], vals,
                                "GPU board energy of one simulated day (history off; the board only)",
                                "Wh per forecast day"))
        written.append(p)
    scr = [x for x in S.get("screen") or [] if x["model"].startswith("wrf")]
    if scr:
        cs = [c for c in cases if any(x["case"] == c for x in scr)]
        lays = sorted({x["layout"] for x in scr})
        series = []
        for c in cs:
            for m in sorted({x["model"] for x in scr if x["case"] == c}):
                series.append((c, m))
        fastest = {(c, m): min(x["median"] for x in scr if x["case"] == c and x["model"] == m) for c, m in series}
        vals = [[next((x["median"] / fastest[(c, m)] for x in scr if x["case"] == c and x["model"] == m and
                       x["layout"] == la), None) for (c, m) in series] for la in lays]
        palette = ["#8c8c8c", "#4d4d4d", "#2f6fb0", "#c0504d", "#6b8e23", "#7a5cc2", "#e6a03c", "#5aa6a6"]
        p = os.path.join(out, "layouts.svg")
        open(p, "w").write(bars(lays, [(f"{c} {NAMES.get(m, m)}", palette[i % len(palette)]) for i, (c, m) in
                                       enumerate(series)], vals, "WRF process layouts: seconds per step relative to the fastest",
                                "time / fastest layout's time", ref_line=1.0))
        written.append(p)
    print("\n".join(written))


if __name__ == "__main__":
    main()
