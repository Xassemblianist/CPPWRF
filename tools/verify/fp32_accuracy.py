#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Accuracy statistics of one set of WRF-format history files against another (numpy and netCDF4 only), and the
report that compares such statistics between pairs (docs/FP32_ACCURACY.md).

  fp32_accuracy.py compare TEST REF [--domain d01] [--case NAME] [--pair NAME] [--vars V,...] [--out FILE.jsonl]
  fp32_accuracy.py report STATS.jsonl... [--md FILE.md] [--svg-dir DIR] [--metric rmse|max_abs|rel_l2]
                   [--num PAIR] [--den PAIR] [--hours 1,3,6,12,24,48,72]

compare: TEST and REF are wrfout files or directories (a directory gives its wrfout_<domain>_* files, and those of
DIR/<domain>/, as CPPWRF writes a nest's). Frames are matched by their Times. For every common frame and variable
(the ones present in both sets) one JSON line: case, pair, domain, time, hours (since the first common frame), var,
n (points), max_abs (max |TEST - REF|), rmse, rel_l2 (||TEST - REF|| / ||REF||), bias (mean TEST - REF), ref_rms.
Variables: the surface fields T2, Q2, U10, V10, PSFC, TSK, SWDOWN, HFX, LH, PBLH; RAIN = RAINNC + RAINC (accumulated
since the start) and RAIN1H (rain in the hour before the frame, when the frame an hour earlier is in both sets); the
3-D fields T, QVAPOR, U, V, W, PH and the hydrometeor mixing ratios and numbers present (QCLOUD, QRAIN, QICE, QSNOW,
QGRAUP, QHAIL, QNCLOUD, QNRAIN, QNICE, QNSNOW, QNGRAUPEL, QNHAIL). Rain categories: for RAIN1H at 0.1, 1 and 5 mm
and for RAIN at 1 and 10 mm the variable "<var>>=<th>mm" with frac (fraction of the points whose category, rain >=
th or not, differs between TEST and REF), wet_ref, wet_test (fractions of rainy points). Values are compared as
float64 (a real*4 file's numbers converted exactly). Exit 1 when the sets have no common frame.

report: reads the JSON lines of several pairs (e.g. "r4-r8": WRF real*4 against WRF real*8, "fp32-r8": CPPWRF FP32
against WRF real*8) and writes markdown tables (one per case and domain at the first hour and, for runs of more than
one hour, at --hours) of the --metric of each pair and the ratio R = metric(--num) / metric(--den) (default
fp32-r8 over r4-r8), a summary table (per case: the median and the largest R over the variables) and with --svg-dir
one SVG figure per case and domain of more than one hour (the metric over time, log scale, of six variables for every
pair present).
"""
import argparse
import datetime
import glob
import json
import math
import os
import sys

import netCDF4
import numpy as np

SURF = ["T2", "Q2", "U10", "V10", "PSFC", "TSK", "SWDOWN", "HFX", "LH", "PBLH"]
ATM = ["T", "QVAPOR", "U", "V", "W", "PH"]
HYD = ["QCLOUD", "QRAIN", "QICE", "QSNOW", "QGRAUP", "QHAIL", "QNCLOUD", "QNRAIN", "QNICE", "QNSNOW", "QNGRAUPEL",
       "QNHAIL"]
RAIN1H_TH = (0.1, 1.0, 5.0)
RAIN_TH = (1.0, 10.0)
ORDER = SURF + ["RAIN", "RAIN1H"] + ATM + HYD


def files(spec, dom):
    out = []
    for s in spec:
        if os.path.isdir(s):
            out += sorted(glob.glob(os.path.join(s, f"wrfout_{dom}_*")) + glob.glob(os.path.join(s, dom, f"wrfout_{dom}_*")))
        else:
            out.append(s)
    return out


def frames(paths):
    """{time string: (path, index)} of the files (a later file's frame replaces an earlier one's)."""
    out = {}
    for p in paths:
        with netCDF4.Dataset(p) as ds:
            t = np.asarray(ds["Times"][:])
            for i in range(t.shape[0]):
                out[b"".join(t[i].tolist()).decode()] = (p, i)
    return out


def tsec(s):
    return datetime.datetime.strptime(s, "%Y-%m-%d_%H:%M:%S").timestamp()


class Reader:
    """Variables of one frame, read once (float64; RAIN = RAINNC + RAINC)."""

    def __init__(self, where):
        self.p, self.i = where
        self.ds = netCDF4.Dataset(self.p)
        self.ds.set_auto_maskandscale(False)

    def has(self, v):
        if v == "RAIN":
            return "RAINNC" in self.ds.variables or "RAINC" in self.ds.variables
        return v in self.ds.variables

    def get(self, v):
        if v == "RAIN":
            r = 0.0
            for x in ("RAINNC", "RAINC"):
                if x in self.ds.variables:
                    r = r + np.asarray(self.ds[x][self.i], dtype=np.float64)
            return r
        return np.asarray(self.ds[v][self.i], dtype=np.float64)

    def close(self):
        self.ds.close()


def stats(a, b):
    d = a - b
    n = d.size
    nb = math.sqrt(float(np.sum(b * b)))
    nd = math.sqrt(float(np.sum(d * d)))
    return dict(n=int(n), max_abs=float(np.max(np.abs(d))) if n else 0.0, rmse=nd / math.sqrt(n) if n else 0.0,
                rel_l2=nd / nb if nb > 0 else (0.0 if nd == 0 else float("inf")), bias=float(np.mean(d)) if n else 0.0,
                ref_rms=nb / math.sqrt(n) if n else 0.0)


def categories(a, b, ths, var):
    out = []
    for th in ths:
        wa, wb = a >= th, b >= th
        out.append(dict(var=f"{var}>={th:g}mm", n=int(a.size), frac=float(np.mean(wa != wb)),
                        wet_ref=float(np.mean(wb)), wet_test=float(np.mean(wa))))
    return out


def compare(a):
    vars_ = a.vars.split(",") if a.vars else ORDER
    ft, fr = frames(files(a.test, a.domain)), frames(files(a.ref, a.domain))
    common = sorted(set(ft) & set(fr), key=tsec)
    if not common:
        print(f"no common frame ({len(ft)} test frames, {len(fr)} reference frames)", file=sys.stderr)
        return 1
    t0 = tsec(common[0])
    out = open(a.out, "a") if a.out else sys.stdout
    prev = {}  # time -> (test rain, reference rain)
    for t in common:
        rt, rr = Reader(ft[t]), Reader(fr[t])
        base = dict(case=a.case, pair=a.pair, domain=a.domain, time=t, hours=round((tsec(t) - t0) / 3600.0, 4))
        recs = []
        try:
            for v in vars_:
                if v == "RAIN1H":
                    continue
                if not (rt.has(v) and rr.has(v)):
                    continue
                x, y = rt.get(v), rr.get(v)
                if x.shape != y.shape:
                    recs.append(dict(var=v, error=f"shape {x.shape} vs {y.shape}"))
                    continue
                recs.append(dict(var=v, **stats(x, y)))
                if v == "RAIN":
                    recs += categories(x, y, RAIN_TH, "RAIN")
                    prev[t] = (x, y)
                    tp = datetime.datetime.strptime(t, "%Y-%m-%d_%H:%M:%S") - datetime.timedelta(hours=1)
                    p = prev.get(tp.strftime("%Y-%m-%d_%H:%M:%S"))
                    if p is not None and "RAIN1H" in vars_:
                        h1, h2 = x - p[0], y - p[1]
                        recs.append(dict(var="RAIN1H", **stats(h1, h2)))
                        recs += categories(h1, h2, RAIN1H_TH, "RAIN1H")
        finally:
            rt.close()
            rr.close()
        for r in recs:
            out.write(json.dumps({**base, **r}) + "\n")
        for k in [k for k in prev if tsec(k) < tsec(t) - 3600.5]:
            del prev[k]
    if a.out:
        out.close()
    return 0


# ---- report
def fmt(x):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "-"
    if x == 0:
        return "0"
    return f"{x:.2e}" if abs(x) < 1e-2 or abs(x) >= 1e4 else f"{x:.3g}"


def ratio(n, d):
    if n is None or d is None:
        return None
    if d == 0:
        return None if n == 0 else float("inf")
    return n / d


def fmt_r(r):
    if r is None:
        return "-"
    if r == float("inf"):
        return "inf"
    s = f"{r:.2f}"
    return f"**{s}**" if r > 1.1 else s


def load(paths):
    recs = []
    for p in paths:
        for line in open(p):
            line = line.strip()
            if line:
                recs.append(json.loads(line))
    return recs


def index(recs):
    """{(case, domain): {pair: {var: {hours: rec}}}}"""
    out = {}
    for r in recs:
        if "error" in r:
            continue
        out.setdefault((r["case"], r["domain"]), {}).setdefault(r["pair"], {}).setdefault(r["var"], {})[r["hours"]] = r
    return out


def var_key(v):
    b = v.split(">=")[0]
    return (ORDER.index(b) if b in ORDER else len(ORDER), v)


def nearest(d, h):
    if not d:
        return None
    k = min(d, key=lambda x: abs(x - h))
    return d[k] if abs(k - h) < 1e-3 else None


def svg(case, dom, pairs, metric, path, vars_=("T2", "U10", "RAIN1H", "T", "U", "QVAPOR")):
    """The metric over time of each pair, one panel per variable, log scale; a self-contained SVG with a light and a
    dark scheme (prefers-color-scheme)."""
    colors = {"r4-r8": "#1f77b4", "fp32-r8": "#d62728", "fp32-r4": "#7f7f7f", "r8n-r8": "#2ca02c", "fp64-r8": "#9467bd"}
    names = {"r4-r8": "WRF real*4 vs WRF real*8", "fp32-r8": "CPPWRF FP32 vs WRF real*8",
             "fp32-r4": "CPPWRF FP32 vs WRF real*4", "r8n-r8": "WRF real*8 (MPI, other CPU) vs WRF real*8"}
    vars_ = [v for v in vars_ if any(v in pairs.get(p, {}) for p in pairs)]
    if not vars_:
        return False
    W, H, cols = 300, 210, 3
    rows = (len(vars_) + cols - 1) // cols
    lw = 34
    tw, th = cols * W, rows * H + lw
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {tw} {th}" width="{tw}" height="{th}" '
         f'font-family="sans-serif" font-size="11">',
         "<style>.bg{fill:#fff}.fg{fill:#222}.ax{stroke:#888;fill:none}.gr{stroke:#ddd}"
         "@media (prefers-color-scheme: dark){.bg{fill:#1b1b1b}.fg{fill:#ddd}.ax{stroke:#999}.gr{stroke:#3a3a3a}}</style>",
         f'<rect class="bg" width="{tw}" height="{th}"/>']
    x0 = 10
    for p in pairs:
        if p not in colors:
            continue
        o.append(f'<line x1="{x0}" y1="14" x2="{x0 + 22}" y2="14" stroke="{colors[p]}" stroke-width="2"'
                 f'{" stroke-dasharray=\"4 3\"" if p in ("fp32-r4", "r8n-r8") else ""}/>')
        o.append(f'<text class="fg" x="{x0 + 26}" y="18">{names.get(p, p)}</text>')
        x0 += 34 + 6.2 * len(names.get(p, p))
    for k, v in enumerate(vars_):
        ox, oy = (k % cols) * W, lw + (k // cols) * H
        pl, pr, pt, pb = ox + 46, ox + W - 10, oy + 18, oy + H - 26
        series = {}
        for p in pairs:
            d = pairs[p].get(v, {})
            pts = sorted((h, r.get(metric)) for h, r in d.items() if r.get(metric) and r.get(metric) > 0)
            if pts and p in colors:
                series[p] = pts
        if not series:
            continue
        hs = [h for s in series.values() for h, _ in s]
        ys = [y for s in series.values() for _, y in s]
        hmax = max(hs) if max(hs) > 0 else 1.0
        lo, hi = math.floor(math.log10(min(ys))), math.ceil(math.log10(max(ys)))
        if hi == lo:
            hi += 1
        X = lambda h: pl + (pr - pl) * h / hmax  # noqa: E731
        Y = lambda y: pb - (pb - pt) * (math.log10(y) - lo) / (hi - lo)  # noqa: E731
        o.append(f'<text class="fg" x="{pl}" y="{oy + 12}" font-weight="bold">{v} ({metric})</text>')
        for e in range(lo, hi + 1):
            o.append(f'<line class="gr" x1="{pl}" y1="{Y(10 ** e):.1f}" x2="{pr}" y2="{Y(10 ** e):.1f}"/>')
            o.append(f'<text class="fg" x="{pl - 4}" y="{Y(10 ** e) + 4:.1f}" text-anchor="end">1e{e}</text>')
        step = 6 if hmax <= 24 else 12 if hmax <= 48 else 24
        if hmax <= 2:
            step = 0.5
        t = 0.0
        while t <= hmax + 1e-9:
            o.append(f'<text class="fg" x="{X(t):.1f}" y="{pb + 14}" text-anchor="middle">{t:g}</text>')
            t += step
        o.append(f'<text class="fg" x="{pr}" y="{pb + 24}" text-anchor="end">hours</text>')
        o.append(f'<rect class="ax" x="{pl}" y="{pt}" width="{pr - pl}" height="{pb - pt}"/>')
        for p, s in series.items():
            pts = " ".join(f"{X(h):.1f},{Y(y):.1f}" for h, y in s)
            dash = ' stroke-dasharray="4 3"' if p in ("fp32-r4", "r8n-r8") else ""
            o.append(f'<polyline fill="none" stroke="{colors[p]}" stroke-width="1.6"{dash} points="{pts}"/>')
    o.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(o) + "\n")
    return True


def report(a):
    idx = index(load(a.stats))
    hours = [float(h) for h in a.hours.split(",")]
    m = a.metric
    num, den = a.num, a.den
    lines = []
    summary = []
    for (case, dom) in sorted(idx, key=lambda k: (k[0], k[1])):
        pairs = idx[(case, dom)]
        if num not in pairs or den not in pairs:
            continue
        vs = sorted(set(pairs[num]) & set(pairs[den]), key=var_key)
        hmax = max(h for v in pairs[num].values() for h in v)
        hs = [h for h in hours if h <= hmax + 1e-6 and any(nearest(pairs[num][v], h) for v in vs)]
        others = [p for p in pairs if p not in (num, den)]
        # first hour: every variable, every pair
        h1 = hs[0] if hs else hmax
        lines.append(f"\n#### {case} {dom}: {m} at {h1:g} h\n")
        head = [den, num] + others
        lines.append("| Variable | " + " | ".join(head) + " | R |")
        lines.append("|---|" + "---|" * (len(head) + 1))
        rs = []
        for v in vs:
            if ">=" in v:
                continue
            vals = [nearest(pairs[p].get(v, {}), h1) for p in head]
            xs = [r.get(m) if r else None for r in vals]
            r = ratio(xs[1], xs[0])
            if r is not None and v not in ("RAIN", "RAIN1H"):
                rs.append((r, v))
            lines.append(f"| {v} | " + " | ".join(fmt(x) for x in xs) + f" | {fmt_r(r)} |")
        cats = [v for v in vs if ">=" in v]
        if cats:
            lines.append(f"\nRain categories at {h1:g} h (fraction of points whose category differs; "
                         "wet: fraction of rainy points in the reference):\n")
            lines.append("| Threshold | " + " | ".join(head) + " | wet |")
            lines.append("|---|" + "---|" * (len(head) + 1))
            for v in cats:
                vals = [nearest(pairs[p].get(v, {}), h1) for p in head]
                wet = next((x["wet_ref"] for x in vals if x), None)
                lines.append(f"| {v} | " + " | ".join(fmt(x["frac"]) if x else "-" for x in vals) + f" | {fmt(wet)} |")
        if len(hs) > 1:
            key = [v for v in ("T2", "Q2", "U10", "PSFC", "RAIN", "RAIN1H", "T", "QVAPOR", "U", "W", "QCLOUD", "QRAIN")
                   if v in vs]
            lines.append(f"\nGrowth: {m} of {den} / {num} (R) at each time\n")
            lines.append("| Variable | " + " | ".join(f"{h:g} h" for h in hs) + " |")
            lines.append("|---|" + "---|" * len(hs))
            for v in key:
                cells = []
                for h in hs:
                    x, y = nearest(pairs[den][v], h), nearest(pairs[num][v], h)
                    xs, ys = (x or {}).get(m), (y or {}).get(m)
                    cells.append(f"{fmt(xs)} / {fmt(ys)} ({fmt_r(ratio(ys, xs))})")
                lines.append(f"| {v} | " + " | ".join(cells) + " |")
            lt = hs[-1]
            rl = [(ratio((nearest(pairs[num][v], lt) or {}).get(m), (nearest(pairs[den][v], lt) or {}).get(m)), v)
                  for v in vs if ">=" not in v and v not in ("RAIN", "RAIN1H")]
            rl = [x for x in rl if x[0] is not None]
            if rl:
                summary.append((case, dom, lt, rl))
            if a.svg_dir:
                os.makedirs(a.svg_dir, exist_ok=True)
                name = f"fp32_growth_{case.lower()}_{dom}.svg"
                if svg(case, dom, pairs, m, os.path.join(a.svg_dir, name)):
                    lines.append(f"\n![{case} {dom}: {m} over time]({os.path.join(a.svg_rel or a.svg_dir, name)})\n")
        if rs:
            summary.append((case, dom, h1, rs))
    out = ["| Case | Domain | Time | median R | largest R (variable) | variables with R > 1.1 |",
           "|---|---|---|---|---|---|"]
    for case, dom, h, rs in sorted(summary, key=lambda x: (x[0], x[1], x[2])):
        rr = sorted(r for r, _ in rs)
        med = rr[len(rr) // 2] if len(rr) % 2 else 0.5 * (rr[len(rr) // 2 - 1] + rr[len(rr) // 2])
        big = max(rs)
        over = ", ".join(f"{v} {r:.2f}" for r, v in sorted(rs, reverse=True) if r > 1.1)
        out.append(f"| {case} | {dom} | {h:g} h | {med:.2f} | {big[0]:.2f} ({big[1]}) | {over or '-'} |")
    text = (f"### Summary: R = {m}({num}) / {m}({den})\n\n" + "\n".join(out) + "\n\n### Per case\n" + "\n".join(lines)
            + "\n")
    if a.md:
        open(a.md, "w").write(text)
    else:
        sys.stdout.write(text)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("compare", help="statistics of TEST against REF (JSON lines)")
    c.add_argument("test", nargs=1)
    c.add_argument("ref", nargs=1)
    c.add_argument("--domain", default="d01")
    c.add_argument("--case", default="case")
    c.add_argument("--pair", default="test-ref")
    c.add_argument("--vars", default="")
    c.add_argument("--out", default="")
    r = sub.add_parser("report", help="markdown tables and SVG figures from the JSON lines of several pairs")
    r.add_argument("stats", nargs="+")
    r.add_argument("--md", default="")
    r.add_argument("--svg-dir", default="")
    r.add_argument("--svg-rel", default="", help="the figures' directory as the markdown links name it")
    r.add_argument("--metric", default="rmse", choices=("rmse", "max_abs", "rel_l2"))
    r.add_argument("--num", default="fp32-r8")
    r.add_argument("--den", default="r4-r8")
    r.add_argument("--hours", default="1,3,6,12,24,48,72")
    a = ap.parse_args()
    sys.exit(compare(a) if a.cmd == "compare" else report(a))


if __name__ == "__main__":
    main()
