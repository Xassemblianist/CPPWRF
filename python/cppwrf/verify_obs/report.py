# SPDX-License-Identifier: GPL-3.0-or-later
"""The verification's output: CSV tables, scores.json, SVG plots (score against lead time; a station map of the bias)
and summary.md. The plots are plain SVG written here: no plotting library is needed."""
import csv
import html
import json
import math
import os

from .scores import NAMES, UNITS

LEAD_VARS = ("t2", "td2", "rh2", "wspd", "wind", "wdir", "mslp", "qnh")
COLOURS = ("#1f6fb4", "#d1495b", "#2e8b57", "#8a5a9f")


def _fmt(v, nd=2):
    if v is None:
        return ""
    if isinstance(v, float):
        if math.isnan(v):
            return ""
        return f"{v:.{nd}f}"
    return str(v)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([_fmt(x, 4) if isinstance(x, float) else ("" if x is None else x) for x in r])


def flat_scores(sc):
    """[(variable, statistic dict)] of a summarise() result without precipitation."""
    out = []
    for var in LEAD_VARS:
        if var in sc:
            out.append((var, sc[var]))
    return out


def score_rows(group_name, group, sc):
    rows = []
    for var, d in flat_scores(sc):
        rows.append([group_name, group, var, UNITS.get(var, "m s-1" if var == "wind" else ""), d.get("n"),
                     d.get("bias"), d.get("mae"), d.get("rmse"), d.get("corr"), d.get("mean_obs"),
                     d.get("mean_fcst")])
    for h, pd in sorted(sc.get("precip", {}).items()):
        d = pd["continuous"]
        rows.append([group_name, group, f"precip_{h}h", "mm", d["n"], d["bias"], d["mae"], d["rmse"], d["corr"],
                     d["mean_obs"], d["mean_fcst"]])
    return rows


SCORE_HEADER = ["run", "group", "variable", "units", "n", "bias", "mae", "rmse", "corr", "mean_obs", "mean_fcst"]


# -- SVG ---------------------------------------------------------------------------------------------------------------
def _nice(lo, hi):
    if hi <= lo:
        hi = lo + 1.0
    span = hi - lo
    step = 10 ** math.floor(math.log10(span / 4))
    for m in (1, 2, 2.5, 5, 10):
        if span / (step * m) <= 6:
            step *= m
            break
    a = math.floor(lo / step) * step
    b = math.ceil(hi / step) * step
    ticks = []
    t = a
    while t <= b + 1e-9 * step:
        ticks.append(round(t, 10))
        t += step
    return a, b, ticks


def lead_plot(path, title, ylabel, series):
    """series: [(label, colour, dash, [(lead, value)])] -> an SVG line plot."""
    W, H, L, R, T, B = 560, 300, 60, 150, 30, 40
    pts = [v for _, _, _, s in series for v in s if v[1] is not None]
    if not pts:
        return False
    x0, x1, xt = _nice(min(p[0] for p in pts), max(p[0] for p in pts))
    y0, y1, yt = _nice(min(0.0, min(p[1] for p in pts)), max(0.0, max(p[1] for p in pts)))

    def X(v):
        return L + (v - x0) / (x1 - x0) * (W - L - R)

    def Y(v):
        return T + (y1 - v) / (y1 - y0) * (H - T - B)
    e = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="sans-serif" font-size="11">', f'<rect width="{W}" height="{H}" fill="white"/>',
         f'<text x="{L}" y="18" font-size="13">{html.escape(title)}</text>']
    for t in yt:
        e.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(t):.1f}" y2="{Y(t):.1f}" stroke="#ddd"/>')
        e.append(f'<text x="{L - 6}" y="{Y(t) + 4:.1f}" text-anchor="end">{t:g}</text>')
    for t in xt:
        e.append(f'<text x="{X(t):.1f}" y="{H - B + 16}" text-anchor="middle">{t:g}</text>')
    if y0 < 0 < y1:
        e.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(0):.1f}" y2="{Y(0):.1f}" stroke="#888"/>')
    e.append(f'<rect x="{L}" y="{T}" width="{W - L - R}" height="{H - T - B}" fill="none" stroke="#444"/>')
    e.append(f'<text x="{(L + W - R) / 2}" y="{H - 6}" text-anchor="middle">lead time (h)</text>')
    e.append(f'<text transform="translate(14,{(T + H - B) / 2}) rotate(-90)" text-anchor="middle">'
             f'{html.escape(ylabel)}</text>')
    for k, (label, colour, dash, s) in enumerate(series):
        s = [p for p in s if p[1] is not None]
        if not s:
            continue
        d = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in s)
        da = ' stroke-dasharray="5,3"' if dash else ""
        e.append(f'<polyline points="{d}" fill="none" stroke="{colour}" stroke-width="1.8"{da}/>')
        for a, b in s:
            e.append(f'<circle cx="{X(a):.1f}" cy="{Y(b):.1f}" r="2" fill="{colour}"/>')
        ly = T + 12 + 16 * k
        e.append(f'<line x1="{W - R + 10}" x2="{W - R + 34}" y1="{ly - 4}" y2="{ly - 4}" stroke="{colour}" '
                 f'stroke-width="1.8"{da}/>')
        e.append(f'<text x="{W - R + 38}" y="{ly}">{html.escape(label)}</text>')
    e.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(e) + "\n")
    return True


def _diverging(x):
    """x in [-1, 1] -> a blue-white-red colour."""
    x = max(-1.0, min(1.0, x))
    if x < 0:
        a = -x
        r, g, b = 1 - 0.85 * a, 1 - 0.6 * a, 1.0 - 0.2 * a
    else:
        r, g, b = 1.0 - 0.2 * x, 1 - 0.75 * x, 1 - 0.8 * x
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def map_plot(path, title, outline, points, units):
    """points: [(lat, lon, bias, label)]; outline: [(lat, lon)] of the domain's edge."""
    if not points:
        return False
    lats = [p[0] for p in outline] + [p[0] for p in points]
    lons = [p[1] for p in outline] + [p[1] for p in points]
    la0, la1, lo0, lo1 = min(lats), max(lats), min(lons), max(lons)
    k = math.cos(math.radians((la0 + la1) / 2))
    M = 30
    s = 440.0 / max(la1 - la0, (lo1 - lo0) * k, 1e-6)  # pixels per degree of latitude: 440 px for the longer side
    W = int(2 * M + (lo1 - lo0) * k * s + 100)
    H = int(2 * M + (la1 - la0) * s)
    H = max(H, 330)

    def P(la, lo):
        return M + (lo - lo0) * k * s, H - M - (la - la0) * s
    vmax = max(abs(p[2]) for p in points) or 1.0
    e = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="sans-serif" font-size="11">', f'<rect width="{W}" height="{H}" fill="white"/>',
         f'<text x="{M}" y="18" font-size="13">{html.escape(title)}</text>']
    d = " ".join("{:.1f},{:.1f}".format(*P(la, lo)) for la, lo in outline)
    e.append(f'<polygon points="{d}" fill="#f6f6f6" stroke="#666"/>')
    for la in range(math.ceil(la0), math.floor(la1) + 1):
        if la % 2 == 0:
            x_a, y_a = P(la, lo0)
            x_b, _ = P(la, lo1)
            e.append(f'<line x1="{x_a:.1f}" x2="{x_b:.1f}" y1="{y_a:.1f}" y2="{y_a:.1f}" stroke="#e4e4e4"/>')
            hemi = "N" if la >= 0 else "S"
            e.append(f'<text x="{x_a - 2:.1f}" y="{y_a + 3:.1f}" text-anchor="end" fill="#888">{abs(la)}{hemi}</text>')
    for lo in range(math.ceil(lo0), math.floor(lo1) + 1):
        if lo % 2 == 0:
            x_a, y_a = P(la0, lo)
            _, y_b = P(la1, lo)
            e.append(f'<line x1="{x_a:.1f}" x2="{x_a:.1f}" y1="{y_a:.1f}" y2="{y_b:.1f}" stroke="#e4e4e4"/>')
            hemi = "E" if lo >= 0 else "W"
            e.append(f'<text x="{x_a:.1f}" y="{y_a + 12:.1f}" text-anchor="middle" fill="#888">{abs(lo)}{hemi}</text>')
    for la, lo, b, lab in points:
        x, y = P(la, lo)
        e.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5.5" fill="{_diverging(b / vmax)}" stroke="#333" '
                 f'stroke-width="0.6"><title>{html.escape(lab)}: {b:+.2f} {html.escape(units)}</title></circle>')
    # colour bar
    x = W - M - 50
    for n in range(21):
        v = 1 - n / 10
        e.append(f'<rect x="{x}" y="{M + 10 + n * 12}" width="14" height="12" fill="{_diverging(v)}"/>')
    e.append(f'<text x="{x + 18}" y="{M + 20}">+{vmax:.1f}</text>')
    e.append(f'<text x="{x + 18}" y="{M + 10 + 10 * 12 + 9}">0</text>')
    e.append(f'<text x="{x + 18}" y="{M + 10 + 20 * 12 + 9}">-{vmax:.1f}</text>')
    e.append(f'<text x="{x - 4}" y="{M + 10 + 21 * 12 + 14}">{html.escape(units)}</text>')
    e.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(e) + "\n")
    return True


def outline_of(run):
    lat, lon = run.lat, run.lon
    edge = ([(lat[0, i], lon[0, i]) for i in range(lon.shape[1])]
            + [(lat[j, -1], lon[j, -1]) for j in range(lat.shape[0])]
            + [(lat[-1, i], lon[-1, i]) for i in range(lon.shape[1] - 1, -1, -1)]
            + [(lat[j, 0], lon[j, 0]) for j in range(lat.shape[0] - 1, -1, -1)])
    return [(float(a), float(b)) for a, b in edge]


def plots(outdir, results, by_lead, by_station, stations, run0):
    """SVG plots: lead_<var>.svg (RMSE solid, bias dashed, per run) and map_bias_<var>.svg (first run)."""
    os.makedirs(outdir, exist_ok=True)
    made = []
    labels = list(results)
    for var in LEAD_VARS:
        series = []
        for k, lab in enumerate(labels):
            leads = sorted(by_lead[lab])
            c = COLOURS[k % len(COLOURS)]
            if var == "wind":
                pts = [(ld, by_lead[lab][ld].get("wind", {}).get("rmse")) for ld in leads]
                series.append((f"{lab} vector RMSE", c, False, pts))
            elif var == "wdir":
                pts = [(ld, by_lead[lab][ld].get("wdir", {}).get("mae")) for ld in leads]
                series.append((f"{lab} MAE", c, False, pts))
                pts = [(ld, by_lead[lab][ld].get("wdir", {}).get("bias")) for ld in leads]
                series.append((f"{lab} bias", c, True, pts))
            else:
                pts = [(ld, by_lead[lab][ld].get(var, {}).get("rmse")) for ld in leads]
                series.append((f"{lab} RMSE", c, False, pts))
                pts = [(ld, by_lead[lab][ld].get(var, {}).get("bias")) for ld in leads]
                series.append((f"{lab} bias", c, True, pts))
        unit = UNITS.get(var, "m s-1")
        p = os.path.join(outdir, f"lead_{var}.svg")
        if lead_plot(p, f"{NAMES[var]}: scores against lead time", unit, series):
            made.append(p)
    lab = labels[0]
    for var in ("t2", "wspd", "td2", "mslp", "qnh"):
        pts = []
        for sid, sc in by_station[lab].items():
            d = sc.get(var)
            if d and d.get("n") and d.get("bias") is not None:
                s = stations[sid]
                pts.append((s.lat, s.lon, d["bias"], f"{sid} {s.name}".strip()))
        p = os.path.join(outdir, f"map_bias_{var}.svg")
        if map_plot(p, f"{NAMES[var]}: bias per station ({lab})", outline_of(run0), pts, UNITS[var]):
            made.append(p)
    return made


# -- Markdown ----------------------------------------------------------------------------------------------------------
def markdown(path, meta, overall, contingency_rows, qc, plot_files, outdir):
    labels = list(overall)
    L = ["# Verification against surface observations", ""]
    for k, v in meta.items():
        L.append(f"- {k}: {v}")
    L += ["", "## Scores over all stations and lead times", ""]
    head = "| variable | units |" + "".join(f" n | bias | MAE | RMSE | corr ({lab}) |" for lab in labels)
    L.append(head)
    L.append("|---|---|" + "---|---|---|---|---|" * len(labels))
    for var in LEAD_VARS:
        if not any(var in overall[lab] for lab in labels):
            continue
        cells = []
        for lab in labels:
            d = overall[lab].get(var, {})
            cells.append(f" {d.get('n', '')} | {_fmt(d.get('bias'))} | {_fmt(d.get('mae'))} | "
                         f"{_fmt(d.get('rmse'))} | {_fmt(d.get('corr'))} |")
        L.append(f"| {NAMES[var]} | {UNITS.get(var, 'm s-1')} |" + "".join(cells))
    L.append("")
    L.append("The wind vector row gives the vector RMSE in the RMSE column; wind direction: bias and MAE in degrees "
             "over observed speeds of at least the --min-wind-dir threshold.")
    if contingency_rows:
        L += ["", "## Precipitation", "", "| run | period (h) | threshold (mm) | n | hits | false alarms | misses | "
              "POD | FAR | CSI | freq. bias | ETS |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in contingency_rows:
            L.append("| " + " | ".join(_fmt(x) for x in r) + " |")
    else:
        L += ["", "## Precipitation", "", "No precipitation amounts in the observations of this period (METARs "
              "outside the United States carry none)."]
    L += ["", "## Quality control", ""]
    for lab, q in qc.items():
        items = ", ".join(f"{k} {v}" for k, v in sorted(q.items()))
        L.append(f"- {lab}: {items or 'nothing rejected'}")
    if plot_files:
        L += ["", "## Plots", ""]
        for p in plot_files:
            rel = os.path.relpath(p, outdir)
            L.append(f"![{os.path.basename(p)}]({rel})")
    L.append("")
    with open(path, "w") as f:
        f.write("\n".join(L))


def write_json(path, obj):
    def conv(o):
        if isinstance(o, dict):
            return {str(k): conv(v) for k, v in o.items()}
        if isinstance(o, (list, tuple)):
            return [conv(v) for v in o]
        if isinstance(o, float) and math.isnan(o):
            return None
        return o
    with open(path, "w") as f:
        json.dump(conv(obj), f, indent=1)
