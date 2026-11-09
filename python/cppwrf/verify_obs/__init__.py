# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-verify-obs: a run's wrfout files against surface observations (docs/VERIFY_OBS.md).

    from cppwrf import verify_obs
    res = verify_obs.verify("run/wrfout_d01_*", obs_source="iem", output_dir="verify")
    res["overall"]["run"]["t2"]["rmse"]

Observations: NOAA NCEI ISD / Global Hourly (obs_source "isd"), the Iowa Environmental Mesonet's METAR archive
("iem") or a plain CSV file ("file"); sources.py downloads and caches them, obs.py parses and checks them. model.py
reads the wrfout files at the stations, scores.py pairs and scores, report.py writes the tables, plots and summary."""
import argparse
import collections
import datetime
import os
import sys
import time as _time

from . import obs as obsmod
from . import report, scores, sources
from .model import Run

__all__ = ["verify", "main"]

THRESHOLDS = (0.1, 1.0, 5.0, 10.0, 25.0)
PRECIP_HOURS = (1, 3, 6, 12, 24)


def _select_catalogue(cat, run, source, start, end):
    """Catalogue stations that may report in the domain and period: inside the run's lat / lon box; ISD: a record
    period covering the run; IEM: an ICAO identifier and a record reaching at least five years before the run."""
    la0, la1, lo0, lo1 = run.bounds()
    out = []
    s0, e0 = f"{start:%Y%m%d}", f"{end:%Y%m%d}"
    recent = f"{start.year - 5}0101"
    for s in cat:
        if not (la0 <= s.lat <= la1 and lo0 <= s.lon <= lo1):
            continue
        if source == "isd" and not (s.begin <= e0 and s.end >= s0):
            continue
        if source == "iem" and (len(s.icao) != 4 or not s.icao.isalnum() or s.end < recent):
            continue
        out.append(s)
    return out


def _load_obs(source, run, start, end, stations, obs_file, fetcher, say):
    qc = obsmod.QC()
    meta, records = {}, []
    if source == "file":
        if not obs_file:
            raise ValueError("--obs-source file needs --obs-file")
        with open(obs_file, encoding="utf-8-sig", errors="replace") as f:
            meta, records, qc = obsmod.parse_file(f.read(), qc)
        return meta, records, qc
    if stations in (None, "auto"):
        say("station catalogue (NCEI isd-history.csv)")
        cat = sources.catalogue(fetcher)
        names = {s.icao: s.name for s in cat if s.icao}
        cand = _select_catalogue(cat, run, source, start, end)
        located = run.locate(cand)
        inside = [L.station for L in located if L.inside]
        say(f"{len(cand)} catalogue stations in the lat / lon box, {len(inside)} inside the domain")
        if source == "isd":
            ids = [s.id for s in inside]
        else:
            ids = sorted({s.icao for s in inside})
    else:
        names = {}
        ids = [s.strip() for s in (stations.split(",") if isinstance(stations, str) else stations) if s.strip()]
    if not ids:
        raise RuntimeError(f"no {source} stations in the domain for {start:%Y-%m-%d %H} to {end:%Y-%m-%d %H} UTC"
                           + (" (ISD's Global Hourly files end in 2025: try --obs-source iem)"
                              if source == "isd" and start.year >= 2025 else ""))
    say(f"{source}: {len(ids)} stations, {start:%Y-%m-%d %H:%M} to {end:%Y-%m-%d %H:%M} UTC")
    if source == "isd":
        texts = sources.isd(fetcher, ids, start, end)
        parse = obsmod.parse_isd
    elif source == "iem":
        texts = sources.iem(fetcher, ids, start, end)
        parse = obsmod.parse_iem
    else:
        raise ValueError(f"observation source {source!r} (isd, iem or file)")
    for t in texts:
        m, r, qc = parse(t, qc)
        meta.update(m)
        records += r
    for s in meta.values():
        if not s.name:
            s.name = names.get(s.icao, "")
    return meta, records, qc


def verify(files, obs_source="iem", stations="auto", output_dir="verify", compare=None, labels=None,
           obs_file=None, cache_dir=None, boundary=None, interpolation="bilinear", height_correction=True,
           lapse_rate=0.0065, window_minutes=30.0, thresholds=THRESHOLDS, precip_hours=PRECIP_HOURS,
           min_wind_dir=1.0, max_elev_diff=None, offline=False, pause=1.0, make_plots=True, log=None):
    """Verify wrfout files (paths or globs) against surface observations; compare: the files of a second run scored
    on the same pairs (e.g. GPU FP32 against CPU FP64). Returns {"output_dir", "files", "overall": {run: scores},
    "by_lead", "by_station", "stations", "pairs"}. The keyword arguments are the command's options
    (docs/VERIFY_OBS.md)."""
    say = log or (lambda s: None)
    t0 = _time.time()
    runs = [Run(files, None, boundary)]
    if compare:
        runs.append(Run(compare, None, boundary))
    if labels:
        for r, lab in zip(runs, labels):
            r.label = lab
    if len(runs) == 2 and runs[0].label == runs[1].label:
        runs[0].label, runs[1].label = runs[0].label + "_A", runs[1].label + "_B"
    run0 = runs[0]
    start = min(min(r.times) for r in runs)
    end = max(max(r.times) for r in runs)
    win = datetime.timedelta(minutes=window_minutes)
    fetcher = sources.Fetcher(cache_dir, pause=pause, offline=offline, log=say)
    meta, records, qc = _load_obs(obs_source, run0, start - win, end + win, stations, obs_file, fetcher, say)
    if obs_source != "file":
        say(f"downloads: {fetcher.requests} requests, {fetcher.bytes / 1e6:.2f} MB (cache {fetcher.cache_dir})")
    by_station = collections.defaultdict(list)
    for o in records:
        if start - win <= o.time <= end + win:
            by_station[o.station].append(o)
    stations_used = [meta[s] for s in sorted(by_station) if s in meta]
    excluded = collections.Counter()
    results, pairs_all, locs_by_run = {}, [], {}
    for r in runs:
        locs = r.locate(stations_used, max_elev_diff)
        for L in locs:
            if not L.inside:
                excluded[f"{r.label}: {L.reason}"] += 1
        locs = [L for L in locs if L.inside]
        locs_by_run[r.label] = locs
        if not locs:
            raise RuntimeError(f"{r.label}: no observing station inside the domain")
        say(f"{r.label}: {len(locs)} stations inside the domain (boundary {r.boundary} points, {r.boundary_source}); "
            f"reading {len(r.frames)} times")
        model = r.sample(locs, interpolation, say)
        results[r.label] = scores.make_pairs(r, locs, model, by_station, window_minutes, height_correction,
                                             lapse_rate)
    if len(runs) == 2:
        # the same pairs for both runs: those whose (station, variable, time, period) both have
        keys = [{(p.station, p.var, p.valid, p.hours) for p in ps} for ps in results.values()]
        common = keys[0] & keys[1]
        for lab in results:
            results[lab] = [p for p in results[lab] if (p.station, p.var, p.valid, p.hours) in common]
    obs_speed = {(o.station, o.time): o.wspd for o in records if o.wspd is not None}
    hours_ok = set(int(h) for h in precip_hours)
    for lab in results:
        results[lab] = [p for p in results[lab] if p.var != "precip" or p.hours in hours_ok]
        pairs_all += results[lab]

    def summ(ps):
        return scores.summarise(ps, thresholds, min_wind_dir, obs_speed)
    overall = {lab: summ(ps) for lab, ps in results.items()}
    by_lead = {lab: {ld: summ(g) for ld, g in sorted(_group(ps, lambda p: p.lead).items())}
               for lab, ps in results.items()}
    by_stat = {lab: {sid: summ(g) for sid, g in sorted(_group(ps, lambda p: p.station).items())}
               for lab, ps in results.items()}
    os.makedirs(output_dir, exist_ok=True)
    files_out = []

    def out(name):
        p = os.path.join(output_dir, name)
        files_out.append(p)
        return p
    stations_csv = []
    for r in runs:
        for L in r.locate(stations_used, max_elev_diff):
            s = L.station
            stations_csv.append([r.label, s.id, s.icao, s.name, s.lat, s.lon, s.elev, L.model_elev,
                                 L.fi, L.fj, "yes" if L.inside else "no", L.reason])
    report.write_csv(out("stations.csv"), ["run", "station", "icao", "name", "lat", "lon", "elevation",
                                           "model_elevation", "i", "j", "used", "reason"], stations_csv)
    report.write_csv(out("pairs.csv"), ["run", "station", "variable", "valid", "lead_h", "obs", "fcst", "dt_min",
                                        "period_h", "obs_time"],
                     [[p.run, p.station, p.var, f"{p.valid:%Y-%m-%d %H:%M}", p.lead, p.obs, p.fcst, p.dt_min,
                       p.hours, f"{p.obs_time:%Y-%m-%d %H:%M}" if p.obs_time else ""] for p in pairs_all])
    rows = []
    for lab in results:
        rows += report.score_rows(lab, "all", overall[lab])
    report.write_csv(out("scores_overall.csv"), report.SCORE_HEADER, rows)
    rows = []
    for lab in results:
        for ld, sc in by_lead[lab].items():
            rows += report.score_rows(lab, f"{ld:g}", sc)
    report.write_csv(out("scores_by_lead.csv"), ["run", "lead_h"] + report.SCORE_HEADER[2:], rows)
    rows = []
    for lab in results:
        for sid, sc in by_stat[lab].items():
            rows += report.score_rows(lab, sid, sc)
    report.write_csv(out("scores_by_station.csv"), ["run", "station"] + report.SCORE_HEADER[2:], rows)
    cont = []
    for lab in results:
        for h, pd in sorted(overall[lab].get("precip", {}).items()):
            for c in pd["contingency"]:
                cont.append([lab, h, c["threshold"], c["n"], c["hits"], c["false_alarms"], c["misses"], c["pod"],
                             c["far"], c["csi"], c["fbias"], c["ets"]])
    report.write_csv(out("contingency.csv"), ["run", "period_h", "threshold_mm", "n", "hits", "false_alarms",
                                              "misses", "pod", "far", "csi", "fbias", "ets"], cont)
    qc_out = {"observations": dict(qc), "stations excluded": dict(excluded)}
    meta_out = {"runs": ", ".join(f"{r.label} ({len(r.frames)} times, {r.paths[0]} ...)" for r in runs),
                "period": f"{start:%Y-%m-%d %H:%M} to {end:%Y-%m-%d %H:%M} UTC",
                "observations": obs_source + (f" ({obs_file})" if obs_file else ""),
                "stations used": len(locs_by_run[run0.label]),
                "pairs": len(pairs_all),
                "interpolation": interpolation,
                "T2 height correction": f"{lapse_rate * 1000:g} K/km" if height_correction else "off",
                "time window": f"+-{window_minutes:g} min",
                "boundary": f"{run0.boundary} points ({run0.boundary_source})"}
    report.write_json(out("scores.json"), {"meta": meta_out, "overall": overall, "by_lead": by_lead,
                                           "by_station": by_stat, "qc": qc_out, "qc_examples": qc.examples})
    plot_files = report.plots(os.path.join(output_dir, "plots"), results, by_lead, by_stat, meta, run0) \
        if make_plots else []
    files_out += plot_files
    report.markdown(out("summary.md"), meta_out, overall, cont, qc_out, plot_files, output_dir)
    say(f"{len(pairs_all)} pairs, {len(files_out)} files in {output_dir} ({_time.time() - t0:.1f} s)")
    return {"output_dir": output_dir, "files": files_out, "overall": overall, "by_lead": by_lead,
            "by_station": by_stat, "stations": meta, "pairs": pairs_all, "qc": qc_out}


def _group(items, key):
    g = {}
    for x in items:
        g.setdefault(key(x), []).append(x)
    return g


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="cppwrf-verify-obs",
        description="Verify wrfout files against surface observations (ISD, IEM METAR or a CSV file): bias, MAE, "
                    "RMSE, correlation, wind vector and direction errors, precipitation contingency scores. "
                    "docs/VERIFY_OBS.md describes the methods and options.")
    ap.add_argument("files", nargs="+", help="wrfout files (globs allowed)")
    ap.add_argument("--obs-source", choices=("isd", "iem", "file"), default="iem",
                    help="observations: NCEI ISD Global Hourly, IEM METAR archive, or --obs-file (default iem)")
    ap.add_argument("--obs-file", help="CSV of observations for --obs-source file")
    ap.add_argument("--stations", default="auto",
                    help="auto (every station in the domain) or a comma-separated list (ICAO for iem, USAF+WBAN "
                         "for isd)")
    ap.add_argument("-o", "--output-dir", default="verify", help="output directory (default ./verify)")
    ap.add_argument("--compare", nargs="+", metavar="FILE", help="wrfout files of a second run to compare")
    ap.add_argument("--labels", help="names of the run(s), comma-separated (default: the directories)")
    ap.add_argument("--cache-dir", help=f"download cache (default {sources.default_cache()})")
    ap.add_argument("--offline", action="store_true", help="use only the download cache")
    ap.add_argument("--pause", type=float, default=1.0, help="seconds between downloads (default 1)")
    ap.add_argument("--boundary", type=int, help="grid points next to the boundary left out (default: "
                                                 "spec_bdy_width of a namelist.input next to the files, else 5)")
    ap.add_argument("--interpolation", choices=("bilinear", "nearest"), default="bilinear")
    ap.add_argument("--no-height-correction", action="store_true",
                    help="compare T2 at the model's terrain height (default: corrected to the station's)")
    ap.add_argument("--lapse-rate", type=float, default=6.5, help="K/km for the T2 height correction (default 6.5)")
    ap.add_argument("--max-elev-diff", type=float, help="leave out stations whose elevation differs from the "
                                                        "model's by more than this (m)")
    ap.add_argument("--window", type=float, default=30.0, help="observation time window, minutes (default 30)")
    ap.add_argument("--thresholds", default="0.1,1,5,10,25", help="precipitation thresholds, mm")
    ap.add_argument("--precip-hours", default="1,3,6,12,24", help="precipitation periods, hours")
    ap.add_argument("--min-wind-dir", type=float, default=1.0,
                    help="observed speed (m/s) from which wind directions are scored (default 1)")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)
    try:
        res = verify(a.files, a.obs_source, a.stations, a.output_dir, a.compare,
                     a.labels.split(",") if a.labels else None, a.obs_file, a.cache_dir, a.boundary,
                     a.interpolation, not a.no_height_correction, a.lapse_rate / 1000.0, a.window,
                     tuple(float(x) for x in a.thresholds.split(",")),
                     tuple(int(x) for x in a.precip_hours.split(",")), a.min_wind_dir, a.max_elev_diff, a.offline,
                     a.pause, not a.no_plots, None if a.quiet else print)
    except (FileNotFoundError, ValueError, RuntimeError, sources.NotFound, OSError) as e:
        print(f"cppwrf-verify-obs: error: {e}", file=sys.stderr)
        return 2
    if not a.quiet:
        for lab, sc in res["overall"].items():
            parts = [f"{v} rmse {sc[v]['rmse']:.2f} bias {sc[v]['bias']:+.2f}"
                     for v in ("t2", "td2", "wspd", "mslp", "qnh") if v in sc]
            print(f"{lab}: " + "; ".join(parts))
        print(f"summary: {os.path.join(res['output_dir'], 'summary.md')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
