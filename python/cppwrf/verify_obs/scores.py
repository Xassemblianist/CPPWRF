# SPDX-License-Identifier: GPL-3.0-or-later
"""Forecast-observation pairs and their scores.

Pairs: for each station, model output time and variable, the observation nearest in time within +-window minutes
that has the variable (a METAR at hh:50 verifies the output at hh+1:00). Precipitation: an observed amount over the
last N hours ending at an output time t (within 1 minute) is paired with the model's accumulation between t - N h and
t, when both are output times of the run.

Continuous scores (Wilks 2011, "Statistical methods in the atmospheric sciences", 3rd ed., chapter 8): bias = mean(f
- o), MAE, RMSE, Pearson correlation. Wind: vector RMSE sqrt(mean((uf - uo)**2 + (vf - vo)**2)) over reports with a
direction or calm; direction error (f - o wrapped to [-180, 180)) over reports whose observed speed is at least
min_wind_dir m s-1 (mean and mean absolute). Precipitation, per accumulation period and threshold, the 2x2
contingency table (hits a, false alarms b, misses c, correct negatives d; an event is amount >= threshold) and POD =
a / (a + c), FAR = b / (a + b), CSI = a / (a + b + c), frequency bias = (a + b) / (a + c), ETS = (a - ar) / (a + b + c
- ar), ar = (a + b)(a + c) / n (Wilks 2011, section 8.2)."""
import datetime
import math

import numpy as np

CONTINUOUS = ("t2", "td2", "rh2", "wspd", "mslp", "qnh")
UNITS = {"t2": "degC", "td2": "degC", "rh2": "%", "wspd": "m s-1", "wdir": "degrees", "u": "m s-1", "v": "m s-1",
         "mslp": "hPa", "qnh": "hPa", "precip": "mm"}
NAMES = {"t2": "2 m temperature", "td2": "2 m dew point", "rh2": "2 m relative humidity", "wspd": "10 m wind speed",
         "wdir": "10 m wind direction", "wind": "10 m wind vector", "mslp": "sea level pressure", "qnh": "QNH",
         "precip": "precipitation"}


class Pair:
    __slots__ = ("run", "station", "var", "valid", "lead", "obs", "fcst", "dt_min", "hours", "obs_time")

    def __init__(self, run, station, var, valid, lead, obs, fcst, dt_min=0.0, hours=None, obs_time=None):
        self.run, self.station, self.var, self.valid, self.lead = run, station, var, valid, lead
        self.obs, self.fcst, self.dt_min, self.hours, self.obs_time = obs, fcst, dt_min, hours, obs_time


def make_pairs(run, locs, model, obs_by_station, window_minutes=30.0, height_correction=True,
               lapse_rate=0.0065):
    """[Pair] of one run (model: Run.sample's dict). T2 is moved from the model's terrain height to the station's
    with lapse_rate (K m-1) when height_correction (and the station's elevation is known)."""
    out = []
    times = run.times
    tindex = {t: n for n, t in enumerate(times)}
    win = window_minutes * 60.0
    for k, L in enumerate(locs):
        s = L.station
        obs = obs_by_station.get(s.id, [])
        if not obs:
            continue
        dz = (L.model_elev - s.elev) if (height_correction and s.elev is not None) else 0.0
        for n, t in enumerate(times):
            lead = run.lead_hours(t)
            best = {}
            for o in obs:
                dt = (o.time - t).total_seconds()
                if abs(dt) > win:
                    continue
                for var in CONTINUOUS + ("wind",):
                    if (o.u if var == "wind" else o.get(var)) is None:
                        continue
                    if var not in best or abs(dt) < abs(best[var][0]):
                        best[var] = (dt, o)
            for var, (dt, o) in best.items():
                dtm = dt / 60.0
                if var == "wind":
                    uf, vf = model["u10"][n, k], model["v10"][n, k]
                    if not (np.isfinite(uf) and np.isfinite(vf)):
                        continue
                    if o.u is not None:
                        out.append(Pair(run.label, s.id, "u", t, lead, o.u, float(uf), dtm, obs_time=o.time))
                        out.append(Pair(run.label, s.id, "v", t, lead, o.v, float(vf), dtm, obs_time=o.time))
                    if o.wdir is not None:
                        out.append(Pair(run.label, s.id, "wdir", t, lead, o.wdir, float(model["wdir"][n, k]), dtm,
                                        obs_time=o.time))
                    continue
                f = float(model[var][n, k])
                if not np.isfinite(f):
                    continue
                if var == "t2":
                    f += lapse_rate * dz
                out.append(Pair(run.label, s.id, var, t, lead, o.get(var), f, dtm, obs_time=o.time))
        seen = set()
        for o in obs:
            for hours, mm in o.precip:
                end = next((t for t in times if abs((t - o.time).total_seconds()) <= 60.0), None)
                if end is None or (s.id, end, hours) in seen:
                    continue
                begin = end - datetime.timedelta(hours=hours)
                if begin not in tindex:
                    continue
                f = model["precip_acc"][tindex[end], k] - model["precip_acc"][tindex[begin], k]
                if not np.isfinite(f):
                    continue
                seen.add((s.id, end, hours))
                out.append(Pair(run.label, s.id, "precip", end, run.lead_hours(end), mm, float(max(f, 0.0)), 0.0,
                                hours=hours, obs_time=o.time))
    return out


def continuous(f, o):
    f = np.asarray(f, dtype=np.float64)
    o = np.asarray(o, dtype=np.float64)
    n = int(f.size)
    if n == 0:
        return {"n": 0, "bias": None, "mae": None, "rmse": None, "corr": None, "mean_obs": None, "mean_fcst": None}
    e = f - o
    corr = None
    if n >= 3 and np.std(f) > 0 and np.std(o) > 0:
        corr = float(np.corrcoef(f, o)[0, 1])
    return {"n": n, "bias": float(e.mean()), "mae": float(np.abs(e).mean()), "rmse": float(math.sqrt((e * e).mean())),
            "corr": corr, "mean_obs": float(o.mean()), "mean_fcst": float(f.mean())}


def direction_error(fd, od):
    d = (np.asarray(fd, dtype=np.float64) - np.asarray(od, dtype=np.float64) + 180.0) % 360.0 - 180.0
    if d.size == 0:
        return {"n": 0, "bias": None, "mae": None}
    return {"n": int(d.size), "bias": float(d.mean()), "mae": float(np.abs(d).mean())}


def vector_rmse(uf, vf, uo, vo):
    uf, vf, uo, vo = (np.asarray(a, dtype=np.float64) for a in (uf, vf, uo, vo))
    if uf.size == 0:
        return {"n": 0, "rmse": None}
    return {"n": int(uf.size), "rmse": float(math.sqrt(((uf - uo) ** 2 + (vf - vo) ** 2).mean()))}


def contingency(f, o, thr):
    f = np.asarray(f, dtype=np.float64) >= thr
    o = np.asarray(o, dtype=np.float64) >= thr
    a = int((f & o).sum())
    b = int((f & ~o).sum())
    c = int((~f & o).sum())
    d = int((~f & ~o).sum())
    n = a + b + c + d

    def div(x, y):
        return x / y if y else None
    ar = (a + b) * (a + c) / n if n else 0.0
    return {"threshold": thr, "n": n, "hits": a, "false_alarms": b, "misses": c, "correct_negatives": d,
            "pod": div(a, a + c), "far": div(b, a + b), "csi": div(a, a + b + c), "fbias": div(a + b, a + c),
            "ets": div(a - ar, a + b + c - ar)}


def _group(pairs, key):
    g = {}
    for p in pairs:
        g.setdefault(key(p), []).append(p)
    return g


def summarise(pairs, thresholds=(0.1, 1.0, 5.0, 10.0, 25.0), min_wind_dir=1.0, obs_speed=None):
    """Scores of a list of pairs: {variable: continuous scores}, plus "wind" (vector RMSE), "wdir" (direction error
    over observed speeds >= min_wind_dir) and "precip" ({hours: {"continuous", "contingency": [...]}}).
    obs_speed: {(station, obs_time): observed speed} for the direction filter."""
    out = {}
    by = _group(pairs, lambda p: p.var)
    for var in CONTINUOUS:
        ps = by.get(var, [])
        if ps:
            out[var] = continuous([p.fcst for p in ps], [p.obs for p in ps])
    us = by.get("u", [])
    vs = {(p.station, p.valid): p for p in by.get("v", [])}
    uv = [(p, vs[(p.station, p.valid)]) for p in us if (p.station, p.valid) in vs]
    if uv:
        out["wind"] = vector_rmse([a.fcst for a, b in uv], [b.fcst for a, b in uv], [a.obs for a, b in uv],
                                  [b.obs for a, b in uv])
    wd = by.get("wdir", [])
    if wd:
        if obs_speed is not None:
            wd = [p for p in wd if obs_speed.get((p.station, p.obs_time), 0.0) >= min_wind_dir]
        out["wdir"] = direction_error([p.fcst for p in wd], [p.obs for p in wd])
    pr = by.get("precip", [])
    if pr:
        out["precip"] = {}
        for h, ps in sorted(_group(pr, lambda p: p.hours).items()):
            f = [p.fcst for p in ps]
            o = [p.obs for p in ps]
            out["precip"][h] = {"continuous": continuous(f, o), "contingency": [contingency(f, o, t)
                                                                                for t in thresholds]}
    return out
