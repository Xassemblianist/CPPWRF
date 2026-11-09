# SPDX-License-Identifier: GPL-3.0-or-later
"""Surface observations: the parsers of NOAA NCEI's ISD / Global Hourly CSV, of METAR reports (as the Iowa
Environmental Mesonet's ASOS download service returns them) and of a plain CSV file, with their quality control.

Every parser returns Obs records in SI-like units: t2 / td2 degC, rh2 %, wspd / gust m s-1, wdir degrees from (None
when variable or calm), u / v m s-1 (None without a direction; 0 when calm), mslp / qnh hPa, precip [(hours, mm)]
ending at the record's time. A value that fails the quality control is None and the reason is counted in a QC
counter ({reason: count}), which the verification reports."""
import collections
import csv
import datetime
import io
import math
import re

KT = 0.514444  # m s-1 per knot
KMH = 1.0 / 3.6
INHG = 33.8638866667  # hPa per inch of mercury
INCH = 25.4  # mm

# gross limits (outside: the value is dropped and counted as "gross_<var>")
LIMITS = {"t2": (-90.0, 60.0), "td2": (-100.0, 40.0), "rh2": (0.0, 100.5), "wspd": (0.0, 75.0), "gust": (0.0, 110.0),
          "mslp": (850.0, 1090.0), "qnh": (850.0, 1090.0), "precip": (0.0, 500.0)}


class QC(collections.Counter):
    """{reason: count} of the quality control, with a few example messages (.examples) of reports that failed."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.examples = []

    def example(self, msg):
        if len(self.examples) < 20:
            self.examples.append(msg)


class Station:
    __slots__ = ("id", "name", "lat", "lon", "elev", "icao", "country", "begin", "end")

    def __init__(self, id, lat, lon, elev=None, name="", icao="", country=""):
        self.id = str(id)
        self.lat = float(lat)
        self.lon = float(lon)
        self.elev = None if elev is None or (isinstance(elev, float) and math.isnan(elev)) else float(elev)
        self.name = name or ""
        self.icao = icao or ""
        self.country = country or ""
        self.begin = self.end = ""

    def __repr__(self):
        return f"Station({self.id!r}, {self.lat:.4f}, {self.lon:.4f}, {self.elev}, {self.name!r})"


class Obs:
    __slots__ = ("station", "time", "t2", "td2", "rh2", "wspd", "wdir", "u", "v", "gust", "mslp", "qnh", "precip",
                 "source")

    def __init__(self, station, time, source=""):
        self.station = station
        self.time = time
        self.t2 = self.td2 = self.rh2 = self.wspd = self.wdir = self.u = self.v = self.gust = None
        self.mslp = self.qnh = None
        self.precip = []
        self.source = source

    def get(self, name):
        return getattr(self, name)

    def __repr__(self):
        vals = {k: getattr(self, k) for k in self.__slots__[2:-1] if getattr(self, k) not in (None, [])}
        return f"Obs({self.station!r}, {self.time:%Y-%m-%d %H:%M}, {vals})"


def es_hpa(tc):
    """Saturation vapour pressure over water (hPa) at tc (degC): the formula of wrf-python's DCOMPUTERH / DCOMPUTETD
    (Bolton 1980), so that observed and model relative humidities are computed alike."""
    return 6.112 * math.exp(17.67 * tc / (tc + 243.5))


def rh_from_td(t, td):
    return 100.0 * es_hpa(td) / es_hpa(t)


def wind_uv(speed, direction):
    """Earth-relative u, v (m s-1) of a wind from `direction` degrees."""
    r = math.radians(direction)
    return -speed * math.sin(r), -speed * math.cos(r)


def finish(o, qc):
    """Gross limits, dew point above temperature, derived RH and wind components."""
    for k in ("t2", "td2", "rh2", "wspd", "gust", "mslp", "qnh"):
        v = getattr(o, k)
        if v is not None:
            lo, hi = LIMITS[k]
            if not lo <= v <= hi:
                setattr(o, k, None)
                qc[f"gross_{k}"] += 1
    if o.t2 is not None and o.td2 is not None and o.td2 > o.t2 + 0.5:
        o.td2 = None
        qc["td_above_t"] += 1
    if o.rh2 is None and o.t2 is not None and o.td2 is not None:
        o.rh2 = min(100.0, rh_from_td(o.t2, o.td2))
    if o.wspd is not None:
        if o.wspd == 0.0:
            o.u = o.v = 0.0
        elif o.wdir is not None:
            o.u, o.v = wind_uv(o.wspd, o.wdir)
    keep = []
    for h, mm in o.precip:
        if mm is None or not LIMITS["precip"][0] <= mm <= LIMITS["precip"][1] or not 0 < h <= 48:
            qc["gross_precip"] += 1
        else:
            keep.append((h, mm))
    o.precip = keep
    return o


# -- ISD / Global Hourly (NCEI) ----------------------------------------------------------------------------------------
# Federal Climate Complex Data Documentation for Integrated Surface Data (ISD), NCEI, 2018: quality codes 0, 1, 4, 5,
# 9 and the letters A, C, I, M, P, R, U pass; 2 / 6 (suspect) and 3 / 7 (erroneous) are rejected.
ISD_BAD_QUALITY = set("2367")
# AA1-AA4 condition codes rejected: 1 measurement impossible or inaccurate, 5-8 deleted / missing periods,
# I / J incomplete amounts. 2 (trace) counts as 0 mm.
ISD_BAD_CONDITION = set("15678IJ")


def _isd_value(field, missing, scale, qc, name):
    """(value / scale or None) of an ISD 'value,quality' pair."""
    if not field:
        return None
    parts = field.split(",")
    raw, q = parts[0], parts[1] if len(parts) > 1 else "1"
    if raw.lstrip("+-") == missing.lstrip("+-"):
        return None
    if q in ISD_BAD_QUALITY:
        qc[f"isd_quality_{name}"] += 1
        return None
    try:
        return int(raw) / scale
    except ValueError:
        qc[f"isd_format_{name}"] += 1
        return None


def parse_isd(text, qc=None):
    """Records of an ISD Global Hourly CSV (the per-station-year files of NCEI / NOAA's open-data bucket, or the
    Access Data Service's dataset=global-hourly). Returns ({station id: Station}, [Obs], qc)."""
    qc = qc if qc is not None else QC()
    f = io.StringIO(text) if isinstance(text, str) else text
    stations, out = {}, []
    for row in csv.DictReader(f):
        sid = row.get("STATION", "").strip()
        if not sid:
            continue
        if sid not in stations:
            try:
                stations[sid] = Station(sid, float(row["LATITUDE"]), float(row["LONGITUDE"]),
                                        float(row["ELEVATION"]) if row.get("ELEVATION") else None,
                                        name=row.get("NAME", "").strip(), icao=row.get("CALL_SIGN", "").strip()
                                        if row.get("CALL_SIGN", "").strip() not in ("99999", "") else "")
            except (KeyError, ValueError):
                qc["isd_station_metadata"] += 1
                continue
        try:
            t = datetime.datetime.strptime(row["DATE"][:19], "%Y-%m-%dT%H:%M:%S")
        except (KeyError, ValueError):
            qc["isd_format_date"] += 1
            continue
        o = Obs(sid, t, "isd")
        qc["records"] += 1
        o.t2 = _isd_value(row.get("TMP"), "9999", 10.0, qc, "tmp")
        o.td2 = _isd_value(row.get("DEW"), "9999", 10.0, qc, "dew")
        o.mslp = _isd_value(row.get("SLP"), "99999", 10.0, qc, "slp")
        ma1 = row.get("MA1")
        if ma1:
            p = ma1.split(",")
            o.qnh = _isd_value(",".join(p[:2]), "99999", 10.0, qc, "altimeter")
        wnd = row.get("WND")
        if wnd:
            p = wnd.split(",")
            if len(p) >= 5:
                spd = _isd_value(f"{p[3]},{p[4]}", "9999", 10.0, qc, "wind_speed")
                calm = p[2] == "C" or (spd == 0.0)
                d = None if calm else _isd_value(f"{p[0]},{p[1]}", "999", 1.0, qc, "wind_dir")
                if spd is not None:
                    o.wspd = 0.0 if calm else spd
                    o.wdir = None if (calm or p[2] == "V") else d
            else:
                qc["isd_format_wind"] += 1
        for g in ("AA1", "AA2", "AA3", "AA4"):
            v = row.get(g)
            if not v:
                continue
            p = v.split(",")
            if len(p) < 4:
                qc["isd_format_precip"] += 1
                continue
            hours, depth, cond, q = p[0], p[1], p[2], p[3]
            if hours == "99" or depth == "9999":
                continue
            if q in ISD_BAD_QUALITY:
                qc["isd_quality_precip"] += 1
                continue
            if cond in ISD_BAD_CONDITION:
                qc["isd_condition_precip"] += 1
                continue
            try:
                o.precip.append((int(hours), 0.0 if cond == "2" else int(depth) / 10.0))
            except ValueError:
                qc["isd_format_precip"] += 1
        out.append(finish(o, qc))
    return stations, out, qc


# -- METAR -------------------------------------------------------------------------------------------------------------
# WMO Manual on Codes No. 306, FM 15 METAR; US remarks (SLPppp, Prrrr, 6RRRR, 7RRRR, TsnnnSnnn) from the Federal
# Meteorological Handbook No. 1 (FCM-H1-2019), chapter 12.
_WIND = re.compile(r"^(\d{3}|VRB|///)(\d{2,3}|//)(?:G(\d{2,3}))?(KT|MPS|KMH)$")
_TEMP = re.compile(r"^(M?\d{2})/(M?\d{2}|//)?$")
_TIME = re.compile(r"^(\d{2})(\d{2})(\d{2})Z$")
_ICAO = re.compile(r"^[A-Z][A-Z0-9]{3}$")


def _tc(s):
    return -int(s[1:]) if s.startswith("M") else int(s)


class MetarError(ValueError):
    pass


def parse_metar(text, valid=None, qc=None):
    """One METAR / SPECI report: an Obs at `valid` (datetime; else the report's day / hour / minute in valid's
    month, which must then be given). Raises MetarError when the report has no station, no time or none of wind,
    temperature and pressure."""
    qc = qc if qc is not None else QC()
    toks = text.replace("=", " ").split()
    while toks and toks[0] in ("METAR", "SPECI", "COR"):
        toks.pop(0)
    if not toks or not _ICAO.match(toks[0]):
        raise MetarError(f"no station: {text[:40]!r}")
    station = toks.pop(0)
    if not toks or not _TIME.match(toks[0]):
        raise MetarError(f"no ddhhmmZ time: {text[:40]!r}")
    dd, hh, mm = (int(x) for x in _TIME.match(toks.pop(0)).groups())
    if valid is None:
        raise MetarError("no date for the report")
    if valid.day != dd or valid.hour != hh or valid.minute != mm:
        qc["metar_time_mismatch"] += 1
    o = Obs(station, valid, "metar")
    if "RMK" in toks:
        k = toks.index("RMK")
        body, rmk = toks[:k], toks[k + 1:]
    else:
        body, rmk = toks, []
    found = False
    for tok in body:
        if tok in ("COR", "AUTO", "NIL"):
            if tok == "NIL":
                raise MetarError("NIL report")
            continue
        m = _WIND.match(tok)
        if m and o.wspd is None:
            d, s, g, unit = m.groups()
            f = {"KT": KT, "MPS": 1.0, "KMH": KMH}[unit]
            if s != "//":
                o.wspd = int(s) * f
                o.wdir = None if d in ("VRB", "///") or int(s) == 0 else float(int(d))
                if d not in ("VRB", "///") and int(d) > 360:
                    o.wdir = None
                    qc["metar_bad_wind_dir"] += 1
                if g:
                    o.gust = int(g) * f
            found = True
            continue
        m = _TEMP.match(tok)
        if m and o.t2 is None:
            o.t2 = float(_tc(m.group(1)))
            if m.group(2) and m.group(2) != "//":
                o.td2 = float(_tc(m.group(2)))
            found = True
            continue
        if re.match(r"^Q\d{4}$", tok):
            o.qnh = float(tok[1:])
            found = True
        elif re.match(r"^A\d{4}$", tok):
            o.qnh = int(tok[1:]) / 100.0 * INHG
            found = True
    hour_end = valid.replace(minute=0, second=0) + datetime.timedelta(hours=1 if valid.minute >= 30 else 0)
    for tok in rmk:
        if re.match(r"^SLP\d{3}$", tok):
            v = int(tok[3:]) / 10.0
            o.mslp = 1000.0 + v if v < 50.0 else 900.0 + v
        elif re.match(r"^T[01]\d{3}[01]\d{3}$", tok):
            t = int(tok[2:5]) / 10.0 * (-1 if tok[1] == "1" else 1)
            td = int(tok[6:9]) / 10.0 * (-1 if tok[5] == "1" else 1)
            o.t2, o.td2 = t, td
        elif re.match(r"^P\d{4}$", tok):
            o.precip.append((1, int(tok[1:]) / 100.0 * INCH))
        elif re.match(r"^6\d{4}$", tok) and hour_end.hour % 3 == 0:
            o.precip.append((6 if hour_end.hour % 6 == 0 else 3, int(tok[1:]) / 100.0 * INCH))
        elif re.match(r"^7\d{4}$", tok) and hour_end.hour in (0, 12):
            o.precip.append((24, int(tok[1:]) / 100.0 * INCH))
    if not found:
        raise MetarError(f"no wind, temperature or pressure group: {text[:60]!r}")
    if o.precip:
        # US hourly / 3-6 h / 24 h amounts are reported near the end of the hour: they end at that hour
        o.time = hour_end
    return o


def parse_iem(text, qc=None):
    """The Iowa Environmental Mesonet ASOS service's CSV (format=onlycomma, data=metar, latlon=yes, elev=yes).
    Returns ({station: Station}, [Obs], qc); a report that does not parse is skipped and counted as metar_error."""
    qc = qc if qc is not None else QC()
    f = io.StringIO(text) if isinstance(text, str) else text
    stations, out = {}, []
    for row in csv.DictReader(f):
        metar = (row.get("metar") or "").strip()
        if not metar or metar == "M":
            continue
        qc["records"] += 1
        try:
            t = datetime.datetime.strptime(row["valid"][:16], "%Y-%m-%d %H:%M")
            o = parse_metar(metar, t, qc)
        except (MetarError, KeyError, ValueError) as e:
            qc["metar_error"] += 1
            if hasattr(qc, "example"):
                qc.example(str(e))
            continue
        sid = o.station
        if sid not in stations:
            try:
                stations[sid] = Station(sid, float(row["lat"]), float(row["lon"]),
                                        float(row["elevation"]) if row.get("elevation") not in (None, "", "M")
                                        else None, icao=sid)
            except (KeyError, ValueError):
                qc["metar_station_metadata"] += 1
                continue
        out.append(finish(o, qc))
    return stations, out, qc


# -- plain CSV -------------------------------------------------------------------------------------------------------
ALIASES = {"station": ("station", "id", "station_id", "istasyon"),
           "time": ("time", "date", "datetime", "valid", "zaman", "tarih"),
           "lat": ("lat", "latitude", "enlem"), "lon": ("lon", "longitude", "boylam"),
           "elev": ("elev", "elevation", "height", "altitude", "yukseklik"),
           "name": ("name", "station_name", "ad"),
           "t2": ("t2", "temperature", "temp", "sicaklik"), "td2": ("td2", "dewpoint", "dew_point", "td"),
           "rh2": ("rh2", "rh", "humidity", "nem"), "wspd": ("wspd", "wind_speed", "ruzgar_hizi"),
           "wdir": ("wdir", "wind_dir", "wind_direction", "ruzgar_yonu"), "gust": ("gust", "wind_gust"),
           "mslp": ("mslp", "slp", "pmsl", "deniz_seviyesi_basinci"), "qnh": ("qnh",),
           "precip": ("precip", "precipitation", "rain", "yagis"), "precip_hours": ("precip_hours", "period")}
MISSING = {"", "na", "nan", "m", "-9999", "-999", "-9999.0", "9999", "null", "none"}


def _key(header):
    h = header.strip().lower()
    for k, names in ALIASES.items():
        if h in names:
            return k
    m = re.match(r"^precip_(\d+)h$", h)
    if m:
        return f"precip_{m.group(1)}h"
    return None


def parse_time(s):
    s = s.strip().replace("T", " ").rstrip("Z")
    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d %H", "%Y-%m-%d_%H:%M:%S", "%Y%m%d%H%M", "%Y%m%d%H",
                "%d.%m.%Y %H:%M"):
        try:
            return datetime.datetime.strptime(s, fmt)
        except ValueError:
            pass
    raise ValueError(f"time {s!r}")


def parse_file(text, qc=None):
    """A plain CSV of observations (docs/VERIFY_OBS.md "Your own observations"): one row per station and time, UTC;
    columns station, time, lat, lon, elev and any of t2, td2, rh2 (degC, %), wspd, gust (m s-1), wdir (degrees),
    mslp, qnh (hPa), precip (mm) with precip_hours, or precip_<N>h columns. Header names are case-insensitive and a
    few aliases (also Turkish) are known; missing values are empty, NA, M or -9999."""
    qc = qc if qc is not None else QC()
    f = io.StringIO(text) if isinstance(text, str) else text
    sample = f.read(4096)
    f.seek(0)
    try:
        dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
    except csv.Error:
        dialect = csv.excel
    reader = csv.reader(f, dialect)
    header = next(reader)
    cols = {}
    for i, h in enumerate(header):
        k = _key(h)
        if k and k not in cols:
            cols[k] = i
    for need in ("station", "time"):
        if need not in cols:
            raise ValueError(f"observation file: no {need!r} column (header: {', '.join(header)})")
    stations, out = {}, []

    def num(row, k):
        if k not in cols or cols[k] >= len(row):
            return None
        s = row[cols[k]].strip().replace(",", ".") if dialect.delimiter != "," else row[cols[k]].strip()
        if s.lower() in MISSING:
            return None
        try:
            return float(s)
        except ValueError:
            qc[f"file_format_{k}"] += 1
            return None
    for row in reader:
        if not row or not "".join(row).strip():
            continue
        qc["records"] += 1
        sid = row[cols["station"]].strip()
        try:
            t = parse_time(row[cols["time"]])
        except ValueError:
            qc["file_format_time"] += 1
            continue
        if sid not in stations:
            lat, lon = num(row, "lat"), num(row, "lon")
            if lat is None or lon is None:
                qc["file_station_metadata"] += 1
                continue
            stations[sid] = Station(sid, lat, lon, num(row, "elev"),
                                    name=row[cols["name"]].strip() if "name" in cols else "")
        o = Obs(sid, t, "file")
        for k in ("t2", "td2", "rh2", "wspd", "wdir", "gust", "mslp", "qnh"):
            setattr(o, k, num(row, k))
        if o.wspd is not None and o.wdir is not None and not 0 <= o.wdir <= 360:
            o.wdir = None
            qc["gross_wdir"] += 1
        p = num(row, "precip")
        if p is not None:
            h = num(row, "precip_hours")
            if h is None:
                qc["file_precip_without_hours"] += 1
            else:
                o.precip.append((int(round(h)), p))
        for k, i in cols.items():
            if k.startswith("precip_") and k.endswith("h") and k != "precip_hours":
                v = num(row, k)
                if v is not None:
                    o.precip.append((int(k[7:-1]), v))
        out.append(finish(o, qc))
    return stations, out, qc
