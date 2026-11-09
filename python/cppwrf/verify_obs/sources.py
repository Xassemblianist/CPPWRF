# SPDX-License-Identifier: GPL-3.0-or-later
"""Observation downloads: NOAA NCEI's ISD / Global Hourly per-station-year CSV files and the Iowa Environmental
Mesonet's ASOS / METAR download service, cached on disk.

Politeness: one request at a time, a pause between requests (default 1 s), up to four retries with exponential
back-off on HTTP 429 / 5xx and network errors (Retry-After honoured), a User-Agent naming the tool, and only the
stations inside the domain and the period of the run. Every response is cached (cache_dir, default
$CPPWRF_OBS_CACHE, else $XDG_CACHE_HOME/cppwrf/obs, else ~/.cache/cppwrf/obs) and re-used: a second verification of
the same period downloads nothing. An ISD year file is kept only for the rows in the requested period."""
import datetime
import hashlib
import os
import time
import urllib.error
import urllib.parse
import urllib.request

from .._version import __version__
from .obs import Station

USER_AGENT = f"cppwrf-verify-obs/{__version__} (+https://github.com/Xassemblianist/CPPWRF)"
ISD_HISTORY = "https://www.ncei.noaa.gov/pub/data/noaa/isd-history.csv"
# the Global Hourly files: NOAA's open-data bucket (where NCEI's /data/global-hourly/access/ redirects), then NCEI
ISD_YEAR = ("https://noaa-global-hourly-pds.s3.amazonaws.com/{year}/{id}.csv",
            "https://www.ncei.noaa.gov/data/global-hourly/access/{year}/{id}.csv")
IEM_ASOS = "https://mesonet.agron.iastate.edu/cgi-bin/request/asos.py"
IEM_BATCH = 20  # stations per IEM request


def default_cache():
    d = os.environ.get("CPPWRF_OBS_CACHE")
    if d:
        return d
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppwrf", "obs")


class NotFound(Exception):
    pass


class Fetcher:
    """Cached, paced HTTP GETs. offline=True: only the cache (a missing entry raises NotFound)."""

    def __init__(self, cache_dir=None, pause=1.0, retries=4, timeout=120.0, offline=False, log=None):
        self.cache_dir = cache_dir or default_cache()
        self.pause = float(pause)
        self.retries = int(retries)
        self.timeout = float(timeout)
        self.offline = offline
        self.log = log or (lambda s: None)
        self._last = 0.0
        self.requests = 0
        self.bytes = 0

    def path(self, name):
        return os.path.join(self.cache_dir, name)

    def cached(self, name):
        p = self.path(name)
        if os.path.exists(p):
            with open(p, encoding="utf-8", errors="replace") as f:
                return f.read()
        if os.path.exists(p + ".missing"):
            raise NotFound(name)
        return None

    def store(self, name, text):
        p = self.path(name)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p + ".part", "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(p + ".part", p)

    def _open(self, url):
        """The response of a GET, paced and retried; NotFound on 404 / 403."""
        if self.offline:
            raise NotFound(url)
        delay = max(self.pause, 1.0)
        for attempt in range(self.retries + 1):
            wait = self._last + self.pause - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            self._last = time.monotonic()
            self.requests += 1
            try:
                req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
                return urllib.request.urlopen(req, timeout=self.timeout)
            except urllib.error.HTTPError as e:
                if e.code in (403, 404):
                    raise NotFound(url) from e
                if e.code not in (429, 500, 502, 503, 504) or attempt == self.retries:
                    raise
                ra = e.headers.get("Retry-After") if e.headers else None
                sleep = float(ra) if ra and ra.isdigit() else delay
            except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
                if attempt == self.retries:
                    raise
                sleep = delay
                self.log(f"  network error ({e}); retrying in {sleep:.0f} s")
            time.sleep(sleep)
            delay *= 2
        raise RuntimeError("unreachable")

    def get(self, url, name):
        """The text at url, from the cache entry `name` when there is one."""
        text = self.cached(name)
        if text is not None:
            return text
        try:
            with self._open(url) as r:
                data = r.read()
        except NotFound:
            if not self.offline:
                self.store(name + ".missing", url)
            raise
        self.bytes += len(data)
        text = data.decode("utf-8", errors="replace")
        self.store(name, text)
        return text

    def get_filtered(self, url, name, keep):
        """Like get, for a large CSV read line by line: only the header and the lines keep(line) accepts are kept
        (and cached)."""
        text = self.cached(name)
        if text is not None:
            return text
        try:
            r = self._open(url)
        except NotFound:
            if not self.offline:
                self.store(name + ".missing", url)
            raise
        lines = []
        with r:
            for i, raw in enumerate(r):
                self.bytes += len(raw)
                line = raw.decode("utf-8", errors="replace")
                if i == 0 or keep(line):
                    lines.append(line)
        text = "".join(lines)
        self.store(name, text)
        return text


def catalogue(fetcher, max_age_days=60):
    """The stations of NCEI's ISD station history (isd-history.csv): [Station] with id USAF+WBAN (the Global Hourly
    file name), name, ICAO, country (FIPS), lat / lon / elevation, and .begin / .end (dates of record)."""
    name = "isd-history.csv"
    p = fetcher.path(name)
    if os.path.exists(p) and time.time() - os.path.getmtime(p) > max_age_days * 86400 and not fetcher.offline:
        os.remove(p)
    import csv
    import io
    out = []
    for row in csv.DictReader(io.StringIO(fetcher.get(ISD_HISTORY, name))):
        try:
            lat, lon = float(row["LAT"]), float(row["LON"])
        except (KeyError, ValueError):
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        try:
            elev = float(row["ELEV(M)"])
        except (KeyError, ValueError):
            elev = None
        if elev is not None and elev <= -999:
            elev = None
        s = Station(row["USAF"] + row["WBAN"], lat, lon, elev, name=row.get("STATION NAME", "").strip(),
                    icao=row.get("ICAO", "").strip(), country=row.get("CTRY", "").strip())
        s.begin = row.get("BEGIN", "")
        s.end = row.get("END", "")
        out.append(s)
    return out


def isd(fetcher, station_ids, start, end):
    """ISD Global Hourly text of each station (one CSV text per station and year), rows within [start, end]."""
    lo, hi = start.strftime("%Y-%m-%dT%H:%M:%S"), end.strftime("%Y-%m-%dT%H:%M:%S")

    def keep(line):
        # "STATION","DATE",...: the DATE is the second quoted field
        d = line.split(",", 2)[1].strip('"') if line.count(",") >= 2 else ""
        return lo <= d <= hi
    texts = []
    for sid in station_ids:
        for year in range(start.year, end.year + 1):
            name = f"isd/{sid}_{year}_{start:%Y%m%d%H}-{end:%Y%m%d%H}.csv"
            for tmpl in ISD_YEAR:
                try:
                    texts.append(fetcher.get_filtered(tmpl.format(year=year, id=sid), name, keep))
                    break
                except NotFound:
                    continue
    return texts


def iem(fetcher, icaos, start, end):
    """IEM ASOS / METAR CSV texts (data=metar with lat / lon / elevation) of the stations, in batches."""
    texts = []
    icaos = sorted(set(icaos))
    t1 = end + datetime.timedelta(minutes=1)
    for k in range(0, len(icaos), IEM_BATCH):
        batch = icaos[k:k + IEM_BATCH]
        q = [("station", s) for s in batch] + [
            ("data", "metar"), ("year1", start.year), ("month1", start.month), ("day1", start.day),
            ("hour1", start.hour), ("minute1", start.minute), ("year2", t1.year), ("month2", t1.month),
            ("day2", t1.day), ("hour2", t1.hour), ("minute2", t1.minute), ("tz", "Etc/UTC"), ("format", "onlycomma"),
            ("latlon", "yes"), ("elev", "yes"), ("missing", "M"), ("trace", "T"), ("direct", "no"),
            ("report_type", "3"), ("report_type", "4")]
        url = IEM_ASOS + "?" + urllib.parse.urlencode(q)
        key = hashlib.sha1(",".join(batch).encode()).hexdigest()[:12]
        name = f"iem/{start:%Y%m%d%H%M}-{end:%Y%m%d%H%M}_{key}.csv"
        try:
            texts.append(fetcher.get(url, name))
        except NotFound:
            continue
    return texts
