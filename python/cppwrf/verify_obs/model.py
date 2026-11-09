# SPDX-License-Identifier: GPL-3.0-or-later
"""A run's wrfout files at the stations.

Station positions: the fractional grid indices (i, j) of a station are found by inverting the bilinear interpolation
of the file's XLAT / XLONG (Newton iterations from the nearest grid point), so every WRF projection works the same
way. A station is inside the verification domain when spec_bdy_width <= i <= nx - 1 - spec_bdy_width (and the same
for j): the relaxation zone, where the boundary data rather than the model sets the fields, is left out.
spec_bdy_width comes from a namelist.input next to the wrfout files (or one directory up, or in ../run), else 5
(WRF's default); --boundary overrides it.

Values at a station: bilinear interpolation of the four surrounding mass points (or the nearest point). Fields: T2,
the 2 m dew point and relative humidity of wrf-python (td2, rh2), 10 m wind rotated to earth-relative with
COSALPHA / SINALPHA, sea level pressure of wrf-python (slp), QNH and the accumulated precipitation (RAINC + RAINSH +
RAINNC with the buckets; cppwrf.post.products.accumulations)."""
import glob
import math
import os
import re

import numpy as np

from ..post import products
from ..post.frame import Frame
from ..post.grib2 import reference_time

# ICAO standard atmosphere: QNH from a pressure p (hPa) at height z (m) (the altimeter-setting formula of the US
# National Weather Service, "Pressure altitude / altimeter setting", NOAA/NWS 1998: QNH = (p**n + k z)**(1/n))
QNH_N = 0.190263
QNH_K = 8.417286e-5
STD_LAPSE = 0.0065  # K m-1


def qnh(p_hpa, z):
    return (np.asarray(p_hpa, dtype=np.float64) ** QNH_N + QNH_K * np.asarray(z, dtype=np.float64)) ** (1.0 / QNH_N)


def spec_bdy_width(path):
    """spec_bdy_width of a namelist.input near a wrfout file, or None."""
    d = os.path.dirname(os.path.abspath(path))
    for cand in (os.path.join(d, "namelist.input"), os.path.join(d, "..", "namelist.input"),
                 os.path.join(d, "..", "run", "namelist.input")):
        if os.path.isfile(cand):
            with open(cand, errors="replace") as f:
                m = re.search(r"^\s*spec_bdy_width\s*=\s*(\d+)", f.read(), re.I | re.M)
            if m:
                return int(m.group(1))
    return None


def expand(files):
    out = []
    for f in ([files] if isinstance(files, str) else list(files)):
        out += sorted(glob.glob(f)) or ([f] if os.path.exists(f) else [])
    if not out:
        raise FileNotFoundError(f"no wrfout files in {files!r}")
    return list(dict.fromkeys(out))


class Location:
    __slots__ = ("station", "fi", "fj", "inside", "model_elev", "reason")

    def __init__(self, station, fi, fj, inside, model_elev, reason=""):
        self.station, self.fi, self.fj, self.inside, self.model_elev, self.reason = (station, fi, fj, inside,
                                                                                     model_elev, reason)


class Run:
    """The frames (time, file, index) of a run's wrfout files of one domain."""

    def __init__(self, files, label=None, boundary=None, rotation="cosalpha"):
        import netCDF4
        from ..wrfout import decode_times
        self.paths = expand(files)
        self.label = label or os.path.basename(os.path.dirname(os.path.abspath(self.paths[0]))) or "run"
        frames = []
        for p in self.paths:
            with netCDF4.Dataset(p) as ds:
                for i, t in enumerate(decode_times(ds["Times"][:])):
                    frames.append((t, p, i))
                if p == self.paths[0]:
                    self.attrs = {k: ds.getncattr(k) for k in ds.ncattrs()}
                    self.lat = np.asarray(ds["XLAT"][0], dtype=np.float64)
                    self.lon = np.asarray(ds["XLONG"][0], dtype=np.float64)
                    self.hgt = np.asarray(ds["HGT"][0], dtype=np.float64)
        frames.sort()
        self.frames = [f for k, f in enumerate(frames) if k == 0 or f[0] != frames[k - 1][0]]
        self.times = [f[0] for f in self.frames]
        self.ref = reference_time(self.attrs)
        self.ny, self.nx = self.lat.shape
        nl = spec_bdy_width(self.paths[0])
        self.boundary = int(boundary) if boundary is not None else (nl if nl is not None else 5)
        self.boundary_source = "option" if boundary is not None else ("namelist.input" if nl is not None
                                                                      else "WRF default")
        self.rotation = rotation

    def lead_hours(self, t):
        return (t - self.ref).total_seconds() / 3600.0

    # -- station positions -------------------------------------------------------------------------------------------
    def _bilinear(self, a, fi, fj):
        i = min(max(int(math.floor(fi)), 0), self.nx - 2)
        j = min(max(int(math.floor(fj)), 0), self.ny - 2)
        x, y = fi - i, fj - j
        return ((1 - x) * (1 - y) * a[j, i] + x * (1 - y) * a[j, i + 1] + (1 - x) * y * a[j + 1, i]
                + x * y * a[j + 1, i + 1])

    def grid_index(self, lat, lon):
        """Fractional (i, j) of a point: Newton on the bilinear interpolation of XLAT / XLONG (longitudes unwrapped
        around the point). Returns None when the point is not inside the grid (or the iteration fails)."""
        dlon = ((self.lon - lon + 180.0) % 360.0) - 180.0
        coslat = math.cos(math.radians(lat))
        d2 = (self.lat - lat) ** 2 + (dlon * coslat) ** 2
        j0, i0 = np.unravel_index(int(np.argmin(d2)), d2.shape)
        fi, fj = float(i0), float(j0)
        for _ in range(20):
            i = min(max(int(math.floor(fi)), 0), self.nx - 2)
            j = min(max(int(math.floor(fj)), 0), self.ny - 2)
            x, y = fi - i, fj - j
            la = self.lat[j:j + 2, i:i + 2]
            lo = dlon[j:j + 2, i:i + 2] * coslat
            f_la = (1 - x) * (1 - y) * la[0, 0] + x * (1 - y) * la[0, 1] + (1 - x) * y * la[1, 0] + x * y * la[1, 1]
            f_lo = (1 - x) * (1 - y) * lo[0, 0] + x * (1 - y) * lo[0, 1] + (1 - x) * y * lo[1, 0] + x * y * lo[1, 1]
            # Jacobian d(lat, lon)/d(fi, fj)
            a = (1 - y) * (la[0, 1] - la[0, 0]) + y * (la[1, 1] - la[1, 0])
            b = (1 - x) * (la[1, 0] - la[0, 0]) + x * (la[1, 1] - la[0, 1])
            c = (1 - y) * (lo[0, 1] - lo[0, 0]) + y * (lo[1, 1] - lo[1, 0])
            d = (1 - x) * (lo[1, 0] - lo[0, 0]) + x * (lo[1, 1] - lo[0, 1])
            det = a * d - b * c
            if det == 0:
                return None
            r1, r2 = lat - f_la, -f_lo  # residuals: lat_station - lat, -(lon - lon_station) cos(lat)
            di = (d * r1 - b * r2) / det
            dj = (-c * r1 + a * r2) / det
            fi, fj = fi + di, fj + dj
            if abs(di) < 1e-9 and abs(dj) < 1e-9:
                break
            if not (-2 <= fi <= self.nx + 1 and -2 <= fj <= self.ny + 1):
                return None
        if not (0 <= fi <= self.nx - 1 and 0 <= fj <= self.ny - 1):
            return None
        return fi, fj

    def bounds(self, margin=0.5):
        """(lat_min, lat_max, lon_min, lon_max) of the grid plus a margin (degrees), for a first selection."""
        lon = self.lon
        return (float(self.lat.min()) - margin, float(self.lat.max()) + margin, float(lon.min()) - margin,
                float(lon.max()) + margin)

    def locate(self, stations, max_elev_diff=None):
        """[Location] of each station; inside False (with the reason) for those outside the grid, in the boundary
        zone, or (max_elev_diff) too far from the model's terrain height."""
        la0, la1, lo0, lo1 = self.bounds()
        out = []
        bw = self.boundary
        for s in stations:
            dl = ((s.lon - lo0) % 360.0)
            if not (la0 <= s.lat <= la1) or dl > ((lo1 - lo0) % 360.0 if lo1 - lo0 < 360 else 360):
                out.append(Location(s, None, None, False, None, "outside the grid"))
                continue
            ij = self.grid_index(s.lat, s.lon)
            if ij is None:
                out.append(Location(s, None, None, False, None, "outside the grid"))
                continue
            fi, fj = ij
            zm = float(self._bilinear(self.hgt, fi, fj))
            if not (bw <= fi <= self.nx - 1 - bw and bw <= fj <= self.ny - 1 - bw):
                out.append(Location(s, fi, fj, False, zm, f"within {bw} points of the boundary"))
                continue
            if max_elev_diff is not None and s.elev is not None and abs(zm - s.elev) > max_elev_diff:
                out.append(Location(s, fi, fj, False, zm, f"|model - station elevation| > {max_elev_diff:g} m"))
                continue
            out.append(Location(s, fi, fj, True, zm))
        return out

    # -- values at the stations ----------------------------------------------------------------------------------------
    def _weights(self, locs, method):
        """(flat indices (4, n), weights (4, n)) of the stations' interpolation."""
        idx = np.zeros((4, len(locs)), dtype=np.int64)
        w = np.zeros((4, len(locs)))
        for k, L in enumerate(locs):
            if method == "nearest":
                i, j = int(round(L.fi)), int(round(L.fj))
                idx[:, k] = j * self.nx + i
                w[:, k] = (1.0, 0.0, 0.0, 0.0)
                continue
            i = min(max(int(math.floor(L.fi)), 0), self.nx - 2)
            j = min(max(int(math.floor(L.fj)), 0), self.ny - 2)
            x, y = L.fi - i, L.fj - j
            idx[:, k] = (j * self.nx + i, j * self.nx + i + 1, (j + 1) * self.nx + i, (j + 1) * self.nx + i + 1)
            w[:, k] = ((1 - x) * (1 - y), x * (1 - y), (1 - x) * y, x * y)
        return idx, w

    def sample(self, locs, method="bilinear", log=None):
        """{variable: array (time, station)} at the located stations: t2 (degC, at the model's terrain height), td2
        (degC), rh2 (%), u10 / v10 (earth-relative, m s-1), wspd (m s-1), wdir (degrees), mslp, qnh (hPa),
        precip_acc (mm since the start), model_elev (m, per station)."""
        import netCDF4
        say = log or (lambda s: None)
        idx, w = self._weights(locs, method)

        def at(a):
            return (np.asarray(a, dtype=np.float64).ravel()[idx] * w).sum(axis=0)
        names = ("t2", "td2", "rh2", "u10", "v10", "mslp", "qnh", "precip_acc")
        out = {k: np.full((len(self.frames), len(locs)), np.nan) for k in names}
        ds = None
        cur = None
        try:
            for n, (t, path, i) in enumerate(self.frames):
                if path != cur:
                    if ds is not None:
                        ds.close()
                    ds = netCDF4.Dataset(path)
                    cur = path
                F = Frame(ds, i, emulate=False)
                out["t2"][n] = at(F.raw("T2")) - 273.15
                out["td2"][n] = at(F.td2())
                out["rh2"][n] = at(F.rh2())
                u, v = F.uv10_earth(self.rotation)
                out["u10"][n], out["v10"][n] = at(u), at(v)
                out["mslp"][n] = at(F.slp())
                out["qnh"][n] = at(qnh(F.raw("PSFC").astype(np.float64) / 100.0, F.raw("HGT")))
                if F.has("RAINNC"):
                    out["precip_acc"][n] = at(products.accumulations(F)["total"])
                if n % 6 == 0:
                    say(f"  {self.label}: {t:%Y-%m-%d %H:%M} read")
        finally:
            if ds is not None:
                ds.close()
        out["wspd"] = np.hypot(out["u10"], out["v10"])
        out["wdir"] = np.degrees(np.arctan2(-out["u10"], -out["v10"])) % 360.0
        out["model_elev"] = np.array([L.model_elev for L in locs], dtype=np.float64)
        return out
