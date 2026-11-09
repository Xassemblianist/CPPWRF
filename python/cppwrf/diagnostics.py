# SPDX-License-Identifier: GPL-3.0-or-later
"""Standard diagnostics of wrfout data (an xarray Dataset or WrfoutData from cppwrf.open_wrfout).

  t2_celsius(ds)              2 m temperature in degrees Celsius
  wind10(ds, earth=True)      10 m wind speed (m s-1) and meteorological direction (degrees, from), earth-relative with
                              COSALPHA / SINALPHA
  slp(ds)                     sea-level pressure (hPa) by WRF's / RIP's method, as wrf-python's getvar("slp")
  precipitation(ds)           accumulated precipitation (mm): RAINC + RAINNC (+ RAINSH) with the bucket counts
  precip_accum(ds, hours=1)   precipitation over the hours before each frame (mm; NaN where that frame is missing)

With xarray data the results are DataArrays on the input's coordinates, otherwise numpy arrays (Time first)."""
import datetime
from typing import Tuple

import numpy as np

# wrf-python's wrf_constants (fortran/wrf_constants.f90): the values its slp and tk use
G = 9.81
RD = 287.0
CP = 7.0 * RD / 2.0
P1000MB = 100000.0
USSALR = 0.0065  # US standard atmosphere lapse rate, K m-1
TC = 273.16 + 17.5
PCONST = 10000.0


def _get(ds, name):
    return np.asarray(ds[name].values if hasattr(ds[name], "values") else ds[name], dtype=np.float64)


def _xr(ds):
    return hasattr(ds, "data_vars")


def _wrap(ds, data, like, name, units, description=""):
    """data on the dimensions and coordinates of variable `like` (xarray input), else the array."""
    if not _xr(ds):
        return data
    import xarray as xr
    ref = ds[like]
    return xr.DataArray(data, dims=ref.dims, coords=ref.coords, name=name,
                        attrs={"units": units, "description": description})


def _times(ds):
    if _xr(ds):
        t = ds["Time"].values
        return [x if isinstance(x, datetime.datetime) else x.astype("datetime64[us]").astype(datetime.datetime)
                for x in t]
    return list(ds.times)


def t2_celsius(ds):
    return _wrap(ds, _get(ds, "T2") - 273.15, "T2", "T2C", "degC", "2 m temperature")


def wind10(ds, earth: bool = True) -> Tuple[object, object]:
    """(speed, direction) of the 10 m wind; direction in degrees the wind blows from (0 = north, 90 = east), earth
    relative when the file has COSALPHA / SINALPHA and earth is True (else relative to the grid)."""
    u, v = _get(ds, "U10"), _get(ds, "V10")
    has_rot = "COSALPHA" in ds and "SINALPHA" in ds if hasattr(ds, "__contains__") else False
    if earth and has_rot:
        ca, sa = _get(ds, "COSALPHA"), _get(ds, "SINALPHA")
        u, v = u * ca - v * sa, v * ca + u * sa
    spd = np.sqrt(u * u + v * v)
    wdir = np.mod(270.0 - np.degrees(np.arctan2(v, u)), 360.0)
    return (_wrap(ds, spd, "U10", "WSPD10", "m s-1", "10 m wind speed"),
            _wrap(ds, wdir, "U10", "WDIR10", "degrees", "10 m wind direction (from)"))


def seaprs(z: np.ndarray, t: np.ndarray, p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Sea-level pressure (hPa) of columns (nz, ny, nx): wrf-python's DCOMPUTESEAPRS (fortran/wrf_user.f90, from RIP),
    statement for statement: height z (m), temperature t (K), pressure p (Pa), water vapour mixing ratio q (kg kg-1),
    level 0 the lowest. Its interpolation to the level PCONST above the surface multiplies by log(plo / phi), as the
    original does."""
    nz = p.shape[0]
    below = p < (p[0] - PCONST)[None]
    if not below.any(axis=0).all():
        raise ValueError("slp: error in finding 100 hPa up (a column without a level 100 hPa above its surface)")
    level = below.argmax(axis=0) + 1  # Fortran's 1-based level
    klo = np.maximum(level - 1, 1)
    khi = np.minimum(klo + 1, nz - 1)
    if (klo == khi).any():
        raise ValueError("slp: trapping levels are weird (klo == khi)")
    take = lambda a, k: np.take_along_axis(a, (k - 1)[None], axis=0)[0]  # noqa: E731
    plo, phi = take(p, klo), take(p, khi)
    tlo = take(t, klo) * (1.0 + 0.608 * take(q, klo))
    thi = take(t, khi) * (1.0 + 0.608 * take(q, khi))
    zlo, zhi = take(z, klo), take(z, khi)
    p_at_pconst = p[0] - PCONST
    t_at_pconst = thi - (thi - tlo) * np.log(p_at_pconst / phi) * np.log(plo / phi)
    z_at_pconst = zhi - (zhi - zlo) * np.log(p_at_pconst / phi) * np.log(plo / phi)
    t_surf = t_at_pconst * (p[0] / p_at_pconst) ** (USSALR * RD / G)
    t_sea_level = t_at_pconst + USSALR * z_at_pconst
    # ridiculous_mm5_test
    l1 = t_sea_level < TC
    l2 = t_surf <= TC
    l3 = ~l1
    t_sea_level = np.where(l2 & l3, TC, TC - 0.005 * (t_surf - TC) ** 2)
    return 0.01 * (p[0] * np.exp((2.0 * G * z[0]) / (RD * (t_sea_level + t_surf))))


def slp(ds):
    """Sea-level pressure (hPa) of every frame, as wrf-python's getvar(..., "slp"): height (PH + PHB) / G at the mass
    levels, temperature from T + 300 and P + PB, QVAPOR below 0 set to 0."""
    p = _get(ds, "P") + _get(ds, "PB")
    tk = (_get(ds, "T") + 300.0) * (p / P1000MB) ** (RD / CP)
    ph = (_get(ds, "PH") + _get(ds, "PHB")) / G
    z = 0.5 * (ph[:, :-1] + ph[:, 1:])  # destaggered (bottom_top_stag -> bottom_top)
    q = np.maximum(_get(ds, "QVAPOR"), 0.0)
    out = np.stack([seaprs(z[n], tk[n], p[n], q[n]) for n in range(p.shape[0])])
    return _wrap(ds, out, "PSFC" if "PSFC" in ds else "T2", "SLP", "hPa", "sea level pressure")


def precipitation(ds):
    """Accumulated precipitation since the simulation start (mm): RAINC + RAINNC (+ RAINSH), with the bucket counts
    I_RAINC / I_RAINNC times the BUCKET_MM global attribute (bucket_mm > 0)."""
    total = _get(ds, "RAINNC").copy()
    for v in ("RAINC", "RAINSH"):
        if v in ds:
            total += _get(ds, v)
    attrs = ds.attrs if hasattr(ds, "attrs") else {}
    bucket = float(attrs.get("BUCKET_MM", 0.0) or 0.0)
    if bucket > 0:
        for v in ("I_RAINC", "I_RAINNC"):
            if v in ds:
                total += _get(ds, v) * bucket
    return _wrap(ds, total, "RAINNC", "PRECIP", "mm", "accumulated total precipitation")


def precip_accum(ds, hours: float = 1.0):
    """Precipitation (mm) over the `hours` before each frame: the accumulation's difference with the frame that time
    earlier (NaN where there is no such frame, as the first ones). hours = 1: hourly precipitation."""
    acc = np.asarray(precipitation(ds))
    times = _times(ds)
    index = {t: k for k, t in enumerate(times)}
    out = np.full_like(acc, np.nan)
    dt = datetime.timedelta(hours=hours)
    for k, t in enumerate(times):
        j = index.get(t - dt)
        if j is not None:
            out[k] = acc[k] - acc[j]
    return _wrap(ds, out, "RAINNC", f"PRECIP_{hours:g}H", "mm", f"precipitation in the last {hours:g} h")
