# SPDX-License-Identifier: GPL-3.0-or-later
"""The fields cppwrf-post writes: name, units, CF standard name, GRIB2 identification (WMO Code Table 4.2 discipline,
category, number; Table 4.5 level types) and how each is computed from a Frame.

Units: each field is computed in its native unit (`units`); NetCDF output converts to the configured unit, GRIB2 to
`grib["units"]`. Missing values are NaN."""
import numpy as np

from . import wrfpy as w

K0 = 273.15


class Product:
    def __init__(self, name, long_name, units, compute, kind="2d", standard_name=None, grib=None, precision=2,
                 needs=(), height=None):
        self.name = name
        self.long_name = long_name
        self.units = units
        self.compute = compute  # (ctx) -> float64 array, NaN where missing
        self.kind = kind  # "2d", "pressure" or "height"
        self.standard_name = standard_name
        self.grib = grib  # dict or None (not written to GRIB2)
        self.precision = precision  # GRIB2 decimal scale factor
        self.needs = needs  # wrfout variables it needs
        self.height = height  # m above ground of a 2 m / 10 m field (a CF scalar coordinate)

    def available(self, frame):
        return all(frame.has(v) for v in self.needs)


def _nan(a):
    a = np.asarray(a, dtype=np.float64)
    return np.where(np.abs(a) > 1e30, np.nan, a)


def _g(d, c, n, units, level=(1, 0, 0), level2=None, stat=None, sign=1.0, add_terrain=False):
    """GRIB2 identification: discipline, category, number (Code Table 4.2), WMO units, first and second fixed surface
    (type, scale factor, scaled value; value None: the field's level), statistical processing (Template 4.8: "acc"
    since the start, "acc_interval" over precip_hours, "max_output" over the output interval) or None (Template 4.0);
    sign -1: written negated (CIN, negative in GRIB2 as NCEP writes it); add_terrain: the terrain height added (a
    height above ground written as GRIB2's geopotential height)."""
    return {"discipline": d, "category": c, "number": n, "units": units, "level": level, "level2": level2,
            "stat": stat, "sign": sign, "add_terrain": add_terrain}


# -- unit conversions --------------------------------------------------------------------------------------------------

_SCALE = {  # (from, to): factor; offsets below
    ("hPa", "Pa"): 100.0, ("Pa", "hPa"): 0.01, ("m s-1", "km h-1"): 3.6, ("km h-1", "m s-1"): 1 / 3.6,
    ("m s-1", "kt"): 3600.0 / 1852.0, ("kt", "m s-1"): 1852.0 / 3600.0, ("1", "%"): 100.0, ("%", "1"): 0.01,
    ("m", "km"): 0.001, ("km", "m"): 1000.0, ("kg m-2", "mm"): 1.0, ("mm", "kg m-2"): 1.0,
    ("1e-5 s-1", "s-1"): 1e-5, ("s-1", "1e-5 s-1"): 1e5, ("dBZ", "dB"): 1.0, ("dB", "dBZ"): 1.0,
    ("m", "cm"): 100.0, ("cm", "m"): 0.01, ("kg kg-1", "g kg-1"): 1000.0, ("g kg-1", "kg kg-1"): 0.001,
}
_ALIAS = {"degC": "degC", "C": "degC", "celsius": "degC", "K": "K", "m/s": "m s-1", "km/h": "km h-1", "mm": "mm",
          "knots": "kt", "knot": "kt"}


def convert(a, src, dst):
    """a (NaN for missing) from unit src to dst."""
    src, dst = _ALIAS.get(src, src), _ALIAS.get(dst, dst)
    if src == dst:
        return a
    if (src, dst) == ("K", "degC"):
        return a - K0
    if (src, dst) == ("degC", "K"):
        return a + K0
    if (src, dst) == ("degC", "degF"):
        return a * 1.8 + 32.0
    if (src, dst) == ("K", "degF"):
        return (a - K0) * 1.8 + 32.0
    if (src, dst) in _SCALE:
        return a * _SCALE[(src, dst)]
    raise ValueError(f"no conversion from {src} to {dst}")


# -- precipitation -----------------------------------------------------------------------------------------------------


def accumulations(frame):
    """{kind: accumulated amount since the start (kg m-2)} of a frame: total, convective (RAINC + RAINSH), grid
    (RAINNC), snow (SNOWNC), graupel (GRAUPELNC), hail (HAILNC); the bucket counts I_RAINC / I_RAINNC times the
    BUCKET_MM attribute added."""
    def get(name):
        return frame.raw(name).astype(np.float64) if frame.has(name) else None
    bucket = float(frame.attr("BUCKET_MM", 0.0) or 0.0)
    conv = np.zeros(frame.raw("RAINNC").shape)
    for v in ("RAINC", "RAINSH"):
        if frame.has(v):
            conv = conv + get(v)
    grid = get("RAINNC")
    if bucket > 0:
        if frame.has("I_RAINC"):
            conv = conv + get("I_RAINC") * bucket
        if frame.has("I_RAINNC"):
            grid = grid + get("I_RAINNC") * bucket
    out = {"conv": conv, "grid": grid, "total": conv + grid}
    zero = np.zeros(grid.shape)
    out["snow"] = get("SNOWNC") if frame.has("SNOWNC") else zero
    out["graupel"] = get("GRAUPELNC") if frame.has("GRAUPELNC") else zero
    out["hail"] = get("HAILNC") if frame.has("HAILNC") else zero
    return out


def ptype(d, t2):
    """Precipitation type (WMO Code Table 4.201) from the model's own accumulations over the interval, d (as
    accumulations()): no wrf-python routine exists for it, so this is CPPWRF's rule, documented in docs/POST.md.
    Where total >= 0.1 mm: the frozen fraction f = (snow + graupel + hail) / grid-scale precipitation;
    f >= 0.9: 5 snow, or 9 graupel when graupel + hail exceed snow; 0.1 < f < 0.9: 7 rain and snow; else 3 freezing
    rain where T2 < 0 degC, 1 rain. NaN where the total is below 0.1 mm or the interval is missing."""
    tot = d["total"]
    frozen = d["snow"] + d["graupel"] + d["hail"]
    with np.errstate(invalid="ignore", divide="ignore"):
        f = np.where(d["grid"] > 0, np.clip(frozen / d["grid"], 0.0, 1.0), 0.0)
    out = np.where(t2 < K0, 3.0, 1.0)
    out = np.where(f > 0.1, 7.0, out)
    out = np.where(f >= 0.9, np.where(d["graupel"] + d["hail"] > d["snow"], 9.0, 5.0), out)
    return np.where(np.isfinite(tot) & (tot >= 0.1), out, np.nan)


# -- the catalogue -----------------------------------------------------------------------------------------------------


def _uv10(ctx):
    return ctx.frame.uv10_earth(ctx.opt["wind_rotation"])


def _cape(ctx, which, i):
    F = ctx.frame
    if which == "sb":
        return _nan(F.cape_sb()[i])
    return _nan(F.cape_2d(which, float(ctx.opt["ml_depth_hpa"]))[i])


def _maxdbz(ctx):
    F = ctx.frame
    src = ctx.opt["max_dbz"]
    if src == "model" or (src == "auto" and F.has("REFL_10CM")):
        return F.raw("REFL_10CM").astype(np.float64).max(axis=0)
    return F.dbz(ctx.opt["dbz_varint"], ctx.opt["dbz_liqskin"]).astype(np.float64).max(axis=0)


def _cloud(ctx, i):
    return _nan(ctx.frame.cloudfrac(ctx.opt["cloud_vert_type"])[i]) * 100.0


def _acc(kind):
    return lambda ctx: ctx.acc[kind]


def _interval(kind):
    return lambda ctx: ctx.interval[kind] if ctx.interval is not None else np.full(ctx.shape2, np.nan)


def surface_products(opt):
    h = int(opt.get("precip_hours", 1))
    H = f"{h}H"
    agl = lambda v: (103, 0, v)  # noqa: E731
    acc0 = "acc"
    acci = "acc_interval"
    P = [
        Product("T2", "2 m temperature", "K", lambda c: c.frame.raw("T2").astype(np.float64), standard_name=
                "air_temperature", grib=_g(0, 0, 0, "K", agl(2)), height=2.0, needs=("T2",)),
        Product("T2C", "2 m temperature", "degC", lambda c: c.frame.raw("T2").astype(np.float64) - K0,
                standard_name="air_temperature", height=2.0, needs=("T2",)),
        Product("TD2", "2 m dew point temperature (wrf-python td2)", "degC",
                lambda c: c.frame.td2().astype(np.float64), standard_name="dew_point_temperature",
                grib=_g(0, 0, 6, "K", agl(2)), height=2.0, needs=("Q2", "PSFC")),
        Product("RH2", "2 m relative humidity (wrf-python rh2)", "%", lambda c: c.frame.rh2().astype(np.float64),
                standard_name="relative_humidity", grib=_g(0, 1, 1, "%", agl(2)), precision=1, height=2.0,
                needs=("Q2", "T2", "PSFC")),
        Product("U10", "10 m wind, eastward (earth-relative)", "m s-1", lambda c: _uv10(c)[0],
                standard_name="eastward_wind", grib=_g(0, 2, 2, "m s-1", agl(10)), height=10.0,
                needs=("U10", "V10")),
        Product("V10", "10 m wind, northward (earth-relative)", "m s-1", lambda c: _uv10(c)[1],
                standard_name="northward_wind", grib=_g(0, 2, 3, "m s-1", agl(10)), height=10.0,
                needs=("U10", "V10")),
        Product("WSPD10", "10 m wind speed", "m s-1", lambda c: w.wspd(*_uv10(c)), standard_name="wind_speed",
                grib=_g(0, 2, 1, "m s-1", agl(10)), height=10.0, needs=("U10", "V10")),
        Product("WDIR10", "10 m wind direction (from, degrees from north)", "degree", lambda c: w.wdir(*_uv10(c)),
                standard_name="wind_from_direction", grib=_g(0, 2, 0, "degree", agl(10)), precision=0, height=10.0,
                needs=("U10", "V10")),
        Product("GUST10", "10 m maximum wind speed in the output interval (WSPD10MAX)", "m s-1",
                lambda c: c.frame.raw("WSPD10MAX").astype(np.float64), standard_name="wind_speed_of_gust",
                grib=_g(0, 2, 22, "m s-1", agl(10), stat="max_output"), height=10.0, needs=("WSPD10MAX",)),
        Product("MSLP", "sea-level pressure (wrf-python slp)", "hPa", lambda c: c.frame.slp().astype(np.float64),
                standard_name="air_pressure_at_mean_sea_level", grib=_g(0, 3, 1, "Pa", (101, 0, 0)), precision=0,
                needs=("P", "PB", "T", "QVAPOR", "PH", "PHB")),
        Product("PSFC", "surface pressure", "Pa", lambda c: c.frame.raw("PSFC").astype(np.float64),
                standard_name="surface_air_pressure", grib=_g(0, 3, 0, "Pa"), precision=0, needs=("PSFC",)),
        Product("HGT", "terrain height", "m", lambda c: c.frame.raw("HGT").astype(np.float64),
                standard_name="surface_altitude", grib=_g(0, 3, 5, "m"), precision=1, needs=("HGT",)),
        Product("PRECIP_ACC", "total precipitation since the start", "kg m-2", _acc("total"),
                standard_name="precipitation_amount", grib=_g(0, 1, 8, "kg m-2", stat=acc0), needs=("RAINNC",)),
        Product("PRECIP_CONV_ACC", "convective precipitation since the start (RAINC + RAINSH)", "kg m-2",
                _acc("conv"), standard_name="convective_precipitation_amount", grib=_g(0, 1, 10, "kg m-2", stat=acc0),
                needs=("RAINNC",)),
        Product("PRECIP_GRID_ACC", "grid-scale precipitation since the start (RAINNC)", "kg m-2", _acc("grid"),
                standard_name="large_scale_precipitation_amount", grib=_g(0, 1, 9, "kg m-2", stat=acc0),
                needs=("RAINNC",)),
        Product(f"PRECIP_{H}", f"total precipitation in the last {h} h", "kg m-2", _interval("total"),
                standard_name="precipitation_amount", grib=_g(0, 1, 8, "kg m-2", stat=acci), needs=("RAINNC",)),
        Product(f"PRECIP_CONV_{H}", f"convective precipitation in the last {h} h", "kg m-2", _interval("conv"),
                standard_name="convective_precipitation_amount", grib=_g(0, 1, 10, "kg m-2", stat=acci),
                needs=("RAINNC",)),
        Product(f"PRECIP_GRID_{H}", f"grid-scale precipitation in the last {h} h", "kg m-2", _interval("grid"),
                standard_name="large_scale_precipitation_amount", grib=_g(0, 1, 9, "kg m-2", stat=acci),
                needs=("RAINNC",)),
        Product("SNOW_ACC", "grid-scale snow and ice since the start, water equivalent (SNOWNC)", "kg m-2",
                _acc("snow"), standard_name="large_scale_snowfall_amount", grib=_g(0, 1, 15, "kg m-2", stat=acc0),
                needs=("SNOWNC",)),
        Product(f"SNOW_{H}", f"grid-scale snow and ice in the last {h} h, water equivalent", "kg m-2",
                _interval("snow"), standard_name="large_scale_snowfall_amount",
                grib=_g(0, 1, 15, "kg m-2", stat=acci), needs=("SNOWNC",)),
        Product("GRAUPEL_ACC", "grid-scale graupel since the start (GRAUPELNC)", "kg m-2", _acc("graupel"),
                grib=_g(0, 1, 75, "kg m-2", stat=acc0), needs=("GRAUPELNC",)),
        Product(f"GRAUPEL_{H}", f"grid-scale graupel in the last {h} h", "kg m-2", _interval("graupel"),
                grib=_g(0, 1, 75, "kg m-2", stat=acci), needs=("GRAUPELNC",)),
        Product("PTYPE", f"precipitation type in the last {h} h (WMO Code Table 4.201: 1 rain, 3 freezing rain, "
                "5 snow, 7 rain and snow, 9 graupel)", "1",
                lambda c: ptype(c.interval, c.frame.raw("T2").astype(np.float64)) if c.interval is not None
                else np.full(c.shape2, np.nan), grib=_g(0, 1, 19, "1"), precision=0, needs=("RAINNC", "T2")),
        Product("SNOWH", "snow depth", "m", lambda c: c.frame.raw("SNOWH").astype(np.float64),
                standard_name="surface_snow_thickness", grib=_g(0, 1, 11, "m"), precision=3, needs=("SNOWH",)),
        Product("SWE", "snow water equivalent", "kg m-2", lambda c: c.frame.raw("SNOW").astype(np.float64),
                standard_name="surface_snow_amount", grib=_g(0, 1, 60, "kg m-2"), precision=1, needs=("SNOW",)),
        Product("CLOUD_LOW", "low cloud fraction (wrf-python cloudfrac)", "%", lambda c: _cloud(c, 0),
                standard_name="low_type_cloud_area_fraction", grib=_g(0, 6, 3, "%", (214, 0, 0)), precision=0,
                needs=("QVAPOR",)),
        Product("CLOUD_MID", "mid cloud fraction (wrf-python cloudfrac)", "%", lambda c: _cloud(c, 1),
                standard_name="medium_type_cloud_area_fraction", grib=_g(0, 6, 4, "%", (224, 0, 0)), precision=0,
                needs=("QVAPOR",)),
        Product("CLOUD_HIGH", "high cloud fraction (wrf-python cloudfrac)", "%", lambda c: _cloud(c, 2),
                standard_name="high_type_cloud_area_fraction", grib=_g(0, 6, 5, "%", (234, 0, 0)), precision=0,
                needs=("QVAPOR",)),
        Product("CLOUD_TOTAL", "total cloud fraction: the largest of the low, mid and high fractions (maximum "
                "overlap)", "%", lambda c: np.fmax(np.fmax(_cloud(c, 0), _cloud(c, 1)), _cloud(c, 2)),
                standard_name="cloud_area_fraction", grib=_g(0, 6, 1, "%", (10, 0, 0)), precision=0,
                needs=("QVAPOR",)),
        Product("PW", "precipitable water (wrf-python pw)", "kg m-2", lambda c: c.frame.pw().astype(np.float64),
                standard_name="atmosphere_mass_content_of_water_vapor", grib=_g(0, 1, 3, "kg m-2", (10, 0, 0)),
                needs=("QVAPOR",)),
        Product("CAPE_SB", "surface-based CAPE: parcel at the lowest model level (wrf-python cape_3d)", "J kg-1",
                lambda c: _cape(c, "sb", 0), standard_name=
                "atmosphere_convective_available_potential_energy_wrt_surface", grib=_g(0, 7, 6, "J kg-1"),
                precision=0, needs=("QVAPOR",)),
        Product("CIN_SB", "surface-based CIN, positive; missing where CAPE < 100 J kg-1 (wrf-python cape_3d)",
                "J kg-1", lambda c: _cape(c, "sb", 1), standard_name="atmosphere_convective_inhibition_wrt_surface",
                grib=_g(0, 7, 7, "J kg-1", sign=-1.0), precision=0, needs=("QVAPOR",)),
        Product("CAPE_ML", "mixed-layer CAPE: parcel mixed over the lowest 100 hPa (wrf-python's capecalc2d)",
                "J kg-1", lambda c: _cape(c, "ml", 0), standard_name=
                "atmosphere_convective_available_potential_energy", grib=_g(0, 7, 6, "J kg-1", (108, 0, 10000),
                                                                             (108, 0, 0)),
                precision=0, needs=("QVAPOR",)),
        Product("CIN_ML", "mixed-layer CIN, positive; missing where CAPE < 100 J kg-1", "J kg-1",
                lambda c: _cape(c, "ml", 1), standard_name="atmosphere_convective_inhibition",
                grib=_g(0, 7, 7, "J kg-1", (108, 0, 10000), (108, 0, 0), sign=-1.0), precision=0, needs=("QVAPOR",)),
        Product("CAPE_MU", "most-unstable CAPE: max theta-e parcel in the lowest 3 km (wrf-python cape_2d mcape)",
                "J kg-1", lambda c: _cape(c, "mu", 0), standard_name=
                "atmosphere_convective_available_potential_energy", grib=_g(0, 7, 6, "J kg-1", (103, 0, 3000),
                                                                             (103, 0, 0)),
                precision=0, needs=("QVAPOR",)),
        Product("CIN_MU", "most-unstable CIN, positive; missing where CAPE < 100 J kg-1 (wrf-python cape_2d mcin)",
                "J kg-1", lambda c: _cape(c, "mu", 1), standard_name="atmosphere_convective_inhibition",
                grib=_g(0, 7, 7, "J kg-1", (103, 0, 3000), (103, 0, 0), sign=-1.0), precision=0, needs=("QVAPOR",)),
        Product("LCL", "lifting condensation level of the most-unstable parcel, above ground (wrf-python cape_2d "
                "lcl)", "m", lambda c: _cape(c, "mu", 2), grib=_g(0, 3, 5, "m", (5, 0, 0), add_terrain=True),
                precision=0, needs=("QVAPOR",)),
        Product("LFC", "level of free convection of the most-unstable parcel, above ground (wrf-python cape_2d "
                "lfc)", "m", lambda c: _cape(c, "mu", 3), grib=_g(0, 3, 5, "m", (14, 0, 0), add_terrain=True),
                precision=0, needs=("QVAPOR",)),
        Product("SRH01", "0-1 km storm-relative helicity (wrf-python helicity)", "m2 s-2",
                lambda c: c.frame.srh(1000.0).astype(np.float64), grib=_g(0, 7, 8, "m2 s-2", (103, 0, 1000),
                                                                         (103, 0, 0)), precision=1, needs=("U",)),
        Product("SRH03", "0-3 km storm-relative helicity (wrf-python helicity)", "m2 s-2",
                lambda c: c.frame.srh(3000.0).astype(np.float64), grib=_g(0, 7, 8, "m2 s-2", (103, 0, 3000),
                                                                         (103, 0, 0)), precision=1, needs=("U",)),
        Product("UH25", "2-5 km updraft helicity (wrf-python updraft_helicity)", "m2 s-2",
                lambda c: c.frame.uh(2000.0, 5000.0).astype(np.float64),
                grib=_g(0, 7, 15, "m2 s-2", (103, 0, 5000), (103, 0, 2000)), precision=2, needs=("W",)),
        Product("FZLEV", "freezing level: the highest 0 degC crossing, above sea level (wrf-python interplevel of z "
                "on tc)", "m", lambda c: _nan(c.frame.interplevel(c.frame.height(), c.frame.tk() - K0, 0.0)[0]),
                standard_name="freezing_level_altitude", grib=_g(0, 3, 5, "m", (204, 0, 0)), precision=0,
                needs=("T",)),
        Product("MAXDBZ", "maximum reflectivity in the column (REFL_10CM, or wrf-python mdbz)", "dBZ", _maxdbz,
                standard_name="equivalent_reflectivity_factor", grib=_g(0, 16, 5, "dB", (10, 0, 0)), precision=1,
                needs=("QRAIN",)),
        Product("CTT", "cloud-top temperature (wrf-python ctt; the lowest level's where there is no cloud)", "degC",
                lambda c: _nan(c.frame.ctt()), standard_name="air_temperature_at_cloud_top",
                grib=_g(0, 0, 0, "K", (3, 0, 0)), precision=1, needs=("QCLOUD",)),
    ]
    return P


def _p(ctx, field, field_type=None):
    F = ctx.frame
    return _nan(F.vinterp(field, "pressure", ctx.plevels, field_type, log_p=ctx.opt["log_p"],
                          extrapolate=ctx.opt["extrapolate"]))


def _uvp(ctx, i):
    def f():
        u, v = ctx.frame.uv_earth(ctx.opt["wind_rotation"])
        dt = ctx.frame.raw("U").dtype if ctx.opt["emulate_wrfpython"] else np.float64
        return u.astype(dt), v.astype(dt)
    if "uv_e" not in ctx.memo:
        ctx.memo["uv_e"] = f()
    return ctx.memo["uv_e"][i]


def pressure_products(opt):
    lv = (100, 0, None)  # scaled value: the level in Pa
    return [
        Product("T", "temperature", "K", lambda c: _p(c, c.frame.tk(), "tk"), "pressure", "air_temperature",
                _g(0, 0, 0, "K", lv)),
        Product("TD", "dew point temperature (wrf-python td)", "degC", lambda c: _p(c, c.frame.td()), "pressure",
                "dew_point_temperature", _g(0, 0, 6, "K", lv)),
        Product("RH", "relative humidity (wrf-python rh)", "%", lambda c: _p(c, c.frame.rh()), "pressure",
                "relative_humidity", _g(0, 1, 1, "%", lv), precision=1),
        Product("Q", "water vapour mixing ratio (QVAPOR)", "kg kg-1", lambda c: _p(c, c.frame.raw("QVAPOR")),
                "pressure", "humidity_mixing_ratio", _g(0, 1, 2, "kg kg-1", lv), precision=6),
        Product("GHT", "geopotential height", "m", lambda c: _p(c, c.frame.height(), "ght"), "pressure",
                "geopotential_height", _g(0, 3, 5, "m", lv), precision=1),
        Product("U", "wind, eastward (earth-relative)", "m s-1", lambda c: _p(c, _uvp(c, 0)), "pressure",
                "eastward_wind", _g(0, 2, 2, "m s-1", lv)),
        Product("V", "wind, northward (earth-relative)", "m s-1", lambda c: _p(c, _uvp(c, 1)), "pressure",
                "northward_wind", _g(0, 2, 3, "m s-1", lv)),
        Product("W", "vertical velocity", "m s-1", lambda c: _p(c, _destag_w(c)), "pressure",
                "upward_air_velocity", _g(0, 2, 9, "m s-1", lv), precision=3),
        Product("OMEGA", "omega (wrf-python omg)", "Pa s-1", lambda c: _p(c, c.frame.omega()), "pressure",
                "lagrangian_tendency_of_air_pressure", _g(0, 2, 8, "Pa s-1", lv), precision=3),
        Product("AVO", "absolute vorticity (wrf-python avo)", "1e-5 s-1", lambda c: _p(c, c.frame.avo()), "pressure",
                "atmosphere_upward_absolute_vorticity", _g(0, 2, 10, "s-1", lv), precision=7),
        Product("DBZ", "reflectivity (wrf-python dbz)", "dBZ",
                lambda c: _p(c, c.frame.dbz(c.opt["dbz_varint"], c.opt["dbz_liqskin"])), "pressure",
                "equivalent_reflectivity_factor", _g(0, 16, 4, "dB", lv), precision=1),
    ]


def _destag_w(ctx):
    from .frame import destagger
    return destagger(ctx.frame.raw("W"), 0)


def _z(ctx, field):
    return _nan(ctx.frame.vinterp(field, "ght_agl", [h / 1000.0 for h in ctx.hlevels]))


def _uvz(ctx):
    if "uv_z" not in ctx.memo:
        ctx.memo["uv_z"] = (_z(ctx, _uvp(ctx, 0)), _z(ctx, _uvp(ctx, 1)))
    return ctx.memo["uv_z"]


def height_products(opt):
    lv = (103, 0, None)
    return [
        Product("U", "wind, eastward (earth-relative)", "m s-1", lambda c: _uvz(c)[0], "height", "eastward_wind",
                _g(0, 2, 2, "m s-1", lv)),
        Product("V", "wind, northward (earth-relative)", "m s-1", lambda c: _uvz(c)[1], "height", "northward_wind",
                _g(0, 2, 3, "m s-1", lv)),
        Product("WSPD", "wind speed (of the interpolated components)", "m s-1", lambda c: w.wspd(*_uvz(c)),
                "height", "wind_speed", _g(0, 2, 1, "m s-1", lv)),
        Product("WDIR", "wind direction (from, degrees from north)", "degree", lambda c: w.wdir(*_uvz(c)), "height",
                "wind_from_direction", _g(0, 2, 0, "degree", lv), precision=0),
        Product("T", "temperature", "K", lambda c: _z(c, c.frame.tk()), "height", "air_temperature",
                _g(0, 0, 0, "K", lv)),
        Product("RH", "relative humidity", "%", lambda c: _z(c, c.frame.rh()), "height", "relative_humidity",
                _g(0, 1, 1, "%", lv), precision=1),
        Product("P", "pressure", "Pa", lambda c: _z(c, c.frame.pressure()), "height", "air_pressure",
                _g(0, 3, 0, "Pa", lv), precision=0),
    ]


def catalogue(opt):
    """{"surface": [...], "pressure": [...], "height": [...]} of every field cppwrf-post knows."""
    return {"surface": surface_products(opt), "pressure": pressure_products(opt), "height": height_products(opt)}
