# cppwrf-post: forecaster fields as NetCDF and GRIB2

`cppwrf-post` turns wrfout files into the fields a forecaster looks at: 2 m temperature and humidity, 10 m wind, sea
level pressure, precipitation, clouds, instability, fields on pressure levels and at heights above ground. It writes
one file per output time: CF-1.8 NetCDF, GRIB2, or both. Turkish: [tr/SON_ISLEM.md](tr/SON_ISLEM.md).

```bash
cppwrf-post run/wrfout_d01_*                                # NetCDF in the current directory
cppwrf-post run/wrfout_d01_* -o post --format both -j 4     # NetCDF and GRIB2, four processes
cppwrf-post run/wrfout_d01_* -c post.yaml                   # your fields, levels, units and packing
cppwrf-post --list-fields                                   # every field, with units
cppwrf-post --print-config > post.ini                       # the defaults, as a file to edit
```

It is part of the `cppwrf` Python package (docs/PYTHON.md) and needs `numpy` and `netCDF4`; a `.yaml` configuration
needs PyYAML, and GRIB2 packing other than simple packing needs ecCodes' Python bindings (`pip install eccodes`).
Simple-packed GRIB2 is written by CPPWRF's own encoder and needs nothing else.

## Methods: wrf-python's

Every diagnostic is computed the way wrf-python 1.3.4.1 computes it: its Fortran routines are ported to numpy
(`python/cppwrf/post/wrfpy.py`, each function naming the routine it ports) and called with the same arguments as
wrf-python's `getvar` (`python/cppwrf/post/frame.py`). Nothing is a new formula, except the precipitation type,
which wrf-python does not have (below). wrf-python is Apache-2.0 (licenses/WRF-PYTHON-LICENSE.txt); its
`psadilookup.dat` table is copied unchanged.

The port was compared with wrf-python itself (`tools/post/compare_wrfpython.py`, wrf-python 1.3.4.1 built from its
source, numpy 2.5) on four wrfout files of the WEB2 Turkey case (100 x 100 x 45 points, 9 km): three single
precision files of a GPU FP32 run and one double precision file of a GPU FP64 run.

| Fields | FP32 files | FP64 file: max abs / max rel difference |
|---|---|---|
| slp, tk, rh, rh2, td, td2, pw, omg, avo | identical | 6e-13 / 4e-14 |
| dbz, mdbz (also use_varint, use_liqskin) | identical | 2e-14 / 3e-13 |
| cloudfrac (height_agl and pres) | identical | 2e-14 / 1e-11 |
| ctt | identical | 1e-13 / 1e-12 |
| cape_2d (mcape, mcin, lcl, lfc), cape_3d lowest level | identical | 3e-10 / 4e-12 |
| helicity 0-1 / 0-3 km, updraft_helicity 2-5 km | identical | identical |
| uvmet, uvmet10 | identical | identical |
| vinterp (pressure, log p, extrapolation; height above ground), interplevel | identical | 1e-12 / 7e-15 |
| freezing level (interplevel of z on tc at 0 degC) | identical | 5e-11 / 4e-14 |

"Identical" means every value equal to the bit. On single precision files `cppwrf-post` reproduces wrf-python's
habit of returning each result in the type of the file's variables (`options.emulate_wrfpython`, on by default);
the double precision file shows only the rounding differences of numpy's operation order. The one field that
differs by more is the 10 m wind rotated with `COSALPHA` / `SINALPHA` (the default, `options.wind_rotation:
cosalpha`): wrf-python's `uvmet10` uses the cone formula of the projection instead, and the two differ by at most
7.7e-7 m s-1 (relative 5e-6) on these files. `wind_rotation: uvmet` gives wrf-python's values exactly.

## Fields

`fields.surface` (default `all`), `fields.pressure` and `fields.height` choose the fields; the names are those of
`cppwrf-post --list-fields`. A field whose input is not in the file (no `WSPD10MAX`, no `REFL_10CM` and no
hydrometeors, ...) is skipped with a message.

### Surface and column

| Name | Units | Method | GRIB2 (discipline-category-number, level) |
|---|---|---|---|
| T2, T2C | K, degC | T2 | 0-0-0, 2 m |
| TD2, RH2 | degC, % | wrf-python td2, rh2 (Q2, PSFC, T2) | 0-0-6, 0-1-1, 2 m |
| U10, V10, WSPD10, WDIR10 | m s-1, degrees | U10 / V10 rotated to earth-relative (COSALPHA / SINALPHA); direction the wind blows from | 0-2-2/3/1/0, 10 m |
| GUST10 | m s-1 | WSPD10MAX: the largest 10 m wind of the output interval (Template 4.8, maximum) | 0-2-22, 10 m |
| MSLP, PSFC | hPa | wrf-python slp (RIP / WRF seaprs); PSFC | 0-3-1 mean sea level; 0-3-0 surface |
| HGT | m | terrain height | 0-3-5 surface |
| PRECIP_ACC, PRECIP_CONV_ACC, PRECIP_GRID_ACC | mm | since the start: RAINC + RAINSH, RAINNC, both; buckets (I_RAINC, I_RAINNC times BUCKET_MM) added | 0-1-8/10/9, Template 4.8 accumulation |
| PRECIP_1H, PRECIP_CONV_1H, PRECIP_GRID_1H | mm | over the last `options.precip_hours` (1 h by default; the names follow) | as above, over the interval |
| SNOW_ACC, SNOW_1H, GRAUPEL_ACC, GRAUPEL_1H | mm water | SNOWNC, GRAUPELNC | 0-1-15, 0-1-75 |
| PTYPE | code | precipitation type of the interval (below) | 0-1-19 (Code Table 4.201) |
| SNOWH, SWE | m, mm | SNOWH, SNOW | 0-1-11, 0-1-60 |
| CLOUD_LOW, CLOUD_MID, CLOUD_HIGH | % | wrf-python cloudfrac, `vert_type` height above ground (300 / 2000 / 6000 m, its default) | 0-6-3/4/5, low / middle / high cloud layer |
| CLOUD_TOTAL | % | the largest of the three (maximum overlap) | 0-6-1, entire atmosphere |
| PW | mm | wrf-python pw | 0-1-3, entire atmosphere |
| CAPE_SB, CIN_SB | J kg-1 | wrf-python cape_3d, the value of the lowest model level's parcel | 0-7-6/7, surface |
| CAPE_ML, CIN_ML | J kg-1 | wrf-python's capecalc2d with the lowest level as the parcel level, mixed over the lowest `options.ml_depth_hpa` (100 hPa) | 0-7-6/7, 100-0 hPa above ground |
| CAPE_MU, CIN_MU | J kg-1 | wrf-python cape_2d mcape / mcin (largest theta-e in the lowest 3 km) | 0-7-6/7, 0-3000 m above ground |
| LCL, LFC | m above ground | wrf-python cape_2d lcl / lfc | 0-3-5 at the LCL / LFC level types, as height above sea level |
| SRH01, SRH03 | m2 s-2 | wrf-python helicity (top 1000 / 3000 m) | 0-7-8, 0-1000 / 0-3000 m |
| UH25 | m2 s-2 | wrf-python updraft_helicity (2000-5000 m) | 0-7-15, 2000-5000 m |
| FZLEV | m above sea level | wrf-python interplevel of height on temperature at 0 degC | 0-3-5, 0 degC isotherm |
| MAXDBZ | dBZ | column maximum of REFL_10CM when the file has it, else wrf-python mdbz (`options.max_dbz`) | 0-16-5, entire atmosphere |
| CTT | degC | wrf-python ctt | 0-0-0, cloud top |

CIN is positive in NetCDF (as wrf-python gives it) and negative in GRIB2 (WMO's sign). CIN is missing where CAPE is
below 100 J kg-1, as in wrf-python.

**Precipitation type** (CPPWRF's rule; wrf-python has none): over the interval, where the total is at least 0.1 mm,
the frozen fraction f is (snow + graupel + hail) / grid-scale precipitation. f >= 0.9: snow (5), or graupel (9)
when graupel and hail exceed snow; 0.1 < f < 0.9: rain and snow (7); otherwise freezing rain (3) where T2 < 0 degC,
else rain (1). Missing where less than 0.1 mm fell. It uses only the microphysics' own surface accumulations, so it
is as good as the scheme's partition; convective precipitation counts as liquid.

**Visibility** is not produced: wrf-python has no visibility routine, and the formulas in use (UPP's, RUC's) depend
on the microphysics scheme and are not wrf-python's methods.

### Pressure levels and heights above ground

| Name | Units | Method |
|---|---|---|
| T, TD, RH, Q, GHT | K, degC, %, kg kg-1, m | wrf-python vinterp to the levels of `levels.pressure` (hPa), in log pressure (`options.log_p`) |
| U, V | m s-1 | earth-relative (rotated before interpolation) |
| W, OMEGA, AVO | m s-1, Pa s-1, 1e-5 s-1 | W on mass levels, wrf-python omg and avo |
| DBZ | dBZ | REFL_10CM or wrf-python dbz |
| U, V, WSPD, WDIR, T, RH, P (heights) | | wrf-python vinterp to `levels.height` (m above ground) |

Below the ground (pressure above the surface pressure) values are missing, unless `options.extrapolate: true`, which
uses vinterp's extrapolation (RIP's standard atmosphere below ground). The defaults are the pressure levels 1000,
925, 850, 700, 500, 300, 250 and 200 hPa and the heights 80, 100 and 120 m (wind-energy hub heights).

## Output files

One file per wrfout time, named by `output.filename` (default `wrfpost_{domain}_{time}`, also `{lead}` in hours):
`wrfpost_d01_2026-11-09_12:00:00.nc` and `.grib2`.

**NetCDF** (CF-1.8): dimensions `time` (1), `y`, `x`, `pressure` (hPa), `height` (m); 2-D fields keep their names,
pressure-level fields end in `_P` and height fields in `_Z` (`T_P`, `U_Z`). Every variable has `standard_name`
where CF defines one, `units`, `grid_mapping` and `coordinates = "lat lon"`; `lat` and `lon` are the 2-D latitudes
and longitudes; `x` and `y` are the projection coordinates in metres, fitted to the grid from XLAT / XLONG. The grid
mapping is `lambert_conformal_conic`, `polar_stereographic`, `mercator` or `latitude_longitude` with WRF's sphere
(radius 6370 km). Time is `hours since` the forecast's start with `forecast_reference_time` and
`forecast_period`; accumulations have `cell_methods = "time: sum"`. Values are float32 (`netcdf.dtype`) and
compressed with zlib level 4 (`netcdf.compression`). The IOOS compliance checker (`compliance-checker --test
cf:1.8`) passes these files without errors or warnings.

**GRIB2**: one message per field and level. Section 1 names centre 255 (missing; set `grib2.centre` to yours),
tables version 30, production status 2 (research). The grid is Template 3.30 (Lambert), 3.20 (polar
stereographic), 3.10 (Mercator) or 3.0 (latitude-longitude), earth shape 1 with radius 6370000 m. Products use
Template 4.0, or 4.8 for accumulations (since the start, or over the last hours) and the gust maximum. Data are
simple-packed (Template 5.0) with a bitmap where values are missing; each field keeps the precision
`10**-D` of its decimal scale factor D (`--list-fields` units; defaults: 0.01 K for temperatures, 0.01 m s-1 for
winds, 1 Pa for pressures, 0.01 mm for precipitation). `grib2.packing: jpeg` or `ccsds` re-packs the messages with
ecCodes (JPEG 2000, CCSDS), when it is installed.

Checks (`tools/post/check_grib2.py`, `tools/post/check_projections.py`, with ecCodes 2.44 and cfgrib 0.9.15): all
3540 messages of a 25-time WEB2 run decode with ecCodes; each one's parameter, level, time range and grid are those
of its field, its values equal the NetCDF file's within the packing precision, and ecCodes' latitudes / longitudes
match XLAT / XLONG to 2e-5 degrees. For synthetic grids of each projection ecCodes' coordinates match within 1e-6
degrees. xarray with cfgrib reads every level type (open with `filter_by_keys={"typeOfLevel": ..., "stepType": ...}`,
as for any multi-level GRIB file). Two limits of other software: with centre 255, ecCodes names a few WMO
parameters `unknown` (for example SRH, UH, the composite reflectivity), although they are WMO's numbers; and cfgrib
cannot put SRH and UH (both "heightAboveGroundLayer", different layers) into one dataset, so filter them by
`shortName` as well.

## Configuration

A `post.yaml` or `post.ini` changes the defaults (`cppwrf-post --print-config` prints them all); a key that does
not exist is an error.

```yaml
output:
  format: both            # netcdf | grib2 | both
  dir: post
  filename: "wrfpost_{domain}_{time}"
fields:
  surface: [T2C, TD2, RH2, U10, V10, WSPD10, GUST10, MSLP, PRECIP_ACC, PRECIP_3H, PTYPE, CLOUD_TOTAL, CAPE_MU]
  pressure: [T, RH, GHT, U, V]
  height: [U, V, WSPD]
levels:
  pressure: [1000, 850, 700, 500, 250]
  height: [80, 100, 120, 150]
units:                     # NetCDF units (GRIB2 always uses WMO's)
  MSLP: hPa
  T2: degC
options:
  wind_rotation: cosalpha  # or uvmet: wrf-python's cone formula
  precip_hours: 3          # PRECIP_3H etc.
  extrapolate: false
netcdf:
  compression: 4
grib2:
  centre: 255
  packing: simple          # simple | jpeg | ccsds
  precision:               # decimal scale factor D: values kept to 10**-D
    T2: 1
    PRECIP_3H: 1
```

The same as `.ini`: sections `[output]`, `[fields]`, `[levels]`, `[units]`, `[options]`, `[netcdf]`, `[grib2]` and
`[grib2.precision]`; lists are comma-separated (`pressure = 1000, 850, 500`).

`options`: `wind_rotation` (cosalpha | uvmet), `log_p` (vinterp in log pressure, default true), `extrapolate`,
`dbz_varint` / `dbz_liqskin` (wrf-python's dbz options), `max_dbz` (auto | model | computed), `cloud_vert_type`
(height_agl | height_msl | pres), `precip_hours`, `ml_depth_hpa`, `emulate_wrfpython`. Units that `units` accepts:
K / degC / degF; Pa / hPa; kg m-2 / mm; m s-1 / kt / km h-1; kg kg-1 / g kg-1; m / km / cm.

## Speed

The files are processed one time at a time, so memory stays that of one frame. On an Intel i7-10750H (6 cores) a
24 h WEB2 run (100 x 100 x 45 points, 25 hourly times, every default field in NetCDF and GRIB2) takes 31-35 s with
one process (230 MB of memory) and 16 s with `-j 4`; the files are the same either way.

## Python

```python
from cppwrf import post
written = post.process("run/wrfout_d01_*", config="post.yaml", output_dir="post", fmt="both", jobs=4)
post.fields()                                  # {"surface": [...], "pressure": [...], "height": [...]}
```

`process(files, config=None, output_dir=None, fmt=None, overrides=None, log=None, jobs=1)` returns the paths
written; `config` is a path or a dict from `cppwrf.post.config.load(path, overrides)`. For single diagnostics,
`cppwrf.post.frame.Frame(netCDF4.Dataset(path), time_index)` has methods named after wrf-python's (`slp()`,
`td2()`, `cape_2d()`, `srh(3000)`, `vinterp(...)`, ...), and `cppwrf.post.wrfpy` the ported routines.

## Tests

`python3 -m unittest discover -s python/tests -p "test_post_*.py"` (ctest `post_diagnostics`, `post_output`, label
`unit`): the diagnostics on synthetic fields with known answers (standard atmosphere columns, saturated air, uniform
shear, a well-mixed layer for the mixed-layer parcel, ...), the GRIB2 encoder against a decoder of its own, the
projections, the configuration, and whole runs on synthetic wrfout files in both formats. The comparisons with
wrf-python, ecCodes and the CF checker are the scripts in `tools/post/` (they need those packages).
