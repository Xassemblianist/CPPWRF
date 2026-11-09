# cppwrf-verify-obs: a run against surface observations

`cppwrf-verify-obs` compares a run's wrfout files with the surface observations of its domain and period: 2 m
temperature, dew point and humidity, 10 m wind, pressure and precipitation, scored per variable, lead time and
station. It downloads the observations itself (NOAA NCEI's ISD, or the Iowa Environmental Mesonet's METAR archive)
or reads your own CSV file. Turkish: [tr/GOZLEM_DOGRULAMA.md](tr/GOZLEM_DOGRULAMA.md).

```bash
cppwrf-verify-obs run/wrfout_d01_*                                   # METARs of every airport in the domain
cppwrf-verify-obs run/wrfout_d01_* -o verify --stations LTAC,LTBA,LTFM
cppwrf-verify-obs run/wrfout_d01_* --obs-source file --obs-file mgm.csv
cppwrf-verify-obs gpu/wrfout_d01_* --compare cpu/wrfout_d01_* --labels gpu-fp32,cpu-fp64
```

It is part of the `cppwrf` Python package (docs/PYTHON.md) and needs `numpy` and `netCDF4` only; the plots are
plain SVG.

## Observations

| `--obs-source` | What | Variables |
|---|---|---|
| `iem` (default) | METAR / SPECI reports from the Iowa Environmental Mesonet's ASOS download service (mesonet.agron.iastate.edu), worldwide airports | T, Td, wind, gust, QNH; in the US also sea level pressure and precipitation (remarks) |
| `isd` | NOAA NCEI Integrated Surface Database, Global Hourly CSV, one file per station and year | T, Td, wind, sea level pressure, altimeter setting, precipitation (AA1-AA4: SYNOP 1-24 h amounts) |
| `file` | your CSV file (`--obs-file`), e.g. a national service's station data | any of the variables below |

**ISD ends in 2025.** NCEI stopped updating the Global Hourly files in August 2025 (its successor, GHCNh, has
another format, which this tool does not read). `--obs-source isd` works for runs up to then; for later runs use
`iem`, or a file. Outside the United States METARs carry no precipitation amounts, so precipitation needs ISD's
SYNOP amounts or a file.

**Stations.** `--stations auto` (the default) takes NCEI's station list (isd-history.csv): for ISD the stations whose
record covers the run, for IEM those with an ICAO identifier that reported in the last five years before the run,
inside the domain. `--stations LTAC,LTBA` (ICAO for `iem`, USAF+WBAN such as `17130099999` for `isd`) names them.

**Downloads** are polite: one request at a time with a pause between them (`--pause`, 1 s), retries with
exponential back-off on HTTP 429 and 5xx (honouring Retry-After), a User-Agent naming the tool, only the stations
of the domain and only the run's period (IEM: 20 stations per request; ISD: a year file is read line by line and
only the run's rows are kept). Everything is cached in `--cache-dir` (default `$CPPWRF_OBS_CACHE`, else
`~/.cache/cppwrf/obs`): verifying the same period again downloads nothing, and `--offline` uses the cache only. The
Turkey example below downloaded 0.1 MB in 3 requests.

**Your own file**: CSV (comma, semicolon or tab separated; a decimal comma with semicolons), one row per station and
time (UTC):

```text
station,time,lat,lon,elev,t2,td2,rh2,wspd,wdir,gust,mslp,qnh,precip,precip_hours
17130,2026-11-09 06:00,39.95,32.88,891,-1.5,-4.2,,2.0,90,,1031.2,,0.4,6
```

Only `station`, `time`, `lat`, `lon` are required (the first row of a station gives its position and `elev`);
units are degC, %, m s-1, degrees, hPa and mm. Precipitation is `precip` with `precip_hours`, or columns
`precip_1h`, `precip_6h`, `precip_24h`, ... Missing values are empty, `NA`, `M` or `-9999`. Some Turkish column
names are understood too (`istasyon`, `tarih`, `enlem`, `boylam`, `yukseklik`, `sicaklik`, `nem`, `ruzgar_hizi`,
`ruzgar_yonu`, `yagis`).

## Quality control

- ISD: values whose quality code is 2, 3, 6 or 7 (suspect, erroneous) are dropped (Federal Climate Complex, ISD
  format document); precipitation with condition codes 1, 5-8, I, J (inaccurate, deleted, missing or incomplete
  periods) too; a trace counts as 0 mm.
- METAR: a report without a station, a `ddhhmmZ` time, or any wind, temperature or pressure group is rejected and
  counted (`metar_error`, with examples in scores.json); NIL reports likewise. Variable winds without a direction
  count for speed only; calm is 0 m s-1. US remarks: SLPppp, the precise temperature group TsnnnSnnn, hourly
  (Prrrr), 3/6-hour (6RRRR) and 24-hour (7RRRR) precipitation, which end at the next full hour.
- Every source: gross limits (T -90..60 degC, Td -100..40 degC, wind 0..75 m s-1, pressures 850..1090 hPa,
  precipitation 0..500 mm), and a dew point more than 0.5 K above the temperature is dropped.

Every rejection is counted by reason in `summary.md` and `scores.json`.

## Model values at the stations

- **Position**: the fractional grid indices of a station come from inverting the bilinear interpolation of XLAT /
  XLONG (Newton iterations), which works for every projection. Stations within `spec_bdy_width` points of the edge
  (the relaxation zone, from a `namelist.input` next to the files or in `../run`, else 5, or `--boundary`) are left
  out; `--max-elev-diff` also leaves out stations whose height differs too much from the model's terrain.
- **Interpolation**: bilinear from the four surrounding points (`--interpolation nearest`: the nearest point).
- **Fields**: T2; wrf-python's td2 and rh2 (as in cppwrf-post); 10 m wind rotated to earth-relative with
  COSALPHA / SINALPHA (speed and direction of the interpolated components); wrf-python's slp for sea level
  pressure; QNH from PSFC and the terrain height with the altimeter-setting formula QNH = (p^0.190263 + 8.417286e-5
  z)^(1/0.190263) (hPa, m; ICAO standard atmosphere, as METAR's QNH is defined); precipitation as RAINC + RAINSH +
  RAINNC with the buckets. The observed relative humidity is computed from T and Td with the same saturation vapour
  pressure as wrf-python's.
- **T2 height correction**: the model's T2 is at the model's terrain height; it is moved to the station's elevation
  with the standard atmosphere's lapse rate, T + 0.0065 K/m x (z_model - z_station) (`--lapse-rate`;
  `--no-height-correction` turns it off). Dew point and humidity are not corrected.
- **Time**: for each output time, the observation nearest in time within `--window` minutes (30) that has the
  variable (a METAR at 11:50 verifies 12:00). Precipitation: an amount over N hours ending at an output time is
  compared with the model's accumulation over the same N hours, when both ends are output times of the run.

## Scores

Per variable, over all stations and times, per lead time, and per station (Wilks 2011, "Statistical methods in the
atmospheric sciences", chapter 8):

- **continuous** (T2, Td2, RH2, wind speed, MSLP, QNH): number of pairs, bias (mean forecast - observation), MAE,
  RMSE, Pearson correlation, mean observed and forecast values;
- **wind vector**: RMSE of the vector difference, sqrt(mean((uf - uo)^2 + (vf - vo)^2)), calm included;
  **direction**: the difference wrapped to -180..180 degrees, mean (bias) and mean absolute, for observed speeds of
  at least `--min-wind-dir` (1 m s-1);
- **precipitation**, per accumulation period (`--precip-hours`, 1, 3, 6, 12, 24 h) and threshold (`--thresholds`,
  0.1, 1, 5, 10, 25 mm): the contingency table (hits a, false alarms b, misses c, correct negatives d; an event is
  amount >= threshold) and POD = a/(a+c), FAR = b/(a+b), CSI = a/(a+b+c), frequency bias = (a+b)/(a+c), ETS =
  (a-ar)/(a+b+c-ar) with ar = (a+b)(a+c)/n; also bias, MAE and RMSE of the amounts.

`--compare FILES` scores a second run on exactly the same pairs (those both runs have), for example a GPU single
precision run against the CPU double precision run of the same case.

## Output

In `-o DIR` (default `verify`):

| File | Contents |
|---|---|
| `summary.md` | the setup, the scores of each run, precipitation contingency, quality control, the plots |
| `scores_overall.csv`, `scores_by_lead.csv`, `scores_by_station.csv` | run, group, variable, units, n, bias, MAE, RMSE, correlation, means |
| `contingency.csv` | run, period, threshold, n, hits, false alarms, misses, POD, FAR, CSI, frequency bias, ETS |
| `pairs.csv` | every forecast-observation pair: station, variable, valid time, lead, values, time offset |
| `stations.csv` | each station: position, elevation, model elevation, grid indices, used or why not |
| `scores.json` | all of the above as JSON, with the quality-control counts and examples |
| `plots/lead_<var>.svg` | RMSE (solid) and bias (dashed) against lead time, one line per run |
| `plots/map_bias_<var>.svg` | the bias per station on a map of the domain |

## Example: the WEB2 Turkey case, GPU FP32 against CPU FP64

The WEB2 sample run: GFS 2026-11-08 18 UTC, 24 h, 100 x 100 points of 9 km centred on 39.5 N 33 E (Lambert), 45
levels, WSM6 / Kain-Fritsch / YSU / Noah; the GPU FP32 and CPU FP64 runs of the same case, every third hour (9
times) verified against the METARs of IEM:

```bash
cppwrf-verify-obs gpu_fp32/wrfout_d01_* --compare cpu_fp64/wrfout_d01_* --labels gpu-fp32,cpu-fp64
```

38 airports lie inside the domain (outside the 5-point boundary zone); 26 reported in the period (890 reports, none
rejected), giving 3156 pairs. The run took 7 s.

| Variable | n | bias | MAE | RMSE | corr | (CPU FP64: bias / RMSE) |
|---|---|---|---|---|---|---|
| 2 m temperature (degC) | 211 | +1.19 | 2.03 | 2.54 | 0.94 | +1.19 / 2.54 |
| 2 m dew point (degC) | 211 | -2.26 | 2.87 | 3.58 | 0.79 | -2.27 / 3.58 |
| 2 m relative humidity (%) | 211 | -11.9 | 14.3 | 18.0 | 0.80 | -11.9 / 18.0 |
| 10 m wind speed (m s-1) | 211 | +0.49 | 1.22 | 1.61 | 0.55 | +0.48 / 1.61 |
| 10 m wind vector (m s-1) | 180 | | | 2.58 | | 2.57 |
| 10 m wind direction (degrees) | 157 | +0.3 | 48.3 | | | +0.3 / MAE 48.4 |
| QNH (hPa) | 211 | -0.29 | 0.72 | 0.89 | 0.91 | -0.29 / 0.89 |

T2 RMSE by lead time (GPU FP32): 2.1 K at +0 h, 3.5-3.6 K at +9 and +12 h (a warm bias of 2.8 K in the afternoon),
1.3-1.4 K in the night (+15 to +21 h), 3.3 K at +24 h. The two runs' scores agree to the second decimal: at a single
station and time their forecasts differ by at most 0.46 K in T2, 0.14 m s-1 in wind speed and 0.01 hPa in QNH, far
below the forecast errors. METARs outside the US carry no precipitation, so this example has no precipitation
scores (the unit tests check them).

## Python

```python
from cppwrf import verify_obs
res = verify_obs.verify("run/wrfout_d01_*", obs_source="iem", output_dir="verify")
res["overall"]["run"]["t2"]          # {"n", "bias", "mae", "rmse", "corr", "mean_obs", "mean_fcst"}
res["by_lead"]["run"][12.0]["wspd"]
```

`verify(files, obs_source="iem", stations="auto", output_dir="verify", compare=None, labels=None, obs_file=None,
cache_dir=None, boundary=None, interpolation="bilinear", height_correction=True, lapse_rate=0.0065,
window_minutes=30, thresholds=(0.1, 1, 5, 10, 25), precip_hours=(1, 3, 6, 12, 24), min_wind_dir=1.0,
max_elev_diff=None, offline=False, pause=1.0, make_plots=True, log=None)` returns `overall`, `by_lead` and
`by_station` scores per run label, the stations, the pairs and the quality-control counts. The parsers
(`cppwrf.verify_obs.obs.parse_isd`, `parse_iem`, `parse_metar`, `parse_file`) and scores
(`cppwrf.verify_obs.scores`) can be used on their own.

## Tests

`python3 -m unittest discover -s python/tests -p test_verify_obs.py` (ctest `verify_obs`, label `unit`, no network):
the ISD and METAR parsers on saved real reports (`python/tests/data`, sources in its README), the CSV reader, the
scores on known answers, station positions, and a whole verification of synthetic wrfout files against synthetic
observations with known errors. `CPPWRF_NETWORK_TESTS=1` adds a small ISD and IEM download.
