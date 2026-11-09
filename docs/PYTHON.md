# The cppwrf Python package

`python/` is a pure-Python package for scripting CPPWRF: WRF namelists, runs of the model and their wrfout files.

```bash
pip install ./python               # needs numpy and netCDF4
pip install "./python[xarray]"     # also xarray: wrfout files as xarray Datasets
```

The package does not contain the model. It finds it, and CPPWRF's namelist translator `tools/wrf2cppwrf.py`, in
this order: `CPPWRF_HOME` (a source tree or `<prefix>/share/cppwrf`), the directory the package sits in (the source
tree's `python/`, or the `share/cppwrf/python` of `cmake --install`), then the prefix of `cppwrf` on `PATH`. The model
binary is `CPPWRF_EXE`, else the one of the requested mode among `bin/`, the source tree's `build*/` directories and
`PATH` (each binary tells its mode with `cppwrf --version`). Its version is the CMake project version.

## Namelists: `cppwrf.namelist`

```python
from cppwrf import namelist as nl

nml = nl.read("namelist.input")                 # {group: {option: [values]}}, lower-case names
nl.get(nml, "domains", "e_we", dom=1)           # domain 2's value (a missing entry: domain 1's)
nml["physics"]["mp_physics"] = [8, 8]
nl.write(nml, "namelist.input")                 # Fortran namelist text (nl.dumps(nml) gives the text)

tr = nl.translate("namelist.input")             # tools/wrf2cppwrf.py as a library: Translation
tr.ok, tr.errors, tr.notices, tr.sections["phys"], tr.ini

res = nl.check("namelist.input")                # what cppwrf-check prints: CheckResult
res.ok                                          # CPPWRF runs this namelist
for o in res.by_status(nl.NOT_PORTED):          # also nl.PORTED, nl.IGNORED
    print(o.group, o.option, o.values, o.reason, o.suggestion)
print(nl.format_check(res, verbose=True))
```

`parse(text)`, `start_date(nml, dom)` (WRF's `YYYY-MM-DD_hh:mm:ss`) and `stream_filename(pattern, dom, date)` (WRF's
`<domain>` / `<date>` file name patterns) are there too. The parsing and the translation are the translator's own
code, so what `check` reports is what the model does.

## Runs: `cppwrf.Run`

A run of a WRF run directory, as `cppwrf-wrf` does it (WRF's file names in the directory, `rsl.out.0000` /
`rsl.error.0000`, CPPWRF's own files in `cppwrf_work/`):

```python
import time, cppwrf

run = cppwrf.Run("/data/case", mode="gpu-fp32")    # or a path to its namelist.input; threads=, exe=, sets=[...]
run.prepare()                                       # translate; NamelistError if an option has no port
run.start()
while run.running:
    p = run.progress()                              # from the log's "Timing for main" lines of domain 1
    print(f"{p.model_time} {100 * p.fraction:.0f}% eta {p.eta_seconds} s, files {p.files}")
    time.sleep(10)
run.wait()                                          # RunFailed if the model stopped with an error
ds = run.open(domain=1)                             # its wrfout files (open_wrfout)
```

`Run(...).run(timeout=None)` prepares, starts and waits. `cancel()` stops the model (`wait()` then raises
`RunFailed`). The exceptions: `CppwrfError` (the base: a missing binary, file or directory), `NamelistError`
(`.check`: the `CheckResult`), `RunFailed` (`.status`: the exit status; `.log_tail`: the end of the model's standard
error). `run.command`, `run.mode`, `run.start_date`, `run.end_date`, `run.returncode` and `run.wrfout(domain)` (the
history files by `history_outname`) describe the run.

## wrfout files: `cppwrf.open_wrfout`

```python
ds = cppwrf.open_wrfout("/data/case")                    # every wrfout_d01_* of the directory, joined along Time
ds = cppwrf.open_wrfout("/data/case/wrfout_d02_*", domain=2, variables=["T2", "U10", "V10"])
```

With xarray installed the result is an `xarray.Dataset`: `Time` decoded from `Times` (datetime64; datetimes for the
ideal cases' year 0001), `XLAT` / `XLONG` (and the `_U` / `_V` ones) as 2-D coordinates of the first frame, the global
attributes in `ds.attrs`. Without xarray, or with `xarray=False`, it is a `WrfoutData`: a dict of numpy arrays (Time
first) with `.times`, `.attrs`, `.dims`, `.units`, `.lat` and `.lon`.

## Diagnostics: `cppwrf.diagnostics`

| Function | Result |
|---|---|
| `t2_celsius(ds)` | 2 m temperature (°C) |
| `wind10(ds, earth=True)` | 10 m wind speed (m s-1) and direction it blows from (degrees), rotated to the earth with `COSALPHA` / `SINALPHA` |
| `slp(ds)` | sea-level pressure (hPa) by WRF's / RIP's method, statement for statement as wrf-python's `getvar(nc, "slp")` (`seaprs(z, t, p, q)` for your own columns) |
| `precipitation(ds)` | accumulated precipitation (mm): `RAINC + RAINNC (+ RAINSH)` plus the buckets (`I_RAINC`, `I_RAINNC` times `BUCKET_MM`) |
| `precip_accum(ds, hours=1)` | precipitation in the `hours` before each frame (mm; NaN without the earlier frame): hourly precipitation |

With a Dataset they return DataArrays on its coordinates, with `WrfoutData` numpy arrays.

## Example: a run and a map of its last frame

```python
import cppwrf, matplotlib.pyplot as plt
from cppwrf import diagnostics as dg

cppwrf.Run("run", mode="gpu-fp32", threads=8).run()
ds = cppwrf.open_wrfout("run")
last = ds.isel(Time=-1)
plt.contourf(ds.XLONG, ds.XLAT, dg.slp(ds).isel(Time=-1), levels=20)
plt.colorbar(label="hPa"); plt.title(str(last.Time.values)[:16]); plt.savefig("slp.png")
```

## Tests

```bash
python3 -m unittest discover -s python/tests      # no GPU, no model, no data (a stand-in binary, generated files)
```

ctest runs them as `python_package`.
