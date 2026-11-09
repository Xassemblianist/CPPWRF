# cppwrf (Python)

CPPWRF from Python: WRF namelists (read, write, translate, check what CPPWRF ports), runs of the model in WRF run
directories (start, follow, cancel, wait) and wrfout files as xarray Datasets or numpy arrays with standard diagnostics.
It also installs the command-line tools `cppwrf-wrf` (drop-in wrf.exe), `cppwrf-realexe` (drop-in real.exe) and
`cppwrf-check`.

```bash
pip install ./python            # numpy and netCDF4; pip install "./python[xarray]" for xarray Datasets
```

The model itself comes from a CPPWRF build or installation (docs/INSTALL.md): the package finds the binaries and
CPPWRF's tools through `CPPWRF_HOME` (the source tree or `<prefix>/share/cppwrf`), `CPPWRF_EXE`, or `cppwrf` on PATH.
The API is described in docs/PYTHON.md. GPL-3.0-or-later.
