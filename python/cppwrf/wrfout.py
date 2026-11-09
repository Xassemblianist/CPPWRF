# SPDX-License-Identifier: GPL-3.0-or-later
"""wrfout files (WRF's or CPPWRF's: the same format) as an xarray Dataset or as numpy arrays.

    ds = cppwrf.open_wrfout("run/wrfout_d01_*")            # xarray, when installed: Time decoded, XLAT / XLONG coords
    data = cppwrf.open_wrfout("run", domain=2, xarray=False)  # WrfoutData: {name: ndarray}, .times, .attrs

The files of one domain are joined along Time in the order of their times."""
import datetime
import glob
import os
from typing import Dict, Iterable, List, Optional, Union

import numpy as np

Paths = Union[str, Iterable[str]]

_COORDS = (("XLAT", "XLONG"), ("XLAT_U", "XLONG_U"), ("XLAT_V", "XLONG_V"))


def wrfout_files(paths: Paths, domain: int = 1) -> List[str]:
    """The files: a directory (its wrfout_d0N_* files), a glob pattern, a file or a list of them."""
    if isinstance(paths, str):
        if os.path.isdir(paths):
            files = glob.glob(os.path.join(paths, f"wrfout_d{domain:02d}_*"))
        else:
            files = glob.glob(paths) or ([paths] if os.path.exists(paths) else [])
    else:
        files = [p for x in paths for p in wrfout_files(x, domain)]
    if not files:
        raise FileNotFoundError(f"no wrfout files in {paths!r}")
    return sorted(dict.fromkeys(files))


def decode_times(chars: np.ndarray) -> List[datetime.datetime]:
    """WRF's Times variable (Time x DateStrLen characters) as datetimes."""
    out = []
    for row in np.atleast_2d(chars):
        s = b"".join(x if isinstance(x, bytes) else bytes(x) for x in row).decode() if row.dtype.kind == "S" \
            else "".join(row)
        out.append(datetime.datetime.strptime(s.strip("\x00 ")[:19], "%Y-%m-%d_%H:%M:%S"))
    return out


class WrfoutData(dict):
    """{variable: ndarray (Time first)} of wrfout files, with .times (datetimes), .attrs (the first file's global
    attributes), .dims ({variable: dimension names}), .units ({variable: units}); .lat / .lon (XLAT / XLONG of the
    first frame)."""

    def __init__(self):
        super().__init__()
        self.times: List[datetime.datetime] = []
        self.attrs: Dict[str, object] = {}
        self.dims: Dict[str, tuple] = {}
        self.units: Dict[str, str] = {}

    @property
    def lat(self) -> np.ndarray:
        return self["XLAT"][0]

    @property
    def lon(self) -> np.ndarray:
        return self["XLONG"][0]


def _read_numpy(files: List[str], variables: Optional[List[str]]) -> WrfoutData:
    import netCDF4
    out = WrfoutData()
    parts: Dict[str, list] = {}
    for i, f in enumerate(files):
        with netCDF4.Dataset(f) as ds:
            ds.set_auto_mask(False)
            if i == 0:
                out.attrs = {k: ds.getncattr(k) for k in ds.ncattrs()}
            out.times += decode_times(ds["Times"][:])
            for name, v in ds.variables.items():
                if name == "Times" or (variables and name not in variables and name not in ("XLAT", "XLONG")):
                    continue
                a = v[:]
                if v.dimensions[:1] == ("Time",):
                    parts.setdefault(name, []).append(a)
                elif i == 0:
                    out[name] = a
                out.dims.setdefault(name, v.dimensions)
                if "units" in v.ncattrs():
                    out.units.setdefault(name, v.getncattr("units"))
    for name, p in parts.items():
        out[name] = np.concatenate(p, axis=0)
    order = sorted(range(len(out.times)), key=lambda k: out.times[k])
    if order != list(range(len(order))):
        out.times = [out.times[k] for k in order]
        for name in parts:
            out[name] = out[name][order]
    return out


def _read_xarray(files: List[str], variables: Optional[List[str]]):
    import xarray as xr
    sets = []
    for f in files:
        ds = xr.open_dataset(f, decode_times=False, mask_and_scale=False)
        if variables:
            keep = [v for v in ds.data_vars if v in variables or v in ("Times", "XLAT", "XLONG")]
            ds = ds[keep]
        sets.append(ds)
    ds = xr.concat(sets, dim="Time", data_vars="minimal", coords="minimal", compat="override") \
        if len(sets) > 1 else sets[0]
    times = decode_times(ds["Times"].values)
    if all(1678 <= t.year <= 2261 for t in times):
        tv = np.array(times, dtype="datetime64[ns]")
    else:  # outside datetime64[ns] (the ideal cases' year 0001): the datetimes themselves
        tv = np.array(times, dtype=object)
    ds = ds.assign_coords(Time=("Time", tv))
    for la, lo in _COORDS:  # the first frame's latitude / longitude as 2-D coordinates
        if la in ds.variables and lo in ds.variables:
            lat = ds[la].isel(Time=0, drop=True) if "Time" in ds[la].dims else ds[la]
            lon = ds[lo].isel(Time=0, drop=True) if "Time" in ds[lo].dims else ds[lo]
            ds = ds.drop_vars([la, lo]).assign_coords({la: lat, lo: lon})
    ds = ds.drop_vars("Times").sortby("Time")
    ds.attrs = dict(sets[0].attrs)
    return ds


def open_wrfout(paths: Paths, domain: int = 1, variables: Optional[List[str]] = None,
                xarray: Optional[bool] = None):
    """wrfout files of one domain: an xarray Dataset (xarray=None: when xarray is installed) or WrfoutData (numpy).
    variables: read only these (and XLAT / XLONG)."""
    files = wrfout_files(paths, domain)
    if xarray is None:
        try:
            import xarray as _  # noqa: F401
            xarray = True
        except ImportError:
            xarray = False
    return _read_xarray(files, variables) if xarray else _read_numpy(files, variables)
