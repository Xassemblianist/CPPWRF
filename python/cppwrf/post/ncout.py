# SPDX-License-Identifier: GPL-3.0-or-later
"""CF-1.8 NetCDF output of one valid time: dimensions time (1), [pressure], [height], y, x; 2-D latitude /
longitude as auxiliary coordinates; projection x / y with a grid_mapping variable when the projection allows it
(grid.py); CF time with forecast_reference_time and forecast_period."""
import numpy as np

from . import products as pr


def write(path, grid, ref_time, valid_time, results, plevels, hlevels, cfg, history=""):
    """results: [(product, values)], values (ny, nx) or (nlev, ny, nx) in the product's native unit, NaN missing."""
    import netCDF4
    nc_cfg = cfg["netcdf"]
    units_cfg = cfg["units"]
    dtype = np.dtype(nc_cfg.get("dtype", "float32"))
    complevel = int(nc_cfg.get("compression", 4) or 0)
    fill = netCDF4.default_fillvals["f4" if dtype == np.float32 else "f8"]
    mapping, has_xy = grid.cf_mapping()
    ds = netCDF4.Dataset(path, "w", format="NETCDF4")
    try:
        ds.Conventions = "CF-1.8"
        ds.title = "CPPWRF post-processed fields (cppwrf-post)"
        ds.institution = "CPPWRF"
        ds.source = f"WRF-ARW output: {grid.attrs.get('TITLE', '').strip()}"
        ds.history = history
        ds.references = "docs/POST.md; diagnostics as wrf-python 1.3.4.1"
        for k in ("MAP_PROJ", "DX", "DY", "TRUELAT1", "TRUELAT2", "STAND_LON", "CEN_LAT", "CEN_LON", "MOAD_CEN_LAT",
                  "GRID_ID", "SIMULATION_START_DATE"):
            if k in grid.attrs:
                ds.setncattr(f"WRF_{k}", grid.attrs[k])
        ds.createDimension("time", None)
        ds.createDimension("y", grid.ny)
        ds.createDimension("x", grid.nx)
        tunits = f"hours since {ref_time:%Y-%m-%d %H:%M:%S}"
        tv = ds.createVariable("time", "f8", ("time",))
        tv.standard_name = "time"
        tv.long_name = "valid time"
        tv.units = tunits
        tv.calendar = "proleptic_gregorian"
        tv.axis = "T"
        lead = (valid_time - ref_time).total_seconds() / 3600.0
        tv[:] = [lead]
        rt = ds.createVariable("forecast_reference_time", "f8")
        rt.standard_name = "forecast_reference_time"
        rt.units = tunits
        rt.calendar = "proleptic_gregorian"
        rt.assignValue(0.0)
        fp = ds.createVariable("forecast_period", "f8", ("time",))
        fp.standard_name = "forecast_period"
        fp.units = "hours"
        fp[:] = [lead]
        if has_xy:
            for name, n, vals, sn in (("x", grid.nx, grid.x, "projection_x_coordinate"),
                                      ("y", grid.ny, grid.y, "projection_y_coordinate")):
                v = ds.createVariable(name, "f8", (name,))
                v.standard_name = sn
                v.units = "m"
                v.axis = name.upper()
                v[:] = vals
        lat = ds.createVariable("lat", "f8", ("y", "x"), zlib=complevel > 0, complevel=complevel or 4)
        lat.standard_name = "latitude"
        lat.units = "degrees_north"
        lat[:] = grid.lat
        lon = ds.createVariable("lon", "f8", ("y", "x"), zlib=complevel > 0, complevel=complevel or 4)
        lon.standard_name = "longitude"
        lon.units = "degrees_east"
        lon[:] = grid.lon
        if mapping:
            crs = ds.createVariable("crs", "i4")
            for k, v in mapping.items():
                crs.setncattr(k, v)
        if plevels:
            ds.createDimension("pressure", len(plevels))
            v = ds.createVariable("pressure", "f8", ("pressure",))
            v.standard_name = "air_pressure"
            v.long_name = "pressure level"
            v.units = "hPa"
            v.positive = "down"
            v.axis = "Z"
            v[:] = plevels
        if hlevels:
            ds.createDimension("height", len(hlevels))
            v = ds.createVariable("height", "f8", ("height",))
            v.standard_name = "height"
            v.long_name = "height above ground"
            v.units = "m"
            v.positive = "up"
            v.axis = "Z"
            v[:] = hlevels
        scalar_heights = {}
        for prod, values in results:
            name = prod.name if prod.kind == "2d" else f"{prod.name}_{'P' if prod.kind == 'pressure' else 'Z'}"
            dims = ("time", "y", "x") if prod.kind == "2d" else \
                ("time", "pressure" if prod.kind == "pressure" else "height", "y", "x")
            unit = units_cfg.get(prod.name if prod.kind == "2d" else name, prod.units)
            data = pr.convert(np.asarray(values, dtype=np.float64), prod.units, unit)
            v = ds.createVariable(name, dtype, dims, zlib=complevel > 0, complevel=complevel or 4, fill_value=fill)
            v.long_name = prod.long_name
            v.units = unit
            if prod.standard_name:
                v.standard_name = prod.standard_name
            coords = ["lat", "lon"]
            if prod.height is not None:
                hname = f"height_{prod.height:g}m"
                if hname not in scalar_heights:
                    hv = ds.createVariable(hname, "f8")
                    hv.standard_name = "height"
                    hv.units = "m"
                    hv.positive = "up"
                    hv.assignValue(prod.height)
                    scalar_heights[hname] = hv
                coords.append(hname)
            v.coordinates = " ".join(coords)
            if mapping:
                v.grid_mapping = "crs"
            if prod.grib and prod.grib.get("stat") in ("acc", "acc_interval"):
                v.cell_methods = "time: sum"
            elif prod.grib and prod.grib.get("stat") == "max_output":
                v.cell_methods = "time: maximum"
            v[0] = np.where(np.isfinite(data), data, fill)
    finally:
        ds.close()
