# SPDX-License-Identifier: GPL-3.0-or-later
"""Decode cppwrf-post's GRIB2 files with ecCodes and check them against the NetCDF file of the same run
(cppwrf-post --format both) and the wrfout file:

- every message decodes; its parameter (discipline, category, number), level, statistical processing and grid are
  those of the field it should be;
- its values equal the NetCDF values (converted to the GRIB2 unit) within the packing precision 0.5 * 10**-D plus the
  float32 rounding of the NetCDF file; missing points are missing in both;
- the latitudes / longitudes ecCodes computes from the grid definition match the wrfout's XLAT / XLONG.

    python tools/post/check_grib2.py post_dir wrfout_file [--cfgrib]

Needs ecCodes' Python bindings (eccodes) and netCDF4; --cfgrib also opens the files with xarray + cfgrib."""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "python"))


def identity(h, ec):
    g = lambda k: ec.codes_get(h, k, int)  # noqa: E731
    tmpl = g("productDefinitionTemplateNumber")
    key = (g("discipline"), g("parameterCategory"), g("parameterNumber"), g("typeOfFirstFixedSurface"),
           g("scaledValueOfFirstFixedSurface") if g("typeOfFirstFixedSurface") != 255 else None,
           g("typeOfSecondFixedSurface"),
           g("scaledValueOfSecondFixedSurface") if g("typeOfSecondFixedSurface") != 255 else None,
           tmpl)
    length = g("lengthOfTimeRange") if tmpl == 8 else None
    return key, length


def expected_keys(cfg):
    """{identity: [(netcdf variable, level index, product)]} from the catalogue."""
    from cppwrf.post import products, _select
    sel = _select(products.catalogue(cfg["options"]), cfg)
    out = {}
    for kind in ("surface", "pressure", "height"):
        for p in sel[kind]:
            gr = p.grib
            if gr is None:
                continue
            tmpl = 8 if gr["stat"] else 0
            if kind == "surface":
                levs = [(None, p.name, None)]
            elif kind == "pressure":
                levs = [(lv * 100.0, f"{p.name}_P", k) for k, lv in enumerate(cfg["levels"]["pressure"])]
            else:
                levs = [(lv, f"{p.name}_Z", k) for k, lv in enumerate(cfg["levels"]["height"])]
            for lv, var, k in levs:
                t1, s1, v1 = gr["level"]
                v1 = int(round(lv)) if v1 is None else v1
                l2 = gr.get("level2")
                key = (gr["discipline"], gr["category"], gr["number"], t1, v1 if t1 != 255 else None,
                       l2[0] if l2 else 255, (l2[2] if l2 else None), tmpl)
                out.setdefault(key, []).append((var, k, p))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("post_dir")
    ap.add_argument("wrfout", nargs="+", help="the wrfout files that were post-processed")
    ap.add_argument("-c", "--config")
    ap.add_argument("--cfgrib", action="store_true")
    a = ap.parse_args(argv)
    import eccodes as ec
    import netCDF4
    from cppwrf.post import config as cfgmod
    from cppwrf.post import products
    cfg = cfgmod.load(a.config)
    keys = expected_keys(cfg)
    wrf_ll = {}
    for f in a.wrfout:
        with netCDF4.Dataset(f) as d:
            wrf_ll[d.getncattr("SIMULATION_START_DATE")] = (np.asarray(d["XLAT"][0], np.float64),
                                                           np.asarray(d["XLONG"][0], np.float64))
    lat, lon = next(iter(wrf_ll.values()))
    nmsg = nbad = 0
    worst = {}
    for gpath in sorted(glob.glob(os.path.join(a.post_dir, "*.grib2"))):
        ncpath = gpath[:-6] + ".nc"
        nc = netCDF4.Dataset(ncpath)
        nc.set_auto_mask(False)
        seen = {}
        with open(gpath, "rb") as fh:
            while True:
                h = ec.codes_grib_new_from_file(fh)
                if h is None:
                    break
                nmsg += 1
                key, length = identity(h, ec)
                cands = keys.get(key)
                if not cands:
                    print(f"{os.path.basename(gpath)}: message {nmsg}: unexpected identity {key}")
                    nbad += 1
                    ec.codes_release(h)
                    continue
                # several fields share an identity (e.g. PRECIP_ACC and PRECIP_1H): the time range tells them apart
                i = seen.get(key, 0)
                seen[key] = i + 1
                var, k, p = cands[min(i, len(cands) - 1)]
                if p.grib["stat"] == "acc_interval" and length is not None:
                    hrs = float(cfg["options"]["precip_hours"])
                    match = [c for c in cands if c[2].grib["stat"] == "acc_interval"]
                    if ec.codes_get(h, "indicatorOfUnitForTimeRange") == 1 and length == hrs and match:
                        var, k, p = match[0]
                if p.grib["stat"] == "acc" and length is not None:
                    match = [c for c in cands if c[2].grib["stat"] == "acc"]
                    fc = ec.codes_get(h, "forecastTime")
                    if fc == 0 and match:
                        var, k, p = match[0]
                    elif fc != 0:
                        var, k, p = [c for c in cands if c[2].grib["stat"] == "acc_interval"][0]
                vals = ec.codes_get_values(h).astype(np.float64)
                if ec.codes_get(h, "bitmapPresent"):
                    vals[vals == ec.codes_get(h, "missingValue")] = np.nan
                ny, nx = ec.codes_get(h, "Ny"), ec.codes_get(h, "Nx")
                vals = vals.reshape(ny, nx)
                ref = nc[var][0] if k is None else nc[var][0, k]
                ref = np.asarray(ref, dtype=np.float64)
                ref[ref == nc[var]._FillValue] = np.nan
                ncunit = nc[var].units
                ref = products.convert(ref, ncunit, p.grib["units"]) * p.grib["sign"]
                if p.grib["add_terrain"]:
                    ref = ref + np.asarray(nc["HGT"][0] if "HGT" in nc.variables else 0.0, np.float64)
                D = ec.codes_get(h, "decimalScaleFactor")
                tol = 0.5 * 10.0 ** -D * (1 + 1e-9) + 4 * np.spacing(np.abs(ref).astype(np.float32)).astype(
                    np.float64) * (1.0 if ncunit == p.grib["units"] else 101.0) + 1e-12
                miss = np.isnan(vals) != np.isnan(ref)
                both = ~np.isnan(vals) & ~np.isnan(ref)
                err = np.abs(vals - ref)
                over = both & (err > np.nan_to_num(tol, nan=0.0))
                name = f"{var}[{k}]" if k is not None else var
                w = worst.setdefault(var, [0.0, 0.0, 0, 0])
                if both.any():
                    w[0] = max(w[0], float(err[both].max()))
                    w[1] = max(w[1], float((err[both] / np.maximum(tol[both], 1e-30)).max()))
                w[2] += int(over.sum())
                w[3] += int(miss.sum())
                if over.any() or miss.any():
                    nbad += 1
                    print(f"{os.path.basename(gpath)}: {name}: {int(over.sum())} values beyond the packing precision, "
                          f"{int(miss.sum())} missing mismatches")
                if nmsg == 1:
                    glat = ec.codes_get_array(h, "latitudes").reshape(ny, nx)
                    glon = ec.codes_get_array(h, "longitudes").reshape(ny, nx)
                    dlat = float(np.abs(glat - lat).max())
                    dlon = float(np.abs(((glon - lon + 180) % 360) - 180).max())
                    print(f"grid ({ec.codes_get(h, 'gridType')}): ecCodes latitudes vs XLAT max {dlat:.2e} deg, "
                          f"longitudes vs XLONG max {dlon:.2e} deg")
                ec.codes_release(h)
        nc.close()
    print(f"{nmsg} messages, {nbad} with problems")
    for var, (e, r, o, m) in sorted(worst.items()):
        print(f"  {var:18s} max |grib - netcdf| {e:.3e}  (max {r:.2f} x tolerance)  over {o}  missing-mismatch {m}")
    if a.cfgrib:
        import warnings
        import xarray as xr
        warnings.simplefilter("ignore")
        gpath = sorted(glob.glob(os.path.join(a.post_dir, "*.grib2")))[-1]
        for tol in ("isobaricInhPa", "heightAboveGround", "surface", "meanSea", "atmosphere",
                    "heightAboveGroundLayer", "pressureFromGroundLayer", "cloudTop", "adiabaticCondensation",
                    "freeConvection"):
            for st in ("instant", "accum", "max"):
                try:
                    ds = xr.open_dataset(gpath, engine="cfgrib", indexpath="",
                                         backend_kwargs={"filter_by_keys": {"typeOfLevel": tol, "stepType": st}})
                except Exception as e:  # noqa: BLE001
                    print(f"cfgrib {tol}/{st}: {type(e).__name__}: {str(e).splitlines()[0]}")
                    continue
                if ds.data_vars:
                    print(f"cfgrib {os.path.basename(gpath)} {tol}/{st}: {sorted(ds.data_vars)} "
                          f"dims {dict(ds.sizes)}")
    return 1 if nbad else 0


if __name__ == "__main__":
    sys.exit(main())
