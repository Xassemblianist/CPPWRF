# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.post's diagnostics against wrf-python's getvar on wrfout files: max |a - b| and max relative difference
per field. Needs wrf-python, netCDF4 and numpy; run with a Python that has them:

    python tools/post/compare_wrfpython.py wrfout_d01_... [--json out.json]

The relative difference is |a - b| / max(|b|, floor) with a floor per field (the field's typical magnitude times 1e-3),
so points near zero do not dominate it. Points missing in one and not the other are counted separately."""
import argparse
import json
import os
import sys
import time
import warnings

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "python"))


def cases(f, wrf):
    """(name, ours, wrf-python's) per diagnostic; both callables."""
    import netCDF4
    ds = netCDF4.Dataset(f)
    lev = [1000., 925., 850., 700., 500., 300., 250., 200.]
    hgt = [0.08, 0.1, 0.12]

    def gv(name, **kw):
        return lambda: wrf.getvar(ds, name, meta=False, **kw)

    out = [
        ("slp", lambda F: F.slp(), gv("slp")),
        ("tk", lambda F: F.tk(), gv("tk")),
        ("rh", lambda F: F.rh(), gv("rh")),
        ("rh2", lambda F: F.rh2(), gv("rh2")),
        ("td", lambda F: F.td(), gv("td")),
        ("td2", lambda F: F.td2(), gv("td2")),
        ("pw", lambda F: F.pw(), gv("pw")),
        ("omg", lambda F: F.omega(), gv("omg")),
        ("avo", lambda F: F.avo(), gv("avo")),
        ("dbz", lambda F: F.dbz(), gv("dbz")),
        ("dbz_varint_liqskin", lambda F: F.dbz(True, True), lambda: wrf.g_dbz.get_dbz(ds, use_varint=True, use_liqskin=True, meta=False)),  # getvar rejects them
        ("mdbz", lambda F: F.dbz().max(axis=0), gv("mdbz")),
        ("cloudfrac_low", lambda F: F.cloudfrac()[0], lambda: gv("cloudfrac")()[0]),
        ("cloudfrac_mid", lambda F: F.cloudfrac()[1], lambda: gv("cloudfrac")()[1]),
        ("cloudfrac_high", lambda F: F.cloudfrac()[2], lambda: gv("cloudfrac")()[2]),
        ("cloudfrac_pres_low", lambda F: F.cloudfrac("pres")[0], lambda: gv("cloudfrac", vert_type="pres")()[0]),
        ("cloudfrac_pres_mid", lambda F: F.cloudfrac("pres")[1], lambda: gv("cloudfrac", vert_type="pres")()[1]),
        ("cloudfrac_pres_high", lambda F: F.cloudfrac("pres")[2], lambda: gv("cloudfrac", vert_type="pres")()[2]),
        ("ctt", lambda F: F.ctt(), gv("ctt")),
        ("cape_2d_mcape", lambda F: F.cape_2d()[0], lambda: gv("cape_2d")()[0]),
        ("cape_2d_mcin", lambda F: F.cape_2d()[1], lambda: gv("cape_2d")()[1]),
        ("cape_2d_lcl", lambda F: F.cape_2d()[2], lambda: gv("cape_2d")()[2]),
        ("cape_2d_lfc", lambda F: F.cape_2d()[3], lambda: gv("cape_2d")()[3]),
        ("cape_3d_cape_lowest", lambda F: F.cape_sb()[0], lambda: gv("cape_3d")()[0][0]),
        ("cape_3d_cin_lowest", lambda F: F.cape_sb()[1], lambda: gv("cape_3d")()[1][0]),
        ("helicity_0-3km", lambda F: F.srh(3000.), gv("helicity", top=3000.)),
        ("helicity_0-1km", lambda F: F.srh(1000.), gv("helicity", top=1000.)),
        ("updraft_helicity_2-5km", lambda F: F.uh(2000., 5000.), gv("updraft_helicity", bottom=2000., top=5000.)),
        ("uvmet10_u", lambda F: F.uv10_earth("uvmet")[0], lambda: gv("uvmet10")()[0]),
        ("uvmet10_v", lambda F: F.uv10_earth("uvmet")[1], lambda: gv("uvmet10")()[1]),
        ("uvmet10_cosalpha_u", lambda F: F.uv10_earth("cosalpha")[0], lambda: gv("uvmet10")()[0]),
        ("uvmet10_cosalpha_v", lambda F: F.uv10_earth("cosalpha")[1], lambda: gv("uvmet10")()[1]),
        ("uvmet_u", lambda F: F.uv_earth("uvmet")[0], lambda: gv("uvmet")()[0]),
        ("vinterp_tk_p_logp", lambda F: F.vinterp(F.tk(), "pressure", lev, "tk", log_p=True),
         lambda: wrf.vinterp(ds, wrf.getvar(ds, "tk", meta=False), "pressure", lev, field_type="tk", log_p=True,
                             meta=False)),
        ("vinterp_z_p_logp", lambda F: F.vinterp(F.height(), "pressure", lev, "ght", log_p=True),
         lambda: wrf.vinterp(ds, wrf.getvar(ds, "z", meta=False), "pressure", lev, field_type="ght", log_p=True,
                             meta=False)),
        ("vinterp_tk_p_extrap", lambda F: F.vinterp(F.tk(), "pressure", lev, "tk", log_p=True, extrapolate=True),
         lambda: wrf.vinterp(ds, wrf.getvar(ds, "tk", meta=False), "pressure", lev, field_type="tk", log_p=True,
                             extrapolate=True, meta=False)),
        ("vinterp_z_p_extrap", lambda F: F.vinterp(F.height(), "pressure", lev, "ght", log_p=True, extrapolate=True),
         lambda: wrf.vinterp(ds, wrf.getvar(ds, "z", meta=False), "pressure", lev, field_type="ght", log_p=True,
                             extrapolate=True, meta=False)),
        ("vinterp_u_aglkm", lambda F: F.vinterp(F.uv_earth("uvmet")[0].astype(F.raw("U").dtype), "ght_agl", hgt),
         lambda: wrf.vinterp(ds, wrf.getvar(ds, "uvmet", meta=False)[0], "ght_agl", hgt, meta=False)),
        ("interplevel_z500", lambda F: F.interplevel(F.height(), F.pressure() * 0.01, 500.)[0],
         lambda: wrf.interplevel(wrf.getvar(ds, "z", meta=False), wrf.getvar(ds, "pressure", meta=False), 500.,
                                 meta=False)),
        ("freezing_level", lambda F: F.interplevel(F.height(), F.tk() - 273.15, 0.)[0],
         lambda: wrf.interplevel(wrf.getvar(ds, "z", meta=False), wrf.getvar(ds, "tc", meta=False), 0., meta=False)),
    ]
    return ds, out


def stats(a, b, fill=9.9692099683868690e36):
    a = np.ma.filled(np.ma.masked_invalid(np.ma.asarray(a, dtype=np.float64)), np.nan)
    b = np.ma.filled(np.ma.masked_invalid(np.ma.asarray(b, dtype=np.float64)), np.nan)
    a = np.where(np.abs(a) > 1e30, np.nan, a)
    b = np.where(np.abs(b) > 1e30, np.nan, b)
    ma, mb = np.isnan(a), np.isnan(b)
    both = ~ma & ~mb
    d = np.abs(a[both] - b[both])
    scale = np.nanmax(np.abs(b[both])) if both.any() else 1.0
    floor = max(scale * 1e-3, 1e-30)
    rel = d / np.maximum(np.abs(b[both]), floor)
    return {"points": int(both.sum()), "missing_mismatch": int((ma != mb).sum()),
            "max_abs": float(d.max()) if d.size else 0.0, "max_rel": float(rel.max()) if rel.size else 0.0,
            "n_differ": int((d > 0).sum()), "range": [float(np.nanmin(b)), float(np.nanmax(b))] if both.any() else []}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--json")
    ap.add_argument("--only")
    args = ap.parse_args(argv)
    warnings.simplefilter("ignore")
    import wrf
    from cppwrf.post.frame import Frame
    report = {}
    for f in args.files:
        ds, items = cases(f, wrf)
        F = Frame(ds, 0)
        rep = report.setdefault(os.path.basename(f), {})
        for name, ours, theirs in items:
            if args.only and name not in args.only.split(","):
                continue
            t0 = time.time()
            a = ours(F)
            t1 = time.time()
            b = theirs()
            t2 = time.time()
            s = stats(a, b)
            s["t_ours"], s["t_wrfpython"] = round(t1 - t0, 3), round(t2 - t1, 3)
            rep[name] = s
            print(f"{os.path.basename(f)} {name:28s} max|d| {s['max_abs']:.3e} rel {s['max_rel']:.3e} "
                  f"differ {s['n_differ']}/{s['points']} miss {s['missing_mismatch']} "
                  f"({s['t_ours']}s / {s['t_wrfpython']}s)", flush=True)
    if args.json:
        with open(args.json, "w") as fp:
            json.dump(report, fp, indent=1)


if __name__ == "__main__":
    main()
