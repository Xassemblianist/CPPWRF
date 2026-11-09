# SPDX-License-Identifier: GPL-3.0-or-later
"""The GRIB2 grid templates of cppwrf-post (3.30 Lambert, 3.20 polar stereographic, 3.10 Mercator, 3.0 lat-lon)
against ecCodes: synthetic WRF grids (lat / lon from the spherical inverse formulas of each projection, Snyder 1987),
one message each, decoded with ecCodes, whose latitudes / longitudes are compared with the grid's.

    python tools/post/check_projections.py     (needs eccodes)"""
import datetime
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "python"))

R = 6370000.0


def inverse(attrs, x, y):
    mp = attrs["MAP_PROJ"]
    lon0 = attrs["STAND_LON"]
    t1 = math.radians(attrs["TRUELAT1"])
    if mp == 1:
        t2 = math.radians(attrs["TRUELAT2"])
        n = math.log(math.cos(t1) / math.cos(t2)) / math.log(math.tan(math.pi / 4 + t2 / 2)
                                                             / math.tan(math.pi / 4 + t1 / 2)) \
            if abs(t1 - t2) > 1e-9 else math.sin(t1)
        F = math.cos(t1) * math.tan(math.pi / 4 + t1 / 2) ** n / n
        rho0 = R * F / math.tan(math.pi / 4 + math.radians(attrs["MOAD_CEN_LAT"]) / 2) ** n
        rho = np.sign(n) * np.sqrt(x ** 2 + (rho0 - y) ** 2)
        theta = np.arctan2(x, rho0 - y)
        lat = 2 * np.arctan((R * F / rho) ** (1 / n)) - np.pi / 2
        lon = lon0 + np.degrees(theta / n)
    elif mp == 2:
        rho = np.sqrt(x ** 2 + y ** 2)
        lat = np.pi / 2 - 2 * np.arctan(rho / (R * (1 + math.sin(t1))))
        lon = lon0 + np.degrees(np.arctan2(x, -y))
    elif mp == 3:
        k = math.cos(t1)
        lon = lon0 + np.degrees(x / (R * k))
        lat = 2 * np.arctan(np.exp(y / (R * k))) - np.pi / 2
    return np.degrees(lat), ((lon + 180) % 360) - 180


def grids():
    nx, ny = 60, 50
    out = []
    for attrs in (dict(MAP_PROJ=1, TRUELAT1=30.0, TRUELAT2=60.0, STAND_LON=-98.0, MOAD_CEN_LAT=40.0, DX=12000.0,
                       DY=12000.0),
                  dict(MAP_PROJ=1, TRUELAT1=39.5, TRUELAT2=39.5, STAND_LON=33.0, MOAD_CEN_LAT=39.5, DX=9000.0,
                       DY=9000.0),
                  dict(MAP_PROJ=2, TRUELAT1=60.0, TRUELAT2=60.0, STAND_LON=-45.0, MOAD_CEN_LAT=75.0, DX=20000.0,
                       DY=20000.0),
                  dict(MAP_PROJ=3, TRUELAT1=20.0, TRUELAT2=0.0, STAND_LON=120.0, MOAD_CEN_LAT=10.0, DX=15000.0,
                       DY=15000.0)):
        from cppwrf.post.grid import Grid
        g0 = Grid(attrs, np.array([[attrs["MOAD_CEN_LAT"]]]), np.array([[attrs["STAND_LON"] + 2.0]]))
        xc, yc = g0.forward(np.array(attrs["MOAD_CEN_LAT"]), np.array(attrs["STAND_LON"] + 2.0))
        x = xc + (np.arange(nx) - (nx - 1) / 2) * attrs["DX"]
        y = yc + (np.arange(ny) - (ny - 1) / 2) * attrs["DY"]
        lat, lon = inverse(attrs, *np.meshgrid(x, y))
        out.append((attrs, lat, lon))
    lat, lon = np.meshgrid(np.linspace(30, 45, 31), np.linspace(20, 50, 61), indexing="ij")
    out.append((dict(MAP_PROJ=6, POLE_LAT=90.0, STAND_LON=0.0, TRUELAT1=0.0, DX=0.5, DY=0.5), lat, lon))
    return out


def main():
    import eccodes as ec
    from cppwrf.post import grib2
    from cppwrf.post.grid import Grid
    ok = True
    t = datetime.datetime(2026, 11, 9)
    for attrs, lat, lon in grids():
        g = Grid(attrs, lat, lon)
        with tempfile.NamedTemporaryFile(suffix=".grib2", delete=False) as f:
            path = f.name
        w = grib2.Writer(path, g, t, t)
        field = np.cos(np.radians(lat)) * 10
        w.write(field, {"discipline": 0, "category": 0, "number": 0, "level": (1, 0, 0), "stat": None}, None, 3)
        w.close()
        with open(path, "rb") as fh:
            h = ec.codes_grib_new_from_file(fh)
            glat = ec.codes_get_array(h, "latitudes").reshape(lat.shape)
            glon = ec.codes_get_array(h, "longitudes").reshape(lat.shape)
            vals = ec.codes_get_values(h).reshape(lat.shape)
            gt = ec.codes_get(h, "gridType")
            ec.codes_release(h)
        os.unlink(path)
        dlat = float(np.abs(glat - lat).max())
        dlon = float(np.abs(((glon - lon + 180) % 360) - 180).max())
        dv = float(np.abs(vals - field).max())
        fit = f", x / y fit {g.fit_error:.2e} m" if g.fit_error is not None else ""
        good = dlat < 1e-5 and dlon < 1e-5 and dv <= 0.0005 + 1e-9
        ok &= good
        print(f"MAP_PROJ {attrs['MAP_PROJ']} ({gt}): max |lat| {dlat:.1e} deg, |lon| {dlon:.1e} deg, values "
              f"{dv:.1e}{fit}: {'ok' if good else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
