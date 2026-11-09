# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-post's output: the GRIB2 encoder, the projections, the configuration and a whole run on synthetic wrfout
files (NetCDF and GRIB2). No network, no GPU; the checks with ecCodes and a CF checker are tools/post/check_*.py."""
import datetime
import math
import os
import struct
import unittest

import numpy as np

from fixtures import scratch  # noqa: F401  (fixtures puts the package on sys.path)

from cppwrf.post import config as cfgmod
from cppwrf.post import grib2, products, process
from cppwrf.post.frame import Frame
from cppwrf.post.grid import Grid

T0 = datetime.datetime(2026, 11, 8, 18)


def lambert_latlon(nx, ny, dx, truelat=39.5, stand_lon=33.0, cen_lat=39.5):
    """A Lambert grid's latitudes / longitudes (inverse of grid.forward), centred on (cen_lat, stand_lon + 1)."""
    attrs = dict(MAP_PROJ=1, TRUELAT1=truelat, TRUELAT2=truelat, STAND_LON=stand_lon, MOAD_CEN_LAT=cen_lat, DX=dx,
                 DY=dx)
    g = Grid(attrs, np.zeros((1, 1)), np.zeros((1, 1)))
    n = g.cone()
    p1 = math.radians(truelat)
    F = math.cos(p1) * math.tan(math.pi / 4 + p1 / 2) ** n / n
    rho0 = 6370000.0 * F / math.tan(math.pi / 4 + math.radians(cen_lat) / 2) ** n
    xc, yc = g.forward(np.array(cen_lat), np.array(stand_lon + 1.0))
    x = xc + (np.arange(nx) - (nx - 1) / 2) * dx
    y = yc + (np.arange(ny) - (ny - 1) / 2) * dx
    X, Y = np.meshgrid(x, y)
    rho = np.sqrt(X ** 2 + (rho0 - Y) ** 2)
    lat = np.degrees(2 * np.arctan((6370000.0 * F / rho) ** (1 / n)) - np.pi / 2)
    lon = stand_lon + np.degrees(np.arctan2(X, rho0 - Y) / n)
    return attrs, lat, lon


def write_full_wrfout(path, times, nz=30, ny=12, nx=14, dtype="f4"):
    """A synthetic wrfout file with every variable cppwrf-post reads: standard-atmosphere columns, a moist unstable
    boundary layer, veering winds, rain and snow growing with time, a Lambert grid."""
    import netCDF4
    attrs, lat, lon = lambert_latlon(nx, ny, 9000.0)
    zs = np.linspace(0.0, 600.0, nx)[None, :] * np.ones((ny, 1))
    zw = zs[None] + np.linspace(0.0, 16000.0, nz + 1)[:, None, None]
    zm = 0.5 * (zw[1:] + zw[:-1])
    t = 300.0 - 0.0065 * zm
    p = 101325.0 * (t / 288.0) ** (9.81 / (287.0 * 0.0065))
    agl = zm - zs[None]
    q = np.where(agl < 1500.0, 0.016, 0.016 * np.exp(-(agl - 1500.0) / 1500.0))
    u = np.broadcast_to(np.minimum(agl, 6000.0)[..., :1] * 0 + 5.0 + 0.002 * agl.mean(axis=(1, 2))[:, None, None],
                        (nz, ny, nx + 1)).copy()
    v = np.broadcast_to(5.0 * np.sin(np.minimum(agl.mean(axis=(1, 2)), 3000.0) / 3000.0)[:, None, None],
                        (nz, ny + 1, nx)).copy()
    with netCDF4.Dataset(path, "w") as ds:
        for k, val in dict(attrs, TITLE=" OUTPUT FROM SYNTHETIC", SIMULATION_START_DATE=f"{T0:%Y-%m-%d_%H:%M:%S}",
                           START_DATE=f"{T0:%Y-%m-%d_%H:%M:%S}", CEN_LAT=39.5, CEN_LON=34.0, GRID_ID=1,
                           BUCKET_MM=-1.0).items():
            ds.setncattr(k, val)
        ds.createDimension("Time", None)
        ds.createDimension("DateStrLen", 19)
        for n, k in (("bottom_top", nz), ("bottom_top_stag", nz + 1), ("south_north", ny), ("west_east", nx),
                     ("south_north_stag", ny + 1), ("west_east_stag", nx + 1)):
            ds.createDimension(n, k)
        tv = ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
        d2 = ("Time", "south_north", "west_east")
        d3 = ("Time", "bottom_top", "south_north", "west_east")
        dw = ("Time", "bottom_top_stag", "south_north", "west_east")
        mk = lambda name, dims, typ=dtype: ds.createVariable(name, typ, dims)  # noqa: E731
        for i, tm in enumerate(times):
            h = (tm - T0).total_seconds() / 3600.0
            tv[i] = netCDF4.stringtochar(np.array([tm.strftime("%Y-%m-%d_%H:%M:%S")], "S19"))
            vals = {
                "XLAT": (d2, lat), "XLONG": (d2, lon), "HGT": (d2, zs), "PSFC": (d2, p[0] * 1.002),
                "T2": (d2, t[0] + 0.5 - 0.3 * h), "Q2": (d2, q[0]), "U10": (d2, np.full((ny, nx), 3.0)),
                "V10": (d2, np.full((ny, nx), 4.0)), "COSALPHA": (d2, np.cos(np.radians(lon - 33.0) * 0.636)),
                "SINALPHA": (d2, np.sin(np.radians(lon - 33.0) * 0.636)),
                "RAINC": (d2, np.full((ny, nx), 0.5 * h)), "RAINSH": (d2, np.zeros((ny, nx))),
                "RAINNC": (d2, np.full((ny, nx), 1.0 * h)), "SNOWNC": (d2, np.full((ny, nx), 0.95 * h)),
                "GRAUPELNC": (d2, np.zeros((ny, nx))), "SNOWH": (d2, np.full((ny, nx), 0.1)),
                "SNOW": (d2, np.full((ny, nx), 20.0)), "MAPFAC_M": (d2, np.ones((ny, nx))),
                "F": (d2, 2 * 7.292e-5 * np.sin(np.radians(lat))),
                "MAPFAC_U": (("Time", "south_north", "west_east_stag"), np.ones((ny, nx + 1))),
                "MAPFAC_V": (("Time", "south_north_stag", "west_east"), np.ones((ny + 1, nx))),
                "P": (d3, p * 0.02), "PB": (d3, p * 0.98), "T": (d3, t * (1e5 / p) ** (287.0 / 1004.5) - 300.0),
                "QVAPOR": (d3, q), "QCLOUD": (d3, np.where((agl > 2000) & (agl < 3000), 1e-3, 0.0)),
                "QRAIN": (d3, np.where(agl < 2000, 1e-3, 0.0)), "QICE": (d3, np.zeros((nz, ny, nx))),
                "QSNOW": (d3, np.where(agl > 4000, 2e-4, 0.0)), "QGRAUP": (d3, np.zeros((nz, ny, nx))),
                "REFL_10CM": (d3, np.where(agl < 2000, 35.0, -30.0)),
                "U": (("Time", "bottom_top", "south_north", "west_east_stag"), u),
                "V": (("Time", "bottom_top", "south_north_stag", "west_east"), v),
                "W": (dw, 0.5 * np.sin(np.pi * (zw - zs[None]) / 16000.0)), "PH": (dw, np.zeros_like(zw)),
                "PHB": (dw, zw * 9.81),
            }
            for name, (dims, a) in vals.items():
                if name not in ds.variables:
                    mk(name, dims)
                ds[name][i] = a
    return path


class Grib2(unittest.TestCase):
    def roundtrip(self, field, D):
        sec, n = grib2.pack_simple(field, D)
        g = {"discipline": 0, "category": 0, "number": 0, "level": (1, 0, 0), "stat": None}
        attrs, lat, lon = lambert_latlon(field.shape[1], field.shape[0], 3000.0)
        grid = Grid(attrs, lat, lon)
        msg = grib2.message(0, grib2.section1(255, 0, 30, T0), grid.grib2_section3(),
                            grib2.section4(0, 0, (1, 0, 0), None, 3600), sec)
        self.assertEqual(struct.unpack(">Q", msg[8:16])[0], len(msg))
        self.assertTrue(msg.endswith(b"7777"))
        return grib2.decode_simple(msg)[3].reshape(field.shape), g

    def test_precision_and_bitmap(self):
        rng = np.random.default_rng(1)
        f = 250 + 40 * rng.random((9, 11))
        f[2, 3] = np.nan
        f[0, 0] = -12.5
        out, _ = self.roundtrip(f, 2)
        self.assertTrue(np.isnan(out[2, 3]))
        m = np.isfinite(f)
        self.assertLessEqual(np.abs(out[m] - f[m]).max(), 0.005 + 1e-9)
        const, _ = self.roundtrip(np.full((4, 5), -30.0), 1)
        np.testing.assert_array_equal(const, -30.0)
        empty, _ = self.roundtrip(np.full((4, 5), np.nan), 1)
        self.assertTrue(np.isnan(empty).all())

    def test_sign_magnitude(self):
        self.assertEqual(grib2.s32(-5), b"\x80\x00\x00\x05")
        self.assertEqual(grib2.s16(-1), b"\x80\x01")
        self.assertEqual(grib2.s32(7), b"\x00\x00\x00\x07")

    def test_template_4_8(self):
        s = grib2.section4(1, 8, (1, 0, 0), None, 18 * 3600, (1, 17 * 3600, 3600), T0 + datetime.timedelta(hours=18))
        self.assertEqual(struct.unpack(">I", s[:4])[0], 58)
        self.assertEqual(struct.unpack(">H", s[7:9])[0], 8)
        self.assertEqual(struct.unpack(">I", s[18:22])[0], 17)  # forecast time: the start of the interval
        self.assertEqual(s[46], 1)  # accumulation
        self.assertEqual(struct.unpack(">I", s[49:53])[0], 1)  # one hour long


class Projection(unittest.TestCase):
    def test_lambert_fit(self):
        attrs, lat, lon = lambert_latlon(20, 15, 9000.0)
        g = Grid(attrs, lat, lon)
        self.assertLess(g.fit_error, 1e-3)
        np.testing.assert_allclose(np.diff(g.x), 9000.0)
        a, has_xy = g.cf_mapping()
        self.assertTrue(has_xy)
        self.assertEqual(a["grid_mapping_name"], "lambert_conformal_conic")

    def test_mercator_and_polar(self):
        g = Grid(dict(MAP_PROJ=3, TRUELAT1=0.0, STAND_LON=10.0, DX=1000.0, DY=1000.0), np.zeros((1, 1)),
                 np.full((1, 1), 10.0))
        x, y = g.forward(np.array(0.0), np.array(11.0))
        self.assertAlmostEqual(float(x), 6370000.0 * math.radians(1.0), places=6)
        self.assertAlmostEqual(float(y), 0.0, places=6)
        g = Grid(dict(MAP_PROJ=2, TRUELAT1=90.0, STAND_LON=0.0, DX=1000.0, DY=1000.0), np.full((1, 1), 90.0),
                 np.zeros((1, 1)))
        x, y = g.forward(np.array(90.0), np.array(0.0))
        self.assertAlmostEqual(float(x), 0.0)
        self.assertAlmostEqual(float(y), 0.0)


class Config(unittest.TestCase):
    def test_ini_yaml_and_errors(self):
        d = scratch("cppwrf_post_cfg_")
        ini = os.path.join(d, "post.ini")
        with open(ini, "w") as f:
            f.write("[output]\nformat = both\n[fields]\nsurface = T2, MSLP\npressure = T, GHT\n"
                    "[levels]\npressure = 850, 500\n[units]\nT2 = degC\n[grib2.precision]\nT2 = 1\n")
        c = cfgmod.load(ini)
        self.assertEqual(c["output"]["format"], "both")
        self.assertEqual(c["fields"]["surface"], ["T2", "MSLP"])
        self.assertEqual(c["levels"]["pressure"], [850, 500])
        self.assertEqual(c["grib2"]["precision"]["T2"], 1)
        self.assertEqual(c["units"]["T2"], "degC")
        with open(ini, "w") as f:
            f.write("[options]\nwind_rotaton = uvmet\n")
        with self.assertRaises(cfgmod.ConfigError):
            cfgmod.load(ini)
        try:
            import yaml  # noqa: F401
        except ImportError:
            return
        y = os.path.join(d, "post.yaml")
        with open(y, "w") as f:
            f.write("output: {format: grib2}\nlevels: {height: [100]}\noptions: {wind_rotation: uvmet}\n")
        c = cfgmod.load(y)
        self.assertEqual((c["output"]["format"], c["levels"]["height"], c["options"]["wind_rotation"]),
                         ("grib2", [100], "uvmet"))

    def test_units(self):
        self.assertAlmostEqual(float(products.convert(np.array(273.15), "K", "degC")), 0.0)
        self.assertAlmostEqual(float(products.convert(np.array(1013.0), "hPa", "Pa")), 101300.0)
        self.assertAlmostEqual(float(products.convert(np.array(10.0), "m s-1", "kt")), 19.438444924406)

    def test_ptype(self):
        z = np.zeros(5)
        d = {"total": np.array([0.0, 2.0, 2.0, 2.0, 2.0]), "grid": np.array([0.0, 2.0, 2.0, 2.0, 2.0]),
             "snow": np.array([0, 0, 2.0, 1.0, 0.2]), "graupel": np.array([0, 0, 0, 0, 1.9]), "hail": z}
        out = products.ptype(d, np.array([270.0, 270.0, 270.0, 280.0, 270.0]))
        self.assertTrue(np.isnan(out[0]))
        np.testing.assert_array_equal(out[1:], [3.0, 5.0, 7.0, 9.0])


class Run(unittest.TestCase):
    def test_process_both_formats(self):
        import netCDF4
        d = scratch("cppwrf_post_run_")
        times = [T0 + datetime.timedelta(hours=h) for h in range(3)]
        for t in times:
            write_full_wrfout(os.path.join(d, f"wrfout_d01_{t:%Y-%m-%d_%H:%M:%S}"), [t])
        out = process(os.path.join(d, "wrfout_d01_*"), output_dir=os.path.join(d, "post"), fmt="both",
                      overrides={"levels": {"pressure": [925, 850, 500], "height": [100]}})
        self.assertEqual(len(out), 6)
        nc = netCDF4.Dataset(out[-2])
        self.assertEqual(nc.Conventions, "CF-1.8")
        self.assertEqual(nc["crs"].grid_mapping_name, "lambert_conformal_conic")
        self.assertEqual(nc["T_P"].shape, (1, 3, 12, 14))
        np.testing.assert_allclose(nc["PRECIP_1H"][0], 1.5, rtol=1e-6)  # 1.5 mm per hour
        np.testing.assert_allclose(nc["PRECIP_ACC"][0], 3.0, rtol=1e-6)
        self.assertTrue((nc["PTYPE"][0] == 5).all())  # 95 % of the grid-scale precipitation is snow
        self.assertGreater(float(nc["CAPE_SB"][0].max()), 100.0)
        self.assertEqual(nc["MSLP"].units, "hPa")
        with open(out[-1], "rb") as f:
            msgs = grib2.split(f.read())
        mslp = [m for m in msgs if grib2.decode_simple(m)[:3] == (0, 3, 1)]
        self.assertEqual(len(mslp), 1)
        vals = grib2.decode_simple(mslp[0])[3].reshape(12, 14)
        np.testing.assert_allclose(vals, nc["MSLP"][0] * 100.0, atol=0.5 + 0.01)
        first = netCDF4.Dataset(out[0])
        self.assertTrue(np.all(np.ma.getmaskarray(first["PRECIP_1H"][0])))  # no earlier frame
        with open(out[1], "rb") as f:
            n0 = len(grib2.split(f.read()))
        self.assertEqual(len(msgs), n0 + 10)  # at +0 h no accumulation is written; now 5 since the start, 5 in 1 h
        nc.close()
        first.close()

    def test_frame_emulation(self):
        d = scratch("cppwrf_post_emu_")
        p32 = write_full_wrfout(os.path.join(d, "a32"), [T0], dtype="f4")
        import netCDF4
        with netCDF4.Dataset(p32) as ds:
            f1, f2 = Frame(ds, 0, emulate=True), Frame(ds, 0, emulate=False)
            a, b = f1.slp(), f2.slp()
            self.assertEqual(a.dtype, np.float32)
            self.assertEqual(b.dtype, np.float64)
            np.testing.assert_allclose(a, b, rtol=1e-6)


if __name__ == "__main__":
    unittest.main()
