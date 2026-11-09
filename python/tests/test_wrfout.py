# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.open_wrfout and cppwrf.diagnostics on synthetic wrfout files."""
import datetime
import math
import os
import unittest

import numpy as np

from fixtures import column, scratch, write_wrfout

import cppwrf
from cppwrf import diagnostics as dg

T0 = datetime.datetime(2026, 10, 12)
HOURS = [T0 + datetime.timedelta(hours=h) for h in range(4)]


def files(bucket=None, rot=True):
    d = scratch("cppwrf_wo_")
    # two files of two frames each, written out of order on purpose
    write_wrfout(os.path.join(d, "wrfout_d01_2026-10-12_02:00:00"), HOURS[2:], seed=0, bucket=bucket, rot=rot)
    write_wrfout(os.path.join(d, "wrfout_d01_2026-10-12_00:00:00"), HOURS[:2], seed=0, bucket=bucket, rot=rot)
    return d


def seaprs_loop(z, t, p, q):
    """DCOMPUTESEAPRS column by column, as the Fortran (the vectorised port is checked against it)."""
    R, G, GAMMA, TC, PCONST = 287.0, 9.81, 0.0065, 273.16 + 17.5, 10000.0
    nz, ny, nx = p.shape
    out = np.zeros((ny, nx))
    for j in range(ny):
        for i in range(nx):
            level = next(k + 1 for k in range(nz) if p[k, j, i] < p[0, j, i] - PCONST)
            klo = max(level - 1, 1)
            khi = min(klo + 1, nz - 1)
            plo, phi = p[klo - 1, j, i], p[khi - 1, j, i]
            tlo = t[klo - 1, j, i] * (1 + 0.608 * q[klo - 1, j, i])
            thi = t[khi - 1, j, i] * (1 + 0.608 * q[khi - 1, j, i])
            zlo, zhi = z[klo - 1, j, i], z[khi - 1, j, i]
            pc = p[0, j, i] - PCONST
            tc = thi - (thi - tlo) * math.log(pc / phi) * math.log(plo / phi)
            zc = zhi - (zhi - zlo) * math.log(pc / phi) * math.log(plo / phi)
            ts = tc * (p[0, j, i] / pc) ** (GAMMA * R / G)
            tsl = tc + GAMMA * zc
            tsl = TC if (ts <= TC and not tsl < TC) else TC - 0.005 * (ts - TC) ** 2
            # the Fortran's 0.01 is a default-real (single precision) literal
            out[j, i] = float(np.float32(0.01)) * (p[0, j, i] * math.exp(2 * G * z[0, j, i] / (R * (tsl + ts))))
    return out


class Numpy(unittest.TestCase):
    def test_open(self):
        d = files()
        w = cppwrf.open_wrfout(d, xarray=False)
        self.assertEqual(w.times, HOURS)
        self.assertEqual(w["T2"].shape, (4, 3, 4))
        self.assertEqual(w.lat.shape, (3, 4))
        self.assertEqual(w.units["T2"], "K")
        self.assertEqual(w.attrs["TITLE"], "synthetic")
        w2 = cppwrf.open_wrfout(os.path.join(d, "wrfout_d01_*"), variables=["T2"], xarray=False)
        self.assertNotIn("U10", w2)
        with self.assertRaises(FileNotFoundError):
            cppwrf.open_wrfout(d, domain=2, xarray=False)

    def test_t2_wind(self):
        w = cppwrf.open_wrfout(files(rot=False), xarray=False)
        np.testing.assert_allclose(dg.t2_celsius(w)[:, 0, 0], [16.85, 17.85, 18.85, 19.85], atol=1e-4)
        spd, wdir = dg.wind10(w)
        np.testing.assert_allclose(spd, 5.0, rtol=1e-6)
        # u = 3 (to the east), v = 4 (to the north): from the south-west, 216.87 degrees
        np.testing.assert_allclose(wdir, 180.0 + math.degrees(math.atan2(3, 4)), atol=1e-4)

    def test_wind_rotation(self):
        w = cppwrf.open_wrfout(files(rot=True), xarray=False)
        _, wg = dg.wind10(w, earth=False)
        _, we = dg.wind10(w, earth=True)
        np.testing.assert_allclose((wg - we) % 360.0, math.degrees(0.1), atol=1e-3)

    def test_precip(self):
        w = cppwrf.open_wrfout(files(bucket=100.0), xarray=False)
        tot = dg.precipitation(w)[:, 0, 0]
        np.testing.assert_allclose(tot, [0.0, 3.0, 106.0, 109.0], atol=1e-5)  # 3 mm/h + one bucket of 100 mm at 2 h
        hourly = dg.precip_accum(w, 1)[:, 0, 0]
        self.assertTrue(np.isnan(hourly[0]))
        np.testing.assert_allclose(hourly[1:], [3.0, 103.0, 3.0], atol=1e-5)
        np.testing.assert_allclose(dg.precip_accum(w, 3)[3, 0, 0], 109.0, atol=1e-5)

    def test_slp(self):
        zw, p, t = column()
        z = 0.5 * (zw[1:] + zw[:-1])
        q = np.full_like(p, 0.005)
        np.testing.assert_allclose(dg.seaprs(z, t, p, q), seaprs_loop(z, t, p, q), rtol=1e-13)
        w = cppwrf.open_wrfout(files(), xarray=False)
        s = dg.slp(w)
        self.assertEqual(s.shape, (4, 3, 4))
        # standard atmosphere columns: sea-level pressure close to 1013.25 hPa over every terrain height
        self.assertTrue(np.all(np.abs(s - 1013.25) < 5.0), s[0])

    def test_slp_needs_100hpa(self):
        p = np.full((3, 1, 1), 100000.0)
        with self.assertRaises(ValueError):
            dg.seaprs(np.zeros_like(p), np.full_like(p, 280.0), p, np.zeros_like(p))


class XArray(unittest.TestCase):
    def setUp(self):
        try:
            import xarray  # noqa: F401
        except ImportError:
            self.skipTest("xarray is not installed")

    def test_dataset(self):
        ds = cppwrf.open_wrfout(files(bucket=100.0))
        self.assertEqual(ds.sizes["Time"], 4)
        self.assertIn("XLAT", ds.coords)
        self.assertEqual(str(ds["Time"].values[1])[:19], "2026-10-12T01:00:00")
        np.testing.assert_allclose(dg.t2_celsius(ds).values[:, 0, 0], [16.85, 17.85, 18.85, 19.85], atol=1e-4)
        np.testing.assert_allclose(dg.precip_accum(ds, 1).values[1:, 0, 0], [3.0, 103.0, 3.0], atol=1e-5)
        self.assertEqual(dg.slp(ds).dims, ("Time", "south_north", "west_east"))


if __name__ == "__main__":
    unittest.main()
