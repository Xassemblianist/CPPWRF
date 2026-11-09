# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.post's port of wrf-python's routines (wrfpy.py) on synthetic fields with known answers. No network, no GPU,
no wrf-python (the comparison with wrf-python itself is tools/post/compare_wrfpython.py)."""
import math
import unittest

import numpy as np

from fixtures import column  # noqa: F401  (fixtures puts the package on sys.path)

from cppwrf.post import wrfpy as w


class Thermo(unittest.TestCase):
    def test_tk_rh_td(self):
        p = np.array([100000.0, 85000.0, 50000.0])
        th = np.array([290.0, 300.0, 320.0])
        tk = w.tk(p, th)
        np.testing.assert_allclose(tk, th * (p / 1e5) ** (287.0 / 1004.5), rtol=1e-15)
        # saturated air: rh 100, and the dew point equals the temperature where Td's formula inverts es's
        t = np.array([273.15 + 20.0])
        es = 6.112 * math.exp(17.67 * 20.0 / (20.0 + 243.5))
        qvs = 0.622 * es / (1000.0 - (1 - 0.622) * es)
        np.testing.assert_allclose(w.rh(np.array([qvs]), np.array([1e5]), t), [100.0], rtol=1e-12)
        np.testing.assert_allclose(w.rh(np.array([qvs / 2]), np.array([1e5]), t), [50.0], rtol=1e-12)
        self.assertEqual(w.rh(np.array([2 * qvs]), np.array([1e5]), t)[0], 100.0)
        e = 0.622 * 0 + qvs * 1000.0 / (0.622 + qvs)
        np.testing.assert_allclose(w.td(np.array([1000.0]), np.array([qvs])),
                                   [(243.5 * math.log(e) - 440.8) / (19.48 - math.log(e))], rtol=1e-14)

    def test_slp_sea_level_column(self):
        zw, p, t = column(nz=40, ny=1, nx=1)
        zm = 0.5 * (zw[1:] + zw[:-1])
        slp = w.seaprs(zm, t, p, np.zeros_like(p))
        # terrain 0: the lowest level is near sea level, so the sea-level pressure is about its pressure
        self.assertAlmostEqual(float(slp[0, 0]), 1013.25, delta=0.6)

    def test_pw_omega(self):
        nz = 10
        rho, dz, q = 1.1, 100.0, 0.01
        p = np.full((nz, 1, 1), rho * 287.0 * 280.0)
        tv = np.full((nz, 1, 1), 280.0)
        ht = (np.arange(nz + 1) * dz)[:, None, None]
        self.assertAlmostEqual(float(w.pw(p, tv, np.full_like(p, q), ht)[0, 0]), q * rho * dz * nz, places=10)
        om = w.omega(np.zeros((1,)), np.array([280.0]), np.array([1.0]), np.array([rho * 287.0 * 280.0]))
        self.assertAlmostEqual(float(om[0]), -rho * 9.81, places=10)

    def test_dbz_rain(self):
        prs, tmk, qr = np.array([90000.0]), np.array([290.0]), np.array([1e-3])
        rho = prs / (287.0 * tmk)
        ze = 720.0 * 1e18 * (1 / (math.pi * 1000.0)) ** 1.75 * (rho * qr) ** 1.75 / 8e6 ** 0.75
        z = w.dbz(prs, tmk, np.zeros(1), qr, np.zeros(1), np.zeros(1), 1)
        np.testing.assert_allclose(z, 10 * np.log10(ze), rtol=1e-14)
        self.assertEqual(float(w.dbz(prs, tmk, np.zeros(1), np.zeros(1), np.zeros(1), np.zeros(1), 1)[0]), -30.0)


class Interp(unittest.TestCase):
    def test_interpz3d_linear(self):
        z = np.linspace(0, 10000, 21)[:, None, None] * np.ones((1, 2, 3))
        out = w.interpz3d(2 * z + 1, z, [1234.5, 20000.0])
        np.testing.assert_allclose(out[0], 2 * 1234.5 + 1, rtol=1e-14)
        self.assertTrue((out[1] == w.DEFAULT_FILL).all())

    def test_vintrp_logp(self):
        p = 1e5 * np.exp(-np.linspace(0, 2, 30))[:, None, None] * np.ones((1, 2, 2))  # Pa
        f = 3.0 * np.log(p / 100.0) + 7.0  # linear in log p
        z = np.zeros_like(p)
        two = np.ones((2, 2))
        out = w.vintrp(f, p, np.full_like(p, 280.0), z, z, two, two * 1000, two * 1000, p / 100.0,
                       [850.0, 500.0, 1100.0],
                       logp=1)
        np.testing.assert_allclose(out[0], 3 * math.log(850.0) + 7, rtol=1e-13)
        np.testing.assert_allclose(out[1], 3 * math.log(500.0) + 7, rtol=1e-13)
        self.assertTrue((out[2] == w.DEFAULT_FILL).all())  # below the lowest level, no extrapolation
        ext = w.vintrp(f, p, np.full_like(p, 280.0), z, z, two, two * 1000, two * 1000, p / 100.0, [1100.0], extrap=1,
                       logp=1)
        np.testing.assert_allclose(ext[0], f[0])  # extrapolation of a field of no type: the lowest level's value

    def test_filter2d(self):
        a = np.full((5, 6), 4.0)
        np.testing.assert_allclose(w.filter2d(a, 3), a)
        d = np.zeros((5, 5))
        d[2, 2] = 6.0
        s = w.filter2d(d, 1, cenweight=2.0)
        self.assertAlmostEqual(s[2, 2], 2.0)  # coef 1/6: (1*6) twice over the two passes
        self.assertAlmostEqual(s[1, 2], 1.0)
        self.assertAlmostEqual(s.sum(), 6.0)


class Kinematics(unittest.TestCase):
    def test_absvort_solid_rotation(self):
        nz, ny, nx, dx, om = 2, 6, 7, 1000.0, 1e-4
        xu = (np.arange(nx + 1) - 0.5) * dx
        yu = np.arange(ny) * dx
        xv = np.arange(nx) * dx
        yv = (np.arange(ny + 1) - 0.5) * dx
        u = np.broadcast_to(-om * yu[:, None] + 0 * xu[None, :], (nz, ny, nx + 1))
        v = np.broadcast_to(om * xv[None, :] + 0 * yv[:, None], (nz, ny + 1, nx))
        av = w.absvort(u, v, np.ones((ny, nx + 1)), np.ones((ny + 1, nx)), np.ones((ny, nx)), np.full((ny, nx), 1e-4),
                       dx, dx)
        np.testing.assert_allclose(av, (2 * om + 1e-4) * 1e5, rtol=1e-12)

    def test_srh(self):
        nz = 40
        z = np.linspace(10, 12000, nz)[:, None, None] * np.ones((1, 2, 2))
        ter = np.zeros((2, 2))
        lat = np.full((2, 2), 40.0)
        u = np.full_like(z, 7.0)
        self.assertTrue(np.allclose(w.srh(u, np.full_like(z, -3.0), z, ter, lat, 3000.0), 0.0))
        # veering (clockwise turning) wind in the northern hemisphere: positive helicity
        ang = np.radians(np.minimum(z, 3000.0) / 3000.0 * 90.0)
        sp = 10.0
        srh = w.srh(-sp * np.cos(ang) * 0 + sp * np.sin(ang), sp * np.cos(ang) * -1 * -1 * 0 - sp * np.cos(ang) * 0
                    + sp * np.cos(ang), z, ter, lat, 3000.0)
        self.assertTrue((srh > 0).all())

    def test_udhel(self):
        nz, ny, nx, dx = 30, 8, 9, 1000.0
        zp = np.linspace(0, 15000, nz + 1)[:, None, None] * np.ones((1, ny, nx))
        us = np.zeros((nz, ny, nx))
        vs = np.zeros((nz, ny, nx))
        self.assertTrue((w.udhel(zp, np.ones((ny, nx)), us, vs, np.ones((nz + 1, ny, nx)), dx, dx) == 0).all())
        om = 1e-3  # vertical vorticity 2 om with v = om x, u = -om y; w = 2 m/s
        X, Y = np.meshgrid(np.arange(nx) * dx, np.arange(ny) * dx)
        us = np.broadcast_to(-om * Y, (nz, ny, nx))
        vs = np.broadcast_to(om * X, (nz, ny, nx))
        uh = w.udhel(zp, np.ones((ny, nx)), us, vs, np.full((nz + 1, ny, nx), 2.0), dx, dx)
        np.testing.assert_allclose(uh[1:-2, 1:-2], 2.0 * 2 * om * 3000.0, rtol=1e-12)
        self.assertTrue((uh[0] == 0).all() and (uh[:, -2:] == 0).all())

    def test_uvmet_and_wind(self):
        u, v = w.uvmet(np.array([1.0]), np.array([0.0]), np.array([40.0]), np.array([43.0]), 33.0, 0.5)
        a = math.radians(10.0 * 0.5)
        np.testing.assert_allclose([u[0], v[0]], [math.cos(a), -math.sin(a)], rtol=1e-14)
        self.assertAlmostEqual(float(w.wdir(np.array([0.0]), np.array([-5.0]))[0]), 0.0)  # from the north
        self.assertAlmostEqual(float(w.wdir(np.array([-5.0]), np.array([0.0]))[0]), 90.0)  # from the east
        self.assertAlmostEqual(w.uvmet_cone(1, 39.5, 39.5), math.sin(math.radians(39.5)))


class Clouds(unittest.TestCase):
    def test_cloudfrac(self):
        z = np.linspace(50, 12000, 40)[:, None, None] * np.ones((1, 1, 1))
        rh = np.where(z < 1000, 100.0, 50.0)  # the mid layer starts at the last level below 2000 m (Fortran)
        lo, mi, hi = w.cloudfrac(z, rh, 1, 300.0, 2000.0, 6000.0)
        self.assertEqual((lo[0, 0], mi[0, 0], hi[0, 0]), (1.0, 0.0, 0.0))
        lo, mi, hi = w.cloudfrac(z + 500, rh, 1, 300.0, 2000.0, 6000.0)  # lowest level above 300 m: low missing
        self.assertEqual(lo[0, 0], w.DEFAULT_FILL)

    def test_ctt(self):
        nz = 30
        z = np.linspace(100, 15000, nz)[:, None, None] * np.ones((1, 1, 1))
        t = 290.0 - 0.0065 * z
        p = 1013.25 * (t / 290.0) ** 5.25
        qw = np.where((z > 5000) & (z < 6000), 0.5, 0.0)  # g/kg, a thick cloud
        c = w.ctt(p, t, np.zeros_like(p), qw, np.zeros_like(p), z, np.zeros((1, 1)))
        self.assertTrue(5000 * -0.0065 + 290 - 273.15 - 8 < c[0, 0] < 290 - 0.0065 * 5000 - 273.15 + 1)
        clear = w.ctt(p, t, np.zeros_like(p), np.zeros_like(p), np.zeros_like(p), z, np.zeros((1, 1)))
        self.assertAlmostEqual(float(clear[0, 0]), float(t[0, 0, 0] - 273.15), places=9)


class Cape(unittest.TestCase):
    def sounding(self, unstable=True):
        nz = 40
        z = np.linspace(20, 16000, nz)[:, None, None] * np.ones((1, 1, 2))
        t = np.where(z < 12000, 302.0 - 0.0068 * z, 302.0 - 0.0068 * 12000)
        p = 1000.0 * np.exp(-z / 8000.0)
        q = np.where(z < 1000, 0.018 if unstable else 0.001, 0.001)
        return p, t, np.full_like(p, 0.0) + q, z

    def test_table_nodes(self):
        thte, prs, tmk = w.psadi_table()
        self.assertEqual(thte.shape, (150,))
        jt, ip = 40, 30
        np.testing.assert_allclose(w.tonpsadiabat(np.array([thte[jt]]), np.array([prs[ip]])), [tmk[ip, jt]])

    def test_unstable_and_stable(self):
        p, t, q, z = self.sounding(True)
        ter, sfp = np.zeros((1, 2)), np.full((1, 2), 1003.0)
        cape, cin = w.cape_3d(p, t, q, z, ter, sfp)
        self.assertGreater(cape[0, 0, 0], 1000.0)  # surface-based parcel of a moist, conditionally unstable sounding
        self.assertEqual(cape[-1, 0, 0], 0.0)  # the top level is 0, as DCAPECALC3D sets it
        mu = w.cape_2d(p, t, q, z, ter, sfp)
        self.assertGreater(mu[0][0, 0], 1000.0)
        self.assertGreaterEqual(mu[3][0, 0], mu[2][0, 0])  # LFC at or above LCL
        # a well-mixed boundary layer (dry adiabatic, constant q, up to 1500 m): the mixed-layer parcel is the lowest
        # level's, so ML and SB CAPE nearly agree (DCAPECALC2D's LCL formula differs slightly from DCAPECALC3D's)
        tm = np.where(z < 1500, 302.0 - 0.0098 * z, 302.0 - 0.0098 * 1500 - 0.0068 * (z - 1500))
        tm = np.maximum(tm, tm.min())
        pm = 1000.0 * np.exp(-z / 8000.0)
        qm = np.where(z < 1500, 0.016, 0.001)
        sb = w.cape_3d(pm, tm, qm, z, ter, sfp)[0][0, 0, 0]
        ml = w.cape_2d(pm, tm, qm, z, ter, sfp, parcel="ml", ml_depth=100.0)[0][0, 0]
        self.assertGreater(sb, 1000.0)
        self.assertLess(abs(ml - sb) / sb, 0.1)
        p, t, q, z = self.sounding(False)
        cape, cin = w.cape_3d(p, t, q, z, ter, sfp)
        self.assertTrue(cape[0, 0, 0] < 100.0 or cape[0, 0, 0] == w.DEFAULT_FILL)
        self.assertEqual(cin[0, 0, 0], w.DEFAULT_FILL)  # CIN is missing where CAPE < 100

    def test_lowest_parcel_equals_cape3d(self):
        from cppwrf.post.frame import _cape_lowest
        p, t, q, z = self.sounding(True)
        ter, sfp = np.zeros((1, 2)), np.full((1, 2), 1003.0)
        c3, i3 = w.cape_3d(p, t, q, z, ter, sfp)
        c, i = _cape_lowest(p, t, q, z, ter, sfp)
        np.testing.assert_array_equal(c, c3[0])
        np.testing.assert_array_equal(i, i3[0])


if __name__ == "__main__":
    unittest.main()
