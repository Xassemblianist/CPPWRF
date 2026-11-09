# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-verify-obs: the ISD and METAR parsers on saved real reports (tests/data, sources in tests/data/README.md),
the plain CSV reader, the scores on known answers, station positions, and a whole verification of synthetic wrfout
files against synthetic observations. No network unless CPPWRF_NETWORK_TESTS=1 (then a small ISD and IEM download)."""
import datetime
import math
import os
import unittest

import numpy as np

from fixtures import scratch  # noqa: F401  (fixtures puts the package on sys.path)

from cppwrf.verify_obs import obs as ob
from cppwrf.verify_obs import scores as S
from cppwrf.verify_obs import sources, verify
from cppwrf.verify_obs.model import Run, qnh

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
T0 = datetime.datetime(2026, 11, 8, 18)


def read(name):
    with open(os.path.join(DATA, name)) as f:
        return f.read()


class Isd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.st, cls.obs, cls.qc = ob.parse_isd(read("isd_17130099999_2025_excerpt.csv"))
        cls.by_time = {o.time: o for o in cls.obs}

    def test_station(self):
        s = self.st["17130099999"]
        self.assertAlmostEqual(s.lat, 39.95)
        self.assertAlmostEqual(s.elev, 891.0)
        self.assertIn("ANKARA", s.name)
        self.assertEqual(len(self.obs), 25)

    def test_values(self):
        o = self.by_time[datetime.datetime(2025, 1, 4, 10)]
        self.assertAlmostEqual(o.t2, 7.6)
        self.assertAlmostEqual(o.mslp, 1018.9)
        self.assertAlmostEqual(o.wspd, 1.0)
        self.assertEqual(o.wdir, 210.0)
        u, v = ob.wind_uv(1.0, 210.0)
        self.assertAlmostEqual(o.u, u)
        self.assertAlmostEqual(o.v, v)
        self.assertAlmostEqual(u, 0.5)  # a wind from 210 degrees blows to the north-east

    def test_calm(self):
        o = self.by_time[datetime.datetime(2025, 1, 4, 19)]
        self.assertEqual((o.wspd, o.wdir, o.u, o.v), (0.0, None, 0.0, 0.0))

    def test_precip(self):
        self.assertEqual(self.by_time[datetime.datetime(2025, 1, 4, 12)].precip, [(6, 0.0)])  # trace
        self.assertEqual(self.by_time[datetime.datetime(2025, 1, 4, 18)].precip, [(12, 3.0)])
        self.assertEqual(self.by_time[datetime.datetime(2025, 1, 5, 0)].precip, [(6, 0.8)])

    def test_quality_codes(self):
        # the five rows of 2025 whose temperature has quality code 2 (suspect)
        suspect = [o for o in self.obs if o.time.month > 1]
        self.assertEqual(len(suspect), 5)
        self.assertTrue(all(o.t2 is None for o in suspect))
        self.assertEqual(self.qc["isd_quality_tmp"], 5)
        self.assertTrue(all(o.td2 is not None for o in suspect))

    def test_rh(self):
        o = self.by_time[datetime.datetime(2025, 1, 4, 10)]
        self.assertAlmostEqual(o.rh2, ob.rh_from_td(o.t2, o.td2))
        self.assertAlmostEqual(ob.rh_from_td(10.0, 10.0), 100.0)


class Metar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.st, cls.obs, cls.qc = ob.parse_iem(read("metar_iem_excerpt.csv"))

    def find(self, station, t):
        return [o for o in self.obs if o.station == station and o.time == t][0]

    def test_counts(self):
        self.assertEqual(len(self.obs), 19)
        self.assertEqual(self.qc["metar_error"], 0)
        self.assertAlmostEqual(self.st["LTAC"].elev, 949.0)

    def test_turkish_report(self):
        o = self.find("LTAC", datetime.datetime(2026, 11, 8, 18, 20))  # VRB01KT CAVOK M00/M04 Q1029
        self.assertAlmostEqual(o.wspd, ob.KT)
        self.assertIsNone(o.wdir)
        self.assertIsNone(o.u)
        self.assertEqual((o.t2, o.td2, o.qnh), (0.0, -4.0, 1029.0))
        o = self.find("LTAC", datetime.datetime(2026, 11, 8, 19, 20))  # 31004KT 270V360
        self.assertEqual(o.wdir, 310.0)
        self.assertAlmostEqual(o.wspd, 4 * ob.KT)
        o = self.find("LTAC", datetime.datetime(2026, 11, 8, 22, 20))  # 00000KT
        self.assertEqual((o.wspd, o.u, o.v), (0.0, 0.0, 0.0))

    def test_us_remarks(self):
        # KORD 020751Z 11018G22KT ... 05/01 A2983 RMK AO2 RAB06E42 SLP105 P0000 T00500011: hourly amount -> 08 UTC
        o = self.find("KORD", datetime.datetime(2025, 4, 2, 8))
        self.assertEqual(o.wdir, 110.0)
        self.assertAlmostEqual(o.gust, 22 * ob.KT)
        self.assertAlmostEqual(o.qnh, 29.83 * ob.INHG)
        self.assertAlmostEqual(o.mslp, 1010.5)
        self.assertEqual((o.t2, o.td2), (5.0, 1.1))
        self.assertEqual(o.precip, [(1, 0.0)])
        # KDSM 020854Z ... SLP951 ... P0023 6....: SLP below 1000 hPa, 0.23 in in the hour, 3 h amount at 09 UTC
        o = self.find("KDSM", datetime.datetime(2025, 4, 2, 9))
        self.assertAlmostEqual(o.mslp, 995.1)
        self.assertIn((1, 0.23 * ob.INCH), o.precip)
        self.assertTrue(any(h == 3 for h, _ in o.precip))

    def test_synthetic(self):
        t = datetime.datetime(2026, 11, 9, 12)
        o = ob.parse_metar("METAR LTBA 091200Z 27005MPS 9999 M10/M15 Q1015=", t)
        self.assertEqual((o.wspd, o.wdir, o.t2, o.td2, o.qnh), (5.0, 270.0, -10.0, -15.0, 1015.0))
        o = ob.parse_metar("COR LTBA 091200Z 36010G20KT 9999 05/// Q1015", t)
        self.assertEqual((o.t2, o.td2), (5.0, None))
        for bad in ("XX 091200Z 27005KT", "LTBA 0912Z 27005KT 10/05", "LTBA 091200Z NIL",
                    "LTBA 091200Z CAVOK NOSIG"):
            with self.assertRaises(ob.MetarError, msg=bad):
                ob.parse_metar(bad, t)
        qc = ob.QC()
        ob.parse_metar("LTBA 091300Z 27005KT 10/05 Q1015", t, qc)
        self.assertEqual(qc["metar_time_mismatch"], 1)

    def test_gross_limits(self):
        qc = ob.QC()
        o = ob.parse_metar("LTBA 091200Z 27005KT 10/15 Q0500", datetime.datetime(2026, 11, 9, 12), qc)
        o = ob.finish(o, qc)
        self.assertIsNone(o.td2)
        self.assertIsNone(o.qnh)
        self.assertEqual(qc["td_above_t"], 1)
        self.assertEqual(qc["gross_qnh"], 1)


class PlainFile(unittest.TestCase):
    def test_turkish_semicolons(self):
        text = ("istasyon;tarih;enlem;boylam;yukseklik;sicaklik;ruzgar_hizi;ruzgar_yonu;yagis_6h;precip_6h\n"
                "17130;2026-11-09 06:00;39,95;32,88;891;-1,5;2,0;90;;3,2\n"
                "17130;2026-11-09 07:00;39,95;32,88;891;NA;M;;;\n")
        st, obs, qc = ob.parse_file(text)
        self.assertAlmostEqual(st["17130"].lat, 39.95)
        self.assertEqual(obs[0].t2, -1.5)
        self.assertAlmostEqual(obs[0].u, -2.0)
        self.assertAlmostEqual(obs[0].v, 0.0, places=12)
        self.assertEqual(obs[0].precip, [(6, 3.2)])
        self.assertIsNone(obs[1].t2)
        self.assertIsNone(obs[1].wspd)

    def test_missing_columns(self):
        with self.assertRaises(ValueError):
            ob.parse_file("lat,lon,t2\n1,2,3\n")


class Scores(unittest.TestCase):
    def test_continuous(self):
        d = S.continuous([1.0, 2.0, 3.0, 4.0], [0.0, 2.0, 2.0, 6.0])
        self.assertAlmostEqual(d["bias"], 0.0)
        self.assertAlmostEqual(d["mae"], 1.0)
        self.assertAlmostEqual(d["rmse"], math.sqrt(1.5))
        self.assertAlmostEqual(d["corr"], np.corrcoef([1, 2, 3, 4], [0, 2, 2, 6])[0, 1])

    def test_direction(self):
        d = S.direction_error([350.0, 10.0, 180.0], [10.0, 350.0, 170.0])
        self.assertAlmostEqual(d["mae"], (20 + 20 + 10) / 3)
        self.assertAlmostEqual(d["bias"], (-20 + 20 + 10) / 3)

    def test_vector(self):
        self.assertAlmostEqual(S.vector_rmse([3.0], [4.0], [0.0], [0.0])["rmse"], 5.0)

    def test_contingency(self):
        f = [0, 2, 2, 0, 7, 0, 0, 0]
        o = [0, 2, 0, 2, 7, 0, 0, 1]
        c = S.contingency(f, o, 1.0)
        self.assertEqual((c["hits"], c["false_alarms"], c["misses"], c["correct_negatives"]), (2, 1, 2, 3))
        self.assertAlmostEqual(c["pod"], 0.5)
        self.assertAlmostEqual(c["far"], 1 / 3)
        self.assertAlmostEqual(c["csi"], 0.4)
        self.assertAlmostEqual(c["fbias"], 0.75)
        ar = 3 * 4 / 8
        self.assertAlmostEqual(c["ets"], (2 - ar) / (5 - ar))
        none = S.contingency([0, 0], [0, 0], 1.0)
        self.assertIsNone(none["pod"])

    def test_qnh(self):
        # the standard atmosphere at 1000 m (898.75 hPa) gives back 1013.25 hPa
        self.assertAlmostEqual(float(qnh(898.746, 1000.0)), 1013.25, places=1)
        self.assertAlmostEqual(float(qnh(1013.25, 0.0)), 1013.25, places=9)


def synthetic_run(d, hours=7):
    from test_post_output import write_full_wrfout
    p = os.path.join(d, "wrfout_d01_2026-11-08_18:00:00")
    write_full_wrfout(p, [T0 + datetime.timedelta(hours=h) for h in range(hours)], nz=20, ny=12, nx=14, dtype="f8")
    return p


class Positions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = scratch("cppwrf_vo_")
        cls.wrf = Run(synthetic_run(cls.dir, 2))

    def test_grid_index(self):
        r = self.wrf
        for j, i in ((5, 6), (0, 0), (11, 13)):
            fi, fj = r.grid_index(r.lat[j, i], r.lon[j, i])
            self.assertAlmostEqual(fi, i, places=6)
            self.assertAlmostEqual(fj, j, places=6)
        la = 0.25 * (r.lat[5, 6] + r.lat[5, 7] + r.lat[6, 6] + r.lat[6, 7])
        lo = 0.25 * (r.lon[5, 6] + r.lon[5, 7] + r.lon[6, 6] + r.lon[6, 7])
        fi, fj = r.grid_index(la, lo)
        self.assertAlmostEqual(fi, 6.5, places=6)
        self.assertAlmostEqual(fj, 5.5, places=6)
        self.assertIsNone(r.grid_index(0.0, 0.0))

    def test_boundary(self):
        r = Run(self.wrf.paths, boundary=3)
        st = [ob.Station("in", r.lat[5, 6], r.lon[5, 6]), ob.Station("edge", r.lat[1, 6], r.lon[1, 6]),
              ob.Station("out", 10.0, 10.0)]
        locs = r.locate(st)
        self.assertEqual([L.inside for L in locs], [True, False, False])
        self.assertIn("boundary", locs[1].reason)
        self.assertEqual(r.boundary_source, "option")
        self.assertEqual(Run(self.wrf.paths).boundary, 5)


class WholeVerification(unittest.TestCase):
    """Observations made from the model's own fields with known errors: T2 1 K colder (bias +1), wind 1 m/s
    weaker, QNH equal, 6 h precipitation 6 mm where the model has 9 mm."""

    def test_verify(self):
        import netCDF4
        d = scratch("cppwrf_vo_")
        path = synthetic_run(d)
        j, i = 6, 7
        rows = ["station,time,lat,lon,elev,t2,td2,wspd,wdir,qnh,precip,precip_hours"]
        with netCDF4.Dataset(path) as ds:
            lat, lon, hgt = (float(ds[v][0, j, i]) for v in ("XLAT", "XLONG", "HGT"))
            for n in range(len(ds["Times"])):
                t = T0 + datetime.timedelta(hours=n)
                t2 = float(ds["T2"][n, j, i]) - 273.15
                ca, sa = float(ds["COSALPHA"][n, j, i]), float(ds["SINALPHA"][n, j, i])
                u, v = 3.0 * ca - 4.0 * sa, 4.0 * ca + 3.0 * sa
                spd = math.hypot(u, v) - 1.0
                wdir = math.degrees(math.atan2(-u, -v)) % 360.0
                q = float(qnh(float(ds["PSFC"][n, j, i]) / 100.0, hgt))
                pr = "6,6" if n == 6 else ","
                rows.append(f"S1,{t:%Y-%m-%d %H:%M},{lat},{lon},{hgt},{t2 - 1.0},,{spd},{wdir},{q},{pr}")
        obs_file = os.path.join(d, "obs.csv")
        with open(obs_file, "w") as f:
            f.write("\n".join(rows) + "\n")
        out = os.path.join(d, "verify")
        res = verify(path, obs_source="file", obs_file=obs_file, output_dir=out, boundary=2, offline=True,
                     compare=path, labels=["a", "b"])
        sc = res["overall"]["a"]
        self.assertEqual(sc["t2"]["n"], 7)
        self.assertAlmostEqual(sc["t2"]["bias"], 1.0, places=6)
        self.assertAlmostEqual(sc["t2"]["rmse"], 1.0, places=6)
        self.assertAlmostEqual(sc["wspd"]["bias"], 1.0, places=6)
        self.assertAlmostEqual(sc["wdir"]["mae"], 0.0, places=6)
        self.assertAlmostEqual(sc["wind"]["rmse"], 1.0, places=6)
        # the synthetic atmosphere is 12 K warmer than the standard one: its QNH (~1240 hPa) fails the gross limits
        self.assertEqual(res["qc"]["observations"]["gross_qnh"], 7)
        self.assertNotIn("qnh", sc)
        p6 = sc["precip"][6]
        self.assertEqual(p6["continuous"]["n"], 1)
        self.assertAlmostEqual(p6["continuous"]["bias"], 3.0, places=6)
        c = {x["threshold"]: x for x in p6["contingency"]}
        self.assertEqual((c[5.0]["hits"], c[10.0]["false_alarms"], c[10.0]["correct_negatives"]), (1, 0, 1))
        self.assertEqual(res["overall"]["b"]["t2"], sc["t2"])
        for name in ("summary.md", "pairs.csv", "scores.json", "scores_by_lead.csv", "contingency.csv",
                     os.path.join("plots", "lead_t2.svg"), os.path.join("plots", "map_bias_t2.svg")):
            self.assertTrue(os.path.exists(os.path.join(out, name)), name)
        # nearest-point sampling of a station on a grid point gives the same; no height correction: T2 at 0 m apart
        res = verify(path, obs_source="file", obs_file=obs_file, output_dir=out, boundary=2, offline=True,
                     interpolation="nearest", height_correction=False, make_plots=False)
        self.assertAlmostEqual(res["overall"][next(iter(res["overall"]))]["t2"]["bias"], 1.0, places=6)


@unittest.skipUnless(os.environ.get("CPPWRF_NETWORK_TESTS") == "1", "network test: set CPPWRF_NETWORK_TESTS=1")
class Network(unittest.TestCase):
    def test_isd_and_iem(self):
        f = sources.Fetcher(scratch("cppwrf_obs_cache_"), pause=1.0)
        t0, t1 = datetime.datetime(2025, 1, 4, 0), datetime.datetime(2025, 1, 4, 23)
        texts = sources.isd(f, ["17130099999"], t0, t1)
        st, obs, qc = ob.parse_isd(texts[0])
        self.assertEqual(len(obs), 24)
        texts = sources.iem(f, ["LTAC"], t0, t1)
        st, obs, qc = ob.parse_iem(texts[0])
        self.assertGreater(len(obs), 30)
        n = f.requests
        sources.isd(f, ["17130099999"], t0, t1)
        self.assertEqual(f.requests, n)  # cached


if __name__ == "__main__":
    unittest.main()
