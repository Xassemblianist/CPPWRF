# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-compare, cppwrf-bench (with a stand-in binary) and cppwrf-fit; perf_reference against docs/PERF.md."""
import contextlib
import datetime
import io
import json
import os
import re
import shutil
import unittest

import numpy as np

from fixtures import HERE, NAMELIST, fake_binary, run_dir, scratch, write_wrfout

from cppwrf import bench, compare, fit
from cppwrf import perf_reference as ref

PERF_MD = os.path.join(HERE, "..", "..", "docs", "PERF.md")

FAKE_BENCH = r'''
import re
cells = 1
for k in ("e_we", "e_sn", "e_vert"):
    cells *= int(re.search(k + r" = (\d+)", open("namelist.input").read()).group(1)) - 1
print("cppwrf: device memory after init: %d MiB used of 6144 MiB; stack limit 1024 bytes x 30720 resident threads "
      "(30 MiB)" % (100 + cells // 1000))
print("cppwrf: done, 6 steps, stepping 0.500 s, 123.00 ns/cell/step")
'''


def quiet(f, *a):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
        rc = f(*a)
    return rc, out.getvalue()


class CompareTest(unittest.TestCase):
    def setUp(self):
        self.a, self.b = scratch("cppwrf_cmp_a_"), scratch("cppwrf_cmp_b_")
        t = [datetime.datetime(2026, 10, 12, h) for h in (0, 1)]
        for d in (self.a, self.b):
            write_wrfout(os.path.join(d, "wrfout_d01_2026-10-12_00:00:00"), t)

    def test_identical(self):
        rc, out = quiet(compare.main, [self.a, self.b])
        self.assertEqual(rc, 0, out)
        self.assertIn("IDENTICAL", out)

    def test_different(self):
        import netCDF4
        with netCDF4.Dataset(os.path.join(self.b, "wrfout_d01_2026-10-12_00:00:00"), "a") as ds:
            ds["T2"][1, 2, 3] = ds["T2"][1, 2, 3] + 0.5
        rc, out = quiet(compare.main, [self.a, self.b, "--json"])
        self.assertEqual(rc, 1)
        r = json.loads(out)[0]
        t2 = next(v for v in r["variables"] if v["name"] == "T2")
        self.assertEqual((t2["frames_differing"], t2["points_differing"]), (1, 1))
        self.assertAlmostEqual(t2["max_abs"], 0.5, places=4)
        self.assertIn("south_north=2, west_east=3", t2["first"])
        self.assertTrue(all(v["points_differing"] == 0 for v in r["variables"] if v["name"] != "T2"))

    def test_no_common_stream(self):
        rc, _ = quiet(compare.main, [self.a, scratch("cppwrf_cmp_c_")])
        self.assertEqual(rc, 2)

    def test_ulp(self):
        x = np.array([1.0, -0.0, 2.0], np.float32)
        y = np.array([np.nextafter(np.float32(1), np.float32(2)), 0.0, 2.0], np.float32)
        self.assertEqual(list(compare.ulp_distance(x, y)), [1.0, 0.0, 0.0])
        self.assertEqual(compare.ulp_distance(np.array([1e300]), np.array([np.nextafter(1e300, 0)]))[0], 1.0)


class PerfReferenceTest(unittest.TestCase):
    @unittest.skipUnless(os.path.isfile(PERF_MD), "docs/PERF.md not in this tree")
    def test_matches_perf_md(self):
        with open(PERF_MD) as f:
            text = f.read()
        ns = text.split("Time per step and per cell")[1].split("Wall time of the whole run")[0]
        for case, modes in ref.NS_PER_CELL_STEP.items():
            row = re.search(rf"^\| {case} \|(.*)\|$", ns, re.M).group(1).split("|")
            vals = [float(x) for x in row]
            self.assertEqual((modes["gpu-fp32"], modes["gpu-fp64"], modes["cpu-fp64"]), tuple(vals[2:5]), case)
        mem = text.split("GPU memory:")[1]
        for case, modes in ref.GPU_MEMORY_MIB.items():
            label = "P4, P4R15" if case == "P4" else case
            row = re.search(rf"^\| {re.escape(label)} \|(.*)\|$", mem, re.M).group(1).split("|")
            peaks = [tuple(int(v) for v in re.findall(r"\d+", row[i])) for i in (0, 2)]
            self.assertEqual([modes["gpu-fp32"], modes["gpu-fp64"]], peaks, case)
        self.assertIn("dt 60 s", text)  # time_factor's radiation fraction assumes P4's dt of 60 s


class BenchTest(unittest.TestCase):
    def test_bench_mode(self):
        d = run_dir()
        exe = fake_binary(d)
        with open(exe, "a") as f:
            f.write(FAKE_BENCH)
        r = bench.bench_mode(exe, "cpu-fp64", (40, 30, 20), 6, 1, d)
        self.assertEqual(r["ns_per_cell_step"], 123.0)
        self.assertAlmostEqual(r["sim_per_wall_stepping"], 6 * 12 / 0.5)
        self.assertEqual([x["cells"] for x in r["runs"]], [24000, 20 * 15 * 20])
        self.assertEqual(r["memory_model"]["what"], "device_used_mib")  # the device line was printed
        self.assertAlmostEqual(r["memory_model"]["b_mib_per_cell"], (124 - 106) / (24000 - 6000))
        self.assertEqual(r["stack_reserve_mib"], 30)
        self.assertAlmostEqual(r["speed_vs_reference"], 134.7 / 123.0)

    def test_sounding_and_size(self):
        rows = bench.sounding().splitlines()
        self.assertEqual(len(rows[0].split()), 3)
        self.assertGreaterEqual(float(rows[-1].split()[0]), 20000.0)
        self.assertEqual(bench.parse_size("100x80X40"), (100, 80, 40))


def bench_result(mode="cpu-fp64"):
    run = {"size": [100, 100, 40], "cells": 400000, "steps": 20, "stepping_s": 8.0, "ns_per_cell_step": 1000.0,
           "wall_s": 9.0, "host_peak_mib": 700.0}
    r = {"mode": mode, "ns_per_cell_step": 1000.0, "runs": [run],
         "memory_model": {"what": "host_peak_mib", "a_mib": 50.0, "b_mib_per_cell": 0.002}}
    if mode.startswith("gpu"):
        r.update(device_total_mib=6144, stack_reserve_mib=30, memory_model={"what": "device_used_mib", "a_mib": 300.0,
                                                                            "b_mib_per_cell": 0.001})
    return r


class FitTest(unittest.TestCase):
    def test_time_factor(self):
        dom = lambda radt, dt=60.0: fit.Domain(10, 10, 10, dt, 1, True, radt)  # noqa: E731
        for m in ("gpu-fp32", "gpu-fp64", "cpu-fp64"):
            self.assertAlmostEqual(fit.time_factor(m, dom(1)), ref.physics_time_factor(m, "P4"))
            self.assertAlmostEqual(fit.time_factor(m, dom(15)), ref.physics_time_factor(m, "P4R15"))
            self.assertAlmostEqual(fit.time_factor(m, dom(30, 30.0)), fit.time_factor(m, dom(60)))
            self.assertEqual(fit.time_factor(m, fit.Domain(10, 10, 10, 60.0, 1, False, 0.0)), 1.0)

    def test_namelist_domains(self):
        text = NAMELIST.replace("max_dom = 1, e_we = 11, e_sn = 11, e_vert = 11",
                                "max_dom = 2, e_we = 11, 31, e_sn = 11, 21, e_vert = 11, 11, parent_id = 0, 1, "
                                "parent_time_step_ratio = 1, 3")
        text = text.replace("cu_physics = 0,", "cu_physics = 0, 1, radt = 10, 5,")
        doms = fit.domains_from_namelist(fit.nl.parse(text))
        self.assertEqual([(d.nx, d.ny, d.nz, d.dt, d.steps) for d in doms], [(10, 10, 10, 12.0, 5), (30, 20, 10, 4.0, 15)])
        self.assertEqual([d.physics for d in doms], [False, True])
        self.assertEqual([d.radt for d in doms], [0.0, 0.0])  # cumulus but no radiation

    def test_estimate(self):
        dom = fit.Domain(200, 100, 40, 12.0, 30, False, 0.0)
        e = fit.estimate(bench_result(), [dom])
        self.assertAlmostEqual(e.stepping_s, 800000 * 30 * 1000e-9)
        self.assertAlmostEqual(e.time_s, e.stepping_s + 1.0)
        self.assertAlmostEqual(e.memory_mib, 50.0 + 0.002 * 800000)
        g = fit.estimate(bench_result("gpu-fp32"), [fit.Domain(200, 100, 40, 60.0, 30, True, 15.0)])
        want = 300.0 + 0.001 * 800000 * ref.physics_memory_factor("gpu-fp32") + 30 * (ref.stack_reserve_factor(
            "gpu-fp32") - 1)
        self.assertAlmostEqual(g.memory_mib, want)
        self.assertEqual((g.memory_kind, g.available_mib, g.verdict), ("device", 6144, "fits"))
        big = fit.estimate(bench_result("gpu-fp32"), [fit.Domain(1000, 1000, 50, 60.0, 1, True, 15.0)])
        self.assertEqual(big.verdict, "does not fit")

    def test_main(self):
        d = scratch("cppwrf_fit_")
        p = os.path.join(d, "bench.json")
        with open(p, "w") as f:
            json.dump({"results": [bench_result("gpu-fp32"), bench_result("gpu-fp64")]}, f)
        rc, out = quiet(fit.main, ["--grid", "100x100x40", "--bench", p, "--json"])
        self.assertEqual(rc, 0)
        self.assertEqual([e["mode"] for e in json.loads(out)["estimates"]], ["gpu-fp32", "gpu-fp64"])
        rc, out = quiet(fit.main, ["--grid", "2000x2000x50", "--bench", p, "--modes", "gpu-fp32"])
        self.assertEqual(rc, 1)
        self.assertIn("does not fit", out)
        self.assertEqual(quiet(fit.main, ["--grid", "10x10x10", "--bench", os.path.join(d, "none.json")])[0], 2)
        shutil.rmtree(d, True)


if __name__ == "__main__":
    unittest.main()
