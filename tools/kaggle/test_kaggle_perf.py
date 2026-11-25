#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline tests of kaggle_perf.sh: the plan, the staging (--dry-run), the run commands, the log parsers, one run of
a stand-in binary through the worker's run loop, and the report. No Kaggle, no GPU, no model build.

  python3 -B tools/kaggle/test_kaggle_perf.py
"""

import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import py_compile
import subprocess
import sys
import tarfile
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "bench"))
import kaggle_perf as perf  # noqa: E402
import kg_perf_body as body  # noqa: E402

LOG = """cppwrf: device memory after set-up: 2100 MiB used of 15095 MiB; stack limit 12400 bytes x 40960 resident threads (484 MiB)
Timing for Writing wrfout_d01_2026-10-12_00:00:00 for domain        1:    0.50000 elapsed seconds
Timing for main: time 2026-10-12_00:01:00 on domain   1:    2.00000 elapsed seconds
Timing for main: time 2026-10-12_00:02:00 on domain   1:    0.25000 elapsed seconds
Timing for main: time 2026-10-12_00:03:00 on domain   1:    0.25000 elapsed seconds
Timing for main: time 2026-10-12_00:04:00 on domain   1:    0.25000 elapsed seconds
cppwrf: done, 4 steps, stepping 2.750 s, 1.00 ns/cell/step
cppwrf halo: worker 0: exchanges 300: 1.250 s in all: pack 0.010 s, wait for the stream 0.900 s, transfer 0.300 s, unpack 0.040 s
cppwrf halo: worker 0: 7 exchanges overlapped by launches (0.020 s of launching between)
cppwrf halo: worker 0:   count   total ms  pack us  wait us  xfer us unpack us  plan
cppwrf halo: worker 0:      40     400.00     10.0   8000.0   1900.0     100.0  u_2,v_2,ph_2 (overlapped)
cppwrf halo: worker 0:     260     850.00      5.0    100.0     30.0       2.0  p,al
cppwrf halo: worker 0: collective kind 1: 16 calls, 4.00 ms (250.0 us each)
cppwrf graphs: 245 launches, 10 updates, 49 instantiations, 10.5 nodes per launch
cppwrf: io out/wrfout_d01 rec 1: values 0.100 s (device copies 0.050 s), staging 0.010 s, queued 0.000 s, puts 0.200 s (pool wait 0.000 s, filter's own compression 0.000 s), sync 0.300 s, total 0.610 s; chunks precompressed 10, taken 10, compressed by the filter 0
cppwrf ktime: 1.500 s in kernels
 60.00%    900.00 ms     240  KSwGpoint<48>
 40.00%    600.00 ms    1200  KLwLayer
"""


class FakeRelease:
    """kaggle_proof.ReleaseData's interface for the tars of the default plan."""

    def __init__(self, drop=()):
        self.index = {}
        for case, c in perf.CASES.items():
            files = set(c.get("inputs", ("wrfinput_d01", "wrfbdy_d01"))) | {"namelist.input", "RRTMG_SW_DATA",
                                                                             "wrfout_d01_2026-10-12_00:00:00"}
            p = f"wrf_runs/{c['run']}/"
            self.index.setdefault(c["tar"], set()).update(p + f for f in files - set(drop))
            self.index[c["tar"]].add(p + "steps/wrfinput_d01")

    def tar(self, name):
        return {"name": name, "bytes": 10**9, "exact": True, "sha256": "0" * 64, "disk": 10**9,
                "assets": [[name, "1" * 64]], "members": [], "links": None}


class PlanTest(unittest.TestCase):
    def test_rounds_alternate_and_diagnostics_first(self):
        runs, comps = perf.plan_runs(["mpi", "single", "nest", "day", "io"], ["dp", "id"], 3)
        self.assertEqual(len({r["id"] for r in runs}), len(runs))
        self.assertEqual([r["order"] for r in runs], list(range(len(runs))))
        groups = {}
        for r in runs:
            groups.setdefault(r["group"], []).append(r)
        for g, rs in groups.items():
            kinds = [r["kind"] for r in rs]
            self.assertEqual(kinds, sorted(kinds, key=lambda k: k != "diag"), g)  # diagnostics first
            timed = [r for r in rs if r["kind"] == "timed"]
            rounds = sorted({r["rep"] for r in timed})
            first = [r["variant"] for r in timed if r["rep"] == rounds[0]]
            for k in rounds:
                order = [r["variant"] for r in timed if r["rep"] == k]
                self.assertEqual(order, first if k % 2 == 0 else first[::-1], g)
            for r in rs:  # timed runs never carry a synchronising or timing knob
                if r["kind"] == "timed":
                    self.assertFalse({"CPPWRF_KTIME", "CPPWRF_MEMINFO", "CPPWRF_HALO_STATS", "CPPWRF_IO_TIMING"}
                                     & set(r["env"]), r["id"])
                    self.assertNotEqual(r["env"]["CPPWRF_GRAPHS"], "stats")
        for c in comps:
            names = {r["variant"] for r in groups[c["group"]] if r["kind"] == "timed"}
            self.assertIn(c["base"], names)
            self.assertIn(c["new"], names)
        self.assertEqual(groups["mpi/P4/dp"][4]["variant"], "plain")  # after 4 diagnostic runs
        self.assertEqual(groups["nest/N3/dp"][-1]["rep"], 1)  # the nest: at most 2 rounds
        io_run = next(r for r in runs if r["group"] == "io/P4/dp" and r["kind"] == "timed")
        self.assertIn("output.format=wrfout", io_run["sets"])
        self.assertIn("output.out_interval=10", io_run["sets"])
        n3 = next(r for r in runs if r["case"] == "N3")
        self.assertTrue(n3["nested"])
        self.assertIn("d02.out_interval=1000000", n3["sets"])

    def test_default_session_fits_four_hours(self):
        runs, _ = perf.plan_runs(["mpi", "single", "nest", "day", "io"], ["dp", "id"], 3)
        tars = perf.tar_specs(FakeRelease(), {r["case"] for r in runs})
        est = perf.session_estimate(runs, ["dp", "id"], tars, 30)
        self.assertLess(est["total_s"], 4 * 3600)
        self.assertGreater(est["total_s"], 2 * 3600)

    def test_inputs_only(self):
        tars = perf.tar_specs(FakeRelease(), {"P4", "N3"})
        m = tars["suite_data.tar"]["members"]
        self.assertIn("wrf_runs/run_P4_r8/wrfinput_d01", m)
        self.assertFalse([x for x in m if "wrfout" in x or "/steps/" in x])
        self.assertIn("wrf_runs/run_N3h_r8/wrfinput_d02", tars["n3_data.tar"]["members"])
        with self.assertRaisesRegex(ValueError, "wrfinput_d02"):
            perf.tar_specs(FakeRelease(drop=("wrfinput_d02",)), {"N3"})


class CommandTest(unittest.TestCase):
    def spec(self, layout, **env):
        return {"layout": layout, "sets": ["real.dir=@RUNS@/run_P4_r8", "output.wrf_timing=true"],
                "env": dict(perf.DEFAULT_ENV, **env), "cpus": 4}

    def test_plain_and_mpi(self):
        argv, env = body.run_command(self.spec("plain"), "/b/cppwrf", "/s/configs/real_p4_r8.ini", "/w/r", "/d/wrf_runs")
        self.assertEqual(argv[:4], ["/b/cppwrf", "/s/configs/real_p4_r8.ini", "--set", "output.dir=/w/r"])
        self.assertIn("real.dir=/d/wrf_runs/run_P4_r8", argv)
        self.assertEqual((env["CUDA_VISIBLE_DEVICES"], env["OMP_NUM_THREADS"]), ("0", "4"))
        argv, env = body.run_command(self.spec("1x2", CPPWRF_HALO_OVERLAP="0"), "/b/cppwrf", "/c.ini", "/w/r", "/d",
                                     ["--mca", "btl", "self,tcp"])
        self.assertEqual(argv[:10], body.MPIRUN + ["--mca", "btl", "self,tcp", "-np", "2"])
        self.assertEqual(argv[-2:], ["--set", "run.ranks=1x2"])
        self.assertEqual((env["CUDA_VISIBLE_DEVICES"], env["OMP_NUM_THREADS"], env["CPPWRF_HALO_OVERLAP"]),
                         ("0,1", "2", "0"))

    def test_overlap_rule_and_deadline(self):
        self.assertTrue(body.overlap_rule({"env": {"CPPWRF_GRAPHS": "1"}}))
        self.assertFalse(body.overlap_rule({"nested": True, "env": {}}))
        self.assertFalse(body.overlap_rule({"env": {"CPPWRF_KTIME": "1"}}))
        self.assertTrue(body.fits(100, 0, 150))
        self.assertFalse(body.fits(100, 0, 149))


class ParseTest(unittest.TestCase):
    def test_parsers(self):
        p = body.parse_log(LOG, 6.0, True)
        t = p["timing"]
        self.assertEqual((t["steps"], t["steady_steps"]), (4, 4))
        self.assertAlmostEqual(t["step_s"], 0.75 / 4)
        self.assertAlmostEqual(t["io_pre"], 0.5)
        h = p["halo"][0]
        self.assertEqual((h["exchanges"], h["overlapped"]), (300, 7))
        self.assertAlmostEqual(h["xfer_s"], 0.3)
        self.assertEqual([x["plan"] for x in h["plans"]], ["p,al", "u_2,v_2,ph_2 (overlapped)"])
        self.assertEqual(h["collectives"], [{"kind": 1, "calls": 16, "ms": 4.0}])
        self.assertEqual(p["graphs"][0]["instantiations"], 49)
        self.assertEqual(p["ktime"]["rows"][0]["kernel"], "KSwGpoint<48>")
        self.assertAlmostEqual(p["ktime"]["total_s"], 1.5)
        self.assertEqual(p["meminfo"][0]["stack_bytes"], 12400)
        self.assertEqual(p["io"]["records"], 1)
        self.assertAlmostEqual(p["io"]["total_s"], 0.61)
        self.assertEqual(body.parse_log(LOG, 6.0, False)["timing"]["steady_steps"], 3)


def record(group, variant, rep, step, kind="timed", wall=10.0):
    return {"id": f"{group}-{variant}-{rep}", "group": group, "variant": variant, "rep": rep, "kind": kind, "exit": 0,
            "wall": wall, "measures": ["M1"], "telemetry": {"gpu0": {"samples": 10, "util_median_pct": 90,
                                                                     "sm_clock_median_mhz": 1590, "power_mean_w": 69,
                                                                     "throttled_samples": 0, "power_cap_samples": 3,
                                                                     "temp_max_c": 70}},
            "parsed": {"timing": {"step_s": step, "steady_steps": 60}, "halo": [], "graphs": [], "ktime": None,
                       "meminfo": [], "io": None}}


class ReportTest(unittest.TestCase):
    def job(self):
        return {"commit": "c" * 40, "id": "f" * 64, "runs": [1] * 13, "measures": perf.MEASURES,
                "comparisons": [{"measure": "M1", "group": "g", "base": "a", "new": "b", "metric": "step_s"},
                                {"measure": "M2", "group": "g", "base": "a", "new": "c", "metric": "step_s"},
                                {"measure": "M2", "group": "g", "base": "a", "new": "d", "metric": "wall"}]}

    def records(self):
        recs = [record("g", "a", r, 1.0 + 0.01 * r) for r in range(3)]
        recs += [record("g", "b", r, 0.8 + 0.01 * r) for r in range(3)]
        recs += [record("g", "c", r, s) for r, s in enumerate((0.95, 1.05, 1.0))]
        recs += [record("g", "x", None, 2.0, kind="diag")]
        recs += [{"id": "s", "kind": "skipped", "group": "g", "variant": "d", "rep": 0}]
        return recs

    def test_aggregate(self):
        agg = body.aggregate(self.job(), self.records())
        c = {x["new"]: x for x in agg["comparisons"]}
        self.assertEqual(c["b"]["verdict"], "faster")
        self.assertAlmostEqual(c["b"]["ratio"]["median"], 1.01 / 0.81)
        self.assertEqual(c["b"]["rounds"], 3)
        self.assertEqual(c["c"]["verdict"], "within noise")
        self.assertEqual(c["d"]["verdict"], "no data")
        self.assertEqual(agg["counts"], {"planned": 13, "recorded": 10, "ok": 10, "failed": 0, "skipped": 1})
        row = next(r for r in agg["rows"] if r["variant"] == "a")
        self.assertAlmostEqual(row["step_s"]["median"], 1.01)
        self.assertEqual(row["power_cap_samples"], 9)
        md = body.markdown(agg, self.job(), "topo: PHB")
        self.assertIn("| M1 | g | a -> b | step_s |", md)
        self.assertIn("## M8 probe", md)

    def test_report_cli_and_collector(self):
        with tempfile.TemporaryDirectory() as t:
            d = Path(t)
            job = self.job()
            (d / "perf-job.json").write_text(json.dumps(job))
            (d / "records.jsonl").write_text("".join(json.dumps(r) + "\n" for r in self.records()) + "not json\n")
            r = subprocess.run([sys.executable, "-B", str(HERE / "kg_perf_body.py"), "report", str(d)],
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("faster", (d / "results.md").read_text())
            self.assertEqual(json.loads((d / "results.json").read_text())["counts"]["ok"], 10)
            # the collector's side: the kernel's archive, the receipt checked against the plan
            out = d / "out"
            out.mkdir()
            with tarfile.open(out / perf.RESULTS, "w:gz") as a:
                for name, data in (("done.json", {"id": job["id"], "shard": 1, "exit": 0}),):
                    b = json.dumps(data).encode()
                    ti = tarfile.TarInfo(name)
                    ti.size = len(b)
                    a.addfile(ti, io.BytesIO(b))
                a.add(d / "records.jsonl", arcname="records.jsonl")
            self.assertEqual(perf.report(out, job), 0)
            self.assertTrue((out / "results" / "results.md").is_file())
            with self.assertRaisesRegex(ValueError, "not this plan"):
                perf.report(out, dict(job, id="0" * 64))


FAKE = r"""#!/usr/bin/env python3
import os, sys
args = sys.argv[1:]
assert args[1] == "--set" and args[2].startswith("output.dir="), args
print("env", os.environ.get("CPPWRF_GRAPHS"), os.environ.get("CUDA_VISIBLE_DEVICES"), os.environ.get("OMP_NUM_THREADS"))
print(open(os.environ["FAKE_LOG"]).read())
"""


class RunLoopTest(unittest.TestCase):
    def test_one_run_through_the_worker(self):
        with tempfile.TemporaryDirectory() as t:
            t = Path(t)
            fake = t / "cppwrf"
            fake.write_text(FAKE)
            fake.chmod(0o755)
            (t / "log.txt").write_text(LOG)
            src = t / "src"
            (src / "configs").mkdir(parents=True)
            (src / "configs/real_p4_r8.ini").write_text("[run]\n")
            out = t / "out"
            (out / "logs").mkdir(parents=True)
            runs, _ = perf.plan_runs(["single"], ["dp"], 1)
            spec = next(r for r in runs if r["case"] == "P4" and r["kind"] == "timed")
            ctx = {"job": {"pause": 0, "cool_delta": 5, "cool_max": 0}, "work": t, "src": src, "bins": {"dp": fake},
                   "out": out, "mca": [], "runs_dir": t / "data", "fields": [], "idle": [],
                   "env": dict(os.environ, FAKE_LOG=str(t / "log.txt"), PATH="/usr/bin:/bin")}
            rec = body.run_one(spec, ctx)
            self.assertEqual(rec["exit"], 0)
            self.assertEqual(rec["parsed"]["timing"]["steps"], 4)
            self.assertAlmostEqual(rec["parsed"]["timing"]["step_s"], 0.1875)
            text = (out / "logs" / f"{spec['id']}.log").read_text()
            self.assertIn("env 1 0 4", text)
            self.assertIn("real.dir=" + str(t / "data") + "/run_P4_r8", text)
            self.assertFalse((t / "runs" / spec["id"]).exists())  # the run directory is removed


SASS = """
\t\tFunction : _Z9KSwGpointILi48EEvv
        /*0010*/                   LDG.E.U8.CONSTANT R2, [R4.64] ;
        /*0020*/                   LDG.E.64 R6, [R8.64] ;
        /*0030*/                   LDC.U8 R0, c[0x3][R2] ;
\t\tFunction : _Z8KAdvectvv
        /*0010*/                   LDG.E.U8 R2, [R4.64] ;
"""


class SassTest(unittest.TestCase):
    def test_byte_loads_of_rrtmg_kernels(self):
        with tempfile.TemporaryDirectory() as t:
            t = Path(t)
            (t / "sass.txt").write_text(SASS)
            tool = t / "cuobjdump"
            tool.write_text(f"#!/bin/sh\ncat {t / 'sass.txt'}\n")
            tool.chmod(0o755)
            path = os.environ["PATH"]
            os.environ["PATH"] = f"{t}:{path}"
            try:
                s = body.sass_byte_loads(t / "cppwrf", t / "out.txt")
            finally:
                os.environ["PATH"] = path
            self.assertEqual(s, {"kernels": 2, "rrtmg_kernels": 1, "rrtmg_loads": 3, "rrtmg_byte_loads": 2})
            self.assertIn("KSwGpoint", (t / "out.txt").read_text())


class DryRunTest(unittest.TestCase):
    def test_dry_run_stages_a_verifiable_bundle(self):
        with tempfile.TemporaryDirectory() as t:
            t = Path(t)
            meta = t / "meta"
            meta.mkdir()
            rel = FakeRelease()
            sums, tars, index = [], [], []
            for name, paths in rel.index.items():
                sums.append(f"{'a' * 64}  {name}")
                tars.append(f"{name} 1000000000 {'b' * 64} 1")
                index += [f"{name}\t0\t{p}\t" for p in sorted(paths)]
            (meta / "SHA256SUMS").write_text("\n".join(sums) + "\n")
            (meta / "TARS.txt").write_text("\n".join(tars) + "\n")
            with gzip.open(meta / "INDEX.tsv.gz", "wt") as f:
                f.write("\n".join(index) + "\n")
            out = t / "out"
            env = {k: v for k, v in os.environ.items() if k != "KAGGLE_USERNAME"}
            r = subprocess.run([sys.executable, "-B", str(HERE / "kaggle_perf.py"), "--out", str(out),
                                "--metadata-dir", str(meta), "--dry-run", "--owner", "tester"],
                               capture_output=True, text=True, env=env)
            self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
            self.assertIn("Estimated session", r.stdout)
            job = json.loads((out / "dataset" / perf.JOB).read_text())
            self.assertEqual(job, json.loads((out / "plan.json").read_text()))
            for name, digest in job["bundle"].items():
                self.assertEqual(hashlib.sha256((out / "dataset" / name).read_bytes()).hexdigest(), digest, name)
            kernel = json.loads((out / "kernel" / "kernel-metadata.json").read_text())
            self.assertEqual(kernel["machine_shape"], "NvidiaTeslaT4")
            self.assertTrue(kernel["is_private"])
            self.assertEqual(kernel["dataset_sources"], [f"tester/cppwrf-perf-{job['commit'][:10]}-{job['id'][:10]}-src"])
            run_py = (out / "kernel" / "run.py").read_text()
            self.assertIn(job["id"], run_py)
            self.assertIn("kgperf-results.tgz", run_py)
            py_compile.compile(str(out / "kernel" / "run.py"), doraise=True)
            self.assertEqual(set(job["builds"]), {"dp", "id"})
            self.assertTrue(all("@RUNS@" in r["sets"][0] for r in job["runs"]))
            # --hours below the estimate refuses the plan
            r = subprocess.run([sys.executable, "-B", str(HERE / "kaggle_perf.py"), "--out", str(t / "o2"),
                                "--metadata-dir", str(meta), "--plan", "--hours", "1"], capture_output=True, text=True)
            self.assertEqual(r.returncode, 2)
            self.assertIn("exceeds --hours", r.stderr)


if __name__ == "__main__":
    unittest.main()
