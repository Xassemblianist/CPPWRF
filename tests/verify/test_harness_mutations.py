# SPDX-License-Identifier: GPL-3.0-or-later
"""Harness regression tests using tiny fixtures and stub model runs.

Run with python3 -B tests/verify/test_harness_mutations.py -v. Fixtures live under
~/cppwrf_scratch/cdx9_tmp.
"""
from contextlib import redirect_stdout, redirect_stderr
from datetime import datetime
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from netCDF4 import Dataset
import numpy as np

import case_data
import wrf_suite
from test_exact_mutations import SCRATCH

ROOT = Path(__file__).resolve().parents[2]
START = "2026-10-12_00:00:00"
ALARM_51 = "WRF_ALARM_SECS_TIL_NEXT_RING_51"
ALARM_55 = "WRF_ALARM_SECS_TIL_NEXT_RING_55"
RESUME_NAMELIST = """
&time_control history_interval=1, restart=.true., restart_interval=1000, /
&domains time_step=60, dx=15000, dy=15000, /
&physics mp_physics=0, cu_physics=0, ra_lw_physics=0, ra_sw_physics=0,
         bl_pbl_physics=0, sf_sfclay_physics=0, sf_surface_physics=0, /
&dynamics diff_opt=0, km_opt=1, /
&bdy_control specified=.true., /
"""


def fixture(directory, seconds, *, kind="wrfout", domain="d01", start=START, alarm=180, clock_delta=0):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{kind}_{domain}_{wrf_suite.times_at(START, seconds[0])}"
    with Dataset(path, "w") as ds:
        ds.createDimension("Time", None)
        ds.createDimension("DateStrLen", 19)
        ds.createDimension("x", 2)
        ds.START_DATE = start
        ds.SIMULATION_START_DATE = START
        if kind == "wrfrst":
            ds.setncattr(ALARM_51, alarm)
            offset = (datetime.fromisoformat(start) - datetime.fromisoformat(START)).total_seconds()
            ds.setncattr(ALARM_55, int(offset) - seconds[0] + clock_delta)
        times = ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
        for i, sec in enumerate(seconds):
            times[i] = np.frombuffer(wrf_suite.times_at(START, sec).encode(), dtype="S1")
        ds.createVariable("XTIME", "f8", ("Time",))[:] = np.array(seconds) / 60
        ds.createVariable("T", "f8", ("Time", "x"))[:] = [[sec, sec + 1] for sec in seconds]
    return path


def snapshot(directory, *, step=3, time=180):
    meta = {"step": step, "time": time, "variables": [
        {"name": "u", "stagger": "M", "shape": [1, 1, 1], "offset": 0, "nbytes": 4, "dtype": "float32"}
    ]}
    (directory / "snap_000003.json").write_text(json.dumps(meta))
    (directory / "snap_000003.bin").write_bytes(np.float32(1).tobytes())


class ScratchMutationCase(unittest.TestCase):
    def setUp(self):
        SCRATCH.mkdir(parents=True, exist_ok=True)
        tmp = tempfile.TemporaryDirectory(prefix="harness-mutations-", dir=SCRATCH)
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.addCleanup(patch.stopall)
        patch.dict(os.environ, {"TMPDIR": str(self.root), "CPPWRF_SCRATCH": str(self.root),
                                "PYTHONDONTWRITEBYTECODE": "1"}).start()
        patch.object(tempfile, "tempdir", str(self.root)).start()

class HarnessMutations(ScratchMutationCase):
    def test_seeded_restart_requires_history_through_end_and_restart(self):
        cfg, inputs = "configs/real_p1_r8.ini", self.root / "run_P1_r8"
        inputs.mkdir()
        (inputs / "wrfinput_d01").touch()
        ref = fixture(self.root / "rst_P1_r8", (0, 60, 120, 180))
        ref.with_name("namelist.input").write_text(RESUME_NAMELIST)
        fixture(ref.parent, (60,), kind="wrfrst", alarm=60)
        rsm = self.root / "rsm_P1_r8"
        ref_history = fixture(rsm, (120, 180), start=wrf_suite.times_at(START, 60))
        ref_restart = fixture(rsm, (120,), kind="wrfrst", start=wrf_suite.times_at(START, 60))
        (rsm / "namelist.input").write_text(RESUME_NAMELIST)

        def run_stub(_binary, _cfg, _sets, out):
            out = Path(out)
            out.mkdir(parents=True, exist_ok=True)
            if out.name in ("straight", "restarted"):
                snapshot(out)
                (out / "restart_000001.nc").touch()
            elif out.name == "resumed_wrfout":
                fixture(out, (120, 180))
            else:
                fixture(out, (120, 180), start=wrf_suite.times_at(START, 60))
                fixture(out, (120,), kind="wrfrst", start=wrf_suite.times_at(START, 60))
            return 0, ""

        def check():
            return wrf_suite.restart_check("stub", self.root / "restart", "p1", cfg, "run_P1_r8")

        with patch.object(wrf_suite, "WR", str(self.root)), patch.object(wrf_suite, "run", run_stub), \
                patch.object(wrf_suite, "config_vs_namelist", return_value=[]):
            ok, msg = check()
            self.assertTrue(ok, msg)
            for mutation, diagnostic in (("truncated history", wrf_suite.times_at(START, 180)),
                                         ("missing restart", "missing WRF restart reference"),
                                         ("no files", "no common frame")):
                with self.subTest(mutation=mutation):
                    fixture(rsm, (120,) if mutation == "truncated history" else (120, 180),
                            start=wrf_suite.times_at(START, 60))
                    fixture(rsm, (120,), kind="wrfrst", start=wrf_suite.times_at(START, 60))
                    if mutation != "truncated history":
                        ref_restart.unlink()
                    if mutation == "no files":
                        ref_history.unlink()
                    ok, msg = check()
                    self.assertFalse(ok, msg)
                    self.assertIn(diagnostic, msg)

    def run_chain(self, name, *, omit_final=False, omit_domain=False, omit_wrf_final=False,
                  extra_resumed=False, bad_alarm=False):
        case = next(c for c in wrf_suite.CASES if c[0] == name)
        _, _, cfg, all_sets, refs, end, _ = case
        mid, interval, native, _ = wrf_suite.chain_at(all_sets)
        sets = [s for s in all_sets if not s.startswith("suite.")]
        nested = isinstance(refs, dict)
        domains = {"d01": 300, "d02": 900} if nested else {"d01": 180 if "d0" in name else 1800}
        cadence = {d: tuple(range(0, end + 1, step)) for d, step in domains.items()}
        wrf_times = tuple(range(0, end + 1, 60)) if "d0" in name else cadence["d01"]
        if native:  # The WRF reference can contain extra frames between CPPWRF outputs.
            wrf_times = (*wrf_times[:-1], int(mid) + 60, wrf_times[-1])
        refdir = self.root / name / "ref"
        ref = fixture(refdir, wrf_times[:-1] if omit_wrf_final else wrf_times)
        if nested:
            refs = {"d01": str(ref), "d02": str(fixture(refdir, cadence["d02"], domain="d02"))}
        else:
            refs = {"": str(ref)}
        (refdir / "namelist.input").touch()

        def run_stub(_binary, _cfg, _sets, out):
            out = Path(out)
            straight = out.name == "a_straight"
            run_start = wrf_suite.times_at(START, mid) if out.name == "b_resumed" else START
            for d, times in cadence.items():
                if omit_domain and d == "d02":
                    continue
                origin = int(mid) if out.name == "b_resumed" else 0
                hist_iv = domains[d] - origin % domains[d] if origin else domains[d]
                history = tuple(range(origin + hist_iv if origin else 0, end + 1, hist_iv))
                if omit_final:
                    history = history[:-1]
                if not straight:
                    history = tuple(t for t in history if t > mid)
                    if extra_resumed:
                        history += (int(mid) + 60,)
                if history:
                    fixture(out, history, domain=d, start=run_start)
                for sec in range(int(interval if straight else mid + interval), end + 1, int(interval)):
                    path = fixture(out, (sec,), kind="wrfrst", domain=d, start=run_start,
                                   alarm=int(interval) - int(bad_alarm and not straight),
                                   clock_delta=int(bad_alarm and not straight))
                    with Dataset(path, "a") as ds:
                        since = (sec - origin) % hist_iv
                        ds.setncattr(wrf_suite.HISTORY_ALARM, hist_iv - since)
                        ds.setncattr(wrf_suite.HISTORY_RINGING, int(since == 0 and (d != "d01" or sec == end)))
            if straight and not nested:
                dt = float(wrf_suite.configuration(cfg, sets)["time"]["dt"])
                (out / f"restart_{round(mid / dt):06d}.nc").touch()
            return 0, ""

        with patch.object(wrf_suite, "run", run_stub), patch.object(wrf_suite, "config_vs_namelist", return_value=[]), \
                patch.object(wrf_suite, "chain_first_step", return_value=[]), redirect_stdout(io.StringIO()):
            return wrf_suite.restart_chain("stub", cfg, sets, refs, end, mid, interval,
                                           str(self.root / name / "out"), native, str(ref))

    def test_d0_and_sr1_cannot_agree_to_drop_final_frame(self):
        for name, stamp in (("real_d0_r8_fp64_cpu_chain", "00:18:00"), ("real_sr1_r8_fp64_cpu_chain", "04:30:00")):
            with self.subTest(name=name):
                self.assertFalse(self.run_chain(name)[0])
                shutil.rmtree(self.root / name)
                bad, _ = self.run_chain(name, omit_final=True)
                self.assertIn(stamp, "\n".join(bad))
                self.assertIn("CPPWRF missing", "\n".join(bad))

    def test_chain_cannot_infer_final_time_from_truncated_wrf(self):
        bad, _ = self.run_chain("real_sr1_r8_fp64_cpu_chain", omit_final=True, omit_wrf_final=True)
        self.assertIn("WRF missing ['2026-10-12_04:30:00']", "\n".join(bad))

    def test_nested_chain_requires_missing_domain(self):
        name = "real_n3_r8_fp64_cpu_chain"
        self.assertFalse(self.run_chain(name)[0])
        shutil.rmtree(self.root / name)
        bad, _ = self.run_chain(name, omit_domain=True)
        self.assertIn("missing straight restart wrfrst_d02_", "\n".join(bad))

    def test_chain_checks_extra_resumed_timestamp_and_restart_alarm(self):
        name = "real_d0_r8_fp64_cpu_chain"
        bad, _ = self.run_chain(name, extra_resumed=True, bad_alarm=True)
        self.assertIn("only-cppwrf", "\n".join(bad))
        self.assertIn(ALARM_51, "\n".join(bad))
        self.assertIn(ALARM_55, "\n".join(bad))

    def test_auxhist3_and_restart_streams_are_required_at_runtime(self):
        refdir, out = self.root / "refs", self.root / "out"
        ref = fixture(refdir, (0, 180))
        cfg = self.root / "run.ini"
        cfg.write_text("[time]\ndt=60\n[run]\nrun_time=180\n[restart]\ninterval=120\n"
                       "[output]\nauxhist3=true\n[phys]\nauxhist3_interval=3\n")
        fixture(out, (0, 180))
        aux = fixture(refdir, (0, 180), kind="auxhist3")
        fixture(out, (0, 180), kind="auxhist3")
        rst = fixture(refdir, (120,), kind="wrfrst")
        cpp_rst = fixture(out, (120,), kind="wrfrst")
        refs = {"": str(ref)}
        self.assertFalse(wrf_suite.exact_case(refs, str(out), 180, ["output.auxhist3=true"])[0])
        self.assertFalse(wrf_suite.wrfrst_case(refs, str(out), cfg, [])[0])
        aux.unlink()
        rst.unlink()
        cpp_rst.unlink()
        self.assertIn("auxhist3: no WRF reference", "\n".join(
            wrf_suite.exact_case(refs, str(out), 180, ["output.auxhist3=true"])[0]))
        self.assertIn("auxhist3: no WRF reference", "\n".join(
            wrf_suite.exact_case(refs, str(out), 180, [], cfg=cfg)[0]))
        self.assertIn("missing WRF reference restart", "\n".join(wrf_suite.wrfrst_case(refs, str(out), cfg, [])[0]))

    def test_wrf_leg_rejects_cppwrf_reference_even_without_namelist(self):
        ref = fixture(self.root / "ref", (0, 180))
        out = self.root / "out"
        fixture(out, (0, 180))
        self.assertFalse(wrf_suite.exact_case({"": str(ref)}, str(out), 180, [])[0])
        with Dataset(ref, "r+") as ds:
            ds.TITLE = " OUTPUT FROM CPPWRF V4.8.0 MODEL"
        bad, _ = wrf_suite.exact_case({"": str(ref)}, str(out), 180, [])
        self.assertIn("CPPWRF TITLE in WRF reference", "\n".join(bad))

    def test_archive_manifest_requires_opted_in_streams(self):
        expected = {"real_g9f2_r8_fp64_cpu": "auxhist3_d01_*", "real_rsa_r8_fp64_cpu": "wrfrst_d01_*",
                    "restart_p1_fp64": "rsm_P1_r8/wrfout_d01_*"}
        for name, pattern in expected.items():
            with self.subTest(name=name):
                needed, _ = case_data.needs(name)
                self.assertTrue(any(p.endswith(pattern) and req for p, req in needed.items()))
                _, problems = case_data.derive(name, {"test.tar": {}}, ["test.tar"])
                self.assertTrue(any(pattern in p for p in problems), problems)

    def load_stage(self):
        with patch.dict(os.environ, CPPWRF_WRF_RUNS=str(self.root)):
            spec = importlib.util.spec_from_file_location("stage_data", ROOT / "tools/colab/stage_data.py")
            stage = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(stage)
        return stage

    def test_staging_preserves_whole_shared_restart_reference(self):
        case, tag = "real_a01_r8_fp64_cpu", "restart_a01_fp64"
        _, _, cfg, sets, ref, t, tol = next(c for c in wrf_suite.CASES if c[0] == case)
        rel = Path(ref).relative_to(wrf_suite.WR)
        source = fixture(self.root / rel.parent, (0, 60, 120, 180))
        self.assertEqual(source.name, rel.name)
        stage = self.load_stage()
        arcname = str(Path("wrf_runs") / rel)
        for names in (case, f"{case},{tag}"):
            archive = self.root / (names + ".tar")
            args = ["stage_data.py", "--refs-only", "--only", names, str(archive)]
            with patch.object(wrf_suite, "CASES", [(case, "cpu_dp", cfg, sets, str(source), t, tol)]), \
                    patch.object(sys, "argv", args), redirect_stdout(io.StringIO()):
                stage.main()
            with tarfile.open(archive) as tf:
                self.assertEqual(tf.getnames().count(arcname), 1)
                payload = tf.extractfile(arcname).read()
            if names == case:
                self.assertNotEqual(payload, source.read_bytes())
            else:
                self.assertEqual(payload, source.read_bytes())

    def test_stage_requires_opted_in_auxiliary_stream(self):
        stage = self.load_stage()
        ref = fixture(self.root / "case", (0, 180))
        cfg = self.root / "case.ini"
        cfg.write_text(f"[real]\ndir={ref.parent}\n[output]\nauxhist3=true\n")
        case = ("aux", "cpu_dp", str(cfg), [], str(ref), 180, {})
        args = ["stage_data.py", "--only", "aux", "--refs-only", str(self.root / "out.tar")]
        with patch.object(wrf_suite, "CASES", [case]), patch.object(sys, "argv", args), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(SystemExit, "missing reference stream.*auxhist3"):
                stage.main()

    def test_stage_rejects_missing_seeded_restart_history(self):
        stage = self.load_stage()
        rst, rsm = self.root / "rst_P1_r8", self.root / "rsm_P1_r8"
        fixture(rst, (0, 60, 120, 180))
        fixture(rst, (60,), kind="wrfrst")
        fixture(rsm, (120,), kind="wrfrst")
        for d in (rst, rsm):
            (d / "namelist.input").touch()
        args = ["stage_data.py", "--only", "restart_p1_fp64", "--refs-only", str(self.root / "out.tar")]
        with patch.object(sys, "argv", args), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(SystemExit, "missing restart reference.*rsm_P1_r8/wrfout"):
                stage.main()

    def test_resumed_namelist_uses_last_assignment_and_copies_nested_fallback(self):
        rst, rsm = self.root / "rst", self.root / "rsm"
        rst.mkdir()
        rsm.mkdir()
        (rst / "old.txt").write_text("stale")
        (rsm / "new.txt").write_text("current")
        nl = rsm / "namelist.input"
        nl.write_text("&time_control\n iofields_filename='old.txt',\n iofields_filename='new.txt',\n /\n")
        path, why = wrf_suite.resumed_namelist(rsm, rst, self.root)
        self.assertEqual(path, str(nl), why)
        (rst / "fields").mkdir()
        (rst / "fields" / "current.txt").write_text("fallback")
        nl.write_text(nl.read_text().replace("'new.txt'", "'fields/current.txt', 'new.txt'"))
        path, why = wrf_suite.resumed_namelist(rsm, rst, self.root)
        self.assertFalse(why)
        self.assertEqual((Path(path).parent / "fields" / "current.txt").read_text(), "fallback")
        self.assertEqual((Path(path).parent / "new.txt").read_text(), "current")
        self.assertFalse((Path(path).parent / "old.txt").exists())

    def test_restart_interval_rejects_unsupported_seconds_and_handles_zero(self):
        nl = self.root / "namelist.input"
        nl.write_text(RESUME_NAMELIST)
        self.assertEqual(wrf_suite.namelist_restart_interval(str(nl)), 60000)
        with self.subTest(interval="seconds"):
            nl.write_text(RESUME_NAMELIST.replace("1000,", "1000, restart_interval_s=30,"))
            with self.assertRaisesRegex(ValueError, "restart_interval_s"):
                wrf_suite.namelist_restart_interval(str(nl))
        with self.subTest(interval="zero"):
            nl.write_text(RESUME_NAMELIST.replace("1000,", "0,"))
            self.assertEqual(wrf_suite.namelist_restart_interval(str(nl)), 0)

    def test_t1_preflight_uses_t1_input(self):
        tag, cfg, run_dir = next(r for r in wrf_suite.RESTARTS if r[0] == "t1")
        ref = fixture(self.root / "run_T1_r8", (0, 60, 120, 180, 240, 300))
        ref.with_name("wrfinput_d01").touch()
        ref.with_name("namelist.input").touch()
        case = next(c for c in wrf_suite.CASES if c[0] == "real_t1_fp64_cpu")
        with patch.object(wrf_suite, "CASES", [(*case[:4], str(ref), *case[5:])]), \
                patch.object(wrf_suite, "WR", str(self.root)), \
                patch.object(wrf_suite, "config_vs_namelist", return_value=[]), \
                patch.object(wrf_suite, "run", return_value=(1, "reached T1 launch")) as run:
            ok, msg = wrf_suite.restart_check("stub", self.root / "out", tag, cfg, run_dir)
        run.assert_called_once()
        self.assertIs(ok, False)
        self.assertIn("reached T1 launch", msg)

    def test_target_for_unavailable_backend_fails_before_running(self):
        args = ["wrf_suite.py", "--cpu-dp", "/bin/true", "--only", "restart_p1_fp64_cuda",
                "--exact", "--allow-skip", "--out", str(self.root)]
        err = io.StringIO()
        with patch.object(sys, "argv", args), patch.object(wrf_suite, "run") as run, \
                redirect_stdout(io.StringIO()), redirect_stderr(err), self.assertRaises(SystemExit) as rc:
            wrf_suite.main()
        run.assert_not_called()
        self.assertNotEqual(rc.exception.code, 0)
        self.assertIn("selected no checks", err.getvalue())

    def test_reporter_prints_zero_counts_and_deduplicates_passes(self):
        for name in ("unit.log", "selftest.log"):
            (self.root / name).write_text("exit 0\n")
        for name in ("files.log", "idealfiles.log"):
            (self.root / name).write_text("PASS files_case\nexit 0\n")
        (self.root / "setup.out").touch()
        (self.root / "suite.log").write_text(
            "PASS case some text [vs WRF tol 0]  (2 s)\n"
            "PASS case repeated [vs WRF tol 0]  (3 s)\n"
            "PASS legacy untagged\nSUITE PASS 2 FAIL 0 SKIP 0\nexit 0\n")
        result = subprocess.run(["bash", ROOT / "tools/gcp/suite_status.sh", self.root, "case,legacy", "1"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PASS 2\nFAIL 0\nSKIP 0", result.stdout)
        self.assertIn("SPLIT PASS vs WRF at tolerance 0: 1; vs WRF by relative L2: 0; "
                      "CPPWRF self-consistency only: 0; untagged: 1", result.stdout)

    def test_reporter_rejects_failed_result_in_merged_log(self):
        for name in ("unit.log", "selftest.log"):
            (self.root / name).write_text("exit 0\n")
        (self.root / "setup.out").touch()
        (self.root / "suite.log").write_text(
            "PASS one [vs WRF tol 0]  (1 s)\nPASS two [vs WRF rel L2]  (1 s)\n"
            "PASS three [self only]  (1 s)\nSUITE PASS 3 FAIL 0 SKIP 0\nexit 0\n"
            "FAIL four [vs WRF tol 0]  (1 s)\nSUITE PASS 0 FAIL 1 SKIP 0\nexit 1\n")
        result = subprocess.run(["bash", ROOT / "tools/gcp/suite_status.sh", self.root, "case"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)

    def test_case_data_problems_exit_nonzero_without_manifest_changes(self):
        manifest = self.root / "archive.tar.txt"
        manifest.write_text("-rw-r--r-- user/group 1 2026-10-12 00:00 wrf_runs/input\n")
        with patch.object(wrf_suite, "CASES", [("demo",)]), patch.object(wrf_suite, "SPECIAL", []), \
                patch.object(case_data, "DATA", {"demo": "archive.tar"}), \
                patch.object(case_data, "order", return_value=["archive.tar"]), \
                patch.object(case_data, "needs", return_value=({"wrf_runs/input": True, "wrf_runs/ref": True}, [])), \
                patch.object(sys, "argv", ["case_data.py", str(self.root)]):
            for expected in ("wrf_runs/ref: in no tar", "no list of archive.tar"):
                with self.subTest(expected=expected):
                    out = io.StringIO()
                    with redirect_stdout(out), self.assertRaises(SystemExit) as rc:
                        case_data.main()
                    self.assertEqual(rc.exception.code, 1)
                    self.assertIn("PROBLEM demo: " + expected, out.getvalue())
                    manifest.unlink(missing_ok=True)


class ComparatorMutations(ScratchMutationCase):
    # Use the same scratch setup, with model-free entry points for the two real comparators.
    def l2_fixture(self, delta, *, w_levels=2):
        wrf, out = self.root / "reference.nc", self.root / "cpp"
        out.mkdir(exist_ok=True)
        with Dataset(wrf, "w") as ds:
            for name, length in (("Time", 1), ("z", 2), ("y", 13), ("x", 13)):
                ds.createDimension(name, length)
            ds.createVariable("XTIME", "f8", ("Time",))[:] = [1]
            for name in ("U", "V", "W", "PH", "PHB", "MU", "MUB", "P", "PB"):
                dims = ("Time", "y", "x") if name in ("MU", "MUB") else ("Time", "z", "y", "x")
                ds.createVariable(name, "f8", dims)[:] = 0 if name in ("PHB", "MUB", "PB") else 1
        fields = []
        with (out / "snap_000001.bin").open("wb") as data:
            for name in ("u", "v", "w", "ph", "mu", "p"):
                shape = (1 if name == "mu" else w_levels if name == "w" else 2, 13, 13)
                fields.append({"name": name, "shape": shape, "dtype": "float64", "offset": data.tell()})
                data.write(np.full(shape, 1 + delta if name == "w" else 1, dtype="<f8").tobytes())
        (out / "snap_000001.json").write_text(json.dumps({"time": 60, "step": 1, "variables": fields}))
        return wrf, out

    def test_l2_gate_uses_unrounded_value(self):
        self.assertLess(wrf_suite.compare(*self.l2_fixture(0.00999), 60)["W"], 0.01)
        self.assertGreater(wrf_suite.compare(*self.l2_fixture(0.01004), 60)["W"], 0.01)

    def test_l2_rejects_broadcasting_and_partial_failed_table(self):
        with self.subTest(mutation="broadcastable shape"):
            with self.assertRaisesRegex(RuntimeError, "W: WRF shape.*CPPWRF shape"):
                wrf_suite.compare(*self.l2_fixture(0, w_levels=1), 60)
        with self.subTest(mutation="failed comparator with a partial table"):
            failed = subprocess.CompletedProcess([], 1, "60 U 0 0 0 1\n", "truncated output")
            with patch.object(wrf_suite.subprocess, "run", return_value=failed), \
                    self.assertRaisesRegex(RuntimeError, "truncated output"):
                wrf_suite.compare("reference", "candidate", 60)

    def test_l2_y_slab_needs_the_flag_and_identical_wrf_rows(self):
        wrf, out = self.root / "slab.nc", self.root / "slab"
        out.mkdir(exist_ok=True)
        with Dataset(wrf, "w") as ds:
            for name, length in (("Time", 1), ("z", 2), ("y", 2), ("ys", 3), ("x", 13)):
                ds.createDimension(name, length)
            ds.createVariable("XTIME", "f8", ("Time",))[:] = [1]
            for name in ("U", "V", "W", "PH", "PHB", "MU", "MUB", "P", "PB"):
                dims = ("Time", "y", "x") if name in ("MU", "MUB") else \
                    ("Time", "z", "ys" if name == "V" else "y", "x")
                ds.createVariable(name, "f8", dims)[:] = 0 if name in ("PHB", "MUB", "PB") else 1
        fields = []
        with (out / "snap_000001.bin").open("wb") as data:
            for name in ("u", "v", "w", "ph", "mu", "p"):
                shape = (1 if name == "mu" else 2, 1, 13)
                fields.append({"name": name, "shape": shape, "dtype": "float64", "offset": data.tell()})
                data.write(np.ones(shape, dtype="<f8").tobytes())
        (out / "snap_000001.json").write_text(json.dumps({"time": 60, "step": 1, "variables": fields}))
        cmd = [sys.executable, ROOT / "tests/verify/compare_real.py", wrf, out]
        self.assertEqual(subprocess.run(cmd + ["--y-slab"], capture_output=True, text=True).returncode, 0)
        with self.subTest(mutation="no --y-slab"):
            r = subprocess.run(cmd, capture_output=True, text=True)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("WRF shape", r.stderr)
        with self.subTest(mutation="WRF rows differ"):
            with Dataset(wrf, "r+") as ds:
                ds["W"][0, 1, 1, 5] = 2
            r = subprocess.run(cmd + ["--y-slab"], capture_output=True, text=True)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("rows differ", r.stderr)

    def test_real_comparator_preserves_int64(self):
        wrf, cpp = self.root / "wrf", self.root / "cpp"
        cmd = [sys.executable, ROOT / "tests/init_real/compare_real.py", cpp, wrf]
        for path in (wrf, cpp):
            with Dataset(path, "w") as ds:
                ds.createDimension("Time", 1)
                ds.createVariable("I64", "i8", ())[:] = 2**53
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        with Dataset(cpp, "r+") as ds:
            ds["I64"][:] = 2**53 + 1
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("integers differ", result.stdout)

    def test_real_driver_requires_comparisons_and_complete_packaged_inventory(self):
        build = self.root / "build"
        exe = build / "src/init_real/cppwrf_real"
        exe.parent.mkdir(parents=True)
        exe.write_text("""#!/usr/bin/env bash
while [[ "$1" != --out-dir ]]; do shift; done
if [[ -n "$FAKE_GENERATED" ]]; then cp "$FAKE_GENERATED"/wrf* "$2"; fi
""")
        exe.chmod(0o755)
        runs = self.root / "runs"
        ref = runs / "real/run_P1"
        ref.mkdir(parents=True)
        (ref / "namelist.input").write_text("&domains max_dom=1, /\n&time_control interval_seconds=3600, /")
        cmd = ["bash", ROOT / "tests/init_real/run_real_test.sh", build]
        env = dict(os.environ, CPPWRF_WRF_RUNS=str(runs), FAKE_GENERATED="")
        for args, diagnostic in (([], "Missing reference case"), ([ref], "missing WRF reference wrfinput_d01")):
            with self.subTest(diagnostic=diagnostic):
                result = subprocess.run([*cmd, *args], env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn(diagnostic, result.stdout)

        generated = self.root / "generated"
        generated.mkdir()
        for name in ("wrfinput_d01", "wrfbdy_d01"):
            with Dataset(ref / name, "w") as ds:
                ds.createDimension("Time", 1)
                ds.createVariable("T", "f8", ())[:] = 42
            shutil.copyfile(ref / name, generated / name)
        env["FAKE_GENERATED"] = str(generated)
        result = subprocess.run([*cmd, ref], env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("2 files compared", result.stdout)
        (ref / "wrfbdy_d01").unlink()
        result = subprocess.run([*cmd, ref], env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("missing WRF reference wrfbdy_d01", result.stdout)


if __name__ == "__main__":
    unittest.main()
