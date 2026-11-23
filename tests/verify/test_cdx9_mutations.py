# SPDX-License-Identifier: GPL-3.0-or-later
"""Synthetic regression cases for the CDX9 audit findings."""
import ast
import io
import json
import os
import shutil
import shlex
import subprocess
import sys
import tarfile
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import unittest

import numpy as np
from unittest.mock import patch

from netCDF4 import Dataset

import case_data
import wrf_suite as suite
import test_harness_mutations as harness
from test_harness_mutations import ROOT, START, ScratchMutationCase, fixture


class RestartClockMutations(ScratchMutationCase):
    def n3(self, mutation=None, *, native_only=False):
        name = "real_n3_r8_fp64_cpu_chain"
        _, _, cfg, sets, _, end, _ = next(c for c in suite.CASES if c[0] == name)
        mid, interval, _, _ = suite.chain_at(sets)
        refs = {d: str(fixture(self.root / "ref", tuple(range(0, end + 1, iv)), domain=d))
                for d, iv in (("d01", 300), ("d02", 900))}
        (self.root / "ref" / "namelist.input").touch()

        def run_stub(_binary, _cfg, _sets, out):
            straight = Path(out).name == "a_straight"
            origin = 0 if straight else int(mid)
            for dom, iv in (("d01", 300), ("d02", 900 if straight else 300)):
                times = tuple(range(0 if straight else origin + iv, end + 1, iv))
                if mutation == "missing" and not straight and dom == "d02":
                    times = (900,)
                history = fixture(Path(out), times, domain=dom, start=suite.times_at(START, origin))
                if mutation == "xtime" and not straight and dom == "d02":
                    with Dataset(history, "a") as ds:
                        ds["XTIME"][-1] = 21
                for sec in range(origin + 300, end + 1, 300):
                    path = fixture(Path(out), (sec,), kind="wrfrst", domain=dom, alarm=300,
                                   start=suite.times_at(START, origin))
                    with Dataset(path, "a") as ds:
                        left = iv - (sec - origin) % iv
                        if mutation == "alarm" and not straight and dom == "d02" and sec == 900:
                            left += 1
                        ds.setncattr("WRF_ALARM_SECS_TIL_NEXT_RING_01", left)
                        ds.setncattr("WRF_ALARM_ISRINGING_01", int(left == iv and (dom != "d01" or sec == end)))
            return 0, ""

        with patch.object(suite, "run", run_stub), patch.object(suite, "config_vs_namelist", return_value=[]), \
                patch.object(suite, "chain_first_step", return_value=[]), redirect_stdout(io.StringIO()):
            return suite.restart_chain("stub", cfg, sets, refs, end, mid, interval,
                                       self.root / "out", native_only=native_only, wrf=refs["d01"])

    def test_n3_shared_state_with_different_leg_cadences(self):
        bad, _ = self.n3()
        self.assertFalse(bad, bad)

    def test_n3_missing_resumed_only_record_fails(self):
        bad, _ = self.n3("missing")
        self.assertIn("CPPWRF missing ['2026-10-12_00:20:00']", "\n".join(bad))

    def test_n3_wrong_history_alarm_fails(self):
        bad, _ = self.n3("alarm")
        self.assertIn("WRF_ALARM_SECS_TIL_NEXT_RING_01 301, expected 300", "\n".join(bad))

    def test_n3_unshared_frame_still_needs_consistent_xtime(self):
        bad, _ = self.n3("xtime")
        self.assertIn("disagrees with history Times 2026-10-12_00:20:00", "\n".join(bad))

    def test_nested_native_only_chain_cannot_pass_without_a_resumed_leg(self):
        bad, _ = self.n3(native_only=True)
        self.assertIn("native-only chain requires a single domain", "\n".join(bad))



def run_cli(cases, argv, stub=None):
    stdout, stderr = io.StringIO(), io.StringIO()
    with patch.object(suite, "CASES", cases), patch.object(sys, "argv", ["wrf_suite.py", *argv]), \
            patch.object(suite, "RESTARTS", []), patch.object(suite, "SPECIAL", []), \
            patch.object(suite, "run", side_effect=stub), redirect_stdout(stdout), redirect_stderr(stderr):
        try:
            suite.main()
        except SystemExit as exc:
            return exc.code, stdout.getvalue() + stderr.getvalue()
    raise AssertionError("suite CLI did not report an exit status")


def output_config(directory, text):
    cfg = directory / "run.ini"
    cfg.write_text(text)
    return str(cfg)


class RestartEvidenceMutations(ScratchMutationCase):
    def test_chain_missing_restarts_in_every_leg_or_missing_native_seed(self):
        name = "real_p4_r8_fp64_cpu_chain"
        _, _, cfg, sets, _, end, _ = next(c for c in suite.CASES if c[0] == name)
        mid, interval, _, _ = suite.chain_at(sets)
        ref = fixture(self.root / "ref", tuple(range(0, end + 1, 180)))
        ref.with_name("namelist.input").touch()
        for mutation in ("missing-restarts", "missing-native-seed", None):
            with self.subTest(mutation=mutation):
                def run_stub(_binary, _cfg, _sets, out):
                    out = Path(out)
                    straight = out.name == "a_straight"
                    origin = int(mid) if out.name == "b_resumed" else 0
                    fixture(out, tuple(range(0 if straight else 540, end + 1, 180)),
                            start=suite.times_at(START, origin))
                    restarts = tuple(range(180 if straight else 540, end + 1, 180))
                    if mutation == "missing-restarts":
                        restarts = (360, 540) if straight else (540,)
                    for sec in restarts:
                        path = fixture(out, (sec,), kind="wrfrst", start=suite.times_at(START, origin))
                        with Dataset(path, "a") as ds:
                            ds.setncattr("WRF_ALARM_SECS_TIL_NEXT_RING_01", 180)
                            ds.setncattr("WRF_ALARM_ISRINGING_01", int(sec == end))
                    if straight and mutation != "missing-native-seed":
                        (out / "restart_000006.nc").touch()
                    return 0, ""

                with patch.object(suite, "run", run_stub), patch.object(suite, "config_vs_namelist", return_value=[]), \
                        patch.object(suite, "chain_first_step", return_value=[]), redirect_stdout(io.StringIO()):
                    bad, _ = suite.restart_chain("stub", cfg, sets, {"": str(ref)}, end, mid, interval,
                                                 self.root / str(mutation), wrf=str(ref))
                if mutation is None:
                    self.assertFalse(bad, bad)
                else:
                    reason = "00:15:00" if mutation == "missing-restarts" else "missing restart seed"
                    self.assertIn(reason, "\n".join(bad))

    def rsa(self, mutation, *, resumed=False, zero=False):
        cfg = "configs/real_rsa_r8.ini"
        sets = ["suite.wrfrst"]
        target = "rsa"
        values = (150, 250, 400, 500, 600)
        if resumed:
            seed = fixture(self.root / "base", (150,), kind="wrfrst", alarm=90)
            cfg = f"dropin:{seed.parent}"
            sets.extend([f"suite.dropin_link={seed}", f"suite.dropin_namelist={self.root / 'ref'}"])
            values = (250,) if zero else (250, 400, 500, 650)
        history = (300, 450, 600) if resumed else (0, 150, 300, 450, 600)
        ref = fixture(self.root / "ref", history)
        if resumed:
            iv = 0 if zero else 2
            ref.with_name("namelist.input").write_text(
                f"&time_control run_seconds=550, restart=.true., restart_interval={iv}, /\n"
                "&domains time_step=50, dx=15000, dy=15000, /\n"
            )
        values = values[:1] if mutation == "truncate" else values
        for sec in values:
            fixture(ref.parent, (sec,), kind="wrfrst")

        candidate = self.root / "candidate"
        shutil.rmtree(candidate, ignore_errors=True)
        fixture(candidate, history)
        for sec in values:
            if mutation == "duplicate" and sec == values[0]:
                duplicate = fixture(candidate / "d01", (sec,), kind="wrfrst")
                with Dataset(duplicate, "a") as ds:
                    ds["T"][0, 0] = 777
            fixture(candidate, (sec,), kind="wrfrst")

        def stub(_binary, _cfg, _sets, out):
            shutil.rmtree(out, ignore_errors=True)
            shutil.copytree(candidate, out)
            return 0, ""

        with patch.object(suite, "dropin_run", side_effect=stub):
            return run_cli([(target, "cpu_dp", cfg, sets, str(ref), 600, {})],
                           ["--cpu-dp", "/bin/true", "--only", target, "--exact", "--out", str(self.root / "out")], stub)

    def test_rsa_requires_configured_inventory_and_rejects_duplicate_copies(self):
        for mutation, needle in (("truncate", "00:10:00"), ("duplicate", "duplicate CPPWRF restart"), (None, "5 wrfrst")):
            with self.subTest(mutation=mutation):
                ref = self.root / "ref"
                if ref.exists():
                    shutil.rmtree(ref)
                rc, output = self.rsa(mutation)
                self.assertEqual(rc, bool(mutation), output)
                self.assertIn(needle, output)

    def test_rsa_resume_requires_rings_from_seed_clock_even_with_zero_interval(self):
        for zero in (False, True):
            with self.subTest(zero=zero):
                if (self.root / "ref").exists():
                    shutil.rmtree(self.root / "ref")
                expected = 1 if zero else 4
                rc, output = self.rsa(None, resumed=True, zero=zero)
                self.assertEqual(rc, 0, output)
                self.assertIn(f"{expected} wrfrst", output)
        shutil.rmtree(self.root / "ref")
        rc, output = self.rsa("truncate", resumed=True)
        self.assertEqual(rc, 1, output)
        self.assertIn("00:10:50", output)

    def test_seeded_restart_clock_must_match_file_name_and_have_exactly_one_record(self):
        for mutation in ("mislabel", "multiple", "xtime", None):
            with self.subTest(mutation=mutation):
                w, c = self.root / "ref", self.root / "out"
                stamp = suite.times_at(START, 120)
                for path in (w, c):
                    if path.exists():
                        shutil.rmtree(path)
                    fixture(path, (120, 180))
                    times = (60,) if mutation == "mislabel" else (120, 180) if mutation == "multiple" else (120,)
                    rst = fixture(path, times, kind="wrfrst")
                    rst.rename(rst.with_name(f"wrfrst_d01_{stamp}"))
                    if mutation == "xtime":
                        with Dataset(path / f"wrfrst_d01_{stamp}", "a") as ds:
                            ds["XTIME"][0] = 1
                bad, _ = suite.compare_seeded_restart(c, w, START, 180)
                if mutation:
                    self.assertIn("one restart record" if mutation == "multiple" else "disagrees with filename",
                                  "\n".join(bad))
                else:
                    self.assertFalse(bad, bad)

    def test_native_split_requires_each_leg_and_prevents_shadowing(self):
        cfg = output_config(self.root, "[time]\ndt=60\n[output]\nout_interval=1\n")
        for mutation in ("first-missing", "resumed-missing", "shadowed", None):
            with self.subTest(mutation=mutation):
                ref = fixture(self.root / str(mutation) / "ref", (0, 60, 120, 180))
                ref.with_name("namelist.input").write_text("&domains\n time_step = 60,\n/\n")

                def stub(_binary, _cfg, _sets, out):
                    out = Path(out)
                    out.mkdir(parents=True, exist_ok=True)
                    if out.name == "a_first":
                        (out / "restart_000001.nc").touch()
                        if mutation != "first-missing":
                            path = fixture(out, (0, 60, 120, 180) if mutation == "resumed-missing" else (0, 60, 120))
                            if mutation == "shadowed":
                                with Dataset(path, "a") as ds:
                                    ds["T"][:2] = 777
                    elif mutation != "resumed-missing":
                        fixture(out, (0, 60, 120, 180) if mutation in ("first-missing", "shadowed") else (120, 180))
                    return 0, ""

                with patch.object(suite, "run", stub):
                    bad, _ = suite.restarted_case("stub", cfg, [], {"": str(ref)}, 180, 60, ref.parent / "job", True)
                if mutation:
                    self.assertIn({"first-missing": "first leg: CPPWRF missing", "resumed-missing": "resumed leg: CPPWRF missing",
                                   "shadowed": "resumed records at or before the split"}[mutation], "\n".join(bad))
                else:
                    self.assertFalse(bad, bad)

    def test_wrf_restart_seed_does_not_require_history_at_split(self):
        cfg = output_config(self.root, "[time]\ndt=300\n[output]\nout_interval=3\n")
        ref = fixture(self.root / "ref", (0, 900, 1800))
        ref.with_name("namelist.input").write_text("&domains\n time_step = 300,\n/\n")

        def seed(path):
            f = fixture(path, (600,), kind="wrfrst")
            with Dataset(f, "a") as ds:
                ds.setncattr("WRF_ALARM_SECS_TIL_NEXT_RING_01", 300)

        seed(ref.parent)

        def stub(_binary, _cfg, _sets, out):
            out = Path(out)
            if out.name == "a_first":
                fixture(out, (0, 900))
                seed(out)
            else:
                fixture(out, (900, 1200, 1500, 1800), start=suite.times_at(START, 600))
            return 0, ""

        with patch.object(suite, "run", stub):
            bad, _ = suite.restarted_case("stub", cfg, [], {"": str(ref)}, 1800, 600, self.root / "job")
        self.assertFalse(bad, bad)
        fixture(ref.parent, (0, 450, 900, 1800))
        with patch.object(suite, "run", stub):
            bad, _ = suite.restarted_case("stub", cfg, [], {"": str(ref)}, 1800, 600, self.root / "extra")
        self.assertIn("only-wrf ['2026-10-12_00:07:30']", "\n".join(bad))

    def test_special_native_snapshot_requires_payload_step_and_descriptors(self):
        inputs = self.root / "inputs"
        inputs.mkdir()
        (inputs / "wrfinput_d01").touch()
        for mutation in ("missing-seed", "empty", "wrong-step", "wrong-time", "bad-descriptor", None):
            with self.subTest(mutation=mutation):
                def stub(_binary, _cfg, _sets, out):
                    out = Path(out)
                    out.mkdir(parents=True)
                    if mutation != "missing-seed":
                        (out / "restart_000001.nc").touch()
                    (out / "snap_000003.bin").write_bytes(b"" if mutation == "empty" else b"test")
                    (out / "snap_000003.json").write_text(json.dumps({
                        "step": 2 if mutation == "wrong-step" else 3,
                        "time": 120 if mutation == "wrong-time" else 180,
                        "variables": [{"name": "u", "dtype": "float32", "shape": [1, 1, 1],
                                       "offset": 8 if mutation == "bad-descriptor" else 0, "nbytes": 4}],
                    }))
                    return 0, ""

                with patch.object(suite, "WR", str(self.root)), patch.object(suite, "run", stub):
                    good, msg = suite.restart_p1("stub", self.root / str(mutation), run_dir="inputs")
                self.assertEqual(good, mutation is None, msg)

    def test_p8_special_uses_registered_duration(self):
        inputs = self.root / "run_P8_r8"
        inputs.mkdir()
        (inputs / "wrfinput_d01").touch()
        ref = fixture(self.root / "rst_P8_r8", (0, 180, 600))
        ref.with_name("namelist.input").touch()
        (self.root / "rsm_P8_r8").mkdir()
        invoked = []

        def stub(_binary, _cfg, sets, _out):
            invoked.append([s for s in sets if s.startswith("run.run_time=")][-1])
            return 1, "stop after inspecting the effective run length"

        with patch.object(suite, "WR", str(self.root)), patch.object(suite, "run", stub), \
                patch.object(suite, "config_vs_namelist", return_value=[]):
            suite.restart_check("stub", self.root / "p8", "p8", "configs/real_p8_r8.ini", "run_P8_r8")
        self.assertEqual(invoked, ["run.run_time=600"])


class InventoryMutations(ScratchMutationCase):
    def cli_input(self):
        cfg = output_config(self.root, "[time]\ndt=60\n")
        ref = fixture(self.root / "ref", (0, 60))
        candidate = self.root / "candidate"
        fixture(candidate, (0, 60))
        cases = [(name, backend, cfg, sets, str(ref), 60, {})
                 for name, backend, sets in (("demo_cpu", "cpu_dp", []), ("demo_cuda", "cuda_dp", []),
                                             ("demo_cpu_slow", "cpu_dp", ["suite.slow"]))]

        def stub(_binary, _cfg, _sets, out):
            shutil.copytree(candidate, out, dirs_exist_ok=True)
            return 0, ""

        return cases, stub

    def test_mixed_explicit_selection_requires_every_result(self):
        cases, stub = self.cli_input()
        for selector, flags, missing in (("demo_cpu,demo_cuda", ["--exact"], "demo_cuda"),
                                        ("demo_cpu", [], "demo_cpu_slow")):
            with self.subTest(selector=selector, flags=flags):
                rc, output = run_cli(cases, ["--cpu-dp", "/bin/true", "--only", selector, *flags,
                                             "--out", str(self.root / "out")], stub)
                self.assertEqual(rc, 1, output)
                self.assertIn("ERROR no result for requested checks: " + missing, output)

    def test_empty_selectors_and_zero_checks_fail_while_reported_skips_are_allowed(self):
        cases, stub = self.cli_input()
        for args in ([], ["--cpu-dp", "/bin/true", "--only", ""],
                     ["--cpu-dp", "/bin/true", "--only", "demo_cpu,"]):
            with self.subTest(args=args):
                rc, output = run_cli(cases, [*args, "--out", str(self.root / "out")], stub)
                self.assertNotEqual(rc, 0, output)
        Path(cases[0][4]).unlink()
        rc, output = run_cli(cases, ["--cpu-dp", "/bin/true", "--only", "demo_cpu", "--exact", "--allow-skip"])
        self.assertEqual(rc, 0, output)
        self.assertIn("SUITE PASS 0 FAIL 0 SKIP 1", output)

    def test_reporter_requires_correct_names_counts_and_file_logs(self):
        for name in ("setup.out", "unit.log", "selftest.log"):
            (self.root / name).write_text("exit 0\n" if name != "setup.out" else "")
        valid = "PASS wanted [self only]  (0 s)\nSUITE PASS 1 FAIL 0 SKIP 0\nexit 0\n"
        for suitelog, files, ideal, need_files in (
            (valid.replace("wanted", "other"), "", "", 0),
            (valid.replace("PASS 1", "PASS 2"), "", "", 0),
            ("SUITE PASS 0 FAIL 0 SKIP 0\nexit 0\n", "", "", 0),
            (valid, "PASS good\nSKIP missing\nexit 0\n", "PASS ideal\nexit 0\n", 1),
            (valid, "PASS good\nexit 0 good\n", "", 1),
            (valid, "PASS good\nexit 0 good\n", "PASS ideal\nexit 0\n", 1),
        ):
            with self.subTest(suitelog=suitelog, files=files, ideal=ideal):
                (self.root / "suite.log").write_text(suitelog)
                (self.root / "files.log").write_text(files)
                (self.root / "idealfiles.log").write_text(ideal)
                result = subprocess.run(["bash", ROOT / "tools/gcp/suite_status.sh", self.root, "wanted", str(need_files)],
                                        capture_output=True, text=True)
                expected = int(suitelog != valid or (need_files and (not ideal or "SKIP" in files)))
                self.assertEqual(result.returncode, expected, result.stdout + result.stderr)

    def test_file_wrappers_reject_empty_selections_and_missing_legs(self):
        build = self.root / "build"
        build.mkdir()
        (build / "cppwrf").symlink_to("/bin/true")
        runs = self.root / "wrf_runs"
        (runs / "run_P1_r8/steps").mkdir(parents=True)
        (runs / "rst_P1_r8").mkdir()
        env = dict(os.environ, CPPWRF_WRF_RUNS=str(runs), CPPWRF_WRF_REF=str(self.root / "ideal"),
                   CPPWRF_ALLOW_SKIP="1")
        shell = ["bash", ROOT / "tests/verify/run_wrf_files_test.sh", build]
        ideal = [sys.executable, ROOT / "tests/verify/ideal_files_test.py", build]
        for cmd, pick in ((shell, "p1"), (shell, "typo"), (ideal, "typo"), (ideal, "ideal_squall2d_x_fp64_cpu")):
            with self.subTest(wrapper=str(cmd[1]), pick=pick):
                result = subprocess.run([*cmd, pick], env=env, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

        ref = fixture(runs / "run_P1_r8/steps", (0, 60))
        exe = build / "cppwrf"
        exe.unlink()
        exe.write_text('''#!/usr/bin/env bash
for x in "$@"; do
  case $x in output.dir=*) cp "$CDX9_REF" "${x#*=}" ;; esac
done
''')
        exe.chmod(0o755)
        result = subprocess.run([*shell, "p1"], env=dict(env, CDX9_REF=str(ref)), capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_exact_case_requires_configured_nested_domain(self):
        cfg = output_config(self.root, "[time]\ndt=60\n[domains]\nmax_dom=2\n")
        ref = fixture(self.root / "ref", (0, 60))
        candidate = self.root / "candidate"
        fixture(candidate, (0, 60))

        def stub(_binary, _cfg, _sets, out):
            shutil.copytree(candidate, out, dirs_exist_ok=True)
            return 0, ""

        for scope, expected in (([], 1), (["suite.history_domains=d01"], 0)):
            with self.subTest(scope=scope):
                rc, output = run_cli([("nested", "cpu_dp", cfg, scope, {"d01": str(ref)}, 60, {})],
                                     ["--cpu-dp", "/bin/true", "--out", str(self.root / "out")], stub)
                self.assertEqual(rc, expected, output)


def auxhist3_fixture(directory, seconds, *, start=START):
    """A subset of A09's h:3 variables with the WRF NetCDF record layout."""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"auxhist3_d01_{suite.times_at(START, seconds[0])}"
    with Dataset(path, "w") as ds:
        ds.createDimension("Time", None)
        ds.createDimension("DateStrLen", 19)
        ds.createDimension("west_east", 1)
        ds.createDimension("south_north", 1)
        ds.START_DATE = start
        ds.SIMULATION_START_DATE = START
        times = ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
        t2min = ds.createVariable("T2MIN", "f8", ("Time", "south_north", "west_east"))
        for i, sec in enumerate(seconds):
            times[i] = np.frombuffer(suite.times_at(START, sec).encode("ascii"), dtype="S1")
            t2min[i, 0, 0] = 273.0 + sec / 3600
    return path


class ContentMutations(ScratchMutationCase):
    def test_a09_split_reads_multirecord_auxhist3_without_xtime(self):
        name = "real_a09_r8_fp64_cpu_1h_rst"
        _, _, cfg, sets, _, end, _ = next(c for c in suite.CASES if c[0] == name)
        mid = int(suite.restart_at(sets))
        split = suite.times_at(START, mid)
        refdir = self.root / "ref"
        ref = fixture(refdir, tuple(range(0, end + 1, 600)))
        auxhist3_fixture(refdir, range(0, end + 1, 180))
        seed = fixture(refdir, (mid,), kind="wrfrst")
        with Dataset(seed, "a") as ds:
            ds.setncattr(suite.HISTORY_ALARM, 600)
            ds.setncattr(suite.AUX_ALARM, 180)
        (refdir / "namelist.input").touch()

        for missing_tick in (False, True):
            with self.subTest(missing_tick=missing_tick):
                def run_stub(_binary, _cfg, _sets, out):
                    out = Path(out)
                    if out.name == "a_first":
                        fixture(out, tuple(range(0, mid + 1, 600)))
                        auxhist3_fixture(out, range(0, mid + 1, 180))
                        shutil.copyfile(seed, out / seed.name)
                    else:
                        fixture(out, tuple(range(mid + 600, end + 1, 600)), start=split)
                        ticks = list(range(mid + 180, end + 1, 180))
                        if missing_tick:
                            ticks.remove(2160)
                        auxhist3_fixture(out, ticks, start=split)
                    return 0, ""

                with patch.object(suite, "run", run_stub):
                    bad, _ = suite.restarted_case("stub", cfg, sets, {"": str(ref)}, end, mid,
                                                  self.root / str(missing_tick))
                if missing_tick:
                    self.assertIn("b_resumed auxhist3: CPPWRF missing ['2026-10-12_00:36:00']", "\n".join(bad))
                else:
                    self.assertFalse(bad, bad)

    def test_auxhist3_inventory_checks_xtime_when_present(self):
        ticks = (0, 180, 360)
        path = auxhist3_fixture(self.root, ticks)
        expected = {suite.times_at(START, sec) for sec in ticks}
        self.assertIn("expected one XTIME per history record",
                      "\n".join(suite.history_inventory([path], expected, START)))
        with Dataset(path, "a") as ds:
            ds.createVariable("XTIME", "f8", ("Time",))[:] = [0, 3, 6]
        self.assertFalse(suite.history_inventory([path], expected, START, require_xtime=False))
        with Dataset(path, "a") as ds:
            ds["XTIME"][1] = 4
        self.assertIn("XTIME 4.0 disagrees with history Times 2026-10-12_00:03:00",
                      "\n".join(suite.history_inventory([path], expected, START, require_xtime=False)))
        malformed = auxhist3_fixture(self.root / "scalar-xtime", ticks)
        with Dataset(malformed, "a") as ds:
            ds.createVariable("XTIME", "f8").assignValue(0)
        self.assertIn("expected one XTIME per history record",
                      "\n".join(suite.history_inventory([malformed], expected, START, require_xtime=False)))

    def test_auxhist3_requires_all_configured_ticks_and_effective_opt_ins(self):
        cfg = output_config(self.root, "[output]\nauxhist3=true ; diagnostics\n[phys]\nauxhist3_interval=3\n")
        w, c = self.root / "ref", self.root / "out"
        ref = fixture(w, (0, 600))
        fixture(c, (0, 600))
        nl = self.root / "dropin"
        nl.mkdir()
        (nl / "namelist.input").write_text(
            "&time_control output_diagnostics=1, io_form_auxhist3=2, auxhist3_interval=3, /\n"
        )
        for config in (cfg, f"dropin:{nl}"):
            with self.subTest(config=config):
                bad, _ = suite.exact_case({"": str(ref)}, c, 600, [], cfg=config)
                self.assertIn("no WRF reference files", "\n".join(bad))
        for path in (w, c):
            fixture(path, (180,), kind="auxhist3")
        bad, _ = suite.exact_case({"": str(ref)}, c, 600, [], cfg=cfg, required_times=(START,))
        self.assertIn("CPPWRF missing ['2026-10-12_00:06:00', '2026-10-12_00:09:00']", "\n".join(bad))
        for path in (w, c):
            fixture(path, (180, 360, 540), kind="auxhist3")
        bad, _ = suite.exact_case({"": str(ref)}, c, 600, [], cfg=cfg)
        self.assertFalse(bad, bad)

    def test_exact_numeric_payload_requires_compared_cells_after_waivers(self):
        for mutation in ("waived", "empty-axis", "extra-zero-record-file"):
            with self.subTest(mutation=mutation):
                dirs = [self.root / mutation / s for s in ("ref", "out")]
                paths = [fixture(d, (60,)) for d in dirs]
                for path in paths:
                    with Dataset(path, "a") as ds:
                        if mutation == "waived":
                            ds.renameVariable("T", "LH")
                            ds.ISFFLX = 0
                        elif mutation == "empty-axis":
                            # The remaining declaration is a numeric field with no cells.
                            ds.setncattr("ISFFLX", 0)
                            ds.renameVariable("T", "LH")
                            ds.createDimension("empty", None)
                            ds.createVariable("U", "f8", ("Time", "empty"))
                if mutation == "waived":
                    with Dataset(paths[1], "a") as ds:
                        ds["LH"][0, 0] = 999
                if mutation == "extra-zero-record-file":
                    empty = paths[1].with_name(f"wrfout_d01_{START}")
                    with Dataset(empty, "w") as ds:
                        ds.createDimension("Time", None)
                        ds.createDimension("DateStrLen", 19)
                        ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
                bad, _ = suite.exact_case({"": str(paths[0])}, dirs[1], 60, [])
                self.assertIn("zero records" if mutation == "extra-zero-record-file" else "no model fields",
                              "\n".join(bad))

    def test_seeded_history_waives_only_individually_undefined_fields(self):
        w, c = self.root / "ref", self.root / "out"
        for directory in (w, c):
            for sec in (120, 180):
                path = fixture(directory, (sec,))
                with Dataset(path, "a") as ds:
                    ds.createVariable("RQCBLTEN", "f8", ("Time",))[:] = 7 if directory == c and sec == 120 else 0
                    if sec == 180:
                        ds.createVariable("QCLOUD", "f8", ("Time",))[:] = 1
            fixture(directory, (120,), kind="wrfrst")
        bad, _ = suite.compare_seeded_restart(c, w, START, 180)
        self.assertFalse(bad, bad)
        with Dataset(c / f"wrfout_d01_{suite.times_at(START, 180)}", "a") as ds:
            ds["RQCBLTEN"][0] = 9
        bad, _ = suite.compare_seeded_restart(c, w, START, 180)
        self.assertIn("RQCBLTEN", "\n".join(bad))

    def test_init_real_comparator_rejects_empty_equal_records(self):
        paths = [self.root / s for s in ("out", "ref")]
        for path in paths:
            with Dataset(path, "w") as ds:
                ds.createDimension("Time", None)
                ds.createVariable("U", "f8", ("Time",))
        result = subprocess.run([sys.executable, ROOT / "tests/init_real/compare_real.py", *paths],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("zero Time records", result.stderr)

    def test_targeted_real_inventory_requires_bdy_sst_and_fdda_at_multiple_times(self):
        build = self.root / "build"
        stub = build / "src/init_real/cppwrf_real"
        stub.parent.mkdir(parents=True)
        stub.write_text("#!/usr/bin/env bash\nwhile [ \"$1\" != --out-dir ]; do shift; done\ncp \"$SOURCE\"/wrfinput_d01 \"$2\"\n")
        stub.chmod(0o755)
        dir_ = self.root / "target"
        initial = fixture(dir_, (0,))
        initial.rename(dir_ / "wrfinput_d01")
        for hours in (1, 0):
            with self.subTest(hours=hours):
                (dir_ / "namelist.input").write_text(f"""
&time_control start_year=2026, start_month=10, start_day=12, start_hour=0,
 end_year=2026, end_month=10, end_day=12, end_hour={hours}, interval_seconds=3600, /
&domains max_dom=1, /
&physics sst_update=1, /
&fdda grid_fdda=1, /
""")
                result = subprocess.run(["bash", ROOT / "tests/init_real/run_real_test.sh", build, dir_],
                                        env=dict(os.environ, SOURCE=str(dir_)), capture_output=True, text=True)
                self.assertEqual(result.returncode, hours, result.stdout + result.stderr)
                if hours:
                    for name in ("wrfbdy_d01", "wrflowinp_d01", "wrffdda_d01"):
                        self.assertIn(f"missing WRF reference {name}", result.stdout)

    def test_default_third_domain_inherits_second_domains_cadence(self):
        conf = {"time": {"dt": "60"}, "output": {"out_interval": "5"}, "domains": {"parent_time_step_ratio": "3"},
                "d02": {"out_interval": "45"}, "d03": {"parent_time_step_ratio": "3"}}
        self.assertEqual(suite.chain_history(conf, ["d01", "d02", "d03"], 1200),
                         {"d01": 300, "d02": 900, "d03": 900})

    def test_year_one_timestamp_keeps_four_digit_year(self):
        self.assertEqual(suite.times_at("0001-01-01_00:00:00", 60), "0001-01-01_00:01:00")


class StagingAndL2Mutations(ScratchMutationCase):
    def test_stage_keeps_dropin_seed_outside_selected_reference_directories(self):
        root = self.root / "wrf_runs"
        (root / "run_P1_r8/steps").mkdir(parents=True)
        ref = fixture(root / "rsm_P1_r8", (120, 180))
        ref.with_name("namelist.input").touch()
        seed = fixture(root / "rst_P1_r8", (60,), kind="wrfrst")
        name = "dropin_rsm_p1_r8_fp64_cpu"
        for flags in ([], ["--refs-only"], ["--no-refs"]):
            with self.subTest(flags=flags):
                archive = self.root / "out.tar"
                result = subprocess.run([sys.executable, ROOT / "tools/colab/stage_data.py", "--only", name,
                                         *flags, archive], env=dict(os.environ, CPPWRF_WRF_RUNS=str(root)),
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                with tarfile.open(archive) as tf:
                    self.assertEqual(tf.extractfile(f"wrf_runs/{seed.relative_to(root)}").read(), seed.read_bytes())

    def test_stage_preserves_stitched_history_and_finds_off_cadence_seed(self):
        stage = harness.HarnessMutations.load_stage(self)
        ref = fixture(self.root / "ref", (0, 900, 1800))
        seed = fixture(ref.parent, (600,), kind="wrfrst")
        cfg = output_config(self.root, f"[real]\ndir={ref.parent}\n")
        name = "stitched"
        archive = self.root / "out.tar"
        case = (name, "cpu_dp", cfg, ["suite.restart_at=600"], str(ref), 1800, {})
        with patch.object(suite, "CASES", [case]), patch.object(sys, "argv", ["stage_data.py", "--refs-only",
                          "--only", name, str(archive)]), redirect_stdout(io.StringIO()):
            stage.main()
        with tarfile.open(archive) as tf:
            self.assertEqual(tf.extractfile("wrf_runs/ref/" + ref.name).read(), ref.read_bytes())
            self.assertEqual(tf.extractfile("wrf_runs/ref/" + seed.name).read(), seed.read_bytes())

    def test_stage_rejects_empty_ideal_reference_and_missing_endpoint(self):
        stage = ROOT / "tools/colab/stage_data.py"
        archive = self.root / "out.tar"
        wrf_ref = self.root / "wrf_ref"
        wrf_ref.mkdir()
        env = dict(os.environ, CPPWRF_WRF_REF=str(wrf_ref), CPPWRF_WRF_RUNS=str(self.root))
        result = subprocess.run([sys.executable, stage, "--only", "ideal_squall2d_x_fp32", archive],
                                env=env, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(archive.exists())

        stage = harness.HarnessMutations.load_stage(self)
        ref = fixture(self.root / "ref", (0, 60))
        cfg = output_config(self.root, f"[real]\ndir={ref.parent}\n")
        case = ("ordinary", "cpu_dp", cfg, [], str(ref), 180, {})
        with patch.object(suite, "CASES", [case]), patch.object(sys, "argv", ["stage_data.py", "--refs-only",
                          "--only", "ordinary", str(archive)]), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "no staging reference frames"):
                stage.main()

    def test_manifest_requires_separate_chain_history_and_namelist_and_skips_native_wrf_seed(self):
        cfg = output_config(self.root, f"[real]\ndir={self.root / 'input'}\n")
        ordinary = self.root / f"ordinary/wrfout_d01_{START}"
        chain = self.root / f"chain/wrfout_d01_{START}"
        case = ("chain", "cpu_dp", cfg, ["suite.chain=600,300", f"suite.chain_wrf={chain}"], str(ordinary), 1200, {})
        native = ("native", "cpu_dp", cfg, ["suite.restart_at=600", "suite.native_restart"], str(ordinary), 1200, {})
        with patch.object(suite, "WR", str(self.root)), patch.object(suite, "CASES", [case, native]):
            _, problems = case_data.derive("chain", {"t": {
                f"wrf_runs/ordinary/{ordinary.name}": (1, "2026-10-12 00:00"),
                "wrf_runs/input/wrfinput_d01": (1, "2026-10-12 00:00"),
            }}, ["t"])
            self.assertIn(f"wrf_runs/chain/{chain.name}: in no tar", problems)
            self.assertIn("wrf_runs/chain/namelist.input: in no tar", problems)
            required, _ = case_data.needs("native")
            self.assertFalse(any("/wrfrst_" in path for path in required), required)

    def test_manifest_rejects_a_single_rsa_restart_as_complete_inventory(self):
        required, _ = case_data.needs("real_rsa_r8_fp64_cpu")
        names = {f"wrf_runs/run_RSA_r8/wrfrst_d01_{suite.times_at(START, s)}" for s in (150, 250, 400, 500, 600)}
        self.assertLessEqual(names, {path for path, mandatory in required.items() if mandatory})

    def test_stage_files_requires_resumed_history_and_the_minute_one_seed(self):
        stage = harness.HarnessMutations.load_stage(self)
        ref, rst, rsm = (self.root / d for d in ("run_P1_r8/steps", "rst_P1_r8", "rsm_P1_r8"))
        fixture(ref, (0, 60, 120))
        fixture(rst, (120,), kind="wrfrst")
        (rst / "namelist.input").write_text("&time_control\n start_year=2026,\n start_month=10,\n start_day=12,\n"
                                           " start_hour=0,\n/\n")
        archive = self.root / "files.tar"
        args = ["stage_data.py", "--only", "", "--files", "p1", str(archive)]
        with patch.object(sys, "argv", args), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(SystemExit, "missing wrfout.*rsm_P1_r8"):
                stage.main()
            fixture(rsm, (120,))
            with self.assertRaisesRegex(SystemExit, "missing minute-1 restart seed"):
                stage.main()
            seed = fixture(rst, (60,), kind="wrfrst")
            stage.main()
        with tarfile.open(archive) as tf:
            self.assertEqual(tf.extractfile(f"wrf_runs/rst_P1_r8/{seed.name}").read(), seed.read_bytes())

    def test_remote_runner_uses_exact_selection_as_staging_does(self):
        source = ast.parse((ROOT / "tools/colab/remote_job.py").read_text())
        expression = next(node.args[0] for node in ast.walk(source) if isinstance(node, ast.Call) and node.args and
                          isinstance(node.func, ast.Name) and node.func.id == "sh" and
                          isinstance(node.args[0], ast.JoinedStr) and any(isinstance(part, ast.Constant) and
                          "tests/verify/wrf_suite.py" in str(part.value) for part in node.args[0].values))
        command = eval(compile(ast.Expression(expression), "<offline CLI template>", "eval"),
                       {"args": "--cpu-dp /bin/true", "cases": "demo_cpu"})
        args = shlex.split(command)[2:]
        args[args.index("--out") + 1] = str(self.root / "out")
        cases, stub = InventoryMutations.cli_input(self)
        rc, output = run_cli(cases, args, stub)
        self.assertEqual(rc, 0, output)
        self.assertNotIn("demo_cpu_slow", output)
        self.assertIn("--exact", args)

    def test_staging_cache_discriminates_revisions(self):
        code = (ROOT / "tools/colab/colab_run.sh").read_text()
        block = "key=" + code.split("\nkey=", 1)[1].split("\nDATA=", 1)[0]
        bin_ = self.root / "bin"
        bin_.mkdir()
        git = bin_ / "git"
        git.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$CDX9_REV"\n')
        git.chmod(0o755)
        env = dict(os.environ, REPO=str(ROOT), STAGE="selected", FILES="", NOREFS="", WT="",
                   PATH=f"{bin_}:{os.environ['PATH']}")
        keys = [subprocess.check_output(["bash", "-e", "-c", block + '\nprintf "%s\\n" "$key"'],
                                       env=dict(env, CDX9_REV=rev), text=True) for rev in ("0688afc", "changed")]
        self.assertNotEqual(*keys)

    def test_l2_rejects_duplicate_snapshot_and_wrf_clocks_and_empty_overlap(self):
        wrf, out = harness.ComparatorMutations.l2_fixture(self, 0)
        first = out / "snap_000001"
        header = json.loads(first.with_suffix(".json").read_text())
        second = out / "snap_000002"
        shutil.copyfile(first.with_suffix(".bin"), second.with_suffix(".bin"))
        second.with_suffix(".json").write_text(json.dumps(dict(header, step=2)))
        with self.assertRaisesRegex(RuntimeError, "duplicate snapshot time"):
            suite.compare(wrf, out, 60)
        second.with_suffix(".json").unlink()

        repeated = self.root / "duplicate-reference"
        with Dataset(wrf) as original, Dataset(repeated, "w") as ds:
            for name, dim in original.dimensions.items():
                ds.createDimension(name, 2 if name == "Time" else len(dim))
            for name, var in original.variables.items():
                ds.createVariable(name, var.dtype, var.dimensions)[:] = np.repeat(var[:], 2, axis=0)
        with self.assertRaisesRegex(RuntimeError, "duplicate WRF XTIME"):
            suite.compare(repeated, out, 60)

        first.with_suffix(".json").write_text(json.dumps(dict(header, time=180)))
        with self.assertRaisesRegex(RuntimeError, "no matched WRF and CPPWRF"):
            suite.compare(wrf, out, 60)

    def test_l2_suite_does_not_replace_duplicate_metric(self):
        result = subprocess.CompletedProcess([], 0, "60 U 0 9 0 1\n60 U 0 0 0 1\n", "")
        with patch.object(suite.subprocess, "run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "duplicate L2 metric"):
                suite.compare("reference", "output", 60)


if __name__ == "__main__":
    unittest.main()
