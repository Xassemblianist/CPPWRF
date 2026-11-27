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
                  extra_resumed=False, bad_alarm=False, split_wrf=False):
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
        wrf_times = wrf_times[:-1] if omit_wrf_final else wrf_times
        if split_wrf:  # WRF's stream in two files (frames_per_outfile): the second named by its first frame
            fixture(refdir, wrf_times[len(wrf_times) // 2:])
            wrf_times = wrf_times[:len(wrf_times) // 2]
        ref = fixture(refdir, wrf_times)
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

    def test_chain_reads_whole_split_wrf_stream(self):  # AUDA4 A4-15
        name = "real_d0_r8_fp64_cpu_chain"
        bad, what = self.run_chain(name, split_wrf=True)
        self.assertEqual(bad, [])
        self.assertIn("in 2 file pairs", what)

    def test_seeded_restart_rejects_extra_resumed_history_frame(self):  # AUDA4 A4-2
        start60 = wrf_suite.times_at(START, 60)
        rsm, res = self.root / "rsm", self.root / "res"
        fixture(rsm, (120, 180), start=start60)
        fixture(rsm, (120,), kind="wrfrst", start=start60, alarm=60000)
        fixture(res, (120,), kind="wrfrst", start=start60, alarm=60000)
        history = fixture(res, (120, 180), start=start60)
        bad, _ = wrf_suite.compare_seeded_restart(str(res), str(rsm), START, 180)
        self.assertEqual(bad, [])
        history.unlink()
        fixture(res, (60, 120, 180), start=start60)  # a frame at the restart time WRF does not write
        bad, _ = wrf_suite.compare_seeded_restart(str(res), str(rsm), START, 180)
        self.assertIn(f"only-cppwrf ['{start60}']", "\n".join(bad))

    def test_layout_restart_cases_are_self_only(self):  # AUDA4 A4-3
        names = [c[0] for c in wrf_suite.CASES if wrf_suite.layout_restart_at(c[3])]
        self.assertTrue(names)
        for name in names:
            self.assertEqual(wrf_suite.kind(name), "self", name)

    def test_netcdf_readers_hold_the_lock(self):  # AUDA4 A4-1
        import out_hashes
        held = []

        def probe(*_args, **_kwargs):
            held.append(wrf_suite._NC._is_owned())
            return []
        with patch.object(out_hashes, "hash_tree", probe):
            wrf_suite.output_hashes("root", "path")
        ref = fixture(self.root / "packed", (0,))  # a packed reference: no namelist.input beside it
        with patch("netCDF4.Dataset", side_effect=OSError(-101, "NetCDF: HDF error")):
            with self.assertRaises(OSError):  # a racy read error is no "not a y slab"
                wrf_suite.y_slab(str(ref))
        self.assertEqual(held, [True])
        self.assertFalse(wrf_suite.y_slab(str(ref)))  # no south_north dimension

    def chain_pair(self, kind):
        straight, resumed = self.root / "straight", self.root / "resumed"
        files = []
        for d in (straight, resumed):
            path = fixture(d, (120, 180), kind=kind)
            with Dataset(path, "a") as ds:
                ds.createVariable("CROPCAT", "i4", ("Time", "x"))[:] = 1
            files.append(path)
        return files

    def test_native_chain_compares_cropcat_and_global_attributes(self):  # AUDA4 A1-7
        def compare(files, kind, wrfrst_resume):
            noise = set()
            bad, _ = wrf_suite.chain_self_stream([str(files[0])], [str(files[1])], wrf_suite.times_at(START, 60),
                                                 restart=kind == "wrfrst", wrfrst_resume=wrfrst_resume, noise=noise)
            return bad, noise
        for kind in ("wrfout", "wrfrst"):
            with self.subTest(kind=kind):
                files = self.chain_pair(kind)
                self.assertEqual(compare(files, kind, False)[0], {})
                with Dataset(files[1], "a") as ds:
                    ds["CROPCAT"][:] = -9999
                bad, _ = compare(files, kind, False)  # native: CPPWRF's restart file carries CROPCAT
                self.assertIn("CROPCAT", bad)
                bad, noise = compare(files, kind, True)  # wrfrst: NOAHMP_INIT resets it, as in WRF
                self.assertEqual((bad, noise), ({}, {"CROPCAT"}))
                with Dataset(files[1], "a") as ds:
                    ds["CROPCAT"][:] = 1
                    ds.SIMULATION_START_DATE = "2026-10-12_00:00:01"
                for resume in (False, True):
                    self.assertIn("(metadata)", compare(files, kind, resume)[0])
                shutil.rmtree(self.root / "straight")
                shutil.rmtree(self.root / "resumed")

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

    def test_l2_rejects_nonfinite_values_of_ungated_fields(self):  # AUDA4 A4-4
        tol = {"U": 3e-4, "W": 1e-2}  # a cuda case gates some fields only

        def gate(res):
            return [v for v, tl in tol.items() if not (res.get(v, float("inf")) <= tl)]
        wrf, out = self.l2_fixture(0)
        self.assertEqual(gate(wrf_suite.compare(wrf, out, 60)), [])
        meta = json.loads((out / "snap_000001.json").read_text())
        v = next(f for f in meta["variables"] if f["name"] == "v")
        with (out / "snap_000001.bin").open("r+b") as data:  # CPPWRF's V NaN at one point
            data.seek(v["offset"])
            data.write(np.float64(np.nan).tobytes())
        with self.assertRaisesRegex(RuntimeError, "V: 1 non-finite values in CPPWRF"):
            wrf_suite.compare(wrf, out, 60)
        wrf, out = self.l2_fixture(0)
        with Dataset(wrf, "a") as ds:  # an infinity in WRF's P
            ds["P"][0, 0, 0, 0] = np.inf
        with self.assertRaisesRegex(RuntimeError, "P: 1 non-finite values in WRF"):
            wrf_suite.compare(wrf, out, 60)

    def snaps(self, d, steps, *, dt=60.0, value=1.0):
        d.mkdir(parents=True, exist_ok=True)
        for step in steps:
            fields = [{"name": "u", "shape": [1, 1, 2], "dtype": "float64", "offset": 0}]
            (d / f"snap_{step:06d}.bin").write_bytes(np.full(2, value + step, dtype="<f8").tobytes())
            (d / f"snap_{step:06d}.json").write_text(json.dumps({"time": step * dt, "step": step,
                                                                 "variables": fields}))

    def test_backend_compare_requires_the_same_steps(self):  # AUDA4 A4-10
        import verify
        a, b = self.root / "a", self.root / "b"
        self.snaps(a, (0, 3))
        self.snaps(b, (0, 3))
        with redirect_stdout(io.StringIO()):
            self.assertFalse(verify.run_compare(str(a), str(b), 1e-3).failed)
            (b / "snap_000003.json").unlink()  # b stopped after its initial snapshot
            self.assertTrue(verify.run_compare(str(a), str(b), 1e-3).failed)
            shutil.rmtree(b)
            self.snaps(b, (0, 3), dt=30.0)  # the same step at another time
            self.assertTrue(verify.run_compare(str(a), str(b), 1e-3).failed)

    def test_wrf_theta_compare_requires_every_wrf_time(self):  # AUDA4 A4-10
        import verify
        wrf, d = self.root / "straka.nc", self.root / "straka"
        with Dataset(wrf, "w") as ds:
            for name, length in (("Time", 2), ("z", 2), ("zs", 3), ("y", 1), ("x", 4)):
                ds.createDimension(name, length)
            ds.createVariable("XTIME", "f8", ("Time",))[:] = [0, 5]
            ds.createVariable("T", "f8", ("Time", "z", "y", "x"))[:] = 0
            ds.createVariable("PH", "f8", ("Time", "zs", "y", "x"))[:] = 0
            ds.createVariable("PHB", "f8", ("Time", "zs", "y", "x"))[:] = np.arange(3)[None, :, None, None] * 9.81
        d.mkdir()
        for step, time in ((0, 0.0), (300, 300.0)):
            fields, blob = [], b""
            for name, value in (("z_m", np.array([0.5, 1.5])[:, None, None] * np.ones((2, 1, 4))),
                                ("th_pert", np.zeros((2, 1, 4)))):
                fields.append({"name": name, "shape": list(value.shape), "dtype": "float64", "offset": len(blob)})
                blob += value.astype("<f8").tobytes()
            (d / f"snap_{step:06d}.bin").write_bytes(blob)
            (d / f"snap_{step:06d}.json").write_text(json.dumps({"time": time, "step": step, "variables": fields}))
        with redirect_stdout(io.StringIO()):
            self.assertFalse(verify.run_wrf(str(wrf), str(d), 0.05).failed)
            (d / "snap_000300.json").unlink()  # CPPWRF wrote t = 0 only
            self.assertTrue(verify.run_wrf(str(wrf), str(d), 0.05).failed)

    def test_files_test_requires_every_domain_cppwrf_wrote(self):  # AUDA4 A4-13
        build, runs = self.root / "build", self.root / "runs"
        ref = runs / "run_N1_r8/steps"
        ref.mkdir(parents=True)
        made = self.root / "made"
        for dom in ("d01", "d02"):
            f = fixture(made / ("" if dom == "d01" else dom), (0,), domain=dom)
            if dom == "d01":
                shutil.copyfile(f, ref / f.name)
            else:
                d02 = f
        build.mkdir()
        (build / "cppwrf").write_text("""#!/usr/bin/env bash
for a in "$@"; do case "$a" in output.dir=*) out=${a#output.dir=};; esac; done
cp "$FAKE_MADE"/wrfout_d01_* "$out"/; mkdir -p "$out"/d02; cp "$FAKE_MADE"/d02/wrfout_d02_* "$out"/d02/
""")
        (build / "cppwrf").chmod(0o755)
        env = dict(os.environ, CPPWRF_WRF_RUNS=str(runs), FAKE_MADE=str(made), CPPWRF_SCRATCH=str(self.root))
        cmd = ["bash", ROOT / "tests/verify/run_wrf_files_test.sh", build, "n1"]
        r = subprocess.run(cmd, env=env, capture_output=True, text=True)  # the reference pack lacks wrfout_d02
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("CPPWRF wrote wrfout_d02, no WRF wrfout_d02", r.stdout)
        shutil.copyfile(d02, ref / d02.name)
        r = subprocess.run(cmd, env=env, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("PASS n1", r.stdout)

    def test_real_comparator_checks_bytes_and_attribute_types(self):  # AUDA4 A4-5
        wrf, cpp = self.root / "wrf.nc", self.root / "cpp.nc"
        cmd = [sys.executable, ROOT / "tests/init_real/compare_real.py", cpp, wrf]
        stamp = b"2026-10-12_00:00:00"

        def write(path, times=stamp + b"\0", flag=np.int32(1), farr=(1.0, 1.0), big=500):
            with Dataset(path, "w") as ds:
                ds.createDimension("Time", 1)
                ds.createDimension("DateStrLen", 20)
                ds.createDimension("x", 2)
                ds.createVariable("Times", "S1", ("Time", "DateStrLen"))[0] = np.frombuffer(times, "S1")
                u = ds.createVariable("U", "f8", ("Time", "x"))
                u[:] = 1
                u.setncattr("flag", flag)
                ds.setncattr("farr", np.array(farr))
                ds.setncattr("bigarr", np.array([big if i == 500 else i for i in range(1005)], np.int32))
        write(wrf)
        for mutation, kw in (("NUL moved in Times", {"times": b"\0" + stamp}),
                             ("attribute int32 vs int64", {"flag": np.int64(1)}),
                             ("float array attribute at 1e-10", {"farr": (1.0, 1.0 + 1e-10)}),
                             ("element 500 of a 1005-element attribute", {"big": -999})):
            with self.subTest(mutation=mutation):
                write(cpp)
                r = subprocess.run(cmd, capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                write(cpp, **kw)
                r = subprocess.run(cmd, capture_output=True, text=True)
                self.assertEqual(r.returncode, 1, r.stdout + r.stderr)

    def test_unit_comparators_reject_nan_and_empty_outputs(self):  # AUDA4 A4-11
        def rc(script, *args):
            return subprocess.run([sys.executable, ROOT / script, *map(str, args)], capture_output=True,
                                  text=True).returncode
        ncols, nz = 2, 3
        inp, wrf, cpp = self.root / "bmj_in.bin", self.root / "bmj_wrf.bin", self.root / "bmj_cpp.bin"
        np.array([ncols, nz], "<i4").tofile(inp)
        n = 2 * 134 + 134 * 76 + 5 * nz * ncols + 6 * ncols
        np.ones(n, "<f8").tofile(wrf)
        bmj = ("tests/phys/cu_bmj/compare.py", wrf, cpp, "--input", inp, "--tol")
        np.ones(n, "<f8").tofile(cpp)
        self.assertEqual(rc(*bmj, "1e30"), 0)
        np.full(n, np.nan, "<f8").tofile(cpp)  # an FP32 driver that wrote NaN everywhere
        self.assertEqual(rc(*bmj, "1e30"), 1)
        for f in (wrf, cpp):  # two empty outputs
            f.write_bytes(b"")
        self.assertEqual(rc(*bmj, "0"), 1)
        lake = ("tests/phys/sf_lake/compare.py", wrf, cpp)
        self.assertEqual(rc(*lake), 1)
        adv_in, adv_w, adv_c = self.root / "adv_in.bin", self.root / "adv_wrf.bin", self.root / "adv_cpp.bin"
        adv_in.write_bytes(np.array([1, 1, 1, 0, 1, 1, 1], "<i4").tobytes())
        np.array([1.0, 2.0, 0.0], "<f8").tofile(adv_w)
        np.array([1.0, 2.0, 0.0], "<f8").tofile(adv_c)
        adv = ("tests/dyn/advection/compare.py", adv_w, adv_c, adv_in, "t")
        self.assertEqual(rc(*adv), 0)
        np.array([1.0, 2.0, -0.0], "<f8").tofile(adv_c)  # WRF +0, CPPWRF -0
        self.assertEqual(rc(*adv), 1)

    def test_fp32_accuracy_fails_on_shape_nan_and_nothing_compared(self):  # AUDA4 A4-14
        tool = ROOT / "tools/verify/fp32_accuracy.py"

        def write(path, w_levels=3, t2=1.0):
            with Dataset(path, "w") as ds:
                ds.createDimension("Time", None)
                ds.createDimension("DateStrLen", 19)
                ds.createDimension("x", 2)
                ds.createDimension("z", w_levels)
                ds.createVariable("Times", "S1", ("Time", "DateStrLen"))[0] = np.frombuffer(START.encode(), "S1")
                ds.createVariable("T2", "f4", ("Time", "x"))[0] = [t2, 2.0]
                ds.createVariable("W", "f4", ("Time", "z", "x"))[0] = 0.0
        test, ref = self.root / f"wrfout_d01_{START}", self.root / "ref" / f"wrfout_d01_{START}"
        ref.parent.mkdir()
        write(ref)

        def run(*extra):
            return subprocess.run([sys.executable, tool, "compare", test, ref, *extra, "--out", self.root / "s.jsonl"],
                                  capture_output=True, text=True)
        write(test)
        self.assertEqual(run().returncode, 0)
        for mutation, kw, extra in (("W staggered differently", {"w_levels": 2}, ()),
                                    ("NaN in T2", {"t2": np.nan}, ()),
                                    ("no requested variable", {}, ("--vars", "NO_SUCH_VAR"))):
            with self.subTest(mutation=mutation):
                write(test, **kw)
                r = run(*extra)
                self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        r = subprocess.run([sys.executable, tool, "report", self.root / "s.jsonl"], capture_output=True, text=True)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("Not compared", r.stdout)

    def test_hash_compare_requires_requested_cases_and_whole_lines(self):  # AUDA4 A4-8
        import out_hashes
        ref, new = self.root / "ref.tsv", self.root / "new.tsv"
        line = "caseA/wrfout_d01\tU\tfloat64\t(1, 2)\tabc\n"
        ref.write_text(line)
        new.write_text(line)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(out_hashes.compare(str(ref), str(new), cases=["caseA"]), 0)
            self.assertEqual(out_hashes.compare(str(ref), str(new), cases=["caseA", "caseB_MISSING"]), 1)
            new.write_text(line + "caseA/wrfout_d01\tV\tfloat64\n")  # truncated line
            with self.assertRaisesRegex(ValueError, "malformed"):
                out_hashes.compare(str(ref), str(new))
            new.write_text(line + line.replace("abc", "abd"))  # two runs appended that disagree
            with self.assertRaisesRegex(ValueError, "hashed twice"):
                out_hashes.compare(str(ref), str(new))
            new.write_text(line + line)  # the same hash twice
            self.assertEqual(out_hashes.compare(str(ref), str(new)), 0)

    def test_golden_compare_checks_every_column_at_the_golden_precision(self):  # AUDA4 A4-9
        tool = ROOT / "tools/regression/compare_golden.py"
        run, gold = self.root / "run", self.root / "gold"

        def write(d, rows):
            d.mkdir(exist_ok=True)
            (d / "diag.csv").write_text("\n".join(rows) + "\n")
            self.snaps(d, (1,))
        write(gold, ["step,time,wall_s,max_abs_w,dry_mass", "1,60,0.1,1.2345678901234567,9.8765432109876543"])

        def rc():
            return subprocess.run([sys.executable, tool, run, gold, "float64"], capture_output=True,
                                  text=True).returncode
        write(run, ["step,time,wall_s,max_abs_w,dry_mass", "1,60,0.2,1.2345678901234567,9.8765432109876543"])
        self.assertEqual(rc(), 0)
        write(run, ["step,time,wall_s,max_abs_w", "1,60,0.2,1.2345678901234567"])  # dry_mass dropped
        self.assertEqual(rc(), 1)
        write(run, ["step,time,wall_s,max_abs_w,dry_mass", "1,60,0.2,1.2,9.9"])  # two digits, 3 % off
        self.assertEqual(rc(), 1)

    def test_matrix_resume_comparison_is_two_way(self):  # AUDA4 A4-7: the call of tools/gcp/matrix_variant.py
        import exact
        straight = fixture(self.root / "straight", (0, 60, 120, 180))
        with Dataset(straight, "a") as ds:
            ds.createVariable("DROPPED_VAR", "f8", ("Time", "x"))[:] = 1
        resumed = fixture(self.root / "resumed", (120,), start=wrf_suite.times_at(START, 60))
        old, n = exact.compare_stream([str(straight)], [str(resumed)], subset=True, global_atts=False)
        self.assertEqual((old, n), ({}, 1))  # the call before: the lost frame and variable passed
        new, _ = exact.compare_stream([str(resumed)], [str(straight)], after=wrf_suite.times_at(START, 60),
                                      global_atts=False)
        self.assertIn("(times)", new)
        self.assertIn("(wrf-only-vars)", new)
        src = (ROOT / "tools/gcp/matrix_variant.py").read_text()
        self.assertIn("exact.compare_stream(files, straight, after=date, global_atts=False, stats=stats)", src)
        self.assertNotIn("subset=True", src)

    def test_files_test_compares_the_resumed_restart_files(self):  # AUDA4 A4-13 (resumed leg)
        build, runs = self.root / "build", self.root / "runs"
        ref, rst, rsm = runs / "run_P1_r8/steps", runs / "rst_P1_r8", runs / "rsm_P1_r8"
        start60 = wrf_suite.times_at(START, 60)
        for d in (ref, rst):
            fixture(d, (0, 60, 120))
        (rst / "namelist.input").write_text("&time_control\n start_year=2026,\n start_month=10,\n start_day=12,\n"
                                            " start_hour=0,\n/\n")
        fixture(rst, (60,), kind="wrfrst", alarm=60)
        fixture(rsm, (120,), start=start60)
        made = fixture(self.root / "made", (120,), kind="wrfrst", start=start60)
        shutil.copyfile(made, rsm / made.name)  # the wrfrst WRF's resumed run wrote at minute 2
        (rsm / "namelist.input").write_text(RESUME_NAMELIST)
        build.mkdir()
        (build / "cppwrf").write_text("""#!/usr/bin/env bash
for a in "$@"; do case "$a" in output.dir=*) out=${a#output.dir=};; esac; done
if [[ "$out" == */resumed ]]; then
  cp "$FAKE_RSM"/wrfout_d01_* "$out"/
  [ -n "$FAKE_RESUMED_RST" ] && cp "$FAKE_RSM"/wrfrst_d01_* "$out"/
else
  cp "$FAKE_RST"/wrfout_d01_* "$FAKE_RST"/wrfrst_d01_* "$out"/
fi
exit 0
""")
        (build / "cppwrf").chmod(0o755)
        env = dict(os.environ, CPPWRF_WRF_RUNS=str(runs), FAKE_RSM=str(rsm), FAKE_RST=str(rst),
                   CPPWRF_SCRATCH=str(self.root), FAKE_RESUMED_RST="1")
        cmd = ["bash", ROOT / "tests/verify/run_wrf_files_test.sh", build, "p1"]
        r = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=ROOT)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        env["FAKE_RESUMED_RST"] = ""  # the resumed run wrote no restart file
        r = subprocess.run(cmd, env=env, capture_output=True, text=True, cwd=ROOT)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn(f"resumed wrfrst_d01_{wrf_suite.times_at(START, 120)} not written", r.stdout)

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
