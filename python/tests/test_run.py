# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.Run with a stand-in binary (fixtures.FAKE): progress from the timing lines, the logs, failures, cancel."""
import os
import unittest

from fixtures import NAMELIST, fake_binary, run_dir

import cppwrf


class RunTest(unittest.TestCase):
    def test_run_progress(self):
        d = run_dir()
        exe = fake_binary(d)
        r = cppwrf.Run(d, exe=exe, threads=2)
        cmd = r.prepare()
        self.assertEqual(cmd[0], exe)
        self.assertEqual(r.mode, "cpu-fp64")
        self.assertEqual((r.start_date, r.end_date), ("2026-10-12_00:00:00", "2026-10-12_00:01:00"))
        r.start()
        r.wait(timeout=60)
        p = r.progress()
        self.assertEqual(p.model_time, "2026-10-12_00:01:00")
        self.assertEqual(p.steps, 5)
        self.assertAlmostEqual(p.fraction, 1.0)
        self.assertEqual(p.files, ["wrfout_d01_2026-10-12_00:00:00"])
        self.assertEqual(r.returncode, 0)
        out = open(os.path.join(d, "rsl.out.0000")).read()
        self.assertTrue(out.rstrip().endswith("d01 2026-10-12_00:01:00 wrf: SUCCESS COMPLETE WRF"))
        self.assertEqual(r.wrfout(), [os.path.join(d, "wrfout_d01_2026-10-12_00:00:00")])

    def test_namelist_path(self):
        d = run_dir()
        r = cppwrf.Run(os.path.join(d, "namelist.input"), exe=fake_binary(d))
        self.assertEqual(r.directory, d)

    def test_failure(self):
        d = run_dir()
        os.environ["FAKE_FAIL"] = "1"
        try:
            with self.assertRaises(cppwrf.RunFailed) as c:
                cppwrf.Run(d, exe=fake_binary(d)).run(timeout=60)
        finally:
            del os.environ["FAKE_FAIL"]
        self.assertEqual(c.exception.status, 3)
        self.assertIn("something went wrong", str(c.exception))
        self.assertIn("FATAL CALLED", open(os.path.join(d, "rsl.error.0000")).read())

    def test_cancel(self):
        d = run_dir()
        os.environ["FAKE_SLEEP"] = "30"
        try:
            r = cppwrf.Run(d, exe=fake_binary(d)).start()
            self.assertTrue(r.running)
            r.cancel(timeout=5)
            with self.assertRaises(cppwrf.RunFailed):
                r.wait(timeout=30)
        finally:
            del os.environ["FAKE_SLEEP"]
        self.assertFalse(r.running)

    def test_namelist_error(self):
        d = run_dir(NAMELIST.replace("mp_physics = 1", "mp_physics = 3"))
        with self.assertRaises(cppwrf.NamelistError) as c:
            cppwrf.Run(d, exe=fake_binary(d)).prepare()
        self.assertIn("mp_physics", str(c.exception))
        self.assertFalse(c.exception.check.ok)

    def test_wrong_mode(self):
        d = run_dir()
        with self.assertRaises(cppwrf.CppwrfError) as c:
            cppwrf.Run(d, mode="gpu-fp32", exe=fake_binary(d)).prepare()
        self.assertIn("gpu-fp32", str(c.exception))


if __name__ == "__main__":
    unittest.main()
