# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf.namelist: parsing, writing, translation and the per-option check."""
import os
import unittest

from fixtures import NAMELIST, run_dir

import cppwrf
from cppwrf import namelist as nlm


class Namelist(unittest.TestCase):
    def test_round_trip(self):
        nml = nlm.parse(NAMELIST)
        self.assertEqual(nlm.get(nml, "domains", "e_we"), 11)
        self.assertIs(nlm.get(nml, "time_control", "restart"), False)
        again = nlm.parse(nlm.dumps(nml))
        self.assertEqual(again, nml)

    def test_write_read(self):
        d = run_dir()
        nml = nlm.read(os.path.join(d, "namelist.input"))
        nml["physics"]["mp_physics"] = [6]
        nlm.write(nml, os.path.join(d, "namelist.input"))
        self.assertEqual(nlm.get(nlm.read(os.path.join(d, "namelist.input")), "physics", "mp_physics"), 6)

    def test_dates_and_names(self):
        nml = nlm.parse(NAMELIST)
        self.assertEqual(nlm.start_date(nml), "2026-10-12_00:00:00")
        self.assertEqual(nlm.stream_filename("wrfout_d<domain>_<date>", 2, "2026-10-12_06:00:00"),
                         "wrfout_d02_2026-10-12_06:00:00")
        n2, on = nlm.without_restart(nlm.parse(NAMELIST.replace("restart = .false.", "restart = .true.")))
        self.assertTrue(on)
        self.assertIs(nlm.get(n2, "time_control", "restart"), False)

    def test_translate(self):
        d = run_dir()
        tr = nlm.translate(os.path.join(d, "namelist.input"))
        self.assertTrue(tr.ok, tr.errors)
        self.assertEqual(tr.sections["run"]["case"], "em_quarter_ss")
        self.assertIn("[run]", tr.ini)

    def test_check(self):
        d = run_dir(NAMELIST.replace("mp_physics = 1", "mp_physics = 3").replace("dx = 2000", "dx = 2000, foo = 1"))
        res = nlm.check(os.path.join(d, "namelist.input"))
        self.assertFalse(res.ok)
        bad = {o.option: o for o in res.by_status(nlm.NOT_PORTED)}
        self.assertIn("mp_physics", bad)
        self.assertIn("use one of", bad["mp_physics"].suggestion)
        self.assertIn("foo", bad)
        self.assertIn("spelling", bad["foo"].suggestion)
        ok = nlm.check(os.path.join(run_dir(), "namelist.input"))
        self.assertTrue(ok.ok)
        self.assertIn("CPPWRF runs this namelist", nlm.format_check(ok))

    def test_restart_is_ported_for_dropin(self):
        d = run_dir(NAMELIST.replace("restart = .false.", "restart = .true."))
        res = nlm.check(os.path.join(d, "namelist.input"))
        self.assertTrue(res.ok, res.translation.errors)
        st = next(o for o in res.options if o.option == "restart")
        self.assertEqual(st.status, nlm.PORTED)
        self.assertFalse(nlm.check(os.path.join(d, "namelist.input"), dropin=False).ok)

    def test_version_is_cmake_version(self):
        import re
        cm = os.path.join(os.path.dirname(__file__), "..", "..", "CMakeLists.txt")
        if not os.path.exists(cm):
            self.skipTest("not in the source tree")
        with open(cm) as f:
            v = re.search(r"project\(CPPWRF VERSION ([0-9.]+)", f.read()).group(1)
        self.assertEqual(cppwrf.__version__, v)


if __name__ == "__main__":
    unittest.main()
