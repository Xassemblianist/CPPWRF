# SPDX-License-Identifier: GPL-3.0-or-later
"""tools/fp32_double_rounding.py finds the decimals whose double-then-float rounding differs from one rounding."""

import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "fp32_double_rounding.py")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import fp32_double_rounding as fdr  # noqa: E402

# 1 + 2^-24 (the midpoint of 1 and 1 + 2^-23) plus 1e-35: binary64 rounds it to the midpoint, binary32 then to the
# even 1; one rounding gives 1 + 2^-23
UP = "1.00000005960464477539062500000000001"
# 1 + 3 * 2^-24 minus 1e-35: binary64 gives the midpoint, binary32 the even 1 + 2^-22; one rounding 1 + 2^-23
DOWN = "1.00000017881393432617187499999999999"
TIE = "1.000000059604644775390625"  # exactly the midpoint: both round to even


class Check(unittest.TestCase):
    def test_values(self):
        self.assertEqual(fdr.check(UP), (1.0, 1.0000001192092896))
        self.assertEqual(fdr.check(DOWN), (1.000000238418579, 1.0000001192092896))
        self.assertIsNone(fdr.check(TIE))
        for s in ("0.1", "9.81", "287.04", "1.e-5", "3.14159265358979323846", "1e-45", "1e39", "0"):
            self.assertIsNone(fdr.check(s), s)
        self.assertEqual(fdr.check(UP.replace("e", "") + "d0"), (1.0, 1.0000001192092896))

    def run_tool(self, *args):
        return subprocess.run([sys.executable, "-I", TOOL, *args], capture_output=True, text=True)

    def test_sources(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("CPPWRF_TEST_DIR")) as d:
            ok = os.path.join(d, "ok.cpp")
            with open(ok, "w") as fh:
                fh.write(f"const real a = real({TIE});\nconst float b = {UP}f;\n// real({UP})\n"
                         f"/* {DOWN} */ const char* s = \"{UP}\";\n"
                         f"const double c = {UP};  // fp32-double-rounding: double\n")
            r = self.run_tool(ok)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            bad = os.path.join(d, "bad.hpp")
            with open(bad, "w") as fh:
                fh.write(f"constexpr real t[2] = {{real(0.5), real({UP})}};\nx = rb({DOWN}) * y;\n")
            r = self.run_tool(bad)
            self.assertEqual(r.returncode, 1)
            self.assertIn("bad.hpp:1: " + UP, r.stdout)
            self.assertIn("bad.hpp:2: " + DOWN, r.stdout)

    def test_data(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("CPPWRF_TEST_DIR")) as d:
            tbl = os.path.join(d, "TABLE.TBL")
            with open(tbl, "w") as fh:
                fh.write("1, 0.15, 3*0.25, 1.5D-3\n")
            self.assertEqual(self.run_tool("--data", tbl).returncode, 0)
            with open(tbl, "a") as fh:
                fh.write(f"2, 2*{UP.replace('e', '')}D0\n")
            r = self.run_tool("--data", tbl)
            self.assertEqual(r.returncode, 1)
            self.assertIn("TABLE.TBL:2:", r.stdout)


if __name__ == "__main__":
    unittest.main()
