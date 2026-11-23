# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise exact.compare_stream with single edits to synthetic WRF output.

Run with python3 -I tests/verify/test_exact_mutations.py -v.
NetCDF fixtures are temporary. Set CPPWRF_MUTATION_SCRATCH to override ~/cppwrf_scratch/cdx9_tmp.
"""

import os
from pathlib import Path
import runpy
import shutil
import tempfile
import unittest

from netCDF4 import Dataset
import numpy as np


# Load the sibling by path because -I excludes the script directory from sys.path.
compare_stream = runpy.run_path(str(Path(__file__).with_name("exact.py")))["compare_stream"]
SCRATCH = Path(os.environ.get("CPPWRF_MUTATION_SCRATCH", Path.home() / "cppwrf_scratch" / "cdx9_tmp"))
FILL = -9999.0


def timestamp(minute):
    return np.frombuffer(f"2026-01-01_00:{minute:02d}:00".encode("ascii"), dtype="S1")


def write_output(path, minutes, *, f32_type="f4", spare_length=1, fill_value=FILL,
                 times_only=False, omit_integer=False, add_extra=False):
    with Dataset(path, "w") as ds:
        ds.createDimension("Time", None)
        ds.createDimension("DateStrLen", 19)
        times = ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
        for row, minute in enumerate(minutes):
            times[row] = timestamp(minute)
        if times_only:
            return

        ds.setncattr("RUN_ID", "control")
        ds.createDimension("west_east", 4)
        # A length change here tests dimension metadata with all variable shapes intact.
        ds.createDimension("unused", spare_length)
        frame = ("Time", "west_east")
        f32 = ds.createVariable("F32", f32_type, frame)
        f32.units = "K"
        f64 = ds.createVariable("F64", "f8", frame, fill_value=fill_value)
        if not omit_integer:
            ds.createVariable("I64", "i8", frame)
        ds.createVariable("STATIC", "f4", ("west_east",))[:] = [1, 2, 3, 4]
        if add_extra:
            ds.createVariable("EXTRA", "i4", ())[:] = 42
        for row, minute in enumerate(minutes):
            f32[row] = [1.0, 0.0, np.nan, minute]
            f64[row] = [1.0, 2.0, FILL, minute]
            if not omit_integer:
                ds["I64"][row] = [2**53, 2**53 + 3, -5, minute]


class ExactMutations(unittest.TestCase):
    def setUp(self):
        SCRATCH.mkdir(parents=True, exist_ok=True)
        temp = tempfile.TemporaryDirectory(prefix="exact-mutations-", dir=SCRATCH)
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.wrf = [root / "wrf-0.nc", root / "wrf-1.nc"]
        self.cpp = [root / "cpp-0.nc", root / "cpp-1.nc"]
        for reference, candidate, minutes in zip(self.wrf, self.cpp, ((0,), (1, 2))):
            write_output(reference, minutes)
            shutil.copyfile(reference, candidate)
        self.second = self.cpp[1]

    def set_value(self, variable, index, value):
        with Dataset(self.second, "r+") as ds:
            ds[variable][index] = value

    def assert_differs(self, key, diagnostic, *, frames=3):
        differences, compared_frames = compare_stream(self.cpp, self.wrf)
        self.assertEqual(compared_frames, frames)
        self.assertTrue(differences, "IDENTICAL for a mutated stream")
        self.assertIn(key, differences)
        messages = differences[key]
        if isinstance(messages, str):
            messages = [messages]
        self.assertTrue(any(diagnostic in message for message in messages),
                        f"Expected {diagnostic!r} in {messages!r}")

    def test_untouched_copy(self):
        stats = {}
        differences, frames = compare_stream(self.cpp, self.wrf, stats=stats)
        self.assertEqual(differences, {}, f"DIFFER for an untouched copy: {differences}")
        self.assertEqual(frames, 3)
        self.assertEqual(stats["files"], 2)
        self.assertEqual(stats["vars"], 4)
        self.assertEqual(stats["static"], 1)
        self.assertEqual(stats["nan_by_var"], {"F32": 3})

    def test_float32_one_ulp(self):
        self.set_value("F32", (0, 0), np.nextafter(np.float32(1), np.float32(np.inf)))
        self.assert_differs("F32", "1/4 max|d|")

    def test_float64_one_ulp(self):
        self.set_value("F64", (0, 0), np.nextafter(np.float64(1), np.float64(np.inf)))
        self.assert_differs("F64", "1/4 max|d|")

    def test_signed_zero(self):
        self.set_value("F32", (0, 1), -0.0)
        self.assert_differs("F32", "signed-zero 1")

    def test_nan_vs_value(self):
        self.set_value("F32", (0, 2), 5.0)
        self.assert_differs("F32", "nan-vs-number 1")

    def test_nan_count_change(self):
        # Keep the existing NaN and add one next to it in the same frame.
        self.set_value("F32", (0, 1), np.nan)
        self.assert_differs("F32", "nan-vs-number 1")

    def test_variable_without_time_changed(self):
        self.set_value("STATIC", 0, 2.0)
        self.assert_differs("STATIC", "(no Time dimension)")

    def test_integer_changed(self):
        self.set_value("I64", (0, 0), 2**53 + 1)
        self.assert_differs("I64", "1/4 integers differ")

    def test_dtype_changed(self):
        write_output(self.second, (1, 2), f32_type="f8")
        self.assert_differs("F32", "type cppwrf float64 wrf float32")

    def test_variable_missing(self):
        write_output(self.second, (1, 2), omit_integer=True)
        self.assert_differs("(wrf-only-vars)", "I64")

    def test_extra_variable(self):
        write_output(self.second, (1, 2), add_extra=True)
        self.assert_differs("(cppwrf-only-vars)", "EXTRA")

    def test_frame_missing(self):
        write_output(self.second, (1,))
        self.assert_differs("(times)", "only-wrf ['2026-01-01_00:02:00']", frames=2)

    def test_extra_frame(self):
        write_output(self.second, (1, 2, 3))
        self.assert_differs("(times)", "only-cppwrf ['2026-01-01_00:03:00']")

    def test_times_string_changed(self):
        self.set_value("Times", 1, timestamp(3))
        self.assert_differs("(times)", "only-cppwrf ['2026-01-01_00:03:00']", frames=2)

    def test_global_attribute_changed_in_second_file(self):
        with Dataset(self.second, "r+") as ds:
            ds.setncattr("RUN_ID", "mutated")
        self.assert_differs("(metadata)", "global attributes {'RUN_ID'")

    def test_variable_attribute_changed(self):
        with Dataset(self.second, "r+") as ds:
            ds["F32"].units = "degC"
        self.assert_differs("(metadata)", "F32: attributes")

    def test_dimension_length_changed(self):
        write_output(self.second, (1, 2), spare_length=2)
        self.assert_differs("(metadata)", "('unused', 2)")

    def test_fill_value_changed(self):
        write_output(self.second, (1, 2), fill_value=-8888.0)
        self.assert_differs("(metadata)", "_FillValue")

    def test_zero_frames_rejected(self):
        write_output(self.wrf[1], ())
        shutil.copyfile(self.wrf[1], self.second)
        differences, frames = compare_stream(self.cpp[1:], self.wrf[1:])
        self.assertEqual(frames, 0)
        self.assertTrue(differences, "IDENTICAL with zero frames")

    def test_zero_data_variables_rejected(self):
        write_output(self.wrf[1], (1,), times_only=True)
        shutil.copyfile(self.wrf[1], self.second)
        stats = {}
        differences, frames = compare_stream(self.cpp[1:], self.wrf[1:], stats=stats)
        self.assertEqual(frames, 1)
        self.assertEqual(stats["vars"], 0)
        self.assertTrue(differences, "IDENTICAL with zero data variables")

    def test_clock_only_reference_rejected(self):
        write_output(self.wrf[1], (1,), times_only=True)
        with Dataset(self.wrf[1], "r+") as ds:
            ds.createVariable("XTIME", "f8", ("Time",))[0] = 1
        shutil.copyfile(self.wrf[1], self.second)
        differences, _ = compare_stream(self.cpp[1:], self.wrf[1:], subset=True)
        self.assertIn("(variables)", differences)

    def test_until_bounds_comparison_and_counts(self):
        stats = {}
        differences, frames = compare_stream(self.cpp, self.wrf, until="2026-01-01_00:01:00",
                                             subset=True, stats=stats)
        self.assertFalse(differences)
        self.assertEqual(frames, 2)
        self.assertEqual(stats["frames"], stats["ref_frames"])
        self.assertLessEqual(stats["vars"], stats["ref_vars"])

    def test_shared_frames_after_bound_still_compared(self):
        self.set_value("F32", (1, 0), 99)
        stats = {}
        differences, frames = compare_stream(self.cpp, self.wrf, until="2026-01-01_00:01:00",
                                             subset=True, stats=stats)
        self.assertIn("F32", differences)
        self.assertEqual(frames, 2)
        self.assertEqual(stats["frames"], stats["ref_frames"])

    def test_cppwrf_output_cannot_masquerade_as_wrf_reference(self):
        with Dataset(self.second, "r+") as ds:
            ds.TITLE = " OUTPUT FROM CPPWRF V4.8.0 MODEL"
        differences, _ = compare_stream(self.cpp, [self.wrf[0], self.second], wrf_reference=True)
        self.assertIn("(reference)", differences)

    def test_duplicate_in_separate_file_cannot_hide_earlier_frame(self):
        shadowed = self.wrf[0].with_name("wrf--shadowed.nc")
        write_output(shadowed, (2,))
        with Dataset(shadowed, "r+") as ds:
            ds["F32"][0, 0] = 99
        differences, _ = compare_stream(self.cpp, [shadowed, *self.wrf], subset=True)
        self.assertIn("(records)", differences)

    def test_duplicate_in_both_streams_rejected(self):
        write_output(self.second, (1, 2, 2))
        shutil.copyfile(self.second, self.wrf[1])
        differences, _ = compare_stream(self.cpp, self.wrf)
        self.assertIn("(records)", differences)

    def test_continuation_can_overlap_first_leg(self):
        continuation = self.second.with_name("continuation.nc")
        write_output(continuation, (2,))
        differences, _ = compare_stream(self.cpp, self.wrf, continuation=[continuation],
                                         split=timestamp(1).tobytes().decode())
        self.assertFalse(differences)

    def test_extra_frame_with_duplicate_time(self):
        write_output(self.second, (1, 2, 2))
        differences, _ = compare_stream(self.cpp, self.wrf)
        self.assertTrue(differences, "IDENTICAL with one extra physical frame at an existing timestamp")

    def test_missing_frame_with_duplicate_time(self):
        write_output(self.wrf[1], (1, 2, 2))
        differences, _ = compare_stream(self.cpp, self.wrf)
        self.assertTrue(differences, "IDENTICAL with one missing physical frame at an existing timestamp")


if __name__ == "__main__":
    unittest.main()
