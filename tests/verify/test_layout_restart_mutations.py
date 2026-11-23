# SPDX-License-Identifier: GPL-3.0-or-later
"""Mutation tests for the layout restart check using NetCDF files and a stub launch."""

import os
from pathlib import Path
import subprocess
from unittest.mock import patch
import unittest

from netCDF4 import Dataset

import wrf_suite
from test_harness_mutations import ScratchMutationCase, fixture


def native_fixture(directory, step, missing_halo=None):
    directory.mkdir(parents=True, exist_ok=True)
    with Dataset(directory / f"restart_{step:06d}.nc", "w") as ds:
        ds.createDimension("Time", 1)
        ds.createDimension("halo_2", 2)
        fields = [f"{name}_{level}" for name in ("u", "v", "w", "t", "ph", "mu") for level in (1, 2)]
        fields += ["p", "al", "ww", "ww_m", "php", "alt", "q2_qv"]
        for name in fields:
            ds.createVariable(name, "f8", ("Time",))[0] = step
            if name != missing_halo:
                ds.createVariable("HALO_" + name, "f8", ("halo_2",))[:] = [step, -step]
        # Carried integers are stored as reals and do not have a companion halo.
        ds.createVariable("sfc_isltyp", "f8", ("Time",))[0] = 1


class LayoutRestartMutations(ScratchMutationCase):
    def check_layout(self, *, nested=False, mutation=None):
        cfg = self.root / "layout.ini"
        cfg.write_text("[run]\ncase = stub\ntiles = 2x2\nrun_time = 60\n[time]\ndt = 60\n"
                       "[restart]\ninterval = 60\n"
                       + ("[domains]\nmax_dom = 2\nparent_time_step_ratio = 3\n" if nested else ""))
        domains = ("d01", "d02") if nested else ("d01",)
        refs = {dom: fixture(self.root / "ref", (0,), domain=dom) for dom in domains}
        root_in_subdir = nested and mutation not in ("mixed root", "nested native")
        launches = []

        def launch(cmd, *, cwd, env, capture_output, text):
            sets = dict(arg.split("=", 1) for arg in cmd[3::2])
            output = Path(sets["output.dir"])
            side = output.name
            launches.append((sets["run.tiles"], env))
            self.assertEqual(float(sets["run.run_time"]), 300)
            self.assertEqual(float(sets["restart.interval"]), 120)
            for dom in domains:
                directory = output / dom if root_in_subdir or dom != "d01" else output
                dt = 60 if dom == "d01" else 20
                for tick in (120, 240, 300):
                    if tick == 240 and mutation == "shared tick":
                        continue
                    omit = side == "b_tiled" and dom == "d02" and tick == 240 and mutation == "nested native"
                    if not omit:
                        missing = None
                        if dom == "d01" and tick == 240:
                            missing = {"core halo": "mu_2", "carried halo": "q2_qv"}.get(mutation)
                        native_fixture(directory, tick // dt, missing_halo=missing)
                    if tick != 300:
                        fixture(directory, (tick,), kind="wrfrst", domain=dom)
            effective = env.get("CPPWRF_TILES", sets["run.tiles"])
            if mutation == "wrong layout" and side == "b_tiled":
                effective = "1"
            log = "cppwrf: case=stub grid=12x12x2 dx=1 dt=60 n_sound=1 steps=5 backend=cpu precision=fp64\n"
            if effective not in ("1", "1x1"):
                px, py = effective.split("x")
                log = f"cppwrf: {px} x {py} tiles, 1 OpenMP threads each\n" + log
            return subprocess.CompletedProcess(cmd, 0, log, "")

        with patch.object(wrf_suite.subprocess, "run", side_effect=launch):
            bad, summary = wrf_suite.layout_restart("stub", cfg, ["run.tiles=3x1"], refs, 300, "2x2", 120,
                                                    self.root / "output")
        return bad, summary, launches

    def test_inventory_for_flat_nested_and_mixed_output(self):
        for nested, mutation in ((False, None), (True, None), (True, "mixed root")):
            with self.subTest(nested=nested, mutation=mutation):
                bad, summary, _ = self.check_layout(nested=nested, mutation=mutation)
                self.assertFalse(bad, bad)
                self.assertIn(f"{6 if nested else 3} restart_<step>.nc", summary)
                self.assertIn(f"{4 if nested else 2} wrfrst files", summary)

    def test_shared_omission_of_a_scheduled_tick(self):
        bad, _, _ = self.check_layout(mutation="shared tick")
        for side in ("a_one", "b_tiled"):
            self.assertIn(f"{side} missing d01/restart_000004.nc", bad)
            self.assertIn(f"{side} missing d01/wrfrst_d01_2026-10-12_00:04:00", bad)

    def test_native_restart_in_child_directory_on_one_side_only(self):
        bad, _, _ = self.check_layout(nested=True, mutation="nested native")
        self.assertEqual(bad, ["b_tiled missing d02/restart_000012.nc"])

    def test_each_native_file_requires_expected_halos_on_both_sides(self):
        for mutation, field in (("core halo", "mu_2"), ("carried halo", "q2_qv")):
            with self.subTest(mutation=mutation):
                bad, _, _ = self.check_layout(mutation=mutation)
                for side in ("a_one", "b_tiled"):
                    self.assertIn(f"{side} d01/restart_000004.nc: missing HALO_{field}", bad)

    def test_inherited_layout_environment_is_removed_from_both_launches(self):
        names = ("CPPWRF_TILES", "CPPWRF_TILE_CHECK", "CPPWRF_TILE_DEVICES", "CPPWRF_RANKS")
        with patch.dict(os.environ, dict.fromkeys(names, "1")):
            bad, _, launches = self.check_layout()
            self.assertEqual(os.environ["CPPWRF_TILES"], "1")
        self.assertFalse(bad, bad)
        self.assertEqual([layout for layout, _ in launches], ["1x1", "2x2"])
        for _, env in launches:
            self.assertFalse(set(names) & env.keys())

    def test_identical_files_cannot_pass_when_both_launches_use_one_patch(self):
        with patch.dict(os.environ, {"CPPWRF_TILES": "1"}):
            bad, _, _ = self.check_layout(mutation="wrong layout")
        self.assertEqual(bad, ["b_tiled: expected effective layout 2x2, log shows 1x1"])


if __name__ == "__main__":
    unittest.main()
