#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for tools/wrf2cppwrf.py namelist translator.

Tests:
1. Pure Python Fortran namelist tokenizer and parser.
2. Translation and key-by-key comparison against CPPWRF reference configs.
3. Multiple domains detection and warning emission.
4. Error detection on unsupported physics/dynamics options.
5. 2-step model execution under tools/heavy.sh.

The tests of WRF reference runs read ${CPPWRF_WRF_RUNS} and ${CPPWRF_WRF_REF} (defaults as tests/verify/wrf_suite.py:
~/wrf_real480, ~/wrf_ref480) and skip when the files are missing; the model runs also need $CPPWRF_BIN.
"""

import os
import re
import subprocess
import sys
import tempfile
import unittest

# set in the environment: the configurations and the model read ${CPPWRF_WRF_RUNS}
os.environ.setdefault("CPPWRF_WRF_RUNS", os.path.expanduser("~/wrf_real480"))
os.environ.setdefault("CPPWRF_WRF_REF", os.path.expanduser("~/wrf_ref480"))


def wrf_runs_path(rel: str) -> str:
    """Path of a WRF reference run file under ${CPPWRF_WRF_RUNS}."""
    return os.path.join(os.environ["CPPWRF_WRF_RUNS"], rel)


def wrf_ref_path(rel: str) -> str:
    """Path of a WRF ideal-case run file under ${CPPWRF_WRF_REF}."""
    return os.path.join(os.environ["CPPWRF_WRF_REF"], rel)

from typing import Any, Dict, List, Optional

# Add workspace to path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO_ROOT)

from tools.wrf2cppwrf import parse_namelist, WrfTranslator, NamelistParseError


CORE_PHYSICS = ("mp_physics", "cu_physics", "ra_lw_physics", "ra_sw_physics", "bl_pbl_physics", "sf_sfclay_physics",
                "sf_surface_physics")


def complete(text: str) -> str:
    """The options a test namelist leaves out and wrf.exe needs: without a physics_suite the seven physics options
    (Registry default -1: wrf.exe stops), diff_opt and km_opt (check_a_mundo stops without them), dx / dy (Registry
    default 200 m: the real case's time-step check and input_wrf's DX check stop wrf.exe). They get WRF's no-physics,
    no-diffusion values, every domain's entry, and 15 km."""
    m = re.search(r"\bmax_dom\s*=\s*(\d+)", text)
    n = int(m.group(1)) if m else 1
    adds = {"physics": [] if "physics_suite" in text else [f"{k} = {n}*0" for k in CORE_PHYSICS
                                                            if not re.search(rf"\b{k}\s*=", text)],
            "dynamics": [f"{k} = {v}" for k, v in (("diff_opt", 0), ("km_opt", 1)) if not re.search(rf"\b{k}\s*=", text)],
            "domains": [f"{k} = 15000" for k in ("dx", "dy") if not re.search(rf"\b{k}\s*=", text)]}
    for grp, items in adds.items():
        if not items:
            continue
        g = re.search(rf"&{grp}\b", text, re.I)
        if g:
            text = text[:g.end()] + " " + ", ".join(items) + "," + text[g.end():]
        else:
            text += f"\n&{grp} " + ", ".join(items) + " /\n"
    return text


def real_case(text: str) -> str:
    """A real-case namelist: domain 1 needs specified = .true. (Registry default .false.; with no condition on a
    side wrf.exe stops in share/module_bc.F boundary_condition_check, and the translator asks for --case); the
    options complete() adds."""
    return complete(text) + "\n&bdy_control specified = .true. /\n"


# real-case keys of the translation that wrf.exe checks against the input file: (namelist group, key, Registry
# default; None: written only when the namelist has it)
REAL_INPUT_CHECKS = {
    ("real", "p_top_requested"): ("domains", "p_top_requested", 5000.0),
    ("real", "iso_temp"): ("dynamics", "iso_temp", 200.0),
    ("real", "dx"): ("domains", "dx", 200.0),
    ("real", "dy"): ("domains", "dy", 200.0),
    ("dyn", "hybrid_opt"): ("dynamics", "hybrid_opt", 2),
    ("real", "e_we"): ("domains", "e_we", 32),
    ("real", "e_sn"): ("domains", "e_sn", 32),
    ("real", "e_vert"): ("domains", "e_vert", 31),
    ("real", "num_land_cat"): ("physics", "num_land_cat", 21),
}


def load_ini_dict(path_or_text: str, is_text: bool = False) -> dict:
    """Parse INI file or text into a dict of {section: {key: value}}."""
    sections = {}
    cur_sec = None
    if is_text:
        lines = path_or_text.splitlines()
    else:
        with open(path_or_text) as f:
            lines = f.readlines()
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            cur_sec = line[1:-1]
            sections[cur_sec] = {}
        elif "=" in line and cur_sec:
            k, v = line.split("=", 1)
            sections[cur_sec][k.strip()] = v.strip()
    return sections


class TestNamelistParser(unittest.TestCase):
    """Test suite for the f90nml-free Fortran namelist parser."""

    def test_basic_parsing(self):
        text = """
        &time_control
         run_days = 0,
         run_hours = 1,
         restart = .false.,
         debug_level = 0
        /
        &domains
         time_step = 60,
         dx = 15000,
         dy = 15000.0,
        /
        """
        nml = parse_namelist(text)
        self.assertIn("time_control", nml)
        self.assertIn("domains", nml)
        self.assertEqual(nml["time_control"]["run_days"], [0])
        self.assertEqual(nml["time_control"]["run_hours"], [1])
        self.assertEqual(nml["time_control"]["restart"], [False])
        self.assertEqual(nml["domains"]["time_step"], [60])
        self.assertEqual(nml["domains"]["dx"], [15000])
        self.assertEqual(nml["domains"]["dy"], [15000.0])

    def test_comments_and_whitespace(self):
        text = """
        ! Top-level comment
        &time_control ! inline comment
         run_days = 2, ! days
         ! full line comment in group
         run_hours = 12
        /
        """
        nml = parse_namelist(text)
        self.assertEqual(nml["time_control"]["run_days"], [2])
        self.assertEqual(nml["time_control"]["run_hours"], [12])

    def test_string_literals(self):
        text = '''
        &io
         filename = 'my_file.nc',
         msg = "Hello, world!",
         nested_quotes = 'It''s fine'
        /
        '''
        nml = parse_namelist(text)
        self.assertEqual(nml["io"]["filename"], ["my_file.nc"])
        self.assertEqual(nml["io"]["msg"], ["Hello, world!"])
        self.assertEqual(nml["io"]["nested_quotes"], ["It's fine"])

    def test_booleans(self):
        text = """
        &flags
         b1 = .true.,
         b2 = .FALSE.,
         b3 = .t.,
         b4 = .f.,
         b5 = T,
         b6 = F
        /
        """
        nml = parse_namelist(text)
        grp = nml["flags"]
        self.assertEqual(grp["b1"], [True])
        self.assertEqual(grp["b2"], [False])
        self.assertEqual(grp["b3"], [True])
        self.assertEqual(grp["b4"], [False])
        self.assertEqual(grp["b5"], [True])
        self.assertEqual(grp["b6"], [False])

    def test_repetition_syntax(self):
        text = """
        &dynamics
         diff_opt = 3*0,
         factors = 2*0.12, 1.0
        /
        """
        nml = parse_namelist(text)
        self.assertEqual(nml["dynamics"]["diff_opt"], [0, 0, 0])
        self.assertEqual(nml["dynamics"]["factors"], [0.12, 0.12, 1.0])

    def test_scientific_notation_and_fortran_d(self):
        text = """
        &floats
         a = 1.0d-4,
         b = 2.5D+02,
         c = 5000.,
         d = .12,
         e = -1.5e-3
        /
        """
        nml = parse_namelist(text)
        grp = nml["floats"]
        self.assertAlmostEqual(grp["a"][0], 1.0e-4)
        self.assertAlmostEqual(grp["b"][0], 250.0)
        self.assertAlmostEqual(grp["c"][0], 5000.0)
        self.assertAlmostEqual(grp["d"][0], 0.12)
        self.assertAlmostEqual(grp["e"][0], -1.5e-3)

    def test_multiple_domains_translation(self):
        text = """
        &time_control
         run_hours = 1,
         history_interval = 10, 10, 10,
         input_from_file = .true., .true., .true.
        /
        &domains
         time_step = 60,
         max_dom = 3,
         e_we = 120, 121, 91,
         e_sn = 90, 91, 61,
         e_vert = 45, 45, 45,
         dx = 15000, 5000, 1000,
         dy = 15000, 5000, 1000,
         parent_id = 0, 1, 2,
         i_parent_start = 1, 40, 30,
         j_parent_start = 1, 30, 20,
         parent_grid_ratio = 1, 3, 5,
         parent_time_step_ratio = 1, 3, 5,
         feedback = 1,
         smooth_option = 2
        /
        &dynamics
         khdif = 0, 0, 10,
         kvdif = 0, 0, 5
        /
        &bdy_control
         specified = .true., .false., .false.,
         nested = .false., .true., .true.
        /
        """
        nml = parse_namelist(complete(text))
        translator = WrfTranslator(nml)
        gen = translator.translate()
        self.assertEqual(translator.errors, [])
        self.assertEqual(translator.warnings, [])
        self.assertEqual(gen["domains"]["max_dom"], 3)
        self.assertEqual(gen["domains"]["feedback"], 1)
        self.assertEqual(gen["domains"]["smooth_option"], 2)
        self.assertEqual(gen["domains"]["i_parent_start"], 40)
        self.assertEqual(gen["domains"]["j_parent_start"], 30)
        self.assertEqual(gen["domains"]["parent_grid_ratio"], 3)
        self.assertEqual(gen["domains"]["parent_time_step_ratio"], 3)
        self.assertEqual(gen["d02"]["wrfinput"], "wrfinput_d02")
        # no [d0N] dt: the model takes the parent's dt / parent_time_step_ratio (20 s, 4 s) as WRF does and
        # refuses another value (src/cases/nest_driver.cpp); the history intervals below are in those steps
        self.assertNotIn("dt", gen["d02"])
        self.assertEqual(gen["d02"]["out_interval"], 30)
        self.assertEqual(gen["d02"]["diag_interval"], 30)
        self.assertEqual(gen["d03"]["parent_id"], 2)
        self.assertEqual(gen["d03"]["i_parent_start"], 30)
        self.assertEqual(gen["d03"]["j_parent_start"], 20)
        self.assertEqual(gen["d03"]["parent_grid_ratio"], 5)
        self.assertEqual(gen["d03"]["parent_time_step_ratio"], 5)
        self.assertEqual(gen["d03"]["wrfinput"], "wrfinput_d03")
        self.assertNotIn("dt", gen["d03"])
        self.assertEqual(gen["d03"]["out_interval"], 150)
        self.assertEqual(gen["d03"]["diag_interval"], 150)
        self.assertEqual(gen["d03"]["khdif"], 10)
        self.assertEqual(gen["d03"]["kvdif"], 5)


class TestRealCasesReproduction(unittest.TestCase):
    """Test suite comparing generated INI against configs/real_*.ini key by key."""

    def _compare_configs(self, nml_rel_path: str, ref_ini_rel_path: str,
                          ignore_keys: Optional[dict] = None, sections: Optional[tuple] = None):
        nml_path = wrf_runs_path(nml_rel_path)
        ref_path = os.path.join(REPO_ROOT, ref_ini_rel_path)
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")
        if not os.path.exists(ref_path):
            self.skipTest(f"Reference INI not found: {ref_path}")

        with open(nml_path) as f:
            nml = parse_namelist(f.read())
        translator = WrfTranslator(nml, namelist_path=nml_path)
        gen = translator.translate()
        ref = load_ini_dict(ref_path)
        self.assertEqual(translator.errors, [])

        ignore_keys = {s: set(keys) for s, keys in (ignore_keys or {}).items()}
        if gen["run"]["case"] == "real":
            # namelist values the model checks against wrfinput_d01 as wrf.exe does (start_em.F, input_wrf.F stop
            # when they differ): the namelist's value or the Registry default; the hand-written configs leave them
            # to the input file
            for (sec, key), (grp, nkey, default) in REAL_INPUT_CHECKS.items():
                nvals = nml.get(grp, {}).get(nkey)
                if not nvals and default is None:
                    self.assertNotIn(key, gen.get(sec, {}), f"[{sec}].{key}")
                    continue
                self.assertEqual(float(gen[sec][key]), float(nvals[0] if nvals else default), f"[{sec}].{key}")
                if key not in ref.get(sec, {}):
                    ignore_keys.setdefault(sec, set()).add(key)
        # WRF writes wrfrst files every restart_interval minutes whatever restart is; a hand-written config of a run
        # that ends before the first restart time has no [restart]
        rst = nml.get("time_control", {}).get("restart_interval", [0])[0]
        if rst > 0 and "restart" not in ref:
            self.assertLess(gen["run"]["run_time"], rst * 60)
            self.assertEqual(gen.pop("restart"), {"interval": rst * 60})

        all_secs = sorted(set(list(gen.keys()) + list(ref.keys())))
        if sections:  # only these sections (the hand-written configs add run/output settings)
            all_secs = [x for x in all_secs if x in sections]
        for s in all_secs:
            self.assertIn(s, gen, f"[{s}] missing in generated config")
            self.assertIn(s, ref, f"[{s}] extra in generated config")

            all_keys = sorted(set(list(gen[s].keys()) + list(ref[s].keys())))
            for k in all_keys:
                if ignore_keys and s in ignore_keys and k in ignore_keys[s]:
                    continue
                if sections and (k not in ref[s] or k not in gen[s]):
                    continue  # default values written by one side only
                self.assertIn(k, gen[s], f"[{s}].{k} missing in generated config")
                self.assertIn(k, ref[s], f"[{s}].{k} extra in generated config")

                # Compare normalized string values (treating '20' and '20.0' as equal)
                gv = str(gen[s][k]).replace(" ", "")
                rv = str(ref[s][k]).replace(" ", "")
                if gv != rv and (gv.endswith(".0") or rv.endswith(".0")):
                    try:
                        if float(gv) == float(rv):
                            gv = rv
                    except ValueError:
                        pass
                self.assertEqual(gv, rv, f"[{s}].{k} mismatch: generated '{gen[s][k]}' vs ref '{ref[s][k]}'")

    def test_real_r0_key_by_key(self):
        self._compare_configs("run_R0_r8/namelist.input", "configs/real_r0.ini")

    def test_real_r1_key_by_key(self):
        self._compare_configs("run_R1_r8/namelist.input", "configs/real_r1.ini")

    def test_real_r2_key_by_key(self):
        self._compare_configs("run_R2/namelist.input", "configs/real_r2.ini")

    def test_real_r3_key_by_key(self):
        self._compare_configs("run_R3/namelist.input", "configs/real_r3.ini")

    def test_real_mp6_key_by_key(self):
        self._compare_configs("run_mp6/namelist.input", "configs/real_mp6.ini")

    def test_real_p1_key_by_key(self):
        self._compare_configs("run_P1_r8/steps/namelist.input", "configs/real_p1_r8.ini", sections=("phys", "dyn"))

    def test_real_p2_key_by_key(self):
        self._compare_configs("run_P2_r8/steps/namelist.input", "configs/real_p2_r8.ini", sections=("phys", "dyn"))

    def test_real_p3_key_by_key(self):
        self._compare_configs("run_P3_r8/steps/namelist.input", "configs/real_p3_r8.ini", sections=("phys", "dyn"))

    def test_real_p8_key_by_key(self):
        self._compare_configs("run_P8_r8/steps/namelist.input", "configs/real_p8_r8.ini", sections=("phys", "dyn"))

    def test_real_n1_key_by_key(self):
        self._compare_configs("run_N1/namelist.input", "configs/real_n1.ini", sections=("domains", "d02"))

    def test_real_n2_key_by_key(self):
        self._compare_configs("run_N2/namelist.input", "configs/real_n2.ini", sections=("domains", "d02"))

    def test_real_n1_steps_key_by_key(self):
        self._compare_configs("run_N1_r8/steps/namelist.input", "configs/real_n1_steps.ini", sections=("domains", "d02"))

    def test_real_n2_steps_key_by_key(self):
        self._compare_configs("run_N2_r8/steps/namelist.input", "configs/real_n2_steps.ini", sections=("domains", "d02"))

    def test_real_p4_diff_opt_mapped(self):
        # run_P4 namelist specifies diff_opt = 1, km_opt = 4
        nml_path = wrf_runs_path("run_P4/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")
        with open(nml_path) as f:
            nml = parse_namelist(f.read())
        t = WrfTranslator(nml, namelist_path=nml_path)
        gen = t.translate()
        self.assertEqual(gen["dyn"].get("diff_opt"), 1)
        self.assertEqual(gen["dyn"].get("km_opt"), 4)
        self.assertEqual(gen["phys"].get("mp_physics"), 6)


class TestIdealCases(unittest.TestCase):
    """Test suite for ideal cases translation."""

    def test_hill2d_mapping(self):
        nml_path = wrf_ref_path("run_hill_p0/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")
        with open(nml_path) as f:
            nml = parse_namelist(f.read())
        t = WrfTranslator(nml, namelist_path=nml_path)
        gen = t.translate()
        self.assertEqual(gen["run"]["case"], "em_hill2d_x")
        self.assertEqual(gen["domain"]["nx"], 201)
        self.assertEqual(gen["domain"]["ny"], 1)
        self.assertEqual(gen["domain"]["nz"], 40)
        self.assertEqual(gen["domain"]["dx"], 2000)
        self.assertEqual(gen["time"]["dt"], 20)
        self.assertEqual(gen["time"]["time_step_sound"], 6)

    def test_quarter_ss_mapping(self):
        nml_path = wrf_ref_path("run_qss/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")
        with open(nml_path) as f:
            nml = parse_namelist(f.read())
        t = WrfTranslator(nml, namelist_path=nml_path)
        gen = t.translate()
        self.assertEqual(gen["run"]["case"], "em_quarter_ss")
        self.assertEqual(gen["domain"]["nx"], 41)
        self.assertEqual(gen["domain"]["ny"], 41)
        self.assertEqual(gen["domain"]["nz"], 40)
        self.assertEqual(gen["time"]["dt"], 12)
        self.assertEqual(gen["time"]["time_step_sound"], 6)

    def test_straka_mapping(self):
        nml_path = wrf_ref_path("run_straka_d1/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")
        with open(nml_path) as f:
            nml = parse_namelist(f.read())
        t = WrfTranslator(nml, namelist_path=nml_path)
        gen = t.translate()
        self.assertEqual(gen["run"]["case"], "em_grav2d_x")
        self.assertEqual(gen["domain"]["nx"], 512)
        self.assertEqual(gen["domain"]["ny"], 1)
        self.assertEqual(gen["domain"]["nz"], 64)
        self.assertEqual(gen["time"]["dt"], 1)
        self.assertEqual(gen["time"]["time_step_sound"], 6)

    def test_wrf_ideal_case_names_and_keys(self):
        # Every WRF ideal_case that CPPWRF ports maps to the src/cases/wrf_ideal.cpp case name; the
        # sounding is the input_sounding next to the namelist; hybrid_opt = 0 is kept (the model's
        # default is 2); open_xs/open_xe give bc_x = open.
        tmp = tempfile.mkdtemp()
        nml_path = os.path.join(tmp, "namelist.input")
        expected = {1: "em_hill2d_x", 2: "em_quarter_ss", 4: "em_squall2d_x", 5: "em_squall2d_y", 6: "em_grav2d_x"}
        for ideal_case, name in expected.items():
            text = f"""
            &time_control run_minutes = 10 /
            &domains time_step = 3, e_we = 202, e_sn = 3, e_vert = 81, dx = 250, dy = 250, ztop = 20000 /
            &dynamics hybrid_opt = 0 /
            &bdy_control periodic_x = .false., open_xs = .true., open_xe = .true., periodic_y = .true. /
            &ideal ideal_case = {ideal_case} /
            """
            t = WrfTranslator(parse_namelist(complete(text)), namelist_path=nml_path)
            gen = t.translate()
            self.assertEqual(t.errors, [], msg=f"ideal_case = {ideal_case}")
            self.assertEqual(gen["run"]["case"], name)
            self.assertEqual(gen["case"]["sounding_file"], os.path.join(tmp, "input_sounding"))
            self.assertEqual(gen["dyn"]["hybrid_opt"], 0)
            self.assertEqual(gen["domain"]["bc_x"], "open")
            self.assertEqual(gen["domain"]["dy"], 250)
            # no damp_opt in the namelist: WRF's Registry default 3 (the model's own default is 0)
            self.assertEqual(gen["dyn"]["damp_opt"], 3)


class TestValidationAndWarnings(unittest.TestCase):
    """Test error and warning detection on unsupported/deviating options."""

    def test_unsupported_mp_physics(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &physics mp_physics = 2 /
        &dynamics /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        t.translate()
        self.assertTrue(any("mp_physics = 2" in e for e in t.errors), t.errors)

    def test_unsupported_diff_opt(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &dynamics diff_opt = 3 /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        t.translate()
        self.assertTrue(any("diff_opt = 3" in e for e in t.errors), t.errors)

    def test_hydrostatic_error(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &dynamics non_hydrostatic = .false. /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        t.translate()
        self.assertTrue(any("non_hydrostatic = .false." in e for e in t.errors))

    def test_noahmp_mapping(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &physics sf_sfclay_physics = 1, sf_surface_physics = 4, bl_pbl_physics = 1 /
        &noah_mp dveg = 4, opt_runsrf = 3, opt_runsub = 3 /
        &dynamics /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        gen = t.translate()
        self.assertEqual(t.warnings, [])
        self.assertEqual(t.errors, [])
        self.assertEqual(gen["phys"]["sf_surface_physics"], 4)
        self.assertEqual(gen["phys"]["num_soil_layers"], 4)
        self.assertEqual(gen["phys"]["dveg"], 4)
        self.assertEqual(gen["phys"]["opt_runsrf"], 3)
        self.assertEqual(gen["phys"]["opt_runsub"], 3)

    def test_noahmp_option_values(self):
        # the Noah-MP option values of WRF 4.8.0 that are ported pass through; the others are errors
        def run(nmp):
            text = f"""
            &time_control /
            &domains time_step = 60 /
            &physics sf_sfclay_physics = 1, sf_surface_physics = 4, bl_pbl_physics = 1 /
            &noah_mp {nmp} /
            &dynamics /
            """
            t = WrfTranslator(parse_namelist(real_case(text)))
            return t, t.translate()
        ok = ("dveg = 2, opt_crs = 2, opt_btr = 3, opt_runsrf = 7, opt_runsub = 7, opt_sfc = 2, opt_frz = 2, "
              "opt_inf = 2, opt_rad = 1, opt_alb = 1, opt_snf = 5, opt_tbot = 1, opt_stc = 3, opt_gla = 2, "
              "opt_rsf = 4, opt_tksno = 5, opt_compact = 1, opt_scf = 2, opt_wetland = 1, soiltstep = 180.")
        t, gen = run(ok)
        self.assertEqual(t.errors, [])
        for k, v in (("dveg", 2), ("opt_runsrf", 7), ("opt_snf", 5), ("opt_scf", 2), ("opt_wetland", 1)):
            self.assertEqual(gen["phys"][k], v)
        for bad in ("dveg = 10", "opt_runsrf = 8, opt_runsub = 8", "opt_runsrf = 5, opt_runsub = 5",
                    "opt_infdv = 1", "opt_alb = 3", "opt_soil = 2", "opt_crop = 1", "opt_irr = 1",
                    "opt_tdrn = 1", "opt_wetland = 2", "opt_scf = 3"):
            t, _ = run(bad)
            self.assertTrue(any("Noah-MP" in e and bad.split()[0] in e for e in t.errors), (bad, t.errors))

    def test_extended_physics_options_mapping(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &physics
         mp_physics = 6,
         mp_tend_lim = 5.0,
         hail_opt = 1,
         use_mp_re = 0,
         sf_sfclay_physics = 1,
         sf_surface_physics = 2,
         num_soil_layers = 4,
         bl_pbl_physics = 1,
         ysu_topdown_pblmix = 0,
         cu_physics = 3,
         ishallow = 1,
         clos_choice = 2,
         kfeta_trigger = 3,
         convtrans_avglen_m = 45.0,
         ra_lw_physics = 4,
         ra_sw_physics = 4,
         radt = 10,
         icloud = 0,
         cldovrlp = 4,
         idcor = 1,
         o3input = 0,
         ghg_input = 0,
         aer_opt = 0,
         rdmaxalb = .false.,
         opt_thcnd = 2,
         kim_tofd = .true.,
         tofd_factor = 0.005
        /
        &dynamics /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        gen = t.translate()
        self.assertEqual(t.errors, [])
        p = gen["phys"]
        self.assertEqual(p["mp_tend_lim"], 5.0)
        self.assertEqual(p["hail_opt"], 1)
        self.assertEqual(p["use_mp_re"], 0)
        self.assertEqual(p["num_soil_layers"], 4)
        self.assertEqual(p["ysu_topdown_pblmix"], 0)
        self.assertEqual(p["ishallow"], 1)
        self.assertEqual(p["clos_choice"], 2)
        self.assertEqual(p["kfeta_trigger"], 3)
        self.assertEqual(p["convtrans_avglen_m"], 45.0)
        self.assertEqual(p["icloud"], 0)
        self.assertEqual(p["cldovrlp"], 4)
        self.assertEqual(p["idcor"], 1)
        self.assertEqual(p["o3input"], 0)
        self.assertEqual(p["ghg_input"], 0)
        self.assertEqual(p["rdmaxalb"], False)
        self.assertEqual(p["opt_thcnd"], 2)
        self.assertEqual(p["kim_tofd"], True)
        self.assertEqual(p["tofd_factor"], 0.005)

    def test_do_radar_ref(self):
        # do_radar_ref 1: REFL_10CM of WSM6 and Thompson at history steps; 0 (default) is not written; other values
        # are rejected
        def translate(value):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = 6, do_radar_ref = {value} /
            &dynamics /
            """)))
            return t, t.translate()
        t, gen = translate(1)
        self.assertEqual(t.errors, [])
        self.assertEqual(gen["phys"]["do_radar_ref"], 1)
        t, gen = translate(0)
        self.assertEqual(t.errors, [])
        self.assertNotIn("do_radar_ref", gen["phys"])
        t, _ = translate(2)
        self.assertTrue(any("do_radar_ref = 2 is not supported" in e for e in t.errors), t.errors)

    def test_usemonalb_rdlai2d(self):
        # usemonalb: landuse_init for every LSM; rdlai2d: Noah only (Noah-MP and no LSM ignore it)
        def phys(extra):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics {extra} /
            &dynamics /
            """)))
            gen = t.translate()
            self.assertEqual(t.errors, [], extra)
            return gen.get("phys", {})
        p = phys("sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1, usemonalb = .true., rdlai2d = .true.")
        self.assertEqual((p["usemonalb"], p["rdlai2d"]), (True, True))
        p = phys("sf_sfclay_physics = 1, sf_surface_physics = 4, bl_pbl_physics = 1, usemonalb = .true., rdlai2d = .true.")
        self.assertEqual(p["usemonalb"], True)
        self.assertNotIn("rdlai2d", p)
        p = phys("sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1, usemonalb = .false., rdlai2d = .false.")
        self.assertNotIn("usemonalb", p)
        self.assertNotIn("rdlai2d", p)
        p = phys("usemonalb = .true.")
        self.assertEqual(p["usemonalb"], True)

    def test_slab_lsm(self):
        # sf_surface_physics = 1 (module_sf_slab.F): 5 soil layers (the Registry default), ifsnow 1 (default) or 0
        def phys(extra):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics sf_sfclay_physics = 1, sf_surface_physics = 1, bl_pbl_physics = 1{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t.errors, gen.get("phys", {})
        err, p = phys("")
        self.assertEqual(err, [])
        self.assertEqual((p["sf_surface_physics"], p["num_soil_layers"]), (1, 5))
        self.assertNotIn("ifsnow", p)
        err, p = phys(", num_soil_layers = 5, ifsnow = 0")
        self.assertEqual(err, [])
        self.assertEqual(p["ifsnow"], 0)
        for extra, msg in ((", ifsnow = 2", "ifsnow = 2"),
                           (", fractional_seaice = 1", "fractional_seaice = 1"),
                           (", sst_update = 1", "sst_update = 1 with sf_surface_physics = 1")):
            err, _ = phys(extra)
            self.assertIn(msg, " ".join(err), extra)

    def test_num_soil_layers_as_check_a_mundo(self):
        # share/module_check_a_mundo.F set_physics_rconfigs sets num_soil_layers from sf_surface_physics before
        # wrf.exe allocates the domains (WRF 4.8.0 real*8 P1 with num_soil_layers = 5: wrfout identical to 4); the
        # translator writes WRF's value and notes the namelist value it replaces
        def phys(lsm, extra):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = 6, sf_sfclay_physics = 1, sf_surface_physics = {lsm}, bl_pbl_physics = 1{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t.errors, t.notices, gen.get("phys", {})
        for lsm, nl, want in ((2, 5, 4), (2, 3, 4), (4, 5, 4), (1, 4, 5), (3, 4, 6), (3, 9, 9), (3, 6, 6),
                              (2, None, 4), (1, None, 5), (3, None, 6)):
            err, notes, p = phys(lsm, "" if nl is None else f", num_soil_layers = {nl}")
            self.assertEqual(err, [], (lsm, nl))
            self.assertEqual(p["num_soil_layers"], want, (lsm, nl))
            replaced = f"num_soil_layers = {nl} is replaced by {want}"
            if nl is not None and nl != want:
                self.assertIn(replaced, " ".join(notes), (lsm, nl))
            else:
                self.assertNotIn("num_soil_layers", " ".join(notes), (lsm, nl))

    def test_surface_layer_and_land_surface_together(self):
        # CPPWRF's column physics runs a surface layer over a land surface model (the model stops at its start
        # otherwise: "PhysicsSuite: only sf_sfclay_physics=1|2|4|5|7, sf_surface_physics=1|2|3|4"); wrf.exe runs
        # either alone
        def errors(phys):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = 6, {phys} /
            &dynamics /
            """)))
            t.translate()
            return " ".join(t.errors)
        for phys, msg in (("sf_sfclay_physics = 1, sf_surface_physics = 0, bl_pbl_physics = 1",
                           "sf_surface_physics = 0 (no land surface model) with sf_sfclay_physics = 1, bl_pbl_physics = 1"),
                          ("sf_sfclay_physics = 1",
                           "sf_surface_physics = 0 (no land surface model) with sf_sfclay_physics = 1 is not"),
                          ("sf_sfclay_physics = 2, sf_surface_physics = 0, bl_pbl_physics = 2, num_soil_layers = 5",
                           "sf_surface_physics = 0 (no land surface model)"),
                          ("sf_surface_physics = 2", "sf_sfclay_physics = 0 (no surface layer) with sf_surface_physics = 2")):
            self.assertIn(msg, errors(phys), phys)
        self.assertEqual(errors("sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1"), "")

    def test_cumulus_or_radiation_without_surface(self):
        # wrf.exe runs cumulus or radiation without a surface layer and a land surface model (module_physics_init.F
        # sfclay_select / sfc_select do nothing for 0); cppwrf stops at its start, so the translator gives the
        # model's error: PhysicsSuite's (src/phys/driver/suite.cpp) with a cumulus scheme, model.cpp's with
        # radiation only
        def errors(phys):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = 6, {phys} /
            &dynamics /
            """)))
            t.translate()
            return " ".join(t.errors)
        suite_msg = ("PhysicsSuite: only sf_sfclay_physics=1|2|4|5|7, sf_surface_physics=1|2|3|4, "
                     "bl_pbl_physics=0|1|2|4|5|7|8 (the PX land surface, sf_surface_physics=7, is not ported)")
        rad_msg = ("phys.ra_lw/ra_sw_physics without surface-layer, land-surface, PBL or cumulus physics is not "
                   "supported")
        for phys, msg in (("cu_physics = 1", suite_msg),
                          ("cu_physics = 2", suite_msg),
                          ("cu_physics = 3", suite_msg),
                          ("cu_physics = 16", suite_msg),
                          ("cu_physics = 1, ra_lw_physics = 4, ra_sw_physics = 4", suite_msg),
                          ("cu_physics = 1, ra_lw_physics = 1, ra_sw_physics = 1", suite_msg),
                          ("ra_lw_physics = 4, ra_sw_physics = 4", rad_msg),
                          ("ra_lw_physics = 1, ra_sw_physics = 1", rad_msg),
                          ("ra_lw_physics = 4, ra_sw_physics = 1", rad_msg)):
            err = errors(phys)
            self.assertIn(msg, err, phys)
            self.assertEqual(err.count("cppwrf would stop"), 1, phys)
        # with a surface layer and a land surface model both run
        for phys in ("sf_sfclay_physics = 1, sf_surface_physics = 2, cu_physics = 1",
                     "sf_sfclay_physics = 1, sf_surface_physics = 2, ra_lw_physics = 4, ra_sw_physics = 4",
                     "sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1, cu_physics = 1, "
                     "ra_lw_physics = 4, ra_sw_physics = 4"):
            self.assertNotIn("cppwrf would stop", errors(phys), phys)

    def test_boulac(self):
        # bl_pbl_physics = 8 (module_bl_boulac.F) over sf_sfclay_physics = 1 or 2 (module_physics_init: 'use sfclay or
        # myjsfc scheme for this pbl option'); verified: no tier warning; Shin-Hong (11) is not ported
        def phys(sfc, pbl, mp=0, extra=""):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = {sfc}, sf_surface_physics = 2, bl_pbl_physics = {pbl} /
            &dynamics /
            {extra}""")))
            gen = t.translate()
            return t.errors, t.warnings, gen
        for sfc, mp in ((1, 0), (2, 6), (1, 8)):
            err, warn, gen = phys(sfc, 8, mp)
            self.assertEqual(err, [])
            self.assertEqual((gen["phys"]["sf_sfclay_physics"], gen["phys"]["bl_pbl_physics"]), (sfc, 8))
            self.assertNotIn("verification tier", " ".join(warn))
        err, _, gen = phys(1, 8, 0, "&cppwrf allow_unverified = .true. /")
        self.assertEqual((err, gen["cppwrf"]["allow_unverified"]), ([], True))
        for sfc in (5, 7):
            err, _, _ = phys(sfc, 8)
            self.assertIn("needs sf_sfclay_physics = 1 or 2", " ".join(err))
        err, _, _ = phys(1, 11)
        self.assertIn("Shin-Hong", " ".join(err))

    def test_qnse(self):
        # bl_pbl_physics = 4 (module_bl_qnsepbl.F) with sf_sfclay_physics = 4 (module_physics_init: 'use qnsesfc scheme
        # for this pbl option'); column-verified (data/option_tiers.txt: a warning). mfshconv 0 or 1 (WRF's default 1
        # runs the EDKF mass flux, module_bl_mfshconvpbl.F; column-verified too); no sea-ice wrapper, no isfflx = 0
        def phys(sfc, pbl, extra_phys="mfshconv = 0,", extra=""):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = 6, sf_sfclay_physics = {sfc}, sf_surface_physics = 2, bl_pbl_physics = {pbl},
             {extra_phys} /
            &dynamics /
            {extra}""")))
            gen = t.translate()
            return t.errors, t.warnings, gen
        err, warn, gen = phys(4, 4)
        self.assertEqual(err, [])
        self.assertEqual((gen["phys"]["sf_sfclay_physics"], gen["phys"]["bl_pbl_physics"], gen["phys"]["mfshconv"]),
                         (4, 4, 0))
        self.assertNotIn("verification tier", " ".join(warn))
        for extra_phys in ("", "mfshconv = 1,"):  # WRF's default: the EDKF mass flux, written to [phys]
            err, warn, gen = phys(4, 4, extra_phys)
            self.assertEqual((err, gen["phys"]["mfshconv"]), ([], 1), extra_phys)
            self.assertNotIn("verification tier", " ".join(warn), extra_phys)
        err, _, _ = phys(4, 4, "mfshconv = 2,")
        self.assertIn("supports 0 and 1", " ".join(err))
        err, _, gen = phys(1, 1, "mfshconv = 1,")  # check_a_mundo resets it for the other PBL schemes
        self.assertEqual(err, [])
        self.assertNotIn("mfshconv", gen["phys"])
        for sfc, pbl, msg in ((1, 4, "needs sf_sfclay_physics = 4"), (4, 1, "with bl_pbl_physics = 4"),
                              (4, 2, "with bl_pbl_physics = 4")):
            err, _, _ = phys(sfc, pbl)
            self.assertIn(msg, " ".join(err), (sfc, pbl))
        err, _, _ = phys(4, 4, "mfshconv = 0, fractional_seaice = 1,")
        self.assertIn("QNSESFC_SEAICE_WRAPPER", " ".join(err))
        err, _, _ = phys(4, 4, "mfshconv = 0, isfflx = 0,")
        self.assertIn("4 (QNSE)", " ".join(err))

    def test_acm2_pleim_xiu(self):
        # bl_pbl_physics = 7 (module_bl_acm.F) over sf_sfclay_physics = 1 or 7 (module_physics_init: 'use sfclay or
        # pxsfc scheme for this pbl option'), with qc and qi; the Pleim-Xiu surface layer with ACM2 and Noah; the PX
        # land surface (sf_surface_physics = 7) is not ported
        def phys(sfc, pbl, extra="", mp=6, lsm=2):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = {sfc}, sf_surface_physics = {lsm}, bl_pbl_physics = {pbl}{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t.errors, gen.get("phys", {})
        for sfc in (1, 7):
            err, p = phys(sfc, 7)
            self.assertEqual(err, [])
            self.assertEqual((p["sf_sfclay_physics"], p["bl_pbl_physics"]), (sfc, 7))
        err, p = phys(7, 7, ", bldt = 3")
        self.assertEqual((err, p["bldt"]), ([], 3))
        for args, msg in (((2, 7), "needs sf_sfclay_physics = 1 or 7"), ((5, 7), "needs sf_sfclay_physics = 1 or 7"),
                          ((7, 1), "sf_sfclay_physics = 7 (Pleim-Xiu) is supported with"),
                          ((7, 0), "sf_sfclay_physics = 7 (Pleim-Xiu) is supported with"),
                          ((7, 7, "", 1), "needs a microphysics with qc and qi"),
                          ((1, 7, "", 0), "needs a microphysics with qc and qi"),
                          ((7, 7, "", 6, 4), "sf_sfclay_physics = 7 (Pleim-Xiu) is supported with"),
                          ((7, 7, ", fractional_seaice = 1"), "fractional_seaice = 1"),
                          ((7, 7, ", isfflx = 0"), "sf_sfclay_physics = 7 with isfflx = 0"),
                          ((7, 7, ", num_soil_layers = 2", 6, 7), "sf_surface_physics = 7 (Pleim-Xiu land surface")):
            err, _ = phys(*args)
            self.assertIn(msg, " ".join(err), args)

    def test_morrison_two_moment(self):
        # mp_physics = 10 (module_mp_morr_two_moment.F): moist qv qc qr qi qs qg and scalar qni qns qnr qng (Registry
        # package morr_two_moment); morr_rimed_ice = 1 (default, hail) or graupel for any other value; not ported:
        # mp_physics = 40, progn = 1, do_radar_ref = 1 with Morrison, the ideal cases
        def tr(extra="", mp=10, case=real_case, cu=1):
            t = WrfTranslator(parse_namelist(case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1,
                     cu_physics = {cu}{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t, gen
        t, g = tr()
        self.assertEqual(t.errors, [])
        self.assertEqual(g["moist"]["species"], "qv,qc,qr,qi,qs,qg,qni,qns,qnr,qng")
        self.assertEqual(g["phys"]["mp_physics"], 10)
        self.assertNotIn("morr_rimed_ice", g["phys"])
        for v, note in (("0", False), ("2", True)):
            t, g = tr(f", morr_rimed_ice = {v}", cu=0)
            self.assertEqual(t.errors, [], v)
            self.assertEqual(g["phys"]["morr_rimed_ice"], 0, v)
            self.assertEqual(any("graupel" in n for n in t.notices), note, v)
        t, g = tr(", morr_rimed_ice = 0", mp=6)  # read by Morrison only
        self.assertEqual(t.errors, [])
        self.assertNotIn("morr_rimed_ice", g["phys"])
        for args, msg in (((", do_radar_ref = 1",), "do_radar_ref = 1 with mp_physics = 10"),
                          ((", progn = 1",), "progn = [1]"),
                          (("", 40), "mp_physics = 40 (Morrison with aerosols"),
                          ((", progn = 1", 6), "progn = [1]")):
            t, _ = tr(*args)
            self.assertIn(msg, " ".join(t.errors), args)
        t = WrfTranslator(parse_namelist(complete("&time_control /\n&domains time_step = 6 /\n"
                                                   "&physics mp_physics = 10 /\n")), override_case="quarter_ss")
        t.translate()
        self.assertIn("mp_physics = 10 (Morrison two-moment) is supported for real-data runs only", " ".join(t.errors))

    def test_p3_one_category(self):
        # mp_physics = 50 / 51 (module_mp_p3.F, one ice category): moist qv qc qr qi and scalar qni qnr qir qib (51:
        # qnc first, Registry packages p3_1category / p3_1category_nc); not ported: 52 (two ice categories), 53
        # (three-moment ice), do_radar_ref = 1, nests, the ideal cases; Milbrandt (9) stays rejected
        def tr(extra="", mp=50, case=real_case):
            t = WrfTranslator(parse_namelist(case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1,
                     cu_physics = 1{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t, gen
        for mp, sp in ((50, "qv,qc,qr,qi,qni,qnr,qir,qib"), (51, "qv,qc,qr,qi,qnc,qni,qnr,qir,qib")):
            t, g = tr(mp=mp)
            self.assertEqual(t.errors, [], mp)
            self.assertEqual(g["moist"]["species"], sp)
            self.assertEqual(g["phys"]["mp_physics"], mp)
        for args, msg in (((", do_radar_ref = 1",), "do_radar_ref = 1 with mp_physics = 50"),
                          (("", 52), "mp_physics"), (("", 53), "mp_physics"), (("", 9), "mp_physics")):
            t, _ = tr(*args)
            self.assertIn(msg, " ".join(t.errors), args)
        doms = ("&domains time_step = 60, max_dom = 2, e_we = 100, 61, e_sn = 100, 61, e_vert = 45, 45,"
                " parent_id = 0, 1, i_parent_start = 1, 30, j_parent_start = 1, 30,"
                " parent_grid_ratio = 1, 3, parent_time_step_ratio = 1, 3 /\n")
        t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n{doms}&physics mp_physics = 51, 51 /\n")))
        t.translate()
        self.assertIn("mp_physics = 51 (P3) with nests is not supported", " ".join(t.errors))
        t = WrfTranslator(parse_namelist(complete("&time_control /\n&domains time_step = 6 /\n"
                                                   "&physics mp_physics = 50 /\n")), override_case="quarter_ss")
        t.translate()
        self.assertIn("mp_physics = 50 (P3) is supported for real-data runs only", " ".join(t.errors))

    def test_nssl(self):
        # mp_physics = 18 with its defaults (2-moment, hail, CCN, graupel / hail volume) and 17 (check_a_mundo makes
        # it 18): moist qh and the scalars of the packages nssl2mconc, nssl_hail, nssl_ccn_opt, nssl_hailvol; every
        # other NSSL setting, &nssl_mp_params, nests and the ideal cases are errors; 19, 21 and 22 stay unsupported
        def tr(extra="", mp=18, case=real_case, group=""):
            t = WrfTranslator(parse_namelist(case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1,
                     cu_physics = 1{extra} /
            &dynamics /
            {group}""")))
            gen = t.translate()
            return t, gen
        sp = "qv,qc,qr,qi,qs,qg,qh,qndrop,qnr,qni,qns,qng,qnh,qnn,qvolg,qvolh"
        for mp, extra in ((18, ""), (17, ""), (18, ", nssl_2moment_on = 1, nssl_hail_on = 1, nssl_ccn_on = 1, "
                                                   "nssl_density_on = 2, nssl_cccn = 0.5e9, nssl_ccn_is_ccna = 1"),
                          (17, ", nssl_hail_on = 0, nssl_density_on = 1"), (18, ", do_radar_ref = 1")):
            t, g = tr(extra, mp)
            self.assertEqual(t.errors, [], (mp, extra))
            self.assertEqual(g["moist"]["species"], sp)
            self.assertEqual(g["phys"]["mp_physics"], 18)
        for args, msg in (((", nssl_hail_on = 0",), "nssl_hail_on = 0"), ((", nssl_density_on = 1",), "nssl_density_on"),
                          ((", nssl_2moment_on = 0",), "nssl_2moment_on"), ((", nssl_ccn_on = 0",), "nssl_ccn_on"),
                          ((", nssl_3moment = 1",), "nssl_3moment"), ((", nssl_3moment = 1", 17), "nssl_3moment"),
                          ((", nssl_cccn = 1.e9",), "nssl_cccn"), ((", nssl_alphah = 1.",), "nssl_alphah"),
                          ((", nssl_cnoh = 1.e5",), "nssl_cnoh"), ((", nssl_icdx = 3",), "nssl_icdx"),
                          ((", nssl_ssat_output = 1",), "nssl_ssat_output"), (("", 19), "mp_physics"),
                          (("", 21), "mp_physics"), (("", 22), "mp_physics"),
                          (("", 18, real_case, "&nssl_mp_params irenuc = 2 /"), "nssl_mp_params")):
            t, _ = tr(*args)
            self.assertIn(msg, " ".join(t.errors), args)
        doms = ("&domains time_step = 60, max_dom = 2, e_we = 100, 61, e_sn = 100, 61, e_vert = 45, 45,"
                " parent_id = 0, 1, i_parent_start = 1, 30, j_parent_start = 1, 30,"
                " parent_grid_ratio = 1, 3, parent_time_step_ratio = 1, 3 /\n")
        t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n{doms}&physics mp_physics = 18, 18 /\n")))
        t.translate()
        self.assertIn("mp_physics = 18 (NSSL) with nests is not supported", " ".join(t.errors))
        t = WrfTranslator(parse_namelist(complete("&time_control /\n&domains time_step = 6 /\n"
                                                   "&physics mp_physics = 18 /\n")), override_case="quarter_ss")
        t.translate()
        self.assertIn("mp_physics = 18 (NSSL) is supported for real-data runs only", " ".join(t.errors))

    def test_wsm3_wsm5(self):
        # mp_physics = 3 / 4 (module_mp_wsm3.F / module_mp_wsm5.F; verified: no tier warning): moist qv qc qr (WSM3) or qv qc qr qi qs (WSM5, Registry packages wsm3scheme / wsm5scheme);
        # not ported: do_radar_ref = 1, the ideal cases; WSM7 (24) and WSM6R (106) are named errors
        def tr(extra="", mp=4, case=real_case):
            t = WrfTranslator(parse_namelist(case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1,
                     cu_physics = 1{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t, gen
        for mp, sp in ((4, "qv,qc,qr,qi,qs"), (3, "qv,qc,qr")):
            t, g = tr(mp=mp)
            self.assertEqual(t.errors, [], mp)
            self.assertEqual(g["moist"]["species"], sp)
            self.assertEqual(g["phys"]["mp_physics"], mp)
            self.assertFalse(any("verification tier" in w for w in t.warnings), mp)
        for args, msg in (((", do_radar_ref = 1", 4), "do_radar_ref = 1 with mp_physics = 4 (WSM5)"),
                          ((", do_radar_ref = 1", 3), "do_radar_ref = 1 with mp_physics = 3 (WSM3)"),
                          (("", 24), "mp_physics = 24 (WSM7"), (("", 106), "mp_physics = 106 (WSM6R")):
            t, _ = tr(*args)
            self.assertIn(msg, " ".join(t.errors), args)
        t = WrfTranslator(parse_namelist(complete("&time_control /\n&domains time_step = 6 /\n"
                                                   "&physics mp_physics = 4 /\n")), override_case="quarter_ss")
        t.translate()
        self.assertIn("mp_physics = 4 (WSM5) is supported for real-data runs only", " ".join(t.errors))

    def test_wdm6(self):
        # mp_physics = 16 (module_mp_wdm6.F): moist qv qc qr qi qs qg, scalars qnn qnc qnr (Registry package
        # wdm6scheme); hail_opt and ccn_conc pass through; not ported: do_radar_ref = 1, nests, ideal cases, no column
        # physics. mp_physics = 14 (WDM5, wdm5scheme): qv qc qr qi qs qnn qnc qnr, also without the column physics.
        # WDM7 (26) is a named error
        def tr(extra="", mp=16, phys=", sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1",
               doms="", case=real_case):
            t = WrfTranslator(parse_namelist(case(f"""
            &time_control /
            &domains time_step = 60{doms} /
            &physics mp_physics = {mp}{phys}{extra} /
            &dynamics /
            """)))
            gen = t.translate()
            return t, gen
        t, g = tr(", hail_opt = 1, ccn_conc = 3.e8")
        self.assertEqual(t.errors, [])
        self.assertEqual(g["moist"]["species"], "qv,qc,qr,qi,qs,qg,qnn,qnc,qnr")
        self.assertEqual(g["phys"]["mp_physics"], 16)
        self.assertEqual(g["phys"]["hail_opt"], 1)
        self.assertEqual(g["phys"]["ccn_conc"], 3.e8)
        t, g = tr()
        self.assertEqual(t.errors, [])
        self.assertNotIn("ccn_conc", g["phys"])
        for phys in ("", ", sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1"):
            t, g = tr(", ccn_conc = 2.e8", mp=14, phys=phys)
            self.assertEqual(t.errors, [])
            self.assertEqual(g["moist"]["species"], "qv,qc,qr,qi,qs,qnn,qnc,qnr")
            self.assertEqual((g["phys"]["mp_physics"], g["phys"]["ccn_conc"]), (14, 2.e8))
        for kw, msg in (({"extra": ", do_radar_ref = 1"}, "do_radar_ref = 1 with mp_physics = 16"),
                        ({"extra": ", do_radar_ref = 1", "mp": 14}, "do_radar_ref = 1 with mp_physics = 14"),
                        ({"phys": ""}, "mp_physics = 16 (WDM6) needs the column physics"),
                        ({"mp": 26}, "mp_physics = 26 (WDM7")):
            t, _ = tr(**kw)
            self.assertIn(msg, " ".join(t.errors), kw)
        t = WrfTranslator(parse_namelist(complete("&time_control /\n&domains time_step = 6 /\n"
                                                   "&physics mp_physics = 16 /\n")), override_case="quarter_ss")
        t.translate()
        self.assertIn("mp_physics = 16 (WDM6) is supported for real-data runs only", " ".join(t.errors))

    def test_ruc_lsm(self):
        # sf_surface_physics = 3 (module_sf_ruclsm.F): 6 or 9 soil levels (check_a_mundo), sfclayrev or MYJSFC, a
        # microphysics with qc; column-verified (tier warning); MYNNSFC, mosaic_lu / mosaic_soil, sst_update, nests
        # and other level counts are errors
        def phys(extra, sfc=1, pbl=1, mp=6, nl=9, more=""):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, sf_sfclay_physics = {sfc}, sf_surface_physics = 3, bl_pbl_physics = {pbl},
                     num_soil_layers = {nl}{extra} /
            &dynamics /
            {more}""")))
            gen = t.translate()
            return t.errors, t.warnings, gen.get("phys", {})
        for nl, sfc, pbl in ((9, 1, 1), (6, 1, 1), (9, 2, 2)):
            err, warn, p = phys("", sfc, pbl, nl=nl)
            self.assertEqual(err, [])
            self.assertEqual((p["sf_surface_physics"], p["num_soil_layers"]), (3, nl))
            self.assertNotIn("verification tier", " ".join(warn))
        err, _, p = phys(", rdlai2d = .true.")
        self.assertEqual((err, p["rdlai2d"]), ([], True))
        for kw, msg in (({"sfc": 5, "pbl": 5}, "MYNN surface layer"),
                        ({"mp": 0}, "cloud water"), ({"extra": ", mosaic_lu = 1"}, "mosaic_lu"),
                        ({"extra": ", mosaic_soil = 1"}, "mosaic_soil"),
                        ({"extra": ", fractional_seaice = 1"}, "fractional_seaice = 1"),
                        ({"extra": ", sst_update = 1"}, "sst_update = 1 with sf_surface_physics = 3")):
            kw.setdefault("extra", "")
            err, _, _ = phys(**kw)
            self.assertIn(msg, " ".join(err), kw)

    def test_force_read_thompson(self):
        def phys(mp, extra):
            t = WrfTranslator(parse_namelist(real_case(f"""
            &time_control /
            &domains time_step = 60 /
            &physics mp_physics = {mp}, {extra} /
            &dynamics /
            """)))
            gen = t.translate()
            self.assertEqual(t.errors, [])
            self.assertFalse([w for w in t.warnings if "thompson" in w])
            return gen["phys"]
        self.assertTrue(phys(8, "force_read_thompson = .true.")["force_read_thompson"])
        self.assertNotIn("force_read_thompson", phys(8, "force_read_thompson = .false."))
        self.assertNotIn("force_read_thompson", phys(6, "force_read_thompson = .true."))  # read only by Thompson
        self.assertNotIn("write_thompson_tables", phys(8, "write_thompson_tables = .false."))

    def test_thompson_aerosol_aware(self):
        """mp_physics = 28 with aer_init_opt = 0: the thompsonaero scalars, wif_input_opt 0 / 1, grav_settling reset to 0
        (check_a_mundo); wif_input_opt = 2, use_aero_icbc, use_rap_aero_icbc, qna_update, dust_emis, MYNN and nests
        are errors."""
        def tr(extra, domains="", case=real_case):
            t = WrfTranslator(parse_namelist(case(f"&time_control /\n&domains time_step = 60{domains} /\n"
                                                  f"&physics mp_physics = 28{extra} /\n&dynamics /\n")))
            return t, t.translate()
        t, gen = tr(", sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1, grav_settling = 1",
                    ", wif_input_opt = 1")
        self.assertEqual(t.errors, [])
        self.assertEqual(gen["moist"]["species"], "qv,qc,qr,qi,qs,qg,qni,qnr,qnc,qnwfa,qnifa,qnbca")
        self.assertEqual((gen["phys"]["mp_physics"], gen["phys"]["wif_input_opt"]), (28, 1))
        self.assertNotIn("grav_settling", gen["phys"])
        self.assertTrue(any("grav_settling" in n and "reset to 0" in n for n in t.notices), t.notices)
        t, gen = tr(", force_read_thompson = .true.")
        self.assertEqual(t.errors, [])
        self.assertTrue(gen["phys"]["force_read_thompson"])
        self.assertNotIn("wif_input_opt", gen["phys"])
        for extra, domains, err in ((", sf_sfclay_physics = 1, bl_pbl_physics = 5", "", "bl_mynn_mixaerosols"),
                                    ("", ", wif_input_opt = 2", "wif_input_opt = 2"),
                                    (", use_aero_icbc = .true.", "", "use_aero_icbc"),
                                    (", use_rap_aero_icbc = .true.", "", "use_rap_aero_icbc"),
                                    (", qna_update = 1", "", "qna_update"),
                                    (", dust_emis = 1", "", "dust_emis")):
            t, _ = tr(extra, domains)
            self.assertIn(err, " ".join(t.errors), (extra, domains))
        # wif_input_opt is read only by the aerosol-aware schemes
        t = WrfTranslator(parse_namelist(real_case("&time_control /\n&domains time_step = 60, wif_input_opt = 1 /\n"
                                                   "&physics mp_physics = 8 /\n&dynamics /\n")))
        t.translate()
        self.assertEqual(t.errors, [])

    def test_opt_thcnd(self):
        base = """
        &time_control /
        &domains time_step = 60 /
        &physics sf_sfclay_physics = 1, sf_surface_physics = 2, num_soil_layers = 4, bl_pbl_physics = 1,
         opt_thcnd = {} /
        &dynamics /
        """
        t = WrfTranslator(parse_namelist(real_case(base.format(2))))
        gen = t.translate()
        self.assertEqual(t.errors, [])
        self.assertEqual(gen["phys"]["opt_thcnd"], 2)
        t = WrfTranslator(parse_namelist(real_case(base.format(1))))
        gen = t.translate()
        self.assertEqual(t.errors, [])
        self.assertNotIn("opt_thcnd", gen["phys"])
        t = WrfTranslator(parse_namelist(real_case(base.format(3))))
        t.translate()
        self.assertTrue(any("opt_thcnd = 3" in e for e in t.errors))

    def test_unported_dynamics_switches_rejected(self):
        for key, val in (("diff_6th_slopeopt", "1"), ("sfs_opt", "1"), ("m_opt", "1"), ("tracer_mix6_off", ".true.")):
            text = f"&time_control /\n&domains time_step = 60 /\n&physics /\n&dynamics {key} = {val} /\n"
            t = WrfTranslator(parse_namelist(real_case(text)))
            t.translate()
            self.assertTrue(any(key in e for e in t.errors), key)
        t = WrfTranslator(parse_namelist(real_case("&time_control /\n&domains time_step = 60 /\n&physics /\n&dynamics sfs_opt = 0 /\n")))
        t.translate()
        self.assertEqual(t.errors, [])

    def test_seaice_parameters_rejected(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &physics sf_sfclay_physics = 1, sf_surface_physics = 2, num_soil_layers = 4, bl_pbl_physics = 1,
         seaice_albedo_default = {} /
        &dynamics /
        """
        t = WrfTranslator(parse_namelist(real_case(text.format(0.65))))
        t.translate()
        self.assertEqual(t.errors, [])
        t = WrfTranslator(parse_namelist(real_case(text.format(0.8))))
        t.translate()
        self.assertTrue(any("seaice_albedo_default = 0.8" in e for e in t.errors))

    def test_extended_dynamics_options_mapping(self):
        text = """
        &time_control /
        &domains time_step = 60 /
        &dynamics
         diff_opt = 2,
         km_opt = 2,
         c_s = 0.18,
         c_k = 0.12,
         mix_isotropic = 1,
         mix_upper_bound = 0.2,
         mix_full_fields = .true.,
         tke_drag_coefficient = 0.01,
         tke_heat_flux = 0.05,
         tke_upper_bound = 500.0,
         moist_mix2_off = .true.,
         scalar_mix2_off = .true.,
         tke_mix2_off = .true.,
         moist_mix6_off = .true.,
         scalar_mix6_off = .true.,
         tke_mix6_off = .true.,
         w_damping = 1,
         w_crit_cfl = 1.2,
         scalar_adv_opt = 0,
         tke_adv_opt = 0,
         epssm = 0.2,
         smdiv = 0.15,
         emdiv = 0.02,
         etac = 0.25
        /
        """
        nml = parse_namelist(real_case(text))
        t = WrfTranslator(nml)
        gen = t.translate()
        self.assertEqual(t.errors, [])
        d = gen["dyn"]
        self.assertEqual(d["c_s"], 0.18)
        self.assertEqual(d["c_k"], 0.12)
        self.assertEqual(d["mix_isotropic"], 1)
        self.assertEqual(d["mix_upper_bound"], 0.2)
        self.assertEqual(d["mix_full_fields"], True)
        self.assertEqual(d["tke_drag_coefficient"], 0.01)
        self.assertEqual(d["tke_heat_flux"], 0.05)
        self.assertEqual(d["tke_upper_bound"], 500.0)
        self.assertEqual(d["moist_mix2_off"], True)
        self.assertEqual(d["scalar_mix2_off"], True)
        self.assertEqual(d["tke_mix2_off"], True)
        self.assertEqual(d["moist_mix6_off"], True)
        self.assertEqual(d["scalar_mix6_off"], True)
        self.assertEqual(d["tke_mix6_off"], True)
        self.assertEqual(d["w_crit_cfl"], 1.2)
        self.assertEqual(d["scalar_adv_opt"], 0)
        self.assertEqual(d["tke_adv_opt"], 0)
        self.assertEqual(d["epssm"], 0.2)
        self.assertEqual(d["smdiv"], 0.15)
        self.assertEqual(d["emdiv"], 0.02)
        self.assertEqual(d["etac"], 0.25)

    def test_physics_suite_conus(self):
        """WRF's default test/em_real namelist: physics_suite CONUS, -1 options, gwd_opt and fractional_seaice."""
        text = ("&time_control /\n&domains time_step = 90, max_dom = 1 /\n"
                "&physics physics_suite = 'CONUS', mp_physics = -1, cu_physics = -1, ra_lw_physics = -1,\n"
                " ra_sw_physics = -1, bl_pbl_physics = -1, sf_sfclay_physics = -1, sf_surface_physics = -1,\n"
                " radt = 15, fractional_seaice = 1 /\n&dynamics gwd_opt = 1, diff_opt = 2, km_opt = 4 /\n")
        t = WrfTranslator(parse_namelist(real_case(text)))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        expect = {"mp_physics": 8, "cu_physics": 6, "ra_lw_physics": 4, "ra_sw_physics": 4, "bl_pbl_physics": 2,
                  "sf_sfclay_physics": 2, "sf_surface_physics": 2, "fractional_seaice": 1}
        for k, v in expect.items():
            self.assertEqual(sec["phys"][k], v, k)
        self.assertEqual(sec["dyn"]["gwd_opt"], 1)

    def test_sst_update(self):
        """sst_update = 1: the auxinput4 interval and a custom wrflowinp name."""
        text = ("&time_control auxinput4_inname = 'lowinp_d<domain>', auxinput4_interval = 180,\n"
                " io_form_auxinput4 = 2 /\n&domains time_step = 60 /\n&physics sst_update = 1, mp_physics = 6, sf_sfclay_physics = 1,\n"
                " sf_surface_physics = 2, bl_pbl_physics = 1 /\n")
        t = WrfTranslator(parse_namelist(real_case(text)))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["sst_update"], 1)
        self.assertEqual(sec["phys"]["auxinput4_interval"], 180)
        self.assertEqual(sec["real"]["wrflowinp"], "lowinp_d01")
        # an interval below 0 (an alarm that never rings) is not supported
        t = WrfTranslator(parse_namelist(real_case(text.replace("auxinput4_interval = 180", "auxinput4_interval = -60"))))
        t.translate()
        self.assertTrue(any("auxinput4_interval < 0" in e for e in t.errors), t.errors)

    def test_sst_update_interval_and_time_step(self):
        """auxinput4_interval that is not a multiple of the time step: WRF's AUXINPUT4 alarm rings at the first step
        past the interval (ESMF_ClockAdvance), input_wrf finds no record of that time in wrflowinp and WRF stops with
        'Could not find matching time in input file' (share/input_wrf.F). An error, not a run that never reads the
        later records (6 h = 21600 s is not a multiple of 70 s; it is one of 60 s and of 50 s)."""
        text = ("&time_control auxinput4_inname = 'wrflowinp_d<domain>', auxinput4_interval = 360,\n"
                " io_form_auxinput4 = 2 /\n&domains time_step = {dt} /\n&physics sst_update = 1, mp_physics = 6,\n"
                " sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 1 /\n")
        for dt in (60, 50):
            t = WrfTranslator(parse_namelist(real_case(text.format(dt=dt))))
            t.translate()
            self.assertEqual(t.errors, [], dt)
        t = WrfTranslator(parse_namelist(real_case(text.format(dt=70))))
        t.translate()
        self.assertTrue(any("auxinput4_interval" in e and "time step" in e for e in t.errors), t.errors)

    def test_sst_skin_tmn_update(self):
        """sst_skin = 1 and tmn_update = 1 (with lagday) with Noah; the other LSMs, lagday = 1, values other than 0 / 1
        and the adaptive time step are errors; lagday without tmn_update has no effect."""
        base = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = 1,\n"
                " sf_surface_physics = {lsm}, bl_pbl_physics = 1, {opts} /\n")
        t = WrfTranslator(parse_namelist(real_case(base.format(lsm=2, opts="sst_skin = 1, tmn_update = 1"))))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["sst_skin"], 1)
        self.assertEqual(sec["phys"]["tmn_update"], 1)
        self.assertNotIn("lagday", sec["phys"])
        t = WrfTranslator(parse_namelist(real_case(base.format(lsm=2, opts="tmn_update = 1, lagday = 5"))))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["lagday"], 5)
        self.assertNotIn("sst_skin", sec["phys"])
        t = WrfTranslator(parse_namelist(real_case(base.format(lsm=2, opts="lagday = 5"))))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertNotIn("lagday", sec["phys"])
        self.assertTrue(any("lagday = 5" in n and "tmn_update = 0" in n for n in t.notices), t.notices)
        for lsm, opts, err in ((4, "sst_skin = 1", "sst_skin = 1 is supported with sf_surface_physics = 2"),
                               (1, "tmn_update = 1", "tmn_update = 1 is supported with sf_surface_physics = 2"),
                               (2, "tmn_update = 1, lagday = 1", "requires lagday=150"),
                               (2, "sst_skin = 2", "sst_skin = 2 is not supported")):
            t = WrfTranslator(parse_namelist(real_case(base.format(lsm=lsm, opts=opts))))
            t.translate()
            self.assertIn(err, " ".join(t.errors), f"{lsm} {opts}: {t.errors}")

    def test_physics_suite_tropical_keeps_explicit_values(self):
        text = ("&time_control /\n&domains time_step = 60 /\n"
                "&physics physics_suite = 'tropical', cu_physics = 1 /\n")
        t = WrfTranslator(parse_namelist(real_case(text)))
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["mp_physics"], 6)
        self.assertEqual(sec["phys"]["cu_physics"], 1)
        self.assertEqual(sec["phys"]["bl_pbl_physics"], 1)
        self.assertEqual(sec["phys"]["sf_sfclay_physics"], 1)

    def test_physics_suite_errors(self):
        for fragment, expected_err in [
                ("&physics mp_physics = -1 /", "needs a physics_suite"),
                ("&physics physics_suite = 'arctic' /", "not recognized"),
                # setup_physics_suite: TRIM drops the trailing blanks only
                ("&physics physics_suite = ' conus' /", "not recognized"),
                ("&physics physics_suite = 'co nus' /", "not recognized"),
                ("&dynamics gwd_opt = 3 /", "gwd_opt = 3"),
                ("&physics sf_sfclay_physics = 1, sf_surface_physics = 2, fractional_seaice = 2 /",
                 "fractional_seaice = 2"),
                ("&physics sf_sfclay_physics = 1, sf_surface_physics = 2, bl_pbl_physics = 2 /",
                 "use myjsfc scheme"),
                ("&physics sf_sfclay_physics = 2, sf_surface_physics = 2, bl_pbl_physics = 1 /",
                 "use sfclay scheme"),
                ("&physics sf_sfclay_physics = 2, sf_surface_physics = 2, bl_pbl_physics = 2, isfflx = 0 /",
                 "uninitialised memory")]:
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{fragment}\n")))
            t.translate()
            self.assertIn(expected_err, " ".join(t.errors), fragment)

    def test_nest_gwd_opt_equal_or_zero(self):
        """check_a_mundo: a nest's gwd_opt is domain 1's or 0."""
        doms = ("&domains time_step = 60, max_dom = 2, e_we = 100, 61, e_sn = 100, 61, e_vert = 45, 45,"
                " parent_id = 0, 1, i_parent_start = 1, 30, j_parent_start = 1, 30,"
                " parent_grid_ratio = 1, 3, parent_time_step_ratio = 1, 3 /\n")
        for gwd, bad in (("0, 1", True), ("1, 0", False), ("1", False), ("1, 1", False)):
            t = WrfTranslator(parse_namelist(f"&time_control /\n{doms}&dynamics gwd_opt = {gwd} /\n"))
            t.translate()
            self.assertEqual(any("gwd_opt must be equal for all domains" in e for e in t.errors), bad, gwd)

    def test_nest_mynn_options_checked_on_every_domain(self):
        """sf_mynn_sfcflux_land / _water are max_domains options: CPPWRF needs one value on all domains; the GFS
        forms (land 4, water 5) are not ported."""
        doms = ("&domains time_step = 60, max_dom = 2, e_we = 100, 61, e_sn = 100, 61, e_vert = 45, 45,"
                " parent_id = 0, 1, i_parent_start = 1, 30, j_parent_start = 1, 30,"
                " parent_grid_ratio = 1, 3, parent_time_step_ratio = 1, 3 /\n")
        phys = "mp_physics = 6, 6, sf_sfclay_physics = 5, 5, sf_surface_physics = 2, 2, bl_pbl_physics = 5, 5"
        for opt, bad in (("sf_mynn_sfcflux_water = 1, 0", True), ("sf_mynn_sfcflux_water = 1, 1", False),
                         ("sf_mynn_sfcflux_water = 1", False), ("sf_mynn_sfcflux_land = 0, 2", True),
                         ("sf_mynn_sfcflux_land = 0", False), ("sf_mynn_sfcflux_water = 0, 0", False),
                         ("sf_mynn_sfcflux_land = 2, 2", False), ("sf_mynn_sfcflux_water = 2*5", True),
                         ("sf_mynn_sfcflux_land = 2*4", True)):
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n{doms}&physics {phys}, {opt} /\n")))
            t.translate()
            self.assertEqual(any("sf_mynn_sfcflux" in e for e in t.errors), bad, (opt, t.errors))

    def test_mynn_pbl_surface_layers(self):
        """module_physics_init.F pbl_init: the MYNN PBL runs with sf_sfclay_physics 5, 1 or 2 and stops otherwise;
        CPPWRF rejects it with fractional_seaice = 1 (sfclayrev / MYJSFC sea-ice wrappers not verified with MYNN)."""
        for sf, extra, err in (("1", "", None), ("2", "", None), ("5", "", None),
                               ("0", "", "needs sf_sfclay_physics = 1, 2 or 5"),
                               ("1", ", fractional_seaice = 1", "fractional_seaice = 1 is not supported"),
                               ("2", ", fractional_seaice = 1", "fractional_seaice = 1 is not supported")):
            phys = f"&physics mp_physics = 8, sf_sfclay_physics = {sf}, sf_surface_physics = 2, bl_pbl_physics = 5{extra} /"
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{phys}\n")))
            cfg = t.translate()
            if err is None:
                self.assertEqual(t.errors, [], (sf, extra))
                self.assertEqual((cfg["phys"]["bl_pbl_physics"], cfg["phys"]["sf_sfclay_physics"]), (5, int(sf)))
            else:
                self.assertIn(err, " ".join(t.errors), (sf, extra))

    def test_dudhia_shortwave(self):
        # RRTMG LW with Dudhia SW (module_ra_sw.F); swrad_scat (swinit) only with Dudhia
        for extra, scat in (("", None), (", swrad_scat = 0.5", 0.5)):
            phys = f"&physics ra_lw_physics = 4, ra_sw_physics = 1, radt = 15, sf_sfclay_physics = 1, sf_surface_physics = 2{extra} /"
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{phys}\n")))
            cfg = t.translate()
            self.assertEqual(t.errors, [], extra)
            self.assertEqual((cfg["phys"]["ra_lw_physics"], cfg["phys"]["ra_sw_physics"]), (4, 1))
            self.assertEqual(cfg["phys"].get("swrad_scat"), scat)
        phys = "&physics ra_lw_physics = 4, ra_sw_physics = 4, sf_sfclay_physics = 1, sf_surface_physics = 2, swrad_scat = 0.5 /"
        t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{phys}\n")))
        cfg = t.translate()
        self.assertEqual(t.errors, [])
        self.assertNotIn("swrad_scat", cfg["phys"])

    def test_rrtm_longwave(self):
        # RRTM LW (module_ra_rrtm.F) with RRTMG or Dudhia SW
        for sw in (4, 1):
            phys = f"&physics ra_lw_physics = 1, ra_sw_physics = {sw}, radt = 15, sf_sfclay_physics = 1, sf_surface_physics = 2 /"
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{phys}\n")))
            cfg = t.translate()
            self.assertEqual(t.errors, [], sw)
            self.assertEqual((cfg["phys"]["ra_lw_physics"], cfg["phys"]["ra_sw_physics"]), (1, sw))

    def test_monotonic_advection(self):
        # moist_adv_opt / scalar_adv_opt / tke_adv_opt = 2: advect_scalar_mono on the last RK stage (module_em.F)
        t = WrfTranslator(parse_namelist(real_case(
            "&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, bl_pbl_physics = 1, sf_sfclay_physics = 1, sf_surface_physics = 2 /\n"
            "&dynamics moist_adv_opt = 2, scalar_adv_opt = 2, diff_opt = 2, km_opt = 2, tke_adv_opt = 2 /\n")))
        d = t.translate()["dyn"]
        self.assertEqual(t.errors, [])
        self.assertEqual((d["moist_adv_opt"], d["scalar_adv_opt"], d["tke_adv_opt"]), (2, 2, 2))

    def test_weno_advection(self):
        # 3 / 4: advect_scalar_weno / advect_scalar_wenopd on the last RK stage (any h/v_sca_adv_order)
        for opt in (3, 4):
            t = WrfTranslator(parse_namelist(real_case(
                "&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, bl_pbl_physics = 1, "
                f"sf_sfclay_physics = 1, sf_surface_physics = 2 /\n&dynamics moist_adv_opt = {opt}, scalar_adv_opt = {opt}, diff_opt = 2, "
                f"km_opt = 2, tke_adv_opt = {opt} /\n")))
            d = t.translate()["dyn"]
            self.assertEqual(t.errors, [])
            self.assertEqual((d["moist_adv_opt"], d["scalar_adv_opt"], d["tke_adv_opt"]), (opt, opt, opt))

    def test_weno_momentum_advection(self):
        # momentum_adv_opt 3: advect_weno_u / _v / _w on the last RK stage; 1 is the default (not written)
        base = "&time_control /\n&domains time_step = 60 /\n&physics sf_sfclay_physics = 1, sf_surface_physics = 2 /\n"
        for opt, want, err in ((3, 3, None), (1, None, None), (2, None, "momentum_adv_opt = 2"),
                               (4, None, "momentum_adv_opt = 4")):
            t = WrfTranslator(parse_namelist(real_case(base + f"&dynamics momentum_adv_opt = {opt} /\n")))
            d = t.translate()["dyn"]
            if err:
                self.assertIn(err, " ".join(t.errors), opt)
            else:
                self.assertEqual(t.errors, [], opt)
                self.assertEqual(d.get("momentum_adv_opt"), want, opt)

    def test_unsupported_physics_and_dynamics_errors(self):
        cases = [
            # share/module_check_a_mundo.F: "bl_mynn_edmf > 0 requires both shcu_physics=0 & ishallow=0"
            ("&physics mp_physics = 6, sf_sfclay_physics = 5, bl_pbl_physics = 5, ishallow = 1 /",
             "requires shcu_physics = 0 and ishallow = 0"),
            ("&physics cu_physics = 3, clos_choice = 99 /", "clos_choice = 99"),
            ("&physics cu_physics = 1, kfeta_trigger = 2 /", "kfeta_trigger = 2"),
            ("&physics sf_surface_physics = 4, opt_crop = 1 /", "opt_crop = 1"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 0 /", "a longwave scheme needs a shortwave scheme"),
            ("&physics ra_lw_physics = 0, ra_sw_physics = 1 /", "a longwave scheme needs a shortwave scheme"),
            ("&physics ra_lw_physics = 1, ra_sw_physics = 0 /", "a longwave scheme needs a shortwave scheme"),
            ("&physics ra_lw_physics = 1, ra_sw_physics = 2 /", "ra_sw_physics = 2 is not supported"),
            ("&physics ra_lw_physics = 3, ra_sw_physics = 4 /", "ra_lw_physics = 3 is not supported"),
            ("&physics ra_lw_physics = 3, ra_sw_physics = 3 /", "CAM radiation (ra_lw_physics = 3 / ra_sw_physics = 3) "
             "is not ported"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 3 /", "CAM radiation"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 2 /", "ra_sw_physics = 2 is not supported"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 1, cu_physics = 3, cu_rad_feedback = .true. /",
             "cu_rad_feedback with Grell-Freitas cumulus and the Dudhia shortwave"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 4, aer_opt = 1 /", "aer_opt = 1"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 4, cldovrlp = 9 /", "cldovrlp = 9"),
            ("&physics ra_lw_physics = 4, ra_sw_physics = 4, cldovrlp = -1 /", "cldovrlp = -1"),
            ("&physics shcu_physics = 2 /", "shcu_physics"),
            # the diagnostics name WRF's group (Registry.EM_COMMON: shalwater_z0 &physics, scm_force_flux &scm)
            ("&physics shalwater_z0 = 2 /", "WRF &physics.shalwater_z0 = 2 (Shallow-water roughness"),
            ("&scm scm_force_flux = 1 /", "WRF &scm.scm_force_flux = 1 (Single-column model prescribed surface flux"),
            ("&physics topo_wind = 3 /", "topo_wind = [3]"),
            ("&dynamics moist_adv_opt = 5 /", "moist_adv_opt = 5"),
            ("&dynamics scalar_adv_opt = -1 /", "scalar_adv_opt = -1"),
            ("&dynamics diff_opt = 2, km_opt = 2, tke_adv_opt = 5 /", "tke_adv_opt = 5"),
            # module_advect_em.F advect_scalar_mono stops for h_sca_adv_order /= 5 and v_sca_adv_order /= 3, 5
            ("&dynamics moist_adv_opt = 2, h_mom_adv_order = 6, h_sca_adv_order = 6 /", "needs h_sca_adv_order = 5"),
            ("&dynamics tke_adv_opt = 2, v_mom_adv_order = 4, v_sca_adv_order = 4 /", "needs h_sca_adv_order = 5"),
            ("&dynamics diff_6th_opt = 3 /", "diff_6th_opt = 3"),
            ("&dynamics h_mom_adv_order = 5, h_sca_adv_order = 6 /", "h_sca_adv_order = 6"),
            ("&dynamics hybrid_opt = 9 /", "hybrid_opt = 9"),
        ]
        for fragment, expected_err in cases:
            text = f"&time_control /\n&domains time_step = 60 /\n{fragment}\n"
            nml = parse_namelist(real_case(text))
            t = WrfTranslator(nml)
            t.translate()
            joined = " ".join(t.errors)
            self.assertIn(expected_err, joined, f"Expected '{expected_err}' in errors for {fragment}, got {t.errors}")

    def test_real_case_needs_specified(self):
        # specified defaults to .false. (Registry); a real case without it has no lateral boundary condition on
        # domain 1 and wrf.exe stops in share/module_bc.F boundary_condition_check
        text = complete("&time_control /\n&domains time_step = 60 /\n")
        t = WrfTranslator(parse_namelist(text))
        t.translate()
        self.assertEqual(len(t.errors), 1, t.errors)
        self.assertIn("pass --case", t.errors[0])
        t = WrfTranslator(parse_namelist(text), override_case="real")
        t.translate()
        self.assertEqual(len(t.errors), 1, t.errors)
        self.assertIn("specified = .false. in a real case", t.errors[0])
        t = WrfTranslator(parse_namelist(real_case(text)), override_case="real")
        self.assertEqual(t.translate()["run"]["case"], "real")
        self.assertEqual(t.errors, [])

    def test_ishallow_without_grell_freitas(self):
        # WRF passes ishallow to the G3 and GF schemes only (phys/module_cumulus_driver.F) and check_a_mundo has no
        # rule for it with other cumulus schemes: Kain-Fritsch with ishallow = 1 runs as without it
        t = WrfTranslator(parse_namelist(real_case("&time_control /\n&domains time_step = 60 /\n"
                                                   "&physics sf_sfclay_physics = 1, sf_surface_physics = 2, cu_physics = 1, ishallow = 1 /\n")))
        t.translate()
        self.assertEqual(t.errors, [])

    def test_invalid_nest_configurations_errors(self):
        base = """
        &time_control input_from_file = .true., .true. /
        &domains
         time_step = 60, max_dom = 2, dx = 30000, 10000, dy = 30000, 10000,
         e_we = 120, {e_we}, e_sn = 90, 91, e_vert = 45, {e_vert},
         parent_id = 0, 1, i_parent_start = 1, {i_start}, j_parent_start = 1, 30,
         parent_grid_ratio = 1, 3, parent_time_step_ratio = 1, 3{extra}
        /
        &bdy_control specified = .true., .false. /
        """

        def errors(e_we=121, e_vert=45, i_start=40, extra=""):
            t = WrfTranslator(parse_namelist(complete(base.format(e_we=e_we, e_vert=e_vert, i_start=i_start, extra=extra))))
            t.translate()
            return t.errors

        self.assertEqual(errors(), [])
        # 1. (e_we - s_we) not a multiple of parent_grid_ratio: "Nested dimensions are illegal" (dyn_em/start_em.F)
        e = errors(e_we=120)
        self.assertEqual(len(e), 1, e)
        self.assertIn("not divisible by parent_grid_ratio", e[0])

        # 2. the nest (40 parent cells from i_parent_start) must end 2 parent cells inside d01 (nx = 119): SINT's
        # stencil reaches 2 cells beyond the nest; 78 + 40 - 1 = 117 is the last allowed end
        self.assertEqual(errors(i_start=78), [])
        for i_start in (79, 100):
            e = errors(i_start=i_start)
            self.assertEqual(len(e), 1, e)
            self.assertIn("reaches within 2 cells of parent domain d01's edge", e[0])

        # 3. Mismatched e_vert and invalid smooth_option
        e = errors(e_vert=60, extra=",\n         smooth_option = 5")
        self.assertTrue(any("e_vert" in x for x in e), e)
        self.assertTrue(any("smooth_option = 5" in x for x in e), e)

        # 4. Noah-MP with nests: wrf.exe 4.8.0 aborts
        t = WrfTranslator(parse_namelist(complete(base.format(e_we=121, e_vert=45, i_start=40, extra="") +
                                                  "&physics sf_surface_physics = 4, 4, sf_sfclay_physics = 1, 1 /\n")))
        t.translate()
        self.assertTrue(any("Noah-MP" in x for x in t.errors), t.errors)


class TestWrfNamelistSemantics(unittest.TestCase):
    """WRF's namelist read, check_a_mundo, start_em and input_wrf rules (variants_pm_R32_*.txt on the VM)."""

    NEST = ("&time_control input_from_file = .true., .true. /\n"
            "&domains time_step = 60, max_dom = 2, e_we = 120, 121, e_sn = 90, 91, e_vert = 45, 45, dx = 15000,"
            " dy = 15000, parent_id = 0, 1, i_parent_start = 1, 40, j_parent_start = 1, 30, parent_grid_ratio = 1, 3,"
            " parent_time_step_ratio = 1, 3 /\n&bdy_control specified = .true., .false. /\n")
    FULL = ("&time_control /\n&domains time_step = 60, dx = 15000, dy = 15000 /\n"
            "&physics mp_physics = 0, cu_physics = 0, ra_lw_physics = 0, ra_sw_physics = 0, bl_pbl_physics = 0,"
            " sf_sfclay_physics = 0, sf_surface_physics = 0 /\n&dynamics diff_opt = 0, km_opt = 1 /\n"
            "&bdy_control specified = .true. /\n")

    @staticmethod
    def translate(text):
        t = WrfTranslator(parse_namelist(complete(text)))
        return t, t.translate()

    @staticmethod
    def errors(text):
        t = WrfTranslator(parse_namelist(text))
        t.translate()
        return t.errors

    def test_parser_fortran_semantics(self):
        # config_reads.inc: REWIND and one READ per group (a repeated group is not read); key(i) assigns from
        # element i, a repeated key again from its first element, null values keep the element
        nml = parse_namelist("&physics cudt = 5, 5, cudt(2) = 0, radt = , 10, bldt = 1, bldt = 2 /\n"
                             "&dynamics km_opt = 2*, 4 /\n&physics cudt = 9 /\n")
        self.assertEqual(nml["physics"]["cudt"], [5, 0])
        self.assertEqual(nml["physics"]["radt"], [None, 10])
        self.assertEqual(nml["physics"]["bldt"], [2])
        self.assertEqual(nml["dynamics"]["km_opt"], [None, None, 4])
        # gfortran's logical input: an optional '.', then T or F (true, F, .tru); a number is an error
        nml = parse_namelist("&bdy_control specified = true, nested = F, periodic_x = .tru /\n")
        self.assertEqual(nml["bdy_control"], {"specified": [True], "nested": [False], "periodic_x": [True]})
        with self.assertRaises(NamelistParseError):
            parse_namelist("&bdy_control specified = 1 /\n")
        # radt(1) unset: the Registry default 0
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 2*1, radt(2) = 3 /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("radt", sec["phys"])
        self.assertEqual(sec["d02"]["phys_radt"], 3)

    def test_iofields_input_streams(self):
        # modify_io_masks1 changes what input_wrf reads from any input stream; CPPWRF reads wrfinput (i:0), wrflowinp
        # (i:4) and wrffdda (i:10) whole, also with --format native (no WrfHistory to check it): an error each;
        # output stream changes (h:0, ...) and other input streams pass
        with tempfile.TemporaryDirectory() as d:
            for line, bad in (("-:i:10:q_ndg_old,q_ndg_new", True), ("+:i:4:sst", True), ("-:i:0:tsk", True),
                              ("+:i:010:u_ndg_old", True), ("-:h:0:rainc", False), ("+:i:11:sst", False)):
                path = os.path.join(d, "iof.txt")
                with open(path, "w") as f:
                    f.write(line + "\n")
                t, _ = self.translate(self.FULL.replace("&time_control /",
                                                        f"&time_control iofields_filename = '{path}' /"))
                hit = [e for e in t.errors if "input stream changes" in e]
                self.assertEqual(bool(hit), bad, (line, t.errors))
                if bad:
                    self.assertIn("wrffdda", hit[0])
            # READ(strmid,'(I3)'): -0 is stream 0; a line of an unknown stream type keeps the last known one
            # (strmtyp_int); ++0 is a Fortran read error
            for text, want in (("-:i:-0:tsk\n", "input stream changes"), ("+:i:11:sst\n-:x:10:u_ndg_old\n", "input stream changes"),
                               ("+:h:0:rainc\n-:x:10:u_ndg_old\n", None), ("+:i:++0:tsk\n", "bad stream id"),
                               # get_fieldstr: the stream type of a line with a bad operation still counts; a
                               # missing field list is no error, the stream id is still read (++0, ':ts' of
                               # '+:i::tsk': read errors); ':0' of '+:i::0:tsk' too; Unicode digits are no digits
                               ("+++:i:11:sst\n-:x:0:tsk\n", "input stream changes"), ("+:i:++0\n", "bad stream id"),
                               ("+:i::tsk\n", "bad stream id"), ("+:i::0:tsk\n", "bad stream id"),
                               ("+:i:\u00b2:tsk\n", "bad stream id"), ("+:i:0\n", None)):
                path = os.path.join(d, "iof.txt")
                with open(path, "w") as f:
                    f.write(text)
                t, _ = self.translate(self.FULL.replace("&time_control /",
                                                        f"&time_control iofields_filename = '{path}' /"))
                if want:
                    self.assertTrue(any(want in e for e in t.errors), (text, t.errors))
                else:
                    self.assertFalse(any("stream" in e for e in t.errors), (text, t.errors))

    def test_urban_canopy(self):
        # sf_urban_physics = 1 (single-layer UCM) with Noah on one domain, URBPARM.TBL or URBPARM_LCZ.TBL
        # (use_wudapt_lcz = 1); BEP / BEP+BEM, other land surfaces, distributed drag,
        # nests and restarts are errors
        noah = self.FULL.replace("bl_pbl_physics = 0,", "bl_pbl_physics = 1,").replace(
            "sf_sfclay_physics = 0, sf_surface_physics = 0", "sf_sfclay_physics = 1, sf_surface_physics = 2")
        t, sec = self.translate(noah.replace("&physics ", "&physics sf_urban_physics = 1, use_wudapt_lcz = 1, "))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["sf_urban_physics"], 1)
        self.assertEqual(sec["phys"]["use_wudapt_lcz"], 1)
        self.assertFalse(any("verification tier" in w for w in t.warnings), t.warnings)
        t, sec = self.translate(noah.replace("&physics ", "&physics sf_urban_physics = 1, "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("use_wudapt_lcz", sec["phys"])
        for extra, msg in (("sf_urban_physics = 2", "BEP urban canopy"), ("sf_urban_physics = 3", "BEP+BEM"),
                           ("sf_urban_physics = 1, slucm_distributed_drag = .true.", "slucm_distributed_drag"),
                           ("sf_urban_physics = 1, use_wudapt_lcz = 2", "use_wudapt_lcz = 2"),
                           ("use_wudapt_lcz = 1", "use_wudapt_lcz")):
            errs = self.errors(complete(noah.replace("&physics ", "&physics " + extra + ", ")))
            self.assertTrue(any(msg in e for e in errs), (extra, errs))
        slab = self.FULL.replace("sf_sfclay_physics = 0, sf_surface_physics = 0", "sf_sfclay_physics = 1, "
                                 "sf_surface_physics = 1").replace("&physics ", "&physics sf_urban_physics = 1, ")
        self.assertTrue(any("Noah LSM" in e for e in self.errors(complete(slab))))
        rst = noah.replace("&physics ", "&physics sf_urban_physics = 1, ").replace("&time_control /",
                                                                                  "&time_control restart = .true. /")
        self.assertTrue(any("restart = .true. with sf_urban_physics" in e for e in self.errors(complete(rst))))

    def test_terrain_options(self):
        # topo_wind 1 / 2, slope_rad 1, topo_shading 1 (with shadlen) on one real-data domain; other values, nests and
        # slope_rad / topo_shading with the adaptive time step are errors
        noah = self.FULL.replace("bl_pbl_physics = 0,", "bl_pbl_physics = 1,").replace(
            "sf_sfclay_physics = 0, sf_surface_physics = 0", "sf_sfclay_physics = 1, sf_surface_physics = 2")
        t, sec = self.translate(noah.replace("&physics ", "&physics topo_wind = 2, slope_rad = 1, topo_shading = 1, "
                                                          "shadlen = 50000., "))
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["phys"]["topo_wind"], sec["phys"]["slope_rad"], sec["phys"]["topo_shading"]), (2, 1, 1))
        self.assertEqual(sec["phys"]["shadlen"], 50000.)
        t, sec = self.translate(noah.replace("&physics ", "&physics topo_wind = 1, "))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["topo_wind"], 1)
        self.assertNotIn("slope_rad", sec["phys"])
        for extra, msg in (("topo_wind = 3", "topo_wind = [3]"), ("slope_rad = 2", "slope_rad = [2]"),
                           ("topo_shading = 2", "topo_shading = [2]")):
            errs = self.errors(complete(noah.replace("&physics ", "&physics " + extra + ", ")))
            self.assertTrue(any(msg in e for e in errs), (extra, errs))
        errs = self.errors(complete(self.NEST + "&physics topo_wind = 1, 1 /\n"))
        self.assertTrue(any("with nests" in e for e in errs), errs)
        ad = noah.replace("&physics ", "&physics slope_rad = 1, ").replace("&domains ", "&domains use_adaptive_time_step = .true., ")
        self.assertTrue(any("use_adaptive_time_step are not" in e for e in self.errors(complete(ad))))

    def test_cu_rad_feedback_cldovrlp_0(self):
        # cldovrlp 0: RRTMG without clouds; cu_rad_feedback per domain (phy_init's icloud_cu); shallowcu_forced_ra
        # acts only with KF-CuP (cu_physics 10)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 1, 1, ra_lw_physics = 2*4, ra_sw_physics = 2*4, "
                                "cldovrlp = 0, cu_rad_feedback = .true., .false., shallowcu_forced_ra = 2*.true. /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["cldovrlp"], 0)
        self.assertEqual(sec["phys"]["cu_rad_feedback"], True)
        self.assertEqual(sec["d02"]["phys_cu_rad_feedback"], False)
        # a nest without an entry: the Registry default .false.
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 3, 3, cu_rad_feedback = .true. /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["d02"]["phys_cu_rad_feedback"], False)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 6, 6, cu_rad_feedback = .false., .true. /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("cu_rad_feedback", sec["phys"])
        self.assertEqual(sec["d02"]["phys_cu_rad_feedback"], True)

    def test_bmj_and_bmj_rad_feedback(self):
        # cu_physics = 2 (BMJ, verified: no tier warning); bmj_rad_feedback per domain, used by BMJ only; with
        # the radiation it needs cloud water and ice (QCCONV / QICONV go into the radiation's QC / QI)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 2, 2, bmj_rad_feedback = .true., .false. /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["cu_physics"], 2)
        self.assertEqual(sec["phys"]["bmj_rad_feedback"], True)
        self.assertEqual(sec["d02"]["phys_bmj_rad_feedback"], False)
        self.assertFalse(any("verification tier" in w for w in t.warnings), t.warnings)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 1, 1, bmj_rad_feedback = 2*.true. /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("bmj_rad_feedback", sec["phys"])
        self.assertTrue(any("bmj_rad_feedback" in n and "no effect" in n for n in t.notices), t.notices)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 2, 2, mp_physics = 2*1, ra_lw_physics = 2*4, "
                              "ra_sw_physics = 2*4, bmj_rad_feedback = .true. /\n")
        self.assertTrue(any("bmj_rad_feedback = .true. with the radiation and mp_physics = 1" in e for e in t.errors),
                        t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 2, 2, mp_physics = 2*6, ra_lw_physics = 2*4, "
                              "ra_sw_physics = 2*4, bmj_rad_feedback = .true. /\n")
        self.assertEqual(t.errors, [])

    def test_grell_3d(self):
        # cu_physics = 5 (Grell-3D, verified: no tier warning); cugd_avedx 1 / 3
        # (other values rejected), the ensemble sizes fixed; ishallow and clos_choice as with Grell-Freitas;
        # cu_physics = 93 (Grell-Devenyi) is not ported
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 5, 5, ishallow = 1, cugd_avedx = 3, "
                                "clos_choice = 3 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["cu_physics"], 5)
        self.assertEqual(sec["phys"]["ishallow"], 1)
        self.assertEqual(sec["phys"]["cugd_avedx"], 3)
        self.assertEqual(sec["phys"]["clos_choice"], 3)
        self.assertFalse(any("verification tier" in w for w in t.warnings), t.warnings)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 5, 5 /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("cugd_avedx", sec["phys"])
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 5, 5, cugd_avedx = 2 /\n")
        self.assertTrue(any("cugd_avedx = 2" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 5, 5, maxens3 = 12 /\n")
        self.assertTrue(any("maxens3 = 12" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 93, 93 /\n")
        self.assertTrue(any("Grell-Devenyi" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 5, 5, ra_sw_physics = 2*1, cu_rad_feedback = .true. /\n")
        self.assertTrue(any("Dudhia shortwave" in e for e in t.errors), t.errors)

    def test_values_wrf_runs_differently(self):
        """Values the translator accepted while wrf.exe runs (or stops) otherwise."""
        def errs(text):
            t, _ = self.translate(text)
            return " ".join(t.errors)
        real = "&domains time_step = 60 /\n&bdy_control specified = .true. /\n"
        # use_theta_m: only 1 (2 mixes the == 1 and /= 0 tests of the dycore)
        self.assertIn("use_theta_m = 2", errs(real + "&dynamics use_theta_m = 2 /\n"))
        # km_opt = 0 with diffusion: calculate_km_kh's CASE DEFAULT stops wrf.exe
        self.assertIn("km_opt = 0", errs(real + "&dynamics diff_opt = 1, km_opt = 0 /\n"))
        self.assertNotIn("km_opt", errs(real + "&dynamics diff_opt = 0, km_opt = 0 /\n"))
        self.assertIn("km_opt = 0", errs(self.NEST + "&dynamics diff_opt = 1, 1, km_opt = 4, 0 /\n"))
        # wif_input_opt = 2 adds QNBCA / QNBCA2D whatever mp_physics; 1 has no effect without mp_physics 28
        self.assertIn("wif_input_opt = 2", errs(real.replace("/\n", ", wif_input_opt = 2 /\n", 1) + "&physics mp_physics = 6 /\n"))
        self.assertIn("wif_input_opt = 3", errs(real.replace("/\n", ", wif_input_opt = 3 /\n", 1) + "&physics mp_physics = 6 /\n"))
        self.assertEqual(errs(real.replace("/\n", ", wif_input_opt = 1 /\n", 1) + "&physics mp_physics = 6 /\n"), "")
        # spec_bdy_width other than 5: refused with nests too (d01 reads a 5-point wrfbdy)
        self.assertIn("spec_bdy_width = 7", errs(self.NEST.replace(".false. /", ".false., spec_bdy_width = 7 /")))
        # check_a_mundo: cu_rad_feedback gives cu_diag = 1 on every such domain when d01 runs GF / G3 / GD
        self.assertIn("cu_diag = 1 on domain 2", errs(
            self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 3, 0, cu_rad_feedback = .true., .true., sf_sfclay_physics = 1, 1,"
            " sf_surface_physics = 2, 2, bl_pbl_physics = 1, 1 /\n"))
        # a nest without an e_vert entry has the Registry default 31
        nest_ev = self.NEST.replace("e_vert = 45, 45,", "e_vert = 45,")
        self.assertIn("e_vert = 31 must match parent domain e_vert = 45", errs(nest_ev))
        # the domain start index of a real case
        self.assertIn("s_we other than 1", errs(real.replace("/\n", ", s_we = 2 /\n", 1)))
        # sst_update: NetCDF wrflowinp with a name of <domain> only (Registry default auxinput4_d<domain>_<date>)
        sst = real + "&physics sst_update = 1, sf_surface_physics = 2 /\n"
        self.assertIn("auxinput4_inname", errs(sst + "&time_control io_form_auxinput4 = 2, auxinput4_interval = 360 /\n"))
        self.assertIn("io_form_auxinput4 = 1", errs(
            sst + "&time_control io_form_auxinput4 = 1, auxinput4_interval = 360,"
            " auxinput4_inname = 'wrflowinp_d<domain>' /\n"))
        self.assertNotIn("auxinput4", errs(
            sst + "&time_control io_form_auxinput4 = 2, auxinput4_interval = 360,"
            " auxinput4_inname = 'wrflowinp_d<domain>' /\n"))

    def test_tslist_in_run_dir(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            nml = os.path.join(d, "namelist.input")
            with open(nml, "w") as f:
                f.write(complete("&domains time_step = 60 /\n&bdy_control specified = .true. /\n"))
            t = WrfTranslator(parse_namelist(open(nml).read()), namelist_path=nml)
            t.translate()
            self.assertFalse(any("tslist" in e for e in t.errors), t.errors)
            open(os.path.join(d, "tslist"), "w").close()
            t = WrfTranslator(parse_namelist(open(nml).read()), namelist_path=nml)
            t.translate()
            self.assertTrue(any("tslist" in e for e in t.errors), t.errors)

    def test_mskf_and_unported_kf_variants(self):
        # cu_physics = 11 (multi-scale Kain-Fritsch) is ported; 10 (KF-CuP) and 99 (old Kain-Fritsch) are not
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 11, 11, kfeta_trigger = 3 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["cu_physics"], 11)
        self.assertEqual(sec["phys"]["kfeta_trigger"], 3)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 11, 11, kfeta_trigger = 2 /\n")
        self.assertTrue(any("kfeta_trigger = 2" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 11, 11, aercu_opt = 1 /\n")
        self.assertTrue(any("aercu_opt" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 11, 11, kf_edrates = 1, 1 /\n")
        self.assertTrue(any("kf_edrates" in e for e in t.errors), t.errors)
        # check_a_mundo: MSKF on any domain sets icloud = 1 for every domain (a nest without cumulus included)
        # (the nest keeps column physics: a parent with it and a nest without is refused)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 11, 0, icloud = 0, ra_lw_physics = 4, 4,"
                                " ra_sw_physics = 4, 4, sf_sfclay_physics = 1, 1, sf_surface_physics = 2, 2,"
                                " bl_pbl_physics = 1, 1 /\n")
        self.assertEqual(t.errors, [])
        self.assertTrue(all("icloud" not in v for v in sec.values() if isinstance(v, dict)), sec)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 1, 0, icloud = 0, ra_lw_physics = 4, 4,"
                                " ra_sw_physics = 4, 4, sf_sfclay_physics = 1, 1, sf_surface_physics = 2, 2,"
                                " bl_pbl_physics = 1, 1 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["icloud"], 0)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 10, 10 /\n")
        self.assertTrue(any("KF-CuP" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 99, 99 /\n")
        self.assertTrue(any("old Kain-Fritsch" in e for e in t.errors), t.errors)

    def test_nest_entries_take_registry_defaults(self):
        # r32cudt1 / r32bldt1: a nest without a cudt / bldt entry calls the scheme every step (Registry default 0,
        # rconfig CUDT / BLDT in upper case)
        t, sec = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 1, 1, cudt = 5, bldt = 1 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["cudt"], 5)
        self.assertEqual(sec["d02"]["phys_cudt"], 0)
        self.assertEqual(sec["d02"]["phys_bldt"], 0)

    def test_mp_physics_of_the_highest_domain(self):
        # frame/module_configure.F: every domain runs mp_physics(max_dom)
        t, sec = self.translate(self.NEST + "&physics mp_physics = 8, 6 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["mp_physics"], 6)
        t, _ = self.translate(self.NEST + "&physics mp_physics = 6, bl_pbl_physics = 1, sf_sfclay_physics = 1, sf_surface_physics = 2 /\n")  # domain 2 has the default -1
        self.assertTrue(any("mp_physics = -1" in e for e in t.errors), t.errors)

    def test_cu_physics_on_a_nest(self):
        # check_a_mundo: a nest's cu_physics is domain 1's or 0
        sfc = "sf_sfclay_physics = 1, 1, sf_surface_physics = 2, 2, bl_pbl_physics = 1, 1, "
        t, _ = self.translate(self.NEST + f"&physics {sfc}cu_physics = 1, 0 /\n")
        self.assertEqual(t.errors, [])
        t, _ = self.translate(self.NEST + f"&physics {sfc}cu_physics = 0, 1 /\n")
        self.assertTrue(any("cu_physics must be equal" in e for e in t.errors), t.errors)
        # the cumulus scheme alone on the parent: the nest has no column physics, whose state arrays WRF feeds back
        t, _ = self.translate(self.NEST + "&physics cu_physics = 1, 0 /\n")
        self.assertTrue(any("no surface layer, LSM or PBL" in e for e in t.errors), t.errors)
        # the cumulus scheme alone on both domains: cppwrf's column physics suite needs a surface layer and an LSM
        t, _ = self.translate(self.NEST + "&physics cu_physics = 1, 1 /\n")
        self.assertTrue(any("PhysicsSuite: only sf_sfclay_physics" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST + "&physics sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, cu_physics = 1, 1 /\n")
        self.assertEqual(t.errors, [])

    def test_options_wrf_needs(self):
        self.assertEqual(self.errors(self.FULL), [])
        for text, err in ((self.FULL.replace("time_step = 60, ", ""), "time_step is not set"),
                          (self.FULL.replace("diff_opt = 0, ", ""), "diff_opt and km_opt must both be set"),
                          (self.FULL.replace("mp_physics = 0, ", ""), "mp_physics = -1"),
                          (self.FULL.replace("time_step = 60", "time_step = 100"), "reasonable_time_step_ratio"),
                          (self.FULL.replace("dx = 15000, dy = 15000", ""), "reasonable_time_step_ratio")):
            self.assertTrue(any(err in e for e in self.errors(text)), (text, err))
        self.assertEqual(self.errors(self.FULL.replace("time_step = 60", "time_step = 100, reasonable_time_step_ratio = 7")), [])

    def test_unported_options(self):
        # r32tempo: WRF writes QNCLOUD, QNWFA, QNIFA; r32cor / r32curv: do_coriolis, do_curvature are not used by
        # wrf.exe; r32nssl: check_a_mundo turns the NSSL options off without NSSL microphysics
        for text, err in ((self.FULL.replace("&physics ", "&physics tempo_aerosolaware = 1, "), "tempo_aerosolaware"),
                          (self.FULL + "&afwa afwa_diag_opt = 1 /\n", "afwa_diag_opt"),
                          (self.FULL.replace("&domains ", "&domains perturb_input = .true., "), "perturb_input"),
                          (self.FULL.replace("&time_control ", "&time_control io_form_history = 102, "), "io_form_history = 102"),
                          (self.FULL.replace("&time_control ", "&time_control history_outname = 'out/x_d<domain>', "),
                           "history_outname")):
            self.assertTrue(any(err in e for e in self.errors(text)), (text, err))
        for text in (self.FULL.replace("&physics ", "&physics tempo_aerosolaware = 0, "),
                     self.FULL.replace("&dynamics ", "&dynamics do_coriolis = .false., do_curvature = .false., "),
                     self.FULL.replace("&physics ", "&physics nssl_2moment_on = 1, "),
                     self.FULL.replace("&dynamics ", "&dynamics damp_opt = 0, zdamp = 3000., ")):
            self.assertEqual(self.errors(text), [], text)

    def test_single_entry_options(self):
        # r34_t1_*: WRF's namelist read stops on more values than a single-entry option (Registry "1") holds
        for opt in ("isfflx = 1, 1", "cldovrlp = 2, 2", "bl_mynn_closure = 2.6, 2.6"):
            text = self.FULL.replace("&physics ", f"&physics {opt}, ")
            self.assertTrue(any("takes one value" in e for e in self.errors(text)), opt)
        for opt in ("isfflx = 1", "cudt = 5, 5"):  # cudt: max_domains entries
            self.assertEqual(self.errors(self.FULL.replace("&physics ", f"&physics {opt}, ")), [], opt)

    def test_namelist_outside_real_dir(self):
        # the model reads the output options and global attributes from real.dir/namelist.input unless
        # output.wrf_namelist names the translated namelist
        tmp = tempfile.mkdtemp()
        for d in ("a", "b"):
            os.makedirs(os.path.join(tmp, d))
            with open(os.path.join(tmp, d, "namelist.input"), "w") as f:
                f.write(self.FULL)
        nml = os.path.join(tmp, "a", "namelist.input")
        for real_dir, expect in ((None, None), (os.path.join(tmp, "a"), None), (os.path.join(tmp, "b"), nml)):
            t = WrfTranslator(parse_namelist(self.FULL), namelist_path=nml, real_dir=real_dir)
            sec = t.translate()
            self.assertEqual(t.errors, [])
            self.assertEqual(sec["output"].get("wrf_namelist"), expect, real_dir)

    def test_output_file_names(self):
        # history_outname / rst_outname: the model names its files as construct_filename2a does; a directory part
        # (CPPWRF writes into output.dir) or an empty name is an error
        def errors(extra):
            t = WrfTranslator(parse_namelist(self.FULL.replace("&time_control ", "&time_control history_interval = 1, " + extra)),
                              output_format="wrfout")
            t.translate()
            return t.errors
        for extra in ("history_outname = 'hist_d<domain>_<year><month><day>_<hour>', ",
                      "restart_interval = 1, rst_outname = 'rst_d<domain>_<date>.nc', ",
                      "use_netcdf_classic = .true., "):  # the model writes NetCDF classic files
            self.assertEqual(errors(extra), [], extra)
        for extra, err in (("history_outname = 'out/wrfout_d<domain>_<date>', ", "history_outname"),
                           ("history_outname = '', ", "history_outname"),
                           ("restart_interval = 1, rst_outname = '/x/wrfrst_d<domain>', ", "rst_outname")):
            self.assertTrue(any(err in e for e in errors(extra)), (extra, errors(extra)))

    def test_input_file_checks(self):
        # input_wrf.F stops when e_we / e_sn / e_vert / num_land_cat differ from the input (r32ewe, r32nlc): the
        # model compares the values the translator passes, a missing entry being the Registry default
        t, sec = self.translate(self.NEST + "&physics num_land_cat = 20 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual([sec["real"][k] for k in ("e_we", "e_sn", "e_vert", "num_land_cat")], [120, 90, 45, 20])
        self.assertEqual([sec["d02"][k] for k in ("e_we", "e_sn", "e_vert")], [121, 91, 45])
        t, sec = self.translate("&time_control /\n&domains time_step = 60 /\n&bdy_control specified = .true. /\n")
        self.assertEqual([sec["real"][k] for k in ("e_we", "e_sn", "e_vert", "num_land_cat")], [32, 32, 31, 21])

    def test_sst_update_on_nests(self):
        # set_timekeeping_alarms: each domain reads its wrflowinp_d0N every auxinput4_interval(id); a missing
        # entry is 0 and that domain's alarm never rings
        text = self.NEST.replace("input_from_file", "auxinput4_inname = 'lowinp_d<domain>', auxinput4_interval = 360,"
                                 " io_form_auxinput4 = 2, input_from_file") + \
            "&physics sst_update = 1, mp_physics = 2*6, sf_sfclay_physics = 2*1, sf_surface_physics = 2*2," \
            " bl_pbl_physics = 2*1 /\n"
        t, sec = self.translate(text)
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["auxinput4_interval"], 360)
        self.assertEqual(sec["d02"]["phys_auxinput4_interval"], 0)
        self.assertEqual(sec["d02"]["wrflowinp"], "lowinp_d02")

    FDDA_PHYS = "&physics mp_physics = 2*6, sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, bl_pbl_physics = 2*1 /\n"

    def test_grid_fdda(self):
        # grid_fdda = 1 (PSUFDDAGD): the &fdda options to [fdda], a nest's own entries to [d02] fdda_*, the auxinput10
        # file name from gfdda_inname (real.exe writes wrffdda_d0N; WRF's default name is gfdda_d<domain>)
        text = self.NEST + self.FDDA_PHYS + (
            "&fdda grid_fdda = 1, 1, gfdda_inname = 'wrffdda_d<domain>', gfdda_interval_m = 360, 360,"
            " gfdda_end_h = 6, 3, io_form_gfdda = 2, guv = 0.0003, 0.0006, gt = 0.0003, 0.0003, gq = 0.0003, 0.0003,"
            " fgdt = 3, 3, if_no_pbl_nudging_uv = 1, 1, if_zfac_t = 1, 1, k_zfac_t = 5, 5, if_ramping = 1,"
            " dtramp_min = -30. /\n")
        t, sec = self.translate(text)
        self.assertEqual(t.errors, [])
        f = sec["fdda"]
        self.assertEqual((f["grid_fdda"], f["gfdda_interval_m"], f["gfdda_end_h"]), (1, 360, 6))
        self.assertEqual((f["guv"], f["fgdt"], f["if_no_pbl_nudging_uv"], f["k_zfac_t"]), (0.0003, 3, 1, 5))
        self.assertEqual((f["if_ramping"], f["dtramp_min"]), (1, -30.0))
        self.assertEqual(sec["d02"]["fdda_guv"], 0.0006)
        self.assertEqual(sec["d02"]["fdda_gfdda_end_h"], 3)
        self.assertNotIn("fdda_gt", sec["d02"])
        self.assertNotIn("wrffdda", sec.get("real", {}))
        # the default name: the model reads gfdda_d01 / gfdda_d02 from real.dir
        t, sec = self.translate(text.replace("gfdda_inname = 'wrffdda_d<domain>',", ""))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["real"]["wrffdda"], "gfdda_d01")
        self.assertEqual(sec["d02"]["wrffdda"], "gfdda_d02")
        # grid_fdda = 1 on the nest only
        t, sec = self.translate(text.replace("grid_fdda = 1, 1", "grid_fdda = 0, 1"))
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["fdda"]["grid_fdda"], sec["d02"]["fdda_grid_fdda"]), (0, 1))
        # gfdda_interval_m that is not a multiple of the time step (70 s): the AUXINPUT10 alarm rings at the first step
        # past the interval, input_wrf finds no analysis of that time in wrffdda and WRF stops (share/input_wrf.F,
        # 'Could not find matching time in input file'): an error, not nudging towards the first analyses for ever
        t, sec = self.translate(text.replace("time_step = 60,", "time_step = 70,"))
        self.assertTrue(any("gfdda_interval_m" in e and "time step" in e for e in t.errors), t.errors)
        # without grid_fdda the &fdda options have no effect (check_a_mundo zeroes the gfdda end and interval)
        t, sec = self.translate(text.replace("grid_fdda = 1, 1", "grid_fdda = 0, 0"))
        self.assertEqual(t.errors, [])
        self.assertNotIn("fdda", sec)

    def test_grid_fdda_errors(self):
        base = ("&fdda grid_fdda = 1, 1, gfdda_inname = 'wrffdda_d<domain>', gfdda_interval_m = 360, 360,"
                " gfdda_end_h = 6, 6, guv = 0.0003, 0.0003 X /\n")
        for extra, expected in [
                (", grid_fdda = 2, 1", "grid_fdda = 2 (spectral nudging)"),
                (", grid_sfdda = 1, 1", "grid_sfdda"),
                (", gfdda_begin_h = 1, 1", "gfdda_begin_h"),
                (", gfdda_end_d = 1, 1", "gfdda_end_d is ignored"),
                (", gfdda_interval_h = 6, 6", "gfdda_interval_h is ignored"),
                (", gfdda_interval_m = 0, 360", "gfdda_interval_m must be > 0"),
                (", gt = -0.1, 0.0003", "Gt must be positive"),
                (", io_form_gfdda = 0", "io_form_gfdda = 2"),
                (", gfdda_end = 360, 360", "gfdda_end = 360"),
                (", gfdda_inname = 'analyses'", "without <domain>")]:
            t, _ = self.translate(self.NEST + self.FDDA_PHYS + base.replace(" X", extra))
            self.assertTrue(any(expected in e for e in t.errors), (extra, t.errors))
        # the physics suite and no adaptive time step
        t, _ = self.translate(self.NEST + base.replace(" X", ""))
        self.assertTrue(any("needs a real-data case with the physics suite" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.NEST.replace("time_step = 60,", "time_step = 60, use_adaptive_time_step = .true.,")
                              + self.FDDA_PHYS + base.replace(" X", ""))
        self.assertTrue(any("grid_fdda = 1 with use_adaptive_time_step" in e for e in t.errors), t.errors)

    def test_buckets_and_prec_acc(self):
        # diagnostic_output_calc: bucket_mm, bucket_J (one entry) and prec_acc_dt (every domain's entry; check_a_mundo's
        # prec_acc_opt = 1 when some domain has prec_acc_dt > 0); a nest's missing entry is the Registry default 0
        phys = ("&physics mp_physics = 2*6, sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, bl_pbl_physics = 2*1,"
                " bucket_mm = 1., bucket_J = 1.e7, prec_acc_dt = 10. /\n")
        t, sec = self.translate(self.NEST + phys.replace(" /\n", ", ra_lw_physics = 2*4, ra_sw_physics = 2*4 /\n"))
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["phys"]["bucket_mm"], sec["phys"]["bucket_j"]), (1.0, 1.e7))
        self.assertEqual((sec["phys"]["prec_acc_dt"], sec["phys"]["prec_acc_opt"]), (10.0, 1))
        self.assertEqual(sec["d02"]["phys_prec_acc_dt"], 0.0)
        t, sec = self.translate(self.NEST + phys.replace("prec_acc_dt = 10.", "prec_acc_dt = 0., 5.").replace(
            "bucket_J = 1.e7, ", ""))
        self.assertEqual(t.errors, [])
        self.assertNotIn("prec_acc_dt", sec["phys"])
        self.assertEqual((sec["phys"]["prec_acc_opt"], sec["d02"]["phys_prec_acc_dt"]), (1, 5.0))
        t, sec = self.translate(self.NEST + phys.replace(" bucket_mm = 1., bucket_J = 1.e7, prec_acc_dt = 10.",
                                                         " bucket_mm = -1., prec_acc_dt = 0."))
        self.assertEqual(t.errors, [])
        self.assertFalse({"bucket_mm", "bucket_j", "prec_acc_dt", "prec_acc_opt"} & set(sec["phys"]))
        # ra_init: bucket_J without RRTMG radiation stops wrf.exe
        t, sec = self.translate(self.NEST + phys)
        self.assertTrue(any("Need CAM or RRTMG" in e for e in t.errors))
        rad = phys.replace(" /\n", ", ra_lw_physics = 2*4, ra_sw_physics = 2*4 /\n")
        t, sec = self.translate(self.NEST + rad)
        self.assertEqual(t.errors, [])
        t, sec = self.translate(self.NEST + rad.replace("ra_sw_physics = 2*4", "ra_sw_physics = 4, 0"))
        self.assertTrue(any("Need CAM or RRTMG" in e for e in t.errors))
        # without the column physics (microphysics only) CPPWRF has no port: an error
        for opt in ("bucket_mm = 1.", "bucket_J = 1.e7", "prec_acc_dt = 10."):
            t, sec = self.translate(self.NEST + f"&physics mp_physics = 2*6, {opt} /\n")
            self.assertTrue(any("bucket_mm / bucket_J / prec_acc_dt" in e for e in t.errors), opt)

    def test_output_and_nwp_diagnostics(self):
        # output_diagnostics = 1 (clwrf_output_calc): every domain's auxhist3_interval (minutes; a domain without one
        # stops wrf.exe), the auxhist3 files with io_form_auxhist3 = 2 (wrfout format only); nwp_diagnostics stays an
        # error (check_a_mundo also sets do_radar_ref = 1)
        phys = "&physics mp_physics = 2*6, sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, bl_pbl_physics = 2*1 /\n"

        def run(tc, physics=phys, fmt="wrfout"):
            text = self.NEST.replace("&time_control ", f"&time_control history_interval = 2*10, {tc}, ") + physics
            t = WrfTranslator(parse_namelist(complete(text)), output_format=fmt)
            return t, t.translate()
        ok = "output_diagnostics = 1, auxhist3_interval = 10, 20, io_form_auxhist3 = 2"
        t, sec = run(ok)
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["phys"]["output_diagnostics"], sec["phys"]["auxhist3_interval"]), (1, 10))
        self.assertEqual(sec["d02"]["phys_auxhist3_interval"], 20)
        self.assertTrue(sec["output"]["auxhist3"])
        # io_form_auxhist3 = 0: the statistics (restart fields) without auxhist3 files
        t, sec = run("output_diagnostics = 1, auxhist3_interval = 2*10")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["output_diagnostics"], 1)
        self.assertNotIn("auxhist3", sec["output"])
        self.assertNotIn("phys_auxhist3_interval", sec["d02"])
        for tc, err in (("output_diagnostics = 1, auxhist3_interval = 10", "NO 'auxhist3_interval'"),  # d02: none
                        (ok + ", auxhist3_interval_s = 30", "auxhist3_interval_s"),
                        (ok + ", auxhist3_begin_m = 30", "auxhist3_begin_m"),
                        (ok.replace("io_form_auxhist3 = 2", "io_form_auxhist3 = 11"), "io_form_auxhist3 = 11"),
                        (ok.replace("output_diagnostics = 1", "output_diagnostics = 2"), "output_diagnostics = 2")):
            t, _ = run(tc)
            self.assertTrue(any(err in e for e in t.errors), (tc, t.errors))
        # auxhist3 files need WRF-format output; the statistics need the column physics
        t, _ = run(ok, fmt="native")
        self.assertTrue(any("io_form_auxhist3 = 2" in e for e in t.errors), t.errors)
        t, _ = run(ok, physics="&physics mp_physics = 2*6 /\n")
        self.assertTrue(any("output_diagnostics = 1 needs the column physics" in e for e in t.errors), t.errors)
        t, _ = run("nwp_diagnostics = 1")
        self.assertTrue(any("nwp_diagnostics = 1" in e and "do_radar_ref" in e for e in t.errors), t.errors)

    def test_mynn_options_of_every_domain(self):
        text = self.NEST + ("&physics mp_physics = 2*8, sf_sfclay_physics = 2*5, sf_surface_physics = 2*2, bl_pbl_physics = 2*5,"
                            " bl_mynn_edmf_dd = 0, 1 /\n")
        t, _ = self.translate(text)
        self.assertTrue(any("bl_mynn_edmf_dd = 1" in e for e in t.errors), t.errors)
        # a ported option must be the same on all domains
        text = self.NEST + ("&physics mp_physics = 2*8, sf_sfclay_physics = 2*5, sf_surface_physics = 2*2, bl_pbl_physics = 2*5,"
                            " bl_mynn_edmf = 1, 0 /\n")
        t, _ = self.translate(text)
        self.assertTrue(any("bl_mynn_edmf with different values per domain" in e for e in t.errors), t.errors)
        text = self.NEST + ("&physics mp_physics = 2*8, sf_sfclay_physics = 2*5, sf_surface_physics = 2*2, bl_pbl_physics = 2*5,"
                            " bl_mynn_edmf = 2*0, bl_mynn_mixlength = 2 /\n")
        t, sec = self.translate(text)
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["phys"]["bl_mynn_edmf"], sec["phys"]["bl_mynn_mixlength"]), (0, 2))

    def test_mynn_ported_options(self):
        """module_bl_mynnedmf.F90: the ported option values reach [phys]; unported ones are errors."""
        base = "&physics mp_physics = 8, sf_sfclay_physics = 5, sf_surface_physics = 2, bl_pbl_physics = 5, "
        for opt, val, key, want in (("bl_mynn_cloudpdf", "0", "bl_mynn_cloudpdf", 0),
                                    ("bl_mynn_cloudpdf", "-1", "bl_mynn_cloudpdf", -1),
                                    ("bl_mynn_mixlength", "0", "bl_mynn_mixlength", 0),
                                    ("bl_mynn_edmf_mom", "0", "bl_mynn_edmf_mom", 0),
                                    ("bl_mynn_edmf_tke", "1", "bl_mynn_edmf_tke", 1),
                                    ("bl_mynn_cloudmix", "0", "bl_mynn_cloudmix", 0),
                                    ("bl_mynn_mixqt", "1", "bl_mynn_mixqt", 1),
                                    ("bl_mynn_ess", "0", "bl_mynn_ess", 0),
                                    ("bl_mynn_mixscalars", "1", "bl_mynn_mixscalars", 1),
                                    ("bl_mynn_output", "1", "bl_mynn_output", 1),
                                    ("tke_budget", "1", "tke_budget", 1),
                                    ("bl_mynn_tkeadvect", ".true.", "bl_mynn_tkeadvect", True),
                                    ("bl_mynn_closure", "2.5", "bl_mynn_closure", 2.5),
                                    ("bl_mynn_closure", "3.0", "bl_mynn_closure", 3.0),
                                    ("sf_mynn_sfcflux_land", "2", "sf_mynn_sfcflux_land", 2),
                                    ("sf_mynn_sfcflux_water", "0", "sf_mynn_sfcflux_water", 0),
                                    ("isftcflx", "1", "isftcflx", 1), ("iz0tlnd", "2", "iz0tlnd", 2)):
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{base}{opt} = {val} /\n")))
            cfg = t.translate()
            self.assertEqual(t.errors, [], (opt, val))
            self.assertEqual(cfg["phys"][key], want, (opt, val))
        for opt, val, err in (("bl_mynn_edmf", "2", "bl_mynn_edmf = 2"), ("bl_mynn_edmf_dd", "1", "bl_mynn_edmf_dd = 1"),
                              ("bl_mynn_mixlength", "3", "bl_mynn_mixlength = 3"),
                              ("bl_mynn_closure", "2.7", "bl_mynn_closure = 2.7"),
                              ("bl_mynn_mixnumcon", "1", "bl_mynn_mixnumcon = 1"),
                              ("sf_mynn_sfcflux_water", "5", "sf_mynn_sfcflux_water = 5"),
                              ("sf_mynn_sfcflux_land", "4", "sf_mynn_sfcflux_land = 4")):
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{base}{opt} = {val} /\n")))
            t.translate()
            self.assertIn(err, " ".join(t.errors), (opt, val))

    def test_grav_settling(self):
        """module_sf_fogdes.F / module_bl_fogdes.F: grav_settling 1, 2 reach [phys]; 3, per-domain values and
        a run without a PBL scheme are errors."""
        base = "&physics mp_physics = 8, sf_sfclay_physics = 1, sf_surface_physics = 2, "
        for pbl, val, want in (("1", "1", 1), ("5", "2", 2)):
            t = WrfTranslator(parse_namelist(real_case(
                f"&time_control /\n&domains time_step = 60 /\n{base}bl_pbl_physics = {pbl}, grav_settling = {val} /\n")))
            cfg = t.translate()
            self.assertEqual(t.errors, [], val)
            self.assertEqual(cfg["phys"]["grav_settling"], want)
        t = WrfTranslator(parse_namelist(real_case(
            f"&time_control /\n&domains time_step = 60 /\n{base}bl_pbl_physics = 1 /\n")))
        self.assertNotIn("grav_settling", t.translate()["phys"])
        for nl, err in ((f"{base}bl_pbl_physics = 1, grav_settling = 3 /", "grav_settling = [3]"),
                        (f"{base}bl_pbl_physics = 0, grav_settling = 1 /", "needs a microphysics scheme")):
            t = WrfTranslator(parse_namelist(real_case(f"&time_control /\n&domains time_step = 60 /\n{nl}\n")))
            t.translate()
            self.assertIn(err, " ".join(t.errors), nl)


    def test_adaptive_time_step(self):
        """use_adaptive_time_step (adapt_timestep_em.F): d01's options and the history / boundary intervals reach
        [time]; nests and sst_update are errors."""
        t = WrfTranslator(parse_namelist(real_case(
            "&time_control history_interval_s = 600 /\n&domains time_step = 60, use_adaptive_time_step = .true., "
            "target_cfl = 1.1, max_time_step = 90, step_to_output_time = .false. /\n&physics /\n")))
        cfg = t.translate()
        self.assertEqual(t.errors, [])
        tm = cfg["time"]
        self.assertTrue(tm["adaptive"])
        self.assertEqual((tm["target_cfl"], tm["max_time_step"], tm["step_to_output_time"]), (1.1, 90, False))
        # interval_seconds: the Registry default 10800 (real_case checks it against the wrfbdy interval)
        self.assertEqual((tm["history_interval_s"], tm["interval_seconds"]), (600, 10800))
        self.assertNotIn("target_hcfl", tm)
        # time.dt: adapt_timestep's first time step, starting_time_step (/ its _den) between max_time_step and
        # min_time_step (-1: NINT(4, 8, 3 * MIN(dx, dy) / 1000), dx, dy Registry 200 m), whatever time_step is
        for dom, want in (("time_step = 0", 1),  # without dx, dy: NINT(0.8) = 1 s
                          ("time_step = 0, dx = 100, dy = 100, min_time_step = 1", 1),  # NINT(0.4) = 0 -> 1 s
                          ("time_step = 0, starting_time_step = 1, starting_time_step_den = 2, min_time_step = 0", 0.5),
                          ("time_step = 60, starting_time_step = 50", 50),
                          ("time_step = 60", 60)):
            nl = real_case(f"&time_control history_interval_s = 600 /\n&domains {dom}, use_adaptive_time_step = .true."
                           " /\n&physics /\n")
            if dom == "time_step = 0":
                nl = re.sub(r"\bd[xy] = 15000,", "", nl)
            t = WrfTranslator(parse_namelist(nl))
            cfg = t.translate()
            self.assertEqual((t.errors, cfg["time"]["dt"]), ([], want), dom)
        # adapt_timestep rounds the starting time step to 1/100 s before the limits: 1/3 s is 0.33 s; 1/250 s is 0
        for dom, want in (("starting_time_step = 1, starting_time_step_den = 3, min_time_step = 0", 0.33),
                          ("starting_time_step = 1, starting_time_step_den = 200, min_time_step = 0", 0.01)):
            t = WrfTranslator(parse_namelist(real_case(
                f"&time_control history_interval_s = 600 /\n&domains time_step = 0, {dom}, use_adaptive_time_step = "
                ".true. /\n&physics /\n")))
            cfg = t.translate()
            self.assertEqual((t.errors, cfg["time"]["dt"]), ([], want), dom)
        for nl, err in (("&domains time_step = 60, use_adaptive_time_step = .true. /\n&physics sst_update = 1 /",
                         "sst_update"),
                        ("&domains time_step = 0, starting_time_step = 1, starting_time_step_den = 250, min_time_step "
                         "= 0, use_adaptive_time_step = .true. /\n&physics /", "not positive"),
                        ("&domains time_step = 0, max_time_step = 60, max_time_step_den = -2, min_time_step = 45, "
                         "use_adaptive_time_step = .true. /\n&physics /", "max_time_step_den = -2"),
                        ("&domains time_step = 0 /\n&physics /", "time_step = 0 without use_adaptive_time_step")):
            t = WrfTranslator(parse_namelist(real_case(f"&time_control history_interval_s = 600 /\n{nl}\n")))
            t.translate()
            self.assertIn(err, " ".join(t.errors), nl)

    def test_scalar_adv_opt_without_physics(self):
        # module_em.F rk_tendency: scalar_adv_opt 3 / 4 advect theta with WENO on the last RK stage, also in dry runs
        ideal = ("&time_control run_minutes = 10 /\n&domains time_step = 3, e_we = 202, e_sn = 3, e_vert = 81, dx = 250,"
                 " dy = 250, ztop = 20000 /\n&dynamics scalar_adv_opt = {v} /\n&bdy_control periodic_x = .true.,"
                 " periodic_y = .true. /\n&ideal ideal_case = 1 /\n")
        for v in (0, 2, 3, 4):
            t, sec = self.translate(ideal.format(v=v))
            self.assertEqual(t.errors, [])
            self.assertEqual(sec["dyn"]["scalar_adv_opt"], v)
        t, sec = self.translate(ideal.format(v=1))
        self.assertNotIn("scalar_adv_opt", sec["dyn"])  # the model's default 1
        t, sec = self.translate(self.FULL.replace("mp_physics = 0", "mp_physics = 8")
                                .replace("&dynamics ", "&dynamics scalar_adv_opt = 2, "))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["dyn"]["scalar_adv_opt"], 2)

    def test_nest_diff_6th_factor(self):
        # each domain diffuses with its own diff_6th_factor when its diff_6th_opt /= 0 (domain 1 has none here)
        t, sec = self.translate(self.NEST + "&dynamics diff_6th_opt = 0, 2, diff_6th_factor = 0.25, 0.25 /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("diff_6th_factor", sec["dyn"])
        self.assertEqual((sec["d02"]["diff_6th_opt"], sec["d02"]["diff_6th_factor"]), (2, 0.25))
        t, sec = self.translate(self.NEST + "&dynamics diff_6th_opt = 2, 2, diff_6th_factor = 0.25, 0.25 /\n")
        self.assertEqual(sec["dyn"]["diff_6th_factor"], 0.25)
        self.assertNotIn("diff_6th_factor", sec["d02"])
        t, sec = self.translate(self.NEST + "&dynamics diff_6th_opt = 0, 2 /\n")
        self.assertNotIn("diff_6th_factor", sec["d02"])  # the Registry default 0.12, the model's

    def test_nest_scalar_advection_orders(self):
        # module_advect_em.F: each domain's h_sca_adv_order / v_sca_adv_order (Registry 5 / 3) for theta and scalars
        for dyn, err in (("h_mom_adv_order = 5, 6", "h_sca_adv_order = 5 differs from h_mom_adv_order = 6"),
                         ("h_mom_adv_order = 5, 6, h_sca_adv_order = 5, 5", "h_sca_adv_order = 5 differs"),
                         ("v_mom_adv_order = 3, 4", "v_sca_adv_order = 3 differs from v_mom_adv_order = 4")):
            t, _ = self.translate(self.NEST + f"&dynamics {dyn} /\n")
            self.assertTrue(any(err in e and "d02" in e for e in t.errors), (dyn, t.errors))
        t, _ = self.translate(self.NEST + "&dynamics h_mom_adv_order = 5, 5, v_mom_adv_order = 3, 3 /\n")
        self.assertEqual(t.errors, [])

    def test_nest_start_and_end_dates(self):
        # set_timekeeping: run_* sets domain 1's stop time only, a nest stops at its end_*; a missing entry is the
        # Registry default (start 1993-03-13_12, end 1993-03-14_12), not domain 1's value
        dates = ("run_hours = 6, start_year = 2024, 2024, start_month = 7, 7, start_day = 1, 1, start_hour = {sh} "
                 "end_year = 2024, 2024, end_month = 7, 7, end_day = 1, 1, end_hour = {eh} ")

        def nest(sh, eh):
            return self.NEST.replace("&time_control ", "&time_control " + dates.format(sh=sh, eh=eh))
        t, _ = self.translate(nest("0, 0,", "6, 6,"))
        self.assertEqual(t.errors, [])
        t, _ = self.translate(nest("0, 0,", "12, 12,"))  # the nest's end after domain 1's stop: WRF's run
        self.assertEqual(t.errors, [])
        t, _ = self.translate(nest("0, 0,", "3, 3,"))
        self.assertTrue(any("end date of domain 2" in e for e in t.errors), t.errors)
        t, _ = self.translate(nest("0,", "6, 6,"))
        self.assertTrue(any("start_hour = [0, 12]" in e for e in t.errors), t.errors)
        t, _ = self.translate(nest("0, 0,", "6,"))
        self.assertTrue(any("end_hour = [6, 12]" in e for e in t.errors), t.errors)

    def test_run_length_from_registry_dates(self):
        # without run_*: end_*(1) - start_*(1), a component the namelist leaves out the Registry default
        t, sec = self.translate(self.FULL)
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["run"]["run_time"], 86400)  # 1993-03-13_12 to 1993-03-14_12
        t, sec = self.translate(self.FULL.replace("&time_control ", "&time_control start_hour = 0, end_day = 13, "))
        self.assertEqual(sec["run"]["run_time"], 12 * 3600)
        t, sec = self.translate(self.FULL.replace("&time_control ", "&time_control end_day = 13, "))
        self.assertTrue(any("run length 0 s" in e for e in t.errors), t.errors)
        t, sec = self.translate(self.FULL.replace("&time_control ", "&time_control end_day = 13, end_hour = 0, "
                                                  "end_minute = 900, "))  # WRFU_TimeSet carries the minutes over
        self.assertEqual(sec["run"]["run_time"], 3 * 3600)

    def test_sst_update_without_column_physics(self):
        # WRF reads wrflowinp (auxinput4) in every run with sst_update = 1; CPPWRF with the column physics only
        text = self.FULL.replace("mp_physics = 0", "mp_physics = 6").replace(
            "&time_control ", "&time_control auxinput4_interval = 360, io_form_auxinput4 = 2, ").replace(
            "&physics ", "&physics sst_update = 1, ")
        t, _ = self.translate(text)
        self.assertTrue(any("sst_update = 1 without" in e for e in t.errors), t.errors)

    def test_ghg_input_non_binary(self):
        # check_a_mundo:974-1004 & radiation_driver: CAM GHG tables only for ghg_input == 1; other values use 0
        rad = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = 1,"
               " sf_surface_physics = 2, bl_pbl_physics = 1, ra_lw_physics = 4, ra_sw_physics = 4, ghg_input = 2 /\n")
        t, sec = self.translate(real_case(rad))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["ghg_input"], 0)
        self.assertTrue(any("ghg_input = 2" in n for n in t.notices), t.notices)
        t, sec = self.translate(self.FULL.replace("&physics ", "&physics ghg_input = 2, "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("phys", sec)
        self.assertTrue(any("ghg_input = 2" in n for n in t.notices), t.notices)

    def test_tice2tsk_if2cold_with_fractional_seaice_0(self):
        # check_a_mundo:1010-1016: warning only when fractional_seaice = 0 and tice2tsk_if2cold = .true.
        noah = self.FULL.replace("sf_sfclay_physics = 0, sf_surface_physics = 0",
                                 "sf_sfclay_physics = 1, sf_surface_physics = 2")
        for base in (noah, self.FULL):
            t, sec = self.translate(base.replace("&physics ", "&physics fractional_seaice = 0, tice2tsk_if2cold = .true., "))
            self.assertEqual(t.errors, [])
            self.assertNotIn("tice2tsk_if2cold", sec.get("phys", {}))
            self.assertTrue(any("You set tice2tsk_if2cold = .true.,  but fractional_seaice = 0" in n
                                for n in t.notices), t.notices)
        t, sec = self.translate(noah.replace("&physics ", "&physics fractional_seaice = 1, tice2tsk_if2cold = .true., "))
        self.assertEqual(t.errors, [])
        self.assertTrue(sec["phys"]["tice2tsk_if2cold"])

    def test_sf_urban_physics_innermost_domain_copy(self):
        # check_a_mundo:1063-1077: every domain adopts sf_urban_physics(max_dom)
        sfc = "sf_sfclay_physics = 2*1, sf_surface_physics = 2*2, bl_pbl_physics = 2*1, "
        for vals in ("1, 0", "1"):
            t, sec = self.translate(self.NEST + f"&physics {sfc}sf_urban_physics = {vals} /\n")
            self.assertEqual(t.errors, [], vals)
            self.assertNotIn("sf_urban_physics", sec["phys"])
            self.assertTrue(any("sf_urban_physics option must be identical in each domain" in n
                                for n in t.notices), (vals, t.notices))
        for vals in ("0, 1", "1, 1"):
            t, _ = self.translate(self.NEST + f"&physics {sfc}sf_urban_physics = {vals} /\n")
            self.assertTrue(any("sf_urban_physics = 1 with nests" in e for e in t.errors), (vals, t.errors))

    def test_mskf_icloud_and_cu_diag_order(self):
        # check_a_mundo:1390-1398 runs before L2198-2239: MSKF sets icloud = 1 and cu_rad_feedback = .true.
        base = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = 1,"
                " sf_surface_physics = 2, bl_pbl_physics = 1, ra_lw_physics = 4, ra_sw_physics = 4, {opts} /\n")
        for ic in (2, 3):
            t, sec = self.translate(real_case(base.format(opts=f"cu_physics = 11, icloud = {ic}")))
            self.assertEqual(t.errors, [], ic)
            self.assertNotIn("icloud", sec["phys"])
            self.assertTrue(any("cu_physics is 11, setting icloud = 1 and cu_rad_feedback = T" in n
                                for n in t.notices), t.notices)
        t, _ = self.translate(real_case(base.format(opts="cu_physics = 1, icloud = 3")))
        self.assertTrue(any("icloud = 3 is not supported" in e for e in t.errors), t.errors)
        t, _ = self.translate(real_case(base.format(opts="cu_physics = 11, cu_rad_feedback = .false., cu_diag = 1")))
        self.assertEqual(t.errors, [])
        t, _ = self.translate(real_case(base.format(opts="cu_physics = 1, cu_rad_feedback = .false., cu_diag = 1")))
        self.assertTrue(any("Using cu_diag=1 requires use of one of the following CU schemes" in e
                            for e in t.errors), t.errors)

    def test_sst_update_0_zeroes_auxinput4(self):
        # check_a_mundo:1446-1459: sst_update = 0 zeroes io_form_auxinput4 and auxinput4_interval{,_y,_d,_h,_m,_s}
        noah = self.FULL.replace("sf_sfclay_physics = 0, sf_surface_physics = 0",
                                 "sf_sfclay_physics = 1, sf_surface_physics = 2")
        tc = "&time_control io_form_auxinput4 = 2, auxinput4_interval_h = 6, auxinput4_interval_m = 30, auxinput4_interval_s = 10 /"
        for base in (noah, self.FULL):
            t, sec = self.translate(base.replace("&time_control /", tc).replace("&physics ", "&physics sst_update = 0, "))
            self.assertEqual(t.errors, [])
            self.assertNotIn("sst_update", sec.get("phys", {}))
            self.assertTrue(any("sst_update is 0, setting io_form_auxinput4 = 0 and auxinput4_interval = 0" in n
                                for n in t.notices), t.notices)

    def test_mynn_edmf_reset_with_non_mynn_pbl(self):
        # check_a_mundo:1775-1782: non-MYNN PBL resets bl_mynn_edmf, bl_mynn_edmf_dd and bl_mynn_output to 0
        opts = "bl_mynn_edmf = 0, bl_mynn_edmf_dd = 1, bl_mynn_output = 1"
        for pbl, sfc in ((1, 1), (0, 5), (0, 0)):
            lsm = 2 if (pbl or sfc) else 0
            text = (f"&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6,"
                    f" sf_sfclay_physics = {sfc}, sf_surface_physics = {lsm}, bl_pbl_physics = {pbl}, {opts} /\n")
            t, sec = self.translate(real_case(text))
            self.assertEqual(t.errors, [], (pbl, sfc))
            for k in ("bl_mynn_edmf", "bl_mynn_edmf_dd", "bl_mynn_output"):
                self.assertNotIn(k, sec.get("phys", {}))
            self.assertTrue(any("bl_mynn_output" in n and "reset to 0" in n for n in t.notices), t.notices)

    def test_mynn_edmf_0_allows_ishallow(self):
        # check_a_mundo:1788-1800: bl_mynn_edmf > 0 forbids ishallow/shcu_physics; bl_mynn_edmf = 0 allows ishallow = 1
        base = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = 5,"
                " sf_surface_physics = 2, bl_pbl_physics = 5, cu_physics = 3, {opts} /\n")
        t, sec = self.translate(real_case(base.format(opts="bl_mynn_edmf = 0, ishallow = 1")))
        self.assertEqual(t.errors, [])
        self.assertEqual((sec["phys"]["bl_mynn_edmf"], sec["phys"]["ishallow"]), (0, 1))
        t, _ = self.translate(real_case(base.format(opts="bl_mynn_edmf = 1, ishallow = 1")))
        self.assertTrue(any("bl_mynn_edmf > 0 requires both shcu_physics=0 & ishallow=0" in e
                            for e in t.errors), t.errors)
        t, _ = self.translate(real_case(base.format(opts="bl_mynn_edmf = 0, shcu_physics = 2")))
        self.assertTrue(any("shcu_physics = 2" in e for e in t.errors), t.errors)

    def test_icloud_bl_non_mynn_and_positive(self):
        # check_a_mundo:1806-1821 & radiation_driver:1404: non-MYNN resets icloud_bl = 1 to 0 and accepts 0;
        # MYNN tests icloud_bl > 0
        noah = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = {sfc},"
                " sf_surface_physics = 2, bl_pbl_physics = {pbl}, icloud_bl = {val} /\n")
        t, sec = self.translate(real_case(noah.format(sfc=1, pbl=1, val=0)))
        self.assertEqual(t.errors, [])
        self.assertNotIn("icloud_bl", sec["phys"])
        t, sec = self.translate(real_case(noah.format(sfc=1, pbl=1, val=1)))
        self.assertEqual(t.errors, [])
        self.assertNotIn("icloud_bl", sec["phys"])
        self.assertTrue(any("Need MYNN PBL for icloud_bl = 1, resetting to 0" in n for n in t.notices), t.notices)
        t, sec = self.translate(self.FULL.replace("&physics ", "&physics icloud_bl = 0, "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("phys", sec)
        t, sec = self.translate(real_case(noah.format(sfc=5, pbl=5, val=2)))
        self.assertEqual(t.errors, [])
        self.assertNotIn("icloud_bl", sec["phys"])
        self.assertTrue(any("icloud_bl = 2" in n for n in t.notices), t.notices)
        t, sec = self.translate(real_case(noah.format(sfc=5, pbl=5, val=0)))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["icloud_bl"], 0)

    def test_lagday_ignored_when_tmn_update_0(self):
        # check_a_mundo:2325-2330: lagday only checked/used when tmn_update = 1
        noah = self.FULL.replace("sf_sfclay_physics = 0, sf_surface_physics = 0",
                                 "sf_sfclay_physics = 1, sf_surface_physics = 2")
        for base in (noah, self.FULL):
            for ld in (1, 30):
                t, sec = self.translate(base.replace("&physics ", f"&physics tmn_update = 0, lagday = {ld}, "))
                self.assertEqual(t.errors, [], ld)
                self.assertNotIn("lagday", sec.get("phys", {}))
                self.assertTrue(any(f"lagday = {ld}" in n and "tmn_update = 0" in n for n in t.notices), t.notices)

    def test_grav_settling_reset_for_mp28_before_validation(self):
        # check_a_mundo:2477-2490: mp_physics = 28 resets any non-zero grav_settling to 0
        text = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 28, sf_sfclay_physics = 1,"
                " sf_surface_physics = 2, bl_pbl_physics = 1, grav_settling = 3 /\n")
        t, sec = self.translate(real_case(text))
        self.assertEqual(t.errors, [])
        self.assertNotIn("grav_settling", sec["phys"])
        self.assertTrue(any("mp_physics == 28, already has gravitational fog settling; resetting grav_settling to 0"
                            in n for n in t.notices), t.notices)

    def test_scalar_pblmix_wsm3_wsm5_and_mynn(self):
        # module_pbl_driver: WSM3 (3) and WSM5 (4) have no 4-D scalars; MYNN turns scalar_pblmix off (check_a_mundo)
        base = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = {mp}, sf_sfclay_physics = {sfc},"
                " sf_surface_physics = 2, bl_pbl_physics = {pbl}, scalar_pblmix = 1 /\n")
        for mp in (3, 4):
            t, _ = self.translate(real_case(base.format(mp=mp, sfc=1, pbl=1)))
            self.assertEqual(t.errors, [], mp)
            self.assertTrue(any("scalar_pblmix" in n for n in t.notices), t.notices)
        t, _ = self.translate(real_case(base.format(mp=8, sfc=1, pbl=1)))
        self.assertTrue(any("scalar_pblmix = 1" in e for e in t.errors), t.errors)
        t, _ = self.translate(real_case(base.format(mp=8, sfc=5, pbl=5)))
        self.assertEqual(t.errors, [])
        self.assertTrue(any("MYNN is set to mix scalars, turning off scalar_pblmix" in n for n in t.notices), t.notices)
        t, _ = self.translate(self.FULL.replace("mp_physics = 0", "mp_physics = 8, scalar_pblmix = 1"))
        self.assertEqual(t.errors, [])

    def test_vert_refine_fact_zero_rejected(self):
        # Registry default vert_refine_fact = 1; any value != 1 is ndown-specific and unported
        t, _ = self.translate(self.FULL.replace("&domains ", "&domains vert_refine_fact = 0, "))
        self.assertTrue(any("vert_refine_fact = [0]" in e for e in t.errors), t.errors)
        t, _ = self.translate(self.FULL.replace("&domains ", "&domains vert_refine_fact = 1, "))
        self.assertEqual(t.errors, [])

    def test_o3input_reset_for_non_rrtmg(self):
        # check_a_mundo:2948-2962: non-RRTMG radiation resets o3input = 0
        base = ("&time_control /\n&domains time_step = 60 /\n&physics sf_sfclay_physics = 1, sf_surface_physics = 2,"
                " bl_pbl_physics = 1, ra_lw_physics = {lw}, ra_sw_physics = {sw}, o3input = 1 /\n")
        t, sec = self.translate(real_case(base.format(lw=1, sw=1)))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["o3input"], 0)
        self.assertTrue(any("RRTMG radiation is not used, setting:  o3input=0 to avoid data pre-processing" in n
                            for n in t.notices), t.notices)
        t, sec = self.translate(self.FULL.replace("&physics ", "&physics o3input = 1, "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("phys", sec)
        self.assertTrue(any("o3input=0" in n for n in t.notices), t.notices)
        t, _ = self.translate(real_case(base.format(lw=4, sw=4)))
        self.assertTrue(any("o3input = 1 is not supported" in e for e in t.errors), t.errors)

    def test_nssl_negative_flags_accepted(self):
        # check_a_mundo:3485-3509: any < 0 value of the four NSSL flags selects mp_physics = 18's defaults
        text = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 18, sf_sfclay_physics = 1,"
                " sf_surface_physics = 2, bl_pbl_physics = 1, nssl_2moment_on = -2, nssl_hail_on = -3,"
                " nssl_ccn_on = -4, nssl_density_on = -5 /\n")
        t, sec = self.translate(real_case(text))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["phys"]["mp_physics"], 18)
        for k in ("nssl_2moment_on", "nssl_hail_on", "nssl_ccn_on", "nssl_density_on"):
            self.assertTrue(any(k in n and "< 0" in n for n in t.notices), (k, t.notices))

    def test_zdamp_vs_ztop_damp_opt(self):
        # Only damp_opt = 2 (rk_rayleigh_damp) crashes when zdamp >= ztop; damp_opt = 1 and 3 are bounded
        ideal = ("&time_control run_minutes = 10 /\n&domains time_step = 3, e_we = 202, e_sn = 3, e_vert = 81,"
                 " dx = 250, dy = 250{ztop} /\n&dynamics damp_opt = {opt}, zdamp = 20000. /\n"
                 "&bdy_control periodic_x = .true., periodic_y = .true. /\n&ideal ideal_case = 1 /\n")
        t, _ = self.translate(ideal.format(opt=2, ztop=""))  # omitted ztop defaults to 15000 m
        self.assertTrue(any("damp_opt = 2" in e and "zdamp = 20000" in e for e in t.errors), t.errors)
        for opt in (1, 3):
            t, sec = self.translate(ideal.format(opt=opt, ztop=", ztop = 15000"))
            self.assertEqual(t.errors, [], opt)
            self.assertEqual((sec["dyn"]["damp_opt"], sec["dyn"]["zdamp"]), (opt, 20000))

    def test_mynn_and_acm2_mp_whitelists(self):
        # MYNN accepts every ported MP carrying qc, qi and qs (4, 6, 8, 10, 14, 16, 17, 18);
        # ACM2 accepts every ported MP carrying qc and qi (4, 6, 8, 10, 14, 16, 17, 18, 28, 50, 51)
        def tr(pbl, sfc, mp):
            text = (f"&time_control /\n&domains time_step = 60 /\n&physics mp_physics = {mp},"
                    f" sf_sfclay_physics = {sfc}, sf_surface_physics = 2, bl_pbl_physics = {pbl} /\n")
            return self.translate(real_case(text))
        for mp in (4, 6, 8, 10, 14, 16, 17, 18):
            t, _ = tr(5, 5, mp)
            self.assertEqual(t.errors, [], ("MYNN", mp, t.errors))
        for mp in (0, 1, 3, 28, 50, 51):
            t, _ = tr(5, 5, mp)
            self.assertTrue(len(t.errors) > 0, ("MYNN", mp))
        for mp in (4, 6, 8, 10, 14, 16, 17, 18, 28, 50, 51):
            t, _ = tr(7, 1, mp)
            self.assertEqual(t.errors, [], ("ACM2", mp, t.errors))
        for mp in (0, 1, 3):
            t, _ = tr(7, 1, mp)
            self.assertTrue(any("needs a microphysics with qc and qi" in e for e in t.errors), ("ACM2", mp, t.errors))

    def test_idcor_and_hail_opt_non_binary(self):
        # WRF treats idcor /= 1 as 0 (Zo_default = 2500 m) and hail_opt /= 1 as 0 (graupel)
        rad = ("&time_control /\n&domains time_step = 60 /\n&physics mp_physics = {mp}, sf_sfclay_physics = 1,"
               " sf_surface_physics = 2, bl_pbl_physics = 1, ra_lw_physics = 4, ra_sw_physics = 4,"
               " idcor = 2, hail_opt = 2 /\n")
        for mp in (6, 16):
            t, sec = self.translate(real_case(rad.format(mp=mp)))
            self.assertEqual(t.errors, [], mp)
            self.assertNotIn("idcor", sec["phys"])
            self.assertNotIn("hail_opt", sec["phys"])
            self.assertTrue(any("idcor = 2" in n and "2500 m" in n for n in t.notices), t.notices)
            self.assertTrue(any("hail_opt = 2" in n and "graupel" in n for n in t.notices), t.notices)

    def test_ideal_hypsometric_opt_reset_to_1(self):
        # share/input_wrf.F:1032-1040: ideal cases always use hypsometric_opt = 1
        ideal = ("&time_control run_minutes = 10 /\n&domains time_step = 3, e_we = 202, e_sn = 3, e_vert = 81,"
                 " dx = 250, dy = 250, ztop = 20000, hypsometric_opt = 2 /\n"
                 "&bdy_control periodic_x = .true., periodic_y = .true. /\n&ideal ideal_case = 1 /\n")
        t, sec = self.translate(ideal)
        self.assertEqual(t.errors, [])
        self.assertNotIn("hypsometric_opt", sec["dyn"])
        self.assertTrue(any("Ideal cases always use hypsometric_opt=1" in n for n in t.notices), t.notices)
        t = WrfTranslator(parse_namelist(complete(ideal)), override_case="straka")
        sec = t.translate()
        self.assertEqual(t.errors, [])
        self.assertNotIn("hypsometric_opt", sec["dyn"])
        self.assertTrue(any("Ideal cases always use hypsometric_opt=1" in n for n in t.notices), t.notices)
        t, sec = self.translate(self.FULL.replace("&domains ", "&domains hypsometric_opt = 1, "))
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["dyn"]["hypsometric_opt"], 1)

    def test_non_max_domains_array_options_checked_beyond_index_0(self):
        # Non-max_domains array options (max_plevs, max_zlevs, max_ocean) must check every supplied element
        for extra, key in (("&diags press_levels = 0, 85000, 50000 /\n", "press_levels"),
                           ("&diags z_levels = 0, 500, 1000 /\n", "z_levels"),
                           ("", "ocean_z")):
            text = (self.FULL.replace("&domains ", "&domains ocean_z = -1, 10., 20., ") if key == "ocean_z"
                    else self.FULL + extra)
            t, _ = self.translate(text)
            self.assertTrue(any(key in e and "is not supported" in e for e in t.errors), (key, t.errors))
        t, _ = self.translate(self.FULL + "&diags z_levels = 0, 0, 0 /\n")
        self.assertEqual(t.errors, [])

    def test_chem_adv_opt_ignored(self):
        # Registry default chem_adv_opt = 1; stock non-Chem wrf.exe ignores chem_adv_opt
        for val in (0, 1):
            t, _ = self.translate(self.FULL.replace("&dynamics ", f"&dynamics chem_adv_opt = {val}, "))
            self.assertEqual(t.errors, [], val)
            self.assertEqual(t.warnings, [], val)
        t, _ = self.translate(self.NEST + "&dynamics chem_adv_opt = 0 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(t.warnings, [])

    def test_moving_nest_registry_defaults_accepted(self):
        # Moving-nest options at WRF Registry defaults must not be rejected
        defaults = ("vortex_interval = 15, max_vortex_speed = 40, corral_dist = 8, track_level = 50000,"
                    " time_to_move = 0., move_id = 0, move_interval = 999999999, move_cd_x = 0, move_cd_y = 0, ")
        t, _ = self.translate(self.FULL.replace("&domains ", "&domains " + defaults))
        self.assertEqual(t.errors, [])
        for bad in ("vortex_interval = 10, ", "move_interval = 999999999, 60, "):
            t, _ = self.translate(self.FULL.replace("&domains ", "&domains " + bad))
            self.assertTrue(any("moving nest option" in e for e in t.errors), (bad, t.errors))

    def test_omitted_nest_parent_id_rejected(self):
        # Registry default parent_id = 0; omitted nest parent_id must not be inferred as dom_id - 1
        three = ("&time_control input_from_file = .true., .true., .true. /\n"
                 "&domains time_step = 60, max_dom = 3, e_we = 120, 121, 91, e_sn = 90, 91, 61, e_vert = 45, 45, 45,"
                 " dx = 15000, 5000, 1000, dy = 15000, 5000, 1000, parent_id = 0, 1,"
                 " i_parent_start = 1, 40, 30, j_parent_start = 1, 30, 20,"
                 " parent_grid_ratio = 1, 3, 5, parent_time_step_ratio = 1, 3, 5 /\n"
                 "&bdy_control specified = .true., .false., .false. /\n")
        t, _ = self.translate(three)
        self.assertTrue(any("Domain d03: invalid parent_id = 0" in e for e in t.errors), t.errors)
        no_pid = self.NEST.replace("parent_id = 0, 1, ", "")
        t, _ = self.translate(no_pid)
        self.assertTrue(any("Domain d02: invalid parent_id = 0" in e for e in t.errors), t.errors)

    def test_nest_s_we_s_sn_no_d0_fallback(self):
        # Registry default s_we / s_sn = 1: a child omitting its entry gets 1, not d01's value
        t, _ = self.translate(self.NEST.replace("e_we = 120, 121,", "s_we = 2, e_we = 120, 121,"))
        self.assertTrue(any("s_we other than 1" in e for e in t.errors), t.errors)
        self.assertFalse(any("not divisible by parent_grid_ratio" in e for e in t.errors), t.errors)

    def test_inactive_options_accepted_with_notice(self):
        # Inactive options ignored by WRF: opt_thcnd outside Noah, cugd_avedx outside G3, shadlen with
        # topo_shading = 0, gwd_dx_factor on domains with gwd_opt = 0
        for lsm in (1, 3, 4):
            text = (f"&time_control /\n&domains time_step = 60 /\n&physics mp_physics = 6, sf_sfclay_physics = 1,"
                    f" sf_surface_physics = {lsm}, bl_pbl_physics = 1, opt_thcnd = 2 /\n")
            t, sec = self.translate(real_case(text))
            self.assertEqual(t.errors, [], lsm)
            self.assertNotIn("opt_thcnd", sec["phys"])
            self.assertTrue(any("opt_thcnd = 2" in n for n in t.notices), (lsm, t.notices))
        t, sec = self.translate(self.FULL.replace("&physics ", "&physics opt_thcnd = 2, "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("phys", sec)

        for text in (real_case("&time_control /\n&domains time_step = 60 /\n&physics sf_sfclay_physics = 1,"
                               " sf_surface_physics = 2, cu_physics = 1, cugd_avedx = 3 /\n"),
                     self.FULL.replace("&physics ", "&physics cugd_avedx = 3, maxens3 = 12, ")):
            t, sec = self.translate(text)
            self.assertEqual(t.errors, [], text)
            self.assertNotIn("cugd_avedx", sec.get("phys", {}))
            self.assertTrue(any("cugd_avedx = 3" in n for n in t.notices), t.notices)

        t, sec = self.translate(self.FULL.replace("&physics ", "&physics topo_shading = 0, shadlen = 50000., "))
        self.assertEqual(t.errors, [])
        self.assertNotIn("phys", sec)
        self.assertTrue(any("shadlen = 50000.0" in n for n in t.notices), t.notices)

        t, sec = self.translate(self.NEST + "&dynamics gwd_opt = 1, 0, gwd_dx_factor = 3.0 /\n")
        self.assertEqual(t.errors, [])
        self.assertEqual(sec["dyn"]["gwd_dx_factor"], 3.0)
        t, _ = self.translate(self.NEST + "&dynamics gwd_opt = 1, 1, gwd_dx_factor = 3.0 /\n")
        self.assertTrue(any("gwd_dx_factor" in e for e in t.errors), t.errors)
        t, sec = self.translate(self.NEST + "&dynamics gwd_opt = 0, 0, gwd_dx_factor = 3.0 /\n")
        self.assertEqual(t.errors, [])
        self.assertNotIn("gwd_dx_factor", sec["dyn"])
        self.assertTrue(any("gwd_dx_factor" in n for n in t.notices), t.notices)

    def test_grid_allowed_false_default_only_error(self):
        # grid_allowed = .false. hits DEFAULT_ONLY_OPTIONS with its specific message
        t, _ = self.translate(self.NEST.replace("max_dom = 2,", "max_dom = 2, grid_allowed = .true., .false.,"))
        self.assertTrue(any("domains that are not integrated (grid_allowed)" in e for e in t.errors), t.errors)


class TestExecution2Steps(unittest.TestCase):
    """Verify that generated INI executes for 2 steps (binary: $CPPWRF_BIN, default build/cppwrf)."""

    def test_run_r0_2steps(self):
        exe = os.environ.get("CPPWRF_BIN", os.path.join(REPO_ROOT, "build", "cppwrf"))
        if not os.path.exists(exe):
            self.skipTest(f"{exe} not found (set CPPWRF_BIN)")

        nml_path = wrf_runs_path("run_R0_r8/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")

        with tempfile.NamedTemporaryFile(suffix=".ini", mode="w", delete=False) as tf:
            ini_path = tf.name

        try:
            # Generate INI using translator CLI
            cmd_trans = [sys.executable, os.path.join(REPO_ROOT, "tools", "wrf2cppwrf.py"),
                         nml_path, "-o", ini_path]
            ret_trans = subprocess.run(cmd_trans, capture_output=True, text=True)
            self.assertEqual(ret_trans.returncode, 0, f"Translator failed: {ret_trans.stderr}")

            # Run 2 steps with heavy.sh
            cmd_run = [os.path.join(REPO_ROOT, "tools", "heavy.sh"), exe, ini_path,
                       "--set", "run.run_time=120"]
            ret_run = subprocess.run(cmd_run, capture_output=True, text=True, cwd=REPO_ROOT)
            self.assertEqual(ret_run.returncode, 0, f"Run failed: {ret_run.stderr}\n{ret_run.stdout}")
            self.assertIn("done, 2 steps", ret_run.stdout)
        finally:
            if os.path.exists(ini_path):
                os.remove(ini_path)

    def test_run_n1_2steps(self):
        exe = os.environ.get("CPPWRF_BIN", os.path.join(REPO_ROOT, "build", "cppwrf"))
        if not os.path.exists(exe):
            self.skipTest(f"{exe} not found (set CPPWRF_BIN)")

        nml_path = wrf_runs_path("run_N1_r8/steps/namelist.input")
        if not os.path.exists(nml_path):
            self.skipTest(f"Namelist not found: {nml_path}")

        with tempfile.NamedTemporaryFile(suffix=".ini", mode="w", delete=False) as tf:
            ini_path = tf.name

        try:
            cmd_trans = [sys.executable, os.path.join(REPO_ROOT, "tools", "wrf2cppwrf.py"),
                         nml_path, "-o", ini_path]
            ret_trans = subprocess.run(cmd_trans, capture_output=True, text=True)
            self.assertEqual(ret_trans.returncode, 0, f"Translator failed: {ret_trans.stderr}")

            cmd_run = [os.path.join(REPO_ROOT, "tools", "heavy.sh"), exe, ini_path,
                       "--set", "run.run_time=120"]
            ret_run = subprocess.run(cmd_run, capture_output=True, text=True, cwd=REPO_ROOT)
            self.assertEqual(ret_run.returncode, 0, f"Nested run failed: {ret_run.stderr}\n{ret_run.stdout}")
        finally:
            if os.path.exists(ini_path):
                os.remove(ini_path)


if __name__ == "__main__":
    unittest.main()
