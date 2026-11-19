#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""tools/ci/affected_cases.py (changed files -> suite cases for development CI runs) and its table
tools/ci/affected_map.txt: every src/phys/* directory mapped, every scheme selector selecting cases, the core sample,
synthetic changes. Reads this checkout's cases and configurations (no git history, no data, no build)."""
import os
import re
import sys
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "ci"))
import affected_cases as ac  # noqa: E402

RULES, CORE = ac.load_map()
CASES = ac.load_cases(REPO)
NAMES = [c.name for c in CASES]
BY = {c.name: c for c in CASES}


def run(files, **kw):
    return ac.affected(files, CASES, RULES, CORE, REPO, **kw)


def with_opt(key, *vals):
    key = key if "." in key else f"phys.{key}"
    return {c.name for c in CASES if not c.slow and any(ac.same(c.conf.get(key, "0"), v) for v in vals)}


class Map(unittest.TestCase):
    def test_every_phys_directory_has_its_own_line(self):
        dirs = sorted(d for d in os.listdir(os.path.join(REPO, "src", "phys"))
                      if os.path.isdir(os.path.join(REPO, "src", "phys", d)))
        self.assertGreater(len(dirs), 30)
        for d in dirs:
            rule = ac.rule_for(f"src/phys/{d}/some_file.cpp", RULES)
            self.assertIsNotNone(rule, d)
            self.assertTrue(rule[0].startswith(f"src/phys/{d}/"), f"src/phys/{d} falls to {rule[0]}")

    def test_selectors_select_cases(self):
        for pat, _rx, sels in RULES:  # every option, each: and cases: selector of the table
            for sel in sels:
                if sel in ("core", "docs", "checks", "config", "group"):
                    continue
                self.assertTrue(ac.select(sel, pat, [c for c in CASES if not c.slow], CORE),
                                f"{pat}: {sel} selects no case")

    def test_core_sample(self):
        self.assertTrue(CORE)
        self.assertEqual([n for n in CORE if n not in NAMES], [])
        core = set(CORE)
        self.assertIn("real_r1_fp64_cpu", core)                        # dycore
        self.assertTrue(any(BY[n].t >= 3600 for n in core))             # one full-physics 1 h case
        self.assertTrue(any(BY[n].conf.get("domains.max_dom", "1") == "2" for n in core))  # a nest
        self.assertTrue(any(n.startswith("restart_") or "_rst" in n for n in core))      # a restart
        self.assertTrue(any(n.startswith("ideal_") for n in core))                         # an ideal case

    def test_every_tracked_kind_of_path_is_mapped(self):
        for p in ("CMakeLists.txt", "src/dyn/rk3.cpp", "src/core/halo.cpp", "src/io/config.cpp",
                  "src/nest/nest_bdy.hpp",
                  "src/cases/model.cpp", "configs/real_p4_r8.ini", "docs/INSTALL.md", "README.md",
                  "tests/verify/wrf_suite.py", "tests/verify/cases/48_nssl.py", "tools/ci/ci_submit.sh",
                  "python/cppwrf/run.py", "tests/phys/mp_nssl/compare.py", "cmake/x.cmake"):
            self.assertIsNotNone(ac.rule_for(p, RULES), p)

    def test_glob(self):
        self.assertTrue(ac.glob_re("src/phys/mp_nssl/**").match("src/phys/mp_nssl/a/b.hpp"))
        self.assertFalse(ac.glob_re("src/phys/mp_nssl/**").match("src/phys/mp_nssl2/a.hpp"))
        self.assertTrue(ac.glob_re("**/*.md").match("docs/tr/a.md"))
        self.assertFalse(ac.glob_re("*.md").match("docs/a.md"))
        self.assertTrue(ac.glob_re("tests/verify/cases/[0-9][0-9]_*.py").match("tests/verify/cases/48_nssl.py"))
        self.assertFalse(ac.glob_re("tests/verify/cases/[0-9][0-9]_*.py").match("tests/verify/cases/common.py"))


class Selections(unittest.TestCase):
    def test_nssl_scheme(self):
        names, warn = run(["src/phys/mp_nssl/nssl_kernel.hpp"])
        self.assertEqual(warn, [])
        self.assertEqual(set(names), with_opt("mp_physics", 17, 18))
        self.assertIn("real_p4n18_r8_fp64_cpu", names)
        self.assertIn("real_p4n18_r8_fp64_cpu_1h", names)       # 1 h versions for the scheme itself
        self.assertNotIn("real_r1_fp64_cpu", names)

    def test_docs_only(self):
        names, warn = run(["docs/USER_GUIDE.md", "README.md", "docs/tr/README.md", "LICENSE"])
        self.assertEqual((names, warn), ([], []))

    def test_tools_only_is_checks(self):
        log = []
        names, warn = run(["tools/ci/ci_submit.sh", "tests/phys/mp_nssl/compare.py"], log=log)
        self.assertEqual((names, warn), ([], []))
        self.assertTrue(any("translator" in x for x in log))

    def test_core(self):
        names, _ = run(["src/dyn/small_step.cpp"])
        self.assertEqual(set(names), set(CORE))
        self.assertEqual([n for n in names if BY[n].t > ac.SHORT_MAX], ["real_p4_r8_fp64_cpu_1h"])

    def test_core_plus_scheme(self):
        names, _ = run(["src/dyn/rk3.cpp", "src/phys/mp_thompson/thompson.cpp"], all_long=True)
        self.assertEqual(set(names), set(CORE) | with_opt("mp_physics", 8, 28))

    def test_broad_scheme_keeps_few_long_cases(self):
        wsm6 = with_opt("mp_physics", 6)
        self.assertGreater(len(wsm6), ac.BROAD)
        log = []
        names, _ = run(["src/phys/mp_wsm6/wsm6_does_not_exist.cpp"], log=log)
        self.assertEqual({n for n in names if BY[n].short}, {n for n in wsm6 if BY[n].short})
        longs = [n for n in names if not BY[n].short]
        self.assertEqual(len(longs), ac.LONG_MAX)
        self.assertEqual(len({BY[n].cfgfile for n in longs}), ac.LONG_MAX)
        self.assertTrue(any("broad scheme mp_physics=6" in x for x in log))
        # a scheme of few cases keeps all its long versions
        names, _ = run(["src/phys/bl_mynn/x.cpp"])
        self.assertEqual(set(names), with_opt("bl_pbl_physics", 5))

    def test_unmapped_is_core_with_warning(self):
        names, warn = run(["some/new/place.cpp"])
        self.assertEqual(set(names), set(CORE))
        self.assertEqual(len(warn), 1)
        self.assertIn("some/new/place.cpp", warn[0])

    def test_included_header_selects_the_including_schemes(self):
        hdr = "src/phys/mp_radar/" + sorted(os.listdir(os.path.join(REPO, "src/phys/mp_radar")))[0]
        users = {ac.scheme_dir(f) for f in ac.includers(REPO, [hdr])[hdr]}
        self.assertIn("mp_thompson", users)
        names, _ = run([hdr], all_long=True)
        self.assertTrue(with_opt("mp_physics", 8, 28) <= set(names))
        # a .cpp file that nothing includes selects its own scheme only
        names, _ = run(["src/phys/cu_bmj/bmj_does_not_exist.cpp"])
        self.assertEqual(set(names), with_opt("cu_physics", 2))

    def test_config_and_group(self):
        names, _ = run(["configs/real_p4n18_r8.ini"])
        self.assertEqual(set(names), {c.name for c in CASES if c.cfgfile == "configs/real_p4n18_r8.ini" and not c.slow})
        self.assertIn("real_p4n18_r8_fp64_cpu", names)
        names, _ = run(["tests/verify/cases/36_bmj.py"])
        self.assertTrue(names)
        self.assertTrue(all(BY[n].group == "tests/verify/cases/36_bmj.py" for n in names))

    def test_driver_one_case_per_value(self):
        names, _ = run(["src/phys/driver/cumulus_driver.cpp"])
        extra = set(names) - set(CORE)
        vals = {ac.num(BY[n].conf.get("phys.cu_physics")) for n in extra}
        self.assertGreaterEqual(len(vals), 5)
        self.assertLessEqual(len(extra), len(vals))
        self.assertTrue(all(BY[n].short for n in extra))

    def test_short_and_slow(self):
        names, _ = run(["src/phys/mp_nssl/nssl.cpp"], short=True)
        self.assertTrue(names and all(BY[n].short for n in names))
        names, _ = run(["src/phys/bl_ysu/ysu.cpp"])
        self.assertFalse(any(BY[n].slow for n in names))
        slow, _ = run(["src/phys/bl_ysu/ysu.cpp"], slow=True, all_long=True)
        self.assertTrue(any(BY[n].slow for n in slow))

    def test_order_is_the_suites(self):
        names, _ = run(["src/dyn/rk3.cpp", "src/phys/mp_nssl/nssl.cpp"])
        self.assertEqual(names, [n for n in NAMES if n in set(names)])

    def test_comment_only_changes(self):
        d = "--- a/x\n+++ b/x\n@@ -1 +1 @@\n-# old words\n+# new words\n+\n"
        self.assertTrue(ac.comment_only_diff("data/option_tiers.txt", d))
        self.assertTrue(ac.comment_only_diff("tools/wrf2cppwrf.py", d))
        self.assertFalse(ac.comment_only_diff("src/dyn/rk3.cpp", d))   # # is the preprocessor there
        self.assertTrue(ac.comment_only_diff("src/dyn/rk3.cpp", d.replace("#", "//")))
        self.assertFalse(ac.comment_only_diff("configs/a.ini", d + "+mp_physics = 6\n"))
        self.assertFalse(ac.comment_only_diff("docs/a.md", d))
        self.assertFalse(ac.comment_only_diff("a.py", "new file mode 100644\n" + d))
        names, _ = run(["src/dyn/rk3.cpp", "README.md"], comments={"src/dyn/rk3.cpp"})
        self.assertEqual(names, [])


class Cli(unittest.TestCase):
    def test_files_option(self):
        import io
        import contextlib
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = ac.main(["--files", "src/phys/mp_nssl/nssl.cpp,docs/FAQ.md"])
        self.assertEqual(rc, 0)
        self.assertEqual(set(out.getvalue().strip().split(",")), with_opt("mp_physics", 17, 18))
        self.assertTrue(re.search(r"\d+ cases", err.getvalue()))

    def test_ci_submit_submits_nothing_without_cases(self):
        import shutil
        import subprocess
        import tempfile
        if subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"], capture_output=True).returncode != 0:
            self.skipTest("not a git checkout")
        with tempfile.TemporaryDirectory() as d:
            calls = os.path.join(d, "calls")
            gh = os.path.join(d, "gh")
            with open(gh, "w") as fh:
                fh.write(f'#!/bin/sh\necho "$*" >> {calls}\nexit 1\n')
            os.chmod(gh, 0o755)
            env = dict(os.environ, PATH=d + os.pathsep + os.environ["PATH"])
            # HEAD..HEAD: no changed file, no case
            r = subprocess.run(["bash", os.path.join(REPO, "tools", "ci", "ci_submit.sh"), "HEAD", "--affected", "HEAD",
                                "--no-wait"], env=env, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            self.assertIn("no suite case affected; nothing submitted", r.stdout)
            self.assertFalse(os.path.exists(calls), "gh was called")
            self.assertIsNotNone(shutil.which("bash"))


if __name__ == "__main__":
    unittest.main()
