#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The suite cases a change affects, for development runs on the CI (tools/ci/ci_submit.sh --affected BASE).

  tools/ci/affected_cases.py BASE [HEAD] [--slow] [--short] [--all-long] [--map FILE]
  tools/ci/affected_cases.py --files PATH[,PATH] [REV] ...   (these paths instead of a git diff; the cases and
                                                              configurations of REV, default this checkout)

The changed files are `git diff --name-only BASE..HEAD` (HEAD default HEAD; both sides of a rename). Each file is
decided by the first matching line of tools/ci/affected_map.txt (the rules are documented there): a physics scheme's
files select the cases whose configuration selects that scheme (1 h versions included), the dynamics, core, I/O,
driver and build files the core sample, documentation nothing, tools and tests outside the suite no case (the run's
translator tests and the comparators' self-test). A file that no line matches selects the core sample, with a warning.
The cases and configurations are read from HEAD's tree (git archive), the CPU FP64 cases of tests/verify/wrf_suite.py
and its special checks (straka_wrf, restart_*_fp64); the slow cases (suite.slow) only with --slow. --short keeps only
the short versions (simulated time <= 1200 s). A scheme that more than 40 cases use (WSM6, YSU, Noah, RRTMG, ...)
selects all its short cases but only 6 of its long ones, each with another configuration (--all-long: all of them).

stdout: the selected case names comma-joined in the suite's order (ci_submit.sh --cases), an empty line when no case
is affected; stderr: a summary (each changed file, its rule and the cases it adds). Exit 0, or 2 on an error.

This is for development feedback only: every merge to main is proven by the full suite (ci_submit.sh --full) and
main's per-push suite run.
"""
import argparse
import glob
import importlib.util
import os
import re
import subprocess
import sys
import tarfile
import tempfile

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
MAP = os.path.join(REPO, "tools", "ci", "affected_map.txt")
SHORT_MAX = 1200.0  # simulated seconds: the 3-step, 5-step, 10 and 20 min versions
BROAD, LONG_MAX = 40, 6  # a scheme of more than BROAD cases (WSM6, YSU, Noah, RRTMG): LONG_MAX of its long versions
INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.M)


def git(*args):
    return subprocess.run(["git", "-C", REPO, *args], check=True, capture_output=True, text=True).stdout


# ---------------------------------------------------------------- the map

def glob_re(pat):
    """A path glob as a regex: ** any number of components, * and ? within one, [..] as fnmatch."""
    out, i = "", 0
    while i < len(pat):
        if pat.startswith("**/", i):
            out, i = out + "(?:.*/)?", i + 3
        elif pat.startswith("**", i):
            out, i = out + ".*", i + 2
        elif pat[i] == "*":
            out, i = out + "[^/]*", i + 1
        elif pat[i] == "?":
            out, i = out + "[^/]", i + 1
        elif pat[i] == "[":
            j = pat.index("]", i)
            out, i = out + pat[i:j + 1], j + 1
        else:
            out, i = out + re.escape(pat[i]), i + 1
    return re.compile(out + r"\Z")


def load_map(path=MAP):
    """(rules [(pattern, regex, [selector])], core sample [name])."""
    rules, core = [], []
    for n, line in enumerate(open(path), 1):
        line = line.split("#", 1)[0].strip() if not line.lstrip().startswith("#") else ""
        if not line:
            continue
        f = line.split()
        if f[0] == "@core":
            core += [x for x in f[1].split(",") if x]
            continue
        if len(f) < 2:
            raise SystemExit(f"{path}:{n}: a pattern without selectors")
        rules.append((f[0], glob_re(f[0]), f[1:]))
    return rules, core


def rule_for(path, rules):
    return next((r for r in rules if r[1].match(path)), None)


# ---------------------------------------------------------------- the cases at HEAD

def extract(rev, dest):
    """HEAD's configs/, tests/verify/ and src/ in `dest` (git archive)."""
    tar = os.path.join(dest, "t.tar")
    with open(tar, "wb") as fh:
        subprocess.run(["git", "-C", REPO, "archive", "--format=tar", rev, "configs", "tests/verify", "src"],
                       check=True, stdout=fh)
    with tarfile.open(tar) as t:
        kw = {"filter": "data"} if hasattr(tarfile, "data_filter") else {}
        t.extractall(os.path.join(dest, "tree"), **kw)
    return os.path.join(dest, "tree")


def read_ini(path):
    """{section.key: value} of a CPPWRF configuration file."""
    conf, sec = {}, None
    for line in open(path):
        line = line.split("#")[0].strip()
        if re.fullmatch(r"\[.+\]", line):
            sec = line[1:-1].strip()
        elif sec and "=" in line:
            k, v = line.split("=", 1)
            conf[f"{sec}.{k.strip()}"] = v.strip()
    return conf


class Case:
    def __init__(self, name, cfg, sets, t, group, slow):
        self.name, self.cfg, self.t, self.group, self.slow = name, cfg, float(t), group, slow
        self.sets = sets
        self.conf, self.cfgfile = {}, None  # the effective configuration; the configuration file it reads

    @property
    def short(self):
        return self.t <= SHORT_MAX


def load_cases(tree):
    """The CPU FP64 cases of the suite in its order, then its special checks: [Case]."""
    vdir = os.path.join(tree, "tests", "verify")
    os.environ.setdefault("CPPWRF_WRF_RUNS", os.path.expanduser("~/wrf_real480"))
    os.environ.setdefault("CPPWRF_WRF_REF", os.path.expanduser("~/wrf_ref480"))
    for m in [m for m in sys.modules if m == "cases" or m.startswith("cases.")]:
        del sys.modules[m]
    sys.path.insert(0, vdir)
    try:
        spec = importlib.util.spec_from_file_location("cases", os.path.join(vdir, "cases", "__init__.py"),
                                                      submodule_search_locations=[os.path.join(vdir, "cases")])
        mod = importlib.util.module_from_spec(spec)
        sys.modules["cases"] = mod
        spec.loader.exec_module(mod)
    finally:
        sys.path.remove(vdir)
    out = []
    for g in mod.GROUPS:
        gfile = "tests/verify/cases/" + g.__name__.split("._", 1)[1] + ".py"
        for name, key, cfg, sets, _wo, t, _tol in g.CASES:
            if key == "cpu_dp":
                out.append(Case(name, cfg, list(sets), t, gfile, "suite.slow" in sets))
    out.append(Case("straka_wrf", "configs/straka_wrf.ini", [], 0, "tests/verify/wrf_suite.py", False))
    for tag, cfg, _rd in mod.RESTARTS:
        out.append(Case(f"restart_{tag}_fp64", cfg, [], 0, "tests/verify/cases/restarts.py", False))
    cfgs = sorted(os.path.relpath(p, tree) for p in glob.glob(os.path.join(tree, "configs", "*.ini")))
    for c in out:
        path = c.cfg
        if path.startswith("dropin:"):  # the configuration of the WRF run directory it runs (real.dir names it)
            run = re.search(r"/(run_[^/]+)", path)
            path = next((p for p in cfgs if run and re.search(rf"/{re.escape(run.group(1))}(/|\s|$)",
                                                              open(os.path.join(tree, p)).read())), None)
        if path and os.path.exists(os.path.join(tree, path)):
            c.conf, c.cfgfile = read_ini(os.path.join(tree, path)), path
        for s in c.sets:
            if "=" in s and not s.startswith("suite."):
                k, v = s.split("=", 1)
                c.conf[k] = v
    return out


# ---------------------------------------------------------------- selectors

OPT = re.compile(r"([A-Za-z0-9_.]+)(!=|>=|=)(.+)")


def num(v):
    try:
        return float(str(v).strip().strip("'\""))
    except ValueError:
        return {".true.": 1.0, "true": 1.0, ".false.": 0.0, "false": 0.0}.get(str(v).strip().lower())


def same(a, b):
    na, nb = num(a), num(b)
    return na == nb if na is not None and nb is not None else str(a).strip().lower() == str(b).strip().lower()


def select(sel, path, cases, core):
    """The cases (a list of names in suite order) one selector picks for the changed file `path`, or a string for the
    special selectors docs / checks."""
    only_short = sel.startswith("short:")
    if only_short:
        sel = sel[len("short:"):]
    if sel in ("docs", "checks"):
        return sel
    if sel == "core":
        return [c.name for c in cases if c.name in core]
    if sel.startswith("each:"):  # one short case per value other than 0
        key = sel[len("each:"):]
        key = key if "." in key else f"phys.{key}"
        first = {}
        for c in cases:
            v = c.conf.get(key, "0")
            if c.short and not same(v, "0") and not c.name.startswith("dropin_"):
                first.setdefault(num(v) if num(v) is not None else v.lower(), c.name)
        return [c.name for c in cases if c.name in first.values()]
    if sel.startswith("cases:"):
        rx = re.compile(sel[len("cases:"):])
        hit = [c for c in cases if rx.search(c.name)]
    elif sel == "config":
        hit = [c for c in cases if c.cfgfile == path]
    elif sel == "group":
        hit = [c for c in cases if c.group == path]
    else:
        m = OPT.fullmatch(sel)
        if not m:
            raise SystemExit(f"affected_map.txt: unknown selector {sel}")
        key, op, vals = m.groups()
        key = key if "." in key else f"phys.{key}"
        vals = vals.split(",")

        def ok(c):
            v = c.conf.get(key, "0")
            if op == "=":
                return any(same(v, x) for x in vals)
            if op == "!=":
                return not any(same(v, x) for x in vals)
            return (num(v) or 0.0) >= float(vals[0])
        hit = [c for c in cases if not c.name.startswith("dropin_") or c.conf]
        hit = [c for c in hit if ok(c)]
    return [c.name for c in hit if c.short or not only_short]


# ---------------------------------------------------------------- includes between scheme directories

def scheme_dir(path):
    m = re.match(r"src/phys/([^/]+)/", path)
    return m.group(1) if m and m.group(1) != "driver" else None


def includers(tree, changed):
    """The files of src/phys/<scheme>/ (not the driver) that include a changed file, directly or through other files
    of scheme directories: {changed file: [including file]}."""
    src = os.path.join(tree, "src")
    rev = {}
    for d, _, files in os.walk(os.path.join(src, "phys")):
        for f in files:
            if not f.endswith((".hpp", ".h", ".cpp", ".cu", ".cuh", ".inc")):
                continue
            p = os.path.join(d, f)
            rel = os.path.relpath(p, tree)
            if not scheme_dir(rel):
                continue
            try:
                txt = open(p, errors="replace").read()
            except OSError:
                continue
            for inc in INCLUDE.findall(txt):
                for base in (d, src):
                    q = os.path.normpath(os.path.join(base, inc))
                    if os.path.exists(q):
                        rev.setdefault(os.path.relpath(q, tree), set()).add(rel)
                        break
    out = {}
    for f in changed:
        seen, todo = set(), [f]
        while todo:
            for g in rev.get(todo.pop(), ()):
                if g not in seen:
                    seen.add(g)
                    todo.append(g)
        out[f] = sorted(seen)
    return out


# ---------------------------------------------------------------- comment-only changes

HASH = (".py", ".sh", ".cmake", ".ini", ".txt", ".toml", ".cfg", ".yml", ".yaml")
SLASH = (".cpp", ".hpp", ".h", ".cc", ".cu", ".cuh", ".inc")


def comment_only(base, head, path):
    """True when every line the diff BASE..HEAD adds or removes in `path` is blank or a line comment
    (comment_only_diff)."""
    try:
        d = git("diff", "-U0", "--no-renames", f"{base}..{head}", "--", path)
    except subprocess.CalledProcessError:
        return False
    return comment_only_diff(path, d)


def comment_only_diff(path, diff):
    """True when every line a `git diff -U0` text adds or removes is blank or a line comment (# in Python, shell,
    CMake, .ini, .txt, YAML and CMakeLists.txt; // in C++ and CUDA). Block comments and docstrings do not count (the
    file is then decided by its rule); new and deleted files never do."""
    name = os.path.basename(path)
    mark = "#" if name.endswith(HASH) or name == "CMakeLists.txt" else "//" if name.endswith(SLASH) else None
    if mark is None or re.search(r"^(new|deleted) file mode", diff, re.M):
        return False
    lines = [x[1:] for x in diff.splitlines() if x[:1] in "+-" and not x.startswith(("+++", "---"))]
    return bool(lines) and all(not x.strip() or x.strip().startswith(mark) for x in lines)


# ---------------------------------------------------------------- main

def affected(files, cases, rules, core, tree, slow=False, short=False, log=None, comments=(), all_long=False):
    """(selected names in suite order, warnings). `log`: a list that receives the summary lines; `comments`: the
    changed files whose diff touches comments only (documentation). An option selector that picks more than BROAD
    cases keeps all its short ones and LONG_MAX of its long (1 h and longer) ones, each of another configuration file
    (the first in suite order), unless `all_long`."""
    log = log if log is not None else []
    warnings, chosen, kinds = [], set(), set()
    inc = includers(tree, [f for f in files if f.startswith("src/phys/")])
    usable = [c for c in cases if slow or not c.slow]
    by = {c.name: c for c in usable}
    limited = []

    def apply(path, rule, why=""):
        add = set()
        for sel in rule[2]:
            r = select(sel, path, usable, core)
            if isinstance(r, str):
                kinds.add(r)
                continue
            if not r and sel not in ("core",) and not sel.startswith("cases:"):
                warnings.append(f"{path}: selector {sel} selects no case; core sample instead")
                r = select("core", path, usable, core)
            longs = [n for n in r if not by[n].short]
            if not all_long and OPT.fullmatch(sel) and len(r) > BROAD and len(longs) > LONG_MAX:
                keep, cfgs = [], set()
                for n in longs:
                    if by[n].cfgfile not in cfgs and len(keep) < LONG_MAX:
                        keep.append(n)
                        cfgs.add(by[n].cfgfile)
                limited.append(f"{sel}: {len(keep)} of its {len(longs)} long cases (--all-long: all)")
                r = [n for n in r if by[n].short or n in keep]
            add |= set(r)
        chosen.update(add)
        new = [c.name for c in usable if c.name in add]
        log.append(f"  {path}{why}: {rule[0]} -> {' '.join(rule[2])}: "
                   f"{len(new)} case{'s' if len(new) != 1 else ''}"
                   + (f" ({', '.join(new[:6])}{', ...' if len(new) > 6 else ''})" if new else ""))

    for f in files:
        if f in comments:
            log.append(f"  {f}: comments only -> docs: 0 cases")
            continue
        rule = rule_for(f, rules)
        if rule is None:
            warnings.append(f"{f}: no line of tools/ci/affected_map.txt matches; core sample")
            rule = ("(unmapped)", None, ["core"])
        apply(f, rule)
        dirs = sorted({scheme_dir(g) for g in inc.get(f, ())} - {scheme_dir(f)})
        for d in dirs:  # schemes that include this file
            r = rule_for(f"src/phys/{d}/x", rules)
            if r:
                apply(f, r, f" (included by src/phys/{d})")
    for x in sorted(set(limited)):
        log.append(f"  broad scheme {x}")
    names = [c.name for c in usable if c.name in chosen and (c.short or not short)]
    if not names and "checks" in kinds:
        log.append("  no suite case: the translator tests and the comparators' self-test only")
    elif not names:
        log.append("  no suite case (documentation only)")
    return names, warnings


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("base", nargs="?", help="the base commit (git diff BASE..HEAD)")
    ap.add_argument("head", nargs="?", default="HEAD")
    ap.add_argument("--files", help="comma-separated changed paths instead of a git diff")
    ap.add_argument("--slow", action="store_true", help="the slow cases (suite.slow) too")
    ap.add_argument("--short", action="store_true", help="only the short versions (<= 1200 s simulated)")
    ap.add_argument("--all-long", action="store_true",
                    help=f"every long version of a scheme's cases (default: at most {LONG_MAX} per selector)")
    ap.add_argument("--map", default=MAP)
    ap.add_argument("--no-comment-filter", action="store_true",
                    help="decide files whose diff touches comments only by their rule too")
    a = ap.parse_args(argv)
    if a.files is not None:
        if a.base and a.head == "HEAD":  # --files PATHS REV
            a.head = a.base
        files = [x for x in a.files.split(",") if x]
    elif a.base:
        try:
            files = [x for x in git("diff", "--name-only", "--no-renames", f"{a.base}..{a.head}").splitlines() if x]
        except subprocess.CalledProcessError as e:
            print(f"affected_cases: git diff {a.base}..{a.head} failed: {e.stderr.strip()}", file=sys.stderr)
            return 2
    else:
        ap.error("BASE or --files is needed")
    rules, core = load_map(a.map)
    with tempfile.TemporaryDirectory(dir=os.environ.get("CPPWRF_SCRATCH") or None) as tmp:
        # --files without a revision: this checkout as it is
        tree = REPO if a.files is not None and a.head == "HEAD" else extract(a.head, tmp)
        cases = load_cases(tree)
        bad = [n for n in core if n not in [c.name for c in cases]]
        if bad:
            print(f"affected_cases: @core names unknown cases: {', '.join(bad)}", file=sys.stderr)
            return 2
        log = []
        comments = set() if a.files is not None or a.no_comment_filter else \
            {f for f in files if comment_only(a.base, a.head, f)}
        names, warnings = affected(files, cases, rules, core, tree, a.slow, a.short, log, comments, a.all_long)
    rng = f"{a.base}..{a.head}" if a.files is None else "--files"
    print(f"[affected] {rng}: {len(files)} changed file{'s' if len(files) != 1 else ''}", file=sys.stderr)
    for line in log:
        print(line, file=sys.stderr)
    for w in warnings:
        print(f"[affected] WARNING {w}", file=sys.stderr)
    nshort = sum(1 for c in cases if c.name in names and c.short)
    print(f"[affected] {len(names)} case{'s' if len(names) != 1 else ''} ({nshort} short, {len(names) - nshort} "
          f"1 h or longer) of {sum(1 for c in cases if a.slow or not c.slow)}", file=sys.stderr)
    print(",".join(names))
    return 0


if __name__ == "__main__":
    sys.exit(main())
