#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Ensemble runs (docs/ENSEMBLE.md): every member's files are byte-identical to that member run alone.

A tiny squall2d (analytic, no data) as an ensemble of 3 members: the configuration as it is (Kessler), WSM6
(mp_physics 6, other species) and another initial state (a warmer bubble in shear). The ensemble runs with one host
thread per member (mode = threads, the default) and one member after the other (mode = sequential); every file of
each member directory must equal, byte for byte, the file of the plain single run of squall2d.ini with the member's
overrides as --set (OMP_NUM_THREADS of the single runs: the environment's; the ensemble's members get a share of it; diag.csv without
its wall-time column),
and of `--set ensemble.member=I`. Refused: an unknown ensemble key, a member above ensemble.members, two members with
one output.dir.

  ensemble_test.py CPPWRF_BINARY WORK_DIR   (from the repository root)
"""
import filecmp
import os
import shutil
import subprocess
import sys

BASE = ["domain.nx=24", "domain.nz=20", "run.run_time=120", "output.diag_interval=2", "output.out_interval=5"]
NC = ["output.format=both", "restart.interval=60"]
MEMBERS = [
    [],
    ["phys.mp_physics=6", "moist.species=qv,qc,qr,qi,qs,qg"],
    ["case.dtheta=3", "case.u_shear=5"],
]
# files whose contents are the run's own record (wall times, the command line), not model output
OWN = {"run.json"}


def same_file(a, b):
    """byte-identical; diag.csv without its wall_s column (the wall time of each row)"""
    if os.path.basename(a) != "diag.csv":
        return filecmp.cmp(a, b, shallow=False)
    rows = []
    for f in (a, b):
        with open(f) as h:
            lines = h.read().splitlines()
        col = lines[0].split(",").index("wall_s")
        rows.append([",".join(c for k, c in enumerate(ln.split(",")) if k != col) for ln in lines])
    return rows[0] == rows[1]


def main():
    binary, work = sys.argv[1], sys.argv[2]
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    fails = 0

    def note(ok, what):
        nonlocal fails
        print(f"{'ok  ' if ok else 'FAIL'} {what}")
        fails += not ok

    def run(name, cfg, sets, fail=False):
        cmd = [binary, cfg]
        for s in sets:
            cmd += ["--set", s]
        r = subprocess.run(cmd, capture_output=True, text=True)
        with open(os.path.join(work, name + ".log"), "w") as f:
            f.write(r.stdout + r.stderr)
        if fail:
            return r.returncode, r.stdout + r.stderr
        if r.returncode != 0:
            print(f"FAIL {name}: exit {r.returncode}: {(r.stdout + r.stderr).strip().splitlines()[-3:]}")
            sys.exit(1)
        return r.stdout

    with open("configs/squall2d.ini") as f:
        text = f.read()
    ens = os.path.join(work, "ens.ini")
    with open(ens, "w") as f:
        f.write(text + "\n[ensemble]\nmembers = 3\n")
        for i, m in enumerate(MEMBERS, 1):
            f.write(f"member{i} = {' | '.join(m)}\n")

    def same_dirs(a, b, what):
        fa = sorted(set(os.listdir(a)) - OWN)
        fb = sorted(set(os.listdir(b)) - OWN)
        if fa != fb or not fa:
            note(False, f"{what}: files {fa} against {fb}")
            return
        diff = [n for n in fa if not same_file(os.path.join(a, n), os.path.join(b, n))]
        note(not diff, f"{what}: {len(fa)} files byte-identical" if not diff else f"{what}: {diff} differ")

    # NetCDF builds: wrfout and restart files too (the members' host work under the ensemble's lock)
    probe = subprocess.run([binary, "configs/squall2d.ini"] + sum([["--set", x] for x in BASE + NC + [
        "run.run_time=6", f"output.dir={work}/probe"]], []), capture_output=True, text=True)
    if probe.returncode == 0:
        BASE.extend(NC)
    else:
        print("note: no NetCDF output in this build (the native files only)")
    singles = []
    for i, m in enumerate(MEMBERS, 1):
        d = os.path.join(work, f"single{i}")
        run(f"single{i}", "configs/squall2d.ini", BASE + m + [f"output.dir={d}"])
        singles.append(d)
    for mode in ("threads", "sequential"):
        d = os.path.join(work, mode)
        out = run(mode, ens, BASE + [f"output.dir={d}", f"ensemble.mode={mode}"])
        note("ensemble of 3 members" in out, f"{mode}: the ensemble's log line")
        for i in range(1, 4):
            same_dirs(os.path.join(d, f"member{i}"), singles[i - 1], f"{mode} member {i} against its single run")
    d = os.path.join(work, "one")
    run("one", ens, BASE + [f"output.dir={d}", "ensemble.member=2"])
    note(not os.path.exists(os.path.join(d, "member1")), "ensemble.member=2 runs member 2 only")
    same_dirs(os.path.join(d, "member2"), singles[1], "ensemble.member=2 against the single run")

    for name, sets, msg in [
        ("bad_key", ["ensemble.membr2=x"], "unknown configuration key 'ensemble.membr2'"),
        ("bad_index", ["ensemble.member4=phys.mp_physics=6"], "the ensemble has 3 members"),
        ("same_dir", [f"ensemble.member3=output.dir={work}/same_dir/member1"], "write to the same output.dir"),
        ("bad_mode", ["ensemble.mode=batch"], "ensemble.mode: threads or sequential"),
    ]:
        rc, log = run(name, ens, BASE + [f"output.dir={work}/{name}"] + sets, fail=True)
        note(rc != 0 and msg in log, f"{name}: refused with '{msg}'" if rc != 0 and msg in log
             else f"{name}: exit {rc}, no '{msg}' in {log.strip().splitlines()[-2:]}")
    print(f"ensemble: {fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
