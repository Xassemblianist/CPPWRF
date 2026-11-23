#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Ensemble runs (docs/ENSEMBLE.md): every member's files are byte-identical to that member run alone.

Analytic part (no data): a tiny squall2d as an ensemble of 3 members: the configuration as it is (Kessler), WSM6
(mp_physics 6, other species) and another initial state (a warmer bubble in shear). The ensemble runs with one host
thread per member (mode = threads, the default) and one member after the other (mode = sequential); in NetCDF builds
with the WRF files written by the output writer thread (output.write_queue_mb, default 1024) and without it
(write_queue_mb = 0). Every file of each member directory (recursively) must equal, byte for byte, the file of the
plain single run of squall2d.ini with the member's overrides as --set (OMP_NUM_THREADS of the single runs: the
environment's; the ensemble's members get a share of it), and of `--set ensemble.member=I` for every member:
diag.csv without its wall-time column, run.json without the output directories and output.write_queue_mb of its
configuration. Refused: an
unknown ensemble key, a member above ensemble.members, two members with one output.dir (also spelled
differently: dir/x/../dir), two members with one output.wrf_dir, an unknown mode.

--cumulus (data: ${CPPWRF_WRF_RUNS}/run_P8_r8/steps; exit 77 without it): P8 (Grell-Freitas, cu_physics = 3) and P8
with cu_physics = 0, both writing a restart file after 3 steps, in sequential and threads mode, against their
single runs: convtrans_prep's averaging counter is each run's own (a counter kept from an earlier member would
show in the second member's restart file).

  ensemble_test.py CPPWRF_BINARY WORK_DIR [--netcdf ON|OFF] [--cumulus]   (from the repository root)
"""
import filecmp
import json
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
P8_MEMBERS = [[], ["phys.cu_physics=0"]]


def same_file(a, b):
    """byte-identical; diag.csv without its wall_s column (the wall time of each row); run.json without the
    configuration's output directories (the member's and the single run's own) and output.write_queue_mb"""
    name = os.path.basename(a)
    if name == "diag.csv":
        rows = []
        for f in (a, b):
            with open(f, newline="") as h:
                lines = h.read().split("\n")
            col = lines[0].split(",").index("wall_s")
            rows.append([",".join(c for k, c in enumerate(ln.split(",")) if k != col) for ln in lines])
        return rows[0] == rows[1]
    if name == "run.json":
        js = []
        for f in (a, b):
            with open(f) as h:
                j = json.load(h)
            for k in ("dir", "wrf_dir", "write_queue_mb"):  # where the files go, and how (not what they hold)
                j.get("config", {}).get("output", {}).pop(k, None)
            js.append(j)
        return js[0] == js[1]
    return filecmp.cmp(a, b, shallow=False)


def files_of(d):
    out = []
    for root, _, names in os.walk(d):
        for n in names:
            out.append(os.path.relpath(os.path.join(root, n), d))
    return sorted(out)


def main():
    args = sys.argv[1:]
    binary, work = args[0], args[1]
    netcdf = "ON"
    if "--netcdf" in args:
        netcdf = args[args.index("--netcdf") + 1].upper()
    cumulus = "--cumulus" in args
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

    def same_dirs(a, b, what, expect=None):
        fa, fb = files_of(a), files_of(b)
        if fa != fb or not fa:
            note(False, f"{what}: files {fa} against {fb}")
            return
        missing = [e for e in (expect or []) if not any(f.startswith(e) for f in fa)]
        if missing:
            note(False, f"{what}: no {missing} among {fa}")
            return
        diff = [n for n in fa if not same_file(os.path.join(a, n), os.path.join(b, n))]
        note(not diff, f"{what}: {len(fa)} files byte-identical" if not diff else f"{what}: {diff} differ")

    def ensemble_ini(src, members, name):
        with open(src) as f:
            text = f.read()
        ens = os.path.join(work, name)
        with open(ens, "w") as f:
            f.write(text + f"\n[ensemble]\nmembers = {len(members)}\n")
            for i, m in enumerate(members, 1):
                f.write(f"member{i} = {' | '.join(m)}\n")
        return ens

    if cumulus:
        runs = os.environ.get("CPPWRF_WRF_RUNS", "")
        steps = os.path.join(runs, "run_P8_r8", "steps")
        if not runs or not os.path.isfile(os.path.join(steps, "wrfinput_d01")):
            print(f"skipped: no {steps}/wrfinput_d01 (CPPWRF_WRF_RUNS)")
            return 77
        # one history frame and one restart file at the end (steps of 60 s; P8 writes ~0.4 GB per frame of each kind)
        base = ["run.run_time=180", "restart.interval=180", "output.out_interval=3"] + (["output.format=both"] if netcdf == "ON" else [])
        ens = ensemble_ini("configs/real_p8_r8.ini", P8_MEMBERS, "p8.ini")
        singles = []
        for i, m in enumerate(P8_MEMBERS, 1):
            d = os.path.join(work, f"p8single{i}")
            run(f"p8single{i}", "configs/real_p8_r8.ini", base + m + [f"output.dir={d}"])
            singles.append(d)
        for mode in ("sequential", "threads"):
            d = os.path.join(work, f"p8{mode}")
            run(f"p8{mode}", ens, base + [f"output.dir={d}", f"ensemble.mode={mode}"])
            for i in range(1, len(P8_MEMBERS) + 1):
                same_dirs(os.path.join(d, f"member{i}"), singles[i - 1], f"P8 {mode} member {i} against its single run",
                          expect=["run.json"] + (["wrfout_d01", "wrfrst_d01"] if netcdf == "ON" else []))
        print(f"ensemble cumulus: {fails} failures")
        return 1 if fails else 0

    ens = ensemble_ini("configs/squall2d.ini", MEMBERS, "ens.ini")
    # NetCDF builds (CMake's): wrfout and restart files too (the members' host work and the output writer thread
    # under the one NetCDF lock)
    if netcdf == "ON":
        BASE.extend(NC)
    else:
        print("note: no NetCDF output in this build (the native files only)")
    singles = []
    for i, m in enumerate(MEMBERS, 1):
        d = os.path.join(work, f"single{i}")
        run(f"single{i}", "configs/squall2d.ini", BASE + m + [f"output.dir={d}"])
        singles.append(d)
    expect = ["run.json", "diag.csv"] + (["wrfout_d01", "wrfrst_d01"] if netcdf == "ON" else [])
    queues = ["1024", "0"] if netcdf == "ON" else ["0"]
    for mode in ("threads", "sequential"):
        for q in queues:
            d = os.path.join(work, f"{mode}_q{q}")
            out = run(f"{mode}_q{q}", ens, BASE + [f"output.dir={d}", f"ensemble.mode={mode}", f"output.write_queue_mb={q}"])
            note("ensemble of 3 members" in out, f"{mode} write_queue_mb={q}: the ensemble's log line")
            for i in range(1, 4):
                same_dirs(os.path.join(d, f"member{i}"), singles[i - 1],
                          f"{mode} write_queue_mb={q} member {i} against its single run", expect)
    for i in range(1, 4):
        d = os.path.join(work, f"one{i}")
        run(f"one{i}", ens, BASE + [f"output.dir={d}", f"ensemble.member={i}"])
        note(os.listdir(d) == [f"member{i}"], f"ensemble.member={i} runs member {i} only")
        same_dirs(os.path.join(d, f"member{i}"), singles[i - 1], f"ensemble.member={i} against the single run", expect)

    refusals = [
        ("bad_key", ["ensemble.membr2=x"], "unknown configuration key 'ensemble.membr2'"),
        ("bad_index", ["ensemble.member4=phys.mp_physics=6"], "the ensemble has 3 members"),
        ("same_dir", [f"ensemble.member3=output.dir={work}/same_dir/member1"], "write to the same output.dir"),
        ("same_dir_alias", [f"ensemble.member3=output.dir={work}/same_dir_alias/x/../member1"],
         "write to the same output.dir"),
        ("bad_mode", ["ensemble.mode=batch"], "ensemble.mode: threads or sequential"),
    ]
    if netcdf == "ON":
        refusals.append(("same_wrf_dir", [f"ensemble.member1=output.wrf_dir={work}/wrf",
                                          f"ensemble.member2=output.wrf_dir={work}/wrf"],
                         "write their WRF files to the same directory"))
    for name, sets, msg in refusals:
        rc, log = run(name, ens, BASE + [f"output.dir={work}/{name}"] + sets, fail=True)
        note(rc != 0 and msg in log, f"{name}: refused with '{msg}'" if rc != 0 and msg in log
             else f"{name}: exit {rc}, no '{msg}' in {log.strip().splitlines()[-2:]}")
    if netcdf == "ON":  # a WRF directory of the configuration: one per member under it
        d = os.path.join(work, "wrfroot")
        run("wrfroot", ens, BASE + [f"output.dir={d}", f"output.wrf_dir={d}_wrf", "ensemble.mode=sequential"])
        ok = all(any(f.startswith("wrfout_d01") for f in files_of(f"{d}_wrf/member{i}")) for i in range(1, 4))
        note(ok, "output.wrf_dir: each member's WRF files in <output.wrf_dir>/memberI")
    print(f"ensemble: {fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
