#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Plan, stage, submit and collect one CPPWRF performance session on Kaggle (2 x Tesla T4).

  kaggle_perf.sh --out DIR --metadata-dir DIR [--plan | --dry-run | --no-wait] [--rev REV] [--owner USER]
                 [--hours 4] [--reps 3] [--builds dp,id[,sp]] [--blocks mpi,single,nest,day,io] [--mbps 30]
  kaggle_perf.sh collect --out DIR [--owner USER]

The session measures what docs/PERF.md and docs/MPI.md leave "not measured yet" and a 2 x T4 node can measure with
the CUDA builds of one revision (M1-M8 of the NEON78 performance map, checked against the code; the plan's
`measures` say what each block answers and what is out of reach). Builds: cuda_dp (FP64) and cuda_sp_id (FP32
identity), both with CPPWRF_MPI, sm_75 (--builds dp,id,sp adds fast FP32). Cases: the suite's real-data domains with
their own inputs from the public data release (data-v1), only the input files of the runs used (no references, no
WRF output). Every timed configuration runs --reps times in rounds whose order alternates (A B, B A, ...); the
diagnostic runs (CPPWRF_KTIME, CPPWRF_MEMINFO, CPPWRF_HALO_STATS, CPPWRF_GRAPHS=stats, CPPWRF_IO_TIMING, which
synchronise or add timers) are run once each, first in their group (they warm it), and are never timed.

--plan writes DIR/plan.json and prints the plan with its estimate (offline); --dry-run also stages the private dataset
and the kernel in DIR (offline); otherwise the dataset USER/cppwrf-perf-<commit>-<plan>-src is created, the private
kernel USER/cppwrf-perf-<commit>-<plan> (machine NvidiaTeslaT4) is pushed and, unless --no-wait, its results are
downloaded when it ends (`collect` downloads them later). The worker is tools/kaggle/kg_perf_body.py; the kernel
wrapper, the release metadata reader and the checked data streaming are tools/kaggle/kaggle_proof.py's and
kg_proof_body.py's. Results: DIR/results/{results.md, results.json, records.jsonl, logs/, telemetry/, probe/}.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))
import kaggle_proof as proof  # noqa: E402
import kg_perf_body as body  # noqa: E402

GIB = proof.GIB
RESULTS = "kgperf-results.tgz"
JOB = "perf-job.json"
BUNDLE = {"kg_perf_body.py": HERE / "kg_perf_body.py", "kg_proof_body.py": HERE / "kg_proof_body.py",
          "telemetry.py": REPO / "tools/bench/telemetry.py", "bench_parse.py": REPO / "tools/bench/bench_parse.py"}
BUILDS = {"dp": {"double": "ON", "identity": "OFF"}, "id": {"double": "OFF", "identity": "ON"},
          "sp": {"double": "OFF", "identity": "OFF"}}

# The cases: suite configurations on their WRF real*8 inputs (the FP32 builds read them too, as in the paper
# benchmark). sph: stepping seconds per simulated hour on a T4, docs/PERF.md "Column work buffers" (PERF2, T4 with
# CPPWRF_DEVICE_MEMORY_MIB=5800, the new build); MORR and V1 have no T4 figure there: guesses from P4 (radt 1,
# Morrison two-moment: the G4 suite's 1 h case takes 1.25 x P4's; V1: 1.9 x P4, as on the G4 in docs/PERF.md F05),
# marked.
CASES = {
    "P1": {"config": "configs/real_p1_r8.ini", "run": "run_P1_r8", "tar": "suite_data.tar", "hours": 1,
           "sph": {"dp": 3.53, "id": 1.12, "sp": 1.1}},
    "P4": {"config": "configs/real_p4_r8.ini", "run": "run_P4_r8", "tar": "suite_data.tar", "hours": 1,
           "sph": {"dp": 15.71, "id": 11.34, "sp": 10.5}},
    "P8": {"config": "configs/real_p8_r8.ini", "run": "run_P8_r8", "tar": "suite_data.tar", "hours": 1,
           "sph": {"dp": 5.05, "id": 1.72, "sp": 1.7}},
    "D0": {"config": "configs/real_d0_r8.ini", "run": "run_D0_r8", "tar": "suite_data.tar", "hours": 1,
           "sph": {"dp": 10.60, "id": 3.57, "sp": 3.5}},
    "MORR": {"config": "configs/real_morr_r8.ini", "run": "run_MORR_r8", "tar": "port_c04_data.tar", "hours": 1,
             "sph": {"dp": 22.0, "id": 14.0, "sp": 13.0}, "guess": True},
    "N3": {"config": "configs/real_n3.ini", "run": "run_N3h_r8", "tar": "n3_data.tar", "hours": 1, "nested": True,
           "inputs": ("wrfinput_d01", "wrfbdy_d01", "wrfinput_d02"), "sph": {"dp": 55.89, "id": 28.87, "sp": 27.0}},
    "V1": {"config": "configs/real_v1_r8.ini", "run": "run_V1_r8", "tar": "suite_data.tar", "hours": 2,
           "sph": {"dp": 30.0, "id": 21.0, "sp": 20.0}, "guess": True},
}
OFF = "1000000"  # history and diagnostics off: only the frame at t = 0 (and the last step's state read)
DEFAULT_ENV = {"CPPWRF_GRAPHS": "1", "CPPWRF_HALO_OVERLAP": "1"}
DIAG_KTIME = {"CPPWRF_KTIME": "1", "CPPWRF_MEMINFO": "1"}
HALO = {"CPPWRF_HALO_STATS": "1"}
GSTATS = {"CPPWRF_GRAPHS": "stats"}
SETUP_S = {"packages": 180, "build": 1320, "probe": 600, "pack": 180}

MEASURES = {
    "M1": "OVL1 with one MPI rank per GPU: CPPWRF_HALO_OVERLAP=1 against 0 in 1x2 and 2x1 on P4, P8, D0 (FP64; P4 "
          "also FP32 identity), with CPPWRF_HALO_STATS' wait / transfer split per plan (docs/PERF.md OVL1: measured "
          "on one shared RTX 2060 only, where the overlap could not show).",
    "M2": "HALO1 and CUDA graphs under MPI on two GPUs: one GPU (plain) against 1x2 / 2x1, CPPWRF_GRAPHS=1 against 0 "
          "(plain and 1x2), CPPWRF_GRAPHS=stats (launches, instantiations, nodes per launch). The node's Open MPI "
          "is not GPU-aware (probe): the pinned host path; a GPU-aware MPI is out of reach here.",
    "M3": "Daytime RRTMG SW (PERF14 is in the build: its effect needs a base build, not planned): V1 (Bay of Bengal, "
          "00 UTC = about 06 local solar time: the shortwave works in every step, radt 1) for 2 h, the length the "
          "suite verifies, CPPWRF_KTIME kernel table and MEMINFO, FP64 and FP32 identity, timed twice each; set "
          "against docs/PERF.md F05's G4 V1 table (KSwPrep, KSwGpoint<48>).",
    "M4": "PERF13 (deferred feedback, nest graphs, KWW / KWPost) as v1.5.0 baselines on one T4: KTIME tables of P1, "
          "P4, P8, MORR, D0, N3 (60 steps, history off) to set against docs/PERF.md's G4 and T4 tables. The change "
          "itself needs a base build (not planned).",
    "M5": "Output (IO2 background writer, IO1 threaded deflate) on a 4-vCPU host: P4 1 h, a wrfout frame every 10 "
          "min, output.write_queue_mb 1024 against 0, CPPWRF_NC_THREADS=0 (HDF5's serial filter), FP64 and FP32 "
          "identity, wall time; CPPWRF_IO_TIMING in diagnostic runs; 1x2 with history on (the queue is off in MPI "
          "runs). The large domain L is not in any data tar (it is made with WPS and GFS): P4 instead.",
    "M6": "v1.5.0 single-GPU baselines on an unconstrained 15 GB T4: cuda_dp against cuda_sp_id (FP32 identity) on "
          "P1, P4, P8, MORR (the paper's C3 physics, its first KTIME profile), D0, N3; MEMINFO peaks and stack "
          "reserves; fast FP32 (cuda) with --builds dp,id,sp.",
    "M7": "The two-way nest N3 with two MPI ranks (one per GPU) against one GPU, CPPWRF_HALO_STATS of 1x2: the cost "
          "of the nest's whole-field copies and single-field exchanges under MPI.",
    "M8": "Node and binary probe (no model run): nvidia-smi topo (PHB expected), P2P access, pinned H2D / D2H and "
          "peer-copy bandwidths, both GPUs' copies at once, Open MPI's CUDA support, nsys / ncu presence, cuobjdump "
          "resource usage and the byte loads (.U8) of the RRTMG kernels (the tables are #pragma pack(1): "
          "rrtmg_lw_tables.hpp, rrtmg_sw_tables.hpp). The Noah / Tiedtke MPI stalls show in M1's collectives.",
}


def estimate(case, build, layout, env=None, frames=0):
    """Seconds of one run: stepping at the T4 rate (x1.6 with CPPWRF_KTIME, x0.8 on two GPUs), start-up, writes, and
    the pause, sampler and cool-down around it."""
    c = CASES[case]
    step = c["hours"] * c["sph"][build]
    if (env or {}).get("CPPWRF_KTIME") == "1":
        step *= 1.6
    if layout != "plain":
        step *= 0.8
    overhead = 10 + (6 if layout != "plain" else 0) + (8 if c.get("nested") else 0)
    return round(step + overhead + frames * (5 if build == "dp" else 4) + 8)


def case_sets(case, history=None):
    c = CASES[case]
    sets = [f"real.dir=@RUNS@/{c['run']}", f"run.run_time={c['hours'] * 3600}", f"output.diag_interval={OFF}",
            "output.wrf_timing=true"]
    if history is None:
        sets.append(f"output.out_interval={OFF}")
    else:
        sets += ["output.format=wrfout", f"output.out_interval={history}"]
    if c.get("nested"):
        sets += [f"d02.out_interval={OFF}", f"d02.diag_interval={OFF}"]
    return sets


class Planner:
    def __init__(self):
        self.runs, self.comparisons = [], []

    def group(self, block, measures, case, group, variants, diags, reps, history=None, comparisons=()):
        """variants / diags: (name, build, layout, env, extra sets); diagnostic runs first, then `reps` rounds of
        the variants, every second round in reverse order."""
        def add(kind, name, build, layout, env, extra, rep):
            e = dict(DEFAULT_ENV, **env)
            frames = 0 if history is None else CASES[case]["hours"] * 60 // history
            rid = re.sub(r"[^A-Za-z0-9]+", "-", f"{group}-{name}-{kind}{'' if rep is None else rep}").strip("-").lower()
            self.runs.append({"id": f"{len(self.runs):03d}-{rid}", "block": block, "group": group, "variant": name,
                              "kind": kind, "case": case, "build": build, "layout": layout, "rep": rep,
                              "order": len(self.runs), "measures": measures, "config": CASES[case]["config"],
                              "sets": case_sets(case, history) + list(extra), "env": e,
                              "nested": bool(CASES[case].get("nested")), "cpus": 4,
                              "estimate": estimate(case, build, layout, e, frames)})

        for name, build, layout, env, extra in diags:
            add("diag", name, build, layout, env, extra, None)
        for r in range(reps):
            for name, build, layout, env, extra in (variants if r % 2 == 0 else variants[::-1]):
                add("timed", name, build, layout, env, extra, r)
        for measure, base, new, m in comparisons:
            self.comparisons.append({"measure": measure, "group": group, "base": base, "new": new, "metric": m})


def plan_runs(blocks, builds, reps, single_cases=("P1", "P4", "P8", "MORR", "D0", "N3")):
    p = Planner()
    if "mpi" in blocks:
        for case, build in (("P4", "dp"), ("P8", "dp"), ("D0", "dp"), ("P4", "id")):
            if build not in builds:
                continue
            full = build == "dp"
            v = [("plain", build, "plain", {}, ()), ("1x2", build, "1x2", {}, ()),
                 ("1x2 ov0", build, "1x2", {"CPPWRF_HALO_OVERLAP": "0"}, ()), ("2x1", build, "2x1", {}, ())]
            if full:
                v += [("2x1 ov0", build, "2x1", {"CPPWRF_HALO_OVERLAP": "0"}, ()),
                      ("1x2 g0", build, "1x2", {"CPPWRF_GRAPHS": "0"}, ()),
                      ("plain g0", build, "plain", {"CPPWRF_GRAPHS": "0"}, ())]
            d = [("1x2 halo-stats", build, "1x2", HALO, ())]
            if full:
                d += [("2x1 halo-stats", build, "2x1", HALO, ()), ("1x2 graph-stats", build, "1x2", GSTATS, ()),
                      ("plain graph-stats", build, "plain", GSTATS, ())]
            c = [("M2", "plain", "1x2", "step_s"), ("M2", "plain", "2x1", "step_s"), ("M2", "plain", "1x2", "wall"),
                 ("M1", "1x2 ov0", "1x2", "step_s")]
            if full:
                c += [("M1", "2x1 ov0", "2x1", "step_s"), ("M2", "1x2 g0", "1x2", "step_s"),
                      ("M2", "plain g0", "plain", "step_s")]
            p.group("mpi", ["M1", "M2", "M8"], case, f"mpi/{case}/{build}", v, d, reps, comparisons=c)
    if "single" in blocks:
        for case in single_cases:
            v = [(b, b, "plain", {}, ()) for b in builds]
            d = [(f"{b} ktime", b, "plain", DIAG_KTIME, ()) for b in builds]
            c = [("M6", a, b, "step_s") for a, b in zip(builds, builds[1:])]
            p.group("single", ["M4", "M6"], case, f"single/{case}", v, d, reps, comparisons=c)
    if "nest" in blocks and "dp" in builds:
        v = [("plain", "dp", "plain", {}, ()), ("1x2", "dp", "1x2", {}, ()), ("2x1", "dp", "2x1", {}, ())]
        d = [("1x2 halo-stats", "dp", "1x2", HALO, ()), ("1x2 graph-stats", "dp", "1x2", GSTATS, ())]
        p.group("nest", ["M7"], "N3", "nest/N3/dp", v, d, min(reps, 2),
                comparisons=[("M7", "plain", "1x2", "step_s"), ("M7", "plain", "2x1", "step_s"),
                             ("M7", "plain", "1x2", "wall")])
    if "day" in blocks:
        bs = [b for b in builds if b in ("dp", "id")]
        p.group("day", ["M3"], "V1", "day/V1", [(b, b, "plain", {}, ()) for b in bs],
                [(f"{b} ktime", b, "plain", DIAG_KTIME, ()) for b in bs], min(reps, 2),
                comparisons=[("M3", a, b, "step_s") for a, b in zip(bs, bs[1:])])
    if "io" in blocks:
        for build in [b for b in builds if b in ("dp", "id")]:
            q = lambda mb: (f"output.write_queue_mb={mb}",)  # noqa: E731
            v = [("q1024", build, "plain", {}, q(1024)), ("q0", build, "plain", {}, q(0))]
            c = [("M5", "q0", "q1024", "wall")]
            d = [("q1024 io-timing", build, "plain", {"CPPWRF_IO_TIMING": "1"}, q(1024))]
            if build == "dp":
                v += [("q1024 nc0", build, "plain", {"CPPWRF_NC_THREADS": "0"}, q(1024)), ("1x2", build, "1x2", {}, ())]
                d += [("q0 io-timing", build, "plain", {"CPPWRF_IO_TIMING": "1"}, q(0))]
                c += [("M5", "q1024 nc0", "q1024", "wall"), ("M5", "q0", "1x2", "wall")]
            p.group("io", ["M5"], "P4", f"io/P4/{build}", v, d, reps if build == "dp" else min(reps, 2), history=10,
                    comparisons=c)
    for i, r in enumerate(p.runs):
        r["order"] = i
    return p.runs, p.comparisons


def tar_specs(release, cases):
    """The data tars of the cases with the input files of each run directory (no WRF output, no subdirectories)."""
    want = {}
    for case in sorted(cases):
        c = CASES[case]
        prefix = f"wrf_runs/{c['run']}/"
        members = sorted(p for p in release.index[c["tar"]] if p.startswith(prefix) and "/" not in p[len(prefix):]
                         and not re.match(r"(wrfout|wrfrst|auxhist)", p[len(prefix):]))
        names = {p[len(prefix):] for p in members}
        missing = [f for f in c.get("inputs", ("wrfinput_d01", "wrfbdy_d01")) if f not in names]
        if missing:
            raise ValueError(f"{case}: {c['tar']} has no {', '.join(missing)} in {prefix} (INDEX)")
        want.setdefault(c["tar"], set()).update(members)
    specs = {}
    for name, members in want.items():
        spec = dict(release.tar(name))
        spec.update(members=sorted(members), disk=spec["bytes"])
        specs[name] = spec
    return specs


def session_estimate(runs, builds, tars, mbps):
    setup = SETUP_S["packages"] + SETUP_S["build"] * len(builds) + SETUP_S["probe"] + SETUP_S["pack"]
    data = sum(t["bytes"] for t in tars.values()) / (mbps * 1e6)
    runs_s = sum(r["estimate"] for r in runs)
    # the data stream runs during the builds
    return {"setup_s": setup, "data_s": round(data), "runs_s": runs_s,
            "total_s": round(setup + max(0, data - SETUP_S["build"] * len(builds)) + runs_s)}


def make_job(args, src, commit, release):
    builds = [b for b in args.builds.split(",") if b]
    blocks = [b for b in args.blocks.split(",") if b]
    if not builds or any(b not in BUILDS for b in builds) or len(set(builds)) != len(builds):
        raise ValueError("--builds: a comma list of dp, id, sp")
    if any(b not in ("mpi", "single", "nest", "day", "io") for b in blocks):
        raise ValueError("--blocks: a comma list of mpi, single, nest, day, io")
    runs, comps = plan_runs(blocks, builds, args.reps)
    if not runs:
        raise ValueError("the selection plans no run")
    cases = {r["case"] for r in runs}
    for case in cases:
        if not (src / CASES[case]["config"]).is_file():
            raise ValueError(f"{CASES[case]['config']} is not in revision {commit}")
    tars = tar_specs(release, cases)
    est = session_estimate(runs, builds, tars, args.mbps)
    limit = round(args.hours * 3600)
    job = {"commit": commit, "limit": limit, "margin": 900, "data_url": proof.DATA_URL,
           "asset_limit": proof.ASSET_LIMIT, "reserve": 16 * GIB, "disk": sum(t["disk"] for t in tars.values()),
           "pause": 3, "cool_delta": 5, "cool_max": 60, "builds": {b: BUILDS[b] for b in builds}, "tars": tars,
           "runs": runs, "comparisons": comps, "measures": MEASURES, "estimate": est,
           "guesses": sorted(c for c in cases if CASES[c].get("guess"))}
    return job


def print_plan(job):
    est = job["estimate"]
    print(f"Revision {job['commit']}   Plan {job['id'][:10]}   builds {','.join(job['builds'])}")
    print("Block    Group               Configs   Timed  Diag   Est. min")
    seen = {}
    for r in job["runs"]:
        k = (r["block"], r["group"])
        s = seen.setdefault(k, {"v": set(), "t": 0, "d": 0, "s": 0})
        s["v"].add(r["variant"])
        s["t" if r["kind"] == "timed" else "d"] += 1
        s["s"] += r["estimate"]
    for (b, g), s in seen.items():
        print(f"{b:8} {g:19} {len(s['v']):8} {s['t']:6} {s['d']:5} {s['s'] / 60:9.1f}")
    print(f"Runs {len(job['runs'])}: {est['runs_s'] / 60:.0f} min; set-up (packages, {len(job['builds'])} builds of "
          f"{SETUP_S['build'] // 60} min, probe, packing) {est['setup_s'] / 60:.0f} min; data "
          f"{sum(t['bytes'] for t in job['tars'].values()) / 1e9:.2f} GB in {est['data_s'] / 60:.0f} min during the builds.")
    print(f"Estimated session {est['total_s'] / 3600:.2f} h (limit {job['limit'] / 3600:g} h, runs stop "
          f"{job['margin'] / 60:.0f} min before it); {2 * est['total_s'] / 3600:.2f} T4 device-hours.")
    for name, t in job["tars"].items():
        print(f"  {name}: {len(t['members'])} input files, {t['bytes'] / 1e9:.2f} GB streamed")
    if job["guesses"]:
        print(f"  T4 rates guessed (no T4 figure in docs/PERF.md): {', '.join(job['guesses'])}")


def stage(job, archive, out, owner):
    ds, kd = out / "dataset", out / "kernel"
    for d in (ds, kd):
        shutil.rmtree(d, ignore_errors=True)
        d.mkdir(parents=True)
    shutil.copyfile(archive, ds / "src.tgz.dat")
    for name, path in BUNDLE.items():
        shutil.copyfile(path, ds / name)
    proof.write_json(ds / JOB, job)
    slug = f"cppwrf-perf-{job['commit'][:10]}-{job['id'][:10]}"
    dataset = f"{owner}/{slug}-src"
    proof.write_json(ds / "dataset-metadata.json", {"title": f"{slug}-src", "id": dataset,
                                                    "licenses": [{"name": "other"}]})
    proof.write_json(kd / "kernel-metadata.json", {"id": f"{owner}/{slug}", "title": slug, "code_file": "run.py",
                     "language": "python", "kernel_type": "script", "is_private": True, "enable_gpu": True,
                     "enable_internet": True, "machine_shape": "NvidiaTeslaT4", "dataset_sources": [dataset],
                     "competition_sources": [], "kernel_sources": []})
    (kd / "run.py").write_text(proof.KERNEL.format(proof_id=job["id"], shard_name=JOB, body="kg_perf_body.py",
                                                   number=1, results=RESULTS))
    return dataset, f"{owner}/{slug}"


def submit(out, job, dataset, kernel):
    state = proof.kaggle("datasets", "status", dataset)
    if state.returncode:
        up = proof.kaggle("datasets", "create", "-p", str(out / "dataset"), "-r", "skip", timeout=1800)
        (out / "dataset.log").write_text(up.stdout)
        up.check_returncode()
    for _ in range(60):
        state = proof.kaggle("datasets", "status", dataset)
        if state.returncode == 0 and re.search(r"\bready\b", state.stdout, re.I):
            break
        if state.returncode == 0 and re.search(r"\b(error|failed)\b", state.stdout, re.I):
            raise ValueError(f"dataset {dataset}: {state.stdout.strip()}")
        time.sleep(10)
    else:
        raise ValueError(f"dataset {dataset} did not become ready")
    r = proof.kaggle("kernels", "push", "-p", str(out / "kernel"), "--timeout", str(job["limit"] + 300), timeout=300)
    (out / "push.log").write_text(r.stdout)
    print(f"{kernel}\n{r.stdout.strip()}", flush=True)
    version = re.search(r"Kernel version (\d+) successfully pushed", r.stdout, re.I)
    if r.returncode or not version:
        raise ValueError("kernel push failed (push.log)")
    (out / "submitted").write_text(f"{kernel}/{version[1]}\n")


def collect(out, wait):
    job = json.loads((out / "plan.json").read_text())
    slug = (out / "submitted").read_text().strip()
    deadline = time.monotonic() + job["limit"] + 3600
    while True:
        st = proof.kaggle("kernels", "status", slug)
        (out / "status.log").write_text(st.stdout)
        m = re.search(r'has status "(?:KernelWorkerStatus\.)?(\w+)"', st.stdout, re.I)
        state = m[1].lower() if m else "unknown"
        if state in {"complete", "error", "cancelled", "canceled"} or not wait or time.monotonic() > deadline:
            break
        time.sleep(60)
    print(f"{slug}: {state}")
    if state != "complete":
        return 1
    with tempfile.TemporaryDirectory(dir=out) as temp:
        r = proof.kaggle("kernels", "output", slug, "-p", temp, "--file-pattern", r"^kgperf-results\.tgz$", timeout=900)
        (out / "download.log").write_text(r.stdout)
        if r.returncode or not (Path(temp) / RESULTS).is_file():
            print("no results archive (download.log)")
            return 1
        (Path(temp) / RESULTS).replace(out / RESULTS)
    return report(out, job)


def report(out, job):
    res = out / "results"
    shutil.rmtree(res, ignore_errors=True)
    res.mkdir()
    with tarfile.open(out / RESULTS, "r:gz") as archive:
        archive.extractall(res, filter="data")
    receipt = json.loads((res / "done.json").read_text())
    if receipt.get("id") != job["id"]:
        raise ValueError("the results are not this plan's")
    if not (res / JOB).is_file():
        proof.write_json(res / JOB, job)
    agg = body.write_report(res, job, body.read_records(res / "records.jsonl"))
    print(f"exit {receipt['exit']}; {agg['counts']}; {res / 'results.md'}")
    return int(receipt["exit"] != 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", choices=("run", "collect"), default="run")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--metadata-dir", type=Path, help="data-v1 TARS.txt, SHA256SUMS and INDEX.tsv[.gz]")
    ap.add_argument("--rev", default="HEAD", help="the model revision to build (default HEAD)")
    ap.add_argument("--owner", help="Kaggle username (else KAGGLE_USERNAME or kaggle config view)")
    ap.add_argument("--plan", action="store_true", help="write and print the plan (offline)")
    ap.add_argument("--dry-run", action="store_true", help="plan and stage the upload files (offline)")
    ap.add_argument("--no-wait", action="store_true", help="push and return (collect later)")
    ap.add_argument("--hours", type=float, default=4.0, help="session limit incl. builds (default 4, at most 11)")
    ap.add_argument("--reps", type=int, default=3, help="rounds of each timed group (default 3; nest/day: at most 2)")
    ap.add_argument("--builds", default="dp,id", help="dp, id, sp (default dp,id)")
    ap.add_argument("--blocks", default="mpi,single,nest,day,io", help="in priority order (default all)")
    ap.add_argument("--mbps", type=float, default=30, help="data transfer estimate, MB/s (default 30)")
    args = ap.parse_args()
    out = args.out.resolve()
    if args.action == "collect":
        if not (out / "plan.json").is_file():
            ap.error(f"{out} has no plan.json")
        if (out / RESULTS).is_file():  # downloaded before: the report again
            return report(out, json.loads((out / "plan.json").read_text()))
        if not (out / "submitted").is_file():
            ap.error(f"{out}: nothing submitted")
        return collect(out, wait=False)
    if args.metadata_dir is None:
        ap.error("--metadata-dir is required to plan")
    if not 0 < args.hours <= 11 or not 1 <= args.reps <= 9 or args.mbps <= 0:
        ap.error("--hours in (0, 11], --reps in [1, 9], --mbps > 0")
    out.mkdir(parents=True, exist_ok=True)
    commit = proof.capture("git", "-C", str(REPO), "rev-parse", "--verify", "--end-of-options", args.rev + "^{commit}")
    with tempfile.TemporaryDirectory(dir=out) as temp:
        root = Path(temp)
        archive = root / "src.tgz"
        proof.run("git", "-C", str(REPO), "archive", "--format=tar.gz", "-o", str(archive), commit)
        src = root / "src"
        src.mkdir()
        proof.run("tar", "xzf", str(archive), "-C", str(src))
        if (src / "src/cases/commit.txt").read_text().strip() != commit:
            raise ValueError("src/cases/commit.txt of the archive does not name the revision")
        release = proof.ReleaseData(args.metadata_dir, args.metadata_dir)
        job = make_job(args, src, commit, release)
        job["bundle"] = {"src.tgz.dat": proof.file_digest(archive),
                         **{n: proof.file_digest(p) for n, p in BUNDLE.items()}}
        ident = json.dumps(job, sort_keys=True).encode() + Path(__file__).read_bytes()
        job["id"] = hashlib.sha256(ident).hexdigest()
        if job["estimate"]["total_s"] > job["limit"]:
            raise ValueError(f"estimated {job['estimate']['total_s'] / 3600:.2f} h exceeds --hours {args.hours:g}: "
                             "fewer --reps, --blocks or --builds")
        plan_path = out / "plan.json"
        if plan_path.is_file() and json.loads(plan_path.read_text()).get("id") != job["id"] and \
                (out / "submitted").is_file():
            raise ValueError(f"{out} holds a submitted different plan; choose another --out")
        proof.write_json(plan_path, job)
        print_plan(job)
        if args.plan and not args.dry_run:
            return 0
        owner = proof.owner_name(args.owner, offline=args.dry_run)
        dataset, kernel = stage(job, archive, out, owner)
    if args.dry_run:
        print(f"Dry run: staged {out / 'dataset'} and {out / 'kernel'} for {kernel}")
        return 0
    submit(out, job, dataset, kernel)
    return 0 if args.no_wait else collect(out, wait=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"kaggle_perf: {error}", file=sys.stderr)
        sys.exit(2)
