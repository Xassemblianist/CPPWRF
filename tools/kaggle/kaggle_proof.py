#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Plan, stage, submit and collect the two-T4 CPPWRF release proof."""

import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
import os
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
SCRATCH = Path(os.environ.get("CPPWRF_SCRATCH", Path.home() / "cppwrf_scratch"))
DATA_URL = "https://github.com/Xassemblianist/cppwrf-data/releases/download/data-v1"
GIB = 2**30
PART_BYTES = 512 * 2**20
ASSET_LIMIT = 2 * GIB
BUILDS = {"dp": "cuda_dp", "id": "cuda_sp_id", "sp": "cuda"}
KINDS = ("vs WRF tol 0", "vs WRF rel L2", "self only")
OVERLAY = "v05_files_data.tar"
BODY = "kg_proof_body.py"
RESULTS = "kgproof-results.tgz"
NAME = re.compile(r"[A-Za-z0-9_.-]+")
SHA = re.compile(r"[0-9a-fA-F]{64}")
CASE_RESULT = re.compile(r"^(PASS|FAIL|SKIP)\s+(\S+)(?:\s|$)")
PASS_KIND = re.compile(r"\[(vs WRF tol 0|vs WRF rel L2|self only)\]  \(\d+ s\)$")
CTEST_TOTAL = re.compile(r"^\d+% tests passed, \d+ tests failed out of [1-9]\d*.*$", re.M)


def run(*args, **kw):
    return subprocess.run(args, check=True, **kw)


def capture(*args, **kw):
    return run(*args, text=True, stdout=subprocess.PIPE, **kw).stdout.strip()


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2) + "\n")


def lines(path):
    return [s for line in path.read_text().splitlines() if (s := line.split("#", 1)[0].strip())]


def split_line(count):
    return (f"SPLIT PASS vs WRF at tolerance 0: {count[KINDS[0]]}; vs WRF by relative L2: {count[KINDS[1]]}; "
            f"CPPWRF self-consistency only: {count[KINDS[2]]}")


def suite_line(count):
    return f"SUITE PASS {count['PASS']} FAIL {count['FAIL']} SKIP {count['SKIP']}"


class ReleaseData:
    def __init__(self, metadata, g4):
        self.sums = {}
        for line in lines(metadata / "SHA256SUMS"):
            digest, asset = line.split()
            asset = asset.removeprefix("*")
            if not SHA.fullmatch(digest) or not NAME.fullmatch(asset) or asset in self.sums:
                raise ValueError(f"invalid or repeated asset in SHA256SUMS: {asset}")
            self.sums[asset] = digest.lower()
        self.sizes = {}
        for line in lines(metadata / "TARS.txt"):
            name, size, digest, count = line.split()
            if not NAME.fullmatch(name) or not SHA.fullmatch(digest) or name in self.sizes:
                raise ValueError(f"invalid or repeated tar in TARS.txt: {name}")
            self.sizes[name] = int(size), digest.lower(), int(count)
        index = metadata / "INDEX.tsv.gz"
        with (gzip.open(index, "rt") if index.is_file() else (metadata / "INDEX.tsv").open()) as f:
            self.index = defaultdict(set)
            for line in f:
                name, typ, path, *_ = line.rstrip("\n").split("\t")
                if typ == "0":
                    self.index[name].add(path.removeprefix("./"))
        self.g4 = g4
        self.specs = {}

    def tar(self, name):
        if name not in self.specs:
            self.specs[name] = self.read_tar(name)
        return self.specs[name]

    def read_tar(self, name):
        if not NAME.fullmatch(name) or not name.endswith(".tar"):
            raise ValueError(f"invalid data tar name: {name}")
        parts = sorted(
            (int(asset[len(name) + 5:]), asset) for asset in self.sums
            if re.fullmatch(re.escape(name) + r"\.part\d+", asset)
        )
        assets = [name] if name in self.sums else [asset for _, asset in parts]
        if not assets or (name not in self.sums and [i for i, _ in parts] != list(range(len(parts)))):
            raise ValueError(f"missing or noncontiguous assets for {name} in SHA256SUMS")
        size, digest, count = self.sizes.get(name, (0, None, len(assets)))
        if count != len(assets):
            raise ValueError(f"SHA256SUMS and TARS.txt disagree on the asset count for {name}")
        if not size and len(assets) < 2:
            raise ValueError(f"need an exact TARS.txt size for {name}")
        # C08 is in the release SHA list but has been omitted from some TARS.txt snapshots.
        transfer = size or PART_BYTES * len(assets)
        members = []
        disk = transfer
        if name == OVERLAY and (self.g4 / "pack_members.tsv").is_file():
            pairs = [line.split("\t") for line in lines(self.g4 / "pack_members.tsv")]
            packs, paths = zip(*pairs)
            members = sorted(set(paths))
            missing = set(members) - self.index[name]
            if missing:
                raise ValueError(f"{name} overlay members missing from INDEX: {sorted(missing)}")
            sizes = {n: int(b) for n, b in (line.split() for line in lines(self.g4 / "data_sizes.tsv"))}
            disk = min(transfer, sum(sizes[p] for p in set(packs)) + GIB)
        links = name + ".links"
        return {"name": name, "bytes": transfer, "exact": bool(size), "sha256": digest, "disk": disk,
                "assets": [[a, self.sums[a]] for a in assets], "members": members,
                "links": [links, self.sums[links]] if links in self.sums else None}


def historical_times(g4):
    result = {}
    for mode in BUILDS:
        log = g4 / "res_G4" / "g4res" / f"suite_{mode}.log"
        if log.is_file():
            for line in log.read_text().splitlines():
                match = CASE_RESULT.match(line)
                elapsed = re.search(r"\((\d+) s\)$", line)
                if match and elapsed:
                    result[mode, match[2]] = int(elapsed[1])
    return result


def suite_cases(src, g4, release, scale):
    suite = src / "tests/verify/wrf_suite.py"
    sys.path.insert(0, str(suite.parent))
    import wrf_suite as wrf

    modes = {c[0]: c[1] for c in wrf.CASES}
    modes.update({n: "cuda_dp" for n in wrf.SPECIAL if n.endswith("_fp64_cuda")})
    modes["restart_p1_fp32_cuda"] = "cuda"
    times = historical_times(g4)
    cases = []
    for mode, key in BUILDS.items():
        names = lines(g4 / f"{mode}_cases.txt")
        if not names or len(names) != len(set(names)):
            raise ValueError(f"empty or repeated cases in {mode}_cases.txt")
        for name in names:
            if modes.get(name) != key or not NAME.fullmatch(name):
                raise ValueError(f"{name} is not a {key} case at the candidate revision")
            # Use the candidate's CLI so map errors are surfaced before anything is submitted.
            tars = capture(sys.executable, "-B", str(suite), "--data-for", name, cwd=src).split(",")
            if name == "real_kesskf_r8_fp64_cuda_t3x1":
                # The tiled KESSKF case also reads run_P4_r8/steps from suite_data.tar.
                tars.append("suite_data.tar")
            for tar in tars:
                release.tar(tar)
            secs = times.get((mode, name), 15 if mode == "dp" else 8)
            cases.append({"name": name, "mode": mode, "tars": list(dict.fromkeys(tars)),
                          "kind": wrf.KINDS[wrf.kind(name)][1:-1],
                          "seconds": 2 + max(1, secs) * (scale if mode == "dp" else 3)})
    return cases


def plan_shards(src, args, release):
    cases = suite_cases(src, args.cases_dir, release, args.dp_scale)
    verify = src / "tests" / "verify"
    order = {name: i for i, name in enumerate(lines(verify / "vm_data.txt"))}

    def ordered(names):
        return sorted(names, key=lambda n: (order.get(n, len(order)), n))

    def total(names, field):
        return sum(release.tar(n)[field] for n in names)

    def make_batch(names, cases=(), ctest=False):
        lanes = [[], []]
        seconds = [0, 0]
        for case in sorted(cases, key=lambda c: (-c["seconds"], c["name"])):
            lane = min(range(2), key=seconds.__getitem__)
            lanes[lane].append(case)
            seconds[lane] += case["seconds"]
        disk = total(names, "disk")
        if disk > limit:
            raise ValueError(f"batch needs {disk / GIB:.2f} GiB with --disk-gib {args.disk_gib}: {', '.join(ordered(names))}")
        transfer = total(names, "bytes")
        return {"tars": ordered(names), "lanes": lanes, "ctest": ctest, "disk": disk,
                "transfer": transfer, "seconds": seconds[0] + seconds[1],
                "estimate": math.ceil(transfer / (args.mbps * 10**6) + (1800 if ctest else max(seconds)))}

    limit = int(args.disk_gib * GIB)
    groups = []
    for case in sorted(cases, key=lambda c: (-total(c["tars"], "bytes"), -c["seconds"], c["name"])):
        need = set(case["tars"])
        fits = [g for g in groups if total(need | g[0], "disk") <= limit]
        if fits:
            names, group = min(fits, key=lambda g: (total(need - g[0], "bytes"), -total(need & g[0], "bytes")))
            names.update(need)
            group.append(case)
        else:
            groups.append((need, [case]))
    batches = [make_batch(names, group) for names, group in groups]
    # Cover the CTests with external WRF inputs wherever the public release provides them.
    ctest_tars = set(lines(verify / "ctest_data.txt")) | {"ideal_data.tar", "suite_data.tar", "port_b03_data.tar"}
    ctest = make_batch(ctest_tars, ctest=True)
    overhead = round(args.build_hours * 3600)
    # Search by session count, then balance batches by estimated wall time. Keep the largest batch on its own
    # if necessary. Shard 1 always pays for the CTest builds and runs.
    min_sessions = min(2, len(batches))
    cap = args.session_hours * 3600
    for count in range(min_sessions, len(batches) + 2):
        shards = [{"batches": [], "estimate": overhead} for _ in range(count)]
        shards[0]["batches"].append(ctest)
        shards[0]["estimate"] += ctest["estimate"] + 900
        for batch in sorted(batches, key=lambda b: -b["estimate"]):
            shard = min(shards, key=lambda s: s["estimate"])
            shard["batches"].append(batch)
            shard["estimate"] += batch["estimate"]
        if max(s["estimate"] for s in shards) <= cap:
            break
    else:
        raise ValueError("a data batch plus its builds exceeds the session estimate limit; reduce --disk-gib or revise timings")
    shards = [s for s in shards if s["batches"]]
    for i, shard in enumerate(shards, 1):
        shard.update(number=i, disk=max(b["disk"] for b in shard["batches"]),
                     transfer=sum(b["transfer"] for b in shard["batches"]))
    return shards


def kernel_slug(plan, number):
    return f"cppwrf-proof-{plan['commit'][:10]}-{plan['id'][:10]}-s{number:02d}"


def print_plan(plan, selected):
    print(f"Candidate {plan['commit']}   Plan {plan['id'][:10]}")
    print("Shard   FP64   FP32-ID   FP32-fast  Batches  Tars   Peak GiB   Fetch GB   Est. h  Selected")
    for shard in plan["shards"]:
        n = shard["number"]
        batches = shard["batches"]
        counts = Counter(c["mode"] for b in batches for lane in b["lanes"] for c in lane)
        tars = {t for b in batches for t in b["tars"]}
        print(f"{n:5} {counts['dp']:6} {counts['id']:9} {counts['sp']:11} {len(batches):8} {len(tars):5}"
              f" {shard['disk'] / GIB:10.2f} {shard['transfer'] / 1e9:10.2f}"
              f" {shard['estimate'] / 3600:8.2f}  {'yes' if n in selected else ''}")
        for b in batches:
            print(f"  {'ctest' if b['ctest'] else 'suite'} {b['disk'] / GIB:.2f} GiB, "
                  f"GPU cases {'/'.join(str(len(lane)) for lane in b['lanes'])}  {','.join(b['tars'])}")
    hours = sum(s["estimate"] for s in plan["shards"]) / 3600
    print(f"Estimated total {hours:.2f} session-hours, {2 * hours:.2f} T4 device-hours; "
          f"reserve {plan['reserve'] / GIB:g} GiB beyond peak data on each shard.")
    if 2 * hours >= 30:
        print("WARNING estimated T4 device-hours meet or exceed 30", file=sys.stderr)
    for name, spec in plan["tars"].items():
        if not spec["exact"]:
            print(f"  {name}: bounded size from its {len(spec['assets'])} SHA256SUMS parts (512 MiB each)")


KERNEL = '''\
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

proof_id = {proof_id!r}
shard_name = {shard_name!r}
working = Path("/kaggle/working")
with tempfile.TemporaryDirectory(dir=working) as tmp:
    logs = Path(tmp)
    rc = 2
    try:
        for marker in Path("/kaggle/input").rglob(shard_name):
            job = json.loads(marker.read_text())
            if job["id"] == proof_id:
                break
        else:
            raise RuntimeError("private proof dataset or expected manifest not mounted")
        for name, digest in job["bundle"].items():
            with (marker.parent / name).open("rb") as stream:
                if hashlib.file_digest(stream, "sha256").hexdigest() != digest:
                    raise RuntimeError(f"wrong SHA256 for {{name}}")
        rc = subprocess.call(["timeout", "--kill-after=30", str(job["limit"]),
                              "python3", "-u", str(marker.parent / {body!r}), str(marker), tmp])
    except (OSError, ValueError, RuntimeError) as error:
        print(f"SETUP_FAILED {{error}}", flush=True)
        (logs / "setup.log").write_text(str(error) + "\\n")
    print(f"proof shard exit {{rc}}", flush=True)
    (logs / "done.json").write_text(json.dumps({{"id": proof_id, "shard": {number}, "exit": rc}}) + "\\n")
    # Kaggle publishes one artifact even for a known proof failure. The collector uses the exit receipt.
    with tarfile.open(working / {results!r}, "w:gz") as archive:
        for path in sorted(logs.iterdir()):
            archive.add(path, arcname=path.name)
'''


def prepare(plan, src_archive, out, owner, selected):
    ds = out / "dataset"
    ds.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src_archive, ds / "src.tgz.dat")
    shutil.copyfile(HERE / BODY, ds / BODY)
    slug = f"cppwrf-proof-{plan['commit'][:10]}-{plan['id'][:10]}-src"
    dataset = f"{owner}/{slug}"
    write_json(ds / "dataset-metadata.json", {"title": slug, "id": dataset, "licenses": [{"name": "other"}]})
    for shard in plan["shards"]:
        n = shard["number"]
        shard_name = f"shard-{n:02d}.json"
        tars = {name for b in shard["batches"] for name in b["tars"]}
        write_json(ds / shard_name, {**{key: plan[key] for key in ("id", "commit", "reserve", "limit", "bundle")},
                                     "data_url": DATA_URL, "asset_limit": ASSET_LIMIT,
                                     "shard": shard, "tars": {name: plan["tars"][name] for name in sorted(tars)}})
        if n not in selected:
            continue
        kernel = out / f"kernel-{n:02d}"
        kernel.mkdir(exist_ok=True)
        slug = kernel_slug(plan, n)
        write_json(kernel / "kernel-metadata.json", {"id": f"{owner}/{slug}", "title": slug, "code_file": "run.py",
                   "language": "python", "kernel_type": "script", "is_private": True, "enable_gpu": True,
                   "enable_internet": True, "machine_shape": "NvidiaTeslaT4", "dataset_sources": [dataset],
                   "competition_sources": [], "kernel_sources": []})
        (kernel / "run.py").write_text(KERNEL.format(proof_id=plan["id"], shard_name=shard_name,
                                                    body=BODY, number=n, results=RESULTS))
    return dataset


def file_digest(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def selected_shards(value, plan):
    have = {s["number"] for s in plan["shards"]}
    chosen = {int(n) for n in value.split(",")} if value else have
    if not chosen <= have:
        raise ValueError(f"unknown shards: {sorted(chosen - have)}")
    return chosen


def kaggle(*args, timeout=120):
    command = ["kaggle", *args]
    try:
        return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        partial = (error.stdout or b"").decode(errors="replace")
        return subprocess.CompletedProcess(command, 124, f"{partial}\nKaggle CLI timed out after {timeout} seconds\n")


def owner_name(value, offline=False):
    owner = value or os.environ.get("KAGGLE_USERNAME")
    if not owner and offline:
        return "dry-run"
    if not owner:
        config = kaggle("config", "view")
        match = re.search(r"(?m)^\s*-?\s*username:\s*(\S+)\s*$", config.stdout)
        owner = match[1] if match else None
    if not owner or not re.fullmatch(r"[A-Za-z0-9_-]+", owner):
        raise ValueError("set --owner or KAGGLE_USERNAME to the Kaggle account")
    return owner


def upload(out, plan, owner, dataset, selected):
    state = kaggle("datasets", "status", dataset)
    if state.returncode:
        upload = kaggle("datasets", "create", "-p", str(out / "dataset"), "-r", "skip", timeout=1800)
        (out / "dataset.log").write_text(upload.stdout)
        upload.check_returncode()
    for _ in range(60):
        state = kaggle("datasets", "status", dataset)
        if state.returncode == 0 and re.search(r"\bready\b", state.stdout, re.I):
            break
        if state.returncode == 0 and re.search(r"\b(error|failed)\b", state.stdout, re.I):
            raise ValueError(f"dataset {dataset}: {state.stdout.strip()}")
        time.sleep(10)
    else:
        raise ValueError(f"dataset {dataset} did not become ready")
    submitted = set()
    for n in sorted(selected):
        version_path = out / f"submitted-{n:02d}"
        version_path.unlink(missing_ok=True)
        result = kaggle("kernels", "push", "-p", str(out / f"kernel-{n:02d}"),
                        "--timeout", str(plan["limit"] + 300), timeout=300)
        (out / f"push-{n:02d}.log").write_text(result.stdout)
        print(f"{owner}/{kernel_slug(plan, n)}\n{result.stdout.strip()}", flush=True)
        version = re.search(r"Kernel version (\d+) successfully pushed", result.stdout, re.I)
        if result.returncode == 0 and version:
            version_path.write_text(version[1] + "\n")
            submitted.add(n)
    return submitted


def collect_download(out, plan, owner, selected, wait):
    pending = set(selected)
    downloaded = set()
    deadline = time.monotonic() + plan["limit"] + 3600
    while pending:
        for n in sorted(pending):
            slug = f"{owner}/{kernel_slug(plan, n)}"
            version_path = out / f"submitted-{n:02d}"
            if version_path.is_file():
                slug += "/" + version_path.read_text().strip()
            status = kaggle("kernels", "status", slug)
            match = re.search(r'has status "(?:KernelWorkerStatus\.)?(\w+)"', status.stdout, re.I)
            state = match[1].lower() if match else "unknown"
            (out / f"status-{n:02d}.log").write_text(status.stdout)
            if status.returncode and re.search(r"\b(401|403)\b|permission .* denied", status.stdout, re.I):
                pending.remove(n)
            elif status.returncode and not version_path.is_file() and re.search(r"404|not found", status.stdout, re.I):
                pending.remove(n)
            if status.returncode == 0 and state in {"complete", "error", "cancelled", "canceled"}:
                pending.remove(n)
                # A failed latest Kaggle session might expose output from a previous version of the kernel.
                if state != "complete":
                    continue
                root = out / f"shard-{n:02d}"
                root.mkdir(exist_ok=True)
                with tempfile.TemporaryDirectory(dir=root) as temp:
                    result = kaggle("kernels", "output", slug, "-p", temp,
                                    "--file-pattern", r"^kgproof-results\.tgz$", timeout=900)
                    (root / "download.log").write_text(result.stdout)
                    archive = Path(temp) / RESULTS
                    if result.returncode == 0 and archive.is_file():
                        archive.replace(root / RESULTS)
                        downloaded.add(n)
        if not wait or not pending or time.monotonic() >= deadline:
            break
        time.sleep(30)
    return downloaded


def read_case(text, case):
    lines = [line for line in text.splitlines() if line]
    rows = [(m[1], m[2], line) for line in lines if (m := CASE_RESULT.match(line))]
    if len(rows) != 1 or rows[0][1] != case["name"]:
        return None, None
    state, _, line = rows[0]
    if state == "PASS":
        kind = PASS_KIND.search(line)
        expected = [split_line(Counter({case["kind"]: 1})), "SUITE PASS 1 FAIL 0 SKIP 0"]
        if not kind or kind[1] != case["kind"] or lines[-3:] != [*expected, "exit 0"]:
            return "FAIL", f"FAIL {case['name']} incomplete or inconsistent PASS evidence"
    return state, line


def merge(out, plan, available):
    summaries = []
    outcomes = defaultdict(Counter)
    splits = defaultdict(Counter)
    diagnostics = []
    ctests = []
    failed = False
    for shard in plan["shards"]:
        n = shard["number"]
        root = out / f"shard-{n:02d}"
        logs = root / "logs"
        state = "MISSING"
        if n in available and (root / RESULTS).is_file():
            try:
                with tempfile.TemporaryDirectory(dir=root) as temp:
                    with tarfile.open(root / RESULTS, "r:gz") as archive:
                        archive.extractall(temp, filter="data")
                    receipt = json.loads((Path(temp) / "done.json").read_text())
                    if (receipt["id"] == plan["id"] and receipt["shard"] == n and
                            type(receipt["exit"]) is int):
                        if logs.exists():
                            shutil.rmtree(logs)
                        shutil.move(temp, logs)
                        state = "PASS" if receipt["exit"] == 0 else "FAIL"
            except (OSError, ValueError, KeyError, tarfile.TarError) as error:
                diagnostics.append(f"shard {n:02d} artifact: {error}")
        trusted = state != "MISSING"
        for batch in shard["batches"]:
            for lane in batch["lanes"]:
                for case in lane:
                    mode, name = case["mode"], case["name"]
                    log = logs / "suite" / mode / f"{name}.log"
                    result, line = read_case(log.read_text(errors="replace"), case) if trusted and log.is_file() else (None, None)
                    outcomes[mode][result or "MISSING"] += 1
                    if result == "PASS":
                        splits[mode][case["kind"]] += 1
                    else:
                        if state == "PASS":
                            state = "FAIL"
                        diagnostics.append(f"shard {n:02d} suite_{mode} {line or f'MISSING {name}'}")
        if n == 1:
            for mode in BUILDS:
                log = logs / f"ctest-b-{mode}.log"
                ctest_fn = log.read_text(errors="replace") if trusted and log.is_file() else ""
                total = CTEST_TOTAL.search(ctest_fn)
                if total and ctest_fn.endswith("\nexit 0\n") and " 0 tests failed " in total[0]:
                    ctests.append(f"ctest b-{mode}: exit 0: {total[0]}")
                    ctests.extend(l.strip() for l in ctest_fn.splitlines() if re.search(r"\*\*\*Skipped|\(Skipped\)", l))
                else:
                    if state == "PASS":
                        state = "FAIL"
                    detail = ctest_fn.splitlines()[-1] if ctest_fn else "MISSING"
                    ctests.append(f"ctest b-{mode}: {detail}: {total[0] if total else 'no CTest summary'}")
            proof = logs / "glibc-math.log"
            math = proof.read_text(errors="replace") if trusted and proof.is_file() else ""
            if math.endswith("\nexit 0\n") and "glibc_math_proof_cuda: identical" in math:
                ctests.append("glibc math proof exit 0: glibc_math_proof_cuda: identical")
            else:
                ctests.append(f"glibc math proof {math.splitlines()[-1] if math else 'MISSING'}")
                if state == "PASS":
                    state = "FAIL"
        summaries.append(f"SHARD {n:02d} {state} ({kernel_slug(plan, n)})")
        if trusted:
            setup = logs / "setup.log"
            if setup.is_file():
                diagnostics.append(f"shard {n:02d} {setup.read_text(errors='replace').strip()}")
            diagnostics.extend(f"shard {n:02d} failed log {p.relative_to(out)}"
                               for p in sorted((logs / "failed").rglob("*")) if p.is_file())
        failed |= state != "PASS"
    for mode in BUILDS:
        count = outcomes[mode]
        summaries.extend([f"suite_{mode} exit {int(bool(count['FAIL'] or count['SKIP'] or count['MISSING']))} (expected {sum(count.values())})",
                          split_line(splits[mode]), suite_line(count), f"MISSING {count['MISSING']}"])
    summaries.extend(ctests + diagnostics + [f"KAGGLE PROOF {'FAIL' if failed else 'PASS'}"])
    summary = "\n".join([f"candidate {plan['commit']}", *summaries]) + "\n"
    temporary = out / "summary.txt.tmp"
    temporary.write_text(summary)
    temporary.replace(out / "summary.txt")
    print(summary, end="")
    return int(failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", choices=("run", "collect"), default="run")
    parser.add_argument("--rev", default="HEAD")
    parser.add_argument("--cases-dir", type=Path, default=SCRATCH / "g4v15", help="G4 dp/id/sp case lists and timing data")
    parser.add_argument("--metadata-dir", type=Path, help="data-v1 TARS.txt, SHA256SUMS and INDEX.tsv[.gz]")
    parser.add_argument("--out", type=Path, required=True, help="plan, staging files, downloads and summary directory")
    parser.add_argument("--owner", help="Kaggle username (alternatively KAGGLE_USERNAME)")
    parser.add_argument("--shards", help="comma-separated one-based shard numbers")
    parser.add_argument("--plan", action="store_true", help="write and print the offline plan")
    parser.add_argument("--dry-run", action="store_true", help="write the plan and stage upload files offline")
    parser.add_argument("--no-wait", action="store_true", help="push selected shards and return")
    parser.add_argument("--local", action="store_true", help="collect existing local artifacts only")
    parser.add_argument("--disk-gib", type=float, default=38, help="max extracted reference data per batch (default 38)")
    parser.add_argument("--reserve-gib", type=float, default=12, help="free scratch reserve beyond reference data (default 12)")
    parser.add_argument("--session-hours", type=float, default=10.5, help="max estimated hours per shard (default 10.5)")
    parser.add_argument("--build-hours", type=float, default=1.10, help="estimated setup and three builds per shard (default 1.10)")
    parser.add_argument("--mbps", type=float, default=30, help="estimated HTTP transfer in decimal MB/s (default 30)")
    parser.add_argument("--dp-scale", type=float, default=7, help="scale factor for historical G4 FP64 case times (default 7)")
    args = parser.parse_args()
    out = args.out.resolve()
    if args.action == "collect":
        if args.plan or args.dry_run or args.no_wait:
            parser.error("--plan, --dry-run and --no-wait apply to run")
        plan = json.loads((out / "plan.json").read_text())
        selected = selected_shards(args.shards, plan)
        all_shards = selected_shards(None, plan)
        available = all_shards if args.local else (all_shards - selected) | collect_download(
            out, plan, owner_name(args.owner), selected, wait=False)
        return merge(out, plan, available)
    if args.local:
        parser.error("--local requires collect")
    if not all(math.isfinite(v) and v > 0 for v in (args.disk_gib, args.reserve_gib, args.session_hours,
                                                     args.build_hours, args.mbps, args.dp_scale)) or args.session_hours > 11:
        parser.error("estimate and disk parameters must be positive, with --session-hours at most 11")
    if args.metadata_dir is None:
        parser.error("--metadata-dir is required to plan or submit")
    out.mkdir(parents=True, exist_ok=True)
    commit = capture("git", "-C", str(REPO), "rev-parse", "--verify", "--end-of-options", args.rev + "^{commit}")
    with tempfile.TemporaryDirectory(dir=out) as temp:
        root = Path(temp)
        src = root / "src"
        src.mkdir()
        archive = root / "src.tgz"
        run("git", "-C", str(REPO), "archive", "--format=tar.gz", "-o", str(archive), commit)
        run("tar", "xzf", str(archive), "-C", str(src))
        release = ReleaseData(args.metadata_dir, args.cases_dir)
        shards = plan_shards(src, args, release)
        plan = {"commit": commit, "reserve": int(args.reserve_gib * GIB), "limit": 11 * 3600,
                "bundle": {"src.tgz.dat": file_digest(archive), BODY: file_digest(HERE / BODY)},
                "shards": shards, "tars": release.specs}
        identity = json.dumps(plan, sort_keys=True).encode() + Path(__file__).read_bytes()
        plan["id"] = hashlib.sha256(identity).hexdigest()
        selected = selected_shards(args.shards, plan)
        plan_path = out / "plan.json"
        if plan_path.is_file() and json.loads(plan_path.read_text())["id"] != plan["id"]:
            raise ValueError(f"{out} already contains a different plan; choose another --out")
        write_json(plan_path, plan)
        print_plan(plan, selected)
        if args.plan and not args.dry_run:
            return 0
        owner = owner_name(args.owner, offline=args.dry_run)
        dataset = prepare(plan, archive, out, owner, selected)
    if args.dry_run:
        print(f"Dry-run staging is in {out}")
        return 0
    submitted = upload(out, plan, owner, dataset, selected)
    rc = int(submitted != selected)
    if args.no_wait:
        return rc
    available = (selected_shards(None, plan) - selected) | collect_download(out, plan, owner, submitted, wait=True)
    return merge(out, plan, available) or rc


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"kaggle_proof: {error}", file=sys.stderr)
        sys.exit(2)
