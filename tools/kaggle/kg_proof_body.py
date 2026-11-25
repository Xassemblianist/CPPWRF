#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Kaggle worker for one CPPWRF proof shard. The kernel wrapper publishes its exit receipt."""

from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time

BUILDS = {"dp": ("ON", "OFF", "--cuda-dp"), "id": ("OFF", "ON", "--cuda-sp-id"),
          "sp": ("OFF", "OFF", "--cuda")}
GIB = 2**30
CHUNK = 4 * 2**20


def log_command(argv, log, *, env=None, cwd=None, append=False):
    with log.open("ab" if append else "wb") as output:
        rc = subprocess.call(argv, stdout=output, stderr=subprocess.STDOUT, env=env, cwd=cwd)
        output.write(f"\nexit {rc}\n".encode())
    return rc


def measured_scratch(out, required):
    paths = list(map(Path, ("/", "/tmp", "/var/tmp", "/root", "/kaggle/working")))
    with (out / "disk.txt").open("w") as log:
        for path in paths:
            if path.exists():
                usage = shutil.disk_usage(path)
                print(f"{path}  dev {path.stat().st_dev}  total {usage.total}  free {usage.free}", file=log)
    choices = [p for p in paths[1:-1] if p.is_dir() and os.access(p, os.W_OK | os.X_OK)]
    if not choices:
        raise RuntimeError("no writable runtime scratch location")
    scratch = max(choices, key=lambda path: shutil.disk_usage(path).free)
    if shutil.disk_usage(scratch).free < required:
        raise RuntimeError(f"{scratch} needs {required / GIB:.2f} GiB free before setup (see disk.txt)")
    return scratch


def get_gpus(out):
    query = "index,name,uuid,memory.total,compute_cap,driver_version"
    gpus = subprocess.check_output(["nvidia-smi", f"--query-gpu={query}", "--format=csv,noheader"], text=True)
    with (out / "env.txt").open("w") as log:
        log.write(gpus)
        print(f"CPUs available: {len(os.sched_getaffinity(0))}", file=log, flush=True)
        for command in (["nvcc", "--version"], ["mpirun", "--version"], ["cmake", "--version"], ["gcc", "--version"]):
            if shutil.which(command[0]):
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    rows = list(csv.reader(gpus.splitlines(), skipinitialspace=True))
    if len(rows) != 2 or len({row[2] for row in rows}) != 2 or any(row[1] != "Tesla T4" or row[4] != "7.5" for row in rows):
        raise RuntimeError("expected exactly two Tesla T4 GPUs with compute capability 7.5 (see env.txt)")
    return [row[2] for row in rows]


def setup_packages(out):
    env = dict(os.environ, DEBIAN_FRONTEND="noninteractive")
    if not shutil.which("nc-config"):
        packages = (["apt-get", "-qq", "update"], ["apt-get", "-qq", "install", "-y", "libnetcdf-dev", "netcdf-bin"])
        for i, cmd in enumerate(packages):
            if log_command(cmd, out / "apt.log", env=env, append=i > 0):
                raise RuntimeError("NetCDF-C setup failed (see apt.log)")
    if subprocess.call([sys.executable, "-c", "import netCDF4, numpy"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL):
        if log_command([sys.executable, "-m", "pip", "install", "-q", "--no-cache-dir", "netCDF4", "numpy"], out / "pip.log"):
            raise RuntimeError("NetCDF Python setup failed (see pip.log)")


def fetch_part(url, asset, digest, path, ceiling):
    for attempt in range(3):
        rc = subprocess.call(["curl", "-fsSL", "--retry", "2", "--retry-delay", "2", "--connect-timeout", "20",
                              "--max-time", "600", "--max-filesize", str(ceiling), "-o", str(path), f"{url}/{asset}"])
        if rc == 0 and 0 < path.stat().st_size <= ceiling:
            with path.open("rb") as stream:
                if hashlib.file_digest(stream, "sha256").hexdigest() == digest:
                    return
        path.unlink(missing_ok=True)
        if attempt < 2:
            print(f"retrying download or SHA256 for {asset}", flush=True)
            time.sleep(8)
    raise RuntimeError(f"DATA_FAILED {asset} (download or SHA256)")


def extract_overlay(stream, dest, members, ceiling):
    remaining = set(members)
    size = 0
    with tarfile.open(fileobj=stream, mode="r|", ignore_zeros=True) as archive:
        for member in archive:
            name = member.name.removeprefix("./")
            if name not in remaining:
                continue
            target = dest / name
            if (not member.isfile() or target.is_symlink() or not target.resolve().is_relative_to(dest.resolve())
                    or Path(name).is_absolute() or ".." in Path(name).parts):
                raise RuntimeError(f"invalid selected reference {name}")
            size += member.size
            if size > ceiling:
                raise RuntimeError("selected overlay exceeds its planned disk budget")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.unlink(missing_ok=True)
            with archive.extractfile(member) as source, target.open("wb") as output:
                shutil.copyfileobj(source, output, CHUNK)
            remaining.remove(name)
    if remaining:
        raise RuntimeError(f"overlay files missing from archive: {', '.join(sorted(remaining))}")


def overlay_cli(dest, member_file, ceiling):
    extract_overlay(sys.stdin.buffer, Path(dest), Path(member_file).read_text().splitlines(), int(ceiling))


def fetch_tar(job, spec, scratch, dest, log, *, links=False):
    url = job["data_url"]
    assets = [spec["links"]] if links else spec["assets"]
    size = 0
    maximum = job["asset_limit"] if links else spec["bytes"]
    total_digest = hashlib.sha256()
    members = spec["members"] if not links else []
    path = scratch / "current.part"
    start = time.monotonic()
    if members:
        member_file = scratch / "members.txt"
        member_file.write_text("\n".join(members) + "\n")
        cmd = [sys.executable, __file__, "overlay", str(dest), str(member_file), str(spec["disk"])]
    else:
        cmd = ["tar", "--no-same-owner", "--no-same-permissions", "--ignore-zeros", "-xf", "-", "-C", str(dest)]
    with subprocess.Popen(cmd, stdin=subprocess.PIPE) as tar:
        try:
            for asset, digest in assets:
                ceiling = min(job["asset_limit"], maximum - size)
                if not links and not spec["exact"]:
                    ceiling = min(ceiling, maximum // len(assets))
                if ceiling <= 0:
                    raise RuntimeError(f"DATA_FAILED {asset} (archive exceeds planned size)")
                fetch_part(url, asset, digest, path, ceiling)
                size += path.stat().st_size
                with path.open("rb") as stream:
                    while chunk := stream.read(CHUNK):
                        total_digest.update(chunk)
                        tar.stdin.write(chunk)
                path.unlink()
            tar.stdin.close()
            if tar.wait():
                raise RuntimeError(f"DATA_FAILED {spec['name']} (unpack)")
        except Exception:
            tar.kill()
            raise
        finally:
            path.unlink(missing_ok=True)
    if not links and spec["exact"] and (size != spec["bytes"] or total_digest.hexdigest() != spec["sha256"]):
        raise RuntimeError(f"DATA_FAILED {spec['name']} (aggregate size or SHA256)")
    name = assets[0][0] if links else spec["name"]
    with log.open("a") as f:
        print(f"{'LINKS' if links else 'TAR'} {name} {size} bytes {time.monotonic() - start:.0f} s", file=f)


def build_all(src, work, out, cpus, ctest):
    for mode, (dp, identity, _) in BUILDS.items():
        build = work / f"b-{mode}"
        log = out / f"build-b-{mode}.log"
        cmake = ["cmake", "-S", str(src), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release", "-DCPPWRF_ENABLE_CUDA=ON",
                 "-DCMAKE_CUDA_ARCHITECTURES=75", f"-DCPPWRF_DOUBLE={dp}", f"-DCPPWRF_CUDA_FP32_IDENTITY={identity}"]
        cmd = ["cmake", "--build", str(build), "--parallel", str(cpus), "--target", "all" if ctest else "cppwrf"]
        if log_command(cmake, log) or log_command(cmd, log, append=True) or not (build / "cppwrf").is_file():
            raise RuntimeError(f"BUILD FAILED b-{mode} (see {log.name})")


def run_ctest(src, work, out, env, gpu):
    env = dict(env, CUDA_VISIBLE_DEVICES=gpu, CPPWRF_SCRATCH=str(work / "ctest-scratch"))
    (work / "ctest-scratch").mkdir()
    bad = False
    for mode in BUILDS:
        log = out / f"ctest-b-{mode}.log"
        rc = log_command(["ctest", "--test-dir", str(work / f"b-{mode}"), "--output-on-failure", "--no-tests=error"], log, env=env)
        bad |= rc != 0
    proof = work / "glibc_math_proof_cuda"
    log = out / "glibc-math.log"
    cmd = ["nvcc", "-std=c++20", "-O3", "--fmad=false", "-arch=sm_75", "-DCPPWRF_GLIBC_MATH", "-Isrc", "-Itools",
           "-Xcompiler", "-fopenmp,-ffp-contract=off", "tools/glibc_math_proof_cuda.cu", "-o", str(proof)]
    rc = log_command(cmd, log, cwd=src, env=env) or log_command([str(proof), "10000000"], log, env=env, append=True)
    return bad or rc != 0


def save_failed(run_dir, dest):
    # Preserve diagnostic text, with a per-case limit so failed runs cannot fill /kaggle/working.
    remaining = 4 * 2**20
    for path in sorted(run_dir.rglob("*")):
        if remaining == 0:
            break
        if path.is_symlink() or path.suffix not in {".log", ".txt", ".json", ".csv"} or not path.is_file():
            continue
        target = dest / path.relative_to(run_dir)
        target.parent.mkdir(parents=True, exist_ok=True)
        count = min(remaining, path.stat().st_size, 2**20)
        with path.open("rb") as source:
            source.seek(-count, os.SEEK_END)
            target.write_bytes(source.read(count))
        remaining -= count


def run_lane(cases, gpu, src, work, out, env):
    env = dict(env, CUDA_VISIBLE_DEVICES=gpu)
    bad = False
    for case in cases:
        name, mode = case["name"], case["mode"]
        log = out / "suite" / mode / f"{name}.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        build = work / f"b-{mode}"
        with tempfile.TemporaryDirectory(prefix="case-", dir=work) as temp:
            cmd = [sys.executable, "-B", str(src / "tests/verify/wrf_suite.py"), BUILDS[mode][2], str(build / "cppwrf"),
                   "--exact", "--only", name, "--out", temp]
            print(f"suite_{mode} {name} {gpu}", flush=True)
            rc = log_command(cmd, log, env=env, cwd=src)
            if rc:
                save_failed(Path(temp), out / "failed" / mode / name)
                bad = True
    return bad


def main(job_path, out):
    job = json.loads(job_path.read_text())
    shard = job["shard"]
    scratch = measured_scratch(out, shard["disk"] + job["reserve"])
    gpus = get_gpus(out)
    setup_packages(out)
    cpus = len(os.sched_getaffinity(0))
    with tempfile.TemporaryDirectory(prefix="cppwrf-proof-", dir=scratch) as temp:
        work = Path(temp)
        src = work / "src"
        src.mkdir()
        subprocess.run(["tar", "xzf", str(job_path.parent / "src.tgz.dat"), "-C", str(src)], check=True)
        if (src / "src/cases/commit.txt").read_text().strip() != job["commit"]:
            raise RuntimeError("candidate commit does not match the git archive")
        print(f"building {job['commit']} on {scratch}", flush=True)
        build_all(src, work, out, cpus, shard["number"] == 1)
        bad = False
        for i, batch in enumerate(shard["batches"], 1):
            free = shutil.disk_usage(work).free
            with (out / "disk.txt").open("a") as log:
                print(f"batch {i}  {work}  free {free}  data {batch['disk']}  reserve {job['reserve']}", file=log)
            if free < batch["disk"] + job["reserve"]:
                raise RuntimeError(f"insufficient scratch for batch {i} (see disk.txt)")
            with tempfile.TemporaryDirectory(prefix="data-", dir=work) as temp_data:
                dest = Path(temp_data)
                print(f"batch {i}: staging {', '.join(batch['tars'])}", flush=True)
                specs = [job["tars"][name] for name in batch["tars"]]
                for spec in specs:
                    fetch_tar(job, spec, work, dest, out / "data.txt")
                for spec in specs:
                    if spec["links"]:
                        fetch_tar(job, spec, work, dest, out / "data.txt", links=True)
                env = dict(os.environ, CPPWRF_WRF_RUNS=str(dest / "wrf_runs"), CPPWRF_WRF_REF=str(dest / "wrf_ref"),
                           OMP_NUM_THREADS=str(max(cpus // 2, 1)), OMP_WAIT_POLICY="passive")
                if batch["ctest"]:
                    bad |= run_ctest(src, work, out, env, gpus[0])
                else:
                    with ThreadPoolExecutor(max_workers=2) as pool:
                        tasks = [pool.submit(run_lane, lane, gpu, src, work, out, env) for lane, gpu in zip(batch["lanes"], gpus)]
                        for task in tasks:
                            bad |= task.result()
            # No later batch refers to this directory. Free all its extracted tars before fetching the next batch.
    return int(bad)


if __name__ == "__main__":
    if sys.argv[1] == "overlay":
        overlay_cli(*sys.argv[2:])
    else:
        try:
            sys.exit(main(Path(sys.argv[1]), Path(sys.argv[2])))
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
            message = f"PROOF FAILED: {error}"
            print(message, file=sys.stderr)
            (Path(sys.argv[2]) / "setup.log").write_text(message + "\n")
            sys.exit(1)
