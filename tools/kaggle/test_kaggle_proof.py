#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline staging, streaming and result-collection fixtures for kaggle_proof.sh."""

from collections import Counter
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import kaggle_proof as proof
import kg_proof_body as body


class ProofFixture(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.out = self.root / "out"

    def archive_bytes(self):
        stream = io.BytesIO()
        with tarfile.open(fileobj=stream, mode="w") as archive:
            for name, payload in (("./wrf_runs/reference", b"checked data"), ("unused", b"other data")):
                member = tarfile.TarInfo(name)
                member.size = len(payload)
                archive.addfile(member, io.BytesIO(payload))
        return stream.getvalue()

    def test_checked_split_archive_and_filtered_overlay(self):
        assets = self.root / "assets"
        assets.mkdir()
        raw = self.archive_bytes()
        parts = [raw[:731], raw[731:]]
        spec = {"name": proof.OVERLAY, "bytes": len(raw), "exact": True, "sha256": hashlib.sha256(raw).hexdigest(),
                "assets": [], "members": ["wrf_runs/reference"], "disk": len(raw)}
        for i, payload in enumerate(parts):
            name = f"{proof.OVERLAY}.part{i:03d}"
            (assets / name).write_bytes(payload)
            spec["assets"].append([name, hashlib.sha256(payload).hexdigest()])
        job = {"data_url": assets.as_uri(), "asset_limit": proof.ASSET_LIMIT}
        dest = self.root / "data"
        dest.mkdir()
        body.fetch_tar(job, spec, self.root, dest, self.root / "data.log")
        self.assertEqual((dest / "wrf_runs/reference").read_bytes(), b"checked data")
        self.assertFalse((dest / "unused").exists())
        self.assertFalse((self.root / "current.part").exists())
        body.fetch_tar(job, dict(spec, members=[]), self.root, dest, self.root / "data.log")
        self.assertEqual((dest / "unused").read_bytes(), b"other data")
        with self.assertRaisesRegex(RuntimeError, "aggregate size or SHA256"):
            body.fetch_tar(job, dict(spec, sha256="a" * 64), self.root, dest, self.root / "data.log")
        (assets / spec["assets"][1][0]).write_bytes(b"wrong checksum")
        with patch.object(body.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "DATA_FAILED.*SHA256"):
                body.fetch_tar(job, spec, self.root, dest, self.root / "data.log")

    def launcher(self, *args):
        fake = self.root / "bin"
        fake.mkdir(exist_ok=True)
        trap = fake / "kaggle"
        trap.write_text(f"#!/bin/sh\ntouch '{fake / 'called'}'\nexit 99\n")
        trap.chmod(0o755)
        env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}")
        result = subprocess.run([str(proof.HERE / "kaggle_proof.sh"), *args, "--out", str(self.out)],
                                text=True, capture_output=True, env=env)
        self.assertFalse((fake / "called").exists())
        return result

    def dry_run(self):
        case_dir = self.root / "cases"
        case_dir.mkdir()
        cases = {"dp": "real_p1t2_r8_fp64_cuda_1h", "id": "real_p1_fp32_cuda_id", "sp": "real_r0_fp32_1h"}
        for mode, name in cases.items():
            (case_dir / f"{mode}_cases.txt").write_text(name + "\n")
        names = {"p1t2_data.tar", "v05_suite_refs.tar", "fp32id_data.tar", "suite_data.tar", "ideal_data.tar",
                 "port_b03_data.tar", "port_gem10_data.tar", "port_gem13_data.tar", "port_v111_data.tar", "real_exe_data.tar"}
        sizes = {name: 10240 for name in names}
        sizes.update({"v05_suite_refs.tar": 10 * 2**20, "fp32id_data.tar": 10 * 2**20})
        digest = "a" * 64
        (self.root / "TARS.txt").write_text("".join(f"{name} {sizes[name]} {digest} 1\n" for name in sorted(names)))
        (self.root / "SHA256SUMS").write_text("".join(f"{digest}  {name}.part000\n" for name in sorted(names)))
        (self.root / "INDEX.tsv").write_text("")
        result = self.launcher("--rev", "main", "--dry-run", "--shards", "1", "--owner", "fixture", "--cases-dir",
                               str(case_dir), "--metadata-dir", str(self.root), "--disk-gib", "0.015")
        self.assertEqual(result.returncode, 0, result.stderr)
        plan = json.loads((self.out / "plan.json").read_text())
        metadata = json.loads((self.out / "kernel-01/kernel-metadata.json").read_text())
        self.assertEqual(metadata["machine_shape"], "NvidiaTeslaT4")
        self.assertTrue(metadata["is_private"] and metadata["enable_internet"])
        self.assertIn(plan["commit"][:10], metadata["id"])
        self.assertEqual(len(plan["shards"]), 2)
        self.assertFalse((self.out / "kernel-02/run.py").exists())
        self.assertEqual(plan["bundle"]["src.tgz.dat"], proof.file_digest(self.out / "dataset/src.tgz.dat"))
        program = self.out / "kernel-01/run.py"
        compile(program.read_text(), str(program), "exec")
        return plan

    def artifact(self, plan, number, outcome="PASS", *, stale=False):
        work = self.root / f"logs-{number}"
        work.mkdir(exist_ok=True)
        shard = plan["shards"][number - 1]
        for batch in shard["batches"]:
            for lane in batch["lanes"]:
                for case in lane:
                    mode, name, kind = (case[k] for k in ("mode", "name", "kind"))
                    logfile = work / "suite" / mode / f"{name}.log"
                    logfile.parent.mkdir(parents=True, exist_ok=True)
                    if mode == "sp" and outcome == "FAIL":
                        logfile.write_text(f"FAIL  {name}  comparator failure\n\nexit 1\n")
                        failed = work / "failed" / mode / name / "run.failed.log"
                        failed.parent.mkdir(parents=True, exist_ok=True)
                        failed.write_text("failed diagnostics\n")
                    else:
                        logfile.write_text(f"PASS  {name}  compared  [{kind}]  (1 s)\n"
                                           f"{proof.split_line(Counter({kind: 1}))}\n"
                                           "SUITE PASS 1 FAIL 0 SKIP 0\n\nexit 0\n")
        if number == 1:
            for mode in proof.BUILDS:
                (work / f"ctest-b-{mode}.log").write_text("100% tests passed, 0 tests failed out of 20\n\nexit 0\n")
            (work / "glibc-math.log").write_text("glibc_math_proof_cuda: identical\n\nexit 0\n")
        proof.write_json(work / "done.json", {"id": "old" if stale else plan["id"], "shard": number,
                                             "exit": int(outcome != "PASS")})
        result = self.out / f"shard-{number:02d}" / proof.RESULTS
        result.parent.mkdir(exist_ok=True)
        with tarfile.open(result, "w:gz") as archive:
            for path in work.iterdir():
                archive.add(path, arcname=path.name)

    def test_dry_run_and_collector(self):
        plan = self.dry_run()
        result = self.launcher("collect", "--local")
        self.assertEqual(result.returncode, 1)
        self.assertIn("SHARD 01 MISSING", result.stdout)
        self.assertIn("SUITE PASS 0 FAIL 0 SKIP 0\nMISSING 1", result.stdout)
        self.artifact(plan, 1, stale=True)
        result = self.launcher("collect", "--local")
        self.assertEqual(result.returncode, 1)
        self.assertIn("SHARD 01 MISSING", result.stdout)
        self.artifact(plan, 1)
        (self.out / "status-02.log").write_text('fixture/kernel/1 has status "KernelWorkerStatus.RUNNING"\n')
        result = self.launcher("collect", "--local")
        self.assertEqual(result.returncode, 1)
        self.assertIn("SHARD 01 PASS", result.stdout)
        self.assertIn("SHARD 02 MISSING", result.stdout)
        self.assertIn('shard 02 Kaggle status: fixture/kernel/1 has status "KernelWorkerStatus.RUNNING"', result.stdout)
        self.artifact(plan, 2)
        result = self.launcher("collect", "--local")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("KAGGLE PROOF PASS", result.stdout)
        self.assertIn("vs WRF by relative L2: 1", result.stdout)
        fast_shard = next(s["number"] for s in plan["shards"]
                          if any(c["mode"] == "sp" for b in s["batches"] for lane in b["lanes"] for c in lane))
        self.artifact(plan, fast_shard, "FAIL")
        result = self.launcher("collect", "--local")
        self.assertEqual(result.returncode, 1)
        self.assertIn("SUITE PASS 0 FAIL 1 SKIP 0", result.stdout)
        self.assertIn(f"failed log shard-{fast_shard:02d}/logs/failed/sp/real_r0_fp32_1h/run.failed.log", result.stdout)

    def test_incomplete_pass_evidence(self):
        case = {"name": "test", "kind": proof.KINDS[0]}
        result, _ = proof.read_case("PASS test [vs WRF tol 0]  (1 s)\nexit 0\n", case)
        self.assertEqual(result, "FAIL")
        result, _ = proof.read_case("PASS different_case [vs WRF tol 0]  (1 s)\nexit 0\n", case)
        self.assertIsNone(result)

    def test_worker_dispatches_two_gpus_and_cleans_scratch(self):
        self.out.mkdir()
        scratch = self.root / "scratch"
        scratch.mkdir()
        commit = "0" * 40
        suite = '''\
import os
from pathlib import Path
import sys
import time

gpu = os.environ["CUDA_VISIBLE_DEVICES"]
name = sys.argv[sys.argv.index("--only") + 1]
assert gpu == f"GPU-{name}"
assert (Path(os.environ["CPPWRF_WRF_RUNS"]) / "reference").read_bytes() == b"checked data"
work = Path(sys.argv[sys.argv.index("--out") + 1]).parent
(work / gpu).touch()
for _ in range(100):
    if (work / f"GPU-{1-int(name)}").is_file():
        break
    time.sleep(0.02)
else:
    sys.exit("other worker never began")
print(f"worker {name} got {gpu}")
'''
        with tarfile.open(self.root / "src.tgz.dat", "w:gz") as archive:
            for name, content in (("src/cases/commit.txt", commit), ("tests/verify/wrf_suite.py", suite)):
                payload = content.encode()
                item = tarfile.TarInfo(name)
                item.size = len(payload)
                archive.addfile(item, io.BytesIO(payload))
        data = self.archive_bytes()
        (self.root / "suite_data.tar").write_bytes(data)
        digest = hashlib.sha256(data).hexdigest()
        spec = {"name": "suite_data.tar", "bytes": len(data), "exact": True, "sha256": digest, "members": [],
                "links": None, "assets": [["suite_data.tar", digest]]}
        cases = [[{"name": str(i), "mode": "dp"}] for i in range(2)]
        batch = {"tars": ["suite_data.tar"], "lanes": cases, "ctest": False, "disk": len(data)}
        job = {"commit": commit, "shard": {"number": 2, "batches": [batch], "disk": len(data)}, "reserve": 0,
               "asset_limit": len(data), "data_url": self.root.as_uri(), "tars": {"suite_data.tar": spec}}
        job_path = self.root / "shard-02.json"
        proof.write_json(job_path, job)
        with (patch.object(body, "measured_scratch", return_value=scratch),
              patch.object(body, "get_gpus", return_value=["GPU-0", "GPU-1"]),
              patch.object(body, "setup_packages"), patch.object(body, "build_all") as builds):
            self.assertEqual(body.main(job_path, self.out), 0)
        builds.assert_called_once()
        self.assertEqual(list(scratch.iterdir()), [])
        for i in range(2):
            self.assertIn(f"worker {i} got GPU-{i}", (self.out / "suite" / "dp" / f"{i}.log").read_text())


if __name__ == "__main__":
    unittest.main()
