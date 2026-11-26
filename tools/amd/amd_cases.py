#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The GPU suite checks of tools/amd/amd_run.sh and the data they read.

  amd_cases.py list KEY [--slow]
      the names of wrf_suite.py's checks of a GPU binary, one per line: KEY cuda_dp (FP64, --hip-dp: the cuda_dp
      cases and the restart checks restart_<tag>_fp64_cuda), cuda_sp_id (FP32 identity, --hip-sp-id) or cuda (fast
      FP32, --hip: the cuda cases and restart_p1_fp32_cuda); the slow cases with --slow
  amd_cases.py packs INDEX NEEDS_DIR NAME...
      the packs of the data release in the packs layout (cppwrf-data's data-v2: INDEX.tsv, every file in one pack)
      that hold every file the checks NAME read (cppwrf-data's ci/needs.py, from NEEDS_DIR), comma-joined, largest
      first; exit 1 when a check's files are not in the index
  amd_cases.py members TAR INDEX NEEDS_DIR NAME...
      the files the checks NAME read from the data tar TAR (wrf_suite.py's DATA) as the packs layout holds them: one
      line "PACK<TAB>FILE" per file (the data-v1 release does not carry v05_files_data.tar itself: amd_run.sh takes
      its files from the data-v2 packs, as the CUDA proofs do)
"""
import os
import sys
from collections import defaultdict

SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(SRC, "tests", "verify"))


def names(key, slow):
    import wrf_suite as w
    out = [c[0] for c in w.CASES if c[1] == key and (slow or not w.slow(c[3]))]
    if key == "cuda_dp":
        out += [f"restart_{tag}_fp64_cuda" for tag, _, _ in w.RESTARTS]
    elif key == "cuda":
        out.append("restart_p1_fp32_cuda")
    return out


def packs(index, needs_dir, checks):
    sys.path.insert(0, needs_dir)
    import needs
    idx = needs.Index(index)
    size = defaultdict(int)
    for name, (_, _, pack, sz) in idx.ent.items():
        size[pack] += sz
    want, bad = set(), []
    for n in checks:
        e = needs.case_entries(SRC, n, idx)
        if not e:
            bad.append(n)
        want |= {idx.ent[x][2] for x in e}
    if bad:
        sys.exit(f"amd_cases.py: no data for {', '.join(bad)} in {index}")
    return sorted(want, key=lambda p: (-size[p], p))


def members(tar, index, needs_dir, checks):
    import wrf_suite as w
    sys.path.insert(0, needs_dir)
    import needs
    idx = needs.Index(index)
    stem = tar[:-4] + "." if tar.endswith(".tar") else tar + "."
    out, bad = set(), []
    for n in checks:
        if tar not in (w.DATA.get(n) or "").split(","):
            continue
        e = needs.case_entries(SRC, n, idx)
        if not e:
            bad.append(n)
        out |= {(idx.ent[x][2], x) for x in e if idx.ent[x][2].startswith(stem) and idx.ent[x][0] != "5"}
    if bad:
        sys.exit(f"amd_cases.py: no data for {', '.join(bad)} in {index}")
    return sorted(out)


def main():
    a = sys.argv[1:]
    if a[:1] == ["list"] and len(a) >= 2:
        print("\n".join(names(a[1], "--slow" in a[2:])))
    elif a[:1] == ["packs"] and len(a) >= 4:
        print(",".join(packs(a[1], a[2], a[3:])))
    elif a[:1] == ["members"] and len(a) >= 5:
        for p, f in members(a[1], a[2], a[3], a[4:]):
            print(f"{p}\t{f}")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
