#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Hashes of model outputs, for a direct comparison of two GPU builds (CUDA FP64 and HIP FP64) without their files.

Every netCDF file under an output directory (wrfout, wrfrst, auxhist, ...; any name, found by its magic bytes): one
line per variable, "FILE<TAB>VARIABLE<TAB>DTYPE<TAB>SHAPE<TAB>SHA256" with FILE relative to the suite's --out
directory, the SHA-256 of the variable's stored bytes (all frames; read as stored: no masking or scaling). Two
builds whose lines are equal wrote the same values bit for bit (-0 and +0 differ, NaN payloads too) in every variable,
the CPPWRF-only ones and the frames WRF's file does not have included; the global attributes are not hashed (the
build's name is one of them).

  out_hashes.py hash ROOT DIR...      the lines of every netCDF file under DIR (relative to ROOT) on stdout
  out_hashes.py compare REF NEW [--cases NAME,...]
      REF and NEW (files of lines, .gz too): every line of REF whose first path component is a case of NEW (or of
      --cases) must be in NEW with the same hash, NEW must have no variable REF lacks, and every case of NEW must have
      reference lines; a case named in --cases that neither file hashed is a difference. A malformed line, or two
      hashes of one variable, is an error. Prints a summary and the first differences; exit 0 when equal, 1 when
      not, 2 when nothing could be compared.

tests/verify/wrf_suite.py --hash-out FILE appends the lines of every passing check's outputs before it deletes them.
"""
import gzip
import hashlib
import os
import sys

MAGIC = (b"CDF\x01", b"CDF\x02", b"CDF\x05", b"\x89HDF\r\n\x1a\n")


def is_netcdf(path):
    try:
        with open(path, "rb") as f:
            head = f.read(8)
    except OSError:
        return False
    return any(head.startswith(m) for m in MAGIC)


def hash_file(path, rel):
    import netCDF4
    import numpy as np
    out = []
    with netCDF4.Dataset(path) as ds:
        ds.set_auto_maskandscale(False)
        ds.set_auto_chartostring(False)
        for name in sorted(ds.variables):
            v = ds.variables[name]
            x = np.ascontiguousarray(v[...])
            h = hashlib.sha256()
            h.update(x.dtype.str.encode())
            h.update(repr(x.shape).encode())
            h.update(x.tobytes())
            out.append(f"{rel}\t{name}\t{x.dtype.str}\t{'x'.join(map(str, x.shape))}\t{h.hexdigest()}")
    return out


def hash_tree(root, d):
    lines = []
    for base, dirs, files in os.walk(d):
        dirs.sort()
        for f in sorted(files):
            p = os.path.join(base, f)
            if not os.path.islink(p) and is_netcdf(p):
                lines += hash_file(p, os.path.relpath(p, root))
    return lines


def read(path):
    """{(file, variable): (type, shape, hash)}. A line without its five fields (a truncated or corrupted file) or
    one key with two different hashes (appended runs that disagree) is an error, not a line to drop."""
    op = gzip.open if path.endswith(".gz") else open
    out = {}
    with op(path, "rt") as f:
        for i, ln in enumerate(f, 1):
            if not ln.strip():
                continue
            p = ln.rstrip("\n").split("\t")
            if len(p) != 5 or not all(p):
                raise ValueError(f"{path}:{i}: malformed hash line {ln.rstrip()!r}")
            k, v = (p[0], p[1]), (p[2], p[3], p[4])
            if out.get(k, v) != v:
                raise ValueError(f"{path}:{i}: {p[0]} {p[1]} hashed twice with different values")
            out[k] = v
    return out


def compare(ref, new, cases=None):
    r, n = read(ref), read(new)
    case = (lambda f: f.split(os.sep, 1)[0])
    want = set(cases) if cases else {case(f) for f, _ in n}
    rr = {k: v for k, v in r.items() if case(k[0]) in want}
    nn = {k: v for k, v in n.items() if case(k[0]) in want}
    if not rr:
        print(f"HASHES none of the {len(want)} cases is in {ref}")
        return 2
    rc = {case(k[0]) for k in rr}
    nc = {case(k[0]) for k in nn}
    absent = sorted(want - rc - nc) if cases else []  # a requested case neither file hashed (skipped, misspelt)
    diff = sorted(k for k in rr if k in nn and rr[k] != nn[k])
    miss = sorted(k for k in rr if k not in nn)
    extra = sorted(k for k in nn if k not in rr and case(k[0]) in rc)
    only_new = sorted(nc - rc)
    for c in absent:
        print(f"ABSENT  {c} (requested, hashed in neither file)")
    for k in diff[:20]:
        print(f"DIFFER  {k[0]} {k[1]}")
    for k in miss[:20]:
        print(f"MISSING {k[0]} {k[1]} (in the reference, not written)")
    for k in extra[:20]:
        print(f"EXTRA   {k[0]} {k[1]} (not in the reference)")
    print(f"HASHES {len(rc & nc)} cases, {len(rr)} variables compared: {len(diff)} differ, {len(miss)} missing, "
          f"{len(extra)} extra; {len(only_new)} cases without reference hashes"
          + (f" ({', '.join(only_new[:10])}{' ...' if len(only_new) > 10 else ''})" if only_new else ""))
    return 0 if not (diff or miss or extra or only_new or absent) else 1


def main():
    a = sys.argv[1:]
    if a[:1] == ["hash"] and len(a) >= 3:
        for d in a[2:]:
            for ln in hash_tree(a[1], d):
                print(ln)
    elif a[:1] == ["compare"] and len(a) in (3, 5):
        cases = a[4].split(",") if len(a) == 5 and a[3] == "--cases" else None
        try:
            sys.exit(compare(a[1], a[2], cases))
        except ValueError as e:
            sys.exit(f"HASHES {e}")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
