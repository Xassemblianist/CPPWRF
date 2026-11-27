#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare a cppwrf_real output file with WRF real.exe's: every variable's values (tolerance 0, as
tests/verify/exact.py: NaN equals only NaN, -0 differs from +0, values as stored),
the variable list and order, dimensions, types, variable attributes and global attributes.
TITLE is ignored (it names the program). Exit status 1 on any difference.
Also used for wrfout/wrfrst files (tests/verify/run_wrf_files_test.sh); --ignore VAR,... skips the
values of the listed variables; --no-values compares the text variables (Times) and the shapes but not the numbers
(a GPU run's files, whose values differ from WRF's at roundoff).

  compare_real.py [--ignore VAR,...] [--no-values] cppwrf_file wrf_file"""
import os
import sys

import numpy as np
from netCDF4 import Dataset

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "verify"))
import exact  # noqa: E402


def values(a, b, skip=(), numbers=True):
    bad = 0
    payload = False
    for n, v in a.variables.items():
        if n not in b.variables or n in skip:
            continue
        w = b.variables[n]
        if v.dtype.kind in 'SU' or n == 'Times' or n.startswith('md___'):
            dif = exact.describe(np.asarray(v[:]), np.asarray(w[:]))  # stored bytes and shape (NUL bytes too)
            if dif:
                print('%-14s text %s' % (n, dif))
                bad += 1
            continue
        x = np.asarray(v[:])
        y = np.asarray(w[:])
        if x.shape != y.shape:
            print('%-14s shape %s vs %s' % (n, x.shape, y.shape))
            bad += 1
            continue
        payload |= n not in exact.CLOCK_VARS and x.dtype.kind in "fiub" and x.size > 0
        if not numbers:
            continue
        dif = exact.describe(x, y)
        if dif:
            bad += 1
            print('%-14s %s' % (n, dif))
    if not payload:
        print("no model fields with compared values or shapes")
        bad += 1
    return bad


def metadata(a, b, ignore):
    """exact.metadata: variable list and order, dimensions (unlimited ones with their length), each variable's
    dimensions, type and attributes (type and every element), the global attributes (type and every element) and
    their order."""
    out = exact.metadata(a, b, tuple(ignore))
    for x in out:
        print(x)
    return len(out)


def main():
    args = sys.argv[1:]
    skip = set()
    numbers = '--no-values' not in args
    args = [x for x in args if x != '--no-values']
    if len(args) == 4 and args[0] == '--ignore':
        skip = set(args[1].split(','))
        args = args[2:]
    if len(args) != 2:
        sys.exit('usage: compare_real.py [--ignore VAR,...] [--no-values] cppwrf_file wrf_file')
    with Dataset(args[0]) as a, Dataset(args[1]) as b:
        for ds in (a, b):
            if "Time" not in ds.dimensions or not len(ds.dimensions["Time"]):
                sys.exit(f"{ds.filepath()}: zero Time records")
        nv = values(exact.raw(a), exact.raw(b), skip, numbers)
        nm = metadata(a, b, {'TITLE'})
    print('%s: %d variables differ, %d metadata differences' % (args[1], nv, nm))
    sys.exit(1 if nv or nm else 0)


if __name__ == '__main__':
    main()
