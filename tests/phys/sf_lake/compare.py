#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Byte comparison of the lake column test outputs (driver_lake.f90 against driver_lake.cpp): every value identical
(NaN only equal to NaN, -0 differs from +0). Prints the first differing values."""
import struct
import sys

a = open(sys.argv[1], "rb").read()
b = open(sys.argv[2], "rb").read()
if len(a) != len(b):
    print(f"FAIL: sizes differ ({len(a)} / {len(b)} bytes)")
    sys.exit(1)
n = len(a) // 8
bad = [i for i in range(n) if a[8 * i:8 * i + 8] != b[8 * i:8 * i + 8]]
if bad:
    for i in bad[:20]:
        x, = struct.unpack("<d", a[8 * i:8 * i + 8])
        y, = struct.unpack("<d", b[8 * i:8 * i + 8])
        print(f"  word {i}: WRF {x!r} C++ {y!r}")
    print(f"FAIL: {len(bad)} of {n} values differ")
    sys.exit(1)
print(f"PASS: {n} values identical")
