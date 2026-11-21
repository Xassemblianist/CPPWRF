#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Double rounding of decimal constants in CPPWRF's FP32 builds.

WRF real*4 (gfortran) rounds a REAL literal without a kind suffix, and a value READ into a REAL, once: decimal to
binary32. CPPWRF writes most constants as a double literal cast to float in FP32 builds (real(c), rb(c), Real(c),
static_cast<float>(c), real{c}, the initialisers of real / float arrays and constexpr tables): decimal to binary64,
then binary64 to binary32. For a decimal whose nearest double is exactly halfway between two floats (and which is
not that double itself) the two roundings give neighbouring floats (1 ulp).

The checker reports every such literal:
  * src mode (default): every decimal floating literal of the C++ / CUDA sources outside comments and strings,
    whatever its context (a superset of the literals that reach a float; a literal written with an f suffix rounds
    once and is fine; one with an L suffix is checked through x87's 64-bit significand). A hit on a line marked
    with the comment `fp32-double-rounding: double` (a literal that never reaches a float) is listed but passes.
  * data mode (--data PATH ...): every decimal number of the text files (tables read with strtod and then cast
    to float, namelists, WRF run directory tables; Fortran D / Q exponents and r*c repeats included).
Exit status 1 when a hit fails, 0 otherwise. Python 3 standard library only.
"""

import argparse
import math
import os
import re
import struct
import sys
from decimal import Decimal, getcontext
from fractions import Fraction

getcontext().prec = 1200  # exact for every double / float (at most 767 significant digits)

SRC_EXT = {".cpp", ".hpp", ".cu", ".cuh", ".h", ".cc", ".cxx", ".inl"}
MARK = "fp32-double-rounding: double"
# decimal floating literal, optional suffix; not part of an identifier, a hex literal or a longer number
CXX_LIT = re.compile(r"(?<![\w.'])((?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+)([fFlL]?)(?![\w.'])")
DATA_LIT = re.compile(r"(?<![\w.])((?:\d+\.\d*|\.\d+)(?:[eEdDqQ][+-]?\d+)?|\d+[eEdDqQ][+-]?\d+)(?![\w.])")
FLOAT_CTX = re.compile(r"\b(?:real|rb|Real|R|T|float|static_cast\s*<\s*(?:float|real|rb|Real|R|T)\s*>)\s*[({]|"
                       r"\b(?:float|real|rb|Real)\b")


def f32(x):
    """binary64 -> binary32, round to nearest even (the C cast); None outside float's range"""
    try:
        return struct.unpack("<f", struct.pack("<f", x))[0]
    except OverflowError:
        return None


def f32_bits(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]


def f32_from_bits(b):
    return struct.unpack("<f", struct.pack("<I", b))[0]


def round_bin(q, p, emin):
    """exact rational q > 0 rounded to p significant bits (round to nearest even), subnormal below 2**emin"""
    e = q.numerator.bit_length() - q.denominator.bit_length()
    if Fraction(2) ** e > q:
        e -= 1
    e = max(e, emin) - (p - 1)  # weight of the last bit
    s = q / Fraction(2) ** e
    n, r = divmod(s.numerator, s.denominator)
    r2 = 2 * r
    if r2 > s.denominator or (r2 == s.denominator and n % 2 == 1):
        n += 1
    return Fraction(n) * Fraction(2) ** e


def check(text, long_double=False):
    """(double-rounded float, correct float) of the decimal text when they differ, else None"""
    t = re.sub(r"[dDqQ]", "e", text)
    D = Decimal(t)
    if D == 0:
        return None
    if long_double:
        q = Fraction(D)
        x = round_bin(q, 64, -16382)
        via = round_bin(x, 24, -126)
        right = round_bin(q, 24, -126)
        if via == right or right >= Fraction(2) ** 128:
            return None
        return float(via), float(right)
    d = float(t)  # correctly rounded binary64
    f = f32(d)
    if f is None or math.isinf(f) or f == d:
        return None
    # the double rounding errs only when d is exactly the midpoint of f and its neighbour on d's side
    b = f32_bits(f)
    other = f32_from_bits(b + 1 if abs(d) > abs(f) else b - 1)
    if d != (f + other) / 2:  # exact in binary64
        return None
    if D == Decimal(d):
        return None  # a true tie: both round to even
    # D lies beyond the midpoint toward `other` exactly when it is on other's side of d
    toward_other = (D > Decimal(d)) == (other > f)
    right = other if toward_other else f
    return (f, right) if right != f else None


def strip_cxx(src):
    """comments and string / character literals blanked, line structure kept"""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == "/" and i + 1 < n and src[i + 1] == "/":
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif c == "/" and i + 1 < n and src[i + 1] == "*":
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", src[i:j]))
            i = j
        elif c == '"' or (c == "'" and not (i > 0 and (src[i - 1].isalnum() or src[i - 1] == "_"))):
            j = i + 1
            while j < n and src[j] != c and src[j] != "\n":
                j += 2 if src[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(c + " " * (j - i - 2) + (c if j - i >= 2 else ""))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def walk(paths, exts=None):
    for p in paths:
        if os.path.isfile(p):
            yield p
            continue
        for root, dirs, files in os.walk(p):
            dirs.sort()
            for fn in sorted(files):
                if exts is None or os.path.splitext(fn)[1] in exts:
                    yield os.path.join(root, fn)


def is_text(path):
    try:
        with open(path, "rb") as fh:
            head = fh.read(8192)
    except OSError:
        return False
    return b"\0" not in head


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("src", nargs="*", help="source files / directories (default: src next to this script)")
    ap.add_argument("--data", nargs="+", default=[], metavar="PATH", help="text tables / namelists / directories")
    ap.add_argument("-v", "--verbose", action="store_true", help="print the counts")
    a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    srcs = a.src or ([] if a.data else [os.path.join(root, "src")])
    fail = listed = n = 0
    for path in walk(srcs, SRC_EXT):
        with open(path, encoding="utf-8", errors="replace") as fh:
            raw = fh.read()
        lines_raw = raw.split("\n")
        for ln, line in enumerate(strip_cxx(raw).split("\n"), 1):
            for m in CXX_LIT.finditer(line):
                if m.group(2) in ("f", "F"):
                    continue
                n += 1
                r = check(m.group(1), long_double=m.group(2) in ("l", "L"))
                if r is None:
                    continue
                ok = MARK in lines_raw[ln - 1]
                ctx = "float context" if FLOAT_CTX.search(line) else "context unclear"
                print(f"{path}:{ln}: {m.group(1)}{m.group(2)}: via double {r[0]!r} ({f32_bits(r[0]):#010x}), "
                      f"correct {r[1]!r} ({f32_bits(r[1]):#010x}); {ctx}" + ("; marked double only" if ok else ""))
                if ok:
                    listed += 1
                else:
                    fail += 1
    for path in walk(a.data):
        if not is_text(path):
            continue
        with open(path, encoding="latin-1") as fh:
            for ln, line in enumerate(fh, 1):
                for m in DATA_LIT.finditer(line):
                    n += 1
                    r = check(m.group(1))
                    if r is not None:
                        fail += 1
                        print(f"{path}:{ln}: {m.group(1)}: via double {r[0]!r} ({f32_bits(r[0]):#010x}), "
                              f"correct {r[1]!r} ({f32_bits(r[1]):#010x}) (read with strtod then a cast)")
    if a.verbose or fail:
        print(f"fp32_double_rounding: {n} decimal literals, {fail} failing, {listed} marked double only",
              file=sys.stderr)
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
