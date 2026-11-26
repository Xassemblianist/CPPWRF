#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""glibc 2.41 libmvec's SSE4 acos, asin, tan (2 doubles) and acosf, asinf, tanf (4 floats) as a C++ header.

  python3 tools/libmvec_sse4_trig.py GLIBC-2.41-SOURCE > src/core/libmvec_sse4_trig.hpp   (on an Intel x86-64 CPU)
  python3 tools/libmvec_sse4_trig.py --check src/core/libmvec_sse4_trig.hpp   (any x86-64 CPU: the number of float
                                       inputs whose rcpps / rsqrtps differs from the header's Intel tables; exit 1 if any)

WRF's gfortran build (FCOPTIM -O2 -ftree-vectorize, no -march: the SSE2 vector ABI) calls _ZGVbN2v_acos, _asin,
_tan and _ZGVbN4v_acosf, _asinf, _tanf in vectorised loops; glibc selects their SSE4 variants on every CPU with
SSE4.1 (sysdeps/x86_64/fpu/multiarch/svml_{d_acos2,d_asin2,d_tan2,s_acosf4,s_asinf4,s_tanf4}_core_sse4.S).
acos / asin (both widths) take `rsqrtps` of Y = 0.5 - 0.5*|x| (rounded to float), tan / tanf take `rcpps` in the
large-argument branch (|x| > 128000 / |x| > 10000 ...): the only instructions of these routines whose result the
IEEE standard does not define. Intel and AMD implement them differently, WRF's references are made on Intel.

The header written here is a transliteration of each routine, instruction by instruction (each C++ statement is
one instruction, with the .S line in a comment): the 128-bit registers are __m128i values, the general-purpose
registers 64-bit integers, the stack frame and glibc's data tables byte arrays at the routine's offsets, the
branches gotos. Every SSE2 instruction is the SSE2 intrinsic of the same instruction (the same IEEE operation on
any x86-64 CPU); rcpps and rsqrtps are Intel's, from the tables dumped here: on Intel, rcpps of a normal float
depends only on the top 11 mantissa bits and rsqrtps only on the exponent's parity and the top 10 mantissa bits,
and both scale exactly with the exponent (checked here over every input; the special classes are coded and
tests/unit/test_libmvec_trig.cpp compares both with the hardware over all 2^32 inputs on Intel). The scalar
fallback lanes call the system libm's acos ... tanf, as the assembly does (acos@PLT ...). glibc's data are copied
bit for bit from the sources.
"""
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from libmvec_tables import Rcpps, cpu_model  # noqa: E402

MULTIARCH = "sysdeps/x86_64/fpu/multiarch"
ROUTINES = [  # (C++ name, .S file, vector type, scalar fallback)
    ("acos2", "svml_d_acos2_core_sse4.S", "pd", "acos"),
    ("asin2", "svml_d_asin2_core_sse4.S", "pd", "asin"),
    ("tan2", "svml_d_tan2_core_sse4.S", "pd", "tan"),
    ("acosf4", "svml_s_acosf4_core_sse4.S", "ps", "acosf"),
    ("asinf4", "svml_s_asinf4_core_sse4.S", "ps", "asinf"),
    ("tanf4", "svml_s_tanf4_core_sse4.S", "ps", "tanf"),
]


class Rsqrtps(Rcpps):
    # loop: movups (%rdi),%xmm0; rsqrtps %xmm0,%xmm0; movups %xmm0,(%rdi); add $16,%rdi; sub $4,%rsi; ja loop; ret
    CODE = bytes.fromhex("0f1007" "0f52c0" "0f1107" "4883c710" "4883ee04" "77ed" "c3")


def dump_tables():
    """Intel's rcpps for [1, 2) (2048 top-11-bit classes) and rsqrtps for [1, 2) and [2, 4) (1024 top-10-bit
    classes each), after checking that these classes and the exponent scaling describe every normal input."""
    rcp, rsq = Rcpps(), Rsqrtps()
    mant = np.arange(1 << 23, dtype=np.uint32)
    r = rcp((np.uint32(127) << 23) | mant)
    if not np.all(r.reshape(2048, 4096) == r.reshape(2048, 4096)[:, :1]):
        sys.exit("rcpps depends on more than the top 11 mantissa bits")
    tr = r.reshape(2048, 4096)[:, 0].copy()
    for e in range(1, 255):
        got = rcp((np.uint32(e) << 23) | mant)
        exp = (r >> 23).astype(np.int64) - (e - 127)
        want = np.where(exp <= 0, 0, (exp.clip(0) << 23) | (r & 0x7FFFFF)).astype(np.uint32)
        if not np.array_equal(got, want):
            sys.exit(f"rcpps does not scale with the exponent (biased exponent {e})")
    ts = []
    for e0 in (127, 128):
        s = rsq((np.uint32(e0) << 23) | mant)
        if not np.all(s.reshape(1024, 8192) == s.reshape(1024, 8192)[:, :1]):
            sys.exit("rsqrtps depends on more than the exponent parity and the top 10 mantissa bits")
        ts.append(s.reshape(1024, 8192)[:, 0].copy())
    for e in range(1, 255):
        odd = 1 if e % 2 == 0 else 0  # e = 128, 130, ...: the [2, 4) class
        base = ts[odd]
        got = rsq((np.uint32(e) << 23) | mant)
        sh = (e - 127 - odd) // 2
        want = ((((base >> 23).astype(np.int64) - sh) << 23) | (base & 0x7FFFFF)).astype(np.uint32)
        if not np.array_equal(got, want[mant >> 13]):
            sys.exit(f"rsqrtps does not scale with the exponent (biased exponent {e})")
    return [int(v) for v in tr], [int(v) for v in np.concatenate(ts)]


def check(header):
    text = open(header).read()

    def arr(name):
        body = text[text.index(f"{name}[2048] = {{"):]
        return np.array([int(v, 16) for v in re.findall(r"0x([0-9a-f]+)", body[:body.index("}")])], dtype=np.uint32)
    trc, trs = arr("kIntelRcp"), arr("kIntelRsqrt")
    mant = np.arange(1 << 23, dtype=np.uint32)
    differ_rc = differ_rs = 0
    rcp, rsq = Rcpps(), Rsqrtps()
    for e in range(1, 255):
        x = (np.uint32(e) << 23) | mant
        r = trc[mant >> 12]
        exp = (r >> 23).astype(np.int64) - (e - 127)
        want = np.where(exp <= 0, 0, (exp.clip(0) << 23) | (r & 0x7FFFFF)).astype(np.uint32)
        differ_rc += int(np.sum(rcp(x) != want))
        odd = 1 if e % 2 == 0 else 0
        s = trs[odd * 1024 + (mant >> 13)]
        sh = (e - 127 - odd) // 2
        want = ((((s >> 23).astype(np.int64) - sh) << 23) | (s & 0x7FFFFF)).astype(np.uint32)
        differ_rs += int(np.sum(rsq(x) != want))
    n = 254 << 23
    print(f"{cpu_model()[1]}: rcpps differs from {header} for {differ_rc} of {n} positive normal floats, "
          f"rsqrtps for {differ_rs}")
    return differ_rc + differ_rs


# ---------------------------------------------------------------------------------------------------- the .S files
def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    return [re.sub(r"(//|#\s+LOE).*$", "", ln).rstrip() for ln in text.split("\n")]


def parse_data(lines):
    """{label: bytes} of the .rodata labels (.quad / .long / .align 16 / .space), and the order."""
    data, order, cur, inside_typedef = {}, [], None, False
    for ln in lines:
        s = ln.strip()
        if s.startswith("#ifdef"):
            inside_typedef = True
        if inside_typedef:
            if s.startswith("#endif"):
                inside_typedef = False
            continue
        m = re.match(r"^([.\w]+):$", s)
        if m and not s.startswith("L("):
            cur = m.group(1)
            data[cur] = bytearray()
            order.append(cur)
            continue
        if cur is None:
            continue
        if s.startswith(".quad"):
            for v in s[5:].split(","):
                data[cur] += struct.pack("<Q", int(v.strip(), 16))
        elif s.startswith(".long"):
            for v in s[5:].split(","):
                data[cur] += struct.pack("<I", int(v.strip(), 16) & 0xFFFFFFFF)
        elif s.startswith(".align"):
            a = int(s.split()[1])
            data[cur] += b"\0" * (-len(data[cur]) % a)
        elif s.startswith(".space"):
            n, fill = [int(v, 0) for v in s[6:].split(",")]
            data[cur] += bytes([fill]) * n
        elif s.startswith(".size") or s.startswith(".type"):
            if s.startswith(".size") and s.split()[1].rstrip(",") == cur:
                cur = None
        elif s.startswith(".section") or not s:
            continue
        elif s.startswith("ENTRY") or s.startswith("END"):
            cur = None
        else:
            sys.exit(f"unknown data line: {s}")
    return data, order


def cname(label):
    return "k_" + label.strip(".").replace(".", "_")


REG64 = {}
for i, n in enumerate(["rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi"]):
    REG64[n] = n
    REG64["e" + n[1:]] = n
for i in range(8, 16):
    REG64[f"r{i}"] = f"r{i}"
    REG64[f"r{i}d"] = f"r{i}"


class Translator:
    def __init__(self, name, path, vt, scalar):
        self.name, self.vt, self.scalar = name, vt, scalar
        text = open(path).read()
        self.defines = {m.group(1): int(m.group(2), 0)
                        for m in re.finditer(r"^#define\s+(\w+)\s+(\d+)\s*$", text, flags=re.M)}
        self.lines = strip_comments(text)
        self.data, self.order = parse_data(self.lines)
        self.prefix = name + "_"
        self.frame = 0
        self.out = []

    def sym(self, label):
        return self.prefix + cname(label)

    def expr(self, e):
        """offset expression of a memory operand: numbers, #defines, at most one data label."""
        off, label = 0, None
        for t in re.findall(r"[+-]?[^+-]+", e.replace(" ", "")):
            sign = -1 if t.startswith("-") else 1
            t = t.lstrip("+-")
            if re.fullmatch(r"\d+|0x[0-9a-fA-F]+", t):
                off += sign * int(t, 0)
            elif t in self.defines:
                off += sign * self.defines[t]
            elif t in self.data:
                assert label is None and sign == 1
                label = t
            else:
                sys.exit(f"{self.name}: unknown symbol {t}")
        return off, label

    def mem(self, op):
        """C++ address expression (const unsigned char*) of an AT&T memory operand."""
        m = re.fullmatch(r"([^()]*)\(([^()]*)\)", op)
        assert m, op
        off, label = self.expr(m.group(1)) if m.group(1) else (0, None)
        parts = [p.strip() for p in m.group(2).split(",")]
        if parts[0] == "%rip":
            assert label is not None and len(parts) == 1
            self.used.add(label)
            return f"(reinterpret_cast<const unsigned char*>({self.sym(label)}) + {off})"
        assert label is None
        terms = []
        base = parts[0].lstrip("%") if parts[0] else None
        if base == "rsp":
            assert len(parts) in (1, 3) and off >= 0
            idx = ""
            if len(parts) == 3:
                idx = f" + {REG64[parts[1].lstrip('%')]} * {int(parts[2])}"
            return f"(stk + {off}{idx})"
        if base:
            terms.append(REG64[base])
        if len(parts) >= 2:
            sc = int(parts[2]) if len(parts) == 3 else 1
            terms.append(f"{REG64[parts[1].lstrip('%')]}" + (f" * {sc}" if sc != 1 else ""))
        return f"(reinterpret_cast<const unsigned char*>({' + '.join(terms)}) + ({off}))"

    def wmem(self, op):
        a = self.mem(op)
        assert a.startswith("(stk"), op  # stores go to the stack frame only
        return a

    def x(self, op):
        m = re.fullmatch(r"%xmm(\d+)", op)
        return f"x{m.group(1)}" if m else None

    def src128(self, op):
        r = self.x(op)
        return r if r else f"ld128({self.mem(op)})"

    def translate(self):
        body = []
        on = False
        for no, ln in enumerate(self.lines, 1):
            s = ln.strip()
            if s.startswith("ENTRY("):
                on = True
                continue
            if s.startswith("END("):
                break
            if not on or not s or s.startswith("cfi_") or s.startswith("."):
                continue
            m = re.fullmatch(r"L\((\w+)\):", s)
            if m:
                body.append(f"L_{m.group(1)}:;")
                continue
            mn, _, rest = s.partition("\t") if "\t" in s else s.partition(" ")
            ops = [o.strip() for o in re.split(r",(?![^()]*\))", rest)] if rest.strip() else []
            body.append(f"{self.insn(mn.strip(), ops, s)}  // {no}: {s.replace(chr(9), ' ')}")
        return body

    def insn(self, mn, ops, s):
        F = {"pd": ("pd", "_mm_castsi128_pd", "_mm_castpd_si128"), "ps": ("ps", "_mm_castsi128_ps", "_mm_castps_si128")}
        arith = {"mulpd": "_mm_mul_pd", "addpd": "_mm_add_pd", "subpd": "_mm_sub_pd", "divpd": "_mm_div_pd",
                 "minpd": "_mm_min_pd", "cmpltpd": "_mm_cmplt_pd", "cmpnltpd": "_mm_cmpnlt_pd",
                 "cmpnlepd": "_mm_cmpnle_pd", "cmpneqpd": "_mm_cmpneq_pd", "cmplepd": "_mm_cmple_pd",
                 "cmpeqpd": "_mm_cmpeq_pd",
                 "mulps": "_mm_mul_ps", "addps": "_mm_add_ps", "subps": "_mm_sub_ps", "divps": "_mm_div_ps",
                 "minps": "_mm_min_ps", "cmpltps": "_mm_cmplt_ps", "cmpnltps": "_mm_cmpnlt_ps",
                 "cmpnleps": "_mm_cmpnle_ps", "cmpneqps": "_mm_cmpneq_ps", "cmpleps": "_mm_cmple_ps",
                 "cmpeqps": "_mm_cmpeq_ps"}
        ints = {"andps": "_mm_and_si128", "pand": "_mm_and_si128", "andnps": "_mm_andnot_si128",
                "orps": "_mm_or_si128", "por": "_mm_or_si128", "pxor": "_mm_xor_si128",
                "pmuludq": "_mm_mul_epu32", "paddq": "_mm_add_epi64", "paddd": "_mm_add_epi32",
                "punpckldq": "_mm_unpacklo_epi32", "punpcklqdq": "_mm_unpacklo_epi64"}
        shifts = {"psrlq": "_mm_srli_epi64", "psllq": "_mm_slli_epi64", "psrld": "_mm_srli_epi32",
                  "pslld": "_mm_slli_epi32"}
        if mn in arith:
            t = "pd" if mn.endswith("pd") else "ps"
            _, c, b = F[t]
            d = self.x(ops[1])
            return f"{d} = {b}({arith[mn]}({c}({d}), {c}({self.src128(ops[0])})));"
        if mn in ints:
            d = self.x(ops[1])
            return f"{d} = {ints[mn]}({d}, {self.src128(ops[0])});"
        if mn in shifts:
            return f"{self.x(ops[1])} = {shifts[mn]}({self.x(ops[1])}, {int(ops[0][1:], 0)});"
        if mn in ("movaps", "movups", "movdqa", "movdqu"):
            if self.x(ops[1]):
                return f"{self.x(ops[1])} = {self.src128(ops[0])};"
            return f"st128({self.wmem(ops[1])}, {self.x(ops[0])});"
        if mn in ("movmskpd", "movmskps"):
            r = REG64[ops[1].lstrip("%")]
            c = "_mm_movemask_pd(_mm_castsi128_pd" if mn == "movmskpd" else "_mm_movemask_ps(_mm_castsi128_ps"
            return f"{r} = std::uint32_t({c}({self.x(ops[0])})));"
        if mn == "cvtpd2ps":
            return f"{self.x(ops[1])} = _mm_castps_si128(_mm_cvtpd_ps(_mm_castsi128_pd({self.x(ops[0])})));"
        if mn == "cvtps2pd":
            return f"{self.x(ops[1])} = _mm_castpd_si128(_mm_cvtps_pd(_mm_castsi128_ps({self.x(ops[0])})));"
        if mn == "movlhps":
            d = self.x(ops[1])
            return f"{d} = _mm_castps_si128(_mm_movelh_ps(_mm_castsi128_ps({d}), _mm_castsi128_ps({self.x(ops[0])})));"
        if mn == "rcpps":
            return f"{self.x(ops[1])} = intel_rcpps({self.x(ops[0])});"
        if mn == "rsqrtps":
            return f"{self.x(ops[1])} = intel_rsqrtps({self.x(ops[0])});"
        if mn == "pshufd":
            return f"{self.x(ops[2])} = _mm_shuffle_epi32({self.x(ops[1])}, {int(ops[0][1:], 0)});"
        if mn == "pextrw":
            return f"{REG64[ops[2].lstrip('%')]} = std::uint32_t(_mm_extract_epi16({self.x(ops[1])}, {int(ops[0][1:], 0)}));"
        if mn == "movd":
            a, b = ops
            if self.x(b) and a.startswith("%") and not self.x(a):
                return f"{self.x(b)} = _mm_cvtsi32_si128(int(std::uint32_t({REG64[a.lstrip('%')]})));"
            if self.x(a) and b.startswith("%"):
                return f"{REG64[b.lstrip('%')]} = std::uint32_t(_mm_cvtsi128_si32({self.x(a)}));"
            if self.x(b):
                return f"{self.x(b)} = ld32({self.mem(a)});"
            sys.exit(f"movd form: {s}")
        if mn == "movq":
            a, b = ops
            if self.x(b):
                return f"{self.x(b)} = ld64({self.mem(a)});"
            if a.startswith("%r") and "(" in b:
                return f"st_u64({self.wmem(b)}, {REG64[a.lstrip('%')]});"
            if "(" in a and b.startswith("%r"):
                return f"{REG64[b.lstrip('%')]} = ld_u64({self.mem(a)});"
            sys.exit(f"movq form: {s}")
        if mn == "movhpd":
            d = self.x(ops[1])
            return f"{d} = ldh64({d}, {self.mem(ops[0])});"
        if mn in ("movsd", "movss"):
            a, b = ops
            n = 64 if mn == "movsd" else 32
            if self.x(b) and "(" in a:
                return f"{self.x(b)} = ld{n}({self.mem(a)});"
            if self.x(a) and "(" in b:
                return f"st{n}({self.wmem(b)}, {self.x(a)});"
            sys.exit(f"{mn} form: {s}")
        if mn == "movl":
            a, b = ops
            d = REG64[b.lstrip("%")]
            if a.startswith("$"):
                return f"{d} = std::uint32_t({int(a[1:], 0)});"
            return f"{d} = std::uint32_t({REG64[a.lstrip('%')]});"
        if mn == "xorl":
            assert ops[0] == ops[1]
            return f"{REG64[ops[0].lstrip('%')]} = 0;"
        if mn == "lea":
            d = REG64[ops[1].lstrip("%")]
            m = re.fullmatch(r"\(%(\w+),\s*%(\w+),\s*(\d+)\)", ops[0])
            if m:  # lea (%rdx, %rdx, 2), %ecx: 32-bit destination
                assert ops[1].startswith("%e")
                return f"{d} = std::uint32_t({REG64[m.group(1)]} + {REG64[m.group(2)]} * {m.group(3)});"
            return f"{d} = reinterpret_cast<std::uint64_t>({self.mem(ops[0])});"
        if mn == "shll":
            d = REG64[ops[1].lstrip("%")]
            return f"{d} = std::uint32_t(std::uint32_t({d}) << {int(ops[0][1:], 0)});"
        if mn == "imull":
            d = REG64[ops[2].lstrip("%")]
            return f"{d} = std::uint32_t(std::uint32_t({REG64[ops[1].lstrip('%')]}) * {int(ops[0][1:], 0)}u);"
        if mn == "testl":
            a, b = (REG64[o.lstrip("%")] for o in ops)
            return f"zf = (std::uint32_t({a}) & std::uint32_t({b})) == 0;"
        if mn == "btl":
            a, b = (REG64[o.lstrip("%")] for o in ops)
            return f"cf = ((std::uint32_t({b}) >> (std::uint32_t({a}) & 31)) & 1) != 0;"
        if mn == "incl":
            d = REG64[ops[0].lstrip("%")]
            return f"{d} = std::uint32_t({d} + 1);"
        if mn == "cmpl":
            d = REG64[ops[1].lstrip("%")]
            return f"lt = std::int32_t({d}) < {int(ops[0][1:], 0)};"
        if mn in ("jne", "jmp", "jl", "jc"):
            lab = re.fullmatch(r"L\((\w+)\)", ops[0]).group(1)
            cond = {"jne": "!zf", "jl": "lt", "jc": "cf", "jmp": None}[mn]
            return f"{'if (' + cond + ') ' if cond else ''}goto L_{lab};"
        if mn in ("subq", "addq"):
            assert ops[1] == "%rsp"
            if mn == "subq":
                self.frame = int(ops[0][1:], 0)
            return ";"
        if mn == "call":
            f = ops[0].replace("@PLT", "")
            assert f == self.scalar
            if self.vt == "pd":
                return f"x0 = _mm_castpd_si128(_mm_set_sd(::{f}(_mm_cvtsd_f64(_mm_castsi128_pd(x0)))));"
            return f"x0 = _mm_castps_si128(_mm_set_ss(::{f}(_mm_cvtss_f32(_mm_castsi128_ps(x0)))));"
        if mn == "ret":
            return "return x0;"
        sys.exit(f"{self.name}: unknown instruction {s}")

    def emit(self):
        self.used = set()
        body = self.translate()
        out = []
        for label in self.order:
            if label not in self.used:
                continue
            b = bytes(self.data[label])
            b += b"\0" * (-len(b) % 4)
            words = struct.unpack(f"<{len(b) // 4}I", b)
            out.append(f"alignas(16) inline constexpr std::uint32_t {self.sym(label)}[{len(words)}] = {{  // {label}")
            for i in range(0, len(words), 8):
                out.append("    " + ", ".join(f"0x{w:08x}" for w in words[i:i + 8]) + ",")
            out.append("};")
        out.append(f"// _ZGVbN{'2' if self.vt == 'pd' else '4'}v_{self.scalar}_sse4")
        out.append(f"inline __m128i {self.name}_sse4(__m128i x0) {{")
        out.append(f"  alignas(16) unsigned char stk[{self.frame}] = {{}};")
        out.append("  __m128i " + ", ".join(f"x{i} = _mm_setzero_si128()" for i in range(1, 16)) + ";")
        out.append("  std::uint64_t rax = 0, rcx = 0, rdx = 0, rsi = 0, rdi = 0, r8 = 0, r9 = 0, r10 = 0, r11 = 0, "
                   "r12 = 0, r13 = 0, r14 = 0;")
        out.append("  bool zf = false, cf = false, lt = false;")
        regs = "".join(body)
        unused = [v for v in ("rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "r11", "r12", "r13", "r14",
                              "zf", "cf", "lt") if not re.search(rf"\b{v}\b", regs)]
        if unused:
            out.append(f"  (void){', (void)'.join(unused)};")
        out += ["  " + b if not b.startswith("L_") else b for b in body]
        out.append("}")
        return out


HELPERS = r"""
// Intel's rcpps / rsqrtps (one lane each; tables for [1, 2) and, rsqrtps, [2, 4)): a zero or denormal input
// gives +-Inf, +-Inf gives +-0 (rcpps) / +0 or the default NaN (rsqrtps), a NaN its quiet NaN, a negative
// non-zero input of rsqrtps the default NaN, and rcpps of |x| >= 2^126 with a result below 2^-126 +-0.
inline std::uint32_t intel_rcp1(std::uint32_t b) {
  const std::uint32_t s = b & 0x80000000u, e = (b >> 23) & 0xffu, m = b & 0x7fffffu;
  if (e == 0) return s | 0x7f800000u;
  if (e == 255) return m ? (b | 0x400000u) : s;
  const std::uint32_t r = kIntelRcp[m >> 12];
  const int re = int(r >> 23) - (int(e) - 127);
  return re <= 0 ? s : (s | (std::uint32_t(re) << 23) | (r & 0x7fffffu));
}
inline std::uint32_t intel_rsqrt1(std::uint32_t b) {
  const std::uint32_t s = b & 0x80000000u, e = (b >> 23) & 0xffu, m = b & 0x7fffffu;
  if (e == 0) return s | 0x7f800000u;
  if (e == 255 && m) return b | 0x400000u;
  if (s) return 0xffc00000u;
  if (e == 255) return 0;
  const std::uint32_t odd = (e & 1u) ? 0u : 1u;  // biased exponent 128, 130 ...: the [2, 4) table
  const std::uint32_t r = kIntelRsqrt[(odd << 10) | (m >> 13)];
  const int re = int(r >> 23) - (int(e) - 127 - int(odd)) / 2;
  return (std::uint32_t(re) << 23) | (r & 0x7fffffu);
}
inline __m128i intel_rcpps(__m128i v) {
  alignas(16) std::uint32_t a[4];
  _mm_store_si128(reinterpret_cast<__m128i*>(a), v);
  for (auto& w : a) w = intel_rcp1(w);
  return _mm_load_si128(reinterpret_cast<const __m128i*>(a));
}
inline __m128i intel_rsqrtps(__m128i v) {
  alignas(16) std::uint32_t a[4];
  _mm_store_si128(reinterpret_cast<__m128i*>(a), v);
  for (auto& w : a) w = intel_rsqrt1(w);
  return _mm_load_si128(reinterpret_cast<const __m128i*>(a));
}
// memory operands (unaligned, as movups / movq / movd / movhpd / movsd / movss)
inline __m128i ld128(const unsigned char* p) { return _mm_loadu_si128(reinterpret_cast<const __m128i*>(p)); }
inline __m128i ld64(const unsigned char* p) { return _mm_loadl_epi64(reinterpret_cast<const __m128i*>(p)); }
inline __m128i ld32(const unsigned char* p) {
  std::int32_t v;
  std::memcpy(&v, p, 4);
  return _mm_cvtsi32_si128(v);
}
inline __m128i ldh64(__m128i x, const unsigned char* p) {
  return _mm_castpd_si128(_mm_loadh_pd(_mm_castsi128_pd(x), reinterpret_cast<const double*>(p)));
}
inline std::uint64_t ld_u64(const unsigned char* p) {
  std::uint64_t v;
  std::memcpy(&v, p, 8);
  return v;
}
inline void st128(unsigned char* p, __m128i x) { _mm_storeu_si128(reinterpret_cast<__m128i*>(p), x); }
inline void st64(unsigned char* p, __m128i x) { _mm_storel_epi64(reinterpret_cast<__m128i*>(p), x); }
inline void st32(unsigned char* p, __m128i x) {
  const std::int32_t v = _mm_cvtsi128_si32(x);
  std::memcpy(p, &v, 4);
}
inline void st_u64(unsigned char* p, std::uint64_t v) { std::memcpy(p, &v, 8); }
"""


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--check":
        sys.exit(1 if check(sys.argv[2]) else 0)
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    glibc = sys.argv[1]
    vendor, model = cpu_model()
    if vendor != "GenuineIntel":
        sys.exit(f"not an Intel CPU ({vendor}): the tables must be Intel's rcpps / rsqrtps")
    trc, trs = dump_tables()
    out = [
        "// SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later",
        "#pragma once",
        "// GENERATED by tools/libmvec_sse4_trig.py from glibc 2.41's sources and Intel's rcpps / rsqrtps; do not",
        "// edit. glibc 2.41 libmvec's _ZGVbN2v_acos / _asin / _tan and _ZGVbN4v_acosf / _asinf / _tanf (the SSE4",
        "// variants glibc selects on every CPU with SSE4.1), transliterated instruction by instruction from",
        "// " + MULTIARCH + "/svml_{d_acos2,d_asin2,d_tan2,s_acosf4,s_asinf4,s_tanf4}_core_sse4.S",
        "// (each statement one instruction, its .S line in the comment; registers x0 ... x15, rax ... r14, the stack",
        "// frame stk, the branches gotos), with rcpps / rsqrtps Intel's on any x86-64 CPU (see the generator's",
        "// docstring). Host x86-64 only (SSE2 intrinsics): tests/phys/libmvec_intel.cpp, tests/unit/test_libmvec_trig.",
        f"// rcpps / rsqrtps dumped on: {model}",
        "#include <emmintrin.h>",
        "",
        "#include <cmath>",
        "#include <cstdint>",
        "#include <cstring>",
        "",
        "namespace cppwrf::mvec_sse4 {",
        "",
        "// Intel rcpps of the floats [1, 2) by their top 11 mantissa bits (result bit patterns)",
        "inline constexpr std::uint32_t kIntelRcp[2048] = {",
    ]
    for i in range(0, 2048, 8):
        out.append("    " + ", ".join(f"0x{w:08x}" for w in trc[i:i + 8]) + ",")
    out += ["};", "// Intel rsqrtps of [1, 2) (0 ... 1023) and [2, 4) (1024 ... 2047) by their top 10 mantissa bits",
            "inline constexpr std::uint32_t kIntelRsqrt[2048] = {"]
    for i in range(0, 2048, 8):
        out.append("    " + ", ".join(f"0x{w:08x}" for w in trs[i:i + 8]) + ",")
    out.append("};")
    out += HELPERS.strip("\n").split("\n")
    for name, f, vt, scalar in ROUTINES:
        out.append("")
        out += Translator(name, f"{glibc}/{MULTIARCH}/{f}", vt, scalar).emit()
    out += ["", "}  // namespace cppwrf::mvec_sse4", ""]
    sys.stdout.write("\n".join(out))


if __name__ == "__main__":
    main()
