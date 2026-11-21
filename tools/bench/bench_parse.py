#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""One timing parser for WRF and CPPWRF (tools/bench/PROTOCOL.md, "What is timed").

  bench_parse.py wrf RSL_OUT [--wall S]        WRF's rsl.out.0000
  bench_parse.py cppwrf LOG [--wall S]         CPPWRF's standard output of a run with output.wrf_timing = true
  bench_parse.py --help

prints one JSON object. Both models print WRF's timing lines ("Timing for main: time T on domain N: S elapsed
seconds" after each step of each domain, "Timing for Writing F for domain N: S elapsed seconds" after each history
or restart write; CPPWRF with output.wrf_timing = true, src/cases/domain_runner.cpp), so one rule splits both:

- main: the sum of domain 1's "Timing for main" lines (a nest's steps run inside its parent's step);
- io: every "Timing for Writing" line of every domain;
- io_in: the writes inside domain 1's steps, i.e. printed before domain 1's last "Timing for main" line. WRF writes
  the frame at t = 0 inside the first step (med_before_solve_io); CPPWRF writes domain 1's frame at t = 0 in its
  set-up, before the first step's clock starts: in CPPWRF mode domain 1's writes before its first step are outside
  (io_pre). A nest's frame at t = 0 is written inside domain 1's first step in both models (the nest starts there);
- stepping = main - io_in: the computation of all steps (with the nests' steps and, in step 1, the nests' start);
- step1 = domain 1's first "Timing for main" minus the writes inside it: the first step (CUDA graph capture,
  first-touch allocations, the nests' start);
- steady = stepping - step1 over steps 2 .. N (steady_steps = N - 1);
- other = wall - stepping - io (with --wall): process start, reading the inputs, initialisation, finalisation.
  wall = other + step1 + steady + io.

Also reported: input (WRF's "Timing for processing wrfinput / lateral boundary" lines), done_stepping (CPPWRF's
"cppwrf: done ... stepping S s" line, the domain runner's own clock: it contains a nest's output, see docs/PERF.md),
and steps_by_domain.
"""
import json
import re
import sys

RE_MAIN = re.compile(r"Timing for main: time (\S+) on domain\s+(\d+):\s+([0-9.]+) elapsed")
RE_WRITE = re.compile(r"Timing for Writing (.*?) for domain\s+(\d+):\s+([0-9.]+) elapsed")
RE_INPUT = re.compile(r"Timing for processing (?:wrfinput file|lateral boundary).*:\s+([0-9.]+) elapsed")
RE_DONE = re.compile(r"cppwrf: done, (\d+) steps, stepping ([0-9.]+) s")


def parse(text, model, wall=None):
    """model: 'wrf' or 'cppwrf'. Returns the components above (seconds); ok False without domain 1 steps."""
    main, writes, steps_by = [], [], {}
    inp = 0.0
    for i, line in enumerate(text.splitlines()):
        m = RE_MAIN.search(line)
        if m:
            d = int(m.group(2))
            steps_by[d] = steps_by.get(d, 0) + 1
            if d == 1:
                main.append((i, float(m.group(3))))
            continue
        m = RE_WRITE.search(line)
        if m:
            writes.append((i, int(m.group(2)), float(m.group(3)), m.group(1).strip()))
            continue
        m = RE_INPUT.search(line)
        if m:
            inp += float(m.group(1))
    r = dict(model=model, steps=len(main), steps_by_domain={str(k): v for k, v in sorted(steps_by.items())},
             input=round(inp, 6), frames=len(writes), ok=bool(main))
    done = RE_DONE.search(text)
    if done:
        r["done_stepping"] = float(done.group(2))
    io_all = sum(w[2] for w in writes)
    r["io"] = io_all
    if not main:
        if wall is not None:
            r["wall"] = wall
        return r
    first, last = main[0][0], main[-1][0]

    def outside_pre(w):  # CPPWRF: domain 1's writes before its first step are in the set-up
        return model == "cppwrf" and w[0] < first and w[1] == 1

    io_in = sum(w[2] for w in writes if w[0] < last and not outside_pre(w))
    io_1 = sum(w[2] for w in writes if w[0] < first and not outside_pre(w))
    io_pre = sum(w[2] for w in writes if outside_pre(w))
    msum = sum(x for _, x in main)
    r.update(main=msum, io_in=io_in, io_pre=io_pre, io_post=io_all - io_in - io_pre, stepping=msum - io_in,
             step1=main[0][1] - io_1, steady_steps=len(main) - 1)
    r["steady"] = r["stepping"] - r["step1"]
    if wall is not None:
        r["wall"] = wall
        r["other"] = wall - r["stepping"] - io_all
    return r


def main():
    a = sys.argv[1:]
    if len(a) < 2 or a[0] not in ("wrf", "cppwrf"):
        print(__doc__)
        sys.exit(0 if a and a[0] in ("-h", "--help") else 2)
    wall = float(a[a.index("--wall") + 1]) if "--wall" in a else None
    print(json.dumps(parse(open(a[1], errors="replace").read(), a[0], wall), indent=1))


if __name__ == "__main__":
    main()
