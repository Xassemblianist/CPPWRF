#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""One timing parser for WRF and CPPWRF (tools/bench/PROTOCOL.md, "What is timed").

  bench_parse.py wrf RSL_OUT [--wall S]                  WRF's rsl.out.0000
  bench_parse.py cppwrf LOG [--wall S] [--overlap]       CPPWRF's standard output with output.wrf_timing = true
  bench_parse.py --help

prints one JSON object. Both models print WRF's timing lines ("Timing for main: time T on domain N: S elapsed
seconds" after each step of each domain, "Timing for Writing F for domain N: S elapsed seconds" after each history
or restart write; WRF also "Timing for processing wrfinput file ... / lateral boundary for domain N"; CPPWRF with
output.wrf_timing = true, src/cases/domain_runner.cpp), so one rule splits both. Domain 1's "Timing for main" lines
cut the log into intervals: interval 1 (step 1) is everything inside domain 1's first step, interval j the lines
after main line j - 1 up to main line j. Inside step 1 is everything before the first main line except what each
model does before its first step clock starts: WRF's domain 1 input ("processing wrfinput file ... for domain 1",
read before integrate), CPPWRF's domain 1 writes (its frame at t = 0, written in the set-up). WRF opens a nest
inside domain 1's first step (nests_to_open: allocation, the nest's wrfinput, the interpolation, start_domain) and
reads the lateral boundaries inside steps (med_latbound_in at t = 0 and every interval_seconds); CPPWRF opens the
nests and reads all boundaries in its set-up. Hence:

- step1 = domain 1's first main line minus the writes and inputs inside it. NOT comparable across models: it holds
  WRF's nest opening and t = 0 boundary read, and CPPWRF's CUDA graph capture; it belongs to the fixed cost;
- steady = domain 1's main lines 2 .. N minus the writes and inputs inside them: the computation of the steps (a
  nest's steps run inside its parent's step), the same work in both models; steady_steps = N - 1. With --overlap
  (CPPWRF CUDA builds with CUDA graphs: a step's main line is printed when the host has launched it, the device's
  work of step n is waited for in step n + 1, the last step is waited for before its line) the intervals 2 .. N
  hold the device work of steps 1 .. N: steady_steps = N;
- step_s = steady / steady_steps: seconds per domain-1 step (the headline compute metric, history off);
- compute = step_s x N: the run's computation, every step at the steady rate;
- io: every write; io_pre (before step 1's clock), io_1 (inside step 1), io_steady (inside steps 2 .. N), io_post
  (after the last main line: the frame at the end, finalisation writes);
- input_in: the inputs inside the steps (input_1 inside step 1, input_steady inside steps 2 .. N); input: all;
- stepping = step1 + steady (both models' "Timing for main" minus what is inside them);
- other = wall - stepping - io - input_in (with --wall): process start, the inputs and initialisation before the
  first step, finalisation. wall = other + step1 + steady + io + input_in;
- marginal = compute + io_steady + io_post + input_steady: the part of the run that grows with the simulated time
  (the steps and the frames after t = 0); fixed = wall - marginal: start-up, the frame at t = 0 and the finish, once
  per run (tools/bench/PROTOCOL.md, "Cost"). With --overlap and history on, steady and io_steady are not separable
  (a write waits for the queued steps), their sum and marginal are.

Also reported: done_stepping (CPPWRF's "cppwrf: done ... stepping S s" line, the domain runner's own clock) and
steps_by_domain.
"""
import json
import re
import sys

RE_MAIN = re.compile(r"Timing for main: time (\S+) on domain\s+(\d+):\s+([0-9.]+) elapsed")
RE_WRITE = re.compile(r"Timing for Writing (.*?) for domain\s+(\d+):\s+([0-9.]+) elapsed")
RE_INPUT = re.compile(r"Timing for processing (wrfinput file|lateral boundary).*?for domain\s+(\d+):\s+([0-9.]+) "
                      r"elapsed")
RE_DONE = re.compile(r"cppwrf: done, (\d+) steps, stepping ([0-9.]+) s")


def parse(text, model, wall=None, overlap=False):
    """model: 'wrf' or 'cppwrf'. Returns the components above (seconds); ok False without domain 1 steps."""
    main, writes, inputs, steps_by = [], [], [], {}
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
            inputs.append((i, int(m.group(2)), float(m.group(3)), m.group(1)))
    r = dict(model=model, steps=len(main), steps_by_domain={str(k): v for k, v in sorted(steps_by.items())},
             input=round(sum(x[2] for x in inputs), 6), frames=len(writes), ok=bool(main), overlap=bool(overlap))
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

    def where(x, is_write):
        """'pre', '1', 'steady' or 'post' for a write or an input line."""
        if x[0] > last:
            return "post"
        if x[0] > first:
            return "steady"
        if is_write and model == "cppwrf" and x[1] == 1:
            return "pre"  # CPPWRF: domain 1's frame at t = 0 is written in the set-up
        if not is_write and x[3] == "wrfinput file" and x[1] == 1:
            return "pre"  # WRF: domain 1's input is read before integrate
        return "1"

    io = dict(pre=0.0, steady=0.0, post=0.0)
    io["1"] = 0.0
    inp = dict(io)
    for w in writes:
        io[where(w, True)] += w[2]
    for x in inputs:
        inp[where(x, False)] += x[2]
    n = len(main)
    step1 = main[0][1] - io["1"] - inp["1"]
    steady = sum(x for _, x in main[1:]) - io["steady"] - inp["steady"]
    steady_steps = n if overlap else n - 1
    step_s = steady / steady_steps if steady_steps > 0 else None
    compute = step_s * n if step_s is not None else None
    r.update(main=sum(x for _, x in main), io_pre=io["pre"], io_1=io["1"], io_steady=io["steady"], io_post=io["post"],
             io_in=io["1"] + io["steady"], input_1=inp["1"], input_steady=inp["steady"],
             input_in=inp["1"] + inp["steady"], step1=step1, steady=steady, steady_steps=steady_steps,
             stepping=step1 + steady, step_s=step_s, compute=compute)
    if compute is not None:
        r["marginal"] = compute + io["steady"] + io["post"] + inp["steady"]
    if wall is not None:
        r["wall"] = wall
        r["other"] = wall - r["stepping"] - io_all - r["input_in"]
        if compute is not None:
            r["fixed"] = wall - r["marginal"]
    return r


def main():
    a = sys.argv[1:]
    if len(a) < 2 or a[0] not in ("wrf", "cppwrf"):
        print(__doc__)
        sys.exit(0 if a and a[0] in ("-h", "--help") else 2)
    wall = float(a[a.index("--wall") + 1]) if "--wall" in a else None
    print(json.dumps(parse(open(a[1], errors="replace").read(), a[0], wall, "--overlap" in a), indent=1))


if __name__ == "__main__":
    main()
