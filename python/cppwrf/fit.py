# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-fit: will this namelist fit on this machine, and how long will it take? An estimate before the run.

From namelist.input (the domains, their nest ratios, the time step, the run length, the physics and radt) and this
machine's cppwrf-bench results (~/.cache/cppwrf/bench.json; run cppwrf-bench first), per mode:
  - memory: the benchmark's memory per cell (host RSS on the CPU, device memory on the GPU) times the cells of all
    domains, with the full physics' larger arrays and GPU stack reserve scaled from docs/PERF.md;
  - time: the benchmark's ns per cell and step times the cells and steps of each domain, with the physics' cost
    relative to the dynamics (docs/PERF.md: P4 radiation every step, P4R15 every 15th, interpolated by radt);
and whether the memory fits: in the GPU's memory, or in the host's available memory (MemAvailable) on the CPU.
docs/TOOLS.md states the accuracy measured.

  cppwrf-fit [namelist.input] [--modes M,M] [--bench FILE] [--json]
  cppwrf-fit --grid NXxNYxNZ [--steps N] [--physics none|full] [--radt MIN]   (no namelist)"""
import argparse
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence

from . import bench
from . import dropin
from . import namelist as nl
from . import perf_reference as ref
from ._version import __version__

# options that make a step "full physics" in the sense of docs/PERF.md's P4 (any of them nonzero)
PHYSICS_KEYS = ("ra_lw_physics", "ra_sw_physics", "bl_pbl_physics", "sf_sfclay_physics", "sf_surface_physics",
                "cu_physics")
FITS, TIGHT = 0.85, 1.0  # fraction of the memory: below FITS "fits", below TIGHT "tight", else "does not fit"


@dataclass
class Domain:
    nx: int
    ny: int
    nz: int
    dt: float  # s
    steps: int
    physics: bool
    radt: float  # min (0: no radiation)

    @property
    def cells(self) -> int:
        return self.nx * self.ny * self.nz


@dataclass
class Estimate:
    mode: str
    time_s: float
    stepping_s: float
    memory_mib: float
    memory_kind: str  # "device" or "host"
    available_mib: Optional[float]
    verdict: str
    notes: List[str] = field(default_factory=list)


def time_factor(mode: str, d: Domain) -> float:
    """Step cost of domain d over the dynamics-only R1's: 1 without physics; with physics, the line through P4
    (radiation every step at dt 60 s and radt 1 min) and P4R15 (every 15th step) in the fraction of steps that call
    the radiation."""
    if not d.physics:
        return 1.0
    f1, f15 = ref.physics_time_factor(mode, "P4"), ref.physics_time_factor(mode, "P4R15")
    b = (f1 - f15) / (1.0 - 1.0 / 15.0)
    a = f1 - b
    r = 0.0 if d.radt <= 0 else 1.0 / max(1, round(d.radt * 60.0 / d.dt))
    return a + b * r


def domains_from_namelist(nml) -> List[Domain]:
    g = lambda grp, key, default=None, dom=0: nl.get(nml, grp, key, default, dom)  # noqa: E731
    n = int(g("domains", "max_dom", 1))
    dt = float(g("domains", "time_step", 60)) + float(g("domains", "time_step_fract_num", 0) or 0) / float(
        g("domains", "time_step_fract_den", 1) or 1)
    run_s = (float(g("time_control", "run_days", 0) or 0) * 86400 + float(g("time_control", "run_hours", 0) or 0) * 3600
             + float(g("time_control", "run_minutes", 0) or 0) * 60 + float(g("time_control", "run_seconds", 0) or 0))
    if run_s <= 0:
        start = nl.start_date(nml)
        e = [int(g("time_control", f"end_{k}", d) or d) for k, d in
             (("year", 1), ("month", 1), ("day", 1), ("hour", 0), ("minute", 0), ("second", 0))]
        end = f"{e[0]:04d}-{e[1]:02d}-{e[2]:02d}_{e[3]:02d}:{e[4]:02d}:{e[5]:02d}"
        run_s = dropin.seconds_between(start, end)
    out: List[Domain] = []
    ratio = {1: 1}
    for i in range(n):
        dom = i + 1
        parent = int(g("domains", "parent_id", 1, i) or 1) if i else 0
        r = int(g("domains", "parent_time_step_ratio", 1, i) or 1) if i else 1
        ratio[dom] = ratio.get(parent, 1) * r if i else 1
        phys = any(int(g("physics", k, 0, i) or 0) > 0 for k in PHYSICS_KEYS) or bool(g("physics", "physics_suite", ""))
        rad = int(g("physics", "ra_lw_physics", 0, i) or 0) > 0 or int(g("physics", "ra_sw_physics", 0, i) or 0) > 0
        if g("physics", "physics_suite", ""):
            rad = rad or int(g("physics", "ra_lw_physics", -1, i)) == -1
        radt = float(g("physics", "radt", 30, i) or 30) if rad else 0.0
        ddt = dt / ratio[dom]
        out.append(Domain(int(g("domains", "e_we", 0, i)) - 1, int(g("domains", "e_sn", 0, i)) - 1,
                          int(g("domains", "e_vert", 0, i)) - 1, ddt, int(round(run_s / ddt)), phys, radt))
    return out


def mem_available_mib() -> Optional[float]:
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return None


def estimate(r: Dict, domains: Sequence[Domain]) -> Estimate:
    """The estimate of one mode from its cppwrf-bench result `r`."""
    mode = r["mode"]
    notes = []
    m = "cpu-fp64" if mode == "cpu-fp32" else mode
    stepping = sum(d.cells * d.steps * r["ns_per_cell_step"] * time_factor(m, d) for d in domains) * 1e-9
    main = r["runs"][0]
    startup = max(main["wall_s"] - main["stepping_s"], 0.0)
    mm = r["memory_model"]
    physics = any(d.physics for d in domains)
    pmf = ref.physics_memory_factor(m) if physics else 1.0
    cells = sum(d.cells for d in domains)
    memory = mm["a_mib"] + mm["b_mib_per_cell"] * cells * pmf
    if mode.startswith("gpu"):
        kind, avail = "device", r.get("device_total_mib")
        if physics and "stack_reserve_mib" in r:
            memory += r["stack_reserve_mib"] * (ref.stack_reserve_factor(mode) - 1.0)
        # the whole device's memory in use was measured: other processes' memory is in "a"
    else:
        kind, avail = "host", mem_available_mib()
        if physics:
            notes.append("CPU physics memory scaled with the GPU FP64 factor (not measured on the CPU)")
    if any(d.physics for d in domains):
        notes.append("physics cost from docs/PERF.md's P4 suite (KF, WSM6, YSU, Noah, RRTMG); other schemes differ")
    if avail is None:
        verdict = "unknown"
    else:
        verdict = "fits" if memory < FITS * avail else ("tight" if memory < TIGHT * avail else "does not fit")
    return Estimate(mode, stepping + startup, stepping, memory, kind, avail, verdict, notes)


def load_bench(path: str) -> Dict:
    with open(path) as f:
        return json.load(f)


def fmt_time(s: float) -> str:
    if s < 120:
        return f"{s:.1f} s"
    if s < 7200:
        return f"{s / 60:.1f} min"
    return f"{s / 3600:.1f} h"


def report(domains: Sequence[Domain], ests: Sequence[Estimate], doc: Dict) -> str:
    lines = []
    for i, d in enumerate(domains):
        lines.append(f"d{i + 1:02d}: {d.nx} x {d.ny} x {d.nz} = {d.cells / 1e6:.2f} M cells, {d.steps} steps of "
                     f"{d.dt:g} s, " + (f"physics, radt {d.radt:g} min" if d.radt else
                                        ("physics" if d.physics else "no physics besides microphysics")))
    lines.append(f"(cppwrf-bench of {doc.get('date', '?')} on {doc.get('machine', {}).get('host', '?')})")
    lines.append(f"{'mode':9s} {'run time':>10s} {'memory MiB':>11s} {'of MiB':>9s}  verdict")
    for e in ests:
        avail = f"{e.available_mib:.0f}" if e.available_mib else "?"
        lines.append(f"{e.mode:9s} {fmt_time(e.time_s):>10s} {e.memory_mib:>11.0f} {avail:>9s}  {e.verdict} "
                     f"({e.memory_kind})")
    for n in sorted({n for e in ests for n in e.notes}):
        lines.append(f"note: {n}")
    lines.append("history output, input reading and restart files add to the time; see docs/TOOLS.md for the accuracy")
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="cppwrf-fit", description=__doc__.split("\n\n")[0])
    ap.add_argument("namelist", nargs="?", default="namelist.input")
    ap.add_argument("--grid", type=bench.parse_size, help="one domain of NXxNYxNZ cells instead of a namelist")
    ap.add_argument("--steps", type=int, default=60, help="with --grid: time steps (default 60)")
    ap.add_argument("--dt", type=float, default=60.0, help="with --grid: time step in s (default 60)")
    ap.add_argument("--physics", choices=("none", "full"), default="full", help="with --grid (default full)")
    ap.add_argument("--radt", type=float, default=15.0, help="with --grid and full physics: radt in min (default 15)")
    ap.add_argument("--modes", help="comma-separated modes (default: every mode benchmarked)")
    ap.add_argument("--bench", default=bench.cache_file(), help="cppwrf-bench's results (default %(default)s)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--version", action="version", version=f"cppwrf-fit {__version__}")
    a = ap.parse_args(argv)
    try:
        doc = load_bench(a.bench)
    except (OSError, ValueError) as e:
        sys.stderr.write(f"cppwrf-fit: no benchmark results ({e}); run cppwrf-bench first\n")
        return 2
    if a.grid:
        phys = a.physics == "full"
        domains = [Domain(*a.grid, a.dt, a.steps, phys, a.radt if phys else 0.0)]
    else:
        try:
            domains = domains_from_namelist(nl.read(a.namelist))
        except (OSError, ValueError, TypeError) as e:
            sys.stderr.write(f"cppwrf-fit: {a.namelist}: {e}\n")
            return 2
    results = doc.get("results", [])
    if a.modes:
        want = a.modes.split(",")
        results = [r for r in results if r["mode"] in want]
        missing = set(want) - {r["mode"] for r in results}
        if missing:
            sys.stderr.write(f"cppwrf-fit: no benchmark of {', '.join(sorted(missing))} in {a.bench}\n")
            return 2
    ests = [estimate(r, domains) for r in results]
    if a.json:
        print(json.dumps({"domains": [asdict(d) | {"cells": d.cells} for d in domains],
                          "estimates": [asdict(e) for e in ests]}, indent=1))
    else:
        print(report(domains, ests, doc))
    return 0 if any(e.verdict in ("fits", "tight") for e in ests) else 1


if __name__ == "__main__":
    sys.exit(main())
