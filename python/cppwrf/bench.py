# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-bench: the speed and memory of this machine's CPPWRF builds, compared with docs/PERF.md.

Each mode the installation has (cpu-fp64, gpu-fp64, gpu-fp32; --modes chooses) runs a built-in benchmark: WRF's
em_quarter_ss supercell (Kessler microphysics, no other physics, no input files) through the drop-in wrf.exe, on a
grid of --size points (default 100 x 100 x 40) for --steps steps, and once on a quarter of the horizontal grid for the
memory model of cppwrf-fit. It prints, per mode: ns per cell and step (the model's stepping time), simulated time per
wall time (stepping, and the whole run), peak memory (host RSS; device memory on the GPU) and the ratio to the
reference machine's dynamics-only case (docs/PERF.md, R1). The result is saved for cppwrf-fit
(~/.cache/cppwrf/bench.json; --save FILE, --no-save).

  cppwrf-bench [--modes M,M] [--size NXxNYxNZ] [--steps N] [--threads N] [--json] [--work DIR] [--keep]"""
import argparse
import datetime
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, Optional, Sequence

from . import _locate
from . import dropin
from . import perf_reference as ref
from ._version import __version__

DT = 12.0  # s, the em_quarter_ss time step at dx = 2 km

NAMELIST = """&time_control
 run_seconds = {run_s}, start_year = 0001, start_month = 01, start_day = 01, start_hour = 00, start_minute = 00,
 start_second = 00, end_year = 0001, end_month = 01, end_day = 02,
 history_interval = 100000, frames_per_outfile = 1, restart = .false., restart_interval = 0, io_form_history = 2,
/
&domains
 time_step = 12, max_dom = 1, e_we = {e_we}, e_sn = {e_sn}, e_vert = {e_vert}, dx = 2000, dy = 2000, ztop = 20000,
/
&physics
 mp_physics = 1, ra_lw_physics = 0, ra_sw_physics = 0, sf_sfclay_physics = 0, sf_surface_physics = 0,
 bl_pbl_physics = 0, cu_physics = 0,
/
&dynamics
 hybrid_opt = 0, diff_opt = 1, km_opt = 1, damp_opt = 3, zdamp = 5000., dampcoef = 0.2, khdif = 500, kvdif = 500,
 time_step_sound = 6, non_hydrostatic = .true., moist_adv_opt = 1, scalar_adv_opt = 1, mix_full_fields = .true.,
/
&bdy_control
 periodic_x = .true., periodic_y = .true.,
/
&ideal
 ideal_case = 2,
/
"""


def sounding() -> str:
    """A conditionally unstable sounding in WRF's input_sounding format, deep enough for ztop = 20 km."""
    rows = ["1000.0 300.0 14.0"]
    for k in range(1, 46):
        z = 500.0 * k
        th = 300.0 + (3.0 * z / 1000.0 if z > 12000 else 1.0 * z / 1000.0)
        qv = max(14.0 * (1.0 - z / 12000.0), 0.0)
        rows.append(f"{z:.1f} {th:.3f} {qv:.4f} {min(z / 1000.0, 10.0) - 5.0:.3f} 0.0")
    return "\n".join(rows) + "\n"


def parse_size(s: str):
    m = re.fullmatch(r"(\d+)[xX](\d+)[xX](\d+)", s)
    if not m:
        raise argparse.ArgumentTypeError(f"size '{s}': NXxNYxNZ (cells, e.g. 100x100x40)")
    return tuple(int(x) for x in m.groups())


def run_case(exe: str, mode: str, size, steps: int, threads: Optional[int], work: str) -> Dict:
    """One benchmark run in a new directory under `work`: the model's stepping time and memory."""
    nx, ny, nz = size
    d = tempfile.mkdtemp(prefix=f"bench_{mode}_{nx}x{ny}x{nz}_", dir=work)
    with open(os.path.join(d, "namelist.input"), "w") as f:
        f.write(NAMELIST.format(run_s=int(steps * DT), e_we=nx + 1, e_sn=ny + 1, e_vert=nz + 1))
    with open(os.path.join(d, "input_sounding"), "w") as f:
        f.write(sounding())
    w = dropin.WrfRun(d, mode, threads, exe)
    w.prepare()
    env = dict(w.env, CPPWRF_MEMINFO="1")
    t0 = time.monotonic()
    p = subprocess.Popen(w.cmd, cwd=d, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    with p.stdout:
        out = p.stdout.read()
    _, status, ru = os.wait4(p.pid, 0)
    wall = time.monotonic() - t0
    p.returncode = os.waitstatus_to_exitcode(status)
    with open(os.path.join(d, "bench.log"), "w") as f:
        f.write(out)
    if p.returncode != 0:
        raise RuntimeError(f"{mode} benchmark run failed ({d}/bench.log): {out.strip().splitlines()[-1:]}")
    m = re.search(r"cppwrf: done, (\d+) steps, stepping ([0-9.]+) s, ([0-9.]+) ns/cell/step", out)
    if not m:
        raise RuntimeError(f"{mode}: no 'cppwrf: done' line in {d}/bench.log")
    r = {"size": [nx, ny, nz], "cells": nx * ny * nz, "steps": int(m.group(1)), "stepping_s": float(m.group(2)),
         "ns_per_cell_step": float(m.group(3)), "wall_s": wall, "host_peak_mib": ru.ru_maxrss / 1024.0, "dir": d}
    mem = re.findall(r"device memory ([^:\n]+): (\d+) MiB used of (\d+) MiB; stack limit \d+ bytes x \d+ resident "
                     r"threads \((\d+) MiB\)", out)
    if mem:
        r["device_used_mib"] = max(int(x[1]) for x in mem)
        r["device_total_mib"] = int(mem[0][2])
        r["stack_reserve_mib"] = int(mem[0][3])
    return r


def bench_mode(exe: str, mode: str, size, steps: int, threads: Optional[int], work: str) -> Dict:
    main = run_case(exe, mode, size, steps, threads, work)
    q = (max(size[0] // 2, 10), max(size[1] // 2, 10), size[2])
    small = run_case(exe, mode, q, max(steps // 2, 2), threads, work)
    sim = main["steps"] * DT
    res = {"mode": mode, "exe": exe, "version": (_locate.describe(exe) or {}).get("version"),
           "ns_per_cell_step": main["ns_per_cell_step"], "sim_per_wall_stepping": sim / main["stepping_s"],
           "sim_per_wall_total": sim / main["wall_s"], "host_peak_mib": main["host_peak_mib"], "runs": [main, small]}
    # memory model of cppwrf-fit: memory = a + b * cells, from the two sizes
    key = "device_used_mib" if "device_used_mib" in main else "host_peak_mib"
    dc = main["cells"] - small["cells"]
    b = (main[key] - small[key]) / dc if dc else 0.0
    res["memory_model"] = {"what": key, "a_mib": main[key] - b * main["cells"], "b_mib_per_cell": b}
    for k in ("device_used_mib", "device_total_mib", "stack_reserve_mib"):
        if k in main:
            res[k] = main[k]
    m = "cpu-fp64" if mode == "cpu-fp32" else mode
    r1 = ref.NS_PER_CELL_STEP["R1"][m]
    res["reference_ns_per_cell_step"] = r1
    res["speed_vs_reference"] = r1 / main["ns_per_cell_step"]
    return res


def cache_file() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppwrf", "bench.json")


def machine() -> Dict:
    info = {"host": platform.node(), "cpu": platform.processor() or platform.machine(), "threads": os.cpu_count()}
    try:
        with open("/proc/cpuinfo") as f:
            info["cpu"] = next(line.split(":", 1)[1].strip() for line in f if line.startswith("model name"))
    except (OSError, StopIteration):
        pass
    try:
        g = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=20)
        if g.returncode == 0 and g.stdout.strip():
            info["gpu"] = g.stdout.strip().splitlines()[0]
    except (OSError, subprocess.TimeoutExpired):
        pass
    return info


def report(results: List[Dict], info: Dict) -> str:
    lines = [f"CPPWRF {__version__} benchmark: em_quarter_ss (dynamics + Kessler), {info.get('cpu')}"
             + (f", GPU {info['gpu']}" if info.get("gpu") else ""),
             f"{'mode':9s} {'grid':>12s} {'ns/cell/step':>13s} {'sim/wall':>9s} {'(total)':>8s} {'peak MiB':>9s} "
             f"{'R1 on the G4':>13s} {'vs G4':>7s}"]
    for r in results:
        main = r["runs"][0]
        grid = "x".join(str(x) for x in main["size"])
        peak = r.get("device_used_mib", r["host_peak_mib"])
        lines.append(f"{r['mode']:9s} {grid:>12s} {r['ns_per_cell_step']:>13.2f} {r['sim_per_wall_stepping']:>9.1f} "
                     f"{r['sim_per_wall_total']:>8.1f} {peak:>9.0f} {r['reference_ns_per_cell_step']:>13.1f} "
                     f"{r['speed_vs_reference']:>6.2f}x")
    lines.append("sim/wall: simulated seconds per wall-clock second (stepping only; total: the whole run with start-up "
                 "and output). peak: device memory in use (GPU modes; whole device) or the process's peak RSS (CPU).")
    lines.append(f"vs G4: the speed against docs/PERF.md's dynamics-only case R1 on the {ref.REFERENCE_MACHINE}; "
                 "R1 is a real-data case without physics, so the ratio is indicative (>1: faster).")
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="cppwrf-bench", description=__doc__.split("\n\n")[1].replace("\n", " "))
    ap.add_argument("--modes", help="comma-separated modes (default: every mode installed)")
    ap.add_argument("--size", type=parse_size, default=(100, 100, 40), help="grid cells NXxNYxNZ (default 100x100x40)")
    ap.add_argument("--steps", type=int, default=20, help="time steps of 12 s (default 20)")
    ap.add_argument("--threads", type=int, help="OpenMP threads (default: OMP_NUM_THREADS or all)")
    ap.add_argument("--work", help="directory of the runs (default: a temporary one, deleted)")
    ap.add_argument("--keep", action="store_true", help="keep the runs' directories")
    ap.add_argument("--json", action="store_true", help="print the results as JSON")
    ap.add_argument("--save", default=cache_file(), help="where cppwrf-fit finds the results (default %(default)s)")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--version", action="version", version=f"cppwrf-bench {__version__}")
    a = ap.parse_args(argv)
    found = {}
    for exe in _locate.model_binaries():
        m = _locate.mode_of(exe)
        if m and m not in found:
            found[m] = exe
    modes = a.modes.split(",") if a.modes else [m for m in ("gpu-fp32", "gpu-fp64", "cpu-fp64", "cpu-fp32") if m in found]
    missing = [m for m in modes if m not in found]
    if missing or not modes:
        sys.stderr.write(f"cppwrf-bench: no cppwrf binary for {', '.join(missing) or 'any mode'} (found: "
                         f"{', '.join(found) or 'none'}); see docs/INSTALL.md\n")
        return 2
    work = a.work or tempfile.mkdtemp(prefix="cppwrf_bench_")
    os.makedirs(work, exist_ok=True)
    results = []
    try:
        for m in modes:
            if not a.json:
                print(f"cppwrf-bench: {m} ({found[m]}) ...", flush=True)
            results.append(bench_mode(found[m], m, a.size, a.steps, a.threads, work))
    except RuntimeError as e:
        sys.stderr.write(f"cppwrf-bench: {e}\n")
        return 1
    finally:
        if not a.keep:
            shutil.rmtree(work, ignore_errors=True) if not a.work else [shutil.rmtree(r["dir"], ignore_errors=True)
                                                                       for x in results for r in x["runs"]]
    info = machine()
    doc = {"version": __version__, "date": datetime.datetime.now().isoformat(timespec="seconds"), "machine": info,
           "threads": a.threads or os.environ.get("OMP_NUM_THREADS"), "results": results}
    if not a.no_save:
        os.makedirs(os.path.dirname(os.path.abspath(a.save)), exist_ok=True)
        with open(a.save, "w") as f:
            json.dump(doc, f, indent=1)
    if a.json:
        print(json.dumps(doc, indent=1))
    else:
        print(report(results, info))
        if not a.no_save:
            print(f"saved for cppwrf-fit: {a.save}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
