#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""auxinput4_interval (sst_update = 1) and gfdda_interval_m (grid_fdda = 1) that are not multiples of the time step.

WRF's AUXINPUT4 / AUXINPUT10 alarm rings at the first step at or past each interval (ESMF_ClockAdvance); input_wrf
then finds no wrflowinp / wrffdda record of that time and stops ("Could not find matching time in input file",
share/input_wrf.F). The model refuses such a configuration at its start; the same configuration with an interval that
is a multiple of dt runs. dt = 32 s divides the boundary interval (21600 s) but not 180 min (10800 s); 360 min is a
multiple of it.

  input_interval_test.py CPPWRF_BINARY WORK_DIR   (from the repository root; exit 77 without
                                                   ${CPPWRF_WRF_RUNS}/run_SU1_r8/steps; the grid_fdda run that goes on
                                                   needs ${CPPWRF_WRF_RUNS}/run_FD1_r8/steps)
"""
import os
import subprocess
import sys


def main():
    binary, work = sys.argv[1], sys.argv[2]
    runs = os.environ.get("CPPWRF_WRF_RUNS", "")
    if not os.path.isfile(os.path.join(runs, "run_SU1_r8", "steps", "wrflowinp_d01")):
        print(f"skip: no {runs}/run_SU1_r8/steps/wrflowinp_d01")
        return 77
    os.makedirs(work, exist_ok=True)
    fails = 0

    def run(name, cfg, sets, want):
        """want: the error message's start, or None: the run must succeed (one step)."""
        nonlocal fails
        cmd = [binary, cfg, "--set", f"output.dir={os.path.join(work, name)}", "--set", "time.dt=32",
               "--set", "run.run_time=32"]
        for s in sets:
            cmd += ["--set", s]
        r = subprocess.run(cmd, capture_output=True, text=True)
        out = (r.stdout + r.stderr).strip().splitlines()
        last = out[-1] if out else ""
        if "require NetCDF" in last or "needs NetCDF" in last:
            print("skip: no NetCDF in this build")
            sys.exit(77)
        if want is None:
            good = r.returncode == 0
        else:
            good = r.returncode != 0 and any(want in x for x in out)
        print(f"{'ok  ' if good else 'FAIL'} {name}: exit {r.returncode}: {last[:240]}")
        fails += not good

    su1 = "configs/real_su1_r8.ini"
    run("sst_update_180min", su1, ["phys.auxinput4_interval=180"],
        "phys.auxinput4_interval (10800 s) is not a multiple of time.dt (32 s)")
    run("sst_update_360min", su1, [], None)
    # grid_fdda: the check comes before the model opens wrffdda_d01, so SU1's inputs serve; the run that goes on needs
    # FD1's analyses
    fdda = ["fdda.grid_fdda=1", "fdda.gfdda_end_h=6", "fdda.guv=0.0003"]
    run("grid_fdda_180min", su1, fdda + ["fdda.gfdda_interval_m=180"],
        "fdda.gfdda_interval_m (10800 s) is not a multiple of time.dt (32 s)")
    if os.path.isfile(os.path.join(runs, "run_FD1_r8", "steps", "wrffdda_d01")):
        run("grid_fdda_360min", "configs/real_fd1_r8.ini", [], None)
    else:
        print(f"note: no {runs}/run_FD1_r8/steps/wrffdda_d01, the grid_fdda run with 360 min left out")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
