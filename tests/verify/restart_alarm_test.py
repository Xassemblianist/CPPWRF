#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The restart alarm of a resumed run, with a restart interval that is not a multiple of the time step.

WRF (external/esmf_time_f90/ESMF_Clock.F90, ESMF_ClockAdvance; share/input_wrf.F, restart_only):
 - an uninterrupted run rings at the first step with PrevRingTime + interval <= time, and then adds the interval to
   PrevRingTime (the rings stay on the multiples of the interval);
 - a run resumed from a restart file sets RingTime = restart time + WRF_ALARM_SECS_TIL_NEXT_RING_51 (output_wrf.F:
   interval - (time - PrevRingTime)); the first ring is the step with RingTime <= time < RingTime + dt, and it sets
   PrevRingTime to that step's time, so the later rings are counted from it, not from the multiples.
WRF 4.8.0 (em_real, time_step 50 s, restart_interval 2 min, 700 s): restart files at 150, 250, 400, 500, 600 s; the
run resumed from the file of 150 s: 250, 400, 500, 650 s (WRF_ALARM_SECS_TIL_NEXT_RING_51 = 120, 90, 110, 80); resumed
with restart_interval = 0: one file, of 250 s (1700000000: the forever interval).

The test runs em_quarter_ss (11 x 11 cells, dt 12 s, restart interval 20 s, 144 s), resumes it from its wrfrst file
of 24 s and from its own restart_000002.nc, and compares the times of the wrfrst files of the three runs, and the
seconds to the next ring they carry, with that rule.

  restart_alarm_test.py CPPWRF_BINARY WORK_DIR     (from the repository root; exit 77 without
                                                    ${CPPWRF_WRF_REF}/run_qss/input_sounding or without NetCDF)
"""
import glob
import os
import shutil
import subprocess
import sys


def wrf_rings(dt, iv, t_end, t0=0, secs_to_ring=None):
    """(time, seconds to the next ring written at that time) of WRF's restart alarm from t0 to t_end; secs_to_ring:
    the WRF_ALARM_SECS_TIL_NEXT_RING_51 of the restart file a resumed run starts from."""
    prev, cur, out = t0, t0, []
    ring_time = None if secs_to_ring is None else t0 + secs_to_ring
    while cur + dt <= t_end:
        cur += dt
        if ring_time is not None and ring_time <= cur < ring_time + dt:
            prev, ring_time = cur, None
        elif iv > 0 and prev + iv <= cur:
            prev += iv
        else:
            continue
        # restart_interval = 0: the forever interval (1700000000 s, written unchanged)
        out.append((cur, iv - (cur - prev) if iv > 0 else 1700000000))
    return out


def main():
    binary, work = sys.argv[1], sys.argv[2]
    fails = 0

    def check(what, got, want):
        nonlocal fails
        if got == want:
            print(f"ok   {what}: {got}")
        else:
            fails += 1
            print(f"FAIL {what}: {got}, WRF: {want}")

    # the rule against WRF 4.8.0's own files
    check("rule, uninterrupted (WRF 4.8.0)", wrf_rings(50, 120, 700), [(150, 90), (250, 110), (400, 80), (500, 100), (600, 120)])
    check("rule, resumed (WRF 4.8.0)", wrf_rings(50, 120, 700, 150, 90), [(250, 120), (400, 90), (500, 110), (650, 80)])
    check("rule, resumed, restart_interval = 0 (WRF 4.8.0)", wrf_rings(50, 0, 700, 150, 90), [(250, 1700000000)])

    sounding = os.path.join(os.environ.get("CPPWRF_WRF_REF", ""), "run_qss", "input_sounding")
    if not os.path.isfile(sounding):
        print(f"skip: no {sounding}")
        return 77
    dt, iv, t_end = 12, 20, 144
    base = [binary, "configs/wrf_quarter_ss.ini", "--set", "domain.nx=11", "--set", "domain.ny=11",
            "--set", f"run.run_time={t_end}", "--set", f"restart.interval={iv}",
            "--set", "output.format=wrfout", "--set", "output.out_interval=1000", "--set", "output.diag_interval=1000"]

    def run(name, extra):
        d = os.path.join(work, name)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(work, exist_ok=True)
        r = subprocess.run(base + ["--set", f"output.dir={d}"] + extra, capture_output=True, text=True)
        if r.returncode != 0:
            if "require NetCDF" in r.stdout + r.stderr:
                print("skip: no NetCDF in this build")
                sys.exit(77)
            print(f"FAIL {name}: exit {r.returncode}: {(r.stdout + r.stderr).strip().splitlines()[-1:]}")
            sys.exit(1)
        import netCDF4
        out = []
        for f in sorted(glob.glob(os.path.join(d, "wrfrst_d01_*"))):
            h, m, s = (int(x) for x in f[-8:].split(":"))
            with netCDF4.Dataset(f) as nc:
                out.append((h * 3600 + m * 60 + s, int(nc.getncattr("WRF_ALARM_SECS_TIL_NEXT_RING_51"))))
        return d, out

    d, got = run("uninterrupted", [])
    want = wrf_rings(dt, iv, t_end)
    check("uninterrupted run, (time, seconds to the next ring)", got, want)
    t0, secs = want[0]
    first = glob.glob(os.path.join(d, f"wrfrst_d01_*_00:00:{t0:02d}"))
    if not first:
        print(f"FAIL no restart file of {t0} s to resume from")
        return 1
    _, got = run("resumed", ["--set", f"run.restart_file={first[0]}"])
    check(f"run resumed from {t0} s, (time, seconds to the next ring)", got, wrf_rings(dt, iv, t_end, t0, secs))
    # restart_interval = 0 in the resumed run: the alarm keeps the file's RingTime and rings once (WRF 4.8.0: the run
    # resumed from 150 s with restart_interval = 0 writes one wrfrst file, of 250 s, with 1700000000 seconds to go)
    _, got = run("resumed_iv0", ["--set", f"run.restart_file={first[0]}", "--set", "restart.interval=0"])
    check(f"run resumed from {t0} s with restart.interval = 0", got, wrf_rings(dt, 0, t_end, t0, secs))
    # CPPWRF's own restart file of that time carries the same WRF_ALARM_SECS_TIL_NEXT_RING_51: the same schedule
    native = os.path.join(d, f"restart_{t0 // dt:06d}.nc")
    _, got = run("resumed_native", ["--set", f"run.restart_file={native}"])
    check(f"run resumed from {t0} s (CPPWRF's restart file), (time, seconds to the next ring)", got,
          wrf_rings(dt, iv, t_end, t0, secs))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
