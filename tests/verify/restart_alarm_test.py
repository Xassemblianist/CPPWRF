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
of 24 s and compares the times of the wrfrst files, and the seconds to the next ring they carry, with that rule. The
run resumed from CPPWRF's own restart_000002.nc continues the uninterrupted run: the same rings and wrfrst files
(variables and global attributes); with another restart interval it rings as a run resumed from the wrfrst file.
A resumed interval shorter than the file's seconds to the next ring (12 s) rings first and writes interval - (time -
PrevRingTime). A resumed run dates its files from the wrfrst file's SIMULATION_START_DATE (run.start_date = the
restart time, the namelist start of WRF's resumed run, changes nothing), and STEPRA is ra_init's in the build's REAL.

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
    # a restart interval shorter than the file's seconds to the next ring (12 s < 16 s): the interval rings before the
    # pending RingTime (ESMF_ClockAdvance's PRED3: PrevRingTime += interval) and output_wrf writes interval - (time -
    # PrevRingTime), it does not read RingTime
    _, got = run("resumed_iv12", ["--set", f"run.restart_file={first[0]}", "--set", "restart.interval=12"])
    check(f"run resumed from {t0} s with restart.interval = 12", got, wrf_rings(dt, 12, t_end, t0, secs))
    # the namelist's start of a resumed run is the restart time: the files' dates count from the simulation start of
    # the wrfrst file (input_wrf), whatever run.start_date says
    d2, got = run("resumed_start", ["--set", f"run.restart_file={first[0]}",
                                    "--set", f"run.start_date=0001-01-01_00:00:{t0:02d}"])
    check(f"run resumed from {t0} s with run.start_date = the restart time", got, wrf_rings(dt, iv, t_end, t0, secs))
    import netCDF4
    starts = sorted({netCDF4.Dataset(f).getncattr("SIMULATION_START_DATE")
                     for f in glob.glob(os.path.join(d2, "wrf*_d01_*"))})
    check("its SIMULATION_START_DATE", starts, ["0001-01-01_00:00:00"])
    # ra_init's STEPRA = max(nint(radt*60./dt), 1) in WRF's REAL: radt = 2.1 min, dt = 12 s is 10.499999 in real*4
    # (10), 10.500000000000002 in real*8 (11)
    prec = subprocess.run([binary, "--version"], capture_output=True, text=True).stdout
    d3, _ = run("stepra", ["--set", "phys.radt=2.1", "--set", f"run.run_time={4 * dt}"])
    with netCDF4.Dataset(sorted(glob.glob(os.path.join(d3, "wrfrst_d01_*")))[0]) as nc:
        check("STEPRA of radt = 2.1 min, dt = 12 s", int(nc["STEPRA"][0]), 10 if "float32" in prec else 11)
    # restart_interval = 0 in the resumed run: the alarm keeps the file's RingTime and rings once (WRF 4.8.0: the run
    # resumed from 150 s with restart_interval = 0 writes one wrfrst file, of 250 s, with 1700000000 seconds to go)
    _, got = run("resumed_iv0", ["--set", f"run.restart_file={first[0]}", "--set", "restart.interval=0"])
    check(f"run resumed from {t0} s with restart.interval = 0", got, wrf_rings(dt, 0, t_end, t0, secs))
    # CPPWRF's own restart file of that time carries the alarm as the run has it (PrevRingTime): the resumed run
    # rings where the uninterrupted one rings, and its wrfrst files are the uninterrupted run's (every variable and
    # global attribute: START_DATE and alarm 55 count from the start of the run that wrote the file)
    native = os.path.join(d, f"restart_{t0 // dt:06d}.nc")
    dn, got = run("resumed_native", ["--set", f"run.restart_file={native}"])
    check(f"run resumed from {t0} s (CPPWRF's restart file), (time, seconds to the next ring)", got,
          [x for x in want if x[0] > t0])
    # with another restart.interval (30 s, 12 s, 0) the file's PrevRingTime is of no use: the alarm of a run resumed
    # from WRF's file, from the restart time and the file's seconds to the next ring
    for iv2 in (30, 12, 0):
        _, got = run(f"resumed_native_iv{iv2}", ["--set", f"run.restart_file={native}", "--set", f"restart.interval={iv2}"])
        check(f"run resumed from {t0} s (CPPWRF's restart file) with restart.interval = {iv2}", got,
              wrf_rings(dt, iv2, t_end, t0, secs))
    for f in sorted(glob.glob(os.path.join(dn, "wrfrst_d01_*"))):
        with netCDF4.Dataset(f) as b, netCDF4.Dataset(os.path.join(d, os.path.basename(f))) as a:
            a.set_auto_maskandscale(False)
            b.set_auto_maskandscale(False)
            atts = sorted(k for k in set(a.ncattrs()) | set(b.ncattrs())
                          if k not in a.ncattrs() or k not in b.ncattrs() or str(a.getncattr(k)) != str(b.getncattr(k)))
            vs = sorted(n for n in a.variables if n not in b.variables or
                        a.variables[n][:].tobytes() != b.variables[n][:].tobytes())
            check(f"{os.path.basename(f)} of the native resume: differing attributes, variables", (atts, vs), ([], []))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
