#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Runs resumed from restart files with changed output or physics intervals (P1, dt 60 s, a few steps).

- CPPWRF's own restart_<step>.nc continues the run that wrote it, also in WRF's files: the wrfrst files after the
  restart step are the uninterrupted run's (variables and global attributes, START_DATE and the alarms among them);
  with another output.out_interval the history alarm rings at the new interval counted from that run's start.
  With another restart.interval the restart alarm is that of a run resumed from WRF's file (RingTime = the restart
  time + WRF_ALARM_SECS_TIL_NEXT_RING_51, PrevRingTime = the restart time). Another phys.auxhist3_interval with
  output_diagnostics = 1 is refused (CLWRF's sums of the file's interval).
- A run resumed from a wrfrst file computes phy_init's step counts from its own options as wrf.exe does (bldt 2 min,
  radt 2 min: STEPBL = STEPRA = 2 in its next wrfrst file, not the file's 1).

  restart_resume_test.py CPPWRF_BINARY WORK_DIR   (from the repository root; exit 77 without
                                                   ${CPPWRF_WRF_RUNS}/run_P1_r8/steps or without NetCDF)
"""
import glob
import os
import shutil
import subprocess
import sys


def main():
    binary, work = sys.argv[1], sys.argv[2]
    data = os.path.join(os.environ.get("CPPWRF_WRF_RUNS", ""), "run_P1_r8", "steps")
    if not os.path.isfile(os.path.join(data, "wrfinput_d01")):
        print(f"skip: no {data}/wrfinput_d01")
        return 77
    import netCDF4
    fails = 0

    def check(what, got, want):
        nonlocal fails
        print(f"{'ok  ' if got == want else 'FAIL'} {what}: {got}" + ("" if got == want else f", want {want}"))
        fails += got != want

    def run(name, sets, fail=False):
        d = os.path.join(work, name)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
        cmd = [binary, "configs/real_p1_r8.ini", "--set", f"output.dir={d}", "--set", f"real.dir={data}",
               "--set", "output.format=both", "--set", "output.diag_interval=1000"]
        for s in sets:
            cmd += ["--set", s]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if fail:
            return r.returncode, r.stdout + r.stderr
        if r.returncode != 0:
            if "require NetCDF" in r.stdout + r.stderr:
                print("skip: no NetCDF in this build")
                sys.exit(77)
            print(f"FAIL {name}: exit {r.returncode}: {(r.stdout + r.stderr).strip().splitlines()[-1:]}")
            sys.exit(1)
        return d

    def same(fa, fb):
        with netCDF4.Dataset(fa) as a, netCDF4.Dataset(fb) as b:
            a.set_auto_maskandscale(False)
            b.set_auto_maskandscale(False)
            atts = sorted(k for k in set(a.ncattrs()) | set(b.ncattrs())
                          if k not in a.ncattrs() or k not in b.ncattrs() or str(a.getncattr(k)) != str(b.getncattr(k)))
            vs = sorted(n for n in a.variables if n not in b.variables or
                        a.variables[n][:].tobytes() != b.variables[n][:].tobytes())
        return atts, vs

    def times(d, kind):
        out = []
        for f in sorted(glob.glob(os.path.join(d, f"{kind}_d01_*"))):
            with netCDF4.Dataset(f) as ds:
                out += [bytes(t).decode()[11:] for t in ds["Times"][:]]
        return out

    # 4 steps, restart files every 2 (native and wrfrst), resumed from restart_000002.nc
    full = run("full", ["run.run_time=240", "restart.interval=120"])
    rsm = run("native", ["run.run_time=240", "restart.interval=120", f"run.restart_file={full}/restart_000002.nc"])
    f = "wrfrst_d01_2026-10-12_00:04:00"
    check(f"{f} of the native resume: differing attributes, variables", same(f"{full}/{f}", f"{rsm}/{f}"), ([], []))
    # the same resumed with output.out_interval = 2: history at 4 min only (every 2 steps from the start)
    rsm2 = run("native_out2", ["run.run_time=240", "restart.interval=120", "output.out_interval=2",
                               f"run.restart_file={full}/restart_000002.nc"])
    check("native resume with output.out_interval = 2: wrfout times", times(rsm2, "wrfout"), ["00:04:00"])
    for x in (full, rsm, rsm2):  # the restart files of P1 are large
        shutil.rmtree(x, ignore_errors=True)
    # 5 steps with restart.interval 3 min: native file at the last step (5 min, PrevRingTime 3 min, 1 min to the next
    # ring), resumed with restart.interval 1 min up to 8 min. WRF's rule from the restart time: rings at 6, 7, 8 min,
    # 60 s to the next ring each (not the 3-min PrevRingTime with a 1-min interval: -60 s)
    full5 = run("full5", ["run.run_time=300", "restart.interval=180"])
    rsm5 = run("native_iv60", ["run.run_time=480", "restart.interval=60", "restart.native=false",
                               f"run.restart_file={full5}/restart_000005.nc"])
    got = []
    for f in sorted(glob.glob(os.path.join(rsm5, "wrfrst_d01_*"))):
        with netCDF4.Dataset(f) as ds:
            got.append((f[-8:], int(ds.getncattr("WRF_ALARM_SECS_TIL_NEXT_RING_51"))))
    check("native resume with restart.interval = 60 s: (wrfrst time, seconds to the next ring)", got,
          [(f"00:{m:02d}:00", 60) for m in range(6, 9)])
    for x in (full5, rsm5):
        shutil.rmtree(x, ignore_errors=True)
    # output_diagnostics = 1: another auxhist3_interval than the file's is refused
    fd = run("full_diag", ["run.run_time=120", "restart.interval=120", "phys.output_diagnostics=1",
                           "phys.auxhist3_interval=2"])
    rc, out = run("native_aux3", ["run.run_time=240", "restart.interval=120", "phys.output_diagnostics=1",
                                  "phys.auxhist3_interval=3", f"run.restart_file={fd}/restart_000002.nc"], fail=True)
    check("native resume with another auxhist3_interval, output_diagnostics = 1: refused",
          (rc != 0, "phys.auxhist3_interval differs" in out), (True, True))
    shutil.rmtree(fd, ignore_errors=True)
    # wrfrst resume at 1 min with bldt = radt = 2 min: STEPBL, STEPRA from the run's own options
    w = run("wrfrst", ["run.run_time=180", "restart.interval=60", "restart.native=false"])
    r = run("wrfrst_bldt2", ["run.run_time=180", "restart.interval=60", "restart.native=false", "phys.bldt=2",
                             "phys.radt=2", f"run.restart_file={w}/wrfrst_d01_2026-10-12_00:01:00"])
    with netCDF4.Dataset(f"{r}/wrfrst_d01_2026-10-12_00:03:00") as ds:
        check("wrfrst resume with phys.bldt = phys.radt = 2: STEPBL, STEPRA of the next wrfrst file",
              (int(ds["STEPBL"][0]), int(ds["STEPRA"][0])), (2, 2))
    shutil.rmtree(work, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
