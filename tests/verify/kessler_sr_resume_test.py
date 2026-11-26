#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Kessler (mp_physics = 1) resumed from a wrfrst file with SR /= 0: SR is 0 after the first step.

WRF's solve_em.F:3852 sets grid%sr = 0. for every mp_physics /= 0 before microphysics_driver; Kessler never sets it
again, so the history SR of a resumed Kessler run is 0 everywhere although the restart file held a nonzero SR (the
step's land surface still reads the file's value). Here: a Kessler run on the P1 data (column physics) writes its
wrfrst at 1 min, SR of a copy is set to 0.5, the run resumed from the copy must write SR = 0 at 2 min.

  kessler_sr_resume_test.py CPPWRF_BINARY WORK_DIR   (from the repository root; exit 77 without
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
    import numpy

    def run(name, sets):
        d = os.path.join(work, name)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
        cmd = [binary, "configs/real_p1_r8.ini", "--set", f"output.dir={d}", "--set", f"real.dir={data}",
               "--set", "output.format=both", "--set", "output.diag_interval=1000", "--set", "phys.mp_physics=1",
               "--set", "moist.species=qv,qc,qr", "--set", "restart.native=false"]
        for s in sets:
            cmd += ["--set", s]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            if "require NetCDF" in r.stdout + r.stderr:
                print("skip: no NetCDF in this build")
                sys.exit(77)
            print(f"FAIL {name}: exit {r.returncode}: {(r.stdout + r.stderr).strip().splitlines()[-3:]}")
            sys.exit(1)
        return d

    w = run("cold", ["run.run_time=60", "restart.interval=60"])
    src = os.path.join(w, "wrfrst_d01_2026-10-12_00:01:00")
    rst = os.path.join(work, "sr_rst", os.path.basename(src))
    os.makedirs(os.path.dirname(rst), exist_ok=True)
    shutil.copy(src, rst)
    with netCDF4.Dataset(rst, "a") as ds:
        ds["SR"][:] = 0.5
    r = run("resumed", ["run.run_time=120", "restart.interval=0", f"run.restart_file={rst}"])
    frame, times = None, []
    for f in sorted(glob.glob(os.path.join(r, "wrfout_d01_*"))):
        with netCDF4.Dataset(f) as ds:
            for n, t in enumerate(ds["Times"][:]):
                times.append(bytes(t).decode())
                if times[-1].endswith("00:02:00"):
                    frame = numpy.asarray(ds["SR"][n])
    fails = 0
    if frame is None:
        print(f"FAIL no 00:02:00 frame in the resumed history ({times})")
        fails += 1
    else:
        got = float(numpy.abs(frame).max())
        print(f"{'ok  ' if got == 0.0 else 'FAIL'} resumed Kessler run, SR at 00:02:00 (wrfrst SR = 0.5): "
              f"max |SR| = {got}, want 0")
        fails += got != 0.0
    shutil.rmtree(work, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
