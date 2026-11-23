#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""CPPWRF's own restart file is read only in the shape its format version writes.

A tiny em_quarter_ss (11 x 11 cells, dt 12 s) writes restart_000002.nc and is resumed from it, in one patch and in
2 x 2 patches:
 - the intact file resumes (and no restart_<step>.nc.partial of the writer stays behind);
 - a HALO_<name> variable repacked to rank 2 (Time, halo_<n>), with the same number of values, is refused before
   anything is read (the reader checks rank, dimension name and length; no buffer overrun);
 - a file without one of its HALO_<name> variables is refused (format version 7 has them for every field);
 - a file of format version 6 is refused with a pointer to the run's wrfrst file.

  restart_guard_test.py CPPWRF_BINARY WORK_DIR   (from the repository root; exit 77 without
                                                  ${CPPWRF_WRF_REF}/run_qss/input_sounding or without NetCDF)
"""
import glob
import os
import shutil
import subprocess
import sys


def copy_nc(src, dst, edit_var=None, drop=(), atts=None):
    """src to dst (netCDF4, the same format), every dimension, variable and attribute; edit_var(name): None to copy,
    else (dims, data) of a replacement; drop: variables left out; atts: global attributes to overwrite."""
    import netCDF4
    with netCDF4.Dataset(src) as a, netCDF4.Dataset(dst, "w", format=a.data_model) as b:
        a.set_auto_maskandscale(False)
        for n, d in a.dimensions.items():
            b.createDimension(n, None if d.isunlimited() else len(d))
        b.setncatts({k: a.getncattr(k) for k in a.ncattrs()})
        if atts:
            b.setncatts(atts)
        for n, v in a.variables.items():
            if n in drop:
                continue
            rep = edit_var(n) if edit_var else None
            if rep is None:
                dims, data = v.dimensions, v[:]
            else:
                dims, data = rep
                for dn, dl in zip(dims, data.shape):
                    if dn not in b.dimensions:
                        b.createDimension(dn, dl)
            w = b.createVariable(n, v.dtype, dims)
            w.setncatts({k: v.getncattr(k) for k in v.ncattrs()})
            w[:] = data


def main():
    binary, work = sys.argv[1], sys.argv[2]
    sounding = os.path.join(os.environ.get("CPPWRF_WRF_REF", ""), "run_qss", "input_sounding")
    if not os.path.isfile(sounding):
        print(f"skip: no {sounding}")
        return 77
    try:
        import netCDF4
    except ImportError:
        print("skip: no netCDF4 module")
        return 77
    fails = 0

    def check(what, ok, detail=""):
        nonlocal fails
        print(f"{'ok  ' if ok else 'FAIL'} {what}" + ("" if ok else f": {detail}"))
        fails += not ok

    base = [binary, "configs/wrf_quarter_ss.ini", "--set", "domain.nx=11", "--set", "domain.ny=11",
            "--set", "output.format=wrfout", "--set", "output.out_interval=1000", "--set", "output.diag_interval=1000"]

    def run(name, extra):
        d = os.path.join(work, name)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
        r = subprocess.run(base + ["--set", f"output.dir={d}"] + extra, capture_output=True, text=True)
        out = r.stdout + r.stderr
        if r.returncode != 0 and "require NetCDF" in out:
            print("skip: no NetCDF in this build")
            sys.exit(77)
        return d, r.returncode, out

    d, rc, out = run("full", ["--set", "run.run_time=48", "--set", "restart.interval=24"])
    if rc != 0:
        print(f"FAIL uninterrupted run: exit {rc}: {out.strip().splitlines()[-1:]}")
        return 1
    rst = os.path.join(d, "restart_000002.nc")
    check("the writer's restart_000002.nc exists", os.path.isfile(rst), sorted(os.listdir(d)))
    check("no .partial file stays behind", not glob.glob(os.path.join(d, "*.partial")), glob.glob(os.path.join(d, "*")))
    with netCDF4.Dataset(rst) as nc:
        check("CPPWRF_RESTART_VERSION 7", int(nc.getncattr("CPPWRF_RESTART_VERSION")) == 7)
        halo = nc["HALO_mu_2"]
        check("HALO_mu_2 is of rank 1", len(halo.dimensions) == 1, halo.dimensions)
        hdim = halo.dimensions[0]

    rank2 = os.path.join(work, "rank2.nc")
    copy_nc(rst, rank2, edit_var=lambda n: (("Time", hdim), netCDF4.Dataset(rst)[n][:].reshape(1, -1))
            if n == "HALO_mu_2" else None)
    nohalo = os.path.join(work, "nohalo.nc")
    copy_nc(rst, nohalo, drop=("HALO_t_2",))
    v6 = os.path.join(work, "v6.nc")
    copy_nc(rst, v6, atts={"CPPWRF_RESTART_VERSION": 6})

    resume = ["--set", "run.run_time=48", "--set", "restart.interval=0"]
    for tiles in ("1x1", "2x2"):
        lay = ["--set", f"run.tiles={tiles}"]
        _, rc, out = run(f"ok_{tiles}", resume + lay + ["--set", f"run.restart_file={rst}"])
        check(f"{tiles}: the intact file resumes", rc == 0, out.strip().splitlines()[-3:])
        _, rc, out = run(f"rank2_{tiles}", resume + lay + ["--set", f"run.restart_file={rank2}"])
        check(f"{tiles}: a rank-2 HALO_mu_2 is refused", rc != 0 and "HALO_mu_2 has rank 2, expected 1" in out,
              out.strip().splitlines()[-3:])
        _, rc, out = run(f"nohalo_{tiles}", resume + lay + ["--set", f"run.restart_file={nohalo}"])
        check(f"{tiles}: a file without HALO_t_2 is refused", rc != 0 and "HALO_t_2 is missing" in out,
              out.strip().splitlines()[-3:])
    _, rc, out = run("v6", resume + ["--set", f"run.restart_file={v6}"])
    check("format version 6 is refused", rc != 0 and "CPPWRF restart format version 6" in out and
          "resume from the run's wrfrst file" in out, out.strip().splitlines()[-3:])
    if not fails:
        shutil.rmtree(work, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
