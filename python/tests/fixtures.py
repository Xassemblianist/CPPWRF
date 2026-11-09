# SPDX-License-Identifier: GPL-3.0-or-later
"""Test fixtures: a small supported namelist (em_quarter_ss), synthetic wrfout files and a fake cppwrf binary."""
import atexit
import datetime
import os
import shutil
import stat
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # the package of this source tree

NAMELIST = """&time_control
 run_minutes = 1, start_year = 2026, start_month = 10, start_day = 12, start_hour = 00, start_minute = 00,
 start_second = 00, end_year = 2026, end_month = 10, end_day = 12, end_hour = 01,
 history_interval_s = 24, frames_per_outfile = 2, restart = .false., restart_interval = 1,
 io_form_history = 2, io_form_restart = 2,
/
&domains
 time_step = 12, max_dom = 1, e_we = 11, e_sn = 11, e_vert = 11, dx = 2000, dy = 2000, ztop = 20000,
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

FAKE = r'''#!/usr/bin/env python3
# a stand-in for cppwrf: --version, then 5 steps of 12 s with WRF's timing lines and one wrfout file
import os, sys, time
a = sys.argv[1:]
if a == ["--version"]:
    print("cppwrf 1.0.0 commit 0000000 backend cpu precision float64"); sys.exit(0)
sets = dict(x.split("=", 1) for x in a[1:] if "=" in x)
wd = sets["output.wrf_dir"]
if os.environ.get("FAKE_FAIL"):
    print("cppwrf: case=em_quarter_ss", flush=True)
    sys.stderr.write("cppwrf: error: something went wrong\n"); sys.exit(3)
open(os.path.join(wd, "wrfout_d01_2026-10-12_00:00:00"), "w").write("x")
print("Timing for Writing wrfout_d01_2026-10-12_00:00:00 for domain        1:    0.01000 elapsed seconds", flush=True)
for n in range(1, 6):
    time.sleep(float(os.environ.get("FAKE_SLEEP", "0.05")))
    print(f"Timing for main: time 2026-10-12_00:{12 * n // 60:02d}:{12 * n % 60:02d} on domain   1:    0.05000 elapsed "
          "seconds", flush=True)
'''


def scratch(prefix):
    """A new directory under CPPWRF_TEST_DIR (default: the system's), removed when the tests end."""
    d = tempfile.mkdtemp(prefix=prefix, dir=os.environ.get("CPPWRF_TEST_DIR"))
    atexit.register(shutil.rmtree, d, True)
    return d


def run_dir(namelist=NAMELIST):
    d = scratch("cppwrf_py_")
    with open(os.path.join(d, "namelist.input"), "w") as f:
        f.write(namelist)
    return d


def fake_binary(d):
    p = os.path.join(d, "fake_cppwrf")
    with open(p, "w") as f:
        f.write(FAKE.replace("#!/usr/bin/env python3", "#!" + sys.executable, 1))
    os.chmod(p, os.stat(p).st_mode | stat.S_IXUSR)
    return p


def column(nz=30, ny=3, nx=4):
    """Hydrostatic columns of the US standard atmosphere's troposphere (6.5 K/km from 288 K and 1013.25 hPa at sea
    level) above terrain 0 to 300 m on a small grid: (height at the w levels, p and T at the mass levels)."""
    zs = np.linspace(0.0, 300.0, nx)[None, :] * np.ones((ny, 1))  # terrain height (m)
    zw = zs[None] + np.linspace(0.0, 15000.0, nz + 1)[:, None, None]
    zm = 0.5 * (zw[1:] + zw[:-1])
    g, rd, gam = 9.81, 287.0, 0.0065
    t = 288.0 - gam * zm
    p = 101325.0 * (t / 288.0) ** (g / (rd * gam))
    return zw, p, t


def write_wrfout(path, times, seed=0, bucket=None, rot=True):
    """A synthetic wrfout file (the variables the diagnostics read) with these frame times."""
    import netCDF4
    rng = np.random.default_rng(seed)
    nz, ny, nx = 30, 3, 4
    zw, p, t = column(nz, ny, nx)
    with netCDF4.Dataset(path, "w") as ds:
        ds.createDimension("Time", None)
        ds.createDimension("DateStrLen", 19)
        for n, k in (("bottom_top", nz), ("bottom_top_stag", nz + 1), ("south_north", ny), ("west_east", nx)):
            ds.createDimension(n, k)
        if bucket:
            ds.setncattr("BUCKET_MM", float(bucket))
        ds.setncattr("TITLE", "synthetic")
        tv = ds.createVariable("Times", "S1", ("Time", "DateStrLen"))
        m3 = ("Time", "bottom_top", "south_north", "west_east")
        m2 = ("Time", "south_north", "west_east")
        v = {k: ds.createVariable(k, "f4", m2) for k in ("XLAT", "XLONG", "T2", "U10", "V10", "COSALPHA",
                                                         "SINALPHA", "RAINC", "RAINNC", "PSFC")}
        v["I_RAINNC"] = ds.createVariable("I_RAINNC", "i4", m2)
        for k in ("P", "PB", "T", "QVAPOR"):
            v[k] = ds.createVariable(k, "f4", m3)
        for k in ("PH", "PHB"):
            v[k] = ds.createVariable(k, "f4", ("Time", "bottom_top_stag", "south_north", "west_east"))
        v["T2"].units = "K"
        for i, tm in enumerate(times):
            tv[i] = netCDF4.stringtochar(np.array([tm.strftime("%Y-%m-%d_%H:%M:%S")], "S19"))
            v["XLAT"][i] = np.linspace(40, 41, ny)[:, None] * np.ones((1, nx))
            v["XLONG"][i] = np.linspace(28, 30, nx)[None, :] * np.ones((ny, 1))
            v["T2"][i] = 290.0 + (tm - datetime.datetime(2026, 10, 12)).total_seconds() / 3600.0
            v["U10"][i] = 3.0
            v["V10"][i] = 4.0
            v["COSALPHA"][i] = 1.0 if not rot else np.cos(0.1)
            v["SINALPHA"][i] = 0.0 if not rot else np.sin(0.1)
            hours = (tm - datetime.datetime(2026, 10, 12)).total_seconds() / 3600.0
            v["RAINC"][i] = 1.0 * hours
            v["RAINNC"][i] = 2.0 * hours
            v["I_RAINNC"][i] = 1 if (bucket and hours >= 2) else 0
            v["PB"][i] = p * 0.98
            v["P"][i] = p * 0.02
            th = t * (100000.0 / p) ** (287.0 / 1004.5)
            v["T"][i] = th - 300.0
            v["QVAPOR"][i] = np.maximum(0.01 * np.exp(-zw[:-1] / 2000.0) + 1e-4 * rng.standard_normal(p.shape), -1e-5)
            v["PHB"][i] = zw * 9.81
            v["PH"][i] = 0.0
            v["PSFC"][i] = p[0]
