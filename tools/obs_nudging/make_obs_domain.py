#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""make_obs_domain.py WRFINPUT OUT [--start YYYY-MM-DD_hh:mm:ss] [--scale F]

A small synthetic observation file for WRF's observation nudging (obs_nudge_opt = 1) in the exact format wrf.exe's
in4dob reads (share/wrf_fddaobs_in.F, OBS_DOMAIN<N>01: one header of 4 records per report, then one record per level
of a sounding or one surface record; formats (1x,a14), (2x,2(f9.4,1x)), (2x,2(a40,3x)),
(2x,2(a16,2x),f8.0,2x,2(l4,2x),i5), (1x,6(f11.3,1x,f11.3,1x)) and (1x,9(f11.3,1x,f11.3,1x))).

The reports lie inside the domain of WRFINPUT (positions given in mass-grid coordinates, their latitude and longitude
from the file's XLAT / XLONG), at times from 1 h before the start to 2 h after it in chronological order, with values
near the initial state of WRFINPUT (so that the temperature innovations pass errob's gross error check, except for
one report made to fail it). They exercise the branches CPPWRF ports: METAR / SYNOP / SHIP surface reports with the
surface pressure given, from the sea-level pressure or from the temperature only; a SHIP report with RH < 70 %; wind
QC 129 (rotated to the grid); a duplicate surface report (replaces the first); a SYNOP PRET METNET= 3 report and a
report outside the domain (both dropped); a report older than the window (dropped); TEMP soundings with a level
without temperature, a level with a height but no pressure (errob's ht_to_p) and a level with wind QC 256; a PILOT
sounding (no temperature); single-level upper-air reports (SATWNDS, ACARS) well above the PBL, a duplicate of one;
a dry surface report (negative moisture innovation, obs_scl_neg_qv_innov). The file is deterministic (no random
numbers): the committed tests/data/obs/OBS_DOMAIN101_P4 is this script's output for run_P4_r8's wrfinput_d01.
"""
import argparse
import datetime

import numpy as np
from netCDF4 import Dataset

M8 = -888888.0
RD, CP, G = 287.0, 1004.5, 9.81


def f11(v):
    return f"{v:11.3f}"


class Domain:
    def __init__(self, path):
        nc = Dataset(path)
        g = lambda n: np.array(nc.variables[n][0], dtype=np.float64)  # noqa: E731
        self.xlat, self.xlong = g("XLAT"), g("XLONG")
        self.p = g("P") + g("PB")
        self.t = (g("T") + 300.0) * (self.p / 1.0e5) ** (RD / CP)
        self.qv = g("QVAPOR")
        u, v = g("U"), g("V")
        self.u = 0.5 * (u[:, :, 1:] + u[:, :, :-1])
        self.v = 0.5 * (v[:, 1:, :] + v[:, :-1, :])
        zw = (g("PH") + g("PHB")) / G
        self.z = 0.5 * (zw[1:] + zw[:-1])
        self.hgt, self.psfc = g("HGT"), g("PSFC")
        self.start = nc.getncattr("START_DATE")
        self.ny, self.nx = self.xlat.shape
        nc.close()

    def at(self, a, i, j):
        """bilinear value of a 2-D field (or of each level of a 3-D one) at mass-grid position (i, j), 1-based"""
        i0, j0 = int(i) - 1, int(j) - 1
        fx, fy = i - int(i), j - int(j)
        b = a[..., j0:j0 + 2, i0:i0 + 2]
        return ((1 - fy) * ((1 - fx) * b[..., 0, 0] + fx * b[..., 0, 1]) +
                fy * ((1 - fx) * b[..., 1, 0] + fx * b[..., 1, 1]))

    def latlon(self, i, j):
        return round(float(self.at(self.xlat, i, j)), 4), round(float(self.at(self.xlong, i, j)), 4)


def rh_of(t, p, qv):
    """the relative humidity (%) whose rh2r (wrf_fddaobs_in.F, iice = 0) is qv"""
    esat = 6.1078 * np.exp(17.2693882 * (t - 273.16) / (t - 35.86))
    rsat = 0.62197 * esat / (p / 100.0 - esat)
    return float(np.clip(100.0 * qv / rsat, 5.0, 100.0))


def header(date, lat, lon, ident, name, platform, elev, sound, nlev):
    return [f" {date}", f"  {lat:9.4f} {lon:9.4f} ", f"  {ident:<40s}   {name:<40s}   ",
            f"  {platform:<16s}  {'FM-?? SYNTHETIC':<16s}  {elev:7.0f}.  {'   T' if sound else '   F'}  {'   F'}  {nlev:5d}"]


def surface(dom, date, i, j, ident, platform, *, dt=1.0, du=1.5, dv=-1.0, rh=None, psfc=True, slp=False,
            wqc=0.0, name=" SURFACE DATA FROM SYNTHETIC SOURCE"):
    lat, lon = dom.latlon(i, j)
    elev = round(float(dom.at(dom.hgt, i, j)))
    t = float(dom.at(dom.t[0], i, j)) + dt
    ps = round(float(dom.at(dom.psfc, i, j)), 1)
    p1 = float(dom.at(dom.p[0], i, j))
    u = round(float(dom.at(dom.u[0], i, j)) + du, 2)
    v = round(float(dom.at(dom.v[0], i, j)) + dv, 2)
    r = rh if rh is not None else round(rh_of(t, p1, float(dom.at(dom.qv[0], i, j))) + 5.0, 1)
    slp_v = round(ps * np.exp(elev / (RD / G * (t + 0.5 * elev * 0.0065))), 1) if slp else M8
    vals = [(slp_v, 0.0 if slp else M8), (M8, M8), (float(elev), 0.0), (round(t, 2), 0.0), (u, wqc), (v, wqc),
            (r, 0.0), (ps if psfc else M8, 0.0 if psfc else M8), (M8, M8)]
    rec = " " + "".join(f"{f11(a)} {f11(b)} " for a, b in vals)
    return header(date, lat, lon, ident, name, platform, elev, False, 1) + [rec]


def sounding(dom, date, i, j, ident, platform, plevs, *, dt=0.8, du=2.0, dv=1.0, notemp=(), hgt_only=(), wqc256=(),
             name="SYNTHETIC SOUNDING"):
    lat, lon = dom.latlon(i, j)
    elev = round(float(dom.at(dom.hgt, i, j)))
    pcol = dom.at(dom.p, i, j)[::-1]  # increasing pressure for np.interp
    ps = float(dom.at(dom.psfc, i, j))
    if len(plevs) > 1:
        plevs = [ps - 1500.0] + [pl for pl in plevs if pl < ps - 3000.0]  # the first level just above the ground
    recs = []
    for n, pl in enumerate(plevs):
        lp = np.log(pl)
        it = lambda a: float(np.interp(lp, np.log(pcol), dom.at(a, i, j)[::-1]))  # noqa: E731
        t, z, qv = it(dom.t) + dt, it(dom.z), it(dom.qv)
        u, v = round(it(dom.u) + du, 2), round(it(dom.v) + dv, 2)
        r = round(rh_of(t, pl, qv), 1)
        p, pqc = (M8, M8) if n in hgt_only else (float(pl), 0.0)
        tt, tqc = (M8, M8) if n in notemp else (round(t, 2), 0.0)
        wq = 256.0 if n in wqc256 else 0.0
        vals = [(p, pqc), (round(z, 1), 0.0), (tt, tqc), (u, wq), (v, wq), (r if n not in notemp else M8, 0.0 if n not in notemp else M8)]
        recs.append(" " + "".join(f"{f11(a)} {f11(b)} " for a, b in vals))
    return header(date, lat, lon, ident, name, platform, elev, True, len(plevs)) + recs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wrfinput")
    ap.add_argument("out")
    ap.add_argument("--start", default=None, help="the run's start (default: wrfinput's START_DATE)")
    ap.add_argument("--scale", type=float, default=1.0, help="positions scaled by this factor (a nest of ratio 1/scale)")
    a = ap.parse_args()
    dom = Domain(a.wrfinput)
    t0 = datetime.datetime.strptime(a.start or dom.start, "%Y-%m-%d_%H:%M:%S")
    nx, ny = dom.nx, dom.ny

    def d(minutes):
        return (t0 + datetime.timedelta(minutes=minutes)).strftime("%Y%m%d%H%M%S")

    def ij(fi, fj):  # a position as fractions of the domain (mass grid, 1-based, inside 3 ... n - 2)
        return 3.0 + fi * (nx - 6), 3.0 + fj * (ny - 6)

    obs = []
    obs += surface(dom, d(-60), *ij(0.30, 0.40), "17000 OLD", "FM-12 SYNOP")              # before the window
    obs += surface(dom, d(0), *ij(0.25, 0.30), "17001 METAR PSFC", "FM-15 METAR")
    obs += surface(dom, d(0), *ij(0.55, 0.35), "17002 SYNOP SLP", "FM-12 SYNOP", psfc=False, slp=True, dt=-0.7)
    obs += surface(dom, d(0), *ij(0.70, 0.60), "17003 SYNOP T ONLY", "FM-12 SYNOP", psfc=False, du=-1.0, dv=2.0)
    obs += surface(dom, d(0), *ij(0.45, 0.55), "17004 PRET", "FM-12 SYNOP PRET")[:2] + \
        [f"  {'12345 METNET= 3':<40s}   {' SURFACE DATA FROM SYNTHETIC SOURCE':<40s}   "] + \
        surface(dom, d(0), *ij(0.45, 0.55), "x", "FM-12 SYNOP PRET")[3:]               # dropped (METNET= 3)
    obs += surface(dom, d(5), *ij(0.60, 0.80), "SHIP1", "FM-13 SHIP", rh=55.0, dt=0.5)  # RH < 70 % dropped
    obs += surface(dom, d(10), *ij(0.35, 0.65), "17005 METAR ROT", "FM-15 METAR", wqc=129.0, du=3.0, dv=-2.0)
    obs += surface(dom, d(10), *ij(0.80, 0.25), "17006 METAR", "FM-15 METAR", dt=1.2)
    obs += surface(dom, d(10), *ij(0.80, 0.25), "17006 METAR DUP", "FM-15 METAR", dt=1.6)  # replaces the one above
    lat_out, lon_out = dom.latlon(*ij(0.5, 0.5))
    rec_out = surface(dom, d(12), *ij(0.5, 0.5), "OUTSIDE", "FM-12 SYNOP")
    rec_out[1] = f"  {lat_out + 25.0:9.4f} {lon_out:9.4f} "                               # outside the domain
    obs += rec_out
    obs += sounding(dom, d(15), *ij(0.40, 0.45), "17130 TEMP", "FM-35 TEMP",
                    [92000.0, 85000.0, 80000.0, 70000.0, 60000.0, 50000.0, 40000.0, 30000.0],
                    notemp=(3,), hgt_only=(5,), wqc256=(6,))
    obs += surface(dom, d(15), *ij(0.65, 0.40), "17007 GROSS", "FM-15 METAR", dt=30.0)    # fails the gross check
    obs += sounding(dom, d(20), *ij(0.75, 0.70), "17240 PILOT", "FM-32 PILOT",
                    [85000.0, 70000.0, 60000.0, 50000.0, 40000.0])
    obs += sounding(dom, d(25), *ij(0.50, 0.50), "SATWND1", "FM-88 SATOB", [30000.0], dt=0.0, du=4.0, dv=-3.0,
                    name="SATWNDS")
    obs += sounding(dom, d(25), *ij(0.50, 0.50), "SATWND1 DUP", "FM-88 SATOB", [30000.0], dt=0.0, du=5.0, dv=-2.0,
                    name="SATWNDS")                                                        # replaces the one above
    obs += sounding(dom, d(28), *ij(0.30, 0.75), "ACARS1", "FM-96 AMDAR", [25000.0], dt=1.0, du=2.0, dv=2.0,
                    name=" ACARS AIRCRAFT")
    obs += surface(dom, d(30), *ij(0.20, 0.55), "17008 DRY", "FM-15 METAR", rh=8.0, dt=0.3)  # negative qv innovation
    obs += surface(dom, d(40), *ij(0.50, 0.20), "17009 METAR", "FM-15 METAR", dt=-1.0, du=-2.0)
    obs += sounding(dom, d(45), *ij(0.55, 0.65), "17131 TEMP", "FM-35 TEMP",
                    [90000.0, 85000.0, 70000.0, 50000.0, 40000.0, 30000.0], dt=-0.6, du=-1.5)
    obs += surface(dom, d(60), *ij(0.40, 0.30), "17010 SYNOP", "FM-12 SYNOP", dt=0.9, du=1.0, dv=1.0)
    obs += surface(dom, d(60), *ij(0.75, 0.50), "17011 METAR", "FM-15 METAR", dt=-0.4)
    obs += sounding(dom, d(80), *ij(0.45, 0.40), "SATWND2", "FM-88 SATOB", [35000.0], dt=0.0, du=-3.0, name="SATWNDS")
    obs += surface(dom, d(100), *ij(0.30, 0.50), "17012 METAR", "FM-15 METAR", dt=0.7)
    obs += surface(dom, d(120), *ij(0.60, 0.45), "17013 METAR", "FM-15 METAR", dt=-0.8)  # after the 1 h window
    with open(a.out, "w") as f:
        f.write("\n".join(obs) + "\n")


if __name__ == "__main__":
    main()
