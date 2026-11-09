# SPDX-License-Identifier: GPL-3.0-or-later
"""The numerical routines of wrf-python 1.3.4.1 (NCAR, Apache License 2.0), ported to numpy.

Each function follows one Fortran routine of wrf-python's `fortran/` directory statement by statement: the same
constants (fortran/wrf_constants.f90), the same order of operations, the same quirks. The loops over the horizontal
grid are vectorised; the loops over levels that depend on earlier levels stay loops over levels, each one a numpy
operation over every column. The docstrings name the routine and the file.

Conventions: 3-D arrays are (bottom_top, south_north, west_east), level 0 at the bottom, as in a wrfout file; inputs
and results are float64. Fortran's single-precision literals that are not exact in binary (0.01 in DCOMPUTESEAPRS,
1.e-15 in wrf_vintrp) are used at their single-precision value, as the Fortran compiler promotes them.

wrf-python is Copyright UCAR, licensed under the Apache License 2.0 (licenses/WRF-PYTHON-LICENSE.txt); its RIP-derived
routines are by Mark Stoelinga and others, as their comments say. The pseudoadiabat table data/psadilookup.dat is
wrf-python's file."""
import os

import numpy as np

SOURCE = "wrf-python 1.3.4.1"

# fortran/wrf_constants.f90
WRF_EARTH_RADIUS = 6370000.0
T_BASE = 300.0
PI = 3.1415926535897932384626433
RAD_PER_DEG = PI / 180.0
DEG_PER_RAD = 180.0 / PI
DEFAULT_FILL = 9.9692099683868690e36
P1000MB = 100000.0
RD = 287.0
RV = 461.6
CP = 1004.5
G = 9.81
USSALR = 0.0065
CELKEL = 273.15
CELKEL_TRIPLE = 273.16
EZERO = 6.112
ESLCON1 = 17.67
ESLCON2 = 29.65
EPS = 0.622
GAMMA = RD / CP
CPMD = 0.887
RGASMD = 0.608
GAMMAMD = RGASMD - CPMD
TLCLC1 = 2840.0
TLCLC2 = 3.5
TLCLC3 = 4.805
TLCLC4 = 55.0
THTECON1 = 3376.0
THTECON2 = 2.54
THTECON3 = 0.81
ABSCOEFI = 0.272
ABSCOEF = 0.145
GAMMA_SEVEN = 720.0
RHOWAT = 1000.0
RHO_R = RHOWAT
RHO_S = 100.0
RHO_G = 400.0
ALPHA = 0.224
SCLHT = RD * 256.0 / G
EXPON = RD * USSALR / G
EXPONI = 1.0 / EXPON


def _f32(x):
    """A Fortran default-real literal as the compiler promotes it to double."""
    return float(np.float32(x))


class DiagnosticError(ValueError):
    """A routine's error status (wrf-python raises DiagnosticError with the same message)."""


# ---------------------------------------------------------------------------------------------------------------------
# fortran/wrf_user.f90


def tk(pressure, theta):
    """DCOMPUTETK (wrf_user.f90): temperature (K) from pressure (Pa) and potential temperature (K)."""
    return (pressure / P1000MB) ** (RD / CP) * theta


def rh(qv, p, t):
    """DCOMPUTERH (wrf_user.f90): relative humidity (%) from mixing ratio (kg kg-1), pressure (Pa) and temperature (K),
    limited to [0, 100]."""
    es = EZERO * np.exp(ESLCON1 * (t - CELKEL) / (t - ESLCON2))
    qvs = EPS * es / (0.01 * p - (1.0 - EPS) * es)
    return 100.0 * np.maximum(np.minimum(qv / qvs, 1.0), 0.0)


def td(pressure_hpa, qv):
    """DCOMPUTETD (wrf_user.f90): dew point (degC) from pressure (hPa) and mixing ratio (kg kg-1)."""
    q = np.maximum(qv, 0.0)
    tdc = q * pressure_hpa / (0.622 + q)
    tdc = np.maximum(tdc, 0.001)
    return (243.5 * np.log(tdc) - 440.8) / (19.48 - np.log(tdc))


def seaprs(z, t, p, q):
    """DCOMPUTESEAPRS (wrf_user.f90, from RIP): sea-level pressure (hPa) of columns: height (m), temperature (K),
    pressure (Pa), mixing ratio (kg kg-1), with the "ridiculous MM5 test". Raises DiagnosticError where wrf-python
    does (no level 100 hPa above the surface; klo == khi)."""
    tc = 273.16 + 17.5
    pconst = 10000.0
    nz = p.shape[0]
    below = p < (p[0] - pconst)[None]
    if not below.any(axis=0).all():
        raise DiagnosticError("Error in finding 100 hPa up")
    level = below.argmax(axis=0) + 1  # Fortran's 1-based level
    klo = np.maximum(level - 1, 1)
    khi = np.minimum(klo + 1, nz - 1)
    if (klo == khi).any():
        raise DiagnosticError("Error trapping levels")
    plo, phi = _take(p, klo - 1), _take(p, khi - 1)
    tlo = _take(t, klo - 1) * (1.0 + 0.608 * _take(q, klo - 1))
    thi = _take(t, khi - 1) * (1.0 + 0.608 * _take(q, khi - 1))
    zlo, zhi = _take(z, klo - 1), _take(z, khi - 1)
    p_at_pconst = p[0] - pconst
    t_at_pconst = thi - (thi - tlo) * np.log(p_at_pconst / phi) * np.log(plo / phi)
    z_at_pconst = zhi - (zhi - zlo) * np.log(p_at_pconst / phi) * np.log(plo / phi)
    t_surf = t_at_pconst * (p[0] / p_at_pconst) ** (USSALR * RD / G)
    t_sea_level = t_at_pconst + USSALR * z_at_pconst
    l1 = t_sea_level < tc
    l2 = t_surf <= tc
    l3 = ~l1
    t_sea_level = np.where(l2 & l3, tc, tc - 0.005 * (t_surf - tc) ** 2)
    return _f32(0.01) * (p[0] * np.exp((2.0 * G * z[0]) / (RD * (t_sea_level + t_surf))))


def _take(a, k):
    """a[k[j, i], j, i] for a level index per column."""
    return np.take_along_axis(a, k[None], axis=0)[0]


def interpz3d(data3d, zdata, levels, missing=DEFAULT_FILL):
    """DINTERP3DZ (wrf_user.f90), wrf-python's interplevel: data3d at the values `levels` of the vertical coordinate
    zdata, linear in zdata; the first bracketing pair of levels searched from the top; missing where none brackets
    (strict inequalities). The search direction comes from the first column, as in the Fortran. Result
    (nlev, ny, nx)."""
    nz = zdata.shape[0]
    ip, im = (1, 0) if zdata[0, 0, 0] > zdata[nz - 1, 0, 0] else (0, 1)
    levels = np.atleast_1d(np.asarray(levels, dtype=np.float64))
    out = np.full((levels.size,) + data3d.shape[1:], missing, dtype=np.float64)
    for n, lev in enumerate(levels):
        o = out[n]
        done = np.zeros(data3d.shape[1:], dtype=bool)
        for kp in range(nz, 1, -1):
            za, zb = zdata[kp - im - 1], zdata[kp - ip - 1]
            hit = ~done & (za < lev) & (zb > lev)
            if hit.any():
                w2 = (lev - za[hit]) / (zb[hit] - za[hit])
                w1 = 1.0 - w2
                o[hit] = w1 * data3d[kp - im - 1][hit] + w2 * data3d[kp - ip - 1][hit]
                done |= hit
    return out


def filter2d(a, it, missing=DEFAULT_FILL, cenweight=2.0):
    """DFILTER2D (wrf_user.f90), wrf-python's smooth2d: `it` passes of a 1-2-1 (centre weight `cenweight`) smoother
    in y then x, the boundary rows and columns unchanged, points next to a missing value unchanged."""
    a = np.array(a, dtype=np.float64)
    cenmult = cenweight / 2.0
    coef = 1.0 / (4.0 + cenweight)
    for _ in range(it):
        b = a.copy()
        c = b[1:-1, 1:-1]
        bad = (b[:-2, 1:-1] == missing) | (c == missing) | (b[2:, 1:-1] == missing)
        inner = a[1:-1, 1:-1]
        inner[...] = np.where(bad, inner, coef * (b[:-2, 1:-1] + cenmult * c + b[2:, 1:-1]))
        bad = (b[1:-1, :-2] == missing) | (c == missing) | (b[1:-1, 2:] == missing)
        inner[...] = np.where(bad, inner, inner + coef * (b[1:-1, :-2] + cenmult * c + b[1:-1, 2:]))
    return a


def uvmet(u, v, lat, lon, cen_long, cone):
    """DCOMPUTEUVMET (wrf_user.f90) for unstaggered u, v: the wind rotated to earth coordinates with the projection's
    cone factor (g_uvmet.py: cone from TRUELAT1/2 for Lambert, 1 for polar stereographic). Returns (u_earth,
    v_earth)."""
    longca = lon - cen_long
    longca = np.where(longca > 180.0, longca - 360.0, longca)
    longca = np.where(longca < -180.0, longca + 360.0, longca)
    rpd = PI / 180.0
    longcb = np.where(lat < 0.0, -longca * cone * rpd, longca * cone * rpd)
    ca, cb = np.cos(longcb), np.sin(longcb)
    return v * cb + u * ca, v * ca - u * cb


def uvmet_cone(map_proj, truelat1, truelat2):
    """The cone factor of g_uvmet.py's _get_uvmet (None: Mercator and lat-lon, no rotation)."""
    import math
    if map_proj == 1:
        rpd = PI / 180.0
        if abs(truelat1 - truelat2) > 0.1 and abs(truelat2 - 90.0) > 0.1:
            cone = math.log(math.cos(truelat1 * rpd)) - math.log(math.cos(truelat2 * rpd))
            return cone / (math.log(math.tan((45.0 - abs(truelat1 / 2.0)) * rpd))
                           - math.log(math.tan((45.0 - abs(truelat2 / 2.0)) * rpd)))
        return math.sin(abs(truelat1) * rpd)
    if map_proj == 2:
        return 1.0
    return None


# ---------------------------------------------------------------------------------------------------------------------
# fortran/wrf_wind.f90, wrf_rip_phys_routines.f90, wrf_pw.f90, wrf_pvo.f90


def wspd(u, v):
    """DCOMPUTEWSPD (wrf_wind.f90)."""
    return np.sqrt(u * u + v * v)


def wdir(u, v):
    """DCOMPUTEWDIR (wrf_wind.f90): direction the wind blows from (degrees). Its argument is always in [90, 450], so
    Fortran's MOD equals numpy's."""
    return np.mod(270.0 - np.arctan2(v, u) * DEG_PER_RAD, 360.0)


def virtual_temp(temp, ratmix):
    """VIRTUAL_TEMP (wrf_rip_phys_routines.f90): virtual temperature (K)."""
    return temp * (EPS + ratmix) / (EPS * (1.0 + ratmix))


def omega(qvp, tmk, www, prs):
    """OMGCALC (wrf_rip_phys_routines.f90): omega (Pa s-1) from w (m s-1) at mass levels."""
    return -G * prs / (RD * ((tmk * (EPS + qvp)) / (EPS * (1.0 + qvp)))) * www


def pw(p, tv, qv, ht):
    """DCOMPUTEPW (wrf_pw.f90): precipitable water (kg m-2 = mm); ht the staggered heights (nz + 1 levels)."""
    out = np.zeros(p.shape[1:])
    for k in range(p.shape[0]):
        out = out + ((p[k] / (RD * tv[k])) * qv[k] * (ht[k + 1] - ht[k]))
    return out


def absvort(u, v, msfu, msfv, msft, cor, dx, dy):
    """DCOMPUTEABSVORT (wrf_pvo.f90): absolute vorticity (1e-5 s-1) from the staggered u (nz, ny, nx+1), v (nz, ny+1,
    nx) and the map factors; one-sided differences at the edges as in the Fortran."""
    ny, nx = msft.shape
    j = np.arange(ny)
    i = np.arange(nx)
    jp1, jm1 = np.minimum(j + 1, ny - 1), np.maximum(j - 1, 0)
    ip1, im1 = np.minimum(i + 1, nx - 1), np.maximum(i - 1, 0)
    dsx = ((ip1 - im1) * dx)[None, :]
    dsy = ((jp1 - jm1) * dy)[:, None]
    mm = msft * msft
    ua = u[:, :, :-1] / msfu[:, :-1]  # u(i, .)/msfu(i, .)
    ub = u[:, :, 1:] / msfu[:, 1:]  # u(i+1, .)/msfu(i+1, .)
    va = v[:, :-1, :] / msfv[:-1, :]  # v(., j)/msfv(., j)
    vb = v[:, 1:, :] / msfv[1:, :]  # v(., j+1)/msfv(., j+1)
    dudy = 0.5 * (ua[:, jp1, :] + ub[:, jp1, :] - ua[:, jm1, :] - ub[:, jm1, :]) / dsy * mm
    dvdx = 0.5 * (va[:, :, ip1] + vb[:, :, ip1] - va[:, :, im1] - vb[:, :, im1]) / dsx * mm
    return (dvdx - dudy + cor) * 1.0e5


# ---------------------------------------------------------------------------------------------------------------------
# fortran/wrf_vinterp.f90


def vintrp(datain, pres, tmk, qvp, ght, terrain, sfp, smsfp, vcarray, levels, icase=0, extrap=0, vcor=1, logp=0,
           missing=DEFAULT_FILL):
    """wrf_vintrp (wrf_vinterp.f90, from RIP), wrf-python's vinterp: datain at the levels of the vertical coordinate
    vcarray: vcor 1 pressure (hPa, levels in hPa), 2/3 exp(-z/SCLHT) of the height above sea level / ground (levels
    in km). logp: interpolate in log(vcarray). extrap: below the lowest level, the lowest level's value or, for
    icase 1 (pressure), 2 (height), 3/4 (temperature degC / K), 5 (theta), 6 (theta-e), RIP's standard-atmosphere
    extrapolation; above the top, the top value. Pressure pres in Pa, sfp / smsfp in hPa. Result (nlev, ny, nx)."""
    nz = datain.shape[0]
    cvcord = {1: "p", 2: "z", 3: "z", 4: "t", 5: "t"}.get(vcor, "")
    levels = np.atleast_1d(np.asarray(levels, dtype=np.float64))
    out = np.empty((levels.size,) + datain.shape[1:])
    for n, lev in enumerate(levels):
        vlev = np.exp(-(lev * 1000.0) / SCLHT) if cvcord == "z" else lev
        tempout = np.full(datain.shape[1:], missing)
        found = np.zeros(datain.shape[1:], dtype=bool)
        for k in range(1, nz):
            r = nz - k  # 0-based index of Fortran's ripk = nz - k + 1
            vcp1, vcp0 = vcarray[r - 1], vcarray[r]
            valp0, valp1 = datain[r], datain[r - 1]
            hit = ~found & (((vlev >= vcp0) & (vlev <= vcp1)) | ((vlev <= vcp0) & (vlev >= vcp1)))
            if not hit.any():
                continue
            found |= hit
            msg = hit & ((valp0 == missing) | (valp1 == missing))
            ok = hit & ~msg
            if logp == 1:
                if vlev == 0.0:
                    raise DiagnosticError("Pres=0.  Unable to take log of 0.")
                c1, c0, tv = np.log(vcp1[ok]), np.log(vcp0[ok]), np.log(vlev)
            else:
                c1, c0, tv = vcp1[ok], vcp0[ok], vlev
            tempout[ok] = _intrp_value(valp0[ok], valp1[ok], tv, c0, c1, icase)
        rest = ~found
        if extrap == 0 or not rest.any():
            out[n] = tempout
            continue
        diff = vcarray[nz - 1] - vcarray[0]
        isign = np.sign(diff)
        above = rest & (isign * vlev >= isign * vcarray[nz - 1])
        tempout[above] = datain[nz - 1][above]
        below = rest & ~above
        tempout[below & (datain[0] == missing)] = missing
        b = below & (datain[0] != missing)
        tempout[b] = datain[0][b]
        if icase > 0 and b.any():
            tempout[b] = _vintrp_extrap(datain, pres, tmk, qvp, ght, terrain, sfp, smsfp, b, vlev, cvcord, icase,
                                        tempout[b])
        out[n] = tempout
    return out


def _intrp_value(valp0, valp1, vlev, vcp0, vcp1, icase):
    """wrf_intrp_value (wrf_vinterp.f90)."""
    if icase == 2:
        valp0 = np.exp(-valp0 / SCLHT)
        valp1 = np.exp(-valp1 / SCLHT)
    if np.any(vcp1 - vcp0 == 0):
        raise DiagnosticError("bad difference in vcp's")
    rvalue = (vlev - vcp0) * (valp1 - valp0) / (vcp1 - vcp0) + valp0
    return -SCLHT * np.log(rvalue) if icase == 2 else rvalue


def _vintrp_extrap(datain, pres, tmk, qvp, ght, terrain, sfp, smsfp, b, vlev, cvcord, icase, value):
    """The below-ground extrapolation of wrf_vintrp for the columns b (their current values `value`)."""
    nz = datain.shape[0]
    value = value.copy()
    plhsl = pres[0][b] * 0.01
    zlhsl = ght[0][b]
    ezlhsl = np.exp(-zlhsl / SCLHT)
    tlhsl = tmk[0][b]
    zsurf = terrain[b]
    qvapor = np.maximum(qvp[0][b] * 0.001, _f32(1.0e-15))
    psurf = sfp[b]
    psurfsm = smsfp[b]
    ezsurf = np.exp(-zsurf / SCLHT)
    done = np.zeros(value.shape, dtype=bool)
    plev = np.zeros(value.shape)
    zlev = np.zeros(value.shape)
    agl = (vlev < ezsurf) if cvcord == "z" else (vlev < psurf) if cvcord == "p" else np.zeros(value.shape, bool)
    if cvcord == "p":
        plev[agl] = vlev
        ezlev = ((plev[agl] - plhsl[agl]) * ezsurf[agl] + (psurf[agl] - plev[agl]) * ezlhsl[agl]) \
            / (psurf[agl] - plhsl[agl])
        zlev[agl] = -SCLHT * np.log(ezlev)
        if icase == 2:
            value[agl] = zlev[agl]
            done |= agl
    elif cvcord == "z":
        ezlev = vlev
        zlev[agl] = -SCLHT * np.log(ezlev)
        plev[agl] = ((ezlev - ezlhsl[agl]) * psurf[agl] + (ezsurf[agl] - ezlev) * plhsl[agl]) \
            / (ezsurf[agl] - ezlhsl[agl])
        if icase == 1:
            value[agl] = plev[agl] * 100.0
            done |= agl
    bg = ~agl
    if bg.any():
        ptarget = psurfsm[bg] - 150.0
        dpmin = np.full(ptarget.shape, 1.0e4)
        presb = pres[:, b][:, bg]
        kexit = np.full(ptarget.shape, nz + 1)
        active = np.ones(ptarget.shape, dtype=bool)
        for k in range(1, nz + 1):
            dp = np.abs(presb[nz - k] * 0.01 - ptarget)
            stop = active & (dp > dpmin)
            kexit[stop] = k
            active &= ~stop
            dpmin = np.where(active, np.minimum(dpmin, dp), dpmin)
        kupper = kexit - 1
        ripk = nz - kupper  # 0-based index of Fortran's nz - kupper + 1
        pbot = np.maximum(plhsl[bg], psurf[bg])
        zbot = np.minimum(zlhsl[bg], zsurf[bg])
        pratio = pbot / (_take(presb, ripk) * 0.01)
        tbotextrap = _take(tmk[:, b][:, bg], ripk) * pratio ** EXPON
        vt = tbotextrap * (EPS + qvapor[bg]) / (EPS * (1.0 + qvapor[bg]))
        if cvcord == "p":
            plev[bg] = vlev
            zlev[bg] = zbot + vt / USSALR * (1.0 - (vlev / pbot) ** EXPON)
            if icase == 2:
                value[bg] = zlev[bg]
                done |= bg
        elif cvcord == "z":
            zlev[bg] = -SCLHT * np.log(vlev)
            plev[bg] = pbot * (1.0 + USSALR / vt * (zbot - zlev[bg])) ** EXPONI
            if icase == 1:
                value[bg] = plev[bg] * 100.0
                done |= bg
    if icase > 2:
        t = ~done
        tlev = tlhsl[t] + (zlhsl[t] - zlev[t]) * USSALR
        qv = np.maximum(qvp[0][b][t], _f32(1.0e-15))
        gammam = GAMMA * (1.0 + GAMMAMD * qv)
        if icase == 3:
            value[t] = tlev - CELKEL
        elif icase == 4:
            value[t] = tlev
        elif icase == 5:
            value[t] = tlev * (1000.0 / plev[t]) ** gammam
        elif icase == 6:
            e = qv * plev[t] / (EPS + qv)
            tlcl = TLCLC1 / (np.log(tlev ** TLCLC2 / e) - TLCLC3) + TLCLC4
            value[t] = tlev * (1000.0 / plev[t]) ** gammam * np.exp((THTECON1 / tlcl - THTECON2) * qv
                                                                   * (1.0 + THTECON3 * qv))
    return value


# ---------------------------------------------------------------------------------------------------------------------
# fortran/rip_cape.f90


_PSADI = None


def psadi_table():
    """DLOOKUP_TABLE (rip_cape.f90): (theta-e levels, pressure levels, temperature[ip, jt]) of data/psadilookup.dat,
    wrf-python's pseudoadiabat table (Bolton 1980)."""
    global _PSADI
    if _PSADI is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "psadilookup.dat")
        with open(path) as f:
            lines = f.read().splitlines()
        nthte, nprs = (int(x) for x in lines[14].split())
        if nthte != 150 or nprs != 150:
            raise DiagnosticError("Number of pressure or theta_e levels in lookup table file not 150")
        vals = []
        for line in lines[15:]:
            vals += [float(line[c:c + 15].replace("D", "E")) for c in range(0, len(line), 15) if line[c:c + 15].strip()]
        vals = np.array(vals)
        thte, prs = vals[:150], vals[150:300]
        tmk = vals[300:300 + 150 * 150].reshape(150, 150).T  # [ip, jt]: ip varies fastest in the file
        _PSADI = (thte, prs, tmk)
    return _PSADI


def tonpsadiabat(thte, prs):
    """TONPSADIABAT (rip_cape.f90): temperature (K) on the pseudoadiabat thte (K) at pressure prs (hPa), bilinear in
    the table; theta-e taken as theta below the table's lowest pressure. The Fortran's bisections give the same
    indices as these sorted searches (the table's axes are strictly monotonic)."""
    athte, aprs, atmk = psadi_table()
    out = np.empty(np.shape(thte))
    dry = prs <= aprs[149]
    out[dry] = thte[dry] * (prs[dry] / 1000.0) ** GAMMA
    m = ~dry
    th, p = thte[m], prs[m]
    jt = np.searchsorted(athte[1:148], th, side="right")  # 0-based: Fortran's jt - 1
    ip = np.searchsorted(-aprs[1:148], -p, side="right")
    fracjt = (th - athte[jt]) / (athte[jt + 1] - athte[jt])
    fracjt2 = 1.0 - fracjt
    fracip = (aprs[ip] - p) / (aprs[ip] - aprs[ip + 1])
    fracip2 = 1.0 - fracip
    t00, t10, t01, t11 = atmk[ip, jt], atmk[ip + 1, jt], atmk[ip, jt + 1], atmk[ip + 1, jt + 1]
    if (t00 > 1e9).any() or (t10 > 1e9).any() or (t01 > 1e9).any() or (t11 > 1e9).any():
        raise DiagnosticError("capecalc3d: Tried to access missing temperature in lookup table. "
                              "Prs and Thte probably unreasonable.")
    out[m] = fracip2 * fracjt2 * t00 + fracip * fracjt2 * t10 + fracip2 * fracjt * t01 + fracip * fracjt * t11
    return out


def _pfcalc(prs, sfp):
    """DPFCALC (rip_cape.f90), terrain-following data: the pressures bounding the layers (top-first arrays)."""
    pf = np.empty_like(prs)
    pf[:-1] = 0.5 * (prs[1:] + prs[:-1])
    pf[-1] = sfp
    return pf


def _lift(prs, tmk, qvp, ght, kpar, tlcl, ethpari, zlcl, cmsg):
    """The parcel ascent of DCAPECALC3D / DCAPECALC2D for one parcel per column: arrays top-first (nz, ncol), kpar
    the parcel's 1-based level per column (Fortran's index: 1 at the top), the parcel's tlcl, theta-e and LCL
    height. Returns cape, cin, the LCL and LFC heights relative to the parcel (zrel(klcl), zrel(klfc))."""
    nz, ncol = prs.shape
    col = np.arange(ncol)
    at = lambda a, k: a[k - 1, col]  # noqa: E731  Fortran a(k) per column
    buoy = np.zeros((2 * nz + 2, ncol))
    zrel = np.zeros((2 * nz + 2, ncol))
    kk = np.zeros(ncol, dtype=np.int64)
    ilcl = np.zeros(ncol, dtype=np.int64)
    klcl = np.zeros(ncol, dtype=np.int64)
    ghtpar, tmkpar, qvppar = at(ght, kpar), at(tmk, kpar), at(qvp, kpar)
    sat = ghtpar >= zlcl
    ilcl[sat] = 2
    klcl[sat] = 1
    for k in range(int(kpar.max()), 0, -1):
        act = k <= kpar
        kk[act] += 1
        gk = ght[k - 1]
        c1 = act & (gk < zlcl)
        c2 = act & ~c1 & (gk >= zlcl) & (ilcl == 0)
        c3 = act & ~c1 & ~c2
        tvlift = np.zeros(ncol)
        tvenv = np.zeros(ncol)
        ghtlift = np.zeros(ncol)
        if c1.any():
            tmklift = tmkpar[c1] - G / (CP * (1.0 + CPMD * qvppar[c1])) * (gk[c1] - ghtpar[c1])
            qk = qvp[k - 1][c1]
            tvenv[c1] = tmk[k - 1][c1] * (EPS + qk) / (EPS * (1.0 + qk))
            tvlift[c1] = tmklift * (EPS + qvppar[c1]) / (EPS * (1.0 + qvppar[c1]))
            ghtlift[c1] = gk[c1]
        if c2.any():
            gk1 = ght[k][c2]  # k + 1 (the level below)
            facden = 1.0 / (gk[c2] - gk1)
            z = zlcl[c2]
            tmkenv = tmk[k][c2] * ((gk[c2] - z) * facden) + tmk[k - 1][c2] * ((z - gk1) * facden)
            qvpenv = qvp[k][c2] * ((gk[c2] - z) * facden) + qvp[k - 1][c2] * ((z - gk1) * facden)
            tvenv[c2] = tmkenv * (EPS + qvpenv) / (EPS * (1.0 + qvpenv))
            tvlift[c2] = tlcl[c2] * (EPS + qvppar[c2]) / (EPS * (1.0 + qvppar[c2]))
            ghtlift[c2] = z
            ilcl[c2] = 1
        if c3.any():
            pk = prs[k - 1][c3]
            tmklift = tonpsadiabat(ethpari[c3], pk)
            eslift = EZERO * np.exp(ESLCON1 * (tmklift - CELKEL) / (tmklift - ESLCON2))
            qvplift = EPS * eslift / (pk - eslift)
            qk = qvp[k - 1][c3]
            tvenv[c3] = tmk[k - 1][c3] * (EPS + qk) / (EPS * (1.0 + qk))
            tvlift[c3] = tmklift * (EPS + qvplift) / (EPS * (1.0 + qvplift))
            ghtlift[c3] = gk[c3]
        ca = col[act]
        ka = kk[act]
        buoy[ka, ca] = G * (tvlift[act] - tvenv[act]) / tvenv[act]
        zrel[ka, ca] = ghtlift[act] - ghtpar[act]
        cr = act & (kk > 1)
        cr[cr] = buoy[kk[cr], col[cr]] * buoy[kk[cr] - 1, col[cr]] < 0.0
        if cr.any():
            cc = col[cr]
            kk[cr] += 1
            kc = kk[cr]
            buoy[kc, cc] = buoy[kc - 1, cc]
            zrel[kc, cc] = zrel[kc - 1, cc]
            buoy[kc - 1, cc] = 0.0
            zrel[kc - 1, cc] = zrel[kc - 2, cc] + buoy[kc - 2, cc] / (buoy[kc - 2, cc] - buoy[kc, cc]) \
                * (zrel[kc, cc] - zrel[kc - 2, cc])
        lc = act & (ilcl == 1)
        klcl[lc] = kk[lc]
        ilcl[lc] = 2
    kmax = kk
    klcl = np.where(ilcl == 0, kmax, klcl)
    km = int(kmax.max())
    benaccum = np.zeros((2 * nz + 2, ncol))
    for k in range(2, km + 1):
        m = k <= kmax
        dz = zrel[k] - zrel[k - 1]
        benaccum[k] = np.where(m, benaccum[k - 1] + 0.5 * dz * (buoy[k - 1] + buoy[k]), benaccum[k])
    kel = np.zeros(ncol, dtype=np.int64)
    elfound = np.zeros(ncol, dtype=bool)
    for k in range(km, 0, -1):
        m = ~elfound & (k <= kmax) & (k >= klcl) & (buoy[k] >= 0.0)
        kel[m] = k
        elfound |= m
    benamin = np.full(ncol, 9e9)
    klfc = kmax.copy()
    for k in range(1, km + 1):
        m = elfound & (k >= klcl) & (k <= kel) & (benaccum[k] < benamin)
        benamin[m] = benaccum[k][m]
        klfc[m] = k
    cape = np.full(ncol, cmsg)
    cin = np.full(ncol, cmsg)
    e = elfound
    cape[e] = np.maximum(benaccum[kel[e], col[e]] - benamin[e], 0.1)
    cin[e] = np.maximum(-benamin[e], 0.1)
    cin[e & (cape < 100.0)] = cmsg
    return cape, cin, zrel[klcl, col], zrel[klfc, col]


def _parcel_props(tmkp, qvpp, prsp, ghtp, two_d):
    """tlcl, theta-e and LCL height of parcels (DCAPECALC3D's, or DCAPECALC2D's with its cpm)."""
    e = np.maximum(1e-20, qvpp * prsp / (EPS + qvpp))
    tlcl = TLCLC1 / (np.log(tmkp ** TLCLC2 / e) - TLCLC3) + TLCLC4
    ethpari = tmkp * (1000.0 / prsp) ** (GAMMA * (1.0 + GAMMAMD * qvpp)) \
        * np.exp((THTECON1 / tlcl - THTECON2) * qvpp * (1.0 + THTECON3 * qvpp))
    if two_d:
        cpm = CP * (1.0 + CPMD * qvpp)
        zlcl = ghtp + (tmkp - tlcl) / (G / cpm)
    else:
        zlcl = ghtp + (tmkp - tlcl) / (G / CP * (1.0 + CPMD * qvpp))
    return tlcl, ethpari, zlcl


def _top_first(prs, tmk, qvp, ght):
    nz = prs.shape[0]
    return tuple(np.ascontiguousarray(a[::-1].reshape(nz, -1)) for a in (prs, tmk, qvp, ght))


def cape_3d(prs, tmk, qvp, ght, ter, sfp, cmsg=DEFAULT_FILL):
    """DCAPECALC3D (rip_cape.f90), wrf-python's cape_3d: CAPE and CIN (J kg-1) of a parcel lifted from every level
    (pressure hPa, temperature K, mixing ratio kg kg-1, height m, terrain m, surface pressure hPa; bottom-first arrays
    as in the file, flipped like cape_left_iter). The top level gets 0; CIN is cmsg where CAPE < 100 J kg-1, both are
    cmsg without an equilibrium level. Level 0 of the result is the surface-based parcel. Returns (cape, cin)."""
    shape = prs.shape
    nz = shape[0]
    p, t, q, z = _top_first(prs, tmk, qvp, ght)
    ncol = p.shape[1]
    cape = np.zeros((nz, ncol))
    cin = np.zeros((nz, ncol))
    for kpar in range(2, nz + 1):
        k = np.full(ncol, kpar)
        tlcl, eth, zlcl = _parcel_props(t[kpar - 1], np.asarray(q[kpar - 1]), p[kpar - 1], z[kpar - 1], False)
        cape[kpar - 1], cin[kpar - 1], _, _ = _lift(p, t, q, z, k, tlcl, eth, zlcl, cmsg)
    return cape[::-1].reshape(shape), cin[::-1].reshape(shape)


def cape_2d(prs, tmk, qvp, ght, ter, sfp, cmsg=DEFAULT_FILL, parcel="mu", ml_depth=100.0):
    """DCAPECALC2D (rip_cape.f90), wrf-python's cape_2d: CAPE, CIN (J kg-1), LCL and LFC heights (m above ground) of
    one parcel per column, averaged over a layer: parcel "mu", as wrf-python, the level of maximum theta-e in the
    lowest 3000 m above ground averaged over about 500 m around it ("MCAPE"). parcel "ml" is the same routine with
    the parcel at the lowest model level averaged over the lowest ml_depth hPa above the surface (a mixed-layer
    parcel: only the parcel's level and depth differ from the Fortran). Arguments as cape_3d. Returns
    (cape, cin, lcl, lfc), each (ny, nx)."""
    nz = prs.shape[0]
    shape2 = prs.shape[1:]
    p, t, q, z = _top_first(prs, tmk, qvp, ght)
    ter = np.ravel(ter)
    pf = _pfcalc(p, np.ravel(sfp))
    ncol = p.shape[1]
    col = np.arange(ncol)
    if parcel == "mu":
        qm = np.maximum(q, 1e-15)
        tl = TLCLC1 / (np.log(t ** TLCLC2 / (qm * p / (EPS + qm))) - TLCLC3) + TLCLC4
        eth = t * (1000.0 / p) ** (GAMMA * (1.0 + GAMMAMD * qm)) \
            * np.exp((THTECON1 / tl - THTECON2) * qm * (1.0 + THTECON3 * qm))
        eth = np.where(z - ter[None] < 3000.0, eth, -1.0)
        klev = np.full(ncol, nz)
        ethmax = np.full(ncol, -1.0)
        for k in range(1, nz + 1):
            m = eth[k - 1] > ethmax
            klev[m] = k
            ethmax[m] = eth[k - 1][m]
        pavg = 500.0 * p[klev - 1, col] * G / (RD * virtual_temp(t[klev - 1, col], q[klev - 1, col]))
    elif parcel == "ml":
        klev = np.full(ncol, nz)
        pavg = np.full(ncol, float(ml_depth))
    else:
        raise ValueError(f"cape_2d: parcel {parcel!r} (mu or ml)")
    kpar1 = klev
    p2 = np.minimum(p[kpar1 - 1, col] + 0.5 * pavg, pf[nz - 1])
    p1 = p2 - pavg
    totthe = np.zeros(ncol)
    totqvp = np.zeros(ncol)
    totprs = np.zeros(ncol)
    active = np.ones(ncol, dtype=bool)
    for k in range(nz, 1, -1):
        active &= ~(pf[k - 1] <= p1)
        m = active & ~(pf[k - 2] >= p2)
        pup, pdn = pf[k - 1], pf[k - 2]
        qk = np.maximum(q[k - 1], 1e-15)
        th = t[k - 1] * (1000.0 / p[k - 1]) ** (GAMMA * (1.0 + GAMMAMD * qk))
        pp1 = np.maximum(p1, pdn)
        pp2 = np.minimum(p2, pup)
        m &= pp2 > pp1
        totqvp = np.where(m, totqvp + qk * (pp2 - pp1), totqvp)
        totthe = np.where(m, totthe + th * (pp2 - pp1), totthe)
        totprs = np.where(m, totprs + (pp2 - pp1), totprs)
    qvppari = totqvp / totprs
    tmkpari = (totthe / totprs) * (p[kpar1 - 1, col] / 1000.0) ** (GAMMA * (1.0 + GAMMAMD * q[kpar1 - 1, col]))
    tlcl, eth, zlcl = _parcel_props(tmkpari, qvppari, p[kpar1 - 1, col], z[kpar1 - 1, col], True)
    cape, cin, zl, zf = _lift(p, t, q, z, kpar1, tlcl, eth, zlcl, cmsg)
    ghtp = z[kpar1 - 1, col]
    lcl = zl + ghtp - ter
    lfc = zf + ghtp - ter
    return tuple(a.reshape(shape2) for a in (cape, cin, lcl, lfc))


# ---------------------------------------------------------------------------------------------------------------------
# fortran/wrf_relhl.f90, calc_uh.f90


def srh(u, v, ght, ter, lat, top):
    """DCALRELHL (wrf_relhl.f90, from RIP4 via C. Mattocks): storm-relative helicity (m2 s-2) between the ground and
    `top` m above it, the storm motion 75 % of the 3-10 km mean wind turned 30 degrees to the right (left in the
    southern hemisphere). Bottom-first u, v (unstaggered), ght (m); wrf-python's get_srh flips them to top-first."""
    nz = u.shape[0]
    shape2 = u.shape[1:]
    uf, vf, zf = (np.ascontiguousarray(a[::-1].reshape(nz, -1)) for a in (u, v, ght))
    ter = np.ravel(ter)
    lat = np.ravel(lat)
    ncol = uf.shape[1]
    k10 = np.zeros(ncol, dtype=np.int64)
    ktop = np.zeros(ncol, dtype=np.int64)
    k3 = np.zeros(ncol, dtype=np.int64)
    active = np.ones(ncol, dtype=bool)
    for k in range(nz, 1, -1):
        agl = zf[k - 1] - ter
        hit = active & (agl > 10000.0) & (k10 == 0)
        k10[hit] = k
        active &= ~hit
        m = active & (agl > top) & (ktop == 0)
        ktop[m] = k
        m = active & (agl > 3000.0) & (k3 == 0)
        k3[m] = k
    k10[k10 == 0] = 2
    sdh = np.zeros(ncol)
    su = np.zeros(ncol)
    sv = np.zeros(ncol)
    for k in range(nz, 1, -1):  # k from k3 down to k10, in that order
        m = (k <= k3) & (k >= k10)
        dh = zf[k - 2] - zf[k - 1]
        sdh = np.where(m, sdh + dh, sdh)
        su = np.where(m, su + 0.5 * dh * (uf[k - 2] + uf[k - 1]), su)
        sv = np.where(m, sv + 0.5 * dh * (vf[k - 2] + vf[k - 1]), sv)
    with np.errstate(invalid="ignore", divide="ignore"):
        ua = su / sdh
        va = sv / sdh
    asp = np.sqrt(ua * ua + va * va)
    adr = np.where((ua == 0.0) & (va == 0.0), 0.0, DEG_PER_RAD * (PI + np.arctan2(ua, va)))
    bsp = 0.75 * asp
    bdr = np.where(lat >= 0, adr + 30.0, adr - 30.0)
    bdr = np.where(bdr > 360.0, bdr - 360.0, bdr)
    cu = -bsp * np.sin(bdr * RAD_PER_DEG)
    cv = -bsp * np.cos(bdr * RAD_PER_DEG)
    total = np.zeros(ncol)
    for k in range(nz - 1, 0, -1):
        m = k >= ktop
        x = ((uf[k - 1] - cu) * (vf[k - 1] - vf[k])) - ((vf[k - 1] - cv) * (uf[k - 1] - uf[k]))
        total = np.where(m, total + x, total)
    return (-total).reshape(shape2)


def udhel(zp, mapfct, us, vs, w, dx, dy, uhmnhgt=2000.0, uhmxhgt=5000.0):
    """DCALCUH (calc_uh.f90, K. Brewster, CAPS; Kain et al. 2008): updraft helicity (m2 s-2) between uhmnhgt and
    uhmxhgt m above the ground where the column's mean w over that layer is upward, else 0. zp the staggered height
    (m, nz + 1 levels), us / vs the unstaggered wind, w staggered; zero on the outer two rows and columns as in the
    Fortran (whose ground reference is zp at its level 2)."""
    nzp1, ny, nx = zp.shape
    nz = nzp1 - 1
    tem1 = np.zeros((nz, ny, nx))
    tem2 = np.zeros((nz, ny, nx))
    twodx, twody = 2.0 * dx, 2.0 * dy
    K, J, I = slice(1, nz - 2), slice(1, ny - 1), slice(1, nx - 1)
    mf = mapfct[J, I]
    tem1[K, J, I] = (0.5 * (w[1:nz - 2, J, I] + w[2:nz - 1, J, I])) * (
        (vs[K, J, 2:nx] - vs[K, J, 0:nx - 2]) / (twodx * mf) - (us[K, 2:ny, I] - us[K, 0:ny - 2, I]) / (twody * mf))
    tem2[K, J, I] = 0.5 * (zp[1:nz - 2, J, I] + zp[2:nz - 1, J, I])
    J2, I2 = slice(1, ny - 2), slice(1, nx - 2)
    zpc = zp[:, J2, I2]
    wc = w[:, J2, I2]
    t1, t2 = tem1[:, J2, I2], tem2[:, J2, I2]
    zbot = zpc[1] + uhmnhgt
    ztop = zpc[1] + uhmxhgt
    at = lambda a, k: _take(a, k - 1)  # noqa: E731  Fortran a(k) per column

    def first_above(a, zlim):
        k = np.full(zlim.shape, nz - 2)
        for kk in range(nz - 3, 1, -1):
            k = np.where(a[kk - 1] > zlim, kk, k)
        return k

    def at_height(a, zc, kz, zlim):
        wgtlw = (at(zc, kz) - zlim) / (at(zc, kz) - at(zc, kz - 1))
        return (wgtlw * at(a, kz - 1)) + ((1.0 - wgtlw) * at(a, kz))

    def integral(a, zc, kb, kt, vb, vt):
        s = 0.5 * (at(a, kb) + vb) * (at(zc, kb) - zbot)
        for k in range(2, nz + 1):
            m = (k >= kb + 1) & (k <= kt - 1)
            s = np.where(m, s + 0.5 * (a[k - 1] + a[k - 2]) * (zc[k - 1] - zc[k - 2]), s)
        return s + 0.5 * (vt + at(a, kt - 1)) * (ztop - at(zc, kt - 1))

    kbot, ktop = first_above(zpc, zbot), first_above(zpc, ztop)
    wsum = integral(wc, zpc, kbot, ktop, at_height(wc, zpc, kbot, zbot), at_height(wc, zpc, ktop, ztop))
    wmean = wsum / (uhmxhgt - uhmnhgt)
    kbot, ktop = first_above(t2, zbot), first_above(t2, ztop)
    with np.errstate(invalid="ignore", divide="ignore"):
        uh = integral(t1, t2, kbot, ktop, at_height(t1, t2, kbot, zbot), at_height(t1, t2, ktop, ztop))
    out = np.zeros((ny, nx))
    out[J2, I2] = np.where(wmean > 0.0, uh, 0.0)
    return out


# ---------------------------------------------------------------------------------------------------------------------
# fortran/wrf_user_dbz.f90, wrf_cloud_fracf.f90, wrf_fctt.f90


def dbz(prs, tmk, qvp, qra, qsn, qgr, sn0, ivarint=0, iliqskin=0):
    """CALCDBZ (wrf_user_dbz.f90, from RIP): equivalent reflectivity factor (dBZ, at least -30) of rain, snow and
    graupel for exponential size distributions; sn0 = 0 (no snow in the file): rain below 0 degC counts as snow;
    ivarint = 1: Thompson et al. (2004) variable intercepts; iliqskin = 1: frozen particles above 0 degC scatter as
    liquid. Pressure in Pa."""
    qvp = np.maximum(qvp, 0.0)
    qra = np.maximum(qra, 0.0)
    qsn = np.maximum(qsn, 0.0)
    qgr = np.maximum(qgr, 0.0)
    if sn0 == 0:
        cold = tmk < CELKEL
        qsn = np.where(cold, qra, qsn)
        qra = np.where(cold, 0.0, qra)
    factor_r = GAMMA_SEVEN * 1.0e18 * (1.0 / (PI * RHO_R)) ** 1.75
    factor_s = GAMMA_SEVEN * 1.0e18 * (1.0 / (PI * RHO_S)) ** 1.75 * (RHO_S / RHOWAT) ** 2 * ALPHA
    factor_g = GAMMA_SEVEN * 1.0e18 * (1.0 / (PI * RHO_G)) ** 1.75 * (RHO_G / RHOWAT) ** 2 * ALPHA
    virtual_t = tmk * (0.622 + qvp) / (0.622 * (1.0 + qvp))
    rhoair = prs / (RD * virtual_t)
    if iliqskin == 1:
        warm = tmk > CELKEL
        factorb_s = np.where(warm, factor_s / ALPHA, factor_s)
        factorb_g = np.where(warm, factor_g / ALPHA, factor_g)
    else:
        factorb_s, factorb_g = factor_s, factor_g
    if ivarint == 1:
        r1, ron2, gon, ron_qr0 = 1.0e-15, 1.0e10, 5.0e7, 0.00010
        ron_min = 8.0e6
        ron_delqr0 = 0.25 * ron_qr0
        ron_const1r = (ron2 - ron_min) * 0.5
        ron_const2r = (ron2 + ron_min) * 0.5
        temp_c = np.minimum(-0.001, tmk - CELKEL)
        sonv = np.minimum(2.0e8, 2.0e6 * np.exp(-0.12 * temp_c))
        with np.errstate(divide="ignore"):
            g = 2.38 * (PI * RHO_G / (rhoair * qgr)) ** 0.92
        gonv = np.where(qgr > r1, np.maximum(1.0e4, np.minimum(g, gon)), gon)
        ronv = np.where(qra > r1, ron_const1r * np.tanh((ron_qr0 - qra) / ron_delqr0) + ron_const2r, ron2)
    else:
        ronv, sonv, gonv = 8.0e6, 2.0e7, 4.0e6
    z_e = factor_r * (rhoair * qra) ** 1.75 / ronv ** 0.75 \
        + factorb_s * (rhoair * qsn) ** 1.75 / sonv ** 0.75 \
        + factorb_g * (rhoair * qgr) ** 1.75 / gonv ** 0.75
    z_e = np.maximum(z_e, 0.001)
    return 10.0 * np.log10(z_e)


def cloudfrac(vert, rhum, vert_inc_w_height, low_thresh, mid_thresh, high_thresh, msg=DEFAULT_FILL):
    """DCLOUDFRAC2 (wrf_cloud_fracf.f90): low, mid and high cloud fraction (0-1) from the maximum relative humidity in
    each layer (bounds on the vertical coordinate `vert`, pressure in Pa by default: 97000, 80000, 45000);
    msg where a layer's lower bound is below the lowest model level (mountains)."""
    nz = vert.shape[0]
    shape2 = vert.shape[1:]
    kclo = np.full(shape2, -1)
    kcmi = np.full(shape2, -1)
    kchi = np.full(shape2, -1)
    for k in range(1, nz + 1):
        vk = vert[k - 1]
        if vert_inc_w_height != 0:
            kclo[vk < low_thresh], kcmi[vk < mid_thresh], kchi[vk < high_thresh] = k, k, k
        else:
            kclo[vk > low_thresh], kcmi[vk > mid_thresh], kchi[vk > high_thresh] = k, k, k
    lowc = np.zeros(shape2)
    midc = np.zeros(shape2)
    highc = np.zeros(shape2)
    for k in range(1, nz + 1):
        r = rhum[k - 1]
        lo = (k >= kclo) & (k < kcmi)
        mi = ~lo & (k >= kcmi) & (k < kchi)
        hi = ~lo & ~mi & (k >= kchi)
        lowc = np.where(lo, np.maximum(r, lowc), lowc)
        midc = np.where(mi, np.maximum(r, midc), midc)
        highc = np.where(hi, np.maximum(r, highc), highc)
    lowc = np.where(kclo >= 1, np.maximum(np.minimum(4.0 * lowc / 100.0 - 3.0, 1.0), 0.0), msg)
    midc = np.where(kcmi >= 1, np.maximum(np.minimum(4.0 * midc / 100.0 - 3.0, 1.0), 0.0), msg)
    highc = np.where(kchi >= 1, np.maximum(np.minimum(2.5 * highc / 100.0 - 1.5, 1.0), 0.0), msg)
    return lowc, midc, highc


def ctt(prs, tk_, qci, qcw, qvp, ght, ter, haveqci=1, fill_nocloud=0, missing=DEFAULT_FILL, opt_thresh=1.0):
    """wrfcttcalc (wrf_fctt.f90, from RIP): cloud-top temperature (degC) where the optical depth of cloud water and
    ice, integrated down from the model top, reaches opt_thresh; pressure hPa, mixing ratios g kg-1 (as wrf-python's
    get_ctt passes them). missing where the column never reaches it (fill_nocloud = 0: the lowest level's
    temperature instead)."""
    nz = prs.shape[0]
    shape2 = prs.shape[1:]
    pr, t, qi, qw, qv, z = (np.ascontiguousarray(a.reshape(nz, -1)) for a in (prs, tk_, qci, qcw, qvp, ght))
    ter = np.ravel(ter)
    ncol = pr.shape[1]
    pf = np.empty((nz, ncol))
    ratmix = 0.001 * qv[0]
    vt = t[0] * (EPS + ratmix) / (EPS * (1.0 + ratmix))
    agl_hgt = z[nz - 1] - ter
    pf[nz - 1] = pr[0] * (vt / (vt + USSALR * agl_hgt)) ** (-G / (RD * USSALR))
    for k in range(1, nz):
        ripk = nz - k + 1
        pf[k - 1] = 0.5 * (pr[ripk - 1] + pr[ripk - 2])
    opdepthd = np.zeros(ncol)
    prsctt = np.full(ncol, -1.0)
    active = np.ones(ncol, dtype=bool)
    for k in range(2, nz + 1):
        opdepthu = opdepthd.copy()
        ripk = nz - k + 1
        dp = 100.0 * (pf[k - 1] - pf[k - 2])
        if haveqci == 0:
            coef = np.where(t[ripk - 1] < CELKEL, ABSCOEFI, ABSCOEF)
            new = opdepthu + coef * qw[ripk - 1] * dp / G
        else:
            new = opdepthd + (ABSCOEF * qw[ripk - 1] + ABSCOEFI * qi[ripk - 1]) * dp / G
        opdepthd = np.where(active, new, opdepthd)
        below = opdepthd < opt_thresh
        if k == nz:
            m = active & below
            if fill_nocloud == 0:
                prsctt[m] = pr[0][m]
            active &= ~m
        m = active & ~below
        if m.any():
            fac = (1.0 - opdepthu[m]) / (opdepthd[m] - opdepthu[m])
            pc = pf[k - 2][m] + fac * (pf[k - 1][m] - pf[k - 2][m])
            prsctt[m] = np.minimum(pr[0][m], np.maximum(pr[nz - 1][m], pc))
            active &= ~m
    out = np.full(ncol, missing)
    have = prsctt > -1
    done = ~have
    for k in range(2, nz + 1):
        ripk = nz - k + 1
        p1, p2 = pr[ripk], pr[ripk - 1]
        m = ~done & (prsctt >= p1) & (prsctt <= p2)
        if m.any():
            fac = (prsctt[m] - p1[m]) / (p2[m] - p1[m])
            arg1 = fac * (t[ripk - 1][m] - t[ripk][m]) - CELKEL
            out[m] = t[ripk][m] + arg1
            done |= m
    return out.reshape(shape2)
