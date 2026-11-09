# SPDX-License-Identifier: GPL-3.0-or-later
"""One time of a wrfout file and wrf-python's diagnostics of it.

The methods follow wrf-python 1.3.4.1's getters (src/wrf/g_*.py) line by line: the same variables, the same
arithmetic on the arrays as read from the file (float32 sums such as P + PB stay float32, as there), then the
routine of wrfpy in float64. wrf-python returns each routine's result in the dtype of its first input (its cast_type
decorator); with emulate=True (the default) the results are rounded the same way, so a float32 wrfout gives the
values wrf-python gives, not more precise ones. A float64 wrfout is float64 throughout either way."""
import numpy as np

from . import wrfpy as w

PA_TO_HPA = 0.01


def destagger(a, axis):
    """wrf-python's destagger (destag.py): the mean of neighbours along `axis`."""
    n = a.shape[axis]
    s1 = [slice(None)] * a.ndim
    s2 = [slice(None)] * a.ndim
    s1[axis] = slice(0, n - 1)
    s2[axis] = slice(1, n)
    return .5 * (a[tuple(s1)] + a[tuple(s2)])


class Frame:
    """Time index `t` of an open netCDF4.Dataset `ds` (a wrfout file)."""

    def __init__(self, ds, t=0, emulate=True):
        self.ds = ds
        self.t = t
        self.emulate = emulate
        self.attrs = {k: ds.getncattr(k) for k in ds.ncattrs()}
        self._raw = {}
        self._memo = {}

    # -- file access ---------------------------------------------------------------------------------------------------
    def has(self, name):
        return name in self.ds.variables

    def raw(self, name):
        """A variable at this time as stored (its own dtype)."""
        if name not in self._raw:
            v = self.ds.variables[name]
            v.set_auto_mask(False)
            a = v[self.t] if v.dimensions[:1] == ("Time",) else v[:]
            self._raw[name] = np.asarray(a)
        return self._raw[name]

    def attr(self, name, default=None):
        return self.attrs.get(name, default)

    def _call(self, fn, ref, *args, **kw):
        """A wrfpy routine on float64 copies of the arrays, its result(s) in ref's dtype (cast_type)."""
        args = [np.asarray(a, dtype=np.float64) if isinstance(a, np.ndarray) else a for a in args]
        out = fn(*args, **kw)
        dt = ref.dtype if self.emulate else np.float64
        if isinstance(out, tuple):
            return tuple(np.asarray(o).astype(dt) for o in out)
        return np.asarray(out).astype(dt)

    def _m(self, key, fn):
        if key not in self._memo:
            self._memo[key] = fn()
        return self._memo[key]

    # -- basic fields (g_pressure, g_temp, g_geoht) --------------------------------------------------------------------
    def pressure(self):
        """Pa (get_pressure)."""
        return self._m("p", lambda: self.raw("P") + self.raw("PB"))

    def theta(self):
        return self._m("theta", lambda: self.raw("T") + w.T_BASE)

    def tk(self):
        """K (get_temp)."""
        return self._m("tk", lambda: self._call(w.tk, self.pressure(), self.pressure(), self.theta()))

    def height(self, msl=True):
        """Height of the mass levels, m above sea level or ground (get_height)."""
        def f():
            z = destagger(self.raw("PH") + self.raw("PHB"), 0) / w.G
            if msl:
                return z
            return z - self.raw("HGT")[None]
        return self._m(("z", msl), f)

    def stag_height(self):
        """(PH + PHB) / G at the w levels, as get_slp, get_pw and get_uh compute it."""
        return self._m("zstag", lambda: (self.raw("PH") + self.raw("PHB")) / w.G)

    def terrain(self):
        return self.raw("HGT")

    def u_mass(self):
        return self._m("um", lambda: destagger(self.raw("U"), -1))

    def v_mass(self):
        return self._m("vm", lambda: destagger(self.raw("V"), -2))

    # -- diagnostics ---------------------------------------------------------------------------------------------------
    def slp(self):
        """hPa (get_slp)."""
        def f():
            q = self.raw("QVAPOR").copy()
            q[q < 0] = 0.
            full_ph = (self.raw("PH") + self.raw("PHB")) / w.G
            destag_ph = destagger(full_ph, 0)
            return self._call(w.seaprs, destag_ph, destag_ph, self.tk(), self.pressure(), q)
        return self._m("slp", f)

    def rh(self):
        """% (get_rh)."""
        def f():
            q = self.raw("QVAPOR").copy()
            q[q < 0] = 0
            return self._call(w.rh, q, q, self.pressure(), self.tk())
        return self._m("rh", f)

    def rh2(self):
        """% (get_rh_2m)."""
        def f():
            q2 = self.raw("Q2").copy()
            q2[q2 < 0] = 0
            return self._call(w.rh, q2, q2, self.raw("PSFC"), self.raw("T2"))
        return self._m("rh2", f)

    def td(self):
        """degC (get_dp)."""
        def f():
            p = .01 * self.pressure()
            q = self.raw("QVAPOR").copy()
            q[q < 0] = 0
            return self._call(w.td, p, p, q)
        return self._m("td", f)

    def td2(self):
        """degC (get_dp_2m)."""
        def f():
            p = .01 * self.raw("PSFC")
            q2 = self.raw("Q2").copy()
            q2[q2 < 0] = 0
            return self._call(w.td, p, p, q2)
        return self._m("td2", f)

    def pw(self):
        """kg m-2 (get_pw)."""
        def f():
            tv = self._call(w.virtual_temp, self.tk(), self.tk(), self.raw("QVAPOR"))
            p = self.pressure()
            return self._call(w.pw, p, p, tv, self.raw("QVAPOR"), self.stag_height())
        return self._m("pw", f)

    def omega(self):
        """Pa s-1 (get_omega)."""
        def f():
            wa = destagger(self.raw("W"), 0)
            q = self.raw("QVAPOR")
            return self._call(w.omega, q, q, self.tk(), wa, self.pressure())
        return self._m("omega", f)

    def avo(self):
        """1e-5 s-1 (get_avo)."""
        def f():
            u = self.raw("U")
            return self._call(w.absvort, u, u, self.raw("V"), self.raw("MAPFAC_U"), self.raw("MAPFAC_V"),
                              self.raw("MAPFAC_M"), self.raw("F"), float(self.attr("DX")), float(self.attr("DY")))
        return self._m("avo", f)

    def _cape_inputs(self):
        p_hpa = PA_TO_HPA * self.pressure()
        z = destagger(self.raw("PH") + self.raw("PHB"), 0) / w.G
        return p_hpa, self.tk(), self.raw("QVAPOR"), z, self.raw("HGT"), PA_TO_HPA * self.raw("PSFC")

    def cape_2d(self, parcel="mu", ml_depth=100.0):
        """(cape, cin, lcl, lfc), J kg-1 and m above ground (get_2dcape; parcel "ml": see wrfpy.cape_2d)."""
        def f():
            args = self._cape_inputs()
            return self._call(w.cape_2d, args[0], *args, parcel=parcel, ml_depth=ml_depth)
        return self._m(("cape2d", parcel, ml_depth if parcel == "ml" else None), f)

    def cape_sb(self):
        """(cape, cin) of the parcel at the lowest model level: level 0 of get_3dcape (DCAPECALC3D's parcels are
        independent, so only that one is lifted)."""
        def f():
            args = self._cape_inputs()
            return self._call(_cape_lowest, args[0], *args)
        return self._m("capesb", f)

    def srh(self, top=3000.0):
        """m2 s-2 (get_srh)."""
        def f():
            z = destagger(self.raw("PH") + self.raw("PHB"), 0) / w.G
            u = self.u_mass()
            return self._call(w.srh, u, u, self.v_mass(), z, self.raw("HGT"), self.raw("XLAT"), float(top))
        return self._m(("srh", top), f)

    def uh(self, bottom=2000.0, top=5000.0):
        """m2 s-2 (get_uh)."""
        def f():
            u = self.u_mass()
            return self._call(w.udhel, u, self.stag_height(), self.raw("MAPFAC_M"), u, self.v_mass(), self.raw("W"),
                              float(self.attr("DX")), float(self.attr("DY")), float(bottom), float(top))
        return self._m(("uh", bottom, top), f)

    def dbz(self, use_varint=False, use_liqskin=False):
        """dBZ (get_dbz)."""
        def f():
            qv = self.raw("QVAPOR")
            qr = self.raw("QRAIN")
            qs = self.raw("QSNOW") if self.has("QSNOW") else np.zeros(qv.shape, qv.dtype)
            qg = self.raw("QGRAUP") if self.has("QGRAUP") else np.zeros(qv.shape, qv.dtype)
            p = self.pressure()
            return self._call(w.dbz, p, p, self.tk(), qv, qr, qs, qg, 1 if qs.any() else 0,
                              1 if use_varint else 0, 1 if use_liqskin else 0)
        return self._m(("dbz", use_varint, use_liqskin), f)

    def cloudfrac(self, vert_type="height_agl"):
        """(low, mid, high) fractions, missing = wrfpy.DEFAULT_FILL (get_cloudfrac): layer bounds 300, 2000, 6000 m
        above ground (wrf-python's default vert_type "height_agl"), or 970, 800, 450 hPa (vert_type "pres")."""
        def f():
            p = self.pressure()
            rhv = self._call(w.rh, self.raw("QVAPOR"), self.raw("QVAPOR"), p, self.tk())
            if vert_type in ("pres", "pressure"):
                return self._call(w.cloudfrac, p, p, rhv, 0, 97000., 80000., 45000., w.DEFAULT_FILL)
            v = self.height(msl=(vert_type == "height_msl"))
            return self._call(w.cloudfrac, v, v, rhv, 1, 300., 2000., 6000., w.DEFAULT_FILL)
        return self._m(("cloudfrac", vert_type), f)

    def ctt(self, fill_nocloud=False, opt_thresh=1.0):
        """degC (get_ctt)."""
        def f():
            qv = self.raw("QVAPOR") * 1000.0
            haveqci = 1 if self.has("QICE") else 0
            qice = self.raw("QICE") * 1000.0 if haveqci else np.zeros(qv.shape, qv.dtype)
            qcld = self.raw("QCLOUD") * 1000.0
            p_hpa = self.pressure() * PA_TO_HPA
            ght = destagger(self.raw("PH") + self.raw("PHB"), 0) / w.G
            return self._call(w.ctt, p_hpa, p_hpa, self.tk(), qice, qcld, qv, ght, self.raw("HGT"), haveqci,
                              1 if fill_nocloud else 0, w.DEFAULT_FILL, float(opt_thresh))
        return self._m(("ctt", fill_nocloud, opt_thresh), f)

    # -- winds ---------------------------------------------------------------------------------------------------------
    def rotate(self, u, v, method="cosalpha"):
        """Grid-relative (u, v) on the mass grid to earth-relative: "cosalpha" with the file's COSALPHA / SINALPHA
        (WRF's own rotation), "uvmet" with wrf-python's cone formula (g_uvmet.py, DCOMPUTEUVMET)."""
        if method == "cosalpha" and self.has("COSALPHA") and self.has("SINALPHA"):
            ca = self.raw("COSALPHA").astype(np.float64)
            sa = self.raw("SINALPHA").astype(np.float64)
            u = np.asarray(u, dtype=np.float64)
            v = np.asarray(v, dtype=np.float64)
            return u * ca - v * sa, v * ca + u * sa
        mp = int(self.attr("MAP_PROJ", 0))
        cone = w.uvmet_cone(mp, float(self.attr("TRUELAT1", 0.0)), float(self.attr("TRUELAT2", 0.0)))
        if cone is None:
            return np.asarray(u, dtype=np.float64), np.asarray(v, dtype=np.float64)
        cen = self.attr("STAND_LON", self.attr("CEN_LON"))
        out = self._call(w.uvmet, u, u, v, self.raw("XLAT"), self.raw("XLONG"), float(cen), float(cone))
        return tuple(np.asarray(o, dtype=np.float64) for o in out)

    def uv10_earth(self, method="cosalpha"):
        return self._m(("uv10", method), lambda: self.rotate(self.raw("U10"), self.raw("V10"), method))

    def uv_earth(self, method="cosalpha"):
        return self._m(("uv", method), lambda: self.rotate(self.u_mass(), self.v_mass(), method))

    # -- vertical interpolation (interp.py) ----------------------------------------------------------------------------
    def vinterp(self, field, vert, levels, field_type=None, log_p=False, extrapolate=False):
        """wrf-python's vinterp for vert "pressure" (levels hPa) or "ght_agl" / "ght_msl" (levels km); missing values
        are wrfpy.DEFAULT_FILL."""
        icase_lookup = {None: 0, "none": 0, "p": 1, "pres": 1, "pressure": 1, "z": 2, "ght": 2, "tc": 3, "tk": 4,
                        "theta": 5, "th": 5, "theta-e": 6, "thetae": 6, "eth": 6}
        icase, extrap = (icase_lookup[field_type], 1) if extrapolate else (0, 0)
        sfp = self.raw("PSFC") * PA_TO_HPA
        p = self.pressure()
        ght = self.height(msl=True)
        smsfp = self._m("smsfp", lambda: self._call(w.filter2d, sfp, sfp, 3, w.DEFAULT_FILL, 2.0))
        if vert in ("pressure", "pres", "p"):
            vcor, vc = 1, p * PA_TO_HPA
        elif vert == "ght_msl":
            vcor, vc = 2, np.exp(-ght / w.SCLHT)
        elif vert == "ght_agl":
            vcor, vc = 3, np.exp(-self.height(msl=False) / w.SCLHT)
        else:
            raise ValueError(f"vinterp: vertical coordinate {vert!r}")
        return self._call(w.vintrp, field, field, p, self.tk(), self.raw("QVAPOR"), ght, self.raw("HGT"), sfp, smsfp,
                          vc, np.asarray(levels, dtype=np.float64), icase, extrap, vcor, 1 if log_p else 0,
                          w.DEFAULT_FILL)

    def interplevel(self, field, vert, levels):
        """wrf-python's interplevel (DINTERP3DZ): linear in `vert`; missing = wrfpy.DEFAULT_FILL."""
        return self._call(w.interpz3d, field, field, vert, np.atleast_1d(np.asarray(levels, dtype=np.float64)),
                          w.DEFAULT_FILL)


def _cape_lowest(prs, tmk, qvp, ght, ter, sfp, cmsg=w.DEFAULT_FILL):
    nz = prs.shape[0]
    p, t, q, z = w._top_first(prs, tmk, qvp, ght)
    k = np.full(p.shape[1], nz)
    tlcl, eth, zlcl = w._parcel_props(t[nz - 1], q[nz - 1], p[nz - 1], z[nz - 1], False)
    cape, cin, _, _ = w._lift(p, t, q, z, k, tlcl, eth, zlcl, cmsg)
    return cape.reshape(prs.shape[1:]), cin.reshape(prs.shape[1:])
