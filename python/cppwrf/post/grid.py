# SPDX-License-Identifier: GPL-3.0-or-later
"""The wrfout grid's map projection: CF-1.8 grid_mapping attributes with projection x / y coordinates, and the GRIB2
grid definition (Section 3: Templates 3.30 Lambert conformal, 3.20 polar stereographic, 3.10 Mercator, 3.0
latitude-longitude).

WRF's projections (MAP_PROJ 1 Lambert, 2 polar stereographic, 3 Mercator, 6 latitude-longitude) are on a sphere of
radius 6370 km. x / y are computed with the spherical formulas of each projection (Snyder 1987, "Map projections - a
working manual", USGS PP 1395: Lambert eq. 15-1 to 15-3, 14-4; polar stereographic 21-33 to 21-35; Mercator 7-1,
7-2) from the grid's XLAT / XLONG, then fitted to a regular grid of spacing DX / DY; the fit's largest error is
reported (Grid.fit_error, m) and a projection whose fit is worse than 1 % of DX is not given x / y."""
import math

import numpy as np

R_EARTH = 6370000.0


class Grid:
    def __init__(self, attrs, lat, lon):
        self.attrs = attrs
        self.lat = np.asarray(lat, dtype=np.float64)
        self.lon = np.asarray(lon, dtype=np.float64)
        self.ny, self.nx = self.lat.shape
        self.map_proj = int(attrs.get("MAP_PROJ", 0))
        self.dx = float(attrs.get("DX", 0.0))
        self.dy = float(attrs.get("DY", 0.0))
        self.truelat1 = float(attrs.get("TRUELAT1", 0.0))
        self.truelat2 = float(attrs.get("TRUELAT2", self.truelat1))
        self.stand_lon = float(attrs.get("STAND_LON", attrs.get("CEN_LON", 0.0)))
        self.cen_lat = float(attrs.get("MOAD_CEN_LAT", attrs.get("CEN_LAT", 0.0)))
        self.pole_lat = float(attrs.get("POLE_LAT", 90.0))
        self.x = self.y = None
        self.fit_error = None
        if self.map_proj in (1, 2, 3):
            px, py = self.forward(self.lat, self.lon)
            i = np.arange(self.nx)[None, :] * self.dx
            j = np.arange(self.ny)[:, None] * self.dy
            x0 = float(np.mean(px - i))
            y0 = float(np.mean(py - j))
            err = max(float(np.abs(px - i - x0).max()), float(np.abs(py - j - y0).max()))
            self.fit_error = err
            if err < 0.01 * self.dx:
                self.x = x0 + np.arange(self.nx) * self.dx
                self.y = y0 + np.arange(self.ny) * self.dy

    # -- projection formulas (spherical) -------------------------------------------------------------------------------
    def cone(self):
        p1, p2 = math.radians(self.truelat1), math.radians(self.truelat2)
        if abs(self.truelat1 - self.truelat2) > 1e-6:
            return math.log(math.cos(p1) / math.cos(p2)) / math.log(math.tan(math.pi / 4 + p2 / 2)
                                                                     / math.tan(math.pi / 4 + p1 / 2))
        return math.sin(p1)

    def forward(self, lat, lon):
        """Projection x, y (m) of lat / lon (degrees)."""
        phi = np.radians(lat)
        dl = np.radians(((lon - self.stand_lon + 180.0) % 360.0) - 180.0)
        if self.map_proj == 1:
            n = self.cone()
            p1 = math.radians(self.truelat1)
            sgn = 1.0 if self.truelat1 >= 0 else -1.0
            F = math.cos(p1) * math.tan(math.pi / 4 + sgn * p1 / 2) ** (n * sgn) / (n * sgn)
            rho = R_EARTH * F / np.tan(np.pi / 4 + sgn * phi / 2) ** (n * sgn)
            rho0 = R_EARTH * F / math.tan(math.pi / 4 + sgn * math.radians(self.cen_lat) / 2) ** (n * sgn)
            return rho * np.sin(n * dl), rho0 - rho * np.cos(n * dl)
        if self.map_proj == 2:
            south = self.truelat1 < 0
            pc = math.radians(abs(self.truelat1))
            ph = -phi if south else phi
            rho = R_EARTH * (1 + math.sin(pc)) * np.tan(np.pi / 4 - ph / 2)
            if south:
                return rho * np.sin(dl), rho * np.cos(dl)
            return rho * np.sin(dl), -rho * np.cos(dl)
        if self.map_proj == 3:
            k = math.cos(math.radians(self.truelat1))
            return R_EARTH * k * dl, R_EARTH * k * np.log(np.tan(np.pi / 4 + phi / 2))
        raise ValueError(f"no x / y for MAP_PROJ {self.map_proj}")

    # -- CF ------------------------------------------------------------------------------------------------------------
    def cf_mapping(self):
        """(grid_mapping attributes, has_xy) for the NetCDF output; attributes None for an unsupported projection."""
        base = {"earth_radius": R_EARTH}
        if self.map_proj == 1:
            sp = [self.truelat1, self.truelat2] if abs(self.truelat1 - self.truelat2) > 1e-6 else self.truelat1
            a = dict(grid_mapping_name="lambert_conformal_conic", standard_parallel=sp,
                     longitude_of_central_meridian=self.stand_lon, latitude_of_projection_origin=self.cen_lat,
                     false_easting=0.0, false_northing=0.0, **base)
        elif self.map_proj == 2:
            a = dict(grid_mapping_name="polar_stereographic", straight_vertical_longitude_from_pole=self.stand_lon,
                     latitude_of_projection_origin=90.0 if self.truelat1 >= 0 else -90.0,
                     standard_parallel=self.truelat1, false_easting=0.0, false_northing=0.0, **base)
        elif self.map_proj == 3:
            a = dict(grid_mapping_name="mercator", longitude_of_projection_origin=self.stand_lon,
                     standard_parallel=self.truelat1, false_easting=0.0, false_northing=0.0, **base)
        elif self.map_proj == 6 and abs(self.pole_lat - 90.0) < 1e-6:
            a = dict(grid_mapping_name="latitude_longitude", **base)
        else:
            return None, False
        return a, self.x is not None

    # -- GRIB2 Section 3 -----------------------------------------------------------------------------------------------
    def grib2_section3(self):
        """Section 3 bytes; earth: sphere of radius 6370000 m (Code Table 3.2, 1); scanning +i, +j (WRF's first row is
        the southernmost); u / v relative to east / north (the fields are earth-rotated)."""
        from .grib2 import u8, u16, u32, s32, deg
        lat0, lon0 = self.lat[0, 0], self.lon[0, 0]
        earth = u8(1) + u8(0) + u32(int(R_EARTH)) + u8(0) + u32(0) + u8(0) + u32(0)
        flags = 0x30  # i and j increments given; u, v relative to east and north
        scan = 0x40  # +i, +j, consecutive points in i
        if self.map_proj == 1:
            tmpl = 30
            body = (earth + u32(self.nx) + u32(self.ny) + s32(deg(lat0)) + u32(deg(lon0 % 360.0)) + u8(flags)
                    + s32(deg(self.truelat1)) + u32(deg(self.stand_lon % 360.0)) + u32(round(self.dx * 1000))
                    + u32(round(self.dy * 1000)) + u8(0 if self.truelat1 >= 0 else 0x80) + u8(scan)
                    + s32(deg(self.truelat1)) + s32(deg(self.truelat2)) + s32(deg(-90.0)) + u32(0))
        elif self.map_proj == 2:
            tmpl = 20
            body = (earth + u32(self.nx) + u32(self.ny) + s32(deg(lat0)) + u32(deg(lon0 % 360.0)) + u8(flags)
                    + s32(deg(self.truelat1)) + u32(deg(self.stand_lon % 360.0)) + u32(round(self.dx * 1000))
                    + u32(round(self.dy * 1000)) + u8(0 if self.truelat1 >= 0 else 0x80) + u8(scan))
        elif self.map_proj == 3:
            tmpl = 10
            lat1, lon1 = self.lat[-1, -1], self.lon[-1, -1]
            body = (earth + u32(self.nx) + u32(self.ny) + s32(deg(lat0)) + u32(deg(lon0 % 360.0)) + u8(flags)
                    + s32(deg(self.truelat1)) + s32(deg(lat1)) + u32(deg(lon1 % 360.0)) + u8(scan) + u32(0)
                    + u32(round(self.dx * 1000)) + u32(round(self.dy * 1000)))
        elif self.map_proj == 6 and abs(self.pole_lat - 90.0) < 1e-6:
            tmpl = 0
            lat1, lon1 = self.lat[-1, -1], self.lon[-1, -1]
            di = (lon1 - lon0) % 360.0 / (self.nx - 1) if self.nx > 1 else 0.0
            dj = (lat1 - lat0) / (self.ny - 1) if self.ny > 1 else 0.0
            body = (earth + u32(self.nx) + u32(self.ny) + u32(0) + u32(0xFFFFFFFF) + s32(deg(lat0))
                    + u32(deg(lon0 % 360.0)) + u8(flags) + s32(deg(lat1)) + u32(deg(lon1 % 360.0)) + u32(deg(di))
                    + u32(deg(dj)) + u8(scan))
        else:
            raise ValueError(f"GRIB2 output: MAP_PROJ {self.map_proj} (POLE_LAT {self.pole_lat}) is not supported "
                             "(Lambert, polar stereographic, Mercator and unrotated latitude-longitude are)")
        head = u8(0) + u32(self.nx * self.ny) + u8(0) + u8(0) + u16(tmpl)
        sec = head + body
        return u32(len(sec) + 5) + u8(3) + sec
