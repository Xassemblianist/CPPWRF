# SPDX-License-Identifier: GPL-3.0-or-later
"""A minimal GRIB edition 2 encoder in numpy (WMO Manual on Codes, FM 92 GRIB edition 2).

One message per field and level: Section 0 (indicator), 1 (identification), 3 (grid: grid.py), 4 (product: Templates
4.0 instantaneous and 4.8 statistically processed), 5 (Template 5.0 simple packing), 6 (bitmap where values are
missing), 7 (data), 8 ("7777"). With packing "jpeg" or "ccsds" the message is re-packed by ecCodes (its Python
bindings must be installed); simple packing needs nothing but numpy.

Simple packing stores Y * 10**D as R + X * 2**E, X an unsigned integer of nbits bits: D (the decimal scale factor)
sets the precision, 10**-D; E is 0 unless the range needs more than 30 bits."""
import datetime
import struct

import numpy as np


def u8(v):
    return struct.pack(">B", v)


def u16(v):
    return struct.pack(">H", v)


def u32(v):
    return struct.pack(">I", v)


def s16(v):
    """GRIB2 signed integers are sign and magnitude, not two's complement."""
    return struct.pack(">H", (0x8000 | -v) if v < 0 else v)


def s32(v):
    return struct.pack(">I", (0x80000000 | -v) if v < 0 else v)


def deg(x):
    """Degrees in GRIB2's units of 1e-6 degree."""
    return int(round(float(x) * 1e6))


MISSING_SURFACE = (255, 255, 0xFFFFFFFF)


def section1(centre, subcentre, tables_version, ref_time, production_status=2, data_type=1):
    t = ref_time
    body = (u16(centre) + u16(subcentre) + u8(tables_version) + u8(0) + u8(1) + u16(t.year) + u8(t.month) + u8(t.day)
            + u8(t.hour) + u8(t.minute) + u8(t.second) + u8(production_status) + u8(data_type))
    return u32(len(body) + 5) + u8(1) + body


def _surface(s):
    if s is None:
        s = MISSING_SURFACE
    typ, scale, value = s
    if typ == 255:
        return u8(255) + u8(255) + u32(0xFFFFFFFF)
    return u8(typ) + (u8(scale & 0xFF) if scale >= 0 else u8(0x80 | -scale)) + s32(int(value))


def _time_unit(seconds):
    """(Code Table 4.4 unit, value): hours when whole, else minutes, else seconds."""
    seconds = int(round(seconds))
    if seconds % 3600 == 0:
        return 1, seconds // 3600
    if seconds % 60 == 0:
        return 0, seconds // 60
    return 13, seconds


def section4(category, number, level, level2=None, lead_seconds=0, stat=None, valid_time=None,
             stat_seconds=0):
    """Template 4.0, or 4.8 when stat is (process, start_lead_seconds, length_seconds): Code Table 4.10 process (1
    accumulation, 2 maximum) over [start, start + length] after the reference time, ending at valid_time."""
    if stat is None:
        unit, ft = _time_unit(lead_seconds)
        tmpl = 0
    else:
        process, start, length = stat
        unit, ft = _time_unit(start)
        tmpl = 8
    body = (u16(0) + u16(tmpl) + u8(category) + u8(number) + u8(2) + u8(255) + u8(255) + u16(0) + u8(0) + u8(unit)
            + u32(ft) + _surface(level) + _surface(level2))
    if stat is not None:
        t = valid_time
        lunit, lval = _time_unit(length)
        body += (u16(t.year) + u8(t.month) + u8(t.day) + u8(t.hour) + u8(t.minute) + u8(t.second) + u8(1) + u32(0)
                 + u8(process) + u8(2) + u8(lunit) + u32(lval) + u8(lunit) + u32(0))
    return u32(len(body) + 5) + u8(4) + body


def pack_simple(values, decimal_scale):
    """Sections 5, 6 and 7 for a 2-D field (NaN = missing): Template 5.0, a bitmap when anything is missing.
    Returns (bytes, number of points packed)."""
    flat = np.asarray(values, dtype=np.float64).ravel()  # +i fastest, rows from the first (south) row: scan 0x40
    present = np.isfinite(flat)
    data = flat[present]
    D = int(decimal_scale)
    E = 0
    if data.size:
        scaled = data * 10.0 ** D
        lo = float(scaled.min())
        R = np.float32(lo)
        if float(R) > lo:
            R = np.nextafter(R, np.float32(-np.inf))
        span = float(scaled.max()) - float(R)
        while span / 2.0 ** E >= 2.0 ** 30:
            E += 1
        X = np.rint((scaled - float(R)) / 2.0 ** E).astype(np.uint64)
        xmax = int(X.max())
        nbits = xmax.bit_length()
        if nbits == 0:  # a constant field: R is the value itself (ecCodes ignores D when there are no bits)
            D = 0
            R = np.float32(data.min())
    else:
        R, X, nbits = np.float32(0.0), np.zeros(0, np.uint64), 0
    sec5 = (u32(int(data.size)) + u16(0) + struct.pack(">f", float(R)) + s16(E) + s16(D) + u8(nbits) + u8(0))
    sec5 = u32(len(sec5) + 5) + u8(5) + sec5
    if present.all():
        sec6 = u32(6) + u8(6) + u8(255)
    else:
        bm = np.packbits(present.astype(np.uint8)).tobytes()
        sec6 = u32(6 + len(bm)) + u8(6) + u8(0) + bm
    if nbits:
        shifts = np.arange(nbits - 1, -1, -1, dtype=np.uint64)
        bits = ((X[:, None] >> shifts[None, :]) & np.uint64(1)).astype(np.uint8)
        payload = np.packbits(bits.ravel()).tobytes()
    else:
        payload = b""
    sec7 = u32(5 + len(payload)) + u8(7) + payload
    return sec5 + sec6 + sec7, int(data.size)


def message(discipline, sec1, sec3, sec4, sec567):
    body = sec1 + sec3 + sec4 + sec567 + b"7777"
    total = 16 + len(body)
    return b"GRIB" + u16(0) + u8(discipline) + u8(2) + struct.pack(">Q", total) + body


def repack(msg, packing):
    """A message re-packed by ecCodes: packing "jpeg" (grid_jpeg) or "ccsds" (grid_ccsds)."""
    try:
        import eccodes
    except ImportError as e:
        raise RuntimeError(f"GRIB2 packing {packing!r} needs ecCodes' Python bindings (pip install eccodes)") from e
    h = eccodes.codes_new_from_message(msg)
    try:
        eccodes.codes_set(h, "packingType", {"jpeg": "grid_jpeg", "ccsds": "grid_ccsds"}[packing])
        return eccodes.codes_get_message(h)
    finally:
        eccodes.codes_release(h)


class Writer:
    """Messages of one valid time, appended to a file."""

    def __init__(self, path, grid, ref_time, valid_time, centre=255, subcentre=0, tables_version=30,
                 production_status=2, packing="simple"):
        self.path = path
        self.ref_time = ref_time
        self.valid_time = valid_time
        self.lead = (valid_time - ref_time).total_seconds()
        self.sec1 = section1(centre, subcentre, tables_version, ref_time, production_status,
                             1 if self.lead > 0 else 0)
        self.sec3 = grid.grib2_section3()
        self.packing = packing
        self.f = open(path, "wb")
        self.count = 0

    def write(self, values, grib, level_value=None, decimal_scale=2, interval_seconds=0):
        """One field (2-D array in GRIB units, NaN missing) with its product's grib entry; level_value fills a level
        whose value is None (pressure in Pa, height in m)."""
        def lev(s):
            if s is None:
                return None
            typ, scale, val = s
            if val is None:
                val, scale = level_value, 0
                while abs(val - round(val)) > 1e-9 and scale < 6:
                    val, scale = val * 10, scale + 1
                val = int(round(val))
            return typ, scale, val
        stat = None
        kind = grib.get("stat")
        if kind == "acc":
            stat = (1, 0, self.lead)
        elif kind == "acc_interval":
            stat = (1, self.lead - interval_seconds, interval_seconds)
        elif kind == "max_output":
            stat = (2, self.lead - interval_seconds, interval_seconds)
        if stat is not None and (stat[2] <= 0 or stat[1] < 0):
            return False
        sec4 = section4(grib["category"], grib["number"], lev(grib["level"]), lev(grib.get("level2")), self.lead, stat,
                        self.valid_time)
        sec567, _ = pack_simple(values, decimal_scale)
        msg = message(grib["discipline"], self.sec1, self.sec3, sec4, sec567)
        if self.packing in ("jpeg", "ccsds"):
            msg = repack(msg, self.packing)
        self.f.write(msg)
        self.count += 1
        return True

    def close(self):
        self.f.close()


def decode_simple(msg):
    """Values of a message of this encoder (Template 5.0, optional bitmap): for the unit tests. Returns (discipline,
    category, number, values with NaN)."""
    assert msg[:4] == b"GRIB" and msg[7] == 2
    discipline = msg[6]
    pos = 16
    category = number = None
    npts = R = E = D = nbits = None
    bitmap = None
    out = None
    while msg[pos:pos + 4] != b"7777":
        length = struct.unpack(">I", msg[pos:pos + 4])[0]
        num = msg[pos + 4]
        sec = msg[pos:pos + length]
        if num == 3:
            npts = struct.unpack(">I", sec[6:10])[0]
        elif num == 4:
            category, number = sec[9], sec[10]
        elif num == 5:
            R = struct.unpack(">f", sec[11:15])[0]
            e, d = struct.unpack(">H", sec[15:17])[0], struct.unpack(">H", sec[17:19])[0]
            E = -(e & 0x7FFF) if e & 0x8000 else e
            D = -(d & 0x7FFF) if d & 0x8000 else d
            nbits = sec[19]
        elif num == 6:
            if sec[5] == 0:
                bitmap = np.unpackbits(np.frombuffer(sec[6:], np.uint8))[:npts].astype(bool)
        elif num == 7:
            present = bitmap if bitmap is not None else np.ones(npts, bool)
            n = int(present.sum())
            if nbits:
                bits = np.unpackbits(np.frombuffer(sec[5:], np.uint8))[:n * nbits].reshape(n, nbits)
                X = (bits.astype(np.uint64) << np.arange(nbits - 1, -1, -1, dtype=np.uint64)).sum(axis=1)
            else:
                X = np.zeros(n)
            vals = (R + X.astype(np.float64) * 2.0 ** E) / 10.0 ** D
            out = np.full(npts, np.nan)
            out[present] = vals
        pos += length
    return discipline, category, number, out


def split(data):
    """The messages of a GRIB file's bytes."""
    out = []
    pos = 0
    while pos < len(data):
        i = data.find(b"GRIB", pos)
        if i < 0:
            break
        total = struct.unpack(">Q", data[i + 8:i + 16])[0]
        out.append(data[i:i + total])
        pos = i + total
    return out


def reference_time(attrs):
    """The forecast's reference time: SIMULATION_START_DATE (else START_DATE) of the wrfout attributes."""
    s = attrs.get("SIMULATION_START_DATE") or attrs.get("START_DATE")
    return datetime.datetime.strptime(str(s).strip()[:19], "%Y-%m-%d_%H:%M:%S")
