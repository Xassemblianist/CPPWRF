# SPDX-License-Identifier: GPL-3.0-or-later
"""cppwrf-post's configuration: the defaults, and a post.yaml or post.ini that changes them (docs/POST.md).

Both formats have the same sections: output, fields, levels, units, options, netcdf, grib2 (and grib2.precision).
A key that is not in the defaults is an error, so that a typo does not pass silently."""
import configparser
import copy
import os

DEFAULT_PRESSURE_LEVELS = [1000, 925, 850, 700, 500, 300, 250, 200]

DEFAULTS = {
    "output": {
        "format": "netcdf",  # netcdf | grib2 | both
        "dir": ".",
        # per wrfout time; {domain} (d01), {time} (YYYY-MM-DD_hh:mm:ss), {lead} (hours, 3 digits)
        "filename": "wrfpost_{domain}_{time}",
    },
    "fields": {
        "surface": "all",  # "all" or a list of names (docs/POST.md)
        "pressure": ["T", "TD", "RH", "Q", "GHT", "U", "V", "W", "OMEGA", "AVO", "DBZ"],
        "height": ["U", "V", "WSPD", "WDIR"],
    },
    "levels": {
        "pressure": DEFAULT_PRESSURE_LEVELS,  # hPa
        "height": [80, 100, 120],  # m above ground
    },
    "units": {  # NetCDF output units per field (GRIB2 always uses WMO's)
        "MSLP": "hPa", "PSFC": "hPa", "TD2": "degC", "CTT": "degC", "TD": "degC",
    },
    "options": {
        "wind_rotation": "cosalpha",  # cosalpha (WRF's COSALPHA / SINALPHA) | uvmet (wrf-python's cone formula)
        "log_p": True,  # vinterp in log pressure
        "extrapolate": False,  # vinterp below ground (RIP's standard atmosphere) instead of missing
        "dbz_varint": False,  # calc_dbz options (wrf-python's use_varint, use_liqskin)
        "dbz_liqskin": False,
        "max_dbz": "auto",  # auto: REFL_10CM of the file when present, else computed | model | computed
        "cloud_vert_type": "height_agl",  # wrf-python's cloudfrac vert_type: height_agl | height_msl | pres
        "precip_hours": 1,  # interval of the PRECIP_<n>H fields
        "ml_depth_hpa": 100.0,  # mixed-layer parcel depth
        "emulate_wrfpython": True,  # round intermediate results to the file's precision, as wrf-python does
    },
    "netcdf": {
        "dtype": "float32",
        "compression": 4,  # zlib level, 0 = none
    },
    "grib2": {
        "centre": 255,  # originating centre (Code Table C-11); 255 = missing
        "subcentre": 0,
        "tables_version": 30,
        "production_status": 2,  # Code Table 1.3: 2 = research
        "packing": "simple",  # simple (Template 5.0) | jpeg | ccsds (re-packed with ecCodes, if installed)
        "precision": {},  # decimal scale factor D per field (values kept to 10**-D); defaults in products.py
    },
}


class ConfigError(ValueError):
    pass


def _parse_value(text):
    t = text.strip()
    if t.lower() in ("true", "yes", "on"):
        return True
    if t.lower() in ("false", "no", "off"):
        return False
    if "," in t:
        return [_parse_value(x) for x in t.split(",") if x.strip()]
    for typ in (int, float):
        try:
            return typ(t)
        except ValueError:
            pass
    return t


def _read_ini(path):
    cp = configparser.ConfigParser(inline_comment_prefixes=("#", ";"))
    cp.optionxform = str  # field names keep their case
    with open(path) as f:
        cp.read_file(f)
    out = {}
    for sec in cp.sections():
        vals = {k: _parse_value(v) for k, v in cp.items(sec)}
        if sec == "grib2.precision":
            out.setdefault("grib2", {})["precision"] = vals
        else:
            out.setdefault(sec, {}).update(vals)
    return out


def _read_yaml(path):
    try:
        import yaml
    except ImportError as e:
        raise ConfigError(f"{path}: reading YAML needs PyYAML (pip install pyyaml), or use the .ini form") from e
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: the top level must be a mapping of sections")
    return data


def _merge(base, extra, where=""):
    for key, val in extra.items():
        if key not in base:
            raise ConfigError(f"unknown setting {where}{key}")
        if isinstance(base[key], dict) and key not in ("units", "precision"):
            if not isinstance(val, dict):
                raise ConfigError(f"{where}{key} must be a section")
            _merge(base[key], val, f"{where}{key}.")
        elif isinstance(base[key], dict):
            base[key].update({str(k): v for k, v in (val or {}).items()})
        else:
            base[key] = val


def load(path=None, overrides=None):
    """The defaults updated with the file at `path` (.yaml / .yml / .ini / .cfg) and then with `overrides` (a dict of
    the same shape)."""
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        ext = os.path.splitext(path)[1].lower()
        data = _read_yaml(path) if ext in (".yaml", ".yml") else _read_ini(path)
        _merge(cfg, data)
    if overrides:
        _merge(cfg, overrides)
    for sec, key in (("fields", "pressure"), ("fields", "height"), ("levels", "pressure"), ("levels", "height")):
        v = cfg[sec][key]
        if v in (None, "", "none"):
            cfg[sec][key] = []
        elif not isinstance(v, list):
            cfg[sec][key] = [v]
    if cfg["output"]["format"] not in ("netcdf", "grib2", "both"):
        raise ConfigError(f"output.format {cfg['output']['format']!r}: netcdf, grib2 or both")
    return cfg


def dump_ini(cfg):
    """A configuration as .ini text (cppwrf-post --print-config)."""
    lines = []
    for sec, vals in cfg.items():
        sub = {k: v for k, v in vals.items() if not isinstance(v, dict)}
        lines.append(f"[{sec}]")
        for k, v in sub.items():
            lines.append(f"{k} = {', '.join(map(str, v)) if isinstance(v, list) else v}")
        lines.append("")
        for k, v in vals.items():
            if isinstance(v, dict) and k not in ("units",):
                lines.append(f"[{sec}.{k}]")
                lines += [f"{kk} = {vv}" for kk, vv in v.items()]
                lines.append("")
    return "\n".join(lines)
