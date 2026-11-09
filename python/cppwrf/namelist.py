# SPDX-License-Identifier: GPL-3.0-or-later
"""WRF namelist.input: read, write, translate to a CPPWRF configuration and check what CPPWRF ports.

The parsing and the translation are tools/wrf2cppwrf.py's, imported as a library: what this module reports is what
the translator (and so the model) does, never a second copy of its option tables."""
import importlib
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from . import _locate

Namelist = Dict[str, Dict[str, List[Any]]]


def translator():
    """The tools/wrf2cppwrf.py module."""
    d = _locate.tools_dir()
    if d not in sys.path:
        sys.path.insert(0, d)
    return importlib.import_module("wrf2cppwrf")


def parse(text: str) -> Namelist:
    """A namelist's text as {group: {option: [values]}} (lower-case names; Fortran syntax: repeat counts, d exponents,
    logicals, comments)."""
    return translator().parse_namelist(text)


def read(path: str) -> Namelist:
    with open(path, encoding="utf-8", errors="replace") as f:
        return parse(f.read())


def _fortran(v: Any) -> str:
    if isinstance(v, bool):
        return ".true." if v else ".false."
    if isinstance(v, str):
        return "'" + v.replace("'", "''") + "'"
    if v is None:
        return ""
    if isinstance(v, float):
        return repr(v)
    return str(v)


def dumps(nml: Namelist) -> str:
    """A namelist as Fortran namelist text (one option per line, the domains' values on the line)."""
    out = []
    for grp, opts in nml.items():
        out.append(f"&{grp}")
        for k, vals in opts.items():
            out.append(f" {k:<36s}= " + ", ".join(_fortran(v) for v in vals) + ",")
        out.append("/")
        out.append("")
    return "\n".join(out)


def write(nml: Namelist, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(dumps(nml))


def get(nml: Namelist, group: str, key: str, default: Any = None, dom: int = 0) -> Any:
    vals = nml.get(group, {}).get(key, [])
    if not vals:
        return default
    return vals[dom] if dom < len(vals) else vals[0]


@dataclass
class Translation:
    """tools/wrf2cppwrf.py's result: the configuration (.ini text and sections) and its messages."""
    ini: str
    sections: Dict[str, Dict[str, Any]]
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    notices: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def translate(nml: Any, namelist_path: str = "", output_format: str = "wrfout", real_dir: Optional[str] = None,
              table_dir: Optional[str] = None, case: Optional[str] = None) -> Translation:
    """Translate a namelist (a path or a parsed Namelist) to a CPPWRF configuration, as
    `tools/wrf2cppwrf.py NAMELIST --format FORMAT` does."""
    t = translator()
    if isinstance(nml, str):
        namelist_path = namelist_path or nml
        nml = read(nml)
    tr = t.WrfTranslator(nml, namelist_path=namelist_path, override_case=case, real_dir=real_dir,
                         table_dir=table_dir, output_format=output_format)
    sections = tr.translate()
    ini = tr.format_ini(sections)
    return Translation(ini, sections, list(tr.errors), list(tr.warnings), list(tr.notices))


# -----------------------------------------------------------------------------------------------------------------
# restart = .true.: cppwrf-wrf resumes from the wrfrst files (the translator itself leaves restarts to --set)

def without_restart(nml: Namelist) -> Tuple[Namelist, bool]:
    """(a copy of the namelist with restart = .false., whether it was .true.)."""
    import copy
    n = copy.deepcopy(nml)
    on = bool(get(n, "time_control", "restart", False))
    if on:
        n["time_control"]["restart"] = [False]
    return n, on


def start_date(nml: Namelist, dom: int = 0) -> str:
    """The namelist's start date of a domain, WRF's YYYY-MM-DD_hh:mm:ss."""
    g = lambda k, d=0: int(get(nml, "time_control", k, d, dom))  # noqa: E731
    return (f"{g('start_year', 2000):04d}-{g('start_month', 1):02d}-{g('start_day', 1):02d}_"
            f"{g('start_hour'):02d}:{g('start_minute'):02d}:{g('start_second'):02d}")


def stream_filename(pattern: str, dom: int, date: str) -> str:
    """WRF's construct_filename2a: <domain>, <date>, <year>, <month>, <day>, <hour> in a stream's file name."""
    return (pattern.replace("<domain>", f"{dom:02d}").replace("<date>", date).replace("<year>", date[0:4])
            .replace("<month>", date[5:7]).replace("<day>", date[8:10]).replace("<hour>", date[11:13]))


# -----------------------------------------------------------------------------------------------------------------
# check: per option, ported / ignored / not ported

PORTED, IGNORED, NOT_PORTED = "ported", "ignored", "not ported"


@dataclass
class OptionStatus:
    group: str
    option: str
    values: List[Any]
    status: str  # PORTED, IGNORED or NOT_PORTED
    reason: str = ""
    suggestion: str = ""


@dataclass
class CheckResult:
    options: List[OptionStatus]
    general: List[str]  # errors about combinations of options (no single option to blame)
    translation: Translation

    @property
    def ok(self) -> bool:
        return not self.translation.errors

    def by_status(self, status: str) -> List[OptionStatus]:
        return [o for o in self.options if o.status == status]


def _mentions(text: str, group: str, key: str) -> bool:
    return re.search(rf"(?<![A-Za-z0-9_]){re.escape(key)}(?![A-Za-z0-9_])", text) is not None


def _suggestion(t, group: str, key: str, message: str) -> str:
    if "is not a WRF 4.8.0 namelist option" in message:
        return "check the spelling and the group (wrf.exe stops on it too)"
    if key in t.SUPPORTED_OPTIONS:
        return "use one of " + ", ".join(str(k) for k in t.SUPPORTED_OPTIONS[key])
    if (group, key) in t.DEFAULT_ONLY_OPTIONS:
        return f"set {key} = {_fortran(t.DEFAULT_ONLY_OPTIONS[(group, key)][0])} (WRF's default) or remove it"
    if key in t.UNSUPPORTED_PHYSICS_SWITCHES or key in t.UNSUPPORTED_DYNAMICS_SWITCHES:
        return f"set {key} = 0 (off) or remove it"
    if key in getattr(t, "SUPPORTED_NOAHMP_OPTIONS", {}):
        return "use one of " + ", ".join(str(v) for v in t.SUPPORTED_NOAHMP_OPTIONS[key])
    reg = t.REGISTRY_DEFAULTS.get(key)
    if reg is not None and "is not supported by CPPWRF (WRF's default" in message:
        return f"remove it or set WRF's default {_fortran(reg[1])}"
    return "see docs/USER_GUIDE.md, 'Rejected Options and Why'"


def check(path: str, real_dir: Optional[str] = None, table_dir: Optional[str] = None,
          dropin: bool = True) -> CheckResult:
    """What CPPWRF does with every option of a namelist.input, without running anything. dropin: as cppwrf-wrf
    runs it (restart = .true. resumes from the wrfrst files)."""
    t = translator()
    nml = read(path)
    nml_t, restart = without_restart(nml) if dropin else (nml, False)
    tr = translate(nml_t, namelist_path=path, output_format="wrfout", real_dir=real_dir, table_dir=table_dir)
    blamed = set()
    opts = []
    for grp, kv in nml.items():
        for key, vals in kv.items():
            errs = [e for e in tr.errors if _mentions(e, grp, key)]
            note = next((n for n in tr.notices if f"&{grp}.{key} " in n or f"&{grp}.{key}=" in n), None)
            warn = next((w for w in tr.warnings if f"&{grp}.{key} " in w), None)
            if key == "restart" and restart:
                st = OptionStatus(grp, key, vals, PORTED, "cppwrf-wrf resumes from the wrfrst files of the start "
                                  "date (rst_inname)")
            elif errs:
                blamed.update(errs)
                st = OptionStatus(grp, key, vals, NOT_PORTED, " ".join(errs), _suggestion(t, grp, key, errs[0]))
            elif warn:
                st = OptionStatus(grp, key, vals, IGNORED, warn.split(": ", 1)[-1])
            elif note:
                why = note.split(": ", 1)[-1]
                if key in t.NO_EFFECT_OPTIONS:
                    st = OptionStatus(grp, key, vals, IGNORED, f"no effect in WRF either here: {why}")
                elif why.startswith("WRF's default"):
                    st = OptionStatus(grp, key, vals, PORTED, "WRF's default value")
                elif key in t.IGNORED_WRF_OPTIONS:
                    st = OptionStatus(grp, key, vals, PORTED if _read_elsewhere(why) else IGNORED, why)
                else:
                    st = OptionStatus(grp, key, vals, PORTED, why)
            else:
                st = OptionStatus(grp, key, vals, PORTED)
            opts.append(st)
    general = [e for e in tr.errors if e not in blamed]
    return CheckResult(opts, general, tr)


def _read_elsewhere(reason: str) -> bool:
    """IGNORED_WRF_OPTIONS whose value CPPWRF does use (read from the input files, the model's namelist read or
    combined into another key), as opposed to WRF's infrastructure options (logging, quilt servers)."""
    r = reason.lower()
    return not any(w in r for w in ("logging", "quilt", "real.exe", "metgrid", "slab lsm only", "always 1",
                                    "always reads"))


def format_check(res: CheckResult, verbose: bool = False, errors_only: bool = False) -> str:
    """The checker's report: one line per option (the ported ones only with verbose, the not ported ones only with
    errors_only), then the general errors."""
    lines = []
    w = max([len(f"&{o.group}.{o.option}") for o in res.options] + [10])
    for o in res.options:
        if (o.status == PORTED and not verbose) or (errors_only and o.status != NOT_PORTED):
            continue
        val = ", ".join(_fortran(v) for v in o.values[:6]) + (", ..." if len(o.values) > 6 else "")
        lines.append(f"{o.status.upper():11s} {f'&{o.group}.{o.option}':<{w}s} = {val}")
        if o.reason:
            lines.append(f"{'':11s}   {o.reason}")
        if o.suggestion:
            lines.append(f"{'':11s}   suggestion: {o.suggestion}")
    for g in res.general:
        lines.append(f"NOT PORTED  (combination) {g}")
    n = {s: len(res.by_status(s)) for s in (PORTED, IGNORED, NOT_PORTED)}
    lines.append(f"{len(res.options)} options: {n[PORTED]} ported, {n[IGNORED]} ignored, {n[NOT_PORTED]} not ported"
                 + (f", {len(res.general)} unsupported combinations" if res.general else "")
                 + (": CPPWRF runs this namelist." if res.ok else ": CPPWRF cannot run this namelist."))
    return "\n".join(lines)
