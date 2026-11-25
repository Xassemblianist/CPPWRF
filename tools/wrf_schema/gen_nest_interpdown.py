#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Extract WRF's nest interpolation list (the Registry "d" fields) of a compiled WRF tree into a table.

WRF's Registry compiles the interpolation of every "d" (down) field into inc/nest_interpdown_interp.inc, which
interp_domain_em_part3 runs for med_interp_domain: at a nest's start and, with moving nests, after every move
(share/mediation_nest_move.F), where each function writes only the points of the nest whose interpolation mask
(imask_nostag / xstag / ystag / xystag) is 1 (interp_fcnm and interp_mask_water_field write every point). The calls
are guarded by the field's allocation (SIZE > 1), i.e. by the Registry package of the configuration. This script
writes one line per call, in WRF's order (the order matters: interp_mask_field reads the nest's lu_index, which an
earlier line interpolated):

    function  field  xstag  ystag  mask  extra

field: the state variable (varname of data/wrf_schema/wrf-4.8.0.tsv) or the 4-D array (moist, scalar, ...: every
member of the configuration); xstag / ystag: T or F; mask: the imask array; extra: the extra arguments as
"c.NAME,n.NAME,..." (the parent's (c) and the nest's (n) field or scalar), "-" when none.

usage: gen_nest_interpdown.py <WRF build dir> <output .tsv>
"""
import os
import re
import sys


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    inc = open(os.path.join(sys.argv[1], "inc", "nest_interpdown_interp.inc")).read()
    pat = re.compile(r"CALL (\w+) \(  &\s*\n(?:\s*\( SIZE.*\n)?\s*(\S+),\s*&\s*! CD field")
    rows = []
    for m in pat.finditer(inc):
        fn, fld = m.group(1), m.group(2)
        rest = inc[m.end():m.end() + 3000]
        st = re.search(r"\n\s*(\.\w+\.), (\.\w+\.),\s*&\s*! xstag", rest)
        mk = re.search(r"ngrid%(imask_\w+)", rest)
        ex = re.search(r"parent_grid_ratio\s*&\n((?:,.*\n)*)\s*\)", rest)
        if not st or not mk:
            raise SystemExit("unparsed call of %s for %s" % (fn, fld))
        name = fld.replace("grid%", "")
        name = name.split("(")[0]
        extra = "".join(ex.group(1).split()) if ex else ""
        extra = extra.replace("&", "").replace("ngrid%", "n.").replace("grid%", "c.").strip(",")
        tf = {".TRUE.": "T", ".FALSE.": "F"}
        rows.append((fn, name, tf[st.group(1)], tf[st.group(2)], mk.group(1), extra or "-"))
    calls = len(re.findall(r"^CALL ", inc, re.M))
    if calls != len(rows):
        raise SystemExit("parsed %d of %d calls" % (len(rows), calls))
    with open(sys.argv[2], "w") as o:
        o.write("# WRF nest interpolation list (inc/nest_interpdown_interp.inc) extracted by "
                "tools/wrf_schema/gen_nest_interpdown.py from %s\n" % os.path.basename(os.path.abspath(sys.argv[1])))
        o.write("# function\tfield\txstag\tystag\tmask\textra\n")
        for r in rows:
            o.write("\t".join(r) + "\n")
    print("%d calls" % len(rows))


if __name__ == "__main__":
    main()
