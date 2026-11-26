# Licensing summary

CPPWRF's own code and its ports of public-domain WRF-ARW modules are licensed under the GNU General Public
License, version 3 or later (`GPL-3.0-or-later`; see `LICENSE`). Parts of the repository derived from third-party
works carry compound `SPDX-License-Identifier` tags and remain subject to their upstream terms as detailed in
`NOTICE.md`.

In particular, the parts derived from AER's RRTMG / RRTM and UCAR's Noah-MP are **not** under the GPL: their terms
prohibit selling the software or works containing it. `NOTICE.md` grants a GNU GPL version 3 section 7 additional
permission allowing CPPWRF's own code to be combined and conveyed with those parts under AER's and UCAR's terms.
Because those parts are included in this repository, CPPWRF as distributed here may be used, modified and
redistributed free of charge, but not sold.

| Component | Files | Licence | Source |
|---|---|---|---|
| CPPWRF original code and public-domain WRF v4.8.0 ports | All files not listed below (including `src/phys/ra_rrtmg_common/{batch_memory.*,cam_gases.*,cam_ozone.*,fortran_read.hpp,strided.hpp,table_dump.hpp}`, `data/wrf_schema/wrf-4.8.0.tsv`, `tools/wrf_max_domains.py`) | `GPL-3.0-or-later` (WRF upstream: public domain; `licenses/WRF-LICENSE.txt`) | CPPWRF; WRF-ARW v4.8.0 (NCAR / UCAR) |
| RRTMG longwave radiation | `src/phys/ra_rrtmg_lw/*` (11 files) | `GPL-3.0-or-later AND LicenseRef-AER-RRTMG` (`licenses/LicenseRef-AER-RRTMG.txt`, no sale; GPLv3 section 7 permission in `NOTICE.md`) | AER RRTMG in WRF v4.8.0 (`phys/module_ra_rrtmg_lw.F`) |
| RRTMG shortwave radiation | `src/phys/ra_rrtmg_sw/*` (12 files) | `GPL-3.0-or-later AND LicenseRef-AER-RRTMG` (`licenses/LicenseRef-AER-RRTMG.txt`, no sale; GPLv3 section 7 permission in `NOTICE.md`) | AER RRTMG in WRF v4.8.0 (`phys/module_ra_rrtmg_sw.F`) |
| RRTMG shared PRNG / kind definitions | `src/phys/ra_rrtmg_common/kissvec.hpp`, `src/phys/ra_rrtmg_common/parkind.hpp` | `GPL-3.0-or-later AND LicenseRef-AER-RRTMG` (`licenses/LicenseRef-AER-RRTMG.txt`, no sale; GPLv3 section 7 permission in `NOTICE.md`) | AER RRTMG in WRF v4.8.0 (`phys/module_ra_rrtmg_lw.F`, `phys/module_ra_rrtmg_sw.F`) |
| RRTMG precomputed binary tables | `data/rrtmg/rrtmg_lw_tables.bin`, `data/rrtmg/rrtmg_sw_tables.bin`, `data/rrtmg/rrtmg_lw_tables_r4.bin`, `data/rrtmg/rrtmg_sw_tables_r4.bin` | `LicenseRef-AER-RRTMG` (`licenses/LicenseRef-AER-RRTMG.txt`, no sale) | AER RRTMG table data dumped from WRF v4.8.0 (`RRTMG_LW_DATA*`, `RRTMG_SW_DATA*`) |
| RRTM longwave radiation and data | `src/phys/ra_rrtm_lw/*` (5 files, including `rrtm_lw_data.hpp`) | `GPL-3.0-or-later AND LicenseRef-AER-RRTMG` (`licenses/LicenseRef-AER-RRTMG.txt`, no sale; GPLv3 section 7 permission in `NOTICE.md`) | AER RRTM in WRF v4.8.0 (`phys/module_ra_rrtm.F`) |
| Noah-MP land surface model | `src/phys/lsm_noahmp/*` (9 files) | `GPL-3.0-or-later AND LicenseRef-UCAR-NoahMP` (`licenses/LicenseRef-UCAR-NoahMP.txt` and `licenses/Noah-MP-LICENSE.txt`, no sale; GPLv3 section 7 permission in `NOTICE.md`) | UCAR Noah-MP v5.1.2 in WRF v4.8.0 (`phys/noahmp`) |
| Post-processing diagnostics (`cppwrf.post`) | `python/cppwrf/post/wrfpy.py`, `python/cppwrf/post/frame.py` | `GPL-3.0-or-later AND Apache-2.0` (`licenses/WRF-PYTHON-LICENSE.txt`) | wrf-python 1.3.4.1 (UCAR / NCAR; includes RIP and NCL routines) |
| Pseudoadiabat lookup table | `python/cppwrf/post/data/psadilookup.dat` | `Apache-2.0` (`licenses/WRF-PYTHON-LICENSE.txt`) | wrf-python 1.3.4.1 (UCAR / NCAR, unchanged) |
| Double-precision libm and SSE4 libmvec (double and float) math ports and tables | `src/core/glibc_math.hpp`, `src/core/glibc_math_tables.hpp`, `src/core/libmvec_x86.hpp`, `src/core/libmvec_tables.hpp`, `src/core/libmvec_f4.hpp`, `src/core/libmvec_f4_tables.hpp`, `src/core/libmvec_sse4_trig.hpp` | `GPL-3.0-or-later AND LGPL-2.1-or-later` | GNU C Library 2.41 (`libm` and `libmvec`, Debian 13 x86-64) |
| Single-precision libm + CORE-MATH port | `src/core/glibc_mathf.hpp` | `GPL-3.0-or-later AND LGPL-2.1-or-later AND MIT` | GNU C Library 2.41 (`libm`, Debian 13 x86-64) and CORE-MATH (`atanf`, `atan2f`, `tanf`, `tanhf`, `log10f`, `asinf`, `acosf`, `erff`, `tgammaf`, `sinhf`) |

See `LICENSE`, `NOTICE.md` and `licenses/` for the full licence texts, required attributions and build/run-time
external library notices.
