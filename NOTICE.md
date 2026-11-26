# Notices

CPPWRF: a C++20/CUDA port of the WRF-ARW model.
Copyright (C) 2026 Ömer Kaan Yılmaz (Xassemblianist)

This program is free software: you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details. You should have received a copy of the GNU General
Public License along with this program (file LICENSE). If not, see <https://www.gnu.org/licenses/>.

Source files written for CPPWRF and ports of public-domain WRF modules carry the tag
`SPDX-License-Identifier: GPL-3.0-or-later`. Source files derived from third-party works carry compound
`SPDX-License-Identifier` expressions naming both licences (using `LicenseRef-*` texts under `licenses/` where no
standard SPDX identifier exists); `docs/LICENSING.md` summarises every component, its files, its licence and its
source. The notices below apply in addition to the GPL to the parts of CPPWRF that are derived from other works.
The parts derived from RRTMG / RRTM (AER) and Noah-MP (UCAR) are an exception: see "Schemes inside WRF with their
own terms" below.

## WRF

CPPWRF is a statement-by-statement port of the Weather Research and Forecasting (WRF) model v4.8.0 (the ARW
dynamical core, `real.exe`, `ideal.exe` and the physics schemes listed in docs/STATUS.md) to C++ and CUDA. The
comments in the sources cite the WRF files and lines that each part follows. Data derived from WRF:

- `data/wrf_schema/wrf-4.8.0.tsv` and `tools/wrf_max_domains.py`: the I/O schema and per-domain Registry defaults
  extracted from WRF's Registry (`tools/wrf_schema/gen_wrf_schema.py`).
- `data/rrtmg/*.bin`: the RRTMG tables as WRF holds them after reading `RRTMG_LW_DATA_DBL` / `RRTMG_SW_DATA_DBL`
  (and `RRTMG_LW_DATA` / `RRTMG_SW_DATA` for the `_r4.bin` files; see below for RRTMG's own notice).
- `src/phys/ra_rrtm_lw/rrtm_lw_data.hpp`: the DATA statements of WRF's `phys/module_ra_rrtm.F`
  (`tools/rrtm_lw_data.py`; see below for RRTM's own notice).
- Constants and coefficient tables in the scheme sources (Kain-Fritsch, WSM6, Thompson, Noah, Noah-MP, MYNN and
  others), taken from the WRF modules they port.

CPPWRF does not ship WRF's table files (`LANDUSE.TBL`, `VEGPARM.TBL`, `SOILPARM.TBL`, `GENPARM.TBL`,
`NoahmpTable.TBL`, the ozone and greenhouse-gas files, `CCN_ACTIVATE.BIN`, Thompson's `.dat` tables): it reads them
from WRF's `run/` directory at run time. The differential tests under `tests/` link the objects of a WRF build that
the user provides; no WRF source file is part of CPPWRF.

The WRF notice, as requested by UCAR (also in licenses/WRF-LICENSE.txt):

> WRF was developed at the National Center for Atmospheric Research (NCAR) which is operated by
> the University Corporation for Atmospheric Research (UCAR). NCAR and UCAR make no proprietary
> claims, either statutory or otherwise, to this version and release of WRF and consider WRF to
> be in the public domain for use by any person or entity for any purpose without any fee or
> charge. UCAR requests that any WRF user include this notice on any partial or full copies of
> WRF. WRF is provided on an "AS IS" basis and any warranties, either express or implied,
> including but not limited to implied warranties of non-infringement, originality,
> merchantability and fitness for a particular purpose, are disclaimed. In no event shall UCAR
> be liable for any damages, whatsoever, whether direct, indirect, consequential or special,
> that arise out of or in connection with the access, use or performance of WRF, including
> infringement actions.

WRF® is a registered trademark of the University Corporation for Atmospheric Research (UCAR). CPPWRF is an
independent project and is not affiliated with or endorsed by NCAR or UCAR.

## Schemes inside WRF with their own terms

**RRTMG and RRTM (AER).** The following files are derived from WRF's `module_ra_rrtmg_lw.F` and
`module_ra_rrtmg_sw.F` (RRTMG, by Atmospheric & Environmental Research, Inc.):

- `src/phys/ra_rrtmg_lw/`: `rrtmg_lw.hpp`, `rrtmg_lw.cpp`, `rrtmg_lw_batch.hpp`, `rrtmg_lw_batch.cpp`,
  `rrtmg_lw_tables.hpp`, `rrtmg_lw_tables.cpp`, `rrtmg_lw_cldprmc.hpp`, `rrtmg_lw_rtrnmc.hpp`,
  `rrtmg_lw_setcoef.hpp`, `rrtmg_lw_stages.hpp`, `rrtmg_lw_taumol.hpp`.
- `src/phys/ra_rrtmg_sw/`: `rrtmg_sw.hpp`, `rrtmg_sw.cpp`, `rrtmg_sw_batch.hpp`, `rrtmg_sw_batch.cpp`,
  `rrtmg_sw_tables.hpp`, `rrtmg_sw_tables.cpp`, `rrtmg_sw_cldprmc.hpp`, `rrtmg_sw_reftra.hpp`,
  `rrtmg_sw_setcoef.hpp`, `rrtmg_sw_stages.hpp`, `rrtmg_sw_taumol.hpp`, `rrtmg_sw_vrtqdr.hpp`.
- `src/phys/ra_rrtmg_common/`: `kissvec.hpp`, `parkind.hpp` (the other files in `src/phys/ra_rrtmg_common/` —
  `batch_memory.hpp`, `batch_memory.cpp`, `cam_gases.hpp`, `cam_gases.cpp`, `cam_ozone.hpp`, `cam_ozone.cpp`,
  `fortran_read.hpp`, `strided.hpp`, `table_dump.hpp` — are CPPWRF's own helpers and ports of WRF's CLWRF / CAM
  modules under `GPL-3.0-or-later`).
- `data/rrtmg/`: `rrtmg_lw_tables.bin`, `rrtmg_sw_tables.bin`, `rrtmg_lw_tables_r4.bin`, `rrtmg_sw_tables_r4.bin`.

The following files are derived from WRF's `module_ra_rrtm.F`, AER's Rapid Radiative Transfer Model (RRTM) as
included in WRF:

- `src/phys/ra_rrtm_lw/`: `rrtm_lw.hpp`, `rrtm_lw.cpp`, `rrtm_lw_tables.hpp`, `rrtm_lw_tables.cpp`,
  `rrtm_lw_data.hpp`.

These carry AER's notice (also in `licenses/LicenseRef-AER-RRTMG.txt`); the source files carry
`SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-AER-RRTMG`:

> Copyright 2002-2008, Atmospheric & Environmental Research, Inc. (AER).
> This software may be used, copied, or redistributed as long as it is
> not sold and this copyright notice is reproduced on each copy made.
> This model is provided as is without any express or implied warranties.
> (http://www.rtweb.aer.com/)

**Noah-MP (UCAR).** `src/phys/lsm_noahmp/` (`noahmp.hpp`, `noahmp.cpp`, `noahmp_tables.hpp`, `noahmp_tables.cpp`,
`noahmp_var.hpp`, `noahmp_rad_therm.hpp`, `noahmp_energy.hpp`, `noahmp_water.hpp`, `noahmp_glacier.hpp`) is
derived from Noah-MP v5.1.2 as distributed with WRF v4.8.0 (`phys/noahmp`). Its licence agreement is reproduced in
`licenses/Noah-MP-LICENSE.txt` and `licenses/LicenseRef-UCAR-NoahMP.txt`; the source files carry
`SPDX-License-Identifier: GPL-3.0-or-later AND LicenseRef-UCAR-NoahMP`. The agreement requires this notice:

> The source of this material is the Research Applications Laboratory at the National Center for Atmospheric
> Research, a program of the University Corporation for Atmospheric Research (UCAR) pursuant to a Cooperative
> Agreement with the National Science Foundation; ©2007 University Corporation for Atmospheric Research. All
> Rights Reserved.

and, on scholarly works associated with it: "The Noah-MP modeling system was developed at the National Center for
Atmospheric Research (NCAR) with collaborations from university partners. NCAR is sponsored by the United States
National Science Foundation."

**These parts are not under the GPL.** The terms above do not allow the AER and UCAR code, or works containing it,
to be sold (AER: "as long as it is not sold"; UCAR: "You shall not sell, license or transfer for a fee the Software,
or any work that in any manner contains the Software"), which the GPL does not allow a licence to require. The files
derived from them (`src/phys/ra_rrtmg_lw/*`, `src/phys/ra_rrtmg_sw/*`, `src/phys/ra_rrtmg_common/kissvec.hpp`,
`src/phys/ra_rrtmg_common/parkind.hpp`, `data/rrtmg/*.bin`, `src/phys/ra_rrtm_lw/*`, `src/phys/lsm_noahmp/*`)
therefore remain under AER's and UCAR's terms, and so does any copy of CPPWRF that contains them: CPPWRF, as
distributed here, may be used, modified and redistributed free of charge, but not sold. The rest of CPPWRF's own
code is under the GPL as stated above.

**Additional permission (GNU GPL version 3, section 7).** As the copyright holder of CPPWRF's own code, Ömer Kaan
Yılmaz grants this additional permission: you may convey CPPWRF's own code combined with the parts listed above that
are derived from AER's RRTMG / RRTM and UCAR's Noah-MP, with those parts under AER's and UCAR's terms instead of the
GPL. The permission covers only those parts; everything else stays under the GPL, and a copy that contains those
parts may not be sold, as their terms require. You may remove this permission from your copies of CPPWRF's own code
(section 7 of the GPL).

## wrf-python

`python/cppwrf/post/wrfpy.py` and `python/cppwrf/post/frame.py` (cppwrf-post's diagnostics;
`SPDX-License-Identifier: GPL-3.0-or-later AND Apache-2.0`) are a numpy port of the Fortran routines and getters
of wrf-python 1.3.4.1 (https://github.com/NCAR/wrf-python), Copyright 2019 University Corporation for Atmospheric
Research, licensed under the Apache License 2.0 (`licenses/WRF-PYTHON-LICENSE.txt`). Many of those routines come
from RIP (Mark Stoelinga) and NCL. `python/cppwrf/post/data/psadilookup.dat` is wrf-python's pseudoadiabat table,
unchanged (`Apache-2.0`). The Apache License 2.0 is compatible with the GPL version 3 under which CPPWRF
distributes the port.

## GNU C Library

The following files port algorithms and tables of the GNU C Library 2.41 (`libm` and `libmvec`, Debian 13 x86-64),
so that the model computes the same values as WRF's gfortran build on the CPU and on the GPU:

- `src/core/glibc_math.hpp`, `src/core/glibc_math_tables.hpp`: double-precision `libm` and 2-lane `libmvec`
  algorithms and tables (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later`).
- `src/core/glibc_mathf.hpp`: single-precision FMA `libm` functions (`expf`, `logf`, `powf`, `sinf`, `cosf`) and
  their tables from glibc 2.41, together with CORE-MATH's single-precision functions below
  (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later AND MIT`).
- `src/core/libmvec_x86.hpp`, `src/core/libmvec_tables.hpp`: 2-lane double `libmvec` `log` and `log10` SSE4
  algorithms and tables (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later`).
- `src/core/libmvec_f4.hpp`, `src/core/libmvec_f4_tables.hpp`: 4-lane float `libmvec` `expf`, `logf`, `powf`,
  `sinf`, `cosf` and `atanf` SSE4 algorithms and tables (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later`).
- `src/core/libmvec_sse4_trig.hpp` (generated by `tools/libmvec_sse4_trig.py`): 2-lane double `libmvec` `acos`,
  `asin`, `tan` and 4-lane float `acosf`, `asinf`, `tanf` SSE4 routines, transliterated instruction by
  instruction, and their tables (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later`).

(`src/core/glibc_mathf_host.cpp` and `tests/phys/libmvec_intel.cpp` are CPPWRF's own `extern "C"` wrappers
forwarding to `core/glibc_mathf.hpp`, `core/libmvec_x86.hpp`, `core/libmvec_f4.hpp` and
`core/libmvec_sse4_trig.hpp`.)

The GNU C Library is Copyright (C) the Free Software Foundation, Inc. and its contributors, licensed under the GNU
Lesser General Public License, version 2.1 or later (`LGPL-2.1-or-later`), whose section 3 allows these parts to
be distributed under the GNU General Public License, as CPPWRF is.

## CORE-MATH

`src/core/glibc_mathf.hpp` (`SPDX-License-Identifier: GPL-3.0-or-later AND LGPL-2.1-or-later AND MIT`) also ports
the correctly rounded single-precision functions `atanf`, `atan2f`, `tanf`, `tanhf`, `log10f`, `asinf`, `acosf`,
`erff`, `tgammaf` and `sinhf` (and their coefficient tables) from Debian 13's GNU C Library 2.41 sources, which
incorporate them from the CORE-MATH project (https://core-math.gitlabpages.inria.fr/, INRIA / Université de
Lorraine / CNRS) under the MIT License (`MIT`). The MIT License is compatible with the GNU General Public License
version 3.

## Libraries used at build or run time

NetCDF-C (Unidata, BSD-3-Clause licence), HDF5 (The HDF Group, BSD-style licence), an OpenMP runtime and, for GPU
builds, the NVIDIA CUDA Toolkit (NVIDIA's licence). They are not part of this repository.
