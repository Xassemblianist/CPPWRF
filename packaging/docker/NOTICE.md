# CPPWRF container notices

CPPWRF is distributed under GPL-3.0-or-later with the additional permission and third-party terms described in
`/opt/cppwrf/share/doc/cppwrf/LICENSING.md` and `NOTICE.md`. Those terms include the AER RRTMG and UCAR Noah-MP
restrictions on sale. The directory contains the licence texts. The OCI source and revision labels identify
the corresponding CPPWRF source and build recipes.

NCAR/UCAR developed WRF and the WRF Preprocessing System. UCAR describes the WRF modeling system as public
domain. The WRF 4.8.0 run tables and the WRF I/O code linked into WPS come from the official WRF release.
This directory includes WRF's public domain notice, the notice for the separately licensed Noah-MP tables,
the WPS README, and the notices for the GRIB2 libraries linked into WPS. See the [NCAR WRF User's Guide](https://www2.mmm.ucar.edu/wrf/users/docs/user_guide_v4/v4.4/users_guide_chap1.html).

- WRF 4.8.0 tables and I/O source. [WRF release](https://github.com/wrf-model/WRF/releases/tag/v4.8.0)
- WPS 4.6.0 and its bundled GRIB2 libraries, with the upstream metgrid and ungrib fixes.
  [WPS release](https://github.com/wrf-model/WPS/releases/tag/v4.6.0), [metgrid fix](https://github.com/wrf-model/WPS/pull/291),
  [ungrib fix](https://github.com/wrf-model/WPS/pull/292)
- Geography and GFS data downloaded at runtime. [UCAR WPS static data](https://www2.mmm.ucar.edu/wrf/users/download/get_sources_wps_geog.html), [NOAA GFS](https://registry.opendata.aws/noaa-gfs-bdp-pds/)
- The operating-system packages (Debian 13 in the CPU image, Ubuntu 24.04 in the CUDA image) keep their copyright
  files in `/usr/share/doc`. The CUDA image is built on NVIDIA's `nvidia/cuda` image, and its CUDA components,
  including the CUDA runtime library that the GPU builds load, are under NVIDIA's licence terms.
