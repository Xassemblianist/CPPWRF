# Agent 4 Report: Real-Data Initialisation (T4.1)

## Status: Paused for System Package Installation

In accordance with Hard Rule 2:
> "Never use git write commands in the CPPWRF repo. Never use sudo. If a system package is missing (e.g. libjasper/libpng for ungrib GRIB2, libnetcdf-dev), STOP and list the apt command for the user."

During the initial environment audit for Part A, `libpng-dev` (required for WPS ungrib GRIB2 support) and `tcsh` were found to be missing from the system.

### Environment Audit Summary
- **OS**: Debian GNU/Linux 13 (trixie) x86_64
- **Compilers**: GCC 14.2.0 (`gcc`, `g++`, `gfortran`)
- **NetCDF C**: 4.9.3 (`/usr/include`, `-lnetcdf`)
- **NetCDF Fortran**: 4.5.4 (`/usr/include`, `-lnetcdff`)
- **Compression / Libraries**:
  - `zlib1g-dev`: Installed (1.3.1)
  - `libjpeg-dev`: Installed (2.1.5)
  - `libpng-dev`: **Missing** (needed for ungrib GRIB2)
  - `tcsh`: **Missing** (recommended for WRF/WPS scripts)

### Action Required from User
Please run the following command with sudo:
```bash
sudo apt update && sudo apt install -y libpng-dev tcsh
```

Once installed, reply to continue and Agent 4 will proceed directly with Part A (WRF v4.6.1 + WPS v4.6.0 build, geographic data download, GFS cycle download, and reference run).
