# RRTMG tables (WRF v4.8.0)

`rrtmg_lw_tables.bin` / `rrtmg_sw_tables.bin` are the RRTMG longwave / shortwave tables exactly as
WRF v4.8.0 (real*8 build) holds them after `rrtmg_lwinit` / `rrtmg_swinit` read
`RRTMG_LW_DATA_DBL` / `RRTMG_SW_DATA_DBL` and reduce them to 140 / 112 g-points (cmbgb*), plus the
cloud property data and the Pade tables: raw memory images of `RrtmgLwTablesOf<double>` /
`RrtmgSwTablesOf<double>` (src/phys/ra_rrtmg_*/rrtmg_*_tables.hpp; read by ra_rrtmg_common/table_dump.hpp),
dumped by an instrumented WRF build (tests/phys/ra_rrtmg_sw/dump_sw_tables.f90 for the SW one).
With them the C++ RRTMG is bit-identical to WRF (docs/PHYSICS_SUITE.md sections 8-10).

`rrtmg_lw_tables_r4.bin` / `rrtmg_sw_tables_r4.bin` are the same tables as WRF v4.8.0's default real*4 build
(`configure` without `-r8`: rb = REAL) holds them: `RRTMG_LW_DATA` / `RRTMG_SW_DATA` read and reduced, and the Pade
tables computed, in single precision. The FP32 builds read them (memory images of
`RrtmgLwTablesOf<float>` / `RrtmgSwTablesOf<float>`: 718924 / 421500 bytes). They were written by the same two dump programs, compiled
like WRF (`-fconvert=big-endian -frecord-marker=4`, no promotion) and linked to the real*4 build's `libwrflib.a`.

The radiation driver looks for them in `phys.rrtmg_tables` (default: the WRF run directory of the
case) and then here (`CPPWRF_DATA_DIR`, set at build time). The ozone climatology
(`ozone*.formatted`) and `CAMtr_volume_mixing_ratio` are read from `phys.rrtmg_tables` as WRF reads
them from its run directory.
