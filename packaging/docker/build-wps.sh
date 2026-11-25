#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

# Optional local source trees must be pristine and writable.
prefix=$(realpath -m "${1:?usage: build-wps.sh PREFIX [WRF_SOURCE WPS_SOURCE]}")
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
if [[ $# == 3 ]]; then
  WRF_DIR=$(realpath "$2")
  wps=$(realpath "$3")
else
  WRF_DIR=$work/wrf
  wps=$work/wps
  mkdir -p "$WRF_DIR" "$wps"
  curl -fsSL --retry 5 -o "$work/wrf.tar.gz" \
    https://github.com/wrf-model/WRF/releases/download/v4.8.0/v4.8.0.tar.gz
  printf '%s  %s\n' f3b997266379e1719d186c92471a8f44f8185e976244abde82145a7fddeaf418 "$work/wrf.tar.gz" | sha256sum -c -
  tar -xzf "$work/wrf.tar.gz" --strip-components=1 -C "$WRF_DIR"
  curl -fsSL --retry 5 -o "$work/wps.tar.gz" \
    https://codeload.github.com/wrf-model/WPS/tar.gz/335c76a111f84503e8b963abaf273ea8053645bb
  tar -xzf "$work/wps.tar.gz" --strip-components=1 -C "$wps"
fi

# WPS geogrid and metgrid link WRF's I/O libraries and four support objects.
# Build only those dependencies, with the default Fortran real kind.
export WRF_DIR
export NETCDF=$work/netcdf
mkdir "$NETCDF"
ln -s /usr/include "$NETCDF/include"
ln -s "/usr/lib/$(gcc -print-multiarch)" "$NETCDF/lib"
io=$WRF_DIR/external
fc='gfortran -O2 -ffree-form -fallow-argument-mismatch'
cpp='cpp -P -nostdinc -traditional-cpp'
make -C "$io/io_netcdf" libwrfio_nf.a "FC=$fc" "CPP=$cpp" RANLIB=ranlib \
  NETCDFPATH="$NETCDF" TRADFLAG='-DUSE_NETCDF4_FEATURES -DWRFIO_NCD_LARGE_FILE_SUPPORT'
make -C "$io/io_int" libwrfio_int.a "FC=$fc" "CPP=$cpp" RANLIB=ranlib FGREP='grep -F'
for lib in io_grib_share io_grib1; do
  make -C "$io/$lib" archive "FC=$fc" "CPP=$cpp" CC=gcc RANLIB=ranlib
done
cp "$io/io_int/module_internal_header_util.o" "$WRF_DIR/frame/"
cp "$io"/ioapi_share/{wrf_io_flags.h,wrf_status_codes.h} "$WRF_DIR/inc/"
(
  cd "$WRF_DIR/frame"
  # For ARW, WRF's Registry generates this data-order constant.
  printf '%s\n' 'integer, parameter :: model_data_order = DATA_ORDER_XZY' > "$work/model_data_order.inc"
  for name in module_driver_constants module_machine; do
    cpp -P -nostdinc -traditional-cpp -DMAX_DOMAINS_F=21 -DIWORDSIZE=4 -DRWORDSIZE=4 -DDWORDSIZE=8 -DLWORDSIZE=4 \
      -I"$work" "$name.F" > "$work/$name.f90"
    gfortran -O2 -c "$work/$name.f90"
  done
  gcc -O2 -DIWORDSIZE=4 -c pack_utils.c
)

cd "$wps"
# Backport WPS #291 (empty met_em files with GCC 14) and #292 (GRIB2 soil
# levels can be lost to rounding), both from the WPS maintainers.
patch --batch --fuzz=0 -p1 <<'PATCH'
--- a/geogrid/src/output_module.F
+++ b/geogrid/src/output_module.F
@@ -471,2 +471,3 @@
       optstatus = 0
+      is_subgrid_var = .false.
       do while (ifieldstatus == 0)
--- a/ungrib/src/rd_grib2.F
+++ b/ungrib/src/rd_grib2.F
@@ -738,4 +738,4 @@
      &               (gfld%ipdtmpl(2) .eq. g2code(3,j)) .and.
-     &               (glevel1 .eq. level1(j)) .and.
-     &               ((glevel2 .eq. level2(j)) .or.
+     &               (nint(glevel1) .eq. level1(j)) .and.
+     &               ((nint(glevel2) .eq. level2(j)) .or.
      &                                   (level2(j) .le. -88))) then
PATCH
# Option 1 in WPS 4.6.0 on Linux x86-64 is GNU serial with GRIB2.
printf '1\n' | ./configure --build-grib2-libs
grep -Eq 'Settings for Linux x86_64, gfortran +\(serial\)' configure.wps
./compile wps > "$work/wps-build.log" 2>&1 || { cat "$work/wps-build.log"; exit 1; }

# compile uses make -i, so verify that it actually linked all three programs.
for tool in geogrid ungrib metgrid; do
  if [[ ! -x $tool.exe ]]; then
    cat "$work/wps-build.log"
    printf 'WPS build did not produce %s.exe\n' "$tool" >&2
    exit 1
  fi
  install -D -m 755 "$tool.exe" "$prefix/wps/$tool.exe"
done
install -D -m 644 geogrid/GEOGRID.TBL.ARW "$prefix/wps/geogrid/GEOGRID.TBL"
install -D -m 644 metgrid/METGRID.TBL.ARW "$prefix/wps/metgrid/METGRID.TBL"
install -D -m 644 ungrib/Variable_Tables/Vtable.GFS "$prefix/wps/ungrib/Variable_Tables/Vtable.GFS"

mkdir -p "$prefix/wrf/run" "$prefix/notices"
cp -aL "$WRF_DIR/run/." "$prefix/wrf/run/"
install -m 644 "$WRF_DIR"/phys/noahmp/parameters/{NoahmpTable.TBL,snicar_*.nc} "$prefix/wrf/run/"
install -m 644 "$WRF_DIR/LICENSE.txt" "$prefix/notices/WRF-LICENSE.txt"
install -m 644 "$WRF_DIR/phys/noahmp/LICENSE.txt" "$prefix/notices/Noah-MP-LICENSE.txt"
install -m 644 README "$prefix/notices/WPS-README"
install -m 644 external/jasper-1.900.29/COPYRIGHT "$prefix/notices/JasPer-COPYRIGHT"
install -m 644 external/jasper-1.900.29/LICENSE "$prefix/notices/JasPer-LICENSE"
install -m 644 external/libpng-1.6.37/LICENSE "$prefix/notices/libpng-LICENSE"
install -m 644 external/zlib-1.2.11/README "$prefix/notices/zlib-README"
