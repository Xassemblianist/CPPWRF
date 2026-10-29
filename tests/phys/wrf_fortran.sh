# Sourced by the differential tests (tests/phys, tests/dyn, tests/nest). The Fortran side of every test is WRF
# v4.8.0's own compiled code, linked from the WRF build tree that made the model's references:
#   WRF_DIR       configure -r8 (-fdefault-real-8): wrf_r8.exe (Kessler, WSM6, KF, sfclayrev, YSU, MYJ, Noah,
#                 Noah-MP, the dycore and the nests)
#   WRF_R8D8_DIR  -r8 with -fdefault-double-8: wrf_r8d8.exe (Thompson, MYNN, Grell-Freitas-Li, New Tiedtke; their
#                 DOUBLE PRECISION locals are REAL*16 in a plain -r8 build)
# No WRF source is compiled by a test: the drivers call the build's objects (main/libwrflib.a).
#
#   wrf_need r8|r8d8         sets W to the build directory; exits 77 (ctest: skipped) when it is not set
#   wrf_fortran OUT SRC...   compiles the test's driver SRC with W's PROMOTION (configure.wrf) and W's modules and
#                            links it to W's libwrflib.a and WRF's external libraries

wrf_need() {
  case $1 in
    r8) W=${WRF_DIR:-} ;;
    r8d8) W=${WRF_R8D8_DIR:-} ;;
    *) echo "wrf_need: r8 or r8d8" >&2; exit 2 ;;
  esac
  if [ -z "$W" ]; then
    echo "SKIP: set $([ "$1" = r8 ] && echo WRF_DIR || echo WRF_R8D8_DIR) to a WRF v4.8.0 -$1 build"
    exit 77
  fi
  [ -f "$W/main/libwrflib.a" ] && [ -f "$W/configure.wrf" ] || { echo "no WRF build in $W"; exit 1; }
  W=$(cd "$W" && pwd)
}

wrf_fortran() {
  local out=$1; shift
  local promo nclib
  promo=$(sed -n 's/^PROMOTION *= *//p' "$W/configure.wrf")
  case $promo in -fdefault-real-8*) ;; *) echo "unexpected PROMOTION '$promo' in $W/configure.wrf"; exit 1 ;; esac
  nclib=$(nf-config --flibs 2>/dev/null || echo "-L${NETCDF:-/usr}/lib -lnetcdff -lnetcdf")
  # shellcheck disable=SC2086
  gfortran -O2 -ffree-form -ffree-line-length-none -fallow-argument-mismatch $promo \
    -I"$W/main" -I"$W/frame" -I"$W/share" -I"$W/phys" -I"$W/dyn_em" -I"$W/inc" -I"$W/external/esmf_time_f90" \
    -J"$(dirname "$out")" "$@" -o "$out" \
    -Wl,--start-group "$W/main/libwrflib.a" "$W/external/fftpack/fftpack5/libfftpack.a" \
    "$W/external/io_grib1/libio_grib1.a" "$W/external/io_grib_share/libio_grib_share.a" \
    "$W/external/io_int/libwrfio_int.a" "$W/external/esmf_time_f90/libesmf_time.a" \
    "$W/external/RSL_LITE/librsl_lite.a" "$W/frame/module_internal_header_util.o" "$W/frame/pack_utils.o" \
    "$W/external/io_netcdf/libwrfio_nf.a" -Wl,--end-group $nclib -lmvec -lm
}
