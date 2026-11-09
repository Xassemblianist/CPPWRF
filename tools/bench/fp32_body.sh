#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The chroot side of tools/bench/colab_fp32.sh in the Debian 13 chroot of tools/colab/remote_chroot_job.sh (gfortran
# 14.2, MPICH, the GCP VMs' NetCDF):
#
#   fp32_body.sh J CASES STAMP
#
# Builds WRF v4.8.0 from the official release source with WRF's configure for GNU (dmpar, option 34): real*4 (the
# usual production build) and real*8 (configure -r8), in /opt/wrfbench as tools/bench/bench_body.sh; makes the large
# domain L when CASES has it (WPS 4.6.0 from J/wps.tar.gz, GFS of 2026-10-12, real.exe of the real*8 build:
# tools/bench/bench_run.py make-large); prepares the cases (tools/bench/fp32_run.py prepare: run directories, CPPWRF's
# configurations, the GPU queue), writes STAMP to /opt/fp1/prepared for tools/bench/fp32_gpu.sh (CUDA builds, GPU
# runs and the statistics, outside the chroot), runs the WRF runs (fp32_run.py wrf), waits for the GPU side
# (/opt/fp1/host_done, at most 2 h) and packs /opt/fp1/out as J/fetch/fp1.tar.gz. Progress: /opt/fp1/out/fp1.log.
J=$1 CASES=$2 STAMP=$3
W=/opt/wrfbench F=/opt/fp1
rm -rf $F && mkdir -p $W $F/out $J/fetch
L() { echo "$(date +%H:%M:%S) $*" >> $F/out/fp1.log; }
touch $J/setup.out
for n in wrf.exe real.exe mpiexec mpiexec.hydra hydra_pmi_proxy; do pkill -9 -x $n; done
set -x
L "start: $(nproc) vCPUs, cases $CASES"
export DEBIAN_FRONTEND=noninteractive
apt-get -qq update > $J/apt.log 2>&1 && apt-get -qq install -y mpich libmpich-dev tcsh m4 perl make curl ca-certificates \
  git file libpng16-16t64 > $J/apt.log 2>&1 || { echo "FP1_SETUP_FAILED apt (apt.log)" >> $J/setup.out; L "apt failed"; }
lscpu > $F/out/lscpu.txt
{ gfortran --version | head -1; gcc --version | head -1; mpichversion 2> /dev/null | head -2; } > $F/out/compilers.txt 2>&1
mountpoint -q /dev/shm || mount -t tmpfs -o size=64g tmpfs /dev/shm || echo "FP1_SETUP_FAILED /dev/shm" >> $J/setup.out
printf '#include <mpi.h>\n#include <stdio.h>\nint main(int c, char **v) { int r; MPI_Init(&c, &v); MPI_Comm_rank(MPI_COMM_WORLD, &r);\nif (!r) printf("MPI OK\\n"); MPI_Finalize(); return 0; }\n' > $W/hello.c
mpicc -o $W/hello $W/hello.c > $J/mpi.log 2>&1
if ! timeout 120 mpiexec -n 12 $W/hello >> $J/mpi.log 2>&1; then
  export FI_PROVIDER=tcp
  timeout 120 mpiexec -n 12 $W/hello >> $J/mpi.log 2>&1 || echo "FP1_SETUP_FAILED mpi (mpi.log)" >> $J/setup.out
fi
L "MPI hello: $(grep -q 'MPI OK' $J/mpi.log && echo ok || echo FAILED)${FI_PROVIDER:+ with FI_PROVIDER=$FI_PROVIDER}"
mkdir -p /opt/netcdf
ln -sfn /usr/bin /opt/netcdf/bin; ln -sfn /usr/include /opt/netcdf/include; ln -sfn /usr/lib/x86_64-linux-gnu /opt/netcdf/lib

wrf_src() {
  [ -s $W/wrf-src.tar.gz ] && return 0
  local u
  for u in https://github.com/wrf-model/WRF/releases/download/v4.8.0/v4.8.0.tar.gz \
           https://github.com/wrf-model/WRF/releases/download/v4.8.0/WRFV4.8.0.tar.gz; do
    if curl -sfL --retry 3 -o $W/src.part "$u" && tar tzf $W/src.part | grep -q 'phys/noahmp/src/'; then
      mv $W/src.part $W/wrf-src.tar.gz; echo "WRF source $u md5 $(md5sum < $W/wrf-src.tar.gz | cut -c1-32)" >> $F/out/wrf_config.txt
      return 0
    fi
  done
  rm -rf $W/src.part $W/WRF
  git clone -q --depth 1 --branch v4.8.0 --recurse-submodules --shallow-submodules https://github.com/wrf-model/WRF $W/WRF &&
    tar czf $W/wrf-src.tar.gz -C $W WRF && rm -rf $W/WRF && echo "WRF source git clone of tag v4.8.0" >> $F/out/wrf_config.txt
}
build_wrf() {  # NAME CONFIGURE_OPTION [-r8]
  local d=$W/WRF-$1
  [ -x $d/main/wrf.exe ] && [ -x $d/main/real.exe ] && return 0
  rm -rf $d && mkdir -p $d && tar xzf $W/wrf-src.tar.gz -C $d --strip-components=1 || return 1
  cd $d || return 1
  export NETCDF=/opt/netcdf WRFIO_NCD_LARGE_FILE_SUPPORT=1
  printf '%s\n1\n' "$2" | ./configure $3 > configure.log 2>&1
  [ -f configure.wrf ] || return 1
  ./compile -j 16 em_real > compile.log 2>&1
  [ -x main/wrf.exe ] && [ -x main/real.exe ]
}
wps() {  # met_em files of the large domain in $W/wps (as bench_body.sh)
  [ -f $W/wps/met_ok ] && return 0
  rm -rf $W/wps && mkdir -p $W/wps && cd $W/wps && tar xzf $J/wps.tar.gz || return 1
  if [ ! -d $W/geog/WPS_GEOG_LOW_RES ]; then
    mkdir -p $W/geog && curl -sfL --retry 3 https://www2.mmm.ucar.edu/wrf/src/wps_files/geog_low_res_mandatory.tar.gz |
      tar xzf - -C $W/geog || return 1
  fi
  local h L=ABCDEFGHIJ n=0
  for h in 000 006; do
    curl -sf --retry 5 -o gfs.f$h "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20261012/00/atmos/gfs.t00z.pgrb2.0p25.f$h" || return 1
    ln -sf gfs.f$h GRIBFILE.AA${L:$n:1}; n=$((n + 1))
  done
  ./geogrid.exe > geogrid.log 2>&1; grep -q "Successful completion of geogrid" geogrid.log || return 1
  ./ungrib.exe > ungrib.log 2>&1; grep -q "Successful completion of ungrib" ungrib.log || return 1
  ./metgrid.exe > metgrid.log 2>&1; grep -q "Successful completion of metgrid" metgrid.log || return 1
  rm -f gfs.f* FILE:*
  touch met_ok
}

t0=$SECONDS
( wrf_src || exit 1
  build_wrf r4 34 "" > /dev/null 2>&1 & build_wrf r8 34 -r8 > /dev/null 2>&1 &
  wait ) > $J/wrf_build.log 2>&1 &
WRFB=$!
case ",$CASES," in *,L,*|,all,) ( wps ) > $J/wps.log 2>&1 ;; esac
wait $WRFB
for n in r4 r8; do
  d=$W/WRF-$n
  if [ -x $d/main/wrf.exe ]; then L "WRF-$n built"; else
    L "WRF-$n FAILED"; echo "FP1_BUILD_FAILED WRF-$n" >> $J/setup.out
    mkdir -p $F/out/logs; tail -30 $d/configure.log > $F/out/logs/WRF-$n.configure.log 2> /dev/null
    grep -B2 -A8 -m5 -E 'Error|error:' $d/compile.log > $F/out/logs/WRF-$n.compile_errors.log 2> /dev/null
  fi
  { echo "== WRF-$n"; grep -E '^(DMPARALLEL|SFC|SCC|DM_FC|DM_CC|FCOPTIM|FCBASEOPTS_NO_G|PROMOTION|ARCH_LOCAL)\s' $d/configure.wrf 2> /dev/null; } >> $F/out/wrf_config.txt
done
L "builds done ($((SECONDS - t0)) s)"
case ",$CASES," in *,L,*|,all,)
  if [ -f $W/wps/met_ok ] && [ -x $W/WRF-r8/main/real.exe ]; then
    [ -s $W/run_L/wrfbdy_d01 ] || python3 $J/src/tools/bench/bench_run.py make-large $J/data/wrf_runs/run_P4_r8 $W/wps >> $J/run.log 2>&1
    L "L: $( [ -s $W/run_L/wrfbdy_d01 ] && echo 'inputs made' || echo 'no inputs (WPS or real.exe failed)')"
  else L "L: WPS failed"; fi ;;
esac
cd $J/src || exit 1
if [ "$CASES" = all ]; then python3 tools/bench/fp32_run.py prepare >> $J/run.log 2>&1
else python3 tools/bench/fp32_run.py prepare "$CASES" >> $J/run.log 2>&1; fi
echo "$STAMP" > $F/prepared
L "prepared: $(grep -c . $F/gpu_queue.txt) GPU runs, $(grep -c '"case"' $F/wrf_plan.json) WRF runs"
python3 tools/bench/fp32_run.py wrf >> $J/run.log 2>&1
touch $F/wrf_done
L "WRF runs done"
for i in $(seq 720); do [ -e $F/host_done ] && break; sleep 10; done
[ -e $F/host_done ] || { L "the GPU side did not finish"; echo "FP1_GPU_SIDE_UNFINISHED" >> $J/setup.out; }
cp $J/run.log $J/wrf_build.log $J/wps.log $F/out/ 2> /dev/null
tar czf $J/fetch/fp1.tar.gz -C $F out --transform 's,^out,fp1,' --exclude='*.nc'
L "end"
exit 0
