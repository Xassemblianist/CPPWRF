#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The chroot side of tools/bench/colab_paper.sh (the paper benchmark, tools/bench/PROTOCOL.md) in the Debian 13
# chroot that tools/bench/paper_job.sh makes (gcc / gfortran 14.2, glibc 2.41, MPICH, Debian's NetCDF):
#
#   paper_body.sh J SET CASES STAMP [PAPER_RUN_OPTION ...]
#
# SET full (the G4 session): builds, in /opt/wrfbench,
# - WRF v4.8.0 from the official release source with WRF's configure for GNU and MPICH, real*4 and real*8
#   (configure -r8), dmpar (option 34) and dm+sm (35): WRF-r4, WRF-r8, WRF-r4sm, WRF-r8sm (compile -j 12, then -j 4
#   and -j 1 when WRF's parallel make loses a module);
# - WRF-r4t and WRF-r4tsm (unless --models leaves out wrf_r4t): real*4 with configure.wrf's FCOPTIM set to
#   -O3 -march=native -funroll-loops (WRF tuned for this CPU; FMA contraction allowed, as gfortran's default);
# - CPPWRF's CPU builds: cpu-dp (FP64, the release build: -march=native, CPPWRF_PORTABLE_LIBMVEC on), cpu-dp-hwlog
#   (CPPWRF_PORTABLE_LIBMVEC off: this CPU's libmvec log, as the WRF builds here) and cpu-sp (FP32);
# - when CASES has C2: the daytime large domain run_Lday (tools/bench/namelist.wps with the dates 2026-10-12 06 - 12
#   UTC: WPS 4.6.0 from J/wps.tar.gz on WPS's low-resolution geography and the GFS 0.25 degree 6 h and 12 h forecasts
#   of 2026-10-12 00 UTC, NOAA's open data; real.exe of WRF-r8: tools/bench/paper_run.py make-large), unless the data
#   tars hold one (wrf_runs/run_Lday);
# SET gpu (an L4 / A100 session: GPU numbers only): cpu-dp and cpu-sp (the proof runs), and WRF-r8 and WPS only to
# make run_Lday when no data tar holds it.
# Then: the build manifest (tools/bench/manifest.py, merged with the GPU side's), and tools/bench/paper_run.py
# prepare, screen (full only), timed, proof, verify, report with /opt/paper as its root, the GPU runs through the
# agent of tools/bench/paper_gpu.sh (STAMP in /opt/paper/body_started tells it that this job has started; it answers
# with /opt/paper/gpu_ready when its builds are done). Nothing else runs during the screening and the timed blocks:
# the builds and downloads end before them, and the screening waits for the GPU side's builds.
# Results: J/fetch/paper.tar.gz (/opt/paper/out without the wrfout files); progress in /opt/paper/out/paper.log.
# PAPER_SAVE_LDAY=1: also J/drive/paper_lday_data.tar (run_Lday's inputs, for a later SET gpu session's data).
case "${1:-}" in /*) ;; *) sed -n '3,28p' "$0"; exit 2 ;; esac  # --help, or no job directory
J=$1 SET=$2 CASES=$3 STAMP=$4; shift 4
W=/opt/wrfbench P=/opt/paper
rm -rf $P && mkdir -p $W $P/out $P/q $J/fetch
touch $J/setup.out
B() { echo "$(date +%H:%M:%S) $*" >> $P/out/paper.log; }
SHA() { echo "$(sha256sum < "$1" | cut -c1-64)  $2" >> $P/out/inputs_sha256.txt; }  # FILE LABEL: an input's digest
echo "$STAMP" > $P/body_started
for n in wrf.exe real.exe cppwrf mpiexec mpiexec.hydra hydra_pmi_proxy; do pkill -9 -x $n; done  # an earlier run's (--keep)
set -x
NP=$(nproc)
B "start: $NP vCPUs, set $SET, cases $CASES, options $*"
export DEBIAN_FRONTEND=noninteractive
apt-get -qq update > $J/apt.log 2>&1 && apt-get -qq install -y mpich libmpich-dev tcsh m4 perl make curl ca-certificates \
  git file libpng16-16t64 > $J/apt.log 2>&1 || { echo "PAPER_SETUP_FAILED apt (apt.log)" >> $J/setup.out; B "apt failed"; }
lscpu > $P/out/lscpu.txt; lscpu -e > $P/out/lscpu_e.txt 2>&1
mountpoint -q /dev/shm || mount -t tmpfs -o size=64g tmpfs /dev/shm || echo "PAPER_SETUP_FAILED /dev/shm" >> $J/setup.out
printf '#include <mpi.h>\n#include <stdio.h>\nint main(int c, char **v) { int r; MPI_Init(&c, &v); MPI_Comm_rank(MPI_COMM_WORLD, &r);\nif (!r) printf("MPI OK\\n"); MPI_Finalize(); return 0; }\n' > $W/hello.c
mpicc -o $W/hello $W/hello.c > $J/mpi.log 2>&1
if ! timeout 120 mpiexec -n $NP $W/hello >> $J/mpi.log 2>&1; then
  export FI_PROVIDER=tcp
  timeout 120 mpiexec -n $NP $W/hello >> $J/mpi.log 2>&1 || echo "PAPER_SETUP_FAILED mpi (mpi.log)" >> $J/setup.out
fi
B "MPI hello: $(grep -q 'MPI OK' $J/mpi.log && echo ok || echo FAILED)${FI_PROVIDER:+ with FI_PROVIDER=$FI_PROVIDER}"
mkdir -p /opt/netcdf
ln -sfn /usr/bin /opt/netcdf/bin; ln -sfn /usr/include /opt/netcdf/include; ln -sfn /usr/lib/x86_64-linux-gnu /opt/netcdf/lib

wrf_src() {  # the release tarball (with its submodules), else a git clone of the tag (as tools/bench/bench_body.sh)
  [ -s $W/wrf-src.tar.gz ] && return 0
  local u
  for u in https://github.com/wrf-model/WRF/releases/download/v4.8.0/v4.8.0.tar.gz \
           https://github.com/wrf-model/WRF/releases/download/v4.8.0/WRFV4.8.0.tar.gz; do
    if curl -sfL --retry 3 -o $W/src.part "$u" && tar tzf $W/src.part | grep -q 'phys/noahmp/src/'; then
      mv $W/src.part $W/wrf-src.tar.gz; echo "WRF source $u sha256 $(sha256sum < $W/wrf-src.tar.gz | cut -c1-64)" >> $P/out/wrf_config.txt
      SHA $W/wrf-src.tar.gz "$u"
      return 0
    fi
  done
  rm -rf $W/src.part $W/WRF
  git clone -q --depth 1 --branch v4.8.0 --recurse-submodules --shallow-submodules https://github.com/wrf-model/WRF $W/WRF &&
    tar czf $W/wrf-src.tar.gz -C $W WRF && rm -rf $W/WRF && echo "WRF source git clone of tag v4.8.0" >> $P/out/wrf_config.txt
}
build_wrf() {  # NAME CONFIGURE_OPTION [-r8] [tuned: FCOPTIM -O3 -march=native -funroll-loops]
  local d=$W/WRF-$1
  [ -x $d/main/wrf.exe ] && [ -x $d/main/real.exe ] && return 0
  rm -rf $d && mkdir -p $d && tar xzf $W/wrf-src.tar.gz -C $d --strip-components=1 || return 1
  cd $d || return 1
  export NETCDF=/opt/netcdf WRFIO_NCD_LARGE_FILE_SUPPORT=1
  printf '%s\n1\n' "$2" | ./configure $3 > configure.log 2>&1
  [ -f configure.wrf ] || return 1
  [ "${4:-}" = tuned ] && sed -i 's/^FCOPTIM[[:space:]]*=.*/FCOPTIM         =       -O3 -march=native -funroll-loops/' configure.wrf
  ./compile -j 12 em_real > compile.log 2>&1
  # WRF's parallel make can lose a module dependency (the c9c95c5 session's real*4 dmpar link): fewer jobs
  [ -x main/wrf.exe ] && [ -x main/real.exe ] || ./compile -j 4 em_real > compile2.log 2>&1
  [ -x main/wrf.exe ] && [ -x main/real.exe ] || ./compile -j 1 em_real > compile3.log 2>&1
  [ -x main/wrf.exe ] && [ -x main/real.exe ]
}
wps_day() {  # met_em files of the daytime large domain in $W/wps_day
  [ -f $W/wps_day/met_ok ] && return 0
  rm -rf $W/wps_day && mkdir -p $W/wps_day && cd $W/wps_day && tar xzf $J/wps.tar.gz || return 1
  SHA $J/wps.tar.gz "wps.tar.gz (WPS 4.6.0 executables and tables, uploaded by the launcher)"
  sed -i -e "s/start_date *= *'[^']*'/start_date = '2026-10-12_06:00:00'/" \
         -e "s/end_date *= *'[^']*'/end_date   = '2026-10-12_12:00:00'/" namelist.wps
  if [ ! -d $W/geog/WPS_GEOG_LOW_RES ]; then
    mkdir -p $W/geog && curl -sfL --retry 3 -o $W/geog.tgz \
      https://www2.mmm.ucar.edu/wrf/src/wps_files/geog_low_res_mandatory.tar.gz && tar xzf $W/geog.tgz -C $W/geog || return 1
    SHA $W/geog.tgz "https://www2.mmm.ucar.edu/wrf/src/wps_files/geog_low_res_mandatory.tar.gz"; rm -f $W/geog.tgz
  fi
  local h L=ABCDEFGHIJ n=0
  for h in 006 012; do
    curl -sf --retry 5 -o gfs.f$h "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20261012/00/atmos/gfs.t00z.pgrb2.0p25.f$h" || return 1
    SHA gfs.f$h "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20261012/00/atmos/gfs.t00z.pgrb2.0p25.f$h"
    ln -sf gfs.f$h GRIBFILE.AA${L:$n:1}; n=$((n + 1))
  done
  ./geogrid.exe > geogrid.log 2>&1; grep -q "Successful completion of geogrid" geogrid.log || return 1
  ./ungrib.exe > ungrib.log 2>&1; grep -q "Successful completion of ungrib" ungrib.log || return 1
  ./metgrid.exe > metgrid.log 2>&1; grep -q "Successful completion of metgrid" metgrid.log || return 1
  rm -f gfs.f* FILE:*
  touch met_ok
}
build_cpu() {  # NAME DOUBLE PORTABLE_LIBMVEC
  cmake -S $J/src -B $W/$1 -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=$2 \
    -DCPPWRF_PORTABLE_LIBMVEC=$3 > $J/build-$1.log 2>&1 && cmake --build $W/$1 -j 16 --target cppwrf >> $J/build-$1.log 2>&1
}

LDAY=$J/data/wrf_runs/run_Lday
NEED_L=0; case ",$CASES," in *,C2,*) [ -s $LDAY/wrfbdy_d01 ] || NEED_L=1 ;; esac
WRFS="r4 r8 r4sm r8sm"; [ "$SET" = full ] || WRFS=; [ "$SET" = full ] || [ $NEED_L = 0 ] || WRFS=r8
# the tuned real*4 builds: in the default models of SET full, or when --models names wrf_r4t
case " $* " in *wrf_r4t*) WRFS="$WRFS r4t r4tsm" ;; *--models*) ;; *) [ "$SET" = full ] && WRFS="$WRFS r4t r4tsm" ;; esac
B "WRF builds: ${WRFS:-none}; large domain to make: $NEED_L"
( [ -z "$WRFS" ] && exit 0; wrf_src || exit 1
  for n in $WRFS; do
    case $n in r4) build_wrf r4 34 "" ;; r8) build_wrf r8 34 -r8 ;; r4sm) build_wrf r4sm 35 "" ;; r8sm) build_wrf r8sm 35 -r8 ;;
      r4t) build_wrf r4t 34 "" tuned ;; r4tsm) build_wrf r4tsm 35 "" tuned ;; esac > /dev/null 2>&1 &
  done
  wait ) > $J/wrf_build.log 2>&1 &
WRFB=$!
if [ $NEED_L = 1 ]; then ( wps_day ) > $J/wps.log 2>&1 & else ( : ) & fi
WPSP=$!
t0=$SECONDS
CPUB="cpu-dp:ON:ON cpu-sp:OFF:ON"; [ "$SET" = full ] && CPUB="cpu-dp:ON:ON cpu-dp-hwlog:ON:OFF cpu-sp:OFF:ON"
for b in $CPUB; do
  n=${b%%:*} r=${b#*:}
  build_cpu $n ${r%%:*} ${r#*:} || echo "PAPER_BUILD_FAILED $n (build-$n.log)" >> $J/setup.out
done
B "CPPWRF CPU builds ($CPUB): $((SECONDS - t0)) s"
wait $WPSP
[ $NEED_L = 0 ] || B "WPS (daytime L): $(ls $W/wps_day/met_em.d01.* 2> /dev/null | wc -l) met_em files ($( [ -f $W/wps_day/met_ok ] && echo ok || echo FAILED))"
mkdir -p $P/out/logs
for f in geogrid ungrib metgrid; do tail -3 $W/wps_day/$f.log > $P/out/logs/wps_day_$f.log 2> /dev/null; done
wait $WRFB
for n in $WRFS; do
  d=$W/WRF-$n
  if [ -x $d/main/wrf.exe ]; then B "WRF-$n built"; else
    B "WRF-$n FAILED"; echo "PAPER_BUILD_FAILED WRF-$n" >> $J/setup.out
    tail -30 $d/configure.log > $P/out/logs/WRF-$n.configure.log 2> /dev/null
    grep -B2 -A8 -m5 -E 'Error|error:' $d/compile*.log > $P/out/logs/WRF-$n.compile_errors.log 2> /dev/null
  fi
done
B "builds done ($((SECONDS - t0)) s)"
cd $J/src || exit 1
if [ $NEED_L = 1 ]; then
  if [ -f $W/wps_day/met_ok ] && [ -x $W/WRF-r8/main/real.exe ]; then
    python3 tools/bench/paper_run.py make-large --base $J/data/wrf_runs/run_FP2_P4_24h_r8 --met $W/wps_day --out $W/run_Lday \
      --start 2026-10-12_06 --end 2026-10-12_12 --wrf $W >> $P/out/make_large.log 2>&1
    B "C2: run_Lday $(tail -1 $P/out/make_large.log)"
  else B "C2: no WPS or real.exe: no large domain"; echo "PAPER_L_FAILED" >> $J/setup.out; fi
else
  [ -d $LDAY ] && ln -sfn $LDAY $W/run_Lday
fi
if [ "${PAPER_SAVE_LDAY:-0}" = 1 ] && [ -s $W/run_Lday/wrfbdy_d01 ]; then
  mkdir -p $J/drive $J/stage/wrf_runs/run_Lday
  for f in $W/run_Lday/*; do ln -sfn "$(readlink -f "$f")" $J/stage/wrf_runs/run_Lday/; done
  tar chf $J/drive/paper_lday_data.tar -C $J/stage wrf_runs && B "paper_lday_data.tar: $(du -h $J/drive/paper_lday_data.tar | cut -f1)" &&
    SHA $J/drive/paper_lday_data.tar "paper_lday_data.tar (written to Drive cppwrf-data)"
fi
M=(--cmake cpu-dp=$W/cpu-dp --cmake cpu-sp=$W/cpu-sp)
[ "$SET" = full ] && M+=(--cmake cpu-dp-hwlog=$W/cpu-dp-hwlog)
for n in $WRFS; do M+=(--wrf WRF-$n=$W/WRF-$n); done
python3 tools/bench/manifest.py collect --side cpu --commit $J/commit.txt "${M[@]}" --out $P/manifest_cpu.json >> $P/out/manifest.log 2>&1
OPTS=(--root $P --data $J/data/wrf_runs --wrf $W --src $J/src --cases "$CASES")
[ "$SET" = full ] || OPTS+=(--models gpu_fp32,gpu_fp32id,gpu_fp64 --proof cpu_fp64,cpu_fp32 --proof-threads $((NP / 2)))
OPTS+=("$@")
python3 tools/bench/paper_run.py prepare "${OPTS[@]}" >> $P/out/run.log 2>&1 ||
  echo "PAPER_PHASE_FAILED prepare exit $? (run.log)" >> $J/setup.out
grep -q 'FAILED' $P/out/paper.log && grep 'wrf2cppwrf.py FAILED' $P/out/paper.log | sed 's/^/PAPER_TRANSLATE_FAILED /' >> $J/setup.out
for i in $(seq 720); do [ -e $P/gpu_ready ] && break; sleep 10; done  # the GPU side's builds end before the timings
B "GPU side: $(cat $P/gpu_ready 2> /dev/null || echo 'not ready after 2 h')"
python3 tools/bench/manifest.py merge $P/manifest_cpu.json $P/manifest_gpu.json --out $P/out/manifest.json >> $P/out/manifest.log 2>&1
STEPS="timed proof verify report"; [ "$SET" = full ] && STEPS="screen $STEPS"
for step in $STEPS; do  # a phase that fails is logged and reported (setup.out: the launcher exits non-zero)
  python3 tools/bench/paper_run.py $step "${OPTS[@]}" >> $P/out/run.log 2>&1 ||
    { rc=$?; B "paper_run.py $step: exit $rc"; echo "PAPER_PHASE_FAILED $step exit $rc (run.log)" >> $J/setup.out; }
done
dpkg-query -W -f='${Package} ${Version}\n' > $P/out/packages_chroot.txt 2>&1  # every package of the chroot
touch $P/q/stop
for i in $(seq 24); do [ -e $P/gpu_exit ] && break; sleep 5; done
cp $J/build-*.log $J/wrf_build.log $J/wps.log $J/mpi.log $P/out/logs/ 2> /dev/null
cp $P/gpu_host.log $P/gpu_build-*.log $P/out/logs/ 2> /dev/null
for n in $WRFS; do { echo "== WRF-$n"; grep -E '^(DMPARALLEL|OMPCPP|SFC|SCC|DM_FC|DM_CC|FCOPTIM|FCBASEOPTS_NO_G|PROMOTION|CFLAGS_LOCAL|ARCH_LOCAL)\s' $W/WRF-$n/configure.wrf 2> /dev/null; } >> $P/out/wrf_config.txt; done
tar czf $J/fetch/paper.tar.gz -C $P out --transform 's,^out,paper,' --exclude='wrfout_*'
B "end"
exit 0
