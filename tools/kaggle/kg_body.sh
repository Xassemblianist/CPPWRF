#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The job of tools/kaggle/kaggle_mpi.sh on a Kaggle GPU runtime (2 x Tesla T4, Ubuntu with the CUDA toolkit and
# Open MPI): builds CUDA FP64 with CPPWRF_MPI from IN/src.tgz.dat (git archive) and runs cuda_dp suite cases with
# every layout of LAYOUTS against the WRF references at tolerance 0 (tests/verify/wrf_suite.py --exact); the
# output files of every layout are compared byte for byte with those of the first layout.
#
#   kg_body.sh IN OUT
#
# IN (the job's private dataset) holds src.tgz.dat, job.env (CASES, LAYOUTS, DATA: comma lists; COMMIT; HOURS).
# LAYOUTS entries:
#   plain   the program as one process (no mpirun)
#   1       mpirun -np 1
#   PxQ     mpirun -np P*Q, run.ranks = PxQ (CPPWRF_RANKS); the ranks take the GPUs by their node-local rank
#           (core/tiles.hpp rank_device: local mod device count), so 1x2 / 2x1 put one rank on each GPU
#   tPxQ    one process, PxQ tiles (CPPWRF_TILES) over the GPUs (CPPWRF_TILE_DEVICES=0,1,...)
# DATA: tars of the public release Xassemblianist/cppwrf-data data-v1 (whole or .part* pieces, each checked against
# its SHA256SUMS), unpacked into /root/kg/data (wrf_runs/, wrf_ref/).
# OUT (in /kaggle/working, kept as the kernel's output): env.txt (GPUs, driver, nvcc, Open MPI, gcc, commit), build.log,
# data.txt, mpi.txt (smoke test), suite_<layout>.log (the suite's lines and "exit N (S s ...)"), sums_<layout>.txt
# (sha256 of every output file but logs and files with wall-clock times), same.txt ("SAME layout N files" /
# "DIFF layout: files"), devices_<layout>.txt (the rank -> device lines of the logs and, from nvidia-smi sampled every
# 500 ms during the layout, the processes seen on each GPU and each GPU's peak memory and utilisation),
# diff_<layout>.txt (for every file that differs from the first layout's: the NetCDF attributes, dimensions and
# variables that differ, with the number of points, the largest difference and the first index), summary.txt;
# done.txt at the end.
IN=$1 OUT=$2
W=/root/kg
mkdir -p "$OUT" $W
exec > "$OUT/body.log" 2>&1
exec 9>> "$OUT/trace.log"  # the trace apart from the result files
BASH_XTRACEFD=9
set -x
. "$IN/job.env"
T0=$SECONDS
END=$((SECONDS + ${HOURS:-3} * 3600))
GH=https://github.com/Xassemblianist/cppwrf-data/releases/download/data-v1
# --- the runtime
{
  echo "commit $COMMIT"
  nvidia-smi --query-gpu=index,name,uuid,memory.total,driver_version,pci.bus_id --format=csv,noheader
  nvcc --version | tail -2
  echo "mpirun: $(mpirun --version 2>&1 | head -1)"
  gcc --version | head -1
  cmake --version | head -1
  grep PRETTY_NAME /etc/os-release
  echo "nproc $(nproc), $(free -g | awk '/Mem:/ {print $2}') GB RAM"
} > "$OUT/env.txt" 2>&1
# --- packages (NetCDF for the program's I/O; netCDF4-python for the suite's comparison)
export DEBIAN_FRONTEND=noninteractive
{ apt-get -qq update && apt-get -qq install -y libnetcdf-dev netcdf-bin libopenmpi-dev openmpi-bin; } > $W/apt.log 2>&1 ||
  echo "APT_FAILED (apt.log)" >> "$OUT/summary.txt"
python3 -c 'import netCDF4, numpy' 2> /dev/null || pip install -q netCDF4 > $W/pip.log 2>&1
python3 -c 'import netCDF4; print("netCDF4", netCDF4.__version__, "netcdf-c", netCDF4.__netcdf4libversion__)' >> "$OUT/env.txt" 2>&1
cp $W/apt.log "$OUT/apt.log" 2> /dev/null
# --- the data, in the background while the program builds
fetch_data() {
  local t sum p i ok
  mkdir -p $W/data $W/parts
  curl -sSfL --retry 5 -o $W/SHA256SUMS "$GH/SHA256SUMS" || { echo "DATA_FAILED SHA256SUMS"; return 1; }
  for t in ${DATA//,/ }; do
    local t0=$SECONDS
    awk -v t="$t" '$2 == t { w = $0 } index($2, t ".part") == 1 { p = p $0 "\n" } END { if (w) print w; else printf "%s", p }' \
      $W/SHA256SUMS > $W/parts.lst
    [ -s $W/parts.lst ] || { echo "DATA_FAILED $t (not in the release)"; continue; }
    ok=1
    while read -r sum p; do
      for i in 1 2 3 4 5; do
        curl -sSfL --retry 5 -o "$W/parts/$p" "$GH/$p" && [ "$(sha256sum "$W/parts/$p" | cut -c1-64)" = "$sum" ] && break
        rm -f "$W/parts/$p"; sleep 20
      done
      [ -s "$W/parts/$p" ] || { ok=0; echo "DATA_FAILED $t ($p)"; break; }
    done < $W/parts.lst
    if [ $ok = 1 ]; then
      awk -v d=$W/parts '{print d "/" $2}' $W/parts.lst | xargs cat | tar xf - -C $W/data || echo "DATA_FAILED $t (unpack)"
      echo "TAR $t $(du -cb $W/parts | tail -1 | cut -f1) bytes $((SECONDS - t0)) s"
    fi
    rm -f $W/parts/*
  done
  echo "DATA_TOTAL $((SECONDS - T0)) s"
}
fetch_data > "$OUT/data.txt" 2>&1 &
DPID=$!
# --- the build: CUDA FP64 with MPI, the T4's architecture only
rm -rf $W/src && mkdir -p $W/src && tar xzf "$IN/src.tgz.dat" -C $W/src
t0=$SECONDS
cmake -S $W/src -B $W/b_dp -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON -DCPPWRF_MPI=ON \
  -DCMAKE_CUDA_ARCHITECTURES=75 -DMPI_CXX_COMPILER="$(command -v mpicxx)" > $W/build.log 2>&1 &&
  cmake --build $W/b_dp -j"$(nproc)" --target cppwrf >> $W/build.log 2>&1
echo "build: exit $?, $((SECONDS - t0)) s" >> "$OUT/env.txt"
grep -E "CUDA|MPI|NetCDF|netcdf" $W/build.log | head -30 > "$OUT/build.log"
tail -60 $W/build.log >> "$OUT/build.log"
wait $DPID
B=$W/b_dp/cppwrf
if [ ! -x $B ] || grep -q DATA_FAILED "$OUT/data.txt"; then
  echo "SETUP_FAILED (build.log, data.txt)" >> "$OUT/summary.txt"; touch "$OUT/done.txt"; exit 1
fi
ldd $B | grep -E "mpi|netcdf|cuda" >> "$OUT/env.txt"
# --- Open MPI: root, 4 vCPUs (more ranks than cores: --oversubscribe); the default transport, else TCP
MPIRUN="mpirun --allow-run-as-root --oversubscribe --bind-to none"
cat > $W/hello.c <<'C'
#include <mpi.h>
#include <stdio.h>
int main(int argc, char** argv) {
  int r, n, s = 0;
  MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &r);
  MPI_Comm_size(MPI_COMM_WORLD, &n);
  MPI_Allreduce(&r, &s, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  if (r == 0) printf("ok %d %d\n", n, s);
  MPI_Finalize();
  return 0;
}
C
mpicc -O1 -o $W/hello $W/hello.c >> "$OUT/mpi.txt" 2>&1
{ df -h /dev/shm | tail -1; } >> "$OUT/mpi.txt" 2>&1
MCA=
for E in "" "--mca btl self,tcp"; do
  if $MPIRUN $E -np 4 $W/hello 2>> "$OUT/mpi.txt" | grep -q '^ok 4 6$'; then MCA=$E; echo "transport used: '${E:-default}'" >> "$OUT/mpi.txt"; break; fi
  echo "transport '${E:-default}' failed" >> "$OUT/mpi.txt"; MCA=none
done
[ "$MCA" != none ] || { echo "SETUP_FAILED no Open MPI transport (mpi.txt)" >> "$OUT/summary.txt"; touch "$OUT/done.txt"; exit 1; }
NDEV=$(nvidia-smi -L | wc -l)
DEVS=$(seq -s, 0 $((NDEV - 1)))
export CPPWRF_WRF_RUNS=$W/data/wrf_runs CPPWRF_WRF_REF=$W/data/wrf_ref
cd $W/src
: > "$OUT/same.txt"
FIRST=
for L in ${LAYOUTS//,/ }; do
  [ $SECONDS -lt $END ] || { echo "TIMEOUT before layout $L" >> "$OUT/summary.txt"; break; }
  R=$W/run_$L ENVS= N=1
  # the suite's binary for the layout; its output also goes to run_<layout>.log (the device lines)
  case $L in
    plain) P="$B" ;;
    1) P="$MPIRUN $MCA -np 1 $B" ;;
    t*) P="env CPPWRF_TILES=${L#t} CPPWRF_TILE_DEVICES=$DEVS $B" ;;
    *) N=$(( ${L%x*} * ${L#*x} )); P="env CPPWRF_RANKS=$L $MPIRUN $MCA -np $N $B" ;;
  esac
  printf '#!/bin/bash\nset -o pipefail\n%s "$@" 2>&1 | tee -a %s\n' "$P" "$W/run_$L.log" > $R
  chmod +x $R
  # nvidia-smi every 500 ms: the processes on each GPU and each GPU's memory and utilisation
  nvidia-smi --query-compute-apps=timestamp,pid,gpu_uuid,used_memory --format=csv,noheader -lms 500 > $W/apps_$L.csv 2>&1 &
  S1=$!
  nvidia-smi --query-gpu=timestamp,index,uuid,utilization.gpu,memory.used --format=csv,noheader,nounits -lms 500 > $W/gpus_$L.csv 2>&1 &
  S2=$!
  t0=$SECONDS
  OMP_NUM_THREADS=$(( $(nproc) / N > 0 ? $(nproc) / N : 1 )) timeout $((END - SECONDS)) \
    python3 tests/verify/wrf_suite.py --cuda-dp $R --only "$CASES" --exact --keep --out $W/s_$L > "$OUT/suite_$L.log" 2>&1
  echo "exit $? ($((SECONDS - t0)) s, layout $L, $N processes)" >> "$OUT/suite_$L.log"
  kill $S1 $S2 2> /dev/null; wait $S1 $S2 2> /dev/null
  {
    echo "# rank -> device lines of the runs' logs"
    grep -h "device of each rank\|MPI ranks as\|tiles,\|tiles and" $W/run_$L.log 2> /dev/null | sort | uniq -c
    echo "# nvidia-smi: processes seen per GPU (uuid: distinct pids, peak MiB of one process)"
    awk -F', ' 'NF >= 4 && $3 ~ /GPU-/ { k = $3; p[k " " $2] = 1; m = $4 + 0; if (m > pk[k]) pk[k] = m }
      END { for (x in p) { split(x, a, " "); n[a[1]]++ } for (k in n) print k ": " n[k] " pids, peak " pk[k] " MiB" }' $W/apps_$L.csv
    echo "# nvidia-smi: per GPU index (samples, samples with utilisation > 0, peak utilisation %, peak memory MiB)"
    awk -F', ' 'NF >= 5 { i = $2; s[i]++; if ($4 + 0 > 0) b[i]++; if ($4 + 0 > u[i]) u[i] = $4 + 0; if ($5 + 0 > m[i]) m[i] = $5 + 0 }
      END { for (i in s) print "GPU " i ": " s[i] " samples, " b[i] + 0 " busy, peak " u[i] + 0 " %, peak " m[i] + 0 " MiB" }' $W/gpus_$L.csv
  } > "$OUT/devices_$L.txt" 2>&1
  (cd $W/s_$L 2> /dev/null && find . -type f ! -name '*.log' ! -name 'run.json' ! -name 'run.ini' ! -name 'diag.csv' ! -name 'rsl.*' \
    ! -name 'timing*' -print0 | sort -z | xargs -0 -r sha256sum) > "$OUT/sums_$L.txt"
  if [ -z "$FIRST" ]; then FIRST=$L
  else
    d=$(diff <(awk '{print $2, $1}' "$OUT/sums_$FIRST.txt" | sort) <(awk '{print $2, $1}' "$OUT/sums_$L.txt" | sort) | grep '^[<>]' |
      awk '{print $2}' | sort -u)
    if [ -z "$d" ] && [ -s "$OUT/sums_$L.txt" ]; then echo "SAME $L $(wc -l < "$OUT/sums_$L.txt") files (as $FIRST)" >> "$OUT/same.txt"
    else
      echo "DIFF $L: $(echo "$d" | head -40 | tr '\n' ' ')" >> "$OUT/same.txt"
      # what differs in each file: NetCDF global attributes, dimensions, variables (points that differ, the largest
      # difference, its first index); other files: the number of differing bytes
      for f in $(echo "$d" | head -40); do
        echo "## $f"
        case $f in
          *.nc) python3 -I - $W/s_$FIRST/$f $W/s_$L/$f <<'PY' ;;
import sys
import netCDF4
import numpy as np
a, b = (netCDF4.Dataset(x) for x in sys.argv[1:3])
for k in sorted(set(a.ncattrs()) | set(b.ncattrs())):
    x, y = getattr(a, k, None), getattr(b, k, None)
    if not (np.array_equal(np.asarray(x), np.asarray(y)) if x is not None and y is not None else x is y):
        print(f"attr {k}: {str(x)[:120]!r} | {str(y)[:120]!r}")
for k in sorted(set(a.dimensions) | set(b.dimensions)):
    x = len(a.dimensions[k]) if k in a.dimensions else None
    y = len(b.dimensions[k]) if k in b.dimensions else None
    if x != y:
        print(f"dim {k}: {x} | {y}")
n = 0
for k in sorted(set(a.variables) | set(b.variables)):
    if k not in a.variables or k not in b.variables:
        print(f"var {k}: only in {'first' if k in a.variables else 'this'}")
        continue
    x, y = a.variables[k][...], b.variables[k][...]
    x, y = np.ma.filled(x, 0), np.ma.filled(y, 0)
    if x.shape != y.shape:
        print(f"var {k}: shape {x.shape} | {y.shape}")
        continue
    if x.dtype.kind in "fc":
        bad = ~((x == y) | (np.isnan(x) & np.isnan(y)))
    else:
        bad = x != y
    if bad.any():
        n += 1
        i = np.argwhere(bad)[0]
        dif = np.abs(x.astype(float) - y.astype(float))[bad].max() if x.dtype.kind in "fiu" else "-"
        print(f"var {k} {x.shape}: {int(bad.sum())} of {bad.size} points differ, max |diff| {dif}, first at {tuple(int(v) for v in i)}: "
              f"{x[tuple(i)]!r} | {y[tuple(i)]!r}")
print(f"{n} variables differ")
PY
          *) echo "$(cmp -l $W/s_$FIRST/$f $W/s_$L/$f 2>&1 | wc -l) bytes differ" ;;
        esac
      done > "$OUT/diff_$L.txt" 2>&1
    fi
  fi
  [ "$FIRST" = "$L" ] || rm -rf $W/s_$L
done
{
  for L in ${LAYOUTS//,/ }; do
    [ -f "$OUT/suite_$L.log" ] || continue
    echo "== $L: $(grep '^SUITE' "$OUT/suite_$L.log" | tail -1); $(tail -1 "$OUT/suite_$L.log")"
    grep -E '^(PASS|FAIL|SKIP) ' "$OUT/suite_$L.log" | cut -c1-300
  done
  cat "$OUT/same.txt"
  echo "total $((SECONDS - T0)) s"
} >> "$OUT/summary.txt"
touch "$OUT/done.txt"
exit 0
