#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The chroot job of tools/colab/colab_tiles.sh: the CPU FP64 build and suite cases run with several tile layouts
# (CPPWRF_TILES), each against the WRF references at tolerance 0, and optionally under the tile check
# (CPPWRF_TILE_CHECK=1: every launch of a step compared with a run without tiles).
#
#   tiles_body.sh J CASES LAYOUTS CHECK_CASES CHECK_LAYOUT JOBS THREADS [GPU]
#
# J holds src/ and data/ (tools/colab/remote_chroot_job.sh). The build lives in /opt/tiles (kept by a --keep session:
# only the files that changed get a new time stamp, so the next build is incremental). Logs in J: setup.out,
# cmake.log, suite_<layout>.log, check_<case>.log. GPU=1: tools/colab/tiles_gpu.sh runs on the runtime next to this
# job; cpu_done tells it the CPU is free (its timings), and this job ends when it has written gpu_done (at most 6 h).
J=$1 CASES=$2 LAYOUTS=$3 CHECK=$4 CHECK_LAYOUT=$5 JOBS=$6 THR=$7 GPU=${8:-0}
P=/opt/tiles
set -x
touch $J/setup.out
mkdir -p $P/src
python3 - $J/src $P/src <<'EOF'
import filecmp, os, shutil, sys
src, dst = sys.argv[1], sys.argv[2]
keep = set()
for root, dirs, files in os.walk(src):
    rel = os.path.relpath(root, src)
    os.makedirs(os.path.join(dst, rel), exist_ok=True)
    for f in files:
        a, b = os.path.join(root, f), os.path.join(dst, rel, f)
        keep.add(os.path.normpath(b))
        if not os.path.exists(b) or not filecmp.cmp(a, b, shallow=False):
            shutil.copy(a, b)  # a new time stamp: rebuilt
for root, dirs, files in os.walk(dst):
    for f in files:
        b = os.path.normpath(os.path.join(root, f))
        if b not in keep:
            os.remove(b)
EOF
[ -f $P/build/CMakeCache.txt ] || cmake -S $P/src -B $P/build -DCMAKE_BUILD_TYPE=Release -DCPPWRF_DOUBLE=ON \
  -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_NATIVE=OFF -DCMAKE_CXX_FLAGS="-march=skylake -mtune=skylake" > $J/cmake.log 2>&1
wait_gpu() {
  touch $J/cpu_done
  [ "$GPU" = 1 ] || return 0
  local end=$((SECONDS + 6 * 3600))
  until [ -e $J/gpu_done ] || [ $SECONDS -gt $end ]; do sleep 20; done
}
cmake --build $P/build -j"$(nproc)" --target cppwrf >> $J/cmake.log 2>&1 ||
  { echo "BUILD_FAILED (cmake.log)" >> $J/setup.out; wait_gpu; exit 1; }
cd $P/src
# layout 1: no CPPWRF_TILES (it would override the run.tiles of the suite's *_tPxQ cases)
tiles_env() { if [ "$1" = 1 ]; then shift; env "$@"; else local l=$1; shift; env CPPWRF_TILES="$l" "$@"; fi; }
export CPPWRF_WRF_RUNS=$J/data/wrf_runs CPPWRF_WRF_REF=$J/data/wrf_ref
for L in ${LAYOUTS//,/ }; do
  [ -n "$CASES" ] || break
  t0=$SECONDS
  tiles_env $L OMP_NUM_THREADS=$THR python3 tests/verify/wrf_suite.py --cpu-dp $P/build/cppwrf --jobs "$JOBS" \
    --out $J/s_$L --only "$CASES" --exact > $J/suite_$L.log 2>&1
  echo "exit $? ($((SECONDS - t0)) s)" >> $J/suite_$L.log
  rm -rf $J/s_$L*
done
for c in ${CHECK//,/ }; do
  CPPWRF_TILES=$CHECK_LAYOUT CPPWRF_TILE_CHECK=1 CPPWRF_TILE_CHECK_LOG=$J/check_$c.txt OMP_NUM_THREADS=$(nproc) python3 tests/verify/wrf_suite.py \
    --cpu-dp $P/build/cppwrf --jobs 1 --out $J/c_$c --only "$c" --exact > $J/check_$c.log 2>&1
  echo "exit $?" >> $J/check_$c.log
  rm -rf $J/c_$c*
done
wait_gpu
exit 0
