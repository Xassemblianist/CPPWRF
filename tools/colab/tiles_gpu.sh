#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The GPU side of tools/colab/colab_tiles.sh on the Colab runtime itself (its Ubuntu with the CUDA toolkit), next to
# the chroot job (tools/colab/tiles_body.sh), whose data it uses (/opt/job is linked to the chroot's):
#
#   tiles_gpu.sh DP_CASES DP_TILE_CASES FP32_CASES LAYOUTS BENCH LAYOUTS_BENCH
#
# Builds CUDA FP64 and FP32 from /content/cpu/src.tar.gz and CUDA FP32 from /content/cpu/in/base_src.tar.gz (the
# base revision), then:
#   - DP_CASES (cuda_dp suite cases) without tiles and DP_TILE_CASES with every other layout of LAYOUTS
#     (CPPWRF_TILES), against the WRF references (tolerance 0): gpu_dp_<layout>.log;
#   - FP32_CASES (cuda suite cases) with the base build and with this build in every layout: the snapshot files of
#     each, and diag.csv without its wall-clock column, must be identical to the base build's without tiles
#     (fp32_same.log);
#   - BENCH (suite cases, FP64 unless NAME@cuda) without history output, with every layout of LAYOUTS_BENCH and
#     without CUDA graphs too: the stepping times (bench.log).
# Logs go to /opt/job (the chroot job's results); gpu_done ends the chroot job's wait.
DP=$1 DPT=$2 FP32=$3 LAYOUTS=$4 BENCH=$5 BLAYOUTS=$6
G=/content/tiles_gpu R=/content/deb13 J=/content/deb13/opt/job
mkdir -p $G
exec > $G/gpu.log 2>&1
set -x
until [ -e /content/cpu/data_done ] || [ -e /content/cpu/done ]; do sleep 10; done
[ -e /content/cpu/done ] && exit 0
ln -sfn $R/opt/job /opt/job
finish() { cp $G/gpu.log $J/gpu_host.log 2> /dev/null; touch $J/gpu_done; }
trap finish EXIT
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader > $J/gpu.txt
nvcc --version | tail -2 >> $J/gpu.txt
for i in $(seq 20); do  # the chroot job may hold apt's locks
  apt-get -o DPkg::Lock::Timeout=600 -qq update > $G/apt.log 2>&1 &&
    apt-get -o DPkg::Lock::Timeout=600 -qq install -y libnetcdf-dev >> $G/apt.log 2>&1 && break
  sleep 30
done
# the suite's comparison (tests/verify/exact.py) reads the files with netCDF4, which a runtime image may lack
python3 -c 'import netCDF4' 2> /dev/null || pip install -q netCDF4 > $G/pip.log 2>&1
rm -rf $G/src $G/base && mkdir -p $G/src $G/base
tar xzf /content/cpu/src.tar.gz -C $G/src
tar xzf /content/cpu/in/base_src.tar.gz -C $G/base
build() {  # SRC BUILD DOUBLE
  local t0=$SECONDS
  cmake -S "$1" -B "$2" -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE="$3" > "$2.log" 2>&1 &&
    cmake --build "$2" -j"$(nproc)" --target cppwrf >> "$2.log" 2>&1
  local rc=$?
  echo "build $2: exit $rc, $((SECONDS - t0)) s" >> $J/gpu.txt
  tail -30 "$2.log" > "$J/$(basename "$2").log"
  return $rc
}
build $G/src $G/b_dp ON &
build $G/src $G/b_sp OFF &
build $G/base $G/b_base OFF &
wait
# layout 1: no CPPWRF_TILES (it would override the run.tiles of the suite's *_tPxQ cases)
tiles_env() { if [ "$1" = 1 ]; then shift; env "$@"; else local l=$1; shift; env CPPWRF_TILES="$l" "$@"; fi; }
export CPPWRF_WRF_RUNS=/opt/job/data/wrf_runs CPPWRF_WRF_REF=/opt/job/data/wrf_ref
cd $G/src
for L in ${LAYOUTS//,/ }; do
  cs=$DP; [ "$L" = 1 ] || cs=$DPT
  [ -n "$cs" ] || continue
  t0=$SECONDS
  tiles_env $L OMP_NUM_THREADS=4 python3 tests/verify/wrf_suite.py --cuda-dp $G/b_dp/cppwrf --only "$cs" --exact \
    --out $G/dp_$L > $J/gpu_dp_$L.log 2>&1
  echo "exit $? ($((SECONDS - t0)) s)" >> $J/gpu_dp_$L.log
  rm -rf $G/dp_$L
done
if [ -n "$FP32" ]; then
  runs="base:$G/b_base:1"
  for L in ${LAYOUTS//,/ }; do runs="$runs new_$L:$G/b_sp:$L"; done
  for r in $runs; do
    IFS=: read -r tag b L <<< "$r"
    tiles_env $L OMP_NUM_THREADS=4 python3 tests/verify/wrf_suite.py --cuda "$b/cppwrf" --only "$FP32" --exact \
      --keep --out $G/sp_$tag > $J/gpu_sp_$tag.log 2>&1
    echo "exit $?" >> $J/gpu_sp_$tag.log
  done
  python3 - $G $J "$FP32" "$LAYOUTS" > $J/fp32_same.log 2>&1 <<'EOF'
import csv, filecmp, glob, os, re, sys
G, J, cases, layouts = sys.argv[1], sys.argv[2], sys.argv[3].split(","), sys.argv[4].split(",")
bad = 0
def rejected(c, L):  # the run stopped at once on the layout (run.tiles: nested runs, 2-D domains)
    try:
        log = open(os.path.join(J, f"gpu_sp_new_{L}.log")).read()
    except OSError:
        return None
    m = re.search(r"^FAIL +" + re.escape(c) + r" +run failed:(.*\n(?!FAIL|PASS|  \()){0,3}?.*(run\.tiles:[^\n]*)", log, re.M)
    return m.group(2) if m else None
def diag(p):  # diag.csv without the wall-clock column
    with open(p) as f:
        return [r[:-1] if r and r[-1] != "" else r for r in csv.reader(f)]
for c in cases:
    base = os.path.join(G, "sp_base", c)
    files = sorted(p for p in glob.glob(os.path.join(base, "**", "*"), recursive=True)
                   if os.path.isfile(p) and (p.endswith(".bin") or p.endswith("diag.csv")))
    for L in layouts:
        r = rejected(c, L) if L != "1" else None
        if r:
            print(f"REJECTED {c} layout {L}: {r}")
            continue
        new = os.path.join(G, f"sp_new_{L}", c)
        diff = 0
        for f in files:
            rel = os.path.relpath(f, base)
            g = os.path.join(new, rel)
            same = os.path.exists(g) and (diag(f) == diag(g) if f.endswith("diag.csv") else filecmp.cmp(f, g, shallow=False))
            if not same:
                if diff < 5:
                    print(f"FAIL {c} {L}: {rel} differs")
                diff += 1
        if not any(f.endswith(".bin") for f in files):
            print(f"FAIL {c} {L}: no snapshot files")
            diff += 1
        print(f"{'SAME' if diff == 0 else 'DIFF'} {c} layout {L}: {len(files)} files (snapshots, diag.csv), {diff} differ")
        bad += diff > 0
print(f"exit {1 if bad else 0}")
EOF
fi
# the bench after the CPU job (the host's cores are free then)
until [ -e $J/cpu_done ]; do sleep 10; done
for spec in ${BENCH//,/ }; do
  name=${spec%@*} prec=${spec#*@}
  [ "$prec" = "$spec" ] && prec=cuda_dp
  b=$G/b_dp; [ "$prec" = cuda ] && b=$G/b_sp
  args=$(python3 - "$name" <<'EOF'
import shlex, sys
sys.path.insert(0, "tests/verify")
import wrf_suite
c = {x[0]: x for x in wrf_suite.CASES}[sys.argv[1]]
s = c[3] + [f"run.run_time={int(c[5])}", "output.out_interval=1000000", "output.diag_interval=1000000"]
print(shlex.join([c[2]] + [a for x in s for a in ("--set", x)]))
EOF
)
  for L in ${BLAYOUTS//,/ } 1:graphs0; do
    env=(CPPWRF_TILES=${L%%:*}); [ "$L" = 1:graphs0 ] && env+=(CPPWRF_GRAPHS=0)
    for rep in 1 2; do
      rm -rf $G/bench_o
      eval "env ${env[*]} OMP_NUM_THREADS=4 $b/cppwrf $args --set output.dir=$G/bench_o" > $G/bench_run.log 2>&1
      echo "BENCH $spec $L rep $rep exit $?: $(grep -E 'stepping' $G/bench_run.log | head -1)" >> $J/bench.log
    done
  done
done
exit 0
