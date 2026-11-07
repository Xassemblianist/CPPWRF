#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Tile invariance on a Colab runtime's CPU (tools/colab/tiles_body.sh in the Debian 13 chroot, as colab_suite.sh):
# suite cases run with every tile layout (CPPWRF_TILES; each layout is compared with the WRF references at
# tolerance 0, as without tiles), and cases under the tile check (CPPWRF_TILE_CHECK=1).
#
#   CPPWRF_GCP_BUCKET=... tools/colab/colab_tiles.sh --cases NAME,... [--layouts 1,2x1,1x2,2x2] [--check NAME,...]
#       [--check-layout 2x2] [--gpu G4] [--jobs N] [--threads N] [--data OBJECT,...] [--out DIR] [--hours H]
#       [--rev REV] [--session NAME] [--keep]
#       [--gpu-dp NAME,...] [--gpu-dp-tiles NAME,...] [--gpu-fp32 NAME,...] [--gpu-layouts 1,2x1,2x2]
#       [--bench NAME[@cuda],...] [--bench-layouts 1,2x1,1x2,2x2] [--base REV]
#
# --gpu-dp / --gpu-dp-tiles / --gpu-fp32 / --bench: tools/colab/tiles_gpu.sh on the runtime's GPU at the same time:
# cuda_dp cases without tiles (--gpu-dp) and with every other --gpu-layouts layout (--gpu-dp-tiles) against the WRF
# references, cuda (FP32) cases whose snapshots must be identical to those of the --base revision's build (default
# origin/main) without tiles in every layout, and the stepping times of the --bench cases (no history output) with
# every --bench-layouts layout (gpu_dp_*.log, fp32_same.log, bench.log).
#
# Data: the tars of the cases (wrf_suite.py --data-for) unless --data. --session / --keep as colab_suite.sh (a kept
# session keeps the build in /opt/tiles of its chroot: the next run's build is incremental). Results in OUT:
# suite_<layout>.log, check_<case>.log, cmake.log; a summary is printed. Exit 0 when every layout's suite ends with
# exit 0 and every check run passes.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${CPPWRF_GCP_BUCKET:?set CPPWRF_GCP_BUCKET}"
BK=$CPPWRF_GCP_BUCKET
GPU=G4 CASES= LAYOUTS=1,2x1,1x2,2x2 CHECK= CHECK_LAYOUT=2x2 DATA= OUT= JOBS=4 THREADS= HOURS=4 REV=HEAD S= KEEP=0
NPROC= ANY_CPU=1 GDP= GDPT= GSP= GLAY=1,2x1,2x2 BENCH= BLAY=1,2x1,1x2,2x2 BASE=origin/main
while [ $# -gt 0 ]; do
  case $1 in
    --cases) CASES=$2; shift 2 ;; --layouts) LAYOUTS=$2; shift 2 ;; --check) CHECK=$2; shift 2 ;;
    --check-layout) CHECK_LAYOUT=$2; shift 2 ;; --gpu) GPU=$2; shift 2 ;; --jobs) JOBS=$2; shift 2 ;;
    --threads) THREADS=$2; shift 2 ;; --data) DATA=$2; shift 2 ;; --out) OUT=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;;
    --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;; --keep) KEEP=1; shift ;;
    --gpu-dp) GDP=$2; shift 2 ;; --gpu-dp-tiles) GDPT=$2; shift 2 ;; --gpu-fp32) GSP=$2; shift 2 ;;
    --gpu-layouts) GLAY=$2; shift 2 ;;
    --bench) BENCH=$2; shift 2 ;; --bench-layouts) BLAY=$2; shift 2 ;; --base) BASE=$2; shift 2 ;;
    *) sed -n '3,17p' "$0"; exit 2 ;;
  esac
done
[ -n "$CASES$CHECK$GDP$GDPT$GSP$BENCH" ] || { echo "--cases, --check, --gpu-dp, --gpu-fp32 or --bench"; exit 2; }
GPUJOB=0; [ -z "$GDP$GDPT$GSP$BENCH" ] || GPUJOB=1
ALL=$(printf '%s\n%s\n%s\n%s\n%s\n%s\n' "${CASES//,/$'\n'}" "${CHECK//,/$'\n'}" "${GDP//,/$'\n'}" "${GDPT//,/$'\n'}" "${GSP//,/$'\n'}" \
  "$(echo "${BENCH//,/$'\n'}" | sed 's/@.*//')" | grep . | sort -u | paste -sd,)
[ -n "$DATA" ] || DATA=$(python3 "$REPO/tests/verify/wrf_suite.py" --data-for "$ALL")
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/tiles_$STAMP}
mkdir -p "$OUT"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" "$REV"
git -C "$REPO" rev-parse --short "$REV" > "$OUT/commit.txt"
[ "$GPUJOB" = 0 ] || git -C "$REPO" archive --format=tar.gz -o "$OUT/base_src.tar.gz" "$BASE"
echo "[colab_tiles] commit $(cat "$OUT/commit.txt"), $GPU, cases $CASES, layouts $LAYOUTS${CHECK:+, check $CHECK ($CHECK_LAYOUT)}, data $DATA"
TAG=colab_tiles
. "$REPO/tools/colab/colab_lib.sh"
cj_start "$GPU"
THR=${THREADS:-$(( NPROC / JOBS > 0 ? NPROC / JOBS : 1 ))}
if [ "$GPUJOB" = 1 ]; then
  cj_send "$OUT/base_src.tar.gz" in/base_src.tar.gz && cj_send "$REPO/tools/colab/tiles_gpu.sh" gpu.sh || exit 1
  rm -f "$OUT/base_src.tar.gz"
fi
cj_launch "$REPO/tools/colab/tiles_body.sh" "$DATA" "$CASES" "$LAYOUTS" "$CHECK" "$CHECK_LAYOUT" "$JOBS" "$THR" "$GPUJOB"
if [ "$GPUJOB" = 1 ]; then
  q() { printf "'%s' " "${@//\'/}"; }
  echo "import subprocess; subprocess.Popen(\"nohup bash /content/cpu/gpu.sh $(q "$GDP" "$GDPT" "$GSP" "$GLAY" "$BENCH" "$BLAY")> /dev/null 2>&1 &\", shell=True); print('started')" |
    rx 60 | grep started > /dev/null || echo "[colab_tiles] the GPU side did not start"
fi
cj_wait "$HOURS" cmake.log
cj_results
bad=0
grep . "$OUT/setup.out" 2> /dev/null && bad=1
for L in ${LAYOUTS//,/ }; do
  [ -n "$CASES" ] || break
  f=$OUT/suite_$L.log
  [ -f "$f" ] || { echo "layout $L: no log"; bad=1; continue; }
  echo "== layout $L: $(grep -E '^SUITE' "$f" | tail -1) $(tail -1 "$f")"
  grep -E '^(FAIL|SKIP) ' "$f" | cut -c1-400 || true
  grep -q '^exit 0' "$f" || bad=1
done
for c in ${CHECK//,/ }; do
  f=$OUT/check_$c.log
  echo "== check $c ($CHECK_LAYOUT): $(grep -E '^(PASS|FAIL|SKIP) ' "$f" 2> /dev/null | cut -c1-600 || echo 'no log')"
  cat "$OUT/check_$c.txt" 2> /dev/null || true
  grep -q '^exit 0' "$f" 2> /dev/null || bad=1
done
if [ "$GPUJOB" = 1 ]; then
  cat "$OUT/gpu.txt" 2> /dev/null || echo "[colab_tiles] no gpu.txt"
  for L in ${GLAY//,/ }; do
    [ -n "$GDP$GDPT" ] || break
    f=$OUT/gpu_dp_$L.log
    echo "== GPU FP64 layout $L: $(grep -cE '^PASS ' "$f" 2> /dev/null) PASS, $(tail -1 "$f" 2> /dev/null)"
    grep -E '^(FAIL|SKIP) ' "$f" 2> /dev/null | cut -c1-400 || true
    grep -q '^exit 0' "$f" 2> /dev/null || bad=1
  done
  if [ -n "$GSP" ]; then
    grep -E '^(SAME|DIFF|FAIL|REJECTED|exit)' "$OUT/fp32_same.log" 2> /dev/null || echo "[colab_tiles] no fp32_same.log"
    grep -q '^exit 0' "$OUT/fp32_same.log" 2> /dev/null || bad=1
  fi
  cat "$OUT/bench.log" 2> /dev/null || true
fi
exit $bad
