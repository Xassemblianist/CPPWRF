#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The legacy regression tool (tools/regression: run.sh, update_golden.sh) on a Colab runtime's CPU, in the Debian 13
# chroot the suite uses (tools/colab/regression_body.sh; the suite's build flags), so that the goldens come from the
# project's reference environment, not from a workstation.
#
#   CPPWRF_GCP_BUCKET=... tools/colab/colab_regression.sh [--mode update|check] [--gpu G4|CPU|L4|A100|T4] [--out DIR]
#       [--hours H] [--rev REV] [--session NAME] [--keep]
#
# --mode update (default): update_golden.sh --quick --no-gpu, then a second run.sh --quick --no-gpu against the new
# goldens; OUT/golden holds them (copy them into tools/regression/golden; perf_history.csv is not part of the
# goldens). --mode check: run.sh --quick --no-gpu against the goldens of REV. --gpu: the runtime (default G4; its GPU
# is not used, the runs are CPU only; G4 and CPU are AMD: CPPWRF_ANY_CPU=1, the build's portable libmvec gives
# Intel's bits). --session / --keep as colab_suite.sh. No data tars. Results in OUT: update.log, run2.log,
# last_report.txt, logs/ (tools/regression/logs); a summary is printed. Exit 0 when the second run.sh ends with
# exit 0 and setup.out is empty.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${CPPWRF_GCP_BUCKET:?set CPPWRF_GCP_BUCKET}"
BK=$CPPWRF_GCP_BUCKET
GPU=G4 MODE=update OUT= HOURS=3 REV=HEAD S= KEEP=0 NPROC= ANY_CPU=0
while [ $# -gt 0 ]; do
  case $1 in
    --mode) MODE=$2; shift 2 ;; --gpu) GPU=$2; shift 2 ;; --out) OUT=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;;
    --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;; --keep) KEEP=1; shift ;;
    *) sed -n '3,16p' "$0"; exit 2 ;;
  esac
done
case $MODE in update|check) ;; *) echo "--mode update or check"; exit 2 ;; esac
case $GPU in L4|A100|T4) ;; G4|CPU) ANY_CPU=1 ;; *) echo "--gpu $GPU: L4, A100, T4, G4 or CPU"; exit 2 ;; esac
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/regression_$STAMP}
mkdir -p "$OUT"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" "$REV"
COMMIT=$(git -C "$REPO" rev-parse "$REV^{commit}")
echo "$COMMIT" > "$OUT/commit.txt"
echo "[colab_regression] commit ${COMMIT:0:7}, $GPU, mode $MODE"
TAG=colab_regression
. "$REPO/tools/colab/colab_lib.sh"
cj_start "$GPU"
cj_launch "$REPO/tools/colab/regression_body.sh" "" "$MODE" "$COMMIT"
cj_wait "$HOURS" run2.log
cj_results
bad=0
grep . "$OUT/setup.out" 2> /dev/null && bad=1
cat "$OUT/cpu.txt" 2> /dev/null || true
if timeout 900 colab download -s "$S" /content/cpu/fetch/regression.tar.gz "$OUT/regression.tar.gz" > /dev/null 2>&1; then
  tar xzf "$OUT/regression.tar.gz" -C "$OUT" && rm -f "$OUT/regression.tar.gz"
else echo "[colab_regression] no regression.tar.gz"; bad=1; fi
[ "$MODE" = check ] || echo "update_golden: $(grep -E '^Successfully' "$OUT/update.log" 2> /dev/null || echo 'no summary') ($(tail -1 "$OUT/update.log" 2> /dev/null))"
grep -E '^(>> |   (Build|CTest|Execution|Verifier|Golden):)' "$OUT/run2.log" 2> /dev/null || true
tail -2 "$OUT/run2.log" 2> /dev/null || echo "[colab_regression] no run2.log"
grep -q '^exit 0' "$OUT/run2.log" 2> /dev/null || bad=1
[ "$MODE" = check ] || [ -d "$OUT/golden" ] || { echo "[colab_regression] no goldens"; bad=1; }
exit $bad
