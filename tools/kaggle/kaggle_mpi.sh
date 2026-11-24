#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Several GPUs: the CUDA FP64 build with CPPWRF_MPI on a Kaggle GPU runtime with two Tesla T4 (tools/kaggle/kg_body.sh):
# cuda_dp suite cases with every layout (one process, MPI ranks one per GPU or several per GPU, tiles over the GPUs),
# each against the WRF references at tolerance 0, and every layout's output files byte for byte against the first's.
#
#   tools/kaggle/kaggle_mpi.sh --cases NAME,... [--layouts plain,1,1x2,2x1,2x2,t1x2,t2x1] [--data TAR,...] [--rev REV]
#       [--name NAME] [--hours H] [--out DIR] [--no-wait]
#
# Needs the kaggle CLI, authenticated (kaggle config view names the user; or KAGGLE_USERNAME); the account's phone
# verification enables a GPU with internet. The job's source (git archive REV), kg_body.sh and job.env go up as the
# private dataset USER/cppwrf-NAME-src (created, else a new version); the private script kernel USER/cppwrf-NAME
# (machine NvidiaTeslaT4: 2 x T4) runs them, and its output is downloaded to OUT/output when it ends (--no-wait:
# push only; later `kaggle kernels output USER/cppwrf-NAME -p OUT/output`). Layouts: kg_body.sh (L@on / L@off: the halo
# transport forced, docs/MPI.md section 4; env.txt says whether the node's Open MPI takes device memory). Data: the cases'
# tars (wrf_suite.py --data-for) from the public release Xassemblianist/cppwrf-data data-v1 unless --data. --hours: the
# job's own limit for its suite runs (the GPU quota counts the whole session, build and data included).
# Exit 0 when every layout's suite ends with exit 0 and every layout's files are those of the first layout.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CASES= LAYOUTS=plain,1,1x2,2x1,2x2,t1x2,t2x1 DATA= REV=HEAD NAME=kg1 HOURS=3 OUT= WAIT=1
while [ $# -gt 0 ]; do
  case $1 in
    --cases) CASES=$2; shift 2 ;; --layouts) LAYOUTS=$2; shift 2 ;; --data) DATA=$2; shift 2 ;;
    --rev) REV=$2; shift 2 ;; --name) NAME=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;; --out) OUT=$2; shift 2 ;;
    --no-wait) WAIT=0; shift ;;
    *) sed -n '3,17p' "$0"; exit 2 ;;
  esac
done
[ -n "$CASES" ] || { echo "--cases"; exit 2; }
command -v kaggle > /dev/null || { echo "no kaggle CLI"; exit 2; }
KU=${KAGGLE_USERNAME:-$(kaggle config view 2> /dev/null | awk '/- username:/ {print $3}')}
[ -n "$KU" ] || { echo "no Kaggle user (KAGGLE_USERNAME or kaggle config view)"; exit 2; }
[ -n "$DATA" ] || DATA=$(python3 "$REPO/tests/verify/wrf_suite.py" --data-for "$CASES")
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/kaggle/${NAME}_$STAMP}
DS=$OUT/dataset KD=$OUT/kernel
rm -rf "$DS" "$KD"; mkdir -p "$DS" "$KD"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
# .dat: Kaggle unpacks the archives it knows in a dataset
git -C "$REPO" archive --format=tar.gz -o "$DS/src.tgz.dat" "$REV"
COMMIT=$(git -C "$REPO" rev-parse --short "$REV")
cp "$REPO/tools/kaggle/kg_body.sh" "$DS/kg_body.sh"
printf 'CASES=%q\nLAYOUTS=%q\nDATA=%q\nCOMMIT=%q\nHOURS=%q\n' "$CASES" "$LAYOUTS" "$DATA" "$COMMIT" "$HOURS" > "$DS/job.env"
cat > "$DS/dataset-metadata.json" <<EOF
{"title": "cppwrf-$NAME-src", "id": "$KU/cppwrf-$NAME-src", "licenses": [{"name": "other"}]}
EOF
echo "[kaggle_mpi] commit $COMMIT, cases $CASES, layouts $LAYOUTS, data $DATA -> $OUT"
if kaggle datasets status "$KU/cppwrf-$NAME-src" > /dev/null 2>&1; then
  kaggle datasets version -p "$DS" -m "$COMMIT $STAMP" -r skip > "$OUT/dataset.log" 2>&1
else
  kaggle datasets create -p "$DS" -r skip > "$OUT/dataset.log" 2>&1  # private unless --public
fi
cat "$OUT/dataset.log"
for i in $(seq 60); do
  s=$(kaggle datasets status "$KU/cppwrf-$NAME-src" 2>&1 || true)
  case $s in *ready*) break ;; *error*) echo "dataset: $s"; exit 1 ;; esac
  sleep 10
done
# the kernel: finds its dataset under /kaggle/input and runs kg_body.sh; the results stay in /kaggle/working/kg
cat > "$KD/run.py" <<'EOF'
import glob, os, subprocess, sys
body = sorted(glob.glob("/kaggle/input/**/kg_body.sh", recursive=True))
if not body:
    sys.exit("no kg_body.sh under /kaggle/input")
src = os.path.dirname(body[0])
rc = subprocess.call(["bash", body[0], src, "/kaggle/working/kg"])
for f in ("summary.txt", "env.txt", "same.txt"):
    p = os.path.join("/kaggle/working/kg", f)
    if os.path.exists(p):
        print(f"== {f}\n" + open(p).read(), flush=True)
sys.exit(rc)
EOF
cat > "$KD/kernel-metadata.json" <<EOF
{"id": "$KU/cppwrf-$NAME", "title": "cppwrf-$NAME", "code_file": "run.py", "language": "python",
 "kernel_type": "script", "is_private": true, "enable_gpu": true, "enable_internet": true,
 "machine_shape": "NvidiaTeslaT4", "dataset_sources": ["$KU/cppwrf-$NAME-src"], "competition_sources": [],
 "kernel_sources": []}
EOF
kaggle kernels push -p "$KD" | tee "$OUT/push.log"
date +%s > "$OUT/pushed_at"
[ "$WAIT" = 1 ] || exit 0
until s=$(kaggle kernels status "$KU/cppwrf-$NAME" 2>&1); echo "$s" | grep -qiE 'complete|error|cancel'; do sleep 60; done
echo "[kaggle_mpi] $s, $(( ($(date +%s) - $(cat "$OUT/pushed_at")) / 60 )) min after the push"
mkdir -p "$OUT/output"
kaggle kernels output "$KU/cppwrf-$NAME" -p "$OUT/output" > "$OUT/output.log" 2>&1 || cat "$OUT/output.log"
R=$OUT/output/kg
[ -f "$R/summary.txt" ] || { echo "no summary.txt"; exit 1; }
cat "$R/env.txt" "$R/summary.txt"
bad=0
grep -q SETUP_FAILED "$R/summary.txt" && bad=1
for L in ${LAYOUTS//,/ }; do
  grep -q '^exit 0 ' "$R/suite_$L.log" 2> /dev/null || bad=1
done
[ "$(grep -c '^SAME ' "$R/same.txt" 2> /dev/null)" = "$(( $(echo "${LAYOUTS//,/ }" | wc -w) - 1 ))" ] || bad=1
exit $bad
