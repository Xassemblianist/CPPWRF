#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The paper benchmark (tools/bench/PROTOCOL.md) on ONE Google Colab runtime: WRF v4.8.0 and CPPWRF built on the
# runtime, WRF's process layouts screened, the models timed in interleaved blocks after a warm-up, the GPU's power
# sampled, the bit identity and FP32 accuracy checked on the timed runs' outputs, the report and figures made.
#
#   CPPWRF_GCP_BUCKET=... tools/bench/colab_paper.sh [--gpu G4|L4|A100] [--set full|gpu] [--cases C1,C2,C3,C4]
#       [--wps DIR] [--out DIR] [--hours H] [--rev REV] [--session NAME] [--keep] [--save-lday | --data-lday]
#       [--cpu-rate R] [-- PAPER_RUN_OPTION ...]
#
# --gpu: the runtime (default G4). --set: full (default with G4: WRF and CPPWRF, PROTOCOL.md "Session 1") or gpu
# (default otherwise: the CUDA builds only, against the CPU builds for the bit identity, "Session 2"; default cases
# C1,C2). --cases: the cases of tools/bench/paper_run.py (C1 P4 24 h, C2 the daytime large domain 6 h, C3 Morrison 6 h,
# C4 the two-way nest 6 h). --wps: WPS 4.6.0's directory as tools/bench/colab_bench.sh (the large domain is made on
# the runtime; ~3 MB uploaded). --hours: the longest wait (default 11). --save-lday: put the large domain's inputs on
# Drive (paper_lday_data.tar, ~2 GB) for a later --set gpu session, which then reads them with --data-lday.
# --cpu-rate: the compute units per hour of a CPU-only runtime with the same CPU, a stated assumption for WRF's
# cost (bench_stats.py); not measured here. After "--": options for paper_run.py (e.g. --reps 3 --nc0).
# Data: suite_data.tar (run_P4_r8: the large domain's tables and namelist) from Drive / the bucket as for
# tools/colab/colab_suite.sh, and the inputs of the suite's long references (run_P4_72h_r8 of long72_data.tar,
# run_N3_24h_r8 of long_data.tar) read with ranged reads as tools/bench/colab_fp32.sh does (fp1_long.tar).
# The job: tools/bench/paper_body.sh in the Debian 13 chroot and tools/bench/paper_gpu.sh on the runtime's Ubuntu.
# Results in OUT (default ${CPPWRF_SCRATCH:-~/cppwrf_scratch}/colab/paper_<date>): paper/ (raw_runs.jsonl, runs.csv,
# verify.jsonl, manifest.json, cases.json, best.json, accuracy/, logs/), report.md and summary.json made here with
# the session's rate (`colab usage`), svg/, usage.txt. Exit 1 when the job failed or a build / case did not run.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
case "${1:-}" in -h|--help) sed -n '3,25p' "$0"; exit 0 ;; esac
: "${CPPWRF_GCP_BUCKET:?set CPPWRF_GCP_BUCKET}"
BK=$CPPWRF_GCP_BUCKET
GPU=G4 SET= CASES= WPS=$HOME/wrf_real/WPS-4.6.0 OUT= HOURS=11 REV=HEAD S= KEEP=0 SAVE=0 CPU_RATE= LDAY=0
NPROC= ANY_CPU=1 PR=()
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) GPU=$2; shift 2 ;; --set) SET=$2; shift 2 ;; --cases) CASES=$2; shift 2 ;; --wps) WPS=$2; shift 2 ;;
    --out) OUT=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;; --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;;
    --keep) KEEP=1; shift ;; --save-lday) SAVE=1; shift ;; --data-lday) LDAY=1; shift ;; --cpu-rate) CPU_RATE=$2; shift 2 ;;
    --) shift; PR=("$@"); break ;;
    *) sed -n '3,25p' "$0"; exit 2 ;;
  esac
done
[ -n "$SET" ] || { [ "$GPU" = G4 ] && SET=full || SET=gpu; }
[ -n "$CASES" ] || { [ "$SET" = full ] && CASES=C1,C2,C3,C4 || CASES=C1,C2; }
DATA=suite_data.tar,fp1_long.tar
[ $LDAY = 0 ] || DATA=$DATA,paper_lday_data.tar
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/paper_$STAMP}
mkdir -p "$OUT"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" "$REV"
git -C "$REPO" rev-parse --short "$REV" > "$OUT/commit.txt"
T=$(mktemp -d "$OUT/wps.XXXX"); mkdir -p "$T/geogrid" "$T/metgrid"
cp -L "$WPS"/geogrid.exe "$WPS"/ungrib.exe "$WPS"/metgrid.exe "$T/"
cp -L "$WPS/geogrid/GEOGRID.TBL" "$T/geogrid/"; cp -L "$WPS/metgrid/METGRID.TBL" "$T/metgrid/"
cp -L "$WPS/ungrib/Variable_Tables/Vtable.GFS" "$T/Vtable"; cp "$REPO/tools/bench/namelist.wps" "$T/"
tar czf "$OUT/wps.tar.gz" -C "$T" .; rm -rf "$T"
echo "[colab_paper] commit $(cat "$OUT/commit.txt"), $GPU, set $SET, cases $CASES, data $DATA, paper_run.py ${PR[*]:-}"
TAG=colab_paper
. "$REPO/tools/colab/colab_lib.sh"
cj_start "$GPU"
cj_send "$OUT/wps.tar.gz" in/wps.tar.gz
cj_send "$OUT/commit.txt" in/commit.txt
cj_send "$REPO/tools/bench/paper_gpu.sh" paper_gpu.sh
# the long cases' inputs: the leading members of two bucket tars, up to their first wrfout member (as colab_fp32.sh)
cj_token || { echo "[colab_paper] token upload failed"; exit 1; }
rx 1800 <<EOF | grep LONG_OK > /dev/null || { echo "[colab_paper] the long inputs were not fetched"; exit 1; }
import os, subprocess
T = "/content/tars"; os.makedirs(T, exist_ok=True)
if not os.path.exists(T + "/fp1_long.tar.ok"):
    with open(T + "/fp1_long.tar", "wb") as o:
        for obj, end in (("long72_data.tar", 122625536), ("long_data.tar", 106426368)):
            r = subprocess.run(["gcloud", "storage", "--access-token-file=/content/cpu/gcs.tok", "cat", "-r", f"0-{end - 1}",
                                "gs://$BK/" + obj], stdout=o)
            assert r.returncode == 0 and o.tell() % 512 == 0, obj
        o.write(b"\0" * 10240)
n = subprocess.run(["tar", "tf", T + "/fp1_long.tar"], capture_output=True, text=True).stdout.split()
assert any(x.endswith("run_P4_72h_r8/wrfbdy_d01") for x in n) and any(x.endswith("run_N3_24h_r8/wrfinput_d02") for x in n), n[-5:]
open(T + "/fp1_long.tar.ok", "w").close()
print("LONG_OK", os.path.getsize(T + "/fp1_long.tar"), len(n))
EOF
# the chroot job (remote_chroot_job.sh starts it with an empty environment: --save-lday goes into its copy)
[ $SAVE = 0 ] || echo "[colab_paper] the large domain's inputs go to Drive (paper_lday_data.tar)"
BODY=$OUT/paper_body.sh
{ head -1 "$REPO/tools/bench/paper_body.sh"; [ $SAVE = 1 ] && echo "export PAPER_SAVE_LDAY=1"; tail -n +2 "$REPO/tools/bench/paper_body.sh"; } > "$BODY"
cj_launch "$BODY" "$DATA" "$SET" "$CASES" "$STAMP" "${PR[@]}"
echo "import subprocess; subprocess.Popen('nohup bash /content/cpu/paper_gpu.sh $STAMP > /dev/null 2>&1 &', shell=True); print('started')" |
  rx 60 | grep started > /dev/null || echo "[colab_paper] the GPU side did not start"
bad=0
cj_wait "$HOURS" ../paper/out/paper.log || bad=1
timeout 120 colab usage 2>&1 | grep -v '^\[colab\]' > "$OUT/usage.txt" || true
cj_results || bad=1
if timeout 1800 colab download -s "$S" /content/cpu/fetch/paper.tar.gz "$OUT/paper.tar.gz" > /dev/null 2>&1; then
  tar xzf "$OUT/paper.tar.gz" -C "$OUT" && rm -f "$OUT/paper.tar.gz"
else echo "[colab_paper] paper.tar.gz: download failed"; bad=1; fi
grep . "$OUT/setup.out" 2> /dev/null && bad=1
rate=$(sed -nE 's/.*Usage rate: *([0-9.]+).*/\1/p' "$OUT/usage.txt" | head -1)
P=$OUT/paper
if [ -f "$P/raw_runs.jsonl" ]; then
  A=("$P/raw_runs.jsonl" --verify "$P/verify.jsonl" --md "$OUT/report.md" --json "$OUT/summary.json" --csv "$OUT/runs.csv")
  for f in manifest cases best; do [ -f "$P/$f.json" ] && A+=(--$f "$P/$f.json"); done
  [ -f "$P/accuracy.md" ] && A+=(--accuracy-md "$P/accuracy.md")
  python3 "$REPO/tools/bench/bench_stats.py" "${A[@]}" ${rate:+--rate "$rate"} ${CPU_RATE:+--cpu-rate "$CPU_RATE"}
  python3 "$REPO/tools/bench/bench_plot.py" "$OUT/summary.json" --out "$OUT/svg" > /dev/null
  echo "[colab_paper] $GPU rate ${rate:-?} compute units / hour; report: $OUT/report.md, figures: $OUT/svg"
else echo "[colab_paper] no raw_runs.jsonl"; bad=1; fi
tail -40 "$P/paper.log" 2> /dev/null || true
exit $bad
