#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The paper benchmark (tools/bench/PROTOCOL.md) on ONE Google Colab runtime: WRF v4.8.0 and CPPWRF built on the
# runtime, WRF's process layouts screened, the models timed in interleaved blocks after a warm-up, the GPU's power
# sampled, the bit identity and FP32 accuracy checked on the timed runs' outputs, the report and figures made.
# No cloud bucket and no gcloud: the data come from Drive (rclone remote cppwrf-drive, folder cppwrf-data), else from
# the public GitHub release cppwrf-data data-v1; the results come back with `colab download` and are also on Drive
# (cppwrf-drive:cppwrf-g4results/paper_<date>/).
#
#   tools/bench/colab_paper.sh [--gpu G4|L4|A100] [--set full|gpu] [--cases C1,C2,C3,C4] [--wps DIR] [--out DIR]
#       [--hours H] [--rev REV] [--session NAME] [--keep] [--save-lday | --data-lday] [--cpu-rate R]
#       [--dry-run] [-- PAPER_RUN_OPTION ...]
#
# --gpu: the runtime (default G4). --set: full (default with G4: WRF and CPPWRF, PROTOCOL.md "Session 1") or gpu
# (default otherwise: the CUDA builds against the CPU builds, "Session 2", cases C1,C2). --cases: tools/bench/
# paper_run.py's cases. --wps: WPS 4.6.0's directory (geogrid / ungrib / metgrid executables and tables, ~3 MB
# uploaded when C2 is in --cases without --data-lday; default ~/wrf_real/WPS-4.6.0). --hours: the longest wait (default 11). --save-lday: put C2's inputs on
# Drive (cppwrf-data/paper_lday_data.tar, ~2 GB) for a later --data-lday session, which reads them instead of making
# them. --cpu-rate: compute units per hour of a CPU-only runtime with the same CPU, a stated assumption for WRF's
# cost (bench_stats.py). --dry-run: print the data list, the job's arguments, where each tar is and paper_run.py's
# estimate; start nothing. After "--": options for paper_run.py (e.g. --reps 4 --nc0).
# Data: fp2_data.tar (run_FP2_P4_24h_r8: C1, C3 and the large domain's tables) and n3_data.tar (run_N3h_r8: C4),
# with --data-lday also paper_lday_data.tar. The job: tools/bench/paper_job.sh on the runtime (the Debian 13 chroot,
# the data, tools/bench/paper_gpu.sh, tools/bench/paper_body.sh). Results in OUT (default
# ${CPPWRF_SCRATCH:-~/cppwrf_scratch}/colab/paper_<date>): paper/ (raw_runs.jsonl, verify.jsonl, manifest.json,
# cases.json, best.json, accuracy/, logs/), job/ (the job's logs), report.md and summary.json made here with the
# session's rate (`colab usage`), runs.csv, svg/, usage.txt. The session is stopped at the end unless --keep.
# Exit 1 when the job failed, data or a build is missing, or the results did not come back.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GPU=G4 SET= CASES= WPS=$HOME/wrf_real/WPS-4.6.0 OUT= HOURS=11 REV=HEAD S= KEEP=0 SAVE=0 LDAY=0 CPU_RATE= DRY=0 PR=()
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) GPU=$2; shift 2 ;; --set) SET=$2; shift 2 ;; --cases) CASES=$2; shift 2 ;; --wps) WPS=$2; shift 2 ;;
    --out) OUT=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;; --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;;
    --keep) KEEP=1; shift ;; --save-lday) SAVE=1; shift ;; --data-lday) LDAY=1; shift ;;
    --cpu-rate) CPU_RATE=$2; shift 2 ;; --dry-run) DRY=1; shift ;;
    --) shift; PR=("$@"); break ;;
    -h|--help) sed -n '3,28p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; sed -n '10,12p' "$0"; exit 2 ;;
  esac
done
case $GPU in G4|L4|A100) ;; *) echo "--gpu $GPU: G4, L4 or A100"; exit 2 ;; esac
[ -n "$SET" ] || { [ "$GPU" = G4 ] && SET=full || SET=gpu; }
case $SET in full|gpu) ;; *) echo "--set $SET: full or gpu"; exit 2 ;; esac
[ -n "$CASES" ] || { [ "$SET" = full ] && CASES=C1,C2,C3,C4 || CASES=C1,C2; }
[ $SAVE = 1 ] && [ $LDAY = 1 ] && { echo "--save-lday and --data-lday exclude each other"; exit 2; }
DATA=fp2_data.tar
case ",$CASES," in *,C4,*) DATA=$DATA,n3_data.tar ;; esac
[ $LDAY = 0 ] || DATA=$DATA,paper_lday_data.tar
NEED_WPS=0
case ",$CASES," in *,C2,*) [ $LDAY = 0 ] && NEED_WPS=1 ;; esac
STAMP=$(date +%Y%m%d_%H%M%S)
EST=()
[ "$SET" = full ] || EST=(--models gpu_fp32,gpu_fp32id,gpu_fp64 --proof cpu_fp64,cpu_fp32)
if [ $DRY = 1 ]; then
  echo "[colab_paper] dry run: $GPU, set $SET, cases $CASES, data $DATA, save-lday $SAVE"
  echo "[colab_paper] job: paper_job.sh $SET $CASES $STAMP $DATA ${PR[*]:-}"
  T=$(mktemp -d "${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/paper_plan.XXXX")
  python3 "$REPO/tools/bench/paper_run.py" plan --root "$T" --cases "$CASES" "${EST[@]}" "${PR[@]}"; rm -rf "$T"
  SUMS=$(timeout 60 curl -sSfL https://github.com/Xassemblianist/cppwrf-data/releases/download/data-v1/SHA256SUMS 2> /dev/null)
  for t in ${DATA//,/ }; do
    w=
    te=$(printf '%s\n' "$t" | sed 's/\./\\./g')
    timeout 120 rclone lsf "cppwrf-drive:cppwrf-data/$t" 2> /dev/null | grep -qx "$t" && w="Drive"
    grep -qE "  ${te}(\.part[0-9a-zA-Z]+)?$" <<< "$SUMS" && w="${w:+$w and }the GitHub release"
    echo "[colab_paper] $t: ${w:-NOT FOUND (Drive or GitHub)}"
  done
  if [ $NEED_WPS = 1 ]; then
    [ -x "$WPS/geogrid.exe" ] && echo "[colab_paper] WPS: $WPS" || echo "[colab_paper] WPS: $WPS has no geogrid.exe"
  else
    echo "[colab_paper] WPS: not needed"
  fi
  exit 0
fi
command -v colab > /dev/null && command -v rclone > /dev/null || { echo "[colab_paper] needs the colab CLI and rclone"; exit 1; }
rclone listremotes | grep -qx "cppwrf-drive:" || { echo "[colab_paper] no rclone remote cppwrf-drive"; exit 1; }
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/paper_$STAMP}
mkdir -p "$OUT/in"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] ||
  echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/in/src.tar.gz" "$REV" || exit 1
git -C "$REPO" rev-parse --short "$REV" > "$OUT/in/commit.txt"
if [ $NEED_WPS = 1 ]; then
  T=$(mktemp -d "$OUT/wps.XXXX"); mkdir -p "$T/geogrid" "$T/metgrid"
  cp -L "$WPS"/geogrid.exe "$WPS"/ungrib.exe "$WPS"/metgrid.exe "$T/" &&
    cp -L "$WPS/geogrid/GEOGRID.TBL" "$T/geogrid/" && cp -L "$WPS/metgrid/METGRID.TBL" "$T/metgrid/" &&
    cp -L "$WPS/ungrib/Variable_Tables/Vtable.GFS" "$T/Vtable" && cp "$REPO/tools/bench/namelist.wps" "$T/" ||
    { echo "[colab_paper] WPS files missing in $WPS"; rm -rf "$T"; exit 1; }
  tar czf "$OUT/in/wps.tar.gz" -C "$T" .; rm -rf "$T"
fi
[ $SAVE = 1 ] && date > "$OUT/in/save_lday"
echo "[colab_paper] commit $(cat "$OUT/in/commit.txt"), $GPU, set $SET, cases $CASES, data $DATA, paper_run.py ${PR[*]:-}"
TAG=colab_paper
. "$REPO/tools/colab/colab_guard.sh"  # the only way to ask for a runtime: one attempt, rate limit, block lock
rx() { timeout "$(( $1 + 120 ))" colab exec -s "$S" --timeout "$1" 2>/dev/null | grep -v '^\[colab\]'; }
cleanup() {
  rm -f "$OUT/in/rclone.conf"
  [ -n "$S" ] || return 0
  echo 'import subprocess; subprocess.run("rm -f /content/in/rclone.conf", shell=True)' | rx 60 > /dev/null 2>&1 || true
  if [ "$KEEP" = 1 ]; then echo "[$TAG] --keep: session $S left running (colab stop -s $S)"
  else timeout 300 colab stop -s "$S" || echo "[$TAG] WARNING: could not stop $S, check: colab sessions"; fi
}
trap cleanup EXIT
if [ -z "$S" ]; then  # one request, no retry (tools/colab/colab_guard.sh)
  S=cppwrf-paper-$(echo "$GPU" | tr A-Z a-z)-$(date +%Y%m%d_%H%M%S)
  echo "[$TAG] new session $S ($GPU)"
  colab_guard_new "$S" --gpu "$GPU" || { echo "[$TAG] no $GPU session"; exit 1; }
fi
echo 'import os, shutil; os.makedirs("/content/in", exist_ok=True); print("DISK", shutil.disk_usage("/").total // 2**30)' |
  rx 60 | grep DISK || { echo "[$TAG] session not answering"; exit 1; }
(umask 077; rclone config show cppwrf-drive > "$OUT/in/rclone.conf")
UPLOADS=(src.tar.gz commit.txt rclone.conf)
[ $NEED_WPS = 1 ] && UPLOADS+=(wps.tar.gz)
[ $SAVE = 1 ] && UPLOADS+=(save_lday)
for f in "${UPLOADS[@]}"; do
  ok=0
  for t in 1 2 3; do timeout 900 colab upload -s "$S" "$OUT/in/$f" "/content/in/$f" 2>&1 | grep -q Uploaded && { ok=1; break; }; sleep 20; done
  [ $ok = 1 ] || { echo "[$TAG] upload of $f failed"; exit 1; }
done
rm -f "$OUT/in/rclone.conf"
ARGS=$(printf "'%s' " "$SET" "$CASES" "$STAMP" "$DATA" "${PR[@]//\'/}")
echo "import subprocess; subprocess.Popen(\"chmod 600 /content/in/rclone.conf; mkdir -p /content/paper_src && tar xzf /content/in/src.tar.gz -C /content/paper_src tools/bench/paper_job.sh && nohup bash /content/paper_src/tools/bench/paper_job.sh $ARGS > /dev/null 2>&1 &\", shell=True); print('started')" |
  rx 120 | grep -q started || { echo "[$TAG] the job did not start"; exit 1; }
echo "[$TAG] $(date +%H:%M) job started (stamp $STAMP)"
bad=0 miss=0 end=$((SECONDS + HOURS * 3600))
while :; do
  sleep 300
  st=$(rx 60 <<'EOF' || true
import os, subprocess
C = "/content/paper_job"
alive = subprocess.run(["pgrep", "-f", "paper_job.sh"], capture_output=True).returncode == 0
print("JOB-DONE" if os.path.exists(C + "/done") else "JOB-RUNNING" if alive else "JOB-DIED")
for p in (C + "/data.txt", "/content/deb13/opt/paper/out/paper.log"):
    if os.path.exists(p):
        ls = open(p).read().splitlines()
        print(os.path.basename(p), len(ls), "lines;", ls[-1][:150] if ls else "")
EOF
)
  echo "[$TAG] $(date +%H:%M) $(echo "$st" | tr '\n' ' ')"
  case $st in *JOB-DONE*) break ;; *JOB-DIED*) echo "[$TAG] the job ended without its done file"; bad=1; break ;; esac
  case $st in *JOB-RUNNING*) miss=0 ;; *) miss=$((miss + 1)); [ $miss -lt 6 ] || { echo "[$TAG] session lost"; bad=1; break; } ;; esac
  [ $SECONDS -lt $end ] || { echo "[$TAG] the job did not end within $HOURS h"; bad=1; break; }
done
timeout 120 colab usage 2>&1 | grep -v '^\[colab\]' > "$OUT/usage.txt" || true
for f in paper.tar.gz results.tar.gz; do
  got=0
  for t in 1 2 3; do timeout 1800 colab download -s "$S" "/content/paper_job/$f" "$OUT/$f" > /dev/null 2>&1 && { got=1; break; }; sleep 30; done
  [ $got = 1 ] || { rclone copyto "cppwrf-drive:cppwrf-g4results/paper_$STAMP/$f" "$OUT/$f" > /dev/null 2>&1 && got=1; }  # the job's Drive copy
  [ $got = 1 ] || { echo "[$TAG] $f: not downloaded (colab nor Drive)"; bad=1; continue; }
  case $f in paper.tar.gz) tar xzf "$OUT/$f" -C "$OUT" ;; *) mkdir -p "$OUT/job" && tar xzf "$OUT/$f" -C "$OUT/job" ;; esac && rm -f "$OUT/$f"
done
grep . "$OUT/job/setup.txt" 2> /dev/null && bad=1
rate=$(sed -nE 's/.*Usage rate: *([0-9.]+).*/\1/p' "$OUT/usage.txt" | head -1)
P=$OUT/paper
if [ -f "$P/raw_runs.jsonl" ]; then
  A=("$P/raw_runs.jsonl" --verify "$P/verify.jsonl" --md "$OUT/report.md" --json "$OUT/summary.json" --csv "$OUT/runs.csv")
  for f in manifest cases best; do [ -f "$P/$f.json" ] && A+=(--$f "$P/$f.json"); done
  [ -f "$P/accuracy.md" ] && A+=(--accuracy-md "$P/accuracy.md")
  python3 "$REPO/tools/bench/bench_stats.py" "${A[@]}" ${rate:+--rate "$rate"} ${CPU_RATE:+--cpu-rate "$CPU_RATE"} ||
    { echo "[$TAG] bench_stats.py failed"; bad=1; }
  python3 "$REPO/tools/bench/bench_plot.py" "$OUT/summary.json" --out "$OUT/svg" > /dev/null ||
    { echo "[$TAG] bench_plot.py failed"; bad=1; }
  echo "[$TAG] $GPU rate ${rate:-?} compute units / hour; report: $OUT/report.md, figures: $OUT/svg"
else echo "[$TAG] no raw_runs.jsonl"; bad=1; fi
tail -40 "$P/paper.log" 2> /dev/null || true
exit $bad
