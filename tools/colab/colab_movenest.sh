#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# WRF v4.8.0 with moving nests and its references, on a Google Colab runtime with an Intel CPU, in the Debian 13 chroot
# of tools/colab/remote_chroot_job.sh (the GCP VMs' gcc, gfortran, glibc and NetCDF, which built the suite's
# wrf_r8.exe). The job (tools/colab/movenest_body.sh) builds WRF from the release source (checked against the
# workstation's WRF source tree) with configure option 34 (dmpar GNU, MPICH; a serial build does not move nests
# correctly) and -r8, nesting 1 (basic) and 2 (preset moves, -DMOVE_NESTS), each run with one MPI rank, checks that both
# reproduce the suite's serial wrf_r8.exe on N3 (every variable, 10 min; the moving build's NEST_POS must differ), then
# runs the references of REFS.txt with the moving build (and the suite cases of --cases with CPPWRF's CPU FP64 build of
# the commit against them). No restart references: CPPWRF does not resume a run with moving nests.
#
#   tools/colab/colab_movenest.sh [REFS.txt] [--gpu L4|A100|T4] [--cases NAME,...] [--stage OBJ:CASE,...] [--vortex]
#       [--no-fp32] [--runs DIR] [--out DIR] [--hours H] [--rev REV] [--session NAME] [--keep] [--dry-run]
#
# REFS.txt: default tests/verify/refs/movenest_refs.txt (format of colab_wrfref.sh; executables wrf_r8_move.exe,
# wrf_r4_move.exe, wrf_r8_vortex.exe made by the job). Data: matrix_nest.tar (run_N3s_r8, wrf_r8.exe) and, unless
# --no-fp32 (then real*4 is not built and the *_cpu_sp references are left out), fp32id4_data.tar (run_N3_cpu_sp).
# --vortex: also the vortex-following build (nesting 3), not used by a reference. --stage OBJ:CASE,...: the
# runtime packs the references of these wrf_suite.py cases as cppwrf-data/OBJ on Drive (only after the reproduction
# check passed). The references come back (refs.tar) into --runs (default $CPPWRF_SCRATCH/wrf_runs_movenest; a
# directory under ~/wrf_* is refused; an existing run_NAME stops the script); their input links point to the base runs
# in $CPPWRF_WRF_RUNS (default ~/wrf_real480, only linked to), their wrf.exe links to RUNS/wrf_move/<exe> (the job's
# executables are not downloaded).
# Results in --out: wrf_build.txt (source check, configure.wrf lines, executables), repro.log (REPRO OK / FAILED),
# moves.log (the moves of each WRF run), refs.out, suite.log. Exit 1 when a build, the reproduction check, a run or a
# case failed. --dry-run: the checks and the job's arguments only.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
usage() { sed -n '3,26p' "$0"; }
REFS=$REPO/tests/verify/refs/movenest_refs.txt
case ${1:-} in -h|--help) usage; exit 0 ;; -*|'') ;; *) REFS=$(readlink -f "$1"); shift ;; esac
GPU=L4 OUT= CASES= STAGE= VORTEX=0 FP32=1 HOURS=4 REV=HEAD S= KEEP=0 DRY=0 RUNS=${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/wrf_runs_movenest
WRF_SRC=${WRF_SRC:-$HOME/wrf_real/WRF-4.8.0-src}
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) GPU=$2; shift 2 ;; --cases) CASES=$2; shift 2 ;; --stage) STAGE=$2; shift 2 ;; --vortex) VORTEX=1; shift ;;
    --no-fp32) FP32=0; shift ;; --runs) RUNS=$2; shift 2 ;; --out) OUT=$2; shift 2 ;; --hours) HOURS=$2; shift 2 ;;
    --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;; --keep) KEEP=1; shift ;; --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;; *) echo "[colab_movenest] unknown option $1"; usage; exit 2 ;;
  esac
done
case $GPU in L4|A100|T4) ;; *) echo "--gpu $GPU: only the Intel runtimes L4, A100, T4"; exit 2 ;; esac
case $STAGE in ''|?*:?*) ;; *) echo "[colab_movenest] --stage $STAGE: OBJ:CASE,..."; exit 2 ;; esac
[ -f "$REFS" ] || { echo "[colab_movenest] $REFS: no such file"; exit 2; }
RUNS_GIVEN=$(realpath -sm "$RUNS"); RUNS=$(readlink -m "$RUNS")
for w in "$HOME"/wrf_*; do  # the WRF trees and runs under ~/wrf_* (also through their links) stay read-only
  [ -e "$w" ] || continue
  for r in "$RUNS_GIVEN" "$RUNS"; do
    case $r/ in "$w"/*|"$(readlink -m "$w")"/*) echo "[colab_movenest] --runs $RUNS_GIVEN: under $w, which stays read-only"; exit 2 ;; esac
  done
done
BASE_RUNS=${CPPWRF_WRF_RUNS:-$HOME/wrf_real480}  # the suite's base runs (run_N3s_r8, ...): the references link to them
[ -d "$WRF_SRC/Registry" ] || { echo "[colab_movenest] WRF_SRC=$WRF_SRC: no WRF source tree"; exit 2; }
DATA=matrix_nest.tar; [ $FP32 = 1 ] && DATA=$DATA,fp32id4_data.tar
TMPREFS=$(mktemp "${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/movenest_refs.XXXXXX")
trap 'rm -f "$TMPREFS"' EXIT
grep -v '^\s*#' "$REFS" | awk -v fp32=$FP32 'NF && (fp32 == 1 || $3 != "wrf_r4_move.exe")' > "$TMPREFS"
bad=$(awk -v vortex=$VORTEX '{ if (NF < 3) print "line " NR ": fewer than 3 fields"
             if (seen[$1]++) print $1 ": defined twice"
             if ($3 !~ /^wrf_(r8_move|r4_move|r8_vortex)\.exe$/) print $1 ": " $3 " (wrf_r8_move.exe, wrf_r4_move.exe, wrf_r8_vortex.exe)"
             if ($3 == "wrf_r8_vortex.exe" && !vortex) print $1 ": wrf_r8_vortex.exe needs --vortex"
             for (i = 4; i <= NF; i++) if ($i !~ /^[a-z_0-9]+\.[A-Za-z_0-9]+=[^ ]+$/) print $1 ": " $i " is no section.key=value" }' "$TMPREFS")
[ -z "$bad" ] || { echo "[colab_movenest] $REFS:"; echo "$bad"; exit 2; }
names=$(awk '{print $1}' "$TMPREFS")
if [ $DRY = 1 ]; then
  echo "[colab_movenest] dry run: $(wc -w <<< "$names") references ($(echo $names)), $GPU, data $DATA, vortex $VORTEX, fp32 $FP32"
  echo "[colab_movenest] job: movenest_body.sh /opt/job '$CASES' '$STAGE' 1 $VORTEX $FP32"
  echo "[colab_movenest] WRF source manifest: $(find "$WRF_SRC" -type f -not -path '*/.git/*' | wc -l) files of $WRF_SRC"
  exit 0
fi
for n in $names; do
  [ ! -e "$RUNS/run_$n" ] || { echo "[colab_movenest] $RUNS/run_$n exists"; exit 1; }; done
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/movenest_$STAMP}
mkdir -p "$OUT"
cp "$TMPREFS" "$OUT/refs.txt"
rm -f "$TMPREFS"; trap - EXIT
( cd "$WRF_SRC" && find . -type f -not -path './.git/*' | sort | xargs md5sum ) > "$OUT/wrf_src_md5.txt"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" "$REV"
echo "[colab_movenest] commit $(git -C "$REPO" rev-parse --short "$REV"), $GPU, references $(echo $names), data $DATA"
BK= TAG=colab_movenest DRIVE_RESULTS=cppwrf-g4results/movenest_$STAMP
. "$REPO/tools/colab/colab_lib.sh"
cj_start "$GPU"
cj_send "$OUT/refs.txt" in/refs.txt
cj_send "$OUT/wrf_src_md5.txt" in/wrf_src_md5.txt
cj_launch "$REPO/tools/colab/movenest_body.sh" "$DATA" "$CASES" "$STAGE" 1 "$VORTEX" "$FP32"
cj_wait "$HOURS" refs.out || { cj_salvage || true; echo "[colab_movenest] the job did not finish; logs in $OUT/partial"; exit 1; }
bad=0
cj_results || bad=1
grep . "$OUT/setup.out" 2>/dev/null && bad=1
echo "== WRF builds"; grep -E '^(WRF source|SRC_|BUILT|==|-D|WRF builds)' "$OUT/wrf_build.txt" 2>/dev/null || { echo "no wrf_build.txt"; bad=1; }
echo "== reproduction"; grep REPRO "$OUT/repro.log" 2>/dev/null || echo "no repro.log"
grep -qx "REPRO OK" "$OUT/repro.log" 2>/dev/null || bad=1
echo "== moves"; cat "$OUT/moves.log" 2>/dev/null || true
good=
for n in $names; do
  if grep -qx "$n SUCCESS" "$OUT/refs.out" 2>/dev/null && grep -qx "$n/steps SUCCESS" "$OUT/refs.out"; then good="$good run_$n"
  else echo "[colab_movenest] run_$n: $(grep -E "^$n(/steps)? WRF-FAILED" "$OUT/refs.out" 2>/dev/null | cut -c1-300 || echo "no result")"; bad=1; fi
done
if [ -n "$good" ]; then
  cj_fetch refs.tar "$OUT/refs.tar" || { echo "[colab_movenest] refs.tar: download failed"; exit 1; }
  mkdir -p "$RUNS"
  stage="$RUNS/.movenest-$STAMP"; mkdir -p "$stage"
  tar xf "$OUT/refs.tar" -C "$stage" || { rm -rf "${stage:?}"; echo "[colab_movenest] $OUT/refs.tar does not unpack"; exit 1; }
  rm -f "$OUT/refs.tar"
  for d in $good; do [ -d "$stage/$d" ] && mv "$stage/$d" "$RUNS/"; done
  rm -rf "${stage:?}"
  for d in $good; do
    find "$RUNS/$d" -type l | while read -r l; do
      t=$(readlink "$l")
      case $t in
        /opt/job/data/runs/*) nt=$BASE_RUNS/${t#/opt/job/data/runs/} ;;
        /opt/job/wrf_runs/*) nt=$RUNS/${t#/opt/job/wrf_runs/} ;;
        */wrf_*.exe) nt=$RUNS/wrf_move/$(basename "$t") ;;
        *) echo "[colab_movenest] link not mapped: $l -> $t"; continue ;;
      esac
      ln -sfn "$nt" "$l"
    done
  done
  echo "[colab_movenest] references in $RUNS:$good"
fi
cat "$OUT/refs.out" 2>/dev/null | grep -E 'SUCCESS|FAILED|files$' || true
if [ -n "$STAGE" ]; then
  o=${STAGE%%:*}
  grep -qx "STAGED $o" "$OUT/stage.log" 2>/dev/null && grep -q "^DRIVE $o " "$OUT/data.txt" 2>/dev/null ||
    { echo "[colab_movenest] $o: not staged ($(grep -m1 STAGE_SKIPPED "$OUT/stage.log" 2>/dev/null || echo "see stage.log, setup.out"))"; bad=1; }
fi
if [ -n "$CASES" ]; then
  echo "== suite.log"; grep -E "^(PASS|FAIL|SKIP)" "$OUT/suite.log" 2>/dev/null || tail -5 "$OUT/suite.log" 2>/dev/null || true
  grep -qx "exit 0" "$OUT/suite.log" 2>/dev/null || { echo "[colab_movenest] suite.log: $(tail -1 "$OUT/suite.log" 2>/dev/null)"; bad=1; }
fi
echo "[colab_movenest] results in $OUT"
exit $bad
