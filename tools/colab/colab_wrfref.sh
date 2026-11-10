#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# WRF reference runs for the suite on a Google Colab runtime with an Intel CPU (tools/gcp/gcp_wrfref.sh's work, with
# prepaid compute units), in the Debian 13 chroot of tools/colab/remote_chroot_job.sh (the GCP VMs' gcc, gfortran,
# glibc and NetCDF, which the WRF executables of the data tars were built with).
#
#   CPPWRF_GCP_BUCKET=... tools/colab/colab_wrfref.sh REFS.txt [--gpu L4|A100|T4] [--data OBJ,...] [--files NAME,...]
#       [--cases NAME,...] [--real NAME,...] [--runs DIR] [--out DIR] [--check-only] [--stage OBJ:CASE,...] [--hours H]
#       [--rev REV] [--session NAME] [--keep] [--head NAME=OBJ:BYTES[+OBJ:BYTES...]]
#
# REFS.txt: one reference per line "NAME BASE_RUN WRF_EXE section.key=value ..." as gcp_wrfref.sh's. The runtime makes
# run_NAME and run_NAME/steps (tools/gcp/wrf_ref_run.py, in parallel), rst_NAME and rsm_NAME
# (tests/verify/make_restart_refs.sh, make_resumed_refs.sh), lists the MD5 of every file WRF wrote (OUT/refs_md5.txt)
# and, with --files / --cases, runs run_wrf_files_test.sh / wrf_suite.py (exact case names) of the commit against them
# (tools/colab/wrfref_body.sh); --real: cppwrf_real of the commit against the real.exe outputs (wrfinput, wrfbdy,
# wrflowinp, wrffdda) of these references (run.real=..., tests/init_real/run_real_test.sh). --data: the tars with the base runs and WRF executables (default matrix_data.tar; from
# Google Drive, else the bucket, as colab_suite.sh). The references are downloaded and unpacked into --runs (default
# $CPPWRF_WRF_RUNS; an existing run_NAME stops the script), the links to the runtime's inputs pointing to the files of
# DIR/BASE_RUN and wrf.exe to $WRF_R8 / $WRF_R8D8 (default ~/wrf_real/WRF-4.8.0-r8[d8]/main/wrf.exe). --check-only:
# only the runs and tests on the runtime (nothing downloaded). --stage OBJ:CASE,...: the runtime packs the references
# of these wrf_suite.py cases (tools/colab/stage_data.py --whole) as the Drive object cppwrf-data/OBJ, which
# colab_suite.sh --data OBJ then reads (nothing downloaded; the bucket token is read-only, so a GCP VM cannot read it).
# --gpu, --hours (default 4), --rev, --session, --keep as colab_suite.sh. Only references whose WRF runs succeeded
# are unpacked. --head: the data tar NAME (list it in --data) made on the runtime from the leading BYTES of the bucket
# tars OBJ (ranged reads, as tools/bench/colab_fp32.sh: e.g. the inputs at the start of long72_data.tar,
# long72_data.tar:122625536 = run_P4_72h_r8's namelist, tables, wrfinput and wrfbdy). Exit status 1 when a run failed,
# an input did not arrive, the build or a test failed, or the job did not finish.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${CPPWRF_GCP_BUCKET:?set CPPWRF_GCP_BUCKET}"
BK=$CPPWRF_GCP_BUCKET
REFS=$(readlink -f "$1"); shift
GPU=L4 DATA=matrix_data.tar OUT= FILES= CASES= REALC= HEAD= FETCH=1 RUNS=${CPPWRF_WRF_RUNS:-$HOME/wrf_real480} STAGE= HOURS=4 REV=HEAD S= KEEP=0 NPROC=
WRF_R8=${WRF_R8:-$HOME/wrf_real/WRF-4.8.0-r8/main/wrf.exe} WRF_R8D8=${WRF_R8D8:-$HOME/wrf_real/WRF-4.8.0-r8d8/main/wrf.exe}
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) GPU=$2; shift 2 ;; --data) DATA=$2; shift 2 ;; --out) OUT=$2; shift 2 ;;
    --files) FILES=$2; shift 2 ;; --cases) CASES=$2; shift 2 ;; --runs) RUNS=$2; shift 2 ;; --real) REALC=$2; shift 2 ;;
    --check-only) FETCH=0; shift ;; --stage) STAGE=$2; FETCH=0; shift 2 ;; --hours) HOURS=$2; shift 2 ;;
    --rev) REV=$2; shift 2 ;; --session) S=$2; shift 2 ;; --keep) KEEP=1; shift ;; --head) HEAD=$2; shift 2 ;;
    *) sed -n '3,29p' "$0"; exit 2 ;;
  esac
done
case $GPU in L4|A100|T4) ;; *) echo "--gpu $GPU: only the Intel runtimes L4, A100, T4"; exit 2 ;; esac
names=$(grep -v '^\s*#' "$REFS" | awk 'NF {print $1}')
[ $FETCH = 0 ] || for n in $names; do for p in run rst rsm; do
  [ ! -e "$RUNS/${p}_$n" ] || { echo "[colab_wrfref] $RUNS/${p}_$n exists"; exit 1; }; done; done
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/colab/wrfref_$STAMP}
mkdir -p "$OUT"
[ "$REV" != HEAD ] || [ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" "$REV"
echo "[colab_wrfref] commit $(git -C "$REPO" rev-parse --short "$REV"), $GPU, $(wc -w <<< "$names") references, data $DATA"
TAG=colab_wrfref
. "$REPO/tools/colab/colab_lib.sh"
cj_start "$GPU"
cj_send "$REFS" in/refs.txt
if [ -n "$HEAD" ]; then  # the --head tar, cached in /content/tars as a fetched data tar
  cj_token || { echo "[colab_wrfref] token upload failed"; exit 1; }
  hname=${HEAD%%=*} hparts=${HEAD#*=}
  rx 1800 <<EOF | grep HEAD_OK || { echo "[colab_wrfref] --head $hname: not made"; exit 1; }
import os, subprocess
T = "/content/tars"; os.makedirs(T, exist_ok=True)
with open(T + "/$hname", "wb") as o:
    for part in "$hparts".split("+"):
        obj, end = part.split(":")
        r = subprocess.run(["gcloud", "storage", "--access-token-file=/content/cpu/gcs.tok", "cat", "-r", f"0-{int(end) - 1}",
                            "gs://$BK/" + obj], stdout=o)
        assert r.returncode == 0 and o.tell() % 512 == 0, obj
    o.write(b"\0" * 10240)
n = subprocess.run(["tar", "tf", T + "/$hname"], capture_output=True, text=True).stdout.split()
open(T + "/$hname.ok", "w").close()
print("HEAD_OK", os.path.getsize(T + "/$hname"), len(n), " ".join(x for x in n if "wrfbdy" in x or "namelist" in x))
EOF
fi
cj_launch "$REPO/tools/colab/wrfref_body.sh" "$DATA" "$FILES" "$CASES" "$STAGE" "$FETCH" "$REALC"
cj_wait "$HOURS" refs.out
cj_results
bad=0
grep . "$OUT/setup.out" 2>/dev/null && bad=1
# the references whose WRF runs succeeded (as gcp_wrfref.sh: wrf_ref_run.py's SUCCESS lines; make_restart_refs.sh /
# make_resumed_refs.sh: "NAME: <count of SUCCESS COMPLETE WRF> ... restart / history files")
good=
for n in $names; do
  steps=1; grep -E "^\s*$n\s.*\srun\.steps=0(\s|$)" "$REFS" > /dev/null && steps=0  # no 3-step and restart runs
  if grep -qx "$n SUCCESS" "$OUT/refs.out" 2>/dev/null && { [ $steps = 0 ] || grep -qx "$n/steps SUCCESS" "$OUT/refs.out"; }
  then good="$good run_$n"
  else echo "[colab_wrfref] run_$n: $(grep -E "^$n(/steps)? (WRF|REAL)-FAILED" "$OUT/refs.out" 2>/dev/null | cut -c1-300 || echo "no result")"; bad=1; fi
  [ $steps = 1 ] || continue
  for p in rst:restart rsm:history; do
    if grep -qE "^$n: (wrf\.log:)?[1-9][0-9]* .* ${p#*:} files$" "$OUT/refs.out" 2>/dev/null; then good="$good ${p%:*}_$n"
    else echo "[colab_wrfref] ${p%:*}_$n: WRF failed or did not run"; bad=1; fi
  done
done
if [ $FETCH = 1 ] && [ -n "$good" ]; then
  t0=$SECONDS
  timeout 7200 colab download -s "$S" /content/cpu/fetch/refs.tar "$OUT/refs.tar" > /dev/null 2>&1 || { echo "[colab_wrfref] refs.tar: download failed"; exit 1; }
  echo "[colab_wrfref] downloaded refs.tar ($(( $(stat -c %s "$OUT/refs.tar") / 1000000 )) MB) in $((SECONDS - t0)) s"
  stage="$RUNS/.wrfref-$STAMP"; mkdir "$stage"
  tar xf "$OUT/refs.tar" -C "$stage" || bad=1
  rm -f "$OUT/refs.tar"
  for d in $good; do [ -d "$stage/$d" ] && mv "$stage/$d" "$RUNS/"; done
  rm -rf "${stage:?}"
  # the runtime's links -> the same local files (base run inputs, the other reference directories, wrf.exe)
  for n in $names; do
    for d in "$RUNS/run_$n" "$RUNS/rst_$n" "$RUNS/rsm_$n"; do
      case " $good " in *" $(basename "$d") "*) [ -d "$d" ] || continue ;; *) continue ;; esac
      find "$d" -type l | while read -r l; do
        t=$(readlink "$l")
        case $t in
          /opt/job/data/runs/*) nt=$(readlink -f "$RUNS/${t#/opt/job/data/runs/}" || echo "$RUNS/${t#/opt/job/data/runs/}") ;;
          /opt/job/wrf_runs/*) nt=$RUNS/${t#/opt/job/wrf_runs/} ;;
          */wrf_r8d8.exe) nt=$WRF_R8D8 ;;
          */wrf_r8.exe) nt=$WRF_R8 ;;
          *) echo "[colab_wrfref] link not mapped: $l -> $t"; continue ;;
        esac
        [ -e "$nt" ] || echo "[colab_wrfref] missing locally: $l -> $nt"
        ln -sfn "$nt" "$l"
      done
    done
  done
  echo "[colab_wrfref] references in $RUNS: $(for n in $names; do ls -d "$RUNS"/{run,rst,rsm}_"$n" 2>/dev/null; done | tr '\n' ' ')"
fi
echo "[colab_wrfref] results in $OUT"; cat "$OUT/refs.out" 2>/dev/null || true
if [ -n "$STAGE" ]; then
  echo "== stage"; tail -3 "$OUT/stage.log" 2>/dev/null || echo "no stage.log"
  grep -q "^DRIVE ${STAGE%%:*} " "$OUT/data.txt" 2>/dev/null || { echo "[colab_wrfref] $STAGE: not staged"; bad=1; }
fi
for f in files.log suite.log real.log; do
  [ -f "$OUT/$f" ] || continue
  echo "== $f"; grep -E "^(PASS|FAIL|SKIP)|compared" "$OUT/$f" || tail -5 "$OUT/$f"
  grep -qx "exit 0" "$OUT/$f" || { echo "[colab_wrfref] $f: $(tail -1 "$OUT/$f")"; bad=1; }
done
[ -z "$FILES" ] || [ -f "$OUT/files.log" ] || { echo "[colab_wrfref] no files.log"; bad=1; }
[ -z "$CASES" ] || [ -f "$OUT/suite.log" ] || { echo "[colab_wrfref] no suite.log"; bad=1; }
[ -z "$REALC" ] || [ -f "$OUT/real.log" ] || { echo "[colab_wrfref] no real.log"; bad=1; }
exit $bad
