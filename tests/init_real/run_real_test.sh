#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# cppwrf_real against WRF real.exe: compare every wrfinput, wrfbdy, wrflowinp and wrffdda output.
#   usage: run_real_test.sh [BUILD_DIR] [case ...]
# With no case arguments, require the 43 runs of real_exe_data.tar under ${CPPWRF_WRF_RUNS}/real.
# A case with a '/' is its own directory. runs:NAME selects ${CPPWRF_WRF_RUNS}/NAME.
# Exit 77 if the selected dataset is absent, or fail if it is incomplete.
set -uo pipefail
shopt -s nullglob
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR=${1:-${REPO_ROOT}/build}; shift || true
EXE=${BUILD_DIR}/src/init_real/cppwrf_real
SCHEMA=${REPO_ROOT}/data/wrf_schema/wrf-4.8.0.tsv
ROOT=${CPPWRF_WRF_RUNS:-}
packaged=0
cases=("$@")
if [ ${#cases[@]} -eq 0 ]; then
  if [ -z "$ROOT" ] || [ ! -d "$ROOT/real" ]; then
    echo 'SKIP: ${CPPWRF_WRF_RUNS}/real not found'
    exit 77
  fi
  packaged=1
  cases=(run_A5_r8 run_HY1_r8 run_N{1,2}{,_r8} run_P{1,2,3,4,6,7,8,9}{,_r8}
         run_P4_12h run_P5_r8{,_1h} run_R{0,1,4,5}{,_r8} run_R{2,3} run_SU1_r8
         run_T1_r8 run_W1{,_r8,_r8_damp0} run_bdy12 run_mp6{,_r8})
fi
refs=()
missing=0
for c in "${cases[@]}"; do
  ref=
  case "$c" in
    runs:*) [ -z "$ROOT" ] || ref=$ROOT/${c#runs:} ;;
    */*) ref=$c ;;
    *) [ -z "$ROOT" ] || ref=$ROOT/real/$c ;;
  esac
  refs+=("$ref")
  if [ ! -d "$ref" ]; then
    echo "Missing reference case $c${ref:+: $ref}"
    ((missing += 1))
  fi
done
if ((missing)); then
  if (( !packaged && missing == ${#cases[@]} )); then exit 77; fi
  exit 1
fi

BASE=${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}
mkdir -p "$BASE" && WORK=$(mktemp -d "$BASE/real_test.XXXXXX") || exit 1
trap 'rm -rf "$WORK"' EXIT
fail=0
for i in "${!cases[@]}"; do
  c=${cases[$i]} ref=${refs[$i]}
  out=$WORK/$i
  mkdir "$out" || exit 1
  if ! inventory=$(PYTHONPATH="$REPO_ROOT/tools${PYTHONPATH:+:$PYTHONPATH}" python3 -c '
import sys
from wrf2cppwrf import WrfTranslator, parse_namelist
with open(sys.argv[1]) as f:
    nl = parse_namelist(f.read())
tr = WrfTranslator(nl)
ndom = int(tr.get_opt("domains", "max_dom", 1))
if ndom < 1:
    sys.exit("max_dom must be positive")
for d in range(ndom):
    tag = f"d{d + 1:02d}"
    print(f"wrfinput_{tag}")
    if sys.argv[2] == "1":
        continue
    elapsed = (tr.domain_date("end", d) - tr.domain_date("start", d)).total_seconds()
    if elapsed <= 0:
        continue
    interval = int(tr.get_opt("time_control", "interval_seconds", 0))
    if interval <= 0 or elapsed < interval:
        sys.exit(f"{tag}: the time range needs a positive interval_seconds and at least two analysis times")
    sst = tr.get_opt("physics", "sst_update", 0)
    fdda = tr.get_dom_opt("fdda", "grid_fdda", d, 0, fallback_d0=False)
    if d == 0:
        print("wrfbdy_d01")
    if sst == 1:
        print(f"wrflowinp_{tag}")
    if fdda >= 1:
        print(f"wrffdda_{tag}")
' "$ref/namelist.input" "$packaged"); then
    echo "FAIL $c: cannot derive required streams from $ref/namelist.input"
    fail=1; continue
  fi
  mapfile -t required <<< "$inventory"
  if ((packaged)); then
    required+=(wrfbdy_d01)
    if [[ $c == run_SU1_r8 ]]; then required+=(wrflowinp_d01); fi
  fi
  ok=1
  for name in "${required[@]}"; do
    if [ ! -f "$ref/$name" ]; then
      echo "FAIL $c: missing WRF reference $name"
      ok=0
    fi
  done
  if (( !ok )); then fail=1; continue; fi

  prec=double; [ ! -f "$ref/precision" ] || prec=$(cat "$ref/precision")
  if ! "$EXE" --namelist "$ref/namelist.input" --met-dir "$ref" --out-dir "$out" \
       --precision "$prec" --schema "$SCHEMA" > "$out/run.log" 2>&1; then
    echo "FAIL $c: cppwrf_real failed"; tail -3 "$out/run.log"; fail=1; continue
  fi
  for f in "$out"/{wrfinput,wrfbdy,wrflowinp,wrffdda}_d[0-9][0-9]; do
    if [ ! -f "$ref/${f##*/}" ]; then
      echo "FAIL $c: missing WRF reference ${f##*/}"
      ok=0
    fi
  done
  compared=0
  for f in "$ref"/{wrfinput,wrfbdy,wrflowinp,wrffdda}_d[0-9][0-9]; do
    python3 "${SCRIPT_DIR}/compare_real.py" "$out/${f##*/}" "$f" > "$out/cmp.log" 2>&1 \
      && ((compared += 1)) || { ok=0; cat "$out/cmp.log"; }
  done
  if ((ok && compared)); then
    echo "PASS $c ($compared files compared)"
  else
    echo "FAIL $c ($compared files compared)"
    fail=1
  fi
done
exit "$fail"
