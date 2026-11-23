#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# cppwrf_real against WRF real.exe: every reference case in ${CPPWRF_WRF_RUNS}/real
# (tools/init_real/make_real_refs.sh) must give identical wrfinput_d0N, wrfbdy_d01 and (sst_update = 1)
# wrflowinp_d0N and (grid_fdda >= 1) wrffdda_d0N files.
#   usage: run_real_test.sh [BUILD_DIR] [case ...]
# A case with a '/' is a directory of its own (a WRF reference run made with real.exe: namelist.input, the met_em
# files and real.exe's outputs; precision double without a precision file); runs:NAME is the directory
# ${CPPWRF_WRF_RUNS}/NAME (a wrfref run made with run.real; a missing one is a SKIP, exit 77 when every case is).
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
if [ -z "${CPPWRF_WRF_RUNS:-}" ] || { [ ! -d "${CPPWRF_WRF_RUNS}/real" ] && [[ "${2:-}" != */* ]] && [[ "${2:-}" != runs:* ]]; }; then
  echo "SKIP: \${CPPWRF_WRF_RUNS}/real not found"; exit 77
fi
BUILD_DIR=${1:-${REPO_ROOT}/build}; shift || true
EXE=${BUILD_DIR}/src/init_real/cppwrf_real
SCHEMA=${REPO_ROOT}/data/wrf_schema/wrf-4.8.0.tsv
# a directory of its own per invocation: ctest runs real_vs_wrf and real_vs_wrf_runs at the same time
BASE=${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}
mkdir -p "$BASE" && WORK=$(mktemp -d "$BASE/real_test.XXXXXX") || exit 1
cases=("$@")
[ ${#cases[@]} -eq 0 ] && for d in "${CPPWRF_WRF_RUNS}"/real/*/; do cases+=("$(basename "$d")"); done
fail=0 skip=0
for c in "${cases[@]}"; do
  ref=${CPPWRF_WRF_RUNS}/real/$c
  [[ "$c" == */* ]] && ref=$c
  [[ "$c" == runs:* ]] && ref=${CPPWRF_WRF_RUNS}/${c#runs:}
  [ -d "$ref" ] || { echo "SKIP $c: $ref not found"; skip=$((skip + 1)); continue; }
  prec=double; [ -f "$ref/precision" ] && prec=$(cat "$ref/precision")
  rm -rf "$WORK"; mkdir -p "$WORK"
  if ! "$EXE" --namelist "$ref/namelist.input" --met-dir "$ref" --out-dir "$WORK" \
       --precision "$prec" --schema "$SCHEMA" > "$WORK/run.log" 2>&1; then
    echo "FAIL $c: cppwrf_real failed"; tail -3 "$WORK/run.log"; fail=1; continue
  fi
  ok=1
  for f in "$ref"/wrfinput_d0? "$ref"/wrfbdy_d01 "$ref"/wrflowinp_d0? "$ref"/wrffdda_d0?; do
    [ -f "$f" ] || continue
    python3 "${SCRIPT_DIR}/compare_real.py" "$WORK/$(basename "$f")" "$f" > "$WORK/cmp.log" 2>&1 \
      || { ok=0; cat "$WORK/cmp.log"; }
  done
  if [ $ok = 1 ]; then echo "PASS $c"; else echo "FAIL $c"; fail=1; fi
done
rm -rf "$WORK"
[ "$fail" = 0 ] && [ "$skip" -gt 0 ] && [ "$skip" = "${#cases[@]}" ] && exit 77
exit $fail
