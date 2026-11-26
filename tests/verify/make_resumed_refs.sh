#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# WRF v4.8.0 runs resumed from a restart file, for the wrfrst-restart test of run_wrf_files_test.sh:
# ${CPPWRF_WRF_RUNS}/rsm_<C> = rst_<C> (make_restart_refs.sh) restarted at minute 1, history every minute.
# CPPWRF_RESUMED_RESTART_INTERVAL sets the resumed restart cadence in minutes (default 1000).
#   usage: make_resumed_refs.sh P1_r8 ...
set -uo pipefail
: "${CPPWRF_WRF_RUNS:?set CPPWRF_WRF_RUNS}"
interval=${CPPWRF_RESUMED_RESTART_INTERVAL:-1000}
[[ "$interval" =~ ^[1-9][0-9]*$ ]] || exit 2
for c in "$@"; do
  src=${CPPWRF_WRF_RUNS}/rst_$c dst=${CPPWRF_WRF_RUNS}/rsm_$c
  [ -d "$src" ] || { echo "$c: no $src"; continue; }
  rm -rf "$dst"; mkdir -p "$dst"; cd "$dst" || exit 1
  for f in "$src"/*; do
    b=$(basename "$f")
    case $b in wrfout*|rsl*|wrfrst*|namelist.input|namelist.output|*.log) ;; *) ln -s "$(readlink -f "$f")" "$b" ;; esac
  done
  # the restart one minute after the start (the namelist's first start_* column; start_minute 0 when not set)
  nv() { sed -nE "s/^ *$1 *= *([0-9]+).*/\1/p" "$src/namelist.input" | head -1; }
  sm=$(nv start_minute); sm=$((10#${sm:-0}))
  at=$(printf '%04d-%02d-%02d_%02d:%02d:00' "$(nv start_year)" "$((10#$(nv start_month)))" "$((10#$(nv start_day)))" \
       "$((10#$(nv start_hour)))" "$((sm + 1))")
  ln -s "$src/wrfrst_d01_$at" .
  [ -f "$src/wrfrst_d02_$at" ] && ln -s "$src/wrfrst_d02_$at" .
  # the same end time: one minute less of run_seconds when set, else of run_minutes
  rs=$(sed -nE 's/^ *run_seconds *= *([0-9]+).*/\1/p' "$src/namelist.input")
  rm=$(sed -nE 's/^ *run_minutes *= *([0-9]+).*/\1/p' "$src/namelist.input")
  if [ "${rs:-0}" -gt 0 ]; then run_sed="s/^( *run_seconds *= *)[0-9]+/\\1$((rs - 60))/"
  else run_sed="s/^( *run_minutes *= *)[0-9]+/\\1$((rm - 1))/"; fi
  sed -E -e 's/^( *restart *= *)\.false\./\1.true./' -e "s/^( *start_minute *= *).*/\\1$((sm + 1)), $((sm + 1)), $((sm + 1)),/" \
         -e "$run_sed" -e "s/^( *restart_interval *= *)[0-9]+/\\1$interval/" \
         "$src/namelist.input" > namelist.input
  grep -q start_minute namelist.input || sed -i 's/^\( *start_hour *=.*\)$/\1\n start_minute = 01, 01, 01,/' namelist.input
  # MALLOC_PERTURB_=128: see make_restart_refs.sh (GFL reads an uninitialised buffer)
  MALLOC_PERTURB_=128 "$(readlink -f "$src/wrf.exe")" > wrf.log 2>&1
  echo "$c: $(grep -c 'SUCCESS COMPLETE WRF' wrf.log rsl.error.0000 2>/dev/null | tr '\n' ' ') $(ls | grep -c wrfout) history files"
done
