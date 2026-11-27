#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# CPPWRF wrfout_d0N and wrfrst_d0N files against WRF v4.8.0's (CPU FP64 build, WRF real*8 runs):
# every variable, value (tolerance 0), dimension, attribute and global attribute must be identical.
#   usage: run_wrf_files_test.sh [BUILD_DIR] [case ...]
# CPPWRF_FILES_TRANSLATED=1: the CPPWRF configuration is made from the WRF run's namelist.input by
# tools/wrf2cppwrf.py instead of configs/ (the user's path from a WRF run directory).
# CPPWRF_FILES_NO_VALUES=1 (a CUDA build): the files' variables, dimensions, attributes, global attributes and times
# are compared, not the numbers (the GPU differs from WRF at roundoff; wrf_suite.py's cuda cases bound the values).
# References: ${CPPWRF_WRF_RUNS}/run_<C>_r8[/steps] (history every step) and
# ${CPPWRF_WRF_RUNS}/rst_<C>_r8 (tests/verify/make_restart_refs.sh: restart every minute) and
# ${CPPWRF_WRF_RUNS}/rsm_<C>_r8 (make_resumed_refs.sh: WRF resumed from rst_<C>_r8 at minute 1; CPPWRF
# resumes from the same wrfrst file).
# RQCBLTEN / RQIBLTEN are not compared when WRF's file has no QCLOUD / QICE: without qc / qi WRF leaves them
# uninitialised. Neither are QZ0 and USTM of f1 (fractional sea ice with sfclayrev): WRF averages them with
# uninitialised open-water locals.
# MYNN's first step depends on a restart alarm ringing at its end (solve_em's restart_flag), so its history
# with restart_interval 1 differs from the plain run: mynn (no restart files) and mynnr (rst_/rsm_ only).
# CPPWRF_ALLOW_SKIP=1 permits missing base runs if another case compares files. An available run must supply
# every registered leg and each invocation must compare at least one file.
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
if [ -z "${CPPWRF_WRF_RUNS:-}" ] || [ ! -d "${CPPWRF_WRF_RUNS}" ]; then
  echo "SKIP: \${CPPWRF_WRF_RUNS} not set"; exit 77
fi
BUILD_DIR=${1:-${REPO_ROOT}/build}; shift || true
EXE=${BUILD_DIR}/cppwrf
CMP="python3 ${REPO_ROOT}/tests/init_real/compare_real.py$([ "${CPPWRF_FILES_NO_VALUES:-0}" = 1 ] && echo " --no-values") --ignore"
# the variables not compared in WRF's file $1 (case $2): RQCBLTEN / RQIBLTEN without QCLOUD / QICE, LH with isfflx 0
# (wrf_suite.uninitialised), f1's QZ0 / USTM
ignored() {
  python3 -c 'import sys, netCDF4
d = netCDF4.Dataset(sys.argv[1]); v = d.variables
print(",".join([x for x, q in (("RQCBLTEN", "QCLOUD"), ("RQIBLTEN", "QICE")) if q not in v]
               + (["LH"] if "ISFFLX" in d.ncattrs() and int(d.getncattr("ISFFLX")) == 0 else [])
               + (["QZ0", "USTM"] if sys.argv[2] == "f1" else []) or ["-"]))' "$1" "$2"
}
# start time from the first namelist.input in the given directories: minute-1 restart stamp or last wrfrst offset [s]
rst_info() {
  python3 -c 'import datetime, glob, os, re, sys
mode, dirs = sys.argv[1], [d for d in sys.argv[2:] if d and d != "-" and os.path.isdir(d)]
t0 = None
for d in dirs:
    p = os.path.join(d, "namelist.input")
    if not os.path.isfile(p):
        continue
    nl = open(p).read()
    vals = []
    for k in ("year", "month", "day", "hour", "minute", "second"):
        m = re.search(rf"^\s*start_{k}\s*=\s*(\d+)", nl, re.M | re.I)
        if m:
            vals.append(int(m.group(1)))
        elif k in ("minute", "second"):
            vals.append(0)
        else:
            vals = None
            break
    if vals:
        t0 = datetime.datetime(*vals)
        break
if mode == "at":
    print((t0 + datetime.timedelta(minutes=1)).strftime("%Y-%m-%d_%H:%M:%S") if t0 else "")
else:
    best = 0
    for d in dirs:
        for f in glob.glob(os.path.join(d, "wrfrst_d01_*")):
            m = re.search(r"(\d{4}-\d{2}-\d{2}_\d{2}:\d{2}:\d{2})$", os.path.basename(f))
            if not m:
                continue
            tf = datetime.datetime.strptime(m.group(1), "%Y-%m-%d_%H:%M:%S")
            secs = int((tf - t0).total_seconds()) if t0 else tf.hour * 3600 + tf.minute * 60 + tf.second
            if secs > best:
                best = secs
    print(best)' "$@"
}
WORK=${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/wrf_files_test
R=${CPPWRF_WRF_RUNS}
cd "$REPO_ROOT"

has_file() {
  for path in "$@"; do
    if [ -f "$path" ]; then return 0; fi
  done
  return 1
}
# the domains (d01 d02 ...) of the KIND_dNN_* files CPPWRF wrote under $1 (its nests in dNN/), not in resumed/
domains_written() {
  find "$1" -path "$1/resumed" -prune -o -type f -name "$2_d0[0-9]_*" -print |
    sed -E "s#.*/$2_(d0[0-9])_.*#\1#" | sort -u
}
# every domain CPPWRF wrote KIND files for ($2 under $1) needs WRF's of that domain in one of the dirs $4...: a
# reference pack without a nest's files would otherwise compare d01 only ($3: the leg's label)
check_domains() {
  local out=$1 kind=$2 label=$3 dom dir found missing=0
  shift 3
  for dom in $(domains_written "$out" "$kind"); do
    found=0
    for dir in "$@"; do
      [ "$dir" != "-" ] && has_file "$dir/${kind}_${dom}_"* && found=1
    done
    if [ $found = 0 ]; then echo "FAIL $name: $label: CPPWRF wrote ${kind}_${dom}, no WRF ${kind}_${dom} in $*"; missing=1; fi
  done
  return $missing
}

# name | config | wrfout reference dir | restart reference dir (or -) | extra --set
CASES="
p1|configs/real_p1_r8.ini|$R/run_P1_r8/steps|$R/rst_P1_r8|
p2|configs/real_p2_r8.ini|$R/run_P2_r8/steps|-|
p3|configs/real_p3_r8.ini|$R/run_P3_r8/steps|$R/rst_P3_r8|
p4|configs/real_p4_r8.ini|$R/run_P4_r8/steps|$R/rst_P4_r8|
p5|configs/real_p5_r8.ini|$R/run_P5_r8|-|
p6|configs/real_p6_r8.ini|$R/run_P6_r8/steps|$R/rst_P6_r8|
p7|configs/real_p7_r8.ini|$R/run_P7_r8/steps|$R/rst_P7_r8|
p8|configs/real_p8_r8.ini|$R/run_P8_r8/steps|$R/rst_P8_r8|--set run.run_time=600
p9|configs/real_p9_r8.ini|$R/run_P9_r8/steps|$R/rst_P9_r8|
mp8|configs/real_mp8_r8.ini|$R/run_mp8_r8/steps|$R/rst_mp8_r8|
mynn|configs/real_mynn_r8.ini|$R/run_MYNN_r8/steps|-|
mynnr|configs/real_mynn_r8.ini|$R/rst_MYNN_r8|$R/rst_MYNN_r8|
r1|configs/real_r1.ini|$R/run_R1_r8|-|
t1|configs/real_t1.ini|$R/run_T1_r8|-|--set output.wrf_namelist=$R/run_T1_r8/namelist.input
n1|configs/real_n1_steps.ini|$R/run_N1_r8/steps|-|
n2|configs/real_n2_steps.ini|$R/run_N2_r8/steps|$R/rst_N2_r8|
n2s1|configs/real_n2_steps.ini|$R/run_N2S1_r8/steps|-|--set real.dir=$R/run_N2S1_r8/steps --set domains.smooth_option=1
n2s2|configs/real_n2_steps.ini|$R/run_N2S2_r8/steps|-|--set real.dir=$R/run_N2S2_r8/steps --set domains.smooth_option=2
n3|configs/real_n3_steps.ini|$R/run_N3_r8/steps|$R/rst_N3_r8|
nmp1|configs/real_nmp1_r8.ini|$R/run_NMP1_r8/steps|$R/rst_NMP1_r8|
v1|configs/real_v1_r8.ini|$R/run_V1_r8/steps|-|--set run.run_time=180
v2|configs/real_v2_r8.ini|$R/run_V2_r8/steps|-|--set run.run_time=180
v3|configs/real_v3_r8.ini|$R/run_V3_r8/steps|-|--set run.run_time=54
t6|configs/real_t6_r8.ini|$R/run_T6_r8/steps|$R/rst_T6_r8|
g2|configs/real_g2_r8.ini|$R/run_G2_r8/steps|$R/rst_G2_r8|
f1|configs/real_f1_r8.ini|$R/run_F1_r8/steps|$R/rst_F1_r8|
f2|configs/real_f2_r8.ini|$R/run_F2_r8/steps|$R/rst_F2_r8|
d0|configs/real_d0_r8.ini|$R/run_D0_r8/steps|$R/rst_D0_r8|
d0v|configs/real_d0v_r8.ini|$R/run_D0V_r8/steps|$R/rst_D0V_r8|
su1|configs/real_su1_r8.ini|$R/rst_SU1_r8|$R/rst_SU1_r8|--set run.run_time=180
hy1|configs/real_p1_r8.ini|$R/run_HY1_r8/steps|-|--set real.dir=$R/run_HY1_r8/steps --set dyn.hypsometric_opt=1
fd1|configs/real_fd1_r8.ini|$R/run_FD1_r8/steps|$R/rst_FD1_r8|
fdn|configs/real_fdn_r8.ini|$R/run_FDN_r8/steps|$R/rst_FDN_r8|
ob1|configs/real_ob1_r8.ini|$R/run_OB1_r8/steps|$R/rst_OB1_r8|
g10sso|configs/real_p1_r8.ini|$R/rst_G10SSO|$R/rst_G10SSO|--set real.dir=$R/run_G10SSO_r8 --set phys.topo_wind=1
g9f1|configs/real_n3_steps.ini|$R/run_G9F1_r8/steps|$R/rst_G9F1|--set real.dir=$R/run_G9F1_r8/steps --set d02.phys_cu_physics=0
g9f5|configs/real_b01_r8.ini|$R/run_G9F5_r8|-|--set real.dir=$R/run_G9F5_r8 --set run.run_time=300 --set time.history_interval_s=3600 --set output.out_interval=1000000000 --set restart.interval=120
g9f6a5|configs/real_p1_r8.ini|$R/run_G9F6A5_r8/steps|-|--set real.dir=$R/run_G9F6A5_r8/steps
g9f6b|configs/real_mynn_r8.ini|$R/run_G9F6B_r8/steps|-|--set real.dir=$R/run_G9F6B_r8/steps
g9f6c|configs/real_n3_steps.ini|$R/run_G9F6C_r8/steps|-|--set real.dir=$R/run_G9F6C_r8/steps --set phys.cu_physics=11 --set d02.phys_cu_physics=0
g9f6d|configs/real_p1_r8.ini|$R/run_G9F6D_r8/steps|$R/rst_G9F6D|--set real.dir=$R/run_G9F6D_r8/steps
g9f6e|configs/real_n3_steps.ini|$R/run_G9F6E_r8/steps|-|--set real.dir=$R/run_G9F6E_r8/steps
g9f7|configs/real_p1_r8.ini|$R/run_G9F7_r8/steps|$R/rst_G9F7|--set real.dir=$R/run_G9F7_r8/steps
"
want=" $* "
fail=0 n=0 skip=0
for requested in "$@"; do
  if ! cut -d'|' -f1 <<< "$CASES" | grep -Fxq -- "$requested" || [ -z "$requested" ]; then
    echo "FAIL unknown file case $requested"; fail=1
  fi
done
while IFS='|' read -r name cfg ref rst extra; do
  [ -z "$name" ] && continue
  [ $# -gt 0 ] && [[ "$want" != *" $name "* ]] && continue
  [ -d "$ref" ] || { echo "SKIP $name: $ref not found"; skip=$((skip + 1)); continue; }
  rsm=${rst/\/rst_/\/rsm_}
  if [ "$rst" != "-" ] && { [ ! -d "$rst" ] || [ ! -d "$rsm" ]; }; then
    echo "FAIL $name: required restart or resumed leg missing ($rst, $rsm)"; fail=1; continue
  fi
  if ! has_file "$ref"/wrfout_d0* ||
     { [ "$rst" != "-" ] && ! has_file "$rst"/wrfrst_d0*; }; then
    echo "FAIL $name: missing straight history or restart reference files"; fail=1; continue
  fi
  sets=(--set output.format=wrfout --set output.out_interval=1 --set output.dir="$WORK/$name")
  if { [ "$rst" != "-" ] && [ -d "$rst" ]; } || compgen -G "$ref/wrfrst_d0*" > /dev/null; then
    sets+=(--set restart.interval=60)
    # restart files only up to WRF's last one (elapsed seconds from the namelist start; ~0.5 GB each for large domains)
    last=$(rst_info last "$rst" "$ref")
    # (not with the adaptive time step: the model refuses restart.end there)
    if [ "$last" -gt 0 ] && ! grep -qE '^[[:space:]]*adaptive[[:space:]]*=[[:space:]]*(true|1)' "$REPO_ROOT/$cfg"; then
      sets+=(--set restart.end="$last")
    fi
  fi
  # shellcheck disable=SC2206
  [ -n "$extra" ] && sets+=($extra)
  rm -rf "${WORK:?}/$name"; mkdir -p "$WORK/$name"
  if [ "${CPPWRF_FILES_TRANSLATED:-0}" = 1 ]; then
    if ! python3 "$REPO_ROOT/tools/wrf2cppwrf.py" "$ref/namelist.input" -o "$WORK/$name/translated.ini" \
         --format wrfout > "$WORK/$name/translate.log" 2>&1; then
      echo "FAIL $name: wrf2cppwrf failed"; tail -3 "$WORK/$name/translate.log"; fail=1; continue
    fi
    cfg=$WORK/$name/translated.ini
  fi
  if ! "$EXE" "$cfg" "${sets[@]}" > "$WORK/$name/run.log" 2>&1; then
    echo "FAIL $name: cppwrf failed"; tail -3 "$WORK/$name/run.log"; fail=1; continue
  fi
  ok=1 before=$n
  check_domains "$WORK/$name" wrfout "straight history" "$ref" "$rst" || ok=0
  if [ "$rst" != "-" ] || compgen -G "$ref/wrfrst_d0*" > /dev/null; then
    check_domains "$WORK/$name" wrfrst "restart files" "$rst" "$ref" || ok=0
  fi
  for dir in "$ref" "$rst"; do
    [ "$dir" = "-" ] || [ ! -d "$dir" ] && continue
    for f in "$dir"/wrfout_d0* "$dir"/wrfrst_d0*; do
      [ -f "$f" ] || continue
      b=$(basename "$f"); dom=${b:7:3}
      ours=$WORK/$name/$b; [ -f "$ours" ] || ours=$WORK/$name/$dom/$b
      if [ ! -f "$ours" ]; then echo "FAIL $name: $b not written"; ok=0; continue; fi
      $CMP "$(ignored "$f" "$name")" "$ours" "$f" > "$WORK/$name/cmp.log" 2>&1 || { ok=0; echo "FAIL $name $b"; head -20 "$WORK/$name/cmp.log"; }
      n=$((n + 1))
    done
  done
  # resumed from WRF's restart file at minute 1 against WRF's resumed run (make_resumed_refs.sh)
  if [ "$rst" != "-" ]; then
    before_resume=$n
    rdir=$WORK/$name/resumed; mkdir -p "$rdir"
    rsets=("${sets[@]}" --set output.dir="$rdir")
    at=$(rst_info at "$rst" "$ref")
    if [ -z "$at" ] || [ ! -f "$rst/wrfrst_d01_$at" ]; then
      echo "FAIL $name: restart file wrfrst_d01_${at:-?} not found in $rst"; ok=0
    else
      rsets+=(--set run.restart_file="$rst/wrfrst_d01_$at")
      if ! "$EXE" "$cfg" "${rsets[@]}" > "$rdir/run.log" 2>&1; then
        echo "FAIL $name: resumed run failed"; tail -3 "$rdir/run.log"; ok=0
      else
        check_domains "$rdir" wrfout "resumed history" "$rsm" || ok=0
        for f in "$rsm"/wrfout_d0*; do
          [ -f "$f" ] || continue
          b=$(basename "$f"); ours=$rdir/$b; [ -f "$ours" ] || ours=$rdir/${b:7:3}/$b
          $CMP "$(ignored "$f" "$name")" "$ours" "$f" > "$rdir/cmp.log" 2>&1 || { ok=0; echo "FAIL $name resumed $b"; head -20 "$rdir/cmp.log"; }
          n=$((n + 1))
        done
      fi
    fi
    if [ "$n" = "$before_resume" ]; then echo "FAIL $name: resumed leg compared 0 files"; ok=0; fi
  fi
  if [ "$n" = "$before" ]; then echo "FAIL $name: compared 0 files"; ok=0; fi
  if [ $ok = 1 ]; then echo "PASS $name"; rm -rf "${WORK:?}/$name"; else fail=1; fi
done <<< "$CASES"
echo "compared $n files, $skip cases skipped"
[ "$skip" = 0 ] || [ "${CPPWRF_ALLOW_SKIP:-0}" = 1 ] || fail=1
[ "$n" -gt 0 ] || fail=1
exit $fail
