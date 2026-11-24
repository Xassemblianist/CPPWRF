#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The first verification of the AMD HIP build on an AMD GPU node (docs/HIP.md section "First run on AMD hardware"):
# one command on a fresh Ubuntu machine with ROCm and one or more AMD Instinct GPUs (MI300X: gfx942) builds HIP FP64,
# HIP FP32 identity and fast HIP FP32 (and CPU FP64, for triage), runs their ctest, the device math proofs and the
# GPU suite against WRF (tests/verify/wrf_suite.py: FP64 and FP32 identity at tolerance 0, fast FP32 by relative L2)
# with the reference data of the public cppwrf-data release, and writes a summary.
#
#   tools/amd/amd_run.sh [--work DIR] [--arch GFX] [--gpus N] [--flavours dp,id,sp] [--cases NAME,...] [--no-slow]
#                        [--no-cpu] [--no-proofs] [--no-ctest] [--no-data] [--compile-only] [--install-rocm VER]
#                        [--redo STEP[,STEP]]
#
# --work: everything goes there (default ~/cppwrf_amd; needs about 250 GB for the data of the full GPU suite):
#   b-hip-dp, b-hip-id, b-hip-sp, b-cpu-dp (builds), data/ (wrf_runs, wrf_ref), suite_<flavour>/ (outputs of failed
#   cases), res/ (the results: summary.txt, env.txt, build_*.log, ctest_*.log, proof_*.log, suite_*.log, data.txt,
#   amd_run.log), state/ (one stamp per finished step).
# --arch: the GPU architecture to compile for (default: the first GPU agent of rocminfo, e.g. gfx942 on MI300X).
# --gpus: the GPUs to run the suite on at once, each with a share of the cases (default: every GPU rocminfo lists;
#   the HIP runtime's device order, HIP_VISIBLE_DEVICES=i per share).
# --flavours: dp (HIP FP64: the cuda_dp checks, --hip-dp), id (HIP FP32 identity: cuda_sp_id, --hip-sp-id), sp (fast
#   HIP FP32: cuda, --hip). --cases: only these checks (exact names; tools/amd/amd_cases.py list KEY names them all).
# --no-slow: without the slow cases (the CUDA proofs run them: 423 FP64 checks).
# --compile-only: dependencies, builds and the proofs' compile only (no GPU needed; --arch required).
# --install-rocm VER: install ROCm VER (e.g. 6.4.1) from repo.radeon.com when /opt/rocm has no HIP compiler (the GPU
#   driver must be there already); default: refuse.
# --redo: run these steps again (deps, env, data, build_<b>, ctest_<b>, proofs, suite_<flavour>).
# Resumable: a step that finished is skipped on the next run (its stamp in state/); a suite run again runs only the
# checks without a PASS line in its earlier logs. Exit 0 only for "AMD RUN PASS" (summary.txt's last line): every
# build, ctest, proof and suite check passed ("AMD COMPILE PASS" with --compile-only).
set -uo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WORK=$HOME/cppwrf_amd ARCH= GPUS= FLAV=dp,id,sp CASES= SLOW=1 CPU=1 PROOFS=1 CTEST=1 DATA=1 COMPILE_ONLY=0
ROCM=${ROCM_PATH:-/opt/rocm} INSTALL_ROCM= REDO=
DATA_REPO=${DATA_REPO:-Xassemblianist/cppwrf-data}
while [ $# -gt 0 ]; do
  case $1 in
    --work) WORK=$2; shift 2 ;; --arch) ARCH=$2; shift 2 ;; --gpus) GPUS=$2; shift 2 ;;
    --flavours) FLAV=$2; shift 2 ;; --cases) CASES=$2; shift 2 ;; --no-slow) SLOW=0; shift ;;
    --no-cpu) CPU=0; shift ;; --no-proofs) PROOFS=0; shift ;; --no-ctest) CTEST=0; shift ;; --no-data) DATA=0; shift ;;
    --compile-only) COMPILE_ONLY=1; shift ;; --install-rocm) INSTALL_ROCM=$2; shift 2 ;; --redo) REDO=$2; shift 2 ;;
    *) sed -n '3,30p' "$0"; exit 2 ;;
  esac
done
mkdir -p "$WORK"/{state,res} || exit 1
WORK=$(cd "$WORK" && pwd)
R=$WORK/res S=$WORK/res/summary.txt D=$WORK/data
exec > >(tee -a "$R/amd_run.log") 2>&1
log() { printf '[amd_run] %s %s\n' "$(date '+%F %T')" "$*"; }
for s in ${REDO//,/ }; do rm -f "$WORK/state/$s"; done
step() {  # step NAME CMD...: CMD once (its stamp state/NAME)
  local n=$1
  shift
  if [ -e "$WORK/state/$n" ]; then log "$n: done earlier (--redo $n runs it again)"; return 0; fi
  log "$n ..."
  if "$@"; then touch "$WORK/state/$n"; log "$n: ok"; return 0; fi
  log "$n: FAILED"
  return 1
}
SUDO=
[ "$(id -u)" = 0 ] || SUDO=sudo
NPROC=$(nproc)

# ---- dependencies: g++, CMake >= 3.24, NetCDF, Python with numpy and netCDF4, git, curl; ROCm's HIP compiler
hip_cxx() {
  local c
  for c in "$ROCM/llvm/bin/clang++" "$ROCM/lib/llvm/bin/clang++"; do [ -x "$c" ] && { echo "$c"; return 0; }; done
  return 1
}
deps() {
  export DEBIAN_FRONTEND=noninteractive
  $SUDO apt-get -qq update > "$R/apt.log" 2>&1
  $SUDO apt-get -qq install -y build-essential git curl ca-certificates cmake libnetcdf-dev netcdf-bin python3 \
    python3-pip python3-numpy python3-netcdf4 >> "$R/apt.log" 2>&1 || { log "apt-get failed (res/apt.log)"; return 1; }
  if ! hip_cxx > /dev/null; then
    if [ -z "$INSTALL_ROCM" ]; then
      log "no HIP compiler under $ROCM (ROCM_PATH): install ROCm (6.0 or later) or pass --install-rocm 6.4.1"
      return 1
    fi
    . /etc/os-release
    $SUDO mkdir -p /etc/apt/keyrings
    curl -fsSL https://repo.radeon.com/rocm/rocm.gpg.key | gpg --dearmor | $SUDO tee /etc/apt/keyrings/rocm.gpg > /dev/null
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/rocm.gpg] https://repo.radeon.com/rocm/apt/$INSTALL_ROCM $VERSION_CODENAME main" |
      $SUDO tee /etc/apt/sources.list.d/rocm.list > /dev/null
    printf 'Package: *\nPin: release o=repo.radeon.com\nPin-Priority: 600\n' | $SUDO tee /etc/apt/preferences.d/rocm-pin-600 > /dev/null
    $SUDO apt-get -qq update >> "$R/apt.log" 2>&1
    # the HIP compiler, runtime and device libraries (rocm-hip-sdk, with every ROCm library, if these do not resolve)
    $SUDO apt-get -qq install -y rocm-hip-runtime-dev rocm-llvm hipcc hip-dev rocm-device-libs rocm-cmake hsa-rocr-dev \
      comgr rocminfo >> "$R/apt.log" 2>&1 || $SUDO apt-get -qq install -y rocm-hip-sdk rocminfo >> "$R/apt.log" 2>&1 ||
      { log "ROCm install failed (res/apt.log)"; return 1; }
    hip_cxx > /dev/null || { log "no HIP compiler after the ROCm install"; return 1; }
  fi
  # CMake 3.24 or later (Ubuntu 22.04 has 3.22): pip's then
  if ! cmake --version | awk 'NR == 1 { split($3, v, "."); exit !(v[1] > 3 || (v[1] == 3 && v[2] >= 24)) }'; then
    python3 -m pip install -q --user cmake >> "$R/apt.log" 2>&1 || python3 -m pip install -q --user --break-system-packages cmake >> "$R/apt.log" 2>&1
    export PATH=$HOME/.local/bin:$PATH
    hash -r
    cmake --version | head -1 | grep -qE ' 3\.(2[4-9]|[3-9][0-9])| [4-9]\.' || { log "CMake 3.24 or later needed"; return 1; }
  fi
  # (a python3 that is not the system's, which the apt packages are built for: pip's own wheels)
  local pipi="python3 -m pip install -q --user --upgrade --ignore-installed numpy netCDF4"
  python3 -c 'import numpy, netCDF4' 2> /dev/null || $pipi >> "$R/apt.log" 2>&1 ||
    $pipi --break-system-packages >> "$R/apt.log" 2>&1
  if ! python3 -c 'import numpy, netCDF4'; then
    log "python3 numpy / netCDF4 missing (the suite needs them)"
    [ "$COMPILE_ONLY" = 1 ] || return 1
  fi
}
export PATH=$HOME/.local/bin:$PATH

# ---- the machine
env_txt() {
  {
    echo "date $(date -u '+%F %T') UTC"
    echo "commit $(git -C "$REPO" rev-parse HEAD 2> /dev/null) $(git -C "$REPO" status --porcelain 2> /dev/null | grep -q . && echo '(modified tree)')"
    grep PRETTY_NAME /etc/os-release
    uname -r
    echo "nproc $NPROC, $(free -g | awk '/Mem:/ {print $2}') GB RAM, disk $(df -h "$WORK" | awk 'NR == 2 {print $4 " free of " $2}')"
    grep -m1 'model name' /proc/cpuinfo
    ldd --version | head -1
    gcc --version | head -1
    cmake --version | head -1
    echo "HIP compiler: $(hip_cxx) $("$(hip_cxx)" --version 2> /dev/null | head -1)"
    "$ROCM/bin/hipconfig" --version 2> /dev/null && echo
    cat "$ROCM/.info/version" 2> /dev/null
    echo "arch $ARCH, GPUs used $GPUS"
    rocminfo 2> /dev/null | grep -E '^\s+(Marketing Name|Name):\s+gfx|Marketing Name' | sed 's/^ */rocminfo: /' | sort | uniq -c
    rocm-smi --showproductname --showmeminfo vram --showdriverversion 2> /dev/null | grep -vE '^=+|^$' | head -40
  } > "$R/env.txt" 2>&1
  return 0
}

# ---- the GPU suite's checks of a flavour and their data
key_of() { case $1 in dp) echo cuda_dp ;; id) echo cuda_sp_id ;; sp) echo cuda ;; esac; }
flag_of() { case $1 in dp) echo --hip-dp ;; id) echo --hip-sp-id ;; sp) echo --hip ;; esac; }
checks_of() {  # the checks of flavour $1 (--cases: those of them)
  local all
  all=$(python3 "$REPO/tools/amd/amd_cases.py" list "$(key_of "$1")" $([ "$SLOW" = 1 ] && echo --slow)) || return 1
  if [ -n "$CASES" ]; then grep -Fx -f <(tr , '\n' <<< "$CASES") <<< "$all"; else echo "$all"; fi
}
# the ctest data of the CUDA proofs (tests that read WRF runs outside the suite's cases)
CTEST_TARS=port_gem10_data.tar,port_gem13_data.tar,port_v111_data.tar,real_exe_data.tar
data() {
  local names tars v05 url=https://github.com/$DATA_REPO/releases/download
  rm -rf "$WORK/cppwrf-data"
  git clone -q --depth 1 "https://github.com/$DATA_REPO" "$WORK/cppwrf-data" || { log "git clone $DATA_REPO failed"; return 1; }
  names=$(for f in ${FLAV//,/ }; do checks_of "$f"; done | sort -u)
  [ -n "$names" ] || { log "no checks selected"; return 1; }
  tars=$(python3 "$REPO/tests/verify/wrf_suite.py" --data-for "$(paste -sd, <<< "$names")") || { log "wrf_suite.py --data-for failed"; return 1; }
  [ "$CTEST" = 1 ] && [ -z "$CASES" ] && tars="$tars,$CTEST_TARS"
  # v05_files_data.tar is not in data-v1 as a whole: its files come from the packs of data-v2 (after the tars)
  v05=0
  grep -qw v05_files_data.tar <<< "${tars//,/ }" && v05=1
  tars=$(tr , '\n' <<< "$tars" | grep -vx v05_files_data.tar | awk '!s[$0]++' | paste -sd,)
  echo "tars $tars" > "$R/data_list.txt"
  mkdir -p "$D" "$WORK/parts"
  : > "$R/setup.out"
  DATA_REPO=$DATA_REPO DATA_TAG=data-v1 PARTS=$WORK/parts LOG=$R/data.txt SETUP=$R/setup.out \
    bash "$WORK/cppwrf-data/ci/fetch_data.sh" "$D" "$tars"
  local rc=$?
  if [ "$v05" = 1 ]; then
    curl -fsSL --retry 5 -o "$WORK/parts/INDEX.tsv.gz" "$url/data-v2/INDEX.tsv.gz" &&
      gunzip -f "$WORK/parts/INDEX.tsv.gz" &&
      curl -fsSL --retry 5 -o "$WORK/parts/SHA256SUMS.v2" "$url/data-v2/SHA256SUMS" || { log "data-v2 index failed"; return 1; }
    # shellcheck disable=SC2086
    python3 "$REPO/tools/amd/amd_cases.py" members v05_files_data.tar "$WORK/parts/INDEX.tsv" "$WORK/cppwrf-data/ci" \
      $names > "$WORK/parts/v05_members.tsv" || { log "v05 members failed"; return 1; }
    local p
    for p in $(cut -f1 "$WORK/parts/v05_members.tsv" | sort -u); do
      # shellcheck disable=SC2046
      python3 "$WORK/cppwrf-data/ci/fetch_pack.py" "$url/data-v2" "$WORK/parts/$p.stat" $(grep -E "  ${p//./\\.}\.part[0-9]+$" \
        "$WORK/parts/SHA256SUMS.v2" | sort -k2 | awk '{print $2, $1}') |
        tar xf - -C "$D" -- $(awk -F'\t' -v p="$p" '$1 == p {print $2}' "$WORK/parts/v05_members.tsv") &&
        echo "PACK $p $(cat "$WORK/parts/$p.stat")" >> "$R/data.txt" || { echo "DATA_FAILED $p" >> "$R/setup.out"; rc=1; }
    done
    # the links the re-pack left out (as fetch_data.sh does for the tars of its list)
    if curl -fsSL --retry 5 -o "$WORK/parts/v05.links" "$url/data-v1/v05_files_data.tar.links"; then
      tar xf "$WORK/parts/v05.links" -C "$D" && echo "LINKS v05_files_data.tar.links" >> "$R/data.txt"
    fi
  fi
  echo "DATA_DONE $(du -sh "$D" | cut -f1)" >> "$R/data.txt"
  grep DATA_FAILED "$R/setup.out" && rc=1
  return $rc
}

# ---- the builds
build() {  # build NAME: b-hip-dp, b-hip-id, b-hip-sp, b-cpu-dp
  local b=$1 dir=$WORK/$1 log=$R/build_$1.log
  local -a o=(-DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_NATIVE=OFF)
  case $b in
    b-hip-*) o+=(-DCPPWRF_ENABLE_HIP=ON -DCMAKE_PREFIX_PATH="$ROCM" -DCMAKE_HIP_COMPILER="$(hip_cxx)"
                 -DCMAKE_HIP_ARCHITECTURES="$ARCH") ;;
    *) o+=(-DCPPWRF_ENABLE_HIP=OFF) ;;
  esac
  case $b in
    *-dp) o+=(-DCPPWRF_DOUBLE=ON) ;;
    b-hip-id) o+=(-DCPPWRF_DOUBLE=OFF -DCPPWRF_HIP_FP32_IDENTITY=ON) ;;
    b-hip-sp) o+=(-DCPPWRF_DOUBLE=OFF -DCPPWRF_HIP_FP32_IDENTITY=OFF) ;;
  esac
  local t0=$SECONDS
  cmake -S "$REPO" -B "$dir" "${o[@]}" > "$log" 2>&1 && cmake --build "$dir" -j"$NPROC" >> "$log" 2>&1
  local rc=$?
  echo "build $b: exit $rc, $((SECONDS - t0)) s; $("$dir/cppwrf" --version 2> /dev/null | head -1)" >> "$R/builds.txt"
  [ $rc = 0 ] || { grep -m20 -E 'error|Error' "$log" >> "$R/builds.txt"; return 1; }
  case $b in  # the binary says what it is
    b-hip-dp) "$dir/cppwrf" --version | grep -q 'backend hip precision float64' ;;
    b-hip-id | b-hip-sp) "$dir/cppwrf" --version | grep -q 'backend hip precision float32' ;;
    *) "$dir/cppwrf" --version | grep -q 'backend cpu precision float64' ;;
  esac
}

# ---- ctest of a build (the reference data in place: the tests that read WRF runs)
ctest_of() {
  local b=$1 log=$R/ctest_$1.log
  (cd "$WORK/$b" && CPPWRF_WRF_RUNS=$D/wrf_runs CPPWRF_WRF_REF=$D/wrf_ref CPPWRF_SCRATCH=$WORK/ctest_scratch \
    timeout 7200 ctest --output-on-failure -j4) > "$log" 2>&1
  local rc=$?
  echo "ctest $b: exit $rc: $(grep -E 'tests passed|No tests were found' "$log")" >> "$R/ctests.txt"
  grep -E 'Test +#.*(Failed|Not Run|\*\*\*)' "$log" | cut -c1-180 >> "$R/ctests.txt"
  return $rc
}

# ---- the device math proofs (device against host, bit for bit): glibc's double functions and libmvec's 2-lane ones
# (10^7 random inputs per function and the edge values), glibc's float functions and libmvec's 4-lane ones (every
# float input; pow and atan2 on 2^28 pairs)
proofs() {  # proofs [compile]: build (and run) both
  local cxx rc=0 only=${1:-}
  cxx=$(hip_cxx) || return 1
  local -a f=(-x hip --offload-arch="$ARCH" -std=c++20 -O3 -ffp-contract=off -fno-gpu-flush-denormals-to-zero
              -fhip-fp32-correctly-rounded-divide-sqrt -fopenmp -I"$REPO/src" -I"$REPO/tools" --hip-link
              -Wl,-rpath,"$(dirname "$cxx")/../lib")
  "$cxx" "${f[@]}" -DCPPWRF_GLIBC_MATH "$REPO/tools/glibc_math_proof_cuda.cu" -o "$WORK/glibc_math_proof_hip" \
    > "$R/proof_fp64.log" 2>&1 &&
    { [ "$only" = compile ] || OMP_NUM_THREADS=$NPROC "$WORK/glibc_math_proof_hip" 10000000 >> "$R/proof_fp64.log" 2>&1; }
  local r1=$?
  echo "glibc math proof (FP64)${only:+ $only}: exit $r1: $(tail -1 "$R/proof_fp64.log")" >> "$R/proofs.txt"
  "$cxx" "${f[@]}" -DCPPWRF_GLIBC_MATH -DCPPWRF_FP32_IDENTITY "$REPO/tools/glibc_mathf_proof_cuda.cu" \
    -o "$WORK/glibc_mathf_proof_hip" > "$R/proof_fp32.log" 2>&1 &&
    { [ "$only" = compile ] || OMP_NUM_THREADS=$NPROC "$WORK/glibc_mathf_proof_hip" 28 >> "$R/proof_fp32.log" 2>&1; }
  local r2=$?
  echo "glibc float math proof (FP32): exit $r2: $(tail -1 "$R/proof_fp32.log")" >> "$R/proofs.txt"
  [ $r1 = 0 ] && [ $r2 = 0 ] || rc=1
  return $rc
}

# ---- the suite of a flavour, its checks shared out over the GPUs (one wrf_suite.py per GPU); a check that passed
# in an earlier run of this step is not run again
passed_of() { cat "$R"/suite_"$1"_*.log 2> /dev/null | awk '$1 == "PASS" {print $2}' | sort -u; }
suite() {
  local fl=$1 bin=$WORK/b-hip-$1/cppwrf run all todo g n
  all=$(checks_of "$fl") || return 1
  todo=$(grep -vx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all")
  run=$(date +%Y%m%d_%H%M%S)
  n=$(grep -c . <<< "$todo")
  log "suite $fl: $(grep -c . <<< "$all") checks, $n to run on $GPUS GPU(s)"
  if [ "$n" -gt 0 ]; then
    local per=$((NPROC / GPUS))
    [ $per -ge 1 ] || per=1
    local -a pids=()
    for ((g = 0; g < GPUS; g++)); do
      local mine
      mine=$(awk -v g=$g -v n="$GPUS" 'NF && (NR - 1) % n == g' <<< "$todo" | paste -sd,)
      [ -n "$mine" ] || continue
      (cd "$REPO" && HIP_VISIBLE_DEVICES=$g OMP_NUM_THREADS=$per CPPWRF_WRF_RUNS=$D/wrf_runs CPPWRF_WRF_REF=$D/wrf_ref \
        python3 tests/verify/wrf_suite.py "$(flag_of "$fl")" "$bin" --exact --only "$mine" --out "$WORK/suite_$fl/g$g" \
        > "$R/suite_${fl}_${run}_g$g.log" 2>&1) &
      pids+=($!)
    done
    for p in "${pids[@]}"; do wait "$p"; done
  fi
  local pass miss
  pass=$(grep -cx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all")
  miss=$(grep -vx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all" | grep -c .)
  {
    echo "suite $fl ($(flag_of "$fl")): $pass of $(grep -c . <<< "$all") checks passed, $miss without a PASS"
    cat "$R"/suite_"$fl"_"$run"_g*.log 2> /dev/null | grep -E '^(FAIL|SKIP|ERROR)( |:)' | cut -c1-400
  } >> "$R/suites.txt"
  [ "$miss" = 0 ]
}

summary() {
  {
    echo "== CPPWRF AMD HIP run, $(date -u '+%F %T') UTC"
    cat "$R/env.txt" 2> /dev/null
    echo "== data"
    grep -E 'DATA_DONE|DATA_FAILED' "$R/data.txt" "$R/setup.out" 2> /dev/null
    echo "== builds"
    cat "$R/builds.txt" 2> /dev/null
    echo "== ctest"
    cat "$R/ctests.txt" 2> /dev/null
    echo "== device math proofs"
    cat "$R/proofs.txt" 2> /dev/null
    echo "== suites"
    cat "$R/suites.txt" 2> /dev/null
    echo "== steps"
    for s in $STEPS; do
      if [ -e "$WORK/state/$s" ]; then echo "$s done"; else echo "$s NOT DONE"; fi
    done
    echo "AMD $([ "$COMPILE_ONLY" = 1 ] && echo COMPILE || echo RUN) $([ "$1" = 0 ] && echo PASS || echo FAIL)"
  } > "$S"
  cat "$S"
}

# ---- the run
STEPS="deps env"
fail=0
step deps deps || { summary 1; exit 1; }
if [ -z "$ARCH" ]; then
  ARCH=$(rocminfo 2> /dev/null | grep -o -m1 'gfx[0-9a-f]\+')
  [ -n "$ARCH" ] || { log "no AMD GPU found by rocminfo: pass --arch (e.g. gfx942) with --compile-only"; summary 1; exit 1; }
fi
if [ -z "$GPUS" ]; then
  GPUS=$(rocminfo 2> /dev/null | awk '/Device Type:[[:space:]]*GPU/ {n++} END {print n + 0}')
  [ "$GPUS" -ge 1 ] || GPUS=1
fi
rm -f "$WORK/state/env"
step env env_txt
log "arch $ARCH, $GPUS GPU(s), flavours $FLAV, work $WORK"
DPID=
if [ "$COMPILE_ONLY" = 0 ] && [ "$DATA" = 1 ]; then
  STEPS="$STEPS data"
  if [ ! -e "$WORK/state/data" ]; then
    (step data data) &  # the data download runs while the programs build
    DPID=$!
  fi
fi
BUILDS=""
for f in ${FLAV//,/ }; do BUILDS="$BUILDS b-hip-$f"; done
[ "$CPU" = 1 ] && BUILDS="$BUILDS b-cpu-dp"
for b in $BUILDS; do
  STEPS="$STEPS build_$b"
  step "build_$b" build "$b" || fail=1
done
if [ -n "$DPID" ]; then wait "$DPID" || fail=1; fi
if [ "$COMPILE_ONLY" = 1 ] && [ "$PROOFS" = 1 ]; then
  STEPS="$STEPS proofs_compile"
  step proofs_compile proofs compile || fail=1
fi
if [ "$COMPILE_ONLY" = 0 ]; then
  [ "$DATA" = 1 ] && [ ! -e "$WORK/state/data" ] && fail=1
  if [ "$CTEST" = 1 ]; then
    for f in ${FLAV//,/ }; do
      STEPS="$STEPS ctest_b-hip-$f"
      [ -e "$WORK/state/build_b-hip-$f" ] && { step "ctest_b-hip-$f" ctest_of "b-hip-$f" || fail=1; }
    done
  fi
  if [ "$PROOFS" = 1 ]; then
    STEPS="$STEPS proofs"
    step proofs proofs || fail=1
  fi
  for f in ${FLAV//,/ }; do
    STEPS="$STEPS suite_$f"
    if [ -e "$WORK/state/build_b-hip-$f" ]; then step "suite_$f" suite "$f" || fail=1
    else fail=1; fi
  done
fi
summary $fail
exit $fail
