#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The first verification of the AMD HIP build on an AMD GPU node (docs/HIP.md section "First run on AMD hardware"):
# one command on a fresh Ubuntu machine with ROCm and one or more AMD Instinct GPUs (MI300X: gfx942) builds HIP FP64,
# HIP FP32 identity and fast HIP FP32 (and CPU FP64, for triage), runs their ctest, the device math proofs and the
# GPU suite against WRF (tests/verify/wrf_suite.py: FP64 and FP32 identity at tolerance 0, fast FP32 by relative L2)
# with the reference data of the public cppwrf-data release, compares the HIP FP64 outputs with the CUDA FP64 build's
# (every variable of every output file, by hashes) and writes a summary.
#
#   tools/amd/amd_run.sh [--work DIR] [--arch GFX] [--gpus N] [--flavours dp,id,sp] [--cases NAME,...] [--no-slow]
#                        [--no-cpu] [--no-proofs] [--no-ctest] [--no-data] [--compile-only] [--install-rocm VER]
#                        [--cuda-hashes FILE|URL] [--no-cross] [--run-hours H] [--redo STEP[,STEP]]
#
# --work: everything goes there (default ~/cppwrf_amd; needs about 250 GB for the data of the full GPU suite):
#   b-hip-dp, b-hip-id, b-hip-sp, b-cpu-dp (builds), data/ (wrf_runs, wrf_ref), suite_<flavour>/ (outputs of failed
#   cases), res/ (the results: summary.txt, env.txt, steps/ (one result file per step, rewritten when the step runs),
#   build_*.log, ctest_*.log, proof_*.log, suite_<flavour>/<key>/ (the suite logs and hashes of one binary), data.txt,
#   amd_run.log), state/ (one stamp per finished step, holding the key of what it checked).
# --arch: the GPU architecture to compile for (default: the first GPU agent of rocminfo, e.g. gfx942 on MI300X).
# --gpus: how many of the job's GPUs run the suite at once, each with a share of the checks (default: all of them).
#   The job's GPUs: HIP_VISIBLE_DEVICES (or ROCR_VISIBLE_DEVICES) as the scheduler set it, else every GPU of rocminfo;
#   each worker gets one of them; ctest and the proofs run on the first.
# --flavours: dp (HIP FP64: the cuda_dp checks, --hip-dp), id (HIP FP32 identity: cuda_sp_id, --hip-sp-id), sp (fast
#   HIP FP32: cuda, --hip). --cases: only these checks (exact names; tools/amd/amd_cases.py list KEY names them all):
#   every name must be a check of a selected flavour (else nothing runs), and a flavour without one is left out.
# --no-slow: without the slow cases (the CUDA proofs run them: 423 FP64 checks).
# --compile-only: dependencies, builds and the proofs' compile only (no GPU needed; --arch required).
# --install-rocm VER: install ROCm VER (e.g. 6.4.1) from repo.radeon.com when $ROCM_PATH (/opt/rocm) has no HIP
#   compiler (the GPU driver must be there already); default: refuse.
# --cuda-hashes: the CUDA FP64 build's output hashes (tests/verify/out_hashes.py; wrf_suite.py --cuda-dp ... --hash-out
#   FILE on an NVIDIA GPU, same commit or one with the same CUDA results): step cross_dp compares the HIP FP64 build's
#   with them, every variable of every output file of every passing check. Without it the run stops before it builds
#   (--no-cross: the run may pass without the direct comparison; the summary says so).
# --run-hours: the longest a model run of the suite may take (default 3; then it fails, exit 124).
# --redo: run these steps again (deps, env, data, build_<b>, ctest_<b>, proofs, suite_<flavour>, cross_dp); --redo
#   suite_<fl> also discards that flavour's PASS lines of the current binary, --redo data every fetched tar's mark.
# Resumable: a step whose stamp holds the current key is skipped (the key: the commit and changes, the architecture,
# the compiler and ROCm for a build; the binaries' sha256 for ctest, the suite and the comparison; the data list for
# the data): a rebuilt binary that differs runs its checks again. A suite started again runs only the checks of the
# same binary without a PASS line. Exit 0 only for "AMD RUN PASS" (summary.txt's last line): every build, ctest,
# proof, suite check and the comparison with CUDA passed ("AMD COMPILE PASS" with --compile-only).
set -uo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WORK=$HOME/cppwrf_amd ARCH= GPUS= FLAV=dp,id,sp CASES= SLOW=1 CPU=1 PROOFS=1 CTEST=1 DATA=1 COMPILE_ONLY=0
ROCM=${ROCM_PATH:-/opt/rocm} INSTALL_ROCM= REDO= CUDA_HASHES= CROSS=1 RUN_HOURS=3
DATA_REPO=${DATA_REPO:-Xassemblianist/cppwrf-data}
usage() { sed -n '3,41p' "$0"; exit 2; }
need() { [ $# -ge 2 ] && [ -n "$2" ] || { echo "amd_run.sh: $1 needs a value" >&2; usage; }; }
while [ $# -gt 0 ]; do
  case $1 in
    --work | --arch | --gpus | --flavours | --cases | --install-rocm | --redo | --cuda-hashes | --run-hours)
      need "$@"
      case $1 in
        --work) WORK=$2 ;; --arch) ARCH=$2 ;; --gpus) GPUS=$2 ;; --flavours) FLAV=$2 ;; --cases) CASES=$2 ;;
        --install-rocm) INSTALL_ROCM=$2 ;; --redo) REDO=$2 ;; --cuda-hashes) CUDA_HASHES=$2 ;; --run-hours) RUN_HOURS=$2 ;;
      esac
      shift 2 ;;
    --no-slow) SLOW=0; shift ;; --no-cpu) CPU=0; shift ;; --no-proofs) PROOFS=0; shift ;; --no-ctest) CTEST=0; shift ;;
    --no-data) DATA=0; shift ;; --compile-only) COMPILE_ONLY=1; shift ;; --no-cross) CROSS=0; shift ;;
    *) usage ;;
  esac
done
[ -z "$GPUS" ] || [[ $GPUS =~ ^[1-9][0-9]*$ ]] || { echo "amd_run.sh: --gpus $GPUS: a positive number" >&2; exit 2; }
[[ $RUN_HOURS =~ ^[1-9][0-9]*$ ]] || { echo "amd_run.sh: --run-hours $RUN_HOURS: a positive number" >&2; exit 2; }
for f in ${FLAV//,/ }; do
  case $f in dp | id | sp) ;; *) echo "amd_run.sh: --flavours: dp, id, sp (not $f)" >&2; exit 2 ;; esac
done
mkdir -p "$WORK"/{state,res/steps} || exit 1
WORK=$(cd "$WORK" && pwd)
R=$WORK/res S=$WORK/res/summary.txt D=$WORK/data
exec > >(tee -a "$R/amd_run.log") 2>&1
log() { printf '[amd_run] %s %s\n' "$(date '+%F %T')" "$*"; }
sha() { sha256sum "$@" 2> /dev/null | sha256sum | cut -c1-16; }
for s in ${REDO//,/ }; do
  rm -f "$WORK/state/$s"
  [ "$s" = data ] && { rm -f "$WORK/state/data_tars"; rm -rf "$WORK/state/data_v05"; }
done
step() {  # step NAME KEY CMD...: CMD once per KEY (its stamp state/NAME holds the key)
  local n=$1 key=$2
  shift 2
  if [ -e "$WORK/state/$n" ] && [ "$(cat "$WORK/state/$n")" = "$key" ]; then
    log "$n: done earlier for this key (--redo $n runs it again)"
    return 0
  fi
  rm -f "$WORK/state/$n"
  log "$n ..."
  if "$@"; then echo "$key" > "$WORK/state/$n"; log "$n: ok"; return 0; fi
  log "$n: FAILED"
  return 1
}
res() { echo "$R/steps/$1.txt"; }  # the result lines of step $1 (rewritten each time it runs)
SUDO=
[ "$(id -u)" = 0 ] || SUDO=sudo
NPROC=$(nproc)

# ---- dependencies: g++, CMake >= 3.24, NetCDF, Python with numpy and netCDF4, git, curl, gnupg; ROCm's HIP compiler
hip_cxx() {
  local c
  for c in "$ROCM/llvm/bin/clang++" "$ROCM/lib/llvm/bin/clang++"; do [ -x "$c" ] && { echo "$c"; return 0; }; done
  return 1
}
deps() {
  export DEBIAN_FRONTEND=noninteractive
  $SUDO apt-get -qq update > "$R/apt.log" 2>&1 || { log "apt-get update failed (res/apt.log)"; return 1; }
  $SUDO apt-get -qq install -y build-essential git curl ca-certificates gnupg cmake libnetcdf-dev netcdf-bin python3 \
    python3-pip python3-numpy python3-netcdf4 >> "$R/apt.log" 2>&1 || { log "apt-get failed (res/apt.log)"; return 1; }
  if ! hip_cxx > /dev/null; then
    if [ -z "$INSTALL_ROCM" ]; then
      log "no HIP compiler under $ROCM (ROCM_PATH): install ROCm (6.0 or later) or pass --install-rocm 6.4.1"
      return 1
    fi
    . /etc/os-release
    $SUDO mkdir -p /etc/apt/keyrings || return 1
    local key
    key=$(mktemp) || return 1
    curl -fsSL --retry 5 -o "$key" https://repo.radeon.com/rocm/rocm.gpg.key &&
      gpg --dearmor < "$key" | $SUDO tee /etc/apt/keyrings/rocm.gpg > /dev/null &&
      [ -s /etc/apt/keyrings/rocm.gpg ] || { rm -f "$key"; log "ROCm signing key failed"; return 1; }
    rm -f "$key"
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/rocm.gpg] https://repo.radeon.com/rocm/apt/$INSTALL_ROCM $VERSION_CODENAME main" |
      $SUDO tee /etc/apt/sources.list.d/rocm.list > /dev/null || return 1
    printf 'Package: *\nPin: release o=repo.radeon.com\nPin-Priority: 600\n' | $SUDO tee /etc/apt/preferences.d/rocm-pin-600 > /dev/null || return 1
    $SUDO apt-get -qq update >> "$R/apt.log" 2>&1 || { log "apt-get update with the ROCm repository failed (res/apt.log)"; return 1; }
    # the HIP compiler, runtime and device libraries (rocm-hip-sdk, with every ROCm library, if these do not resolve)
    $SUDO apt-get -qq install -y rocm-hip-runtime-dev rocm-llvm hipcc hip-dev rocm-device-libs rocm-cmake hsa-rocr-dev \
      comgr rocminfo >> "$R/apt.log" 2>&1 || $SUDO apt-get -qq install -y rocm-hip-sdk rocminfo >> "$R/apt.log" 2>&1 ||
      { log "ROCm install failed (res/apt.log)"; return 1; }
    [ -d "$ROCM" ] || ROCM=/opt/rocm-$INSTALL_ROCM
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

# ---- the job's GPUs (the scheduler's mask, else rocminfo's), one per suite worker; HIP_VISIBLE_DEVICES=<one of them>
GPU_IDS=()
gpu_ids() {
  local mask=${HIP_VISIBLE_DEVICES:-${ROCR_VISIBLE_DEVICES:-}} n i
  if [ -n "$mask" ]; then
    IFS=, read -ra GPU_IDS <<< "$mask"
    # under ROCR_VISIBLE_DEVICES alone the HIP runtime numbers the visible GPUs from 0
    if [ -z "${HIP_VISIBLE_DEVICES:-}" ]; then
      n=${#GPU_IDS[@]}
      GPU_IDS=()
      for ((i = 0; i < n; i++)); do GPU_IDS+=("$i"); done
    fi
  else
    n=$(rocminfo 2> /dev/null | awk '/Device Type:[[:space:]]*GPU/ {n++} END {print n + 0}')
    for ((i = 0; i < n; i++)); do GPU_IDS+=("$i"); done
  fi
}

# ---- the machine
env_txt() {
  {
    echo "date $(date -u '+%F %T') UTC"
    echo "commit $(git -C "$REPO" rev-parse HEAD 2> /dev/null) $(git -C "$REPO" status --porcelain --untracked-files=no 2> /dev/null | grep -q . && echo '(modified tree)')"
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
    echo "arch $ARCH; the job's GPUs ${GPU_IDS[*]:-none} (HIP_VISIBLE_DEVICES=${HIP_VISIBLE_DEVICES:-} ROCR_VISIBLE_DEVICES=${ROCR_VISIBLE_DEVICES:-}); suite workers $GPUS"
    rocminfo 2> /dev/null | grep -E '^\s+(Marketing Name|Name):\s+gfx|Marketing Name' | sed 's/^ */rocminfo: /' | sort | uniq -c
    rocm-smi --showproductname --showmeminfo vram --showdriverversion 2> /dev/null | grep -vE '^=+|^$' | head -40
  } > "$R/env.txt" 2>&1
  return 0
}

# ---- the GPU suite's checks of a flavour and their data
key_of() { case $1 in dp) echo cuda_dp ;; id) echo cuda_sp_id ;; sp) echo cuda ;; esac; }
flag_of() { case $1 in dp) echo --hip-dp ;; id) echo --hip-sp-id ;; sp) echo --hip ;; esac; }
all_of() {  # every check of flavour $1 (with the slow ones unless --no-slow)
  python3 "$REPO/tools/amd/amd_cases.py" list "$(key_of "$1")" $([ "$SLOW" = 1 ] && echo --slow)
}
checks_of() {  # the checks of flavour $1 (--cases: those of them; possibly none)
  local all
  all=$(all_of "$1") || return 1
  if [ -n "$CASES" ]; then grep -Fx -f <(tr , '\n' <<< "$CASES" | grep .) <<< "$all" || true; else echo "$all"; fi
}
# --cases: every name a check of a selected flavour (one typo stops the run before anything is built or fetched);
# the flavours without a selected check are left out
select_cases() {
  [ -n "$CASES" ] || return 0
  local f all="" bad keep=""
  for f in ${FLAV//,/ }; do all+=$(all_of "$f")$'\n' || return 1; done
  bad=$(tr , '\n' <<< "$CASES" | grep . | grep -vxF -f <(grep . <<< "$all"))
  if [ -n "$bad" ]; then
    log "--cases: no check of the flavours $FLAV$([ "$SLOW" = 0 ] && echo ' (without the slow ones: --no-slow)') named: $(paste -sd' ' <<< "$bad")"
    return 1
  fi
  for f in ${FLAV//,/ }; do [ -n "$(checks_of "$f")" ] && keep+="${keep:+,}$f"; done
  [ "$keep" = "$FLAV" ] || log "--cases: flavours $keep only (the others have none of the checks)"
  FLAV=$keep
}
# the ctest data of the CUDA proofs (tests that read WRF runs outside the suite's cases)
CTEST_TARS=port_gem10_data.tar,port_gem13_data.tar,port_v111_data.tar,real_exe_data.tar
data_list() {  # NAMES (the checks), TARS (the data-v1 tars, in order), V05 (v05_files_data.tar's files needed)
  local tars f
  NAMES=$(for f in ${FLAV//,/ }; do checks_of "$f"; done | sort -u)
  [ -n "$NAMES" ] || { log "no checks selected"; return 1; }
  tars=$(python3 "$REPO/tests/verify/wrf_suite.py" --data-for "$(paste -sd, <<< "$NAMES")") || { log "wrf_suite.py --data-for failed"; return 1; }
  [ "$CTEST" = 1 ] && [ -z "$CASES" ] && tars="$tars,$CTEST_TARS"
  V05=0
  grep -qw v05_files_data.tar <<< "${tars//,/ }" && V05=1
  TARS=$(tr , '\n' <<< "$tars" | grep . | grep -vx v05_files_data.tar | awk '!s[$0]++' | paste -sd,)
}
data() {
  local url=https://github.com/$DATA_REPO/releases/download rc=0 t todo
  rm -rf "$WORK/cppwrf-data"
  git clone -q --depth 1 "https://github.com/$DATA_REPO" "$WORK/cppwrf-data" || { log "git clone $DATA_REPO failed"; return 1; }
  data_list || return 1
  echo "tars $TARS v05 $V05" > "$R/data_list.txt"
  mkdir -p "$D" "$WORK/parts"
  # each attempt with fresh transfer state (a stale TAR.ok of an earlier attempt must not pass a failed checksum);
  # the tars unpacked in order are kept (state/data_tars): an attempt starts after the longest finished prefix
  # (fetch_data.sh unpacks in the list's order, a later tar replacing an earlier one's files)
  rm -f "$WORK/parts"/*.ok "$WORK/parts"/*.failed "$WORK/parts"/*.sums "$WORK/parts"/*.tmp "$WORK/parts"/*.stat
  touch "$WORK/state/data_tars"
  todo=$(tr , '\n' <<< "$TARS" | awk -v f="$WORK/state/data_tars" 'BEGIN {while ((getline l < f) > 0) d[l] = 1}
         !p && d[$0] {next} {p = 1; print}' | paste -sd,)
  : > "$R/setup.out"
  : > "$R/data_attempt.txt"
  if [ -n "$todo" ]; then
    DATA_REPO=$DATA_REPO DATA_TAG=data-v1 PARTS=$WORK/parts LOG=$R/data_attempt.txt SETUP=$R/setup.out \
      bash "$WORK/cppwrf-data/ci/fetch_data.sh" "$D" "$todo" || rc=1
    cat "$R/data_attempt.txt" >> "$R/data.txt"
    # the finished prefix of this attempt
    tr , '\n' <<< "$todo" | while read -r t; do grep -q "^TAR $t " "$R/data_attempt.txt" || break; echo "$t"; done \
      >> "$WORK/state/data_tars"
  fi
  # every tar's links again after the tars (the links had the last word on the reference machines)
  for t in ${TARS//,/ }; do
    grep -qE "  ${t//./\\.}\.links$" "$WORK/parts/SHA256SUMS" 2> /dev/null || continue
    if curl -fsSL --retry 5 -o "$WORK/parts/$t.links" "$url/data-v1/$t.links" &&
       (cd "$WORK/parts" && grep -E "  ${t//./\\.}\.links$" SHA256SUMS | sha256sum -c --quiet -) &&
       tar xf "$WORK/parts/$t.links" -C "$D"; then rm -f "$WORK/parts/$t.links"
    else echo "DATA_FAILED $t.links" >> "$R/setup.out"; rc=1; fi
  done
  # v05_files_data.tar is not in data-v1 as a whole: its files come from the packs of data-v2, after every tar
  if [ "$V05" = 1 ] && [ $rc = 0 ]; then
    curl -fsSL --retry 5 -o "$WORK/parts/INDEX.tsv.gz" "$url/data-v2/INDEX.tsv.gz" &&
      gunzip -f "$WORK/parts/INDEX.tsv.gz" &&
      curl -fsSL --retry 5 -o "$WORK/parts/SHA256SUMS.v2" "$url/data-v2/SHA256SUMS" || { log "data-v2 index failed"; return 1; }
    # shellcheck disable=SC2086
    python3 "$REPO/tools/amd/amd_cases.py" members v05_files_data.tar "$WORK/parts/INDEX.tsv" "$WORK/cppwrf-data/ci" \
      $NAMES > "$WORK/parts/v05_members.tsv" || { log "v05 members failed"; return 1; }
    local p parts
    mkdir -p "$WORK/state/data_v05"
    for p in $(cut -f1 "$WORK/parts/v05_members.tsv" | sort -u); do
      [ -e "$WORK/state/data_v05/$p" ] && continue
      # the pack's parts (.part00 ..., .partaa ...) or the pack itself
      parts=$(grep -E "  ${p//./\\.}(\.part[0-9a-z]+)?$" "$WORK/parts/SHA256SUMS.v2" | sort -k2 | awk '{print $2, $1}')
      [ -n "$parts" ] || { echo "DATA_FAILED $p (not in data-v2's SHA256SUMS)" >> "$R/setup.out"; rc=1; continue; }
      # shellcheck disable=SC2046,SC2086
      if python3 "$WORK/cppwrf-data/ci/fetch_pack.py" "$url/data-v2" "$WORK/parts/$p.stat" $parts |
         tar xf - -C "$D" -- $(awk -F'\t' -v p="$p" '$1 == p {print $2}' "$WORK/parts/v05_members.tsv") &&
         [ -s "$WORK/parts/$p.stat" ]; then
        echo "PACK $p $(cat "$WORK/parts/$p.stat")" >> "$R/data.txt"
        touch "$WORK/state/data_v05/$p"
      else echo "DATA_FAILED $p" >> "$R/setup.out"; rc=1; fi
    done
    # the links the re-pack left out (as fetch_data.sh does for the tars of its list), checked
    if [ $rc = 0 ]; then
      if curl -fsSL --retry 5 -o "$WORK/parts/SHA256SUMS.v1" "$url/data-v1/SHA256SUMS" &&
         curl -fsSL --retry 5 -o "$WORK/parts/v05_files_data.tar.links" "$url/data-v1/v05_files_data.tar.links" &&
         (cd "$WORK/parts" && grep -E '  v05_files_data\.tar\.links$' SHA256SUMS.v1 | sha256sum -c --quiet -) &&
         tar xf "$WORK/parts/v05_files_data.tar.links" -C "$D"; then
        echo "LINKS v05_files_data.tar.links" >> "$R/data.txt"
      else echo "DATA_FAILED v05_files_data.tar.links" >> "$R/setup.out"; rc=1; fi
    fi
  fi
  echo "DATA_DONE $(du -sh "$D" | cut -f1)" >> "$R/data.txt"
  grep -q DATA_FAILED "$R/setup.out" && rc=1
  { grep -E 'DATA_DONE' "$R/data.txt" | tail -1; cat "$R/setup.out"; } > "$(res data)"
  return $rc
}

# ---- the builds
build_key() {  # what a build depends on: the commit and its changes, the build, the architecture, the compilers
  local b=$1
  {
    git -C "$REPO" rev-parse HEAD
    git -C "$REPO" diff HEAD 2> /dev/null | sha256sum
    echo "$b $ARCH"
    [[ $b == b-hip-* ]] && { "$(hip_cxx)" --version 2> /dev/null | head -1; cat "$ROCM/.info/version" 2> /dev/null; }
    gcc --version | head -1
  } | sha256sum | cut -c1-16
}
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
  echo "build $b: exit $rc, $((SECONDS - t0)) s; $("$dir/cppwrf" --version 2> /dev/null | head -1)" > "$(res "build_$b")"
  [ $rc = 0 ] || { grep -m20 -E 'error|Error' "$log" >> "$(res "build_$b")"; return 1; }
  case $b in  # the binary says what it is
    b-hip-dp) "$dir/cppwrf" --version | grep -q 'backend hip precision float64' ;;
    b-hip-id | b-hip-sp) "$dir/cppwrf" --version | grep -q 'backend hip precision float32' ;;
    *) "$dir/cppwrf" --version | grep -q 'backend cpu precision float64' ;;
  esac
}
bin_key() {  # the sha256 of a build's cppwrf (the suite), or of all its executables (ctest: bin_key B all)
  # shellcheck disable=SC2046
  if [ "${2:-}" = all ]; then sha $(find "$WORK/$1" -maxdepth 1 -type f -executable | sort)
  else sha "$WORK/$1/cppwrf"; fi
}

# ---- ctest of a build (the reference data in place: the tests that read WRF runs), on the job's first GPU
ctest_of() {
  local b=$1 log=$R/ctest_$1.log out sk
  out=$(res "ctest_$b")
  (cd "$WORK/$b" && HIP_VISIBLE_DEVICES=${GPU_IDS[0]:-0} CPPWRF_WRF_RUNS=$D/wrf_runs CPPWRF_WRF_REF=$D/wrf_ref \
    CPPWRF_SCRATCH=$WORK/ctest_scratch timeout 7200 ctest --output-on-failure -j4) > "$log" 2>&1
  local rc=$?
  echo "ctest $b: exit $rc: $(grep -E 'tests passed|No tests were found' "$log")" > "$out"
  grep -E 'Test +#.*(Failed|Not Run|Timeout|Exception)' "$log" | cut -c1-180 >> "$out"
  # skipped tests (exit 77: their WRF data were not fetched; --cases leaves the ctest tars out)
  sk=$(grep -E 'Test +#.*\*\*\*Skipped' "$log" | sed -E 's/.*Test +#[0-9]+: ([^ ]+).*/\1/' | paste -sd' ')
  [ -z "$sk" ] || echo "ctest $b: SKIPPED (not checked$([ -n "$CASES" ] && echo ': --cases leaves the ctest tars out')): $sk" >> "$out"
  return $rc
}

# ---- the device math proofs (device against host, bit for bit): glibc's double functions and libmvec's 2-lane ones
# (10^7 random inputs per function and the edge values), glibc's float functions and libmvec's 4-lane ones (every
# float input; pow and atan2 on 2^28 pairs). The FP32 proof's host lanes outside the vector ranges call glibc 2.41's
# float functions (src/core/glibc_mathf_host.cpp, linked as in the FP32 builds), not the node's libm.
proof_key() {
  { git -C "$REPO" rev-parse HEAD; git -C "$REPO" diff HEAD 2> /dev/null | sha256sum; echo "$ARCH $ROCM"
    "$(hip_cxx)" --version 2> /dev/null | head -1; } | sha256sum | cut -c1-16
}
proofs() {  # proofs [compile]: build (and run) both
  local cxx rc=0 only=${1:-} out
  out=$(res "proofs${only:+_$only}")
  cxx=$(hip_cxx) || return 1
  local -a f=(-x hip --offload-arch="$ARCH" --rocm-path="$ROCM" -std=c++20 -O3 -ffp-contract=off
              -fno-gpu-flush-denormals-to-zero -fhip-fp32-correctly-rounded-divide-sqrt -fno-builtin-exp2
              -fno-builtin-exp2f -fopenmp -I"$REPO/src" -I"$REPO/tools")
  local -a l=(--hip-link --rocm-path="$ROCM" -fopenmp -Wl,-rpath,"$ROCM/lib" -Wl,-rpath,"$(dirname "$cxx")/../lib")
  local -a run=(env HIP_VISIBLE_DEVICES="${GPU_IDS[0]:-0}" OMP_NUM_THREADS="$NPROC")
  "$cxx" "${f[@]}" -DCPPWRF_GLIBC_MATH -c "$REPO/tools/glibc_math_proof_cuda.cu" -o "$WORK/glibc_math_proof_hip.o" \
    > "$R/proof_fp64.log" 2>&1 &&
    "$cxx" "$WORK/glibc_math_proof_hip.o" "${l[@]}" -o "$WORK/glibc_math_proof_hip" >> "$R/proof_fp64.log" 2>&1 &&
    { [ "$only" = compile ] || "${run[@]}" "$WORK/glibc_math_proof_hip" 10000000 >> "$R/proof_fp64.log" 2>&1; }
  local r1=$?
  echo "glibc math proof (FP64)${only:+ $only}: exit $r1: $(tail -1 "$R/proof_fp64.log")" > "$out"
  g++ -std=c++20 -O3 -ffp-contract=off -I"$REPO/src" -c "$REPO/src/core/glibc_mathf_host.cpp" \
    -o "$WORK/glibc_mathf_host.o" > "$R/proof_fp32.log" 2>&1 &&
    "$cxx" "${f[@]}" -DCPPWRF_GLIBC_MATH -DCPPWRF_FP32_IDENTITY -c "$REPO/tools/glibc_mathf_proof_cuda.cu" \
      -o "$WORK/glibc_mathf_proof_hip.o" >> "$R/proof_fp32.log" 2>&1 &&
    "$cxx" "$WORK/glibc_mathf_proof_hip.o" "$WORK/glibc_mathf_host.o" "${l[@]}" -o "$WORK/glibc_mathf_proof_hip" \
      >> "$R/proof_fp32.log" 2>&1 &&
    { [ "$only" = compile ] || "${run[@]}" "$WORK/glibc_mathf_proof_hip" 28 >> "$R/proof_fp32.log" 2>&1; }
  local r2=$?
  echo "glibc float math proof (FP32)${only:+ $only}: exit $r2: $(tail -1 "$R/proof_fp32.log")" >> "$out"
  [ $r1 = 0 ] && [ $r2 = 0 ] || rc=1
  return $rc
}

# ---- the suite of a flavour, its checks shared out over the GPUs (one wrf_suite.py per GPU). The PASS lines, logs
# and output hashes of a binary are kept under res/suite_<fl>/<its sha256>/: a check that passed with this binary is
# not run again; --redo suite_<fl> starts the binary's records anew
suite_dir() { echo "$R/suite_$1/$(bin_key "b-hip-$1")"; }
passed_of() { cat "$(suite_dir "$1")"/*.log 2> /dev/null | awk '$1 == "PASS" {print $2}' | sort -u; }
suite() {
  local fl=$1 bin=$WORK/b-hip-$1/cppwrf run all todo n sd out wl=""
  sd=$(suite_dir "$fl")
  out=$(res "suite_$fl")
  mkdir -p "$sd"
  all=$(checks_of "$fl") || return 1
  [ -n "$all" ] || { echo "suite $fl: no checks" > "$out"; return 1; }
  todo=$(grep -vx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all")
  run=$(date +%Y%m%d_%H%M%S)
  n=$(grep -c . <<< "$todo")
  log "suite $fl: $(grep -c . <<< "$all") checks, $n to run on $GPUS GPU(s) (${GPU_IDS[*]:0:$GPUS})"
  local -a pids=() wg=()
  if [ "$n" -gt 0 ]; then
    local per=$((NPROC / GPUS)) g mine
    [ $per -ge 1 ] || per=1
    for ((g = 0; g < GPUS; g++)); do
      mine=$(awk -v g=$g -v n="$GPUS" 'NF && (NR - 1) % n == g' <<< "$todo" | paste -sd,)
      [ -n "$mine" ] || continue
      # shellcheck disable=SC2046
      (cd "$REPO" && HIP_VISIBLE_DEVICES=${GPU_IDS[$g]} OMP_NUM_THREADS=$per CPPWRF_WRF_RUNS=$D/wrf_runs \
        CPPWRF_WRF_REF=$D/wrf_ref CPPWRF_SUITE_RUN_TIMEOUT=$((RUN_HOURS * 3600)) \
        python3 tests/verify/wrf_suite.py "$(flag_of "$fl")" "$bin" --exact --only "$mine" --out "$WORK/suite_$fl/g$g" \
          $([ "$fl" = dp ] && echo --hash-out "$sd/hashes_${run}_g$g.tsv") \
        > "$sd/${run}_g$g.log" 2>&1) &
      pids+=($!)
      wg+=("$g")
    done
    local i rc lg
    for i in "${!pids[@]}"; do
      wait "${pids[$i]}"
      rc=$?
      lg=$sd/${run}_g${wg[$i]}.log
      wl+="  worker g${wg[$i]} (HIP_VISIBLE_DEVICES=${GPU_IDS[${wg[$i]}]}): exit $rc, $(grep '^SUITE ' "$lg" | tail -1)"$'\n'
      grep -q '^SUITE ' "$lg" || wl+="$(tail -5 "$lg" | sed 's/^/    | /')"$'\n'  # a worker that died: its last lines
    done
  fi
  local pass miss
  pass=$(grep -cx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all")
  miss=$(grep -vx -F -f <(passed_of "$fl"; echo '#none#') <<< "$all" | grep -c .)
  {
    echo "suite $fl ($(flag_of "$fl"), binary $(bin_key "b-hip-$fl")): $pass of $(grep -c . <<< "$all") checks passed, $miss without a PASS"
    printf '%s' "$wl"
    cat "$sd"/"${run}"_g*.log 2> /dev/null | grep -E '^(FAIL|SKIP|ERROR)( |:)|Traceback|Error:' | cut -c1-400
  } > "$out"
  [ "$miss" = 0 ]
}

# ---- HIP FP64 against CUDA FP64, directly: the hashes of every variable of every output file of every passing
# check (the CPPWRF-only variables and the frames WRF's files lack included) against the CUDA build's
cross() {
  local sd ref out
  sd=$(suite_dir dp)
  out=$(res cross_dp)
  case $CUDA_HASHES in
    http://* | https://*)
      curl -fsSL --retry 5 -o "$WORK/cuda_dp_hashes" "$CUDA_HASHES" ||
        { echo "HIP vs CUDA FP64: cannot fetch $CUDA_HASHES" > "$out"; return 1; }
      ref=$WORK/cuda_dp_hashes
      [[ $CUDA_HASHES == *.gz ]] && { mv "$ref" "$ref.gz"; ref=$ref.gz; } ;;
    *) ref=$CUDA_HASHES ;;
  esac
  cat "$sd"/hashes_*.tsv > "$WORK/hip_dp_hashes.tsv" 2> /dev/null
  [ -s "$WORK/hip_dp_hashes.tsv" ] || { echo "HIP vs CUDA FP64: no HIP hashes (no passing check)" > "$out"; return 1; }
  python3 "$REPO/tests/verify/out_hashes.py" compare "$ref" "$WORK/hip_dp_hashes.tsv" > "$R/cross_dp.log" 2>&1
  local rc=$?
  { echo "HIP vs CUDA FP64 directly (every variable of every output file; reference $(basename "$CUDA_HASHES")): $([ $rc = 0 ] && echo PASS || echo "FAIL (exit $rc)")"
    grep -E '^(HASHES|DIFFER|MISSING|EXTRA)' "$R/cross_dp.log" | head -40; } > "$out"
  return $rc
}

summary() {
  {
    echo "== CPPWRF AMD HIP run, $(date -u '+%F %T') UTC"
    cat "$R/env.txt" 2> /dev/null
    echo "== results of the steps"
    for s in $STEPS; do [ -s "$(res "$s")" ] && cat "$(res "$s")"; done
    if [ "$COMPILE_ONLY" = 0 ] && [[ ",$FLAV," == *,dp,* ]] && [ "$CROSS" = 0 ]; then
      echo "HIP vs CUDA FP64 directly: NOT CHECKED (--no-cross): equal only where both equal WRF (its variables and frames)"
    fi
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
GPUS_ASKED=$GPUS
step deps deps deps || { summary 1; exit 1; }
select_cases || { summary 1; exit 2; }
[ -n "$FLAV" ] || { log "no flavour left"; summary 1; exit 2; }
if [ -z "$ARCH" ]; then
  ARCH=$(rocminfo 2> /dev/null | grep -o -m1 'gfx[0-9a-f]\+')
  [ -n "$ARCH" ] || { log "no AMD GPU found by rocminfo: pass --arch (e.g. gfx942) with --compile-only"; summary 1; exit 1; }
fi
gpu_ids
if [ "$COMPILE_ONLY" = 0 ]; then
  [ ${#GPU_IDS[@]} -ge 1 ] || { log "no GPU of this job (rocminfo, HIP_VISIBLE_DEVICES)"; summary 1; exit 1; }
  GPUS=${GPUS_ASKED:-${#GPU_IDS[@]}}
  [ "$GPUS" -le ${#GPU_IDS[@]} ] || { log "--gpus $GPUS: the job has ${#GPU_IDS[@]} GPU(s) (${GPU_IDS[*]})"; summary 1; exit 2; }
  if [[ ",$FLAV," == *,dp,* ]] && [ "$CROSS" = 1 ] && [ -z "$CUDA_HASHES" ]; then
    log "no --cuda-hashes: the HIP FP64 outputs cannot be compared with CUDA's (pass --no-cross to run without it)"
    summary 1
    exit 2
  fi
else
  GPUS=0
fi
rm -f "$WORK/state/env"
step env env env_txt
log "arch $ARCH, GPUs ${GPU_IDS[*]:-none} ($GPUS for the suite), flavours $FLAV, work $WORK"
for s in ${REDO//,/ }; do  # --redo suite_<fl>: that binary's PASS lines and hashes too
  case $s in
    suite_dp | suite_id | suite_sp)
      if [ -x "$WORK/b-hip-${s#suite_}/cppwrf" ]; then
        d=$(suite_dir "${s#suite_}")
        [ -d "$d" ] && mkdir -p "$d/../old" && mv "$d" "$d/../old/$(basename "$d")_$(date +%s)"
      fi ;;
  esac
done
DPID= dkey=
if [ "$COMPILE_ONLY" = 0 ] && [ "$DATA" = 1 ]; then
  STEPS="$STEPS data"
  data_list || { summary 1; exit 1; }
  dkey=$(echo "$DATA_REPO $TARS $V05 $NAMES" | sha256sum | cut -c1-16)
  if ! [ -e "$WORK/state/data" ] || [ "$(cat "$WORK/state/data")" != "$dkey" ]; then
    (step data "$dkey" data) &  # the data download runs while the programs build
    DPID=$!
  fi
fi
BUILDS=""
for f in ${FLAV//,/ }; do BUILDS="$BUILDS b-hip-$f"; done
[ "$CPU" = 1 ] && BUILDS="$BUILDS b-cpu-dp"
for b in $BUILDS; do
  STEPS="$STEPS build_$b"
  step "build_$b" "$(build_key "$b")" build "$b" || fail=1
done
if [ -n "$DPID" ]; then wait "$DPID" || fail=1; fi
if [ "$COMPILE_ONLY" = 1 ] && [ "$PROOFS" = 1 ]; then
  STEPS="$STEPS proofs_compile"
  step proofs_compile "$(proof_key)" proofs compile || fail=1
fi
built() { [ -e "$WORK/state/build_$1" ] && [ "$(cat "$WORK/state/build_$1")" = "$(build_key "$1")" ]; }
if [ "$COMPILE_ONLY" = 0 ]; then
  [ "$DATA" = 1 ] && [ "$(cat "$WORK/state/data" 2> /dev/null)" != "$dkey" ] && fail=1
  if [ "$CTEST" = 1 ]; then
    for f in ${FLAV//,/ }; do
      STEPS="$STEPS ctest_b-hip-$f"
      if built "b-hip-$f"; then step "ctest_b-hip-$f" "$(bin_key "b-hip-$f" all)${CASES:+ cases}" ctest_of "b-hip-$f" || fail=1
      else fail=1; fi
    done
  fi
  if [ "$PROOFS" = 1 ]; then
    STEPS="$STEPS proofs"
    step proofs "$(proof_key)" proofs || fail=1
  fi
  for f in ${FLAV//,/ }; do
    STEPS="$STEPS suite_$f"
    if built "b-hip-$f"; then step "suite_$f" "$(bin_key "b-hip-$f") $(checks_of "$f" | sha256sum | cut -c1-16)" suite "$f" || fail=1
    else fail=1; fi
  done
  if [[ ",$FLAV," == *,dp,* ]] && [ "$CROSS" = 1 ]; then
    STEPS="$STEPS cross_dp"
    if [ -e "$WORK/state/suite_dp" ]; then
      step cross_dp "$(bin_key b-hip-dp) $CUDA_HASHES $(sha "$CUDA_HASHES")" cross || fail=1
    else
      echo "HIP vs CUDA FP64 directly: not compared (suite dp did not pass)" > "$(res cross_dp)"
      fail=1
    fi
  fi
fi
summary $fail
exit $fail
