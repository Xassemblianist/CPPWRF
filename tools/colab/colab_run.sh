#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Run CPPWRF's GPU suite cases (and timings) on a Google Colab GPU runtime through Google's Colab API
# (tools/colab/colab_api.py; tools/colab/colab_guard.sh asks for the runtime). A created runtime is always deleted at the
# end; an attached one is never deleted.
#
#   tools/colab/colab_run.sh --attach [RUNTIME] | --gpu T4 | --gpu L4|G4|A100|H100 --create
#                            [--fallback T4|L4|G4|A100|H100] (one request for it when --gpu has no capacity)
#                            [--cases NAME,...] [--files NAME,...] [--bench NAME[@cuda|@cuda_dp],...]
#                            [--env K=V]... [--nsys] [--ncu REGEX|top] [--base REV] [--device-mib N]
#                            [--wrf-tables] [--keep] [--out DIR]
#
#   --attach a GPU runtime the owner opened in the Colab web UI (G4 is the fastest per credit): RUNTIME (an ID from
#            `tools/colab/colab_api.py runtimes`), else $CPPWRF_COLAB_RUNTIME, else the only free one (see
#            colab_guard_attach). Our files go to /content/cppwrf_<session> there and are removed at the end with our
#            session and processes; the runtime stays up.
#   --gpu    create a runtime from the API: only T4 works (creating G4 / A100 / L4 / H100 fails, a Google bug);
#            --create asks for another accelerator anyway (one request, as always). Neither --attach nor --gpu:
#            prints how to open a runtime in the browser.
#   --cases  wrf_suite.py cases to run on the VM against the WRF references (default: the GPU core set below;
#            '' for none: the data then have no reference files)
#   --files  run_wrf_files_test.sh cases to run with the CUDA FP64 build (files' structure and times; default none)
#   --bench  suite cases to run again without history output for the stepping time (default: P1 and D0);
#            NAME@cuda / NAME@cuda_dp: with the FP32 / FP64 build instead of the case's own
#   --env    an environment variable for the model runs (repeatable, e.g. CPPWRF_KTIME=1): the suite cases run
#            with it, each bench case once without it (the stepping time) and once with it
#   --nsys   each bench case also under nsys, for its time and for half of it: per step of the time loop the
#            kernel launches and time, copies, synchronisations, allocations; the top 15 kernels (job/profile/)
#   --ncu    ncu on the bench cases' kernels whose mangled names match REGEX (e.g. 'KTendU|YSURunner'; "top": the
#            nsys top 15) in their first 2 steps (at most 500 launches): achieved occupancy, local-memory traffic,
#            DRAM bytes, registers and stack
#   --base   also build revision REV (git archive) and run every bench case with both builds in the same session:
#            the stepping time twice each, alternating, the device memory and the stack reserve of both, the kernel
#            times of both with --env CPPWRF_KTIME=1, and the case's own outputs (history every interval of its
#            configuration) of both builds compared byte for byte
#   --device-mib  a second process holds device memory so that N MiB are free for every run (a smaller GPU's memory
#            on this one: 6000 for a 6 GB card; the resident threads, and with them the stack reserve, stay this GPU's)
#   --wrf-tables  upload WRF's Thompson tables (370 MB) instead of computing them on the VM: the Thompson cases
#            (mp_physics 8 / 28) are then bit-identical to WRF in FP64
#   --keep   leave the runtime running (delete it later with `tools/colab/colab_api.py stop -s <session>`)
#   --out    local results directory (default: ${CPPWRF_SCRATCH:-~/cppwrf_scratch}/colab/<date>_<gpu>)
#   --release-data  the VM fetches the cases' data tars of the cppwrf-data release (wrf_suite.py --data-for, SHA256SUMS
#            checked, as the Kaggle proof) instead of staging them from ${CPPWRF_WRF_RUNS} and uploading them
#   MAX_UPLOAD_MIN (environment, default 20): stop when the upload would take longer (slow regions)
#
# The source is the committed HEAD (git archive); the data come from ${CPPWRF_WRF_RUNS} through
# tools/colab/stage_data.py (cached per revision and case list). Results: summary.txt (GPU, build times, the device math proof
# (tools/glibc_math_proof_cuda.cu, 10^7 inputs per function; --env CPPWRF_JOB_GLIBC_PROOF=N), PASS/FAIL lines
# with the relative L2 differences, ns/cell/step, device memory, the nsys / ncu lines), the logs, profile/ (per
# bench case the nsys tables per step and the ncu kernel table) and nsys/ (nsys stats as CSV).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
. "$REPO/tools/colab/colab_guard.sh"  # the only way to ask for a runtime: one request per spec, 2 runtimes at most, block lock
SCRATCH=${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}
GPU= ATTACH= ATTACH_RT= CREATE= FALLBACK= KEEP=0 OUT= FILES= NSYS= NCU= BASE= DEVMIB= ENVS=() KEYS= MAX_UPLOAD_MIN=${MAX_UPLOAD_MIN:-20} TABLES=--no-thompson-tables WT= RELDATA=
CASES=real_r1_fp64_cuda,real_p1_r8_fp64_cuda_1h,real_p1_fp32_1h,real_p4_r8_fp64_cuda_1h,real_p8_r8_fp64_cuda_1h,real_t6_r8_fp64_cuda_1h,real_d0_r8_fp64_cuda_1h,real_nmp1_r8_fp64_cuda_1h
BENCH=real_p1_r8_fp64_cuda_1h,real_p1_fp32_1h,real_d0_r8_fp64_cuda_1h
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) GPU=$2; shift 2 ;;
    --attach) ATTACH=1; shift; case ${1:-} in ''|--*) ;; *) ATTACH_RT=$1; shift ;; esac ;;
    --create) CREATE=1; shift ;;
    --fallback) FALLBACK=$2; shift 2 ;;
    --cases) CASES=$2; shift 2 ;;
    --files) FILES=$2; shift 2 ;;
    --bench) BENCH=$2; shift 2 ;;
    --env) case $2 in [A-Za-z_]*=*) ;; *) echo "--env needs K=V, not '$2'"; exit 2 ;; esac
           ENVS+=(--env "$2"); KEYS=${KEYS:+$KEYS,}${2%%=*}; shift 2 ;;
    --nsys) NSYS=1; shift ;;
    --ncu) NCU=$2; shift 2 ;;
    --base) BASE=$2; shift 2 ;;
    --device-mib) DEVMIB=$2; shift 2 ;;
    --wrf-tables) TABLES= WT=wrf-tables; shift ;;
    --keep) KEEP=1; shift ;;
    --release-data) RELDATA=1; shift ;;
    --out) OUT=$2; shift 2 ;;
    *) sed -n '3,44p' "$0"; exit 2 ;;
  esac
done
if [ -n "$ATTACH" ]; then
  [ -z "$GPU$FALLBACK$CREATE" ] || { echo "--attach takes no --gpu / --fallback / --create"; exit 2; }
else
  case $GPU in
    '') colab_guard_how_to_attach; exit 2 ;;
    T4) ;;
    *) [ -n "$CREATE" ] || { echo "--gpu $GPU: creating it from the API fails (a Google bug); --create to ask anyway"
                             colab_guard_how_to_attach; exit 2; } ;;
  esac
  for g in "$GPU" $FALLBACK; do case $g in T4|L4|G4|A100|H100) ;; *) echo "unknown GPU $g"; exit 2 ;; esac; done
fi
[ "$NCU" != top ] || [ -n "$NSYS" ] || { echo "--ncu top needs --nsys"; exit 2; }
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-$SCRATCH/colab/${STAMP}_${GPU:-attach}}
S=cppwrf-$(echo "${GPU:-attach}" | tr A-Z a-z)-$STAMP
mkdir -p "$OUT" "$SCRATCH/colab/cache"

# ---- local: source and data
[ -z "$(git -C "$REPO" status --porcelain --untracked-files=no)" ] || echo "note: uncommitted changes are not sent (HEAD only)"
git -C "$REPO" archive --format=tar.gz -o "$OUT/src.tar.gz" HEAD
git -C "$REPO" rev-parse --short HEAD > "$OUT/commit.txt"
if [ -n "$BASE" ]; then
  git -C "$REPO" archive --format=tar.gz -o "$OUT/base.tar.gz" "$BASE"
  echo "base $(git -C "$REPO" rev-parse --short "$BASE")" >> "$OUT/commit.txt"
fi
STAGE=$(echo "$CASES,$BENCH" | sed 's/@cuda[a-z_]*//g')
# no --cases and no --files: the bench cases need no reference files
NOREFS=; [ -n "$CASES$FILES" ] || NOREFS=--no-refs
key=$( (echo "$STAGE" | tr ',' '\n' | sort -u; echo "files:$FILES$NOREFS$WT";
         git -C "$REPO" rev-parse HEAD; sha256sum "$REPO/tools/colab/stage_data.py") | tr '\n' ',' | sha256sum | cut -c1-16)
DATA=$SCRATCH/colab/cache/data_$key.tar
# GPU runs only: CPPWRF computes the Thompson tables on the VM instead of uploading WRF's 370 MB (--wrf-tables: WRF's)
DATA_URL=https://github.com/Xassemblianist/cppwrf-data/releases/download/data-v1
if [ -n "$RELDATA" ]; then
  RELTARS=$(cd "$REPO" && python3 tests/verify/wrf_suite.py --data-for "$STAGE")
  echo "[colab_run] release data: $RELTARS"
else
  : "${CPPWRF_WRF_RUNS:?set CPPWRF_WRF_RUNS (or --release-data)}"
fi
if [ -z "$RELDATA" ] && [ ! -f "$DATA" ]; then
  staging=$(mktemp "$DATA.XXXXXX")
  if python3 "$REPO/tools/colab/stage_data.py" --only "$STAGE" --files "$FILES" $TABLES $NOREFS "$staging"; then
    mv -f "$staging" "$DATA"
  else
    rm -f "$staging"
    exit 1
  fi
fi

# ---- remote
timeout 120 $COLAB_API usage | tail -3 || true
echo "[colab_run] session $S (${GPU:-attach})"
cleanup() {
  [ -z "$ATTACH" ] || [ "${COLAB_GUARD_ATTACHED:-0}" = 1 ] || return 0  # no attach happened: nothing of ours remote
  if [ "$KEEP" = 1 ]; then echo "[colab_run] --keep: session $S left running ($COLAB_API stop -s $S)"
  else timeout 300 $COLAB_API stop -s "$S" || echo "[colab_run] WARNING: could not stop $S, check: $COLAB_API sessions"; fi
}
trap cleanup EXIT
if [ -n "$ATTACH" ]; then
  colab_guard_attach "$S" "$ATTACH_RT" || exit 1
  RR=$COLAB_GUARD_ROOT  # our directory on the owner's runtime
  [ -n "$RR" ] || { echo "[colab_run] attach gave no directory"; exit 1; }
  echo "[colab_run] runtime $S: $COLAB_GUARD_GPU (attached to $COLAB_GUARD_RUNTIME, files in $RR)" | tee "$OUT/runtime.txt"
else
  colab_guard_new "$S" --gpu "$GPU" ${FALLBACK:+--fallback "$FALLBACK"} || exit 1
  RR=/content
  echo "[colab_run] runtime $S: $COLAB_GUARD_GPU" | tee "$OUT/runtime.txt"
fi
# uploads go through Jupyter's contents API, which refuses large files (and `tools/colab/colab_api.py upload` exits 0 anyway):
# send 32 MiB parts, join them on the VM and check the md5 (each part is cut just before it is sent: the disk
# never holds a second copy of the data)
upload() {
  local src=$1 dst=$2 parts p n=0 sum got
  parts=$(mktemp -d "$SCRATCH/colab/parts.XXXXXX")
  local total t0=$SECONDS
  total=$(( ($(stat -c %s "$src") + 33554431) / 33554432 ))
  [ "$total" -gt 0 ] || total=1
  while [ "$n" -lt "$total" ]; do
    p=$parts/p$(printf %04d "$n")
    dd if="$src" of="$p" bs=32M skip="$n" count=1 iflag=fullblock status=none
    if ! timeout 900 $COLAB_API upload -s "$S" "$p" "$dst.part.$(basename "$p")" 2>&1 | grep -q "Uploaded"; then
      echo "[colab_run] upload of part $(basename "$p") of $src failed"; rm -rf "$parts"; return 1
    fi
    rm -f "$p"
    n=$((n + 1))
    # the VM bills while it waits: give up when this region's upload would take too long
    local eta=$(( (SECONDS - t0) * (total - n) / n / 60 ))
    [ "$total" -gt 1 ] && echo "[colab_run] $(basename "$src"): $n/$total parts, ~${eta} min left"
    if [ "$n" -ge 2 ] && [ "$eta" -gt "$MAX_UPLOAD_MIN" ]; then
      echo "[colab_run] upload too slow (~${eta} min left > MAX_UPLOAD_MIN=$MAX_UPLOAD_MIN), giving up"
      rm -rf "$parts"; return 1
    fi
  done
  rm -rf "$parts"
  sum=$(md5sum "$src" | cut -c1-32)
  got=$(printf '%s\n' "import glob, hashlib, os" \
    "parts = sorted(glob.glob('$dst.part.p*'))" \
    "o = open('$dst', 'wb')" \
    "for p in parts: o.write(open(p, 'rb').read()); os.remove(p)" \
    "o.close()" \
    "print('MD5', hashlib.md5(open('$dst', 'rb').read()).hexdigest())" | timeout 900 $COLAB_API exec -s "$S" --timeout 600 || true)
  case $got in
    *"MD5 $sum"*) echo "[colab_run] uploaded $src ($n parts)" ;;
    *) echo "[colab_run] $dst: md5 mismatch"; return 1 ;;
  esac
}
upload "$OUT/src.tar.gz" "$RR/src.tar.gz" || exit 1
[ -z "$BASE" ] || upload "$OUT/base.tar.gz" "$RR/base.tar.gz" || exit 1
[ -n "$RELDATA" ] || upload "$DATA" "$RR/data.tar" || exit 1
upload "$OUT/commit.txt" "$RR/commit.txt" || exit 1
upload "$REPO/tools/colab/remote_job.py" "$RR/remote_job.py" || exit 1
fenv=("${ENVS[@]}" --env CPPWRF_JOB_ROOT="$RR"); [ -z "$FILES" ] || fenv+=(--env CPPWRF_JOB_FILES="$FILES")
[ -z "$KEYS" ] || fenv+=(--env CPPWRF_JOB_ENV_KEYS="$KEYS")
[ -z "$NSYS" ] || fenv+=(--env CPPWRF_JOB_NSYS=1)
[ -z "$NCU" ] || fenv+=(--env CPPWRF_JOB_NCU="$NCU")
[ -z "$BASE" ] || fenv+=(--env CPPWRF_JOB_BASE=1)
[ -z "$DEVMIB" ] || fenv+=(--env CPPWRF_JOB_DEVICE_MIB="$DEVMIB")
[ -z "$RELDATA" ] || fenv+=(--env CPPWRF_JOB_DATA_TARS="$RELTARS" --env CPPWRF_JOB_DATA_URL="$DATA_URL")
timeout 300 $COLAB_API exec -s "$S" --timeout 120 --env CPPWRF_JOB_CASES="$CASES" --env CPPWRF_JOB_BENCH="$BENCH" "${fenv[@]}" <<'EOF'
import os, subprocess
env = dict(os.environ)
r = env["CPPWRF_JOB_ROOT"]
subprocess.Popen(f"nohup python3 {r}/remote_job.py > {r}/remote_job.log 2>&1 &", shell=True, env=env)
print("job started")
EOF
# poll: the job writes $RR/job/done (also when it fails); JOB-DIED: no done file and no job process
miss=0
while :; do
  sleep "${CPPWRF_COLAB_POLL_S:-60}"
  st=$(timeout 180 $COLAB_API exec -s "$S" --timeout 60 --env CPPWRF_JOB_ROOT="$RR" <<'EOF' 2>/dev/null || true
import os, subprocess
r = os.environ["CPPWRF_JOB_ROOT"]
alive = subprocess.run(["pgrep", "-f", r + "/remote_job.py"], capture_output=True).returncode == 0
print("JOB-DONE" if os.path.exists(r + "/job/done") else "JOB-RUNNING" if alive else "JOB-DIED")
s = r + "/job/summary.txt"
print(open(s).read().splitlines()[-1] if os.path.exists(s) and open(s).read() else "(starting)")
EOF
)
  echo "[colab_run] $(date +%H:%M) $(echo "$st" | grep -v '^\[colab\]' | tr '\n' ' ')"
  case $st in *JOB-DONE*|*JOB-DIED*) break ;; esac
  case $st in
    *JOB-RUNNING*) miss=0 ;;
    *) miss=$((miss + 1)); echo "[colab_run] session not answering ($miss)"; timeout 120 $COLAB_API sessions || true
       [ $miss -lt 5 ] || { echo "[colab_run] giving up"; exit 1; } ;;
  esac
done
case $st in *JOB-DIED*) echo "[colab_run] the job ended without its done file (killed?)"; exit 1 ;; esac
timeout 1800 $COLAB_API download -s "$S" "$RR/results.tar.gz" "$OUT/results.tar.gz" || { echo "[colab_run] download failed"; exit 1; }
tar xzf "$OUT/results.tar.gz" -C "$OUT"
rm -f "$OUT/src.tar.gz" "$OUT/base.tar.gz"
echo "[colab_run] results in $OUT/job"
cat "$OUT/job/summary.txt"
if grep -qE "^suite (FAIL|SKIP|ERROR)|^files (FAIL|SKIP)|^job error|^glibc device math: exit [1-9]|^ab .*DIFFERENT" "$OUT/job/summary.txt" ||
   { [ -n "$FILES" ] && ! grep -q "^files compared " "$OUT/job/summary.txt"; } ||
   { [ -n "$CASES" ] && ! grep -q "^suite SUITE " "$OUT/job/summary.txt"; }; then
  echo "[colab_run] the suite did not pass"; exit 1
fi
