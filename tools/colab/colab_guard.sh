# SPDX-License-Identifier: GPL-3.0-or-later
# The one way a CPPWRF tool asks Google Colab for a runtime: Google's Colab API through tools/colab/colab_api.py
# (`colab_api.py new`). Sourced by tools/colab/colab_lib.sh, tools/colab/colab_run.sh, tools/colab/drive_sync.sh and
# tools/bench/colab_paper.sh; nothing else may call `colab_api.py new` (nor the old `colab new` / `colab run`);
# tests/tools/test_colab_guard.py checks the repository. Automated, repeated runtime requests (a retry every 15 min
# for 12 h after "Service Unavailable") got the account's Colab runtimes blocked "on suspicion of abusive activity"
# once; this guard makes that impossible:
#
#   colab_guard_new SESSION --gpu ACC [--fallback ACC2] [--shape standard|highmem]
#     - GPU runtimes only (Colab is for GPU work; CPU work runs on the workstation gate or on GitHub CI).
#     - at most COLAB_GUARD_MAX_RUNTIMES (2) of this project's runtimes at once (list_runtimes, names
#       runtimes/cppwrf-*; the owner's and CPPMPAS's runtimes are neither counted nor touched); the count and the
#       request run under one lock, so two tools cannot both pass it.
#     - ONE create request per spec and no retry: failure prints why and returns 1. Only when Colab answers "no
#       capacity / quota" (FAILED_PRECONDITION) for ACC, one request for the fallback spec ACC2 follows, and that is
#       the last. No loops, no backoff. (A create that is still running, DEADLINE_EXCEEDED, is waited for with the
#       identical request inside colab_api.py; that never makes a second runtime.)
#     - output with "abusive", "blocked" or "suspicion" writes $COLAB_GUARD_DIR/BLOCKED: every later request of every
#       tool is refused until a person has read Colab's message and deleted that file.
#     - the session is registered before the request and the shell's EXIT trap (kept as the caller set it, then
#       extended) deletes the runtime on any exit: success, failure, an error later, a request that failed half way,
#       SIGTERM (bash runs the EXIT trap then). KEEP=1 (the tools' --keep) leaves a runtime that came up running.
#     On success COLAB_GUARD_GPU is the accelerator the runtime got.
#   colab_guard_attach SESSION [RUNTIME]
#     - no runtime request: use a GPU runtime the owner opened in the Colab web UI (`colab_api.py attach`: RUNTIME,
#       else $CPPWRF_COLAB_RUNTIME, else the only free GPU runtime that is not cppwrf-* / cppmpas-*; refused when
#       there are several or none). Creating G4 / A100 / L4 from the API fails (a Google bug); T4 still works.
#     - our own kernel session cppwrf-SESSION on it; our files only below COLAB_GUARD_ROOT (/content/cppwrf_SESSION).
#     - the EXIT trap (as for colab_guard_new) runs `colab_api.py stop`, which on an attached runtime only kills our
#       processes, removes our directory and closes our session: an attached runtime is NEVER deleted. KEEP=1 leaves
#       our session and files.
#     On success COLAB_GUARD_GPU is its accelerator, COLAB_GUARD_ATTACHED=1.
#   colab_guard_how_to_attach          prints how to open a runtime in the browser
#   colab_guard_status                 the lock, this project's runtimes, the requests of the last 24 h
#   tools/colab/colab_guard.sh status  the same from the command line
# Every request is logged to $COLAB_GUARD_DIR/attempts (for people; it limits nothing).
#
# Callers set their own EXIT trap (if any) BEFORE colab_guard_new. Retries are allowed only for operations on a
# runtime that is already up (uploads, downloads, exec), and only a few times.

COLAB_GUARD_MAX_RUNTIMES=2
COLAB_GUARD_CREATE_TIMEOUT_S=2400
# the client (CPPWRF_COLAB_API only for the offline tests) and the state of all tools of this user
COLAB_API=${CPPWRF_COLAB_API:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/colab_api.py}
COLAB_GUARD_DIR=${CPPWRF_COLAB_GUARD_DIR:-$HOME/.cache/cppwrf/colab}
COLAB_GUARD_SESSIONS=()

_cg_say() { echo "[colab_guard] $*" >&2; }

colab_guard_status() {
  local now; now=$(date +%s)
  if [ -e "$COLAB_GUARD_DIR/BLOCKED" ]; then
    _cg_say "LOCKED: $COLAB_GUARD_DIR/BLOCKED exists, no runtime requests until a person deletes it:"
    sed 's/^/    /' "$COLAB_GUARD_DIR/BLOCKED" >&2
  else _cg_say "no lock"; fi
  _cg_say "this project's runtimes (limit $COLAB_GUARD_MAX_RUNTIMES):"
  timeout 120 "$COLAB_API" own 2>&1 | sed 's/^/    /' >&2 || true
  _cg_say "runtime requests in the last 24 h:"
  [ -f "$COLAB_GUARD_DIR/attempts" ] &&
    awk -v now="$now" '$1 > now - 86400' "$COLAB_GUARD_DIR/attempts" | while read -r t s f; do
      echo "    $(date -d "@$t" '+%F %T') $s $f" >&2; done
  return 0
}

colab_guard_on_exit() {
  local s
  [ "${#COLAB_GUARD_SESSIONS[@]}" -gt 0 ] || return 0
  if [ "${KEEP:-0}" = 1 ] && [ "${COLAB_GUARD_UP:-0}" = 1 ]; then
    _cg_say "--keep: runtime ${COLAB_GUARD_SESSIONS[*]} left running ($COLAB_API stop -s NAME)"; return 0
  fi
  for s in "${COLAB_GUARD_SESSIONS[@]}"; do
    case " ${COLAB_GUARD_ATTACHING:-} " in *" $s "*)  # an attach: nothing to do without its record
      [ -e "$COLAB_GUARD_DIR/attached/$(_cg_rid "$s")" ] || continue ;; esac
    if [ -e "$COLAB_GUARD_DIR/attached/$(_cg_rid "$s")" ]; then  # stop only detaches there
      timeout 600 "$COLAB_API" stop -s "$s" > /dev/null 2>&1 && _cg_say "detached $s (the runtime stays up)" ||
        _cg_say "could not clean up $s on the attached runtime: check /content/cppwrf_* there"
    else
      timeout 600 "$COLAB_API" stop -s "$s" > /dev/null 2>&1 && _cg_say "deleted runtime $s" ||
        _cg_say "could not delete runtime $s: check with $COLAB_API own"
    fi
  done
  COLAB_GUARD_SESSIONS=()
}

_cg_rid() {  # colab_api.py's runtime_id
  local r; r=$(printf '%s' "$1" | tr 'A-Z' 'a-z' | sed 's/[^a-z0-9-]/-/g; s/^-*//; s/-*$//')
  case $r in cppwrf-*) ;; *) r=cppwrf-$r ;; esac
  r=${r:0:63}; printf '%s' "${r%%-}"
}

_cg_install_trap() {
  local prev
  prev=$(trap -p EXIT)
  case $prev in *colab_guard_on_exit*) ;; *)
    if [ -n "$prev" ]; then
      eval "set -- $prev"      # trap -- 'COMMAND' EXIT
      trap -- "$3"$'\n'"colab_guard_on_exit" EXIT
    else trap -- colab_guard_on_exit EXIT; fi ;;
  esac
}

_cg_blocked() {  # OUTPUT SESSION ACC: Colab's block message -> the lock
  printf '%s' "$1" | grep -qiE 'abusive|blocked|suspicion' || return 1
  { date '+%F %T'; echo "runtime $2, $3"; printf '%s\n' "$1" | tail -20; } > "$COLAB_GUARD_DIR/BLOCKED"
  _cg_say "Colab reports a BLOCK. Wrote $COLAB_GUARD_DIR/BLOCKED: every runtime request is refused until a person"
  _cg_say "has dealt with it and deleted that file. Do not retry."
}

# under the lock: the runtime limit, then one request per spec; 0: up (the accelerator in $COLAB_GUARD_DIR/gpu.PID)
_cg_request() {
  local s=$1 gpu=$2 fb=$3 shape=$4 pid=$5 n out rc acc
  n=$(timeout 300 "$COLAB_API" own 2> "$COLAB_GUARD_DIR/own.err" | head -1) || n=
  case $n in ''|*[!0-9]*)
    _cg_say "REFUSED: could not count this project's runtimes:"; cat "$COLAB_GUARD_DIR/own.err" >&2
    _cg_blocked "$(cat "$COLAB_GUARD_DIR/own.err")" "$s" list || true
    return 1 ;;
  esac
  if [ "$n" -ge "$COLAB_GUARD_MAX_RUNTIMES" ]; then
    _cg_say "REFUSED: $n of this project's runtimes are up (limit $COLAB_GUARD_MAX_RUNTIMES); see $COLAB_API own"
    return 1
  fi
  for acc in "$gpu" $fb; do
    echo "$(date +%s) $s $acc" >> "$COLAB_GUARD_DIR/attempts"
    _cg_say "runtime request: $s $acc (one request, no retry)"
    rc=0; out=$(timeout "$COLAB_GUARD_CREATE_TIMEOUT_S" "$COLAB_API" new -s "$s" --gpu "$acc" --shape "$shape" 2>&1) || rc=$?
    [ -z "$out" ] || printf '%s\n' "$out" >&2
    if _cg_blocked "$out" "$s" "$acc"; then return 1; fi
    if [ $rc -eq 0 ]; then echo "$acc" > "$COLAB_GUARD_DIR/gpu.$pid"; return 0; fi
    if [ $rc -ne 3 ]; then _cg_say "FAILED: runtime $s $acc (exit $rc). Not retried."; return 1; fi
    _cg_say "no $acc capacity / quota${fb:+ (fallback: one request for $fb)}"
    [ "$acc" != "$fb" ] || break
  done
  _cg_say "FAILED: no runtime for $s. Not retried."
  return 1
}

colab_guard_new() {
  local s=$1 gpu= fb= shape=standard rc; shift
  [ -n "$s" ] || { _cg_say "colab_guard_new: no session name"; return 2; }
  while [ $# -gt 0 ]; do
    case $1 in
      --gpu) gpu=$2; shift 2 ;; --fallback) fb=$2; shift 2 ;; --shape) shape=$2; shift 2 ;;
      *) _cg_say "colab_guard_new: unknown argument $1"; return 2 ;;
    esac
  done
  case ${gpu^^} in ''|CPU|NONE)
    _cg_say "REFUSED: Colab is for GPU work only (no CPU runtimes): use the gate or GitHub CI"; return 1 ;;
  esac
  [ "${fb^^}" != "${gpu^^}" ] || fb=
  mkdir -p "$COLAB_GUARD_DIR"
  if [ -e "$COLAB_GUARD_DIR/BLOCKED" ]; then
    _cg_say "REFUSED: $COLAB_GUARD_DIR/BLOCKED exists: Colab reported a block. Nothing asks for a runtime until a"
    _cg_say "person has read Colab's message (contents below) and deleted that file:"
    sed 's/^/    /' "$COLAB_GUARD_DIR/BLOCKED" >&2
    return 1
  fi
  # registered before the request: the trap deletes a runtime that came up half way too
  COLAB_GUARD_SESSIONS+=("$s")
  _cg_install_trap
  rc=0
  (
    flock -w "$COLAB_GUARD_CREATE_TIMEOUT_S" 9 || { _cg_say "REFUSED: could not lock $COLAB_GUARD_DIR/request.lock"; exit 1; }
    _cg_request "$s" "$gpu" "$fb" "$shape" $$
  ) 9> "$COLAB_GUARD_DIR/request.lock" || rc=$?
  if [ $rc -ne 0 ]; then rm -f "$COLAB_GUARD_DIR/gpu.$$"; return 1; fi
  COLAB_GUARD_GPU=$(cat "$COLAB_GUARD_DIR/gpu.$$"); rm -f "$COLAB_GUARD_DIR/gpu.$$"
  COLAB_GUARD_UP=1
  _cg_say "runtime $s is up ($COLAB_GUARD_GPU)"
  return 0
}

colab_guard_how_to_attach() {
  _cg_say "Creating G4 / A100 / L4 runtimes from the API fails (a Google bug: google-colab-cli issues 236, 273, 282,"
  _cg_say "292). Open one in the browser and attach to it: colab.research.google.com > any notebook > Runtime >"
  _cg_say "Change runtime type > GPU (G4) > Connect; then run the tool with --attach (or --attach RUNTIME_ID from"
  _cg_say "\`$COLAB_API runtimes\`). Only T4 may be created: --gpu T4."
}

colab_guard_attach() {
  local s=$1 want=${2:-} out rc=0
  [ -n "$s" ] || { _cg_say "colab_guard_attach: no session name"; return 2; }
  mkdir -p "$COLAB_GUARD_DIR"
  if [ -e "$COLAB_GUARD_DIR/BLOCKED" ]; then
    _cg_say "REFUSED: $COLAB_GUARD_DIR/BLOCKED exists: Colab reported a block. A person must read it and delete it:"
    sed 's/^/    /' "$COLAB_GUARD_DIR/BLOCKED" >&2
    return 1
  fi
  # registered first: the trap cleans up after a half-done attach too (without its record nothing is done remotely)
  COLAB_GUARD_SESSIONS+=("$s")
  COLAB_GUARD_ATTACHING="${COLAB_GUARD_ATTACHING:-} $s"
  _cg_install_trap
  out=$(timeout 900 "$COLAB_API" attach -s "$s" ${want:+--runtime "$want"} 2>&1) || rc=$?
  [ -z "$out" ] || printf '%s\n' "$out" >&2
  if _cg_blocked "$out" "$s" attach; then return 1; fi
  if [ $rc -ne 0 ]; then
    [ $rc -ne 4 ] || colab_guard_how_to_attach
    _cg_say "FAILED: no runtime to attach for $s (exit $rc)"; return 1
  fi
  COLAB_GUARD_GPU=$(printf '%s\n' "$out" | sed -n 's/^Accelerator: //p')
  COLAB_GUARD_ROOT=$(printf '%s\n' "$out" | sed -n 's/^Root: //p')
  COLAB_GUARD_RUNTIME=$(printf '%s\n' "$out" | sed -n 's|^Session READY: \(runtimes/[^/]*\)/.*|\1|p')
  COLAB_GUARD_UP=1 COLAB_GUARD_ATTACHED=1
  _cg_say "attached $s to $COLAB_GUARD_RUNTIME ($COLAB_GUARD_GPU), our files in $COLAB_GUARD_ROOT; never deleted here"
  return 0
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  case ${1:-} in status) colab_guard_status ;; *) echo "usage: $0 status   (source it for colab_guard_new)" >&2; exit 2 ;; esac
fi
