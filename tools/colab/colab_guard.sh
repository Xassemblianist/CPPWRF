# SPDX-License-Identifier: GPL-3.0-or-later
# The one way a CPPWRF tool asks Google Colab for a runtime (`colab new`). Sourced by tools/colab/colab_lib.sh,
# tools/colab/colab_run.sh, tools/colab/drive_sync.sh and tools/bench/colab_paper.sh; nothing else may call
# `colab new` or `colab run` (which also asks for a runtime); tests/tools/test_colab_guard.py checks the repository. Automated, repeated session requests (a retry every 15 min for 12 h after "Service Unavailable") got the
# account's Colab runtimes blocked "on suspicion of abusive activity" once; this guard makes that impossible:
#
#   colab_guard_new SESSION [ARG...]   `timeout 900 colab new -s SESSION ARG...`, at most once, or refuse:
#     - ONE attempt per call and no retry: any failure (non-zero exit, or output with 502/503/504 / unavailable /
#       quota / refused / rejected / error / failed / traceback) prints why and returns 1; the caller then exits.
#       The CLI's own assignment retries are switched off (COLAB_CLI_ASSIGN_RETRIES=0).
#     - a rate limit shared by every tool and shell of this user: each attempt (also a failed one) is written to
#       $COLAB_GUARD_DIR/attempts; a new attempt is refused while there was one in the last COLAB_GUARD_MIN_GAP_S
#       seconds or COLAB_GUARD_MAX_PER_DAY in the last 24 h. CPPWRF_COLAB_FORCE=1, set by a person for one command,
#       lets that one call through the rate limit (never through the lock below); the variable is cleared after the
#       call, and at most COLAB_GUARD_MAX_FORCED_PER_DAY forced attempts are allowed in 24 h.
#     - output with "abusive", "blocked" or "suspicion" writes $COLAB_GUARD_DIR/BLOCKED: every later request of every
#       tool is refused until a person has read Colab's message and deleted that file.
#     - the session is registered before `colab new` runs and the shell's EXIT trap (kept as the caller set it, then
#       extended) stops it on any exit, also when the request failed or timed out half way, or the shell got SIGTERM
#       (bash runs the EXIT trap then). KEEP=1 (the tools' --keep) leaves a session that came up running.
#   colab_guard_status                 the lock and the attempts of the last 24 h
#   tools/colab/colab_guard.sh status  the same from the command line
#
# Callers set their own EXIT trap (if any) BEFORE colab_guard_new. Retries are allowed only for operations on a
# session that is already up (uploads, downloads, exec), and only a few times.

COLAB_GUARD_MIN_GAP_S=600          # at most one runtime request per 10 minutes
COLAB_GUARD_MAX_PER_DAY=6          # at most 6 runtime requests per 24 hours
COLAB_GUARD_MAX_FORCED_PER_DAY=3   # of which at most 3 forced by CPPWRF_COLAB_FORCE=1 past the two limits above
COLAB_GUARD_ASSIGN_TIMEOUT_S=900
# the state of all tools of this user (CPPWRF_COLAB_GUARD_DIR only for the offline tests)
COLAB_GUARD_DIR=${CPPWRF_COLAB_GUARD_DIR:-$HOME/.cache/cppwrf/colab}
COLAB_GUARD_SESSIONS=()

_cg_say() { echo "[colab_guard] $*" >&2; }

colab_guard_status() {
  local now; now=$(date +%s)
  if [ -e "$COLAB_GUARD_DIR/BLOCKED" ]; then
    _cg_say "LOCKED: $COLAB_GUARD_DIR/BLOCKED exists, no runtime requests until a person deletes it:"
    sed 's/^/    /' "$COLAB_GUARD_DIR/BLOCKED" >&2
  else _cg_say "no lock"; fi
  _cg_say "attempts in the last 24 h (limits: 1 per $((COLAB_GUARD_MIN_GAP_S / 60)) min, $COLAB_GUARD_MAX_PER_DAY per 24 h):"
  [ -f "$COLAB_GUARD_DIR/attempts" ] &&
    awk -v now="$now" '$1 > now - 86400' "$COLAB_GUARD_DIR/attempts" | while read -r t s f; do
      echo "    $(date -d "@$t" '+%F %T') $s $f" >&2; done
  return 0
}

# 0: the attempt may run (and is recorded); 1: refused. Called with the state file's lock held.
_cg_admit() {
  local s=$1 forced=$2 now last n nf
  now=$(date +%s)
  touch "$COLAB_GUARD_DIR/attempts"
  last=$(awk '{t = $1} END {print t + 0}' "$COLAB_GUARD_DIR/attempts")
  n=$(awk -v now="$now" '$1 > now - 86400' "$COLAB_GUARD_DIR/attempts" | wc -l)
  nf=$(awk -v now="$now" '$1 > now - 86400 && $3 == "forced"' "$COLAB_GUARD_DIR/attempts" | wc -l)
  if [ "$forced" = forced ]; then
    if [ "$nf" -ge "$COLAB_GUARD_MAX_FORCED_PER_DAY" ]; then
      _cg_say "REFUSED: already $nf forced runtime requests in the last 24 h (limit $COLAB_GUARD_MAX_FORCED_PER_DAY)"; return 1
    fi
    _cg_say "CPPWRF_COLAB_FORCE=1: past the rate limit for this one request"
  elif [ $((now - last)) -lt "$COLAB_GUARD_MIN_GAP_S" ]; then
    _cg_say "REFUSED: the last runtime request was $((now - last)) s ago (at most one per $COLAB_GUARD_MIN_GAP_S s)."
    _cg_say "Do not retry in a loop. A person may override once: CPPWRF_COLAB_FORCE=1 <command>"; return 1
  elif [ "$n" -ge "$COLAB_GUARD_MAX_PER_DAY" ]; then
    _cg_say "REFUSED: $n runtime requests in the last 24 h (limit $COLAB_GUARD_MAX_PER_DAY)."
    _cg_say "Do not retry in a loop. A person may override once: CPPWRF_COLAB_FORCE=1 <command>"; return 1
  fi
  # keep only the last 24 h; the record is written before the request, so a request that hangs still counts
  awk -v now="$now" '$1 > now - 86400' "$COLAB_GUARD_DIR/attempts" > "$COLAB_GUARD_DIR/attempts.new"
  echo "$now $s $forced" >> "$COLAB_GUARD_DIR/attempts.new"
  mv -f "$COLAB_GUARD_DIR/attempts.new" "$COLAB_GUARD_DIR/attempts"
}

colab_guard_on_exit() {
  local s
  [ "${#COLAB_GUARD_SESSIONS[@]}" -gt 0 ] || return 0
  if [ "${KEEP:-0}" = 1 ] && [ "${COLAB_GUARD_UP:-0}" = 1 ]; then
    _cg_say "--keep: session ${COLAB_GUARD_SESSIONS[*]} left running (colab stop -s NAME)"; return 0
  fi
  for s in "${COLAB_GUARD_SESSIONS[@]}"; do
    timeout 300 colab stop -s "$s" > /dev/null 2>&1 && _cg_say "stopped $s" ||
      _cg_say "could not stop $s (already stopped or never created; check: colab sessions)"
  done
  COLAB_GUARD_SESSIONS=()
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

colab_guard_new() {
  local s=$1 forced=normal out rc; shift
  [ -n "$s" ] || { _cg_say "colab_guard_new: no session name"; return 2; }
  if [ "${CPPWRF_COLAB_FORCE:-0}" = 1 ]; then forced=forced; fi
  unset CPPWRF_COLAB_FORCE   # one call only, also for later calls of the same shell and its children
  mkdir -p "$COLAB_GUARD_DIR"
  if [ -e "$COLAB_GUARD_DIR/BLOCKED" ]; then
    _cg_say "REFUSED: $COLAB_GUARD_DIR/BLOCKED exists: Colab reported a block. Nothing asks for a runtime until a"
    _cg_say "person has read Colab's message (contents below) and deleted that file:"
    sed 's/^/    /' "$COLAB_GUARD_DIR/BLOCKED" >&2
    return 1
  fi
  (
    flock -w 30 9 || { _cg_say "REFUSED: could not lock $COLAB_GUARD_DIR/attempts.lock"; exit 1; }
    _cg_admit "$s" "$forced"
  ) 9> "$COLAB_GUARD_DIR/attempts.lock" || return 1
  COLAB_GUARD_SESSIONS+=("$s")
  _cg_install_trap
  _cg_say "runtime request: colab new -s $s $* (one attempt, no retry)"
  # COLAB_CLI_ASSIGN_RETRIES=0: the CLI itself (0.8.0, client.py assign) would POST again up to 3 times on 502/503/504
  rc=0; out=$(COLAB_CLI_ASSIGN_RETRIES=0 timeout "$COLAB_GUARD_ASSIGN_TIMEOUT_S" colab new -s "$s" "$@" 2>&1) || rc=$?
  [ -z "$out" ] || printf '%s\n' "$out" >&2
  if printf '%s' "$out" | grep -qiE 'abusive|blocked|suspicion'; then
    { date '+%F %T'; echo "session $s, colab new $*, exit $rc"; printf '%s\n' "$out" | tail -20; } > "$COLAB_GUARD_DIR/BLOCKED"
    _cg_say "Colab reports a BLOCK. Wrote $COLAB_GUARD_DIR/BLOCKED: every runtime request is refused until a person"
    _cg_say "has dealt with it and deleted that file. Do not retry."
    return 1
  fi
  if [ $rc -ne 0 ] || printf '%s' "$out" | grep -qiE '50[234]|unavailable|quota|refused|rejected|error|failed|traceback'; then
    _cg_say "FAILED: colab new -s $s (exit $rc). Not retried; the next request is allowed in $((COLAB_GUARD_MIN_GAP_S / 60)) min at the earliest."
    return 1
  fi
  COLAB_GUARD_UP=1
  return 0
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  case ${1:-} in status) colab_guard_status ;; *) echo "usage: $0 status   (source it for colab_guard_new)" >&2; exit 2 ;; esac
fi
