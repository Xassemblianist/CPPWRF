#!/usr/bin/env bash
# Agent watchdog (Agent 0): every 20 minutes, an agent that is idle with an empty queue and whose
# last turn ended in ERROR (typically the Gemini quota) gets a "continue" message, so work
# resumes by itself once the quota resets. Run: nohup tools/agents/revive.sh >/dev/null 2>&1 &
# Log: ~/cppwrf_agents/revive.log
set -uo pipefail
ST="$HOME/cppwrf_agents"
DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$ST/revive.log"
INTERVAL="${REVIVE_INTERVAL:-1200}"
while true; do
  for n in $(ls "$ST" | sed -n 's/^agent\([0-9]*\)\.cid$/\1/p' | sort -un); do
    exec 7>"$ST/agent$n.lock"
    if ! flock -n 7; then continue; fi  # a worker is running
    flock -u 7
    [ -s "$ST/agent$n.queue" ] && continue
    [ -s "$ST/agent$n.last" ] || continue
    if head -c 40 "$ST/agent$n.last" | grep -qE "^(ERROR|UNPARSED)"; then
      echo "$(date '+%F %T') reviving Agent $n" >> "$LOG"
      "$DIR/agent.sh" send "$n" "From Agent 0 (watchdog): your previous turn ended with an error (probably the Gemini quota, which may have reset now). Continue your current task exactly where you stopped; reread your report file and task file first." >/dev/null
    fi
  done
  sleep "$INTERVAL"
done
