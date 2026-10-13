#!/usr/bin/env bash
# Agent 0's dispatcher for Gemini agents through the Antigravity CLI (agy).
#   tools/agents/agent.sh send <N> "<message>"   queue a message for Agent N; starts a worker
#                                                if none is running (one turn at a time per agent)
#   tools/agents/agent.sh status                 per agent: running?, queued messages, last result
#   tools/agents/agent.sh log <N> [lines]        tail Agent N's log
#   tools/agents/agent.sh reset <N>              forget Agent N's conversation (next send starts fresh)
#   tools/agents/agent.sh pause|resume <N>       stop starting workers for N (quota) / restart its queue
# State and logs live in ~/cppwrf_agents/ (outside the repo). Each agent keeps one CLI
# conversation; the first message of a conversation gets the role preamble below.
# Owner: Agent 0.
set -uo pipefail
AGY="${AGY:-$HOME/.local/bin/agy}"
MODEL="${AGENT_MODEL:-gemini-3.8-flash-high}"
REPO=/home/ostriquetrum/CPPWRF
ST="$HOME/cppwrf_agents"
mkdir -p "$ST"

preamble() {
  cat <<EOF
You are Agent $1 of the CPPWRF project ($REPO), a Gemini 3.8 Flash High agent working for
Agent 0 (the director). Before anything else read docs/team/PLAN.md and docs/team/HIERARCHY.md:
they are binding (ownership, quality gate, tools/heavy.sh job slots, no git writes, no sudo,
nothing in the repo root). You receive orders from Agent 0 through this conversation. Keep your
report file current; end every turn with a short summary: what you did, the exact commands that
prove it, what is next, and anything you need from Agent 0.

EOF
}

worker() {  # drains the queue of agent $1, one agy turn per message
  local n=$1 q="$ST/agent$1.queue" cidf="$ST/agent$1.cid" log="$ST/agent$1.log"
  exec 8>"$ST/agent$n.lock"
  flock -n 8 || exit 0                       # another worker is already running
  while [ -s "$q" ]; do
    local msg; msg=$(python3 - "$q" <<'PY'
import sys, json
p = sys.argv[1]; lines = open(p).read().splitlines()
print(json.loads(lines[0])); open(p, 'w').write(''.join(l + '\n' for l in lines[1:]))
PY
)
    local args=(-p "" --model "$MODEL" --output-format json --print-timeout 12h --dangerously-skip-permissions)
    if [ -s "$cidf" ]; then args+=(--conversation "$(cat "$cidf")"); else msg="$(preamble "$n")$msg"; fi
    args[1]="$msg"
    echo "===== $(date '+%F %T') SEND to Agent $n =====" >> "$log"
    echo "$msg" >> "$log"
    local out; out=$(cd "$REPO" && "$AGY" "${args[@]}" 2>>"$log")
    echo "===== $(date '+%F %T') RESULT =====" >> "$log"
    echo "$out" >> "$log"
    local cid; cid=$(echo "$out" | python3 -c "import sys,json
try: print(json.loads(sys.stdin.read()).get('conversation_id',''))
except Exception: print('')")
    [ -n "$cid" ] && echo "$cid" > "$cidf"
    echo "$out" | python3 -c "import sys,json
try: d=json.loads(sys.stdin.read()); print(d.get('status'), '|', d.get('response','')[-1500:])
except Exception as e: print('UNPARSED', e)" > "$ST/agent$n.last"
  done
}

cmd=${1:-}; shift || true
case "$cmd" in
  send)
    n=$1; msg=$2
    python3 -c "import sys,json; print(json.dumps(sys.argv[1]))" "$msg" >> "$ST/agent$n.queue"
    if [ -e "$ST/agent$n.paused" ]; then echo "queued for Agent $n (paused: no worker started)"; else
      nohup "$0" _worker "$n" >/dev/null 2>&1 &
      echo "queued for Agent $n"; fi ;;
  pause) touch "$ST/agent$1.paused"; echo "Agent $1 paused (running turn not stopped)" ;;
  resume) rm -f "$ST/agent$1.paused"; nohup "$0" _worker "$1" >/dev/null 2>&1 & echo "Agent $1 resumed" ;;
  _worker) worker "$1" ;;
  status)
    for f in "$ST"/agent*.cid "$ST"/agent*.queue; do [ -e "$f" ] || continue; done
    for n in $(ls "$ST" | sed -n 's/^agent\([0-9]*\)\..*/\1/p' | sort -un); do
      exec 7>"$ST/agent$n.lock"
      if flock -n 7; then run=idle; flock -u 7; else run=RUNNING; fi
      qn=$( [ -s "$ST/agent$n.queue" ] && wc -l < "$ST/agent$n.queue" || echo 0)
      echo "Agent $n: $run, queued $qn, conversation $(cat "$ST/agent$n.cid" 2>/dev/null || echo none)"
      [ -s "$ST/agent$n.last" ] && head -c 400 "$ST/agent$n.last" | sed 's/^/   last: /' && echo
      true
    done ;;
  log) tail -n "${2:-60}" "$ST/agent$1.log" ;;
  reset) rm -f "$ST/agent$1.cid"; echo "Agent $1 conversation forgotten" ;;
  *) sed -n '2,10p' "$0"; exit 2 ;;
esac
