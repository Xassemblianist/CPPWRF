#!/usr/bin/env bash
# Wave 2 dispatch (Agent 0): one command starts all Gemini agents on their next tasks.
#   tools/agents/wave2.sh            send all
#   tools/agents/wave2.sh 3 4        send only to the listed agents
set -uo pipefail
A="$(dirname "$0")/agent.sh"
COMMON="Read docs/team/PLAN.md and docs/team/HIERARCHY.md again first (ownership changed: Agent 8 is new; tools/heavy.sh is re-entrant now, so scripts may call it internally). Machine rules unchanged: every build/run via tools/heavy.sh, -j4, timings only with --exclusive, nothing in the repo root, no git writes, never git stash. When done, write your report and stop."
declare -A MSG=(
 [2]="From Agent 0: continue scheme 4 (RRTMG) exactly where you stopped (your plan is in AGENT2_REPORT_PHYS.md; the quota ran out during your Fortran build). LW first, then SW. $COMMON"
 [3]="From Agent 0: your task is T3.8 in docs/team/AGENT3_T38_HILL_CASE.md (hill2d initial state in CPPWRF, verified against your WRF runs). $COMMON"
 [4]="From Agent 0: your T4.2 runs are excellent, thank you. Your task is T4.3 in docs/team/AGENT4_T43_BC_PORT.md (lateral boundary call map + port + differential test). $COMMON"
 [5]="From Agent 0: Kain-Fritsch and Noah are committed. You are continuing in a new CLI conversation: read docs/team/AGENT5_PHYSICS2.md and your previous reports (docs/team/AGENT5_REPORT*.md) first. Continue with item 3, Thompson (mp_physics=8): tables first (verified against Fortran-generated tables), then the column kernel, differential-tested. Ask Agent 2 (via your report) for a convention review when done. $COMMON"
 [6]="From Agent 0: your T6.2 is committed (52ecdbc). Your task is T6.3 in docs/team/AGENT6_T63_WRFINPUT_READER.md. $COMMON"
 [7]="From Agent 0: your task is audit A3 in docs/team/AGENT7_A3_DYCORE_AUDIT.md (and your standing rules in docs/team/AGENT7_AUDITOR.md). $COMMON"
 [8]="From Agent 0: welcome. Your task is T8.1 in docs/team/AGENT8_PHYS_DRIVER.md. $COMMON"
)
ids=("$@"); [ ${#ids[@]} -eq 0 ] && ids=(2 3 4 5 6 7 8)
for n in "${ids[@]}"; do "$A" send "$n" "${MSG[$n]}"; sleep 2; done
