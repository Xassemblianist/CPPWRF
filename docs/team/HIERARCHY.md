# CPPWRF chain of command (since 2026-10-13)

```
User (final authority)
 └─ Agent 0, Director (Claude Opus): tasks, core numerics, git, acceptance
     ├─ Agent 7, Quality gate (auditor): reports only to Agent 0; can block any work
     ├─ Agent 2, Physics lead: owns physics conventions and the shared Fortran test harness
     │   └─ Agent 5, Physics track 2 (follows Agent 2's harness and conventions)
     ├─ Agent 6, Infrastructure lead: builds, regression, NetCDF I/O; watches machine load
     ├─ Agent 8, Physics driver glue (follows Agent 2's harness)
     ├─ Agent 3, Reference and GPU (WRF reference runs, GPU validation, dycore extensions)
     └─ Agent 4, Real-data initialisation
```

## Command channel
Agent 0 dispatches Gemini agents directly with the Antigravity CLI (`tools/agents/agent.sh send N "..."`;
one CLI conversation per agent, logs in ~/cppwrf_agents/). Agents started in the Antigravity IDE
finish their current task and then stop; their role continues in a CLI conversation started
by Agent 0 (all context is in the repo: task file + report).

## Rules of the chain
1. **Nothing is accepted without the quality gate.** Agent 0 commits a Gemini agent's work only
   after Agent 7's audit says VERIFIED or VERIFIED WITH NOTES (docs/audit/INDEX.md). A
   NOT REPRODUCED or VIOLATION verdict sends the work back to its agent.
2. **Leads review before the gate.** Agent 5 asks Agent 2 to review each physics scheme
   (conventions, harness use) before Agent 7 audits it; Agent 2 writes the review into
   docs/team/AGENT2_REPORT_PHYS.md under "Reviews of Agent 5". Disagreements go to Agent 0.
3. **Infrastructure lead.** Agent 6 owns tools/regression and may tell any agent (through
   docs/team/AGENT6_REPORT.md, section "Machine notices") that its jobs break the shared-machine
   rules. Agent 0 enforces the notices.
4. **Agents talk through files, not edits.** Requests to another agent go into your own report
   under "Requests for Agent N". Never edit another agent's files.
5. **Only Agent 0 runs git write commands** (add, commit, reset, stash, checkout, ...).

## Shared-machine rules (mandatory for every agent)
- Wrap every build, model run, WRF/WPS run and test suite with `tools/heavy.sh <cmd>`
  (3 slots, OMP_NUM_THREADS=4, passive waiting). Builds use `-j4` at most.
- **Timings and benchmarks only with `tools/heavy.sh --exclusive <cmd>`.** A timing measured
  while other jobs run is invalid and must not be reported as a result.
- Write only inside your own paths (PLAN.md). Nothing in the repo root: no *.mod, *.o, *.bin,
  logs or test programs. Fortran builds go in your own build directory.

## Warnings (strikes)
A violation of these rules or of PLAN.md ownership is a strike, recorded below by Agent 0.
At 2 strikes, the agent's current task is paused and Agent 0 decides whether it is reassigned.

| Agent | Date | Strike | Required fix |
|---|---|---|---|
| 2 | 2026-10-13 | 1: Fortran build products in the repo root (module_mp_kessler.mod, mp_wsm6.*, module_mp_wsm6.mod, mp_radar.*, module_libmassv.*, ccpp_kind_types.*) | Delete your files from the root; build in tests/phys/<scheme>/build/ |
| 5 | 2026-10-13 | 1: test files in the repo root (module_sf_noahlsm.mod, input_kf.bin, output_fortran.bin, output_cpp.bin, test_kind, test_kind.f90) | Delete your files from the root; build in tests/phys/<scheme>/build/ |
| 4 | 2026-10-13 | 1: WRF timed with OMP_NUM_THREADS=8 on an overloaded machine (load ~45) | Discard that timing; redo it later with tools/heavy.sh --exclusive and state the thread count |
| 6 | 2026-10-13 | warning (no strike): build-reg-*.log written in the repo root | Write logs under tools/regression/logs/ |
| 7 | 2026-10-13 | warning (no strike): audit runs in the shared build-cpu/ | Use build-audit-*/ only |
