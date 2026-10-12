# Agent 7 task T7.1: independent auditor for all agents

You are Agent 7 (Gemini 3.8 Flash High) in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md.
Six agents work in parallel; Agent 0 (the manager) has little capacity to re-check everything.
Your job: **re-verify every claim in every report independently**, so Agent 0 only has to read
your audits. You never write model code. Trust nothing you have not re-run yourself.

## Ownership (only these paths)
docs/audit/ (new): one file per audited report, plus docs/audit/INDEX.md. Scratch work goes
in out/audit/ and in /tmp/cppwrf_audit_*/. Do not edit any other file in the repo, ~/wrf_ref or
~/wrf_real. Never use git write commands, never use sudo. Read-only git commands (log, diff,
show, status, blame) are encouraged.

## What to audit, and when
Reports, and the agents that write them:
docs/STATUS.md (Agent 1), docs/team/AGENT2_REPORT_PHYS.md (2), AGENT3_REPORT_T33/T34/T36.md (3),
AGENT4_REPORT.md (4), AGENT5_REPORT.md (5), AGENT6_REPORT.md (6).
Work in a loop: find the report sections that changed since your last audit (track them in
docs/audit/INDEX.md: report, section, git commit or file mtime, audit status). Audit each new
section. When nothing is new, wait 10 minutes (sleep 600) and check again. Stop when the user
tells you to.

## How to audit one report section
1. **Reproduce.** Re-run the exact commands the report claims (builds into your own build dirs,
   e.g. build-audit-*/, outputs under out/audit/). Compare numbers: same result? Within the
   stated tolerance? Note any number you cannot reproduce, and how far off it is.
2. **Acceptance criteria.** Check the criteria in the agent's task file (docs/team/*.md) one by
   one, e.g. physics: max rel diff <= 1e-12 on >= 2000 columns that really cover the listed
   regimes (inspect the input generator: are hydrometeors, supercooled layers, zero mixing
   ratios and so on actually present?).
3. **Test strength.** Can the test detect a real bug? Do one mutation per audited component:
   copy the source to /tmp, change one coefficient or sign, rebuild there, and confirm the
   test FAILS. A test that still passes after the mutation is worthless: report it.
4. **Rule violations.** `git log --stat` / `git status` / file mtimes: did the agent edit
   files outside its ownership in PLAN.md? Run git write commands (reflog, unexpected commits)?
   Use sudo (shell history if visible)? Change a tolerance or a test to make it pass?
5. **Physics sanity** (where relevant): units, signs, conservation, reasonable magnitudes. Is
   any clamping, ad-hoc limiter or "if NaN set 0" hiding a problem? (grep for isnan, fmax/fmin
   clamps, magic numbers without a WRF source line reference)
6. **GPU vs CPU**: if the report claims CUDA results, re-run at least one case on the GPU.
   Do not time anything while other agents are building (check `top`/`nvidia-smi`; if busy,
   mark timing claims "not re-measured, machine busy").

## Audit file format (docs/audit/<report>_<section>.md, max 40 lines)
- Verdict: VERIFIED / VERIFIED WITH NOTES / NOT REPRODUCED / VIOLATION
- Table: claim | report value | my value | match?
- Mutation test: what was changed, did the test catch it
- Rule check result
- Issues for Agent 0, ranked by severity, each with the exact command that shows it
Keep docs/audit/INDEX.md current: one line per audit with its verdict, so Agent 0 can read
only that file and open the non-VERIFIED ones.

## Resource etiquette
The machine has 12 cores, 15 GB RAM and one GPU, shared by six agents. Build with -j4 at most,
run one heavy job at a time, and delete big scratch outputs when done.
