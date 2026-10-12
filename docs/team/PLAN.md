# CPPWRF team plan (owner: Agent 0)

This file overrides any conflicting ownership statement in an agent's original prompt.

**Since 2026-10-13 Agent 1 and Agent 0 are one agent ("Agent 0", Claude Opus 5.5).** Every
reference to Agent 1 (ownership, integration, git, STATUS.md) now means Agent 0.

**Chain of command, quality gate, shared-machine rules and strikes: docs/team/HIERARCHY.md (binding).**

## Rules for every Gemini agent (shared machine: 12 cores, 15 GB RAM, 1 GPU)
- Wrap builds/runs/tests with `tools/heavy.sh`; timings only with `tools/heavy.sh --exclusive`.
- Run everything with `OMP_NUM_THREADS=4 OMP_WAIT_POLICY=passive`; build with `-j4` at most.
- Never write files in the repo root or outside your own paths (no *.mod, *.o, *.bin, test
  programs there): compile Fortran in your own build directory.
- One heavy job at a time per agent.

## Agents
| Name | Model | Role |
|---|---|---|
| Agent 0 | Claude Opus 5.5 | Manager. Writes the hard numerical kernels it owns, the verification suite, reviews, and all task prompts. |
| Agent 1 | Claude Opus 5.5 (medium) | Lead implementer: build, core, large-step dynamics, cases, CUDA. |
| Agent 2 | Gemini 3.8 Flash High | Physics port (src/phys/) with differential tests against the original Fortran (docs/team/AGENT2_PHYSICS.md). |
| Agent 3 | Gemini 3.8 Flash High | Reference WRF runs (T3.3), GPU validation and benchmarks (T3.4), dycore extensions (T3.6), I/O and scripts. Works from a task file in docs/team/. |
| Agent 4 | Gemini 3.8 Flash High | Real-data initialisation: WPS + real.exe equivalent, differential-tested against real.exe (docs/team/AGENT4_REAL_INIT.md). |
| Agent 5 | Gemini 3.8 Flash High | Second physics track: Noah LSM, Kain-Fritsch, Thompson (docs/team/AGENT5_PHYSICS2.md). |
| Agent 6 | Gemini 3.8 Flash High | WRF-format NetCDF output, restarts, regression system (docs/team/AGENT6_NETCDF_REGRESSION.md). |
| Agent 7 | Gemini 3.8 Flash High | Independent auditor: re-runs every report's claims, mutation-tests the tests, checks rule violations (docs/team/AGENT7_AUDITOR.md). Never writes model code. |

Agent 0 has slightly more authority (it sets tasks), but Agents 0 and 1 are peers on technical
questions: disagree in writing (docs/STATUS.md "Disagreements" for Agent 1, docs/review/REVIEW.md
for Agent 0). The user decides.

## File ownership. Edit only your own paths
| Path | Owner |
|---|---|
| CMakeLists.txt (top level), cmake/ | Agent 1 |
| src/core/ (backend, field, grid, constants, precision) | Agent 1 |
| src/dyn/ **except** src/dyn/small_step.* | Agent 1 |
| src/dyn/small_step.hpp, src/dyn/small_step.cpp | Agent 1 (since M2; written by Agent 0) |
| src/cases/, src/main.cpp, configs/ | Agent 1 |
| src/io/ (incl. src/io/CMakeLists.txt), tests/unit/io/ | **Agent 3** |
| scripts/ (Python readers/plots) | **Agent 3** |
| tools/gpu/, docs/PERF_MEASURED.md | **Agent 3** |
| src/phys/, tests/phys/ (physics ports + Fortran differential tests) | **Agent 2** |
| src/init_real/, tests/init_real/, ~/wrf_real/ (real-data initialisation) | **Agent 4** |
| ~/wrf_ref/ (reference WRF for idealized cases) | **Agent 3** |
| src/phys/{lsm_noah,cu_kf,mp_thompson}/ and tests/phys/ same names | **Agent 5** |
| src/io_netcdf/, tests/io_netcdf/, tools/regression/ | **Agent 6** |
| docs/audit/ | **Agent 7** |
| tests/unit/ (all except io) | Agent 1 |
| tests/verify/ | Agent 0 |
| docs/team/, docs/review/ | Agent 0 |
| docs/STATUS.md, docs/PERF.md | Agent 1 |

If you need a change in someone else's file, write the request in your own report and let its owner make the change.

**Git:** only Agent 1 runs git write commands (add/commit/checkout/stash/reset). It commits
the other agents' files as they are, at its own commit points. Agents 0 and 3 never run git
write commands. The leftover history from the abandoned Gemini attempt (commit 248e463) must
not be kept: Agent 1 re-initialises the repository (`rm -rf .git && git init -b main`) before
its first commit.

## Interfaces between owners

### A. Acoustic small step (Agent 1 calls it, Agent 0 implements it)
Agent 1 implements the RK3 driver and the large-step tendencies. Per RK stage it calls:

```cpp
// src/dyn/small_step.hpp  (Agent 0)
namespace cppwrf::dyn {
struct SmallStepParams { real epssm = 0.1, smdiv = 0.1, emdiv = 0.01; };
class SmallStep {
 public:
  SmallStep(const Grid& g, const SmallStepParams& p);   // allocates its own scratch once
  // Runs the n_small acoustic substeps of one RK stage (TN ch. 3, small_step_prep ..
  // small_step_finish in WRF terms). On entry: s holds the time-t state and the
  // stage-start state; tend holds the large-step tendencies. On exit: s holds the
  // stage-end state, including the time-averaged mass fluxes used by scalar advection.
  void run_stage(State& s, const Tendencies& tend, const BaseState& base,
                 real dt_stage, real dts, int n_small);
};
}
```
Agent 1 owns the definitions of `Grid`, `State`, `Tendencies` and `BaseState`. Use **WRF
Registry names** for every member (u_1/u_2, v_*, w_*, t_*, ph_*, mu_*, mub, phb, pb, alb,
al, alt, p, php, ww, muu, muv, mut, ru, rv, rw, ru_tend, rv_tend, rw_tend, t_tend, ph_tend,
mu_tend, ru_m, rv_m, ww_m, c2a, fnm, fnp, rdnw, rdn, dnw, dn, ...), so the TN and the WRF
source map 1:1 onto our code. Agent 0 adds the members it needs (e.g. small-step saves)
by asking Agent 1. Until small_step exists, Agent 1 may keep a stub that throws.

### B. I/O and config (Agent 1 calls them, Agent 3 implements them)
Library target `cppwrf_io` (C++20, no CUDA, no external dependencies). Full spec:
docs/team/AGENT3_IO.md. Agent 1 must not write its own INI parser or output writer;
it uses these headers:
- `src/io/config.hpp`: `cppwrf::io::Config`
- `src/io/output.hpp`: `VarView`, `SnapshotWriter`, `DiagWriter`, `JsonWriter`

The on-disk format (run.json, diag.csv, snap_*.bin/.json) is unchanged from Agent 1's
original prompt.

## Task routing (Agent 0 decides, user executes)
- **Agent 3 (Gemini Flash):** parsers, file formats, scripts, plots, boilerplate, docs.
  Allowed only when the spec is complete, the task is self-contained, and correctness is
  checked by unit tests.
- **Agent 1 + Agent 0:** numerics, data layout, GPU kernels, anything where a subtle bug
  produces a plausible but wrong atmosphere.

## Communication
The user relays messages. Each agent reports in its own file:
- Agent 1 -> docs/STATUS.md
- Agent 3 -> docs/team/AGENT3_REPORT.md
- Agent 0 -> docs/review/REVIEW.md

## Routing rule (user, 2026-10-13)
Gemini's quota is effectively unlimited; Opus's is scarce. Every long, mechanical or
repetitive job (builds, runs, sweeps, benchmarks, profiling, plots, scripts, docs) goes to
Agent 3. Opus agents do only numerical-core code and review.
