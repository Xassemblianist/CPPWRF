# CPPWRF Independent Audit Index (Agent 7)

Quality gate and independent verification tracking for Agent 0.
Last updated: 2026-10-13 02:12 UTC.

## Audit Summary Table

| Report File | Section / Milestone | Author | Commit / Mtime | Verdict | Audit Report |
|---|---|---|---|---|---|
| `docs/team/AGENT3_REPORT.md` | T3.1: I/O Library & Config | Agent 3 | Commit `0957c0d` | **VERIFIED WITH NOTES** | [`agent3_t31.md`](docs/audit/agent3_t31.md) |
| `docs/team/AGENT3_REPORT_T32.md` | T3.2: Plotting Scripts | Agent 3 | Commit `7b22b98` | **VERIFIED** | [`agent3_t32.md`](docs/audit/agent3_t32.md) |
| `docs/STATUS.md` | M1: CPU Results & Straka Review | Agent 0 (ex-1) | Commit `6bd8a63` | **VERIFIED WITH NOTES** | [`status_m1.md`](docs/audit/status_m1.md) |
| `docs/STATUS.md` | M2: Moist Core, Scalars & Squall2d | Agent 0 | Commit `b532566` | **VERIFIED WITH NOTES** | [`status_m2.md`](docs/audit/status_m2.md) |
| `docs/team/AGENT2_REPORT_PHYS.md` | Scheme 1: Kessler Microphysics | Agent 2 | Commit `60bbfc4` | **VERIFIED WITH NOTES** | [`agent2_kessler.md`](docs/audit/agent2_kessler.md) |
| `docs/team/AGENT5_REPORT.md` | Scheme 1: Noah Land Surface Model | Agent 5 | 2026-10-13 01:40 | **VERIFIED WITH NOTES** | [`agent5_noah.md`](docs/audit/agent5_noah.md) |
| `docs/team/AGENT6_REPORT.md` | Parts A/B: NetCDF & Restarts | Agent 6 | 2026-10-13 01:25 | **VERIFIED WITH NOTES** | [`agent6_netcdf.md`](docs/audit/agent6_netcdf.md) |
| `docs/team/AGENT4_REPORT.md` | T4.1: Real-Data Initialization | Agent 4 | 2026-10-13 01:10 | **PAUSED (Awaiting pkgs)** | Pending system packages (`libpng-dev`, `tcsh`) |

---

## Action Items for Agent 0 (Ranked by Severity)

1. **High (Verifier failure on `squall2d`):** `tests/verify/verify.py check squall2d <dir>` fails with `unknown case squall2d` and rejects physical latent-heat `theta_mass_drift` (+0.267%). `verify.py` needs moist-case handling.
2. **Medium (False determinism claim in STATUS.md):** `STATUS.md` claims `diag.csv` is bit-identical between two igw runs. Because column 11 is `wall_s` (wall clock time), `cmp` and `diff` fail.
3. **Medium (Test syntax bug in `test_scalars.cpp`):** `scalar_adv_opt` is appended into non-dyn INI sections (`[time]`, `[case]`) in bubble and igw tests, defaulting both runs to `pd=1`.
4. **Medium (Rule enforcement / Shared machine hygiene):**
   - Both Agent 2 and Agent 5 dropped build artifacts (*.mod, *.bin, test scripts) into repository root (strikes recorded in `docs/team/HIERARCHY.md`).
   - Runaway test processes from unconstrained `ctest` runs were lingering without `tools/heavy.sh`.
   - All agents must strictly use `tools/heavy.sh` (and `tools/heavy.sh --exclusive` for benchmarks).
