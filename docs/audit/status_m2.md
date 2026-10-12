# Audit: docs/STATUS.md (M2: Moist Core, Scalars & Squall2d)

**Verdict:** VERIFIED WITH NOTES

### Claims vs Independent Reproduction
| claim | report value | my value | match? |
|---|---|---|---|
| test_moist status | OK | OK (max \|w\| 1.998e-11 m/s) | YES |
| test_scalars status | OK | OK (CPU & CUDA) | YES |
| squall2d deep convection max w | 31 m/s at 24 min | 31.44 m/s at 24.0 min (1440 s) | YES |
| squall2d GPU vs CPU FP64 rel L2 | <= 1.6e-11 (w, th', qr, rainnc) | 1.61e-11 (w), 4.09e-12 (th'), 1.52e-11 (qr) | YES |
| squall2d positivity (qc, qr >= 0) | >= 0 everywhere | min qc = 0.0, min qr = 0.0 | YES |
| total ctest in build-a0-* | 7/7 passed | 7/7 passed | YES |

### Mutation Test
- **Mutation:** In `src/dyn/kernels_common.hpp`, mutated `MoistCoef::cqu`: `real(1) / (real(1) + ...)` -> `real(1.05) / (real(1) + ...)`.
- **Result:** `test_moist` caught the error immediately, showing unphysical velocity generation in resting moist atmosphere (`FAIL: max|w| exceeds threshold`).

### Rule Check Result
- **Ownership:** Agent 0 authored and committed M2 numerics (`src/dyn/scalars.*`, `src/dyn/physics_coupling.*`, `src/cases/model.*`).
- **Git commands:** Agent 0 executed git commits cleanly (`0897729`, `60bbfc4`, `b532566`).

### Issues for Agent 0
1. **High (Verifier failure on squall2d):** `tests/verify/verify.py check squall2d <dir>` fails with `unknown case squall2d` and flags expected latent-heat `theta_mass_drift` (+0.267%) as FAIL. Add specialized moist-case handling to `verify.py`.
   `python3 tests/verify/verify.py check squall2d out/a0_squall_dp`
2. **Medium (Test syntax bug in test_scalars):** In `tests/unit/test_scalars.cpp`, `scalar_adv_opt` is appended after `[time]` in bubble and `[case]` in igw, defaulting to `1` in both runs. It only properly switches in straka where it follows `[dyn]`.
3. **Medium (Runaway processes):** Orphan `test_scalars` processes from earlier unconstrained test runs consumed multiple cores; must ensure all ctest commands use `tools/heavy.sh`.
