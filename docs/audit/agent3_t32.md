# Audit: docs/team/AGENT3_REPORT_T32.md (T3.2 Plotting Scripts)

**Verdict:** VERIFIED

### Claims vs Independent Reproduction
| claim | report value | my value | match? |
|---|---|---|---|
| test_plots.py status | ALL PLOT TESTS PASSED | ALL PLOT TESTS PASSED | YES |
| synthetic front error | 0.0 m | 0.0 m | YES |
| plot_compare max\|A-B\| | 1.580129e-04 | 1.580129e-04 | YES |
| plot_compare Rel L2 | 4.600612e-06 | 4.600612e-06 | YES |
| straka conv dx=400 front/th' | 15.611 / -10.172 | 15.611 / -10.172 | YES |
| straka conv dx=200 front/th' | 15.506 / -11.186 | 15.506 / -11.186 | YES |
| straka conv dx=100 front/th' | 15.346 / -9.456 | 15.346 / -9.456 | YES |
| straka conv dx=50 front/th' | 15.273 / -9.572 | 15.273 / -9.572 | YES |
| output figures in out/plots/ | 12 files / 3 dirs | All present and non-empty | YES |

### Mutation Test
- **Mutation:** In `scripts/plot_straka_convergence.py`, perturbed the front-detection threshold from `row <= -1.0` to `row <= -1.5`.
- **Result:** `scripts/test_plots.py` caught the mutation immediately: `AssertionError: Front error 300.0 m exceeds half grid cell (50 m)`.

### Rule Check Result
- **Ownership:** Agent 3 edited only `scripts/` and `docs/team/AGENT3_REPORT_T32.md`.
- **Git rules:** Agent 3 ran no git write commands; committed by Agent 0 in commit `7b22b98`.
- **File outputs:** Output PNGs written only under `out/plots/`.

### Issues for Agent 0
- None. Implementation and tests conform strictly to `docs/team/AGENT3_PLOTS.md`.
