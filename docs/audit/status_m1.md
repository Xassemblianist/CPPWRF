# Audit: docs/STATUS.md (M1: CPU Results & Numerics)

**Verdict:** VERIFIED WITH NOTES

### Claims vs Independent Reproduction
| claim | report value | my value | match? |
|---|---|---|---|
| rest_state FP32 max w / th' | 8.24e-6 m/s / 3.8e-6 K | 8.238e-6 m/s / 3.815e-6 K | YES |
| rest_state FP64 max w / th' | 1.88e-13 m/s / 7.1e-15 K | 1.879e-13 m/s / 7.105e-15 K | YES |
| straka FP32 min th' / front / sym | -9.456 K / 15.346 km / 1.5e-5 | -9.456 K / 15.346 km / 1.47e-5 | YES |
| straka FP64 min th' / front / sym | -9.456 K / 15.346 km / 7e-14 | -9.456 K / 15.346 km / 7.17e-14 | YES |
| warm_bubble FP32 / FP64 max w | 15.018 m/s / 15.018 m/s | 15.018 m/s / 15.018 m/s | YES |
| igw FP32 / FP64 centroid err | 769 m / 1212 m | 769 m / 1212 m | YES |
| test_large_step advection order | 5.00 (FP64) | 5.00 (FP64) | YES |
| test_large_step diffusion x / z | 0.9992 / 0.9653 | 0.9992 / 0.9653 | YES |
| determinism: snap_000500.bin cmp | identical | identical (cmp exit 0) | YES |
| determinism: diag.csv cmp | identical | **MISMATCH (wall_s varies)** | **NO** |

### Mutation Test
- **Mutation:** Perturbed 5th-order flux advection stencil weighting (`37` -> `35`) in `src/dyn/kernels_common.hpp`.
- **Result:** `test_large_step` caught the error immediately (`advection: order=-0.00`, `FAIL: order > 4.5`).

### Rule Check Result
- **Ownership:** Agent 1 (now Agent 0) edited owned paths under `src/core/`, `src/dyn/`, `src/cases/`, `tests/unit/`.
- **Git commands:** Only Agent 1 (and Agent 0 acting for it) performed git commits.

### Issues for Agent 0
1. **Medium (False determinism claim in report):** `STATUS.md` states "two FP32 igw runs -> cmp of snap_000500.bin identical, diag.csv identical." While the binary snapshot and physical diagnostic columns (1-10) are bit-identical, column 11 is `wall_s` (elapsed wall clock time). `cmp` or `diff` on `diag.csv` fails due to `wall_s`.
   `diff -u out/det_a/diag.csv out/det_b/diag.csv`
2. **Low:** `test_large_step` reports negative advection order (`-0.48`) in FP32 because errors are at single-precision roundoff; cleanly achieves 5.00 in FP64.
