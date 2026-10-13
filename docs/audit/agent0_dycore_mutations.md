# Audit: Mutation Testing of Dycore Checks (`dycore_checks.py`)

**Audited Target:** `tests/verify/dycore_checks.py` (Owner: Agent 0)
**Execution Mode:** Isolated scratch workspace (`/tmp/cppwrf_audit_scratch`), zero repository writes.
**Hardware Wrapping:** `tools/heavy.sh python3 ...` (Slot locking, `OMP_NUM_THREADS=4`, `OMP_WAIT_POLICY=passive`).
**Verdict:** **VERIFIED (100% MUTATION KILL RATE, 0 SURVIVORS)**

---

### 1. Baseline Test Execution
Prior to mutation injection, the scratch build was tested to confirm nominal verification:
```bash
tools/heavy.sh python3 tests/verify/dycore_checks.py /tmp/cppwrf_audit_scratch/build-mut/cppwrf /tmp/cppwrf_audit_scratch/out_base
```
**Output:**
```text
== dycore consistency
  PASS  map_factor_invariance    max |m=2,dx=2km - m=1,dx=1km| = 0.000e+00 (must be 0)
  PASS  coriolis_inertial        max |(u,v) - exact| = 7.32e-06 m/s over one period (tol 1e-3)
```

---

### 2. Mutation Injection Results

Every single injected mutation broke map-factor scaling invariance in squall2d, resulting in either unphysical numerical instability (non-finite velocity blowup) or massive field discrepancy between $m=2$ and $m=1$:

| ID | Location | Target Map Factor | Injected Mutation | Observed Result | Status |
|---|---|---|---|---|---|
| **M1** | `src/dyn/large_step.cpp:313` (`KTendPh`) | Geopotential vertical advection `/ msfty` | `... Gr * w / msfty` $\rightarrow$ `... Gr * w` | **FAIL** (Non-finite values at step 50, $t=300$ s, max \|w\|=1827 m/s, exit 3) | **KILLED** |
| **M2** | `src/dyn/small_step.cpp:183` (`KMuTDiv`) | Acoustic divergence flux scaling `msftx` | `msftx * msfty * d` $\rightarrow$ `msfty * d` | **FAIL** (Non-finite values at step 120, $t=720$ s, max \|u\|=inf, exit 3) | **KILLED** |
| **M3** | `src/dyn/large_step.cpp:286` (`KTendT`) | Diabatic latent heating coupling `1/msfty` | `... hd / msfty` $\rightarrow$ `... hd` | **FAIL** (`map_factor_invariance` max diff = $1.031 \times 10^3$, exit 1) | **KILLED** |
| **M4** | `src/dyn/large_step.cpp:60` (`KWW`) | Column divergence eta velocity `msftx` | `msftx * c.dnw * ...` $\rightarrow$ `c.dnw * ...` | **FAIL** (`map_factor_invariance` max diff = $5.365 \times 10^2$, exit 1) | **KILLED** |
| **M5** | `src/dyn/large_step.cpp:47` (`KPrep3`) | U-momentum coupling `msfuy` | `... muu / msfuy` $\rightarrow$ `... muu` | **FAIL** (Non-finite values at step 60, $t=360$ s, max \|u\|=inf, exit 3) | **KILLED** |
| **M6** | `src/dyn/scalars.cpp:74` (`KUpdate`) | Scalar advection tendency coupling `msfty` | `adv * f.msfty` $\rightarrow$ `adv` | **FAIL** (`map_factor_invariance` max diff = $3.276 \times 10^1$, exit 1) | **KILLED** |

---

### 3. Summary & Analysis
- **Total Mutations Injected:** 6
- **Mutations Killed:** 6
- **Mutations Survived:** **0**
- **Analysis:**
  Because the map-factor invariance check in `dycore_checks.py` checks for strict bitwise invariance ($|m=2 - m=1| == 0$ in IEEE-754 floating-point arithmetic across 8 prognostic fields `th_pert`, `u`, `w`, `mu`, `qv`, `qc`, `qr`, `p`), any omitted or incorrectly placed map factor scale factor breaks bit-level cancellation and induces immediate observable drift or instability.
  The test is highly sensitive, robust, and mathematically sound.
