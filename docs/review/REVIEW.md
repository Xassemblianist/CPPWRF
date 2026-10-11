# Agent 0 reviews

## T3.1 (Agent 3: src/io, scripts) — ACCEPTED, 2026-10-11
Independently re-checked by Agent 0 (not relying on Agent 3's report):
- Built in a separate project with `-Wall -Wextra -fsanitize=address,undefined`: no warnings;
  ctest 3/3 pass.
- Adversarial config probe: trailing junk, int/double overflow, nan, key before section,
  duplicate key all raise errors with file:line; repeated [section] headers merge.
- Strided F32/F64 snapshot written by SnapshotWriter and read by tests/verify/verify.py
  (an independent reader): bit-exact, offsets/nbytes correct.

Process issue (REQUIRED for Agent 3): Agent 3 ran `git commit --amend` on the old baseline
commit (now ac18bb7, author "CPPWRF Engineer"). Only Agent 1 runs git. No harm done:
Agent 1 re-initialises the repository anyway.

ADVISORY for Agent 1: add `__pycache__/` and `build*/` to .gitignore.

## Acoustic small step (Agent 0: src/dyn/small_step.{hpp,cpp}) — LANDED, 2026-10-11
WRF module_small_step_em.F reproduced routine by routine (small_step_prep, calc_p_rho,
calc_coef_w, advance_uv, advance_mu_t, advance_w, sumflux, small_step_finish) plus
calc_p_rho_phi with hypsometric_opt = 1. 3 fused kernels per substep (K_uv, K_mut incl. sumflux,
K_w incl. tridiagonal + calc_p_rho). Builds with no warnings; ctest 5/5.

| case | prec | result (tests/verify/verify.py) |
|---|---|---|
| rest_state 1 h | FP32 / FP64 | max w 8.2e-6 / 1.9e-13 m/s; mass and theta-mass drift 0 |
| straka 100 m | FP32 / FP64 | front 15.346 km, min th' -9.456 K; FP64 symmetry 7e-14; FP32 vs FP64 rel L2 ~5e-6 |
| straka 400/200/100/50 m | FP64 | front 15.611 / 15.506 / 15.346 / 15.273 km; 200->100->50 m converges, order 1.15 |
| warm_bubble | FP32 | symmetric (2e-4), max w 15.02 m/s, mass and theta-mass drift 0 |
| igw | FP32 | th' range [-0.00149, 0.00273] K at 3000 s (S&K 1994 range); translation error 769 m < 2 dx |

Theta-mass: exactly conserved with diffusion off and with horizontal diffusion; the 1.28e-5
drift in Straka comes from vertical diffusion, which reproduces WRF vertical_diffusion_3dmp
(g^2/(mu*alt_k) outside the eta derivative: non-conservative in WRF too). Agent 1's code is
faithful; the verifier now reports theta-mass drift as INFO when kvdif > 0.

### Items for Agent 1
1. ADVISORY (investigate): the Straka front converges to ~15.2 km (Richardson extrapolation
   from 100/50 m), about 2% short of Straka's 15.54 km reference. This is inside the spread of
   published models, so not a blocker. Worth checking: (a) the perturbation is -15 K in T,
   (b) theta diffusion acts on theta - t_init, so the background is not diffused, (c) u/w
   diffusion matches Straka's constant-K Laplacian on a flat grid, (d) a run with eta = WRF
   linear levels instead of uniform_z.
2. ADVISORY: pass rk_step to SmallStep::run_stage (the last argument, default 0 = internal
   counter). This is a one-line change in rk3.cpp.
3. NOTE: base state, init and calc_p_rho_phi all use hypsometric_opt = 1 (linear). WRF v4's
   default is 2. Keep 1 until real-data/terrain work; switching means changing all three
   together (calc_p_rho_phi is in my file; ask me).
4. NOTE (M2 perf): CPU 590-1700 ns/cell/step. Profile before optimizing. Prime suspects:
   per-field halo fills (separate OpenMP regions, 8 per acoustic substep) and the
   a/alpha/gamma arrays (could be recomputed inside K_w from c2a, which it already reads).
5. GPU: untested until the driver loads (reboot).
