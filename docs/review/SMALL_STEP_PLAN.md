# small_step plan (Agent 0) — what Agent 1's types must contain

Mapping onto WRF (dyn_em/module_small_step_em.F, solve_em.F) and TN ch. 3:

| Our call inside SmallStep::run_stage | WRF routine | Fused GPU kernel |
|---|---|---|
| prep (once per stage) | small_step_prep, calc_p_rho (init), calc_coef_w | K0: per-cell prep; K0c: per-column coefficients |
| loop n_small: horizontal momentum | advance_uv | K1 (u and v in one kernel) |
| mu, theta, omega | advance_mu_t | K2: column kernel (vertical integral of divergence -> mu', ww, t) |
| vertical implicit w/phi | advance_w | K3: column kernel, tridiagonal in registers |
| time-averaged mass fluxes | sumflux | folded into K1/K2/K3 (no separate pass) |
| p', alpha' diagnosis + smdiv save | calc_p_rho | folded into K3 (column already in registers) |
| finish (once per stage) | small_step_finish | K4 |

Target: about 3 kernels per substep, with each prognostic read once and written once.

## Members I need (WRF Registry names; Agent 1 owns the structs)
**State** (3D unless noted; `_1` = time-t value, `_2` = current RK-stage value):
u_1 u_2 v_1 v_2 w_1 w_2 t_1 t_2 ph_1 ph_2; mu_1 mu_2 (2D); p al alt php;
ww (omega mass flux); mut muu muv (2D, full column mass at t); muts muus muvs (2D, stage);
ru_m rv_m ww_m (time-averaged fluxes for scalar advection — outputs of SmallStep).

**BaseState:** mub (2D); pb phb alb t0 (=300 K); p_top; 1D vertical: znu znw dnw dn rdnw
rdn fnm fnp, c1h c2h c1f c2f (hybrid coefficients; c1=1, c2=0 when hybrid_opt=0); cf1 cf2
cf3 (surface extrapolation); constants rdx rdy. Map factors: idealized = 1. Do not add
msf arrays yet, but keep a single place where they would go.

**Tendencies:** ru_tend rv_tend rw_tend t_tend ph_tend (3D), mu_tend (2D). These are the
large-step tendencies, already coupled with mass, as in WRF.

**SmallStep allocates privately:** u_save v_save w_save t_save ph_save mu_save ww1 c2a
pm1 a alpha gamma mudf. Do not put them in State.

## Behaviour I implement (for Agent 1's RK3 driver)
- Stage dt and substeps follow solve_em: stage 1 = dt/3 with 1 substep (when time_step_sound
  is chosen so that n_small for the full step is >= 2); stage 2 = dt/2 with n/2; stage 3 =
  dt with n substeps. Agent 1 computes n and passes dt_stage, dts and n_small.
- On return: u_2..mu_2 hold the stage-end values (re-coupled as in small_step_finish);
  ru_m, rv_m, ww_m hold the time-averaged fluxes; p and al are consistent with t_2/mu_2/ph_2.
- epssm, smdiv, emdiv as in SmallStepParams (namelist defaults 0.1 / 0.1 / 0.01).
- Boundary handling: periodic and symmetric x/y via the halo-fill helper that Agent 1
  provides (I call it; I do not write my own halo code). Rigid lid: w=0 at k=nz (top);
  free-slip flat bottom: w=0 at k=0.
