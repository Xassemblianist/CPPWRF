# Lateral Boundary Call Map & Boundary Alterations for WRF Real-Data Dycore
**Author:** Agent 4 (Gemini 3.8 Flash High)  
**Task:** T4.3  
**Target Audience:** Agent 0 (Director / Core Numerics Integrator)  
**Configuration Target:** Real-data simulation, `spec_bdy_width = 5`, `spec_zone = 1`, `relax_zone = 4`, `specified = .true.`, physics off, non-periodic domain ($0 \le i < nx$, $0 \le j < ny$, $0 \le k < nz$).

---

## 1. Boundary Configuration & Geometry

### 1.1 Zone Definitions
For `spec_bdy_width = 5`, `spec_zone = 1`, `relax_zone = 4`:
- **Specified Zone (`spec_zone = 1`)**:
  - Distance $b = 0$ (outermost row/column on boundary faces).
  - Fortran index: $1$ on west/south, $ide / jde$ (or $ide-1 / jde-1$) on east/north.
  - C++ 0-indexed: $i = 0, nx-1$ (or $nx$ for U), $j = 0, ny-1$ (or $ny$ for V).
  - Treatment: Large-step tendencies explicitly assigned from `*_BTXS/*_BTXE/*_BTYS/*_BTYE` via `spec_bdytend` (`spec_bdy_dry` / `spec_bdy_scalar`). Prognostic variables updated during acoustic substeps via `spec_bdyupdate` / `spec_bdyupdate_ph`, and overwritten at time-step completion by `spec_bdy_final` using linearly interpolated values from `wrfbdy_d01` to eliminate numerical roundoff drift.
- **Relaxation Zone (`relax_zone = 4`)**:
  - Distance $b \in [1, 4]$ ($1 \le b\_dist \le 4$, cells 2 to 5 from domain boundary).
  - Fortran index: $2..5$ on west/south, $(ide-4)..(ide-1)$ on east/north.
  - C++ 0-indexed: $i = 1..4$ and $(nx-5)..(nx-2)$.
  - Treatment: In **RK step 1 only**, `relax_bdy_dry` adds a combination of Newtonian relaxation towards the boundary value and horizontal diffusion of the error between the model state and the boundary value:
    $$\tau_{relax} = f_{cx}(b+1) \cdot \Delta F - g_{cx}(b+1) \cdot \nabla_h^2 (\Delta F)$$
    where $\Delta F = (F_{bdy} + dtbc \cdot \dot{F}_{tend}) - F_{model}$.
- **Interior Zone**:
  - Cells $\ge 5$ from domain boundary ($i \ge 5$ and $i \le nx-6$, $j \ge 5$ and $j \le ny-6$).
  - Full-order interior stencils (5th/6th order horizontal flux advection, standard pressure gradient, coriolis, curvature).

### 1.2 Relaxation Coefficients (`lbc_fcx_gcx`)
Calculated once at model startup (`dyn_em/start_em.F:350`):
For $loop = \text{spec\_zone} + 1 \dots \text{spec\_zone} + \text{relax\_zone}$ (Fortran 1-based indices $2 \dots 5$):
$$\text{factor} = \frac{\text{spec\_zone} + \text{relax\_zone} - loop}{\text{relax\_zone} - 1} = \frac{5 - loop}{3}$$
$$fcx(loop) = \frac{0.1}{\Delta t} \cdot \text{factor} \cdot \exp\left(-(loop - (\text{spec\_zone} + 1)) \cdot \text{spec\_exp}\right)$$
$$gcx(loop) = \frac{1.0}{50.0 \cdot \Delta t} \cdot \text{factor} \cdot \exp\left(-(loop - (\text{spec\_zone} + 1)) \cdot \text{spec\_exp}\right)$$
For default `spec_exp = 0.0`:
- $loop = 2$ ($b=1$, cell 2): $\text{factor} = 1.0$, $fcx(2) = 0.1 / \Delta t$, $gcx(2) = 0.02 / \Delta t$
- $loop = 3$ ($b=2$, cell 3): $\text{factor} = 2/3$, $fcx(3) = 0.1 \cdot (2/3) / \Delta t$, $gcx(3) = 0.02 \cdot (2/3) / \Delta t$
- $loop = 4$ ($b=3$, cell 4): $\text{factor} = 1/3$, $fcx(4) = 0.1 \cdot (1/3) / \Delta t$, $gcx(4) = 0.02 \cdot (1/3) / \Delta t$
- $loop = 5$ ($b=4$, cell 5): $\text{factor} = 0.0$, $fcx(5) = 0.0$, $gcx(5) = 0.0$
- $loop = 1$ ($b=0$, cell 1): unused in relaxation ($fcx(1) = 0$, $gcx(1) = 0$).

---

## 2. Boundary Array Storage & Time Interpolation

### 2.1 Storage Convention
Lateral boundary data from `wrfbdy_d01` is stored in 4 boundary slabs:
- West (`XS`): dimensions $(ny, nz, 5)$, indices $(j, k, b)$
- East (`XE`): dimensions $(ny, nz, 5)$, indices $(j, k, b)$
- South (`YS`): dimensions $(nx, nz, 5)$, indices $(i, k, b)$
- North (`YE`): dimensions $(nx, nz, 5)$, indices $(i, k, b)$
where $b \in [1, 5]$ ($b=1$ is the outer boundary row/col, $b=5$ is innermost).
For 2D field $\mu$, the vertical dimension is $1$.

### 2.2 Boundary Variable Coupling in `wrfbdy_d01`
- **U**: Staggered U, $\mu$-coupled and map-scale divided: $U_{bdy} = \frac{\mu_u u}{m_{uy}}$
- **V**: Staggered V, $\mu$-coupled and map-scale divided: $V_{bdy} = \frac{\mu_v v}{m_{vx}}$
- **W**: Staggered W, $\mu$-coupled and map-scale divided: $W_{bdy} = \frac{\mu_w w}{m_{ty}}$
- **T**: Unstaggered perturbation dry potential temperature $\theta$, $\mu$-coupled: $T_{bdy} = \mu \theta$
- **PH**: Staggered W perturbation geopotential $\phi'$, $\mu$-coupled: $PH_{bdy} = \mu \phi'$
- **MU**: Unstaggered column dry-air mass perturbation $\mu$, uncoupled: $MU_{bdy} = \mu$
- **QVAPOR**: Unstaggered water vapor mixing ratio $q_v$, $\mu$-coupled: $Q_{bdy} = \mu q_v$

### 2.3 Time Interpolation
For time step beginning at $t$ and large-step increment $\Delta t$:
1. At the beginning of the big step (`solve_em.F:372`):
   $$dtbc = dtbc + \Delta t$$
   where $dtbc$ is elapsed seconds since the boundary interval start ($t_0$).
2. Linear interpolation for target boundary field $F$ at time $t + \Delta t$:
   $$F_{target}(b) = F_{start}(b) + dtbc \cdot \dot{F}_{tend}(b)$$
   where $F_{start}$ is `*_BXS/*_BXE/*_BYS/*_BYE` and $\dot{F}_{tend}$ is `*_BTXS/*_BTXE/*_BTYS/*_BTYE`.

---

## 3. Step-by-Step Chronological Execution Sequence

Below is the exact execution trace of every boundary-related routine throughout a full time step in `solve_em.F`:

### Phase 1: Time Step Initialization (Pre-RK)
1. **Elapsed boundary time update**:
   - `grid%dtbc = grid%dtbc + grid%dt` (`solve_em.F:372`)
2. **Kinematic Lower Boundary Condition for $W$**:
   - Call: `set_w_surface(config_flags, znw, fill_w_flag, w, ht, u, v, cf1, cf2, cf3, rdx, rdy, msftx, msfty, ...)` (`solve_em.F:436`)
   - Function: Computes terrain-following vertical velocity at the surface ($k=0$ in C++, $k=1$ in Fortran):
     $$w(i, 0, j) = m_{ty}(i,j) \cdot v_{sfc} \cdot \frac{\partial h_t}{\partial y} + m_{tx}(i,j) \cdot u_{sfc} \cdot \frac{\partial h_t}{\partial x}$$
   - Non-periodic boundary limits: Boundary derivatives $\partial h_t/\partial x$ and $\partial h_t/\partial y$ clamp to domain limits (`im1_limit = ids`, `ip1_limit = ide-1`, `jm1_limit = jds`, `jp1_limit = jde-1`).
3. **Moisture Halo / Boundary Prep (if moisture active)**:
   - Call: `set_physical_bc3d(moist_old, 'p', config_flags, ...)` (`solve_em.F:537`)

---

### Phase 2: RK3 Integration Loop (`rk_step = 1, 2, 3`)

#### Sub-Phase 2A: Large-Step Boundary Synchronization & Preparation
1. **Halo Extrapolation for Coupling State**:
   - Call: `rk_phys_bc_dry_1(config_flags, ru, rv, rw, ww, muu, muv, mut, php, alt, p, ...)` (`solve_em.F:721`)
   - Calls `set_physical_bc3d` for `ru` ('u'), `rv` ('v'), `rw` ('w'), `ww` ('w'), `php` ('w'), `alt` ('t'), `p` ('p'), and `set_physical_bc2d` for `muu` ('u'), `muv` ('v'), `mut` ('t').
   - For `specified = .true.`: Performs flat extrapolation into domain halos (3 halo cells outward: $i < 0$ or $i \ge nx$, $j < 0$ or $j \ge ny$).
2. **Diagnostics Halo Updates**:
   - `set_physical_bc3d(rho, 'p')` (`solve_em.F:729`)
   - `set_physical_bc3d(al, 'p')` (`solve_em.F:736`)
   - `set_physical_bc3d(ph_2, 'w')` (`solve_em.F:743`)
3. **Staggered Dry-Air Column Mass (`calc_mu_uv`)**:
   - Call: `calc_mu_uv(config_flags, mu, mub, muu, muv, ...)`
   - Computes:
     - Interior U: $\mu_u(i,j) = 0.5 \cdot (\mu(i,j) + \mu(i-1,j) + \mu_b(i,j) + \mu_b(i-1,j))$
     - Interior V: $\mu_v(i,j) = 0.5 \cdot (\mu(i,j) + \mu(i,j-1) + \mu_b(i,j) + \mu_b(i,j-1))$
     - Non-periodic West U ($i = 0$): $\mu_u(0,j) = \mu(0,j) + \mu_b(0,j)$
     - Non-periodic East U ($i = nx$): $\mu_u(nx,j) = \mu(nx-1,j) + \mu_b(nx-1,j)$
     - Non-periodic South V ($j = 0$): $\mu_v(i,0) = \mu(i,0) + \mu_b(i,0)$
     - Non-periodic North V ($j = ny$): $\mu_v(i,ny) = \mu(i,ny-1) + \mu_b(i,ny-1)$

#### Sub-Phase 2B: Large-Step Dycore Tendencies (Interior alterations detailed in Section 4)
- Tendency terms calculated: `horizontal_pressure_gradient`, `coriolis`, `curvature`, `rhs_ph`, `advect_u`, `advect_v`, `advect_w`, `advect_scalar`.
- All outer boundary rows/cols are intentionally skipped or computed with degraded stencils (see Section 4).

#### Sub-Phase 2C: Lateral Boundary Coupling (Tendency Insertion)
1. **Newtonian Relaxation & Error Diffusion (`relax_bdy_dry`)**:
   - **Condition:** Executed ONLY when `rk_step == 1` (`solve_em.F:932-956`).
   - Call:
     ```fortran
     CALL relax_bdy_dry( config_flags, &
          ru_tend, rv_tend, ph_tend, t_tend, rw_tend, mu_tend, &
          c1h, c2h, c1f, c2f, ru, rv, ph, t, w, mu, mut, &
          u_bxs, u_bxe, u_bys, u_bye, ... &
          spec_bdy_width, spec_zone, relax_zone, dtbc, fcx, gcx, ... )
     ```
   - Operations:
     - Applies `relax_bdytend_core` to `ru_tend` ('u') and `rv_tend` ('v') using `ru` and `rv`.
     - Mass-weights $ph$ (`rfield = ph * (c1f * mut + c2f)`) and applies `relax_bdytend_core` to `ph_tend` ('h').
     - Mass-weights $t$ (`rfield = t * (c1h * mut + c2h)`) and applies `relax_bdytend_core` to `t_tend` ('t').
     - Applies `relax_bdytend_core` to `mu_tend` ('m') using uncoupled `mu`.
     - (Note: for real-data specified runs, `w` is NOT relaxed; `rw_tend` is only relaxed for nested grids).
2. **Specified Boundary Tendencies (`spec_bdy_dry`)**:
   - **Condition:** Executed on **every RK step** (`rk_step == 1, 2, 3`) (`solve_em.F:977-994`).
   - Call:
     ```fortran
     CALL spec_bdy_dry( config_flags, &
          ru_tend, rv_tend, ph_tend, t_tend, rw_tend, mu_tend, &
          u_bxs, u_bxe, u_bys, u_bye, ... &
          spec_bdy_width, spec_zone, ... )
     ```
   - Operations:
     - Calls `spec_bdytend` for `ru_tend` ('u'), `rv_tend` ('v'), `ph_tend` ('h'), `t_tend` ('t'), `mu_tend` ('m').
     - Sets outer boundary cells ($b\_dist = 0$):
       $$\text{field\_tend}(i,k,j) = \text{field\_bdy\_tend}(*, k, 1)$$
3. **Moisture / Scalar Boundary Coupling (if active)**:
   - In `rk_step == 1`: `relax_bdy_scalar(moist_tend, moist, mu, ...)` (`solve_em.F:2259`)
   - In every RK step: `spec_bdy_scalar(moist_tend, ...)` (`solve_em.F:2276`)

---

### Phase 3: Acoustic Small Step Integration (`small_step`)
Inside each RK stage, $n_{small}$ substeps integrate acoustic modes with time step $dts$:

#### Sub-Phase 3A: Pre-Small Step Halo Synchronization
Before entering small steps (`solve_em.F:1181-1251`), boundary halos are filled via `set_physical_bc*`:
- `set_physical_bc3d(ru_tend, 'u')`
- `set_physical_bc3d(rv_tend, 'v')`
- `set_physical_bc3d(ph_2, 'w')`
- `set_physical_bc3d(al, 'p')`
- `set_physical_bc3d(p, 'p')`
- `set_physical_bc3d(t_1, 'p')`
- `set_physical_bc3d(t_save, 't')`
- `set_physical_bc2d(mu_1, 't')`
- `set_physical_bc2d(mu_2, 't')`
- `set_physical_bc2d(mudf, 't')`

#### Sub-Phase 3B: Acoustic Substep Loop ($m = 1 \dots n_{small}$)
1. **Advance Momentum ($u, v$)**:
   - `advance_uv`: Updates interior U and V points using pressure gradients.
     - $u$ loop: $i \in [1, nx-1]$, $j \in [1, ny-2]$ (skips outer $i=0, nx$ and $j=0, ny-1$).
     - $v$ loop: $i \in [1, nx-2]$, $j \in [1, ny-1]$ (skips outer $i=0, nx-1$ and $j=0, ny$).
2. **Specified Boundary Update for Horizontal Momentum**:
   - Calls (`solve_em.F:1340-1350`):
     ```fortran
     CALL spec_bdyupdate(grid%u_2, grid%ru_tend, dts_rk, 'u', config_flags, spec_zone, ...)
     CALL spec_bdyupdate(grid%v_2, grid%rv_tend, dts_rk, 'v', config_flags, spec_zone, ...)
     ```
   - Advances outer boundary cells using:
     $$u_2(i,k,j) = u_2(i,k,j) + dts_{rk} \cdot ru\_tend(i,k,j)$$
     $$v_2(i,k,j) = v_2(i,k,j) + dts_{rk} \cdot rv\_tend(i,k,j)$$
3. **Advance Mass and Potential Temperature ($\mu, \theta$)**:
   - `advance_mu_t`: Updates interior points $i \in [1, nx-2]$, $j \in [1, ny-2]$. Outer cells are untouched.
4. **Specified Boundary Update for Mass and Temperature**:
   - Calls (`solve_em.F:1456-1476`):
     ```fortran
     CALL spec_bdyupdate(grid%t_2, t_tend, dts_rk, 't', config_flags, spec_zone, ...)
     CALL spec_bdyupdate(grid%mu_2, mu_tend, dts_rk, 'm', config_flags, spec_zone, ...)
     CALL spec_bdyupdate(grid%muts, mu_tend, dts_rk, 'm', config_flags, spec_zone, ...)
     ```
   - Advances outer boundary cells with $dts_{rk} \cdot tend$.
5. **Advance Vertical Velocity and Geopotential ($w, \phi$)**:
   - `advance_w`: Updates interior points $i \in [1, nx-2]$, $j \in [1, ny-2]$. Outer cells are untouched.
6. **Specified Boundary Update for Geopotential ($\phi$)**:
   - Call (`solve_em.F:1581`):
     ```fortran
     CALL spec_bdyupdate_ph(ph_save, grid%ph_2, ph_tend, mu_tend, grid%muts, c1f, c2f, dts_rk, 'h', config_flags, spec_zone, ...)
     ```
   - In outer boundary cells:
     $$\mu_{old}(i,j) = \mu_{ts}(i,j) - dts_{rk} \cdot \mu\_tend(i,j)$$
     $$ph_2(i,k,j) = ph_2(i,k,j) \cdot \frac{c_{1f}(k) \mu_{old} + c_{2f}(k)}{c_{1f}(k) \mu_{ts} + c_{2f}(k)} + dts_{rk} \cdot \frac{ph\_tend(i,k,j)}{c_{1f}(k) \mu_{ts} + c_{2f}(k)} + ph\_save(i,k,j) \cdot \left(\frac{c_{1f}(k) \mu_{old} + c_{2f}(k)}{c_{1f}(k) \mu_{ts} + c_{2f}(k)} - 1\right)$$
7. **Vertical Velocity Boundary Condition ($w$)**:
   - Call (`solve_em.F:1593`):
     ```fortran
     CALL zero_grad_bdy(grid%w_2, 'w', config_flags, spec_zone, ...)
     ```
   - Sets zero horizontal gradient on $w_2$ at outer domain boundary:
     - West ($i=0$): $w_2(0, k, j) = w_2(1, k, j)$
     - East ($i=nx-1$): $w_2(nx-1, k, j) = w_2(nx-2, k, j)$
     - South ($j=0$): $w_2(i, k, 0) = w_2(i, k, 1)$
     - North ($j=ny-1$): $w_2(i, k, ny-1) = w_2(i, k, ny-2)$

#### Sub-Phase 3C: Post-Small Step Halo Synchronization
- Post-acoustic updates: `set_physical_bc3d` for `ph_2`, `al`, `p`, and `set_physical_bc2d` for `muts`, `mu_2`, `mudf` (`solve_em.F:1665-1703`).
- Mass flux boundary halo fills: `set_physical_bc3d(ru_m, 'u')`, `set_physical_bc3d(rv_m, 'v')` (`solve_em.F:1757-1765`).

#### Sub-Phase 3D: Scalar Advection & Flow-Dependent Boundary
- Moisture / scalar advection: `advect_scalar` uses time-averaged mass fluxes `ru_m`, `rv_m`, `ww_m`.
- If moisture has no specified boundaries (`.NOT. have_bcs_moist`):
  - Call: `flow_dep_bdy(moist, u, v, config_flags, spec_zone, ...)` (`solve_em.F:2339`)
  - Sets outflow points to zero-gradient ($field_{bdy} = field_{interior}$) and inflow points to zero ($field_{bdy} = 0$).

---

### Phase 4: Time Step Finalization (Post-RK)

1. **Halo Updates**:
   - `rk_phys_bc_dry_2`: fills halos for $u, v, w, t, ph, mu$ (`solve_em.F:3240`).
   - `set_phys_bc_dry_2`: fills halos for $u_1, u_2, v_1, v_2, w_1, w_2, t_1, t_2, \dots$ (`solve_em.F:4452`).
2. **Exact Boundary Replacement (`spec_bdy_final`)**:
   - **Condition:** Executed once at the end of the time step (`solve_em.F:4540-4612`).
   - Directly overwrites outer boundary cells ($b\_dist = 0$) using boundary file values interpolated to $t+\Delta t$ ($dtbc$):
     - For $u_2$:
       $$u_2(i,k,j) = m_{sfu\_y}(i,j) \cdot \frac{U_{bdy}(j,k,1) + dtbc \cdot \dot{U}_{tend}(j,k,1)}{c_{1h}(k) \mu_u(i,j) + c_{2h}(k)}$$
     - For $v_2$:
       $$v_2(i,k,j) = m_{sfv\_x}(i,j) \cdot \frac{V_{bdy}(i,k,1) + dtbc \cdot \dot{V}_{tend}(i,k,1)}{c_{1h}(k) \mu_v(i,j) + c_{2h}(k)}$$
     - For $w_2$:
       $$w_2(i,k,j) = m_{sft\_y}(i,j) \cdot \frac{W_{bdy}(*,k,1) + dtbc \cdot \dot{W}_{tend}(*,k,1)}{c_{1f}(k) \mu_t(i,j) + c_{2f}(k)}$$
     - For $t_2$:
       $$t_2(i,k,j) = \frac{T_{bdy}(*,k,1) + dtbc \cdot \dot{T}_{tend}(*,k,1)}{c_{1h}(k) \mu_t(i,j) + c_{2h}(k)}$$
     - For $ph_2$:
       $$ph_2(i,k,j) = \frac{PH_{bdy}(*,k,1) + dtbc \cdot \dot{PH}_{tend}(*,k,1)}{c_{1f}(k) \mu_t(i,j) + c_{2f}(k)}$$
     - For $mu_2$:
       $$mu_2(i,j) = MU_{bdy}(*,1,1) + dtbc \cdot \dot{MU}_{tend}(*,1,1)$$
   - This eliminates all numerical roundoff accumulation, guaranteeing exact agreement with boundary driving fields.

---

## 4. Interior Dycore Loop Limits & Degraded Stencils

When `config_flags%specified == .true.` on a real-data grid with rigid/specified lateral boundaries, the *interior* dycore routines alter their loop bounds and degrade horizontal stencils to avoid referencing nonexistent halo data beyond the physical boundary:

| Dycore Routine | Field | Fortran Loop Limits (`specified`) | C++ 0-Indexed Equivalent | Degraded Stencils & Special Handling |
|---|---|---|---|---|
| **`calc_mu_uv`** | `muu` | $i \in [ids+1, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [1, nx-1]$<br>$j \in [0, ny-1]$ | At $i=ids$ ($i=0$): $\mu_u = \mu + \mu_b$.<br>At $i=ide$ ($i=nx$): $\mu_u = \mu + \mu_b$. |
| | `muv` | $i \in [ids, ide-1]$<br>$j \in [jds+1, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [1, ny-1]$ | At $j=jds$ ($j=0$): $\mu_v = \mu + \mu_b$.<br>At $j=jde$ ($j=ny$): $\mu_v = \mu + \mu_b$. |
| **`horizontal_pressure_gradient`** | `ru_tend` | $i \in [ids+1, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [1, nx-1]$<br>$j \in [0, ny-1]$ | Outer U faces ($i=0, nx$) skipped. |
| | `rv_tend` | $i \in [ids, ide-1]$<br>$j \in [jds+1, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [1, ny-1]$ | Outer V faces ($j=0, ny$) skipped. |
| **`coriolis`** | `ru_tend` | $i \in [ids+1, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [1, nx-1]$<br>$j \in [0, ny-1]$ | Outer U faces skipped. |
| | `rv_tend` | $i \in [ids, ide-1]$<br>$j \in [jds+1, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [1, ny-1]$ | Outer V faces skipped. |
| | `rw_tend` | $i \in [ids, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [0, ny-1]$ | Standard interior mass points. |
| **`curvature`** | `vxgm` | $i \in [ids, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [0, ny-1]$ | Evaluated for interior cells; outer boundary cells filled by flat 0-order extrapolation: $vxgm(ids-1)=vxgm(ids)$, $vxgm(ide)=vxgm(ide-1)$, etc. |
| | `ru_tend` | $i \in [ids+1, ide-1]$<br>$j \in [jds, jde-1]$ | $i \in [1, nx-1]$<br>$j \in [0, ny-1]$ | Outer U faces skipped. |
| | `rv_tend` | $i \in [ids, ide-1]$<br>$j \in [jds+1, jde-1]$ | $i \in [0, nx-1]$<br>$j \in [1, ny-1]$ | Outer V faces skipped. |
| **`rhs_ph`** | `ph_tend` | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-2]$ | Full 5th/6th order horizontal flux advection in core interior ($i \in [ids+3, ide-4]$).<br>**Degradation:**<br>- At distance 2 ($ids+2, ide-3$): degrades to **4th-order** stencil.<br>- At distance 1 ($ids+1, ide-2$): degrades to **2nd-order** stencil.<br>- Row/col 0 ($ids, ide-1$): untouched (set by `spec_bdy_dry`). |
| **`advect_u`** | `tendency` (u) | $i \in [ids+1, ide-1]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-1]$<br>$j \in [1, ny-2]$ | **Y-advection flux:**<br>- Core interior: 5th/6th order `flux6`.<br>- At $j=jds+2, jde-2$: degrades to 4th order `flux4`.<br>- At $j=jds+1, jde-1$: degrades to 2nd order flux.<br>**X-advection flux:**<br>- Core interior: 5th/6th order `flux6`.<br>- At $i=ids+2, ide-1$: degrades to 4th order `flux4`.<br>- At $i=ids+1$: 2nd-order flux with upstream normal wind $u_b = u(ids)$ (if $u(ids+1) < 0$, $u_b = u(ids+1)$).<br>- At $i=ide$: 2nd-order flux with upstream normal wind $u_b = u(ide)$ (if $u(ide-1) > 0$, $u_b = u(ide-1)$). |
| **`advect_v`** | `tendency` (v) | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-1]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-1]$ | Symmetric analogue to `advect_u`:<br>- X-flux degrades to 4th order at $ids+2, ide-2$ and 2nd order at $ids+1, ide-1$.<br>- Y-flux degrades to 4th order at $jds+2, jde-1$ and 2nd order with upstream normal wind $v_b$ at $jds+1, jde$. |
| **`advect_w`** | `tendency` (w) | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-2]$ | Horizontal fluxes degrade to 4th order at distance 2 and 2nd order at distance 1. Row/col 0 untouched (filled by `zero_grad_bdy`). |
| **`advect_scalar`** | `tendency` (q) | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-2]$ | Mass-point scalar advection; horizontal stencils degrade to 4th order and 2nd order near boundaries. Outer cells set by `spec_bdy_scalar` or `flow_dep_bdy`. |
| **`advance_uv`** | $u$ | $i \in [ids+1, ide-1]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-1]$<br>$j \in [1, ny-2]$ | Skips outer U faces $i=0, nx$ and edge $j=0, ny-1$. |
| | $v$ | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-1]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-1]$ | Skips outer V faces $j=0, ny$ and edge $i=0, nx-1$. |
| **`advance_mu_t`** | $\mu, \theta$ | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-2]$ | Skips outer mass points $i=0, nx-1$ and $j=0, ny-1$. |
| **`advance_w`** | $w, \phi$ | $i \in [ids+1, ide-2]$<br>$j \in [jds+1, jde-2]$ | $i \in [1, nx-2]$<br>$j \in [1, ny-2]$ | Skips outer vertical columns $i=0, nx-1$ and $j=0, ny-1$. |

---

## 5. C++ Architecture and Data Interface

For implementation by Agent 0:
1. `src/bdy/bdy_data.hpp`: Reader-independent struct `BdyData` holding 4 lateral boundary slabs:
   - `west`, `east` (size: $ny \times nz \times 5$)
   - `south`, `north` (size: $nx \times nz \times 5$)
   Holding both base values ($V_{bdy}$) and tendencies ($\dot{V}_{tend}$) for variables: `u`, `v`, `w`, `t`, `ph`, `mu`, and optional scalars `q`.
2. `src/bdy/bdy_kernels.hpp`: Host/device kernels parameterized on precision `real` (FP32/FP64):
   - `calc_lbc_fcx_gcx`: Computes $f_{cx}$ and $g_{cx}$ profiles.
   - `relax_bdy_dry`: Computes Newtonian + Laplacian relaxation tendencies in cells $2..5$.
   - `spec_bdy_dry` / `spec_bdy_scalar`: Injects specified boundary tendencies in cell $1$.
   - `spec_bdyupdate` / `spec_bdyupdate_ph`: Advances acoustic boundary states during small steps.
   - `zero_grad_bdy`: Enforces zero horizontal gradient on $w$.
   - `flow_dep_bdy`: Flow-dependent inflow/outflow condition for uncoupled scalars.
   - `spec_bdy_final`: Overwrites outer boundary row with exact interpolated values.
   - `set_physical_bc3d` / `set_physical_bc2d`: Flat halo filling for non-periodic specified grids.
