# Agent 3 Report: Task T3.7 (WRF em_hill2d_x Terrain Reference)

## 1. WRF Build & Execution
- Built in `~/wrf_ref/WRF_hill` (copy of WRF, `clean -a`, GNU serial 32, nesting 0).
  Compiled with `tools/heavy.sh ./compile em_hill2d_x` (`ideal.exe` and `wrf.exe` created).
- Run `run_hill` (3600 s, dt=20 s, 180 steps, history=600 s): SUCCESS.
  Defaults: `diff_opt=2`, `km_opt=1`, `khdif=0`, `kvdif=0`, `damp_opt=3`, `zdamp=20000.`, `dampcoef=0.1`.
- Run `run_hill_nodiff` (`diff_opt=0`, `damp_opt=0`, 3600 s): STABLE, completed 3600 s without blowup.
  W range: [-0.115, 0.112] m/s vs default [-0.091, 0.081] m/s.

## 2. Terrain & Grid Specification (dyn_em/module_initialize_ideal.F)
- Terrain formula: $h(i, j) = h_m / (1 + ((i - i_{cm}) / x_a)^2)$
  $h_m = 100.0\text{ m}$, $x_a = 5.0$ cells ($a = 10.0\text{ km}$), $i_{cm} = \text{ide}/2 = 101$ (1-based, $i \in [1, 201]$; index 100 in 0-based).
  Surface geopotential: $\phi_b(i, 1, j) = g \cdot h(i, j)$ ($g = 9.81\text{ m/s}^2$), $\phi'(i, 1, j) = 0$.
- Grid: $dx = dy = 2000.0\text{ m}$, $nx = 201$ (`e_we=202`), $ny = 1$ (`e_sn=3`), $nz = 40$ (`e_vert=41`).
- Time stepping: $dt = 20.0\text{ s}$, acoustic substeps $n_{sound} = 6$ (`time_step_sound=6`).
- Boundary conditions:
  East-west: Open (`open_xs = .true.`, `open_xe = .true.`, `periodic_x = .false.`).
  North-south: Periodic (`periodic_y = .true.`).
  Top: Damped layer (`damp_opt = 3`, `zdamp = 20000.0\text{ m}`, `dampcoef = 0.1`).

## 3. Vertical Coordinate & Sounding
- Top boundary & pressure: $z_{top} = 30000.0\text{ m}$, $p_{top} = 66.368965\text{ Pa}$ (`P_TOP`).
- Coordinate: Hybrid (`hybrid_opt = 2`), stretching $z_{scale} = 8000.0 / z_{top} = 4/15 \approx 0.2666667$.
  Formula ($kde=41$): $\eta(k) = (\exp(-(k-1)/((kde-1) z_{scale})) - \exp(-1/z_{scale})) / (1 - \exp(-1/z_{scale}))$.
  Levels $\eta(1..41)$ [surface to top]:
  [1.0, 0.908355, 0.824911, 0.748935, 0.679758, 0.616771, 0.559421, 0.507204, 0.459659, 0.416369,
   0.376953, 0.341064, 0.308387, 0.278635, 0.251544, 0.226878, 0.204420, 0.183971, 0.165352, 0.148400,
   0.132964, 0.118910, 0.106114, 0.094462, 0.083854, 0.074194, 0.065399, 0.057391, 0.050100, 0.043462,
   0.037417, 0.031913, 0.026902, 0.022339, 0.018185, 0.014402, 0.010958, 0.007822, 0.004967, 0.002367, 0.0]
- Sounding (`input_sounding`):
  Surface: $p_{sfc} = 1000.0\text{ mb} = 100000.0\text{ Pa}$, $\theta_{sfc} = 288.0\text{ K}$, $q_{vs} = 0.0$.
  Profile: $U = 10.0\text{ m/s}$ (uniform), $V = 0.0$, $q_v = 0.0$. Constant $N = 0.01\text{ s}^{-1}$:
  $\theta(z) = \theta_0 \exp(N^2 z / g)$ with $\theta(0) = 288.0\text{ K}$, $\theta(30000) = 391.08\text{ K}$.

## 4. Reference Field Dataset
- Extractor script: `scripts/extract_hill_ref.py`.
- Saved file: `~/wrf_ref/run_hill/ref_fields.npz` (929 KB).
- Contents:
  - `W`: shape (7, 41, 201), times 0 to 3600 s every 600 s ($j=0$ slice).
  - `U`: shape (7, 40, 202), times 0 to 3600 s every 600 s ($j=0$ slice).
  - `T`: shape (7, 40, 201), perturbation theta ($\theta - 300\text{ K}$).
  - `PH`: shape (7, 41, 201), perturbation geopotential.
  - $t=0$ base arrays: `HGT` (201,), `ZNW` (41,), `MUB` (201,), `PHB` (41, 201).

## 5. T3.7b Follow-up: Periodic Lateral BCs & hybrid_opt=0

### Runs & Stability
- `run_hill_p`: `periodic_x=.true.`, `open_xs=open_xe=.false.`, `hybrid_opt=0`, default diff/damp (`diff_opt=2`, `km_opt=1`, `damp_opt=3`, `zdamp=20000.`, `dampcoef=0.1`). 3600 s (dt=20 s, 180 steps, history=600 s). SUCCESS. $W \in [-0.0898, 0.0803]\text{ m/s}$, $U \in [9.601, 10.773]\text{ m/s}$.
- `run_hill_p0`: `periodic_x=.true.`, `open_xs=open_xe=.false.`, `hybrid_opt=0`, `diff_opt=0`, `damp_opt=0`. 3600 s (history=600 s). STABLE, completed 3600 s without blowup. $W \in [-0.1054, 0.1010]\text{ m/s}$, $U \in [9.601, 10.774]\text{ m/s}$.
- `run_hill_p0_steps`: Identical setup to `run_hill_p0`, first 3 time steps only (60 s total, dt=20 s), `history_interval_s=20`. 4 frames (t = 0, 20, 40, 60 s):
  - $t=0\text{ s}$: $W \in [-0.0631, 0.0631]$, $U \in [10.000, 10.000]$, $T \in [-10.84, 84.54]$, $\text{PH} \in [-0.0025, 0.0042]$
  - $t=20\text{ s}$: $W \in [-0.0631, 0.0631]$, $U \in [9.983, 10.058]$, $T \in [-10.84, 84.54]$, $\text{PH} \in [-6.093, 6.113]$
  - $t=40\text{ s}$: $W \in [-0.0633, 0.0633]$, $U \in [9.965, 10.098]$, $T \in [-10.84, 84.54]$, $\text{PH} \in [-6.159, 6.240]$
  - $t=60\text{ s}$: $W \in [-0.0634, 0.0649]$, $U \in [9.959, 10.114]$, $T \in [-10.84, 84.54]$, $\text{PH} \in [-5.382, 5.955]$

### Coordinate & Base State Arrays (hybrid_opt=0)
- `P_TOP`: `66.368965 Pa` (identical in all runs).
- `ZNW`: 41 levels, analytical formula identical to Section 3 ($\eta \in [1.0, 0.0]$).
- `MUB`: Shape (201,), bitwise identical across all 3 periodic runs:
  - Centre minimum (mountain peak, $i=101$): `98752.4062 Pa`.
  - Outer boundary (flat ground, $i=1, 201$): `99930.6875 Pa`.
- `PHB`: Shape (41, 201), bitwise identical across all 3 periodic runs:
  - Surface ($k=1$): $\text{PHB}(i, 1) = g \cdot h(i) \in [2.4464, 981.0000]\text{ m}^2/\text{s}^2$.
  - Model top ($k=41$): Uniform across all columns $= 291103.2812\text{ m}^2/\text{s}^2$ ($z_{top} = 29674.14\text{ m}$).

### Extracted Datasets
- `~/wrf_ref/run_hill_p/ref_fields.npz`: 7 slices (t=0..3600 s every 600 s), 929 KB.
- `~/wrf_ref/run_hill_p0/ref_fields.npz`: 7 slices (t=0..3600 s every 600 s), 929 KB.
- `~/wrf_ref/run_hill_p0_steps/ref_fields.npz`: 4 slices (t=0, 20, 40, 60 s), 533 KB.
