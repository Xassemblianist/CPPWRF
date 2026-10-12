# Agent 3 Report: Task T3.3 (Reference WRF Serial Build & Straka Run)

## 1. Build Commands & Result
- Cloned WRF v4.6.1 into `~/wrf_ref/WRF`. Symlinked NetCDF C/Fortran in `~/wrf_ref/netcdf`.
- Configured: `printf '32\n0\n' | ./configure` (GNU serial, nesting 0, no MPI/RSL).
- Compiled with `tools/heavy.sh`: `export NETCDF=~/wrf_ref/netcdf NETCDFF=~/wrf_ref/netcdf J="-j 4" && ./compile em_grav2d_x`.
- Result: SUCCESS (`main/ideal.exe` and `main/wrf.exe` created). OpenMP build deferred by Agent 0.

## 2. WRF namelist.input Values (test/em_grav2d_x, 900 s)
- Domains: `dx=100`, `dy=100`, `e_we=513` (nx=512), `e_sn=3` (ny=1), `e_vert=65` (nz=64), `ztop=6409.0`.
- Time: `time_step=1`, `run_seconds=900`, `history_interval_s=300`.
- Dynamics: `rk_ord=3`, `diff_opt=2`, `km_opt=1`, `khdif=75`, `kvdif=75`, `smdiv=0.1`, `emdiv=0.01`, `epssm=0.1`.
- Sound/Adv: `time_step_sound=6`, `h_mom_adv_order=5`, `v_mom_adv_order=3`, `h_sca_adv_order=5`, `v_sca_adv_order=3`.
- Physics & BCs: non-hydrostatic=.true., hybrid_opt=0, periodic_x=.true., periodic_y=.true., mp/ra/pbl=0.

## 3. Timing Output (measured with tools/heavy.sh --exclusive, 1 thread serial)
- `/usr/bin/time -v`: Wall 61.14 s | User 56.24 s | Sys 4.87 s | CPU 99% | Max RSS 273,708 KB.
- WRF "Timing for main" (900 steps): First step = 0.1300 s | Typical (median) = 0.0663 s | Mean = 0.0678 s | Total stepping = 61.00 s.

## 4. Comparison Table (t = 900 s, front at theta' <= -1 K linearly interpolated)
| Model | Precision | dx (m) | Front (km) | Min theta' (K) | Wall (s) | Speedup vs WRF |
|---|---|---|---|---|---|---|
| WRF v4.6.1 (serial) | FP64 | 100 | 14.989 | -7.458 | 61.14 | 1.00x |
| CPPWRF (`a0_straka_dp`) | FP64 | 100 | 15.346 | -9.456 | 31.40 | 1.95x |
| CPPWRF (`a0_straka`) | FP32 | 100 | 15.346 | -9.456 | 29.74 | 2.06x |
- Output plot saved to `out/plots/wrf_vs_cppwrf_straka.png`.

## 5. Differences: WRF em_grav2d_x vs CPPWRF configs/straka.ini
- Domain & Eta: WRF has ztop=6409m with hydrostatic pressure eta levels; CPPWRF has ztop=6400m uniform dz=100m.
- Thermal diffusion (Prandtl number): WRF `diff_opt=2` sets thermal diffusion with default Pr=1/3 ($K_{th} = 225\text{ m}^2/\text{s}$), diffusing $\theta'$ more. CPPWRF sets `pr_inv=1` ($K_{th} = 75\text{ m}^2/\text{s}$ per Straka et al. 1993), keeping $\theta'$ colder (-9.46 K vs -7.46 K).
- BCs & Adv: Both use periodic X, 5th-order horizontal, 3rd-order vertical advection, RK3 dt=1s, 6 acoustic sub-steps.
