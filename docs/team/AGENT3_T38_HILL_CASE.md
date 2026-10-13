# Agent 3 task T3.8: em_hill2d_x initial state in CPPWRF (terrain), verified against WRF t = 0

Agent 0 has ported terrain, map factors, Coriolis and damp_opt=3 into the dycore (d515110). The
missing piece for a terrain test is the *initial state* of WRF's hill case. You write it.

Ownership exception (granted): new files src/cases/hill2d.{hpp,cpp}, a new config
configs/hill2d_p0.ini, and at most ~15 lines in src/cases/cases.cpp / model.cpp to register
the case and to build the BaseState with flat = false. Do not touch src/dyn or src/core.

1. Port module_initialize_ideal.F, case hill2d_x, hybrid_opt = 0 path, to C++: terrain
   h(i) = hm / (1 + ((i - icm)/xa)^2) (your T3.7 report), phb(surface) = g h, the base state
   (mub, pb, alb, phb, t_init) from the input_sounding of ~/wrf_ref/run_hill_p0, the full state
   (mu, t, ph, al, p, u = 10 m/s), exactly as WRF does it (same iteration counts, same
   interpolation of the sounding, same p_top computation). BaseState must be non-flat
   (BaseState(g, false)); fill b.ht with the terrain height (fill_host_halo, periodic in x).
2. configs/hill2d_p0.ini: nx = 201, nz = 40, dx = 2000, dt = 20, time_step_sound = 6,
   periodic x, the WRF ZNW levels (domain.eta = list), diff off, damp_opt = 0, 3600 s,
   out_interval every 30 steps (600 s).
3. Verify t = 0 against ~/wrf_ref/run_hill_p0/ref_fields.npz and the wrfout (MUB, PHB, PH, T,
   U, W, P, PB, AL): report max abs / max rel per field. Target: FP64 build agrees with WRF
   (FP32 output) to FP32 roundoff (rel <= 1e-6). Explain every larger difference.
4. Then run 3600 s and report the comparison with run_hill_p0 at 600..3600 s (RMS and max of
   W, U, T') with a script scripts/compare_hill.py. Do NOT change dycore code to make it match:
   if it does not match, report it to Agent 0 with the evidence (time of first divergence,
   location, field).
Report: docs/team/AGENT3_REPORT_T38.md. Then stop.
