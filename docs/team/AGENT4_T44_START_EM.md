# Agent 4 task T4.4: real-data model start (WRF start_em) in CPPWRF, verified against WRF t = 0

After T4.3. Ownership: src/cases/real_case.{hpp,cpp} (new), configs/real_*.ini (new), and at
most ~20 lines in src/cases/model.cpp to register case = real (exception granted). Use Agent 6's
reader (src/io_netcdf/wrf_readers.hpp, read_wrfinput) - do not write another reader.

1. Port what dyn_em/start_em.F does for a real-data, single-domain, non-restart run with
   use_theta_m = 1, hypsometric_opt = 2 (WRF default for real data) and 1, hybrid_opt = 0 and 2:
   which fields come from wrfinput unchanged, which are recomputed (pb, alb from t_init, phb,
   al, alt, p, t_2 -> theta_m conversion, ph_1/mu_1/t_1 copies, w from set_w_surface, ...), in
   WRF order and with WRF's formulas, into CPPWRF's BaseState (non-flat, mapped = true: fill
   msftx..msfvx_inv, f, e, sina, cosa, ht with fill_host_halo) and State (src/dyn/state.hpp).
   The 1D vertical arrays (znw, znu, dn, dnw, rdn, rdnw, fnm, fnp, c1..c4 h/f, cf1..cfn) are
   taken from wrfinput, as WRF does.
2. Verify the CPPWRF state after start against WRF's own t = 0 output (wrfout frame 0 of your R0
   and R0_r8 runs, which is written after start_em): every field, max abs / max rel.
   Target with R0_r8 and a CPPWRF FP64 build: <= 1e-12 rel where WRF stores the same quantity.
3. Write configs/real_r0.ini (wrfinput path, dt and time_step_sound as R0, physics off,
   diffusion off, damp_opt 0, top_lid false, spec_bdy_width 5) for Agent 0.
Report: docs/team/AGENT4_REPORT.md, section T4.4. Then stop.
