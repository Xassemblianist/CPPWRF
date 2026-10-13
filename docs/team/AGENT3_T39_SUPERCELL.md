# Agent 3 task T3.9: 3D supercell (em_quarter_ss) reference and CPPWRF case

Your hill2d work is committed (ab9b5a9) and, after an Agent 0 fix of the top boundary, CPPWRF
matches your WRF hill run to w RMS 8e-5 m/s. Next: the first 3D moist validation.

1. WRF: separate copy ~/wrf_ref/WRF_qss (as for the hill), `compile em_quarter_ss` (-j4, heavy.sh).
   Run ~/wrf_ref/run_qss with the default sounding and bubble but: mp_physics=1 (Kessler),
   diff_opt=1, km_opt=1, khdif=kvdif=500 (or the value the default case uses for km_opt=1; write
   it down), damp_opt=3 (default zdamp/dampcoef), top_lid=.false., periodic_x/y, hybrid_opt=0,
   all other physics off, 2 h, history every 600 s; plus run_qss_steps (history every step,
   first 3 steps). Record the namelist, ZNW, P_TOP, dt, time_step_sound.
2. CPPWRF: new case "quarter_ss" (ownership exception like T3.8: src/cases/quarter_ss.{hpp,cpp},
   configs/quarter_ss.ini, <= 15 lines in cases.cpp/model.cpp). Port module_initialize_ideal.F
   case quarter_ss (input_sounding with u, v and qv, the bubble, the moist hydrostatic balance)
   exactly as WRF. Moist species qv, qc, qr; phys.mp_physics = 1; dyn.damp_opt = 3.
3. Verify t = 0 against WRF (all fields, FP32-roundoff target), then compare every 600 s (w, u,
   v, theta, qr, RAINNC: RMS, max, and max w / total rain time series) with a script
   scripts/compare_qss.py, FP64 and FP32, CPU and CUDA. Deep convection is chaotic: report
   when the RMS difference starts to grow and how it compares with CPPWRF FP32-vs-FP64.
   Do not change dycore code; report suspected dycore differences to Agent 0 with evidence
   (first step and location of divergence from the _steps run).
Report: docs/team/AGENT3_REPORT_T39.md. Then stop.
