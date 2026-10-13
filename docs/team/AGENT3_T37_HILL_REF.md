# Agent 3 task T3.7: WRF terrain reference (em_hill2d_x)

Agent 0 is adding terrain, map factors, Coriolis and curvature to the CPPWRF dycore. Provide a
WRF reference for flow over a 2D hill. Your T3.3 work is committed (166a615).

1. Make a separate WRF copy for the ideal hill case: ~/wrf_ref/WRF_hill (cp -a of ~/wrf_ref/WRF
   minus main/*.exe is fine, then ./clean -a and configure 32 serial, nesting 0),
   `tools/heavy.sh ./compile em_hill2d_x` (-j4 max). Do not touch ~/wrf_ref/WRF.
2. Run directory ~/wrf_ref/run_hill: the default em_hill2d_x namelist and input_sounding,
   except: history every 600 s, run 3600 s total (or the default length if shorter), and record
   whatever diff_opt/km_opt/damp_opt the default uses. Then a second run run_hill_nodiff with
   diff_opt=0, damp_opt=0 (if it stays stable; if not, report it). Every run via tools/heavy.sh.
3. Write down exactly (from dyn_em/module_initialize_ideal.F, case hill2d_x): the terrain formula
   (hm, xa, centre index), the eta levels (ZNW from wrfout), p_top, the sounding, dx, dt,
   time_step_sound, nx, nz, bc. Agent 0 will reproduce it in CPPWRF from your report.
4. From the wrfout files, extract to ~/wrf_ref/run_hill/ref_fields.npz: W, U, T, PH at each
   history time (j = 0 slice), plus HGT, ZNW, MUB, PHB at t = 0 (numpy, script in scripts/).

Report: docs/team/AGENT3_REPORT_T37.md (<= 60 lines). Then stop.
