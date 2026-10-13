# A6 (Agent 7): why does the CPPWRF squall2d initial state differ from WRF em_squall2d_x?

In A4 your squall setup (80 levels, dx 1 km) differed from WRF already at t = 0 (theta max
1.04 K, RMS 0.28 K). For dry cases (Straka, hill2d) CPPWRF's ideal init matches WRF; the moist
ideal init (src/cases/cases.cpp squall2d / quarter_ss; WRF dyn_em/module_initialize_ideal.F
em_squall2d_x, with input_sounding) has never been compared at t = 0.
1. Build the exactly matching pair: WRF em_squall2d_x with its default namelist.input and
   input_sounding (~/wrf_ref has the ideal builds), and CPPWRF configs/squall2d.ini adjusted to
   the same grid, levels (WRF's eta levels: check znw in the WRF output), sounding, bubble,
   p_top, dt. Use r8 WRF if available (or build ideal.exe/wrf.exe r8 in a scratch copy).
2. Compare the t = 0 state field by field (T/THM, QVAPOR, P, PB, PH, PHB, MU, MUB, U, ALB, ...)
   and find every difference; for each, the exact cause with lines on both sides (config
   mismatch vs code difference). A code difference is a CPPWRF bug: describe the fix precisely
   (Agent 0 or Agent 3 will apply it; you only write the report and test scripts).
3. Once t = 0 matches (with a fix applied in a scratch copy if needed), run 10 min and report
   theta/w differences: no microphysics first (mp_physics 0, only qv advected), then Kessler.
Report: docs/audit/squall_init.md. Rules as always.
