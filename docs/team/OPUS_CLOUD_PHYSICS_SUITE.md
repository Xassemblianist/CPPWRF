# Task for a cloud Claude (Opus) session: WRF-identical physics driver glue (P1 suite)

Repository: https://github.com/Xassemblianist/CPPWRF (C++20/CUDA re-implementation of WRF
v4.6.1 ARW; results must be identical to WRF, see docs/STATUS.md, docs/PHYSICS_DRIVER.md,
docs/NESTING.md for the house style). Budget is limited: work focused, no exploratory rewrites.

## Rules
- Commits: author `Xassemblianist <omerkaan20102003@gmail.com>`
  (`git -c user.name=Xassemblianist -c user.email=omerkaan20102003@gmail.com commit ...`),
  NO "Co-Authored-By" lines. Work on a new branch `opus/physics-suite-p1` from `main`, push it,
  open a PR to main at the end. Do not touch src/dyn/ (dycore, owned by Agent 0) except to read.
- Same results as WRF: a port is correct only if a differential test against the ORIGINAL WRF
  Fortran (compiled unmodified with gfortran -fdefault-real-8) gives bit-exact or <= 1e-12 rel
  in FP64. "Plausible" or "within a few %" is a failure.
- Code must stay single-source CPU/CUDA (kernels = trivially copyable structs launched with
  parallel_for/column_for, see src/core/backend.hpp; no host pointers in device code; no
  std::string/std::vector in kernels; literals as real(...); FP32 build must compile).
  The cloud machine may have no GPU/nvcc: build and test the CPU FP64 and FP32 configs
  (cmake -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON/OFF).

## Starting point
The physics suite draft is on branch `wip/agents-2026-10-13` (written by a weaker agent, NOT
correct): src/phys/driver/suite.{hpp,cpp}, phy_prep.*, tendency_coupling.*, physics_init.*,
plus the verified column schemes on main: src/phys/sf_sfclayrev, bl_ysu, lsm_noah, cu_kf (each
bit-exact vs Fortran in tests/phys/*). The dycore calls the suite through
src/dyn/column_physics.hpp (ColumnPhysics: part1 = first_rk_step_part1 after
init_zero_tendency; part2 = calculate_phy_tend + update_phy_ten before the stage-1 diffusion;
after_rk = phy_prep_part2), see src/dyn/rk3.cpp and src/dyn/large_step.cpp.
Known defects of the draft: SFCLAYREV gets p_hyd instead of p_phy; ch = chs after SFCLAYREV
missing; host table pointer in device code; Field2 temporaries allocated every step; FP32 does
not compile (View1<const double>); kernels carrying std::string tables; a real-data step-1
comparison against WRF r8 differs by 10-50 % in PBLH, RTHBLTEN, RVBLTEN (so the glue is wrong).

## The task
Make the physics driver glue for this option set exactly WRF's:
`sf_sfclay_physics = 1 (sfclayrev), sf_surface_physics = 2 (Noah, num_soil_layers 4),
bl_pbl_physics = 1 (YSU), bldt = 0, isfflx = 1, fractional_seaice = 0, mp/cu/ra = 0`,
real data (specified boundaries), use_theta_m = 1.
1. Download WRF v4.6.1 source (https://github.com/wrf-model/WRF, tag v4.6.1). Read
   dyn_em/module_first_rk_step_part1.F, part2.F, phys/module_surface_driver.F,
   phys/module_pbl_driver.F, phys/module_physics_addtendc.F, dyn_em/module_big_step_utilities_em.F
   (phy_prep, phy_prep_part2), dyn_em/module_em.F (calculate_phy_tend). List every statement
   these options execute (pre-processing, the scheme calls with their exact arguments,
   post-processing, what persists between steps, itimestep/stepbl logic).
2. Rewrite src/phys/driver/suite.* (and the helpers) as a statement-by-statement port of that
   path, device-resident, persistent fields allocated once, tables uploaded once.
3. Differential test (tests/phys/suite/): a Fortran driver that calls the ORIGINAL
   surface_driver + pbl_driver (+ phy_prep, calculate_phy_tend, update_phy_ten) on a synthetic
   but physical 3D tile (e.g. 24x20x44, terrain, land/water/sea-ice/snow points, stable and
   unstable columns, realistic soil states; Noah tables LANDUSE/VEGPARM/SOILPARM/GENPARM.TBL from
   WRF run/), for 3 consecutive physics calls with state carried over, and the C++ suite on the
   same inputs: compare every output (HFX, QFX, LH, UST, ZNT, MOL, PBLH, TSK, TSLB, SMOIS, SH2O,
   CHS, CHS2, CQS2, QSFC, T2, Q2, U10, V10, RUBLTEN, RVBLTEN, RTHBLTEN, RQVBLTEN, the coupled
   *_tendf fields). Pass = bit-exact or <= 1e-12 rel in CPU FP64. Compiling the WRF driver
   modules standalone needs stubs for module_state_description / config flags: keep the
   physics statements untouched and list every stub.
4. Write docs/PHYSICS_SUITE.md: the call map (WRF file:line -> C++ file:line) and the test
   commands. Report the test table in the PR description.
Out of scope for this session: KF, RRTMG, WSM6/Thompson coupling (Agent 0 wires them after),
the dycore, nesting, I/O.
