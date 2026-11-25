# Nesting in CPPWRF: design

Scope: WRF 4.8.0 two-way (feedback = 1) and one-way (feedback = 0) static nests with odd
parent_grid_ratio, the same vertical levels in all domains (no vertical refinement), nests that read
their own wrfinput_d0N (input_from_file = .true.), real-data cases; prescribed moves of a nest of domain 1
(unverified, section "Moving nests" below). Not implemented: ideal-case nests, vortex-following moving nests, vertical
refinement.

## WRF call sequence (frame/module_integrate.F integrate, recursive per domain)
```
loop over parent steps:
  solve_interface(parent)                     one parent step (solve_em)
  for each child:
    med_nest_force(parent, child)             couple -> interp_domain_em_part1 (copy the coarse
                                              region onto the intermediate grid) ->
                                              force_domain_em_part2 -> nest_forcedown_interp.inc:
                                              bdy_interp for every field with a nest LBC
                                              -> uncouple
    integrate(child)                          parent_time_step_ratio child steps (recursive:
                                              the child forces/feeds back its own children)
    med_nest_feedback(parent, child)          if feedback: copy_fcn (+ smooth_option) of the
                                              child interior onto the parent
```
Nest LBC (share/interp_fcn.F bdy_interp1, SINT): for the boundary zone sz = min(max(spec_zone,
relax_zone+1), spec_bdy_width) = 5 of the child,
  value    = the child's current field (coupled),
  tendency = (sint(parent field at the new parent time) - child field) / dt_parent,
in the same _bxs/_bxe/_bys/_bye slabs as wrfbdy. dtbc restarts at 0 for each parent step and
advances by dt_child (so the child boundary moves linearly to the interpolated parent state).

## Differences of a nested domain from a specified (real-data) domain in solve_em
| piece | specified | nested |
|---|---|---|
| relax coefficients fcx, gcx | 0.1/dt, 1/dt/50 ramp x spongeweight(spec_exp) | same ramp, no spongeweight (module_bc_em.F 1312-1330) |
| W | zero_grad_bdy after advance_w | relax_bdy_dry also for w (mass_weight w, mut), spec_bdytend rw_tend, spec_bdyupdate w (solve_em 1603), spec_bdy_final w (solve_em 4566) |
| moist species | qv only (have_bcs_moist .false.), others flow_dep_bdy | all species relax_bdy_scalar + spec_bdy_scalar (solve_em 2257, 4622) |
| scalars | have_bcs_scalar | all (solve_em 2804) |
| adaptive dt | - | only when dtbc ~ 0 |
| microphysics spec-zone skip | yes | same |

## Initialisation (share/mediation_integrate.F med_nest_initial, input_from_file)
med_interp_domain(parent, nest) (all 'd' fields by interp_fcn; ht_int, mub_fine, phb_fine kept),
read wrfinput_d0N, blend_terrain(ht, mub, phb) when save_topo_from_real = 1, adjust_tempqv,
med_nest_feedback(parent, nest) right away (the parent changes at t = 0 under feedback),
start_domain(nest), start_domain(parent) again. Details: docs/NEST_NUMERICS.md.

## CPPWRF design
- A domain is a cases::Model (grid, base, state, tendencies, RK3, LateralBC,
  diffusion, physics). A `NestLink` holds parent*, child*, i/j_parent_start, ratio, time ratio,
  feedback, the intermediate coarse arrays and the interpolation kernels (src/nest).
- The time loop is `advance(domain)`: rk->step(domain); for each child: force(link);
  repeat ratio times advance(child); if feedback: feedback(link).
- LateralBC has (a) W as a boundary variable with the nested relax/spec/update/final path,
  (b) any number of scalar species with boundary data (all moist species in nests), (c) the
  nested fcx/gcx, (d) `set_interval_from_device` so force() writes slabs on the device
  without host round trips, and dtbc reset per parent step.
- Verification: WRF runs N1 (one-way) and N2 (two-way), real*8. CPPWRF on the CPU in FP64 is
  bit-identical to WRF on both domains (N2 for 1 h); see docs/NEST_NUMERICS.md section 8.

## Moving nests (prescribed moves)

WRF 4.8.0 dmpar (one MPI rank) built with "preset moves" (configure nesting 2: `-DMOVE_NESTS`) runs
`med_nest_move` (share/mediation_nest_move.F) after each nest's `med_nest_feedback` in the parent's step, unless domain
1 is at its stop time (frame/module_integrate.F:446). CPPWRF (src/cases/nest_move.cpp, src/nest/nest_move.hpp, called
from run_nested in the same place) ports, in WRF's order:

1. `time_for_move` / `time_for_move2`: the first move of `move_id` = the nest whose `move_interval` (minutes from the
   clock's start, the restart time in a resumed run) has come, used once (its interval becomes 999999999); the
   innermost-nest corral check against domain 1 (`corral_dist`): a move that would bring the nest within corral_dist
   parent cells of an edge is cancelled in that direction. `|move_cd| > 1`: WRF's "invalid move" error.
2. `shift_domain_em`: every state array of the nest whose MemoryOrder starts with X..Y moves by
   `px = isign(parent_grid_ratio, move_cd_x)` points (x, then y; `p(ips:ipf) = p(ips+px:ipf+px)` over the memory rows,
   `ipf = ide` for x-staggered arrays, else `ide-1`), the halo / extra column included; the imask arrays (1 outside
   the patch, 0 on it) move with them, so 1 marks the strip the move brought in. In CPPWRF these are every field
   allocated while the nest's FieldRegistry scope is active (the model's state, tendencies and physics arrays) and the
   history fields the model does not hold (WrfHistory's move arrays), shifted on host copies.
3. `grid%i_parent_start` / `j_parent_start` += the move, and `nl_set_i/j_parent_start`: the interpolation, the forcing
   and the feedback read the namelist record's values, and the wrfout files' `I_PARENT_START` / `J_PARENT_START` follow
   the move. A serial build (`-DSTUBMPI`) skips `nl_set_*` and step 5's `start_domain` (both under
   `DM_PARALLEL && !STUBMPI`) while its intermediate grid moves: it interpolates from the wrong parent window and is
   not the reference.
4. `med_interp_domain`: every Registry "d" field of the configuration in inc/nest_interpdown_interp.inc's order
   (data/wrf_schema/nest_interpdown-4.8.0.tsv, tools/wrf_schema/gen_nest_interpdown.py), each writing where the
   imask is 1 (interp_fcnm and interp_mask_water_field: everywhere): interp_fcn (SINT), interp_fcni / fcnm /
   fcnm_imask / fcnm_lu (nearest), interp_mask_field, interp_mask_soil, interp_mask_water_field, interp_fcn_blint_ll
   (longitudes). Other functions stop the run with an error.
5. `start_domain(nest)` with `nest%moved` (start_em, not a restart, the first trip): the base state from HGT over
   the whole domain, `t_1 = t_2` and `mu_1 = mu_2` where imask = 1, `ph_1 = ph_2`, then `al`, `p` (from the `_1`
   levels), `alt`, `p_hyd`, `p_hyd_w`, the terrain slopes (TOPOSLPX, TOPOSLPY, LAP_HGT, SLOPE, SLP_AZI), and
   `phy_init` on the new points (set_tiles with imask_nostag; moved: IVGTYP, landuse_init, the effective radii,
   bl_init's accumulations, Noah's LSMINIT, kf_eta_init; THC), ported with the column physics suite for
   mp_physics 0 / 6, cu_physics 0 / 1, bl_pbl_physics 0 / 1, sf_sfclay_physics 0 / 1, sf_surface_physics 0 / 2,
   ra 0 / 4, without urban, fractional_seaice, sst_update, tmn_update, usemonalb, gwd_opt, the terrain options and
   grid_fdda (fg_init's fddagdinit); other physics, or none, is an error before the first step (translator and model);
   `ctopo = ctopo2 = 1` (topo_wind = 0) over the whole nest; `set_physical_bc2d / 3d` (the nest's edge points copied
   into the halo) of u, v, w, ph, t (both levels), mu_1, mu_2, the base state, php, al, alt, tke, moist, scalar and
   the map factors, sina, cosa, e, f. Then
   `u_1 = u_2`, `v_1`, `t_1`, `w_1`, `ph_1`, `tke_1`, `mu_1` where imask = 1. WRF's T (th_phy_m_t0 of the last solve)
   moves with the nest and takes the parent's in the new points.

NEST_POS: the moving build sets it at each domain's input (ht + 500 where ht > 0) and rewrites the parent's after each
feedback (the nest's points `ht * 1.5 + 1000`, elsewhere `ht + 500` where ht is not 0), so wrfout_d01's NEST_POS
differs from the basic build's; CPPWRF writes the moving build's values when moves are configured. The host caches
of latitude / longitude (Model::xlat / xlong, the radiation driver's) follow the move. Not supported (errors): a
moving nest that is not a nest of domain 1, has nests or siblings; tiles or MPI ranks; the adaptive time step;
`parent_grid_ratio` > 3; a nest whose interpolation window leaves domain 1's points (i/j_parent_start < 5 or within
5 parent cells plus the nest of the east / north edge); a resumed run (`run.restart_file`); vortex following;
`time_to_move`.
