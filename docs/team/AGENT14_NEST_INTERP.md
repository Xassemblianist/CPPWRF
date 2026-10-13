# Agent 14 task T14.1: WRF nesting interpolation and feedback kernels

Agent 0 is implementing two-way nesting (docs/NESTING.md, being written). You port the
interpolation/feedback numerics as device kernels, differential-tested against the ORIGINAL
WRF 4.6.1 Fortran (~/wrf_real/WRF-4.6.1-r8, compiled unmodified, -fdefault-real-8), CPU and
CUDA FP64 bit-exact or <= 1e-12 rel, on realistic coarse/fine pairs (take coarse fields from
~/wrf_real/run_R0_r8 wrfout: U, V, W, PH, T, MU, QVAPOR, HGT, MUB, PHB; ratio 3; odd and even
i_parent_start; staggered and unstaggered fields; nest touching close to the parent edge).
Port (share/interp_fcn.F, share/sint.F, frame/module_..., dyn_em/nest_init_utils.F,
share/mediation_*.F as needed):
1. interp_fcn with the default SINT method (and sint.F: sint, sintb), for ratio 3 (also 5),
   x/y staggered and unstaggered, the masks (imask) as WRF uses them for EM.
2. bdy_interp / bdy_interp1 (nest LBC values and tendencies: value = current nest field,
   tendency = (sint(parent new) - nest)/cdt, the xs/xe/ys/ye slab layout and sz = max(spec_zone,
   relax_zone+1) capped by spec_bdy_width), in our LateralBC slab layout (pos, k, b), pos fastest,
   see src/dyn/lateral_bc.hpp (do not edit it: write a converter or produce that layout).
3. copy_fcn (feedback averaging, odd ratios: mass, x-stag, y-stag), and the feedback smoothing
   options smooth_option 1 and 2 (smoother_desmoother / 1-2-1, nest_feedbackup_smooth.inc).
4. couple_or_uncouple_em (which fields, which mu, c1/c2 coefficients) as used around force and
   feedback, and the extra steps in med_feedback_domain / feedback_domain_em_part2 and
   med_nest_force (e.g. what is recomputed after feedback).
5. Nest initialisation pieces for input_from_file: blend_terrain, adjust_tempqv, copy_3d_field,
   and what med_nest_initial does in order (document it precisely).
Ownership: src/nest/ and tests/nest/ (new). Header-only or .cpp kernels in the house style
(src/dyn/lateral_bc.cpp, src/phys/bl_ysu/ysu.hpp). Also write docs/NEST_NUMERICS.md: for every
routine, the WRF file:line, inputs/outputs, index conventions (C++ k = Fortran k - 1, nest
(1,1) = parent (i_parent_start, j_parent_start)), and where in the time step WRF calls it.
Rules: tools/heavy.sh (--light for quick checks), -j4, nothing in the repo root, no git writes,
never git stash. Report: docs/team/AGENT14_REPORT.md.
