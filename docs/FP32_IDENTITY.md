# CPPWRF CPU FP32 identity with WRF real*4

CPPWRF's FP64 builds are bit-identical to WRF v4.8.0 real*8 (docs/STATUS.md). This page covers the CPU FP32 build
(`-DCPPWRF_DOUBLE=OFF`, no CUDA): on the configurations below it is bit-identical to stock WRF v4.8.0 real*4, every
history variable and every bit, so a CPU FP32 run can stand in for a WRF real*4 run; and the CUDA FP32 identity build
(`-DCPPWRF_CUDA_FP32_IDENTITY=ON`, section [GPU: the identity build](#gpu-the-identity-build)), which gives the same
bits on the GPU. docs/FP32_ACCURACY.md covers the fast CUDA FP32 build (the default), which is not identical to
anything and is measured against WRF real*8 instead.

## What is identical

| Case | Physics | Reference | Compared | Result |
|---|---|---|---|---|
| `real_p1_fp32_cpu` | P1: revised MM5 surface layer, Noah, YSU, no microphysics / cumulus / radiation | WRF real*4 run_P1/steps (3 minutes, history every step) | 4 frames x 175 variables | identical |
| `real_p1_fp32_cpu_1h` | P1 | WRF real*4 run_P1 (1 h) | 13 variables at 1 h | identical |
| `real_p9_fp32_cpu_1h` | P9: Eta surface layer, Noah, MYJ | WRF real*4 run_P9 (1 h) | 13 variables at 1 h | identical |
| `real_p2_fp32_cpu_1h` | P2: P1 + Kain-Fritsch | WRF real*4 run_P2 (1 h) | 13 variables at 1 h | identical |
| `real_p3_fp32_cpu_1h` | P3: P1 + RRTMG LW/SW (radt 1 min) | WRF real*4 run_P3 (1 h) | 13 variables at 1 h | identical |
| `real_p4_fp32_cpu` | P4: P1 + Kain-Fritsch + WSM6 + RRTMG LW/SW (radt 1 min, clouds, WSM6 effective radii), diff_opt 1 / km_opt 4 | WRF real*4 run_P4_cpu_sp/steps (3 minutes, history every step) | 4 frames x 197 variables | identical |
| `real_p4_fp32_cpu_1h` | P4 | WRF real*4 run_P4_cpu_sp (1 h, history every 10 minutes) | 7 frames x 219 variables | identical |
| `real_nmp1_fp32_cpu_1h` | NMP1: Noah-MP | WRF real*4 run_NMP1 (1 h) | 13 variables at 1 h | identical |
| `real_p6_fp32_cpu` | P6: P1 + Thompson (mp_physics 8) | WRF real*4 run_P6_cpu_sp/steps (3 minutes, history every step) | 4 frames x 167 variables | identical |
| `real_p6_fp32_cpu_1h` | P6 | WRF real*4 run_P6_cpu_sp (1 h, history at 0 and 1 h) | 2 frames x 167 variables | identical |
| `real_mynn_fp32_cpu` | MYNN: P6 with the MYNN surface layer and the MYNN-EDMF PBL (icloud_bl 1) + RRTMG LW/SW (radt 10 min) | WRF real*4 run_MYNN_cpu_sp/steps (3 minutes, history every step) | 4 frames x 217 variables | identical |
| `real_mynn_fp32_cpu_1h` | MYNN | WRF real*4 run_MYNN_cpu_sp (1 h, history every 10 minutes) | 7 frames x 217 variables | identical |
| `real_c01_fp32_cpu` | C01: P6 with the aerosol-aware Thompson (mp_physics 28, aer_init_opt 0: thompson_init's QNWFA / QNIFA profiles and QNWFA2D) | WRF real*4 run_C01_cpu_sp/steps (3 minutes, history every step) | 4 frames x 174 variables | identical |
| `real_c01_fp32_cpu_1h` | C01 | WRF real*4 run_C01_cpu_sp (1 h, history every 10 minutes) | 7 frames x 174 variables | identical |
| `real_p4d6_fp32_cpu` | D6: P4 with WDM6 (mp_physics 16; CCN from ccn_conc) | WRF real*4 run_D6_cpu_sp/steps (3 minutes, history every step) | 4 frames x 200 variables | identical |
| `real_p4d6_fp32_cpu_1h` | D6 | WRF real*4 run_D6_cpu_sp (1 h, history every 10 minutes) | 7 frames x 222 variables | identical |
| `real_p4d5_fp32_cpu` | D5: P4 with WDM5 (mp_physics 14) | WRF real*4 run_D5_cpu_sp/steps | 4 frames x 199 variables | identical |
| `real_p4d5_fp32_cpu_1h` | D5 | WRF real*4 run_D5_cpu_sp (1 h) | 7 frames x 221 variables | identical |
| `real_morr_fp32_cpu` | MORR: P4 with Morrison (mp_physics 10, hail) | WRF real*4 run_MORR_cpu_sp/steps | 4 frames x 201 variables | identical |
| `real_morr_fp32_cpu_1h` | MORR | WRF real*4 run_MORR_cpu_sp (1 h) | 7 frames x 223 variables | identical |
| `real_morg_fp32_cpu` | MORG: P4 with Morrison (graupel, morr_rimed_ice 0), no cumulus | WRF real*4 run_MORG_cpu_sp/steps | 4 frames x 201 variables | identical |
| `real_morg_fp32_cpu_1h` | MORG | WRF real*4 run_MORG_cpu_sp (1 h) | 7 frames x 219 variables | identical |
| `real_g2_fp32_cpu` | G2: P2 with the gravity wave drag (gwd_opt 1) | WRF real*4 run_G2_cpu_sp/steps | 4 frames x 174 variables | identical |
| `real_g2_fp32_cpu_1h` | G2 | WRF real*4 run_G2_cpu_sp (1 h) | 7 frames x 196 variables | identical |
| `real_n1_fp32_cpu` | N1: one-way nest (d02 5 km, ratio 3), no physics | WRF real*4 run_N1_cpu_sp/steps (3 minutes, history every d01 and d02 step) | 14 frames x 318 variables (both domains) | identical |
| `real_n1_fp32_cpu_1h` | N1 | WRF real*4 run_N1_cpu_sp (1 h, history every 10 minutes) | 14 frames x 318 variables | identical |
| `real_n3_fp32_cpu` | N3: two-way nest (feedback, smooth_option 2) with the P4 physics on both domains | WRF real*4 run_N3_cpu_sp/steps (3 minutes, history every minute) | 8 frames x 394 variables | identical |
| `real_n3_fp32_cpu_1h` | N3 | WRF real*4 run_N3_cpu_sp (1 h) | 14 frames x 394 variables | identical |
| `real_mon3_fp32_cpu` | MON3: N3 with monotonic moist and scalar advection (moist_adv_opt / scalar_adv_opt 2) | WRF real*4 run_MON3_cpu_sp/steps | 8 frames x 394 variables | identical |
| `real_mon3_fp32_cpu_1h` | MON3 | WRF real*4 run_MON3_cpu_sp (1 h) | 14 frames x 394 variables | identical |
| `real_kesskf_fp32_cpu` | KESSKF: P4 with Kessler (mp_physics 1) | WRF real*4 run_KESSKF_cpu_sp/steps | 4 frames x 194 variables | identical |
| `real_kesskf_fp32_cpu_1h` | KESSKF | WRF real*4 run_KESSKF_cpu_sp (1 h) | 7 frames x 216 variables | identical |
| `real_acm7_fp32_cpu` | ACM7: P4 with the ACM2 PBL over the Pleim-Xiu surface layer | WRF real*4 run_ACM7_cpu_sp/steps | 4 frames x 197 variables | identical |
| `real_acm7_fp32_cpu_1h` | ACM7 | WRF real*4 run_ACM7_cpu_sp (1 h) | 7 frames x 219 variables | identical |
| `real_q4_fp32_cpu` | Q4: P4 with the QNSE PBL and surface layer | WRF real*4 run_Q4_cpu_sp/steps | 4 frames x 201 variables | identical |
| `real_q4_fp32_cpu_1h` | Q4 | WRF real*4 run_Q4_cpu_sp (1 h) | 7 frames x 223 variables | identical |
| `real_t6_fp32_cpu` | T6: P2 with Tiedtke (cu_physics 6) | WRF real*4 run_T6_cpu_sp/steps | 4 frames x 160 variables | identical |
| `real_t6_fp32_cpu_1h` | T6 | WRF real*4 run_T6_cpu_sp (1 h) | 7 frames x 181 variables | identical |
| `real_p7_fp32_cpu` | P7: P2 with New Tiedtke (cu_physics 16) | WRF real*4 run_P7_cpu_sp/steps | 4 frames x 160 variables | identical |
| `real_p7_fp32_cpu_1h` | P7 | WRF real*4 run_P7_cpu_sp (1 h) | 7 frames x 181 variables | identical |
| `real_dud4_fp32_cpu` | DUD4: P4 with the Dudhia shortwave (ra_sw_physics 1) | WRF real*4 run_DUD_cpu_sp/steps | 4 frames x 181 variables | identical |
| `real_dud4_fp32_cpu_1h` | DUD4 | WRF real*4 run_DUD_cpu_sp (1 h) | 7 frames x 203 variables | identical |
| `real_slab1_fp32_cpu` | SLAB1: P1 with the 5-layer thermal diffusion land surface (sf_surface_physics 1) | WRF real*4 run_SLAB1_cpu_sp/steps | 4 frames x 159 variables | identical |
| `real_slab1_fp32_cpu_1h` | SLAB1 | WRF real*4 run_SLAB1_cpu_sp (1 h) | 7 frames x 174 variables | identical |

The P4 references are WRF real*4 runs made on the workstation, as run_P1/steps (both in fp32id_data.tar), and so
are the P6 references (fp32id2_data.tar) and the MYNN and C01 ones (fp32id3_data.tar, from the P6 real*4 inputs;
run_C01_cpu_sp holds CCN_ACTIVATE.BIN). The CPU FP32 build of the branch base (before the MYNN and C01 changes)
differs from both 3-minute references from the first step on (MYNN: U at 19896 of 469920 points; C01: QNWFA, QNIFA
and QNWFA2D already at t = 0). The P6 run directories hold no Thompson table files (qr_acr_qg_V4.dat,
qr_acr_qsV2.dat, freezeH2O.dat): CPPWRF computes the tables as WRF real*4 does, about three minutes at start. All 89
tables and coefficients of thompson_init were compared with WRF real*4's in a dump; WRF v4.8.0 does not compute
t1_qg_qc, t2_qg_sd and t2_qg_me there, scheme 8 does not read them.
The references of D6 ... SLAB1 (fp32id4_data.tar) are WRF real*4 runs made on the workstation from run_P4_cpu_sp's
real*4 inputs (D6, D5, MORR, MORG, KESSKF, ACM7, Q4, DUD4), from run_P2's (T6, P7), and from real.exe real*4 runs on
the met_em files of run_P1 (SLAB1: the 5-layer soil input), of run_P2 (G2: gwd_opt 1 writes CON, OA1-4, OL1-4;
ELVMAX = HGT + 3 VAR set in REAL, as in the FP64 G2 input) and of run_N1 (N3, MON3: both domains with the P4
physics); N1 uses run_N1's real*4 inputs. Each case checks its configuration against the reference's namelist. The
CPU FP32 build of the branch base (ce0718d) differs from WRF real*4 on D6 / D5 (QCLOUD at 63 points after the first
step), MORR / MORG (QICE at 52), N1 (the d01 base state PHB / MUB at t = 0) and N3; KESSKF, ACM7, Q4, T6, P7, G2,
SLAB1 and DUD4 needed no change.
The older WRF real*4 run_P4 of port_fp32r4_data.tar, the reference of the CUDA FP32 case `real_p4_fp32_1h`,
differs from a rerun of the same `wrf.exe` with the same inputs and namelist here from the first 10 minutes on
(U at 176903 of 469920 points), with or without glibc's FMA variants; it was made by another run of WRF real*4 and
is not used for the CPU FP32 cases.

The platform is x86-64 Linux. The references are WRF real*4 runs on Debian 13, against its glibc 2.41's libm and
libmvec; WRF real*4 is built with gfortran 14.2 `-O2 -ftree-vectorize -funroll-loops` (SSE2, its configure default),
CPPWRF with GCC and the suite's flags (`-march=skylake`, `CPPWRF_NATIVE=OFF` in CI). The functions CPPWRF ports are
sequences of IEEE operations (expf, logf, sinf, cosf); glibc's powf starts from the SSE `rcpps` estimate, which
differs between Intel and AMD, and the port reads Intel's values from a table instead of executing `rcpps`, so CPPWRF
reproduces the references (made on Intel) on either vendor's CPUs.

### Independent of the host's libm

glibc's float functions differ between versions: 2.40 and 2.41 replaced several with CORE-MATH's correctly rounded
ones, and a CPU FP32 build on Ubuntu 24.04 (glibc 2.39) calling its own libm differed from WRF real*4 from t = 0 (P1:
W at 204905 of 476595 points, 1 ulp). The FP32 builds therefore do not call libm's float functions: the executables
`cppwrf` and `cppwrf_real` of an FP32 build (CPU, and the host code of the CUDA builds) define them
(src/core/glibc_mathf_host.cpp), so every call in the program, `std::exp(float)`, `libm::exp(float)`, the scalar
iterations of the vectorised loops, the `sincosf` g++ makes of a sinf and a cosf of one argument, table and
initialisation code, computes glibc 2.41's result (src/core/glibc_mathf.hpp):

| Function | glibc 2.41's implementation | Proof against glibc 2.41 (every float input) |
|---|---|---|
| expf, logf, powf, sinf, cosf | the FMA variants libm's ifuncs select (`__expf_fma`, ...), from libm's machine code | 0 differ (powf: 2^32 pairs + special values) |
| sincosf | sinf's and cosf's results | 0 differ |
| atanf, atan2f, tanf, tanhf, log10f, asinf, acosf, sinhf, erff, tgammaf | CORE-MATH (generic C, no FMA variants) | 0 differ (atan2f: 2^32 pairs + special values) |

tools/glibc_mathf_proof.cpp makes the comparison (also built without `-march`, where `fma()` is libm's exact fma: the
same results); the libm compared is Debian's 2.41-12, whose sinhf takes the CORE-MATH exceptional case
x = 0x1.250bfep-11 that the 2.41 tarball's code misses (the port follows the libm). The ctest `fp32_host_libm` checks
that `cppwrf` imports none of these functions (`nm -D --undefined-only`). Calls with constant arguments are folded by
g++ (correctly rounded) as before. Still libm's: sqrtf, fmodf, roundf, lroundf, llroundf, fminf, fmaxf, strtof
(exact or correctly rounded by definition, the same in every version), and the double functions (exp, log, pow, sin,
cos, atan, tanh, log10, tgamma, the complex cabs / clog / csqrt of the radar reflectivity, libmvec's 2-lane exp and
pow), which the FP64 builds call as well. The FP64 builds are unchanged.

## GPU: the identity build

`cmake -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF -DCPPWRF_CUDA_FP32_IDENTITY=ON` builds a CUDA FP32 binary whose
results are the CPU FP32 build's, bit for bit (and so WRF real*4's on the cases above). It differs from the fast CUDA
FP32 build (the default) in:

- arithmetic: `--fmad=false` in every kernel (no fused multiply-add contraction), `--ftz=false --prec-div=true
  --prec-sqrt=true` stated explicitly;
- math functions on the device: glibc's scalar float functions as Debian's glibc 2.41 computes them on CPUs with
  FMA (src/core/glibc_mathf.hpp: expf, logf, powf, sinf, cosf transcribed from libm's FMA variants with their
  fused multiply-adds as fma(); atanf, atan2f, tanf, tanhf, log10f, asinf, acosf, sinhf, erff, tgammaf from
  CORE-MATH's correctly rounded sources; the same functions as the host's, section
  [Independent of the host's libm](#independent-of-the-hosts-libm)),
  glibc's double functions and libmvec's 2-lane double functions as in the CUDA FP64 build
  (src/core/glibc_math.hpp), and libmvec's 4-lane float functions (src/core/libmvec_f4.hpp) with the CPU build's
  lane rule (iterations 0 .. 4*(n/4)-1 of the loop vectorised, the rest scalar);
- sums: `Reducer::sum` (the `diag.csv` masses and energy) adds each level in the CPU build's (j, i) order, one
  thread per level.

The kernels, launches and their order are the fast build's. Proof:

- tools/glibc_mathf_proof.cpp: the ports against the system libm, every float input of expf, logf, sinf, cosf,
  atanf, tanf, tanhf, log10f, asinf, acosf, sinhf, erff, tgammaf and sincosf and 2^32 pseudo-random powf and atan2f
  pairs plus special values: 0 differ.
- tools/glibc_mathf_proof_cuda.cu: the device against the host, every float input of the same functions and of the
  4-lane expf, logf, sinf, cosf, atanf, and 2^32 pairs of the scalar and the 4-lane powf: 0 differ (RTX 2060, sm_75);
  tanf (every input) and atan2f (2^32 pairs) likewise (`proof_cuda 32 tanf,atan2f`), and asinf, acosf, sinhf, erff,
  tgammaf (every input, `proof_cuda 32 asinf,acosf,erff,tgammaf,sinhf`).
  tests/unit/test_glibc_mathf.cpp repeats both checks on a sample in every build (the device part in CUDA builds).
- The suite's backend `cuda_sp_id` (`tests/verify/wrf_suite.py --cuda-sp-id BIN`) runs every `cpu_sp` case on the
  GPU (names `..._cuda_id...`, e.g. `real_p4_fp32_cuda_id_1h`) against the same WRF real*4 references at tolerance 0.
  On an RTX 2060 (sm_75) all of them pass: P1 and P4 (3 minutes with history every step, and 1 h), P2, P3, P9, NMP1
  (1 h), P6, MYNN and C01 (3 minutes and 1 h), and D6, D5, MORR, MORG, G2, N1, N3, MON3, KESSKF, ACM7, Q4, T6, P7,
  SLAB1, DUD4 (3 minutes and 1 h). T6 (Tiedtke's tan) and G2 (GWDO's atan2) differed after 30 / 60 minutes while the
  device took CUDA's tanf / atan2f; the CORE-MATH ports (glibc 2.41's are the same correctly rounded functions) fixed
  them.
- Without a WRF real*4 reference, the identity build against the CPU FP32 build (P4's inputs, 3 minutes, every
  model state variable of every step and the `diag.csv` sums): identical for P5 (WSM6), MYNN (Thompson, MYNN surface
  layer and PBL), Morrison, WSM5, Kessler + Kain-Fritsch, ACM2 with the Pleim-Xiu surface layer, multi-scale
  Kain-Fritsch, Dudhia shortwave, BouLac and MYJ / Eta surface layer configurations (configs/real_p5_r8.ini,
  real_mynn_r8, real_morr_r8, real_p4w5_r8, real_kesskf_r8, real_acm7_r8, real_m4_r8, real_dud_r8, real_bl4_r8,
  real_f2_r8).

Cost (RTX 2060 laptop GPU, sm_75, P4 10 steps, `CPPWRF_KTIME=1`; one machine, indicative only): kernel time 0.98 s
fast, 1.00 s identity (+3 %); the stepping time without KTIME 0.98-1.06 s against 1.01 s, within the run-to-run
spread. By kernel group (fast -> identity, ms over the 10 steps): RRTMG LW 626 -> 634 (its batch kernels have
`--fmad=false` in the fast build already), dynamics 178 -> 185, WSM6 45 -> 50, Kain-Fritsch 23 -> 27, YSU 26.6 ->
27.1, Noah and the surface layer 2.8 -> 4.5. GPUs with a larger share of time in FMA-heavy kernels, or with slow
FP64 (the glibc ports compute in double), will pay more; the fast build stays the default.

## How it is proven

- The suite's backend `cpu_sp` (`tests/verify/wrf_suite.py --cpu-sp BIN`) runs these cases at tolerance 0 against
  the WRF real*4 history files; each case also checks the configuration against WRF's namelist. CI runs them when
  cppwrf-data builds the FP32 binary as well (a second CMake build with `-DCPPWRF_DOUBLE=OFF`).
- Each change was found by checkpoint dumps: a WRF real*4 `wrf.exe` linked from copies of solve_em.F and
  module_first_rk_step_part1.F with dumps at one time step (the WRF tree is untouched), against CPPWRF's
  `CPPWRF_VM_DUMP` / `CPPWRF_VM_DUMP_STEP` dumps of the same arrays, compared bit for bit up to the first differing
  checkpoint.
- WRF's vectorised loops call glibc's 4-lane SSE4 float functions (`_ZGVbN4v_expf`, `_ZGVbN4v_logf`,
  `_ZGVbN4v_sinf`, `_ZGVbN4v_cosf`, `_ZGVbN4v_atanf`, `_ZGVbN4vv_powf`) for iterations 0 .. 4*(n/4)-1 and scalar libm for the rest
  (gfortran's `-fopt-info-vec` lists the loops). src/core/libmvec_f4.hpp ports them lane by lane from glibc's
  sources, with tables generated by tools/libmvec_f4_tables.py; tests/unit/test_libmvec_f4.cpp checks them against
  the system libmvec (`test_libmvec_f4 all`: expf, logf, sinf, cosf and atanf over all 2^32 inputs, 0 differ).
  `vm::exp / log / pow / sin / cos / atan (float, i, n)` choose the
  vector or the scalar function by the loop index as WRF's loop does.
- Init-time constants are compared with WRF real*4's module variables directly: a small program calls WRF real*4's
  wdm6init / wdm5init (63 / 25 variables, hail_opt 0 and 1) and MORR_TWO_MOMENT_INIT (87 variables, morr_rimed_ice
  0 and 1) from the real*4 build's libwrflib.a and prints them next to CPPWRF's Wdm6Params<float> /
  MorrisonParams<float>: 0 differ. gfortran folds a constant expression in the build's REAL, each operation rounded
  and powers correctly rounded: r0**3., (dimax/dicon)**2, dimax**8, (24.)**(.3333333), (actr*1.E-6)**3, (10.E-6)**3
  ... are the binary32 results, not the real*8 values rounded to float.
- Nests: start_em's base state for max_dom > 1 (pb, t_init, alb from MUB; phb of the first pass), blend_terrain,
  adjust_tempqv, the press_adj of the nest's mu, copy_fcn's feedback average (cfld += 1./REAL(n) * nfld), sm121 and
  smdsm in REAL; the nest's lateral tendencies keep WRF's REAL*8 rdt (already so).
- Values WRF real*4 stores in REAL are rounded where WRF rounds them (base state, lateral boundary coefficients,
  Noah init, history T, WSM6's mp_wsm6_init products, the MYNN surface layer's psi tables (entry 0 a constant, 1 ..
  1000 four lanes at a time; powf(x, 0.5) in the scalar fallback), MYNN-EDMF's vectorised plume powers, R_v/R_d of moist_physics_prep_em, Thompson's REAL constants,
  gamma coefficients and the REAL subexpressions of its table builders, ...): a constant WRF folds
  in REAL (r_v/r_d is 1 ulp below the double quotient rounded to float) or a product it evaluates in REAL from left
  to right; the FP64 build keeps its code path unchanged (`if constexpr` on the real type), and
  FP64 results are checked byte for byte against main.

## What is not identical yet

- The fast CUDA FP32 build (the default): FMA contraction and CUDA's math library; it stays measured in
  docs/FP32_ACCURACY.md. The identity build covers the cases above only; hypotf (no FP32 code calls it) and the
  optional radar reflectivity's complex logarithm are CUDA's on its device.
- Schemes and options other than those of the cases above are not checked against WRF real*4: aerosol-aware Thompson
  (mp_physics 28) with aerosol input fields or wif_input_opt 2, the MYNN options other than the defaults
  (MYNNX1 / MYNNX2, SPP), WDM6 with hail_opt 1, the other microphysics, cumulus, PBL, surface and radiation schemes,
  nests other than N1 / N3 / MON3 (moving nests, more than two domains, vertical refinement), restarts. Each needs
  the same work: its WRF real*4 reference, the dumps, its vectorised loops, its REAL constants.
- Other compilers or libm versions for WRF real*4 (Intel, AVX2 builds use the 8-lane functions): not covered; the
  references are WRF real*4 built against glibc 2.41, and CPPWRF computes glibc 2.41's float functions on any host.
