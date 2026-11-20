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

The P4 references are WRF real*4 runs made on the workstation, as run_P1/steps (both in fp32id_data.tar), and so
are the P6 references (fp32id2_data.tar) and the MYNN and C01 ones (fp32id3_data.tar, from the P6 real*4 inputs;
run_C01_cpu_sp holds CCN_ACTIVATE.BIN). The CPU FP32 build of the branch base (before the MYNN and C01 changes) differs
from both 3-minute references from the first step on (MYNN: U at 19896 of 469920 points; C01: QNWFA, QNIFA and
QNWFA2D already at t = 0). The P6 run directories hold no Thompson table files (qr_acr_qg_V4.dat,
qr_acr_qsV2.dat, freezeH2O.dat): CPPWRF computes the tables as WRF real*4 does, about three minutes at start. All 89
tables and coefficients of thompson_init were compared with WRF real*4's in a dump; WRF v4.8.0 does not compute
t1_qg_qc, t2_qg_sd and t2_qg_me there, scheme 8 does not read them.
The older WRF real*4 run_P4 of port_fp32r4_data.tar, the reference of the CUDA FP32 case `real_p4_fp32_1h`,
differs from a rerun of the same `wrf.exe` with the same inputs and namelist here from the first 10 minutes on
(U at 176903 of 469920 points), with or without glibc's FMA variants; it was made by another run of WRF real*4 and
is not used for the CPU FP32 cases.

The platform is x86-64 Linux with glibc's libm and libmvec (2.41 here, CI's runners have Ubuntu's). WRF real*4 is
built with gfortran 14.2 `-O2 -ftree-vectorize -funroll-loops` (SSE2, its configure default); CPPWRF with GCC and
the suite's flags (`-march=skylake`, `CPPWRF_NATIVE=OFF` in CI). The functions CPPWRF ports are sequences of
IEEE operations (expf, logf, sinf, cosf); glibc's powf starts from the SSE `rcpps` estimate, which differs between
Intel and AMD, and the port reads Intel's values from a table instead of executing `rcpps`, so CPPWRF reproduces
the references (made on Intel) on either vendor's runners.

## GPU: the identity build

`cmake -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF -DCPPWRF_CUDA_FP32_IDENTITY=ON` builds a CUDA FP32 binary whose
results are the CPU FP32 build's, bit for bit (and so WRF real*4's on the cases above). It differs from the fast CUDA
FP32 build (the default) in:

- arithmetic: `--fmad=false` in every kernel (no fused multiply-add contraction), `--ftz=false --prec-div=true
  --prec-sqrt=true` stated explicitly;
- math functions on the device: glibc's scalar float functions as Debian's glibc 2.41 computes them on CPUs with
  FMA (src/core/glibc_mathf.hpp: expf, logf, powf, sinf, cosf transcribed from libm's FMA variants with their
  fused multiply-adds as fma(); atanf, tanhf, log10f from CORE-MATH's correctly rounded sources), glibc's double
  functions and libmvec's 2-lane double functions as in the CUDA FP64 build (src/core/glibc_math.hpp), and
  libmvec's 4-lane float functions (src/core/libmvec_f4.hpp) with the CPU build's lane rule (iterations 0 ..
  4*(n/4)-1 of the loop vectorised, the rest scalar);
- sums: `Reducer::sum` (the `diag.csv` masses and energy) adds each level in the CPU build's (j, i) order, one
  thread per level.

The kernels, launches and their order are the fast build's. Proof:

- tools/glibc_mathf_proof.cpp: the ports against the system libm, every float input of expf, logf, sinf, cosf,
  atanf, tanhf and log10f and 2^32 pseudo-random powf pairs plus special values: 0 differ.
- tools/glibc_mathf_proof_cuda.cu: the device against the host, every float input of the same functions and of the
  4-lane expf, logf, sinf, cosf, atanf, and 2^32 pairs of the scalar and the 4-lane powf: 0 differ (RTX 2060, sm_75).
  tests/unit/test_glibc_mathf.cpp repeats both checks on a sample in every build (the device part in CUDA builds).
- The suite's backend `cuda_sp_id` (`tests/verify/wrf_suite.py --cuda-sp-id BIN`) runs every `cpu_sp` case on the
  GPU (names `..._cuda_id...`, e.g. `real_p4_fp32_cuda_id_1h`) against the same WRF real*4 references at tolerance 0.
  On an RTX 2060 (sm_75) all of them pass: P1 and P4 (3 minutes with history every step, and 1 h), P2, P3, P9, NMP1
  (1 h), P6, MYNN and C01 (3 minutes and 1 h).
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
- Values WRF real*4 stores in REAL are rounded where WRF rounds them (base state, lateral boundary coefficients,
  Noah init, history T, WSM6's mp_wsm6_init products, the MYNN surface layer's psi tables (entry 0 a constant, 1 ..
  1000 four lanes at a time; powf(x, 0.5) in the scalar fallback), MYNN-EDMF's vectorised plume powers, R_v/R_d of moist_physics_prep_em, Thompson's REAL constants,
  gamma coefficients and the REAL subexpressions of its table builders, ...): a constant WRF folds
  in REAL (r_v/r_d is 1 ulp below the double quotient rounded to float) or a product it evaluates in REAL from left
  to right; the FP64 build keeps its code path unchanged (`if constexpr` on the real type), and
  FP64 results are checked byte for byte against main.

## What is not identical yet

- The fast CUDA FP32 build (the default): FMA contraction and CUDA's math library; it stays measured in
  docs/FP32_ACCURACY.md. The identity build covers the cases above only; a float function other than expf, logf,
  powf, sinf, cosf, atanf, tanhf and log10f (asinf, acosf, atan2f, tanf, sinhf, erff, tgammaf, hypotf) is still
  CUDA's on its device, and the optional radar reflectivity's complex logarithm is CUDA's.
- Schemes and options other than those of the cases above are not checked against WRF real*4: aerosol-aware Thompson
  (mp_physics 28) with aerosol input fields or wif_input_opt 2, the MYNN options other than the defaults
  (MYNNX1 / MYNNX2, SPP), the other microphysics, cumulus, PBL, surface and radiation schemes, nests,
  restarts. Each needs the same work: its WRF real*4 reference, the dumps, its vectorised loops, its REAL constants.
- Other compilers or libm versions for WRF real*4 (Intel, AVX2 builds use the 8-lane functions): not covered.
