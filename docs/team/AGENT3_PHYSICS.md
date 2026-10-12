# Agent 3 task T3.5: WRF physics port with differential testing against the Fortran original

Long task. Work through it in order; each scheme is a milestone with its own report section.
You may run it in parallel with T3.4 (different files), but never time benchmarks while
compiling.

You are Agent 3 in CPPWRF (/home/ostriquetrum/CPPWRF). Read docs/team/PLAN.md first.
Reference source: WRF v4.6.1 in ~/wrf_ref/WRF (from T3.3; clone it if missing).

## Why this method
A physics port can look plausible and still be wrong. Every routine you port is therefore
checked against **the original Fortran routine, compiled and run on the same inputs**. That
differential test is the acceptance criterion, not your own judgement.

## Ownership (only these paths)
- src/phys/ (new): C++ physics, one directory per scheme (src/phys/mp_kessler/, ...),
  plus src/phys/CMakeLists.txt defining library `cppwrf_phys`.
- tests/phys/ (new): differential-test harnesses, Fortran drivers, input generators.
- docs/team/AGENT3_REPORT_T35.md.
Do NOT edit anything else: Agent 1 integrates your library into the model. Never use git write
commands; never use sudo; do not modify WRF Fortran sources (copy into tests/phys/fortran/
if you need to compile them standalone, and record each change, e.g. stubbed wrf_error).

## C++ conventions (match the model)
- C++20, single-source kernels like the dycore: include "core/backend.hpp", "core/field.hpp".
  Each scheme is a `column_for` kernel (one thread per (i,j) column, loop over k inside), a
  trivially copyable struct with `CPPWRF_HD void operator()(int i,int j) const`. No virtual,
  no std::vector, no exceptions, no allocation inside kernels. It must compile with nvcc
  (CUDA) and g++ (OpenMP). Scratch arrays per column: fixed-size local arrays with a
  compile-time kMaxLevels (e.g. 128); assert nz <= kMaxLevels on the host.
- Arguments mirror the WRF subroutine (same names, same units, same meaning) but take
  `View3<real>` / `View2<real>` (see src/core/field.hpp). Index order is our (i,j,k) with
  C++ k = Fortran k - 1. Physical constants come from WRF's module_model_constants.F values
  (put them in src/phys/phys_constants.hpp; do not change src/core/constants.hpp).
- Precision: `real` (float or double). The differential tests run the FP64 build.
- Port statement by statement, keeping WRF's order of operations (needed for 1e-12
  agreement). Comment each block with the Fortran line range it came from.

## Differential test harness (build it first, reuse it for every scheme)
tests/phys/<scheme>/:
1. A Fortran driver (gfortran) that calls the ORIGINAL WRF subroutine on columns read from a
   binary input file and writes all INOUT/OUT arrays to a binary output file.
2. A C++ driver that does the same with your port (FP64 build, CPU backend).
3. An input generator (python3/numpy): N >= 2000 columns that cover the regime, i.e.
   realistic soundings (tropical, mid-latitude, dry, saturated layers, supercooled, with
   and without hydrometeors, near-zero and exactly-zero mixing ratios, and very small dt and
   large dt), with a fixed random seed.
4. A comparison script: per output variable, max abs diff, max rel diff (with a floor),
   and the column of the worst case. PASS = max rel diff <= 1e-12 (FP64) for every variable,
   or a documented, justified exception (e.g. a different but equivalent pow/exp in libm
   giving 1e-14 differences that grow through iterations; show the evidence).
5. A ctest entry that runs 1-4 (or a fast subset) so the test can be re-run anytime.
Also run the C++ port under the CUDA build on the same inputs and report max rel diff GPU vs
CPU (FP64 expected ~1e-12, FP32 expected ~1e-5).

## Schemes, in this order
1. mp_physics = 1, Kessler (phys/module_mp_kessler.F). Needed for the moist bubble and the
   supercell cases.
2. mp_physics = 6, WSM6 (phys/module_mp_wsm6.F), including its sedimentation and the
   saturation-adjustment loops. Port the version actually called by module_microphysics_driver
   in v4.6.1 (check the driver; note any #ifdef paths).
3. sf_sfclay_physics = 1 (revised MM5 surface layer, module_sf_sfclayrev.F) and
   bl_pbl_physics = 1 (YSU, module_bl_ysu.F).
4. ra_sw/ra_lw = 4 (RRTMG; module_ra_rrtmg_sw.F, module_ra_rrtmg_lw.F). Large: lookup tables
   and k-distributions. Plan it before you start (data files, initialisation, memory on the
   GPU) and write the plan into the report before coding.
Stop after each scheme and write its report section; continue to the next one in the same
session only if the previous one PASSES.

## Report: docs/team/AGENT3_REPORT_T35.md
Per scheme: files, Fortran changes needed for standalone compilation, the input regimes, the
comparison table (variable | max abs | max rel | worst column), CPU vs CUDA differences,
ns/column on CPU-12 and on the GPU for 10^5 columns x 50 levels, and open issues.
