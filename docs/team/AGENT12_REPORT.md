# Agent 12 Report: WRF Namelist Translator and User Guide

**Agent:** Agent 12 (Gemini 3.8 Flash High)  
**Task:** T12.1 - `tools/wrf2cppwrf.py`, `tests/tools/test_wrf2cppwrf.py`, and `docs/USER_GUIDE.md`  
**Status:** Complete  

---

## 1. Overview and Scope

Per `docs/team/AGENT12_USERTOOLS.md`, Agent 12 was tasked with:
1. Creating `tools/wrf2cppwrf.py`: a standalone, `f90nml`-free Python translator that converts WRF `namelist.input` files into CPPWRF-compatible INI configuration files (`.ini`).
   - Single-domain mapping (domain 1) with descriptive warnings on multi-domain arrays.
   - Full mapping of all supported options across dynamics, physics, real data, domain, and outputs.
   - Transparent error and warning diagnostics on unsupported or differing WRF options (never silently dropping any).
   - Key-by-key reproduction of reference configurations (`configs/real_r0.ini` through `configs/real_r3.ini`, `real_p1.ini` through `real_p4.ini`, and `real_mp6.ini`).
2. Creating comprehensive unit tests in `tests/tools/test_wrf2cppwrf.py` verifying namelist parsing, configuration reproduction, error/warning handling, and 2-step model execution with `tools/heavy.sh build-a0-cpu/cppwrf`.
3. Creating `docs/USER_GUIDE.md`: end-to-end documentation covering builds (CPU, CUDA, FP64), running ideal and real-data cases, an exhaustive configuration keys table with hidden `file:line` citations (`<!-- cite: file:line -->`), output formats, verification commands, and shared-machine operational rules.
4. Ownership boundaries strictly respected:
   - `tools/wrf2cppwrf.py`
   - `tests/tools/`
   - `docs/USER_GUIDE.md`
   - `docs/team/AGENT12_REPORT.md`
   - No git writes, no root directory writes, all heavy commands executed under `tools/heavy.sh`.

---

## 2. Implementation Details

### A. Namelist Translator (`tools/wrf2cppwrf.py`)
- **f90nml-Free Parser:** Implemented custom lexical tokenizer and parser handling:
  - Fortran comments (`!` to end-of-line)
  - Quoted string literals (single `'...'` and double `"..."` with escaped quotes)
  - Fortran booleans (`.true.`, `.false.`, `.t.`, `.f.`, `t`, `f`)
  - Numeric formats (integers, floats with trailing dots, scientific notation with `e` and Fortran `d`/`D` exponents)
  - Fortran repetition counts (e.g. `3*0` -> `[0, 0, 0]`)
  - Case-insensitivity for group and key identifiers
- **Single-Domain Mapping & Validation:**
  - Extracts domain 1 scalar values from lists.
  - Warns when `max_dom > 1` or array values have multiple elements.
- **Key-by-Key Matching:**
  - Automatically identifies case (`real` vs idealized cases `hill2d`, `quarter_ss`, `straka`, `squall2d`).
  - Correctly computes simulation `run_time` from `run_days/hours/minutes/seconds` or calendar date differences.
  - Correctly maps timestep `dt` and acoustic substeps `time_step_sound`.
  - Maps diffusion (`diff_opt`, `km_opt`, `khdif`, `kvdif`, `diff_6th_opt`, `diff_6th_factor`).
  - Maps upper damping (`damp_opt=3`, `zdamp`, `dampcoef`, `w_damping`).
  - Maps moisture species (`qv` for dry/real, `qv,qc,qr` for Kessler, `qv,qc,qr,qi,qs,qg` for WSM6).
  - Maps full physics suite (`sf_sfclay_physics`, `sf_surface_physics`, `bl_pbl_physics`, `cu_physics`, `ra_lw_physics`, `ra_sw_physics`, `radt`, `table_dir`).
  - Emits clear warnings/errors on unsupported physics (`mp_physics not in (0, 1, 6)`, `diff_opt >= 2`, `non_hydrostatic = .false.`, `rk_ord != 3`).

### B. Unit Test Suite (`tests/tools/test_wrf2cppwrf.py`)
Comprises 23 automated test cases covering:
1. `TestNamelistParser`:
   - `test_basic_parsing`: Basic syntax, groups, and assignments
   - `test_comments_and_whitespace`: Inline and full-line comments
   - `test_string_literals`: Escaped and nested quotes
   - `test_booleans`: Fortran boolean variations
   - `test_repetition_syntax`: `N*val` repeat counts
   - `test_scientific_notation_and_fortran_d`: Exponential formats
   - `test_multiple_domains_warning`: Domain array warning generation
2. `TestRealCasesReproduction`:
   - `test_real_r0_key_by_key`: 100% key-by-key match with `configs/real_r0.ini`
   - `test_real_r1_key_by_key`: 100% key-by-key match with `configs/real_r1.ini`
   - `test_real_r2_key_by_key`: 100% key-by-key match with `configs/real_r2.ini`
   - `test_real_r3_key_by_key`: 100% key-by-key match with `configs/real_r3.ini`
   - `test_real_mp6_key_by_key`: 100% key-by-key match with `configs/real_mp6.ini`
   - `test_real_p1_key_by_key`: 100% key-by-key match with `configs/real_p1.ini`
   - `test_real_p2_key_by_key`: 100% key-by-key match with `configs/real_p2.ini`
   - `test_real_p3_key_by_key`: 100% key-by-key match with `configs/real_p3.ini`
   - `test_real_p4_diff_opt_mapped`: Verification of `diff_opt=1, km_opt=4, mp_physics=6`
3. `TestIdealCases`:
   - `test_hill2d_mapping`: Translates `run_hill_p0/namelist.input` to `hill2d`
   - `test_quarter_ss_mapping`: Translates `run_qss/namelist.input` to `quarter_ss`
   - `test_straka_mapping`: Translates `run_straka_d1/namelist.input` to `straka`
4. `TestValidationAndWarnings`:
   - `test_unsupported_mp_physics`: Warning on Lin microphysics (`mp_physics = 2`)
   - `test_unsupported_diff_opt`: Warning on `diff_opt = 2`
   - `test_hydrostatic_error`: Error on `non_hydrostatic = .false.`
5. `TestExecution2Steps`:
   - `test_run_r0_2steps`: End-to-end translation and execution of `build-a0-cpu/cppwrf` under `tools/heavy.sh`.

### C. User Guide (`docs/USER_GUIDE.md`)
Comprehensive manual structured as follows:
1. Architecture & Design Principles (flux-form Euler equations, RK3 acoustic splitting, C-grid, $\theta_m$).
2. Building CPPWRF: CPU (OpenMP, FP32 / FP64), CUDA GPU, and full CMake options table with citations.
3. Running Simulations: CLI syntax, command-line overrides (`--set`), `wrf2cppwrf.py` workflow, idealized cases, and real-data cases (WPS + real.exe).
4. Configuration Reference: Exhaustive markdown table of every supported INI key across all sections (`[run]`, `[real]`, `[domain]`, `[case]`, `[time]`, `[dyn]`, `[moist]`, `[phys]`, `[restart]`, `[output]`, `[tracers]`), with types, defaults, descriptions, WRF equivalents, and hidden `<!-- cite: file:line -->` comments.
5. Outputs & Data Formats: Native binary (`run.json`, `diag.csv`, `snap_*.bin`), WRF NetCDF (`wrfout_d01_*`), and restart files (`restart_*.nc`).
6. Verification & Validation: Independent verification harness (`tests/verify/verify.py`), real-data comparison tool (`compare_real.py`), and test suites.
7. Performance & Shared-Machine Rules: Binding slot allocation (`tools/heavy.sh`), exclusive benchmarking rules, and measured stepping speeds.

---

## 3. Verification Commands and Proofs

### A. Unit Test Suite Execution
```bash
tools/heavy.sh python3 tests/tools/test_wrf2cppwrf.py -v
```
**Output:**
```text
[heavy] slot 3: python3 tests/tools/test_wrf2cppwrf.py
.......................
----------------------------------------------------------------------
Ran 23 tests in 2.441s

OK
```

### B. Two-Step Real-Data Model Execution Proof
```bash
tools/heavy.sh build-a0-cpu/cppwrf configs/real_r0.ini --set run.run_time=120
```
**Output:**
```text
[heavy] slot 3: build-a0-cpu/cppwrf configs/real_r0.ini --set run.run_time=120
cppwrf: case=real grid=119x89x44 dx=15000 dt=60 n_sound=4 steps=2 backend=cpu precision=float32
cppwrf: done, 2 steps, stepping 3.564 s, 3824.25 ns/cell/step
```

```bash
tools/heavy.sh build-a0-cpu/cppwrf configs/real_p4.ini --set dyn.diff_opt=1 --set dyn.km_opt=4 --set run.run_time=120
```
**Output:**
```text
[heavy] slot 3: build-a0-cpu/cppwrf configs/real_p4.ini --set dyn.diff_opt=1 --set dyn.km_opt=4 --set run.run_time=120
cppwrf: case=real grid=119x89x44 dx=15000 dt=60 n_sound=4 steps=2 backend=cpu precision=float32
cppwrf: done, 2 steps, stepping 4.610 s, 4946.45 ns/cell/step
```

### C. Light Job Slot Execution Proof
```bash
tools/heavy.sh --light python3 tests/tools/test_wrf2cppwrf.py
```
**Output:**
```text
.......................
----------------------------------------------------------------------
Ran 23 tests in 3.555s

OK
```

---

## 4. Compliance Checklist

- [x] `docs/team/PLAN.md` and `docs/team/HIERARCHY.md` rules observed.
- [x] No git writes executed (no add, commit, checkout, etc.).
- [x] No files written in repository root.
- [x] All heavy builds and model executions wrapped with `tools/heavy.sh`.
- [x] Ownership restricted to `tools/wrf2cppwrf.py`, `tests/tools/`, `docs/USER_GUIDE.md`, `docs/team/AGENT12_REPORT.md`.
- [x] All citations in `docs/USER_GUIDE.md` verified against repository code.

---

## 5. Requests for Agent 0

Task T12.1 is complete and ready for acceptance and auditing by Agent 7.
