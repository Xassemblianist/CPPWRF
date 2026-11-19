# Contributing to CPPWRF

Thank you for helping. CPPWRF is a port of WRF-ARW v4.8.0 whose FP64 results are bit-identical to WRF's. Every
contribution has to keep that, so this page is mostly about how changes are checked.

Questions, bug reports and requests for a WRF option go to the issue tracker, with its templates. Security problems
go by e-mail instead (SECURITY.md). Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## What helps most

- **Bug reports with a small case.** A namelist and a few steps that show the problem; `cppwrf-check` and
  `cppwrf-compare` output (docs/TOOLS.md).
- **Ports of WRF options.** Schemes and options outside the ported list (docs/MIGRATING_FROM_WRF.md, docs/STATUS.md).
  Open an issue first. A port is verified with a WRF reference run that the suite compares with; until then it can
  be merged at a lower verification tier (below).
- **Tools, documentation, packaging and translations** (docs/tr/ has the Turkish pages).

## Building and testing

docs/INSTALL.md has the build. Before a pull request:

```bash
cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON
cmake --build build-cpu -j "$(nproc)"
ctest --test-dir build-cpu -L unit --output-on-failure      # unit tests, the tools, the Python package, lints
examples/selftest/run.sh cpu-fp64                             # must print the reference hash
```

Changes to the model (src/dyn, src/phys, the kernels, the math functions, src/core numerics) also need the WRF
identity suite, `tests/verify/wrf_suite.py`. It compares CPPWRF with WRF v4.8.0 reference runs at tolerance 0 in
FP64. The reference data are large, and the maintainers run the suite on every pull request that touches the model;
say in the pull request which cases you ran.

### Development runs on the CI

The CPU suite runs on GitHub-hosted runners (`tools/ci/ci_submit.sh`), a pool shared by everyone. While a change is
being developed, run only the cases it can affect:

```bash
tools/ci/affected_cases.py origin/main HEAD            # which cases, and why (stderr)
tools/ci/ci_submit.sh HEAD --affected origin/main       # submit just those
```

`affected_cases.py` maps each changed file through `tools/ci/affected_map.txt`. A physics scheme's files select the
cases that use that scheme, including their 1 h versions. A scheme that most cases use, such as WSM6 or YSU, keeps
all its short cases but only 6 of its long ones. The dynamics, core, I/O, driver and build files select a
fixed core sample: the R1/T1 dycore, P4 for 3 steps and 1 h, the N3 nest, a restart, an ideal case and Straka.
Documentation selects nothing. Tools and tests outside the suite select only the translator tests and the self-test.
A file the table does not cover selects the core sample and prints a warning. When you add a scheme directory under
src/phys, give it a line in that table (`ctest -R affected_cases` checks this).

**`--affected` is for development feedback only. It never proves a merge.** Every merge to main is proven by the full
suite, `tools/ci/ci_submit.sh <commit> --full`, before it lands, and again by main's suite run on every push.
Nothing reaches main on an affected-only run.

## Rules for model code

- **FP64 results must not change.** `cpu-fp64` and `gpu-fp64` give WRF real*8's values bit for bit, and a change
  that alters a single bit of any suite case is a bug, even if it is "more accurate". This includes the order of
  sums, the constants and the math functions.
- **FP32 output must not change either** unless the pull request is about FP32 and says so.
- **No value-changing compiler flags.** `-ffast-math`, `-Ofast` and `--use_fast_math` are rejected by the
  configuration.
- **WRF's source is the specification.** Comments name the WRF routine a function ports (file and subroutine), and
  differences from WRF are explained where they are.
- **Every kernel launch that can raise a WRF fatal stop is followed by `fatal_check()`** (`ctest -R lint_fatal_check`).
- **Configuration keys** are registered from their string literals in src/ (src/cases/config_keys.cpp); document new
  ones in docs/USER_GUIDE.md.
- **Register a new option's tier** in `data/option_tiers.txt` when it is merged before its whole-model WRF
  comparison (the one table: the model, cppwrf_real and tools/wrf2cppwrf.py read it; do not add another mechanism).
  One row per option value, e.g.

  ```
  phys.mp_physics   physics.mp_physics   4   column_verified   WSM5 (tests/phys/mp_wsm5)
  ```

  `column_verified` needs a column driver that matches WRF's Fortran bit for bit (tests/phys/<scheme>), the minimum
  for a new scheme; `unverified` (ported only) is for options where no column driver is possible, with the reason in
  the note. Options without a row are verified. Such an option runs only with `[cppwrf] allow_unverified = true`;
  list its tier in CHANGELOG.md and docs/STATUS.md, and delete the row when the suite proves it against WRF.

## Style

- C++20 and CUDA in the style of the surrounding code: 2-space indent, lines of about 120 columns, names as in WRF
  where a variable ports a WRF one.
- Python 3.9+, 120 columns, the standard library plus `numpy` and `netCDF4` (`xarray` optional).
- Every new source file starts with `SPDX-License-Identifier: GPL-3.0-or-later`.
- docs/MIGRATING_FROM_WRF.md's option tables are generated: after changing tools/wrf2cppwrf.py's tables, run
  `python3 tools/gen_option_docs.py` (ctest `docs_options_current` checks it).

## Pull requests

- One topic per pull request, on a branch of your fork, with the template's checklist filled in.
- Commits with a clear message: what changed and why.
- By contributing you agree that your contribution is licensed under the GPL-3.0-or-later, CPPWRF's licence.
