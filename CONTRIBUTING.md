# Contributing to CPPWRF

Thank you for helping. CPPWRF is a port of WRF-ARW v4.8.0 whose FP64 results are bit-identical to WRF's. Every
contribution has to keep that, so this page is mostly about how changes are checked.

Questions, bug reports and requests for a WRF option go to the issue tracker, with its templates. Security problems
go by e-mail instead (SECURITY.md). Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## What helps most

- **Bug reports with a small case.** A namelist and a few steps that show the problem; `cppwrf-check` and
  `cppwrf-compare` output (docs/TOOLS.md).
- **Ports of WRF options.** Schemes and options outside the ported list (docs/MIGRATING_FROM_WRF.md, docs/STATUS.md).
  Open an issue first: a port is only accepted with a WRF reference run that the suite can compare with.
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
