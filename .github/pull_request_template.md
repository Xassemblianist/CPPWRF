## What and why

<!-- What the pull request changes, and why. Link the issue it closes. -->

## How it was tested

<!-- Commands and results: ctest, the self-test, suite cases (tests/verify/wrf_suite.py), runs. -->

## Checklist

- [ ] `ctest -L unit` passes on a CPU FP64 build, and `examples/selftest/run.sh cpu-fp64` prints PASS.
- [ ] FP64 results are unchanged (bit for bit), or the pull request is not about the model.
- [ ] Model changes: the suite cases I ran are listed above.
- [ ] FP32 results are unchanged, or the change to them is explained.
- [ ] New source files have an SPDX header; docs are updated (docs/USER_GUIDE.md for new configuration keys).
- [ ] The generated docs are current (`python3 tools/gen_option_docs.py --check`).
