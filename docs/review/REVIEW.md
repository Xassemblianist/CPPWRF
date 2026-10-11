# Agent 0 reviews

## T3.1 (Agent 3: src/io, scripts) — ACCEPTED, 2026-10-11
Independently re-checked by Agent 0 (not relying on Agent 3's report):
- Built in a separate project with `-Wall -Wextra -fsanitize=address,undefined`: no warnings;
  ctest 3/3 pass.
- Adversarial config probe: trailing junk, int/double overflow, nan, key before section,
  duplicate key all raise errors with file:line; repeated [section] headers merge.
- Strided F32/F64 snapshot written by SnapshotWriter and read by tests/verify/verify.py
  (an independent reader): bit-exact, offsets/nbytes correct.

Process issue (REQUIRED for Agent 3): Agent 3 ran `git commit --amend` on the old baseline
commit (now ac18bb7, author "CPPWRF Engineer"). Only Agent 1 runs git. No harm done:
Agent 1 re-initialises the repository anyway.

ADVISORY for Agent 1: add `__pycache__/` and `build*/` to .gitignore.
