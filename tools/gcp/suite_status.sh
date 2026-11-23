#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The result of a CPU suite run (tools/gcp/gcp_run.sh, tools/colab/colab_suite.sh): prints the PASS / FAIL / SKIP counts,
# the SPLIT line (the PASSes by what they compared with: WRF at tolerance 0, WRF by relative L2, CPPWRF itself only)
# and lines of OUT's logs and exits with 0 only for a complete suite without FAIL or SKIP (wrf_suite.py's SUITE line and
# exit 0), passing translator tests and comparator self-test, an empty setup.out and, for the full suite (CASES empty),
# files, ideal files, differential and ctest tests without FAIL or SKIP.
#
#   suite_status.sh OUT CASES [FILES]   (FILES=1: a targeted suite with the output files tests, colab_suite.sh --files)
set -euo pipefail
OUT=$1 CASES=$2 FILES=${3:-0}
# A merged log can repeat a case. Count each case once per result, using its comparison tag when present.
bad=0
if [ -f "$OUT/suite.log" ]; then
  awk -v wanted="$CASES" '
    BEGIN {
      if (wanted != "") {
        n = split(wanted, names, ",")
        for (i = 1; i <= n; i++) expected[names[i]] = 1
      }
    }
    /^(PASS|FAIL|SKIP) / {
      if (NF < 2) invalid = 1
      seen[$2] = 1
      if (!(($1 SUBSEP $2) in block)) totals[$1]++
      block[$1, $2] = 1
    }
    /^PASS / {
      if ($0 ~ /\[vs WRF tol 0\]  \([0-9]+ s\)$/) pass[$2] = "wrf0"
      else if ($0 ~ /\[vs WRF rel L2\]  \([0-9]+ s\)$/) pass[$2] = "wrfl2"
      else if ($0 ~ /\[self only\]  \([0-9]+ s\)$/) pass[$2] = "self"
      else if (!($2 in pass)) pass[$2] = "untagged"
    }
    /^FAIL / { fail[$2] = 1 }
    /^SKIP / { skip[$2] = 1 }
    /^SUITE / {
      summaries++
      if ($0 !~ /^SUITE PASS [0-9]+ FAIL [0-9]+ SKIP [0-9]+$/ ||
          $3 != totals["PASS"] || $5 != totals["FAIL"] || $7 != totals["SKIP"] || !length(block)) {
        print "[gcp_run] INCOMPLETE: SUITE counts disagree with result rows"
        invalid = 1
      }
      delete block
      delete totals
    }
    END {
      for (name in pass) count[pass[name]]++
      printf "PASS %d\nFAIL %d\nSKIP %d\n", length(pass), length(fail), length(skip)
      printf "SPLIT PASS vs WRF at tolerance 0: %d; vs WRF by relative L2: %d; CPPWRF self-consistency only: %d", \
        count["wrf0"], count["wrfl2"], count["self"]
      if (count["untagged"]) printf "; untagged: %d", count["untagged"]
      printf "\n"
      for (name in expected) if (!(name in seen)) {
        print "[gcp_run] INCOMPLETE: missing result for " name
        invalid = 1
      }
      for (name in seen) if (wanted != "" && !(name in expected)) {
        print "[gcp_run] unexpected result for " name
        invalid = 1
      }
      if (invalid || !summaries || length(block) || !length(seen)) exit 1
    }
  ' "$OUT/suite.log" || bad=1
fi
grep -E "^(FAIL|SKIP|ERROR)([ :]|$)" "$OUT/suite.log" 2>/dev/null || true
cat "$OUT/setup.out" 2>/dev/null | grep . && bad=1
grep -q "^exit 0$" "$OUT/unit.log" 2>/dev/null || { echo "[gcp_run] translator tests: $(tail -1 "$OUT/unit.log" 2>/dev/null || echo 'no unit.log')"; bad=1; }
grep -q "^exit 0$" "$OUT/selftest.log" 2>/dev/null || { echo "[gcp_run] comparator self-test: $(grep -E "WRONG|SELFTEST" "$OUT/selftest.log" 2>/dev/null || echo 'no selftest.log')"; bad=1; }
if ! grep -q "^SUITE " "$OUT/suite.log" 2>/dev/null; then echo "[gcp_run] INCOMPLETE: the suite did not end (no SUITE line)"; bad=1
else
  grep "^SUITE " "$OUT/suite.log"
  grep -q "^exit 0$" "$OUT/suite.log" || bad=1
  grep -Eq '^(FAIL|SKIP|ERROR)([ :]|$)|^exit [1-9]' "$OUT/suite.log" && bad=1
fi
if [ -z "$CASES" ] || [ "$FILES" = 1 ]; then  # the full suite: the output files tests too (remote_suite.sh)
  for f in files idealfiles; do
    if [ ! -s "$OUT/$f.log" ]; then echo "[gcp_run] INCOMPLETE: missing or empty $f.log"; bad=1; continue; fi
    echo "$f: PASS $(grep -c '^PASS ' "$OUT/$f.log" || true) FAIL $(grep -c '^FAIL ' "$OUT/$f.log" || true) SKIP $(grep -c '^SKIP ' "$OUT/$f.log" || true)"
    if grep -E '^(FAIL|SKIP)([ :]|$)|^exit [1-9]' "$OUT/$f.log" | cut -c1-200; then bad=1; fi
    grep -q '^exit 0\( \|$\)' "$OUT/$f.log" && grep -q '^PASS ' "$OUT/$f.log" || bad=1
  done
fi
if [ -z "$CASES" ]; then
  for f in diff ctest; do
    if [ ! -f "$OUT/$f.log" ]; then echo "[gcp_run] INCOMPLETE: no $f.log"; bad=1; continue; fi
    echo "$f: $(grep -E 'tests passed' "$OUT/$f.log" || echo 'no ctest summary')"
    grep -E '^\s*[0-9]+ - .*\((Failed|Timeout|SEGFAULT|Subprocess|Skipped|Not Run)|^SKIPPED ' "$OUT/$f.log" || true
    grep -qx "exit 0" "$OUT/$f.log" || bad=1
  done
fi
exit $bad
