#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The CPU FP64 suite of a commit on free GitHub-hosted runners (the work of tools/colab/colab_suite.sh): the workflow
# suite.yml of github.com/Xassemblianist/cppwrf-data, with the reference data of that repository
# (release assets). Works for any local commit: a commit that is not on origin/main travels as a git bundle (only the
# objects that origin/main lacks) uploaded to its `jobs` release and deleted again after the run.
#
#   tools/ci/ci_submit.sh [COMMIT] [--cases NAME,...] [--full] [--data TAR,...] [--slow] [--suite-jobs N]
#       [--data-tag TAG] [--out DIR] [--hours H] [--no-wait]
#
# COMMIT: default HEAD. Without --cases the full suite: every CPU FP64 case of wrf_suite.py but the slow ones (packed
# into --suite-jobs jobs, default 13, by their recorded times), the output files tests (run_wrf_files_test.sh), the
# ideal files test, the differential and ctest tests (tools/gcp/run_diff_tests.sh), the translator tests and the
# comparators' self-test. --cases: only these suite cases (exact names) and the translator tests and self-test;
# --full adds the other jobs. --data: the data tars instead of the cases' own (wrf_suite.py --data-for).
# --slow: the slow cases too (their tars must be in the data release). --data-tag: the cppwrf-data release
# (default data-v1). The script waits for the run (--hours, default 8; --no-wait: print the run's URL and exit),
# downloads the result files (no data) to --out (default ~/cppwrf_scratch/ci/<run id>) and prints the summary of
# tools/gcp/suite_status.sh (SUITE PASS n FAIL n SKIP n, files:, idealfiles:, diff:, ctest:); exit 0 only for a run
# without FAIL or SKIP whose gate job passed. Needs gh, logged in with the rights to start its workflows and
# upload to its releases (no token is stored anywhere).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CI=${CPPWRF_CI_REPO:-Xassemblianist/cppwrf-data}
COMMIT=HEAD CASES= FULL=0 DATA= SLOW=0 NJ=13 DTAG=data-v1 OUT= HOURS=8 WAIT=1
while [ $# -gt 0 ]; do
  case $1 in
    --cases) CASES=$2; shift 2 ;; --full) FULL=1; shift ;; --data) DATA=$2; shift 2 ;; --slow) SLOW=1; shift ;;
    --suite-jobs) NJ=$2; shift 2 ;; --data-tag) DTAG=$2; shift 2 ;; --out) OUT=$2; shift 2 ;;
    --hours) HOURS=$2; shift 2 ;; --no-wait) WAIT=0; shift ;;
    -*) sed -n '3,25p' "$0"; exit 2 ;;
    *) COMMIT=$1; shift ;;
  esac
done
SHA=$(git -C "$REPO" rev-parse --verify "$COMMIT^{commit}")
git -C "$REPO" fetch -q origin main 2> /dev/null || echo "[ci_submit] note: git fetch origin failed; origin/main as known locally"
BASE=$(git -C "$REPO" rev-parse origin/main)
TAG="ci-$(date +%Y%m%d-%H%M%S)-${SHA:0:8}"
BUNDLE=
if ! git -C "$REPO" merge-base --is-ancestor "$SHA" "$BASE"; then
  # a bundle of the commits that origin/main lacks, without touching any ref: header, the boundary commits as
  # prerequisites, the commit as refs/heads/ci, then a thin pack (git bundle's own format, v2)
  BUNDLE=$TAG.bundle
  TMPD=$(mktemp -d "${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/ci_bundle.XXXXXX")
  trap 'rm -rf "$TMPD"' EXIT
  { echo "# v2 git bundle"
    git -C "$REPO" rev-list --boundary "$SHA" "^$BASE" | sed -n 's/^-\(.*\)/-\1/p'
    echo "$SHA refs/heads/ci"; echo
    printf '%s\n^%s\n' "$SHA" "$BASE" | git -C "$REPO" pack-objects --revs --thin --stdout --delta-base-offset -q
  } > "$TMPD/$BUNDLE"
  git -C "$REPO" bundle verify -q "$TMPD/$BUNDLE" > /dev/null 2>&1 || { echo "[ci_submit] the bundle does not verify"; exit 1; }
  gh release view jobs -R "$CI" > /dev/null 2>&1 ||
    gh release create jobs -R "$CI" --title jobs --notes "git bundles of the commits under test (deleted after each run)" > /dev/null
  gh release upload jobs "$TMPD/$BUNDLE" -R "$CI" > /dev/null
  echo "[ci_submit] bundle $BUNDLE ($(stat -c %s "$TMPD/$BUNDLE") bytes, $(git -C "$REPO" rev-list --count "$SHA" "^$BASE") commits on origin/main ${BASE:0:8})"
fi
cleanup_bundle() { [ -z "$BUNDLE" ] || gh release delete-asset jobs "$BUNDLE" -R "$CI" -y > /dev/null 2>&1 || true; }
gh workflow run suite.yml -R "$CI" -f ref="$SHA" -f bundle="$BUNDLE" -f cases="$CASES" -f full="$FULL" -f data="$DATA" \
  -f slow="$SLOW" -f suite_jobs="$NJ" -f data_tag="$DTAG" -f tag="$TAG" > /dev/null
RUN=
for _ in $(seq 30); do  # the run of this dispatch, by its name
  sleep 5
  RUN=$(gh run list -R "$CI" --workflow suite.yml --limit 20 --json databaseId,displayTitle \
        --jq ".[] | select(.displayTitle == \"$TAG\") | .databaseId" | head -1)
  [ -n "$RUN" ] && break
done
[ -n "$RUN" ] || { echo "[ci_submit] the run $TAG did not appear"; cleanup_bundle; exit 1; }
URL=https://github.com/$CI/actions/runs/$RUN
echo "[ci_submit] commit ${SHA:0:8}, run $RUN: $URL"
[ "$WAIT" = 1 ] || exit 0
OUT=${OUT:-${CPPWRF_SCRATCH:-$HOME/cppwrf_scratch}/ci/$RUN}
mkdir -p "$OUT"
echo "$URL" > "$OUT/run.txt"
t0=$SECONDS last=
while :; do  # one loop on the run's status (every minute), the jobs that ended as they end
  st=$(gh run view "$RUN" -R "$CI" --json status,conclusion --jq '.status + " " + .conclusion' 2> /dev/null || echo "unknown ")
  done_jobs=$(gh run view "$RUN" -R "$CI" --json jobs --jq '[.jobs[] | select(.status == "completed")] | length' 2> /dev/null || echo "?")
  line="$st, $done_jobs jobs done"
  [ "$line" = "$last" ] || { echo "[ci_submit] $(date +%H:%M) $line"; last=$line; }
  case $st in completed*) break ;; esac
  [ $((SECONDS - t0)) -lt $((HOURS * 3600)) ] || { echo "[ci_submit] still running after $HOURS h: $URL"; exit 1; }
  sleep 60
done
cleanup_bundle
for i in 1 2 3 4 5; do  # the artifact can take a moment to be listed after the run ends
  rm -rf "$OUT/results.tmp"  # (gh run download does not overwrite files)
  gh run download "$RUN" -R "$CI" -n results -D "$OUT/results.tmp" > /dev/null 2>&1 && cp -r "$OUT/results.tmp/." "$OUT/" && rm -rf "$OUT/results.tmp" && break
  [ $i = 5 ] && { echo "[ci_submit] no results artifact: $URL"; exit 1; }
  sleep 20
done
echo "[ci_submit] results in $OUT ($(wc -l < "$OUT/jobs.txt") jobs; $URL)"
cat "$OUT/jobs.txt"
FILES=$FULL; [ -z "$CASES" ] && FILES=0
bash "$REPO/tools/gcp/suite_status.sh" "$OUT" "$CASES" "$FILES" && good=1 || good=0
[ "${st#completed }" = success ] || { echo "[ci_submit] run conclusion: ${st#completed }"; good=0; }
[ $good = 1 ]
