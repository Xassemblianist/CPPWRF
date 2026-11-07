#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The chroot job of tools/colab/colab_regression.sh: the legacy regression tool (tools/regression) in the Debian 13
# chroot the suite uses, with the suite's build flags (CPPWRF_NATIVE=OFF, -march=skylake).
#
#   regression_body.sh J MODE COMMIT
#
# J holds src/ (git archive of COMMIT). MODE update: tools/regression/update_golden.sh --quick --no-gpu (new goldens
# from this tree), then a second tools/regression/run.sh --quick --no-gpu against them (run2.log); MODE check: run.sh
# --quick --no-gpu against the goldens in the tree (run2.log). Logs in J: setup.out, apt.log, update.log, run2.log
# (ends with "exit N", run.sh's exit code), last_report.txt; fetch/regression.tar.gz: tools/regression/logs,
# last_report.md and (MODE update) golden/.
J=$1 MODE=$2 COMMIT=$3
set -x
touch $J/setup.out
case $MODE in update|check) ;; *) echo "BAD_MODE $MODE" >> $J/setup.out; exit 1 ;; esac
command -v ninja > /dev/null || { apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get -qq install -y ninja-build; } \
  > $J/apt.log 2>&1 || { echo "SETUP_FAILED ninja-build (apt.log)" >> $J/setup.out; exit 1; }
cd $J/src || exit 1
export CPPWRF_REG_COMMIT=$COMMIT CPPWRF_REG_BUILD_JOBS=$(nproc)
export CPPWRF_REG_CMAKE_ARGS="-DCPPWRF_NATIVE=OFF -DCMAKE_CXX_FLAGS=-march=skylake"
lscpu | grep -E '^(Vendor ID|Model name|CPU\(s\)):' > $J/cpu.txt
if [ "$MODE" = update ]; then
  t0=$SECONDS
  bash tools/regression/update_golden.sh --quick --no-gpu > $J/update.log 2>&1
  echo "exit $? ($((SECONDS - t0)) s)" >> $J/update.log
fi
t0=$SECONDS
bash tools/regression/run.sh --quick --no-gpu > $J/run2.log 2>&1
echo "exit $? ($((SECONDS - t0)) s)" >> $J/run2.log
cp tools/regression/last_report.md $J/last_report.txt 2> /dev/null
mkdir -p $J/fetch
tar czf $J/fetch/regression.tar.gz -C tools/regression logs last_report.md $([ "$MODE" = update ] && echo golden)
exit 0
