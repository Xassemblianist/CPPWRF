#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The job of tools/colab/colab_wrfref.sh in the Debian 13 chroot (tools/colab/remote_chroot_job.sh): the work of
# tools/gcp/remote_wrfref.sh without the bucket. Makes every reference of refs.txt (tools/gcp/wrf_ref_run.py: run_NAME
# and run_NAME/steps, in parallel) and its restart references (tests/verify/make_restart_refs.sh,
# make_resumed_refs.sh), lists the MD5 of every file WRF wrote (refs_md5.txt: "MD5 DIR/FILE"), packs them as
# fetch/refs.tar (links kept; FETCH=1), packs the suite's tar of STAGE (OBJ:CASE,...; tools/colab/stage_data.py --whole)
# as drive/OBJ (remote_chroot_job.sh puts it on Drive) and, with FILES, the files test's references of those cases
# (tools/gcp/stage_files_data.py --cases) as drive/OBJ with _files.tar for _data.tar, then with FILES / CASES builds CPPWRF and runs
# run_wrf_files_test.sh / wrf_suite.py against them, with REAL (NAME,...) builds cppwrf_real and runs
# tests/init_real/run_real_test.sh on the real.exe outputs of run_NAME (their logs end with "exit N").
#
#   wrfref_body.sh J FILES CASES STAGE FETCH [REAL]
#
# J: /opt/job with refs.txt, src/ and data/ (data/runs: the base runs, data/wrf: the WRF executables, as on the VM).
J=$1 FILES=$2 CASES=$3 STAGE=$4 FETCH=$5 REAL=${6:-}
W=$J/wrf_runs; mkdir -p $W
set -x
touch $J/setup.out
# a data tar that did not arrive (remote_chroot_job.sh): no run on partial data, nothing staged
if [ -s $J/data_failed ]; then echo "NO_RUNS: $(tr '\n' ' ' < $J/data_failed)" >> $J/setup.out; exit 0; fi
grep -v '^\s*#' $J/refs.txt | grep -v '^\s*$' | while read -r name base exe sets; do
  echo "python3 $J/src/tools/gcp/wrf_ref_run.py $J/data/runs $W $J/data/wrf/$exe $name $base $sets"
done > $J/cmds.txt
xargs -P "$(nproc)" -I{} sh -c '{}' < $J/cmds.txt > $J/refs.out 2>&1; ref_rc=$?
names=$(grep -v '^\s*#' $J/refs.txt | awk 'NF {print $1}')
rst_rc=0 rsm_rc=0
if [ "${NO_RESTART_REFS:-0}" != 1 ]; then  # NO_RESTART_REFS=1: no rst_ / rsm_ references (tools/colab/movenest_body.sh)
  # shellcheck disable=SC2086
  CPPWRF_WRF_RUNS=$W bash $J/src/tests/verify/make_restart_refs.sh $names >> $J/refs.out 2>&1; rst_rc=$?
  # shellcheck disable=SC2086
  CPPWRF_WRF_RUNS=$W bash $J/src/tests/verify/make_resumed_refs.sh $names >> $J/refs.out 2>&1; rsm_rc=$?
fi
dirs=$(for n in $names; do for p in run rst rsm; do [ -d "$W/${p}_$n" ] && echo "${p}_$n"; done; done)
# shellcheck disable=SC2086
( cd $W && find $dirs -type f \( -name 'wrfout_*' -o -name 'wrfrst_*' -o -name 'auxhist*' -o -name 'wrfinput_*' \
    -o -name 'wrfbdy_*' -o -name 'wrflowinp_*' -o -name 'wrffdda_*' \) | sort | xargs -r md5sum | awk '{print "MD5", $2, $1}' ) > $J/refs_md5.txt
# shellcheck disable=SC2086
[ "$FETCH" = 0 ] || { mkdir -p $J/fetch && tar cf $J/fetch/refs.tar -C $W $dirs; }
if [ -n "$STAGE" ]; then  # the suite's tar of these references, to Drive (remote_chroot_job.sh uploads drive/*)
  # only when every WRF run of refs.txt succeeded (exit 0 and every expected SUCCESS line), and only a finished tar
  # (written aside, moved into drive/): a failed reference or packing must not replace the Drive object
  mkdir -p $J/drive $J/stage_tmp
  stage_bad= files_bad=
  [ "$ref_rc" = 0 ] || stage_bad="wrf_ref_run exit $ref_rc"
  if grep -qE '(WRF|REAL)-FAILED' $J/refs.out; then
    stage_bad="${stage_bad:+$stage_bad; }$(grep -cE '(WRF|REAL)-FAILED' $J/refs.out) failed"
  fi
  for n in $names; do
    steps=1; grep -qE "^\s*$n\s.*\srun\.steps=0(\s|$)" $J/refs.txt && steps=0
    grep -qx "$n SUCCESS" $J/refs.out || stage_bad="${stage_bad:+$stage_bad; }run_$n"
    if [ "$steps" = 1 ]; then
      grep -qx "$n/steps SUCCESS" $J/refs.out || stage_bad="${stage_bad:+$stage_bad; }run_$n/steps"
      # make_restart_refs.sh / make_resumed_refs.sh: "N" (wrf.log only) or "wrf.log:N rsl.error.0000:M" (a dmpar
      # wrf.exe writes its messages to rsl.error.0000)
      if [ "${NO_RESTART_REFS:-0}" != 1 ]; then
        grep -qE "^$n: (wrf\.log:[0-9]+ )?((wrf\.log|rsl\.error\.0000):)?[1-9][0-9]*( +\S+)* +[1-9][0-9]* restart files$" $J/refs.out || files_bad="${files_bad:+$files_bad; }rst_$n"
        grep -qE "^$n: (wrf\.log:[0-9]+ )?((wrf\.log|rsl\.error\.0000):)?[1-9][0-9]*( +\S+)* +[1-9][0-9]* history files$" $J/refs.out || files_bad="${files_bad:+$files_bad; }rsm_$n"
      fi
    fi
  done
  [ "$rst_rc" = 0 ] || files_bad="${files_bad:+$files_bad; }make_restart_refs exit $rst_rc"
  [ "$rsm_rc" = 0 ] || files_bad="${files_bad:+$files_bad; }make_resumed_refs exit $rsm_rc"
  # the staged cases may read the restart references (drop-in resumed cases): none of them staged from a failed set
  [ -z "$files_bad" ] || stage_bad="${stage_bad:+$stage_bad; }restart/resumed runs failed ($files_bad)"
  if [ -n "$stage_bad" ]; then
    echo "STAGE_SKIPPED: WRF runs failed ($stage_bad)" > $J/stage.log
  else
    obj=${STAGE%%:*}
    CPPWRF_WRF_RUNS=$W python3 $J/src/tools/colab/stage_data.py --whole --only "${STAGE#*:}" "$J/stage_tmp/$obj" > $J/stage.log 2>&1 \
      && mv $J/stage_tmp/$obj $J/drive/$obj && echo "STAGED $obj" >> $J/stage.log
    if [ -n "$FILES" ]; then
      fobj=${obj%_data.tar}_files.tar
      if [ -n "$files_bad" ]; then
        echo "STAGE_SKIPPED: $fobj: restart/resumed runs failed ($files_bad)" >> $J/stage.log
      else
        CPPWRF_WRF_RUNS=$W python3 $J/src/tools/gcp/stage_files_data.py --cases "$FILES" "$J/stage_tmp/$fobj" >> $J/stage.log 2>&1 \
          && mv $J/stage_tmp/$fobj $J/drive/$fobj && echo "STAGED $fobj" >> $J/stage.log
      fi
    fi
  fi
  rm -rf $J/stage_tmp
fi
[ -n "$FILES$CASES$REAL" ] || exit 0
targets=cppwrf; [ -z "$REAL" ] || targets="cppwrf cppwrf_real"
# shellcheck disable=SC2086
cmake -S $J/src -B $J/build -DCMAKE_BUILD_TYPE=Release -DCPPWRF_DOUBLE=ON -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_NATIVE=OFF \
  -DCMAKE_CXX_FLAGS="-march=skylake -mtune=skylake" > $J/cmake.log 2>&1 && cmake --build $J/build -j"$(nproc)" --target $targets >> $J/cmake.log 2>&1 \
  || echo "BUILD_FAILED (cmake.log)" >> $J/setup.out
cd $J/src
if [ -n "$REAL" ]; then
  # shellcheck disable=SC2046
  CPPWRF_WRF_RUNS=$W CPPWRF_SCRATCH=$J bash tests/init_real/run_real_test.sh $J/build $(for n in ${REAL//,/ }; do echo "$W/run_$n"; done) > $J/real.log 2>&1
  echo "exit $?" >> $J/real.log
fi
if [ -n "$FILES" ]; then
  # shellcheck disable=SC2086
  CPPWRF_WRF_RUNS=$W CPPWRF_SCRATCH=$J bash tests/verify/run_wrf_files_test.sh $J/build ${FILES//,/ } > $J/files.log 2>&1
  echo "exit $?" >> $J/files.log
fi
if [ -n "$CASES" ]; then
  CPPWRF_WRF_RUNS=$W OMP_NUM_THREADS=4 python3 tests/verify/wrf_suite.py --cpu-dp $J/build/cppwrf \
    --only "$CASES" --exact --jobs "$(( $(nproc) / 4 > 0 ? $(nproc) / 4 : 1 ))" --out $J/suite > $J/suite.log 2>&1
  echo "exit $?" >> $J/suite.log
fi
exit 0
