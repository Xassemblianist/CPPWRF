#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The Colab side of tools/bench/colab_paper.sh (root, under nohup): the Debian 13 chroot, the data, the GPU side and
# the chroot job of the paper benchmark (tools/bench/PROTOCOL.md). No cloud bucket: the data tars stream from the
# Drive remote cppwrf-drive (folder cppwrf-data) into tar x, a tar not on Drive comes from the public GitHub release
# Xassemblianist/cppwrf-data data-v1 (512 MiB parts, each checked against the release's SHA256SUMS).
#
#   paper_job.sh SET CASES STAMP DATA [PAPER_RUN_OPTION ...]
#
# /content/in: src.tar.gz (git archive), commit.txt, wps.tar.gz, rclone.conf (only the cppwrf-drive section),
# save_lday (present: PAPER_SAVE_LDAY=1). DATA: the tars, comma-separated, unpacked into /opt/job/data in order.
# Steps: debootstrap trixie into /content/deb13 with build-essential, cmake, gfortran, NetCDF, python3-netcdf4 and
# numpy (once per session, as tools/colab/remote_chroot_job.sh); /opt/job = src/, data/, commit.txt, wps.tar.gz;
# tools/bench/paper_gpu.sh STAMP on the runtime; tools/bench/paper_body.sh in the chroot. At the end (also when a
# step fails): /content/paper_job/paper.tar.gz (the benchmark's results) and results.tar.gz (this job's logs), both
# also copied to Drive (cppwrf-drive:cppwrf-g4results/paper_STAMP/), the job's drive/ tars (paper_lday_data.tar) to
# cppwrf-drive:cppwrf-data, rclone.conf deleted, then /content/paper_job/done. Progress: /content/paper_job/job.log,
# /content/deb13/opt/paper/out/paper.log.
case "${1:-}" in full|gpu) ;; *) sed -n '3,18p' "$0"; exit 2 ;; esac
SET=$1 CASES=$2 STAMP=$3 DATA=$4; shift 4
I=/content/in C=/content/paper_job R=/content/deb13
J=$R/opt/job GH=https://github.com/Xassemblianist/cppwrf-data/releases/download/data-v1
rm -rf $C; mkdir -p $C/parts
exec > $C/job.log 2>&1
RC="--config $I/rclone.conf"
finish() {
  [ -f $J/fetch/paper.tar.gz ] && cp $J/fetch/paper.tar.gz $C/
  mkdir -p $C/res; cp -f $C/*.log $C/*.txt $C/res/ 2> /dev/null
  tar czf $C/results.tar.gz -C $C/res .
  for f in $C/paper.tar.gz $C/results.tar.gz; do
    [ -f $f ] && { rclone $RC copyto $f "cppwrf-drive:cppwrf-g4results/paper_$STAMP/$(basename $f)" >> $C/drive.log 2>&1 ||
      echo "DRIVE_FAILED $(basename $f)" >> $C/setup.txt; }
  done
  for f in $J/drive/*; do
    [ -f "$f" ] || continue
    rclone $RC copyto "$f" "cppwrf-drive:cppwrf-data/$(basename "$f")" --drive-chunk-size 256M >> $C/drive.log 2>&1 &&
      echo "DRIVE $(basename "$f") $(stat -c %s "$f")" >> $C/data.txt || echo "DRIVE_FAILED $(basename "$f")" >> $C/setup.txt
  done
  rm -f $I/rclone.conf
  date > $C/done
}
trap finish EXIT
: > $C/setup.txt; : > $C/data.txt
set -x
export DEBIAN_FRONTEND=noninteractive
command -v rclone > /dev/null && command -v debootstrap > /dev/null ||
  { apt-get -qq update && apt-get -qq install -y debootstrap rclone > /dev/null; } || { echo "SETUP_FAILED apt" >> $C/setup.txt; exit 1; }
mnt() { for d in proc sys dev dev/pts; do mountpoint -q $R/$d || mount --bind /$d $R/$d; done; }
if [ ! -f $R/.ready ]; then  # once per session (a --keep session reuses it)
  for d in dev/pts dev sys proc; do umount $R/$d 2> /dev/null; done
  rm -rf $R
  debootstrap trixie $R http://deb.debian.org/debian > $C/debootstrap.log 2>&1 || { echo "SETUP_FAILED debootstrap" >> $C/setup.txt; exit 1; }
  mnt
  cp /etc/resolv.conf $R/etc/resolv.conf
  chroot $R env -i PATH=/usr/sbin:/usr/bin:/sbin:/bin DEBIAN_FRONTEND=noninteractive bash -c 'apt-get -qq update &&
    apt-get -qq install -y build-essential cmake gfortran libnetcdf-dev libnetcdff-dev python3-netcdf4 python3-numpy > /dev/null' \
    > $C/packages.log 2>&1 || { echo "SETUP_FAILED packages (packages.log)" >> $C/setup.txt; exit 1; }
  touch $R/.ready
fi
mnt
chroot $R dpkg-query -W -f='${Package} ${Version}\n' libc6 gcc-14 gfortran-14 cmake libnetcdf22 python3-numpy \
  python3-netcdf4 > $C/versions.txt 2>&1
dpkg-query -W -f='${Package} ${Version}\n' > $C/host_packages.txt 2>&1  # the runtime's own (the CUDA side)
{ cat /etc/os-release; uname -a; nvidia-smi 2>&1 | head -12; nvcc --version 2>&1 | tail -2; } > $C/host.txt 2>&1
rm -rf $J; mkdir -p $J/src $J/data
tar xzf $I/src.tar.gz -C $J/src || { echo "SRC_FAILED" >> $C/setup.txt; exit 1; }
cp $I/commit.txt $I/wps.tar.gz $J/
# the GPU side builds while the data come and the chroot job builds (it waits for /opt/paper/body_started = STAMP)
nohup bash $J/src/tools/bench/paper_gpu.sh $STAMP > /dev/null 2>&1 &
drive_tar() {  # TAR: rclone cat | tar x, three tries
  local t=$1 i
  for i in 1 2 3; do
    rclone $RC cat "cppwrf-drive:cppwrf-data/$t" 2>> $C/fetch_$t.log | tee >(sha256sum | cut -c1-64 > $C/sha_$t) |
      tar xf - -C $J/data 2>> $C/fetch_$t.log && [ "${PIPESTATUS[0]}" = 0 ] && return 0
    sleep 30
  done
  return 1
}
gh_tar() {  # TAR: its release parts (SHA256SUMS) downloaded one by one, checked, streamed into tar x
  local t=$1
  [ -s $C/SHA256SUMS ] || curl -sSfL --retry 5 -o $C/SHA256SUMS "$GH/SHA256SUMS" || return 1
  grep -q "  $t\.part" $C/SHA256SUMS || return 1
  ( set -o pipefail
    grep "  $t\.part" $C/SHA256SUMS | while read -r sum p; do
      ok=0
      for i in 1 2 3 4 5; do
        curl -sSfL --retry 5 -o $C/parts/$p "$GH/$p" 2>> $C/fetch_$t.log &&
          [ "$(sha256sum $C/parts/$p | cut -c1-64)" = "$sum" ] && { ok=1; break; }
        echo "retry $p $i" >> $C/fetch_$t.log; sleep 20
      done
      [ $ok = 1 ] || { echo "PART_FAILED $p" >> $C/fetch_$t.log; exit 1; }
      cat $C/parts/$p; rm -f $C/parts/$p
    done | tee >(sha256sum | cut -c1-64 > $C/sha_$t) | tar xf - -C $J/data ) 2>> $C/fetch_$t.log
}
for t in ${DATA//,/ }; do
  t0=$SECONDS
  if rclone $RC lsf "cppwrf-drive:cppwrf-data/$t" 2> /dev/null | grep -qx "$t" && drive_tar $t; then src=drive
  elif gh_tar $t; then src=github
  else echo "DATA_FAILED $t" >> $C/setup.txt; continue; fi
  sleep 2  # the digest's process substitution ends after tar
  echo "TAR $t $src $((SECONDS - t0)) s sha256 $(cat $C/sha_$t 2> /dev/null)" >> $C/data.txt
done
E=; [ -e $I/save_lday ] && E=PAPER_SAVE_LDAY=1
chroot $R env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME=/root $E \
  bash /opt/job/src/tools/bench/paper_body.sh /opt/job "$SET" "$CASES" "$STAMP" "$@" > $C/body.log 2>&1
cp $J/setup.out $C/body_setup.txt 2> /dev/null
cat $J/setup.out >> $C/setup.txt 2> /dev/null
exit 0
