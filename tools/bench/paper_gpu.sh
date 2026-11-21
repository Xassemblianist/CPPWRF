#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The GPU side of tools/bench/colab_paper.sh, on the Colab runtime itself (its Ubuntu with the CUDA toolkit; the
# Debian 13 chroot has no CUDA), started next to the chroot job (tools/bench/paper_body.sh):
#
#   paper_gpu.sh STAMP
#
# Builds CPPWRF's three CUDA binaries from /content/cpu/src.tar.gz, each the release build of README.md for this
# GPU: cuda (FP32, the fast default), cuda_sp_id (FP32 with -DCPPWRF_CUDA_FP32_IDENTITY=ON: the CPU FP32 build's
# bits) and cuda_dp (FP64); waits until the chroot job has written STAMP into /opt/paper/body_started; links
# /opt/paper, /opt/job and /opt/wrfbench to the chroot's (the jobs name chroot paths); writes the GPU side's manifest
# (tools/bench/manifest.py collect --side gpu) and /opt/paper/gpu_ready ("cuda=EXIT cuda_sp_id=EXIT cuda_dp=EXIT");
# then runs tools/bench/bench_exec.py agent on /opt/paper/q (each job: one CPPWRF run with nvidia-smi sampling,
# tools/bench/telemetry.py) until the chroot job writes /opt/paper/q/stop. Ends with /opt/paper/gpu_exit.
case "${1:-}" in ""|-*) sed -n '3,14p' "$0"; exit 2 ;; esac  # --help, or no stamp
STAMP=$1
G=/content/paper_gpu R=/content/deb13 P=/content/deb13/opt/paper
rm -rf $G && mkdir -p $G
exec > $G/gpu.log 2>&1
set -x
nvidia-smi
for i in $(seq 20); do  # the chroot job installs debootstrap at the same time: wait for apt's locks
  apt-get -o DPkg::Lock::Timeout=600 -qq update > $G/apt.log 2>&1 &&
    apt-get -o DPkg::Lock::Timeout=600 -qq install -y libnetcdf-dev >> $G/apt.log 2>&1 && break
  sleep 30
done
mkdir -p $G/src && tar xzf /content/cpu/src.tar.gz -C $G/src
st=
for b in cuda:OFF:OFF cuda_sp_id:OFF:ON cuda_dp:ON:OFF; do
  n=${b%%:*} r=${b#*:}
  t0=$SECONDS
  cmake -S $G/src -B $G/$n -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=${r%%:*} \
    -DCPPWRF_CUDA_FP32_IDENTITY=${r#*:} > $G/build-$n.log 2>&1 && cmake --build $G/$n -j"$(nproc)" --target cppwrf >> $G/build-$n.log 2>&1
  st="$st $n=$? (${n}_build_s=$((SECONDS - t0)))"
done
until [ "$(cat $P/body_started 2> /dev/null)" = "$STAMP" ]; do
  [ -e /content/cpu/done ] && exit 0
  sleep 10
done
trap 'touch $P/gpu_exit' EXIT
ln -sfn $R/opt/paper /opt/paper
ln -sfn $R/opt/job /opt/job
ln -sfn $R/opt/wrfbench /opt/wrfbench
for f in $G/build-*.log; do tail -40 "$f" > "$P/gpu_$(basename "$f")"; done
python3 $G/src/tools/bench/manifest.py collect --side gpu --commit /opt/job/commit.txt --cmake cuda=$G/cuda \
  --cmake cuda_sp_id=$G/cuda_sp_id --cmake cuda_dp=$G/cuda_dp --out $P/manifest_gpu.json
echo "$st" > $P/gpu_ready
python3 $G/src/tools/bench/bench_exec.py agent $P/q --bins $G --idle-exit 43200
cp $G/gpu.log $P/gpu_host.log 2> /dev/null
exit 0
