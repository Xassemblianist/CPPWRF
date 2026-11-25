#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

default_mode=$1
shift
if [[ -n ${VCS_REF:-} ]]; then
  printf '%s\n' "$VCS_REF" > /src/src/cases/commit.txt
fi

for mode; do
  backend=OFF
  precision=OFF
  cuda_flags=()
  if [[ $mode == gpu-* ]]; then
    backend=ON
    cuda_flags=(-DCMAKE_CUDA_HOST_COMPILER=g++-14 "-DCMAKE_CUDA_ARCHITECTURES=$CUDA_ARCHS")
  fi
  if [[ $mode == *-fp64 ]]; then
    precision=ON
  fi
  cmake -S /src -B /build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-14 \
    -DCPPWRF_ENABLE_CUDA="$backend" -DCPPWRF_DOUBLE="$precision" -DCPPWRF_NATIVE=OFF \
    -DCPPWRF_PORTABLE_LIBMVEC=ON -DCPPWRF_CUDA_GLIBC_MATH=ON -DCPPWRF_CUDA_FP32_IDENTITY=ON \
    "${cuda_flags[@]}"
  cmake --build /build --parallel "$BUILD_JOBS" --target cppwrf cppwrf_real
  cmake --install /build --prefix /opt/cppwrf --component runtime
  rm -rf /build
done
ln -sfn "cppwrf-$default_mode" /opt/cppwrf/bin/cppwrf
