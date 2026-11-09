#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The release files of CPPWRF VERSION: a source tarball (git archive of the tag vVERSION, or of HEAD with --head) and
# one binary tarball per build flavour (CPack, cmake/Install.cmake), with SHA256SUMS, in OUT. It builds, it does not
# upload: the last lines list what the owner uploads to the GitHub release. No credentials are read or needed.
#
#   tools/release/make_release.sh VERSION [--out DIR] [--head] [--flavours cpu-fp64,gpu-fp64,gpu-fp32]
#                                         [--cuda-archs "80;86;89;90;120"] [--test] [--jobs N]
#
# VERSION must be CMakeLists.txt's project version. The flavours default to cpu-fp64 and, with nvcc on PATH,
# gpu-fp64 and gpu-fp32. The binaries are portable (CPPWRF_NATIVE=OFF). sm_120 needs CUDA >= 12.8: without it the
# default architectures leave it out. --test runs `ctest -L unit` in each build. The build trees are OUT/build-*
# (several GB with CUDA; deleted at the end unless --keep).
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
VERSION=${1:?usage: make_release.sh VERSION [--out DIR] [--head] [--flavours LIST] [--cuda-archs LIST] [--test]}
shift
OUT=$PWD/release-$VERSION HEAD_REF="" FLAVOURS="" ARCHS="" TEST=0 KEEP=0 JOBS=$(nproc)
while [ $# -gt 0 ]; do
  case $1 in
    --out) OUT=$2; shift ;;
    --head) HEAD_REF=HEAD ;;
    --flavours) FLAVOURS=$2; shift ;;
    --cuda-archs) ARCHS=$2; shift ;;
    --test) TEST=1 ;;
    --keep) KEEP=1 ;;
    --jobs) JOBS=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

cmake_version=$(sed -nE 's/^project\(CPPWRF VERSION ([0-9.]+).*/\1/p' "$REPO/CMakeLists.txt")
[ "$cmake_version" = "$VERSION" ] || { echo "VERSION $VERSION is not CMakeLists.txt's project version $cmake_version" >&2; exit 1; }
REF=${HEAD_REF:-v$VERSION}
git -C "$REPO" rev-parse -q --verify "$REF^{commit}" > /dev/null || { echo "no git ref $REF (tag the release or pass --head)" >&2; exit 1; }
if [ -n "$HEAD_REF" ] && [ -n "$(git -C "$REPO" status --porcelain --untracked-files=no)" ]; then
  echo "the working tree has uncommitted changes: the tarballs are built from $REF, not from them" >&2
fi
if [ -z "$FLAVOURS" ]; then
  FLAVOURS=cpu-fp64
  command -v nvcc > /dev/null && FLAVOURS="$FLAVOURS,gpu-fp64,gpu-fp32"
fi
if [ -z "$ARCHS" ] && [[ $FLAVOURS == *gpu* ]]; then
  cuda=$(nvcc --version | sed -nE 's/.*release ([0-9]+)\.([0-9]+).*/\1 \2/p')
  read -r major minor <<< "$cuda"
  if [ "$major" -gt 12 ] || { [ "$major" -eq 12 ] && [ "$minor" -ge 8 ]; }; then ARCHS="80;86;89;90;120"
  else ARCHS="80;86;89;90"; fi
fi
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)

# the sources of the release: git archive writes the commit into src/cases/commit.txt (export-subst)
SRC=$OUT/cppwrf-$VERSION
git -C "$REPO" archive --format=tar.gz --prefix="cppwrf-$VERSION/" -o "$OUT/cppwrf-$VERSION-source.tar.gz" "$REF"
rm -rf "$SRC"
tar -C "$OUT" -xzf "$OUT/cppwrf-$VERSION-source.tar.gz"

for f in ${FLAVOURS//,/ }; do
  case $f in
    cpu-fp64) cuda=OFF dp=ON ;; cpu-fp32) cuda=OFF dp=OFF ;; gpu-fp64) cuda=ON dp=ON ;; gpu-fp32) cuda=ON dp=OFF ;;
    *) echo "unknown flavour $f" >&2; exit 2 ;;
  esac
  B=$OUT/build-$f
  args=(-DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=$cuda -DCPPWRF_DOUBLE=$dp -DCPPWRF_NATIVE=OFF)
  [ $cuda = ON ] && args+=("-DCMAKE_CUDA_ARCHITECTURES=$ARCHS")
  echo "== $f: cmake ${args[*]}"
  cmake -S "$SRC" -B "$B" "${args[@]}" > "$OUT/build-$f.log" 2>&1
  cmake --build "$B" -j "$JOBS" >> "$OUT/build-$f.log" 2>&1 || { tail -20 "$OUT/build-$f.log"; exit 1; }
  if [ $TEST = 1 ]; then
    (cd "$B" && OMP_NUM_THREADS=2 ctest -L unit --output-on-failure) >> "$OUT/build-$f.log" 2>&1 ||
      { echo "ctest -L unit failed in $f ($OUT/build-$f.log)" >&2; exit 1; }
  fi
  (cd "$B" && cpack -B "$OUT" > /dev/null)
  rm -rf "$OUT/_CPack_Packages"
  [ $KEEP = 1 ] || rm -rf "$B"
done
rm -rf "$SRC"

cd "$OUT"
sha256sum cppwrf-"$VERSION"-*.tar.gz > SHA256SUMS
echo
echo "Release files of CPPWRF $VERSION ($REF = $(git -C "$REPO" rev-parse --short "$REF^{commit}")) in $OUT:"
cat SHA256SUMS
echo
echo "Upload to the GitHub release v$VERSION (the owner does this; nothing was uploaded):"
for t in cppwrf-"$VERSION"-*.tar.gz SHA256SUMS; do echo "  $OUT/$t"; done
