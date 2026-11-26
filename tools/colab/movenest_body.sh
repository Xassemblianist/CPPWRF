#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# The job of tools/colab/colab_movenest.sh in the Debian 13 chroot of tools/colab/remote_chroot_job.sh (gcc /
# gfortran 14, glibc 2.41 and NetCDF of the GCP VMs that built the suite's wrf_r8.exe): WRF v4.8.0 with moving nests,
# then the moving-nest references.
#
#   movenest_body.sh J CASES STAGE FETCH VORTEX FP32
#
# J: /opt/job with refs.txt (tests/verify/refs/movenest_refs.txt), wrf_src_md5.txt (the workstation's WRF 4.8.0 source
# tree, "MD5  ./PATH" lines), src/ (CPPWRF) and data/ (matrix_nest.tar: runs/run_N3s_r8 and wrf/wrf_r8.exe, the
# suite's WRF; FP32=1: fp32id4_data.tar too, runs/run_N3_cpu_sp with WRF real*4's inputs).
# 1. The release source (github.com/wrf-model/WRF v4.8.0.tar.gz), its files against wrf_src_md5.txt (wrf_build.txt:
#    SRC_SAME or SRC_DIFFERS with the files).
# 2. WRF's configure option 34 (GNU gfortran / gcc dmpar: -DDM_PARALLEL, MPICH of Debian), -r8, NETCDF = Debian's
#    files, otherwise as the suite's serial wrf_r8.exe, ./compile em_real: nesting 1 (basic: WRF-r8-basic), 2 (preset
#    moves, -DMOVE_NESTS: WRF-r8-move), with VORTEX=1 also 3 (vortex following, -DMOVE_NESTS -DVORTEX_CENTER:
#    WRF-r8-vortex), with FP32=1 also real*4 preset moves (configure without -r8: WRF-r4-move). A serial build
#    (-DSTUBMPI) does not move a nest correctly: share/mediation_nest_move.F sets the nest's i/j_parent_start in the
#    namelist record (which the interpolation, forcing and feedback read) and calls start_domain on the moved nest only
#    #if defined(DM_PARALLEL) && ! defined(STUBMPI). Each executable goes to data/wrf as wrf_<build>.bin with a
#    wrapper wrf_<build>.exe (mpiexec -n 1: one MPI rank); configure.wrf lines in wrf_build.txt. The builds run
#    nproc / 4 at a time (compile -j 4).
# 3. The reproduction check (repro.log): run_N3s_r8's namelist for 10 min with history every minute on both domains,
#    run by the suite's serial wrf_r8.exe (REPa), the dmpar basic WRF with one rank (REPb) and the dmpar moving-nest
#    WRF without a move (REPm); REPb must equal REPa in every variable of every frame of both domains, REPm too except
#    NEST_POS, which must differ (the moving build sets it at each domain's input and rewrites the parent's after each
#    feedback). "REPRO OK" or "REPRO FAILED ...".
# 4. The references of refs.txt with the moving-nest WRF (tools/colab/wrfref_body.sh with NO_RESTART_REFS=1: run_NAME,
#    run_NAME/steps, refs_md5.txt, fetch/refs.tar; with STAGE the suite's tar of those cases, only after REPRO OK),
#    then with CASES the suite cases (wrf_suite.py --exact) of CPPWRF's CPU FP64 build against them. Every rsl / wrf.log
#    of a moving run keeps its "moving", "New SW corner" and "MOAD can not move" lines (moves.log: the moves WRF made,
#    per reference).
J=$1 CASES=$2 STAGE=$3 FETCH=$4 VORTEX=${5:-0} FP32=${6:-0}
W=/opt/wrfmove
mkdir -p $W $J/data/wrf
touch $J/setup.out
set -x
if [ -s $J/data_failed ]; then echo "NO_RUNS: $(tr '\n' ' ' < $J/data_failed)" >> $J/setup.out; exit 0; fi
export DEBIAN_FRONTEND=noninteractive
apt-get -qq update > $J/apt.log 2>&1 && apt-get -qq install -y tcsh m4 perl make curl ca-certificates file mpich libmpich-dev >> $J/apt.log 2>&1 ||
  { echo "MOVE_SETUP_FAILED apt (apt.log)" >> $J/setup.out; exit 0; }
{ gfortran --version | head -1; gcc --version | head -1; mpichversion 2> /dev/null | head -1; } > $J/wrf_build.txt 2>&1
mkdir -p /opt/netcdf  # WRF's configure wants one NETCDF directory: Debian's files
ln -sfn /usr/bin /opt/netcdf/bin; ln -sfn /usr/include /opt/netcdf/include; ln -sfn /usr/lib/x86_64-linux-gnu /opt/netcdf/lib

# 1. the source
if [ ! -s $W/wrf-src.tar.gz ]; then
  curl -sfL --retry 5 -o $W/src.part https://github.com/wrf-model/WRF/releases/download/v4.8.0/v4.8.0.tar.gz &&
    mv $W/src.part $W/wrf-src.tar.gz || { echo "MOVE_SETUP_FAILED WRF source download" >> $J/setup.out; exit 0; }
fi
echo "WRF source v4.8.0.tar.gz md5 $(md5sum < $W/wrf-src.tar.gz | cut -c1-32)" >> $J/wrf_build.txt
rm -rf $W/src && mkdir -p $W/src && tar xzf $W/wrf-src.tar.gz -C $W/src --strip-components=1
( cd $W/src && find . -type f | sort | xargs md5sum ) > $J/wrf_src_colab_md5.txt
python3 - $J/wrf_src_md5.txt $J/wrf_src_colab_md5.txt >> $J/wrf_build.txt <<'EOF'
import sys
def load(p):
    return {l[34:].strip(): l[:32] for l in open(p) if len(l) > 34}
ws, co = load(sys.argv[1]), load(sys.argv[2])
# the workstation's tree is a git checkout: its .git* files and .ci / .github are not in the release tarball
skip = lambda p: p.startswith(("./.git", "./.ci/", "./.github/"))
diff = sorted(p for p in set(ws) | set(co) if not skip(p) and ws.get(p) != co.get(p))
print("SRC_SAME" if not diff else "SRC_DIFFERS %d files: %s" % (len(diff), " ".join(diff[:40])))
EOF

# 2. the builds (nproc / 4 at a time)
build_wrf() {  # NAME NESTING [-r8]
  local d=$W/WRF-$1
  rm -rf $d && cp -a $W/src $d && cd $d || return 1
  export NETCDF=/opt/netcdf WRFIO_NCD_LARGE_FILE_SUPPORT=1
  printf '34\n%s\n' "$2" | ./configure $3 > configure.log 2>&1
  [ -f configure.wrf ] || return 1
  ./compile -j 4 em_real > compile.log 2>&1
  [ -x main/wrf.exe ]
}
builds="r8-basic:1:-r8 r8-move:2:-r8"
[ "$VORTEX" = 1 ] && builds="$builds r8-vortex:3:-r8"
[ "$FP32" = 1 ] && builds="$builds r4-move:2:"
par=$(( $(nproc) / 4 )); [ $par -ge 1 ] || par=1
t0=$SECONDS
k=0
for b in $builds; do
  IFS=: read -r n nest r8 <<< "$b"
  ( build_wrf "$n" "$nest" "$r8" ) > /dev/null 2>&1 &
  k=$((k + 1)); [ $((k % par)) -ne 0 ] || wait
done
wait
for b in $builds; do
  IFS=: read -r n nest r8 <<< "$b"
  d=$W/WRF-$n
  { echo "== WRF-$n (nesting $nest${r8:+, $r8})"
    grep -E '^(SFC|SCC|DM_FC|DM_CC|FCOPTIM|FCBASEOPTS_NO_G|PROMOTION|RWORDSIZE|CFLAGS_LOCAL|ARCH_LOCAL)\s' $d/configure.wrf 2> /dev/null
    grep -oE -- '-D(DM_PARALLEL|STUBMPI|MOVE_NESTS|VORTEX_CENTER)' $d/configure.wrf 2> /dev/null | sort -u | tr '\n' ' '; echo; } >> $J/wrf_build.txt
  x=wrf_${n/-/_}
  if [ -x $d/main/wrf.exe ] && ! grep -q -- '-DSTUBMPI' $d/configure.wrf; then
    cp $d/main/wrf.exe $J/data/wrf/$x.bin
    printf '#!/bin/sh\n# one MPI rank of the dmpar build (tools/colab/movenest_body.sh)\nexec mpiexec -launcher fork -n 1 %s "$@"\n' \
      $J/data/wrf/$x.bin > $J/data/wrf/$x.exe
    chmod +x $J/data/wrf/$x.exe
    echo "BUILT WRF-$n $x.exe (mpiexec -n 1 $x.bin) md5 $(md5sum < $d/main/wrf.exe | cut -c1-32)" >> $J/wrf_build.txt
  else
    echo "MOVE_BUILD_FAILED WRF-$n" >> $J/setup.out
    tail -30 $d/configure.log > $J/WRF-$n.configure.log 2> /dev/null
    grep -B2 -A8 -m5 -E 'Error|error:' $d/compile.log > $J/WRF-$n.compile_errors.log 2> /dev/null
  fi
done
echo "WRF builds: $((SECONDS - t0)) s" >> $J/wrf_build.txt
[ -x $J/data/wrf/wrf_r8_move.exe ] || exit 0

# 3. the reproduction check
R=$J/repro_runs; mkdir -p $R
for x in a:wrf_r8.exe b:wrf_r8_basic.exe m:wrf_r8_move.exe; do
  [ -x $J/data/wrf/${x#*:} ] || continue
  python3 $J/src/tools/gcp/wrf_ref_run.py $J/data/runs $R $J/data/wrf/${x#*:} REP${x%%:*} run_N3s_r8 \
    time_control.run_seconds=600 time_control.history_interval_s=60,60 run.steps=0 >> $J/repro.log 2>&1 &
done
wait
python3 - $R >> $J/repro.log 2>&1 <<'EOF'
import glob, os, sys
import netCDF4, numpy as np
R = sys.argv[1]
ok = True
def files(n):
    return sorted(glob.glob(os.path.join(R, "run_" + n, "wrfout_d0*")))
fa = files("REPa")
for dom in ("d01", "d02"):  # 10 min with history every minute on both domains: 11 frames each
    nf = sum(len(netCDF4.Dataset(f)["Times"]) for f in fa if os.path.basename(f).startswith("wrfout_" + dom))
    if nf < 11:
        print(f"REPRO REPa: {nf} frames of {dom}, 11 expected"); ok = False
for other, allowed in (("REPb", set()), ("REPm", {"NEST_POS"})):
    fo = files(other)
    if not fa or [os.path.basename(f) for f in fa] != [os.path.basename(f) for f in fo]:
        print(f"REPRO {other}: files differ: {fa} {fo}"); ok = False; continue
    for a, b in zip(fa, fo):
        A, B = netCDF4.Dataset(a), netCDF4.Dataset(b)
        diff = []
        for v in A.variables:
            if v not in B.variables:
                diff.append(v + "(missing)"); continue
            x, y = A[v][:], B[v][:]
            if x.shape != y.shape or not np.array_equal(np.ma.getdata(x), np.ma.getdata(y)):
                diff.append(v)
        extra = [v for v in B.variables if v not in A.variables]
        bad = [v for v in diff if v not in allowed] + extra
        print(f"REPRO {other} {os.path.basename(a)}: {len(A['Times'])} frames, differing {diff or 'none'}"
              + (f", NOT ALLOWED {bad}" if bad else ""))
        if bad:
            ok = False
        if other == "REPm" and "NEST_POS" not in diff:
            # the moving build's NEST_POS (terrain + 500 m) differs from the basic build's: an equal one means the
            # build did not move nests (-DMOVE_NESTS missing)
            print(f"REPRO {other} {os.path.basename(a)}: NEST_POS equal: not the moving build"); ok = False
print("REPRO OK" if ok else "REPRO FAILED")
EOF
grep -h "moving\|MOAD" $R/run_REPm/rsl.error.0000 $R/run_REPm/wrf.log 2> /dev/null | head -5 >> $J/repro.log
grep -qx "REPRO OK" $J/repro.log || { echo "REPRO_FAILED (repro.log): no staging" >> $J/setup.out; STAGE=; }

# 4. the references and the suite cases (wrfref_body.sh, which builds CPPWRF when CASES is set)
# no restart references: a moving nest in a resumed run is not supported by CPPWRF (nest_driver.cpp)
NO_RESTART_REFS=1 bash $J/src/tools/colab/wrfref_body.sh $J "" "$CASES" "$STAGE" "$FETCH"
for d in $J/wrf_runs/run_* $J/wrf_runs/run_*/steps; do
  [ -d "$d" ] || continue
  echo "== ${d#$J/wrf_runs/}: $(grep -h -E '^ *moving|Grid +[0-9]+ New SW corner|MOAD can not move' "$d"/rsl.error.0000 "$d"/wrf.log 2> /dev/null | tr -s ' ' | paste -sd ';')"
done > $J/moves.log
exit 0
