#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The build and run manifest of a benchmark session (tools/bench/PROTOCOL.md, "Manifest").

  manifest.py collect --out FILE.json [--side NAME] [--commit REV_OR_FILE] [--cmake NAME=BUILD_DIR ...]
                      [--wrf NAME=WRF_DIR ...] [--exe NAME=PATH ...]
  manifest.py merge FILE.json ... --out FILE.json
  manifest.py --help

collect: what this machine and these builds are, as one JSON object under the key --side (default "host"):
- host: the CPU model, vCPUs, cores and sockets (lscpu), NUMA nodes, memory, kernel, OS (os-release), glibc
  (ldd --version), the compilers found (gcc, g++, gfortran, nvcc, mpich / Open MPI), the GPU (nvidia-smi: name,
  driver, memory, compute capability, power limit, clocks) and the energy counters (tools/bench/telemetry.py probe);
- each --cmake build: its CMakeCache.txt options (CMAKE_BUILD_TYPE, CMAKE_CXX_FLAGS*, CMAKE_CUDA_FLAGS*,
  CMAKE_CUDA_ARCHITECTURES, CMAKE_CXX_COMPILER, every CPPWRF_* option), the flavour they make (cpu-fp64, cpu-fp32,
  cuda-fp64, cuda-fp32 fast, cuda-fp32 identity, + mpi, portable libmvec on / off, native), `cppwrf --version` and the
  SHA-256 of BUILD_DIR/cppwrf;
- each --wrf build: configure.wrf's DMPARALLEL, OMPCPP, SFC, SCC, DM_FC, DM_CC, FCOPTIM, FCBASEOPTS_NO_G, PROMOTION,
  CFLAGS_LOCAL, ARCH_LOCAL lines and the SHA-256 of main/wrf.exe and main/real.exe;
- each --exe: its SHA-256;
- commit: --commit (a file holding it, or a git revision of the repository this script is in).
merge: one object from several (the keys of later files are added; the same side twice keeps the later one).
"""
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import telemetry  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WRF_KEYS = ("DMPARALLEL", "OMPCPP", "OMP", "SFC", "SCC", "DM_FC", "DM_CC", "FCOPTIM", "FCBASEOPTS_NO_G", "PROMOTION",
            "CFLAGS_LOCAL", "ARCH_LOCAL", "FCCOMPAT")


def sh(cmd, timeout=60):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip() if r.returncode == 0 else None
    except (subprocess.TimeoutExpired, OSError):
        return None


def sha256(path):
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def first_line(cmd):
    out = sh(cmd)
    return out.splitlines()[0] if out else None


def host():
    lscpu = {}
    for line in (sh("LC_ALL=C lscpu") or "").splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            lscpu[k.strip()] = v.strip()
    cores = sh("lscpu -p=CORE,SOCKET | grep -v '^#' | sort -u | wc -l")
    osr = {}
    if os.path.exists("/etc/os-release"):
        for line in open("/etc/os-release"):
            if "=" in line:
                k, v = line.rstrip("\n").split("=", 1)
                osr[k] = v.strip('"')
    mem = None
    if os.path.exists("/proc/meminfo"):
        m = re.search(r"MemTotal:\s+(\d+)", open("/proc/meminfo").read())
        mem = int(m.group(1)) // 1024 if m else None
    gpu = None
    if shutil.which("nvidia-smi"):
        q = ("name,driver_version,memory.total,compute_cap,power.limit,power.max_limit,clocks.max.sm,clocks.max.mem,"
             "pci.bus_id")
        gpu = dict(query=q, value=sh(f"nvidia-smi --query-gpu={q} --format=csv,noheader"),
                   smi=first_line("nvidia-smi | grep -m1 'CUDA Version'"))
    comp = {}
    for n, c in (("gcc", "gcc --version"), ("g++", "g++ --version"), ("gfortran", "gfortran --version"),
                 ("nvcc", "nvcc --version | tail -2 | head -1"), ("mpich", "mpichversion | head -1"),
                 ("mpiexec", "mpiexec --version 2>&1 | head -2 | tr '\\n' ' '"), ("cmake", "cmake --version"),
                 ("python", "python3 --version"), ("glibc", "ldd --version")):
        tool = c.split()[0]
        comp[n] = first_line(f"command -v {tool} > /dev/null && {c}")
    return dict(cpu=lscpu.get("Model name"), vcpus=os.cpu_count(), cores=int(cores) if cores and cores.isdigit() else None,
                sockets=lscpu.get("Socket(s)"), threads_per_core=lscpu.get("Thread(s) per core"),
                numa_nodes=lscpu.get("NUMA node(s)"), cpu_flags_avx512=("avx512f" in (lscpu.get("Flags") or "")),
                cpu_max_mhz=lscpu.get("CPU max MHz"), memory_mib=mem, kernel=platform.release(),
                os=osr.get("PRETTY_NAME"), compilers=comp, gpu=gpu, energy_counters=telemetry.rapl_probe(),
                hostname_hash=hashlib.sha256(platform.node().encode()).hexdigest()[:12])


def cmake_build(d):
    cache = {}
    p = os.path.join(d, "CMakeCache.txt")
    if os.path.exists(p):
        for line in open(p, errors="replace"):
            m = re.match(r"([A-Za-z0-9_]+):[A-Z]+=(.*)$", line.rstrip("\n"))
            if m and (m.group(1).startswith("CPPWRF_") or m.group(1) in (
                    "CMAKE_BUILD_TYPE", "CMAKE_CXX_FLAGS", "CMAKE_CXX_FLAGS_RELEASE", "CMAKE_CUDA_FLAGS",
                    "CMAKE_CUDA_FLAGS_RELEASE", "CMAKE_CUDA_ARCHITECTURES", "CMAKE_CXX_COMPILER", "CMAKE_CUDA_COMPILER")):
                cache[m.group(1)] = m.group(2)
    on = lambda k: cache.get(k, "").upper() in ("ON", "1", "TRUE", "YES")  # noqa: E731
    cuda, dp = on("CPPWRF_ENABLE_CUDA"), on("CPPWRF_DOUBLE")
    fl = f"{'cuda' if cuda else 'cpu'}-{'fp64' if dp else 'fp32'}"
    if cuda and not dp:
        fl += " identity" if on("CPPWRF_CUDA_FP32_IDENTITY") else " fast"
    if on("CPPWRF_MPI"):
        fl += " + mpi"
    if not cuda:
        fl += f", portable libmvec {'on' if on('CPPWRF_PORTABLE_LIBMVEC') else 'off'}"
    if "CPPWRF_NATIVE" in cache:
        fl += f", native {'on' if on('CPPWRF_NATIVE') else 'off'}"
    exe = os.path.join(d, "cppwrf")
    ver = sh(f"'{exe}' --version", 30) if os.path.isfile(exe) else None
    return dict(dir=d, flavour=fl, cmake=cache, version=ver, sha256=sha256(exe))


def wrf_build(d):
    flags = {}
    p = os.path.join(d, "configure.wrf")
    if os.path.exists(p):
        for line in open(p, errors="replace"):
            m = re.match(r"([A-Z_]+)\s*=\s*(.*)$", line.rstrip("\n"))
            if m and m.group(1) in WRF_KEYS and m.group(1) not in flags:
                flags[m.group(1)] = re.sub(r"\s+", " ", m.group(2)).strip()
    return dict(dir=d, configure_wrf=flags, wrf_exe_sha256=sha256(os.path.join(d, "main", "wrf.exe")),
                real_exe_sha256=sha256(os.path.join(d, "main", "real.exe")))


def commit(spec):
    if not spec:
        spec = "HEAD"
    if os.path.isfile(spec):
        return open(spec).read().strip()
    return sh(f"git -C '{REPO}' rev-parse --short '{spec}'") or spec


def pairs(a, flag):
    out = []
    for i, x in enumerate(a):
        if x == flag and i + 1 < len(a) and "=" in a[i + 1]:
            out.append(a[i + 1].split("=", 1))
    return out


def main():
    a = sys.argv[1:]
    if not a or a[0] in ("-h", "--help") or "--out" not in a:
        print(__doc__)
        sys.exit(0 if a and a[0] in ("-h", "--help") else 2)
    out = a[a.index("--out") + 1]
    if a[0] == "merge":
        m = {}
        for f in a[1:a.index("--out")]:
            if os.path.exists(f):
                m.update(json.load(open(f)))
    elif a[0] == "collect":
        side = a[a.index("--side") + 1] if "--side" in a else "host"
        m = {side: dict(host=host(),
                        commit=commit(a[a.index("--commit") + 1] if "--commit" in a else None),
                        cppwrf_builds={n: cmake_build(d) for n, d in pairs(a, "--cmake")},
                        wrf_builds={n: wrf_build(d) for n, d in pairs(a, "--wrf")},
                        executables={n: dict(path=p, sha256=sha256(p)) for n, p in pairs(a, "--exe")})}
    else:
        print(__doc__)
        sys.exit(2)
    with open(out, "w") as f:
        json.dump(m, f, indent=1)
    print(out)


if __name__ == "__main__":
    main()
