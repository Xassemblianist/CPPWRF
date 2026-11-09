# Installing CPPWRF

This guide goes from a clean Debian or Ubuntu system to an installed CPPWRF that passes its self-test. It covers three
ways to get CPPWRF: build it from source, unpack a release tarball, or use a container. [QUICKSTART.md](QUICKSTART.md)
continues with a real forecast. The Turkish version is [tr/KURULUM.md](tr/KURULUM.md).

## 1. Choose the builds

One source tree gives up to four builds, called *flavours*. They can be installed side by side, and
`cppwrf-wrf --mode` picks one of them at run time.

| Flavour | CMake flags | Use |
|---|---|---|
| `cpu-fp64` | `-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON` | Bit-identical to WRF v4.8.0 real*8. Runs on any x86-64 machine with OpenMP. |
| `gpu-fp64` | `-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON` | Bit-identical to WRF real*8 and to `cpu-fp64`. Runs on one NVIDIA GPU, about 3 to 5 times faster than WRF real*8. |
| `gpu-fp32` | `-DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF` | The fast mode. It is as close to WRF real*8 as WRF's own real*4 build is. |
| `cpu-fp32` | `-DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=OFF` | Not part of the verification suite. |

## 2. Packages

Debian 13 is the system the proofs of bit identity use (gcc 14.2, glibc 2.41). On it:

```bash
sudo apt install build-essential g++-14 cmake libnetcdf-dev python3 python3-netcdf4 python3-numpy git
```

Ubuntu 24.04 has the same packages (`g++-14` is in `universe`). Its CMake is 3.28, which is new enough: CPPWRF needs
CMake 3.24 or later.

What CPPWRF needs:

- **A C++20 compiler.** The proofs use g++ 14.2. g++ 11 to 14 are known to work. CPPWRF's arithmetic must match
  gfortran's, so use GNU compilers; clang is not tested.
- **CMake 3.24 or later**, and **OpenMP** (it comes with g++).
- **NetCDF-C with its headers** (`libnetcdf-dev`). NetCDF-4/HDF5 support is needed for compressed output.
- **Python 3 with `numpy` and `netCDF4`** for the tools. The namelist translator itself needs only the standard
  library.
- **For the GPU builds: the CUDA Toolkit 12 or later and an NVIDIA GPU.**
  - The whole CUDA build is known to compile with CUDA 12.4 and with CUDA 13.0.
  - The proofs ran on sm_80 (A100), sm_89 (L4) and sm_120 (RTX PRO 6000 Blackwell).
  - sm_120 needs CUDA 12.8 or later.
  - Install the toolkit from NVIDIA (<https://developer.nvidia.com/cuda-downloads>). nvcc must accept your host
    g++: CUDA 12.4 accepts g++ up to 13, and CUDA 12.8 and 13.x accept g++ 14. If your default g++ is newer than
    nvcc accepts, pass `-DCMAKE_CUDA_HOST_COMPILER=g++-13`.

## 3. Build

From the source tree (`git clone https://github.com/Xassemblianist/CPPWRF.git`):

```bash
# CPU, FP64
cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=OFF -DCPPWRF_DOUBLE=ON
cmake --build build-cpu -j "$(nproc)"

# GPU, FP64 and FP32 (the GPU of this machine; see CMAKE_CUDA_ARCHITECTURES below)
cmake -S . -B build-gpu-dp -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=ON
cmake --build build-gpu-dp -j "$(nproc)"
cmake -S . -B build-gpu -DCMAKE_BUILD_TYPE=Release -DCPPWRF_ENABLE_CUDA=ON -DCPPWRF_DOUBLE=OFF
cmake --build build-gpu -j "$(nproc)"

ctest --test-dir build-cpu -L unit --output-on-failure      # the unit tests (a few minutes)
```

A CUDA build compiles for a long time. On a machine with little memory, use fewer jobs (`-j 4`): nvcc takes a few
GB per job on the large physics sources.

### CMake options

| Option | Default | Meaning |
|---|---|---|
| `CPPWRF_ENABLE_CUDA` | `ON` when nvcc is found | The CUDA backend (one GPU) instead of OpenMP on the CPU. |
| `CPPWRF_DOUBLE` | `OFF` | `ON`: FP64. `OFF`: FP32. |
| `CMAKE_CUDA_ARCHITECTURES` | `native` | The GPUs to compile for. `native` means the GPU of the build machine. For binaries that run elsewhere, list them, e.g. `80;86;89;90;120`. |
| `CPPWRF_NATIVE` | `ON` | `-march=native`. Use `OFF` for binaries that must run on other CPUs. The values are the same either way. |
| `CPPWRF_PORTABLE_LIBMVEC` | `ON` | See "The glibc-math options" below. |
| `CPPWRF_CUDA_GLIBC_MATH` | `ON` | See "The glibc-math options" below. |
| `CMAKE_INSTALL_PREFIX` | `/usr/local` | Where `cmake --install` puts the files. |

### The glibc-math options and why they exist

WRF's results depend on the exact rounding of its math functions. Two options make CPPWRF repeat that rounding:

- **`CPPWRF_PORTABLE_LIBMVEC`.** gfortran calls glibc's vector math library, libmvec, in vectorised loops. libmvec's
  `log` and `log10` use an approximate reciprocal instruction whose last bit differs between Intel and AMD CPUs. The
  WRF references were made on Intel. `ON` computes these two functions as libmvec does on Intel, on every CPU, so
  CPPWRF matches the references on AMD too. `OFF` uses the machine's own libmvec. It then matches a WRF built and
  run on the same machine, but not the Intel-made references.
- **`CPPWRF_CUDA_GLIBC_MATH`** (CUDA FP64 builds only). `ON` runs ports of glibc's `exp`, `log`, `pow` and the other
  math functions on the GPU instead of CUDA's math library. That is what makes `gpu-fp64` bit-identical to WRF.
  It costs 0 to 7 % of the FP64 time. `OFF` uses CUDA's math library, which agrees with WRF to roundoff only.

The CMake files also refuse flags that change results: `-ffast-math`, `-Ofast`, `-ffp-contract=fast`, and on the GPU
`--use_fast_math`, `-ftz=true`, `-prec-div=false` and `-prec-sqrt=false`. They set `-ffp-contract=off` and nvcc's
`--fmad=false` (FP64) themselves. docs/USER_GUIDE.md ("Required Flags and Determinism") explains each one.

## 4. Install

```bash
cmake --install build-cpu --prefix "$HOME/cppwrf"       # and build-gpu-dp, build-gpu: one prefix holds them all
export PATH="$HOME/cppwrf/bin:$PATH"
```

The layout is:

| Path | Contents |
|---|---|
| `bin/cppwrf-<flavour>` | The model of each installed flavour. `bin/cppwrf` links to the last one installed. |
| `bin/cppwrf_real` | real.exe |
| `bin/cppwrf-wrf`, `bin/cppwrf-realexe`, `bin/cppwrf-check`, `bin/cppwrf-compare`, `bin/cppwrf-bench`, `bin/cppwrf-fit` | The user tools |
| `share/cppwrf/data` | The RRTMG tables and the WRF I/O schema that the binaries read |
| `share/cppwrf/tools`, `share/cppwrf/python`, `share/cppwrf/verify` | The namelist translator, the `cppwrf` Python package, the comparison code |
| `share/cppwrf/configs`, `share/cppwrf/examples` | Example configurations, the quick start and the self-test |
| `share/doc/cppwrf` | The documentation and the licences |

The installed binaries find their data from their own location (`bin/../share/cppwrf/data`), so the source tree can
be deleted. `CPPWRF_DATA_DIR` overrides the location. A build tree that is not installed reads `data/` from the
source tree it was built from.

The Python package can also be installed with pip (`pip install ./python`, see docs/PYTHON.md). Without pip, set
`PYTHONPATH=$HOME/cppwrf/share/cppwrf/python`.

WRF's physics tables (`LANDUSE.TBL`, `VEGPARM.TBL`, `RRTM_DATA`, ...) are not part of CPPWRF. They come from the
`run/` directory of WRF v4.8.0's source. WRF itself does not need to be built.

## 5. Check the installation

```bash
$HOME/cppwrf/share/cppwrf/examples/selftest/run.sh cpu-fp64     # and gpu-fp64, gpu-fp32 for the GPU builds
```

The self-test runs a tiny em_quarter_ss case (10 x 10 x 10 points, 5 steps) through `cppwrf-wrf`. It then hashes
the values of the last frame (SHA-256 over U, V, W, PH, T, MU, P, QVAPOR, QCLOUD and QRAIN as float64). The FP64
flavours must print exactly:

```
selftest cpu-fp64: 1c771835584e049d54934c255fb2a6db152c615e3601bb6d52d25682fce9b993 (...)
PASS: identical to the reference values (expected_fp64.txt)
```

`gpu-fp64` must print the same hash, because it is bit-identical to `cpu-fp64`. `gpu-fp32` has no reference hash.
The test checks that its run succeeds with finite values. The hash covers the values only, not the bytes of the
NetCDF file, so it does not depend on the NetCDF or HDF5 version.

`cppwrf-bench` (docs/QUICKSTART.md) then measures the speed of this machine.

## 6. Release tarballs

A release has one tarball per flavour, for example `cppwrf-1.0.0-cpu-fp64-linux-x86_64.tar.gz` and
`cppwrf-1.0.0-gpu-fp32-sm80-86-89-90-120-linux-x86_64.tar.gz`. Unpack one anywhere and use its `bin/`; it has the
layout of section 4. The binaries are built portably (`CPPWRF_NATIVE=OFF`) on Debian 13 and need:

- **glibc 2.38 or later** and the **libstdc++ of GCC 13.2 or later** (`GLIBCXX_3.4.32`). Debian 13 and Ubuntu 24.04
  have both.
- **Debian 13's NetCDF and HDF5 libraries** (`libnetcdf.so.22`, `libhdf5_serial.so.310`: `apt install libnetcdf22`)
  and `libgomp1`. Other distributions ship other NetCDF versions: build from source there, or use a container.
- **For the GPU tarballs:**
  - an NVIDIA driver for the CUDA version of the build. The tarball's notes name the version; CUDA 12.8 needs a
    driver of 570 or later, and CUDA 13.0 needs 580 or later;
  - the CUDA runtime library `libcudart.so.<major>` (the `cuda-cudart` package of NVIDIA's repository, or the
    toolkit);
  - a GPU among the architectures in the file name.

`cmake --build build-cpu --target package` makes such a tarball from a build. `tools/release/make_release.sh VERSION`
builds every flavour from the release tag, with checksums.

## 7. Containers

The recipes are in `packaging/`:

```bash
docker build -f packaging/docker/Dockerfile.cpu -t cppwrf:cpu .        # Debian 13, cpu-fp64
docker build -f packaging/docker/Dockerfile.cuda -t cppwrf:cuda .      # CUDA 12.8 on Ubuntu 24.04: gpu-fp32, gpu-fp64, cpu-fp64
docker run --rm --gpus all -v "$PWD/case:/work" cppwrf:cuda cppwrf-wrf --mode gpu-fp32

apptainer build cppwrf-cuda.sif packaging/apptainer/cppwrf-cuda.def
cd case && apptainer run --nv ../cppwrf-cuda.sif --mode gpu-fp64
```

- The CUDA images build both GPU precisions and the CPU build. `CUDA_ARCHS` (a build argument) chooses the
  architectures.
- The CUDA image is Ubuntu 24.04 (glibc 2.39). The proofs of bit identity ran on Debian 13 (glibc 2.41), and the host
  code calls glibc's libm. In that image, check `gpu-fp64` against `cpu-fp64` with `cppwrf-compare` before relying
  on bit identity with WRF.
- Pin the base images by digest for reproducible images.

A Spack recipe is in `packaging/spack` (`spack repo add packaging/spack; spack install cppwrf +cuda cuda_arch=89`).

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Could NOT find netCDF` or `netcdf.h: No such file` | The NetCDF headers are missing: `apt install libnetcdf-dev`. For a NetCDF in another prefix, pass `-DCMAKE_PREFIX_PATH=/that/prefix`. |
| `CMAKE_CXX_FLAGS: '-ffast-math' changes the floating-point results` | Remove the flag (also check `CXXFLAGS` in the environment). CPPWRF refuses flags that change results. |
| `nvcc fatal: Unsupported gpu architecture 'compute_120'` | Your CUDA is older than 12.8: drop 120 from `CMAKE_CUDA_ARCHITECTURES`. |
| `unsupported GNU version! gcc versions later than 13 are not supported` | nvcc does not accept the default g++: pass `-DCMAKE_CUDA_HOST_COMPILER=g++-13` (or install a newer CUDA). |
| `CMAKE_CUDA_ARCHITECTURES ... native ... no GPU detected` | You are building on a machine without a GPU: list the architectures, e.g. `-DCMAKE_CUDA_ARCHITECTURES=89`. |
| nvcc killed, or `virtual memory exhausted` | Fewer parallel jobs: `cmake --build build-gpu -j 2`. |
| `error while loading shared libraries: libnetcdf.so.22` | A release tarball on a system with another NetCDF: build from source, or use a container. |
| `error while loading shared libraries: libcudart.so.12` | Install the CUDA runtime (`cuda-cudart-12-x`) or add the toolkit's `lib64` to `LD_LIBRARY_PATH`. |
| `CUDA driver version is insufficient for CUDA runtime version` | Update the NVIDIA driver, or use a build made with an older CUDA. |
| `cannot open .../wrf_schema/wrf-4.8.0.tsv` | The binary cannot find its data. Run the installed binary from `bin/`, or set `CPPWRF_DATA_DIR` to `share/cppwrf/data` (or to the source tree's `data`). |
| `radiation: cannot find rrtmg_lw_tables.bin` | The same: `CPPWRF_DATA_DIR`. |
| `a cppwrf binary for mode gpu-fp64 was not found` | That flavour is not installed in the prefix, or not on `PATH`. Install it, or name the binary with `--exe` or `CPPWRF_EXE`. |
| The self-test hash differs | A build with flags that change results, `CPPWRF_PORTABLE_LIBMVEC=OFF`, `CPPWRF_CUDA_GLIBC_MATH=OFF`, or a non-GNU compiler. Rebuild with the defaults and report it if it persists (docs/FAQ.md). |
