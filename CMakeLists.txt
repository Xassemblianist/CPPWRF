# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.24)
project(CPPWRF VERSION 1.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(CPPWRF_DOUBLE "Use double precision (real = double)" OFF)
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  set(_cuda_default ON)
else()
  set(_cuda_default OFF)
endif()
option(CPPWRF_ENABLE_CUDA "Build the CUDA backend" ${_cuda_default})
option(CPPWRF_ENABLE_HIP "Build the HIP (AMD ROCm) backend" OFF)
if(CPPWRF_ENABLE_CUDA AND CPPWRF_ENABLE_HIP)
  message(FATAL_ERROR "CPPWRF_ENABLE_CUDA and CPPWRF_ENABLE_HIP are mutually exclusive; set -DCPPWRF_ENABLE_CUDA=OFF for a HIP build")
endif()

if(CPPWRF_ENABLE_CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES native)  # the GPU of the build machine; pass e.g. 80;90 for servers
  endif()
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 20)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)
  # kernel parameters above 4 KB (LaunchBatch / parallel_fused pack up to 8 kernels per launch, up to 32 KB): CUDA 12.1
  # and compute capability 7.0 or later
  if(CMAKE_CUDA_COMPILER_VERSION VERSION_LESS 12.1)
    message(FATAL_ERROR "CPPWRF's CUDA build needs CUDA 12.1 or later (kernel parameters above 4 KB); found ${CMAKE_CUDA_COMPILER_VERSION}")
  endif()
elseif(CPPWRF_ENABLE_HIP)
  if(NOT DEFINED CMAKE_HIP_ARCHITECTURES)
    set(CMAKE_HIP_ARCHITECTURES "gfx90a;gfx942;gfx1100")
  endif()
  enable_language(HIP)
  set(CMAKE_HIP_STANDARD 20)
  set(CMAKE_HIP_STANDARD_REQUIRED ON)
  find_package(hip REQUIRED)
else()
  find_package(OpenMP REQUIRED COMPONENTS CXX)
endif()

# Strict IEEE: no fast-math anywhere (determinism and honest stability tests).
# -ffp-contract=off in every build type, after the user's flags: no FMA contraction on the CPU, as WRF's
# gfortran build on x86-64 (no -march: no FMA instructions); with -march=native or skylake g++ would
# otherwise fuse a*b+c (its default is -ffp-contract=fast). CUDA builds: the same for nvcc's host code.
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-ffp-contract=off> $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-ffp-contract=off>)
# glibc libmvec's vector log / log10 take an approximate reciprocal (rcpps) whose result differs between Intel and
# AMD; ON: CPPWRF computes them as libmvec does on Intel (src/core/libmvec_x86.hpp), the same on every x86-64 CPU.
# OFF: libmvec's own (the machine's rcpps).
option(CPPWRF_PORTABLE_LIBMVEC "Vector log / log10 as glibc libmvec on Intel, on any x86-64 CPU" ON)
if(NOT CPPWRF_PORTABLE_LIBMVEC)
  add_compile_definitions(CPPWRF_LIBMVEC_HW_LOG)
endif()
option(CPPWRF_NATIVE "Tune for the build machine's CPU (-march=native); OFF for portable binaries" ON)
if(CPPWRF_NATIVE)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
else()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()
set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
# CUDA FP64: the arithmetic of WRF's real*8 build bit for bit. --fmad=false: no fused multiply-add contraction
# (gfortran on x86-64 without -march emits none; nvcc fuses a*b+c by default), every kernel; division and sqrt are
# correctly rounded (-prec-div / -prec-sqrt true, the default; checked below). CPPWRF_CUDA_GLIBC_MATH: glibc's
# libm and libmvec algorithms on the device and the host (src/core/glibc_math.hpp) instead of CUDA's math library.
option(CPPWRF_CUDA_GLIBC_MATH "CUDA FP64 build: glibc's math functions on the device (bit-identical to WRF / CPU)" ON)
if(CPPWRF_ENABLE_CUDA AND CPPWRF_DOUBLE)
  string(APPEND CMAKE_CUDA_FLAGS " --fmad=false")
  if(NOT CPPWRF_CUDA_GLIBC_MATH)
    add_compile_definitions(CPPWRF_CUDA_LIBM)
  endif()
endif()
# CUDA FP32 identity mode: the CPU FP32 build's results (WRF real*4's on the verified configurations) on the GPU, bit
# for bit, at a cost in speed. --fmad=false in every kernel, IEEE division / sqrt and gradual underflow stated
# explicitly, glibc's float and double math functions and libmvec's 4-lane float functions on the device
# (CPPWRF_FP32_IDENTITY: src/core/backend.hpp, glibc_mathf.hpp, libmvec_f4.hpp). ON (default): the CUDA FP32 build
# is WRF real*4's arithmetic; OFF: the fast CUDA FP32 build (FMA contraction, CUDA's math library;
# docs/FP32_ACCURACY.md). The option has no effect on FP64 and CPU builds.
option(CPPWRF_CUDA_FP32_IDENTITY "CUDA FP32 build: bit-identical to the CPU FP32 build (strict arithmetic, glibc's math on the device); OFF: the fast FP32 build" ON)
if(CPPWRF_CUDA_FP32_IDENTITY AND CPPWRF_ENABLE_CUDA AND NOT CPPWRF_DOUBLE)
  if(NOT CPPWRF_CUDA_GLIBC_MATH)
    message(FATAL_ERROR "CPPWRF_CUDA_FP32_IDENTITY needs CPPWRF_CUDA_GLIBC_MATH=ON (-DCPPWRF_CUDA_FP32_IDENTITY=OFF: the fast FP32 build)")
  endif()
  string(APPEND CMAKE_CUDA_FLAGS " --fmad=false --ftz=false --prec-div=true --prec-sqrt=true")
  add_compile_definitions(CPPWRF_FP32_IDENTITY)
endif()
# flags that change results: reassociation, reciprocals, flush-to-zero, approximate division / sqrt, x87 excess
# precision (WRF's arithmetic is defined for SSE2 x86-64 and aarch64: x87 / -m32 builds were never supported), and
# in the strict CUDA builds (FP64, FP32 identity) FMA contraction. Checked on the flags of every configuration the
# build can compile (a multi-configuration generator's types, else CMAKE_BUILD_TYPE): the flags of the configuration
# follow CMAKE_<LANG>_FLAGS on the command line and win.
set(_cppwrf_configs ${CMAKE_CONFIGURATION_TYPES})
if(NOT _cppwrf_configs)
  set(_cppwrf_configs "${CMAKE_BUILD_TYPE}")
endif()
if(NOT _cppwrf_configs)
  set(_cppwrf_configs NONE)  # no build type: CMAKE_<LANG>_FLAGS alone
endif()
set(_cppwrf_x87 "-mfpmath=[^ ]*387|-m32|-mno-sse2?( |$)|-mx32|-fexcess-precision=fast")
if(CPPWRF_ENABLE_CUDA AND (CPPWRF_DOUBLE OR CPPWRF_CUDA_FP32_IDENTITY))
  set(_cppwrf_cuda_fma "|fmad[= ]*true")
else()
  set(_cppwrf_cuda_fma "")
endif()
foreach(_cppwrf_cfg IN LISTS _cppwrf_configs)
  string(TOUPPER "${_cppwrf_cfg}" _cppwrf_bt)
  if("${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_cppwrf_bt}}" MATCHES
     "(-Ofast|-f(fast-math|unsafe-math-optimizations|associative-math|reciprocal-math|finite-math-only)|-ffp-contract=(fast|on)|${_cppwrf_x87})")
    message(FATAL_ERROR "CMAKE_CXX_FLAGS: '${CMAKE_MATCH_1}' changes the floating-point results (CPPWRF needs strict IEEE)")
  endif()
  if("${CMAKE_CUDA_FLAGS} ${CMAKE_CUDA_FLAGS_${_cppwrf_bt}}" MATCHES
     "(use_fast_math|ftz=true|prec-div=false|prec-sqrt=false|-Xcompiler=?[^ ]*(-Ofast|-ffast-math|${_cppwrf_x87})${_cppwrf_cuda_fma})")
    message(FATAL_ERROR "CMAKE_CUDA_FLAGS: '${CMAKE_MATCH_1}' changes the floating-point results (CPPWRF needs strict IEEE)")
  endif()
endforeach()
if(CPPWRF_ENABLE_HIP)
  set(CMAKE_HIP_FLAGS_RELEASE "-O3")
  # Strict IEEE on both host and device passes: no FMA contraction, IEEE subnormals (no flush-to-zero)
  add_compile_options($<$<COMPILE_LANGUAGE:HIP>:-ffp-contract=off>
                      $<$<COMPILE_LANGUAGE:HIP>:-fno-gpu-flush-denormals-to-zero>)
  option(CPPWRF_HIP_GLIBC_MATH "HIP FP64 build: glibc's math functions on the device (bit-identical to WRF / CPU)" ON)
  if(CPPWRF_DOUBLE AND NOT CPPWRF_HIP_GLIBC_MATH)
    add_compile_definitions(CPPWRF_HIP_LIBM)
  endif()
  # ON (default): the HIP FP32 build is WRF real*4's arithmetic, as the CUDA FP32 build; OFF: the fast HIP FP32 build
  option(CPPWRF_HIP_FP32_IDENTITY "HIP FP32 build: bit-identical to the CPU FP32 build (strict arithmetic, glibc's math on the device); OFF: the fast FP32 build" ON)
  if(CPPWRF_HIP_FP32_IDENTITY AND NOT CPPWRF_DOUBLE)
    if(NOT CPPWRF_HIP_GLIBC_MATH)
      message(FATAL_ERROR "CPPWRF_HIP_FP32_IDENTITY needs CPPWRF_HIP_GLIBC_MATH=ON (-DCPPWRF_HIP_FP32_IDENTITY=OFF: the fast FP32 build)")
    endif()
    add_compile_definitions(CPPWRF_FP32_IDENTITY)
  endif()
  foreach(_cppwrf_cfg IN LISTS _cppwrf_configs)
    string(TOUPPER "${_cppwrf_cfg}" _cppwrf_bt)
    if("${CMAKE_HIP_FLAGS} ${CMAKE_HIP_FLAGS_${_cppwrf_bt}}" MATCHES
       "(-Ofast|-f(fast-math|unsafe-math-optimizations|associative-math|reciprocal-math|finite-math-only|gpu-flush-denormals-to-zero|gpu-approx-transcendentals)|-ffp-contract=(fast|on)|${_cppwrf_x87})")
      message(FATAL_ERROR "CMAKE_HIP_FLAGS: '${CMAKE_MATCH_1}' changes the floating-point results (CPPWRF needs strict IEEE)")
    endif()
  endforeach()
endif()

enable_testing()
add_subdirectory(src/io)

file(GLOB CPPWRF_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/src/core/*.cpp
  ${CMAKE_SOURCE_DIR}/src/dyn/*.cpp
  ${CMAKE_SOURCE_DIR}/src/cases/*.cpp)
# glibc 2.41's float functions (src/core/glibc_mathf_host.cpp) belong to the FP32 builds on x86-64 only (below): the
# FP64 builds call the host's libm as before
if(CPPWRF_DOUBLE OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  list(REMOVE_ITEM CPPWRF_SOURCES ${CMAKE_SOURCE_DIR}/src/core/glibc_mathf_host.cpp)
endif()
# Column physics used by the dycore (P1/P2 suite: sfclayrev + Noah + YSU + KF and its driver glue)
list(APPEND CPPWRF_SOURCES
  ${CMAKE_SOURCE_DIR}/src/phys/driver/suite.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/surface_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/pbl_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/cumulus_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/fdda_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/phy_prep.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/tendency_coupling.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/landuse_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/radiation.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw_batch.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw_batch.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_gases.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_ozone.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/batch_memory.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/sf_sfclayrev/sfclayrev.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/sf_myjsfc/myjsfc.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/sf_qnsesfc/qnsesfc.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/bl_ysu/ysu.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/bl_myj/myjpbl.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/bl_qnse/qnsepbl.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/bl_qnse/mfshconv.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/lsm_noah/noah.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/lsm_noah/noah_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/sf_urban/urban_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/lsm_noahmp/noahmp.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/lsm_noahmp/noahmp_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/lsm_ruc/ruc_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/cu_kf/kf.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/cu_kf/kf_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/cu_mskf/mskf.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_thompson/thompson.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_thompson/thompson_tables.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_morrison/morrison.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_p3/p3.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_nssl/nssl.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_nssl/nssl_scheme.cpp)
set_source_files_properties(
  ${CMAKE_SOURCE_DIR}/src/phys/mp_thompson/thompson.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_morrison/morrison.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_p3/p3.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/mp_nssl/nssl_scheme.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/suite.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/surface_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/pbl_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/cumulus_driver.cpp
  ${CMAKE_SOURCE_DIR}/src/phys/driver/fdda_driver.cpp
  PROPERTIES COMPILE_OPTIONS "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>")

add_library(cppwrf_core STATIC ${CPPWRF_SOURCES})
# MPI ranks as the workers of a decomposed run (docs/MPI.md): mpirun -n N cppwrf ... Off: no MPI is needed and
# the build is the one without this option.
option(CPPWRF_MPI "Domain decomposition over MPI ranks (mpirun -n N cppwrf)" OFF)
if(CPPWRF_MPI)
  find_package(MPI REQUIRED COMPONENTS CXX)
  target_compile_definitions(cppwrf_core PUBLIC CPPWRF_MPI MPICH_SKIP_MPICXX OMPI_SKIP_MPICXX)  # the C interface only
  # the headers and the libraries only: the compile and link options of the MPI wrapper are the distribution's
  # (Ubuntu's MPICH: -flto=auto, which nvcc's host compile of a CUDA source cannot take)
  target_include_directories(cppwrf_core SYSTEM PUBLIC ${MPI_CXX_INCLUDE_DIRS})
  target_link_libraries(cppwrf_core PUBLIC ${MPI_CXX_LIBRARIES})
endif()
# tables shipped with the sources (data/rrtmg)
target_compile_definitions(cppwrf_core PRIVATE CPPWRF_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
target_include_directories(cppwrf_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
# the configuration keys CPPWRF reads (every "section.key" literal of src/; src/cases/config_keys.cpp): a key of a
# configuration that is not among them is an error. Regenerated when a source changes.
file(GLOB_RECURSE CPPWRF_KEY_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.hpp
     ${CMAKE_SOURCE_DIR}/src/*.cu)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/generated/config_keys.inc
  COMMAND ${CMAKE_COMMAND} -DROOT=${CMAKE_SOURCE_DIR}/src -DOUT=${CMAKE_BINARY_DIR}/generated/config_keys.inc
          -P ${CMAKE_SOURCE_DIR}/src/cases/config_keys.cmake
  DEPENDS ${CPPWRF_KEY_SOURCES} ${CMAKE_SOURCE_DIR}/src/cases/config_keys.cmake VERBATIM)
target_sources(cppwrf_core PRIVATE ${CMAKE_BINARY_DIR}/generated/config_keys.inc)
target_include_directories(cppwrf_core PRIVATE ${CMAKE_BINARY_DIR}/generated)
# the commit of the sources (CPPWRF's own restart files record it): git, or the hash that git archive writes into
# src/cases/commit.txt (export-subst in .gitattributes) for a source tarball
set(CPPWRF_COMMIT "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} -C ${CMAKE_SOURCE_DIR} rev-parse --show-toplevel HEAD
                  OUTPUT_VARIABLE _git_out RESULT_VARIABLE _git_rc ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_git_rc EQUAL 0)
    string(REPLACE "\n" ";" _git_out "${_git_out}")
    list(GET _git_out 0 _git_top)
    get_filename_component(_git_top "${_git_top}" REALPATH)
    get_filename_component(_src_top "${CMAKE_SOURCE_DIR}" REALPATH)
    if(_git_top STREQUAL _src_top)
      list(GET _git_out 1 CPPWRF_COMMIT)
    endif()
  endif()
endif()
if(CPPWRF_COMMIT STREQUAL "unknown")
  file(STRINGS ${CMAKE_SOURCE_DIR}/src/cases/commit.txt _commit LIMIT_COUNT 1)
  if(_commit MATCHES "^[0-9a-f]+$")
    set(CPPWRF_COMMIT ${_commit})
  endif()
endif()
message(STATUS "CPPWRF commit: ${CPPWRF_COMMIT}")
set_property(SOURCE ${CMAKE_SOURCE_DIR}/src/cases/domain_runner.cpp APPEND PROPERTY COMPILE_DEFINITIONS
             CPPWRF_COMMIT="${CPPWRF_COMMIT}")
target_link_libraries(cppwrf_core PUBLIC cppwrf_io)
# glibc libmvec: Grell-Freitas-Li uses the 2-lane vector pow where WRF's gfortran build vectorises (cu_gfl)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  target_link_libraries(cppwrf_core PUBLIC mvec)
endif()
if(EXISTS ${CMAKE_SOURCE_DIR}/src/dyn/small_step.cpp)
  target_compile_definitions(cppwrf_core PUBLIC CPPWRF_HAVE_SMALL_STEP)
endif()
if(CPPWRF_DOUBLE)
  target_compile_definitions(cppwrf_core PUBLIC CPPWRF_DOUBLE)
endif()

# NetCDF-C library
find_package(netCDF CONFIG QUIET)
if(NOT TARGET netCDF::netcdf)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(NETCDF netcdf)
  endif()
endif()
find_path(NETCDF_INCDIR netcdf.h HINTS /usr/include /usr/local/include)
if(TARGET netCDF::netcdf OR NETCDF_FOUND OR NETCDF_INCDIR)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/io_netcdf)
    add_subdirectory(src/io_netcdf)
    target_link_libraries(cppwrf_core PUBLIC cppwrf_io_netcdf)
    target_compile_definitions(cppwrf_core PUBLIC CPPWRF_HAVE_NETCDF)
    set(CPPWRF_NETCDF_OUTPUT ON)
  endif()
  if(EXISTS ${CMAKE_SOURCE_DIR}/tests/io_netcdf)
    add_subdirectory(tests/io_netcdf)
  endif()
  # C++ real.exe (host only)
  find_library(NETCDF_LIBRARY NAMES netcdf HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)
  set(NETCDF_INCLUDE_DIR ${NETCDF_INCDIR})
  add_subdirectory(src/wrf_io)
  target_link_libraries(cppwrf_core PUBLIC cppwrf_wrfio)
  add_subdirectory(src/init_real)
  # cppwrf_real against WRF real.exe output (skipped, exit 77, without ${CPPWRF_WRF_RUNS}/real: real_exe_data.tar of
  # tests/verify/ctest_data.txt, which the CI diff job fetches)
  add_test(NAME real_vs_wrf COMMAND bash ${CMAKE_SOURCE_DIR}/tests/init_real/run_real_test.sh ${CMAKE_BINARY_DIR}
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(real_vs_wrf PROPERTIES SKIP_RETURN_CODE 77)
  # WRF real.exe runs of the wrfref workflow (port_gem10_data.tar): lagrange_order 1, default_soiltype 14, met_em
  # files with VAR_SSO / FLAG_VAR_SSO, and the plain P1 met_em; port_gem13_data.tar: sea ice with SST, ISLAKE -1,
  # SPECHUMD (use_sh_qv), no GHT, top-down levels, sfcprs2, pass-through fields, soil water (skipped, exit 77, without them; the tar is in
  # tests/verify/ctest_data.txt, the data of the CI diff job that runs the ctest tests)
  add_test(NAME real_vs_wrf_runs COMMAND bash ${CMAKE_SOURCE_DIR}/tests/init_real/run_real_test.sh ${CMAKE_BINARY_DIR}
           runs:run_G10REAL_r8 runs:run_G10LAG1_r8 runs:run_G10DST_r8 runs:run_G10SSO_r8
           runs:run_G13ICE_r8 runs:run_G13ICEF_r8 runs:run_G13NOLAKE_r8 runs:run_G13SH_r8 runs:run_G13SHQ_r8
           runs:run_G13NOGHT_r8 runs:run_G13FLIP_r8 runs:run_G13S2P_r8 runs:run_G13PT_r8 runs:run_G13SW_r8
           runs:run_V111TAV_r8 runs:run_V111TAVT_r8 runs:run_V111TS2P_r8 runs:run_V111TSLAB_r8 runs:run_V111TNL_r8
           runs:run_V111PMS_r8 runs:run_V111PMT_r8 runs:run_V111NSH_r8
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(real_vs_wrf_runs PROPERTIES SKIP_RETURN_CODE 77)
  # wrfout/wrfrst files identical to WRF v4.8.0's (CPU FP64 builds; skipped, exit 77, without ${CPPWRF_WRF_RUNS})
  if(CPPWRF_DOUBLE AND NOT CPPWRF_ENABLE_CUDA AND NOT CPPWRF_ENABLE_HIP)
    add_test(NAME wrf_files_vs_wrf COMMAND bash ${CMAKE_SOURCE_DIR}/tests/verify/run_wrf_files_test.sh ${CMAKE_BINARY_DIR}
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_tests_properties(wrf_files_vs_wrf PROPERTIES SKIP_RETURN_CODE 77)
  endif()
endif()

# Single-source kernels: every source of the core is compiled by nvcc (CUDA) or amdclang++ (HIP).
function(cppwrf_device_sources)
  if(CPPWRF_ENABLE_CUDA)
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE CUDA)
  elseif(CPPWRF_ENABLE_HIP)
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE HIP)
  endif()
endfunction()

if(CPPWRF_ENABLE_CUDA)
  cppwrf_device_sources(${CPPWRF_SOURCES})
  # RRTMG host code (tables, constants, the column drivers of the checks) and RRTM LW (host columns with OpenMP):
  # C++; the device path is radiation.cpp + rrtmg_lw_batch.cpp + rrtmg_sw_batch.cpp
  set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_gases.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_ozone.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw_tables.cpp
    PROPERTIES LANGUAGE CXX)
  # RRTMG on the device: no FMA contraction (WRF's x86 arithmetic; table indices int(tblint*x + 0.5)
  # are discontinuous, so contracted products move them)
  set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw_batch.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw_batch.cpp
    PROPERTIES COMPILE_OPTIONS "--fmad=false")
  target_compile_definitions(cppwrf_core PUBLIC CPPWRF_USE_CUDA)
  target_compile_options(cppwrf_core PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr --extended-lambda>)
  target_link_libraries(cppwrf_core PUBLIC CUDA::cudart)
  # host OpenMP for the radiation columns (C++ sources only)
  find_package(OpenMP REQUIRED COMPONENTS CXX)
  target_link_libraries(cppwrf_core PUBLIC OpenMP::OpenMP_CXX)
elseif(CPPWRF_ENABLE_HIP)
  cppwrf_device_sources(${CPPWRF_SOURCES})
  # RRTMG host code and RRTM LW stay C++, as in CUDA builds; glibc_mathf_host.cpp defines host C math
  # symbols (expf, ...) and stays C++ so HIP device math overloads are untouched. All HIP sources already
  # receive -ffp-contract=off globally above.
  set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_lw/rrtmg_lw_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_sw/rrtmg_sw_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_gases.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtmg_common/cam_ozone.cpp
    ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw.cpp ${CMAKE_SOURCE_DIR}/src/phys/ra_rrtm_lw/rrtm_lw_tables.cpp
    ${CMAKE_SOURCE_DIR}/src/core/glibc_mathf_host.cpp
    PROPERTIES LANGUAGE CXX)
  target_compile_definitions(cppwrf_core PUBLIC CPPWRF_USE_HIP)
  # With native enable_language(HIP), CMake handles device compilation/linking via CMAKE_HIP_ARCHITECTURES;
  # hip::host supplies amdhip64 and ROCm headers without injecting -x hip into the CXX files above.
  target_link_libraries(cppwrf_core PUBLIC hip::host)
  # host OpenMP for the radiation columns (C++ sources only)
  find_package(OpenMP REQUIRED COMPONENTS CXX)
  target_link_libraries(cppwrf_core PUBLIC OpenMP::OpenMP_CXX)
else()
  target_link_libraries(cppwrf_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/src/main.cpp)
  add_executable(cppwrf src/main.cpp)
  cppwrf_device_sources(src/main.cpp)
  target_link_libraries(cppwrf PRIVATE cppwrf_core)
  target_compile_definitions(cppwrf PRIVATE CPPWRF_VERSION="${PROJECT_VERSION}" CPPWRF_COMMIT="${CPPWRF_COMMIT}")
endif()
# FP32 builds on x86-64: glibc 2.41's float functions (expf, logf, powf, ...) defined in the executables themselves
# (src/core/glibc_mathf_host.cpp), in place of the host's libm, whose float functions differ between glibc versions;
# the ctest fp32_host_libm checks that cppwrf imports none of them
if(NOT CPPWRF_DOUBLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(CPPWRF_GLIBCF_HOST_FUNCS expf logf powf sinf cosf sincosf tanf atanf atan2f asinf acosf sinhf tanhf log10f erff
      tgammaf)
  if(TARGET cppwrf)
    target_sources(cppwrf PRIVATE ${CMAKE_SOURCE_DIR}/src/core/glibc_mathf_host.cpp)
    string(REPLACE ";" "|" _gfh "${CPPWRF_GLIBCF_HOST_FUNCS}")
    # also no libmvec vector variant of them (_ZGVbN4v_expf ...: a loop g++ vectorised with the host's libmvec)
    add_test(NAME fp32_host_libm
             COMMAND bash -c "! nm -D --undefined-only \"$1\" | grep -E \" (_ZGV[A-Za-z0-9]*_)?(${_gfh})(@|$)\"" _ $<TARGET_FILE:cppwrf>)
    set_tests_properties(fp32_host_libm PROPERTIES LABELS unit)
  endif()
endif()
# cppwrf_real (both precisions at run time, --precision float in any build): glibc 2.41's float functions in every
# x86-64 build, so that its single-precision files do not depend on the flavour installed last into a prefix
if(TARGET cppwrf_real AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  target_sources(cppwrf_real PRIVATE ${CMAKE_SOURCE_DIR}/src/core/glibc_mathf_host.cpp)
  target_include_directories(cppwrf_real PRIVATE ${CMAKE_SOURCE_DIR}/src)
  # cppwrf_real imports none of them either, nor their libmvec variants (its vectorised logf / expf: libmvec_f4.hpp)
  set(_gfr expf logf powf sinf cosf sincosf tanf atanf atan2f asinf acosf sinhf tanhf log10f erff tgammaf)
  string(REPLACE ";" "|" _gfr "${_gfr}")
  add_test(NAME fp32_host_libm_real
           COMMAND bash -c "! nm -D --undefined-only \"$1\" | grep -E \" (_ZGV[A-Za-z0-9]*_)?(${_gfr})(@|$)\"" _ $<TARGET_FILE:cppwrf_real>)
  set_tests_properties(fp32_host_libm_real PROPERTIES LABELS unit)
endif()

# Unit tests (tests/unit/*.cpp; tests/unit/io belongs to cppwrf_io)
file(GLOB CPPWRF_UNIT_TESTS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/unit/*.cpp)
foreach(src ${CPPWRF_UNIT_TESTS})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  cppwrf_device_sources(${src})
  target_link_libraries(${name} PRIVATE cppwrf_core ${CMAKE_DL_LIBS})  # dlopen (test_glibc_mathf; glibc < 2.34: libdl)
  add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  # OpenMP threads that spin while waiting stall the tests for hours on a loaded machine
  set_tests_properties(${name} PROPERTIES ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
endforeach()

# Namelist translator (tools/wrf2cppwrf.py; python3, no WRF data needed)
add_subdirectory(tests/tools)

# every launch of a kernel that raises a WRF fatal stop is followed by fatal_check() (tools/lint_fatal_check.py)
add_test(NAME lint_fatal_check COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/lint_fatal_check.py ${CMAKE_SOURCE_DIR}/src)
set_tests_properties(lint_fatal_check PROPERTIES LABELS unit)
# no decimal literal whose double-then-float rounding differs from gfortran's one rounding to real*4
# (tools/fp32_double_rounding.py)
add_test(NAME fp32_double_rounding COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/fp32_double_rounding.py -v
         ${CMAKE_SOURCE_DIR}/src)
set_tests_properties(fp32_double_rounding PROPERTIES LABELS unit)
# deliberate faults the model refuses: unknown configuration keys, nz above a column limit, a restart with other
# options; the non-finite check of every step (tests/robustness/run_test.sh, analytic cases)
if(TARGET cppwrf)
  add_test(NAME robustness COMMAND bash ${CMAKE_SOURCE_DIR}/tests/robustness/run_test.sh $<TARGET_FILE:cppwrf>
           ${CMAKE_BINARY_DIR}/robustness WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(robustness PROPERTIES LABELS unit ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # WRF's restart alarm with an interval that is not a multiple of the time step, also in a run resumed from a
  # restart file (tests/verify/restart_alarm_test.py; a tiny em_quarter_ss, skipped without WRF's input_sounding)
  add_test(NAME restart_alarm COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/restart_alarm_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/restart_alarm WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(restart_alarm PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # auxinput4_interval / gfdda_interval_m that are not multiples of the time step: refused as WRF stops
  # (tests/verify/input_interval_test.py; one step of SU1, skipped without ${CPPWRF_WRF_RUNS}/run_SU1_r8/steps)
  add_test(NAME input_interval COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/input_interval_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/input_interval WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(input_interval PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # CPPWRF's own restart file in another shape (a rank-2 HALO_ variable, a missing one, format version 6) is refused,
  # in one patch and in 2 x 2 patches (tests/verify/restart_guard_test.py; a tiny em_quarter_ss, skipped without
  # WRF's input_sounding)
  add_test(NAME restart_guard COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/restart_guard_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/restart_guard WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(restart_guard PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # runs resumed from CPPWRF's own restart file and from a wrfrst file with changed intervals
  # (tests/verify/restart_resume_test.py; P1, skipped without ${CPPWRF_WRF_RUNS}/run_P1_r8/steps)
  add_test(NAME restart_resume COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/restart_resume_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/restart_resume WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(restart_resume PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # Kessler resumed from a wrfrst file with SR /= 0: SR = 0 after the first step (solve_em.F:3852)
  # (tests/verify/kessler_sr_resume_test.py; P1, skipped without ${CPPWRF_WRF_RUNS}/run_P1_r8/steps)
  add_test(NAME kessler_sr_resume COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/kessler_sr_resume_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/kessler_sr_resume WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(kessler_sr_resume PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # an ensemble of 3 members (one host thread each, and one after the other; the output writer thread on and off):
  # every member's files byte-identical to its single run (tests/verify/ensemble_test.py; a tiny squall2d, no data)
  if(CPPWRF_NETCDF_OUTPUT)
    set(_ens_nc ON)
  else()
    set(_ens_nc OFF)
  endif()
  add_test(NAME ensemble COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/ensemble_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/ensemble --netcdf ${_ens_nc} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(ensemble PROPERTIES LABELS unit ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
  # members with Grell-Freitas (P8) and without cumulus, sequential and threads: convtrans_prep's averaging counter
  # is each member's own (ensemble_test.py --cumulus; skipped without ${CPPWRF_WRF_RUNS}/run_P8_r8/steps)
  add_test(NAME ensemble_cumulus COMMAND python3 ${CMAKE_SOURCE_DIR}/tests/verify/ensemble_test.py
           $<TARGET_FILE:cppwrf> ${CMAKE_BINARY_DIR}/ensemble_cumulus --netcdf ${_ens_nc} --cumulus
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  set_tests_properties(ensemble_cumulus PROPERTIES LABELS unit SKIP_RETURN_CODE 77
                       ENVIRONMENT "OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
endif()

# Differential tests against WRF v4.8.0's own compiled code (gfortran; slow): physics schemes, the
# physics driver glue, diffusion kernels, nest numerics. ctest -L diff runs them; they need WRF_DIR (a WRF v4.8.0
# -r8 build tree) and WRF_R8D8_DIR (-r8 -fdefault-double-8), some CPPWRF_WRF_RUNS (tests/phys/wrf_fortran.sh), and
# exit 77 (skipped) without them. tools/gcp/gcp_run.sh runs them on every full VM suite.
option(CPPWRF_DIFF_TESTS "Register the WRF-Fortran differential tests with ctest" OFF)
if(CPPWRF_DIFF_TESTS)
  find_package(OpenMP REQUIRED COMPONENTS CXX)  # the CPU reference drivers (also in CUDA builds)
  add_subdirectory(tests/phys)
  set(DIFF_WORK ${CMAKE_BINARY_DIR}/diff_work)
  foreach(t dyn/diffusion phys/suite nest)
    string(REPLACE "/" "_" tname "diff_${t}")
    add_test(NAME ${tname} COMMAND bash ${CMAKE_SOURCE_DIR}/tests/${t}/run_test.sh WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_tests_properties(${tname} PROPERTIES LABELS diff SKIP_RETURN_CODE 77  # 77: no WRF build / runs
                         ENVIRONMENT "DIFF_TEST_BUILD_DIR=${DIFF_WORK}")
  endforeach()
  # scalar advection of the last RK stage (adv_opt 1, 2): ScalarTransport against WRF's module_advect_em /
  # module_em, tolerance 0 (CPU FP64 builds)
  if(CPPWRF_DOUBLE AND NOT CPPWRF_ENABLE_CUDA AND NOT CPPWRF_ENABLE_HIP)
    add_executable(driver_advection tests/dyn/advection/driver_advection.cpp)
    target_link_libraries(driver_advection PRIVATE cppwrf_core)
    add_test(NAME diff_dyn_advection COMMAND bash ${CMAKE_SOURCE_DIR}/tests/dyn/advection/run_test.sh
             $<TARGET_FILE:driver_advection> WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_tests_properties(diff_dyn_advection PROPERTIES LABELS diff SKIP_RETURN_CODE 77
                         ENVIRONMENT "DIFF_TEST_BUILD_DIR=${DIFF_WORK}")
  endif()
  # column replays of the model's cumulus calls through WRF's objects (CPU FP64 cppwrf; need
  # CPPWRF_WRF_RUNS and WRF_R8D8_DIR)
  if(CPPWRF_DOUBLE AND NOT CPPWRF_ENABLE_CUDA AND NOT CPPWRF_ENABLE_HIP)
    foreach(t cu_gfl cu_ntiedtke)
      add_test(NAME diff_phys_${t} COMMAND bash ${CMAKE_SOURCE_DIR}/tests/phys/${t}/run_test.sh ${CMAKE_BINARY_DIR}
               WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
      set_tests_properties(diff_phys_${t} PROPERTIES LABELS diff SKIP_RETURN_CODE 77
                           ENVIRONMENT "CPPWRF_SCRATCH=${DIFF_WORK}")
    endforeach()
  endif()
endif()

# cmake --install and CPack (cmake/Install.cmake)
include(cmake/Install.cmake)
