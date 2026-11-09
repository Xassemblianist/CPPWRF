# SPDX-License-Identifier: GPL-3.0-or-later
# cmake --install: a normal prefix layout (docs/INSTALL.md, "Installing")
#   bin/                    cppwrf-<flavour> (and the link cppwrf to it), cppwrf_real, the drop-in tools
#   share/cppwrf/data       the RRTMG tables and the WRF I/O schema the binaries read (core/data_dir.hpp finds them
#                           from bin/; CPPWRF_DATA_DIR overrides)
#   share/cppwrf/tools      the namelist translator; share/cppwrf/python the cppwrf package the tools import;
#                           share/cppwrf/verify the comparison code of cppwrf-compare
#   share/cppwrf/configs, share/cppwrf/examples
#   share/doc/cppwrf        README, CHANGELOG, docs/, the licences
# The flavour (cpu-fp64, cpu-fp32, gpu-fp64, gpu-fp32) names the model binary, so the builds of several flavours can
# be installed into one prefix (cppwrf-wrf --mode chooses among them); `cppwrf` links to the last one installed.
include(GNUInstallDirs)
if(CPPWRF_ENABLE_CUDA)
  set(_cppwrf_backend gpu)
else()
  set(_cppwrf_backend cpu)
endif()
if(CPPWRF_DOUBLE)
  set(CPPWRF_FLAVOUR ${_cppwrf_backend}-fp64)
else()
  set(CPPWRF_FLAVOUR ${_cppwrf_backend}-fp32)
endif()
set(CPPWRF_SHARE ${CMAKE_INSTALL_DATAROOTDIR}/cppwrf)
set(CPPWRF_DOC ${CMAKE_INSTALL_DATAROOTDIR}/doc/cppwrf)

if(TARGET cppwrf)
  install(PROGRAMS $<TARGET_FILE:cppwrf> DESTINATION ${CMAKE_INSTALL_BINDIR} RENAME cppwrf-${CPPWRF_FLAVOUR}
          COMPONENT runtime)
  install(CODE "file(CREATE_LINK cppwrf-${CPPWRF_FLAVOUR} \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/cppwrf\" SYMBOLIC)"
          COMPONENT runtime)
endif()
if(TARGET cppwrf_real)
  # as built (install(TARGETS) would rewrite its RPATH; the binaries link system libraries only)
  install(PROGRAMS $<TARGET_FILE:cppwrf_real> DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime)
endif()
foreach(t cppwrf-wrf cppwrf-realexe cppwrf-check cppwrf-compare cppwrf-bench cppwrf-fit cppwrf-post
          cppwrf-verify-obs)
  if(EXISTS ${CMAKE_SOURCE_DIR}/tools/${t})
    install(PROGRAMS tools/${t} DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime)
  endif()
endforeach()
install(DIRECTORY data/ DESTINATION ${CPPWRF_SHARE}/data COMPONENT runtime)
install(PROGRAMS tools/wrf2cppwrf.py DESTINATION ${CPPWRF_SHARE}/tools COMPONENT runtime)
install(FILES tools/wrf_max_domains.py tools/wrf_namelist_options.py DESTINATION ${CPPWRF_SHARE}/tools
        COMPONENT runtime)
install(DIRECTORY python/cppwrf DESTINATION ${CPPWRF_SHARE}/python COMPONENT runtime
        PATTERN __pycache__ EXCLUDE)
install(FILES tests/verify/exact.py tests/verify/compare_real.py DESTINATION ${CPPWRF_SHARE}/verify COMPONENT runtime)
install(DIRECTORY configs/ DESTINATION ${CPPWRF_SHARE}/configs COMPONENT runtime)
install(DIRECTORY examples/ DESTINATION ${CPPWRF_SHARE}/examples COMPONENT runtime USE_SOURCE_PERMISSIONS)
if(EXISTS ${CMAKE_SOURCE_DIR}/tools/completion/cppwrf.bash)
  # bash-completion loads a completion by the command's name: one copy per tool
  foreach(t cppwrf-wrf cppwrf-realexe cppwrf-check cppwrf-compare cppwrf-bench cppwrf-fit cppwrf-post
          cppwrf-verify-obs)
    install(FILES tools/completion/cppwrf.bash DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/bash-completion/completions
            RENAME ${t} COMPONENT runtime)
  endforeach()
endif()
install(FILES README.md CHANGELOG.md NOTICE.md CONTRIBUTING.md SECURITY.md CODE_OF_CONDUCT.md
        DESTINATION ${CPPWRF_DOC} COMPONENT runtime)
install(FILES LICENSE DESTINATION ${CPPWRF_DOC} RENAME LICENSE COMPONENT runtime)
install(DIRECTORY licenses/ docs/ DESTINATION ${CPPWRF_DOC} COMPONENT runtime)

# CPack: one .tar.gz per flavour, cppwrf-<version>-<flavour>[-sm<arch>-...]-linux-<cpu>.tar.gz (cmake --build B
# --target package, or cpack in B; tools/release/make_release.sh builds them all)
set(_cppwrf_arch "")
if(CPPWRF_ENABLE_CUDA)
  set(_archs "${CMAKE_CUDA_ARCHITECTURES}")
  if(_archs STREQUAL "native" AND CMAKE_CUDA_ARCHITECTURES_NATIVE)
    set(_archs "${CMAKE_CUDA_ARCHITECTURES_NATIVE}")
  endif()
  string(REGEX REPLACE "-(real|virtual)" "" _archs "${_archs}")
  list(REMOVE_DUPLICATES _archs)
  list(JOIN _archs "-" _archs)
  set(_cppwrf_arch "-sm${_archs}")
endif()
string(TOLOWER "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}" _cppwrf_sys)
set(CPACK_GENERATOR TGZ)
set(CPACK_PACKAGE_NAME cppwrf)
set(CPACK_PACKAGE_VENDOR "CPPWRF")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++20 / CUDA port of WRF-ARW 4.8.0 (${CPPWRF_FLAVOUR})")
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_SOURCE_DIR}/LICENSE)
set(CPACK_PACKAGE_FILE_NAME "cppwrf-${PROJECT_VERSION}-${CPPWRF_FLAVOUR}${_cppwrf_arch}-${_cppwrf_sys}")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_SOURCE_GENERATOR TGZ)
set(CPACK_SOURCE_PACKAGE_FILE_NAME "cppwrf-${PROJECT_VERSION}-source")
set(CPACK_SOURCE_IGNORE_FILES "/\\\\.git/" "/build[^/]*/" "/out/" "__pycache__" "\\\\.egg-info/")
include(CPack)
