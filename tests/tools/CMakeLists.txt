# SPDX-License-Identifier: GPL-3.0-or-later
# tools/wrf2cppwrf.py (WRF namelist -> CPPWRF configuration): ctest -L unit; the 2-step runs use this build's cppwrf
add_test(NAME wrf2cppwrf_translator COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(wrf2cppwrf_translator PROPERTIES LABELS unit SKIP_RETURN_CODE 77)
if(TARGET cppwrf)
  set_tests_properties(wrf2cppwrf_translator PROPERTIES ENVIRONMENT "CPPWRF_BIN=$<TARGET_FILE:cppwrf>")
endif()
# the drop-in wrf.exe / real.exe and the namelist checker (tools/cppwrf-wrf, cppwrf-realexe, cppwrf-check): file names,
# rsl logs, exit status on a tiny em_quarter_ss run in the build directory
add_test(NAME dropin_tools COMMAND python3 -m unittest discover -v -s tests/tools -p test_dropin.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set(_dropin_env "CPPWRF_TEST_DIR=${CMAKE_BINARY_DIR}/dropin_test;OMP_NUM_THREADS=2;OMP_WAIT_POLICY=passive")
if(TARGET cppwrf)
  list(APPEND _dropin_env "CPPWRF_BIN=$<TARGET_FILE:cppwrf>")
endif()
if(TARGET cppwrf_real)
  list(APPEND _dropin_env "CPPWRF_REAL_BIN=$<TARGET_FILE:cppwrf_real>")
endif()
set_tests_properties(dropin_tools PROPERTIES LABELS unit ENVIRONMENT "${_dropin_env}")
# verification tiers (data/option_tiers.txt): translator, cppwrf-check, the model's gate and wrfout attribute, cppwrf_real's
# gate, on the same tiny run with a dummy row ($CPPWRF_OPTION_TIERS_EXTRA)
add_test(NAME option_tiers COMMAND python3 -m unittest -v tests/tools/test_option_tiers.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set(_tiers_env "${_dropin_env}")
list(TRANSFORM _tiers_env REPLACE "dropin_test$" "option_tiers_test")
set_tests_properties(option_tiers PROPERTIES LABELS unit ENVIRONMENT "${_tiers_env}")
# the Python package (python/cppwrf): namelists, Run with a stand-in binary, wrfout files and diagnostics
add_test(NAME python_package COMMAND python3 -m unittest discover -s python/tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(python_package PROPERTIES LABELS unit ENVIRONMENT "CPPWRF_TEST_DIR=${CMAKE_BINARY_DIR}")
# docs/MIGRATING_FROM_WRF.md's option tables are the translator's (tools/gen_option_docs.py --check)
add_test(NAME docs_options_current COMMAND python3 tools/gen_option_docs.py --check WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(docs_options_current PROPERTIES LABELS unit)
# cppwrf-post (python/cppwrf/post): the port of wrf-python's diagnostics on fields with known answers; the GRIB2
# encoder, projections, configuration and whole runs on synthetic wrfout files (no network, no GPU)
add_test(NAME post_diagnostics COMMAND python3 -m unittest discover -s python/tests -p "test_post_diag.py"
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
add_test(NAME post_output COMMAND python3 -m unittest discover -s python/tests -p "test_post_output.py"
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(post_diagnostics post_output PROPERTIES LABELS unit ENVIRONMENT "CPPWRF_TEST_DIR=${CMAKE_BINARY_DIR}")
# cppwrf-verify-obs (python/cppwrf/verify_obs): ISD / METAR parsers on saved reports, scores, a whole verification of
# synthetic files (no network; CPPWRF_NETWORK_TESTS=1 adds a small ISD and IEM download)
add_test(NAME verify_obs COMMAND python3 -m unittest discover -s python/tests -p "test_verify_obs.py"
         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_tests_properties(verify_obs PROPERTIES LABELS unit ENVIRONMENT "CPPWRF_TEST_DIR=${CMAKE_BINARY_DIR}")
