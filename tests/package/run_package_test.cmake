# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Driver for the installed-package consumer test (cmake -P). Installs the
# ReUseX_Development component of an already-built tree into a fresh prefix,
# then configures, builds and runs tests/package/consumer against it.
#
# Inputs (-D): CONSUMER_SOURCE_DIR, WORK_DIR, GENERATOR, and either
#   REUSEX_BUILD_DIR   - build tree to `cmake --install` (the ctest), or
#   INSTALLED_PREFIX   - an existing install to test as-is, skipping the
#                        install step (the nix installCheckPhase, whose
#                        absolute CMAKE_INSTALL_*DIR ignore --prefix);
# optional: BUILD_CONFIG, CXX_COMPILER, OUTER_PREFIX_PATH.

foreach(_var CONSUMER_SOURCE_DIR WORK_DIR GENERATOR)
    if(NOT DEFINED ${_var})
        message(FATAL_ERROR "run_package_test.cmake: ${_var} is required")
    endif()
endforeach()
if(NOT BUILD_CONFIG)
    set(BUILD_CONFIG Release)
endif()

function(run_step name)
    message(STATUS "[package-test] ${name}")
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "[package-test] ${name} failed (exit ${_rc})")
    endif()
endfunction()

set(_consumer_build ${WORK_DIR}/build)
file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

if(INSTALLED_PREFIX)
    set(_prefix ${INSTALLED_PREFIX})
elseif(REUSEX_BUILD_DIR)
    set(_prefix ${WORK_DIR}/prefix)
    run_step("install ReUseX_Development into ${_prefix}"
        ${CMAKE_COMMAND} --install ${REUSEX_BUILD_DIR}
            --config ${BUILD_CONFIG}
            --component ReUseX_Development
            --prefix ${_prefix})
else()
    message(FATAL_ERROR "run_package_test.cmake: set REUSEX_BUILD_DIR or "
                        "INSTALLED_PREFIX")
endif()

file(GLOB _config "${_prefix}/lib*/cmake/ReUseX/ReUseXConfig.cmake")
if(NOT _config)
    message(FATAL_ERROR "[package-test] no lib*/cmake/ReUseX/ReUseXConfig.cmake "
                        "under ${_prefix}")
endif()
foreach(_hdr reusex/core/ProjectDB.hpp reusex/core/version.hpp
             reusex/pipeline/stages.hpp reusex/extern/pcl/planar_region_growing.hpp)
    if(NOT EXISTS ${_prefix}/include/${_hdr})
        message(FATAL_ERROR "[package-test] header not installed: include/${_hdr}")
    endif()
endforeach()

# The package must be found under the fresh prefix and nowhere else; the outer
# prefix path only supplies ReUseX's own dependencies (PCL, CGAL, ...).
set(_prefix_path "${_prefix}")
if(OUTER_PREFIX_PATH)
    list(APPEND _prefix_path ${OUTER_PREFIX_PATH})
endif()
string(REPLACE ";" "\;" _prefix_path_arg "${_prefix_path}")
set(_configure_args
    -S ${CONSUMER_SOURCE_DIR} -B ${_consumer_build} -G ${GENERATOR}
    -DCMAKE_BUILD_TYPE=${BUILD_CONFIG}
    "-DCMAKE_PREFIX_PATH=${_prefix_path_arg}"
    -DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON
    -DREUSEX_EXPECTED_PREFIX=${_prefix})
if(CXX_COMPILER)
    list(APPEND _configure_args -DCMAKE_CXX_COMPILER=${CXX_COMPILER})
endif()
run_step("configure consumer" ${CMAKE_COMMAND} ${_configure_args})
# Configure a second time: a CUDA package's legacy FindCUDA cache state used to
# make every consumer *reconfigure* fail ("Unknown CMake command
# find_cuda_helper_libs"); ReUseXConfig.cmake now guards against it.
run_step("reconfigure consumer" ${CMAKE_COMMAND} ${_configure_args})
run_step("build consumer"
    ${CMAKE_COMMAND} --build ${_consumer_build} --config ${BUILD_CONFIG})

# Locate the executables (single- or multi-config generator layouts).
foreach(_exe consumer consumer_umbrella)
    file(GLOB_RECURSE _bin LIST_DIRECTORIES false
         "${_consumer_build}/${_exe}" "${_consumer_build}/*/${_exe}")
    if(NOT _bin)
        message(FATAL_ERROR "[package-test] built ${_exe} not found")
    endif()
    list(GET _bin 0 _bin)
    run_step("run ${_exe}" ${_bin} ${WORK_DIR}/${_exe}.rux)
endforeach()

message(STATUS "[package-test] ReUseX package consumed successfully")
