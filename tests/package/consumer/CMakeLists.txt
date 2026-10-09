# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# A minimal external consumer of the installed ReUseX CMake package. It is
# configured and built by tests/package/run_package_test.cmake against a fresh
# `cmake --install` prefix, so the exported package cannot silently rot. Write
# it exactly the way ruxd (or any other downstream project) would.
cmake_minimum_required(VERSION 3.17)
project(reusex_package_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ReUseX CONFIG REQUIRED)

# Guard against resolving some other ReUseX (a system install, the package
# registry) instead of the prefix under test.
if(DEFINED REUSEX_EXPECTED_PREFIX)
    file(REAL_PATH "${REUSEX_EXPECTED_PREFIX}" _expected)
    file(REAL_PATH "${ReUseX_DIR}" _found)
    string(FIND "${_found}" "${_expected}" _at)
    if(NOT _at EQUAL 0)
        message(FATAL_ERROR "ReUseX resolved to ${_found}, expected a package "
                            "under ${_expected}")
    endif()
endif()
message(STATUS "ReUseX ${ReUseX_VERSION} from ${ReUseX_DIR} "
               "(ML backends: ${ReUseX_ML_BACKENDS}; CUDA: ${ReUseX_WITH_CUDA})")

# Per-module targets: what a focused consumer links.
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE ReUseX::pipeline ReUseX::core)

# The umbrella: every module, so every $<LINK_ONLY:...> dependency of every
# static library (ML backends, GTSAM, VTK, OpenMVS, ...) has to resolve.
add_executable(consumer_umbrella main.cpp)
target_link_libraries(consumer_umbrella PRIVATE ReUseX::reusex)
