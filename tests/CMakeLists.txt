# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Unit Tests with Catch2 v3
# ===============================================

message(STATUS "Configuring unit tests...")

# -----------------------------------------------
# Find dependencies
# -----------------------------------------------
find_package(Catch2 3 REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)

# Note: ENABLE_COVERAGE option is now set in main CMakeLists.txt
# Coverage flags are automatically inherited from parent scope

# -----------------------------------------------
# Collect test source files from unit/ directory
# -----------------------------------------------
#
# The unit tests are split across TWO executables, purely for run-time speed
# (#268). `catch_discover_tests` registers one ctest test per TEST_CASE, and
# each one pays a full process start. When every test case linked the umbrella
# `reusex` target the binary pulled in 505 shared objects — libtorch_cuda
# (1.8 GB), TensorRT, cuOpt, NCCL, the OpenCV CUDA modules — and the dynamic
# loader spent ~1.9 s per process before main() even ran. Multiplied by ~360
# test cases that was ~11 minutes of pure relocation work.
#
# Only tests/unit/vision needs the ML stack, tests/unit/ruxd needs ruxd_lib
# (which links `reusex` PRIVATE — for a static library that still propagates
# to the final executable link), and tests/unit/visualize needs the PCL/Qt
# viewer module. Those three directories get their own "heavy" binary;
# everything else links just the modules it actually uses and starts an order
# of magnitude faster.
#
# Test *names* are unchanged either way: ctest still sees one test per
# TEST_CASE, so `ctest -R <name>` keeps working exactly as before.
file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp")

set(HEAVY_TEST_SOURCES ${UNIT_TEST_SOURCES})
list(FILTER HEAVY_TEST_SOURCES INCLUDE REGEX "/unit/(vision|ruxd|visualize)/")

# reusex/vision/nms.hpp is a torch::Tensor API, and MLBackendConfig.cmake drops
# nms.cpp/.hpp from the library entirely when the LibTorch backend is off. Its
# test has to follow the same rule or a -DML_BACKENDS=ONNX build stops
# compiling.
if(NOT "LibTorch" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER HEAVY_TEST_SOURCES EXCLUDE REGEX "/unit/vision/test_nms\\.cpp$")
    message(STATUS "Excluding test_nms.cpp (LibTorch backend disabled)")
endif()

set(LIGHT_TEST_SOURCES ${UNIT_TEST_SOURCES})
list(FILTER LIGHT_TEST_SOURCES EXCLUDE REGEX "/unit/(vision|ruxd|visualize)/")

# -----------------------------------------------
# Create unit test executables
# -----------------------------------------------
add_executable(reusex_unit_tests ${LIGHT_TEST_SOURCES})

# Deliberately NOT the umbrella `reusex` target: linking the individual
# modules keeps reusex_vision (and with it libtorch/TensorRT) out of this
# binary. Adding a test that needs vision means moving it to
# tests/unit/vision/, not widening this list.
#
# rux_gui_lib (the `rux gui` server, #265) is deliberately in the LIGHT binary.
# It links reusex_pipeline rather than the `reusex` umbrella, so it pulls no
# vision/ML closure -- tests/unit/rux_gui costs nothing extra here, and keeping
# it out of the heavy binary preserves the fast start-up this split bought.
target_link_libraries(reusex_unit_tests
    PRIVATE
        reusex_core
        reusex_io
        reusex_utils
        reusex_geometry_common
        reusex_segmentation
        reusex_reconstruction
        reusex_slam
        reusex_pipeline
        rux_gui_lib
        Catch2::Catch2WithMain
        fmt::fmt
        OpenSSL::Crypto
        SQLite::SQLite3
)

target_include_directories(reusex_unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/reusex/include
    ${CMAKE_BINARY_DIR}/generated
)

# tests/unit/core/test_stage_contract.cpp asserts that docs/CONTRACTS.md still
# says what the stage contract table says (#246). Reading the checked-in
# markdown needs the source tree, which a test binary otherwise cannot locate.
target_compile_definitions(reusex_unit_tests
    PRIVATE REUSEX_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Vision + ruxd tests: these genuinely need the ML backends and the HTTP
# routing/auth logic, so they carry the full dependency closure.
if(HEAVY_TEST_SOURCES)
    add_executable(reusex_unit_tests_vision ${HEAVY_TEST_SOURCES})

    # ruxd_lib carries the ruxd HTTP routing/auth logic (everything but
    # main()), so tests/unit/ruxd can exercise it directly.
    target_link_libraries(reusex_unit_tests_vision
        PRIVATE
            reusex
            ruxd_lib
            Catch2::Catch2WithMain
            fmt::fmt
            OpenSSL::Crypto
            SQLite::SQLite3
    )

    target_include_directories(reusex_unit_tests_vision PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )

    # configure_ml_backends() links torch PRIVATE to reusex_vision, so neither
    # the torch headers nor its ABI flags reach a consumer of the umbrella
    # `reusex` target. tests/unit/vision/test_nms.cpp includes
    # reusex/vision/nms.hpp, which is torch::Tensor-typed, so it needs its own
    # link.
    #
    # find_package(Torch) has to be repeated here: Dependencies.cmake runs in
    # the libs/reusex directory scope, and imported targets are visible only in
    # the directory that created them (and below). Without this, `torch`
    # degrades to a bare -ltorch with no include directories and the test fails
    # to compile on <torch/torch.h>. Kept behind the same guard as the source
    # filter above.
    if("LibTorch" IN_LIST ENABLED_ML_BACKENDS)
        find_package(Torch REQUIRED)
        target_link_libraries(reusex_unit_tests_vision PRIVATE torch)
    endif()
endif()

# -----------------------------------------------
# Integration tests (future)
# -----------------------------------------------
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")

if(INTEGRATION_TEST_SOURCES)
    add_executable(reusex_integration_tests ${INTEGRATION_TEST_SOURCES})
    target_link_libraries(reusex_integration_tests
        PRIVATE
            reusex
            Catch2::Catch2WithMain
            fmt::fmt
            # cv::imwrite, for the opt-in render artifacts written when
            # REUSEX_TEST_RENDER_DIR is set (#294).
            opencv_imgcodecs
    )
    target_include_directories(reusex_integration_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )

    # test_cli_exit_codes.cpp asserts the process exit code of the real binary
    # (#299) — the one thing no in-process test can observe — so it needs the
    # path to `rux` and an explicit build-order dependency on it. `tests` is
    # added after `apps/rux`, so the target exists here.
    if(TARGET rux)
        target_compile_definitions(reusex_integration_tests PRIVATE
            REUSEX_RUX_EXECUTABLE="$<TARGET_FILE:rux>")
        add_dependencies(reusex_integration_tests rux)
    endif()
endif()

# -----------------------------------------------
# Benchmarks (Catch2 BENCHMARK; run via scripts/bench.sh, not ctest)
# -----------------------------------------------
file(GLOB_RECURSE BENCHMARK_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/*.cpp")

if(BENCHMARK_SOURCES)
    add_executable(reusex_benchmarks ${BENCHMARK_SOURCES})
    target_link_libraries(reusex_benchmarks
        PRIVATE
            reusex
            Catch2::Catch2WithMain
            fmt::fmt
    )
    target_include_directories(reusex_benchmarks PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )
endif()

# -----------------------------------------------
# Enable CTest and discover tests
# -----------------------------------------------
include(CTest)
include(Catch)

# Discover each test binary twice, splitting on the Catch2 `[gpu]` tag.
#
# There is exactly one GPU on a dev box, and the cuOpt / cuDSS tests fall over
# when several of them initialise the solver at the same time ("CUDSS call
# ended unsuccessfully ... cudssSetThreadingLayer" at `ctest -j48`). That is a
# device-contention problem, not the temp-path collision fixed in #262, and it
# is the one thing that stopped the suite scaling past -j8.
#
# RESOURCE_LOCK makes ctest run every [gpu]-tagged test one at a time while
# leaving the other ~350 free to run fully parallel. The locked set is small
# (~11 tests, ~30 s total), well under the critical path set by the slowest
# single test, so serializing them costs no wall time.
#
# Two calls on one target are safe: catch_discover_tests derives its generated
# file names from a hash of TEST_SPEC, so the two sets do not overwrite each
# other. Total discovered test count is unchanged.
function(reusex_discover_tests target)
    catch_discover_tests(${target} TEST_SPEC "~[gpu]")
    catch_discover_tests(${target} TEST_SPEC "[gpu]"
                         PROPERTIES RESOURCE_LOCK reusex_gpu)
endfunction()

reusex_discover_tests(reusex_unit_tests)

# -----------------------------------------------
# Contract documents (#265)
# -----------------------------------------------
# docs/gui/openapi.yaml is a deliverable other code is generated from, so a
# parse failure there is a broken build, not a docs nit -- PR #274 shipped it
# unparseable. The check is a script rather than a C++ test so pre-commit and
# CI can run it without building anything.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME gui_api_contract_parses
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/scripts/check-openapi.py)
else()
    message(STATUS "Python3 not found: skipping the GUI contract parse test")
endif()

if(HEAVY_TEST_SOURCES)
    reusex_discover_tests(reusex_unit_tests_vision)
endif()

if(INTEGRATION_TEST_SOURCES)
    catch_discover_tests(reusex_integration_tests)
endif()

message(STATUS "Unit tests configured successfully")
