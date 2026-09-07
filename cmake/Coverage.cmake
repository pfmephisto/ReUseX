# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Code coverage instrumentation (gcov/gcovr)
# ===============================================
# Enabled via -DENABLE_COVERAGE=ON. When on, adds --coverage (GCC/Clang:
# -fprofile-arcs -ftest-coverage at compile time, --coverage at link time) via
# add_compile_options()/add_link_options(). Because this module is included
# from the top-level CMakeLists.txt *before* the add_subdirectory() calls,
# the flags are inherited by every target configured afterwards in this
# directory scope and below: the reusex library (libs/reusex), the rux/ruxd
# apps, and the unit test binaries (tests/CMakeLists.txt). Dependencies
# consumed as prebuilt Nix packages (PCL, CGAL, libtorch, ...) are unaffected
# since they are never recompiled from source here.
#
# Coverage instrumentation disables most optimizations' ability to produce
# meaningful line/branch counts (inlining and dead-code elimination distort or
# hide coverage), so this is only meaningful for Debug builds. A Release (or
# RelWithDebInfo/MinSizeRel) build combined with ENABLE_COVERAGE=ON still
# works, but the report will be misleading, so we warn loudly rather than
# fail the configure step outright (some workflows intentionally probe this).
#
# Driven end-to-end by scripts/coverage.sh, which configures a dedicated
# build-coverage/ tree (kept separate from build/ so day-to-day Release/Debug
# builds are never instrumented), runs ctest, and renders an HTML + per-module
# report with gcovr.

option(ENABLE_COVERAGE "Enable code coverage instrumentation (gcc/clang --coverage)" OFF)

if(ENABLE_COVERAGE)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(WARNING
            "ENABLE_COVERAGE=ON with CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}'. "
            "Coverage instrumentation is only meaningful for Debug builds: "
            "Release-style optimizations inline/eliminate code, which "
            "distorts line and branch counts. Reconfigure with "
            "-DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON for a trustworthy "
            "report (this is what scripts/coverage.sh does).")
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "Code coverage enabled (--coverage, ${CMAKE_CXX_COMPILER_ID})")
        # CXX only - deliberately NOT applied to CUDA (see below).
        # -fprofile-update=atomic: rux/ruxd/reusex_unit_tests spawn dozens of
        # threads before main() even starts (TBB/OpenMP/AWS-SDK thread pools
        # warming up during static initialization); atomic counter updates
        # avoid a data race on the gcov counters at negligible runtime cost.
        add_compile_options(
            $<$<COMPILE_LANGUAGE:CXX>:--coverage>
            $<$<COMPILE_LANGUAGE:CXX>:-fprofile-update=atomic>
        )
        add_link_options(--coverage -fprofile-update=atomic)

        # Two cooperating fixes for a real, empirically-hit bug: with
        # WITH_CUDA=ON, nothing below ever produced a single .gcda for a
        # .cpp file (only for the handful of .cu translation units) even
        # though --coverage/-fprofile-arcs compiled fine and __gcov_init() was
        # observably called for every TU (checked with gdb).
        #
        # Root cause: CMake's CUDA/CUDAToolkit plumbing adds an explicit -L
        # for nvcc's pinned, CUDA-compatible host GCC (observed: 14.3.0, vs.
        # the project's main compiler, 15.2.0). That -L is searched before the
        # linker's *implicit* search of the main compiler's own lib directory,
        # so the bare "-lgcov" that --coverage adds under the hood resolved
        # against the *wrong* compiler's libgcov.a. GCC's gcov ABI
        # (GCOV_VERSION) is not stable across major versions (`gcov` flags the
        # skew at read time: "version 'B43*', prefer 'B52*'"), so every
        # gcc-15-compiled .cpp TU's registration was silently rejected inside
        # __gcov_init's version check - 0 coverage data for any .cpp file.
        # Fix 1 (below): pin libgcov.a to the exact file the *main* compiler
        # would resolve on its own, removing the -L ambiguity.
        #
        # Fix 2 (above, $<COMPILE_LANGUAGE:CXX> guard): even with libgcov.a
        # pinned, .cu files are still *compiled* by nvcc's older host GCC, so
        # their embedded gcov_info structs still carry the old-format
        # GCOV_VERSION and would now fail the (correct) version check instead.
        # Excluding CUDA from instrumentation avoids that - and costs nothing
        # the Catch2 suite could have covered anyway (no GPU-dependent tests).
        # See scripts/coverage.sh and docs/guides/TESTING.md.
        execute_process(
            COMMAND ${CMAKE_CXX_COMPILER} --coverage -print-file-name=libgcov.a
            OUTPUT_VARIABLE _reusex_coverage_libgcov
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(EXISTS "${_reusex_coverage_libgcov}")
            message(STATUS "Code coverage: pinning libgcov.a to ${_reusex_coverage_libgcov}")
            # link_libraries() (not add_link_options()): the archive must be
            # placed in the *library* list, ordered after the object files
            # that reference __gcov_init, or static-archive link-order rules
            # mean it gets scanned before anything needs it and nothing is
            # pulled from it. add_link_options() places raw flags too early
            # (before the objects) to satisfy that requirement.
            link_libraries("${_reusex_coverage_libgcov}")
        else()
            message(WARNING
                "Code coverage: could not resolve libgcov.a for "
                "${CMAKE_CXX_COMPILER} (got '${_reusex_coverage_libgcov}'). "
                "If WITH_CUDA=ON pulls in a second GCC's lib directory ahead "
                "of the main compiler's own, the linker may silently pick up "
                "the wrong libgcov.a and coverage data will be empty for "
                ".cpp files. See cmake/Coverage.cmake.")
        endif()
        unset(_reusex_coverage_libgcov)
    else()
        message(WARNING
            "ENABLE_COVERAGE requested but coverage instrumentation is only "
            "supported with GCC or Clang (detected compiler: "
            "${CMAKE_CXX_COMPILER_ID}); ignoring.")
    endif()
endif()
