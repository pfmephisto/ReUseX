# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# MIP Solver Configuration
# ===============================================
#
# This file configures which Mixed Integer Programming (MIP) solver is used
# for room segmentation optimization in the Solidifier class.
#
# ## Available Solvers
#
# - **cuOpt** (Recommended for NVIDIA GPUs):
#   - GPU-accelerated optimization using NVIDIA cuOpt
#   - Fastest for large problems on CUDA-capable GPUs
#   - Requires: NVIDIA GPU with CUDA support, cuOpt library
#   - Compile definition: USE_CUOPT
#   - Note: Currently experimental support
#
# - **HiGHS** (Recommended for CPU):
#   - High-performance open-source MIP solver
#   - Excellent CPU performance, widely used
#   - Pure C++ implementation, portable
#   - Compile definition: USE_HIGHS
#   - Built with GPU/CUDA support for PDLP solver (not currently used by Solidifier)
#
# - **SCIP** (Alternative CPU solver):
#   - Academic MIP solver with good performance
#   - More features than HiGHS but slower in practice
#   - Requires: SCIP library
#   - Compile definition: CGAL_USE_SCIP
#
# - **GLPK** (Fallback):
#   - GNU Linear Programming Kit
#   - Slower than HiGHS/SCIP, but lightweight
#   - Compile definition: CGAL_USE_GLPK
#   - Not currently tested
#
# ## Solver Selection Priority
#
# When MIP_SOLVER=AUTO (default):
# 1. cuOpt (if available and GPU present)
# 2. HiGHS (preferred CPU solver)
# 3. SCIP (fallback)
# 4. Error if none found
#
# ## Usage
#
# Set the solver via CMake:
#   -DMIP_SOLVER=AUTO     # Auto-detect best available (default)
#   -DMIP_SOLVER=CUOPT    # Force cuOpt (requires GPU)
#   -DMIP_SOLVER=HIGHS    # Force HiGHS
#   -DMIP_SOLVER=SCIP     # Force SCIP
#
# ## Implementation Details
#
# The solver is selected at compile-time using preprocessor definitions.
# The Solidifier class (geometry/Solidifier.cpp) uses these definitions to
# include the appropriate CGAL MIP traits header and instantiate the solver.
#
# Solver parameters (e.g., memory limits, timeout) can be configured in
# Solidifier::Impl::_configureSolver() in Solidifier.cpp.

# -----------------------------------------------
# MIP Solver Selection
# -----------------------------------------------

# User-configurable option
set(MIP_SOLVER "AUTO" CACHE STRING "MIP solver to use (AUTO, CUOPT, HIGHS, SCIP)")
set_property(CACHE MIP_SOLVER PROPERTY STRINGS AUTO CUOPT HIGHS SCIP GLPK)

# Determine which solver to enable based on user preference and availability
if(MIP_SOLVER STREQUAL "AUTO")
    # Auto-detect: prefer GPU solver, then HiGHS, then SCIP
    if(cuOpt_FOUND)
        set(USE_MIP_SOLVER "CUOPT")
    elseif(highs_FOUND)
        set(USE_MIP_SOLVER "HIGHS")
    elseif(SCIP_FOUND)
        set(USE_MIP_SOLVER "SCIP")
    elseif(GLPK_FOUND)
        set(USE_MIP_SOLVER "GLPK")
    else()
        message(FATAL_ERROR
            "No MIP solver found. Please install one of:\n"
            "  - HiGHS (recommended, CPU): https://highs.dev/\n"
            "  - SCIP (alternative, CPU): https://www.scipopt.org/\n"
            "  - cuOpt (GPU): https://developer.nvidia.com/cuopt\n"
            "Or specify -DMIP_SOLVER=<solver> to use a specific solver.")
    endif()
else()
    # User explicitly requested a solver
    set(USE_MIP_SOLVER ${MIP_SOLVER})
endif()

# -----------------------------------------------
# Configure MIP Solver Compile Definitions and Libraries
# -----------------------------------------------
# This function is called from ReUseXLibrary.cmake after the target is created
function(configure_mip_solver TARGET_NAME)
    if(USE_MIP_SOLVER STREQUAL "CUOPT")
        if(NOT cuOpt_FOUND)
            message(FATAL_ERROR
                "cuOpt solver requested but not found.\n"
                "Please install cuOpt or use -DMIP_SOLVER=AUTO to auto-detect.")
        endif()
        target_compile_definitions(${TARGET_NAME} PUBLIC USE_CUOPT)
        target_link_libraries(${TARGET_NAME} PUBLIC cuOpt::cuOpt)
        message(STATUS "MIP Solver: cuOpt (GPU-accelerated)")

    elseif(USE_MIP_SOLVER STREQUAL "HIGHS")
        if(NOT highs_FOUND)
            message(FATAL_ERROR
                "HiGHS solver requested but not found.\n"
                "Please install HiGHS or use -DMIP_SOLVER=AUTO to auto-detect.")
        endif()
        target_compile_definitions(${TARGET_NAME} PUBLIC USE_HIGHS)
        # Note: highs::highs already linked in main library dependencies
        message(STATUS "MIP Solver: HiGHS (CPU)")

    elseif(USE_MIP_SOLVER STREQUAL "SCIP")
        if(NOT SCIP_FOUND)
            message(FATAL_ERROR
                "SCIP solver requested but not found.\n"
                "Please install SCIP or use -DMIP_SOLVER=AUTO to auto-detect.")
        endif()
        target_compile_definitions(${TARGET_NAME} PUBLIC CGAL_USE_SCIP)
        # Note: CGAL::SCIP_support already linked in main library dependencies
        message(STATUS "MIP Solver: SCIP (CPU)")

    elseif(USE_MIP_SOLVER STREQUAL "GLPK")
        if(NOT GLPK_FOUND)
            message(FATAL_ERROR
                "GLPK solver requested but not found.\n"
                "Please install GLPK or use -DMIP_SOLVER=AUTO to auto-detect.")
        endif()
        target_compile_definitions(${TARGET_NAME} PUBLIC CGAL_USE_GLPK)
        target_link_libraries(${TARGET_NAME} PUBLIC GLPK::GLPK)
        message(STATUS "MIP Solver: GLPK (CPU, fallback)")

    else()
        message(FATAL_ERROR
            "Unknown MIP solver: ${USE_MIP_SOLVER}\n"
            "Valid options: AUTO, CUOPT, HIGHS, SCIP, GLPK")
    endif()

    # Export for visibility in parent scope
    set(USE_MIP_SOLVER ${USE_MIP_SOLVER} PARENT_SCOPE)
endfunction()
