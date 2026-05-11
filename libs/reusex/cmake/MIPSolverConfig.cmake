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
# - **HiGHS** (Default):
#   - High-performance open-source MIP solver
#   - Excellent CPU performance, widely used
#   - Pure C++ implementation, portable
#   - Compile definition: USE_HIGHS
#
# ## Solver Selection Priority
#
# When MIP_SOLVER=AUTO (default):
# 1. cuOpt (if available and GPU present)
# 2. HiGHS (default CPU solver)
# 3. Error if none found
#
# ## Usage
#
# Set the solver via CMake:
#   -DMIP_SOLVER=AUTO    # Auto-detect best available (default)
#   -DMIP_SOLVER=CUOPT   # Force cuOpt (requires GPU)
#   -DMIP_SOLVER=HIGHS   # Force HiGHS

# -----------------------------------------------
# MIP Solver Selection
# -----------------------------------------------

set(MIP_SOLVER "AUTO" CACHE STRING "MIP solver to use (AUTO, CUOPT, HIGHS)")
set_property(CACHE MIP_SOLVER PROPERTY STRINGS AUTO CUOPT HIGHS)

if(MIP_SOLVER STREQUAL "AUTO")
    if(cuOpt_FOUND)
        set(USE_MIP_SOLVER "CUOPT")
    elseif(highs_FOUND)
        set(USE_MIP_SOLVER "HIGHS")
    else()
        message(FATAL_ERROR
            "No MIP solver found. Please install HiGHS: https://highs.dev/\n"
            "Or for GPU acceleration: https://developer.nvidia.com/cuopt")
    endif()
else()
    set(USE_MIP_SOLVER ${MIP_SOLVER})
endif()

# -----------------------------------------------
# Configure MIP Solver Compile Definitions and Libraries
# -----------------------------------------------
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

    else()
        message(FATAL_ERROR "Unknown MIP solver: ${USE_MIP_SOLVER}. Valid options: AUTO, CUOPT, HIGHS")
    endif()

    set(USE_MIP_SOLVER ${USE_MIP_SOLVER} PARENT_SCOPE)
endfunction()
