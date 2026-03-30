# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Library Target Configuration
# ===============================================

# -----------------------------------------------
# Collect source files
# -----------------------------------------------
# Using GLOB_RECURSE with CONFIGURE_DEPENDS to automatically detect new files
# Note: CONFIGURE_DEPENDS tells CMake to recheck the glob at build time
file(GLOB_RECURSE REUSEX_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp")
file(GLOB_RECURSE REUSEX_CUDA_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cu")
list(APPEND REUSEX_SOURCES ${REUSEX_CUDA_SOURCES})

file(GLOB_RECURSE REUSEX_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.cuh")

# Exclude visualization files from main library
list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/visualize/.*\\.cpp$")
list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/visualize/.*\\.hpp$")

list(LENGTH REUSEX_SOURCES REUSEX_SOURCE_COUNT)
message(STATUS "Found ${REUSEX_SOURCE_COUNT} ReUseX source files")

# -----------------------------------------------
# Create symlink for prefixed includes
# -----------------------------------------------
# Create build/include/reusex -> libs/reusex/include symlink
# This allows consumers (like rux) to use #include <reusex/core/...>
# while internal code uses #include "core/..." directly
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include)
file(CREATE_LINK
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include/reusex
    SYMBOLIC
)

# -----------------------------------------------
# Generate version header
# -----------------------------------------------
file(READ ${CMAKE_SOURCE_DIR}/LICENSE.md LICENSE_TEXT)
string(REPLACE "\"" "\\\"" LICENSE_TEXT "${LICENSE_TEXT}")
# Use CMAKE_PROJECT_VERSION to get the root project version (not subdirectory project version)
set(PROJECT_VERSION ${CMAKE_PROJECT_VERSION})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/version.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/core/version.hpp
    @ONLY
)

# -----------------------------------------------
# Create library target
# -----------------------------------------------
# Note: Target name is "ReUseX" regardless of project name for consistency
add_library(ReUseX SHARED ${REUSEX_SOURCES} ${REUSEX_HEADERS})

# Include directories
target_include_directories(ReUseX
    PUBLIC
        # For consumers (rux, external): see headers as reusex/core/...
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        # For internal library code: direct access to flat structure
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -----------------------------------------------
# Link dependencies
# -----------------------------------------------
target_link_libraries(ReUseX
    PUBLIC
        # Geometry & CGAL
        CGAL::CGAL
        CGAL::Eigen3_support
        CGAL::SCIP_support
        CGAL::TBB_support

        # MIP Solver (HiGHS primary, SCIP fallback)
        highs::highs

        # 3D Formats
        OpenNURBS
        E57Format

        # Point Cloud Library (excluding visualization)
        pcl_common
        pcl_io
        pcl_filters
        pcl_segmentation
        pcl_registration

        # Ray tracing
        embree

        # Linear Algebra
        Eigen3::Eigen

        # Formatting used by ReUseX logging facade
        fmt::fmt

        # JSON serialization
        nlohmann_json::nlohmann_json

        # Graph algorithms
        SuiteSparse::GraphBLAS
        SuiteSparse::LAGraph
        SuiteSparse::LAGraphX

        # Computer Vision
        opencv_core
        opencv_imgproc
        opencv_highgui

        # SLAM
        rtabmap::rtabmap

    PRIVATE
        # Deep Learning (keep internal to avoid exposing torch ABI)
        torch

        # Ranges
        range-v3::range-v3
	      trtsam3::trtsam_core

	      tokenizers_cpp::tokenizers_cpp

        # Speckle upload support
        CURL::libcurl
        nlohmann_json::nlohmann_json
        OpenSSL::Crypto
)

# -----------------------------------------------
# Compiler options
# -----------------------------------------------
target_link_libraries(ReUseX PUBLIC ${COMMON_LINKER_FLAGS})
# Apply compiler flags only to C/C++, not CUDA (CUDA has its own flags)
target_compile_options(ReUseX PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILER_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${COMMON_COMPILER_FLAGS}>
)

# -----------------------------------------------
# MIP Solver Selection
# -----------------------------------------------
# Priority: cuOpt (GPU) > HiGHS > SCIP
set(MIP_SOLVER "AUTO" CACHE STRING "MIP solver to use (AUTO, CUOPT, HIGHS, SCIP)")
set_property(CACHE MIP_SOLVER PROPERTY STRINGS AUTO CUOPT HIGHS SCIP)

# Determine which solver to enable
if(MIP_SOLVER STREQUAL "AUTO")
  if(cuOpt_FOUND)
    set(USE_MIP_SOLVER "CUOPT")
  elseif(highs_FOUND)
    set(USE_MIP_SOLVER "HIGHS")
  elseif(SCIP_FOUND)
    set(USE_MIP_SOLVER "SCIP")
  else()
    message(FATAL_ERROR "No MIP solver found. Please install cuOpt, HiGHS, or SCIP.")
  endif()
else()
  set(USE_MIP_SOLVER ${MIP_SOLVER})
endif()

# Set compile definition and link library
if(USE_MIP_SOLVER STREQUAL "CUOPT")
  if(NOT cuOpt_FOUND)
    message(FATAL_ERROR "cuOpt solver requested but not found")
  endif()
  target_compile_definitions(ReUseX PUBLIC USE_CUOPT)
  target_link_libraries(ReUseX PUBLIC cuOpt::cuOpt)
  message(STATUS "MIP Solver: cuOpt (GPU-accelerated)")
elseif(USE_MIP_SOLVER STREQUAL "HIGHS")
  if(NOT highs_FOUND)
    message(FATAL_ERROR "HiGHS solver requested but not found")
  endif()
  target_compile_definitions(ReUseX PUBLIC USE_HIGHS)
  message(STATUS "MIP Solver: HiGHS (CPU)")
elseif(USE_MIP_SOLVER STREQUAL "SCIP")
  if(NOT SCIP_FOUND)
    message(FATAL_ERROR "SCIP solver requested but not found")
  endif()
  target_compile_definitions(ReUseX PUBLIC CGAL_USE_SCIP)
  message(STATUS "MIP Solver: SCIP (CPU)")
else()
  message(FATAL_ERROR "Unknown MIP solver: ${USE_MIP_SOLVER}")
endif()

# -----------------------------------------------
# Export library information
# -----------------------------------------------
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REUSEX_LIBRARY_TARGET ReUseX PARENT_SCOPE)
endif()
