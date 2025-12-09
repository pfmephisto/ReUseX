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
     "${CMAKE_CURRENT_SOURCE_DIR}/src/ReUseX/**/*.cpp")

file(GLOB_RECURSE REUSEX_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/ReUseX/**/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/pcl/**/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/spdmon/**/*.hpp")

message(STATUS "Found ${CMAKE_MATCH_COUNT} ReUseX source files")

# -----------------------------------------------
# Generate version header
# -----------------------------------------------
file(READ ${CMAKE_SOURCE_DIR}/LICENSE.md LICENSE_TEXT)
string(REPLACE "\"" "\\\"" LICENSE_TEXT "${LICENSE_TEXT}")
configure_file(
    ${CMAKE_SOURCE_DIR}/include/ReUseX/core/version.hpp.in
    ${CMAKE_BINARY_DIR}/generated/ReUseX/core/version.hpp
    @ONLY
)

# -----------------------------------------------
# Create library target
# -----------------------------------------------
add_library(${PROJECT_NAME} SHARED ${REUSEX_SOURCES} ${REUSEX_HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# -----------------------------------------------
# Link dependencies
# -----------------------------------------------
target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        # Geometry & CGAL
        CGAL::CGAL
        CGAL::Eigen3_support
        CGAL::SCIP_support
        CGAL::TBB_support

        # 3D Formats
        OpenNURBS
        E57Format

        # Point Cloud Library
        pcl_common
        pcl_io
        pcl_filters
        pcl_segmentation
        pcl_registration
        pcl_visualization
        
        # Ray tracing
        embree

        # Linear Algebra
        Eigen3::Eigen

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
        
        # Logging (keep internal to avoid ABI issues)
        spdlog::spdlog
        
        # Ranges
        range-v3::range-v3
)

# -----------------------------------------------
# Compiler options
# -----------------------------------------------
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(${PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})

# -----------------------------------------------
# Export library information
# -----------------------------------------------
set(REUSEX_LIBRARY_TARGET ${PROJECT_NAME} PARENT_SCOPE)
