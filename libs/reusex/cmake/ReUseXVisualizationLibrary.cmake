# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Visualization Library Target Configuration
# ===============================================

# -----------------------------------------------
# Collect visualization source files
# -----------------------------------------------
file(GLOB REUSEX_VIS_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/visualize/*.cpp")

file(GLOB REUSEX_VIS_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/visualize/*.hpp")

# Add geometry files that use Visualizer
list(APPEND REUSEX_VIS_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/mesh.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/texture_mesh.cpp")

list(LENGTH REUSEX_VIS_SOURCES REUSEX_VIS_SOURCE_COUNT)
message(STATUS "Found ${REUSEX_VIS_SOURCE_COUNT} ReUseX visualization source files")

# -----------------------------------------------
# Create visualization library target
# -----------------------------------------------
add_library(ReUseX_visualization SHARED ${REUSEX_VIS_SOURCES} ${REUSEX_VIS_HEADERS})

# Include directories
target_include_directories(ReUseX_visualization
    PUBLIC
        # For consumers: see headers as reusex/visualize/...
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        # For internal library code: direct access to flat structure
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# -----------------------------------------------
# Link dependencies
# -----------------------------------------------
target_link_libraries(ReUseX_visualization 
    PUBLIC
        # Depend on main ReUseX library
        ReUseX

        # Point Cloud Library visualization
        pcl_visualization
        
        # Linear Algebra
        Eigen3::Eigen
)

# -----------------------------------------------
# Compiler options
# -----------------------------------------------
target_link_libraries(ReUseX_visualization PUBLIC ${COMMON_LINKER_FLAGS})
# Apply compiler flags only to C/C++ (visualization has no CUDA files, but keep consistent)
target_compile_options(ReUseX_visualization PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILER_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${COMMON_COMPILER_FLAGS}>
)

# -----------------------------------------------
# Export library information
# -----------------------------------------------
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REUSEX_VISUALIZATION_LIBRARY_TARGET ReUseX_visualization PARENT_SCOPE)
endif()
