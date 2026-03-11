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
     "${CMAKE_CURRENT_SOURCE_DIR}/include/ReUseX/visualize/*.hpp")

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
target_include_directories(ReUseX_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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

    PRIVATE 
        # Logging
        spdlog::spdlog
)

# -----------------------------------------------
# Compiler options
# -----------------------------------------------
target_link_libraries(ReUseX_visualization PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(ReUseX_visualization PUBLIC ${COMMON_COMPILER_FLAGS})

# -----------------------------------------------
# Export library information
# -----------------------------------------------
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REUSEX_VISUALIZATION_LIBRARY_TARGET ReUseX_visualization PARENT_SCOPE)
endif()
