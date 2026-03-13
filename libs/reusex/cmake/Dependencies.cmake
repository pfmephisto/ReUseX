# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Find all project dependencies
# ===============================================

message(STATUS "Finding project dependencies...")

# -----------------------------------------------
# Core dependencies
# -----------------------------------------------
find_package(Qt6 COMPONENTS Core Widgets Gui OpenGL REQUIRED) 
find_package(Boost CONFIG REQUIRED)
find_package(TBB REQUIRED)

# Logging and formatting
find_package(spdlog CONFIG REQUIRED)
set_target_properties(spdlog::spdlog PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
find_package(range-v3 CONFIG REQUIRED)

# -----------------------------------------------
# Deep Learning
# -----------------------------------------------
find_package(Torch 2.9.0 CONFIG REQUIRED)
find_package(trtsam3 CONFIG REQUIRED)
find_package(tokenizers_cpp CONFIG REQUIRED)

# -----------------------------------------------
# Computer Vision & Point Cloud Processing
# -----------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED)

# -----------------------------------------------
# Geometry & Optimization
# -----------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(highs REQUIRED)
find_package(SCIP REQUIRED)
find_package(embree REQUIRED)

# CGAL with support modules
find_package(CGAL REQUIRED Core)
include(CGAL_Eigen3_support)
include(CGAL_SCIP_support)
include(CGAL_TBB_support)

# -----------------------------------------------
# Graph Algorithms
# -----------------------------------------------
find_package(GraphBLAS REQUIRED)
find_package(LAGraph REQUIRED)

# -----------------------------------------------
# I/O Formats
# -----------------------------------------------
find_package(E57Format REQUIRED)
find_package(opennurbs REQUIRED)

# -----------------------------------------------
# Optional GUI support
# -----------------------------------------------
option(GUI_ENABLED "if true, enables GUI visualization features" OFF)
if (GUI_ENABLED)
    message(STATUS "GUI support enabled")
    message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
    message(STATUS "CGAL_GRAPHICSVIEW_PACKAGE_DIR: ${CGAL_GRAPHICSVIEW_PACKAGE_DIR}")
    message(STATUS "CGAL_MODULES_DIR: ${CGAL_MODULES_DIR}")
    
    find_package(CGAL COMPONENTS Qt6) 
    
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    add_definitions(-DCGAL_USE_BASIC_VIEWER)
else()
    message(STATUS "GUI support disabled")
endif()

message(STATUS "All dependencies found successfully")
