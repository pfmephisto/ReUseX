# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX - Point Cloud Processing Library
# ===============================================

cmake_minimum_required(VERSION 3.17)

project(ReUseX
    VERSION 0.0.2
    LANGUAGES CXX CUDA
    DESCRIPTION "Tool for processing interior lidar scans of buildings"
)

# ===============================================
# Global Settings
# ===============================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Global properties
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # For clangd/LSP support
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)  # Required for shared libraries

# Suppress CGAL warnings
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON)

enable_language(CXX)
enable_language(C)

# ===============================================
# Binary output directory
# ===============================================
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()

# ===============================================
# Shared CMake Modules
# ===============================================
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# ===============================================
# Build Subprojects
# ===============================================

# Core library
add_subdirectory(libs/reusex)

# CLI application
add_subdirectory(apps/rux)

# Optional Python bindings
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

# ===============================================
# Unit Tests
# ===============================================
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ===============================================
# Documentation
# ===============================================
option(BUILD_DOCUMENTATION "Build documentation" ON)
if(BUILD_DOCUMENTATION)
    include(Documentation)
endif()

# ===============================================
# Installation
# ===============================================
include(Installation)

# ===============================================
# Summary
# ===============================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "ReUseX Configuration Summary")
message(STATUS "========================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build Type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "Install Prefix:    ${CMAKE_INSTALL_PREFIX}")
message(STATUS "C++ Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CUDA Support:      ${CMAKE_CUDA_COMPILER_ID} ${CMAKE_CUDA_COMPILER_VERSION}")
message(STATUS "Visualization:     ${BUILD_VISUALIZATION}")
message(STATUS "Documentation:     ${BUILD_DOCUMENTATION}")
message(STATUS "Unit Tests:        ${BUILD_TESTS}")
message(STATUS "Python Bindings:   ${BUILD_PYTHON_BINDINGS}")
message(STATUS "========================================")
message(STATUS "")
