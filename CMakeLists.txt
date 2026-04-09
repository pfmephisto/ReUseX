# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX - Point Cloud Processing Library
# ===============================================

cmake_minimum_required(VERSION 3.17)

project(ReUseX
    VERSION 0.0.4
    LANGUAGES CXX CUDA
    DESCRIPTION "Tool for processing interior lidar scans of buildings"
)

# ===============================================
# CUDA Flags (must be set early to avoid conflicts)
# ===============================================
# Prevent "incompatible redefinition for option 'optimize'" warning from nvcc
# PyTorch adds -O2 via INTERFACE_COMPILE_OPTIONS, so we match it to avoid conflicts
# Using -O2 instead of -O3 to prevent "incompatible redefinition" warning
set(CMAKE_CUDA_FLAGS "" CACHE STRING "Base CUDA compiler flags" FORCE)
set(CMAKE_CUDA_FLAGS_RELEASE "-DNDEBUG" CACHE STRING "CUDA Release flags (no -O flag, let deps handle it)" FORCE)
set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "-g -DNDEBUG" CACHE STRING "CUDA RelWithDebInfo flags" FORCE)
set(CMAKE_CUDA_FLAGS_DEBUG "-g" CACHE STRING "CUDA Debug flags" FORCE)

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
# Coverage Support (must be set before library)
# ===============================================
option(ENABLE_COVERAGE "Enable code coverage" OFF)
if(ENABLE_COVERAGE)
    message(STATUS "Code coverage enabled")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # These flags will be inherited by all targets
        add_compile_options(--coverage)
        add_link_options(--coverage)
    else()
        message(WARNING "Coverage only supported with GCC or Clang")
    endif()
endif()

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
message(STATUS "MIP Solver:        ${USE_MIP_SOLVER}")
# Convert list to comma-separated string for display
if(ENABLED_ML_BACKENDS)
    string(REPLACE ";" ", " ML_BACKENDS_DISPLAY "${ENABLED_ML_BACKENDS}")
else()
    set(ML_BACKENDS_DISPLAY "NONE")
endif()
message(STATUS "ML Backends:       ${ML_BACKENDS_DISPLAY}")
message(STATUS "Documentation:     ${BUILD_DOCUMENTATION}")
message(STATUS "Unit Tests:        ${BUILD_TESTS}")
message(STATUS "Python Bindings:   ${BUILD_PYTHON_BINDINGS}")
message(STATUS "========================================")
message(STATUS "")
