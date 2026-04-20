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
find_package(fmt CONFIG REQUIRED)
#find_package(spdlog CONFIG REQUIRED)
#set_target_properties(spdlog::spdlog PROPERTIES
#    POSITION_INDEPENDENT_CODE ON
#)
find_package(range-v3 CONFIG REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

# -----------------------------------------------
# Protobuf dependencies (for ML backends)
# -----------------------------------------------
# Both TensorRT (via tokenizers_cpp) and LibTorch depend on protobuf.
# Protobuf requires abseil and utf8_range to load successfully.
# Finding all three explicitly here prevents target redefinition errors when
# multiple backends try to find protobuf independently.
#
# Root cause: tokenizers_cpp calls find_package(Protobuf QUIET), creating
# partial targets. When LibTorch calls find_package(Protobuf CONFIG QUIET),
# CMake detects some targets exist but not all, triggering an error.
#
# Solution: Find protobuf once with all dependencies satisfied.
find_package(absl CONFIG REQUIRED)
find_package(utf8_range CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)

# -----------------------------------------------
# ML Backends (Optional)
# -----------------------------------------------
# User can specify which backends to enable via:
#   -DML_BACKENDS="AUTO" (default - find all available)
#   -DML_BACKENDS="TensorRT;LibTorch" (explicit list)
#   -DML_BACKENDS="NONE" (disable all)

set(ML_BACKENDS "AUTO" CACHE STRING "ML backends to enable (AUTO, NONE, or list: TensorRT;LibTorch;ONNX;OpenVINO)")

# Define available backends and their dependencies
set(ML_BACKEND_TensorRT_PACKAGES "trtsam3;tokenizers_cpp")
set(ML_BACKEND_LibTorch_PACKAGES "Torch 2.9.0")
set(ML_BACKEND_ONNX_PACKAGES "onnxruntime;tokenizers_cpp")
set(ML_BACKEND_OpenVINO_PACKAGES "openvino")  # Future

# -----------------------------------------------
# Workaround: OpenCV 4.13.0 + LibTorch legacy FindCUDA cache pollution
# -----------------------------------------------
# LibTorch's Caffe2Config.cmake calls a bundled legacy find_package(CUDA) which
# caches CUDA_FOUND and CUDA_VERSION. On reconfiguration, when trtsam3 triggers
# find_dependency(OpenCV) before LibTorch is processed, OpenCV sees the cached
# CUDA_FOUND=TRUE and calls the legacy find_cuda_helper_libs() macro -- which
# does not exist yet (macros are not cached). Two mitigations:
#
# 1. Unset legacy FindCUDA cache variables so OpenCV uses modern CUDAToolkit.
# 2. Define a find_cuda_helper_libs shim as a safety net.

# Mitigation 1: Clear stale legacy FindCUDA cache entries
unset(CUDA_FOUND CACHE)
unset(CUDA_VERSION CACHE)
unset(CUDA_VERSION_STRING CACHE)

# Mitigation 2: Provide find_cuda_helper_libs if not already defined
if(NOT COMMAND find_cuda_helper_libs)
  macro(find_cuda_helper_libs _name)
    find_library(CUDA_${_name}_LIBRARY
      NAMES ${_name}
      PATHS "${CUDAToolkit_LIBRARY_DIR}" "${CUDA_TOOLKIT_ROOT_DIR}"
      PATH_SUFFIXES lib64 lib/x64 lib
      NO_DEFAULT_PATH
    )
    mark_as_advanced(CUDA_${_name}_LIBRARY)
  endmacro()
endif()

# Determine which backends to search for
if(ML_BACKENDS STREQUAL "AUTO")
    set(BACKENDS_TO_FIND TensorRT LibTorch ONNX OpenVINO)
elseif(ML_BACKENDS STREQUAL "NONE")
    set(BACKENDS_TO_FIND "")
    message(STATUS "All ML backends disabled")
else()
    set(BACKENDS_TO_FIND ${ML_BACKENDS})
endif()

# Find each backend
set(ENABLED_ML_BACKENDS "")
foreach(backend IN LISTS BACKENDS_TO_FIND)
    message(STATUS "Searching for ${backend} backend...")
    set(all_found TRUE)
    foreach(pkg IN LISTS ML_BACKEND_${backend}_PACKAGES)
        # Parse package name and version (e.g., "Torch 2.9.0" -> "Torch" + "2.9.0")
        string(REPLACE " " ";" pkg_parts ${pkg})
        list(GET pkg_parts 0 pkg_name)
        list(LENGTH pkg_parts pkg_parts_len)
        if(pkg_parts_len GREATER 1)
            list(GET pkg_parts 1 pkg_version)
            find_package(${pkg_name} ${pkg_version} CONFIG QUIET)
        else()
            find_package(${pkg_name} CONFIG QUIET)
        endif()

        if(NOT ${pkg_name}_FOUND)
            set(all_found FALSE)
            if(NOT ML_BACKENDS STREQUAL "AUTO")
                message(FATAL_ERROR "${backend} backend requested but ${pkg_name} not found")
            endif()
            break()
        endif()
    endforeach()

    if(all_found)
        list(APPEND ENABLED_ML_BACKENDS ${backend})
        message(STATUS "ML Backend ${backend}: ENABLED")
    else()
        message(STATUS "ML Backend ${backend}: DISABLED (dependencies not found)")
    endif()
endforeach()

# At least one backend must be enabled (unless explicitly NONE)
if(NOT ML_BACKENDS STREQUAL "NONE" AND NOT ENABLED_ML_BACKENDS)
    message(FATAL_ERROR "No ML backends available. Install at least one: LibTorch, TensorRT, ONNX Runtime, or OpenVINO")
endif()

# Export for use in library CMakeLists and root summary
set(ENABLED_ML_BACKENDS ${ENABLED_ML_BACKENDS} PARENT_SCOPE)

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
find_package(cuOpt)  # Optional - requires NVIDIA GPU
find_package(embree REQUIRED)

# CGAL with support modules
find_package(CGAL REQUIRED Core)
include(CGAL_Eigen3_support)
include(CGAL_SCIP_support)
include(CGAL_TBB_support)

# -----------------------------------------------
# Graph Algorithms
# -----------------------------------------------
find_package(igraph REQUIRED)

# -----------------------------------------------
# I/O Formats
# -----------------------------------------------
find_package(E57Format REQUIRED)
find_package(opennurbs REQUIRED)

# -----------------------------------------------
# Speckle upload support (HTTP, JSON, hashing)
# -----------------------------------------------
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

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
