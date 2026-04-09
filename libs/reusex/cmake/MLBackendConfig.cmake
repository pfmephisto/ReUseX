# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ML Backend Configuration
# ===============================================
#
# This file configures which ML inference backends are enabled and links
# their dependencies to the ReUseX library target.
#
# ## Available Backends
#
# - **TensorRT**: NVIDIA TensorRT for optimized GPU inference
#   - File extensions: .engine
#   - Requirements: CUDA-compatible GPU, TensorRT library
#   - Fastest inference on NVIDIA GPUs
#
# - **LibTorch**: PyTorch C++ API for neural network inference
#   - File extensions: .pt, .pth, .torchscript
#   - Requirements: PyTorch/LibTorch library
#   - Cross-platform CPU/GPU support, best model compatibility
#
# - **ONNX**: ONNX Runtime for cross-platform inference
#   - File extensions: .onnx
#   - Requirements: ONNX Runtime library
#   - Standard interchange format, CPU and GPU execution providers
#
# - **OpenVINO**: Intel OpenVINO toolkit
#   - File extensions: .xml, .bin
#   - Requirements: OpenVINO toolkit
#   - Optimized for Intel hardware (CPU, GPU, VPU)
#
# ## Usage
#
# Backends are selected via the ML_BACKENDS CMake variable:
#   -DML_BACKENDS="TensorRT;LibTorch"  # Enable specific backends
#   -DML_BACKENDS="AUTO"                # Auto-detect available backends
#
# The ENABLED_ML_BACKENDS variable is set by the root CMakeLists.txt and
# contains the list of backends to enable in this build.
#
# ## Build System Integration
#
# For each enabled backend:
# 1. Compile definition REUSEX_USE_<BACKEND> is added
# 2. Backend-specific source files are included in the build
# 3. Backend libraries are linked to the ReUseX target
# 4. Disabled backend source files are excluded from compilation

# -----------------------------------------------
# Exclude disabled ML backend source files
# -----------------------------------------------
# This must happen before add_library() is called, so it's included
# in ReUseXLibrary.cmake before target creation

if(NOT "TensorRT" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/vision/tensor_rt/.*\\.(cpp|cu)$")
    list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/vision/tensor_rt/.*\\.hpp$")
    message(STATUS "Excluding TensorRT source files from build")
endif()

if(NOT "LibTorch" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/vision/libtorch/.*\\.cpp$")
    list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/vision/libtorch/.*\\.hpp$")
    # Exclude LibTorch-specific utility files (NMS uses torch::Tensor)
    list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/vision/nms\\.cpp$")
    list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/vision/nms\\.hpp$")
    message(STATUS "Excluding LibTorch source files from build")
endif()

if(NOT "ONNX" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/vision/onnx/.*\\.cpp$")
    list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/vision/onnx/.*\\.hpp$")
endif()

if(NOT "OpenVINO" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER REUSEX_SOURCES EXCLUDE REGEX ".*/vision/openvino/.*\\.cpp$")
    list(FILTER REUSEX_HEADERS EXCLUDE REGEX ".*/vision/openvino/.*\\.hpp$")
endif()

# -----------------------------------------------
# Configure ML Backend Compile Definitions and Libraries
# -----------------------------------------------
# This function is called from ReUseXLibrary.cmake after the target is created
function(configure_ml_backends TARGET_NAME)
    foreach(backend IN LISTS ENABLED_ML_BACKENDS)
        if(backend STREQUAL "TensorRT")
            target_compile_definitions(${TARGET_NAME} PRIVATE REUSEX_USE_TENSORRT)
            target_link_libraries(${TARGET_NAME} PRIVATE
                trtsam3::trtsam_core
                tokenizers_cpp::tokenizers_cpp
            )
            message(STATUS "Enabled ML backend: TensorRT")

        elseif(backend STREQUAL "LibTorch")
            target_compile_definitions(${TARGET_NAME} PRIVATE REUSEX_USE_LIBTORCH)
            target_link_libraries(${TARGET_NAME} PRIVATE torch)
            message(STATUS "Enabled ML backend: LibTorch")

        elseif(backend STREQUAL "ONNX")
            target_compile_definitions(${TARGET_NAME} PRIVATE REUSEX_USE_ONNX)
            target_link_libraries(${TARGET_NAME} PRIVATE onnxruntime::onnxruntime)
            message(STATUS "Enabled ML backend: ONNX Runtime")

        elseif(backend STREQUAL "OpenVINO")
            target_compile_definitions(${TARGET_NAME} PRIVATE REUSEX_USE_OPENVINO)
            target_link_libraries(${TARGET_NAME} PRIVATE openvino::openvino)
            message(STATUS "Enabled ML backend: OpenVINO")

        else()
            message(WARNING "Unknown ML backend requested: ${backend}")
        endif()
    endforeach()

    # Summary message
    list(LENGTH ENABLED_ML_BACKENDS BACKEND_COUNT)
    if(BACKEND_COUNT EQUAL 0)
        message(STATUS "No ML backends enabled - ML features will be unavailable")
    else()
        message(STATUS "Total ML backends enabled: ${BACKEND_COUNT}")
    endif()
endfunction()
