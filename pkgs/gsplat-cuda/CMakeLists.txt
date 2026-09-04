# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Minimal CMake build for gsplat's CUDA rasterization C++ backend
# (nerfstudio-project/gsplat, Apache-2.0). gsplat ships no CMakeLists for
# its csrc/ tree (upstream builds via torch cpp_extension); this file is
# dropped into the gsplat/cuda/ directory by the Nix derivation and mirrors
# the flags from gsplat/cuda/build.py::get_build_parameters().
cmake_minimum_required(VERSION 3.24)

project(gsplat_cuda LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 80 86 89)
endif()

# LibTorch provides the Torch CMake package config (TorchConfig.cmake),
# which also pulls in Caffe2::* imported targets and CUDA settings.
find_package(Torch REQUIRED)
find_package(CUDAToolkit REQUIRED)

# A handful of TUs pull in <torch/extension.h> (guarded by GSPLAT_BUILD_3DGUT),
# which transitively needs <Python.h> and <pybind11/pybind11.h>. We do NOT
# build a Python extension -- no pybind entry (ext.cpp) is compiled -- but the
# headers must resolve. Grab the include dirs only.
find_package(Python3 COMPONENTS Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# ---------------------------------------------------------------------------
# Sources. Upstream globs csrc/*.cu + csrc/*.cpp. The pybind entry point
# (gsplat/cuda/ext.cpp) lives OUTSIDE csrc/ so it is naturally excluded.
# CameraWrappers.cu / ExternalDistortionWrappers.cu are internally guarded by
# #if GSPLAT_BUILD_CAMERA_WRAPPERS (default 0 in Config.h) and compile to empty
# translation units, so they are harmless to keep in the glob.
# ---------------------------------------------------------------------------
file(GLOB GSPLAT_CU_SOURCES  CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/csrc/*.cu")
file(GLOB GSPLAT_CPP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/csrc/*.cpp")

add_library(gsplat STATIC ${GSPLAT_CU_SOURCES} ${GSPLAT_CPP_SOURCES})
add_library(gsplat::gsplat ALIAS gsplat)

target_include_directories(gsplat
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/gsplat>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/csrc"
    "${CMAKE_CURRENT_SOURCE_DIR}/csrc/third_party/glm"
    ${Python3_INCLUDE_DIRS}
)
target_link_libraries(gsplat PRIVATE pybind11::headers)

# glm is bundled header-only; expose it to consumers too (some public headers
# under include/ pull in glm types).
target_include_directories(gsplat
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/csrc/third_party/glm>"
    "$<INSTALL_INTERFACE:include/gsplat/third_party/glm>"
)

# Torch propagates its own include dirs, CUDA libs and the C++ ABI macro.
target_link_libraries(gsplat PUBLIC ${TORCH_LIBRARIES} CUDA::cudart)

target_compile_options(gsplat PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
  $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
  $<$<COMPILE_LANGUAGE:CUDA>:--forward-unknown-opts>
  $<$<COMPILE_LANGUAGE:CUDA>:-use_fast_math>
  $<$<COMPILE_LANGUAGE:CUDA>:-diag-suppress=3189,20012,186>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-attributes>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-unknown-pragmas>
)

set_target_properties(gsplat PROPERTIES
  CUDA_SEPARABLE_COMPILATION OFF
  EXPORT_NAME gsplat
)

# ---------------------------------------------------------------------------
# Install + package config so consumers can find_package(gsplat-cuda CONFIG)
# and link gsplat::gsplat.
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS gsplat
  EXPORT gsplat-cudaTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Public headers (include/) + bundled glm needed transitively by include/.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gsplat"
)
# The main C++ API headers (Rasterization.h, RasterizeToPixelsFromWorld3DGS.h,
# Projection.h, SphericalHarmonics.h, QuatScaleToCovar.h, Relocation.h, ...)
# live in csrc/ alongside the .cu implementations and include each other by bare
# name, so ship every csrc header flat into the same include dir. Consumers get
# -I.../include/gsplat and `#include "Rasterization.h"` resolves.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/csrc/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gsplat"
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.cuh"
    PATTERN "third_party" EXCLUDE
)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/csrc/third_party/glm/glm"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gsplat/third_party/glm"
)

install(EXPORT gsplat-cudaTargets
  FILE gsplat-cudaTargets.cmake
  NAMESPACE gsplat::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gsplat-cuda"
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cuda-config.cmake.in"
[==[
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(Torch)
find_dependency(CUDAToolkit)
include("${CMAKE_CURRENT_LIST_DIR}/gsplat-cudaTargets.cmake")
]==])

configure_package_config_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cuda-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cudaConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gsplat-cuda"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cudaConfigVersion.cmake"
  VERSION 0.1.0
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cudaConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/gsplat-cudaConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/gsplat-cuda"
)
