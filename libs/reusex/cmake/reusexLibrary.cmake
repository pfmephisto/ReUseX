# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Library Target Configuration
# ===============================================
#
# The library is split into one static library per pipeline-stage module, with
# explicit `target_link_libraries` encoding the layer diagram in
# docs/STANDARDS.md §1. Illegal dependencies surface as link errors instead of
# being silently allowed (#222).
#
# Layer diagram (a module may only depend on lower layers):
#
#   Layer 4:  visualize
#   Layer 3:  segmentation  reconstruction  slam  io  vision   (peers)
#   Layer 2:  core
#   Layer 1:  utils, types.hpp
#
# `reusex_geometry_common` holds the CGAL/PCL geometry helpers (`geometry/utils`,
# `cgal_utils`, `CoplanarPolygon`, `BuildingComponent`) that both reconstruction
# and the core persistence layer need; it sits just above utils so those
# consumers can share it without linking each other.
#
# The `reusex` target remains a backward-compatible INTERFACE umbrella that
# links every module, so apps/rux, apps/ruxd, tests and bindings build
# unchanged.

# -----------------------------------------------
# Per-module source discovery
# -----------------------------------------------
# Phase B (#222) relocated segmentation/reconstruction/slam into their own
# src/<module>/ dirs. The shared CGAL/PCL helpers (reusex_geometry_common) still
# live under src/geometry/ and are listed explicitly below.
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Layer 1 — utils (no internal dependencies)
file(GLOB_RECURSE REUSEX_UTILS_SOURCES CONFIGURE_DEPENDS "${SRC}/utils/*.cpp")

# Layer 2 — core (ProjectDB, logging, materials, stages, validate)
file(GLOB_RECURSE REUSEX_CORE_SOURCES CONFIGURE_DEPENDS "${SRC}/core/*.cpp")

# Layer 1.5 — shared geometry helpers (CGAL/PCL primitives). Kept separate so
# core (persistence of BuildingComponent) and the geometry peers can depend on
# it without depending on each other.
set(REUSEX_GEOMETRY_COMMON_SOURCES
    ${SRC}/geometry/utils.cpp
    ${SRC}/geometry/cgal_utils.cpp
    ${SRC}/geometry/CoplanarPolygon.cpp
    ${SRC}/geometry/BuildingComponent.cpp)

# Layer 3 — io (format conversion only)
file(GLOB_RECURSE REUSEX_IO_SOURCES CONFIGURE_DEPENDS "${SRC}/io/*.cpp")

# Layer 3 — vision (ML models, backends, datasets) — includes CUDA kernels
file(GLOB_RECURSE REUSEX_VISION_SOURCES CONFIGURE_DEPENDS "${SRC}/vision/*.cpp")
file(GLOB_RECURSE REUSEX_VISION_CUDA_SOURCES CONFIGURE_DEPENDS "${SRC}/vision/*.cu")
list(APPEND REUSEX_VISION_SOURCES ${REUSEX_VISION_CUDA_SOURCES})

# Layer 3 — segmentation (planes, rooms, instances, cloud creation/refinement)
# Relocated to src/segmentation/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_SEGMENTATION_SOURCES CONFIGURE_DEPENDS
     "${SRC}/segmentation/*.cpp")

# Layer 3 — reconstruction (cell complex, solidifier, mesh, texture, windows)
# Relocated to src/reconstruction/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_RECONSTRUCTION_SOURCES CONFIGURE_DEPENDS
     "${SRC}/reconstruction/*.cpp")

# Layer 3 — slam (registration / pose-graph optimization)
# Relocated to src/slam/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_SLAM_SOURCES CONFIGURE_DEPENDS
     "${SRC}/slam/*.cpp")

# Layer 4 — visualize (optional PCL/Qt/VTK)
file(GLOB_RECURSE REUSEX_VISUALIZE_SOURCES CONFIGURE_DEPENDS
     "${SRC}/visualize/*.cpp")

# Headers (attached to targets for IDE/install visibility; one flat tree).
file(GLOB_RECURSE REUSEX_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.cuh")

# -----------------------------------------------
# ML Backend Configuration (filters vision sources for disabled backends)
# -----------------------------------------------
# MLBackendConfig filters the variable named REUSEX_SOURCES, so hand it the
# vision source list under that name, then read it back.
set(REUSEX_SOURCES ${REUSEX_VISION_SOURCES})
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MLBackendConfig.cmake)
set(REUSEX_VISION_SOURCES ${REUSEX_SOURCES})

# -----------------------------------------------
# Create symlink for prefixed includes
# -----------------------------------------------
# build/include/reusex -> libs/reusex/include so consumers can use
# #include <reusex/core/...> while internal code uses "core/..." directly.
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
set(PROJECT_VERSION ${CMAKE_PROJECT_VERSION})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/version.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/core/version.hpp
    @ONLY
)

# ===============================================
# Shared usage requirements (include dirs, flags, external deps)
# ===============================================
# Every module links this INTERFACE target, so include dirs, compile options
# and third-party libraries are declared once and propagated uniformly. It also
# propagates to consumers of the `reusex` umbrella (rux, ruxd, tests, bindings).
add_library(reusex_common INTERFACE)

target_include_directories(reusex_common INTERFACE
    # Prefixed access for consumers: reusex/core/...
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/include>
    # Flat access for internal library code: core/..., geometry/...
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(reusex_common INTERFACE
    # Geometry & CGAL
    CGAL::CGAL
    CGAL::Eigen3_support
    CGAL::TBB_support
    # MIP Solver
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
    pcl_surface
    # Ray tracing
    embree
    # Linear Algebra
    Eigen3::Eigen
    # Formatting used by ReUseX logging facade
    fmt::fmt
    # JSON serialization
    nlohmann_json::nlohmann_json
    # Graph algorithms
    igraph::igraph
    # Computer Vision
    opencv_core
    opencv_imgproc
    opencv_highgui
    # SLAM database access
    rtabmap::rtabmap
    # Common linker flags
    ${COMMON_LINKER_FLAGS}
)

target_compile_options(reusex_common INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILER_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${COMMON_COMPILER_FLAGS}>
)

# MIP solver compile definition (USE_HIGHS / USE_CUOPT) attaches to the shared
# config so every module and the umbrella inherit it. MIPSolverConfig computes
# USE_MIP_SOLVER; its configure_mip_solver() helper uses PUBLIC keywords that an
# INTERFACE target rejects, so we apply the result with INTERFACE keywords here.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MIPSolverConfig.cmake)
if(USE_MIP_SOLVER STREQUAL "CUOPT")
    target_compile_definitions(reusex_common INTERFACE USE_CUOPT)
    target_link_libraries(reusex_common INTERFACE cuopt::cuopt)
    message(STATUS "MIP Solver: cuOpt (GPU-accelerated)")
elseif(USE_MIP_SOLVER STREQUAL "HIGHS")
    target_compile_definitions(reusex_common INTERFACE USE_HIGHS)
    # highs::highs already linked above.
    message(STATUS "MIP Solver: HiGHS (CPU)")
else()
    message(FATAL_ERROR "Unknown MIP solver: ${USE_MIP_SOLVER}")
endif()

# Some third-party PRIVATE-only dependencies of individual translation units
# (not part of the public interface). Declared on a dedicated INTERFACE target
# so we can attach them to the specific owning module(s) below.
add_library(reusex_private_deps INTERFACE)
target_link_libraries(reusex_private_deps INTERFACE
    # Ranges
    range-v3::range-v3
    # Speckle upload support (io/speckle.cpp)
    CURL::libcurl
    OpenSSL::Crypto
    # EXIF metadata reading (io/exif.cpp)
    Exiv2::exiv2lib
    # OpenMVS — Multi-View Stereo (AGPL-3.0-or-later), kept PRIVATE
    OpenMVS::MVS
)

# ===============================================
# Module targets
# ===============================================
# Helper: declare a module static library with the shared config attached.
function(reusex_add_module NAME)
    add_library(${NAME} STATIC ${ARGN})
    target_link_libraries(${NAME} PUBLIC reusex_common)
    target_link_libraries(${NAME} PRIVATE reusex_private_deps)
    set_target_properties(${NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

# --- Layer 1 ---------------------------------------------------------------
reusex_add_module(reusex_utils ${REUSEX_UTILS_SOURCES})

# --- Layer 1.5 — shared geometry helpers -----------------------------------
# geometry_common uses core/logging only; it must NOT depend on core to avoid a
# cycle (core -> geometry_common). It links utils + the shared config.
reusex_add_module(reusex_geometry_common ${REUSEX_GEOMETRY_COMMON_SOURCES})
target_link_libraries(reusex_geometry_common PUBLIC reusex_utils)

# --- Layer 2 ---------------------------------------------------------------
reusex_add_module(reusex_core ${REUSEX_CORE_SOURCES})
target_link_libraries(reusex_core PUBLIC reusex_utils)
# sqlite3 backs ProjectDB. Previously pulled in transitively via the monolithic
# shared lib; per-module static libs need it named explicitly so the symbols are
# on the final link line (DSO-missing-from-command-line otherwise).
find_package(SQLite3 REQUIRED)
target_link_libraries(reusex_core PUBLIC SQLite::SQLite3)
# LAYERING EXCEPTION (#222): core/ProjectDB persists geometry::BuildingComponent
# (a Layer-3 reconstruction type) and calls CoplanarPolygon (de)serialization.
# Those definitions live in reusex_geometry_common. Documented upward dependency
# (core -> geometry helper). See docs/STANDARDS.md §1 and the tdg TODO in
# src/core/ProjectDB.cpp.
target_link_libraries(reusex_core PUBLIC reusex_geometry_common)

# --- Layer 3 (peers — MUST NOT link each other) ----------------------------
reusex_add_module(reusex_io ${REUSEX_IO_SOURCES})
target_link_libraries(reusex_io PUBLIC reusex_core)
# LAYERING EXCEPTION (#222): io/export_scene consumes geometry::BuildingComponent
# / cgal_utils / unweld to serialize a reconstructed scene. Peer io -> geometry
# dependency, documented in docs/STANDARDS.md §1 ([target]) and export_scene.hpp.
target_link_libraries(reusex_io PUBLIC reusex_geometry_common reusex_reconstruction)

reusex_add_module(reusex_vision ${REUSEX_VISION_SOURCES})
target_link_libraries(reusex_vision PUBLIC reusex_core)
# ML backend libraries + REUSEX_USE_* defines attach to the vision module only.
configure_ml_backends(reusex_vision)

reusex_add_module(reusex_segmentation ${REUSEX_SEGMENTATION_SOURCES})
target_link_libraries(reusex_segmentation PUBLIC reusex_core reusex_geometry_common)

reusex_add_module(reusex_reconstruction ${REUSEX_RECONSTRUCTION_SOURCES})
target_link_libraries(reusex_reconstruction PUBLIC reusex_core reusex_geometry_common)

reusex_add_module(reusex_slam ${REUSEX_SLAM_SOURCES})
target_link_libraries(reusex_slam PUBLIC reusex_core)
# GTSAM — factor-graph pose-graph optimization, used only inside
# PlaneGraphOptimizer.cpp. PRIVATE: not part of any public header. As a static
# lib this still propagates for final linking.
target_link_libraries(reusex_slam PRIVATE gtsam)
# OpenCV feature matching (ORB + BFMatcher) backs the P2 loop-closure front-end
# (LoopClosure.cpp). PRIVATE: not part of any public header. opencv_core /
# opencv_imgproc already arrive via reusex_common.
target_link_libraries(reusex_slam PRIVATE opencv_features2d)
# NARROW EXCEPTION (#222): slam registration reuses segmentation's Surfel /
# surfel_extraction (PlaneGraphOptimizer, JointPairwiseRegistration operate on
# extracted surfels). Explicit peer edge slam -> segmentation, kept narrow.
target_link_libraries(reusex_slam PUBLIC reusex_segmentation)

# --- Layer 4 — visualize (optional) ----------------------------------------
if(REUSEX_VISUALIZE_SOURCES)
    reusex_add_module(reusex_visualize ${REUSEX_VISUALIZE_SOURCES})
    target_link_libraries(reusex_visualize PUBLIC reusex_core)
endif()

# ===============================================
# Umbrella target — backward compatible `reusex`
# ===============================================
# INTERFACE library linking every module, so existing consumers that link
# `reusex` and include <reusex/...> keep working unchanged.
add_library(reusex INTERFACE)
target_link_libraries(reusex INTERFACE
    reusex_common
    reusex_utils
    reusex_geometry_common
    reusex_core
    reusex_io
    reusex_vision
    reusex_segmentation
    reusex_reconstruction
    reusex_slam
)
if(TARGET reusex_visualize)
    target_link_libraries(reusex INTERFACE reusex_visualize)
endif()

# -----------------------------------------------
# Diagnostics
# -----------------------------------------------
foreach(mod utils geometry_common core io vision segmentation reconstruction slam visualize)
    if(TARGET reusex_${mod})
        get_target_property(_srcs reusex_${mod} SOURCES)
        list(LENGTH _srcs _n)
        message(STATUS "reusex module '${mod}': ${_n} source(s)")
    endif()
endforeach()

# -----------------------------------------------
# Export library information
# -----------------------------------------------
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REUSEX_LIBRARY_TARGET reusex PARENT_SCOPE)
    set(USE_MIP_SOLVER ${USE_MIP_SOLVER} PARENT_SCOPE)
    set(ENABLED_ML_BACKENDS ${ENABLED_ML_BACKENDS} PARENT_SCOPE)
endif()
