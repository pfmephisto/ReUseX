# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Installation: the ReUseX CMake package
# ===============================================
#
# Installs the library as a relocatable CMake package so an external project
# (ruxd, after the repo split) can write
#
#     find_package(ReUseX CONFIG REQUIRED)
#     target_link_libraries(app PRIVATE ReUseX::pipeline)
#
# Layout under the install prefix:
#
#     include/reusex/...           public headers  (#include <reusex/core/...>)
#     include/reusex/extern/...    vendored CGAL/PCL extension headers
#     lib/libreusex_<module>.a     one static library per module
#     lib/cmake/ReUseX/            ReUseXConfig.cmake, ReUseXConfigVersion.cmake,
#                                  ReUseXTargets*.cmake
#
# Everything the package needs is in the `ReUseX_Development` install
# component, so `cmake --install build --component ReUseX_Development` installs
# the library without the executables (tests/package uses exactly that).
# Executables (rux, ruxd) install from their own CMakeLists.txt.

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(REUSEX_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX)
set(REUSEX_INSTALL_COMPONENT ReUseX_Development)

# -----------------------------------------------
# Targets
# -----------------------------------------------
# REUSEX_PACKAGE_TARGETS comes from libs/reusex/cmake/reusexLibrary.cmake,
# which also sets each target's EXPORT_NAME (reusex_core -> ReUseX::core).
# `reusex` is an INTERFACE umbrella whose INTERFACE_LINK_LIBRARIES point at the
# per-module static libraries, and every module links the INTERFACE helpers
# reusex_common / reusex_private_deps, so all of them must be in the same export
# set or install(EXPORT) fails at generate time.
install(TARGETS ${REUSEX_PACKAGE_TARGETS}
    EXPORT ReUseXTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ${REUSEX_INSTALL_COMPONENT}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ${REUSEX_INSTALL_COMPONENT}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${REUSEX_INSTALL_COMPONENT}
)

# -----------------------------------------------
# Headers
# -----------------------------------------------
# libs/reusex/include/<module>/... -> include/reusex/<module>/..., matching the
# build tree's build/include/reusex symlink, so <reusex/...> works unchanged.
install(DIRECTORY libs/reusex/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/reusex
    COMPONENT ${REUSEX_INSTALL_COMPONENT}
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.cuh"
)

# Vendored CGAL / PCL extension headers. Public headers include them by their
# upstream-style path (<pcl/planar_region_growing.hpp>), so they get their own
# include root (added to reusex_common's INSTALL_INTERFACE).
install(DIRECTORY libs/reusex/extern/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/reusex/extern
    COMPONENT ${REUSEX_INSTALL_COMPONENT}
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
)

# Generated headers: build/generated/reusex/{core/version.hpp, vision/sam3/...}
install(DIRECTORY ${CMAKE_BINARY_DIR}/generated/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT ${REUSEX_INSTALL_COMPONENT}
    FILES_MATCHING PATTERN "*.hpp"
)

# -----------------------------------------------
# Export set + package config
# -----------------------------------------------
install(EXPORT ReUseXTargets
    FILE ReUseXTargets.cmake
    NAMESPACE ReUseX::
    DESTINATION ${REUSEX_INSTALL_CMAKEDIR}
    COMPONENT ${REUSEX_INSTALL_COMPONENT}
)

# Build-time facts the config needs to reproduce the dependency lookups. Static
# libraries carry their PRIVATE deps as $<LINK_ONLY:...>, so every one of those
# imported targets must exist in the consumer too — including the optional ML
# backends, the MIP solver and the CUDA-only modules, which depend on what this
# particular build found.
set(REUSEX_PKG_WITH_CUDA ${WITH_CUDA})
set(REUSEX_PKG_ML_BACKENDS "${ENABLED_ML_BACKENDS}")
set(REUSEX_PKG_MIP_SOLVER ${USE_MIP_SOLVER})
set(REUSEX_PKG_HAVE_GSPLAT ${REUSEX_HAVE_GSPLAT})
if(TARGET reusex_visualize)
    set(REUSEX_PKG_HAVE_VISUALIZE ON)
else()
    set(REUSEX_PKG_HAVE_VISUALIZE OFF)
endif()

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ReUseXConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    INSTALL_DESTINATION ${REUSEX_INSTALL_CMAKEDIR}
)

# 0.x: a minor bump may break the API, so only the same major.minor matches.
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMinorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    DESTINATION ${REUSEX_INSTALL_CMAKEDIR}
    COMPONENT ${REUSEX_INSTALL_COMPONENT}
)

message(STATUS "Installation configured to ${CMAKE_INSTALL_PREFIX} "
               "(CMake package: ${REUSEX_INSTALL_CMAKEDIR}/ReUseXConfig.cmake)")
