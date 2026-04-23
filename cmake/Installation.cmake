# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Installation Configuration
# ===============================================

include(GNUInstallDirs)

# -----------------------------------------------
# Install library (if target exists)
# -----------------------------------------------
if(TARGET reusex)
    install(TARGETS reusex
        EXPORT reusexTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# -----------------------------------------------
# Install executable (if target exists)
# -----------------------------------------------
if(TARGET rux)
    install(TARGETS rux
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif() 

# -----------------------------------------------
# Install headers
# -----------------------------------------------
install(DIRECTORY libs/reusex/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/reusex
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.cuh"
)

# Install external headers (CGAL, pcl, spdmon extensions)
install(DIRECTORY libs/reusex/extern/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/reusex
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/generated/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

# -----------------------------------------------
# Export targets
# -----------------------------------------------
install(EXPORT reusexTargets
    FILE reusexTargets.cmake
    NAMESPACE reusex::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/reusex
)

# -----------------------------------------------
# Generate and install CMake config files
# -----------------------------------------------
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/reusexConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/reusexConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/reusexConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/reusex
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/reusexConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/reusexConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/reusex
)

message(STATUS "Installation configured to ${CMAKE_INSTALL_PREFIX}")
