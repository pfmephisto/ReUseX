# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Installation Configuration
# ===============================================

include(GNUInstallDirs)

# -----------------------------------------------
# Install library
# -----------------------------------------------
install(TARGETS ReUseX
    EXPORT ReUseXTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# -----------------------------------------------
# Install executable
# -----------------------------------------------
install(TARGETS rux
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
) 

# -----------------------------------------------
# Install headers
# -----------------------------------------------
install(DIRECTORY include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/generated/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp"
)

# -----------------------------------------------
# Export targets
# -----------------------------------------------
install(EXPORT ReUseXTargets
    FILE ReUseXTargets.cmake
    NAMESPACE ReUseX::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)

# -----------------------------------------------
# Generate and install CMake config files
# -----------------------------------------------
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ReUseXConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)

message(STATUS "Installation configured to ${CMAKE_INSTALL_PREFIX}")
