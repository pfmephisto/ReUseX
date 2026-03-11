# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Python Bindings
# ===============================================

cmake_minimum_required(VERSION 3.17)

# Check if building as part of main project or standalone
if(NOT TARGET ReUseX)
    # Building Python bindings standalone via scikit-build-core
    project(ReUseXPython)

    # Add parent project (core library)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../libs/reusex
                     ${CMAKE_BINARY_DIR}/reusex)
endif()

# ===============================================
# Generate Python version file
# ===============================================

# Get the project version from parent
if(NOT PROJECT_VERSION)
    set(PROJECT_VERSION ${ReUseX_VERSION})
endif()

# Configure version file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/_version.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/reusex/_version.py
    @ONLY
)

# ===============================================
# Install Python package files
# ===============================================

# Install the generated version file
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/reusex/_version.py
    DESTINATION reusex
)

# Install the package __init__.py
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/reusex/__init__.py
    DESTINATION reusex
)

# ===============================================
# Future: pybind11 module
# ===============================================
# When implementing actual bindings, add:
#
# find_package(pybind11 CONFIG REQUIRED)
#
# pybind11_add_module(_reusex
#     src/bindings.cpp
#     # ... other source files
# )
#
# target_link_libraries(_reusex PRIVATE ReUseX)
#
# install(TARGETS _reusex
#     LIBRARY DESTINATION reusex
# )

message(STATUS "Python bindings: Version file generation configured")
message(STATUS "Python bindings: pybind11 module not yet implemented")
