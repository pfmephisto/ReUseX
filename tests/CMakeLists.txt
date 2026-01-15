# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Unit Tests with Catch2 v3
# ===============================================

message(STATUS "Configuring unit tests...")

# -----------------------------------------------
# Find Catch2
# -----------------------------------------------
find_package(Catch2 3 REQUIRED)

# -----------------------------------------------
# Collect test source files
# -----------------------------------------------
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# -----------------------------------------------
# Create test executable
# -----------------------------------------------
add_executable(reusex_tests ${TEST_SOURCES})

# Link against ReUseX library and Catch2
target_link_libraries(reusex_tests 
    PRIVATE 
        ${PROJECT_NAME}
        Catch2::Catch2WithMain
        fmt::fmt
)

# Include ReUseX headers
target_include_directories(reusex_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated
)

# -----------------------------------------------
# Enable CTest and discover tests
# -----------------------------------------------
include(CTest)
include(Catch)
catch_discover_tests(reusex_tests)

message(STATUS "Unit tests configured successfully")
