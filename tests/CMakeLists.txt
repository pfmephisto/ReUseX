# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Unit Tests with Catch2 v3
# ===============================================

message(STATUS "Configuring unit tests...")

# -----------------------------------------------
# Find dependencies
# -----------------------------------------------
find_package(Catch2 3 REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)

# Note: ENABLE_COVERAGE option is now set in main CMakeLists.txt
# Coverage flags are automatically inherited from parent scope

# -----------------------------------------------
# Collect test source files from unit/ directory
# -----------------------------------------------
file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp")

# -----------------------------------------------
# Create unit test executable
# -----------------------------------------------
add_executable(reusex_unit_tests ${UNIT_TEST_SOURCES})

# Link against reusex library and Catch2
target_link_libraries(reusex_unit_tests
    PRIVATE
        reusex
        Catch2::Catch2WithMain
        fmt::fmt
        OpenSSL::Crypto
)

# Include ReUseX headers from new location
target_include_directories(reusex_unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/reusex/include
    ${CMAKE_BINARY_DIR}/generated
)

# -----------------------------------------------
# Integration tests (future)
# -----------------------------------------------
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")

if(INTEGRATION_TEST_SOURCES)
    add_executable(reusex_integration_tests ${INTEGRATION_TEST_SOURCES})
    target_link_libraries(reusex_integration_tests
        PRIVATE
            reusex
            Catch2::Catch2WithMain
            fmt::fmt
    )
    target_include_directories(reusex_integration_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )
endif()

# -----------------------------------------------
# Enable CTest and discover tests
# -----------------------------------------------
include(CTest)
include(Catch)
catch_discover_tests(reusex_unit_tests)

if(INTEGRATION_TEST_SOURCES)
    catch_discover_tests(reusex_integration_tests)
endif()

message(STATUS "Unit tests configured successfully")
