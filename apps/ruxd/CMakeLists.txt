# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.17)
project(RuxDaemon)

find_package(Crow CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

# Backend clients.
# libpqxx and redis++ are imported via pkg-config: libpqxx ships no CMake
# config, and redis++'s CMake config does find_dependency(hiredis) which the
# nixpkgs hiredis (pkg-config only) does not satisfy. Their .pc files pull in
# libpq / hiredis transitively.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBPQXX REQUIRED IMPORTED_TARGET libpqxx)
pkg_check_modules(REDISPP REQUIRED IMPORTED_TARGET redis++)
find_package(AWSSDK REQUIRED COMPONENTS s3)

# Collect sources. Everything except main() goes into a static library so the
# routing/auth logic is reachable from the unit tests (tests/unit/ruxd/); the
# executable is then just the CLI entry point on top of it.
file(GLOB_RECURSE ruxd_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM ruxd_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

add_library(ruxd_lib STATIC ${ruxd_SOURCES})
# PUBLIC: consumers (ruxd, tests) include auth.hpp/endpoints.hpp, which pull in
# Crow and nlohmann_json headers.
target_include_directories(ruxd_lib PUBLIC include)
target_link_libraries(ruxd_lib
    PUBLIC
        Crow::Crow
        nlohmann_json::nlohmann_json
        Threads::Threads
    PRIVATE
        reusex
        PkgConfig::LIBPQXX
        PkgConfig::REDISPP
        ${AWSSDK_LINK_LIBRARIES}
)

add_executable(ruxd src/main.cpp)
target_link_libraries(ruxd PRIVATE
    ruxd_lib
    reusex
    CLI11::CLI11
    spdlog::spdlog
)

install(TARGETS ruxd DESTINATION bin)
