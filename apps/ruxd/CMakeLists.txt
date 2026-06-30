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

# Collect sources
file(GLOB_RECURSE ruxd_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(ruxd ${ruxd_SOURCES})
target_include_directories(ruxd PRIVATE include)
target_link_libraries(ruxd PRIVATE
    reusex
    Crow::Crow
    CLI11::CLI11
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    Threads::Threads
    PkgConfig::LIBPQXX
    PkgConfig::REDISPP
    ${AWSSDK_LINK_LIBRARIES}
)

install(TARGETS ruxd DESTINATION bin)
