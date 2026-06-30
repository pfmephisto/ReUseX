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
)

install(TARGETS ruxd DESTINATION bin)
