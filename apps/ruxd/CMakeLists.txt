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
# The web API encodes images with cv::imencode, which lives in imgcodecs — the
# reusex umbrella only carries core/imgproc/highgui, so name it explicitly.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Backend clients.
# libpqxx and redis++ are imported via pkg-config: libpqxx ships no CMake
# config, and redis++'s CMake config does find_dependency(hiredis) which the
# nixpkgs hiredis (pkg-config only) does not satisfy. Their .pc files pull in
# libpq / hiredis transitively.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBPQXX REQUIRED IMPORTED_TARGET libpqxx)
pkg_check_modules(REDISPP REQUIRED IMPORTED_TARGET redis++)
find_package(AWSSDK REQUIRED COMPONENTS s3)

# Collect sources. Everything except main() goes into static libraries so the
# logic is reachable from the unit tests; the executable is then just the CLI
# entry point on top of them. Two libraries, split by link weight (#268):
#
#   ruxd_api_lib  src/api/: the web GUI's REST + WebSocket API (formerly
#                 `rux gui`'s rux_gui_lib). Links reusex_core and
#                 reusex_pipeline ONLY      -> the LIGHT test binary
#                                              (tests/unit/ruxd_api/)
#   ruxd_lib      everything else: backend clients, auth, the
#                 legacy service routes, and the heavy pieces
#                 injected into the API (SAM3, renderer, ICP,
#                 optimize). Links the `reusex` umbrella
#                                           -> the HEAVY test binary
#                                              (tests/unit/ruxd/)
#
# Both are globbed, so a new file needs no CMake edit — but put it under
# src/api/ only if it links no further than core + pipeline.
file(GLOB_RECURSE ruxd_api_SOURCES CONFIGURE_DEPENDS "src/api/*.cpp")
file(GLOB_RECURSE ruxd_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM ruxd_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM ruxd_SOURCES ${ruxd_api_SOURCES})

# ---------------------------------------------------------------------------
# ruxd_api_lib -- the web API, light
# ---------------------------------------------------------------------------
add_library(ruxd_api_lib STATIC ${ruxd_api_SOURCES})
# PUBLIC: consumers (ruxd, rux, tests) include api/api.hpp, which pulls in the
# reusex pipeline headers and nlohmann_json. Crow stays PRIVATE — api/Server.hpp
# is pimpl'd precisely so crow.h never leaks past this target.
target_include_directories(ruxd_api_lib PUBLIC include)
# reusex_pipeline, NOT the `reusex` umbrella: the API server touches core
# (ProjectDB) and pipeline (the job runner) only, and the umbrella would drag
# reusex_vision -- libtorch, TensorRT -- into every consumer, including the
# light unit-test binary (#268). Anything heavier is injected by ruxd's main
# (see include/injected.hpp).
target_link_libraries(ruxd_api_lib
    PUBLIC
        reusex_core
        reusex_pipeline
        nlohmann_json::nlohmann_json
    PRIVATE
        Crow::Crow
        spdlog::spdlog
        Threads::Threads
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
)

add_library(ruxd_lib STATIC ${ruxd_SOURCES})
# PUBLIC: consumers (ruxd, tests) include auth.hpp/endpoints.hpp, which pull in
# Crow and nlohmann_json headers.
target_include_directories(ruxd_lib PUBLIC include)
target_link_libraries(ruxd_lib
    PUBLIC
        Crow::Crow
        nlohmann_json::nlohmann_json
        Threads::Threads
        # PUBLIC: connection_pool.hpp (a public header, used by the unit tests)
        # logs through spdlog.
        spdlog::spdlog
        ruxd_api_lib
        # PUBLIC: cli.hpp (ruxd::configure_cli, tested by tests/unit/ruxd)
        # takes a CLI::App.
        CLI11::CLI11
    PRIVATE
        reusex
        opencv_imgcodecs
        PkgConfig::LIBPQXX
        PkgConfig::REDISPP
        ${AWSSDK_LINK_LIBRARIES}
)

# main.cpp is `return ruxd::run(argc, argv);` (STANDARDS §1.1).
add_executable(ruxd src/main.cpp)
target_link_libraries(ruxd PRIVATE ruxd_lib reusex)

install(TARGETS ruxd DESTINATION bin)
