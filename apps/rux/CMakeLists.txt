# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.17)
project(RuxCLI)

find_package(CLI11 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
# `rux gui` (#265) reuses the same Crow build as apps/ruxd. Image endpoints
# encode with cv::imencode, which lives in imgcodecs — the reusex umbrella only
# carries core/imgproc/highgui, so name it explicitly here.
find_package(Crow CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Collect sources. The GUI server goes into its own static library, mirroring
# the ruxd_lib split, so tests/unit/rux_gui can exercise the route table and the
# JSON serialization without standing up a socket.
file(GLOB_RECURSE rux_gui_SOURCES CONFIGURE_DEPENDS "src/gui/*.cpp")
file(GLOB_RECURSE rux_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM rux_SOURCES ${rux_gui_SOURCES})

add_library(rux_gui_lib STATIC ${rux_gui_SOURCES})
# PUBLIC: consumers (rux, tests) include gui/api.hpp, which pulls in the reusex
# pipeline headers and nlohmann_json. Crow stays PRIVATE — gui/Server.hpp is
# pimpl'd precisely so crow.h never leaks past this target.
target_include_directories(rux_gui_lib PUBLIC include)
# reusex_pipeline, NOT the `reusex` umbrella: the GUI server touches core
# (ProjectDB) and pipeline (the job runner) only, and the umbrella would drag
# reusex_vision -- libtorch, TensorRT -- into every consumer, including the
# light unit-test binary (#268).
target_link_libraries(rux_gui_lib
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

add_executable(rux ${rux_SOURCES})
target_include_directories(rux PRIVATE include)
target_link_libraries(rux PRIVATE
    rux_gui_lib
    reusex
    CLI11::CLI11
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    pcl_visualization
    # cv::imwrite, used by `rux render` to write the headless render (#294)
    opencv_imgcodecs
)

install(TARGETS rux DESTINATION bin)
