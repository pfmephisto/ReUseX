# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.17)
project(RuxCLI)

find_package(CLI11 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Collect sources. Nothing but `main()` lives in the executable (#249); the app
# logic is split across two static libraries so ctest can link it:
#
#   rux_core_lib  app logic with no viewer and no ML closure -> the LIGHT test
#                 binary (tests/unit/rux/)
#   rux_lib       everything else: subcommands, routers, the
#                 interactive viewer, rux::run()             -> the HEAVY test
#                 binary (tests/unit/rux_app/)
#
# The web GUI backend that used to live here as rux_gui_lib is now
# ruxd_api_lib in the standalone `ruxd` repo (`ruxd --local`); `rux gui`
# is gone.
#
# Only rux_core_lib is an explicit source list; rux_lib is globbed, so a new
# subcommand still needs no CMake edit.

# ---------------------------------------------------------------------------
# rux_core_lib -- app logic that needs neither the viewer nor the ML backends
# ---------------------------------------------------------------------------
# These files depend on reusex_core and the standard library, nothing more.
# Keeping them in their own target is what lets their tests live in
# `reusex_unit_tests`, which does not pay libtorch/TensorRT's ~0.6 s of
# dynamic-loader time per test process (#268). A file belongs here only if it
# genuinely links no further than reusex_core -- when in doubt, leave it in
# rux_lib, where the glob picks it up for free.
set(rux_core_SOURCES
    src/database/format_handler.cpp
    src/database/input_handler.cpp
    src/database/path_parser.cpp
    src/filter_utils.cpp
    src/stage_prerequisites.cpp
)

add_library(rux_core_lib STATIC ${rux_core_SOURCES})
target_include_directories(rux_core_lib PUBLIC include)
target_link_libraries(rux_core_lib
    PUBLIC
        # filter_utils.hpp and stage_prerequisites.hpp expose ProjectDB and the
        # reusex point-cloud aliases to their callers.
        reusex_core
    PRIVATE
        spdlog::spdlog
)

# ---------------------------------------------------------------------------
# rux_lib -- the rest of the CLI, everything except main()
# ---------------------------------------------------------------------------
file(GLOB_RECURSE rux_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
# src/main.cpp defines `main`, which cannot be linked into a Catch2 binary that
# supplies its own. That single symbol is the entire reason for the split; the
# CLI wiring it used to sit next to is now rux::run() in src/rux.cpp, inside
# this library.
list(REMOVE_ITEM rux_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
foreach(rux_core_source IN LISTS rux_core_SOURCES)
    list(REMOVE_ITEM rux_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${rux_core_source}")
endforeach()

add_library(rux_lib STATIC ${rux_SOURCES})
target_include_directories(rux_lib PUBLIC include)
target_link_libraries(rux_lib
    PUBLIC
        rux_core_lib
        reusex
        CLI11::CLI11
        spdlog::spdlog
        nlohmann_json::nlohmann_json
    PRIVATE
        # `rux view` is an interactive UI, so it lives in the app layer
        # (STANDARDS §1) and links the PCL viewer directly. PRIVATE still
        # reaches the final executable link -- CMake records a static library's
        # private deps as $<LINK_ONLY:...> on its interface -- it just keeps the
        # viewer headers off consumers' include paths.
        pcl_visualization
        # cv::imwrite, used by `rux render` to write the headless render (#294)
        opencv_imgcodecs
)

add_executable(rux src/main.cpp)
target_link_libraries(rux PRIVATE rux_lib)

install(TARGETS rux DESTINATION bin)

# The native Qt client (Stream Q): rux_qt_core, rux_qt_lib, rux-qt-gallery.
# BUILD_QT_CLIENT is defined in the top-level CMakeLists.txt.
# Plain `rux` (no subcommand) launches the Qt client through rux_qt_lib;
# rux_qt_core alone decides whether to (launch.hpp), so a build without the
# client still prints help instead of doing nothing.
# apps/rux/qt always defines rux_qt_core (standard library only) and the Qt
# targets only under BUILD_QT_CLIENT.
add_subdirectory(qt)
# PUBLIC: main.cpp and tests/unit/rux_app consult rux_qt/background.hpp
# (detached Qt-client work that outlives the window); it is std-only.
target_link_libraries(rux_lib PUBLIC rux_qt_core)
# The Qt client goes on the executable only: main.cpp hands rux::run a
# launcher (rux_app.hpp, GuiLauncher). Linking it into rux_lib would drag Qt
# Widgets + VTK GUISupportQt into reusex_unit_tests_vision (#268).
if(BUILD_QT_CLIENT)
    target_link_libraries(rux PRIVATE rux_qt_lib)
    target_compile_definitions(rux PRIVATE RUX_HAVE_QT_CLIENT)
endif()
