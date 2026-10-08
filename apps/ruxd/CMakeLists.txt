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
find_package(SQLite3 REQUIRED)
# The web API encodes images with cv::imencode, which lives in imgcodecs — the
# reusex umbrella only carries core/imgproc/highgui, so name it explicitly.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
# Password hashing (argon2id via EVP_KDF, OpenSSL >= 3.2), session tokens and
# SHA-256 (src/api/credentials.cpp).
find_package(OpenSSL 3.2 REQUIRED COMPONENTS Crypto)

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
#   ruxd_pg_lib   src/pg/: server mode's Postgres stores, migrations and
#                 `ruxd admin`. Links ruxd_api_lib + libpqxx
#                                           -> the LIGHT test binary
#                                              (tests/unit/ruxd_pg/)
#   ruxd_lib      everything else: backend clients, the server and local
#                 entry points, the CLI (ruxd::run), and the heavy pieces
#                 injected into the API (SAM3, renderer, ICP,
#                 optimize). Links the `reusex` umbrella
#                                           -> the HEAVY test binary
#                                              (tests/unit/ruxd/)
#
#   ruxd_cli_lib  src/cli/: the option set and post-parse defaults
#                 (configure_cli, finish_invocation). Links ruxd_api_lib +
#                 CLI11 only        -> the LIGHT test binary
#                                      (tests/unit/ruxd_cli/)
#
# All are globbed, so a new file needs no CMake edit — but put it under
# src/api/ or src/cli/ only if it links no further than core + pipeline.
file(GLOB_RECURSE ruxd_api_SOURCES CONFIGURE_DEPENDS "src/api/*.cpp")
file(GLOB_RECURSE ruxd_cli_SOURCES CONFIGURE_DEPENDS "src/cli/*.cpp")
file(GLOB_RECURSE ruxd_pg_SOURCES CONFIGURE_DEPENDS "src/pg/*.cpp")
file(GLOB_RECURSE ruxd_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
list(REMOVE_ITEM ruxd_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM ruxd_SOURCES ${ruxd_api_SOURCES} ${ruxd_cli_SOURCES}
     ${ruxd_pg_SOURCES})

# Crow buffers request bodies with no limit of its own; overlays/crow.nix
# patches in CROW_MAX_REQUEST_BODY. It must have ONE value in every TU that
# includes Crow (the parser is inline code), so it is a PUBLIC definition of
# the API library, which everything that includes Crow links. 72 MiB: the
# largest upload chunk (64 MiB, UploadLimits::max_chunk_bytes) plus headroom.
set(RUXD_MAX_REQUEST_BODY 75497472)

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
target_compile_definitions(ruxd_api_lib PUBLIC
    CROW_MAX_REQUEST_BODY=${RUXD_MAX_REQUEST_BODY})
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
        # cases.cpp checks an upload is a ReUseX project before adopting it.
        SQLite3::SQLite3
        # credentials.cpp: argon2id, random tokens, SHA-256.
        OpenSSL::Crypto
)

# ---------------------------------------------------------------------------
# ruxd_cli_lib -- the command line's parsing rules, light
# ---------------------------------------------------------------------------
add_library(ruxd_cli_lib STATIC ${ruxd_cli_SOURCES})
target_include_directories(ruxd_cli_lib PUBLIC include)
# PUBLIC: cli.hpp takes a CLI::App and names api::ServerOptions.
target_link_libraries(ruxd_cli_lib
    PUBLIC
        ruxd_api_lib
        CLI11::CLI11
    PRIVATE
        spdlog::spdlog
)

# ---------------------------------------------------------------------------
# ruxd_pg_lib -- server mode's Postgres stores, migrations and `ruxd admin`
# ---------------------------------------------------------------------------
# Light: ruxd_api_lib + libpqxx, so tests/unit/ruxd_pg runs in the light test
# binary. The schema (migrations/NNN_*.sql) is embedded into a generated
# source, regenerated whenever a migration file changes.
file(GLOB ruxd_MIGRATIONS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/migrations/*.sql")
set(ruxd_MIGRATIONS_CPP "${CMAKE_CURRENT_BINARY_DIR}/migrations_embedded.cpp")
add_custom_command(
    OUTPUT "${ruxd_MIGRATIONS_CPP}"
    COMMAND "${CMAKE_COMMAND}"
            -DMIGRATIONS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/migrations
            -DOUTPUT=${ruxd_MIGRATIONS_CPP}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedMigrations.cmake"
    DEPENDS ${ruxd_MIGRATIONS}
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedMigrations.cmake"
    COMMENT "Embedding ruxd's Postgres migrations"
    VERBATIM)
add_library(ruxd_pg_lib STATIC ${ruxd_pg_SOURCES} "${ruxd_MIGRATIONS_CPP}")
target_include_directories(ruxd_pg_lib PUBLIC include)
target_link_libraries(ruxd_pg_lib
    PUBLIC
        ruxd_api_lib
        # PUBLIC: pg/database.hpp holds a ConnectionPool (connection_pool.hpp
        # logs through spdlog); callers of Database::with() dereference pqxx.
        PkgConfig::LIBPQXX
        spdlog::spdlog
)

add_library(ruxd_lib STATIC ${ruxd_SOURCES})
# PUBLIC: consumers (ruxd, tests) include clients.hpp / connection_pool.hpp
# and the CLI and server headers.
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
        ruxd_cli_lib
        ruxd_pg_lib
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
