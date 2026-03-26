cmake_minimum_required(VERSION 3.17)
project(RuxCLI)

find_package(CLI11 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

# Collect sources
file(GLOB_RECURSE rux_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(rux ${rux_SOURCES})
target_include_directories(rux PRIVATE include)
target_link_libraries(rux PRIVATE
    ReUseX
    CLI11::CLI11
    spdlog::spdlog
    pcl_visualization
)

install(TARGETS rux DESTINATION bin)
