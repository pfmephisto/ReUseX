# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Rux CLI Executable Configuration
# ===============================================

# -----------------------------------------------
# Collect source files
# -----------------------------------------------
file(GLOB_RECURSE rux_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/rux/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/rux/**/*.cpp")

file(GLOB_RECURSE rux_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/rux/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/rux/**/*.hpp")

# -----------------------------------------------
# Find CLI11 for command-line parsing
# -----------------------------------------------
find_package(CLI11 CONFIG REQUIRED) 

# -----------------------------------------------
# Create executable target
# -----------------------------------------------
add_executable(rux src/rux/rux.cpp ${rux_SOURCES} ${rux_HEADERS})

# Include directories
target_include_directories(rux PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# -----------------------------------------------
# Link dependencies
# -----------------------------------------------
target_link_libraries(rux 
    PUBLIC 
        CLI11::CLI11 
    PRIVATE 
        ${PROJECT_NAME} 
        spdlog::spdlog
)

# Link visualization library if enabled
if(BUILD_VISUALIZATION)
    target_link_libraries(rux PRIVATE ${PROJECT_NAME}_visualization)
    target_compile_definitions(rux PRIVATE REUSEX_HAS_VISUALIZATION)
endif()

# -----------------------------------------------
# Compiler options
# -----------------------------------------------
target_link_libraries(rux PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(rux PUBLIC ${COMMON_COMPILER_FLAGS})
