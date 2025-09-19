# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.17)
if (NOT SKBUILD_PROJECT_VERSION)
    set(SKBUILD_PROJECT_VERSION 0.0.1)
endif()
project(ReUseX VERSION ${SKBUILD_PROJECT_VERSION} LANGUAGES CXX)


# ===============================================
# global settings

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile commands for clangd and nvim highlighting
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON) # Suppress CGAL debug/release warning 
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE) # Needed for dynamic linked spdlog
set(SPDLOG_FMT_EXTERNAL ON)

enable_language(CXX)
enable_language(C)

# ===============================================
# options

option(LIN_ENABLE_ASAN "if true, enables clang/MSVC address sanitizer" OFF)
option(LIN_ENABLE_MSAN "if true, enables clang/MSVC memory sanitizer" OFF)
option(LIN_ENABLE_UBSAN "if true, enables clang/MSVC undefined behaviour sanitizer" OFF)
option(LIN_ENABLE_TSAN "if true, enables clang/MSVC thread sanitizer" OFF)

if (LIN_ENABLE_ASAN AND LIN_ENABLE_TSAN)
    message(FATAL_ERROR "Can only enable one of TSan or ASan at a time")
endif()
if (LIN_ENABLE_ASAN AND LIN_ENABLE_MSAN)
    message(FATAL_ERROR "Can only enable one of ASan or MSan at a time")
endif()

option(LIN_ENABLE_WERROR "if true, enables -Werror, /WX" OFF)



# ===============================================
# compiler and linker flags

set(COMMON_COMPILER_FLAGS "")
set(COMMON_LINKER_FLAGS "")

if (MSVC)
    list(APPEND COMMON_COMPILER_FLAGS /MP)
    list(APPEND COMMON_COMPILER_FLAGS /openmp)

    if (LIN_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS /WX)
    endif()
else()
    list(APPEND COMMON_COMPILER_FLAGS -Wall -Wextra)
    
    # Add support for OpenMP
    list(APPEND COMMON_COMPILER_FLAGS -fopenmp -lgomp)

    if (LIN_ENABLE_WERROR)
        # list(APPEND COMMON_COMPILER_FLAGS -Werror)
        list(APPEND COMMON_COMPILER_FLAGS -Wno-unused-variable -Wno-unused-function)
    endif()


    if (LIN_ENABLE_ASAN OR LIN_ENABLE_TSAN OR LIN_ENABLE_MSAN OR LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fno-omit-frame-pointer -g)
        list(APPEND COMMON_LINKER_FLAGS -fno-omit-frame-pointer -g)
    endif()

    if (LIN_ENABLE_ASAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=address)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=address)
    endif()

    if (LIN_ENABLE_TSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=thread)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=thread)
    endif()

    if (LIN_ENABLE_MSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=memory)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=memory)
    endif()

    if (LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
        list(APPEND COMMON_LINKER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
    endif()
endif()



# ===============================================
# Bin dir
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()


#find_package(Qt6 COMPONENTS Core Widgets Gui OpenGL REQUIRED) 
find_package(Boost CONFIG REQUIRED)
find_package(TBB REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
# find_package(spdmon REQUIRED)
 
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(RTABMap REQUIRED)
find_package(GraphBLAS REQUIRED)
find_package(LAGraph REQUIRED)
find_package(PCL REQUIRED)
 
find_package(Eigen3 REQUIRED)
find_package(SCIP REQUIRED)
find_package(embree REQUIRED)
 

find_package(CGAL REQUIRED Core) #OPTIONAL_COMPONENTS Qt6
include(CGAL_Eigen3_support)
include(CGAL_SCIP_support)
include(CGAL_TBB_support)
 
 
 
find_package(opennurbs REQUIRED)
 
# option(GUI_ENABLED "if true, enables GUI" OFF)
# if (GUI_ENABLED)
# 	set(CMAKE_AUTOMOC ON)
#         set(CMAKE_AUTORCC ON)
#         set(CMAKE_AUTOUIC ON)
# 	add_definitions(-DCGAL_USE_BASIC_VIEWER)
# endif()



# ===============================================
# Library sources
set(REUSEX_SOURCES
    src/ReUseX/io.cpp
    src/ReUseX/Yolo.cpp
)

set(REUSEX_HEADERS
    include/ReUseX/about.hpp.in
    include/ReUseX/filter.hpp
    include/ReUseX/fmt_formatter.hpp
    include/ReUseX/io.hpp
    include/ReUseX/split.hpp
    include/ReUseX/straiten.hpp
    include/ReUseX/Yolo.hpp
    include/pcl/markov_clustering.hpp
    include/pcl/planar_region_growing.hpp
    include/pcl/polygonal_surface_reconstruction.hpp
    include/spdmon/spdmon.hpp
) 

# Replace version and license in about.hpp file
file(READ ${CMAKE_SOURCE_DIR}/LICENSE.md LICENSE_TEXT)
string(REPLACE "\"" "\\\"" LICENSE_TEXT "${LICENSE_TEXT}")
configure_file(
    ${CMAKE_SOURCE_DIR}/include/ReUseX/about.hpp.in
    ${CMAKE_BINARY_DIR}/generated/ReUseX/about.hpp
    @ONLY
)


# ===============================================
# ReUseX library
add_library(${PROJECT_NAME} SHARED ${REUSEX_SOURCES} ${REUSEX_HEADERS}) #STATIC #SHARED
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC

	CGAL::CGAL
        CGAL::Eigen3_support
        CGAL::SCIP_support
        CGAL::TBB_support

        OpenNURBS

	pcl_common
        pcl_io
        pcl_filters
        pcl_segmentation
        pcl_registration
        pcl_visualization
        embree
        # TBB::tbb

        Eigen3::Eigen

        # mpfr
        # MPI::MPI_CXX

	SuiteSparse::GraphBLAS
        SuiteSparse::LAGraph
        SuiteSparse::LAGraphX

	opencv_core 
        opencv_imgproc
        opencv_highgui

        rtabmap::rtabmap

    PRIVATE 
        #fmt::fmt  #fmt-header-onlyonly
	fmt::fmt-header-only
        spdlog::spdlog
        # Move Library to Private as their stops being used in the public interface
)

target_compile_definitions(${PROJECT_NAME} PUBLIC SPDLOG_FMT_EXTERNAL)
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(${PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})


# ===============================================
# rux executable
# ===============================================

set(rux_SOURCES
    src/rux/annotate.cpp
    src/rux/cellcomplex.cpp
    src/rux/export.cpp
    src/rux/seg-planes.cpp
    src/rux/seg-rooms.cpp
    src/rux/surface_reconstruction.cpp
)

set(rux_HEADERS
    include/rux/annotate.hpp
    include/rux/cellcomplex.hpp
    include/rux/export.hpp
    include/rux/seg-planes.hpp
    include/rux/seg-rooms.hpp
    include/rux/surface_reconstruction.hpp
) 

find_package(CLI11 CONFIG REQUIRED) 

add_executable(rux src/rux/rux.cpp ${rux_SOURCES} ${rux_HEADERS})
target_include_directories(rux PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(rux 
	PUBLIC 
	  CLI11::CLI11 
	  ${PROJECT_NAME} 
	PRIVATE 
	  fmt::fmt-header-only
	  spdlog::spdlog
  )
target_link_libraries(rux PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(rux PUBLIC ${COMMON_COMPILER_FLAGS})
target_compile_definitions(rux PRIVATE SPDLOG_FMT_EXTERNAL)
set_target_properties(rux PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# ===============================================
# # Python module
# ===============================================
# find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
# find_package(pybind11 REQUIRED)
# # scikit_build
# Python_add_library(_core MODULE src/ReUseX.cpp WITH_SOABI)
# target_link_libraries(_core PRIVATE pybind11::headers ${PROJECT_NAME})
# 
# # This is passing in the version as a define just as an example
# target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
#install(TARGETS _core DESTINATION ReUseX)




# ===============================================
# Installation & Export
# ===============================================
include(GNUInstallDirs)

# Library
install(TARGETS ReUseX # rux # optinal 
    EXPORT ReUseXTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Binary
install(TARGETS rux
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
) 

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ReUseXTargets
    FILE ReUseXTargets.cmake
    NAMESPACE ReUseX::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ReUseXConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ReUseXConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ReUseX
)
