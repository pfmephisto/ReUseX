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
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


enable_language(CXX)
enable_language(C)


# ===============================================
# options

option(LIN_ENABLE_ASAN "if true, enables clang/MSVC address sanitizer" OFF)
option(LIN_ENABLE_MSAN "if true, enables clang/MSVC memory sanitizer" OFF)
option(LIN_ENABLE_UBSAN "if true, enables clang/MSVC undefined behaviour sanitizer" OFF)
option(LIN_ENABLE_TSAN "if true, enables clang/MSVC thread sanitizer" OFF)
option(OPENMP, "if true, enables ")

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

# Generate compile commands for clangd and nvim highlighting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===============================================
# Bin dir
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()


find_package(Boost CONFIG REQUIRED)
find_package(MPI REQUIRED)
find_package(TBB REQUIRED)
find_package(spdlog REQUIRED)
find_package(spdmon REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(SCIP REQUIRED)

find_package(CGAL REQUIRED Core)
include(CGAL_Eigen3_support)
include(CGAL_SCIP_support)
include(CGAL_TBB_support)

find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(embree REQUIRED)
get_property(importTargets DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
find_package(g2o REQUIRED)
get_property(importTargetsAfter DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
list(REMOVE_ITEM importTargetsAfter ${importTargets})
message("Listing TARGETS __________________________________")
message("${importTargetsAfter}")

find_package(opennurbs REQUIRED)

find_package(PCL REQUIRED)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
# find_package(pybind11 REQUIRED)


find_package(RTABMap REQUIRED)
find_package(GraphBLAS REQUIRED)
find_package(LAGraph REQUIRED)


# ===============================================
# configure executable


file(GLOB_RECURSE LIB_SOURCES
    "src/*.cc"
    "src/*.cpp"
    #"src/*.cu"
)

file(GLOB_RECURSE LIB_HEADERS
    "include/*.hh"
)

list(FILTER LIB_SOURCES EXCLUDE REGEX ".*py.cpp")

# Exclude python bindings
list(FILTER LIB_SOURCES EXCLUDE REGEX ".*bindings.*")
list(FILTER LIB_SOURCES EXCLUDE REGEX ".*ReUseX.cpp")
list(FILTER LIB_HEADERS EXCLUDE REGEX ".*bindings.*")


# group sources according to folder structure
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_SOURCES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_HEADERS})



# ===============================================
# main libary
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME}  PRIVATE 
	# pybind11::embed
    spdlog::spdlog
)
target_link_libraries(${PROJECT_NAME} PUBLIC

    OpenNURBS
    embree
    TBB::tbb

    Eigen3::Eigen

    CGAL::CGAL
    CGAL::Eigen3_support
    CGAL::SCIP_support
    CGAL::TBB_support

    rtabmap::rtabmap

    g2o::g2o_ceres_ad
    g2o::stuff
    g2o::opengl_helper
    g2o::core
    g2o::g2o_cli_library
    g2o::g2o_simulator_library
    g2o::types_slam2d
    g2o::types_slam2d_addons
    g2o::types_sclam2d
    g2o::types_slam3d
    g2o::types_slam3d_addons
    g2o::types_sba
    g2o::types_icp
    g2o::types_sim3
    g2o::solver_pcg
    g2o::solver_dense
    g2o::solver_slam2d_linear
    g2o::solver_structure_only
    g2o::solver_csparse
    g2o::csparse_extension
    g2o::solver_cholmod
    g2o::solver_eigen

    fmt
    mpfr
    MPI::MPI_CXX

    SuiteSparse::GraphBLAS
    SuiteSparse::LAGraph
    SuiteSparse::LAGraphX

    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    ${PCL_DEFINITIONS}
    SPDLOG_FMT_EXTERNAL
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARY_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


target_include_directories(${PROJECT_NAME} PUBLIC "src")
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(${PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})

if(NOT DEFINED CLI11_DIR)
set (CLI11_DIR "/opt/CLI11" CACHE STRING "CLI11 git repository")
endif()
include_directories(${CLI11_DIR}/include)

add_executable(reusex_rtabmap_export src/reusex_rtabmap_export.cpp)
target_link_libraries(reusex_rtabmap_export PRIVATE ${PROJECT_NAME})

add_executable(reusex_rhino_export src/reusex_rhino_export.cpp)
target_link_libraries(reusex_rhino_export PRIVATE ${PROJECT_NAME})

add_executable(reusex_plane_segmentation src/reusex_plane_segmentation.cpp)
target_link_libraries(reusex_plane_segmentation PRIVATE ${PROJECT_NAME})

add_executable(reusex_polygonal_surface_reconstruction src/reusex_polygonal_surface_reconstruction.cpp)
target_link_libraries(reusex_polygonal_surface_reconstruction PRIVATE ${PROJECT_NAME})

add_executable(reusex_markov_clustering src/reusex_markov_clustering.cpp)
target_link_libraries(reusex_markov_clustering PRIVATE ${PROJECT_NAME})

# ===============================================
# # scikit_build
# Python_add_library(_core MODULE src/ReUseX.cpp WITH_SOABI)
# target_link_libraries(_core PRIVATE pybind11::headers ${PROJECT_NAME})
# 
# # This is passing in the version as a define just as an example
# target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
#install(TARGETS _core DESTINATION ReUseX)

