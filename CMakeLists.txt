cmake_minimum_required(VERSION 3.10)
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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()


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


# find_package(Gurobi REQUIRED)
find_path(GUROBI_INCLUDE_DIRS
    NAMES gurobi_c.h
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES include)

find_library(GUROBI_LIBRARY
    NAMES gurobi gurobi120 
    # NAMES gurobi120 libgurobi_c++
    # NAMES gurobi gurobi110 libgurobi110 libgurobi_c++ 
    # NAMES gurobi_c++ gurobi_c
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES lib)
message(STATUS "GUROBI_LIBRARY: ${GUROBI_LIBRARY}")


find_library(GUROBI_CXX_LIBRARY
    NAMES gurobi_c++
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES lib)
set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})

find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY)
# add_compile_definitions(CGAL_USE_GUROBI=1)


find_package(Eigen3 REQUIRED)
find_package(SCIP REQUIRED)

find_package(CGAL REQUIRED Core)
include(CGAL_Eigen3_support)
include(CGAL_SCIP_support)
include(CGAL_TBB_support)

find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(embree REQUIRED)
find_package(g2o REQUIRED)

find_package(xtensor REQUIRED)
find_package(xtensor-io REQUIRED)
find_package(HighFive REQUIRED)
find_package(opennurbs REQUIRED)

find_package(PCL REQUIRED)
find_package(polyscope REQUIRED)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 REQUIRED)




# ===============================================
# configure executable


file(GLOB_RECURSE LIB_SOURCES
    "src/*.cc"
    "src/*.cpp"
)

file(GLOB_RECURSE LIB_HEADERS
    "src/*.hh"
)

list(FILTER LIB_SOURCES EXCLUDE REGEX ".*py.cpp")


# group sources according to folder structure
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_SOURCES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_HEADERS})



# ===============================================
# main libary
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)



target_link_libraries(${PROJECT_NAME}  PRIVATE 
    pybind11::embed
    optimized ${GUROBI_CXX_LIBRARY}
    debug ${GUROBI_CXX_DEBUG_LIBRARY}
)
target_link_libraries(${PROJECT_NAME} PUBLIC

    HighFive
    xtensor
    xtensor-io
    
    OpenNURBS
    embree
    polyscope

    Eigen3::Eigen

    CGAL::CGAL
    CGAL::Eigen3_support
    CGAL::SCIP_support
    CGAL::TBB_support


    spdlog


    fmt
    mpfr
    MPI::MPI_CXX

    ${OpenCV_LIBS}
    ${GUROBI_LIBRARY}
    ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    ${PCL_DEFINITIONS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARY_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${GUROBI_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)



target_include_directories(${PROJECT_NAME} PUBLIC "src")
target_link_libraries(${PROJECT_NAME} PUBLIC ${COMMON_LINKER_FLAGS})
target_compile_options(${PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})



# ===============================================
# scikit_build
Python_add_library(_core MODULE src/ReUseX.cpp WITH_SOABI)
target_link_libraries(_core PRIVATE pybind11::headers ${PROJECT_NAME})

# This is passing in the version as a define just as an example
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
install(TARGETS _core DESTINATION ReUseX)

