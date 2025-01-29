cmake_minimum_required(VERSION 3.8)
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

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

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

# ===============================================
# Bin dir
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BIN_DIR})
# Set CMAKE_POSITION_INDEPENDENT_CODE to ON
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(GLOW_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)

# REMOVED: this is not needed anymore
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE On)



#set("CUDA_TOOLKIT_ROOT_DIR" "/usr/local/cuda-12-3")
#set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include/")
#set(CUDA_CUDART_LIBRARY "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")

if(Off)

    message(STATUS "Building inside a Nix environment.")
    # ===============================================

    # find_package(CUDA REQUIRED)
    # find_package(CUDAToolkit REQUIRED)
    # find_package(TBB REQUIRED)
    find_package(Boost REQUIRED)
    # find_package(SCIP REQUIRED)
    # find_package(Gurobi REQUIRED)
    find_package(embree REQUIRED)

    # find_package(Eigen REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(CGAL REQUIRED)
    find_package(PCL REQUIRED)
    find_package(Python3 REQUIRED)

    find_package(xtensor REQUIRED)
    find_package(xtensor-io REQUIRED)

    find_package(fmt REQUIRED)
    find_package(polyscope REQUIRED)
    find_package(pybind11 REQUIRED)
    
    find_package(opennurbs REQUIRED)

    find_package(HighFive REQUIRED)

    find_package(g2o REQUIRED)

else()

# ===============================================
# find packages

    include(cmake/CUDA.cmake)
    include(cmake/TBB.cmake)
    include(cmake/Boost.cmake)
    include(cmake/SCIP.cmake)
    include(cmake/Gurobi.cmake)
    include(cmake/Embree.cmake)

    include(cmake/Eigen.cmake)
    include(cmake/OpenCV.cmake)
    include(cmake/CGAL.cmake)
    include(cmake/PCL.cmake)
    include(cmake/Python.cmake)

    find_package(xtensor REQUIRED)
    find_package(xtensor-io REQUIRED)




# ===============================================
# add submodules

    add_subdirectory(extern/fmt)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/pybind11)
    add_subdirectory(extern/opennurbs)
    add_subdirectory(extern/hdf5)

    add_subdirectory(extern/highfive)



    set(OpenGL_GL_PREFERENCE  "GLVND") # use GLVND OpenGL library instead of legacy.
    find_package(g2o REQUIRED)

endif()



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


## ===============================================
include_directories( ${OpenCV_INCLUDE_DIRS} )

# ===============================================
# main libary
add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
    pybind11::embed 
    optimized ${GUROBI_CXX_LIBRARY}
    debug ${GUROBI_CXX_DEBUG_LIBRARY}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    HighFive
    xtensor
    xtensor-io
    
    OpenNURBS
    
    polyscope
    embree
    ${CUDA_LIBRARIES} 
    ${CUDA_CUBLAS_LIBRARIES}

    Eigen3::Eigen3

    CGAL::Eigen3_support
    CGAL::SCIP_support
    CGAL::TBB_support

    ${GUROBI_LIBRARY}

    ${PCL_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}

    fmt
    mpfr
    MPI::MPI_CXX

    ${OpenCV_LIBS}

    ${COMMON_LINKER_FLAGS}
)

# # dependency grouping in the IDE
# foreach(TARGET_NAME
#     polyscope
#     OpenNURBS
# )
# set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER "Extern")
# endforeach()


target_include_directories(${PROJECT_NAME} PUBLIC "src")
target_compile_options(${PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})



# ===============================================
# scikit_build
python_add_library(_core MODULE src/ReUseX.cpp WITH_SOABI)
target_link_libraries(_core PRIVATE pybind11::headers ${PROJECT_NAME})

# This is passing in the version as a define just as an example
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
install(TARGETS _core DESTINATION ReUseX)

