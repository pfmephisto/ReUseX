cmake_minimum_required(VERSION 3.17)
project(ReUseXLib LANGUAGES CXX CUDA)

# Include library-specific modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CompilerOptions)
include(CUDAOptions)
include(Dependencies)
include(ReUseXLibrary)

message(STATUS "")
message(STATUS "========================================")
message(STATUS "ReUseX Library Configuration Summary")
message(STATUS "========================================")
message(STATUS "Solver:            ${USE_MIP_SOLVER}")
message(STATUS "========================================")
message(STATUS "")
