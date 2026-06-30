cmake_minimum_required(VERSION 3.17)
project(reusexLib LANGUAGES CXX)

# Include library-specific modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CompilerOptions)
# CUDA language is enabled (and architectures configured) only for CUDA builds;
# the .cu sources live entirely under the TensorRT backend, which is excluded
# when CUDA/trtsam3 are absent.
if(WITH_CUDA)
    include(CUDAOptions)
endif()
include(Dependencies)
include(reusexLibrary)
