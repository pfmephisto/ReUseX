# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  highs =
    (prev.highs.override {
      # Use GCC 13 as the build stdenv - CUDA 12.8 doesn't fully support GCC 14+
      stdenv = prev.gcc13Stdenv;
    }).overrideAttrs
    (
      self: super: {
        cmakeFlags =
          (super.cmakeFlags or [])
          ++ [
            (prev.lib.cmakeFeature "CUPDLP_GPU" "ON")
            # Explicitly set CUDA host compiler to GCC 13 (override CUDA hooks)
            (prev.lib.cmakeFeature "CMAKE_CUDA_HOST_COMPILER" "${prev.gcc13Stdenv.cc}/bin/g++")
          ];

        # Set environment variable for CUDA host compiler
        CUDAHOSTCXX = "${prev.gcc13Stdenv.cc}/bin/g++";

        nativeBuildInputs =
          (super.nativeBuildInputs or [])
          ++ (with prev.cudaPackages; [
            cuda_nvcc
          ]);

        buildInputs =
          (super.buildInputs or [])
          ++ (with prev.cudaPackages; [
            libcublas
            libcusparse
            cuda_cudart
          ]);
      }
    );
}
