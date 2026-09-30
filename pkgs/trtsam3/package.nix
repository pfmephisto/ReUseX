# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  stdenv,
  cmake,
  fetchFromGitHub,
  ninja,
  cudaPackages,
  opencv,
  freetype,
}:
stdenv.mkDerivation rec {
  pname = "trt-sam3";
  version = "0.0.1";

  src = fetchFromGitHub {
    owner = "leon0514";
    repo = "${pname}";
    rev = "cd52afa83290e35f87c34ec8d2d321cae805c2b5";
    sha256 = "sha256-GMFglZZOidl+2wEiQSizQ8bXixABgT+TIK20+3Nh9EI=";
  };

  patches = [
    ./install.patch
  ];

  nativeBuildInputs = [
    cmake
    ninja
    # The compiler only. Not the merged `cudatoolkit`: propagating that put the
    # whole toolkit (nvcc included) into every consumer's RUNPATH and thus its
    # runtime closure.
    cudaPackages.cuda_nvcc
  ];

  # trtsam_core's exported link interface is `cudart;cublas;cudnn` (bare
  # names) plus the TensorRT libs, and its headers include <cuda_runtime.h>.
  propagatedBuildInputs = [
    cudaPackages.cuda_cudart
    cudaPackages.libcublas
    cudaPackages.tensorrt
    cudaPackages.cudnn
    opencv
    freetype
  ];
}
