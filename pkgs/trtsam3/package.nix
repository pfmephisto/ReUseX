# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  stdenv,
  cmake,
  fetchFromGitHub,
  ninja,
  cudatoolkit,
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
  ];

  propagatedBuildInputs = [
    cudatoolkit
    cudaPackages.tensorrt
    cudaPackages.cudnn
    opencv
    freetype
  ];
}
