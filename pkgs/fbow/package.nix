# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  lib,
  stdenv,
  cmake,
  opencv,
  cudaPackages,
  ...
}: let
in
  stdenv.mkDerivation rec {
    pname = "fbow";
    version = "e148dbc";

    src = fetchFromGitHub {
      owner = "rmsalinas";
      repo = "${pname}";
      rev = "e148dbcbd8ee1976b1b07988b5761a9171b9f01a";
      #fetchSubmodules = true;
      sha256 = "sha256-nuTgr4Rb1HLYE2RUXEeI7x+FRz3gE0LuZdbwTkU0WVg=";
    };

    nativeBuildInputs = [
      cmake
    ];

    propagationBuildInputs = [
    ];

    buildInputs = [
      opencv
    ];

    cmakeFlags = [
      "-D CUDA_TOOLKIT_ROOT_DIR=${cudaPackages.cudatoolkit}"
    ];

    meta = with lib; {
      description = "FBOW (Fast Bag of Words) is an extremmely optimized version of the DBow2/DBow3 libraries.";
      #license = licenses.mit;
      maintainers = with maintainers; [];
    };
  }
