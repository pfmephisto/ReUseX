# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  cmake,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  cudaPackages,
  qt6,
  pkg-config,
  cudatoolkit,
  opennurbs,
  scip-solver,
  boost,
  # fmt,
  spdlog,
  #spdmon,
  range-v3,
  pcl,
  embree,
  eigen,
  cgal,
  rtabmap,
  librealsense,
  octomap,
  # suitesparse,
  suitesparse-graphblas,
  LAGraph,
  mpfr,
  opencv,
  # glfw,
  # python3Packages.pybind11,
  # python,
  # imgui,
  # glm,
  # libGLU,
  catch2_3,
  libe57format,
  xercesc,
  cli11,
  #libtorch-bin,
  libtorch,
  mkl,
  qt6Packages,
  protobuf,
  libpng,
  ...
}: let
  effectiveStdenv =
    if cudaSupport
    then cudaPackages.backendStdenv
    else stdenv;
in
  effectiveStdenv.mkDerivation rec {
    pname = "ReUseX";
    version = "0.0.1";

    src = ./.;

    # Native dependencies
    # programs and libraries used at build-time
    nativeBuildInputs = [
      cmake
      pkg-config
      cudatoolkit
      qt6.qtbase
      # qt6Packages.wrapQtAppsHook
      # qt6.wrapQtAppsHook
      qt6.wrapQtAppsNoGuiHook
    ];

    buildInputs =
      [
        opennurbs
        scip-solver
        boost

        # fmt
        spdlog
        #spdmon
        range-v3
        libpng

        pcl
        embree
        eigen
        cgal

        rtabmap
        librealsense
        octomap

        libe57format
        xercesc

        # suitesparse
        suitesparse-graphblas
        LAGraph

        mpfr

        opencv
        cli11

        #libtorch-bin
        libtorch
        protobuf # should be in libtorch?

        catch2_3

        mkl
      ]
      ++ (
        if cudaSupport
        then
          with cudaPackages; [
            cuda_cudart
            cudnn
          ]
        else []
      );

    dontWrapQtApps = true;

    meta = with lib; {
      description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
      license = licenses.gpl3Plus;
      maintainers = with maintainers; [];
    };
  }
