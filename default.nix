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
  highs,
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
  # suitesparse-graphblas,
  # LAGraph,
  igraph,
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
  curl,
  nlohmann_json,
  openssl,
  #libtorch-bin,
  libtorch,
  oneDNN,
  qt6Packages,
  protobuf,
  libpng,
  trtsam3,
  tokenizers-cpp,
  onnxruntime,
  exiv2,
  # cuOpt,
}: let
  effectiveStdenv =
    if cudaSupport
    then cudaPackages.backendStdenv
    else stdenv;
in
  effectiveStdenv.mkDerivation rec {
    pname = "ReUseX";
    version = "0.0.2";

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
        highs
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
        # suitesparse-graphblas
        # LAGraph
        igraph

        mpfr

        opencv
        cli11

        trtsam3

        #libtorch-bin
        libtorch
        oneDNN
        protobuf # should be in libtorch?
        onnxruntime

        catch2_3
        tokenizers-cpp

        curl
        nlohmann_json
        openssl
        exiv2
      ]
      ++ (
        if cudaSupport
        then
          with cudaPackages; [
            cuda_cudart
            cudnn
          ]
        # cuOpt is optional GPU solver (disabled - marked broken)
        # ++ [cuOpt]
        else []
      );

    dontWrapQtApps = true;

    meta = with lib; {
      description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
      license = licenses.gpl3Plus;
      mainProgram = "rux";
      maintainers = with maintainers; [];
    };
  }
