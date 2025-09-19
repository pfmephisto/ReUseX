# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  cmake,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  qt6,
  pkg-config,
  wrapGAppsHook3,
  cudatoolkit,
  opennurbs,
  scip-solver,
  boost,
  # fmt,
  spdlog,
  #spdmon,
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
  cli11,
  ...
}:
stdenv.mkDerivation rec {
  pname = "ReUseX";
  version = "0.0.1";

  src = ../../.;

  # Native dependencies
  # programs and libraries used at build-time
  nativeBuildInputs = [
    cmake
    qt6.wrapQtAppsHook
    pkg-config
    wrapGAppsHook3
    cudatoolkit
  ];

  buildInputs = [
    opennurbs
    scip-solver
    boost

    # fmt
    spdlog
    #spdmon

    pcl
    embree
    eigen
    cgal

    rtabmap
    librealsense
    octomap

    # suitesparse
    suitesparse-graphblas
    LAGraph

    mpfr

    opencv
    # glfw

    # python3Packages.pybind11
    # python

    # imgui
    # glm
    # libGLU

    cli11
  ];

  dontWrapQtApps = true;

  meta = with lib; {
    description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
    license = licenses.gpl3Plus;
    maintainers = with maintainers; [];
  };
}
