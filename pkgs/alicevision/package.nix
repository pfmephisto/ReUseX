{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
  ...
}: let
  flann = pkgs.flann.overrideAttrs {
    src = fetchFromGitHub {
      owner = "alicevision";
      repo = "flann";
      rev = "9e79ca6881b779fb842e83c17aef1f4c816ce058";
      fetchSubmodules = true;
      sha256 = "sha256-CrcBp3kMY/pK6TSYSowaHJzZaEWNKMtGFk/y64QOivs=";
    };
    patches = [];
  };
in
  stdenv.mkDerivation rec {
    pname = "AliceVision";
    version = "e17a609-dev";

    src = fetchFromGitHub {
      owner = "alicevision";
      repo = "${pname}";
      rev = "e17a60985c7e712dbf355041d495becf20db693f";
      fetchSubmodules = true;
      sha256 = "sha256-c63cQyVS6L8r1ld+/EX56TcPklQXz3iJZZG5lPdCe7U=";
    };

    nativeBuildInputs = with pkgs; [
      cmake
    ];

    propagationBuildInputs = with pkgs; [
      libpng
      libjpeg
      libtiff
      xorg.libXxf86vm
      xorg.libXi
      xorg.libXrandr
      graphviz
    ];

    buildInputs = with pkgs; [
      assimp
      boost
      ceres-solver
      suitesparse
      coin-utils
      clp
      eigen
      expat
      flann
      geogram
      nanoflann
      openexr
      openimageio
      openmesh
      osi
      zlib

      # Optional:
      alembic
      cctag
      cudaPackages.cudatoolkit
      # magma
      # mosek
      opencv
      opengv
      pcl
      # popsift
      # uncertaintyte
      lemon
      libe57format
      # MeshSDFilter
      onnxruntime
    ];

    # cmakeFlags = [
    #   "-DCeres_DIR=${pkgs.ceres-solver}/lib/cmake/Ceres/"
    #   #"-DFLANN_INCLUDE_DIR=${pkgs.flann}/include"
    #   "-Dflann_DIR:PATH=${pkgs.flann}"
    #   "-DOPENEXR_HOME:PATH=${pkgs.openexr}/"
    #   "-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=${pkgs.openimageio}/lib"
    #   "-DOPENIMAGEIO_INCLUDE_DIR:PATH=${pkgs.openimageio}/include"
    #   "-DGEOGRAM_INSTALL_PREFIX:PATH=${pkgs.geogram}/"
    #   "-DOPENIMAGEIO_LIBRARY_DIR_HINTS:PATH=${pkgs.openimageio}/lib"
    #   "-DOPENIMAGEIO_INCLUDE_DIR:PATH=${pkgs.openimageio}/include"
    #   # BOOST_NO_CXX11
    #   # ALICEVISION_USE_OPENMP
    #   # ALICEVISION_USE_CCTAG
    #   "-DCCTag_DIR:PATH=${pkgs.cctag}/lib/cmake/CCTag"
    #   # ALICEVISION_USE_APRILTAG
    #   #"-Dapriltag_DIR:PATH=${pkgs.apriltag}/share/apriltag"
    #   # ALICEVISION_USE_OPENGV
    #   "-DOPENGV_DIR:PATH=${pkgs.opengv}/lib/cmake/opengv"
    #   # ALICEVISION_USE_ALEMBIC
    #   "-DAlembic_DIR:PATH=${pkgs.alembic}/lib/cmake/Alembic"
    #   #ALICEVISION_USE_CUDA
    #   "-DCUDA_TOOLKIT_ROOT_DIR:PATH=${pkgs.cudaPackages.cudatoolkit}/"
    #   # ALICEVISION_USE_POPSIFT
    #   # "-DPopSift_DIR:PATH=${pkgs.popsift}/lib/cmake/PopSift"
    #   # ALICEVISION_USE_UNCERTAINTYTE
    #   # "-DUncertaintyTE_DIR:PATH=${pkgs.uncertaintyte}/
    #   # "-DMAGMA_ROOT:PATH=${pkgs.magma}/"
    #   "ALICEVISION_USE_OPENCV =ON"
    #   "-DOpenCV_DIR:PATH=${pkgs.opencv}/lib/cmake/opencv4"
    #   # ALICEVISION_REQUIRE_CERES_WITH_SUITESPARSE
    #   # BUILD_SHARED_LIBS
    #   # ALICEVISION_BUILD_TESTS
    #   # ALICEVISION_BUILD_DOC
    #   # ALICEVISION_BUILD_COVERAGE
    #   # ALICEVISION_BUILD_SWIG_BINDING
    # ];

    meta = with lib; {
      description = "Photogrammetric Computer Vision Framework.";
      license = licenses.mit;
      maintainers = with maintainers; [];
    };
  }
