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
  igraph,
  mpfr,
  opencv,
  # glfw,
  blender,
  # python,
  # imgui,
  # glm,
  # libGLU,
  catch2_3,
  libe57format,
  cli11,
  curl,
  crow,
  asio,
  libpqxx,
  libpq,
  redis-plus-plus,
  hiredis,
  aws-sdk-cpp-s3,
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
  openmvs,
  nanoflann,
  libjxl,
  cuOpt,
  addDriverRunpath,
}: let
  effectiveStdenv =
    if cudaSupport
    then cudaPackages.backendStdenv
    else stdenv;
in
  effectiveStdenv.mkDerivation rec {
    pname = "ReUseX";
    version = "0.0.5";

    src = ./.;

    # Native dependencies
    # programs and libraries used at build-time
    nativeBuildInputs =
      [
        cmake
        pkg-config
        qt6.qtbase
        # qt6Packages.wrapQtAppsHook
        # qt6.wrapQtAppsHook
        qt6.wrapQtAppsNoGuiHook
        blender.pythonPackages.python # Pin Python version to Blender's (3.11)
      ]
      # CUDA-only build tools:
      # - cudatoolkit provides nvcc, required to compile the .cu sources and for
      #   CMake's `enable_language(CUDA)`.
      # - addDriverRunpath ships a helper that patches a binary's RUNPATH to
      #   include /run/opengl-driver/lib. On NixOS the real libcuda.so lives
      #   there, not in any Nix store path — without this any CUDA-using binary
      #   fails at runtime with cudaErrorStubLibrary.
      ++ lib.optionals cudaSupport [
        cudatoolkit
        addDriverRunpath
      ];

    buildInputs =
      [
        opennurbs
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

        igraph

        mpfr

        opencv
        cli11

        #libtorch-bin
        libtorch
        oneDNN
        protobuf # should be in libtorch?
        onnxruntime

        catch2_3
        tokenizers-cpp
        blender.pythonPackages.pybind11

        curl
        # Crow HTTP server (+ asio backend) for the `ruxd` service worker
        crow
        asio
        # Backend clients for the `ruxd` service worker:
        #   libpqxx        — PostgreSQL (job/metadata store)
        #   redis-plus-plus — Redis (cache / queue), built on hiredis
        #   aws-sdk-cpp-s3  — S3 object storage (s3-only build, see overlay)
        libpqxx
        libpq
        redis-plus-plus
        hiredis
        aws-sdk-cpp-s3
        nlohmann_json
        openssl
        exiv2

        # OpenMVS — library-linked Multi-View Stereo for `rux create dense`.
        # AGPL-3.0-or-later; combined work is therefore AGPL.
        openmvs
        # nanoflann is exposed in OpenMVS::Common's link interface; CMake
        # validates it at consumer configure time, so it must be findable.
        nanoflann
        # OpenMVS::IO link interface contains a bare `jxl` library name
        # (rather than an absolute path or imported target), so libjxl
        # must be on the linker search path at consumer link time.
        libjxl
      ]
      # CUDA-only runtime dependencies. trtsam3 is the TensorRT-based SAM
      # backend and cuOpt is the GPU MIP/LP solver — both are inherently CUDA;
      # the vision/solver code paths that use them are gated out by CMake when
      # the backends aren't found.
      ++ lib.optionals cudaSupport (
        [
          trtsam3
          cuOpt
        ]
        ++ (with cudaPackages; [
          cuda_cudart
          cudnn
        ])
      );

    # Drive the project's WITH_CUDA option from cudaSupport. For CPU and ROCm
    # builds (cudaSupport = false) this keeps CMake from enabling the CUDA
    # language (no nvcc required) and auto-excludes the CUDA-only code paths
    # (TensorRT backend + .cu kernels, cuOpt solver). ROCm-ness itself comes
    # from libtorch/onnxruntime being built under a rocmSupport nixpkgs.
    cmakeFlags = [
      (lib.cmakeBool "WITH_CUDA" cudaSupport)
    ];

    dontWrapQtApps = true;

    # Patch the installed binaries and shared libraries so the dynamic loader
    # finds the host NVIDIA driver (libcuda.so) at /run/opengl-driver/lib.
    # Without this, `nix run .#default` would die with cudaErrorStubLibrary.
    postFixup = lib.optionalString cudaSupport ''
      for f in $(find $out/bin $out/lib -type f \
                      \( -executable -o -name '*.so' -o -name '*.so.*' \) \
                      2>/dev/null); do
        if isELF "$f"; then
          addDriverRunpath "$f"
        fi
      done
    '';

    meta = with lib; {
      description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
      license = licenses.gpl3Plus;
      mainProgram = "rux";
      maintainers = with maintainers; [];
    };
  }
