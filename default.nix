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
  gtsam,
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
  # Static React/Vite bundle served by `rux gui` (pkgs/reusex-gui-frontend).
  # Not a link-time dependency: it is copied into share/reusex/gui by
  # postInstall below. Kept as its own derivation so the two build graphs stay
  # independent — an npm/lockfile change must not invalidate the multi-hour C++
  # build, and editing a .cpp must not re-run the npm build.
  reusex-gui-frontend,
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
  gsplat-cuda,
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
        # wrapQtAppsNoGuiHook was deprecated in nixpkgs and now just warns and
        # forwards here; wrapQtAppsHook is the supported name.
        qt6.wrapQtAppsHook
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

        # GTSAM — factor-graph optimization for the `rux optimize` pose-graph
        # back-end (plane-landmark factors + GNC). MIT.
        gtsam

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
          # gsplat's CUDA rasterization backend (Apache-2.0), vendored as a
          # standalone LibTorch-linked static library. Backs the `reusex_gsplat`
          # module and `rux create gsplat` (#240). CUDA-only by construction;
          # reusexLibrary.cmake skips the module when it is not found.
          gsplat-cuda
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

    # Drop the prebuilt GUI bundle next to the binaries. `rux gui` falls back to
    # <install prefix>/share/reusex/gui when neither --assets nor
    # $RUX_GUI_ASSETS is set (apps/rux/src/gui/assets.cpp), so this is what
    # makes `nix run .#default -- gui` serve a UI at all.
    #
    # Nothing here interacts with the postFixup runpath loop below: that loop
    # only walks $out/bin and $out/lib and additionally guards on isELF, while
    # these are static web assets under $out/share.
    postInstall = ''
      mkdir -p $out/share/reusex/gui
      cp -r ${reusex-gui-frontend}/share/reusex/gui/. $out/share/reusex/gui/
      chmod -R u+w $out/share/reusex/gui
    '';

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
