# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  cmake,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  # Opt-in torch-free build (`.#cuda-notorch` / `.#cpu-notorch`). When false,
  # libtorch and gsplat-cuda (itself LibTorch-linked) are left out of
  # buildInputs and ML_BACKENDS is pinned to the non-LibTorch backends, so:
  #   - the LibTorch ML backend (YOLO .pt), vision/nms and the vendored
  #     torchvision kernel are excluded by MLBackendConfig.cmake;
  #   - reusex_gsplat is skipped (REUSEX_HAVE_GSPLAT off) and
  #     `rux create gsplat` reports that the build lacks support.
  # The default `true` keeps the derivation byte-identical to before.
  withLibtorch ? true,
  cudaPackages,
  qt6,
  pkg-config,
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
  # rux NEEDs libmetis.so directly (via GTSAM's ordering API). Without metis in
  # our own buildInputs its lib dir is absent from rux's RUNPATH, so the binary
  # fails at startup with "libmetis.so: cannot open shared object file".
  metis,
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
  # Prebuilt libtorch (pkgs/libtorch: 2.9.0+cu128, CUDA 12.x), replacing the
  # from-source build whose pinned pytorch submodules became unfetchable.
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
  # Individual CUDA redist libraries linked by the build. This is the set
  # LibTorch's Caffe2 CMake config, trtsam3, cuOpt and OpenCV's CUDA modules
  # resolve through FindCUDAToolkit / CUDA::* targets, plus the headers they
  # include. Shared with the dev shell through passthru (see shell.nix).
  cudaLibraries = lib.optionals cudaSupport (with cudaPackages; [
    cuda_cudart
    cccl # <thrust/*>, <cub/*> (CUDA::cccl / torch headers)
    cuda_nvrtc # Torch imported target references CUDA_nvrtc_LIBRARY
    cuda_nvtx
    cuda_profiler_api
    libcublas
    libcufft
    libcurand
    libcusolver
    libcusparse
    cudnn
  ]);

  effectiveStdenv =
    if cudaSupport
    then cudaPackages.backendStdenv
    else stdenv;
in
  effectiveStdenv.mkDerivation rec {
    pname = "ReUseX";
    version = "0.0.5";

    src = lib.fileset.toSource {
      root = ./.;
      fileset = lib.fileset.unions [
        # Build system
        ./CMakeLists.txt
        ./cmake
        # Embedded into the generated version header by reusexLibrary.cmake
        # (file(READ ...LICENSE.md)); the build fails to configure without it.
        ./LICENSE.md
        # C++ library source, headers, and CMake config
        ./libs
        # Applications: rux CLI and ruxd service worker.
        # apps/blender has no CMakeLists.txt and is not part of the C++ build.
        ./apps/rux
        ./apps/ruxd
        # Python bindings (pybind11, BUILD_PYTHON_BINDINGS)
        ./bindings
        # Tests: unit, integration, benchmarks, support, and binary fixtures
        ./tests
        # ctest registers this as the gui_api_contract_parses test
        ./scripts/check-openapi.py
        # Accessed at test runtime via REUSEX_SOURCE_DIR (test_stage_contract.cpp)
        ./docs/CONTRACTS.md
        # Embedded into a generated header by reusexLibrary.cmake (the native
        # TensorRT EngineBuilder's default SAM 3.1 recipe)
        ./python/reusex_sam3/engine-build.json
        # Accessed at test runtime by scripts/check-openapi.py
        ./docs/gui/openapi.yaml
        ./docs/gui/events.schema.json
      ];
    };

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
      # - cuda_nvcc is the compiler, required to compile the .cu sources and for
      #   CMake's `enable_language(CUDA)`. It is deliberately the bare redist
      #   package, not `cudatoolkit`: the merged toolkit's lib/ ends up in
      #   rux's RUNPATH and drags the whole toolkit (nvcc included, ~4 GiB)
      #   into the runtime closure. The libraries actually linked are listed
      #   individually in buildInputs below.
      # - addDriverRunpath ships a helper that patches a binary's RUNPATH to
      #   include /run/opengl-driver/lib. On NixOS the real libcuda.so lives
      #   there, not in any Nix store path — without this any CUDA-using binary
      #   fails at runtime with cudaErrorStubLibrary.
      ++ lib.optionals cudaSupport [
        cudaPackages.cuda_nvcc
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
        # METIS — pulled in transitively via GTSAM; listed so libmetis.so lands
        # in rux's RUNPATH (see the arg comment above).
        metis

        rtabmap
        librealsense
        octomap

        libe57format

        igraph

        mpfr

        opencv
        cli11
      ]
      ++ lib.optionals withLibtorch [libtorch]
      ++ [
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
        ++ lib.optionals withLibtorch [
          # gsplat's CUDA rasterization backend (Apache-2.0), vendored as a
          # standalone LibTorch-linked static library. Backs the `reusex_gsplat`
          # module and `rux create gsplat` (#240). CUDA-only by construction;
          # reusexLibrary.cmake skips the module when it is not found.
          gsplat-cuda
        ]
        # Individual CUDA redist libraries instead of the merged `cudatoolkit`
        # (see nativeBuildInputs and `cudaLibraries` above).
        ++ cudaLibraries
      );

    # Drive the project's WITH_CUDA option from cudaSupport. For CPU and ROCm
    # builds (cudaSupport = false) this keeps CMake from enabling the CUDA
    # language (no nvcc required) and auto-excludes the CUDA-only code paths
    # (TensorRT backend + .cu kernels, cuOpt solver). ROCm-ness itself comes
    # from libtorch/onnxruntime being built under a rocmSupport nixpkgs.
    cmakeFlags =
      [
        (lib.cmakeBool "WITH_CUDA" cudaSupport)
      ]
      # Without libtorch, pin the backend list instead of relying on AUTO: an
      # explicit list makes a missing TensorRT/ONNX dependency a configure
      # error rather than a silently smaller build. This is what AUTO resolves
      # to in the default build, minus LibTorch (OpenVINO is not packaged).
      ++ lib.optionals (!withLibtorch) [
        (lib.cmakeFeature "ML_BACKENDS" (
          if cudaSupport
          then "TensorRT;ONNX"
          else "ONNX"
        ))
      ];

    # Wrapped by hand in postFixup: only `rux` (the Qt client) needs Qt's
    # plugin path; wrapping everything would add a shell hop to every tool.
    dontWrapQtApps = true;

    # The CUDA packages this build compiles against, for shell.nix to merge
    # into one toolkit root. Inside this derivation nixpkgs'
    # setupCUDAToolkitCompilers hook passes -DCUDAToolkit_INCLUDE_DIR/_ROOT
    # lists to CMake, which is what lets the split redist packages satisfy
    # LibTorch's bundled FindCUDAToolkit. A dev shell never runs that hook, so
    # it needs a single directory instead. passthru is not part of the
    # derivation, so this does not touch the package or its closure.
    passthru.cudaToolkitPackages =
      lib.optionals cudaSupport ([cudaPackages.cuda_nvcc] ++ cudaLibraries);

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
    postFixup =
      lib.optionalString cudaSupport ''
        for f in $(find $out/bin $out/lib -type f \
                        \( -executable -o -name '*.so' -o -name '*.so.*' \) \
                        2>/dev/null); do
          if isELF "$f"; then
            addDriverRunpath "$f"
          fi
        done
      ''
      # Plain `rux` is the native Qt client (apps/rux/qt), so the installed
      # binary must find Qt's platform plugins (xcb, wayland) outside the dev
      # shell. dontWrapQtApps above keeps the hook from wrapping every binary
      # (ruxd and the tools need no Qt plugins); only rux is wrapped, after
      # the runpath loop so patchelf sees the real ELF, not the wrapper.
      + ''
        wrapQtApp $out/bin/rux
      '';

    meta = with lib; {
      description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
      license = licenses.gpl3Plus;
      mainProgram = "rux";
      maintainers = with maintainers; [];
    };
  }
