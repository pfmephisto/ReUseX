# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  lib,
  fetchFromGitHub,
  cudaPackages,
  cmake,
  boost,
  bzip2,
  zlib,
  tbb,
  argparse,
  rapids-cmake,
  cpm-cmake,
  rapids-logger,
  cccl,
  rmm,
  raft,
  cudss,
  nvtx3,
  spdlog,
  fmt,
  ...
}: let
  rapids_cmake_dir = "${rapids-cmake}/share/cmake/rapids-cmake";

  # Pre-fetch PaPILO (cuOpt fetches this via FetchContent)
  papilo-src = fetchFromGitHub {
    owner = "scipopt";
    repo = "papilo";
    rev = "741a2b9c8155b249d6df574d758b4d97d4417520";
    hash = "sha256-iIZWPozQ/yLfdr+v4HpENMcsSW6Zrb/KR7tx5mN0NOc=";
  };

  # Pre-fetch spdlog and fmt sources for CPM (rapids_cpm_rapids_logger)
  spdlog-src = fetchFromGitHub {
    owner = "gabime";
    repo = "spdlog";
    rev = "27cb4c76708608465c413f6d0e6b8d99a4d84302";
    hash = "sha256-F7khXbMilbh5b+eKnzcB0fPPWQqUHqAYPWJb83OnUKQ=";
  };

  fmt-src = fetchFromGitHub {
    owner = "fmtlib";
    repo = "fmt";
    rev = "0c9fce2ffefecfdce794e1859584e25877b7b592";
    hash = "sha256-IKNt4xUoVi750zBti5iJJcCk3zivTt7nU12RIf8pM+0=";
  };
in
  cudaPackages.backendStdenv.mkDerivation rec {
    pname = "cuOpt";
    version = "25.10.00";

    src = fetchFromGitHub {
      owner = "NVIDIA";
      repo = "cuopt";
      rev = "v${version}";
      hash = "sha256-fXnfxMYW0KPwI3mI+71jy7aEVQHtJxdQyGjOkJV2+/M=";
    };

    postPatch = ''
      # Replace the RAPIDS.cmake FetchContent include with local rapids-cmake setup
      substituteInPlace cmake/rapids_config.cmake \
        --replace-fail \
          'include("''${CMAKE_CURRENT_LIST_DIR}/RAPIDS.cmake")' \
          '# Nix: Use local rapids-cmake instead of FetchContent
      set(rapids-cmake-dir "${rapids_cmake_dir}" CACHE INTERNAL "" FORCE)
      if(NOT "''${rapids-cmake-dir}" IN_LIST CMAKE_MODULE_PATH)
        list(APPEND CMAKE_MODULE_PATH "''${rapids-cmake-dir}")
      endif()
      include("''${rapids-cmake-dir}/rapids-version.cmake")'
    '';

    preConfigure = ''
      cd cpp

      # Point CMake to Nix-provided dependencies
      export CMAKE_PREFIX_PATH="${cccl}:${nvtx3}:${rmm}:${raft}:${rapids-logger}:${cudss}:$CMAKE_PREFIX_PATH"

      # Set CUDSS_DIR for FindCUDSS.cmake fallback path
      export CUDSS_DIR="${cudss}"
    '';

    nativeBuildInputs = [
      cmake
      cudaPackages.cuda_nvcc
    ];

    buildInputs =
      [
        boost
        bzip2
        zlib
        tbb
        argparse
        rapids-cmake
        cpm-cmake
        cccl
        nvtx3
        rmm
        raft
        rapids-logger
        cudss
        spdlog
        fmt
      ]
      ++ (with cudaPackages; [
        cuda_cudart
        cuda_profiler_api
        libcublas
        libcusolver
        libcusparse
        libcurand
      ]);

    cmakeFlags = [
      # Provide CPM.cmake locally so rapids_cpm_init() works offline
      "-DCPM_DOWNLOAD_LOCATION=${cpm-cmake}/CPM.cmake"

      # Let CPM find Nix-provided packages first
      "-DCPM_USE_LOCAL_PACKAGES=ON"

      # Provide exact source for deps that CPM must build from source
      "-DCPM_spdlog_SOURCE=${spdlog-src}"
      "-DCPM_fmt_SOURCE=${fmt-src}"

      # Skip RAPIDS fetching — use Nix-provided CCCL, RMM, RAFT via find_package
      "-DFETCH_RAPIDS=OFF"

      # Provide PaPILO source locally (avoids FetchContent git clone)
      "-DFETCHCONTENT_SOURCE_DIR_PAPILO=${papilo-src}"

      # Use specific CUDA architectures instead of "all"
      # 75=Turing, 80=Ampere, 86=Ampere(Gaming), 89=Ada, 90=Hopper
      "-DCMAKE_CUDA_ARCHITECTURES=75;80;86;89;90"

      # Build only LP/MIP solver, not routing engine
      "-DBUILD_LP_ONLY=ON"

      # Disable tests and benchmarks (require GPU runtime)
      "-DBUILD_TESTS=OFF"
      "-DBUILD_BENCHMARKS=OFF"

      # Work around format string bug in dual_simplex/basis_updates.hpp
      # (upstream has %\n instead of %%\n in printf)
      "-DCMAKE_CXX_FLAGS=-Wno-error=format"

      # Standard install directories
      "-DCMAKE_INSTALL_INCLUDEDIR=include"
      "-DCMAKE_INSTALL_LIBDIR=lib"
    ];

    # Ensure libraries are installed correctly
    installPhase = ''
      runHook preInstall

      # Use CMake's native install
      cmake --install .

      # Create lib directory if it doesn't exist
      mkdir -p $out/lib

      # Handle lib64 → lib migration if needed
      if [ -d "$out/lib64" ] && [ "$(ls -A $out/lib64)" ]; then
        cp -r $out/lib64/* $out/lib/
        rm -rf $out/lib64
      fi

      # Create include directory if headers weren't installed
      if [ ! -d "$out/include/cuopt" ]; then
        mkdir -p $out/include
        cp -r ../include/cuopt $out/include/
      fi

      runHook postInstall
    '';

    postInstall = ''
      # If native CMake config doesn't exist, create it
      if [ ! -f "$out/lib/cmake/cuOpt/cuOptConfig.cmake" ]; then
        mkdir -p $out/lib/cmake/cuOpt

        cat > $out/lib/cmake/cuOpt/cuOptConfig.cmake <<'EOF'
      # cuOpt CMake Config File (Nix fallback)
      if(NOT TARGET cuopt::cuopt)
        # Try to find the library
        find_library(CUOPT_LIBRARY
          NAMES cuopt
          PATHS "''${CMAKE_CURRENT_LIST_DIR}/../.."
          PATH_SUFFIXES lib
          NO_DEFAULT_PATH
        )

        if(NOT CUOPT_LIBRARY)
          message(FATAL_ERROR "cuOpt library not found in ''${CMAKE_CURRENT_LIST_DIR}/../..")
        endif()

        # Create imported target
        add_library(cuopt::cuopt SHARED IMPORTED)
        set_target_properties(cuopt::cuopt PROPERTIES
          IMPORTED_LOCATION "''${CUOPT_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "''${CMAKE_CURRENT_LIST_DIR}/../../include"
        )

        # Mark as found
        set(cuOpt_FOUND TRUE)

        # Version info
        set(cuOpt_VERSION "${version}")
        set(cuOpt_VERSION_MAJOR "25")
        set(cuOpt_VERSION_MINOR "10")
      endif()
      EOF

        # Create version file
        cat > $out/lib/cmake/cuOpt/cuOptConfigVersion.cmake <<'EOF'
      set(PACKAGE_VERSION "${version}")
      if("''${PACKAGE_FIND_VERSION}" VERSION_EQUAL "''${PACKAGE_VERSION}")
        set(PACKAGE_VERSION_EXACT TRUE)
      endif()
      if("''${PACKAGE_FIND_VERSION}" VERSION_LESS_EQUAL "''${PACKAGE_VERSION}")
        set(PACKAGE_VERSION_COMPATIBLE TRUE)
      endif()
      EOF
      fi

      # Validate installation
      if [ ! -f "$out/lib/libcuopt.so" ] && [ ! -f "$out/lib/libcuopt.dylib" ]; then
        echo "ERROR: cuOpt library not found after installation"
        ls -R $out/
        exit 1
      fi
    '';

    # Skip tests (require GPU runtime)
    doCheck = false;

    meta = with lib; {
      description = "NVIDIA GPU-accelerated optimization engine";
      homepage = "https://github.com/NVIDIA/cuopt";
      license = licenses.unfree;
      platforms = platforms.linux;
      maintainers = [];
    };
  }
