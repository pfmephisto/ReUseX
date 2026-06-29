{
  lib,
  fetchFromGitHub,
  cudaPackages,
  cmake,
  git,
  python3,
  cccl,
  rmm,
  rapids-cmake,
  rapids-logger,
  cpm-cmake,
  cuco,
  cutlass,
  spdlog,
  fmt,
}: let
  rapids_cmake_dir = "${rapids-cmake}/share/cmake/rapids-cmake";

  # Pre-fetch spdlog and fmt sources for CPM (exact versions from rapids-cmake versions.json)
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
    pname = "raft";
    version = "25.10.00";

    src = fetchFromGitHub {
      owner = "rapidsai";
      repo = "raft";
      rev = "v${version}";
      hash = "sha256-d/ghw3lkmcE8v8NhHUhXst6/jdgY93Q61KYhHdQJKtA=";
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

      # Create VERSION and RAPIDS_BRANCH files
      echo "${version}" > ../VERSION
      echo "branch-25.10" > ../RAPIDS_BRANCH

      # Point CMake to Nix-provided dependencies
      export CMAKE_PREFIX_PATH="${cccl}:${rmm}:${rapids-logger}:$CMAKE_PREFIX_PATH"
    '';

    nativeBuildInputs = [
      cmake
      cudaPackages.cuda_nvcc
      git
      python3
    ];

    buildInputs =
      [
        rapids-cmake
        cpm-cmake
        cccl
        rmm
        rapids-logger
        spdlog
        fmt
      ]
      ++ (with cudaPackages; [
        cuda_cudart
        libcublas
        libcusolver
        libcusparse
      ]);

    cmakeFlags = [
      # Provide CPM.cmake locally so rapids_cpm_init() works offline
      "-DCPM_DOWNLOAD_LOCATION=${cpm-cmake}/CPM.cmake"

      # Let CPM find Nix-provided packages first
      "-DCPM_USE_LOCAL_PACKAGES=ON"

      # Provide exact source for CPM deps (always_download or no cmake config)
      "-DCPM_cuco_SOURCE=${cuco}"
      "-DCPM_NvidiaCutlass_SOURCE=${cutlass}"
      "-DCPM_spdlog_SOURCE=${spdlog-src}"
      "-DCPM_fmt_SOURCE=${fmt-src}"

      # Disable cuco's test data download (not needed and requires network)
      "-DCUCO_DOWNLOAD_ROARING_TESTDATA=OFF"

      # Use specific CUDA architectures instead of "all"
      # 75=Turing, 80=Ampere, 86=Ampere(Gaming), 89=Ada, 90=Hopper
      "-DCMAKE_CUDA_ARCHITECTURES=75;80;86;89;90"

      # Disable tests and benchmarks (require GPU runtime)
      "-DBUILD_TESTS=OFF"
      "-DRAFT_BUILD_TESTS=OFF"
      "-DRAFT_BUILD_BENCHMARKS=OFF"

      # Build header-only mode (no compiled libraries)
      "-DRAFT_COMPILE_LIBRARIES=OFF"

      # Standard install directories
      "-DCMAKE_INSTALL_INCLUDEDIR=include"
      "-DCMAKE_INSTALL_LIBDIR=lib"
    ];

    # Skip tests (require GPU runtime)
    doCheck = false;

    meta = with lib; {
      description = "RAPIDS RAFT - Reusable Accelerated Functions and Tools for vector search and more";
      homepage = "https://github.com/rapidsai/raft";
      license = licenses.asl20;
      platforms = platforms.linux;
      maintainers = [];
    };
  }
