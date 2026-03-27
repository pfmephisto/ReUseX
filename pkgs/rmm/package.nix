{
  lib,
  fetchFromGitHub,
  cudaPackages,
  cmake,
  cccl,
  nvtx3,
  rapids-cmake,
  rapids-logger,
  cpm-cmake,
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
    pname = "rmm";
    version = "25.10.00";

    src = fetchFromGitHub {
      owner = "rapidsai";
      repo = "rmm";
      rev = "v${version}";
      hash = "sha256-QF3EC4PedQA9DMaTku0FsfSSt6QqVzqtDos2oqbjMO8=";
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
      export CMAKE_PREFIX_PATH="${cccl}:${nvtx3}:${rapids-logger}:$CMAKE_PREFIX_PATH"
    '';

    nativeBuildInputs = [
      cmake
      cudaPackages.cuda_nvcc
    ];

    buildInputs =
      [
        rapids-cmake
        cpm-cmake
        cccl
        nvtx3
        rapids-logger
        spdlog
        fmt
      ]
      ++ (with cudaPackages; [
        cuda_cudart
        libcublas
      ]);

    cmakeFlags = [
      # Provide CPM.cmake locally so rapids_cpm_init() works offline
      "-DCPM_DOWNLOAD_LOCATION=${cpm-cmake}/CPM.cmake"

      # Let CPM find Nix-provided packages first
      "-DCPM_USE_LOCAL_PACKAGES=ON"

      # Provide exact source for deps that CPM must build from source
      "-DCPM_spdlog_SOURCE=${spdlog-src}"
      "-DCPM_fmt_SOURCE=${fmt-src}"

      # Use specific CUDA architectures instead of "all"
      # 75=Turing, 80=Ampere, 86=Ampere(Gaming), 89=Ada, 90=Hopper
      "-DCMAKE_CUDA_ARCHITECTURES=75;80;86;89;90"

      # Disable tests and benchmarks (require GPU runtime)
      "-DBUILD_TESTS=OFF"
      "-DBUILD_BENCHMARKS=OFF"

      # Standard install directories
      "-DCMAKE_INSTALL_INCLUDEDIR=include"
      "-DCMAKE_INSTALL_LIBDIR=lib"
    ];

    # Skip tests (require GPU runtime)
    doCheck = false;

    meta = with lib; {
      description = "RAPIDS Memory Manager - GPU memory allocation library";
      homepage = "https://github.com/rapidsai/rmm";
      license = licenses.asl20;
      platforms = platforms.linux;
      maintainers = [];
    };
  }
