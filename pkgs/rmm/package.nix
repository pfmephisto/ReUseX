{
  lib,
  fetchFromGitHub,
  cudaPackages,
  cmake,
  cccl,
  rapids-cmake,
  spdlog,
  fmt,
}:
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
        # Patch CMakeLists.txt to skip FetchContent and use Nix-provided rapids-cmake
        substituteInPlace cpp/CMakeLists.txt \
          --replace-fail 'include(../cmake/rapids_config.cmake)' \
                         '# Nix: Skip rapids_config.cmake which uses FetchContent
    list(APPEND CMAKE_MODULE_PATH "$ENV{RAPIDS_CMAKE_DIR}")' \
          --replace-fail 'rapids_cpm_init()' \
                         '# Nix: Skip rapids_cpm_init() which tries to download CPM
    # rapids_cpm_init()' \
          --replace-fail 'rapids_cpm_rapids_logger' \
                         '# Nix: Skip rapids_cpm_rapids_logger which uses CPM
    # rapids_cpm_rapids_logger' \
          --replace-fail 'create_logger_macros' \
                         '# Nix: Skip create_logger_macros - not needed
    # create_logger_macros'
  '';

  preConfigure = ''
    cd cpp

    # Create VERSION and RAPIDS_BRANCH files
    echo "${version}" > ../VERSION
    echo "branch-25.10" > ../RAPIDS_BRANCH

    # Set RAPIDS_CMAKE_DIR for patched CMakeLists.txt
    export RAPIDS_CMAKE_DIR="${rapids-cmake}/share/cmake/rapids-cmake"

    # Point CMake to CCCL
    export CMAKE_PREFIX_PATH="${cccl}:$CMAKE_PREFIX_PATH"
  '';

  nativeBuildInputs = [
    cmake
    cudaPackages.cuda_nvcc
  ];

  buildInputs =
    [
      rapids-cmake
      cccl
      spdlog
      fmt
    ]
    ++ (with cudaPackages; [
      cuda_cudart
      libcublas
    ]);

  cmakeFlags = [
    # RAPIDS version variables (normally set by rapids_config.cmake)
    "-DRAPIDS_VERSION=25.10.00"
    "-DRAPIDS_VERSION_MAJOR=25"
    "-DRAPIDS_VERSION_MINOR=10"
    "-DRAPIDS_VERSION_PATCH=00"
    "-DRAPIDS_VERSION_MAJOR_MINOR=25.10"
    "-Drapids-cmake-dir=${rapids-cmake}/share/cmake/rapids-cmake"

    # Use specific CUDA architectures instead of "all"
    # 75=Turing, 80=Ampere, 86=Ampere(Gaming), 89=Ada, 90=Hopper
    "-DCMAKE_CUDA_ARCHITECTURES=75;80;86;89;90"

    # Disable tests and benchmarks (require GPU runtime)
    "-DBUILD_TESTS=OFF"
    "-DBUILD_BENCHMARKS=OFF"

    # Standard install directories
    "-DCMAKE_INSTALL_INCLUDEDIR=include"
    "-DCMAKE_INSTALL_LIBDIR=lib"

    # Don't fetch dependencies
    "-DCPM_DOWNLOAD_ALL=OFF"
  ];

  # Skip tests (require GPU runtime)
  doCheck = false;

  meta = with lib; {
    description = "RAPIDS Memory Manager - GPU memory allocation library";
    homepage = "https://github.com/rapidsai/rmm";
    license = licenses.asl20;
    platforms = platforms.linux;
    maintainers = [];

    # TODO: Complete RAPIDS CPM integration for Nix builds
    # category=I/O estimate=1w
    # Status: Package structure 100% complete, comprehensive patches applied
    #
    # Remaining challenge: RAPIDS packages have deep CPM (CMake Package Manager)
    # integration for dependency fetching. RMM requires CCCL via CPM, despite
    # being provided by Nix. Current patches skip rapids_cpm_init() and related
    # functions, but cmake/thirdparty/get_cccl.cmake still invokes CPM.
    #
    # Completed work:
    # - ✅ Package structure and dependencies
    # - ✅ Source hash verified
    # - ✅ RAPIDS version variables set
    # - ✅ rapids-cmake-dir configured
    # - ✅ VERSION/RAPIDS_BRANCH files
    # - ✅ CPM initialization skipped
    # - ✅ Logger dependencies skipped
    #
    # Possible solutions:
    # 1. Provide CPM.cmake and allow controlled dependency fetching
    # 2. Patch all cmake/thirdparty/get_*.cmake files to skip CPM
    # 3. Use find_package() overrides to inject Nix-provided dependencies
    # 4. Wait for upstream RAPIDS to support system packages better
    #
    # See: docs/CUOPT_NIX_IMPLEMENTATION.md for full implementation details
    broken = true;
  };
}
