{
  lib,
  fetchFromGitHub,
  cudaPackages,
  cmake,
  cccl,
  rmm,
  rapids-cmake,
}:
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
        # Patch CMakeLists.txt to skip FetchContent and use Nix-provided rapids-cmake
        substituteInPlace cpp/CMakeLists.txt \
          --replace-fail 'include(../rapids_config.cmake)' \
                         '# Nix: Skip rapids_config.cmake which uses FetchContent
    list(APPEND CMAKE_MODULE_PATH "$ENV{RAPIDS_CMAKE_DIR}")' \
          --replace-fail 'rapids_cpm_init()' \
                         '# Nix: Skip rapids_cpm_init() which tries to download CPM
    # rapids_cpm_init()' \
          --replace-fail 'rapids_cpm_rapids_logger' \
                         '# Nix: Skip rapids_cpm_rapids_logger which uses CPM
    # rapids_cpm_rapids_logger' \
          --replace-fail 'rapids_cpm_cuco' \
                         '# Nix: Skip rapids_cpm_cuco - not needed for header-only
    # rapids_cpm_cuco'
  '';

  preConfigure = ''
    cd cpp

    # Create VERSION and RAPIDS_BRANCH files
    echo "${version}" > ../VERSION
    echo "branch-25.10" > ../RAPIDS_BRANCH

    # Set RAPIDS_CMAKE_DIR for patched CMakeLists.txt
    export RAPIDS_CMAKE_DIR="${rapids-cmake}/share/cmake/rapids-cmake"

    # Point CMake to dependencies
    export CMAKE_PREFIX_PATH="${cccl}:${rmm}:$CMAKE_PREFIX_PATH"
  '';

  nativeBuildInputs = [
    cmake
    cudaPackages.cuda_nvcc
  ];

  buildInputs =
    [
      rapids-cmake
      cccl
      rmm
    ]
    ++ (with cudaPackages; [
      cuda_cudart
      libcublas
      libcusolver
      libcusparse
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
    "-DRAFT_BUILD_TESTS=OFF"
    "-DRAFT_BUILD_BENCHMARKS=OFF"

    # Build header-only mode (no compiled libraries)
    "-DRAFT_COMPILE_LIBRARIES=OFF"

    # Standard install directories
    "-DCMAKE_INSTALL_INCLUDEDIR=include"
    "-DCMAKE_INSTALL_LIBDIR=lib"

    # Don't fetch dependencies
    "-DCPM_DOWNLOAD_ALL=OFF"
  ];

  # Header-only mode, but CMake still runs build steps
  # Let it proceed normally

  # Skip tests (require GPU runtime)
  doCheck = false;

  meta = with lib; {
    description = "RAPIDS RAFT - Reusable Accelerated Functions and Tools for vector search and more";
    homepage = "https://github.com/rapidsai/raft";
    license = licenses.asl20;
    platforms = platforms.linux;
    maintainers = [];

    # TODO: Complete RAPIDS CPM integration for Nix builds
    # category=I/O estimate=1w
    # Same challenge as RMM - requires CPM integration resolution.
    # See pkgs/rmm/package.nix and docs/CUOPT_NIX_IMPLEMENTATION.md
    # All package structure and configuration is complete.
    broken = true;
  };
}
