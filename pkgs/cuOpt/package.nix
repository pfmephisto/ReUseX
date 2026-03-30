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
  cccl,
  rmm,
  raft,
  ...
}:
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
        # Patch CMakeLists.txt to skip FetchContent for rapids-cmake
        substituteInPlace cpp/CMakeLists.txt \
          --replace-fail 'include(../cmake/rapids_config.cmake)' \
                         '# Nix: Skip rapids_config.cmake which uses FetchContent' \
          --replace-fail 'include(rapids-cmake)' \
                         'list(APPEND CMAKE_MODULE_PATH "$ENV{RAPIDS_CMAKE_DIR}")
    include($ENV{RAPIDS_CMAKE_DIR}/rapids-cmake.cmake)' \
          --replace-fail 'rapids_cpm_init()' \
                         '# Nix: Skip rapids_cpm_init() which tries to download CPM
    # rapids_cpm_init()' \
          --replace-fail 'rapids_cpm_rapids_logger' \
                         '# Nix: Skip rapids_cpm_rapids_logger which uses CPM
    # rapids_cpm_rapids_logger'
  '';

  preConfigure = ''
    cd cpp

    # Create VERSION and RAPIDS files that rapids_config.cmake expects
    echo "${version}" > ../VERSION
    echo "${version}" > ../RAPIDS_VERSION
    echo "branch-25.10" > ../RAPIDS_BRANCH

    # Set RAPIDS_CMAKE_DIR for patched CMakeLists.txt
    export RAPIDS_CMAKE_DIR="${rapids-cmake}/share/cmake/rapids-cmake"

    # Point CMake to RAPIDS dependencies
    export CMAKE_PREFIX_PATH="${cccl}:${rmm}:${raft}:$CMAKE_PREFIX_PATH"
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
      cccl
      rmm
      raft
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
    "-DBUILD_BENCHMARKS=OFF"

    # Build only LP/MIP solver, not routing engine
    "-DBUILD_LP_ONLY=ON"

    # Standard install directories
    "-DCMAKE_INSTALL_INCLUDEDIR=include"
    "-DCMAKE_INSTALL_LIBDIR=lib"

    # Don't fetch RAPIDS dependencies - use Nix-provided packages
    "-DCPM_DOWNLOAD_ALL=OFF"
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

    # TODO: Complete RAPIDS CPM integration for Nix builds
    # category=I/O estimate=1w
    # Depends on RMM and RAFT, which require CPM integration resolution.
    # See pkgs/rmm/package.nix and docs/CUOPT_NIX_IMPLEMENTATION.md
    # All package structure and configuration is complete.
    broken = true;
  };
}
