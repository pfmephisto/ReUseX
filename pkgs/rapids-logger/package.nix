{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  rapids-cmake,
  cpm-cmake,
  spdlog,
  fmt,
}: let
  rapids_cmake_dir = "${rapids-cmake}/share/cmake/rapids-cmake";

  # rapids-logger expects spdlog 1.14.1 via CPM; nixpkgs has a different version.
  # Pre-fetch the exact spdlog source so CPM uses it in-tree.
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
  stdenv.mkDerivation rec {
    pname = "rapids-logger";
    version = "0.1.0";

    src = fetchFromGitHub {
      owner = "rapidsai";
      repo = "rapids-logger";
      rev = "46070bb255482f0782ca840ae45de9354380e298";
      hash = "sha256-/K5/j/1czaOs5G06Gpd+I+3OTDAa6Z+6tS0VW1+yEcI=";
    };

    postPatch = ''
          # Replace rapids_config.cmake with a version that uses local rapids-cmake
          cat > rapids_config.cmake << 'NIXEOF'
      set(_rapids_version 25.04.00)
      if(_rapids_version MATCHES [[^([0-9][0-9])\.([0-9][0-9])\.([0-9][0-9])]])
        set(RAPIDS_VERSION_MAJOR "''${CMAKE_MATCH_1}")
        set(RAPIDS_VERSION_MINOR "''${CMAKE_MATCH_2}")
        set(RAPIDS_VERSION_PATCH "''${CMAKE_MATCH_3}")
        set(RAPIDS_VERSION_MAJOR_MINOR "''${RAPIDS_VERSION_MAJOR}.''${RAPIDS_VERSION_MINOR}")
        set(RAPIDS_VERSION "''${RAPIDS_VERSION_MAJOR}.''${RAPIDS_VERSION_MINOR}.''${RAPIDS_VERSION_PATCH}")
      else()
        message(FATAL_ERROR "Could not determine RAPIDS version.")
      endif()
      NIXEOF

          # Append the Nix-specific rapids-cmake setup (store paths get substituted by Nix)
          echo '# Nix: Use local rapids-cmake instead of downloading' >> rapids_config.cmake
          echo 'set(rapids-cmake-dir "${rapids_cmake_dir}" CACHE INTERNAL "" FORCE)' >> rapids_config.cmake
          echo 'if(NOT "''${rapids-cmake-dir}" IN_LIST CMAKE_MODULE_PATH)' >> rapids_config.cmake
          echo '  list(APPEND CMAKE_MODULE_PATH "''${rapids-cmake-dir}")' >> rapids_config.cmake
          echo 'endif()' >> rapids_config.cmake
          echo 'include("''${rapids-cmake-dir}/rapids-version.cmake")' >> rapids_config.cmake
    '';

    nativeBuildInputs = [cmake];

    buildInputs = [
      rapids-cmake
      cpm-cmake
    ];

    cmakeFlags = [
      "-DCPM_DOWNLOAD_LOCATION=${cpm-cmake}/CPM.cmake"
      "-DCPM_spdlog_SOURCE=${spdlog-src}"
      "-DCPM_fmt_SOURCE=${fmt-src}"
      "-DBUILD_TESTS=OFF"
      "-DBUILD_SHARED_LIBS=ON"
      "-DRAPIDS_LOGGER_HIDE_ALL_SPDLOG_SYMBOLS=OFF"
    ];

    doCheck = false;

    meta = with lib; {
      description = "RAPIDS Logger - ABI-stable spdlog wrapper for RAPIDS libraries";
      homepage = "https://github.com/rapidsai/rapids-logger";
      license = licenses.asl20;
      platforms = platforms.linux;
      maintainers = [];
    };
  }
