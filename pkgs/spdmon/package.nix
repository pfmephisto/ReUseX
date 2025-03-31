{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
  ...
}:
stdenv.mkDerivation rec {
  pname = "spdmon";
  version = "084cf46";

  src = fetchFromGitHub {
    owner = "michalber";
    repo = "${pname}";
    rev = "${version}";
    sha256 = "sha256-M/I2E1GETb3zrGS43lS3/LDUgZ7Va9SgFOCSwD5lMiU=";
  };

  postPatch = ''
    substituteInPlace CMakeLists.txt \
      --replace-fail "add_subdirectory(\''${CMAKE_CURRENT_LIST_DIR}/external/spdlog)" "find_package(fmt REQUIRED)
    set(SPDLOG_FMT_EXTERNAL ON)
    find_package(spdlog REQUIRED)"
    substituteInPlace include/spdmon/spdmon.hpp \
      --replace-fail "#include <spdlog/fmt/bundled/chrono.h>" "#include <fmt/core.h>
    #include <spdlog/fmt/chrono.h>"
  '';

  cmakeFlags = [
    (lib.cmakeBool "spdmon_BUILD_HEADERS_ONLY" true)
    (lib.cmakeBool "spdmon_BUILD_EXECUTABLE" false)
    (lib.cmakeBool "spdmon_GENERATE_EXPORT_HEADER" false)
    (lib.cmakeBool "spdmon_ENABLE_UNIT_TESTING" false)
    (lib.cmakeBool "spdmon_USE_GTEST" false)
    (lib.cmakeBool "spdmon_ENABLE_BENCHMARKING" false)
    (lib.cmakeBool "SPDLOG_FMT_EXTERNAL" true)
  ];

  #dontBuild = true;

  installPhase = ''
  cp -r $src/include ./
  make install
  '';

  #doCheck = false;

  nativeBuildInputs = with pkgs; [
    cmake
    gtest
    spdlog
    fmt_11
  ];



  propagateBuildInputs = with pkgs; [spdlog fmt_11];

  meta = with lib; {
    description = "A simple to use progress bar for C++";
    license = licenses.mit;
    maintainers = with maintainers; [];
  };
}
