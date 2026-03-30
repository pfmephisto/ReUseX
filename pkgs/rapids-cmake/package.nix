# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
}:
stdenv.mkDerivation rec {
  pname = "rapids-cmake";
  version = "25.10.00";

  src = fetchFromGitHub {
    owner = "rapidsai";
    repo = "rapids-cmake";
    rev = "v${version}";
    hash = "sha256-oDI297jMUUdnV+kX3qvNnhiUWNP4oCvXBHbTyR925yg=";
  };

  nativeBuildInputs = [cmake];

  # rapids-cmake is a pure CMake module collection, no compilation
  dontBuild = true;

  installPhase = ''
    runHook preInstall

    # Install CMake modules to standard location
    mkdir -p $out/share/cmake/rapids-cmake
    cp -r ../rapids-cmake/* $out/share/cmake/rapids-cmake/

    # Install VERSION files both in rapids-cmake/ and parent cmake/ directory
    # (rapids-version.cmake looks for ../VERSION relative to rapids-cmake/)
    cp ../VERSION $out/share/cmake/rapids-cmake/
    cp ../VERSION $out/share/cmake/
    cp ../RAPIDS_BRANCH $out/share/cmake/rapids-cmake/ || true
    cp ../RAPIDS_BRANCH $out/share/cmake/ || true

    runHook postInstall
  '';

  meta = with lib; {
    description = "RAPIDS CMake build system utilities";
    homepage = "https://github.com/rapidsai/rapids-cmake";
    license = licenses.asl20;
    platforms = platforms.all;
    maintainers = [];
  };
}
