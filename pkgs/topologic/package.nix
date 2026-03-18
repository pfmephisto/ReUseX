# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  stdenv,
  cmake,
  fetchFromGitHub,
  ninja,
  opencascade-occt,
  libuuid,
  #python313Packages,
}:
stdenv.mkDerivation rec {
  pname = "Topologic";
  version = "8.0.4";

  src = fetchFromGitHub {
    owner = "wassimj";
    repo = "${pname}";
    rev = "v${version}";
    # fetchSubmodules = true;
    sha256 = "sha256-Q4eIdE7z06wqgGH2I9TSj349v+hsgdM3YX9l+d3KeDw=";
  };

  nativeBuildInputs = [
    cmake
    ninja
  ];

  postPatch = ''
    substituteInPlace CMakeLists.txt \
      --replace "add_subdirectory(TopologicPythonBindings)" ""
  '';

  buildInputs = [
    opencascade-occt
    libuuid
    #python313Packages.pybind11
  ];

  cmakeFlags = [
    (lib.cmakeBool "USE_CONDA_PYBIND11" true)
  ];

  propagatedBuildInputs = [
  ];
}
