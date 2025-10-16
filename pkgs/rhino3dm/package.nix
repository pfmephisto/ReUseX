# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  fetchPypi,
  python3Packages,
  cmake,
  ...
}:
python3Packages.buildPythonPackage rec {
  pname = "rhino3dm";
  version = "8.17.0";
  format = "setuptools";

  src = fetchPypi {
    inherit pname version;
    sha256 = "sha256-htnTw7R71wKG352vQHhYIZFNHgyTOHvjS8vYHZ8+yb8=";
  };

  preConfigure = ''
    echo changing in to src directory
    cd src
  '';

  postConfigure = ''
    cd ..
    echo "moving $cmakeBuildDir one level up (should be project root)"
    mv $cmakeBuildDir ../
    echo changing current directory back to root.
    cd ..
  '';

  nativeBuildInputs = [
    python3Packages.setuptools
    python3Packages.distutils
    cmake
  ];

  meta = with lib; {
    description = "";
    license = licenses.bsd3;
    maintainers = with maintainers; [];
  };
}
