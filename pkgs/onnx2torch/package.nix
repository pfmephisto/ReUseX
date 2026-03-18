# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  fetchPypi,
  fetchFromGitHub,
  python3Packages,
  cmake,
  ...
}:
python3Packages.buildPythonPackage rec {
  pname = "onnx2torch";
  version = "1.5.15";

  pyproject = true;

  pythonImportsCheck = ["onnx2torch"];

  src = fetchFromGitHub {
    owner = "ENOT-AutoDL";
    repo = "onnx2torch";
    rev = "v${version}";
    sha256 = "sha256-RqH+IVImbmyjUdIQbIV8bXNwD4vKu5hQoUyMuQ0wiSg=";
  };

  nativeBuildInputs = with python3Packages; [
    numpy
    onnx
    torch
    torchvision
  ];

  meta = with lib; {
    description = "";
    license = licenses.asl20;
    maintainers = with maintainers; [];
  };
}
