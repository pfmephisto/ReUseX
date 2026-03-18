# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  python3Packages,
  lib,
  ...
}:
python3Packages.buildPythonPackage rec {
  pname = "specklepy";
  version = "3.0.10";
  pyproject = true;

  src = fetchFromGitHub {
    owner = "specklesystems";
    repo = "specklepy";
    rev = "${version}";
    sha256 = "sha256-3MQQH/LtH3mdcdHrYxqS955BF9Q6C2wn5CjhkZASlh4="; # 3.0.0a7
  };

  doCheck = false;

  nativeBuildInputs = [
    python3Packages.poetry-core
    python3Packages.hatchling
    python3Packages.hatch-vcs
  ];

  propagatedBuildInputs = with python3Packages; [
    appdirs
    attrs
    deprecated
    gql
    httpx
    pydantic
    pydantic-settings
    ujson
    stringcase
    requests-toolbelt
    websockets
    requests
  ];

  meta = with lib; {
    description = "The Speckle Python SDK";
    license = licenses.bsd3;
    maintainers = with maintainers; [];
  };
}
