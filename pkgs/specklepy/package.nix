{
  fetchFromGitHub,
  pkgs,
  python3Packages,
  lib,
  ...
}:
pkgs.python3Packages.buildPythonPackage rec {
  pname = "specklepy";
  version = "3.0.0a7";
  pyproject = true;

  src = fetchFromGitHub {
    owner = "specklesystems";
    repo = "specklepy";
    rev = "${version}";
    sha256 = "sha256-Jyo9XGADWeIWXnJwh0n72KSEbbq0jsCyOtKr+A/y6Og="; # 3.0.0a7
  };

  doCheck = false;

  nativeBuildInputs = with pkgs; [
    python3Packages.poetry-core
    python3Packages.hatchling
    python3Packages.hatch-vcs
  ];

  propagatedBuildInputs = with pkgs.python3Packages; [
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
    description = "xtensor plugin to read and write images, audio files, numpy (compressed) npz and HDF5";
    license = licenses.bsd3;
    maintainers = with maintainers; [];
  };
}
