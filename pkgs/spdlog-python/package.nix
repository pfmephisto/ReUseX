{
  fetchPypi,
  fetchFromGitHub,
  pkgs,
  python3Packages,
  callPackage,
  lib,
  ...
}: let
  buildPythonPackage = python3Packages.buildPythonPackage;

in
  buildPythonPackage rec {
    pname = "spdlog";
    version = "2.0.6";
    pyproject = true;

    src = fetchPypi rec {
        inherit pname version;
        sha256 = "sha256-l5HPF/H9IxaMCggDFws8v4Y9foGa3usl4iTyQuP57O8=";
    };

    postPatch = ''
      substituteInPlace setup.py \
        --replace-fail "'pytest-runner'" ""
    '';

    doCheck = false;

    propagatedBuildInputs = with pkgs.python3Packages; [
      wheel
      setuptools
      pybind11
    ];

    meta = with lib; {
      description = "python wrapper around C++ spdlog logging library";
      license = licenses.mit;
      maintainers = with maintainers; [];
    };
  }
