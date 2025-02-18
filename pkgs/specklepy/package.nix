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
  pythonOlder = python3Packages.pythonOlder;

  httpx-0-26-0 = buildPythonPackage rec {
    pname = "httpx";
    version = "0.26.0";
    format = "pyproject";

    src = fetchPypi {
      pname = "httpx";
      version = "0.25.2";
      sha256 = "sha256-i4/KoMjqewXt1poJTmOiCUxO/LSBKft1c2G8QjwK2eg=";
      # sha256 = lib.fakeSha256;
    };
    doCheck = false;

    build-system = with python3Packages; [
      hatch-fancy-pypi-readme
      hatchling
    ];

    propagatedBuildInputs = with python3Packages; [
      anyio
      certifi
      httpcore
      idna
      sniffio
      requests-toolbelt
      websockets
    ];
  };

  attrs-24-0-0 = buildPythonPackage rec {
    pname = "attrs";
    version = "24.0.0";
    disabled = pythonOlder "3.7";
    format = "pyproject";

    doCheck = false;

    src = fetchPypi {
      pname = "attrs";
      version = "23.2.0";
      sha256 = "sha256-k13DtSnCYvbPduUId9NaS9PB3hlP1B9HoreujxmXHzA=";
    };

    build-system = with python3Packages; [
      hatch-fancy-pypi-readme
      hatchling
      hatch-vcs
    ];

    nativeBuildInputs = with python3Packages; [hatchling];

    passthru.tests = {
      pytest = callPackage ./tests.nix {};
    };
  };
in
  pkgs.python3Packages.buildPythonPackage rec {
    pname = "specklepy";
    version = "2.21.2";
    pyproject = true;

    # src = fetchPypi rec {
    #     inherit pname version;
    #     sha256 = "sha256-ukS7kWfO0y5Sun5XF9Cx0OzhqiPKAiQ8518Oq6COgQc=";
    # };

    patches = [
      ./version.patch
    ];

    #patchPhase

    configurePhase = ''
      ${pkgs.python3}/bin/python ./patch_version.py ${version}
    '';

    src = fetchFromGitHub {
      owner = "specklesystems";
      repo = "specklepy";
      rev = "${version}";
      sha256 = "sha256-SRwINUjrPzx3/uEfAoKx5XE0CbUrx85PD3GJ+vd3rAs=";
    };

    doCheck = false;

    nativeBuildInputs = with pkgs; [
      python3Packages.poetry-core
    ];

    propagatedBuildInputs = with pkgs.python3Packages; [
      deprecated
      appdirs
      gql
      pydantic
      stringcase
      ujson
      httpx-0-26-0 # httpx<0.26.0,>=0.25.0 not satisfied by version 0.27.2
      attrs
      #attrs-24-0-0 # attrs<24.0.0,>=23.1.0 not satisfied by version 24.2.0
    ];

    meta = with lib; {
      description = "xtensor plugin to read and write images, audio files, numpy (compressed) npz and HDF5";
      license = licenses.bsd3;
      maintainers = with maintainers; [];
    };
  }
