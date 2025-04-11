{
  stdenv,
  # buildPythonPackage,
  fetchFromGitHub,
  config,
  cudaSupport ? config.cudaSupport,
  pkgs,
  ...
}: let
  g2o-pymem = pkgs.g2o.overrideAttrs (old: {
    version = "pymem";

    patches = [
      ./pybind_11.patch
    ];

    src = fetchFromGitHub {
      owner = "RainerKuemmerle";
      repo = "g2o";
      rev = "c203321596a38502cb3008a3883805cb91d3535a";
      sha256 = "sha256-oGOzQpU0BW0KDjUZPK0pYjknio2rC2dQoDVLWrIb+SI=";
    };

    # nativeBuildInputs = (old.nativeBuildInputs or []) ++ (with pkgs; [
    #     git
    # ]);

    buildInputs =
      (old.buildInputs or [])
      ++ (with pkgs; [
        python3Packages.pybind11
        nlohmann_json
      ]);

    cmakeFlags =
      (old.cmakeFlags or [])
      ++ [
        "-DG2O_BUILD_PYTHON=ON"
      ];
  });

  python = pkgs.python3;
in
  pkgs.python3Packages.buildPythonPackage rec {
    stdenv =
      if cudaSupport
      then pkgs.cudaPackages.backendStdenv
      else pkgs.stdenv;

    pname = g2o-pymem.pname;
    version = g2o-pymem.version;

    dontUnpack = true;
    pyproject = false;

    propagatedBuildInputs = with pkgs; [
      python
    ];

    installPhase = ''
      mkdir -p "$out/${python.sitePackages}/${pname}"
      export PYTHONPATH="$out/${python.sitePackages}:$PYTHONPATH"

      cp -r ${g2o-pymem}/g2o/* $out/${python.sitePackages}/${pname}
      touch $out/${python.sitePackages}/${pname}/__init__.py
      # echo "from .g2opy import g2o as g2o">> $out/${python.sitePackages}/${pname}/__init__.py
      echo "from .g2opy import *" >> $out/${python.sitePackages}/${pname}/__init__.py

      mkdir -p "$out/${python.sitePackages}/${pname}-${version}.dist-info"
      # Create a minimal METADATA file
      cat > "$out/${python.sitePackages}/${pname}-${version}.dist-info/METADATA" <<EOF
      Metadata-Version: 2.1
      Name: ${pname}
      Version: ${version}
      Summary: A Python binding for g2o
      EOF
    '';
  }
