# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  stdenv,
  # buildPythonPackage,
  python3Packages,
  fetchFromGitHub,
  nlohmann_json,
  config,
  cudaPackages,
  cudaSupport ? config.cudaSupport,
  g2o,
  python3,
  ...
}: let
  g2o-pymem = g2o.overrideAttrs (old: {
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

    buildInputs =
      (old.buildInputs or [])
      ++ [
        python3Packages.pybind11
        nlohmann_json
      ];

    cmakeFlags =
      (old.cmakeFlags or [])
      ++ [
        "-DG2O_BUILD_PYTHON=ON"
      ];
  });
in
  python3Packages.buildPythonPackage rec {
    stdenv =
      if cudaSupport
      then cudaPackages.backendStdenv
      else stdenv;

    pname = g2o-pymem.pname;
    version = g2o-pymem.version;

    dontUnpack = true;
    pyproject = false;

    propagatedBuildInputs = [
      python3
    ];

    installPhase = ''
      mkdir -p "$out/${python3.sitePackages}/${pname}"
      export PYTHONPATH="$out/${python3.sitePackages}:$PYTHONPATH"

      cp -r ${g2o-pymem}/g2o/* $out/${python3.sitePackages}/${pname}
      touch $out/${python3.sitePackages}/${pname}/__init__.py
      # echo "from .g2opy import g2o as g2o">> $out/${python3.sitePackages}/${pname}/__init__.py
      echo "from .g2opy import *" >> $out/${python3.sitePackages}/${pname}/__init__.py

      mkdir -p "$out/${python3.sitePackages}/${pname}-${version}.dist-info"
      # Create a minimal METADATA file
      cat > "$out/${python3.sitePackages}/${pname}-${version}.dist-info/METADATA" <<EOF
      Metadata-Version: 2.1
      Name: ${pname}
      Version: ${version}
      Summary: A Python binding for g2o
      EOF
    '';
  }
