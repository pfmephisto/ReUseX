# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  #buildPythonPackage,
  python3,
  python3Packages,
  fetchPypi,
  fetchFromGitHub,
  ffmpeg-full,
  ffmpeg_4,
  cudaSupport ? false,
  cudatoolkit,
  stdenv,
  cmake,
  fetchurl,
  ninja,
  pkg-config,
  #setuptools,
  #torch,
  #torchvision,
  #numpy,
  #tqdm,
  #hydra-core,
  #iopath,
  #pillow,
}: let
  version = "0.6.0";

  hashes = {
    "3.0.0" = "sha256-T/bFF83KBpZuLUk31r4jc6tvei2QJcSfGjXrAckKWPk=";
    "0.6.0" = "sha256-GeNOvtn1yBQRx8D5zl4fa1PoKBzvhlrxx92uUMdbCfI=";
  };

  bpe_simple_vocab_16e6 = fetchurl {
    url = "https://github.com/facebookresearch/sam3/raw/refs/heads/main/assets/bpe_simple_vocab_16e6.txt.gz";
    hash = "sha256-kkaRrCiOVECSNhFWUq1KolD0ggPeUKnkcipuzUjWgEo=";
  };

  src = fetchFromGitHub {
    owner = "dmlc"; # "johnnynunez";
    repo = "decord"; # "decord2";
    tag = "v${version}";
    fetchSubmodules = true;
    hash = hashes.${version};
  };

  decord_cpp = stdenv.mkDerivation rec {
    inherit src version;
    pname = "decord-cpp";

    nativeBuildInputs = [
      cmake
    ];

    build-system = with python3Packages; [
      setuptools
    ];

    cmakeFlags = [
      (lib.cmakeBool "USECUDA" false) # cudaSupport)
    ];

    buildInputs = [
      ffmpeg_4
      #python3Packages.setuptools
    ]; # ++ lib.optional cudaSupport [ python3Packages.cudatoolkit ];
  };

  decord = python3Packages.buildPythonPackage rec {
    inherit src version;
    pname = "decord";
    format = "pyproject";
    #pyproject = true;

    sourceRoot = "${src.name}/python";
    strictDeps = true;

    build-system = with python3Packages; [
      setuptools
      wheel
      python3Packages.cmake
      python3Packages.ninja
    ];

    dontUseCmakeConfigure = true;

    env = {
      DECORD_LIBRARY_PATH = "${decord_cpp.out}/lib/";
    };

    propagatedBuildInputs = with python3Packages; [
      decord_cpp
      numpy
    ];

    pythonImportsCheck = [
      "decord"
    ];
  };
  #regex = python3Packages.regex.overridePythonAttrs (old: rec {
  #  version = "2025.11.3";
  #  src = fetchFromGitHub {
  #    owner = "mrabarnett";
  #    repo = "mrab-regex";
  #    tag = "2025.11.3";
  #    hash = "sha256-KEn+8DoAAq2OBqnl7vluqn1UPBpIfmO1v4wxKUZrcyA=";
  #  };
  #  checkPhase = '''';
  #  pythonImportsCheck = [];
  #});
in
  python3Packages.buildPythonPackage rec {
    pname = "sam3";
    version = "0.1.2";
    pyproject = true;

    src = fetchPypi {
      inherit pname version;
      hash = "sha256-VeZS0WB3KyWRDOAocOxZEK17ez0jEO8Jo60G4Ge5ycU=";
    };

    build-system = with python3Packages; [
      setuptools
      torch
    ];

    pythonImportsCheck = [
      "sam3"
    ];

    propagatedBuildInputs = with python3Packages; [
      torch
      torchvision
      numpy
      tqdm
      iopath
      pillow
      timm
      ftfy
      huggingface-hub
      einops
      pycocotools
      regex
      decord
    ];

    postInstall = ''
      mkdir -p $out/${python3.sitePackages}/assets
      cp ${bpe_simple_vocab_16e6} $out/${python3.sitePackages}/assets/bpe_simple_vocab_16e6.txt.gz
    '';

    meta = {
      description = "SAM 3: Segment Anything ith Concepts";
      homepage = "http://pypi.org/project/sam3";
      license = with lib.licenses; [
        bsd3
        asl20
      ];
      maintainers = with lib.maintainers; [];
    };
  }
