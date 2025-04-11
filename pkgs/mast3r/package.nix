{
  fetchFromGitHub,
  lib,
  pkgs,
  ...
}:
pkgs.python3Packages.buildPythonPackage rec {
  pname = "mast3r";
  version = "0.0.1+e06b009";
  pyproject = false;

  stdenv =
    if pkgs.config.cudaSupport
    then pkgs.cudaPackages.backendStdenv
    else pkgs.stdenv;

  src = fetchFromGitHub {
    owner = "naver";
    repo = "${pname}";
    fetchSubmodules = true;
    rev = "e06b0093ddacfd8267cdafe5387954a650af0d3b";
    # sha256 = lib.fakeSha256;
    sha256 = "sha256-xkLHkaQ3YrYETKB16EoUiiz51A9IaAXTihA1QVVg7T8=";
  };

  mast3r_meta_pacakge = pkgs.python3Packages.mkPythonMetaPackage {
    inherit pname version;
  };

  postPatch = ''
    patch -p1 < ${./torch_amp.patch}
  '';

  buildPhase = ''
    cd dust3r/croco/models/curope/
       python setup.py build_ext --inplace
       cd ../../../../
  '';

  env = {
    CUDA_HOME = "${pkgs.cudaPackages.cudatoolkit}";
    TORCH_CUDA_ARCH_LIST = "8.0;8.6+PTX";
    #    CUDA_PATH="${pkgs.cudaPackages.cudatoolkit}";
  };

  buildInputs = with pkgs; [cudaPackages.cudatoolkit];

  installPhase = ''
    # First, call the default install phase to preserve default behavior
    runHook preInstall

    # Then, your custom install commands
    mkdir -p $out/${pkgs.python3.sitePackages}/mast3r
    mkdir -p $out/${pkgs.python3.sitePackages}/dust3r

    cp -r ./mast3r/* $out/${pkgs.python3.sitePackages}/mast3r
    cp -r ./dust3r/* $out/${pkgs.python3.sitePackages}/dust3r

    cp -r ${mast3r_meta_pacakge}/${pkgs.python3.sitePackages}/*  $out/${pkgs.python3.sitePackages}/

    export PYTHONPATH=$out/${pkgs.python3.sitePackages}/mast3r:$PYTHONPATH

    # And then call the post-install hook (if necessary)
    runHook postInstall
  '';

  propagatedBuildInputs = with pkgs.python3Packages;
    [
      torch
      setuptools
      einops
      huggingface-hub
    ]
    ++ (with pkgs; [
      cudaPackages.cudatoolkit
    ]);

  # Metadata (optional)
  meta = with lib; {
    description = "Grounding Image Matching in 3D with MASt3R ";
    license = licenses.cc-by-nc-sa-40;
    homepage = "https://github.com/naver/mast3r";
  };
}
