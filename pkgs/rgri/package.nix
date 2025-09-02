{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
  ...
}:
stdenv.mkDerivation rec {
  pname = "rgri";
  version = "25-228";

  src = fetchFromGitHub {
    owner = "GraphBLAS";
    repo = "${pname}";
    rev = "92f306cd5476beaeb218386c8958667e522737bd";
    sha256 = "sha256-f3v9eJK0vM/Uo37tJVb/ZCulfTyAwussbVfdhc1nfhQ=";
  };

  #postPatch = ''
  #  substituteInPlace shed/setversion \
  #    --replace-fail "#!/bin/bash -e" "#!${pkgs.bash}/bin/bash -e" \
  #'';

  #preAutoreconf = ''
  #  cp configure.ac.in configure.ac
  #'';

  #preConfigure = ''
  #  ./shed/setversion ${version}
  #'';

  #postConfigure = ''
  #  substituteInPlace Makefile \
  #    --replace "doc" ""
  #'';

  #nativeBuildInputs = with pkgs; [
  #  autoreconfHook
  #  zoem
  #];

  #buildInputs = with pkgs; [
  #];

  #propagatedBuildInputs = with pkgs; [
  #  cimfomfa
  #];

  installPhase = ''
    ls -lp
    mkdir -p $out/include
    cp -r include/* $out/include/
  '';

  #postInstall = ''
  #  mkdir -p $out/src/
  #  cp -r src/ $out/
  #'';

  meta = with lib; {
    description = "Reference implementation of the draft C++ GraphBLAS specification.";
    license = licenses.bsd3;
    maintainers = with maintainers; [];
  };
}
