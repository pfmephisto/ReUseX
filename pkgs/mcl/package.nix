# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  bash,
  lib,
  stdenv,
  bash,
  autoreconfHook,
  zoem,
  cimfomfa,
  ...
}:
stdenv.mkDerivation rec {
  pname = "mcl";
  version = "22-282";

  src = fetchFromGitHub {
    owner = "micans";
    repo = "${pname}";
    rev = "rel-${version}";
    sha256 = "sha256-CrNFynaukvGWBo/nrsBgegpSuFZxFKnq/sqjc2SZvq8=";
  };

  postPatch = ''
    substituteInPlace shed/setversion \
      --replace-fail "#!/bin/bash -e" "#!${bash}/bin/bash -e" \
  '';

  preAutoreconf = ''
    cp configure.ac.in configure.ac
  '';

  preConfigure = ''
    ./shed/setversion ${version}
  '';

  postConfigure = ''
    substituteInPlace Makefile \
      --replace "doc" ""
  '';

  nativeBuildInputs = [
    autoreconfHook
    zoem
  ];

  buildInputs = [
  ];

  propagatedBuildInputs = [
    cimfomfa
  ];

  postInstall = ''
    mkdir -p $out/src/
    cp -r src/ $out/
  '';

  meta = with lib; {
    description = "MCL, the Markov Cluster algorithm, also known as Markov Clustering, is a method and program for clustering weighted or simple networks, a.k.a. graphs.";
    license = licenses.gpl3;
    maintainers = with maintainers; [];
  };
}
