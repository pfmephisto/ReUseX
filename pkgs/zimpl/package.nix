# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  stdenv,
  cmake,
  fetchFromGitHub,
  bison,
  flex,
  gmp,
  ...
}:
stdenv.mkDerivation rec {
  pname = "zimpl";
  version = "3.6.2";

  src = fetchFromGitHub {
    owner = "scipopt";
    repo = "${pname}";
    rev = "v362";
    sha256 = "sha256-juqAwzqBArsFXmz7L7RQaE78EhQdP5P51wQFlCoo7/o=";
  };

  nativeBuildInputs = [cmake];

  buildInputs = [
    bison
    flex
    gmp
  ];
}
