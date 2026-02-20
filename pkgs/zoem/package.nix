# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  fetchpatch,
  lib,
  stdenv,
  autoreconfHook,
  cimfomfa,
  ...
}:
stdenv.mkDerivation rec {
  pname = "zoem";
  version = "21-341";

  src = fetchFromGitHub {
    owner = "micans";
    repo = "${pname}";
    rev = "${version}";
    sha256 = "sha256-mZGBnUXCOLytaHzt6hWTUzdpMNcQNni0ezCXC2U8djM=";
  };

  patches = [
    (fetchpatch {
      url = "https://salsa.debian.org/science-team/zoem/-/raw/master/debian/patches/gcc-10.patch?ref_type=heads";
      hash = "sha256-gDHxAtmL4diiJObXI5z9aiHnSrcYTl+J3FTWIb25m98=";
    })
  ];

  preAutoreconf = ''
    cp configure.ac.in configure.ac
  '';

  postConfigure = ''
    # Remove docs from SUBDIRS so make doesn't try to build them
    substituteInPlace Makefile \
      --replace "doc" ""
  '';

  NIX_CFLAGS_COMPILE = ["-std=gnu89"];

  nativeBuildInputs = [
    autoreconfHook
  ];

  buildInputs = [
    cimfomfa
  ];

  meta = with lib; {
    description = "A macro/programming language with stacks.";
    license = licenses.gpl2;
    maintainers = with maintainers; [];
  };
}
