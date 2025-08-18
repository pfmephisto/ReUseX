{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
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
    (pkgs.fetchpatch {
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

  nativeBuildInputs = with pkgs; [
    autoreconfHook
  ];

  buildInputs = with pkgs; [
    cimfomfa
  ];

  meta = with lib; {
    description = "A macro/programming language with stacks.";
    license = licenses.gpl2;
    maintainers = with maintainers; [];
  };
}
