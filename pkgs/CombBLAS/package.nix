{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
  ...
}:
stdenv.mkDerivation rec {
  pname = "CombBLAS";
  version = "2.0.0";

  src = fetchFromGitHub {
    owner = "PASSIONLab";
    repo = "${pname}";
    rev = "v${version}";
    sha256 = "sha256-EANubRd2IZcjFoSJFqm6GoeAeWakI8yOewF7qPClmS4=";
  };

  nativeBuildInputs = [pkgs.cmake];

  buildInputs = with pkgs; [
    mpi
  ];

  meta = with lib; {
    description = "The Combinatorial BLAS (CombBLAS) is an extensible distributed-memory parallel graph library offering a small but powerful set of linear algebra primitives specifically targeting graph analytics. ";
    license = licenses.gpl3;
    maintainers = with maintainers; [];
  };
}
