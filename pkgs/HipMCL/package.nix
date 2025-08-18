{
  fetchFromBitbucket,
  pkgs,
  lib,
  stdenv,
  ...
}:
stdenv.mkDerivation rec {
  pname = "HipMCL";
  version = "ca6a4fb1-dev";

  src = fetchFromBitbucket {
    owner = "azadcse";
    repo = "${pname}";
    rev = "ca6a4fb17d0dc46a6fa7401a1ffdc9ec317de10c";
    sha256 = "sha256-v5UvYrplS7HhZj3vgF4MqMDTGfEiWnMHubVNCBf6RNY=";
  };

  nativeBuildInputs = [pkgs.cmake];

  buildInputs = with pkgs; [
    mpi
  ];

  meta = with lib; {
    description = "HipMCL: A High-Performance Parallel Algorithm for Large-Scale Network Clustering.";
    license = licenses.gpl3;
    maintainers = with maintainers; [];
  };
}
