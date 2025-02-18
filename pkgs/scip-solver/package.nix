{
  stdenv,
  fetchFromGitHub,
  lib,
  pkgs,
  ...
}:
stdenv.mkDerivation rec {
  pname = "scip-solver";
  version = "9.2.0";

  src = fetchFromGitHub {
    owner = "scipopt";
    repo = "scip";
    rev = "v920";
    sha256 = "sha256-F9PBnPuGh+vDYuBL9R0pWg0PUiDrrKT4YdOH1K22dRk=";
  };

  nativeBuildInputs = with pkgs; [cmake];

  propagatedBuildInputs = with pkgs; [tbb];

  strictDeps = true;

  doCheck = true;

  buildInputs = with pkgs; [
    soplex
    zlib
    readline
    gmp
    papilo
    zimpl
    ipopt
    boost
  ];

  meta = {
    homepage = "https://scipopt.org/";
    description = "SCIP - Solving Constraint Integer Programs ";
    #  Apache-2.0 license
    license = with lib.licenses; [asl20];
    # maintainers = with lib.maintainers; [ david-r-cox ];
    platforms = lib.platforms.unix;
  };
}
