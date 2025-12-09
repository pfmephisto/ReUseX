# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  stdenv,
  fetchFromGitHub,
  lib,
  cmake,
  onetbb,
  soplex,
  zlib,
  readline,
  gmp,
  papilo,
  zimpl,
  ipopt,
  boost,
  ...
}:
stdenv.mkDerivation rec {
  pname = "scip-solver";
  version = "923";

  src = fetchFromGitHub {
    owner = "scipopt";
    repo = "scip";
    rev = "v920";
    sha256 = "sha256-F9PBnPuGh+vDYuBL9R0pWg0PUiDrrKT4YdOH1K22dRk=";
  };

  nativeBuildInputs = [cmake];

  propagatedBuildInputs = [onetbb];

  strictDeps = true;

  doCheck = false;

  buildInputs = [
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
