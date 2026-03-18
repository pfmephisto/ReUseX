# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  lib,
  stdenv,
  autoreconfHook,
  ...
}:
stdenv.mkDerivation rec {
  pname = "cimfomfa";
  version = "21-361";

  src = fetchFromGitHub {
    owner = "micans";
    repo = "${pname}";
    rev = "773de7390a7140f8aeb476e059b0ee634c250903";
    sha256 = "sha256-DAAnuBY93y+sxX5X0mEVf11KAtPH9Bcr59JU6gNPiIc=";
  };

  preAutoreconf = ''
    cp configure.ac.in configure.ac
  '';

  nativeBuildInputs = [
    autoreconfHook
  ];

  meta = with lib; {
    description = "Another blooming C utility library.";
    license = licenses.gpl2;
    maintainers = with maintainers; [];
  };
}
