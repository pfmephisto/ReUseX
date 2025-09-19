# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  cmake,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  ...
}:
stdenv.mkDerivation rec {
  pname = "ReUseX";
  version = "0.0.1";

  src = ../../.;

  nativeBuildInputs = [
    cmake
  ];

  meta = with lib; {
    description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
    license = licenses.gpl3Plus;
    maintainers = with maintainers; [];
  };
}
