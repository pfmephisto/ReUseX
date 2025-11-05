# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  fetchFromGitHub,
  cudaPackages,
  cmake,
  ...
}:
cudaPackages.backendStdenv.mkDerivation rec {
  pname = "rapids-cmake";
  version = "25.10.00";

  src = fetchFromGitHub {
    owner = "rapidsai";
    repo = "rapids-cmake";
    rev = "v${version}";
    hash = "sha256-oDI297jMUUdnV+kX3qvNnhiUWNP4oCvXBHbTyR925yg=";
  };

  nativeBuildInputs = [cmake];
}
