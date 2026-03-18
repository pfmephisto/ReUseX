# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  lib,
  stdenv,
  cmake,
  onetbb,
  boost,
  mkl,
  eigen,
  metis,
  ...
}:
stdenv.mkDerivation rec {
  pname = "gtsam";
  version = "4.2";

  src = fetchFromGitHub {
    owner = "borglab";
    repo = "${pname}";
    rev = "${version}";
    sha256 = "sha256-HjpGrHclpm2XsicZty/rX/RM/762wzmj4AAoEfni8es=";
  };

  nativeBuildInputs = [
    cmake
  ];

  propagationBuildInputs = [
    onetbb
  ];

  buildInputs = [
    boost
    mkl

    eigen
    metis
    #ccache
    #metis
    #gperftools
  ];

  cmakeFlags = [
    (lib.cmakeFeature "CMAKE_POLICY_VERSION_MINIMUM" "3.5")
    "-DGTSAM_USE_SYSTEM_EIGEN=ON"
    "-DGTSAM_USE_SYSTEM_METIS=ON"
  ];

  meta = with lib; {
    description = "GTSAM is a library of C++ classes that implement smoothing and mapping (SAM) in robotics and vision, using factor graphs and Bayes networks as the underlying computing paradigm rather than sparse matrices. ";
    license = licenses.mit;
    maintainers = with maintainers; [];
  };
}
