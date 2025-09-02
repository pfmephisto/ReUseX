{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
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

  nativeBuildInputs = with pkgs; [
    cmake
  ];

  propagationBuildInputs = with pkgs; [
  ];

  buildInputs = with pkgs; [
    boost
    tbb_2022
    mkl

    eigen
    metis
    #ccache
    #metis
    #gperftools
  ];

  cmakeFlags = [
    "-DGTSAM_USE_SYSTEM_EIGEN=ON"
    "-DGTSAM_USE_SYSTEM_METIS=ON"
  ];

  meta = with lib; {
    description = "GTSAM is a library of C++ classes that implement smoothing and mapping (SAM) in robotics and vision, using factor graphs and Bayes networks as the underlying computing paradigm rather than sparse matrices. ";
    license = licenses.mit;
    maintainers = with maintainers; [];
  };
}
