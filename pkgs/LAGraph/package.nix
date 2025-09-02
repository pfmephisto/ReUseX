{
  fetchFromGitHub,
  pkgs,
  lib,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  ...
}:
stdenv.mkDerivation rec {
  pname = "LAGraph";
  version = "1.2.0";

  src = fetchFromGitHub {
    owner = "GraphBLAS";
    repo = "${pname}";
    rev = "v${version}";
    sha256 = "sha256-FjqGrOc67aeWTU+p8JByPNjqcJiwDVhviJv4xtAh3LU=";
  };

  patches = [
    ./1.patch
  ];

  nativeBuildInputs = with pkgs; [
    cmake
    # gfortran
  ];

  buildInputs = with pkgs;
    [
      suitesparse-graphblas
    ]
    ++ lib.optionals cudaSupport [pkgs.cudatoolkit];

  cmakeFlags = [
    (lib.cmakeBool "BUILD_SHARED_LIBS" true)
  ];

  meta = with lib; {
    description = "This is a library plus a test harness for collecting algorithms that use the GraphBLAS.";
    license = licenses.bsd3;
    maintainers = with maintainers; [];
  };
}
