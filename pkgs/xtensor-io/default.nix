{
    mkDerivation,
    cudaSupport ? config.cudaSupport,
    cudaPackages,
    stdenv,
    fetchFromGitHub,
    pkgs,
    ...
}:
mkDerivation {

    stdenv = if cudaSupport then cudaPackages.backendStdenv else stdenv;

    pname = "xtensor-io";
    version = "0.13.0";

    src = fetchFromGitHub {
        owner = "xtensor-stack";
        repo = "xtensor-io";
        rev = "a96674992af48b75c14b1ee6c4580d7abd979afe";
        fetchSubmodules = true;
        sha256 = "sha256-Jm0q7U2rULPVEeluuaKJanNPVNdcfrjYeKdWzWJSMXo=";
    };

    propagatedBuildInputs = with pkgs; [
        xtensor
        ghc_filesystem
    ];

    cmakeFlags = [ "-DCMAKE_INSTALL_LIBDIR=share" ];

    buildInputs = with pkgs; [
        cmake
    ];

    meta = with lib; {
        description = "xtensor plugin to read and write images, audio files, numpy (compressed) npz and HDF5";
        license = licenses.bsd3;
        maintainers = with maintainers; [  ];
    };
}