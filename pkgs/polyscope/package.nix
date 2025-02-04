{
    pkgs,
    fetchFromGitHub,
    stdenv,
    ...
}:
let
    polyscopeConfig = ./polyscopeConfig.cmake;
in
stdenv.mkDerivation rec {

    pname = "polyscope";
    version = "2.3.0";

    src = fetchFromGitHub {
        owner = "nmwsharp";
        repo = "${pname}";
        rev = "v${version}";
        fetchSubmodules = true;
        sha256 = "sha256-pViqQ/7F0+7LfVVNkb/Yb/iOdOm1puY8QEoNip0LsYk=";
    };

    nativeBuildInputs = with pkgs; [
        cmake
    ];

    buildInputs = with pkgs; [
        xorg.libX11
        xorg.libXrandr
        xorg.libXinerama
        xorg.libXcursor
        xorg.libxcb
        xorg.libXi
        libGL
    ];

    fixupPhase = ''
        mkdir -p $out/share/cmake/polyscope
        cp ${polyscopeConfig} $out/share/cmake/polyscope/polyscopeConfig.cmake
    '';


    meta = with pkgs.lib; {
        description = "Polyscope is a C++/Python viewer and user interface for 3D data such as meshes and point clouds.";
        homepage = "https://polyscope.run/";
        license = licenses.mit;
        maintainers = with maintainers; [  ];
    };
}