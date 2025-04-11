{
  pkgs,
  fetchFromGitHub,
  stdenv,
  lib,
  ...
}: let
  polyscopeConfig = ./polyscopeConfig.cmake;
in
  stdenv.mkDerivation rec {
    pname = "polyscope";
    version = "2.4.0";
    # version = "36da8ec";

    src = fetchFromGitHub {
      owner = "nmwsharp";
      repo = "${pname}";
      # rev = "v${version}";
      rev = "v${version}";
      fetchSubmodules = true;
      sha256 = "sha256-Cq/MHzw9lCSTmGsD6gxAciKtJDQPQAxWC/GU5v2Q9so="; # polyscope-2.4.0
      # sha256 = "sha256-pViqQ/7F0+7LfVVNkb/Yb/iOdOm1puY8QEoNip0LsYk="; # polyscope-2.3.0
      #sha256 = "sha256-QpbOjq5eKMDItyktFKyQ++2mM2h9onraNTM3jyOa1pA="; # Has does not match any more 
      # hash = "sha256-QpbOjq5eKMDItyktFKyQ++2mM2h9onraNTM3jyOa1pA="; # polyscope-36da8ec
      # hash = lib.fakeHash;
    };

    # outputs = [ "out" "dev" ];

    # nativeBuildInputs = with pkgs; [
    #     cmake
    # ];

    installPhase = ''
      mkdir -p $out/src
      cp -r $src/* $out/src/

      mkdir -p $out/share/cmake/polyscope
      cp ${polyscopeConfig} $out/share/cmake/polyscope/polyscopeConfig.cmake
    '';

    propagatedBuildInputs = with pkgs; [
      xorg.libX11
      xorg.libXrandr
      xorg.libXinerama
      xorg.libXcursor
      xorg.libxcb
      xorg.libXi

      # libGL

      libGLU.dev
      mesa
      freeglut.dev
    ];

    # fixupPhase = ''
    #     mkdir -p $out/share/cmake/polyscope
    #     cp ${polyscopeConfig} $out/share/cmake/polyscope/polyscopeConfig.cmake
    # '';

    meta = with pkgs.lib; {
      description = "Polyscope is a C++/Python viewer and user interface for 3D data such as meshes and point clouds.";
      homepage = "https://polyscope.run/";
      license = licenses.mit;
      maintainers = with maintainers; [];
    };
  }
