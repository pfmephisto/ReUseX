{
  lib,
  stdenv,
  fetchFromGitHub,
}:
stdenv.mkDerivation rec {
  pname = "cccl";
  version = "2.8.0";

  src = fetchFromGitHub {
    owner = "NVIDIA";
    repo = "cccl";
    rev = "v${version}";
    hash = "sha256-QscUJsXhHPiju7nHGMMJZTTcqg0NbFqaXejPXnG4QSA=";
  };

  # Header-only libraries - no build needed
  dontBuild = true;
  dontConfigure = true;

  # Install headers directly
  installPhase = ''
    runHook preInstall

    mkdir -p $out/include

    # Install Thrust headers
    cp -r thrust/thrust $out/include/

    # Install CUB headers
    cp -r cub/cub $out/include/

    # Install libcudacxx headers
    cp -r libcudacxx/include/cuda $out/include/

    # Install CMake config files if they exist
    mkdir -p $out/lib/cmake/cccl
    if [ -d "lib/cmake" ]; then
      cp -r lib/cmake/* $out/lib/cmake/cccl/ || true
    fi

    runHook postInstall
  '';

  meta = with lib; {
    description = "CUDA C++ Core Libraries - Thrust, CUB, and libcudacxx (header-only)";
    homepage = "https://github.com/NVIDIA/cccl";
    license = licenses.asl20;
    platforms = platforms.all;
    maintainers = [];
  };
}
