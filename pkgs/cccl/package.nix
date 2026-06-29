{
  lib,
  stdenv,
  fetchFromGitHub,
}:
stdenv.mkDerivation rec {
  pname = "cccl";
  version = "3.0.3";

  src = fetchFromGitHub {
    owner = "NVIDIA";
    repo = "cccl";
    rev = "8c04b6539859932f5602e86d38314e4d87f96420";
    hash = "sha256-4MUZSPa/e/lD5FtKdPH6m7qvFd6tkHxy7wPWDq74KAk=";
  };

  # Header-only libraries - no build needed
  dontBuild = true;
  dontConfigure = true;

  installPhase = ''
    runHook preInstall

    mkdir -p $out/include

    # Install Thrust headers
    cp -r thrust/thrust $out/include/

    # Install CUB headers
    cp -r cub/cub $out/include/

    # Install libcudacxx headers (cuda/ and nv/ directories)
    cp -r libcudacxx/include/* $out/include/

    # Install CMake config files for all components
    # The cccl-config.cmake uses relative paths: ../thrust/, ../cub/, ../libcudacxx/
    # so all must be under the same lib/cmake/ parent
    mkdir -p $out/lib/cmake
    cp -r lib/cmake/cccl $out/lib/cmake/
    cp -r lib/cmake/thrust $out/lib/cmake/
    cp -r lib/cmake/cub $out/lib/cmake/
    cp -r lib/cmake/libcudacxx $out/lib/cmake/

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
