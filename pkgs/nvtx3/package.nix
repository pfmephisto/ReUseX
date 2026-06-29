{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
}:
stdenv.mkDerivation rec {
  pname = "nvtx3";
  version = "3.2.0";

  src = fetchFromGitHub {
    owner = "NVIDIA";
    repo = "NVTX";
    rev = "69c9949150ac1c310758a304082228a36d5e4758";
    hash = "sha256-uB1HHLVoOO0rWOcOqfypdiveagnjDcQQZNP7xBHhCwE=";
  };

  sourceRoot = "${src.name}/c";

  nativeBuildInputs = [cmake];

  cmakeFlags = [
    "-DBUILD_EXAMPLES=OFF"
  ];

  meta = with lib; {
    description = "NVIDIA NVTX3 - Tools Extension Library for tracing and annotation (header-only)";
    homepage = "https://github.com/NVIDIA/NVTX";
    license = licenses.asl20;
    platforms = platforms.all;
    maintainers = [];
  };
}
