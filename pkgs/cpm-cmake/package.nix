{
  stdenvNoCC,
  fetchurl,
}:
stdenvNoCC.mkDerivation rec {
  pname = "cpm-cmake";
  version = "0.40.2";

  src = fetchurl {
    url = "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${version}/CPM.cmake";
    hash = "sha256-yM3DLAOBZTjOInge1ylk3IZLKjSjENO3EEgSpcotg10=";
  };

  dontUnpack = true;
  dontBuild = true;
  dontConfigure = true;

  installPhase = ''
    runHook preInstall
    mkdir -p $out
    cp $src $out/CPM.cmake
    runHook postInstall
  '';

  meta = {
    description = "CMake Package Manager (CPM.cmake) - single file for offline builds";
    homepage = "https://github.com/cpm-cmake/CPM.cmake";
  };
}
