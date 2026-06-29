{
  lib,
  stdenvNoCC,
  fetchurl,
  autoPatchelfHook,
  cudaPackages,
}:
stdenvNoCC.mkDerivation rec {
  pname = "cudss";
  version = "0.7.1.4";

  src = fetchurl {
    url = "https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-x86_64/libcudss-linux-x86_64-${version}_cuda12-archive.tar.xz";
    hash = "sha256-lGVx2eoWT5SOQC3ZehRUHLkPvsgAM2z6euZEr1k3Yy8=";
  };

  nativeBuildInputs = [autoPatchelfHook];

  buildInputs = with cudaPackages; [
    cuda_cudart
    libcublas
  ];

  dontBuild = true;
  dontConfigure = true;

  installPhase = ''
    runHook preInstall

    mkdir -p $out
    cp -r include $out/
    cp -r lib $out/

    # Remove optional communication layer plugins that require MPI/NCCL
    # (not needed for single-node cuOpt usage)
    rm -f $out/lib/libcudss_commlayer_openmpi*
    rm -f $out/lib/libcudss_commlayer_nccl*

    runHook postInstall
  '';

  meta = with lib; {
    description = "NVIDIA cuDSS - GPU-accelerated direct sparse solver library";
    homepage = "https://developer.nvidia.com/cudss";
    license = licenses.unfree;
    platforms = ["x86_64-linux"];
    maintainers = [];
  };
}
