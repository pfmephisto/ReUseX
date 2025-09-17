{...}: final: prev: {
  suitesparse-graphblas =
    (prev.suitesparse-graphblas.override {
      #stdenv = prev.cudaPackages.stdenv-linux;
    }).overrideAttrs
    (old: {
      cmakeFlags =
        (old.cmakeFlags or [])
        ++ [
          (prev.lib.cmakeBool "GRAPHBLAS_USE_CUDA" true)
        ];
      buildInputs = (old.buildInputs or []) ++ [prev.cudaPackages.cudatoolkit];
    });
}
