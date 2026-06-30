# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  opencv = prev.opencv.override {
    enableGtk2 = true;
    enableGtk3 = true;
    enableVtk = true;
    enableTbb = true;
    # tbb = prev.tbb_2022;
    # cuDNN (and thus CUDA) only when the nixpkgs instance has CUDA enabled.
    # OpenCV's own enableCuda already follows config.cudaSupport; enableCudnn is
    # the one that would otherwise force CUDA into a cudaSupport=false build.
    enableCudnn = prev.config.cudaSupport or false;
    enablePython = true;
    enableUnfree = true;
  };
}
