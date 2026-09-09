# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# gsplat CUDA rasterization C++ backend (nerfstudio-project/gsplat, Apache-2.0)
# vendored as a standalone C++/CUDA static library built against LibTorch, so
# ReUseX can find_package(gsplat-cuda CONFIG) and link gsplat::gsplat.
#
# Upstream ships no CMake for its csrc/ tree (it builds via torch
# cpp_extension); we drop in pkgs/gsplat-cuda/CMakeLists.txt (mirrors the
# flags from gsplat/cuda/build.py) and point it at gsplat/cuda/.
#
# NOTE: any edit to this file or to the adjacent CMakeLists.txt changes the
# derivation hash and forces a full rebuild from source. Budget for it: the
# 3DGUT rasterizer translation units cost ~20 min of nvcc *per CUDA
# architecture*, so batch package changes rather than making them one at a
# time (see `cudaCapabilities` below).
{
  lib,
  config,
  cmake,
  ninja,
  fetchFromGitHub,
  libtorch,
  protobuf,
  python3,
  python3Packages,
  cudaPackages,
  addDriverRunpath,
  cudaSupport ? config.cudaSupport,
  # Which CUDA architectures the kernels are compiled for (#301). Each entry
  # multiplies nvcc time — the 3DGUT rasterizer TUs alone are ~20 min each —
  # so the previous hard-coded "8.0;8.6;8.9" made every cold build over an
  # hour. This flake declares no substituters, so nothing about this package
  # is cached or shared and a multi-architecture artifact buys nothing; the
  # default is the development machine's GPU (RTX 6000 Ada, sm_89).
  #
  # Other hardware: override just this package, which leaves every other CUDA
  # derivation's hash (and cache hits) alone — unlike setting nixpkgs' global
  # `config.cudaCapabilities`, which rebuilds the whole CUDA closure:
  #
  #   gsplat-cuda.override { cudaCapabilities = ["8.0" "8.6" "8.9"]; }
  cudaCapabilities ? ["8.9"],
}: let
  effectiveStdenv = cudaPackages.backendStdenv;
in
  effectiveStdenv.mkDerivation rec {
    pname = "gsplat-cuda";
    version = "0.1.0-28e794c";

    src = fetchFromGitHub {
      owner = "nerfstudio-project";
      repo = "gsplat";
      rev = "28e794ca44a4c25ffc39175370c5ee7b38bfcc36";
      # glm IS a git submodule (gsplat/cuda/csrc/third_party/glm -> g-truc/glm);
      # the rasterization headers include <glm/gtc/type_ptr.hpp>, so it must be
      # fetched. (googletest submodule comes along too; unused here.)
      fetchSubmodules = true;
      hash = "sha256-MIu6GOkM2Gi1mO4yb0yZZHsDdS6nEuSlOjy0pSJWTvM=";
    };

    # Build only gsplat/cuda/ (the C++/CUDA rasterization backend). Drop our
    # minimal CMakeLists.txt there.
    postPatch = ''
      cp ${./CMakeLists.txt} gsplat/cuda/CMakeLists.txt
    '';

    # Point CMake at the sub-directory that now has our CMakeLists.txt.
    # nixpkgs' cmake setup-hook cd's into build/ under the source root and
    # resolves cmakeDir relative to there, so reach back up into the tree.
    cmakeDir = "../gsplat/cuda";

    nativeBuildInputs = [
      cmake
      ninja
      cudaPackages.cuda_nvcc
      addDriverRunpath
    ];

    buildInputs =
      [
        libtorch
        # Caffe2Config.cmake (pulled in by TorchConfig) needs protobuf; libtorch
        # links it but the config still runs find_package(Protobuf) at configure.
        protobuf
        # A few TUs (PrimingChainEncoding.cu, guarded by GSPLAT_BUILD_3DGUT)
        # include <torch/extension.h>, which transitively needs <Python.h> and
        # <pybind11/pybind11.h>. Headers only -- we compile no pybind module.
        python3
        python3Packages.pybind11
      ]
      ++ (with cudaPackages; [
        cuda_cudart
        cuda_cccl # <thrust/*>, <cub/*> pulled in by torch/glm
        cuda_nvrtc # Torch imported target references CUDA_nvrtc_LIBRARY
        cuda_nvtx
        libcublas
        libcurand
        libcusparse
        libcusolver
        cudnn
      ]);

    # LibTorch's Caffe2Config.cmake needs the ABI/CUDA arch list. Torch_DIR
    # is discovered via CMAKE_PREFIX_PATH (libtorch in buildInputs), but set
    # it explicitly to be robust.
    cmakeFlags = [
      (lib.cmakeOptionType "path" "Torch_DIR" "${libtorch}/share/cmake/Torch")
      (lib.cmakeFeature "CMAKE_CUDA_ARCHITECTURES"
        (lib.concatStringsSep ";" (map cudaPackages.flags.dropDot cudaCapabilities)))
      (lib.cmakeFeature "TORCH_CUDA_ARCH_LIST" (lib.concatStringsSep ";" cudaCapabilities))
      (lib.cmakeFeature "CMAKE_BUILD_TYPE" "Release")
    ];

    # nvcc + torch headers are memory-hungry; keep parallelism modest so a
    # 50-TU CUDA build does not OOM the machine.
    enableParallelBuilding = true;

    meta = with lib; {
      description = "gsplat CUDA rasterization C++ backend as a LibTorch-linked static library";
      homepage = "https://github.com/nerfstudio-project/gsplat";
      license = licenses.asl20;
      platforms = ["x86_64-linux"];
      broken = !cudaSupport;
    };
  }
