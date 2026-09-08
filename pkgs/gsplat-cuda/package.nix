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
# NOTE: comments in *this* file are free, but any edit to the adjacent
# CMakeLists.txt changes the derivation hash and forces a full rebuild — which
# is over an hour of nvcc (see the CUDA-architectures TODO below). Both known
# issues are therefore recorded here rather than fixed in place.
#
# FIXME: Stop leaking pybind11::headers into gsplat's exported interface
# category=I/O estimate=2h issue=240
# CMakeLists.txt does `target_link_libraries(gsplat PRIVATE pybind11::headers)`.
# For a STATIC library CMake records even PRIVATE dependencies in the exported
# INTERFACE_LINK_LIBRARIES (wrapped in $<LINK_ONLY:>), because a static archive
# must be re-linked transitively. A consumer running find_package(gsplat-cuda)
# without pybind11 already in scope then fails at configure time with "The link
# interface of target gsplat::gsplat contains: pybind11::headers but the target
# was not found". The fix is to consume ${pybind11_INCLUDE_DIRS} as a PRIVATE
# include directory instead of linking the target — pybind11 is only ever
# needed for headers here, never for symbols. Worked around on the consumer
# side by a find_package(pybind11 CONFIG QUIET) ahead of
# find_package(gsplat-cuda) in libs/reusex/cmake/reusexLibrary.cmake.
#
# TODO: Build gsplat for one CUDA architecture instead of three
# category=I/O estimate=2h issue=240
# CMAKE_CUDA_ARCHITECTURES is pinned to 80;86;89, so nvcc compiles every kernel
# three times. The 3DGUT rasterizer TUs (RasterizeToPixelsFromWorld3DGS*Fwd.cu)
# take ~20 min per architecture on their own, making a cold build of this
# package well over an hour and any change to it effectively unaffordable
# mid-session. Expose the architecture list as a derivation argument defaulting
# to nixpkgs' cudaCapabilities so a developer can build only their own GPU's
# architecture, keeping the three-arch list for anything cached and shared.
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
      (lib.cmakeFeature "CMAKE_CUDA_ARCHITECTURES" "80;86;89")
      (lib.cmakeFeature "TORCH_CUDA_ARCH_LIST" "8.0;8.6;8.9")
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
