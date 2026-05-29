# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# Two fixes layered on top of upstream nixpkgs.openmvs (v2.4.0):
#
# 1. The shipped CMake config (`OpenMVSTargets.cmake`) declares its
#    imported tool targets at `$out/bin/OpenMVS/<Tool>`, but the upstream
#    `postInstall` runs `mv $out/bin/OpenMVS/* $out/bin` which flattens
#    them and leaves the cmake config dangling. Every downstream
#    `find_package(OpenMVS)` then fails with "imported target references
#    file ... but this file does not exist". We override postInstall to
#    preserve the original layout (and just drop the Tests binary, which
#    is what upstream's last line removes).
#
# 2. ReUseX's OpenCV overlay enables CUDA, and OpenCV's installed cmake
#    config eagerly runs `find_package(CUDAToolkit)` in *any* consumer.
#    The upstream openmvs derivation doesn't pull in cudatoolkit, so the
#    rebuild fails with "Could not find nvcc". We add CUDA to
#    nativeBuildInputs/buildInputs so OpenCV's config can satisfy itself
#    (OpenMVS itself does not use CUDA in this build — its own CUDA
#    detection still reports "Can't find CUDA. Continuing without it.",
#    which is fine).
{...}: final: prev: {
  openmvs = prev.openmvs.overrideAttrs (old: {
    nativeBuildInputs =
      (old.nativeBuildInputs or [])
      ++ [prev.cudaPackages.cuda_nvcc];
    buildInputs =
      (old.buildInputs or [])
      ++ [
        prev.cudaPackages.cuda_cudart
        prev.cudaPackages.cuda_cccl
        # OpenMVS's Common library auto-links CUDA::curand whenever CUDA is
        # detected, regardless of whether the build actually uses it.
        prev.cudaPackages.libcurand
      ];
    # Replace the upstream postInstall, which both flattens bin/OpenMVS/*
    # AND removes Tests. The shipped CMake config imports every tool
    # binary including Tests; both changes leave dangling target imports.
    # Leaving the install tree untouched is the simplest fix.
    #
    # Also propagate include directories on the imported targets: OpenMVS's
    # shipped CMake config sets OpenMVS_INCLUDE_DIRS to "" and assigns no
    # INTERFACE_INCLUDE_DIRECTORIES to OpenMVS::{Common,Math,IO,MVS}. That
    # works for OpenMVS's own apps (which add their own -I) but not for
    # downstream consumers — internal headers like Common/Config.h do
    # `#include "ConfigLocal.h"` expecting `$out/include/OpenMVS` on the
    # include path. We append it to every imported library target so
    # consumers don't have to.
    postInstall = ''
      cfg=$out/lib/cmake/OpenMVS/OpenMVSConfig.cmake
      cat >> "$cfg" <<EOF

      # Patched by ReUseX overlay: propagate include dirs on imported
      # library targets so consumers of e.g. #include <OpenMVS/MVS.h>
      # can resolve OpenMVS's internal relative includes.
      foreach(_openmvs_tgt OpenMVS::Common OpenMVS::Math OpenMVS::IO OpenMVS::MVS)
        if(TARGET \''${_openmvs_tgt})
          set_property(TARGET \''${_openmvs_tgt} APPEND PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES "$out/include/OpenMVS")
        endif()
      endforeach()
      set(OpenMVS_INCLUDE_DIRS "$out/include/OpenMVS")
      EOF
    '';

    # The upstream check phase runs a PipelineTest that needs a real GPU.
    # Nix's build sandbox has no GPU access, so it segfaults. The test is
    # meaningful at runtime, not build time — skip it here.
    doCheck = false;
  });
}
