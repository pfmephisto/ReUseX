# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# Route the heavy C++/CUDA package compiles through ccache so that repeated
# rebuilds (nixpkgs bumps, GC, tweaking our own overlays/pkgs) reuse object
# files instead of recompiling from scratch.
#
# Host plumbing lives in the NixOS config (nix-link): `programs.ccache.enable`
# creates /var/cache/ccache (root:nixbld 0770) and `extra-sandbox-paths` makes
# it writable inside the build sandbox. This overlay only wires the compiler
# wrappers; it does not create the directory. If the directory is missing or
# read-only the wrapper transparently falls back to a build-local cache so the
# build still succeeds (just uncached).
#
# Config choices (verified to produce cross-derivation cache hits on this host):
#   compiler_check=content  - store compilers all have mtime=1, so the default
#                             mtime check is unreliable; hash the compiler
#                             contents instead.
#   hash_dir=false          - ignore the build CWD (a fresh /build/... per
#                             derivation) so an identical TU hits regardless of
#                             which sandbox it was first compiled in.
_: final: prev: let
  extraConfig = ''
    export CCACHE_DIR="/var/cache/ccache"
    export CCACHE_MAXSIZE="60G"
    export CCACHE_UMASK=007
    export CCACHE_COMPRESS=1
    export CCACHE_COMPILERCHECK=content
    export CCACHE_NOHASHDIR=1
    export CCACHE_SLOPPINESS=random_seed,time_macros,include_file_mtime,include_file_ctime
    if [ ! -d "$CCACHE_DIR" ] || [ ! -w "$CCACHE_DIR" ]; then
      echo "ccache: '$CCACHE_DIR' unavailable in sandbox, falling back to \$TMPDIR (uncached build)" >&2
      export CCACHE_DIR="$TMPDIR/ccache"
      mkdir -p "$CCACHE_DIR"
    fi
  '';

  # Wrap an arbitrary stdenv's C/C++ compiler with ccache, preserving the rest
  # of the stdenv (matters for cudaPackages.backendStdenv, which pairs a
  # specific gcc with nvcc).
  wrapCcache = base:
    final.overrideCC base (final.ccacheWrapper.override {
      inherit (base) cc;
      inherit extraConfig;
    });
in {
  # Shared, pre-configured ccache stdenv. Consumed here and by flake.nix for the
  # custom (pkgs/*) packages and the CPU ReUseX build.
  ccacheStdenv = wrapCcache prev.stdenv;

  # CUDA host compiler. Any package that resolves cudaPackages.backendStdenv
  # from the (overlaid) top-level set now compiles through ccache: opencv
  # (enableCuda), cuOpt, gsplat-cuda and the CUDA ReUseX build.
  cudaPackages =
    prev.cudaPackages
    // {
      backendStdenv = wrapCcache prev.cudaPackages.backendStdenv;
    };

  # The nixpkgs bump builds SuiteSparse with CUDA (GPU CHOLMOD) enabled, so its
  # public header cholmod.h now #includes <cublas_v2.h>/<cuda_runtime.h>. Every
  # consumer (g2o, ceres, rtabmap, openmvs) then needs those on its include path
  # or fails with "cublas_v2.h: No such file or directory". Propagate them.
  # Keeps SuiteSparse's GPU sparse solvers enabled.
  suitesparse = prev.suitesparse.overrideAttrs (old: {
    propagatedBuildInputs =
      (old.propagatedBuildInputs or [])
      ++ [
        final.cudaPackages.libcublas
        final.cudaPackages.cuda_cudart
        final.cudaPackages.cuda_nvcc # crt/host_defines.h (pulled in by cuda_runtime.h)
      ];
  });

  # Heavy nixpkgs C++ packages that stay on the plain stdenv even under CUDA
  # (they invoke nvcc as a separate tool). Wrapping the stdenv arg is a no-op
  # for any build path that ignores it, so it is safe to apply unconditionally.
  opencv = prev.opencv.override {stdenv = final.ccacheStdenv;};
  pcl = prev.pcl.override {stdenv = final.ccacheStdenv;};
  rtabmap = prev.rtabmap.override {stdenv = final.ccacheStdenv;};
  openmvs = prev.openmvs.override {stdenv = final.ccacheStdenv;};
  highs = prev.highs.override {stdenv = final.ccacheStdenv;};
}
