# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Prebuilt libtorch (PyTorch C++ API), CUDA 12.8 build. We use the upstream
# prebuilt "shared-with-deps" distribution instead of compiling PyTorch from
# source: the from-source build is enormous and its pinned git submodules became
# unfetchable. cu128 (CUDA 12.8) is ABI/runtime-compatible with this stack's
# CUDA 12.9 (same major); autoPatchelfHook + the cudaPackages buildInputs below
# repoint the bundled libs at the stack's own CUDA runtime.
{
  stdenv,
  lib,
  config,
  fetchurl,
  unzip,
  autoPatchelfHook,
  addDriverRunpath,
  zlib,
  cudaPackages,
  cudaSupport ? config.cudaSupport,
}:
stdenv.mkDerivation rec {
  pname = "libtorch";
  version = "2.9.0";

  src = fetchurl {
    url = "https://download.pytorch.org/libtorch/cu128/libtorch-shared-with-deps-${version}%2Bcu128.zip";
    hash = "sha256-dVVfd4hPFJ1TdJTaaVOjYmzqaJlrB/txHkqD1PSLT9s=";
  };

  nativeBuildInputs =
    [
      unzip
      autoPatchelfHook
    ]
    ++ lib.optionals cudaSupport [addDriverRunpath];

  # Runtime libraries the prebuilt .so's link against. autoPatchelfHook resolves
  # each libtorch .so's NEEDED entries against these and rewrites the rpath.
  buildInputs =
    [
      stdenv.cc.cc.lib # libstdc++/libgcc_s
      zlib
    ]
    ++ lib.optionals cudaSupport (with cudaPackages; [
      cuda_cudart
      cuda_nvrtc
      cuda_cupti
      libcublas
      libcufft
      libcurand
      libcusolver
      libcusparse
      cudnn
      nccl
    ]);

  # Some optional deps (e.g. numa) are dlopen'd, not NEEDED; don't fail on them.
  autoPatchelfIgnoreMissingDeps = true;

  dontBuild = true;
  dontConfigure = true;
  dontStrip = true;

  unpackPhase = ''
    runHook preUnpack
    unzip -q $src
    runHook postUnpack
  '';

  installPhase = ''
    runHook preInstall
    mkdir -p $out
    cp -r libtorch/include $out/
    cp -r libtorch/share $out/

    # The prebuilt libtorch vendors a full copy of fmt's headers under
    # include/fmt/ (fmt 11.2.0 for the 2.9.0 build). Because libtorch's include
    # dir is on every consumer's compile line, that bundled <fmt/...> SHADOWS the
    # stack's own fmt. After the nixpkgs bump to fmt 12 this became fatal: ReUseX
    # code (e.g. reusex_vision's annotate.cpp, via core/logging.hpp) compiled its
    # fmt calls as fmt::v11 against these headers, then failed to link against the
    # fmt 12 .so we actually ship — "undefined reference to fmt::v11::vformat /
    # fmt::v11::report_error". Drop the bundled copy so <fmt/...> resolves to the
    # single nixpkgs fmt (12.x) everywhere; libtorch's own headers use only fmt's
    # stable public API and compile fine against it, and libtorch's .so already
    # has its fmt statically linked in, so nothing needs these headers at runtime.
    rm -rf $out/include/fmt
    install -Dm755 -t $out/lib libtorch/lib/*.so*
    # Drop Java bindings we never use.
    rm -f $out/lib/lib*jni* 2>/dev/null || true
    # TorchConfig.cmake hardcodes ''${TORCH_INSTALL_PREFIX}/lib; with a single
    # output that resolves to $out, so no rewrite is needed.

    # Caffe2's public/mkl.cmake does `find_package(MKL QUIET)` then unconditionally
    # adds ''${MKL_INCLUDE_DIR} to caffe2::mkl's interface includes. When MKL isn't
    # present at the CONSUMER's configure time that becomes "MKL_INCLUDE_DIR-NOTFOUND",
    # a non-existent path that fails CMake generate in downstream projects. The
    # prebuilt libtorch bundles MKL in lib/, so consumers need no MKL headers —
    # guard the include so a not-found MKL doesn't inject a bogus path.
    substituteInPlace $out/share/cmake/Caffe2/public/mkl.cmake \
      --replace-fail \
        'target_include_directories(caffe2::mkl INTERFACE ''${MKL_INCLUDE_DIR})' \
        'if(MKL_INCLUDE_DIR)
  target_include_directories(caffe2::mkl INTERFACE ''${MKL_INCLUDE_DIR})
endif()'

    # Same not-found footgun on the link side: with no MKL at the consumer's
    # configure time, `find_package(MKL QUIET)` leaves MKL_LIBRARIES set to the
    # literal string "FALSE" (nixpkgs' FindMKL), which gets baked into
    # caffe2::mkl's INTERFACE_LINK_LIBRARIES and reaches the final link as a bare
    # `-lFALSE` — `ld: cannot find -lFALSE`. The prebuilt libtorch bundles MKL in
    # lib/ and its .so's already NEED it, so consumers must not add an MKL link
    # item of their own; guard it so a not-found MKL injects nothing.
    substituteInPlace $out/share/cmake/Caffe2/public/mkl.cmake \
      --replace-fail \
        'target_link_libraries(caffe2::mkl INTERFACE ''${MKL_LIBRARIES})' \
        'if(MKL_LIBRARIES)
  target_link_libraries(caffe2::mkl INTERFACE ''${MKL_LIBRARIES})
endif()'

    # And the same on the link-DIRECTORIES side. With no MKL found, MKL_ROOT is
    # empty, so this hardcoded set_property expands to the bogus absolute paths
    # "/lib;/lib/intel64;/lib/intel64_win;/lib/win-x64". They get baked into every
    # downstream binary's RUNPATH, and CMake's install-time RPATH_CHANGE then fails
    # to reconcile them ("could not write new RPATH ... which does not contain
    # /lib:/lib/intel64:..."). Guard it so a not-found MKL contributes no link
    # directories. (Upstream even flags this line as a hack — pytorch#73008.)
    substituteInPlace $out/share/cmake/Caffe2/public/mkl.cmake \
      --replace-fail \
        'set_property(
  TARGET caffe2::mkl PROPERTY INTERFACE_LINK_DIRECTORIES
  ''${MKL_ROOT}/lib ''${MKL_ROOT}/lib/intel64 ''${MKL_ROOT}/lib/intel64_win ''${MKL_ROOT}/lib/win-x64)' \
        'if(MKL_ROOT)
  set_property(
    TARGET caffe2::mkl PROPERTY INTERFACE_LINK_DIRECTORIES
    ''${MKL_ROOT}/lib ''${MKL_ROOT}/lib/intel64 ''${MKL_ROOT}/lib/intel64_win ''${MKL_ROOT}/lib/win-x64)
endif()'
    runHook postInstall
  '';

  postFixup = lib.optionalString cudaSupport ''
    find $out/lib -type f \( -name '*.so' -o -name '*.so.*' \) | while read -r so; do
      addDriverRunpath "$so"
    done
  '';

  meta = {
    description = "C++ API of the PyTorch machine learning framework (prebuilt, CUDA 12.8)";
    homepage = "https://pytorch.org/";
    sourceProvenance = with lib.sourceTypes; [binaryNativeCode];
    license = lib.licenses.bsd3;
    platforms = ["x86_64-linux"];
  };
}
