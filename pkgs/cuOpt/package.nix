# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  fetchFromGitHub,
  cudaPackages,
  cmake,
  boost,
  bzip2,
  ccache,
  zlib,
  tbb,
  argparse,
  doxygen,
  gtest,
  ...
}: let
  rapids-cmake = fetchFromGitHub {
    owner = "rapidsai";
    repo = "rapids-cmake";
    # rev = "v${version}";
    rev = "v25.10.00";
    hash = "sha256-oDI297jMUUdnV+kX3qvNnhiUWNP4oCvXBHbTyR925yg=";
  };
in
  cudaPackages.backendStdenv.mkDerivation rec {
    pname = "cuOpt";
    version = "25.10.00";

    src = fetchFromGitHub {
      owner = "NVIDIA";
      repo = "cuopt";
      rev = "v${version}";
      hash = "sha256-fXnfxMYW0KPwI3mI+71jy7aEVQHtJxdQyGjOkJV2+/M=";
    };

    preConfigure = ''
      cd cpp
    '';

    nativeBuildInputs = [cmake];

    buildInputs = [
      rapids-cmake
      # rapids-build-backend
      # rapids-dask-dependency
      # rapids-logger

      boost
      bzip2
      ccache
      zlib
      tbb
      argparse
      doxygen
      gtest

      # cxx-compiler
      # c-compiler
      # clang-tools
      # clang
      # cmake
      # make
      # ninja

      # cuda-nvcc
      # cuda-nvtx-dev
      # cuda-python
      # cuda-sanitizer-api
      # cuda-version
      # cudf
      # cupy
      # cuvs
      # libcurand-dev
      # libcusolver-dev
      # libcusparse-dev
      # libraft-headers

      # exhale ?
      # gcc_linux-64
      # gmock
      # librmm
      #libcudss-dev
      #pre-commit
      # rmm
      # scikit-build-core
      # sysroot_linux-64

      # Python dependencies
      # cython
      # breathe
      # fastapi
      # msgpack-numpy==0.4.8
      # msgpack-python==1.1.0
      # myst-nb
      # myst-parser
      # notebook
      # numba-cuda
      # numba
      # numpy
      # numpydoc
      # pandas
      # pexpect
      # pip
      # ipython
      # jsonref
      # pylibraft
      # pyrsistent
      # pytest-cov
      # pytest
      # python
      # requests
      # uvicorn
      # psutil

      # Docs dependencies
      # sphinx
      # sphinx-copybutton
      # sphinx-design
      # sphinx-markdown-tables
      # sphinx_rtd_theme
      # sphinxcontrib-openapi
      # sphinxcontrib-websupport
    ];
  }
