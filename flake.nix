{
  
  description = "ReUseX";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    pyproject-nix = {
      url = "github:nix-community/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";
  }; # end of inputs

  outputs = { self, nixpkgs, flake-utils, pyproject-nix}:
    flake-utils.lib.eachDefaultSystem (system:  
    let
      inherit (nixpkgs) lib;
      pkgs = import nixpkgs { 
        inherit system; 
       config = {
          # allowUnfree = true;  # Allow unfree packages globally in this flake
          allowUnfreePredicate = pkg: builtins.elem (lib.getName pkg) [
             # Gurobi Solver
             "gurobi" 
             "gurobipy"

             # CUDA Support in PCL an OpenCV
             "cuda_nvcc"
             "cuda_cudart"
             "cuda_cccl"
             "libnpp"
             "libcublas"
             "libcufft"
             "cuda-merged"
             "cuda_cuobjdump"
             "cuda_gdb"
             "cuda_nvdisasm"
             "cuda_nvprune"
             "cuda_cupti"
             "cuda_cuxxfilt"
             "cuda_nvml_dev"
             "cuda_nvrtc"
             "cuda_nvtx"
             "cuda_profiler_api"
             "cuda_sanitizer_api"
             "libcurand"
             "libcusolver"
             "libnvjitlink"
             "libcusparse"
             "cudnn"
           ];
       };
      };

      # hacks = pkgs.callPackage pyproject-nix.build.hacks {};

      # overlay = final: prev: {
      #   # Adapt torch from nixpkgs
      #   torch = hacks.nixpkgsPrebuilt {
      #     from = pkgs.python3Packages.torchWithoutCuda;
      #     prev = prev.torch;
      #   };

      #   other = hacks.nixpkgsPrebuilt {
      #     from = pkgs.python3Packages.torchWithoutCuda;
      #     prev = prev.torch.overrideAttrs(old: {
      #       passthru = old.passthru // {
      #         dependencies = lib.filterAttrs (name: _: ! lib.hasPrefix "nvidia" name) old.passthru.dependencies;
      #       };
      #     });        
      #   };
      # };

      python = pkgs.python3.override {
        packageOverrides = final: prev: let 
          localPackages = self.packages.${system};
        in{
          rhino3dm =  localPackages.rhino3dm;
          specklepy = localPackages.specklepy;
        };
      };

      # Load the pyproject.toml file
      project = pyproject-nix.lib.project.loadPyproject {
        projectRoot = ./.;
      };
    
    in {

      packages = {

        polyscope = pkgs.stdenv.mkDerivation {
          pname = "polyscope";
          version = "2.3.0";

          src = pkgs.fetchFromGitHub {
            owner = "nmwsharp";
            repo = "polyscope";
            rev = "v2.3.0";
            fetchSubmodules = true;
            sha256 = "sha256-pViqQ/7F0+7LfVVNkb/Yb/iOdOm1puY8QEoNip0LsYk=";
            # sha256 = lib.fakeSha256;
          };


          nativeBuildInputs = with pkgs; [
            cmake
          ];

          buildInputs = with pkgs; [
            xorg.libX11
            xorg.libXrandr
            xorg.libXinerama
            xorg.libXcursor
            xorg.libxcb
            xorg.libXi
            libGL
          ];

          fixupPhase = ''
            mkdir -p $out/share/cmake/polyscope
            cat > $out/share/cmake/polyscope/polyscopeConfig.cmake << 'EOF'
            # Findpolyscope.cmake
            #[============[
            Findpolyscope
            ----------
            find polyscope library.
            Set 'polyscope_DIR' to the path of this file.

            Result Variables
            ^^^^^^^^^^^^^^^^
            ``polyscope_FOUND``
            ``polyscope_INCLUDE_DIRS``
            ``polyscope_LIBRARIES``

            Cache Variables
            ^^^^^^^^^^^^^^^
            ``polyscope_INCLUDE_DIR``
                the path include the polyscope.h.

            ]============]

            # use find_path to get a path anchor
            find_path(polyscope_INCLUDE_DIR
                NAMES polyscope.h
                PATHS $out/include/polyscope
                PATH_SUFFIXES polyscope)
            set(polyscope_DIR $out)

            if($${polyscope_INCLUDE_DIR} STREQUAL polyscope-NOTFOUND)
              message("can't found polyscope")
              set(polyscope_FOUND False)
            else()
              message("found polyscope at path:" $${polyscope_INCLUDE_DIR})
              
              set(polyscope_FOUND True)
              set(polyscope_INCLUDE_DIRS $${polyscope_DIR}/include
                                        $${polyscope_DIR}/include/render
                                        $${polyscope_DIR}/include/render/mock_opengl
                                        $${polyscope_DIR}/include/render/opengl
                                        $${polyscope_DIR}/include/render/opengl/shaders
              )
              set(polyscope_LIBRARIES $${polyscope_DIR}/lib

              )
            endif()
            EOF
          '';


          meta = with pkgs.lib; {
            description = "Polyscope is a C++/Python viewer and user interface for 3D data such as meshes and point clouds.";
            homepage = "https://polyscope.run/";
            license = licenses.mit;
            maintainers = with maintainers; [  ];
          };};

        opennurbs = pkgs.stdenv.mkDerivation {
          pname = "opennurbs";
          version = "v8.13.24317.13001";

          src = pkgs.fetchFromGitHub {
            owner = "mcneel";
            repo = "opennurbs";
            rev = "v8.13.24317.13001";
            fetchSubmodules = true;
            sha256 = "sha256-Q+ExlsJqsjUXQs8le/bjp8nw6I10W0YWJUNgAjKTNXg=";
            # sha256 = lib.fakeSha256;
          };

          nativeBuildInputs = with pkgs; [
            cmake
          ];

          fixupPhase = ''
            mkdir -p $out/share/cmake/opennurbs
            cat > $out/share/cmake/opennurbs/opennurbsConfig.cmake << 'EOF'

            set(polyscope_DIR $${opennurbs_INCLUDE_DIR} $out)

            if($${opennurbs_INCLUDE_DIR} STREQUAL opennurbs-NOTFOUND)
              message("can't found opennurbs")
              set(opennurbs_FOUND False)
            else()
              message("found opennurbs at path:" $${opennurbs_INCLUDE_DIR})
              
              set(opennurbs_FOUND True)
              set(opennurbs_INCLUDE_DIRS $out/lnclude/OpenNURBS
                                         $out/lnclude/opennurbsStatic
              )
              set(opennurbs_LIBRARIES $out/lib
              )

            endif()
            EOF
          '';



          # installPhase = ''
          # echo AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
          # ls -lp 
          # echo ""
          # # make install
          # '';

          # installPhase = ''
          #   mkdir -p $out/include
          #   cp -r * $out/include

          #   # Create the CMake module directory
          #   mkdir -p $dev/share/cmake/OpenNURBS
          #   # Write the polyscopeConfig.cmake file to this directory
          #   cat > $dev/share/cmake/OpenNURBS/OpenNURBSConfig.cmake << 'EOF'
          #   # FindOpenNURBS.cmake
          #   #[============[
          #   FindOpenNURBS
          #   --------------
          #   Find the OpenNURBS library and headers.

          #   This module searches for the OpenNURBS library and sets the following variables:

          #   Result Variables
          #   ^^^^^^^^^^^^^^^^
          #   - `OpenNURBS_FOUND`: Set to `True` if OpenNURBS is found.

          #   Cache Variables
          #   ^^^^^^^^^^^^^^^
          #   - `OpenNURBS_INCLUDE_DIRS`: Directories containing OpenNURBS header files.
          #   - `OpenNURBS_LIBRARIES`: Paths to the OpenNURBS library files.

          #   ]============]

          #   # Locate the OpenNURBS headers
          #   find_path(OpenNURBS_INCLUDE_DIR
          #     NAMES opennurbs.h
          #     PATHS $${CMAKE_CURRENT_LIST_DIR}/../include
          #     PATH_SUFFIXES opennurbs
          #   )

          #   # Locate the OpenNURBS library
          #   find_library(OpenNURBS_LIBRARY
          #     NAMES opennurbs
          #     PATHS $${CMAKE_CURRENT_LIST_DIR}/../lib
          #   )

          #   # Set result variables
          #   if (OpenNURBS_INCLUDE_DIR AND OpenNURBS_LIBRARY)
          #     set(OpenNURBS_FOUND True)
          #     message(STATUS "Found OpenNURBS: $${OpenNURBS_LIBRARY}")
          #   else()
          #     set(OpenNURBS_FOUND False)
          #     message(WARNING "Could not find OpenNURBS")
          #   endif()

          #   # Set include directories and libraries
          #   if (OpenNURBS_FOUND)
          #     set(OpenNURBS_INCLUDE_DIRS $${OpenNURBS_INCLUDE_DIR})
          #     set(OpenNURBS_LIBRARIES $${OpenNURBS_LIBRARY})
          #   endif()

          #   # Mark as advanced so these variables aren't accidentally overridden
          #   mark_as_advanced(OpenNURBS_INCLUDE_DIR OpenNURBS_LIBRARY)
          #   EOF
          # '';

          meta = with lib; {
            description = "OpenNURBS libraries allow anyone to read and write the 3DM file format without the need for Rhino. ";
            license = licenses.mit;
            maintainers = with maintainers; [  ];
          };};
        rhino3dm = pkgs.python3Packages.buildPythonPackage rec {

          pname = "rhino3dm";
          version = "8.9.0";
          format = "setuptools";

          src = pkgs.python.pkgs.fetchPypi {
            pname = "rhino3dm";
            version = "8.9.0";
            sha256 = "sha256-sB4J26Va/QDX89w9UlR9PFETBKpH/M+yoElUJ+rU/7I=";
            # sha256 = lib.fakeSha256;
          };



          preConfigure = ''
            echo changing in to src directory
            cd src
          '';

          postConfigure = ''
            cd ..
            echo "moving $cmakeBuildDir one level up (should be project root)"
            mv $cmakeBuildDir ../
            echo changing current directory back to root.
            cd ..
          '';

          nativeBuildInputs = with pkgs; [
            python3Packages.setuptools
            python3Packages.distutils
            cmake
          ];

          # cmakeFlags = [
          #   # "-DROOT_PATH=src/"
          # ];

          # doCheck = false;

          meta = with lib; {
            description = "";
            license = licenses.bsd3;
            maintainers = with maintainers; [  ];
          };};
        specklepy = pkgs.python3.pkgs.buildPythonPackage {

          pname = "specklepy";
          version = "2.21.2";
          format = "pyproject";

          src = pkgs.python3.pkgs.fetchPypi {
            pname = "specklepy";
            version = "2.21.2";
            sha256 = "sha256-ukS7kWfO0y5Sun5XF9Cx0OzhqiPKAiQ8518Oq6COgQc=";
            # sha256 = lib.fakeSha256;
          };

          doCheck = false;

          nativeBuildInputs = with pkgs; [
            python3Packages.poetry-core
          ];
    

          propagatedBuildInputs = with pkgs.python3.pkgs; [
            deprecated
            appdirs
            (
              pkgs.python3.pkgs.buildPythonPackage {
                pname = "httpx";
                version = "0.26.0";
                format = "pyproject";
                
                src = pkgs.python3.pkgs.fetchPypi {
                  pname = "httpx";
                  version = "0.25.2";
                  sha256 = "sha256-i4/KoMjqewXt1poJTmOiCUxO/LSBKft1c2G8QjwK2eg=";
                  # sha256 = lib.fakeSha256;
                };
                doCheck = false;

                build-system = with pkgs.python3Packages; [
                  hatch-fancy-pypi-readme
                  hatchling
                ];

              propagatedBuildInputs = with pkgs.python3Packages; [
                anyio
                certifi
                httpcore
                idna
                sniffio
              ];

            }) # httpx<0.26.0,>=0.25.0 not satisfied by version 0.27.2
            gql
            (
              pkgs.python3.pkgs.buildPythonPackage {

                pname = "attrs";
                version = "24.0.0";
                disabled = pythonOlder "3.7";
                format = "pyproject";

                
                src = pkgs.python3.pkgs.fetchPypi {
                  pname = "attrs";
                  version = "23.2.0";
                  sha256 = "sha256-k13DtSnCYvbPduUId9NaS9PB3hlP1B9HoreujxmXHzA=";
                };


                doCheck = false;

                build-system = with pkgs.python3Packages; [
                  hatch-fancy-pypi-readme
                  hatchling
                  hatch-vcs
                ];

                nativeBuildInputs = [ pkgs.python3Packages.hatchling ];

                passthru.tests = {
                  pytest = callPackage ./tests.nix { };
                };
              } 
            ) # attrs<24.0.0,>=23.1.0 not satisfied by version 24.2.0
            pydantic
            stringcase
            ujson
          ];

          meta = with lib; {
            description = "xtensor plugin to read and write images, audio files, numpy (compressed) npz and HDF5";
            license = licenses.bsd3;
            maintainers = with maintainers; [  ];
          };};

        highfive = pkgs.highfive;
        xtensor = pkgs.xtensor;
        xtensor-io = pkgs.stdenv.mkDerivation {
          pname = "xtensor-io";
          version = "0.13.0";

          src = pkgs.fetchFromGitHub {
            owner = "xtensor-stack";
            repo = "xtensor-io";
            rev = "a96674992af48b75c14b1ee6c4580d7abd979afe";
            fetchSubmodules = true;
            sha256 = "sha256-Jm0q7U2rULPVEeluuaKJanNPVNdcfrjYeKdWzWJSMXo=";
            # sha256 = lib.fakeSha256;
          };

          propagatedBuildInputs = with pkgs; [
            xtensor
            ghc_filesystem
          ];

          cmakeFlags = [ "-DCMAKE_INSTALL_LIBDIR=share" ];

          buildInputs = with pkgs; [
            cmake
          ];

          meta = with lib; {
            description = "xtensor plugin to read and write images, audio files, numpy (compressed) npz and HDF5";
            license = licenses.bsd3;
            maintainers = with maintainers; [  ];
          };};
        embree = pkgs.embree;
        eigen = pkgs.eigen;
        cgal = pkgs.cgal;
        pcl =  pkgs.stdenv.mkDerivation rec {

          pname = "pcl";
          version = "1.14.1";

          cudaSupport = true;

          src = pkgs.fetchFromGitHub {
            owner = "PointCloudLibrary";
            repo = "pcl";
            rev = "${pname}-${version}";
            sha256 = "sha256-vu5pG4/FE8GCJfd8OZbgRguGJMMZr9PEEdbxUsuV/5Q=";
            #  sha256 = lib.fakeSha256;
          };

          nativeBuildInputs = with pkgs; [
            pkg-config
            cmake
            libsForQt5.qt5.wrapQtAppsHook
          ] ++ lib.optionals cudaSupport [ pkgs.cudaPackages.cuda_nvcc ];

          buildInputs = with pkgs; [
              eigen
              libusb1
              libpcap
              libsForQt5.qt5.qtbase
              xorg.libXt
          ]
            ++ lib.optionals pkgs.stdenv.hostPlatform.isDarwin [
              Cocoa
              AGL
            ];

          propagatedBuildInputs = with pkgs; [
            boost183
            flann
            libpng
            libtiff
            qhull
            vtk
          ];

          cmakeFlags =
            lib.optionals pkgs.stdenv.hostPlatform.isDarwin [
              "-DOPENGL_INCLUDE_DIR=${pkgs.OpenGL}/Library/Frameworks"
              "-DCMAKE_CXX_STANDARD=17" 
              "-DCMAKE_CUDA_STANDARD=17"
            ]
            ++ lib.optionals cudaSupport [ "-DWITH_CUDA=true" ];

          meta = {
            homepage = "https://pointclouds.org/";
            description = "Open project for 2D/3D image and point cloud processing";
            license = lib.licenses.bsd3;
            maintainers = [ ];
            platforms = with lib.platforms; linux ++ darwin;
          };}; 
        fmt = pkgs.fmt;
        mpfr = pkgs.mpfr;
        mpi = pkgs.mpi;
        opencv = pkgs.opencv.override {
            enableCuda = true;}; 
        gurobi = pkgs.gurobi;
        scip = pkgs.scip;
        tbb = pkgs.tbb;


        # ReUseX
        default =
          let
            # Returns an attribute set that can be passed to `buildPythonPackage`.
            attrs = project.renderers.buildPythonPackage { inherit python;  };

            localPackages = self.packages.${system};
          in
          python.pkgs.buildPythonPackage (attrs // {


            # patchPhase = ''
            #   # echo "Updating submodules"
            #   git init .
            #   git submodule update --init --recursive
            # '';


            # Build dependencies
            propagatedBuildInputs = (attrs.propagatedBuildInputs or [
            ]) ++ ( with pkgs; [ 
              # specklepy
              # pkgs.tbb.dev

            ]);


            effectiveStdenv = pkgs.cudaPackages.backendStdenv;


            # Rutime dependencies
            # These are often programs and libraries used by the new derivation at run-time
            buildInputs = (attrs.buildInputs or [
            ]) ++ (with pkgs; [
              localPackages.polyscope
              localPackages.opennurbs
              localPackages.highfive
              localPackages.xtensor
              localPackages.xtensor-io
              localPackages.embree
              localPackages.eigen
              localPackages.cgal
              localPackages.pcl
              localPackages.fmt
              localPackages.mpfr
              localPackages.mpi
              localPackages.opencv
              localPackages.gurobi
              localPackages.scip
              localPackages.tbb

              cudaPackages.cudatoolkit
              cudaPackages.cuda_cudart
              cudaPackages.cuda_cccl # <thrust/*>
              cudaPackages.libnpp # npp.h
              nvidia-optical-flow-sdk
              cudaPackages.cudnn
              cudaPackages.libcublas
              cudaPackages.libcufft
              cudaPackages.cuda_nvcc

              glfw
              glm
              python3Packages.pybind11
            ]);


            # Native dependencies
            # programs and libraries used at build-time
            nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ ( with pkgs;[
              cmake
            ]);

            # Pass additional CMake flags
            cmakeFlags = [
              "-DCUDA_LIB=${pkgs.cudaPackages.cudatoolkit}/lib64/stubs/libcuda.so"
              "-DCUDA_TOOLKIT_ROOT_DIR=${pkgs.cudaPackages.cudatoolkit}"
              # "-DCMAKE_PREFIX_PATH=${localPackages.polyscope}/include/polyscope"
              "-DHighFive_DIR=${localPackages.highfive}/share/HighFive/CMake"
            ];

            # Enviroment variables
            env = {
              # "GUROBI_HOME" = "${pkgs.gurobi}/lib";
            };

          }); # end of ReUseX

      

      }; # end of packages
      

      devShells.default =
      let
        arg = project.renderers.withPackages { inherit python; };
        pythonEnv = python.withPackages arg;
      in
       pkgs.mkShellNoCC {
        packages = with pkgs; [
          pythonEnv
        ];

        shellHook = ''
          echo "Entering dev shell"
        '';
      }; # end of devShells

    }); # end of outputs
}