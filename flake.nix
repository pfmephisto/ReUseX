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

        cudaSupport = true;
        allowUnfreePredicate = pkg: builtins.elem (lib.getName pkg) [
          # Python model
          "mast3r"

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
       
       overlays = [
        (final: prev: {
          # Fix the CUDA environment for the suiteSparse package
          suitesparse = prev.suitesparse.override (old: {
            stdenv = if pkgs.config.cudaSupport then pkgs.cudaPackages.backendStdenv else pkgs.stdenv;
            });
          })
        (final: prev: {
          # Change PCL to the main branch
          pcl = prev.pcl.overrideAttrs (old: {
            pname = "pcl";
            # version = "1.14.1";
            version = "6e4eb4e0c1222adcaddd32ccfd6dbcb08746f25c";
            src = pkgs.fetchFromGitHub {
              owner = "PointCloudLibrary";
              repo = "pcl";
              rev = "6e4eb4e0c1222adcaddd32ccfd6dbcb08746f25c";
              sha256 = "sha256-OHzJwTtv+7CR+0UfyP0qB64lzFgUJG/0RWVreWo7KO8=";
              # sha256 = lib.fakeSha256;
            };
          });
         })
        # (final: prev: (prev.lib.packagesFromDirectoryRecursive {
        #     callPackage = prev.lib.callPackageWith final;
        #     directory = ./pkgs;
        #   })
        #  )
       ];
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
          g2opy = localPackages.g2opy;
          mast3r = localPackages.mast3r;
        };
      };

      localPackages = self.packages.${system};

      # Load the pyproject.toml file
      pyproject = builtins.fromTOML (builtins.readFile ./pyproject.toml);
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
          };
        };
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




          meta = with lib; {
            description = "OpenNURBS libraries allow anyone to read and write the 3DM file format without the need for Rhino. ";
            license = licenses.mit;
            maintainers = with maintainers; [  ];
          };
        };
        xtensor-io = pkgs.stdenv.mkDerivation {

          stdenv = if pkgs.config.cudaSupport then pkgs.cudaPackages.backendStdenv else pkgs.stdenv;

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
          };
        };
        g2o = pkgs.g2o.overrideAttrs (old: {

          version = "pymem";

          patches = [
          (builtins.toFile "inline-patch" ''
            diff --git a/python/CMakeLists.txt b/python/CMakeLists.txt
            index 010e6551..4ddd0cb2 100644
            --- a/python/CMakeLists.txt
            +++ b/python/CMakeLists.txt
            @@ -1,13 +1,6 @@
            -# download pybind11
            -include(FetchContent)
            -FetchContent_Declare(
            -  pybind11
            -  GIT_REPOSITORY https://github.com/pybind/pybind11.git
            -  GIT_TAG        v2.13.6
            -)
            +find_package(pybind11)
             # For Windows: Prevent overriding the parent project's compiler/linker settings
             set(PYBIND11_FINDPYTHON ON)
            -FetchContent_MakeAvailable(pybind11)
            
             if (TARGET simulator_lib)
             set(PYSIM_SRC "simulator/py_simulator.cpp")
            '')
          ];

          src = pkgs.fetchFromGitHub {
            owner = "RainerKuemmerle";
            repo = "g2o";
            rev = "c203321596a38502cb3008a3883805cb91d3535a";
            sha256 = "sha256-oGOzQpU0BW0KDjUZPK0pYjknio2rC2dQoDVLWrIb+SI=";
            # sha256 = lib.fakeSha256;
          };

          nativeBuildInputs = (old.nativeBuildInputs or []) ++ (with pkgs; [
            git
          ]);

          buildInputs = (old.buildInputs or []) ++ (with pkgs; [
            python3Packages.pybind11
            nlohmann_json
          ]);

          cmakeFlags = (old.cmakeFlags or []) ++ [
            "-DG2O_BUILD_PYTHON=ON"
          ];

        });


        g2opy = pkgs.python3.pkgs.buildPythonPackage rec {

          stdenv = if pkgs.config.cudaSupport then pkgs.cudaPackages.backendStdenv else pkgs.stdenv;

          pname = "g2o";
          version = "0.0.1";
          # format = "other";


          dontUnpack = true;
          pyproject = false;

          propagatedBuildInputs = [
            python
          ];

          installPhase = ''
            echo "Installing the package"2

            mkdir -p "$out/${python.sitePackages}/g2o"
            export PYTHONPATH="$out/${python.sitePackages}:$PYTHONPATH"

            cp -r ${localPackages.g2o}/g2o/* $out/${python.sitePackages}/g2o
            # touch $out/${python.sitePackages}/g2o/__init__.py

            echo "Finished installing the package"
          '';
          
        };
        mast3r = pkgs.python3.pkgs.buildPythonPackage rec {

          pname = "mast3r";
          version = "e06b009";

          src = pkgs.fetchFromGitHub {
            owner = "naver";
            repo = "mast3r";
            fetchSubmodules = true;
            rev = "e06b0093ddacfd8267cdafe5387954a650af0d3b";
            # sha256 = lib.fakeSha256;
            sha256 = "sha256-xkLHkaQ3YrYETKB16EoUiiz51A9IaAXTihA1QVVg7T8=";
          };

          patches = [
            (builtins.toFile "inline-patch" ''
              diff --git a/pyproject.toml b/pyproject.toml
              new file mode 100644
              index 0000000..49f1eb5
              --- /dev/null
              +++ b/pyproject.toml
              @@ -0,0 +1,34 @@
              +[build-system]
              +requires = ["setuptools>=42", "wheel"]
              +build-backend = "setuptools.build_meta"
              +
              +[project]
              +name = "mast3r"
              +version = "0.1.0"  # Update this based on the actual version
              +description = "MAST3R: Multimodal reasoning for situational awareness"
              +#authors = [
              +#    {name = "NAVER AI Lab", email = "your-email@domain.com"}  # Update the email address
              +#]
              +license = {file = "LICENSE"}
              +readme = "README.md"
              +requires-python = ">=3.8"  # Adjust Python version based on the project requirements
              +keywords = ["multimodal", "reasoning", "AI", "situational-awareness"]
              +classifiers = [
              +    "Programming Language :: Python :: 3",
              +    "License :: OSI Approved :: Apache Software License",  # Update if a different license applies
              +    "Operating System :: OS Independent",
              +]
              +
              +[project.urls]
              +Homepage = "https://github.com/naver/mast3r"
              +Source = "https://github.com/naver/mast3r"
              +Issues = "https://github.com/naver/mast3r/issues"
              +
              +[tool.setuptools]
              +packages = ["mast3r", "dust3r"]
              +
              +[tool.setuptools.package-dir]
              +# Define the root directory for these packages
              +"mast3r" = "."
              +"dust3r" = "."
              +
              +#[project.dependencies]
              +# List dependencies here. Add them if they are mentioned in the project's requirements.
              +#torch = ">=1.10.0"
              +#numpy = ">=1.21.0"
              +#opencv-python = ">=4.5.0"
              +#scipy = ">=1.7.0"
              +# Add any other dependencies the project requires.
            '')
          ];

          format = "pyproject";

          buildInputs = with pkgs; [
            python3Packages.setuptools
          ];


          # Metadata (optional)
          meta = with lib; {
            description = "Grounding Image Matching in 3D with MASt3R ";
            license = licenses.cc-by-nc-sa-40;
            homepage = "https://github.com/naver/mast3r";
          };


        };
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


          meta = with lib; {
            description = "";
            license = licenses.bsd3;
            maintainers = with maintainers; [  ];
          };
        };
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
          };
        };

        editablePkg = default.overrideAttrs (oldAttrs: {
          nativeBuildInputs = oldAttrs.nativeBuildInputs ++ [
            (python.pkgs.mkPythonEditablePackage {
              pname = pyproject.project.name;
              inherit (pyproject.project) scripts version;
              root = "$PWD";
            })
          ];
        });

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
              localPackages.xtensor-io
              highfive
              xtensor
              
              embree
              eigen
              cgal
              pcl
              fmt
              mpfr
              mpi
              opencv
              gurobi
              scip
              tbb

              cudatoolkit
              nvidia-optical-flow-sdk
              cudaPackages.cuda_cudart
              cudaPackages.cuda_cccl # <thrust/*>
              cudaPackages.libnpp # npp.h
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
              "-DHighFive_DIR=${pkgs.highfive}/share/HighFive/CMake"
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
        argEditable = project.renderers.mkPythonEditablePackage { inherit python; };

        myPython = python.override {
          packageOverrides = final: prev: {
            ReUseX = final.mkPythonEditablePackage argEditable;
          };
        };

        pythonEnv = myPython.withPackages (ps: with ps; [ ReUseX ]);

        ReUseXPython_dependanceis = python.withPackages arg;
        localPackages = self.packages.${system};
      in
       pkgs.mkShellNoCC {
        packages = with pkgs; [
          pythonEnv
          ReUseXPython_dependanceis
          localPackages.mast3r
          localPackages.g2opy
          python3Packages.torch
        ];

        # inputsFrom = [ editablePkg ];

        shellHook = ''
          echo "Entering dev shell"
          export VIRTUAL_ENV_PROMPT="ReUseX Environment"
        '';
      }; # end of devShells

    }); # end of outputs
}
