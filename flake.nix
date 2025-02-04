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
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (system: # flake-utils.lib.eachDefaultSystem (system:
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
          "cudatoolkit"


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
        (final: prev: (prev.lib.packagesFromDirectoryRecursive {
            callPackage = prev.lib.callPackageWith final;
            directory = ./pkgs;
          })
         )
        (final: prev: {
          papilo = prev.papilo.overrideAttrs (old: {
            version = "2.4.0";
            src = pkgs.fetchFromGitHub {
              owner = "scipopt";
              repo = "papilo";
              rev = "v2.4.0";
              sha256 = "sha256-WMw9v57nuP6MHj9Ft4l5FxdIF5VUWCRm/909tbz7VD4=";
            };
            propagatedBuildInputs = with pkgs;[ tbb_2022_0 ];
            # outputs = [ "out" "bin" "lib" ];
          });
         })
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
      #         dependencies = lib.filterAt trs (name: _: ! lib.hasPrefix "nvidia" name) old.passthru.dependencies;
      #       };
      #     });        
      #   };
      # };

      cudatoolkit_joined = pkgs.symlinkJoin {
        name = "${pkgs.cudaPackages.cudatoolkit.name}-joined";
        paths = with pkgs.cudaPackages; [ 
          cudatoolkit.out 
          cudatoolkit.lib 
          nccl.dev 
          nccl.out 
          cudnn.out 
          cudnn.dev 
        ];
      };

      python = pkgs.python3.override {
        packageOverrides = final: prev: let 
        in{
          rhino3dm =  prev.pkgs.rhino3dm;
          specklepy = prev.pkgs.specklepy;
          g2o = prev.pkgs.g2opy;
          mast3r = prev.pkgs.mast3r;
        };
      };

      # localPackages = self.packages.${system};

      # Load the pyproject.toml file
      pyproject = builtins.fromTOML (builtins.readFile ./pyproject.toml);
      project = pyproject-nix.lib.project.loadPyproject {
        projectRoot = ./.;
      };


      ReUseX_Package = 
      let
          attrs = project.renderers.buildPythonPackage { inherit python;  };
      in
          python.pkgs.buildPythonPackage (attrs // {

            postConfigure = ''
              echo "moving one level up (should be project root)"
              cd ..
              ls
            '';

            # Native dependencies
            # programs and libraries used at build-time
            nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ ( with pkgs;[
              cmake
              ninja
              mpi
              cudatoolkit_joined
            ]);

            # configureFlags = (attrs.configureFlags or []) ++ [
            #   "--with-gurobi-incdir=${pkgs.gurobi}/include"
            #   "--with-gurobi-lib=-lgurobi${pkgs.gurobi.libSuffix}"
            # ];

            # Rutime dependencies
            # These are often programs and libraries used by the new derivation at run-time
            buildInputs = (attrs.buildInputs or [
            ]) ++ (with pkgs; [

              polyscope
              opennurbs
              xtensor-io
              scip-solver

              highfive
              xtensor

              g2o
              
              pcl
              boost
              embree

              eigen
              cgal

              fmt
              mpfr

              opencv
              tbb_2022_0

              glfw

              python3Packages.pybind11
              python

              imgui
              glm

              gurobi
            ]);


            # # Build dependencies
            # propagatedBuildInputs = (attrs.propagatedBuildInputs or [
            # ]) ++ ( with pkgs; [ 
            #   # specklepy
            #   # pkgs.tbb.dev
            # ]);

            # Pass additional CMake flags
            cmakeFlags = (attrs.cmakeFlags or []) ++ [
              "-DCGAL_DIR=${pkgs.cgal}/lib/cmake/CGAL"

              "-DHighFive_DIR=${pkgs.highfive}/share/HighFive/CMake"
              "-DOpenGL_GL_PREFERENCE=GLVND"
            ] ++ [
              (lib.strings.cmakeBool "CUDA_FAST_MATH" true)
              (lib.strings.cmakeFeature "CUDA_NVCC_FLAGS" "--expt-relaxed-constexpr")
              (lib.strings.cmakeFeature "GUROBI_DIR" "${pkgs.gurobi}")
            ];

          }); # end of ReUseX
    
    in {


      packages =
        # All custom packages
        (pkgs.lib.packagesFromDirectoryRecursive {
          callPackage = pkgs.lib.callPackageWith pkgs;
          directory = ./pkgs;
        }) // {


        # ReUseX
        default = ReUseX_Package;

        editablePkg = ReUseX_Package.overrideAttrs (oldAttrs: {
          nativeBuildInputs = oldAttrs.nativeBuildInputs ++ [
            (python.pkgs.mkPythonEditablePackage {
              pname = pyproject.project.name;
              inherit (pyproject.project) scripts version;
              root = "$PWD";
            })
          ];
        });

      }; # end of packages
      


      devShells.default =
      let
        arg = project.renderers.withPackages { inherit python; }; 
      in
      pkgs.mkShell{

        # buildInputs = with pkgs; [
        #   cmake
        #   ninja
        #   mpi
        #   cudatoolkit_joined
        #   python
        #   (python.withPackages arg)
          
        # ];

        inputsFrom = [ self.packages.${system}.default ];


        shellHook = ''
          echo "Entering dev shell"
          export VIRTUAL_ENV_PROMPT="ReUseX Environment"
        '';
      }; # end of devShells

      devShells.python =
      let
        arg = project.renderers.withPackages { inherit python; };
        argEditable = project.renderers.mkPythonEditablePackage { inherit python; };

        myPython = python.override {
          packageOverrides = final: prev: {
            ReUseX = final.mkPythonEditablePackage argEditable;
          };
        };

        pythonEnv = myPython.withPackages (ps: with ps; [ ReUseX ]);

        # ReUseXPython_dependanceis = python.withPackages arg;
      in
       pkgs.mkShellNoCC {
        packages = with pkgs; [
          ReUseXPython_dependanceis
          pythonEnv
          mast3r
          g2opy
          python3Packages.torch

          cmake
          ninja
          mpi
          cudatoolkit_joined
        ];

        inputsFrom = [ self.packages.${system}.editablePkg ];

        shellHook = ''
          echo "Entering dev shell"
          export VIRTUAL_ENV_PROMPT="ReUseX Environment"
        '';
      }; # end of devShells

    }); # end of outputs
}
