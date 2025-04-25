{
  description = "ReUseX";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    pyproject-nix = {
      url = "github:nix-community/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";

    nixvim = {
      url = "github:nix-community/nixvim";
    };
    #alejandra = {
    #    url = "github:kamadorueda/alejandra/3.1.0";
    #  inputs.nixpkgs.follows = "nixpkgs";
    #};
  }; # end of inputs

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    pyproject-nix,
    nixvim,
  }:
    flake-utils.lib.eachSystem ["x86_64-linux"] (system:
      # flake-utils.lib.eachDefaultSystem (system:
      let
        inherit (nixpkgs) lib;

        # Import nixpkgs with custom configurations and overlays
        pkgs = import nixpkgs {
          inherit system;

          # Set systm comfigurations such as CUDA support and unfree packages
          config = {
            cudaSupport = true;
            hardware.nvidia.open = false;
            allowUnfreePredicate = pkg:
              builtins.elem (lib.getName pkg) [
                # Python model
                "mast3r"

                # Gurobi Solver
                "gurobi"
                "gurobipy"

                # CUDA Support in PCL an OpenCV
                "cuda_cudart"
                "cuda_nvcc"
                "cuda_nvml_dev"
                "cuda_cccl"
                "libcublas"
                "libnpp"
                "libcufft"
                "cuda-merged"
                "cuda_cuobjdump"
                "cuda_gdb"
                "cuda_nvdisasm"
                "cuda_nvprune"
                "cuda_cupti"
                "cuda_cuxxfilt"
                "cuda_nvrtc"
                "cuda_nvtx"
                "cuda_profiler_api"
                "cuda_sanitizer_api"
                "libcurand"
                "libcusolver"
                "libnvjitlink"
                "libcusparse"
                "cudnn"
                "nvidia-x11"
                # "cudatoolkit"
                "triton"
                "torch"
              ];
          };

          # Set overlays and custom fixes for broken packages
          overlays = [
            (final: prev: {
              # Fix the CUDA environment for the suiteSparse package
              suitesparse = prev.suitesparse.override (old: {
                stdenv =
                  if pkgs.config.cudaSupport
                  then pkgs.cudaPackages.backendStdenv
                  else pkgs.stdenv;
              });
            })
            # (final: prev: {
            #   # Change PCL to the main branch
            #   pcl = prev.pcl.overrideAttrs (old: {
            #     pname = "pcl";
            #     # version = "1.14.1";
            #     version = "1.15.0";
            #     src = pkgs.fetchFromGitHub {
            #       owner = "PointCloudLibrary";
            #       repo = "pcl";
            #       rev = "pcl-1.15.0";
	    #       sha256 = "sha256-UCuQMWGwe+YxeGj0Y6m5IT58NW2lAWN5RqyZnvyFSr4=";
	    #       # sha256 = "sha256-OHzJwTtv+7CR+0UfyP0qB64lzFgUJG/0RWVreWo7KO8=";
            #       # sha256 = lib.fakeSha256;
            #     };
            #   });
            # })
            (
              final: prev: (prev.lib.packagesFromDirectoryRecursive {
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
                propagatedBuildInputs = with pkgs; [tbb_2022_0];
              });
            })
          ];
        };

        # Customize the python environment by adding required packages
        python = pkgs.python3.override {
          packageOverrides = final: prev: let
          in {
            rhino3dm = prev.pkgs.rhino3dm;
            specklepy = prev.pkgs.specklepy;
            g2o = prev.pkgs.g2opy;
            mast3r = prev.pkgs.mast3r;
	    spdlog = prev.pkgs.spdlog-python;
          };
        };

        # Load the pyproject.toml file
        pyproject = builtins.fromTOML (builtins.readFile ./pyproject.toml);
        project = pyproject-nix.lib.project.loadPyproject {
          projectRoot = ./.;
        };

        ReUseX_Package = let
          attrs = project.renderers.buildPythonPackage {inherit python;};
        in
          python.pkgs.buildPythonPackage (attrs
            // {
              postConfigure = ''
                echo "moving one level up (should be project root)"
                cd ..
              '';

              # Native dependencies
              # programs and libraries used at build-time
              nativeBuildInputs =
                (attrs.nativeBuildInputs or [])
                ++ (with pkgs; [
                  cmake
                  ninja
                  mpi
                  cudatoolkit
                ]);

              # Rutime dependencies
              # These are often programs and libraries used by the new derivation at run-time
              buildInputs =
                (attrs.buildInputs
                  or [
                  ])
                ++ (with pkgs; [
                  opennurbs
                  xtensor-io
                  scip-solver

                  highfive
                  xtensor

                  g2o

                  # pcl
                  boost
                  embree

                  eigen
                  cgal

                  fmt
		  spdlog
		  spdmon
                  
		  mpfr

                  opencv
                  tbb_2022_0

                  glfw

                  python3Packages.pybind11
                  python

                  imgui
                  glm

		  libGLU

                  gurobi
                ]);

              propagatedBuildInputs =
                (attrs.propagatedBuildInputs or [])
                ++ (with pkgs; [
                  pcl
                  hdf5
                ]);

              # Pass additional CMake flags
              cmakeFlags =
                (attrs.cmakeFlags or [])
                ++ [
                  "-DCGAL_DIR=${pkgs.cgal}/lib/cmake/CGAL"

                  "-DHighFive_DIR=${pkgs.highfive}/share/HighFive/CMake"
                  "-DOpenGL_GL_PREFERENCE=GLVND"
                ]
                ++ [
                  (lib.strings.cmakeBool "CUDA_FAST_MATH" true)
                  (lib.strings.cmakeFeature "CUDA_NVCC_FLAGS" "--expt-relaxed-constexpr")
                  (lib.strings.cmakeFeature "GUROBI_DIR" "${pkgs.gurobi}")
                ];
            }); # end of ReUseX
      in {
        formatter = nixpkgs.legacyPackages.${system}.alejandra;

        packages =
          {
            default = ReUseX_Package; # ReUseX
          }
          // # All custom packages
          (pkgs.lib.packagesFromDirectoryRecursive {
            callPackage = pkgs.lib.callPackageWith pkgs;
            directory = ./pkgs;
          }); # end of packages

        devShells = {



          default = let
            arg = project.renderers.withPackages {inherit python;};

	    arg_1 = project.renderers.mkPythonEditablePackage {
              inherit python;
              root = "$REPO_ROOT/python";
            };

            myPython = python.override {
              packageOverrides = final: prev: {
                ReUseX = final.mkPythonEditablePackage arg_1;
              };
            };

            editablePkg = ReUseX_Package.overrideAttrs (oldAttrs: {
              nativeBuildInputs =
                oldAttrs.nativeBuildInputs
                ++ [
                  (
                    python.pkgs.mkPythonEditablePackage arg_1
                    // {
                      # pname = pyproject.project.name;
                      # inherit (pyproject.project) scripts version;
                      # root = lib.mkOverride "$REPO_ROOT/python/ReUseX";
                    }
                  )
                ];
            });



          in
            pkgs.mkShell {
              inputsFrom = [
	        self.packages.${system}.default
	      ];

              packages = with pkgs;
                [
                  cmake
                  ninja
                  mpi
                  cudatoolkit
		  gdb
                ]
                ++ [
		  libnotify # Send noctification when build finishes
		]
                ++ [
                  # Python Environment
                  ((myPython.withPackages (ps:
                    with ps; [
                      ReUseX
                      torch
                      torchvision
                      safetensors
                      numba
                      opencv-python
                      matplotlib
                      tqdm
                      scipy
                      ipykernel
                    ]))

                  .override (args: {ignoreCollisions = true;}))
                ];	

              shellHook = ''
                echo "Entering dev shell"
                export VIRTUAL_ENV_PROMPT="ReUseX Environment"
                export REPO_ROOT=$(pwd)
                export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
              '';
            }; # end of default shell
        }; # end of devShells
      }); # end of outputs
}
