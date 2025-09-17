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

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    pyproject-nix,
  }:
    flake-utils.lib.eachSystem ["x86_64-linux"] (
      system:
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
            allowUnfree = true;
          };

          # Set overlays and custom fixes for broken packages
          overlays = import ./overlays {inherit lib;};
        };

        # Customize the python environment by adding required packages
        python = pkgs.python3.override {
          packageOverrides = final: prev: let
          in {
            specklepy = prev.pkgs.specklepy;
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
          python.pkgs.buildPythonPackage (
            attrs
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
                  wrapGAppsHook
                ]);

              # Rutime dependencies
              # These are often programs and libraries used by the new derivation at run-time
              buildInputs =
                (
                  attrs.buildInputs or [
                  ]
                )
                ++ (with pkgs; [
                  opennurbs
                  scip-solver

                  # pcl
                  boost
                  embree

                  eigen
                  cgal

                  rtabmap
                  # libsForQt6.qtbase
                  librealsense
                  octomap
                  # qt6.full
                  # qtcreator
                  # qt6.qtdeclarative

                  fmt
                  spdlog
                  spdmon

                  mpfr

                  opencv
                  tbb_2022

                  glfw

                  python3Packages.pybind11
                  python

                  imgui
                  glm

                  libGLU

                  cli11

                  suitesparse-graphblas
                  suitesparse
                  LAGraph
                ]);

              propagatedBuildInputs =
                (attrs.propagatedBuildInputs or [])
                ++ (with pkgs; [
                  # vtkWithQt5
                  #(pcl.override {vtk = pkgs.vtkWithQt5;})
                ]);

              # Pass additional CMake flags
              cmakeFlags =
                (attrs.cmakeFlags or [])
                ++ [
                  "-DCGAL_DIR=${pkgs.cgal}/lib/cmake/CGAL"
                  "-DOpenGL_GL_PREFERENCE=GLVND"
                ]
                ++ [
                  (lib.strings.cmakeBool "CUDA_FAST_MATH" true)
                  (lib.strings.cmakeFeature "CUDA_NVCC_FLAGS" "--expt-relaxed-constexpr")
                ];

              dontWrapQtApps = true;
            }
          ); # end of ReUseX
      in {
        formatter = nixpkgs.legacyPackages.${system}.alejandra;

        packages =
          {
            default = ReUseX_Package; # ReUseX
            rtabmap = pkgs.rtabmap;
          }
          # All custom packages
          // (pkgs.lib.packagesFromDirectoryRecursive {
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

              buildInputs = with pkgs; [
                gtk4
                pkg-config
              ];

              packages = with pkgs;
                [
                  cmake
                  ninja
                  mpi
                  cudatoolkit
                  gdb

                  valgrind
                  kdePackages.kcachegrind
                ]
                ++ [
                  libnotify # Send noctification when build finishes
                  sqlite

                  gtk2
                  gtk3
                  glib
                  ffmpeg
                  libva
                  pkg-config
                ]
                ++ [
                  # Python Environment
                  (
                    (myPython.withPackages (
                      ps:
                        with ps; [
                          ReUseX
                          torch
                          torchvision
                          safetensors
                          opencv-python
                          matplotlib
                          tqdm
                          scipy
                          ipykernel
                        ]
                    )).override
                    (args: {
                      ignoreCollisions = true;
                    })
                  )
                ];

              shellHook = ''
                echo "Entering dev shell"
                export VIRTUAL_ENV_PROMPT="ReUseX Environment"
                export QT_STYLE_OVERRIDE="fusion"
                export REPO_ROOT=$(pwd)
                export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
                ./tmux_session
              '';
            }; # end of default shell
        }; # end of devShells
      }
    ); # end of outputs
}
