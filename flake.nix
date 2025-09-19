# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  description = "ReUseX";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    pyproject-nix = {
      url = "github:nix-community/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";

    pre-commit-hooks = {
      url = "github:cachix/git-hooks.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  }; # end of inputs

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    pyproject-nix,
    pre-commit-hooks,
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
                  qt6.wrapQtAppsHook
                  pkg-config
                  wrapGAppsHook3
                  cudatoolkit
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
                  boost

                  fmt
                  spdlog
                  #spdmon

                  pcl
                  embree
                  eigen
                  cgal

                  rtabmap
                  librealsense
                  octomap

                  # suitesparse
                  suitesparse-graphblas
                  LAGraph

                  mpfr

                  opencv
                  # glfw

                  # python3Packages.pybind11
                  # python

                  # imgui
                  # glm
                  # libGLU

                  cli11
                ]);

              propagatedBuildInputs =
                (attrs.propagatedBuildInputs or [])
                ++ (with pkgs; [
                  ]);

              # # Pass additional CMake flags
              # cmakeFlags =
              #   (attrs.cmakeFlags or [])
              #   ++ [
              #     "-DCGAL_DIR=${pkgs.cgal}/lib/cmake/CGAL"
              #     "-DOpenGL_GL_PREFERENCE=GLVND"
              #   ]
              #   ++ [
              #     (lib.strings.cmakeBool "CUDA_FAST_MATH" true)
              #     (lib.strings.cmakeFeature "CUDA_NVCC_FLAGS" "--expt-relaxed-constexpr")
              #   ];

              dontWrapQtApps = true;
            }
          ); # end of ReUseX
      in {
        formatter = pkgs.alejandra;

        checks.pre-commit-check = pre-commit-hooks.lib.${system}.run {
          src = ./.;
          default_stages = ["pre-commit"];
          hooks = {
            check-added-large-files.enable = true;
            check-case-conflicts.enable = true;
            check-executables-have-shebangs.enable = true;
            check-shebang-scripts-are-executable.enable = true;
            check-merge-conflicts.enable = true;
            alejandra.enable = true;
            reuse = {
              enable = true;
            };
          };
        };

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

              buildInputs = with pkgs;
                [
                  gtk4
                  pkg-config
                ]
                ++ self.checks.${system}.pre-commit-check.enabledPackages;

              packages = with pkgs;
                [
                  cmake
                  ninja
                  cudatoolkit
                  gdb

                  valgrind
                  kdePackages.kcachegrind
                ]
                ++ [
                  libnotify # Send noctification when build finishes
                  sqlite
                  ffmpeg
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

              shellHook =
                ''
                  echo "Entering dev shell"
                  export VIRTUAL_ENV_PROMPT="ReUseX Environment"
                  export QT_STYLE_OVERRIDE="fusion"
                  export REPO_ROOT=$(pwd)
                  export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
                  ./tmux_session
                ''
                + self.checks.${system}.pre-commit-check.shellHook;
            }; # end of default shell
        }; # end of devShells
      }
    ); # end of outputs
}
