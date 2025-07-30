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
            allowUnfree = true;
          };

          # Set overlays and custom fixes for broken packages
          overlays = [
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
                propagatedBuildInputs = with pkgs; [tbb_2022];
              });
            })
            (final: prev: {
              opencv4 = prev.opencv4.override {
                enableGtk2 = true;
                # gtk2 = prev.pkgs.gtk2;
                # gtk2 = prev.pkgs.gtk2-x11;
                enableGtk3 = true;
                # gtk3 = prev.pkgs.gtk3;
                # gtk3 = prev.pkgs.gtk3-x11;
                enableVtk = true;
                enableTbb = true;
                enableCudnn = true;
                enablePython = true;
                enableUnfree = true;
              };
            })
            (findal: prev: {
              rtabmap =
                prev.rtabmap.overrideAttrs
                (old: {
                  #version = "0.22.0-dev-2e71831";
                  #patches = [];
                  #src = pkgs.fetchFromGitHub {
                  #  owner = "introlab";
                  #  repo = "rtabmap";
                  #  #rev = "0.22.0-jazzy";
                  #  # hash = "sha256-zlr9ydQnpIvef+x4LSK47Mwbz8PLkHUPTvKJxKaiqqI=";
                  #  rev = "2e71831324d5e9fe1d412241f794691687c6b2e0";
                  #  hash = "sha256-NShWR037KYL3Cnsa3m6St9QXrJaK5kpY9Qfoh1p0knA=";
                  #};

                  buildInputs =
                    (old.buildInputs or [])
                    ++ (with prev.pkgs; [
                      #python3Packages.pybind11
                      tbb
                      gtsam
                    ]);

                  cmakeFlags =
                    (prev.rtabmap.cmakeFlags or [])
                    ++ [
                      "-DWITH_TORCH=ON"
                      "-DWITH_GTSAM=ON"
                      #"-DWITH_PYTHON=ON"
                      #"-DWITH_PYTHON_THREADING=ON"
                      "-DWITH_LIBLAS=ON"
                      "-DWITH_CERES=ON"
                      "-DWITH_CVSBA=ON"
                      "-DWITH_CCCORELIB=ON"
                      "-DWITH_LOAM=ON"
                      "-DWITH_FLOAM=ON"
                      "-DWITH_DEPTHAI=ON"
                      "-DWITH_XVSDK=ON"
                      "-DWITH_GRIDMAP=ON"
                      "-DWITH_CPUTSDF=ON"
                      "-DWITH_OPENCHISEL=ON"
                      "-DWITH_ALICE_VISION=ON"
                      "-DWITH_FOVIS=ON"
                      "-DWITH_VISO2=ON"
                      "-DWITH_DVO=ON"
                      "-DWITH_ORB_SLAM=ON"
                      "-DWITH_OKVIS=ON"
                      "-DWITH_MSCKF_VIO=ON"
                      "-DWITH_VINS=ON"
                      "-DWITH_OPENVINS=ON"
                    ];
                });
            })
          ];
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
                  wrapGAppsHook
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

                  pcl
                  boost
                  embree

                  eigen
                  cgal

                  rtabmap
                  #libsForQt5.qtbase
                  librealsense
                  octomap

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

                  gurobi
                  cli11
                ]);

              propagatedBuildInputs =
                (attrs.propagatedBuildInputs or [])
                ++ (with pkgs; [
                  vtkWithQt5
                  #(pcl.override {vtk = pkgs.vtkWithQt5;})
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
            rtabmap = pkgs.rtabmap;
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

              buildInputs = with pkgs; [gtk4 pkg-config];

              packages = with pkgs;
                [
                  cmake
                  ninja
                  mpi
                  cudatoolkit
                  gdb

                  #pkgs.qt5.full
                  #pkgs.qtcreator
                  #pkgs.qt5.qtdeclarative

                  valgrind
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
                export QT_STYLE_OVERRIDE="fusion"
                export REPO_ROOT=$(pwd)
                export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
                ./tmux_session
              '';
            }; # end of default shell
        }; # end of devShells
      }); # end of outputs
}
