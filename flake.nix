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
            default = pkgs.ReUseX; # ReUseX
            rtabmap = pkgs.rtabmap;
          }
          # All custom packages
          // (pkgs.lib.packagesFromDirectoryRecursive {
            callPackage = pkgs.lib.callPackageWith pkgs;
            directory = ./pkgs;
          }); # end of packages

        devShells = {
          default = pkgs.mkShell {
            inputsFrom = [self.packages.${system}.default];
            buildInputs = self.checks.${system}.pre-commit-check.enabledPackages;

            packages = with pkgs; [
              gdb
              help2man # For generating man pages from --help output
              pandoc
              sphinx

              valgrind
              kdePackages.kcachegrind
              libnotify # Send noctification when build finishes
              sqlite
              ffmpeg
            ];

            shellHook =
              ''
                echo "Entering dev shell"
                export VIRTUAL_ENV_PROMPT="ReUseX Environment"
                export QT_STYLE_OVERRIDE="fusion"
                export REPO_ROOT=$(pwd)
                export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
                # ./tmux_session
              ''
              + self.checks.${system}.pre-commit-check.shellHook;
          }; # end of default shell
        }; # end of devShells
      }
    ); # end of outputs
}
