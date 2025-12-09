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

        devShells =
          {
            default = import ./devshell.nix {
              inherit pkgs self system;
            };
          }
          // (
            let
              devshellFiles = builtins.readDir ./devshells;
              validShell = name: devshellFiles.${name} == "regular" && lib.hasSuffix ".nix" name;
              shellNames = builtins.filter validShell (builtins.attrNames devshellFiles);
              toShellAttr = name: {
                name = lib.removeSuffix ".nix" name;
                value = import (./devshells + "/${name}") {
                  inherit pkgs self system;
                };
              };
            in
              builtins.listToAttrs (builtins.map toShellAttr shellNames)
          ); # end of devShells
      }
    ); # end of outputs
}
