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
            # reuse = {
            #   enable = true;
            # };
            git-lfs-pre-push = {
              enable = true;
              name = "git-lfs pre-push";
              entry = "${pkgs.writeShellScript "git-lfs-pre-push" ''
                exec ${pkgs.git-lfs}/bin/git-lfs pre-push "$PRE_COMMIT_REMOTE_NAME" "$PRE_COMMIT_REMOTE_URL"
              ''}";
              stages = ["pre-push"];
              pass_filenames = false;
              always_run = true;
            };

            git-lfs-post-checkout = {
              enable = true;
              name = "git-lfs post-checkout";
              entry = "${pkgs.writeShellScript "git-lfs-post-checkout" ''
                exec ${pkgs.git-lfs}/bin/git-lfs post-checkout "$PRE_COMMIT_FROM_REF" "$PRE_COMMIT_TO_REF" "$PRE_COMMIT_CHECKOUT_TYPE"
              ''}";
              stages = ["post-checkout"];
              pass_filenames = false;
              always_run = true;
            };

            git-lfs-post-merge = {
              enable = true;
              name = "git-lfs post-merge";
              entry = "${pkgs.writeShellScript "git-lfs-post-merge" ''
                exec ${pkgs.git-lfs}/bin/git-lfs post-merge "$PRE_COMMIT_IS_SQUASH_MERGE"
              ''}";
              stages = ["post-merge"];
              pass_filenames = false;
              always_run = true;
            };

            git-lfs-post-commit = {
              enable = true;
              name = "git-lfs post-commit";
              entry = "${pkgs.git-lfs}/bin/git-lfs post-commit";
              stages = ["post-commit"];
              pass_filenames = false;
              always_run = true;
            };
          };
        };

        packages = let
          # Get all custom packages
          allPackages = pkgs.lib.packagesFromDirectoryRecursive {
            callPackage = pkgs.lib.callPackageWith pkgs;
            directory = ./pkgs;
          };
          # Filter out broken packages from exports
          nonBrokenPackages =
            lib.filterAttrs (
              name: pkg:
                !(pkg.meta.broken or false)
            )
            allPackages;
        in
          {
            default = pkgs.callPackage ./default.nix {}; # ReUseX
            rtabmap = pkgs.rtabmap;
          }
          # All custom packages (excluding broken ones)
          // nonBrokenPackages; # end of packages

        devShells =
          {
            default = import ./shell.nix {
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
