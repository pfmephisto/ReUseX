# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  description = "ReUseX";

  # Our overlays rebuild OpenCV/RTABMap/GTSAM/HiGHS/OpenNURBS and we vendor
  # libtorch + tokenizers-cpp, none of which cache.nixos.org can serve. Those
  # paths live in reusex.cachix.org (public, read-only without a token) so
  # neither CI nor a fresh dev machine has to build them from source.
  # See docs/guides/ci-cache.md. Nix asks before honouring these the first
  # time; CI opts in up front via `accept-flake-config = true`.
  nixConfig = {
    extra-substituters = ["https://reusex.cachix.org"];
    extra-trusted-public-keys = [
      "reusex.cachix.org-1:0+y68O2+rBxXytqIepgqOSRsWa+5Ccpst9weS0sUBak="
    ];
  };

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
    pre-commit-hooks,
    ...
  }:
    (flake-utils.lib.eachSystem ["x86_64-linux"] (
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
            # Build CUDA code only for the workstation GPU (RTX 6000 Ada = sm_89)
            # instead of the default multi-arch list — much faster builds, smaller
            # closure. This is the only GPU this stack targets.
            cudaCapabilities = ["8.9"];
            # tensorrt (cuda12.9-tensorrt-10.16.1.11, updated by the nixpkgs bump)
            # carries no known vulnerabilities, so no permittedInsecurePackages
            # entry is needed anymore.
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
            # Nix static analysis (anti-patterns + dead code). statix runs
            # repo-wide, so skip .direnv (cached flake-input sources).
            statix = {
              enable = true;
              settings.ignore = [".direnv"];
            };
            deadnix.enable = true;
            # C++/CUDA formatting per .clang-format (docs/STANDARDS.md §9).
            clang-format = {
              enable = true;
              types_or = ["c++" "c" "cuda"];
            };
            reuse = {
              enable = true;
            };
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

        # Hermetic build-and-test gate: builds the CPU variant with the unit
        # tests enabled and runs ctest inside the sandbox. The CPU variant is
        # used so the check needs no GPU (works in CI and on any machine).
        # Run with: nix build .#checks.x86_64-linux.tests  (or nix flake check)
        # For the fast incremental dev loop use scripts/check.sh instead.
        checks.tests = self.packages.${system}.cpu.overrideAttrs (old: {
          pname = old.pname + "-tests";
          doCheck = true;
          checkPhase = ''
            runHook preCheck
            # --parallel: the suite is dominated by per-process loader
            # overhead, so this is a near-linear speedup (#268). Safe since
            # #262 gave temp files pid-unique names.
            ctest --output-on-failure --parallel "''${NIX_BUILD_CORES:-1}"
            runHook postCheck
          '';
        });

        packages = let
          # Get all custom packages
          allPackages = pkgs.lib.packagesFromDirectoryRecursive {
            callPackage = pkgs.lib.callPackageWith pkgs;
            directory = ./pkgs;
          };
          # Filter out broken packages from exports
          nonBrokenPackages = lib.filterAttrs (_name: pkg: !(pkg.meta.broken or false)) allPackages;

          # Heavy custom packages that compile through the plain stdenv (i.e. are
          # NOT covered by the ccache'd cudaPackages.backendStdenv from
          # overlays/ccache.nix) get their compiler wrapped in ccache here.
          # Prebuilt (libtorch) and Rust (tokenizers-cpp) packages are excluded:
          # ccache does not help them.
          ccachedPackageNames = ["gtsam" "trtsam3" "opennurbs"];
          ccachedPackages =
            nonBrokenPackages
            // lib.genAttrs
            (lib.filter (n: nonBrokenPackages ? ${n}) ccachedPackageNames)
            (n: nonBrokenPackages.${n}.override {stdenv = pkgs.ccacheStdenv;});

          # ReUseX build variants. The default/CUDA build reuses the top-level
          # (CUDA-configured) nixpkgs; the cpu and rocm builds use a fresh
          # nixpkgs with the matching GPU config. cudaSupport drives the
          # WITH_CUDA CMake option, which gates the CUDA language, the TensorRT
          # backend (+ its .cu kernels) and the cuOpt solver.
          reusex = pkgs.callPackage ./default.nix {}; # CUDA (default)

          # Opt-in torch-free CUDA build (see withLibtorch in default.nix).
          reusexCudaNoTorch = pkgs.callPackage ./default.nix {withLibtorch = false;};

          mkReusex = {
            cudaSupport ? false,
            rocmSupport ? false,
            withLibtorch ? true,
          }: let
            variantPkgs = import nixpkgs {
              inherit system;
              config = {
                inherit cudaSupport rocmSupport;
                allowUnfree = true;
              };
              overlays = import ./overlays {inherit lib;};
            };
          in
            variantPkgs.callPackage ./default.nix {
              inherit cudaSupport withLibtorch;
              # The CUDA path already gets ccache via cudaPackages.backendStdenv
              # (overlays/ccache.nix); wrap the plain stdenv so the CPU/ROCm
              # ReUseX compiles are cached too.
              stdenv = variantPkgs.ccacheStdenv;
            };

          reusexCpu = mkReusex {};
          reusexRocm = mkReusex {rocmSupport = true;};
          reusexCpuNoTorch = mkReusex {withLibtorch = false;};

          # Shared OCI image builder: ruxd as PID 1 for a given ReUseX variant.
          mkImage = {
            package,
            tag,
            extraEnv ? [],
          }:
            pkgs.dockerTools.buildLayeredImage {
              name = "ruxd";
              inherit tag;
              contents = [
                package
                pkgs.cacert
              ];
              config = {
                Entrypoint = ["${package}/bin/ruxd"];
                ExposedPorts = {"8080/tcp" = {};};
                # Server mode: cases live on the /data volume; give it
                # DATABASE_URL (or DATABASE_URL_FILE) at `docker run`. ruxd
                # binds loopback by default, which a container cannot reach.
                Volumes = {"/data" = {};};
                Env =
                  [
                    "RUXD_PORT=8080"
                    "RUXD_BIND=0.0.0.0"
                    "RUXD_DATA_DIR=/data"
                    "SSL_CERT_FILE=/etc/ssl/certs/ca-bundle.crt"
                  ]
                  ++ extraEnv;
              };
            };
        in
          {
            # ReUseX build variants (each provides bin/ruxd and bin/rux).
            default = reusex; # CUDA / NVIDIA GPU
            cuda = reusex;
            cpu = reusexCpu;
            rocm = reusexRocm;
            # Opt-in variants without libtorch: no LibTorch ML backend (YOLO
            # .pt) and no `rux create gsplat`. TensorRT/ONNX (CUDA) or ONNX
            # (CPU) inference is unaffected.
            cuda-notorch = reusexCudaNoTorch;
            cpu-notorch = reusexCpuNoTorch;

            inherit (pkgs) rtabmap;

            # OCI image running ruxd as PID 1. The default image is the CUDA
            # build (multi-GB): run with the nvidia container runtime on a GPU
            # host; libcuda comes from the host driver, never bundled.
            #   nix build .#ruxd-container && docker load < result
            ruxd-container = mkImage {
              package = reusex;
              tag = "latest";
              # Consumed by the nvidia container runtime to inject the host GPU
              # + driver (libcuda) at runtime.
              extraEnv = [
                "NVIDIA_VISIBLE_DEVICES=all"
                "NVIDIA_DRIVER_CAPABILITIES=compute,utility"
              ];
            };

            # CPU-only image — no CUDA in the closure.
            ruxd-container-cpu = mkImage {
              package = reusexCpu;
              tag = "cpu";
            };
          }
          # All custom packages (excluding broken ones), heavy ones ccache-wrapped
          // ccachedPackages; # end of packages

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
    ))
    # System-independent outputs (merged with the per-system set above).
    // {
      # NixOS module for deploying ruxd as a systemd service (e.g. using a
      # local NixOS host as an on-demand worker). Import and set
      # `services.ruxd.enable = true`.
      nixosModules.ruxd = import ./modules/ruxd.nix {inherit self;};
      nixosModules.default = self.nixosModules.ruxd;
    }; # end of outputs
}
