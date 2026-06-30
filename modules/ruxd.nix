# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# NixOS module for the ReUseX ruxd HTTP service worker. Exposed by the flake as
# `nixosModules.ruxd`; import it on a NixOS host and set `services.ruxd.enable`.
#
# Secrets (DATABASE_URL, RUXD_AUTH_TOKEN, AWS credentials) belong in
# `environmentFile`, NOT in the Nix store. Non-secret settings (port, threads,
# redis URL, S3 endpoint/region/bucket) are plain options.
{self}: {
  config,
  lib,
  pkgs,
  ...
}: let
  cfg = config.services.ruxd;
in {
  options.services.ruxd = {
    enable = lib.mkEnableOption "the ReUseX ruxd HTTP service worker";

    package = lib.mkOption {
      type = lib.types.package;
      default = self.packages.${pkgs.stdenv.hostPlatform.system}.default;
      defaultText = lib.literalExpression "reusex flake default package";
      description = "Package providing bin/ruxd.";
    };

    port = lib.mkOption {
      type = lib.types.port;
      default = 8080;
      description = "TCP port ruxd listens on.";
    };

    threads = lib.mkOption {
      type = lib.types.int;
      default = 0;
      description = "Number of worker threads (0 = auto / hardware concurrency).";
    };

    redisUrl = lib.mkOption {
      type = lib.types.str;
      default = "tcp://127.0.0.1:6379";
      description = "Redis URI.";
    };

    s3 = {
      endpoint = lib.mkOption {
        type = lib.types.str;
        default = "";
        description = "S3 endpoint URL (empty = real AWS).";
      };
      region = lib.mkOption {
        type = lib.types.str;
        default = "us-east-1";
        description = "S3 region.";
      };
      bucket = lib.mkOption {
        type = lib.types.str;
        default = "";
        description = "S3 bucket name.";
      };
    };

    environmentFile = lib.mkOption {
      type = lib.types.nullOr lib.types.path;
      default = null;
      example = "/run/secrets/ruxd.env";
      description = ''
        Path to a systemd EnvironmentFile holding secrets, e.g.:
          DATABASE_URL=postgresql://user:pass@host:5432/db
          RUXD_AUTH_TOKEN=...
          AWS_ACCESS_KEY_ID=...
          AWS_SECRET_ACCESS_KEY=...
        Keeps secrets out of the world-readable Nix store.
      '';
    };

    extraEnvironment = lib.mkOption {
      type = lib.types.attrsOf lib.types.str;
      default = {};
      description = "Extra environment variables passed to the service.";
    };

    openFirewall = lib.mkOption {
      type = lib.types.bool;
      default = false;
      description = "Open the listen port in the firewall.";
    };
  };

  config = lib.mkIf cfg.enable {
    systemd.services.ruxd = {
      description = "ReUseX ruxd HTTP service worker";
      wantedBy = ["multi-user.target"];
      wants = ["network-online.target"];
      after = ["network-online.target"];

      environment =
        {
          RUXD_PORT = toString cfg.port;
          RUXD_THREADS = toString cfg.threads;
          REDIS_URL = cfg.redisUrl;
          AWS_REGION = cfg.s3.region;
        }
        // lib.optionalAttrs (cfg.s3.endpoint != "") {AWS_ENDPOINT_URL = cfg.s3.endpoint;}
        // lib.optionalAttrs (cfg.s3.bucket != "") {RUXD_S3_BUCKET = cfg.s3.bucket;}
        // cfg.extraEnvironment;

      serviceConfig =
        {
          ExecStart = "${cfg.package}/bin/ruxd";
          Restart = "on-failure";
          RestartSec = 2;
          DynamicUser = true;
          # Light hardening — kept compatible with CUDA device access
          # (/dev/nvidia* are world-accessible on NixOS).
          NoNewPrivileges = true;
          ProtectHome = true;
          PrivateTmp = true;
          ProtectControlGroups = true;
          ProtectKernelTunables = true;
        }
        // lib.optionalAttrs (cfg.environmentFile != null) {
          EnvironmentFile = cfg.environmentFile;
        };
    };

    networking.firewall = lib.mkIf cfg.openFirewall {
      allowedTCPPorts = [cfg.port];
    };
  };
}
