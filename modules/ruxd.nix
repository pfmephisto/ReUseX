# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# NixOS module for the ReUseX ruxd HTTP service worker. Exposed by the flake as
# `nixosModules.ruxd`; import it on a NixOS host and set `services.ruxd.enable`.
#
# Secrets (DATABASE_URL, RUXD_AUTH_TOKEN, AWS credentials) belong in
# `environmentFile`, NOT in the Nix store. Non-secret settings (port, threads,
# bind, data dir, ...) are plain options. `redisUrl` and `s3.*` are RESERVED:
# ruxd accepts them but does not use Redis or S3 yet.
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
      default = "";
      description = "Reserved, unused: ruxd does not use Redis yet. Redis URI.";
    };

    s3 = {
      endpoint = lib.mkOption {
        type = lib.types.str;
        default = "";
        description = "Reserved, unused: ruxd does not use S3 yet. S3 endpoint URL (empty = real AWS).";
      };
      region = lib.mkOption {
        type = lib.types.str;
        default = "us-east-1";
        description = "Reserved, unused. S3 region.";
      };
      bucket = lib.mkOption {
        type = lib.types.str;
        default = "";
        description = "Reserved, unused. S3 bucket name.";
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

    dataDir = lib.mkOption {
      type = lib.types.str;
      default = "/var/lib/ruxd";
      description = ''
        Where case files are stored, one directory per case (--data-dir).
        The default is the service's systemd StateDirectory. Users, sessions
        and the case catalogue live in Postgres (DATABASE_URL); create the
        first administrator with `ruxd admin create-user --email … --admin`.
      '';
    };

    bind = lib.mkOption {
      type = lib.types.str;
      default = "127.0.0.1";
      example = "0.0.0.0";
      description = ''
        Interface to bind (--bind). The default, loopback, suits a TLS
        reverse proxy on the same host (set allowOrigins to its public
        origin, and trustedProxies to its address). Bind beyond loopback only
        with TLS in front: the session cookie is then Secure.
      '';
    };

    allowOrigins = lib.mkOption {
      type = lib.types.listOf lib.types.str;
      default = [];
      example = ["https://ruxd.example.dk"];
      description = "Public origins the frontend is served under (--allow-origin).";
    };

    trustedProxies = lib.mkOption {
      type = lib.types.listOf lib.types.str;
      default = [];
      example = ["127.0.0.1"];
      description = ''
        Reverse proxies (CIDRs) whose X-Forwarded-For names the client
        (--trusted-proxy), for the login back-off.
      '';
    };

    cookieSecure = lib.mkOption {
      type = lib.types.enum ["auto" "always" "never"];
      default = "auto";
      description = "Whether the session cookie is Secure (--cookie-secure).";
    };

    auditRetentionDays = lib.mkOption {
      type = lib.types.ints.unsigned;
      default = 365;
      description = "Days the audit log is kept; 0 = for ever (--audit-retention-days).";
    };

    authTokenFile = lib.mkOption {
      type = lib.types.nullOr lib.types.path;
      default = null;
      description = ''
        File holding the superuser token (--auth-token-file, at least 32
        characters). Keeps it out of the process list and the Nix store.
      '';
    };

    openFirewall = lib.mkOption {
      type = lib.types.bool;
      default = false;
      description = ''
        Open the listen port in the firewall. Only useful with a non-loopback
        `bind`.
      '';
    };
  };

  config = lib.mkIf cfg.enable {
    systemd.services.ruxd = {
      description = "ReUseX ruxd web GUI server";
      wantedBy = ["multi-user.target"];
      wants = ["network-online.target"];
      after = ["network-online.target"];

      environment =
        {
          RUXD_PORT = toString cfg.port;
          RUXD_THREADS = toString cfg.threads;
          RUXD_DATA_DIR = cfg.dataDir;
          AWS_REGION = cfg.s3.region;
        }
        // lib.optionalAttrs (cfg.redisUrl != "") {REDIS_URL = cfg.redisUrl;}
        // lib.optionalAttrs (cfg.s3.endpoint != "") {AWS_ENDPOINT_URL = cfg.s3.endpoint;}
        // lib.optionalAttrs (cfg.s3.bucket != "") {RUXD_S3_BUCKET = cfg.s3.bucket;}
        // cfg.extraEnvironment;

      serviceConfig =
        {
          ExecStart = lib.escapeShellArgs (
            ["${cfg.package}/bin/ruxd" "--bind" cfg.bind "--cookie-secure" cfg.cookieSecure "--audit-retention-days" (toString cfg.auditRetentionDays)]
            ++ lib.concatMap (o: ["--allow-origin" o]) cfg.allowOrigins
            ++ lib.concatMap (p: ["--trusted-proxy" p]) cfg.trustedProxies
            # Through systemd's credential store: readable by the dynamic user
            # whatever the file's own permissions.
            ++ lib.optionals (cfg.authTokenFile != null) ["--auth-token-file" "%d/auth-token"]
          );
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
        // lib.optionalAttrs (cfg.dataDir == "/var/lib/ruxd") {
          StateDirectory = "ruxd";
        }
        // lib.optionalAttrs (cfg.authTokenFile != null) {
          LoadCredential = "auth-token:${toString cfg.authTokenFile}";
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
