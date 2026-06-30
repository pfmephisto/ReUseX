# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Devshell providing a local backend stack (PostgreSQL + Redis + SeaweedFS S3)
# for testing the ruxd service worker, orchestrated by process-compose. Enter
# with `nix develop .#services` and run `services up`.
#
# The backend connection env vars that ruxd reads (DATABASE_URL, REDIS_URL,
# AWS_*, RUXD_S3_BUCKET) are exported here, so once the stack is up you can run
# `ruxd` with no extra flags and it will connect to the local services.
{
  pkgs,
  self,
  system,
  ...
}: let
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │       ReUseX  •  backend services shell      │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  services up         start postgres + redis + seaweedfs (process-compose)"
    echo "  services up -D      ... detached (process-compose down to stop)"
    echo "  services down       stop the stack"
    echo ""
    echo "  ruxd is pre-configured via env vars for the local stack:"
    echo "    DATABASE_URL=$DATABASE_URL"
    echo "    REDIS_URL=$REDIS_URL"
    echo "    AWS_ENDPOINT_URL=$AWS_ENDPOINT_URL  bucket=$RUXD_S3_BUCKET"
    echo "  Just run:  ruxd -v        (no backend flags needed)"
    echo ""
    echo "  State lives under .dev/ (git-ignored); delete it to reset."
    echo ""
  '';

  services = pkgs.writeShellScriptBin "services" ''
    exec ${pkgs.process-compose}/bin/process-compose \
      -f "''${PWD}/process-compose.yaml" "$@"
  '';
in
  pkgs.mkShell {
    packages = with pkgs; [
      process-compose
      postgresql
      redis
      seaweedfs
      curl
      services
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    # Backend endpoints for ruxd — must match process-compose.yaml. mkShell
    # exports each of these as an environment variable in the shell.
    DATABASE_URL = "postgresql://ruxd@127.0.0.1:5432/ruxd";
    REDIS_URL = "tcp://127.0.0.1:6379";
    AWS_ENDPOINT_URL = "http://127.0.0.1:9000";
    AWS_ACCESS_KEY_ID = "ruxadmin";
    AWS_SECRET_ACCESS_KEY = "ruxadmin123";
    AWS_REGION = "us-east-1";
    RUXD_S3_BUCKET = "reusex";

    shellHook = motd;
  }
