# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Devshell providing a local backend stack (PostgreSQL + Redis + MinIO) for
# testing the ruxd service worker, orchestrated by process-compose. Enter with
# `nix develop .#services` and run `services up`.
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
    echo "  services up         start postgres + redis + minio (process-compose)"
    echo "  services up -D      ... detached (process-compose down to stop)"
    echo "  services down       stop the stack"
    echo ""
    echo "  Point ruxd at the stack:"
    echo "    DATABASE_URL=postgresql://ruxd@127.0.0.1:5432/ruxd"
    echo "    REDIS_URL=tcp://127.0.0.1:6379"
    echo "    AWS_ENDPOINT_URL=http://127.0.0.1:9000"
    echo "    AWS_ACCESS_KEY_ID=ruxadmin AWS_SECRET_ACCESS_KEY=ruxadmin123"
    echo "    ruxd --s3-bucket reusex -v"
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

    shellHook = motd;
  }
