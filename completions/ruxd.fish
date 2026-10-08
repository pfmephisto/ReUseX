# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

# Fish shell completions for ruxd (hand-written).

complete -c ruxd -f

set -l ruxd_admin_cmds create-user set-password list-users disable-user create-token list-tokens revoke-token register-case

# `ruxd admin …`: the multi-user server's users and cases (needs --pg-url)
complete -c ruxd -n "not __fish_seen_subcommand_from admin" -a admin -d "Manage the server's users and cases"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a create-user -d "Create a user (the first: with --admin)"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a set-password -d "Set a user's password (prompted)"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a list-users -d "List every user"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a disable-user -d "Disable (or --enable) a user"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a create-token -d "Create an API token, printed once"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a list-tokens -d "List API tokens"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a revoke-token -d "Revoke an API token by id"
complete -c ruxd -n "__fish_seen_subcommand_from admin; and not __fish_seen_subcommand_from $ruxd_admin_cmds" -a register-case -d "Serve an existing .rux as a case"
complete -c ruxd -n "__fish_seen_subcommand_from $ruxd_admin_cmds" -l email -r -d "The user's email"
complete -c ruxd -n "__fish_seen_subcommand_from create-user create-token register-case" -l name -r -d "Display, token or case name"
complete -c ruxd -n "__fish_seen_subcommand_from create-user" -l admin -d "An administrator"
complete -c ruxd -n "__fish_seen_subcommand_from disable-user" -l enable -d "Re-enable instead"
complete -c ruxd -n "__fish_seen_subcommand_from create-token" -l case -r -d "Limit the token to this case id"
complete -c ruxd -n "__fish_seen_subcommand_from create-token" -l expires-days -r -d "Days until it expires (0 = never; default 90)"
complete -c ruxd -n "__fish_seen_subcommand_from revoke-token" -l id -r -d "The token's id (list-tokens)"
complete -c ruxd -n "__fish_seen_subcommand_from register-case" -l path -r -F -d "The project file"
complete -c ruxd -n "__fish_seen_subcommand_from register-case" -l owner -r -d "Email of its owner"

complete -c ruxd -s h -l help -d "Print this help message and exit"
complete -c ruxd -s v -l verbose -d "Increase verbosity, use -vv & -vvv for more details"
complete -c ruxd -s p -l port -r -d "Port to listen on (local mode default: 8420)"
complete -c ruxd -s t -l threads -r -d "Number of worker threads (0 = auto)"
complete -c ruxd -l auth-token -r -d "Superuser token (server); access token (--local, required beyond loopback)"

# The web GUI, both modes; --local serves one person with no login (formerly `rux gui`)
complete -c ruxd -l local -r -F -d "Serve the web GUI for a .rux file, or every .rux in a directory"
complete -c ruxd -l data-dir -r -a "(__fish_complete_directories)" -d "Where case files are stored (required in server mode)"
complete -c ruxd -l trusted-proxy -r -d "Reverse proxy whose X-Forwarded-For is honoured (CIDR)"
complete -c ruxd -l audit-retention-days -r -d "Days the audit log is kept (0 = for ever)"
complete -c ruxd -l auth-token-file -r -F -d "Read --auth-token from a file"
complete -c ruxd -l pg-url-file -r -F -d "Read --pg-url from a file"
complete -c ruxd -l cookie-secure -r -a "auto always never" -d "Mark the session cookie Secure (server mode)"
complete -c ruxd -l job-workers -r -d "Pipeline jobs that may run at once (default 1; one per case)"
complete -c ruxd -l max-open-cases -r -d "Most cases kept open at once (default 16)"
complete -c ruxd -l case-idle-minutes -r -d "Close a case unused this long (default 10)"
complete -c ruxd -l max-upload-mb -r -d "Largest .rux upload accepted, in MiB"
complete -c ruxd -l bind -r -d "Interface to bind (default: 127.0.0.1)"
complete -c ruxd -l allow-origin -r -d "Additional allowed browser origin (repeatable)"
complete -c ruxd -l assets -r -a "(__fish_complete_directories)" -d "Directory holding the frontend bundle"
complete -c ruxd -l open-browser -d "Open the system browser once listening"
complete -c ruxd -l segment-cuda -d "Use CUDA/TensorRT for the segment endpoints (default)"
complete -c ruxd -l no-segment-cuda -d "Run segment inference on the CPU (ONNX)"
complete -c ruxd -l sam3-model -r -a "(__fish_complete_directories)" -d "Explicit SAM3 model directory"
complete -c ruxd -l models-dir -r -a "(__fish_complete_directories)" -d "Base directory for managed models"
complete -c ruxd -l sam3-manifest-url -r -d "SAM3 ONNX bundle release manifest URL"

# Server mode backends
complete -c ruxd -l pg-url -r -d "PostgreSQL connection string"
complete -c ruxd -l pg-pool-size -r -d "PostgreSQL connection pool size (0 = per worker thread)"
complete -c ruxd -l pg-acquire-timeout-ms -r -d "Wait for a free PostgreSQL connection (ms)"
complete -c ruxd -l redis-url -r -d "Reserved, unused: Redis URI (tcp://host:port)"
complete -c ruxd -l s3-endpoint -r -d "Reserved, unused: S3 endpoint URL (empty = real AWS)"
complete -c ruxd -l s3-region -r -d "Reserved, unused: S3 region"
complete -c ruxd -l s3-bucket -r -d "Reserved, unused: S3 bucket name"
complete -c ruxd -l s3-access-key -r -d "Reserved, unused: S3 access key id"
complete -c ruxd -l s3-secret-key -r -d "Reserved, unused: S3 secret access key"
complete -c ruxd -l s3-path-style -d "Reserved, unused: Path-style S3 addressing (MinIO/Ceph)"
complete -c ruxd -l s3-virtual-style -d "Reserved, unused: Virtual-hosted S3 addressing"
