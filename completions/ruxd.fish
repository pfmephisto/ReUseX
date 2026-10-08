# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

# Fish shell completions for ruxd (hand-written; ruxd has no subcommands).

complete -c ruxd -f

complete -c ruxd -s h -l help -d "Print this help message and exit"
complete -c ruxd -s v -l verbose -d "Increase verbosity, use -vv & -vvv for more details"
complete -c ruxd -s p -l port -r -d "Port to listen on (local mode default: 8420)"
complete -c ruxd -s t -l threads -r -d "Number of worker threads (0 = auto)"
complete -c ruxd -l auth-token -r -d "Access token (required with --local beyond loopback)"

# Local mode: the web GUI for one project (formerly `rux gui`)
complete -c ruxd -l local -r -F -d "Serve the web GUI for a .rux file or a directory holding one"
complete -c ruxd -l bind -r -d "Interface to bind in local mode (default: 127.0.0.1)"
complete -c ruxd -l allow-origin -r -d "Additional allowed browser origin (repeatable)"
complete -c ruxd -l assets -r -a "(__fish_complete_directories)" -d "Directory holding the frontend bundle"
complete -c ruxd -l open-browser -d "Open the system browser once listening"
complete -c ruxd -l segment-cuda -d "Use CUDA/TensorRT for the segment endpoints (default)"
complete -c ruxd -l no-segment-cuda -d "Run segment inference on the CPU (ONNX)"
complete -c ruxd -l sam3-model -r -a "(__fish_complete_directories)" -d "Explicit SAM3 model directory"
complete -c ruxd -l models-dir -r -a "(__fish_complete_directories)" -d "Base directory for managed models"
complete -c ruxd -l sam3-manifest-url -r -d "SAM3 ONNX bundle release manifest URL"

# Service mode backends
complete -c ruxd -l pg-url -r -d "PostgreSQL connection string"
complete -c ruxd -l pg-pool-size -r -d "PostgreSQL connection pool size (0 = per worker thread)"
complete -c ruxd -l pg-acquire-timeout-ms -r -d "Wait for a free PostgreSQL connection (ms)"
complete -c ruxd -l redis-url -r -d "Redis URI (tcp://host:port)"
complete -c ruxd -l s3-endpoint -r -d "S3 endpoint URL (empty = real AWS)"
complete -c ruxd -l s3-region -r -d "S3 region"
complete -c ruxd -l s3-bucket -r -d "S3 bucket name"
complete -c ruxd -l s3-access-key -r -d "S3 access key id"
complete -c ruxd -l s3-secret-key -r -d "S3 secret access key"
complete -c ruxd -l s3-path-style -d "Path-style S3 addressing (MinIO/Ceph)"
complete -c ruxd -l s3-virtual-style -d "Virtual-hosted S3 addressing"
