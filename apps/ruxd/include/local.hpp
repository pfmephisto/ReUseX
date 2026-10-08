// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `ruxd --local <file.rux | dir>`: the single-user web GUI.
//
// Serves the bundled frontend and the REST + WebSocket API
// (docs/gui/openapi.yaml) for one project, with no Postgres, Redis or S3.
// Formerly `rux gui`.

#include <api/Server.hpp>

#include <filesystem>
#include <string>

namespace ruxd {

struct LocalOptions {
  /// The file or directory given to --local (see api::resolve_local_project).
  std::filesystem::path target;

  /// Server settings. `project` is filled from `target`; the CLI binds the
  /// rest of the fields directly so its defaults are the library's
  /// (STANDARDS §4). `open_browser` defaults to false for a daemon.
  api::ServerOptions server;

  /// Explicit SAM3 model directory (a TRT engine dir or an ONNX dir). Empty =
  /// resolve and, on first use, prepare the managed model.
  std::string sam3_model_dir;
  /// Base directory for managed models (else $REUSEX_MODELS_DIR / XDG cache).
  std::string models_dir;
  /// Release manifest URL for the portable ONNX bundle (else built-in).
  std::string sam3_manifest_url;
};

/// Resolve the project, wire the injected pieces (SAM3 segmenters, managed
/// model provider, renderer, ICP, optimize) and serve until SIGINT/SIGTERM.
/// @return process exit code; startup errors are logged, not thrown.
int run_local(LocalOptions options);

} // namespace ruxd
