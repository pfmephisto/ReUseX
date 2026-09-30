// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include "gui/Server.hpp"

#include <CLI/CLI.hpp>
#include <memory>

/// Options for the `rux gui` subcommand.
///
/// Holds a rux::gui::ServerOptions rather than restating its fields, so the CLI
/// cannot drift from the library defaults (STANDARDS §4). Only genuinely
/// CLI-shaped state lives alongside it.
struct SubcommandGuiOptions {
  rux::gui::ServerOptions server;
  /// The flag is negative (`--no-browser`) while the option it controls is
  /// positive (`open_browser`), so it cannot simply bind to that field.
  bool no_browser = false;

  /// Managed-SAM3-model configuration (self-contained packaging). All optional:
  /// when omitted, the segment endpoints resolve the managed model from a
  /// well-known cache dir, downloading the ONNX and building engines on first
  /// use.
  ///
  /// Explicit SAM3 model directory (a TRT engine dir or an ONNX dir). Empty ⟹
  /// resolve the managed model.
  std::string sam3_model_dir;
  /// Base models directory override (else $REUSEX_MODELS_DIR / XDG cache).
  std::string models_dir;
  /// Release manifest URL for the portable ONNX bundle (else the built-in
  /// default). See docs/sam3.1-tensorrt.md.
  std::string sam3_manifest_url;
};

void setup_subcommand_gui(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_gui(SubcommandGuiOptions const &opt,
                       const RuxOptions &global_opt);
