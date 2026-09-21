// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The `rux create attributes` pipeline stage (#373): for each MATERIAL passport
// in the project, resolve its linked instance(s), crop the best sensor view of
// their points, ask a vision-language model to describe the material, and store
// a free-text description + arbitrary key/value attributes in ProjectDB keyed
// by the material GUID (for MaterialPassport use).

#include <cstddef>
#include <filesystem>
#include <string>

namespace reusex::vision {

struct IVlmClient;

/// Options for the VLM attribute-extraction stage. CLI flags mirror these
/// defaults (STANDARDS §4).
struct DescribeConfig {
  /// OpenAI-compatible chat/completions base URL. Default is a local Ollama.
  /// The `/chat/completions` path is appended if not already present.
  std::string api_url = "http://localhost:11434/v1";
  /// Model name the endpoint routes to.
  std::string model = "qwen2.5vl";
  /// Optional bearer token (cloud providers). Empty = no Authorization header.
  std::string api_key;
  /// Name of the instance-label cloud used to resolve a material's points.
  /// Materials link to instances in THIS cloud (via `instance_materials`).
  std::string instances_cloud = "instances";
  /// Name of the XYZRGB cloud index-aligned with @c instances_cloud.
  std::string point_cloud = "cloud";
  /// Instruction sent alongside each crop. A default prompt requesting a
  /// description + attributes object is used when empty.
  std::string prompt;
  /// Skip materials that already have a stored annotation (resume runs).
  bool skip_existing = false;
  /// Pixels of padding around the projected instance bbox before cropping.
  int crop_padding = 8;
  /// Minimum projected in-bounds points for a frame to be a usable view.
  int min_view_points = 20;
  /// Max points to reproject per frame (subsample large instances for speed).
  std::size_t max_points_per_frame = 5000;
};

/// The default instruction: asks for a JSON object with a free-text
/// `description` and an arbitrary `attributes` map. Exposed so the CLI help can
/// show it and tests can pin it. The caller may override it to request any
/// keys.
std::string default_describe_prompt();

/// Run the attribute stage against the project at @p dbPath, constructing the
/// concrete OpenAI-compatible client from @p config.
/// @return 0 on success, non-zero on a fatal error. A per-material VLM failure
///         is logged and skipped, not fatal (STANDARDS §5).
int describe(const std::filesystem::path &dbPath, const DescribeConfig &config);

/// Testable core: run the stage with an injected client (no network). Used by
/// unit tests and by the network entry point above.
/// @return the number of materials for which an annotation was stored.
int describe_with_client(const std::filesystem::path &dbPath,
                         const DescribeConfig &config, IVlmClient &client);

} // namespace reusex::vision
