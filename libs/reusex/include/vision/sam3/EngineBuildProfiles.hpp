// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace reusex::vision::sam3 {

/// Per-engine TensorRT build recipe, deserialized from ``engine-build.json``.
///
/// ``engine-build.json`` is the SINGLE SOURCE OF TRUTH for how each SAM 3.1
/// ONNX graph is turned into a TensorRT engine. It is emitted by the Python
/// export pipeline (``python/reusex_sam3/build_engines.py``) and shipped in the
/// model bundle next to the ``*.onnx`` files. Both the Python ``trtexec``
/// driver and the native C++ ``EngineBuilder`` read it, so the two can never
/// drift.
///
/// The recipe here is already fully resolved — the fp32-vision-encoder rule and
/// its batch-1/workspace overrides (see ``build_engines.py`` FP32_ENGINES /
/// FP32_SHAPE_OVERRIDES / FP32_WORKSPACE_MB) are baked into ``precision``,
/// ``shapes`` and ``workspace_mb`` by the emitter, so the consumer applies the
/// values verbatim with no special cases.
///
/// This header is backend-agnostic (no NvInfer include), so it compiles in
/// CPU-only builds; the NvInfer-dependent consumer lives in
/// ``vision/tensor_rt/common/EngineBuilder.hpp``.
struct EngineProfile {
  /// Min / opt / max dimensions for one dynamic (or static) input tensor.
  struct ShapeProfile {
    std::vector<int> min;
    std::vector<int> opt;
    std::vector<int> max;

    bool operator==(const ShapeProfile &) const = default;
  };

  /// "fp16" or "fp32". The bf16-native vision-encoder MUST be "fp32": fp16
  /// silently corrupts its ViT-L trunk into all-background output.
  std::string precision = "fp16";

  /// trtexec / IBuilderConfig workspace memory pool, in MiB.
  int workspace_mb = 8192;

  /// Optimization-profile shapes, keyed by input tensor name. Empty for a fully
  /// static engine.
  std::map<std::string, ShapeProfile> shapes;

  bool operator==(const EngineProfile &) const = default;

  [[nodiscard]] bool is_fp16() const { return precision == "fp16"; }
  [[nodiscard]] bool is_fp32() const { return precision == "fp32"; }
};

/// The full set of per-engine build recipes.
struct EngineBuildProfiles {
  int schema_version = 1;
  /// Revision of the recipe's contents (``recipe_version``, absent ⟹ 1).
  /// Bumped by ``build_engines.py`` whenever a profile changes, so a newer
  /// built-in recipe can supersede the one an older model bundle shipped.
  int recipe_version = 1;
  std::map<std::string, EngineProfile> engines;

  /// Parse ``engine-build.json`` from disk. Throws ``std::runtime_error`` on a
  /// missing file, malformed JSON, or an unsupported schema version.
  static EngineBuildProfiles from_file(const std::filesystem::path &path);

  /// Parse from an in-memory JSON string (used by tests).
  static EngineBuildProfiles from_string(const std::string &json_text);

  /// The canonical recipe (``python/reusex_sam3/engine-build.json``) embedded
  /// at build time. The fallback when an ONNX directory does not ship its own
  /// ``engine-build.json`` (a bare ``make -C python export`` does not).
  static const EngineBuildProfiles &builtin();

  /// Serialize back to ``engine-build.json`` form (no ``_comment``). Used to
  /// stamp an engine directory with the recipe its engines were built from.
  [[nodiscard]] std::string to_json() const;

  /// Look up one engine's recipe, or ``std::nullopt`` if absent.
  [[nodiscard]] std::optional<EngineProfile>
  find(const std::string &engine_name) const;
};

/// The text-only fallback for an engine whose geometry-prompt profile cannot
/// be built: an export that bakes the prompt length or box count into its
/// graph (the TorchScript exporter can constant-fold the trace shape) builds
/// only at that shape. For ``decoder`` the prompt length is pinned to its
/// minimum (the 32 text tokens); for ``geometry-encoder`` the box count is
/// pinned to its maximum (the trace shape, 8). The result is recipe v1's
/// shape for that engine: text segmentation keeps working and TensorRTSam3's
/// capability check turns geometry prompts off. ``std::nullopt`` for every
/// other engine (it has no geometry dimension to give up).
std::optional<EngineProfile> text_only_fallback(const std::string &engine_name,
                                                const EngineProfile &profile);

} // namespace reusex::vision::sam3
