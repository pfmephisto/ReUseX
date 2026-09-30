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
  };

  /// "fp16" or "fp32". The bf16-native vision-encoder MUST be "fp32": fp16
  /// silently corrupts its ViT-L trunk into all-background output.
  std::string precision = "fp16";

  /// trtexec / IBuilderConfig workspace memory pool, in MiB.
  int workspace_mb = 8192;

  /// Optimization-profile shapes, keyed by input tensor name. Empty for a fully
  /// static engine.
  std::map<std::string, ShapeProfile> shapes;

  [[nodiscard]] bool is_fp16() const { return precision == "fp16"; }
  [[nodiscard]] bool is_fp32() const { return precision == "fp32"; }
};

/// The full set of per-engine build recipes.
struct EngineBuildProfiles {
  int schema_version = 1;
  std::map<std::string, EngineProfile> engines;

  /// Parse ``engine-build.json`` from disk. Throws ``std::runtime_error`` on a
  /// missing file, malformed JSON, or an unsupported schema version.
  static EngineBuildProfiles from_file(const std::filesystem::path &path);

  /// Parse from an in-memory JSON string (used by tests).
  static EngineBuildProfiles from_string(const std::string &json_text);

  /// Look up one engine's recipe, or ``std::nullopt`` if absent.
  [[nodiscard]] std::optional<EngineProfile>
  find(const std::string &engine_name) const;
};

} // namespace reusex::vision::sam3
