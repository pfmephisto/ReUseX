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
  /// Only in an engine directory's stamp: the engines that were built with
  /// text_only_fallback() instead of their profile in ``engines`` (their
  /// geometry-prompt profile failed with a shape error). Serialized as
  /// ``fallback_engines``, and only when non-empty.
  std::vector<std::string> fallback_engines;
  /// Only in a stamp, next to ``fallback_engines``: the sha256 of each
  /// fallback engine's ONNX at the time its profile failed. Together with the
  /// stamped profile in ``engines`` it makes the fallback permanent: the
  /// profile is retried only when the recipe or the ONNX changed
  /// (sam3::fallback_engines_to_retry). Serialized as ``fallback_onnx_sha256``
  /// (name -> hex digest), only when non-empty; a stamp without it (written
  /// before fingerprints existed) is retried once and then fingerprinted.
  std::map<std::string, std::string> fallback_onnx_sha256;

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

/// The box count the in-repo exporter traces the geometry encoder at
/// (``export_detector.GEOM_NUM_BOXES``, tied to
/// ``build_engines.GEOM_MAX_BOXES``). text_only_fallback() pins the encoder's
/// box axis to the profile max on the assumption that the two agree; a test
/// checks the built-in recipe against it.
inline constexpr int kGeometryTraceBoxes = 8;

/// Why an optimization profile cannot apply to a network input whose dims are
/// ``network_dims`` (-1 = dynamic): a rank mismatch, or a static dim the
/// profile's min or max does not equal. ``std::nullopt`` when it fits. Used to
/// tell a baked-shape export from other build failures before building.
std::optional<std::string>
profile_shape_conflict(const std::vector<long long> &network_dims,
                       const EngineProfile::ShapeProfile &profile);

/// Whether a failed TensorRT build's error messages point at the profile's
/// shapes (a dimension, reshape or profile error) rather than at resources:
/// any message about memory, allocation or disk makes it a resource failure.
/// No messages ⟹ false (unknown is not a shape error).
bool is_shape_build_error(const std::vector<std::string> &builder_errors);

} // namespace reusex::vision::sam3
