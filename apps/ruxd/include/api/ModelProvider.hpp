// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Injectable managed-SAM3-model provider (self-contained packaging, lazy
// provisioning).
//
// LAYERING: ruxd_api_lib must not link reusex_vision. This interface lets the
// HTTP handlers resolve an omitted `model_path` to a managed SAM3 model —
// downloading the portable ONNX on first use and building the GPU-specific
// TensorRT engines on-device — without the GUI library pulling in the vision /
// TensorRT closure. ruxd's main (ruxd_lib) registers the concrete
// implementation (which wraps reusex::vision::sam3::prepare_sam3_model).

#pragma once

#include <string>
#include <vector>

namespace ruxd::api {

/// Snapshot of the managed model's provisioning state.
struct ModelPrepStatus {
  /// One of: "absent", "not_built" (ONNX present, engines not built, nothing
  /// running), "downloading", "building", "ready", "error" (last attempt
  /// failed; ensure() retries after a backoff).
  std::string state = "absent";
  /// Best-effort progress within the current phase, 0..1.
  float progress = 0.0f;
  /// Human-readable detail.
  std::string message;
  /// Directory to load with the segmenter — set only when state == "ready".
  std::string model_path;
  /// In state "not_built": the engines that exist but were built from an
  /// older recipe, when rebuilding them is all that is needed (a one-time
  /// update, no download). Empty for a first-time build.
  std::vector<std::string> update_engines;
};

/// Resolves an omitted request `model_path` to a managed SAM3 model, preparing
/// it lazily in the background.
class IModelProvider {
    public:
  virtual ~IModelProvider() = default;

  /// Return the current status, kicking off background preparation if it has
  /// not started yet (or retrying a failed one). Non-blocking: never waits for
  /// a download or engine build.
  virtual ModelPrepStatus ensure(bool use_cuda) = 0;

  /// Pure status probe with no side effects (does not start preparation).
  virtual ModelPrepStatus status(bool use_cuda) = 0;
};

} // namespace ruxd::api
