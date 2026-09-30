// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>
#include <functional>
#include <string>

namespace reusex::vision::sam3 {

/// Lifecycle state of the managed SAM3 model on this device.
enum class PrepState {
  absent,      ///< nothing on disk yet
  downloading, ///< fetching the portable ONNX bundle
  building,    ///< compiling GPU-specific TensorRT engines from ONNX
  ready,       ///< engines (or ONNX, for CPU) present and loadable
  error        ///< preparation failed (see message)
};

const char *to_string(PrepState state);

/// Best-effort progress report emitted during preparation.
struct PrepProgress {
  PrepState state = PrepState::absent;
  float fraction = 0.0f; ///< 0..1 within the current phase (best-effort)
  std::string message;   ///< human-readable detail
};

using ProgressCallback = std::function<void(const PrepProgress &)>;

/// Controls where the managed SAM3 model lives and how it is fetched.
struct Sam3AssetOptions {
  /// Base models directory. Empty → resolve from ``REUSEX_MODELS_DIR`` or the
  /// XDG cache default.
  std::filesystem::path models_dir;

  /// URL of the release *manifest* (JSON listing the ONNX bundle files +
  /// sha256). Empty → the built-in pinned default.
  std::string manifest_url;

  /// If true, missing ONNX may be downloaded. If false, a missing bundle is a
  /// hard error (offline / air-gapped operation).
  bool allow_download = true;

  /// Build engines for CUDA (true) or leave the ONNX in place for the CPU/ONNX
  /// runtime backend (false).
  bool use_cuda = true;
};

/// Resolve the base models directory:
///   explicit ``models_dir`` > ``$REUSEX_MODELS_DIR`` >
///   ``$XDG_CACHE_HOME``/reusex/models
/// (falling back to ``$HOME/.cache/reusex/models``).
std::filesystem::path
resolve_models_dir(const std::filesystem::path &explicit_dir = {});

/// The directory holding the portable ONNX + tokenizer.json + tracker-meta.json
/// + engine-build.json for the managed SAM3 model. Honors
/// ``$REUSEX_SAM3_ONNX_DIR`` (a pre-provided export) over the managed location.
std::filesystem::path sam3_onnx_dir(const Sam3AssetOptions &opts);

/// The device-specific engine cache directory for the managed SAM3 model
/// (keyed by GPU + TensorRT version). Only meaningful in a TensorRT build.
std::filesystem::path sam3_engine_dir(const Sam3AssetOptions &opts);

/// Non-blocking status probe: is the managed model already loadable, and in
/// what state? Does no downloading or building.
PrepProgress sam3_status(const Sam3AssetOptions &opts);

/// Ensure the managed SAM3 model is ready and return a directory that
/// ``vision::create_model_from_path`` can load:
///   * ``use_cuda`` → downloads ONNX if missing, builds any missing engines,
///     and returns the engine cache dir;
///   * otherwise → downloads ONNX if missing and returns the ONNX dir.
///
/// Blocking and potentially slow (a full engine build is minutes). Reports
/// progress via ``cb``. Throws ``std::runtime_error`` on failure.
std::filesystem::path prepare_sam3_model(const Sam3AssetOptions &opts,
                                         const ProgressCallback &cb = {});

} // namespace reusex::vision::sam3
