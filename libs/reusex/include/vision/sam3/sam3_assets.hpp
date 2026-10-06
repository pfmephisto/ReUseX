// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/vision/sam3/EngineBuildProfiles.hpp"

#include <atomic>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::vision::sam3 {

/// Lifecycle state of the managed SAM3 model on this device.
enum class PrepState {
  absent,      ///< the (complete) ONNX bundle is not on disk yet
  not_built,   ///< ONNX present, device engines not built, no build running
  downloading, ///< fetching the portable ONNX bundle
  building,    ///< compiling GPU-specific TensorRT engines from ONNX
  ready,       ///< engines (or ONNX, for CPU) present and loadable
  error        ///< preparation failed (see message)
};

const char *to_string(PrepState state);

/// Thrown by prepare_sam3_model() when Sam3AssetOptions::cancel was raised.
class Sam3PrepCancelled : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

/// Best-effort progress report emitted during preparation.
struct PrepProgress {
  PrepState state = PrepState::absent;
  float fraction = 0.0f; ///< 0..1 within the current phase (best-effort)
  std::string message;   ///< human-readable detail
  /// Only from sam3_status() in state ``not_built``: when the engines are all
  /// there and the only gap is engines built from an older recipe
  /// (stale_engines()), their names — a one-time rebuild of those engines,
  /// no download. Empty for a first-time build.
  std::vector<std::string> update_engines;
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

  /// Optional cooperative-cancellation flag (not owned). Checked between
  /// download chunks, between files and between engine builds; when it reads
  /// true, prepare_sam3_model() throws Sam3PrepCancelled. A single in-flight
  /// TensorRT ``buildSerializedNetwork`` call cannot be interrupted, so
  /// cancellation during a build takes effect once that engine finishes.
  const std::atomic<bool> *cancel = nullptr;
};

/// The detector engines every SAM3 model needs (vision encoder, text encoder,
/// geometry encoder, decoder). Anything else in engine-build.json (the video
/// tracker graphs) is optional: built when its ONNX is present, skipped
/// otherwise.
const std::vector<std::string> &required_engines();

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

/// Files the ONNX directory still lacks before it counts as a complete bundle
/// (empty ⟹ complete). A downloaded bundle records its manifest as
/// ``bundle-manifest.json``; then every non-optional file it lists is
/// required. Every bundle — downloaded or a local ``make export`` — must hold
/// the required_engines() ONNX plus ``tokenizer.json``. Checks presence only
/// (sha256 is verified once, at download time, before the atomic publish).
std::vector<std::string>
missing_onnx_files(const std::filesystem::path &onnx_dir);

/// Files the engine directory still lacks before it is loadable (empty ⟹
/// ready): the required_engines(), an engine for every optional graph whose
/// ONNX is present, ``tokenizer.json``, and — when a tracker engine is present
/// and the ONNX dir carries one — ``tracker-meta.json``. A stale engine (see
/// stale_engines()) counts as missing, so it is rebuilt. Pure disk check; no
/// TensorRT needed.
std::vector<std::string>
missing_engine_files(const std::filesystem::path &onnx_dir,
                     const std::filesystem::path &engine_dir);

/// The engine-build recipe for an ONNX dir: its own ``engine-build.json`` when
/// present and at least as new (``recipe_version``) as the canonical recipe
/// embedded at build time, else that built-in recipe
/// (EngineBuildProfiles::builtin()). The newer-wins rule lets a recipe change
/// reach managed installs whose downloaded bundle ships an older recipe.
EngineBuildProfiles
load_engine_build_profiles(const std::filesystem::path &onnx_dir);

/// Built engines (names, without ``.engine``) whose recipe differs from the
/// one load_engine_build_profiles() now selects. An engine directory records
/// the recipe its engines were built from as ``engine-build.json``; a
/// directory built before that stamp existed is compared with the ONNX dir's
/// own ``engine-build.json`` (what the builder used then), and with no such
/// file is assumed current. Engines not built yet are missing, not stale.
std::vector<std::string> stale_engines(const std::filesystem::path &onnx_dir,
                                       const std::filesystem::path &engine_dir);

/// Engines (names, without ``.engine``) the engine directory's stamp records
/// as built text-only because their geometry-prompt profile failed with a
/// shape error (EngineBuildProfiles::fallback_engines). They load and serve
/// text prompts. Empty when there is no stamp or it cannot be read.
std::vector<std::string>
fallback_engines(const std::filesystem::path &engine_dir);

/// The fallback_engines() whose recipe profile is worth building again: the
/// ONNX it failed against has changed since (its sha256 differs from the
/// stamp's ``fallback_onnx_sha256``, or the stamp predates fingerprints).
/// A changed *recipe* needs no entry here: stale_engines() already rebuilds
/// such an engine from scratch. Everything else stays text-only for good —
/// the failure is a deterministic property of the ONNX graph against the
/// profile, so retrying it every session would only re-run a multi-minute
/// build that fails again. To force a retry anyway, delete the engine file
/// (``<engine_dir>/<name>.engine``): it is then missing and rebuilt.
std::vector<std::string>
fallback_engines_to_retry(const std::filesystem::path &onnx_dir,
                          const std::filesystem::path &engine_dir);

/// Non-blocking status probe: is the managed model already loadable, and in
/// what state? Does no downloading or building, and knows nothing about work
/// in flight — so it reports ``absent`` / ``not_built`` / ``ready`` (or
/// ``error`` for a CUDA request in a build without TensorRT), never
/// ``downloading`` / ``building``; a caller that runs preparation tracks those
/// itself.
PrepProgress sam3_status(const Sam3AssetOptions &opts);

/// Ensure the managed SAM3 model is ready and return a directory that
/// ``vision::create_model_from_path`` can load:
///   * ``use_cuda`` → downloads ONNX if missing, builds any missing engines,
///     and returns the engine cache dir;
///   * otherwise → downloads ONNX if missing and returns the ONNX dir.
///
/// The ONNX bundle is downloaded into a private staging directory and renamed
/// into place only after every manifest file is present and sha256-verified,
/// so an interrupted download is retried from scratch rather than mistaken for
/// a complete one. Downloads are serialized process-wide, and two processes
/// racing on the same models dir resolve by rename-if-absent. A pre-provided
/// ``$REUSEX_SAM3_ONNX_DIR`` is never downloaded into: if incomplete it is a
/// hard error naming the missing files.
///
/// Blocking and potentially slow (a full engine build is minutes). Reports
/// progress via ``cb``. Throws ``std::runtime_error`` on failure and
/// Sam3PrepCancelled when ``opts.cancel`` is raised.
std::filesystem::path prepare_sam3_model(const Sam3AssetOptions &opts,
                                         const ProgressCallback &cb = {});

} // namespace reusex::vision::sam3
