// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Factories for the heavy pieces ruxd injects into the web API (api::Server).
//
// ruxd_api_lib links reusex_core and reusex_pipeline only, so the light test
// binary can cover it (#268). SAM3 inference (reusex_vision), the evidence
// renderer (reusex_visualize / VTK), managed-SAM3 provisioning and the
// `optimize` stage (reusex_slam / GTSAM) are built here in ruxd_lib, which
// links the `reusex` umbrella, and handed to the server through its
// set_*() hooks and ServerOptions. The ICP refine function is the same kind
// of piece; it has its own header (icp.hpp).

#include <api/FrameSegmenter.hpp>
#include <api/ModelProvider.hpp>
#include <api/ViewRenderer.hpp>

#include <reusex/pipeline/stages.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace ruxd {

/// SAM3 frame segmenter; loads and caches one model per (path, cuda), falls
/// back to CPU when a CUDA load fails, and serialises inference.
std::unique_ptr<api::IFrameSegmenter> make_frame_segmenter();

/// SAM3 panorama segmenter (perspective tiling); same caching rules.
std::unique_ptr<api::IPanoramaSegmenter> make_panorama_segmenter();

/// Headless VTK renderer for GET /api/v1/renders (one render at a time).
std::unique_ptr<api::IViewRenderer> make_view_renderer();

/// Managed SAM3 model: downloads the portable ONNX bundle and builds the
/// device's engines on first use. @p explicit_model, when set, short-circuits
/// that to a fixed model directory. Empty @p models_dir / @p manifest_url
/// select the defaults ($REUSEX_MODELS_DIR / XDG cache, built-in manifest).
std::unique_ptr<api::IModelProvider>
make_sam3_model_provider(std::filesystem::path models_dir,
                         std::string explicit_model, std::string manifest_url);

/// The default pipeline executor plus the `optimize` stage (#464):
/// reusex::pipeline::stage_executor_with_optimize(), shared with the Qt client.
reusex::pipeline::StageExecutor make_stage_executor();

} // namespace ruxd
