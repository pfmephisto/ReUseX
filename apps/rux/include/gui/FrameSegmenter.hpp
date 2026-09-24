// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Injectable segmenter interfaces for interactive SAM3 segmentation (#409,
// #448).
//
// LAYERING: rux_gui_lib must not link reusex_vision (libtorch / TensorRT would
// follow, bloating the light test binary). These interfaces let the HTTP
// handlers in rux_gui_lib call segment_image / segment_panorama indirectly:
// the rux app layer (rux_lib, which links the full reusex umbrella) registers
// concrete implementations. The server returns HTTP 503 when no implementation
// is registered.

#pragma once

#include <reusex/vision/sam3_prompt.hpp>

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace rux::gui {

/// Result of a single-frame segmentation.
struct SegmentFrameResult {
  /// CV_32S label map the same size as the input frame image.
  /// -1 = background; 0..N = class index matching prompt order.
  cv::Mat label_map;

  /// Class names indexed by label id (empty ⟹ model default list was used).
  std::vector<std::string> class_names;
};

/// Segmenter hook injected into the GUI server by the rux app layer.
///
/// Implementations may cache the loaded model across calls; the server calls
/// segment() on the Crow worker thread, so implementations must be thread-safe
/// or serialise internally.
class IFrameSegmenter {
    public:
  virtual ~IFrameSegmenter() = default;

  /// Run SAM3 inference on a single BGR frame image.
  ///
  /// @param image_bgr  Color frame image (CV_8UC3).
  /// @param prompts    SAM3 prompts. Empty ⟹ model default list.
  /// @param confidence Detection confidence threshold [0,1].
  /// @param model_path Filesystem path to the SAM3 model (TRT dir or .onnx).
  /// @return Segmentation result with a CV_32S label map.
  virtual SegmentFrameResult
  segment(const cv::Mat &image_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, const std::string &model_path) = 0;
};

// ---------------------------------------------------------------------------
// Panorama segmenter (#448)
// ---------------------------------------------------------------------------

/// Result of a 360-panorama segmentation.
struct SegmentPanoramaResult {
  /// CV_32S equirect label map the same size as the input panorama.
  /// -1 = background; 0..N = class index matching prompt order.
  cv::Mat label_map;

  /// Class names indexed by label id (empty ⟹ model default list was used).
  std::vector<std::string> class_names;
};

/// Segmenter hook for POST /api/v1/panoramas/<id>/segment (#448).
///
/// Injected by the rux app layer (which links reusex_vision) so the GUI
/// library stays free of libtorch/TensorRT. The server returns 503 when no
/// implementation is registered.
class IPanoramaSegmenter {
    public:
  virtual ~IPanoramaSegmenter() = default;

  /// Run SAM3 inference on a 360 equirectangular panorama image.
  ///
  /// @param equirect_bgr  Equirect panorama (CV_8UC3, BGR).
  /// @param prompts       SAM3 text prompts. Empty ⟹ model default list.
  /// @param confidence    Detection confidence threshold [0,1].
  /// @param n_yaw         Number of equator tiles around the sphere (default
  /// 8).
  /// @param fov_deg       Per-tile horizontal FOV in degrees (default 90.0).
  /// @param model_path    Filesystem path to the SAM3 model.
  /// @return Segmentation result with a CV_32S equirect label map.
  virtual SegmentPanoramaResult
  segment(const cv::Mat &equirect_bgr,
          const std::vector<reusex::vision::Sam3Prompt> &prompts,
          float confidence, int n_yaw, double fov_deg,
          const std::string &model_path) = 0;
};

} // namespace rux::gui
