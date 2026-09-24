// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Injectable segmenter interface for POST /api/v1/frames/<id>/segment (#409).
//
// LAYERING: rux_gui_lib must not link reusex_vision (libtorch / TensorRT would
// follow, bloating the light test binary). This interface lets the HTTP handler
// in rux_gui_lib call segment_image indirectly: the rux app layer (rux_lib,
// which links the full reusex umbrella) registers a concrete implementation.
// The server returns HTTP 503 when no implementation is registered.

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

} // namespace rux::gui
