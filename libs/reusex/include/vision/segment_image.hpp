// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Single-frame SAM3 segmentation — the per-image building block factored out
// of segment_panorama so the GUI endpoint and CLI can call it directly (#409).
//
// Backend dispatch: tries TensorRT first (most common), then ONNX Runtime.
// Detection is runtime, via the IData subtype returned by forward(). At least
// one of REUSEX_USE_TENSORRT / REUSEX_USE_ONNX_RUNTIME must be compiled into
// the vision library for any result to be produced.

#pragma once

#include "reusex/vision/IModel.hpp"
#include "reusex/vision/sam3_prompt.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace reusex::vision {

/// Segment a single BGR image with a SAM3 model.
///
/// @param model      A SAM3 model created via create_model_from_path().
///                   Must be TensorRT or ONNX; other types return an all-(-1)
///                   map.
/// @param image_bgr  Input BGR image (CV_8UC3, non-empty).
/// @param prompts    SAM3 prompts (text + optional boxes). Empty ⟹ model
/// default.
/// @param confidence Global detection confidence threshold [0,1].
///                   Per-prompt Sam3Prompt::confidence overrides when >= 0.
/// @return CV_32S label map the same size as @p image_bgr.
///         -1 = background; 0..N = class index matching prompt order.
///         Returns an empty Mat on empty input; all-(-1) on no detections.
cv::Mat segment_image(IModel &model, const cv::Mat &image_bgr,
                      const std::vector<Sam3Prompt> &prompts = {},
                      float confidence = 0.5f);

} // namespace reusex::vision
